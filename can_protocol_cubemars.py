"""
can_protocol_cubemars.py
========================
CubeMars AK-series motor protocol handler (V3 firmware).

Servo mode only — position-velocity loop (mode 6) intended for
use alongside the existing CANopen motors on can0.

CAN frame structure (servo mode)
---------------------------------
Extended frame, 29-bit arbitration ID:
    bits [28:8]  = Control Mode ID  (the command type)
    bits [7:0]   = Drive ID         (motor CAN ID set in RLink)

So:  arb_id = (mode_id << 8) | drive_id

Feedback frames from the motor use the same structure:
    bits [28:8]  = Function ID  (0x29 = real-time status)
    bits [7:0]   = Drive ID

Control mode IDs (servo mode)
------------------------------
    0   Duty cycle
    1   Current loop
    2   Current brake
    3   Speed loop
    4   Position loop
    6   Position-velocity loop   <-- what we use
    15  Motor disable
    16  Feedback message config

MIT (force control) mode ID = 8  — NOT used here.
"""

import struct
import time


# --------------------------------------------------------
# Control mode IDs
# --------------------------------------------------------
_MODE_DUTY          = 0
_MODE_CURRENT       = 1
_MODE_CURRENT_BRAKE = 2
_MODE_SPEED         = 3
_MODE_POSITION      = 4
_MODE_SET_ORIGIN    = 5
_MODE_POS_VEL       = 6
_MODE_DISABLE       = 15
_MODE_FEEDBACK_CFG  = 16

# Function IDs seen in feedback frames
_FUNC_SERVO_ENTER   = 0x2C   # sent once when motor enters servo mode
_FUNC_STATUS        = 0x29   # real-time status broadcast

# Feedback config value: bit 15 set = enable periodic status frames
_FEEDBACK_ENABLE    = 0x8000

# Default timeouts
_RECV_TIMEOUT       = 0.5    # seconds to wait for a single frame


class CubeMarsProtocol:
    """
    CubeMars AK-series motor protocol handler.

    Wraps a CanBus instance exactly like CanOpenProtocol does.
    All CubeMars-specific bit packing lives here.

    Parameters
    ----------
    bus : CanBus
        Shared CanBus instance (must be open before any calls).
    """

    def __init__(self, bus):
        self._bus = bus

    # --------------------------------------------------------
    # Internal helpers
    # --------------------------------------------------------

    def _make_arb_id(self, mode_id, drive_id):
        """Build a 29-bit extended arbitration ID."""
        return (mode_id << 8) | drive_id

    def _send(self, mode_id, drive_id, data=b''):
        """Send one extended CAN frame through the shared CanBus."""
        arb_id = self._make_arb_id(mode_id, drive_id)

        self._bus.send(
            arb_id=arb_id,
            data=bytes(data),
            is_extended_id=True,
        )

    def _recv_from(self, drive_id, func_id=None, timeout=_RECV_TIMEOUT):
        """
        Retrieve a buffered extended frame from a specific motor.
        Unrelated CAN frames remain available to other consumers.
        """

        def matches(msg):
            if not msg.is_extended_id:
                return False

            msg_drive_id = msg.arbitration_id & 0xFF
            msg_func_id = (msg.arbitration_id >> 8) & 0xFFFFFF

            if msg_drive_id != drive_id:
                return False

            if func_id is not None and msg_func_id != func_id:
                return False

            return True

        return self._bus.recv_matching(
            matches,
            timeout=timeout,
        )

    # --------------------------------------------------------
    # Feedback enable
    # --------------------------------------------------------

    def enable_feedback(self, drive_id):
        """
        Tell the motor to broadcast periodic status frames.

        Sends the feedback message configuration command (mode 16)
        with bit 15 set, which enables the 0x29 status broadcast
        at whatever CAN feedback rate is configured in the motor.

        This is the first thing to call after powering a motor.
        Safe to call with the motor stationary — sends no motion
        commands.

        Parameters
        ----------
        drive_id : int   Motor CAN ID (2 or 3 for shoulder/elbow).

        Returns
        -------
        True if a response frame was received, False on timeout.
        """
        # Data: 6 reserved zero bytes + 2-byte config value big-endian
        data = bytes(6) + struct.pack('>H', _FEEDBACK_ENABLE)
        self._send(_MODE_FEEDBACK_CFG, drive_id, data)

        # Motor responds with a 0x2C frame when it enters servo mode
        # or a 0x29 status frame shortly after
        msg = self._recv_from(drive_id, timeout=_RECV_TIMEOUT)
        return msg is not None

    # --------------------------------------------------------
    # Status read
    # --------------------------------------------------------

    def read_status(self, drive_id, timeout=_RECV_TIMEOUT):
        """
        Wait for one 0x29 real-time status frame from drive_id.

        The motor broadcasts these periodically once feedback is
        enabled.  This method just waits for the next one.

        Returns
        -------
        dict with keys:
            position   float   degrees  (range -3200 to +3200)
            speed      float   ERPM     (range -320000 to +320000)
            current    float   Amps     (range -60 to +60)
            temp       int     °C       (range -20 to +127)
            error      int     fault code (0 = no fault)
        or None on timeout.

        Fault codes
        -----------
            0  no fault
            1  motor over-temperature
            2  overcurrent
            3  overvoltage
            4  undervoltage
            5  encoder fault
            6  MOSFET over-temperature
            7  motor stall
        """
        msg = self._recv_from(drive_id, func_id=_FUNC_STATUS, timeout=timeout)
        if msg is None:
            return None

        return self._parse_status(msg.data)

    def read_latest_status(self, drive_id):
        """
        Return the latest cached status frame without consuming it.

        Returns:
            (status, timestamp), or (None, None) if no status
            frame has been received.
        """
        arb_id = (_FUNC_STATUS << 8) | drive_id

        msg, timestamp = self._bus.get_latest(
            arb_id,
            is_extended_id=True,
        )

        if msg is None:
            return None, None

        return self._parse_status(msg.data), timestamp

    def _parse_status(self, data):
        """
        Unpack an 8-byte 0x29 status payload.

        Layout (from manual section 2.3.1):
            bytes 0-1  : position  int16   range -32000..32000 → -3200..3200°
            bytes 2-3  : speed     int16   range -32000..32000 → -320000..320000 ERPM
            bytes 4-5  : current   int16   range -6000..6000   → -60..60 A
            byte  6    : temp      int8    range -20..127      °C
            byte  7    : error     uint8   fault code
        """
        pos_raw  = struct.unpack_from('>h', data, 0)[0]
        spd_raw  = struct.unpack_from('>h', data, 2)[0]
        cur_raw  = struct.unpack_from('>h', data, 4)[0]
        temp     = struct.unpack_from('b',  data, 6)[0]
        error    = data[7]

        return {
            'position' : pos_raw  * 0.1,    # degrees
            'speed'    : spd_raw  * 10.0,   # ERPM
            'current'  : cur_raw  * 0.01,   # Amps
            'temp'     : temp,               # °C
            'error'    : error,
        }

    # --------------------------------------------------------
    # Position/Velocity
    # --------------------------------------------------------

    def send_pos_vel(self, drive_id, pos_deg,
                 speed_erpm=5000, accel_erpm=5000):
        """
        Send a position-velocity loop command (mode 6).

        Parameters
        ----------
        drive_id   : int    Motor CAN ID
        pos_deg    : float  Target position in degrees
        speed_erpm : int    Max speed in ERPM
        accel_erpm : int    Max acceleration in ERPM/s
        """
        import struct
        arb_id  = self._make_arb_id(_MODE_POS_VEL, drive_id)
        pos_raw = int(pos_deg * 10000.0)
        spd_raw = int(speed_erpm / 10)
        acc_raw = int(accel_erpm / 10)
        data    = struct.pack('>ihh', pos_raw, spd_raw, acc_raw)
        self._send(_MODE_POS_VEL, drive_id, data)

    # --------------------------------------------------------
    # Disable
    # --------------------------------------------------------

    def disable(self, drive_id):
        """
        Send the motor disable command (mode 15).

        No data payload needed.  The motor responds with a frame
        confirming the disable.  Safe to call at any time.
        """
        self._send(_MODE_DISABLE, drive_id)