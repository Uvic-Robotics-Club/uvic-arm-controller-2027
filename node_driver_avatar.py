"""
node_driver_avatar.py
=====================
NodeDriverBase implementation for the Avatar CANopen motors.

Each physical Avatar motor is represented by one AvatarNode instance.
The instance owns all state for that joint — current position,
enable state, home position — and communicates with the drive
exclusively through the CanOpenProtocol handler passed at construction.

Usage
-----
    from can_bus            import CanBus
    from can_protocol_canopen import CanOpenProtocol
    from node_driver_avatar import AvatarNode

    bus      = CanBus()
    protocol = CanOpenProtocol(bus)

    bus.open()

    base = AvatarNode(
        protocol     = protocol,
        node_id      = 1,
        name         = "Base",
        gear_ratio   = 100,
        encoder_cpr  = 32768,
        home_counts  = 57681,    # from home_positions.json
    )

    base.enable()
    base.step(waypoint=58000)
    # ... state machine sends SYNC after calling step() on all nodes
    base.disable()

    bus.close()
"""

import struct
import time

from node_driver_base import NodeDriverBase, NodeState


# ============================================================
# AVATAR-SPECIFIC CONSTANTS
# ============================================================

# DS402 controlwords
CW_SHUTDOWN           = 0x0006
CW_SWITCH_ON          = 0x0007
CW_ENABLE_OPERATION   = 0x000F
CW_ENABLE_INTERPOLATION = 0x001F

# DS402 operation mode
MODE_INTERPOLATED_POSITION = 7

# How many ticks to hold position after enable to flush the
# drive's stale internal setpoint before starting a trajectory
PRIME_TICKS = 20

# SYNC interval — must match the state machine tick rate
SYNC_INTERVAL = 0.020   # seconds

# Statusword bits
SW_FAULT_BIT         = 0x0008   # bit 3  — fault
SW_TARGET_REACHED    = 0x0400   # bit 10 — target reached


# Maximum position divergence before watchdog triggers.
# If the drive reports a position more than this many counts
# from the last commanded waypoint, something has gone wrong.
# Set conservatively — can be tuned once position feedback
# is confirmed reliable on these drives.
MAX_POSITION_DIVERGENCE = 50000   # counts

class AvatarNode(NodeDriverBase):
    """
    Driver for a single Avatar CANopen motor.

    All motion commands are in encoder counts.  Unit conversion
    between degrees and counts is handled by degrees_to_counts()
    and counts_to_degrees().
    """

    def __init__(
        self,
        protocol,
        node_id,
        name,
        gear_ratio,
        encoder_cpr,
        home_counts,
    ):
        """
        Parameters
        ----------
        protocol    : CanOpenProtocol
            Shared protocol handler for the CAN bus.
        node_id     : int
            CANopen node ID (1–127).
        name        : str
            Human-readable joint name, e.g. 'Base'.
        gear_ratio  : int
            Gearbox reduction ratio (e.g. 100 for 100:1).
        encoder_cpr : int
            Encoder counts per revolution of the motor shaft
            (e.g. 32768).
        home_counts : int
            Absolute encoder count corresponding to the home
            position, as saved by home_arm.py.
        """
        self._protocol    = protocol
        self._node_id     = node_id
        self._name        = name
        self._gear_ratio  = gear_ratio
        self._encoder_cpr = encoder_cpr
        self._home_counts = home_counts

        self._state       = NodeState.DISABLED
        self._position    = None   # unknown until enable() reads TPDO

    # --------------------------------------------------------
    # Identity
    # --------------------------------------------------------

    @property
    def node_id(self):
        return self._node_id

    @property
    def name(self):
        return self._name

    # --------------------------------------------------------
    # State
    # --------------------------------------------------------

    @property
    def state(self):
        return self._state

    @property
    def position(self):
        return self._position

    # --------------------------------------------------------
    # Lifecycle
    # --------------------------------------------------------

    def enable(self):
        """
        Run the DS402 enable sequence and read the initial position.

        Sequence
        --------
        1. Set interpolated position mode via SDO (0x6060 = 7)
        2. Shutdown  → Switch on  → Enable operation  → Enable interpolation
        3. Read initial position from TPDO (with retries)
        4. Prime the interpolation buffer at the current position

        Returns True on success, False on failure.
        """
        print(f"  [{self._name}] Enabling node {self._node_id}...")

        # Set mode
        self._protocol.sdo_write(
            self._node_id, 0x6060, 0x00, MODE_INTERPOLATED_POSITION, size=1
        )

        # DS402 state machine sequence
        self._protocol.sdo_write(
            self._node_id, 0x6040, 0x00, CW_SHUTDOWN, size=2
        )
        time.sleep(0.05)

        self._protocol.sdo_write(
            self._node_id, 0x6040, 0x00, CW_SWITCH_ON, size=2
        )
        time.sleep(0.05)

        self._protocol.sdo_write(
            self._node_id, 0x6040, 0x00, CW_ENABLE_OPERATION, size=2
        )
        time.sleep(0.1)

        self._protocol.sdo_write(
            self._node_id, 0x6040, 0x00, CW_ENABLE_INTERPOLATION, size=2
        )
        time.sleep(0.1)

        # Read initial position
        pos, sw = self._protocol.read_tpdo1_retry(self._node_id)

        if pos is None:
            print(f"  [{self._name}] FAILED to read initial position.")
            self._state = NodeState.FAULT
            return False

        self._position = pos
        self._state    = NodeState.ENABLED

        print(
            f"  [{self._name}] Enabled"
            f" | pos={pos}"
            f" | sw=0x{sw:04X}"
        )

        # Prime interpolation buffer at current position
        self.prime(PRIME_TICKS)

        return True

    def disable(self):
        """Send the DS402 shutdown command to disable the drive."""
        self._protocol.sdo_write(
            self._node_id, 0x6040, 0x00, CW_SHUTDOWN, size=2
        )
        self._state = NodeState.DISABLED
        print(f"  [{self._name}] Disabled.")

    # --------------------------------------------------------
    # Motion
    # --------------------------------------------------------

    def step(self, waypoint):
        """
        Send one RPDO1 targeting waypoint.

        Called once per tick by the state machine.  Does NOT send
        a SYNC — the state machine broadcasts one SYNC after calling
        step() on all active nodes.

        Updates self._position to waypoint.
        """
        self._protocol.send_rpdo1(
            self._node_id,
            CW_ENABLE_INTERPOLATION,
            MODE_INTERPOLATED_POSITION,
            waypoint,
        )
        self._position = waypoint

    def prime(self, ticks):
        """
        Hold the current position for `ticks` SYNC cycles.

        Sends RPDO1 + SYNC `ticks` times at SYNC_INTERVAL spacing,
        all targeting the current position.  This overwrites the
        drive's stale internal setpoint so the first real trajectory
        waypoint doesn't cause a lurch.

        This method blocks — it is only called during enable(),
        before the main tick loop starts.
        """
        if self._position is None:
            return

        next_sync = time.monotonic()

        for _ in range(ticks):
            self._protocol.send_rpdo1(
                self._node_id,
                CW_ENABLE_INTERPOLATION,
                MODE_INTERPOLATED_POSITION,
                self._position,
            )
            next_sync += SYNC_INTERVAL
            remaining = next_sync - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            self._protocol.send_sync()

    # --------------------------------------------------------
    # Watchdog
    # --------------------------------------------------------

    def check_watchdog(self):
        """
        Check the latest cached TPDO1 without consuming a CAN frame.

        Fault conditions:
        - Statusword fault bit is set AND target-reached is clear.

        Returns True if healthy, False if a fault was detected.
        """

        msg, timestamp = self._protocol._bus.get_latest(
            0x180 + self._node_id,
            is_extended_id=False,
        )

        # No feedback has arrived yet.
        if msg is None:
            print(f"  [{self._name}] No TPDO1 feedback received.")
            self._state = NodeState.FAULT
            return False

        # TPDO1 layout: position (4 bytes), statusword (2 bytes).
        if len(msg.data) < 6:
            print(f"  [{self._name}] Invalid TPDO1 payload length.")
            self._state = NodeState.FAULT
            return False

        sw = struct.unpack_from("<H", msg.data, 4)[0]

        fault_bit = bool(sw & SW_FAULT_BIT)
        target_reached = bool(sw & SW_TARGET_REACHED)

        if fault_bit and not target_reached:
            print(
                f"  [{self._name}] FAULT detected in statusword"
                f" 0x{sw:04X}"
            )
            self._state = NodeState.FAULT
            return False

        return True

    # --------------------------------------------------------
    # Unit conversion
    # --------------------------------------------------------

    def degrees_to_counts(self, degrees):
        """
        Convert output shaft degrees (relative to home) to an
        absolute encoder count.

            counts = home + degrees * gear_ratio * encoder_cpr / 360

        Parameters
        ----------
        degrees : float   Output shaft angle relative to home.

        Returns
        -------
        int : absolute encoder count target.
        """
        return int(
            self._home_counts
            + degrees * self._gear_ratio * self._encoder_cpr / 360
        )

    def counts_to_degrees(self, counts):
        """
        Convert an absolute encoder count to output shaft degrees
        relative to home.

            degrees = (counts - home) * 360 / (gear_ratio * encoder_cpr)

        Parameters
        ----------
        counts : int   Absolute encoder count.

        Returns
        -------
        float : output shaft angle in degrees relative to home.
        """
        return (
            (counts - self._home_counts)
            * 360
            / (self._gear_ratio * self._encoder_cpr)
        )