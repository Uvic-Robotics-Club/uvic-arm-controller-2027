"""
can_protocol_canopen.py
=======================
CANopen protocol framing for the Avatar motors.

Wraps a CanBus instance and exposes CANopen-specific operations:
SDO writes, SYNC broadcasts, RPDO1 sends, and TPDO1 reads.

All Avatar-specific protocol knowledge lives here and in
node_driver_avatar.py.  Nothing above this layer needs to know
about COB-IDs, SDO command specifiers, or PDO mappings.

Usage
-----
    bus      = CanBus()
    canopen  = CanOpenProtocol(bus)

    bus.open()
    canopen.sdo_write(node_id=1, index=0x6040, subindex=0, value=0x000F, size=2)
    canopen.send_sync()
    canopen.send_rpdo1(node_id=1, controlword=0x001F, mode=7, target_pos=12345)
    pos, sw = canopen.read_tpdo1(node_id=1)
    bus.close()
"""

import struct
import time


# How long to wait for an SDO acknowledgement
SDO_TIMEOUT = 1.0   # seconds

# Default timeout for a single TPDO read attempt
TPDO_TIMEOUT = 2.0  # seconds


class CanOpenProtocol:
    """
    CANopen protocol handler.

    Depends on a CanBus instance for raw frame I/O.  Does not manage
    the bus lifecycle — the caller opens and closes the bus.
    """

    def __init__(self, bus):
        """
        Parameters
        ----------
        bus : CanBus
            An open (or not-yet-open) CanBus instance.  Must be open
            before any methods on this class are called.
        """
        self._bus = bus

    # --------------------------------------------------------
    # SDO write
    # --------------------------------------------------------

    def sdo_write(self, node_id, index, subindex, value, size=4):
        """
        Write a value to a node's object dictionary via SDO.

        Parameters
        ----------
        node_id  : int   CANopen node ID (1–127)
        index    : int   Object dictionary index  (e.g. 0x6040)
        subindex : int   Object dictionary subindex (e.g. 0x00)
        value    : int   Value to write (signed or unsigned)
        size     : int   Byte width of the value: 1, 2, or 4

        Returns
        -------
        True  if the node acknowledged the write.
        False if the request timed out or the node returned an error.
        """
        cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]

        data = bytes([
            cmd,
            index & 0xFF,
            (index >> 8) & 0xFF,
            subindex,
        ]) + value.to_bytes(4, "little", signed=(value < 0))

        self._bus.send(arb_id=0x600 + node_id, data=data)

        # Wait for SDO response from node (COB-ID 0x580 + node_id)
        deadline = time.monotonic() + SDO_TIMEOUT
        while time.monotonic() < deadline:
            msg = self._bus.recv(timeout=0.1)
            if msg and msg.arbitration_id == (0x580 + node_id):
                return msg.data[0] == 0x60   # 0x60 = success

        return False   # timed out

    # --------------------------------------------------------
    # SYNC
    # --------------------------------------------------------

    def send_sync(self):
        """
        Broadcast a CANopen SYNC message (COB-ID 0x080).

        Clocks the interpolation engine of every SYNC-triggered node
        on the bus simultaneously.
        """
        self._bus.send(arb_id=0x080, data=[])

    # --------------------------------------------------------
    # RPDO1
    # --------------------------------------------------------

    def send_rpdo1(self, node_id, controlword, mode, target_pos):
        """
        Send RPDO1 to an Avatar node.

        PDO mapping (Avatar firmware):
            bytes 0-1 : controlword  (uint16 LE)
            byte  2   : mode         (uint8)
            bytes 3-6 : target pos   (int32 LE)

        Parameters
        ----------
        node_id     : int   CANopen node ID
        controlword : int   DS402 controlword  (e.g. 0x001F)
        mode        : int   Operation mode     (7 = interpolated pos)
        target_pos  : int   Target position in encoder counts
        """
        data = struct.pack("<HBi", controlword, mode, target_pos)
        self._bus.send(arb_id=0x200 + node_id, data=data)

    # --------------------------------------------------------
    # TPDO1
    # --------------------------------------------------------

    def read_tpdo1(self, node_id, timeout=TPDO_TIMEOUT):
        """
        Wait for a TPDO1 from node_id and return its contents.

        Avatar TPDO1 payload (COB-ID 0x180 + node_id):
            bytes 0-3 : actual position  (int32 LE)
            bytes 4-5 : statusword       (uint16 LE)

        Returns
        -------
        (position, statusword) on success, or (None, None) on timeout.

        Notes
        -----
        The TPDO update rate on these drives is ~10 Hz and lags behind
        actual motion.  Only use this at rest — never mid-trajectory.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._bus.recv(timeout=0.1)
            if msg and msg.arbitration_id == (0x180 + node_id):
                pos = struct.unpack_from("<i", msg.data, 0)[0]
                sw  = struct.unpack_from("<H", msg.data, 4)[0]
                return pos, sw
        return None, None

    def read_tpdo1_retry(self, node_id, attempts=3, timeout=2.0):
        """
        Attempt to read TPDO1 several times with a pause between tries.

        Used at node enable time when the bus may still be settling
        after the previous node's traffic.

        Returns
        -------
        (position, statusword) on success, or (None, None) if all
        attempts fail.
        """
        for attempt in range(attempts):
            pos, sw = self.read_tpdo1(node_id, timeout=timeout)
            if pos is not None:
                return pos, sw
            if attempt < attempts - 1:
                time.sleep(0.3)
        return None, None