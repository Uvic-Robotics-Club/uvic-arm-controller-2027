"""
can_bus.py
==========
Thin wrapper around a physical CAN interface.

Responsibilities
----------------
- Open and close the socketcan interface.
- Send raw CAN frames (any arbitration ID, any payload).
- Receive raw CAN frames with a timeout.

This class has zero knowledge of CANopen, CubeMars, or any other
protocol.  All protocol framing lives in can_protocol_*.py files
that depend on this class.

Usage
-----
    bus = CanBus()
    bus.open()
    bus.send(arb_id=0x601, data=bytes([0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]))
    msg = bus.recv(timeout=0.1)
    bus.close()

    # Or as a context manager:
    with CanBus() as bus:
        ...
"""

import can


DEFAULT_CHANNEL   = "can0"
DEFAULT_INTERFACE = "socketcan"


class CanBusError(Exception):
    """Raised when the CAN interface cannot be opened or has failed."""
    pass


class CanBus:
    """
    Manages a single CAN interface and exposes raw send/recv.

    One instance is created at startup and passed to every protocol
    handler and node driver via dependency injection.
    """

    def __init__(
        self,
        channel   = DEFAULT_CHANNEL,
        interface = DEFAULT_INTERFACE,
    ):
        self._channel   = channel
        self._interface = interface
        self._bus       = None

    # --------------------------------------------------------
    # Lifecycle
    # --------------------------------------------------------

    def open(self):
        """Open the CAN interface.  Must be called before send/recv."""
        if self._bus is not None:
            return
        try:
            self._bus = can.Bus(
                channel   = self._channel,
                interface = self._interface,
            )
        except Exception as e:
            raise CanBusError(f"Failed to open {self._channel}: {e}") from e

    def close(self):
        """Shut down the CAN interface cleanly."""
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    def _require_open(self):
        if self._bus is None:
            raise CanBusError(
                "CAN bus is not open — call open() first."
            )

    # --------------------------------------------------------
    # Raw send
    # --------------------------------------------------------

    def send(self, arb_id, data):
        """
        Send a single CAN frame.
 
        Parameters
        ----------
        arb_id : int            11-bit arbitration ID
        data   : bytes | list   Up to 8 bytes of payload
 
        Raises
        ------
        CanBusError
            If the interface is down or the frame could not be sent.
            Common cause: socketcan interface not brought up, or
            bitrate mismatch (expected 1 Mbps).
            Run:  sudo ip link set can0 up type can bitrate 1000000
        """
        self._require_open()
 
        msg = can.Message(
            arbitration_id = arb_id,
            data           = bytes(data),
            is_extended_id = False,
        )
 
        try:
            self._bus.send(msg)
 
        except OSError as e:
            if e.errno == 105:
                # Errno 105: No buffer space available.
                # The kernel socketcan TX queue is full — transient
                # congestion at high send rates.  Wait one tick and
                # retry once before faulting.
                # Permanent fix: sudo ip link set can0 txqueuelen 1000
                import time
                time.sleep(0.002)
                try:
                    self._bus.send(msg)
                except Exception as retry_e:
                    raise CanBusError(
                        f"CAN TX buffer full and retry failed: {retry_e}\n"
                        f"  Run: sudo ip link set can0 txqueuelen 1000"
                    ) from retry_e
            elif e.errno == 100:
                raise CanBusError(
                    f"CAN interface is down: {e}\n"
                    f"  Run: sudo ip link set can0 up type can bitrate 1000000"
                ) from e
            else:
                raise CanBusError(f"CAN send failed (errno {e.errno}): {e}") from e
 
        except Exception as e:
            raise CanBusError(f"CAN send failed: {e}") from e

    # --------------------------------------------------------
    # Raw recv
    # --------------------------------------------------------

    def recv(self, timeout=0.1):
        """
        Wait for the next CAN frame.

        Parameters
        ----------
        timeout : float   Seconds to wait.  0 = non-blocking.

        Returns
        -------
        can.Message on success, or None on timeout.
        """
        self._require_open()
        return self._bus.recv(timeout=timeout)