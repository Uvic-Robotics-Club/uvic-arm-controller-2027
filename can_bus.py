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
import threading
import time

from collections import deque

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
        # Receive management
        self._rx_queue = deque(maxlen=4096)
        self._latest_frames = {}

        self._rx_condition = threading.Condition()
        self._rx_stop = threading.Event()
        self._rx_thread = None

        # Prevent simultaneous sends from interleaving.
        self._tx_lock = threading.Lock()

    # --------------------------------------------------------
    # Lifecycle
    # --------------------------------------------------------

    def open(self):
        """Open the CAN interface and start centralized reception."""
        if self._bus is not None:
            return

        try:
            self._bus = can.Bus(
                channel=self._channel,
                interface=self._interface,
            )
        except Exception as e:
            raise CanBusError(
                f"Failed to open {self._channel}: {e}"
            ) from e

        self._rx_stop.clear()

        self._rx_thread = threading.Thread(
            target=self._receive_loop,
            name="CAN-Receiver",
            daemon=True,
        )

        self._rx_thread.start()

    def _receive_loop(self):
        """
        Read every incoming CAN frame from the physical interface.

        This is the ONLY method that should directly call
        python-can's recv().
        """
        while not self._rx_stop.is_set():
            try:
                msg = self._bus.recv(timeout=0.1)

                if msg is None:
                    continue

                with self._rx_condition:
                    self._rx_queue.append(msg)

                    key = (
                        msg.is_extended_id,
                        msg.arbitration_id,
                    )

                    self._latest_frames[key] = (
                        msg,
                        time.monotonic(),
                    )

                    self._rx_condition.notify_all()

            except Exception as e:
                if self._rx_stop.is_set():
                    break

                print(f"[CAN RX] Receive error: {e}")
                time.sleep(0.01)

    def close(self):
        """Stop reception and shut down the CAN interface."""
        if self._bus is None:
            return

        self._rx_stop.set()

        with self._rx_condition:
            self._rx_condition.notify_all()

        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None

        self._bus.shutdown()
        self._bus = None

        with self._rx_condition:
            self._rx_queue.clear()
            self._latest_frames.clear()

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

    def send(self, arb_id, data, is_extended_id=False):
        """
        Send a standard or extended CAN frame.
        """
        self._require_open()

        msg = can.Message(
            arbitration_id=arb_id,
            data=bytes(data),
            is_extended_id=is_extended_id,
        )

        try:
            with self._tx_lock:
                self._bus.send(msg)

        except OSError as e:
            if e.errno == 100:
                raise CanBusError(
                    f"CAN interface is down: {e}"
                ) from e

            if e.errno == 105:
                raise CanBusError(
                    "CAN TX buffer full (ENOBUFS). "
                    "Transmission could not be queued."
                ) from e

            raise CanBusError(
                f"CAN send failed (errno {e.errno}): {e}"
            ) from e

        except Exception as e:
            raise CanBusError(f"CAN send failed: {e}") from e

    # --------------------------------------------------------
    # Recv
    # --------------------------------------------------------

    def recv_matching(self, predicate, timeout=0.1):
        """
        Retrieve the next buffered frame satisfying predicate.

        Frames that do not match remain available for other consumers.
        """
        self._require_open()

        deadline = time.monotonic() + timeout

        with self._rx_condition:
            while True:

                for index, msg in enumerate(self._rx_queue):
                    if predicate(msg):
                        del self._rx_queue[index]
                        return msg

                remaining = deadline - time.monotonic()

                if remaining <= 0:
                    return None

                self._rx_condition.wait(timeout=remaining)


    def recv(self, timeout=0.1):
        """
        Retrieve the next buffered frame, regardless of its ID.

        Prefer recv_matching() in protocol implementations.
        """
        return self.recv_matching(
            lambda msg: True,
            timeout=timeout,
        )


    def get_latest(self, arb_id, is_extended_id=False):
        """
        Return the most recently received frame for an arbitration ID.

        Returns:
            (message, timestamp) or (None, None)
        """
        self._require_open()

        key = (is_extended_id, arb_id)

        with self._rx_condition:
            return self._latest_frames.get(key, (None, None))