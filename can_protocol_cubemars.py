"""
can_protocol_cubemars.py
========================
CubeMars motor protocol framing.

Placeholder — to be implemented once CubeMars documentation
is available.

CubeMars motors (AK series and similar) use a proprietary CAN
protocol that is NOT CANopen.  The frame format, arbitration IDs,
and command set are completely different from the Avatar motors,
but they share the same physical CAN bus (can0).

What is known so far
--------------------
- CubeMars motors typically use extended CAN IDs (29-bit).
- Commands are usually structured as:
    arbitration_id : encodes motor ID + command type
    data           : position, velocity, torque values packed
                     as fixed-point or float fields
- The exact mapping depends on the firmware version and motor model.

Implementation notes
--------------------
When documentation is available, implement this class with the same
interface pattern as CanOpenProtocol:
  - Wrap a CanBus instance (passed in __init__)
  - Expose named methods for each command type
  - Keep all magic numbers and bit packing here, not in the driver

Usage (future)
--------------
    bus      = CanBus()
    cubemars = CubeMarsProtocol(bus)

    bus.open()
    cubemars.send_position_command(motor_id=1, position=45.0)
    response = cubemars.read_response(motor_id=1)
    bus.close()
"""


class CubeMarsProtocol:
    """
    CubeMars motor protocol handler.

    Not yet implemented — stub only.
    """

    def __init__(self, bus):
        """
        Parameters
        ----------
        bus : CanBus
            Shared CanBus instance.  Must be open before any
            methods on this class are called.
        """
        self._bus = bus
        raise NotImplementedError(
            "CubeMarsProtocol is not yet implemented. "
            "Add the motor documentation and implement this class."
        )