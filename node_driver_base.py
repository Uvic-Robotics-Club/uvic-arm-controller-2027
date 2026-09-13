"""
node_driver_base.py
===================
Abstract base class defining the interface that every node driver
must implement, regardless of motor type or CAN protocol.

The state machine and arm controller only ever interact with nodes
through this interface.  This is what makes it possible to mix
AvatarNode and CubeMarsNode instances in the same arm without the
higher layers needing to know which is which.

Implementing a new driver
-------------------------
Subclass NodeDriverBase and implement every method marked
@abstractmethod.  The constructor signature is up to the subclass
since different motor types need different configuration, but at
minimum it should accept the protocol handler and a node/motor ID.

Example
-------
    class AvatarNode(NodeDriverBase):

        def __init__(self, protocol, node_id, gear_ratio, encoder_cpr):
            self._protocol    = protocol
            self._node_id     = node_id
            self._gear_ratio  = gear_ratio
            self._encoder_cpr = encoder_cpr

        def enable(self):
            ...

        def step(self, waypoint):
            ...

Node states
-----------
Every node is always in one of these states.  The state machine
reads node.state to decide what to do next.

    DISABLED   — motor not enabled, not accepting motion commands
    ENABLED    — motor enabled and holding position, ready to move
    FAULT      — motor reported an error or watchdog triggered;
                 requires explicit recovery before re-enabling
"""

from abc import ABC, abstractmethod
from enum import Enum, auto


class NodeState(Enum):
    DISABLED = auto()
    ENABLED  = auto()
    FAULT    = auto()


class NodeDriverBase(ABC):
    """
    Abstract base class for a single motor node.

    All motion is expressed in encoder counts at this layer.
    Unit conversion (degrees ↔ counts) lives in arm.py above.
    """

    # --------------------------------------------------------
    # Identity  (subclasses must set these in __init__)
    # --------------------------------------------------------

    @property
    @abstractmethod
    def node_id(self):
        """
        The motor's identifier on the CAN bus.

        For CANopen motors this is the CANopen node ID (1–127).
        For CubeMars motors this is the motor ID set in firmware.
        """

    @property
    @abstractmethod
    def name(self):
        """Human-readable joint name, e.g. 'Base' or 'Wrist Roll'."""

    # --------------------------------------------------------
    # State
    # --------------------------------------------------------

    @property
    @abstractmethod
    def state(self):
        """
        Current NodeState of this node.

        The state machine polls this after every tick to detect
        faults and decide whether to continue a trajectory.
        """

    @property
    @abstractmethod
    def position(self):
        """
        Last known position in encoder counts.

        Updated by enable() from the TPDO, then tracked
        mathematically during motion.  Not read back mid-trajectory
        because the TPDO update rate is too slow on these drives.
        """

    # --------------------------------------------------------
    # Lifecycle
    # --------------------------------------------------------

    @abstractmethod
    def enable(self):
        """
        Enable the motor and prepare it for motion.

        Responsibilities:
        - Run the drive's enable sequence (e.g. DS402 state machine
          for CANopen, or equivalent for other protocols).
        - Read the initial position from the drive and store it
          so that position tracking starts from a real value.
        - Set state to ENABLED on success, FAULT on failure.

        Returns
        -------
        True  on success.
        False if the drive could not be enabled or position could
              not be read.
        """

    @abstractmethod
    def disable(self):
        """
        Disable the motor.

        The drive should hold its last position passively (or go
        torque-free, depending on firmware).  Sets state to DISABLED.
        """

    # --------------------------------------------------------
    # Motion  (called once per tick by the state machine)
    # --------------------------------------------------------

    @abstractmethod
    def step(self, waypoint):
        """
        Command the motor to move toward waypoint on this tick.

        This is called once per SYNC tick during a trajectory.  It
        should send exactly one motion command to the drive — the
        state machine handles the SYNC broadcast after calling step()
        on all nodes, so this method must NOT send a SYNC itself.

        Parameters
        ----------
        waypoint : int
            Target encoder count for this tick.  The state machine
            pre-computes the full waypoint list and passes them one
            at a time.

        Notes
        -----
        - This method must be non-blocking and return quickly.
        - Do not sleep inside this method.
        - Update self.position to waypoint after sending.
        """

    @abstractmethod
    def prime(self, ticks):
        """
        Hold the current position for a number of ticks before moving.

        Called after enable() to overwrite the drive's stale internal
        setpoint with the actual current position, preventing the
        initial lurch caused by the drive resuming a pre-power-off
        setpoint.

        Parameters
        ----------
        ticks : int
            Number of SYNC ticks to hold position.  The state machine
            sends the SYNC after each call to step(), so prime()
            should call step(self.position) exactly `ticks` times,
            sleeping SYNC_INTERVAL between each, then return.

        Notes
        -----
        Unlike step(), prime() IS allowed to block — it is called
        once during the enable sequence before the main tick loop
        takes over.
        """

    # --------------------------------------------------------
    # Watchdog
    # --------------------------------------------------------

    @abstractmethod
    def check_watchdog(self):
        """
        Check for fault conditions and update state if one is found.

        Called once per tick by the state machine.  Should check
        whatever fault conditions are detectable for this motor type,
        for example:
          - Drive statusword indicates a fault bit
          - Position has diverged too far from the commanded waypoint
          - No TPDO received within the expected window

        If a fault is detected, set state to FAULT and return False.
        Return True if everything looks healthy.

        Notes
        -----
        This method must be non-blocking and return quickly.
        Heavy reads (e.g. SDO reads) should not be done here on
        every tick — cache values or use TPDO data instead.
        """

    # --------------------------------------------------------
    # Unit conversion helpers
    # --------------------------------------------------------

    @abstractmethod
    def degrees_to_counts(self, degrees):
        """
        Convert output shaft degrees to encoder counts relative to home.

        Parameters
        ----------
        degrees : float
            Desired output shaft angle in degrees, relative to the
            home position (positive = away from home).

        Returns
        -------
        int : absolute encoder count target
              = home_position + degrees * gear_ratio * encoder_cpr / 360
        """

    @abstractmethod
    def counts_to_degrees(self, counts):
        """
        Convert an absolute encoder count to output shaft degrees
        relative to home.

        Parameters
        ----------
        counts : int   Absolute encoder count.

        Returns
        -------
        float : output shaft angle in degrees relative to home position.
        """