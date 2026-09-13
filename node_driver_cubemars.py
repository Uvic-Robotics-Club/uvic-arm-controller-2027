"""
node_driver_cubemars.py
=======================
NodeDriverBase implementation for the CubeMars AK-series motors.

Each physical CubeMars motor is represented by one CubeMarsNode
instance. Mirrors AvatarNode in structure but uses CubeMarsProtocol
instead of CANopen, and sends continuous position commands to
maintain hold since CubeMars motors go torque-free on timeout.

Usage
-----
    from can_bus                import CanBus
    from can_protocol_cubemars  import CubeMarsProtocol
    from node_driver_cubemars   import CubeMarsNode

    bus      = CanBus()
    protocol = CubeMarsProtocol(bus)

    bus.open()

    shoulder = CubeMarsNode(
        protocol    = protocol,
        node_id     = 2,
        name        = "Shoulder",
        gear_ratio  = 1,
        home_deg    = -21.6,    # from home_positions.json
    )

    shoulder.enable()
    shoulder.step(waypoint=-20.0)
    shoulder.disable()

    bus.close()
"""

import time

from node_driver_base import NodeDriverBase, NodeState


# ============================================================
# CUBEMARS-SPECIFIC CONSTANTS
# ============================================================

# How many hold commands to send after enable before
# handing control to the tick loop — mirrors PRIME_TICKS
# in AvatarNode but in time rather than SYNC ticks
PRIME_TICKS = 20

# Tick interval — must match state machine tick rate
SYNC_INTERVAL = 0.020   # seconds  (50 Hz)

# Default speed and acceleration for position-velocity commands
# These are conservative — tune once motion is confirmed working
DEFAULT_SPEED_ERPM = 30000
DEFAULT_ACCEL_ERPM = 20000

# Temperature fault thresholds
MAX_MOTOR_TEMP  = 70    # degrees C
MAX_MOSFET_TEMP = 80    # degrees C

# Maximum position divergence before watchdog triggers (degrees)
MAX_POSITION_DIVERGENCE = 10.0


class CubeMarsNode(NodeDriverBase):
    """
    Driver for a single CubeMars AK-series motor.

    All motion is expressed in degrees at this layer since
    CubeMars handles encoder resolution internally.
    Position is tracked mathematically after enable, same
    pattern as AvatarNode.

    Note on units
    -------------
    Unlike AvatarNode which works in encoder counts,
    CubeMarsNode works in degrees directly. degrees_to_counts
    and counts_to_degrees are thin wrappers that apply the
    gear ratio, keeping the interface identical to AvatarNode
    so the state machine does not need to know the difference.
    """

    def __init__(
        self,
        protocol,
        node_id,
        name,
        gear_ratio,
        home_deg,
        speed_erpm = DEFAULT_SPEED_ERPM,
        accel_erpm = DEFAULT_ACCEL_ERPM,
    ):
        """
        Parameters
        ----------
        protocol   : CubeMarsProtocol
            Shared protocol handler for the CAN bus.
        node_id    : int
            CubeMars motor CAN ID (2 for shoulder, 3 for elbow).
        name       : str
            Human-readable joint name, e.g. 'Shoulder'.
        gear_ratio : int
            Gearbox reduction ratio. Use 1 if position is already
            output-shaft degrees.
        home_deg   : float
            Motor position in degrees corresponding to the arm
            home position, as saved by home_arm_cubemars.py.
        speed_erpm : int
            Default speed for position-velocity commands.
        accel_erpm : int
            Default acceleration for position-velocity commands.
        """
        self._protocol   = protocol
        self._node_id    = node_id
        self._name       = name
        self._gear_ratio = gear_ratio
        self._home_deg   = home_deg
        self._speed_erpm = speed_erpm
        self._accel_erpm = accel_erpm

        self._state      = NodeState.DISABLED
        self._position   = None   # degrees, unknown until enable()

        self.waypoint_step = 0.5   # degrees per 2ms tick

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

    @property
    def requires_continuous_hold(self):
        return True

    # --------------------------------------------------------
    # Lifecycle
    # --------------------------------------------------------

    def enable(self):
        """
        Enable the motor and read its initial position.

        Sequence
        --------
        1. Enable feedback broadcasts
        2. Read initial position from status frame
        3. Send hold command at current position
        4. Prime hold for PRIME_TICKS before returning

        Returns True on success, False on failure.
        """
        print(f"  [{self._name}] Enabling motor {self._node_id}...")

        # Enable periodic status broadcasts
        ok = self._protocol.enable_feedback(self._node_id)
        if not ok:
            print(f"  [{self._name}] No response from motor.")
            self._state = NodeState.FAULT
            return False

        # Read initial position
        status = self._protocol.read_status(self._node_id, timeout=2.0)
        if status is None:
            print(f"  [{self._name}] Could not read initial position.")
            self._state = NodeState.FAULT
            return False

        if status['error'] != 0:
            print(
                f"  [{self._name}] Motor fault on enable:"
                f" error={status['error']}"
            )
            self._state = NodeState.FAULT
            return False

        self._position = status['position']
        self._state    = NodeState.ENABLED

        print(
            f"  [{self._name}] Enabled"
            f" | pos={self._position:.1f} deg"
            f" | temp={status['temp']}C"
        )

        # Prime — hold current position before tick loop takes over
        self.prime(PRIME_TICKS)

        return True

    def disable(self):
        """Send the motor disable command and set state to DISABLED."""
        self._protocol.disable(self._node_id)
        self._state = NodeState.DISABLED
        print(f"  [{self._name}] Disabled.")

    # --------------------------------------------------------
    # Motion
    # --------------------------------------------------------

    def step(self, waypoint):
        """
        Send one position-velocity command targeting waypoint.

        Called once per tick by the state machine — both during
        trajectory execution and during idle hold. Does NOT send
        a SYNC (CubeMars does not use SYNC).

        Parameters
        ----------
        waypoint : float   Target position in degrees.
        """
        self._protocol.send_pos_vel(
            self._node_id,
            waypoint,
            self._speed_erpm,
            self._accel_erpm,
        )
        self._position = waypoint

    def prime(self, ticks):
        """
        Hold current position for ticks cycles before motion starts.

        Blocks — only called during enable() before the tick loop.
        Mirrors AvatarNode.prime() but without SYNC since CubeMars
        does not use the CANopen SYNC mechanism.
        """
        if self._position is None:
            return

        next_tick = time.monotonic()

        for _ in range(ticks):
            self._protocol.send_pos_vel(
                self._node_id,
                self._position,
                self._speed_erpm,
                self._accel_erpm,
            )
            next_tick += SYNC_INTERVAL
            remaining  = next_tick - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)

    # --------------------------------------------------------
    # Watchdog
    # --------------------------------------------------------

    def check_watchdog(self):
        """
        Check for fault conditions on this motor.

        Reads the latest status frame non-blocking. Checks:
        - Motor fault code
        - Motor temperature
        - Driver board temperature

        Returns True if healthy, False if a fault was detected.
        Sets state to FAULT on failure.
        """
        status = self._protocol.read_status(
            self._node_id, timeout=0
        )

        # No frame ready this tick — not a fault, motor broadcasts
        # at its configured rate which may be slower than tick rate
        if status is None:
            return True

        if status['error'] != 0:
            print(
                f"  [{self._name}] Motor fault:"
                f" error code {status['error']}"
            )
            self._state = NodeState.FAULT
            return False

        if status['temp'] > MAX_MOTOR_TEMP:
            print(
                f"  [{self._name}] Over temperature:"
                f" {status['temp']}C > {MAX_MOTOR_TEMP}C"
            )
            self._state = NodeState.FAULT
            return False

        return True

    # --------------------------------------------------------
    # Unit conversion
    # --------------------------------------------------------

    def degrees_to_counts(self, degrees):
        """
        Convert output shaft degrees relative to home to an
        absolute motor position in degrees.

        For CubeMars, 'counts' are degrees since the motor
        handles encoder resolution internally. Gear ratio is
        applied so the interface matches AvatarNode.

            motor_deg = home_deg + degrees * gear_ratio

        Parameters
        ----------
        degrees : float   Output shaft angle relative to home.

        Returns
        -------
        float : absolute motor position in degrees.
        """
        return self._home_deg + degrees * self._gear_ratio

    def counts_to_degrees(self, counts):
        """
        Convert an absolute motor position in degrees to output
        shaft degrees relative to home.

            degrees = (motor_deg - home_deg) / gear_ratio

        Parameters
        ----------
        counts : float   Absolute motor position in degrees.

        Returns
        -------
        float : output shaft angle in degrees relative to home.
        """
        if counts is None:
            return None
        return (counts - self._home_deg) / self._gear_ratio