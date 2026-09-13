"""
state_machine.py
================
Arm controller state machine.

Owns all state transitions, trajectory execution, and watchdog
logic.  Has no I/O, no loops, and no sleeping — the main loop
in main.py calls tick() and handle_command() on a fixed interval.

States
------
    DISABLED  — startup state, motors not enabled
    IDLE      — motors enabled, holding position, ready for commands
    MOVING    — executing a point-to-point trajectory
    HOMING    — executing a home sequence (mechanically same as MOVING)
    FAULT     — fault detected, all motion stopped, requires reset

Valid command transitions
-------------------------
    ENABLE   : DISABLED  → IDLE
    MOVE     : IDLE      → MOVING
    HOME     : IDLE      → HOMING
    DISABLE  : IDLE      → DISABLED
    ESTOP    : any       → FAULT
    RESET    : FAULT     → DISABLED

Usage
-----
    sm = ArmStateMachine(nodes, protocol)
    sm.handle_command(Command(type="ENABLE"))

    # In the tick loop:
    while True:
        tick_start = time.monotonic()
        sm.handle_command(pending_command_if_any)
        sm.tick()
        status = sm.status()
        # ... send status outward ...
        elapsed = time.monotonic() - tick_start
        time.sleep(max(0, SYNC_INTERVAL - elapsed))
"""

from dataclasses import dataclass, field
from enum        import Enum, auto
from typing      import Optional

from can_bus          import CanBusError
from node_driver_base import NodeState


# ============================================================
# CONSTANTS
# ============================================================

WAYPOINT_STEP = 250     # encoder counts per tick
SYNC_INTERVAL = 0.002   # seconds — must match main.py tick rate
PRIME_TICKS   = 20      # hold-position ticks after enable


# ============================================================
# ARM STATES
# ============================================================

class ArmState(Enum):
    DISABLED = auto()
    IDLE     = auto()
    MOVING   = auto()
    HOMING   = auto()
    FAULT    = auto()


# ============================================================
# COMMAND
# ============================================================

@dataclass
class Command:
    """
    A single instruction to the state machine.

    Fields
    ------
    type    : str
        One of ENABLE MOVE MOVETO HOME DISABLE ESTOP RESET STATUS QUIT.

    targets : dict | None
        For MOVE and MOVETO commands: {node_id: degrees} mapping.
        MOVE  — degrees are relative to current position.
        MOVETO — degrees are absolute from home position.
        None for all other commands.
    """
    type:    str
    targets: Optional[dict] = None


# ============================================================
# STATUS  (outward heartbeat)
# ============================================================

@dataclass
class NodeStatus:
    """Position and state snapshot for one node."""
    node_id : int
    name    : str
    state   : NodeState
    position: Optional[int]    # encoder counts
    degrees : Optional[float]  # relative to home


@dataclass
class ArmStatus:
    """
    Full arm status snapshot sent as a heartbeat every tick.

    main.py reads this after each tick and forwards it to the
    operator (terminal for now, serial to Jetson later).
    """
    arm_state   : ArmState
    nodes       : list          # list[NodeStatus]
    fault_reason: Optional[str] # set when arm_state == FAULT
    tick        : int           # monotonically increasing tick counter


# ============================================================
# STATE MACHINE
# ============================================================

class ArmStateMachine:
    """
    Controls the arm's operational state and trajectory execution.

    Parameters
    ----------
    nodes    : list[NodeDriverBase]
        All node driver instances the controller manages.
        For now just node 1 (Base) and node 6 (Wrist Roll).
    protocol : CanOpenProtocol
        Shared protocol handler — used to send SYNC broadcasts.
    """

    def __init__(self, nodes, protocol):
        self._nodes        = {node.node_id: node for node in nodes}
        self._protocol     = protocol
        self._state        = ArmState.DISABLED
        self._fault_reason = None
        self._tick_count   = 0

        # Trajectory state — populated when entering MOVING or HOMING
        # key: node_id, value: list of encoder count waypoints
        self._waypoints:   dict = {}
        self._tick_index:  int  = 0
        self._total_ticks: int  = 0

    # --------------------------------------------------------
    # Public interface
    # --------------------------------------------------------

    @property
    def state(self):
        return self._state

    def handle_command(self, cmd: Optional[Command]):
        """
        Process one incoming command.

        Called by main.py once per tick before tick().
        If cmd is None (no command this tick) this is a no-op.
        """
        if cmd is None:
            return

        t = cmd.type.upper()

        if t == "ENABLE":
            self._cmd_enable()

        elif t == "MOVE":
            self._cmd_move_relative(cmd.targets or {})

        elif t == "MOVETO":
            self._cmd_move_absolute(cmd.targets or {})

        elif t == "HOME":
            self._cmd_home()

        elif t == "DISABLE":
            self._cmd_disable()

        elif t == "ESTOP":
            self._cmd_estop("Operator ESTOP")

        elif t == "RESET":
            self._cmd_reset()

        else:
            print(f"  [SM] Unknown command: {cmd.type!r}")

    def tick(self):
        """
        Advance the state machine by one tick.

        Called by main.py every SYNC_INTERVAL seconds.
        Order of operations:
            1. Watchdog checks on all enabled nodes
            2. Advance trajectory if MOVING or HOMING
            3. Increment tick counter
        """
        try:
            # 1. Watchdog
            if self._state in (ArmState.IDLE, ArmState.MOVING,
                            ArmState.HOMING):
                for node in self._nodes.values():
                    if node.state == NodeState.ENABLED:
                        if not node.check_watchdog():
                            self._enter_fault(
                                f"Watchdog fault on node"
                                f" {node.node_id} ({node.name})"
                            )
                            return

            # 2. Trajectory
            if self._state in (ArmState.MOVING, ArmState.HOMING):
                self._advance_trajectory()

            # 3. CubeMars idle hold — step at current position
            #    every tick so position tracking stays accurate.
            #    Avatar nodes hold passively so don't need this.
            elif self._state == ArmState.IDLE:
                for node in self._nodes.values():
                    if (node.state == NodeState.ENABLED
                            and node.requires_continuous_hold):
                        node.step(node.position)

        except CanBusError as e:
            self._enter_fault(f"CAN bus error: {e}")
            return

        self._tick_count += 1

    def status(self) -> ArmStatus:
        """
        Return a snapshot of the current arm state.

        Called by main.py after each tick to build the heartbeat.
        """
        node_statuses = []
        for node in self._nodes.values():
            pos = node.position
            deg = node.counts_to_degrees(pos) if pos is not None else None
            node_statuses.append(NodeStatus(
                node_id  = node.node_id,
                name     = node.name,
                state    = node.state,
                position = pos,
                degrees  = deg,
            ))

        return ArmStatus(
            arm_state    = self._state,
            nodes        = node_statuses,
            fault_reason = self._fault_reason,
            tick         = self._tick_count,
        )

    # --------------------------------------------------------
    # Command handlers
    # --------------------------------------------------------

    def _cmd_enable(self):
        if self._state != ArmState.DISABLED:
            print(f"  [SM] ENABLE ignored — current state: {self._state.name}")
            return

        print("  [SM] Enabling all nodes...")
        all_ok = True

        try:
            for node in self._nodes.values():
                if node.state == NodeState.ENABLED:
                    # Already enabled from pre-enable step — skip
                    print(f"  [{node.name}] Already enabled — skipping")
                    continue
                ok = node.enable()
                if not ok:
                    self._enter_fault(
                        f"Failed to enable node {node.node_id} ({node.name})"
                    )
                    all_ok = False
                    break
        except CanBusError as e:
            self._enter_fault(f"CAN bus error during enable: {e}")
            return

        if all_ok:
            self._state = ArmState.IDLE
            print("  [SM] → IDLE")

    def _cmd_move_relative(self, targets: dict):
        """
        Move one or more nodes by degrees relative to their current position.

        targets : {node_id: degrees}
            e.g. {1: 10, 6: -20}  →  node 1 +10°, node 6 -20° from current
        """
        if self._state != ArmState.IDLE:
            print(f"  [SM] MOVE ignored — current state: {self._state.name}")
            return

        if not targets:
            print("  [SM] MOVE ignored — no targets specified")
            return

        for node_id in targets:
            if node_id not in self._nodes:
                print(f"  [SM] MOVE ignored — unknown node_id: {node_id}")
                return

        # Use degrees_to_counts for the delta by converting
        # current + delta through the node's own unit conversion
        encoder_targets = {}
        for node_id, degrees in targets.items():
            node   = self._nodes[node_id]
            # degrees_to_counts gives absolute target from home
            # so we need current degrees + delta degrees
            current_deg = node.counts_to_degrees(node.position)
            encoder_targets[node_id] = node.degrees_to_counts(
                current_deg + degrees
            )

        self._build_trajectories(encoder_targets)
        self._state = ArmState.MOVING

        for node_id, degrees in targets.items():
            node = self._nodes[node_id]
            print(
                f"  [SM] MOVE node {node_id} ({node.name})"
                f"  {degrees:+.2f}° relative"
                f"  → {encoder_targets[node_id]}"
            )
        print(f"  [SM] {self._total_ticks} steps  → MOVING")

    def _cmd_move_absolute(self, targets: dict):
        """
        Move one or more nodes to degrees absolute from their home position.

        targets : {node_id: degrees}
            e.g. {1: 45, 6: 90}  →  node 1 to 45°, node 6 to 90° from home
        """
        if self._state != ArmState.IDLE:
            print(f"  [SM] MOVETO ignored — current state: {self._state.name}")
            return

        if not targets:
            print("  [SM] MOVETO ignored — no targets specified")
            return

        for node_id in targets:
            if node_id not in self._nodes:
                print(f"  [SM] MOVETO ignored — unknown node_id: {node_id}")
                return

        # Convert absolute degrees to encoder counts
        encoder_targets = {}
        for node_id, degrees in targets.items():
            node = self._nodes[node_id]
            encoder_targets[node_id] = node.degrees_to_counts(degrees)

        self._build_trajectories(encoder_targets)
        self._state = ArmState.MOVING

        for node_id, degrees in targets.items():
            node = self._nodes[node_id]
            print(
                f"  [SM] MOVETO node {node_id} ({node.name})"
                f"  {degrees:.2f}° from home"
                f"  → {encoder_targets[node_id]} counts"
            )
        print(f"  [SM] {self._total_ticks} steps  → MOVING")

    def _cmd_home(self):
        if self._state != ArmState.IDLE:
            print(f"  [SM] HOME ignored — current state: {self._state.name}")
            return

        print("  [SM] Building home trajectories...")
        targets = {}
        for node in self._nodes.values():
            targets[node.node_id] = node.degrees_to_counts(0.0)

        self._build_trajectories(targets)
        self._state = ArmState.HOMING
        print(f"  [SM] → HOMING  ({self._total_ticks} steps)")

    def _cmd_disable(self):
        if self._state != ArmState.IDLE:
            print(f"  [SM] DISABLE ignored — current state: {self._state.name}")
            return

        print("  [SM] Disabling all nodes...")
        for node in self._nodes.values():
            node.disable()

        self._state = ArmState.DISABLED
        print("  [SM] → DISABLED")

    def _cmd_estop(self, reason="Operator ESTOP"):
        self._enter_fault(reason)

    def _cmd_reset(self):
        if self._state != ArmState.FAULT:
            print(f"  [SM] RESET ignored — current state: {self._state.name}")
            return

        print("  [SM] Resetting from FAULT → DISABLED")
        self._fault_reason = None
        self._waypoints    = {}
        self._tick_index   = 0
        self._total_ticks  = 0
        self._state        = ArmState.DISABLED

    # --------------------------------------------------------
    # Trajectory
    # --------------------------------------------------------

    def _build_trajectories(self, targets: dict):
        """
        Pre-compute waypoint lists for all target nodes.

        Parameters
        ----------
        targets : dict  {node_id: target_encoder_count}

        All lists are padded to the same length so all nodes
        finish on the same tick.
        """
        raw = {}
        for node_id, target in targets.items():
            node  = self._nodes[node_id]
            start = node.position
            # Get step size appropriate for this node's units
            step  = getattr(node, 'waypoint_step', WAYPOINT_STEP)
            raw[node_id] = self._make_waypoints(start, target, step)

        max_len = max(len(wps) for wps in raw.values()) if raw else 0
        for node_id in raw:
            while len(raw[node_id]) < max_len:
                raw[node_id].append(raw[node_id][-1])

        self._waypoints   = raw
        self._tick_index  = 0
        self._total_ticks = max_len

    def _make_waypoints(self, start, target, step_size=WAYPOINT_STEP):
        """Build a list of waypoints from start to target."""
        delta = target - start
        if delta == 0:
            return [target]

        # Use the node's own step size if it defines one,
        # otherwise fall back to the global default.
        # This handles the difference between Avatar (counts)
        # and CubeMars (degrees) units.
        direction = 1 if delta > 0 else -1
        step      = direction * abs(WAYPOINT_STEP)

        waypoints = []
        current   = start
        while abs(target - current) > abs(step):
            current += step
            waypoints.append(current)
        waypoints.append(target)
        return waypoints

    def _advance_trajectory(self):
        """
        Send one waypoint to each active node then broadcast SYNC.

        Called every tick while in MOVING or HOMING state.
        Transitions to IDLE when all waypoints are exhausted.
        """
        if self._tick_index >= self._total_ticks:
            # Trajectory complete — hold final position for a few ticks
            self._finish_trajectory()
            return

        # Step each node with its waypoint for this tick
        for node_id, wps in self._waypoints.items():
            self._nodes[node_id].step(wps[self._tick_index])

        # One SYNC clocks all nodes simultaneously
        self._protocol.send_sync()

        self._tick_index += 1

    def _finish_trajectory(self):
        """
        Hold all nodes at their final position for PRIME_TICKS then
        transition to IDLE.
        """
        # Send final position to all nodes a few more times so the
        # drive doesn't stall at the last waypoint
        for node_id, wps in self._waypoints.items():
            self._nodes[node_id].step(wps[-1])
        self._protocol.send_sync()

        # Once we've held long enough, transition
        # We reuse _tick_index continuing past _total_ticks as
        # a hold counter
        hold_ticks = self._tick_index - self._total_ticks
        if hold_ticks >= 5:
            prev_state         = self._state
            self._state        = ArmState.IDLE
            self._waypoints    = {}
            self._tick_index   = 0
            self._total_ticks  = 0
            print(f"  [SM] Trajectory complete → IDLE"
                  f"  (was {prev_state.name})")

        self._tick_index += 1

    # --------------------------------------------------------
    # Fault
    # --------------------------------------------------------

    def _enter_fault(self, reason: str):
        print(f"  [SM] FAULT: {reason}")
        self._fault_reason = reason
        self._state        = ArmState.FAULT
        self._waypoints    = {}
        self._tick_index   = 0
        self._total_ticks  = 0

        # Best-effort disable of all enabled nodes
        for node in self._nodes.values():
            if node.state == NodeState.ENABLED:
                try:
                    node.disable()
                except Exception:
                    pass