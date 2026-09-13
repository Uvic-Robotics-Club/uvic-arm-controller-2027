"""
main.py
=======
Pi arm controller entry point.

Owns the 2ms tick loop, keyboard command input, and heartbeat
output.  The state machine has no I/O — this file is the only
place that reads input or prints output during normal operation.

Keyboard commands (simulating Jetson serial input)
--------------------------------------------------
    enable                — enable all nodes
    move <id> <deg>       — move node <id> by <deg> degrees RELATIVE
                            to current position.  e.g. move 1 10
    moveto <id> <deg>     — move node <id> to <deg> degrees ABSOLUTE
                            from home.  e.g. moveto 1 45
    home                  — move all nodes to home position (0°)
    disable               — disable all nodes
    estop                 — emergency stop → FAULT
    reset                 — recover from FAULT → DISABLED
    status                — print current status once
    quit  /  exit         — disable and exit

Input is read on a background thread so the tick loop is never
blocked waiting for keyboard input.

Usage
-----
    cd ~/pi_controller
    source venv/bin/activate
    python main.py
"""

import json
import queue
import sys
import threading
import time

from can_bus              import CanBus
from can_protocol_canopen import CanOpenProtocol
from node_driver_avatar   import AvatarNode
from state_machine        import ArmStateMachine, ArmState, Command


# ============================================================
# CONFIGURATION
# ============================================================

HOME_FILE     = "/home/pi/pi_controller/home_positions.json"
ACTIVE_NODES  = [1, 6]      # nodes to load and manage
SYNC_INTERVAL = 0.002       # seconds — must match state_machine.py

# How often to print the heartbeat status to the terminal
# (every N ticks).  At 2ms ticks, 50 = ~100ms = 10Hz display rate.
STATUS_PRINT_INTERVAL = 50


# ============================================================
# LOAD NODES FROM JSON
# ============================================================

def load_nodes(protocol):
    """Instantiate AvatarNode for each active node from the home file."""
    with open(HOME_FILE) as f:
        data = json.load(f)

    nodes = []
    for node_id in ACTIVE_NODES:
        entry = data[str(node_id)]
        nodes.append(AvatarNode(
            protocol    = protocol,
            node_id     = node_id,
            name        = entry["name"],
            gear_ratio  = entry["gear_ratio"],
            encoder_cpr = entry["encoder_cpr"],
            home_counts = entry["encoder_home"],
        ))
    return nodes


# ============================================================
# COMMAND PARSER
# ============================================================

def parse_command(line: str):
    """
    Parse a keyboard input line into a Command.

    Returns a Command on success, None if the line is empty,
    or raises ValueError with a message if the input is invalid.
    """
    parts = line.strip().lower().split()
    if not parts:
        return None

    t = parts[0]

    if t == "enable":
        return Command(type="ENABLE")

    elif t == "disable":
        return Command(type="DISABLE")

    elif t == "home":
        return Command(type="HOME")

    elif t == "estop":
        return Command(type="ESTOP")

    elif t == "reset":
        return Command(type="RESET")

    elif t == "status":
        return Command(type="STATUS")

    elif t in ("quit", "exit"):
        return Command(type="QUIT")

    elif t == "move":
        # Accepts one or more node_id/degrees pairs:
        #   move 1 10
        #   move 1 10 6 -20
        args = parts[1:]
        if len(args) < 2 or len(args) % 2 != 0:
            raise ValueError(
                "Usage: move <node_id> <degrees> [<node_id> <degrees> ...]"
            )
        try:
            targets = {
                int(args[i]): float(args[i + 1])
                for i in range(0, len(args), 2)
            }
        except ValueError:
            raise ValueError(
                "Usage: move <node_id> <degrees> [<node_id> <degrees> ...]"
            )
        return Command(type="MOVE", targets=targets)

    elif t == "moveto":
        # Accepts one or more node_id/degrees pairs:
        #   moveto 1 45
        #   moveto 1 45 6 90
        args = parts[1:]
        if len(args) < 2 or len(args) % 2 != 0:
            raise ValueError(
                "Usage: moveto <node_id> <degrees> [<node_id> <degrees> ...]"
            )
        try:
            targets = {
                int(args[i]): float(args[i + 1])
                for i in range(0, len(args), 2)
            }
        except ValueError:
            raise ValueError(
                "Usage: moveto <node_id> <degrees> [<node_id> <degrees> ...]"
            )
        return Command(type="MOVETO", targets=targets)

    else:
        raise ValueError(f"Unknown command: {t!r}")


# ============================================================
# KEYBOARD INPUT THREAD
# ============================================================

def input_thread(cmd_queue: queue.Queue, stop_event: threading.Event):
    """
    Reads lines from stdin and puts parsed Commands onto cmd_queue.

    Runs on a daemon thread so it never blocks the tick loop.
    """
    print("  Commands:")
    print("    enable | disable | home | estop | reset | status | quit")
    print("    move   <id> <deg> [<id> <deg> ...]   relative move")
    print("    moveto <id> <deg> [<id> <deg> ...]   absolute from home")
    print()

    while not stop_event.is_set():
        try:
            line = input("> ")
        except EOFError:
            cmd_queue.put(Command(type="QUIT"))
            break

        try:
            cmd = parse_command(line)
            if cmd is not None:
                cmd_queue.put(cmd)
        except ValueError as e:
            print(f"  Error: {e}")


# ============================================================
# STATUS DISPLAY
# ============================================================

def print_status(status):
    """Print a one-line heartbeat status to the terminal."""
    node_parts = []
    for ns in status.nodes:
        deg = f"{ns.degrees:.1f}°" if ns.degrees is not None else "?"
        node_parts.append(f"{ns.name}={deg}")

    fault = f"  FAULT: {status.fault_reason}" \
            if status.fault_reason else ""

    print(f"  [{status.arm_state.name:8s}]  tick={status.tick:6d}"
          f"  {' | '.join(node_parts)}{fault}")


# ============================================================
# MAIN
# ============================================================

def main():

    print()
    print("=" * 56)
    print("  ROVER ARM  —  PI CONTROLLER")
    print("=" * 56)
    print(f"  Nodes        : {ACTIVE_NODES}")
    print(f"  Tick rate    : {int(1/SYNC_INTERVAL)} Hz  ({SYNC_INTERVAL*1000:.0f}ms)")
    print(f"  Home file    : {HOME_FILE}")
    print()

    bus      = CanBus()
    protocol = CanOpenProtocol(bus)

    try:
        bus.open()
    except Exception as e:
        print(f"  ERROR: Could not open CAN bus: {e}")
        return

    nodes = load_nodes(protocol)
    sm    = ArmStateMachine(nodes, protocol)

    # --------------------------------------------------------
    # Start keyboard input thread
    # --------------------------------------------------------
    cmd_queue  = queue.Queue()
    stop_event = threading.Event()

    t = threading.Thread(
        target = input_thread,
        args   = (cmd_queue, stop_event),
        daemon = True,
    )
    t.start()

    # --------------------------------------------------------
    # Tick loop
    # --------------------------------------------------------
    print(f"  State: {sm.state.name}")
    print()

    running    = True
    tick_count = 0

    try:
        while running:
            tick_start = time.monotonic()

            # 1. Read one pending command (non-blocking)
            cmd = None
            try:
                cmd = cmd_queue.get_nowait()
            except queue.Empty:
                pass

            # Handle quit outside the state machine
            if cmd and cmd.type == "QUIT":
                print("  Quitting...")
                running = False
                break

            # Handle status print outside the state machine
            if cmd and cmd.type == "STATUS":
                print_status(sm.status())
                cmd = None

            # 2. Hand command to state machine
            sm.handle_command(cmd)

            # 3. Tick the state machine
            sm.tick()

            # 4. Sleep remainder of tick on an absolute clock
            elapsed   = time.monotonic() - tick_start
            remaining = SYNC_INTERVAL - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\n  CTRL-C received.")

    finally:
        stop_event.set()
        print()
        print("  Shutting down...")

        # Best-effort disable of all nodes
        if sm.state not in (ArmState.DISABLED, ArmState.FAULT):
            sm.handle_command(Command(type="DISABLE"))
        elif sm.state == ArmState.FAULT:
            for node in nodes:
                try:
                    node.disable()
                except Exception:
                    pass

        bus.close()
        print("  Done.")
        print()


if __name__ == "__main__":
    main()