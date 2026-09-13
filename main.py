"""
main.py
=======
Pi arm controller entry point.

Owns the 2ms tick loop, keyboard command input, and heartbeat
output.  The state machine has no I/O — this file is the only
place that reads input or prints output during normal operation.

Keyboard commands
-----------------
    enable                — enable all nodes
    move <id> <deg>       — move node <id> by <deg> degrees RELATIVE
    moveto <id> <deg>     — move node <id> to <deg> degrees ABSOLUTE
    home                  — move all nodes to home position (0°)
    disable               — disable all nodes
    estop                 — emergency stop → FAULT
    reset                 — recover from FAULT → DISABLED
    status                — print current status once
    quit  /  exit         — disable and exit
"""

import json
import queue
import struct
import sys
import threading
import time

import can

from can_bus              import CanBus
from can_protocol_canopen import CanOpenProtocol
from can_protocol_cubemars import CubeMarsProtocol
from node_driver_avatar   import AvatarNode
from node_driver_cubemars import CubeMarsNode
from state_machine        import ArmStateMachine, ArmState, Command


# ============================================================
# CONFIGURATION
# ============================================================

HOME_FILE     = "/home/pi/pi_controller/home_positions.json"

# Avatar CANopen nodes managed by the state machine
AVATAR_NODE_IDS   = [1, 4, 5, 6]

# CubeMars nodes — held continuously, also in state machine
CUBEMARS_NODE_IDS = [2, 3]

SYNC_INTERVAL         = 0.002   # seconds — must match state_machine.py
STATUS_PRINT_INTERVAL = 50      # ticks between heartbeat prints


# ============================================================
# LOAD NODES FROM JSON
# ============================================================

def load_nodes(canopen_protocol, cubemars_protocol):
    """
    Instantiate all node drivers from the home file.

    Returns (avatar_nodes, cubemars_nodes) as separate lists
    so main can handle them differently during startup.
    """
    with open(HOME_FILE) as f:
        data = json.load(f)

    avatar_nodes = []
    for node_id in AVATAR_NODE_IDS:
        entry = data[str(node_id)]
        avatar_nodes.append(AvatarNode(
            protocol    = canopen_protocol,
            node_id     = node_id,
            name        = entry["name"],
            gear_ratio  = entry["gear_ratio"],
            encoder_cpr = entry["encoder_cpr"],
            home_counts = entry["encoder_home"],
        ))

    cubemars_nodes = []
    for node_id in CUBEMARS_NODE_IDS:
        entry = data[str(node_id)]
        cubemars_nodes.append(CubeMarsNode(
            protocol   = cubemars_protocol,
            node_id    = node_id,
            name       = entry["name"],
            gear_ratio = entry["gear_ratio"],
            home_deg   = entry["encoder_home"],
        ))

    return avatar_nodes, cubemars_nodes

# ============================================================
# CUBEMARS STARTUP
# ============================================================

def start_cubemars_holding(cubemars_nodes):
    """
    Enable all CubeMars motors.

    Returns True if all motors responded, False if any failed.
    """
    print("  Pre-enabling CubeMars motors...")
    all_ok = True

    for node in cubemars_nodes:
        ok = node.start_holding()
        if not ok:
            print(f"  WARNING: {node.name} (motor {node.node_id})"
                  f" failed to enable — will not hold")
            all_ok = False
            continue

        print(f"  Motor {node.node_id} ({node.name})"
              f" holding at {node.position:.1f} deg")

    print()
    return all_ok

# ============================================================
# COMMAND PARSER
# ============================================================

def parse_command(line: str):
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
        args = parts[1:]
        if len(args) < 2 or len(args) % 2 != 0:
            raise ValueError(
                "Usage: move <node_id> <degrees> "
                "[<node_id> <degrees> ...]"
            )
        try:
            targets = {
                int(args[i]): float(args[i + 1])
                for i in range(0, len(args), 2)
            }
        except ValueError:
            raise ValueError(
                "Usage: move <node_id> <degrees> "
                "[<node_id> <degrees> ...]"
            )
        return Command(type="MOVE", targets=targets)

    elif t == "moveto":
        args = parts[1:]
        if len(args) < 2 or len(args) % 2 != 0:
            raise ValueError(
                "Usage: moveto <node_id> <degrees> "
                "[<node_id> <degrees> ...]"
            )
        try:
            targets = {
                int(args[i]): float(args[i + 1])
                for i in range(0, len(args), 2)
            }
        except ValueError:
            raise ValueError(
                "Usage: moveto <node_id> <degrees> "
                "[<node_id> <degrees> ...]"
            )
        return Command(type="MOVETO", targets=targets)

    else:
        raise ValueError(f"Unknown command: {t!r}")


# ============================================================
# KEYBOARD INPUT THREAD
# ============================================================

def input_thread(cmd_queue: queue.Queue, stop_event: threading.Event):
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
    node_parts = []
    for ns in status.nodes:
        deg = f"{ns.degrees:.1f}°" if ns.degrees is not None else "?"
        node_parts.append(f"{ns.name}={deg}")

    fault = (f"  FAULT: {status.fault_reason}"
             if status.fault_reason else "")

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
    print(f"  Avatar nodes   : {AVATAR_NODE_IDS}")
    print(f"  CubeMars motors: {CUBEMARS_NODE_IDS}")
    print(f"  Tick rate      : {int(1/SYNC_INTERVAL)} Hz"
          f"  ({SYNC_INTERVAL*1000:.0f}ms)")
    print(f"  Home file      : {HOME_FILE}")
    print()

    # --------------------------------------------------------
    # Open CAN bus: main bus for all protocol I/O,
    # --------------------------------------------------------
    main_bus = CanBus()

    try:
        main_bus.open()
    except Exception as e:
        print(f"  ERROR: Could not open CAN bus: {e}")
        return

    canopen_protocol  = CanOpenProtocol(main_bus)
    cubemars_protocol = CubeMarsProtocol(main_bus)

    # --------------------------------------------------------
    # Load nodes
    # --------------------------------------------------------
    try:
        avatar_nodes, cubemars_nodes = load_nodes(
            canopen_protocol, cubemars_protocol
        )
    except Exception as e:
        print(f"  ERROR: Could not load home positions: {e}")
        main_bus.close()
        return

    cubemars_startup_ok = start_cubemars_holding(cubemars_nodes)

    # --------------------------------------------------------
    # Build state machine with all nodes
    # --------------------------------------------------------
    all_nodes = avatar_nodes + cubemars_nodes
    sm        = ArmStateMachine(all_nodes, canopen_protocol)

    if not cubemars_startup_ok:
        sm.handle_command(Command(
            type="ESTOP",
            reason="One or more CubeMars motors failed startup initialization",
        ))

    # --------------------------------------------------------
    # Keyboard input thread
    # --------------------------------------------------------
    cmd_queue  = queue.Queue()
    stop_event = threading.Event()

    input_t = threading.Thread(
        target = input_thread,
        args   = (cmd_queue, stop_event),
        daemon = True,
    )
    input_t.start()

    # --------------------------------------------------------
    # Tick loop
    # --------------------------------------------------------
    print(f"  State: {sm.state.name}")
    print()

    running = True

    try:
        while running:
            tick_start = time.monotonic()

            # 1. Read one pending command
            cmd = None
            try:
                cmd = cmd_queue.get_nowait()
            except queue.Empty:
                pass

            if cmd and cmd.type == "QUIT":
                print("  Quitting...")
                running = False
                break

            if cmd and cmd.type == "STATUS":
                print_status(sm.status())
                cmd = None

            # 2. Hand command to state machine
            sm.handle_command(cmd)

            # 3. Tick state machine
            sm.tick()

            # 4. Heartbeat print
            # if sm.status().tick % STATUS_PRINT_INTERVAL == 0:
            #     print_status(sm.status())

            # 5. Sleep remainder of tick
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

        # Disable Avatar nodes via state machine
        if sm.state not in (ArmState.DISABLED, ArmState.FAULT):
            sm.handle_command(Command(type="DISABLE"))
        elif sm.state == ArmState.FAULT:
            for node in avatar_nodes:
                try:
                    node.disable()
                except Exception:
                    pass

        # Disable CubeMars motors
        for node in cubemars_nodes:
            try:
                node.disable()
                node.release_torque()
            except Exception as exc:
                print(
                    f"Failed to release torque on "
                    f"{node.name}: {exc}"
                )

        main_bus.close()
        print("  Done.")
        print()


if __name__ == "__main__":
    main()