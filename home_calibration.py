"""
home_arm.py
===========
Interactive homing script for the rover arm.

For each node the operator jogs the joint to its physical
home position using the keyboard, then confirms.  The encoder
count at that moment is saved to home_positions.json.

Controls during jog
-------------------
  +   or  =   : step positive
  -           : step negative
  f           : toggle fine / coarse step size
  s           : cycle speed  (slow → medium → fast)
  [           : decrease step size by 5000 counts
  ]           : increase step size by 5000 counts
  q           : confirm — save this position as home
  x           : abort this node (skip without saving)
  CTRL-C      : abort entire homing run

How jogging works
-----------------
We use the same proven RPDO1 + SYNC pattern from the working
interpolation test script.  Each keypress runs a small
move_interpolated burst toward the new target.  Position is
tracked mathematically — we do NOT read back from the TPDO
mid-jog because the TPDO update rate on these drives is too
slow (~10 Hz) and returns stale values.

Position is only read from the TPDO once: at startup, to get
the initial encoder count so our math starts from a real value.

home_positions.json schema
--------------------------
{
    "<node_id>": {
        "name":         "<joint name>",
        "encoder_home": <int>,
        "gear_ratio":   <int>,
        "encoder_cpr":  <int>,
        "timestamp":    "<ISO-8601>"
    },
    ...
}

The ROS2 bridge on the Jetson reads this file at startup.
All relative moves are then:

    target_encoder = encoder_home + degrees * gear_ratio * CPR / 360
"""

import can
import json
import os
import struct
import sys
import termios
import time
import tty
from datetime import datetime, timezone


# ============================================================
# CAN BUS
# ============================================================

bus = can.Bus(
    channel="can0",
    interface="socketcan"
)


# ============================================================
# MOTOR CONFIGURATION
# ============================================================

ENCODER_CPR = 32768

NODE_CONFIG = {
    1: {"name": "Base",         "gear_ratio": 100},
    4: {"name": "Gripper Roll", "gear_ratio": 50},
    5: {"name": "Wrist Pitch",  "gear_ratio": 50},
    6: {"name": "Wrist Roll",   "gear_ratio": 50},
}

# Homing order — change if mechanical sequencing matters
HOMING_ORDER = [1, 4, 5, 6]

# Where home positions are persisted
HOME_FILE = "/home/pi/pi_contrller/home_positions.json"

# Jog step sizes in encoder counts
COARSE_STEP = 80000   # ~8.8 deg on 100:1 / ~17.6 deg on 50:1
FINE_STEP   = 500     # ~0.055 deg on 100:1 / ~0.11 deg on 50:1

# Interpolation timing
SYNC_INTERVAL = 0.020   # seconds between SYNCs — do not change

# Waypoint step sizes by speed level (encoder counts per tick).
# Higher = faster angular velocity.
SPEED_LEVELS = {
    "slow":   250,
    "medium": 500,
    "fast":   1000,
}
SPEED_ORDER = ["slow", "medium", "fast"]   # cycle order for 's' key

# Step size adjustment via [ and ] keys
STEP_INCREMENT = 5000
STEP_MIN       = 500
STEP_MAX       = 200000


# ============================================================
# SDO WRITE
# ============================================================

def sdo_write(node_id, index, subindex, value, size=4):

    cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]

    data = bytes([
        cmd,
        index & 0xFF,
        (index >> 8) & 0xFF,
        subindex,
    ]) + value.to_bytes(4, "little", signed=(value < 0))

    bus.send(can.Message(
        arbitration_id=0x600 + node_id,
        data=data,
        is_extended_id=False,
    ))

    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == (0x580 + node_id):
            return msg.data[0] == 0x60

    return False


# ============================================================
# SYNC
# ============================================================

def send_sync():
    bus.send(can.Message(
        arbitration_id=0x080,
        data=[],
        is_extended_id=False,
    ))


# ============================================================
# RPDO1
# ============================================================

def send_rpdo1(node_id, controlword, mode, target_pos):
    data = struct.pack("<HBi", controlword, mode, target_pos)
    bus.send(can.Message(
        arbitration_id=0x200 + node_id,
        data=data,
        is_extended_id=False,
    ))


# ============================================================
# READ POSITION  (used only at startup)
# ============================================================

def read_initial_position(node_id, timeout=2.0):
    """
    Wait for a TPDO from node_id and return its position.

    Called once per node at enable time.  Not used during jogging
    because the TPDO update rate on these drives is ~10 Hz and
    returns stale values — too slow to track mid-move.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == (0x180 + node_id):
            pos = struct.unpack_from("<i", msg.data, 0)[0]
            sw  = struct.unpack_from("<H", msg.data, 4)[0]
            return pos, sw
    return None, None


# ============================================================
# ENABLE / DISABLE
# ============================================================

def enable_node(node_id):
    """Enable in interpolated position mode (mode 7)."""

    print(f"\n  Enabling node {node_id} ({NODE_CONFIG[node_id]['name']})...")

    sdo_write(node_id, 0x6060, 0x00, 7,      size=1)   # interpolated pos mode
    sdo_write(node_id, 0x6040, 0x00, 0x0006, size=2)   # shutdown
    time.sleep(0.05)
    sdo_write(node_id, 0x6040, 0x00, 0x0007, size=2)   # switch on
    time.sleep(0.05)
    sdo_write(node_id, 0x6040, 0x00, 0x000F, size=2)   # enable operation
    time.sleep(0.1)
    sdo_write(node_id, 0x6040, 0x00, 0x001F, size=2)   # enable interpolation
    time.sleep(0.1)

    pos, sw = read_initial_position(node_id)
    if pos is None:
        print(f"  Node {node_id}: FAILED to read initial position.")
        return None

    print(f"  Node {node_id} enabled | mode=7 | pos={pos} | sw=0x{sw:04X}")
    return pos


def disable_node(node_id):
    sdo_write(node_id, 0x6040, 0x00, 0x0006, size=2)
    print(f"  Node {node_id} disabled.")


# ============================================================
# JOG MOVE
# ============================================================

def jog_move(node_id, start_pos, target_pos, waypoint_step):
    """
    Move from start_pos to target_pos using the proven
    interpolation burst pattern.

    waypoint_step controls how many encoder counts are covered
    per SYNC tick, setting the angular velocity of the move.
    Position is NOT read back — the caller trusts the target.
    """
    delta     = target_pos - start_pos
    direction = 1 if delta > 0 else -1
    step      = direction * abs(waypoint_step)

    waypoints = []
    current   = start_pos
    while abs(target_pos - current) > abs(step):
        current += step
        waypoints.append(current)
    waypoints.append(target_pos)

    next_sync = time.monotonic()

    for waypoint in waypoints:
        send_rpdo1(node_id, 0x001F, 7, waypoint)
        next_sync += SYNC_INTERVAL
        remaining = next_sync - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        send_sync()

    # Hold at target for a few extra ticks so the drive doesn't
    # stall at the last waypoint
    for _ in range(3):
        send_rpdo1(node_id, 0x001F, 7, target_pos)
        next_sync += SYNC_INTERVAL
        remaining = next_sync - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        send_sync()


# ============================================================
# KEYBOARD (single character, no Enter required)
# ============================================================

def _getch():
    """Read one character from stdin without echoing or buffering."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def _flush_stdin():
    """
    Discard any keypresses that arrived while jog_move was running.

    Without this, holding a key or tapping rapidly queues up input
    that fires off as a burst of moves after the current one finishes.
    """
    import select
    while select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.read(1)


# ============================================================
# LOAD / SAVE HOME FILE
# ============================================================

def load_home_file():
    if os.path.exists(HOME_FILE):
        with open(HOME_FILE, "r") as f:
            data = json.load(f)
        return {int(k): v for k, v in data.items()}
    return {}


def save_home_file(homes):
    os.makedirs(os.path.dirname(HOME_FILE), exist_ok=True)
    serialisable = {str(k): v for k, v in homes.items()}
    with open(HOME_FILE, "w") as f:
        json.dump(serialisable, f, indent=4)
    print(f"\n  Saved → {HOME_FILE}")


# ============================================================
# HOME ONE NODE
# ============================================================

def home_node(node_id, homes):
    """
    Interactive homing for a single node.

    Returns True  if the operator confirmed a home position.
    Returns False if the operator chose to skip this node.
    Raises  on CAN / hardware errors.
    """
    cfg  = NODE_CONFIG[node_id]
    name = cfg["name"]

    print()
    print("=" * 56)
    print(f"  NODE {node_id}  —  {name}")
    print("=" * 56)

    if node_id in homes:
        prev = homes[node_id]["encoder_home"]
        print(f"  NOTE: existing home for this node = {prev}")
        print("  Confirming will overwrite it.")

    # --------------------------------------------------------
    # Enable and get starting position from TPDO
    # --------------------------------------------------------
    current_pos = enable_node(node_id)
    if current_pos is None:
        print("  Cannot enable node — skipping.")
        return False

    time.sleep(0.3)

    # --------------------------------------------------------
    # Jog loop
    # --------------------------------------------------------
    step_size    = COARSE_STEP
    fine_mode    = False
    speed_name   = "medium"
    waypoint_step = SPEED_LEVELS[speed_name]

    def _print_status():
        step_label = "FINE  " if fine_mode else "COARSE"
        print(f"  [{step_label}  {speed_name:6s}]"
              f"  step={step_size:6d}  speed={waypoint_step} counts/tick"
              f"  pos={current_pos:8d}")

    print()
    print("  JOG CONTROLS")
    print("  ------------")
    print("  +  /  =   step positive")
    print("  -         step negative")
    print("  f         toggle fine / coarse step")
    print("  s         cycle speed  (slow → medium → fast)")
    print("  [  /  ]   decrease / increase step by 5000 counts")
    print("  q         CONFIRM — save as home")
    print("  x         skip this node")
    print()
    _print_status()
    print()

    try:
        while True:

            ch = _getch()

            if ch in ("+", "="):
                target = current_pos + step_size
                jog_move(node_id, current_pos, target, waypoint_step)
                current_pos = target
                _flush_stdin()   # discard keys pressed during the move
                _print_status()

            elif ch == "-":
                target = current_pos - step_size
                jog_move(node_id, current_pos, target, waypoint_step)
                current_pos = target
                _flush_stdin()
                _print_status()

            elif ch == "f":
                fine_mode = not fine_mode
                step_size = FINE_STEP if fine_mode else COARSE_STEP
                _print_status()

            elif ch == "s":
                idx        = SPEED_ORDER.index(speed_name)
                speed_name = SPEED_ORDER[(idx + 1) % len(SPEED_ORDER)]
                waypoint_step = SPEED_LEVELS[speed_name]
                _print_status()

            elif ch == "[":
                step_size = max(STEP_MIN, step_size - STEP_INCREMENT)
                fine_mode = (step_size == FINE_STEP)
                _print_status()

            elif ch == "]":
                step_size = min(STEP_MAX, step_size + STEP_INCREMENT)
                fine_mode = False
                _print_status()

            elif ch == "q":
                homes[node_id] = {
                    "name":         name,
                    "encoder_home": current_pos,
                    "gear_ratio":   cfg["gear_ratio"],
                    "encoder_cpr":  ENCODER_CPR,
                    "timestamp":    datetime.now(timezone.utc).isoformat(),
                }
                print()
                print(f"  HOME SET  →  node {node_id} ({name})")
                print(f"              encoder_home = {current_pos}")
                disable_node(node_id)
                return True

            elif ch == "x":
                print(f"  Skipping node {node_id}.")
                disable_node(node_id)
                return False

            elif ch == "\x03":   # CTRL-C
                disable_node(node_id)
                raise KeyboardInterrupt

    except Exception:
        disable_node(node_id)
        raise


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    print()
    print("=" * 56)
    print("  ROVER ARM  —  HOMING SCRIPT")
    print("=" * 56)
    print(f"  Home file : {HOME_FILE}")
    print(f"  Nodes     : {HOMING_ORDER}")
    print()

    homes = load_home_file()
    saved_count   = 0
    skipped_nodes = []

    try:
        for node_id in HOMING_ORDER:

            if node_id not in NODE_CONFIG:
                print(f"  Node {node_id} not in NODE_CONFIG — skipping.")
                continue

            result = home_node(node_id, homes)

            if result:
                save_home_file(homes)
                saved_count += 1
            else:
                skipped_nodes.append(node_id)

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n  Homing aborted by operator.")

    finally:
        bus.shutdown()

    print()
    print("=" * 56)
    print("  HOMING COMPLETE")
    print("=" * 56)
    print(f"  Saved   : {saved_count} node(s)")
    if skipped_nodes:
        print(f"  Skipped : {skipped_nodes}")
    print()

    if homes:
        print("  Stored positions:")
        print()
        for nid, entry in homes.items():
            print(f"    Node {nid}  {entry['name']:<14}"
                  f"  home = {entry['encoder_home']:8d}"
                  f"  @ {entry['timestamp']}")
    print()