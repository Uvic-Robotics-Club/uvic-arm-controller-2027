"""
home_calibration.py
===============
Unified homing script for all rover arm motors.

Homes Avatar CANopen motors (nodes 1, 4, 5, 6) and CubeMars
motors (nodes 2, 3) in a single run, saving all results to
the same home_positions.json file.

Avatar motors  — use RPDO1 + SYNC interpolation pattern
CubeMars motors — use continuous position-velocity commands

Controls (same for both motor types)
-------------------------------------
  +  /  =  : step positive
  -         : step negative
  f         : toggle fine / coarse step
  s         : cycle speed
  [  /  ]   : decrease / increase step size
  q         : confirm home
  x         : skip this motor
  CTRL-C    : abort entire run
"""

import json
import os
import select
import struct
import sys
import termios
import threading
import time
import tty
from datetime import datetime, timezone

import can


# ============================================================
# CAN BUS
# ============================================================

# Main bus — used for all protocol operations and reads
bus = can.Bus(channel="can0", interface="socketcan")

# Hold bus — dedicated to the background hold manager
# Separate instance so send/recv on the main bus is never
# interrupted by the hold manager's continuous sends
hold_bus = can.Bus(channel="can0", interface="socketcan")


# ============================================================
# MOTOR CONFIGURATION
# ============================================================

AVATAR_NODES = {
    1: {"name": "Base",         "gear_ratio": 100, "encoder_cpr": 32768},
    4: {"name": "Gripper Roll", "gear_ratio": 50,  "encoder_cpr": 32768},
    5: {"name": "Wrist Pitch",  "gear_ratio": 50,  "encoder_cpr": 32768},
    6: {"name": "Wrist Roll",   "gear_ratio": 50,  "encoder_cpr": 32768},
}

CUBEMARS_NODES = {
    2: {"name": "Shoulder", "gear_ratio": 1},
    3: {"name": "Elbow",    "gear_ratio": 1},
}

# Homing order — interleave as needed for mechanical sequencing
HOMING_ORDER = [
    (1,  "avatar"),
    (2,  "cubemars"),
    (3,  "cubemars"),
    (4,  "avatar"),
    (5,  "avatar"),
    (6,  "avatar"),
]

HOME_FILE = "/home/pi/pi_controller/home_positions.json"


# ============================================================
# SHARED JOG PARAMETERS
# ============================================================

AVATAR_COARSE_STEP    = 80000
AVATAR_FINE_STEP      = 500
AVATAR_SYNC_INTERVAL  = 0.020
AVATAR_SPEED_LEVELS   = {"slow": 250, "medium": 500, "fast": 1000}

CUBEMARS_COARSE_STEP  = 5.0
CUBEMARS_FINE_STEP    = 0.5
CUBEMARS_SEND_INTERVAL = 0.020
CUBEMARS_SPEED_LEVELS = {"slow": 10.0, "medium": 30.0, "fast": 60.0}

SPEED_ORDER     = ["slow", "medium", "fast"]
STEP_INCREMENT  = {"avatar": 5000, "cubemars": 1.0}
STEP_MIN        = {"avatar": 500,  "cubemars": 0.5}
STEP_MAX        = {"avatar": 200000, "cubemars": 45.0}


# ============================================================
# CUBEMARS BACKGROUND HOLD
# ============================================================

class CubeMarsHoldManager:

    def __init__(self, hold_bus):
        self._bus        = hold_bus
        self._positions  = {}
        self._lock       = threading.Lock()
        self._stop_event = threading.Event()
        self._thread     = None

    def set_position(self, drive_id, pos_deg):
        with self._lock:
            self._positions[drive_id] = pos_deg

    def clear_position(self, drive_id):
        with self._lock:
            self._positions.pop(drive_id, None)

    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _send_pos_vel(self, drive_id, pos_deg,
                      speed_erpm=2000, accel_erpm=2000):
        """Send directly on hold_bus — never touches the main bus."""
        arb_id  = (6 << 8) | drive_id
        pos_raw = int(pos_deg * 10000.0)
        spd_raw = int(speed_erpm / 10)
        acc_raw = int(accel_erpm / 10)
        data    = struct.pack('>ihh', pos_raw, spd_raw, acc_raw)
        self._bus.send(can.Message(
            arbitration_id = arb_id,
            data           = data,
            is_extended_id = True,
        ))

    def _loop(self):
        next_send = time.monotonic()
        while not self._stop_event.is_set():
            with self._lock:
                positions = dict(self._positions)

            for drive_id, pos_deg in positions.items():
                try:
                    self._send_pos_vel(drive_id, pos_deg)
                except Exception as e:
                    print(f"  [HoldManager] Send failed: {e}")

            next_send += CUBEMARS_SEND_INTERVAL
            remaining  = next_send - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            else:
                next_send = time.monotonic()

# ============================================================
# AVATAR PROTOCOL
# ============================================================

def avatar_sdo_write(node_id, index, subindex, value, size=4):
    cmd  = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]
    data = bytes([
        cmd, index & 0xFF, (index >> 8) & 0xFF, subindex,
    ]) + value.to_bytes(4, "little", signed=(value < 0))
    bus.send(can.Message(
        arbitration_id=0x600 + node_id, data=data,
        is_extended_id=False,
    ))
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == (0x580 + node_id):
            return msg.data[0] == 0x60
    return False


def avatar_send_sync():
    bus.send(can.Message(
        arbitration_id=0x080, data=[], is_extended_id=False,
    ))


def avatar_send_rpdo1(node_id, controlword, mode, target_pos):
    import struct
    data = struct.pack("<HBi", controlword, mode, target_pos)
    bus.send(can.Message(
        arbitration_id=0x200 + node_id, data=data,
        is_extended_id=False,
    ))


def avatar_read_position(node_id, timeout=2.0):
    import struct
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == (0x180 + node_id):
            pos = struct.unpack_from("<i", msg.data, 0)[0]
            sw  = struct.unpack_from("<H", msg.data, 4)[0]
            return pos, sw
    return None, None


def avatar_enable(node_id):
    avatar_sdo_write(node_id, 0x6060, 0x00, 7,      size=1)
    avatar_sdo_write(node_id, 0x6040, 0x00, 0x0006, size=2)
    time.sleep(0.05)
    avatar_sdo_write(node_id, 0x6040, 0x00, 0x0007, size=2)
    time.sleep(0.05)
    avatar_sdo_write(node_id, 0x6040, 0x00, 0x000F, size=2)
    time.sleep(0.1)
    avatar_sdo_write(node_id, 0x6040, 0x00, 0x001F, size=2)
    time.sleep(0.1)
    pos, sw = avatar_read_position(node_id)
    return pos, sw


def avatar_disable(node_id):
    avatar_sdo_write(node_id, 0x6040, 0x00, 0x0006, size=2)


def avatar_jog_move(node_id, start_pos, target_pos, waypoint_step):
    import struct
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
    for wp in waypoints:
        avatar_send_rpdo1(node_id, 0x001F, 7, wp)
        next_sync += AVATAR_SYNC_INTERVAL
        rem = next_sync - time.monotonic()
        if rem > 0:
            time.sleep(rem)
        avatar_send_sync()

    for _ in range(3):
        avatar_send_rpdo1(node_id, 0x001F, 7, target_pos)
        next_sync += AVATAR_SYNC_INTERVAL
        rem = next_sync - time.monotonic()
        if rem > 0:
            time.sleep(rem)
        avatar_send_sync()


# ============================================================
# CUBEMARS PROTOCOL
# ============================================================

def cubemars_send_ext(arb_id, data):
    bus.send(can.Message(
        arbitration_id=arb_id, data=bytes(data),
        is_extended_id=True,
    ))


def cubemars_enable_feedback(drive_id):
    import struct
    arb_id = (16 << 8) | drive_id
    data   = bytes(6) + struct.pack('>H', 0x8000)
    cubemars_send_ext(arb_id, data)
    
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.05)
        if msg and msg.is_extended_id:
            recv_drive_id = msg.arbitration_id & 0xFF
            func_id       = (msg.arbitration_id >> 8) & 0xFFFFFF
            print(f"  [enable_feedback] drive_id={drive_id} "
                  f"got frame from drive={recv_drive_id} "
                  f"func=0x{func_id:02X}")
            if recv_drive_id == drive_id:
                return True
    print(f"  [enable_feedback] timed out waiting for drive {drive_id}")
    return False


def cubemars_read_position(drive_id, timeout=2.0):
    import struct
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg = bus.recv(timeout=min(remaining, 0.05))
        if msg is None:
            continue
        if not msg.is_extended_id:
            continue
        recv_drive_id = msg.arbitration_id & 0xFF
        func_id       = (msg.arbitration_id >> 8) & 0xFFFFFF
        print(f"  [read_position] want drive_id={drive_id} "
              f"got drive={recv_drive_id} func=0x{func_id:02X}")
        if recv_drive_id != drive_id:
            continue
        if func_id != 0x29:
            continue
        pos_raw = struct.unpack_from('>h', msg.data, 0)[0]
        pos     = pos_raw * 0.1
        print(f"  [read_position] drive {drive_id} pos={pos:.1f} deg")
        return pos
    print(f"  [read_position] timed out for drive {drive_id}")
    return None


def cubemars_send_pos_vel(drive_id, pos_deg,
                          speed_erpm=5000, accel_erpm=5000):
    import struct
    arb_id  = (6 << 8) | drive_id
    pos_raw = int(pos_deg * 10000.0)
    spd_raw = int(speed_erpm / 10)
    acc_raw = int(accel_erpm / 10)
    data    = struct.pack('>ihh', pos_raw, spd_raw, acc_raw)
    cubemars_send_ext(arb_id, data)


def cubemars_disable(drive_id):
    arb_id = (15 << 8) | drive_id
    cubemars_send_ext(arb_id, bytes(8))


def cubemars_jog_move(drive_id, target_deg, speed_dps):
    speed_erpm = int(abs(speed_dps) * 100)
    accel_erpm = speed_erpm * 2
    deadline   = time.monotonic() + 2.0
    next_send  = time.monotonic()
    while time.monotonic() < deadline:
        cubemars_send_pos_vel(drive_id, target_deg,
                              speed_erpm, accel_erpm)
        next_send += CUBEMARS_SEND_INTERVAL
        rem = next_send - time.monotonic()
        if rem > 0:
            time.sleep(rem)


# ============================================================
# KEYBOARD
# ============================================================

def _getch():
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def _flush_stdin():
    while select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.read(1)


# ============================================================
# LOAD / SAVE
# ============================================================

def load_home_file():
    if os.path.exists(HOME_FILE):
        with open(HOME_FILE) as f:
            data = json.load(f)
        return {int(k): v for k, v in data.items()}
    return {}


def save_home_file(homes):
    os.makedirs(os.path.dirname(HOME_FILE), exist_ok=True)
    with open(HOME_FILE, "w") as f:
        json.dump({str(k): v for k, v in homes.items()}, f, indent=4)
    print(f"\n  Saved -> {HOME_FILE}")


# ============================================================
# HOME AVATAR NODE
# ============================================================

def home_avatar_node(node_id, homes):
    cfg  = AVATAR_NODES[node_id]
    name = cfg["name"]

    print()
    print("=" * 56)
    print(f"  NODE {node_id}  —  {name}  [CANopen / Avatar]")
    print("=" * 56)

    if node_id in homes:
        print(f"  NOTE: existing home = {homes[node_id]['encoder_home']}")
        print("  Confirming will overwrite it.")

    print(f"\n  Enabling node {node_id}...")
    pos, sw = avatar_enable(node_id)
    if pos is None:
        print("  Failed to read initial position — skipping.")
        return False

    current_pos   = pos
    step_size     = AVATAR_COARSE_STEP
    fine_mode     = False
    speed_name    = "medium"
    waypoint_step = AVATAR_SPEED_LEVELS[speed_name]

    print(f"  Node {node_id} enabled | pos={pos} | sw=0x{sw:04X}")

    def _print_status():
        label = "FINE  " if fine_mode else "COARSE"
        print(f"  [{label}  {speed_name:6s}]"
              f"  step={step_size}  pos={current_pos}")

    _print_jog_help()
    _print_status()
    print()

    try:
        while True:
            ch = _getch()

            if ch in ("+", "="):
                target = current_pos + step_size
                avatar_jog_move(node_id, current_pos,
                                target, waypoint_step)
                current_pos = target
                _flush_stdin()
                _print_status()

            elif ch == "-":
                target = current_pos - step_size
                avatar_jog_move(node_id, current_pos,
                                target, waypoint_step)
                current_pos = target
                _flush_stdin()
                _print_status()

            elif ch == "f":
                fine_mode = not fine_mode
                step_size = (AVATAR_FINE_STEP if fine_mode
                             else AVATAR_COARSE_STEP)
                _print_status()

            elif ch == "s":
                idx        = SPEED_ORDER.index(speed_name)
                speed_name = SPEED_ORDER[(idx + 1) % 3]
                waypoint_step = AVATAR_SPEED_LEVELS[speed_name]
                _print_status()

            elif ch == "[":
                step_size = max(STEP_MIN["avatar"],
                                step_size - STEP_INCREMENT["avatar"])
                _print_status()

            elif ch == "]":
                step_size = min(STEP_MAX["avatar"],
                                step_size + STEP_INCREMENT["avatar"])
                _print_status()

            elif ch == "q":
                homes[node_id] = {
                    "name":         name,
                    "encoder_home": current_pos,
                    "gear_ratio":   cfg["gear_ratio"],
                    "encoder_cpr":  cfg["encoder_cpr"],
                    "timestamp":    _now(),
                }
                print(f"\n  HOME SET -> node {node_id} ({name})"
                      f" encoder_home={current_pos}")
                avatar_disable(node_id)
                return True

            elif ch == "x":
                print(f"  Skipping node {node_id}.")
                avatar_disable(node_id)
                return False

            elif ch == "\x03":
                avatar_disable(node_id)
                raise KeyboardInterrupt

    except Exception:
        avatar_disable(node_id)
        raise


# ============================================================
# HOME CUBEMARS NODE
# ============================================================

def home_cubemars_node(drive_id, homes, hold_manager):
    cfg  = CUBEMARS_NODES[drive_id]
    name = cfg["name"]

    print()
    print("=" * 56)
    print(f"  MOTOR {drive_id}  —  {name}  [CubeMars]")
    print("=" * 56)

    if drive_id in homes:
        print(f"  NOTE: existing home = "
              f"{homes[drive_id]['encoder_home']}")
        print("  Confirming will overwrite it.")

    # Check if already enabled and holding from pre-enable step
    with hold_manager._lock:
        current_pos = hold_manager._positions.get(drive_id)

    if current_pos is not None:
        print(f"  Motor {drive_id} already holding "
              f"at {current_pos:.1f} deg")
    else:
        # Not pre-enabled — try to enable now
        print(f"\n  Enabling motor {drive_id}...")
        if not cubemars_enable_feedback(drive_id):
            print("  No response — skipping.")
            return False

        current_pos = cubemars_read_position(drive_id, timeout=2.0)
        if current_pos is None:
            print("  Could not read position — skipping.")
            return False

        print(f"  Motor {drive_id} online | pos={current_pos:.1f} deg")
        hold_manager.set_position(drive_id, current_pos)
        time.sleep(0.1)

    step_size  = CUBEMARS_COARSE_STEP
    fine_mode  = False
    speed_name = "medium"
    speed_dps  = CUBEMARS_SPEED_LEVELS[speed_name]

    def _print_status():
        label = "FINE  " if fine_mode else "COARSE"
        print(f"  [{label}  {speed_name:6s}]"
              f"  step={step_size:.1f}deg"
              f"  speed={speed_dps:.0f}dps"
              f"  pos={current_pos:.1f}deg")

    _print_jog_help()
    _print_status()
    print()

    try:
        while True:
            ch = _getch()

            if ch in ("+", "="):
                hold_manager.clear_position(drive_id)
                target = current_pos + step_size
                cubemars_jog_move(drive_id, target, speed_dps)
                current_pos = target
                _flush_stdin()
                _print_status()
                hold_manager.set_position(drive_id, current_pos)

            elif ch == "-":
                hold_manager.clear_position(drive_id)
                target = current_pos - step_size
                cubemars_jog_move(drive_id, target, speed_dps)
                current_pos = target
                _flush_stdin()
                _print_status()
                hold_manager.set_position(drive_id, current_pos)

            elif ch == "f":
                fine_mode = not fine_mode
                step_size = (CUBEMARS_FINE_STEP if fine_mode
                             else CUBEMARS_COARSE_STEP)
                _print_status()

            elif ch == "s":
                idx        = SPEED_ORDER.index(speed_name)
                speed_name = SPEED_ORDER[(idx + 1) % 3]
                speed_dps  = CUBEMARS_SPEED_LEVELS[speed_name]
                _print_status()

            elif ch == "[":
                step_size = max(STEP_MIN["cubemars"],
                                step_size - STEP_INCREMENT["cubemars"])
                _print_status()

            elif ch == "]":
                step_size = min(STEP_MAX["cubemars"],
                                step_size + STEP_INCREMENT["cubemars"])
                _print_status()

            elif ch == "q":
                homes[drive_id] = {
                    "name":         name,
                    "encoder_home": current_pos,
                    "gear_ratio":   cfg["gear_ratio"],
                    "encoder_cpr":  None,
                    "timestamp":    _now(),
                }
                print(f"\n  HOME SET -> motor {drive_id} ({name})"
                      f" encoder_home={current_pos:.1f}")
                # DO NOT disable and DO NOT clear position —
                # motor stays held via hold_manager for all
                # subsequent nodes in the homing run
                return True

            elif ch == "x":
                print(f"  Skipping motor {drive_id}.")
                # DO NOT disable and DO NOT clear position —
                # motor stays held via hold_manager for all
                # subsequent nodes in the homing run
                return False

            elif ch == "\x03":
                cubemars_disable(drive_id)
                hold_manager.clear_position(drive_id)
                raise KeyboardInterrupt

    except Exception:
        cubemars_disable(drive_id)
        hold_manager.clear_position(drive_id)
        raise


# ============================================================
# SHARED HELPERS
# ============================================================

def _print_jog_help():
    print()
    print("  JOG CONTROLS")
    print("  ------------")
    print("  +  /  =   step positive")
    print("  -         step negative")
    print("  f         toggle fine / coarse step")
    print("  s         cycle speed  (slow -> medium -> fast)")
    print("  [  /  ]   decrease / increase step size")
    print("  q         CONFIRM — save as home")
    print("  x         skip this motor")
    print()


def _now():
    return datetime.now(timezone.utc).isoformat()


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    print()
    print("=" * 56)
    print("  ROVER ARM  —  UNIFIED HOMING")
    print("=" * 56)
    print(f"  Home file : {HOME_FILE}")
    print(f"  Order     : {[n for n, _ in HOMING_ORDER]}")
    print()

    homes         = load_home_file()
    saved_count   = 0
    skipped_nodes = []

    hold_manager = CubeMarsHoldManager(hold_bus)
    hold_manager.start()

    # --------------------------------------------------------
    # Pre-enable all CubeMars motors so they hold position
    # from the very start, even while Avatar nodes are being
    # homed first
    # --------------------------------------------------------
    print("  Pre-enabling CubeMars motors...")
    for node_id, motor_type in HOMING_ORDER:
        if motor_type != "cubemars":
            continue

        cfg  = CUBEMARS_NODES[node_id]
        name = cfg["name"]

        if not cubemars_enable_feedback(node_id):
            print(f"  WARNING: Motor {node_id} ({name}) "
                  f"did not respond — will not hold during homing")
            continue

        pos = cubemars_read_position(node_id, timeout=2.0)
        if pos is None:
            print(f"  WARNING: Motor {node_id} ({name}) "
                  f"position unreadable — will not hold during homing")
            continue

        hold_manager.set_position(node_id, pos)
        print(f"  Motor {node_id} ({name}) holding at {pos:.1f} deg")

    print()
    time.sleep(0.5)

    # --------------------------------------------------------
    # Main homing loop
    # --------------------------------------------------------
    try:
        for node_id, motor_type in HOMING_ORDER:
            if motor_type == "avatar":
                result = home_avatar_node(node_id, homes)
            else:
                result = home_cubemars_node(
                    node_id, homes, hold_manager
                )

            if result:
                save_home_file(homes)
                saved_count += 1
            else:
                skipped_nodes.append(node_id)

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n  Homing aborted by operator.")

    finally:
        with hold_manager._lock:
            still_held = list(hold_manager._positions.keys())
        for drive_id in still_held:
            print(f"  Disabling motor {drive_id}...")
            cubemars_disable(drive_id)

        hold_manager.stop()
        bus.shutdown()
        hold_bus.shutdown()

    print()
    print("=" * 56)
    print("  HOMING COMPLETE")
    print("=" * 56)
    print(f"  Saved   : {saved_count} motor(s)")
    if skipped_nodes:
        print(f"  Skipped : {skipped_nodes}")
    print()

    if homes:
        print("  Stored positions:")
        print()
        for nid, entry in homes.items():
            print(f"    Motor {nid}  {entry['name']:<14}"
                  f"  home = {entry['encoder_home']}"
                  f"  @ {entry['timestamp']}")
    print()