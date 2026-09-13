"""
test_cubemars_comms.py
======================
Minimal CubeMars communication test.

Enables feedback on each motor, reads one status frame,
and prints the result.  Sends NO motion commands.

Run:
    python test_cubemars_comms.py
"""

import time
from can_bus import CanBus
from can_protocol_cubemars import CubeMarsProtocol

MOTOR_IDS = [2, 3]   # shoulder, elbow

def hold_position(cm, drive_id, current_position_deg):
    import struct, can

    arb_id = (6 << 8) | drive_id

    pos_raw = int(current_position_deg * 10000.0)
    spd_raw = int(5000 / 10)    # 5000 ERPM, divided by 10 per protocol
    acc_raw = int(5000 / 10)    # 5000 ERPM/s², divided by 10 per protocol

    # Correct layout: int32 pos + int16 spd + int16 acc, all big-endian
    data = struct.pack('>ihh', pos_raw, spd_raw, acc_raw)

    print(f"  Sending: ID=0x{arb_id:08X}  "
          f"data=[{' '.join(f'{b:02X}' for b in data)}]")

    msg = can.Message(
        arbitration_id = arb_id,
        data           = data,
        is_extended_id = True,
    )
    cm._bus._bus.send(msg)

    # Listen for acknowledgement
    import time
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        reply = cm._bus._bus.recv(timeout=0.05)
        if reply and reply.is_extended_id:
            print(f"  Reply:   ID=0x{reply.arbitration_id:08X}  "
                  f"data=[{' '.join(f'{b:02X}' for b in reply.data)}]")

with CanBus() as bus:
    cm = CubeMarsProtocol(bus)

    for drive_id in MOTOR_IDS:
        print(f"\n--- Motor {drive_id} ---")

        print("  Sending feedback enable...")
        got_response = cm.enable_feedback(drive_id)

        if not got_response:
            print("  No response — motor may be off or wrong ID")
            continue

        print("  Response received. Waiting for status frame...")
        time.sleep(0.1)

        status = cm.read_status(drive_id, timeout=1.0)

        if status is None:
            print("  No status frame received")
        else:
            print(f"  position : {status['position']:.1f} deg")
            print(f"  speed    : {status['speed']:.0f} ERPM")
            print(f"  current  : {status['current']:.2f} A")
            print(f"  temp     : {status['temp']} C")
            print(f"  error    : {status['error']}  "
                  f"({'OK' if status['error'] == 0 else 'FAULT'})")

        status = cm.read_status(drive_id, timeout=1.0)
        if status:
            print(f"  Holding at {status['position']:.1f} deg")
            hold_position(cm, drive_id, status['position'])

