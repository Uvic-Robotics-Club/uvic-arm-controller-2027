"""
can_listen.py
=============
Dump all CAN frames for 5 seconds, extended and standard.
"""

import can
import time

with can.Bus(channel="can0", interface="socketcan") as bus:
    print("Listening for 5 seconds — power cycle motor 3 now...")
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg:
            ext = "EXT" if msg.is_extended_id else "STD"
            data_hex = " ".join(f"{b:02X}" for b in msg.data)
            print(f"  [{ext}] ID=0x{msg.arbitration_id:08X}  data=[{data_hex}]")