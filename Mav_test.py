#!/usr/bin/env python3
"""
Talk to the Cube Orange over MAVLink from the Raspberry Pi.

What it does:
  1. opens the connection to the flight controller (FC)
  2. waits for the FC's HEARTBEAT (this also tells us the FC's system id)
  3. sends the Pi's own HEARTBEAT once per second ("I'm the companion computer, I'm alive")
  4. prints every FC heartbeat's mode / armed state, and every text message the FC sends
     (e.g. "PreArm: ..." errors), and warns if the FC's heartbeat stops

Run (inside your venv):  pip install pymavlink pyserial
    python mav_test.py --device /dev/ttyACM0                    # Cube plugged into a Pi USB port
    python mav_test.py --device /dev/serial0 --baud 921600      # Cube TELEM2 wired to the Pi's GPIO UART
Quit with Ctrl+C.
"""

import argparse
import os
import sys
import time

os.environ["MAVLINK20"] = "1"   # use MAVLink 2, the version ArduPilot talks by default
from pymavlink import mavutil

ap = argparse.ArgumentParser(description="MAVLink heartbeat test")
ap.add_argument("--device", default="/dev/ttyACM0", help="serial device of the Cube")
ap.add_argument("--baud", type=int, default=115200, help="baud rate (ignored for USB)")
args = ap.parse_args()

mav = mavutil.mavlink
print(f"Opening {args.device} at {args.baud} baud ...")
m = mavutil.mavlink_connection(
    args.device,
    baud=args.baud,
    source_system=1,                                  # our own system id
    source_component=mav.MAV_COMP_ID_ONBOARD_COMPUTER,  # 191 = onboard computer
)

print("Waiting for the flight controller's heartbeat (15 s) ...")
hb = m.wait_heartbeat(timeout=15)
if hb is None:
    print("No heartbeat received. Check: cable/wiring, device name, baud rate, SERIAL port settings on the Cube.")
    sys.exit(1)

print(f"Connected: system {m.target_system}, component {hb.get_srcComponent()}, "
      f"vehicle type {hb.type}, autopilot {hb.autopilot}")

last_sent = 0.0
last_fc_hb = time.time()
warned = False

try:
    while True:
        now = time.time()

        # 1) send OUR heartbeat once per second
        if now - last_sent >= 1.0:
            m.mav.heartbeat_send(
                mav.MAV_TYPE_ONBOARD_CONTROLLER,   # what we are
                mav.MAV_AUTOPILOT_INVALID,         # we are not an autopilot
                0, 0,
                mav.MAV_STATE_ACTIVE,
            )
            last_sent = now

        # 2) listen for messages (wait at most 0.2 s so the loop keeps running)
        msg = m.recv_match(blocking=True, timeout=0.2)

        if msg is not None and msg.get_srcSystem() == m.target_system:
            kind = msg.get_type()
            if kind == "HEARTBEAT" and msg.get_srcComponent() == mav.MAV_COMP_ID_AUTOPILOT1:
                last_fc_hb = now
                if warned:
                    print("FC heartbeat is back.")
                    warned = False
                armed = "ARMED" if m.motors_armed() else "disarmed"
                print(f"FC heartbeat | mode {m.flightmode} | {armed}")
            elif kind == "STATUSTEXT":
                print(f"FC says: {msg.text}")

        # 3) heartbeat watchdog: the FC normally sends 1 per second
        if now - last_fc_hb > 3.0 and not warned:
            print("WARNING: no heartbeat from the FC for 3 s (link lost?)")
            warned = True

except KeyboardInterrupt:
    print("\nBye.")