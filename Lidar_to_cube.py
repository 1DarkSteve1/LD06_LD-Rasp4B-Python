#!/usr/bin/env python3
"""
Lidar -> Raspberry Pi -> Cube Orange bridge.

  LD06 lidar --(UART)--> this script on the Pi --(MAVLink)--> Cube Orange (ArduRover)

What it does, 10 times per second:
  1. takes the latest distance for each degree from the lidar (reader code from lidar_view.py)
  2. shrinks the 360 degrees into 72 sectors of 5 degrees (closest object in each sector, in cm)
  3. sends them to the Cube as an OBSTACLE_DISTANCE message
It also sends the Pi's own heartbeat once per second and prints what the Cube says.

Needs lidar_view.py in the same folder.   pip install pymavlink pyserial

Examples:
  python lidar_to_cube.py --lidar /dev/ttyUSB0 --no-fc              # lidar only, print the sectors
  python lidar_to_cube.py --demo --fc /dev/ttyACM0                  # fake lidar -> Cube (tests the MAVLink part)
  python lidar_to_cube.py --lidar /dev/ttyUSB0 --fc /dev/serial0 --baud 921600   # the real setup
"""

import argparse
import os
import sys
import threading
import time

os.environ["MAVLINK20"] = "1"   # MAVLink 2 is required for OBSTACLE_DISTANCE
import serial
from pymavlink import mavutil

import Lidar_view as lv      # reuses Scan, reader_loop (packet reading + CRC) and demo_loop

mav = mavutil.mavlink
UNKNOWN = 65535              # MAVLink: "no information for this sector"
SECTORS = 72                 # OBSTACLE_DISTANCE always holds 72 values
INCREMENT = 5                # degrees per sector (72 x 5 = 360)


def build_sectors(scan, persist, rotate, min_cm, max_cm):
    """Turn the 360 per-degree distances (mm) into 72 sector distances (cm)."""
    now = time.monotonic()
    per_degree = [None] * 360                      # distance in cm for each degree, vehicle frame
    for b in range(360):
        d = scan.dist[b]
        if d == 0 or now - scan.stamp[b] > persist:
            continue                               # no echo, or too old
        cm = d // 10                               # mm -> cm
        if cm < min_cm or cm > max_cm:
            continue
        per_degree[(b + rotate) % 360] = cm        # lidar angle -> car angle

    sectors = []
    for s in range(SECTORS):
        centre = s * INCREMENT                     # sector s is centred on s*5 degrees (0 = front)
        vals = [per_degree[(centre + k) % 360] for k in range(-2, 3)]
        vals = [v for v in vals if v is not None]
        sectors.append(min(vals) if vals else UNKNOWN)   # closest object wins
    return sectors


def describe(sectors):
    known = [(cm, i * INCREMENT) for i, cm in enumerate(sectors) if cm != UNKNOWN]
    if not known:
        return "no obstacles seen"
    cm, deg = min(known)
    return f"nearest {cm} cm at {deg} deg | {len(known)}/{SECTORS} sectors have data"


def main():
    ap = argparse.ArgumentParser(description="LD06 -> OBSTACLE_DISTANCE bridge")
    ap.add_argument("--lidar", default="/dev/ttyUSB0", help="serial port of the lidar")
    ap.add_argument("--fc", default="/dev/serial0", help="serial port of the Cube (or udpout:host:port)")
    ap.add_argument("--baud", type=int, default=921600, help="baud rate to the Cube (ignored for USB)")
    ap.add_argument("--rotate", type=int, default=0, help="degrees to add to lidar angles (if its arrow is not at the car's front)")
    ap.add_argument("--min-cm", type=int, default=10, help="ignore readings closer than this")
    ap.add_argument("--max-cm", type=int, default=1200, help="ignore readings farther than this (LD06: 12 m)")
    ap.add_argument("--persist", type=float, default=0.3, help="seconds before an old reading is dropped")
    ap.add_argument("--rate", type=float, default=10.0, help="messages per second to the Cube")
    ap.add_argument("--no-fc", action="store_true", help="do not talk to the Cube, only print the sectors")
    ap.add_argument("--demo", action="store_true", help="fake lidar data (no lidar needed)")
    args = ap.parse_args()

    # ---------------------------------------------------------- lidar side
    scan = lv.Scan()
    stop = threading.Event()
    if args.demo:
        threading.Thread(target=lv.demo_loop, args=(scan, stop), daemon=True).start()
        print("Using FAKE lidar data (demo room).")
    else:
        try:
            ser = serial.Serial(args.lidar, lv.BAUD, timeout=1)
        except serial.SerialException as e:
            print(f"Could not open the lidar on {args.lidar}: {e}")
            sys.exit(1)
        threading.Thread(target=lv.reader_loop, args=(ser, scan, True, stop), daemon=True).start()
        print(f"Reading lidar on {args.lidar}.")

    # ---------------------------------------------------------- Cube side
    m = None
    if not args.no_fc:
        print(f"Opening the Cube on {args.fc} ...")
        m = mavutil.mavlink_connection(
            args.fc, baud=args.baud,
            source_system=1, source_component=mav.MAV_COMP_ID_ONBOARD_COMPUTER)
        print("Waiting for the Cube's heartbeat (15 s) ...")
        if m.wait_heartbeat(timeout=15) is None:
            print("No heartbeat. Run mav_test.py first to fix the connection.")
            sys.exit(1)
        print(f"Connected to system {m.target_system}.")

    # ---------------------------------------------------------- main loop
    period = 1.0 / args.rate
    next_send = time.monotonic()
    last_hb = 0.0
    last_print = 0.0
    last_packets, last_packets_change = 0, time.monotonic()
    last_state = None
    lidar_ok = True

    try:
        while True:
            now = time.monotonic()

            # is the lidar still delivering packets?
            if scan.packets != last_packets:
                last_packets, last_packets_change = scan.packets, now
            alive = (now - last_packets_change) < 0.5
            if alive != lidar_ok:
                lidar_ok = alive
                print("Lidar data is back." if alive else
                      "WARNING: no lidar data. Not sending obstacle messages, so the Cube notices.")

            sectors = build_sectors(scan, args.persist, args.rotate, args.min_cm, args.max_cm)

            if m is not None:
                if now - last_hb >= 1.0:           # our own heartbeat, 1 per second
                    m.mav.heartbeat_send(mav.MAV_TYPE_ONBOARD_CONTROLLER,
                                         mav.MAV_AUTOPILOT_INVALID, 0, 0, mav.MAV_STATE_ACTIVE)
                    last_hb = now

                if alive:                          # the obstacle message
                    m.mav.obstacle_distance_send(
                        int(time.time() * 1e6),    # time_usec
                        mav.MAV_DISTANCE_SENSOR_LASER,
                        sectors,                   # 72 distances in cm
                        INCREMENT,                 # degrees between values
                        args.min_cm, args.max_cm,
                        0.0, 0.0,                  # increment_f, angle_offset (unused: 0 = front)
                        mav.MAV_FRAME_BODY_FRD)    # angles measured clockwise from the car's front

                while True:                        # read what the Cube sent us
                    msg = m.recv_match(blocking=False)
                    if msg is None:
                        break
                    if msg.get_srcSystem() != m.target_system:
                        continue
                    kind = msg.get_type()
                    if kind == "STATUSTEXT":
                        print(f"Cube says: {msg.text}")
                    elif kind == "HEARTBEAT" and msg.get_srcComponent() == mav.MAV_COMP_ID_AUTOPILOT1:
                        state = (m.flightmode, m.motors_armed())
                        if state != last_state:
                            print(f"Cube mode {state[0]}, {'ARMED' if state[1] else 'disarmed'}")
                            last_state = state

            if now - last_print >= 1.0:
                print(f"lidar {scan.packets} packets | {describe(sectors)}")
                last_print = now

            next_send += period
            time.sleep(max(0.0, next_send - time.monotonic()))
            if next_send < time.monotonic() - 1:   # fell far behind: resynchronise
                next_send = time.monotonic()

    except KeyboardInterrupt:
        print("\nBye.")
    finally:
        stop.set()


if __name__ == "__main__":
    main()