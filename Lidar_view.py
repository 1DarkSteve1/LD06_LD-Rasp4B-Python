#!/usr/bin/env python3
"""
LD06 lidar -> live "radar" drawing in the terminal (works over SSH).

Run:
    python lidar_view.py --demo                  # fake room, no lidar needed (tests the display)
    python lidar_view.py --port /dev/serial0     # real lidar on the GPIO UART
    python lidar_view.py --port /dev/ttyUSB0     # real lidar on a USB-UART adapter

Options:
    --range 4      how many metres the outer edge of the circle represents
    --rotate 0     turn the picture by this many degrees (if your lidar is mounted rotated)
    --persist 0.5  seconds a point stays on screen before it is erased

Reading: the front of the lidar (the arrow on top) is UP, angles go clockwise.
Quit with Ctrl+C. Needs only: pip install pyserial
"""

import argparse
import math
import random
import shutil
import sys
import threading
import time

import serial

BAUD = 230400
HEADER = b"\x54\x2c"      # every packet starts with these two bytes
PACKET_LEN = 47           # header(2) + speed(2) + start(2) + 12 points(36) + end(2) + time(2) + crc(1)
POINTS_PER_PACKET = 12


# ---------------------------------------------------------------- CRC check
def _make_crc_table():
    """CRC-8, polynomial 0x4D (the one LDRobot uses), no reflection, start value 0."""
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return table


CRC_TABLE = _make_crc_table()


def crc8(data):
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


# ---------------------------------------------------------------- shared data
class Scan:
    """Latest distance for each whole degree (0-359), written by the reader thread."""

    def __init__(self):
        self.dist = [0] * 360        # millimetres, 0 = nothing
        self.stamp = [0.0] * 360     # when each degree was last updated
        self.speed = 0               # degrees per second
        self.packets = 0
        self.bad = 0                 # packets that failed the CRC check


def handle_packet(pkt, scan):
    scan.speed = int.from_bytes(pkt[2:4], "little")
    start = int.from_bytes(pkt[4:6], "little") / 100
    end = int.from_bytes(pkt[42:44], "little") / 100
    if end < start:
        end += 360
    step = (end - start) / (POINTS_PER_PACKET - 1)
    now = time.monotonic()
    for i in range(POINTS_PER_PACKET):
        o = 6 + 3 * i
        dist = int.from_bytes(pkt[o:o + 2], "little")   # mm
        if dist == 0:
            continue
        b = int(round((start + step * i) % 360)) % 360
        scan.dist[b] = dist
        scan.stamp[b] = now
    scan.packets += 1


def reader_loop(ser, scan, check_crc, stop):
    buf = bytearray()
    while not stop.is_set():
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf.extend(chunk)
        while True:
            i = buf.find(HEADER)
            if i < 0:
                del buf[:-1]          # keep the last byte, it might be the start of a header
                break
            if i:
                del buf[:i]           # throw away junk before the header
            if len(buf) < PACKET_LEN:
                break                 # wait for the rest of the packet
            pkt = bytes(buf[:PACKET_LEN])
            if not check_crc or crc8(pkt[:-1]) == pkt[-1]:
                del buf[:PACKET_LEN]
                handle_packet(pkt, scan)
            else:
                del buf[:1]           # false header: skip one byte and search again
                scan.bad += 1


# ---------------------------------------------------------------- demo source
def _ray_distance(deg, t):
    """Distance (m) from the centre of a fake room to its wall / a moving box, in direction deg."""
    a = math.radians(deg)
    dx, dy = math.sin(a), math.cos(a)           # 0 deg = up (+y), clockwise
    best = 1e9
    if abs(dx) > 1e-9:
        best = min(best, (1.5 if dx > 0 else -1.5) / dx)
    if abs(dy) > 1e-9:
        best = min(best, (2.0 if dy > 0 else -1.0) / dy)
    px, py, r = 0.8 * math.cos(t * 0.7), 0.8 + 0.6 * math.sin(t * 0.7), 0.2
    tp = dx * px + dy * py
    d2 = px * px + py * py - tp * tp
    if tp > 0 and d2 < r * r:
        best = min(best, tp - math.sqrt(r * r - d2))
    return best


def demo_loop(scan, stop):
    t0 = time.monotonic()
    while not stop.is_set():
        now = time.monotonic()
        for b in range(360):
            scan.dist[b] = int(_ray_distance(b, now - t0) * 1000) + random.randint(-15, 15)
            scan.stamp[b] = now
        scan.speed = 3600
        scan.packets += 37
        time.sleep(0.1)


# ---------------------------------------------------------------- drawing
# Each terminal character is a Braille cell = 2 dots wide x 4 dots tall,
# which gives much finer pictures than normal characters.
DOT_BITS = {(0, 0): 0x01, (0, 1): 0x02, (0, 2): 0x04, (1, 0): 0x08,
            (1, 1): 0x10, (1, 2): 0x20, (0, 3): 0x40, (1, 3): 0x80}


class Canvas:
    def __init__(self, cols, rows):
        self.cols, self.rows = cols, rows
        self.w, self.h = cols * 2, rows * 4      # size in dots
        self.pts = [[0] * cols for _ in range(rows)]    # lidar points (green)
        self.grid = [[0] * cols for _ in range(rows)]   # rings and markers (grey)

    def plot(self, layer, x, y):
        x, y = int(round(x)), int(round(y))
        if 0 <= x < self.w and 0 <= y < self.h:
            layer[y // 4][x // 2] |= DOT_BITS[(x % 2, y % 4)]

    def render(self):
        lines = []
        for r in range(self.rows):
            out, cur = [], None
            for c in range(self.cols):
                p, g = self.pts[r][c], self.grid[r][c]
                if p:
                    col, ch = "\x1b[92m", chr(0x2800 | p | g)
                elif g:
                    col, ch = "\x1b[90m", chr(0x2800 | g)
                else:
                    col, ch = None, " "
                if col != cur:
                    out.append(col or "\x1b[0m")
                    cur = col
                out.append(ch)
            lines.append("".join(out) + "\x1b[0m")
        return lines


def ring_step(max_range):
    if max_range <= 2:
        return 0.5
    if max_range <= 5:
        return 1.0
    if max_range <= 10:
        return 2.0
    return 4.0


def build_frame(scan, args, cols, rows, pps):
    plot_rows = max(rows - 3, 5)
    cv = Canvas(cols, plot_rows)
    cx, cy = cv.w / 2, cv.h / 2
    R = min(cv.w, cv.h) / 2 - 2                 # radius of the outer circle, in dots

    # distance rings
    step = ring_step(args.range)
    n = 1
    while n * step <= args.range + 1e-9:
        rr = R * n * step / args.range
        samples = max(24, int(rr * 2))
        for k in range(samples):
            a = 2 * math.pi * k / samples
            cv.plot(cv.grid, cx + rr * math.sin(a), cy - rr * math.cos(a))
        n += 1
    for t in range(int(R * 0.12)):              # little tick pointing to the front (up)
        cv.plot(cv.grid, cx, cy - t)

    # lidar points
    now = time.monotonic()
    rot = math.radians(args.rotate)
    count = 0
    for b in range(360):
        d = scan.dist[b]
        if d == 0 or now - scan.stamp[b] > args.persist:
            continue
        m = d / 1000
        if m > args.range:
            continue
        r = R * m / args.range
        a = math.radians(b) + rot
        x, y = cx + r * math.sin(a), cy - r * math.cos(a)
        cv.plot(cv.pts, x, y)
        cv.plot(cv.pts, x + 1, y)
        count += 1

    head = (f"LD06 | edge {args.range:g} m | rings {ring_step(args.range):g} m | "
            f"{scan.speed / 360:.1f} rev/s | {pps} pkt/s | {count} pts | bad CRC {scan.bad}")
    status = "" if scan.packets else "No data yet: check wiring, port name and that the lidar is powered"
    foot = "Front = up, angles go clockwise. Ctrl+C to quit."
    return [head[:cols], status[:cols]] + cv.render() + [foot[:cols]]


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="Live terminal view of an LD06 lidar")
    ap.add_argument("--port", default="/dev/serial0", help="serial port (default /dev/serial0)")
    ap.add_argument("--range", type=float, default=4.0, help="metres shown at the outer circle")
    ap.add_argument("--rotate", type=float, default=0.0, help="rotate the picture (degrees)")
    ap.add_argument("--persist", type=float, default=0.5, help="seconds a point stays visible")
    ap.add_argument("--demo", action="store_true", help="fake data, no lidar needed")
    ap.add_argument("--skip-crc", action="store_true", help="do not check packet checksums")
    args = ap.parse_args()

    scan = Scan()
    stop = threading.Event()

    if args.demo:
        target, targs = demo_loop, (scan, stop)
    else:
        try:
            ser = serial.Serial(args.port, BAUD, timeout=1)
        except serial.SerialException as e:
            print(f"Could not open {args.port}: {e}")
            print("Check the port name, that the UART is enabled, and that you are in the 'dialout' group.")
            sys.exit(1)
        target, targs = reader_loop, (ser, scan, not args.skip_crc, stop)

    threading.Thread(target=target, args=targs, daemon=True).start()

    sys.stdout.write("\x1b[?1049h\x1b[?25l\x1b[2J")   # separate screen, hide cursor
    last_size, last_packets, last_t = None, 0, time.monotonic()
    pps = 0
    try:
        while True:
            size = shutil.get_terminal_size()
            cols, rows = max(size.columns - 1, 20), max(size.lines, 8)
            if (cols, rows) != last_size:
                sys.stdout.write("\x1b[2J")
                last_size = (cols, rows)

            now = time.monotonic()
            if now - last_t >= 1.0:
                pps = int((scan.packets - last_packets) / (now - last_t))
                last_packets, last_t = scan.packets, now

            frame = build_frame(scan, args, cols, rows, pps)
            sys.stdout.write("\x1b[H" + "\n".join(frame))
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        sys.stdout.write("\x1b[?25h\x1b[?1049l")      # restore cursor and normal screen
        sys.stdout.flush()


if __name__ == "__main__":
    main()