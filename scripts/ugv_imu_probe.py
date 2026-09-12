#!/usr/bin/env python3
"""Safe activity probe for the Waveshare UGV02 IMU fields.

This script sends ONLY the module selection command:
    {"T":4,"cmd":0}
so RoArm/gimbal data no longer overwrites T=1001 IMU fields. It sends no motor
commands. While it runs, slowly tilt and rotate the robot by hand. The script
checks whether gyro/accelerometer/magnetometer values become non-zero or change.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial is not installed. Install it with: sudo apt install python3-serial", file=sys.stderr)
    raise SystemExit(2)

FIELDS = ("gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz")


def send_json(ser: serial.Serial, obj: dict) -> None:
    payload = (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")
    ser.write(payload)
    ser.flush()
    print(f"TX: {obj}")


def as_float(value) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=8.0)
    args = parser.parse_args()

    if not (3.0 <= args.seconds <= 30.0):
        print("ERROR: --seconds must be between 3 and 30.", file=sys.stderr)
        return 2

    print("SAFE IMU TEST: no motor command will be sent.")
    print("During the test, slowly tilt and rotate the robot by hand.")
    print(f"Opening {args.port} at {args.baud} baud ...")

    samples = []
    t1005 = 0
    malformed = 0

    try:
        with serial.Serial(args.port, args.baud, timeout=0.15) as ser:
            ser.reset_input_buffer()
            send_json(ser, {"T": 4, "cmd": 0})
            started = time.monotonic()
            next_print = started

            while time.monotonic() - started < args.seconds:
                raw = ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                try:
                    msg = json.loads(text)
                except json.JSONDecodeError:
                    malformed += 1
                    continue

                if msg.get("T") == 1005:
                    t1005 += 1
                    continue
                if msg.get("T") != 1001:
                    continue

                sample = tuple(as_float(msg.get(name)) for name in FIELDS)
                samples.append(sample)

                now = time.monotonic()
                if now >= next_print:
                    print(
                        f"sample={len(samples):03d} "
                        f"gyro=({sample[0]:g},{sample[1]:g},{sample[2]:g}) "
                        f"acc=({sample[3]:g},{sample[4]:g},{sample[5]:g}) "
                        f"mag=({sample[6]:g},{sample[7]:g},{sample[8]:g})"
                    )
                    next_print = now + 0.5

    except PermissionError:
        print(f"ERROR: permission denied for {args.port}", file=sys.stderr)
        return 1
    except serial.SerialException as exc:
        print(f"ERROR: serial port problem: {exc}", file=sys.stderr)
        return 1

    print()
    if not samples:
        print(f"SUMMARY: T1001=0 T1005={t1005} malformed={malformed}")
        print("FAIL: no T=1001 base feedback was received.")
        return 1

    valid_columns = list(zip(*samples))
    ranges = []
    nonzero = False
    changed = False
    for name, values in zip(FIELDS, valid_columns):
        finite = [v for v in values if math.isfinite(v)]
        if not finite:
            ranges.append((name, math.nan, math.nan))
            continue
        lo = min(finite)
        hi = max(finite)
        ranges.append((name, lo, hi))
        if any(abs(v) > 1e-9 for v in finite):
            nonzero = True
        if hi - lo > 1e-9:
            changed = True

    print(f"SUMMARY: T1001={len(samples)} T1005={t1005} malformed={malformed}")
    print("RANGES:")
    for name, lo, hi in ranges:
        print(f"  {name}: {lo:g} .. {hi:g}")

    if nonzero and changed:
        print("OK: IMU fields are active and changed while the robot was moved.")
        return 0
    if nonzero:
        print("PARTIAL: IMU fields are non-zero but did not change during the test.")
        print("Repeat while clearly tilting/rotating the robot by hand.")
        return 1

    print("NO IMU DATA: all observed gyro/accel/mag fields remained zero.")
    print("The base, encoders, lidar and motion control can still be used without this IMU.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
