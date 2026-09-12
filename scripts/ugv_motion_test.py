#!/usr/bin/env python3
"""Guarded motion diagnostic for the Waveshare UGV02 lower controller.

Supports two firmware command paths:

  t1  -> {"T":1,"L":m/s,"R":m/s}
  t13 -> {"T":13,"X":m/s,"Z":rad/s}

The script can move the robot and is blocked unless --run is supplied. Run only
with all six drive wheels lifted clear of the ground. During the test it repeats
the selected command at 10 Hz, watches T=1001 feedback, then sends zero velocity
and T=0 before exiting.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial is not installed. Install it with: sudo apt install python3-serial", file=sys.stderr)
    raise SystemExit(2)


def send_json(ser: serial.Serial, obj: dict, *, quiet: bool = False) -> None:
    ser.write((json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8"))
    ser.flush()
    if not quiet:
        print(f"TX: {obj}")


def command_for(mode: str, speed: float) -> dict:
    if mode == "t1":
        return {"T": 1, "L": speed, "R": speed}
    return {"T": 13, "X": speed, "Z": 0.0}


def stop_command_for(mode: str) -> dict:
    if mode == "t1":
        return {"T": 1, "L": 0.0, "R": 0.0}
    return {"T": 13, "X": 0.0, "Z": 0.0}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mode", choices=("t1", "t13"), required=True)
    parser.add_argument("--speed", type=float, default=0.15, help="forward test speed in m/s")
    parser.add_argument("--duration", type=float, default=1.5, help="test duration in seconds")
    parser.add_argument("--run", action="store_true", help="required safety acknowledgement")
    args = parser.parse_args()

    if not args.run:
        print("BLOCKED: this test can move the robot.")
        print("Lift ALL six drive wheels clear of the ground and keep hands/cables away.")
        print("Then rerun with --run.")
        print()
        print(
            f"Example: python3 scripts/ugv_motion_test.py --mode {args.mode} "
            f"--speed {args.speed} --duration {args.duration} --run"
        )
        return 2

    if not (0.05 <= abs(args.speed) <= 0.25):
        print("ERROR: --speed magnitude must be between 0.05 and 0.25 m/s.", file=sys.stderr)
        return 2
    if not (0.5 <= args.duration <= 3.0):
        print("ERROR: --duration must be between 0.5 and 3.0 seconds.", file=sys.stderr)
        return 2

    print("WARNING: MOTOR TEST ENABLED. All six drive wheels must be off the ground.")
    print(f"Mode={args.mode} port={args.port} baud={args.baud} speed={args.speed:.3f}m/s duration={args.duration:.2f}s")

    cmd = command_for(args.mode, args.speed)
    stop_cmd = stop_command_for(args.mode)

    max_l = 0.0
    max_r = 0.0
    first_odom = None
    last_odom = None
    t1001_count = 0
    malformed = 0
    command_count = 0

    try:
        with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
            ser.reset_input_buffer()
            print(f"TX command: {cmd}")

            started = time.monotonic()
            next_tx = started
            deadline = started + args.duration

            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_tx:
                    send_json(ser, cmd, quiet=True)
                    command_count += 1
                    next_tx += 0.10

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

                if msg.get("T") != 1001:
                    continue

                t1001_count += 1
                try:
                    left = float(msg.get("L", 0.0))
                    right = float(msg.get("R", 0.0))
                    odl = float(msg.get("odl", 0.0))
                    odr = float(msg.get("odr", 0.0))
                except (TypeError, ValueError):
                    continue

                max_l = max(max_l, abs(left))
                max_r = max(max_r, abs(right))
                if first_odom is None:
                    first_odom = (odl, odr)
                last_odom = (odl, odr)
                print(f"RX BASE: L={left:.4f} R={right:.4f} odl={odl:g} odr={odr:g}")

            send_json(ser, stop_cmd)
            time.sleep(0.20)
            send_json(ser, {"T": 0})

    except KeyboardInterrupt:
        print("\nInterrupted. Attempting emergency stop.", file=sys.stderr)
        try:
            with serial.Serial(args.port, args.baud, timeout=0.1) as stop_ser:
                send_json(stop_ser, stop_cmd)
                send_json(stop_ser, {"T": 0})
        except Exception as exc:
            print(f"WARNING: could not send stop after interrupt: {exc}", file=sys.stderr)
        return 130
    except (PermissionError, serial.SerialException) as exc:
        print(f"ERROR: serial problem: {exc}", file=sys.stderr)
        return 1

    odom_changed = first_odom is not None and last_odom is not None and first_odom != last_odom

    print()
    print(
        f"SUMMARY: mode={args.mode} commands={command_count} T1001={t1001_count} "
        f"max|L|={max_l:.4f} max|R|={max_r:.4f} odom_changed={odom_changed} malformed={malformed}"
    )

    if max_l > 0.01 or max_r > 0.01 or odom_changed:
        print(f"OK: motion detected using {args.mode} command path.")
        return 0

    print(f"NO MOTION: no encoder movement detected using {args.mode} command path.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
