#!/usr/bin/env python3
"""Guarded functional test for Waveshare UGV02 CMD_ROS_CTRL (T=13).

This script DOES command the drive motors. It is intentionally blocked unless
--run is supplied. Run it only with all drive wheels safely lifted off the ground.

Test sequence:
  1. Open GPIO UART at 115200.
  2. Send emergency reset (T=999).
  3. Send a small forward ROS velocity command (T=13).
  4. Observe T=1001 wheel-speed / odometry feedback.
  5. Send zero velocity and then emergency stop (T=0) in a finally block.

The purpose is to prove that the firmware installed on this particular UGV02
supports the native ROS-style {T:13, X:m/s, Z:rad/s} command before building the
ROS 2 cmd_vel bridge around it.
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


def send_json(ser: serial.Serial, obj: dict) -> None:
    payload = (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")
    ser.write(payload)
    ser.flush()
    print(f"TX: {obj}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--speed", type=float, default=0.08, help="test linear speed in m/s")
    parser.add_argument("--duration", type=float, default=0.50, help="motion command duration in seconds")
    parser.add_argument("--run", action="store_true", help="required safety acknowledgement")
    args = parser.parse_args()

    if not args.run:
        print("BLOCKED: this test can move the robot.")
        print("Lift ALL drive wheels clear of the ground, keep hands/cables away from wheels,")
        print("then rerun with --run.")
        print()
        print(
            f"Example: python3 scripts/ugv_ros_ctrl_test.py --port {args.port} "
            f"--speed {args.speed} --duration {args.duration} --run"
        )
        return 2

    if not (0.01 <= abs(args.speed) <= 0.20):
        print("ERROR: for this diagnostic, --speed magnitude must be between 0.01 and 0.20 m/s.", file=sys.stderr)
        return 2
    if not (0.10 <= args.duration <= 2.0):
        print("ERROR: for this diagnostic, --duration must be between 0.10 and 2.0 seconds.", file=sys.stderr)
        return 2

    print("WARNING: MOTOR TEST ENABLED. All six drive wheels must be off the ground.")
    print(f"Opening {args.port} at {args.baud} baud ...")

    max_l = 0.0
    max_r = 0.0
    first_odom = None
    last_odom = None
    t1001_count = 0

    try:
        with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
            ser.reset_input_buffer()

            # Reset a possible previous emergency-stop latch.
            send_json(ser, {"T": 999})
            time.sleep(0.10)

            send_json(ser, {"T": 13, "X": args.speed, "Z": 0.0})
            deadline = time.monotonic() + args.duration

            while time.monotonic() < deadline:
                raw = ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace").strip()
                try:
                    msg = json.loads(text)
                except (json.JSONDecodeError, TypeError):
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

            # Normal stop before leaving the protected block.
            send_json(ser, {"T": 13, "X": 0.0, "Z": 0.0})
            time.sleep(0.15)

            # Leave the controller in emergency-stop state after the diagnostic.
            send_json(ser, {"T": 0})

    except KeyboardInterrupt:
        print("\nInterrupted by user. Attempting stop.", file=sys.stderr)
        try:
            with serial.Serial(args.port, args.baud, timeout=0.1) as stop_ser:
                send_json(stop_ser, {"T": 13, "X": 0.0, "Z": 0.0})
                send_json(stop_ser, {"T": 0})
        except Exception as exc:
            print(f"WARNING: could not send stop after interrupt: {exc}", file=sys.stderr)
        return 130
    except (PermissionError, serial.SerialException) as exc:
        print(f"ERROR: serial problem: {exc}", file=sys.stderr)
        return 1

    odom_changed = False
    if first_odom is not None and last_odom is not None:
        odom_changed = first_odom != last_odom

    print()
    print(
        f"SUMMARY: T1001={t1001_count}, max|L|={max_l:.4f}, max|R|={max_r:.4f}, "
        f"odom_changed={odom_changed}"
    )

    if max_l > 0.005 or max_r > 0.005 or odom_changed:
        print("OK: installed ESP32 firmware responds to T=13 ROS velocity control.")
        print("Controller was left in emergency-stop state (T=0).")
        return 0

    print("WARNING: no wheel motion was detected in feedback.")
    print("This does not prove T=13 is unsupported; the test speed may be below the motor/PID threshold.")
    print("Controller was left in emergency-stop state (T=0).")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
