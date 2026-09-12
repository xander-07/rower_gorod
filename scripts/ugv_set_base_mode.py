#!/usr/bin/env python3
"""Switch Waveshare UGV02 moduleType to base-only mode and inspect feedback.

This script does NOT command the drive motors. It sends only:
    {"T":4,"cmd":0}
which selects moduleType=0 (base only / no RoArm / no gimbal) in RAM, then listens
for T=1001 feedback. The setting is not persisted by this command alone.
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
    parser.add_argument("--seconds", type=float, default=4.0)
    args = parser.parse_args()

    if not (1.0 <= args.seconds <= 15.0):
        print("ERROR: --seconds must be between 1 and 15.", file=sys.stderr)
        return 2

    print("Safe configuration test: no motor command will be sent.")
    print(f"Opening {args.port} at {args.baud} baud ...")

    t1001 = 0
    t1005 = 0
    other = 0
    malformed = 0
    first_acc = None
    last_acc = None

    try:
        with serial.Serial(args.port, args.baud, timeout=0.25) as ser:
            ser.reset_input_buffer()
            send_json(ser, {"T": 4, "cmd": 0})
            started = time.monotonic()

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
                    print(f"RAW: {text[:160]}")
                    continue

                msg_type = msg.get("T")
                if msg_type == 1001:
                    t1001 += 1
                    acc = (msg.get("ax"), msg.get("ay"), msg.get("az"))
                    if first_acc is None:
                        first_acc = acc
                    last_acc = acc
                    print(
                        "BASE "
                        f"L={msg.get('L')} R={msg.get('R')} "
                        f"odl={msg.get('odl')} odr={msg.get('odr')} "
                        f"v={msg.get('v')} "
                        f"gyro=({msg.get('gx')},{msg.get('gy')},{msg.get('gz')}) "
                        f"raw_acc=({msg.get('ax')},{msg.get('ay')},{msg.get('az')}) "
                        f"mag=({msg.get('mx')},{msg.get('my')},{msg.get('mz')})"
                    )
                elif msg_type == 1005:
                    t1005 += 1
                    print(f"SERVO T=1005: {msg}")
                else:
                    other += 1
                    print(f"JSON T={msg_type}: {msg}")

    except PermissionError:
        print(f"ERROR: permission denied for {args.port}", file=sys.stderr)
        return 1
    except serial.SerialException as exc:
        print(f"ERROR: serial port problem: {exc}", file=sys.stderr)
        return 1

    print()
    print(
        f"SUMMARY: T1001={t1001} T1005={t1005} other={other} malformed={malformed} "
        f"first_raw_acc={first_acc} last_raw_acc={last_acc}"
    )
    if t1001 == 0:
        print("WARNING: no base feedback was observed.")
        return 1

    print("OK: moduleType=0 command was sent and base feedback is present.")
    print("Note: ax/ay/az are raw IMU values in this firmware, not SI units yet.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
