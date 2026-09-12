#!/usr/bin/env python3
"""Read-only probe for the Waveshare UGV02 lower controller on Raspberry Pi 5.

This tool opens the GPIO UART and listens for newline-delimited JSON feedback from
ESP32. It never writes commands, so it cannot intentionally move the robot.
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--max-lines", type=int, default=50)
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud ...")
    print("Read-only probe: no commands will be sent to the UGV02 controller.")

    valid_json = 0
    base_feedback = 0
    other_json = 0
    malformed = 0
    started = time.monotonic()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.25) as ser:
            while (time.monotonic() - started) < args.seconds and valid_json < args.max_lines:
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

                valid_json += 1
                msg_type = msg.get("T")
                if msg_type == 1001:
                    base_feedback += 1
                    print(
                        "BASE "
                        f"L={msg.get('L')} R={msg.get('R')} "
                        f"odl={msg.get('odl')} odr={msg.get('odr')} "
                        f"v={msg.get('v')} "
                        f"gyro=({msg.get('gx')},{msg.get('gy')},{msg.get('gz')}) "
                        f"acc=({msg.get('ax')},{msg.get('ay')},{msg.get('az')})"
                    )
                else:
                    other_json += 1
                    print(f"JSON T={msg_type}: {msg}")

    except PermissionError:
        print(
            f"ERROR: permission denied for {args.port}. Add rower to dialout and log in again: "
            "sudo usermod -aG dialout rower",
            file=sys.stderr,
        )
        return 1
    except serial.SerialException as exc:
        print(f"ERROR: serial port problem: {exc}", file=sys.stderr)
        return 1

    elapsed = time.monotonic() - started
    print(
        f"SUMMARY: elapsed={elapsed:.2f}s valid_json={valid_json} "
        f"base_feedback_T1001={base_feedback} other_json={other_json} malformed={malformed}"
    )

    if base_feedback > 0:
        print("OK: Waveshare base feedback is present on the GPIO UART.")
        return 0

    print("WARNING: no T=1001 base feedback was observed. No commands were sent.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
