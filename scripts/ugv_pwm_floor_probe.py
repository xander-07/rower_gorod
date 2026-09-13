#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time

import serial


def send_json(ser: serial.Serial, payload: dict) -> None:
    ser.write((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
    ser.flush()


def finite(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def drain(ser: serial.Serial, rx: bytearray, samples: list[dict]) -> None:
    waiting = ser.in_waiting
    if waiting > 0:
        rx.extend(ser.read(waiting))

    while b"\n" in rx:
        raw, _, rest = rx.partition(b"\n")
        rx[:] = rest
        text = raw.decode("utf-8", errors="replace").strip()
        if not text:
            continue
        try:
            msg = json.loads(text)
        except json.JSONDecodeError:
            continue
        if msg.get("T") == 1001:
            samples.append(msg)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Guarded floor test using the Waveshare vendor CMD_PWM_INPUT command "
            "directly: T=11, L=<PWM>, R=<PWM>. This bypasses the lower-controller "
            "speed PID and all ROS cmd_vel/ramp logic."
        )
    )
    parser.add_argument("--run", action="store_true", help="Required to allow motion.")
    parser.add_argument("--port", default="/dev/rower_base")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--pwm", type=int, default=70, help="Equal left/right PWM, 30..120 (vendor range is +/-255).")
    parser.add_argument("--seconds", type=float, default=0.8, help="Motion duration, 0.3..1.5 s.")
    parser.add_argument("--rate", type=float, default=10.0, help="Command repeat rate, 5..20 Hz.")
    args = parser.parse_args()

    if not args.run:
        print("REFUSING TO MOVE: pass --run only after stopping ROS bringup/mapping and clearing the floor.")
        return 2
    if not (30 <= abs(args.pwm) <= 120):
        print("ERROR: |--pwm| must be in 30..120 for this guarded floor test.")
        return 2
    if not (0.3 <= args.seconds <= 1.5):
        print("ERROR: --seconds must be in 0.3..1.5 s.")
        return 2
    if not (5.0 <= args.rate <= 20.0):
        print("ERROR: --rate must be in 5..20 Hz.")
        return 2

    print("DIRECT WAVESHARE PWM FLOOR PROBE ENABLED.")
    print("This sends the vendor command T=11 directly to the ESP32 and bypasses its speed PID.")
    print("IMPORTANT: stop rower_bringup / mapping first so this script is the only UART owner.")
    print(f"Command: {{\"T\":11,\"L\":{args.pwm},\"R\":{args.pwm}}} for {args.seconds:.2f}s")
    print("The vendor protocol allows +/-255 PWM; this guarded test intentionally uses a lower value.")
    print("Use at least 1.0 m clear space. Be ready to cut robot power if it moves unexpectedly.")
    print("Starting in 3 seconds...")
    time.sleep(3.0)

    ser = serial.Serial(args.port, args.baud, timeout=0)
    rx = bytearray()
    samples: list[dict] = []
    motion_samples: list[dict] = []

    try:
        ser.reset_input_buffer()
        send_json(ser, {"T": 4, "cmd": 0})
        send_json(ser, {"T": 999})

        # Establish a clean stopped baseline while already in raw-PWM mode.
        for _ in range(4):
            send_json(ser, {"T": 11, "L": 0, "R": 0})
            time.sleep(0.08)
            drain(ser, rx, samples)
        baseline = samples[-1] if samples else None

        period = 1.0 / args.rate
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            started = time.monotonic()
            send_json(ser, {"T": 11, "L": args.pwm, "R": args.pwm})
            before = len(samples)
            drain(ser, rx, samples)
            if len(samples) > before:
                motion_samples.extend(samples[before:])
            delay = period - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)

        # Direct PWM zero, then firmware emergency stop for deterministic end state.
        for _ in range(4):
            send_json(ser, {"T": 11, "L": 0, "R": 0})
            time.sleep(0.05)
            drain(ser, rx, samples)
        send_json(ser, {"T": 0})
        time.sleep(0.1)
        drain(ser, rx, samples)

        if not motion_samples:
            print("PWM_SUMMARY: no T=1001 feedback during motion")
            return 3

        first = baseline if baseline is not None else motion_samples[0]
        last = motion_samples[-1]
        odl0, odr0 = finite(first.get("odl")), finite(first.get("odr"))
        odl1, odr1 = finite(last.get("odl")), finite(last.get("odr"))
        if None not in (odl0, odr0, odl1, odr1):
            dl = odl1 - odl0
            dr = odr1 - odr0
            print(
                "PWM_COUNTERS: "
                f"odl={odl0:.0f}->{odl1:.0f} delta={dl:.0f} "
                f"odr={odr0:.0f}->{odr1:.0f} delta={dr:.0f} "
                f"difference={dr - dl:.0f}"
            )

        paired = []
        for msg in motion_samples:
            left = finite(msg.get("L"))
            right = finite(msg.get("R"))
            if left is not None and right is not None:
                paired.append((left, right))
        if paired:
            lvals = [p[0] for p in paired]
            rvals = [p[1] for p in paired]
            diffs = [l - r for l, r in paired]
            print(
                "PWM_SPEEDS: "
                f"n={len(paired)} "
                f"L_mean={statistics.fmean(lvals):.4f} "
                f"R_mean={statistics.fmean(rvals):.4f} "
                f"mean_L_minus_R={statistics.fmean(diffs):.4f} "
                f"max_abs_L_minus_R={max(abs(v) for v in diffs):.4f}"
            )

        volts = [finite(s.get("v")) for s in motion_samples]
        volts = [v / 100.0 for v in volts if v is not None]
        if volts:
            print(
                "PWM_BATTERY: "
                f"mean={statistics.fmean(volts):.2f}V min={min(volts):.2f}V max={max(volts):.2f}V"
            )

        print("PWM_SUMMARY: raw T=11 vendor PWM test finished; report straight / steady arc / weave.")
        return 0
    finally:
        try:
            send_json(ser, {"T": 11, "L": 0, "R": 0})
            send_json(ser, {"T": 0})
        except Exception:
            pass
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
