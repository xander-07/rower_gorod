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
    wire = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
    ser.write(wire)
    ser.flush()


def drain_feedback(ser: serial.Serial, rx: bytearray, samples: list[dict]) -> None:
    waiting = ser.in_waiting
    if waiting > 0:
        rx.extend(ser.read(waiting))

    while b"\n" in rx:
        raw, _, remainder = rx.partition(b"\n")
        rx[:] = remainder
        text = raw.decode("utf-8", errors="replace").strip()
        if not text:
            continue
        try:
            msg = json.loads(text)
        except json.JSONDecodeError:
            continue
        if msg.get("T") == 1001:
            samples.append(msg)


def finite(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Guarded direct Waveshare lower-controller floor probe. "
            "This bypasses ROS /cmd_vel and talks T=1 directly to the ESP32."
        )
    )
    parser.add_argument("--run", action="store_true", help="Required to allow motion.")
    parser.add_argument("--port", default="/dev/rower_base")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--speed", type=float, default=0.10, help="Equal L/R target in m/s (0.05..0.12).")
    parser.add_argument("--seconds", type=float, default=3.0, help="Motion duration (1.0..5.0 s).")
    parser.add_argument("--rate", type=float, default=10.0, help="T=1 command rate (5..20 Hz).")
    parser.add_argument(
        "--set-pid",
        action="store_true",
        help="Temporarily send T=2 PID values before the run. Runtime only unless firmware saves them elsewhere.",
    )
    parser.add_argument("--pid-p", type=float, default=200.0)
    parser.add_argument("--pid-i", type=float, default=2500.0)
    parser.add_argument("--pid-d", type=float, default=0.0)
    parser.add_argument("--pid-limit", type=float, default=255.0)
    args = parser.parse_args()

    if not args.run:
        print("REFUSING TO MOVE: pass --run only after stopping ROS bringup and clearing the floor.")
        return 2
    if not (0.05 <= args.speed <= 0.12):
        print("ERROR: --speed must be in 0.05..0.12 m/s.")
        return 2
    if not (1.0 <= args.seconds <= 5.0):
        print("ERROR: --seconds must be in 1.0..5.0 s.")
        return 2
    if not (5.0 <= args.rate <= 20.0):
        print("ERROR: --rate must be in 5..20 Hz.")
        return 2

    nominal = args.speed * args.seconds
    print("DIRECT LOWER-CONTROLLER FLOOR PROBE ENABLED.")
    print("IMPORTANT: stop rower_bringup / mapping before this test so this script is the only UART owner.")
    print(f"Command: T=1 L=R={args.speed:.3f} m/s for {args.seconds:.2f}s (~{nominal:.2f} m nominal).")
    if args.set_pid:
        print(
            "Temporary PID override: "
            f"P={args.pid_p:g} I={args.pid_i:g} D={args.pid_d:g} L={args.pid_limit:g}"
        )
    else:
        print("PID override: OFF (uses the lower controller's current PID state).")
    print("Use at least 1.0 m clear space. Be ready to cut robot power if motion is unsafe.")
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
        if args.set_pid:
            send_json(
                ser,
                {
                    "T": 2,
                    "P": args.pid_p,
                    "I": args.pid_i,
                    "D": args.pid_d,
                    "L": args.pid_limit,
                },
            )
        for _ in range(5):
            send_json(ser, {"T": 1, "L": 0.0, "R": 0.0})
            time.sleep(0.1)
            drain_feedback(ser, rx, samples)

        baseline = samples[-1] if samples else None

        period = 1.0 / args.rate
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            started = time.monotonic()
            send_json(ser, {"T": 1, "L": args.speed, "R": args.speed})
            before = len(samples)
            drain_feedback(ser, rx, samples)
            if len(samples) > before:
                motion_samples.extend(samples[before:])
            delay = period - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)

        # Normal zero first, then hard stop for a deterministic safe end state.
        for _ in range(5):
            send_json(ser, {"T": 1, "L": 0.0, "R": 0.0})
            time.sleep(0.05)
            drain_feedback(ser, rx, samples)
        send_json(ser, {"T": 0})
        time.sleep(0.1)
        drain_feedback(ser, rx, samples)

        if not motion_samples:
            print("DIRECT_SUMMARY: no T=1001 feedback during motion")
            return 3

        left = [finite(s.get("L")) for s in motion_samples]
        right = [finite(s.get("R")) for s in motion_samples]
        paired = [(l, r) for l, r in zip(left, right) if l is not None and r is not None]
        diffs = [l - r for l, r in paired]

        dominance_flips = 0
        last_sign = 0
        for diff in diffs:
            sign = 1 if diff > 0.01 else -1 if diff < -0.01 else 0
            if sign and last_sign and sign != last_sign:
                dominance_flips += 1
            if sign:
                last_sign = sign

        if paired:
            lvals = [p[0] for p in paired]
            rvals = [p[1] for p in paired]
            print(
                "DIRECT_SPEEDS: "
                f"n={len(paired)} "
                f"L_mean={statistics.fmean(lvals):.4f} L_std={statistics.pstdev(lvals):.4f} "
                f"R_mean={statistics.fmean(rvals):.4f} R_std={statistics.pstdev(rvals):.4f} "
                f"mean_L_minus_R={statistics.fmean(diffs):.4f} "
                f"max_abs_L_minus_R={max(abs(v) for v in diffs):.4f} "
                f"dominance_flips={dominance_flips}"
            )

        first = baseline if baseline is not None else motion_samples[0]
        last = motion_samples[-1]
        odl0, odr0 = finite(first.get("odl")), finite(first.get("odr"))
        odl1, odr1 = finite(last.get("odl")), finite(last.get("odr"))
        if None not in (odl0, odr0, odl1, odr1):
            dl = odl1 - odl0
            dr = odr1 - odr0
            print(
                "DIRECT_COUNTERS: "
                f"odl={odl0:.0f}->{odl1:.0f} delta={dl:.0f} "
                f"odr={odr0:.0f}->{odr1:.0f} delta={dr:.0f} "
                f"difference={dr - dl:.0f}"
            )

        volts = [finite(s.get("v")) for s in motion_samples]
        volts = [v / 100.0 for v in volts if v is not None]
        if volts:
            print(
                "DIRECT_BATTERY: "
                f"mean={statistics.fmean(volts):.2f}V min={min(volts):.2f}V max={max(volts):.2f}V"
            )

        print(
            "DIRECT_SUMMARY: direct T=1 test finished; report whether the chassis moved straight, "
            "arced steadily, or weaved left/right."
        )
        return 0
    finally:
        try:
            send_json(ser, {"T": 1, "L": 0.0, "R": 0.0})
            send_json(ser, {"T": 0})
        except Exception:
            pass
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
