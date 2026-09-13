#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial


class ScanCollector(Node):
    def __init__(self) -> None:
        super().__init__('rower_pwm_lidar_turn_probe')
        self.latest_scan: list[float] | None = None
        self.scan_increment: float | None = None
        self.scan_seq = 0
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = list(msg.ranges)
        self.scan_increment = float(msg.angle_increment)
        self.scan_seq += 1


def send_json(ser: serial.Serial, payload: dict) -> None:
    ser.write((json.dumps(payload, separators=(',', ':')) + '\n').encode('utf-8'))
    ser.flush()


def drain_base(ser: serial.Serial, rx: bytearray, samples: list[dict]) -> None:
    waiting = ser.in_waiting
    if waiting > 0:
        rx.extend(ser.read(waiting))
    while b'\n' in rx:
        raw, _, rest = rx.partition(b'\n')
        rx[:] = rest
        text = raw.decode('utf-8', errors='replace').strip()
        if not text:
            continue
        try:
            msg = json.loads(text)
        except json.JSONDecodeError:
            continue
        if msg.get('T') == 1001:
            samples.append(msg)


def finite(value):
    try:
        x = float(value)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def collect_scans(node: ScanCollector, count: int, timeout: float = 3.0) -> list[list[float]]:
    scans: list[list[float]] = []
    deadline = time.monotonic() + timeout
    last_seq = node.scan_seq
    while len(scans) < count and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest_scan is not None and node.scan_seq != last_seq:
            scans.append(list(node.latest_scan))
            last_seq = node.scan_seq
    return scans


def median_scan(scans: list[list[float]], min_range: float = 0.08, max_range: float = 8.0) -> list[float]:
    if not scans:
        return []
    n = min(len(s) for s in scans)
    out: list[float] = []
    for i in range(n):
        vals = [s[i] for s in scans if math.isfinite(s[i]) and min_range <= s[i] <= max_range]
        out.append(statistics.median(vals) if vals else math.inf)
    return out


def robust_shift_score(before: list[float], after: list[float], shift: int) -> tuple[float, int]:
    n = min(len(before), len(after))
    diffs: list[float] = []
    for i in range(n):
        a = after[i]
        b = before[(i + shift) % n]
        if math.isfinite(a) and math.isfinite(b):
            denom = max(0.20, min(a, b))
            diffs.append(abs(a - b) / denom)
    if len(diffs) < max(80, n // 4):
        return math.inf, len(diffs)
    diffs.sort()
    keep = diffs[: max(1, int(len(diffs) * 0.80))]
    return sum(keep) / len(keep), len(diffs)


def estimate_rotation(
    before: list[float],
    after: list[float],
    angle_increment: float,
    min_angle_deg: float,
    max_angle_deg: float,
) -> tuple[float, int, float, int]:
    n = min(len(before), len(after))
    if n == 0 or angle_increment <= 0.0:
        raise ValueError('invalid scan data')
    min_shift = max(1, int(math.radians(min_angle_deg) / angle_increment))
    max_shift = min(n // 2 - 1, int(math.radians(max_angle_deg) / angle_increment))
    if min_shift >= max_shift:
        raise ValueError('invalid LiDAR shift search limits')

    best_shift = None
    best_score = math.inf
    best_overlap = 0
    for shift in range(-max_shift, max_shift + 1):
        if abs(shift) < min_shift:
            continue
        score, overlap = robust_shift_score(before, after, shift)
        if score < best_score:
            best_shift = shift
            best_score = score
            best_overlap = overlap
    if best_shift is None:
        raise RuntimeError('could not estimate rotation from scans')
    return best_shift * angle_increment, best_shift, best_score, best_overlap


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Guarded in-place turn calibration using raw Waveshare T=11 PWM and LiDAR scan alignment. '
            'Run the rower_lidar ROS node separately; keep rower_base_bridge/mapping stopped.'
        )
    )
    parser.add_argument('--run', action='store_true', help='Required to allow motion.')
    parser.add_argument('--port', default='/dev/rower_base')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--direction', choices=('left', 'right'), required=True)
    parser.add_argument('--pwm', type=int, default=30, help='Absolute PWM per side, 20..80.')
    parser.add_argument('--seconds', type=float, default=0.8, help='Turn duration, 0.4..1.5 s.')
    parser.add_argument('--rate', type=float, default=10.0, help='PWM command repeat rate, 5..20 Hz.')
    parser.add_argument('--scans', type=int, default=7, help='Median scans before/after, 3..12.')
    parser.add_argument('--min-angle', type=float, default=5.0)
    parser.add_argument('--max-angle', type=float, default=150.0)
    args = parser.parse_args()

    if not args.run:
        print('REFUSING TO TURN: pass --run only after clearing space around the robot.')
        return 2
    if not (20 <= abs(args.pwm) <= 80):
        print('ERROR: |--pwm| must be in 20..80.')
        return 2
    if not (0.4 <= args.seconds <= 1.5):
        print('ERROR: --seconds must be in 0.4..1.5 s.')
        return 2
    if not (5.0 <= args.rate <= 20.0):
        print('ERROR: --rate must be in 5..20 Hz.')
        return 2
    if not (3 <= args.scans <= 12):
        print('ERROR: --scans must be in 3..12.')
        return 2
    if not (2.0 <= args.min_angle < args.max_angle <= 175.0):
        print('ERROR: require 2 <= --min-angle < --max-angle <= 175.')
        return 2

    if args.direction == 'left':
        left_pwm, right_pwm, direction_sign = -abs(args.pwm), abs(args.pwm), 1.0
    else:
        left_pwm, right_pwm, direction_sign = abs(args.pwm), -abs(args.pwm), -1.0

    print('DIRECT PWM + LIDAR TURN PROBE ENABLED.')
    print('Keep rower_base_bridge and mapping STOPPED. Only the lidar ROS node should be running.')
    print('Use a static scene with asymmetric walls/objects visible to LiDAR.')
    print('Clear at least 0.5 m around the robot and be ready to cut power.')
    print(f'Command: T=11 L={left_pwm} R={right_pwm} for {args.seconds:.2f}s ({args.direction.upper()})')
    print('Starting in 3 seconds...')
    time.sleep(3.0)

    rclpy.init()
    node = ScanCollector()
    ser = serial.Serial(args.port, args.baud, timeout=0)
    rx = bytearray()
    base_samples: list[dict] = []

    try:
        # Wait for scan topic.
        before_raw = collect_scans(node, args.scans, timeout=4.0)
        if len(before_raw) < 3 or node.scan_increment is None:
            print('ERROR: not enough /scan data. Start rower_lidar first.')
            return 3
        before = median_scan(before_raw)

        ser.reset_input_buffer()
        send_json(ser, {'T': 4, 'cmd': 0})
        send_json(ser, {'T': 999})
        for _ in range(4):
            send_json(ser, {'T': 11, 'L': 0, 'R': 0})
            time.sleep(0.08)
            drain_base(ser, rx, base_samples)
        baseline = base_samples[-1] if base_samples else None

        period = 1.0 / args.rate
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            started = time.monotonic()
            send_json(ser, {'T': 11, 'L': left_pwm, 'R': right_pwm})
            drain_base(ser, rx, base_samples)
            rclpy.spin_once(node, timeout_sec=min(0.03, period))
            delay = period - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)

        for _ in range(5):
            send_json(ser, {'T': 11, 'L': 0, 'R': 0})
            time.sleep(0.05)
            drain_base(ser, rx, base_samples)
            rclpy.spin_once(node, timeout_sec=0.02)
        send_json(ser, {'T': 0})
        time.sleep(0.25)

        after_raw = collect_scans(node, args.scans, timeout=4.0)
        if len(after_raw) < 3:
            print('ERROR: not enough /scan data after turn.')
            return 3
        after = median_scan(after_raw)

        shift_angle, shift, score, overlap = estimate_rotation(
            before,
            after,
            node.scan_increment,
            args.min_angle,
            args.max_angle,
        )
        magnitude_deg = abs(math.degrees(shift_angle))
        chassis_angle_deg = direction_sign * magnitude_deg
        avg_rate = math.radians(magnitude_deg) / args.seconds
        print(
            'PWM_LIDAR_RESULT: '
            f'chassis_angle={chassis_angle_deg:.2f}deg magnitude={magnitude_deg:.2f}deg '
            f'avg_rate={avg_rate:.4f}rad/s raw_scan_shift={math.degrees(shift_angle):.2f}deg '
            f'shift={shift}bins overlap={overlap} score={score:.4f}'
        )

        if baseline is not None and base_samples:
            last = base_samples[-1]
            odl0, odr0 = finite(baseline.get('odl')), finite(baseline.get('odr'))
            odl1, odr1 = finite(last.get('odl')), finite(last.get('odr'))
            if None not in (odl0, odr0, odl1, odr1):
                dl = odl1 - odl0
                dr = odr1 - odr0
                print(
                    'PWM_RAW_COUNTERS: '
                    f'odl={odl0:.0f}->{odl1:.0f} delta={dl:.0f} '
                    f'odr={odr0:.0f}->{odr1:.0f} delta={dr:.0f} '
                    f'delta_difference={dr - dl:.0f}'
                )

        print('PWM_TURN_SUMMARY: report whether the turn was smooth, jerky, or translated noticeably.')
        return 0
    finally:
        try:
            send_json(ser, {'T': 11, 'L': 0, 'R': 0})
            send_json(ser, {'T': 0})
        except Exception:
            pass
        ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
