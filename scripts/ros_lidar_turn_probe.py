#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarTurnProbe(Node):
    def __init__(self, rate_hz: float) -> None:
        super().__init__('rower_lidar_turn_probe')
        self.rate_hz = rate_hz
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.latest_scan = None
        self.scan_increment = None

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = list(msg.ranges)
        self.scan_increment = float(msg.angle_increment)

    def publish(self, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)


def spin_for(node: LidarTurnProbe, seconds: float, command: float | None = None) -> None:
    period = 1.0 / node.rate_hz
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        started = time.monotonic()
        if command is not None:
            node.publish(command)
        rclpy.spin_once(node, timeout_sec=min(0.03, period))
        delay = period - (time.monotonic() - started)
        if delay > 0:
            time.sleep(delay)


def collect_scans(node: LidarTurnProbe, count: int, timeout: float = 3.0) -> list[list[float]]:
    scans = []
    deadline = time.monotonic() + timeout
    last_obj = None
    while len(scans) < count and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest_scan is not None and node.latest_scan is not last_obj:
            scans.append(list(node.latest_scan))
            last_obj = node.latest_scan
    return scans


def median_scan(scans: list[list[float]], min_range: float = 0.08, max_range: float = 8.0) -> list[float]:
    if not scans:
        return []
    n = min(len(s) for s in scans)
    out = []
    for i in range(n):
        vals = [s[i] for s in scans if math.isfinite(s[i]) and min_range <= s[i] <= max_range]
        out.append(statistics.median(vals) if vals else math.inf)
    return out


def robust_shift_score(before: list[float], after: list[float], shift: int) -> tuple[float, int]:
    n = min(len(before), len(after))
    diffs = []
    for i in range(n):
        a = after[i]
        b = before[(i + shift) % n]
        if math.isfinite(a) and math.isfinite(b):
            # Relative/log-like error reduces domination by far walls.
            denom = max(0.20, min(a, b))
            diffs.append(abs(a - b) / denom)
    if len(diffs) < max(80, n // 4):
        return math.inf, len(diffs)
    diffs.sort()
    keep = diffs[: max(1, int(len(diffs) * 0.80))]
    return sum(keep) / len(keep), len(diffs)


def estimate_rotation(before: list[float], after: list[float], angle_increment: float, expected_sign: int) -> tuple[float, int, float, int]:
    n = min(len(before), len(after))
    if n == 0 or angle_increment <= 0.0:
        raise ValueError('invalid scan data')

    max_shift = min(n // 2 - 1, int(math.radians(170.0) / angle_increment))
    candidates = range(1, max_shift + 1) if expected_sign > 0 else range(-max_shift, 0)

    best_shift = None
    best_score = math.inf
    best_overlap = 0
    for shift in candidates:
        score, overlap = robust_shift_score(before, after, shift)
        if score < best_score:
            best_shift = shift
            best_score = score
            best_overlap = overlap

    if best_shift is None:
        raise RuntimeError('could not estimate rotation from scans')

    angle = best_shift * angle_increment
    return angle, best_shift, best_score, best_overlap


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Estimate real in-place rotation from LiDAR scan alignment while commanding /cmd_vel.'
    )
    parser.add_argument('--run', action='store_true', help='Required to allow motion.')
    parser.add_argument('--angular', type=float, default=0.80, help='Angular command rad/s, +/-0.40..1.00.')
    parser.add_argument('--seconds', type=float, default=2.0, help='Command duration 1.0..3.0 s.')
    parser.add_argument('--rate', type=float, default=10.0, help='cmd_vel publish rate 5..20 Hz.')
    parser.add_argument('--scans', type=int, default=7, help='Number of scans to median before/after.')
    args = parser.parse_args()

    if not args.run:
        print('REFUSING TO TURN: pass --run only after clearing space around the robot.')
        return 2
    if not (0.40 <= abs(args.angular) <= 1.00):
        print('ERROR: |--angular| must be between 0.40 and 1.00 rad/s.')
        return 2
    if not (1.0 <= args.seconds <= 3.0):
        print('ERROR: --seconds must be between 1.0 and 3.0 s.')
        return 2
    if not (5.0 <= args.rate <= 20.0):
        print('ERROR: --rate must be between 5 and 20 Hz.')
        return 2
    if not (3 <= args.scans <= 15):
        print('ERROR: --scans must be between 3 and 15.')
        return 2

    direction = 'LEFT / CCW' if args.angular > 0 else 'RIGHT / CW'
    print('LIDAR TURN CALIBRATION ENABLED.')
    print('Use a static room/arena with walls or objects visible around the robot.')
    print('Clear at least 0.5 m around the robot and do not move nearby objects during the test.')
    print(f'Command: angular.z={args.angular:.3f} rad/s for {args.seconds:.2f}s ({direction})')
    print('Starting in 3 seconds...')
    time.sleep(3.0)

    rclpy.init()
    node = LidarTurnProbe(args.rate)
    try:
        # Zero command before collecting the reference scan.
        for _ in range(6):
            node.publish(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

        before_raw = collect_scans(node, args.scans)
        if len(before_raw) < 3 or node.scan_increment is None:
            print('ERROR: not enough /scan data before turn')
            return 3
        before = median_scan(before_raw)

        spin_for(node, args.seconds, command=args.angular)

        # Explicit stop and let the chassis settle before the second scan set.
        for _ in range(10):
            node.publish(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        spin_for(node, 0.5)

        after_raw = collect_scans(node, args.scans)
        if len(after_raw) < 3:
            print('ERROR: not enough /scan data after turn')
            return 3
        after = median_scan(after_raw)

        angle, shift, score, overlap = estimate_rotation(
            before,
            after,
            node.scan_increment,
            1 if args.angular > 0 else -1,
        )
        bin_deg = math.degrees(node.scan_increment)
        print(
            'LIDAR_RESULT: '
            f'angle={math.degrees(angle):.2f}deg '
            f'shift={shift}bins bin_size={bin_deg:.3f}deg '
            f'overlap={overlap} score={score:.4f}'
        )
        print('NOTE: LiDAR angle is independent of wheel odometry and is the value to use for turn calibration.')
        return 0
    finally:
        try:
            for _ in range(6):
                node.publish(0.0)
                rclpy.spin_once(node, timeout_sec=0.02)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
