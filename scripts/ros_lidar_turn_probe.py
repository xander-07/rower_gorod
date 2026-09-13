#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LidarTurnProbe(Node):
    def __init__(self, rate_hz: float) -> None:
        super().__init__('rower_lidar_turn_probe')
        self.rate_hz = rate_hz
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.raw_sub = self.create_subscription(String, '/base/raw_feedback', self._raw_cb, 20)

        self.latest_scan = None
        self.scan_increment = None
        self.scan_seq = 0

        self.latest_pose = None
        self.latest_raw = None

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = list(msg.ranges)
        self.scan_increment = float(msg.angle_increment)
        self.scan_seq += 1

    @staticmethod
    def _yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.latest_pose = (
            float(pose.position.x),
            float(pose.position.y),
            self._yaw(pose.orientation),
        )

    def _raw_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            odl = float(payload['odl'])
            odr = float(payload['odr'])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        self.latest_raw = (odl, odr)

    def publish(self, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


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

    shift_angle = best_shift * angle_increment
    return shift_angle, best_shift, best_score, best_overlap


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Estimate real in-place rotation from LiDAR and compare it with wheel odometry.'
    )
    parser.add_argument('--run', action='store_true', help='Required to allow motion.')
    parser.add_argument('--angular', type=float, default=0.25, help='Angular command rad/s, +/-0.20..2.00.')
    parser.add_argument('--seconds', type=float, default=2.0, help='Command duration 1.0..3.0 s.')
    parser.add_argument('--rate', type=float, default=10.0, help='cmd_vel publish rate 5..20 Hz.')
    parser.add_argument('--scans', type=int, default=9, help='Number of scans to median before/after.')
    parser.add_argument('--min-angle', type=float, default=10.0, help='Ignore trivial scan shifts below this angle.')
    parser.add_argument('--max-angle', type=float, default=170.0, help='Maximum scan-alignment angle to search.')
    args = parser.parse_args()

    if not args.run:
        print('REFUSING TO TURN: pass --run only after clearing space around the robot.')
        return 2
    if not (0.20 <= abs(args.angular) <= 2.00):
        print('ERROR: |--angular| must be between 0.20 and 2.00 rad/s.')
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
    if not (2.0 <= args.min_angle < args.max_angle <= 175.0):
        print('ERROR: require 2 <= --min-angle < --max-angle <= 175 degrees.')
        return 2

    direction = 'LEFT / CCW' if args.angular > 0 else 'RIGHT / CW'
    direction_sign = 1.0 if args.angular > 0 else -1.0
    nominal_deg = math.degrees(args.angular * args.seconds)

    print('LIDAR TURN CALIBRATION ENABLED.')
    print('Use a static room/arena with walls or objects visible around the robot.')
    print('Clear at least 0.5 m around the robot and do not move nearby objects during the test.')
    print(f'Command: angular.z={args.angular:.3f} rad/s for {args.seconds:.2f}s ({direction})')
    print(f'Nominal commanded angle: {nominal_deg:.1f} deg')
    print(f'Scan matcher search: {args.min_angle:.1f}..{args.max_angle:.1f} deg, BOTH array directions')
    print('Starting in 3 seconds...')
    time.sleep(3.0)

    rclpy.init()
    node = LidarTurnProbe(args.rate)
    try:
        for _ in range(6):
            node.publish(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

        before_raw_scans = collect_scans(node, args.scans)
        if len(before_raw_scans) < 3 or node.scan_increment is None:
            print('ERROR: not enough /scan data before turn')
            return 3
        before_scan = median_scan(before_raw_scans)

        start_pose = node.latest_pose
        start_raw = node.latest_raw

        spin_for(node, args.seconds, command=args.angular)

        for _ in range(10):
            node.publish(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        spin_for(node, 0.5)

        end_pose = node.latest_pose
        end_raw = node.latest_raw

        after_raw_scans = collect_scans(node, args.scans)
        if len(after_raw_scans) < 3:
            print('ERROR: not enough /scan data after turn')
            return 3
        after_scan = median_scan(after_raw_scans)

        shift_angle, shift, score, overlap = estimate_rotation(
            before_scan,
            after_scan,
            node.scan_increment,
            args.min_angle,
            args.max_angle,
        )

        bin_deg = math.degrees(node.scan_increment)
        magnitude_deg = abs(math.degrees(shift_angle))
        chassis_angle_deg = direction_sign * magnitude_deg
        lidar_avg_rate = math.radians(magnitude_deg) / args.seconds
        response_ratio = lidar_avg_rate / abs(args.angular)

        print(
            'LIDAR_RESULT: '
            f'chassis_angle={chassis_angle_deg:.2f}deg '
            f'magnitude={magnitude_deg:.2f}deg '
            f'raw_scan_shift={math.degrees(shift_angle):.2f}deg '
            f'shift={shift}bins bin_size={bin_deg:.3f}deg '
            f'overlap={overlap} score={score:.4f}'
        )

        odom_angle_deg = math.nan
        if start_pose is not None and end_pose is not None:
            x0, y0, yaw0 = start_pose
            x1, y1, yaw1 = end_pose
            odom_angle_deg = math.degrees(angle_diff(yaw1, yaw0))
            print(
                'ODOM_RESULT: '
                f'angle={odom_angle_deg:.2f}deg '
                f'center_drift={math.hypot(x1 - x0, y1 - y0):.4f}m'
            )
        else:
            print('ODOM_RESULT: unavailable')

        raw_dl = math.nan
        raw_dr = math.nan
        raw_diff = math.nan
        if start_raw is not None and end_raw is not None:
            odl0, odr0 = start_raw
            odl1, odr1 = end_raw
            raw_dl = odl1 - odl0
            raw_dr = odr1 - odr0
            raw_diff = raw_dr - raw_dl
            print(
                'RAW_COUNTERS: '
                f'odl={odl0:.0f}->{odl1:.0f} delta={raw_dl:.0f} '
                f'odr={odr0:.0f}->{odr1:.0f} delta={raw_dr:.0f} '
                f'delta_difference={raw_diff:.0f}'
            )
        else:
            print('RAW_COUNTERS: unavailable')

        print(
            'TURN_METRICS: '
            f'commanded_avg={abs(args.angular):.4f}rad/s '
            f'lidar_avg={lidar_avg_rate:.4f}rad/s '
            f'response_ratio={response_ratio:.4f} '
            f'odom_angle={odom_angle_deg:.2f}deg '
            f'raw_diff={raw_diff:.0f}'
        )

        if magnitude_deg <= args.min_angle + 2.0 * bin_deg:
            print('WARNING: best match is near the minimum search boundary; use a less symmetric scene.')
        elif score > 0.35:
            print('WARNING: scan-match score is weak; repeat with more nearby asymmetric objects/walls visible.')
        else:
            print('OK: LiDAR scan alignment found a non-trivial rotation candidate.')
        print('NOTE: raw_scan_shift sign is scan-array correlation sign; chassis_angle sign follows cmd_vel direction.')
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