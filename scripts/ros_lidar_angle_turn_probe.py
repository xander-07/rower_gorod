#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String


class AngleTurnProbe(Node):
    def __init__(self) -> None:
        super().__init__('rower_lidar_angle_turn_probe')

        # Keep command history minimal: stale non-zero cmd_vel messages are
        # especially undesirable when we are stopping on a precise yaw target.
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.estop_pub = self.create_publisher(Empty, '/base/emergency_stop', 1)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.create_subscription(String, '/base/raw_feedback', self._raw_cb, 20)

        self.latest_scan: list[float] | None = None
        self.scan_increment: float | None = None
        self.scan_seq = 0
        self.latest_pose: tuple[float, float, float] | None = None
        self.latest_pose_mono: float | None = None
        self.latest_raw: tuple[float, float] | None = None

    @staticmethod
    def _yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = list(msg.ranges)
        self.scan_increment = float(msg.angle_increment)
        self.scan_seq += 1

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.latest_pose = (
            float(p.position.x),
            float(p.position.y),
            self._yaw(p.orientation),
        )
        self.latest_pose_mono = time.monotonic()

    def _raw_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.latest_raw = (float(payload['odl']), float(payload['odr']))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return

    def publish(self, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)

    def emergency_stop(self) -> None:
        self.estop_pub.publish(Empty())


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def collect_scans(node: AngleTurnProbe, count: int, timeout: float = 4.0) -> list[list[float]]:
    scans: list[list[float]] = []
    deadline = time.monotonic() + timeout
    last_seq = node.scan_seq
    while len(scans) < count and time.monotonic() < deadline:
        if node.latest_scan is not None and node.scan_seq != last_seq:
            scans.append(list(node.latest_scan))
            last_seq = node.scan_seq
        time.sleep(0.01)
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


def stop_robot(node: AngleTurnProbe, *, emergency: bool = False, seconds: float = 0.35) -> None:
    if emergency:
        node.emergency_stop()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        node.publish(0.0)
        time.sleep(0.02)
    if emergency:
        node.emergency_stop()


def wait_for_fresh_pose(node: AngleTurnProbe, timeout: float = 3.0, max_age: float = 0.15) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if node.latest_pose is not None and node.latest_pose_mono is not None:
            if time.monotonic() - node.latest_pose_mono <= max_age:
                return True
        time.sleep(0.01)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Guarded turn-to-angle test. The robot turns via normal ROS /cmd_vel, '
            'stops from fresh /odom feedback (Waveshare-style), and then '
            'independently verifies the real rotation using LiDAR scan alignment.'
        )
    )
    parser.add_argument('--run', action='store_true', help='Required to allow motion.')
    parser.add_argument('--angle', type=float, required=True, help='Target chassis angle in degrees. +left, -right; 15..120 deg magnitude.')
    parser.add_argument('--angular', type=float, default=0.40, help='Command magnitude in rad/s, 0.20..1.00. In current PWM mode this mainly selects turn direction; bridge uses calibrated turn PWM.')
    parser.add_argument('--rate', type=float, default=50.0, help='Control-loop rate, 20..100 Hz. ROS callbacks run independently in a background executor.')
    parser.add_argument('--timeout', type=float, default=8.0, help='Safety timeout in seconds, 2..12 s.')
    parser.add_argument('--scans', type=int, default=9, help='Median scans before/after, 5..15.')
    parser.add_argument('--stop-margin', type=float, default=2.0, help='Stop this many degrees before the requested target to absorb transport/mechanical delay, 0..8 deg.')
    parser.add_argument('--max-odom-age', type=float, default=0.15, help='Abort if /odom feedback is older than this many seconds, 0.08..0.50.')
    args = parser.parse_args()

    if not args.run:
        print('REFUSING TO TURN: pass --run only after clearing space around the robot.')
        return 2
    if not (15.0 <= abs(args.angle) <= 120.0):
        print('ERROR: |--angle| must be between 15 and 120 degrees.')
        return 2
    if not (0.20 <= abs(args.angular) <= 1.00):
        print('ERROR: |--angular| must be between 0.20 and 1.00 rad/s.')
        return 2
    if not (20.0 <= args.rate <= 100.0):
        print('ERROR: --rate must be in 20..100 Hz.')
        return 2
    if not (2.0 <= args.timeout <= 12.0):
        print('ERROR: --timeout must be in 2..12 s.')
        return 2
    if not (5 <= args.scans <= 15):
        print('ERROR: --scans must be in 5..15.')
        return 2
    if not (0.0 <= args.stop_margin <= 8.0):
        print('ERROR: --stop-margin must be in 0..8 degrees.')
        return 2
    if not (0.08 <= args.max_odom_age <= 0.50):
        print('ERROR: --max-odom-age must be in 0.08..0.50 s.')
        return 2

    direction_sign = 1.0 if args.angle > 0.0 else -1.0
    command = direction_sign * abs(args.angular)
    target_deg = abs(args.angle)
    target_rad = math.radians(target_deg)
    stop_trigger_deg = max(1.0, target_deg - args.stop_margin)
    stop_trigger_rad = math.radians(stop_trigger_deg)
    direction = 'LEFT / CCW' if direction_sign > 0 else 'RIGHT / CW'

    print('ODOM-CLOSED-LOOP ANGLE TURN TEST ENABLED.')
    print('ROS callbacks now run continuously in a background executor so stop decisions use fresh /odom data.')
    print('At the stop threshold the probe publishes zero AND /base/emergency_stop, then keeps publishing zero.')
    print('LiDAR is used only after the stop as an independent verification of the real chassis angle.')
    print('Keep mapping STOPPED for this calibration. rower_bringup with enable_motion:=true must be running.')
    print('Clear at least 0.5 m around the robot and be ready to stop the bringup terminal with Ctrl+C.')
    print(
        f'Target: {args.angle:+.1f} deg ({direction}); '
        f'stop trigger={direction_sign * stop_trigger_deg:+.1f} deg; '
        f'cmd angular.z={command:+.3f} rad/s; timeout={args.timeout:.1f}s'
    )
    print('Starting in 3 seconds...')
    time.sleep(3.0)

    rclpy.init()
    node = AngleTurnProbe()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        stop_robot(node, seconds=0.30)

        before_raw = collect_scans(node, args.scans)
        if len(before_raw) < 5 or node.scan_increment is None:
            print('ERROR: not enough /scan data. Ensure rower_bringup/lidar is running.')
            return 3
        if not wait_for_fresh_pose(node, timeout=2.0, max_age=args.max_odom_age):
            print('ERROR: no fresh /odom data. Ensure rower_base_bridge is running.')
            return 3

        before_scan = median_scan(before_raw)
        x0, y0, yaw0 = node.latest_pose
        raw0 = node.latest_raw

        period = 1.0 / args.rate
        deadline = time.monotonic() + args.timeout
        reached = False
        max_progress = 0.0
        stop_reason = ''

        while time.monotonic() < deadline:
            started = time.monotonic()

            pose = node.latest_pose
            pose_stamp = node.latest_pose_mono
            if pose is None or pose_stamp is None:
                stop_robot(node, emergency=True)
                print('ERROR: /odom disappeared during turn; emergency stop sent.')
                return 4

            pose_age = time.monotonic() - pose_stamp
            if pose_age > args.max_odom_age:
                stop_robot(node, emergency=True)
                print(
                    'ERROR: stale /odom during turn; '
                    f'age={pose_age:.3f}s > limit={args.max_odom_age:.3f}s. Emergency stop sent.'
                )
                return 4

            progress = direction_sign * angle_diff(pose[2], yaw0)
            max_progress = max(max_progress, progress)

            # Check the target BEFORE sending another non-zero command. This is
            # the key difference from the previous version, which could keep
            # issuing turn commands while odom callbacks were queued behind
            # scan/raw callbacks.
            if progress >= stop_trigger_rad:
                reached = True
                stop_reason = f'fresh odom reached {math.degrees(progress):.2f}deg'
                node.publish(0.0)
                node.emergency_stop()
                break

            node.publish(command)

            delay = period - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)

        stop_robot(node, emergency=True, seconds=0.45)
        time.sleep(0.25)

        if not reached:
            print(f'ERROR: target not reached before timeout; max_odom_progress={math.degrees(max_progress):.2f}deg')
            return 4
        if not wait_for_fresh_pose(node, timeout=1.0, max_age=args.max_odom_age):
            print('ERROR: /odom did not remain fresh after stop.')
            return 3

        x1, y1, yaw1 = node.latest_pose
        odom_angle = math.degrees(angle_diff(yaw1, yaw0))
        drift = math.hypot(x1 - x0, y1 - y0)
        raw1 = node.latest_raw

        after_raw = collect_scans(node, args.scans)
        if len(after_raw) < 5:
            print('ERROR: not enough /scan data after turn.')
            return 3
        after_scan = median_scan(after_raw)

        min_angle = max(5.0, target_deg - 40.0)
        max_angle = min(170.0, target_deg + 40.0)
        shift_angle, shift, score, overlap = estimate_rotation(
            before_scan,
            after_scan,
            node.scan_increment,
            min_angle,
            max_angle,
        )
        lidar_magnitude = abs(math.degrees(shift_angle))
        lidar_angle = direction_sign * lidar_magnitude

        print(
            'ANGLE_TURN_RESULT: '
            f'target={args.angle:+.2f}deg '
            f'stop_trigger={direction_sign * stop_trigger_deg:+.2f}deg '
            f'odom={odom_angle:+.2f}deg '
            f'lidar={lidar_angle:+.2f}deg '
            f'odom_error={odom_angle - args.angle:+.2f}deg '
            f'lidar_error={lidar_angle - args.angle:+.2f}deg '
            f'center_drift={drift:.4f}m '
            f'stop_reason="{stop_reason}"'
        )
        print(
            'LIDAR_VERIFY: '
            f'raw_scan_shift={math.degrees(shift_angle):+.2f}deg '
            f'shift={shift}bins overlap={overlap} score={score:.4f}'
        )

        if raw0 is not None and raw1 is not None:
            dl = raw1[0] - raw0[0]
            dr = raw1[1] - raw0[1]
            print(
                'RAW_COUNTERS: '
                f'odl={raw0[0]:.0f}->{raw1[0]:.0f} delta={dl:.0f} '
                f'odr={raw0[1]:.0f}->{raw1[1]:.0f} delta={dr:.0f} '
                f'delta_difference={dr - dl:.0f}'
            )
        else:
            print('RAW_COUNTERS: unavailable')

        if score <= 0.08:
            print('TURN_VERIFY_SUMMARY: LiDAR match is strong. Compare target, odom and lidar errors above.')
        else:
            print('TURN_VERIFY_SUMMARY: LiDAR match is weak; repeat in a more asymmetric static scene before changing calibration.')
        return 0
    finally:
        try:
            stop_robot(node, emergency=True, seconds=0.25)
        except Exception:
            pass
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
