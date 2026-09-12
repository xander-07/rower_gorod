#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class FloorTurnProbe(Node):
    def __init__(self, rate_hz: float) -> None:
        super().__init__('rower_floor_turn_probe')
        self.rate_hz = rate_hz
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.raw_sub = self.create_subscription(String, '/base/raw_feedback', self._raw_cb, 20)

        self.first_pose = None
        self.last_pose = None
        self.first_raw = None
        self.last_raw = None
        self.odom_samples = 0
        self.raw_samples = 0
        self.max_linear = 0.0
        self.max_angular = 0.0

    @staticmethod
    def _yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        sample = (
            float(pose.position.x),
            float(pose.position.y),
            self._yaw(pose.orientation),
        )
        if self.first_pose is None:
            self.first_pose = sample
        self.last_pose = sample
        self.max_linear = max(self.max_linear, abs(float(msg.twist.twist.linear.x)))
        self.max_angular = max(self.max_angular, abs(float(msg.twist.twist.angular.z)))
        self.odom_samples += 1

    def _raw_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            odl = float(payload['odl'])
            odr = float(payload['odr'])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        sample = (odl, odr)
        if self.first_raw is None:
            self.first_raw = sample
        self.last_raw = sample
        self.raw_samples += 1

    def publish(self, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def spin_for(node: FloorTurnProbe, seconds: float, command: float | None = None) -> None:
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


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Guarded on-floor in-place turn calibration for the Rower skid-steer base.'
    )
    parser.add_argument('--run', action='store_true', help='Required to allow motion.')
    parser.add_argument(
        '--angular',
        type=float,
        default=0.80,
        help='Angular command in rad/s. Positive=left/CCW, negative=right/CW. |value| 0.40..1.00.',
    )
    parser.add_argument('--seconds', type=float, default=2.0, help='Command duration, 1.0..3.0 s.')
    parser.add_argument('--rate', type=float, default=10.0, help='cmd_vel publish rate, 5..20 Hz.')
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

    direction = 'LEFT / CCW' if args.angular > 0 else 'RIGHT / CW'
    nominal_deg = math.degrees(args.angular * args.seconds)

    print('FLOOR TURN TEST ENABLED.')
    print('Clear at least 0.5 m around the robot and mark its starting heading on the floor.')
    print('Be ready to stop the bringup terminal with Ctrl+C if motion is unexpected.')
    print(f'Command: angular.z={args.angular:.3f} rad/s for {args.seconds:.2f}s ({direction})')
    print(f'Nominal commanded angle before drivetrain/slip effects: {nominal_deg:.1f} deg')
    print('Starting in 3 seconds...')
    time.sleep(3.0)

    rclpy.init()
    node = FloorTurnProbe(args.rate)
    try:
        # Establish baseline while explicitly commanding zero.
        for _ in range(6):
            node.publish(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        spin_for(node, 0.5)

        # Re-zero the measurement origin immediately before the turn.
        node.first_pose = node.last_pose
        node.first_raw = node.last_raw
        node.odom_samples = 0
        node.raw_samples = 0
        node.max_linear = 0.0
        node.max_angular = 0.0

        spin_for(node, args.seconds, command=args.angular)

        # Explicit stop, then collect settling samples.
        for _ in range(10):
            node.publish(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        spin_for(node, 0.5)

        if node.first_pose is None or node.last_pose is None:
            print('SUMMARY: no /odom samples received')
            return 3

        x0, y0, yaw0 = node.first_pose
        x1, y1, yaw1 = node.last_pose
        dx = x1 - x0
        dy = y1 - y0
        drift = math.hypot(dx, dy)
        dyaw = angle_diff(yaw1, yaw0)

        print(
            'SUMMARY: '
            f'odom_samples={node.odom_samples} '
            f'dx={dx:.4f}m dy={dy:.4f}m center_drift={drift:.4f}m '
            f'dyaw={math.degrees(dyaw):.2f}deg '
            f'max|linear.x|={node.max_linear:.4f}m/s '
            f'max|angular.z|={node.max_angular:.4f}rad/s'
        )

        if node.first_raw is not None and node.last_raw is not None:
            odl0, odr0 = node.first_raw
            odl1, odr1 = node.last_raw
            dl = odl1 - odl0
            dr = odr1 - odr0
            print(
                'RAW_COUNTERS: '
                f'samples={node.raw_samples} '
                f'odl={odl0:.0f}->{odl1:.0f} delta={dl:.0f} '
                f'odr={odr0:.0f}->{odr1:.0f} delta={dr:.0f} '
                f'delta_difference={dr - dl:.0f}'
            )
        else:
            print('RAW_COUNTERS: unavailable')

        print('MEASURE: measure the REAL chassis heading change in degrees from the floor marks.')
        print('Return the measured angle and whether the robot rotated about its center or translated noticeably.')
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
