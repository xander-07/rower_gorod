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


class FloorStraightProbe(Node):
    def __init__(self, rate_hz: float) -> None:
        super().__init__('rower_floor_straight_probe')
        self.rate_hz = rate_hz
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.raw_sub = self.create_subscription(String, '/base/raw_feedback', self._raw_cb, 20)
        self.first = None
        self.last = None
        self.samples = 0
        self.linear_samples = []
        self.angular_samples = []
        self.last_raw = None
        self.first_raw = None
        self.raw_samples = 0

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        sample = (
            float(pose.position.x),
            float(pose.position.y),
            self._yaw_from_quaternion(pose.orientation),
        )
        if self.first is None:
            self.first = sample
        self.last = sample
        self.linear_samples.append(float(msg.twist.twist.linear.x))
        self.angular_samples.append(float(msg.twist.twist.angular.z))
        self.samples += 1

    def _raw_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            odl = float(payload['odl'])
            odr = float(payload['odr'])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        self.last_raw = (odl, odr)
        if self.first_raw is None:
            self.first_raw = self.last_raw
        self.raw_samples += 1

    def publish(self, linear_x: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = 0.0
        self.pub.publish(msg)


def spin_for(node: FloorStraightProbe, seconds: float, command: float | None = None) -> None:
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


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Guarded low-speed straight-line floor calibration for the ROS base bridge.'
    )
    parser.add_argument('--run', action='store_true', help='Required to allow floor motion.')
    parser.add_argument('--speed', type=float, default=0.06, help='Forward command in m/s (0.03..0.10).')
    parser.add_argument('--seconds', type=float, default=1.5, help='Command duration in seconds (0.5..40.0).')
    parser.add_argument('--rate', type=float, default=10.0, help='cmd_vel publish rate in Hz (5..20).')
    args = parser.parse_args()

    if not args.run:
        print('REFUSING TO MOVE: pass --run only after placing the robot on a clear, flat floor.')
        return 2
    if not (0.03 <= args.speed <= 0.10):
        print('ERROR: --speed must be between 0.03 and 0.10 m/s for floor calibration.')
        return 2
    if not (0.5 <= args.seconds <= 40.0):
        print('ERROR: --seconds must be between 0.5 and 40.0 s.')
        return 2
    if not (5.0 <= args.rate <= 20.0):
        print('ERROR: --rate must be between 5 and 20 Hz.')
        return 2

    nominal_travel = args.speed * args.seconds
    if nominal_travel > 2.2:
        print(
            'ERROR: guarded floor test is limited to 2.2 m nominal travel. '
            'Reduce --speed or --seconds.'
        )
        return 2

    print('FLOOR MOTION TEST ENABLED.')
    print(
        f'Place the robot on a clear, flat floor with at least '
        f'{nominal_travel + 0.5:.1f} m free space in front.'
    )
    print('Be ready to stop the bringup terminal with Ctrl+C if the robot behaves unexpectedly.')
    print(f'Command: linear.x={args.speed:.3f} m/s, angular.z=0 for {args.seconds:.2f}s')
    print(f'Nominal commanded travel (before acceleration/deceleration): {nominal_travel:.3f} m')
    print('Starting in 3 seconds...')
    time.sleep(3.0)

    rclpy.init()
    node = FloorStraightProbe(args.rate)
    try:
        # Establish a baseline and ensure a zero command is present first.
        for _ in range(5):
            node.publish(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        spin_for(node, 0.5)

        # Re-zero measurement origin at the start of commanded travel.
        node.first = node.last
        node.first_raw = node.last_raw
        node.linear_samples.clear()
        node.angular_samples.clear()
        node.samples = 0
        node.raw_samples = 0

        spin_for(node, args.seconds, command=args.speed)

        # Explicit stop and collect settling odometry/counters.
        for _ in range(8):
            node.publish(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        spin_for(node, 0.5)

        if node.first is None or node.last is None:
            print('SUMMARY: no /odom samples received')
            return 3

        x0, y0, yaw0 = node.first
        x1, y1, yaw1 = node.last
        dx = x1 - x0
        dy = y1 - y0
        distance = math.hypot(dx, dy)
        dyaw = angle_diff(yaw1, yaw0)
        max_linear = max((abs(v) for v in node.linear_samples), default=0.0)
        max_angular = max((abs(v) for v in node.angular_samples), default=0.0)

        print(
            'SUMMARY: '
            f'odom_samples={node.samples} '
            f'dx={dx:.4f}m dy={dy:.4f}m odom_distance={distance:.4f}m '
            f'dyaw={math.degrees(dyaw):.2f}deg '
            f'max|linear.x|={max_linear:.4f}m/s '
            f'max|angular.z|={max_angular:.4f}rad/s'
        )

        if node.first_raw is not None and node.last_raw is not None:
            odl0, odr0 = node.first_raw
            odl1, odr1 = node.last_raw
            dl = odl1 - odl0
            dr = odr1 - odr0
            avg_counts = (abs(dl) + abs(dr)) / 2.0
            # Current Waveshare firmware transmits int(en_odom_* * 100), so one
            # integer count is nominally 0.01 m before physical scale calibration.
            nominal_counter_distance = avg_counts * 0.01
            print(
                'RAW_COUNTERS: '
                f'samples={node.raw_samples} '
                f'odl={odl0:.0f}->{odl1:.0f} delta={dl:.0f} '
                f'odr={odr0:.0f}->{odr1:.0f} delta={dr:.0f} '
                f'avg_delta={avg_counts:.1f} '
                f'nominal_counter_distance={nominal_counter_distance:.3f}m'
            )
        else:
            print('RAW_COUNTERS: unavailable (update/rebuild rower_base_bridge if needed)')

        print('MEASURE: physically measure the robot travel from its start center to finish center.')
        print('Return the measured distance in mm and whether it visibly pulled left or right.')
        return 0
    finally:
        try:
            for _ in range(5):
                node.publish(0.0)
                rclpy.spin_once(node, timeout_sec=0.02)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
