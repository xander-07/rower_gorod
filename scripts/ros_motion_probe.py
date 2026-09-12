#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MotionProbe(Node):
    def __init__(self, speed: float, duration: float, rate_hz: float) -> None:
        super().__init__('rower_ros_motion_probe')
        self.speed = speed
        self.duration = duration
        self.rate_hz = rate_hz
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.samples = 0
        self.first_x = None
        self.first_y = None
        self.last_x = None
        self.last_y = None
        self.max_linear = 0.0
        self.max_angular = 0.0

    def _odom_cb(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.first_x is None:
            self.first_x = x
            self.first_y = y
        self.last_x = x
        self.last_y = y
        self.max_linear = max(self.max_linear, abs(float(msg.twist.twist.linear.x)))
        self.max_angular = max(self.max_angular, abs(float(msg.twist.twist.angular.z)))
        self.samples += 1

    def publish(self, linear_x: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = 0.0
        self.pub.publish(msg)


def spin_for(node: MotionProbe, seconds: float, publish_speed: float | None = None) -> None:
    period = 1.0 / node.rate_hz
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if publish_speed is not None:
            node.publish(publish_speed)
        rclpy.spin_once(node, timeout_sec=min(period, 0.05))
        remaining = period - 0.01
        if remaining > 0:
            time.sleep(remaining)


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Guarded ROS 2 /cmd_vel -> base bridge -> encoder/odom motion test.'
    )
    parser.add_argument('--run', action='store_true', help='Required to allow motion commands.')
    parser.add_argument('--speed', type=float, default=0.10, help='Forward speed in m/s (max 0.15).')
    parser.add_argument('--seconds', type=float, default=1.0, help='Command duration (max 2.0 s).')
    parser.add_argument('--rate', type=float, default=10.0, help='Publish rate in Hz.')
    args = parser.parse_args()

    if not args.run:
        print('REFUSING TO MOVE: pass --run only with all six wheels safely off the ground.')
        return 2
    if not (0.02 <= args.speed <= 0.15):
        print('ERROR: --speed must be between 0.02 and 0.15 m/s for this diagnostic.')
        return 2
    if not (0.2 <= args.seconds <= 2.0):
        print('ERROR: --seconds must be between 0.2 and 2.0 s.')
        return 2
    if not (5.0 <= args.rate <= 20.0):
        print('ERROR: --rate must be between 5 and 20 Hz.')
        return 2

    print('WARNING: ROS MOTION TEST ENABLED. ALL SIX WHEELS MUST BE OFF THE GROUND.')
    print(f'Publishing /cmd_vel linear.x={args.speed:.3f} m/s for {args.seconds:.2f}s at {args.rate:.1f}Hz')

    rclpy.init()
    node = MotionProbe(args.speed, args.seconds, args.rate)
    try:
        # Collect baseline odometry before commanding motion.
        spin_for(node, 0.5)
        spin_for(node, args.seconds, publish_speed=args.speed)

        # Explicit stop, then allow the bridge watchdog/feedback to settle.
        for _ in range(5):
            node.publish(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        spin_for(node, 0.5)

        if node.first_x is None or node.last_x is None:
            print('SUMMARY: no /odom samples received')
            return 3

        dx = node.last_x - node.first_x
        dy = node.last_y - node.first_y
        displacement = math.hypot(dx, dy)
        print(
            'SUMMARY: '
            f'odom_samples={node.samples} '
            f'dx={dx:.4f}m dy={dy:.4f}m displacement={displacement:.4f}m '
            f'max|linear.x|={node.max_linear:.4f}m/s '
            f'max|angular.z|={node.max_angular:.4f}rad/s'
        )
        if node.max_linear > 0.01 or displacement > 0.005:
            print('OK: ROS /cmd_vel motion path and odometry response are present.')
            return 0

        print('NO MOTION DETECTED: verify that rower_base_bridge was launched with enable_motion:=true.')
        return 4
    finally:
        node.publish(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
