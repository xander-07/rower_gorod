#!/usr/bin/env python3
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class SlamScanGate(Node):
    """Suppress LiDAR scans while the skid-steer base is rotating.

    The STL-19P builds one 360-degree scan over roughly 0.1 s. During a fast
    in-place turn that scan is motion-distorted: points at the beginning and end
    of the revolution were measured at different chassis headings. Feeding those
    scans into slam_toolbox produced the observed fan/duplicated-wall failure.

    This node closes immediately on a discrete mapping turn request or angular
    /cmd_vel command, also watches measured odometry as a safety backstop, and
    only reopens after the chassis has remained rotationally quiet for a short
    settling interval. Straight-line scans continue to pass through normally.
    """

    def __init__(self) -> None:
        super().__init__('rower_slam_scan_gate')

        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_slam')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('turn_request_topic', '/mapping/turn_angle_deg')
        self.declare_parameter('command_angular_threshold', 0.05)
        self.declare_parameter('odom_block_threshold', 0.12)
        self.declare_parameter('odom_release_threshold', 0.05)
        self.declare_parameter('command_freshness_sec', 0.30)
        self.declare_parameter('settle_time_sec', 0.60)
        self.declare_parameter('require_odom_before_open', True)

        input_scan = str(self.get_parameter('input_scan_topic').value)
        output_scan = str(self.get_parameter('output_scan_topic').value)
        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        turn_request_topic = str(self.get_parameter('turn_request_topic').value)
        self.command_threshold = float(self.get_parameter('command_angular_threshold').value)
        self.odom_block_threshold = float(self.get_parameter('odom_block_threshold').value)
        self.odom_release_threshold = float(self.get_parameter('odom_release_threshold').value)
        self.command_freshness = float(self.get_parameter('command_freshness_sec').value)
        self.settle_time = float(self.get_parameter('settle_time_sec').value)
        self.require_odom = bool(self.get_parameter('require_odom_before_open').value)

        if self.command_threshold < 0.0:
            raise ValueError('command_angular_threshold must be >= 0')
        if self.odom_release_threshold < 0.0:
            raise ValueError('odom_release_threshold must be >= 0')
        if self.odom_block_threshold < self.odom_release_threshold:
            raise ValueError('odom_block_threshold must be >= odom_release_threshold')
        if self.command_freshness <= 0.0 or self.settle_time < 0.0:
            raise ValueError('command_freshness_sec must be > 0 and settle_time_sec must be >= 0')

        self.pub = self.create_publisher(LaserScan, output_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, input_scan, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, cmd_topic, self._cmd_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)
        self.create_subscription(Float64, turn_request_topic, self._turn_request_cb, 10)

        now = time.monotonic()
        self.blocked = True
        self.last_rotation_seen = now
        self.last_cmd_time: float | None = None
        self.last_cmd_angular = 0.0
        self.last_odom_angular = 0.0
        self.have_odom = False
        self.forwarded = 0
        self.dropped = 0
        self.last_stats = now

        self.create_timer(0.02, self._state_timer)
        self.get_logger().warning(
            f'SLAM scan gate active: {input_scan} -> {output_scan}; '
            f'block turn requests, cmd>|{self.command_threshold:.2f}| rad/s or odom>|{self.odom_block_threshold:.2f}| rad/s; '
            f'reopen after odom<={self.odom_release_threshold:.2f} rad/s for {self.settle_time:.2f}s.'
        )

    def _fresh_turn_command(self, now: float) -> bool:
        return (
            self.last_cmd_time is not None
            and now - self.last_cmd_time <= self.command_freshness
            and abs(self.last_cmd_angular) >= self.command_threshold
        )

    def _set_blocked(self, blocked: bool, reason: str) -> None:
        if blocked == self.blocked:
            return
        self.blocked = blocked
        state = 'CLOSED' if blocked else 'OPEN'
        self.get_logger().warning(f'SLAM scan gate {state}: {reason}')

    def _turn_request_cb(self, msg: Float64) -> None:
        now = time.monotonic()
        self.last_rotation_seen = now
        self._set_blocked(True, f'discrete turn request={float(msg.data):+.1f}deg')

    def _cmd_cb(self, msg: Twist) -> None:
        now = time.monotonic()
        self.last_cmd_time = now
        self.last_cmd_angular = float(msg.angular.z)
        if abs(self.last_cmd_angular) >= self.command_threshold:
            self.last_rotation_seen = now
            self._set_blocked(True, f'angular cmd_vel={self.last_cmd_angular:+.3f} rad/s')

    def _odom_cb(self, msg: Odometry) -> None:
        now = time.monotonic()
        self.have_odom = True
        self.last_odom_angular = float(msg.twist.twist.angular.z)
        magnitude = abs(self.last_odom_angular)

        if magnitude >= self.odom_block_threshold:
            self.last_rotation_seen = now
            self._set_blocked(True, f'odom angular={self.last_odom_angular:+.3f} rad/s')
        elif self.blocked and magnitude > self.odom_release_threshold:
            self.last_rotation_seen = now

    def _state_timer(self) -> None:
        now = time.monotonic()

        if self._fresh_turn_command(now):
            self.last_rotation_seen = now
            self._set_blocked(True, 'fresh angular command')
            return

        if self.require_odom and not self.have_odom:
            self.last_rotation_seen = now
            self._set_blocked(True, 'waiting for odometry')
            return

        if abs(self.last_odom_angular) > self.odom_release_threshold:
            self.last_rotation_seen = now
            self._set_blocked(True, 'odometry still rotating')
            return

        quiet_for = now - self.last_rotation_seen
        if quiet_for >= self.settle_time:
            self._set_blocked(False, f'rotation quiet for {quiet_for:.2f}s')

        if now - self.last_stats >= 5.0:
            self.get_logger().info(
                f'scan gate stats: state={"CLOSED" if self.blocked else "OPEN"} '
                f'forwarded={self.forwarded} dropped={self.dropped} '
                f'odom_w={self.last_odom_angular:+.3f} cmd_w={self.last_cmd_angular:+.3f}'
            )
            self.last_stats = now

    def _scan_cb(self, msg: LaserScan) -> None:
        if self.blocked:
            self.dropped += 1
            return
        self.pub.publish(msg)
        self.forwarded += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamScanGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
