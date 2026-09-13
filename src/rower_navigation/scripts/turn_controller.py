#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float64


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class MappingTurnController(Node):
    """Execute discrete in-place turns from fresh odometry.

    The browser sends only an angle request. This node owns the turn command until
    the odometry target is reached, then forces zero / emergency-stop and keeps
    the chassis quiet for a short settling interval. That avoids arbitrary
    keyboard-hold turns and gives slam_toolbox a predictable pose change.
    """

    def __init__(self) -> None:
        super().__init__('rower_mapping_turn_controller')

        self.declare_parameter('request_topic', '/mapping/turn_angle_deg')
        self.declare_parameter('state_topic', '/mapping/turn_active')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('emergency_topic', '/base/emergency_stop')
        self.declare_parameter('angular_command', 0.40)
        self.declare_parameter('left_stop_margin_deg', 3.0)
        self.declare_parameter('right_stop_margin_deg', 2.0)
        self.declare_parameter('min_angle_deg', 5.0)
        self.declare_parameter('max_angle_deg', 120.0)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('odom_timeout_sec', 0.20)
        self.declare_parameter('turn_timeout_sec', 8.0)
        self.declare_parameter('settle_time_sec', 0.70)

        request_topic = str(self.get_parameter('request_topic').value)
        state_topic = str(self.get_parameter('state_topic').value)
        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        emergency_topic = str(self.get_parameter('emergency_topic').value)

        self.angular_command = abs(float(self.get_parameter('angular_command').value))
        self.left_margin = float(self.get_parameter('left_stop_margin_deg').value)
        self.right_margin = float(self.get_parameter('right_stop_margin_deg').value)
        self.min_angle = float(self.get_parameter('min_angle_deg').value)
        self.max_angle = float(self.get_parameter('max_angle_deg').value)
        self.rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self.turn_timeout = float(self.get_parameter('turn_timeout_sec').value)
        self.settle_time = float(self.get_parameter('settle_time_sec').value)

        if self.angular_command <= 0.0:
            raise ValueError('angular_command must be > 0')
        if not (0.0 <= self.left_margin <= 15.0 and 0.0 <= self.right_margin <= 15.0):
            raise ValueError('turn stop margins must be in 0..15 deg')
        if not (0.0 < self.min_angle <= self.max_angle <= 180.0):
            raise ValueError('require 0 < min_angle_deg <= max_angle_deg <= 180')
        if self.rate_hz <= 0.0 or self.odom_timeout <= 0.0 or self.turn_timeout <= 0.0:
            raise ValueError('rate/timeouts must be > 0')
        if self.settle_time < 0.0:
            raise ValueError('settle_time_sec must be >= 0')

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 1)
        self.estop_pub = self.create_publisher(Empty, emergency_topic, 1)
        self.state_pub = self.create_publisher(Bool, state_topic, 10)

        self.create_subscription(Float64, request_topic, self._request_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)
        self.create_subscription(Empty, emergency_topic, self._external_estop_cb, 10)

        self.latest_yaw: float | None = None
        self.latest_odom_time: float | None = None

        self.phase = 'IDLE'  # IDLE | TURNING | SETTLING
        self.direction = 0.0
        self.target_deg = 0.0
        self.stop_trigger_rad = 0.0
        self.start_yaw = 0.0
        self.started_at = 0.0
        self.settle_until = 0.0
        self.max_progress_deg = 0.0
        self._publishing_own_estop = False

        self.create_timer(1.0 / self.rate_hz, self._timer)
        self._publish_state(False)
        self.get_logger().warning(
            'Mapping turn controller ready: discrete odom-closed-loop turns; '
            f'cmd={self.angular_command:.2f}rad/s, margins L={self.left_margin:.1f}deg R={self.right_margin:.1f}deg, '
            f'settle={self.settle_time:.2f}s.'
        )

    @staticmethod
    def _yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_yaw = self._yaw(msg.pose.pose.orientation)
        self.latest_odom_time = time.monotonic()

    def _publish_twist(self, angular: float) -> None:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _publish_state(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self.state_pub.publish(msg)

    def _send_estop(self) -> None:
        self._publishing_own_estop = True
        try:
            self.estop_pub.publish(Empty())
        finally:
            self._publishing_own_estop = False

    def _odom_is_fresh(self, now: float) -> bool:
        return (
            self.latest_yaw is not None
            and self.latest_odom_time is not None
            and now - self.latest_odom_time <= self.odom_timeout
        )

    def _request_cb(self, msg: Float64) -> None:
        requested = float(msg.data)
        magnitude = abs(requested)
        now = time.monotonic()

        if self.phase != 'IDLE':
            self.get_logger().warning(
                f'Ignoring turn request {requested:+.1f}deg: controller is {self.phase.lower()}.'
            )
            return
        if not (self.min_angle <= magnitude <= self.max_angle):
            self.get_logger().warning(
                f'Rejecting turn request {requested:+.1f}deg: allowed magnitude is '
                f'{self.min_angle:.1f}..{self.max_angle:.1f}deg.'
            )
            return
        if not self._odom_is_fresh(now):
            self.get_logger().error('Rejecting turn request: /odom is missing or stale.')
            return

        self.direction = 1.0 if requested > 0.0 else -1.0
        margin = self.left_margin if self.direction > 0.0 else self.right_margin
        stop_trigger_deg = max(1.0, magnitude - margin)

        self.target_deg = magnitude
        self.stop_trigger_rad = math.radians(stop_trigger_deg)
        self.start_yaw = float(self.latest_yaw)
        self.started_at = now
        self.max_progress_deg = 0.0
        self.phase = 'TURNING'
        self._publish_state(True)

        side = 'LEFT/CCW' if self.direction > 0.0 else 'RIGHT/CW'
        self.get_logger().warning(
            f'TURN START {side}: request={requested:+.1f}deg, '
            f'stop_trigger={self.direction * stop_trigger_deg:+.1f}deg.'
        )

    def _external_estop_cb(self, _msg: Empty) -> None:
        if self._publishing_own_estop:
            return
        if self.phase != 'IDLE':
            self.get_logger().warning('TURN CANCELLED by emergency stop.')
        self.phase = 'IDLE'
        self._publish_twist(0.0)
        self._publish_state(False)

    def _begin_settle(self, now: float, reason: str) -> None:
        self._publish_twist(0.0)
        self._send_estop()
        self.phase = 'SETTLING'
        self.settle_until = now + self.settle_time
        self.get_logger().warning(reason)

    def _timer(self) -> None:
        now = time.monotonic()

        if self.phase == 'IDLE':
            return

        if self.phase == 'SETTLING':
            self._publish_twist(0.0)
            if now >= self.settle_until:
                self.phase = 'IDLE'
                self._publish_state(False)
                self.get_logger().warning('TURN DONE: chassis settling interval complete.')
            return

        if not self._odom_is_fresh(now):
            self._begin_settle(now, 'TURN ABORT: /odom became stale; emergency stop sent.')
            return

        if now - self.started_at > self.turn_timeout:
            self._begin_settle(
                now,
                f'TURN ABORT: timeout; max_progress={self.max_progress_deg:.1f}deg; emergency stop sent.',
            )
            return

        progress = self.direction * angle_diff(float(self.latest_yaw), self.start_yaw)
        progress_deg = math.degrees(progress)
        self.max_progress_deg = max(self.max_progress_deg, progress_deg)

        if progress >= self.stop_trigger_rad:
            self._begin_settle(
                now,
                f'TURN TARGET: odom reached {progress_deg:.2f}deg; zero + emergency stop; settling.',
            )
            return

        self._publish_twist(self.direction * self.angular_command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingTurnController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_twist(0.0)
            node._send_estop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
