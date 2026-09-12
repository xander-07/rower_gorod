#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster

import serial


class RowerBaseBridge(Node):
    """Bridge the Waveshare UGV02 JSON UART protocol to ROS 2.

    By default this node is READ-ONLY with respect to drive motion. It sends the
    safe module-selection command T=4/cmd=0 at startup, but it will not reset the
    emergency stop and will not send T=1 wheel commands unless enable_motion is
    explicitly set to true.
    """

    def __init__(self) -> None:
        super().__init__('rower_base_bridge')

        self.declare_parameter('serial_port', '/dev/rower_base')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('track_width', 0.172)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('command_rate_hz', 10.0)
        self.declare_parameter('cmd_timeout', 0.35)
        self.declare_parameter('max_wheel_speed', 0.25)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        self.command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)

        if self.track_width <= 0.0:
            raise ValueError('track_width must be > 0')
        if self.command_rate_hz <= 0.0:
            raise ValueError('command_rate_hz must be > 0')
        if self.cmd_timeout <= 0.0:
            raise ValueError('cmd_timeout must be > 0')
        if self.max_wheel_speed <= 0.0:
            raise ValueError('max_wheel_speed must be > 0')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.ser = serial.Serial(self.serial_port, self.baud, timeout=0)
        self.ser.reset_input_buffer()
        self._rx_buffer = bytearray()

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_feedback_time: Optional[float] = None
        self._last_cmd_time: Optional[float] = None
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._motion_warning_sent = False
        self._closed = False

        # moduleType is RAM-only in the Waveshare firmware, so force base-only
        # mode every time this bridge starts. This does not command the motors.
        self._send_json({'T': 4, 'cmd': 0})

        if self.enable_motion:
            # Previous diagnostics deliberately end with T=0 emergency stop.
            # Motion is only armed when the operator explicitly requests it.
            self._send_json({'T': 999})
            self.get_logger().warning(
                'MOTION ENABLED: cmd_vel will be converted to T=1 wheel-speed commands.'
            )
        else:
            self.get_logger().info(
                'Motion is DISABLED (enable_motion:=false). Telemetry/odometry only.'
            )

        self.create_timer(0.01, self._read_serial)
        self.create_timer(1.0 / self.command_rate_hz, self._command_timer)

        self.get_logger().info(
            f'Opened {self.serial_port} at {self.baud} baud; track_width={self.track_width:.3f} m'
        )

    def _send_json(self, payload: dict) -> None:
        wire = (json.dumps(payload, separators=(',', ':')) + '\n').encode('utf-8')
        self.ser.write(wire)
        self.ser.flush()

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self._cmd_linear = float(msg.linear.x)
        self._cmd_angular = float(msg.angular.z)
        self._last_cmd_time = time.monotonic()
        if not self.enable_motion and not self._motion_warning_sent:
            self.get_logger().warning(
                'cmd_vel received but motion is disabled. Restart with '
                '-p enable_motion:=true only when you are ready to move the robot.'
            )
            self._motion_warning_sent = True

    def _command_timer(self) -> None:
        if not self.enable_motion or self._closed:
            return

        now = time.monotonic()
        if self._last_cmd_time is None or (now - self._last_cmd_time) > self.cmd_timeout:
            linear = 0.0
            angular = 0.0
        else:
            linear = self._cmd_linear
            angular = self._cmd_angular

        left = linear - angular * self.track_width / 2.0
        right = linear + angular * self.track_width / 2.0
        left = max(-self.max_wheel_speed, min(self.max_wheel_speed, left))
        right = max(-self.max_wheel_speed, min(self.max_wheel_speed, right))
        self._send_json({'T': 1, 'L': round(left, 4), 'R': round(right, 4)})

    def _read_serial(self) -> None:
        if self._closed:
            return
        waiting = self.ser.in_waiting
        if waiting <= 0:
            return

        self._rx_buffer.extend(self.ser.read(waiting))
        while b'\n' in self._rx_buffer:
            raw, _, remainder = self._rx_buffer.partition(b'\n')
            self._rx_buffer = bytearray(remainder)
            text = raw.decode('utf-8', errors='replace').strip()
            if not text:
                continue
            try:
                msg = json.loads(text)
            except json.JSONDecodeError:
                # Opening a streaming UART can begin in the middle of a line.
                self.get_logger().debug(f'Ignoring malformed/partial UART line: {text[:120]}')
                continue
            if msg.get('T') == 1001:
                self._handle_base_feedback(msg)

    def _handle_base_feedback(self, msg: dict) -> None:
        try:
            left = float(msg.get('L', 0.0))
            right = float(msg.get('R', 0.0))
        except (TypeError, ValueError):
            return

        now_mono = time.monotonic()
        if self._last_feedback_time is not None:
            dt = now_mono - self._last_feedback_time
            if 0.0 < dt < 1.0:
                linear = (left + right) / 2.0
                angular = (right - left) / self.track_width
                yaw_mid = self._yaw + angular * dt * 0.5
                self._x += linear * math.cos(yaw_mid) * dt
                self._y += linear * math.sin(yaw_mid) * dt
                self._yaw = math.atan2(
                    math.sin(self._yaw + angular * dt),
                    math.cos(self._yaw + angular * dt),
                )
        self._last_feedback_time = now_mono

        linear = (left + right) / 2.0
        angular = (right - left) / self.track_width
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular

        # Initial conservative planar covariance; tune after floor calibration.
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.20
        odom.twist.covariance[0] = 0.03
        odom.twist.covariance[7] = 0.03
        odom.twist.covariance[35] = 0.10
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        raw_v = msg.get('v')
        try:
            voltage = float(raw_v) / 100.0
        except (TypeError, ValueError):
            voltage = math.nan

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = voltage
        battery.temperature = math.nan
        battery.current = math.nan
        battery.charge = math.nan
        battery.capacity = math.nan
        battery.design_capacity = math.nan
        battery.percentage = math.nan
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        battery.present = True
        self.battery_pub.publish(battery)

    def stop_controller(self) -> None:
        if self._closed:
            return
        if self.enable_motion:
            try:
                self._send_json({'T': 1, 'L': 0.0, 'R': 0.0})
                self._send_json({'T': 0})
            except serial.SerialException:
                pass
        self._closed = True
        if self.ser.is_open:
            self.ser.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RowerBaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_controller()
        node.destroy_node()
        # ROS 2's signal handler may already have shut down the default context
        # after Ctrl+C. Avoid calling shutdown twice, which raises RCLError.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
