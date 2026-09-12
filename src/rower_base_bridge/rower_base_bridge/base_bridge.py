#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

import serial


class RowerBaseBridge(Node):
    """Bridge the Waveshare UGV02 JSON UART protocol to ROS 2.

    Drive commands use Waveshare T=1 left/right velocity control. Pose odometry
    uses cumulative ``odl`` / ``odr`` counters from T=1001. Pure in-place
    angular commands and wheel-odometry yaw have separate skid-steer
    calibrations derived from LiDAR turn tests on the real robot.

    Motion remains disabled unless ``enable_motion`` is explicitly true.
    """

    def __init__(self) -> None:
        super().__init__('rower_base_bridge')

        self.declare_parameter('serial_port', '/dev/rower_base')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('track_width', 0.172)
        self.declare_parameter('odom_meters_per_count', 0.0102)
        self.declare_parameter('counter_step_limit', 20.0)
        self.declare_parameter('twist_window_sec', 0.60)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('command_rate_hz', 10.0)
        self.declare_parameter('cmd_timeout', 0.35)
        self.declare_parameter('max_wheel_speed', 0.25)

        # Straight-line drivetrain calibration.
        self.declare_parameter('left_command_scale', 0.965)
        self.declare_parameter('right_command_scale', 1.035)

        # In-place turn command calibration from LiDAR tests. Before
        # compensation, measured physical angular speed followed approximately:
        #   left : omega_real = 0.4804 * omega_raw - 0.1810
        #   right: omega_real = 0.4523 * omega_raw - 0.1463
        # Therefore desired physical omega is mapped back to the raw command by
        # gain * |omega_desired| + offset. This correction is intentionally
        # limited to near-zero linear velocity because moving arcs have not yet
        # been separately calibrated.
        self.declare_parameter('angular_compensation_enabled', True)
        self.declare_parameter('angular_compensation_linear_threshold', 0.02)
        self.declare_parameter('angular_command_deadband', 0.03)
        self.declare_parameter('angular_command_gain_left', 2.0817)
        self.declare_parameter('angular_command_offset_left', 0.3767)
        self.declare_parameter('angular_command_gain_right', 2.2110)
        self.declare_parameter('angular_command_offset_right', 0.3235)
        self.declare_parameter('angular_command_max_raw', 2.50)

        # Wheel counters substantially over-report chassis yaw during skid-steer
        # turns. Direction-specific scales are based on the 1.6 and 2.0 rad/s
        # LiDAR calibration runs. Linear distance remains unscaled by these.
        self.declare_parameter('odom_yaw_scale_left', 0.60)
        self.declare_parameter('odom_yaw_scale_right', 0.54)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.odom_meters_per_count = float(self.get_parameter('odom_meters_per_count').value)
        self.counter_step_limit = float(self.get_parameter('counter_step_limit').value)
        self.twist_window_sec = float(self.get_parameter('twist_window_sec').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        self.command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.left_command_scale = float(self.get_parameter('left_command_scale').value)
        self.right_command_scale = float(self.get_parameter('right_command_scale').value)

        self.angular_compensation_enabled = bool(
            self.get_parameter('angular_compensation_enabled').value
        )
        self.angular_compensation_linear_threshold = float(
            self.get_parameter('angular_compensation_linear_threshold').value
        )
        self.angular_command_deadband = float(
            self.get_parameter('angular_command_deadband').value
        )
        self.angular_command_gain_left = float(
            self.get_parameter('angular_command_gain_left').value
        )
        self.angular_command_offset_left = float(
            self.get_parameter('angular_command_offset_left').value
        )
        self.angular_command_gain_right = float(
            self.get_parameter('angular_command_gain_right').value
        )
        self.angular_command_offset_right = float(
            self.get_parameter('angular_command_offset_right').value
        )
        self.angular_command_max_raw = float(
            self.get_parameter('angular_command_max_raw').value
        )
        self.odom_yaw_scale_left = float(self.get_parameter('odom_yaw_scale_left').value)
        self.odom_yaw_scale_right = float(self.get_parameter('odom_yaw_scale_right').value)

        if self.track_width <= 0.0:
            raise ValueError('track_width must be > 0')
        if self.odom_meters_per_count <= 0.0:
            raise ValueError('odom_meters_per_count must be > 0')
        if self.counter_step_limit <= 0.0:
            raise ValueError('counter_step_limit must be > 0')
        if not (0.1 <= self.twist_window_sec <= 2.0):
            raise ValueError('twist_window_sec must be in 0.1..2.0')
        if self.command_rate_hz <= 0.0:
            raise ValueError('command_rate_hz must be > 0')
        if self.cmd_timeout <= 0.0:
            raise ValueError('cmd_timeout must be > 0')
        if self.max_wheel_speed <= 0.0:
            raise ValueError('max_wheel_speed must be > 0')
        if not (0.5 <= self.left_command_scale <= 1.5):
            raise ValueError('left_command_scale must be in 0.5..1.5')
        if not (0.5 <= self.right_command_scale <= 1.5):
            raise ValueError('right_command_scale must be in 0.5..1.5')
        if self.angular_compensation_linear_threshold < 0.0:
            raise ValueError('angular_compensation_linear_threshold must be >= 0')
        if self.angular_command_deadband < 0.0:
            raise ValueError('angular_command_deadband must be >= 0')
        if self.angular_command_gain_left <= 0.0 or self.angular_command_gain_right <= 0.0:
            raise ValueError('angular command gains must be > 0')
        if self.angular_command_offset_left < 0.0 or self.angular_command_offset_right < 0.0:
            raise ValueError('angular command offsets must be >= 0')
        if self.angular_command_max_raw <= 0.0:
            raise ValueError('angular_command_max_raw must be > 0')
        if not (0.0 < self.odom_yaw_scale_left <= 1.5):
            raise ValueError('odom_yaw_scale_left must be in 0..1.5')
        if not (0.0 < self.odom_yaw_scale_right <= 1.5):
            raise ValueError('odom_yaw_scale_right must be in 0..1.5')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.raw_feedback_pub = self.create_publisher(String, 'base/raw_feedback', 20)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.ser = serial.Serial(self.serial_port, self.baud, timeout=0)
        self.ser.reset_input_buffer()
        self._rx_buffer = bytearray()

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_counter_left: Optional[float] = None
        self._last_counter_right: Optional[float] = None
        self._counter_history = deque()
        self._last_cmd_time: Optional[float] = None
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._motion_warning_sent = False
        self._closed = False

        # moduleType is RAM-only in the Waveshare firmware, so force base-only
        # mode every time this bridge starts. This does not command the motors.
        self._send_json({'T': 4, 'cmd': 0})

        if self.enable_motion:
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
            f'Opened {self.serial_port} at {self.baud} baud; '
            f'track_width={self.track_width:.3f} m; '
            f'odom_scale={self.odom_meters_per_count:.5f} m/count; '
            f'drive_scales L={self.left_command_scale:.3f} R={self.right_command_scale:.3f}; '
            f'yaw_scales L={self.odom_yaw_scale_left:.3f} R={self.odom_yaw_scale_right:.3f}; '
            f'angular_compensation={"on" if self.angular_compensation_enabled else "off"}'
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

    def _calibrated_angular_command(self, linear: float, angular: float) -> float:
        """Map desired in-place chassis yaw rate to the raw skid-steer command."""
        if not self.angular_compensation_enabled:
            return angular
        if abs(linear) > self.angular_compensation_linear_threshold:
            return angular
        if abs(angular) < self.angular_command_deadband:
            return 0.0

        magnitude = abs(angular)
        if angular > 0.0:
            raw = (
                self.angular_command_gain_left * magnitude
                + self.angular_command_offset_left
            )
        else:
            raw = (
                self.angular_command_gain_right * magnitude
                + self.angular_command_offset_right
            )
        raw = min(raw, self.angular_command_max_raw)
        return math.copysign(raw, angular)

    def _yaw_scale(self, raw_dtheta: float) -> float:
        if raw_dtheta > 0.0:
            return self.odom_yaw_scale_left
        if raw_dtheta < 0.0:
            return self.odom_yaw_scale_right
        return 1.0

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

        calibrated_angular = self._calibrated_angular_command(linear, angular)

        left = (
            linear - calibrated_angular * self.track_width / 2.0
        ) * self.left_command_scale
        right = (
            linear + calibrated_angular * self.track_width / 2.0
        ) * self.right_command_scale
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
                self.get_logger().debug(f'Ignoring malformed/partial UART line: {text[:120]}')
                continue
            if msg.get('T') == 1001:
                self._handle_base_feedback(msg)

    @staticmethod
    def _finite_float(value) -> Optional[float]:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    def _counter_twist(self, now_mono: float, odl: float, odr: float) -> tuple[float, float]:
        """Estimate velocity from a rolling window of cumulative counters."""
        self._counter_history.append((now_mono, odl, odr))
        cutoff = now_mono - self.twist_window_sec
        while len(self._counter_history) > 2 and self._counter_history[1][0] <= cutoff:
            self._counter_history.popleft()

        if len(self._counter_history) < 2:
            return 0.0, 0.0

        t0, odl0, odr0 = self._counter_history[0]
        dt = now_mono - t0
        if dt <= 0.05:
            return 0.0, 0.0

        dl = (odl - odl0) * self.odom_meters_per_count
        dr = (odr - odr0) * self.odom_meters_per_count
        linear = (dl + dr) / (2.0 * dt)
        raw_angular = (dr - dl) / (self.track_width * dt)
        angular = raw_angular * self._yaw_scale(raw_angular)
        return linear, angular

    def _handle_base_feedback(self, msg: dict) -> None:
        left = self._finite_float(msg.get('L'))
        right = self._finite_float(msg.get('R'))
        if left is None or right is None:
            return

        odl = self._finite_float(msg.get('odl'))
        odr = self._finite_float(msg.get('odr'))

        raw_feedback = {
            'L': left,
            'R': right,
            'odl': msg.get('odl'),
            'odr': msg.get('odr'),
            'v': msg.get('v'),
        }
        raw_msg = String()
        raw_msg.data = json.dumps(raw_feedback, separators=(',', ':'))
        self.raw_feedback_pub.publish(raw_msg)

        now_mono = time.monotonic()
        linear = 0.0
        angular = 0.0

        if odl is not None and odr is not None:
            if self._last_counter_left is not None and self._last_counter_right is not None:
                dcl = odl - self._last_counter_left
                dcr = odr - self._last_counter_right

                if abs(dcl) <= self.counter_step_limit and abs(dcr) <= self.counter_step_limit:
                    dl = dcl * self.odom_meters_per_count
                    dr = dcr * self.odom_meters_per_count
                    ds = (dl + dr) / 2.0
                    raw_dtheta = (dr - dl) / self.track_width
                    dtheta = raw_dtheta * self._yaw_scale(raw_dtheta)
                    yaw_mid = self._yaw + dtheta * 0.5
                    self._x += ds * math.cos(yaw_mid)
                    self._y += ds * math.sin(yaw_mid)
                    self._yaw = math.atan2(
                        math.sin(self._yaw + dtheta),
                        math.cos(self._yaw + dtheta),
                    )
                else:
                    self.get_logger().warning(
                        f'Ignoring implausible odometer counter jump: '
                        f'dL={dcl:.1f} dR={dcr:.1f}'
                    )
                    self._counter_history.clear()

            self._last_counter_left = odl
            self._last_counter_right = odr
            linear, angular = self._counter_twist(now_mono, odl, odr)
        else:
            linear = (left + right) / 2.0
            raw_angular = (right - left) / self.track_width
            angular = raw_angular * self._yaw_scale(raw_angular)

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

        # Wheel yaw is skid-sensitive, so retain a deliberately conservative
        # yaw covariance even after empirical scaling.
        odom.pose.covariance[0] = 0.03
        odom.pose.covariance[7] = 0.03
        odom.pose.covariance[35] = 0.20
        odom.twist.covariance[0] = 0.04
        odom.twist.covariance[7] = 0.04
        odom.twist.covariance[35] = 0.20
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
