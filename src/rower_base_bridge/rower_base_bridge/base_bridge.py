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
from std_msgs.msg import Empty, String
from tf2_ros import TransformBroadcaster

import serial


class RowerBaseBridge(Node):
    """Bridge the Waveshare UGV02 JSON UART protocol to ROS 2.

    The empirically validated default drive mode is raw Waveshare PWM (T=11).
    It bypasses the lower controller's unstable low-speed velocity PID that
    caused visible left/right weaving with T=1/T=13 on the real six-wheel base.

    Pose odometry still comes from cumulative ``odl`` / ``odr`` counters in
    T=1001 feedback. For the current mapping phase, mixed linear+angular commands
    are intentionally converted to stop/rotate/straight motion: any meaningful
    angular command gets priority and becomes an in-place turn.

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
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout', 0.35)

        # Empirical raw-PWM drive path. On the real robot:
        #   T=11 L=40 R=40 -> straight, counters 14/14, ~0.116 m/s feedback.
        #   T=11 L=-80 R=80 -> calibrated left/CCW in-place turn.
        #   T=11 L=80 R=-80 -> calibrated right/CW in-place turn.
        self.declare_parameter('drive_mode', 'pwm')
        self.declare_parameter('pwm_linear_reference_speed', 0.10)
        self.declare_parameter('pwm_linear_reference', 40)
        self.declare_parameter('pwm_linear_min', 40)
        self.declare_parameter('pwm_linear_max', 70)
        self.declare_parameter('pwm_turn_left', 80)
        self.declare_parameter('pwm_turn_right', 80)
        self.declare_parameter('pwm_linear_deadband', 0.02)
        self.declare_parameter('pwm_angular_deadband', 0.05)
        self.declare_parameter('pwm_turn_linear_threshold', 0.02)

        # Legacy T=1 velocity-PID path retained only as a fallback diagnostic.
        self.declare_parameter('max_wheel_speed', 0.25)
        self.declare_parameter('wheel_accel_limit', 0.12)
        self.declare_parameter('wheel_decel_limit', 0.18)
        self.declare_parameter('left_command_scale', 1.0)
        self.declare_parameter('right_command_scale', 1.0)
        self.declare_parameter('angular_compensation_enabled', True)
        self.declare_parameter('angular_compensation_linear_threshold', 0.02)
        self.declare_parameter('angular_command_deadband', 0.03)
        self.declare_parameter('angular_command_gain_left', 2.0817)
        self.declare_parameter('angular_command_offset_left', 0.3767)
        self.declare_parameter('angular_command_gain_right', 2.2110)
        self.declare_parameter('angular_command_offset_right', 0.3235)
        self.declare_parameter('angular_command_max_raw', 2.50)

        # LiDAR-validated wheel-counter yaw scales for the raw-PWM drive path.
        self.declare_parameter('odom_yaw_scale_left', 0.447)
        self.declare_parameter('odom_yaw_scale_right', 0.44)

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

        self.drive_mode = str(self.get_parameter('drive_mode').value).strip().lower()
        self.pwm_linear_reference_speed = float(self.get_parameter('pwm_linear_reference_speed').value)
        self.pwm_linear_reference = int(self.get_parameter('pwm_linear_reference').value)
        self.pwm_linear_min = int(self.get_parameter('pwm_linear_min').value)
        self.pwm_linear_max = int(self.get_parameter('pwm_linear_max').value)
        self.pwm_turn_left = int(self.get_parameter('pwm_turn_left').value)
        self.pwm_turn_right = int(self.get_parameter('pwm_turn_right').value)
        self.pwm_linear_deadband = float(self.get_parameter('pwm_linear_deadband').value)
        self.pwm_angular_deadband = float(self.get_parameter('pwm_angular_deadband').value)
        self.pwm_turn_linear_threshold = float(self.get_parameter('pwm_turn_linear_threshold').value)

        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.wheel_accel_limit = float(self.get_parameter('wheel_accel_limit').value)
        self.wheel_decel_limit = float(self.get_parameter('wheel_decel_limit').value)
        self.left_command_scale = float(self.get_parameter('left_command_scale').value)
        self.right_command_scale = float(self.get_parameter('right_command_scale').value)
        self.angular_compensation_enabled = bool(self.get_parameter('angular_compensation_enabled').value)
        self.angular_compensation_linear_threshold = float(self.get_parameter('angular_compensation_linear_threshold').value)
        self.angular_command_deadband = float(self.get_parameter('angular_command_deadband').value)
        self.angular_command_gain_left = float(self.get_parameter('angular_command_gain_left').value)
        self.angular_command_offset_left = float(self.get_parameter('angular_command_offset_left').value)
        self.angular_command_gain_right = float(self.get_parameter('angular_command_gain_right').value)
        self.angular_command_offset_right = float(self.get_parameter('angular_command_offset_right').value)
        self.angular_command_max_raw = float(self.get_parameter('angular_command_max_raw').value)
        self.odom_yaw_scale_left = float(self.get_parameter('odom_yaw_scale_left').value)
        self.odom_yaw_scale_right = float(self.get_parameter('odom_yaw_scale_right').value)

        if self.drive_mode not in ('pwm', 'velocity_pid'):
            raise ValueError("drive_mode must be 'pwm' or 'velocity_pid'")
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
        if self.pwm_linear_reference_speed <= 0.0:
            raise ValueError('pwm_linear_reference_speed must be > 0')
        if not (1 <= self.pwm_linear_reference <= 255):
            raise ValueError('pwm_linear_reference must be in 1..255')
        if not (1 <= self.pwm_linear_min <= self.pwm_linear_max <= 255):
            raise ValueError('require 1 <= pwm_linear_min <= pwm_linear_max <= 255')
        if not (1 <= self.pwm_turn_left <= 255 and 1 <= self.pwm_turn_right <= 255):
            raise ValueError('pwm_turn_left/right must be in 1..255')
        if self.pwm_linear_deadband < 0.0 or self.pwm_angular_deadband < 0.0:
            raise ValueError('PWM deadbands must be >= 0')
        if self.pwm_turn_linear_threshold < 0.0:
            raise ValueError('pwm_turn_linear_threshold must be >= 0')
        if self.max_wheel_speed <= 0.0:
            raise ValueError('max_wheel_speed must be > 0')
        if self.wheel_accel_limit <= 0.0 or self.wheel_decel_limit <= 0.0:
            raise ValueError('wheel accel/decel limits must be > 0')
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
        self.estop_sub = self.create_subscription(Empty, 'base/emergency_stop', self._emergency_stop_cb, 10)
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
        self._sent_left = 0.0
        self._sent_right = 0.0
        self._last_command_tick = time.monotonic()
        self._motion_warning_sent = False
        self._mixed_command_warning_sent = False
        self._closed = False

        self._send_json({'T': 4, 'cmd': 0})

        if self.enable_motion:
            self._send_json({'T': 999})
            if self.drive_mode == 'pwm':
                self.get_logger().warning('MOTION ENABLED: cmd_vel uses raw T=11 PWM; Waveshare speed PID is bypassed.')
            else:
                self.get_logger().warning('MOTION ENABLED: legacy T=1 velocity-PID mode is active.')
        else:
            self.get_logger().info('Motion is DISABLED (enable_motion:=false). Telemetry/odometry only.')

        self.create_timer(0.01, self._read_serial)
        self.create_timer(1.0 / self.command_rate_hz, self._command_timer)

        if self.drive_mode == 'pwm':
            drive_details = (
                f'PWM straight ref={self.pwm_linear_reference}@{self.pwm_linear_reference_speed:.3f}m/s '
                f'range={self.pwm_linear_min}..{self.pwm_linear_max}; '
                f'turn L={self.pwm_turn_left} R={self.pwm_turn_right}'
            )
        else:
            drive_details = (
                f'T=1 scales L={self.left_command_scale:.3f} R={self.right_command_scale:.3f}; '
                f'slew accel={self.wheel_accel_limit:.3f} decel={self.wheel_decel_limit:.3f}m/s^2'
            )

        self.get_logger().info(
            f'Opened {self.serial_port} at {self.baud} baud; drive_mode={self.drive_mode}; '
            f'{drive_details}; track_width={self.track_width:.3f}m; '
            f'odom_scale={self.odom_meters_per_count:.5f}m/count; '
            f'yaw_scales L={self.odom_yaw_scale_left:.3f} R={self.odom_yaw_scale_right:.3f}'
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
            self.get_logger().warning('cmd_vel received but motion is disabled. Restart with enable_motion:=true only when ready.')
            self._motion_warning_sent = True

    def _send_motor_stop(self) -> None:
        self._sent_left = 0.0
        self._sent_right = 0.0
        if self.drive_mode == 'pwm':
            self._send_json({'T': 11, 'L': 0, 'R': 0})
        else:
            self._send_json({'T': 1, 'L': 0.0, 'R': 0.0})

    def _emergency_stop_cb(self, _msg: Empty) -> None:
        if not self.enable_motion or self._closed:
            return
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._last_cmd_time = time.monotonic()
        try:
            self._send_motor_stop()
            self.get_logger().warning('EMERGENCY STOP: motor command forced to zero.')
        except serial.SerialException:
            pass

    def _pwm_for_linear_speed(self, linear: float) -> int:
        magnitude = abs(linear)
        if magnitude < self.pwm_linear_deadband:
            return 0
        raw = int(round(self.pwm_linear_reference * magnitude / self.pwm_linear_reference_speed))
        raw = max(self.pwm_linear_min, min(self.pwm_linear_max, raw))
        return raw if linear > 0.0 else -raw

    def _pwm_command(self, linear: float, angular: float) -> tuple[int, int]:
        if abs(angular) >= self.pwm_angular_deadband:
            if abs(linear) > self.pwm_turn_linear_threshold and not self._mixed_command_warning_sent:
                self.get_logger().warning('Mixed linear+angular cmd_vel in PWM mapping mode: linear is ignored; turning in place.')
                self._mixed_command_warning_sent = True
            if angular > 0.0:
                return -self.pwm_turn_left, self.pwm_turn_left
            return self.pwm_turn_right, -self.pwm_turn_right
        pwm = self._pwm_for_linear_speed(linear)
        return pwm, pwm

    def _calibrated_angular_command(self, linear: float, angular: float) -> float:
        if not self.angular_compensation_enabled:
            return angular
        if abs(linear) > self.angular_compensation_linear_threshold:
            return angular
        if abs(angular) < self.angular_command_deadband:
            return 0.0
        magnitude = abs(angular)
        if angular > 0.0:
            raw = self.angular_command_gain_left * magnitude + self.angular_command_offset_left
        else:
            raw = self.angular_command_gain_right * magnitude + self.angular_command_offset_right
        return math.copysign(min(raw, self.angular_command_max_raw), angular)

    def _yaw_scale(self, raw_dtheta: float) -> float:
        if raw_dtheta > 0.0:
            return self.odom_yaw_scale_left
        if raw_dtheta < 0.0:
            return self.odom_yaw_scale_right
        return 1.0

    @staticmethod
    def _slew_wheel(current: float, target: float, dt: float, accel: float, decel: float) -> float:
        if dt <= 0.0 or current == target:
            return target
        if current * target < 0.0:
            step = decel * dt
            if abs(current) <= step:
                return 0.0
            return current - math.copysign(step, current)
        limit = accel if abs(target) > abs(current) else decel
        step = limit * dt
        delta = target - current
        if abs(delta) <= step:
            return target
        return current + math.copysign(step, delta)

    def _command_timer(self) -> None:
        if not self.enable_motion or self._closed:
            return
        now = time.monotonic()
        dt = max(0.0, min(0.25, now - self._last_command_tick))
        self._last_command_tick = now
        stale = self._last_cmd_time is None or (now - self._last_cmd_time) > self.cmd_timeout
        if stale:
            self._send_motor_stop()
            return

        linear = self._cmd_linear
        angular = self._cmd_angular
        if self.drive_mode == 'pwm':
            left_pwm, right_pwm = self._pwm_command(linear, angular)
            self._sent_left = float(left_pwm)
            self._sent_right = float(right_pwm)
            self._send_json({'T': 11, 'L': left_pwm, 'R': right_pwm})
            return

        calibrated_angular = self._calibrated_angular_command(linear, angular)
        target_left = (linear - calibrated_angular * self.track_width / 2.0) * self.left_command_scale
        target_right = (linear + calibrated_angular * self.track_width / 2.0) * self.right_command_scale
        target_left = max(-self.max_wheel_speed, min(self.max_wheel_speed, target_left))
        target_right = max(-self.max_wheel_speed, min(self.max_wheel_speed, target_right))
        self._sent_left = self._slew_wheel(self._sent_left, target_left, dt, self.wheel_accel_limit, self.wheel_decel_limit)
        self._sent_right = self._slew_wheel(self._sent_right, target_right, dt, self.wheel_accel_limit, self.wheel_decel_limit)
        self._send_json({'T': 1, 'L': round(self._sent_left, 4), 'R': round(self._sent_right, 4)})

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
        return linear, raw_angular * self._yaw_scale(raw_angular)

    def _handle_base_feedback(self, msg: dict) -> None:
        left = self._finite_float(msg.get('L'))
        right = self._finite_float(msg.get('R'))
        if left is None or right is None:
            return
        odl = self._finite_float(msg.get('odl'))
        odr = self._finite_float(msg.get('odr'))

        raw_msg = String()
        raw_msg.data = json.dumps({'L': left, 'R': right, 'odl': msg.get('odl'), 'odr': msg.get('odr'), 'v': msg.get('v')}, separators=(',', ':'))
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
                    self._yaw = math.atan2(math.sin(self._yaw + dtheta), math.cos(self._yaw + dtheta))
                else:
                    self.get_logger().warning(f'Ignoring implausible odometer counter jump: dL={dcl:.1f} dR={dcr:.1f}')
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
                self._send_motor_stop()
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
