#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    description_share = get_package_share_directory('rower_description')
    xacro_file = os.path.join(description_share, 'urdf', 'rower.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    enable_motion = LaunchConfiguration('enable_motion')
    drive_mode = LaunchConfiguration('drive_mode')
    pwm_linear_reference = LaunchConfiguration('pwm_linear_reference')
    pwm_turn_left = LaunchConfiguration('pwm_turn_left')
    pwm_turn_right = LaunchConfiguration('pwm_turn_right')
    left_command_scale = LaunchConfiguration('left_command_scale')
    right_command_scale = LaunchConfiguration('right_command_scale')
    odom_meters_per_count = LaunchConfiguration('odom_meters_per_count')
    track_width = LaunchConfiguration('track_width')
    odom_yaw_scale_left = LaunchConfiguration('odom_yaw_scale_left')
    odom_yaw_scale_right = LaunchConfiguration('odom_yaw_scale_right')
    command_rate_hz = LaunchConfiguration('command_rate_hz')
    wheel_accel_limit = LaunchConfiguration('wheel_accel_limit')
    wheel_decel_limit = LaunchConfiguration('wheel_decel_limit')
    publish_tf = LaunchConfiguration('publish_tf')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_frame = LaunchConfiguration('odom_frame')

    return LaunchDescription([
        DeclareLaunchArgument('enable_motion', default_value='false', description='Allow /cmd_vel to command the physical drive motors.'),
        DeclareLaunchArgument('drive_mode', default_value='pwm', description='Drive backend: pwm or velocity_pid.'),
        DeclareLaunchArgument('pwm_linear_reference', default_value='40', description='Raw PWM for straight mapping speed.'),
        DeclareLaunchArgument('pwm_turn_left', default_value='80', description='Raw PWM magnitude for left/CCW in-place turning.'),
        DeclareLaunchArgument('pwm_turn_right', default_value='80', description='Raw PWM magnitude for right/CW in-place turning.'),
        DeclareLaunchArgument('left_command_scale', default_value='1.0', description='Legacy T=1 left drive gain; ignored in pwm mode.'),
        DeclareLaunchArgument('right_command_scale', default_value='1.0', description='Legacy T=1 right drive gain; ignored in pwm mode.'),
        DeclareLaunchArgument('odom_meters_per_count', default_value='0.0102', description='Calibrated physical scale for cumulative odl/odr counters.'),
        DeclareLaunchArgument('track_width', default_value='0.172', description='Geometric skid-steer track width.'),
        DeclareLaunchArgument('odom_yaw_scale_left', default_value='0.447', description='LiDAR-calibrated left/CCW wheel-counter yaw scale.'),
        DeclareLaunchArgument('odom_yaw_scale_right', default_value='0.44', description='LiDAR-calibrated right/CW wheel-counter yaw scale.'),
        DeclareLaunchArgument('command_rate_hz', default_value='20.0', description='Motor command refresh rate.'),
        DeclareLaunchArgument('wheel_accel_limit', default_value='0.12', description='Legacy T=1 wheel acceleration limit.'),
        DeclareLaunchArgument('wheel_decel_limit', default_value='0.18', description='Legacy T=1 wheel deceleration limit.'),
        DeclareLaunchArgument('publish_tf', default_value='true', description='Publish odom->base_link TF from the wheel base bridge.'),
        DeclareLaunchArgument('odom_topic', default_value='/odom', description='Output topic for wheel odometry.'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Frame id written into wheel odometry.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rower_lidar',
            executable='lidar_node',
            name='rower_lidar',
            output='screen',
            parameters=[{'serial_port': '/dev/rower_lidar', 'frame_id': 'laser'}],
        ),
        Node(
            package='rower_base_bridge',
            executable='base_bridge',
            name='rower_base_bridge',
            output='screen',
            remappings=[('odom', odom_topic)],
            parameters=[{
                'serial_port': '/dev/rower_base',
                'base_frame': 'base_link',
                'odom_frame': ParameterValue(odom_frame, value_type=str),
                'publish_tf': ParameterValue(publish_tf, value_type=bool),
                'enable_motion': ParameterValue(enable_motion, value_type=bool),
                'drive_mode': ParameterValue(drive_mode, value_type=str),
                'pwm_linear_reference': ParameterValue(pwm_linear_reference, value_type=int),
                'pwm_turn_left': ParameterValue(pwm_turn_left, value_type=int),
                'pwm_turn_right': ParameterValue(pwm_turn_right, value_type=int),
                'left_command_scale': ParameterValue(left_command_scale, value_type=float),
                'right_command_scale': ParameterValue(right_command_scale, value_type=float),
                'odom_meters_per_count': ParameterValue(odom_meters_per_count, value_type=float),
                'track_width': ParameterValue(track_width, value_type=float),
                'odom_yaw_scale_left': ParameterValue(odom_yaw_scale_left, value_type=float),
                'odom_yaw_scale_right': ParameterValue(odom_yaw_scale_right, value_type=float),
                'command_rate_hz': ParameterValue(command_rate_hz, value_type=float),
                'wheel_accel_limit': ParameterValue(wheel_accel_limit, value_type=float),
                'wheel_decel_limit': ParameterValue(wheel_decel_limit, value_type=float),
            }],
        ),
    ])
