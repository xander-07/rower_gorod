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
    left_command_scale = LaunchConfiguration('left_command_scale')
    right_command_scale = LaunchConfiguration('right_command_scale')
    odom_meters_per_count = LaunchConfiguration('odom_meters_per_count')
    track_width = LaunchConfiguration('track_width')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_motion',
            default_value='false',
            description='Allow /cmd_vel to command the drive motors. Default is false.',
        ),
        DeclareLaunchArgument(
            'left_command_scale',
            default_value='0.965',
            description='Calibrated multiplicative gain for the left drive side.',
        ),
        DeclareLaunchArgument(
            'right_command_scale',
            default_value='1.035',
            description='Calibrated multiplicative gain for the right drive side.',
        ),
        DeclareLaunchArgument(
            'odom_meters_per_count',
            default_value='0.0104',
            description='Initial physical scale for cumulative odl/odr counters.',
        ),
        DeclareLaunchArgument(
            'track_width',
            default_value='0.172',
            description='Current skid-steer track width used by odometry; turn calibration is next.',
        ),
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
            parameters=[{
                'serial_port': '/dev/rower_lidar',
                'frame_id': 'laser',
            }],
        ),
        Node(
            package='rower_base_bridge',
            executable='base_bridge',
            name='rower_base_bridge',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rower_base',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'enable_motion': ParameterValue(enable_motion, value_type=bool),
                'left_command_scale': ParameterValue(left_command_scale, value_type=float),
                'right_command_scale': ParameterValue(right_command_scale, value_type=float),
                'odom_meters_per_count': ParameterValue(odom_meters_per_count, value_type=float),
                'track_width': ParameterValue(track_width, value_type=float),
            }],
        ),
    ])
