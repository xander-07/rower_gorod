#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('rower_bringup')
    navigation_share = get_package_share_directory('rower_navigation')

    enable_motion = LaunchConfiguration('enable_motion')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_foxglove = LaunchConfiguration('enable_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'enable_motion': enable_motion,
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(enable_foxglove),
        parameters=[{
            'address': '0.0.0.0',
            'port': ParameterValue(foxglove_port, value_type=int),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_motion',
            default_value='false',
            description='Allow /cmd_vel to drive the physical robot. Default false for safe SLAM startup.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock. Keep false on the physical robot.',
        ),
        DeclareLaunchArgument(
            'enable_foxglove',
            default_value='true',
            description='Start Foxglove WebSocket bridge for remote visualization.',
        ),
        DeclareLaunchArgument(
            'foxglove_port',
            default_value='8765',
            description='TCP port for the Foxglove WebSocket bridge.',
        ),
        robot_launch,
        foxglove_bridge,
        # Give the LiDAR, odometry and static TF a moment to appear before SLAM
        # starts consuming /scan. This also avoids noisy startup TF warnings.
        TimerAction(period=2.0, actions=[slam_launch]),
    ])
