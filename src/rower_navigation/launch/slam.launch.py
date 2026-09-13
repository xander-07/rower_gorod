#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory('rower_navigation')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    params_file = os.path.join(navigation_share, 'config', 'slam_toolbox.yaml')
    official_online_async = os.path.join(
        slam_toolbox_share,
        'launch',
        'online_async_launch.py',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # A rotating 2D LiDAR produces motion-distorted 360-degree scans on a fast
    # skid-steer turn. Keep raw /scan for diagnostics, but only let SLAM see
    # /scan_slam after the chassis has settled rotationally.
    scan_gate = Node(
        package='rower_navigation',
        executable='scan_gate.py',
        name='rower_slam_scan_gate',
        output='screen',
        parameters=[{
            'input_scan_topic': '/scan',
            'output_scan_topic': '/scan_slam',
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'command_angular_threshold': 0.05,
            'odom_block_threshold': 0.12,
            'odom_release_threshold': 0.05,
            'command_freshness_sec': 0.30,
            'settle_time_sec': 0.60,
            'require_odom_before_open': True,
        }],
    )

    # slam_toolbox is a lifecycle node on ROS 2 Jazzy. Using its official
    # online_async launch is important: it configures and activates the node.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(official_online_async),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
            'slam_params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock. Keep false on the physical robot.',
        ),
        scan_gate,
        # Give the gate and odometry a moment to appear before slam_toolbox
        # subscribes to the filtered scan stream.
        TimerAction(period=0.8, actions=[slam]),
    ])
