#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

    # The project workspace is bind-mounted into the ROS container at this
    # stable path by scripts/run_ros2_docker.sh. Run the gate explicitly with
    # Python instead of relying on ROS libexec discovery.
    scan_gate_script = '/workspace/rower_gorod/src/rower_navigation/scripts/scan_gate.py'

    scan_gate = ExecuteProcess(
        cmd=[
            'python3', scan_gate_script,
            '--ros-args',
            '-r', '__node:=rower_slam_scan_gate',
            '-p', 'input_scan_topic:=/scan',
            '-p', 'output_scan_topic:=/scan_slam',
            '-p', 'cmd_vel_topic:=/cmd_vel',
            '-p', 'odom_topic:=/odom',
            '-p', 'turn_request_topic:=/mapping/turn_angle_deg',
            '-p', 'command_angular_threshold:=0.05',
            '-p', 'odom_block_threshold:=0.12',
            '-p', 'odom_release_threshold:=0.05',
            '-p', 'command_freshness_sec:=0.30',
            '-p', 'settle_time_sec:=0.60',
            '-p', 'require_odom_before_open:=true',
        ],
        output='screen',
    )

    # During map acquisition, map is intentionally the same metric frame as
    # odom. slam_toolbox is configured with transform_publish_period=0.0, so it
    # cannot move map->odom underneath the robot while matching scans.
    map_to_odom_identity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        output='screen',
    )

    # slam_toolbox is a lifecycle node on ROS 2 Jazzy. It now only paints the
    # occupancy grid at poses supplied by the calibrated odometry; scan matching
    # and loop-closure pose corrections are disabled in slam_toolbox.yaml.
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
        map_to_odom_identity,
        scan_gate,
        # Give fixed TF, gate and odometry a moment to appear before slam_toolbox
        # subscribes to the filtered scan stream.
        TimerAction(period=0.8, actions=[slam]),
    ])
