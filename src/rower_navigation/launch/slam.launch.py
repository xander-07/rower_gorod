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
    # Python instead of relying on ROS libexec discovery; this also works with
    # an existing --symlink-install overlay when a new helper script was added
    # after the package was first configured.
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
            '-p', 'command_angular_threshold:=0.05',
            '-p', 'odom_block_threshold:=0.12',
            '-p', 'odom_release_threshold:=0.05',
            '-p', 'command_freshness_sec:=0.30',
            '-p', 'settle_time_sec:=0.60',
            '-p', 'require_odom_before_open:=true',
        ],
        output='screen',
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
