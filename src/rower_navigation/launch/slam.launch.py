#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    # slam_toolbox is a lifecycle node on ROS 2 Jazzy.  Using its official
    # online_async launch is important: it configures and activates the node.
    # Starting async_slam_toolbox_node as a plain Node leaves /slam_toolbox
    # visible in `ros2 node list`, but no /map or map->odom TF is produced.
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
        slam,
    ])
