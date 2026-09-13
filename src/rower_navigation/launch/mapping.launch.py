#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('rower_bringup')
    navigation_share = get_package_share_directory('rower_navigation')
    web_dir = os.path.join(navigation_share, 'web')

    enable_motion = LaunchConfiguration('enable_motion')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_web = LaunchConfiguration('enable_web')
    web_port = LaunchConfiguration('web_port')
    rosbridge_port = LaunchConfiguration('rosbridge_port')

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

    # The workspace is bind-mounted at this stable path by run_ros2_docker.sh.
    # Run the mapping turn helper directly so a newly added script does not
    # depend on an already-configured colcon libexec overlay.
    turn_controller_script = '/workspace/rower_gorod/src/rower_navigation/scripts/turn_controller.py'
    turn_controller = ExecuteProcess(
        cmd=[
            'python3', turn_controller_script,
            '--ros-args',
            '-r', '__node:=rower_mapping_turn_controller',
            '-p', 'request_topic:=/mapping/turn_angle_deg',
            '-p', 'state_topic:=/mapping/turn_active',
            '-p', 'cmd_vel_topic:=/cmd_vel',
            '-p', 'odom_topic:=/odom',
            '-p', 'emergency_topic:=/base/emergency_stop',
            '-p', 'angular_command:=0.40',
            '-p', 'left_stop_margin_deg:=3.0',
            '-p', 'right_stop_margin_deg:=2.0',
            '-p', 'control_rate_hz:=50.0',
            '-p', 'odom_timeout_sec:=0.20',
            '-p', 'turn_timeout_sec:=8.0',
            '-p', 'settle_time_sec:=0.70',
        ],
        output='screen',
    )

    # Local-only browser dashboard transport. No cloud/external website is
    # required: the browser connects directly to the Raspberry Pi.
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        condition=IfCondition(enable_web),
        parameters=[{
            'address': '0.0.0.0',
            'port': ParameterValue(rosbridge_port, value_type=int),
            'max_message_size': 10000000,
        }],
    )

    web_server = ExecuteProcess(
        cmd=[
            'python3', '-m', 'http.server', web_port,
            '--bind', '0.0.0.0',
            '--directory', web_dir,
        ],
        output='screen',
        condition=IfCondition(enable_web),
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
            'enable_web',
            default_value='true',
            description='Serve the local SLAM dashboard and rosbridge WebSocket.',
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='HTTP port for the local browser dashboard.',
        ),
        DeclareLaunchArgument(
            'rosbridge_port',
            default_value='9090',
            description='WebSocket port used by the local dashboard.',
        ),
        robot_launch,
        turn_controller,
        rosbridge,
        web_server,
        # Give LiDAR, odometry and static TF a moment to appear before SLAM.
        TimerAction(period=2.0, actions=[slam_launch]),
    ])
