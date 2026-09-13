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

    # During mapping, wheel odometry remains available on /wheel_odom for motor
    # control and diagnostics, but it no longer owns odom->base_link. RF2O uses
    # consecutive LiDAR scans to estimate the actual 2D robot motion, including
    # in-place rotation, and publishes the mapping /odom + odom->base_link TF.
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'enable_motion': enable_motion,
            'publish_tf': 'false',
            'odom_topic': '/wheel_odom',
            'odom_frame': 'wheel_odom',
        }.items(),
    )

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0,
        }],
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Keep turn stopping on the already LiDAR-calibrated wheel odometry. RF2O is
    # responsible for the mapping pose; the wheel counters are still the safest
    # short-horizon feedback for the motor stop threshold.
    turn_controller_script = '/workspace/rower_gorod/src/rower_navigation/scripts/turn_controller.py'
    turn_controller = ExecuteProcess(
        cmd=[
            'python3', turn_controller_script,
            '--ros-args',
            '-r', '__node:=rower_mapping_turn_controller',
            '-p', 'request_topic:=/mapping/turn_angle_deg',
            '-p', 'state_topic:=/mapping/turn_active',
            '-p', 'cmd_vel_topic:=/cmd_vel',
            '-p', 'odom_topic:=/wheel_odom',
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
        DeclareLaunchArgument('enable_motion', default_value='false', description='Allow /cmd_vel to drive the physical robot.'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock. Keep false on the physical robot.'),
        DeclareLaunchArgument('enable_web', default_value='true', description='Serve the local SLAM dashboard and rosbridge WebSocket.'),
        DeclareLaunchArgument('web_port', default_value='8080', description='HTTP port for the local browser dashboard.'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090', description='WebSocket port used by the local dashboard.'),
        robot_launch,
        rf2o,
        turn_controller,
        rosbridge,
        web_server,
        TimerAction(period=2.0, actions=[slam_launch]),
    ])
