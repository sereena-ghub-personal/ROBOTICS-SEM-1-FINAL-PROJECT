#!/usr/bin/env python3
"""
Launch file for Full Lane Following System

Launches all nodes including camera, track controller, and recorder.
The micro-ROS agent must be started separately.

Usage:
    ros2 launch lane_following_robot full_system.launch.py

With micro-ROS agent (in separate terminal):
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='0',
        description='Camera device ID'
    )

    base_velocity_arg = DeclareLaunchArgument(
        'base_velocity',
        default_value='0.30',
        description='Base forward velocity (0.0 to 1.0)'
    )

    enable_recording_arg = DeclareLaunchArgument(
        'enable_recording',
        default_value='true',
        description='Enable video recording'
    )

    # Camera Publisher Node
    camera_publisher_node = Node(
        package='lane_following_robot',
        executable='camera_publisher_node.py',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('camera_device'),
            'topic': 'camera_image',
            'debug_topic': 'camera_image_debug',
            'fps': 30.0,
            'width': 640,
            'height': 480,
            'roi_height_ratio': 0.55,
        }]
    )

    # Track Controller Node
    track_controller_node = Node(
        package='lane_following_robot',
        executable='track_controller_node.py',
        name='track_controller',
        output='screen',
        parameters=[{
            'image_topic': 'camera_image',
            'cmd_vel_topic': 'cmd_vel',
            'base_v': LaunchConfiguration('base_velocity'),
            'kp': 0.0008,
            'max_w': 0.50,
            'deadband_px': 25.0,
            'err_alpha': 0.10,
            'process_every_n_frames': 2,
        }]
    )

    # Camera Recorder Node
    camera_recorder_node = Node(
        package='lane_following_robot',
        executable='camera_recorder_node.py',
        name='camera_ros_recorder',
        output='screen',
    )

    return LaunchDescription([
        camera_device_arg,
        base_velocity_arg,
        enable_recording_arg,
        camera_publisher_node,
        track_controller_node,
        camera_recorder_node,
    ])
