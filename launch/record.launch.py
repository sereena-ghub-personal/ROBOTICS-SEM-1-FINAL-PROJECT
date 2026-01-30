#!/usr/bin/env python3
"""
Launch file for Camera Recorder

Launches camera recorder node to save debug footage.

Usage:
    ros2 launch lane_following_robot record.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Camera Recorder Node
    camera_recorder_node = Node(
        package='lane_following_robot',
        executable='camera_recorder_node.py',
        name='camera_ros_recorder',
        output='screen',
    )

    return LaunchDescription([
        camera_recorder_node,
    ])
