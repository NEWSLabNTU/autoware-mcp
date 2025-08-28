#!/usr/bin/env python3
"""ROS2 Talker-Listener Demo Launch File"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for talker-listener demo."""

    return LaunchDescription(
        [
            # Talker node
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="talker",
                output="screen",
            ),
            # Listener node
            Node(
                package="demo_nodes_cpp",
                executable="listener",
                name="listener",
                output="screen",
            ),
        ]
    )
