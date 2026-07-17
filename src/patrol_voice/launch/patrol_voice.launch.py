#!/usr/bin/env python3
"""patrol_voice 启动文件"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="patrol_voice",
            executable="voice_node",
            name="patrol_voice",
            output="screen",
            parameters=[{
                "serial_port": "/dev/myspeech",
                "voice_enabled": True,
                "command_enabled": True,
            }],
        ),
    ])
