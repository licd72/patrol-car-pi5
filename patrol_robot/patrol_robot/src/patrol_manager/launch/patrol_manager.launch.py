#!/usr/bin/env python3
"""patrol_manager 启动文件"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="patrol_manager",
            executable="patrol_state_machine",
            name="patrol_state_machine",
            output="screen",
            parameters=[{
                "patrol_mode": "sequential",
                "track_duration": 10.0,
                "confirm_ratio": 0.6,
                "alert_cooldown": 30.0,
            }],
        ),
    ])
