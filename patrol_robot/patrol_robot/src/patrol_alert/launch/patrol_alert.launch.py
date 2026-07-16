#!/usr/bin/env python3
"""patrol_alert 启动文件"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="patrol_alert",
            executable="alert_dispatcher",
            name="alert_dispatcher",
            output="screen",
            parameters=[{
                "max_snapshots_per_alert": 5,
            }],
        ),
    ])
