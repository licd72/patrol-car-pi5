#!/usr/bin/env python3
"""patrol_web 启动文件"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="patrol_web",
            executable="web_server",
            name="patrol_web",
            output="screen",
            parameters=[{
                "snapshot_dir": "/home/pi/patrol_robot/snapshots/",
            }],
        ),
    ])
