#!/usr/bin/env python3
"""巡逻小车统一起动 — patrol 节点 (驱动由 container_init.sh 管理)"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    yolo_node = TimerAction(period=2.0, actions=[Node(
        package="patrol_yolo", executable="yolo_detector", name="patrol_yolo",
        output="screen", parameters=[{
            "model_path": "/home/pi/patrol_robot/models/yolov5n.onnx",
            "confidence": 0.5, "detect_interval": 0.5}]),
    ])

    manager_node = TimerAction(period=4.0, actions=[Node(
        package="patrol_manager", executable="patrol_state_machine",
        name="patrol_state_machine", output="screen"),
    ])

    alert_node = TimerAction(period=6.0, actions=[Node(
        package="patrol_alert", executable="alert_dispatcher",
        name="alert_dispatcher", output="screen"),
    ])

    voice_node = TimerAction(period=8.0, actions=[Node(
        package="patrol_voice", executable="voice_node", name="patrol_voice",
        output="screen", parameters=[{"voice_enabled": False}]),
    ])

    web_node = TimerAction(period=10.0, actions=[Node(
        package="patrol_web", executable="web_server", name="patrol_web",
        output="screen"),
    ])

    return LaunchDescription([yolo_node, manager_node, alert_node, voice_node, web_node])
