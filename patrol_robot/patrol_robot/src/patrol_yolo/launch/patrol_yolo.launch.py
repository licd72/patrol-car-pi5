#!/usr/bin/env python3
"""
patrol_yolo 启动文件

简化的单节点启动, 适用于开发调试。
完整系统启动请用 scripts/patrol_start.sh
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("patrol_yolo")

    # ── 启动参数 ──
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value=os.path.join(pkg_dir, "..", "..", "..", "..", "models", "yolov5n.onnx"),
        description="ONNX 模型路径"
    )
    confidence_arg = DeclareLaunchArgument(
        "confidence", default_value="0.5",
        description="检测置信度阈值"
    )
    detect_interval_arg = DeclareLaunchArgument(
        "detect_interval", default_value="0.5",
        description="检测间隔 (秒), RPi5 建议 0.5"
    )

    # ── YOLO 检测节点 ──
    yolo_node = Node(
        package="patrol_yolo",
        executable="yolo_detector",
        name="patrol_yolo",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "confidence": LaunchConfiguration("confidence"),
            "detect_interval": LaunchConfiguration("detect_interval"),
        }],
        remappings=[
            ("/camera/rgb/image_raw", "/camera/rgb/image_raw"),
        ],
    )

    return LaunchDescription([
        model_path_arg,
        confidence_arg,
        detect_interval_arg,
        yolo_node,
    ])
