#!/bin/bash
# ==============================================================
# 无人巡逻小车 — RPi5 一键部署脚本
# 用法: bash deploy_rpi5.sh [rpi_ip]
# ==============================================================
set -euo pipefail

RPI_IP="${1:-192.168.1.100}"
RPI_USER="pi"
PROJECT_DIR="/home/pi/patrol_robot"

echo "========================================"
echo "  巡逻小车 RPi5 部署"
echo "  目标: ${RPI_USER}@${RPI_IP}"
echo "========================================"

# ── 1. 检查连接 ──
echo ""
echo "[1/6] 检查 SSH 连接..."
ssh -o ConnectTimeout=5 "${RPI_USER}@${RPI_IP}" "echo 'SSH OK'" || {
    echo "错误: 无法连接到树莓派 ${RPI_IP}"
    echo "请确认: 1) 树莓派已开机  2) IP 地址正确  3) SSH 已启用"
    exit 1
}

# ── 2. 同步项目文件 ──
echo ""
echo "[2/6] 同步项目文件 → RPi5..."
rsync -avz --delete \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.git' \
    --exclude='models/*.onnx' \
    ../patrol_robot/ \
    "${RPI_USER}@${RPI_IP}:${PROJECT_DIR}/src/"

# ── 3. 安装系统依赖 ──
echo ""
echo "[3/6] 安装系统依赖..."
ssh "${RPI_USER}@${RPI_IP}" bash -s << 'REMOTE_SCRIPT'
set -e

# ROS2 Humble (如果未装)
if ! dpkg -l | grep -q ros-humble-ros-base; then
    echo "安装 ROS2 Humble..."
    sudo apt update
    sudo apt install -y ros-humble-ros-base ros-humble-cv-bridge \
        ros-humble-vision-msgs ros-humble-slam-toolbox \
        ros-humble-navigation2 ros-humble-nav2-bringup
fi

# Python 依赖
echo "安装 Python 依赖..."
sudo apt install -y python3-pip python3-opencv python3-numpy
pip3 install --break-system-packages onnxruntime

# GStreamer (视频推流)
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-rtsp

echo "依赖安装完成"
REMOTE_SCRIPT

# ── 4. 编译 ROS2 工作区 ──
echo ""
echo "[4/6] 编译 ROS2 工作区..."
ssh "${RPI_USER}@${RPI_IP}" bash -s << 'REMOTE_SCRIPT'
set -e
source /opt/ros/humble/setup.bash
cd ~/patrol_robot
colcon build --symlink-install --packages-select patrol_yolo
echo "编译完成"
REMOTE_SCRIPT

# ── 5. 下载 ONNX 模型 (如果还没有) ──
echo ""
echo "[5/6] 准备 YOLO 模型..."
ssh "${RPI_USER}@${RPI_IP}" bash -s << 'REMOTE_SCRIPT'
MODEL_DIR="$HOME/patrol_robot/models"
MODEL_FILE="$MODEL_DIR/yolov5n.onnx"
mkdir -p "$MODEL_DIR"

if [ ! -f "$MODEL_FILE" ]; then
    echo "下载 YOLOv5n ONNX 模型..."
    # 从 Ultralytics 官方下载, 或从本地拷贝
    pip3 install --break-system-packages ultralytics
    python3 -c "
from ultralytics import YOLO
model = YOLO('yolov5n.pt')
model.export(format='onnx', imgsz=640)
import shutil
shutil.move('yolov5n.onnx', '$MODEL_FILE')
print('模型导出完成')
"
else
    echo "模型已存在: $MODEL_FILE"
fi
REMOTE_SCRIPT

# ── 6. 测试节点 ──
echo ""
echo "[6/6] 测试 patrol_yolo 节点..."
ssh "${RPI_USER}@${RPI_IP}" bash -s << 'REMOTE_SCRIPT'
source /opt/ros/humble/setup.bash
source ~/patrol_robot/install/setup.bash

# 后台启动节点
timeout 5 ros2 run patrol_yolo yolo_detector \
    --ros-args -p model_path:="$HOME/patrol_robot/models/yolov5n.onnx" \
    2>&1 || true

echo ""
echo "========================================"
echo "  部署完成!"
echo "========================================"
echo ""
echo "手动启动方式:"
echo "  ssh pi@${1:-192.168.1.100}"
echo "  cd ~/patrol_robot"
echo "  source install/setup.bash"
echo "  ros2 launch patrol_yolo patrol_yolo.launch.py"
echo ""
REMOTE_SCRIPT
