#!/bin/bash
# ==============================================================
# 巡逻小车 — 容器初始化脚本 (v5 — 引入 patrol_bridge)
# 
# 变更 (2026-07-17):
#   [0.5] 新增 patrol_bridge (统一 /cmd_vel → STM32 底盘), 
#         对抗 STM32 ~100ms watchdog, 10Hz heartbeat 重发
# ==============================================================
export TZ='Asia/Shanghai'
export ROBOT_TYPE=x3
echo "=== 巡逻系统启动 $(date) (ROBOT_TYPE=$ROBOT_TYPE) ==="

source /opt/ros/foxy/setup.bash
source /home/pi/patrol_robot/patrol_robot/install/setup.bash

# ── 设备初始化 ──
echo "[0] 释放摄像头..."
fuser -k /dev/video0 2>/dev/null || true
sleep 1

# ── 启动顺序 ──
echo "[0.5] 底盘桥 patrol_bridge (/cmd_vel → STM32)..."
ros2 run patrol_bridge cmd_vel_bridge > /tmp/cmd_vel_bridge.log 2>&1 &
sleep 2

echo "[1] 相机 (V4L2+MJPG)..."
source /opt/ros/foxy/setup.bash
python3 /home/pi/patrol_robot/patrol_robot/src/simple_camera.py &
sleep 2

echo "[2] 激光雷达..."
source /opt/ros/foxy/setup.bash
python3 /home/pi/patrol_robot/patrol_robot/src/ydlidar_driver.py &
sleep 2

echo "[3] YOLO检测..."
ros2 run patrol_yolo yolo_detector --ros-args -p model_path:=/home/pi/patrol_robot/models/yolov5n.onnx -p confidence:=0.3 &
sleep 2

echo "[4] 巡逻状态机..."
ros2 run patrol_manager patrol_state_machine &
sleep 2

echo "[5] 报警调度..."
ros2 run patrol_alert alert_dispatcher &
sleep 2

echo "[6] 语音..."
ros2 run patrol_voice voice_node --ros-args -p serial_port:=/dev/myspeech -p voice_enabled:=true -p command_enabled:=true &
sleep 2

echo "[7] Web面板..."
ros2 run patrol_web web_server &

echo "=== 全部启动完成 ==="
wait
