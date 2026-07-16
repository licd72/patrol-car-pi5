#!/bin/bash
# ==============================================================
# 巡逻小车 — 一键启动全系统 (v3 — 含 Web 面板)
# ==============================================================
set -e

echo "========================================"
echo "  巡逻小车全系统启动 v3"
echo "  $(date)"
echo "========================================"

# ── 环境 ──
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/patrol_robot/install/setup.bash 2>/dev/null

# ── 1. 底盘 + 雷达 (真车必选) ──
echo "[1/7] 底盘驱动..."
ros2 launch rosmaster_bringup base.launch.py 2>/dev/null &
sleep 3

# ── 2. SLAM + 定位 ──
echo "[2/7] SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=/home/pi/patrol_robot/config/slam_params.yaml 2>/dev/null &
sleep 3

# ── 3. 导航 ──
echo "[3/7] Navigation2..."
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=/home/pi/patrol_robot/config/nav2_params.yaml 2>/dev/null &
sleep 3

# ── 4. YOLO 检测 ──
echo "[4/7] YOLO 异常检测..."
ros2 run patrol_yolo yolo_detector \
    --ros-args \
    -p model_path:="/home/pi/patrol_robot/models/yolov5n.onnx" \
    -p detect_interval:=0.5 \
    -p confidence:=0.5 \
    &
sleep 2

# ── 5. 巡逻状态机 ──
echo "[5/7] 巡逻状态机..."
ros2 run patrol_manager patrol_state_machine \
    --ros-args -p patrol_mode:="sequential" &
sleep 1

# ── 6. 报警联动 ──
echo "[6/7] 报警联动..."
ros2 run patrol_alert alert_dispatcher &
sleep 1

# ── 7. Web 面板 ──
echo "[7/7] Web 监控面板..."
ros2 run patrol_web web_server &
sleep 2

echo ""
echo "========================================"
echo "  全系统已启动 (7 个节点)"
echo "========================================"
echo ""
echo "活跃节点:"
ros2 node list 2>/dev/null
echo ""
echo "访问 Web 面板: http://$(hostname -I | awk '{print $1}'):5000"
echo ""
echo "Ctrl+C 停止全部"

wait
