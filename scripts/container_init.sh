#!/bin/bash
export TZ=Asia/Shanghai
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export FASTRTPS_SHM_DISABLE=1
export ROBOT_TYPE=x3
echo "=== 巡逻系统启动 $(date) ==="

# 释放摄像头
fuser -k /dev/video0 2>/dev/null; sleep 0.5

source /opt/ros/foxy/setup.bash
source /home/pi/patrol_robot/patrol_robot/install/setup.bash

echo "[bridge] 底盘桥..."
ros2 run patrol_bridge cmd_vel_bridge > /tmp/cmd_vel_bridge.log 2>&1 &
sleep 3

echo "[camera] 相机..."
python3 /home/pi/patrol_robot/patrol_robot/src/simple_camera.py > /tmp/camera.log 2>&1 &
sleep 2

echo "[lidar] 激光雷达..."
python3 /home/pi/patrol_robot/patrol_robot/src/ydlidar_driver.py > /tmp/lidar.log 2>&1 &
sleep 2

echo "[yolo] 目标检测..."
ros2 run patrol_yolo yolo_detector --ros-args -p model_path:=/home/pi/patrol_robot/models/yolov5n.onnx -p confidence:=0.5 -p detect_interval:=1.0 > /tmp/yolo.log 2>&1 &
sleep 2

echo "[state] 巡逻状态机..."
ros2 run patrol_manager patrol_state_machine > /tmp/state.log 2>&1 &
sleep 1

echo "[alert] 报警分发..."
ros2 run patrol_alert alert_dispatcher > /tmp/alert.log 2>&1 &
sleep 1

echo "[voice] 语音节点..."
ros2 run patrol_voice voice_node --ros-args -p serial_port:=/dev/myspeech -p voice_enabled:=true -p command_enabled:=true > /tmp/voice.log 2>&1 &
sleep 1

echo "[web] Web面板..."
ros2 run patrol_web web_server > /tmp/web.log 2>&1 &

echo "=== 全部启动完成 ==="
wait
