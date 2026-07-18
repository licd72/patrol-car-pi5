#!/bin/bash
export PYTHONPATH="/home/pi/patrol_robot/patrol_robot/src/Speech_Lib:$PYTHONPATH"
export TZ=Asia/Shanghai
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROBOT_TYPE=x3
echo "=== 巡逻系统启动 $(date) ==="

# 确保 FIFO 存在
[ -p /tmp/cmd_fifo ] || mkfifo /tmp/cmd_fifo

fuser -k /dev/video0 2>/dev/null; sleep 0.5
source /opt/ros/foxy/setup.bash
source /home/pi/patrol_robot/patrol_robot/install/setup.bash

echo "[bridge] 底盘桥(FIFO)..."
ros2 run patrol_bridge cmd_vel_bridge > /tmp/cmd_vel_bridge.log 2>&1 &
sleep 3

echo "[camera] ..."; python3 /home/pi/patrol_robot/patrol_robot/src/simple_camera.py > /tmp/camera.log 2>&1 & sleep 2
echo "[lidar] ..."; python3 /home/pi/patrol_robot/patrol_robot/src/ydlidar_driver.py > /tmp/lidar.log 2>&1 & sleep 2
echo "[yolo] ..."; ros2 run patrol_yolo yolo_detector --ros-args -p model_path:=/home/pi/patrol_robot/models/yolov5n.onnx -p confidence:=0.5 -p detect_interval:=1.0 > /tmp/yolo.log 2>&1 & sleep 2
echo "[state] ..."; ros2 run patrol_manager patrol_state_machine > /tmp/state.log 2>&1 & sleep 1
echo "[alert] ..."; ros2 run patrol_alert alert_dispatcher > /tmp/alert.log 2>&1 & sleep 1
echo "[voice] ..."; ros2 run patrol_voice voice_node --ros-args -p serial_port:=/dev/myspeech -p voice_enabled:=true -p command_enabled:=true > /tmp/voice.log 2>&1 & sleep 1
echo "[web] ..."; ros2 run patrol_web web_server > /tmp/web.log 2>&1 &

echo "=== 全部启动完成 ==="
wait
