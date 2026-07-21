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

# # # === STM32 健康检查 ===
echo "[check] 等待 STM32..."
for i in $(seq 1 8); do
    RAW=$(python3 -u -c "import sys,time; sys.path.insert(0,'/home/pi/patrol_robot'); from Rosmaster_Lib import Rosmaster; b=Rosmaster(car_type=1,com='/dev/myserial'); b.create_receive_threading(); time.sleep(0.3); b.set_auto_report_state(True); time.sleep(0.5); v=b.get_battery_voltage(); import os; print(v); os._exit(0)" 2>/dev/null)
    BAT=$(echo "$RAW" | grep -oE '[0-9]+[.][0-9]+' | head -1)
    if [ "$BAT" != "0.0" ] && [ -n "$BAT" ]; then
        echo "[check] STM32 就绪: ${BAT}V"
        break
    fi
    echo "[check] 等待 ($i/8)..."
    sleep 3
done

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

sleep 2
nohup python3 /home/pi/patrol_robot/patrol_robot/src/joy2vel.py > /tmp/joy2vel.log 2>&1 &

# twist_mux + 手柄 (10s延迟等待其他节点就绪)
sleep 10
nohup ros2 run twist_mux twist_mux --ros-args --params-file /home/pi/patrol_robot/config/twist_mux_topics.yaml > /tmp/twist_mux.log 2>&1 &
sleep 3
nohup ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0 -p autorepeat_rate:=10.0 > /tmp/joy_node.log 2>&1 &
sleep 2
nohup python3 /home/pi/patrol_robot/patrol_robot/src/joy2vel.py > /tmp/joy2vel.log 2>&1 &
echo "=== 全部启动完成 ==="\nwait\n\n# twist_mux + 手柄 (等待其他节点就绪)\nsleep 10\nnohup ros2 run twist_mux twist_mux --ros-args --params-file /home/pi/patrol_robot/config/twist_mux_topics.yaml > /tmp/twist_mux.log 2>&1 &\nsleep 2\nnohup ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0 -p autorepeat_rate:=10.0 > /tmp/joy_node.log 2>&1 &\nsleep 2\nnohup python3 /home/pi/patrol_robot/patrol_robot/src/joy2vel.py > /tmp/joy2vel.log 2>&1 &\n
