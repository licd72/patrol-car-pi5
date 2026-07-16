#!/bin/bash
export TZ='Asia/Shanghai'
export ROBOT_TYPE=x3
export RPLIDAR_TYPE=4ROS
echo "=== 巡逻系统启动 $(date) === (ROBOT_TYPE=$ROBOT_TYPE)"

source /opt/ros/foxy/setup.bash
# 清除旧的 pyc 缓存
rm -f /root/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_bringup/lib/python3.8/site-packages/yahboomcar_bringup/__pycache__/Mcnamu_driver_X3.cpython-38.pyc 2>/dev/null
source /home/pi/patrol_robot/patrol_robot/install/setup.bash
source /home/pi/yahboomcar_ws/install/setup.bash 2>/dev/null || true  # 底盘驱动依赖

# ── 设备初始化 ──
echo "[0] 释放摄像头设备..."
fuser -k /dev/video0 2>/dev/null || true
sleep 1

# 验证摄像头 (最多重试5次)
for i in $(seq 1 5); do
    if python3 /home/pi/patrol_robot/patrol_robot/src/_cam_check.py 2>/dev/null; then
        echo "[OK] 摄像头验证通过"
        break
    fi
    echo "[RETRY $i/5] 摄像头未就绪, 等2秒..."
    sleep 2
done

# X3 麦轮驱动
echo "[1] 底盘驱动 (ROBOT_TYPE=$ROBOT_TYPE)..."
sleep 3
sleep 1
sleep 1 || true

echo "[2] 相机..."
python3 /home/pi/patrol_robot/patrol_robot/src/simple_camera.py &
sleep 2

echo "[3] 激光雷达..."
python3 /home/pi/patrol_robot/patrol_robot/src/ydlidar_driver.py &
sleep 2

echo "[4] YOLO检测..."
ros2 run patrol_yolo yolo_detector --ros-args -p model_path:=/home/pi/patrol_robot/models/yolov5n.onnx -p confidence:=0.3 &
sleep 2

echo "[5] 巡逻状态机..."
ros2 run patrol_manager patrol_state_machine &
sleep 2

echo "[6] 报警调度..."
ros2 run patrol_alert alert_dispatcher &
sleep 2

echo "[7] 语音..."
ros2 run patrol_voice voice_node --ros-args -p serial_port:=/dev/myspeech -p voice_enabled:=true -p command_enabled:=true &
sleep 2

echo "[8] Web面板..."
ros2 run patrol_web web_server &

echo "=== 全部启动完成 ==="
wait
