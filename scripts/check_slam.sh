#!/bin/bash
# 快速启动 SLAM Web 面板 (容器重启后运行)
set -e

echo "=== SLAM Web 启动 ==="

# 安装 docker CLI (容器重启后丢失)
if ! docker exec patrol_car which docker > /dev/null 2>&1; then
    echo "安装 docker CLI..."
    docker exec patrol_car bash -c "apt-get update -qq && apt-get install -y -qq docker.io 2>&1" | tail -1
fi

# 杀掉抢串口的进程 (bridge/serial_worker 会和 slam_web 的 Rosmaster 冲突)
docker exec patrol_car pkill -9 -f cmd_vel_bridge 2>/dev/null; true
docker exec patrol_car pkill -9 -f serial_worker 2>/dev/null; true
sleep 2
echo "串口已释放"

# 启动摄像头(如需要)
if ! docker exec patrol_car bash -c 'source /opt/ros/foxy/setup.bash && ros2 topic list 2>/dev/null' | grep -q camera; then
    docker exec -d patrol_car bash -c 'source /opt/ros/foxy/setup.bash; python3 /home/pi/patrol_robot/patrol_robot/src/simple_camera.py > /tmp/camera.log 2>&1'
    sleep 3
    echo "摄像头已启动"
fi

# 启动 slam_web
docker exec patrol_car pkill -9 -f slam_web 2>/dev/null; true
sleep 2
docker exec -d patrol_car bash -c "
export PYTHONPATH=/home/pi/patrol_robot/patrol_robot/install/lib/python3.8/site-packages:\$PYTHONPATH
source /opt/ros/foxy/setup.bash
python3 -m slam_web.web_server > /tmp/slam_web.log 2>&1
"
sleep 6

echo ""
echo "═══════════════════════════════════"
echo "  SLAM Web 已启动"
echo "  http://192.168.31.75:5001"
echo "═══════════════════════════════════"
