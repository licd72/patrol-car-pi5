#!/bin/bash
# slam_start.sh — SLAM建图环境一键启动
# 杀掉冲突进程后按顺序启动所有依赖

set -e
source /opt/ros/foxy/setup.bash

echo "=== 清理冲突进程 ==="
pkill -9 -f serial_worker 2>/dev/null; true
pkill -9 -f odom_tf 2>/dev/null; true
pkill -9 -f slam_web 2>/dev/null; true
# 只保留一个camera和一个lidar
CAM_COUNT=$(ps aux | grep simple_camera | grep python3 | grep -v grep | wc -l)
if [ "$CAM_COUNT" -gt 1 ]; then
  pkill -9 -f simple_camera 2>/dev/null; true
fi
sleep 2

echo "=== 1. serial_worker (底盘驱动) ==="
cd /home/pi/patrol_robot/serial_driver
nohup python3 serial_worker.py > /tmp/serial_worker.log 2>&1 &
sleep 2
grep -q "READY" /tmp/serial_worker.log && echo "  OK" || echo "  FAIL"

echo "=== 2. 摄像头 ==="
nohup python3 /home/pi/patrol_robot/patrol_robot/src/simple_camera.py > /tmp/camera.log 2>&1 &
sleep 3
grep -q "Camera OK" /tmp/camera.log && echo "  OK" || echo "  FAIL"

echo "=== 3. odom TF ==="
nohup python3 /home/pi/patrol_robot/scripts/odom_tf.py > /tmp/odom_tf.log 2>&1 &
sleep 2
echo "  started"

echo "=== 4. slam_web (:5001) ==="
export PYTHONPATH=/home/pi/patrol_robot/patrol_robot/install/lib/python3.8/site-packages:$PYTHONPATH
nohup python3 -m slam_web.web_server > /tmp/slam_web.log 2>&1 &
sleep 4
curl -s -o /dev/null -w "  HTTP %{http_code}\n" http://localhost:5001/

echo ""
echo "=== 全部启动完成 ==="
echo "浏览器: http://192.168.31.75:5001"
