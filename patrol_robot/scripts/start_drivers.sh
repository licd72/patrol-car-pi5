#!/bin/bash
# 启动硬件驱动: 摄像头 + 激光雷达
# 在容器内执行: docker exec patrol_car bash /home/pi/patrol_robot/scripts/start_drivers.sh

source /opt/ros/foxy/setup.bash

echo "=== 启动摄像头 (/dev/video0) ==="
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p pixel_format:=mjpeg \
  -p image_width:=640 \
  -p image_height:=480 \
  -r image_raw:=/camera/rgb/image_raw \
  &

sleep 2

echo "=== 启动激光雷达 (/dev/ttyUSB0) ==="
python3 /home/pi/patrol_robot/patrol_robot/src/ydlidar_driver.py &

sleep 1

echo "=== 驱动启动完成 ==="
echo "相机 → /camera/rgb/image_raw"
echo "雷达 → /scan"

wait
