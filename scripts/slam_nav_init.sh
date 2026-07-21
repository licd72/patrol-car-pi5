#!/bin/bash
# slam_nav启动 v6: 自包含, 启动时安装依赖
source /opt/ros/humble/setup.bash
echo "=== SLAM v6 ==="

# 0. 安装依赖 (容器重建丢失)
pip3 install pyserial 2>/dev/null

# 1. 雷达驱动 (patrol_car已验证版本, 修复Humble兼容)
python3 -c "
import sys; sys.path.insert(0,'/home/pi/patrol_robot')
# 打补丁: 加 _last_scan_cmd 初始化
src = open('/home/pi/patrol_robot/patrol_robot/src/ydlidar_driver.py').read()
open('/tmp/ydlidar_fixed.py','w').write(src.replace('def _run(self):','def _run(self):\n        self._last_scan_cmd = 0'))
" 
python3 /tmp/ydlidar_fixed.py &
sleep 4
echo "lidar started"

# 2. TF
cat > /tmp/robot_desc.yaml << 'YAMLEOF'
/**:
  ros__parameters:
    robot_description: "<?xml version=\"1.0\"?><robot name=\"r\"><link name=\"base_footprint\"/><link name=\"base_link\"/><link name=\"laser_frame\"/><link name=\"odom\"/><joint name=\"j1\" type=\"fixed\"><parent link=\"odom\"/><child link=\"base_footprint\"/></joint><joint name=\"j2\" type=\"fixed\"><parent link=\"base_footprint\"/><child link=\"base_link\"/><origin xyz=\"0 0 0.075\"/></joint><joint name=\"j3\" type=\"fixed\"><parent link=\"base_link\"/><child link=\"laser_frame\"/><origin xyz=\"0.044 0 0.11\"/></joint></robot>"
YAMLEOF
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file /tmp/robot_desc.yaml &
sleep 2

# 3. map_bridge
python3 /home/pi/patrol_robot/scripts/map_bridge.py &
sleep 1

# 4. 等雷达就绪再启 slam_toolbox
echo "等待 /scan..."
for i in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -q /scan; then
        echo "/scan 就绪 (${i}s)"
        break
    fi
    sleep 1
done

ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /home/pi/patrol_robot/config/slam_params.yaml -p use_sim_time:=false
