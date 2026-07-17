#!/bin/bash
source /opt/ros/foxy/setup.bash
source /home/pi/patrol_robot/patrol_robot/install/setup.bash
echo "=== 停止探索建图 ==="
for p in explore_runner nav2_explore slam_toolbox robot_tf controller_server planner_server bt_navigator lifecycle_manager costmap; do
    pkill -f "$p" 2>/dev/null && echo "  已停止 $p" || true
done
sleep 2
for i in 1 2 3 4 5; do
    ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{}' 2>/dev/null
    sleep 0.1
done
echo "✅ 已停车"
