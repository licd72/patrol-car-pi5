#!/bin/bash
# 停止探索建图 + 停车 (快速版)
source /opt/ros/foxy/setup.bash
source /home/pi/patrol_robot/patrol_robot/install/setup.bash

echo "=== 停止探索建图 ==="

# 1. 杀 explorer + lifecycle_manager 先（防重启）
pkill -f 'nav2_explore.py' 2>/dev/null && echo "  已停止 explorer" || true
pkill -f 'explore_runner.py' 2>/dev/null || true
pkill -f 'lifecycle_manager' 2>/dev/null && echo "  已停止 lifecycle" || true

# 2. 杀全部 Nav2 + SLAM + TF（一次性）
pkill -f 'controller_server|planner_server|bt_navigator|recoveries_server|waypoint_follower|navigation_launch|slam_toolbox|robot_tf|start_slam' 2>/dev/null
sleep 1

# 3. 强制清理
pkill -9 -f 'slam_toolbox|robot_tf|nav2_explore|explore_runner|controller_server|planner_server|bt_navigator|recoveries_server|waypoint_follower|lifecycle_manager|navigation_launch' 2>/dev/null
sleep 1

# 4. 停车
echo "停车..."
timeout 2 ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{}' 2>/dev/null || true

echo "✅ 全部停止，车已停"
