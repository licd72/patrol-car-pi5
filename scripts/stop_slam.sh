#!/bin/bash
# 停止探索建图 + 停车
source /opt/ros/foxy/setup.bash
source /home/pi/patrol_robot/patrol_robot/install/setup.bash

echo "=== 停止探索建图 ==="

# 1. 先杀 explorer（停止发导航目标）
pkill -f 'nav2_explore.py' 2>/dev/null && echo "  已停止 explorer" || true
pkill -f 'explore_runner.py' 2>/dev/null && echo "  已停止 explore_runner" || true
sleep 1

# 2. 先杀 lifecycle_manager（防止它自动重启其他 Nav2 节点）
pkill -f 'lifecycle_manager' 2>/dev/null && echo "  已停止 lifecycle_manager" || true
sleep 1

# 3. 杀 Nav2 子节点
for p in controller_server planner_server bt_navigator recoveries_server waypoint_follower; do
    pkill -f "$p" 2>/dev/null && echo "  已停止 $p" || true
done
sleep 1

# 4. 杀 Nav2 launch 父进程
pkill -f 'navigation_launch' 2>/dev/null && echo "  已停止 nav2_launch" || true
pkill -f 'nav2_bringup' 2>/dev/null || true
sleep 1

# 5. 杀 SLAM + TF
pkill -f 'slam_toolbox' 2>/dev/null && echo "  已停止 slam_toolbox" || true
pkill -f 'robot_tf.py' 2>/dev/null && echo "  已停止 static_tf" || true
pkill -f 'start_slam.sh' 2>/dev/null || true
sleep 1

# 6. 强制清理残留（-9）
pkill -9 -f 'slam_toolbox|robot_tf|nav2_explore|explore_runner|controller_server|planner_server|bt_navigator|recoveries_server|waypoint_follower|lifecycle_manager|navigation_launch' 2>/dev/null || true
sleep 1

# 7. 停车（发零速）
echo "停车..."
for i in 1 2 3 4 5; do
    ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{}' 2>/dev/null
    sleep 0.1
done

# 8. 确认
REMAIN=$(pgrep -f 'slam_toolbox|robot_tf|nav2_explore|explore_runner|controller_server|planner_server|bt_navigator|recoveries_server|waypoint_follower|lifecycle_manager|navigation_launch' 2>/dev/null | wc -l)
if [ "$REMAIN" -eq 0 ]; then
    echo "✅ 全部停止，车已停"
else
    echo "⚠️ 仍有 $REMAIN 个残留进程"
    pgrep -af 'slam_toolbox|robot_tf|nav2_explore|explore_runner|controller_server|planner_server|bt_navigator|recoveries_server|waypoint_follower|lifecycle_manager|navigation_launch' 2>/dev/null
fi
