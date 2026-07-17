#!/bin/bash
# ==============================================================
# 巡逻小车 - 停止探索建图 + 停车
#
# 用法:
#   docker exec patrol_car bash /home/pi/patrol_robot/scripts/stop_slam.sh
# ==============================================================
source /opt/ros/foxy/setup.bash
source /home/pi/patrol_robot/patrol_robot/install/setup.bash

echo "=== 停止探索建图 ==="

# 1. 停止探索和建图进程
pkill -f 'explore_runner.py' 2>/dev/null && echo "  已停止 explorer" || true
pkill -f 'async_slam_toolbox_node' 2>/dev/null && echo "  已停止 slam_toolbox" || true
pkill -f 'robot_tf.py' 2>/dev/null && echo "  已停止 static_tf" || true
pkill -f 'start_slam.sh' 2>/dev/null || true
sleep 1

# 2. 发零速停车 (确保车轮停转)
echo "停车..."
for i in 1 2 3 4 5; do
    ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{}' 2>/dev/null
    sleep 0.1
done
echo "  已发送零速命令"

# 3. 确认进程已清理
REMAIN=$(ps aux | grep -E 'explore_runner|slam_toolbox|robot_tf' | grep -v grep | wc -l)
if [ "$REMAIN" -eq 0 ]; then
    echo "✅ 所有建图进程已停止"
else
    echo "⚠️ 仍有 $REMAIN 个残留进程，强制清理..."
    pkill -9 -f 'explore_runner.py' 2>/dev/null || true
    pkill -9 -f 'slam_toolbox' 2>/dev/null || true
    pkill -9 -f 'robot_tf.py' 2>/dev/null || true
    echo "  已强制清理"
fi

echo "完成. 车已停."
