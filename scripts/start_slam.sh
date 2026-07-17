#!/bin/bash
export TZ='Asia/Shanghai'
source /opt/ros/foxy/setup.bash
source /home/pi/patrol_robot/patrol_robot/install/setup.bash

cleanup() {
    echo ""
    echo "正在保存地图..."
    MAP_DIR=/home/pi/patrol_robot/maps; mkdir -p $MAP_DIR
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    ros2 run nav2_map_server map_saver_cli -f $MAP_DIR/slam_$TIMESTAMP 2>/dev/null
    echo "地图已保存: $MAP_DIR/slam_$TIMESTAMP.{pgm,yaml}"
    echo "清理进程..."
    for p in explore_runner nav2_explore slam_toolbox robot_tf controller_server planner_server bt_navigator lifecycle_manager costmap; do
        pkill -f "$p" 2>/dev/null || true
    done
    echo "建图结束."
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "=== 自主探索建图系统 (Nav2) ==="
echo "时间: $(date)"

# 0. 清理
echo "[0] 清理旧进程..."
for p in explore_runner nav2_explore slam_toolbox robot_tf controller_server planner_server bt_navigator lifecycle_manager costmap; do
    pkill -f "$p" 2>/dev/null && echo "  已停止 $p" || true
done
sleep 2

# 1. TF
echo "[1] 静态 TF..."
python3 /home/pi/patrol_robot/scripts/robot_tf.py > /tmp/robot_tf.log 2>&1 &
sleep 2

# 2. SLAM
echo "[2] slam_toolbox 建图..."
ros2 run slam_toolbox async_slam_toolbox_node     --ros-args --params-file /home/pi/patrol_robot/config/slam_params.yaml     > /tmp/slam.log 2>&1 &
sleep 5

# 3. Nav2
echo "[3] Nav2 导航栈..."
ros2 launch nav2_bringup navigation_launch.py     params_file:=/home/pi/patrol_robot/config/nav2_params.yaml     use_sim_time:=false autostart:=true     > /tmp/nav2.log 2>&1 &
sleep 6

# 4. 探索
echo "[4] 前沿探索 (Nav2)..."
python3 -u /home/pi/patrol_robot/nav2_explore.py > /tmp/explore.log 2>&1 &

echo ""
echo "=== 全部启动完成 ==="
echo "  Web: http://192.168.31.75:5000"
echo "  Ctrl+C 停止并保存地图"
echo ""

# 等待 Ctrl+C
while true; do sleep 1; done
