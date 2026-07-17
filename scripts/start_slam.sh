#!/bin/bash
# 巡逻小车 - 自主探索建图一键启动
# 用法: docker exec -it patrol_car bash /home/pi/patrol_robot/scripts/start_slam.sh
export TZ='Asia/Shanghai'
source /opt/ros/foxy/setup.bash
source /home/pi/patrol_robot/patrol_robot/install/setup.bash

cleanup() {
    trap '' SIGINT SIGTERM   # 禁用 trap 防止递归
    echo ""
    echo "=== 停止建图 ==="
    echo "保存地图..."
    MAP_DIR=/home/pi/patrol_robot/maps; mkdir -p $MAP_DIR
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    timeout 5 ros2 run nav2_map_server map_saver_cli -f $MAP_DIR/slam_$TIMESTAMP 2>/dev/null || true
    if [ -f "$MAP_DIR/slam_$TIMESTAMP.pgm" ]; then
        echo "  地图已保存: $MAP_DIR/slam_$TIMESTAMP.{pgm,yaml}"
    else
        echo "  地图保存失败（SLAM 可能已无数据）"
    fi
    echo "停止进程..."
    bash /home/pi/patrol_robot/scripts/stop_slam.sh 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "=== 自主探索建图系统 (Nav2) ==="
echo "时间: $(date)"

# ── 0. 清理旧进程 ──
echo "[0] 清理旧进程..."
trap '' SIGINT SIGTERM   # 清理时禁用 trap
bash /home/pi/patrol_robot/scripts/stop_slam.sh 2>/dev/null
trap cleanup SIGINT SIGTERM  # 恢复 trap
sleep 2

# ── 1. 静态 TF ──
echo "[1] 静态 TF..."
python3 /home/pi/patrol_robot/scripts/robot_tf.py > /tmp/robot_tf.log 2>&1 &
sleep 2

# ── 2. SLAM ──
echo "[2] slam_toolbox 建图..."
ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args --params-file /home/pi/patrol_robot/config/slam_params.yaml \
    > /tmp/slam.log 2>&1 &
sleep 5

# ── 3. Nav2 导航栈 ──
echo "[3] Nav2 导航栈..."
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=/home/pi/patrol_robot/config/nav2_params.yaml \
    use_sim_time:=false autostart:=true \
    > /tmp/nav2.log 2>&1 &
sleep 8

# ── 4. 前沿探索 ──
echo "[4] 前沿探索..."
python3 -u /home/pi/patrol_robot/nav2_explore.py > /tmp/explore.log 2>&1 &

echo ""
echo "=== 建图已启动 ==="
echo "  Web: http://192.168.31.75:5000"
echo "  SLAM:  docker exec patrol_car tail -f /tmp/slam.log"
echo "  Nav2:  docker exec patrol_car tail -f /tmp/nav2.log"
echo "  探索:  docker exec patrol_car tail -f /tmp/explore.log"
echo ""
echo "Ctrl+C 停止并保存地图"
echo ""

while true; do sleep 1; done
