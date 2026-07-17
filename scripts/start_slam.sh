#!/bin/bash
# ==============================================================
# 巡逻小车 - 自主探索建图一键启动 (按需运行, 不随容器启动)
#
# 用法:
#   docker exec -it patrol_car bash /home/pi/patrol_robot/scripts/start_slam.sh
#
# 启动后会自动开始探索建图, 地图实时在 Web :5000 显示
# 结束后地图保存到 /home/pi/patrol_robot/maps/
# ==============================================================
export TZ='Asia/Shanghai'
source /opt/ros/foxy/setup.bash
source /home/pi/patrol_robot/patrol_robot/install/setup.bash

echo "=== 自主探索建图系统 ==="
echo "时间: $(date)"
echo ""

# 1. 静态 TF: base_footprint -> base_link -> laser_frame
echo "[1] 静态 TF..."
python3 /home/pi/patrol_robot/scripts/robot_tf.py > /tmp/robot_tf.log 2>&1 &
sleep 1

# 2. SLAM (slam_toolbox async_slam_toolbox_node)
echo "[2] slam_toolbox 建图..."
ros2 run slam_toolbox async_slam_toolbox_node     --ros-args --params-file /home/pi/patrol_robot/config/slam_params.yaml     > /tmp/slam.log 2>&1 &
sleep 3

# 3. 自主探索 (explore_runner.py)
echo "[3] 自主探索..."
python3 -u /home/pi/patrol_robot/explore_runner.py > /tmp/explore.log 2>&1 &
EXPLORE_PID=$!

echo ""
echo "=== 建图已启动 ==="
echo "  Web 面板: http://192.168.31.75:5000 (实时地图)"
echo "  SLAM 日志: docker exec patrol_car tail -f /tmp/slam.log"
echo "  探索日志: docker exec patrol_car tail -f /tmp/robot_tf.log"
echo ""
echo "按 Ctrl+C 停止并保存地图..."
echo ""

# 等待退出
wait $EXPLORE_PID 2>/dev/null

# 保存地图
echo "正在保存地图..."
MAP_DIR=/home/pi/patrol_robot/maps
mkdir -p $MAP_DIR
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
ros2 run nav2_map_server map_saver_cli -f $MAP_DIR/slam_$TIMESTAMP 2>/dev/null
echo "地图已保存: $MAP_DIR/slam_$TIMESTAMP.{pgm,yaml}"

# 清理
kill $(jobs -p) 2>/dev/null
echo "建图结束."
