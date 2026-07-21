#!/bin/bash
# ═══════════════════════════════════════════════════
# SLAM Web 控制面板 — 独立启动脚本
# 不修改 patrol 系统任何文件, 完全独立运行
# ═══════════════════════════════════════════════════

set -e

CONTAINER="patrol_car"
WS="/home/pi/patrol_robot/patrol_robot"

echo "=== SLAM Web 部署 ==="

# 1. 编译 slam_web 包
echo "[1/4] 编译 slam_web..."
docker exec $CONTAINER bash -c "
  source /opt/ros/foxy/setup.bash
  cd $WS
  colcon build --packages-select slam_web --merge-install
"
echo "  ✓ 编译完成"

# 2. 清除 pyc 缓存
echo "[2/4] 清理缓存..."
docker exec $CONTAINER bash -c "
  find $WS/install -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null
  find $WS/install -name '*.pyc' -delete 2>/dev/null
"
echo "  ✓ 缓存已清理"

# 3. 安装依赖 (Pillow 已装)
echo "[3/4] 检查依赖..."
docker exec $CONTAINER bash -c "
  pip3 list 2>/dev/null | grep -qi pillow || pip3 install pillow
  pip3 list 2>/dev/null | grep -qi numpy || pip3 install numpy
"
echo "  ✓ 依赖就绪"

# 4. 启动 (后台运行, 杀掉旧进程)
echo "[4/4] 启动 SLAM Web 面板..."
docker exec $CONTAINER bash -c "
  pkill -f 'slam_web' 2>/dev/null || true
  sleep 1
  source /opt/ros/foxy/setup.bash
  source $WS/install/setup.bash
  nohup ros2 run slam_web web_server > /tmp/slam_web.log 2>&1 &
  sleep 3
  echo 'PID:' \$!
"

echo ""
echo "═══════════════════════════════════════════"
echo "  ✅ SLAM Web 面板已启动"
echo "  🌐 http://192.168.31.75:5001"
echo "═══════════════════════════════════════════"
