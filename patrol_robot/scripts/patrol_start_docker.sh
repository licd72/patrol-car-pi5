#!/bin/bash
# ==============================================================
# 巡逻小车 — Docker 一键启动
# 用法: bash patrol_start_docker.sh [build|start|stop|logs]
# ==============================================================
set -e

PROJECT_DIR="/home/pi/patrol_robot"
cd "$PROJECT_DIR"

case "${1:-start}" in
  build)
    echo "=== 构建 Docker 镜像 ==="
    docker build -t patrol-robot:foxy .
    echo "✅ 镜像构建完成"
    ;;

  start)
    echo "=== 编译 ROS2 包 ==="
    docker run --rm \
      -v $PROJECT_DIR:/home/pi/patrol_robot \
      -w /home/pi/patrol_robot/patrol_robot \
      patrol-robot:foxy \
      bash -c "
        source /opt/ros/foxy/setup.bash &&
        colcon build --symlink-install 2>&1 | tail -12
      "

    echo ""
    echo "=== 启动巡逻全系统 ==="
    docker compose up -d

    echo ""
    echo "========================================"
    echo "  巡逻系统已启动!"
    echo "========================================"
    echo "  Web 面板: http://192.168.31.75:5000"
    echo "  查看日志: bash patrol_start_docker.sh logs"
    echo "  停止系统: bash patrol_start_docker.sh stop"
    ;;

  stop)
    docker compose down
    echo "系统已停止"
    ;;

  logs)
    docker compose logs -f --tail=50
    ;;

  restart)
    $0 stop
    $0 start
    ;;

  *)
    echo "用法: $0 {build|start|stop|logs|restart}"
    exit 1
    ;;
esac
