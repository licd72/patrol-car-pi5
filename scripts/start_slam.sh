#!/bin/bash
# 在 slam_nav 容器内启动 SLAM (含 robot_state_publisher)
source /opt/ros/humble/setup.bash

echo "=== SLAM 启动 (v2) ==="

# ── 1. 启动 robot_state_publisher (发布静态 TF) ──
# 最小化 URDF: base_footprint → base_link → laser_frame
ROBOT_DESC="${ROBOT_DESC:-/tmp/mini_robot.urdf}"
if [ ! -f "$ROBOT_DESC" ]; then
    cat > "$ROBOT_DESC" << 'URDFEOF'
<?xml version="1.0"?>
<robot name="yahboom_x3_mini">
  <link name="base_footprint"/>
  <link name="base_link"/>
  <link name="laser_frame"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.075" rpy="0 0 0"/>
  </joint>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="0.044 0 0.11" rpy="0 0 0"/>
  </joint>
</robot>
URDFEOF
fi

# 使用 YAML params file 避免命令行 XML 解析崩溃
cat > /tmp/robot_desc.yaml << YAMLEOF
/**:
  ros__parameters:
    robot_description: "$(cat $ROBOT_DESC | sed 's/"/\\"/g' | tr '\n' ' ')"
YAMLEOF

ros2 run robot_state_publisher robot_state_publisher     --ros-args --params-file /tmp/robot_desc.yaml &
RSP_PID=$!
echo "robot_state_publisher PID=$RSP_PID"
sleep 2

# ── 2. 启动 slam_toolbox ──
PARAMS="/home/pi/patrol_robot/config/slam_params.yaml"
if [ ! -f "$PARAMS" ]; then
    echo "⚠ 使用默认参数"
    PARAMS=""
fi

ros2 run slam_toolbox async_slam_toolbox_node     --ros-args --params-file ${PARAMS:-/dev/null}     -p use_sim_time:=false

# 退出时清理
kill $RSP_PID 2>/dev/null
