#!/bin/bash
sleep 25
docker exec -d patrol_car bash -c '
  source /opt/ros/foxy/setup.bash
  ros2 run twist_mux twist_mux --ros-args --params-file /home/pi/patrol_robot/config/twist_mux_topics.yaml > /tmp/twist_mux.log 2>&1
  sleep 3
  ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0 -p autorepeat_rate:=10.0 > /tmp/joy_node.log 2>&1
  sleep 2
  python3 /home/pi/patrol_robot/patrol_robot/src/joy2vel.py > /tmp/joy2vel.log 2>&1
'
