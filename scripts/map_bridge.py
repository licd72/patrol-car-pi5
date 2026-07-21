#!/usr/bin/env python3
"""map_bridge: Humble内订阅 /map -> JSON文件 -> Foxy读 (不用PIL)"""
import rclpy, json, time, base64, sys
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

OUTPUT = "/tmp/map_data.json"

class MapBridge(Node):
    def __init__(self):
        super().__init__('map_bridge')
        self.sub = self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.get_logger().info('map_bridge ready')

    def _on_map(self, msg):
        try:
            w, h = msg.info.width, msg.info.height
            if w == 0 or h == 0:
                return
            data = {
                "width": w, "height": h,
                "resolution": msg.info.resolution,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "cells": list(msg.data),  # raw occupancy values
                "stamp": time.time()
            }
            with open(OUTPUT, 'w') as f:
                json.dump(data, f)
        except Exception as e:
            self.get_logger().warn(f"err: {e}")

def main():
    rclpy.init()
    rclpy.spin(MapBridge())

if __name__ == '__main__':
    main()
