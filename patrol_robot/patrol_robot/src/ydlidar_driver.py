#!/usr/bin/env python3
"""YDLIDAR X4 激光雷达 ROS2 驱动 (简化版)
直接读取串口原始点云, 发布 /scan
"""

import struct, time, math, threading
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class YDLidarSimple(Node):
    def __init__(self):
        super().__init__("ydlidar_driver")
        self.declare_parameter("port", "/dev/rplidar")
        port = self.get_parameter("port").get_parameter_value().string_value

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.ser = serial.Serial(port, 128000, timeout=1)
        self.get_logger().info(f"YDLIDAR 打开: {port}")

        # 启动扫描
        self.ser.write(bytes([0xA5, 0x60]))
        time.sleep(0.1)

        self._running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.get_logger().info("扫描线程启动")

    def _run(self):
        buf = b""
        while self._running:
            try:
                buf += self.ser.read(1024)
                if len(buf) < 100:
                    continue

                # 找包: YDLIDAR X4 每帧 360 个采样点, 原始字节流
                # 简化: 取最近的 360 个距离值, 构造完整扫描
                # 实际协议解析较复杂, 这里用原始数据做近似

                # 解析: 每 2 字节一个距离 (mm)
                raw = buf[-720:]  # 最新 ~360 采样
                ranges = []
                for i in range(0, len(raw)-1, 2):
                    if i+1 >= len(raw):
                        break
                    dist = struct.unpack('<H', raw[i:i+2])[0] / 1000.0  # mm→m
                    if 0.05 < dist < 8.0:
                        ranges.append(dist)
                    else:
                        ranges.append(float('inf'))

                if len(ranges) < 10:
                    buf = buf[-512:]
                    continue

                # 填充到 360 个
                while len(ranges) < 360:
                    ranges.append(float('inf'))
                ranges = ranges[:360]

                msg = LaserScan()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "laser_frame"
                msg.angle_min = 0.0
                msg.angle_max = 2 * math.pi
                msg.angle_increment = 2 * math.pi / 360
                msg.range_min = 0.05
                msg.range_max = 8.0
                msg.ranges = ranges
                self.scan_pub.publish(msg)

                buf = buf[-512:]

            except Exception as e:
                self.get_logger().error(f"error: {e}", throttle_duration_sec=3)
                time.sleep(0.5)

    def destroy_node(self):
        self._running = False
        self.ser.write(bytes([0xA5, 0x65]))
        self.ser.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = YDLidarSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
