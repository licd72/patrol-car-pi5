#!/usr/bin/env python3
"""YDLIDAR X4 v4 - 基于 SYNC 包(ct&1==1) 触发发布"""
import struct, time, math, threading
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

HEADER = b"\x5a\xa5"

def _pa(x):
    return ((x >> 1) & 0x7FFF) / 64.0

class YDLidarX4(Node):
    def __init__(self):
        super().__init__("ydlidar_driver")
        self.declare_parameter("port", "/dev/rplidar")
        port = self.get_parameter("port").get_parameter_value().string_value

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.ser = serial.Serial(port, 128000, timeout=0.5)
        self.ser.write(bytes([0xA5, 0x65])); time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.ser.write(bytes([0xA5, 0x60])); time.sleep(0.3)
        self.get_logger().info(f"X4 打开: {port}")

        self._running = True
        self._n_scans = 0
        self.buf = b""
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.create_timer(2.0, self._stat)

    def _stat(self):
        self.get_logger().info(f"stat: {self._n_scans} scans")

    def _publish(self, ranges):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = 2 * math.pi / 360
        msg.time_increment = 0.0
        msg.scan_time = 0.15
        msg.range_min = 0.05
        msg.range_max = 8.0
        msg.ranges = ranges
        self.scan_pub.publish(msg)
        self._n_scans += 1

    def _run(self):
        ranges = [float("inf")] * 360
        got_first_sync = False
        while self._running:
            try:
                data = self.ser.read(4096)
                if not data:
                    continue
                self.buf += data
                if len(self.buf) > 65536:
                    self.buf = self.buf[-8192:]

                while True:
                    idx = self.buf.find(HEADER)
                    if idx < 0 or len(self.buf) - idx < 10:
                        if len(self.buf) > 4096:
                            self.buf = self.buf[-2:]
                        break
                    pkt = self.buf[idx:]
                    ct = pkt[2]
                    N = pkt[3]
                    total_len = 10 + N * 2
                    if N > 100 or total_len > 220:
                        self.buf = self.buf[idx+2:]
                        continue
                    if len(pkt) < total_len:
                        break

                    is_sync = (ct & 0x01) == 0x01
                    fsa = struct.unpack("<H", pkt[4:6])[0]
                    lsa = struct.unpack("<H", pkt[6:8])[0]
                    a_start = _pa(fsa)
                    a_end = _pa(lsa)
                    if a_end < a_start:
                        a_end += 360.0

                    # 遇到 SYNC 且已经收过一次 -> 发布上一圈
                    if is_sync and got_first_sync:
                        self._publish(list(ranges))
                        ranges = [float("inf")] * 360
                    if is_sync:
                        got_first_sync = True

                    # 累积距离到 ranges
                    if got_first_sync:
                        for i in range(N):
                            raw = struct.unpack("<H", pkt[10+i*2:12+i*2])[0]
                            dist_m = (raw >> 2) / 1000.0
                            if N > 1:
                                angle_deg = a_start + (a_end - a_start) * i / (N - 1)
                            else:
                                angle_deg = a_start
                            idx_a = int(angle_deg) % 360
                            if 0.05 < dist_m < 8.0:
                                ranges[idx_a] = dist_m

                    self.buf = self.buf[idx + total_len:]

            except Exception as e:
                self.get_logger().error(f"error: {e}")
                time.sleep(0.3)

    def destroy_node(self):
        self._running = False
        try:
            self.ser.write(bytes([0xA5, 0x65]))
            self.ser.close()
        except: pass
        super().destroy_node()

def main():
    rclpy.init()
    n = YDLidarX4()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
