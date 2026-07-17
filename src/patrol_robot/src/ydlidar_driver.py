#!/usr/bin/env python3
"""YDLIDAR X4 v5 - 时间触发 (每 200ms 发一次) + sensor QoS"""
import struct, time, math, threading
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

HEADER = b"\x5a\xa5"

def _pa(x):
    return ((x >> 1) & 0x7FFF) / 64.0


class YDLidarX4(Node):
    def __init__(self):
        super().__init__("ydlidar_driver")
        self.declare_parameter("port", "/dev/rplidar")
        self.declare_parameter("publish_hz", 5.0)
        port = self.get_parameter("port").get_parameter_value().string_value
        self._pub_period = 1.0 / self.get_parameter("publish_hz").get_parameter_value().double_value

        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.ser = serial.Serial(port, 128000, timeout=0.5)
        self.ser.write(bytes([0xA5, 0x65])); time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.ser.write(bytes([0xA5, 0x60])); time.sleep(0.3)
        self.get_logger().info(f"X4 打开: {port}, publish_hz={1.0/self._pub_period:.1f}")

        self._running = True
        self._n_scans = 0
        self._n_pkts = 0
        self._ranges = [float("inf")] * 360
        self._lock = threading.Lock()
        self._last_pub = time.time()

        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.create_timer(self._pub_period, self._tick_publish)
        self.create_timer(2.0, self._stat)

    def _stat(self):
        self.get_logger().info(f"stat: pkts={self._n_pkts} scans={self._n_scans}")

    def _tick_publish(self):
        with self._lock:
            ranges = list(self._ranges)
            # 保留有效点，未收到的重置为 inf 用于下一帧
            # 采用衰减: 上一帧的旧点保留 (激光转一圈需 ~200ms)
        # 计算有效点数
        n_valid = sum(1 for r in ranges if r != float("inf"))
        if n_valid < 10:
            return  # 数据太少不发
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = 2 * math.pi / 360
        msg.time_increment = 0.0
        msg.scan_time = self._pub_period
        msg.range_min = 0.05
        msg.range_max = 8.0
        msg.ranges = ranges
        self.scan_pub.publish(msg)
        self._n_scans += 1

    def _run(self):
        buf = b""
        while self._running:
            try:
                data = self.ser.read(4096)
                if not data:
                    continue
                buf += data
                if len(buf) > 65536:
                    buf = buf[-8192:]

                while True:
                    idx = buf.find(HEADER)
                    if idx < 0 or len(buf) - idx < 10:
                        if len(buf) > 4096:
                            buf = buf[-2:]
                        break
                    pkt = buf[idx:]
                    ct = pkt[2]
                    N = pkt[3]
                    total_len = 10 + N * 2
                    if N > 100 or total_len > 220:
                        buf = buf[idx+2:]
                        continue
                    if len(pkt) < total_len:
                        break

                    fsa = struct.unpack("<H", pkt[4:6])[0]
                    lsa = struct.unpack("<H", pkt[6:8])[0]
                    a_start = _pa(fsa)
                    a_end = _pa(lsa)
                    if a_end < a_start:
                        a_end += 360.0

                    with self._lock:
                        for i in range(N):
                            raw = struct.unpack("<H", pkt[10+i*2:12+i*2])[0]
                            dist_m = (raw >> 2) / 1000.0
                            if N > 1:
                                angle_deg = a_start + (a_end - a_start) * i / (N - 1)
                            else:
                                angle_deg = a_start
                            idx_a = int(angle_deg) % 360
                            if 0.05 < dist_m < 8.0:
                                self._ranges[idx_a] = dist_m

                    buf = buf[idx + total_len:]
                    self._n_pkts += 1

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
