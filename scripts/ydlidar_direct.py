#!/usr/bin/env python3
"""ydlidar_direct: 直接读串口发布 /scan (Humble容器, 无需apt)"""
import rclpy, struct, time, math, serial
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

DEV = '/dev/rplidar'
BAUD = 230400
N = 360  # 点数
ANGLE_MIN = 0.0
ANGLE_MAX = 2 * math.pi
ANGLE_INC = (ANGLE_MAX - ANGLE_MIN) / N

class YDLidar(Node):
    def __init__(self):
        super().__init__('ydlidar_direct')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.ser = serial.Serial(DEV, BAUD, timeout=1)

        # 启动电机 (DTR=low)
        self.ser.dtr = False
        time.sleep(0.1)

        # 发送开始扫描命令
        self.ser.write(b'\xA5\x60')
        time.sleep(0.1)

        self.timer = self.create_timer(0.1, self._spin)
        self.get_logger().info(f'YDLidar direct on {DEV}')

    def _spin(self):
        try:
            # 读一包: header(0xAA 0x55) + CT + LSN + FSA + LSA + CS + data
            raw = self.ser.read(7)
            if len(raw) < 7 or raw[0] != 0xAA or raw[1] != 0x55:
                return
            ct = raw[2]  # CT: 1=start
            lsn = raw[3]
            fsa = struct.unpack('<H', raw[4:6])[0] / 64.0  # degrees
            lsa = struct.unpack('<H', raw[5:7])[0] / 64.0

            # 读点数据
            pts = []
            for i in range(lsn + 1):
                d = self.ser.read(2)
                if len(d) < 2: break
                dist = struct.unpack('<H', d)[0] / 4000.0  # mm -> m
                pts.append(dist)
            if ct == 1:  # 完整一圈
                msg = LaserScan()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'laser_frame'
                msg.angle_min = ANGLE_MIN
                msg.angle_max = ANGLE_MAX
                msg.angle_increment = ANGLE_INC
                msg.range_min = 0.1; msg.range_max = 8.0
                msg.ranges = [float('inf')] * N
                if lsn > 0:
                    for i, d in enumerate(pts):
                        idx = int((fsa + (lsa-fsa)*i/max(1,lsn-1) - 90) / 360 * N) % N
                        if 0.1 < d < 8.0:
                            msg.ranges[idx] = d
                msg.intensities = [0.0] * N
                self.pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f'read: {e}')

def main():
    rclpy.init()
    rclpy.spin(YDLidar())

if __name__ == '__main__':
    main()
