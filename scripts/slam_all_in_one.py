#!/usr/bin/env python3
"""slam_all_in_one: 雷达驱动 + slam_toolbox 同进程运行"""
import subprocess, sys, os, time

# 先装依赖 (如果容器重启丢失)
os.system("apt-get install -y ros-foxy-slam-toolbox 2>/dev/null")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import struct, serial, threading, math

class SlamAllInOne(Node):
    def __init__(self):
        super().__init__("slam_all_in_one")
        
        # 雷达发布 (RELIABLE QoS, 和 slam_toolbox 匹配)
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos)
        
        # 打开串口
        self.ser = serial.Serial("/dev/rplidar", 230400, timeout=1)
        self.ser.dtr = False
        time.sleep(0.1)
        self.get_logger().info("雷达已打开")
        
        # 启动扫描线程
        self._running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("扫描线程已启动")
        
        # 启动 slam_toolbox (子进程)
        self._st = subprocess.Popen([
            "ros2", "run", "slam_toolbox", "async_slam_toolbox_node",
            "--ros-args", "--params-file",
            "/home/pi/patrol_robot/config/slam_params.yaml",
            "-p", "use_sim_time:=false"
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.get_logger().info(f"slam_toolbox PID={self._st.pid}")
        
        # 定时器检查状态
        self._scan_count = 0
        self.create_timer(5.0, self._status)
    
    def _status(self):
        self.get_logger().info(f"已发 {self._scan_count} 帧扫描")
    
    def _scan_loop(self):
        buf = b""
        last_cmd = time.time()
        while self._running:
            try:
                # 每2秒发送扫描命令
                if time.time() - last_cmd > 2:
                    self.ser.write(bytes([0xA5, 0x82]))
                    last_cmd = time.time()
                
                chunk = self.ser.read(1024)
                if not chunk:
                    continue
                buf += chunk
                
                # 解析帧
                while len(buf) >= 7:
                    if buf[0] != 0xAA or buf[1] != 0x55:
                        buf = buf[1:]
                        continue
                    
                    ct = buf[2]
                    lsn = buf[3]
                    fsa = struct.unpack("<H", buf[4:6])[0] / 64.0
                    lsa = struct.unpack("<H", buf[5:7])[0] / 64.0
                    
                    need = 7 + (lsn + 1) * 2
                    if len(buf) < need:
                        break
                    
                    if ct == 1:  # 完整一圈
                        pts = []
                        for i in range(lsn + 1):
                            d = struct.unpack("<H", buf[7+i*2:9+i*2])[0]
                            pts.append((d / 4000.0))
                        
                        msg = LaserScan()
                        msg.header.frame_id = "laser_frame"
                        msg.header.stamp = self.get_clock().now().to_msg()
                        
                        n = len(pts)
                        msg.angle_min = 0.0
                        msg.angle_max = 2 * math.pi
                        msg.angle_increment = 2 * math.pi / 360
                        msg.range_min = 0.05
                        msg.range_max = 8.0
                        msg.ranges = [float("inf")] * 360
                        
                        for i, d in enumerate(pts):
                            ang = (fsa + (lsa - fsa) * i / max(1, n-1)) * math.pi / 180
                            idx = int((ang + math.pi) / (2*math.pi) * 360) % 360
                            if 0.05 < d < 8.0:
                                msg.ranges[idx] = d
                        
                        self.scan_pub.publish(msg)
                        self._scan_count += 1
                    
                    buf = buf[need:]
                    
            except Exception as e:
                self.get_logger().error(f"扫描错误: {e}")
                time.sleep(0.1)

def main():
    rclpy.init()
    node = SlamAllInOne()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
