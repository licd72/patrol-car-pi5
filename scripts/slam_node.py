#!/usr/bin/env python3
"""slam_node v2: lidar + SLAM same process, error log, one-shot timer"""
import rclpy, subprocess, struct, time, math, serial, os, json, base64, io, threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from PIL import Image as PILImage
import numpy as np

MAP_FILE = "/tmp/map_data.json"

class SlamNode(Node):
    def __init__(self):
        super().__init__("slam_node")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos)
        
        self.ser = serial.Serial("/dev/rplidar", 230400, timeout=1)
        self.ser.dtr = False; time.sleep(0.1)
        self.get_logger().info("Lidar opened")
        
        self._scan_count = 0
        self._err_count = 0
        threading.Thread(target=self._scan_loop, daemon=True).start()
        self.get_logger().info("Scan thread started")
        
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)
        self._st_started = False
        self.create_timer(2.0, self._start_st)
        self.create_timer(5.0, self._status)
    
    def _start_st(self):
        if self._st_started: return
        self._st_started = True
        self._st = subprocess.Popen(
            ["ros2", "run", "slam_toolbox", "async_slam_toolbox_node",
             "--ros-args", "--params-file",
             "/home/pi/patrol_robot/config/slam_params.yaml",
             "-p", "use_sim_time:=false"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.get_logger().info("slam_toolbox PID={}".format(self._st.pid))
    
    def _status(self):
        has_map = os.path.exists(MAP_FILE)
        self.get_logger().info("scan:{} err:{} map:{}".format(self._scan_count, self._err_count, 'YES' if has_map else 'NO'))
    
    def _scan_loop(self):
        buf = b""; last_cmd = 0
        while True:
            try:
                t = time.time()
                if t - last_cmd > 2:
                    self.ser.write(bytes([0xA5, 0x82]))
                    last_cmd = t
                chunk = self.ser.read(1024)
                if not chunk: continue
                buf += chunk
                while len(buf) >= 7:
                    if buf[0] != 0xAA or buf[1] != 0x55:
                        buf = buf[1:]; continue
                    ct, lsn = buf[2], buf[3]
                    fsa = struct.unpack("<H", buf[4:6])[0] / 64.0
                    lsa = struct.unpack("<H", buf[5:7])[0] / 64.0
                    need = 7 + (lsn + 1) * 2
                    if len(buf) < need: break
                    if ct == 1:
                        pts = []
                        for i in range(lsn + 1):
                            d = struct.unpack("<H", buf[7+i*2:9+i*2])[0]
                            pts.append(d / 4000.0)
                        msg = LaserScan()
                        msg.header.frame_id = "laser_frame"
                        msg.header.stamp = self.get_clock().now().to_msg()
                        n_pts = len(pts)
                        msg.angle_min = 0.0
                        msg.angle_max = 2 * math.pi
                        msg.angle_increment = 2*math.pi/360
                        msg.range_min = 0.05
                        msg.range_max = 8.0
                        msg.ranges = [float("inf")] * 360
                        for i, d in enumerate(pts):
                            ang = (fsa+(lsa-fsa)*i/max(1,n_pts-1))*math.pi/180
                            idx = int((ang+math.pi)/(2*math.pi)*360)%360
                            if 0.05 < d < 8.0:
                                msg.ranges[idx] = d
                        self.scan_pub.publish(msg)
                        self._scan_count += 1
                    buf = buf[need:]
            except Exception as e:
                self._err_count += 1
    
    def _on_map(self, msg):
        try:
            w, h = msg.info.width, msg.info.height
            if w == 0 or h == 0: return
            arr = np.array(msg.data, dtype=np.int8).reshape(h, w)
            img = PILImage.new("L", (w, h))
            pixels = []
            for v in arr.flat:
                if v == -1: pixels.append(128)
                elif v == 0: pixels.append(255)
                else: pixels.append(255 - int((v/100.0)*255))
            img.putdata(pixels)
            buf = io.BytesIO(); img.save(buf, format="PNG")
            b64 = base64.b64encode(buf.getvalue()).decode()
            with open(MAP_FILE, "w") as f:
                json.dump({"b64": b64, "stamp": msg.header.stamp.sec}, f)
        except:
            pass

def main():
    rclpy.init()
    rclpy.spin(SlamNode())

if __name__ == "__main__":
    main()
