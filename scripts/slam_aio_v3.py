#!/usr/bin/env python3
"""slam_all_in_one final: X4 lidar + inline grid mapper + map bridge — ALL in one process"""
import rclpy, struct, time, math, serial, json, base64, io, threading, os
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np

MAP_FILE = "/tmp/map_data.json"
RESOLUTION = 0.05  # 5cm per cell
MAP_SIZE = 400     # 400x400 cells = 20m x 20m

class SlamAllInOne(Node):
    def __init__(self):
        super().__init__("slam_aio")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", qos)
        
        # Lidar
        self.ser = serial.Serial("/dev/rplidar", 128000, timeout=0.5)
        self.ser.dtr = False; time.sleep(0.2)
        self.ser.write(bytes([0xA5, 0x65])); time.sleep(0.2)
        self.ser.read(500)
        self.ser.write(bytes([0xA5, 0x60])); time.sleep(0.5)
        self.get_logger().info("X4 lidar @ 128000 ready")
        
        # Grid map (simple occupancy grid without SLAM optimization)
        self._grid = np.full((MAP_SIZE, MAP_SIZE), -1, dtype=np.int8)  # -1=unknown
        self._pose = (MAP_SIZE//2, MAP_SIZE//2, 0.0)  # x(cells), y(cells), theta
        
        self._scan_count = 0
        self._last_move_time = time.time()
        self._vx = 0.0; self._vy = 0.0; self._vz = 0.0  # current velocity
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        
        # Subscribe own /map in same process (only this works for DDS)
        # Note: map data is written directly in _publish_map, so _on_map is not needed
        self.create_timer(5.0, self._status)
        self.create_timer(2.0, self._publish_map)
    
    def _status(self):
        obs = np.sum(self._grid == 100)
        free = np.sum(self._grid == 0)
        self.get_logger().info("scan:{} grid:{}obs/{}free".format(self._scan_count, obs, free))
    
    def _loop(self):
        buf = b""; last_cmd = 0
        while self._running:
            try:
                t = time.time()
                if t - last_cmd > 2:
                    self.ser.write(bytes([0xA5, 0x60]))
                    last_cmd = t
                
                data = self.ser.read(2048)
                if len(data) < 100:
                    continue
                
                # Extract ranges
                ranges = []
                for i in range(0, len(data) - 1, 2):
                    val = struct.unpack("<H", data[i:i+2])[0]
                    if val > 0:
                        ranges.append(val / 400.0)  # X4 raw unit: calibrate
                
                if len(ranges) < 10:
                    continue
                
                # Publish LaserScan
                msg = LaserScan()
                msg.header.frame_id = "laser_frame"
                msg.header.stamp = self.get_clock().now().to_msg()
                n = len(ranges)
                msg.angle_increment = 2 * math.pi / n
                msg.angle_min = 0.0
                msg.angle_max = msg.angle_min + msg.angle_increment * n
                msg.range_min = 0.05
                msg.range_max = 8.0
                msg.ranges = [r if 0.05 < r < 8.0 else float("inf") for r in ranges]
                self.scan_pub.publish(msg)
                self._scan_count += 1
                
                # Read velocity from slam_web shared file
                try:
                    with open("/tmp/vel.json") as f:
                        vel = json.load(f)
                        self._vx = vel.get("vx", 0.0)
                        self._vy = vel.get("vy", 0.0)
                        self._vz = vel.get("vz", 0.0)
                except:
                    self._vx = self._vy = self._vz = 0.0
                
                # Update pose estimate from velocity
                now = time.time()
                dt = now - self._last_move_time
                self._last_move_time = now
                if dt > 0 and dt < 2.0:  # ignore large gaps
                    # Update position in cell units
                    px, py, ptheta = self._pose
                    dx = self._vx * dt / RESOLUTION
                    dy = self._vy * dt / RESOLUTION
                    dtheta = self._vz * dt
                    # Rotate dx,dy by current theta
                    c, s = math.cos(ptheta), math.sin(ptheta)
                    px += dx * c - dy * s
                    py += dx * s + dy * c
                    ptheta += dtheta
                    # Clamp to map bounds
                    px = max(0, min(MAP_SIZE-1, px))
                    py = max(0, min(MAP_SIZE-1, py))
                    self._pose = (px, py, ptheta)
                
                # Build grid map
                self._update_grid(ranges)
                
            except Exception:
                time.sleep(0.1)
    
    def _update_grid(self, ranges):
        """Ray-cast each range into occupancy grid (accumulative)"""
        px, py, ptheta = self._pose
        n = len(ranges)
        
        new_obs = set()
        new_free = set()
        
        for i, r in enumerate(ranges):
            if r <= 0.05 or r >= 8.0:
                continue
            ang = ptheta + i * 2 * math.pi / n
            
            # Endpoint (obstacle)
            ex = px + r * math.cos(ang) / RESOLUTION
            ey = py + r * math.sin(ang) / RESOLUTION
            ecx, ecy = int(ex), int(ey)
            if 0 <= ecx < MAP_SIZE and 0 <= ecy < MAP_SIZE:
                new_obs.add((ecx, ecy))
            
            # Free space (only mark if not already occupied)
            for d in np.arange(0.1, min(r, 6.0), 0.15):
                wx = px + d * math.cos(ang) / RESOLUTION
                wy = py + d * math.sin(ang) / RESOLUTION
                cx, cy = int(wx), int(wy)
                if 0 <= cx < MAP_SIZE and 0 <= cy < MAP_SIZE:
                    new_free.add((cx, cy))
        
        # Apply to grid (don't overwrite occupied with free)
        for cx, cy in new_obs:
            self._grid[cy, cx] = 100
        for cx, cy in new_free:
            if self._grid[cy, cx] <= 0:  # unknown or free → free
                self._grid[cy, cx] = 0
    
    def _publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = RESOLUTION
        msg.info.width = MAP_SIZE
        msg.info.height = MAP_SIZE
        msg.info.origin.position.x = -MAP_SIZE * RESOLUTION / 2.0
        msg.info.origin.position.y = -MAP_SIZE * RESOLUTION / 2.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self._grid.flatten().tolist()
        self.map_pub.publish(msg)
        
        # Also write to file in the format slam_web expects
        try:
            data = {
                "width": MAP_SIZE, "height": MAP_SIZE,
                "resolution": RESOLUTION,
                "origin_x": -MAP_SIZE * RESOLUTION / 2.0,
                "origin_y": -MAP_SIZE * RESOLUTION / 2.0,
                "cells": self._grid.flatten().tolist(),
                "stamp": msg.header.stamp.sec
            }
            with open(MAP_FILE, "w") as f:
                json.dump(data, f)
        except:
            pass
    
    def _on_map(self, msg):
        """Convert /map to PNG base64 for slam_web"""
        try:
            w, h = msg.info.width, msg.info.height
            if w == 0: return
            arr = np.array(msg.data, dtype=np.int8).reshape(h, w)
            pixels = []
            for v in arr.flat:
                if v == -1: pixels.append(128)
                elif v == 0: pixels.append(255)
                else: pixels.append(255 - int(min((abs(v)/100.0)*255, 255)))
            from PIL import Image as PILImage
            img = PILImage.new("L", (w, h))
            img.putdata(pixels)
            buf = io.BytesIO(); img.save(buf, format="PNG")
            b64 = base64.b64encode(buf.getvalue()).decode()
            with open(MAP_FILE, "w") as f:
                json.dump({"b64": b64, "stamp": msg.header.stamp.sec}, f)
        except:
            pass

def main():
    rclpy.init()
    rclpy.spin(SlamAllInOne())

if __name__ == "__main__":
    main()
