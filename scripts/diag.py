#!/usr/bin/env python3
"""SLAM系统全面诊断"""
import subprocess, json, os, sys

def run(cmd, shell=True):
    try:
        r = subprocess.run(cmd, shell=shell, capture_output=True, text=True, timeout=10)
        return r.stdout.strip(), r.stderr.strip(), r.returncode
    except:
        return "", "timeout", -1

def docker(cmd):
    return run(f"docker exec patrol_car {cmd}")

def host(cmd):
    return run(cmd)

print("=" * 50)
print("SLAM系统诊断")
print("=" * 50)

# A. 服务端口
out, _, _ = host("curl -s -o /dev/null -w '%{http_code}' http://192.168.31.75:5000/")
print(f"A. patrol_web(5000): HTTP {out}")
out, _, _ = host("curl -s -o /dev/null -w '%{http_code}' http://192.168.31.75:5001/")
print(f"   slam_web(5001):  HTTP {out}")

# B. slam_web进程
out, _, _ = docker("ps aux | grep 'slam_web' | grep python3 | grep -v grep | wc -l")
print(f"B. slam_web进程数: {out.strip()}")

# C. 摄像头topic
out, _, _ = docker("bash -c 'source /opt/ros/foxy/setup.bash && ros2 topic info /camera/rgb/image_raw 2>&1'")
pub = "Publisher count: 1" in out
sub = "Subscription count: 1" in out or "Subscription count: 0" in out
print(f"C. camera: Publisher={pub}, HasSubscription={sub}")

# D. video_feed
out, _, _ = docker("bash -c 'timeout 3 curl -s --max-time 2 http://localhost:5001/video_feed 2>/dev/null | wc -c'")
print(f"D. video_feed: {out.strip()} bytes")

# E. SLAM容器
out, _, _ = host("docker ps --filter name=slam_nav --format '{{.Names}} {{.Status}}'")
print(f"E. slam_nav: {out.strip() or 'NOT RUNNING'}")

# F. /map
out, _, _ = docker("bash -c 'source /opt/ros/foxy/setup.bash && ros2 topic info /map 2>&1'")
print(f"F. /map: {'Publisher=1' if 'Publisher count: 1' in out else 'NO PUBLISHER'}")

# G. 地图大小
out, _, _ = docker("curl -s http://localhost:5001/api/map 2>/dev/null")
has_map = '"b64":"iVBOR' in out
print(f"G. map数据: {'YES' if has_map else 'NO'}")

# H. HTML引号
out, _, _ = docker("grep 'video_feed' /home/pi/patrol_robot/patrol_robot/install/lib/python3.8/site-packages/slam_web/templates/slam.html")
has_bs = '\\"' in out
print(f"H. HTML转义: {'BROKEN(有反斜杠)' if has_bs else 'OK'}")

# I. 控制
out, _, _ = host('curl -s -X POST http://192.168.31.75:5001/api/control -H "Content-Type: application/json" -d \'{"direction":"forward","speed":0.2,"duration":1.0}\'')
print(f"I. 控制API: {out}")

# J. SLAM日志
out, _, _ = host("docker logs slam_nav --tail 3 2>&1")
has_drop = 'queue is full' in out
print(f"J. SLAM丢帧: {'YES(有问题)' if has_drop else 'NO(正常)'}")

# K. odom_tf
out, _, _ = docker("ps aux | grep odom_tf | grep -v grep | wc -l")
print(f"K. odom_tf: {out.strip()}个")

print("=" * 50)
