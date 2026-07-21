#!/usr/bin/env python3
import ydlidar, time, sys

baud = int(sys.argv[1]) if len(sys.argv) > 1 else 230400
ltype = sys.argv[2] if len(sys.argv) > 2 else "tof"

ydlidar.os_init()
laser = ydlidar.CYdLidar()

laser.setlidaropt(ydlidar.LidarPropSerialPort, "/dev/rplidar")
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, baud)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF if ltype == "tof" else ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
laser.setlidaropt(ydlidar.LidarPropMaxRange, 8.0)
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.05)

print(f"Testing: baud={baud}, type={ltype}")
ret = laser.initialize()
print(f"init: {ret}")
if ret:
    ret = laser.turnOn()
    print(f"turnOn: {ret}")
    if ret:
        scan = ydlidar.LaserScan()
        for i in range(5):
            if laser.doProcessSimple(scan):
                print(f"SCAN: {scan.points.size()} pts, freq={1.0/scan.config.scan_time:.1f}Hz")
            else:
                print("no data")
            time.sleep(0.3)
        laser.turnOff()
laser.disconnecting()
