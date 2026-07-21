#!/usr/bin/env python3
"""test baud rates for YDLIDAR X4"""
import serial, time

for baud in [128000, 230400, 512000]:
    try:
        s = serial.Serial("/dev/rplidar", baud, timeout=1)
        s.dtr = False; time.sleep(0.1)
        s.write(bytes([0xA5, 0x60])); time.sleep(0.1)
        s.read(200)
        s.write(bytes([0xA5, 0x82])); time.sleep(0.5)
        data = s.read(300)
        found = b"\xAA\x55" in data
        print(f"{baud}: AA55={'YES' if found else 'no'}, len={len(data)}, first={data[:10].hex()}")
        s.close()
    except Exception as e:
        print(f"{baud}: ERROR {e}")
