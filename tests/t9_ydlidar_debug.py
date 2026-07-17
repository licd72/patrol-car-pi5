#!/usr/bin/env python3
"""调试 X4 数据流协议"""
import serial, time, struct
s = serial.Serial('/dev/rplidar', 128000, timeout=1)
s.write(bytes([0xA5, 0x65])); time.sleep(0.3); s.reset_input_buffer()
s.write(bytes([0xA5, 0x60])); time.sleep(0.5)

data = b''
t0 = time.time()
while time.time() - t0 < 2:
    data += s.read(2048)
s.write(bytes([0xA5, 0x65])); s.close()

print("总字节:", len(data))

positions = []
i = 0
while i < len(data)-1:
    if data[i] == 0x5A and data[i+1] == 0xA5:
        positions.append(i)
    i += 1
print("headers:", len(positions))
if len(positions) >= 2:
    print("平均间隔:", (positions[-1]-positions[0])/(len(positions)-1))

for j, p in enumerate(positions[:20]):
    if p + 20 > len(data):
        break
    pkt = data[p:p+40]
    ct = pkt[2]
    lsn = pkt[3]
    fsa = struct.unpack('<H', pkt[4:6])[0]
    lsa = struct.unpack('<H', pkt[6:8])[0]
    cs = struct.unpack('<H', pkt[8:10])[0]
    a_start = ((fsa >> 1) & 0x7FFF) / 64.0
    a_end = ((lsa >> 1) & 0x7FFF) / 64.0
    sync = "SYNC" if (ct & 1) else "    "
    print("pkt#%d pos=%d ct=0x%02x [%s] N=%d a=%.1f -> %.1f" %
          (j, p, ct, sync, lsn, a_start, a_end))
    if lsn > 0 and p + 10 + lsn*2 <= len(data):
        dists = [struct.unpack('<H', data[p+10+k*2:p+12+k*2])[0] for k in range(min(lsn,5))]
        dists_m = [(d >> 2) / 1000.0 for d in dists]
        print("   raw:", dists, "m:", ["%.2f" % d for d in dists_m])
