#!/usr/bin/env python3
import time, os, subprocess
import Adafruit_SSD1306 as SSD
from PIL import Image, ImageDraw, ImageFont

oled = SSD.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1)
oled.begin()
oled.clear()
oled.display()

W, H = oled.width, oled.height
img = Image.new("1", (W, H))
draw = ImageDraw.Draw(img)
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 9)

def refresh():
    oled.image(img)
    oled.display()

def cpu():
    with open("/proc/stat") as f: p = f.readline().split()
    t1 = sum(int(p[i]) for i in range(2,11)); i1 = int(p[5])
    time.sleep(0.1)
    with open("/proc/stat") as f: p = f.readline().split()
    t2 = sum(int(p[i]) for i in range(2,11)); i2 = int(p[5])
    return int((t2-t1-(i2-i1))*100/(t2-t1))

def ip():
    try:
        r = subprocess.check_output("ip -4 addr show wlan0 | grep inet | awk '{print $2}' | cut -d/ -f1", shell=True).decode().strip()
        return r if r else "N/A"
    except: return "N/A"

c = cpu(); cnt = 0
while True:
    cnt += 1
    if cnt >= 5: c = cpu(); cnt = 0
    draw.rectangle((0, 0, W, H), outline=0, fill=0)
    now = time.strftime("%H:%M")
    s = os.statvfs("/")
    sd = int((1-(s.f_frsize*s.f_bavail)/(s.f_frsize*s.f_blocks))*100)
    with open("/proc/meminfo") as f: m = f.readlines()[:2]
    mt = int(m[0].split()[1])/1048576; mf = int(m[1].split()[1])/1048576
    ram = int((1-mf/mt)*100)
    draw.text((0, 0), "CPU:{}% {}  SD:{}%".format(c, now, sd), font=font, fill=255)
    draw.text((0, 11), "RAM:{}% {:.1f}G".format(ram, mt), font=font, fill=255)
    draw.text((0, 22), "IP:{}".format(ip()), font=font, fill=255)
    refresh()
    time.sleep(1)
