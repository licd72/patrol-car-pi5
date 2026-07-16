#!/usr/bin/env python3
"""重启+8项自检脚本 (在本地运行, 通过 SSH 检查 Pi)"""
import paramiko, time, sys, json

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("192.168.31.75", username="pi", password="yahboom", timeout=15)

def sh(cmd, t=30):
    _,o,_ = ssh.exec_command(cmd, timeout=t)
    return o.read().decode(errors='replace').rstrip()

def check(round_num):
    print(f"\n{'='*60}\n### 第 {round_num} 次重启\n{'='*60}", flush=True)
    print(f"[{round_num}.1] docker restart patrol_car ...", flush=True)
    sh("docker restart patrol_car", 30)

    print(f"[{round_num}.2] 等待 40 秒容器完全启动 (DDS 发现需时间)", flush=True)
    time.sleep(40)

    R = {}
    R["c"] = "Up" in sh("docker ps --filter name=patrol_car --format '{{.Status}}'")

    n = sh("docker exec patrol_car bash -c 'source /opt/ros/foxy/setup.bash && ros2 node list'")
    for k in ["cmd_vel_bridge","patrol_state_machine","patrol_yolo",
              "alert_dispatcher","patrol_voice","patrol_web","simple_camera","ydlidar_driver"]:
        R[k] = f"/{k}" in n

    ti = sh("docker exec patrol_car bash -c 'source /opt/ros/foxy/setup.bash && ros2 topic info /cmd_vel'")
    R["sub>=1"] = "Subscription count: 1" in ti or "Subscription count: 2" in ti

    R["web200"] = "200" == sh("curl -s -o /dev/null -w '%{http_code}' http://192.168.31.75:5000/api/state")

    bl = sh("docker exec patrol_car cat /tmp/cmd_vel_bridge.log 2>&1")
    R["bridge_ok"] = "桥就绪" in bl and "Traceback" not in bl

    print(f"[{round_num}.3] 端到端: pub /cmd_vel forward 2s", flush=True)
    sh("""docker exec patrol_car bash -c "source /opt/ros/foxy/setup.bash && timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10" 2>&1""", 8)
    time.sleep(1.5)
    bl2 = sh("docker exec patrol_car cat /tmp/cmd_vel_bridge.log 2>&1")
    R["超时停车"] = "超时" in bl2 or "停车" in bl2

    v = sh("""docker exec patrol_car python3 -c "
from Rosmaster_Lib import Rosmaster
import time
c=Rosmaster(com='/dev/myserial',debug=False); c.set_car_type(1); c.create_receive_threading()
time.sleep(1); print(f'V={c.get_battery_voltage()}')
" 2>&1 | tail -1""")
    R["stm32"] = "V=1" in v

    return R, v, bl2[-400:]

if __name__ == "__main__":
    rn = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    R, v, bl = check(rn)
    for k, ok in R.items():
        print(f"  {'✅' if ok else '❌'} {k}")
    print(f"\nSTM32: {v.strip()}")
    print(f"\nbridge log tail:\n{bl}")
    print(f"\n### 通过率: {sum(R.values())}/{len(R)} 项")
    json.dump({k: bool(v) for k, v in R.items()},
              open(f"/tmp/round{rn}_result.json", "w"), ensure_ascii=False)
    ssh.close()
