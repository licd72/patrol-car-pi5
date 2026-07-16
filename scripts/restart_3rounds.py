#!/usr/bin/env python3
"""快速3轮重启+自检 (总耗时 ~4 分钟)"""
import paramiko, time, sys, json, io
buf = io.StringIO()
def out(*a): 
    s = " ".join(str(x) for x in a); print(s, flush=True); buf.write(s+"\n")

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("192.168.31.75", username="pi", password="yahboom", timeout=15)

def sh(cmd, t=25):
    _,o,_ = ssh.exec_command(cmd, timeout=t)
    return o.read().decode(errors='replace').rstrip()

def one_round(rn):
    out(f"\n{'='*50}\n### 第 {rn} 次重启 (t={time.strftime('%H:%M:%S')})\n{'='*50}")
    sh("docker restart patrol_car", 25)
    out(f"  已 docker restart, 等 40s ...")
    time.sleep(40)

    R = {}
    R["容器Up"] = "Up" in sh("docker ps --filter name=patrol_car --format '{{.Status}}'")

    nodes = sh("docker exec patrol_car bash -c 'source /opt/ros/foxy/setup.bash && ros2 node list'")
    for k in ["cmd_vel_bridge","patrol_state_machine","patrol_yolo",
              "alert_dispatcher","patrol_voice","patrol_web",
              "simple_camera","ydlidar_driver"]:
        R[f"节点/{k}"] = f"/{k}" in nodes

    ti = sh("docker exec patrol_car bash -c 'source /opt/ros/foxy/setup.bash && ros2 topic info /cmd_vel'")
    R["/cmd_vel订阅=1"] = "Subscription count: 1" in ti

    R["Web:5000"] = "200" == sh("curl -s -o /dev/null -w '%{http_code}' http://192.168.31.75:5000/api/state")

    bl = sh("docker exec patrol_car cat /tmp/cmd_vel_bridge.log 2>&1")
    R["bridge就绪"] = "桥就绪" in bl
    R["bridge无异常"] = "Traceback" not in bl and "SerialException" not in bl

    # 端到端: 发前进 2s, 通过桥日志确认收到
    sh("""docker exec patrol_car bash -c "source /opt/ros/foxy/setup.bash && timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10" >/dev/null 2>&1""", 8)
    time.sleep(1.5)
    bl2 = sh("docker exec patrol_car cat /tmp/cmd_vel_bridge.log 2>&1")
    R["端到端pub触发"] = "超时" in bl2 or "停车" in bl2

    v = sh("""docker exec patrol_car python3 -c "
from Rosmaster_Lib import Rosmaster
import time
c=Rosmaster(com='/dev/myserial',debug=False); c.set_car_type(1); c.create_receive_threading()
time.sleep(1); print(f'V={c.get_battery_voltage()}')" 2>&1 | tail -1""", 15)
    R["STM32存活"] = "V=1" in v

    for k, ok in R.items():
        out(f"  {'✅' if ok else '❌'} {k}")
    passed = sum(R.values())
    out(f"  通过率: {passed}/{len(R)}")
    return R, passed == len(R)

if __name__ == "__main__":
    all_ok = True
    for rn in range(1, 4):
        R, ok = one_round(rn)
        if not ok:
            all_ok = False
    out(f"\n{'='*50}\n### 最终结果: {'✅ 3/3 全部通过' if all_ok else '❌ 至少一次未全通过'}\n{'='*50}")
    open(r"C:\Users\jiaojian-home\patrol-car-pi5\docs\restart-verification.log", "w",
         encoding="utf-8").write(buf.getvalue())
    ssh.close()
