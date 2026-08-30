#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR3 一键零重力拖动示教工具 (Zero-Gravity Freedrive Tool)
集成功能：
1. 自动检测并解除安全保护状态 (Unlock Protective Stop)
2. 自动检测电源与抱闸并完成上电松闸 (Power On & Brake Release)
3. 自动注入高精度重力补偿负载 (Payload Setup: 1.24kg, [12mm, 0, 72mm])
4. 一键激活零重力自由拖动与实时示教打点 (Waypoint Recording)
5. 退出时平稳锁定当前姿态并输出示教点清单
"""

import sys
import os
import time
import math
import socket
import struct
import select
import termios
import tty
import signal

ROBOT_IP_DEFAULT = os.environ.get("TRACER_UR_IP", "192.168.131.3")
DASHBOARD_PORT = 29999
SECONDARY_PORT = 30002

# 默认标定负载参数 (AG95 夹爪 0.99kg + D405 相机 0.072kg + 支架螺丝 0.18kg)
DEFAULT_PAYLOAD_MASS = 1.24          # kg
DEFAULT_PAYLOAD_COG = [0.012, 0.0, 0.072]  # [X: 12mm, Y: 0mm, Z: 72mm]


def recvall(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise EOFError("Socket connection lost")
        buf.extend(chunk)
    return bytes(buf)


def dashboard_exchange(command, host=ROBOT_IP_DEFAULT, port=DASHBOARD_PORT, timeout=3.0):
    """向 Dashboard 发送单行指令并获取返回"""
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.settimeout(timeout)
            s.recv(4096)  # greeting
            s.sendall((command + "\n").encode("ascii"))
            return s.recv(4096).decode("utf-8", errors="replace").strip()
    except Exception as e:
        return f"Error: {e}"


def get_clean_mode(response, prefix):
    if prefix in response:
        return response.split(prefix)[-1].strip()
    return response.strip()


def query_status(host=ROBOT_IP_DEFAULT):
    r_resp = dashboard_exchange("robotmode", host=host)
    s_resp = dashboard_exchange("safetymode", host=host)
    rm = get_clean_mode(r_resp, "Robotmode:")
    sm = get_clean_mode(s_resp, "Safetymode:")
    return rm, sm


def ensure_safety_normal(host=ROBOT_IP_DEFAULT):
    """检测并自动解除保护停机"""
    rm, sm = query_status(host)
    
    if sm in ["NORMAL", "REDUCED"]:
        return True
        
    if sm == "PROTECTIVE_STOP":
        print("⚠️  检测到控制器处于安全保护状态 (PROTECTIVE_STOP)，正在自动解除...")
        dashboard_exchange("close safety popup", host=host)
        time.sleep(0.2)
        resp = dashboard_exchange("unlock protective stop", host=host)
        
        if "until 5s" in resp:
            print("⏳ 控制器要求等待 5 秒安全恢复倒计时...", end="", flush=True)
            for _ in range(5):
                time.sleep(1.0)
                print(".", end="", flush=True)
            print(" 重试解锁...")
            resp = dashboard_exchange("unlock protective stop", host=host)
            
        # 轮询等待恢复为 NORMAL
        for _ in range(15):
            time.sleep(0.5)
            _, sm = query_status(host)
            if sm in ["NORMAL", "REDUCED"]:
                print("✅ 安全保护停机已成功解除！(Safetymode: NORMAL)")
                return True
                
        print(f"❌ 解除保护停机超时，当前状态: {sm}")
        return False
        
    elif "EMERGENCY_STOP" in sm:
        print("🛑 致命错误: 实体急停按钮处于被按下状态 (EMERGENCY_STOP)！")
        print("   请先物理拔起/旋转释放急停按钮后再试。")
        return False
    elif "FAULT" in sm or "VIOLATION" in sm:
        print(f"🛑 硬件或安全故障状态: {sm}，请检查示教器或控制器日志。")
        return False
    else:
        print(f"⚠️  未知安全状态: {sm}")
        return False


def ensure_power_and_brakes(host=ROBOT_IP_DEFAULT):
    """检测并自动上电和释放抱闸"""
    rm, _ = query_status(host)
    
    if rm == "RUNNING":
        return True
        
    if rm in ["POWER_OFF", "DISCONNECTED"]:
        print("⚡ 机械臂处于断电状态，正在自动上电 (Power On)...")
        dashboard_exchange("power on", host=host)
        
        # 等待进入 IDLE (上电完成)
        for _ in range(25):
            time.sleep(0.5)
            rm, sm = query_status(host)
            if rm in ["IDLE", "POWER_ON", "RUNNING"]:
                print("✅ 机械臂上电完成！")
                break
        else:
            print(f"❌ 上电超时，当前状态: {rm}")
            return False

    rm, _ = query_status(host)
    if rm in ["IDLE", "POWER_ON"]:
        print("🔓 正在释放机械臂抱闸 (Brake Release)...")
        dashboard_exchange("brake release", host=host)
        
        # 等待进入 RUNNING (抱闸松开)
        for _ in range(25):
            time.sleep(0.5)
            rm, sm = query_status(host)
            if rm == "RUNNING":
                print("✅ 抱闸已释放，机械臂处于就绪运行状态 (RUNNING)！")
                return True
        else:
            print(f"❌ 释放抱闸超时，当前状态: {rm}")
            return False
            
    return rm == "RUNNING"


def read_joint_angles(host=ROBOT_IP_DEFAULT, port=SECONDARY_PORT, timeout=2.5):
    """读取当前各关节角度"""
    joint_angles = [0.0] * 6
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            for _ in range(8):
                hdr = recvall(s, 4)
                plen = struct.unpack('!i', hdr)[0]
                pdata = recvall(s, plen - 4)
                if pdata[0] == 16:  # Robot State
                    offset = 1
                    while offset < len(pdata):
                        sub_len, sub_type = struct.unpack('!iB', pdata[offset:offset+5])
                        sub_payload = pdata[offset+5: offset+sub_len]
                        if sub_type == 1:  # Joint Data
                            j_angles = []
                            for j in range(6):
                                j_offset = j * 41
                                q_act = struct.unpack('!d', sub_payload[j_offset: j_offset+8])[0]
                                j_angles.append(q_act)
                            return j_angles
                        offset += sub_len
                    break
    except Exception:
        pass
    return joint_angles


def start_freedrive(mass=DEFAULT_PAYLOAD_MASS, cog=DEFAULT_PAYLOAD_COG, host=ROBOT_IP_DEFAULT, port=SECONDARY_PORT):
    """注入标定负载参数并启动零重力循环"""
    script = (
        "def freedrive_loop():\n"
        f"  set_payload({mass:.4f}, [{cog[0]:.4f}, {cog[1]:.4f}, {cog[2]:.4f}])\n"
        "  freedrive_mode()\n"
        "  while True:\n"
        "    sync()\n"
        "  end\n"
        "end\n"
        "freedrive_loop()\n"
    )
    s = socket.create_connection((host, port), timeout=3.0)
    s.sendall(script.encode("utf-8"))
    s.close()


def stop_freedrive(host=ROBOT_IP_DEFAULT, port=SECONDARY_PORT):
    script = (
        "def stop_loop():\n"
        "  end_freedrive_mode()\n"
        "  stopj(2.5)\n"
        "end\n"
        "stop_loop()\n"
    )
    try:
        s = socket.create_connection((host, port), timeout=3.0)
        s.sendall(script.encode("utf-8"))
        s.close()
    except Exception as e:
        print(f"停止指令发送异常: {e}")


class NonBlockingInput:
    def __init__(self):
        self.old_settings = None

    def __enter__(self):
        if sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, type, value, traceback):
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            char = sys.stdin.read(1)
            if not char:  # Pipe EOF
                return 'q'
            return char
        return None


def main():
    host = os.environ.get("TRACER_UR_IP", ROBOT_IP_DEFAULT)

    print("=" * 72)
    print("      🚀 UR3 一键零重力拖动示教工具 (Zero-Gravity Freedrive)")
    print("=" * 72)
    print(f"🔍 正在连接 UR3 控制器 ({host})...")

    # 1. 检查并解除安全保护状态 (Protective Stop)
    if not ensure_safety_normal(host):
        sys.exit(1)

    # 2. 检查并自动上电、释放抱闸 (Power on & Brake Release)
    if not ensure_power_and_brakes(host):
        sys.exit(1)

    # 3. 准备负载参数
    print(f"📦 注入重力补偿负载 -> 质量: {DEFAULT_PAYLOAD_MASS:.2f} kg, 质心: ({DEFAULT_PAYLOAD_COG[0]*1000:.0f}mm, {DEFAULT_PAYLOAD_COG[1]*1000:.0f}mm, {DEFAULT_PAYLOAD_COG[2]*1000:.0f}mm)")

    # 4. 激活 Freedrive 模式
    print("\n" + "-" * 72)
    print("🟢 正在激活【零重力拖动模式】...")
    start_freedrive(DEFAULT_PAYLOAD_MASS, DEFAULT_PAYLOAD_COG, host)
    time.sleep(0.3)

    print("=" * 72)
    print("🎉 零重力模式已开启！您现在可以用手扶住夹爪自由拖动机械臂。")
    print("=" * 72)
    print("⌨️  操作快捷键:")
    print("  👉 按 [Enter 回车] 或 [Q 键] 或 [Ctrl+C] : 立即锁定并退出零重力模式")
    print("  👉 按 [R 键] 或 [空格键 Space]           : 记录当前示教点 (打点采样)")
    print("-" * 72)

    recorded_waypoints = []
    start_time = time.time()

    def handle_exit(signum=None, frame=None):
        print("\n\n🔒 正在锁定机械臂并退出零重力模式...")
        stop_freedrive(host)
        time.sleep(0.5)
        final_q = read_joint_angles(host)
        final_deg = [round(math.degrees(a), 2) for a in final_q]
        print(f"✅ 机械臂已安全锁定！")
        print(f"📍 最终锁定位姿 (关节角度/度): {final_deg}")

        if recorded_waypoints:
            print("\n" + "=" * 72)
            print("📋 【本次拖动示教记录的点位清单】:")
            for idx, wp in enumerate(recorded_waypoints, 1):
                deg_str = ", ".join([f"{d:.2f}" for d in wp["deg"]])
                rad_str = ", ".join([f"{r:.4f}" for r in wp["rad"]])
                print(f"  [点位 {idx}] (t={wp['time']:.1f}s):")
                print(f"    角度 (deg): [{deg_str}]")
                print(f"    弧度 (rad): [{rad_str}]")
            print("=" * 72)
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    with NonBlockingInput() as nbi:
        while True:
            key = nbi.get_key()
            if key is not None:
                if key in ['\n', '\r', 'q', 'Q', '\x03']:
                    handle_exit()
                elif key in ['r', 'R', ' ']:
                    cur_q = read_joint_angles(host)
                    cur_deg = [round(math.degrees(a), 2) for a in cur_q]
                    elapsed = time.time() - start_time
                    recorded_waypoints.append({
                        "time": elapsed,
                        "deg": cur_deg,
                        "rad": cur_q
                    })
                    print(f"  📌 [已记录第 {len(recorded_waypoints)} 个示教点] (t={elapsed:.1f}s) -> 角度: {cur_deg}")

            time.sleep(0.05)


if __name__ == "__main__":
    main()
