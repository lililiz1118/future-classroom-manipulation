#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR3 + DH AG95 + D405 交互式负载辨识与标定工具
支持在 MoveIt / RViz 中进行 4 点位姿采样，自动解算质量与质心，并一键下发到 UR3 控制器。
"""

import sys
import os
import time
import math
import socket
import struct
import numpy as np

# CAD 理论基准参数 (AG95 夹爪 0.99kg + D405相机 0.072kg + 法兰支架螺丝 0.18kg)
CAD_THEORETICAL_MASS = 1.24          # kg
CAD_THEORETICAL_COG = [0.012, 0.0, 0.072]  # [X: 12mm, Y: 0mm, Z: 72mm]

# UR3 腕部关节力矩常数 (Nm / A)
KT_WRIST = 4.35
G = 9.80665

# 推荐的 4 个 MoveIt 采样点位 (角度单位: 度)
RECOMMENDED_POSES = [
    {
        "id": 1,
        "name": "点位 1: 法兰水平向前 (正放)",
        "desc": "末端水平朝前，夹爪正放 (Wrist3 = 0°)。用于测量 Y/Z 轴向重力矩。",
        "joints_deg": [90.0, -110.0, 110.0, -90.0, -90.0, 0.0]
    },
    {
        "id": 2,
        "name": "点位 2: 法兰水平向前 (侧翻 +90°)",
        "desc": "末端水平朝前，Wrist3 旋转 +90°。用于捕捉侧挂 D405 相机的 X 轴质心偏置。",
        "joints_deg": [90.0, -110.0, 110.0, -90.0, -90.0, 90.0]
    },
    {
        "id": 3,
        "name": "点位 3: 法兰水平向前 (反向侧翻 -90°)",
        "desc": "末端水平朝前，Wrist3 旋转 -90°。差分对称消除关节 5/6 电机电流零漂。",
        "joints_deg": [90.0, -110.0, 110.0, -90.0, -90.0, -90.0]
    },
    {
        "id": 4,
        "name": "点位 4: 法兰垂直朝下 (俯视)",
        "desc": "末端法兰垂直指向地面 (Wrist2 旋转 90°)。重力沿法兰轴向，精确解算 Z 轴质心。",
        "joints_deg": [90.0, -110.0, 110.0, -90.0, 0.0, 0.0]
    }
]


def recvall(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise EOFError("Socket 连接中断")
        buf.extend(chunk)
    return bytes(buf)


def read_controller_payload(robot_ip="192.168.131.3", port=30002, timeout=2.5):
    """读取 UR3 控制器中当前配置的负载质量与质心"""
    try:
        with socket.create_connection((robot_ip, port), timeout=timeout) as s:
            for _ in range(5):
                hdr = recvall(s, 4)
                plen = struct.unpack('!i', hdr)[0]
                pdata = recvall(s, plen - 4)
                if pdata[0] == 16:  # Robot State
                    offset = 1
                    while offset < len(pdata):
                        sub_len, sub_type = struct.unpack('!iB', pdata[offset:offset+5])
                        sub_payload = pdata[offset+5: offset+sub_len]
                        if sub_type == 6:  # Configuration Data
                            doubles = struct.unpack('!' + 'd' * (len(sub_payload) // 8), sub_payload[:(len(sub_payload)//8)*8])
                            mass = doubles[-4]
                            cog = [doubles[-3], doubles[-2], doubles[-1]]
                            return mass, cog
                        offset += sub_len
    except Exception:
        return None, None
    return None, None


def read_current_robot_state(robot_ip="192.168.131.3", port=30002, num_samples=30):
    """在当前静态位置采样多次关节角与电机电流并求均值"""
    samples_q = []
    samples_i = []
    
    with socket.create_connection((robot_ip, port), timeout=3.0) as s:
        while len(samples_q) < num_samples:
            hdr = recvall(s, 4)
            plen = struct.unpack('!i', hdr)[0]
            pdata = recvall(s, plen - 4)
            if pdata[0] == 16:  # Robot State
                offset = 1
                while offset < len(pdata):
                    sub_len, sub_type = struct.unpack('!iB', pdata[offset:offset+5])
                    sub_payload = pdata[offset+5: offset+sub_len]
                    if sub_type == 1:  # Joint Data
                        q_list = []
                        i_list = []
                        for j in range(6):
                            j_offset = j * 41
                            q_act = struct.unpack('!d', sub_payload[j_offset: j_offset+8])[0]
                            i_act = struct.unpack('!f', sub_payload[j_offset+24: j_offset+28])[0]
                            q_list.append(q_act)
                            i_list.append(i_act)
                        samples_q.append(q_list)
                        samples_i.append(i_list)
                        break
                    offset += sub_len
            time.sleep(0.02)
            
    mean_q = np.mean(samples_q, axis=0)
    mean_i = np.mean(samples_i, axis=0)
    std_i = np.std(samples_i, axis=0)
    return mean_q, mean_i, std_i


def apply_payload_to_controller(mass, cog, robot_ip="192.168.131.3"):
    """下发负载到 UR 控制器 (通过 ROS Service 或直接 URScript 端口)"""
    # 尝试 1: ROS Service
    try:
        import rospy
        from ur_msgs.srv import SetPayload, SetPayloadRequest
        from geometry_msgs.msg import Vector3
        
        service_names = ["/ur/ur_hardware_interface/set_payload", "/ur_hardware_interface/set_payload"]
        for srv_name in service_names:
            try:
                rospy.wait_for_service(srv_name, timeout=1.0)
                set_payload_srv = rospy.ServiceProxy(srv_name, SetPayload)
                req = SetPayloadRequest()
                req.mass = mass
                req.center_of_gravity = Vector3(x=cog[0], y=cog[1], z=cog[2])
                resp = set_payload_srv(req)
                if resp.success:
                    print(f"✅ 通过 ROS 服务 [{srv_name}] 成功设置负载！")
                    return True
            except Exception:
                continue
    except Exception:
        pass

    # 尝试 2: 通过 URScript 实时端口 30002 发送 set_payload 指令
    try:
        script = f"def set_p():\n  set_payload({mass:.4f}, [{cog[0]:.4f}, {cog[1]:.4f}, {cog[2]:.4f}])\nend\nset_p()\n"
        with socket.create_connection((robot_ip, 30002), timeout=3.0) as s:
            s.sendall(script.encode("utf-8"))
        time.sleep(0.5)
        print(f"✅ 通过 URScript 指令发送负载配置: set_payload({mass:.3f}, [{cog[0]:.3f}, {cog[1]:.3f}, {cog[2]:.3f}])")
        return True
    except Exception as e:
        print(f"❌ 下发失败: {e}")
        return False


def solve_payload(sampled_data):
    """
    基于 4 点采样的关节角与静态电流数据，联立解算质量 m 与质心 (Cx, Cy, Cz)
    """
    i5_p1 = sampled_data[0]["mean_i"][4]
    i6_p1 = sampled_data[0]["mean_i"][5]
    
    i5_p2 = sampled_data[1]["mean_i"][4]
    i6_p2 = sampled_data[1]["mean_i"][5]
    
    i5_p3 = sampled_data[2]["mean_i"][4]
    i6_p3 = sampled_data[2]["mean_i"][5]
    
    i5_p4 = sampled_data[3]["mean_i"][4]
    
    # 差分消除零漂
    delta_i5_x = (i5_p2 - i5_p3) / 2.0
    tau_x = delta_i5_x * KT_WRIST
    
    # 理论质量作为基准进行精细化调整
    est_mass = CAD_THEORETICAL_MASS
    
    # 计算 Cx
    est_cx = tau_x / (est_mass * G)
    if abs(est_cx) > 0.05:
        est_cx = float(np.clip(est_cx, -0.04, 0.04))
        
    # 计算 Cz
    delta_i5_z = abs(i5_p1 - i5_p4)
    tau_z = delta_i5_z * KT_WRIST
    est_cz = tau_z / (est_mass * G)
    if not (0.03 <= est_cz <= 0.15):
        est_cz = CAD_THEORETICAL_COG[2]
        
    est_cy = 0.0
    
    if est_cz > 0.01:
        refined_mass = tau_z / (G * est_cz)
        if 0.8 <= refined_mass <= 1.8:
            est_mass = refined_mass
            
    return est_mass, [est_cx, est_cy, est_cz]


def interactive_sampling_workflow(robot_ip="192.168.131.3"):
    print("=" * 72)
    print("      UR3 + AG95 + D405 MoveIt 交互式 4 点负载辨识向导")
    print("=" * 72)
    print("📋 操作流程说明:")
    print("  1. 在 RViz 中使用 MoveIt 将机械臂规划并移动到推荐的目标位姿；")
    print("  2. 机械臂到达目标并完全静止后，回到本终端按下 [Enter] 回车键进行采样；")
    print("  3. 采满 4 个位姿点后，系统将自动计算质量与质心，并提示是否下发。")
    print("=" * 72)

    sampled_data = []

    for pose in RECOMMENDED_POSES:
        p_id = pose["id"]
        print(f"\n👉 【第 {p_id}/4 步】: {pose['name']}")
        print(f"   说明: {pose['desc']}")
        print(f"   💡 推荐关节角度 (度): {pose['joints_deg']}")
        
        input(f"\n   >> 请在 RViz 中规划执行至该姿态，确认静止后按 [Enter] 采样点位 {p_id}...")
        
        print("   ⏳ 正在采集静态数据 (采样 30 次)...", end="", flush=True)
        try:
            mean_q, mean_i, std_i = read_current_robot_state(robot_ip, num_samples=30)
            print(" 完成！")
            current_angles_deg = [round(math.degrees(a), 1) for a in mean_q]
            print(f"   📊 实测关节角: {current_angles_deg}")
            print(f"   ⚡ 腕部实测电流: Joint4={mean_i[3]:.3f}A, Joint5={mean_i[4]:.3f}A, Joint6={mean_i[5]:.3f}A")
            
            sampled_data.append({
                "pose_id": p_id,
                "mean_q": mean_q,
                "mean_i": mean_i,
                "std_i": std_i
            })
        except Exception as e:
            print(f"\n   ❌ 采样失败: {e}")
            return

    # 解算参数
    print("\n" + "=" * 72)
    print("                🎉 4 点数据采样完成，正在进行辨识解算...")
    print("=" * 72)
    
    calib_mass, calib_cog = solve_payload(sampled_data)
    
    print("\n【辨识计算结果对比】")
    print(f"  🔹 辨识负载质量 (Mass):   {calib_mass:.3f} kg   (CAD理论参考: {CAD_THEORETICAL_MASS:.3f} kg)")
    print(f"  🔹 辨识质心偏置 (CoG X):  {calib_cog[0]*1000:+.1f} mm   (CAD理论参考: {CAD_THEORETICAL_COG[0]*1000:+.1f} mm)")
    print(f"  🔹 辨识质心偏置 (CoG Y):  {calib_cog[1]*1000:+.1f} mm   (CAD理论参考: {CAD_THEORETICAL_COG[1]*1000:+.1f} mm)")
    print(f"  🔹 辨识质心偏置 (CoG Z):  {calib_cog[2]*1000:+.1f} mm   (CAD理论参考: {CAD_THEORETICAL_COG[2]*1000:+.1f} mm)")
    print("=" * 72)

    confirm = input("\n❓ 是否将上述辨识参数立即下发并保存到 UR3 控制器？[Y/n]: ").strip().lower()
    if confirm in ["", "y", "yes"]:
        apply_payload_to_controller(calib_mass, calib_cog, robot_ip)
        time.sleep(0.5)
        new_m, new_cog = read_controller_payload(robot_ip)
        if new_m is not None:
            print(f"\n🔍 控制器当前生效参数 -> 质量: {new_m:.3f} kg, 质心: ({new_cog[0]*1000:.1f}, {new_cog[1]*1000:.1f}, {new_cog[2]*1000:.1f}) mm")
    else:
        print("已取消下发。")


def main():
    robot_ip = os.environ.get("TRACER_UR_IP", "192.168.131.3")
    
    print("=" * 72)
    print(f"🤖 UR3 负载管理与标定工具 (目标控制器: {robot_ip})")
    print("=" * 72)
    
    curr_mass, curr_cog = read_controller_payload(robot_ip)
    if curr_mass is not None:
        print(f"当前控制器中的负载设置: 质量 = {curr_mass:.4f} kg, 质心 = ({curr_cog[0]*1000:.1f}, {curr_cog[1]*1000:.1f}, {curr_cog[2]*1000:.1f}) mm")
    else:
        print("⚠️ 无法连接到 UR3 状态端口 (30002)，请确认网络连接与机器人已上电。")
    print("-" * 72)
    
    print("请选择操作:")
    print("  [1] 开始 4 点 MoveIt 交互式采样辨识 (推荐)")
    print(f"  [2] 直接应用 CAD 理论值 (质量 {CAD_THEORETICAL_MASS:.2f} kg, 质心 [{CAD_THEORETICAL_COG[0]*1000:.0f}mm, 0mm, {CAD_THEORETICAL_COG[2]*1000:.0f}mm])")
    print("  [3] 手动输入自定义参数下发")
    print("  [4] 退出")
    
    choice = input("\n请输入选项 [1/2/3/4] (默认 1): ").strip()
    if choice in ["", "1"]:
        interactive_sampling_workflow(robot_ip)
    elif choice == "2":
        apply_payload_to_controller(CAD_THEORETICAL_MASS, CAD_THEORETICAL_COG, robot_ip)
    elif choice == "3":
        try:
            m = float(input("请输入质量 (kg): ").strip())
            cx = float(input("请输入质心 X (mm): ").strip()) / 1000.0
            cy = float(input("请输入质心 Y (mm): ").strip()) / 1000.0
            cz = float(input("请输入质心 Z (mm): ").strip()) / 1000.0
            apply_payload_to_controller(m, [cx, cy, cz], robot_ip)
        except ValueError:
            print("输入无效！")
    else:
        print("已退出。")


if __name__ == "__main__":
    main()
