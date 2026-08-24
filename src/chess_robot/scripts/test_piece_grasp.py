#!/home/yuan/robot_ssh/.conda/envs/global_3.10/bin/python3
# -*- coding: utf-8 -*-
"""
Phase 0.3: 扁平棋子夹取策略测试

流程:
  1. 机械臂回 home
  2. 用户把棋子放在固定位置
  3. freedrive 拖拽到目标夹取位姿
  4. 自动闭合夹爪 → 抬升 → 判断成功/失败
  5. 重复多轮，记录所有结果

用法:
  python3 test_piece_grasp.py              # 交互式测试
  python3 test_piece_grasp.py --rounds 3   # 指定测试轮数
"""

import sys
import os
import time
import json
import threading
from datetime import datetime

# ── math3d monkey-patch ──
import math3d.vector as _mv
_orig_truediv = _mv.Vector.__truediv__

def _patched_truediv(self, other):
    if _mv.utils.is_num_type(other):
        if _mv.np.isclose(other, 0.0):
            raise ZeroDivisionError("In division of vector by scalar")
        return _mv.Vector(1.0 / other * self._data)
    else:
        raise _mv.utils.Error("__truediv__ : Could not divide by non-number")

_mv.Vector.__truediv__ = _patched_truediv
_mv.Vector.__div__ = _patched_truediv
# ─────────────────────────

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '../../my_ur_control/scripts'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '../../my_ur_control/scripts/ur_control'))

import urx
import numpy as np
import collections.abc
collections.Iterable = collections.abc.Iterable

from gripper import AG95NoInit, read_current, slow_close_until_current

# ── 参数 ─────────────────────────────────────────────────
ROBOT_IP = "192.168.131.3"
GRIPPER_PORT = "/dev/dh_gripper_usb"

# 安全位姿
HOME_J = [1.60624647, -2.69281799, 2.24398565, -2.31487161, -1.60637647, 0.00013183]

# 夹爪参数
GRIP_OPEN_START = 800     # 抓取前夹爪开度（略大于棋子直径）
LIFT_HEIGHT = 0.05        # 抓取后抬升高度 (m)
LIFT_VEL = 0.03           # 抬升速度 (m/s)
LIFT_ACC = 0.05           # 抬升加速度

# 结果保存路径
RESULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
RESULT_FILE = os.path.join(RESULT_DIR, 'grasp_test_results.json')


def fmt_arr(values, width=8):
    inner = ", ".join(f"{v:{width}.4f}" for v in values)
    return f"[{inner}]"


def check_robot_ready(robot):
    """检查机械臂状态是否可操作"""
    data = robot.secmon.get_all_data()
    rmd = data.get('RobotModeData', {})
    mode = rmd.get('robotMode', -1)
    if mode != 7:
        names = {0: "DISCONNECTED", 1: "CONFIRM_SAFETY", 2: "BOOTING",
                 3: "POWER_OFF", 4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE", 7: "RUNNING"}
        print(f"❌ 机械臂未就绪: {names.get(mode, 'UNKNOWN')} (需 RUNNING)")
        return False
    return True


def do_grasp(gripper, start_pos=GRIP_OPEN_START):
    """力控闭合夹爪，返回 (停止位置, 电流值)"""
    print(f"  夹爪从 {start_pos} 开始力控闭合...")
    gripper.set_vel(100)
    pos, cur = slow_close_until_current(gripper, start_pos=start_pos)
    print(f"  停止位置: {pos}, 电流: {cur}")
    return pos, cur


def do_lift(robot, height=LIFT_HEIGHT):
    """从当前位置垂直抬升"""
    pose = robot.getl()
    target = pose[:]
    target[2] += height
    print(f"  垂直抬升 {height*1000:.0f}mm → z={target[2]:.4f}")
    robot.movel(target, acc=LIFT_ACC, vel=LIFT_VEL)


def print_pose_info(joints, pose):
    """格式化打印位姿信息"""
    print(f"  关节角 (rad):  {fmt_arr(joints)}")
    print(f"  TCP (m+rotvec): {fmt_arr(pose)}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="扁平棋子夹取策略测试")
    parser.add_argument("--rounds", type=int, default=0,
                        help="测试轮数 (0=无限循环)")
    parser.add_argument("--grip-open", type=int, default=GRIP_OPEN_START,
                        help=f"抓取前夹爪开度 (默认 {GRIP_OPEN_START})")
    args = parser.parse_args()

    results = []
    round_num = 0

    # ═══ 连接 ═══════════════════════════════════════════
    print("=" * 60)
    print("  Phase 0.3: 扁平棋子夹取策略测试")
    print("=" * 60)

    print(f"\n连接 UR 机械臂 {ROBOT_IP} ...")
    robot = urx.Robot(ROBOT_IP)

    if not check_robot_ready(robot):
        robot.close()
        sys.exit(1)
    print("✅ 机械臂就绪")

    print(f"\n连接 DH 夹爪 {GRIPPER_PORT} ...")
    gripper = AG95NoInit(port=GRIPPER_PORT)
    gripper.initialize()
    print(f"✅ 夹爪就绪 (位置: {gripper.read_pos()})")

    # ═══ 回 home ════════════════════════════════════════
    print(f"\n→ 回 home 位姿...")
    robot.movej(HOME_J, acc=0.5, vel=0.5)
    time.sleep(1)
    print("✅ 已到达 home")

    print("\n" + "=" * 60)
    print("  测试说明:")
    print("  1. 把棋子放在桌面上固定位置")
    print("  2. 输入策略名称 (如 'A-垂直侧夹')")
    print("  3. 进入 freedrive，拖拽机械臂到夹取位姿")
    print("  4. 按 Enter 退出 freedrive")
    print("  5. 夹爪自动闭合，机械臂抬升")
    print("  6. 观察并记录结果 (成功/失败/备注)")
    print("=" * 60)

    while True:
        if args.rounds > 0 and round_num >= args.rounds:
            break

        round_num += 1
        print(f"\n{'─' * 60}")
        print(f"  第 {round_num} 轮" +
              (f" (共 {args.rounds} 轮)" if args.rounds > 0 else ""))
        print(f"{'─' * 60}")

        # ── 输入策略名 ──
        strategy = input("\n策略名称 (回车跳过本轮, 'q' 退出): ").strip()
        if strategy.lower() in ('q', 'quit', 'exit'):
            break
        if not strategy:
            print("  跳过本轮")
            continue

        # ── 回到 home, 夹爪全开 ──
        print("\n→ 回 home + 夹爪全开...")
        robot.movej(HOME_J, acc=0.5, vel=0.5)
        gripper.set_vel(200)
        gripper.set_pos(1000)
        time.sleep(1.5)

        # ── 用户放棋子 ──
        input(f"\n📌 请将棋子放在桌面固定位置，按 Enter 继续...")

        # ── Freedrive ──
        print(f"\n→ 进入 freedrive，拖拽到 [{strategy}] 夹取位姿...")
        print("  按 Enter 退出 freedrive 并执行夹取")
        robot.set_freedrive(True, timeout=99999)
        input("  拖拽中... 到位后按 Enter: ")
        robot.set_freedrive(False)
        time.sleep(0.3)

        # ── 记录位姿 ──
        joints = robot.getj()
        pose = robot.getl()
        print(f"\n  当前位姿 ({strategy}):")
        print_pose_info(joints, pose)

        # ── 闭合夹爪 ──
        print(f"\n→ 夹爪从 {args.grip_open} 开始闭合...")
        gripper.set_vel(200)
        gripper.set_pos(args.grip_open)
        time.sleep(1)
        stop_pos, stop_cur = do_grasp(gripper, start_pos=args.grip_open)

        # ── 抬升 ──
        print("\n→ 垂直抬升...")
        do_lift(robot, LIFT_HEIGHT)
        time.sleep(0.5)

        # ── 用户判断 ──
        print("\n  观察棋子是否被成功夹起")
        verdict = input("  结果 (y=成功 / n=失败 / s=跳过): ").strip().lower()

        if verdict == 's':
            print("  跳过，不记录")
            # 回到安全状态
            gripper.set_pos(1000)
            time.sleep(1)
            robot.movej(HOME_J, acc=0.5, vel=0.5)
            continue

        success = verdict == 'y'
        notes = input("  备注 (可选): ").strip()

        # ── 记录结果 ──
        result = {
            "round": round_num,
            "strategy": strategy,
            "timestamp": datetime.now().isoformat(),
            "success": success,
            "joints": [round(j, 8) for j in joints],
            "pose": [round(p, 8) for p in pose],
            "grip_open_start": args.grip_open,
            "grip_stop_position": stop_pos,
            "grip_stop_current": stop_cur,
            "lift_height": LIFT_HEIGHT,
            "notes": notes,
        }
        results.append(result)
        print(f"\n  {'✅ 成功' if success else '❌ 失败'} — 已记录 ({len(results)} 条)")

        # ── 释放 + 回 home ──
        print("\n→ 释放棋子 + 回 home...")
        gripper.set_pos(1000)
        time.sleep(0.5)
        robot.movej(HOME_J, acc=0.5, vel=0.5)
        print("✅ 本轮结束")

    # ═══ 保存结果 ═══════════════════════════════════════
    print("\n" + "=" * 60)

    if results:
        os.makedirs(RESULT_DIR, exist_ok=True)
        # 合并已有结果
        existing = []
        if os.path.exists(RESULT_FILE):
            with open(RESULT_FILE) as f:
                existing = json.load(f)
        all_results = existing + results
        with open(RESULT_FILE, 'w') as f:
            json.dump(all_results, f, indent=2, ensure_ascii=False)
        print(f"  结果已保存到: {RESULT_FILE}")
        print(f"  本次: {len(results)} 条, 累计: {len(all_results)} 条")

        # 打印汇总
        print(f"\n{'─' * 60}")
        print("  本次测试汇总:")
        print(f"{'─' * 60}")
        for r in results:
            status = "✅" if r['success'] else "❌"
            print(f"  {status} 轮{r['round']}: {r['strategy']}")
            print(f"     joints = {r['joints']}")
            print(f"     pose   = {r['pose']}")
            if r['notes']:
                print(f"     备注: {r['notes']}")
    else:
        print("  无结果记录")

    # ═══ 清理 ═══════════════════════════════════════════
    robot.movej(HOME_J, acc=0.5, vel=0.5)
    gripper.ser.close()
    robot.close()
    print("\n✅ 测试完成")


if __name__ == "__main__":
    main()
