#!/home/yuan/robot_ssh/.conda/envs/global_3.10/bin/python3
# -*- coding: utf-8 -*-
"""
Phase 0.2: DH AG-160-95 夹爪功能验证脚本

两种模式:
  python3 verify_gripper.py --direct          # 直连串口 (推荐，可靠)
  python3 verify_gripper.py --ros             # ROS topic (需先 roslaunch)
  python3 verify_gripper.py --direct --manual # 手动控制
  python3 verify_gripper.py --direct --quick  # 仅快速开合
"""

import sys
import time
import argparse


# ── 参数 ──────────────────────────────────────────────
PORT = "/dev/dh_gripper_usb"
POS_OPEN = 1000
POS_CLOSE = 0
POS_HALF = 500
VEL_DEFAULT = 200
FORCE_DEFAULT = 50
FORCE_GRASP = 80


# ═══════════════════════════════════════════════════════
#  直连串口模式 (AG95NoInit)
# ═══════════════════════════════════════════════════════

def direct_send_cmd(g, position, speed=VEL_DEFAULT, force=FORCE_DEFAULT):
    """直连串口: 设置位置 + 速度 + 力"""
    g.set_vel(speed)
    g.set_force(force) if hasattr(g, 'set_force') else None
    g.set_pos(position)


def direct_read_pos(g):
    return g.read_pos()


def direct_test_basic(g):
    """基本开合"""
    print("\n" + "=" * 60)
    print("  测试 1: 基本开合 (直连串口)")
    print("=" * 60)

    print(f"\n[初始位置] {direct_read_pos(g)}")

    print("\n→ 全开 (1000)...")
    direct_send_cmd(g, POS_OPEN, speed=200)
    time.sleep(2)
    print(f"  位置: {direct_read_pos(g)}")

    print("\n→ 半开 (500)...")
    direct_send_cmd(g, POS_HALF, speed=200)
    time.sleep(1)
    print(f"  位置: {direct_read_pos(g)}")

    print("\n→ 全闭 (0)...")
    direct_send_cmd(g, POS_CLOSE, speed=100)
    time.sleep(2)
    print(f"  位置: {direct_read_pos(g)}")

    print("\n→ 全开 (1000)...")
    direct_send_cmd(g, POS_OPEN, speed=200)
    time.sleep(2)
    print(f"  位置: {direct_read_pos(g)}")

    print("\n✅ 基本开合测试完成")


def direct_test_force(g):
    """力控闭合"""
    print("\n" + "=" * 60)
    print("  测试 2: 力控闭合 (直连串口)")
    print("=" * 60)
    input("\n按 Enter 开始 (确保夹爪无障碍物) ...")

    print("\n→ 全开...")
    direct_send_cmd(g, POS_OPEN)
    time.sleep(2)

    print("→ 力控闭合 (force=80, speed=80)...")
    g.set_vel(80)
    g.set_force(80)
    g.set_pos(POS_CLOSE)
    time.sleep(3)
    pos = direct_read_pos(g)
    print(f"  停止位置: {pos}")
    if pos < 50:
        print("  (空载闭合到底)")
    else:
        print(f"  ✅ 在 {pos} 处停止 — 力控生效")

    print("\n→ 全开...")
    direct_send_cmd(g, POS_OPEN)
    time.sleep(2)
    print("\n✅ 力控闭合测试完成")


def direct_test_grasp(g):
    """抓取模拟"""
    print("\n" + "=" * 60)
    print("  测试 3: 抓取模拟 (直连串口)")
    print("  请把棋子/物体放在夹爪两指之间")
    print("=" * 60)
    input("\n按 Enter 开始 ...")

    print("\n→ 全开...")
    direct_send_cmd(g, POS_OPEN)
    time.sleep(2)

    print("→ 力控闭合 (force=80, speed=80)...")
    g.set_vel(80)
    g.set_force(80)
    g.set_pos(POS_CLOSE)
    time.sleep(3)
    pos = direct_read_pos(g)
    print(f"  停止位置: {pos}")
    if 50 < pos < 950:
        print(f"  ✅ 夹爪在 {pos} 处停止，检测到物体！")
    elif pos <= 50:
        print(f"  ⚠️ 闭合到底 ({pos})，未检测到物体或物体太薄")
    else:
        print(f"  ⚠️ 几乎未闭合 ({pos})，物体太厚？")

    print("\n→ 释放...")
    direct_send_cmd(g, POS_OPEN)
    time.sleep(2)
    print("\n✅ 抓取模拟测试完成")


def direct_manual(g):
    """手动控制"""
    print("\n" + "=" * 60)
    print("  手动控制 (直连串口)")
    print("  <pos> [speed] [force]  |  q 退出")
    print("  例: 500  |  0 80 80  |  1000")
    print("=" * 60)

    while True:
        try:
            cmd = input("\n> ").strip()
            if cmd.lower() in ('q', 'quit', 'exit'):
                break
            if not cmd:
                print(f"  当前: {direct_read_pos(g)}")
                continue
            parts = cmd.split()
            pos = int(parts[0])
            spd = int(parts[1]) if len(parts) > 1 else VEL_DEFAULT
            f = int(parts[2]) if len(parts) > 2 else FORCE_DEFAULT
            direct_send_cmd(g, pos, speed=spd, force=f)
            time.sleep(0.3)
            print(f"  → 位置: {direct_read_pos(g)}")
        except (ValueError, IndexError):
            print("  格式错误")
        except (KeyboardInterrupt, EOFError):
            break

    print("\n退出手动模式")


# ═══════════════════════════════════════════════════════
#  ROS topic 模式
# ═══════════════════════════════════════════════════════

_ros_state = None
_ros_joint = None


def _ros_state_cb(msg):
    global _ros_state
    _ros_state = msg


def _ros_joint_cb(msg):
    global _ros_joint
    _ros_joint = msg


def ros_wait_for_state(timeout=15.0):
    global _ros_state
    start = time.time()
    while _ros_state is None and time.time() - start < timeout:
        time.sleep(0.1)
    return _ros_state


def ros_print_state():
    s = _ros_state
    j = _ros_joint
    if s is None:
        print("  (无状态数据)")
        return
    print(f"  初始化: {s.is_initialized}, 状态: {s.grip_state}, "
          f"位置: {s.position:.0f}, 目标位置: {s.target_position:.0f}, "
          f"目标力: {s.target_force:.0f}")
    if j is not None and j.position:
        actual_pos = 1000 - (j.position[0] / 0.637 * 1000)
        print(f"  关节状态: {j.position[0]:.4f} rad → 约 {actual_pos:.0f} 开度")


def ros_send_cmd(position, speed=VEL_DEFAULT, force=FORCE_DEFAULT):
    import rospy
    from dh_gripper_msgs.msg import GripperCtrl
    pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=1)
    time.sleep(0.1)
    cmd = GripperCtrl()
    cmd.initialize = False
    cmd.position = float(position)
    cmd.speed = float(speed)
    cmd.force = float(force)
    pub.publish(cmd)
    print(f"  [ROS] pos={position}, speed={speed}, force={force}")


def ros_test_basic():
    print("\n" + "=" * 60)
    print("  测试 1: 基本开合 (ROS)")
    print("=" * 60)

    print("\n[初始]"); time.sleep(0.3); ros_print_state()

    print("\n→ 全开..."); ros_send_cmd(POS_OPEN, 200); time.sleep(2); ros_print_state()
    print("\n→ 半开..."); ros_send_cmd(POS_HALF, 200); time.sleep(1); ros_print_state()
    print("\n→ 全闭..."); ros_send_cmd(POS_CLOSE, 100); time.sleep(2); ros_print_state()
    print("\n→ 全开..."); ros_send_cmd(POS_OPEN, 200); time.sleep(2); ros_print_state()

    print("\n✅ ROS 基本开合完成")


def ros_test_force():
    print("\n" + "=" * 60)
    print("  测试 2: 力控闭合 (ROS)")
    print("=" * 60)
    input("\n按 Enter 开始 ...")

    ros_send_cmd(POS_OPEN); time.sleep(2)
    print("→ 力控闭合 (force=80)...")
    ros_send_cmd(POS_CLOSE, speed=80, force=FORCE_GRASP)
    time.sleep(3); ros_print_state()
    ros_send_cmd(POS_OPEN); time.sleep(2)
    print("\n✅ ROS 力控闭合完成")


def ros_manual():
    import rospy
    print("\n" + "=" * 60)
    print("  手动控制 (ROS)")
    print("  <pos> [speed] [force]  |  q 退出")
    print("=" * 60)
    while True:
        try:
            cmd = input("\n> ").strip()
            if cmd.lower() in ('q', 'quit', 'exit'):
                break
            if not cmd:
                ros_print_state()
                continue
            parts = cmd.split()
            pos = float(parts[0])
            spd = float(parts[1]) if len(parts) > 1 else VEL_DEFAULT
            f = float(parts[2]) if len(parts) > 2 else FORCE_DEFAULT
            ros_send_cmd(pos, speed=spd, force=f)
            time.sleep(0.3)
            ros_print_state()
        except (ValueError, IndexError):
            print("  格式错误")
        except (KeyboardInterrupt, EOFError):
            break
    print("\n退出")


# ═══════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="DH AG-160-95 夹爪验证")
    parser.add_argument("--direct", action="store_true",
                        help="直连串口模式 (推荐，不需 ROS)")
    parser.add_argument("--ros", action="store_true",
                        help="ROS topic 模式 (需先 roslaunch)")
    parser.add_argument("--quick", action="store_true",
                        help="仅快速开合")
    parser.add_argument("--manual", action="store_true",
                        help="手动控制模式")
    args = parser.parse_args()

    # 默认直连模式
    if not args.ros and not args.direct:
        args.direct = True

    print("=" * 60)
    print("  DH AG-160-95 夹爪功能验证")
    print(f"  模式: {'直连串口' if args.direct else 'ROS topic'}")
    print("=" * 60)

    if args.direct:
        # ── 直连串口 ──
        sys.path.insert(0, '/home/jt001/tracer_ws/src/my_ur_control/scripts')
        from gripper import AG95NoInit

        g = AG95NoInit(port=PORT)
        try:
            # 确保已初始化（断电重启后必须）
            g.initialize()
            if args.manual:
                direct_manual(g)
            elif args.quick:
                direct_test_basic(g)
            else:
                direct_test_basic(g)
                direct_test_force(g)
                direct_test_grasp(g)
        finally:
            g.ser.close()

    else:
        # ── ROS topic ──
        import rospy
        from dh_gripper_msgs.msg import GripperState
        from sensor_msgs.msg import JointState

        rospy.init_node("verify_gripper", anonymous=True, disable_signals=False)
        rospy.Subscriber("/gripper/states", GripperState, _ros_state_cb)
        rospy.Subscriber("/gripper/joint_states", JointState, _ros_joint_cb)

        print("\n等待夹爪状态数据...")
        state = ros_wait_for_state()
        if state is None:
            print("❌ 无法获取夹爪状态！请确认:")
            print("   1. roslaunch dh_gripper_driver dh_gripper.launch 已启动")
            print("   2. /dev/dh_gripper_usb 已连接")
            print("   3. USB 口正常（可先试 --direct 模式排查）")
            sys.exit(1)

        print("✅ 夹爪状态可用")
        ros_print_state()

        if args.manual:
            ros_manual()
        elif args.quick:
            ros_test_basic()
        else:
            ros_test_basic()
            ros_test_force()

    print("\n" + "=" * 60)
    print("  验证完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()
