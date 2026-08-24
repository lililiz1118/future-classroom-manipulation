#!/home/yuan/robot_ssh/.conda/envs/global_3.10/bin/python3
# -*- coding: utf-8 -*-
"""
UR 机械臂 freedrive 示教模式 — 手动拖动机器人
终端布局:
  上方: 滚动历史 — 仅关节角 (保留历史)
  下方: 固定状态栏 — 关节角 / 末端位置 / 欧拉角(deg) / 四元数 (4行原地刷新)
"""

import sys
import os
import time
import shutil
import threading

# ── 修复 math3d 库的调试打印 "Other: 0.9999..." ──
import math3d.vector as _mv
_mv_orig_truediv = _mv.Vector.__truediv__

def _patched_truediv(self, other):
    if _mv.utils.is_num_type(other):
        if _mv.np.isclose(other, 0.0):
            raise ZeroDivisionError("In division of vector by scalar")
        return _mv.Vector(1.0 / other * self._data)
    else:
        raise _mv.utils.Error("__truediv__ : Could not divide by non-number")

_mv.Vector.__truediv__ = _patched_truediv
_mv.Vector.__div__ = _patched_truediv
# ────────────────────────────────────────────────────────

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ur_control'))

import urx
import numpy as np
from scipy.spatial.transform import Rotation
import collections.abc
collections.Iterable = collections.abc.Iterable

ROBOT_IP = "192.168.131.3"

ROBOT_MODE_NAMES = {
    0: "ROBOT_MODE_DISCONNECTED",
    1: "ROBOT_MODE_CONFIRM_SAFETY",
    2: "ROBOT_MODE_BOOTING",
    3: "ROBOT_MODE_POWER_OFF",
    4: "ROBOT_MODE_POWER_ON",
    5: "ROBOT_MODE_IDLE",
    6: "ROBOT_MODE_BACKDRIVE",
    7: "ROBOT_MODE_RUNNING",
}

CONTROL_MODE_NAMES = {
    0: "MODE_STOPPED",
    1: "MODE_FREEDRIVE",
    2: "MODE_TEACH",
    3: "MODE_AUTOMATIC",
}


def _fmt_arr(values, width=8):
    """格式化数组: [ 1.6062, -2.6928, ...]"""
    inner = ", ".join(f"{v:{width}.4f}" for v in values)
    return f"[{inner}]"


def _rotvec_to_euler_deg(rx, ry, rz):
    """旋转矢量 → 欧拉角 (deg, xyz 外旋)"""
    r = Rotation.from_rotvec([rx, ry, rz])
    roll, pitch, yaw = r.as_euler('xyz', degrees=True)
    return [roll, pitch, yaw]


def _rotvec_to_quat(rx, ry, rz):
    """旋转矢量 → 四元数 [qx, qy, qz, qw]"""
    r = Rotation.from_rotvec([rx, ry, rz])
    q = r.as_quat()  # scipy returns [x, y, z, w]
    return [q[0], q[1], q[2], q[3]]


class FreedriveDisplay:
    """
    双区域终端显示:
    - 上方滚动区域: 仅关节角 (1行历史)
    - 下方固定区域: 关节角 + 末端位置 + 欧拉角(deg) + 四元数 (4行原地刷新)
    """

    STATUS_LINES = 5  # 分隔线 + 4 行数据

    def __init__(self, robot):
        self._robot = robot
        self._running = False
        self._thread = None
        self._tty = sys.stdout.isatty()
        self._interval = 0.2
        self._term_height = 24

    def start(self):
        self._running = True
        self._setup()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self._teardown()

    # ── ANSI 终端设置 / 恢复 ──────────────────────────

    def _setup(self):
        if not self._tty:
            return
        self._term_height = shutil.get_terminal_size().lines
        for _ in range(self._term_height - 1):
            sys.stdout.write("\n")
        scroll_bottom = self._term_height - self.STATUS_LINES
        if scroll_bottom < 1:
            scroll_bottom = 1
        sys.stdout.write(f"\033[0;{scroll_bottom}r")
        sys.stdout.write(f"\033[{scroll_bottom};1H")
        sys.stdout.flush()

    def _teardown(self):
        if not self._tty:
            return
        sys.stdout.write("\033[r")
        sys.stdout.write(f"\033[{self._term_height};1H\n")
        sys.stdout.flush()

    # ── 刷新循环 ──────────────────────────────────────

    def _loop(self):
        count = 0
        while self._running:
            try:
                joints = self._robot.getj()
                pose = self._robot.getl()
            except Exception:
                time.sleep(self._interval)
                continue

            j_str = _fmt_arr(joints)
            p_str = _fmt_arr(pose[:3], width=8)

            # 旋转矢量 → 欧拉角 + 四元数
            rx, ry, rz = pose[3], pose[4], pose[5]
            euler = _rotvec_to_euler_deg(rx, ry, rz)
            e_str = _fmt_arr(euler, width=7)
            quat = _rotvec_to_quat(rx, ry, rz)
            q_str = _fmt_arr(quat, width=9)

            if self._tty:
                self._update_tty(j_str, p_str, e_str, q_str)
            else:
                if count % 30 == 0:
                    ts = time.strftime("%H:%M:%S")
                    sys.stdout.write(
                        f"[{ts}] 关节: {j_str}  位置: {p_str}  "
                        f"RPY: {e_str}  Quat: {q_str}\n"
                    )
                    sys.stdout.flush()

            count += 1
            time.sleep(self._interval)

    def _update_tty(self, j_str, p_str, e_str, q_str):
        """ANSI 双区域更新: 滚动区仅关节角 + 固定区4行状态"""
        try:
            self._term_height = shutil.get_terminal_size().lines
        except Exception:
            pass
        scroll_bottom = max(1, self._term_height - self.STATUS_LINES)

        ts = time.strftime("%H:%M:%S")

        # 1. 滚动区域: 仅输出 1 行关节角历史
        sys.stdout.write(f"\033[{scroll_bottom};1H")
        sys.stdout.write(f"[{ts}] 关节 (rad):     {j_str}\n")

        # 2. 固定状态栏: 刷新 4 行完整状态
        sys.stdout.write("\033[s")
        status_top = scroll_bottom + 1
        sys.stdout.write(f"\033[{status_top};1H")
        sys.stdout.write("\033[K" + "─" * 70)
        sys.stdout.write(f"\n\033[K  关节 (rad):       {j_str}")
        sys.stdout.write(f"\n\033[K  末端 (m):         {p_str}")
        sys.stdout.write(f"\n\033[K  末端 RPY (deg):   {e_str}")
        sys.stdout.write(f"\n\033[K  末端 Quat:        {q_str}")
        sys.stdout.write("\033[u")
        sys.stdout.flush()


if __name__ == '__main__':
    # ── 连接 ──────────────────────────────────────────────
    print(f"连接 UR 机械臂 {ROBOT_IP} ...", flush=True)
    robot = urx.Robot(ROBOT_IP)

    # ── 状态诊断 ──────────────────────────────────────────
    data = robot.secmon.get_all_data()
    rmd = data.get('RobotModeData', {})

    robot_mode = rmd.get('robotMode', -1)
    control_mode = rmd.get('controlMode', -1)

    print(f"\n{'='*50}")
    print(f"  机器人状态诊断:")
    print(f"    robotMode:    {robot_mode} ({ROBOT_MODE_NAMES.get(robot_mode, 'UNKNOWN')})")
    print(f"    controlMode:  {control_mode} ({CONTROL_MODE_NAMES.get(control_mode, 'UNKNOWN')})")
    print(f"    isPowerOn:    {rmd.get('isPowerOnRobot')}")
    print(f"    isEnabled:    {rmd.get('isRealRobotEnabled')}")
    print(f"    isConnected:  {rmd.get('isRobotConnected')}")
    print(f"    isEStop:      {rmd.get('isEmergencyStopped')}")
    print(f"    isSecStop:    {rmd.get('isSecurityStopped')}")
    print(f"    isProgRunning:{rmd.get('isProgramRunning')}")
    print(f"{'='*50}\n", flush=True)

    # ── 问题诊断 ──────────────────────────────────────────
    issues = []
    if robot_mode != 7:
        issues.append(
            f"❌ 机器人不在 RUNNING 模式 "
            f"(当前={ROBOT_MODE_NAMES.get(robot_mode, 'UNKNOWN')})，无法执行 freedrive"
        )
        issues.append("   → 请先: power_on → brake_release → load_program ext_ctl.urp → play")
    if rmd.get('isEmergencyStopped'):
        issues.append("❌ 急停被按下！请释放急停按钮")
    if rmd.get('isSecurityStopped'):
        issues.append("❌ 安全停止中！请检查安全配置")
    if not rmd.get('isPowerOnRobot'):
        issues.append("❌ 机器人未上电！请在示教器上按 ON 开启关节电源")
    if not rmd.get('isRealRobotEnabled'):
        issues.append("❌ 机器人未使能！")

    if issues:
        print("⚠️  无法进入 freedrive，请先解决以下问题：\n")
        for i in issues:
            print(i)
        print(f"\n💡 也可以直接在示教器上按 Freedrive 按钮（手型图标）手动拖拽")
        robot.close()
        sys.exit(1)

    print("✅ 机器人状态正常\n")

    # ── 进入 freedrive ────────────────────────────────────
    print("=" * 60)
    print("  进入 freedrive 模式（无超时，可无限拖拽）")
    print("  上方: 滚动历史 (仅关节角)")
    print("  下方: 固定状态栏 (关节/位置/欧拉角/四元数)")
    print("  按 Enter 或 Ctrl+C 退出")
    print("=" * 60)
    input("  按 Enter 开始 freedrive ...")

    robot.set_freedrive(True, timeout=99999)
    print("freedrive 已激活 → 拖拽机械臂到你想要的位姿\n", flush=True)

    # ── 启动双区域显示 ────────────────────────────────────
    display = FreedriveDisplay(robot)
    display.start()

    # ── 等待用户退出 ──────────────────────────────────────
    try:
        input()
    except (KeyboardInterrupt, EOFError):
        pass

    # ── 退出 ──────────────────────────────────────────────
    display.stop()

    robot.set_freedrive(False)
    print("freedrive 已退出")

    # ── 打印最终位姿（可复制的 Python 格式）───────────────
    time.sleep(0.3)
    try:
        joints = robot.getj()
        pose = robot.getl()
    except Exception:
        joints = None
        pose = None

    if joints is not None and pose is not None:
        print(f"\n{'='*60}")
        print(f"  最终位姿 (Freedrive 结束时的值)")
        print(f"  坐标系: UR arm base 系 (ur_arm_base_link)")
        print(f"{'='*60}")
        print(f"  home_j  = {[round(j, 8) for j in joints]}")
        print(f"  home    = {[round(p, 8) for p in pose]}")
        print(f"  (可直接复制到 CLAUDE.md / 配置文件中)")
        print(f"{'='*60}")

        # 同时打印 ur_home_pos/quat 作为参考，方便对比
        print(f"\n{'='*60}")
        print(f"  robot_move_script.py 中的 ur_home 参考值")
        print(f"  坐标系: UR arm base 系 (ur_arm_base_link)")
        print(f"{'='*60}")
        print(f"  ur_home_pos  = [-0.37137498, 0.03115423, 0.26795007]")
        print(f"  ur_home_quat = [0.00833616, 0.68928081, -0.00361613, -0.7244373]")
        print(f"  (来自 robot_move_script.py，独立位姿，与 ur3_grab.py 的 home/home_j 不同)")
        print(f"{'='*60}")

    robot.close()
    print("已断开连接")
