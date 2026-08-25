#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
keyboard_teleop.py — 导航键盘监听、急停与遥控接管节点

功能：
1. 默认处于 AUTONOMOUS NAVIGATION 模式：不发布 /cmd_vel，由 move_base / guide_manager 导航；
2. 当按下【空格键】时：触发急停（调用 /guide/cancel，发布 /move_base/cancel，下发零速制动），锁定为 MANUAL 遥控模式；
3. 在 MANUAL 模式下：
   - 空格 / X：急停制动 (持续发布零速，阻止底盘移动)
   - W/S 或 ↑/↓：前进/后退
   - A/D 或 ←/→：左转/右转
   - Q/Z：增加/减小线速度 (步进 10%)
   - E/C：增加/减小角速度 (步进 10%)
   - H：显示帮助菜单
4. 当下发新航点/导航目标时（/guide/goal, /guide/navigate, RViz 2D Nav Goal）：
   - 自动检测到新目标，立刻释放 /cmd_vel 控制权，切回 AUTONOMOUS NAVIGATION 模式。
"""

import os
import sys
import select
import termios
import tty
import threading
import json
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID


BANNER_HELP = """
===================================================================
 [Tracer Keyboard Teleop & Emergency Stop]
 -----------------------------------------------------------------
  当前模式: {mode}
 -----------------------------------------------------------------
  【快捷键说明】
    空格键 (SPACE) : 【急停】取消当前自主导航，切换至键盘遥控模式
    W / S 或 ↑ / ↓ : 前进 / 后退
    A / D 或 ← / → : 原地左转 / 原地右转
    X              : 停车制动 (零速)
    Q / Z          : 增加 / 减小线速度 (当前: {speed:.2f} m/s)
    E / C          : 增加 / 减小角速度 (当前: {turn:.2f} rad/s)
    H              : 显示此帮助菜单
 -----------------------------------------------------------------
  【切回自主导航】
    在 RViz 中点击 2D Nav Goal，或下达新航点任务时，将自动切回自主模式
===================================================================
"""


class KeyboardTeleop:
    MODE_AUTO = "AUTONOMOUS_NAVIGATION"
    MODE_MANUAL = "MANUAL_TELEOP"

    def __init__(self):
        self._lock = threading.RLock()
        self.mode = self.MODE_AUTO

        # 速度参数
        self.speed = float(rospy.get_param("~speed", 0.3))      # m/s
        self.turn = float(rospy.get_param("~turn", 0.6))        # rad/s
        self.max_speed = float(rospy.get_param("~max_speed", 1.0))
        self.max_turn = float(rospy.get_param("~max_turn", 1.5))
        self.repeat_rate = float(rospy.get_param("~repeat_rate", 20.0))  # Hz

        # 当前控制指令
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.last_key_time = rospy.Time(0)
        self.last_cancel_time = rospy.Time(0)
        self.key_timeout = 0.6  # 无按键持续 0.6s 后自动平滑归零

        # 话题与服务
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pub_move_base_cancel = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)

        # 订阅新目标检测（仅当收到新的明确指令时才切回 AUTO）
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self._on_move_base_goal, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._on_simple_goal, queue_size=1)
        rospy.Subscriber("/guide/goal", String, self._on_guide_goal, queue_size=1)

        # 导引服务代理（取消导航）
        self._srv_guide_cancel = rospy.ServiceProxy("/guide/cancel", Trigger)

        # 终端属性保存
        self.old_terminal_settings = None
        self.tty_fd = None

        # 控制循环定时器 (20Hz)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.repeat_rate), self._control_loop)

        rospy.loginfo("[KeyboardTeleop] Initialized. Default Mode: %s", self.mode)
        self.print_banner()

    def print_banner(self):
        try:
            sys.stdout.write(
                BANNER_HELP.format(
                    mode=self.mode,
                    speed=self.speed,
                    turn=self.turn
                )
            )
            sys.stdout.flush()
        except Exception:
            pass

    def _on_guide_goal(self, msg):
        """检测到下发新航点时切回 AUTO"""
        with self._lock:
            if (rospy.Time.now() - self.last_cancel_time).to_sec() < 1.0:
                return  # 过滤急停刚触发时的旧消息
            if self.mode == self.MODE_MANUAL:
                self.mode = self.MODE_AUTO
                self.target_linear_x = 0.0
                self.target_angular_z = 0.0
                rospy.loginfo(
                    "\n>>> [MODE SWITCH] New guide waypoint '%s' received. Switched to AUTO MODE <<<\n",
                    msg.data.strip()
                )
                self.print_banner()

    def _on_move_base_goal(self, msg):
        """检测到 move_base 收到新 goal 时切回 AUTO"""
        with self._lock:
            if (rospy.Time.now() - self.last_cancel_time).to_sec() < 1.0:
                return
            if self.mode == self.MODE_MANUAL:
                self.mode = self.MODE_AUTO
                self.target_linear_x = 0.0
                self.target_angular_z = 0.0
                rospy.loginfo(
                    "\n>>> [MODE SWITCH] New move_base goal detected. Switched to AUTO MODE <<<\n"
                )
                self.print_banner()

    def _on_simple_goal(self, msg):
        """检测到 RViz 2D Nav Goal 时切回 AUTO"""
        with self._lock:
            if (rospy.Time.now() - self.last_cancel_time).to_sec() < 1.0:
                return
            if self.mode == self.MODE_MANUAL:
                self.mode = self.MODE_AUTO
                self.target_linear_x = 0.0
                self.target_angular_z = 0.0
                rospy.loginfo(
                    "\n>>> [MODE SWITCH] 2D Nav Goal received. Switched to AUTO MODE <<<\n"
                )
                self.print_banner()

    def emergency_stop_and_takeover(self):
        """触发急停并锁定为键盘遥控模式"""
        with self._lock:
            self.mode = self.MODE_MANUAL
            self.last_cancel_time = rospy.Time.now()
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0
            self.last_key_time = rospy.Time.now()

            # 1. 立即下发零速制动
            twist = Twist()
            for _ in range(3):
                self.pub_cmd_vel.publish(twist)

            # 2. 调用 /guide/cancel 服务取消航点导航与倒计时
            try:
                if self._srv_guide_cancel.wait_for_service(timeout=rospy.Duration(0.5)):
                    resp = self._srv_guide_cancel()
                    rospy.logwarn("[KeyboardTeleop] Called /guide/cancel: %s", resp.message)
            except Exception as e:
                pass

            # 3. 双重保险：直接取消 move_base
            try:
                self.pub_move_base_cancel.publish(GoalID())
            except Exception:
                pass

            rospy.logwarn(
                "\n***************************************************\n"
                "  [EMERGENCY STOP ACTIVATED] Navigation Cancelled! \n"
                "  Vehicle stopped. Switched to MANUAL TELEOP MODE.\n"
                "  Use W/S/A/D or Arrows to drive, Space to stop.  \n"
                "***************************************************\n"
            )
            self.print_banner()

    def _control_loop(self, event):
        """在 MANUAL 模式下以固定高频(20Hz)持续发布 /cmd_vel，保证底盘在急停后绝对静止或受控"""
        with self._lock:
            if self.mode != self.MODE_MANUAL:
                return

            now = rospy.Time.now()
            # 如果超过一段时间没有按移动键，速度自动归零（防失控）
            if (now - self.last_key_time).to_sec() > self.key_timeout:
                self.target_linear_x = 0.0
                self.target_angular_z = 0.0

            twist = Twist()
            twist.linear.x = self.target_linear_x
            twist.angular.z = self.target_angular_z
            self.pub_cmd_vel.publish(twist)

    def _get_key(self):
        """非阻塞读取单个字符或方向键转义序列"""
        fd = self.tty_fd if self.tty_fd is not None else sys.stdin.fileno()
        r, _, _ = select.select([fd], [], [], 0.05)
        if not r:
            return None

        try:
            if self.tty_fd is not None:
                ch = os.read(self.tty_fd, 1).decode('latin1')
            else:
                ch = sys.stdin.read(1)
        except Exception:
            return None

        if ch == "\x1b":  # 方向键转义序列 \x1b[A 等
            try:
                if self.tty_fd is not None:
                    seq = os.read(self.tty_fd, 2).decode('latin1')
                else:
                    seq = sys.stdin.read(2)
                if seq == "[A":
                    return "UP"
                elif seq == "[B":
                    return "DOWN"
                elif seq == "[C":
                    return "RIGHT"
                elif seq == "[D":
                    return "LEFT"
            except Exception:
                pass
            return "ESC"
        return ch

    def run(self):
        # 尝试获取可用 TTY 设备 (sys.stdin 或 /dev/tty)
        target_fd = None
        if sys.stdin.isatty():
            target_fd = sys.stdin.fileno()
        else:
            try:
                self.tty_fd = os.open("/dev/tty", os.O_RDONLY | os.O_NONBLOCK)
                target_fd = self.tty_fd
                rospy.loginfo("[KeyboardTeleop] Bound directly to /dev/tty for input reading.")
            except Exception as e:
                rospy.logwarn("[KeyboardTeleop] Neither stdin nor /dev/tty is available (%s).", e)

        if target_fd is not None:
            try:
                self.old_terminal_settings = termios.tcgetattr(target_fd)
                tty.setraw(target_fd)
            except Exception as exc:
                rospy.logwarn("[KeyboardTeleop] Failed to set raw terminal mode: %s", exc)

        try:
            while not rospy.is_shutdown():
                key = self._get_key()
                if key is None:
                    continue

                # 处理 Ctrl+C
                if key == "\x03":
                    rospy.loginfo("Ctrl+C detected. Exiting.")
                    break

                with self._lock:
                    # 1. 空格键：任何模式下一键急停并锁定 MANUAL 模式
                    if key == " ":
                        self.emergency_stop_and_takeover()
                        continue

                    # 2. 帮助菜单
                    if key.lower() == "h" or key == "?":
                        self.print_banner()
                        continue

                    # 3. 调速按键
                    if key.lower() == "q":
                        self.speed = min(self.max_speed, round(self.speed * 1.1, 3))
                        rospy.loginfo("Linear speed: %.2f m/s", self.speed)
                        continue
                    elif key.lower() == "z":
                        self.speed = max(0.05, round(self.speed * 0.9, 3))
                        rospy.loginfo("Linear speed: %.2f m/s", self.speed)
                        continue
                    elif key.lower() == "e":
                        self.turn = min(self.max_turn, round(self.turn * 1.1, 3))
                        rospy.loginfo("Angular speed: %.2f rad/s", self.turn)
                        continue
                    elif key.lower() == "c":
                        self.turn = max(0.1, round(self.turn * 0.9, 3))
                        rospy.loginfo("Angular speed: %.2f rad/s", self.turn)
                        continue

                    # 4. 如果不在 MANUAL 模式，忽略移动按键并提示
                    if self.mode != self.MODE_MANUAL:
                        if key.lower() in ("w", "s", "a", "d", "x") or key in ("UP", "DOWN", "LEFT", "RIGHT"):
                            rospy.loginfo_throttle(
                                2.0,
                                "[KeyboardTeleop] Currently in AUTO NAV mode. Press SPACE first to stop and take over."
                            )
                        continue

                    # 5. MANUAL 模式下的移动控制
                    self.last_key_time = rospy.Time.now()

                    if key.lower() == "w" or key == "UP":
                        self.target_linear_x = self.speed
                        self.target_angular_z = 0.0
                    elif key.lower() == "s" or key == "DOWN":
                        self.target_linear_x = -self.speed
                        self.target_angular_z = 0.0
                    elif key.lower() == "a" or key == "LEFT":
                        self.target_linear_x = 0.0
                        self.target_angular_z = self.turn
                    elif key.lower() == "d" or key == "RIGHT":
                        self.target_linear_x = 0.0
                        self.target_angular_z = -self.turn
                    elif key.lower() == "x":
                        self.target_linear_x = 0.0
                        self.target_angular_z = 0.0

        except Exception as exc:
            rospy.logerr("[KeyboardTeleop] Error in keyboard loop: %s", exc)
        finally:
            self.restore_terminal()

    def restore_terminal(self):
        fd = self.tty_fd if self.tty_fd is not None else sys.stdin.fileno()
        if self.old_terminal_settings is not None:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, self.old_terminal_settings)
            except Exception:
                pass
        if self.tty_fd is not None:
            try:
                os.close(self.tty_fd)
            except Exception:
                pass


def main():
    rospy.init_node("keyboard_teleop", anonymous=False)
    teleop = KeyboardTeleop()
    teleop.run()


if __name__ == "__main__":
    main()
