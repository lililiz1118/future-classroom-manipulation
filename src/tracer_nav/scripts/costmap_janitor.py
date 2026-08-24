#!/usr/bin/env python3
# coding=utf8
# 周期性清理代价地图 v2 (2026-07-27)
#
# v1 教训(真机撞击事故): 固定20s清图, 恰逢机器人逼近障碍——障碍处于近场
# 盲环(BoxFilter盒内返点被置NaN/被遮挡)时, 2s观测池里没有它的新观测,
# 清图后无法补标 → TEB看到干净地图径直前进撞击; 同时全局图被清空,
# "路线堵死"的信息也丢了, 规划器认为路通、不会停车重规划。
#
# v2 原则: **导航中绝不清图**。只在 move_base 无活动目标(且已空闲
# idle_grace 秒)时清理——行人轨迹残留照样在任务间隙被扫掉, 任务执行
# 期间障碍标记连续性完整保留。
import threading

import rospy
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalStatusArray

ACTIVE_STATES = {0, 1, 6, 7}  # PENDING/ACTIVE/PREEMPTING/RECALLING
last_active = [0.0]

def cb_status(msg):
    if any(s.status in ACTIVE_STATES for s in msg.status_list):
        last_active[0] = rospy.get_time()

rospy.init_node('costmap_janitor')
period = rospy.get_param('~period', 20.0)
idle_grace = rospy.get_param('~idle_grace', 3.0)
rospy.Subscriber('/move_base/status', GoalStatusArray, cb_status, queue_size=1)
rospy.wait_for_service('/move_base/clear_costmaps')
clear = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
rospy.loginfo('costmap_janitor v2: clear every %.0fs, ONLY when no active goal (idle_grace=%.0fs)',
              period, idle_grace)
def do_clear():
    # 独立守护线程调用: 关机时move_base先死会卡死阻塞的服务调用,
    # 守护线程不阻碍进程退出(修复Ctrl-C需SIGKILL的问题)
    try:
        clear()
        rospy.loginfo('janitor: costmaps cleared (idle)')
    except Exception as e:
        rospy.logwarn('clear_costmaps failed: %s', e)

elapsed = 0.0
while not rospy.is_shutdown():
    rospy.sleep(0.5)          # 小步长睡眠, 关停响应快
    elapsed += 0.5
    if elapsed < period:
        continue
    elapsed = 0.0
    if rospy.get_time() - last_active[0] < idle_grace:
        rospy.logdebug('janitor: goal active, skip')
        continue
    threading.Thread(target=do_clear, daemon=True).start()
