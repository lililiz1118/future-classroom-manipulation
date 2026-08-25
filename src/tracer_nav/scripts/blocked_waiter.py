#!/usr/bin/env python3
# coding=utf8
# 导航监督器: 堵路静默等待 + 大掉头预旋转 (2026-07-27 v2)
#
# 功能1 堵路等待: TEB 没有"原地等待"概念——堵死时仍每周期尝试接近目标,
#   表现为原地摆动直到弃单。检测"有目标但持续无进展"→取消目标(车真正静止)
#   → 周期用 /move_base/make_plan 纯计算探路 → 路通后自动重发原目标。
#
# 功能2 预旋转(用户设计): 倒车重罚(300)保住车尾盲区安全, 但 TEB 的轨迹
#   词汇表里没有中途原地旋转, 大掉头会被优化成"前进画圈"。解法: 新目标的
#   起始路径方向与车头夹角>100°时, 先发一个"原地转到位"的中间目标(纯旋转,
#   TEB 原生支持且带障碍检查), 转完再发真目标。
#
# 安全设计: 本节点只会 cancel 和 resend 目标, 永不发布运动指令;
# 误判的最坏后果 = 多停几秒(等待)或多转一次(预旋转)后自动恢复。
# 预留: 进入等待时可接 /voice/speak 播报"请让一让"(guide_manager 阶段接入)。
import collections
import math

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatusArray, GoalID
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from tf.transformations import quaternion_from_euler, euler_from_quaternion

STUCK_WINDOW = 6.0     # 持续无进展判定窗口(s)
MIN_PROGRESS = 0.05    # 窗口内距目标距离至少应减少(m)
GOAL_MIN_AGE = 8.0     # 目标刚下发的起步期不判定堵路
NEAR_GOAL_SKIP = 0.5   # 距目标很近时(终点微调)不判定
CHECK_PERIOD = 5.0     # 等待期探路间隔(s)
PREROTATE_DEG = 100.0  # 起始路径方向与车头夹角超过此值触发预旋转
ACTIVE_STATES = {0, 1, 6, 7}

goal_pose = None       # 当前真目标 (PoseStamped, map系)
goal_stamp = 0.0
active = False
waiting = False
progress = collections.deque()

prerotating = False    # 正在执行预旋转中间目标
pending_goal = None    # 预旋转完成后要补发的真目标
prerotate_done_for = None   # 已做过预旋转的目标标识(防循环)
latest_plan = None


def goal_key(p):
    return (round(p.pose.position.x, 2), round(p.pose.position.y, 2))


def cb_goal(msg):
    global goal_pose, goal_stamp, waiting, prerotating
    p = msg.goal.target_pose
    goal_pose = p
    goal_stamp = rospy.get_time()
    waiting = False
    progress.clear()
    # 我们自己补发的目标(预旋转/续航)不再触发新一轮预旋转判定,
    # 由 prerotate_done_for / prerotating 状态区分


def cb_status(msg):
    global active
    active = any(s.status in ACTIVE_STATES for s in msg.status_list)


def cb_plan(msg):
    global latest_plan
    if len(msg.poses) >= 2:
        latest_plan = msg


def robot_pose(buf):
    t = buf.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
    q = t.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return t.transform.translation.x, t.transform.translation.y, yaw


def ang_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def path_initial_dir(plan, rx, ry):
    """全局路径开头~0.5m段的前进方向"""
    px, py = rx, ry
    for pose in plan.poses:
        dx = pose.pose.position.x - rx
        dy = pose.pose.position.y - ry
        if math.hypot(dx, dy) > 0.5:
            return math.atan2(dy, dx)
        px, py = pose.pose.position.x, pose.pose.position.y
    dx, dy = px - rx, py - ry
    if math.hypot(dx, dy) > 0.1:
        return math.atan2(dy, dx)
    return None


def main():
    global waiting, prerotating, pending_goal, prerotate_done_for
    rospy.init_node('blocked_waiter')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, cb_goal, queue_size=1)
    rospy.Subscriber('/move_base/status', GoalStatusArray, cb_status, queue_size=1)
    rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, cb_plan, queue_size=1)
    pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
    pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.wait_for_service('/move_base/make_plan')
    make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
    rospy.loginfo('blocked_waiter v2: 堵路静默等待 + >%.0f°大掉头预旋转', PREROTATE_DEG)

    last_check = 0.0
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rate.sleep()
        now = rospy.get_time()
        if goal_pose is None:
            continue
        try:
            rx, ry, ryaw = robot_pose(buf)
        except Exception:
            continue
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y
        dist = math.hypot(gx - rx, gy - ry)

        # ── 预旋转状态机 ──
        if prerotating:
            if not active:   # 旋转目标已完成(或失败)
                prerotating = False
                if pending_goal is not None:
                    rospy.loginfo('blocked_waiter: 预旋转完成, 下发真目标')
                    pending_goal.header.stamp = rospy.Time.now()
                    pub_goal.publish(pending_goal)
                    pending_goal = None
            continue

        # 新的真目标: 判断是否需要预旋转(每个目标只做一次)
        if (active and latest_plan is not None and dist > 1.0
                and prerotate_done_for != goal_key(goal_pose)
                and now - goal_stamp < 3.0):
            pdir = path_initial_dir(latest_plan, rx, ry)
            if pdir is not None and abs(ang_diff(pdir, ryaw)) > math.radians(PREROTATE_DEG):
                rospy.loginfo('blocked_waiter: 路径方向与车头差%.0f°>阈值, 先原地预旋转',
                              math.degrees(abs(ang_diff(pdir, ryaw))))
                prerotate_done_for = goal_key(goal_pose)
                pending_goal = goal_pose
                pub_cancel.publish(GoalID())
                rot = PoseStamped()
                rot.header.frame_id = 'map'
                rot.header.stamp = rospy.Time.now()
                rot.pose.position.x = rx
                rot.pose.position.y = ry
                q = quaternion_from_euler(0, 0, pdir)
                rot.pose.orientation.x = q[0]
                rot.pose.orientation.y = q[1]
                rot.pose.orientation.z = q[2]
                rot.pose.orientation.w = q[3]
                prerotating = True
                rospy.sleep(0.3)     # 等cancel生效
                pub_goal.publish(rot)
                continue

        # ── 堵路等待状态机 ──
        if waiting:
            if now - last_check < CHECK_PERIOD:
                continue
            last_check = now
            try:
                start = PoseStamped()
                start.header.frame_id = 'map'
                start.header.stamp = rospy.Time(0)
                start.pose.position.x = rx
                start.pose.position.y = ry
                start.pose.orientation.w = 1.0
                resp = make_plan(start=start, goal=goal_pose, tolerance=0.3)
                if len(resp.plan.poses) > 0:
                    rospy.loginfo('blocked_waiter: 路已通, 重发目标续航')
                    goal_pose.header.stamp = rospy.Time.now()
                    pub_goal.publish(goal_pose)
                    waiting = False
                    progress.clear()
                else:
                    rospy.loginfo('blocked_waiter: 仍堵塞, 继续静默等待')
                    # TODO(guide_manager): 此处接 /voice/speak "请让一让"
            except Exception as e:
                rospy.logwarn('make_plan 调用失败: %s', e)
            continue

        if not active or now - goal_stamp < GOAL_MIN_AGE or dist < NEAR_GOAL_SKIP:
            progress.clear()
            continue
        progress.append((now, dist))
        while progress and now - progress[0][0] > STUCK_WINDOW:
            progress.popleft()
        if progress and now - progress[0][0] > STUCK_WINDOW * 0.9:
            if progress[0][1] - dist < MIN_PROGRESS:
                rospy.logwarn('blocked_waiter: %.0fs 无进展, 取消目标静默等待 (堵路)', STUCK_WINDOW)
                pub_cancel.publish(GoalID())
                waiting = True
                last_check = now
                progress.clear()


if __name__ == '__main__':
    main()
