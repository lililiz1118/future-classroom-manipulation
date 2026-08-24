#!/usr/bin/env python3
"""Owns move_base goals and provides named waypoint navigation with recovery."""

import collections
import copy
import json
import math
import os
import threading

import actionlib
import rospkg
import rospy
import tf2_ros
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty, Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from tracer_nav.msg import (
    NavigateWaypointAction,
    NavigateWaypointFeedback,
    NavigateWaypointResult,
)


ACTIVE_STATES = {
    "PREPARING",
    "PREROTATING",
    "NAVIGATING",
    "RETRYING",
    "WAITING_FOR_PATH",
    "PAUSED",
}
TERMINAL_STATES = {"SUCCEEDED", "FAILED", "CANCELED"}


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def yaw_from_quaternion(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class GuideManager:
    def __init__(self):
        self._lock = threading.RLock()
        self._waypoints_file = rospy.get_param(
            "~waypoints_file",
            os.path.join(
                rospkg.RosPack().get_path("tracer_nav"), "config", "waypoints.yaml"
            ),
        )
        self._base_frame = rospy.get_param("~base_frame", "base_link")
        self._move_base_action = rospy.get_param("~move_base_action", "/move_base")
        self._make_plan_service = rospy.get_param(
            "~make_plan_service", "/move_base/make_plan"
        )
        self._clear_service = rospy.get_param(
            "~clear_costmaps_service", "/move_base/clear_costmaps"
        )
        self._default_retries = int(rospy.get_param("~default_retries", 2))
        self._retry_delay = float(rospy.get_param("~retry_delay", 1.5))
        self._blocked_timeout = float(rospy.get_param("~blocked_timeout", 12.0))
        self._start_grace = float(rospy.get_param("~start_grace", 8.0))
        self._min_translation = float(rospy.get_param("~min_translation", 0.05))
        self._min_rotation = math.radians(
            float(rospy.get_param("~min_rotation_deg", 10.0))
        )
        self._near_goal_distance = float(
            rospy.get_param("~near_goal_distance", 0.5)
        )
        self._plan_check_period = float(
            rospy.get_param("~plan_check_period", 5.0)
        )
        self._plan_tolerance = float(rospy.get_param("~plan_tolerance", 0.3))
        self._max_blocked_wait = float(rospy.get_param("~max_blocked_wait", 0.0))
        self._prerotate_enabled = bool(
            rospy.get_param("~prerotate_enabled", True)
        )
        self._prerotate_angle = math.radians(
            float(rospy.get_param("~prerotate_angle_deg", 100.0))
        )
        self._prerotate_path_distance = float(
            rospy.get_param("~prerotate_path_distance", 0.5)
        )
        self._start_delay = float(rospy.get_param("~start_delay", 3.0))
        if self._default_retries < 0:
            raise ValueError("default_retries must be non-negative")
        if self._retry_delay < 0.0:
            raise ValueError("retry_delay must be non-negative")
        if self._blocked_timeout <= 0.0 or self._plan_check_period <= 0.0:
            raise ValueError("blocked_timeout and plan_check_period must be positive")
        if self._start_grace < 0.0 or self._plan_tolerance < 0.0:
            raise ValueError("start_grace and plan_tolerance must be non-negative")
        if self._start_delay < 0.0:
            raise ValueError("start_delay must be non-negative")

        self._frame_id = "map"
        self._home_waypoint = "home"
        self._waypoints = {}
        self._load_waypoints()

        self._task_id = 0
        self._leg_id = 0
        self._task_owner = "none"
        self._waypoint_name = ""
        self._target_pose = None
        self._state = "IDLE"
        self._message = "ready"
        self._distance_remaining = float("nan")
        self._retry_count = 0
        self._max_retries = self._default_retries
        self._goal_sent_at = 0.0
        self._waiting_since = 0.0
        self._last_plan_probe = 0.0
        self._probe_running = False
        self._progress = collections.deque()

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._move_base = actionlib.SimpleActionClient(
            self._move_base_action, MoveBaseAction
        )
        wait_timeout = float(rospy.get_param("~move_base_wait_timeout", 60.0))
        rospy.loginfo("guide_manager: waiting for %s", self._move_base_action)
        if not self._move_base.wait_for_server(rospy.Duration(wait_timeout)):
            raise RuntimeError(
                "move_base action server unavailable after %.1fs" % wait_timeout
            )

        self._make_plan = rospy.ServiceProxy(self._make_plan_service, GetPlan)
        self._clear_costmaps = rospy.ServiceProxy(self._clear_service, Empty)

        self._state_pub = rospy.Publisher(
            "/guide/state", String, queue_size=1, latch=True
        )
        self._active_pub = rospy.Publisher(
            "/guide/active_waypoint", String, queue_size=1, latch=True
        )
        self._distance_pub = rospy.Publisher(
            "/guide/distance_remaining", Float32, queue_size=1
        )
        self._cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/guide/goal", String, self._goal_topic_callback, queue_size=1)
        rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self._rviz_goal_callback, queue_size=1
        )
        rospy.Service("/guide/pause", Trigger, self._pause_service)
        rospy.Service("/guide/resume", Trigger, self._resume_service)
        rospy.Service("/guide/cancel", Trigger, self._cancel_service)
        rospy.Service("/guide/retry", Trigger, self._retry_service)
        rospy.Service("/guide/home", Trigger, self._home_service)
        rospy.Service("/guide/clear_costmaps", Trigger, self._clear_service_callback)
        rospy.Service("/guide/reload_waypoints", Trigger, self._reload_service)

        self._action_server = actionlib.SimpleActionServer(
            "/guide/navigate",
            NavigateWaypointAction,
            execute_cb=self._execute_action,
            auto_start=False,
        )
        self._action_server.start()

        self._timer = rospy.Timer(rospy.Duration(0.5), self._monitor)
        self._publish_state()
        rospy.loginfo(
            "guide_manager ready: %d waypoints, home=%s",
            len(self._waypoints),
            self._home_waypoint,
        )

    def _load_waypoints(self):
        path = os.path.abspath(os.path.expanduser(self._waypoints_file))
        if not os.path.exists(path):
            directory = os.path.dirname(path)
            os.makedirs(directory, exist_ok=True)
            initial_data = {
                "frame_id": "map",
                "home_waypoint": "home",
                "waypoints": {
                    "home": {"x": 0.0, "y": 0.0, "yaw": 0.0}
                }
            }
            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump(initial_data, f, allow_unicode=True, sort_keys=False)
            rospy.loginfo("guide_manager: initialized missing waypoints file: %s", path)

        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        frame_id = str(data.get("frame_id", "map"))
        home_waypoint = str(data.get("home_waypoint", "home"))
        raw_waypoints = data.get("waypoints") or {}
        if not isinstance(raw_waypoints, dict) or not raw_waypoints:
            raise ValueError("waypoints.yaml must contain a non-empty waypoints map")

        parsed = {}
        for name, values in raw_waypoints.items():
            waypoint_name = str(name).strip()
            if not waypoint_name:
                raise ValueError("waypoint names must not be empty")
            if not isinstance(values, dict):
                raise ValueError("waypoint %s must be a mapping" % name)
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(values["x"])
            pose.pose.position.y = float(values["y"])
            pose.pose.position.z = float(values.get("z", 0.0))
            position = (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )
            if not all(math.isfinite(value) for value in position):
                raise ValueError("waypoint %s position must be finite" % name)
            if "quaternion" in values:
                quat = values["quaternion"]
                if not isinstance(quat, (list, tuple)) or len(quat) != 4:
                    raise ValueError("waypoint %s quaternion must have 4 values" % name)
                quat = [float(value) for value in quat]
                if not all(math.isfinite(value) for value in quat):
                    raise ValueError("waypoint %s quaternion must be finite" % name)
                norm = math.sqrt(sum(value * value for value in quat))
                if norm < 1e-9:
                    raise ValueError("waypoint %s quaternion must not be zero" % name)
                quat = [value / norm for value in quat]
                pose.pose.orientation.x = float(quat[0])
                pose.pose.orientation.y = float(quat[1])
                pose.pose.orientation.z = float(quat[2])
                pose.pose.orientation.w = float(quat[3])
            else:
                yaw = float(values.get("yaw", 0.0))
                if not math.isfinite(yaw):
                    raise ValueError("waypoint %s yaw must be finite" % name)
                quat = quaternion_from_euler(0.0, 0.0, yaw)
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
            parsed[waypoint_name] = pose

        if home_waypoint not in parsed:
            raise ValueError("home_waypoint %s is not defined" % home_waypoint)

        with self._lock:
            self._frame_id = frame_id
            self._home_waypoint = home_waypoint
            self._waypoints = parsed
            self._waypoints_file = path

    def _current_pose(self):
        transform = self._tf_buffer.lookup_transform(
            self._frame_id,
            self._base_frame,
            rospy.Time(0),
            rospy.Duration(1.0),
        )
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _make_plan_to_target(self, target):
        start = self._current_pose()
        request = GetPlanRequest()
        request.start = start
        request.goal = copy.deepcopy(target)
        request.goal.header.stamp = rospy.Time.now()
        request.tolerance = self._plan_tolerance
        response = self._make_plan(request)
        return start, response.plan

    def _initial_path_yaw(self, start, path):
        sx = start.pose.position.x
        sy = start.pose.position.y
        last_x = sx
        last_y = sy
        for stamped_pose in path.poses:
            x = stamped_pose.pose.position.x
            y = stamped_pose.pose.position.y
            if math.hypot(x - sx, y - sy) >= self._prerotate_path_distance:
                return math.atan2(y - sy, x - sx)
            last_x, last_y = x, y
        if math.hypot(last_x - sx, last_y - sy) > 0.1:
            return math.atan2(last_y - sy, last_x - sx)
        return None

    def _begin_task(self, waypoint_name, max_retries, owner):
        with self._lock:
            if waypoint_name not in self._waypoints:
                return None, "unknown waypoint: %s" % waypoint_name
            self._task_id += 1
            self._leg_id += 1
            task_id = self._task_id
            self._task_owner = owner
            self._waypoint_name = waypoint_name
            self._target_pose = copy.deepcopy(self._waypoints[waypoint_name])
            self._state = "PREPARING"
            self._message = "preparing route"
            self._distance_remaining = float("nan")
            self._retry_count = 0
            self._max_retries = (
                self._default_retries if max_retries < 0 else max_retries
            )
            self._goal_sent_at = 0.0
            self._waiting_since = 0.0
            self._last_plan_probe = 0.0
            self._probe_running = False
            self._progress.clear()
            self._move_base.cancel_goal()

        self._publish_state()
        threading.Thread(
            target=self._prepare_and_dispatch,
            args=(task_id, True),
            daemon=True,
        ).start()
        return task_id, "accepted"

    def _begin_pose_task(self, target_pose, max_retries, owner):
        with self._lock:
            self._task_id += 1
            self._leg_id += 1
            task_id = self._task_id
            self._task_owner = owner
            self._waypoint_name = "rviz_goal"
            self._target_pose = copy.deepcopy(target_pose)
            self._state = "PREPARING"
            self._message = "preparing route"
            self._distance_remaining = float("nan")
            self._retry_count = 0
            self._max_retries = (
                self._default_retries if max_retries < 0 else max_retries
            )
            self._goal_sent_at = 0.0
            self._waiting_since = 0.0
            self._last_plan_probe = 0.0
            self._probe_running = False
            self._progress.clear()
            self._move_base.cancel_goal()

        self._publish_state()
        threading.Thread(
            target=self._prepare_and_dispatch,
            args=(task_id, True),
            daemon=True,
        ).start()
        return task_id, "accepted"

    def _rviz_goal_callback(self, msg):
        self._begin_pose_task(msg, self._default_retries, "rviz")

    def _prepare_and_dispatch(self, task_id, allow_prerotate):
        with self._lock:
            if (
                task_id != self._task_id
                or self._state not in {"PREPARING", "RETRYING"}
            ):
                return
            target = copy.deepcopy(self._target_pose)

        # 启动前延迟倒计时（留出时间切换至键盘遥控终端）
        if allow_prerotate and self._start_delay > 0.0:
            rospy.loginfo(
                "\n>>> [guide_manager] Navigation goal accepted! Starting in %.1fs... (Switch terminal / Press SPACE to cancel) <<<\n",
                self._start_delay,
            )
            delay_start = rospy.get_time()
            while rospy.get_time() - delay_start < self._start_delay:
                remaining = self._start_delay - (rospy.get_time() - delay_start)
                with self._lock:
                    if task_id != self._task_id or self._state not in {"PREPARING", "RETRYING"}:
                        rospy.loginfo("[guide_manager] Navigation startup canceled during countdown.")
                        return
                    self._message = "starting in %.1fs" % remaining
                self._publish_state()
                rospy.sleep(0.1)

        if self._prerotate_enabled and allow_prerotate:
            try:
                rospy.wait_for_service(self._make_plan_service, timeout=2.0)
                start, path = self._make_plan_to_target(target)
                path_yaw = self._initial_path_yaw(start, path)
                robot_yaw = yaw_from_quaternion(start.pose.orientation)
                if (
                    path_yaw is not None
                    and abs(angle_difference(path_yaw, robot_yaw))
                    > self._prerotate_angle
                ):
                    rotation_goal = copy.deepcopy(start)
                    quat = quaternion_from_euler(0.0, 0.0, path_yaw)
                    rotation_goal.pose.orientation.x = quat[0]
                    rotation_goal.pose.orientation.y = quat[1]
                    rotation_goal.pose.orientation.z = quat[2]
                    rotation_goal.pose.orientation.w = quat[3]
                    self._send_leg(task_id, rotation_goal, "PREROTATING")
                    return
            except Exception as exc:
                rospy.logwarn("guide_manager: pre-rotation check skipped: %s", exc)

        self._send_leg(task_id, target, "NAVIGATING")

    def _send_leg(self, task_id, pose, phase):
        with self._lock:
            if (
                task_id != self._task_id
                or self._state not in {"PREPARING", "RETRYING"}
            ):
                return
            self._leg_id += 1
            leg_id = self._leg_id
            goal = MoveBaseGoal()
            goal.target_pose = copy.deepcopy(pose)
            goal.target_pose.header.stamp = rospy.Time.now()
            self._state = phase
            self._message = (
                "rotating to path heading"
                if phase == "PREROTATING"
                else "moving to waypoint"
            )
            self._goal_sent_at = rospy.get_time()
            self._waiting_since = 0.0
            self._progress.clear()
            self._move_base.send_goal(
                goal,
                done_cb=lambda status, result: self._move_base_done(
                    task_id, leg_id, phase, status, result
                ),
                feedback_cb=lambda feedback: self._move_base_feedback(
                    task_id, leg_id, phase, feedback
                ),
            )
        self._publish_state()

    def _move_base_done(self, task_id, leg_id, phase, status, _result):
        with self._lock:
            if task_id != self._task_id or leg_id != self._leg_id:
                return
            if self._state in {"PAUSED", "WAITING_FOR_PATH", "CANCELED"}:
                return

            if status == GoalStatus.SUCCEEDED:
                if phase == "PREROTATING":
                    self._state = "PREPARING"
                    self._message = "pre-rotation complete"
                    threading.Thread(
                        target=self._prepare_and_dispatch,
                        args=(task_id, False),
                        daemon=True,
                    ).start()
                else:
                    self._state = "SUCCEEDED"
                    self._message = "waypoint reached"
                    self._distance_remaining = 0.0
                self._publish_state()
                return

            if status == GoalStatus.PREEMPTED:
                self._state = "FAILED"
                self._message = "move_base goal was preempted externally"
                self._publish_state()
                return

            if self._retry_count < self._max_retries:
                self._retry_count += 1
                self._state = "RETRYING"
                self._message = "move_base failed; retry scheduled"
                self._progress.clear()
                threading.Thread(
                    target=self._retry_after_delay,
                    args=(task_id,),
                    daemon=True,
                ).start()
            else:
                self._state = "FAILED"
                self._message = "move_base failed with status %d" % status
            self._publish_state()

    def _retry_after_delay(self, task_id):
        rospy.sleep(self._retry_delay)
        with self._lock:
            if task_id != self._task_id or self._state != "RETRYING":
                return
        self._prepare_and_dispatch(task_id, True)

    def _move_base_feedback(self, task_id, leg_id, phase, feedback):
        if phase != "NAVIGATING":
            return
        pose = feedback.base_position.pose
        now = rospy.get_time()
        with self._lock:
            if (
                task_id != self._task_id
                or leg_id != self._leg_id
                or self._state != "NAVIGATING"
            ):
                return
            dx = self._target_pose.pose.position.x - pose.position.x
            dy = self._target_pose.pose.position.y - pose.position.y
            distance = math.hypot(dx, dy)
            yaw = yaw_from_quaternion(pose.orientation)
            self._distance_remaining = distance
            self._progress.append((now, pose.position.x, pose.position.y, yaw, distance))
            while self._progress and now - self._progress[0][0] > self._blocked_timeout:
                self._progress.popleft()

        self._distance_pub.publish(Float32(data=distance))

    def _enter_waiting(self, task_id, leg_id):
        with self._lock:
            if (
                task_id != self._task_id
                or leg_id != self._leg_id
                or self._state != "NAVIGATING"
            ):
                return
            self._leg_id += 1
            self._state = "WAITING_FOR_PATH"
            self._message = "no progress; waiting for a valid path"
            self._waiting_since = rospy.get_time()
            self._last_plan_probe = 0.0
            self._progress.clear()
            self._move_base.cancel_goal()
        self._publish_state()

    def _probe_path(self, task_id):
        try:
            with self._lock:
                if task_id != self._task_id or self._state != "WAITING_FOR_PATH":
                    return
                target = copy.deepcopy(self._target_pose)
            _start, path = self._make_plan_to_target(target)
            if len(path.poses) >= 2:
                with self._lock:
                    if task_id != self._task_id or self._state != "WAITING_FOR_PATH":
                        return
                    self._state = "PREPARING"
                    self._message = "path available; resuming"
                self._prepare_and_dispatch(task_id, False)
            else:
                with self._lock:
                    if task_id == self._task_id and self._state == "WAITING_FOR_PATH":
                        self._message = "path still blocked"
        except Exception as exc:
            with self._lock:
                if task_id == self._task_id and self._state == "WAITING_FOR_PATH":
                    self._message = "path check failed: %s" % exc
        finally:
            with self._lock:
                if task_id == self._task_id:
                    self._probe_running = False
            self._publish_state()

    def _monitor(self, _event):
        now = rospy.get_time()
        start_probe = False
        enter_waiting = None
        with self._lock:
            if self._state == "NAVIGATING" and self._progress:
                old = self._progress[0]
                new = self._progress[-1]
                enough_history = new[0] - old[0] >= self._blocked_timeout * 0.9
                outside_start_grace = now - self._goal_sent_at >= self._start_grace
                if enough_history and outside_start_grace and new[4] > self._near_goal_distance:
                    translation = math.hypot(new[1] - old[1], new[2] - old[2])
                    rotation = abs(angle_difference(new[3], old[3]))
                    distance_gain = old[4] - new[4]
                    if (
                        translation < self._min_translation
                        and rotation < self._min_rotation
                        and distance_gain < self._min_translation
                    ):
                        enter_waiting = (self._task_id, self._leg_id)

            if self._state == "WAITING_FOR_PATH":
                if (
                    self._max_blocked_wait > 0.0
                    and now - self._waiting_since >= self._max_blocked_wait
                ):
                    self._state = "FAILED"
                    self._message = "blocked wait timeout"
                elif (
                    not self._probe_running
                    and now - self._last_plan_probe >= self._plan_check_period
                ):
                    self._probe_running = True
                    self._last_plan_probe = now
                    start_probe = True
                    probe_task_id = self._task_id

        if enter_waiting is not None:
            self._enter_waiting(*enter_waiting)
        if start_probe:
            threading.Thread(
                target=self._probe_path,
                args=(probe_task_id,),
                daemon=True,
            ).start()
        self._publish_state()

    def _pause_service(self, _request):
        with self._lock:
            if self._state not in ACTIVE_STATES or self._state == "PAUSED":
                return TriggerResponse(False, "no active navigation to pause")
            self._leg_id += 1
            self._state = "PAUSED"
            self._message = "paused by operator"
            self._progress.clear()
            self._move_base.cancel_goal()
        self._publish_state()
        return TriggerResponse(True, "navigation paused")

    def _resume_service(self, _request):
        with self._lock:
            if self._state != "PAUSED" or self._target_pose is None:
                return TriggerResponse(False, "navigation is not paused")
            task_id = self._task_id
            self._state = "PREPARING"
            self._message = "resuming"
        self._publish_state()
        threading.Thread(
            target=self._prepare_and_dispatch,
            args=(task_id, True),
            daemon=True,
        ).start()
        return TriggerResponse(True, "navigation resumed")

    def _cancel_service(self, _request):
        with self._lock:
            if self._state == "IDLE" or self._state in TERMINAL_STATES:
                return TriggerResponse(False, "no active navigation to cancel")
            self._task_id += 1
            self._leg_id += 1
            self._state = "CANCELED"
            self._message = "canceled by operator"
            self._progress.clear()
            self._move_base.cancel_goal()
            try:
                self._cmd_vel_pub.publish(Twist())
            except Exception:
                pass
        self._publish_state()
        return TriggerResponse(True, "navigation canceled")

    def _retry_service(self, _request):
        with self._lock:
            if self._target_pose is None:
                return TriggerResponse(False, "no waypoint is available for retry")
            self._leg_id += 1
            self._move_base.cancel_goal()
            task_id = self._task_id
            self._state = "PREPARING"
            self._message = "manual retry"
            self._retry_count = 0
            self._progress.clear()
        self._publish_state()
        threading.Thread(
            target=self._prepare_and_dispatch,
            args=(task_id, True),
            daemon=True,
        ).start()
        return TriggerResponse(True, "navigation retry started")

    def _home_service(self, _request):
        task_id, message = self._begin_task(
            self._home_waypoint, self._default_retries, "service"
        )
        return TriggerResponse(task_id is not None, message)

    def _clear_service_callback(self, _request):
        with self._lock:
            if self._state in ACTIVE_STATES:
                return TriggerResponse(
                    False, "refusing to clear costmaps during active navigation"
                )
        try:
            rospy.wait_for_service(self._clear_service, timeout=2.0)
            self._clear_costmaps()
            return TriggerResponse(True, "costmaps cleared while idle")
        except Exception as exc:
            return TriggerResponse(False, "clear_costmaps failed: %s" % exc)

    def _reload_service(self, _request):
        with self._lock:
            if self._state in ACTIVE_STATES:
                return TriggerResponse(
                    False, "refusing to reload waypoints during active navigation"
                )
        try:
            self._load_waypoints()
            self._publish_state()
            return TriggerResponse(True, "waypoints reloaded")
        except Exception as exc:
            return TriggerResponse(False, "reload failed: %s" % exc)

    def _goal_topic_callback(self, message):
        task_id, detail = self._begin_task(
            message.data.strip(), self._default_retries, "topic"
        )
        if task_id is None:
            rospy.logerr("guide_manager: %s", detail)

    def _execute_action(self, goal):
        task_id, detail = self._begin_task(
            goal.waypoint.strip(), int(goal.max_retries), "action"
        )
        if task_id is None:
            self._action_server.set_aborted(
                NavigateWaypointResult(False, "FAILED", detail), detail
            )
            return

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self._action_server.is_preempt_requested():
                with self._lock:
                    if task_id == self._task_id:
                        self._leg_id += 1
                        self._state = "CANCELED"
                        self._message = "action preempted"
                        self._move_base.cancel_goal()
                result = NavigateWaypointResult(False, "CANCELED", "action preempted")
                self._action_server.set_preempted(result, result.message)
                self._publish_state()
                return

            with self._lock:
                if task_id != self._task_id:
                    result = NavigateWaypointResult(
                        False, "CANCELED", "replaced by another navigation request"
                    )
                    self._action_server.set_preempted(result, result.message)
                    return
                state = self._state
                message = self._message
                distance = self._distance_remaining
                retries = self._retry_count

            feedback = NavigateWaypointFeedback()
            feedback.state = state
            feedback.distance_remaining = (
                distance if math.isfinite(distance) else -1.0
            )
            feedback.retry_count = retries
            self._action_server.publish_feedback(feedback)

            if state == "SUCCEEDED":
                result = NavigateWaypointResult(True, state, message)
                self._action_server.set_succeeded(result, message)
                return
            if state == "FAILED":
                result = NavigateWaypointResult(False, state, message)
                self._action_server.set_aborted(result, message)
                return
            if state == "CANCELED":
                result = NavigateWaypointResult(False, state, message)
                self._action_server.set_preempted(result, message)
                return
            rate.sleep()

    def _snapshot(self):
        with self._lock:
            distance = self._distance_remaining
            return {
                "state": self._state,
                "message": self._message,
                "waypoint": self._waypoint_name,
                "distance_remaining": (
                    round(distance, 3) if math.isfinite(distance) else None
                ),
                "retry_count": self._retry_count,
                "max_retries": self._max_retries,
            }

    def _publish_state(self):
        if not hasattr(self, "_state_pub"):
            return
        snapshot = self._snapshot()
        self._state_pub.publish(
            String(data=json.dumps(snapshot, ensure_ascii=False, sort_keys=True))
        )
        self._active_pub.publish(String(data=snapshot["waypoint"]))


def main():
    rospy.init_node("guide_manager")
    try:
        GuideManager()
    except Exception as exc:
        rospy.logfatal("guide_manager startup failed: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
