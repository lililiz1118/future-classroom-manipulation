#!/usr/bin/env python3
"""Maintain one stable RANSAC-derived tabletop in the MoveIt Planning Scene."""

import os
import sys


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKSPACE_SOURCE = os.path.abspath(os.path.join(PACKAGE_ROOT, "..", "..", ".."))
ANYGRASP_SOURCE = os.path.join(WORKSPACE_SOURCE, "anygrasp_ros", "src")
if ANYGRASP_SOURCE not in sys.path:
    sys.path.insert(0, ANYGRASP_SOURCE)

import moveit_commander
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import PlanningScene
from tf.transformations import quaternion_matrix

from anygrasp_ros.table_geometry import (
    plane_from_normal_and_center_height,
    table_surface_from_plane,
)
from tracer_bringup.table_collision import (
    StableTableTracker,
    TablePoseSample,
    build_collision_box,
)


TABLE_OBJECT_ID = "table_surface"


def build_planning_scene_message(object_id, frame_id, stamp, box):
    if str(object_id) != TABLE_OBJECT_ID:
        raise ValueError("table collision object id must remain table_surface")
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(box.center[0])
    pose.pose.position.y = float(box.center[1])
    pose.pose.position.z = float(box.center[2])
    pose.pose.orientation.x = float(box.quaternion[0])
    pose.pose.orientation.y = float(box.quaternion[1])
    pose.pose.orientation.z = float(box.quaternion[2])
    pose.pose.orientation.w = float(box.quaternion[3])

    collision = PlanningSceneInterface.make_box(object_id, pose, box.size)
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.world.collision_objects = [collision]
    return scene


def build_status_message(
    object_id, frame_id, stamp, state, age_sec, collision_present
):
    message = DiagnosticArray()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    entry = DiagnosticStatus()
    entry.name = "table_collision/%s" % object_id
    entry.hardware_id = object_id
    entry.message = state
    if state == "fresh":
        entry.level = DiagnosticStatus.OK
    elif state == "stale":
        entry.level = DiagnosticStatus.STALE
    else:
        entry.level = DiagnosticStatus.WARN
    age_text = "n/a" if age_sec is None else "%.6f" % float(age_sec)
    entry.values = [
        KeyValue("state", str(state)),
        KeyValue("model_age_sec", age_text),
        KeyValue("collision_present", "true" if collision_present else "false"),
    ]
    message.status = [entry]
    return message


class TableCollisionUpdater:
    def __init__(self):
        rospy.init_node("table_collision_updater", anonymous=False)
        if not bool(rospy.get_param("~table_collision_enabled", False)):
            raise RuntimeError("table collision updater must be explicitly enabled")

        workspace = rospy.get_param("~workspace")
        self._frame_id = str(workspace["frame_id"]).strip()
        self._roi_xy = (
            float(workspace["x_min"]),
            float(workspace["x_max"]),
            float(workspace["y_min"]),
            float(workspace["y_max"]),
        )
        self._object_id = str(rospy.get_param("~table_object_id")).strip()
        self._thickness = float(rospy.get_param("~table_collision_thickness"))
        self._xy_margin = float(rospy.get_param("~table_xy_margin"))
        self._input_topic = str(
            rospy.get_param(
                "~table_surface_pose_topic", "/yolo_world/table_surface_pose"
            )
        )
        self._stable_pose_topic = str(rospy.get_param("~stable_table_pose_topic"))
        self._status_topic = str(rospy.get_param("~table_status_topic"))
        status_period = float(rospy.get_param("~status_publish_period"))
        if not self._frame_id or not self._object_id:
            raise ValueError("table frame and object id must not be empty")
        if self._object_id != TABLE_OBJECT_ID:
            raise ValueError("table collision object id must remain table_surface")
        if status_period <= 0.0:
            raise ValueError("status_publish_period must be positive")

        self._tracker = StableTableTracker(
            expected_frame=self._frame_id,
            stable_plane_frames=int(rospy.get_param("~stable_plane_frames")),
            max_height_variation=float(rospy.get_param("~max_height_variation")),
            max_normal_angle_deg=float(rospy.get_param("~max_normal_angle_deg")),
            update_height_threshold=float(
                rospy.get_param("~update_height_threshold")
            ),
            update_angle_threshold_deg=float(
                rospy.get_param("~update_angle_threshold_deg")
            ),
            max_table_plane_age=float(rospy.get_param("~max_table_plane_age")),
        )

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.wait_for_service("/apply_planning_scene")
        self._scene = PlanningSceneInterface(synchronous=True)
        self._stable_pose_publisher = rospy.Publisher(
            self._stable_pose_topic, PoseStamped, queue_size=1, latch=True
        )
        self._status_publisher = rospy.Publisher(
            self._status_topic, DiagnosticArray, queue_size=1, latch=True
        )
        self._subscriber = rospy.Subscriber(
            self._input_topic, PoseStamped, self._pose_callback, queue_size=10
        )
        self._status_timer = rospy.Timer(
            rospy.Duration.from_sec(status_period), self._status_timer_callback
        )
        rospy.loginfo(
            "[table collision] enabled | input=%s frame=%s id=%s roi=%s "
            "thickness=%.3f margin=%.3f",
            self._input_topic,
            self._frame_id,
            self._object_id,
            self._roi_xy,
            self._thickness,
            self._xy_margin,
        )

    def _stable_pose(self, model):
        plane = plane_from_normal_and_center_height(
            model.normal, model.height_at_roi_center, self._roi_xy
        )
        surface = table_surface_from_plane(plane, self._roi_xy)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.from_sec(model.stamp_sec)
        pose.header.frame_id = model.frame_id
        pose.pose.position.x = float(surface.center[0])
        pose.pose.position.y = float(surface.center[1])
        pose.pose.position.z = float(surface.center[2])
        pose.pose.orientation.x = float(surface.quaternion[0])
        pose.pose.orientation.y = float(surface.quaternion[1])
        pose.pose.orientation.z = float(surface.quaternion[2])
        pose.pose.orientation.w = float(surface.quaternion[3])
        return pose

    def _pose_callback(self, message):
        quaternion = np.array(
            [
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ],
            dtype=np.float64,
        )
        quaternion_norm = float(np.linalg.norm(quaternion))
        if not np.isfinite(quaternion).all() or quaternion_norm <= np.finfo(float).eps:
            rospy.logwarn_throttle(5.0, "[table collision] invalid pose quaternion")
            return
        quaternion /= quaternion_norm
        normal = quaternion_matrix(quaternion)[:3, 2]
        sample = TablePoseSample(
            frame_id=message.header.frame_id,
            stamp_sec=message.header.stamp.to_sec(),
            normal=normal,
            height_at_roi_center=float(message.pose.position.z),
        )
        decision = self._tracker.observe(sample, rospy.Time.now().to_sec())
        if decision.reason in (
            "first_stable_model",
            "model_changed",
            "below_update_threshold",
        ):
            self._stable_pose_publisher.publish(
                self._stable_pose(decision.stable_model)
            )
        if not decision.update_scene:
            return

        model = decision.stable_model
        box = build_collision_box(
            model.normal,
            model.height_at_roi_center,
            self._roi_xy,
            self._thickness,
            self._xy_margin,
        )
        scene = build_planning_scene_message(
            self._object_id,
            self._frame_id,
            rospy.Time.from_sec(model.stamp_sec),
            box,
        )
        if not self._scene.apply_planning_scene(scene):
            rospy.logerr(
                "[table collision] ApplyPlanningScene rejected %s; keeping prior object",
                self._object_id,
            )
            return
        self._tracker.confirm_scene_update(model)
        rospy.loginfo(
            "[table collision] updated %s | normal=(%.6f %.6f %.6f) "
            "height=%.6f center=(%.6f %.6f %.6f) size=(%.6f %.6f %.6f)",
            self._object_id,
            model.normal[0],
            model.normal[1],
            model.normal[2],
            model.height_at_roi_center,
            box.center[0],
            box.center[1],
            box.center[2],
            box.size[0],
            box.size[1],
            box.size[2],
        )

    def _status_timer_callback(self, _event):
        now = rospy.Time.now()
        status = self._tracker.status(now.to_sec())
        self._status_publisher.publish(
            build_status_message(
                self._object_id,
                self._frame_id,
                now,
                status.state,
                status.age_sec,
                status.collision_present,
            )
        )


def main():
    TableCollisionUpdater()
    rospy.spin()


if __name__ == "__main__":
    main()
