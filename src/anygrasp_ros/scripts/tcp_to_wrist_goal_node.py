#!/usr/bin/env python3
"""Convert base-frame AG95 TCP goals into MoveIt's wrist_3_link goals.

This node is a read-only TF geometry bridge.  It has no MoveIt, IK, UR,
trajectory, or gripper-control interface and never publishes a target TF.
"""

import os
import sys
from dataclasses import dataclass

SYSTEM_DIST_PACKAGES = "/usr/lib/python3/dist-packages"
if SYSTEM_DIST_PACKAGES not in sys.path:
    sys.path.append(SYSTEM_DIST_PACKAGES)
PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE_SOURCE = os.path.join(PACKAGE_ROOT, "src")
if PACKAGE_SOURCE not in sys.path:
    sys.path.insert(0, PACKAGE_SOURCE)

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray

if SYSTEM_DIST_PACKAGES in sys.path:
    sys.path.remove(SYSTEM_DIST_PACKAGES)

from anygrasp_ros.core import (
    compute_wrist_goal_transform,
    reconstruct_tcp_goal_transform,
    rotation_matrix_to_quaternion,
)


BASE_FRAME = "base_link"
URBASE_FRAME = "ur_arm_base_link"
WRIST_FRAME = "ur_arm_wrist_3_link"
TCP_FRAME = "ag95_tcp"
_AXIS_COLORS = (
    ColorRGBA(1.0, 0.0, 0.0, 1.0),
    ColorRGBA(0.0, 1.0, 0.0, 1.0),
    ColorRGBA(0.0, 0.35, 1.0, 1.0),
)


@dataclass(frozen=True)
class GoalResult:
    header: Header
    base_from_tcp_goal: np.ndarray
    base_from_wrist_goal: np.ndarray
    base_from_tcp_reconstructed: np.ndarray


class TcpToWristGoalNode:
    """Bridge desired TCP poses to the wrist endpoint used by current MoveIt."""

    def __init__(self):
        rospy.init_node("tcp_to_wrist_goal_node")
        self._expected_input_frame = str(
            rospy.get_param("~input_frame", URBASE_FRAME)
        ).strip()
        self._output_frame = str(rospy.get_param("~output_frame", BASE_FRAME)).strip()
        self._wrist_frame = str(rospy.get_param("~wrist_frame", WRIST_FRAME)).strip()
        self._tcp_frame = str(rospy.get_param("~tcp_frame", TCP_FRAME)).strip()
        if not all(
            (
                self._expected_input_frame,
                self._output_frame,
                self._wrist_frame,
                self._tcp_frame,
            )
        ):
            raise ValueError("input, output, wrist, and tcp frames must not be empty")
        self._tf_timeout = rospy.Duration.from_sec(
            float(rospy.get_param("~tf_timeout", 0.2))
        )
        if self._tf_timeout.to_sec() <= 0.0:
            raise ValueError("tf_timeout must be positive")
        grasp_input_topic = rospy.get_param(
            "~grasp_input_topic", "/anygrasp/best_grasp_tcp"
        )
        pregrasp_input_topic = rospy.get_param(
            "~pregrasp_input_topic", "/anygrasp/pre_grasp_tcp"
        )
        grasp_output_topic = rospy.get_param(
            "~grasp_output_topic", "/anygrasp/grasp_wrist_goal"
        )
        pregrasp_output_topic = rospy.get_param(
            "~pregrasp_output_topic", "/anygrasp/pre_grasp_wrist_goal"
        )
        marker_topic = rospy.get_param(
            "~marker_topic", "/anygrasp/tcp_wrist_goal_markers"
        )
        self._grasp_publisher = rospy.Publisher(
            grasp_output_topic, PoseStamped, queue_size=1
        )
        self._pregrasp_publisher = rospy.Publisher(
            pregrasp_output_topic, PoseStamped, queue_size=1
        )
        self._marker_publisher = rospy.Publisher(marker_topic, MarkerArray, queue_size=1)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._marker_lifetime = rospy.Duration(0)
        self._latest_results = {}
        self._grasp_subscriber = rospy.Subscriber(
            grasp_input_topic, PoseStamped, self._handle_grasp, queue_size=1
        )
        self._pregrasp_subscriber = rospy.Subscriber(
            pregrasp_input_topic, PoseStamped, self._handle_pregrasp, queue_size=1
        )
        rospy.loginfo(
            "[TCP wrist goal] %s/%s -> %s/%s using TF %s <- %s and %s <- %s",
            grasp_input_topic,
            pregrasp_input_topic,
            grasp_output_topic,
            pregrasp_output_topic,
            self._output_frame,
            self._expected_input_frame,
            self._wrist_frame,
            self._tcp_frame,
        )

    @staticmethod
    def _point(values):
        return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))

    @staticmethod
    def _normalized_quaternion(quaternion):
        values = np.asarray(quaternion, dtype=np.float64)
        norm = np.linalg.norm(values)
        if values.shape != (4,) or not np.isfinite(values).all() or norm <= np.finfo(np.float64).eps:
            raise ValueError("orientation must be finite and non-zero")
        return values / norm

    @classmethod
    def _transform_matrix(cls, transform):
        quaternion = cls._normalized_quaternion(
            [
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ]
        )
        matrix = quaternion_matrix(quaternion)
        translation = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z],
            dtype=np.float64,
        )
        if not np.isfinite(translation).all():
            raise ValueError("transform translation must be finite")
        matrix[:3, 3] = translation
        return matrix

    @classmethod
    def _pose_matrix(cls, pose):
        quaternion = cls._normalized_quaternion(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        matrix = quaternion_matrix(quaternion)
        translation = np.array(
            [pose.position.x, pose.position.y, pose.position.z], dtype=np.float64
        )
        if not np.isfinite(translation).all():
            raise ValueError("goal translation must be finite")
        matrix[:3, 3] = translation
        return matrix

    def _axis_marker(self, header, namespace, marker_id, transform, length):
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [
            self._point(transform[:3, 3]),
            self._point(transform[:3, 3] + transform[:3, marker_id % 3] * length),
        ]
        marker.scale.x = 0.004
        marker.scale.y = 0.008
        marker.scale.z = 0.012
        marker.color = _AXIS_COLORS[marker_id % 3]
        marker.lifetime = self._marker_lifetime
        return marker

    def _publish_markers(self):
        markers = [Marker()]
        markers[0].action = Marker.DELETEALL
        marker_id = 0
        for kind, result in sorted(self._latest_results.items()):
            for name, transform, length in (
                ("tcp_goal", result.base_from_tcp_goal, 0.08),
                ("wrist_goal", result.base_from_wrist_goal, 0.065),
                ("tcp_reconstructed", result.base_from_tcp_reconstructed, 0.045),
            ):
                for _ in range(3):
                    markers.append(
                        self._axis_marker(
                            result.header,
                            kind + "_" + name + "_axes",
                            marker_id,
                            transform,
                            length,
                        )
                    )
                    marker_id += 1
        self._marker_publisher.publish(MarkerArray(markers=markers))

    def _handle_grasp(self, message):
        self._handle_goal(message, "grasp")

    def _handle_pregrasp(self, message):
        self._handle_goal(message, "pregrasp")

    def _handle_goal(self, message, kind):
        if message.header.frame_id != self._expected_input_frame:
            rospy.logwarn(
                "[TCP wrist goal] dropping %s in frame %r; expected %r",
                kind,
                message.header.frame_id,
                self._expected_input_frame,
            )
            return
        try:
            # tf2 lookup(target, source, ...) returns target_from_source.
            base_from_urbase_msg = self._tf_buffer.lookup_transform(
                self._output_frame,
                self._expected_input_frame,
                message.header.stamp,
                self._tf_timeout,
            )
            wrist_from_tcp_msg = self._tf_buffer.lookup_transform(
                self._wrist_frame,
                self._tcp_frame,
                message.header.stamp,
                self._tf_timeout,
            )
            base_from_urbase = self._transform_matrix(base_from_urbase_msg.transform)
            urbase_from_tcp_goal = self._pose_matrix(message.pose)
            wrist_from_tcp = self._transform_matrix(wrist_from_tcp_msg.transform)
            base_from_wrist_goal = compute_wrist_goal_transform(
                base_from_urbase, urbase_from_tcp_goal, wrist_from_tcp
            )
            base_from_tcp_goal = base_from_urbase @ urbase_from_tcp_goal
            base_from_tcp_reconstructed = reconstruct_tcp_goal_transform(
                base_from_wrist_goal, wrist_from_tcp
            )
            quaternion = rotation_matrix_to_quaternion(base_from_wrist_goal[:3, :3])
        except (tf2_ros.TransformException, ValueError, np.linalg.LinAlgError) as error:
            rospy.logwarn("[TCP wrist goal] dropping %s: %s", kind, error)
            return

        output = PoseStamped()
        output.header = Header(stamp=message.header.stamp, frame_id=self._output_frame)
        output.pose.position = self._point(base_from_wrist_goal[:3, 3])
        output.pose.orientation.x = float(quaternion[0])
        output.pose.orientation.y = float(quaternion[1])
        output.pose.orientation.z = float(quaternion[2])
        output.pose.orientation.w = float(quaternion[3])
        if kind == "grasp":
            self._grasp_publisher.publish(output)
        else:
            self._pregrasp_publisher.publish(output)
        self._latest_results[kind] = GoalResult(
            header=output.header,
            base_from_tcp_goal=base_from_tcp_goal,
            base_from_wrist_goal=base_from_wrist_goal,
            base_from_tcp_reconstructed=base_from_tcp_reconstructed,
        )
        self._publish_markers()


if __name__ == "__main__":
    TcpToWristGoalNode()
    rospy.spin()
