#!/usr/bin/env python3
"""Convert a pure base-frame AnyGrasp pose to the AG95 task-TCP orientation.

This debug-only node publishes no TF and contains no motion, IK, MoveIt, UR,
or gripper-control code.  ``ag95_tcp`` remains the physical TCP defined by the
robot URDF; this node merely publishes the target pose for later planning.
"""

import os
import sys

# Match the perception node's ROS/Conda import boundary: ROS Noetic's Python
# modules live in the system dist-packages directory, while this script must
# also be importable directly from its source path before a catkin install.
SYSTEM_DIST_PACKAGES = "/usr/lib/python3/dist-packages"
if SYSTEM_DIST_PACKAGES not in sys.path:
    sys.path.append(SYSTEM_DIST_PACKAGES)

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE_SOURCE = os.path.join(PACKAGE_ROOT, "src")
if PACKAGE_SOURCE not in sys.path:
    sys.path.insert(0, PACKAGE_SOURCE)

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray

if SYSTEM_DIST_PACKAGES in sys.path:
    sys.path.remove(SYSTEM_DIST_PACKAGES)

from anygrasp_ros.core import (
    ag95_tcp_rotation_from_grasp,
    compute_pregrasp_position,
    rotation_matrix_to_quaternion,
)


DEFAULT_BASE_FRAME = "ur_arm_base_link"
DEFAULT_INPUT_TOPIC = "/anygrasp/best_grasp_base"
DEFAULT_OUTPUT_TOPIC = "/anygrasp/best_grasp_tcp"
DEFAULT_MARKER_TOPIC = "/anygrasp/best_grasp_tcp_markers"
DEFAULT_PREGRASP_TOPIC = "/anygrasp/pre_grasp_tcp"
DEFAULT_PREGRASP_MARKER_TOPIC = "/anygrasp/pre_grasp_markers"
_AXIS_COLORS = (
    ColorRGBA(1.0, 0.0, 0.0, 1.0),
    ColorRGBA(0.0, 1.0, 0.0, 1.0),
    ColorRGBA(0.0, 0.35, 1.0, 1.0),
)


class BestGraspTcpNode:
    """Publish the orientation-only AnyGrasp G -> AG95 TCP target conversion."""

    def __init__(self):
        rospy.init_node("best_grasp_tcp_node")
        self._expected_base_frame = str(
            rospy.get_param("~base_frame", DEFAULT_BASE_FRAME)
        ).strip()
        if not self._expected_base_frame:
            raise ValueError("base_frame must not be empty")
        self._axis_length = float(rospy.get_param("~axis_length", 0.08))
        if not np.isfinite(self._axis_length) or self._axis_length <= 0.0:
            raise ValueError("axis_length must be finite and positive")
        self._pregrasp_distance = float(
            rospy.get_param("~pregrasp_distance", 0.08)
        )
        if (
            not np.isfinite(self._pregrasp_distance)
            or self._pregrasp_distance <= 0.0
        ):
            raise ValueError("pregrasp_distance must be finite and positive")
        self._marker_lifetime = rospy.Duration(0)
        input_topic = rospy.get_param("~input_topic", DEFAULT_INPUT_TOPIC)
        output_topic = rospy.get_param("~output_topic", DEFAULT_OUTPUT_TOPIC)
        marker_topic = rospy.get_param("~marker_topic", DEFAULT_MARKER_TOPIC)
        pregrasp_topic = rospy.get_param("~pregrasp_topic", DEFAULT_PREGRASP_TOPIC)
        pregrasp_marker_topic = rospy.get_param(
            "~pregrasp_marker_topic", DEFAULT_PREGRASP_MARKER_TOPIC
        )
        self._pose_publisher = rospy.Publisher(output_topic, PoseStamped, queue_size=1)
        self._marker_publisher = rospy.Publisher(
            marker_topic, MarkerArray, queue_size=1
        )
        self._pregrasp_publisher = rospy.Publisher(
            pregrasp_topic, PoseStamped, queue_size=1
        )
        self._pregrasp_marker_publisher = rospy.Publisher(
            pregrasp_marker_topic, MarkerArray, queue_size=1
        )
        self._subscriber = rospy.Subscriber(
            input_topic, PoseStamped, self._handle_best_grasp_base, queue_size=1
        )
        rospy.loginfo(
            "[AnyGrasp TCP] %s -> %s -> %s | required frame: %s | pregrasp retreat: %.3f m",
            input_topic,
            output_topic,
            pregrasp_topic,
            self._expected_base_frame,
            self._pregrasp_distance,
        )

    @staticmethod
    def _point(values):
        return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))

    @staticmethod
    def _normalized_orientation(pose):
        quaternion = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=np.float64,
        )
        norm = np.linalg.norm(quaternion)
        if not np.isfinite(quaternion).all() or norm <= np.finfo(np.float64).eps:
            raise ValueError("input orientation must be finite and non-zero")
        return quaternion / norm

    def _axis_marker(self, header, namespace, marker_id, center, axis, length):
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [self._point(center), self._point(center + axis * length)]
        marker.scale.x = 0.004
        marker.scale.y = 0.008
        marker.scale.z = 0.012
        marker.color = _AXIS_COLORS[marker_id % 3]
        marker.lifetime = self._marker_lifetime
        return marker

    def _debug_markers(self, header, center, base_from_grasp, base_from_tcp):
        markers = [Marker()]
        markers[0].header = header
        markers[0].action = Marker.DELETEALL
        for index in range(3):
            markers.append(
                self._axis_marker(
                    header,
                    "anygrasp_grasp_axes",
                    index,
                    center,
                    base_from_grasp[:, index],
                    self._axis_length,
                )
            )
        for index in range(3):
            markers.append(
                self._axis_marker(
                    header,
                    "ag95_tcp_target_axes",
                    index + 3,
                    center,
                    base_from_tcp[:, index],
                    self._axis_length * 0.72,
                )
            )
        return MarkerArray(markers=markers)

    def _pregrasp_debug_markers(
        self, header, pregrasp_center, grasp_center, base_from_tcp
    ):
        markers = [Marker()]
        markers[0].header = header
        markers[0].action = Marker.DELETEALL
        for index in range(3):
            markers.append(
                self._axis_marker(
                    header,
                    "pregrasp_tcp_axes",
                    index,
                    pregrasp_center,
                    base_from_tcp[:, index],
                    self._axis_length,
                )
            )
        for index in range(3):
            markers.append(
                self._axis_marker(
                    header,
                    "grasp_tcp_axes",
                    index + 3,
                    grasp_center,
                    base_from_tcp[:, index],
                    self._axis_length * 0.72,
                )
            )
        approach = Marker()
        approach.header = header
        approach.ns = "pregrasp_approach"
        approach.id = 6
        approach.type = Marker.ARROW
        approach.action = Marker.ADD
        approach.pose.orientation.w = 1.0
        approach.points = [self._point(pregrasp_center), self._point(grasp_center)]
        approach.scale.x = 0.006
        approach.scale.y = 0.012
        approach.scale.z = 0.018
        approach.color = ColorRGBA(1.0, 1.0, 0.1, 1.0)
        approach.lifetime = self._marker_lifetime
        markers.append(approach)
        return MarkerArray(markers=markers)

    def _handle_best_grasp_base(self, message):
        if message.header.frame_id != self._expected_base_frame:
            rospy.logwarn(
                "[AnyGrasp TCP] dropping grasp in frame %r; expected %r",
                message.header.frame_id,
                self._expected_base_frame,
            )
            return
        center = np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=np.float64,
        )
        if not np.isfinite(center).all():
            rospy.logwarn("[AnyGrasp TCP] dropping grasp with non-finite position")
            return
        try:
            base_from_grasp = quaternion_matrix(
                self._normalized_orientation(message.pose)
            )[:3, :3]
            base_from_tcp = ag95_tcp_rotation_from_grasp(base_from_grasp)
            tcp_quaternion = rotation_matrix_to_quaternion(base_from_tcp)
        except ValueError as error:
            rospy.logwarn("[AnyGrasp TCP] dropping invalid grasp orientation: %s", error)
            return

        target = PoseStamped()
        target.header = Header(
            stamp=message.header.stamp, frame_id=message.header.frame_id
        )
        # The goal is the same grasp center.  Do not apply the 0.175 m URDF TCP
        # offset here: it is the robot model's gripper_base_link -> ag95_tcp TF.
        target.pose.position.x = message.pose.position.x
        target.pose.position.y = message.pose.position.y
        target.pose.position.z = message.pose.position.z
        target.pose.orientation.x = float(tcp_quaternion[0])
        target.pose.orientation.y = float(tcp_quaternion[1])
        target.pose.orientation.z = float(tcp_quaternion[2])
        target.pose.orientation.w = float(tcp_quaternion[3])
        self._pose_publisher.publish(target)
        # Reconstruct the matrix from the final published TCP orientation, so
        # the local +Z used for retreat is exactly the published target's +Z.
        base_from_final_tcp = quaternion_matrix(tcp_quaternion)[:3, :3]
        self._marker_publisher.publish(
            self._debug_markers(
                target.header, center, base_from_grasp, base_from_final_tcp
            )
        )
        try:
            pregrasp_position = compute_pregrasp_position(
                center, base_from_final_tcp, self._pregrasp_distance
            )
        except ValueError as error:
            rospy.logwarn("[AnyGrasp TCP] dropping invalid pregrasp: %s", error)
            return
        pregrasp = PoseStamped()
        pregrasp.header = Header(
            stamp=target.header.stamp, frame_id=target.header.frame_id
        )
        pregrasp.pose.position = self._point(pregrasp_position)
        pregrasp.pose.orientation.x = target.pose.orientation.x
        pregrasp.pose.orientation.y = target.pose.orientation.y
        pregrasp.pose.orientation.z = target.pose.orientation.z
        pregrasp.pose.orientation.w = target.pose.orientation.w
        self._pregrasp_publisher.publish(pregrasp)
        self._pregrasp_marker_publisher.publish(
            self._pregrasp_debug_markers(
                target.header, pregrasp_position, center, base_from_final_tcp
            )
        )


if __name__ == "__main__":
    BestGraspTcpNode()
    rospy.spin()
