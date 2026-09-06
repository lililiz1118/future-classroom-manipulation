import importlib.util
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


ROS_PYTHON = "/opt/ros/noetic/lib/python3/dist-packages"
if ROS_PYTHON not in sys.path:
    sys.path.append(ROS_PYTHON)
SYSTEM_PYTHON = "/usr/lib/python3/dist-packages"
if SYSTEM_PYTHON not in sys.path:
    sys.path.append(SYSTEM_PYTHON)

import rospy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = PACKAGE_ROOT / "src"
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))
NODE_PATH = PACKAGE_ROOT / "scripts" / "best_grasp_tcp_node.py"
SPEC = importlib.util.spec_from_file_location("best_grasp_tcp_node_under_test", NODE_PATH)
NODE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NODE)


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_node():
    node = NODE.BestGraspTcpNode.__new__(NODE.BestGraspTcpNode)
    node._expected_base_frame = "ur_arm_base_link"
    node._axis_length = 0.08
    node._pose_publisher = CapturePublisher()
    node._marker_publisher = CapturePublisher()
    node._marker_lifetime = rospy.Duration(0)
    return node


def make_pose(frame_id="ur_arm_base_link"):
    pose = PoseStamped()
    pose.header = Header(stamp=rospy.Time(123, 456), frame_id=frame_id)
    pose.pose.position.x = 0.123
    pose.pose.position.y = -0.456
    pose.pose.position.z = 0.789
    pose.pose.orientation.w = 1.0
    return pose


class BestGraspTcpNodeTest(unittest.TestCase):
    def test_valid_base_grasp_preserves_center_and_applies_only_fixed_orientation_mapping(self):
        node = make_node()
        incoming = make_pose()

        node._handle_best_grasp_base(incoming)

        self.assertEqual(len(node._pose_publisher.messages), 1)
        output = node._pose_publisher.messages[0]
        self.assertEqual(output.header.frame_id, "ur_arm_base_link")
        self.assertEqual(output.header.stamp, incoming.header.stamp)
        self.assertEqual(
            (output.pose.position.x, output.pose.position.y, output.pose.position.z),
            (incoming.pose.position.x, incoming.pose.position.y, incoming.pose.position.z),
        )
        np.testing.assert_allclose(
            [
                output.pose.orientation.x,
                output.pose.orientation.y,
                output.pose.orientation.z,
                output.pose.orientation.w,
            ],
            [0.5, 0.5, 0.5, 0.5],
            atol=1e-7,
        )
        self.assertEqual(len(node._marker_publisher.messages), 1)
        markers = node._marker_publisher.messages[0].markers
        self.assertEqual(len(markers), 7)
        self.assertEqual(markers[0].action, Marker.DELETEALL)
        self.assertEqual(
            [marker.ns for marker in markers[1:]],
            ["anygrasp_grasp_axes"] * 3 + ["ag95_tcp_target_axes"] * 3,
        )

    def test_wrong_input_frame_warns_and_is_dropped_without_publication(self):
        node = make_node()

        with patch.object(NODE.rospy, "logwarn") as warning:
            node._handle_best_grasp_base(make_pose(frame_id="camera_color_optical_frame"))

        warning.assert_called_once()
        self.assertEqual(node._pose_publisher.messages, [])
        self.assertEqual(node._marker_publisher.messages, [])

    def test_zero_length_input_orientation_warns_and_is_dropped(self):
        node = make_node()
        incoming = make_pose()
        incoming.pose.orientation.w = 0.0

        with patch.object(NODE.rospy, "logwarn") as warning:
            node._handle_best_grasp_base(incoming)

        warning.assert_called_once()
        self.assertEqual(node._pose_publisher.messages, [])
        self.assertEqual(node._marker_publisher.messages, [])


if __name__ == "__main__":
    unittest.main()
