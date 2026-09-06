import importlib.util
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np


ROS_PYTHON = "/opt/ros/noetic/lib/python3/dist-packages"
if ROS_PYTHON not in sys.path:
    sys.path.append(ROS_PYTHON)
SYSTEM_PYTHON = "/usr/lib/python3/dist-packages"
if SYSTEM_PYTHON not in sys.path:
    sys.path.append(SYSTEM_PYTHON)

import rospy  # noqa: E402
import tf2_ros  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from tf.transformations import quaternion_from_matrix  # noqa: E402


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = PACKAGE_ROOT / "src"
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))
NODE_PATH = PACKAGE_ROOT / "scripts" / "tcp_to_wrist_goal_node.py"
SPEC = importlib.util.spec_from_file_location("tcp_to_wrist_goal_node_under_test", NODE_PATH)
NODE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NODE)


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeTfBuffer:
    def __init__(self, transforms=None, error=None):
        self.transforms = transforms or {}
        self.error = error
        self.calls = []

    def lookup_transform(self, target_frame, source_frame, stamp, timeout):
        self.calls.append((target_frame, source_frame, stamp, timeout))
        if self.error is not None:
            raise self.error
        return self.transforms[(target_frame, source_frame)]


def transform(parent, child, matrix):
    message = TransformStamped()
    message.header = Header(frame_id=parent)
    message.child_frame_id = child
    message.transform.translation.x = matrix[0, 3]
    message.transform.translation.y = matrix[1, 3]
    message.transform.translation.z = matrix[2, 3]
    quaternion = quaternion_from_matrix(matrix)
    message.transform.rotation.x = quaternion[0]
    message.transform.rotation.y = quaternion[1]
    message.transform.rotation.z = quaternion[2]
    message.transform.rotation.w = quaternion[3]
    return message


def make_input():
    message = PoseStamped()
    message.header = Header(stamp=rospy.Time(123, 456), frame_id="ur_arm_base_link")
    message.pose.position.x = 0.4
    message.pose.position.y = 0.5
    message.pose.position.z = 0.6
    message.pose.orientation.x = math.sqrt(0.5)
    message.pose.orientation.w = math.sqrt(0.5)
    return message


def make_node(error=None):
    base_from_urbase = np.array(
        [[0.0, -1.0, 0.0, 1.0], [1.0, 0.0, 0.0, 2.0], [0.0, 0.0, 1.0, 3.0], [0.0, 0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    wrist_from_tcp = np.array(
        [[0.0, 0.0, 1.0, 0.1], [0.0, 1.0, 0.0, 0.2], [-1.0, 0.0, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    node = NODE.TcpToWristGoalNode.__new__(NODE.TcpToWristGoalNode)
    node._expected_input_frame = "ur_arm_base_link"
    node._output_frame = "base_link"
    node._wrist_frame = "ur_arm_wrist_3_link"
    node._tcp_frame = "ag95_tcp"
    node._tf_timeout = rospy.Duration.from_sec(0.2)
    node._tf_buffer = FakeTfBuffer(
        {
            ("base_link", "ur_arm_base_link"): transform("base_link", "ur_arm_base_link", base_from_urbase),
            ("ur_arm_wrist_3_link", "ag95_tcp"): transform("ur_arm_wrist_3_link", "ag95_tcp", wrist_from_tcp),
        },
        error=error,
    )
    node._grasp_publisher = CapturePublisher()
    node._pregrasp_publisher = CapturePublisher()
    node._marker_publisher = CapturePublisher()
    node._marker_lifetime = rospy.Duration(0)
    node._latest_results = {}
    return node


class TcpToWristGoalNodeTest(unittest.TestCase):
    def test_grasp_goal_uses_target_source_tf_order_and_outputs_hand_derived_wrist_pose(self):
        node = make_node()
        incoming = make_input()

        node._handle_goal(incoming, "grasp")

        self.assertEqual(
            [(call[0], call[1]) for call in node._tf_buffer.calls],
            [
                ("base_link", "ur_arm_base_link"),
                ("ur_arm_wrist_3_link", "ag95_tcp"),
            ],
        )
        self.assertTrue(all(call[2] == incoming.header.stamp for call in node._tf_buffer.calls))
        self.assertEqual(len(node._grasp_publisher.messages), 1)
        output = node._grasp_publisher.messages[0]
        self.assertEqual(output.header.frame_id, "base_link")
        self.assertEqual(output.header.stamp, incoming.header.stamp)
        np.testing.assert_allclose(
            [output.pose.position.x, output.pose.position.y, output.pose.position.z],
            [0.4, 2.7, 3.4],
            atol=1e-7,
        )
        np.testing.assert_allclose(
            [output.pose.orientation.x, output.pose.orientation.y, output.pose.orientation.z, output.pose.orientation.w],
            [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],
            atol=1e-7,
        )

    def test_pregrasp_goal_publishes_to_its_distinct_topic(self):
        node = make_node()

        node._handle_goal(make_input(), "pregrasp")

        self.assertEqual(node._grasp_publisher.messages, [])
        self.assertEqual(len(node._pregrasp_publisher.messages), 1)
        self.assertEqual(node._pregrasp_publisher.messages[0].header.frame_id, "base_link")

    def test_wrong_input_frame_warns_and_drops_without_tf_or_output(self):
        node = make_node()
        incoming = make_input()
        incoming.header.frame_id = "camera_color_optical_frame"

        with patch.object(NODE.rospy, "logwarn") as warning:
            node._handle_goal(incoming, "grasp")

        warning.assert_called_once()
        self.assertEqual(node._tf_buffer.calls, [])
        self.assertEqual(node._grasp_publisher.messages, [])

    def test_tf_lookup_failure_warns_and_drops_without_output(self):
        node = make_node(error=tf2_ros.LookupException("missing transform"))

        with patch.object(NODE.rospy, "logwarn") as warning:
            node._handle_goal(make_input(), "grasp")

        warning.assert_called_once()
        self.assertEqual(node._grasp_publisher.messages, [])


if __name__ == "__main__":
    unittest.main()
