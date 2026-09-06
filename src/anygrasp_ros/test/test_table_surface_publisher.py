#!/home/jt001/.conda/envs/anygrasp/bin/python
import importlib.util
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


for path in (
    "/usr/lib/python3/dist-packages",
    "/opt/ros/noetic/lib/python3/dist-packages",
):
    if path not in sys.path:
        sys.path.append(path)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))
NODE_PATH = PACKAGE_ROOT / "scripts" / "table_surface_publisher_node.py"


def load_node_module():
    spec = importlib.util.spec_from_file_location(
        "table_surface_publisher_node", NODE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class TableSurfacePublisherTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_node_module()

    def make_node(self):
        node = self.module.TableSurfacePublisherNode.__new__(
            self.module.TableSurfacePublisherNode
        )
        node._workspace_frame = "ur_arm_base_link"
        node._publisher = RecordingPublisher()
        return node

    def test_node_disables_cuda_before_loading_open3d_preprocessing(self):
        self.assertEqual(os.environ.get("CUDA_VISIBLE_DEVICES"), "-1")

    def test_module_does_not_load_neural_or_yolo_stack(self):
        self.assertNotIn("torch", sys.modules)
        self.assertNotIn("yolo_world_ros", sys.modules)

    @staticmethod
    def valid_result(module, frame_id="ur_arm_base_link"):
        geometry = module.TableSurfaceGeometry(
            plane_model=np.array([0.0, 0.0, 1.0, -0.24]),
            normal=np.array([0.0, 0.0, 1.0]),
            center=np.array([0.0, 0.4, 0.24]),
            corners=np.array(
                [
                    [-0.1, 0.3, 0.24],
                    [-0.1, 0.5, 0.24],
                    [0.1, 0.3, 0.24],
                    [0.1, 0.5, 0.24],
                ]
            ),
            rotation=np.eye(3),
            quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
            frame_id=frame_id,
        )
        return SimpleNamespace(plane_valid=True, table_geometry=geometry)

    def test_valid_geometry_publishes_neutral_pose_in_workspace_frame(self):
        node = self.make_node()
        stamp = self.module.rospy.Time.from_sec(12.5)

        accepted = node._publish_plane_result(self.valid_result(self.module), stamp)

        self.assertTrue(accepted)
        self.assertEqual(len(node._publisher.messages), 1)
        pose = node._publisher.messages[0]
        self.assertEqual(pose.header.frame_id, "ur_arm_base_link")
        self.assertEqual(pose.header.stamp, stamp)
        self.assertAlmostEqual(pose.pose.position.z, 0.24)
        self.assertAlmostEqual(pose.pose.orientation.w, 1.0)

    def test_invalid_plane_does_not_replace_last_valid_table(self):
        node = self.make_node()

        accepted = node._publish_plane_result(
            SimpleNamespace(plane_valid=False, table_geometry=None),
            self.module.rospy.Time.from_sec(13.0),
        )

        self.assertFalse(accepted)
        self.assertEqual(node._publisher.messages, [])

    def test_geometry_from_another_frame_is_rejected(self):
        node = self.make_node()
        result = self.valid_result(self.module, "camera_frame")

        with self.assertRaisesRegex(ValueError, "ur_arm_base_link"):
            node._publish_plane_result(result, self.module.rospy.Time.from_sec(14.0))


if __name__ == "__main__":
    unittest.main()
