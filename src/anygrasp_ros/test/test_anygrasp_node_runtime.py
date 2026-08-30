import importlib.util
import math
import subprocess
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np


ROS_PYTHON = "/opt/ros/noetic/lib/python3/dist-packages"
if ROS_PYTHON not in sys.path:
    sys.path.append(ROS_PYTHON)
SYSTEM_PYTHON = "/usr/lib/python3/dist-packages"
if SYSTEM_PYTHON not in sys.path:
    sys.path.append(SYSTEM_PYTHON)

import rospy  # noqa: E402
from sensor_msgs import point_cloud2  # noqa: E402
from sensor_msgs.msg import PointField  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "scripts" / "anygrasp_d405_node.py"
SPEC = importlib.util.spec_from_file_location("anygrasp_d405_node_under_test", NODE_PATH)
NODE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NODE)


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeGrasp:
    def __init__(self, score):
        self.score = float(score)
        self.translation = np.array([score, 0.02, 0.5], dtype=np.float64)
        self.rotation_matrix = np.eye(3, dtype=np.float64)
        self.width = 0.04
        self.depth = 0.03


class FakeGraspGroup:
    def __init__(self, scores, tracker=None):
        self.items = [item if isinstance(item, FakeGrasp) else FakeGrasp(item) for item in scores]
        self.tracker = tracker if tracker is not None else {"nms": 0, "sort": 0}

    def __len__(self):
        return len(self.items)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return FakeGraspGroup(self.items[index], self.tracker)
        return self.items[index]

    def nms(self):
        self.tracker["nms"] += 1
        return self

    def sort_by_score(self):
        self.tracker["sort"] += 1
        self.items.sort(key=lambda grasp: grasp.score, reverse=True)
        return self


class FakeAdapter:
    def __init__(self, scores):
        self.group = FakeGraspGroup(scores)
        self.calls = []

    def infer(self, points, colors, bounds):
        self.calls.append((points.copy(), colors.copy(), tuple(bounds)))
        return self.group


def make_node(scores=(0.2, 0.9, 0.5)):
    node = NODE.AnyGraspD405Node.__new__(NODE.AnyGraspD405Node)
    node._workspace_bounds = (-0.5, 0.5, -0.5, 0.5, 0.1, 1.5)
    node._min_workspace_points = 1
    node._top_n = 2
    node._publish_input = True
    node._inference_rate = 1.0
    node._marker_lifetime = rospy.Duration.from_sec(2.5)
    node._input_publisher = CapturePublisher()
    node._marker_publisher = CapturePublisher()
    node._best_publisher = CapturePublisher()
    node._adapter = FakeAdapter(scores)
    return node


class LatestMessageBufferTest(unittest.TestCase):
    def test_take_latest_drops_intermediate_messages(self):
        buffer = NODE.LatestMessageBuffer()
        first = object()
        second = object()
        buffer.update(first)
        buffer.update(second)

        message, sequence = buffer.take_latest(0)
        repeated, repeated_sequence = buffer.take_latest(sequence)

        self.assertIs(message, second)
        self.assertEqual(sequence, 2)
        self.assertIsNone(repeated)
        self.assertEqual(repeated_sequence, 2)


class PointCloudConversionTest(unittest.TestCase):
    def test_cloud_conversion_preserves_xyz_and_decodes_packed_rgb(self):
        header = Header(frame_id="d405_depth_optical_frame")
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 16, PointField.FLOAT32, 1),
        ]
        packed = np.array([0x00FF0000, 0x000000FF], dtype=np.uint32).view(np.float32)
        cloud = point_cloud2.create_cloud(
            header,
            fields,
            [(0.1, 0.2, 0.3, float(packed[0])), (-0.1, 0.0, 0.8, float(packed[1]))],
        )

        points, colors = NODE.AnyGraspD405Node._cloud_to_arrays(cloud)

        np.testing.assert_allclose(points, [[0.1, 0.2, 0.3], [-0.1, 0.0, 0.8]])
        np.testing.assert_allclose(colors, [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


class ProcessCloudTest(unittest.TestCase):
    def test_process_cloud_runs_nms_sort_top_n_and_preserves_header(self):
        node = make_node()
        header = Header(stamp=rospy.Time(123, 456), frame_id="d405_depth_optical_frame")
        message = SimpleNamespace(header=header)
        points = np.array([[0.0, 0.0, 0.5], [0.1, 0.0, 0.6]], dtype=np.float32)
        colors = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
        node._cloud_to_arrays = lambda unused: (points, colors)

        with patch.object(NODE.rospy, "logwarn_throttle"):
            node._process_cloud(message)

        self.assertEqual(len(node._adapter.calls), 1)
        np.testing.assert_allclose(node._adapter.calls[0][0], points)
        self.assertEqual(node._adapter.calls[0][2], node._workspace_bounds)
        self.assertEqual(node._adapter.group.tracker, {"nms": 1, "sort": 1})
        self.assertEqual(len(node._input_publisher.messages), 1)
        self.assertEqual(node._input_publisher.messages[0].header.frame_id, header.frame_id)
        self.assertEqual(node._input_publisher.messages[0].width, 2)
        self.assertEqual(len(node._best_publisher.messages), 1)
        pose = node._best_publisher.messages[0]
        self.assertEqual(pose.header.frame_id, header.frame_id)
        self.assertEqual(pose.header.stamp, header.stamp)
        self.assertAlmostEqual(pose.pose.position.x, 0.9)
        quaternion = pose.pose.orientation
        norm = math.sqrt(
            quaternion.x**2 + quaternion.y**2 + quaternion.z**2 + quaternion.w**2
        )
        self.assertAlmostEqual(norm, 1.0)
        markers = node._marker_publisher.messages[0].markers
        self.assertEqual(len(markers), 7)
        self.assertEqual(markers[0].action, Marker.DELETEALL)
        self.assertEqual({marker.header.frame_id for marker in markers}, {header.frame_id})

    def test_empty_workspace_publishes_current_empty_cloud_and_clears_markers(self):
        node = make_node()
        header = Header(stamp=rospy.Time(321, 654), frame_id="camera_frame")
        message = SimpleNamespace(header=header)
        node._cloud_to_arrays = lambda unused: (
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 3), dtype=np.float32),
        )

        with patch.object(NODE.rospy, "logwarn_throttle"):
            node._process_cloud(message)

        self.assertEqual(node._adapter.calls, [])
        self.assertEqual(len(node._input_publisher.messages), 1)
        empty_cloud = node._input_publisher.messages[0]
        self.assertEqual(empty_cloud.header.frame_id, header.frame_id)
        self.assertEqual(empty_cloud.header.stamp, header.stamp)
        self.assertEqual(empty_cloud.width, 0)
        self.assertEqual(len(node._marker_publisher.messages), 1)
        self.assertEqual(node._marker_publisher.messages[0].markers[0].action, Marker.DELETEALL)
        self.assertEqual(node._best_publisher.messages, [])


class BackendFailureClassificationTest(unittest.TestCase):
    def test_cuda_oom_and_minkowski_failures_are_fatal(self):
        self.assertTrue(hasattr(NODE, "is_fatal_backend_error"))
        classifier = NODE.is_fatal_backend_error
        self.assertTrue(classifier(RuntimeError("CUDA out of memory")))
        self.assertTrue(classifier(RuntimeError("MinkowskiEngine kernel launch failed")))
        self.assertTrue(classifier(RuntimeError("Minkowski Engine backend unavailable")))
        self.assertFalse(classifier(RuntimeError("single frame could not be decoded")))

    def test_run_shuts_down_and_reraises_minkowski_failure(self):
        node = NODE.AnyGraspD405Node.__new__(NODE.AnyGraspD405Node)
        node._inference_rate = 1.0
        message = SimpleNamespace(header=Header(frame_id="camera_frame"))
        node._latest = Mock()
        node._latest.take_latest.return_value = (message, 1)
        node._process_cloud = Mock(
            side_effect=RuntimeError("MinkowskiEngine kernel launch failed")
        )
        node._publish_clear_markers = Mock()
        fake_rate = Mock()

        with patch.object(NODE.rospy, "Rate", return_value=fake_rate), patch.object(
            NODE.rospy, "is_shutdown", side_effect=[False, True]
        ), patch.object(NODE.rospy, "logfatal"), patch.object(
            NODE.rospy, "logerr_throttle"
        ), patch.object(NODE.rospy, "signal_shutdown") as signal_shutdown:
            with self.assertRaisesRegex(RuntimeError, "MinkowskiEngine"):
                node.run()

        signal_shutdown.assert_called_once()


class AnyGraspAdapterWorkingDirectoryTest(unittest.TestCase):
    def test_adapter_restores_working_directory_after_model_load(self):
        environment = __import__("os").environ.copy()
        ros_python = "/opt/ros/noetic/lib/python3/dist-packages"
        existing = environment.get("PYTHONPATH", "")
        environment["PYTHONPATH"] = ros_python + (
            __import__("os").pathsep + existing if existing else ""
        )
        probe = f'''import importlib.util
import os
import tempfile
from pathlib import Path

node_path = Path({str(NODE_PATH)!r})
spec = importlib.util.spec_from_file_location("adapter_cwd_probe", node_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
original = os.getcwd()
with tempfile.TemporaryDirectory() as temporary:
    sdk = Path(temporary) / "sdk"
    sdk.mkdir()
    checkpoint = sdk / "checkpoint.tar"
    checkpoint.write_bytes(b"checkpoint")
    (sdk / "gsnet.py").write_text(
        "class AnyGrasp:\\n"
        "    def __init__(self, config): self.config = config\\n"
        "    def load_net(self): return None\\n",
        encoding="utf-8",
    )
    config = {{
        "max_gripper_width": 0.1,
        "gripper_height": 0.03,
        "top_down_grasp": True,
        "debug": True,
        "voxel_size": 0.005,
        "apply_object_mask": True,
        "dense_grasp": False,
        "collision_detection": True,
    }}
    module.AnyGraspAdapter(str(sdk), str(checkpoint), config)
    print(os.getcwd() == original)
'''

        result = subprocess.run(
            [sys.executable, "-c", probe],
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip().splitlines()[-1], "True")


if __name__ == "__main__":
    unittest.main()
