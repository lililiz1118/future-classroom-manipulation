import importlib.util
import math
import struct
import subprocess
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import yaml


ROS_PYTHON = "/opt/ros/noetic/lib/python3/dist-packages"
if ROS_PYTHON not in sys.path:
    sys.path.append(ROS_PYTHON)
SYSTEM_PYTHON = "/usr/lib/python3/dist-packages"
if SYSTEM_PYTHON not in sys.path:
    sys.path.append(SYSTEM_PYTHON)

import rospy  # noqa: E402
import tf2_ros  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs import point_cloud2  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "anygrasp_d405.yaml"
NODE_PATH = PACKAGE_ROOT / "scripts" / "anygrasp_d405_node.py"
SPEC = importlib.util.spec_from_file_location("anygrasp_d405_node_under_test", NODE_PATH)
NODE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NODE)
from anygrasp_ros.core import FilteredCloud  # noqa: E402
from anygrasp_ros.preprocessing import (  # noqa: E402
    PlaneRemovalResult,
    RansacPlaneConfig,
)


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeGrasp:
    def __init__(self, score):
        self.score = float(score)
        self.translation = np.array([score, 0.02, 0.5], dtype=np.float64)
        self.rotation_matrix = np.array(
            [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
            dtype=np.float64,
        )
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


class FakeTfBuffer:
    def __init__(self, transform=None, error=None):
        self.transform = transform
        self.error = error
        self.calls = []

    def lookup_transform(self, target_frame, source_frame, stamp, timeout):
        self.calls.append((target_frame, source_frame, stamp, timeout))
        if self.error is not None:
            raise self.error
        return self.transform


def make_transform(stamp=rospy.Time(999, 1)):
    transform = TransformStamped()
    transform.header = Header(stamp=stamp, frame_id="ur_arm_base_link")
    transform.child_frame_id = "d405_depth_optical_frame"
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.translation.z = 3.0
    root_half = math.sqrt(0.5)
    transform.transform.rotation.z = root_half
    transform.transform.rotation.w = root_half
    return transform


def make_node(scores=(0.2, 0.9, 0.5)):
    node = NODE.AnyGraspD405Node.__new__(NODE.AnyGraspD405Node)
    node._workspace_frame = "ur_arm_base_link"
    node._workspace_bounds = (0.5, 1.5, 1.5, 3.0, 3.0, 4.0)
    node._dynamic_lims_margin = 0.01
    node._tf_timeout = rospy.Duration.from_sec(0.2)
    node._tf_buffer = FakeTfBuffer(make_transform())
    node._min_workspace_points = 1
    node._ransac_config = RansacPlaneConfig(
        enabled=False,
        distance_threshold=0.008,
        ransac_n=3,
        num_iterations=1000,
        min_points=3,
        max_normal_angle_deg=15.0,
        table_height_min=0.20,
        table_height_max=0.28,
        min_inliers=3,
        min_inlier_ratio=0.20,
        min_object_points=1,
    )
    node._top_n = 2
    node._publish_input = True
    node._inference_rate = 1.0
    node._marker_lifetime = rospy.Duration.from_sec(2.5)
    node._input_publisher = CapturePublisher()
    node._workspace_publisher = CapturePublisher()
    node._object_publisher = CapturePublisher()
    node._marker_publisher = CapturePublisher()
    node._best_publisher = CapturePublisher()
    node._best_base_publisher = CapturePublisher()
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
    def test_float32_rgb_is_read_directly_as_packed_uint32(self):
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

        with patch.object(
            NODE.point_cloud2,
            "read_points",
            side_effect=AssertionError("read_points must not be used"),
        ):
            points, packed_rgb = NODE.AnyGraspD405Node._cloud_to_arrays(cloud)

        np.testing.assert_allclose(points, [[0.1, 0.2, 0.3], [-0.1, 0.0, 0.8]])
        np.testing.assert_array_equal(packed_rgb, [0x00FF0000, 0x000000FF])
        self.assertEqual(points.dtype, np.float32)
        self.assertEqual(packed_rgb.dtype, np.uint32)

    def test_uint32_rgb_and_organized_row_padding_are_read_directly(self):
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.UINT32, 1),
        ]
        point_step = 16
        row_step = 36
        data = bytearray(row_step * 2)
        samples = [
            (0, 0, (0.1, 0.2, 0.3, 0x00102030)),
            (0, 1, (0.4, 0.5, 0.6, 0x00405060)),
            (1, 0, (0.7, 0.8, 0.9, 0x00708090)),
            (1, 1, (1.0, 1.1, 1.2, 0x00A0B0C0)),
        ]
        for row, column, values in samples:
            struct.pack_into("<fffI", data, row * row_step + column * point_step, *values)
        cloud = PointCloud2(
            height=2,
            width=2,
            fields=fields,
            is_bigendian=False,
            point_step=point_step,
            row_step=row_step,
            data=bytes(data),
            is_dense=True,
        )

        points, packed_rgb = NODE.AnyGraspD405Node._cloud_to_arrays(cloud)

        np.testing.assert_allclose(
            points,
            [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9], [1.0, 1.1, 1.2]],
        )
        np.testing.assert_array_equal(
            packed_rgb, [0x00102030, 0x00405060, 0x00708090, 0x00A0B0C0]
        )


class NodeDefaultsTest(unittest.TestCase):
    def test_publish_input_cloud_defaults_to_false_without_parameter(self):
        workspace = {
            "frame_id": "ur_arm_base_link",
            "x_min": -0.5,
            "x_max": 0.5,
            "y_min": -0.5,
            "y_max": 0.5,
            "z_min": 0.1,
            "z_max": 1.5,
        }

        def get_param(name, *default):
            if name == "~python_executable":
                return sys.executable
            if name == "~workspace":
                return workspace
            if default:
                return default[0]
            raise KeyError(name)

        with patch.object(NODE.rospy, "init_node"), patch.object(
            NODE.rospy, "get_param", side_effect=get_param
        ), patch.object(NODE, "AnyGraspAdapter"), patch.object(
            NODE.rospy, "Publisher", return_value=CapturePublisher()
        ), patch.object(NODE.rospy, "Subscriber"), patch.object(
            NODE.tf2_ros, "Buffer", return_value=FakeTfBuffer(make_transform())
        ), patch.object(NODE.tf2_ros, "TransformListener"):
            node = NODE.AnyGraspD405Node()

        self.assertFalse(node._publish_input)

    def test_ransac_parameters_are_loaded_from_checked_in_yaml(self):
        config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))

        def get_param(name, *default):
            key = name.removeprefix("~")
            if key in config:
                return config[key]
            if default:
                return default[0]
            raise KeyError(name)

        with patch.object(NODE.rospy, "init_node"), patch.object(
            NODE.rospy, "get_param", side_effect=get_param
        ), patch.object(NODE, "AnyGraspAdapter"), patch.object(
            NODE.rospy, "Publisher", return_value=CapturePublisher()
        ), patch.object(NODE.rospy, "Subscriber"), patch.object(
            NODE.tf2_ros, "Buffer", return_value=FakeTfBuffer(make_transform())
        ), patch.object(NODE.tf2_ros, "TransformListener"):
            node = NODE.AnyGraspD405Node()

        self.assertEqual(node._object_cloud_topic, "/anygrasp/object_cloud")
        self.assertEqual(
            node._ransac_config,
            RansacPlaneConfig(
                enabled=True,
                distance_threshold=0.008,
                ransac_n=3,
                num_iterations=1000,
                min_points=1000,
                max_normal_angle_deg=15.0,
                table_height_min=0.20,
                table_height_max=0.28,
                min_inliers=500,
                min_inlier_ratio=0.20,
                min_object_points=1000,
            ),
        )


class ProcessCloudTest(unittest.TestCase):
    def test_process_cloud_runs_nms_sort_top_n_and_preserves_header(self):
        node = make_node()
        header = Header(stamp=rospy.Time(123, 456), frame_id="d405_depth_optical_frame")
        message = SimpleNamespace(header=header)
        points = np.array([[0.0, 0.0, 0.5], [0.1, 0.0, 0.6]], dtype=np.float32)
        packed_rgb = np.array([0x00FF0000, 0x0000FF00], dtype=np.uint32)
        node._cloud_to_arrays = lambda unused: (points, packed_rgb)

        with patch.object(NODE.rospy, "logwarn_throttle"):
            node._process_cloud(message)

        self.assertEqual(len(node._adapter.calls), 1)
        np.testing.assert_allclose(node._adapter.calls[0][0], points)
        np.testing.assert_allclose(
            node._adapter.calls[0][2],
            (-0.01, 0.11, -0.01, 0.01, 0.49, 0.61),
            atol=1e-6,
        )
        self.assertEqual(
            node._tf_buffer.calls[0][:3],
            ("ur_arm_base_link", header.frame_id, header.stamp),
        )
        self.assertEqual(node._adapter.group.tracker, {"nms": 1, "sort": 1})
        self.assertEqual(len(node._input_publisher.messages), 1)
        self.assertEqual(node._input_publisher.messages[0].header.frame_id, header.frame_id)
        self.assertEqual(node._input_publisher.messages[0].width, 2)
        self.assertEqual(len(node._workspace_publisher.messages), 1)
        workspace_cloud = node._workspace_publisher.messages[0]
        self.assertEqual(workspace_cloud.header.frame_id, "ur_arm_base_link")
        self.assertEqual(workspace_cloud.header.stamp, header.stamp)
        self.assertEqual(workspace_cloud.width, 2)
        workspace_records = list(
            point_cloud2.read_points(
                workspace_cloud,
                field_names=("x", "y", "z", "rgb"),
                skip_nans=False,
            )
        )
        np.testing.assert_allclose(
            [record[:3] for record in workspace_records],
            [[1.0, 2.0, 3.5], [1.0, 2.1, 3.6]],
            atol=1e-6,
        )
        workspace_rgb = np.asarray(
            [record[3] for record in workspace_records], dtype=np.float32
        ).view(np.uint32)
        np.testing.assert_array_equal(workspace_rgb, [0x00FF0000, 0x0000FF00])
        self.assertEqual(len(node._object_publisher.messages), 1)
        object_cloud = node._object_publisher.messages[0]
        self.assertEqual(object_cloud.header.frame_id, "ur_arm_base_link")
        self.assertEqual(object_cloud.header.stamp, header.stamp)
        self.assertEqual(object_cloud.width, 2)
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
        self.assertEqual(len(node._best_base_publisher.messages), 1)
        base_pose = node._best_base_publisher.messages[0]
        self.assertEqual(base_pose.header.frame_id, "ur_arm_base_link")
        self.assertEqual(base_pose.header.stamp, header.stamp)
        self.assertAlmostEqual(base_pose.pose.position.x, 0.98, places=6)
        self.assertAlmostEqual(base_pose.pose.position.y, 2.9, places=6)
        self.assertAlmostEqual(base_pose.pose.position.z, 3.5, places=6)
        self.assertAlmostEqual(base_pose.pose.orientation.x, 0.5, places=6)
        self.assertAlmostEqual(base_pose.pose.orientation.y, 0.5, places=6)
        self.assertAlmostEqual(base_pose.pose.orientation.z, 0.5, places=6)
        self.assertAlmostEqual(base_pose.pose.orientation.w, 0.5, places=6)
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
            np.empty(0, dtype=np.uint32),
        )

        with patch.object(NODE.rospy, "logwarn_throttle"):
            node._process_cloud(message)

        self.assertEqual(node._adapter.calls, [])
        self.assertEqual(len(node._input_publisher.messages), 1)
        empty_cloud = node._input_publisher.messages[0]
        self.assertEqual(empty_cloud.header.frame_id, header.frame_id)
        self.assertEqual(empty_cloud.header.stamp, header.stamp)
        self.assertEqual(empty_cloud.width, 0)
        self.assertEqual(len(node._workspace_publisher.messages), 1)
        self.assertEqual(
            node._workspace_publisher.messages[0].header.frame_id,
            "ur_arm_base_link",
        )
        self.assertEqual(node._workspace_publisher.messages[0].width, 0)
        self.assertEqual(len(node._object_publisher.messages), 1)
        self.assertEqual(node._object_publisher.messages[0].width, 0)
        self.assertEqual(len(node._marker_publisher.messages), 1)
        self.assertEqual(node._marker_publisher.messages[0].markers[0].action, Marker.DELETEALL)
        self.assertEqual(node._best_publisher.messages, [])
        self.assertEqual(node._best_base_publisher.messages, [])

    def test_ransac_fallback_publishes_original_roi_and_continues_inference(self):
        node = make_node()
        node._ransac_config = RansacPlaneConfig(
            enabled=True,
            distance_threshold=0.008,
            ransac_n=3,
            num_iterations=100,
            min_points=3,
            max_normal_angle_deg=15.0,
            table_height_min=0.20,
            table_height_max=0.28,
            min_inliers=3,
            min_inlier_ratio=0.20,
            min_object_points=1,
        )
        points = np.array([[0.0, 0.0, 0.5], [0.1, 0.0, 0.6]], dtype=np.float32)
        message = SimpleNamespace(
            header=Header(stamp=rospy.Time(22, 33), frame_id="d405_depth_optical_frame")
        )
        node._cloud_to_arrays = lambda unused: (
            points,
            np.array([0x00FF0000, 0x0000FF00], dtype=np.uint32),
        )

        with patch.object(NODE.rospy, "logwarn_throttle") as warning:
            node._process_cloud(message)

        self.assertEqual(node._object_publisher.messages[0].width, 2)
        self.assertEqual(node._input_publisher.messages[0].width, 2)
        np.testing.assert_array_equal(node._adapter.calls[0][0], points)
        self.assertIn("fallback", warning.call_args.args[1])

    def test_workspace_object_and_input_clouds_show_roi_ransac_and_sor_stages(self):
        node = make_node()
        message = SimpleNamespace(
            header=Header(stamp=rospy.Time(7, 8), frame_id="d405_depth_optical_frame")
        )
        points = np.array(
            [[0.0, 0.0, 0.5], [0.1, 0.0, 0.6], [0.2, 0.0, 0.7]],
            dtype=np.float32,
        )
        node._cloud_to_arrays = lambda unused: (
            points,
            np.array([0x00FF0000, 0x0000FF00, 0x000000FF], dtype=np.uint32),
        )

        def keep_two_after_ransac(cloud, workspace_points, unused_config):
            return PlaneRemovalResult(
                camera_cloud=FilteredCloud(
                    points=cloud.points[:2],
                    colors=cloud.colors[:2],
                    raw_count=cloud.raw_count,
                    valid_count=cloud.valid_count,
                    workspace_count=2,
                ),
                workspace_points=workspace_points[:2],
                applied=True,
                reason="accepted",
                plane_model=(0.0, 0.0, -1.0, 0.24),
                inlier_count=1,
                inlier_ratio=1.0 / 3.0,
                table_height=0.24,
                normal_angle_deg=0.0,
            )

        def keep_first(cloud, unused_neighbors, unused_ratio):
            return FilteredCloud(
                points=cloud.points[:1],
                colors=cloud.colors[:1],
                raw_count=cloud.raw_count,
                valid_count=cloud.valid_count,
                workspace_count=1,
            )

        node._outlier_filter_enabled = True
        node._outlier_nb_neighbors = 1
        node._outlier_std_ratio = 1.0
        with patch.object(
            NODE, "remove_table_plane", side_effect=keep_two_after_ransac
        ), patch.object(NODE, "remove_statistical_outliers", side_effect=keep_first):
            node._process_cloud(message)

        self.assertEqual(node._workspace_publisher.messages[0].width, 3)
        self.assertEqual(node._object_publisher.messages[0].width, 2)
        self.assertEqual(node._input_publisher.messages[0].width, 1)

    def test_missing_timestamped_tf_skips_frame_without_parsing_or_inference(self):
        node = make_node()
        node._tf_buffer = FakeTfBuffer(error=tf2_ros.ExtrapolationException("too old"))
        header = Header(stamp=rospy.Time(44, 55), frame_id="d405_depth_optical_frame")
        message = SimpleNamespace(header=header)
        node._cloud_to_arrays = Mock(side_effect=AssertionError("cloud must not be parsed"))

        with patch.object(NODE.rospy, "logwarn_throttle") as warning:
            node._process_cloud(message)

        self.assertEqual(node._adapter.calls, [])
        self.assertEqual(node._workspace_publisher.messages, [])
        self.assertEqual(node._object_publisher.messages, [])
        self.assertEqual(node._input_publisher.messages, [])
        self.assertEqual(node._best_base_publisher.messages, [])
        self.assertIn("waiting for TF", warning.call_args.args[1])

    def test_process_cloud_logs_all_timing_stages_in_milliseconds(self):
        node = make_node()
        message = SimpleNamespace(header=Header(frame_id="camera_frame"))
        node._cloud_to_arrays = lambda unused: (
            np.array([[0.0, 0.0, 0.5]], dtype=np.float32),
            np.array([0x00FF0000], dtype=np.uint32),
        )

        with patch.object(NODE.rospy, "logwarn_throttle"), patch.object(
            NODE.rospy, "loginfo"
        ) as loginfo:
            node._process_cloud(message)

        timing_formats = [
            call.args[0]
            for call in loginfo.call_args_list
            if call.args and "timing ms" in call.args[0]
        ]
        self.assertEqual(len(timing_formats), 1)
        timing_format = timing_formats[0]
        for label in (
            "PointCloud2 parse",
            "finite+workspace filter",
            "RANSAC table filter",
            "RViz object publish",
            "RViz input publish",
            "AnyGrasp inference",
            "NMS+sort",
            "TOTAL",
        ):
            self.assertIn(label, timing_format)

    def test_failed_inference_records_elapsed_time_before_reraising(self):
        node = make_node()
        message = SimpleNamespace(header=Header(frame_id="camera_frame"))
        node._cloud_to_arrays = lambda unused: (
            np.array([[0.0, 0.0, 0.5]], dtype=np.float32),
            np.array([0x00FF0000], dtype=np.uint32),
        )
        node._adapter.infer = Mock(side_effect=RuntimeError("inference failed"))
        clock_values = iter(
            [
                0.0,
                1.0,
                1.001,
                2.0,
                2.001,
                3.0,
                3.002,
                4.0,
                4.001,
                5.0,
                5.001,
                6.0,
                6.010,
                7.0,
            ]
        )

        with patch.object(NODE.time, "perf_counter", side_effect=clock_values), patch.object(
            NODE.rospy, "loginfo"
        ) as loginfo:
            with self.assertRaisesRegex(RuntimeError, "inference failed"):
                node._process_cloud(message)

        timing_calls = [
            call
            for call in loginfo.call_args_list
            if call.args and "timing ms" in call.args[0]
        ]
        self.assertEqual(len(timing_calls), 1)
        self.assertAlmostEqual(timing_calls[0].args[6], 10.0)


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
import sys
import tempfile
import types
from pathlib import Path

torch = types.ModuleType("torch")
torch.set_num_threads = lambda value: None
torch.set_num_interop_threads = lambda value: None
torch.get_num_threads = lambda: 2
torch.get_num_interop_threads = lambda: 1
sys.modules["torch"] = torch

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
