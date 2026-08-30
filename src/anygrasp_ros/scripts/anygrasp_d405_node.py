#!/home/jt001/.conda/envs/anygrasp/bin/python
"""Run AnyGrasp on the newest D405 cloud and publish camera-frame results."""

import os
import sys
import threading
import time
from types import SimpleNamespace


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
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

if SYSTEM_DIST_PACKAGES in sys.path:
    sys.path.remove(SYSTEM_DIST_PACKAGES)

from anygrasp_ros.core import (
    decode_packed_rgb,
    filter_workspace,
    grasp_axes,
    rotation_matrix_to_quaternion,
)


EXPECTED_PYTHON = "/home/jt001/.conda/envs/anygrasp/bin/python"


def is_fatal_backend_error(error):
    """Return True for backend failures that make later inference unreliable."""
    description = "%s.%s: %s" % (
        type(error).__module__,
        type(error).__name__,
        error,
    )
    normalized = description.lower()
    return (
        "out of memory" in normalized
        or "minkowskiengine" in normalized
        or "minkowski engine" in normalized
    )


class LatestMessageBuffer:
    """Store one immutable ROS message reference and discard older frames."""

    def __init__(self):
        self._lock = threading.Lock()
        self._message = None
        self._sequence = 0

    def update(self, message):
        with self._lock:
            self._message = message
            self._sequence += 1

    def take_latest(self, last_sequence):
        with self._lock:
            if self._message is None or self._sequence == last_sequence:
                return None, last_sequence
            return self._message, self._sequence


class AnyGraspAdapter:
    """Thin adapter around the exact SDK API used by the validated demo."""

    def __init__(self, sdk_dir, checkpoint_path, config):
        if not os.path.isdir(sdk_dir):
            raise FileNotFoundError("AnyGrasp SDK directory not found: " + sdk_dir)
        if not os.path.isfile(checkpoint_path):
            raise FileNotFoundError("AnyGrasp checkpoint not found: " + checkpoint_path)
        if sdk_dir not in sys.path:
            sys.path.insert(0, sdk_dir)
        original_directory = os.getcwd()
        try:
            os.chdir(sdk_dir)
            from gsnet import AnyGrasp

            sdk_config = SimpleNamespace(
                checkpoint_path=checkpoint_path,
                max_gripper_width=float(config["max_gripper_width"]),
                gripper_height=float(config["gripper_height"]),
                top_down_grasp=bool(config["top_down_grasp"]),
                debug=bool(config["debug"]),
            )
            sdk_config.max_gripper_width = max(
                0.0, min(0.1, sdk_config.max_gripper_width)
            )
            self._model = AnyGrasp(sdk_config)
            self._model.load_net()
        finally:
            os.chdir(original_directory)
        self._voxel_size = float(config["voxel_size"])
        self._apply_object_mask = bool(config["apply_object_mask"])
        self._dense_grasp = bool(config["dense_grasp"])
        self._collision_detection = bool(config["collision_detection"])

    def infer(self, points, colors, workspace_bounds):
        grasps, _ = self._model.get_grasp(
            points,
            colors,
            lims=list(workspace_bounds),
            voxel_size=self._voxel_size,
            apply_object_mask=self._apply_object_mask,
            dense_grasp=self._dense_grasp,
            collision_detection=self._collision_detection,
        )
        return grasps


class AnyGraspD405Node:
    def __init__(self):
        rospy.init_node("anygrasp_d405_node")
        expected_python = rospy.get_param("~python_executable", EXPECTED_PYTHON)
        if os.path.realpath(sys.executable) != os.path.realpath(expected_python):
            raise RuntimeError(
                "AnyGrasp node must use %s, got %s" % (expected_python, sys.executable)
            )

        self._cloud_topic = rospy.get_param("~cloud_topic", "/d405/depth/color/points")
        self._best_topic = rospy.get_param("~best_grasp_topic", "/anygrasp/best_grasp")
        self._marker_topic = rospy.get_param("~marker_topic", "/anygrasp/grasp_markers")
        self._input_cloud_topic = rospy.get_param("~input_cloud_topic", "/anygrasp/input_cloud")
        self._top_n = int(rospy.get_param("~top_n", 10))
        self._inference_rate = float(rospy.get_param("~inference_rate", 1.0))
        self._min_workspace_points = int(rospy.get_param("~min_workspace_points", 1000))
        self._publish_input = bool(rospy.get_param("~publish_input_cloud", True))
        if self._top_n <= 0 or self._inference_rate <= 0.0 or self._min_workspace_points <= 0:
            raise ValueError("top_n, inference_rate, and min_workspace_points must be positive")

        workspace = rospy.get_param("~workspace")
        self._workspace_bounds = (
            float(workspace["x_min"]),
            float(workspace["x_max"]),
            float(workspace["y_min"]),
            float(workspace["y_max"]),
            float(workspace["z_min"]),
            float(workspace["z_max"]),
        )
        adapter_config = {
            "max_gripper_width": rospy.get_param("~max_gripper_width", 0.1),
            "gripper_height": rospy.get_param("~gripper_height", 0.03),
            "top_down_grasp": rospy.get_param("~top_down_grasp", True),
            "debug": rospy.get_param("~debug", True),
            "voxel_size": rospy.get_param("~voxel_size", 0.005),
            "apply_object_mask": rospy.get_param("~apply_object_mask", True),
            "dense_grasp": rospy.get_param("~dense_grasp", False),
            "collision_detection": rospy.get_param("~collision_detection", True),
        }
        sdk_dir = rospy.get_param(
            "~sdk_dir", "/home/jt001/anygrasp_sdk/grasp_detection"
        )
        checkpoint_path = rospy.get_param(
            "~checkpoint_path",
            "/home/jt001/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar",
        )

        rospy.loginfo("[AnyGrasp] Python: %s", sys.executable)
        rospy.loginfo("[AnyGrasp] loading model from %s", checkpoint_path)
        self._adapter = AnyGraspAdapter(sdk_dir, checkpoint_path, adapter_config)
        rospy.loginfo("[AnyGrasp] model loaded")

        self._best_publisher = rospy.Publisher(
            self._best_topic, PoseStamped, queue_size=1
        )
        self._marker_publisher = rospy.Publisher(
            self._marker_topic, MarkerArray, queue_size=1
        )
        self._input_publisher = rospy.Publisher(
            self._input_cloud_topic, PointCloud2, queue_size=1
        )
        self._latest = LatestMessageBuffer()
        self._subscriber = rospy.Subscriber(
            self._cloud_topic,
            PointCloud2,
            self._latest.update,
            queue_size=1,
            buff_size=32 * 1024 * 1024,
        )
        self._marker_lifetime = rospy.Duration.from_sec(
            max(2.5 / self._inference_rate, 1.0)
        )

    @staticmethod
    def _header_from_cloud(message):
        return Header(stamp=message.header.stamp, frame_id=message.header.frame_id)

    @staticmethod
    def _cloud_to_arrays(message):
        fields = {field.name: field for field in message.fields}
        missing = [name for name in ("x", "y", "z", "rgb") if name not in fields]
        if missing:
            raise ValueError("PointCloud2 missing fields: " + ", ".join(missing))
        rgb_datatype = fields["rgb"].datatype
        if rgb_datatype not in (PointField.FLOAT32, PointField.UINT32):
            raise ValueError("PointCloud2 rgb must be packed FLOAT32 or UINT32")

        rows = list(
            point_cloud2.read_points(
                message,
                field_names=("x", "y", "z", "rgb"),
                skip_nans=False,
            )
        )
        if not rows:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
            )
        points = np.fromiter(
            (coordinate for row in rows for coordinate in row[:3]),
            dtype=np.float32,
            count=len(rows) * 3,
        ).reshape(-1, 3)
        rgb_dtype = np.float32 if rgb_datatype == PointField.FLOAT32 else np.uint32
        packed_rgb = np.fromiter(
            (row[3] for row in rows), dtype=rgb_dtype, count=len(rows)
        )
        colors = decode_packed_rgb(packed_rgb, rgb_datatype)
        return points, colors

    @staticmethod
    def _pack_colors(colors):
        channels = np.clip(np.rint(colors * 255.0), 0.0, 255.0).astype(np.uint32)
        packed = (channels[:, 0] << 16) | (channels[:, 1] << 8) | channels[:, 2]
        return packed.astype(np.uint32, copy=False).view(np.float32)

    def _publish_input_cloud(self, filtered, header):
        if not self._publish_input:
            return
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.FLOAT32, 1),
        ]
        packed_rgb = self._pack_colors(filtered.colors)
        records = np.column_stack((filtered.points, packed_rgb)).tolist()
        self._input_publisher.publish(point_cloud2.create_cloud(header, fields, records))

    @staticmethod
    def _point(values):
        return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))

    @staticmethod
    def _color(red, green, blue, alpha=1.0):
        return ColorRGBA(r=red, g=green, b=blue, a=alpha)

    def _clear_marker(self, header):
        marker = Marker()
        marker.header = header
        marker.ns = "anygrasp_clear"
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    def _publish_clear_markers(self, header):
        self._marker_publisher.publish(MarkerArray(markers=[self._clear_marker(header)]))

    def _publish_grasps(self, grasps, header):
        markers = [self._clear_marker(header)]
        marker_id = 1
        for index in range(len(grasps)):
            grasp = grasps[index]
            center = np.asarray(grasp.translation, dtype=np.float64)
            approach, opening, _ = grasp_axes(grasp.rotation_matrix)
            is_best = index == 0

            center_marker = Marker()
            center_marker.header = header
            center_marker.ns = "anygrasp_centers"
            center_marker.id = marker_id
            marker_id += 1
            center_marker.type = Marker.SPHERE
            center_marker.action = Marker.ADD
            center_marker.pose.position = self._point(center)
            center_marker.pose.orientation.w = 1.0
            diameter = 0.018 if is_best else 0.011
            center_marker.scale.x = diameter
            center_marker.scale.y = diameter
            center_marker.scale.z = diameter
            center_marker.color = (
                self._color(0.1, 1.0, 0.1) if is_best else self._color(1.0, 0.7, 0.1)
            )
            center_marker.lifetime = self._marker_lifetime
            markers.append(center_marker)

            approach_marker = Marker()
            approach_marker.header = header
            approach_marker.ns = "anygrasp_approach"
            approach_marker.id = marker_id
            marker_id += 1
            approach_marker.type = Marker.ARROW
            approach_marker.action = Marker.ADD
            approach_marker.points = [
                self._point(center),
                self._point(center + approach * (0.09 if is_best else 0.065)),
            ]
            approach_marker.scale.x = 0.007 if is_best else 0.004
            approach_marker.scale.y = 0.014 if is_best else 0.008
            approach_marker.scale.z = 0.018 if is_best else 0.012
            approach_marker.color = (
                self._color(0.1, 1.0, 0.1) if is_best else self._color(1.0, 0.2, 0.1)
            )
            approach_marker.lifetime = self._marker_lifetime
            markers.append(approach_marker)

            half_width = max(float(grasp.width), 0.01) * 0.5
            jaw_depth = max(float(grasp.depth), 0.025)
            left = center - opening * half_width
            right = center + opening * half_width
            gripper_marker = Marker()
            gripper_marker.header = header
            gripper_marker.ns = "anygrasp_opening"
            gripper_marker.id = marker_id
            marker_id += 1
            gripper_marker.type = Marker.LINE_LIST
            gripper_marker.action = Marker.ADD
            gripper_marker.points = [
                self._point(left),
                self._point(right),
                self._point(left),
                self._point(left + approach * jaw_depth),
                self._point(right),
                self._point(right + approach * jaw_depth),
            ]
            gripper_marker.scale.x = 0.006 if is_best else 0.003
            gripper_marker.color = (
                self._color(0.1, 0.8, 1.0) if is_best else self._color(0.2, 0.5, 1.0)
            )
            gripper_marker.lifetime = self._marker_lifetime
            markers.append(gripper_marker)

        self._marker_publisher.publish(MarkerArray(markers=markers))

        best = grasps[0]
        quaternion = rotation_matrix_to_quaternion(best.rotation_matrix)
        pose = PoseStamped()
        pose.header = header
        pose.pose.position = self._point(best.translation)
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        self._best_publisher.publish(pose)

    def _process_cloud(self, message):
        header = self._header_from_cloud(message)
        points, colors = self._cloud_to_arrays(message)
        filtered = filter_workspace(points, colors, self._workspace_bounds)
        rospy.loginfo(
            "[AnyGrasp] cloud frame: %s | raw points: %d | valid points: %d | workspace points: %d",
            header.frame_id,
            filtered.raw_count,
            filtered.valid_count,
            filtered.workspace_count,
        )
        self._publish_input_cloud(filtered, header)
        if filtered.workspace_count == 0:
            rospy.logwarn_throttle(5.0, "[AnyGrasp] workspace is empty; skipping inference")
            self._publish_clear_markers(header)
            return
        if filtered.workspace_count < self._min_workspace_points:
            rospy.logwarn_throttle(
                5.0,
                "[AnyGrasp] workspace has only %d points (minimum %d); skipping inference",
                filtered.workspace_count,
                self._min_workspace_points,
            )
            self._publish_clear_markers(header)
            return

        started = time.perf_counter()
        grasps = self._adapter.infer(
            filtered.points, filtered.colors, self._workspace_bounds
        )
        before_nms = len(grasps)
        if before_nms == 0:
            elapsed = time.perf_counter() - started
            rospy.logwarn_throttle(5.0, "[AnyGrasp] no grasp detected")
            rospy.loginfo(
                "[AnyGrasp] inference time: %.3f s | grasps before nms: 0 | grasps after nms: 0 | published grasps: 0",
                elapsed,
            )
            self._publish_clear_markers(header)
            return

        grasps = grasps.nms().sort_by_score()
        after_nms = len(grasps)
        selected = grasps[: min(self._top_n, after_nms)]
        elapsed = time.perf_counter() - started
        if len(selected) == 0:
            self._publish_clear_markers(header)
            return
        self._publish_grasps(selected, header)
        rospy.loginfo(
            "[AnyGrasp] inference time: %.3f s | grasps before nms: %d | grasps after nms: %d | published grasps: %d | best score: %.6f",
            elapsed,
            before_nms,
            after_nms,
            len(selected),
            float(selected[0].score),
        )

    def run(self):
        rate = rospy.Rate(self._inference_rate)
        last_sequence = 0
        while not rospy.is_shutdown():
            message, sequence = self._latest.take_latest(last_sequence)
            if message is None:
                rospy.logwarn_throttle(5.0, "[AnyGrasp] waiting for point cloud")
                rate.sleep()
                continue
            last_sequence = sequence
            try:
                self._process_cloud(message)
            except Exception as exc:
                if is_fatal_backend_error(exc):
                    rospy.logfatal("[AnyGrasp] fatal inference backend error: %s", exc)
                    rospy.signal_shutdown("fatal AnyGrasp inference backend error")
                    raise
                rospy.logerr_throttle(5.0, "[AnyGrasp] frame processing failed: %s", exc)
                self._publish_clear_markers(self._header_from_cloud(message))
            rate.sleep()


def main():
    try:
        node = AnyGraspD405Node()
        node.run()
    except Exception as exc:
        if rospy.core.is_initialized():
            rospy.logfatal("[AnyGrasp] node terminated: %s", exc)
        else:
            print("[AnyGrasp] fatal startup error: %s" % exc, file=sys.stderr)
        raise


if __name__ == "__main__":
    main()
