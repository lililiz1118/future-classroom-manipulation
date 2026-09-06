#!/home/jt001/.conda/envs/anygrasp/bin/python
"""Publish RANSAC tabletop geometry from D405 point clouds and TF only."""

import os
import sys
import time


# This base perception task is CPU-only even when the AnyGrasp environment has
# a CUDA-enabled Open3D build.
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"


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
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf.transformations import quaternion_matrix

from anygrasp_ros.core import select_workspace, transform_points
from anygrasp_ros.preprocessing import RansacPlaneConfig, remove_table_plane
from anygrasp_ros.table_geometry import TableSurfaceGeometry


EXPECTED_PYTHON = "/home/jt001/.conda/envs/anygrasp/bin/python"


def parse_xyz_arrays(message):
    """Parse organized or unorganized PointCloud2 XYZ without requiring RGB."""
    fields = {field.name: field for field in message.fields}
    missing = [name for name in ("x", "y", "z") if name not in fields]
    if missing:
        raise ValueError("PointCloud2 missing fields: " + ", ".join(missing))
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype != PointField.FLOAT32 or field.count != 1:
            raise ValueError("PointCloud2 %s must be scalar FLOAT32" % name)

    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    if width == 0 or height == 0:
        return np.empty((0, 3), dtype=np.float32)
    if point_step <= 0 or row_step < width * point_step:
        raise ValueError("PointCloud2 has invalid point_step or row_step")
    required_bytes = (height - 1) * row_step + width * point_step
    if len(message.data) < required_bytes:
        raise ValueError("PointCloud2 data is shorter than its dimensions require")
    for name in ("x", "y", "z"):
        if fields[name].offset < 0 or fields[name].offset + 4 > point_step:
            raise ValueError("PointCloud2 %s field exceeds point_step" % name)

    byte_order = ">" if message.is_bigendian else "<"
    record_dtype = np.dtype(
        {
            "names": ("x", "y", "z"),
            "formats": (byte_order + "f4",) * 3,
            "offsets": tuple(fields[name].offset for name in ("x", "y", "z")),
            "itemsize": point_step,
        }
    )
    records = np.ndarray(
        shape=(height, width),
        dtype=record_dtype,
        buffer=message.data,
        strides=(row_step, point_step),
    )
    return np.column_stack(
        tuple(records[name].reshape(-1) for name in ("x", "y", "z"))
    ).astype(np.float32, copy=False)


def rotation_translation(transform_stamped):
    transform = transform_stamped.transform
    quaternion = transform.rotation
    rotation = quaternion_matrix(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    )[:3, :3]
    translation = np.array(
        [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ],
        dtype=np.float64,
    )
    return rotation, translation


class TableSurfacePublisherNode:
    def __init__(self):
        rospy.init_node("table_surface_publisher", anonymous=False)
        expected_python = rospy.get_param("~python_executable", EXPECTED_PYTHON)
        if os.path.realpath(sys.executable) != os.path.realpath(expected_python):
            raise RuntimeError(
                "table surface publisher must use %s, got %s"
                % (expected_python, sys.executable)
            )

        self._cloud_topic = str(
            rospy.get_param("~cloud_topic", "/d405/depth/color/points")
        )
        self._output_topic = str(
            rospy.get_param("~table_surface_pose_topic", "/table_surface_pose")
        )
        workspace = rospy.get_param("~workspace")
        self._workspace_frame = str(workspace["frame_id"]).strip()
        self._workspace_bounds = tuple(
            float(workspace[name])
            for name in ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")
        )
        detection_rate = float(rospy.get_param("~table_detection_rate", 1.0))
        tf_timeout = float(rospy.get_param("~tf_timeout", 0.2))
        if self._workspace_frame != "ur_arm_base_link":
            raise ValueError("table workspace frame must be ur_arm_base_link")
        if not np.isfinite(detection_rate) or detection_rate <= 0.0:
            raise ValueError("table_detection_rate must be finite and positive")
        if not np.isfinite(tf_timeout) or tf_timeout <= 0.0:
            raise ValueError("tf_timeout must be finite and positive")
        self._detection_period = 1.0 / detection_rate
        self._last_started_at = None
        self._tf_timeout = rospy.Duration.from_sec(tf_timeout)
        self._ransac_config = RansacPlaneConfig(
            enabled=bool(rospy.get_param("~ransac_enabled")),
            distance_threshold=float(rospy.get_param("~ransac_distance_threshold")),
            ransac_n=int(rospy.get_param("~ransac_n")),
            num_iterations=int(rospy.get_param("~ransac_num_iterations")),
            min_points=int(rospy.get_param("~ransac_min_points")),
            max_normal_angle_deg=float(
                rospy.get_param("~ransac_max_normal_angle_deg")
            ),
            table_height_min=float(rospy.get_param("~ransac_table_height_min")),
            table_height_max=float(rospy.get_param("~ransac_table_height_max")),
            min_inliers=int(rospy.get_param("~ransac_min_inliers")),
            min_inlier_ratio=float(rospy.get_param("~ransac_min_inlier_ratio")),
            min_object_points=int(rospy.get_param("~ransac_min_object_points")),
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._publisher = rospy.Publisher(
            self._output_topic, PoseStamped, queue_size=1, latch=True
        )
        self._subscriber = rospy.Subscriber(
            self._cloud_topic,
            PointCloud2,
            self._cloud_callback,
            queue_size=1,
            buff_size=32 * 1024 * 1024,
        )
        rospy.loginfo(
            "[table surface] ready | cloud=%s output=%s frame=%s rate=%.3fHz",
            self._cloud_topic,
            self._output_topic,
            self._workspace_frame,
            detection_rate,
        )

    def _publish_plane_result(self, plane_result, stamp):
        if not plane_result.plane_valid:
            return False
        geometry = plane_result.table_geometry
        if geometry is None:
            raise ValueError("valid table plane has no table_geometry")
        if geometry.frame_id != self._workspace_frame:
            raise ValueError(
                "table geometry frame must be %s, got %s"
                % (self._workspace_frame, geometry.frame_id or "empty")
            )
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = geometry.frame_id
        pose.pose.position.x = float(geometry.center[0])
        pose.pose.position.y = float(geometry.center[1])
        pose.pose.position.z = float(geometry.center[2])
        pose.pose.orientation.x = float(geometry.quaternion[0])
        pose.pose.orientation.y = float(geometry.quaternion[1])
        pose.pose.orientation.z = float(geometry.quaternion[2])
        pose.pose.orientation.w = float(geometry.quaternion[3])
        self._publisher.publish(pose)
        return True

    def _cloud_callback(self, message):
        started_at = time.monotonic()
        if (
            self._last_started_at is not None
            and started_at - self._last_started_at < self._detection_period
        ):
            return
        self._last_started_at = started_at
        try:
            points = parse_xyz_arrays(message)
            transform = self._tf_buffer.lookup_transform(
                self._workspace_frame,
                message.header.frame_id,
                message.header.stamp,
                self._tf_timeout,
            )
            rotation, translation = rotation_translation(transform)
            workspace_points = transform_points(points, rotation, translation)
            selection = select_workspace(
                points,
                workspace_points,
                np.zeros(points.shape[0], dtype=np.uint32),
                self._workspace_bounds,
            )
            result = remove_table_plane(
                selection.camera_cloud,
                selection.workspace_points,
                self._ransac_config,
                table_frame=self._workspace_frame,
                table_roi_xy=self._workspace_bounds[:4],
            )
            if not self._publish_plane_result(result, message.header.stamp):
                rospy.logwarn_throttle(
                    5.0,
                    "[table surface] no valid plane; keeping last published table | reason=%s",
                    result.reason,
                )
                return
            rospy.loginfo_throttle(
                5.0,
                "[table surface] valid | inliers=%d ratio=%.3f height=%.4f processing=%.1fms",
                result.inlier_count,
                result.inlier_ratio,
                result.table_height,
                (time.monotonic() - started_at) * 1000.0,
            )
        except tf2_ros.TransformException as exc:
            rospy.logwarn_throttle(
                5.0, "[table surface] waiting for D405 TF: %s", exc
            )
        except (AttributeError, TypeError, ValueError) as exc:
            rospy.logwarn_throttle(5.0, "[table surface] rejected cloud: %s", exc)


def main():
    node = TableSurfacePublisherNode()
    rospy.spin()
    return node


if __name__ == "__main__":
    main()
