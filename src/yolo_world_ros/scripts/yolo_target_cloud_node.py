#!/home/jt001/.conda/envs/anygrasp/bin/python
"""Build a debug target cloud from timestamp-matched YOLO and D405 data."""

import copy
import os
import sys
import threading
import time


SYSTEM_DIST_PACKAGES = "/usr/lib/python3/dist-packages"
if SYSTEM_DIST_PACKAGES not in sys.path:
    sys.path.append(SYSTEM_DIST_PACKAGES)

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE_SOURCE = os.path.join(PACKAGE_ROOT, "src")
if PACKAGE_SOURCE not in sys.path:
    sys.path.insert(0, PACKAGE_SOURCE)

WORKSPACE_SOURCE = os.path.dirname(PACKAGE_ROOT)
ANYGRASP_SOURCE = os.path.join(WORKSPACE_SOURCE, "anygrasp_ros", "src")
if os.path.isdir(ANYGRASP_SOURCE) and ANYGRASP_SOURCE not in sys.path:
    sys.path.insert(0, ANYGRASP_SOURCE)

import numpy as np
import rospy
import tf2_ros
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from std_msgs.msg import String
from tf.transformations import quaternion_matrix

from anygrasp_ros.core import select_workspace, transform_points
from anygrasp_ros.preprocessing import RansacPlaneConfig, remove_table_plane
from yolo_world_ros.target_cloud import (
    CameraModel,
    TimedCloudCache,
    pack_colors_to_float32,
    parse_cloud_arrays,
    parse_detection_json,
    select_bbox_points,
    stamp_parts_to_ns,
)


NSEC_PER_SEC = 1_000_000_000


def _rotation_translation(transform_stamped):
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


class YoloTargetCloudNode:
    """ROS orchestration; only detection callbacks run geometric processing."""

    def __init__(self):
        rospy.init_node("yolo_target_cloud_node", anonymous=False)

        self._cloud_topic = rospy.get_param(
            "~cloud_topic", "/d405/depth/color/points"
        )
        self._camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/d405/color/camera_info"
        )
        self._detection_topic = rospy.get_param(
            "~detection_topic", "/yolo_world/target_detection"
        )
        self._target_cloud_topic = rospy.get_param(
            "~target_cloud_topic", "/yolo_world/target_cloud"
        )
        self._color_frame = rospy.get_param(
            "~color_frame", "d405_color_optical_frame"
        )
        self._workspace_frame = rospy.get_param(
            "~workspace/frame_id", "ur_arm_base_link"
        )
        workspace = rospy.get_param("~workspace")
        self._workspace_bounds = (
            float(workspace["x_min"]),
            float(workspace["x_max"]),
            float(workspace["y_min"]),
            float(workspace["y_max"]),
            float(workspace["z_min"]),
            float(workspace["z_max"]),
        )

        cache_duration = float(rospy.get_param("~cloud_cache_duration_sec", 1.0))
        self._max_stamp_delta_sec = float(
            rospy.get_param("~max_stamp_delta_sec", 0.02)
        )
        max_detection_age_sec = float(
            rospy.get_param("~max_detection_age_sec", 0.5)
        )
        self._max_detection_age_ns = int(
            round(max_detection_age_sec * NSEC_PER_SEC)
        )
        self._tf_timeout = rospy.Duration(
            float(rospy.get_param("~tf_timeout_sec", 0.2))
        )
        self._stale_check_period_sec = float(
            rospy.get_param("~stale_check_period_sec", 0.1)
        )
        self._require_ransac_success = bool(
            rospy.get_param("~require_ransac_success", True)
        )
        self._log_throttle_sec = float(rospy.get_param("~log_throttle_sec", 1.0))
        if self._max_stamp_delta_sec < 0.0:
            raise ValueError("max_stamp_delta_sec must be non-negative")
        if self._max_detection_age_ns <= 0:
            raise ValueError("max_detection_age_sec must be positive")
        if self._stale_check_period_sec <= 0.0:
            raise ValueError("stale_check_period_sec must be positive")

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

        self._cloud_cache = TimedCloudCache(cache_duration)
        self._state_lock = threading.Lock()
        self._camera_model = None
        self._last_seen_detection_stamp_ns = None
        self._active_detection_stamp_ns = None
        self._last_target_activity_ns = None
        self._processing = False
        self._target_visible = False

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(cache_duration + 1.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._publisher = rospy.Publisher(
            self._target_cloud_topic, PointCloud2, queue_size=1
        )
        self._cloud_subscriber = rospy.Subscriber(
            self._cloud_topic,
            PointCloud2,
            self._cloud_callback,
            queue_size=1,
            buff_size=32 * 1024 * 1024,
        )
        self._camera_info_subscriber = rospy.Subscriber(
            self._camera_info_topic,
            CameraInfo,
            self._camera_info_callback,
            queue_size=1,
        )
        self._detection_subscriber = rospy.Subscriber(
            self._detection_topic,
            String,
            self._detection_callback,
            queue_size=1,
        )
        self._stale_timer = rospy.Timer(
            rospy.Duration(self._stale_check_period_sec),
            self._stale_timer_callback,
        )
        rospy.loginfo(
            "[YOLO target cloud] ready | cloud=%s detection=%s output=%s "
            "cache=%.3fs stamp_delta=%.3fs detection_age=%.3fs",
            self._cloud_topic,
            self._detection_topic,
            self._target_cloud_topic,
            cache_duration,
            self._max_stamp_delta_sec,
            max_detection_age_sec,
        )

    def _now_ns(self):
        if hasattr(self, "_clock_now_ns"):
            return int(self._clock_now_ns())
        now = rospy.Time.now()
        return stamp_parts_to_ns(now.secs, now.nsecs)

    def _logwarn(self, message, *args):
        rospy.logwarn_throttle(self._log_throttle_sec, message, *args)

    def _cloud_callback(self, message):
        """Cache only; deliberately contains no parsing, TF, or RANSAC."""
        try:
            stamp_ns = stamp_parts_to_ns(
                message.header.stamp.secs, message.header.stamp.nsecs
            )
            self._cloud_cache.add(stamp_ns, message)
        except (AttributeError, TypeError, ValueError) as exc:
            self._logwarn("[YOLO target cloud] rejected cloud header: %s", exc)

    def _camera_info_callback(self, message):
        try:
            model = CameraModel(
                message.width,
                message.height,
                message.K,
                message.D,
                message.distortion_model,
            )
        except (AttributeError, TypeError, ValueError) as exc:
            self._logwarn("[YOLO target cloud] rejected CameraInfo: %s", exc)
            return
        with self._state_lock:
            self._camera_model = model

    def _detection_callback(self, message):
        try:
            detection = parse_detection_json(message.data)
        except (AttributeError, TypeError, ValueError) as exc:
            self._logwarn("[YOLO target cloud] rejected detection: %s", exc)
            return

        with self._state_lock:
            if (
                self._last_seen_detection_stamp_ns is not None
                and detection.stamp_ns <= self._last_seen_detection_stamp_ns
            ):
                return
            self._last_seen_detection_stamp_ns = detection.stamp_ns

        received_ns = self._now_ns()
        age_ns = received_ns - detection.stamp_ns
        if age_ns > self._max_detection_age_ns:
            self._logwarn(
                "[YOLO target cloud] stale detection ignored: age=%.3fs limit=%.3fs",
                age_ns / float(NSEC_PER_SEC),
                self._max_detection_age_ns / float(NSEC_PER_SEC),
            )
            self._clear_visible_target()
            return
        if detection.frame_id != self._color_frame:
            self._logwarn(
                "[YOLO target cloud] detection frame %s does not match color frame %s",
                detection.frame_id,
                self._color_frame,
            )
            self._clear_visible_target()
            return

        match = self._cloud_cache.nearest(
            detection.stamp_ns, self._max_stamp_delta_sec
        )
        if match is None:
            self._logwarn(
                "[YOLO target cloud] no same-acquisition cloud within %.3fs",
                self._max_stamp_delta_sec,
            )
            return

        with self._state_lock:
            self._last_target_activity_ns = received_ns
            self._processing = True
        try:
            self._process_match(detection, match)
        except Exception as exc:
            self._logwarn("[YOLO target cloud] processing failed: %s", exc)
            self._clear_visible_target(match.message.header)
        finally:
            with self._state_lock:
                self._processing = False

    def _stale_timer_callback(self, _event):
        now_ns = self._now_ns()
        with self._state_lock:
            if (
                not self._target_visible
                or self._last_target_activity_ns is None
                or self._processing
            ):
                return
            stale = (
                now_ns - self._last_target_activity_ns
                > self._max_detection_age_ns
            )
            if not stale:
                return
            self._target_visible = False
            self._active_detection_stamp_ns = None
            self._last_target_activity_ns = None
        latest = self._cloud_cache.latest()
        if latest is not None:
            self._publish_empty(latest.message.header)

    def _clear_visible_target(self, header=None):
        with self._state_lock:
            was_visible = self._target_visible
            self._target_visible = False
            self._active_detection_stamp_ns = None
            self._last_target_activity_ns = None
        if not was_visible:
            return
        if header is None:
            latest = self._cloud_cache.latest()
            header = None if latest is None else latest.message.header
        if header is not None:
            self._publish_empty(header)

    def _lookup_arrays(self, target_frame, source_frame, stamp):
        transform = self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            stamp,
            self._tf_timeout,
        )
        return _rotation_translation(transform)

    def _process_match(self, detection, match):
        started = time.perf_counter()
        cloud_message = match.message
        source_header = cloud_message.header
        with self._state_lock:
            camera_model = self._camera_model
        if camera_model is None:
            raise RuntimeError("color CameraInfo is not available")

        points, packed_rgb = parse_cloud_arrays(cloud_message)
        workspace_rotation, workspace_translation = self._lookup_arrays(
            self._workspace_frame,
            source_header.frame_id,
            source_header.stamp,
        )
        workspace_points = transform_points(
            points, workspace_rotation, workspace_translation
        )
        workspace_selection = select_workspace(
            points,
            workspace_points,
            packed_rgb,
            self._workspace_bounds,
        )
        plane_result = remove_table_plane(
            workspace_selection.camera_cloud,
            workspace_selection.workspace_points,
            self._ransac_config,
        )
        if self._require_ransac_success and not plane_result.applied:
            self._logwarn(
                "[YOLO target cloud] RANSAC not applied (%s); publishing empty cloud",
                plane_result.reason,
            )
            self._publish_empty(source_header)
            self._mark_target_published(detection.stamp_ns, target_count=0)
            return

        color_rotation, color_translation = self._lookup_arrays(
            self._color_frame,
            source_header.frame_id,
            source_header.stamp,
        )
        color_points = transform_points(
            plane_result.camera_cloud.points,
            color_rotation,
            color_translation,
        )
        target = select_bbox_points(
            plane_result.camera_cloud.points,
            plane_result.camera_cloud.colors,
            color_points,
            camera_model,
            detection.bbox,
        )
        output = self._create_cloud(target.points, target.colors, source_header)
        self._publisher.publish(output)
        self._mark_target_published(detection.stamp_ns, target.target_count)

        elapsed_ms = (time.perf_counter() - started) * 1000.0
        rospy.loginfo_throttle(
            self._log_throttle_sec,
            "[YOLO target cloud] class=%s confidence=%.3f "
            "bbox=(%d,%d,%d,%d) cloud_stamp=%d.%09d detection_stamp=%d.%09d "
            "delta=%.3fms raw=%d workspace=%d ransac=%d applied=%s "
            "projected=%d target=%d processing=%.1fms"
            % (
                detection.class_name,
                detection.confidence,
                detection.bbox.xmin,
                detection.bbox.ymin,
                detection.bbox.xmax,
                detection.bbox.ymax,
                match.stamp_ns // NSEC_PER_SEC,
                match.stamp_ns % NSEC_PER_SEC,
                detection.stamp_ns // NSEC_PER_SEC,
                detection.stamp_ns % NSEC_PER_SEC,
                match.delta_ns / 1e6,
                workspace_selection.camera_cloud.raw_count,
                workspace_selection.camera_cloud.workspace_count,
                plane_result.camera_cloud.workspace_count,
                plane_result.applied,
                target.projected_count,
                target.target_count,
                elapsed_ms,
            ),
        )

    @staticmethod
    def _create_cloud(points, colors, source_header):
        header = copy.deepcopy(source_header)
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.FLOAT32, 1),
        ]
        packed_rgb = pack_colors_to_float32(colors)
        records = np.column_stack((points, packed_rgb)).tolist()
        return point_cloud2.create_cloud(header, fields, records)

    def _mark_target_published(self, detection_stamp_ns, target_count):
        visible = int(target_count) > 0
        activity_ns = self._now_ns() if visible else None
        with self._state_lock:
            self._target_visible = visible
            self._active_detection_stamp_ns = detection_stamp_ns if visible else None
            self._last_target_activity_ns = activity_ns

    def _publish_empty(self, source_header):
        empty = np.empty((0, 3), dtype=np.float32)
        self._publisher.publish(self._create_cloud(empty, empty, source_header))


def main():
    node = YoloTargetCloudNode()
    rospy.spin()
    return node


if __name__ == "__main__":
    main()
