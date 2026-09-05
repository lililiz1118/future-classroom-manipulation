#!/home/jt001/.conda/envs/anygrasp/bin/python
"""Build a debug target cloud from timestamp-matched YOLO and D405 data."""

import copy
from dataclasses import dataclass
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
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from std_msgs.msg import String
from tf.transformations import quaternion_matrix

from anygrasp_ros.core import select_workspace, transform_points
from anygrasp_ros.preprocessing import RansacPlaneConfig, remove_table_plane
from anygrasp_ros.table_geometry import table_surface_from_plane
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


@dataclass(frozen=True)
class PreprocessedCloud:
    """One exact PointCloud2 sample after base-frame ROI and table RANSAC."""

    stamp_ns: int
    source_frame: str
    message_identity: int
    header: object
    workspace_selection: object
    plane_result: object


@dataclass(frozen=True)
class PreprocessOutcome:
    """Explicit result so cadence deferral is never mistaken for empty geometry."""

    status: str
    result: object = None
    reason: str = None


class ExactPreprocessedCache:
    """Acquisition-identity cache; callers serialize access with node state lock."""

    def __init__(self, duration_sec):
        self._duration_ns = int(round(float(duration_sec) * NSEC_PER_SEC))
        if self._duration_ns <= 0:
            raise ValueError("processed cache duration must be positive")
        self._samples = {}
        self._newest_stamp_ns = None

    @staticmethod
    def key_for(result):
        return (
            int(result.stamp_ns),
            str(result.source_frame),
            int(result.message_identity),
        )

    def get(self, key):
        return self._samples.get(key)

    def add(self, stamp_ns, result):
        stamp_ns = int(stamp_ns)
        self._samples[self.key_for(result)] = result
        if self._newest_stamp_ns is None or stamp_ns > self._newest_stamp_ns:
            self._newest_stamp_ns = stamp_ns
        cutoff = self._newest_stamp_ns - self._duration_ns
        self._samples = {
            key: value
            for key, value in self._samples.items()
            if key[0] >= cutoff
        }

    @property
    def count(self):
        return len(self._samples)


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
    """Rate-bounded table preprocessing plus timestamp-matched target selection."""

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
        self._table_surface_pose_topic = rospy.get_param(
            "~table_surface_pose_topic", "/yolo_world/table_surface_pose"
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
        table_preprocess_rate_hz = float(
            rospy.get_param("~table_preprocess_rate_hz", 5.0)
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
        if not np.isfinite(table_preprocess_rate_hz) or table_preprocess_rate_hz <= 0.0:
            raise ValueError("table_preprocess_rate_hz must be finite and positive")
        self._table_preprocess_period_ns = int(
            round(NSEC_PER_SEC / table_preprocess_rate_hz)
        )

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
        self._processed_cloud_cache = ExactPreprocessedCache(cache_duration)
        self._state_lock = threading.Lock()
        self._camera_model = None
        self._last_seen_detection_stamp_ns = None
        self._active_detection_stamp_ns = None
        self._last_target_activity_ns = None
        self._last_detection_received_ns = None
        self._last_preprocess_started_ns = None
        self._preprocess_inflight = set()
        self._processing = False
        self._target_visible = False

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(cache_duration + 1.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._publisher = rospy.Publisher(
            self._target_cloud_topic, PointCloud2, queue_size=1
        )
        self._table_surface_publisher = rospy.Publisher(
            self._table_surface_pose_topic, PoseStamped, queue_size=1
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
            "cache=%.3fs stamp_delta=%.3fs detection_age=%.3fs preprocess=%.3fHz",
            self._cloud_topic,
            self._detection_topic,
            self._target_cloud_topic,
            cache_duration,
            self._max_stamp_delta_sec,
            max_detection_age_sec,
            table_preprocess_rate_hz,
        )

    def _now_ns(self):
        if hasattr(self, "_clock_now_ns"):
            return int(self._clock_now_ns())
        now = rospy.Time.now()
        return stamp_parts_to_ns(now.secs, now.nsecs)

    def _logwarn(self, message, *args):
        rospy.logwarn_throttle_identical(self._log_throttle_sec, message, *args)

    @staticmethod
    def _plane_result_fields(plane_result):
        plane = plane_result.plane_model
        plane_text = "none" if plane is None else ",".join(
            "%.6f" % float(value) for value in plane
        )
        height = plane_result.table_height
        angle = plane_result.normal_angle_deg
        return {
            "applied": str(bool(plane_result.applied)).lower(),
            "plane_valid": str(bool(plane_result.plane_valid)).lower(),
            "plane_reason": plane_result.reason,
            "plane": plane_text,
            "inliers": int(plane_result.inlier_count),
            "inlier_ratio": "%.6f" % float(plane_result.inlier_ratio),
            "table_height": "n/a" if height is None else "%.6f" % float(height),
            "normal_angle_deg": "n/a" if angle is None else "%.6f" % float(angle),
        }

    def _log_target_empty(self, reason, **fields):
        details = " ".join(
            "%s=%s" % (key, fields[key]) for key in sorted(fields)
        )
        template = "[YOLO target cloud] empty reason=" + str(reason)
        if details:
            self._logwarn(template + " %s", details)
        else:
            self._logwarn(template)

    @staticmethod
    def _preprocess_key(message, stamp_ns):
        return (int(stamp_ns), str(message.header.frame_id), id(message))

    @staticmethod
    def _preprocess_matches(preprocessed, message, stamp_ns):
        return (
            preprocessed.stamp_ns == int(stamp_ns)
            and preprocessed.source_frame == str(message.header.frame_id)
            and preprocessed.message_identity == id(message)
        )

    def _claim_preprocess_work(self, key, now_ns):
        with self._state_lock:
            cached = self._processed_cloud_cache.get(key)
            if cached is not None:
                return PreprocessOutcome("cache_hit", cached)
            if key in self._preprocess_inflight:
                return PreprocessOutcome("in_flight")
            last = self._last_preprocess_started_ns
            if (
                last is not None
                and now_ns >= last
                and now_ns - last < self._table_preprocess_period_ns
            ):
                return PreprocessOutcome("rate_limited")
            self._last_preprocess_started_ns = now_ns
            self._preprocess_inflight.add(key)
            return PreprocessOutcome("started")

    def _preprocess_cloud(self, message):
        """Run the one ROI/RANSAC pass bound to this PointCloud2 header."""
        source_header = message.header
        stamp_ns = stamp_parts_to_ns(
            source_header.stamp.secs, source_header.stamp.nsecs
        )
        points, packed_rgb = parse_cloud_arrays(message)
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
        return PreprocessedCloud(
            stamp_ns=stamp_ns,
            source_frame=str(source_header.frame_id),
            message_identity=id(message),
            header=source_header,
            workspace_selection=workspace_selection,
            plane_result=plane_result,
        )

    def _get_or_preprocess(self, message, stamp_ns, now_ns, trigger):
        key = self._preprocess_key(message, stamp_ns)
        claim = self._claim_preprocess_work(key, now_ns)
        if claim.status != "started":
            return claim
        try:
            try:
                preprocessed = self._preprocess_cloud(message)
            except tf2_ros.TransformException as exc:
                self._logwarn(
                    "[table preprocess] failure reason=tf_failure trigger=%s "
                    "stamp_ns=%d error=%s",
                    trigger,
                    stamp_ns,
                    exc,
                )
                return PreprocessOutcome("failed", reason="tf_failure")
            except (AttributeError, TypeError, ValueError) as exc:
                self._logwarn(
                    "[table preprocess] failure reason=preprocess_failure trigger=%s "
                    "stamp_ns=%d error=%s",
                    trigger,
                    stamp_ns,
                    exc,
                )
                return PreprocessOutcome("failed", reason="preprocess_failure")
            if not self._preprocess_matches(preprocessed, message, stamp_ns):
                self._logwarn(
                    "[table preprocess] failure reason=stamp_or_identity_mismatch "
                    "requested_stamp=%d actual_stamp=%d requested_frame=%s "
                    "actual_frame=%s",
                    stamp_ns,
                    preprocessed.stamp_ns,
                    message.header.frame_id,
                    preprocessed.source_frame,
                )
                return PreprocessOutcome(
                    "failed", reason="stamp_or_identity_mismatch"
                )

            with self._state_lock:
                self._processed_cloud_cache.add(stamp_ns, preprocessed)
                self._preprocess_inflight.discard(key)
            self._publish_table_surface_pose(
                preprocessed.plane_result, preprocessed.header
            )
            if not preprocessed.plane_result.plane_valid:
                workspace_count = (
                    preprocessed.workspace_selection.camera_cloud.workspace_count
                )
                rejection = (
                    "workspace_empty" if workspace_count == 0 else "table_plane_rejected"
                )
                self._logwarn(
                    "[table preprocess] plane rejected reason=%s trigger=%s "
                    "stamp_ns=%d %s",
                    rejection,
                    trigger,
                    stamp_ns,
                    " ".join(
                        "%s=%s" % (field, value)
                        for field, value in self._plane_result_fields(
                            preprocessed.plane_result
                        ).items()
                    ),
                )
            return PreprocessOutcome("processed", preprocessed)
        finally:
            with self._state_lock:
                self._preprocess_inflight.discard(key)

    def _cloud_callback(self, message):
        """Cache every cloud and maintain table geometry at a bounded cadence."""
        try:
            stamp_ns = stamp_parts_to_ns(
                message.header.stamp.secs, message.header.stamp.nsecs
            )
            self._cloud_cache.add(stamp_ns, message)
        except (AttributeError, TypeError, ValueError) as exc:
            self._logwarn("[YOLO target cloud] rejected cloud header: %s", exc)
            return

        now_ns = self._now_ns()
        with self._state_lock:
            last_detection = self._last_detection_received_ns
        detection_recent = (
            last_detection is not None
            and now_ns >= last_detection
            and now_ns - last_detection <= self._max_detection_age_ns
        )
        if not detection_recent:
            self._get_or_preprocess(message, stamp_ns, now_ns, "table_cadence")

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
            self._clear_visible_target(
                reason="stale_detection",
                detection_age_sec="%.6f" % (age_ns / float(NSEC_PER_SEC)),
                limit_sec="%.6f"
                % (self._max_detection_age_ns / float(NSEC_PER_SEC)),
            )
            return
        if detection.frame_id != self._color_frame:
            self._clear_visible_target(
                reason="detection_frame_mismatch",
                detection_frame=detection.frame_id,
                expected_frame=self._color_frame,
            )
            return

        match = self._cloud_cache.nearest(
            detection.stamp_ns, self._max_stamp_delta_sec
        )
        if match is None:
            self._log_target_empty(
                "detection_cloud_cache_mismatch",
                detection_stamp_ns=detection.stamp_ns,
                max_stamp_delta_sec="%.6f" % self._max_stamp_delta_sec,
            )
            return

        with self._state_lock:
            self._last_detection_received_ns = received_ns
            self._processing = True
        try:
            preprocess_outcome = self._get_or_preprocess(
                match.message,
                match.stamp_ns,
                received_ns,
                "target_match",
            )
            if preprocess_outcome.status in ("rate_limited", "in_flight"):
                return
            if preprocess_outcome.status == "failed":
                self._log_target_empty(
                    "target_preprocess_failure",
                    preprocess_reason=preprocess_outcome.reason,
                    cloud_stamp_ns=match.stamp_ns,
                    detection_stamp_ns=detection.stamp_ns,
                    stamp_delta_ns=match.delta_ns,
                )
                self._clear_visible_target(match.message.header)
                return
            self._process_match(detection, match, preprocess_outcome.result)
        except Exception as exc:
            self._log_target_empty("target_processing_failure", error=exc)
            self._clear_visible_target(
                match.message.header,
            )
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
            self._log_target_empty(
                "target_stale",
                max_age_sec="%.6f"
                % (self._max_detection_age_ns / float(NSEC_PER_SEC)),
            )
            self._publish_empty(latest.message.header)

    def _clear_visible_target(self, header=None, reason=None, **fields):
        with self._state_lock:
            was_visible = self._target_visible
            self._target_visible = False
            self._active_detection_stamp_ns = None
            self._last_target_activity_ns = None
        if reason is not None:
            self._log_target_empty(reason, **fields)
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

    def _publish_table_surface_pose(self, plane_result, source_header):
        """Publish the accepted RANSAC plane without running another fit."""
        if not plane_result.plane_valid or plane_result.plane_model is None:
            return False
        try:
            surface = table_surface_from_plane(
                plane_result.plane_model,
                self._workspace_bounds[:4],
            )
        except (TypeError, ValueError) as exc:
            self._logwarn("[YOLO target cloud] invalid accepted table plane: %s", exc)
            return False

        pose = PoseStamped()
        pose.header.stamp = source_header.stamp
        pose.header.frame_id = self._workspace_frame
        pose.pose.position.x = float(surface.center[0])
        pose.pose.position.y = float(surface.center[1])
        pose.pose.position.z = float(surface.center[2])
        pose.pose.orientation.x = float(surface.quaternion[0])
        pose.pose.orientation.y = float(surface.quaternion[1])
        pose.pose.orientation.z = float(surface.quaternion[2])
        pose.pose.orientation.w = float(surface.quaternion[3])
        self._table_surface_publisher.publish(pose)
        return True

    def _process_match(self, detection, match, preprocessed):
        started = time.perf_counter()
        if preprocessed.stamp_ns != match.stamp_ns:
            raise ValueError(
                "preprocess stamp %d does not match cloud stamp %d"
                % (preprocessed.stamp_ns, match.stamp_ns)
            )
        source_header = preprocessed.header
        workspace_selection = preprocessed.workspace_selection
        plane_result = preprocessed.plane_result
        with self._state_lock:
            camera_model = self._camera_model
        if camera_model is None:
            raise RuntimeError("color CameraInfo is not available")

        if self._require_ransac_success and not plane_result.applied:
            if workspace_selection.camera_cloud.workspace_count == 0:
                empty_reason = "workspace_empty"
            elif plane_result.plane_valid:
                empty_reason = "insufficient_target_points"
            else:
                empty_reason = "table_plane_rejected"
            self._log_target_empty(
                empty_reason,
                **self._plane_result_fields(plane_result)
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
        if target.target_count == 0:
            self._log_target_empty(
                "bbox_projection_empty",
                projected_points=target.projected_count,
                target_points=target.target_count,
            )
            self._publish_empty(source_header)
        else:
            output = self._create_cloud(target.points, target.colors, source_header)
            self._publisher.publish(output)
            rospy.loginfo_throttle(
                self._log_throttle_sec,
                "[YOLO target cloud] nonempty target_points=%d",
                target.target_count,
            )
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
