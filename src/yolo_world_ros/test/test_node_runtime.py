import importlib.util
import json
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


SYSTEM_DIST_PACKAGES = "/usr/lib/python3/dist-packages"
ROS_DIST_PACKAGES = "/opt/ros/noetic/lib/python3/dist-packages"
for path in (SYSTEM_DIST_PACKAGES, ROS_DIST_PACKAGES):
    if path not in sys.path:
        sys.path.append(path)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = PACKAGE_ROOT / "src"
ANYGRASP_SOURCE = PACKAGE_ROOT.parent / "anygrasp_ros" / "src"
for path in (PACKAGE_SOURCE, ANYGRASP_SOURCE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

NODE_PATH = PACKAGE_ROOT / "scripts" / "yolo_target_cloud_node.py"
SPEC = importlib.util.spec_from_file_location("yolo_target_cloud_node", NODE_PATH)
NODE_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NODE_MODULE)

from yolo_world_ros.target_cloud import TimedCloudCache, stamp_parts_to_ns  # noqa: E402
from anygrasp_ros.core import FilteredCloud  # noqa: E402


def detection_message(secs=10, nsecs=0):
    return SimpleNamespace(
        data=json.dumps(
            {
                "header": {
                    "stamp": {"secs": secs, "nsecs": nsecs},
                    "frame_id": "d405_color_optical_frame",
                },
                "bbox": {"xmin": 10, "ymin": 20, "xmax": 30, "ymax": 40},
                "class_name": "box",
                "confidence": 0.8,
            }
        )
    )


def cloud_message(secs=10, nsecs=0):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(secs=secs, nsecs=nsecs),
            frame_id="d405_depth_optical_frame",
        )
    )


def bare_node(now_ns):
    node = NODE_MODULE.YoloTargetCloudNode.__new__(NODE_MODULE.YoloTargetCloudNode)
    node._cloud_cache = TimedCloudCache(1.0)
    node._processed_cloud_cache = NODE_MODULE.ExactPreprocessedCache(1.0)
    node._table_preprocess_period_ns = 200000000
    node._last_preprocess_started_ns = None
    node._preprocess_inflight = set()
    node._last_detection_received_ns = None
    node._max_stamp_delta_sec = 0.02
    node._max_detection_age_ns = 500000000
    node._color_frame = "d405_color_optical_frame"
    node._last_seen_detection_stamp_ns = None
    node._active_detection_stamp_ns = None
    node._last_target_activity_ns = None
    node._processing = False
    node._target_visible = False
    node._clock_now_ns = lambda: now_ns
    node._camera_model = object()
    node._state_lock = NODE_MODULE.threading.Lock()
    node._processed = []
    node._process_match = lambda *args: node._processed.append(args)
    node._published_empty_headers = []
    node._publish_empty = lambda header, *args, **kwargs: node._published_empty_headers.append(header)
    node._logwarn = lambda *args: None
    node._empty_reasons = []
    node._log_target_empty = lambda reason, **_fields: node._empty_reasons.append(reason)
    return node


class DetectionDrivenRuntimeTest(unittest.TestCase):
    def test_rate_limited_detection_preserves_recent_visible_target(self):
        now_ns = stamp_parts_to_ns(10, 100000000)
        node = bare_node(now_ns)
        cloud = cloud_message(10, 0)
        node._cloud_cache.add(stamp_parts_to_ns(10, 0), cloud)
        node._last_preprocess_started_ns = now_ns
        node._target_visible = True
        node._active_detection_stamp_ns = stamp_parts_to_ns(9, 900000000)
        previous_activity_ns = stamp_parts_to_ns(10, 0)
        node._last_target_activity_ns = previous_activity_ns
        node._preprocess_cloud = lambda *_: self.fail(
            "rate-limited detection repeated preprocessing"
        )

        node._detection_callback(detection_message(10, 0))

        self.assertEqual(node._published_empty_headers, [])
        self.assertTrue(node._target_visible)
        self.assertEqual(node._last_target_activity_ns, previous_activity_ns)
        self.assertEqual(node._processed, [])

    def test_concurrent_callbacks_run_single_ransac_for_same_cloud_identity(self):
        now_ns = stamp_parts_to_ns(10, 0)
        node = bare_node(now_ns)
        cloud = cloud_message(10, 0)
        entered = threading.Event()
        release = threading.Event()
        calls = []

        def preprocess(message):
            calls.append(message)
            entered.set()
            self.assertTrue(release.wait(2.0))
            return NODE_MODULE.PreprocessedCloud(
                stamp_ns=now_ns,
                source_frame=message.header.frame_id,
                message_identity=id(message),
                header=message.header,
                workspace_selection=object(),
                plane_result=SimpleNamespace(plane_valid=True, applied=True),
            )

        node._preprocess_cloud = preprocess
        node._publish_table_surface_pose = lambda *_: True
        cloud_thread = threading.Thread(target=node._cloud_callback, args=(cloud,))
        cloud_thread.start()
        self.assertTrue(entered.wait(2.0))

        node._detection_callback(detection_message(10, 0))
        release.set()
        cloud_thread.join(2.0)

        self.assertFalse(cloud_thread.is_alive())
        self.assertEqual(calls, [cloud])
        self.assertEqual(node._published_empty_headers, [])

    def test_completion_between_cache_miss_and_claim_reuses_finished_result(self):
        now_ns = stamp_parts_to_ns(10, 500000000)
        node = bare_node(now_ns)
        cloud = cloud_message(10, 0)
        stamp_ns = stamp_parts_to_ns(10, 0)
        completed = NODE_MODULE.PreprocessedCloud(
            stamp_ns=stamp_ns,
            source_frame=cloud.header.frame_id,
            message_identity=id(cloud),
            header=cloud.header,
            workspace_selection=object(),
            plane_result=SimpleNamespace(plane_valid=True, applied=True),
        )
        at_claim = threading.Event()
        release_claim = threading.Event()
        original_claim = node._claim_preprocess_work

        def delayed_claim(key, claim_now_ns):
            at_claim.set()
            self.assertTrue(release_claim.wait(2.0))
            return original_claim(key, claim_now_ns)

        node._claim_preprocess_work = delayed_claim
        duplicate_calls = []
        node._preprocess_cloud = lambda message: (
            duplicate_calls.append(message) or completed
        )
        node._publish_table_surface_pose = lambda *_: True
        outcomes = []
        claimant = threading.Thread(
            target=lambda: outcomes.append(
                node._get_or_preprocess(
                    cloud, stamp_ns, now_ns, "completion_boundary_test"
                )
            )
        )
        claimant.start()
        self.assertTrue(at_claim.wait(2.0))
        node._processed_cloud_cache.add(stamp_ns, completed)
        release_claim.set()
        claimant.join(2.0)

        self.assertFalse(claimant.is_alive())
        self.assertEqual(duplicate_calls, [])
        self.assertEqual(outcomes[0].status, "cache_hit")
        self.assertIs(outcomes[0].result, completed)

    def test_first_processing_failure_is_diagnosed_when_target_not_visible(self):
        node = bare_node(stamp_parts_to_ns(10, 0))

        node._clear_visible_target(
            cloud_message(10, 0).header,
            reason="target_processing_failure",
            error="boom",
        )

        self.assertEqual(node._empty_reasons, ["target_processing_failure"])
        self.assertEqual(node._published_empty_headers, [])

    def test_empty_diagnostic_categories_have_independent_throttle_keys(self):
        node = bare_node(stamp_parts_to_ns(10, 0))
        templates = []
        node._logwarn = lambda template, *_args: templates.append(template)

        NODE_MODULE.YoloTargetCloudNode._log_target_empty(node, "workspace_empty")
        NODE_MODULE.YoloTargetCloudNode._log_target_empty(
            node, "bbox_projection_empty"
        )

        self.assertNotEqual(templates[0], templates[1])

    def test_cloud_callback_publishes_valid_table_without_any_yolo_detection(self):
        node = bare_node(stamp_parts_to_ns(10, 0))
        node._workspace_frame = "ur_arm_base_link"
        node._workspace_bounds = (-0.37, 0.21, 0.23, 0.54, 0.29, 0.40)
        node._ransac_config = object()
        table_poses = []
        node._table_surface_publisher = SimpleNamespace(publish=table_poses.append)
        tf_stamps = []
        node._lookup_arrays = lambda _target, _source, stamp: (
            tf_stamps.append(stamp) or np.eye(3),
            np.zeros(3),
        )
        points = np.array([[0.0, 0.3, 0.3], [0.1, 0.4, 0.35]], dtype=np.float32)
        plane_result = SimpleNamespace(
            applied=True,
            plane_valid=True,
            reason="accepted",
            plane_model=(0.0, 0.0, 1.0, -0.3),
            inlier_count=1,
            inlier_ratio=0.5,
            table_height=0.3,
            normal_angle_deg=0.0,
            camera_cloud=FilteredCloud(points, np.zeros((2, 3)), 2, 2, 2),
        )
        ransac_calls = []
        originals = {
            name: getattr(NODE_MODULE, name)
            for name in ("parse_cloud_arrays", "select_workspace", "remove_table_plane")
        }
        NODE_MODULE.parse_cloud_arrays = lambda _: (points, np.zeros(2, dtype=np.uint32))
        NODE_MODULE.select_workspace = lambda *_: SimpleNamespace(
            camera_cloud=plane_result.camera_cloud,
            workspace_points=points,
        )
        NODE_MODULE.remove_table_plane = lambda *_: (
            ransac_calls.append(True) or plane_result
        )
        cloud = cloud_message(10, 0)
        try:
            node._cloud_callback(cloud)
        finally:
            for name, value in originals.items():
                setattr(NODE_MODULE, name, value)

        self.assertEqual(len(ransac_calls), 1)
        self.assertEqual(len(table_poses), 1)
        self.assertEqual(node._processed, [])
        self.assertEqual(node._processed_cloud_cache.count, 1)
        self.assertIs(tf_stamps[0], cloud.header.stamp)

    def test_30hz_clouds_are_preprocessed_at_configured_5hz_without_detection(self):
        now = [stamp_parts_to_ns(10, 0)]
        node = bare_node(now[0])
        node._clock_now_ns = lambda: now[0]
        preprocess_stamps = []
        node._preprocess_cloud = lambda message: (
            preprocess_stamps.append(
                stamp_parts_to_ns(message.header.stamp.secs, message.header.stamp.nsecs)
            )
            or SimpleNamespace(
                stamp_ns=stamp_parts_to_ns(
                    message.header.stamp.secs, message.header.stamp.nsecs
                ),
                source_frame=message.header.frame_id,
                message_identity=id(message),
                header=message.header,
                plane_result=SimpleNamespace(plane_valid=True, applied=True),
            )
        )
        node._publish_table_surface_pose = lambda *_: False

        for index in range(30):
            nsecs = index * 33333333
            now[0] = stamp_parts_to_ns(10, nsecs)
            node._cloud_callback(cloud_message(10, nsecs))

        self.assertEqual(len(preprocess_stamps), 5)

    def test_detection_never_reuses_preprocess_result_from_another_cloud_stamp(self):
        now_ns = stamp_parts_to_ns(11, 100000000)
        node = bare_node(now_ns)
        current_cloud = cloud_message(11, 0)
        node._cloud_cache.add(stamp_parts_to_ns(11, 0), current_cloud)
        node._processed_cloud_cache.add(
            stamp_parts_to_ns(10, 0),
            NODE_MODULE.PreprocessedCloud(
                stamp_ns=stamp_parts_to_ns(10, 0),
                source_frame="d405_depth_optical_frame",
                message_identity=12345,
                header=cloud_message(10, 0).header,
                workspace_selection=object(),
                plane_result=SimpleNamespace(plane_valid=True, applied=True),
            ),
        )
        fresh = SimpleNamespace(
            stamp_ns=stamp_parts_to_ns(11, 0),
            source_frame=current_cloud.header.frame_id,
            message_identity=id(current_cloud),
            header=current_cloud.header,
            workspace_selection=object(),
            plane_result=SimpleNamespace(plane_valid=True, applied=True),
        )
        calls = []
        node._preprocess_cloud = lambda message: calls.append(message) or fresh
        node._publish_table_surface_pose = lambda *_: True

        node._detection_callback(detection_message(11, 0))

        self.assertEqual(calls, [current_cloud])
        self.assertEqual(len(node._processed), 1)
        self.assertIs(node._processed[0][2], fresh)
        self.assertEqual(node._empty_reasons, [])

    def test_accepted_plane_result_publishes_upward_surface_pose_at_source_stamp(self):
        node = NODE_MODULE.YoloTargetCloudNode.__new__(NODE_MODULE.YoloTargetCloudNode)
        node._workspace_frame = "ur_arm_base_link"
        node._workspace_bounds = (-0.37, 0.21, 0.23, 0.54, 0.29, 0.40)
        published = []
        node._table_surface_publisher = SimpleNamespace(publish=published.append)
        self.assertTrue(
            hasattr(node, "_publish_table_surface_pose"),
            "table surface publisher implementation is missing",
        )
        header = cloud_message(12, 345).header
        result = SimpleNamespace(
            applied=True,
            plane_valid=True,
            plane_model=(0.0, 0.0, -2.0, 0.6),
        )

        accepted = node._publish_table_surface_pose(result, header)

        self.assertTrue(accepted)
        self.assertEqual(len(published), 1)
        pose = published[0]
        self.assertEqual(pose.header.frame_id, "ur_arm_base_link")
        self.assertIs(pose.header.stamp, header.stamp)
        self.assertAlmostEqual(pose.pose.position.x, -0.08)
        self.assertAlmostEqual(pose.pose.position.y, 0.385)
        self.assertAlmostEqual(pose.pose.position.z, 0.3)
        self.assertAlmostEqual(pose.pose.orientation.x, 0.0)
        self.assertAlmostEqual(pose.pose.orientation.y, 0.0)
        self.assertAlmostEqual(pose.pose.orientation.z, 0.0)
        self.assertAlmostEqual(pose.pose.orientation.w, 1.0)

    def test_failed_plane_result_does_not_publish_a_removal_or_replacement(self):
        node = NODE_MODULE.YoloTargetCloudNode.__new__(NODE_MODULE.YoloTargetCloudNode)
        published = []
        node._table_surface_publisher = SimpleNamespace(publish=published.append)
        self.assertTrue(
            hasattr(node, "_publish_table_surface_pose"),
            "table surface publisher implementation is missing",
        )

        accepted = node._publish_table_surface_pose(
            SimpleNamespace(applied=False, plane_valid=False, plane_model=None),
            cloud_message().header,
        )

        self.assertFalse(accepted)
        self.assertEqual(published, [])

    def test_valid_table_plane_publishes_when_target_removal_is_not_applied(self):
        node = NODE_MODULE.YoloTargetCloudNode.__new__(NODE_MODULE.YoloTargetCloudNode)
        node._workspace_frame = "ur_arm_base_link"
        node._workspace_bounds = (-0.37, 0.21, 0.23, 0.54, 0.29, 0.40)
        published = []
        node._table_surface_publisher = SimpleNamespace(publish=published.append)
        result = SimpleNamespace(
            applied=False,
            plane_valid=True,
            reason="too_few_object_points",
            plane_model=(0.0, 0.0, 1.0, -0.3),
        )

        accepted = node._publish_table_surface_pose(result, cloud_message().header)

        self.assertTrue(accepted)
        self.assertEqual(len(published), 1)

    def test_exact_preprocessed_cloud_is_reused_by_matching_detection(self):
        now_ns = stamp_parts_to_ns(10, 0)
        node = bare_node(now_ns)
        preprocessing_calls = []
        plane_result = SimpleNamespace(
            applied=True,
            plane_valid=True,
            reason="accepted",
            plane_model=(0.0, 0.0, 1.0, -0.3),
        )

        def preprocess(message):
            preprocessing_calls.append(message.header.stamp)
            return NODE_MODULE.PreprocessedCloud(
                stamp_ns=now_ns,
                source_frame=message.header.frame_id,
                message_identity=id(message),
                header=message.header,
                workspace_selection=object(),
                plane_result=plane_result,
            )

        node._preprocess_cloud = preprocess
        node._publish_table_surface_pose = lambda *_: True
        node._cloud_callback(cloud_message(10, 0))
        node._detection_callback(detection_message(10, 0))

        self.assertEqual(len(preprocessing_calls), 1)
        self.assertEqual(len(node._processed), 1)
        self.assertEqual(node._processed[0][2].stamp_ns, now_ns)

    def test_target_processing_reuses_exact_preprocess_result_without_ransac(self):
        node = NODE_MODULE.YoloTargetCloudNode.__new__(NODE_MODULE.YoloTargetCloudNode)
        node._workspace_frame = "ur_arm_base_link"
        node._workspace_bounds = (-0.37, 0.21, 0.23, 0.54, 0.29, 0.40)
        node._camera_model = object()
        node._state_lock = NODE_MODULE.threading.Lock()
        node._ransac_config = object()
        node._require_ransac_success = True
        node._color_frame = "d405_color_optical_frame"
        node._log_throttle_sec = 1.0
        node._lookup_arrays = lambda *_: (np.eye(3), np.zeros(3))
        target_clouds = []
        node._publisher = SimpleNamespace(publish=target_clouds.append)
        node._create_cloud = lambda points, colors, header: (points, colors, header)
        node._mark_target_published = lambda *_args, **_kwargs: None
        node._publish_empty = lambda *_: self.fail("valid plane unexpectedly rejected")
        node._log_target_empty = lambda *_args, **_kwargs: self.fail(
            "valid target unexpectedly diagnosed as empty"
        )

        points = np.array([[0.0, 0.3, 0.3], [0.1, 0.4, 0.35]], dtype=np.float32)
        colors = np.zeros((2, 3), dtype=np.float32)
        plane_result = SimpleNamespace(
            applied=True,
            plane_valid=True,
            reason="accepted",
            plane_model=(0.0, 0.0, 1.0, -0.3),
            camera_cloud=FilteredCloud(points, colors, 2, 2, 2),
        )
        originals = {
            name: getattr(NODE_MODULE, name)
            for name in ("remove_table_plane", "select_bbox_points")
        }
        NODE_MODULE.remove_table_plane = lambda *_: self.fail(
            "target processing repeated RANSAC"
        )
        NODE_MODULE.select_bbox_points = lambda *_: SimpleNamespace(
            points=points,
            colors=colors,
            projected_count=2,
            target_count=2,
        )
        old_log = NODE_MODULE.rospy.loginfo_throttle
        NODE_MODULE.rospy.loginfo_throttle = lambda *_args, **_kwargs: None
        detection = SimpleNamespace(
            class_name="box",
            confidence=0.9,
            bbox=SimpleNamespace(xmin=1, ymin=2, xmax=3, ymax=4),
            stamp_ns=10_000_000_000,
        )
        match = SimpleNamespace(
            message=cloud_message(10, 0),
            stamp_ns=10_000_000_000,
            delta_ns=0,
        )
        preprocessed = SimpleNamespace(
            stamp_ns=match.stamp_ns,
            header=match.message.header,
            workspace_selection=SimpleNamespace(
                camera_cloud=plane_result.camera_cloud,
                workspace_points=points,
            ),
            plane_result=plane_result,
        )
        try:
            node._process_match(detection, match, preprocessed)
        finally:
            for name, value in originals.items():
                setattr(NODE_MODULE, name, value)
            NODE_MODULE.rospy.loginfo_throttle = old_log

        self.assertEqual(len(target_clouds), 1)

    def test_valid_plane_but_insufficient_object_points_has_distinct_empty_reason(self):
        node = NODE_MODULE.YoloTargetCloudNode.__new__(NODE_MODULE.YoloTargetCloudNode)
        node._camera_model = object()
        node._state_lock = NODE_MODULE.threading.Lock()
        node._require_ransac_success = True
        node._color_frame = "d405_color_optical_frame"
        reasons = []
        empty_headers = []
        node._log_target_empty = lambda reason, **_fields: reasons.append(reason)
        node._publish_empty = lambda header: empty_headers.append(header)
        node._mark_target_published = lambda *_args, **_kwargs: None
        plane_result = SimpleNamespace(
            applied=False,
            plane_valid=True,
            reason="too_few_object_points",
            plane_model=(0.0, 0.0, 1.0, -0.3),
            inlier_count=100,
            inlier_ratio=0.9,
            table_height=0.3,
            normal_angle_deg=0.2,
            camera_cloud=FilteredCloud(
                np.zeros((102, 3), dtype=np.float32),
                np.zeros((102, 3), dtype=np.float32),
                102,
                102,
                102,
            ),
        )
        match = SimpleNamespace(
            message=cloud_message(10, 0),
            stamp_ns=stamp_parts_to_ns(10, 0),
            delta_ns=0,
        )
        preprocessed = SimpleNamespace(
            stamp_ns=match.stamp_ns,
            header=match.message.header,
            workspace_selection=SimpleNamespace(
                camera_cloud=plane_result.camera_cloud,
                workspace_points=np.zeros((102, 3), dtype=np.float32),
            ),
            plane_result=plane_result,
        )
        detection = SimpleNamespace(stamp_ns=match.stamp_ns)

        node._process_match(detection, match, preprocessed)

        self.assertEqual(reasons, ["insufficient_target_points"])
        self.assertEqual(empty_headers, [match.message.header])

    def test_cloud_callback_always_caches_when_rate_gate_skips_preprocessing(self):
        node = bare_node(stamp_parts_to_ns(10, 100000000))
        node._last_preprocess_started_ns = stamp_parts_to_ns(10, 100000000)
        node._preprocess_cloud = lambda *_: self.fail(
            "rate-limited cloud ran preprocessing"
        )
        cloud = cloud_message(10, 0)

        node._cloud_callback(cloud)

        self.assertEqual(node._cloud_cache.count, 1)
        self.assertIs(node._cloud_cache.latest().message, cloud)

    def test_duplicate_detection_stamp_runs_processing_once(self):
        node = bare_node(stamp_parts_to_ns(10, 100000000))
        node._cloud_callback(cloud_message(10, 0))
        node._get_or_preprocess = lambda message, stamp_ns, *_args: (
            NODE_MODULE.PreprocessOutcome(
                "cache_hit",
                SimpleNamespace(
                    stamp_ns=stamp_ns,
                    header=message.header,
                ),
            )
        )
        message = detection_message(10, 0)

        node._detection_callback(message)
        node._detection_callback(message)

        self.assertEqual(len(node._processed), 1)

    def test_detection_age_rejects_old_detection_even_with_exact_cloud_stamp(self):
        node = bare_node(stamp_parts_to_ns(10, 600000000))
        node._cloud_callback(cloud_message(10, 0))

        node._detection_callback(detection_message(10, 0))

        self.assertEqual(node._processed, [])

    def test_stamp_delta_rejects_fresh_detection_with_21_ms_cloud_gap(self):
        node = bare_node(stamp_parts_to_ns(10, 100000000))
        node._cloud_callback(cloud_message(10, 21000000))

        node._detection_callback(detection_message(10, 0))

        self.assertEqual(node._processed, [])

    def test_single_stamp_miss_does_not_clear_still_fresh_target(self):
        node = bare_node(stamp_parts_to_ns(10, 100000000))
        previous = cloud_message(9, 900000000)
        node._cloud_callback(previous)
        node._target_visible = True
        node._active_detection_stamp_ns = stamp_parts_to_ns(9, 800000000)
        node._last_target_activity_ns = stamp_parts_to_ns(10, 0)

        node._detection_callback(detection_message(10, 0))

        self.assertEqual(node._processed, [])
        self.assertEqual(node._published_empty_headers, [])
        self.assertTrue(node._target_visible)

    def test_stale_timer_clears_visible_target_once_with_real_cloud_header(self):
        node = bare_node(stamp_parts_to_ns(10, 600000000))
        newest = cloud_message(10, 500000000)
        node._cloud_callback(newest)
        node._target_visible = True
        node._active_detection_stamp_ns = stamp_parts_to_ns(10, 0)
        node._last_target_activity_ns = stamp_parts_to_ns(10, 0)

        node._stale_timer_callback(None)
        node._stale_timer_callback(None)

        self.assertEqual(node._published_empty_headers, [newest.header])
        self.assertFalse(node._target_visible)

    def test_stale_timer_does_not_clear_while_detection_is_processing(self):
        node = bare_node(stamp_parts_to_ns(10, 600000000))
        newest = cloud_message(10, 500000000)
        node._cloud_callback(newest)
        node._target_visible = True
        node._active_detection_stamp_ns = stamp_parts_to_ns(10, 0)
        node._last_target_activity_ns = stamp_parts_to_ns(10, 0)
        node._processing = True

        node._stale_timer_callback(None)

        self.assertEqual(node._published_empty_headers, [])
        self.assertTrue(node._target_visible)

    def test_successful_publish_refreshes_watchdog_after_processing_latency(self):
        now = [stamp_parts_to_ns(10, 600000000)]
        node = bare_node(now[0])
        node._clock_now_ns = lambda: now[0]
        newest = cloud_message(10, 500000000)
        node._cloud_callback(newest)

        node._mark_target_published(stamp_parts_to_ns(10, 0), target_count=100)
        now[0] = stamp_parts_to_ns(11, 0)
        node._stale_timer_callback(None)

        self.assertEqual(node._published_empty_headers, [])
        self.assertTrue(node._target_visible)


if __name__ == "__main__":
    unittest.main()
