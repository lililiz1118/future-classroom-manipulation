import importlib.util
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


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
    node._process_match = lambda detection, match: node._processed.append(
        (detection, match)
    )
    node._published_empty_headers = []
    node._publish_empty = lambda header: node._published_empty_headers.append(header)
    node._logwarn = lambda *args: None
    return node


class DetectionDrivenRuntimeTest(unittest.TestCase):
    def test_cloud_callback_only_caches_and_never_processes(self):
        node = bare_node(stamp_parts_to_ns(10, 100000000))
        node._process_match = lambda *_: self.fail("cloud callback ran expensive processing")
        cloud = cloud_message(10, 0)

        node._cloud_callback(cloud)

        self.assertEqual(node._cloud_cache.count, 1)
        self.assertIs(node._cloud_cache.latest().message, cloud)

    def test_duplicate_detection_stamp_runs_processing_once(self):
        node = bare_node(stamp_parts_to_ns(10, 100000000))
        node._cloud_callback(cloud_message(10, 0))
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
