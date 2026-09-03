import json
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = PACKAGE_ROOT / "src"
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))

from yolo_world_ros.target_cloud import (  # noqa: E402
    BoundingBox,
    CameraModel,
    TimedCloudCache,
    pack_colors_to_float32,
    parse_cloud_arrays,
    parse_detection_json,
    project_points,
    select_bbox_points,
    stamp_parts_to_ns,
)


class DetectionParsingTest(unittest.TestCase):
    def test_parses_current_yolo_json_contract_without_changing_stamp(self):
        payload = json.dumps(
            {
                "header": {
                    "seq": 31,
                    "stamp": {"secs": 1788437956, "nsecs": 625617743},
                    "frame_id": "d405_color_optical_frame",
                },
                "bbox": {"xmin": 539, "ymin": 84, "xmax": 800, "ymax": 341},
                "center": {"u": 669, "v": 212},
                "class_name": "box",
                "confidence": 0.7802,
            }
        )

        detection = parse_detection_json(payload)

        self.assertEqual(detection.stamp_ns, 1788437956625617743)
        self.assertEqual(detection.frame_id, "d405_color_optical_frame")
        self.assertEqual(detection.class_name, "box")
        self.assertAlmostEqual(detection.confidence, 0.7802)
        self.assertEqual(
            (detection.bbox.xmin, detection.bbox.ymin,
             detection.bbox.xmax, detection.bbox.ymax),
            (539, 84, 800, 341),
        )

    def test_rejects_bbox_with_reversed_edges(self):
        payload = json.dumps(
            {
                "header": {
                    "stamp": {"secs": 10, "nsecs": 0},
                    "frame_id": "d405_color_optical_frame",
                },
                "bbox": {"xmin": 20, "ymin": 5, "xmax": 10, "ymax": 30},
                "class_name": "box",
                "confidence": 0.8,
            }
        )

        with self.assertRaisesRegex(ValueError, "bbox"):
            parse_detection_json(payload)

    def test_rejects_nan_confidence(self):
        payload = json.dumps(
            {
                "header": {
                    "stamp": {"secs": 10, "nsecs": 0},
                    "frame_id": "d405_color_optical_frame",
                },
                "bbox": {"xmin": 1, "ymin": 2, "xmax": 3, "ymax": 4},
                "class_name": "box",
                "confidence": float("nan"),
            }
        )

        with self.assertRaisesRegex(ValueError, "confidence"):
            parse_detection_json(payload)


class StampConversionTest(unittest.TestCase):
    def test_preserves_nanoseconds_at_large_epoch(self):
        self.assertEqual(
            stamp_parts_to_ns(1788437956, 625617743),
            1788437956625617743,
        )

    def test_rejects_nsec_outside_ros_range(self):
        with self.assertRaisesRegex(ValueError, "nsecs"):
            stamp_parts_to_ns(10, 1000000000)


class TimedCloudCacheTest(unittest.TestCase):
    def test_prunes_by_time_duration_instead_of_frame_count(self):
        cache = TimedCloudCache(duration_sec=1.0)
        cache.add(stamp_parts_to_ns(10, 0), "old")
        cache.add(stamp_parts_to_ns(10, 500000000), "middle")
        cache.add(stamp_parts_to_ns(11, 100000000), "new")

        self.assertEqual(cache.count, 2)
        self.assertEqual(cache.latest().message, "new")

    def test_nearest_accepts_19_ms_and_reports_delta(self):
        cache = TimedCloudCache(duration_sec=1.0)
        cache.add(stamp_parts_to_ns(10, 19000000), "cloud")

        match = cache.nearest(stamp_parts_to_ns(10, 0), max_delta_sec=0.02)

        self.assertIsNotNone(match)
        self.assertEqual(match.message, "cloud")
        self.assertEqual(match.delta_ns, 19000000)

    def test_nearest_rejects_21_ms(self):
        cache = TimedCloudCache(duration_sec=1.0)
        cache.add(stamp_parts_to_ns(10, 21000000), "adjacent-cloud")

        match = cache.nearest(stamp_parts_to_ns(10, 0), max_delta_sec=0.02)

        self.assertIsNone(match)

    def test_out_of_order_old_sample_is_pruned_against_newest_stamp(self):
        cache = TimedCloudCache(duration_sec=1.0)
        cache.add(stamp_parts_to_ns(11, 500000000), "newest")
        cache.add(stamp_parts_to_ns(9, 0), "late-old")

        self.assertEqual(cache.count, 1)
        self.assertEqual(cache.latest().message, "newest")


class ProjectionTest(unittest.TestCase):
    def test_zero_distortion_uses_pinhole_projection(self):
        camera = CameraModel(
            width=640,
            height=480,
            k=(100.0, 0.0, 320.0, 0.0, 100.0, 240.0, 0.0, 0.0, 1.0),
            d=(0.0, 0.0, 0.0, 0.0, 0.0),
            distortion_model="plumb_bob",
        )

        pixels, valid = project_points(
            np.array([[1.0, 2.0, 2.0]], dtype=np.float32), camera
        )

        np.testing.assert_allclose(pixels, [[370.0, 340.0]], atol=1e-6)
        np.testing.assert_array_equal(valid, [True])

    def test_plumb_bob_distortion_matches_hand_calculated_pixel(self):
        camera = CameraModel(
            width=100,
            height=100,
            k=(100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 1.0),
            d=(0.1, 0.01, 0.001, 0.002, 0.0),
            distortion_model="plumb_bob",
        )

        pixels, valid = project_points(
            np.array([[0.1, 0.2, 1.0]], dtype=np.float32), camera
        )

        np.testing.assert_allclose(pixels, [[10.06825, 20.1215]], atol=1e-5)
        np.testing.assert_array_equal(valid, [True])

    def test_rejects_nonfinite_nonpositive_depth_and_out_of_image_points(self):
        camera = CameraModel(
            width=640,
            height=480,
            k=(100.0, 0.0, 320.0, 0.0, 100.0, 240.0, 0.0, 0.0, 1.0),
            d=(),
            distortion_model="plumb_bob",
        )
        points = np.array(
            [
                [0.0, 0.0, 1.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
                [np.nan, 0.0, 1.0],
                [10.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

        _, valid = project_points(points, camera)

        np.testing.assert_array_equal(valid, [True, False, False, False, False])

    def test_bbox_selection_is_inclusive_and_preserves_source_xyz_rgb_alignment(self):
        camera = CameraModel(
            width=640,
            height=480,
            k=(80.0, 0.0, 320.0, 0.0, 80.0, 240.0, 0.0, 0.0, 1.0),
            d=(),
            distortion_model="plumb_bob",
        )
        source_points = np.array(
            [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
            dtype=np.float32,
        )
        colors = np.array(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        color_points = np.array(
            [[0.0, 0.0, 1.0], [0.125, 0.125, 1.0], [0.25, 0.0, 1.0]],
            dtype=np.float32,
        )

        selection = select_bbox_points(
            source_points,
            colors,
            color_points,
            camera,
            BoundingBox(320, 240, 330, 250),
        )

        np.testing.assert_allclose(selection.points, source_points[:2])
        np.testing.assert_allclose(selection.colors, colors[:2])
        self.assertEqual(selection.projected_count, 3)
        self.assertEqual(selection.target_count, 2)


class PointCloudParsingTest(unittest.TestCase):
    def test_parses_two_padded_rows_and_preserves_packed_rgb_bits(self):
        fields = [
            SimpleNamespace(name="x", offset=0, datatype=7, count=1),
            SimpleNamespace(name="y", offset=4, datatype=7, count=1),
            SimpleNamespace(name="z", offset=8, datatype=7, count=1),
            SimpleNamespace(name="rgb", offset=12, datatype=7, count=1),
        ]
        row_one = (
            struct.pack("<fffI", 1.0, 2.0, 3.0, 0x00112233)
            + struct.pack("<fffI", 4.0, 5.0, 6.0, 0x00445566)
            + b"PAD!"
        )
        row_two = (
            struct.pack("<fffI", 7.0, 8.0, 9.0, 0x00778899)
            + struct.pack("<fffI", 10.0, 11.0, 12.0, 0x00AABBCC)
            + b"ROW!"
        )
        message = SimpleNamespace(
            fields=fields,
            data=row_one + row_two,
            width=2,
            height=2,
            point_step=16,
            row_step=36,
            is_bigendian=False,
        )

        points, packed_rgb = parse_cloud_arrays(message)

        np.testing.assert_allclose(
            points,
            [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0],
             [7.0, 8.0, 9.0], [10.0, 11.0, 12.0]],
        )
        np.testing.assert_array_equal(
            packed_rgb,
            np.array([0x00112233, 0x00445566, 0x00778899, 0x00AABBCC], dtype=np.uint32),
        )

    def test_packs_normalized_rgb_into_ros_float32_bit_pattern(self):
        packed = pack_colors_to_float32(
            np.array([[1.0, 0.5, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
        )

        np.testing.assert_array_equal(
            packed.view(np.uint32),
            np.array([0x00FF8000, 0x000000FF], dtype=np.uint32),
        )


if __name__ == "__main__":
    unittest.main()
