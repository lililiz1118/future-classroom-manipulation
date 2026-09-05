#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))
sys.path.insert(0, str(WORKSPACE_SRC / "anygrasp_ros" / "src"))

try:
    from tracer_bringup.table_collision import (
        StableTableTracker,
        TablePoseSample,
        build_collision_box,
    )
except ImportError:
    StableTableTracker = None
    TablePoseSample = None
    build_collision_box = None


ROI_XY = (-0.37, 0.21, 0.23, 0.54)


def sample(stamp, height=0.3, normal=(0.0, 0.0, 1.0), frame="ur_arm_base_link"):
    return TablePoseSample(
        frame_id=frame,
        stamp_sec=float(stamp),
        normal=np.asarray(normal, dtype=np.float64),
        height_at_roi_center=float(height),
    )


class TableCollisionGeometryTest(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(build_collision_box, "table collision implementation is missing")

    def test_horizontal_box_has_roi_size_plus_margin_and_extends_downward(self):
        box = build_collision_box(
            normal=(0.0, 0.0, 1.0),
            height_at_roi_center=0.3,
            roi_xy=ROI_XY,
            thickness=0.05,
            xy_margin=0.025,
        )

        np.testing.assert_allclose(box.size, [0.63, 0.36, 0.05], atol=1e-12)
        np.testing.assert_allclose(box.surface_center, [-0.08, 0.385, 0.3])
        np.testing.assert_allclose(box.center, [-0.08, 0.385, 0.275])

    def test_tilted_box_top_face_lies_on_plane_and_covers_all_roi_corners(self):
        normal = np.array([0.08, -0.05, 0.995536], dtype=np.float64)
        normal /= np.linalg.norm(normal)
        box = build_collision_box(
            normal=normal,
            height_at_roi_center=0.3,
            roi_xy=ROI_XY,
            thickness=0.05,
            xy_margin=0.025,
        )

        top_center = box.center + box.rotation[:, 2] * box.size[2] / 2.0
        np.testing.assert_allclose(top_center, box.surface_center, atol=1e-12)
        plane_d = -float(np.dot(normal, box.surface_center))
        top_residuals = box.top_corners @ normal + plane_d
        np.testing.assert_allclose(top_residuals, np.zeros(4), atol=1e-12)

        local_roi = (box.roi_corners - box.surface_center) @ box.rotation[:, :2]
        half_extents = box.size[:2] / 2.0
        self.assertTrue(np.all(np.abs(local_roi) <= half_extents + 1e-12))
        self.assertTrue(np.all(box.center == box.surface_center - normal * 0.025))


class StableTableTrackerTest(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(StableTableTracker, "stable table tracker is missing")
        self.tracker = StableTableTracker(
            expected_frame="ur_arm_base_link",
            stable_plane_frames=5,
            max_height_variation=0.005,
            max_normal_angle_deg=2.0,
            update_height_threshold=0.004,
            update_angle_threshold_deg=1.5,
            max_table_plane_age=1.0,
        )

    def test_requires_five_stable_samples_before_first_update(self):
        for index, height in enumerate((0.300, 0.301, 0.299, 0.302), start=1):
            decision = self.tracker.observe(sample(index, height), now_sec=index + 0.1)
            self.assertFalse(decision.update_scene)

        decision = self.tracker.observe(sample(5, 0.300), now_sec=5.1)

        self.assertTrue(decision.update_scene)
        self.assertAlmostEqual(decision.stable_model.height_at_roi_center, 0.300)
        self.assertIsNone(self.tracker.scene_model)
        self.tracker.confirm_scene_update(decision.stable_model)
        self.assertEqual(self.tracker.scene_model.stamp_sec, 5.0)

    def test_height_or_normal_outlier_prevents_update(self):
        heights = (0.300, 0.301, 0.312, 0.299, 0.300)
        for index, height in enumerate(heights, start=1):
            height_decision = self.tracker.observe(
                sample(index, height), now_sec=index + 0.1
            )
        self.assertFalse(height_decision.update_scene)
        self.assertEqual(height_decision.reason, "unstable_height")

        angle_tracker = StableTableTracker(
            expected_frame="ur_arm_base_link",
            stable_plane_frames=5,
            max_height_variation=0.005,
            max_normal_angle_deg=2.0,
            update_height_threshold=0.004,
            update_angle_threshold_deg=1.5,
            max_table_plane_age=1.0,
        )
        tilted = (np.sin(np.deg2rad(4.0)), 0.0, np.cos(np.deg2rad(4.0)))
        normals = ((0.0, 0.0, 1.0),) * 2 + (tilted,) + ((0.0, 0.0, 1.0),) * 2
        for index, normal in enumerate(normals, start=1):
            normal_decision = angle_tracker.observe(
                sample(index, normal=normal), now_sec=index + 0.1
            )
        self.assertFalse(normal_decision.update_scene)
        self.assertEqual(normal_decision.reason, "unstable_normal")

    def test_small_stable_change_refreshes_freshness_without_scene_update(self):
        for index in range(1, 6):
            first = self.tracker.observe(sample(index), now_sec=index + 0.1)
        self.assertTrue(first.update_scene)
        self.assertTrue(
            hasattr(self.tracker, "confirm_scene_update"),
            "scene update confirmation is missing",
        )
        self.tracker.confirm_scene_update(first.stable_model)

        for index in range(6, 11):
            second = self.tracker.observe(
                sample(index, height=0.302), now_sec=index + 0.1
            )

        self.assertFalse(second.update_scene)
        self.assertEqual(second.reason, "below_update_threshold")
        self.assertEqual(self.tracker.latest_stable_model.stamp_sec, 10.0)
        self.assertEqual(self.tracker.scene_model.stamp_sec, 5.0)
        self.assertEqual(self.tracker.status(now_sec=10.5).state, "fresh")

    def test_stale_wrong_frame_and_out_of_order_samples_never_replace_scene_model(self):
        for index in range(1, 6):
            accepted = self.tracker.observe(sample(index), now_sec=index + 0.1)
        self.assertTrue(
            hasattr(self.tracker, "confirm_scene_update"),
            "scene update confirmation is missing",
        )
        self.tracker.confirm_scene_update(accepted.stable_model)
        original = self.tracker.scene_model

        stale = self.tracker.observe(sample(20, height=0.35), now_sec=22.0)
        wrong_frame = self.tracker.observe(
            sample(21, height=0.35, frame="camera"), now_sec=21.1
        )
        out_of_order = self.tracker.observe(sample(4.5, height=0.35), now_sec=5.0)

        self.assertFalse(stale.update_scene)
        self.assertEqual(stale.reason, "stale")
        self.assertFalse(wrong_frame.update_scene)
        self.assertEqual(wrong_frame.reason, "wrong_frame")
        self.assertFalse(out_of_order.update_scene)
        self.assertEqual(out_of_order.reason, "out_of_order")
        self.assertIs(self.tracker.scene_model, original)
        self.assertEqual(self.tracker.status(now_sec=7.0).state, "stale")
        self.assertTrue(self.tracker.status(now_sec=7.0).collision_present)

    def test_unconfirmed_scene_update_is_retried_on_next_stable_sample(self):
        for index in range(1, 6):
            first = self.tracker.observe(sample(index), now_sec=index + 0.1)
        self.assertTrue(first.update_scene)
        self.assertIsNone(self.tracker.scene_model)

        retry = self.tracker.observe(sample(6), now_sec=6.1)

        self.assertTrue(retry.update_scene)
        self.assertEqual(retry.reason, "first_stable_model")
        self.assertIsNone(self.tracker.scene_model)

    def test_long_input_gap_requires_a_new_complete_stability_window(self):
        for index in range(1, 5):
            decision = self.tracker.observe(sample(index), now_sec=index + 0.1)
            self.assertFalse(decision.update_scene)

        after_gap = self.tracker.observe(sample(20), now_sec=20.1)

        self.assertFalse(after_gap.update_scene)
        self.assertEqual(after_gap.reason, "collecting")
        for index in range(21, 25):
            decision = self.tracker.observe(sample(index), now_sec=index + 0.1)
        self.assertTrue(decision.update_scene)
        self.assertEqual(decision.stable_model.stamp_sec, 24.0)


if __name__ == "__main__":
    unittest.main()
