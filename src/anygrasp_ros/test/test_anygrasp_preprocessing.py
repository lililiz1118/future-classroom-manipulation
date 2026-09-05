#!/home/jt001/.conda/envs/anygrasp/bin/python

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from anygrasp_ros.core import FilteredCloud


def make_cloud(camera_points, colors=None):
    camera_points = np.asarray(camera_points, dtype=np.float32)
    if colors is None:
        colors = np.arange(camera_points.shape[0] * 3, dtype=np.float32).reshape(-1, 3)
    return FilteredCloud(
        points=camera_points,
        colors=np.asarray(colors, dtype=np.float32),
        raw_count=camera_points.shape[0] + 10,
        valid_count=camera_points.shape[0] + 5,
        workspace_count=camera_points.shape[0],
    )


def load_ransac_api(test_case):
    try:
        from anygrasp_ros.preprocessing import (
            RansacPlaneConfig,
            assess_table_plane,
            remove_table_plane,
        )
    except ImportError as exc:
        test_case.fail("RANSAC table preprocessing is not implemented: %s" % exc)
    return RansacPlaneConfig, assess_table_plane, remove_table_plane


def make_ransac_config(config_type, **overrides):
    values = {
        "enabled": True,
        "distance_threshold": 0.001,
        "ransac_n": 3,
        "num_iterations": 500,
        "min_points": 20,
        "max_normal_angle_deg": 10.0,
        "table_height_min": 0.20,
        "table_height_max": 0.28,
        "min_inliers": 20,
        "min_inlier_ratio": 0.25,
        "min_object_points": 5,
    }
    values.update(overrides)
    return config_type(**values)


class StatisticalOutlierFilterTest(unittest.TestCase):
    def test_removes_isolated_point_and_keeps_colors_aligned(self):
        try:
            from anygrasp_ros.preprocessing import (
                remove_statistical_outliers,
            )
        except ModuleNotFoundError as exc:
            self.fail(
                "statistical outlier preprocessing is not implemented: %s"
                % exc
            )

        grid = np.array(
            [
                [x * 0.005, y * 0.005, 0.25]
                for x in range(5)
                for y in range(5)
            ],
            dtype=np.float32,
        )
        points = np.vstack(
            [grid, np.array([[1.0, 1.0, 1.0]], dtype=np.float32)]
        )
        colors = np.zeros((26, 3), dtype=np.float32)
        colors[-1] = [1.0, 0.0, 0.0]

        cloud = FilteredCloud(
            points=points,
            colors=colors,
            raw_count=100,
            valid_count=90,
            workspace_count=26,
        )

        result = remove_statistical_outliers(
            cloud,
            nb_neighbors=5,
            std_ratio=1.0,
        )

        self.assertEqual(result.raw_count, 100)
        self.assertEqual(result.valid_count, 90)
        self.assertEqual(result.workspace_count, 25)
        np.testing.assert_allclose(
            result.colors,
            np.zeros((25, 3), dtype=np.float32),
        )


class RansacTablePlaneFilterTest(unittest.TestCase):
    def test_disabled_returns_original_roi_without_running_plane_removal(self):
        config_type, _, remove_table_plane = load_ransac_api(self)
        workspace_points = np.array(
            [[0.0, 0.0, 0.24], [0.1, 0.1, 0.25]], dtype=np.float32
        )
        cloud = make_cloud(workspace_points + [1.0, 2.0, 3.0])

        result = remove_table_plane(
            cloud,
            workspace_points,
            make_ransac_config(config_type, enabled=False),
        )

        self.assertFalse(result.applied)
        self.assertEqual(result.reason, "disabled")
        np.testing.assert_array_equal(result.camera_cloud.points, cloud.points)
        np.testing.assert_array_equal(result.camera_cloud.colors, cloud.colors)
        np.testing.assert_array_equal(result.workspace_points, workspace_points)

    def test_synthetic_horizontal_table_is_removed_with_xyz_rgb_alignment(self):
        config_type, _, remove_table_plane = load_ransac_api(self)
        table = np.array(
            [
                [x, y, 0.24]
                for x in np.linspace(-0.1, 0.1, 20)
                for y in np.linspace(0.30, 0.50, 20)
            ],
            dtype=np.float32,
        )
        random = np.random.RandomState(7)
        objects = np.column_stack(
            (
                random.uniform(-0.08, 0.08, 40),
                random.uniform(0.32, 0.48, 40),
                random.uniform(0.255, 0.275, 40),
            )
        ).astype(np.float32)
        workspace_points = np.vstack((table, objects))
        camera_points = workspace_points + np.array([0.4, -0.2, 0.7], dtype=np.float32)
        colors = np.arange(workspace_points.shape[0] * 3, dtype=np.float32).reshape(-1, 3)

        result = remove_table_plane(
            make_cloud(camera_points, colors),
            workspace_points,
            make_ransac_config(config_type),
        )

        self.assertTrue(result.applied)
        self.assertTrue(hasattr(result, "plane_valid"), "plane validity is not exposed")
        self.assertTrue(result.plane_valid)
        self.assertEqual(result.inlier_count, table.shape[0])
        self.assertAlmostEqual(result.inlier_ratio, 400.0 / 440.0, places=6)
        self.assertAlmostEqual(result.table_height, 0.24, places=5)
        np.testing.assert_allclose(result.workspace_points, objects)
        np.testing.assert_allclose(result.camera_cloud.points, camera_points[-40:])
        np.testing.assert_array_equal(result.camera_cloud.colors, colors[-40:])
        self.assertEqual(
            result.camera_cloud.points.shape[0], result.camera_cloud.colors.shape[0]
        )

    def test_table_height_band_prevents_larger_object_plane_from_hijacking_ransac(self):
        config_type, _, remove_table_plane = load_ransac_api(self)
        table = np.array(
            [
                [x, y, 0.300]
                for x in np.linspace(-0.10, 0.10, 20)
                for y in np.linspace(0.30, 0.50, 20)
            ],
            dtype=np.float32,
        )
        larger_object_plane = np.array(
            [
                [x, y, 0.365]
                for x in np.linspace(-0.08, 0.08, 40)
                for y in np.linspace(0.32, 0.48, 20)
            ],
            dtype=np.float32,
        )
        workspace_points = np.vstack((table, larger_object_plane))
        camera_points = workspace_points + np.array(
            [0.4, -0.2, 0.7], dtype=np.float32
        )

        result = remove_table_plane(
            make_cloud(camera_points),
            workspace_points,
            make_ransac_config(
                config_type,
                distance_threshold=0.002,
                table_height_min=0.29,
                table_height_max=0.31,
                min_inliers=300,
                min_object_points=700,
            ),
        )

        self.assertTrue(result.applied, result.reason)
        self.assertAlmostEqual(result.table_height, 0.300, places=3)
        self.assertEqual(result.camera_cloud.workspace_count, 800)
        np.testing.assert_allclose(result.workspace_points, larger_object_plane)

    def test_negative_z_plane_normal_is_accepted_and_height_uses_inlier_median(self):
        config_type, assess_table_plane, _ = load_ransac_api(self)
        workspace_points = np.array(
            [
                [0.0, 0.3, 0.239],
                [0.1, 0.3, 0.240],
                [0.0, 0.4, 0.241],
                [0.1, 0.4, 0.242],
            ],
            dtype=np.float32,
        )

        assessment = assess_table_plane(
            workspace_points,
            np.array([0.0, 0.0, -1.0, 99.0]),
            np.arange(4),
            make_ransac_config(
                config_type,
                min_points=4,
                min_inliers=4,
                min_inlier_ratio=1.0,
                min_object_points=0,
            ),
        )

        self.assertTrue(assessment.accepted)
        self.assertAlmostEqual(assessment.normal_angle_deg, 0.0, places=6)
        self.assertAlmostEqual(assessment.table_height, 0.2405, places=6)

    def test_dominant_vertical_plane_is_rejected_and_roi_is_used_as_fallback(self):
        config_type, _, remove_table_plane = load_ransac_api(self)
        wall = np.array(
            [
                [0.0, y, z]
                for y in np.linspace(0.30, 0.50, 15)
                for z in np.linspace(0.22, 0.27, 10)
            ],
            dtype=np.float32,
        )
        non_plane = np.array(
            [[0.02, 0.35, 0.23], [0.03, 0.42, 0.25], [0.04, 0.46, 0.26]],
            dtype=np.float32,
        )
        workspace_points = np.vstack((wall, non_plane))
        cloud = make_cloud(workspace_points + [0.5, 0.5, 0.5])

        result = remove_table_plane(
            cloud,
            workspace_points,
            make_ransac_config(config_type, min_object_points=1),
        )

        self.assertFalse(result.applied)
        self.assertEqual(result.reason, "normal_angle")
        self.assertTrue(hasattr(result, "plane_valid"), "plane validity is not exposed")
        self.assertFalse(result.plane_valid)
        self.assertIsNotNone(result.plane_model)
        self.assertIsNotNone(result.table_height)
        np.testing.assert_array_equal(result.camera_cloud.points, cloud.points)
        np.testing.assert_array_equal(result.workspace_points, workspace_points)

    def test_too_few_remaining_points_falls_back_to_original_roi(self):
        config_type, _, remove_table_plane = load_ransac_api(self)
        table = np.array(
            [
                [x, y, 0.24]
                for x in np.linspace(-0.1, 0.1, 10)
                for y in np.linspace(0.30, 0.50, 10)
            ],
            dtype=np.float32,
        )
        objects = np.array(
            [[0.01, 0.35, 0.26], [0.03, 0.42, 0.27]], dtype=np.float32
        )
        workspace_points = np.vstack((table, objects))
        cloud = make_cloud(workspace_points + [0.5, 0.5, 0.5])

        result = remove_table_plane(
            cloud,
            workspace_points,
            make_ransac_config(config_type, min_object_points=5),
        )

        self.assertFalse(result.applied)
        self.assertEqual(result.reason, "too_few_object_points")
        self.assertTrue(hasattr(result, "plane_valid"), "plane validity is not exposed")
        self.assertTrue(
            result.plane_valid,
            "a geometrically accepted table remains valid environment geometry",
        )
        np.testing.assert_array_equal(result.camera_cloud.points, cloud.points)
        np.testing.assert_array_equal(result.camera_cloud.colors, cloud.colors)
        np.testing.assert_array_equal(result.workspace_points, workspace_points)

    def test_too_few_roi_points_falls_back_without_exception(self):
        config_type, _, remove_table_plane = load_ransac_api(self)
        workspace_points = np.zeros((2, 3), dtype=np.float32)
        cloud = make_cloud(workspace_points)

        result = remove_table_plane(
            cloud,
            workspace_points,
            make_ransac_config(config_type, min_points=3),
        )

        self.assertFalse(result.applied)
        self.assertEqual(result.reason, "too_few_roi_points")
        np.testing.assert_array_equal(result.workspace_points, workspace_points)

    def test_open3d_exception_falls_back_to_original_roi(self):
        config_type, _, remove_table_plane = load_ransac_api(self)
        workspace_points = np.zeros((20, 3), dtype=np.float32)
        cloud = make_cloud(workspace_points)

        with patch(
            "anygrasp_ros.preprocessing.o3d.geometry.PointCloud",
            side_effect=RuntimeError("synthetic Open3D failure"),
        ):
            result = remove_table_plane(
                cloud,
                workspace_points,
                make_ransac_config(config_type),
            )

        self.assertFalse(result.applied)
        self.assertIn("ransac_error", result.reason)
        np.testing.assert_array_equal(result.camera_cloud.points, cloud.points)
        np.testing.assert_array_equal(result.workspace_points, workspace_points)

    def test_horizontal_plane_outside_temporary_height_window_is_rejected(self):
        config_type, assess_table_plane, _ = load_ransac_api(self)
        workspace_points = np.array(
            [[0.0, 0.3, 0.35], [0.1, 0.3, 0.35], [0.0, 0.4, 0.35]],
            dtype=np.float32,
        )

        assessment = assess_table_plane(
            workspace_points,
            np.array([0.0, 0.0, 1.0, -0.35]),
            np.arange(3),
            make_ransac_config(
                config_type,
                min_points=3,
                min_inliers=3,
                min_inlier_ratio=1.0,
                min_object_points=0,
            ),
        )

        self.assertFalse(assessment.accepted)
        self.assertEqual(assessment.reason, "table_height")
        self.assertAlmostEqual(assessment.table_height, 0.35, places=6)


if __name__ == "__main__":
    unittest.main()
