#!/home/jt001/.conda/envs/anygrasp/bin/python

import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

try:
    from anygrasp_ros.core import (  # noqa: E402
        decode_packed_rgb,
        dynamic_point_bounds,
        grasp_axes,
        rotation_matrix_to_quaternion,
        select_finite_cloud,
        select_workspace,
        transform_points,
    )
except (ImportError, ModuleNotFoundError) as exc:
    CORE_IMPORT_ERROR = exc
else:
    CORE_IMPORT_ERROR = None


class CoreModuleContractTest(unittest.TestCase):
    def test_core_module_is_available(self):
        self.assertIsNone(CORE_IMPORT_ERROR, str(CORE_IMPORT_ERROR))


@unittest.skipIf(CORE_IMPORT_ERROR is not None, "core module not implemented")
class DecodePackedRgbTest(unittest.TestCase):
    def test_float32_packed_rgb_decodes_to_normalized_channels(self):
        packed = np.array([0x00FF0000, 0x0000FF00, 0x000000FF], dtype=np.uint32)
        values = packed.view(np.float32)

        colors = decode_packed_rgb(values, datatype=7)

        np.testing.assert_allclose(
            colors,
            np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32),
        )
        self.assertEqual(colors.dtype, np.float32)

    def test_uint32_packed_rgb_decodes_without_numeric_cast(self):
        values = np.array([0x00102030], dtype=np.uint32)

        colors = decode_packed_rgb(values, datatype=6)

        np.testing.assert_allclose(
            colors,
            np.array([[16.0 / 255.0, 32.0 / 255.0, 48.0 / 255.0]], dtype=np.float32),
        )


@unittest.skipIf(CORE_IMPORT_ERROR is not None, "core module not implemented")
class FiniteCloudSelectionTest(unittest.TestCase):
    def test_nonfinite_xyz_is_removed_without_changing_camera_coordinates_or_rgb(self):
        points = np.array(
            [
                [0.10, 0.20, 0.30],
                [np.nan, 0.40, 0.50],
                [0.60, np.inf, 0.70],
                [-0.10, 0.00, 0.80],
            ],
            dtype=np.float32,
        )
        packed_rgb = np.array(
            [0x00FF0000, 0x0000FF00, 0x000000FF, 0x004080BF],
            dtype=np.uint32,
        )

        result = select_finite_cloud(points, packed_rgb)

        self.assertEqual(result.raw_count, 4)
        self.assertEqual(result.valid_count, 2)
        self.assertEqual(result.workspace_count, 2)
        np.testing.assert_array_equal(result.points, points[[0, 3]])
        np.testing.assert_allclose(
            result.colors,
            np.array(
                [[1.0, 0.0, 0.0], [64.0 / 255.0, 128.0 / 255.0, 191.0 / 255.0]],
                dtype=np.float32,
            ),
        )


@unittest.skipIf(CORE_IMPORT_ERROR is not None, "core module not implemented")
class PointTransformTest(unittest.TestCase):
    def test_translation_is_applied_to_all_points(self):
        points = np.array([[0.1, 0.2, 0.3], [-1.0, 0.0, 2.0]], dtype=np.float32)

        transformed = transform_points(
            points,
            rotation=np.eye(3),
            translation=np.array([1.0, 2.0, 3.0]),
        )

        np.testing.assert_allclose(
            transformed,
            [[1.1, 2.2, 3.3], [0.0, 2.0, 5.0]],
            atol=1e-6,
        )
        self.assertEqual(transformed.dtype, np.float32)

    def test_rotation_then_translation_uses_target_from_source_convention(self):
        rotation = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

        transformed = transform_points(
            np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 1.0]], dtype=np.float32),
            rotation=rotation,
            translation=np.array([10.0, 20.0, 30.0]),
        )

        np.testing.assert_allclose(
            transformed,
            [[10.0, 21.0, 30.0], [8.0, 20.0, 31.0]],
            atol=1e-6,
        )


@unittest.skipIf(CORE_IMPORT_ERROR is not None, "core module not implemented")
class WorkspaceSelectionTest(unittest.TestCase):
    def test_base_frame_mask_selects_original_camera_points_and_matching_rgb(self):
        camera_points = np.array(
            [
                [10.0, 0.0, 0.1],
                [20.0, 0.0, 0.2],
                [30.0, 0.0, 0.3],
                [40.0, 0.0, 0.4],
                [50.0, 0.0, 0.5],
            ],
            dtype=np.float32,
        )
        base_points = np.array(
            [
                [0.5, 0.5, 0.5],
                [1.1, 0.5, 0.5],
                [0.5, -0.1, 0.5],
                [0.5, 0.5, 1.1],
                [0.2, 0.3, 0.4],
            ],
            dtype=np.float32,
        )
        packed_rgb = np.array(
            [0x00FF0000, 0x0000FF00, 0x000000FF, 0x00FFFF00, 0x004080BF],
            dtype=np.uint32,
        )

        result = select_workspace(
            camera_points,
            base_points,
            packed_rgb,
            (0.0, 1.0, 0.0, 1.0, 0.0, 1.0),
        )

        self.assertEqual(result.camera_cloud.raw_count, 5)
        self.assertEqual(result.camera_cloud.valid_count, 5)
        self.assertEqual(result.camera_cloud.workspace_count, 2)
        np.testing.assert_array_equal(result.camera_cloud.points, camera_points[[0, 4]])
        np.testing.assert_array_equal(result.workspace_points, base_points[[0, 4]])
        np.testing.assert_allclose(
            result.camera_cloud.colors,
            np.array(
                [[1.0, 0.0, 0.0], [64.0 / 255.0, 128.0 / 255.0, 191.0 / 255.0]],
                dtype=np.float32,
            ),
        )

    def test_camera_base_and_rgb_length_mismatch_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "same number"):
            select_workspace(
                np.zeros((2, 3), dtype=np.float32),
                np.zeros((1, 3), dtype=np.float32),
                np.zeros(1, dtype=np.uint32),
                (-0.5, 0.5, -0.5, 0.5, 0.1, 1.5),
            )


@unittest.skipIf(CORE_IMPORT_ERROR is not None, "core module not implemented")
class DynamicPointBoundsTest(unittest.TestCase):
    def test_bounds_include_configured_margin_on_each_axis(self):
        points = np.array(
            [[-0.2, 1.0, 0.4], [0.3, 1.5, 0.9], [0.0, 1.2, 0.6]],
            dtype=np.float32,
        )

        bounds = dynamic_point_bounds(points, margin=0.02)

        np.testing.assert_allclose(
            bounds,
            (-0.22, 0.32, 0.98, 1.52, 0.38, 0.92),
            atol=1e-7,
        )


@unittest.skipIf(CORE_IMPORT_ERROR is not None, "core module not implemented")
class RotationConversionTest(unittest.TestCase):
    def test_identity_rotation_maps_to_ros_identity_quaternion(self):
        quaternion = rotation_matrix_to_quaternion(np.eye(3, dtype=np.float64))

        np.testing.assert_allclose(quaternion, [0.0, 0.0, 0.0, 1.0], atol=1e-7)

    def test_ninety_degree_z_rotation_uses_xyzw_order(self):
        rotation = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

        quaternion = rotation_matrix_to_quaternion(rotation)

        root_half = np.sqrt(0.5)
        np.testing.assert_allclose(quaternion, [0.0, 0.0, root_half, root_half], atol=1e-7)

    def test_grasp_axes_are_rotation_matrix_columns(self):
        rotation = np.array(
            [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            dtype=np.float64,
        )

        approach, opening, orthogonal = grasp_axes(rotation)

        np.testing.assert_array_equal(approach, [0.0, 1.0, 0.0])
        np.testing.assert_array_equal(opening, [0.0, 0.0, 1.0])
        np.testing.assert_array_equal(orthogonal, [1.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
