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
        filter_workspace,
        grasp_axes,
        rotation_matrix_to_quaternion,
    )
except ModuleNotFoundError as exc:
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
class WorkspaceFilterTest(unittest.TestCase):
    def test_invalid_points_are_removed_before_inclusive_workspace_crop(self):
        points = np.array(
            [
                [0.0, 0.0, 0.5],
                [np.nan, 0.0, 0.5],
                [0.6, 0.0, 0.5],
                [0.0, 0.0, 2.0],
                [-0.5, 0.5, 0.1],
            ],
            dtype=np.float32,
        )
        colors = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [1.0, 1.0, 0.0],
                [0.25, 0.5, 0.75],
            ],
            dtype=np.float32,
        )

        result = filter_workspace(points, colors, (-0.5, 0.5, -0.5, 0.5, 0.1, 1.5))

        self.assertEqual(result.raw_count, 5)
        self.assertEqual(result.valid_count, 4)
        self.assertEqual(result.workspace_count, 2)
        np.testing.assert_allclose(result.points, points[[0, 4]])
        np.testing.assert_allclose(result.colors, colors[[0, 4]])

    def test_point_color_length_mismatch_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "same number"):
            filter_workspace(
                np.zeros((2, 3), dtype=np.float32),
                np.zeros((1, 3), dtype=np.float32),
                (-0.5, 0.5, -0.5, 0.5, 0.1, 1.5),
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
