import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

try:
    from anygrasp_ros.table_geometry import table_surface_from_plane
except ImportError:
    table_surface_from_plane = None


ROI_XY = (-0.37, 0.21, 0.23, 0.54)


class TableSurfaceGeometryTest(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(
            table_surface_from_plane,
            "table_surface_from_plane implementation is missing",
        )

    def test_horizontal_plane_is_normalized_and_centered_at_roi_center(self):
        surface = table_surface_from_plane((0.0, 0.0, 2.0, -0.6), ROI_XY)

        np.testing.assert_allclose(surface.plane_model, [0.0, 0.0, 1.0, -0.3])
        np.testing.assert_allclose(surface.normal, [0.0, 0.0, 1.0])
        np.testing.assert_allclose(surface.center, [-0.08, 0.385, 0.3])
        np.testing.assert_allclose(surface.rotation, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(surface.quaternion, [0.0, 0.0, 0.0, 1.0])

    def test_positive_and_negative_plane_signs_produce_identical_surface(self):
        positive = table_surface_from_plane((0.1, -0.2, 0.97, -0.25), ROI_XY)
        negative = table_surface_from_plane((-0.1, 0.2, -0.97, 0.25), ROI_XY)

        np.testing.assert_allclose(positive.plane_model, negative.plane_model)
        np.testing.assert_allclose(positive.center, negative.center)
        np.testing.assert_allclose(positive.rotation, negative.rotation)
        np.testing.assert_allclose(positive.quaternion, negative.quaternion)

    def test_tilted_planes_produce_right_handed_unit_quaternions(self):
        angle = np.deg2rad(6.0)
        normals = (
            np.array([0.0, -np.sin(angle), np.cos(angle)]),
            np.array([np.sin(angle), 0.0, np.cos(angle)]),
        )

        for normal in normals:
            center = np.array([-0.08, 0.385, 0.3])
            plane = np.append(normal, -float(np.dot(normal, center)))
            surface = table_surface_from_plane(plane, ROI_XY)

            with self.subTest(normal=normal.tolist()):
                np.testing.assert_allclose(surface.rotation[:, 2], normal, atol=1e-12)
                np.testing.assert_allclose(
                    surface.rotation.T @ surface.rotation,
                    np.eye(3),
                    atol=1e-12,
                )
                self.assertAlmostEqual(np.linalg.det(surface.rotation), 1.0, places=12)
                self.assertAlmostEqual(np.linalg.norm(surface.quaternion), 1.0, places=12)
                residuals = surface.corners @ surface.normal + surface.plane_model[3]
                np.testing.assert_allclose(residuals, np.zeros(4), atol=1e-12)

    def test_rejects_plane_that_cannot_be_evaluated_over_xy_roi(self):
        with self.assertRaises(ValueError):
            table_surface_from_plane((1.0, 0.0, 0.0, -0.2), ROI_XY)


if __name__ == "__main__":
    unittest.main()
