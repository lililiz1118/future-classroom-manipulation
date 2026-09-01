#!/home/jt001/.conda/envs/anygrasp/bin/python

import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from anygrasp_ros.core import FilteredCloud


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


if __name__ == "__main__":
    unittest.main()
