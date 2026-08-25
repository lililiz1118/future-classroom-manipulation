#!/usr/bin/env python3
import subprocess
import unittest

import yaml


class CalibratedUrLaunchTest(unittest.TestCase):
    def test_chess_startup_preloads_the_calibrated_robot_model(self):
        result = subprocess.run(
            [
                "roslaunch",
                "--dump-params",
                "tracer_bringup",
                "chess_ur_startup.launch",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        params = yaml.safe_load(result.stdout)

        self.assertIn("/robot_description", params)
        self.assertIn("-0.2436409187296593", params["/robot_description"])
        self.assertEqual(
            params["/ur/ur_hardware_interface/kinematics/hash"],
            "calib_13945068365021364089",
        )


if __name__ == "__main__":
    unittest.main()
