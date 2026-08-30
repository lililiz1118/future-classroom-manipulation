#!/usr/bin/env python3
import subprocess
import shutil
import unittest

import yaml


@unittest.skipUnless(shutil.which("roslaunch"), "requires a ROS Noetic environment")
class HeadlessLaunchTest(unittest.TestCase):
    def dump(self, filename):
        result = subprocess.run(
            ["roslaunch", "--dump-params", "tracer_bringup", filename],
            check=True,
            capture_output=True,
            text=True,
        )
        return yaml.safe_load(result.stdout)

    def test_driver_launch_uses_real_calibration_and_headless_mode(self):
        params = self.dump("ur3_headless_driver.launch")
        self.assertEqual(
            params["/ur/ur_hardware_interface/robot_ip"], "192.168.131.3"
        )
        self.assertEqual(
            params["/ur/ur_hardware_interface/reverse_ip"], "192.168.131.1"
        )
        self.assertTrue(params["/ur/ur_hardware_interface/headless_mode"])
        self.assertEqual(
            params["/ur/ur_hardware_interface/kinematics/hash"],
            "calib_13945068365021364089",
        )
        self.assertIn("-0.2436409187296593", params["/robot_description"])

    def test_move_group_enables_scaled_controller_trajectory_execution(self):
        params = self.dump("ur3_moveit_execution.launch")
        self.assertTrue(params["/move_group/allow_trajectory_execution"])
        controllers = params["/move_group/controller_list"]
        self.assertEqual(
            controllers[0]["name"], "ur/ur_arm_scaled_pos_joint_traj_controller"
        )
        self.assertEqual(controllers[0]["action_ns"], "follow_joint_trajectory")


if __name__ == "__main__":
    unittest.main()
