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
        self.assertEqual(
            params.get(
                "/move_group/trajectory_execution/execution_duration_monitoring"
            ),
            False,
        )
        controllers = params["/move_group/controller_list"]
        self.assertEqual(
            controllers[0]["name"], "ur/ur_arm_scaled_pos_joint_traj_controller"
        )
        self.assertEqual(controllers[0]["action_ns"], "follow_joint_trajectory")

    def test_ag95_launch_uses_real_device_and_does_not_respawn(self):
        result = subprocess.run(
            [
                "roslaunch",
                "--dump-params",
                "tracer_bringup",
                "ag95_gripper_state.launch",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr or result.stdout)
        params = yaml.safe_load(result.stdout)
        self.assertEqual(params["/dh_gripper_driver/gripper_id"], "1")
        self.assertEqual(params["/dh_gripper_driver/gripper_model"], "AG95_MB")
        self.assertEqual(
            params["/dh_gripper_driver/connect_port"], "/dev/dh_gripper_usb"
        )
        self.assertEqual(params["/dh_gripper_driver/baudrate"], "115200")

        import roslaunch

        launch_path = roslaunch.rlutil.resolve_launch_arguments(
            ["tracer_bringup", "ag95_gripper_state.launch"]
        )[0]
        launch_config = roslaunch.config.load_config_default([launch_path], None)
        nodes = {node.name: node for node in launch_config.nodes}
        self.assertTrue(nodes["dh_gripper_driver"].required)
        self.assertFalse(nodes["dh_gripper_driver"].respawn)
        self.assertIn("gripper_joint_state_relay", nodes)

    def test_bringup_declares_the_ag95_runtime_dependencies(self):
        result = subprocess.run(
            ["rospack", "depends1", "tracer_bringup"],
            check=False,
            capture_output=True,
            text=True,
        )
        dependencies = set(result.stdout.splitlines())

        self.assertIn("dh_gripper_driver", dependencies)
        self.assertIn("dh_gripper_msgs", dependencies)


if __name__ == "__main__":
    unittest.main()
