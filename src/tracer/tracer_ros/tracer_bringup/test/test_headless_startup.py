#!/usr/bin/env python3
import os
import sys
import tempfile
import unittest

import yaml


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from tracer_bringup.headless_dashboard import RobotStatus, SafetyGateError  # noqa: E402
from tracer_bringup.headless_startup import (  # noqa: E402
    StartupAborted,
    StartupConfig,
    StartupCoordinator,
    StartupError,
    assert_exclusive_controller,
    validate_calibration,
)


EXPECTED_HASH = "calib_13945068365021364089"
JOINTS = [
    "ur_arm_shoulder_pan_joint",
    "ur_arm_shoulder_lift_joint",
    "ur_arm_elbow_joint",
    "ur_arm_wrist_1_joint",
    "ur_arm_wrist_2_joint",
    "ur_arm_wrist_3_joint",
]
TARGET = "ur_arm_scaled_pos_joint_traj_controller"


class FakeDashboard:
    def __init__(self, status):
        self.current_status = status
        self.events = []

    def preflight(self, allow_reduced):
        self.events.append("dashboard_preflight")
        return self.current_status

    def power_on(self):
        self.events.append("power_on")

    def brake_release(self):
        self.events.append("brake_release")

    def wait_robot_mode(self, modes, timeout, allow_reduced):
        self.events.append("wait:" + ",".join(sorted(modes)))
        if "RUNNING" in modes and len(modes) == 1:
            self.current_status = RobotStatus("RUNNING", "NORMAL")
        else:
            self.current_status = RobotStatus("POWER_ON", "NORMAL")
        return self.current_status


class FakeRuntime:
    def __init__(self, fail_at=None):
        self.events = []
        self.fail_at = fail_at

    def _event(self, name):
        self.events.append(name)
        if self.fail_at == name:
            raise StartupError(name)

    def preflight(self, config):
        self._event("runtime_preflight")

    def assert_no_conflicts(self):
        self._event("no_conflicts")

    def start_driver(self, config):
        self._event("start_driver")

    def wait_driver_ready(self, config):
        self._event("driver_ready")

    def set_speed_slider(self, fraction):
        self._event("set_speed:%.2f" % fraction)

    def start_move_group(self, config):
        self._event("start_move_group")

    def wait_move_group_ready(self, config):
        self._event("move_group_ready")

    def start_rviz(self, config):
        self._event("start_rviz")

    def supervise(self):
        self._event("supervise")

    def shutdown(self):
        self.events.append("shutdown")


def config():
    return StartupConfig(
        robot_ip="192.168.131.3",
        reverse_ip="192.168.131.1",
        calibration_path="/tmp/real.yaml",
        expected_calibration_hash=EXPECTED_HASH,
        speed_slider=0.05,
        allow_reduced=False,
        state_timeout=20.0,
    )


class CalibrationValidationTest(unittest.TestCase):
    def _write_yaml(self, data):
        handle = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
        yaml.safe_dump(data, handle)
        handle.close()
        self.addCleanup(lambda: os.unlink(handle.name))
        return handle.name

    def test_accepts_complete_real_calibration(self):
        kinematics = {
            name: {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
            for name in ("shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3")
        }
        kinematics["hash"] = EXPECTED_HASH
        path = self._write_yaml({"kinematics": kinematics})
        self.assertEqual(validate_calibration(path, EXPECTED_HASH), EXPECTED_HASH)

    def test_rejects_wrong_robot_calibration_hash(self):
        path = self._write_yaml({"kinematics": {"hash": "calib_wrong"}})
        with self.assertRaises(StartupError):
            validate_calibration(path, EXPECTED_HASH)


class ControllerExclusivityTest(unittest.TestCase):
    @staticmethod
    def controller(name, state, resources):
        return {
            "name": name,
            "state": state,
            "claimed_resources": [
                {
                    "hardware_interface": "hardware_interface::PositionJointInterface",
                    "resources": resources,
                }
            ],
        }

    def test_target_is_the_only_running_controller_claiming_arm_joints(self):
        snapshot = [
            self.controller(TARGET, "running", JOINTS),
            self.controller("ur_arm_joint_state_controller", "running", []),
            self.controller("ur_arm_pos_joint_traj_controller", "stopped", JOINTS),
        ]
        assert_exclusive_controller(snapshot, TARGET, JOINTS)

    def test_rejects_second_running_motion_controller(self):
        snapshot = [
            self.controller(TARGET, "running", JOINTS),
            self.controller("ur_arm_pos_joint_traj_controller", "running", JOINTS),
        ]
        with self.assertRaises(StartupError):
            assert_exclusive_controller(snapshot, TARGET, JOINTS)


class StartupCoordinatorTest(unittest.TestCase):
    def test_confirmation_rejection_has_no_mutating_side_effect(self):
        dashboard = FakeDashboard(RobotStatus("POWER_OFF", "NORMAL"))
        runtime = FakeRuntime()
        coordinator = StartupCoordinator(
            dashboard, runtime, config(), confirm=lambda _: False, output=lambda _: None
        )

        with self.assertRaises(StartupAborted):
            coordinator.run()

        self.assertEqual(dashboard.events, ["dashboard_preflight"])
        self.assertEqual(runtime.events, ["runtime_preflight", "no_conflicts"])

    def test_blocked_safety_state_stops_before_confirmation(self):
        dashboard = FakeDashboard(RobotStatus("POWER_OFF", "PROTECTIVE_STOP"))
        runtime = FakeRuntime()
        confirmations = []
        coordinator = StartupCoordinator(
            dashboard,
            runtime,
            config(),
            confirm=lambda prompt: confirmations.append(prompt) or True,
            output=lambda _: None,
        )

        with self.assertRaises(SafetyGateError):
            coordinator.run()

        self.assertEqual(confirmations, [])
        self.assertEqual(dashboard.events, ["dashboard_preflight"])

    def test_unstartable_robot_mode_stops_before_confirmation(self):
        dashboard = FakeDashboard(RobotStatus("BACKDRIVE", "NORMAL"))
        runtime = FakeRuntime()
        confirmations = []
        coordinator = StartupCoordinator(
            dashboard,
            runtime,
            config(),
            confirm=lambda prompt: confirmations.append(prompt) or True,
            output=lambda _: None,
        )

        with self.assertRaises(StartupError):
            coordinator.run()

        self.assertEqual(confirmations, [])

    def test_successful_startup_preserves_hardware_then_ros_order(self):
        dashboard = FakeDashboard(RobotStatus("POWER_OFF", "NORMAL"))
        runtime = FakeRuntime()
        coordinator = StartupCoordinator(
            dashboard, runtime, config(), confirm=lambda _: True, output=lambda _: None
        )

        coordinator.run()

        self.assertEqual(
            dashboard.events,
            [
                "dashboard_preflight",
                "power_on",
                "wait:IDLE,POWER_ON,RUNNING",
                "brake_release",
                "wait:RUNNING",
            ],
        )
        self.assertEqual(
            runtime.events,
            [
                "runtime_preflight",
                "no_conflicts",
                "start_driver",
                "driver_ready",
                "set_speed:0.05",
                "start_move_group",
                "move_group_ready",
                "start_rviz",
                "supervise",
                "shutdown",
            ],
        )

    def test_runtime_failure_after_driver_start_triggers_cleanup(self):
        dashboard = FakeDashboard(RobotStatus("RUNNING", "NORMAL"))
        runtime = FakeRuntime(fail_at="driver_ready")
        coordinator = StartupCoordinator(
            dashboard, runtime, config(), confirm=lambda _: True, output=lambda _: None
        )

        with self.assertRaises(StartupError):
            coordinator.run()

        self.assertEqual(runtime.events[-1], "shutdown")


if __name__ == "__main__":
    unittest.main()
