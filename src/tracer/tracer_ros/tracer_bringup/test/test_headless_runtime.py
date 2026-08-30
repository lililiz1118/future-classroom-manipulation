#!/usr/bin/env python3
from types import SimpleNamespace
import os
import subprocess
import sys
import tempfile
import unittest

import yaml


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from tracer_bringup.headless_runtime import (  # noqa: E402
    REQUIRED_JOINTS,
    RosRuntime,
    assert_joint_state_complete,
    assert_no_conflicting_nodes,
    assert_ros_network_environment,
    assert_route_uses_reverse_ip,
    controller_snapshot,
    should_start_robot_state_publisher,
)
from tracer_bringup.headless_startup import StartupError  # noqa: E402
from tracer_bringup.headless_startup import StartupConfig  # noqa: E402


class RuntimePreflightTest(unittest.TestCase):
    def test_preflight_rejects_driver_package_without_launch_executables(self):
        kinematics = {
            name: {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }
            for name in (
                "shoulder",
                "upper_arm",
                "forearm",
                "wrist_1",
                "wrist_2",
                "wrist_3",
            )
        }
        kinematics["hash"] = "calib_13945068365021364089"
        calibration = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False
        )
        yaml.safe_dump({"kinematics": kinematics}, calibration)
        calibration.close()
        self.addCleanup(lambda: os.unlink(calibration.name))

        class MissingDriverExecutablesRuntime(RosRuntime):
            def _run(self, command, timeout=10.0, required=True):
                if command[0] == "ping":
                    return subprocess.CompletedProcess(command, 0, stdout="", stderr="")
                if command[:3] == ["ip", "route", "get"]:
                    return subprocess.CompletedProcess(
                        command,
                        0,
                        stdout=(
                            "192.168.131.3 dev enp2s0 src 192.168.131.1 uid 1000\n"
                        ),
                        stderr="",
                    )
                if command[:2] == ["rospack", "find"]:
                    return subprocess.CompletedProcess(
                        command, 0, stdout="/tmp/%s\n" % command[-1], stderr=""
                    )
                if command[:2] == ["rosversion", "ur_robot_driver"]:
                    return subprocess.CompletedProcess(
                        command, 0, stdout="2.4.1\n", stderr=""
                    )
                if command[:3] == ["rosrun", "--prefix", "/usr/bin/true"]:
                    return subprocess.CompletedProcess(
                        command,
                        1,
                        stdout="",
                        stderr="Cannot locate node of type [%s]" % command[-1],
                    )
                raise AssertionError("Unexpected command: %r" % (command,))

        runtime = MissingDriverExecutablesRuntime(
            environment={"ROS_IP": "192.168.131.1"}
        )
        config = StartupConfig(
            robot_ip="192.168.131.3",
            reverse_ip="192.168.131.1",
            calibration_path=calibration.name,
            expected_calibration_hash="calib_13945068365021364089",
        )

        with self.assertRaisesRegex(StartupError, "ur_robot_driver_node"):
            runtime.preflight(config)

    def test_route_must_use_the_ur_private_interface(self):
        assert_route_uses_reverse_ip(
            "192.168.131.3 dev enp2s0 src 192.168.131.1 uid 1000", "192.168.131.1"
        )
        with self.assertRaises(StartupError):
            assert_route_uses_reverse_ip(
                "192.168.131.3 via 192.168.43.1 dev wlo1 src 192.168.43.16",
                "192.168.131.1",
            )

    def test_existing_ur_or_moveit_control_nodes_are_conflicts(self):
        assert_no_conflicting_nodes(["/rosout", "/tracer_base_node"])
        for node in (
            "/ur/ur_hardware_interface",
            "/move_group",
            "/servo_server",
            "/keyboard_jog",
        ):
            with self.subTest(node=node), self.assertRaises(StartupError):
                assert_no_conflicting_nodes(["/rosout", node])

    def test_ros_network_identity_must_be_the_ur_private_interface(self):
        assert_ros_network_environment(
            {
                "ROS_IP": "192.168.131.1",
                "ROS_MASTER_URI": "http://192.168.131.1:11311",
            },
            "192.168.131.1",
        )
        with self.assertRaises(StartupError):
            assert_ros_network_environment(
                {
                    "ROS_IP": "192.168.43.16",
                    "ROS_MASTER_URI": "http://192.168.131.1:11311",
                },
                "192.168.131.1",
            )
        with self.assertRaises(StartupError):
            assert_ros_network_environment(
                {
                    "ROS_IP": "192.168.131.1",
                    "ROS_HOSTNAME": "localhost",
                    "ROS_MASTER_URI": "http://192.168.131.1:11311",
                },
                "192.168.131.1",
            )

    def test_robot_state_publisher_is_started_only_when_not_already_running(self):
        self.assertTrue(should_start_robot_state_publisher(["/rosout"]))
        self.assertFalse(
            should_start_robot_state_publisher(["/rosout", "/robot_state_publisher"])
        )


class RosSnapshotTest(unittest.TestCase):
    def test_joint_state_requires_all_six_named_joints(self):
        assert_joint_state_complete(SimpleNamespace(name=list(REQUIRED_JOINTS)))
        with self.assertRaises(StartupError):
            assert_joint_state_complete(SimpleNamespace(name=list(REQUIRED_JOINTS[:-1])))

    def test_controller_message_is_converted_without_losing_claims(self):
        response = SimpleNamespace(
            controller=[
                SimpleNamespace(
                    name="ur_arm_scaled_pos_joint_traj_controller",
                    state="running",
                    claimed_resources=[
                        SimpleNamespace(
                            hardware_interface="hardware_interface::PositionJointInterface",
                            resources=list(REQUIRED_JOINTS),
                        )
                    ],
                )
            ]
        )
        self.assertEqual(
            controller_snapshot(response),
            [
                {
                    "name": "ur_arm_scaled_pos_joint_traj_controller",
                    "state": "running",
                    "claimed_resources": [
                        {
                            "hardware_interface": "hardware_interface::PositionJointInterface",
                            "resources": list(REQUIRED_JOINTS),
                        }
                    ],
                }
            ],
        )

    def test_waits_past_base_joint_messages_until_ur_state_arrives(self):
        class FakeRospy:
            class ROSException(Exception):
                pass

            def __init__(self):
                self.messages = [
                    SimpleNamespace(name=["left_wheel_joint"]),
                    SimpleNamespace(name=list(REQUIRED_JOINTS)),
                ]

            def wait_for_message(self, topic, message_type, timeout):
                return self.messages.pop(0)

        runtime = RosRuntime(environment={})
        runtime._rospy = FakeRospy()
        message = runtime._wait_for_complete_joint_state("/joint_states", object, 1.0)
        self.assertEqual(message.name, list(REQUIRED_JOINTS))

    def test_transient_topic_timeout_does_not_abort_ready_wait(self):
        class FakeRospy:
            class ROSException(Exception):
                pass

            def __init__(self):
                self.calls = 0

            def wait_for_message(self, topic, message_type, timeout):
                self.calls += 1
                if self.calls == 1:
                    raise self.ROSException("not yet")
                return SimpleNamespace(data=True)

        runtime = RosRuntime(environment={})
        runtime._rospy = FakeRospy()
        self.assertTrue(runtime._wait_for_true("/ready", object, 1.0).data)

    def test_speed_wait_ignores_queued_precommand_value(self):
        class FakeRospy:
            class ROSException(Exception):
                pass

            def __init__(self):
                self.messages = [SimpleNamespace(data=1.0), SimpleNamespace(data=0.05)]

            def wait_for_message(self, topic, message_type, timeout):
                return self.messages.pop(0)

        runtime = RosRuntime(environment={})
        runtime._rospy = FakeRospy()
        message = runtime._wait_for_speed_range("/speed", object, 0.05, 1.0)
        self.assertEqual(message.data, 0.05)


if __name__ == "__main__":
    unittest.main()
