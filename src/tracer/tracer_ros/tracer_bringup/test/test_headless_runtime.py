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

import tracer_bringup.headless_runtime as headless_runtime  # noqa: E402
from tracer_bringup.headless_runtime import (  # noqa: E402
    REQUIRED_JOINTS,
    SPEED_SCALING_TOPIC,
    RosRuntime,
    assert_no_conflicting_nodes,
    assert_ros_network_environment,
    assert_route_uses_reverse_ip,
    classify_d405_nodes,
    controller_snapshot,
    should_start_robot_state_publisher,
)
from tracer_bringup.headless_startup import StartupError  # noqa: E402
from tracer_bringup.headless_startup import StartupConfig  # noqa: E402


GRIPPER_JOINT = "gripper_finger1_joint"


class CameraNodeClassificationTest(unittest.TestCase):
    def test_disabled_complete_d405_is_ignored(self):
        nodes = ["/d405/realsense2_camera", "/d405/realsense2_camera_manager"]
        self.assertEqual(classify_d405_nodes(nodes, False), "disabled")

    def test_absent_and_external_states(self):
        self.assertEqual(classify_d405_nodes(["/rosout"], True), "absent")
        self.assertEqual(
            classify_d405_nodes(
                ["/d405/realsense2_camera", "/d405/realsense2_camera_manager"],
                True,
            ),
            "external",
        )

    def test_partial_d405_is_rejected(self):
        for node in (
            "/d405/realsense2_camera",
            "/d405/realsense2_camera_manager",
        ):
            with self.subTest(node=node), self.assertRaisesRegex(
                StartupError, "Incomplete D405"
            ):
                classify_d405_nodes([node], True)

    def test_any_d455_is_rejected_even_when_d405_is_disabled(self):
        for node in (
            "/d455/realsense2_camera",
            "/d455/realsense2_camera_manager",
        ):
            with self.subTest(node=node), self.assertRaisesRegex(
                StartupError, "D455"
            ):
                classify_d405_nodes([node], False)


class RuntimePreflightTest(unittest.TestCase):
    def test_gripper_device_must_be_an_accessible_character_device(self):
        assert_gripper_device_ready = headless_runtime.assert_gripper_device_ready
        assert_gripper_device_ready("/dev/null")
        with self.assertRaisesRegex(StartupError, "does not exist"):
            assert_gripper_device_ready("/dev/definitely_missing_ag95")

        regular_file = tempfile.NamedTemporaryFile(delete=False)
        regular_file.close()
        self.addCleanup(lambda: os.unlink(regular_file.name))
        with self.assertRaisesRegex(StartupError, "not a character device"):
            assert_gripper_device_ready(regular_file.name)

    def test_speed_scaling_topic_matches_the_driver_published_interface(self):
        self.assertEqual(SPEED_SCALING_TOPIC, "/ur/speed_scaling_factor")

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

    def test_preflight_rejects_unbuilt_gripper_driver_executable(self):
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

        class MissingGripperExecutableRuntime(RosRuntime):
            def _run(self, command, timeout=10.0, required=True):
                if command[0] == "ping":
                    return subprocess.CompletedProcess(command, 0, stdout="", stderr="")
                if command[:3] == ["ip", "route", "get"]:
                    return subprocess.CompletedProcess(
                        command,
                        0,
                        stdout="192.168.131.3 dev enp2s0 src 192.168.131.1\n",
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
                        1 if command[-2:] == ["dh_gripper_driver", "dh_gripper_driver"] else 0,
                        stdout="",
                        stderr="Cannot locate AG95 driver",
                    )
                raise AssertionError("Unexpected command: %r" % (command,))

        runtime = MissingGripperExecutableRuntime(
            environment={"ROS_IP": "192.168.131.1"}
        )
        config = StartupConfig(
            robot_ip="192.168.131.3",
            reverse_ip="192.168.131.1",
            calibration_path=calibration.name,
            expected_calibration_hash="calib_13945068365021364089",
            gripper_device="/dev/null",
        )

        with self.assertRaisesRegex(StartupError, "dh_gripper_driver"):
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

        with self.assertRaisesRegex(StartupError, "Route to UR controller"):
            assert_route_uses_reverse_ip(
                "192.168.131.3 dev enp2s0 src", "192.168.131.1"
            )

    def test_existing_ur_or_moveit_control_nodes_are_conflicts(self):
        assert_no_conflicting_nodes(["/rosout", "/tracer_base_node"])
        for node in (
            "/ur/ur_hardware_interface",
            "/move_group",
            "/servo_server",
            "/keyboard_jog",
            "/dh_gripper_driver",
            "/gripper_joint_state_relay",
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
    def test_gripper_joint_samples_must_advance_and_be_fresh(self):
        class Duration:
            def __init__(self, seconds):
                self.seconds = seconds

            def to_sec(self):
                return self.seconds

        class Stamp:
            def __init__(self, seconds):
                self.seconds = seconds

            def __le__(self, other):
                return self.seconds <= other.seconds

            def __sub__(self, other):
                return Duration(self.seconds - other.seconds)

        class Time:
            @staticmethod
            def now():
                return Stamp(100.0)

        runtime = RosRuntime(environment={})
        runtime._rospy = SimpleNamespace(Time=Time)
        check = runtime._assert_fresh_advancing_joint_states
        check(
            SimpleNamespace(header=SimpleNamespace(stamp=Stamp(99.8))),
            SimpleNamespace(header=SimpleNamespace(stamp=Stamp(99.9))),
            "/gripper/joint_states",
        )
        with self.assertRaisesRegex(StartupError, "not advancing"):
            check(
                SimpleNamespace(header=SimpleNamespace(stamp=Stamp(99.9))),
                SimpleNamespace(header=SimpleNamespace(stamp=Stamp(99.9))),
                "/gripper/joint_states",
            )
        with self.assertRaisesRegex(StartupError, "stale"):
            check(
                SimpleNamespace(header=SimpleNamespace(stamp=Stamp(97.0))),
                SimpleNamespace(header=SimpleNamespace(stamp=Stamp(98.0))),
                "/gripper/joint_states",
            )

    def test_gripper_ready_wait_ignores_uninitialized_and_wrong_joint_messages(self):
        class FakeRospy:
            class ROSException(Exception):
                pass

            def __init__(self, messages):
                self.messages = list(messages)

            def wait_for_message(self, topic, message_type, timeout):
                return self.messages.pop(0)

        runtime = RosRuntime(environment={})
        runtime._rospy = FakeRospy(
            [SimpleNamespace(is_initialized=False), SimpleNamespace(is_initialized=True)]
        )
        state = runtime._wait_for_initialized_gripper("/gripper/states", object, 1.0)
        self.assertTrue(state.is_initialized)

        runtime._rospy = FakeRospy(
            [
                SimpleNamespace(name=["left_wheel_joint"]),
                SimpleNamespace(name=[GRIPPER_JOINT]),
            ]
        )
        joint_state = runtime._wait_for_named_joint_state(
            "/gripper/joint_states", object, {GRIPPER_JOINT}, 1.0
        )
        self.assertEqual(joint_state.name, [GRIPPER_JOINT])

    def test_shared_joint_state_wait_keeps_one_subscription_until_gripper_arrives(self):
        class FakeSubscription:
            def __init__(self):
                self.unregistered = False

            def unregister(self):
                self.unregistered = True

        class FakeRospy:
            def __init__(self):
                self.subscription = FakeSubscription()

            def Subscriber(self, topic, message_type, callback, queue_size):
                callback(SimpleNamespace(name=list(REQUIRED_JOINTS)))
                callback(SimpleNamespace(name=[GRIPPER_JOINT]))
                return self.subscription

        runtime = RosRuntime(environment={})
        runtime._rospy = FakeRospy()

        message = runtime._wait_for_named_joint_on_busy_topic(
            "/joint_states", object, {GRIPPER_JOINT}, 1.0
        )

        self.assertEqual(message.name, [GRIPPER_JOINT])
        self.assertTrue(runtime._rospy.subscription.unregistered)

    def test_gripper_readiness_uses_persistent_wait_on_shared_joint_states(self):
        class RecordingRuntime(RosRuntime):
            def __init__(self):
                super().__init__(environment={})
                self.busy_topic_wait = None

            def _wait_for_initialized_gripper(self, *args):
                return SimpleNamespace(is_initialized=True)

            def _wait_for_named_joint_state(self, *args):
                return SimpleNamespace(name=[GRIPPER_JOINT])

            def _assert_fresh_advancing_joint_states(self, *args):
                return None

            def _wait_for_named_joint_on_busy_topic(self, *args):
                self.busy_topic_wait = args
                return SimpleNamespace(name=[GRIPPER_JOINT])

        runtime = RecordingRuntime()
        runtime.wait_gripper_ready(
            StartupConfig(
                robot_ip="192.168.131.3",
                reverse_ip="192.168.131.1",
                calibration_path="/tmp/real.yaml",
                expected_calibration_hash="calib_13945068365021364089",
            )
        )

        self.assertEqual(runtime.busy_topic_wait[0], "/joint_states")
        self.assertEqual(runtime.busy_topic_wait[2], {GRIPPER_JOINT})

    def test_start_gripper_passes_the_confirmed_physical_device(self):
        class RecordingRuntime(RosRuntime):
            def __init__(self):
                super().__init__(environment={})
                self.launch = None

            def _launch(self, label, command):
                self.launch = (label, list(command))

        runtime = RecordingRuntime()
        startup_config = StartupConfig(
            robot_ip="192.168.131.3",
            reverse_ip="192.168.131.1",
            calibration_path="/tmp/real.yaml",
            expected_calibration_hash="calib_13945068365021364089",
            gripper_device="/dev/test_ag95",
        )

        runtime.start_gripper(startup_config)

        self.assertEqual(
            runtime.launch,
            (
                "ag95_gripper",
                [
                    "roslaunch",
                    "tracer_bringup",
                    "ag95_gripper_state.launch",
                    "gripper_device:=/dev/test_ag95",
                ],
            ),
        )

    def test_start_rviz_uses_the_headless_model_root_configuration(self):
        class RecordingRuntime(RosRuntime):
            def __init__(self):
                super().__init__(environment={})
                self.launch = None

            def _launch(self, label, command):
                self.launch = (label, list(command))

        runtime = RecordingRuntime()
        runtime.package_paths["tracer_bringup"] = "/workspace/tracer_bringup"

        runtime.start_rviz(
            StartupConfig(
                robot_ip="192.168.131.3",
                reverse_ip="192.168.131.1",
                calibration_path="/tmp/real.yaml",
                expected_calibration_hash="calib_13945068365021364089",
            )
        )

        self.assertEqual(
            runtime.launch,
            (
                "rviz",
                [
                    "roslaunch",
                    "moveit_config",
                    "moveit_rviz.launch",
                    "rviz_config:=%s"
                    % os.path.join(
                        "/workspace/tracer_bringup",
                        "config",
                        "ur3_headless_moveit.rviz",
                    ),
                ],
            ),
        )

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
        message = runtime._wait_for_named_joint_state(
            "/joint_states", object, REQUIRED_JOINTS, 1.0
        )
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
