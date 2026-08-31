#!/usr/bin/env python3
from types import SimpleNamespace
import os
import signal
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest import mock

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
from tracer_bringup.control_chain_monitor import ControlChainFault  # noqa: E402
from tracer_bringup.runtime_config import load_ur_runtime_policy  # noqa: E402


GRIPPER_JOINT = "gripper_finger1_joint"
RUNTIME_POLICY = load_ur_runtime_policy(
    os.path.join(PACKAGE_ROOT, "config", "ur3_runtime.yaml")
)


class LaunchOutputTest(unittest.TestCase):
    def test_launch_reports_only_summarized_child_diagnostics(self):
        diagnostics = []
        runtime = RosRuntime(environment=os.environ)
        runtime.diagnostic_output = diagnostics.append
        runtime._launch(
            "move_group",
            [
                sys.executable,
                "-c",
                (
                    "import sys; "
                    "print('ordinary initialization'); "
                    "print('[WARN] planning scene is stale', file=sys.stderr); "
                    "print('[ERROR] planner stopped', file=sys.stderr)"
                ),
            ],
        )

        runtime.processes[0][1].wait(timeout=5.0)
        runtime.shutdown()

        self.assertEqual(
            diagnostics,
            [
                "⚠️ [MoveIt] 节点警告｜原文: [WARN] planning scene is stale",
                "❌ [MoveIt] 节点报错｜原文: [ERROR] planner stopped",
            ],
        )

    def test_launch_preserves_ros_process_death_diagnostics(self):
        diagnostics = []
        runtime = RosRuntime(environment=os.environ)
        runtime.diagnostic_output = diagnostics.append
        runtime._launch(
            "ur_driver",
            [
                sys.executable,
                "-c",
                (
                    "import sys; "
                    "print('REQUIRED process [driver-1] has died!', file=sys.stderr); "
                    "print('process has died [pid 42, exit code 1]', file=sys.stderr); "
                    "print('log file: /tmp/driver.log', file=sys.stderr)"
                ),
            ],
        )

        runtime.processes[0][1].wait(timeout=5.0)
        runtime.shutdown()

        self.assertEqual(
            diagnostics,
            [
                "❌ [UR3 驱动] 节点报错｜原文: REQUIRED process [driver-1] has died!",
                "❌ [UR3 驱动] 节点报错｜原文: process has died [pid 42, exit code 1]",
                "❌ [UR3 驱动] 节点报错｜原文: log file: /tmp/driver.log",
            ],
        )

    def test_launch_preserves_multiline_error_context(self):
        diagnostics = []
        runtime = RosRuntime(environment=os.environ)
        runtime.diagnostic_output = diagnostics.append
        runtime._launch(
            "move_group",
            [
                sys.executable,
                "-c",
                (
                    "import sys; "
                    "print('[ERROR] planner crashed', file=sys.stderr); "
                    "print('  at planner.cpp:42', file=sys.stderr); "
                    "print('  invalid group name: arm', file=sys.stderr); "
                    "print('', file=sys.stderr); "
                    "print('[WARN] using fallback planner', file=sys.stderr); "
                    "print('  retrying with RRTConnect', file=sys.stderr)"
                ),
            ],
        )

        runtime.processes[0][1].wait(timeout=5.0)
        runtime.shutdown()

        self.assertEqual(
            diagnostics,
            [
                "❌ [MoveIt] 节点报错｜原文: [ERROR] planner crashed",
                "❌ [MoveIt] 报错详情｜原文: at planner.cpp:42",
                "❌ [MoveIt] 报错详情｜原文: invalid group name: arm",
                "⚠️ [MoveIt] 节点警告｜原文: [WARN] using fallback planner",
                "⚠️ [MoveIt] 警告详情｜原文: retrying with RRTConnect",
            ],
        )

    def test_output_failure_does_not_stop_stderr_drain(self):
        runtime = RosRuntime(environment=os.environ)
        output_attempts = []
        received = []

        def flaky_output(message):
            output_attempts.append(message)
            if len(output_attempts) == 1:
                raise RuntimeError("terminal output unavailable")
            received.append(message)

        runtime.diagnostic_output = flaky_output
        runtime._launch(
            "move_group",
            [
                sys.executable,
                "-c",
                (
                    "import sys; "
                    "print('[ERROR] first failure', file=sys.stderr); "
                    "print('[ERROR] second failure', file=sys.stderr)"
                ),
            ],
        )
        process = runtime.processes[0][1]
        process.wait(timeout=5.0)
        runtime.shutdown()

        self.assertEqual(
            received,
            ["❌ [MoveIt] 节点报错｜原文: [ERROR] second failure"],
        )

    def test_invalid_utf8_does_not_stop_stderr_drain(self):
        diagnostics = []
        runtime = RosRuntime(environment=os.environ)
        runtime.diagnostic_output = diagnostics.append
        runtime._launch(
            "rviz",
            [
                sys.executable,
                "-c",
                (
                    "import os, sys; "
                    "os.write(2, b'\\xff\\n'); "
                    "print('[ERROR] renderer stopped', file=sys.stderr)"
                ),
            ],
        )

        runtime.processes[0][1].wait(timeout=5.0)
        runtime.shutdown()

        self.assertEqual(
            diagnostics,
            ["❌ [RViz] 节点报错｜原文: [ERROR] renderer stopped"],
        )

    def test_shutdown_reaps_child_that_ignores_soft_signals(self):
        ready = threading.Event()
        runtime = RosRuntime(environment=os.environ)
        runtime.diagnostic_output = lambda _message: ready.set()
        runtime._launch(
            "move_group",
            [
                sys.executable,
                "-c",
                (
                    "import signal, sys, time; "
                    "signal.signal(signal.SIGINT, signal.SIG_IGN); "
                    "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
                    "print('[WARN] ready', file=sys.stderr, flush=True); "
                    "time.sleep(30)"
                ),
            ],
        )
        process = runtime.processes[0][1]
        self.assertTrue(ready.wait(timeout=2.0))

        with mock.patch.object(
            headless_runtime, "SHUTDOWN_SIGINT_TIMEOUT", 0.2, create=True
        ), mock.patch.object(
            headless_runtime, "SHUTDOWN_SIGTERM_TIMEOUT", 0.2, create=True
        ), mock.patch.object(
            headless_runtime, "SHUTDOWN_SIGKILL_TIMEOUT", 1.0, create=True
        ):
            runtime.shutdown()

        still_running = process.poll() is None
        if still_running:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait(timeout=2.0)
        self.assertFalse(still_running, "shutdown must reap stubborn child processes")

    def test_shutdown_waits_for_each_stage_before_starting_next(self):
        events = []

        class FakeProcess:
            def __init__(self, label, pid):
                self.label = label
                self.pid = pid
                self.running = True

            def poll(self):
                return None if self.running else 0

            def wait(self, timeout):
                events.append(("wait", self.label))
                self.running = False
                return 0

        class RecordingRuntime(RosRuntime):
            def _run(self, command, timeout=10.0, required=True):
                if list(command[:2]) == ["rosnode", "kill"]:
                    events.append(("stop", "controller_spawner"))
                    return subprocess.CompletedProcess(command, 0, "", "")
                if list(command[:2]) == ["rosnode", "list"]:
                    return subprocess.CompletedProcess(command, 0, "/rosout\n", "")
                raise AssertionError("unexpected command: %r" % (command,))

        runtime = RecordingRuntime(environment={})
        runtime.processes = [
            ("ur_driver", FakeProcess("ur_driver", 1001)),
            ("ag95_gripper", FakeProcess("ag95_gripper", 1002)),
            ("d405_camera", FakeProcess("d405_camera", 1003)),
            ("move_group", FakeProcess("move_group", 1004)),
            ("rviz", FakeProcess("rviz", 1005)),
        ]

        def record_signal(process, requested_signal):
            events.append(("signal", process.label, requested_signal))

        with mock.patch.object(
            runtime, "_signal_process_group", side_effect=record_signal
        ):
            runtime.shutdown()

        self.assertEqual(
            events,
            [
                ("signal", "move_group", signal.SIGINT),
                ("wait", "move_group"),
                ("stop", "controller_spawner"),
                ("signal", "ur_driver", signal.SIGINT),
                ("wait", "ur_driver"),
                ("signal", "ag95_gripper", signal.SIGINT),
                ("wait", "ag95_gripper"),
                ("signal", "d405_camera", signal.SIGINT),
                ("wait", "d405_camera"),
                ("signal", "rviz", signal.SIGINT),
                ("wait", "rviz"),
            ],
        )


class D405LaunchOwnershipTest(unittest.TestCase):
    @staticmethod
    def config(enabled=True):
        return StartupConfig(
            robot_ip="192.168.131.3",
            reverse_ip="192.168.131.1",
            calibration_path="/tmp/real.yaml",
            expected_calibration_hash="calib_13945068365021364089",
            runtime_policy=RUNTIME_POLICY,
            enable_d405=enabled,
        )

    def test_absent_camera_uses_dedicated_launch(self):
        class RecordingRuntime(RosRuntime):
            def __init__(self):
                super().__init__(environment={})
                self.launch = None

            def _launch(self, label, command):
                self.launch = (label, list(command))

        runtime = RecordingRuntime()
        runtime.d405_state = "absent"
        runtime.start_d405(self.config())
        self.assertEqual(
            runtime.launch,
            (
                "d405_camera",
                ["roslaunch", "tracer_bringup", "ur3_d405_camera.launch"],
            ),
        )

    def test_external_or_disabled_camera_is_not_started(self):
        for state, enabled in (("external", True), ("disabled", False)):
            runtime = RosRuntime(environment={})
            runtime.d405_state = state
            runtime._launch = lambda *args: self.fail("must not launch D405")
            runtime.start_d405(self.config(enabled))

    def test_owned_camera_is_registered_and_shutdown_with_sigint(self):
        process = mock.Mock(pid=4321)
        process.poll.side_effect = (None, 0)
        runtime = RosRuntime(environment={})
        runtime.processes = [("d405_camera", process)]

        with mock.patch.object(os, "getpgid", return_value=4321), mock.patch.object(
            os, "killpg"
        ) as killpg:
            runtime.shutdown()

        killpg.assert_called_once_with(4321, signal.SIGINT)


class DriverLaunchPolicyTest(unittest.TestCase):
    def test_driver_command_explicitly_passes_controlled_receive_timeout(self):
        class RecordingRuntime(RosRuntime):
            def __init__(self):
                super().__init__(environment={})
                self.launch = None

            def _launch(self, label, command):
                self.launch = (label, list(command))

        runtime = RecordingRuntime()
        runtime.start_driver(
            StartupConfig(
                robot_ip="192.168.131.3",
                reverse_ip="192.168.131.1",
                calibration_path="/tmp/real.yaml",
                expected_calibration_hash="calib_13945068365021364089",
                runtime_policy=RUNTIME_POLICY,
            )
        )

        self.assertEqual(runtime.launch[0], "ur_driver")
        self.assertIn("robot_receive_timeout:=0.10", runtime.launch[1])


class ControlChainFaultGateTest(unittest.TestCase):
    def test_move_group_cannot_start_after_latched_fault(self):
        class FaultMonitor:
            def raise_if_fault(self):
                raise ControlChainFault("safety mode=REDUCED")

        runtime = RosRuntime(environment={})
        runtime.control_chain_monitor = FaultMonitor()
        runtime._launch = lambda *_args: self.fail("MoveIt must not launch")

        with self.assertRaisesRegex(StartupError, "safety mode=REDUCED"):
            runtime.start_move_group(
                StartupConfig(
                    robot_ip="192.168.131.3",
                    reverse_ip="192.168.131.1",
                    calibration_path="/tmp/real.yaml",
                    expected_calibration_hash="calib_13945068365021364089",
                    runtime_policy=RUNTIME_POLICY,
                )
            )

    def test_fault_stops_move_group_before_supervision_raises(self):
        events = []

        class Process:
            def __init__(self, code=None):
                self.code = code

            def poll(self):
                return self.code

        class FaultMonitor:
            def raise_if_fault(self):
                raise ControlChainFault("robot_program_running=False")

        class RecordingRuntime(RosRuntime):
            def _shutdown_process(self, process):
                events.append(process)
                process.code = 0

        move_group = Process()
        runtime = RecordingRuntime(environment={})
        runtime.control_chain_monitor = FaultMonitor()
        runtime.processes = [("move_group", move_group), ("rviz", Process(0))]

        with self.assertRaisesRegex(
            StartupError,
            "CONTROL CHAIN FAULT: robot_program_running=False.*Full restart required",
        ):
            runtime.supervise()

        self.assertEqual(events, [move_group])
        self.assertNotIn("move_group", [label for label, _ in runtime.processes])

    def test_blocking_topic_wait_aborts_on_latched_fault(self):
        class FakeRospy:
            class ROSException(Exception):
                pass

            def wait_for_message(self, topic, message_type, timeout):
                raise self.ROSException("no message")

        class FaultMonitor:
            def raise_if_fault(self):
                raise ControlChainFault("joint_states stale")

        runtime = RosRuntime(environment={})
        runtime._rospy = FakeRospy()
        runtime.control_chain_monitor = FaultMonitor()

        with self.assertRaisesRegex(StartupError, "joint_states stale"):
            runtime._wait_for_matching_message(
                "/test", object, 1.0, lambda _message: True, lambda _last: "timeout"
            )


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
            runtime_policy=RUNTIME_POLICY,
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
            runtime_policy=RUNTIME_POLICY,
            gripper_device="/dev/null",
        )

        with self.assertRaisesRegex(StartupError, "dh_gripper_driver"):
            runtime.preflight(config)

    def test_owned_d405_requires_the_nodelet_library_but_external_d405_does_not(self):
        class NodesRuntime(RosRuntime):
            def __init__(self, nodes, environment):
                super().__init__(environment=environment)
                self.nodes = nodes

            def _run(self, command, timeout=10.0, required=True):
                if command == ["rosnode", "list"]:
                    return subprocess.CompletedProcess(
                        command,
                        0,
                        stdout="\n".join(self.nodes) + "\n",
                        stderr="",
                    )
                raise AssertionError("Unexpected command: %r" % (command,))

        with tempfile.TemporaryDirectory() as prefix:
            environment = {
                "CMAKE_PREFIX_PATH": prefix,
                "LD_LIBRARY_PATH": os.path.join(prefix, "lib"),
            }
            with self.assertRaisesRegex(
                StartupError, "librealsense2_camera.so.*catkin_make"
            ):
                NodesRuntime([], environment).assert_no_conflicts(
                    StartupConfig(
                        robot_ip="",
                        reverse_ip="",
                        calibration_path="",
                        expected_calibration_hash="",
                        runtime_policy=RUNTIME_POLICY,
                        enable_d405=True,
                    )
                )

            external_nodes = [
                "/d405/realsense2_camera",
                "/d405/realsense2_camera_manager",
            ]
            NodesRuntime(external_nodes, environment).assert_no_conflicts(
                StartupConfig(
                    robot_ip="",
                    reverse_ip="",
                    calibration_path="",
                    expected_calibration_hash="",
                    runtime_policy=RUNTIME_POLICY,
                    enable_d405=True,
                )
            )
            NodesRuntime([], environment).assert_no_conflicts(
                StartupConfig(
                    robot_ip="",
                    reverse_ip="",
                    calibration_path="",
                    expected_calibration_hash="",
                    runtime_policy=RUNTIME_POLICY,
                    enable_d405=False,
                )
            )

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
            "/joint_state_aggregator",
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


class CameraDuration:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


class CameraStamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def __le__(self, other):
        return self.seconds <= other.seconds

    def __sub__(self, other):
        return CameraDuration(self.seconds - other.seconds)


def image(stamp, width=640, height=480):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=CameraStamp(stamp)),
        width=width,
        height=height,
    )


def camera_info(width, height):
    return SimpleNamespace(width=width, height=height)


def camera_config(enable_d405=True):
    return StartupConfig(
        robot_ip="192.168.131.3",
        reverse_ip="192.168.131.1",
        calibration_path="/tmp/real.yaml",
        expected_calibration_hash="calib_13945068365021364089",
        runtime_policy=RUNTIME_POLICY,
        state_timeout=0.01,
        enable_d405=enable_d405,
    )


class FakeCameraRospy:
    class ROSException(Exception):
        pass

    def __init__(self, messages, now):
        self.messages = {topic: list(values) for topic, values in messages.items()}
        self.calls = []
        now_stamp = CameraStamp(now)
        self.Time = SimpleNamespace(now=lambda: now_stamp)

    def wait_for_message(self, topic, message_type, timeout):
        self.calls.append((topic, timeout))
        values = self.messages.get(topic, [])
        if not values:
            raise self.ROSException("timeout")
        return values.pop(0)


def camera_runtime(color, depth, info, now):
    runtime = RosRuntime(environment={})
    runtime._rospy = FakeCameraRospy(
        {
            "/d405/color/image_raw": color,
            "/d405/depth/image_rect_raw": depth,
            "/d405/color/camera_info": info,
        },
        now,
    )
    return runtime


class D405ReadinessTest(unittest.TestCase):
    def test_ready_requires_exactly_two_color_depth_frames_and_matching_info(self):
        runtime = camera_runtime(
            color=[image(99.8), image(99.9)],
            depth=[image(99.8), image(99.9)],
            info=[camera_info(640, 480)],
            now=100.0,
        )

        runtime.wait_d405_ready(camera_config())

        self.assertEqual(
            [topic for topic, _timeout in runtime._rospy.calls],
            [
                "/d405/color/image_raw",
                "/d405/color/image_raw",
                "/d405/depth/image_rect_raw",
                "/d405/depth/image_rect_raw",
                "/d405/color/camera_info",
            ],
        )

    def test_disabled_camera_returns_without_subscribing(self):
        runtime = camera_runtime(color=[], depth=[], info=[], now=100.0)

        runtime.wait_d405_ready(camera_config(enable_d405=False))

        self.assertEqual(runtime._rospy.calls, [])

    def test_missing_color_reports_exact_topic(self):
        runtime = camera_runtime(color=[], depth=[], info=[], now=100.0)
        with self.assertRaisesRegex(StartupError, "/d405/color/image_raw"):
            runtime.wait_d405_ready(camera_config())

    def test_missing_depth_reports_exact_topic(self):
        runtime = camera_runtime(
            color=[image(99.8), image(99.9)], depth=[], info=[], now=100.0
        )
        with self.assertRaisesRegex(StartupError, "/d405/depth/image_rect_raw"):
            runtime.wait_d405_ready(camera_config())

    def test_non_advancing_stale_and_future_color_are_rejected(self):
        cases = (
            ([image(99.9), image(99.9)], "not advancing"),
            ([image(97.0), image(98.0)], "stale"),
            ([image(100.1), image(100.2)], "future"),
        )
        for color, reason in cases:
            with self.subTest(reason=reason):
                runtime = camera_runtime(color=color, depth=[], info=[], now=100.0)
                with self.assertRaisesRegex(
                    StartupError, "/d405/color/image_raw.*%s" % reason
                ):
                    runtime.wait_d405_ready(camera_config())

    def test_non_advancing_stale_and_future_depth_are_rejected(self):
        cases = (
            ([image(99.9), image(99.9)], "not advancing"),
            ([image(97.0), image(98.0)], "stale"),
            ([image(100.1), image(100.2)], "future"),
        )
        for depth, reason in cases:
            with self.subTest(reason=reason):
                runtime = camera_runtime(
                    color=[image(99.8), image(99.9)],
                    depth=depth,
                    info=[],
                    now=100.0,
                )
                with self.assertRaisesRegex(
                    StartupError, "/d405/depth/image_rect_raw.*%s" % reason
                ):
                    runtime.wait_d405_ready(camera_config())

    def test_non_positive_color_dimensions_are_rejected(self):
        for width, height in ((0, 480), (640, 0), (-1, 480), (640, -1)):
            with self.subTest(width=width, height=height):
                runtime = camera_runtime(
                    color=[image(99.8), image(99.9, width=width, height=height)],
                    depth=[image(99.8), image(99.9)],
                    info=[camera_info(width, height)],
                    now=100.0,
                )
                with self.assertRaisesRegex(
                    StartupError, "/d405/color/image_raw.*positive"
                ):
                    runtime.wait_d405_ready(camera_config())

    def test_camera_info_must_match_positive_color_dimensions(self):
        runtime = camera_runtime(
            color=[image(99.8), image(99.9)],
            depth=[image(99.8), image(99.9)],
            info=[camera_info(1280, 720)],
            now=100.0,
        )
        with self.assertRaisesRegex(
            StartupError, "camera_info.*1280x720.*640x480"
        ):
            runtime.wait_d405_ready(camera_config())

    def test_missing_camera_info_reports_exact_topic(self):
        runtime = camera_runtime(
            color=[image(99.8), image(99.9)],
            depth=[image(99.8), image(99.9)],
            info=[],
            now=100.0,
        )
        with self.assertRaisesRegex(StartupError, "/d405/color/camera_info"):
            runtime.wait_d405_ready(camera_config())

    def test_owned_camera_exit_replaces_generic_timeout(self):
        runtime = camera_runtime(color=[], depth=[], info=[], now=100.0)
        runtime.processes.append(
            ("d405_camera", SimpleNamespace(poll=lambda: 7))
        )
        with self.assertRaisesRegex(
            StartupError, "d405_camera exited unexpectedly with code 7"
        ):
            runtime.wait_d405_ready(camera_config())


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
                runtime_policy=RUNTIME_POLICY,
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
            runtime_policy=RUNTIME_POLICY,
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
                    "publish_joint_state_relay:=false",
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
                runtime_policy=RUNTIME_POLICY,
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

    def test_control_chain_readiness_starts_the_monitor_and_checks_driver_params(self):
        events = []

        class FakeMonitor:
            def __init__(self, rospy, policy, **kwargs):
                events.append(("constructed", policy, kwargs))

            def start(self):
                events.append("started")

            def wait_until_ready(self, timeout):
                events.append(("ready", timeout))

            def raise_if_fault(self):
                pass

        class FakeRospy:
            def wait_for_message(self, topic, message_type, timeout):
                return SimpleNamespace(data=0.5)

            def get_param(self, name, default):
                return "calib_13945068365021364089"

        runtime = RosRuntime(
            environment={}, control_chain_monitor_factory=FakeMonitor
        )
        runtime._rospy = FakeRospy()
        config = StartupConfig(
            robot_ip="192.168.131.3",
            reverse_ip="192.168.131.1",
            calibration_path="/tmp/real.yaml",
            expected_calibration_hash="calib_13945068365021364089",
            runtime_policy=RUNTIME_POLICY,
        )

        runtime.wait_control_chain_ready(config)

        self.assertEqual(events[1:], ["started", ("ready", config.state_timeout)])
        self.assertIs(events[0][1], RUNTIME_POLICY)

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
