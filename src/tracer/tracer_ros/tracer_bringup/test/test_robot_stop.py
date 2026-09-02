#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from tracer_bringup.robot_stop import (  # noqa: E402
    StopCoordinator,
    StopError,
    LinuxRobotPlatform,
    classify_process,
    main,
)


class FakeRobotPlatform:
    def __init__(self):
        self.running = {"anygrasp", "move_group", "ur_driver"}
        self.mode = "RUNNING"
        self.events = []
        self.power_off_error = False
        self.stop_errors = set()
        self.inspection_error_when_powered_off = False

    def component_running(self, name):
        if (
            self.inspection_error_when_powered_off
            and self.mode == "POWER_OFF"
            and name != "anygrasp"
        ):
            raise StopError("process table unavailable")
        return name in self.running

    def stop_component(self, name):
        self.events.append("stop:%s" % name)
        if name in self.stop_errors:
            raise StopError("cannot stop %s" % name)
        self.running.discard(name)

    def robot_mode(self):
        return self.mode

    def stop_program(self):
        self.events.append("dashboard:stop")

    def power_off(self):
        self.events.append("dashboard:power_off")
        if self.power_off_error:
            raise StopError("Dashboard unavailable")
        self.mode = "POWER_OFF"

    def wait_robot_mode(self, expected, timeout):
        return self.mode == expected


class StopCoordinatorTest(unittest.TestCase):
    def test_anygrasp_stop_leaves_every_ur3_component_running(self):
        platform = FakeRobotPlatform()

        result = StopCoordinator(platform).stop_anygrasp()

        self.assertTrue(result.success)
        self.assertNotIn("anygrasp", platform.running)
        self.assertEqual(platform.running, {"move_group", "ur_driver"})

    def test_ur3_stop_powers_off_before_driver_exit_and_keeps_anygrasp(self):
        platform = FakeRobotPlatform()
        platform.running = {
            "anygrasp",
            "rviz",
            "move_group",
            "d405_camera",
            "ag95_gripper",
            "ur_driver",
            "headless_supervisor",
        }

        result = StopCoordinator(platform).stop_ur3()

        self.assertTrue(result.success)
        self.assertTrue(result.ready_for_cabinet_power_button)
        self.assertEqual(platform.mode, "POWER_OFF")
        self.assertEqual(platform.running, {"anygrasp"})
        self.assertEqual(
            platform.events,
            [
                "stop:rviz",
                "stop:move_group",
                "dashboard:stop",
                "dashboard:power_off",
                "stop:d405_camera",
                "stop:ag95_gripper",
                "stop:ur_driver",
                "stop:headless_supervisor",
            ],
        )

    def test_dashboard_failure_still_removes_ur3_software_but_is_not_ready(self):
        platform = FakeRobotPlatform()
        platform.running = {
            "anygrasp",
            "rviz",
            "move_group",
            "d405_camera",
            "ag95_gripper",
            "ur_driver",
            "headless_supervisor",
        }
        platform.power_off_error = True

        result = StopCoordinator(platform).stop_ur3()

        self.assertFalse(result.success)
        self.assertFalse(result.ready_for_cabinet_power_button)
        self.assertEqual(platform.running, {"anygrasp"})

    def test_one_process_failure_does_not_skip_the_rest_of_the_shutdown(self):
        platform = FakeRobotPlatform()
        platform.running = {
            "anygrasp",
            "rviz",
            "move_group",
            "d405_camera",
            "ag95_gripper",
            "ur_driver",
            "headless_supervisor",
        }
        platform.stop_errors = {"move_group"}

        result = StopCoordinator(platform).stop_ur3()

        self.assertFalse(result.success)
        self.assertFalse(result.ready_for_cabinet_power_button)
        self.assertEqual(platform.running, {"anygrasp", "move_group"})
        self.assertIn("cannot stop move_group", result.error)
        self.assertEqual(platform.mode, "POWER_OFF")
        self.assertIn("stop:headless_supervisor", platform.events)

    def test_final_process_inspection_failure_suppresses_cabinet_clearance(self):
        platform = FakeRobotPlatform()
        platform.inspection_error_when_powered_off = True

        result = StopCoordinator(platform).stop_ur3()

        self.assertFalse(result.success)
        self.assertFalse(result.ready_for_cabinet_power_button)
        self.assertIn("process table unavailable", result.error)


class ProcessClassificationTest(unittest.TestCase):
    def test_classifies_only_known_launches_and_entry_points(self):
        cases = {
            "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch anygrasp_ros anygrasp_d405.launch": "anygrasp",
            "/opt/conda/bin/python /workspace/src/anygrasp_ros/scripts/anygrasp_d405_node.py __name:=anygrasp_d405_node": "anygrasp",
            "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch moveit_config moveit_rviz.launch": "rviz",
            "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch tracer_bringup ur3_moveit_execution.launch": "move_group",
            "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch tracer_bringup ur3_d405_camera.launch": "d405_camera",
            "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch tracer_bringup ag95_gripper_state.launch": "ag95_gripper",
            "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch tracer_bringup ur3_headless_driver.launch": "ur_driver",
            "/usr/bin/python3 /workspace/tracer_bringup/scripts/ur3_headless_moveit.py": "headless_supervisor",
        }
        for command, expected in cases.items():
            with self.subTest(command=command):
                self.assertEqual(classify_process(command), expected)

    def test_does_not_classify_unrelated_commands_that_mention_anygrasp(self):
        unrelated = (
            "python3 test_anygrasp_launch_config.py",
            "grep -i anygrasp",
            "nano src/anygrasp_ros/README.md",
            "python3 unrelated_graspnet_demo.py",
            "nano /workspace/tracer_bringup/scripts/ur3_headless_moveit.py",
            "python3 /workspace/ur3_headless_moveit.py",
            "grep roslaunch tracer_bringup ur3_headless_driver.launch",
            "pytest --file /opt/ros/noetic/bin/roslaunch tracer_bringup ur3_headless_driver.launch",
        )
        for command in unrelated:
            with self.subTest(command=command):
                self.assertIsNone(classify_process(command))


class FakeLinuxState:
    def __init__(self):
        self.nodes = {"/anygrasp_d405_node", "/ur/ur_hardware_interface"}
        self.processes = {
            100: (100, "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch anygrasp_ros anygrasp_d405.launch"),
            200: (200, "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch tracer_bringup ur3_headless_driver.launch"),
        }
        self.remove_on_signal = signal.SIGINT
        self.signal_history = []
        self.rosnode_timeout = False
        self.ps_timeout = False
        self.parent_only_on_sigint = False

    def run(self, command, timeout):
        if command == ["rosnode", "list"]:
            if self.rosnode_timeout:
                raise subprocess.TimeoutExpired(command, timeout)
            return subprocess.CompletedProcess(
                command, 0, stdout="\n".join(sorted(self.nodes)) + "\n", stderr=""
            )
        if command[:2] == ["rosnode", "kill"]:
            self.nodes.discard(command[2])
            return subprocess.CompletedProcess(command, 0, stdout="", stderr="")
        if command == ["ps", "-eo", "pid=,pgid=,args="]:
            if self.ps_timeout:
                raise subprocess.TimeoutExpired(command, timeout)
            output = "".join(
                "%d %d %s\n" % (pid, pgid, args)
                for pid, (pgid, args) in sorted(self.processes.items())
            )
            return subprocess.CompletedProcess(command, 0, stdout=output, stderr="")
        raise AssertionError("unexpected command: %r" % (command,))

    def signal_group(self, pgid, requested_signal):
        self.signal_history.append(requested_signal)
        if self.parent_only_on_sigint and requested_signal == signal.SIGINT:
            self.processes = {
                pid: record
                for pid, record in self.processes.items()
                if record[0] != pgid or classify_process(record[1]) is None
            }
        elif requested_signal >= self.remove_on_signal:
            self.processes = {
                pid: record
                for pid, record in self.processes.items()
                if record[0] != pgid
            }

    def process_group_alive(self, pgid):
        return any(record[0] == pgid for record in self.processes.values())


class LinuxRobotPlatformTest(unittest.TestCase):
    def test_process_table_timeout_is_unknown_not_stopped(self):
        state = FakeLinuxState()
        state.nodes = set()
        state.ps_timeout = True
        platform = LinuxRobotPlatform(
            runner=state.run,
            signal_group=state.signal_group,
            process_group_alive=state.process_group_alive,
            current_process_group=999,
        )

        with self.assertRaisesRegex(StopError, "process table"):
            platform.component_running("anygrasp")

    def test_ros_master_timeout_falls_back_to_the_process_table(self):
        state = FakeLinuxState()
        state.rosnode_timeout = True
        platform = LinuxRobotPlatform(
            runner=state.run,
            signal_group=state.signal_group,
            process_group_alive=state.process_group_alive,
            current_process_group=999,
        )

        self.assertTrue(platform.component_running("anygrasp"))

    def test_stopping_anygrasp_removes_its_real_node_and_process_group_only(self):
        state = FakeLinuxState()
        platform = LinuxRobotPlatform(
            runner=state.run,
            signal_group=state.signal_group,
            process_group_alive=state.process_group_alive,
            current_process_group=999,
        )

        platform.stop_component("anygrasp")

        self.assertFalse(platform.component_running("anygrasp"))
        self.assertTrue(platform.component_running("ur_driver"))
        self.assertEqual(state.signal_history, [signal.SIGINT])

    def test_stubborn_process_group_escalates_from_interrupt_to_terminate(self):
        state = FakeLinuxState()
        state.remove_on_signal = signal.SIGTERM
        platform = LinuxRobotPlatform(
            runner=state.run,
            signal_group=state.signal_group,
            process_group_alive=state.process_group_alive,
            current_process_group=999,
            shutdown_timeouts=(0.0, 0.0, 0.0),
        )

        platform.stop_component("anygrasp")

        self.assertFalse(platform.component_running("anygrasp"))
        self.assertEqual(state.signal_history, [signal.SIGINT, signal.SIGTERM])

    def test_escalation_tracks_original_group_after_classified_parent_exits(self):
        state = FakeLinuxState()
        state.processes[101] = (100, "/usr/bin/rviz --display-config unrelated.rviz")
        state.parent_only_on_sigint = True
        state.remove_on_signal = signal.SIGTERM
        platform = LinuxRobotPlatform(
            runner=state.run,
            signal_group=state.signal_group,
            process_group_alive=state.process_group_alive,
            current_process_group=999,
            shutdown_timeouts=(0.0, 0.0, 0.0),
        )

        platform.stop_component("anygrasp")

        self.assertFalse(state.process_group_alive(100))
        self.assertTrue(state.process_group_alive(200))
        self.assertEqual(state.signal_history, [signal.SIGINT, signal.SIGTERM])


class StatefulDashboard:
    def __init__(self):
        self.mode = "RUNNING"
        self.program_state = "PLAYING"
        self.commands = []

    def query(self, command):
        if command == "robotmode":
            return "Robotmode: %s" % self.mode
        if command == "programState":
            return "Program state: %s" % self.program_state
        raise AssertionError("unexpected query: %s" % command)

    def command(self, command):
        self.commands.append(command)
        if command == "stop":
            self.program_state = "STOPPED"
        elif command == "power off":
            self.mode = "POWER_OFF"
        else:
            raise AssertionError("unexpected command: %s" % command)
        return "OK"


class LinuxRobotDashboardTest(unittest.TestCase):
    def test_dashboard_stop_reaches_stopped_program_and_power_off_mode(self):
        dashboard = StatefulDashboard()
        state = FakeLinuxState()
        platform = LinuxRobotPlatform(
            runner=state.run,
            dashboard=dashboard,
            current_process_group=999,
        )

        platform.stop_program()
        platform.power_off()

        self.assertTrue(platform.wait_robot_mode("POWER_OFF", timeout=0.0))
        self.assertEqual(dashboard.program_state, "STOPPED")
        self.assertEqual(dashboard.commands, ["stop", "power off"])


class RobotStopCliTest(unittest.TestCase):
    def test_installed_entry_point_exposes_the_three_commands(self):
        script = os.path.join(PACKAGE_ROOT, "scripts", "robot_stop.py")

        result = subprocess.run(
            [sys.executable, script, "--help"],
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("{status,anygrasp,ur3}", result.stdout)

    def test_new_terminal_wrapper_loads_ros_before_showing_help(self):
        script = os.path.join(PACKAGE_ROOT, "scripts", "robot_stop.sh")
        clean_environment = {
            "HOME": os.path.expanduser("~"),
            "PATH": "/usr/bin:/bin",
        }

        result = subprocess.run(
            ["bash", script, "--help"],
            capture_output=True,
            text=True,
            env=clean_environment,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("{status,anygrasp,ur3}", result.stdout)

    def test_status_reports_anygrasp_and_ur3_independently(self):
        platform = FakeRobotPlatform()
        lines = []

        exit_code = main(["status"], platform=platform, output=lines.append)

        self.assertEqual(exit_code, 0)
        self.assertEqual(
            lines,
            [
                "AnyGrasp: RUNNING",
                "UR3 control chain: RUNNING (move_group, ur_driver)",
                "UR robot mode: RUNNING",
            ],
        )

    def test_anygrasp_command_reports_stop_without_changing_ur3(self):
        platform = FakeRobotPlatform()
        lines = []

        exit_code = main(["anygrasp"], platform=platform, output=lines.append)

        self.assertEqual(exit_code, 0)
        self.assertEqual(lines, ["AnyGrasp: STOPPED"])
        self.assertEqual(platform.running, {"move_group", "ur_driver"})
        self.assertEqual(platform.mode, "RUNNING")

    def test_ur3_command_reports_when_the_cabinet_button_is_safe_to_press(self):
        platform = FakeRobotPlatform()
        lines = []

        exit_code = main(["ur3"], platform=platform, output=lines.append)

        self.assertEqual(exit_code, 0)
        self.assertEqual(
            lines,
            [
                "UR3 control chain: STOPPED",
                "UR robot mode: POWER_OFF",
                "可以按机械臂控制柜的关闭按钮。",
                "AnyGrasp was not stopped: RUNNING",
            ],
        )

    def test_ur3_command_explains_dashboard_failure_without_clearance(self):
        platform = FakeRobotPlatform()
        platform.power_off_error = True
        lines = []

        exit_code = main(["ur3"], platform=platform, output=lines.append)

        self.assertEqual(exit_code, 1)
        self.assertIn("Shutdown error: Dashboard unavailable", lines)
        self.assertFalse(any(line.startswith("可以按") for line in lines))

    def test_final_dashboard_disconnect_revokes_cabinet_clearance(self):
        class DisconnectAfterPowerOffPlatform(FakeRobotPlatform):
            def robot_mode(self):
                if self.mode == "POWER_OFF":
                    raise StopError("Dashboard disconnected")
                return self.mode

        platform = DisconnectAfterPowerOffPlatform()
        lines = []

        exit_code = main(["ur3"], platform=platform, output=lines.append)

        self.assertEqual(exit_code, 1)
        self.assertIn("UR robot mode: UNREACHABLE", lines)
        self.assertFalse(any(line.startswith("可以按") for line in lines))

    def test_final_process_inspection_error_returns_failure_instead_of_crashing(self):
        platform = FakeRobotPlatform()
        platform.inspection_error_when_powered_off = True
        lines = []

        exit_code = main(["ur3"], platform=platform, output=lines.append)

        self.assertEqual(exit_code, 1)
        self.assertTrue(any("process table unavailable" in line for line in lines))
        self.assertFalse(any(line.startswith("可以按") for line in lines))


if __name__ == "__main__":
    unittest.main()
