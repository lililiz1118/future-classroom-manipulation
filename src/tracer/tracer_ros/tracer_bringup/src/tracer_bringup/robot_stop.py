#!/usr/bin/env python3
"""Safe, independently scoped shutdown operations for AnyGrasp and UR3."""

import argparse
from dataclasses import dataclass
import os
import shlex
import signal
import subprocess
import time
from typing import Any

from .headless_dashboard import DashboardClient, DashboardError, parse_robot_mode


class StopError(RuntimeError):
    """A robot or process boundary could not complete a stop operation."""


LAUNCH_COMPONENTS = {
    ("anygrasp_ros", "anygrasp_d405.launch"): "anygrasp",
    ("moveit_config", "moveit_rviz.launch"): "rviz",
    ("tracer_bringup", "ur3_moveit_execution.launch"): "move_group",
    ("tracer_bringup", "ur3_d405_camera.launch"): "d405_camera",
    ("tracer_bringup", "ag95_gripper_state.launch"): "ag95_gripper",
    ("tracer_bringup", "ur3_headless_driver.launch"): "ur_driver",
}
UR3_COMPONENTS = (
    "rviz",
    "move_group",
    "d405_camera",
    "ag95_gripper",
    "ur_driver",
    "headless_supervisor",
)

COMPONENT_NODES = {
    "anygrasp": {"/anygrasp_d405_node"},
    "move_group": {"/move_group"},
    "d405_camera": {
        "/d405/realsense2_camera",
        "/d405/realsense2_camera_manager",
        "/d405_to_plate",
    },
    "ag95_gripper": {"/dh_gripper_driver"},
    "ur_driver": {
        "/controller_stopper",
        "/joint_state_aggregator",
        "/robot_state_publisher",
        "/ur/ros_control_controller_spawner",
        "/ur/ur_hardware_interface",
        "/ur/ur_hardware_interface/ur_robot_state_helper",
    },
}
COMPONENT_NODE_PREFIXES = {
    "rviz": ("/rviz_",),
    "ur_driver": ("/ur/",),
}


def classify_process(command: str):
    try:
        tokens = shlex.split(command)
    except ValueError:
        return None
    if not tokens:
        return None
    executable = os.path.basename(tokens[0])
    is_python = executable.startswith("python")
    if is_python and len(tokens) >= 2:
        script = tokens[1].replace("\\", "/")
        if script.endswith("/anygrasp_ros/scripts/anygrasp_d405_node.py"):
            return "anygrasp"
        if script.endswith("/tracer_bringup/scripts/ur3_headless_moveit.py"):
            return "headless_supervisor"
    launch_index = None
    if executable == "roslaunch":
        launch_index = 0
    elif is_python and len(tokens) >= 2 and os.path.basename(tokens[1]) == "roslaunch":
        launch_index = 1
    if launch_index is not None and launch_index + 2 < len(tokens):
        return LAUNCH_COMPONENTS.get(
            (
                tokens[launch_index + 1],
                os.path.basename(tokens[launch_index + 2]),
            )
        )
    return None


@dataclass(frozen=True)
class ProcessRecord:
    pid: int
    process_group: int
    command: str


def _run_command(command, timeout):
    return subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def _process_group_exists(process_group):
    try:
        os.killpg(process_group, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


class LinuxRobotPlatform:
    def __init__(
        self,
        runner=_run_command,
        signal_group=os.killpg,
        current_process_group=None,
        shutdown_timeouts=(8.0, 2.0, 1.0),
        monotonic=time.monotonic,
        sleeper=time.sleep,
        dashboard=None,
        process_group_alive=_process_group_exists,
    ):
        self.runner = runner
        self.signal_group = signal_group
        self.current_process_group = (
            os.getpgrp() if current_process_group is None else current_process_group
        )
        self.shutdown_timeouts = shutdown_timeouts
        self.monotonic = monotonic
        self.sleeper = sleeper
        self.dashboard = dashboard or DashboardClient("192.168.131.3")
        self.process_group_alive = process_group_alive

    def _run(self, command, timeout):
        try:
            return self.runner(command, timeout)
        except subprocess.TimeoutExpired as exc:
            return subprocess.CompletedProcess(
                command,
                124,
                stdout=exc.stdout or "",
                stderr=exc.stderr or "timeout",
            )
        except OSError as exc:
            return subprocess.CompletedProcess(
                command,
                127,
                stdout="",
                stderr=str(exc),
            )

    def _nodes(self):
        result = self._run(["rosnode", "list"], 4.0)
        if result.returncode != 0:
            return set()
        return {line.strip() for line in result.stdout.splitlines() if line.strip()}

    def _processes(self):
        result = self._run(["ps", "-eo", "pid=,pgid=,args="], 4.0)
        if result.returncode != 0:
            raise StopError(
                "Cannot inspect process table: %s"
                % (result.stderr or result.stdout or "unknown error")
            )
        records = []
        for line in result.stdout.splitlines():
            fields = line.strip().split(None, 2)
            if len(fields) != 3:
                continue
            try:
                records.append(ProcessRecord(int(fields[0]), int(fields[1]), fields[2]))
            except ValueError:
                continue
        return records

    @staticmethod
    def _node_belongs_to(component, node):
        if node in COMPONENT_NODES.get(component, set()):
            return True
        return any(
            node.startswith(prefix)
            for prefix in COMPONENT_NODE_PREFIXES.get(component, ())
        )

    def component_running(self, component):
        if any(self._node_belongs_to(component, node) for node in self._nodes()):
            return True
        return any(
            classify_process(record.command) == component
            for record in self._processes()
        )

    def _component_process_groups(self, component):
        return {
            record.process_group
            for record in self._processes()
            if classify_process(record.command) == component
            and record.process_group != self.current_process_group
        }

    def _wait_for_process_groups_exit(self, process_groups, timeout):
        deadline = self.monotonic() + timeout
        remaining = {
            process_group
            for process_group in process_groups
            if self.process_group_alive(process_group)
        }
        while True:
            if not remaining:
                return set()
            if self.monotonic() >= deadline:
                return remaining
            self.sleeper(min(0.05, max(0.0, deadline - self.monotonic())))
            remaining = {
                process_group
                for process_group in remaining
                if self.process_group_alive(process_group)
            }

    def stop_component(self, component):
        remaining_groups = self._component_process_groups(component)
        for node in sorted(self._nodes()):
            if self._node_belongs_to(component, node):
                self._run(["rosnode", "kill", node], 8.0)
        for requested_signal, timeout in zip(
            (signal.SIGINT, signal.SIGTERM, signal.SIGKILL),
            self.shutdown_timeouts,
        ):
            if not remaining_groups:
                return
            for process_group in sorted(remaining_groups):
                try:
                    self.signal_group(process_group, requested_signal)
                except ProcessLookupError:
                    pass
                except PermissionError as exc:
                    raise StopError(
                        "Cannot signal process group %d for %s"
                        % (process_group, component)
                    ) from exc
            remaining_groups = self._wait_for_process_groups_exit(
                remaining_groups, timeout
            )
        if remaining_groups:
            raise StopError(
                "%s process groups still running after SIGKILL: %s"
                % (component, ", ".join(str(group) for group in sorted(remaining_groups)))
            )

    def robot_mode(self):
        try:
            return parse_robot_mode(self.dashboard.query("robotmode"))
        except DashboardError as exc:
            raise StopError(str(exc)) from exc

    def stop_program(self):
        try:
            self.dashboard.command("stop")
            deadline = self.monotonic() + 10.0
            while True:
                state = self.dashboard.query("programState").upper()
                if "STOPPED" in state:
                    return
                if self.monotonic() >= deadline:
                    raise StopError(
                        "UR program did not reach STOPPED; last=%s" % state
                    )
                self.sleeper(min(0.25, max(0.0, deadline - self.monotonic())))
        except DashboardError as exc:
            raise StopError(str(exc)) from exc

    def power_off(self):
        try:
            self.dashboard.command("power off")
        except DashboardError as exc:
            raise StopError(str(exc)) from exc

    def wait_robot_mode(self, expected, timeout):
        deadline = self.monotonic() + timeout
        while True:
            if self.robot_mode() == expected:
                return True
            if self.monotonic() >= deadline:
                return False
            self.sleeper(min(0.25, max(0.0, deadline - self.monotonic())))


@dataclass(frozen=True)
class StopResult:
    success: bool
    ready_for_cabinet_power_button: bool = False
    error: str = ""


@dataclass(frozen=True)
class StatusResult:
    anygrasp_running: bool
    ur3_running_components: tuple
    robot_mode: str
    error: str = ""


class StopCoordinator:
    def __init__(self, platform: Any):
        self.platform = platform

    def stop_anygrasp(self) -> StopResult:
        try:
            self.platform.stop_component("anygrasp")
        except StopError as exc:
            return StopResult(success=False, error=str(exc))
        return StopResult(success=not self.platform.component_running("anygrasp"))

    def status(self) -> StatusResult:
        errors = []
        try:
            mode = self.platform.robot_mode()
        except StopError as exc:
            mode = "UNREACHABLE"
            errors.append(str(exc))
        try:
            anygrasp_running = self.platform.component_running("anygrasp")
            ur3_components = tuple(
                component
                for component in UR3_COMPONENTS
                if self.platform.component_running(component)
            )
        except StopError as exc:
            anygrasp_running = False
            ur3_components = ("UNKNOWN",)
            errors.append(str(exc))
        return StatusResult(
            anygrasp_running=anygrasp_running,
            ur3_running_components=ur3_components,
            robot_mode=mode,
            error="; ".join(errors),
        )

    def stop_ur3(self) -> StopResult:
        errors = []

        def stop_component(component):
            try:
                self.platform.stop_component(component)
            except StopError as exc:
                errors.append(str(exc))

        stop_component("rviz")
        stop_component("move_group")

        try:
            if self.platform.robot_mode() != "POWER_OFF":
                self.platform.stop_program()
                self.platform.power_off()
            powered_off = self.platform.wait_robot_mode("POWER_OFF", timeout=20.0)
        except StopError as exc:
            powered_off = False
            errors.append(str(exc))
        if not powered_off and not errors:
            errors.append("UR robot did not reach POWER_OFF")

        remaining_components = (
            "d405_camera",
            "ag95_gripper",
            "ur_driver",
            "headless_supervisor",
        )
        for component in remaining_components:
            stop_component(component)

        try:
            software_stopped = not any(
                self.platform.component_running(component)
                for component in (
                    "rviz",
                    "move_group",
                    *remaining_components,
                )
            )
        except StopError as exc:
            software_stopped = False
            errors.append(str(exc))
        verified_safe = software_stopped and powered_off and not errors
        return StopResult(
            success=verified_safe,
            ready_for_cabinet_power_button=verified_safe,
            error="; ".join(errors),
        )


def main(argv=None, platform=None, output=print) -> int:
    parser = argparse.ArgumentParser(
        prog="robot-stop",
        description="Safely stop AnyGrasp or the UR3 control chain",
    )
    parser.add_argument("command", choices=("status", "anygrasp", "ur3"))
    arguments = parser.parse_args(argv)
    coordinator = StopCoordinator(platform or LinuxRobotPlatform())

    if arguments.command == "anygrasp":
        result = coordinator.stop_anygrasp()
        output("AnyGrasp: %s" % ("STOPPED" if result.success else "STILL RUNNING"))
        return 0 if result.success else 1

    if arguments.command == "ur3":
        result = coordinator.stop_ur3()
        status = coordinator.status()
        final_verified = (
            result.success
            and status.robot_mode == "POWER_OFF"
            and not status.ur3_running_components
        )
        if final_verified:
            output("UR3 control chain: STOPPED")
            output("UR robot mode: %s" % status.robot_mode)
            output("可以按机械臂控制柜的关闭按钮。")
        else:
            output("UR3 shutdown: INCOMPLETE")
            output("UR robot mode: %s" % status.robot_mode)
            if result.error:
                output("Shutdown error: %s" % result.error)
            elif status.robot_mode != "POWER_OFF":
                output("Shutdown error: final POWER_OFF verification failed")
            elif status.ur3_running_components:
                output(
                    "Shutdown error: components still running: %s"
                    % ", ".join(status.ur3_running_components)
                )
            output("尚未确认 POWER_OFF；请勿按控制柜关闭按钮。")
        output(
            "AnyGrasp was not stopped: %s"
            % ("RUNNING" if status.anygrasp_running else "STOPPED")
        )
        return 0 if final_verified else 1

    if arguments.command == "status":
        status = coordinator.status()
        output(
            "AnyGrasp: %s" % ("RUNNING" if status.anygrasp_running else "STOPPED")
        )
        if status.ur3_running_components:
            output(
                "UR3 control chain: RUNNING (%s)"
                % ", ".join(status.ur3_running_components)
            )
        else:
            output("UR3 control chain: STOPPED")
        output("UR robot mode: %s" % status.robot_mode)
        if status.error:
            output("Inspection error: %s" % status.error)
        return 1 if status.error else 0

    parser.error("command is not implemented")
    return 2
