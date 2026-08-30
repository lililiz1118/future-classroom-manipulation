#!/usr/bin/env python3
"""ROS-facing runtime for the staged UR3 headless startup."""

import os
import signal
import socket
import stat
import subprocess
import time
from typing import Any, Dict, Iterable, List, Sequence
from urllib.parse import urlparse

from .headless_startup import (
    StartupConfig,
    StartupError,
    assert_exclusive_controller,
    validate_calibration,
)


REQUIRED_JOINTS = (
    "ur_arm_shoulder_pan_joint",
    "ur_arm_shoulder_lift_joint",
    "ur_arm_elbow_joint",
    "ur_arm_wrist_1_joint",
    "ur_arm_wrist_2_joint",
    "ur_arm_wrist_3_joint",
)
GRIPPER_JOINT = "gripper_finger1_joint"
TARGET_CONTROLLER = "ur_arm_scaled_pos_joint_traj_controller"
SPEED_SCALING_TOPIC = "/ur/speed_scaling_factor"
CONFLICTING_NODES = {
    "/ur/ur_hardware_interface",
    "/move_group",
    "/servo_server",
    "/keyboard_jog",
    "/dh_gripper_driver",
    "/gripper_joint_state_relay",
}
REQUIRED_UR_DRIVER_EXECUTABLES = (
    "ur_robot_driver_node",
    "controller_stopper_node",
    "robot_state_helper",
)


def assert_gripper_device_ready(path: str) -> None:
    if not os.path.exists(path):
        raise StartupError("AG95 device does not exist: %s" % path)
    resolved = os.path.realpath(path)
    try:
        mode = os.stat(resolved).st_mode
    except OSError as exc:
        raise StartupError("Cannot inspect AG95 device %s: %s" % (path, exc)) from exc
    if not stat.S_ISCHR(mode):
        raise StartupError("AG95 device is not a character device: %s" % path)
    if not os.access(path, os.R_OK | os.W_OK):
        raise StartupError("AG95 device is not readable and writable: %s" % path)


def assert_ros_network_environment(environment: Dict[str, str], reverse_ip: str) -> None:
    ros_ip = environment.get("ROS_IP", "")
    ros_hostname = environment.get("ROS_HOSTNAME", "")
    if ros_ip != reverse_ip:
        raise StartupError("ROS_IP must be %s, got %s" % (reverse_ip, ros_ip or "unset"))
    if ros_hostname:
        raise StartupError(
            "ROS_HOSTNAME must be unset when ROS_IP=%s is used; got %s"
            % (reverse_ip, ros_hostname)
        )


def assert_route_uses_reverse_ip(route_output: str, reverse_ip: str) -> None:
    tokens = route_output.split()
    if "src" not in tokens or tokens[tokens.index("src") + 1] != reverse_ip:
        raise StartupError(
            "Route to UR controller does not use ROS host address %s: %s"
            % (reverse_ip, route_output.strip())
        )


def assert_no_conflicting_nodes(nodes: Iterable[str]) -> None:
    conflicts = sorted(set(nodes) & CONFLICTING_NODES)
    if conflicts:
        raise StartupError("Existing UR control nodes must be stopped: %s" % ", ".join(conflicts))


def should_start_robot_state_publisher(nodes: Iterable[str]) -> bool:
    return "/robot_state_publisher" not in set(nodes)


def assert_joint_state_complete(message: Any) -> None:
    missing = sorted(set(REQUIRED_JOINTS) - set(message.name))
    if missing:
        raise StartupError("/joint_states is missing UR joints: %s" % ", ".join(missing))


def controller_snapshot(response: Any) -> List[Dict[str, Any]]:
    result = []
    for controller in response.controller:
        result.append(
            {
                "name": controller.name,
                "state": controller.state,
                "claimed_resources": [
                    {
                        "hardware_interface": claim.hardware_interface,
                        "resources": list(claim.resources),
                    }
                    for claim in controller.claimed_resources
                ],
            }
        )
    return result


class RosRuntime:
    def __init__(self, environment=None):
        self.environment = dict(environment or os.environ)
        self.processes = []
        self._rospy = None
        self.start_robot_state_publisher = True
        self.package_paths = {}

    def _run(self, command: Sequence[str], timeout: float = 10.0, required=True):
        try:
            result = subprocess.run(
                list(command),
                capture_output=True,
                text=True,
                timeout=timeout,
                env=self.environment,
            )
        except subprocess.TimeoutExpired as exc:
            if not required:
                return subprocess.CompletedProcess(
                    list(command), 124, stdout=exc.stdout or "", stderr=exc.stderr or "timeout"
                )
            raise StartupError("Command timed out: %s" % " ".join(command)) from exc
        except OSError as exc:
            raise StartupError("Command failed: %s: %s" % (" ".join(command), exc)) from exc
        if required and result.returncode != 0:
            raise StartupError(
                "Command failed (%d): %s\n%s"
                % (result.returncode, " ".join(command), (result.stderr or result.stdout).strip())
            )
        return result

    def preflight(self, config: StartupConfig) -> None:
        assert_ros_network_environment(self.environment, config.reverse_ip)
        validate_calibration(config.calibration_path, config.expected_calibration_hash)
        assert_gripper_device_ready(config.gripper_device)
        self._run(["ping", "-c", "1", "-W", "2", config.robot_ip], timeout=4.0)
        route = self._run(["ip", "route", "get", config.robot_ip], timeout=3.0)
        assert_route_uses_reverse_ip(route.stdout, config.reverse_ip)
        for package in (
            "tracer_bringup",
            "ur_robot_driver",
            "moveit_config",
            "dh_gripper_driver",
        ):
            result = self._run(["rospack", "find", package], timeout=5.0)
            self.package_paths[package] = result.stdout.strip()
        version = self._run(["rosversion", "ur_robot_driver"], timeout=5.0).stdout.strip()
        if version != "2.4.1":
            raise StartupError("Expected ur_robot_driver 2.4.1, got %s" % (version or "unknown"))
        for executable in REQUIRED_UR_DRIVER_EXECUTABLES:
            result = self._run(
                [
                    "rosrun",
                    "--prefix",
                    "/usr/bin/true",
                    "ur_robot_driver",
                    executable,
                ],
                timeout=5.0,
                required=False,
            )
            if result.returncode != 0:
                raise StartupError(
                    "Required ROS executable is unavailable: ur_robot_driver/%s\n%s"
                    % (executable, (result.stderr or result.stdout).strip())
                )
        result = self._run(
            [
                "rosrun",
                "--prefix",
                "/usr/bin/true",
                "dh_gripper_driver",
                "dh_gripper_driver",
            ],
            timeout=5.0,
            required=False,
        )
        if result.returncode != 0:
            raise StartupError(
                "Required ROS executable is unavailable: "
                "dh_gripper_driver/dh_gripper_driver\n%s"
                % (result.stderr or result.stdout).strip()
            )

    def assert_no_conflicts(self) -> None:
        result = self._run(["rosnode", "list"], timeout=4.0, required=False)
        if result.returncode == 0:
            nodes = [line.strip() for line in result.stdout.splitlines()]
            assert_no_conflicting_nodes(nodes)
            self.start_robot_state_publisher = should_start_robot_state_publisher(nodes)
            return
        diagnostics = str(result.stderr or result.stdout).lower()
        if not any(text in diagnostics for text in ("master", "connection refused", "unable to communicate")):
            raise StartupError("Cannot inspect existing ROS nodes: %s" % diagnostics.strip())

    def _launch(self, label: str, command: Sequence[str]) -> None:
        try:
            process = subprocess.Popen(
                list(command), env=self.environment, start_new_session=True
            )
        except OSError as exc:
            raise StartupError("Cannot start %s: %s" % (label, exc)) from exc
        self.processes.append((label, process))

    def start_driver(self, config: StartupConfig) -> None:
        self._launch(
            "ur_driver",
            [
                "roslaunch",
                "tracer_bringup",
                "ur3_headless_driver.launch",
                "robot_ip:=%s" % config.robot_ip,
                "reverse_ip:=%s" % config.reverse_ip,
                "kinematics_config:=%s" % config.calibration_path,
                "start_robot_state_publisher:=%s"
                % ("true" if self.start_robot_state_publisher else "false"),
            ],
        )

    def start_gripper(self, config: StartupConfig) -> None:
        self._launch(
            "ag95_gripper",
            [
                "roslaunch",
                "tracer_bringup",
                "ag95_gripper_state.launch",
                "gripper_device:=%s" % config.gripper_device,
            ],
        )

    def _wait_for_master(self, timeout: float) -> None:
        uri = self.environment.get("ROS_MASTER_URI", "http://localhost:11311")
        parsed = urlparse(uri)
        host = parsed.hostname or "localhost"
        port = parsed.port or 11311
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.processes and self.processes[0][1].poll() is not None:
                raise StartupError("UR Driver launch exited before ROS master became ready")
            try:
                with socket.create_connection((host, port), timeout=0.5):
                    return
            except OSError:
                time.sleep(0.25)
        raise StartupError("ROS master did not become reachable at %s" % uri)

    def _init_ros(self, timeout: float) -> None:
        if self._rospy is not None:
            return
        self._wait_for_master(timeout)
        import rospy

        rospy.init_node("ur3_headless_startup", anonymous=True, disable_signals=True)
        self._rospy = rospy

    def _wait_for_true(self, topic: str, message_type: Any, timeout: float) -> Any:
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                last = self._rospy.wait_for_message(
                    topic,
                    message_type,
                    timeout=max(0.1, min(1.0, deadline - time.monotonic())),
                )
            except self._rospy.ROSException:
                continue
            if bool(last.data):
                return last
        raise StartupError("Timed out waiting for true on %s; last=%s" % (topic, last))

    def _wait_for_complete_joint_state(
        self, topic: str, message_type: Any, timeout: float
    ) -> Any:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                message = self._rospy.wait_for_message(
                    topic,
                    message_type,
                    timeout=max(0.1, min(1.0, deadline - time.monotonic())),
                )
            except self._rospy.ROSException:
                continue
            if set(REQUIRED_JOINTS).issubset(message.name):
                return message
        raise StartupError("Timed out waiting for all six UR joints on %s" % topic)

    def _wait_for_named_joint_state(
        self,
        topic: str,
        message_type: Any,
        required_joints: Iterable[str],
        timeout: float,
    ) -> Any:
        required = set(required_joints)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                message = self._rospy.wait_for_message(
                    topic,
                    message_type,
                    timeout=max(0.1, min(1.0, deadline - time.monotonic())),
                )
            except self._rospy.ROSException:
                continue
            if required.issubset(message.name):
                return message
        raise StartupError(
            "Timed out waiting for joints %s on %s"
            % (", ".join(sorted(required)), topic)
        )

    def _wait_for_initialized_gripper(
        self, topic: str, message_type: Any, timeout: float
    ) -> Any:
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                last = self._rospy.wait_for_message(
                    topic,
                    message_type,
                    timeout=max(0.1, min(1.0, deadline - time.monotonic())),
                )
            except self._rospy.ROSException:
                continue
            if bool(last.is_initialized):
                return last
        raise StartupError(
            "AG95 did not report initialized=true on %s; last=%s" % (topic, last)
        )

    def _assert_fresh_advancing_joint_states(
        self, first: Any, second: Any, topic: str
    ) -> None:
        if second.header.stamp <= first.header.stamp:
            raise StartupError("%s timestamps are not advancing" % topic)
        age = (self._rospy.Time.now() - second.header.stamp).to_sec()
        if age < 0.0 or age > 1.0:
            raise StartupError("%s is stale by %.3f seconds" % (topic, age))

    def _wait_for_speed_range(
        self, topic: str, message_type: Any, requested: float, timeout: float
    ) -> Any:
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                last = self._rospy.wait_for_message(
                    topic,
                    message_type,
                    timeout=max(0.1, min(1.0, deadline - time.monotonic())),
                )
            except self._rospy.ROSException:
                continue
            if 0.0 < last.data <= requested + 0.02:
                return last
        raise StartupError(
            "Speed scaling did not reach the requested low-speed range; last=%s"
            % (None if last is None else last.data)
        )

    def wait_driver_ready(self, config: StartupConfig) -> None:
        self._init_ros(config.state_timeout)
        from controller_manager_msgs.srv import ListControllers
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64

        first = self._wait_for_complete_joint_state(
            "/joint_states", JointState, config.state_timeout
        )
        second = self._wait_for_complete_joint_state("/joint_states", JointState, 2.0)
        self._assert_fresh_advancing_joint_states(first, second, "/joint_states")

        self._wait_for_true(
            "/ur/ur_hardware_interface/robot_program_running",
            Bool,
            config.state_timeout,
        )
        service_name = "/ur/controller_manager/list_controllers"
        self._rospy.wait_for_service(service_name, timeout=config.state_timeout)
        response = self._rospy.ServiceProxy(service_name, ListControllers)()
        assert_exclusive_controller(
            controller_snapshot(response), TARGET_CONTROLLER, REQUIRED_JOINTS
        )
        scaling = self._rospy.wait_for_message(
            SPEED_SCALING_TOPIC, Float64, timeout=config.state_timeout
        )
        if not 0.0 < scaling.data <= 1.0:
            raise StartupError("Invalid speed scaling value before startup: %s" % scaling.data)
        actual_hash = self._rospy.get_param("/ur/ur_hardware_interface/kinematics/hash", "")
        if actual_hash != config.expected_calibration_hash:
            raise StartupError(
                "Driver loaded calibration %s instead of %s"
                % (actual_hash, config.expected_calibration_hash)
            )

    def wait_gripper_ready(self, config: StartupConfig) -> None:
        from dh_gripper_msgs.msg import GripperState
        from sensor_msgs.msg import JointState

        self._wait_for_initialized_gripper(
            "/gripper/states", GripperState, config.state_timeout
        )
        first = self._wait_for_named_joint_state(
            "/gripper/joint_states", JointState, {GRIPPER_JOINT}, config.state_timeout
        )
        second = self._wait_for_named_joint_state(
            "/gripper/joint_states", JointState, {GRIPPER_JOINT}, 2.0
        )
        self._assert_fresh_advancing_joint_states(
            first, second, "/gripper/joint_states"
        )
        self._wait_for_named_joint_state(
            "/joint_states", JointState, {GRIPPER_JOINT}, 5.0
        )

    def set_speed_slider(self, fraction: float) -> None:
        from std_msgs.msg import Float64
        from ur_msgs.srv import SetSpeedSliderFraction

        service_name = "/ur/ur_hardware_interface/set_speed_slider"
        self._rospy.wait_for_service(service_name, timeout=10.0)
        response = self._rospy.ServiceProxy(service_name, SetSpeedSliderFraction)(fraction)
        if not response.success:
            raise StartupError("UR Driver rejected speed slider %.3f" % fraction)
        self._wait_for_speed_range(SPEED_SCALING_TOPIC, Float64, fraction, 5.0)

    def start_move_group(self, config: StartupConfig) -> None:
        self._launch(
            "move_group",
            ["roslaunch", "tracer_bringup", "ur3_moveit_execution.launch"],
        )

    def wait_move_group_ready(self, config: StartupConfig) -> None:
        from actionlib_msgs.msg import GoalStatusArray

        self._rospy.wait_for_service("/get_planning_scene", timeout=config.state_timeout)
        self._rospy.wait_for_message(
            "/move_group/status", GoalStatusArray, timeout=config.state_timeout
        )

    def start_rviz(self, config: StartupConfig) -> None:
        rviz_config = os.path.join(
            self.package_paths["moveit_config"], "launch", "moveit.rviz"
        )
        self._launch(
            "rviz",
            [
                "roslaunch",
                "moveit_config",
                "moveit_rviz.launch",
                "rviz_config:=%s" % rviz_config,
            ],
        )

    def supervise(self) -> None:
        while True:
            for label, process in self.processes:
                code = process.poll()
                if code is None:
                    continue
                if label == "rviz" and code == 0:
                    return
                raise StartupError("%s exited unexpectedly with code %d" % (label, code))
            time.sleep(0.5)

    def shutdown(self) -> None:
        for _, process in reversed(self.processes):
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGINT)
                except (OSError, ProcessLookupError):
                    pass
        deadline = time.monotonic() + 8.0
        for _, process in reversed(self.processes):
            remaining = max(0.0, deadline - time.monotonic())
            if process.poll() is None:
                try:
                    process.wait(timeout=remaining)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    except (OSError, ProcessLookupError):
                        pass
