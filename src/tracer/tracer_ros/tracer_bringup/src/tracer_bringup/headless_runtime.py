#!/usr/bin/env python3
"""ROS-facing runtime for the staged UR3 headless startup."""

import os
import signal
import socket
import stat
import subprocess
import sys
import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Sequence
from urllib.parse import urlparse

from .control_chain_monitor import (
    ControlChainFault,
    ControlChainNotReady,
    RosControlChainMonitor,
    controller_snapshot,
)
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
    "/joint_state_aggregator",
}
D405_NODES = {
    "/d405/realsense2_camera",
    "/d405/realsense2_camera_manager",
}
D455_NODES = {
    "/d455/realsense2_camera",
    "/d455/realsense2_camera_manager",
}
D405_NODELET_LIBRARY = "librealsense2_camera.so"
REQUIRED_ROS_EXECUTABLES = (
    ("ur_robot_driver", "ur_robot_driver_node"),
    ("ur_robot_driver", "controller_stopper_node"),
    ("ur_robot_driver", "robot_state_helper"),
    ("dh_gripper_driver", "dh_gripper_driver"),
)
COMPONENT_NAMES = {
    "ur_driver": "UR3 驱动",
    "ag95_gripper": "AG95 夹爪",
    "d405_camera": "D405 相机",
    "move_group": "MoveIt",
    "rviz": "RViz",
}
ERROR_MARKERS = (
    "ERROR",
    "FATAL",
    "TRACEBACK",
    "EXCEPTION",
    "FAILED",
    "FAILURE",
    "HAS DIED",
    "EXIT CODE",
    "LOG FILE:",
)
WARNING_MARKERS = ("WARN", "WARNING")
SHUTDOWN_SIGINT_TIMEOUT = 8.0
SHUTDOWN_SIGTERM_TIMEOUT = 2.0
SHUTDOWN_SIGKILL_TIMEOUT = 1.0
CONTROLLER_SPAWNER_NODE = "/ur/ros_control_controller_spawner"
SHUTDOWN_PROCESS_ORDER = (
    "rviz",
    "move_group",
    "d405_camera",
    "ag95_gripper",
    "ur_driver",
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
    try:
        route_source = tokens[tokens.index("src") + 1]
    except (ValueError, IndexError):
        route_source = None
    if route_source != reverse_ip:
        raise StartupError(
            "Route to UR controller does not use ROS host address %s: %s"
            % (reverse_ip, route_output.strip())
        )


def assert_no_conflicting_nodes(nodes: Iterable[str]) -> None:
    conflicts = sorted(set(nodes) & CONFLICTING_NODES)
    if conflicts:
        raise StartupError("Existing UR control nodes must be stopped: %s" % ", ".join(conflicts))


def classify_d405_nodes(nodes: Iterable[str], enable_d405: bool) -> str:
    node_set = set(nodes)
    d455 = sorted(node_set & D455_NODES)
    if d455:
        raise StartupError(
            "D455 nodes must be stopped before headless manipulation: %s"
            % ", ".join(d455)
        )
    if not enable_d405:
        return "disabled"
    present = node_set & D405_NODES
    if not present:
        return "absent"
    if present == D405_NODES:
        return "external"
    missing = sorted(D405_NODES - present)
    raise StartupError(
        "Incomplete D405 node set; stop the old D405 launch before retrying; "
        "present=%s missing=%s"
        % (", ".join(sorted(present)), ", ".join(missing))
    )


def assert_d405_nodelet_library_available(environment: Dict[str, str]) -> None:
    search_directories = []
    for directory in environment.get("LD_LIBRARY_PATH", "").split(os.pathsep):
        if directory:
            search_directories.append(directory)
    for prefix in environment.get("CMAKE_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            search_directories.append(os.path.join(prefix, "lib"))

    checked = []
    for directory in search_directories:
        candidate = os.path.join(directory, D405_NODELET_LIBRARY)
        if candidate in checked:
            continue
        checked.append(candidate)
        if os.path.isfile(candidate):
            return

    locations = ", ".join(checked) if checked else "no ROS library paths configured"
    raise StartupError(
        "Required D405 nodelet library %s is missing from the active workspace "
        "(%s); rebuild the worktree with realsense2_camera included in "
        "CATKIN_WHITELIST_PACKAGES, then run catkin_make"
        % (D405_NODELET_LIBRARY, locations)
    )


def should_start_robot_state_publisher(nodes: Iterable[str]) -> bool:
    return "/robot_state_publisher" not in set(nodes)


class RosRuntime:
    def __init__(
        self,
        environment=None,
        diagnostic_output=None,
        control_chain_monitor_factory=RosControlChainMonitor,
    ):
        self.environment = dict(environment or os.environ)
        self.processes = []
        self.diagnostic_output = diagnostic_output or self._print_diagnostic
        self._diagnostic_threads = []
        self._rospy = None
        self.start_robot_state_publisher = True
        self.d405_state = "absent"
        self.package_paths = {}
        self.control_chain_monitor = None
        self._control_chain_monitor_factory = control_chain_monitor_factory
        self._health_evaluation_period = 0.10

    @staticmethod
    def _print_diagnostic(message: str) -> None:
        print(message, file=sys.stderr)

    def _emit_diagnostic(self, message: str) -> None:
        try:
            self.diagnostic_output(message)
        except Exception:
            # Presentation must never stop draining a child's stderr pipe.
            pass

    def _relay_child_diagnostics(self, label: str, stream: Any) -> None:
        component = COMPONENT_NAMES.get(label, label)
        active_kind = None
        try:
            for raw_line in stream:
                line = raw_line.strip()
                if not line:
                    active_kind = None
                    continue
                normalized = line.upper()
                if any(marker in normalized for marker in ERROR_MARKERS):
                    active_kind = ("❌", "报错详情")
                    self._emit_diagnostic(
                        "❌ [%s] 节点报错｜原文: %s" % (component, line)
                    )
                elif any(marker in normalized for marker in WARNING_MARKERS):
                    active_kind = ("⚠️", "警告详情")
                    self._emit_diagnostic(
                        "⚠️ [%s] 节点警告｜原文: %s" % (component, line)
                    )
                elif active_kind is not None:
                    icon, summary = active_kind
                    self._emit_diagnostic(
                        "%s [%s] %s｜原文: %s"
                        % (icon, component, summary, line)
                    )
        finally:
            stream.close()

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

    def _assert_ros_executable_available(self, package: str, executable: str) -> None:
        result = self._run(
            [
                "rosrun",
                "--prefix",
                "/usr/bin/true",
                package,
                executable,
            ],
            timeout=5.0,
            required=False,
        )
        if result.returncode != 0:
            raise StartupError(
                "Required ROS executable is unavailable: %s/%s\n%s"
                % (
                    package,
                    executable,
                    (result.stderr or result.stdout).strip(),
                )
            )

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
            "realsense2_camera",
        ):
            result = self._run(["rospack", "find", package], timeout=5.0)
            self.package_paths[package] = result.stdout.strip()
        version = self._run(["rosversion", "ur_robot_driver"], timeout=5.0).stdout.strip()
        if version != "2.4.1":
            raise StartupError("Expected ur_robot_driver 2.4.1, got %s" % (version or "unknown"))
        for package, executable in REQUIRED_ROS_EXECUTABLES:
            self._assert_ros_executable_available(package, executable)

    def assert_no_conflicts(self, config: StartupConfig) -> None:
        result = self._run(["rosnode", "list"], timeout=4.0, required=False)
        if result.returncode == 0:
            nodes = [line.strip() for line in result.stdout.splitlines()]
            assert_no_conflicting_nodes(nodes)
            self.start_robot_state_publisher = should_start_robot_state_publisher(nodes)
            self.d405_state = classify_d405_nodes(nodes, config.enable_d405)
        else:
            diagnostics = str(result.stderr or result.stdout).lower()
            if not any(
                text in diagnostics
                for text in ("master", "connection refused", "unable to communicate")
            ):
                raise StartupError("Cannot inspect existing ROS nodes: %s" % diagnostics.strip())
            self.d405_state = "disabled" if not config.enable_d405 else "absent"
        if self.d405_state == "absent":
            assert_d405_nodelet_library_available(self.environment)

    def _launch(self, label: str, command: Sequence[str]) -> None:
        try:
            process = subprocess.Popen(
                list(command),
                env=self.environment,
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                errors="replace",
                bufsize=1,
            )
        except OSError as exc:
            raise StartupError("Cannot start %s: %s" % (label, exc)) from exc
        self.processes.append((label, process))
        diagnostic_thread = threading.Thread(
            target=self._relay_child_diagnostics,
            args=(label, process.stderr),
            daemon=True,
        )
        diagnostic_thread.start()
        self._diagnostic_threads.append(diagnostic_thread)

    def start_driver(self, config: StartupConfig) -> None:
        self._launch(
            "ur_driver",
            [
                "roslaunch",
                "tracer_bringup",
                "ur3_headless_driver.launch",
                "robot_ip:=%s" % config.robot_ip,
                "reverse_ip:=%s" % config.reverse_ip,
                "robot_receive_timeout:=%.2f"
                % config.runtime_policy.robot_receive_timeout,
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
                "publish_joint_state_relay:=false",
            ],
        )

    def start_d405(self, config: StartupConfig) -> None:
        if not config.enable_d405 or self.d405_state == "external":
            return
        if self.d405_state != "absent":
            raise StartupError("Cannot start D405 from state: %s" % self.d405_state)
        self._launch(
            "d405_camera",
            ["roslaunch", "tracer_bringup", "ur3_d405_camera.launch"],
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

    def _wait_for_matching_message(
        self,
        topic: str,
        message_type: Any,
        timeout: float,
        accepts: Callable[[Any], bool],
        timeout_error: Callable[[Any], str],
    ) -> Any:
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            self._raise_if_control_fault()
            try:
                last = self._rospy.wait_for_message(
                    topic,
                    message_type,
                    timeout=max(0.1, min(1.0, deadline - time.monotonic())),
                )
            except self._rospy.ROSException:
                continue
            if accepts(last):
                return last
        raise StartupError(timeout_error(last))

    def _wait_for_service_with_health(self, service_name: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._raise_if_control_fault()
            try:
                self._rospy.wait_for_service(
                    service_name,
                    timeout=max(0.1, min(0.5, deadline - time.monotonic())),
                )
                return
            except self._rospy.ROSException:
                continue
        raise StartupError("Timed out waiting for service %s" % service_name)

    def _wait_for_true(self, topic: str, message_type: Any, timeout: float) -> Any:
        return self._wait_for_matching_message(
            topic,
            message_type,
            timeout,
            lambda message: bool(message.data),
            lambda last: "Timed out waiting for true on %s; last=%s"
            % (topic, last),
        )

    def _wait_for_named_joint_state(
        self,
        topic: str,
        message_type: Any,
        required_joints: Iterable[str],
        timeout: float,
    ) -> Any:
        required = set(required_joints)
        description = "joints %s" % ", ".join(sorted(required))
        return self._wait_for_matching_message(
            topic,
            message_type,
            timeout,
            lambda message: required.issubset(message.name),
            lambda _last: "Timed out waiting for %s on %s" % (description, topic),
        )

    def _wait_for_named_joint_on_busy_topic(
        self,
        topic: str,
        message_type: Any,
        required_joints: Iterable[str],
        timeout: float,
    ) -> Any:
        required = set(required_joints)
        matched: List[Any] = []
        ready = threading.Event()

        def observe(message: Any) -> None:
            if required.issubset(message.name) and not ready.is_set():
                matched.append(message)
                ready.set()

        subscription = self._rospy.Subscriber(
            topic, message_type, observe, queue_size=100
        )
        try:
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self._raise_if_control_fault()
                if ready.wait(min(0.1, deadline - time.monotonic())):
                    return matched[0]
        finally:
            subscription.unregister()
        raise StartupError(
            "Timed out waiting for joints %s on %s"
            % (", ".join(sorted(required)), topic)
        )

    def _wait_for_initialized_gripper(
        self, topic: str, message_type: Any, timeout: float
    ) -> Any:
        return self._wait_for_matching_message(
            topic,
            message_type,
            timeout,
            lambda message: bool(message.is_initialized),
            lambda last: "AG95 did not report initialized=true on %s; last=%s"
            % (topic, last),
        )

    def _assert_fresh_advancing_joint_states(
        self, first: Any, second: Any, topic: str
    ) -> None:
        self._assert_fresh_advancing_headers(first, second, topic)

    def _assert_fresh_advancing_headers(
        self, first: Any, second: Any, topic: str
    ) -> None:
        if second.header.stamp <= first.header.stamp:
            raise StartupError("%s timestamps are not advancing" % topic)
        age = (self._rospy.Time.now() - second.header.stamp).to_sec()
        if age < 0.0:
            raise StartupError(
                "%s timestamp is in the future by %.3f seconds" % (topic, -age)
            )
        if age > 1.0:
            raise StartupError("%s is stale by %.3f seconds" % (topic, age))

    def _camera_startup_error(self, message: str) -> StartupError:
        for label, process in self.processes:
            if label == "d405_camera":
                code = process.poll()
                if code is not None:
                    return StartupError(
                        "d405_camera exited unexpectedly with code %d" % code
                    )
        return StartupError(message)

    def _wait_for_speed_range(
        self, topic: str, message_type: Any, requested: float, timeout: float
    ) -> Any:
        return self._wait_for_matching_message(
            topic,
            message_type,
            timeout,
            lambda message: 0.0 < message.data <= requested + 0.02,
            lambda last: (
                "Speed scaling did not reach the requested low-speed range; last=%s"
                % (None if last is None else last.data)
            ),
        )

    def wait_control_chain_ready(self, config: StartupConfig) -> None:
        self._init_ros(config.state_timeout)
        from std_msgs.msg import Float64

        self._health_evaluation_period = (
            config.runtime_policy.health_evaluation_period
        )
        self.control_chain_monitor = self._control_chain_monitor_factory(
            self._rospy,
            config.runtime_policy,
            required_joints=REQUIRED_JOINTS,
            target_controller=TARGET_CONTROLLER,
            diagnostic_output=self._emit_diagnostic,
        )
        self.control_chain_monitor.start()
        try:
            self.control_chain_monitor.wait_until_ready(config.state_timeout)
        except (ControlChainFault, ControlChainNotReady) as exc:
            raise StartupError(str(exc)) from exc
        scaling = self._wait_for_matching_message(
            SPEED_SCALING_TOPIC,
            Float64,
            config.state_timeout,
            lambda _message: True,
            lambda _last: "Timed out waiting for %s" % SPEED_SCALING_TOPIC,
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
        self._wait_for_named_joint_on_busy_topic(
            "/joint_states", JointState, {GRIPPER_JOINT}, 5.0
        )

    def wait_d405_ready(self, config: StartupConfig) -> None:
        if not config.enable_d405:
            return

        self._init_ros(config.state_timeout)
        from sensor_msgs.msg import CameraInfo, Image

        color_topic = "/d405/color/image_raw"
        depth_topic = "/d405/depth/image_rect_raw"
        info_topic = "/d405/color/camera_info"
        deadline = time.monotonic() + config.state_timeout

        def receive(topic: str, message_type: Any) -> Any:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise self._camera_startup_error(
                    "Timed out waiting for required D405 message on %s" % topic
                )
            try:
                return self._wait_for_matching_message(
                    topic,
                    message_type,
                    remaining,
                    lambda _message: True,
                    lambda _last: "Timed out waiting for required D405 message on %s"
                    % topic,
                )
            except StartupError as exc:
                if "CONTROL CHAIN FAULT" in str(exc):
                    raise
                raise self._camera_startup_error(str(exc)) from exc

        color_first = receive(color_topic, Image)
        color_second = receive(color_topic, Image)
        self._assert_fresh_advancing_headers(
            color_first, color_second, color_topic
        )
        color_dimensions = (color_second.width, color_second.height)
        if color_dimensions[0] <= 0 or color_dimensions[1] <= 0:
            raise StartupError(
                "%s dimensions must be positive; got %dx%d"
                % (color_topic, color_dimensions[0], color_dimensions[1])
            )

        depth_first = receive(depth_topic, Image)
        depth_second = receive(depth_topic, Image)
        self._assert_fresh_advancing_headers(
            depth_first, depth_second, depth_topic
        )

        color_info = receive(info_topic, CameraInfo)
        info_dimensions = (color_info.width, color_info.height)
        if info_dimensions != color_dimensions:
            raise StartupError(
                "%s camera_info dimensions %dx%d do not match %s dimensions %dx%d"
                % (
                    info_topic,
                    info_dimensions[0],
                    info_dimensions[1],
                    color_topic,
                    color_dimensions[0],
                    color_dimensions[1],
                )
            )

    def set_speed_slider(self, fraction: float) -> None:
        from std_msgs.msg import Float64
        from ur_msgs.srv import SetSpeedSliderFraction

        service_name = "/ur/ur_hardware_interface/set_speed_slider"
        self._wait_for_service_with_health(service_name, 10.0)
        response = self._rospy.ServiceProxy(service_name, SetSpeedSliderFraction)(fraction)
        if not response.success:
            raise StartupError("UR Driver rejected speed slider %.3f" % fraction)
        self._wait_for_speed_range(SPEED_SCALING_TOPIC, Float64, fraction, 5.0)

    def start_move_group(self, config: StartupConfig) -> None:
        self._raise_if_control_fault()
        self._launch(
            "move_group",
            ["roslaunch", "tracer_bringup", "ur3_moveit_execution.launch"],
        )

    def wait_move_group_ready(self, config: StartupConfig) -> None:
        from actionlib_msgs.msg import GoalStatusArray

        self._wait_for_service_with_health(
            "/get_planning_scene", config.state_timeout
        )
        self._wait_for_matching_message(
            "/move_group/status",
            GoalStatusArray,
            config.state_timeout,
            lambda _message: True,
            lambda _last: "Timed out waiting for /move_group/status",
        )

    def start_rviz(self, config: StartupConfig) -> None:
        rviz_config = os.path.join(
            self.package_paths["tracer_bringup"],
            "config",
            "ur3_headless_moveit.rviz",
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
            self._raise_if_control_fault()
            for label, process in self.processes:
                code = process.poll()
                if code is None:
                    continue
                if label == "rviz" and code == 0:
                    return
                raise StartupError("%s exited unexpectedly with code %d" % (label, code))
            time.sleep(self._health_evaluation_period)

    def _disable_moveit_execution(self) -> None:
        retained = []
        for label, process in self.processes:
            if label == "move_group":
                self._shutdown_process(process)
            else:
                retained.append((label, process))
        self.processes = retained

    def _raise_if_control_fault(self) -> None:
        if self.control_chain_monitor is None:
            return
        try:
            self.control_chain_monitor.raise_if_fault()
        except ControlChainFault as exc:
            self._disable_moveit_execution()
            message = (
                "CONTROL CHAIN FAULT: %s. New MoveIt Execute is disabled. "
                "Full restart required."
            ) % exc
            self._emit_diagnostic("❌ " + message)
            raise StartupError(message) from exc

    def _shutdown_controller_spawner(self) -> None:
        result = self._run(
            ["rosnode", "kill", CONTROLLER_SPAWNER_NODE],
            timeout=SHUTDOWN_SIGINT_TIMEOUT,
            required=False,
        )
        if result.returncode != 0:
            return

        deadline = time.monotonic() + SHUTDOWN_SIGINT_TIMEOUT
        while time.monotonic() < deadline:
            remaining = max(0.1, deadline - time.monotonic())
            nodes = self._run(
                ["rosnode", "list"],
                timeout=min(1.0, remaining),
                required=False,
            )
            if nodes.returncode != 0:
                return
            if CONTROLLER_SPAWNER_NODE not in nodes.stdout.splitlines():
                return
            time.sleep(0.1)

    def _shutdown_process(self, process: Any) -> None:
        self._signal_process_group(process, signal.SIGINT)
        running = self._wait_for_processes([process], SHUTDOWN_SIGINT_TIMEOUT)
        for process in running:
            self._signal_process_group(process, signal.SIGTERM)
        running = self._wait_for_processes(running, SHUTDOWN_SIGTERM_TIMEOUT)
        for process in running:
            self._signal_process_group(process, signal.SIGKILL)
        self._wait_for_processes(running, SHUTDOWN_SIGKILL_TIMEOUT)

    def shutdown(self) -> None:
        if self.control_chain_monitor is not None:
            self.control_chain_monitor.stop()
        ordered_labels = set(SHUTDOWN_PROCESS_ORDER)
        driver_running = any(
            label == "ur_driver" and process.poll() is None
            for label, process in self.processes
        )

        for label in SHUTDOWN_PROCESS_ORDER:
            if label == "ur_driver" and driver_running:
                self._shutdown_controller_spawner()
            for process_label, process in self.processes:
                if process_label == label:
                    self._shutdown_process(process)

        for label, process in self.processes:
            if label not in ordered_labels:
                self._shutdown_process(process)

        for diagnostic_thread in self._diagnostic_threads:
            diagnostic_thread.join(timeout=1.0)

    @staticmethod
    def _signal_process_group(process: Any, requested_signal: int) -> None:
        try:
            os.killpg(process.pid, requested_signal)
        except (OSError, ProcessLookupError):
            pass

    @staticmethod
    def _wait_for_processes(processes: Sequence[Any], timeout: float) -> List[Any]:
        deadline = time.monotonic() + timeout
        for process in processes:
            if process.poll() is not None:
                continue
            remaining = max(0.0, deadline - time.monotonic())
            try:
                process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                pass

        still_running = [
            process
            for process in processes
            if RosRuntime._process_group_is_running(process)
        ]
        while still_running and time.monotonic() < deadline:
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
            for process in still_running:
                process.poll()
            still_running = [
                process
                for process in still_running
                if RosRuntime._process_group_is_running(process)
            ]
        return still_running

    @staticmethod
    def _process_group_is_running(process: Any) -> bool:
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True
