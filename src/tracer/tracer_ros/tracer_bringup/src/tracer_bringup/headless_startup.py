#!/usr/bin/env python3
"""Safety decisions and ordered orchestration for headless UR3 MoveIt."""

from dataclasses import dataclass
import os
from typing import Any, Callable, Dict, Iterable, List, Sequence

import yaml

from .headless_dashboard import RobotStatus, assert_safe_mode


class StartupError(RuntimeError):
    """Preflight, readiness, or startup failure."""


class StartupAborted(StartupError):
    """Operator declined the explicit hardware confirmation."""


@dataclass(frozen=True)
class StartupConfig:
    robot_ip: str
    reverse_ip: str
    calibration_path: str
    expected_calibration_hash: str
    gripper_device: str = "/dev/dh_gripper_usb"
    enable_d405: bool = True
    speed_slider: float = 0.05
    allow_reduced: bool = False
    state_timeout: float = 30.0
    preflight_only: bool = False

    def __post_init__(self):
        if not 0.0 < self.speed_slider <= 0.10:
            raise ValueError("speed_slider must be in (0, 0.10]")


def validate_calibration(path: str, expected_hash: str) -> str:
    if not os.path.isfile(path):
        raise StartupError("Calibration YAML does not exist: %s" % path)
    try:
        with open(path, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise StartupError("Cannot read calibration YAML %s: %s" % (path, exc)) from exc
    kinematics = document.get("kinematics") if isinstance(document, dict) else None
    if not isinstance(kinematics, dict):
        raise StartupError("Calibration YAML has no kinematics map: %s" % path)
    actual_hash = kinematics.get("hash")
    if actual_hash != expected_hash:
        raise StartupError(
            "Calibration hash mismatch: expected %s, got %s"
            % (expected_hash, actual_hash)
        )
    required_links = ("shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3")
    required_fields = ("x", "y", "z", "roll", "pitch", "yaw")
    for link in required_links:
        values = kinematics.get(link)
        if not isinstance(values, dict) or any(field not in values for field in required_fields):
            raise StartupError("Calibration YAML is incomplete at kinematics.%s" % link)
    return actual_hash


def assert_exclusive_controller(
    snapshot: Sequence[Dict[str, Any]], target: str, arm_joints: Iterable[str]
) -> None:
    arm_joint_set = set(arm_joints)
    target_running = False
    conflicts: List[str] = []
    for controller in snapshot:
        if controller.get("state") != "running":
            continue
        name = controller.get("name", "<unnamed>")
        claimed = set()
        for claim in controller.get("claimed_resources", []):
            claimed.update(claim.get("resources", []))
        if name == target:
            target_running = True
            if not arm_joint_set.issubset(claimed):
                raise StartupError("Target controller does not claim all six UR joints")
        elif claimed & arm_joint_set:
            conflicts.append(name)
    if not target_running:
        raise StartupError("Target controller is not running: %s" % target)
    if conflicts:
        raise StartupError(
            "Other running controllers claim UR joints: %s" % ", ".join(sorted(conflicts))
        )


class StartupCoordinator:
    def __init__(
        self,
        dashboard: Any,
        runtime: Any,
        config: StartupConfig,
        confirm: Callable[[str], bool],
        output: Callable[[str], None] = print,
    ):
        self.dashboard = dashboard
        self.runtime = runtime
        self.config = config
        self.confirm = confirm
        self.output = output

    def run(self) -> None:
        self.runtime.preflight(self.config)
        self.runtime.assert_no_conflicts(self.config)
        status: RobotStatus = self.dashboard.preflight(self.config.allow_reduced)
        assert_safe_mode(status.safety_mode, self.config.allow_reduced)
        allowed_initial_modes = {"POWER_OFF", "BOOTING", "POWER_ON", "IDLE", "RUNNING"}
        if status.robot_mode not in allowed_initial_modes:
            raise StartupError("Robot mode is not startable: %s" % status.robot_mode)
        self.output(
            "UR3 %s | robot=%s | safety=%s | calibration=%s | AG95=%s | D405=%s | speed=%.0f%%"
            % (
                self.config.robot_ip,
                status.robot_mode,
                status.safety_mode,
                self.config.expected_calibration_hash,
                self.config.gripper_device,
                "required" if self.config.enable_d405 else "disabled",
                self.config.speed_slider * 100.0,
            )
        )
        if self.config.preflight_only:
            self.output("Preflight passed; no hardware state was changed.")
            return
        if not self.confirm(
            "确认工作区和夹爪周围无人、独立硬件急停可用，并允许 UR3 "
            "上电、松闸及 DH AG95 初始化"
        ):
            raise StartupAborted("Operator confirmation was not accepted")

        if status.robot_mode == "POWER_OFF":
            self.dashboard.power_on()
            status = self.dashboard.wait_robot_mode(
                {"POWER_ON", "IDLE", "RUNNING"},
                self.config.state_timeout,
                self.config.allow_reduced,
            )
        elif status.robot_mode == "BOOTING":
            status = self.dashboard.wait_robot_mode(
                {"POWER_ON", "IDLE", "RUNNING"},
                self.config.state_timeout,
                self.config.allow_reduced,
            )
        if status.robot_mode != "RUNNING":
            self.dashboard.brake_release()
            status = self.dashboard.wait_robot_mode(
                {"RUNNING"}, self.config.state_timeout, self.config.allow_reduced
            )

        try:
            self.runtime.start_driver(self.config)
            self.runtime.wait_driver_ready(self.config)
            self.runtime.start_gripper(self.config)
            self.runtime.wait_gripper_ready(self.config)
            if self.config.enable_d405:
                self.runtime.start_d405(self.config)
                self.runtime.wait_d405_ready(self.config)
                try:
                    self.runtime.start_anygrasp(self.config)
                    self.runtime.wait_anygrasp_ready(self.config)
                except StartupError as exc:
                    self.runtime.stop_anygrasp()
                    self.output(
                        "[WARNING] AnyGrasp unavailable: %s; "
                        "MoveIt/RViz remain available for manual target poses."
                        % exc
                    )
            self.runtime.set_speed_slider(self.config.speed_slider)
            self.runtime.start_move_group(self.config)
            self.runtime.wait_move_group_ready(self.config)
            self.runtime.start_rviz(self.config)
            self.runtime.supervise()
        finally:
            self.runtime.shutdown()
