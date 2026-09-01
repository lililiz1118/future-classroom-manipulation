#!/usr/bin/env python3
"""Command-line interface for guarded UR3 headless MoveIt startup."""

import argparse
import os
import subprocess
import sys
from typing import Callable, Optional, Sequence

from .headless_dashboard import DashboardClient, DashboardError
from .headless_runtime import RosRuntime
from .headless_startup import StartupAborted, StartupConfig, StartupCoordinator, StartupError
from .runtime_config import RuntimePolicyError, load_ur_runtime_policy


EXPECTED_CALIBRATION_HASH = "calib_13945068365021364089"


def _low_speed_fraction(value: str) -> float:
    try:
        fraction = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("speed slider must be a number") from exc
    if not 0.0 < fraction <= 0.10:
        raise argparse.ArgumentTypeError("speed slider must be in (0, 0.10]")
    return fraction


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Safely start UR3 CB3 headless Driver, MoveIt and RViz"
    )
    parser.add_argument("--robot-ip", default="192.168.131.3")
    parser.add_argument("--reverse-ip", default="192.168.131.1")
    parser.add_argument("--dashboard-port", type=int, default=29999)
    parser.add_argument("--calibration", default=None)
    parser.add_argument("--gripper-device", default="/dev/dh_gripper_usb")
    parser.add_argument(
        "--no-d405",
        action="store_false",
        dest="enable_d405",
        help="run UR3, AG95, MoveIt and RViz without requiring or starting D405",
    )
    parser.add_argument(
        "--driver-only",
        action="store_true",
        help="diagnose only the guarded UR driver control chain; skip AG95, D405, MoveIt and RViz",
    )
    parser.add_argument("--speed-slider", type=_low_speed_fraction, default=0.05)
    parser.add_argument(
        "--runtime-config",
        default=None,
        help="validated UR3 runtime policy YAML",
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="run read-only checks and exit before confirmation or hardware changes",
    )
    parser.add_argument("--state-timeout", type=float, default=30.0)
    return parser


def explicit_confirmation(
    prompt: str,
    input_fn: Callable[[str], str] = input,
    output: Callable[[str], None] = print,
) -> bool:
    output("")
    output("================ UR3 实机安全确认 ================")
    output("当前系统即将执行下述硬件启动操作。")
    output(prompt)
    output("本程序不会自动解除保护停机、急停、Fault 或 Violation。")
    output("没有示教器时，SSH/Ctrl+C 不能替代独立硬件急停。")
    return input_fn("仅输入大写 START 继续，其他输入均取消: ") == "START"


def _default_calibration_path() -> str:
    package_path = subprocess.check_output(
        ["rospack", "find", "tracer_bringup"], text=True
    ).strip()
    return os.path.join(package_path, "config", "ur3_calibration.yaml")


def _default_runtime_config_path() -> str:
    package_path = subprocess.check_output(
        ["rospack", "find", "tracer_bringup"], text=True
    ).strip()
    return os.path.join(package_path, "config", "ur3_runtime.yaml")


def main(argv: Optional[Sequence[str]] = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    try:
        calibration_path = arguments.calibration or _default_calibration_path()
        runtime_policy = load_ur_runtime_policy(
            arguments.runtime_config or _default_runtime_config_path()
        )
        config = StartupConfig(
            robot_ip=arguments.robot_ip,
            reverse_ip=arguments.reverse_ip,
            calibration_path=calibration_path,
            expected_calibration_hash=EXPECTED_CALIBRATION_HASH,
            runtime_policy=runtime_policy,
            gripper_device=arguments.gripper_device,
            enable_d405=arguments.enable_d405 and not arguments.driver_only,
            driver_only=arguments.driver_only,
            speed_slider=arguments.speed_slider,
            state_timeout=arguments.state_timeout,
            preflight_only=arguments.preflight_only,
        )
        dashboard = DashboardClient(
            arguments.robot_ip, port=arguments.dashboard_port, timeout=2.0
        )
        runtime = RosRuntime()
        coordinator = StartupCoordinator(
            dashboard,
            runtime,
            config,
            confirm=explicit_confirmation,
            output=print,
        )
        coordinator.run()
        return 0
    except StartupAborted as exc:
        print("⏹️ 已取消｜原文: %s" % exc, file=sys.stderr)
        return 2
    except (
        DashboardError,
        RuntimePolicyError,
        StartupError,
        OSError,
        subprocess.SubprocessError,
    ) as exc:
        print("❌ 启动失败｜原文: %s" % exc, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print(
            "\n🛑 已收到 Ctrl+C，启动流程已尝试安全关闭 "
            "RViz、MoveIt、AG95 和 UR3 驱动。",
            file=sys.stderr,
        )
        return 130
