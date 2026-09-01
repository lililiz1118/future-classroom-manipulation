#!/usr/bin/env python3
from contextlib import redirect_stderr
import io
import os
import sys
import unittest
from unittest import mock


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from tracer_bringup.headless_cli import (  # noqa: E402
    build_argument_parser,
    explicit_confirmation,
    main,
)
from tracer_bringup.headless_startup import StartupAborted, StartupError  # noqa: E402


RUNTIME_POLICY_PATH = os.path.join(PACKAGE_ROOT, "config", "ur3_runtime.yaml")


class HeadlessCliTest(unittest.TestCase):
    def test_defaults_to_five_percent_and_normal_safety_only(self):
        arguments = build_argument_parser().parse_args([])
        self.assertEqual(arguments.speed_slider, 0.05)
        self.assertFalse(hasattr(arguments, "allow_reduced"))
        self.assertFalse(arguments.preflight_only)
        self.assertFalse(arguments.driver_only)

        with self.assertRaises(SystemExit):
            build_argument_parser().parse_args(["--allow-reduced"])

    def test_main_loads_the_selected_runtime_policy(self):
        observed = []

        def capture(coordinator):
            observed.append(coordinator.config.runtime_policy)

        with mock.patch(
            "tracer_bringup.headless_cli.StartupCoordinator.run",
            autospec=True,
            side_effect=capture,
        ):
            exit_code = main(
                [
                    "--calibration",
                    "/tmp/test-calibration.yaml",
                    "--runtime-config",
                    RUNTIME_POLICY_PATH,
                ]
            )

        self.assertEqual(exit_code, 0)
        self.assertEqual(observed[0].robot_receive_timeout, 0.10)
        self.assertEqual(observed[0].joint_state_timeout, 0.50)

    def test_operator_can_select_the_physical_gripper_device(self):
        parser = build_argument_parser()
        action = next(
            (
                candidate
                for candidate in parser._actions
                if candidate.dest == "gripper_device"
            ),
            None,
        )
        self.assertIsNotNone(
            action, "the CLI must accept an explicit physical AG95 device"
        )
        arguments = parser.parse_args(["--gripper-device", "/dev/test_ag95"])

        self.assertEqual(arguments.gripper_device, "/dev/test_ag95")

    def test_speed_above_ten_percent_is_rejected(self):
        with self.assertRaises(SystemExit):
            build_argument_parser().parse_args(["--speed-slider", "0.11"])

    def test_only_exact_start_token_confirms_hardware_change(self):
        outputs = []
        self.assertTrue(
            explicit_confirmation("warning", input_fn=lambda _: "START", output=outputs.append)
        )
        for answer in ("yes", "start", " START ", ""):
            with self.subTest(answer=answer):
                self.assertFalse(
                    explicit_confirmation(
                        "warning", input_fn=lambda _, value=answer: value, output=lambda _: None
                    )
                )

    def test_confirmation_warns_that_ag95_initialization_can_move(self):
        outputs = []

        explicit_confirmation(
            "将初始化 AG95（夹爪可能运动）。",
            input_fn=lambda _: "cancel",
            output=outputs.append,
        )

        self.assertTrue(
            any("AG95" in line and "可能运动" in line for line in outputs),
            "the single START gate must disclose physical gripper motion",
        )

    def test_confirmation_does_not_invent_unrequested_hardware(self):
        outputs = []

        explicit_confirmation(
            "仅驱动诊断，不启动夹爪、相机、MoveIt 或 RViz。",
            input_fn=lambda _: "cancel",
            output=outputs.append,
        )

        self.assertFalse(any("AG95" in line for line in outputs))

    def test_startup_failure_has_chinese_summary_and_original_diagnostics(self):
        errors = io.StringIO()

        with mock.patch(
            "tracer_bringup.headless_cli.StartupCoordinator.run",
            side_effect=StartupError("UR Driver did not become ready"),
        ), redirect_stderr(errors):
            exit_code = main(
                [
                    "--calibration",
                    "/tmp/test-calibration.yaml",
                    "--runtime-config",
                    RUNTIME_POLICY_PATH,
                ]
            )

        self.assertEqual(exit_code, 1)
        self.assertEqual(
            errors.getvalue().strip(),
            "❌ 启动失败｜原文: UR Driver did not become ready",
        )

    def test_operator_cancellation_is_reported_concisely(self):
        errors = io.StringIO()

        with mock.patch(
            "tracer_bringup.headless_cli.StartupCoordinator.run",
            side_effect=StartupAborted("Operator confirmation was not accepted"),
        ), redirect_stderr(errors):
            exit_code = main(
                [
                    "--calibration",
                    "/tmp/test-calibration.yaml",
                    "--runtime-config",
                    RUNTIME_POLICY_PATH,
                ]
            )

        self.assertEqual(exit_code, 2)
        self.assertEqual(
            errors.getvalue().strip(),
            "⏹️ 已取消｜原文: Operator confirmation was not accepted",
        )

    def test_keyboard_interrupt_reports_safe_shutdown(self):
        errors = io.StringIO()

        with mock.patch(
            "tracer_bringup.headless_cli.StartupCoordinator.run",
            side_effect=KeyboardInterrupt,
        ), redirect_stderr(errors):
            exit_code = main(
                [
                    "--calibration",
                    "/tmp/test-calibration.yaml",
                    "--runtime-config",
                    RUNTIME_POLICY_PATH,
                ]
            )

        self.assertEqual(exit_code, 130)
        self.assertEqual(
            errors.getvalue().strip(),
            "🛑 已收到 Ctrl+C，启动流程已尝试安全关闭 RViz、MoveIt、AG95 和 UR3 驱动。",
        )


    def test_d405_is_required_by_default_and_can_be_disabled(self):
        parser = build_argument_parser()
        self.assertTrue(parser.parse_args([]).enable_d405)
        self.assertFalse(parser.parse_args(["--no-d405"]).enable_d405)

    def test_driver_only_is_explicit_and_disables_d405_in_startup_config(self):
        observed = []

        def capture(coordinator):
            observed.append(coordinator.config)

        with mock.patch(
            "tracer_bringup.headless_cli.StartupCoordinator.run",
            autospec=True,
            side_effect=capture,
        ):
            exit_code = main(
                [
                    "--calibration",
                    "/tmp/test-calibration.yaml",
                    "--runtime-config",
                    RUNTIME_POLICY_PATH,
                    "--driver-only",
                ]
            )

        self.assertEqual(exit_code, 0)
        self.assertTrue(observed[0].driver_only)
        self.assertFalse(observed[0].enable_d405)

if __name__ == "__main__":
    unittest.main()
