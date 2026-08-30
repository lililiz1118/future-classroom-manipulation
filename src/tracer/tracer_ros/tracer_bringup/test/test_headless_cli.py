#!/usr/bin/env python3
import os
import sys
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from tracer_bringup.headless_cli import (  # noqa: E402
    build_argument_parser,
    explicit_confirmation,
)


class HeadlessCliTest(unittest.TestCase):
    def test_defaults_to_five_percent_and_normal_safety_only(self):
        arguments = build_argument_parser().parse_args([])
        self.assertEqual(arguments.speed_slider, 0.05)
        self.assertFalse(arguments.allow_reduced)
        self.assertFalse(arguments.preflight_only)

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
            "warning", input_fn=lambda _: "cancel", output=outputs.append
        )

        self.assertTrue(
            any("AG95" in line and "可能运动" in line for line in outputs),
            "the single START gate must disclose physical gripper motion",
        )


    def test_d405_is_required_by_default_and_can_be_disabled(self):
        parser = build_argument_parser()
        self.assertTrue(parser.parse_args([]).enable_d405)
        self.assertFalse(parser.parse_args(["--no-d405"]).enable_d405)

if __name__ == "__main__":
    unittest.main()
