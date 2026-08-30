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


if __name__ == "__main__":
    unittest.main()
