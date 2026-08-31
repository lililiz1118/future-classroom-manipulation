#!/usr/bin/env python3
import dataclasses
import os
import sys
import tempfile
import unittest

import yaml


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from tracer_bringup.runtime_config import (  # noqa: E402
    RuntimePolicyError,
    load_ur_runtime_policy,
)


class UrRuntimePolicyTest(unittest.TestCase):
    def _write_policy(self, document):
        handle = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
        yaml.safe_dump(document, handle)
        handle.close()
        self.addCleanup(lambda: os.unlink(handle.name))
        return handle.name

    def test_checked_in_policy_controls_receive_and_health_timing(self):
        policy = load_ur_runtime_policy(
            os.path.join(PACKAGE_ROOT, "config", "ur3_runtime.yaml")
        )

        self.assertEqual(policy.robot_receive_timeout, 0.10)
        self.assertEqual(policy.health_evaluation_period, 0.10)
        self.assertEqual(policy.controller_poll_period, 0.25)
        self.assertEqual(policy.joint_state_timeout, 0.50)
        self.assertEqual(policy.ready_joint_samples, 2)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            policy.robot_receive_timeout = 0.02

    def test_rejects_missing_unknown_and_invalid_policy_values(self):
        valid = {
            "robot_receive_timeout": 0.10,
            "health": {
                "evaluation_period": 0.10,
                "controller_poll_period": 0.25,
                "joint_state_timeout": 0.50,
                "ready_joint_samples": 2,
            },
        }
        cases = [
            {},
            dict(valid, unexpected=True),
            dict(valid, robot_receive_timeout=0.0),
            dict(valid, robot_receive_timeout=0.11),
            dict(valid, health=dict(valid["health"], ready_joint_samples=1)),
        ]

        for document in cases:
            with self.subTest(document=document):
                with self.assertRaises(RuntimePolicyError):
                    load_ur_runtime_policy(self._write_policy(document))


if __name__ == "__main__":
    unittest.main()
