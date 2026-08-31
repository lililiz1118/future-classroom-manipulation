#!/usr/bin/env python3
import os
import sys
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from tracer_bringup.control_chain_health import (  # noqa: E402
    ControlChainHealth,
    ControlChainState,
)
from tracer_bringup.runtime_config import load_ur_runtime_policy  # noqa: E402


POLICY = load_ur_runtime_policy(
    os.path.join(PACKAGE_ROOT, "config", "ur3_runtime.yaml")
)


def starting_health():
    return ControlChainHealth(POLICY)


def ready_health():
    health = starting_health()
    health.observe_robot_mode("RUNNING")
    health.observe_safety_mode("NORMAL")
    health.observe_program_running(True)
    health.observe_controller(True)
    health.observe_joint_state(9.90, 100.0, 0.01, True)
    health.observe_joint_state(9.95, 100.02, 0.01, True)
    if health.evaluate(10.0) is not ControlChainState.READY:
        raise AssertionError(health.readiness_blockers(10.0))
    return health


class ControlChainHealthTest(unittest.TestCase):
    def test_ready_requires_every_control_chain_observation(self):
        observations = {
            "robot": lambda health: health.observe_robot_mode("RUNNING"),
            "safety": lambda health: health.observe_safety_mode("NORMAL"),
            "program": lambda health: health.observe_program_running(True),
            "controller": lambda health: health.observe_controller(True),
            "joint_one": lambda health: health.observe_joint_state(
                9.90, 100.0, 0.01, True
            ),
            "joint_two": lambda health: health.observe_joint_state(
                9.95, 100.02, 0.01, True
            ),
        }

        for omitted in observations:
            with self.subTest(omitted=omitted):
                health = starting_health()
                for name, observe in observations.items():
                    if name != omitted:
                        observe(health)
                self.assertIs(health.evaluate(10.0), ControlChainState.STARTING)
                self.assertTrue(health.readiness_blockers(10.0))

    def test_two_complete_advancing_fresh_samples_enter_ready(self):
        health = ready_health()

        self.assertIs(health.state, ControlChainState.READY)
        self.assertEqual(health.snapshot.advancing_joint_samples, 2)
        self.assertIsNone(health.fault_reason)

    def test_program_transition_from_true_to_false_faults_immediately(self):
        health = ready_health()

        health.observe_program_running(False)

        self.assertIs(health.state, ControlChainState.FAULT)
        self.assertEqual(health.fault_reason, "robot_program_running=False")

    def test_robot_safety_and_controller_violations_fault_immediately(self):
        cases = (
            (
                lambda health: health.observe_robot_mode("IDLE"),
                "robot mode=IDLE (expected RUNNING)",
            ),
            (
                lambda health: health.observe_safety_mode("REDUCED"),
                "safety mode=REDUCED (expected NORMAL)",
            ),
            (
                lambda health: health.observe_controller(False, "controller stopped"),
                "trajectory controller unhealthy: controller stopped",
            ),
        )

        for violate, expected_reason in cases:
            with self.subTest(expected_reason=expected_reason):
                health = ready_health()
                violate(health)
                self.assertIs(health.state, ControlChainState.FAULT)
                self.assertEqual(health.fault_reason, expected_reason)

    def test_joint_receive_timeout_faults_ready_state(self):
        health = ready_health()

        health.evaluate(10.46)

        self.assertIs(health.state, ControlChainState.FAULT)
        self.assertEqual(
            health.fault_reason,
            "joint_states stale: 0.510s > 0.500s",
        )

    def test_invalid_joint_samples_fault_ready_state(self):
        cases = (
            ((10.01, 100.04, 0.01, False), "joint_states missing required UR joints"),
            ((10.01, 100.02, 0.01, True), "joint_states timestamp did not advance"),
            ((10.01, 100.04, -0.01, True), "joint_states timestamp is in the future"),
            ((10.01, 100.04, 0.51, True), "joint_states header is stale by 0.510s"),
        )

        for arguments, expected_reason in cases:
            with self.subTest(expected_reason=expected_reason):
                health = ready_health()
                health.observe_joint_state(*arguments)
                self.assertIs(health.state, ControlChainState.FAULT)
                self.assertEqual(health.fault_reason, expected_reason)

    def test_fault_is_latched_and_preserves_first_reason(self):
        health = ready_health()
        health.observe_program_running(False)

        health.observe_robot_mode("RUNNING")
        health.observe_safety_mode("NORMAL")
        health.observe_program_running(True)
        health.observe_controller(True)
        health.observe_joint_state(10.01, 100.04, 0.01, True)
        health.evaluate(10.02)

        self.assertIs(health.state, ControlChainState.FAULT)
        self.assertEqual(health.fault_reason, "robot_program_running=False")


if __name__ == "__main__":
    unittest.main()
