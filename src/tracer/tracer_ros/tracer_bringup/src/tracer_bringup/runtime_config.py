"""Validated runtime policy for the guarded UR3 control chain."""

from dataclasses import dataclass
import math
from typing import Any, Dict

import yaml


class RuntimePolicyError(ValueError):
    """The checked-in UR runtime policy is incomplete or unsafe."""


@dataclass(frozen=True)
class UrRuntimePolicy:
    robot_receive_timeout: float
    health_evaluation_period: float
    controller_poll_period: float
    joint_state_timeout: float
    ready_joint_samples: int


def _require_exact_keys(mapping: Any, required, context: str) -> Dict[str, Any]:
    if not isinstance(mapping, dict):
        raise RuntimePolicyError("%s must be a map" % context)
    actual = set(mapping)
    expected = set(required)
    if actual != expected:
        raise RuntimePolicyError(
            "%s keys must be exactly %s; got %s"
            % (context, sorted(expected), sorted(actual))
        )
    return mapping


def _positive_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimePolicyError("%s must be a number" % name)
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise RuntimePolicyError("%s must be positive and finite" % name)
    return result


def load_ur_runtime_policy(path: str) -> UrRuntimePolicy:
    try:
        with open(path, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimePolicyError("Cannot read UR runtime policy %s: %s" % (path, exc)) from exc

    root = _require_exact_keys(
        document, {"robot_receive_timeout", "health"}, "UR runtime policy"
    )
    health = _require_exact_keys(
        root["health"],
        {
            "evaluation_period",
            "controller_poll_period",
            "joint_state_timeout",
            "ready_joint_samples",
        },
        "UR runtime policy health",
    )
    robot_receive_timeout = _positive_number(
        root["robot_receive_timeout"], "robot_receive_timeout"
    )
    timeout_ticks = robot_receive_timeout / 0.02
    if not math.isclose(timeout_ticks, round(timeout_ticks), abs_tol=1e-9):
        raise RuntimePolicyError("robot_receive_timeout must be a multiple of 0.02")
    ready_joint_samples = health["ready_joint_samples"]
    if isinstance(ready_joint_samples, bool) or not isinstance(ready_joint_samples, int):
        raise RuntimePolicyError("ready_joint_samples must be an integer")
    if ready_joint_samples < 2:
        raise RuntimePolicyError("ready_joint_samples must be at least 2")

    return UrRuntimePolicy(
        robot_receive_timeout=robot_receive_timeout,
        health_evaluation_period=_positive_number(
            health["evaluation_period"], "health.evaluation_period"
        ),
        controller_poll_period=_positive_number(
            health["controller_poll_period"], "health.controller_poll_period"
        ),
        joint_state_timeout=_positive_number(
            health["joint_state_timeout"], "health.joint_state_timeout"
        ),
        ready_joint_samples=ready_joint_samples,
    )
