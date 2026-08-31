#!/usr/bin/env python3
"""Small, strict client for the CB3 Dashboard line protocol."""

from dataclasses import dataclass
import re
import socket
import time
from typing import Iterable


class DashboardError(RuntimeError):
    """Dashboard connection or protocol failure."""


class SafetyGateError(DashboardError):
    """Robot safety state is not explicitly allowed."""


@dataclass(frozen=True)
class RobotStatus:
    robot_mode: str
    safety_mode: str


def _parse_mode(response: str, label: str) -> str:
    match = re.fullmatch(r"\s*%s\s*:\s*([A-Za-z_]+)\s*" % label, response)
    if not match:
        raise DashboardError("Unexpected Dashboard response: %r" % response)
    return match.group(1).upper()


def parse_robot_mode(response: str) -> str:
    return _parse_mode(response, "Robotmode")


def parse_safety_mode(response: str) -> str:
    return _parse_mode(response, "Safetymode")


def assert_safe_mode(mode: str) -> None:
    normalized = mode.strip().upper()
    if normalized != "NORMAL":
        raise SafetyGateError(
            "Safety mode %s is blocked; no automatic recovery will be attempted"
            % normalized
        )


class DashboardClient:
    def __init__(self, host: str, port: int = 29999, timeout: float = 2.0):
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)

    def _exchange(self, command: str) -> str:
        if not command or "\n" in command or "\r" in command:
            raise DashboardError("Dashboard command must be one non-empty line")
        try:
            with socket.create_connection(
                (self.host, self.port), timeout=self.timeout
            ) as connection:
                connection.settimeout(self.timeout)
                stream = connection.makefile("rwb", buffering=0)
                greeting = stream.readline(4096)
                if not greeting:
                    raise DashboardError("Dashboard closed before its greeting")
                stream.write((command + "\n").encode("ascii"))
                response = stream.readline(4096)
        except (OSError, UnicodeError) as exc:
            raise DashboardError(
                "Dashboard %s:%d command %r failed: %s"
                % (self.host, self.port, command, exc)
            ) from exc
        if not response:
            raise DashboardError("Dashboard returned an empty response to %r" % command)
        return response.decode("utf-8", errors="replace").strip()

    def query(self, command: str) -> str:
        return self._exchange(command)

    def command(self, command: str) -> str:
        response = self._exchange(command)
        lowered = response.lower()
        if "fail" in lowered or "not allowed" in lowered:
            raise DashboardError("Dashboard rejected %r: %s" % (command, response))
        return response

    def status(self) -> RobotStatus:
        robot_mode = parse_robot_mode(self.query("robotmode"))
        safety_mode = parse_safety_mode(self.query("safetymode"))
        assert_safe_mode(safety_mode)
        return RobotStatus(robot_mode, safety_mode)

    def preflight(self) -> RobotStatus:
        return self.status()

    def power_on(self) -> str:
        return self.command("power on")

    def brake_release(self) -> str:
        return self.command("brake release")

    def wait_robot_mode(
        self,
        modes: Iterable[str],
        timeout: float,
        poll_interval: float = 0.5,
    ) -> RobotStatus:
        expected = {mode.upper() for mode in modes}
        deadline = time.monotonic() + timeout
        last_status = None
        while time.monotonic() < deadline:
            last_status = self.status()
            if last_status.robot_mode in expected:
                return last_status
            time.sleep(poll_interval)
        raise DashboardError(
            "Timed out waiting for robot mode %s; last status was %s"
            % (sorted(expected), last_status)
        )
