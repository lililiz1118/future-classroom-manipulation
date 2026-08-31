#!/usr/bin/env python3
import os
import socket
import sys
import threading
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from tracer_bringup.headless_dashboard import (  # noqa: E402
    DashboardClient,
    SafetyGateError,
    assert_safe_mode,
    parse_robot_mode,
    parse_safety_mode,
)


class OneShotDashboardServer:
    def __init__(self, response):
        self.response = response
        self.command = None
        self._ready = threading.Event()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.bind(("127.0.0.1", 0))
        self.port = self._socket.getsockname()[1]
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def _serve(self):
        self._socket.listen(1)
        self._ready.set()
        connection, _ = self._socket.accept()
        with connection:
            connection.sendall(b"Connected: Universal Robots Dashboard Server\n")
            data = b""
            while not data.endswith(b"\n"):
                data += connection.recv(1024)
            self.command = data.decode("ascii").strip()
            connection.sendall((self.response + "\n").encode("ascii"))
        self._socket.close()

    def start(self):
        self._thread.start()
        self._ready.wait(1.0)

    def join(self):
        self._thread.join(1.0)


class DashboardParsingTest(unittest.TestCase):
    def test_parses_cb3_robot_and_safety_modes(self):
        self.assertEqual(parse_robot_mode("Robotmode: POWER_OFF"), "POWER_OFF")
        self.assertEqual(parse_safety_mode("Safetymode: NORMAL"), "NORMAL")

    def test_reduced_is_always_blocked(self):
        with self.assertRaises(SafetyGateError):
            assert_safe_mode("REDUCED")

    def test_every_dangerous_or_unknown_safety_mode_is_blocked(self):
        blocked = (
            "PROTECTIVE_STOP",
            "ROBOT_EMERGENCY_STOP",
            "SYSTEM_EMERGENCY_STOP",
            "FAULT",
            "VIOLATION",
            "RECOVERY",
            "SAFEGUARD_STOP",
            "UNKNOWN",
        )
        for mode in blocked:
            with self.subTest(mode=mode), self.assertRaises(SafetyGateError):
                assert_safe_mode(mode)

    def test_dashboard_client_performs_real_line_protocol_exchange(self):
        server = OneShotDashboardServer("Robotmode: IDLE")
        server.start()
        client = DashboardClient("127.0.0.1", port=server.port, timeout=1.0)

        response = client.query("robotmode")
        server.join()

        self.assertEqual(response, "Robotmode: IDLE")
        self.assertEqual(server.command, "robotmode")


if __name__ == "__main__":
    unittest.main()
