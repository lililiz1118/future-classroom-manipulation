#!/usr/bin/env python3
import fcntl
import importlib.util
import os
import signal
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

try:
    from tracer_bringup import freedrive_camera
except ImportError:
    freedrive_camera = None


FREEDRIVE_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "ur3_freedrive.py"


def load_freedrive_script():
    spec = importlib.util.spec_from_file_location("ur3_freedrive_tested", FREEDRIVE_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.socket = SimpleNamespace(
        create_connection=module.socket.create_connection,
    )
    module.signal = SimpleNamespace(
        signal=module.signal.signal,
        SIGINT=module.signal.SIGINT,
        SIGTERM=module.signal.SIGTERM,
        SIGHUP=module.signal.SIGHUP,
    )
    module.time = SimpleNamespace(
        sleep=module.time.sleep,
        time=module.time.time,
    )
    return module


class D405NodeClassificationTest(unittest.TestCase):
    def test_distinguishes_absent_external_and_partial_camera_nodes(self):
        self.assertIsNotNone(
            freedrive_camera,
            "the freedrive camera integration module must exist",
        )
        classify = freedrive_camera.classify_d405_nodes
        self.assertEqual(classify([]), "absent")
        self.assertEqual(
            classify(
                [
                    "/d405/realsense2_camera",
                    "/d405/realsense2_camera_manager",
                ]
            ),
            "external",
        )
        with self.assertRaisesRegex(
            freedrive_camera.CameraViewError, "Incomplete D405 node set"
        ):
            classify(["/d405/realsense2_camera"])


class DesktopEnvironmentTest(unittest.TestCase):
    def test_defaults_ssh_launches_to_the_active_local_x11_session(self):
        self.assertTrue(
            hasattr(freedrive_camera, "build_desktop_environment"),
            "freedrive must provide the robot-local display environment",
        )
        xauthority = "/run/user/1000/gdm/Xauthority"
        environment = freedrive_camera.build_desktop_environment(
            {"PATH": "/usr/bin"},
            uid=1000,
            path_exists=lambda path: path == xauthority,
        )
        self.assertEqual(environment["DISPLAY"], ":0")
        self.assertEqual(environment["XAUTHORITY"], xauthority)
        self.assertEqual(environment["XDG_RUNTIME_DIR"], "/run/user/1000")
        self.assertEqual(environment["PATH"], "/usr/bin")

    def test_overrides_ssh_x11_forwarding_with_the_robot_local_display(self):
        xauthority = "/run/user/1000/gdm/Xauthority"
        environment = freedrive_camera.build_desktop_environment(
            {
                "DISPLAY": "localhost:10.0",
                "XAUTHORITY": "/tmp/ssh-xauthority",
            },
            uid=1000,
            path_exists=lambda path: path == xauthority,
        )
        self.assertEqual(environment["DISPLAY"], ":0")
        self.assertEqual(environment["XAUTHORITY"], xauthority)


class FrameFreshnessWatchdogTest(unittest.TestCase):
    def test_rejects_stale_stream_and_refreshes_on_each_color_frame(self):
        self.assertTrue(
            hasattr(freedrive_camera, "FrameFreshnessWatchdog"),
            "freedrive must detect a frozen D405 stream",
        )
        clock = [10.0]
        callbacks = []

        class Subscription:
            def unregister(self):
                return None

        watchdog = freedrive_camera.FrameFreshnessWatchdog(
            "/d405/color/image_raw",
            stale_after=2.0,
            monotonic=lambda: clock[0],
            subscriber_factory=lambda topic, callback: (
                callbacks.append((topic, callback)) or Subscription()
            ),
        )
        clock[0] = 11.9
        watchdog.assert_fresh()
        callbacks[0][1](object())
        clock[0] = 13.8
        watchdog.assert_fresh()
        clock[0] = 14.1
        with self.assertRaisesRegex(
            freedrive_camera.CameraViewError, "D405 color stream is stale"
        ):
            watchdog.assert_fresh()


class HealthyWatchdog:
    def assert_fresh(self):
        return None

    def shutdown(self):
        return None


class CameraViewSessionTest(unittest.TestCase):
    def test_start_cleans_owned_camera_when_setup_raises_unexpected_exception(self):
        signals = []

        def run(command, **kwargs):
            if command == ["rosnode", "list"]:
                return SimpleNamespace(
                    returncode=1,
                    stdout="",
                    stderr="ERROR: Unable to communicate with master!",
                )
            raise OSError("rostopic failed unexpectedly")

        class Process:
            pid = 1900

            def poll(self):
                return None

            def wait(self, timeout):
                return 0

        session = freedrive_camera.CameraViewSession(
            run=run,
            popen=lambda *args, **kwargs: Process(),
            sleep=lambda _: None,
            getpgid=lambda pid: pid,
            killpg=lambda process_group, sent_signal: signals.append(sent_signal),
        )

        with self.assertRaisesRegex(OSError, "rostopic failed unexpectedly"):
            session.start()

        self.assertEqual(signals, [signal.SIGINT])
        self.assertEqual(session.processes, [])

    def test_starts_missing_d405_waits_for_color_and_opens_local_viewer(self):
        self.assertTrue(
            hasattr(freedrive_camera, "CameraViewSession"),
            "freedrive must supervise the camera and local viewer",
        )
        run_commands = []
        launched = []
        rostopic_attempts = 0

        def run(command, **kwargs):
            nonlocal rostopic_attempts
            run_commands.append((list(command), kwargs))
            if command == ["rosnode", "list"]:
                return SimpleNamespace(
                    returncode=1,
                    stdout="",
                    stderr="ERROR: Unable to communicate with master!",
                )
            if command[0] == "rostopic":
                rostopic_attempts += 1
                if rostopic_attempts < 3:
                    return SimpleNamespace(
                        returncode=1,
                        stdout="",
                        stderr="ERROR: Unable to communicate with master!",
                    )
            return SimpleNamespace(returncode=0, stdout="header: ready", stderr="")

        class Process:
            def __init__(self, pid):
                self.pid = pid

            def poll(self):
                return None

        def popen(command, **kwargs):
            launched.append((list(command), kwargs))
            return Process(1000 + len(launched))

        xauthority = "/run/user/1000/gdm/Xauthority"
        session = freedrive_camera.CameraViewSession(
            run=run,
            popen=popen,
            sleep=lambda _: None,
            environment={"PATH": "/usr/bin"},
            uid=1000,
            path_exists=lambda path: path == xauthority,
            watchdog_factory=lambda topic: HealthyWatchdog(),
        )
        try:
            session.start()
        except freedrive_camera.CameraViewError as exc:
            self.fail("camera readiness must retry startup races: %s" % exc)

        self.assertEqual(
            [entry[0] for entry in launched],
            [
                ["roslaunch", "tracer_bringup", "ur3_d405_camera.launch"],
                [
                    "rosrun",
                    "image_view",
                    "image_view",
                    "image:=/d405/color/image_raw",
                    "_autosize:=true",
                    "__name:=ur3_freedrive_camera_view",
                ],
            ],
        )
        self.assertEqual(
            run_commands[1][0],
            [
                "rostopic",
                "echo",
                "-n",
                "1",
                "/d405/color/image_raw/header",
            ],
        )
        self.assertEqual(launched[1][1]["env"]["DISPLAY"], ":0")
        self.assertEqual(launched[1][1]["env"]["XAUTHORITY"], xauthority)
        self.assertEqual(rostopic_attempts, 3)

    def test_reuses_external_d405_and_stops_only_the_owned_viewer(self):
        self.assertTrue(
            hasattr(freedrive_camera.CameraViewSession, "shutdown"),
            "freedrive must clean up the image window and an owned camera",
        )
        launched = []
        signals = []

        def run(command, **kwargs):
            if command == ["rosnode", "list"]:
                return SimpleNamespace(
                    returncode=0,
                    stdout=(
                        "/d405/realsense2_camera\n"
                        "/d405/realsense2_camera_manager\n"
                    ),
                    stderr="",
                )
            return SimpleNamespace(returncode=0, stdout="header: ready", stderr="")

        class Process:
            pid = 2400

            def poll(self):
                return None

            def wait(self, timeout):
                return 0

        def popen(command, **kwargs):
            launched.append(list(command))
            return Process()

        session = freedrive_camera.CameraViewSession(
            run=run,
            popen=popen,
            sleep=lambda _: None,
            environment={"PATH": "/usr/bin"},
            uid=1000,
            path_exists=lambda path: path == "/run/user/1000/gdm/Xauthority",
            getpgid=lambda pid: pid + 10,
            killpg=lambda process_group, sent_signal: signals.append(
                (process_group, sent_signal)
            ),
            watchdog_factory=lambda topic: HealthyWatchdog(),
        )
        session.start()
        session.shutdown()

        self.assertEqual(
            launched,
            [
                [
                    "rosrun",
                    "image_view",
                    "image_view",
                    "image:=/d405/color/image_raw",
                    "_autosize:=true",
                    "__name:=ur3_freedrive_camera_view",
                ]
            ],
        )
        self.assertEqual(signals, [(2410, signal.SIGINT)])

    def test_shutdown_waits_after_sigterm_before_forgetting_child(self):
        signals = []
        waits = []

        class Process:
            pid = 3100

            def poll(self):
                return None

            def wait(self, timeout):
                waits.append(timeout)
                if len(waits) == 1:
                    raise subprocess.TimeoutExpired(cmd="viewer", timeout=timeout)
                return 0

        session = freedrive_camera.CameraViewSession(
            getpgid=lambda pid: pid,
            killpg=lambda process_group, sent_signal: signals.append(sent_signal),
        )
        session.processes = [("d405_viewer", Process())]
        session.shutdown()

        self.assertEqual(signals, [signal.SIGINT, signal.SIGTERM])
        self.assertEqual(waits, [3.0, 3.0])
        self.assertEqual(session.processes, [])

    def test_session_health_fails_when_color_watchdog_reports_stale_frames(self):
        self.assertTrue(
            hasattr(freedrive_camera.CameraViewSession, "_start_watchdog"),
            "camera session must attach a color-frame freshness watchdog",
        )

        class StaleWatchdog:
            def assert_fresh(self):
                raise freedrive_camera.CameraViewError(
                    "D405 color stream is stale"
                )

            def shutdown(self):
                return None

        session = freedrive_camera.CameraViewSession(
            watchdog_factory=lambda topic: StaleWatchdog()
        )
        session._start_watchdog()
        with self.assertRaisesRegex(
            freedrive_camera.CameraViewError, "D405 color stream is stale"
        ):
            session.assert_alive()


class FreedriveMainIntegrationTest(unittest.TestCase):
    def test_camera_is_ready_before_freedrive_and_arm_locks_before_view_closes(self):
        freedrive = load_freedrive_script()
        self.assertTrue(
            hasattr(freedrive, "CameraViewSession"),
            "the freedrive entrypoint must integrate the camera view",
        )
        events = []
        installed_signals = []

        class Camera:
            def start(self):
                events.append("camera.start")

            def assert_alive(self):
                events.append("camera.check")

            def shutdown(self):
                events.append("camera.shutdown")

        class Input:
            def __enter__(self):
                return self

            def __exit__(self, *_):
                return None

            def get_key(self):
                return "q"

        freedrive.CameraViewSession = Camera
        freedrive.NonBlockingInput = Input
        freedrive.ensure_safety_normal = lambda host: True
        freedrive.ensure_power_and_brakes = lambda host: True
        freedrive.start_freedrive = lambda *args: events.append("freedrive.start")
        freedrive.stop_freedrive = lambda *args: events.append("freedrive.stop")
        freedrive.read_joint_angles = lambda *args: [0.0] * 6
        def install_signal(sent_signal, handler):
            installed_signals.append(sent_signal)
            events.append("signal.install")

        freedrive.signal.signal = install_signal
        freedrive.time.sleep = lambda *_: None

        with self.assertRaises(SystemExit) as exit_context:
            freedrive.main()

        self.assertEqual(exit_context.exception.code, 0)
        self.assertLess(events.index("camera.start"), events.index("freedrive.start"))
        self.assertLess(events.index("signal.install"), events.index("freedrive.start"))
        self.assertLess(events.index("freedrive.stop"), events.index("camera.shutdown"))
        self.assertIn(signal.SIGHUP, installed_signals)

    def test_unexpected_exception_still_locks_arm_before_closing_camera(self):
        freedrive = load_freedrive_script()
        events = []

        class Camera:
            def start(self):
                events.append("camera.start")

            def assert_alive(self):
                events.append("camera.check")

            def shutdown(self):
                events.append("camera.shutdown")

        class BrokenInput:
            def __enter__(self):
                return self

            def __exit__(self, *_):
                return None

            def get_key(self):
                raise RuntimeError("terminal disconnected")

        freedrive.CameraViewSession = Camera
        freedrive.NonBlockingInput = BrokenInput
        freedrive.ensure_safety_normal = lambda host: True
        freedrive.ensure_power_and_brakes = lambda host: True
        freedrive.start_freedrive = lambda *args: events.append("freedrive.start")
        freedrive.stop_freedrive = lambda *args: events.append("freedrive.stop")
        freedrive.signal.signal = lambda *args: None
        freedrive.time.sleep = lambda *_: None

        with self.assertRaisesRegex(RuntimeError, "terminal disconnected"):
            freedrive.main()

        self.assertIn("freedrive.stop", events)
        self.assertIn("camera.shutdown", events)
        self.assertLess(events.index("freedrive.stop"), events.index("camera.shutdown"))

    def test_exception_immediately_after_activation_still_locks_arm(self):
        freedrive = load_freedrive_script()
        events = []

        class Camera:
            def start(self):
                return None

            def assert_alive(self):
                return None

            def shutdown(self):
                events.append("camera.shutdown")

        freedrive.CameraViewSession = Camera
        freedrive.ensure_safety_normal = lambda host: True
        freedrive.ensure_power_and_brakes = lambda host: True
        freedrive.start_freedrive = lambda *args: events.append("freedrive.start")
        freedrive.stop_freedrive = lambda *args: events.append("freedrive.stop")
        freedrive.signal.signal = lambda *args: None

        def fail_after_activation(duration):
            if duration == 0.3:
                raise RuntimeError("post-activation setup failed")

        freedrive.time.sleep = fail_after_activation

        with self.assertRaisesRegex(RuntimeError, "post-activation setup failed"):
            freedrive.main()

        self.assertEqual(
            events,
            ["freedrive.start", "freedrive.stop", "camera.shutdown"],
        )

    def test_failed_arm_stop_keeps_camera_open_and_reports_failure(self):
        freedrive = load_freedrive_script()
        events = []

        class Camera:
            def start(self):
                events.append("camera.start")

            def assert_alive(self):
                return None

            def shutdown(self):
                events.append("camera.shutdown")

        class Input:
            calls = 0

            def __enter__(self):
                return self

            def __exit__(self, *_):
                return None

            def get_key(self):
                self.calls += 1
                if self.calls == 1:
                    return "q"
                raise RuntimeError("end test after retry remains available")

        def fail_stop(*args):
            events.append("freedrive.stop.failed")
            raise freedrive.FreedriveStopError("stop not confirmed")

        freedrive.CameraViewSession = Camera
        freedrive.NonBlockingInput = Input
        freedrive.ensure_safety_normal = lambda host: True
        freedrive.ensure_power_and_brakes = lambda host: True
        freedrive.start_freedrive = lambda *args: None
        freedrive.stop_freedrive = fail_stop
        freedrive.signal.signal = lambda *args: None
        freedrive.time.sleep = lambda *_: None

        with self.assertRaisesRegex(
            freedrive.FreedriveStopError, "stop not confirmed"
        ):
            freedrive.main()

        self.assertGreaterEqual(events.count("freedrive.stop.failed"), 1)
        self.assertNotIn("camera.shutdown", events)


class FreedriveStopVerificationTest(unittest.TestCase):
    def test_socket_failure_is_reported_instead_of_claiming_the_arm_locked(self):
        freedrive = load_freedrive_script()
        self.assertTrue(
            hasattr(freedrive, "FreedriveStopError"),
            "failed stop commands need an explicit failure result",
        )

        def fail_connection(*args, **kwargs):
            raise OSError("secondary interface unavailable")

        freedrive.socket.create_connection = fail_connection
        with self.assertRaisesRegex(
            freedrive.FreedriveStopError, "secondary interface unavailable"
        ):
            freedrive.stop_freedrive()

    def test_stop_waits_until_dashboard_confirms_program_not_running(self):
        freedrive = load_freedrive_script()
        sent = []
        dashboard_commands = []
        responses = iter(["Program running: true", "Program running: false"])

        class Socket:
            def sendall(self, payload):
                sent.append(payload.decode("utf-8"))

            def close(self):
                return None

        freedrive.socket.create_connection = lambda *args, **kwargs: Socket()

        def dashboard(command, **kwargs):
            dashboard_commands.append(command)
            return next(responses)

        freedrive.dashboard_exchange = dashboard
        freedrive.time.sleep = lambda *_: None
        freedrive.stop_freedrive()

        self.assertIn("end_freedrive_mode()", sent[0])
        self.assertEqual(dashboard_commands, ["running", "running"])


class UR3ControlModeLockTest(unittest.TestCase):
    def test_freedrive_and_moveit_wrappers_refuse_an_existing_control_session(self):
        worktree_root = FREEDRIVE_SCRIPT.parents[5]
        with tempfile.TemporaryDirectory() as directory:
            temporary = Path(directory)
            fake_bin = temporary / "bin"
            fake_devel = temporary / "devel"
            fake_bin.mkdir()
            fake_devel.mkdir()
            (fake_devel / "setup.bash").write_text(
                'export PATH="%s:$PATH"\n' % fake_bin,
                encoding="utf-8",
            )
            for executable in ("python3", "rosrun"):
                path = fake_bin / executable
                path.write_text("#!/usr/bin/env bash\nexit 99\n", encoding="utf-8")
                path.chmod(0o755)

            lock_path = temporary / "ur3-control.lock"
            with lock_path.open("w") as held_lock:
                fcntl.flock(held_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                environment = dict(os.environ)
                environment["TRACER_WS"] = str(temporary)
                environment["TRACER_UR3_CONTROL_LOCK"] = str(lock_path)
                for wrapper in (
                    worktree_root / "ur3_freedrive.sh",
                    worktree_root / "ur3_moveit_headless.sh",
                ):
                    result = subprocess.run(
                        [str(wrapper)],
                        env=environment,
                        capture_output=True,
                        text=True,
                        timeout=5.0,
                        check=False,
                    )
                    self.assertEqual(
                        result.returncode,
                        75,
                        "%s bypassed the held control lock: %s"
                        % (wrapper.name, result.stderr or result.stdout),
                    )


if __name__ == "__main__":
    unittest.main()
