#!/usr/bin/env python3
"""D405 camera and local image viewer lifecycle for UR3 freedrive."""

import os
import signal
import subprocess
import time
from typing import Iterable


D405_NODES = {
    "/d405/realsense2_camera",
    "/d405/realsense2_camera_manager",
}


class CameraViewError(RuntimeError):
    """Raised when the freedrive camera view cannot be made ready."""


class FrameFreshnessWatchdog:
    """Track live D405 frames and reject a frozen camera stream."""

    def __init__(
        self,
        topic,
        stale_after=2.0,
        monotonic=time.monotonic,
        subscriber_factory=None,
    ):
        self.topic = topic
        self.stale_after = stale_after
        self._monotonic = monotonic
        self._last_frame = monotonic()
        if subscriber_factory is None:
            import rospy
            from sensor_msgs.msg import Image

            if not rospy.core.is_initialized():
                rospy.init_node(
                    "ur3_freedrive_camera_watchdog",
                    anonymous=True,
                    disable_signals=True,
                )
            subscriber_factory = lambda name, callback: rospy.Subscriber(
                name, Image, callback, queue_size=1
            )
        self._subscription = subscriber_factory(topic, self._on_frame)

    def _on_frame(self, _message):
        self._last_frame = self._monotonic()

    def assert_fresh(self):
        age = self._monotonic() - self._last_frame
        if age > self.stale_after:
            raise CameraViewError(
                "D405 color stream is stale (no frame for %.2f seconds)" % age
            )

    def shutdown(self):
        self._subscription.unregister()


def build_desktop_environment(
    environment=None, uid=None, path_exists=os.path.exists
):
    """Build an environment targeting the robot's active local X11 desktop."""
    result = dict(os.environ if environment is None else environment)
    desktop_uid = os.getuid() if uid is None else uid
    runtime_directory = "/run/user/%d" % desktop_uid
    xauthority = os.path.join(runtime_directory, "gdm", "Xauthority")
    if not path_exists(xauthority):
        raise CameraViewError(
            "Robot desktop Xauthority is unavailable: %s" % xauthority
        )
    result["DISPLAY"] = ":0"
    result["XAUTHORITY"] = xauthority
    result["XDG_RUNTIME_DIR"] = runtime_directory
    result["DBUS_SESSION_BUS_ADDRESS"] = "unix:path=%s/bus" % runtime_directory
    return result


def classify_d405_nodes(nodes: Iterable[str]) -> str:
    """Return absent/external, rejecting a half-started D405 launch."""
    present = set(nodes) & D405_NODES
    if not present:
        return "absent"
    if present == D405_NODES:
        return "external"
    missing = sorted(D405_NODES - present)
    raise CameraViewError(
        "Incomplete D405 node set; stop the old D405 launch before retrying; "
        "present=%s missing=%s"
        % (", ".join(sorted(present)), ", ".join(missing))
    )


class CameraViewSession:
    """Own a standalone D405 launch when needed and a local image window."""

    def __init__(
        self,
        run=subprocess.run,
        popen=subprocess.Popen,
        sleep=time.sleep,
        monotonic=time.monotonic,
        environment=None,
        uid=None,
        path_exists=os.path.exists,
        getpgid=os.getpgid,
        killpg=os.killpg,
        camera_timeout=20.0,
        watchdog_factory=None,
    ):
        self._run = run
        self._popen = popen
        self._sleep = sleep
        self._monotonic = monotonic
        self.environment = dict(os.environ if environment is None else environment)
        self.uid = os.getuid() if uid is None else uid
        self.path_exists = path_exists
        self._getpgid = getpgid
        self._killpg = killpg
        self.camera_timeout = camera_timeout
        self.processes = []
        self.watchdog = None
        self._watchdog_factory = watchdog_factory or (
            lambda topic: FrameFreshnessWatchdog(
                topic, monotonic=self._monotonic
            )
        )

    def _detect_camera_state(self):
        result = self._run(
            ["rosnode", "list"],
            capture_output=True,
            text=True,
            timeout=4.0,
            check=False,
        )
        if result.returncode == 0:
            return classify_d405_nodes(result.stdout.splitlines())
        diagnostics = str(result.stderr or result.stdout).lower()
        if any(
            marker in diagnostics
            for marker in ("master", "connection refused", "unable to communicate")
        ):
            return "absent"
        raise CameraViewError(
            "Cannot inspect existing ROS nodes: %s" % diagnostics.strip()
        )

    def _launch(self, label, command, environment):
        try:
            process = self._popen(
                list(command), env=environment, start_new_session=True
            )
        except OSError as exc:
            raise CameraViewError("Cannot start %s: %s" % (label, exc)) from exc
        self.processes.append((label, process))

    def start(self):
        try:
            self._start()
        except BaseException:
            self.shutdown()
            raise

    def _start(self):
        camera_state = self._detect_camera_state()
        if camera_state == "absent":
            self._launch(
                "d405_camera",
                ["roslaunch", "tracer_bringup", "ur3_d405_camera.launch"],
                self.environment,
            )
        self._wait_for_color_image()
        self._start_watchdog()

        desktop_environment = build_desktop_environment(
            self.environment,
            uid=self.uid,
            path_exists=self.path_exists,
        )
        self._launch(
            "d405_viewer",
            [
                "rosrun",
                "image_view",
                "image_view",
                "image:=/d405/color/image_raw",
                "_autosize:=true",
                "__name:=ur3_freedrive_camera_view",
            ],
            desktop_environment,
        )
        self._sleep(0.5)
        self.assert_alive()

    def _start_watchdog(self):
        self.watchdog = self._watchdog_factory("/d405/color/image_raw")

    def _wait_for_color_image(self):
        deadline = self._monotonic() + self.camera_timeout
        last_diagnostics = "no image received"
        command = [
            "rostopic",
            "echo",
            "-n",
            "1",
            "/d405/color/image_raw/header",
        ]
        while True:
            self.assert_alive()
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                raise CameraViewError(
                    "D405 color image did not become ready within %.1f seconds: %s"
                    % (self.camera_timeout, last_diagnostics)
                )
            try:
                ready = self._run(
                    command,
                    capture_output=True,
                    text=True,
                    timeout=min(2.0, remaining),
                    check=False,
                )
                if ready.returncode == 0:
                    return
                last_diagnostics = str(ready.stderr or ready.stdout).strip()
            except subprocess.TimeoutExpired:
                last_diagnostics = "waiting for the first color image"
            self._sleep(min(0.25, max(0.0, deadline - self._monotonic())))

    def assert_alive(self):
        for label, process in self.processes:
            code = process.poll()
            if code is not None:
                raise CameraViewError(
                    "%s exited unexpectedly with code %d" % (label, code)
                )
        if self.watchdog is not None:
            self.watchdog.assert_fresh()

    def shutdown(self):
        """Stop only processes started by this session, viewer first."""
        if self.watchdog is not None:
            self.watchdog.shutdown()
            self.watchdog = None
        processes = list(reversed(self.processes))
        for _, process in processes:
            if process.poll() is None:
                try:
                    self._killpg(self._getpgid(process.pid), signal.SIGINT)
                except (OSError, ProcessLookupError):
                    pass
        for _, process in processes:
            if process.poll() is None:
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    try:
                        self._killpg(self._getpgid(process.pid), signal.SIGTERM)
                    except (OSError, ProcessLookupError):
                        pass
                    try:
                        process.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        try:
                            self._killpg(self._getpgid(process.pid), signal.SIGKILL)
                            process.wait(timeout=1.0)
                        except (OSError, ProcessLookupError, subprocess.TimeoutExpired) as exc:
                            raise CameraViewError(
                                "Cannot terminate owned process %s" % process.pid
                            ) from exc
        self.processes = []
