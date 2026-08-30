# UR3 + D405 Independent Manipulation Startup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `ur3_moveit_headless.sh` provide a standalone UR3 + AG95 + D405 + MoveIt + RViz path that reuses a healthy external D405, starts one when absent, and never depends on the Tracer base launch.

**Architecture:** Extend the staged Python coordinator with a D405 configuration flag, pure camera-node classification, owned-versus-external process semantics, and live color/depth/camera-info readiness checks. A small D405-only launch wrapper avoids the base stack; the RViz configuration embeds the exact color topic.

**Tech Stack:** Ubuntu 20.04, ROS Noetic, Python 3 `unittest`, rospy, roslaunch XML, RViz YAML, catkin.

**Spec:** `docs/superpowers/specs/2026-08-26-ur3-d405-manipulation-startup-design.md`

## Global Constraints

- Work only in `/home/jt001/tracer_ws/.worktrees/ur3-headless-moveit` on `codex/ur3-headless-moveit`.
- Do not modify `nav_test_ws`, navigation, D455 drivers, MID-360, IMU, or global orchestration.
- Automated checks must not connect to Dashboard, power on, release brakes, set speed, send motion, reset USB, or kill external ROS nodes.
- Default mode requires D405; `--no-d405` preserves arm-only operation.
- Reuse a complete healthy external D405 and never stop it.
- Start and own D405 only when both expected D405 nodes are absent.
- Reject any D455 node or partial D405 node set with `StartupError`.
- RViz topic is exactly `/d405/color/image_raw`, without surrounding whitespace.
- No verification step executes a trajectory.

---

## Preparation

- [ ] **Step 1: Confirm the worktree is isolated and clean**

```bash
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
git rev-parse --git-dir
git rev-parse --git-common-dir
git branch --show-current
git status --short
```

Expected: linked worktree, branch `codex/ur3-headless-moveit`, empty status.

- [ ] **Step 2: Establish a clean baseline**

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -m unittest discover \
  -s src/tracer/tracer_ros/tracer_bringup/test \
  -p 'test_headless_*.py' -v
```

Expected: build exits 0 and all existing headless tests pass. Stop and investigate any baseline failure.

---

### Task 1: Add the D405 operator configuration

**Files:**
- Modify: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_startup.py:21-36`
- Modify: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_cli.py:28-49,73-87`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_cli.py:16-39`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py:107-181`

**Interfaces:**
- Consumes: existing `StartupConfig` and `build_argument_parser()`.
- Produces: `StartupConfig.enable_d405: bool = True` and CLI flag `--no-d405` with destination `enable_d405`.

- [ ] **Step 1: Write the failing CLI and config tests**

Add to `test_headless_cli.py`:

```python
def test_d405_is_required_by_default_and_can_be_disabled(self):
    parser = build_argument_parser()
    self.assertTrue(parser.parse_args([]).enable_d405)
    self.assertFalse(parser.parse_args(["--no-d405"]).enable_d405)
```

Extend the existing configuration-default test:

```python
self.assertTrue(startup_config.enable_d405)
```

- [ ] **Step 2: Verify RED**

```bash
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_cli.py -v
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py -v
```

Expected: FAIL because argparse and `StartupConfig` do not expose `enable_d405`.

- [ ] **Step 3: Implement the minimal interface**

Add to `StartupConfig`:

```python
enable_d405: bool = True
```

Add to `build_argument_parser()`:

```python
parser.add_argument(
    "--no-d405",
    action="store_false",
    dest="enable_d405",
    help="run UR3, AG95, MoveIt and RViz without requiring or starting D405",
)
```

Pass it into `StartupConfig`:

```python
enable_d405=arguments.enable_d405,
```

Extend the startup summary format and argument tuple with:

```python
"UR3 %s | robot=%s | safety=%s | calibration=%s | AG95=%s | D405=%s | speed=%.0f%%"

"required" if self.config.enable_d405 else "disabled",
```

Keep the existing six summary arguments in their current order, insert the D405 expression after `self.config.gripper_device`, and leave `self.config.speed_slider * 100.0` last.

Keep the exact `START` confirmation and `--preflight-only` boundary unchanged.

- [ ] **Step 4: Verify GREEN**

```bash
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_cli.py -v
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py -v
python3 -m unittest discover \
  -s src/tracer/tracer_ros/tracer_bringup/test \
  -p 'test_headless_*.py' -v
```

Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_startup.py \
  src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_cli.py \
  src/tracer/tracer_ros/tracer_bringup/test/test_headless_cli.py \
  src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py
git commit -m "feat: configure D405 for headless manipulation"
```

---

### Task 2: Classify camera nodes and reject conflicts

**Files:**
- Modify: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_runtime.py:33-46,88-123,171-201`
- Modify: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_startup.py:106-109`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py:86-97`
- Modify: `src/tracer/tracer_ros/tracer_bringup/package.xml:58-73`

**Interfaces:**
- Consumes: `StartupConfig.enable_d405` and one `rosnode list` snapshot.
- Produces: `classify_d405_nodes(nodes: Iterable[str], enable_d405: bool) -> str` returning `disabled`, `absent`, or `external`; `RosRuntime.d405_state`; `assert_no_conflicts(config)`.

- [ ] **Step 1: Write failing classification tests**

Import `classify_d405_nodes`, then add:

```python
class CameraNodeClassificationTest(unittest.TestCase):
    def test_disabled_complete_d405_is_ignored(self):
        nodes = ["/d405/realsense2_camera", "/d405/realsense2_camera_manager"]
        self.assertEqual(classify_d405_nodes(nodes, False), "disabled")

    def test_absent_and_external_states(self):
        self.assertEqual(classify_d405_nodes(["/rosout"], True), "absent")
        self.assertEqual(
            classify_d405_nodes(
                ["/d405/realsense2_camera", "/d405/realsense2_camera_manager"],
                True,
            ),
            "external",
        )

    def test_partial_d405_is_rejected(self):
        for node in (
            "/d405/realsense2_camera",
            "/d405/realsense2_camera_manager",
        ):
            with self.subTest(node=node), self.assertRaisesRegex(
                StartupError, "Incomplete D405"
            ):
                classify_d405_nodes([node], True)

    def test_any_d455_is_rejected_even_when_d405_is_disabled(self):
        for node in (
            "/d455/realsense2_camera",
            "/d455/realsense2_camera_manager",
        ):
            with self.subTest(node=node), self.assertRaisesRegex(
                StartupError, "D455"
            ):
                classify_d405_nodes([node], False)
```

Change `FakeRuntime.assert_no_conflicts` to accept `config` and record `no_conflicts:d405` or `no_conflicts:no-d405`. Update existing expected events.

- [ ] **Step 2: Add the failing package dependency test**

```python
self.assertIn("realsense2_camera", dependencies)
```

- [ ] **Step 3: Verify RED**

```bash
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py CameraNodeClassificationTest -v
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py StartupCoordinatorTest -v
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py \
  HeadlessLaunchTest.test_bringup_declares_the_ag95_runtime_dependencies -v
```

Expected: missing classifier, signature mismatch, and missing dependency.

- [ ] **Step 4: Implement classification and config propagation**

Add:

```python
D405_NODES = {
    "/d405/realsense2_camera",
    "/d405/realsense2_camera_manager",
}
D455_NODES = {
    "/d455/realsense2_camera",
    "/d455/realsense2_camera_manager",
}


def classify_d405_nodes(nodes: Iterable[str], enable_d405: bool) -> str:
    node_set = set(nodes)
    d455 = sorted(node_set & D455_NODES)
    if d455:
        raise StartupError(
            "D455 nodes must be stopped before headless manipulation: %s"
            % ", ".join(d455)
        )
    if not enable_d405:
        return "disabled"
    present = node_set & D405_NODES
    if not present:
        return "absent"
    if present == D405_NODES:
        return "external"
    missing = sorted(D405_NODES - present)
    raise StartupError(
        "Incomplete D405 node set; stop the old D405 launch before retrying; "
        "present=%s missing=%s"
        % (", ".join(sorted(present)), ", ".join(missing))
    )
```

Initialize:

```python
self.d405_state = "absent"
```

Change the method to `assert_no_conflicts(self, config: StartupConfig)`, preserve existing control-node and RSP checks, then set:

```python
self.d405_state = classify_d405_nodes(nodes, config.enable_d405)
```

When no ROS master is reachable, set:

```python
self.d405_state = "disabled" if not config.enable_d405 else "absent"
```

Change the coordinator call:

```python
self.runtime.assert_no_conflicts(self.config)
```

Add `realsense2_camera` to preflight package lookup and:

```xml
<exec_depend>realsense2_camera</exec_depend>
```

- [ ] **Step 5: Verify GREEN**

```bash
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py CameraNodeClassificationTest -v
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py StartupCoordinatorTest -v
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py \
  HeadlessLaunchTest.test_bringup_declares_the_ag95_runtime_dependencies -v
python3 -m unittest discover \
  -s src/tracer/tracer_ros/tracer_bringup/test \
  -p 'test_headless_*.py' -v
```

Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_runtime.py \
  src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_startup.py \
  src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py \
  src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py \
  src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py \
  src/tracer/tracer_ros/tracer_bringup/package.xml
git commit -m "feat: reject conflicting RealSense camera nodes"
```

---

### Task 3: Add the owned D405 launch stage

**Files:**
- Create: `src/tracer/tracer_ros/tracer_bringup/launch/ur3_d405_camera.launch`
- Modify: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_runtime.py:203-237`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py`

**Interfaces:**
- Consumes: `RosRuntime.d405_state`, `StartupConfig.enable_d405`, `_launch()`.
- Produces: `start_d405(config)`, owned child label `d405_camera`, and a D405-only launch.

- [ ] **Step 1: Write failing ownership tests**

Add `from unittest import mock` with the test imports, then add:

```python
class D405LaunchOwnershipTest(unittest.TestCase):
    @staticmethod
    def config(enabled=True):
        return StartupConfig(
            robot_ip="192.168.131.3",
            reverse_ip="192.168.131.1",
            calibration_path="/tmp/real.yaml",
            expected_calibration_hash="calib_13945068365021364089",
            enable_d405=enabled,
        )

    def test_absent_camera_uses_dedicated_launch(self):
        class RecordingRuntime(RosRuntime):
            def __init__(self):
                super().__init__(environment={})
                self.launch = None

            def _launch(self, label, command):
                self.launch = (label, list(command))

        runtime = RecordingRuntime()
        runtime.d405_state = "absent"
        runtime.start_d405(self.config())
        self.assertEqual(
            runtime.launch,
            (
                "d405_camera",
                ["roslaunch", "tracer_bringup", "ur3_d405_camera.launch"],
            ),
        )

    def test_external_or_disabled_camera_is_not_started(self):
        for state, enabled in (("external", True), ("disabled", False)):
            runtime = RosRuntime(environment={})
            runtime.d405_state = state
            runtime._launch = lambda *args: self.fail("must not launch D405")
            runtime.start_d405(self.config(enabled))

    def test_owned_camera_is_registered_and_shutdown_with_sigint(self):
        process = mock.Mock(pid=4321)
        process.poll.side_effect = (None, 0)
        runtime = RosRuntime(environment={})
        runtime.processes = [("d405_camera", process)]

        with mock.patch.object(os, "getpgid", return_value=4321), mock.patch.object(
            os, "killpg"
        ) as killpg:
            runtime.shutdown()

        killpg.assert_called_once_with(4321, signal.SIGINT)
```

Import `signal` with the standard-library imports. The external/disabled test proves those cameras never enter `processes`; the shutdown test proves an owned `d405_camera` follows the existing SIGINT-first lifecycle without touching hardware.

- [ ] **Step 2: Write the failing launch-isolation test**

```python
def test_d405_launch_contains_no_base_or_d455_nodes(self):
    result = subprocess.run(
        ["roslaunch", "--nodes", "tracer_bringup", "ur3_d405_camera.launch"],
        check=True,
        capture_output=True,
        text=True,
    )
    nodes = set(result.stdout.splitlines())
    self.assertEqual(
        nodes,
        {
            "/d405/realsense2_camera_manager",
            "/d405/realsense2_camera",
            "/d405/d405_to_plate",
        },
    )
    self.assertFalse(any("d455" in node for node in nodes))
```

- [ ] **Step 3: Verify RED**

```bash
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py D405LaunchOwnershipTest -v
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py \
  HeadlessLaunchTest.test_d405_launch_contains_no_base_or_d455_nodes -v
```

Expected: method and launch file missing.

- [ ] **Step 4: Create the wrapper and runtime stage**

`ur3_d405_camera.launch`:

```xml
<?xml version="1.0"?>
<launch>
  <!-- Own only the wrist D405: no base, D455, lidar, IMU, or photo service. -->
  <include file="$(find realsense2_camera)/launch/rs_camera_d405.launch" />
</launch>
```

Runtime:

```python
def start_d405(self, config: StartupConfig) -> None:
    if not config.enable_d405 or self.d405_state == "external":
        return
    if self.d405_state != "absent":
        raise StartupError("Cannot start D405 from state: %s" % self.d405_state)
    self._launch(
        "d405_camera",
        ["roslaunch", "tracer_bringup", "ur3_d405_camera.launch"],
    )
```

- [ ] **Step 5: Verify GREEN**

```bash
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py D405LaunchOwnershipTest -v
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py \
  HeadlessLaunchTest.test_d405_launch_contains_no_base_or_d455_nodes -v
python3 -m unittest discover \
  -s src/tracer/tracer_ros/tracer_bringup/test \
  -p 'test_headless_*.py' -v
```

Expected: all pass and only three D405 nodes are listed.

- [ ] **Step 6: Commit**

```bash
git add src/tracer/tracer_ros/tracer_bringup/launch/ur3_d405_camera.launch \
  src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_runtime.py \
  src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py \
  src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py
git commit -m "feat: launch D405 independently of the Tracer base"
```

---

### Task 4: Require live D405 color, depth, and calibration data

**Files:**
- Modify: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_runtime.py:238-445`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py`

**Interfaces:**
- Consumes: rospy, `state_timeout`, owned label `d405_camera`, `sensor_msgs/Image`, `sensor_msgs/CameraInfo`.
- Produces: `wait_d405_ready(config)`, `_assert_fresh_advancing_headers()`, `_camera_startup_error()`.

- [ ] **Step 1: Write failing readiness tests**

Use fake stamps/messages matching existing `RosSnapshotTest` conventions. Add:

```python
class D405ReadinessTest(unittest.TestCase):
    def test_ready_requires_advancing_color_depth_and_matching_info(self):
        runtime = camera_runtime(
            color=[image(99.8), image(99.9)],
            depth=[image(99.8), image(99.9)],
            info=[camera_info(640, 480)],
            now=100.0,
        )
        runtime.wait_d405_ready(camera_config())

    def test_disabled_camera_returns_without_subscribing(self):
        runtime = camera_runtime(color=[], depth=[], info=[], now=100.0)
        runtime.wait_d405_ready(camera_config(enable_d405=False))

    def test_missing_color_reports_exact_topic(self):
        runtime = camera_runtime(color=[], depth=[], info=[], now=100.0)
        with self.assertRaisesRegex(StartupError, "/d405/color/image_raw"):
            runtime.wait_d405_ready(camera_config())

    def test_non_advancing_and_stale_color_are_rejected(self):
        cases = (
            ([image(99.9), image(99.9)], "not advancing"),
            ([image(97.0), image(98.0)], "stale"),
        )
        for color, message in cases:
            runtime = camera_runtime(color=color, depth=[], info=[], now=100.0)
            with self.assertRaisesRegex(StartupError, message):
                runtime.wait_d405_ready(camera_config())

    def test_camera_info_must_match_positive_color_dimensions(self):
        runtime = camera_runtime(
            color=[image(99.8), image(99.9)],
            depth=[image(99.8), image(99.9)],
            info=[camera_info(1280, 720)],
            now=100.0,
        )
        with self.assertRaisesRegex(StartupError, "camera_info"):
            runtime.wait_d405_ready(camera_config())

    def test_owned_camera_exit_replaces_generic_timeout(self):
        runtime = camera_runtime(color=[], depth=[], info=[], now=100.0)
        runtime.processes.append(
            ("d405_camera", SimpleNamespace(poll=lambda: 7))
        )
        with self.assertRaisesRegex(
            StartupError, "d405_camera exited unexpectedly with code 7"
        ):
            runtime.wait_d405_ready(camera_config())
```

Add these real helpers above `D405ReadinessTest` so the tests never contact ROS or hardware:

```python
class CameraDuration:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


class CameraStamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def __le__(self, other):
        return self.seconds <= other.seconds

    def __sub__(self, other):
        return CameraDuration(self.seconds - other.seconds)


def image(stamp, width=640, height=480):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=CameraStamp(stamp)),
        width=width,
        height=height,
    )


def camera_info(width, height):
    return SimpleNamespace(width=width, height=height)


def camera_config(enable_d405=True):
    return StartupConfig(
        robot_ip="192.168.131.3",
        reverse_ip="192.168.131.1",
        calibration_path="/tmp/real.yaml",
        expected_calibration_hash="calib_13945068365021364089",
        state_timeout=0.01,
        enable_d405=enable_d405,
    )


class FakeCameraRospy:
    class ROSException(Exception):
        pass

    def __init__(self, messages, now):
        self.messages = {topic: list(values) for topic, values in messages.items()}
        now_stamp = CameraStamp(now)
        self.Time = SimpleNamespace(now=lambda: now_stamp)

    def wait_for_message(self, topic, message_type, timeout):
        values = self.messages.get(topic, [])
        if not values:
            raise self.ROSException("timeout")
        return values.pop(0)


def camera_runtime(color, depth, info, now):
    runtime = RosRuntime(environment={})
    runtime._rospy = FakeCameraRospy(
        {
            "/d405/color/image_raw": color,
            "/d405/depth/image_rect_raw": depth,
            "/d405/color/camera_info": info,
        },
        now,
    )
    return runtime
```

- [ ] **Step 2: Verify RED**

```bash
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py D405ReadinessTest -v
```

Expected: `wait_d405_ready` missing.

- [ ] **Step 3: Implement freshness and owned-process error helpers**

```python
def _assert_fresh_advancing_headers(self, first, second, topic):
    if second.header.stamp <= first.header.stamp:
        raise StartupError("%s timestamps are not advancing" % topic)
    age = (self._rospy.Time.now() - second.header.stamp).to_sec()
    if age < 0.0 or age > 1.0:
        raise StartupError("%s is stale by %.3f seconds" % (topic, age))


def _camera_startup_error(self, message):
    for label, process in self.processes:
        if label == "d405_camera":
            code = process.poll()
            if code is not None:
                return StartupError(
                    "d405_camera exited unexpectedly with code %d" % code
                )
    return StartupError(message)
```

Make `_assert_fresh_advancing_joint_states()` delegate to the generic header helper so existing semantics stay unchanged.

- [ ] **Step 4: Implement one-deadline readiness**

`wait_d405_ready(config)` returns immediately when disabled, initializes rospy, then uses one `time.monotonic() + state_timeout` deadline. Receive two color frames, two depth frames, and one color CameraInfo. Catch `rospy.ROSException` and raise `_camera_startup_error()` naming the exact topic. Validate:

```python
color_topic = "/d405/color/image_raw"
depth_topic = "/d405/depth/image_rect_raw"
info_topic = "/d405/color/camera_info"
```

For both image streams call `_assert_fresh_advancing_headers()`. Reject non-positive color dimensions and enforce:

```python
(camera_info.width, camera_info.height) == (
    color_second.width,
    color_second.height,
)
```

The mismatch message must contain `camera_info` and both dimensions.

- [ ] **Step 5: Verify GREEN**

```bash
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py \
  D405ReadinessTest RosSnapshotTest -v
python3 -m unittest discover \
  -s src/tracer/tracer_ros/tracer_bringup/test \
  -p 'test_headless_*.py' -v
```

Expected: all pass, including existing joint-state freshness tests.

- [ ] **Step 6: Commit**

```bash
git add src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_runtime.py \
  src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py
git commit -m "feat: require live D405 image streams"
```

---

### Task 5: Integrate D405 into startup and RViz

**Files:**
- Modify: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_startup.py:153-164`
- Modify: `src/tracer/tracer_ros/tracer_bringup/config/ur3_headless_moveit.rviz:6-42`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py:99-135`

**Interfaces:**
- Consumes: `start_d405()`, `wait_d405_ready()`, `enable_d405`.
- Produces: Driver → AG95 → D405 → speed → MoveIt → RViz, and enabled display `D405 Color`.

- [ ] **Step 1: Write failing startup-order tests**

Add to `FakeRuntime`:

```python
def start_d405(self, config):
    self._event("start_d405")

def wait_d405_ready(self, config):
    self._event("d405_ready")
```

For default config, insert after `gripper_ready`:

```python
"start_d405",
"d405_ready",
```

Add a config with `enable_d405=False`, run the coordinator, and assert neither event appears.

- [ ] **Step 2: Write failing RViz assertions**

Rename the old RViz test to describe D405. Replace the Image exclusion with:

```python
self.assertIn("rviz/Image", enabled_classes)
self.assertTrue(enabled_classes.isdisjoint({"rviz/Map", "rviz/PointCloud2"}))
image_display = next(
    display
    for display in manager["Displays"]
    if display["Class"] == "rviz/Image"
)
self.assertEqual(image_display["Name"], "D405 Color")
self.assertEqual(image_display["Image Topic"], "/d405/color/image_raw")
self.assertEqual(
    image_display["Image Topic"], image_display["Image Topic"].strip()
)
self.assertEqual(image_display["Transport Hint"], "raw")
self.assertEqual(image_display["Queue Size"], 2)
```

- [ ] **Step 3: Verify RED**

```bash
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py StartupCoordinatorTest -v
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py HeadlessRvizConfigTest -v
```

Expected: missing coordinator events and Image display.

- [ ] **Step 4: Integrate the camera stage**

Immediately after gripper readiness:

```python
if self.config.enable_d405:
    self.runtime.start_d405(self.config)
    self.runtime.wait_d405_ready(self.config)
```

Keep speed, MoveIt, and RViz after D405 readiness.

- [ ] **Step 5: Add the exact RViz display**

```yaml
    - Class: rviz/Image
      Enabled: true
      Image Topic: /d405/color/image_raw
      Name: D405 Color
      Queue Size: 2
      Transport Hint: raw
      Unreliable: false
```

Insert it after RobotModel and before MotionPlanning.

- [ ] **Step 6: Verify GREEN**

```bash
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py StartupCoordinatorTest -v
python3 src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py HeadlessRvizConfigTest -v
python3 -m unittest discover \
  -s src/tracer/tracer_ros/tracer_bringup/test \
  -p 'test_headless_*.py' -v
```

Expected: all pass; YAML test proves no hidden whitespace.

- [ ] **Step 7: Commit**

```bash
git add src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_startup.py \
  src/tracer/tracer_ros/tracer_bringup/config/ur3_headless_moveit.rviz \
  src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py \
  src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py
git commit -m "feat: show D405 in headless manipulation RViz"
```

---

### Task 6: Document and verify the standalone workflow

**Files:**
- Modify: `src/tracer/tracer_ros/tracer_bringup/scripts/README_UR3_HEADLESS_MOVEIT.md:1-54`
- Verify: every file changed by Tasks 1-5

**Interfaces:**
- Consumes: final CLI, ownership behavior, topics, conflicts, and shutdown semantics.
- Produces: exact standalone, external-camera, arm-only, diagnosis, and shutdown instructions.

- [ ] **Step 1: Update the README with exact commands**

Default:

```bash
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
TRACER_WS="$PWD" ./ur3_moveit_headless.sh
```

Read-only and arm-only:

```bash
TRACER_WS="$PWD" ./ur3_moveit_headless.sh --preflight-only
TRACER_WS="$PWD" ./ur3_moveit_headless.sh --no-d405
```

State explicitly: no base launch is required; healthy external D405 is reused; owned D405 exits with headless; D455 and partial D405 are manual-stop conflicts; RViz opens the exact topic automatically; no external nodes are killed and no trajectory is auto-executed.

Add diagnostics:

```bash
rosnode list | grep -E '^/d(405|455)/'
rostopic hz /d405/color/image_raw
rostopic hz /d405/depth/image_rect_raw
rostopic info /d405/color/image_raw
```

- [ ] **Step 2: Run consistency checks**

```bash
git diff --check
python3 -m compileall -q \
  src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup \
  src/tracer/tracer_ros/tracer_bringup/scripts
```

Expected: exit 0, no output.

- [ ] **Step 3: Run all automated tests**

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 -m unittest discover \
  -s src/tracer/tracer_ros/tracer_bringup/test \
  -p 'test_headless_*.py' -v
```

Expected: 0 failures and 0 errors.

- [ ] **Step 4: Build**

```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

Expected: exit 0.

- [ ] **Step 5: Verify launch isolation without hardware**

```bash
source devel/setup.bash
roslaunch --nodes tracer_bringup ur3_d405_camera.launch
```

Expected exactly:

```text
/d405/realsense2_camera_manager
/d405/realsense2_camera
/d405/d405_to_plate
```

No D455, base, lidar, or IMU node.

- [ ] **Step 6: Run read-only preflight**

Only after old D455/partial D405 nodes have stopped:

```bash
TRACER_WS="$PWD" ./ur3_moveit_headless.sh --preflight-only
```

Expected: exits before `START`, with no hardware mutation and no camera launch.

- [ ] **Step 7: Commit**

```bash
git add src/tracer/tracer_ros/tracer_bringup/scripts/README_UR3_HEADLESS_MOVEIT.md
git commit -m "docs: explain standalone UR3 D405 startup"
git status --short
git log --oneline -7
```

Expected: clean status and one commit for each verified task.

---

## Hardware Acceptance Checkpoint

Run only after all automated checks pass. Keep the independent emergency stop accessible, clear the work area, and never click RViz Execute.

1. Stop the old headless launch first with Ctrl+C; wait for RViz, MoveIt, AG95, and UR Driver to exit.
2. Stop the old base launch with Ctrl+C; wait for D405/D455 and sensors to exit.
3. Confirm no `/d405/...` or `/d455/...` nodes remain.
4. Start the default worktree command and provide the existing exact `START` confirmation.
5. Confirm no D455 nodes and approximately 30 Hz on both D405 image topics.
6. Confirm RViz `D405 Color` is OK without manual topic entry.
7. Ctrl+C headless and confirm its owned D405 exits.
8. Start only `ur3_d405_camera.launch`, start headless again, and confirm external reuse; headless exit must leave the external D405 running.

No step sends a planning or execution goal.
