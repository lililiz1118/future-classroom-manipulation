# UR3 CB3 Headless MoveIt Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a guarded single-command startup path that powers and releases a CB3 UR3 through Dashboard, starts the ROS driver in headless mode, validates the real control chain, applies a 5–10% speed slider, then starts MoveIt and RViz for Plan & Execute.

**Architecture:** Pure Python modules own Dashboard parsing, safety decisions, calibration validation, and orchestration. A ROS-facing executable implements environment checks, readiness observation and child-process supervision. Driver, move_group and RViz start as separate stages so no later stage can mask an earlier failure.

**Tech Stack:** Ubuntu 20.04, ROS Noetic, Python 3.8, `rospy`, UR ROS Driver 2.4.1, MoveIt 1, Catkin, `unittest`/`nosetests`.

**Spec:** `docs/superpowers/specs/2026-08-24-ur3-headless-moveit-design.md`

## Global Constraints

- Target controller is `192.168.131.3`; reverse ROS interface is `192.168.131.1`.
- Use `headless_mode:=true` and the real hash `calib_13945068365021364089`.
- Never automate safety restart, protective-stop unlock, E-stop recovery or trajectory execution.
- Require one exact interactive confirmation before `power on` or `brake release`.
- Accept `REDUCED` only with an explicit CLI flag.
- Default speed slider is `0.05`; reject values outside `(0, 0.10]`.
- Keep keyboard teleoperation and MoveIt Servo files unchanged and never start them from this entry.
- Tests must be offline and must not access `192.168.131.3:29999`.

---

### Task 1: Dashboard and Safety Domain

**Files:**
- Create: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_dashboard.py`
- Create: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_dashboard.py`

**Interfaces:**
- Produces: `DashboardClient.query(command) -> str`, `DashboardClient.command(command) -> str`, `parse_robot_mode(str) -> str`, `parse_safety_mode(str) -> str`, and `assert_safe_mode(mode, allow_reduced) -> None`.

- [ ] Write tests with literal Dashboard greetings/responses for `NORMAL`, explicit `REDUCED`, and every blocked safety family.
- [ ] Run `python3 -m unittest ...test_headless_dashboard -v`; verify failure because the module does not exist.
- [ ] Implement strict ASCII line exchange, response normalization, timeouts and safety rejection.
- [ ] Re-run the focused test and the full tracer_bringup test directory.

### Task 2: Calibration, Controller Exclusivity and Orchestration

**Files:**
- Create: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_startup.py`
- Create: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py`
- Create: `src/tracer/tracer_ros/tracer_bringup/setup.py`

**Interfaces:**
- Consumes: Dashboard status and a runtime object.
- Produces: `validate_calibration(path, expected_hash) -> str`, `assert_exclusive_controller(snapshot, target, joints) -> None`, and `StartupCoordinator.run() -> None`.

- [ ] Write tests proving bad/missing/default calibration fails and the real fixture hash succeeds.
- [ ] Write tests proving confirmation rejection causes zero mutating Dashboard/runtime calls.
- [ ] Write tests proving successful order is preflight → confirm → power → brake → Driver → ready → speed → move_group → ready → RViz.
- [ ] Write controller snapshots showing the target alone succeeds while any second running joint controller fails.
- [ ] Run the focused tests and verify expected missing-feature failures.
- [ ] Implement the smallest domain/orchestration code satisfying those tests, then re-run all unit tests.

### Task 3: ROS Runtime and Staged Launch Files

**Files:**
- Create: `src/tracer/tracer_ros/tracer_bringup/scripts/ur3_headless_moveit.py`
- Create: `src/tracer/tracer_ros/tracer_bringup/launch/ur3_headless_driver.launch`
- Create: `src/tracer/tracer_ros/tracer_bringup/launch/ur3_moveit_execution.launch`
- Create: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/CMakeLists.txt`
- Modify: `src/tracer/tracer_ros/tracer_bringup/package.xml`

**Interfaces:**
- Implements runtime methods used by `StartupCoordinator`: environment/conflict checks, child launch, Driver readiness, speed slider, move_group readiness, RViz launch and cleanup.

- [ ] Write launch dump tests that assert the real calibration hash, headless flag, reverse IP, enabled trajectory execution and scaled controller action mapping.
- [ ] Run the launch tests and verify failure because launch files are missing.
- [ ] Implement the two launch files and ROS runtime, including two-message joint freshness and controller claimed-resource checks.
- [ ] Add Catkin Python setup, runtime dependencies and executable/test installation.
- [ ] Run launch dump tests and all package tests.

### Task 4: Single-Command UX and Documentation

**Files:**
- Create: `ur3_moveit_headless.sh`
- Create: `src/tracer/tracer_ros/tracer_bringup/scripts/README_UR3_HEADLESS_MOVEIT.md`
- Modify: `src/tracer/tracer_ros/tracer_bringup/CMakeLists.txt`

**Interfaces:**
- Produces: `./ur3_moveit_headless.sh [--allow-reduced] [--speed-slider FRACTION] [--preflight-only]`.

- [ ] Add a shell syntax test and a CLI `--help` smoke test with no robot access.
- [ ] Implement the wrapper to source Noetic/workspace and set the UR-network ROS defaults without overwriting explicit caller values.
- [ ] Document startup, exact confirmation, state gates, Plan/Execute workflow, shutdown and recovery exclusions.
- [ ] Re-run shell/CLI tests and all package tests.

### Task 5: Verification and Deployment

**Files:**
- Verify all files above; modify only defects found by verification.

**Interfaces:**
- Produces: verified files ready for the robot's main workspace.

- [ ] Run all tracer_bringup unit tests with the isolated worktree first in `ROS_PACKAGE_PATH`.
- [ ] Run `roslaunch --dump-params` for both new launch files and confirm it performs no hardware actions.
- [ ] Run `bash -n ur3_moveit_headless.sh` and Python byte-compilation.
- [ ] Run a scoped Catkin build for `tracer_bringup`.
- [ ] Review `git diff --check`, `git status --short`, and the requirements checklist.
- [ ] Copy only verified feature files into `/home/jt001/tracer_ws`, preserving `readme.txt` and `src/ur_keyboard_teleop/`.
- [ ] Re-run offline tests and the scoped build in the main workspace; do not run the live startup command.
