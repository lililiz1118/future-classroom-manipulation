# UR3 Control-Chain Resilience Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the UR3 reverse interface stable under AnyGrasp CPU load on the current generic kernel, and fail closed through a latched STARTING/READY/FAULT control-chain health model when execution safety is lost.

**Architecture:** One versioned UR runtime policy file is loaded into immutable Python configuration and passed through our launch wrappers, while one independent AnyGrasp resource policy is applied before NumPy/PyTorch imports. A pure control-chain state machine owns STARTING/READY/FAULT transitions; a thin ROS monitor converts robot, safety, program, controller, and joint-state observations into that model and makes MoveIt unavailable on terminal FAULT.

**Tech Stack:** ROS Noetic, Python 3, roslaunch XML, PyYAML, rospy, controller_manager_msgs, ur_dashboard_msgs/ur_msgs, sensor_msgs, std_msgs, PyTorch, unittest/nosetests.

**Spec:** `docs/superpowers/specs/2026-08-31-ur3-control-chain-resilience-design.md`

## Global Constraints

- Work only in `/home/jt001/tracer_ws/.worktrees/ur3-headless-moveit` on `codex/ur3-headless-moveit`.
- Do not modify any file under `src/ur_ros/Universal_Robots_ROS_Driver/ur_robot_driver`, including official `ur_control.launch`.
- Do not introduce PREEMPT_RT work, kernel tuning, CPU affinity, or automatic motion recovery.
- Preserve the independently launched AnyGrasp workflow; do not add AnyGrasp to the UR3 startup coordinator.
- Preserve the user's existing unstaged `src/tracer/tracer_ros/tracer_bringup/config/ur3_headless_moveit.rviz` changes and never stage them.
- Remove the obsolete `--allow-reduced` path completely; NORMAL is the only READY safety mode.
- Do not continue or replay a trajectory after FAULT. A complete control-chain restart is mandatory.
- Use literal, focused `git add <paths...>` commands only. Never use `git add .`.

---

### Task 1: Central UR runtime policy and explicit receive-timeout propagation

**Files:**
- Create: `src/tracer/tracer_ros/tracer_bringup/config/ur3_runtime.yaml`
- Create: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/runtime_config.py`
- Create: `src/tracer/tracer_ros/tracer_bringup/test/test_runtime_config.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/CMakeLists.txt`
- Modify: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_startup.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_cli.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_cli.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py`

**Interfaces:**
- Produces: immutable `UrRuntimePolicy` with `robot_receive_timeout`, `health_evaluation_period`, `controller_poll_period`, `joint_state_timeout`, and `ready_joint_samples`.
- Produces: `load_ur_runtime_policy(path: str) -> UrRuntimePolicy` with strict numeric/range validation and unknown/missing-key errors.
- Changes: `StartupConfig.runtime_policy: UrRuntimePolicy`; no duplicate health constants in runtime code.
- Removes: `StartupConfig.allow_reduced` and CLI flag `--allow-reduced`.

- [ ] Add failing tests for the checked-in YAML values (`0.10`, `0.10`, `0.25`, `0.50`, `2`), immutable loading, invalid/missing values, CLI loading of the default policy, and rejection of the removed `--allow-reduced` flag.
- [ ] Run `python3 -m unittest src/tracer/tracer_ros/tracer_bringup/test/test_runtime_config.py src/tracer/tracer_ros/tracer_bringup/test/test_headless_cli.py src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py -v` and confirm failures refer to the absent policy module/config and old reduced-mode interface.
- [ ] Implement the minimal policy dataclass/loader, add `config/ur3_runtime.yaml`, wire it through the CLI and `StartupConfig`, remove every `allow_reduced` branch, and register the new test in CMake.
- [ ] Re-run the focused tests and require all to pass.
- [ ] Run `grep -R -n -- '--allow-reduced\|allow_reduced' src/tracer/tracer_ros/tracer_bringup` and require no production or test references.
- [ ] Commit only the files in this task with `git commit -m "refactor: centralize UR3 runtime policy"`.

### Task 2: Launch-wrapper timeout contract without upstream edits

**Files:**
- Modify: `src/tracer/tracer_ros/tracer_bringup/launch/ur3_headless_driver.launch`
- Modify: `src/tracer/tracer_ros/tracer_bringup/launch/tracer_ur_bringup.launch`
- Modify: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_runtime.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py`

**Interfaces:**
- `ur3_headless_driver.launch` declares `robot_receive_timeout` with the controlled default `0.10` and forwards it.
- `tracer_ur_bringup.launch` declares a pass-through argument retaining the upstream-compatible default `0.02`, then forwards it to official `ur_control.launch`.
- `RosRuntime.start_driver()` explicitly emits `robot_receive_timeout:=<runtime_policy.robot_receive_timeout>` even though the headless launch has the same default.

- [ ] Extend launch-contract tests to trace `robot_receive_timeout` through both owned launch files to the official include, and runtime-command tests to require the literal effective argument `robot_receive_timeout:=0.1`/`0.10`.
- [ ] Run `python3 -m unittest src/tracer/tracer_ros/tracer_bringup/test/test_headless_launch.py src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py -v` and confirm the new assertions fail.
- [ ] Add only the two wrapper arguments/forwarding edges and the explicit runtime command argument; do not edit the official driver package.
- [ ] Re-run the focused tests and require all to pass.
- [ ] Run `git diff -- src/ur_ros/Universal_Robots_ROS_Driver/ur_robot_driver` and require empty output.
- [ ] Commit only the files in this task with `git commit -m "fix: pass controlled UR receive timeout"`.

### Task 3: Pure latched control-chain state machine

**Files:**
- Create: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/control_chain_health.py`
- Create: `src/tracer/tracer_ros/tracer_bringup/test/test_control_chain_health.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/CMakeLists.txt`

**Interfaces:**
- Produces: `ControlChainState(Enum)` with exactly `STARTING`, `READY`, and `FAULT`.
- Produces: immutable `ControlChainSnapshot` containing normalized robot mode, safety mode, program-running value, trajectory-controller status/reason, joint-state completeness, receive time, header stamp, and advancing-sample count.
- Produces: `ControlChainHealth.observe_*()` methods plus `evaluate(now: float) -> ControlChainState`, `fault_reason`, and `readiness_blockers(now: float)`.
- Invariant: STARTING becomes READY only when robot mode is `RUNNING`, safety mode is `NORMAL`, program is true, target controller is exclusively running, and at least two complete, advancing, non-stale UR joint samples exist.
- Invariant: after READY, the first invalid observation or stale-joint evaluation latches FAULT permanently and preserves the first reason.

- [ ] Write table-driven pure unit tests for every READY prerequisite, two-sample advancement, stale/future/incomplete joint data, `True -> False` program transition, stopped/conflicting controller, abnormal robot/safety modes, first-reason preservation, and the impossibility of FAULT returning to READY.
- [ ] Run `python3 -m unittest src/tracer/tracer_ros/tracer_bringup/test/test_control_chain_health.py -v` and confirm import failure because the module does not exist.
- [ ] Implement the smallest lock-protected state machine with no ROS imports and deterministic injected monotonic/ROS timestamps.
- [ ] Re-run the focused tests and require all to pass.
- [ ] Register the test in CMake and run the complete tracer_bringup unit suite.
- [ ] Commit only the files in this task with `git commit -m "feat: add latched UR3 control-chain health state"`.

### Task 4: ROS health monitor and fail-closed MoveIt supervision

**Files:**
- Create: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/control_chain_monitor.py`
- Create: `src/tracer/tracer_ros/tracer_bringup/test/test_control_chain_monitor.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_runtime.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/src/tracer_bringup/headless_startup.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_runtime.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/test/test_headless_startup.py`
- Modify: `src/tracer/tracer_ros/tracer_bringup/CMakeLists.txt`

**Interfaces:**
- Produces: `RosControlChainMonitor` subscribing to `/ur/ur_hardware_interface/robot_mode`, `/ur/ur_hardware_interface/safety_mode`, `/ur/ur_hardware_interface/robot_program_running`, and `/ur/joint_states`.
- Polls: `/ur/controller_manager/list_controllers` every configured `controller_poll_period`, normalizing `assert_exclusive_controller()` results into healthy/reason observations.
- Produces: `start()`, `wait_until_ready(timeout)`, `raise_if_fault()`, `state`, `fault_reason`, and idempotent `stop()`.
- Changes: `RosRuntime.wait_control_chain_ready(config)` replaces the one-shot `wait_driver_ready()` decision and retains calibration/speed sanity checks after state READY.
- Changes: every later blocking readiness loop and `supervise()` checks the latched health state.
- Produces: `_disable_moveit_execution()` that stops/removes the managed `move_group` process before raising `StartupError("CONTROL CHAIN FAULT: ... Full restart required")`.
- Invariant: ROS process liveness can never override FAULT; no monitor callback starts controllers, restarts programs, or resumes trajectories.

- [ ] Add mocked-monitor tests for topic normalization, controller polling, descriptive readiness timeout, idempotent stop, and immediate FAULT propagation.
- [ ] Add runtime/coordinator tests proving startup prints STARTING then READY, any post-READY violation stops `move_group` before raising, Execute endpoints are no longer managed after that stop, shutdown still follows the existing order, and no recovery/retry method is called.
- [ ] Run the four focused health/runtime/startup test modules and confirm the new assertions fail.
- [ ] Implement the ROS adapter and minimal runtime/coordinator integration; keep dashboard power/brake, gripper, D405, speed, MoveIt, and RViz ordering unchanged.
- [ ] Re-run the focused tests and then the entire tracer_bringup unit suite; require all to pass.
- [ ] Commit only the files in this task with `git commit -m "feat: fail closed on UR3 control-chain faults"`.

### Task 5: Central AnyGrasp CPU resource policy

**Files:**
- Create: `src/anygrasp_ros/config/anygrasp_resources.yaml`
- Create: `src/anygrasp_ros/src/anygrasp_ros/runtime_resources.py`
- Create: `src/anygrasp_ros/test/test_runtime_resources.py`
- Modify: `src/anygrasp_ros/scripts/anygrasp_d405_node.py`
- Modify: `src/anygrasp_ros/launch/anygrasp_d405.launch`
- Modify: `src/anygrasp_ros/CMakeLists.txt`
- Modify: `src/anygrasp_ros/test/test_anygrasp_launch_config.py`
- Modify: `src/anygrasp_ros/test/test_anygrasp_node_contract.py`
- Modify: `src/anygrasp_ros/test/test_anygrasp_node_runtime.py`

**Interfaces:**
- Checked-in conservative defaults: PyTorch intra-op `2`, inter-op `1`; `OMP_NUM_THREADS=2`; `MKL_NUM_THREADS=2`; `OPENBLAS_NUM_THREADS=2`; process nice increment `10`.
- Produces: immutable `AnyGraspResourcePolicy`, strict `load_resource_policy(path)`, `apply_process_environment(policy)`, and `configure_torch(torch_module, policy)`.
- Launch sets only `ANYGRASP_RESOURCE_CONFIG=<resolved YAML path>`; individual thread values never appear in launch or node code.
- Node applies process environment and nice before importing NumPy, then applies PyTorch intra/inter-op settings immediately before importing `gsnet`, and logs every requested/effective value once.
- All settings remain configurable through the YAML path argument and a replacement YAML file; defaults are not duplicated elsewhere.

- [ ] Add pure tests for YAML parsing/validation, environment mutation, nice invocation/error reporting, torch setter calls/effective reads, and invalid values.
- [ ] Extend AST/launch tests to prove resource setup precedes NumPy, SDK/torch setup precedes `gsnet`, the launch passes one config path, and no thread/nice literals are scattered through scripts/launch.
- [ ] Run `python3 -m unittest discover -s src/anygrasp_ros/test -v` with the AnyGrasp interpreter and confirm the new tests fail.
- [ ] Implement the resource module/YAML and wire the launch/node at the two import boundaries without changing inference behavior or coupling AnyGrasp to UR startup.
- [ ] Re-run the full AnyGrasp unit suite and require all to pass.
- [ ] Run a source scan for `OMP_NUM_THREADS`, `MKL_NUM_THREADS`, `OPENBLAS_NUM_THREADS`, `set_num_threads`, `set_num_interop_threads`, and `nice`; require policy values to exist only in the resource YAML and application logic only in `runtime_resources.py`.
- [ ] Commit only the files in this task with `git commit -m "perf: bound AnyGrasp CPU resources"`.

### Task 6: Documentation and full automated verification

**Files:**
- Modify: `src/tracer/tracer_ros/tracer_bringup/README.md` if present, otherwise the existing headless-operation document identified by repository search.
- Modify: `src/anygrasp_ros/README.md`

**Interfaces:**
- Documents: STARTING/READY/FAULT criteria, terminal restart requirement, why node liveness is insufficient, effective timeout, AnyGrasp defaults/override file, and the fact that PREEMPT_RT is out of scope.
- Documents: operator fault-injection procedure that stops the robot control program only while no trajectory is executing, plus the expected Execute denial.

- [ ] Add/update documentation tests only if the repository already enforces documented commands or paths; otherwise review the rendered Markdown directly.
- [ ] Update the minimum relevant documents without changing unrelated setup guidance.
- [ ] Run `python3 -m unittest discover -s src/tracer/tracer_ros/tracer_bringup/test -v`.
- [ ] Run `/home/jt001/.conda/envs/anygrasp/bin/python -m unittest discover -s src/anygrasp_ros/test -v`.
- [ ] Run `catkin_make` from the worktree and require exit code 0.
- [ ] Run `git diff --check`, inspect `git status --short`, and confirm the pre-existing RViz file is still unstaged and unmodified by this work.
- [ ] Commit only the documentation files with `git commit -m "docs: explain UR3 fault-latched operation"`.

### Task 7: Controlled live validation on the generic kernel

**Files:**
- Verify only; do not edit production files during this task unless a newly reproduced defect first receives a failing regression test.

**Interfaces:**
- Consumes: `UR3 + D405 + independently launched AnyGrasp + RViz`.
- Produces: timestamped command/log evidence for every acceptance criterion, with no automatic motion continuation after fault.

- [ ] Confirm `uname -r` still reports the generic kernel and record that PREEMPT_RT was not introduced.
- [ ] With the physical workspace clear and the hardware emergency stop available, run the existing guarded UR3 startup, manually enter `START`, and observe explicit `STARTING -> READY`.
- [ ] Query `/ur/ur_hardware_interface/robot_receive_timeout` and require `0.10`; also inspect the driver process/roslaunch arguments to prove the value was passed through our wrapper.
- [ ] Require robot mode `RUNNING`, safety mode `NORMAL`, `robot_program_running=True`, target trajectory controller `running` with all six joints, and advancing `/ur/joint_states` within the configured `0.50 s` threshold.
- [ ] Launch AnyGrasp independently with the resource policy, record effective PyTorch/OMP/MKL/OpenBLAS/nice values from its startup log, and verify a normal large D405 PointCloud2 is processed to grasp output while RViz remains running.
- [ ] Observe the combined system under repeated normal AnyGrasp processing for a bounded validation window; search driver logs for `Sending data through socket failed`, reverse-interface disconnects, controller stops, or false health reports.
- [ ] While no trajectory is executing, intentionally stop the robot control program once. Require immediate terminal `FAULT` with the exact first cause, managed `move_group` termination, and a full-restart-required diagnostic.
- [ ] Attempt a new RViz MoveIt Execute after FAULT and prove it cannot be sent because the managed MoveIt execution endpoint is unavailable; do not restart or resume the prior motion automatically.
- [ ] Perform one full clean control-chain restart, re-establish READY, then shut down using the existing operator flow.
- [ ] Capture final `git status`, recent commit list, relevant ROS/log excerpts, and state whether any reverse-interface drop was observed during the bounded healthy-load window.

## Final Review Checklist

- [ ] Map each of the ten user requirements to at least one passing automated test or live evidence item.
- [ ] Search the implementation plan and changed source for unfinished-work markers, dummy values, duplicated resource settings, and unhandled recovery branches; require none.
- [ ] Confirm exact type/value consistency between YAML loaders, dataclasses, roslaunch string formatting, ROS message normalization, and tests.
- [ ] Confirm official UR driver files and the existing RViz configuration were not included in any task commit.
- [ ] Before claiming completion, invoke `superpowers:verification-before-completion` and report only evidence observed in the final verification run.
- [ ] Invoke `superpowers:requesting-code-review`, address any findings with regression tests, and repeat affected verification.
