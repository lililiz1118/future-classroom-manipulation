# UR3 Control-Chain Resilience Design

## Context

The UR3 headless control path currently uses the upstream driver's default
`robot_receive_timeout` of 20 ms. On the robot PC's generic Ubuntu kernel,
bursty perception work can delay the control loop long enough for the robot-side
URScript to time out and close the reverse interface. The ROS driver process,
MoveIt, and RViz can remain alive after that disconnect, which makes process
liveness look healthy even though trajectory execution is no longer available.

Recent logs show that eight of ten unexpected reverse-interface disconnects
occurred during an AnyGrasp point-cloud callback. The active AnyGrasp environment
uses PyTorch with MKL and OpenMP, starts with 10 intra-op threads and 8 inter-op
threads, and has no OMP or MKL limits. NumPy uses OpenBLAS. The fix therefore
needs both a controlled receive-timeout policy and explicit resource limits for
perception, plus health monitoring based on control semantics rather than
process liveness.

## Scope

This change will:

- pass `robot_receive_timeout=0.10` through project-owned UR3 launch files;
- centralize UR3 control timing and health thresholds in a runtime policy;
- centralize AnyGrasp CPU resource limits in one configurable resource file;
- add a terminal `STARTING -> READY -> FAULT` control-chain state machine;
- fail closed when the control chain becomes unhealthy after reaching READY;
- stop MoveIt before the rest of the managed control chain on FAULT;
- preserve the existing guarded startup order and independent AnyGrasp launch;
- remove `--allow-reduced`, because READY requires safety mode NORMAL.

This change will not:

- modify upstream `ur_robot_driver` sources or upstream launch files;
- install or configure a PREEMPT_RT kernel;
- automatically restart the URScript, reconnect the driver, or resume motion;
- make AnyGrasp part of the UR3 startup lifecycle;
- modify the user's uncommitted RViz configuration.

## Architecture

### Runtime policy

Project-owned control settings will be represented by a validated immutable
runtime policy. Its initial controlled values are:

- reverse-interface receive timeout: 0.10 seconds;
- health evaluation period: 0.10 seconds;
- controller polling period: 0.25 seconds;
- joint-state stale threshold: 0.50 seconds.

`StartupConfig` carries this policy so the same value used to launch the driver
is also visible to tests and operator diagnostics. The project launch chain
will expose and forward `robot_receive_timeout` as follows:

```text
ur3_headless_moveit.py
  -> ur3_headless_driver.launch
     -> tracer_ur_bringup.launch
        -> upstream ur_control.launch
```

The headless entry explicitly passes `robot_receive_timeout:=0.10`. The generic
`tracer_ur_bringup.launch` keeps an explicit argument and forwards it without
changing the upstream file.

### AnyGrasp resource policy

AnyGrasp remains independently launched. One package-owned YAML file contains
all CPU resource settings:

- PyTorch intra-op threads: 2;
- PyTorch inter-op threads: 1;
- OMP threads: 2;
- MKL threads: 2;
- OpenBLAS threads: 2;
- process nice value: 10.

The launch file passes only the resource-file path. A focused resource-policy
module loads and validates the file before NumPy or PyTorch initialization,
sets the thread-related environment variables, and lowers process priority.
Immediately before the SDK imports PyTorch-backed model code, it applies the
PyTorch intra-op and inter-op limits. The node logs the effective settings.

The values are conservative verification defaults, not hard-coded permanent
assumptions. Operators can select a different validated resource YAML through a
launch argument without editing Python or launch internals.

### Control-chain health state machine

A pure, ROS-independent state machine owns these states:

```text
STARTING -> READY -> FAULT
```

FAULT is terminal for the lifetime of the launcher. Healthy observations after
a fault do not return the system to READY.

A thin ROS monitor supplies observations to the state machine:

- `/ur/ur_hardware_interface/robot_mode`;
- `/ur/ur_hardware_interface/safety_mode`;
- `/ur/ur_hardware_interface/robot_program_running`;
- `/ur/joint_states`;
- `/ur/controller_manager/list_controllers`.

The monitor starts as soon as the UR driver is available and remains active
through gripper, camera, MoveIt, and RViz startup and throughout supervision.

## State semantics

### STARTING

STARTING means the launcher is collecting a complete, current control snapshot.
It does not imply that MoveIt execution is safe. Transition to READY requires
all of the following at the same time:

- robot mode equals RUNNING;
- safety mode equals NORMAL;
- `robot_program_running` is true;
- `ur_arm_scaled_pos_joint_traj_controller` is running and exclusively claims
  all six UR arm joints;
- two qualifying `/ur/joint_states` samples contain all six arm joints, have
  advancing timestamps, and the latest sample is no older than the configured
  stale threshold.

Failure to reach READY within the existing startup timeout is a startup error,
not a runtime FAULT, because the system was never declared executable.

### READY

READY is the only state in which the launcher may present the control chain as
healthy and continue exposing MoveIt execution. Readiness remains continuously
monitored; process liveness alone never satisfies health.

### FAULT

After READY, the first observed violation atomically transitions to FAULT and
records one stable reason. Fault triggers include:

- `robot_program_running` changes from true to false;
- the target trajectory controller is no longer running or loses its required
  joint claims;
- joint states stop arriving or become stale;
- robot mode is no longer RUNNING;
- safety mode is no longer NORMAL.

The fault reason names the failed condition and observed value. Subsequent
symptoms do not overwrite the original reason.

## Fail-closed behavior

When FAULT is observed, the runtime will:

1. print a clear `CONTROL CHAIN FAULT` diagnostic with the first cause;
2. stop the managed MoveIt process first, removing the Execute action endpoint;
3. raise a startup/runtime error into the coordinator;
4. reuse the existing ordered shutdown for the remaining managed components;
5. tell the operator that a complete control-chain restart is required.

It will not resend the robot program, restart a controller, replay a trajectory,
or return to READY. RViz may close as part of normal ordered shutdown, but
MoveIt is disabled first so a new Execute cannot be accepted during teardown.

## Existing safety behavior

The startup confirmation, Dashboard power-on and brake-release sequence,
calibration check, 5% default speed slider, AG95 initialization, optional D405
startup, MoveIt startup, and RViz startup remain ordered as they are today.

The `--allow-reduced` option is removed. A launcher whose READY contract requires
NORMAL cannot truthfully offer REDUCED as an executable state. Preflight still
reports the actual robot and safety modes, but guarded control proceeds only in
NORMAL.

## Testing strategy

### Automated tests

Tests will be written before production changes and will cover:

- runtime-policy validation and the 0.10-second default;
- launch argument propagation through every project-owned launch layer;
- AnyGrasp resource-file validation;
- pre-import OMP, MKL, and OpenBLAS environment application;
- PyTorch intra-op and inter-op application;
- nice-priority lowering without attempting to raise an already lower-priority
  process;
- STARTING to READY only when every required condition is valid;
- each READY-to-FAULT trigger independently;
- terminal FAULT behavior and first-reason preservation;
- startup timeout before READY;
- MoveIt being stopped before general shutdown after FAULT;
- removal of `--allow-reduced` from the CLI.

The existing tracer_bringup and anygrasp_ros test suites must remain green.

### Runtime verification

With the physical workspace clear and the existing human START confirmation
completed, run UR3, D405, AnyGrasp, MoveIt, and RViz together. Verify:

- `/ur/ur_hardware_interface/robot_receive_timeout` equals 0.10;
- the AnyGrasp process environment contains the configured OMP, MKL, and
  OpenBLAS limits;
- the AnyGrasp process nice value is 10 or lower priority if inherited;
- AnyGrasp reports effective PyTorch limits of 2 intra-op and 1 inter-op thread;
- large D405 clouds continue producing grasp results;
- the state machine reports READY and `robot_program_running` is true;
- no reverse-interface disconnect occurs during the observation window;
- deliberately stopping the robot control program produces FAULT with the
  expected reason;
- the MoveIt Execute endpoint is unavailable after FAULT;
- no automatic recovery or motion resumption occurs.

Stopping the robot program is performed only after confirming no trajectory is
executing. Restarting the physical control chain continues to require the
existing human safety confirmation.

## Acceptance criteria

The implementation is accepted when automated tests pass, project-owned launch
files visibly forward 0.10 without upstream changes, resource settings are
observable in the live AnyGrasp process, the combined workload remains connected
during the test window, and a deliberate program stop produces a terminal FAULT
that disables MoveIt and requires a complete restart.
