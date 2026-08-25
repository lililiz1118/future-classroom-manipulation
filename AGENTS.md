# Robottracer workspace agreements

## Purpose

This is the ROS Noetic workspace for the Future Classroom embodied teaching
robot. The target system combines trusted teaching Q&A, guided navigation and
fixed-point pick-and-place. The `codex/ur3-headless-moveit` branch implements
the UR3 + AG95 execution foundation only; Tracer navigation is maintained by
other team members and is not implemented by this branch.

## Canonical environment

- Robot PC: `jt001@jt001-pc2`
- UR controller: `192.168.131.3`; robot-side ROS interface: `192.168.131.1`
- Main workspace: `/home/jt001/tracer_ws`
- Development worktree: `/home/jt001/tracer_ws/.worktrees/ur3-headless-moveit`
- OS/ROS: Ubuntu 20.04 / ROS Noetic

## Build and checks

```bash
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Use `roslaunch --dump-params` for launch checks that must not start hardware.
Automated tests must never connect to the UR Dashboard, power on, release
brakes, change hardware speed or send motion.

## Main entry points

- UR3 + AG95 read-only preflight: `./ur3_moveit_headless.sh --preflight-only`
- Guarded UR3 + AG95 startup: `./ur3_moveit_headless.sh`
- Navigation interface: `roslaunch tracer_nav nav_all.launch` (team-owned,
  outside this branch's implementation scope)
- Detailed arm procedure:
  `src/tracer/tracer_ros/tracer_bringup/scripts/README_UR3_HEADLESS_MOVEIT.md`

The guarded startup may power on, release brakes and initialize the gripper
only after exact `START` confirmation. Keep the physical emergency stop
accessible and verify the work area first. It never executes a trajectory
automatically or clears safety faults.

## Source layout and status

- `src/tracer/tracer_ros/tracer_bringup`: current UR3/AG95 startup feature
- `src/tracer_nav`, `src/FAST_LIO_LOCALIZATION`: existing navigation/localization;
  do not claim or modify them as part of this branch without explicit scope
- `src/urdf/tcurdf`: combined robot model
- `src/chess_robot`: legacy experiment, not the project identity

The current branch contains the guarded UR3 headless MoveIt path and AG95
integration. Chess hand-eye calibration remains unverified (recorded error
about 323-335 mm) and must not be used for autonomous motion.
