# Robottracer workspace agreements

## Purpose

This is the ROS Noetic workspace for the Tracer mobile base, UR3 arm, Livox
MID-360 localization/navigation, cameras, and the in-progress chess robot.

## Canonical environment

- Robot PC: `jt001@jt001-pc2`, Wi-Fi IP `172.20.10.7`
- UR controller: `192.168.131.3`
- Workspace: `/home/jt001/tracer_ws`
- OS/ROS: Ubuntu 20.04 / ROS Noetic

## Build and checks

```bash
cd /home/jt001/tracer_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Use `roslaunch --dump-params` for launch-file checks that must not start
hardware. Never power on, release brakes, play a UR program, or send motion as
part of an automated test.

## Main entry points

- Navigation: `roslaunch tracer_nav nav_all.launch`
- Calibrated chess UR startup: `roslaunch tracer_bringup chess_ur_startup.launch`
- Keyboard MoveIt Servo: branch `codex/ur-keyboard-teleop`, pending integration

`chess_ur_startup.launch` performs dashboard power/brake/program actions. Keep
the physical emergency stop accessible and verify the work area first.

## Source layout

- `src/tracer_nav`, `src/FAST_LIO_LOCALIZATION`: navigation/localization
- `src/FAST_LIO`, `src/mppi_local_planner`, `src/SA-MPPI`: vendored dependencies
- `src/tracer/tracer_ros`: Tracer base and bringup
- `src/urdf/tcurdf`: combined robot model
- `src/chess_robot`: experimental chess validation tools

## Current status

- Navigation and selected dependency packages build successfully.
- Calibrated UR launch parameter tests pass without starting hardware.
- Chess hand-eye calibration is not verified; recorded errors are about
  323-335 mm. Do not use it for autonomous motion.
- A full unfiltered build still fails in the existing DH gripper package because
  generated message headers are not ordered before driver compilation.
