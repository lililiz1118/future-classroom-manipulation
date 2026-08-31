# Future Classroom Manipulation workspace agreements

## Purpose

This repository owns the perception and manipulation subsystem of the Future
Classroom robot: sensor input, object recognition and pose estimation, UR3
MoveIt planning, AG95 execution, and operation feedback. The
`codex/ur3-headless-moveit` branch currently implements the UR3 + AG95
execution foundation. Navigation, teaching Q&A and global task orchestration
belong to other repositories and must not be claimed as this repository's work.

## Canonical environment

- Robot PC: `jt001@jt001-pc2`
- UR controller: `192.168.131.3`; robot-side ROS interface: `192.168.131.1`
- Main workspace: `/home/jt001/tracer_ws`
- Development worktree: `/home/jt001/tracer_ws/.worktrees/ur3-headless-moveit`
- GitHub: `lililiz1118/future-classroom-manipulation`
- OS/ROS: Ubuntu 20.04 / ROS Noetic

## Build and checks

```bash
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES='anygrasp_ros;dh_gripper_driver;dh_gripper_msgs;moveit_config;realsense2_camera;tcurdf;tracer_bringup;ur_dashboard_msgs;ur_description;ur_msgs;ur_robot_driver'
source devel/setup.bash
test -f devel/lib/librealsense2_camera.so
```

Use `roslaunch --dump-params` for launch checks that must not start hardware.
Automated tests must never connect to the UR Dashboard, power on, release
brakes, change hardware speed or send motion.

## Main entry points

- UR3 + AG95 read-only preflight: `./ur3_moveit_headless.sh --preflight-only`
- Guarded UR3 + AG95 startup: `./ur3_moveit_headless.sh`
- Independent AnyGrasp perception: `roslaunch anygrasp_ros anygrasp_d405.launch`
- Navigation interface: `roslaunch tracer_nav nav_all.launch` (team-owned,
  outside this branch's implementation scope)
- Detailed arm procedure:
  `src/tracer/tracer_ros/tracer_bringup/scripts/README_UR3_HEADLESS_MOVEIT.md`

The guarded startup may power on, release brakes and initialize the gripper
only after exact `START` confirmation. Keep the physical emergency stop
accessible and verify the work area first. It never executes a trajectory
automatically or clears safety faults. A post-READY control-chain fault is
terminal: managed MoveIt execution is removed and a full restart is required.

## Source layout and status

- `src/tracer/tracer_ros/tracer_bringup`: current UR3/AG95 startup feature
- `src/anygrasp_ros`: independently launched AnyGrasp D405 perception and its
  centralized CPU resource policy
- `src/tracer_nav`, `src/FAST_LIO_LOCALIZATION`: integration copies maintained
  elsewhere; do not modify them without explicit cross-repository scope
- `src/urdf/tcurdf`: combined robot model
- `src/chess_robot`: legacy experiment; its unverified hand-eye calibration
  (recorded error about 323-335 mm) must not be used for autonomous motion

New perception, pose-estimation and grasp-orchestration work should use focused
ROS packages with explicit message, service or action contracts for the team
integration repository. Do not place new product logic in vendored packages.
