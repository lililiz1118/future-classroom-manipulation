# AnyGrasp D405 ROS Integration Design

## Scope

Add a perception-only ROS Noetic package that consumes the live
`/d405/depth/color/points` stream, runs the already validated AnyGrasp SDK,
keeps the results in the input cloud frame, and publishes grasp poses and RViz
markers. The package must not depend on or invoke MoveIt, UR controllers, the
UR dashboard, trajectory controllers, or gripper control.

## Verified Runtime Facts

- Development worktree: `/home/jt001/tracer_ws/.worktrees/ur3-headless-moveit`
  on `codex/ur3-headless-moveit`; it was clean before this change.
- Point cloud type: `sensor_msgs/PointCloud2`, approximately 30 Hz.
- Point cloud frame: `d405_depth_optical_frame`.
- Point fields: float32 `x`, `y`, `z`, and packed-float32 `rgb`.
- Point cloud size: 258,521 points per frame, currently marked dense with no
  NaN/Inf, but it contains finite invalid ranges up to `z=65.535 m`.
- The initial camera-frame ROI contains about 40,789 points.
- TF between `d405_depth_optical_frame` and `d405_color_optical_frame` exists.
- Installed SDK signature:
  `get_grasp(points, colors, lims=None, voxel_size=0.005,
  apply_object_mask=True, dense_grasp=False, collision_detection=True)`.
- Installed GraspNet API defines rotation-matrix column 0 as approach and
  column 1 as the opening direction.

## Architecture

Create one ROS package named `anygrasp_ros`. A small pure-NumPy module owns
packed-RGB decoding, finite/ROI filtering, rotation validation, and quaternion
conversion. The executable ROS node owns PointCloud2 conversion, a
latest-message-only subscriber, model initialization/inference, ROS logging,
and publishers. Keeping numerical transformations separate makes the critical
frame/orientation logic testable without importing CUDA, AnyGrasp, or ROS.

The node uses the verified Conda interpreter directly. Before importing ROS it
appends `/usr/lib/python3/dist-packages` to `sys.path`, preserving the Conda
site-packages and NumPy ahead of Ubuntu's Python 3.8 packages. The launch file
does not include any robot-control launch file.

## Data Flow

1. Subscribe to `/d405/depth/color/points` with `queue_size=1`.
2. The callback stores only the newest message under a lock.
3. A configurable 1 Hz loop atomically takes the newest unprocessed message.
4. Decode float32 XYZ and packed RGB, reject non-finite points, then apply the
   camera-frame workspace limits from YAML.
5. Publish the exact filtered input as `/anygrasp/input_cloud`.
6. Call AnyGrasp with the demo-compatible configuration: 0.1 m maximum width,
   0.03 m gripper height, top-down grasps enabled, object mask enabled, dense
   grasp disabled, 0.005 m voxel size, and collision detection enabled.
7. Apply `nms()`, `sort_by_score()`, and truncate to `top_n`.
8. Publish the best candidate as `geometry_msgs/PoseStamped` and all retained
   candidates as `visualization_msgs/MarkerArray`, with the original cloud
   timestamp and `d405_depth_optical_frame` (or the actual incoming frame).

## Orientation and Visualization

The AnyGrasp 3x3 rotation matrix is treated as the grasp frame expressed in
the camera frame: column 0 is approach, column 1 is opening, and column 2 is the
remaining right-handed axis. It is projected to the nearest proper rotation
matrix before conversion to ROS quaternion order `x, y, z, w`.

Each grasp marker includes a center sphere, an approach arrow, and an opening
line. The best grasp is larger and uses a distinct color. Every published
MarkerArray starts with `DELETEALL`; no-result and failed-frame paths also
publish `DELETEALL` so stale candidates do not remain in RViz.

## Failure Handling

- Missing cloud, empty input, insufficient workspace points, and no grasps use
  throttled warnings and continue with later frames.
- Ordinary single-frame conversion or inference failures are logged and the
  next frame is attempted.
- SDK import, checkpoint load, model initialization, MinkowskiEngine failure,
  and CUDA out-of-memory are fatal and terminate the node visibly.
- No result is silently relabeled as `base_link`; all output stays in the
  actual input frame.

## Verification

Use strict TDD for pure transformations, then run the complete unit suite,
`catkin_make`, a Conda/ROS import check, a perception-only launch, and the
requested `rosnode`/`rostopic` inspections. Runtime verification must record
the actual frame, inference time, grasp counts, and best score without claiming
success for anything not observed.
