# YOLO bbox to D405 target cloud design

## Scope

Build a thin ROS Noetic package that converts each new valid YOLO-World detection into a debug `sensor_msgs/PointCloud2` containing the corresponding 3D object points. This phase ends at `/yolo_world/target_cloud`; it does not change AnyGrasp inference input, load the AnyGrasp SDK/model, initialize CUDA, control the arm, or execute grasps.

## Existing pipeline decision

`/anygrasp/object_cloud` already contains base-frame workspace ROI points after the current table-plane RANSAC and before SOR. Its header preserves the source point-cloud acquisition stamp and uses `ur_arm_base_link`. However, it is published by the full AnyGrasp inference callback at only about 0.20 Hz. It therefore cannot be paired within 20 ms with 5 Hz YOLO detections or follow a moving object at the required rate.

The new node will consequently subscribe to `/d405/depth/color/points`, cache messages only, and reuse the lightweight `anygrasp_ros.core.select_workspace`, `transform_points`, and `anygrasp_ros.preprocessing.remove_table_plane` functions. Import inspection confirmed that these modules load NumPy/Open3D only: they do not load torch, GSNet, the AnyGrasp SDK/model, or initialize CUDA.

## Runtime architecture

The package is `yolo_world_ros`, but the node runs with `/home/jt001/.conda/envs/anygrasp/bin/python` to obtain Open3D. The YOLO process remains in the independent `yolox-world` environment and communicates only through ROS topics.

The raw point-cloud callback performs only three bounded operations: append the original ROS message to a deque, prune entries older than `cloud_cache_duration_sec` relative to the newest cloud stamp, and update the newest real cloud header used to clear RViz. It never parses the binary cloud or runs ROI, TF transforms, RANSAC, or projection.

Each previously unseen valid detection stamp triggers at most one processing pass. The detection callback rejects malformed, duplicate, out-of-order, or stale detections, selects the nearest cached cloud by acquisition stamp, and rejects a match beyond `max_stamp_delta_sec`. Only then does it parse and filter that one cloud. Processing therefore follows YOLO's current approximate 5 Hz rate, not the D405 point-cloud rate.

A lightweight timer checks target activity. Incoming detections are first rejected when their original acquisition stamp is older than `max_detection_age_sec`. While an accepted detection is undergoing geometry, the timer does not clear an existing target. Successful publication refreshes the output watchdog so the unavoidable RANSAC time does not immediately erase the result. If target activity then stops for `max_detection_age_sec`, the timer publishes one empty target cloud with the newest real cached cloud header. It does not rerun geometry.

## Time semantics

- `cloud_cache_duration_sec` defaults to `1.0`. It is a time window, not a frame count, and covers measured YOLO latency with margin.
- `max_stamp_delta_sec` defaults to `0.02`. It answers only whether the detection and cloud belong to the same acquisition instant.
- `max_detection_age_sec` defaults to `0.5`. It independently prevents a delayed detection from entering processing and bounds how long a published cloud remains after target activity stops.
- Matching and duplicate checks use integer nanoseconds to avoid loss of precision from floating-point epoch seconds.
- Published target clouds inherit the matched source cloud stamp. `rospy.Time.now()` is never substituted for acquisition time.

## Geometry and data flow

For a matched pair:

1. Parse `x`, `y`, `z`, and packed `rgb` from the non-organized source cloud while preserving row/field layout.
2. Query TF `ur_arm_base_link <- source_cloud_frame` at the cloud stamp.
3. Transform all finite source points to the base frame and call the existing `select_workspace` with the current bounds from `anygrasp_d405.yaml`.
4. Call the existing `remove_table_plane` with the current RANSAC configuration. With `require_ransac_success=true`, a fallback or rejected plane produces an empty debug cloud rather than reintroducing the table.
5. Query TF `d405_color_optical_frame <- source_cloud_frame` at the same cloud stamp. For the current source this is the RealSense `d405_color_optical_frame <- d405_depth_optical_frame` depth-to-color extrinsic.
6. Transform the RANSAC-surviving source-frame XYZ into the color optical frame.
7. Keep finite points with `Z > 0`, project through the color `CameraInfo` `K` matrix, and apply its `plumb_bob` distortion coefficients because YOLO consumes `/d405/color/image_raw`. Zero distortion reduces to the ordinary pinhole equations.
8. Keep projected points inside the image and the inclusive YOLO bbox.
9. Publish the selected original source-frame XYZ/RGB as an unorganized `PointCloud2` on `/yolo_world/target_cloud`.

The output header is copied from the matched point cloud, so the current expected frame is `d405_depth_optical_frame`. Empty clearing clouds also use a real cached cloud header.

## ROS interfaces and parameters

Subscriptions:

- `/d405/depth/color/points` (`sensor_msgs/PointCloud2`)
- `/d405/color/camera_info` (`sensor_msgs/CameraInfo`)
- `/yolo_world/target_detection` (`std_msgs/String`, current JSON contract)

Publication:

- `/yolo_world/target_cloud` (`sensor_msgs/PointCloud2`, fields `x y z rgb`)

Fusion-specific defaults:

- `cloud_topic: /d405/depth/color/points`
- `camera_info_topic: /d405/color/camera_info`
- `detection_topic: /yolo_world/target_detection`
- `target_cloud_topic: /yolo_world/target_cloud`
- `color_frame: d405_color_optical_frame`
- `cloud_cache_duration_sec: 1.0`
- `max_stamp_delta_sec: 0.02`
- `max_detection_age_sec: 0.5`
- `tf_timeout_sec: 0.2`
- `stale_check_period_sec: 0.1`
- `require_ransac_success: true`
- `log_throttle_sec: 1.0`

The launch file first loads the existing `anygrasp_ros/config/anygrasp_d405.yaml` into the new node's private namespace, then loads the fusion-specific YAML. This keeps workspace and RANSAC parameters single-sourced without starting or modifying AnyGrasp.

## Observability and failure behavior

Rate-limited logs report class, confidence, bbox, cloud stamp, detection stamp, absolute time delta, raw point count, workspace count, RANSAC output count and status, target count, and processing time. Malformed detections, missing camera calibration, cache misses, excessive stamp deltas, stale detections, TF failures, unsupported clouds, and RANSAC fallback are skipped safely and logged with throttling.

The node never republishes a prior non-empty target as a new result. A single acquisition-cache miss is skipped and the bounded watchdog remains responsible for expiry, which avoids RViz flicker when ROS drops one 30 Hz cloud. It publishes an empty cloud once when target activity stops or when a matched detection proves that no valid target points remain, ensuring RViz does not retain a ghost target indefinitely.

## Test and acceptance plan

Unit tests cover detection parsing, time-window cache pruning, nearest-stamp matching, independent age and delta rejection, raw `plumb_bob` projection, invalid/invisible points, bbox masking, XYZ/RGB alignment, binary PointCloud2 parsing, output header preservation, duplicate detection suppression, detection-driven processing, and one-shot stale clearing.

Integration verification will build the thin package, run its tests, confirm the process has no torch/CUDA/AnyGrasp SDK modules, start the node alongside the existing D405, YOLO and AnyGrasp system, inspect topic type/rate/header/fields, collect point-count logs, and visualize `/yolo_world/target_cloud` in RViz. Acceptance requires the cloud to cover the detected cuboid, exclude most table/gripper/background points, follow motion, and disappear when detection expires. Physical motion cases that cannot be actuated safely by software require the operator to move the object or arm while observing RViz.
