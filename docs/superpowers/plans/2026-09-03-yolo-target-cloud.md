# YOLO bbox to D405 target cloud implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a timestamp-correct `/yolo_world/target_cloud` generated once per new valid YOLO detection from the matching D405 point cloud.

**Architecture:** A thin `yolo_world_ros` node caches raw clouds by a configurable time window and performs ROI, the existing AnyGrasp RANSAC, depth-to-color projection, and bbox selection only when a new valid detection arrives. Pure geometry/cache code is separated from ROS orchestration and tested first.

**Tech Stack:** ROS Noetic, rospy, sensor_msgs, tf2_ros, NumPy, Open3D through the existing `anygrasp` conda environment.

**Spec:** `docs/superpowers/specs/2026-09-03-yolo-target-cloud-design.md`

## Global Constraints

- Do not modify AnyGrasp's formal inference input, YOLO-World third-party files, MoveIt, gripper, or arm-control paths.
- Do not load the AnyGrasp SDK/model, torch, or an additional CUDA context.
- Raw point-cloud callbacks cache and prune only; expensive work is driven solely by each new valid YOLO detection.
- Cache duration defaults to 1.0 s; stamp matching defaults to 0.02 s; detection expiry defaults to 0.5 s, with independent semantics.
- Reuse the current AnyGrasp workspace ROI and RANSAC functions and parameters; do not add SOR or segmentation.
- Preserve real acquisition stamps and frames on every non-empty or empty published cloud.

---

### Task 1: Package contract and pure detection/cache API

**Files:**
- Create: `src/yolo_world_ros/package.xml`
- Create: `src/yolo_world_ros/CMakeLists.txt`
- Create: `src/yolo_world_ros/setup.py`
- Create: `src/yolo_world_ros/src/yolo_world_ros/__init__.py`
- Create: `src/yolo_world_ros/src/yolo_world_ros/target_cloud.py`
- Create: `src/yolo_world_ros/test/test_target_cloud.py`
- Create: `src/yolo_world_ros/test/test_package_manifest.py`

**Interfaces:**
- Produces: `Detection`, `TimedCloudCache(duration_sec)`, `parse_detection_json(payload) -> Detection`, `stamp_parts_to_ns(secs, nsecs) -> int`, `TimedCloudCache.add(stamp_ns, message)`, and `TimedCloudCache.nearest(stamp_ns, max_delta_sec) -> Optional[CloudMatch]`.
- Consumes: current YOLO JSON fields `header.stamp.secs/nsecs`, `bbox`, `class_name`, and `confidence`.

- [ ] Write `unittest` cases with literal stamps `10.000000000`, `10.019000000`, and `10.021000000`; assert that 19 ms matches, 21 ms does not, and adding stamp `11.100000000` prunes every sample older than `10.100000000` from a one-second cache.
- [ ] Run `/home/jt001/.conda/envs/anygrasp/bin/python -m unittest discover -s src/yolo_world_ros/test -p 'test_target_cloud.py' -v`; expect `ModuleNotFoundError: yolo_world_ros`.
- [ ] Implement frozen dataclasses, strict JSON validation, integer nanosecond conversion, and deque pruning. Reject nonpositive cache/delta values and invalid bbox ordering.
- [ ] Re-run the focused command; expect all Task 1 cases to pass.

### Task 2: Projection and point-cloud data transformations

**Files:**
- Modify: `src/yolo_world_ros/src/yolo_world_ros/target_cloud.py`
- Modify: `src/yolo_world_ros/test/test_target_cloud.py`

**Interfaces:**
- Produces: `CameraModel(width, height, k, d, distortion_model)`, `project_points(points_color, camera) -> (pixels, valid_mask)`, `select_bbox_points(source_points, colors, color_points, camera, bbox) -> TargetSelection`, and `parse_cloud_arrays(message) -> (points, packed_rgb)`.
- Consumes: NumPy arrays, `CameraInfo` values, ROS PointCloud2 fields, and target-from-source transforms.

- [ ] Add tests where `(X,Y,Z)=(1,2,2)`, `fx=fy=100`, `cx=320`, `cy=240` must project to `(370,340)`; use hand-calculated nonzero `k1,p1,p2` literals for distortion; include NaN, `Z=0`, negative Z, and boundary bbox points. Build a two-row binary cloud with padding and assert exact XYZ/RGB output order.
- [ ] Run the focused test command; expect `ImportError` for the new projection/parser names.
- [ ] Implement NumPy-vectorized `plumb_bob` equations, inclusive image/bbox masks, and field-offset/endian-aware parsing. Never derive a point index from `(u,v)`.
- [ ] Re-run focused tests, then run `/home/jt001/.conda/envs/anygrasp/bin/python -m unittest discover -s src/anygrasp_ros/test -p 'test_anygrasp_core.py' -v` and the analogous preprocessing test; expect zero failures.

### Task 3: Detection-driven ROS orchestration

**Files:**
- Create: `src/yolo_world_ros/scripts/yolo_target_cloud_node.py`
- Create: `src/yolo_world_ros/test/test_node_runtime.py`

**Interfaces:**
- Produces: `YoloTargetCloudNode`, with `_cloud_callback(message)`, `_detection_callback(message)`, `_stale_timer_callback(event)`, `_process_match(detection, match)`, and `_publish_empty(header)`.
- Consumes: Task 1/2 pure functions and `anygrasp_ros.core`/`preprocessing` pure ROI/RANSAC APIs.

- [ ] Write runtime tests by constructing the class with `__new__` and injecting a real `TimedCloudCache` plus small fakes. Make `_process_match` raise if the cloud callback invokes it; count calls from two duplicate detections; assert a 0.6 s-old detection is rejected even with zero stamp delta; assert a fresh detection with 21 ms cloud delta is independently rejected; and assert two timer ticks publish only one empty cloud.
- [ ] Run the runtime test; expect failure because `scripts/yolo_target_cloud_node.py` is absent.
- [ ] Implement the callbacks and matched pipeline. Use TF `workspace_frame <- cloud.header.frame_id` and `color_frame <- cloud.header.frame_id` at `cloud.header.stamp`; copy the source header into non-empty and empty outputs; require `plane_result.applied` when configured.
- [ ] Run focused runtime tests and the full `yolo_world_ros` test discovery; expect zero failures.

### Task 4: Configuration, launch, and operator documentation

**Files:**
- Create: `src/yolo_world_ros/config/yolo_target_cloud.yaml`
- Create: `src/yolo_world_ros/launch/yolo_target_cloud.launch`
- Create: `src/yolo_world_ros/README.md`
- Create: `src/yolo_world_ros/test/test_launch_config.py`
- Modify: `src/yolo_world_ros/CMakeLists.txt`

**Interfaces:**
- Produces: a launchable perception-only node using the AnyGrasp interpreter and the existing AnyGrasp ROI/RANSAC YAML.
- Consumes: all required topics and functions from Tasks 1–3.

- [ ] Write tests that parse YAML/XML and assert the exact topic/default values, two ordered `<rosparam>` loads (AnyGrasp first, fusion second), the anygrasp Python launch prefix, one perception node, and zero `<include>` elements.
- [ ] Run `python -m unittest ...test_launch_config -v`; expect missing-file failure.
- [ ] Add the YAML, launch file, executable/install declarations, and README commands for RViz fixed frame, display topic, RGB transformer, size, and decay.
- [ ] Run full package tests and `roslaunch --dump-params yolo_world_ros yolo_target_cloud.launch`; expect exit status 0 without hardware access.

### Task 5: Build and live robot verification

**Files:**
- No production-file changes unless a newly reproduced failure first receives a failing regression test.

**Interfaces:**
- Consumes: the complete thin package and the running D405, YOLO, TF, and current AnyGrasp pipeline.
- Produces: runtime evidence for topic contract, processing cadence, counts, alignment, stale clearing, and environment isolation.

- [ ] Build `anygrasp_ros;yolo_world_ros` in the existing worktree and run every `yolo_world_ros` test plus existing AnyGrasp preprocessing/core tests.
- [ ] Launch the new node and verify `/yolo_world/target_cloud` is `sensor_msgs/PointCloud2` with matched acquisition stamp, expected source frame, and `x y z rgb` fields.
- [ ] Compare processing log frequency with `/yolo_world/target_detection`; confirm the 30 Hz cache callback does not run RANSAC.
- [ ] Record raw/workspace/RANSAC/target point counts and detection/cloud stamp deltas over live detections.
- [ ] Add `/yolo_world/target_cloud` to RViz with fixed frame `ur_arm_base_link`, PointCloud2 style `Points`, RGB color transformer, point size about 0.003 m, decay time 0, and confirm static cuboid localization.
- [ ] Verify an expired/missing detection clears the cloud. Ask the operator for physical object/slow-arm motion if those cases cannot be performed safely from software.
- [ ] Stop immediately after target-cloud validation; do not connect the topic to AnyGrasp inference.
