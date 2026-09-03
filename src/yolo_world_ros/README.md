# yolo_world_ros

Thin ROS Noetic integration that projects the current YOLO-World bbox onto a
timestamp-matched D405 point cloud. It publishes only the debug topic
`/yolo_world/target_cloud`; it does not load AnyGrasp or change its input.

The raw cloud callback only retains messages for the configured one-second time
window. Each new valid detection triggers one workspace ROI, one existing
AnyGrasp RANSAC pass, the depth-optical to color-optical TF transform, raw color
camera projection, and bbox selection.

## Start

```bash
cd /home/jt001/tracer_ws/.worktrees/ur3-headless-moveit
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch yolo_world_ros yolo_target_cloud.launch
```

The launch uses `/home/jt001/.conda/envs/anygrasp/bin/python` for NumPy/Open3D,
but imports only `anygrasp_ros.core` and `anygrasp_ros.preprocessing`. It does
not import torch, GSNet, the AnyGrasp SDK, or create a CUDA context.

## RViz

Set RViz `Fixed Frame` to `ur_arm_base_link`, then add a `PointCloud2` display:

- Topic: `/yolo_world/target_cloud`
- Style: `Points`
- Color Transformer: `RGB8`
- Size (m): approximately `0.003`
- Decay Time: `0`

The published XYZ remains in the source point-cloud frame, currently
`d405_depth_optical_frame`, with the matched source acquisition timestamp. TF
lets RViz display it in `ur_arm_base_link`. When the detection expires, one
empty cloud with a real D405 cloud header clears the old target.

## Timing parameters

- `cloud_cache_duration_sec=1.0`: retained acquisition-time window.
- `max_stamp_delta_sec=0.02`: same-acquisition detection/cloud match limit.
- `max_detection_age_sec=0.5`: independent operational detection expiry.

The original acquisition stamp is checked against this age before processing.
After a successful target publish, the same bounded interval acts as an output
watchdog; a single dropped same-stamp cloud is skipped without clearing a still
fresh target, while a stopped detection stream clears it once.

No SOR, segmentation, AnyGrasp inference input, or robot-control topic belongs
to this package.
