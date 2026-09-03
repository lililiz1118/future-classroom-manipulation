#!/home/jt001/.conda/envs/anygrasp/bin/python
"""在最新一帧 D405 点云上运行 AnyGrasp，并发布相机坐标系下的抓姿。

本文件是 ROS 编排层：读取参数、接收点云、调用纯计算函数和 AnyGrasp SDK，
最后发布抓姿。点云裁剪等可独立测试的数值逻辑位于 ``core.py``。
"""

import os
import sys
import threading
import time
from types import SimpleNamespace


# AnyGrasp 使用 Conda Python，但 ROS Noetic 的 Python 包安装在系统目录。
# 先临时加入该目录完成 ROS 消息导入，随后再移除，避免污染 SDK 的导入环境。
SYSTEM_DIST_PACKAGES = "/usr/lib/python3/dist-packages"
if SYSTEM_DIST_PACKAGES not in sys.path:
    sys.path.append(SYSTEM_DIST_PACKAGES)

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE_SOURCE = os.path.join(PACKAGE_ROOT, "src")
if PACKAGE_SOURCE not in sys.path:
    sys.path.insert(0, PACKAGE_SOURCE)

from anygrasp_ros.runtime_resources import (
    configure_torch,
    format_process_report,
    format_torch_report,
    initialize_resource_policy,
)


# 必须在导入 NumPy/PyTorch 之前应用 CPU 线程数和进程 nice 策略。
RESOURCE_POLICY, PROCESS_RESOURCE_REPORT = initialize_resource_policy(
    os.path.join(PACKAGE_ROOT, "config", "anygrasp_resources.yaml")
)

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import quaternion_matrix, quaternion_multiply
from visualization_msgs.msg import Marker, MarkerArray

if SYSTEM_DIST_PACKAGES in sys.path:
    sys.path.remove(SYSTEM_DIST_PACKAGES)

from anygrasp_ros.core import (
    dynamic_point_bounds,
    grasp_axes,
    rotation_matrix_to_quaternion,
    select_workspace,
    transform_points,
)

from anygrasp_ros.preprocessing import (
    RansacPlaneConfig,
    remove_statistical_outliers,
    remove_table_plane,
)


EXPECTED_PYTHON = "/home/jt001/.conda/envs/anygrasp/bin/python"


def is_fatal_backend_error(error):
    """判断异常是否已破坏推理后端，导致后续帧也不再可信。"""
    description = "%s.%s: %s" % (
        type(error).__module__,
        type(error).__name__,
        error,
    )
    normalized = description.lower()
    return (
        "out of memory" in normalized
        or "minkowskiengine" in normalized
        or "minkowski engine" in normalized
    )


class LatestMessageBuffer:
    """只保存最新一帧 ROS 消息，防止推理较慢时积压过期点云。

    回调线程通过 ``update`` 写入；主推理循环通过 ``take_latest`` 读取。
    sequence 用来判断这一帧是否已经处理过。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._message = None
        self._sequence = 0

    def update(self, message):
        """由 ROS 订阅回调调用，用新消息直接替换旧消息。"""
        with self._lock:
            self._message = message
            self._sequence += 1

    def take_latest(self, last_sequence):
        """有新帧时返回 ``(消息, 序号)``，否则返回 ``(None, 原序号)``。"""
        with self._lock:
            if self._message is None or self._sequence == last_sequence:
                return None, last_sequence
            return self._message, self._sequence


class AnyGraspAdapter:
    """隔离第三方 AnyGrasp SDK 的加载和调用细节。

    ROS 主节点只依赖本类的 ``infer``，不需要知道 checkpoint、工作目录和
    ``gsnet.AnyGrasp`` 的具体初始化方式。
    """

    def __init__(self, sdk_dir, checkpoint_path, config):
        # SDK 会按相对路径加载内部资源，因此导入和加载模型期间切换到 SDK 目录。
        if not os.path.isdir(sdk_dir):
            raise FileNotFoundError("AnyGrasp SDK directory not found: " + sdk_dir)
        if not os.path.isfile(checkpoint_path):
            raise FileNotFoundError("AnyGrasp checkpoint not found: " + checkpoint_path)
        if sdk_dir not in sys.path:
            sys.path.insert(0, sdk_dir)
        original_directory = os.getcwd()
        try:
            os.chdir(sdk_dir)
            import torch

            torch_report = configure_torch(torch, RESOURCE_POLICY)
            rospy.loginfo(format_torch_report(torch_report))
            from gsnet import AnyGrasp

            sdk_config = SimpleNamespace(
                checkpoint_path=checkpoint_path,
                max_gripper_width=float(config["max_gripper_width"]),
                gripper_height=float(config["gripper_height"]),
                top_down_grasp=bool(config["top_down_grasp"]),
                debug=bool(config["debug"]),
            )
            sdk_config.max_gripper_width = max(
                0.0, min(0.1, sdk_config.max_gripper_width)
            )
            # load_net() 在节点启动阶段一次性加载网络权重；每帧不会重复加载。
            self._model = AnyGrasp(sdk_config)
            self._model.load_net()
        finally:
            os.chdir(original_directory)
        self._voxel_size = float(config["voxel_size"])
        self._apply_object_mask = bool(config["apply_object_mask"])
        self._dense_grasp = bool(config["dense_grasp"])
        self._collision_detection = bool(config["collision_detection"])

    def infer(self, points, colors, camera_bounds):
        """把 ROI 内的 Nx3 点和 Nx3 颜色交给 SDK，返回候选抓姿集合。"""
        grasps, _ = self._model.get_grasp(
            points,
            colors,
            lims=list(camera_bounds),
            voxel_size=self._voxel_size,
            apply_object_mask=self._apply_object_mask,
            dense_grasp=self._dense_grasp,
            collision_detection=self._collision_detection,
        )
        return grasps


class AnyGraspD405Node:
    """连接 D405 点云、AnyGrasp 推理以及 ROS 输出话题的主节点。"""

    def __init__(self):
        """读取 ROS 参数、加载模型，并建立订阅器和发布器。"""
        rospy.init_node("anygrasp_d405_node")
        rospy.loginfo(format_process_report(PROCESS_RESOURCE_REPORT))
        expected_python = rospy.get_param("~python_executable", EXPECTED_PYTHON)
        if os.path.realpath(sys.executable) != os.path.realpath(expected_python):
            raise RuntimeError(
                "AnyGrasp node must use %s, got %s" % (expected_python, sys.executable)
            )

        # 这些 ``~参数`` 由 launch 文件加载 anygrasp_d405.yaml 后提供。
        self._cloud_topic = rospy.get_param("~cloud_topic", "/d405/depth/color/points")
        self._best_topic = rospy.get_param("~best_grasp_topic", "/anygrasp/best_grasp")
        self._best_base_topic = rospy.get_param(
            "~best_grasp_base_topic", "/anygrasp/best_grasp_base"
        )
        self._marker_topic = rospy.get_param("~marker_topic", "/anygrasp/grasp_markers")
        self._input_cloud_topic = rospy.get_param("~input_cloud_topic", "/anygrasp/input_cloud")
        self._workspace_cloud_topic = rospy.get_param(
            "~workspace_cloud_topic", "/anygrasp/workspace_cloud"
        )
        self._object_cloud_topic = rospy.get_param(
            "~object_cloud_topic", "/anygrasp/object_cloud"
        )
        self._top_n = int(rospy.get_param("~top_n", 10))
        self._inference_rate = float(rospy.get_param("~inference_rate", 1.0))
        self._min_workspace_points = int(rospy.get_param("~min_workspace_points", 1000))
        self._publish_input = bool(rospy.get_param("~publish_input_cloud", False))
        if self._top_n <= 0 or self._inference_rate <= 0.0 or self._min_workspace_points <= 0:
            raise ValueError("top_n, inference_rate, and min_workspace_points must be positive")

        # ROI 边界始终属于 workspace.frame_id，而不是输入相机 frame。
        workspace = rospy.get_param("~workspace")
        self._workspace_frame = str(workspace["frame_id"]).strip()
        self._workspace_bounds = (
            float(workspace["x_min"]),
            float(workspace["x_max"]),
            float(workspace["y_min"]),
            float(workspace["y_max"]),
            float(workspace["z_min"]),
            float(workspace["z_max"]),
        )
        self._dynamic_lims_margin = float(
            rospy.get_param("~dynamic_lims_margin", 0.01)
        )
        self._tf_timeout = rospy.Duration.from_sec(
            float(rospy.get_param("~tf_timeout", 0.2))
        )
        if not self._workspace_frame:
            raise ValueError("workspace frame_id must not be empty")
        x_min, x_max, y_min, y_max, z_min, z_max = self._workspace_bounds
        if x_min > x_max or y_min > y_max or z_min > z_max:
            raise ValueError("workspace minimums must not exceed maximums")
        if self._dynamic_lims_margin <= 0.0:
            raise ValueError("dynamic_lims_margin must be positive")
        if self._tf_timeout.to_sec() <= 0.0:
            raise ValueError("tf_timeout must be positive")
        outlier_config = rospy.get_param(
            "~statistical_outlier_filter",
            {},
        )
        self._outlier_filter_enabled = bool(
            outlier_config.get("enabled", False)
        )
        self._outlier_nb_neighbors = int(
            outlier_config.get("nb_neighbors", 20)
        )
        self._outlier_std_ratio = float(
            outlier_config.get("std_ratio", 2.0)
        )

        if self._outlier_nb_neighbors < 1:
            raise ValueError("outlier nb_neighbors must be at least 1")
        if self._outlier_std_ratio <= 0.0:
            raise ValueError("outlier std_ratio must be positive")

        # ur_arm_base_link follows REP-103, so its Z axis is vertical.  The
        # normal-angle check itself handles the plane normal's +/- ambiguity.
        self._ransac_config = RansacPlaneConfig(
            enabled=bool(rospy.get_param("~ransac_enabled", True)),
            distance_threshold=float(
                rospy.get_param("~ransac_distance_threshold", 0.008)
            ),
            ransac_n=int(rospy.get_param("~ransac_n", 3)),
            num_iterations=int(rospy.get_param("~ransac_num_iterations", 1000)),
            min_points=int(rospy.get_param("~ransac_min_points", 1000)),
            max_normal_angle_deg=float(
                rospy.get_param("~ransac_max_normal_angle_deg", 15.0)
            ),
            table_height_min=float(
                rospy.get_param("~ransac_table_height_min", 0.20)
            ),
            table_height_max=float(
                rospy.get_param("~ransac_table_height_max", 0.28)
            ),
            min_inliers=int(rospy.get_param("~ransac_min_inliers", 500)),
            min_inlier_ratio=float(
                rospy.get_param("~ransac_min_inlier_ratio", 0.20)
            ),
            min_object_points=int(
                rospy.get_param("~ransac_min_object_points", 1000)
            ),
        )

        # 这组参数会原样进入 AnyGraspAdapter，最终控制 SDK 的抓姿生成方式。
        adapter_config = {
            "max_gripper_width": rospy.get_param("~max_gripper_width", 0.1),
            "gripper_height": rospy.get_param("~gripper_height", 0.03),
            "top_down_grasp": rospy.get_param("~top_down_grasp", True),
            "debug": rospy.get_param("~debug", True),
            "voxel_size": rospy.get_param("~voxel_size", 0.005),
            "apply_object_mask": rospy.get_param("~apply_object_mask", True),
            "dense_grasp": rospy.get_param("~dense_grasp", False),
            "collision_detection": rospy.get_param("~collision_detection", True),
        }
        sdk_dir = rospy.get_param(
            "~sdk_dir", "/home/jt001/anygrasp_sdk/grasp_detection"
        )
        checkpoint_path = rospy.get_param(
            "~checkpoint_path",
            "/home/jt001/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar",
        )

        rospy.loginfo("[AnyGrasp] Python: %s", sys.executable)
        rospy.loginfo("[AnyGrasp] workspace frame: %s", self._workspace_frame)
        rospy.loginfo(
            "[AnyGrasp] workspace x: [%.6f, %.6f]",
            self._workspace_bounds[0],
            self._workspace_bounds[1],
        )
        rospy.loginfo(
            "[AnyGrasp] workspace y: [%.6f, %.6f]",
            self._workspace_bounds[2],
            self._workspace_bounds[3],
        )
        rospy.loginfo(
            "[AnyGrasp] workspace z: [%.6f, %.6f]",
            self._workspace_bounds[4],
            self._workspace_bounds[5],
        )
        rospy.loginfo(
            "[AnyGrasp] RANSAC table filter | enabled: %s | distance: %.4f m | "
            "normal angle: %.1f deg | temporary table height: [%.3f, %.3f] m",
            self._ransac_config.enabled,
            self._ransac_config.distance_threshold,
            self._ransac_config.max_normal_angle_deg,
            self._ransac_config.table_height_min,
            self._ransac_config.table_height_max,
        )
        rospy.loginfo("[AnyGrasp] loading model from %s", checkpoint_path)
        self._adapter = AnyGraspAdapter(sdk_dir, checkpoint_path, adapter_config)
        rospy.loginfo("[AnyGrasp] model loaded")

        # best_grasp 发布得分最高的抓姿；markers 只用于 RViz 可视化。
        self._best_publisher = rospy.Publisher(
            self._best_topic, PoseStamped, queue_size=1
        )
        self._best_base_publisher = rospy.Publisher(
            self._best_base_topic, PoseStamped, queue_size=1
        )
        self._marker_publisher = rospy.Publisher(
            self._marker_topic, MarkerArray, queue_size=1
        )
        self._input_publisher = rospy.Publisher(
            self._input_cloud_topic, PointCloud2, queue_size=1
        )
        self._workspace_publisher = rospy.Publisher(
            self._workspace_cloud_topic, PointCloud2, queue_size=1
        )
        self._object_publisher = rospy.Publisher(
            self._object_cloud_topic, PointCloud2, queue_size=1
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        # ROS 回调只替换最新消息，耗时的解析和推理全部留在主循环中执行。
        self._latest = LatestMessageBuffer()
        self._subscriber = rospy.Subscriber(
            self._cloud_topic,
            PointCloud2,
            self._latest.update,
            queue_size=1,
            buff_size=32 * 1024 * 1024,
        )
        self._marker_lifetime = rospy.Duration.from_sec(
            max(2.5 / self._inference_rate, 1.0)
        )

    @staticmethod
    def _header_from_cloud(message):
        """保留输入点云的时间戳和坐标系，供所有输出消息复用。"""
        return Header(stamp=message.header.stamp, frame_id=message.header.frame_id)

    @staticmethod
    def _cloud_to_arrays(message):
        """将 PointCloud2 的二进制缓冲区解析为 XYZ 和打包 RGB 数组。

        返回：
        - points: ``float32``、形状 ``(N, 3)``；
        - packed_rgb: ``uint32``、形状 ``(N,)``。

        这里只解析完整输入帧；ROI 裁剪发生在后面的 ``select_workspace``。
        """
        fields = {field.name: field for field in message.fields}
        missing = [name for name in ("x", "y", "z", "rgb") if name not in fields]
        if missing:
            raise ValueError("PointCloud2 missing fields: " + ", ".join(missing))
        rgb_datatype = fields["rgb"].datatype
        if rgb_datatype not in (PointField.FLOAT32, PointField.UINT32):
            raise ValueError("PointCloud2 rgb must be packed FLOAT32 or UINT32")
        for name in ("x", "y", "z"):
            if fields[name].datatype != PointField.FLOAT32 or fields[name].count != 1:
                raise ValueError("PointCloud2 %s must be scalar FLOAT32" % name)
        if fields["rgb"].count != 1:
            raise ValueError("PointCloud2 rgb must be scalar packed data")

        width = int(message.width)
        height = int(message.height)
        point_step = int(message.point_step)
        row_step = int(message.row_step)
        if width == 0 or height == 0:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty(0, dtype=np.uint32),
            )
        if point_step <= 0 or row_step < width * point_step:
            raise ValueError("PointCloud2 has invalid point_step or row_step")
        required_bytes = (height - 1) * row_step + width * point_step
        if len(message.data) < required_bytes:
            raise ValueError("PointCloud2 data is shorter than its dimensions require")
        for name in ("x", "y", "z", "rgb"):
            if fields[name].offset < 0 or fields[name].offset + 4 > point_step:
                raise ValueError("PointCloud2 %s field exceeds point_step" % name)

        # 使用带 offset/stride 的结构化视图，兼容有行填充的 organized cloud，
        # 也避免逐点调用 read_points 带来的大量 Python 循环开销。
        byte_order = ">" if message.is_bigendian else "<"
        rgb_format = "f4" if rgb_datatype == PointField.FLOAT32 else "u4"
        record_dtype = np.dtype(
            {
                "names": ("x", "y", "z", "rgb"),
                "formats": (
                    byte_order + "f4",
                    byte_order + "f4",
                    byte_order + "f4",
                    byte_order + rgb_format,
                ),
                "offsets": tuple(fields[name].offset for name in ("x", "y", "z", "rgb")),
                "itemsize": point_step,
            }
        )
        records = np.ndarray(
            shape=(height, width),
            dtype=record_dtype,
            buffer=message.data,
            strides=(row_step, point_step),
        )
        points = np.column_stack(
            (
                records["x"].reshape(-1),
                records["y"].reshape(-1),
                records["z"].reshape(-1),
            )
        ).astype(np.float32, copy=False)
        rgb_values = records["rgb"].reshape(-1)
        if rgb_datatype == PointField.FLOAT32:
            rgb_values = rgb_values.view(np.dtype(byte_order + "u4"))
        packed_rgb = rgb_values.astype(np.uint32, copy=False)
        return points, packed_rgb

    @staticmethod
    def _pack_colors(colors):
        """将 ``[0, 1]`` 的 Nx3 RGB 重新打包成 ROS PointCloud2 RGB 字段。"""
        channels = np.clip(np.rint(colors * 255.0), 0.0, 255.0).astype(np.uint32)
        packed = (channels[:, 0] << 16) | (channels[:, 1] << 8) | channels[:, 2]
        return packed.astype(np.uint32, copy=False).view(np.float32)

    def _publish_input_cloud(self, filtered, header):
        """按需发布 SOR 后、相机坐标系下的 AnyGrasp 输入点云。"""
        if not self._publish_input:
            return
        self._input_publisher.publish(
            self._create_cloud(filtered.points, filtered.colors, header)
        )

    def _publish_workspace_cloud(self, selection, source_header):
        """发布 base-frame ROI 后、RANSAC 前的调试点云。"""
        workspace_header = Header(
            stamp=source_header.stamp,
            frame_id=self._workspace_frame,
        )
        self._workspace_publisher.publish(
            self._create_cloud(
                selection.workspace_points,
                selection.camera_cloud.colors,
                workspace_header,
            )
        )

    def _publish_object_cloud(self, result, source_header):
        """发布 base-frame RANSAC 后、SOR 前的调试点云。

        RANSAC disabled 或 fallback 时 result 包含原 ROI，因此该话题仍连续发布。
        """
        workspace_header = Header(
            stamp=source_header.stamp,
            frame_id=self._workspace_frame,
        )
        self._object_publisher.publish(
            self._create_cloud(
                result.workspace_points,
                result.camera_cloud.colors,
                workspace_header,
            )
        )

    def _create_cloud(self, points, colors, header):
        """Create one XYZ/RGB PointCloud2 without changing its timestamp/frame."""
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.FLOAT32, 1),
        ]
        packed_rgb = self._pack_colors(colors)
        records = np.column_stack((points, packed_rgb)).tolist()
        return point_cloud2.create_cloud(header, fields, records)

    @staticmethod
    def _point(values):
        return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))

    @staticmethod
    def _color(red, green, blue, alpha=1.0):
        return ColorRGBA(r=red, g=green, b=blue, a=alpha)

    def _clear_marker(self, header):
        """创建 DELETEALL Marker，避免旧抓姿在新一帧无结果时残留。"""
        marker = Marker()
        marker.pose.orientation.w = 1.0

        marker.header = header
        marker.ns = "anygrasp_clear"
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    def _publish_clear_markers(self, header):
        """清除 RViz 中上一次发布的全部 AnyGrasp Marker。"""
        self._marker_publisher.publish(MarkerArray(markers=[self._clear_marker(header)]))

    def _publish_grasps(self, grasps, header, camera_to_workspace):
        """将已按分数排序的抓姿发布为 RViz Marker 和最佳 PoseStamped。

        ``grasps[0]`` 必须是最佳抓姿。Marker 表示抓取中心、接近方向和夹爪开口；
        best_grasp 保持输入点云坐标系，best_grasp_base 使用该点云时间戳对应的
        camera_to_workspace 同时变换位置和姿态，并沿用原始点云时间戳。
        """
        markers = [self._clear_marker(header)]
        marker_id = 1
        for index in range(len(grasps)):
            grasp = grasps[index]
            # SDK 的 rotation_matrix 三列依次定义接近、开口及其正交方向。
            center = np.asarray(grasp.translation, dtype=np.float64)
            approach, opening, _ = grasp_axes(grasp.rotation_matrix)
            is_best = index == 0

            # 球体表示抓取中心；第 0 个（最佳）抓姿使用更大、更醒目的标记。
            center_marker = Marker()
            center_marker.header = header
            center_marker.ns = "anygrasp_centers"
            center_marker.id = marker_id
            marker_id += 1
            center_marker.type = Marker.SPHERE
            center_marker.action = Marker.ADD
            center_marker.pose.position = self._point(center)
            center_marker.pose.orientation.w = 1.0
            diameter = 0.018 if is_best else 0.011
            center_marker.scale.x = diameter
            center_marker.scale.y = diameter
            center_marker.scale.z = diameter
            center_marker.color = (
                self._color(0.1, 1.0, 0.1) if is_best else self._color(1.0, 0.7, 0.1)
            )
            center_marker.lifetime = self._marker_lifetime
            markers.append(center_marker)

            # 箭头从抓取中心沿 approach 方向指出机械臂接近物体的方向。
            approach_marker = Marker()
            approach_marker.pose.orientation.w = 1.0
            approach_marker.header = header
            approach_marker.ns = "anygrasp_approach"
            approach_marker.id = marker_id
            marker_id += 1
            approach_marker.type = Marker.ARROW
            approach_marker.action = Marker.ADD
            approach_marker.points = [
                self._point(center),
                self._point(center + approach * (0.09 if is_best else 0.065)),
            ]
            approach_marker.scale.x = 0.007 if is_best else 0.004
            approach_marker.scale.y = 0.014 if is_best else 0.008
            approach_marker.scale.z = 0.018 if is_best else 0.012
            approach_marker.color = (
                self._color(0.1, 1.0, 0.1) if is_best else self._color(1.0, 0.2, 0.1)
            )
            approach_marker.lifetime = self._marker_lifetime
            markers.append(approach_marker)

            # 三条线表示两指之间的开口，以及两根手指沿 approach 的深度。
            half_width = max(float(grasp.width), 0.01) * 0.5
            jaw_depth = max(float(grasp.depth), 0.025)
            left = center - opening * half_width
            right = center + opening * half_width
            gripper_marker = Marker()
            gripper_marker.pose.orientation.w = 1.0
            gripper_marker.header = header
            gripper_marker.ns = "anygrasp_opening"
            gripper_marker.id = marker_id
            marker_id += 1
            gripper_marker.type = Marker.LINE_LIST
            gripper_marker.action = Marker.ADD
            gripper_marker.points = [
                self._point(left),
                self._point(right),
                self._point(left),
                self._point(left + approach * jaw_depth),
                self._point(right),
                self._point(right + approach * jaw_depth),
            ]
            gripper_marker.scale.x = 0.006 if is_best else 0.003
            gripper_marker.color = (
                self._color(0.1, 0.8, 1.0) if is_best else self._color(0.2, 0.5, 1.0)
            )
            gripper_marker.lifetime = self._marker_lifetime
            markers.append(gripper_marker)

        self._marker_publisher.publish(MarkerArray(markers=markers))

        # 只有最高分抓姿作为机器可消费的 PoseStamped 发布；其余仅作可视化。
        best = grasps[0]
        quaternion = rotation_matrix_to_quaternion(best.rotation_matrix)
        pose = PoseStamped()
        pose.header = header
        pose.pose.position = self._point(best.translation)
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        self._best_publisher.publish(pose)
        transform = camera_to_workspace.transform
        transform_quaternion = np.array(
            [
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ],
            dtype=np.float64,
        )
        rotation = quaternion_matrix(transform_quaternion)[:3, :3]
        translation = np.array(
            [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ],
            dtype=np.float64,
        )
        base_position = transform_points(
            np.asarray(best.translation, dtype=np.float32).reshape(1, 3),
            rotation,
            translation,
        )[0]
        base_quaternion = quaternion_multiply(transform_quaternion, quaternion)
        base_quaternion /= np.linalg.norm(base_quaternion)
        base_pose = PoseStamped()
        base_pose.header = Header(stamp=header.stamp, frame_id=self._workspace_frame)
        base_pose.pose.position = self._point(base_position)
        base_pose.pose.orientation.x = float(base_quaternion[0])
        base_pose.pose.orientation.y = float(base_quaternion[1])
        base_pose.pose.orientation.z = float(base_quaternion[2])
        base_pose.pose.orientation.w = float(base_quaternion[3])
        self._best_base_publisher.publish(base_pose)

    def _process_cloud(self, message):
        """处理单帧点云：解析 → ROI → RANSAC → SOR → 推理 → 发布。"""
        total_started = time.perf_counter()
        timing_ms = {
            "parse": 0.0,
            "filter": 0.0,
            "ransac": 0.0,
            "object_publish": 0.0,
            "input_publish": 0.0,
            "inference": 0.0,
            "nms_sort": 0.0,
        }

        # 所有阶段统一计时，便于判断高负载来自解析、过滤还是网络推理。
        def measure(stage, operation):
            stage_started = time.perf_counter()
            try:
                return operation()
            finally:
                timing_ms[stage] = (time.perf_counter() - stage_started) * 1000.0

        try:
            header = self._header_from_cloud(message)
            try:
                camera_to_workspace = self._tf_buffer.lookup_transform(
                    self._workspace_frame,
                    header.frame_id,
                    header.stamp,
                    self._tf_timeout,
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exc:
                rospy.logwarn_throttle(
                    5.0,
                    "[AnyGrasp] waiting for TF: %s <- %s at point-cloud stamp; "
                    "base-frame workspace filtering is unavailable: %s",
                    self._workspace_frame,
                    header.frame_id,
                    exc,
                )
                self._publish_clear_markers(header)
                return
            # 第一步：解析完整 PointCloud2。此时尚未减少点数。
            points, packed_rgb = measure(
                "parse", lambda: self._cloud_to_arrays(message)
            )
            # 第二步：用同一帧 TF 做一次 NumPy 向量化 R/t 变换，在 base frame
            # 计算 ROI mask，并把 mask 应用回原 camera-frame XYZ/RGB。
            transform = camera_to_workspace.transform
            quaternion = transform.rotation
            rotation = quaternion_matrix(
                [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            )[:3, :3]
            translation = np.array(
                [
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ],
                dtype=np.float64,
            )

            def transform_and_select():
                workspace_points = transform_points(points, rotation, translation)
                return select_workspace(
                    points,
                    workspace_points,
                    packed_rgb,
                    self._workspace_bounds,
                )

            selection = measure("filter", transform_and_select)
            filtered = selection.camera_cloud
            rospy.loginfo(
                "[AnyGrasp] cloud frame: %s | raw points: %d | valid points: %d | workspace points: %d",
                header.frame_id,
                filtered.raw_count,
                filtered.valid_count,
                filtered.workspace_count,
            )

            # workspace_cloud 的定义固定为 base-frame ROI 后、RANSAC 前。
            self._publish_workspace_cloud(selection, header)

            plane_result = measure(
                "ransac",
                lambda: remove_table_plane(
                    filtered,
                    selection.workspace_points,
                    self._ransac_config,
                ),
            )
            plane_text = (
                "none"
                if plane_result.plane_model is None
                else " ".join("%.6f" % value for value in plane_result.plane_model)
            )
            height_text = (
                "n/a"
                if plane_result.table_height is None
                else "%.6f m" % plane_result.table_height
            )
            rospy.loginfo(
                "[AnyGrasp] ROI points: %d | RANSAC plane: %s | "
                "candidate table height: %s | Plane inliers: %d | "
                "Plane ratio: %.2f%% | Object points after plane removal: %d | "
                "applied: %s",
                filtered.workspace_count,
                plane_text,
                height_text,
                plane_result.inlier_count,
                plane_result.inlier_ratio * 100.0,
                plane_result.camera_cloud.workspace_count,
                plane_result.applied,
            )
            if self._ransac_config.enabled and not plane_result.applied:
                rospy.logwarn_throttle(
                    5.0,
                    "[AnyGrasp] table removal fallback (%s); using original ROI cloud",
                    plane_result.reason,
                )

            # object_cloud 始终发布；fallback/disabled 时内容就是原 ROI。
            measure(
                "object_publish",
                lambda: self._publish_object_cloud(plane_result, header),
            )
            filtered = plane_result.camera_cloud

            if getattr(self, "_outlier_filter_enabled", False):
                before_outlier_filter = filtered.workspace_count
                filter_started = time.perf_counter()

                filtered = remove_statistical_outliers(
                    filtered,
                    self._outlier_nb_neighbors,
                    self._outlier_std_ratio,
                )

                outlier_filter_ms = (
                    time.perf_counter() - filter_started
                ) * 1000.0

                rospy.loginfo(
                    "[AnyGrasp] statistical outlier filter | "
                    "before: %d | after: %d | removed: %d | "
                    "nb_neighbors: %d | std_ratio: %.3f | "
                    "timing ms: %.3f",
                    before_outlier_filter,
                    filtered.workspace_count,
                    before_outlier_filter - filtered.workspace_count,
                    self._outlier_nb_neighbors,
                    self._outlier_std_ratio,
                    outlier_filter_ms,
                )

            if self._publish_input:
                measure(
                    "input_publish",
                    lambda: self._publish_input_cloud(filtered, header),
                )
            # 点数不足时不调用昂贵的神经网络，同时清除上一帧可视化结果。
            if filtered.workspace_count == 0:
                rospy.logwarn_throttle(
                    5.0, "[AnyGrasp] workspace is empty; skipping inference"
                )
                self._publish_clear_markers(header)
                return
            if filtered.workspace_count < self._min_workspace_points:
                rospy.logwarn_throttle(
                    5.0,
                    "[AnyGrasp] workspace has only %d points (minimum %d); skipping inference",
                    filtered.workspace_count,
                    self._min_workspace_points,
                )
                self._publish_clear_markers(header)
                return

            # 点数达标后才计算动态 camera-frame lims，避免对空/不足点云求范围。
            camera_bounds = dynamic_point_bounds(
                filtered.points,
                self._dynamic_lims_margin,
            )
            # 第三步：只有 ROI 内点数达标后，才把点和颜色送入 AnyGrasp SDK。
            grasps = measure(
                "inference",
                lambda: self._adapter.infer(
                    filtered.points, filtered.colors, camera_bounds
                ),
            )
            before_nms = len(grasps)
            if before_nms == 0:
                rospy.logwarn_throttle(5.0, "[AnyGrasp] no grasp detected")
                rospy.loginfo(
                    "[AnyGrasp] grasps before nms: 0 | grasps after nms: 0 | published grasps: 0"
                )
                self._publish_clear_markers(header)
                return

            # 第四步：NMS 去除高度重叠的重复抓姿，再按置信度从高到低排序。
            grasps = measure("nms_sort", lambda: grasps.nms().sort_by_score())
            after_nms = len(grasps)
            # top_n 只限制发布数量；它不会减少前面的网络输入或推理计算量。
            selected = grasps[: min(self._top_n, after_nms)]
            if len(selected) == 0:
                self._publish_clear_markers(header)
                return
            self._publish_grasps(selected, header, camera_to_workspace)
            rospy.loginfo(
                "[AnyGrasp] grasps before nms: %d | grasps after nms: %d | published grasps: %d | best score: %.6f",
                before_nms,
                after_nms,
                len(selected),
                float(selected[0].score),
            )
        finally:
            total_ms = (time.perf_counter() - total_started) * 1000.0
            rospy.loginfo(
                "[AnyGrasp] timing ms | PointCloud2 parse: %.3f | "
                "finite+workspace filter: %.3f | RANSAC table filter: %.3f | "
                "RViz object publish: %.3f | RViz input publish: %.3f | "
                "AnyGrasp inference: %.3f | NMS+sort: %.3f | TOTAL: %.3f",
                timing_ms["parse"],
                timing_ms["filter"],
                timing_ms["ransac"],
                timing_ms["object_publish"],
                timing_ms["input_publish"],
                timing_ms["inference"],
                timing_ms["nms_sort"],
                total_ms,
            )

    def run(self):
        """按 inference_rate 处理最新帧；旧帧不会排队补算。"""
        rate = rospy.Rate(self._inference_rate)
        last_sequence = 0
        while not rospy.is_shutdown():
            # inference_rate=0.2 表示每 5 秒最多处理一帧，而不是每秒 5 帧。
            message, sequence = self._latest.take_latest(last_sequence)
            if message is None:
                rospy.logwarn_throttle(5.0, "[AnyGrasp] waiting for point cloud")
                rate.sleep()
                continue
            last_sequence = sequence
            try:
                self._process_cloud(message)
            except Exception as exc:
                if is_fatal_backend_error(exc):
                    rospy.logfatal("[AnyGrasp] fatal inference backend error: %s", exc)
                    rospy.signal_shutdown("fatal AnyGrasp inference backend error")
                    raise
                rospy.logerr_throttle(5.0, "[AnyGrasp] frame processing failed: %s", exc)
                self._publish_clear_markers(self._header_from_cloud(message))
            rate.sleep()


def main():
    """节点进程入口：启动失败和运行期致命失败最终都会以异常退出。"""
    try:
        node = AnyGraspD405Node()
        node.run()
    except Exception as exc:
        if rospy.core.is_initialized():
            rospy.logfatal("[AnyGrasp] node terminated: %s", exc)
        else:
            print("[AnyGrasp] fatal startup error: %s" % exc, file=sys.stderr)
        raise


if __name__ == "__main__":
    main()
