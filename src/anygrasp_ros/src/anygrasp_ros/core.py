"""AnyGrasp ROS 节点与测试共用的纯数值计算。

本模块不依赖 rospy、话题或模型，因此 ROI、颜色和姿态计算可以单独测试。
数组约定：点和颜色均为 ``(N, 3)``，旋转矩阵为 ``(3, 3)``。
"""

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np


POINT_FIELD_UINT32 = 6
POINT_FIELD_FLOAT32 = 7


@dataclass(frozen=True)
class FilteredCloud:
    """一次点云过滤的结果及各阶段点数统计。

    ``raw_count`` 是输入总数，``valid_count`` 是有限 XYZ 点数，
    ``workspace_count`` 是最终落入 ROI 的点数。
    """

    points: np.ndarray
    colors: np.ndarray
    raw_count: int
    valid_count: int
    workspace_count: int


@dataclass(frozen=True)
class WorkspaceSelection:
    """Base-frame ROI selection while preserving camera-frame inference data."""

    camera_cloud: FilteredCloud
    workspace_points: np.ndarray


def decode_packed_rgb(values: np.ndarray, datatype: int) -> np.ndarray:
    """把 ROS 打包 RGB 解码为范围 ``[0, 1]`` 的 ``float32 (N, 3)``。"""
    flat = np.asarray(values).reshape(-1)
    if datatype == POINT_FIELD_FLOAT32:
        packed = np.asarray(flat, dtype=np.float32).view(np.uint32)
    elif datatype == POINT_FIELD_UINT32:
        packed = np.asarray(flat, dtype=np.uint32)
    else:
        raise ValueError("rgb field must use PointField FLOAT32 or UINT32")

    # packed RGB 的位布局为 0x00RRGGBB，位移后分别取出三个 8 bit 通道。
    colors = np.column_stack(
        ((packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF)
    )
    return (colors.astype(np.float32) / np.float32(255.0)).reshape(-1, 3)


def select_finite_cloud(
    camera_points: np.ndarray,
    packed_rgb: np.ndarray,
) -> FilteredCloud:
    """Keep finite camera-frame XYZ and its aligned packed RGB samples."""
    camera_array = np.asarray(camera_points, dtype=np.float32)
    packed_array = np.asarray(packed_rgb)
    if camera_array.ndim != 2 or camera_array.shape[1] != 3:
        raise ValueError("camera_points must have shape (N, 3)")
    if packed_array.ndim != 1:
        raise ValueError("packed_rgb must have shape (N,)")
    if camera_array.shape[0] != packed_array.shape[0]:
        raise ValueError("camera_points and packed_rgb must contain the same samples")

    finite_mask = np.isfinite(camera_array).all(axis=1)
    finite_points = camera_array[finite_mask].astype(np.float32, copy=False)
    finite_packed_rgb = packed_array[finite_mask].astype(np.uint32, copy=False)
    finite_count = int(finite_points.shape[0])
    return FilteredCloud(
        points=finite_points,
        colors=decode_packed_rgb(finite_packed_rgb, POINT_FIELD_UINT32),
        raw_count=int(camera_array.shape[0]),
        valid_count=finite_count,
        workspace_count=finite_count,
    )


def transform_points(
    points: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    """Apply one target-from-source rigid transform to an ``(N, 3)`` array."""
    point_array = np.asarray(points, dtype=np.float32)
    rotation_array = np.asarray(rotation, dtype=np.float64)
    translation_array = np.asarray(translation, dtype=np.float64)
    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if rotation_array.shape != (3, 3):
        raise ValueError("rotation must have shape (3, 3)")
    if translation_array.shape != (3,):
        raise ValueError("translation must have shape (3,)")
    if not np.isfinite(rotation_array).all() or not np.isfinite(translation_array).all():
        raise ValueError("rotation and translation must be finite")

    # Row-vector equivalent of p_target = R_target_source * p_source + t.
    return (point_array @ rotation_array.T + translation_array).astype(
        np.float32, copy=False
    )


def select_workspace(
    camera_points: np.ndarray,
    workspace_points: np.ndarray,
    packed_rgb: np.ndarray,
    bounds: Sequence[float],
) -> WorkspaceSelection:
    """Build a base-frame ROI mask and apply it to camera XYZ and packed RGB.

    ``bounds`` 顺序是 ``x_min, x_max, y_min, y_max, z_min, z_max``，
    始终属于 ``workspace_points`` 的坐标系。
    """
    camera_array = np.asarray(camera_points, dtype=np.float32)
    workspace_array = np.asarray(workspace_points, dtype=np.float32)
    packed_array = np.asarray(packed_rgb)
    if camera_array.ndim != 2 or camera_array.shape[1] != 3:
        raise ValueError("camera_points must have shape (N, 3)")
    if workspace_array.ndim != 2 or workspace_array.shape[1] != 3:
        raise ValueError("workspace_points must have shape (N, 3)")
    if packed_array.ndim != 1:
        raise ValueError("packed_rgb must have shape (N,)")
    if not (
        camera_array.shape[0]
        == workspace_array.shape[0]
        == packed_array.shape[0]
    ):
        raise ValueError(
            "camera_points, workspace_points, and packed_rgb must contain the same number of samples"
        )
    if len(bounds) != 6:
        raise ValueError("workspace bounds must be x_min,x_max,y_min,y_max,z_min,z_max")

    x_min, x_max, y_min, y_max, z_min, z_max = (float(v) for v in bounds)
    if x_min > x_max or y_min > y_max or z_min > z_max:
        raise ValueError("workspace minimums must not exceed maximums")

    # combined_mask 与原始 camera XYZ/RGB 逐一对应。
    combined_mask = np.isfinite(camera_array).all(axis=1)
    combined_mask &= np.isfinite(workspace_array).all(axis=1)
    valid_count = int(np.count_nonzero(combined_mask))
    combined_mask &= workspace_array[:, 0] >= x_min
    combined_mask &= workspace_array[:, 0] <= x_max
    combined_mask &= workspace_array[:, 1] >= y_min
    combined_mask &= workspace_array[:, 1] <= y_max
    combined_mask &= workspace_array[:, 2] >= z_min
    combined_mask &= workspace_array[:, 2] <= z_max

    selected_camera_points = camera_array[combined_mask].astype(np.float32, copy=False)
    selected_workspace_points = workspace_array[combined_mask].astype(
        np.float32, copy=False
    )
    workspace_packed_rgb = packed_array[combined_mask].astype(np.uint32, copy=False)
    workspace_colors = decode_packed_rgb(workspace_packed_rgb, POINT_FIELD_UINT32)
    camera_cloud = FilteredCloud(
        points=selected_camera_points,
        colors=workspace_colors,
        raw_count=int(camera_array.shape[0]),
        valid_count=valid_count,
        workspace_count=int(selected_camera_points.shape[0]),
    )
    return WorkspaceSelection(
        camera_cloud=camera_cloud,
        workspace_points=selected_workspace_points,
    )


def dynamic_point_bounds(points: np.ndarray, margin: float) -> Tuple[float, ...]:
    """Return camera-frame XYZ bounds expanded by ``margin`` on all sides."""
    point_array = np.asarray(points, dtype=np.float32)
    margin_value = float(margin)
    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if point_array.shape[0] == 0:
        raise ValueError("cannot compute bounds for an empty point cloud")
    if not np.isfinite(point_array).all():
        raise ValueError("points must be finite")
    if not np.isfinite(margin_value) or margin_value < 0.0:
        raise ValueError("margin must be finite and non-negative")

    minimum = point_array.min(axis=0).astype(np.float64) - margin_value
    maximum = point_array.max(axis=0).astype(np.float64) + margin_value
    return (
        float(minimum[0]),
        float(maximum[0]),
        float(minimum[1]),
        float(maximum[1]),
        float(minimum[2]),
        float(maximum[2]),
    )


def _proper_rotation(matrix: np.ndarray) -> np.ndarray:
    """把有数值误差的 3x3 矩阵投影到最近的合法旋转矩阵。"""
    rotation = np.asarray(matrix, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("rotation matrix must have shape (3, 3)")
    if not np.isfinite(rotation).all():
        raise ValueError("rotation matrix must be finite")
    # SVD 正交化可消除网络输出中的轻微缩放/剪切误差。
    left, _, right = np.linalg.svd(rotation)
    projected = left @ right
    if np.linalg.det(projected) < 0.0:
        left[:, -1] *= -1.0
        projected = left @ right
    return projected


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """将抓姿旋转矩阵转换为 ROS 使用的归一化 ``[x, y, z, w]`` 四元数。"""
    rotation = _proper_rotation(matrix)
    trace = float(np.trace(rotation))
    # 根据最大对角分量选择数值更稳定的计算分支，避免接近 180° 时精度变差。
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
        w = 0.25 * scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
            w = (rotation[2, 1] - rotation[1, 2]) / scale
        elif index == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
            w = (rotation[0, 2] - rotation[2, 0]) / scale
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
            w = (rotation[1, 0] - rotation[0, 1]) / scale

    quaternion = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm <= np.finfo(np.float64).eps:
        raise ValueError("rotation matrix produced a zero quaternion")
    quaternion /= norm
    # q 与 -q 表示同一旋转；统一令 w >= 0，减少相邻帧符号无意义跳变。
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return quaternion


def grasp_axes(matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """返回旋转矩阵三列：接近方向、夹爪开口方向、两者的正交方向。"""
    rotation = _proper_rotation(matrix)
    return rotation[:, 0].copy(), rotation[:, 1].copy(), rotation[:, 2].copy()
