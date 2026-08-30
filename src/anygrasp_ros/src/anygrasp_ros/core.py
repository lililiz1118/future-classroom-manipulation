"""Numerical contracts shared by the AnyGrasp ROS node and its tests."""

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np


POINT_FIELD_UINT32 = 6
POINT_FIELD_FLOAT32 = 7


@dataclass(frozen=True)
class FilteredCloud:
    points: np.ndarray
    colors: np.ndarray
    raw_count: int
    valid_count: int
    workspace_count: int


def decode_packed_rgb(values: np.ndarray, datatype: int) -> np.ndarray:
    """Decode ROS packed RGB values into float32 colors in [0, 1]."""
    flat = np.asarray(values).reshape(-1)
    if datatype == POINT_FIELD_FLOAT32:
        packed = np.asarray(flat, dtype=np.float32).view(np.uint32)
    elif datatype == POINT_FIELD_UINT32:
        packed = np.asarray(flat, dtype=np.uint32)
    else:
        raise ValueError("rgb field must use PointField FLOAT32 or UINT32")

    colors = np.column_stack(
        ((packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF)
    )
    return (colors.astype(np.float32) / np.float32(255.0)).reshape(-1, 3)


def filter_workspace(
    points: np.ndarray,
    colors: np.ndarray,
    bounds: Sequence[float],
) -> FilteredCloud:
    """Remove invalid samples and crop using inclusive XYZ bounds."""
    point_array = np.asarray(points, dtype=np.float32)
    color_array = np.asarray(colors, dtype=np.float32)
    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if color_array.ndim != 2 or color_array.shape[1] != 3:
        raise ValueError("colors must have shape (N, 3)")
    if point_array.shape[0] != color_array.shape[0]:
        raise ValueError("points and colors must contain the same number of samples")
    if len(bounds) != 6:
        raise ValueError("workspace bounds must be x_min,x_max,y_min,y_max,z_min,z_max")

    x_min, x_max, y_min, y_max, z_min, z_max = (float(v) for v in bounds)
    if x_min > x_max or y_min > y_max or z_min > z_max:
        raise ValueError("workspace minimums must not exceed maximums")

    finite_mask = np.isfinite(point_array).all(axis=1) & np.isfinite(color_array).all(axis=1)
    valid_points = point_array[finite_mask]
    valid_colors = color_array[finite_mask]
    workspace_mask = (
        (valid_points[:, 0] >= x_min)
        & (valid_points[:, 0] <= x_max)
        & (valid_points[:, 1] >= y_min)
        & (valid_points[:, 1] <= y_max)
        & (valid_points[:, 2] >= z_min)
        & (valid_points[:, 2] <= z_max)
    )
    workspace_points = valid_points[workspace_mask].astype(np.float32, copy=False)
    workspace_colors = valid_colors[workspace_mask].astype(np.float32, copy=False)
    return FilteredCloud(
        points=workspace_points,
        colors=workspace_colors,
        raw_count=int(point_array.shape[0]),
        valid_count=int(valid_points.shape[0]),
        workspace_count=int(workspace_points.shape[0]),
    )


def _proper_rotation(matrix: np.ndarray) -> np.ndarray:
    rotation = np.asarray(matrix, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("rotation matrix must have shape (3, 3)")
    if not np.isfinite(rotation).all():
        raise ValueError("rotation matrix must be finite")
    left, _, right = np.linalg.svd(rotation)
    projected = left @ right
    if np.linalg.det(projected) < 0.0:
        left[:, -1] *= -1.0
        projected = left @ right
    return projected


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Convert a grasp-frame rotation to a normalized ROS xyzw quaternion."""
    rotation = _proper_rotation(matrix)
    trace = float(np.trace(rotation))
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
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return quaternion


def grasp_axes(matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return approach, opening, and orthogonal axes from matrix columns."""
    rotation = _proper_rotation(matrix)
    return rotation[:, 0].copy(), rotation[:, 1].copy(), rotation[:, 2].copy()
