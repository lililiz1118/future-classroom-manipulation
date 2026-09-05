"""Pure table-plane geometry shared by perception and collision handling."""

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from anygrasp_ros.core import rotation_matrix_to_quaternion


@dataclass(frozen=True)
class TableSurfaceGeometry:
    plane_model: np.ndarray
    normal: np.ndarray
    center: np.ndarray
    corners: np.ndarray
    rotation: np.ndarray
    quaternion: np.ndarray


def _roi_values(roi_xy: Sequence[float]):
    values = np.asarray(roi_xy, dtype=np.float64).reshape(-1)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError("roi_xy must contain finite x_min,x_max,y_min,y_max")
    x_min, x_max, y_min, y_max = values
    if x_min > x_max or y_min > y_max:
        raise ValueError("ROI minimums must not exceed maximums")
    return float(x_min), float(x_max), float(y_min), float(y_max)


def normalize_table_plane(plane_model: Sequence[float]) -> np.ndarray:
    """Normalize an Open3D ``[a,b,c,d]`` plane and point its normal upward."""
    model = np.asarray(plane_model, dtype=np.float64).reshape(-1)
    if model.shape != (4,) or not np.isfinite(model).all():
        raise ValueError("plane_model must contain four finite coefficients")
    normal_norm = float(np.linalg.norm(model[:3]))
    if normal_norm <= np.finfo(np.float64).eps:
        raise ValueError("plane normal must be non-zero")
    normalized = model / normal_norm
    if normalized[2] < 0.0:
        normalized *= -1.0
    return normalized


def table_basis(normal: Sequence[float]) -> np.ndarray:
    """Return right-handed columns ``[local_x, local_y, normal]``."""
    upward = np.asarray(normal, dtype=np.float64).reshape(-1)
    if upward.shape != (3,) or not np.isfinite(upward).all():
        raise ValueError("normal must contain three finite values")
    norm = float(np.linalg.norm(upward))
    if norm <= np.finfo(np.float64).eps:
        raise ValueError("normal must be non-zero")
    upward = upward / norm
    if upward[2] < 0.0:
        upward *= -1.0

    base_x = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    tangent_x = base_x - float(np.dot(base_x, upward)) * upward
    tangent_norm = float(np.linalg.norm(tangent_x))
    if tangent_norm <= 1e-10:
        base_y = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        tangent_x = base_y - float(np.dot(base_y, upward)) * upward
        tangent_norm = float(np.linalg.norm(tangent_x))
    if tangent_norm <= 1e-10:
        raise ValueError("cannot construct a table tangent basis")
    tangent_x /= tangent_norm
    tangent_y = np.cross(upward, tangent_x)
    tangent_y /= np.linalg.norm(tangent_y)
    tangent_x = np.cross(tangent_y, upward)
    return np.column_stack((tangent_x, tangent_y, upward))


def table_surface_from_plane(
    plane_model: Sequence[float], roi_xy: Sequence[float]
) -> TableSurfaceGeometry:
    """Intersect a base-frame XY ROI with a plane and construct its surface pose."""
    plane = normalize_table_plane(plane_model)
    if abs(float(plane[2])) <= 1e-10:
        raise ValueError("table plane must be evaluable as z(x,y)")
    x_min, x_max, y_min, y_max = _roi_values(roi_xy)
    xy = np.array(
        [
            [x_min, y_min],
            [x_min, y_max],
            [x_max, y_min],
            [x_max, y_max],
        ],
        dtype=np.float64,
    )
    z = -(xy[:, 0] * plane[0] + xy[:, 1] * plane[1] + plane[3]) / plane[2]
    corners = np.column_stack((xy, z))

    center_x = (x_min + x_max) / 2.0
    center_y = (y_min + y_max) / 2.0
    center_z = -(plane[0] * center_x + plane[1] * center_y + plane[3]) / plane[2]
    center = np.array([center_x, center_y, center_z], dtype=np.float64)
    rotation = table_basis(plane[:3])
    return TableSurfaceGeometry(
        plane_model=plane,
        normal=plane[:3].copy(),
        center=center,
        corners=corners,
        rotation=rotation,
        quaternion=rotation_matrix_to_quaternion(rotation),
    )


def plane_from_normal_and_center_height(
    normal: Sequence[float], height_at_roi_center: float, roi_xy: Sequence[float]
) -> np.ndarray:
    """Rebuild a normalized plane through the ROI-center representative height."""
    x_min, x_max, y_min, y_max = _roi_values(roi_xy)
    normalized = np.asarray(normal, dtype=np.float64).reshape(-1)
    if normalized.shape != (3,) or not np.isfinite(normalized).all():
        raise ValueError("normal must contain three finite values")
    norm = float(np.linalg.norm(normalized))
    height = float(height_at_roi_center)
    if norm <= np.finfo(np.float64).eps or not np.isfinite(height):
        raise ValueError("normal and height must define a finite plane")
    normalized /= norm
    if normalized[2] < 0.0:
        normalized *= -1.0
    center = np.array(
        [(x_min + x_max) / 2.0, (y_min + y_max) / 2.0, height],
        dtype=np.float64,
    )
    return np.append(normalized, -float(np.dot(normalized, center)))
