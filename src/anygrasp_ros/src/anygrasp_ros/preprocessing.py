"""Point-cloud preprocessing before AnyGrasp inference."""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import open3d as o3d

from anygrasp_ros.core import FilteredCloud


@dataclass(frozen=True)
class RansacPlaneConfig:
    """Validated parameters for one bounded, single-plane RANSAC pass."""

    enabled: bool
    distance_threshold: float
    ransac_n: int
    num_iterations: int
    min_points: int
    max_normal_angle_deg: float
    table_height_min: float
    table_height_max: float
    min_inliers: int
    min_inlier_ratio: float
    min_object_points: int

    def __post_init__(self):
        numeric_values = (
            self.distance_threshold,
            self.max_normal_angle_deg,
            self.table_height_min,
            self.table_height_max,
            self.min_inlier_ratio,
        )
        if not np.isfinite(numeric_values).all():
            raise ValueError("RANSAC floating-point parameters must be finite")
        if self.distance_threshold <= 0.0:
            raise ValueError("RANSAC distance_threshold must be positive")
        if self.ransac_n < 3:
            raise ValueError("RANSAC ransac_n must be at least 3")
        if self.num_iterations < 1:
            raise ValueError("RANSAC num_iterations must be positive")
        if self.min_points < self.ransac_n:
            raise ValueError("RANSAC min_points must be at least ransac_n")
        if not 0.0 <= self.max_normal_angle_deg <= 90.0:
            raise ValueError("RANSAC max_normal_angle_deg must be in [0, 90]")
        if self.table_height_min > self.table_height_max:
            raise ValueError("RANSAC table height minimum must not exceed maximum")
        if self.min_inliers < self.ransac_n:
            raise ValueError("RANSAC min_inliers must be at least ransac_n")
        if not 0.0 <= self.min_inlier_ratio <= 1.0:
            raise ValueError("RANSAC min_inlier_ratio must be in [0, 1]")
        if self.min_object_points < 0:
            raise ValueError("RANSAC min_object_points must be non-negative")


@dataclass(frozen=True)
class TablePlaneAssessment:
    """Geometric checks for the largest plane returned by Open3D."""

    accepted: bool
    reason: str
    plane_model: Tuple[float, float, float, float]
    inlier_indices: np.ndarray
    inlier_count: int
    inlier_ratio: float
    table_height: Optional[float]
    normal_angle_deg: Optional[float]


@dataclass(frozen=True)
class PlaneRemovalResult:
    """Camera/base arrays after RANSAC, or the unchanged ROI on fallback."""

    camera_cloud: FilteredCloud
    workspace_points: np.ndarray
    applied: bool
    reason: str
    plane_model: Optional[Tuple[float, float, float, float]]
    inlier_count: int
    inlier_ratio: float
    table_height: Optional[float]
    normal_angle_deg: Optional[float]


def _fallback_result(
    cloud,
    workspace_points,
    reason,
    plane_model=None,
    inlier_count=0,
    inlier_ratio=0.0,
    table_height=None,
    normal_angle_deg=None,
):
    return PlaneRemovalResult(
        camera_cloud=cloud,
        workspace_points=workspace_points,
        applied=False,
        reason=reason,
        plane_model=plane_model,
        inlier_count=int(inlier_count),
        inlier_ratio=float(inlier_ratio),
        table_height=table_height,
        normal_angle_deg=normal_angle_deg,
    )


def assess_table_plane(workspace_points, plane_model, inlier_indices, config):
    """Check whether one RANSAC candidate is a plausible horizontal table.

    ``ur_arm_base_link`` is REP-103 aligned, so Z is vertical.  Plane-normal
    sign is ambiguous; the angle therefore uses ``abs(dot(normal, Z))``.
    Candidate height is the median Z of its actual inlier points rather than
    the algebraic ``-d/c`` value.
    """
    points = np.asarray(workspace_points, dtype=np.float32)
    model = np.asarray(plane_model, dtype=np.float64).reshape(-1)
    indices = np.asarray(inlier_indices, dtype=np.int64).reshape(-1)
    point_count = int(points.shape[0])
    plane_tuple = tuple(float(value) for value in model)
    inlier_count = int(indices.size)
    inlier_ratio = float(inlier_count) / float(point_count) if point_count else 0.0

    if model.shape != (4,) or not np.isfinite(model).all():
        return TablePlaneAssessment(
            False, "invalid_plane_model", plane_tuple, indices,
            inlier_count, inlier_ratio, None, None,
        )
    if indices.size == 0 or np.any(indices < 0) or np.any(indices >= point_count):
        return TablePlaneAssessment(
            False, "invalid_inliers", plane_tuple, indices,
            inlier_count, inlier_ratio, None, None,
        )

    normal = model[:3]
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm <= np.finfo(np.float64).eps:
        return TablePlaneAssessment(
            False, "invalid_plane_normal", plane_tuple, indices,
            inlier_count, inlier_ratio, None, None,
        )
    vertical_alignment = float(np.clip(abs(normal[2]) / normal_norm, 0.0, 1.0))
    normal_angle_deg = float(np.degrees(np.arccos(vertical_alignment)))
    table_height = float(np.median(points[indices, 2]))

    if normal_angle_deg > config.max_normal_angle_deg:
        reason = "normal_angle"
    elif not config.table_height_min <= table_height <= config.table_height_max:
        reason = "table_height"
    elif inlier_count < config.min_inliers:
        reason = "too_few_inliers"
    elif inlier_ratio < config.min_inlier_ratio:
        reason = "low_inlier_ratio"
    else:
        reason = "accepted"
    return TablePlaneAssessment(
        accepted=reason == "accepted",
        reason=reason,
        plane_model=plane_tuple,
        inlier_indices=indices,
        inlier_count=inlier_count,
        inlier_ratio=inlier_ratio,
        table_height=table_height,
        normal_angle_deg=normal_angle_deg,
    )


def remove_table_plane(cloud, workspace_points, config):
    """Remove one plausible table plane while preserving XYZ/RGB alignment.

    Open3D fits the base-frame ROI points.  The resulting boolean keep mask is
    applied to camera-frame XYZ, RGB, and base-frame XYZ together.  Any normal
    detection failure returns the unchanged ROI so inference can continue.
    """
    workspace_array = np.asarray(workspace_points, dtype=np.float32)
    camera_points = np.asarray(cloud.points, dtype=np.float32)
    colors = np.asarray(cloud.colors, dtype=np.float32)
    if workspace_array.ndim != 2 or workspace_array.shape[1] != 3:
        raise ValueError("workspace_points must have shape (N, 3)")
    if camera_points.ndim != 2 or camera_points.shape[1] != 3:
        raise ValueError("cloud points must have shape (N, 3)")
    if colors.ndim != 2 or colors.shape[1] != 3:
        raise ValueError("cloud colors must have shape (N, 3)")
    if not (
        workspace_array.shape[0] == camera_points.shape[0] == colors.shape[0]
    ):
        raise ValueError("workspace XYZ, camera XYZ, and RGB must stay aligned")

    point_count = int(workspace_array.shape[0])
    if not config.enabled:
        return _fallback_result(cloud, workspace_array, "disabled")
    if point_count < config.min_points:
        return _fallback_result(cloud, workspace_array, "too_few_roi_points")

    table_band_mask = (
        (workspace_array[:, 2] >= config.table_height_min)
        & (workspace_array[:, 2] <= config.table_height_max)
    )
    table_band_indices = np.flatnonzero(table_band_mask)
    if table_band_indices.size >= config.min_points:
        fit_indices = table_band_indices
    else:
        fit_indices = np.arange(point_count, dtype=np.int64)
    fit_points = workspace_array[fit_indices]

    try:
        open3d_cloud = o3d.geometry.PointCloud()
        open3d_cloud.points = o3d.utility.Vector3dVector(
            fit_points.astype(np.float64, copy=False)
        )
        plane_model, inlier_indices = open3d_cloud.segment_plane(
            distance_threshold=float(config.distance_threshold),
            ransac_n=int(config.ransac_n),
            num_iterations=int(config.num_iterations),
        )
    except Exception as exc:  # Open3D exceptions must not terminate the ROS node.
        return _fallback_result(
            cloud,
            workspace_array,
            "ransac_error: %s" % exc,
        )

    fit_inlier_indices = np.asarray(inlier_indices, dtype=np.int64).reshape(-1)
    if (
        fit_inlier_indices.size == 0
        or np.any(fit_inlier_indices < 0)
        or np.any(fit_inlier_indices >= fit_indices.size)
    ):
        return _fallback_result(
            cloud,
            workspace_array,
            "invalid_inliers",
            tuple(float(value) for value in np.asarray(plane_model).reshape(-1)),
            fit_inlier_indices.size,
        )
    workspace_inlier_indices = fit_indices[fit_inlier_indices]

    assessment = assess_table_plane(
        workspace_array,
        plane_model,
        workspace_inlier_indices,
        config,
    )
    if not assessment.accepted:
        return _fallback_result(
            cloud,
            workspace_array,
            assessment.reason,
            assessment.plane_model,
            assessment.inlier_count,
            assessment.inlier_ratio,
            assessment.table_height,
            assessment.normal_angle_deg,
        )

    keep_mask = np.ones(point_count, dtype=bool)
    keep_mask[assessment.inlier_indices] = False
    object_count = int(np.count_nonzero(keep_mask))
    if object_count < config.min_object_points:
        return _fallback_result(
            cloud,
            workspace_array,
            "too_few_object_points",
            assessment.plane_model,
            assessment.inlier_count,
            assessment.inlier_ratio,
            assessment.table_height,
            assessment.normal_angle_deg,
        )

    filtered_cloud = FilteredCloud(
        points=camera_points[keep_mask].astype(np.float32, copy=False),
        colors=colors[keep_mask].astype(np.float32, copy=False),
        raw_count=cloud.raw_count,
        valid_count=cloud.valid_count,
        workspace_count=object_count,
    )
    return PlaneRemovalResult(
        camera_cloud=filtered_cloud,
        workspace_points=workspace_array[keep_mask].astype(np.float32, copy=False),
        applied=True,
        reason="accepted",
        plane_model=assessment.plane_model,
        inlier_count=assessment.inlier_count,
        inlier_ratio=assessment.inlier_ratio,
        table_height=assessment.table_height,
        normal_angle_deg=assessment.normal_angle_deg,
    )


def remove_statistical_outliers(cloud, nb_neighbors, std_ratio):
    """Remove points with statistically abnormal neighbour distances."""

    nb_neighbors = int(nb_neighbors)
    std_ratio = float(std_ratio)

    if nb_neighbors < 1:
        raise ValueError("nb_neighbors must be at least 1")
    if std_ratio <= 0.0:
        raise ValueError("std_ratio must be positive")

    point_count = int(cloud.points.shape[0])
    if point_count <= nb_neighbors:
        return cloud

    open3d_cloud = o3d.geometry.PointCloud()
    open3d_cloud.points = o3d.utility.Vector3dVector(
        np.asarray(cloud.points, dtype=np.float64)
    )

    _, inlier_indices = open3d_cloud.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio,
    )
    inlier_indices = np.asarray(inlier_indices, dtype=np.int64)

    points = np.asarray(
        cloud.points[inlier_indices],
        dtype=np.float32,
    )
    colors = np.asarray(
        cloud.colors[inlier_indices],
        dtype=np.float32,
    )

    return FilteredCloud(
        points=points,
        colors=colors,
        raw_count=cloud.raw_count,
        valid_count=cloud.valid_count,
        workspace_count=int(points.shape[0]),
    )
