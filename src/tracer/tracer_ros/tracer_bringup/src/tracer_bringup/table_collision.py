"""Pure stability and box geometry for a RANSAC-derived table collision."""

from collections import deque
from dataclasses import dataclass
from itertools import combinations
from typing import Optional, Sequence

import numpy as np

from anygrasp_ros.table_geometry import (
    plane_from_normal_and_center_height,
    table_surface_from_plane,
)


@dataclass(frozen=True)
class TablePoseSample:
    frame_id: str
    stamp_sec: float
    normal: np.ndarray
    height_at_roi_center: float


@dataclass(frozen=True)
class StableTableModel:
    frame_id: str
    stamp_sec: float
    normal: np.ndarray
    height_at_roi_center: float


@dataclass(frozen=True)
class TableUpdateDecision:
    update_scene: bool
    reason: str
    stable_model: Optional[StableTableModel]


@dataclass(frozen=True)
class TableModelStatus:
    state: str
    age_sec: Optional[float]
    collision_present: bool


@dataclass(frozen=True)
class CollisionBox:
    center: np.ndarray
    surface_center: np.ndarray
    size: np.ndarray
    rotation: np.ndarray
    quaternion: np.ndarray
    roi_corners: np.ndarray
    top_corners: np.ndarray


def _normal_angle_deg(first, second):
    dot = float(np.clip(np.dot(first, second), -1.0, 1.0))
    return float(np.degrees(np.arccos(dot)))


def _normalized_upward(normal):
    value = np.asarray(normal, dtype=np.float64).reshape(-1)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("normal must contain three finite values")
    norm = float(np.linalg.norm(value))
    if norm <= np.finfo(np.float64).eps:
        raise ValueError("normal must be non-zero")
    value = value / norm
    if value[2] < 0.0:
        value *= -1.0
    return value


def build_collision_box(
    normal: Sequence[float],
    height_at_roi_center: float,
    roi_xy: Sequence[float],
    thickness: float,
    xy_margin: float,
) -> CollisionBox:
    thickness = float(thickness)
    margin = float(xy_margin)
    if not np.isfinite(thickness) or thickness <= 0.0:
        raise ValueError("table collision thickness must be positive")
    if not np.isfinite(margin) or margin < 0.0:
        raise ValueError("table XY margin must be non-negative")

    plane = plane_from_normal_and_center_height(normal, height_at_roi_center, roi_xy)
    surface = table_surface_from_plane(plane, roi_xy)
    local_roi = (surface.corners - surface.center) @ surface.rotation[:, :2]
    minimum = local_roi.min(axis=0)
    maximum = local_roi.max(axis=0)
    local_midpoint = (minimum + maximum) / 2.0
    surface_center = (
        surface.center
        + surface.rotation[:, 0] * local_midpoint[0]
        + surface.rotation[:, 1] * local_midpoint[1]
    )
    size = np.array(
        [maximum[0] - minimum[0] + 2.0 * margin,
         maximum[1] - minimum[1] + 2.0 * margin,
         thickness],
        dtype=np.float64,
    )
    center = surface_center - surface.normal * thickness / 2.0
    half_x, half_y = size[0] / 2.0, size[1] / 2.0
    top_corners = np.array(
        [
            surface_center + sx * surface.rotation[:, 0] + sy * surface.rotation[:, 1]
            for sx in (-half_x, half_x)
            for sy in (-half_y, half_y)
        ]
    )
    return CollisionBox(
        center=center,
        surface_center=surface_center,
        size=size,
        rotation=surface.rotation,
        quaternion=surface.quaternion,
        roi_corners=surface.corners,
        top_corners=top_corners,
    )


class StableTableTracker:
    """Accept fresh pose samples and decide when the Planning Scene needs a write."""

    def __init__(
        self,
        expected_frame,
        stable_plane_frames,
        max_height_variation,
        max_normal_angle_deg,
        update_height_threshold,
        update_angle_threshold_deg,
        max_table_plane_age,
    ):
        self.expected_frame = str(expected_frame)
        self.stable_plane_frames = int(stable_plane_frames)
        self.max_height_variation = float(max_height_variation)
        self.max_normal_angle_deg = float(max_normal_angle_deg)
        self.update_height_threshold = float(update_height_threshold)
        self.update_angle_threshold_deg = float(update_angle_threshold_deg)
        self.max_table_plane_age = float(max_table_plane_age)
        if not self.expected_frame:
            raise ValueError("expected frame must not be empty")
        if self.stable_plane_frames < 1:
            raise ValueError("stable_plane_frames must be positive")
        thresholds = (
            self.max_height_variation,
            self.max_normal_angle_deg,
            self.update_height_threshold,
            self.update_angle_threshold_deg,
            self.max_table_plane_age,
        )
        if not np.isfinite(thresholds).all() or any(value < 0.0 for value in thresholds):
            raise ValueError("table stability thresholds must be finite and non-negative")
        if self.max_table_plane_age <= 0.0:
            raise ValueError("max_table_plane_age must be positive")
        self._samples = deque(maxlen=self.stable_plane_frames)
        self._last_input_stamp = None
        self.latest_stable_model = None
        self.scene_model = None

    def _reject(self, reason):
        return TableUpdateDecision(False, reason, self.latest_stable_model)

    def observe(self, sample_value: TablePoseSample, now_sec: float):
        if str(sample_value.frame_id) != self.expected_frame:
            return self._reject("wrong_frame")
        stamp = float(sample_value.stamp_sec)
        now = float(now_sec)
        height = float(sample_value.height_at_roi_center)
        if not np.isfinite((stamp, now, height)).all() or stamp <= 0.0:
            return self._reject("invalid")
        age = now - stamp
        if age < 0.0:
            return self._reject("future")
        if age > self.max_table_plane_age:
            return self._reject("stale")
        if self._last_input_stamp is not None and stamp <= self._last_input_stamp:
            return self._reject("out_of_order")
        try:
            normal = _normalized_upward(sample_value.normal)
        except ValueError:
            return self._reject("invalid")

        if (
            self._last_input_stamp is not None
            and stamp - self._last_input_stamp > self.max_table_plane_age
        ):
            self._samples.clear()
        self._last_input_stamp = stamp
        self._samples.append((stamp, height, normal))
        if len(self._samples) < self.stable_plane_frames:
            return self._reject("collecting")

        heights = np.array([item[1] for item in self._samples], dtype=np.float64)
        if float(heights.max() - heights.min()) > self.max_height_variation:
            return self._reject("unstable_height")
        normals = [item[2] for item in self._samples]
        maximum_angle = max(
            (_normal_angle_deg(first, second) for first, second in combinations(normals, 2)),
            default=0.0,
        )
        if maximum_angle > self.max_normal_angle_deg:
            return self._reject("unstable_normal")

        representative_normal = _normalized_upward(np.mean(normals, axis=0))
        candidate = StableTableModel(
            frame_id=self.expected_frame,
            stamp_sec=max(item[0] for item in self._samples),
            normal=representative_normal,
            height_at_roi_center=float(np.median(heights)),
        )
        self.latest_stable_model = candidate
        if self.scene_model is None:
            return TableUpdateDecision(True, "first_stable_model", candidate)

        height_change = abs(
            candidate.height_at_roi_center - self.scene_model.height_at_roi_center
        )
        angle_change = _normal_angle_deg(candidate.normal, self.scene_model.normal)
        if (
            height_change > self.update_height_threshold
            or angle_change > self.update_angle_threshold_deg
        ):
            return TableUpdateDecision(True, "model_changed", candidate)
        return TableUpdateDecision(False, "below_update_threshold", candidate)

    def confirm_scene_update(self, model: StableTableModel):
        if model is None or model.frame_id != self.expected_frame:
            raise ValueError("confirmed scene model must use the expected frame")
        self.scene_model = model

    def status(self, now_sec: float):
        if self.latest_stable_model is None:
            return TableModelStatus("waiting", None, self.scene_model is not None)
        age = max(0.0, float(now_sec) - self.latest_stable_model.stamp_sec)
        state = "fresh" if age <= self.max_table_plane_age else "stale"
        return TableModelStatus(state, age, self.scene_model is not None)
