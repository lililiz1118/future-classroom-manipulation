"""Pure detection parsing and timestamped point-cloud caching."""

from collections import deque
from dataclasses import dataclass
import json
import math
import threading
from typing import Any, Deque, Optional, Sequence, Tuple

import numpy as np


NSEC_PER_SEC = 1_000_000_000


@dataclass(frozen=True)
class BoundingBox:
    xmin: int
    ymin: int
    xmax: int
    ymax: int


@dataclass(frozen=True)
class Detection:
    stamp_ns: int
    frame_id: str
    class_name: str
    confidence: float
    bbox: BoundingBox


@dataclass(frozen=True)
class CloudSample:
    stamp_ns: int
    message: Any


@dataclass(frozen=True)
class CloudMatch:
    stamp_ns: int
    message: Any
    delta_ns: int


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    k: Tuple[float, ...]
    d: Tuple[float, ...]
    distortion_model: str

    def __init__(self, width, height, k, d, distortion_model):
        object.__setattr__(self, "width", int(width))
        object.__setattr__(self, "height", int(height))
        object.__setattr__(self, "k", tuple(float(value) for value in k))
        object.__setattr__(self, "d", tuple(float(value) for value in d))
        object.__setattr__(self, "distortion_model", str(distortion_model))
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera image dimensions must be positive")
        if len(self.k) != 9 or not np.isfinite(self.k).all():
            raise ValueError("camera K must contain nine finite values")
        if self.distortion_model not in ("", "plumb_bob"):
            raise ValueError("only plumb_bob camera distortion is supported")
        if len(self.d) not in (0, 4, 5) or not np.isfinite(self.d).all():
            raise ValueError("plumb_bob D must contain zero, four, or five finite values")


@dataclass(frozen=True)
class TargetSelection:
    points: np.ndarray
    colors: np.ndarray
    projected_count: int
    target_count: int


def stamp_parts_to_ns(secs: int, nsecs: int) -> int:
    """Convert a ROS stamp to exact integer nanoseconds."""
    if isinstance(secs, bool) or not isinstance(secs, int) or secs < 0:
        raise ValueError("stamp secs must be a non-negative integer")
    if (
        isinstance(nsecs, bool)
        or not isinstance(nsecs, int)
        or not 0 <= nsecs < NSEC_PER_SEC
    ):
        raise ValueError("stamp nsecs must be an integer in [0, 1000000000)")
    return secs * NSEC_PER_SEC + nsecs


def _required_mapping(parent, key):
    value = parent.get(key)
    if not isinstance(value, dict):
        raise ValueError("%s must be an object" % key)
    return value


def _required_int(parent, key):
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("%s must be an integer" % key)
    return value


def parse_detection_json(payload: str) -> Detection:
    """Parse the current `/yolo_world/target_detection` JSON contract."""
    try:
        document = json.loads(payload)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError("detection payload must be valid JSON") from exc
    if not isinstance(document, dict):
        raise ValueError("detection payload must be an object")

    header = _required_mapping(document, "header")
    stamp = _required_mapping(header, "stamp")
    stamp_ns = stamp_parts_to_ns(
        _required_int(stamp, "secs"),
        _required_int(stamp, "nsecs"),
    )
    frame_id = header.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError("header frame_id must be a non-empty string")

    bbox_data = _required_mapping(document, "bbox")
    bbox = BoundingBox(
        xmin=_required_int(bbox_data, "xmin"),
        ymin=_required_int(bbox_data, "ymin"),
        xmax=_required_int(bbox_data, "xmax"),
        ymax=_required_int(bbox_data, "ymax"),
    )
    if bbox.xmin > bbox.xmax or bbox.ymin > bbox.ymax:
        raise ValueError("bbox minimums must not exceed maximums")

    class_name = document.get("class_name")
    if not isinstance(class_name, str) or not class_name:
        raise ValueError("class_name must be a non-empty string")
    confidence = document.get("confidence")
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
        raise ValueError("confidence must be numeric")
    confidence = float(confidence)
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence must be finite and in [0, 1]")

    return Detection(
        stamp_ns=stamp_ns,
        frame_id=frame_id,
        class_name=class_name,
        confidence=confidence,
        bbox=bbox,
    )


class TimedCloudCache:
    """Thread-safe ROS-message cache pruned by acquisition-time duration."""

    def __init__(self, duration_sec: float):
        duration = float(duration_sec)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("cloud cache duration must be finite and positive")
        self._duration_ns = int(round(duration * NSEC_PER_SEC))
        self._samples: Deque[CloudSample] = deque()
        self._newest_stamp_ns: Optional[int] = None
        self._lock = threading.Lock()

    def add(self, stamp_ns: int, message: Any) -> None:
        stamp_ns = int(stamp_ns)
        with self._lock:
            self._samples.append(CloudSample(stamp_ns, message))
            if self._newest_stamp_ns is None or stamp_ns > self._newest_stamp_ns:
                self._newest_stamp_ns = stamp_ns
            cutoff = self._newest_stamp_ns - self._duration_ns
            self._samples = deque(
                sample for sample in self._samples if sample.stamp_ns >= cutoff
            )

    @property
    def count(self) -> int:
        with self._lock:
            return len(self._samples)

    def latest(self) -> Optional[CloudSample]:
        with self._lock:
            if not self._samples:
                return None
            return max(self._samples, key=lambda sample: sample.stamp_ns)

    def nearest(self, stamp_ns: int, max_delta_sec: float) -> Optional[CloudMatch]:
        max_delta = float(max_delta_sec)
        if not math.isfinite(max_delta) or max_delta < 0.0:
            raise ValueError("max stamp delta must be finite and non-negative")
        max_delta_ns = int(round(max_delta * NSEC_PER_SEC))
        with self._lock:
            if not self._samples:
                return None
            nearest = min(
                self._samples,
                key=lambda sample: (abs(sample.stamp_ns - stamp_ns), -sample.stamp_ns),
            )
            delta_ns = abs(nearest.stamp_ns - int(stamp_ns))
            if delta_ns > max_delta_ns:
                return None
            return CloudMatch(nearest.stamp_ns, nearest.message, delta_ns)


def project_points(
    points_color: np.ndarray,
    camera: CameraModel,
) -> Tuple[np.ndarray, np.ndarray]:
    """Project color-optical-frame XYZ into the raw color image."""
    points = np.asarray(points_color, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_color must have shape (N, 3)")

    pixels = np.full((points.shape[0], 2), np.nan, dtype=np.float64)
    usable = np.isfinite(points).all(axis=1) & (points[:, 2] > 0.0)
    indices = np.flatnonzero(usable)
    if indices.size == 0:
        return pixels, usable

    selected = points[indices]
    x = selected[:, 0] / selected[:, 2]
    y = selected[:, 1] / selected[:, 2]
    coefficients = np.zeros(5, dtype=np.float64)
    if camera.d:
        coefficients[: len(camera.d)] = camera.d
    k1, k2, p1, p2, k3 = coefficients
    radius2 = x * x + y * y
    radial = 1.0 + k1 * radius2 + k2 * radius2**2 + k3 * radius2**3
    x_distorted = x * radial + 2.0 * p1 * x * y + p2 * (radius2 + 2.0 * x * x)
    y_distorted = y * radial + p1 * (radius2 + 2.0 * y * y) + 2.0 * p2 * x * y

    fx, skew, cx = camera.k[0], camera.k[1], camera.k[2]
    fy, cy = camera.k[4], camera.k[5]
    u = fx * x_distorted + skew * y_distorted + cx
    v = fy * y_distorted + cy
    pixels[indices, 0] = u
    pixels[indices, 1] = v

    in_image = (
        np.isfinite(u)
        & np.isfinite(v)
        & (u >= 0.0)
        & (u < float(camera.width))
        & (v >= 0.0)
        & (v < float(camera.height))
    )
    valid = np.zeros(points.shape[0], dtype=bool)
    valid[indices] = in_image
    return pixels, valid


def select_bbox_points(
    source_points: np.ndarray,
    colors: np.ndarray,
    color_points: np.ndarray,
    camera: CameraModel,
    bbox: BoundingBox,
) -> TargetSelection:
    """Select source-frame XYZ/RGB whose color projection is inside a bbox."""
    source = np.asarray(source_points, dtype=np.float32)
    color_xyz = np.asarray(color_points, dtype=np.float32)
    color_values = np.asarray(colors, dtype=np.float32)
    for name, values in (
        ("source_points", source),
        ("color_points", color_xyz),
        ("colors", color_values),
    ):
        if values.ndim != 2 or values.shape[1] != 3:
            raise ValueError("%s must have shape (N, 3)" % name)
    if not source.shape[0] == color_xyz.shape[0] == color_values.shape[0]:
        raise ValueError("source XYZ, color-frame XYZ, and RGB must stay aligned")

    pixels, projected_mask = project_points(color_xyz, camera)
    bbox_mask = projected_mask.copy()
    bbox_mask &= pixels[:, 0] >= bbox.xmin
    bbox_mask &= pixels[:, 0] <= bbox.xmax
    bbox_mask &= pixels[:, 1] >= bbox.ymin
    bbox_mask &= pixels[:, 1] <= bbox.ymax
    return TargetSelection(
        points=source[bbox_mask].astype(np.float32, copy=False),
        colors=color_values[bbox_mask].astype(np.float32, copy=False),
        projected_count=int(np.count_nonzero(projected_mask)),
        target_count=int(np.count_nonzero(bbox_mask)),
    )


def parse_cloud_arrays(message) -> Tuple[np.ndarray, np.ndarray]:
    """Parse XYZ and packed RGB from organized or unorganized PointCloud2 data."""
    fields = {field.name: field for field in message.fields}
    missing = sorted({"x", "y", "z", "rgb"} - set(fields))
    if missing:
        raise ValueError("PointCloud2 missing fields: " + ", ".join(missing))
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype != 7 or field.count != 1:
            raise ValueError("PointCloud2 %s must be scalar FLOAT32" % name)
    rgb_field = fields["rgb"]
    if rgb_field.datatype not in (6, 7) or rgb_field.count != 1:
        raise ValueError("PointCloud2 rgb must be scalar packed FLOAT32 or UINT32")

    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    if width < 0 or height < 0 or point_step <= 0 or row_step < width * point_step:
        raise ValueError("PointCloud2 has invalid dimensions or strides")
    required_size = row_step * height
    if len(message.data) < required_size:
        raise ValueError("PointCloud2 data is shorter than its dimensions require")
    for name in ("x", "y", "z", "rgb"):
        if fields[name].offset < 0 or fields[name].offset + 4 > point_step:
            raise ValueError("PointCloud2 %s field exceeds point_step" % name)

    byte_order = ">" if message.is_bigendian else "<"
    rgb_format = byte_order + ("f4" if rgb_field.datatype == 7 else "u4")
    dtype = np.dtype(
        {
            "names": ["x", "y", "z", "rgb"],
            "formats": [byte_order + "f4"] * 3 + [rgb_format],
            "offsets": [fields[name].offset for name in ("x", "y", "z", "rgb")],
            "itemsize": point_step,
        }
    )
    records = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=message.data,
        strides=(row_step, point_step),
    )
    points = np.column_stack(
        tuple(records[name].reshape(-1) for name in ("x", "y", "z"))
    ).astype(np.float32, copy=False)
    rgb_values = records["rgb"].reshape(-1)
    if rgb_field.datatype == 7:
        rgb_values = rgb_values.view(np.dtype(byte_order + "u4"))
    packed_rgb = rgb_values.astype(np.uint32, copy=False)
    return points, packed_rgb


def pack_colors_to_float32(colors: np.ndarray) -> np.ndarray:
    """Pack normalized RGB rows into the bit pattern ROS stores as FLOAT32."""
    values = np.asarray(colors, dtype=np.float32)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("colors must have shape (N, 3)")
    if not np.isfinite(values).all():
        raise ValueError("colors must be finite")
    channels = np.clip(np.rint(values * 255.0), 0.0, 255.0).astype(np.uint32)
    packed = (channels[:, 0] << 16) | (channels[:, 1] << 8) | channels[:, 2]
    return packed.astype(np.uint32, copy=False).view(np.float32)
