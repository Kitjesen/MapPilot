"""Thread-safe RGB-D and pose synchronization for scene perception."""

from __future__ import annotations

import math
import threading
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, TypeAlias, cast

import numpy as np

from runtime.msgs.geometry import Transform
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from runtime.runtime_interface import map_frame_id, odom_frame_id
from runtime.tf import map_from_odom_transform_from_mapping


@dataclass(frozen=True)
class SynchronizedFrame:
    """One immutable selection of source messages and resolved map poses."""

    color: Image
    depth: Image
    intrinsics: CameraIntrinsics
    odometry: Odometry
    map_odom: Transform | None
    timestamp: float
    frame_id: str
    map_from_body: np.ndarray
    map_from_camera: np.ndarray

    @property
    def color_ts(self) -> float:
        """Compatibility spelling for code that names the source stream."""

        return self.timestamp


@dataclass(frozen=True)
class FrameDrop:
    """A finalized sample that cannot produce a trustworthy observation."""

    reason: str
    timestamp: float


FrameResult: TypeAlias = SynchronizedFrame | FrameDrop


@dataclass(frozen=True)
class _PoseSelection:
    odometry: Odometry
    map_odom: Transform | None
    map_from_body: np.ndarray


class FrameSynchronizer:
    """Pair RGB-D samples and resolve their closest pose into ``map``."""

    _BUFFER_LIMITS = {
        "color": 8,
        "depth": 8,
        "odometry": 64,
        "map_odom": 16,
    }

    def __init__(
        self,
        *,
        camera_to_body: np.ndarray,
        skip_frames: int,
        max_rgbd_skew_s: float,
        max_odom_age_s: float,
        max_map_odom_age_s: float,
        output_frame_id: str = "map",
    ) -> None:
        self._camera_to_body = _readonly_matrix(camera_to_body)
        if output_frame_id != map_frame_id():
            raise ValueError(f"output_frame_id must be {map_frame_id()!r}")
        self._skip_frames = max(1, int(skip_frames))
        self._max_rgbd_skew_s = _time_window("max_rgbd_skew_s", max_rgbd_skew_s)
        self._max_odom_age_s = _time_window("max_odom_age_s", max_odom_age_s)
        self._max_map_odom_age_s = _time_window(
            "max_map_odom_age_s", max_map_odom_age_s
        )
        self._output_frame_id = output_frame_id
        self._lock = threading.RLock()
        self._colors: list[Image] = []
        self._depths: list[Image] = []
        self._odometry: list[Odometry] = []
        self._map_odom: list[Transform] = []
        self._intrinsics: CameraIntrinsics | None = None
        self._depth_watermark = -math.inf
        self._odometry_watermark = -math.inf
        self._map_odom_watermark = -math.inf
        self._color_frames = 0
        self._matched_frames = 0
        self._dropped_frames = 0
        self._last_drop_reason = ""
        self._drop_reasons: dict[str, int] = {}

    def push_color(self, color: Image) -> tuple[FrameResult, ...]:
        with self._lock:
            self._color_frames += 1
            if self._color_frames % self._skip_frames:
                return ()
            if not _timestamp_is_valid(color.ts):
                return (self._drop("invalid_sample_timestamp", 0.0),)
            if not _color_image_is_valid(color):
                return (self._drop("invalid_color_image", float(color.ts)),)
            self._colors.append(color)
            return self._drain_ready() + self._prune_buffers()

    def push_depth(self, depth: Image) -> tuple[FrameResult, ...]:
        with self._lock:
            if not _timestamp_is_valid(depth.ts):
                return (self._drop("invalid_sample_timestamp", 0.0),)
            if not _depth_image_is_valid(depth):
                return (self._drop("invalid_depth_image", float(depth.ts)),)
            self._depths.append(depth)
            self._depth_watermark = max(self._depth_watermark, float(depth.ts))
            return self._drain_ready() + self._prune_buffers()

    def push_camera_info(self, intrinsics: CameraIntrinsics) -> tuple[FrameResult, ...]:
        with self._lock:
            self._intrinsics = intrinsics
            return self._drain_ready()

    def push_odometry(self, odometry: Odometry) -> tuple[FrameResult, ...]:
        with self._lock:
            if not _timestamp_is_valid(odometry.ts):
                return (self._drop("invalid_odometry_timestamp", 0.0),)
            self._odometry.append(odometry)
            self._odometry_watermark = max(self._odometry_watermark, float(odometry.ts))
            return self._drain_ready() + self._prune_buffers()

    def push_map_odom(self, value: Mapping[str, Any]) -> tuple[FrameResult, ...]:
        with self._lock:
            transform = map_from_odom_transform_from_mapping(value)
            if transform is None:
                return (self._drop("invalid_map_odom_transform", _mapping_timestamp(value)),)
            self._map_odom.append(transform)
            self._map_odom_watermark = max(self._map_odom_watermark, float(transform.ts))
            return self._drain_ready() + self._prune_buffers()

    def health(self) -> dict[str, Any]:
        with self._lock:
            return {
                "color_frames": self._color_frames,
                "matched_frames": self._matched_frames,
                "dropped_frames": self._dropped_frames,
                "last_drop_reason": self._last_drop_reason,
                "drop_reasons": dict(self._drop_reasons),
                "buffer_sizes": {
                    "color": len(self._colors),
                    "depth": len(self._depths),
                    "odometry": len(self._odometry),
                    "map_odom": len(self._map_odom),
                },
            }

    def _drain_ready(self) -> tuple[FrameResult, ...]:
        results: list[FrameResult] = []

        # Exact matches are globally preferred.  Without this pass, an earlier
        # colour could consume a depth frame that belongs exactly to a later one.
        if self._intrinsics is not None:
            for color in sorted(tuple(self._colors), key=lambda sample: sample.ts):
                depth_index = next(
                    (
                        index
                        for index, depth in enumerate(self._depths)
                        if math.isclose(color.ts, depth.ts, rel_tol=0.0, abs_tol=1e-9)
                    ),
                    None,
                )
                if depth_index is None:
                    continue
                depth = self._depths[depth_index]
                result = self._prepare_frame(color, depth)
                if result is None:
                    continue
                _pop_identity(self._colors, color)
                self._depths.pop(depth_index)
                results.append(result)

        if not math.isfinite(self._depth_watermark):
            return tuple(results)

        for color in sorted(tuple(self._colors), key=lambda sample: sample.ts):
            if self._depth_watermark < float(color.ts):
                continue
            candidates = [
                (abs(float(depth.ts) - float(color.ts)), index, depth)
                for index, depth in enumerate(self._depths)
                if abs(float(depth.ts) - float(color.ts)) <= self._max_rgbd_skew_s
            ]
            if not candidates:
                _pop_identity(self._colors, color)
                results.append(self._drop("rgb_depth_time_skew", float(color.ts)))
                continue
            if self._intrinsics is None:
                continue
            _, depth_index, depth = min(candidates, key=lambda item: (item[0], item[2].ts))
            result = self._prepare_frame(color, depth)
            if result is None:
                continue
            _pop_identity(self._colors, color)
            self._depths.pop(depth_index)
            results.append(result)
        return tuple(results)

    def _prepare_frame(self, color: Image, depth: Image) -> FrameResult | None:
        assert self._intrinsics is not None
        color_frame = str(color.frame_id or "")
        depth_frame = str(depth.frame_id or "")
        info_frame = str(self._intrinsics.frame_id or "")
        if color_frame and depth_frame and color_frame != depth_frame:
            return self._drop("rgb_depth_frame_mismatch", float(color.ts))
        if info_frame and any(
            camera_frame and camera_frame != info_frame
            for camera_frame in (color_frame, depth_frame)
        ):
            return self._drop("camera_info_frame_mismatch", float(color.ts))
        if not _intrinsics_are_valid(self._intrinsics, color=color, depth=depth):
            return self._drop("invalid_camera_intrinsics", float(color.ts))

        if not _timestamp_is_valid(color.ts) or not _timestamp_is_valid(depth.ts):
            return self._drop("invalid_sample_timestamp", _valid_timestamp_or_zero(color.ts))
        selection = self._select_pose(float(color.ts))
        if selection is None:
            return None
        if isinstance(selection, FrameDrop):
            return selection

        odometry = selection.odometry
        map_odom = selection.map_odom
        map_from_body = selection.map_from_body
        self._matched_frames += 1
        self._last_drop_reason = ""
        map_from_body = _readonly_matrix(map_from_body)
        map_from_camera = _readonly_matrix(map_from_body @ self._camera_to_body)
        return SynchronizedFrame(
            color=color,
            depth=depth,
            intrinsics=self._intrinsics,
            odometry=odometry,
            map_odom=map_odom,
            timestamp=float(color.ts),
            frame_id=self._output_frame_id,
            map_from_body=map_from_body,
            map_from_camera=map_from_camera,
        )

    def _select_pose(self, timestamp: float) -> _PoseSelection | FrameDrop | None:
        if not self._odometry:
            return None
        odometry = min(self._odometry, key=lambda sample: abs(sample.ts - timestamp))
        if not _timestamp_is_valid(odometry.ts):
            return self._drop("invalid_odometry_timestamp", timestamp)
        if abs(float(odometry.ts) - timestamp) > self._max_odom_age_s:
            if self._odometry_watermark <= timestamp + self._max_odom_age_s:
                return None
            return self._drop("odom_time_skew", timestamp)
        try:
            source = _pose_matrix(odometry)
        except (TypeError, ValueError, ZeroDivisionError):
            return self._drop("invalid_odometry_pose", timestamp)
        map_odom: Transform | None = None
        if odometry.frame_id == self._output_frame_id:
            map_from_body = source
        elif odometry.frame_id == odom_frame_id():
            if not self._map_odom:
                if self._map_odom_watermark <= timestamp + self._max_map_odom_age_s:
                    return None
                return self._drop("missing_map_odom_transform", timestamp)
            map_odom = min(self._map_odom, key=lambda sample: abs(sample.ts - timestamp))
            if abs(float(map_odom.ts) - timestamp) > self._max_map_odom_age_s:
                if self._map_odom_watermark <= timestamp + self._max_map_odom_age_s:
                    return None
                return self._drop("map_odom_time_skew", timestamp)
            map_from_body = map_odom.to_matrix() @ source
        else:
            return self._drop("odometry_frame_unsupported", timestamp)
        return _PoseSelection(
            odometry=odometry,
            map_odom=map_odom,
            map_from_body=_readonly_matrix(map_from_body),
        )

    def _prune_buffers(self) -> tuple[FrameResult, ...]:
        results: list[FrameResult] = []
        for samples, watermark, max_age, reason in (
            (
                self._depths,
                self._depth_watermark,
                self._max_rgbd_skew_s,
                "depth_expired",
            ),
            (
                self._odometry,
                self._odometry_watermark,
                self._max_odom_age_s,
                "odometry_expired",
            ),
            (
                self._map_odom,
                self._map_odom_watermark,
                self._max_map_odom_age_s,
                "map_odom_expired",
            ),
        ):
            results.extend(
                self._expire_support(
                    samples,
                    watermark=watermark,
                    max_age=max_age,
                    reason=reason,
                )
            )
        for name, samples in (
            ("color", self._colors),
            ("depth", self._depths),
            ("odometry", self._odometry),
            ("map_odom", self._map_odom),
        ):
            limit = self._BUFFER_LIMITS[name]
            while len(samples) > limit:
                evicted = min(samples, key=lambda sample: float(sample.ts))
                _pop_identity(samples, evicted)
                results.append(
                    self._drop(
                        f"{name}_buffer_overflow",
                        float(cast(Any, evicted).ts),
                    )
                )
        return tuple(results)

    def _expire_support(
        self,
        samples: list[Any],
        *,
        watermark: float,
        max_age: float,
        reason: str,
    ) -> list[FrameDrop]:
        if not math.isfinite(watermark):
            return []
        cutoff = watermark - max_age
        expired: list[FrameDrop] = []
        for sample in tuple(samples):
            timestamp = float(sample.ts)
            if timestamp >= cutoff:
                continue
            if any(abs(float(color.ts) - timestamp) <= max_age for color in self._colors):
                continue
            _pop_identity(samples, sample)
            expired.append(self._drop(reason, timestamp))
        return expired

    def _drop(self, reason: str, timestamp: float) -> FrameDrop:
        self._dropped_frames += 1
        self._last_drop_reason = reason
        self._drop_reasons[reason] = self._drop_reasons.get(reason, 0) + 1
        return FrameDrop(reason=reason, timestamp=timestamp)


def _pose_matrix(odometry: Odometry) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = odometry.pose.orientation.normalize().to_rotation_matrix()
    matrix[:3, 3] = [odometry.pose.x, odometry.pose.y, odometry.pose.z]
    return cast(np.ndarray, matrix)


def _intrinsics_are_valid(
    intrinsics: CameraIntrinsics,
    *,
    color: Image,
    depth: Image,
) -> bool:
    if not math.isfinite(float(intrinsics.fx)) or float(intrinsics.fx) <= 0.0:
        return False
    if not math.isfinite(float(intrinsics.fy)) or float(intrinsics.fy) <= 0.0:
        return False
    if not np.isfinite(intrinsics.K_matrix).all():
        return False
    if not np.isfinite(intrinsics.D_vector).all():
        return False
    expected_shape = (int(intrinsics.height), int(intrinsics.width))
    return bool(color.data.shape[:2] == expected_shape == depth.data.shape[:2])


def _color_image_is_valid(image: Image) -> bool:
    ndim = int(getattr(image.data, "ndim", -1))
    if image.height <= 0 or image.width <= 0:
        return False
    if image.format in {ImageFormat.BGR, ImageFormat.RGB}:
        return ndim == 3 and image.channels == 3
    if image.format is ImageFormat.RGBA:
        return ndim == 3 and image.channels == 4
    return image.format is ImageFormat.GRAY and ndim == 2


def _depth_image_is_valid(image: Image) -> bool:
    return bool(
        image.format in {ImageFormat.DEPTH_U16, ImageFormat.DEPTH_F32}
        and int(getattr(image.data, "ndim", -1)) == 2
        and image.height > 0
        and image.width > 0
    )


def _readonly_matrix(value: np.ndarray) -> np.ndarray:
    matrix = np.array(value, dtype=np.float64, copy=True)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("transform matrix must be finite and 4x4")
    matrix.flags.writeable = False
    return cast(np.ndarray, matrix)


def _timestamp_is_valid(value: Any) -> bool:
    try:
        timestamp = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(timestamp) and timestamp > 0.0


def _time_window(name: str, value: Any) -> float:
    try:
        window = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite non-negative number") from exc
    if not math.isfinite(window) or window < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number")
    return window


def _valid_timestamp_or_zero(value: Any) -> float:
    return float(value) if _timestamp_is_valid(value) else 0.0


def _mapping_timestamp(value: Mapping[str, Any]) -> float:
    try:
        timestamp = float(value.get("stamp_s", value.get("ts", 0.0)))
    except (TypeError, ValueError):
        return 0.0
    return timestamp if math.isfinite(timestamp) and timestamp > 0.0 else 0.0


def _pop_identity(samples: list[Any], target: Any) -> None:
    for index, sample in enumerate(samples):
        if sample is target:
            samples.pop(index)
            return
