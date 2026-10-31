"""Observed camera coverage for native, geometry-checked search proposals."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import SemanticObservation


def camera_view(observation: SemanticObservation) -> dict[str, float]:
    """Project the calibrated optical frustum into the native proposal plane.

    Coverage ranks where to observe next. It never establishes free space,
    ground support or motion feasibility; those stay with native 3D navigation.
    """
    info, depth = observation.camera_info, observation.depth
    transform = np.asarray(observation.map_from_camera, dtype=float)
    if info is None or depth is None or transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("search_camera_geometry_unavailable")
    if not math.isfinite(info.fx) or info.fx <= 0 or not 0 < info.cx < info.width:
        raise ValueError("search_camera_intrinsics_unavailable")
    rotation = transform[:3, :3]
    forward = rotation @ np.array([0.0, 0.0, 1.0])
    horizontal = math.hypot(float(forward[0]), float(forward[1]))
    if horizontal < 0.1:
        raise ValueError("search_camera_has_no_horizontal_view")
    left = rotation @ np.array([-info.cx / info.fx, 0.0, 1.0])
    right = rotation @ np.array([(info.width - info.cx) / info.fx, 0.0, 1.0])
    angle = math.atan2(float(left[1]), float(left[0])) - math.atan2(float(right[1]), float(right[0]))
    fov = abs(math.atan2(math.sin(angle), math.cos(angle)))
    if fov <= 0:
        raise ValueError("search_camera_has_no_horizontal_view")
    data = np.asarray(depth.data)
    if data.ndim != 2:
        raise ValueError("search_depth_unavailable")
    sample = data[::16, ::16].astype(float)
    if data.dtype.kind in "ui":
        sample *= info.depth_scale
    valid = sample[np.isfinite(sample) & (sample > 0)]
    if not valid.size:
        raise ValueError("search_depth_unavailable")
    distance = float(np.percentile(valid, 90)) * horizontal
    return {
        "x": float(transform[0, 3]), "y": float(transform[1, 3]),
        "yaw": math.atan2(float(forward[1]), float(forward[0])),
        "range_m": distance, "horizontal_fov_rad": fov,
    }


@dataclass
class SearchHistory:
    """Coverage and proposal bookkeeping for one existing instruction epoch."""

    instruction: str
    instruction_epoch: int
    navigation_context: tuple[str, str, int]
    observation_boundary: tuple[int, str, int]
    frame_epoch: int = 0
    reference_z: float = 0.0
    views: list[dict[str, float]] = field(default_factory=list)
    rejected: list[tuple[float, float, float, float]] = field(default_factory=list)
    pending: bool = False
    moves: int = 0
    after_ts: float = 0.0
    interrupting: bool = False
    retry_after: float = 0.0

    def observe(self, view: dict[str, float]) -> None:
        if self.views:
            old = self.views[-1]
            angle = view["yaw"] - old["yaw"]
            if (math.hypot(view["x"] - old["x"], view["y"] - old["y"]) < 0.30
                    and abs(math.atan2(math.sin(angle), math.cos(angle))) < math.radians(15)):
                return
        if len(self.views) < 256:
            self.views.append(dict(view))

    def excludes(self, pose: tuple[float, float, float, float]) -> bool:
        return any(math.dist(pose[:3], old[:3]) < 0.15
                   and abs(math.atan2(math.sin(pose[3] - old[3]), math.cos(pose[3] - old[3]))) < 0.15
                   for old in self.rejected)
