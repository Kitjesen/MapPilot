"""Ground-robot feasibility checks for global planner paths.

This module evaluates geometric path output only. It does not inspect live
costmaps, robot state, or planner internals; integration code can combine this
report with planner diagnostics later.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass, fields, replace
from typing import Any


@dataclass(frozen=True)
class GroundPathFeasibilityConfig:
    """Thresholds for conservative ground-path feasibility checks."""

    min_point_count: int = 2
    max_z_range_m: float = 2.0
    max_segment_dz_m: float = 1.0
    max_slope: float = 0.7
    slope_xy_epsilon_m: float = 1e-6

    def to_dict(self) -> dict[str, Any]:
        """Return JSON-friendly threshold values."""
        return asdict(self)


@dataclass
class GroundPathFeasibilityReport:
    """Geometric path metrics plus a ground-executable verdict."""

    point_count: int
    path_length_3d: float
    path_length_xy: float
    z_min: float | None
    z_max: float | None
    z_range: float | None
    max_segment_dz: float
    max_slope: float
    ground_executable: bool
    reasons: list[str]
    thresholds: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-friendly dictionary report."""
        return {
            "point_count": self.point_count,
            "path_length_3d": self.path_length_3d,
            "path_length_xy": self.path_length_xy,
            "z_min": self.z_min,
            "z_max": self.z_max,
            "z_range": self.z_range,
            "max_segment_dz": self.max_segment_dz,
            "max_slope": self.max_slope,
            "ground_executable": self.ground_executable,
            "reasons": list(self.reasons),
            "thresholds": dict(self.thresholds),
        }

    as_dict = to_dict


def evaluate_ground_path(
    path: Any,
    config: GroundPathFeasibilityConfig | Mapping[str, Any] | None = None,
) -> GroundPathFeasibilityReport:
    """Evaluate whether a global path is plausible for a ground robot.

    Args:
        path: Iterable of points. Points may be sequences ``(x, y[, z])``,
            dicts with ``x``/``y``/optional ``z``, or objects with
            ``x``/``y``/optional ``z`` or ``position`` attributes.
        config: Optional threshold dataclass or dict override.

    Returns:
        GroundPathFeasibilityReport containing required metrics and reasons.
    """

    cfg = _resolve_config(config)
    raw_points = _materialize_path(path)
    points, invalid_point_count, non_finite_point_count = _normalize_points(raw_points)

    metrics = _measure_path(points, cfg)
    reasons: list[str] = []

    if len(raw_points) < int(cfg.min_point_count):
        reasons.append("too_few_points")
    if invalid_point_count:
        reasons.append("invalid_point")
    if non_finite_point_count:
        reasons.append("non_finite_point")

    z_range = metrics["z_range"]
    if z_range is not None and _exceeds(float(z_range), cfg.max_z_range_m):
        reasons.append("z_range_too_large")
    if _exceeds(metrics["max_segment_dz"], cfg.max_segment_dz_m):
        reasons.append("segment_dz_too_large")
    max_slope = metrics["max_slope"]
    if (not math.isfinite(max_slope)) or _exceeds(max_slope, cfg.max_slope):
        reasons.append("segment_slope_too_large")

    return GroundPathFeasibilityReport(
        point_count=len(raw_points),
        path_length_3d=metrics["path_length_3d"],
        path_length_xy=metrics["path_length_xy"],
        z_min=metrics["z_min"],
        z_max=metrics["z_max"],
        z_range=z_range,
        max_segment_dz=metrics["max_segment_dz"],
        max_slope=max_slope,
        ground_executable=not reasons,
        reasons=reasons,
        thresholds=cfg.to_dict(),
    )


def _resolve_config(
    config: GroundPathFeasibilityConfig | Mapping[str, Any] | None,
) -> GroundPathFeasibilityConfig:
    if config is None:
        return GroundPathFeasibilityConfig()
    if isinstance(config, GroundPathFeasibilityConfig):
        return config
    if isinstance(config, Mapping):
        valid_keys = {field.name for field in fields(GroundPathFeasibilityConfig)}
        unknown = sorted(str(key) for key in config.keys() if key not in valid_keys)
        if unknown:
            raise TypeError(f"unknown ground path feasibility config keys: {unknown}")
        return replace(GroundPathFeasibilityConfig(), **dict(config))
    raise TypeError("config must be GroundPathFeasibilityConfig, mapping, or None")


def _materialize_path(path: Any) -> list[Any]:
    if path is None:
        return []
    try:
        return list(path)
    except TypeError:
        return []


def _normalize_points(
    raw_points: list[Any],
) -> tuple[list[tuple[float, float, float]], int, int]:
    points: list[tuple[float, float, float]] = []
    invalid_point_count = 0
    non_finite_point_count = 0
    for raw in raw_points:
        try:
            point = _point_to_xyz(raw)
        except (KeyError, TypeError, ValueError):
            invalid_point_count += 1
            points.append((math.nan, math.nan, math.nan))
            continue
        if not all(math.isfinite(value) for value in point):
            non_finite_point_count += 1
        points.append(point)
    return points, invalid_point_count, non_finite_point_count


def _point_to_xyz(point: Any) -> tuple[float, float, float]:
    if point is None:
        raise ValueError("point is None")
    if isinstance(point, Mapping):
        if "position" in point:
            return _point_to_xyz(point["position"])
        return (
            float(point["x"]),
            float(point["y"]),
            float(point.get("z", 0.0)),
        )
    if hasattr(point, "pose"):
        return _point_to_xyz(point.pose)
    if hasattr(point, "position"):
        return _point_to_xyz(point.position)
    if hasattr(point, "x") and hasattr(point, "y"):
        return (
            float(point.x),
            float(point.y),
            float(getattr(point, "z", 0.0)),
        )

    values = list(point)
    if len(values) < 2:
        raise ValueError("point must contain at least x and y")
    z = values[2] if len(values) >= 3 else 0.0
    return (float(values[0]), float(values[1]), float(z))


def _measure_path(
    points: list[tuple[float, float, float]],
    config: GroundPathFeasibilityConfig,
) -> dict[str, Any]:
    finite_points = [point for point in points if all(math.isfinite(v) for v in point)]
    if finite_points:
        z_values = [point[2] for point in finite_points]
        z_min = min(z_values)
        z_max = max(z_values)
        z_range = z_max - z_min
    else:
        z_min = None
        z_max = None
        z_range = None

    path_length_xy = 0.0
    path_length_3d = 0.0
    max_segment_dz = 0.0
    max_slope = 0.0
    xy_epsilon = max(float(config.slope_xy_epsilon_m), 0.0)

    for start, end in zip(points, points[1:]):
        if not all(math.isfinite(value) for value in (*start, *end)):
            continue
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        dxy = math.hypot(dx, dy)
        abs_dz = abs(dz)
        path_length_xy += dxy
        path_length_3d += math.hypot(dxy, abs_dz)
        max_segment_dz = max(max_segment_dz, abs_dz)
        if dxy <= xy_epsilon:
            slope = math.inf if abs_dz > xy_epsilon else 0.0
        else:
            slope = abs_dz / dxy
        max_slope = max(max_slope, slope)

    return {
        "path_length_3d": path_length_3d,
        "path_length_xy": path_length_xy,
        "z_min": z_min,
        "z_max": z_max,
        "z_range": z_range,
        "max_segment_dz": max_segment_dz,
        "max_slope": max_slope,
    }


def _exceeds(value: float, threshold: float | None) -> bool:
    if threshold is None:
        return False
    return float(value) > float(threshold) + 1e-9
