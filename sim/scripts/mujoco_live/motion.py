"""Motion, obstacle, and point-cloud helpers for the MuJoCo live gate."""

from __future__ import annotations

import json
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Sequence

from runtime.msgs.numpy_compat import is_numpy_array, np
from runtime.runtime_interface import TOPICS, topic_default_frame_id

SIM_NAV_ODOMETRY_FRAME_ID = topic_default_frame_id(TOPICS.odometry)

def _relative_times_for_scan(
    point_count: int,
    lidar_period_s: float,
    *,
    scan_time_profile: str,
) -> np.ndarray:
    """Return per-point times matching how this validation scan was produced."""

    count = max(0, int(point_count))
    profile = str(scan_time_profile or "synthetic_rolling").strip().lower()
    if profile == "instantaneous":
        return np.zeros(count, dtype=np.float32)
    if profile == "synthetic_rolling":
        return np.linspace(
            0.0,
            float(lidar_period_s),
            num=count,
            endpoint=False,
            dtype=np.float32,
        )
    raise ValueError(f"unsupported scan_time_profile: {scan_time_profile}")

def _coerce_xyzi_cloud(points: Any, *, default_intensity: float = 100.0) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    if pts.size == 0:
        return np.zeros((0, 4), dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError(f"expected point cloud shape (N, >=3), got {pts.shape}")
    if pts.shape[1] < 4:
        intensity = np.full((len(pts), 1), float(default_intensity), dtype=np.float32)
        return np.hstack([pts[:, :3], intensity]).astype(np.float32, copy=False)
    return pts[:, :4].astype(np.float32, copy=False)

def _physical_rolling_scan_from_samples(
    samples: Sequence[tuple[float, np.ndarray, np.ndarray, int]],
    *,
    scan_start_s: float,
    scan_end_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, int]:
    """Build one scan from subscans captured at their actual simulation times."""

    start = float(scan_start_s)
    end = float(scan_end_s)
    eps = 1e-6
    sensor_chunks: list[np.ndarray] = []
    world_chunks: list[np.ndarray] = []
    time_chunks: list[np.ndarray] = []
    moving_point_count = 0
    selected_subscans = 0
    for sim_time_s, cloud_sensor, cloud_world, moving_count in samples:
        t = float(sim_time_s)
        if t < start - eps or t > end + eps:
            continue
        sensor_pts = np.asarray(cloud_sensor, dtype=np.float32)
        world_pts = np.asarray(cloud_world, dtype=np.float32)
        if sensor_pts.size == 0 or sensor_pts.ndim != 2 or sensor_pts.shape[1] < 4:
            continue
        if world_pts.size == 0 or world_pts.ndim != 2 or world_pts.shape[1] < 4:
            world_pts = np.zeros((len(sensor_pts), 4), dtype=np.float32)
        sensor_chunks.append(sensor_pts[:, :4])
        world_chunks.append(world_pts[:, :4])
        relative_t = max(0.0, min(float(end - start), t - start))
        time_chunks.append(np.full(len(sensor_pts), relative_t, dtype=np.float32))
        moving_point_count += max(0, int(moving_count))
        selected_subscans += 1
    if not sensor_chunks:
        return (
            np.zeros((0, 4), dtype=np.float32),
            np.zeros((0, 4), dtype=np.float32),
            np.zeros(0, dtype=np.float32),
            0,
            0,
        )
    return (
        np.vstack(sensor_chunks).astype(np.float32, copy=False),
        np.vstack(world_chunks).astype(np.float32, copy=False),
        np.concatenate(time_chunks).astype(np.float32, copy=False),
        int(moving_point_count),
        int(selected_subscans),
    )

def _area_growth(samples: list[float]) -> dict[str, float]:
    if not samples:
        return {"first_m2": 0.0, "last_m2": 0.0, "max_m2": 0.0, "growth_m2": 0.0}
    first = float(samples[0])
    last = float(samples[-1])
    max_area = float(max(samples))
    return {
        "first_m2": first,
        "last_m2": last,
        "max_m2": max_area,
        "growth_m2": max(0.0, max_area - first),
    }

def _coverage_growth(samples: list[dict[str, float]]) -> dict[str, float]:
    ratios: list[float] = []
    known_values: list[float] = []
    total_values: list[float] = []
    for sample in samples:
        known = float(sample.get("known_m2") or 0.0)
        unknown = float(sample.get("unknown_m2") or 0.0)
        total = max(0.0, known + unknown)
        ratio = known / total if total > 0.0 else 0.0
        ratios.append(float(ratio))
        known_values.append(float(known))
        total_values.append(float(total))
    if not ratios:
        return {
            "first_ratio": 0.0,
            "last_ratio": 0.0,
            "max_ratio": 0.0,
            "growth_ratio": 0.0,
            "first_known_m2": 0.0,
            "last_known_m2": 0.0,
            "first_total_m2": 0.0,
            "last_total_m2": 0.0,
        }
    first = ratios[0]
    max_ratio = max(ratios)
    return {
        "first_ratio": float(first),
        "last_ratio": float(ratios[-1]),
        "max_ratio": float(max_ratio),
        "growth_ratio": float(max(0.0, max_ratio - first)),
        "first_known_m2": float(known_values[0]),
        "last_known_m2": float(known_values[-1]),
        "first_total_m2": float(total_values[0]),
        "last_total_m2": float(total_values[-1]),
    }

def _box_surface_points(
    box: dict[str, Any],
    *,
    spacing: float,
    intensity: float,
) -> list[tuple[float, float, float, float]]:
    cx, cy, cz = [float(v) for v in box["position"][:3]]
    hx, hy, hz = [float(v) for v in box["half_size"][:3]]
    z = max(0.15, min(cz + hz * 0.35, 0.75))
    step = max(0.02, float(spacing))
    xs = np.arange(cx - hx, cx + hx + step * 0.5, step)
    ys = np.arange(cy - hy, cy + hy + step * 0.5, step)
    points: list[tuple[float, float, float, float]] = []
    thin_x = hx <= step * 0.75
    thin_y = hy <= step * 0.75
    if thin_x or thin_y:
        for x in xs:
            for y in ys:
                points.append((float(x), float(y), float(z), float(intensity)))
        return points
    for x in xs:
        points.append((float(x), float(cy - hy), float(z), float(intensity)))
        points.append((float(x), float(cy + hy), float(z), float(intensity)))
    for y in ys:
        points.append((float(cx - hx), float(y), float(z), float(intensity)))
        points.append((float(cx + hx), float(y), float(z), float(intensity)))
    return points

def _point_box_clearance(point: tuple[float, float], box: dict[str, Any]) -> float:
    cx, cy = [float(v) for v in box["position"][:2]]
    hx, hy = [float(v) for v in box["half_size"][:2]]
    dx = abs(float(point[0]) - cx) - hx
    dy = abs(float(point[1]) - cy) - hy
    outside_x = max(dx, 0.0)
    outside_y = max(dy, 0.0)
    if dx <= 0.0 and dy <= 0.0:
        return -min(-dx, -dy)
    return math.hypot(outside_x, outside_y)

def _live_moving_obstacle_boxes_from_pose(
    *,
    position_xy: tuple[float, float],
    yaw_rad: float,
    elapsed_s: float,
    mode: str,
    count: int,
    start_s: float,
    duration_s: float,
    period_s: float,
    forward_m: float,
    forward_step_m: float,
    lateral_phase_step_rad: float,
    lateral_amplitude_m: float,
    along_amplitude_m: float,
    radius_m: float,
    height_m: float,
) -> list[dict[str, Any]]:
    mode = str(mode or "none")
    if mode == "none":
        return []
    if elapsed_s < float(start_s):
        return []
    if float(duration_s) > 0.0 and elapsed_s > float(start_s) + float(duration_s):
        return []
    if mode != "robot_crossing":
        raise ValueError(f"unsupported moving_obstacle_mode: {mode}")

    obstacle_count = int(max(1, min(32, int(count))))
    t = float(elapsed_s) - float(start_s)
    period = max(0.1, float(period_s))
    base_phase = 2.0 * math.pi * t / period
    fwd = np.asarray([math.cos(yaw_rad), math.sin(yaw_rad)], dtype=np.float64)
    normal = np.asarray([-fwd[1], fwd[0]], dtype=np.float64)
    anchor = np.asarray(position_xy, dtype=np.float64)
    radius = max(0.02, float(radius_m))
    height = max(0.05, float(height_m))

    boxes: list[dict[str, Any]] = []
    for obstacle_idx in range(obstacle_count):
        offset = (float(obstacle_idx) - (float(obstacle_count) - 1.0) * 0.5) * float(forward_step_m)
        phase = base_phase + float(obstacle_idx) * float(lateral_phase_step_rad)
        lateral = float(lateral_amplitude_m) * math.sin(phase)
        along = float(along_amplitude_m) * math.sin(phase * 0.5)
        center = anchor + fwd * (float(forward_m) + offset + along) + normal * lateral
        boxes.append(
            {
                "name": (
                    "live_robot_crossing_obstacle"
                    if obstacle_count == 1
                    else f"live_robot_crossing_obstacle_{obstacle_idx}"
                ),
                "floor_id": 0,
                "position": [float(center[0]), float(center[1]), height * 0.5],
                "half_size": [radius, radius, height * 0.5],
                "elapsed_s": float(elapsed_s),
            }
        )
    return boxes

def _live_moving_obstacle_points(
    boxes: list[dict[str, Any]],
    *,
    spacing: float,
    intensity: float,
) -> list[tuple[float, float, float, float]]:
    points: list[tuple[float, float, float, float]] = []
    for box in boxes:
        points.extend(_box_surface_points(box, spacing=spacing, intensity=intensity))
    return points

def _live_moving_obstacle_speed_bounds(
    *,
    period_s: float,
    lateral_amplitude_m: float,
    along_amplitude_m: float,
) -> dict[str, float]:
    period = max(0.1, float(period_s))
    lateral_peak = abs(float(lateral_amplitude_m)) * 2.0 * math.pi / period
    along_peak = abs(float(along_amplitude_m)) * math.pi / period
    return {
        "peak_lateral_speed_mps": round(float(lateral_peak), 4),
        "peak_along_speed_mps": round(float(along_peak), 4),
        "peak_planar_speed_bound_mps": round(float(math.hypot(lateral_peak, along_peak)), 4),
    }

def _live_moving_obstacle_trail_clearance(
    *,
    timed_trail: list[tuple[float, float, float, float]],
    robot_radius_m: float,
    mode: str,
    count: int,
    start_s: float,
    duration_s: float,
    period_s: float,
    forward_m: float,
    forward_step_m: float,
    lateral_phase_step_rad: float,
    lateral_amplitude_m: float,
    along_amplitude_m: float,
    radius_m: float,
    height_m: float,
) -> dict[str, Any]:
    if str(mode or "none") == "none":
        return {"checked": False, "collision": False}
    best: float | None = None
    best_sample: dict[str, Any] | None = None
    active_count = 0
    for elapsed_s, x, y, yaw in timed_trail:
        boxes = _live_moving_obstacle_boxes_from_pose(
            position_xy=(float(x), float(y)),
            yaw_rad=float(yaw),
            elapsed_s=float(elapsed_s),
            mode=mode,
            count=count,
            start_s=start_s,
            duration_s=duration_s,
            period_s=period_s,
            forward_m=forward_m,
            forward_step_m=forward_step_m,
            lateral_phase_step_rad=lateral_phase_step_rad,
            lateral_amplitude_m=lateral_amplitude_m,
            along_amplitude_m=along_amplitude_m,
            radius_m=radius_m,
            height_m=height_m,
        )
        if not boxes:
            continue
        active_count += 1
        for box in boxes:
            clearance = _point_box_clearance((float(x), float(y)), box)
            if best is None or clearance < best:
                best = clearance
                best_sample = {
                    "elapsed_s": round(float(elapsed_s), 3),
                    "robot_xy": [round(float(x), 4), round(float(y), 4)],
                    "obstacle": box.get("name"),
                    "obstacle_xy": [
                        round(float(box["position"][0]), 4),
                        round(float(box["position"][1]), 4),
                    ],
                }
    margin = None if best is None else float(best) - float(robot_radius_m)
    return {
        "checked": True,
        "active_trail_sample_count": int(active_count),
        "min_clearance_m": None if best is None else round(float(best), 4),
        "min_clearance_minus_robot_radius_m": None if margin is None else round(float(margin), 4),
        "collision": bool(margin is not None and margin < 0.0),
        "robot_radius_m": float(robot_radius_m),
        "min_sample": best_sample,
    }

def _goal_xy(value: Any) -> tuple[float, float] | None:
    if isinstance(value, dict):
        value = value.get("goal") or value.get("current_goal")
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    try:
        return float(value[0]), float(value[1])
    except (TypeError, ValueError):
        return None

def _goal_xy_matches(
    a: tuple[float, float] | None,
    b: tuple[float, float] | None,
    *,
    tolerance: float = 1.0,
) -> bool:
    if a is None or b is None:
        return False
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1])) <= tolerance

def _parse_inspection_goals(value: str | list[Any] | tuple[Any, ...] | None) -> list[list[float]]:
    if value is None:
        return []
    raw: Any = value
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return []
        if text.startswith("["):
            raw = json.loads(text)
        else:
            raw = [item.strip() for item in text.split(";") if item.strip()]
    goals: list[list[float]] = []
    if not isinstance(raw, (list, tuple)):
        raise ValueError("inspection goals must be a JSON list or ';'-separated x,y[,z] list")
    for item in raw:
        frame_id = ""
        if isinstance(item, dict):
            frame_id = str(item.get("frame_id") or "")
            coords = [item.get("x"), item.get("y"), item.get("z", 0.0)]
        elif isinstance(item, str):
            coords = [part.strip() for part in item.split(",")]
        elif isinstance(item, (list, tuple)):
            coords = list(item)
        else:
            raise ValueError(f"unsupported inspection goal item: {item!r}")
        if frame_id and frame_id != SIM_NAV_ODOMETRY_FRAME_ID:
            raise ValueError(
                "inspection goal frame must be "
                f"{SIM_NAV_ODOMETRY_FRAME_ID}: {frame_id}"
            )
        if len(coords) < 2:
            raise ValueError(f"inspection goal needs x,y: {item!r}")
        try:
            x = float(coords[0])
            y = float(coords[1])
            z = float(coords[2]) if len(coords) > 2 and coords[2] not in (None, "") else 0.0
        except (TypeError, ValueError) as exc:
            raise ValueError(f"non-numeric inspection goal: {item!r}") from exc
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            raise ValueError(f"non-finite inspection goal: {item!r}")
        goals.append([x, y, z])
    return goals

def _map_frame_origin_world_xy_from_tomogram(
    tomogram: Path | str | None,
) -> tuple[float, float] | None:
    if not tomogram:
        return None
    path = Path(str(tomogram))
    candidates = []
    if path.is_dir():
        candidates.append(path / "metadata.json")
    else:
        candidates.append(path.parent / "metadata.json")
    for candidate in candidates:
        try:
            payload = json.loads(candidate.read_text(encoding="utf-8"))
        except Exception:
            continue
        origin = payload.get("map_frame_origin_world_xy")
        if isinstance(origin, (list, tuple)) and len(origin) >= 2:
            try:
                return (float(origin[0]), float(origin[1]))
            except (TypeError, ValueError):
                continue
    return None

def _state_in_map_frame(
    state: Any,
    origin_world_xy: tuple[float, float] | None,
) -> Any:
    position = np.asarray(state.position, dtype=np.float64).copy()
    if origin_world_xy is not None and position.shape[0] >= 2:
        position[0] -= float(origin_world_xy[0])
        position[1] -= float(origin_world_xy[1])
    return SimpleNamespace(
        position=position,
        orientation=np.asarray(state.orientation, dtype=np.float64).copy(),
        linear_velocity=np.asarray(state.linear_velocity, dtype=np.float64).copy(),
        angular_velocity=np.asarray(state.angular_velocity, dtype=np.float64).copy(),
    )

def _limit_command_delta(
    *,
    target: tuple[float, float, float],
    previous: tuple[float, float, float],
    dt_s: float,
    linear_accel_limit: float,
    angular_accel_limit: float,
) -> tuple[float, float, float]:
    dt = max(0.0, float(dt_s))
    linear_step = max(0.0, float(linear_accel_limit)) * dt
    angular_step = max(0.0, float(angular_accel_limit)) * dt

    def _limit_axis(target_value: float, previous_value: float, step: float) -> float:
        delta = float(target_value) - float(previous_value)
        if step <= 0.0:
            return float(target_value)
        return float(previous_value) + float(np.clip(delta, -step, step))

    return (
        _limit_axis(float(target[0]), float(previous[0]), linear_step),
        _limit_axis(float(target[1]), float(previous[1]), linear_step),
        _limit_axis(float(target[2]), float(previous[2]), angular_step),
    )

def _select_nav_cmd_for_step(
    *,
    latest_nav_cmd: dict[str, float],
    now_s: float,
    cmd_vel_timeout_s: float,
) -> dict[str, float | bool | None]:
    cmd_stamp = float(latest_nav_cmd.get("stamp") or 0.0)
    age_s = max(0.0, float(now_s) - cmd_stamp) if cmd_stamp > 0.0 else None
    timeout_s = float(cmd_vel_timeout_s)
    fresh = bool(cmd_stamp > 0.0 and (timeout_s <= 0.0 or float(age_s or 0.0) <= timeout_s))
    return {
        "fresh": fresh,
        "age_s": age_s,
        "vx": float(latest_nav_cmd.get("vx") or 0.0) if fresh else 0.0,
        "vy": float(latest_nav_cmd.get("vy") or 0.0) if fresh else 0.0,
        "wz": float(latest_nav_cmd.get("wz") or 0.0) if fresh else 0.0,
    }

def _motion_consistency_report(
    *,
    fastlio2_moved_m: float | None,
    fastlio2_path_length_m: float | None,
    sim_moved_m: float | None,
    sim_path_length_m: float | None,
) -> dict[str, Any]:
    checked = bool(fastlio2_moved_m is not None and sim_moved_m is not None)
    report: dict[str, Any] = {
        "checked": checked,
        "ok": checked,
        "fastlio2_moved_m": fastlio2_moved_m,
        "sim_moved_m": sim_moved_m,
        "fastlio2_path_length_m": fastlio2_path_length_m,
        "sim_path_length_m": sim_path_length_m,
        "motion_delta_error_m": None,
        "max_allowed_motion_error_m": None,
        "max_allowed_fastlio2_moved_m": None,
        "motion_scale_ratio": None,
    }
    if not checked:
        return report

    fast_m = float(fastlio2_moved_m or 0.0)
    sim_m = float(sim_moved_m or 0.0)
    error_m = abs(fast_m - sim_m)
    # Keep startup tolerance, but do not let short fixed-motion checks hide
    # large odometry scale errors. A 0.4x or 2x estimate is not acceptable even
    # when the absolute distance is still below a meter.
    allowed_error_m = max(0.12, sim_m * 0.35)
    report.update(
        {
            "ok": bool(error_m <= allowed_error_m),
            "motion_delta_error_m": error_m,
            "max_allowed_motion_error_m": allowed_error_m,
            "max_allowed_fastlio2_moved_m": sim_m + allowed_error_m,
            "motion_scale_ratio": (
                fast_m / sim_m if abs(sim_m) > 1e-6 else None
            ),
        }
    )
    return report

def _nav_planner_has_live_map(navigation: Any) -> bool:
    planner_svc = getattr(navigation, "_planner_svc", None)
    if planner_svc is None:
        return False
    try:
        return bool(getattr(planner_svc, "has_map", False))
    except Exception:
        return False

def _exploration_area_sample(grid: dict[str, Any]) -> dict[str, float]:
    counts = dict(grid.get("counts") or {})
    resolution = float(grid.get("resolution") or 0.0)
    cell_area = resolution * resolution
    unknown = int(counts.get("unknown") or 0)
    free = int(counts.get("free") or 0)
    occupied = int(counts.get("occupied") or 0)
    known = free + occupied
    return {
        "unknown_cells": float(unknown),
        "free_cells": float(free),
        "occupied_cells": float(occupied),
        "known_cells": float(known),
        "unknown_m2": float(unknown * cell_area),
        "free_m2": float(free * cell_area),
        "occupied_m2": float(occupied * cell_area),
        "known_m2": float(known * cell_area),
    }

def _normalize_localization_backend(value: Any) -> str:
    backend = str(value or "").strip().lower().replace("-", "_")
    if backend in {"", "portable", "portable_lio", "portable_fastlio2_like", "fastlio2_portable"}:
        raise ValueError(
            "portable_lio was removed: it was a lightweight estimator, not a "
            "validated no-ROS Fast-LIO2 implementation"
        )
    raise ValueError(f"unsupported localization_backend: {value}")

def _core_cloud_xy_stats(points: Any) -> dict[str, Any]:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[0] <= 0 or pts.shape[1] < 2:
        return {"point_count": 0, "xy_area_m2": 0.0}
    xy = pts[:, :2]
    finite = np.isfinite(xy).all(axis=1)
    xy = xy[finite]
    if xy.shape[0] <= 0:
        return {"point_count": 0, "xy_area_m2": 0.0}
    mins = np.min(xy, axis=0)
    maxs = np.max(xy, axis=0)
    span = np.maximum(maxs - mins, 0.0)
    return {
        "point_count": int(xy.shape[0]),
        "min_x": float(mins[0]),
        "min_y": float(mins[1]),
        "max_x": float(maxs[0]),
        "max_y": float(maxs[1]),
        "xy_area_m2": float(span[0] * span[1]),
    }
