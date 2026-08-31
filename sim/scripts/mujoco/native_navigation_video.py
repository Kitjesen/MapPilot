"""Render a clean video from a native-DDS MuJoCo navigation state log."""

from __future__ import annotations

import json
import math
import shutil
import subprocess
from bisect import bisect_right
from pathlib import Path
from typing import Any

import numpy as np
from sim.engine.mujoco.lidar import ROBOT_COLLISION_GEOM_GROUP

_CANDIDATE_BGR = {
    "feasible": (188, 188, 188),
    "collision_blocked": (68, 68, 232),
    "rotation_blocked": (92, 82, 224),
    "terrain_cost": (28, 166, 255),
    "terrain_blocked": (42, 42, 255),
    "direction_rejected": (104, 104, 112),
}
_SELECTED_BGR = (48, 245, 72)
_RECOVERY_BGR = (214, 92, 196)
_STOPPED_BGR = (42, 42, 255)
_DYNAMIC_BGR = (224, 82, 232)
_INACTIVE_PATH_BGR = (218, 186, 72)
_RAW_LIDAR_BGR = (255, 209, 31)
_PLANNER_OBSTACLE_BGR = (48, 54, 170)
_MAX_STATUS_AGE_S = 0.75
_MAX_PLANNER_SNAPSHOT_AGE_S = 0.75
_MAX_LIDAR_OVERLAY_AGE_S = 0.25
_PRESENTATION_LIDAR_MAX_Z_ABOVE_ROBOT_M = 1.8
_RAW_LIDAR_RADIUS_M = 0.014
_PLANNER_OBSTACLE_RADIUS_M = 0.012
_MAX_RAW_LIDAR_OVERLAY_POINTS = 640
_PRESENTATION_LIGHT_GAIN = 0.78
_MAX_KEYFRAME_WHITE_CLIP_FRACTION = 0.10


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    debug_by_id: dict[int, dict[str, Any]] = {}
    debug_path = path.with_name(f"{path.stem}_planner_debug.jsonl")
    if debug_path.is_file():
        for line in debug_path.read_text(encoding="utf-8").splitlines():
            try:
                snapshot = json.loads(line)
                snapshot_id = int(snapshot.get("id"))
            except (json.JSONDecodeError, TypeError, ValueError):
                continue
            if isinstance(snapshot, dict):
                debug_by_id[snapshot_id] = snapshot

    motion_rows: list[dict[str, Any]] = []
    nav_overlay_by_id: dict[int, dict[str, Any]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(value, dict) or not value.get("qpos"):
            continue
        motion_rows.append(value)
        try:
            snapshot_id = int(value.get("planner_debug_id"))
        except (TypeError, ValueError):
            continue
        if snapshot_id <= 0 or snapshot_id in nav_overlay_by_id:
            continue
        nav_overlay_by_id[snapshot_id] = {
            "global_path": value.get("global_path") or [],
            "local_path": value.get("local_path") or [],
            "last_local": value.get("local_diagnostics") or {},
            "input_gate": value.get("input_gate") or {},
            "dynamic_objects": value.get("dynamic_objects") or [],
        }

    rows: list[dict[str, Any]] = []
    active_exact_snapshot_id = -1
    exact_snapshot_motion_stamp_s = math.nan
    for value in motion_rows:
        try:
            snapshot_id = int(value.get("planner_debug_id"))
        except (TypeError, ValueError):
            snapshot_id = -1
        snapshot: dict[str, Any] | None = None
        try:
            frame_stamp = float(value.get("t"))
        except (TypeError, ValueError):
            frame_stamp = math.nan
        snapshot = debug_by_id.get(snapshot_id)
        if snapshot is not None and snapshot_id > 0:
            if snapshot_id != active_exact_snapshot_id:
                active_exact_snapshot_id = snapshot_id
                exact_snapshot_motion_stamp_s = frame_stamp
            value["planner_debug_join"] = "exact_id"
            value["planner_debug_motion_stamp_s"] = exact_snapshot_motion_stamp_s
        if snapshot is not None:
            planner_debug = snapshot.get("local_planner_debug") or {}
            selected_id = int(snapshot.get("id"))
            status_stamp = float(snapshot.get("nav_status_stamp_s"))
            value["planner_debug_id"] = selected_id
            value["nav_status_stamp_s"] = status_stamp
            value["local_planner_debug"] = planner_debug
            value["local_candidates"] = planner_debug.get("candidates") or []
            value["local_map"] = snapshot.get("local_map") or {}
            legacy_overlay = nav_overlay_by_id.get(selected_id) or {}
            for field in ("global_path", "local_path", "dynamic_objects"):
                if field in snapshot:
                    value[field] = snapshot.get(field) or []
                elif field in legacy_overlay:
                    value[field] = legacy_overlay.get(field) or []
            if "input_gate" in snapshot:
                value["input_gate"] = snapshot.get("input_gate") or {}
            elif "input_gate" in legacy_overlay:
                value["input_gate"] = legacy_overlay.get("input_gate") or {}
            if "last_local" in snapshot or "last_local" in legacy_overlay:
                local_diagnostics = (
                    snapshot.get("last_local")
                    if "last_local" in snapshot
                    else legacy_overlay.get("last_local")
                ) or {}
                value["local_diagnostics"] = local_diagnostics
                value["local_reason"] = str(local_diagnostics.get("reason") or "")
            if value.get("planner_debug_join") == "exact_id":
                join_stamp = float(value.get("planner_debug_motion_stamp_s"))
                if math.isfinite(frame_stamp) and frame_stamp >= join_stamp:
                    value["nav_status_hold_age_s"] = frame_stamp - join_stamp
            elif math.isfinite(frame_stamp) and frame_stamp >= status_stamp:
                value["nav_status_hold_age_s"] = frame_stamp - status_stamp
        rows.append(value)
    return rows


def _resample_rows_for_cfr(
    rows: list[dict[str, Any]], fps: float
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Resample telemetry onto a CFR timeline without inventing new state."""

    frame_rate = max(0.1, float(fps))
    frame_period_s = 1.0 / frame_rate
    timed_rows: list[tuple[float, dict[str, Any]]] = []
    for row in rows:
        try:
            stamp = float(row.get("t"))
        except (TypeError, ValueError):
            continue
        if not math.isfinite(stamp):
            continue
        if timed_rows and stamp <= timed_rows[-1][0]:
            continue
        timed_rows.append((stamp, row))
    if not timed_rows:
        return [], {
            "source_rows": 0,
            "output_frames": 0,
            "timeline_preserved": False,
        }

    stamps = [item[0] for item in timed_rows]
    span_s = max(0.0, stamps[-1] - stamps[0])
    output_count = max(1, int(math.ceil(span_s * frame_rate - 1e-9)) + 1)
    output_rows: list[dict[str, Any]] = []
    used_source_indices: set[int] = set()
    held_frames = 0
    for frame_index in range(output_count):
        target_stamp = stamps[0] + frame_index * frame_period_s
        source_index = bisect_right(stamps, target_stamp + 1e-9) - 1
        source_index = min(max(0, source_index), len(timed_rows) - 1)
        source_stamp, source_row = timed_rows[source_index]
        rendered_row = dict(source_row)
        rendered_row["t"] = target_stamp
        rendered_row["timeline_source_stamp_s"] = source_stamp
        rendered_row["timeline_hold_age_s"] = max(0.0, target_stamp - source_stamp)
        output_rows.append(rendered_row)
        used_source_indices.add(source_index)
        if target_stamp - source_stamp > frame_period_s * 0.5:
            held_frames += 1

    gaps = [right - left for left, right in zip(stamps, stamps[1:])]
    source_duration_s = span_s + frame_period_s
    encoded_duration_s = len(output_rows) / frame_rate
    duration_error_s = abs(encoded_duration_s - source_duration_s)
    timeline = {
        "source_rows": len(timed_rows),
        "output_frames": len(output_rows),
        "source_span_s": span_s,
        "source_duration_s": source_duration_s,
        "encoded_duration_s": encoded_duration_s,
        "duration_error_s": duration_error_s,
        "effective_input_hz": (
            (len(timed_rows) - 1) / span_s if len(timed_rows) > 1 and span_s > 0.0 else 0.0
        ),
        "max_source_gap_s": max(gaps) if gaps else 0.0,
        "gaps_over_0_15_s": sum(gap > 0.15 for gap in gaps),
        "gaps_over_0_5_s": sum(gap > 0.5 for gap in gaps),
        "gaps_over_1_0_s": sum(gap > 1.0 for gap in gaps),
        "held_frames": held_frames,
        "source_rows_used": len(used_source_indices),
        "timeline_preserved": duration_error_s <= frame_period_s + 1e-6,
    }
    return output_rows, timeline


def _path3(value: Any) -> list[np.ndarray]:
    points: list[np.ndarray] = []
    for item in value or []:
        try:
            point = np.asarray(item[:3], dtype=np.float64)
        except (TypeError, ValueError):
            continue
        if point.shape == (3,) and np.isfinite(point).all():
            points.append(point)
    return points


def _presentation_vertical_slice(
    points: Any, row: dict[str, Any]
) -> list[np.ndarray]:
    """Hide overhead returns only in the replay presentation."""

    try:
        robot_z = float(row.get("z") or 0.0)
    except (TypeError, ValueError):
        robot_z = 0.0
    max_z = robot_z + _PRESENTATION_LIDAR_MAX_Z_ABOVE_ROBOT_M
    return [point for point in _path3(points) if point[2] <= max_z]


def _presentation_lidar_points(row: dict[str, Any]) -> list[np.ndarray]:
    """Return a fresh, display-only vertical slice of the recorded raw LiDAR."""

    try:
        hold_age_s = float(row.get("timeline_hold_age_s") or 0.0)
    except (TypeError, ValueError):
        return []
    if not math.isfinite(hold_age_s) or hold_age_s > _MAX_LIDAR_OVERLAY_AGE_S:
        return []
    return _presentation_vertical_slice(row.get("lidar_world"), row)


def _presentation_planner_obstacle_points(row: dict[str, Any]) -> list[np.ndarray]:
    """Return fresh planner obstacles with the same display-only roof slice."""

    local_map = _effective_local_map(row)
    if not bool(local_map.get("enabled")):
        return []
    points = (
        local_map.get("obstacle_points")
        if local_map.get("obstacle_points_fresh") is not False
        else []
    )
    if not points:
        collision = local_map.get("collision") or {}
        if collision.get("live") is True and collision.get("complete") is True:
            points = collision.get("occupied_points") or []
    return _presentation_vertical_slice(points, row)


def _presentation_filter_status_text() -> str:
    return (
        "presentation filter: overhead raw lidar + planner obstacles > "
        f"robot+{_PRESENTATION_LIDAR_MAX_Z_ABOVE_ROBOT_M:.1f}m hidden"
    )


def _sensor_overlay_legend() -> tuple[tuple[str, tuple[int, int, int]], ...]:
    return (
        ("raw lidar", _RAW_LIDAR_BGR),
        ("planner obstacles", _PLANNER_OBSTACLE_BGR),
    )


def _candidate_overlay_legend() -> tuple[tuple[str, tuple[int, int, int]], ...]:
    return (
        ("feasible", _candidate_bgr("feasible", selected=False)),
        ("blocked", _candidate_bgr("collision_blocked", selected=False)),
        ("terrain", _candidate_bgr("terrain_cost", selected=False)),
        ("selected", _SELECTED_BGR),
        ("dynamic", _DYNAMIC_BGR),
    )


def _bgr_to_rgba(
    color: tuple[int, int, int], alpha: float
) -> tuple[float, float, float, float]:
    blue, green, red = color
    return red / 255.0, green / 255.0, blue / 255.0, alpha


def _frame_luma_metrics(frame_bgr: np.ndarray) -> dict[str, float]:
    """Return compact exposure diagnostics for an encoded presentation frame."""

    frame = np.asarray(frame_bgr, dtype=np.float32)
    if frame.ndim != 3 or frame.shape[2] < 3 or frame.size == 0:
        return {
            "white_clip_fraction": 0.0,
            "pure_white_fraction": 0.0,
            "luma_p50": 0.0,
            "luma_p90": 0.0,
            "luma_p99": 0.0,
        }
    luma = (
        frame[:, :, 0] * 0.114
        + frame[:, :, 1] * 0.587
        + frame[:, :, 2] * 0.299
    )
    return {
        "white_clip_fraction": float(np.mean(luma >= 245.0)),
        "pure_white_fraction": float(np.mean(np.all(frame[:, :, :3] >= 254.5, axis=2))),
        "luma_p50": float(np.percentile(luma, 50.0)),
        "luma_p90": float(np.percentile(luma, 90.0)),
        "luma_p99": float(np.percentile(luma, 99.0)),
    }


def _candidate_bgr(state: str, *, selected: bool) -> tuple[int, int, int]:
    if selected:
        return _SELECTED_BGR
    return _CANDIDATE_BGR.get(str(state), _CANDIDATE_BGR["direction_rejected"])


def _candidate_rgba(state: str, *, selected: bool) -> tuple[float, float, float, float]:
    blue, green, red = _candidate_bgr(state, selected=selected)
    alpha = 0.98 if selected else 0.56
    return red / 255.0, green / 255.0, blue / 255.0, alpha


def _selected_candidate(row: dict[str, Any]) -> dict[str, Any] | None:
    return next(
        (
            candidate
            for candidate in _effective_local_candidates(row)
            if isinstance(candidate, dict) and bool(candidate.get("selected"))
        ),
        None,
    )


def _local_path_bgr(row: dict[str, Any]) -> tuple[int, int, int]:
    if _local_safety_stopped(row):
        return _STOPPED_BGR
    if _local_is_recovery(row):
        return _RECOVERY_BGR
    if _selected_candidate(row) is not None:
        return _SELECTED_BGR
    return _INACTIVE_PATH_BGR


def _traversability_is_complete(traversability: dict[str, Any]) -> bool:
    return bool(traversability.get("complete"))


def _fused_cost_provenance_label(traversability: dict[str, Any]) -> str:
    provenance = traversability.get("provenance")
    if isinstance(provenance, str) and provenance.strip():
        return provenance.strip()
    source_layers: Any = traversability.get("source_layers")
    if isinstance(provenance, dict):
        source_layers = provenance.get("source_layers") or provenance.get("layers")
    if isinstance(source_layers, (list, tuple)):
        layers = [str(layer).strip() for layer in source_layers if str(layer).strip()]
        if layers:
            return "+".join(layers)
    return "occupancy+terrain/unknown"


def _fused_cost_status_text(traversability: dict[str, Any]) -> str:
    risk_total = int(traversability.get("risk_cells_total") or 0)
    risk_returned = int(
        traversability.get("risk_cells_returned")
        or len(traversability.get("risk_cells") or [])
    )
    grid_state = "complete" if _traversability_is_complete(traversability) else "sampled/unknown"
    fresh_state = "fresh" if bool(traversability.get("fresh")) else "stale/n-a"
    return (
        f"fused cost cells {risk_returned}/{risk_total or '?'}  |  "
        f"{_fused_cost_provenance_label(traversability)}  |  {grid_state}  |  {fresh_state}"
    )


def _snapshot_age_s(row: dict[str, Any]) -> float | None:
    debug = row.get("local_planner_debug") or {}
    if not bool(debug.get("valid")):
        return None
    status_age = _nav_status_age_s(row)
    if status_age is not None:
        return status_age
    try:
        frame_stamp = float(row.get("t"))
        snapshot_stamp = float(debug.get("timestamp_s"))
    except (TypeError, ValueError):
        return None
    if (
        not math.isfinite(frame_stamp)
        or not math.isfinite(snapshot_stamp)
        or frame_stamp <= 0.0
        or snapshot_stamp <= 0.0
        or snapshot_stamp > frame_stamp
    ):
        return None
    return frame_stamp - snapshot_stamp


def _nav_status_age_s(row: dict[str, Any]) -> float | None:
    if row.get("planner_debug_join") == "exact_id":
        try:
            frame_stamp = float(row.get("t"))
            join_stamp = float(row.get("planner_debug_motion_stamp_s"))
        except (TypeError, ValueError):
            return None
        if (
            not math.isfinite(frame_stamp)
            or not math.isfinite(join_stamp)
            or frame_stamp <= 0.0
            or join_stamp <= 0.0
            or join_stamp > frame_stamp
        ):
            return None
        return frame_stamp - join_stamp
    try:
        frame_stamp = float(row.get("t"))
        status_stamp = float(row.get("nav_status_stamp_s"))
    except (TypeError, ValueError):
        return None
    if (
        not math.isfinite(frame_stamp)
        or not math.isfinite(status_stamp)
        or frame_stamp <= 0.0
        or status_stamp <= 0.0
        or status_stamp > frame_stamp
    ):
        return None
    return frame_stamp - status_stamp


def _planner_snapshot_is_fresh(row: dict[str, Any]) -> bool:
    age = _snapshot_age_s(row)
    return age is not None and age <= _MAX_PLANNER_SNAPSHOT_AGE_S


def _nav_status_is_fresh(row: dict[str, Any]) -> bool:
    age = _nav_status_age_s(row)
    return age is not None and age <= _MAX_STATUS_AGE_S


def _effective_local_candidates(row: dict[str, Any]) -> list[dict[str, Any]]:
    if not _planner_snapshot_is_fresh(row) or not _nav_status_is_fresh(row):
        return []
    return [
        candidate
        for candidate in (row.get("local_candidates") or [])
        if isinstance(candidate, dict)
    ]


def _effective_local_map(row: dict[str, Any]) -> dict[str, Any]:
    if not _nav_status_is_fresh(row):
        return {}
    local_map = row.get("local_map") or {}
    return local_map if isinstance(local_map, dict) else {}


def _effective_traversability(row: dict[str, Any]) -> dict[str, Any]:
    local_map = _effective_local_map(row)
    if not bool(local_map.get("enabled")):
        return {}
    traversability = local_map.get("traversability") or {}
    if not isinstance(traversability, dict) or traversability.get("fresh") is False:
        return {}
    try:
        rows = int(traversability.get("rows") or 0)
        cols = int(traversability.get("cols") or 0)
        resolution = float(traversability.get("resolution_m") or 0.0)
    except (TypeError, ValueError):
        return {}
    origin = traversability.get("origin_xy") or []
    if rows <= 0 or cols <= 0 or resolution <= 0.0 or len(origin) < 2:
        return {}
    return traversability


def _local_map_has_visible_content(
    row: dict[str, Any], *, range_m: float = 2.0
) -> bool:
    """Prove that planner obstacles or fused-cost cells land in the inset."""

    if not bool(_effective_local_map(row).get("enabled")):
        return False
    robot_x = float(row.get("x") or 0.0)
    robot_y = float(row.get("y") or 0.0)
    radius_sq = max(0.0, float(range_m)) ** 2
    for point in _presentation_planner_obstacle_points(row):
        if (float(point[0]) - robot_x) ** 2 + (float(point[1]) - robot_y) ** 2 <= radius_sq:
            return True

    traversability = _effective_traversability(row)
    origin = traversability.get("origin_xy") or []
    if len(origin) < 2:
        return False
    resolution = float(traversability.get("resolution_m") or 0.0)
    if resolution <= 0.0:
        return False
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    for cell in traversability.get("risk_cells") or []:
        if not isinstance(cell, (list, tuple)) or len(cell) < 2:
            continue
        cell_x = origin_x + (int(cell[1]) + 0.5) * resolution
        cell_y = origin_y + (int(cell[0]) + 0.5) * resolution
        if (cell_x - robot_x) ** 2 + (cell_y - robot_y) ** 2 <= radius_sq:
            return True
    return False


def _effective_dynamic_objects(row: dict[str, Any]) -> list[dict[str, Any]]:
    if not _nav_status_is_fresh(row):
        return []
    return [
        item
        for item in (row.get("dynamic_objects") or [])
        if isinstance(item, dict)
    ]


def _local_is_recovery(row: dict[str, Any]) -> bool:
    if not _nav_status_is_fresh(row):
        return False
    diagnostics = row.get("local_diagnostics") or {}
    return (
        str(row.get("local_reason") or "") in {"recovering", "recovery_path"}
        or int(diagnostics.get("recovery_state") or 0) > 0
    )


def _local_goal_reached(row: dict[str, Any]) -> bool:
    if not _nav_status_is_fresh(row):
        return False
    diagnostics = row.get("local_diagnostics") or {}
    return bool(diagnostics.get("goal_reached")) or str(
        row.get("local_reason") or ""
    ) == "goal_reached"


def _local_safety_stopped(row: dict[str, Any]) -> bool:
    if not _nav_status_is_fresh(row):
        return False
    diagnostics = row.get("local_diagnostics") or {}
    final_safety = diagnostics.get("final_safety") or {}
    return bool(diagnostics.get("near_field_stop")) or bool(final_safety.get("stopped"))


def _candidate_display_bgr(
    row: dict[str, Any], candidate: dict[str, Any]
) -> tuple[int, int, int]:
    if bool(candidate.get("selected")):
        if _local_safety_stopped(row):
            return _STOPPED_BGR
        if _local_is_recovery(row):
            return _RECOVERY_BGR
    return _candidate_bgr(
        str(candidate.get("state") or "direction_rejected"),
        selected=bool(candidate.get("selected")),
    )


def _candidate_display_rgba(
    row: dict[str, Any], candidate: dict[str, Any]
) -> tuple[float, float, float, float]:
    blue, green, red = _candidate_display_bgr(row, candidate)
    return red / 255.0, green / 255.0, blue / 255.0, 0.92


def _terrain_bgr(
    risk: float,
    *,
    soft_cost: float = 40.0,
    hard_cost: float = 90.0,
) -> tuple[int, int, int]:
    if risk < 0.0:
        return 92, 92, 92
    if risk >= hard_cost:
        return 44, 52, 238
    if risk >= soft_cost:
        return 24, 178, 255
    fraction = max(0.0, min(1.0, risk / max(1.0, soft_cost)))
    return (
        int(78 - 36 * fraction),
        int(126 + 56 * fraction),
        int(62 + 36 * fraction),
    )


def _world_xy_to_panel(
    x: float,
    y: float,
    *,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    center: tuple[int, int],
    pixels_per_m: float,
) -> tuple[int, int]:
    dx = float(x) - robot_x
    dy = float(y) - robot_y
    c = math.cos(robot_yaw)
    s = math.sin(robot_yaw)
    body_x = c * dx + s * dy
    body_y = -s * dx + c * dy
    return (
        int(round(center[0] - body_y * pixels_per_m)),
        int(round(center[1] - body_x * pixels_per_m)),
    )


def _render_local_planner_inset(
    frame_bgr: np.ndarray,
    row: dict[str, Any],
    *,
    panel_size: int = 440,
    range_m: float = 2.0,
) -> np.ndarray:
    import cv2

    frame = np.ascontiguousarray(frame_bgr)
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame_bgr must be an HxWx3 image")
    height, width = frame.shape[:2]
    size = max(180, min(int(panel_size), height - 24, width - 24))
    origin_x = width - size - 12
    origin_y = 12
    panel = np.full((size, size, 3), (22, 28, 31), dtype=np.uint8)
    center = (size // 2, size // 2 + 12)
    pixels_per_m = (size * 0.43) / max(0.5, float(range_m))
    robot_x = float(row.get("x") or 0.0)
    robot_y = float(row.get("y") or 0.0)
    robot_yaw = float(row.get("yaw") or 0.0)

    for radius_m in (0.5, 1.0, float(range_m)):
        cv2.circle(
            panel,
            center,
            max(1, int(round(radius_m * pixels_per_m))),
            (62, 70, 74),
            1,
            cv2.LINE_AA,
        )
    cv2.line(panel, (center[0], size - 8), (center[0], 42), (64, 76, 82), 1, cv2.LINE_AA)
    cv2.line(panel, (8, center[1]), (size - 8, center[1]), (64, 76, 82), 1, cv2.LINE_AA)

    traversability = _effective_traversability(row)
    planner_debug = row.get("local_planner_debug") or {}
    soft_cost = float(planner_debug.get("traversability_soft_cost") or 40.0)
    hard_cost = float(planner_debug.get("traversability_hard_cost") or 90.0)
    rows = int(traversability.get("rows") or 0)
    cols = int(traversability.get("cols") or 0)
    resolution = float(traversability.get("resolution_m") or 0.0)
    grid_origin = traversability.get("origin_xy") or [0.0, 0.0]
    if traversability:
        gx = float(grid_origin[0])
        gy = float(grid_origin[1])
        grid_corners = np.asarray(
            [
                _world_xy_to_panel(
                    gx,
                    gy,
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                    center=center,
                    pixels_per_m=pixels_per_m,
                ),
                _world_xy_to_panel(
                    gx + cols * resolution,
                    gy,
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                    center=center,
                    pixels_per_m=pixels_per_m,
                ),
                _world_xy_to_panel(
                    gx + cols * resolution,
                    gy + rows * resolution,
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                    center=center,
                    pixels_per_m=pixels_per_m,
                ),
                _world_xy_to_panel(
                    gx,
                    gy + rows * resolution,
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                    center=center,
                    pixels_per_m=pixels_per_m,
                ),
            ],
            dtype=np.int32,
        )
        grid_complete = _traversability_is_complete(traversability)
        grid_fill = (27, 48, 32) if grid_complete else (49, 51, 52)
        grid_outline = (76, 112, 82) if grid_complete else (92, 96, 98)
        cv2.fillConvexPoly(panel, grid_corners, grid_fill, cv2.LINE_AA)
        cv2.polylines(panel, [grid_corners], True, grid_outline, 1, cv2.LINE_AA)
        cell_radius = max(1, int(math.ceil(0.55 * resolution * pixels_per_m)))
        for cell in traversability.get("risk_cells") or []:
            if not isinstance(cell, (list, tuple)) or len(cell) < 3:
                continue
            grid_row = int(cell[0])
            grid_col = int(cell[1])
            risk = float(cell[2])
            pixel = _world_xy_to_panel(
                gx + (grid_col + 0.5) * resolution,
                gy + (grid_row + 0.5) * resolution,
                robot_x=robot_x,
                robot_y=robot_y,
                robot_yaw=robot_yaw,
                center=center,
                pixels_per_m=pixels_per_m,
            )
            cv2.rectangle(
                panel,
                (pixel[0] - cell_radius, pixel[1] - cell_radius),
                (pixel[0] + cell_radius, pixel[1] + cell_radius),
                _terrain_bgr(risk, soft_cost=soft_cost, hard_cost=hard_cost),
                -1,
                cv2.LINE_AA,
            )

    for raw_point in _presentation_lidar_points(row):
        if len(raw_point) < 2:
            continue
        pixel = _world_xy_to_panel(
            raw_point[0],
            raw_point[1],
            robot_x=robot_x,
            robot_y=robot_y,
            robot_yaw=robot_yaw,
            center=center,
            pixels_per_m=pixels_per_m,
        )
        if 0 <= pixel[0] < size and 0 <= pixel[1] < size:
            cv2.circle(panel, pixel, 2, _RAW_LIDAR_BGR, -1, cv2.LINE_AA)

    visible_obstacles = _presentation_planner_obstacle_points(row)
    for obstacle in visible_obstacles:
        if len(obstacle) < 2:
            continue
        pixel = _world_xy_to_panel(
            obstacle[0],
            obstacle[1],
            robot_x=robot_x,
            robot_y=robot_y,
            robot_yaw=robot_yaw,
            center=center,
            pixels_per_m=pixels_per_m,
        )
        if 0 <= pixel[0] < size and 0 <= pixel[1] < size:
            cv2.circle(panel, pixel, 2, _PLANNER_OBSTACLE_BGR, -1, cv2.LINE_AA)

    for dynamic_object in _effective_dynamic_objects(row):
        centroid = dynamic_object.get("centroid") or []
        velocity = dynamic_object.get("velocity") or []
        if len(centroid) < 2:
            continue
        pixel = _world_xy_to_panel(
            centroid[0],
            centroid[1],
            robot_x=robot_x,
            robot_y=robot_y,
            robot_yaw=robot_yaw,
            center=center,
            pixels_per_m=pixels_per_m,
        )
        if not (0 <= pixel[0] < size and 0 <= pixel[1] < size):
            continue
        cv2.circle(panel, pixel, 6, _DYNAMIC_BGR, 2, cv2.LINE_AA)
        if len(velocity) >= 2:
            velocity_tip = _world_xy_to_panel(
                float(centroid[0]) + float(velocity[0]),
                float(centroid[1]) + float(velocity[1]),
                robot_x=robot_x,
                robot_y=robot_y,
                robot_yaw=robot_yaw,
                center=center,
                pixels_per_m=pixels_per_m,
            )
            cv2.arrowedLine(
                panel,
                pixel,
                velocity_tip,
                _DYNAMIC_BGR,
                2,
                cv2.LINE_AA,
                tipLength=0.25,
            )

    candidates = _effective_local_candidates(row)
    for candidate in candidates:
        points = _path3(candidate.get("path") if isinstance(candidate, dict) else None)
        if len(points) < 2:
            continue
        polyline = np.asarray(
            [
                _world_xy_to_panel(
                    point[0],
                    point[1],
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                    center=center,
                    pixels_per_m=pixels_per_m,
                )
                for point in points
            ],
            dtype=np.int32,
        )
        selected = bool(candidate.get("selected"))
        color = _candidate_display_bgr(row, candidate)
        cv2.polylines(
            panel,
            [polyline],
            False,
            color,
            5 if selected else 2,
            cv2.LINE_AA,
        )
        cv2.circle(panel, tuple(polyline[-1]), 4 if selected else 3, color, -1, cv2.LINE_AA)

    selected_path = (
        _path3(row.get("local_path")) if _nav_status_is_fresh(row) else []
    )
    if len(selected_path) >= 2:
        selected_polyline = np.asarray(
            [
                _world_xy_to_panel(
                    point[0],
                    point[1],
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                    center=center,
                    pixels_per_m=pixels_per_m,
                )
                for point in selected_path
            ],
            dtype=np.int32,
        )
        cv2.polylines(panel, [selected_polyline], False, _local_path_bgr(row), 5, cv2.LINE_AA)

    half_length = max(3, int(round(0.50 * pixels_per_m)))
    half_width = max(2, int(round(0.30 * pixels_per_m)))
    cv2.rectangle(
        panel,
        (center[0] - half_width, center[1] - half_length),
        (center[0] + half_width, center[1] + half_length),
        (235, 226, 205),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        f"LOCAL MAP + PLANNER {range_m:.1f}m  |  forward ^",
        (14, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (235, 238, 240),
        1,
        cv2.LINE_AA,
    )
    selected_candidate = _selected_candidate(row)
    snapshot_age = _snapshot_age_s(row)
    age_label = f"age {snapshot_age:.2f}s" if snapshot_age is not None else "age n/a"
    if _local_goal_reached(row):
        summary = f"GOAL REACHED  |  {age_label}"
        summary_color = _SELECTED_BGR
    elif _local_safety_stopped(row):
        reason = str(((row.get("local_diagnostics") or {}).get("final_safety") or {}).get("reason") or "stop")
        summary = f"SAFETY STOP  |  {reason}  |  {age_label}"
        summary_color = _STOPPED_BGR
    elif _local_is_recovery(row):
        summary = f"RECOVERY PATH  |  {age_label}"
        summary_color = _RECOVERY_BGR
    elif selected_candidate is not None:
        summary = (
            f"reps {len(candidates)}  |  selected "
            f"r{int(selected_candidate.get('rotation_index', -1))}/"
            f"g{int(selected_candidate.get('group_id', -1))}  |  "
            f"terrain {float(selected_candidate.get('terrain_risk') or 0.0):.0f}  |  {age_label}"
        )
        summary_color = (205, 224, 212)
    elif bool(planner_debug.get("valid")) and not _planner_snapshot_is_fresh(row):
        summary = f"DEBUG STALE - OVERLAYS HIDDEN  |  {age_label}"
        summary_color = (72, 132, 255)
    elif not bool(planner_debug.get("valid")):
        summary = "DEBUG UNAVAILABLE"
        summary_color = (104, 176, 244)
    else:
        reason = str(row.get("local_reason") or "inactive")
        summary = f"NO SELECTED CANDIDATE  |  {reason}  |  {age_label}"
        summary_color = (54, 182, 255)
    cv2.putText(
        panel,
        summary,
        (14, 46),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        summary_color,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        _fused_cost_status_text(traversability),
        (14, 64),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.34,
        (180, 190, 194),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        _presentation_filter_status_text(),
        (14, 82),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.30,
        (146, 166, 174),
        1,
        cv2.LINE_AA,
    )

    sensor_legend_y = size - 36
    sensor_legend_x = 14
    for label, color in _sensor_overlay_legend():
        cv2.circle(panel, (sensor_legend_x + 8, sensor_legend_y), 3, color, -1, cv2.LINE_AA)
        cv2.putText(
            panel,
            label,
            (sensor_legend_x + 17, sensor_legend_y + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (220, 224, 226),
            1,
            cv2.LINE_AA,
        )
        label_width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.34, 1)[0][0]
        sensor_legend_x += label_width + 42

    legend = _candidate_overlay_legend()
    legend_y = size - 16
    legend_x = 14
    for label, color in legend:
        cv2.line(panel, (legend_x, legend_y), (legend_x + 18, legend_y), color, 3, cv2.LINE_AA)
        cv2.putText(
            panel,
            label,
            (legend_x + 23, legend_y + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.36,
            (220, 224, 226),
            1,
            cv2.LINE_AA,
        )
        legend_x += 78

    roi = frame[origin_y : origin_y + size, origin_x : origin_x + size]
    roi[:] = cv2.addWeighted(panel, 0.94, roi, 0.06, 0.0)
    cv2.rectangle(frame, (origin_x, origin_y), (origin_x + size - 1, origin_y + size - 1), (112, 122, 128), 1)
    return frame


def _next_geom(scene: Any) -> Any | None:
    if int(scene.ngeom) >= int(scene.maxgeom):
        return None
    geom = scene.geoms[int(scene.ngeom)]
    scene.ngeom += 1
    return geom


def _add_sphere(
    mujoco: Any,
    scene: Any,
    position: np.ndarray,
    radius: float,
    rgba: tuple[float, float, float, float],
) -> None:
    geom = _next_geom(scene)
    if geom is None:
        return
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.asarray([radius, radius, radius], dtype=np.float64),
        np.asarray(position, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(-1),
        np.asarray(rgba, dtype=np.float32),
    )


def _add_segment(
    mujoco: Any,
    scene: Any,
    start: np.ndarray,
    end: np.ndarray,
    radius: float,
    rgba: tuple[float, float, float, float],
) -> None:
    if float(np.linalg.norm(end - start)) < 1e-5:
        return
    geom = _next_geom(scene)
    if geom is None:
        return
    color = np.asarray(rgba, dtype=np.float32)
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.asarray([radius, 0.0, 0.0], dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(-1),
        color,
    )
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        float(radius),
        np.asarray(start, dtype=np.float64),
        np.asarray(end, dtype=np.float64),
    )
    geom.rgba[:] = color


def _draw_path(
    mujoco: Any,
    scene: Any,
    points: list[np.ndarray],
    *,
    radius: float,
    rgba: tuple[float, float, float, float],
    z_offset: float,
    max_segments: int,
) -> None:
    if len(points) < 2:
        return
    step = max(1, int(math.ceil((len(points) - 1) / max(1, max_segments))))
    sampled = points[::step]
    if not np.array_equal(sampled[-1], points[-1]):
        sampled.append(points[-1])
    lifted = [point + np.asarray([0.0, 0.0, z_offset]) for point in sampled]
    for start, end in zip(lifted, lifted[1:]):
        _add_segment(mujoco, scene, start, end, radius, rgba)


def _make_robot_readable(mujoco: Any, model: Any) -> None:
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_id < 0:
        return
    for geom_id in range(int(model.ngeom)):
        body_id = int(model.geom_bodyid[geom_id])
        cursor = body_id
        belongs_to_robot = False
        while cursor > 0:
            if cursor == base_id:
                belongs_to_robot = True
                break
            cursor = int(model.body_parentid[cursor])
        if not belongs_to_robot:
            continue
        name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or "").lower()
        if name.endswith("_foot_visual"):
            model.geom_rgba[geom_id][3] = 0.0
        elif "wheel" in name:
            model.geom_rgba[geom_id] = np.asarray([0.03, 0.03, 0.04, 1.0])
        else:
            model.geom_rgba[geom_id] = np.asarray([0.10, 0.18, 0.24, 1.0])


def _hide_observer_occluders(mujoco: Any, model: Any) -> None:
    """Hide sensor-only overhead geometry in the presentation replay."""
    for geom_id in range(int(model.ngeom)):
        name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or "").lower()
        if name.startswith(("roof_deck", "roof_strip")):
            model.geom_group[geom_id] = 4


def _apply_presentation_light_gain(
    model: Any,
    gain: float = _PRESENTATION_LIGHT_GAIN,
) -> None:
    """Prevent the replay-only light rig from clipping bright scene materials."""

    gain = float(gain)
    if not math.isfinite(gain) or gain <= 0.0:
        raise ValueError("presentation light gain must be finite and positive")
    for field in ("light_diffuse", "light_ambient", "light_specular"):
        values = getattr(model, field)
        values[:] = np.asarray(values) * gain
    headlight = model.vis.headlight
    for field in ("diffuse", "ambient", "specular"):
        values = getattr(headlight, field)
        values[:] = np.asarray(values) * gain


def _configure_camera(
    camera: Any,
    *,
    preset: str,
    start: np.ndarray,
    goal: np.ndarray,
) -> None:
    """Frame the compact comparison course or set a readable chase view."""

    if preset == "follow":
        camera.distance = 3.25
        camera.elevation = -27.0
        camera.azimuth = 145.0
        camera.lookat[:] = [start[0] + 0.70, start[1], max(0.42, start[2] + 0.12)]
        return
    center = (np.asarray(start, dtype=np.float64) + np.asarray(goal, dtype=np.float64)) * 0.5
    camera.distance = max(5.25, float(np.linalg.norm(goal[:2] - start[:2])) * 1.75)
    camera.elevation = -48.0
    camera.azimuth = 135.0
    camera.lookat[:] = [center[0], center[1], max(0.30, center[2] * 0.55)]


def _transcode_h264(raw_path: Path, output: Path) -> tuple[bool, str]:
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raw_path.replace(output)
        return False, "ffmpeg_unavailable_mp4v_preserved"
    proc = subprocess.run(
        [
            ffmpeg,
            "-y",
            "-loglevel",
            "error",
            "-i",
            str(raw_path),
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "19",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output),
        ],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if proc.returncode != 0 or not output.is_file():
        raw_path.replace(output)
        return False, f"ffmpeg_failed:{proc.stderr[-500:]}"
    raw_path.unlink(missing_ok=True)
    return True, "h264_yuv420p"


def _validate_video_decode(path: Path) -> dict[str, Any]:
    """Decode the complete artifact so container metadata alone cannot pass."""

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg:
        proc = subprocess.run(
            [
                ffmpeg,
                "-v",
                "error",
                "-xerror",
                "-i",
                str(path),
                "-map",
                "0:v:0",
                "-f",
                "null",
                "-",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
        )
        return {
            "ok": proc.returncode == 0,
            "method": "ffmpeg_full_decode",
            "error": "" if proc.returncode == 0 else proc.stderr[-1000:],
        }

    import cv2

    capture = cv2.VideoCapture(str(path))
    reported_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) if capture.isOpened() else 0
    decoded_frames = 0
    while capture.isOpened():
        ok, _frame = capture.read()
        if not ok:
            break
        decoded_frames += 1
    capture.release()
    return {
        "ok": decoded_frames > 0 and (
            reported_frames <= 0 or decoded_frames == reported_frames
        ),
        "method": "opencv_full_decode",
        "decoded_frames": decoded_frames,
        "reported_frames": reported_frames,
        "error": "",
    }


def render_native_navigation_video(
    *,
    world: str | Path,
    policy_path: str | Path,
    motion_log: str | Path,
    output: str | Path,
    goal: list[float],
    width: int = 1920,
    height: int = 1080,
    fps: float = 24.0,
    active_only: bool = True,
    show_debug_inset: bool = True,
    show_sensor_points: bool = True,
    camera_preset: str = "overview",
    show_goal: bool = True,
    require_candidate_evidence: bool = True,
) -> dict[str, Any]:
    import cv2
    from sim.scripts.mujoco.native_dds_sensors import build_engine

    import mujoco

    log_path = Path(motion_log).expanduser().resolve()
    output_path = Path(output).expanduser().resolve()
    loaded_rows = _load_jsonl(log_path)
    if active_only:
        driving_rows = [row for row in loaded_rows if bool(row.get("driving"))]
        first_navigation_row = next(
            (
                index
                for index, row in enumerate(driving_rows)
                if bool((row.get("local_diagnostics") or {}).get("active"))
            ),
            0,
        )
        source_rows = driving_rows[first_navigation_row:]
    else:
        source_rows = loaded_rows
    if not source_rows:
        reason = "motion_log_has_no_driving_samples" if active_only else "motion_log_has_no_samples"
        return {"ok": False, "reason": reason}
    rows, timeline = _resample_rows_for_cfr(source_rows, float(fps))
    if not rows:
        return {"ok": False, "reason": "motion_log_has_no_valid_timestamps"}
    camera_preset = str(camera_preset).strip().lower()
    if camera_preset not in {"overview", "follow"}:
        raise ValueError(f"unknown camera preset: {camera_preset}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    raw_path = output_path.with_name(f"{output_path.stem}.raw.mp4")
    first_frame_path = output_path.with_name(f"{output_path.stem}_first.png")
    middle_frame_path = output_path.with_name(f"{output_path.stem}_middle.png")
    last_frame_path = output_path.with_name(f"{output_path.stem}_last.png")
    engine = build_engine(
        world=Path(world),
        drive_mode="policy",
        start=[float(value) for value in rows[0].get("start_xyz") or [0.0, 0.0, 0.48]],
        mujoco_memory="96M",
        mid360_samples_per_frame=15000,
        lidar_backend="mujoco_lidar",
        mujoco_lidar_backend="cpu",
        require_product_lidar_backend=True,
        policy_path=Path(policy_path),
    )
    model = engine.model
    data = engine.data
    _make_robot_readable(mujoco, model)
    _hide_observer_occluders(mujoco, model)
    _apply_presentation_light_gain(model)
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), int(width))
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), int(height))
    renderer = mujoco.Renderer(model, height=int(height), width=int(width), max_geom=24000)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    start_position = np.asarray(
        rows[0].get("start_xyz") or [rows[0]["x"], rows[0]["y"], rows[0]["z"]],
        dtype=np.float64,
    )
    goal_position = np.asarray(goal[:3], dtype=np.float64)
    goal_position[2] = max(0.16, float(goal_position[2]))
    _configure_camera(
        camera,
        preset=camera_preset,
        start=start_position,
        goal=goal_position,
    )
    scene_option = mujoco.MjvOption()
    scene_option.geomgroup[:] = 1
    scene_option.geomgroup[ROBOT_COLLISION_GEOM_GROUP] = 0
    scene_option.geomgroup[4] = 0
    writer = cv2.VideoWriter(
        str(raw_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(fps),
        (int(width), int(height)),
    )
    if not writer.isOpened():
        renderer.close()
        engine.close()
        return {"ok": False, "reason": "video_writer_open_failed"}

    trail: list[np.ndarray] = []
    frame_count = 0
    keyframe_luma: dict[str, dict[str, float]] = {}
    try:
        for row_index, row in enumerate(rows):
            qpos = np.asarray(row["qpos"], dtype=np.float64)
            qpos_count = min(len(qpos), len(data.qpos))
            data.qpos[:qpos_count] = qpos[:qpos_count]
            data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)
            robot = np.asarray([row["x"], row["y"], row["z"]], dtype=np.float64)
            if not trail or float(np.linalg.norm(robot[:2] - trail[-1][:2])) >= 0.015:
                trail.append(robot.copy())

            if camera_preset == "follow":
                robot_yaw = float(row.get("yaw") or 0.0)
                lookahead_xy = 0.78 * np.asarray([math.cos(robot_yaw), math.sin(robot_yaw)], dtype=np.float64)
                camera.azimuth = 145.0 + math.degrees(robot_yaw)
            else:
                lookahead_xy = None
            if lookahead_xy is not None:
                camera.lookat[:] = [
                    robot[0] + lookahead_xy[0],
                    robot[1] + lookahead_xy[1],
                    max(0.42, float(robot[2]) + 0.14),
                ]
            renderer.update_scene(data, camera, scene_option=scene_option)
            scene = renderer.scene
            scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 1
            _draw_path(
                mujoco,
                scene,
                _path3(row.get("global_path")),
                radius=0.025,
                rgba=(0.90, 0.56, 0.12, 0.95),
                z_offset=0.06,
                max_segments=80,
            )
            if show_sensor_points:
                planner_obstacles = _presentation_planner_obstacle_points(row)
                for point in planner_obstacles[:200]:
                    _add_sphere(
                        mujoco,
                        scene,
                        point,
                        _PLANNER_OBSTACLE_RADIUS_M,
                        _bgr_to_rgba(_PLANNER_OBSTACLE_BGR, 0.82),
                    )
            for candidate in _effective_local_candidates(row):
                _draw_path(
                    mujoco,
                    scene,
                    _path3(candidate.get("path")),
                    radius=0.010 if not bool(candidate.get("selected")) else 0.020,
                    rgba=_candidate_display_rgba(row, candidate),
                    z_offset=0.085,
                    max_segments=14,
                )
            for dynamic_object in _effective_dynamic_objects(row):
                centroid = _path3([dynamic_object.get("centroid") or []])
                if not centroid:
                    continue
                position = centroid[0]
                velocity = _path3([dynamic_object.get("velocity") or []])
                _add_sphere(mujoco, scene, position, 0.07, (0.91, 0.32, 0.88, 0.92))
                if velocity:
                    _add_segment(
                        mujoco,
                        scene,
                        position,
                        position + velocity[0],
                        0.018,
                        (0.91, 0.32, 0.88, 0.92),
                    )
            local_path_bgr = _local_path_bgr(row)
            local_path_rgba = (
                local_path_bgr[2] / 255.0,
                local_path_bgr[1] / 255.0,
                local_path_bgr[0] / 255.0,
                0.95,
            )
            _draw_path(
                mujoco,
                scene,
                _path3(row.get("local_path")) if _nav_status_is_fresh(row) else [],
                radius=0.032,
                rgba=local_path_rgba,
                z_offset=0.10,
                max_segments=60,
            )
            trail_floor = [np.asarray([point[0], point[1], 0.06]) for point in trail]
            _draw_path(
                mujoco,
                scene,
                trail_floor,
                radius=0.018,
                rgba=(0.16, 0.60, 0.76, 0.88),
                z_offset=0.0,
                max_segments=90,
            )
            if show_goal:
                _add_sphere(mujoco, scene, goal_position, 0.11, (0.12, 0.62, 0.42, 0.88))
            if show_sensor_points:
                lidar_points = _presentation_lidar_points(row)
                for point in lidar_points[:_MAX_RAW_LIDAR_OVERLAY_POINTS]:
                    _add_sphere(
                        mujoco,
                        scene,
                        point,
                        _RAW_LIDAR_RADIUS_M,
                        _bgr_to_rgba(_RAW_LIDAR_BGR, 0.92),
                    )

            frame_rgb = renderer.render().copy()
            frame_bgr = frame_rgb[:, :, ::-1]
            if row_index == 0:
                keyframe_luma["first"] = _frame_luma_metrics(frame_bgr)
            if row_index == len(rows) // 2:
                keyframe_luma["middle"] = _frame_luma_metrics(frame_bgr)
            if row_index == len(rows) - 1:
                keyframe_luma["last"] = _frame_luma_metrics(frame_bgr)
            if show_debug_inset:
                frame_bgr = _render_local_planner_inset(frame_bgr, row)
            writer.write(frame_bgr)
            if frame_count == 0:
                cv2.imwrite(str(first_frame_path), frame_bgr)
            if row_index == len(rows) // 2:
                cv2.imwrite(str(middle_frame_path), frame_bgr)
            frame_count += 1
            if frame_count == len(rows):
                cv2.imwrite(str(last_frame_path), frame_bgr)
    finally:
        writer.release()
        renderer.close()
        engine.close()

    transcoded, codec = _transcode_h264(raw_path, output_path)
    capture = cv2.VideoCapture(str(output_path))
    decoded_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) if capture.isOpened() else 0
    decoded_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) if capture.isOpened() else 0
    decoded_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)) if capture.isOpened() else 0
    capture.release()
    decode_validation = _validate_video_decode(output_path)
    candidate_frames = sum(bool(_effective_local_candidates(row)) for row in rows)
    exact_planner_join_frames = sum(
        row.get("planner_debug_join") == "exact_id" for row in rows
    )
    selected_candidate_frames = sum(
        any(
            bool(candidate.get("selected"))
            for candidate in _effective_local_candidates(row)
        )
        for row in rows
    )
    stale_candidate_frames_hidden = sum(
        bool(row.get("local_candidates")) and not bool(_effective_local_candidates(row))
        for row in rows
    )
    local_map_frames = sum(
        bool(_effective_local_map(row).get("enabled")) for row in rows
    )
    visible_local_map_frames = sum(
        _local_map_has_visible_content(row) for row in rows
    )
    traversability_frames = sum(
        bool(_effective_traversability(row))
        for row in rows
    )
    stale_local_map_frames_hidden = sum(
        bool((row.get("local_map") or {}).get("enabled"))
        and not bool(_effective_local_map(row).get("enabled"))
        for row in rows
    )
    stale_status_frames = sum(not _nav_status_is_fresh(row) for row in rows)
    dynamic_object_frames = sum(bool(_effective_dynamic_objects(row)) for row in rows)
    snapshot_ages = [
        age
        for row in rows
        if (age := _snapshot_age_s(row)) is not None
    ]
    overlays = ["global_path"]
    if candidate_frames:
        overlays.append("local_candidates")
    overlays.extend(["local_path", "executed_trail", "live_lidar"])
    if local_map_frames:
        overlays.append("planner_local_map")
    if traversability_frames:
        overlays.append("traversability_cost")
    if dynamic_object_frames:
        overlays.append("dynamic_objects")
    if show_goal:
        overlays.append("goal")
    brightness_ok = bool(keyframe_luma) and all(
        metrics["white_clip_fraction"] <= _MAX_KEYFRAME_WHITE_CLIP_FRACTION
        for metrics in keyframe_luma.values()
    )
    presentation_evidence_ok = (
        (not require_candidate_evidence or (candidate_frames > 0 and selected_candidate_frames > 0))
        and local_map_frames > 0
        and visible_local_map_frames > 0
        and exact_planner_join_frames > 0
    )
    return {
        "ok": (
            output_path.is_file()
            and decoded_frames > 0
            and bool(timeline.get("timeline_preserved"))
            and presentation_evidence_ok
            and bool(decode_validation.get("ok"))
        ),
        "video": str(output_path),
        "codec": codec,
        "h264_transcoded": transcoded,
        "frames_written": frame_count,
        "frames_decoded": decoded_frames,
        "decode_validation": decode_validation,
        "width": decoded_width,
        "height": decoded_height,
        "fps": float(fps),
        "first_frame": str(first_frame_path),
        "middle_frame": str(middle_frame_path),
        "last_frame": str(last_frame_path),
        "motion_log": str(log_path),
        "overlays": overlays,
        "candidate_frames": candidate_frames,
        "exact_planner_join_frames": exact_planner_join_frames,
        "selected_candidate_frames": selected_candidate_frames,
        "stale_candidate_frames_hidden": stale_candidate_frames_hidden,
        "local_map_frames": local_map_frames,
        "visible_local_map_frames": visible_local_map_frames,
        "stale_local_map_frames_hidden": stale_local_map_frames_hidden,
        "traversability_frames": traversability_frames,
        "dynamic_object_frames": dynamic_object_frames,
        "stale_status_frames": stale_status_frames,
        "timeline": timeline,
        "snapshot_age_s": {
            "available": bool(snapshot_ages),
            "meaning": "motion_frame_clock_age_of_exact_planner_debug_id",
            "join_policy": "exact_planner_debug_id_fail_closed",
            "max": max(snapshot_ages) if snapshot_ages else None,
            "mean": (
                float(sum(snapshot_ages) / len(snapshot_ages))
                if snapshot_ages
                else None
            ),
        },
        "text_overlay": True,
        "active_only": bool(active_only),
        "inset_overlay": bool(show_debug_inset),
        "sensor_point_overlay": bool(show_sensor_points),
        "camera_preset": camera_preset,
        "goal_overlay": bool(show_goal),
        "candidate_evidence_required": bool(require_candidate_evidence),
        "presentation_lighting": {
            "gain": _PRESENTATION_LIGHT_GAIN,
            "keyframe_white_clip_limit": _MAX_KEYFRAME_WHITE_CLIP_FRACTION,
            "keyframe_luma": keyframe_luma,
            "brightness_ok": brightness_ok,
        },
        "point_overlay_style": {
            "raw_lidar_radius_m": _RAW_LIDAR_RADIUS_M,
            "raw_lidar_max_points": _MAX_RAW_LIDAR_OVERLAY_POINTS,
            "planner_obstacle_radius_m": _PLANNER_OBSTACLE_RADIUS_M,
        },
        "candidate_legend": {
            "feasible": "gray",
            "collision_blocked": "red",
            "rotation_blocked": "red",
            "terrain_cost": "amber",
            "terrain_blocked": "red",
            "selected": "bright_green",
        },
    }
