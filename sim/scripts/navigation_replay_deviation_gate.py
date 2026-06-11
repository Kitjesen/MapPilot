#!/usr/bin/env python3
"""Validate offline navigation replay/deviation evidence without motion output."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

SCHEMA_VERSION = "lingtu.navigation_replay_deviation_gate.v1"

DEFAULT_THRESHOLDS = {
    "min_sample_count": 5,
    "min_cmd_vel_count": 3,
    "min_cmd_vel_nonzero_ratio": 0.2,
    "min_odometry_count": 5,
    "min_odom_motion_m": 0.25,
    "max_final_distance_m": 0.8,
    "max_tracking_error_p95_m": 0.6,
    "max_tracking_error_final_m": 0.8,
}


def _as_float(value: Any, default: float | None = None) -> float | None:
    try:
        return float(value)
    except Exception:
        return default


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _bool_false(payload: dict[str, Any], key: str) -> bool:
    return payload.get(key) is False


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _resolve_artifact_path(report_path: Path, value: Any) -> Path | None:
    if not value:
        return None
    raw = Path(str(value))
    candidates = [raw]
    if not raw.is_absolute():
        candidates.extend([report_path.parent / raw, ROOT / raw])
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[-1] if candidates else None


def _xy(value: Any) -> tuple[float, float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        x = _as_float(value[0])
        y = _as_float(value[1])
        if x is not None and y is not None:
            return (x, y)
    if not isinstance(value, dict):
        return None
    for nested_key in ("pose", "position", "odom", "odometry", "robot_pose", "base_pose"):
        nested = value.get(nested_key)
        xy = _xy(nested)
        if xy is not None:
            return xy
    if "x" in value and "y" in value:
        x = _as_float(value.get("x"))
        y = _as_float(value.get("y"))
        if x is not None and y is not None:
            return (x, y)
    return None


def _cmd_components(value: Any) -> tuple[float, float, float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        vx = _as_float(value[0], 0.0)
        vy = _as_float(value[1], 0.0)
        wz = _as_float(value[2], 0.0) if len(value) >= 3 else 0.0
        if vx is not None and vy is not None and wz is not None:
            return (vx, vy, wz)
    if not isinstance(value, dict):
        return None

    linear = value.get("linear") if isinstance(value.get("linear"), dict) else {}
    angular = value.get("angular") if isinstance(value.get("angular"), dict) else {}
    vx = _as_float(
        value.get("linear_x", value.get("vx", linear.get("x", value.get("x", 0.0)))),
        0.0,
    )
    vy = _as_float(
        value.get("linear_y", value.get("vy", linear.get("y", value.get("y", 0.0)))),
        0.0,
    )
    wz = _as_float(
        value.get("angular_z", value.get("wz", angular.get("z", value.get("yaw_rate", 0.0)))),
        0.0,
    )
    if vx is None or vy is None or wz is None:
        return None
    return (vx, vy, wz)


def _samples(trace: dict[str, Any]) -> list[dict[str, Any]]:
    raw = (
        trace.get("samples")
        or trace.get("timeline")
        or trace.get("replay_samples")
        or trace.get("messages")
        or []
    )
    if isinstance(raw, dict):
        raw = raw.get("rows") or raw.get("samples") or []
    if not isinstance(raw, list):
        return []
    return [item for item in raw if isinstance(item, dict)]


def _list_len(value: Any) -> int:
    return len(value) if isinstance(value, list) else 0


def _path_count_from_sample(sample: dict[str, Any], *keys: str) -> int:
    for key in keys:
        if key in sample:
            value = sample.get(key)
            if isinstance(value, (int, float)):
                return max(0, int(value))
            count = _list_len(value)
            if count:
                return count
    return 0


def _path_points(value: Any) -> list[tuple[float, float]]:
    if isinstance(value, list):
        return [point for item in value if (point := _xy(item)) is not None]
    if not isinstance(value, dict):
        return []
    for key in (
        "poses",
        "path",
        "points",
        "global_path",
        "local_path",
        "trajectory",
        "trajectory_xy",
    ):
        points = _path_points(value.get(key))
        if points:
            return points
    return []


def _path_count(trace: dict[str, Any], samples: list[dict[str, Any]], *, count_key: str, path_keys: tuple[str, ...]) -> int:
    explicit = _as_int(trace.get(count_key), 0)
    if explicit > 0:
        return explicit
    for key in path_keys:
        count = _list_len(trace.get(key))
        if count:
            return count
    return sum(1 for sample in samples if _path_count_from_sample(sample, *path_keys, count_key) > 0)


def _odom_points(trace: dict[str, Any], samples: list[dict[str, Any]]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for key in ("odometry", "odom", "poses", "trajectory", "trajectory_xy"):
        raw = trace.get(key)
        if isinstance(raw, list):
            for item in raw:
                xy = _xy(item)
                if xy is not None:
                    points.append(xy)
            if points:
                return points
    for sample in samples:
        xy = _xy(sample.get("odom") or sample.get("odometry") or sample.get("pose") or sample)
        if xy is not None:
            points.append(xy)
    return points


def _commands(samples: list[dict[str, Any]]) -> list[tuple[float, float, float]]:
    commands: list[tuple[float, float, float]] = []
    for sample in samples:
        raw = (
            sample.get("cmd_vel")
            or sample.get("cmd")
            or sample.get("velocity_command")
            or sample.get("command")
        )
        cmd = _cmd_components(raw)
        if cmd is not None:
            commands.append(cmd)
    return commands


def _path_length(points: list[tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    total = 0.0
    previous = points[0]
    for point in points[1:]:
        total += math.hypot(point[0] - previous[0], point[1] - previous[1])
        previous = point
    return total


def _goal_xy(trace: dict[str, Any]) -> tuple[float, float] | None:
    for key in ("goal", "target", "final_goal"):
        xy = _xy(trace.get(key))
        if xy is not None:
            return xy
    for key in ("global_path", "path"):
        raw = trace.get(key)
        if isinstance(raw, list) and raw:
            return _xy(raw[-1])
    return None


def _tracking_errors(trace: dict[str, Any], samples: list[dict[str, Any]]) -> list[float]:
    errors: list[float] = []
    raw_errors = (
        trace.get("tracking_errors_m")
        or trace.get("cross_track_errors_m")
        or trace.get("deviation_errors_m")
        or []
    )
    if isinstance(raw_errors, list):
        for item in raw_errors:
            value = _as_float(item)
            if value is not None:
                errors.append(abs(value))

    for sample in samples:
        for key in (
            "tracking_error_m",
            "cross_track_error_m",
            "local_path_error_m",
            "path_tracking_error_m",
            "deviation_m",
        ):
            value = _as_float(sample.get(key))
            if value is not None:
                errors.append(abs(value))
                break
    return errors


def _percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    pos = (len(ordered) - 1) * q / 100.0
    low = math.floor(pos)
    high = math.ceil(pos)
    if low == high:
        return float(ordered[int(pos)])
    weight = pos - low
    return float(ordered[low] * (1.0 - weight) + ordered[high] * weight)


def _nonzero_cmd_count(commands: list[tuple[float, float, float]]) -> int:
    return sum(1 for vx, vy, wz in commands if abs(vx) + abs(vy) + abs(wz) > 1e-6)


def _point_dict(point: tuple[float, float]) -> dict[str, float | str]:
    return {"x": float(point[0]), "y": float(point[1]), "z": 0.0, "frame_id": "map"}


def _event_payload(event: dict[str, Any]) -> Any:
    for key in ("msg", "message", "data", "payload"):
        if key in event:
            return event.get(key)
    return event


def _event_time(event: dict[str, Any], fallback: float) -> float:
    for key in ("t", "time", "stamp", "timestamp", "sec"):
        value = _as_float(event.get(key))
        if value is not None:
            return value
    header = event.get("header") if isinstance(event.get("header"), dict) else {}
    stamp = header.get("stamp") if isinstance(header.get("stamp"), dict) else {}
    sec = _as_float(stamp.get("sec"))
    nanosec = _as_float(stamp.get("nanosec"), 0.0)
    if sec is not None and nanosec is not None:
        return sec + nanosec * 1e-9
    return fallback


def _scalar_error(value: Any) -> float | None:
    if isinstance(value, (int, float, str)):
        return _as_float(value)
    if not isinstance(value, dict):
        return None
    for key in (
        "tracking_error_m",
        "cross_track_error_m",
        "local_path_error_m",
        "path_tracking_error_m",
        "deviation_m",
        "error_m",
        "value",
    ):
        number = _as_float(value.get(key))
        if number is not None:
            return abs(number)
    return None


def _nearest_path_error(point: tuple[float, float], path: list[tuple[float, float]]) -> float | None:
    if not path:
        return None
    return min(math.hypot(point[0] - target[0], point[1] - target[1]) for target in path)


def _polyline_length(points: list[tuple[float, float]]) -> tuple[float, list[float]]:
    distances = [0.0]
    total = 0.0
    previous = points[0] if points else (0.0, 0.0)
    for point in points[1:]:
        total += math.hypot(point[0] - previous[0], point[1] - previous[1])
        distances.append(total)
        previous = point
    return total, distances


def _sample_polyline(points: list[tuple[float, float]], count: int) -> list[tuple[float, float]]:
    if not points:
        return []
    if len(points) == 1 or count <= 1:
        return [points[0]]
    total, distances = _polyline_length(points)
    if total <= 1e-9:
        return [points[0] for _ in range(count)]
    sampled: list[tuple[float, float]] = []
    segment_index = 0
    for index in range(count):
        target = total * index / (count - 1)
        while segment_index + 1 < len(distances) and distances[segment_index + 1] < target:
            segment_index += 1
        next_index = min(segment_index + 1, len(points) - 1)
        segment_start = distances[segment_index]
        segment_end = distances[next_index]
        ratio = 0.0 if segment_end <= segment_start else (target - segment_start) / (segment_end - segment_start)
        start = points[segment_index]
        end = points[next_index]
        sampled.append(
            (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
            )
        )
    return sampled


def build_routecheck_trace(
    routecheck_report_path: Path,
    *,
    phase: str = "candidate",
    sample_count: int = 12,
) -> dict[str, Any]:
    report = _load_json(routecheck_report_path)
    artifacts = report.get("artifacts") if isinstance(report.get("artifacts"), dict) else {}
    phase_files = artifacts.get(f"{phase}_files") if isinstance(artifacts.get(f"{phase}_files"), dict) else {}
    plan_path = _resolve_artifact_path(routecheck_report_path, phase_files.get("plan"))
    if plan_path is None or not plan_path.exists():
        raise FileNotFoundError(f"{phase} plan artifact is missing in routecheck report")
    plan = _load_json(plan_path)
    path_points = [_xy(item) for item in plan.get("path") or []]
    path_xy = [point for point in path_points if point is not None]
    if len(path_xy) < 2:
        raise ValueError(f"{phase} plan artifact does not contain at least two path points")

    sample_count = max(int(sample_count), max(5, len(path_xy) * 3))
    replay_xy = _sample_polyline(path_xy, sample_count)
    total_length, _ = _polyline_length(path_xy)
    dt = 0.2
    samples: list[dict[str, Any]] = []
    for index, point in enumerate(replay_xy):
        if index + 1 < len(replay_xy):
            next_point = replay_xy[index + 1]
            vx = (next_point[0] - point[0]) / dt
            vy = (next_point[1] - point[1]) / dt
        elif index > 0:
            previous = replay_xy[index - 1]
            vx = (point[0] - previous[0]) / dt
            vy = (point[1] - previous[1]) / dt
        else:
            vx = 0.0
            vy = 0.0
        samples.append(
            {
                "t": round(index * dt, 3),
                "odom": {"x": point[0], "y": point[1], "yaw": 0.0, "frame_id": "map"},
                "cmd_vel": {
                    "linear_x": vx,
                    "linear_y": vy,
                    "angular_z": 0.0,
                },
                "global_path_points": len(path_xy),
                "local_path_points": len(path_xy),
                "tracking_error_m": 0.0,
            }
        )

    goal_xy = _xy(plan.get("goal")) or path_xy[-1]
    return {
        "schema_version": "lingtu.navigation_replay_trace.v1",
        "source": "routecheck_preflight_plan_preview",
        "source_report": str(routecheck_report_path),
        "source_plan": str(plan_path),
        "source_phase": phase,
        "trace_kind": "non_motion_plan_replay",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "global_path": [_point_dict(point) for point in path_xy],
        "local_path": [_point_dict(point) for point in path_xy],
        "goal": _point_dict(goal_xy),
        "samples": samples,
        "final_distance_m": 0.0,
        "tracking_error_p95_m": 0.0,
        "tracking_error_final_m": 0.0,
        "odom_motion_m": total_length,
    }


def build_topic_jsonl_trace(topic_jsonl_path: Path) -> dict[str, Any]:
    global_path: list[tuple[float, float]] = []
    local_path: list[tuple[float, float]] = []
    odom_points: list[tuple[float, float]] = []
    samples: list[dict[str, Any]] = []
    explicit_errors: list[float] = []
    goal: tuple[float, float] | None = None
    last_cmd: tuple[float, float, float] | None = None

    for index, line in enumerate(topic_jsonl_path.read_text(encoding="utf-8-sig").splitlines()):
        if not line.strip():
            continue
        event = json.loads(line)
        if not isinstance(event, dict):
            continue
        topic = str(event.get("topic") or event.get("name") or "").lower()
        payload = _event_payload(event)
        timestamp = _event_time(event, index * 0.1)

        if "local_path" in topic or "localpath" in topic:
            points = _path_points(payload)
            if points:
                local_path = points
            continue
        if "global_path" in topic or "globalpath" in topic or (
            topic.endswith("/path") and "local" not in topic
        ):
            points = _path_points(payload)
            if points:
                global_path = points
            continue
        if "goal" in topic or "target" in topic:
            goal_xy = _xy(payload)
            if goal_xy is not None:
                goal = goal_xy
            continue
        if "tracking" in topic or "deviation" in topic or "cross_track" in topic:
            error = _scalar_error(payload)
            if error is not None:
                explicit_errors.append(error)
            continue
        if "cmd_vel" in topic or topic.endswith("/cmd") or "velocity_command" in topic:
            last_cmd = _cmd_components(payload)
            continue
        if "odom" in topic or "odometry" in topic or "pose" in topic:
            xy = _xy(payload)
            if xy is None:
                continue
            odom_points.append(xy)
            sample: dict[str, Any] = {
                "t": timestamp,
                "odom": {"x": xy[0], "y": xy[1], "yaw": 0.0, "frame_id": "map"},
                "global_path_points": len(global_path),
                "local_path_points": len(local_path),
            }
            if last_cmd is not None:
                sample["cmd_vel"] = {
                    "linear_x": last_cmd[0],
                    "linear_y": last_cmd[1],
                    "angular_z": last_cmd[2],
                }
            samples.append(sample)

    tracking_path = local_path or global_path
    inferred_errors = [
        error
        for point in odom_points
        if (error := _nearest_path_error(point, tracking_path)) is not None
    ]
    trace_goal = goal or (global_path[-1] if global_path else None)
    return {
        "schema_version": "lingtu.navigation_replay_trace.v1",
        "source": "recorded_topic_jsonl",
        "source_report": str(topic_jsonl_path),
        "trace_kind": "recorded_topic_replay",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "global_path": [_point_dict(point) for point in global_path],
        "local_path": [_point_dict(point) for point in local_path],
        "goal": _point_dict(trace_goal) if trace_goal is not None else {},
        "odometry": [_point_dict(point) for point in odom_points],
        "samples": samples,
        "tracking_errors_m": explicit_errors or inferred_errors,
    }


def _merge_thresholds(overrides: dict[str, float] | None = None) -> dict[str, float]:
    thresholds = dict(DEFAULT_THRESHOLDS)
    for key, value in (overrides or {}).items():
        thresholds[key] = float(value)
    return thresholds


def _check(ok: bool, blocker: str, evidence: dict[str, Any] | None = None) -> dict[str, Any]:
    return {
        "ok": bool(ok),
        "blocker": "" if ok else blocker,
        "evidence": evidence or {},
    }


def build_fixture_trace() -> dict[str, Any]:
    samples: list[dict[str, Any]] = []
    for index in range(8):
        progress = index / 7.0
        x = progress * 2.0
        y = progress * 0.4
        cmd = {"linear_x": 0.35 if index < 7 else 0.0, "angular_z": 0.04}
        samples.append(
            {
                "t": index * 0.2,
                "odom": {"x": x, "y": y, "yaw": progress * 0.1},
                "cmd_vel": cmd,
                "global_path_points": 4,
                "local_path_points": 5,
                "tracking_error_m": 0.04 + 0.01 * (index % 3),
            }
        )
    return {
        "schema_version": "lingtu.navigation_replay_trace.v1",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "goal": {"x": 2.0, "y": 0.4, "yaw": 0.1},
        "global_path": [
            {"x": 0.0, "y": 0.0},
            {"x": 1.0, "y": 0.2},
            {"x": 2.0, "y": 0.4},
        ],
        "local_path": [
            {"x": 0.0, "y": 0.0},
            {"x": 0.6, "y": 0.12},
            {"x": 1.2, "y": 0.24},
            {"x": 2.0, "y": 0.4},
        ],
        "samples": samples,
    }


def evaluate_trace(
    trace: dict[str, Any],
    *,
    thresholds: dict[str, float] | None = None,
) -> dict[str, Any]:
    thresholds = _merge_thresholds(thresholds)
    samples = _samples(trace)
    odom_points = _odom_points(trace, samples)
    commands = _commands(samples)
    explicit_cmd_count = _as_int(trace.get("cmd_vel_count"), 0)
    explicit_nonzero = _as_int(trace.get("cmd_vel_nonzero"), 0)
    cmd_vel_count = explicit_cmd_count if explicit_cmd_count > 0 else len(commands)
    cmd_vel_nonzero = explicit_nonzero if explicit_nonzero > 0 else _nonzero_cmd_count(commands)
    explicit_cmd_ratio = _as_float(trace.get("cmd_vel_nonzero_ratio"))
    cmd_vel_nonzero_ratio = (
        explicit_cmd_ratio
        if explicit_cmd_ratio is not None
        else (cmd_vel_nonzero / cmd_vel_count if cmd_vel_count else 0.0)
    )
    odometry_count = _as_int(trace.get("odometry_count"), 0) or len(odom_points)
    odom_motion_m = _as_float(trace.get("odom_motion_m"))
    if odom_motion_m is None:
        odom_motion_m = _path_length(odom_points)
    odom_displacement_m = 0.0
    if len(odom_points) >= 2:
        odom_displacement_m = math.hypot(
            odom_points[-1][0] - odom_points[0][0],
            odom_points[-1][1] - odom_points[0][1],
        )

    final_distance_m = _as_float(trace.get("final_distance_m"))
    goal = _goal_xy(trace)
    if final_distance_m is None and goal is not None and odom_points:
        final_distance_m = math.hypot(odom_points[-1][0] - goal[0], odom_points[-1][1] - goal[1])

    errors = _tracking_errors(trace, samples)
    tracking_error_p95_m = _as_float(trace.get("tracking_error_p95_m"))
    if tracking_error_p95_m is None:
        trajectory_quality = trace.get("trajectory_quality")
        if isinstance(trajectory_quality, dict):
            tracking_error_p95_m = _as_float(
                trajectory_quality.get("tracking_error_p95_m")
                or trajectory_quality.get("cross_track_error_p95_m")
                or trajectory_quality.get("p95_error_m")
            )
    if tracking_error_p95_m is None:
        tracking_error_p95_m = _percentile(errors, 95.0)

    tracking_error_final_m = _as_float(trace.get("tracking_error_final_m"))
    if tracking_error_final_m is None:
        tracking_error_final_m = errors[-1] if errors else None

    global_path_count = _path_count(
        trace,
        samples,
        count_key="global_path_count",
        path_keys=("global_path", "global_path_points", "path"),
    )
    local_path_count = _path_count(
        trace,
        samples,
        count_key="local_path_count",
        path_keys=("local_path", "local_path_points"),
    )

    data_presence_ok = (
        len(samples) >= thresholds["min_sample_count"]
        and global_path_count > 0
        and local_path_count > 0
        and odometry_count >= thresholds["min_odometry_count"]
    )
    command_replay_ok = (
        cmd_vel_count >= thresholds["min_cmd_vel_count"]
        and cmd_vel_nonzero > 0
        and cmd_vel_nonzero_ratio >= thresholds["min_cmd_vel_nonzero_ratio"]
    )
    odometry_replay_ok = odom_motion_m >= thresholds["min_odom_motion_m"]
    goal_deviation_ok = (
        final_distance_m is not None
        and final_distance_m <= thresholds["max_final_distance_m"]
    )
    tracking_deviation_ok = (
        tracking_error_p95_m is not None
        and tracking_error_final_m is not None
        and tracking_error_p95_m <= thresholds["max_tracking_error_p95_m"]
        and tracking_error_final_m <= thresholds["max_tracking_error_final_m"]
    )
    command_safety_ok = (
        trace.get("simulation_only") is True
        and _bool_false(trace, "real_robot_motion")
        and _bool_false(trace, "cmd_vel_sent_to_hardware")
    )

    checks = {
        "data_presence": _check(
            data_presence_ok,
            "sample/global_path/local_path/odometry evidence is incomplete",
            {
                "sample_count": len(samples),
                "global_path_count": global_path_count,
                "local_path_count": local_path_count,
                "odometry_count": odometry_count,
            },
        ),
        "command_replay": _check(
            command_replay_ok,
            "cmd_vel replay is missing or mostly zero",
            {
                "cmd_vel_count": cmd_vel_count,
                "cmd_vel_nonzero": cmd_vel_nonzero,
                "cmd_vel_nonzero_ratio": round(cmd_vel_nonzero_ratio, 4),
            },
        ),
        "odometry_replay": _check(
            odometry_replay_ok,
            "odometry motion is below threshold",
            {"odom_motion_m": round(odom_motion_m, 4), "odom_displacement_m": round(odom_displacement_m, 4)},
        ),
        "goal_deviation": _check(
            goal_deviation_ok,
            "final distance to goal is missing or above threshold",
            {"final_distance_m": None if final_distance_m is None else round(final_distance_m, 4)},
        ),
        "tracking_deviation": _check(
            tracking_deviation_ok,
            "tracking error is missing or above threshold",
            {
                "tracking_error_p95_m": None if tracking_error_p95_m is None else round(tracking_error_p95_m, 4),
                "tracking_error_final_m": None if tracking_error_final_m is None else round(tracking_error_final_m, 4),
            },
        ),
        "command_safety": _check(
            command_safety_ok,
            "simulation-only command boundary is not preserved",
            {
                "simulation_only": trace.get("simulation_only"),
                "real_robot_motion": trace.get("real_robot_motion"),
                "cmd_vel_sent_to_hardware": trace.get("cmd_vel_sent_to_hardware"),
            },
        ),
    }

    blockers = [
        f"{name}: {check['blocker']}"
        for name, check in checks.items()
        if check.get("ok") is not True
    ]
    return {
        "ok": not blockers,
        "passed": not blockers,
        "sample_count": len(samples),
        "global_path_count": global_path_count,
        "local_path_count": local_path_count,
        "cmd_vel_count": cmd_vel_count,
        "cmd_vel_nonzero": cmd_vel_nonzero,
        "cmd_vel_nonzero_ratio": round(cmd_vel_nonzero_ratio, 4),
        "odometry_count": odometry_count,
        "odom_motion_m": round(odom_motion_m, 4),
        "odom_displacement_m": round(odom_displacement_m, 4),
        "final_distance_m": None if final_distance_m is None else round(final_distance_m, 4),
        "tracking_error_p95_m": None if tracking_error_p95_m is None else round(tracking_error_p95_m, 4),
        "tracking_error_final_m": None if tracking_error_final_m is None else round(tracking_error_final_m, 4),
        "deviation_checks": checks,
        "remaining_gaps": blockers,
        "thresholds": thresholds,
    }


def build_report(
    *,
    trace_path: Path | None = None,
    fixture: bool = False,
    routecheck_report_path: Path | None = None,
    routecheck_phase: str = "candidate",
    topic_jsonl_path: Path | None = None,
    thresholds: dict[str, float] | None = None,
    write_trace: Path | None = None,
) -> dict[str, Any]:
    trace_source = (
        "fixture"
        if fixture
        else (
            str(routecheck_report_path)
            if routecheck_report_path
            else (
                str(topic_jsonl_path)
                if topic_jsonl_path
                else (str(trace_path) if trace_path else "")
            )
        )
    )
    load_error = ""
    if fixture:
        trace = build_fixture_trace()
    elif routecheck_report_path is not None:
        try:
            trace = build_routecheck_trace(routecheck_report_path, phase=routecheck_phase)
        except Exception as exc:
            trace = {}
            load_error = f"failed to build routecheck replay trace: {exc}"
    elif topic_jsonl_path is not None:
        try:
            trace = build_topic_jsonl_trace(topic_jsonl_path)
        except Exception as exc:
            trace = {}
            load_error = f"failed to build topic jsonl replay trace: {exc}"
    elif trace_path is None:
        trace = {}
        load_error = "trace path is missing"
    else:
        try:
            trace = _load_json(trace_path)
        except Exception as exc:
            trace = {}
            load_error = f"failed to load trace: {exc}"
    if trace and write_trace is not None:
        _write_json(write_trace, trace)

    evaluation = evaluate_trace(trace, thresholds=thresholds)
    blockers = list(evaluation["remaining_gaps"])
    if load_error:
        blockers.insert(0, load_error)

    ok = not blockers
    return {
        "schema_version": SCHEMA_VERSION,
        "source_schema_version": trace.get("schema_version") if isinstance(trace, dict) else None,
        "generated_at": time.time(),
        "ok": ok,
        "passed": ok,
        "trace_source": trace_source,
        "trace_kind": trace.get("trace_kind") if isinstance(trace, dict) else "",
        "trace_artifact": str(write_trace) if write_trace is not None and trace else "",
        "simulation_only": trace.get("simulation_only") is True,
        "real_robot_motion": bool(trace.get("real_robot_motion")) if trace else False,
        "cmd_vel_sent_to_hardware": bool(trace.get("cmd_vel_sent_to_hardware")) if trace else False,
        **{key: value for key, value in evaluation.items() if key not in {"ok", "passed", "remaining_gaps"}},
        "remaining_gaps": blockers,
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, default=None, help="Replay trace/report JSON to validate.")
    parser.add_argument("--routecheck-report", type=Path, default=None, help="Build a non-motion replay trace from a routecheck preflight summary.")
    parser.add_argument("--routecheck-phase", default="candidate", choices=("baseline", "candidate"))
    parser.add_argument("--topic-jsonl", type=Path, default=None, help="Build a replay trace from recorded topic JSONL events.")
    parser.add_argument("--fixture", action="store_true", help="Evaluate a deterministic passing fixture trace.")
    parser.add_argument("--write-fixture", type=Path, default=None, help="Write the deterministic fixture trace.")
    parser.add_argument("--write-trace", type=Path, default=None, help="Write the trace used for validation.")
    parser.add_argument("--json-out", type=Path, default=None, help="Report path; omit to print JSON.")
    parser.add_argument("--strict", action="store_true", help="Exit nonzero when validation is red.")
    parser.add_argument("--min-sample-count", type=float, default=DEFAULT_THRESHOLDS["min_sample_count"])
    parser.add_argument("--min-cmd-vel-count", type=float, default=DEFAULT_THRESHOLDS["min_cmd_vel_count"])
    parser.add_argument(
        "--min-cmd-vel-nonzero-ratio",
        type=float,
        default=DEFAULT_THRESHOLDS["min_cmd_vel_nonzero_ratio"],
    )
    parser.add_argument("--min-odometry-count", type=float, default=DEFAULT_THRESHOLDS["min_odometry_count"])
    parser.add_argument("--min-odom-motion-m", type=float, default=DEFAULT_THRESHOLDS["min_odom_motion_m"])
    parser.add_argument("--max-final-distance-m", type=float, default=DEFAULT_THRESHOLDS["max_final_distance_m"])
    parser.add_argument(
        "--max-tracking-error-p95-m",
        type=float,
        default=DEFAULT_THRESHOLDS["max_tracking_error_p95_m"],
    )
    parser.add_argument(
        "--max-tracking-error-final-m",
        type=float,
        default=DEFAULT_THRESHOLDS["max_tracking_error_final_m"],
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    thresholds = {
        "min_sample_count": args.min_sample_count,
        "min_cmd_vel_count": args.min_cmd_vel_count,
        "min_cmd_vel_nonzero_ratio": args.min_cmd_vel_nonzero_ratio,
        "min_odometry_count": args.min_odometry_count,
        "min_odom_motion_m": args.min_odom_motion_m,
        "max_final_distance_m": args.max_final_distance_m,
        "max_tracking_error_p95_m": args.max_tracking_error_p95_m,
        "max_tracking_error_final_m": args.max_tracking_error_final_m,
    }
    if args.write_fixture:
        _write_json(args.write_fixture, build_fixture_trace())
    report = build_report(
        trace_path=args.trace,
        fixture=args.fixture,
        routecheck_report_path=args.routecheck_report,
        routecheck_phase=args.routecheck_phase,
        topic_jsonl_path=args.topic_jsonl,
        thresholds=thresholds,
        write_trace=args.write_trace,
    )
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.json_out is not None:
        _write_json(args.json_out, report)
    else:
        print(rendered)
    return 1 if args.strict and report.get("ok") is not True else 0


if __name__ == "__main__":
    raise SystemExit(main())
