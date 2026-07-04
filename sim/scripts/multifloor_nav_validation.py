#!/usr/bin/env python3
"""Non-motion multi-floor navigation simulation validation.

The script is simulation-only. It builds a deterministic two-floor scene,
validates a LiDAR localization contract against a synthetic map cloud, runs
global planning, exercises global_path->local_path->cmd_vel->mux dataflow, and
checks frontier exploration candidates. It never starts robot services or sends
commands to real hardware.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
import math
import pickle
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry, Path as NavPath
from nav.services.plan.global_planner.service import GlobalPlanner
from nav.services.plan.global_planner.algorithm.pct.runtime.api import inspect_pct_runtime
from sim.engine.scenarios.multifloor_assets import (
    DEFAULT_GOAL,
    DEFAULT_START,
    build_multifloor_assets,
    sample_multifloor_map_points,
)


VALIDATION_LEVEL = "kinematic_module_ports"
VALIDATION_SCOPE = {
    "physical_gait_verified": False,
    "physical_stair_motion_verified": False,
    "cross_floor_physical_verified": False,
    "slam_verified": False,
    "real_lidar_verified": False,
    "ros2_topic_bridge_verified": False,
    "hardware_safety_interlock_verified": False,
}


@dataclass(frozen=True)
class RouteCase:
    name: str
    route_mode: str
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    description: str
    require_pct: bool = False
    min_required_z_max: float | None = None


ROUTE_CASES: dict[str, RouteCase] = {
    "same_floor": RouteCase(
        name="same_floor",
        route_mode="same_floor",
        start=DEFAULT_START,
        goal=(1.9, -1.25, 0.0),
        description="lower-floor room route",
    ),
    "lower_approach": RouteCase(
        name="lower_approach",
        route_mode="same_floor",
        start=DEFAULT_START,
        goal=(2.2, -1.6, 0.0),
        description="lower-floor approach to the stair entry",
    ),
    "upper_floor": RouteCase(
        name="upper_floor",
        route_mode="same_floor",
        start=(4.3, 1.4, 2.5),
        goal=DEFAULT_GOAL,
        description="upper-floor route after the stair exit",
    ),
    "cross_floor": RouteCase(
        name="cross_floor",
        route_mode="cross_floor",
        start=DEFAULT_START,
        goal=DEFAULT_GOAL,
        description="lower floor to upper floor through the stair transition",
        require_pct=True,
        min_required_z_max=2.0,
    ),
}

MATRIX_ROUTES = ("same_floor", "lower_approach", "upper_floor", "cross_floor")


def _validation_scope() -> dict[str, Any]:
    return {
        "validation_level": VALIDATION_LEVEL,
        **VALIDATION_SCOPE,
    }


def _tomogram_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _pct_runtime_evidence() -> dict[str, Any]:
    try:
        info = inspect_pct_runtime(ROOT)
        evidence = {
            "ok": bool(info.get("ok")),
            "canonical_arch": info.get("canonical_arch"),
            "python_tag": info.get("python_tag"),
            "lib_dir": info.get("lib_dir"),
            "missing": info.get("missing", []),
            "shared_missing": info.get("shared_missing", []),
            "error": info.get("error", ""),
        }
        for key in (
            "known_good_python_tag",
            "python_abi_matches_known_good",
            "platform_system",
            "os_name",
            "native_binary_format",
            "host_platform_supported",
            "host_platform_blocker",
            "candidate_diagnostics",
            "recommended_build_command",
        ):
            if key in info:
                evidence[key] = info[key]
        return evidence
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _is_production_local_planner_backend(backend: str) -> bool:
    return backend in {"nanobind", "cmu", "cmu_py"}


def _algorithm_backend_evidence(
    module: object,
    *,
    requested: str,
    exercised_by: str,
    native_backend: str | None = None,
) -> dict[str, Any]:
    status = getattr(module, "_backend_status", None)
    actual = str(
        getattr(status, "effective", "")
        or getattr(module, "_backend", "")
        or requested
    )
    configured = str(getattr(status, "configured", "") or requested)
    degraded = bool(getattr(status, "degraded", False) or actual != requested)
    degraded_reason = str(
        getattr(status, "degraded_reason", "") or getattr(status, "reason", "") or ""
    )
    if degraded and not degraded_reason and actual != requested:
        degraded_reason = f"effective backend changed from {requested} to {actual}"
    evidence: dict[str, Any] = {
        "requested": requested,
        "configured_backend": configured,
        "backend_actual": actual,
        "degraded": degraded,
        "degraded_reason": degraded_reason,
        "exercised_by": exercised_by,
    }
    if native_backend is not None:
        evidence["native_backend_used"] = actual == native_backend
    return evidence


def _safe_norm(points: np.ndarray) -> np.ndarray:
    arr = np.asarray(points, dtype=np.float64)
    return arr[np.all(np.isfinite(arr), axis=1)]


def _yaw_matrix(yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.asarray([[c, -s], [s, c]], dtype=np.float64)


def _scan_from_pose(
    map_points: np.ndarray,
    pose: tuple[float, float, float, float],
    *,
    radius: float = 5.0,
    max_points: int = 420,
) -> np.ndarray:
    x, y, z, yaw = pose
    points = _safe_norm(map_points)
    delta = points - np.asarray([x, y, z], dtype=np.float64)
    keep = np.linalg.norm(delta[:, :2], axis=1) <= radius
    local = delta[keep]
    if len(local) > max_points:
        idx = np.linspace(0, len(local) - 1, max_points).astype(int)
        local = local[idx]
    rot = _yaw_matrix(-yaw)
    local[:, :2] = local[:, :2] @ rot.T
    return local


def _nearest_rms(a: np.ndarray, b: np.ndarray) -> float:
    if len(a) == 0 or len(b) == 0:
        return float("inf")
    # Chunked brute-force keeps the dependency surface numpy-only.
    mins: list[np.ndarray] = []
    for start in range(0, len(a), 256):
        chunk = a[start : start + 256]
        d2 = np.sum((chunk[:, None, :] - b[None, :, :]) ** 2, axis=2)
        mins.append(np.min(d2, axis=1))
    return float(np.sqrt(np.mean(np.concatenate(mins))))


def _score_pose_alignment(
    scan_body: np.ndarray,
    map_points: np.ndarray,
    pose: tuple[float, float, float, float],
) -> float:
    x, y, z, yaw = pose
    rot = _yaw_matrix(yaw)
    world = np.asarray(scan_body, dtype=np.float64).copy()
    world[:, :2] = world[:, :2] @ rot.T
    world += np.asarray([x, y, z], dtype=np.float64)
    return _nearest_rms(world, map_points)


def _offset_pose(
    pose: tuple[float, float, float, float],
    offset: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return tuple(float(pose[i] + offset[i]) for i in range(4))  # type: ignore[return-value]


def _xy_drift_m(offset: tuple[float, float, float, float]) -> float:
    return float(math.hypot(offset[0], offset[1]))


def _run_lidar_localization_replay(
    map_points: np.ndarray,
    poses: list[tuple[float, float, float, float]],
) -> dict[str, Any]:
    replay_plan = [
        {
            "phase": "initial_lock",
            "pose": poses[0],
            "offset": (0.0, 0.0, 0.0, 0.0),
            "recovery_action": "none",
        },
        {
            "phase": "nominal_tracking",
            "pose": (1.7, -0.9, 0.45, 0.28),
            "offset": (0.04, -0.03, 0.0, 0.015),
            "recovery_action": "none",
        },
        {
            "phase": "drift_injected",
            "pose": poses[1],
            "offset": (0.72, -0.48, 0.0, 0.28),
            "recovery_action": "hold_last_pose",
        },
        {
            "phase": "scan_map_relocalization",
            "pose": poses[1],
            "offset": (0.0, 0.0, 0.0, 0.0),
            "recovery_action": "scan_to_map_reset",
        },
        {
            "phase": "post_recovery_tracking",
            "pose": poses[2],
            "offset": (0.03, 0.02, 0.0, -0.01),
            "recovery_action": "none",
        },
    ]

    frames: list[dict[str, Any]] = []
    transitions: list[dict[str, Any]] = []
    previous_state: str | None = None
    recovery_count = 0
    max_drift_m = 0.0
    ok = True
    for frame_id, step in enumerate(replay_plan):
        true_pose = step["pose"]
        offset = step["offset"]
        estimated_pose = _offset_pose(true_pose, offset)
        scan = _scan_from_pose(map_points, true_pose)
        aligned_rms = _score_pose_alignment(scan, map_points, true_pose)
        estimated_rms = _score_pose_alignment(scan, map_points, estimated_pose)
        drift_m = _xy_drift_m(offset)
        max_drift_m = max(max_drift_m, drift_m)

        if previous_state == "LOST" and estimated_rms < 0.045 and step["recovery_action"] == "scan_to_map_reset":
            state = "RECOVERED"
            recovery_count += 1
        elif estimated_rms < 0.08:
            state = "LOCKED"
        else:
            state = "LOST"

        if previous_state is not None and previous_state != state:
            transitions.append(
                {
                    "frame": frame_id,
                    "from": previous_state,
                    "to": state,
                    "phase": step["phase"],
                }
            )
        previous_state = state
        ok = ok and state in {"LOCKED", "LOST", "RECOVERED"}
        frames.append(
            {
                "frame": frame_id,
                "phase": step["phase"],
                "true_pose": [round(float(v), 4) for v in true_pose],
                "estimated_pose": [round(float(v), 4) for v in estimated_pose],
                "scan_points": int(len(scan)),
                "aligned_rms_m": round(aligned_rms, 5),
                "estimated_rms_m": round(estimated_rms, 5),
                "drift_m": round(drift_m, 4),
                "yaw_drift_rad": round(float(abs(offset[3])), 4),
                "localization_state": state,
                "recovery_action": step["recovery_action"],
            }
        )

    final_state = "LOCKED" if frames[-1]["localization_state"] in {"LOCKED", "RECOVERED"} else frames[-1]["localization_state"]
    expected_states = [frame["localization_state"] for frame in frames]
    ok = (
        ok
        and expected_states == ["LOCKED", "LOCKED", "LOST", "RECOVERED", "LOCKED"]
        and recovery_count == 1
        and final_state == "LOCKED"
    )
    return {
        "ok": ok,
        "replay_source": "deterministic_simulated_scan_pose_sequence",
        "not_fastlio2_rosbag": True,
        "frames": frames,
        "transitions": transitions,
        "drift_m": round(max_drift_m, 4),
        "recovery_count": recovery_count,
        "final_state": final_state,
    }


def run_lidar_localization_contract() -> dict[str, Any]:
    map_points = sample_multifloor_map_points()
    rng = np.random.default_rng(17)
    poses = [
        (DEFAULT_START[0], DEFAULT_START[1], DEFAULT_START[2] + 0.45, 0.15),
        (3.1, -0.2, 1.25, 0.85),
        (DEFAULT_GOAL[0], DEFAULT_GOAL[1], DEFAULT_GOAL[2] + 0.45, -0.25),
    ]
    samples: list[dict[str, Any]] = []
    robustness_samples: list[dict[str, Any]] = []
    rejection_samples: list[dict[str, Any]] = []
    locked = True
    for pose in poses:
        scan = _scan_from_pose(map_points, pose)
        aligned = _score_pose_alignment(scan, map_points, pose)
        drift_pose = (pose[0] + 0.55, pose[1] - 0.35, pose[2], pose[3] + 0.22)
        drifted = _score_pose_alignment(scan, map_points, drift_pose)
        state = "LOCKED" if aligned < 0.035 and drifted > aligned + 0.20 else "LOST"
        locked = locked and state == "LOCKED"
        samples.append(
            {
                "pose": [round(float(v), 4) for v in pose],
                "scan_points": int(len(scan)),
                "aligned_rms_m": round(aligned, 5),
                "drifted_rms_m": round(drifted, 5),
                "localization_state": state,
            }
        )
        sparse_noisy = scan[::3].copy()
        if len(sparse_noisy):
            sparse_noisy += rng.normal(0.0, 0.012, size=sparse_noisy.shape)
        sparse_aligned = _score_pose_alignment(sparse_noisy, map_points, pose)
        sparse_state = "LOCKED" if len(sparse_noisy) >= 80 and sparse_aligned < 0.065 else "LOST"
        locked = locked and sparse_state == "LOCKED"
        robustness_samples.append(
            {
                "pose": [round(float(v), 4) for v in pose],
                "scan_points": int(len(sparse_noisy)),
                "noise_sigma_m": 0.012,
                "aligned_rms_m": round(sparse_aligned, 5),
                "localization_state": sparse_state,
            }
        )
    first_pose = poses[0]
    first_scan = _scan_from_pose(map_points, first_pose)
    for name, candidate in [
        ("wrong_xy_yaw", (first_pose[0] + 1.25, first_pose[1] + 1.05, first_pose[2], first_pose[3] + 0.7)),
        ("wrong_floor", (first_pose[0], first_pose[1], first_pose[2] + 2.5, first_pose[3])),
    ]:
        rms = _score_pose_alignment(first_scan, map_points, candidate)
        state = "LOST" if rms > 0.16 else "LOCKED"
        locked = locked and state == "LOST"
        rejection_samples.append(
            {
                "name": name,
                "candidate_pose": [round(float(v), 4) for v in candidate],
                "aligned_rms_m": round(rms, 5),
                "localization_state": state,
            }
        )
    replay = _run_lidar_localization_replay(map_points, poses)
    return {
        "ok": locked and replay["ok"],
        "backend": "synthetic_lidar_map_alignment",
        "health_source": "simulated_mid360_scan_vs_multifloor_map_cloud",
        "replay_source": replay["replay_source"],
        "not_fastlio2_rosbag": replay["not_fastlio2_rosbag"],
        "real_fastlio2_rosbag_verified": False,
        "odom_frame": "map",
        "map_cloud_points": int(len(map_points)),
        "samples": samples,
        "robustness_samples": robustness_samples,
        "rejection_samples": rejection_samples,
        "frames": replay["frames"],
        "transitions": replay["transitions"],
        "drift_m": replay["drift_m"],
        "recovery_count": replay["recovery_count"],
        "final_state": replay["final_state"],
    }


def _path_points(path: list[np.ndarray]) -> list[list[float]]:
    return [[float(p[0]), float(p[1]), float(p[2])] for p in path]


def _load_tomogram(path: Path) -> dict[str, Any]:
    with Path(path).open("rb") as fh:
        raw = pickle.load(fh)
    if not isinstance(raw, dict) or "data" not in raw:
        raise ValueError(f"unexpected tomogram format: {path}")
    return raw


def _tomogram_point_indices(
    raw: dict[str, Any],
    point: list[float],
) -> tuple[int, int] | None:
    data = np.asarray(raw["data"])
    if data.ndim != 4:
        return None
    _channels, _slices, x_cells, y_cells = data.shape
    res = float(raw.get("resolution", 0.2))
    origin = raw.get("origin") or raw.get("grid_info", {}).get("origin")
    if origin is None:
        center = np.asarray(raw.get("center", [0.0, 0.0])[:2], dtype=np.float64)
        origin_arr = center - np.asarray([x_cells * res / 2.0, y_cells * res / 2.0], dtype=np.float64)
    else:
        origin_arr = np.asarray(origin[:2], dtype=np.float64)
    xi = int(round((float(point[0]) - origin_arr[0]) / res))
    yi = int(round((float(point[1]) - origin_arr[1]) / res))
    if xi < 0 or xi >= x_cells or yi < 0 or yi >= y_cells:
        return None
    return xi, yi


def validate_path_against_tomogram(
    tomogram: Path,
    path_points: list[list[float]],
    *,
    obstacle_thr: float = 49.9,
    body_height_offset_m: float = 0.60,
    z_tolerance_m: float = 0.85,
) -> dict[str, Any]:
    if not path_points:
        return {"ok": False, "reason": "empty_path", "point_count": 0}
    try:
        raw = _load_tomogram(tomogram)
    except Exception as exc:
        return {"ok": False, "reason": "tomogram_load_failed", "error": str(exc), "point_count": len(path_points)}

    data = np.asarray(raw["data"], dtype=np.float32)
    trav = data[0]
    elev_g = data[3] if data.shape[0] > 3 else np.zeros_like(trav)
    invalid: list[dict[str, Any]] = []
    checked = 0
    for idx, point in enumerate(path_points):
        indices = _tomogram_point_indices(raw, point)
        if indices is None:
            invalid.append({"index": idx, "point": point, "reason": "xy_out_of_bounds"})
            continue
        xi, yi = indices
        free_slices = np.where(trav[:, xi, yi] <= obstacle_thr)[0]
        if len(free_slices) == 0:
            invalid.append({"index": idx, "point": point, "reason": "occupied_xy"})
            continue
        z = float(point[2]) if len(point) > 2 else 0.0
        expected = elev_g[free_slices, xi, yi] + body_height_offset_m
        z_errors = np.abs(expected - z)
        if float(np.min(z_errors)) > z_tolerance_m:
            invalid.append(
                {
                    "index": idx,
                    "point": point,
                    "reason": "z_not_near_traversable_slice",
                    "nearest_z_error_m": round(float(np.min(z_errors)), 4),
                }
            )
            continue
        checked += 1
    return {
        "ok": not invalid,
        "point_count": len(path_points),
        "checked_count": checked,
        "invalid_count": len(invalid),
        "invalid_examples": invalid[:5],
    }


def _point_segment_distance(
    point_xy: np.ndarray,
    start_xy: np.ndarray,
    end_xy: np.ndarray,
) -> float:
    delta = end_xy - start_xy
    denom = float(np.dot(delta, delta))
    if denom <= 1e-12:
        return float(np.linalg.norm(point_xy - start_xy))
    alpha = float(np.clip(np.dot(point_xy - start_xy, delta) / denom, 0.0, 1.0))
    nearest = start_xy + alpha * delta
    return float(np.linalg.norm(point_xy - nearest))


def validate_transition_against_metadata(
    tomogram: Path,
    transition_path: list[list[float]],
    *,
    xy_tolerance_m: float = 0.15,
    z_tolerance_m: float = 0.15,
) -> dict[str, Any]:
    if not transition_path:
        return {"ok": False, "reason": "missing_transition"}
    try:
        raw = _load_tomogram(tomogram)
    except Exception as exc:
        return {"ok": False, "reason": "tomogram_load_failed", "error": str(exc)}
    transitions = raw.get("floor_transitions") or []
    floors = [float(v) for v in raw.get("floor_heights", [])]
    if not transitions or len(floors) < 2:
        return {"ok": False, "reason": "missing_floor_transition_metadata"}
    transition = transitions[0]
    lower = np.asarray(transition["lower_xy"], dtype=np.float64)
    upper = np.asarray(transition["upper_xy"], dtype=np.float64)
    width = float(transition.get("width_m", 0.0))
    distances = [
        _point_segment_distance(np.asarray(point[:2], dtype=np.float64), lower, upper)
        for point in transition_path
    ]
    zs = [float(point[2]) if len(point) > 2 else 0.0 for point in transition_path]
    max_distance = max(distances) if distances else float("inf")
    z_min = min(zs) if zs else float("inf")
    z_max = max(zs) if zs else float("-inf")
    allowed_distance = width * 0.5 + xy_tolerance_m
    ok = (
        max_distance <= allowed_distance
        and z_min <= min(floors) + z_tolerance_m
        and z_max >= max(floors) - z_tolerance_m
    )
    return {
        "ok": ok,
        "transition_name": transition.get("name"),
        "kind": transition.get("kind"),
        "point_count": len(transition_path),
        "max_centerline_distance_m": round(float(max_distance), 4),
        "allowed_distance_m": round(float(allowed_distance), 4),
        "z_min": round(float(z_min), 4),
        "z_max": round(float(z_max), 4),
        "floor_heights": floors,
    }


def _planner_status(
    *,
    planner: str,
    feasible: bool,
    backend_available: bool,
    native_runtime: dict[str, Any] | None,
    error: str,
) -> str:
    if feasible:
        return "pass"
    if planner == "pct":
        runtime_ok = bool(native_runtime.get("ok")) if isinstance(native_runtime, dict) else False
        error_text = str(error or "").lower()
        if (
            not backend_available
            or not runtime_ok
            or "tomogram file not found" in error_text
            or "no runnable pct native modules" in error_text
            or "pct runtime/loadtomogram failed" in error_text
        ):
            return "blocked"
    return "fail"


def _service_plan_report(svc: Any | None) -> dict[str, Any]:
    if svc is None:
        return {}
    try:
        report = getattr(svc, "last_plan_report", {}) or {}
    except Exception:
        return {}
    return dict(report) if isinstance(report, dict) else {}


def _planner_value(value: Any, default: str) -> str:
    return str(value or default).lower().strip()


def run_global_planner(
    *,
    planner: str,
    tomogram: Path,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    downsample_dist: float,
    safe_goal_tolerance: float = 0.0,
) -> dict[str, Any]:
    t0 = time.time()
    svc: GlobalPlanner | None = None
    backend: Any | None = None
    native_runtime = _pct_runtime_evidence() if planner == "pct" else None
    try:
        svc = GlobalPlanner(
            planner_name=planner,
            tomogram=str(tomogram),
            obstacle_thr=49.9,
            downsample_dist=downsample_dist,
        )
        svc.setup()
        backend = getattr(svc, "_backend", None)
        path, plan_ms = svc.plan(
            np.asarray(start, dtype=float),
            np.asarray(goal, dtype=float),
            safe_goal_tolerance=safe_goal_tolerance,
        )
        plan_report = _service_plan_report(svc)
        pts = _path_points(path)
        arr = np.asarray(pts, dtype=np.float64)
        distance = float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1))) if len(arr) > 1 else 0.0
        backend_reached_goal_raw = plan_report.get("reached_goal", True)
        backend_reached_goal = (
            bool(backend_reached_goal_raw)
            if backend_reached_goal_raw is not None
            else True
        )
        safe_goal = plan_report.get("safe_goal") or (pts[-1] if pts else None)
        safe_goal_distance_m: float | None = None
        if safe_goal is not None:
            try:
                safe_goal_xy = np.asarray(safe_goal[:2], dtype=np.float64)
                requested_goal_xy = np.asarray(goal[:2], dtype=np.float64)
                safe_goal_distance_m = float(np.linalg.norm(safe_goal_xy - requested_goal_xy))
            except Exception:
                safe_goal_distance_m = None
        projection_limit_m = max(0.35, float(safe_goal_tolerance))
        goal_projection_within_tolerance = (
            safe_goal_distance_m is None or safe_goal_distance_m <= projection_limit_m
        )
        reached_goal = bool(backend_reached_goal and goal_projection_within_tolerance)
        feasible = bool(pts) and reached_goal
        selected_planner = _planner_value(plan_report.get("selected_planner"), planner)
        planner_requested = _planner_value(plan_report.get("primary_planner"), planner)
        selected_backend = backend
        if selected_planner != planner:
            selected_backend = getattr(svc, "_fallback_backend", None) or backend
        backend_available = bool(getattr(selected_backend, "available", True))
        requested_backend_available = bool(getattr(backend, "available", True))
        error = None
        if not pts:
            error = "planner returned empty path"
        elif not backend_reached_goal:
            error = "planner returned partial path without reaching goal"
        elif not goal_projection_within_tolerance:
            error = "planner safe goal is too far from requested goal"
        status = _planner_status(
            planner=planner,
            feasible=feasible,
            backend_available=backend_available,
            native_runtime=native_runtime,
            error=error or "",
        )
        return {
            "planner": planner,
            "planner_requested": planner_requested,
            "selected_planner": selected_planner,
            "fallback_reason": str(plan_report.get("fallback_reason") or ""),
            "plan_safety_policy": str(plan_report.get("policy") or ""),
            "rejected_plans": (
                plan_report.get("rejected_plans")
                if isinstance(plan_report.get("rejected_plans"), list)
                else []
            ),
            "status": status,
            "blocked": status == "blocked",
            "ok": feasible,
            "feasible": feasible,
            "backend_available": backend_available,
            "backend_requested_available": requested_backend_available,
            "planner_class": selected_backend.__class__.__name__ if selected_backend is not None else None,
            "backend_requested_class": backend.__class__.__name__ if backend is not None else None,
            "native_runtime": native_runtime,
            "native_backend_used": (
                selected_planner == "pct"
                and bool(getattr(selected_backend, "available", False))
                and feasible
            ),
            "tomogram": str(tomogram),
            "tomogram_sha256": _tomogram_sha256(tomogram),
            "planner_scope": (
                "native_pct_single_route"
                if planner == "pct"
                else "ground_floor_dev_sim_astar"
            ),
            "safe_goal_tolerance": float(safe_goal_tolerance),
            "safe_goal": safe_goal,
            "safe_goal_distance_m": (
                round(float(safe_goal_distance_m), 4)
                if safe_goal_distance_m is not None
                else None
            ),
            "safe_goal_projection_limit_m": round(float(projection_limit_m), 4),
            "goal_projection_within_tolerance": goal_projection_within_tolerance,
            "backend_reached_goal": backend_reached_goal,
            "reached_goal": reached_goal,
            "count": len(pts),
            "distance_m": round(distance, 4),
            "plan_ms": round(float(plan_ms), 3),
            "wall_ms": round((time.time() - t0) * 1000.0, 3),
            "z_min": round(float(np.min(arr[:, 2])), 4) if len(arr) else None,
            "z_max": round(float(np.max(arr[:, 2])), 4) if len(arr) else None,
            "start": list(start),
            "goal": list(goal),
            "first": pts[0] if pts else None,
            "last": pts[-1] if pts else None,
            "path": pts,
            "error": error,
        }
    except Exception as exc:
        error = str(exc)
        plan_report = _service_plan_report(svc)
        selected_planner = _planner_value(plan_report.get("selected_planner"), planner)
        planner_requested = _planner_value(plan_report.get("primary_planner"), planner)
        backend_available = bool(getattr(backend, "available", False)) if backend is not None else False
        load_error = str(getattr(backend, "_load_error", "") or "")
        status = _planner_status(
            planner=planner,
            feasible=False,
            backend_available=backend_available,
            native_runtime=native_runtime,
            error="; ".join(item for item in (error, load_error) if item),
        )
        return {
            "planner": planner,
            "planner_requested": planner_requested,
            "selected_planner": selected_planner,
            "fallback_reason": str(plan_report.get("fallback_reason") or ""),
            "plan_safety_policy": str(plan_report.get("policy") or ""),
            "rejected_plans": (
                plan_report.get("rejected_plans")
                if isinstance(plan_report.get("rejected_plans"), list)
                else []
            ),
            "status": status,
            "blocked": status == "blocked",
            "ok": False,
            "feasible": False,
            "backend_available": backend_available,
            "planner_class": backend.__class__.__name__ if backend is not None else None,
            "backend_requested_available": backend_available,
            "backend_requested_class": backend.__class__.__name__ if backend is not None else None,
            "native_runtime": native_runtime,
            "native_backend_used": False,
            "tomogram": str(tomogram),
            "tomogram_sha256": _tomogram_sha256(tomogram) if Path(tomogram).exists() else "",
            "planner_scope": (
                "native_pct_single_route"
                if planner == "pct"
                else "ground_floor_dev_sim_astar"
            ),
            "safe_goal_tolerance": float(safe_goal_tolerance),
            "safe_goal": None,
            "safe_goal_distance_m": None,
            "safe_goal_projection_limit_m": round(float(max(0.35, safe_goal_tolerance)), 4),
            "goal_projection_within_tolerance": False,
            "backend_reached_goal": False,
            "reached_goal": False,
            "count": 0,
            "distance_m": 0.0,
            "wall_ms": round((time.time() - t0) * 1000.0, 3),
            "start": list(start),
            "goal": list(goal),
            "path": [],
            "error": error,
        }


def _stair_transition_endpoints(tomogram: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    with Path(tomogram).open("rb") as fh:
        raw = pickle.load(fh)
    transitions = raw.get("floor_transitions") or []
    if not transitions:
        return (3.2, -0.1, 0.0), (3.2, -0.1, 2.5)
    transition = transitions[0]
    floor_heights = raw.get("floor_heights") or [0.0, 2.5]
    lower_floor = int(transition.get("floor_from", 0))
    upper_floor = int(transition.get("floor_to", 1))
    lower_xy = transition.get("lower_xy", (3.2, -0.1))
    upper_xy = transition.get("upper_xy", lower_xy)
    lower_z = float(floor_heights[lower_floor]) if lower_floor < len(floor_heights) else 0.0
    upper_z = float(floor_heights[upper_floor]) if upper_floor < len(floor_heights) else 2.5
    return (
        (float(lower_xy[0]), float(lower_xy[1]), lower_z),
        (float(upper_xy[0]), float(upper_xy[1]), upper_z),
    )


def _stair_transition_path(
    lower: tuple[float, float, float],
    upper: tuple[float, float, float],
) -> list[list[float]]:
    lower_arr = np.asarray(lower, dtype=np.float64)
    upper_arr = np.asarray(upper, dtype=np.float64)
    return [
        [float(v) for v in (lower_arr + (upper_arr - lower_arr) * alpha)]
        for alpha in np.linspace(0.0, 1.0, 7)
    ]


def run_route_planner(
    *,
    planner: str,
    route: str,
    tomogram: Path,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    downsample_dist: float,
) -> dict[str, Any]:
    if route != "cross_floor" or planner != "pct":
        return run_global_planner(
            planner=planner,
            tomogram=tomogram,
            start=start,
            goal=goal,
            downsample_dist=downsample_dist,
        )

    # The original PCT planner plans within one traversable layer. Multi-floor
    # navigation is therefore validated as a floor graph: per-floor PCT route
    # segments plus an explicit stairs transition edge.
    lower_stair, upper_stair = _stair_transition_endpoints(tomogram)
    transition = _stair_transition_path(lower_stair, upper_stair)
    segment_specs = [
        ("floor_0_to_stairs", start, lower_stair),
        ("floor_1_from_stairs", upper_stair, goal),
    ]
    segments: list[dict[str, Any]] = []
    combined: list[list[float]] = []
    for name, seg_start, seg_goal in segment_specs:
        result = run_global_planner(
            planner=planner,
            tomogram=tomogram,
            start=seg_start,
            goal=seg_goal,
            downsample_dist=downsample_dist,
        )
        result["name"] = name
        segments.append(result)
        if not result.get("feasible"):
            return {
                "planner": planner,
                "status": result.get("status", "fail"),
                "blocked": bool(result.get("blocked")),
                "ok": False,
                "feasible": False,
                "backend_available": bool(result.get("backend_available")),
                "planner_class": result.get("planner_class"),
                "native_runtime": result.get("native_runtime"),
                "native_backend_used": False,
                "tomogram": str(tomogram),
                "tomogram_sha256": _tomogram_sha256(tomogram),
                "planner_scope": "floor_graph_pct_composition",
                "route_strategy": "floor_graph_pct_segments",
                "native_pct_single_plan_verified": False,
                "cross_floor_physical_verified": False,
                "transition_motion_verified": False,
                "native_pct_required_segments": len(segment_specs),
                "native_pct_feasible_segments": sum(
                    1 for segment in segments if segment.get("feasible") and segment.get("native_backend_used")
                ),
                "count": 0,
                "distance_m": 0.0,
                "start": list(start),
                "goal": list(goal),
                "path": [],
                "segments": segments,
                "transition": transition,
                "error": result.get("error") or f"segment failed: {name}",
            }
        points = result["path"]
        if combined and points and np.allclose(combined[-1], points[0]):
            combined.extend(points[1:])
        else:
            combined.extend(points)
        if name == "floor_0_to_stairs":
            if combined and transition and np.allclose(combined[-1], transition[0]):
                combined.extend(transition[1:])
            else:
                combined.extend(transition)

    arr = np.asarray(combined, dtype=np.float64)
    distance = float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1))) if len(arr) > 1 else 0.0
    return {
        "planner": planner,
        "status": "pass",
        "blocked": False,
        "ok": True,
        "feasible": True,
        "backend_available": all(bool(s.get("backend_available")) for s in segments),
        "planner_class": "floor_graph_pct_segments",
        "native_runtime": next((s.get("native_runtime") for s in segments if s.get("native_runtime")), None),
        "native_backend_used": all(bool(s.get("native_backend_used")) for s in segments),
        "tomogram": str(tomogram),
        "tomogram_sha256": _tomogram_sha256(tomogram),
        "planner_scope": "floor_graph_pct_composition",
        "native_pct_single_plan_verified": False,
        "cross_floor_physical_verified": False,
        "transition_motion_verified": False,
        "native_pct_required_segments": len(segment_specs),
        "native_pct_feasible_segments": sum(
            1 for segment in segments if segment.get("feasible") and segment.get("native_backend_used")
        ),
        "count": len(combined),
        "distance_m": round(distance, 4),
        "plan_ms": round(sum(float(s.get("plan_ms") or 0.0) for s in segments), 3),
        "wall_ms": round(sum(float(s.get("wall_ms") or 0.0) for s in segments), 3),
        "z_min": round(float(np.min(arr[:, 2])), 4) if len(arr) else None,
        "z_max": round(float(np.max(arr[:, 2])), 4) if len(arr) else None,
        "start": list(start),
        "goal": list(goal),
        "first": combined[0] if combined else None,
        "last": combined[-1] if combined else None,
        "path": combined,
        "segments": segments,
        "transition": transition,
        "route_strategy": "floor_graph_pct_segments",
        "error": None,
    }


def _pose_stamped(point: list[float], frame_id: str = "map") -> PoseStamped:
    return PoseStamped(
        pose=Pose(
            position=Vector3(point[0], point[1], point[2] if len(point) > 2 else 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        frame_id=frame_id,
        ts=time.time(),
    )


def _pose_with_yaw(point: list[float], yaw: float) -> Pose:
    return Pose(
        position=Vector3(point[0], point[1], point[2] if len(point) > 2 else 0.0),
        orientation=Quaternion.from_yaw(yaw),
    )


def _yaw_between(a: list[float], b: list[float], fallback: float = 0.0) -> float:
    dx = float(b[0]) - float(a[0])
    dy = float(b[1]) - float(a[1])
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return fallback
    return math.atan2(dy, dx)


def _resample_path_for_replay(
    path_points: list[list[float]],
    *,
    step_m: float = 0.25,
) -> list[dict[str, Any]]:
    if len(path_points) < 2:
        return []
    samples: list[dict[str, Any]] = []
    last_yaw = _yaw_between(path_points[0], path_points[1])
    for idx in range(len(path_points) - 1):
        start = np.asarray(path_points[idx], dtype=np.float64)
        end = np.asarray(path_points[idx + 1], dtype=np.float64)
        xy_dist = float(np.linalg.norm(end[:2] - start[:2]))
        yaw = _yaw_between(path_points[idx], path_points[idx + 1], fallback=last_yaw)
        last_yaw = yaw
        count = max(1, int(math.ceil(xy_dist / step_m))) if xy_dist > 1e-6 else 1
        for n in range(count):
            if samples and n == 0:
                continue
            alpha = float(n / count)
            point = start + (end - start) * alpha
            # Add a bounded lateral offset so replay exercises steering instead
            # of feeding a perfectly zero-error path.
            lateral = 0.035 * math.sin((len(samples) + 1) * 0.7)
            point[0] += -math.sin(yaw) * lateral
            point[1] += math.cos(yaw) * lateral
            samples.append({"point": [float(v) for v in point], "yaw": yaw})
    samples.append({"point": [float(v) for v in path_points[-1]], "yaw": last_yaw})
    return samples


def _nearest_xy_error(point: list[float], path_points: list[list[float]]) -> float:
    if not path_points:
        return float("inf")
    p = np.asarray(point[:2], dtype=np.float64)
    pts = [np.asarray(item[:2], dtype=np.float64) for item in path_points]
    best = float("inf")
    for idx in range(len(pts) - 1):
        a = pts[idx]
        b = pts[idx + 1]
        best = min(best, _point_segment_distance(p, a, b))
    if len(pts) == 1:
        best = float(np.linalg.norm(p - pts[0]))
    return best


def _nav_path_from_points(path_points: list[list[float]]) -> NavPath:
    poses: list[PoseStamped] = []
    yaw = 0.0
    for idx, point in enumerate(path_points):
        if idx + 1 < len(path_points):
            yaw = _yaw_between(point, path_points[idx + 1], fallback=yaw)
        poses.append(
            PoseStamped(
                pose=_pose_with_yaw(point, yaw),
                frame_id="map",
                ts=time.time(),
            )
        )
    return NavPath(poses=poses, frame_id="map")


def _thin_nav_path(path: NavPath, *, min_xy_step: float = 0.12) -> NavPath:
    if len(path.poses) <= 2:
        return path
    kept = [path.poses[0]]
    last_xy = np.asarray([
        path.poses[0].pose.position.x,
        path.poses[0].pose.position.y,
    ], dtype=float)
    for pose in path.poses[1:-1]:
        xy = np.asarray([pose.pose.position.x, pose.pose.position.y], dtype=float)
        if float(np.linalg.norm(xy - last_xy)) >= min_xy_step:
            kept.append(pose)
            last_xy = xy
    if kept[-1] is not path.poses[-1]:
        kept.append(path.poses[-1])
    return NavPath(poses=kept, frame_id=path.frame_id)


def _prepare_local_path_for_follower(
    path: NavPath,
    *,
    current_xy: np.ndarray | None = None,
    min_xy_step: float = 0.12,
    min_start_dist: float = 0.20,
) -> NavPath:
    """Make dense local-planner paths stable for nav_kernel follower replay."""
    prepared = _thin_nav_path(path, min_xy_step=min_xy_step)
    if current_xy is None or len(prepared.poses) <= 2:
        return prepared

    first_keep = 0
    for idx, pose in enumerate(prepared.poses[:-1]):
        xy = np.asarray([pose.pose.position.x, pose.pose.position.y], dtype=float)
        if float(np.linalg.norm(xy - current_xy)) >= min_start_dist:
            first_keep = idx
            break
    else:
        first_keep = max(0, len(prepared.poses) - 2)

    if first_keep <= 0:
        return prepared
    return NavPath(poses=prepared.poses[first_keep:], frame_id=prepared.frame_id)


def run_tracking_replay(
    path_points: list[list[float]],
    *,
    backend: str = "nav_kernel",
    max_speed: float = 0.25,
    max_abs_yaw_rate: float = 1.2,
    max_tracking_error_m: float = 0.12,
) -> dict[str, Any]:
    if len(path_points) < 2:
        return {"ok": False, "error": "need at least two path points"}

    from nav.local.path_follower import PathFollower

    follower = PathFollower(
        backend=backend,
        max_speed=max_speed,
        min_speed=0.05,
        lookahead=0.6,
        goal_tolerance=0.2,
    )
    cmd_samples: list[dict[str, float]] = []

    def _record_cmd(twist: Twist) -> None:
        cmd_samples.append(
            {
                "linear_x": round(float(twist.linear.x), 5),
                "linear_y": round(float(twist.linear.y), 5),
                "angular_z": round(float(twist.angular.z), 5),
            }
        )

    follower.cmd_vel._add_callback(_record_cmd)
    follower.setup()
    follower.start()
    replay_samples = _resample_path_for_replay(path_points)
    errors: list[float] = []
    try:
        first = replay_samples[0]
        follower.odometry._deliver(
            Odometry(
                pose=_pose_with_yaw(first["point"], first["yaw"]),
                frame_id="map",
                child_frame_id="body",
            )
        )
        follower.local_path._deliver(_nav_path_from_points(path_points))
        base_ts = time.time()
        for idx, sample in enumerate(replay_samples):
            point = sample["point"]
            errors.append(_nearest_xy_error(point, path_points))
            follower.odometry._deliver(
                Odometry(
                    pose=_pose_with_yaw(point, sample["yaw"]),
                    ts=base_ts + idx * 0.1,
                    frame_id="map",
                    child_frame_id="body",
                )
            )
        final_yaw = replay_samples[-1]["yaw"]
        for settle_idx in range(8):
            follower.odometry._deliver(
                Odometry(
                    pose=_pose_with_yaw(path_points[-1], final_yaw),
                    ts=base_ts + (len(replay_samples) + settle_idx) * 0.1,
                    frame_id="map",
                    child_frame_id="body",
                )
            )
    finally:
        follower.stop()

    linear_values = [abs(float(cmd["linear_x"])) for cmd in cmd_samples]
    yaw_values = [abs(float(cmd["angular_z"])) for cmd in cmd_samples]
    final_linear = linear_values[-1] if linear_values else float("inf")
    max_linear = max(linear_values) if linear_values else float("inf")
    max_yaw = max(yaw_values) if yaw_values else float("inf")
    max_error = max(errors) if errors else float("inf")
    mean_error = float(np.mean(errors)) if errors else float("inf")
    speed_limit_ok = max_linear <= max_speed + 0.03
    yaw_limit_ok = max_yaw <= max_abs_yaw_rate
    stop_ok = final_linear <= 0.03
    error_ok = max_error <= max_tracking_error_m
    return {
        "ok": bool(cmd_samples) and speed_limit_ok and yaw_limit_ok and stop_ok and error_ok,
        "non_motion": True,
        "driver_used": False,
        "goal_sent": False,
        "cmd_vel_sent_to_hardware": False,
        "backend_requested": backend,
        "backend_actual": follower._backend,
        "native_backend_used": follower._backend == "nav_kernel",
        "odom_tick_count": len(replay_samples) + 8,
        "cmd_count": len(cmd_samples),
        "max_tracking_error_m": round(float(max_error), 4),
        "mean_tracking_error_m": round(float(mean_error), 4),
        "max_linear_x": round(float(max_linear), 5),
        "max_abs_angular_z": round(float(max_yaw), 5),
        "final_abs_linear_x": round(float(final_linear), 5),
        "speed_limit_ok": speed_limit_ok,
        "yaw_limit_ok": yaw_limit_ok,
        "stop_ok": stop_ok,
        "tracking_error_ok": error_ok,
        "replay_source": "scripted_odom_samples_on_planned_path",
    }


def _split_bridge_motion_segments(
    path_points: list[list[float]],
    *,
    min_xy_distance_m: float = 0.45,
    vertical_jump_m: float = 0.9,
) -> tuple[list[list[list[float]]], int]:
    if len(path_points) < 2:
        return [], 0
    segments: list[list[list[float]]] = []
    current: list[list[float]] = [path_points[0]]
    transition_edges = 0
    in_transition = False
    for prev, point in zip(path_points, path_points[1:]):
        xy_dist = float(np.linalg.norm(np.asarray(point[:2], dtype=float) - np.asarray(prev[:2], dtype=float)))
        z_diff = abs((float(point[2]) if len(point) > 2 else 0.0) - (float(prev[2]) if len(prev) > 2 else 0.0))
        is_transition_edge = z_diff > vertical_jump_m or (z_diff > 0.25 and xy_dist < 0.80)
        if is_transition_edge:
            transition_edges += 1
            if not in_transition and current:
                prev_z = float(prev[2]) if len(prev) > 2 else 0.0
                projected_endpoint = [
                    float(point[0]),
                    float(point[1]),
                    prev_z,
                ]
                if not np.allclose(np.asarray(current[-1][:2], dtype=float), np.asarray(projected_endpoint[:2], dtype=float)):
                    current.append(projected_endpoint)
                elif len(current[-1]) > 2:
                    current[-1][2] = prev_z
            if not in_transition and len(current) >= 2:
                segments.append(current)
            current = [point]
            in_transition = True
            continue
        if in_transition and len(current) == 1:
            point_z = float(point[2]) if len(point) > 2 else 0.0
            transition_exit = [
                float(current[0][0]),
                float(current[0][1]),
                point_z,
            ]
            current = [transition_exit]
            in_transition = False
        current.append(point)
    if len(current) >= 2:
        segments.append(current)

    filtered: list[list[list[float]]] = []
    for segment in segments:
        xy = np.asarray([p[:2] for p in segment], dtype=float)
        distance = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy) > 1 else 0.0
        if distance >= min_xy_distance_m:
            filtered.append(segment)
    return filtered, transition_edges


def _path_xy_length(path_points: list[list[float]]) -> float:
    if len(path_points) < 2:
        return 0.0
    xy = np.asarray([p[:2] for p in path_points], dtype=float)
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


def _bridge_segment_timeout(path_points: list[list[float]], *, max_speed: float, floor_s: float) -> float:
    distance = _path_xy_length(path_points)
    effective_speed = max(0.08, max_speed * 0.35)
    return max(floor_s, min(45.0, 6.0 + distance / effective_speed))


def _bridge_segment_motion_ok(report: dict[str, Any]) -> bool:
    if not report.get("ok"):
        return False
    if report.get("cmd_vel_sent_to_hardware") is not False:
        return False
    if int(report.get("mux_cmd_count", 0)) <= 0:
        return False
    if int(report.get("path_follower_cmd_count", 0)) <= 0:
        return False
    nonzero_linear = report.get("nonzero_linear_xy_cmd_count", report.get("nonzero_linear_cmd_count", 0))
    if int(nonzero_linear or 0) <= 0:
        return False
    max_linear = report.get("max_linear_xy", report.get("max_linear_x", 0.0))
    if float(max_linear or 0.0) <= 0.02:
        return False
    if float(report.get("moved_m", 0.0) or 0.0) <= 0.05:
        return False
    return True


def _sample_xyz_trace(points: list[list[float]], max_points: int = 240) -> list[list[float]]:
    if not points:
        return []
    limit = max(2, int(max_points))
    if len(points) <= limit:
        selected = points
    else:
        indices = np.linspace(0, len(points) - 1, limit, dtype=int)
        selected = [points[int(index)] for index in np.unique(indices)]
    return [
        [round(float(point[0]), 4), round(float(point[1]), 4), round(float(point[2]), 4)]
        for point in selected
    ]


def _run_mujoco_bridge_segment(
    segment: list[list[float]],
    *,
    scene_xml: Path,
    timeout_s: float,
    max_speed: float,
    local_planner_backend: str,
) -> dict[str, Any]:
    from nav.services.plan.local_planner.service import LocalPlanner
    from nav.local.path_follower import PathFollower
    from drivers.sim.mujoco.driver import MujocoDriverModule
    from nav.services.safety.velocity_mux import VelocityMux
    from nav.mission.tracking.waypoint_tracker import EV_PATH_COMPLETE, WaypointTracker

    start = segment[0]
    goal = segment[-1]
    start_z = float(start[2]) if len(start) > 2 and float(start[2]) > 0.3 else 0.55
    driver = MujocoDriverModule(
        world=str(scene_xml),
        render=False,
        enable_camera=False,
        drive_mode="kinematic",
        sim_rate=50.0,
        start_pos=(float(start[0]), float(start[1]), start_z),
        odom_frame_id="map",
        max_linear_vel=max_speed,
        max_angular_vel=1.0,
    )
    local_planner = LocalPlanner(backend=local_planner_backend, corridor_lookahead_m=1.8)
    follower = PathFollower(
        backend="nav_kernel",
        max_speed=max_speed,
        min_speed=0.05,
        lookahead=0.8,
        goal_tolerance=0.25,
    )
    mux = VelocityMux(source_timeout=0.8)

    odom_samples: list[list[float]] = []
    lidar_counts: list[int] = []
    local_path_count = 0
    forwarded_local_path_count = 0
    last_local_path_forward = 0.0
    last_forwarded_local_path_count = 0
    follower_cmd_count = 0
    mux_cmds: list[dict[str, float]] = []
    tracker = WaypointTracker(threshold=0.55, final_threshold=0.35, stuck_timeout=max(timeout_s, 2.0))
    tracker.reset([np.asarray(p, dtype=float) for p in segment[1:]], np.asarray(start, dtype=float))
    tracker_events: list[str] = []
    last_waypoint_idx = -1
    last_waypoint_publish = 0.0
    path_complete = False

    def _on_odom(odom: Odometry) -> None:
        nonlocal last_waypoint_idx, last_waypoint_publish, path_complete
        odom_samples.append([
            float(odom.pose.position.x),
            float(odom.pose.position.y),
            float(odom.pose.position.z),
        ])
        local_planner.odometry._deliver(odom)
        follower.odometry._deliver(odom)
        status = tracker.update(
            np.asarray(odom_samples[-1], dtype=float),
            odom.yaw,
        )
        if status.event:
            tracker_events.append(status.event)
        if status.event == EV_PATH_COMPLETE:
            path_complete = True
            follower.map_frame_jump_event._deliver({"reason": "sim_bridge_path_complete"})
            mux.path_follower_cmd_vel._deliver(Twist())
            return

        waypoint = tracker.current_waypoint
        now = time.time()
        should_publish_wp = (
            waypoint is not None
            and (
                status.wp_index != last_waypoint_idx
                or now - last_waypoint_publish >= 0.20
            )
        )
        if should_publish_wp:
            last_waypoint_idx = status.wp_index
            last_waypoint_publish = now
            local_planner.waypoint._deliver(_pose_stamped([float(v) for v in waypoint]))

    def _on_local_path(path: NavPath) -> None:
        nonlocal local_path_count, forwarded_local_path_count, last_local_path_forward
        nonlocal last_forwarded_local_path_count
        local_path_count += 1
        now = time.time()
        if forwarded_local_path_count > 0 and now - last_local_path_forward < 0.20:
            return
        last_local_path_forward = now
        forwarded_local_path_count += 1
        current_xy = np.asarray(odom_samples[-1][:2], dtype=float) if odom_samples else None
        follower_path = (
            _prepare_local_path_for_follower(path, current_xy=current_xy)
            if _is_production_local_planner_backend(local_planner_backend)
            else path
        )
        last_forwarded_local_path_count = len(follower_path.poses)
        follower.local_path._deliver(follower_path)

    def _on_follower_cmd(twist: Twist) -> None:
        nonlocal follower_cmd_count
        follower_cmd_count += 1
        mux.path_follower_cmd_vel._deliver(twist)

    def _on_mux_cmd(twist: Twist) -> None:
        mux_cmds.append(
            {
                "linear_x": round(float(twist.linear.x), 5),
                "linear_y": round(float(twist.linear.y), 5),
                "angular_z": round(float(twist.angular.z), 5),
            }
        )
        driver.cmd_vel._deliver(twist)

    driver.odometry._add_callback(_on_odom)
    driver.lidar_cloud._add_callback(lambda cloud: lidar_counts.append(int(len(cloud.points))))
    local_planner.local_path._add_callback(_on_local_path)
    follower.cmd_vel._add_callback(_on_follower_cmd)
    mux.driver_cmd_vel._add_callback(_on_mux_cmd)

    modules = [local_planner, follower, mux, driver]
    try:
        for module in modules:
            module.setup()
        if driver._engine is None:
            return {
                "ok": False,
                "error": "MujocoDriverModule setup failed",
                "scene_xml": str(scene_xml),
            }
        local_planner.global_path._deliver([np.asarray(p, dtype=float) for p in segment])
        for module in modules:
            module.start()

        first_odom_deadline = time.time() + min(3.0, timeout_s)
        while time.time() < first_odom_deadline and not odom_samples:
            time.sleep(0.02)
        if not odom_samples:
            return {"ok": False, "error": "no odometry from mujoco driver"}

        start_xy = np.asarray(odom_samples[-1][:2], dtype=float)
        goal_xy = np.asarray(goal[:2], dtype=float)
        start_dist = float(np.linalg.norm(goal_xy - start_xy))
        best_dist = start_dist
        end_dist = start_dist
        moved = 0.0
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            time.sleep(0.05)
            if not odom_samples:
                continue
            pos_xy = np.asarray(odom_samples[-1][:2], dtype=float)
            moved = max(moved, float(np.linalg.norm(pos_xy - start_xy)))
            end_dist = float(np.linalg.norm(goal_xy - pos_xy))
            best_dist = min(best_dist, end_dist)
            if (path_complete or end_dist < 0.35) and follower_cmd_count > 0 and mux_cmds:
                break

        # Stop command remains inside the simulated driver input.
        mux.path_follower_cmd_vel._deliver(Twist())
        time.sleep(0.1)
        max_vx = max((abs(cmd["linear_x"]) for cmd in mux_cmds), default=0.0)
        max_vy = max((abs(cmd["linear_y"]) for cmd in mux_cmds), default=0.0)
        max_linear_xy = max((math.hypot(cmd["linear_x"], cmd["linear_y"]) for cmd in mux_cmds), default=0.0)
        max_wz = max((abs(cmd["angular_z"]) for cmd in mux_cmds), default=0.0)
        nonzero_vx_count = sum(1 for cmd in mux_cmds if abs(cmd["linear_x"]) > 0.02)
        nonzero_vy_count = sum(1 for cmd in mux_cmds if abs(cmd["linear_y"]) > 0.02)
        nonzero_linear_xy_count = sum(1 for cmd in mux_cmds if math.hypot(cmd["linear_x"], cmd["linear_y"]) > 0.02)
        mean_abs_vx = float(np.mean([abs(cmd["linear_x"]) for cmd in mux_cmds])) if mux_cmds else 0.0
        mean_abs_vy = float(np.mean([abs(cmd["linear_y"]) for cmd in mux_cmds])) if mux_cmds else 0.0
        mean_abs_linear_xy = (
            float(np.mean([math.hypot(cmd["linear_x"], cmd["linear_y"]) for cmd in mux_cmds]))
            if mux_cmds
            else 0.0
        )
        distance_reduced = start_dist - best_dist
        final_odom = odom_samples[-1] if odom_samples else None
        ok = (
            len(odom_samples) >= 3
            and bool(lidar_counts)
            and local_path_count > 0
            and follower_cmd_count > 0
            and bool(mux_cmds)
            and nonzero_linear_xy_count > 0
            and moved >= min(0.20, max(0.05, start_dist * 0.15))
            and distance_reduced >= min(0.15, max(0.04, start_dist * 0.10))
            and (path_complete or end_dist < 0.35)
            and max_linear_xy <= max_speed + 0.05
            and max_wz <= 1.25
        )
        return {
            "ok": ok,
            "non_motion": False,
            "simulation_only": True,
            "driver": "MujocoDriverModule",
            "drive_mode": "kinematic",
            "local_planner_backend": local_planner._backend,
            "path_follower_backend": follower._backend,
            "native_backend_used": follower._backend == "nav_kernel",
            "waypoint_total": len(segment) - 1,
            "waypoint_final_index": tracker.wp_index,
            "tracker_events": tracker_events,
            "timeout_s": round(float(timeout_s), 3),
            "odom_count": len(odom_samples),
            "lidar_cloud_count": len(lidar_counts),
            "last_lidar_points": lidar_counts[-1] if lidar_counts else 0,
            "local_path_count": local_path_count,
            "forwarded_local_path_count": forwarded_local_path_count,
            "local_path_thinning_min_xy_step_m": (
                0.12 if _is_production_local_planner_backend(local_planner_backend) else 0.0
            ),
            "local_path_trim_start_dist_m": (
                0.20 if _is_production_local_planner_backend(local_planner_backend) else 0.0
            ),
            "last_forwarded_local_path_count": last_forwarded_local_path_count,
            "path_follower_cmd_count": follower_cmd_count,
            "mux_cmd_count": len(mux_cmds),
            "start": [float(v) for v in start],
            "goal": [float(v) for v in goal],
            "first_odom": odom_samples[0] if odom_samples else None,
            "final_odom": final_odom,
            "odom_trace": _sample_xyz_trace(odom_samples),
            "moved_m": round(float(moved), 4),
            "start_dist_m": round(float(start_dist), 4),
            "best_dist_m": round(float(best_dist), 4),
            "end_dist_m": round(float(end_dist), 4),
            "distance_reduced_m": round(float(distance_reduced), 4),
            "reached_goal": bool(path_complete or end_dist < 0.35),
            "max_linear_x": round(float(max_vx), 5),
            "max_linear_y": round(float(max_vy), 5),
            "max_linear_xy": round(float(max_linear_xy), 5),
            "mean_abs_linear_x": round(float(mean_abs_vx), 5),
            "mean_abs_linear_y": round(float(mean_abs_vy), 5),
            "mean_abs_linear_xy": round(float(mean_abs_linear_xy), 5),
            "nonzero_linear_cmd_count": nonzero_vx_count,
            "nonzero_linear_y_cmd_count": nonzero_vy_count,
            "nonzero_linear_xy_cmd_count": nonzero_linear_xy_count,
            "max_abs_angular_z": round(float(max_wz), 5),
            "first_mux_cmd": mux_cmds[0] if mux_cmds else None,
            "last_mux_cmd": mux_cmds[-1] if mux_cmds else None,
            "cmd_vel_sent_to_hardware": False,
        }
    except Exception as exc:
        return {
            "ok": False,
            "error": str(exc),
            "scene_xml": str(scene_xml),
            "local_planner_backend": local_planner_backend,
            "production_local_planner_verified": False,
        }
    finally:
        for module in reversed(modules):
            try:
                module.stop()
            except Exception:
                pass


def run_mujoco_bridge_loop(
    path_points: list[list[float]],
    *,
    scene_xml: Path,
    timeout_s: float = 8.0,
    max_speed: float = 0.35,
    local_planner_backend: str = "simple",
) -> dict[str, Any]:
    if len(path_points) < 2:
        return {"ok": False, "error": "need at least two path points"}
    segments, transition_edges = _split_bridge_motion_segments(path_points)
    if not segments:
        return {"ok": False, "error": "no horizontal motion segment available"}

    segment_reports = [
        _run_mujoco_bridge_segment(
            segment,
            scene_xml=scene_xml,
            timeout_s=_bridge_segment_timeout(segment, max_speed=max_speed, floor_s=timeout_s),
            max_speed=max_speed,
            local_planner_backend=local_planner_backend,
        )
        for segment in segments[:2]
    ]
    return {
        "ok": all(_bridge_segment_motion_ok(item) for item in segment_reports),
        "simulation_only": True,
        "driver": "MujocoDriverModule",
        "drive_mode": "kinematic",
        "local_planner_backend": local_planner_backend,
        "production_local_planner_verified": (
            _is_production_local_planner_backend(local_planner_backend)
            and all(item.get("ok") for item in segment_reports)
        ),
        "cmd_vel_sent_to_hardware": False,
        "segment_count": len(segment_reports),
        "transition_edges_skipped": transition_edges,
        "vertical_transition_note": (
            "Kinematic MuJoCo bridge validates per-floor motion segments; "
            "vertical stair transition is validated by transition_validation."
        ),
        "segments": segment_reports,
    }


def run_command_flow(
    path_points: list[list[float]],
    *,
    local_planner_backend: str = "simple",
) -> dict[str, Any]:
    from nav.services.plan.local_planner.service import LocalPlanner
    from nav.local.path_follower import PathFollower
    from nav.services.safety.velocity_mux import VelocityMux

    path_follower_backend = "pid"
    local_planner = LocalPlanner(backend=local_planner_backend, corridor_lookahead_m=2.5)
    path_follower = PathFollower(
        backend=path_follower_backend,
        max_speed=0.25,
        min_speed=0.05,
        lookahead=0.6,
    )
    mux = VelocityMux(source_timeout=1.0)
    modules = [local_planner, path_follower, mux]

    seen = {"local_path": 0, "path_follower_cmd": 0, "mux_cmd": 0}
    local_summaries: list[dict[str, Any]] = []
    cmd_samples: list[dict[str, float]] = []

    def _record_local_path(path: NavPath) -> None:
        seen["local_path"] += 1
        local_summaries.append(
            {
                "count": len(path.poses),
                "frame_id": path.frame_id,
                "first": path.poses[0].pose.position.to_list() if path.poses else None,
                "last": path.poses[-1].pose.position.to_list() if path.poses else None,
            }
        )
        path_follower.local_path._deliver(path)

    def _record_follower_cmd(twist: Twist) -> None:
        seen["path_follower_cmd"] += 1
        mux.path_follower_cmd_vel._deliver(twist)

    def _record_mux_cmd(twist: Twist) -> None:
        seen["mux_cmd"] += 1
        cmd_samples.append(
            {
                "linear_x": round(float(twist.linear.x), 5),
                "angular_z": round(float(twist.angular.z), 5),
            }
        )

    local_planner.local_path._add_callback(_record_local_path)
    path_follower.cmd_vel._add_callback(_record_follower_cmd)
    mux.driver_cmd_vel._add_callback(_record_mux_cmd)

    try:
        for module in modules:
            module.setup()
            module.start()
        if len(path_points) < 2:
            raise ValueError("need at least two global path points")
        odom = _odom_at_path_start(path_points)
        local_planner.odometry._deliver(odom)
        path_follower.odometry._deliver(odom)
        local_planner.global_path._deliver([np.asarray(p, dtype=float) for p in path_points])
        local_planner.waypoint._deliver(_pose_stamped(path_points[min(1, len(path_points) - 1)]))
        # Production local-planner backends publish on odom ticks, not on the
        # waypoint callback. Send a few deterministic ticks before judging the
        # command-flow gate so the probe reflects the runtime trigger pattern.
        for _ in range(5):
            local_planner.odometry._deliver(odom)
            path_follower.odometry._deliver(odom)
            if seen["local_path"] > 0 and seen["mux_cmd"] > 0:
                break
            time.sleep(0.02)
    except Exception as exc:
        local_planner_evidence = _algorithm_backend_evidence(
            local_planner,
            requested=local_planner_backend,
            exercised_by="command_flow",
            native_backend="nanobind",
        )
        path_follower_evidence = _algorithm_backend_evidence(
            path_follower,
            requested=path_follower_backend,
            exercised_by="command_flow",
            native_backend="nav_kernel",
        )
        return {
            "ok": False,
            "error": str(exc),
            "non_motion": True,
            "local_planner_backend": local_planner_backend,
            "local_planner_backend_requested": local_planner_backend,
            "local_planner_backend_actual": local_planner_evidence["backend_actual"],
            "path_follower_backend_requested": path_follower_backend,
            "path_follower_backend_actual": path_follower_evidence["backend_actual"],
            "algorithm_backends": {
                "local_planner": local_planner_evidence,
                "path_follower": path_follower_evidence,
            },
            "production_local_planner_verified": False,
            "driver_used": False,
            "goal_sent": False,
            "cmd_vel_sent_to_hardware": False,
        }
    finally:
        for module in reversed(modules):
            module.stop()

    local_planner_evidence = _algorithm_backend_evidence(
        local_planner,
        requested=local_planner_backend,
        exercised_by="command_flow",
        native_backend="nanobind",
    )
    path_follower_evidence = _algorithm_backend_evidence(
        path_follower,
        requested=path_follower_backend,
        exercised_by="command_flow",
        native_backend="nav_kernel",
    )
    local_planner_actual = str(local_planner_evidence["backend_actual"])
    path_follower_actual = str(path_follower_evidence["backend_actual"])
    ok = (
        seen["local_path"] > 0
        and seen["path_follower_cmd"] > 0
        and seen["mux_cmd"] > 0
        and bool(cmd_samples)
        and abs(cmd_samples[-1]["linear_x"]) > 0.0
    )
    return {
        "ok": ok,
        "non_motion": True,
        "local_planner_backend": local_planner_actual,
        "local_planner_backend_requested": local_planner_backend,
        "local_planner_backend_actual": local_planner_actual,
        "path_follower_backend_requested": path_follower_backend,
        "path_follower_backend_actual": path_follower_actual,
        "algorithm_backends": {
            "local_planner": local_planner_evidence,
            "path_follower": path_follower_evidence,
        },
        "production_local_planner_verified": ok and _is_production_local_planner_backend(local_planner_actual),
        "driver_used": False,
        "goal_sent": False,
        "cmd_vel_sent_to_hardware": False,
        "seen": seen,
        "last_local_path": local_summaries[-1] if local_summaries else None,
        "last_mux_cmd": cmd_samples[-1] if cmd_samples else None,
    }


def _path_initial_yaw(path_points: list[list[float]]) -> float:
    if len(path_points) < 2:
        return 0.0
    start = np.asarray(path_points[0][:2], dtype=float)
    for point in path_points[1:]:
        target = np.asarray(point[:2], dtype=float)
        delta = target - start
        if float(np.linalg.norm(delta)) > 1e-6:
            return float(math.atan2(delta[1], delta[0]))
    return 0.0


def _odom_at_path_start(path_points: list[list[float]]) -> Odometry:
    start = path_points[0]
    yaw = _path_initial_yaw(path_points)
    return Odometry(
        pose=Pose(
            position=Vector3(
                float(start[0]),
                float(start[1]),
                float(start[2]) if len(start) > 2 else 0.0,
            ),
            orientation=Quaternion.from_yaw(yaw),
        ),
        frame_id="map",
    )


def run_frontier_exploration_probe() -> dict[str, Any]:
    from nav.exploration.frontier_explorer_module import WavefrontFrontierExplorer

    explorer = WavefrontFrontierExplorer(
        min_frontier_size=2,
        safe_distance=0.0,
        lookahead_distance=6.0,
        explore_rate=10.0,
        costmap_max_age=60.0,
    )
    frontiers_seen: list[list[dict[str, Any]]] = []
    goals: list[list[float]] = []
    explorer.frontiers._add_callback(frontiers_seen.append)
    explorer.exploration_goal._add_callback(
        lambda goal: goals.append([float(goal.x), float(goal.y), float(goal.z)])
    )
    explorer.setup()
    explorer.start()
    try:
        costmap = _frontier_probe_costmap()
        odom = Odometry(pose=Pose(DEFAULT_START[0], DEFAULT_START[1], DEFAULT_START[2]), frame_id="map")
        explorer.odometry._deliver(odom)
        explorer.costmap._deliver(costmap)
        frontiers = explorer.get_frontiers()
        status = explorer.begin_exploration()
        deadline = time.time() + 1.5
        while time.time() < deadline and not goals:
            time.sleep(0.05)
        explorer.end_exploration()
    finally:
        explorer.stop()
    return {
        "ok": bool(frontiers) and bool(goals),
        "closed_loop": False,
        "probe_mode": "candidate_only",
        "status": status,
        "frontier_count": len(frontiers),
        "published_frontier_batches": len(frontiers_seen),
        "goal_count": len(goals),
        "first_goal": goals[0] if goals else None,
    }


def _frontier_probe_costmap() -> dict[str, Any]:
    grid = np.full((45, 55), -1, dtype=np.int16)
    # Seed the robot's current sensor-visible free space. The start pose maps
    # to cell (row=12, col=10), so this known-free patch must include it; the
    # frontier explorer intentionally no longer traverses unknown cells.
    grid[10:29, 8:26] = 0
    grid[20:23, 18:21] = 100
    return {
        "grid": grid,
        "resolution": 0.2,
        "origin": np.asarray([-2.4, -4.5], dtype=np.float32),
        "origin_x": -2.4,
        "origin_y": -4.5,
        "width": grid.shape[1],
        "height": grid.shape[0],
    }


def _known_cell_count(grid: np.ndarray) -> int:
    return int(np.sum(np.asarray(grid) != -1))


def _reveal_frontier_neighborhood(
    grid: np.ndarray,
    costmap: dict[str, Any],
    goal: list[float],
    *,
    round_index: int,
) -> dict[str, Any]:
    before = _known_cell_count(grid)
    resolution = float(costmap["resolution"])
    origin_x = float(costmap["origin_x"])
    origin_y = float(costmap["origin_y"])
    xi = int(round((float(goal[0]) - origin_x) / resolution))
    yi = int(round((float(goal[1]) - origin_y) / resolution))
    y0 = max(0, yi - 12 + round_index * 13)
    y1 = min(grid.shape[0], yi + 13 + round_index * 13)
    x0 = max(0, xi - 1)
    x1 = min(grid.shape[1], xi + 30)
    if x0 < x1 and y0 < y1:
        region = grid[y0:y1, x0:x1]
        region[region == -1] = 0
    after = _known_cell_count(grid)
    return {
        "known_cells_before": before,
        "known_cells_after": after,
        "known_cells_delta": after - before,
        "reveal_bounds": {"x0": x0, "x1": x1, "y0": y0, "y1": y1},
    }


def _plan_to_frontier_goal(
    *,
    planner: str,
    tomogram: Path,
    start: tuple[float, float, float],
    frontier_goal: list[float],
    downsample_dist: float,
) -> dict[str, Any]:
    raw_goal = (
        float(frontier_goal[0]),
        float(frontier_goal[1]),
        float(frontier_goal[2]) if len(frontier_goal) > 2 else 0.0,
    )
    offsets = [
        (0.0, 0.0),
        (0.0, -0.8),
        (0.0, -1.2),
        (-0.4, -0.8),
        (0.4, -0.8),
        (0.0, 0.8),
        (0.0, 1.2),
    ]
    last_plan: dict[str, Any] | None = None
    for dx, dy in offsets:
        goal = (raw_goal[0] + dx, raw_goal[1] + dy, raw_goal[2])
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            plan = run_global_planner(
                planner=planner,
                tomogram=tomogram,
                start=start,
                goal=goal,
                downsample_dist=downsample_dist,
                safe_goal_tolerance=1.2,
            )
        plan["frontier_goal"] = list(raw_goal)
        plan["navigation_goal"] = list(goal)
        plan["goal_projected_to_traversable"] = bool(dx or dy)
        last_plan = plan
        if plan.get("feasible") and plan.get("path"):
            return plan
    return last_plan or {
        "planner": planner,
        "ok": False,
        "feasible": False,
        "path": [],
        "frontier_goal": list(raw_goal),
        "navigation_goal": list(raw_goal),
        "goal_projected_to_traversable": False,
        "error": "frontier planning did not run",
    }


def run_frontier_navigation_closed_loop_probe(
    *,
    output_dir: Path | None = None,
    rounds: int = 2,
    planner: str = "astar",
    downsample_dist: float = 0.5,
    local_planner_backend: str = "simple",
) -> dict[str, Any]:
    from nav.exploration.frontier_explorer_module import WavefrontFrontierExplorer

    rounds = max(0, int(rounds))
    assets_dir = output_dir or (ROOT / "artifacts" / "multifloor_frontier_loop")
    assets = build_multifloor_assets(assets_dir)
    explorer = WavefrontFrontierExplorer(
        min_frontier_size=2,
        safe_distance=0.0,
        lookahead_distance=6.0,
        explore_rate=10.0,
        costmap_max_age=60.0,
    )
    costmap = _frontier_probe_costmap()
    grid = costmap["grid"]
    initial_known = _known_cell_count(grid)
    pose = (
        float(DEFAULT_START[0]),
        float(DEFAULT_START[1]),
        float(DEFAULT_START[2]),
    )
    round_reports: list[dict[str, Any]] = []
    goal_reached_events = 0

    explorer.setup()
    explorer.start()
    try:
        for round_index in range(rounds):
            explorer.odometry._deliver(
                Odometry(pose=Pose(pose[0], pose[1], pose[2]), frame_id="map")
            )
            explorer.costmap._deliver(costmap)
            frontiers = explorer.get_frontiers()
            if not frontiers:
                round_reports.append(
                    {
                        "round": round_index + 1,
                        "ok": False,
                        "error": "no frontier candidates available",
                    }
                )
                break

            frontier = frontiers[0]
            frontier_goal = [float(frontier["cx"]), float(frontier["cy"]), 0.0]
            plan = _plan_to_frontier_goal(
                planner=planner,
                tomogram=assets.tomogram,
                start=pose,
                frontier_goal=frontier_goal,
                downsample_dist=downsample_dist,
            )
            path = plan.get("path") or []
            path_validation = (
                validate_path_against_tomogram(assets.tomogram, path)
                if plan.get("feasible") and path
                else {"ok": False, "reason": "global_plan_failed", "error": plan.get("error")}
            )
            command_flow = (
                run_command_flow(path, local_planner_backend=local_planner_backend)
                if path_validation.get("ok")
                else {"ok": False, "error": "path validation failed"}
            )
            tracking_replay = (
                run_tracking_replay(path)
                if path_validation.get("ok")
                else {"ok": False, "error": "path validation failed"}
            )
            goal_reached = bool(
                plan.get("feasible")
                and path_validation.get("ok")
                and command_flow.get("ok")
                and tracking_replay.get("ok")
            )
            reveal = _reveal_frontier_neighborhood(
                grid,
                costmap,
                plan.get("navigation_goal") or frontier_goal,
                round_index=round_index,
            )
            if goal_reached:
                goal_reached_events += 1
                reached = plan.get("last") or path[-1]
                pose = (
                    float(reached[0]),
                    float(reached[1]),
                    float(reached[2]) if len(reached) > 2 else 0.0,
                )
                explorer.goal_reached._deliver(True)

            round_reports.append(
                {
                    "round": round_index + 1,
                    "ok": goal_reached and reveal["known_cells_delta"] > 0,
                    "frontier_count": len(frontiers),
                    "goal": {
                        "frontier": frontier_goal,
                        "navigation": plan.get("navigation_goal"),
                        "safe": plan.get("safe_goal"),
                        "safe_goal_tolerance": plan.get("safe_goal_tolerance"),
                        "projected_to_traversable": plan.get("goal_projected_to_traversable", False),
                    },
                    "path": path,
                    "path_validation": path_validation,
                    "command_flow": command_flow,
                    "tracking_replay": tracking_replay,
                    "goal_reached": goal_reached,
                    "map_update": reveal,
                }
            )
    finally:
        explorer.stop()

    final_known = _known_cell_count(grid)
    rounds_completed = sum(1 for item in round_reports if item.get("ok"))
    ok = bool(rounds > 0 and rounds_completed == rounds and goal_reached_events == rounds and final_known > initial_known)
    return {
        "ok": ok,
        "closed_loop": True,
        "probe_mode": "frontier_navigation_closed_loop",
        "simulation_only": True,
        "real_robot_motion": False,
        "driver_used": False,
        "cmd_vel_sent_to_hardware": False,
        "planner": planner,
        "requested_rounds": rounds,
        "rounds_completed": rounds_completed,
        "known_cells_initial": initial_known,
        "known_cells_final": final_known,
        "known_cells_delta": final_known - initial_known,
        "goal_reached_events": goal_reached_events,
        "rounds": round_reports,
    }


def run_mujoco_scene_load(scene_xml: Path) -> dict[str, Any]:
    try:
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(scene_xml))
        data = mujoco.MjData(model)
        for _ in range(10):
            mujoco.mj_step(model, data)
        return {
            "ok": True,
            "model": model.names.decode("utf-8", errors="ignore").split("\x00")[0],
            "ngeom": int(model.ngeom),
            "nbody": int(model.nbody),
        }
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _select_route(route: str) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    case = ROUTE_CASES[route]
    return case.start, case.goal


def _required_planner_ok(
    *,
    planners: list[str],
    planning: list[dict[str, Any]],
    case: RouteCase,
) -> bool:
    def validation_ok(plan: dict[str, Any], key: str) -> bool:
        value = plan.get(key)
        return isinstance(value, dict) and value.get("ok") is True

    def native_pct_ok(plan: dict[str, Any]) -> bool:
        return (
            plan.get("planner") == "pct"
            and plan.get("feasible") is True
            and plan.get("backend_available") is True
            and plan.get("native_backend_used") is True
            and validation_ok(plan, "path_validation")
        )

    if case.require_pct:
        return any(
            native_pct_ok(p)
            and validation_ok(p, "transition_validation")
            and (case.min_required_z_max is None or (p.get("z_max") or 0.0) >= case.min_required_z_max)
            for p in planning
        )
    if "pct" not in planners:
        return any(p.get("feasible") is True and validation_ok(p, "path_validation") for p in planning)
    return any(native_pct_ok(p) for p in planning)


def _native_pct_gate(
    *,
    planners: list[str],
    planning: list[dict[str, Any]],
    case: RouteCase,
) -> dict[str, Any]:
    required = case.require_pct or "pct" in planners
    pct_plans = [plan for plan in planning if plan.get("planner") == "pct"]
    selected = next((plan for plan in pct_plans if plan.get("feasible")), pct_plans[0] if pct_plans else None)
    if not required:
        return {
            "required": False,
            "ok": True,
            "status": "not_required",
            "blocked": False,
            "reason": "pct planner was not requested for this route",
        }
    if selected is None:
        return {
            "required": True,
            "ok": False,
            "status": "blocked",
            "blocked": True,
            "reason": "pct planner was not included in --planners",
        }

    ok = _required_planner_ok(planners=planners, planning=planning, case=case)
    required_segments = int(selected.get("native_pct_required_segments") or 1)
    feasible_segments = int(selected.get("native_pct_feasible_segments") or (1 if selected.get("native_backend_used") else 0))
    runtime = selected.get("native_runtime") or {}
    status = "pass" if ok else str(selected.get("status") or "fail")
    blocked = status == "blocked" or bool(selected.get("blocked"))
    return {
        "required": True,
        "ok": bool(ok),
        "status": status,
        "blocked": bool(blocked and not ok),
        "reason": "" if ok else selected.get("error") or "native pct gate failed",
        "planner": "pct",
        "planner_requested": str(selected.get("planner_requested") or selected.get("planner") or "pct"),
        "selected_planner": str(selected.get("selected_planner") or selected.get("planner") or "pct"),
        "fallback_reason": str(selected.get("fallback_reason") or ""),
        "plan_safety_policy": str(selected.get("plan_safety_policy") or ""),
        "rejected_plans": selected.get("rejected_plans") if isinstance(selected.get("rejected_plans"), list) else [],
        "backend_available": bool(selected.get("backend_available")),
        "backend_requested_available": bool(
            selected.get("backend_requested_available", selected.get("backend_available"))
        ),
        "native_backend_used": bool(selected.get("native_backend_used")),
        "planner_class": selected.get("planner_class"),
        "backend_requested_class": selected.get("backend_requested_class"),
        "runtime_ok": bool(runtime.get("ok")) if isinstance(runtime, dict) else False,
        "runtime": runtime,
        "tomogram": selected.get("tomogram"),
        "tomogram_sha256": selected.get("tomogram_sha256"),
        "route_strategy": selected.get("route_strategy") or selected.get("planner_scope"),
        "native_pct_single_plan_verified": bool(selected.get("native_pct_single_plan_verified", selected.get("planner_scope") == "native_pct_single_route")),
        "floor_graph_composition_verified": selected.get("route_strategy") == "floor_graph_pct_segments",
        "cross_floor_physical_verified": bool(selected.get("cross_floor_physical_verified", False)),
        "transition_motion_verified": bool(selected.get("transition_motion_verified", False)),
        "native_pct_required_segments": required_segments,
        "native_pct_feasible_segments": feasible_segments,
        "path_validation_ok": isinstance(selected.get("path_validation"), dict) and selected["path_validation"].get("ok") is True,
        "transition_validation_ok": (
            not selected.get("transition")
            or (isinstance(selected.get("transition_validation"), dict) and selected["transition_validation"].get("ok") is True)
        ),
        "z_min": selected.get("z_min"),
        "z_max": selected.get("z_max"),
        "path_count": selected.get("count", 0),
    }


def run_route_case(
    *,
    case: RouteCase,
    output_dir: Path,
    planners: list[str],
    downsample_dist: float,
    skip_mujoco: bool,
    bridge_loop: bool = False,
    bridge_loop_timeout_s: float = 8.0,
    bridge_loop_max_speed: float = 0.35,
    local_planner_backend: str = "simple",
    require_production_local_planner: bool = False,
    frontier_loop: bool = False,
    frontier_rounds: int = 2,
    shared_health: dict[str, Any] | None = None,
    shared_exploration: dict[str, Any] | None = None,
) -> dict[str, Any]:
    assets = build_multifloor_assets(output_dir, start=case.start, goal=case.goal)

    planning = [
        run_route_planner(
            planner=planner,
            route=case.route_mode,
            tomogram=assets.tomogram,
            start=case.start,
            goal=case.goal,
            downsample_dist=downsample_dist,
        )
        for planner in planners
    ]
    for plan in planning:
        if plan.get("feasible") and plan.get("path"):
            plan["path_validation"] = validate_path_against_tomogram(assets.tomogram, plan["path"])
        if plan.get("transition"):
            plan["transition_validation"] = validate_transition_against_metadata(
                assets.tomogram,
                plan["transition"],
            )
    native_pct_gate = _native_pct_gate(planners=planners, planning=planning, case=case)
    best_path = next((p["path"] for p in planning if p.get("feasible") and p.get("path")), [])
    command_flow = run_command_flow(best_path, local_planner_backend=local_planner_backend) if best_path else {
        "ok": False,
        "error": "no feasible path available for command-flow probe",
    }
    tracking_replay = run_tracking_replay(best_path) if best_path else {
        "ok": False,
        "error": "no feasible path available for tracking replay",
    }
    mujoco_bridge_loop = None
    if bridge_loop:
        mujoco_bridge_loop = run_mujoco_bridge_loop(
            best_path,
            scene_xml=assets.scene_xml,
            timeout_s=bridge_loop_timeout_s,
            max_speed=bridge_loop_max_speed,
            local_planner_backend=local_planner_backend,
        ) if best_path else {
            "ok": False,
            "error": "no feasible path available for MuJoCo bridge loop",
        }
    local_planner_backend_actual = str(
        command_flow.get(
            "local_planner_backend_actual",
            command_flow.get("local_planner_backend", local_planner_backend),
        )
    )
    production_local_planner_verified = bool(
        _is_production_local_planner_backend(local_planner_backend_actual)
        and command_flow.get("ok")
        and (not bridge_loop or (mujoco_bridge_loop or {}).get("production_local_planner_verified"))
    )
    exploration = shared_exploration
    if exploration is None:
        exploration = (
            run_frontier_navigation_closed_loop_probe(
                output_dir=output_dir / "frontier_loop",
                rounds=frontier_rounds,
                planner="astar",
                downsample_dist=downsample_dist,
                local_planner_backend=local_planner_backend,
            )
            if frontier_loop
            else run_frontier_exploration_probe()
        )

    report = {
        "schema_version": 1,
        **_validation_scope(),
        "local_planner_backend_requested": local_planner_backend,
        "local_planner_backend_verified": local_planner_backend_actual,
        "production_local_planner_required": require_production_local_planner,
        "production_local_planner_verified": production_local_planner_verified,
        "non_motion": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "simulation_motion": bool(bridge_loop),
        "simulated_driver_used": bool(bridge_loop),
        "driver_used": False,
        "gateway_used": False,
        "goal_sent": False,
        "cmd_vel_sent_to_hardware": False,
        "validation_limitations": [
            "kinematic MuJoCo mode does not verify gait policy, contacts, slip, actuator limits, or physical stairs",
            "synthetic_lidar_map_alignment does not verify Fast-LIO2, localizer, Super-LIO, IMU timing, or ROS2 topic flow",
            "cross_floor route is floor-graph composition plus metadata transition, not native single-shot 3D PCT",
            "simple local planner backend does not verify CMU/nanobind terrain scoring",
        ],
        "route": case.name,
        "route_mode": case.route_mode,
        "description": case.description,
        "assets": {
            "scene_xml": str(assets.scene_xml),
            "tomogram": str(assets.tomogram),
            "map_pcd": str(assets.map_pcd),
            "metadata": str(assets.metadata),
            "floors": list(assets.floors),
            "start": list(case.start),
            "goal": list(case.goal),
        },
        "mujoco_scene_load": None if skip_mujoco else run_mujoco_scene_load(assets.scene_xml),
        "lidar_localization": shared_health if shared_health is not None else run_lidar_localization_contract(),
        "planning": planning,
        "native_pct_gate": native_pct_gate,
        "command_flow": command_flow,
        "algorithm_backends": command_flow.get("algorithm_backends", {}),
        "tracking_replay": tracking_replay,
        "mujoco_bridge_loop": mujoco_bridge_loop,
        "exploration": exploration,
    }
    report["passed"] = bool(
        report["lidar_localization"].get("ok")
        and _required_planner_ok(planners=planners, planning=planning, case=case)
        and report["native_pct_gate"].get("ok")
        and report["command_flow"].get("ok")
        and report["tracking_replay"].get("ok")
        and (not bridge_loop or (report["mujoco_bridge_loop"] or {}).get("ok"))
        and (not require_production_local_planner or report["production_local_planner_verified"])
        and report["exploration"].get("ok")
        and (not frontier_loop or report["exploration"].get("closed_loop") is True)
        and (skip_mujoco or (report["mujoco_scene_load"] or {}).get("ok"))
    )
    return report


def run_route_matrix(
    *,
    output_dir: Path,
    planners: list[str],
    downsample_dist: float,
    skip_mujoco: bool,
    bridge_loop: bool = False,
    bridge_loop_timeout_s: float = 8.0,
    bridge_loop_max_speed: float = 0.35,
    local_planner_backend: str = "simple",
    require_production_local_planner: bool = False,
    frontier_loop: bool = False,
    frontier_rounds: int = 2,
    routes: tuple[str, ...] = MATRIX_ROUTES,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    shared_health = run_lidar_localization_contract()
    shared_exploration = (
        run_frontier_navigation_closed_loop_probe(
            output_dir=output_dir / "frontier_loop",
            rounds=frontier_rounds,
            planner="astar",
            downsample_dist=downsample_dist,
            local_planner_backend=local_planner_backend,
        )
        if frontier_loop
        else run_frontier_exploration_probe()
    )
    cases = [
        run_route_case(
            case=ROUTE_CASES[name],
            output_dir=output_dir / name,
            planners=planners,
            downsample_dist=downsample_dist,
            skip_mujoco=skip_mujoco,
            bridge_loop=bridge_loop,
            bridge_loop_timeout_s=bridge_loop_timeout_s,
            bridge_loop_max_speed=bridge_loop_max_speed,
            local_planner_backend=local_planner_backend,
            require_production_local_planner=require_production_local_planner,
            frontier_loop=frontier_loop,
            frontier_rounds=frontier_rounds,
            shared_health=shared_health,
            shared_exploration=shared_exploration,
        )
        for name in routes
    ]
    local_planner_backend_actuals = sorted(
        {
            str(case.get("local_planner_backend_verified", local_planner_backend))
            for case in cases
        }
    )
    local_planner_backend_verified: str | list[str]
    if len(local_planner_backend_actuals) == 1:
        local_planner_backend_verified = local_planner_backend_actuals[0]
    else:
        local_planner_backend_verified = local_planner_backend_actuals
    summary = {
        "schema_version": 1,
        **_validation_scope(),
        "local_planner_backend_requested": local_planner_backend,
        "local_planner_backend_verified": local_planner_backend_verified,
        "local_planner_backend_actuals": local_planner_backend_actuals,
        "production_local_planner_required": require_production_local_planner,
        "production_local_planner_verified": all(
            case.get("production_local_planner_verified") for case in cases
        ),
        "non_motion": True,
        "driver_used": False,
        "simulated_driver_used": bool(bridge_loop),
        "gateway_used": False,
        "goal_sent": False,
        "cmd_vel_sent_to_hardware": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "simulation_motion": bool(bridge_loop),
        "validation_limitations": [
            "matrix suite is a kinematic/dataflow gate and is not a real robot or gait-policy gate",
            "synthetic lidar localization is not a ROS2 SLAM/localizer replay",
            "cross-floor PCT is validated as floor-graph composition unless native single-plan evidence is explicitly present",
        ],
        "suite": "multifloor_route_matrix",
        "planners": planners,
        "routes": list(routes),
        "bridge_loop_enabled": bridge_loop,
        "frontier_loop_enabled": frontier_loop,
        "frontier_rounds": frontier_rounds,
        "exploration": shared_exploration,
        "passed": all(case["passed"] for case in cases),
        "case_count": len(cases),
        "passed_count": sum(1 for case in cases if case["passed"]),
        "blocked_count": sum(1 for case in cases if case.get("native_pct_gate", {}).get("blocked")),
        "native_pct_gate_passed_count": sum(1 for case in cases if case.get("native_pct_gate", {}).get("ok")),
        "native_pct_blocked_count": sum(1 for case in cases if case.get("native_pct_gate", {}).get("blocked")),
        "cases": cases,
    }
    summary["failures"] = [
        {
            "route": case["route"],
            "planning_errors": [
                {
                    "planner": plan.get("planner"),
                    "status": plan.get("status"),
                    "blocked": plan.get("blocked"),
                    "feasible": plan.get("feasible"),
                    "error": plan.get("error"),
                }
                for plan in case.get("planning", [])
                if not plan.get("feasible")
            ],
            "command_flow_ok": case.get("command_flow", {}).get("ok"),
            "tracking_replay_ok": case.get("tracking_replay", {}).get("ok"),
            "native_pct_gate_ok": case.get("native_pct_gate", {}).get("ok"),
            "native_pct_gate_status": case.get("native_pct_gate", {}).get("status"),
            "native_pct_gate_blocked": case.get("native_pct_gate", {}).get("blocked"),
            "native_pct_gate_reason": case.get("native_pct_gate", {}).get("reason"),
            "production_local_planner_verified": case.get("production_local_planner_verified"),
            "mujoco_bridge_loop_ok": (
                None
                if case.get("mujoco_bridge_loop") is None
                else case.get("mujoco_bridge_loop", {}).get("ok")
            ),
        }
        for case in cases
        if not case["passed"]
    ]
    return summary


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="artifacts/multifloor_nav_validation")
    parser.add_argument("--route", choices=[*ROUTE_CASES.keys(), "matrix"], default="cross_floor")
    parser.add_argument(
        "--matrix-routes",
        default=",".join(MATRIX_ROUTES),
        help="Comma-separated route names used when --route matrix is selected.",
    )
    parser.add_argument("--planners", default="pct,astar", help="Comma-separated planner list")
    parser.add_argument("--downsample-dist", type=float, default=0.5)
    parser.add_argument("--skip-mujoco", action="store_true")
    parser.add_argument(
        "--bridge-loop",
        action="store_true",
        help="Run simulated MuJoCo driver + PathFollower + VelocityMux closed-loop validation.",
    )
    parser.add_argument(
        "--frontier-loop",
        action="store_true",
        help="Run simulated frontier goal -> global plan -> tracking replay -> map reveal closed-loop validation.",
    )
    parser.add_argument("--frontier-rounds", type=int, default=2)
    parser.add_argument("--bridge-loop-timeout", type=float, default=22.0)
    parser.add_argument("--bridge-loop-max-speed", type=float, default=0.35)
    parser.add_argument(
        "--local-planner-backend",
        choices=["simple", "nanobind", "cmu", "cmu_py"],
        default="simple",
        help="LocalPlanner backend used by command-flow and bridge-loop probes.",
    )
    parser.add_argument(
        "--require-production-local-planner",
        action="store_true",
        help="Fail strict reports unless local planner probes use a production backend.",
    )
    parser.add_argument("--json-out", default="")
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    out = Path(args.output_dir).resolve()
    planners = [item.strip().lower() for item in args.planners.split(",") if item.strip()]

    if args.route == "matrix":
        matrix_routes = tuple(item.strip() for item in args.matrix_routes.split(",") if item.strip())
        unknown_routes = sorted(set(matrix_routes) - set(ROUTE_CASES))
        if unknown_routes:
            raise SystemExit(f"unknown --matrix-routes entries: {', '.join(unknown_routes)}")
        report = run_route_matrix(
            output_dir=out,
            planners=planners,
            downsample_dist=args.downsample_dist,
            skip_mujoco=args.skip_mujoco,
            bridge_loop=args.bridge_loop,
            bridge_loop_timeout_s=args.bridge_loop_timeout,
            bridge_loop_max_speed=args.bridge_loop_max_speed,
            local_planner_backend=args.local_planner_backend,
            require_production_local_planner=args.require_production_local_planner,
            frontier_loop=args.frontier_loop,
            frontier_rounds=args.frontier_rounds,
            routes=matrix_routes,
        )
    else:
        report = run_route_case(
            case=ROUTE_CASES[args.route],
            output_dir=out,
            planners=planners,
            downsample_dist=args.downsample_dist,
            skip_mujoco=args.skip_mujoco,
            bridge_loop=args.bridge_loop,
            bridge_loop_timeout_s=args.bridge_loop_timeout,
            bridge_loop_max_speed=args.bridge_loop_max_speed,
            local_planner_backend=args.local_planner_backend,
            require_production_local_planner=args.require_production_local_planner,
            frontier_loop=args.frontier_loop,
            frontier_rounds=args.frontier_rounds,
        )

    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        Path(args.json_out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json_out).write_text(text + "\n", encoding="utf-8")
    return 0 if report["passed"] or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
