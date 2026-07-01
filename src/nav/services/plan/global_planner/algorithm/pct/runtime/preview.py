"""PCT runtime preview CLI.

Example:
    python -m nav.services.plan.global_planner.algorithm.pct.runtime.preview \
        --tomogram artifacts/scene/tomogram.pickle \
        --start 2 3 0 --goal 18 11 0 --json
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
import math
import pickle
import platform
import sys
from pathlib import Path
from typing import Any

import numpy as np

from .api import (
    PCT_NATIVE_BINARY_FORMAT,
    PCT_RUST_FFI_BINARY_FORMAT,
    PCT_RUST_PROCESS_BINARY_FORMAT,
    load_tomogram_planner,
    resolve_pct_planner_runtime,
)


ACTUAL_SCHEMA = "lingtu.pct.preview.actual.v2"
DEFAULT_GOAL_TOLERANCE_M = 0.3
DEFAULT_MAX_SLOPE_DZ_PER_M = 1.0
DEFAULT_MAX_STEP_DZ_M = 0.35
DEFAULT_VERTICAL_SEGMENT_MIN_DZ_M = 0.25
DEFAULT_VERTICAL_SEGMENT_MAX_DXY_M = 0.05


def _distance(path: np.ndarray) -> float:
    if path.ndim != 2 or len(path) < 2:
        return 0.0
    cols = min(path.shape[1], 3)
    deltas = np.diff(path[:, :cols], axis=0)
    return float(np.sum(np.linalg.norm(deltas, axis=1)))


def _point_error(a: np.ndarray, b: np.ndarray) -> float:
    cols = min(a.shape[0], b.shape[0], 3)
    return float(math.dist(a[:cols], b[:cols]))


def _as_point(values: list[float]) -> np.ndarray:
    if len(values) != 3:
        raise ValueError("point must contain exactly x y z")
    return np.asarray(values, dtype=np.float64)


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def _tomogram_pickle_summary(path: Path) -> dict[str, Any]:
    try:
        with path.open("rb") as handle:
            payload = pickle.load(handle)
    except Exception as exc:  # noqa: BLE001 - diagnostic metadata must not break the smoke report.
        return {"content_error": f"{type(exc).__name__}: {exc}"}

    summary: dict[str, Any] = {}
    if isinstance(payload, dict):
        summary["keys"] = sorted(str(key) for key in payload.keys())
        for field in ("schema", "schema_version", "version"):
            if field in payload:
                summary[field] = _jsonable(payload[field])
                break
        data = payload.get("data")
    else:
        summary["python_type"] = type(payload).__name__
        data = payload

    shape = getattr(data, "shape", None)
    dtype = getattr(data, "dtype", None)
    if shape is not None:
        summary["data_shape"] = [int(item) for item in shape]
    if dtype is not None:
        summary["data_dtype"] = str(dtype)
    return summary


def _file_fingerprint(path_value: Any) -> dict[str, Any]:
    if path_value is None:
        return {}

    path = Path(path_value)
    report: dict[str, Any] = {"path": str(path)}
    try:
        resolved = path.resolve(strict=True)
        stat = resolved.stat()
    except OSError as exc:
        report.update(
            {
                "exists": False,
                "error": f"{type(exc).__name__}: {exc}",
            }
        )
        return report

    digest = hashlib.sha256()
    try:
        with resolved.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        report.update(
            {
                "exists": False,
                "resolved_path": str(resolved),
                "error": f"{type(exc).__name__}: {exc}",
            }
        )
        return report

    report.update(
        {
            "exists": True,
            "resolved_path": str(resolved),
            "size_bytes": int(stat.st_size),
            "sha256": digest.hexdigest(),
        }
    )
    report.update(_tomogram_pickle_summary(resolved))
    return report


def _sample_path(path: np.ndarray, max_points: int) -> list[dict[str, Any]]:
    if path.ndim != 2 or len(path) == 0 or max_points <= 0:
        return []
    count = min(int(max_points), len(path))
    if count == len(path):
        indices = list(range(len(path)))
    else:
        indices = sorted({int(round(idx)) for idx in np.linspace(0, len(path) - 1, count)})
    return [
        {
            "index": int(index),
            "point": path[index, : min(3, path.shape[1])].astype(float).tolist(),
        }
        for index in indices
    ]


def _sample_path_by_arclength(
    path: np.ndarray,
    fractions: tuple[float, ...] = (0.0, 0.25, 0.5, 0.75, 1.0),
) -> list[dict[str, Any]]:
    if path.ndim != 2 or len(path) == 0:
        return []
    if len(path) == 1:
        point = path[0, : min(3, path.shape[1])].astype(float).tolist()
        return [{"fraction": float(fraction), "point": point} for fraction in fractions]

    deltas = np.diff(path[:, :2], axis=0)
    segment_lengths = np.linalg.norm(deltas, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total = float(cumulative[-1])
    if total <= 0.0:
        point = path[0, : min(3, path.shape[1])].astype(float).tolist()
        return [{"fraction": float(fraction), "point": point} for fraction in fractions]

    samples: list[dict[str, Any]] = []
    for fraction in fractions:
        clamped = min(1.0, max(0.0, float(fraction)))
        target = total * clamped
        upper = int(np.searchsorted(cumulative, target, side="right"))
        upper = min(max(upper, 1), len(path) - 1)
        lower = upper - 1
        span = float(cumulative[upper] - cumulative[lower])
        alpha = 0.0 if span <= 0.0 else (target - cumulative[lower]) / span
        point = path[lower, :3] + (path[upper, :3] - path[lower, :3]) * alpha
        samples.append({"fraction": clamped, "point": point.astype(float).tolist()})
    return samples


def _bounds_xyz(path: np.ndarray) -> dict[str, list[float]]:
    if path.ndim != 2 or len(path) == 0:
        return {"min": [], "max": []}
    xyz = path[:, : min(3, path.shape[1])]
    return {
        "min": np.min(xyz, axis=0).astype(float).tolist(),
        "max": np.max(xyz, axis=0).astype(float).tolist(),
    }


def _tomogram_meta(planner: Any) -> dict[str, Any]:
    fields = ("resolution", "center", "map_dim", "n_slice", "slice_h0", "slice_dh")
    meta: dict[str, Any] = {}
    for field in fields:
        if hasattr(planner, field):
            meta[field] = _jsonable(getattr(planner, field))
    return meta


def _map_bounds_report(planner: Any) -> dict[str, Any]:
    try:
        resolution = float(getattr(planner, "resolution"))
        center = np.asarray(getattr(planner, "center"), dtype=np.float64).reshape(2)
        map_dim = np.asarray(getattr(planner, "map_dim"), dtype=np.int64).reshape(2)
        offset = np.asarray(getattr(planner, "offset"), dtype=np.float64).reshape(2)
    except Exception:
        return {"available": False}
    if resolution <= 0 or np.any(map_dim <= 0) or not np.all(np.isfinite(center)):
        return {"available": False}
    min_xy = (np.asarray([0.0, 0.0]) - offset) * resolution + center
    max_xy = (map_dim.astype(np.float64) - 1.0 - offset) * resolution + center
    return {
        "available": True,
        "resolution": resolution,
        "center": center.astype(float).tolist(),
        "map_dim": map_dim.astype(int).tolist(),
        "offset": offset.astype(float).tolist(),
        "min_xy": min_xy.astype(float).tolist(),
        "max_xy": max_xy.astype(float).tolist(),
    }


def _point_bounds_report(planner: Any, point: np.ndarray) -> dict[str, Any]:
    bounds = _map_bounds_report(planner)
    if not bounds.get("available"):
        return {"available": False, "in_bounds": None}
    resolution = float(bounds["resolution"])
    center = np.asarray(bounds["center"], dtype=np.float64)
    map_dim = np.asarray(bounds["map_dim"], dtype=np.int64)
    offset = np.asarray(bounds["offset"], dtype=np.float64)
    raw_idx = np.round((point[:2] - center) / resolution).astype(np.int64) + offset.astype(np.int64)
    clipped_idx = np.asarray(
        [
            np.clip(raw_idx[0], 0, map_dim[0] - 1),
            np.clip(raw_idx[1], 0, map_dim[1] - 1),
        ],
        dtype=np.int64,
    )
    projected = (clipped_idx.astype(np.float64) - offset) * resolution + center
    in_bounds = bool(np.array_equal(raw_idx, clipped_idx))
    return {
        "available": True,
        "in_bounds": in_bounds,
        "raw_idx": raw_idx.astype(int).tolist(),
        "clipped_idx": clipped_idx.astype(int).tolist(),
        "projected_world_xy": projected.astype(float).tolist(),
        "clamp_error_m": float(math.dist(projected, point[:2])),
    }


def _returned_path_blocked_sample_count(planner: Any, path: np.ndarray) -> int | None:
    checker = getattr(planner, "_hard_obstacle_sample_count", None)
    if not callable(checker):
        return None
    try:
        return int(checker(path))
    except Exception:
        return None


def _kinematic_report(
    path: np.ndarray,
    *,
    max_slope_dz_per_m: float,
    max_step_dz_m: float,
    vertical_segment_min_dz_m: float,
    vertical_segment_max_dxy_m: float,
) -> dict[str, Any]:
    if path.ndim != 2 or path.shape[0] < 2 or path.shape[1] < 3:
        return {"available": False, "ok": True, "reason": "path has fewer than two 3D points"}
    deltas = np.diff(path[:, :3], axis=0)
    dxy = np.linalg.norm(deltas[:, :2], axis=1)
    dz = np.abs(deltas[:, 2])
    slopes = dz / np.maximum(dxy, 1e-9)
    slope_violations = np.where(slopes > max_slope_dz_per_m)[0].astype(int).tolist()
    step_violations = np.where(dz > max_step_dz_m)[0].astype(int).tolist()
    vertical_like = np.where((dz >= vertical_segment_min_dz_m) & (dxy <= vertical_segment_max_dxy_m))[0].astype(int).tolist()
    ok = not slope_violations and not step_violations and not vertical_like
    worst_idx = int(np.argmax(slopes)) if slopes.size else None
    return {
        "available": True,
        "ok": bool(ok),
        "max_slope_dz_per_m": float(np.max(slopes)) if slopes.size else 0.0,
        "max_step_dz_m": float(np.max(dz)) if dz.size else 0.0,
        "min_segment_dxy_m": float(np.min(dxy)) if dxy.size else 0.0,
        "slope_limit_dz_per_m": float(max_slope_dz_per_m),
        "step_limit_dz_m": float(max_step_dz_m),
        "vertical_segment_min_dz_m": float(vertical_segment_min_dz_m),
        "vertical_segment_max_dxy_m": float(vertical_segment_max_dxy_m),
        "slope_violation_count": len(slope_violations),
        "step_violation_count": len(step_violations),
        "vertical_like_segment_count": len(vertical_like),
        "sample_slope_violation_indices": slope_violations[:10],
        "sample_step_violation_indices": step_violations[:10],
        "sample_vertical_like_indices": vertical_like[:10],
        "worst_segment": None if worst_idx is None else {
            "index": worst_idx,
            "dxy_m": float(dxy[worst_idx]),
            "dz_m": float(dz[worst_idx]),
            "slope_dz_per_m": float(slopes[worst_idx]),
            "from": path[worst_idx, :3].astype(float).tolist(),
            "to": path[worst_idx + 1, :3].astype(float).tolist(),
        },
    }


def _status_report(
    *,
    planner: Any,
    path: np.ndarray,
    start: np.ndarray,
    goal: np.ndarray,
    goal_error_m: float,
    finite: bool,
    goal_tolerance_m: float,
    max_slope_dz_per_m: float,
    max_step_dz_m: float,
    vertical_segment_min_dz_m: float,
    vertical_segment_max_dxy_m: float,
    diagnostics: dict[str, Any],
) -> dict[str, Any]:
    valid_shape = bool(path.ndim == 2 and path.shape[0] > 0 and path.shape[1] >= 3)
    start_bounds = _point_bounds_report(planner, start)
    goal_bounds = _point_bounds_report(planner, goal)
    start_in_bounds = start_bounds.get("in_bounds")
    goal_in_bounds = goal_bounds.get("in_bounds")
    reached_goal = bool(goal_error_m <= goal_tolerance_m)
    returned_blocked = _returned_path_blocked_sample_count(planner, path)
    obstacle_clear = True if returned_blocked is None else returned_blocked == 0
    kinematics = _kinematic_report(
        path,
        max_slope_dz_per_m=max_slope_dz_per_m,
        max_step_dz_m=max_step_dz_m,
        vertical_segment_min_dz_m=vertical_segment_min_dz_m,
        vertical_segment_max_dxy_m=vertical_segment_max_dxy_m,
    )

    if not valid_shape or not finite:
        code = "INVALID_PATH"
    elif start_in_bounds is False:
        code = "START_OUT_OF_MAP"
    elif goal_in_bounds is False:
        code = "GOAL_OUT_OF_MAP"
    elif not reached_goal:
        code = "GOAL_NOT_REACHED"
    elif not obstacle_clear:
        code = "HARD_OBSTACLE_COLLISION"
    elif not kinematics.get("ok", True):
        code = "KINEMATICALLY_INVALID"
    else:
        code = "SUCCESS"
    ok = code == "SUCCESS"
    return {
        "ok": ok,
        "code": code,
        "goal_tolerance_m": float(goal_tolerance_m),
        "reached_goal": reached_goal,
        "start_in_bounds": start_in_bounds,
        "goal_in_bounds": goal_in_bounds,
        "map_bounds": _map_bounds_report(planner),
        "start_bounds": start_bounds,
        "goal_bounds": goal_bounds,
        "returned_path_blocked_sample_count": returned_blocked,
        "obstacle_clear": obstacle_clear,
        "kinematics": kinematics,
        "optimizer_accepted": diagnostics.get("last_optimizer_accepted"),
        "optimizer_reject_reason": diagnostics.get("last_optimizer_reject_reason", ""),
    }


def _planner_diagnostics(planner: Any) -> dict[str, Any]:
    fields = (
        "last_path_mode",
        "last_optimizer_enabled",
        "last_optimizer_attempted",
        "last_optimizer_accepted",
        "last_optimizer_reject_reason",
        "last_optimizer_blocked_sample_count",
        "last_raw_path_blocked_sample_count",
        "last_optimizer_elapsed_ms",
        "last_optimizer_initial_cost",
        "last_optimizer_final_cost",
        "last_optimizer_iterations",
        "last_optimizer_accepted_steps",
        "last_optimizer_nonlinear_optimizer",
        "last_optimizer_linear_solver",
        "last_optimizer_linear_solve_fallbacks",
        "last_optimizer_input_states",
        "last_optimizer_output_states",
        "last_optimizer_trajectory_expanded",
        "last_optimizer_interpolation_steps",
        "last_optimizer_call_mode",
        "last_raw_path_count",
        "optimize_trajectory",
        "use_quintic",
        "max_heading_rate",
        "obstacle_thr",
    )
    diagnostics: dict[str, Any] = {}
    for field in fields:
        if hasattr(planner, field):
            diagnostics[field] = _jsonable(getattr(planner, field))
    return diagnostics


def _optimizer_accessor_view(planner: Any) -> tuple[Any | None, str]:
    required = ("get_result_matrix", "get_layers", "get_heights")
    if all(hasattr(planner, name) for name in required):
        return planner, "planner"

    native_planner = getattr(planner, "planner", None)
    if native_planner is None:
        return None, "unavailable"
    getter_name = (
        "get_trajectory_optimizer_wnoj"
        if bool(getattr(planner, "use_quintic", False))
        else "get_trajectory_optimizer"
    )
    getter = getattr(native_planner, getter_name, None)
    if getter is None:
        return None, "unavailable"
    try:
        return getter(), f"native:{getter_name}"
    except Exception as exc:  # pragma: no cover - defensive native-boundary report
        return {"error": str(exc)}, "error"


def _optimizer_accessor_report(planner: Any) -> dict[str, Any]:
    view, source = _optimizer_accessor_view(planner)
    if view is None:
        return {
            "available": False,
            "source": source,
            "native_wrapper_compatible": False,
            "finite": False,
        }
    if isinstance(view, dict) and "error" in view:
        return {
            "available": False,
            "source": source,
            "native_wrapper_compatible": False,
            "finite": False,
            "error": view["error"],
        }

    arrays: dict[str, dict[str, Any]] = {}
    raw: dict[str, np.ndarray] = {}
    for accessor in (
        "get_result_matrix",
        "get_layers",
        "get_heights",
        "get_ceilings",
        "get_opt_init_value",
        "get_opt_init_layer",
        "get_heading_rate",
    ):
        getter = getattr(view, accessor, None)
        if getter is None:
            arrays[accessor] = {"available": False, "shape": [], "finite": False}
            continue
        try:
            arr = np.asarray(getter(), dtype=np.float64)
        except Exception as exc:  # pragma: no cover - defensive native-boundary report
            arrays[accessor] = {
                "available": False,
                "shape": [],
                "finite": False,
                "error": str(exc),
            }
            continue
        raw[accessor] = arr
        arrays[accessor] = {
            "available": True,
            "shape": list(arr.shape),
            "finite": bool(np.all(np.isfinite(arr))),
        }

    result_matrix = raw.get("get_result_matrix")
    layers = raw.get("get_layers")
    heights = raw.get("get_heights")
    opt_init_value = raw.get("get_opt_init_value")
    opt_init_layer = raw.get("get_opt_init_layer")
    row_count = int(result_matrix.shape[0]) if result_matrix is not None and result_matrix.ndim == 2 else 0
    state_dim = int(result_matrix.shape[1]) if result_matrix is not None and result_matrix.ndim == 2 else 0
    compatible = (
        result_matrix is not None
        and result_matrix.ndim == 2
        and row_count > 0
        and layers is not None
        and layers.shape == (row_count,)
        and heights is not None
        and heights.shape == (row_count,)
        and opt_init_value is not None
        and opt_init_value.shape == (state_dim, row_count)
        and opt_init_layer is not None
        and opt_init_layer.shape == (row_count,)
    )
    finite = all(item.get("finite") for item in arrays.values() if item.get("available"))
    return {
        "available": True,
        "source": source,
        "native_wrapper_compatible": bool(compatible),
        "finite": bool(finite),
        "row_count": row_count,
        "state_dim": state_dim,
        "result_matrix_shape": arrays["get_result_matrix"]["shape"],
        "layers_shape": arrays["get_layers"]["shape"],
        "heights_shape": arrays["get_heights"]["shape"],
        "ceilings_shape": arrays["get_ceilings"]["shape"],
        "opt_init_value_shape": arrays["get_opt_init_value"]["shape"],
        "opt_init_layer_shape": arrays["get_opt_init_layer"]["shape"],
        "heading_rate_shape": arrays["get_heading_rate"]["shape"],
        "arrays": arrays,
    }


def build_preview_report(
    *,
    planner: Any,
    runtime_paths: Any,
    result: Any,
    start: np.ndarray,
    goal: np.ndarray,
    tomogram_path: Any | None = None,
    obstacle_thr: float | None = None,
    sample_count: int = 9,
    goal_tolerance_m: float = DEFAULT_GOAL_TOLERANCE_M,
    max_slope_dz_per_m: float = DEFAULT_MAX_SLOPE_DZ_PER_M,
    max_step_dz_m: float = DEFAULT_MAX_STEP_DZ_M,
    vertical_segment_min_dz_m: float = DEFAULT_VERTICAL_SEGMENT_MIN_DZ_M,
    vertical_segment_max_dxy_m: float = DEFAULT_VERTICAL_SEGMENT_MAX_DXY_M,
) -> dict[str, Any]:
    runtime_name = resolve_pct_planner_runtime()
    optimizer_call_mode = str(getattr(planner, "last_optimizer_call_mode", "") or "")
    rust_binary_format = (
        PCT_RUST_FFI_BINARY_FORMAT
        if optimizer_call_mode == "ffi"
        else PCT_RUST_PROCESS_BINARY_FORMAT
    )
    planner_impl_class = type(planner).__name__
    official_native_impl = planner_impl_class == "TomogramPlanner" and runtime_name == "native"
    runtime = {
        "runtime": runtime_name,
        "planner_impl_class": planner_impl_class,
        "official_native_pct_impl": official_native_impl,
        "implementation_note": (
            "official C++ PCT OfflineElePlanner + GTSAM GPMP"
            if official_native_impl
            else "cross-platform replacement path; not the official native PCT planner"
        ),
        "lib_dir": str(runtime_paths.lib_dir),
        "arch": runtime_paths.canonical_arch,
        "python": runtime_paths.python_tag,
        "platform_system": platform.system().lower(),
        "native_binary_format": (
            rust_binary_format
            if runtime_name == "rust_process"
            else PCT_NATIVE_BINARY_FORMAT
        ),
        "rust_optimizer_call_mode": optimizer_call_mode or None,
    }
    input_spec = {
        "start": start.tolist(),
        "goal": goal.tolist(),
    }
    if obstacle_thr is None and hasattr(planner, "obstacle_thr"):
        obstacle_thr = float(getattr(planner, "obstacle_thr"))
    if obstacle_thr is not None:
        input_spec["obstacle_thr"] = float(obstacle_thr)
    tomogram_file = _file_fingerprint(tomogram_path)
    if tomogram_path is not None:
        input_spec["tomogram"] = str(tomogram_path)
    if tomogram_file:
        input_spec["tomogram_file"] = tomogram_file
        if "sha256" in tomogram_file:
            input_spec["tomogram_sha256"] = tomogram_file["sha256"]
        if "size_bytes" in tomogram_file:
            input_spec["tomogram_size_bytes"] = tomogram_file["size_bytes"]
        if "data_shape" in tomogram_file:
            input_spec["tomogram_data_shape"] = tomogram_file["data_shape"]
        if "data_dtype" in tomogram_file:
            input_spec["tomogram_data_dtype"] = tomogram_file["data_dtype"]
        for schema_field in ("schema", "schema_version", "version"):
            if schema_field in tomogram_file:
                input_spec[f"tomogram_{schema_field}"] = tomogram_file[schema_field]
                break
    if result is None or len(result) == 0:
        start_bounds = _point_bounds_report(planner, start)
        goal_bounds = _point_bounds_report(planner, goal)
        return {
            "schema": ACTUAL_SCHEMA,
            "ok": False,
            "status": {
                "ok": False,
                "code": "NO_PATH",
                "goal_tolerance_m": float(goal_tolerance_m),
                "reached_goal": False,
                "start_in_bounds": start_bounds.get("in_bounds"),
                "goal_in_bounds": goal_bounds.get("in_bounds"),
                "map_bounds": _map_bounds_report(planner),
                "start_bounds": start_bounds,
                "goal_bounds": goal_bounds,
            },
            "status_code": "NO_PATH",
            "error": "pct returned no path",
            "input": input_spec,
            "runtime": runtime,
            "tomogram_meta": _tomogram_meta(planner),
            "diagnostics": _planner_diagnostics(planner),
            "optimizer_accessors": _optimizer_accessor_report(planner),
        }

    arr = np.asarray(result, dtype=np.float64)
    finite = bool(np.all(np.isfinite(arr)))
    path_distance = _distance(arr)
    goal_error = _point_error(arr[-1], goal)
    start_error = _point_error(arr[0], start)
    path_samples = _sample_path(arr, sample_count)
    arclength_samples = _sample_path_by_arclength(arr)
    diagnostics = _planner_diagnostics(planner)
    status = _status_report(
        planner=planner,
        path=arr,
        start=start,
        goal=goal,
        goal_error_m=goal_error,
        finite=finite,
        goal_tolerance_m=goal_tolerance_m,
        max_slope_dz_per_m=max_slope_dz_per_m,
        max_step_dz_m=max_step_dz_m,
        vertical_segment_min_dz_m=vertical_segment_min_dz_m,
        vertical_segment_max_dxy_m=vertical_segment_max_dxy_m,
        diagnostics=diagnostics,
    )
    path_report = {
        "shape": list(arr.shape),
        "finite": finite,
        "count": int(len(arr)),
        "distance_m": path_distance,
        "start_error_m": start_error,
        "goal_error_m": goal_error,
        "goal_tolerance_m": float(goal_tolerance_m),
        "reached_goal": status["reached_goal"],
        "blocked_sample_count": status["returned_path_blocked_sample_count"],
        "obstacle_clear": status["obstacle_clear"],
        "bounds_xyz": _bounds_xyz(arr),
        "z_range_m": float(np.nanmax(arr[:, 2]) - np.nanmin(arr[:, 2])) if arr.shape[1] >= 3 else 0.0,
        "samples": {
            "by_index": path_samples,
            "by_arclength_fraction": arclength_samples,
        },
    }
    report = {
        "schema": ACTUAL_SCHEMA,
        "ok": bool(status["ok"]),
        "status": status,
        "status_code": status["code"],
        "planner": "pct",
        "path_count": int(len(arr)),
        "path_distance_m": path_distance,
        "start": start.tolist(),
        "goal": goal.tolist(),
        "first": arr[0, :3].tolist(),
        "last": arr[-1, :3].tolist(),
        "goal_error_m": goal_error,
        "input": input_spec,
        "path": path_report,
        "path_samples": path_samples,
        "diagnostics": diagnostics,
        "pct_diagnostics": diagnostics,
        "tomogram_meta": _tomogram_meta(planner),
        "optimizer_accessors": _optimizer_accessor_report(planner),
        "runtime": runtime,
    }
    return report


def run_preview(args: argparse.Namespace) -> dict[str, Any]:
    start = _as_point(args.start)
    goal = _as_point(args.goal)
    planner, paths = load_tomogram_planner(
        args.tomogram,
        repo_root=args.repo_root,
        obstacle_thr=args.obstacle_thr,
    )
    result = planner.plan(start[:2], goal[:2], float(start[2]), float(goal[2]))
    return build_preview_report(
        planner=planner,
        runtime_paths=paths,
        result=result,
        start=start,
        goal=goal,
        tomogram_path=args.tomogram,
        obstacle_thr=args.obstacle_thr,
        sample_count=getattr(args, "sample_count", 9),
        goal_tolerance_m=getattr(args, "goal_tolerance", DEFAULT_GOAL_TOLERANCE_M),
        max_slope_dz_per_m=getattr(args, "max_slope", DEFAULT_MAX_SLOPE_DZ_PER_M),
        max_step_dz_m=getattr(args, "max_step_dz", DEFAULT_MAX_STEP_DZ_M),
        vertical_segment_min_dz_m=getattr(args, "vertical_segment_min_dz", DEFAULT_VERTICAL_SEGMENT_MIN_DZ_M),
        vertical_segment_max_dxy_m=getattr(args, "vertical_segment_max_dxy", DEFAULT_VERTICAL_SEGMENT_MAX_DXY_M),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run original PCT planner on a tomogram.")
    parser.add_argument("--tomogram", required=True, help="Path to tomogram.pickle")
    parser.add_argument("--start", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--goal", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--obstacle-thr", type=float, default=49.9)
    parser.add_argument("--repo-root", default=None)
    parser.add_argument("--sample-count", type=int, default=9)
    parser.add_argument("--goal-tolerance", type=float, default=DEFAULT_GOAL_TOLERANCE_M)
    parser.add_argument("--max-slope", type=float, default=DEFAULT_MAX_SLOPE_DZ_PER_M)
    parser.add_argument("--max-step-dz", type=float, default=DEFAULT_MAX_STEP_DZ_M)
    parser.add_argument("--vertical-segment-min-dz", type=float, default=DEFAULT_VERTICAL_SEGMENT_MIN_DZ_M)
    parser.add_argument("--vertical-segment-max-dxy", type=float, default=DEFAULT_VERTICAL_SEGMENT_MAX_DXY_M)
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON only")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    native_log = ""
    try:
        if args.json:
            capture = io.StringIO()
            with contextlib.redirect_stdout(capture), contextlib.redirect_stderr(capture):
                report = run_preview(args)
            native_log = capture.getvalue()
            if native_log:
                report["native_log"] = native_log[-8000:]
        else:
            report = run_preview(args)
    except Exception as exc:
        report = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        if not args.json:
            print(report["error"], file=sys.stderr)
        else:
            if native_log:
                report["native_log"] = native_log[-8000:]
            print(json.dumps(report, ensure_ascii=False, indent=2))
        return 2

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
