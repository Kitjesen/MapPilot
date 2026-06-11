#!/usr/bin/env python3
"""Validate large-terrain navigation assets without robot motion."""

from __future__ import annotations

import argparse
import json
import math
import pickle
import platform
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.msgs.numpy_compat import numpy_import_is_safe

np: Any = None
GlobalPlannerService: Any = None
evaluate_plan_safety: Any = None
grid_from_tomogram: Any = None
path_distance: Any = None
inspect_pct_runtime: Any = None
build_large_terrain_assets: Any = None


DEFAULT_MATRIX_ROUTES = ("terrain_short", "terrain_long", "terrain_narrow_gap", "terrain_slope_bypass")
DEFAULT_PLANNERS = ("astar",)
SELECTION_POLICY = "first_route_ok_after_primary"
PCT_OPTIMIZE_TRAJECTORY_ENV = "LINGTU_PCT_OPTIMIZE_TRAJECTORY"
_FALSE_ENV_VALUES = {"0", "false", "no", "off"}


def _load_runtime() -> None:
    global GlobalPlannerService
    global build_large_terrain_assets
    global evaluate_plan_safety
    global grid_from_tomogram
    global inspect_pct_runtime
    global np
    global path_distance

    if np is None:
        import numpy as _np

        np = _np
    if GlobalPlannerService is None:
        from nav.global_planner_service import GlobalPlannerService as _GlobalPlannerService

        GlobalPlannerService = _GlobalPlannerService
    if evaluate_plan_safety is None or grid_from_tomogram is None or path_distance is None:
        from nav.plan_safety import (
            evaluate_plan_safety as _evaluate_plan_safety,
            grid_from_tomogram as _grid_from_tomogram,
            path_distance as _path_distance,
        )

        evaluate_plan_safety = _evaluate_plan_safety
        grid_from_tomogram = _grid_from_tomogram
        path_distance = _path_distance
    if inspect_pct_runtime is None:
        from global_planning.pct_planner_runnable.runtime import (
            inspect_pct_runtime as _inspect_pct_runtime,
        )

        inspect_pct_runtime = _inspect_pct_runtime
    if build_large_terrain_assets is None:
        from sim.engine.scenarios.large_terrain_assets import (
            build_large_terrain_assets as _build_large_terrain_assets,
        )

        build_large_terrain_assets = _build_large_terrain_assets


def _not_exercised_algorithm_backends() -> dict[str, dict[str, str]]:
    return {
        "local_planner": {
            "status": "not_exercised",
            "exercised_by": "large_terrain_global_planning_assets",
            "reason": "large_terrain validates global planning assets and path safety only",
        },
        "path_follower": {
            "status": "not_exercised",
            "exercised_by": "large_terrain_global_planning_assets",
            "reason": "large_terrain does not run tracking or cmd_vel generation",
        },
    }


def _asset_paths(output_dir: str | Path) -> dict[str, str]:
    root = Path(output_dir)
    return {
        "scene_xml": str(root / "large_terrain_scene.xml"),
        "tomogram": str(root / "tomogram.pickle"),
        "map_pcd": str(root / "map.pcd"),
        "metadata": str(root / "metadata.json"),
    }


def _planner_names(planners: Iterable[str]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(planner.lower().strip() for planner in planners if planner.strip()))


def _source_map_artifacts(assets: LargeTerrainAssets) -> dict[str, Any]:
    metadata: dict[str, Any] = {}
    try:
        loaded = json.loads(assets.metadata.read_text(encoding="utf-8"))
        if isinstance(loaded, dict):
            metadata = loaded
    except Exception:
        metadata = {}

    metadata_assets = metadata.get("artifacts") if isinstance(metadata.get("artifacts"), dict) else {}
    map_pcd = dict(metadata_assets.get("map_pcd")) if isinstance(metadata_assets.get("map_pcd"), dict) else {}
    tomogram = dict(metadata_assets.get("tomogram")) if isinstance(metadata_assets.get("tomogram"), dict) else {}
    map_sha = str(map_pcd.get("sha256") or "")
    tomogram_sha = str(tomogram.get("sha256") or "")
    tomogram_source_sha = str(tomogram.get("source_map_sha256") or "")
    try:
        point_count = int(map_pcd.get("point_count") or 0)
    except (TypeError, ValueError):
        point_count = 0
    same_source_pcd = bool(map_sha and point_count > 0)
    same_source_tomogram = bool(
        tomogram_sha
        and tomogram_source_sha
        and map_sha
        and tomogram_source_sha == map_sha
    )
    return {
        "ok": same_source_pcd and same_source_tomogram,
        "source_contract": {
            "same_source_pcd": same_source_pcd,
            "same_source_tomogram": same_source_tomogram,
        },
        "metadata": {
            "path": str(assets.metadata),
            "schema_version": metadata.get("schema_version") or "",
            "source_profile": metadata.get("source_profile") or "",
            "data_source": metadata.get("data_source") or "",
            "mapping_source": metadata.get("mapping_source") or "",
            "frame_id": metadata.get("frame_id") or "",
        },
        "assets": {
            "map_pcd": map_pcd,
            "tomogram": tomogram,
        },
    }


def _pct_runtime_evidence() -> dict[str, Any]:
    global inspect_pct_runtime

    if inspect_pct_runtime is None:
        from global_planning.pct_planner_runnable.runtime import (
            inspect_pct_runtime as _inspect_pct_runtime,
        )

        inspect_pct_runtime = _inspect_pct_runtime
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


def _environment_preflight_report(
    output_dir: str | Path,
    *,
    routes: Iterable[str],
    planners: Iterable[str],
    allow_unstable_windows_numpy: bool = False,
    platform_system: str | None = None,
) -> dict[str, Any] | None:
    planner_names = _planner_names(planners)
    resolved_platform = platform_system or platform.system()
    numpy_safe = numpy_import_is_safe()
    native_runtime = _pct_runtime_evidence() if "pct" in planner_names else None

    blockers: list[str] = []
    blocked_reason = ""
    if (
        resolved_platform.lower() == "windows"
        and not numpy_safe
        and not allow_unstable_windows_numpy
    ):
        blocked_reason = "windows_mingw_numpy_not_accepted"
        blockers.append(
            "Windows/MINGW NumPy large-terrain runtime is not accepted; run this "
            "gate on Linux or pass --allow-unstable-windows-numpy for manual diagnosis"
        )
    pct_runtime_blocked = bool(
        "pct" in planner_names
        and isinstance(native_runtime, dict)
        and native_runtime.get("ok") is not True
    )
    if pct_runtime_blocked:
        blockers.append("PCT native runtime unavailable")
        if not blocked_reason:
            blocked_reason = "pct_native_runtime_unavailable"

    host_guard_required = bool(
        not allow_unstable_windows_numpy
        and (
            not numpy_safe
            or (resolved_platform.lower() == "windows" and pct_runtime_blocked)
        )
    )
    if not blockers or not host_guard_required:
        return None

    environment = {
        "platform_system": resolved_platform,
        "accepted_host": False,
        "accepted_platforms": ["Linux"],
        "numpy_import_safe": numpy_safe,
        "blocked_reason": blocked_reason,
        "blockers": blockers,
        "manual_diagnosis_flag": "--allow-unstable-windows-numpy",
        "claim_boundary": "environment_blocked_no_algorithm_claim",
    }
    return {
        "schema_version": "lingtu.large_terrain_nav_validation.v1",
        "ok": False,
        "execution_mode": "host_guard",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "validation_level": "environment_preflight",
        "selection_policy": SELECTION_POLICY,
        "algorithm_backends": _not_exercised_algorithm_backends(),
        "assets": _asset_paths(output_dir),
        "native_runtime": native_runtime,
        "environment": environment,
        "environment_blockers": blockers,
        "routes": list(routes),
        "planners": list(planner_names),
        "cases": [
            {
                "route": str(route_name),
                "ok": False,
                "planning": [
                    {
                        "planner": planner_name,
                        "planner_requested": planner_name,
                        "selected_planner": planner_name,
                        "feasible": False,
                        "blocked": True,
                        "native_backend_used": False,
                        "native_runtime": native_runtime if planner_name == "pct" else None,
                        "status": "blocked",
                        "failure_category": "environment_runtime",
                        "error": "; ".join(blockers),
                        "route_ok": False,
                        "path": [],
                    }
                    for planner_name in planner_names
                ],
                "selection": {
                    "policy": SELECTION_POLICY,
                    "primary_planner": planner_names[0] if planner_names else "",
                    "selected_planner": "",
                    "selected_route_ok": False,
                    "fallback_used": False,
                    "rejected_planners": [
                        {
                            "planner": planner_name,
                            "selected_planner": planner_name,
                            "feasible": False,
                            "route_ok": False,
                            "reason": "environment_blocked",
                        }
                        for planner_name in planner_names
                    ],
                },
                "path_safety": {},
                "gate_crossing": {"checked": False},
            }
            for route_name in routes
        ],
        "errors": blockers,
    }


def _route_map(assets: LargeTerrainAssets) -> dict[str, TerrainRoute]:
    return {route.name: route for route in assets.routes}


def _path_distance(path: list[tuple[float, float, float]] | list[list[float]]) -> float:
    return path_distance(path)


def _with_route_endpoints(
    route: TerrainRoute,
    path: list[tuple[float, float, float]] | list[list[float]],
) -> list[list[float]]:
    full = [[float(route.start[0]), float(route.start[1]), float(route.start[2])]]
    full.extend([[float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0] for p in path])
    full.append([float(route.goal[0]), float(route.goal[1]), float(route.goal[2])])

    deduped: list[list[float]] = []
    for point in full:
        if not deduped:
            deduped.append(point)
            continue
        if math.hypot(point[0] - deduped[-1][0], point[1] - deduped[-1][1]) > 1e-6:
            deduped.append(point)
    return deduped


def _path_safety(path: list[tuple[float, float, float]] | list[list[float]], tomo: dict[str, Any], *, obstacle_thr: float) -> dict[str, Any]:
    return evaluate_plan_safety(path, grid_from_tomogram(tomo), obstacle_thr=obstacle_thr)


def _path_goal_evidence(
    route: TerrainRoute,
    path: list[tuple[float, float, float]] | list[list[float]],
    *,
    tolerance_m: float = 0.25,
) -> dict[str, Any]:
    route_goal = [float(route.goal[0]), float(route.goal[1])]
    path_goal = [float(path[-1][0]), float(path[-1][1])] if path else []
    distance = (
        float(math.dist(route_goal, path_goal))
        if len(path_goal) == 2
        else float("inf")
    )
    return {
        "checked": True,
        "reached_goal": bool(distance <= float(tolerance_m)),
        "tolerance_m": float(tolerance_m),
        "route_goal_xy": route_goal,
        "path_goal_xy": path_goal,
        "goal_shift_m": None if not math.isfinite(distance) else round(distance, 4),
    }


def _gate_crossing(path: list[tuple[float, float, float]] | list[list[float]]) -> dict[str, Any]:
    xy = np.asarray([[float(p[0]), float(p[1])] for p in path], dtype=float)
    if len(xy) == 0:
        return {"checked": False, "passed_gate": False}
    near_wall = xy[np.abs(xy[:, 0]) < 0.45]
    if len(near_wall) == 0:
        return {"checked": True, "passed_gate": False, "min_y_at_wall": None}
    min_y = float(np.min(near_wall[:, 1]))
    max_y = float(np.max(near_wall[:, 1]))
    return {
        "checked": True,
        "passed_gate": bool(min_y > -1.30 and max_y < 1.15),
        "min_y_at_wall": round(min_y, 4),
        "max_y_at_wall": round(max_y, 4),
    }


def _service_plan_report(svc: Any) -> dict[str, Any]:
    try:
        report = getattr(svc, "last_plan_report", {}) or {}
    except Exception:
        return {}
    return dict(report) if isinstance(report, dict) else {}


def _planner_value(value: Any, default: str) -> str:
    return str(value or default).lower().strip()


def _pct_optimizer_enabled_from_env(env: dict[str, str] | None = None, *, default: bool = True) -> bool:
    values = os.environ if env is None else env
    raw = values.get(PCT_OPTIMIZE_TRAJECTORY_ENV)
    if raw is None:
        return bool(default)
    text = str(raw).strip().lower()
    if not text:
        return bool(default)
    return text not in _FALSE_ENV_VALUES


def _pct_planning_mode_fields(
    planner_name: str,
    *,
    env: dict[str, str] | None = None,
    optimizer_enabled: bool | None = None,
    planner_diagnostics: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if planner_name != "pct":
        return {}
    diagnostics = planner_diagnostics if isinstance(planner_diagnostics, dict) else {}
    diag_enabled = diagnostics.get("pct_optimizer_enabled")
    if diag_enabled in (True, False):
        enabled = bool(diag_enabled)
    elif optimizer_enabled is None:
        enabled = _pct_optimizer_enabled_from_env(env)
    else:
        enabled = bool(optimizer_enabled)
    path_mode = str(diagnostics.get("pct_planner_path_mode") or "").strip()
    if path_mode not in {"native_astar_raw_path", "optimized_trajectory"}:
        path_mode = "optimized_trajectory" if enabled else "native_astar_raw_path"

    attempted = diagnostics.get("pct_optimizer_attempted")
    accepted = diagnostics.get("pct_optimizer_accepted")
    try:
        blocked_count = int(diagnostics.get("pct_optimizer_blocked_sample_count") or 0)
    except (TypeError, ValueError):
        blocked_count = 0
    try:
        raw_blocked_count = int(diagnostics.get("pct_optimizer_raw_blocked_sample_count") or 0)
    except (TypeError, ValueError):
        raw_blocked_count = 0
    return {
        "pct_optimizer_enabled": enabled,
        "pct_optimizer_attempted": (
            bool(attempted) if attempted in (True, False) else None
        ),
        "pct_optimizer_accepted": (
            bool(accepted) if accepted in (True, False) else None
        ),
        "pct_optimizer_reject_reason": str(
            diagnostics.get("pct_optimizer_reject_reason") or ""
        ),
        "pct_optimizer_blocked_sample_count": blocked_count,
        "pct_optimizer_raw_blocked_sample_count": raw_blocked_count,
        "pct_planner_path_mode": path_mode,
    }


def _should_isolate_pct_planner(planner_name: str) -> bool:
    if planner_name != "pct":
        return False
    if os.environ.get("LINGTU_LARGE_TERRAIN_PLAN_CHILD") == "1":
        return False
    return getattr(GlobalPlannerService, "__module__", "") == "nav.global_planner_service"


def _tail_text(value: str, *, max_chars: int = 4000) -> str:
    if len(value) <= max_chars:
        return value
    return value[-max_chars:]


def _pct_child_exit_error(returncode: int | None) -> str:
    if returncode is None:
        return "PCT planner child process did not return an exit code"
    if returncode < 0:
        signum = -int(returncode)
        try:
            signame = signal.Signals(signum).name
        except ValueError:
            signame = f"signal_{signum}"
        return f"PCT planner child process terminated by signal {signum} ({signame})"
    return f"PCT planner child process exited with code {returncode}"


def _pct_child_failure_report(
    planner_name: str,
    route: TerrainRoute,
    *,
    native_runtime: dict[str, Any] | None,
    returncode: int | None,
    error: str,
    stdout: str = "",
    stderr: str = "",
    pct_optimizer_enabled: bool | None = None,
) -> dict[str, Any]:
    report = {
        "planner": planner_name,
        "planner_requested": planner_name,
        "selected_planner": planner_name,
        "fallback_reason": "",
        "plan_safety_policy": "",
        "rejected_plans": [],
        "backend_class": "",
        "backend_requested_class": "",
        "feasible": False,
        "blocked": True,
        "backend_available": bool(native_runtime and native_runtime.get("ok") is True),
        "backend_requested_available": bool(native_runtime and native_runtime.get("ok") is True),
        "native_backend_used": False,
        "native_runtime": native_runtime,
        "status": "failed",
        "failure_category": "planner_process_crash",
        "load_error": "",
        "error": error,
        "returncode": returncode,
        "stdout_tail": _tail_text(stdout),
        "stderr_tail": _tail_text(stderr),
        "plan_ms": 0.0,
        "start": list(route.start),
        "goal": list(route.goal),
        "path": [],
    }
    report.update(
        _pct_planning_mode_fields(
            planner_name,
            optimizer_enabled=pct_optimizer_enabled,
        )
    )
    return report


def _plan_with_backend_subprocess(
    planner_name: str,
    assets: LargeTerrainAssets,
    route: TerrainRoute,
    *,
    obstacle_thr: float,
    native_runtime: dict[str, Any] | None,
) -> dict[str, Any]:
    run_root = assets.metadata.parent / "_plan_cases"
    run_root.mkdir(parents=True, exist_ok=True)
    json_out = run_root / f"{route.name}_{planner_name}.json"
    cmd = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--internal-plan-case",
        "--output-dir",
        str(assets.metadata.parent),
        "--plan-route",
        route.name,
        "--plan-planner",
        planner_name,
        "--obstacle-thr",
        str(obstacle_thr),
        "--json-out",
        str(json_out),
    ]
    env = dict(os.environ)
    env["LINGTU_LARGE_TERRAIN_PLAN_CHILD"] = "1"
    if planner_name == "pct":
        env.setdefault(PCT_OPTIMIZE_TRAJECTORY_ENV, "0")
    pct_optimizer_enabled = _pct_optimizer_enabled_from_env(env)
    py_paths = [str(SRC), str(ROOT)]
    if env.get("PYTHONPATH"):
        py_paths.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(py_paths)
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(ROOT),
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=float(os.environ.get("LINGTU_LARGE_TERRAIN_PLAN_TIMEOUT_S", "180")),
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        return _pct_child_failure_report(
            planner_name,
            route,
            native_runtime=native_runtime,
            returncode=None,
            error=f"PCT planner child process timed out after {exc.timeout}s",
            stdout=exc.stdout or "",
            stderr=exc.stderr or "",
            pct_optimizer_enabled=pct_optimizer_enabled,
        )

    if json_out.exists():
        try:
            loaded = json.loads(json_out.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                loaded.setdefault("child_process", {})
                loaded["child_process"].update(
                    {
                        "returncode": proc.returncode,
                        "stdout_tail": _tail_text(proc.stdout or ""),
                        "stderr_tail": _tail_text(proc.stderr or ""),
                    }
                )
                for key, value in _pct_planning_mode_fields(
                    planner_name,
                    optimizer_enabled=pct_optimizer_enabled,
                ).items():
                    loaded.setdefault(key, value)
                return loaded
        except Exception:
            pass

    return _pct_child_failure_report(
        planner_name,
        route,
        native_runtime=native_runtime,
        returncode=proc.returncode,
        error=_pct_child_exit_error(proc.returncode),
        stdout=proc.stdout or "",
        stderr=proc.stderr or "",
        pct_optimizer_enabled=pct_optimizer_enabled,
    )


def _plan_with_backend_direct(
    planner_name: str,
    assets: LargeTerrainAssets,
    route: TerrainRoute,
    *,
    obstacle_thr: float,
) -> dict[str, Any]:
    planner_name = planner_name.lower().strip()
    native_runtime = _pct_runtime_evidence() if planner_name == "pct" else None
    environment_blocked = bool(
        planner_name == "pct"
        and isinstance(native_runtime, dict)
        and native_runtime.get("ok") is not True
    )
    svc = GlobalPlannerService(
        planner_name=planner_name,
        tomogram=str(assets.tomogram),
        downsample_dist=0.2,
        obstacle_thr=obstacle_thr,
    )
    try:
        svc.setup()
    except Exception as exc:
        report = {
            "planner": planner_name,
            "planner_requested": planner_name,
            "selected_planner": planner_name,
            "fallback_reason": "",
            "plan_safety_policy": "",
            "rejected_plans": [],
            "backend_class": "",
            "feasible": False,
            "blocked": True,
            "error": str(exc),
            "native_backend_used": False,
            "native_runtime": native_runtime,
            "status": "blocked" if environment_blocked else "failed",
            "failure_category": "environment_runtime" if environment_blocked else "planner_runtime",
            "plan_ms": 0.0,
            "start": list(route.start),
            "goal": list(route.goal),
            "path": [],
        }
        report.update(_pct_planning_mode_fields(planner_name))
        return report
    backend = svc._backend
    backend_available = bool(getattr(backend, "available", True))
    start_t = time.perf_counter()
    plan_report: dict[str, Any] = {}
    try:
        path, plan_ms = svc.plan(
            np.asarray(route.start, dtype=float),
            np.asarray(route.goal, dtype=float),
            safe_goal_tolerance=0.0,
        )
        plan_report = _service_plan_report(svc)
        elapsed_ms = float(plan_ms if plan_ms is not None else (time.perf_counter() - start_t) * 1000.0)
        error = ""
    except Exception as exc:
        path = []
        plan_report = _service_plan_report(svc)
        elapsed_ms = (time.perf_counter() - start_t) * 1000.0
        error = str(exc)
    selected_planner = _planner_value(plan_report.get("selected_planner"), planner_name)
    primary_planner = _planner_value(plan_report.get("primary_planner"), planner_name)
    selected_backend = backend
    if selected_planner != planner_name:
        selected_backend = getattr(svc, "_fallback_backend", None) or backend
    selected_backend_available = bool(getattr(selected_backend, "available", True))
    native_backend_used = bool(selected_planner == "pct" and selected_backend_available and path)
    status = "passed" if path else "blocked" if environment_blocked else "failed"
    rejected_plans = plan_report.get("rejected_plans", [])
    planner_diagnostics = (
        dict(plan_report.get("planner_diagnostics"))
        if isinstance(plan_report.get("planner_diagnostics"), dict)
        else {}
    )
    report = {
        "planner": planner_name,
        "planner_requested": primary_planner,
        "selected_planner": selected_planner,
        "fallback_reason": str(plan_report.get("fallback_reason") or ""),
        "plan_safety_policy": str(plan_report.get("policy") or ""),
        "rejected_plans": rejected_plans if isinstance(rejected_plans, list) else [],
        "reached_goal": bool(plan_report.get("reached_goal", bool(path))),
        "backend_class": selected_backend.__class__.__name__ if selected_backend is not None else "",
        "backend_requested_class": backend.__class__.__name__ if backend is not None else "",
        "feasible": bool(path),
        "blocked": bool(not path),
        "backend_available": selected_backend_available,
        "backend_requested_available": backend_available,
        "native_backend_used": native_backend_used,
        "native_runtime": native_runtime,
        "planner_diagnostics": planner_diagnostics,
        "status": status,
        "failure_category": (
            ""
            if path
            else "environment_runtime"
            if environment_blocked
            else "planner_runtime"
        ),
        "load_error": str(getattr(backend, "_load_error", "")) if backend is not None else "",
        "error": error,
        "plan_ms": round(float(elapsed_ms), 3),
        "start": list(route.start),
        "goal": list(route.goal),
        "path": [[float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0] for p in path],
    }
    report.update(
        _pct_planning_mode_fields(
            planner_name,
            planner_diagnostics=planner_diagnostics,
        )
    )
    return report


def _plan_with_backend(
    planner_name: str,
    assets: LargeTerrainAssets,
    route: TerrainRoute,
    *,
    obstacle_thr: float,
) -> dict[str, Any]:
    planner_name = planner_name.lower().strip()
    native_runtime = _pct_runtime_evidence() if planner_name == "pct" else None
    environment_blocked = bool(
        planner_name == "pct"
        and isinstance(native_runtime, dict)
        and native_runtime.get("ok") is not True
    )
    if _should_isolate_pct_planner(planner_name) and not environment_blocked:
        return _plan_with_backend_subprocess(
            planner_name,
            assets,
            route,
            obstacle_thr=obstacle_thr,
            native_runtime=native_runtime,
        )
    return _plan_with_backend_direct(
        planner_name,
        assets,
        route,
        obstacle_thr=obstacle_thr,
    )


def _run_internal_plan_case(
    output_dir: Path,
    *,
    route_name: str,
    planner_name: str,
    obstacle_thr: float,
) -> dict[str, Any]:
    _load_runtime()
    assets = build_large_terrain_assets(output_dir)
    route_by_name = _route_map(assets)
    if route_name not in route_by_name:
        raise ValueError(f"unknown route {route_name!r}")
    return _plan_with_backend_direct(
        planner_name,
        assets,
        route_by_name[route_name],
        obstacle_thr=obstacle_thr,
    )


def _plan_evidence(
    route: TerrainRoute,
    plan: dict[str, Any],
    tomo: dict[str, Any],
    *,
    obstacle_thr: float,
) -> dict[str, Any]:
    raw_path = plan.get("path") or []
    path = _with_route_endpoints(route, raw_path)
    direct = float(np.linalg.norm(np.asarray(route.goal[:2]) - np.asarray(route.start[:2])))
    routed = _path_distance(path)
    route_distance_tolerance_m = 0.05
    safety = _path_safety(path, tomo, obstacle_thr=obstacle_thr)
    path_goal = _path_goal_evidence(route, raw_path)
    gate = (
        _gate_crossing(path)
        if route.name in {"terrain_long", "terrain_narrow_gap", "terrain_complex_slalom"}
        else {"checked": False}
    )
    ok = (
        bool(plan.get("feasible"))
        and bool(path_goal["reached_goal"])
        and bool(safety["ok"])
        and routed + route_distance_tolerance_m >= route.min_routed_distance_m
        and (not gate.get("checked") or bool(gate.get("passed_gate")))
    )
    return {
        "ok": bool(ok),
        "path": path,
        "metrics": {
            "direct_distance_m": round(direct, 4),
            "route_distance_m": round(routed, 4),
            "min_required_route_distance_m": route.min_routed_distance_m,
            "route_distance_tolerance_m": route_distance_tolerance_m,
        },
        "path_goal": path_goal,
        "path_safety": safety,
        "gate_crossing": gate,
    }


def _selection_evidence(planning: list[dict[str, Any]]) -> dict[str, Any]:
    primary = planning[0] if planning else {}
    selected = next(
        (
            plan
            for plan in planning
            if bool(plan.get("feasible")) and bool(plan.get("route_ok"))
        ),
        None,
    )
    rejected = [
        {
            "planner": str(plan.get("planner_requested") or plan.get("planner", "")),
            "selected_planner": str(plan.get("selected_planner") or plan.get("planner", "")),
            "feasible": bool(plan.get("feasible")),
            "route_ok": bool(plan.get("route_ok")),
            "reason": (
                "environment_blocked"
                if plan.get("failure_category") == "environment_runtime"
                else
                "not_feasible"
                if not plan.get("feasible")
                else "unsafe_or_invalid_route"
                if not plan.get("route_ok")
                else ""
            ),
        }
        for plan in planning
        if not (bool(plan.get("feasible")) and bool(plan.get("route_ok")))
    ]
    primary_planner = str(primary.get("planner_requested") or primary.get("planner", ""))
    selected_planner = str(selected.get("selected_planner") or selected.get("planner", "")) if selected else ""
    return {
        "policy": SELECTION_POLICY,
        "primary_planner": primary_planner,
        "selected_planner": selected_planner,
        "selected_route_ok": bool(selected),
        "fallback_used": bool(selected and primary and selected_planner != primary_planner),
        "rejected_planners": rejected,
    }


def run_validation(
    output_dir: str | Path,
    *,
    routes: Iterable[str] = DEFAULT_MATRIX_ROUTES,
    planners: Iterable[str] = DEFAULT_PLANNERS,
    obstacle_thr: float = 49.9,
) -> dict[str, Any]:
    _load_runtime()
    assets = build_large_terrain_assets(output_dir)
    map_artifacts = _source_map_artifacts(assets)
    route_by_name = _route_map(assets)
    with assets.tomogram.open("rb") as fh:
        tomo = pickle.load(fh)

    cases = []
    all_ok = True
    planner_names = _planner_names(planners)
    native_runtime = _pct_runtime_evidence() if "pct" in planner_names else None
    environment_blockers: list[str] = []
    if isinstance(native_runtime, dict) and native_runtime.get("ok") is not True:
        environment_blockers.append("PCT native runtime unavailable")
    for route_name in routes:
        if route_name not in route_by_name:
            raise ValueError(f"unknown route {route_name!r}")
        route = route_by_name[route_name]
        planning = [_plan_with_backend(planner, assets, route, obstacle_thr=obstacle_thr) for planner in planner_names]
        plan_evidence = [
            _plan_evidence(route, plan, tomo, obstacle_thr=obstacle_thr)
            for plan in planning
        ]
        for plan, evidence in zip(planning, plan_evidence):
            plan["metrics"] = evidence["metrics"]
            plan["path_goal"] = evidence["path_goal"]
            plan["path_safety"] = evidence["path_safety"]
            plan["gate_crossing"] = evidence["gate_crossing"]
            plan["route_ok"] = evidence["ok"]
        primary_idx = next((idx for idx, item in enumerate(planning) if item.get("feasible")), 0)
        primary = planning[primary_idx] if planning else {}
        primary_evidence = plan_evidence[primary_idx] if plan_evidence else {}
        safe_primary_idx = next(
            (
                idx
                for idx, item in enumerate(planning)
                if item.get("feasible") and bool((item.get("path_safety") or {}).get("ok"))
            ),
            None,
        )
        safe_primary = planning[safe_primary_idx] if safe_primary_idx is not None else None
        selection = _selection_evidence(planning)
        case_ok = (
            bool(planning)
            and all(bool(item.get("feasible")) for item in planning)
            and all(bool(item.get("route_ok")) for item in planning)
        )
        all_ok = all_ok and case_ok
        cases.append(
            {
                "route": route.name,
                "description": route.description,
                "ok": case_ok,
                "assets": {
                    "scene_xml": str(assets.scene_xml),
                    "tomogram": str(assets.tomogram),
                    "map_pcd": str(assets.map_pcd),
                    "metadata": str(assets.metadata),
                    "start": list(route.start),
                    "goal": list(route.goal),
                },
                "map_artifacts": map_artifacts,
                "deliverable_contract": {
                    "checks": {
                        "same_source_map_artifact": map_artifacts.get("ok") is True,
                    }
                },
                "planning": planning,
                "primary_planner": primary.get("planner", ""),
                "safe_primary_planner": safe_primary.get("planner", "") if safe_primary else "",
                "selection": selection,
                "metrics": primary_evidence.get("metrics", {}),
                "path_safety": primary_evidence.get("path_safety", {}),
                "gate_crossing": primary_evidence.get("gate_crossing", {}),
            }
        )

    return {
        "schema_version": "lingtu.large_terrain_nav_validation.v1",
        "ok": all_ok,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "validation_level": "global_planning_assets",
        "selection_policy": SELECTION_POLICY,
        "algorithm_backends": _not_exercised_algorithm_backends(),
        "assets": {
            "scene_xml": str(assets.scene_xml),
            "tomogram": str(assets.tomogram),
            "map_pcd": str(assets.map_pcd),
            "metadata": str(assets.metadata),
        },
        "map_artifacts": map_artifacts,
        "deliverable_contract": {
            "checks": {
                "same_source_map_artifact": map_artifacts.get("ok") is True,
            }
        },
        "native_runtime": native_runtime,
        "environment_blockers": environment_blockers,
        "routes": list(routes),
        "planners": list(planner_names),
        "cases": cases,
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts/large_terrain_nav_validation")
    parser.add_argument("--routes", default=",".join(DEFAULT_MATRIX_ROUTES))
    parser.add_argument("--planners", default=",".join(DEFAULT_PLANNERS))
    parser.add_argument("--json-out", type=Path, default=ROOT / "artifacts/large_terrain_nav_validation/report.json")
    parser.add_argument("--obstacle-thr", type=float, default=49.9)
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--internal-plan-case", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--plan-route", default="", help=argparse.SUPPRESS)
    parser.add_argument("--plan-planner", default="", help=argparse.SUPPRESS)
    parser.add_argument(
        "--allow-unstable-windows-numpy",
        action="store_true",
        help="Manual diagnosis only: bypass the Windows/MINGW NumPy guard.",
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    if args.internal_plan_case:
        report = _run_internal_plan_case(
            args.output_dir,
            route_name=args.plan_route,
            planner_name=args.plan_planner,
            obstacle_thr=args.obstacle_thr,
        )
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report.get("feasible") else 1

    routes = tuple(item.strip() for item in args.routes.split(",") if item.strip())
    planners = tuple(item.strip() for item in args.planners.split(",") if item.strip())
    report = _environment_preflight_report(
        args.output_dir,
        routes=routes,
        planners=planners,
        allow_unstable_windows_numpy=args.allow_unstable_windows_numpy,
    )
    if report is None:
        report = run_validation(
            args.output_dir,
            routes=routes,
            planners=planners,
            obstacle_thr=args.obstacle_thr,
        )
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report.get("ok"):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
