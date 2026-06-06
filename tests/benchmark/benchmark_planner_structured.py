#!/usr/bin/env python3
"""Structured planner benchmark — runs A* and PCT on a synthetic map fixture.

Emits structured JSON results per planner/route pair (schema_version=1)
for CI/analysis ingestion.  Runs without ROS2.

Usage:
    PYTHONPATH=src python tests/benchmark/benchmark_planner_structured.py
    PYTHONPATH=src python tests/benchmark/benchmark_planner_structured.py --planners astar
    PYTHONPATH=src python tests/benchmark/benchmark_planner_structured.py --json-out /tmp/report.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import pickle
import subprocess
import sys
import time
import uuid
from pathlib import Path
from typing import Any

# ---- sys.path setup (mirrors src/core/tests/conftest.py) ----
_REPO_ROOT = Path(__file__).resolve().parents[2]
_SRC = _REPO_ROOT / "src"
for _p in [str(_REPO_ROOT), str(_SRC)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

# ---- imports after path setup ----
from core.efficiency_status import benchmark_claim_metadata
from core.efficiency_status import classify_benchmark_error

IMPORT_ERROR: str | None = None
GlobalPlannerService: Any | None = None
evaluate_backend_path_safety: Any | None = None
build_large_terrain_assets: Any | None = None
LargeTerrainAssets: Any | None = None

PLATFORM_ASSUMPTION = (
    "synthetic planner regression benchmark; not S100P real-robot performance"
)
SCHEMA_VERSION = 1

# Default benchmark routes (subset of large_terrain_routes() names)
DEFAULT_ROUTES = ("terrain_short", "terrain_long", "terrain_narrow_gap")
DEFAULT_PLANNERS = ("astar", "pct")


def _module_import_probe(module_name: str) -> str | None:
    """Return an import error string without risking this process."""
    proc = subprocess.run(
        [sys.executable, "-c", f"import {module_name}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if proc.returncode == 0:
        return None
    detail = (proc.stderr or proc.stdout or "").strip().splitlines()
    tail = detail[-1] if detail else f"exit code {proc.returncode}"
    return f"missing dependency: {module_name} ({tail})"


def _ensure_benchmark_dependencies() -> str | None:
    """Load heavy planner dependencies only when a real benchmark will run."""
    global IMPORT_ERROR
    global GlobalPlannerService
    global evaluate_backend_path_safety
    global build_large_terrain_assets
    global LargeTerrainAssets

    if IMPORT_ERROR is not None:
        return IMPORT_ERROR
    if (
        GlobalPlannerService is not None
        and evaluate_backend_path_safety is not None
        and build_large_terrain_assets is not None
    ):
        return None

    numpy_error = _module_import_probe("numpy")
    if numpy_error is not None:
        IMPORT_ERROR = numpy_error
        return IMPORT_ERROR

    try:
        from nav.global_planner_service import GlobalPlannerService as _PlannerService
        from nav.plan_safety import evaluate_backend_path_safety as _evaluate_safety
        from sim.engine.scenarios.large_terrain_assets import (
            LargeTerrainAssets as _LargeTerrainAssets,
            build_large_terrain_assets as _build_assets,
        )
    except Exception as exc:  # pragma: no cover - environment dependent.
        IMPORT_ERROR = f"missing benchmark dependency: {exc}"
        return IMPORT_ERROR

    GlobalPlannerService = _PlannerService
    evaluate_backend_path_safety = _evaluate_safety
    build_large_terrain_assets = _build_assets
    LargeTerrainAssets = _LargeTerrainAssets
    return None


# ------------------------------------------------------------------ #
#  Fixture helpers                                                    #
# ------------------------------------------------------------------ #

def _sha256_file(path: Path) -> str:
    """Compute the SHA-256 hex digest of a file."""
    d = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            d.update(chunk)
    return d.hexdigest()


def _write_gate_metadata(fixture_dir: Path) -> dict[str, Any]:
    """Rewrite metadata.json so PCT _map_artifact_gate passes.

    build_large_terrain_assets emits a metadata.json without the
    ``artifacts`` and provenance fields required by
    ``validate_saved_map_artifact_dir``.  This function computes the
    missing fields (sha256, point_count, shape, ...) and rewrites the file.
    """
    tomogram_path = fixture_dir / "tomogram.pickle"
    map_pcd_path = fixture_dir / "map.pcd"

    tomogram_sha = _sha256_file(tomogram_path)
    map_pcd_sha = _sha256_file(map_pcd_path)
    tomogram_size = tomogram_path.stat().st_size
    map_pcd_size = map_pcd_path.stat().st_size

    # Count PCD points (the ASCII PCD emitted by _write_ascii_pcd
    # has a header with POINTS N, then data rows.)
    pcd_text = map_pcd_path.read_text(encoding="utf-8")
    point_count = 0
    for line in pcd_text.splitlines():
        if line.startswith("POINTS"):
            try:
                point_count = int(line.split()[1])
            except (IndexError, ValueError):
                pass
            break

    # Load tomogram pickle to extract data shape
    with tomogram_path.open("rb") as fh:
        tomo_raw = pickle.load(fh)
    tomo_data = tomo_raw.get("data") if isinstance(tomo_raw, dict) else None
    tomogram_shape = list(tomo_data.shape) if tomo_data is not None else []

    from datetime import datetime, timezone

    frame_id = "map"
    source_profile = "benchmark"
    data_source = "synthetic"
    slam_source = "benchmark"

    meta = {
        "schema_version": 1,
        "name": "benchmark_fixture",
        "source": "synthetic",
        "source_profile": source_profile,
        "data_source": data_source,
        "slam_source": slam_source,
        "localization_source": "benchmark",
        "mapping_source": "benchmark",
        "frame_id": frame_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "artifacts": {
            "map_pcd": {
                "path": map_pcd_path.name,
                "sha256": map_pcd_sha,
                "format": "pcd",
                "size": map_pcd_size,
                "frame_id": frame_id,
                "source_profile": source_profile,
                "data_source": data_source,
                "slam_source": slam_source,
                "point_count": point_count,
            },
            "tomogram": {
                "path": tomogram_path.name,
                "sha256": tomogram_sha,
                "format": "pickle",
                "size": tomogram_size,
                "frame_id": frame_id,
                "source_profile": source_profile,
                "data_source": data_source,
                "source_map_sha256": map_pcd_sha,
                "shape": tomogram_shape,
            },
        },
    }

    (fixture_dir / "metadata.json").write_text(
        json.dumps(meta, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return meta


def _build_fixture(output_dir: Path) -> LargeTerrainAssets:
    """Generate synthetic map artifacts and return asset descriptor."""
    import_error = _ensure_benchmark_dependencies()
    if import_error is not None:
        raise RuntimeError(import_error)
    assert build_large_terrain_assets is not None

    fixture_dir = output_dir / "fixture"
    assets = build_large_terrain_assets(
        fixture_dir,
        shape_xy=(121, 81),
        resolution=0.2,
    )
    # Rewrite metadata with artifacts section for PCT gate
    _write_gate_metadata(fixture_dir)
    return assets


# ------------------------------------------------------------------ #
#  Benchmark runner                                                   #
# ------------------------------------------------------------------ #

def _path_distance(path: list[Any]) -> float:
    """Total Euclidean distance along a list of (x,y,z) waypoints."""
    if not path or len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(path)):
        x1, y1 = float(path[i][0]), float(path[i][1])
        x0, y0 = float(path[i - 1][0]), float(path[i - 1][1])
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def _result_base(planner: str) -> dict[str, Any]:
    metadata = benchmark_claim_metadata(generated_at=time.time())
    return {
        "schema_version": SCHEMA_VERSION,
        "planner": planner,
        "ok": True,
        "status": "pass",
        "selected_backend": "",
        "fallback_used": False,
        "direct_goal_fallback_used": False,
        "latency_ms": 0.0,
        "path_length_m": 0.0,
        "waypoint_count": 0,
        "path_safety_score": 0.0,
        "map_artifact_gate_ok": True,
        "platform_assumption": PLATFORM_ASSUMPTION,
        **metadata,
    }


def benchmark_planner(
    planner_name: str,
    fixture_dir: Path,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    *,
    obstacle_thr: float = 49.9,
) -> dict[str, Any]:
    """Run a single planner benchmark and return a structured result dict.

    The returned dict conforms to schema_version 1 and is JSON-serializable.
    """
    planner_name = planner_name.lower().strip()
    tomogram_path = fixture_dir / "tomogram.pickle"
    result = _result_base(planner_name)
    if GlobalPlannerService is None:
        import_error = _ensure_benchmark_dependencies()
        if import_error is not None:
            result["ok"] = False
            result["status"] = "skip"
            result["error"] = import_error
            result["map_artifact_gate_ok"] = False
            return result
    assert GlobalPlannerService is not None

    svc = GlobalPlannerService(
        planner_name=planner_name,
        tomogram=str(tomogram_path),
        downsample_dist=0.2,
        obstacle_thr=obstacle_thr,
        plan_safety_policy="observe",
    )

    try:
        svc.setup()
    except Exception as exc:
        status = classify_benchmark_error(str(exc))
        result["ok"] = False
        result["status"] = status
        result["error"] = f"setup failed: {exc}"
        result["map_artifact_gate_ok"] = False
        return result

    # Read gate (setup() stores the validated gate)
    gate = svc.map_artifact_gate
    result["map_artifact_gate_ok"] = bool(gate.get("ok", True))

    # Determine selected backend class name
    backend = svc._backend
    selected_backend_name = ""
    if backend is not None:
        cls_name = type(backend).__name__
        selected_backend_name = cls_name.lstrip("_").lower()

    start_time = time.perf_counter()
    try:
        path, plan_ms = svc.plan(
            tuple(float(value) for value in start),
            tuple(float(value) for value in goal),
            safe_goal_tolerance=0.0,
        )
        elapsed_ms = (
            float(plan_ms)
            if plan_ms is not None
            else (time.perf_counter() - start_time) * 1000.0
        )
        result["latency_ms"] = round(elapsed_ms, 3)
    except Exception as exc:
        result["selected_backend"] = selected_backend_name
        status = classify_benchmark_error(str(exc))
        result["ok"] = False
        result["status"] = status
        result["error"] = f"plan failed: {exc}"
        result["latency_ms"] = round(
            (time.perf_counter() - start_time) * 1000.0, 3
        )
        return result

    # Fallback / direct-goal info from last_plan_report
    report = svc.last_plan_report
    result["fallback_used"] = bool(report.get("fallback_reason"))
    result["direct_goal_fallback_used"] = not bool(
        report.get("reached_goal", True)
    )

    # Path metrics
    result["selected_backend"] = selected_backend_name
    result["path_length_m"] = round(_path_distance(path), 4)
    result["waypoint_count"] = len(path)

    # Safety score (1.0 = fully safe, 0.0 = fully blocked)
    if path:
        try:
            assert evaluate_backend_path_safety is not None
            path_list = [
                [float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0]
                for p in path
            ]
            safety = evaluate_backend_path_safety(
                path_list, backend, obstacle_thr=obstacle_thr
            )
            if safety is not None:
                blocked_frac = float(safety.get("blocked_fraction", 0.0))
                result["path_safety_score"] = round(
                    max(0.0, 1.0 - blocked_frac), 4
                )
        except Exception:
            pass  # remain at default 0.0

    return result


# ------------------------------------------------------------------ #
#  CLI                                                                #
# ------------------------------------------------------------------ #

def _resolve_routes(
    assets: LargeTerrainAssets,
    names: list[str],
) -> list[Any]:
    """Resolve route names to route objects, warning on unknown names."""
    route_map = {r.name: r for r in assets.routes}
    resolved: list[Any] = []
    for name in names:
        if name in route_map:
            resolved.append(route_map[name])
        else:
            print(
                f"Warning: unknown route {name!r}, skipping. "
                f"Available: {list(route_map)}",
                file=sys.stderr,
            )
    return resolved


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=_REPO_ROOT / "artifacts/benchmark_planner",
        help="Output directory for fixture and report (default: artifacts/benchmark_planner)",
    )
    parser.add_argument(
        "--planners",
        default=",".join(DEFAULT_PLANNERS),
        help="Comma-separated planner names (default: astar,pct)",
    )
    parser.add_argument(
        "--routes",
        default=",".join(DEFAULT_ROUTES),
        help="Comma-separated route names (default: terrain_short,terrain_long,terrain_narrow_gap)",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Path for JSON report (default: <output-dir>/report.json)",
    )
    parser.add_argument(
        "--keep-fixture",
        action="store_true",
        help="Keep the generated fixture directory after run (for inspection)",
    )
    return parser


def write_import_skip_report(
    *,
    json_out: Path,
    planners: list[str],
    routes: list[str],
    error: str,
) -> int:
    generated_at = time.time()
    result = {
        "schema_version": SCHEMA_VERSION,
        "planner": "all",
        "ok": False,
        "status": "skip",
        "selected_backend": "",
        "fallback_used": False,
        "direct_goal_fallback_used": False,
        "latency_ms": 0.0,
        "path_length_m": 0.0,
        "waypoint_count": 0,
        "path_safety_score": 0.0,
        "map_artifact_gate_ok": False,
        "platform_assumption": PLATFORM_ASSUMPTION,
        **benchmark_claim_metadata(generated_at=generated_at),
        "error": error,
    }
    report = {
        "schema_version": SCHEMA_VERSION,
        **benchmark_claim_metadata(generated_at=generated_at),
        "planners": planners,
        "routes": routes,
        "results": [result],
        "status_counts": {"pass": 0, "skip": 1, "fail": 0},
    }
    json_out.parent.mkdir(parents=True, exist_ok=True)
    json_out.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


def main() -> int:
    args = build_parser().parse_args()
    generated_at = time.time()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    json_out = args.json_out or output_dir / "report.json"

    planner_names = [
        p.strip().lower()
        for p in args.planners.split(",")
        if p.strip()
    ]
    route_names = [r.strip() for r in args.routes.split(",") if r.strip()]

    import_error = _ensure_benchmark_dependencies()
    if import_error is not None:
        return write_import_skip_report(
            json_out=json_out,
            planners=planner_names,
            routes=route_names,
            error=import_error,
        )

    # ---- Build synthetic map fixture ----
    assets = _build_fixture(output_dir)
    fixture_dir = output_dir / "fixture"

    # ---- Parse planners and routes ----
    routes = _resolve_routes(assets, route_names)

    if not routes:
        print("Error: no valid routes specified", file=sys.stderr)
        return 1

    # ---- Run benchmarks ----
    results: list[dict[str, Any]] = []
    for planner in planner_names:
        for route in routes:
            result = benchmark_planner(
                planner,
                fixture_dir,
                tuple(route.start),
                tuple(route.goal),
            )
            result["route"] = route.name
            result["route_description"] = route.description
            results.append(result)

    # ---- Build report ----
    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        **benchmark_claim_metadata(generated_at=generated_at),
        "planners": planner_names,
        "routes": route_names,
        "results": results,
        "status_counts": {
            "pass": sum(1 for item in results if item.get("status") == "pass"),
            "skip": sum(1 for item in results if item.get("status") == "skip"),
            "fail": sum(1 for item in results if item.get("status") == "fail"),
        },
    }

    # ---- Write output ----
    json_out.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    # Also print to stdout for pipe-friendly usage
    print(json.dumps(report, ensure_ascii=False, indent=2))

    # ---- Cleanup ----
    if not args.keep_fixture and fixture_dir.is_dir():
        import shutil

        shutil.rmtree(fixture_dir)

    # Exit non-zero only for failures. Skips are explicit non-passing results
    # for optional unavailable backends.
    for r in results:
        if r.get("status") == "fail":
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
