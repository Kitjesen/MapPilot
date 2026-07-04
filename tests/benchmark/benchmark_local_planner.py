#!/usr/bin/env python3
"""Benchmark LocalPlanner Python fallback vs native C++ backend.

Usage:
    PYTHONPATH=src python tests/benchmark/benchmark_local_planner.py
    PYTHONPATH=src python tests/benchmark/benchmark_local_planner.py --json-out artifacts/benchmark_local_planner.json
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Callable

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SRC = _REPO_ROOT / "src"
_NB_WIN = _SRC / "nav" / "kernel" / "build_nb_win"
for _p in (str(_REPO_ROOT), str(_SRC), str(_NB_WIN)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np

from nav.services.plan.local_planner.cmu_py import plan_cmu_py_local_path
from nav.services.plan.local_planner.models import CmuPyLocalPlannerRequest
from nav.services.plan.local_planner.path_tables import (
    load_cmu_py_paths,
    local_planner_paths_dir,
)


def _bench(fn: Callable[[], Any], *, reps: int, warmup: int) -> dict[str, float]:
    for _ in range(warmup):
        fn()
    vals: list[float] = []
    for _ in range(reps):
        start = time.perf_counter_ns()
        fn()
        vals.append((time.perf_counter_ns() - start) / 1_000_000.0)
    vals.sort()
    p95_idx = max(0, min(len(vals) - 1, int(len(vals) * 0.95) - 1))
    return {
        "median_ms": round(statistics.median(vals), 3),
        "p95_ms": round(vals[p95_idx], 3),
        "min_ms": round(vals[0], 3),
        "max_ms": round(vals[-1], 3),
    }


def _sample_points(rng: np.random.Generator, n: int) -> np.ndarray:
    return np.stack(
        [
            rng.uniform(-0.5, 4.0, n).astype(np.float32),
            rng.uniform(-2.5, 2.5, n).astype(np.float32),
            rng.uniform(-0.05, 0.45, n).astype(np.float32),
            rng.uniform(0.0, 0.5, n).astype(np.float32),
        ],
        axis=1,
    ).astype(np.float32)


def run() -> dict[str, Any]:
    try:
        import lingtu_nav_kernel as nav_kernel
    except Exception as exc:
        return {
            "schema_version": "lingtu.local_planner_benchmark.v1",
            "ok": False,
            "error": f"native kernel unavailable: {exc}",
        }

    paths_dir = local_planner_paths_dir()
    path_data = load_cmu_py_paths(paths_dir)
    core = nav_kernel.LocalPlanner(nav_kernel.LocalPlannerParams())
    if not core.load_paths(paths_dir):
        return {
            "schema_version": "lingtu.local_planner_benchmark.v1",
            "ok": False,
            "error": f"failed to load local planner paths from {paths_dir}",
        }

    robot = np.array([0.0, 0.0, 0.0], dtype=float)
    goal = np.array([3.0, 0.0, 0.0], dtype=float)
    rng = np.random.default_rng(7)
    rows = []
    for n, py_reps in ((500, 8), (2000, 5), (5000, 3)):
        points = _sample_points(rng, n)
        obstacle_flat = np.ascontiguousarray(points, dtype=np.float32).ravel()
        request = CmuPyLocalPlannerRequest(
            path_data=path_data,
            robot_pos=robot,
            robot_yaw=0.0,
            goal=goal,
            obstacle_points_world=points,
            use_traversability_cost=False,
        )
        py = _bench(
            lambda: plan_cmu_py_local_path(request),
            reps=py_reps,
            warmup=1,
        )
        cpp = _bench(
            lambda: core.plan_frame_without_grid(
                0.0,
                0.0,
                0.0,
                0.0,
                3.0,
                0.0,
                obstacle_flat,
                time.time(),
            ),
            reps=50,
            warmup=5,
        )
        rows.append(
            {
                "obstacle_points": n,
                "cmu_py": py,
                "cpp_nanobind": cpp,
                "median_speedup_x": round(py["median_ms"] / cpp["median_ms"], 1),
            }
        )

    return {
        "schema_version": "lingtu.local_planner_benchmark.v1",
        "ok": True,
        "platform": sys.platform,
        "python": sys.version.split()[0],
        "paths_dir": paths_dir,
        "rows": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()

    report = run()
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
