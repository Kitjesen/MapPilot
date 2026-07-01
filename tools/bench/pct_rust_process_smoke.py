#!/usr/bin/env python3
"""Run a Windows-safe PCT rust_process preview smoke.

The script builds a tiny synthetic tomogram, forces the PCT runtime to use the
Rust GPMP process runtime, and emits the same preview JSON shape as
``nav.services.plan.global_planner.algorithm.pct.runtime.preview``. It can also compare the output against a golden fixture.
"""

from __future__ import annotations

import argparse
import json
import os
import pickle
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
RUST_CRATE = SRC / "kernels" / "planning" / "gpmp_trajectory_optimizer"
DEFAULT_LM_GOLDEN = (
    SRC
    / "nav"
    / "tests"
    / "planning_backends"
    / "fixtures"
    / "pct_preview"
    / "rust_process_synthetic_smoke.json"
)
DEFAULT_GN_GOLDEN = (
    SRC
    / "nav"
    / "tests"
    / "planning_backends"
    / "fixtures"
    / "pct_preview"
    / "rust_process_synthetic_gn_smoke.json"
)
DEFAULT_GOLDENS_BY_OPTIMIZER = {
    "levenberg_marquardt": DEFAULT_LM_GOLDEN,
    "gauss_newton": DEFAULT_GN_GOLDEN,
}


def _optimizer_binary_name() -> str:
    return "gpmp_optimize.exe" if os.name == "nt" else "gpmp_optimize"


def _default_optimizer_binary() -> Path:
    return RUST_CRATE / "target" / "release" / _optimizer_binary_name()


def build_optimizer() -> Path:
    subprocess.run(
        [
            "cargo",
            "build",
            "--release",
            "--manifest-path",
            str(RUST_CRATE / "Cargo.toml"),
        ],
        cwd=ROOT,
        check=True,
    )
    return _default_optimizer_binary()


def resolve_optimizer_binary(path: Path | None, *, build: bool) -> Path:
    if path is not None:
        resolved = path.resolve()
        if not resolved.is_file():
            raise FileNotFoundError(f"Rust GPMP optimizer binary does not exist: {resolved}")
        return resolved

    env_value = os.environ.get("LINGTU_GPMP_OPTIMIZER_BIN")
    if env_value:
        resolved = Path(env_value).resolve()
        if not resolved.is_file():
            raise FileNotFoundError(f"LINGTU_GPMP_OPTIMIZER_BIN does not exist: {resolved}")
        return resolved

    binary = _default_optimizer_binary()
    if build or not binary.is_file():
        binary = build_optimizer()
    if not binary.is_file():
        raise FileNotFoundError(
            f"Rust GPMP optimizer binary does not exist: {binary}. "
            "Pass --build or --optimizer-bin."
        )
    return binary.resolve()


def default_golden_for_optimizer(nonlinear_optimizer: str) -> Path:
    return DEFAULT_GOLDENS_BY_OPTIMIZER.get(
        nonlinear_optimizer,
        DEFAULT_LM_GOLDEN,
    )


def write_synthetic_tomogram(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = np.zeros((5, 1, 13, 15), dtype=np.float32)
    data[0, :, :, :] = 1.0
    data[3, :, :, :] = 0.0
    data[4, :, :, :] = 3.0
    with path.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 1.0,
                "center": [0.0, 0.0],
                "slice_h0": 0.0,
                "slice_dh": 1.0,
                "grid_info": {"axis_order": "row_y_col_x"},
            },
            handle,
            protocol=pickle.HIGHEST_PROTOCOL,
        )
    return path


def run_preview(
    *,
    optimizer_bin: Path,
    work_dir: Path,
    linear_solver: str,
    nonlinear_optimizer: str,
) -> dict[str, Any]:
    sys.path.insert(0, str(SRC))
    from nav.services.plan.global_planner.algorithm.pct.runtime.api import load_pct_planner_runtime
    from nav.services.plan.global_planner.algorithm.pct.runtime.preview import build_preview_report

    tomogram = write_synthetic_tomogram(work_dir / "tomogram.pickle")
    os.environ["LINGTU_PCT_PLANNER_RUNTIME"] = "rust_process"
    os.environ["LINGTU_GPMP_OPTIMIZER_BIN"] = str(optimizer_bin)
    os.environ["LINGTU_PCT_RUST_LINEAR_SOLVER"] = linear_solver
    os.environ["LINGTU_PCT_RUST_NONLINEAR_OPTIMIZER"] = nonlinear_optimizer

    runtime = load_pct_planner_runtime(tomogram, repo_root=ROOT)
    planner = runtime.planner
    start = np.asarray([-5.0, 0.0, 0.0], dtype=np.float64)
    goal = np.asarray([5.0, 3.0, 0.0], dtype=np.float64)
    result = planner.plan(start[:2], goal[:2], float(start[2]), float(goal[2]))
    return build_preview_report(
        planner=planner,
        runtime_paths=runtime.runtime_paths,
        result=result,
        start=start,
        goal=goal,
        tomogram_path=tomogram,
        obstacle_thr=49.9,
        sample_count=3,
    )


def compare_with_golden(golden: Path, actual: dict[str, Any]) -> dict[str, Any]:
    sys.path.insert(0, str(ROOT))
    from tools.bench.pct_preview_compare import compare

    with golden.open("r", encoding="utf-8-sig") as handle:
        golden_payload = json.load(handle)
    return compare(golden_payload, actual)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--optimizer-bin", type=Path, default=None)
    parser.add_argument("--build", action="store_true", help="build gpmp_optimize if needed")
    parser.add_argument("--linear-solver", default="block_tridiagonal")
    parser.add_argument(
        "--nonlinear-optimizer",
        choices=sorted(DEFAULT_GOLDENS_BY_OPTIMIZER),
        default="levenberg_marquardt",
    )
    parser.add_argument("--work-dir", type=Path, default=None)
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument(
        "--golden",
        type=Path,
        default=None,
        help="golden fixture; defaults to the fixture for --nonlinear-optimizer",
    )
    parser.add_argument("--compare", action="store_true")
    parser.add_argument("--enforce", action="store_true")
    parser.add_argument("--json", action="store_true", help="print JSON only")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    optimizer_bin = resolve_optimizer_binary(args.optimizer_bin, build=args.build)
    golden = args.golden or default_golden_for_optimizer(str(args.nonlinear_optimizer))
    cleanup = args.work_dir is None
    if args.work_dir is None:
        temp_dir = tempfile.TemporaryDirectory(prefix="lingtu_pct_rust_process_")
        work_dir = Path(temp_dir.name)
    else:
        temp_dir = None
        work_dir = args.work_dir
        work_dir.mkdir(parents=True, exist_ok=True)

    try:
        actual = run_preview(
            optimizer_bin=optimizer_bin,
            work_dir=work_dir,
            linear_solver=str(args.linear_solver),
            nonlinear_optimizer=str(args.nonlinear_optimizer),
        )
        if args.json_out is not None:
            args.json_out.parent.mkdir(parents=True, exist_ok=True)
            args.json_out.write_text(
                json.dumps(actual, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )

        comparison = None
        if args.compare or args.enforce:
            comparison = compare_with_golden(golden, actual)

        if args.json:
            payload: dict[str, Any] = {"actual": actual}
            if comparison is not None:
                payload["comparison"] = comparison
            print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
        else:
            print(f"PCT rust_process smoke: {'OK' if actual.get('ok') else 'FAILED'}")
            diagnostics = actual.get("diagnostics") or {}
            print(f"  path_count: {actual.get('path_count')}")
            print(f"  goal_error_m: {actual.get('goal_error_m')}")
            print(f"  linear_solver: {diagnostics.get('last_optimizer_linear_solver')}")
            print(
                "  nonlinear_optimizer: "
                f"{diagnostics.get('last_optimizer_nonlinear_optimizer')}"
            )
            print(f"  fallbacks: {diagnostics.get('last_optimizer_linear_solve_fallbacks')}")
            if comparison is not None:
                print(f"  golden: {comparison['verdict']}")

        if args.enforce:
            if not actual.get("ok"):
                return 1
            if comparison is not None and comparison.get("verdict") != "pass":
                return 1
        return 0
    finally:
        if cleanup and temp_dir is not None:
            temp_dir.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
