#!/usr/bin/env python3
"""Compare Rust GPMP dense and block-tridiagonal optimizer paths.

The benchmark is Windows-safe and talks to the same ``gpmp_optimize`` process
binary used by the PCT ``rust_process`` runtime. It is intentionally scoped to
chain-structured WNOJ/WNOA trajectory graphs: that is the structure where the
block-tridiagonal solver is expected to improve runtime over the dense fallback.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
RUST_CRATE = ROOT / "src" / "kernels" / "planning" / "gpmp_trajectory_optimizer"
SCHEMA = "lingtu.pct_gpmp_optimizer_compare.v1"
DEFAULT_NONLINEAR_OPTIMIZERS = ("levenberg_marquardt", "gauss_newton")


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
            "--bin",
            "gpmp_optimize",
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


def build_request(
    *,
    mode: str,
    state_count: int,
    linear_solver: str,
    nonlinear_optimizer: str,
    max_iterations: int,
) -> dict[str, Any]:
    if state_count < 2:
        raise ValueError("state_count must be at least 2")
    if mode not in {"wnoj", "wnoa"}:
        raise ValueError(f"unsupported mode: {mode}")

    states: list[list[float]] = []
    layers: list[int] = []
    heights: list[float] = []
    for index in range(state_count):
        t = index / (state_count - 1)
        x = 50.0 * t
        y = 6.0 * math.sin(t * math.pi * 1.5) + 2.0 * t
        dx = 50.0 / (state_count - 1)
        dy = (
            6.0 * math.pi * 1.5 * math.cos(t * math.pi * 1.5) / (state_count - 1)
            + 2.0 / (state_count - 1)
        )
        if mode == "wnoj":
            ddy = -6.0 * (math.pi * 1.5) ** 2 * math.sin(t * math.pi * 1.5)
            ddy /= (state_count - 1) ** 2
            states.append([x, dx, 0.0, y, dy, ddy])
            endpoint_prior_sigmas = [0.05, 0.5, 2.0, 0.05, 0.5, 2.0]
        else:
            states.append([x, dx, y, dy])
            endpoint_prior_sigmas = [0.05, 0.5, 0.05, 0.5]
        layers.append(0)
        heights.append(0.0)

    return {
        "schema": "lingtu.pct_gpmp.optimize.request.v1",
        "mode": mode,
        "states": states,
        "layers": layers,
        "height_hints": heights,
        "endpoint_prior_sigmas": endpoint_prior_sigmas,
        "gp_qc": 10.0,
        "delta": 1.0,
        "heading_rate_sigma": 1.0,
        "max_heading_rate": 10.0,
        "interpolation_steps": 1,
        "config": {
            "max_iterations": max_iterations,
            "initial_lambda": 200.0,
            "lambda_up": 10.0,
            "lambda_down": 0.3,
            "gradient_tolerance": 1e-8,
            "step_tolerance": 1e-8,
            "cost_tolerance": 1e-9,
            "linear_solver": linear_solver,
            "nonlinear_optimizer": nonlinear_optimizer,
        },
    }


def run_optimizer(optimizer_bin: Path, request: dict[str, Any]) -> tuple[dict[str, Any], float]:
    start = time.perf_counter()
    completed = subprocess.run(
        [str(optimizer_bin)],
        input=json.dumps(request, separators=(",", ":")),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=ROOT,
        check=False,
    )
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    if completed.returncode != 0:
        raise RuntimeError(
            "gpmp_optimize failed with "
            f"code {completed.returncode}: {completed.stderr or completed.stdout}"
        )
    try:
        response = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"gpmp_optimize emitted invalid JSON: {completed.stdout!r}") from exc
    if not response.get("ok"):
        raise RuntimeError(f"gpmp_optimize rejected request: {response.get('error')}")
    return response, elapsed_ms


def run_case(
    *,
    optimizer_bin: Path,
    mode: str,
    state_count: int,
    nonlinear_optimizer: str,
    max_iterations: int,
    repeats: int,
) -> dict[str, Any]:
    runs: dict[str, list[dict[str, Any]]] = {"dense": [], "block_tridiagonal": []}
    for linear_solver in ("dense", "block_tridiagonal"):
        request = build_request(
            mode=mode,
            state_count=state_count,
            linear_solver=linear_solver,
            nonlinear_optimizer=nonlinear_optimizer,
            max_iterations=max_iterations,
        )
        for _ in range(repeats):
            response, elapsed_ms = run_optimizer(optimizer_bin, request)
            report = response["report"]
            optimizer_elapsed_ms = float(report.get("elapsed_ms", elapsed_ms))
            runs[linear_solver].append(
                {
                    "elapsed_ms": optimizer_elapsed_ms,
                    "process_elapsed_ms": elapsed_ms,
                    "initial_cost": float(report["initial_cost"]),
                    "final_cost": float(report["final_cost"]),
                    "iterations": int(report["iterations"]),
                    "accepted_steps": int(report["accepted_steps"]),
                    "reported_linear_solver": str(report["linear_solver"]),
                    "reported_nonlinear_optimizer": str(report["nonlinear_optimizer"]),
                    "linear_solve_fallbacks": int(report["linear_solve_fallbacks"]),
                    "state_count": len(response.get("states") or []),
                }
            )

    dense = summarize_runs(runs["dense"])
    block = summarize_runs(runs["block_tridiagonal"])
    return {
        "mode": mode,
        "state_count": state_count,
        "nonlinear_optimizer": nonlinear_optimizer,
        "max_iterations": max_iterations,
        "repeats": repeats,
        "dense": dense,
        "block_tridiagonal": block,
        "speedup": safe_ratio(dense["median_elapsed_ms"], block["median_elapsed_ms"]),
        "final_cost_delta_abs": abs(dense["final_cost"] - block["final_cost"]),
        "accepted_step_delta_abs": abs(dense["accepted_steps"] - block["accepted_steps"]),
    }


def summarize_runs(runs: list[dict[str, Any]]) -> dict[str, Any]:
    if not runs:
        raise ValueError("no runs to summarize")
    elapsed = [float(run["elapsed_ms"]) for run in runs]
    representative = min(runs, key=lambda run: float(run["elapsed_ms"]))
    return {
        "median_elapsed_ms": statistics.median(elapsed),
        "min_elapsed_ms": min(elapsed),
        "max_elapsed_ms": max(elapsed),
        "final_cost": representative["final_cost"],
        "initial_cost": representative["initial_cost"],
        "iterations": representative["iterations"],
        "accepted_steps": representative["accepted_steps"],
        "reported_linear_solver": representative["reported_linear_solver"],
        "reported_nonlinear_optimizer": representative["reported_nonlinear_optimizer"],
        "linear_solve_fallbacks": representative["linear_solve_fallbacks"],
        "state_count": representative["state_count"],
    }


def safe_ratio(numerator: float, denominator: float) -> float | None:
    if denominator <= 0.0:
        return None
    return numerator / denominator


def evaluate(
    payload: dict[str, Any],
    *,
    min_speedup: float,
    final_cost_abs_tol: float,
    required_nonlinear_optimizers: tuple[str, ...] = DEFAULT_NONLINEAR_OPTIMIZERS,
) -> dict[str, Any]:
    failed: list[str] = []
    cases = payload.get("cases", [])
    cases = cases if isinstance(cases, list) else []
    seen_pairs = {
        (str(case.get("mode")), str(case.get("nonlinear_optimizer")))
        for case in cases
        if isinstance(case, dict)
    }
    for required_mode in ("wnoj", "wnoa"):
        for required_optimizer in required_nonlinear_optimizers:
            if (required_mode, required_optimizer) not in seen_pairs:
                failed.append(f"{required_mode}:{required_optimizer}:missing_case")
    for case in cases:
        if not isinstance(case, dict):
            failed.append("case:not_object")
            continue
        nonlinear_optimizer = str(case.get("nonlinear_optimizer"))
        label = f"{case.get('mode')}:{nonlinear_optimizer}:{case.get('state_count')}"
        block = case.get("block_tridiagonal") if isinstance(case.get("block_tridiagonal"), dict) else {}
        dense = case.get("dense") if isinstance(case.get("dense"), dict) else {}
        if not dense:
            failed.append(f"{label}:dense_missing")
        if not block:
            failed.append(f"{label}:block_tridiagonal_missing")
        if dense.get("reported_linear_solver") != "dense":
            failed.append(f"{label}:dense_solver_not_reported")
        if block.get("reported_linear_solver") != "block_tridiagonal":
            failed.append(f"{label}:block_solver_not_reported")
        if dense.get("reported_nonlinear_optimizer") != nonlinear_optimizer:
            failed.append(f"{label}:dense_nonlinear_optimizer_not_reported")
        if block.get("reported_nonlinear_optimizer") != nonlinear_optimizer:
            failed.append(f"{label}:block_nonlinear_optimizer_not_reported")
        if int(block.get("linear_solve_fallbacks") or 0) != 0:
            failed.append(f"{label}:block_solver_fallbacks")
        final_cost_delta = case.get("final_cost_delta_abs")
        if not isinstance(final_cost_delta, (int, float)) or final_cost_delta > final_cost_abs_tol:
            failed.append(f"{label}:final_cost_delta")
        speedup = case.get("speedup")
        if speedup is None or speedup < min_speedup:
            failed.append(f"{label}:speedup_below_threshold")
    return {
        "verdict": "pass" if not failed else "fail",
        "failed_checks": failed,
        "min_speedup": min_speedup,
        "final_cost_abs_tol": final_cost_abs_tol,
        "required_nonlinear_optimizers": list(required_nonlinear_optimizers),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--optimizer-bin", type=Path, default=None)
    parser.add_argument("--build", action="store_true", help="build gpmp_optimize if needed")
    parser.add_argument("--modes", nargs="+", choices=["wnoj", "wnoa"], default=["wnoj", "wnoa"])
    parser.add_argument("--state-counts", nargs="+", type=int, default=[96])
    parser.add_argument(
        "--nonlinear-optimizer",
        choices=DEFAULT_NONLINEAR_OPTIMIZERS,
        default=None,
        help="run one nonlinear optimizer; kept for legacy scripts",
    )
    parser.add_argument(
        "--nonlinear-optimizers",
        nargs="+",
        choices=DEFAULT_NONLINEAR_OPTIMIZERS,
        default=None,
        help="nonlinear optimizers to benchmark; defaults to both LM and GN",
    )
    parser.add_argument("--max-iterations", type=int, default=8)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--min-speedup", type=float, default=1.25)
    parser.add_argument("--final-cost-abs-tol", type=float, default=1e-6)
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--json", action="store_true", help="print JSON only")
    parser.add_argument("--enforce", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    optimizer_bin = resolve_optimizer_binary(args.optimizer_bin, build=args.build)
    nonlinear_optimizers = tuple(
        args.nonlinear_optimizers
        or ([args.nonlinear_optimizer] if args.nonlinear_optimizer else DEFAULT_NONLINEAR_OPTIMIZERS)
    )
    cases = [
        run_case(
            optimizer_bin=optimizer_bin,
            mode=mode,
            state_count=state_count,
            nonlinear_optimizer=nonlinear_optimizer,
            max_iterations=int(args.max_iterations),
            repeats=max(1, int(args.repeats)),
        )
        for mode in args.modes
        for state_count in args.state_counts
        for nonlinear_optimizer in nonlinear_optimizers
    ]
    payload = {
        "schema": SCHEMA,
        "optimizer_bin": str(optimizer_bin),
        "required_nonlinear_optimizers": list(nonlinear_optimizers),
        "cases": cases,
    }
    payload["summary"] = evaluate(
        payload,
        min_speedup=float(args.min_speedup),
        final_cost_abs_tol=float(args.final_cost_abs_tol),
        required_nonlinear_optimizers=nonlinear_optimizers,
    )

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(f"PCT GPMP optimizer compare: {payload['summary']['verdict']}")
        for case in payload["cases"]:
            print(
                "  "
                f"{case['mode']} states={case['state_count']} "
                f"dense={case['dense']['median_elapsed_ms']:.3f}ms "
                f"block={case['block_tridiagonal']['median_elapsed_ms']:.3f}ms "
                f"speedup={case['speedup']:.2f} "
                f"cost_delta={case['final_cost_delta_abs']:.3e}"
            )
        if payload["summary"]["failed_checks"]:
            print("  failed_checks:")
            for check in payload["summary"]["failed_checks"]:
                print(f"    - {check}")

    if args.enforce and payload["summary"]["verdict"] != "pass":
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
