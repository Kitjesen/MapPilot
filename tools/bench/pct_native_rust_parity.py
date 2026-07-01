#!/usr/bin/env python3
"""Compare PCT native preview output with the Rust process runtime.

This is the Linux/native parity gate for the PCT optimizer migration work. It
runs ``nav.services.plan.global_planner.algorithm.pct.runtime.preview`` twice in isolated subprocesses with the same tomogram
and start/goal: once through the current native runtime, once through
``rust_process``. The output is machine-readable so release gates can require
real same-input parity evidence instead of relying on Windows-only smokes.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
RUST_CRATE = SRC / "kernels" / "planning" / "gpmp_trajectory_optimizer"
DEFAULT_TOMOGRAM = (
    SRC
    / "nav"
    / "planning"
    / "vendor"
    / "pct_planner"
    / "rsc"
    / "tomogram"
    / "building2_9.pickle"
)
SCHEMA = "lingtu.pct.native_rust_parity.v1"


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


def resolve_optimizer_binary(path: Path | None, *, build: bool) -> Path | None:
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
    if build or binary.is_file():
        if build or not binary.is_file():
            binary = build_optimizer()
        return binary.resolve()
    return None


def run_preview_runtime(
    *,
    runtime: str,
    tomogram: Path,
    start: list[float],
    goal: list[float],
    obstacle_thr: float,
    sample_count: int,
    optimizer_bin: Path | None,
) -> dict[str, Any]:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(SRC) + os.pathsep + env.get("PYTHONPATH", "")
    env["LINGTU_PCT_PLANNER_RUNTIME"] = runtime
    if optimizer_bin is not None:
        env["LINGTU_GPMP_OPTIMIZER_BIN"] = str(optimizer_bin)
    command = [
        sys.executable,
        "-m",
        "nav.services.plan.global_planner.algorithm.pct.runtime.preview",
        "--tomogram",
        str(tomogram),
        "--start",
        *(str(value) for value in start),
        "--goal",
        *(str(value) for value in goal),
        "--obstacle-thr",
        str(obstacle_thr),
        "--sample-count",
        str(sample_count),
        "--repo-root",
        str(ROOT),
        "--json",
    ]
    completed = subprocess.run(
        command,
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    payload = parse_preview_stdout(completed.stdout)
    payload.setdefault("schema", "lingtu.pct.preview.actual.v2")
    payload["subprocess"] = {
        "runtime": runtime,
        "returncode": completed.returncode,
        "stderr_tail": completed.stderr[-8000:],
    }
    if completed.returncode != 0:
        payload["ok"] = False
        payload.setdefault("error", f"{runtime} preview failed with code {completed.returncode}")
    return payload


def parse_preview_stdout(stdout: str) -> dict[str, Any]:
    try:
        data = json.loads(stdout)
        return data if isinstance(data, dict) else {"ok": False, "error": "preview emitted non-object JSON"}
    except json.JSONDecodeError:
        start = stdout.find("{")
        end = stdout.rfind("}")
        if start >= 0 and end > start:
            try:
                data = json.loads(stdout[start : end + 1])
                return data if isinstance(data, dict) else {"ok": False, "error": "preview emitted non-object JSON"}
            except json.JSONDecodeError:
                pass
        return {
            "ok": False,
            "error": "preview emitted invalid JSON",
            "stdout_tail": stdout[-8000:],
        }


def _num(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _vec(value: Any) -> list[float] | None:
    if not isinstance(value, list):
        return None
    out: list[float] = []
    for item in value:
        number = _num(item)
        if number is None:
            return None
        out.append(number)
    return out


def _metric(actual: dict[str, Any], field: str) -> Any:
    if field in actual:
        return actual[field]
    path = actual.get("path") if isinstance(actual.get("path"), dict) else {}
    mapping = {
        "path_count": "count",
        "path_distance_m": "distance_m",
        "goal_error_m": "goal_error_m",
        "start_error_m": "start_error_m",
    }
    nested = mapping.get(field)
    return path.get(nested) if nested else None


def _fraction_samples(actual: dict[str, Any]) -> dict[float, list[float]]:
    path = actual.get("path") if isinstance(actual.get("path"), dict) else {}
    samples = path.get("samples") if isinstance(path.get("samples"), dict) else {}
    values = samples.get("by_arclength_fraction")
    if not isinstance(values, list):
        return {}
    out: dict[float, list[float]] = {}
    for sample in values:
        if not isinstance(sample, dict):
            continue
        fraction = _num(sample.get("fraction"))
        point = _vec(sample.get("point"))
        if fraction is not None and point is not None:
            out[round(fraction, 6)] = point
    return out


def _input_value(actual: dict[str, Any], field: str) -> Any:
    input_spec = actual.get("input") if isinstance(actual.get("input"), dict) else {}
    return input_spec.get(field)


def _optimizer_accessors(actual: dict[str, Any]) -> dict[str, Any]:
    accessors = actual.get("optimizer_accessors")
    return accessors if isinstance(accessors, dict) else {}


def _shape(value: Any) -> list[int] | None:
    if not isinstance(value, list):
        return None
    shape: list[int] = []
    for item in value:
        number = _num(item)
        if number is None or int(number) != number:
            return None
        shape.append(int(number))
    return shape


def _check_optimizer_flag(
    runtime: str,
    accessors: dict[str, Any],
    field: str,
) -> dict[str, Any]:
    value = accessors.get(field)
    return {
        "name": f"optimizer_accessors:{runtime}_{field}",
        "passed": value is True,
        runtime: value,
    }


def _check_optimizer_scalar_equal(
    field: str,
    native_accessors: dict[str, Any],
    rust_accessors: dict[str, Any],
) -> dict[str, Any]:
    native = native_accessors.get(field)
    rust = rust_accessors.get(field)
    return {
        "name": f"optimizer_accessors:{field}",
        "passed": native == rust and native is not None,
        "native": native,
        "rust_process": rust,
    }


def _check_optimizer_shape_equal(
    field: str,
    native_accessors: dict[str, Any],
    rust_accessors: dict[str, Any],
) -> dict[str, Any]:
    native = _shape(native_accessors.get(field))
    rust = _shape(rust_accessors.get(field))
    return {
        "name": f"optimizer_accessors:{field}",
        "passed": native is not None and native == rust,
        "native": native_accessors.get(field),
        "rust_process": rust_accessors.get(field),
    }


def _check_delta(
    name: str,
    native_value: Any,
    rust_value: Any,
    tolerance: float,
) -> dict[str, Any]:
    native = _num(native_value)
    rust = _num(rust_value)
    if native is None or rust is None:
        return {
            "name": name,
            "passed": False,
            "native": native_value,
            "rust_process": rust_value,
            "reason": "non_finite_number",
        }
    delta = abs(native - rust)
    return {
        "name": name,
        "passed": delta <= tolerance,
        "native": native,
        "rust_process": rust,
        "abs_delta": delta,
        "abs_tol": tolerance,
    }


def _check_vector_delta(
    name: str,
    native_value: Any,
    rust_value: Any,
    tolerance: float,
) -> dict[str, Any]:
    native = _vec(native_value)
    rust = _vec(rust_value)
    if native is None or rust is None or len(native) != len(rust):
        return {
            "name": name,
            "passed": False,
            "native": native_value,
            "rust_process": rust_value,
            "reason": "shape_or_number_mismatch",
        }
    max_delta = max((abs(lhs - rhs) for lhs, rhs in zip(native, rust)), default=0.0)
    return {
        "name": name,
        "passed": max_delta <= tolerance,
        "native": native,
        "rust_process": rust,
        "max_abs_delta": max_delta,
        "abs_tol": tolerance,
    }


def compare_actuals(
    native: dict[str, Any],
    rust: dict[str, Any],
    *,
    path_count_delta_max: int,
    path_distance_abs_tol: float,
    goal_error_abs_tol: float,
    sample_abs_tol: float,
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    checks.append(
        {
            "name": "native_ok",
            "passed": bool(native.get("ok")),
            "native": native.get("ok"),
        }
    )
    checks.append(
        {
            "name": "rust_process_ok",
            "passed": bool(rust.get("ok")),
            "rust_process": rust.get("ok"),
        }
    )
    for field in ("start", "goal", "obstacle_thr", "tomogram_sha256"):
        checks.append(
            {
                "name": f"input:{field}",
                "passed": _input_value(native, field) == _input_value(rust, field),
                "native": _input_value(native, field),
                "rust_process": _input_value(rust, field),
            }
        )

    native_count = _num(_metric(native, "path_count"))
    rust_count = _num(_metric(rust, "path_count"))
    checks.append(
        {
            "name": "path_count_delta",
            "passed": (
                native_count is not None
                and rust_count is not None
                and abs(native_count - rust_count) <= path_count_delta_max
            ),
            "native": native_count,
            "rust_process": rust_count,
            "abs_delta": (
                abs(native_count - rust_count)
                if native_count is not None and rust_count is not None
                else None
            ),
            "max_delta": path_count_delta_max,
        }
    )
    checks.append(
        _check_delta(
            "path_distance_m",
            _metric(native, "path_distance_m"),
            _metric(rust, "path_distance_m"),
            path_distance_abs_tol,
        )
    )
    checks.append(
        _check_delta(
            "goal_error_m",
            _metric(native, "goal_error_m"),
            _metric(rust, "goal_error_m"),
            goal_error_abs_tol,
        )
    )
    checks.append(_check_vector_delta("first", native.get("first"), rust.get("first"), sample_abs_tol))
    checks.append(_check_vector_delta("last", native.get("last"), rust.get("last"), sample_abs_tol))

    native_samples = _fraction_samples(native)
    rust_samples = _fraction_samples(rust)
    for fraction in sorted(set(native_samples) & set(rust_samples)):
        checks.append(
            _check_vector_delta(
                f"arclength_sample:{fraction}",
                native_samples[fraction],
                rust_samples[fraction],
                sample_abs_tol,
            )
        )

    native_accessors = _optimizer_accessors(native)
    rust_accessors = _optimizer_accessors(rust)
    for runtime, accessors in (
        ("native", native_accessors),
        ("rust_process", rust_accessors),
    ):
        for field in ("available", "native_wrapper_compatible", "finite"):
            checks.append(_check_optimizer_flag(runtime, accessors, field))
    for field in ("row_count", "state_dim"):
        checks.append(_check_optimizer_scalar_equal(field, native_accessors, rust_accessors))
    for field in (
        "result_matrix_shape",
        "layers_shape",
        "heights_shape",
        "ceilings_shape",
        "opt_init_value_shape",
        "opt_init_layer_shape",
        "heading_rate_shape",
    ):
        checks.append(_check_optimizer_shape_equal(field, native_accessors, rust_accessors))

    failed = [check["name"] for check in checks if not check.get("passed")]
    return {
        "verdict": "pass" if not failed else "fail",
        "failed_checks": failed,
        "checks": checks,
        "tolerances": {
            "path_count_delta_max": path_count_delta_max,
            "path_distance_abs_tol": path_distance_abs_tol,
            "goal_error_abs_tol": goal_error_abs_tol,
            "sample_abs_tol": sample_abs_tol,
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tomogram", type=Path, default=DEFAULT_TOMOGRAM)
    parser.add_argument("--start", nargs=3, type=float, default=[2.0, 3.0, 0.0])
    parser.add_argument("--goal", nargs=3, type=float, default=[18.0, 11.0, 0.0])
    parser.add_argument("--obstacle-thr", type=float, default=49.9)
    parser.add_argument("--sample-count", type=int, default=9)
    parser.add_argument("--optimizer-bin", type=Path, default=None)
    parser.add_argument("--build", action="store_true", help="build gpmp_optimize if needed")
    parser.add_argument("--path-count-delta-max", type=int, default=8)
    parser.add_argument("--path-distance-abs-tol", type=float, default=2.0)
    parser.add_argument("--goal-error-abs-tol", type=float, default=1.0)
    parser.add_argument("--sample-abs-tol", type=float, default=3.0)
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--json", action="store_true", help="print JSON only")
    parser.add_argument("--enforce", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    optimizer_bin = resolve_optimizer_binary(args.optimizer_bin, build=args.build)
    native = run_preview_runtime(
        runtime="native",
        tomogram=args.tomogram,
        start=list(args.start),
        goal=list(args.goal),
        obstacle_thr=float(args.obstacle_thr),
        sample_count=int(args.sample_count),
        optimizer_bin=None,
    )
    rust = run_preview_runtime(
        runtime="rust_process",
        tomogram=args.tomogram,
        start=list(args.start),
        goal=list(args.goal),
        obstacle_thr=float(args.obstacle_thr),
        sample_count=int(args.sample_count),
        optimizer_bin=optimizer_bin,
    )
    comparison = compare_actuals(
        native,
        rust,
        path_count_delta_max=int(args.path_count_delta_max),
        path_distance_abs_tol=float(args.path_distance_abs_tol),
        goal_error_abs_tol=float(args.goal_error_abs_tol),
        sample_abs_tol=float(args.sample_abs_tol),
    )
    payload = {
        "schema": SCHEMA,
        "tomogram": str(args.tomogram),
        "start": list(args.start),
        "goal": list(args.goal),
        "native": native,
        "rust_process": rust,
        "summary": comparison,
    }
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(f"PCT native/rust parity: {comparison['verdict']}")
        print(f"  native_ok: {native.get('ok')}")
        print(f"  rust_process_ok: {rust.get('ok')}")
        print(f"  failed_checks: {', '.join(comparison['failed_checks']) or 'none'}")

    if args.enforce and comparison["verdict"] != "pass":
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
