#!/usr/bin/env python3
"""Run the Windows-safe PCT Rust runtime acceptance gate.

This gate proves the Python PCT runtime calls the Rust optimizer through the
real planner path, then checks the sparse optimizer performance gate and feeds
the produced artifacts into the kernel migration status validator.
"""

from __future__ import annotations

import argparse
import contextlib
import importlib.util
import json
import os
import sys
import tempfile
from pathlib import Path
from typing import Any, Iterator


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "lingtu.pct_rust_runtime_acceptance.v1"
DEFAULT_NONLINEAR_OPTIMIZERS = ("levenberg_marquardt", "gauss_newton")
REQUIRED_STATUS_CLAIMS = (
    "pct_gpmp_obstacle_optimizer_kernel",
    "pct_gpmp_rust_process_runtime",
    "pct_rust_process_golden_parity",
    "pct_rust_process_gn_golden_parity",
    "pct_gpmp_optimizer_performance",
)
RUNTIME_ENV_KEYS = (
    "LINGTU_PCT_PLANNER_RUNTIME",
    "LINGTU_GPMP_OPTIMIZER_BIN",
    "LINGTU_PCT_RUST_LINEAR_SOLVER",
    "LINGTU_PCT_RUST_NONLINEAR_OPTIMIZER",
)


def _load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _load_smoke_module() -> Any:
    return _load_module(
        "pct_rust_process_smoke_acceptance",
        ROOT / "tools" / "bench" / "pct_rust_process_smoke.py",
    )


def _load_optimizer_compare_module() -> Any:
    return _load_module(
        "pct_gpmp_optimizer_compare_acceptance",
        ROOT / "tools" / "bench" / "pct_gpmp_optimizer_compare.py",
    )


def _load_migration_status_module() -> Any:
    return _load_module(
        "validate_kernel_migration_status_acceptance",
        ROOT / "tools" / "validate" / "validate_kernel_migration_status.py",
    )


def _repo_path(path: Path) -> str:
    try:
        return path.resolve().relative_to(ROOT.resolve()).as_posix()
    except ValueError:
        return str(path)


def _artifact_path(path_value: str) -> Path:
    path = Path(path_value)
    return path if path.is_absolute() else ROOT / path


@contextlib.contextmanager
def _preserve_env(keys: tuple[str, ...]) -> Iterator[None]:
    previous = {key: os.environ.get(key) for key in keys}
    try:
        yield
    finally:
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def run_runtime_smoke(
    *,
    optimizer_bin: Path,
    work_dir: Path,
    nonlinear_optimizer: str,
    linear_solver: str,
) -> dict[str, Any]:
    smoke = _load_smoke_module()
    actual_path = work_dir / f"pct_rust_process_{nonlinear_optimizer}.json"
    with _preserve_env(RUNTIME_ENV_KEYS):
        actual = smoke.run_preview(
            optimizer_bin=optimizer_bin,
            work_dir=work_dir / nonlinear_optimizer,
            linear_solver=linear_solver,
            nonlinear_optimizer=nonlinear_optimizer,
        )
    actual_path.write_text(json.dumps(actual, indent=2, sort_keys=True), encoding="utf-8")
    golden = smoke.default_golden_for_optimizer(nonlinear_optimizer)
    comparison = smoke.compare_with_golden(golden, actual)
    diagnostics = actual.get("diagnostics") if isinstance(actual, dict) else {}
    runtime = actual.get("runtime") if isinstance(actual, dict) else {}
    return {
        "nonlinear_optimizer": nonlinear_optimizer,
        "linear_solver": linear_solver,
        "actual_json": _repo_path(actual_path),
        "golden": _repo_path(golden),
        "actual": actual,
        "comparison": comparison,
        "summary": {
            "ok": bool(actual.get("ok")),
            "comparison": comparison.get("verdict"),
            "runtime": runtime.get("runtime") if isinstance(runtime, dict) else None,
            "call_mode": diagnostics.get("last_optimizer_call_mode")
            if isinstance(diagnostics, dict)
            else None,
            "path_count": actual.get("path_count"),
            "goal_error_m": actual.get("goal_error_m"),
            "fallbacks": diagnostics.get("last_optimizer_linear_solve_fallbacks")
            if isinstance(diagnostics, dict)
            else None,
        },
    }


def run_optimizer_compare(
    *,
    optimizer_bin: Path,
    work_dir: Path,
    state_count: int,
    repeats: int,
    max_iterations: int,
    min_speedup: float,
    final_cost_abs_tol: float,
) -> dict[str, Any]:
    module = _load_optimizer_compare_module()
    cases = [
        module.run_case(
            optimizer_bin=optimizer_bin,
            mode=mode,
            state_count=state_count,
            nonlinear_optimizer=nonlinear_optimizer,
            max_iterations=max_iterations,
            repeats=repeats,
        )
        for mode in ("wnoj", "wnoa")
        for nonlinear_optimizer in DEFAULT_NONLINEAR_OPTIMIZERS
    ]
    payload = {
        "schema": module.SCHEMA,
        "optimizer_bin": str(optimizer_bin),
        "required_nonlinear_optimizers": list(DEFAULT_NONLINEAR_OPTIMIZERS),
        "cases": cases,
    }
    payload["summary"] = module.evaluate(
        payload,
        min_speedup=min_speedup,
        final_cost_abs_tol=final_cost_abs_tol,
        required_nonlinear_optimizers=DEFAULT_NONLINEAR_OPTIMIZERS,
    )
    output_path = work_dir / "pct_gpmp_optimizer_compare.json"
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    return {
        "json": _repo_path(output_path),
        "payload": payload,
    }


def build_migration_status(
    *,
    runtime_smokes: dict[str, dict[str, Any]],
    optimizer_compare: dict[str, Any],
) -> dict[str, Any]:
    module = _load_migration_status_module()
    lm_actual = _artifact_path(runtime_smokes["levenberg_marquardt"]["actual_json"])
    gn_actual = _artifact_path(runtime_smokes["gauss_newton"]["actual_json"])
    compare_json = _artifact_path(optimizer_compare["json"])
    status = module.build_status(
        ROOT,
        pct_rust_process_actual_json=lm_actual,
        require_pct_rust_process_golden=True,
        pct_rust_process_gn_actual_json=gn_actual,
        require_pct_rust_process_gn_golden=True,
        pct_gpmp_optimizer_compare_json=compare_json,
        require_pct_gpmp_optimizer_compare=True,
    )
    return status


def summarize_migration_status(status: dict[str, Any]) -> dict[str, Any]:
    claims = status.get("claims") if isinstance(status.get("claims"), dict) else {}
    return {
        "ok": bool(status.get("ok")),
        "claim_allowed": bool(status.get("claim_allowed")),
        "claims": {claim: claims.get(claim) for claim in REQUIRED_STATUS_CLAIMS},
        "blocker_count": len(status.get("blockers") or []),
        "warning_count": len(status.get("warnings") or []),
        "pct_rust_process_golden_readiness": (
            status.get("pct_rust_process_golden_readiness") or {}
        ),
        "pct_rust_process_gn_golden_readiness": (
            status.get("pct_rust_process_gn_golden_readiness") or {}
        ),
        "pct_gpmp_optimizer_compare_readiness": (
            status.get("pct_gpmp_optimizer_compare_readiness") or {}
        ),
    }


def evaluate_acceptance(payload: dict[str, Any]) -> dict[str, Any]:
    failed: list[str] = []
    smokes = payload.get("runtime_smokes") if isinstance(payload, dict) else {}
    if not isinstance(smokes, dict):
        failed.append("runtime_smokes:not_object")
        smokes = {}
    for nonlinear_optimizer in DEFAULT_NONLINEAR_OPTIMIZERS:
        smoke = smokes.get(nonlinear_optimizer)
        if not isinstance(smoke, dict):
            failed.append(f"{nonlinear_optimizer}:missing_runtime_smoke")
            continue
        actual = smoke.get("actual") if isinstance(smoke.get("actual"), dict) else {}
        comparison = (
            smoke.get("comparison") if isinstance(smoke.get("comparison"), dict) else {}
        )
        diagnostics = (
            actual.get("diagnostics") if isinstance(actual.get("diagnostics"), dict) else {}
        )
        runtime = actual.get("runtime") if isinstance(actual.get("runtime"), dict) else {}
        if not actual.get("ok"):
            failed.append(f"{nonlinear_optimizer}:actual_not_ok")
        if runtime.get("runtime") != "rust_process":
            failed.append(f"{nonlinear_optimizer}:runtime_not_rust_process")
        if diagnostics.get("last_optimizer_nonlinear_optimizer") != nonlinear_optimizer:
            failed.append(f"{nonlinear_optimizer}:wrong_nonlinear_optimizer")
        if diagnostics.get("last_optimizer_linear_solver") != "block_tridiagonal":
            failed.append(f"{nonlinear_optimizer}:wrong_linear_solver")
        if int(diagnostics.get("last_optimizer_linear_solve_fallbacks") or 0) != 0:
            failed.append(f"{nonlinear_optimizer}:linear_solver_fallback")
        if diagnostics.get("last_optimizer_call_mode") not in {"ffi", "process"}:
            failed.append(f"{nonlinear_optimizer}:optimizer_not_called")
        if comparison.get("verdict") != "pass":
            failed.append(f"{nonlinear_optimizer}:golden_compare_failed")

    optimizer_compare = payload.get("optimizer_compare")
    if not isinstance(optimizer_compare, dict):
        failed.append("optimizer_compare:not_object")
        optimizer_payload = {}
    else:
        optimizer_payload = optimizer_compare.get("payload")
        if not isinstance(optimizer_payload, dict):
            failed.append("optimizer_compare:payload_missing")
            optimizer_payload = {}
    summary = (
        optimizer_payload.get("summary")
        if isinstance(optimizer_payload.get("summary"), dict)
        else {}
    )
    if summary.get("verdict") != "pass":
        failed.append("optimizer_compare_failed")

    status = payload.get("migration_status")
    if not isinstance(status, dict):
        failed.append("migration_status:not_object")
    else:
        if not status.get("ok"):
            failed.append("migration_status_not_ok")
        claims = status.get("claims") if isinstance(status.get("claims"), dict) else {}
        for claim in REQUIRED_STATUS_CLAIMS:
            if claims.get(claim) is not True:
                failed.append(f"migration_status_claim_false:{claim}")

    return {
        "verdict": "pass" if not failed else "fail",
        "failed_checks": sorted(set(failed)),
        "required_status_claims": list(REQUIRED_STATUS_CLAIMS),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--optimizer-bin", type=Path, default=None)
    parser.add_argument("--build", action="store_true", help="build Rust optimizer artifacts")
    parser.add_argument("--work-dir", type=Path, default=None)
    parser.add_argument("--linear-solver", default="block_tridiagonal")
    parser.add_argument("--state-count", type=int, default=96)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--max-iterations", type=int, default=8)
    parser.add_argument("--min-speedup", type=float, default=1.25)
    parser.add_argument("--final-cost-abs-tol", type=float, default=1e-6)
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--json", action="store_true", help="print JSON only")
    parser.add_argument("--enforce", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    smoke = _load_smoke_module()
    optimizer_bin = smoke.resolve_optimizer_binary(args.optimizer_bin, build=args.build)

    temp_dir: tempfile.TemporaryDirectory[str] | None = None
    if args.work_dir is None:
        temp_dir = tempfile.TemporaryDirectory(prefix="lingtu_pct_rust_acceptance_")
        work_dir = Path(temp_dir.name)
    else:
        work_dir = args.work_dir
        work_dir.mkdir(parents=True, exist_ok=True)

    try:
        runtime_smokes = {
            nonlinear_optimizer: run_runtime_smoke(
                optimizer_bin=optimizer_bin,
                work_dir=work_dir,
                nonlinear_optimizer=nonlinear_optimizer,
                linear_solver=str(args.linear_solver),
            )
            for nonlinear_optimizer in DEFAULT_NONLINEAR_OPTIMIZERS
        }
        optimizer_compare = run_optimizer_compare(
            optimizer_bin=optimizer_bin,
            work_dir=work_dir,
            state_count=int(args.state_count),
            repeats=max(1, int(args.repeats)),
            max_iterations=int(args.max_iterations),
            min_speedup=float(args.min_speedup),
            final_cost_abs_tol=float(args.final_cost_abs_tol),
        )
        migration_status = build_migration_status(
            runtime_smokes=runtime_smokes,
            optimizer_compare=optimizer_compare,
        )
        migration_status_summary = summarize_migration_status(migration_status)
        payload = {
            "schema": SCHEMA,
            "optimizer_bin": str(optimizer_bin),
            "work_dir": str(work_dir),
            "runtime_smokes": runtime_smokes,
            "optimizer_compare": optimizer_compare,
            "migration_status": migration_status_summary,
        }
        payload["summary"] = evaluate_acceptance(payload)

        if args.json_out is not None:
            args.json_out.parent.mkdir(parents=True, exist_ok=True)
            args.json_out.write_text(
                json.dumps(payload, indent=2, sort_keys=True),
                encoding="utf-8",
            )

        if args.json:
            print(json.dumps(payload, indent=2, sort_keys=True))
        else:
            summary = payload["summary"]
            print(f"PCT Rust runtime acceptance: {summary['verdict']}")
            for name, smoke_payload in runtime_smokes.items():
                smoke_summary = smoke_payload["summary"]
                print(
                    "  "
                    f"{name}: runtime={smoke_summary['runtime']} "
                    f"call={smoke_summary['call_mode']} "
                    f"golden={smoke_summary['comparison']} "
                    f"path_count={smoke_summary['path_count']} "
                    f"fallbacks={smoke_summary['fallbacks']}"
                )
            compare_summary = optimizer_compare["payload"]["summary"]
            print(f"  optimizer_compare: {compare_summary['verdict']}")
            if summary["failed_checks"]:
                print("  failed_checks:")
                for check in summary["failed_checks"]:
                    print(f"    - {check}")

        if args.enforce and payload["summary"]["verdict"] != "pass":
            return 1
        return 0
    finally:
        if temp_dir is not None:
            temp_dir.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
