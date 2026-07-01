#!/usr/bin/env python3
"""Run the PCT GPMP Rust-vs-legacy-math acceptance gate.

The gate compares the Rust GPMP pure-math fixture against the legacy C++
process-model headers, then feeds the artifacts into the kernel migration
status validator. It does not build or link legacy native optimizer libraries.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
RUST_CRATE = ROOT / "src" / "kernels" / "planning" / "gpmp_trajectory_optimizer"
SCHEMA = "lingtu.pct_gpmp_math_acceptance.v1"
REQUIRED_CASES = (
    "wnoj_positive_heading_rate",
    "wnoj_negative_heading_rate",
    "wnoj_heading_rate_inside_limit",
    "wnoa_nominal",
)


def _load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _load_cpp_baseline_module() -> Any:
    return _load_module(
        "pct_gpmp_math_cpp_baseline_acceptance",
        ROOT / "tools" / "bench" / "pct_gpmp_math_cpp_baseline.py",
    )


def _load_compare_module() -> Any:
    return _load_module(
        "pct_gpmp_math_compare_acceptance",
        ROOT / "tools" / "bench" / "pct_gpmp_math_compare.py",
    )


def _load_status_module() -> Any:
    return _load_module(
        "validate_kernel_migration_status_math_acceptance",
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


def _run_command(command: list[str], *, cwd: Path) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            command,
            cwd=str(cwd),
            check=True,
            text=True,
            encoding="utf-8",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except subprocess.CalledProcessError as exc:
        stdout = (exc.stdout or "").strip()
        stderr = (exc.stderr or "").strip()
        details = "\n".join(
            part
            for part in [
                f"command failed ({exc.returncode}): {' '.join(command)}",
                f"stdout:\n{stdout}" if stdout else "",
                f"stderr:\n{stderr}" if stderr else "",
            ]
            if part
        )
        raise RuntimeError(details) from exc


def run_rust_fixture(*, cargo: str, work_dir: Path) -> dict[str, Any]:
    completed = _run_command(
        [
            cargo,
            "run",
            "--manifest-path",
            str(RUST_CRATE / "Cargo.toml"),
            "--example",
            "math_fixture",
            "--quiet",
        ],
        cwd=ROOT,
    )
    payload = json.loads(completed.stdout)
    if not isinstance(payload, dict):
        raise ValueError("Rust math fixture did not emit a JSON object")
    output_path = work_dir / "pct_gpmp_math_rust.json"
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    return {
        "json": _repo_path(output_path),
        "payload": payload,
    }


def run_cpp_baseline(
    *,
    work_dir: Path,
    cmake: str,
    config: str,
    eigen_include_dir: Path | None,
) -> dict[str, Any]:
    module = _load_cpp_baseline_module()
    payload = module.run_cpp_baseline(
        repo_root=ROOT,
        build_dir=work_dir / "cpp_baseline_build",
        cmake=cmake,
        config=config,
        eigen_include_dir=eigen_include_dir,
    )
    output_path = work_dir / "pct_gpmp_math_cpp.json"
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    return {
        "json": _repo_path(output_path),
        "payload": payload,
    }


def run_compare(
    *,
    rust_fixture: dict[str, Any],
    cpp_baseline: dict[str, Any],
    work_dir: Path,
    abs_tol: float,
    rel_tol: float,
) -> dict[str, Any]:
    module = _load_compare_module()
    result = module.compare(
        rust_fixture["payload"],
        cpp_baseline["payload"],
        abs_tol=abs_tol,
        rel_tol=rel_tol,
    )
    output_path = work_dir / "pct_gpmp_math_compare.json"
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    return {
        "json": _repo_path(output_path),
        "payload": result,
    }


def build_migration_status(
    *,
    rust_fixture: dict[str, Any],
    cpp_baseline: dict[str, Any],
) -> dict[str, Any]:
    module = _load_status_module()
    return module.build_status(
        ROOT,
        pct_gpmp_math_rust_json=_artifact_path(rust_fixture["json"]),
        pct_gpmp_math_baseline_json=_artifact_path(cpp_baseline["json"]),
        require_pct_gpmp_math=True,
    )


def summarize_status(status: dict[str, Any]) -> dict[str, Any]:
    claims = status.get("claims") if isinstance(status.get("claims"), dict) else {}
    return {
        "ok": bool(status.get("ok")),
        "claim_allowed": bool(status.get("claim_allowed")),
        "claims": {
            "pct_gpmp_math_kernel_parity": claims.get("pct_gpmp_math_kernel_parity"),
        },
        "blocker_count": len(status.get("blockers") or []),
        "warning_count": len(status.get("warnings") or []),
        "pct_gpmp_math_readiness": status.get("pct_gpmp_math_readiness") or {},
    }


def evaluate_acceptance(payload: dict[str, Any]) -> dict[str, Any]:
    failed: list[str] = []
    rust_fixture = payload.get("rust_fixture")
    if not isinstance(rust_fixture, dict) or not isinstance(rust_fixture.get("payload"), dict):
        failed.append("rust_fixture_missing")
        rust_payload: dict[str, Any] = {}
    else:
        rust_payload = rust_fixture["payload"]
    cpp_baseline = payload.get("cpp_baseline")
    if not isinstance(cpp_baseline, dict) or not isinstance(cpp_baseline.get("payload"), dict):
        failed.append("cpp_baseline_missing")
        cpp_payload: dict[str, Any] = {}
    else:
        cpp_payload = cpp_baseline["payload"]
    if rust_payload.get("schema") != "lingtu.pct_gpmp_math.result.v1":
        failed.append("rust_fixture_invalid_schema")
    if cpp_payload.get("schema") != "lingtu.pct_gpmp_math.result.v1":
        failed.append("cpp_baseline_invalid_schema")
    rust_cases = {
        case.get("case")
        for case in rust_payload.get("cases", [])
        if isinstance(case, dict)
    }
    cpp_cases = {
        case.get("case")
        for case in cpp_payload.get("cases", [])
        if isinstance(case, dict)
    }
    for case_name in REQUIRED_CASES:
        if case_name not in rust_cases:
            failed.append(f"rust_missing_case:{case_name}")
        if case_name not in cpp_cases:
            failed.append(f"cpp_missing_case:{case_name}")

    comparison = payload.get("comparison")
    if not isinstance(comparison, dict):
        failed.append("comparison_missing")
        comparison_payload: dict[str, Any] = {}
    else:
        comparison_payload = comparison.get("payload")
        if not isinstance(comparison_payload, dict):
            failed.append("comparison_payload_missing")
            comparison_payload = {}
    if comparison_payload.get("verdict") != "pass":
        failed.append("comparison_failed")

    status = payload.get("migration_status")
    if not isinstance(status, dict):
        failed.append("migration_status_missing")
    else:
        claims = status.get("claims") if isinstance(status.get("claims"), dict) else {}
        readiness = status.get("pct_gpmp_math_readiness")
        readiness_status = (
            readiness.get("status") if isinstance(readiness, dict) else None
        )
        if not status.get("ok"):
            failed.append("migration_status_not_ok")
        if claims.get("pct_gpmp_math_kernel_parity") is not True:
            failed.append("migration_status_claim_false:pct_gpmp_math_kernel_parity")
        if readiness_status != "pass":
            failed.append("migration_status_math_readiness_not_pass")

    return {
        "verdict": "pass" if not failed else "fail",
        "failed_checks": sorted(set(failed)),
        "required_cases": list(REQUIRED_CASES),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--work-dir", type=Path, default=None)
    parser.add_argument("--cargo", default="cargo")
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--config", default="Release")
    parser.add_argument("--eigen-include-dir", type=Path, default=None)
    parser.add_argument("--abs-tol", type=float, default=1e-9)
    parser.add_argument("--rel-tol", type=float, default=1e-9)
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--json", action="store_true", help="print JSON only")
    parser.add_argument("--enforce", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    temp_dir: tempfile.TemporaryDirectory[str] | None = None
    if args.work_dir is None:
        temp_dir = tempfile.TemporaryDirectory(prefix="lingtu_pct_gpmp_math_acceptance_")
        work_dir = Path(temp_dir.name)
    else:
        work_dir = args.work_dir
        work_dir.mkdir(parents=True, exist_ok=True)

    try:
        rust_fixture = run_rust_fixture(cargo=str(args.cargo), work_dir=work_dir)
        cpp_baseline = run_cpp_baseline(
            work_dir=work_dir,
            cmake=str(args.cmake),
            config=str(args.config),
            eigen_include_dir=args.eigen_include_dir,
        )
        comparison = run_compare(
            rust_fixture=rust_fixture,
            cpp_baseline=cpp_baseline,
            work_dir=work_dir,
            abs_tol=float(args.abs_tol),
            rel_tol=float(args.rel_tol),
        )
        migration_status = summarize_status(
            build_migration_status(
                rust_fixture=rust_fixture,
                cpp_baseline=cpp_baseline,
            )
        )
        payload = {
            "schema": SCHEMA,
            "work_dir": str(work_dir),
            "rust_fixture": rust_fixture,
            "cpp_baseline": cpp_baseline,
            "comparison": comparison,
            "migration_status": migration_status,
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
            print(f"PCT GPMP math acceptance: {summary['verdict']}")
            print(f"  cases: {', '.join(summary['required_cases'])}")
            print(f"  comparison: {comparison['payload'].get('verdict')}")
            readiness = migration_status.get("pct_gpmp_math_readiness") or {}
            print(f"  migration_status_math: {readiness.get('status')}")
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
