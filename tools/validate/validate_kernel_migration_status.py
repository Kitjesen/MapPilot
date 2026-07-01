#!/usr/bin/env python3
"""Aggregate kernel migration status into one machine-readable gate."""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any


SCHEMA = "lingtu.kernel_migration_status.v1"
PCT_RUST_RUNTIME_ACCEPTANCE_SCHEMA = "lingtu.pct_rust_runtime_acceptance.v1"
ROOT_DIR = Path(__file__).resolve().parents[2]
DEFAULT_PCT_GOLDEN = (
    "src/nav/tests/planning_backends/fixtures/pct_preview/building2_9_smoke.json"
)
DEFAULT_PCT_RUST_PROCESS_GOLDEN = (
    "src/nav/tests/planning_backends/fixtures/pct_preview/rust_process_synthetic_smoke.json"
)
DEFAULT_PCT_RUST_PROCESS_GN_GOLDEN = (
    "src/nav/tests/planning_backends/fixtures/pct_preview/rust_process_synthetic_gn_smoke.json"
)
REQUIRED_PCT_GPMP_MATH_CASES = (
    "wnoj_positive_heading_rate",
    "wnoj_negative_heading_rate",
    "wnoj_heading_rate_inside_limit",
    "wnoa_nominal",
)
REQUIRED_PCT_GPMP_OPTIMIZER_MODES = ("wnoj", "wnoa")
REQUIRED_PCT_GPMP_OPTIMIZER_NONLINEAR = (
    "levenberg_marquardt",
    "gauss_newton",
)
REQUIRED_CAMERA_LIDAR_ABI_SMOKE_CHECKS = (
    "abi_version",
    "pose3_size",
    "ct_gicp_size",
    "ct_gicp_source_size",
    "ct_gicp_target_size",
    "linearization_size",
    "optimizer_config_size",
    "optimizer_result_size",
    "return_code",
    "used_correspondences",
    "cost",
    "gradient",
    "rhs",
    "hessian",
    "optimizer_return_code",
    "optimizer_used_correspondences",
    "optimizer_cost_reduction",
    "optimizer_final_cost_finite",
    "optimizer_pose0_translation",
    "dynamic_optimizer_return_code",
    "dynamic_optimizer_used_correspondences",
    "dynamic_optimizer_cost_reduction",
    "dynamic_optimizer_final_cost_finite",
    "dynamic_optimizer_pose0_translation",
)


def _load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _repo_path(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return str(path)


def _load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8-sig") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def _pose_graph_status(repo_root: Path) -> dict[str, Any]:
    module = _load_module(
        "validate_pose_graph_opt_coverage_status",
        repo_root / "tools/validate/validate_pose_graph_opt_coverage.py",
    )
    audit = module.scan_repository(repo_root)
    payload = audit.to_jsonable()
    covered = [
        surface["surface"]
        for surface in payload["capability_surfaces"]
        if surface.get("coverage") == "covered_by_pose_graph_opt"
    ]
    return {
        "covered_surfaces": covered,
        "violations": payload["violations"],
        "category_counts": payload["category_counts"],
        "remaining_dependency_surface_summary": payload[
            "remaining_dependency_surface_summary"
        ],
        "remaining_dependency_subsurface_summary": payload[
            "remaining_dependency_subsurface_summary"
        ],
    }


def _pct_contract_status(repo_root: Path) -> dict[str, Any]:
    module = _load_module(
        "validate_pct_gpmp_migration_contract_status",
        repo_root / "tools/validate/validate_pct_gpmp_migration_contract.py",
    )
    contract = module.build_contract(repo_root)
    return {
        "ok": bool(contract["ok"]),
        "schema": contract["schema"],
        "covered_by_pose_graph_opt": contract["covered_by_pose_graph_opt"],
        "missing_required_checks": contract["missing_required_checks"],
        "runtime_boundary": contract["runtime_boundary"],
        "rust_math_kernel": contract.get("rust_math_kernel", {}),
        "required_capability_groups": contract["required_capability_groups"],
    }


def _camera_lidar_contract_status(repo_root: Path) -> dict[str, Any]:
    module = _load_module(
        "validate_camera_lidar_calibration_migration_contract_status",
        repo_root / "tools/validate/validate_camera_lidar_calibration_migration_contract.py",
    )
    contract = module.build_contract(repo_root)
    return {
        "ok": bool(contract["ok"]),
        "schema": contract["schema"],
        "covered_by_pose_graph_opt": contract["covered_by_pose_graph_opt"],
        "missing_required_checks": contract["missing_required_checks"],
        "default_library_source_audit": contract.get("default_library_source_audit", {}),
        "rust_kernel": contract.get("rust_kernel", {}),
        "required_capability_groups": contract["required_capability_groups"],
        "current_gtsam_surfaces": contract.get("current_gtsam_surfaces", {}),
        "removed_gtsam_build_files": contract.get("removed_gtsam_build_files", []),
    }


def _pct_golden_status(
    repo_root: Path,
    golden_path: Path,
    actual_path: Path | None,
    *,
    require_actual: bool,
    blocker_prefix: str = "pct_golden",
) -> tuple[dict[str, Any], list[str]]:
    blockers: list[str] = []
    payload: dict[str, Any] = {
        "golden": _repo_path(golden_path, repo_root),
        "actual": _repo_path(actual_path, repo_root) if actual_path else None,
        "status": "not_requested",
        "verdict": None,
        "failed_checks": [],
        "skipped_checks": [],
    }
    if actual_path is None:
        if require_actual:
            payload["status"] = "missing_actual"
            blockers.append(f"{blocker_prefix}_actual_missing")
        return payload, blockers

    if not golden_path.is_file():
        payload["status"] = "fail"
        blockers.append(f"{blocker_prefix}_missing")
        return payload, blockers
    if not actual_path.is_file():
        payload["status"] = "missing_actual"
        blockers.append(f"{blocker_prefix}_actual_missing")
        return payload, blockers

    module = _load_module(
        "pct_preview_compare_status",
        repo_root / "tools/bench/pct_preview_compare.py",
    )
    result = module.compare(_load_json(golden_path), _load_json(actual_path))
    payload.update(
        {
            "status": "pass" if result["verdict"] == "pass" else "fail",
            "verdict": result["verdict"],
            "failed_checks": result["failed_checks"],
            "skipped_checks": result["skipped_checks"],
        }
    )
    if result["verdict"] != "pass":
        blockers.append(f"{blocker_prefix}_compare_failed")
    return payload, blockers


def _pct_gpmp_math_status(
    repo_root: Path,
    rust_result_path: Path | None,
    baseline_path: Path | None,
    *,
    require_compare: bool,
) -> tuple[dict[str, Any], list[str]]:
    blockers: list[str] = []
    payload: dict[str, Any] = {
        "rust_result": _repo_path(rust_result_path, repo_root)
        if rust_result_path
        else None,
        "baseline": _repo_path(baseline_path, repo_root) if baseline_path else None,
        "status": "not_requested",
        "verdict": None,
        "failed_checks": [],
        "skipped_checks": [],
        "matched_cases": [],
        "missing_in_rust": [],
        "missing_in_baseline": [],
    }

    if rust_result_path is None and baseline_path is None:
        if require_compare:
            payload["status"] = "missing_compare_inputs"
            blockers.append("pct_gpmp_math_compare_inputs_missing")
        return payload, blockers
    if rust_result_path is None:
        payload["status"] = "missing_rust_result"
        blockers.append("pct_gpmp_math_rust_result_missing")
        return payload, blockers
    if baseline_path is None:
        payload["status"] = "missing_baseline"
        blockers.append("pct_gpmp_math_baseline_missing")
        return payload, blockers
    if not rust_result_path.is_file():
        payload["status"] = "missing_rust_result"
        blockers.append("pct_gpmp_math_rust_result_missing")
        return payload, blockers
    if not baseline_path.is_file():
        payload["status"] = "missing_baseline"
        blockers.append("pct_gpmp_math_baseline_missing")
        return payload, blockers

    module = _load_module(
        "pct_gpmp_math_compare_status",
        repo_root / "tools/bench/pct_gpmp_math_compare.py",
    )
    result = module.compare(_load_json(rust_result_path), _load_json(baseline_path))
    failed_checks = list(result["failed_checks"])
    matched_cases = list(result["matched_cases"])
    matched_case_set = set(matched_cases)
    for required_case in REQUIRED_PCT_GPMP_MATH_CASES:
        if required_case not in matched_case_set:
            failed_checks.append(f"{required_case}:missing_required_case")
    if result["missing_in_rust"]:
        failed_checks.append("missing_cases_in_rust")
    if result["missing_in_baseline"]:
        failed_checks.append("missing_cases_in_baseline")
    failed_checks = sorted(
        {
            check if isinstance(check, str) else str(check.get("name", check))
            for check in failed_checks
        }
    )
    verdict = "pass" if result["verdict"] == "pass" and not failed_checks else "fail"
    payload.update(
        {
            "status": "pass" if verdict == "pass" else "fail",
            "verdict": verdict,
            "failed_checks": failed_checks,
            "skipped_checks": result["skipped_checks"],
            "matched_cases": matched_cases,
            "missing_in_rust": result["missing_in_rust"],
            "missing_in_baseline": result["missing_in_baseline"],
            "required_cases": list(REQUIRED_PCT_GPMP_MATH_CASES),
        }
    )
    if verdict != "pass":
        blockers.append("pct_gpmp_math_compare_failed")
    return payload, blockers


def _pct_gpmp_optimizer_compare_status(
    repo_root: Path,
    compare_path: Path | None,
    *,
    require_compare: bool,
) -> tuple[dict[str, Any], list[str]]:
    blockers: list[str] = []
    payload: dict[str, Any] = {
        "result": _repo_path(compare_path, repo_root) if compare_path else None,
        "status": "not_requested",
        "verdict": None,
        "failed_checks": [],
        "cases": [],
    }
    if compare_path is None:
        if require_compare:
            payload["status"] = "missing_result"
            blockers.append("pct_gpmp_optimizer_compare_missing")
        return payload, blockers
    if not compare_path.is_file():
        payload["status"] = "missing_result"
        blockers.append("pct_gpmp_optimizer_compare_missing")
        return payload, blockers

    result = _load_json(compare_path)
    if result.get("schema") != "lingtu.pct_gpmp_optimizer_compare.v1":
        payload["status"] = "fail"
        payload["failed_checks"] = ["invalid_schema"]
        blockers.append("pct_gpmp_optimizer_compare_failed")
        return payload, blockers

    summary = result.get("summary") or {}
    failed_checks = list(summary.get("failed_checks") or [])
    min_speedup = float(summary.get("min_speedup", 1.25))
    final_cost_abs_tol = float(summary.get("final_cost_abs_tol", 1e-6))
    cases = result.get("cases") if isinstance(result.get("cases"), list) else []
    if not cases:
        failed_checks.append("missing_cases")
    mode_optimizer_pairs = {
        (
            str(case.get("mode")),
            str(case.get("nonlinear_optimizer")),
        )
        for case in cases
        if isinstance(case, dict)
    }
    for required_mode in REQUIRED_PCT_GPMP_OPTIMIZER_MODES:
        for required_optimizer in REQUIRED_PCT_GPMP_OPTIMIZER_NONLINEAR:
            if (required_mode, required_optimizer) not in mode_optimizer_pairs:
                failed_checks.append(f"{required_mode}:{required_optimizer}:missing_case")
    for case in cases:
        if not isinstance(case, dict):
            failed_checks.append("case:not_object")
            continue
        mode = str(case.get("mode"))
        nonlinear_optimizer = str(case.get("nonlinear_optimizer"))
        state_count = case.get("state_count")
        label = f"{mode}:{nonlinear_optimizer}:{state_count}"
        dense = case.get("dense") if isinstance(case.get("dense"), dict) else {}
        block = (
            case.get("block_tridiagonal")
            if isinstance(case.get("block_tridiagonal"), dict)
            else {}
        )
        if not dense:
            failed_checks.append(f"{label}:dense_missing")
        if not block:
            failed_checks.append(f"{label}:block_tridiagonal_missing")
        if dense.get("reported_linear_solver") != "dense":
            failed_checks.append(f"{label}:dense_solver_not_reported")
        if block.get("reported_linear_solver") != "block_tridiagonal":
            failed_checks.append(f"{label}:block_solver_not_reported")
        if dense.get("reported_nonlinear_optimizer") != nonlinear_optimizer:
            failed_checks.append(f"{label}:dense_nonlinear_optimizer_not_reported")
        if block.get("reported_nonlinear_optimizer") != nonlinear_optimizer:
            failed_checks.append(f"{label}:block_nonlinear_optimizer_not_reported")
        if int(block.get("linear_solve_fallbacks") or 0) != 0:
            failed_checks.append(f"{label}:block_solver_fallbacks")
        speedup = case.get("speedup")
        if not isinstance(speedup, (int, float)) or speedup < min_speedup:
            failed_checks.append(f"{label}:speedup_below_threshold")
        final_cost_delta = case.get("final_cost_delta_abs")
        if (
            not isinstance(final_cost_delta, (int, float))
            or final_cost_delta > final_cost_abs_tol
        ):
            failed_checks.append(f"{label}:final_cost_delta")
    failed_checks = sorted(set(failed_checks))
    verdict = "pass" if summary.get("verdict") == "pass" and not failed_checks else "fail"
    payload.update(
        {
            "status": "pass" if verdict == "pass" else "fail",
            "verdict": verdict,
            "failed_checks": failed_checks,
            "cases": [
                {
                    "mode": case.get("mode"),
                    "nonlinear_optimizer": case.get("nonlinear_optimizer"),
                    "state_count": case.get("state_count"),
                    "speedup": case.get("speedup"),
                    "final_cost_delta_abs": case.get("final_cost_delta_abs"),
                }
                for case in cases
                if isinstance(case, dict)
            ],
        }
    )
    if verdict != "pass":
        blockers.append("pct_gpmp_optimizer_compare_failed")
    return payload, blockers


def _resolve_acceptance_artifact(repo_root: Path, value: Any) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    path = Path(value)
    return path if path.is_absolute() else repo_root / path


def _pct_rust_runtime_acceptance_status(
    repo_root: Path,
    acceptance_path: Path | None,
) -> tuple[dict[str, Any], list[str], dict[str, Path]]:
    blockers: list[str] = []
    artifacts: dict[str, Path] = {}
    payload: dict[str, Any] = {
        "result": _repo_path(acceptance_path, repo_root) if acceptance_path else None,
        "status": "not_requested",
        "verdict": None,
        "failed_checks": [],
        "derived_artifacts": {},
    }
    if acceptance_path is None:
        return payload, blockers, artifacts
    if not acceptance_path.is_file():
        payload["status"] = "missing_result"
        blockers.append("pct_rust_runtime_acceptance_missing")
        return payload, blockers, artifacts

    result = _load_json(acceptance_path)
    failed_checks: list[str] = []
    if result.get("schema") != PCT_RUST_RUNTIME_ACCEPTANCE_SCHEMA:
        failed_checks.append("invalid_schema")

    summary = result.get("summary") if isinstance(result.get("summary"), dict) else {}
    if summary.get("verdict") != "pass":
        failed_checks.append("summary_not_pass")
    failed_checks.extend(str(item) for item in summary.get("failed_checks") or [])

    runtime_smokes = (
        result.get("runtime_smokes")
        if isinstance(result.get("runtime_smokes"), dict)
        else {}
    )
    optimizer_compare = (
        result.get("optimizer_compare")
        if isinstance(result.get("optimizer_compare"), dict)
        else {}
    )
    artifact_specs = {
        "levenberg_marquardt_actual_json": (
            runtime_smokes.get("levenberg_marquardt") or {}
        ).get("actual_json")
        if isinstance(runtime_smokes.get("levenberg_marquardt"), dict)
        else None,
        "gauss_newton_actual_json": (runtime_smokes.get("gauss_newton") or {}).get(
            "actual_json"
        )
        if isinstance(runtime_smokes.get("gauss_newton"), dict)
        else None,
        "optimizer_compare_json": optimizer_compare.get("json"),
    }
    derived_artifacts: dict[str, str | None] = {}
    for name, raw_value in artifact_specs.items():
        artifact_path = _resolve_acceptance_artifact(repo_root, raw_value)
        if artifact_path is None:
            failed_checks.append(f"{name}:missing_path")
            derived_artifacts[name] = None
            continue
        artifacts[name] = artifact_path
        derived_artifacts[name] = _repo_path(artifact_path, repo_root)
        if not artifact_path.is_file():
            failed_checks.append(f"{name}:missing_file")

    failed_checks = sorted(set(failed_checks))
    verdict = "pass" if not failed_checks else "fail"
    payload.update(
        {
            "status": "pass" if verdict == "pass" else "fail",
            "verdict": verdict,
            "failed_checks": failed_checks,
            "derived_artifacts": derived_artifacts,
        }
    )
    if verdict != "pass":
        blockers.append("pct_rust_runtime_acceptance_failed")
    return payload, blockers, artifacts


def _pct_native_rust_parity_status(
    repo_root: Path,
    parity_path: Path | None,
    *,
    require_parity: bool,
) -> tuple[dict[str, Any], list[str]]:
    blockers: list[str] = []
    payload: dict[str, Any] = {
        "result": _repo_path(parity_path, repo_root) if parity_path else None,
        "status": "not_requested",
        "verdict": None,
        "failed_checks": [],
    }
    if parity_path is None:
        if require_parity:
            payload["status"] = "missing_result"
            blockers.append("pct_native_rust_parity_missing")
        return payload, blockers
    if not parity_path.is_file():
        payload["status"] = "missing_result"
        blockers.append("pct_native_rust_parity_missing")
        return payload, blockers

    result = _load_json(parity_path)
    if result.get("schema") != "lingtu.pct.native_rust_parity.v1":
        payload["status"] = "fail"
        payload["failed_checks"] = ["invalid_schema"]
        blockers.append("pct_native_rust_parity_failed")
        return payload, blockers

    summary = result.get("summary") or {}
    verdict = summary.get("verdict")
    payload.update(
        {
            "status": "pass" if verdict == "pass" else "fail",
            "verdict": verdict,
            "failed_checks": list(summary.get("failed_checks") or []),
            "native_ok": bool((result.get("native") or {}).get("ok")),
            "rust_process_ok": bool((result.get("rust_process") or {}).get("ok")),
        }
    )
    if verdict != "pass":
        blockers.append("pct_native_rust_parity_failed")
    return payload, blockers


def _camera_lidar_abi_smoke_status(
    repo_root: Path,
    smoke_path: Path | None,
    *,
    require_smoke: bool,
) -> tuple[dict[str, Any], list[str]]:
    blockers: list[str] = []
    payload: dict[str, Any] = {
        "result": _repo_path(smoke_path, repo_root) if smoke_path else None,
        "status": "not_requested",
        "verdict": None,
        "failed_checks": [],
        "library": None,
    }
    if smoke_path is None:
        if require_smoke:
            payload["status"] = "missing_result"
            blockers.append("camera_lidar_abi_smoke_missing")
        return payload, blockers
    if not smoke_path.is_file():
        payload["status"] = "missing_result"
        blockers.append("camera_lidar_abi_smoke_missing")
        return payload, blockers

    result = _load_json(smoke_path)
    if result.get("schema") != "lingtu.camera_lidar_optimizer_abi_smoke.v1":
        payload["status"] = "fail"
        payload["failed_checks"] = ["invalid_schema"]
        blockers.append("camera_lidar_abi_smoke_failed")
        return payload, blockers

    summary = result.get("summary") if isinstance(result.get("summary"), dict) else {}
    failed_checks = list(summary.get("failed_checks") or [])
    checks = result.get("checks") if isinstance(result.get("checks"), dict) else {}
    failed_checks.extend(name for name, passed in checks.items() if not passed)
    failed_checks.extend(
        f"{name}:missing"
        for name in REQUIRED_CAMERA_LIDAR_ABI_SMOKE_CHECKS
        if name not in checks
    )
    failed_checks = sorted(set(failed_checks))
    verdict = "pass" if result.get("ok") is True and not failed_checks else "fail"
    payload.update(
        {
            "status": "pass" if verdict == "pass" else "fail",
            "verdict": verdict,
            "failed_checks": failed_checks,
            "library": result.get("library"),
            "optimizer_initial_cost": (result.get("result") or {}).get("optimizer_initial_cost"),
            "optimizer_final_cost": (result.get("result") or {}).get("optimizer_final_cost"),
        }
    )
    if verdict != "pass":
        blockers.append("camera_lidar_abi_smoke_failed")
    return payload, blockers


def build_status(
    repo_root: Path,
    *,
    pct_golden: Path | None = None,
    pct_actual_json: Path | None = None,
    require_pct_golden: bool = False,
    pct_rust_process_golden: Path | None = None,
    pct_rust_process_actual_json: Path | None = None,
    require_pct_rust_process_golden: bool = False,
    pct_rust_process_gn_golden: Path | None = None,
    pct_rust_process_gn_actual_json: Path | None = None,
    require_pct_rust_process_gn_golden: bool = False,
    pct_gpmp_math_rust_json: Path | None = None,
    pct_gpmp_math_baseline_json: Path | None = None,
    require_pct_gpmp_math: bool = False,
    pct_gpmp_optimizer_compare_json: Path | None = None,
    require_pct_gpmp_optimizer_compare: bool = False,
    pct_rust_runtime_acceptance_json: Path | None = None,
    pct_native_rust_parity_json: Path | None = None,
    require_pct_native_rust_parity: bool = False,
    camera_lidar_abi_smoke_json: Path | None = None,
    require_camera_lidar_abi_smoke: bool = False,
) -> dict[str, Any]:
    root = repo_root.resolve()
    golden_path = pct_golden or root / DEFAULT_PCT_GOLDEN
    if not golden_path.is_absolute():
        golden_path = root / golden_path
    actual_path = pct_actual_json
    if actual_path is not None and not actual_path.is_absolute():
        actual_path = root / actual_path
    rust_process_golden_path = pct_rust_process_golden or root / DEFAULT_PCT_RUST_PROCESS_GOLDEN
    if not rust_process_golden_path.is_absolute():
        rust_process_golden_path = root / rust_process_golden_path
    rust_process_actual_path = pct_rust_process_actual_json
    if rust_process_actual_path is not None and not rust_process_actual_path.is_absolute():
        rust_process_actual_path = root / rust_process_actual_path
    rust_process_gn_golden_path = (
        pct_rust_process_gn_golden or root / DEFAULT_PCT_RUST_PROCESS_GN_GOLDEN
    )
    if not rust_process_gn_golden_path.is_absolute():
        rust_process_gn_golden_path = root / rust_process_gn_golden_path
    rust_process_gn_actual_path = pct_rust_process_gn_actual_json
    if rust_process_gn_actual_path is not None and not rust_process_gn_actual_path.is_absolute():
        rust_process_gn_actual_path = root / rust_process_gn_actual_path
    math_rust_path = pct_gpmp_math_rust_json
    if math_rust_path is not None and not math_rust_path.is_absolute():
        math_rust_path = root / math_rust_path
    math_baseline_path = pct_gpmp_math_baseline_json
    if math_baseline_path is not None and not math_baseline_path.is_absolute():
        math_baseline_path = root / math_baseline_path
    optimizer_compare_path = pct_gpmp_optimizer_compare_json
    if optimizer_compare_path is not None and not optimizer_compare_path.is_absolute():
        optimizer_compare_path = root / optimizer_compare_path
    rust_runtime_acceptance_path = pct_rust_runtime_acceptance_json
    if (
        rust_runtime_acceptance_path is not None
        and not rust_runtime_acceptance_path.is_absolute()
    ):
        rust_runtime_acceptance_path = root / rust_runtime_acceptance_path
    native_rust_parity_path = pct_native_rust_parity_json
    if native_rust_parity_path is not None and not native_rust_parity_path.is_absolute():
        native_rust_parity_path = root / native_rust_parity_path
    camera_lidar_abi_smoke_path = camera_lidar_abi_smoke_json
    if camera_lidar_abi_smoke_path is not None and not camera_lidar_abi_smoke_path.is_absolute():
        camera_lidar_abi_smoke_path = root / camera_lidar_abi_smoke_path

    blockers: list[str] = []
    warnings: list[str] = []
    pose_graph = _pose_graph_status(root)
    pct_contract = _pct_contract_status(root)
    camera_lidar_contract = _camera_lidar_contract_status(root)
    (
        pct_rust_runtime_acceptance_status,
        pct_rust_runtime_acceptance_blockers,
        pct_rust_runtime_acceptance_artifacts,
    ) = _pct_rust_runtime_acceptance_status(
        root,
        rust_runtime_acceptance_path,
    )
    if rust_process_actual_path is None:
        rust_process_actual_path = pct_rust_runtime_acceptance_artifacts.get(
            "levenberg_marquardt_actual_json"
        )
    if rust_process_gn_actual_path is None:
        rust_process_gn_actual_path = pct_rust_runtime_acceptance_artifacts.get(
            "gauss_newton_actual_json"
        )
    if optimizer_compare_path is None:
        optimizer_compare_path = pct_rust_runtime_acceptance_artifacts.get(
            "optimizer_compare_json"
        )
    pct_golden_status, pct_golden_blockers = _pct_golden_status(
        root,
        golden_path,
        actual_path,
        require_actual=require_pct_golden,
    )
    pct_rust_process_golden_status, pct_rust_process_golden_blockers = _pct_golden_status(
        root,
        rust_process_golden_path,
        rust_process_actual_path,
        require_actual=require_pct_rust_process_golden,
        blocker_prefix="pct_rust_process_golden",
    )
    (
        pct_rust_process_gn_golden_status,
        pct_rust_process_gn_golden_blockers,
    ) = _pct_golden_status(
        root,
        rust_process_gn_golden_path,
        rust_process_gn_actual_path,
        require_actual=require_pct_rust_process_gn_golden,
        blocker_prefix="pct_rust_process_gn_golden",
    )
    pct_gpmp_math_status, pct_gpmp_math_blockers = _pct_gpmp_math_status(
        root,
        math_rust_path,
        math_baseline_path,
        require_compare=require_pct_gpmp_math,
    )
    (
        pct_gpmp_optimizer_compare_status,
        pct_gpmp_optimizer_compare_blockers,
    ) = _pct_gpmp_optimizer_compare_status(
        root,
        optimizer_compare_path,
        require_compare=require_pct_gpmp_optimizer_compare,
    )
    pct_native_rust_parity_status, pct_native_rust_parity_blockers = (
        _pct_native_rust_parity_status(
            root,
            native_rust_parity_path,
            require_parity=require_pct_native_rust_parity,
        )
    )
    camera_lidar_abi_smoke_status, camera_lidar_abi_smoke_blockers = (
        _camera_lidar_abi_smoke_status(
            root,
            camera_lidar_abi_smoke_path,
            require_smoke=require_camera_lidar_abi_smoke,
        )
    )

    if pose_graph["violations"]:
        blockers.append("pose_graph_opt_coverage_violations")
    if not pct_contract["ok"]:
        blockers.append("pct_gpmp_contract_failed")
    if not camera_lidar_contract["ok"]:
        blockers.append("camera_lidar_calibration_contract_failed")
    blockers.extend(pct_golden_blockers)
    blockers.extend(pct_rust_process_golden_blockers)
    blockers.extend(pct_rust_process_gn_golden_blockers)
    blockers.extend(pct_gpmp_math_blockers)
    blockers.extend(pct_gpmp_optimizer_compare_blockers)
    blockers.extend(pct_rust_runtime_acceptance_blockers)
    blockers.extend(pct_native_rust_parity_blockers)
    blockers.extend(camera_lidar_abi_smoke_blockers)

    remaining = pose_graph["remaining_dependency_surface_summary"]
    if remaining:
        warnings.append("remaining_gtsam_dependency_surfaces_are_tracked_not_removed")
    if pct_golden_status["status"] == "not_requested":
        warnings.append("pct_golden_actual_not_requested")
    if pct_rust_process_golden_status["status"] == "not_requested":
        warnings.append("pct_rust_process_golden_actual_not_requested")
    if pct_rust_process_gn_golden_status["status"] == "not_requested":
        warnings.append("pct_rust_process_gn_golden_actual_not_requested")
    if pct_gpmp_math_status["status"] == "not_requested":
        warnings.append("pct_gpmp_math_compare_not_requested")
    if pct_gpmp_optimizer_compare_status["status"] == "not_requested":
        warnings.append("pct_gpmp_optimizer_compare_not_requested")
    if pct_rust_runtime_acceptance_status["status"] == "not_requested":
        warnings.append("pct_rust_runtime_acceptance_not_requested")
    if pct_native_rust_parity_status["status"] == "not_requested":
        warnings.append("pct_native_rust_parity_not_requested")
    if camera_lidar_abi_smoke_status["status"] == "not_requested":
        warnings.append("camera_lidar_abi_smoke_not_requested")

    ok = not blockers
    pgo_hba_replacement_allowed = not pose_graph["violations"]
    pct_gpmp_math_parity = pct_gpmp_math_status["status"] == "pass"
    pct_gpmp_optimizer_performance = pct_gpmp_optimizer_compare_status["status"] == "pass"
    pct_native_rust_parity = pct_native_rust_parity_status["status"] == "pass"
    camera_lidar_abi_smoke = camera_lidar_abi_smoke_status["status"] == "pass"
    pct_gpmp_capabilities = set(
        (pct_contract.get("rust_math_kernel") or {}).get("covered_capabilities") or []
    )
    pct_gpmp_obstacle_optimizer_kernel = {
        "DenseElevationMap safe layer/height/bilinear obstacle query semantics",
        "WNOJ/WNOA obstacle residuals and Jacobians",
        "WNOJ/WNOA interpolated obstacle chain-rule Jacobians",
        "WNOJ/WNOA dense LM/GN batch optimizer for portable small graphs",
        "WNOJ/WNOA block-tridiagonal sparse LM/GN batch optimizer for trajectory-chain graphs",
        "WNOJ/WNOA faer sparse Cholesky LM/GN batch optimizer for non-chain graphs",
        "Rust process runtime can select LM or GN nonlinear optimization",
        "PCT runtime accepts explicit sparse GPMP linear solver requests",
        "Rust process native-like GPMP optimizer result accessors",
        "PCT preview/golden validates native wrapper optimizer accessor shape parity",
        "Windows-safe dense-vs-block-tridiagonal performance and cost gate",
    }.issubset(pct_gpmp_capabilities)
    pct_gpmp_rust_process_runtime = (
        "Rust process runtime callable from PCT runtime on Windows/lightweight hosts"
        in pct_gpmp_capabilities
    )
    pct_gpmp_docker_gtsam_free = (
        "PCT Docker build packages Rust GPMP optimizer artifacts without GTSAM native modules"
        in pct_gpmp_capabilities
        and "PCT runtime resolves packaged Rust optimizer artifacts from pct/runtime/rust/<arch>"
        in pct_gpmp_capabilities
    )
    pct_gpmp_server_setup_default_rust = (
        "Server setup builds and installs PCT Rust GPMP runtime artifacts by default"
        in pct_gpmp_capabilities
    )
    camera_lidar_capabilities = set(
        (camera_lidar_contract.get("rust_kernel") or {}).get("covered_capabilities") or []
    )
    camera_lidar_ct_icp_kernel = {
        "SE3 exp/log/compose/inverse/interpolate_rt for calibration factors",
        "CT-ICP fixed-correspondence point-to-plane residuals",
        "CT-ICP endpoint Jacobians through continuous-time SE3 interpolation",
        "CT-ICP two-pose Hessian/rhs assembly matching HessianFactor shape",
        "timestamp-keyed endpoint derivative cache for repeated source times",
    }.issubset(camera_lidar_capabilities)
    camera_lidar_ct_gicp_kernel = {
        "CT-GICP fixed-correspondence covariance-weighted Mahalanobis residuals",
        "CT-GICP endpoint Jacobians through continuous-time SE3 interpolation",
        "CT-GICP two-pose Hessian/rhs assembly matching HessianFactor shape",
        "Rust-owned nearest-neighbor correspondence search and max-distance rejection",
        "singular covariance rejection for CT-GICP fused information",
    }.issubset(camera_lidar_capabilities)
    camera_lidar_c_abi = (
        "C ABI for CT-ICP/CT-GICP two-pose linearization calls"
        in camera_lidar_capabilities
    )
    camera_lidar_cpp_rust_abi_bridge = (
        "pure C++ Rust ABI header declares optimizer and correspondence entry points without GTSAM"
        in camera_lidar_capabilities
        and "C++ Rust bridge guards ABI version and struct sizes before factor calls"
        in camera_lidar_capabilities
    )
    camera_lidar_dynamic_integrator_rust_runtime = (
        "CT-GICP two-pose LM optimizer with prior and between constraints"
        in camera_lidar_capabilities
        and "C ABI for CT-GICP nearest-neighbor correspondence construction calls"
        in camera_lidar_capabilities
        and "C ABI for CT-GICP two-pose optimization calls"
        in camera_lidar_capabilities
        and "Rust-owned dynamic CT-GICP correspondence rebuild and two-pose optimization loop"
        in camera_lidar_capabilities
        and "C ABI for dynamic CT-GICP scan optimization calls"
        in camera_lidar_capabilities
        and "dynamic point cloud integrator delegates CT-GICP correspondence rebuild and optimization loop to one Rust C ABI call"
        in camera_lidar_capabilities
        and "dynamic point cloud integrator no longer owns CT-GICP nearest-neighbor search in the Rust runtime path"
        in camera_lidar_capabilities
        and "dynamic point cloud integrator has no GTSAM runtime fallback source path"
        in camera_lidar_capabilities
    )
    camera_lidar_default_rust_required = (
        "default calibration build is Rust-required and skips GTSAM discovery/linkage"
        in camera_lidar_capabilities
    )
    camera_lidar_default_library_source_gtsam_free = (
        "default camera-LiDAR shared library source list is GTSAM-free and excludes legacy CT factor headers"
        in camera_lidar_capabilities
        and bool((camera_lidar_contract.get("default_library_source_audit") or {}).get("ok"))
    )
    camera_lidar_docker_gtsam_free = (
        "camera-LiDAR calibration Docker images use ROS base images, Rust/Cargo, and explicit Ceres/Iridescence builds without GTSAM"
        in camera_lidar_capabilities
    )
    camera_lidar_legacy_gtsam_factors_removed = (
        "legacy CT-ICP/CT-GICP GTSAM factor headers are removed from the repository"
        in camera_lidar_capabilities
        and "legacy Rust-to-GTSAM helper header is removed from the repository"
        in camera_lidar_capabilities
        and "legacy camera-LiDAR GTSAM test support source is removed from the repository"
        in camera_lidar_capabilities
        and not (
            camera_lidar_contract.get("current_gtsam_surfaces", {}).get(
                "legacy_factor_headers"
            )
            or []
        )
        and all(
            item.get("removed")
            for item in camera_lidar_contract.get("removed_gtsam_build_files", [])
            if any(
                marker in str(item.get("path", ""))
                for marker in (
                    "integrated_ct_",
                    "rust_camera_lidar_optimizer_gtsam.hpp",
                    "src/test/outliers.cpp",
                )
            )
        )
    )
    full_replacement_allowed = (
        ok
        and not remaining
        and pct_golden_status["status"] == "pass"
        and pct_rust_process_golden_status["status"] == "pass"
        and pct_rust_process_gn_golden_status["status"] == "pass"
        and pct_gpmp_math_parity
        and pct_gpmp_optimizer_performance
        and pct_native_rust_parity
        and pct_gpmp_obstacle_optimizer_kernel
        and pct_gpmp_rust_process_runtime
        and pct_gpmp_docker_gtsam_free
        and pct_gpmp_server_setup_default_rust
        and camera_lidar_ct_icp_kernel
        and camera_lidar_ct_gicp_kernel
        and camera_lidar_c_abi
        and camera_lidar_cpp_rust_abi_bridge
        and camera_lidar_dynamic_integrator_rust_runtime
        and camera_lidar_default_rust_required
        and camera_lidar_default_library_source_gtsam_free
        and camera_lidar_docker_gtsam_free
        and camera_lidar_legacy_gtsam_factors_removed
        and camera_lidar_abi_smoke
    )
    return {
        "schema": SCHEMA,
        "repo_root": str(root),
        "ok": ok,
        "claim_allowed": full_replacement_allowed,
        "claims": {
            "pgo_hba_pose_graph_opt_replacement": pgo_hba_replacement_allowed,
            "pct_gpmp_math_kernel_parity": pct_gpmp_math_parity,
            "pct_gpmp_obstacle_optimizer_kernel": pct_gpmp_obstacle_optimizer_kernel,
            "pct_gpmp_rust_process_runtime": pct_gpmp_rust_process_runtime,
            "pct_gpmp_docker_gtsam_free": pct_gpmp_docker_gtsam_free,
            "pct_gpmp_server_setup_default_rust": pct_gpmp_server_setup_default_rust,
            "pct_gpmp_optimizer_performance": pct_gpmp_optimizer_performance,
            "pct_native_rust_parity": pct_native_rust_parity,
            "pct_rust_process_golden_parity": pct_rust_process_golden_status["status"] == "pass",
            "pct_rust_process_gn_golden_parity": (
                pct_rust_process_gn_golden_status["status"] == "pass"
            ),
            "camera_lidar_ct_icp_kernel": camera_lidar_ct_icp_kernel,
            "camera_lidar_ct_gicp_kernel": camera_lidar_ct_gicp_kernel,
            "camera_lidar_c_abi": camera_lidar_c_abi,
            "camera_lidar_cpp_rust_abi_bridge": camera_lidar_cpp_rust_abi_bridge,
            "camera_lidar_dynamic_integrator_rust_runtime": (
                camera_lidar_dynamic_integrator_rust_runtime
            ),
            "camera_lidar_default_rust_required": camera_lidar_default_rust_required,
            "camera_lidar_default_library_source_gtsam_free": (
                camera_lidar_default_library_source_gtsam_free
            ),
            "camera_lidar_docker_gtsam_free": camera_lidar_docker_gtsam_free,
            "camera_lidar_legacy_gtsam_factors_removed": (
                camera_lidar_legacy_gtsam_factors_removed
            ),
            "camera_lidar_abi_smoke": camera_lidar_abi_smoke,
            "full_gtsam_replacement": full_replacement_allowed,
        },
        "pose_graph_opt": pose_graph,
        "remaining": remaining,
        "pct_gpmp_contract": pct_contract,
        "camera_lidar_calibration_contract": camera_lidar_contract,
        "pct_golden_readiness": pct_golden_status,
        "pct_rust_process_golden_readiness": pct_rust_process_golden_status,
        "pct_rust_process_gn_golden_readiness": pct_rust_process_gn_golden_status,
        "pct_gpmp_math_readiness": pct_gpmp_math_status,
        "pct_gpmp_optimizer_compare_readiness": pct_gpmp_optimizer_compare_status,
        "pct_rust_runtime_acceptance_readiness": pct_rust_runtime_acceptance_status,
        "pct_native_rust_parity_readiness": pct_native_rust_parity_status,
        "camera_lidar_abi_smoke_readiness": camera_lidar_abi_smoke_status,
        "blockers": blockers,
        "warnings": warnings,
    }


def print_text_report(status: dict[str, Any]) -> None:
    print(f"Kernel migration status: {'OK' if status['ok'] else 'FAILED'}")
    print(f"  schema: {status['schema']}")
    print(f"  PGO/HBA claim: {status['claims']['pgo_hba_pose_graph_opt_replacement']}")
    print(
        "  PCT/GPMP obstacle+optimizer kernel: "
        f"{status['claims']['pct_gpmp_obstacle_optimizer_kernel']}"
    )
    print(
        "  PCT/GPMP Rust process runtime: "
        f"{status['claims']['pct_gpmp_rust_process_runtime']}"
    )
    print(
        "  PCT/GPMP Docker GTSAM-free packaging: "
        f"{status['claims']['pct_gpmp_docker_gtsam_free']}"
    )
    print(
        "  PCT/GPMP server setup default Rust runtime: "
        f"{status['claims']['pct_gpmp_server_setup_default_rust']}"
    )
    print(
        "  PCT/GPMP optimizer performance: "
        f"{status['claims']['pct_gpmp_optimizer_performance']}"
    )
    print(f"  PCT native/rust parity: {status['claims']['pct_native_rust_parity']}")
    print(
        "  camera-LiDAR CT-ICP kernel: "
        f"{status['claims']['camera_lidar_ct_icp_kernel']}"
    )
    print(
        "  camera-LiDAR CT-GICP kernel: "
        f"{status['claims']['camera_lidar_ct_gicp_kernel']}"
    )
    print(f"  camera-LiDAR C ABI: {status['claims']['camera_lidar_c_abi']}")
    print(
        "  camera-LiDAR C++ Rust ABI bridge: "
        f"{status['claims']['camera_lidar_cpp_rust_abi_bridge']}"
    )
    print(
        "  camera-LiDAR dynamic integrator Rust runtime: "
        f"{status['claims']['camera_lidar_dynamic_integrator_rust_runtime']}"
    )
    print(
        "  camera-LiDAR default Rust-required build: "
        f"{status['claims']['camera_lidar_default_rust_required']}"
    )
    print(
        "  camera-LiDAR default library source GTSAM-free: "
        f"{status['claims']['camera_lidar_default_library_source_gtsam_free']}"
    )
    print(
        "  camera-LiDAR legacy GTSAM factors removed: "
        f"{status['claims']['camera_lidar_legacy_gtsam_factors_removed']}"
    )
    print(
        "  camera-LiDAR Rust ABI smoke: "
        f"{status['claims']['camera_lidar_abi_smoke']}"
    )
    print(f"  full GTSAM replacement claim: {status['claims']['full_gtsam_replacement']}")
    print(f"  covered surfaces: {', '.join(status['pose_graph_opt']['covered_surfaces'])}")
    print(f"  remaining surfaces: {', '.join(status['remaining'].keys()) or 'none'}")
    print(f"  PCT contract: {'OK' if status['pct_gpmp_contract']['ok'] else 'FAILED'}")
    rust_kernel = status["pct_gpmp_contract"].get("rust_math_kernel") or {}
    if rust_kernel:
        print(f"  PCT/GPMP Rust math kernel: {rust_kernel.get('path', 'unknown')}")
    camera_kernel = status["camera_lidar_calibration_contract"].get("rust_kernel") or {}
    if camera_kernel:
        print(f"  camera-LiDAR Rust kernel: {camera_kernel.get('path', 'unknown')}")
    print(f"  PCT golden: {status['pct_golden_readiness']['status']}")
    print(f"  PCT rust_process golden: {status['pct_rust_process_golden_readiness']['status']}")
    print(
        "  PCT rust_process GN golden: "
        f"{status['pct_rust_process_gn_golden_readiness']['status']}"
    )
    print(f"  PCT/GPMP math compare: {status['pct_gpmp_math_readiness']['status']}")
    print(
        "  PCT/GPMP optimizer compare: "
        f"{status['pct_gpmp_optimizer_compare_readiness']['status']}"
    )
    print(
        "  PCT Rust runtime acceptance: "
        f"{status['pct_rust_runtime_acceptance_readiness']['status']}"
    )
    print(
        "  PCT native/rust parity: "
        f"{status['pct_native_rust_parity_readiness']['status']}"
    )
    print(
        "  camera-LiDAR ABI smoke: "
        f"{status['camera_lidar_abi_smoke_readiness']['status']}"
    )
    if status["blockers"]:
        print("  blockers:")
        for blocker in status["blockers"]:
            print(f"    - {blocker}")
    if status["warnings"]:
        print("  warnings:")
        for warning in status["warnings"]:
            print(f"    - {warning}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=ROOT_DIR)
    parser.add_argument("--pct-golden", type=Path, default=None)
    parser.add_argument("--pct-actual-json", type=Path, default=None)
    parser.add_argument("--pct-rust-process-golden", type=Path, default=None)
    parser.add_argument("--pct-rust-process-actual-json", type=Path, default=None)
    parser.add_argument("--pct-rust-process-gn-golden", type=Path, default=None)
    parser.add_argument("--pct-rust-process-gn-actual-json", type=Path, default=None)
    parser.add_argument("--pct-gpmp-math-rust-json", type=Path, default=None)
    parser.add_argument("--pct-gpmp-math-baseline-json", type=Path, default=None)
    parser.add_argument("--pct-gpmp-optimizer-compare-json", type=Path, default=None)
    parser.add_argument("--pct-rust-runtime-acceptance-json", type=Path, default=None)
    parser.add_argument("--pct-native-rust-parity-json", type=Path, default=None)
    parser.add_argument("--camera-lidar-abi-smoke-json", type=Path, default=None)
    parser.add_argument(
        "--require-pct-golden",
        action="store_true",
        help="fail unless --pct-actual-json exists and passes the PCT golden compare",
    )
    parser.add_argument(
        "--require-pct-rust-process-golden",
        action="store_true",
        help=(
            "fail unless --pct-rust-process-actual-json exists and passes "
            "the rust_process synthetic golden compare"
        ),
    )
    parser.add_argument(
        "--require-pct-rust-process-gn-golden",
        action="store_true",
        help=(
            "fail unless --pct-rust-process-gn-actual-json exists and passes "
            "the rust_process synthetic Gauss-Newton golden compare"
        ),
    )
    parser.add_argument(
        "--require-pct-gpmp-math",
        action="store_true",
        help=(
            "fail unless --pct-gpmp-math-rust-json and "
            "--pct-gpmp-math-baseline-json exist and compare cleanly"
        ),
    )
    parser.add_argument(
        "--require-pct-gpmp-optimizer-compare",
        action="store_true",
        help="fail unless --pct-gpmp-optimizer-compare-json exists and passes",
    )
    parser.add_argument(
        "--require-pct-native-rust-parity",
        action="store_true",
        help="fail unless --pct-native-rust-parity-json exists and passes",
    )
    parser.add_argument(
        "--require-camera-lidar-abi-smoke",
        action="store_true",
        help="fail unless --camera-lidar-abi-smoke-json exists and passes",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help=(
            "alias for --require-pct-golden, --require-pct-rust-process-golden, "
            "--require-pct-rust-process-gn-golden, --require-pct-gpmp-math, "
            "--require-pct-gpmp-optimizer-compare, and "
            "--require-pct-native-rust-parity, and "
            "--require-camera-lidar-abi-smoke"
        ),
    )
    parser.add_argument("--json", action="store_true", help="emit JSON instead of text")
    args = parser.parse_args(argv)

    status = build_status(
        args.repo_root,
        pct_golden=args.pct_golden,
        pct_actual_json=args.pct_actual_json,
        require_pct_golden=args.require_pct_golden or args.strict,
        pct_rust_process_golden=args.pct_rust_process_golden,
        pct_rust_process_actual_json=args.pct_rust_process_actual_json,
        require_pct_rust_process_golden=args.require_pct_rust_process_golden or args.strict,
        pct_rust_process_gn_golden=args.pct_rust_process_gn_golden,
        pct_rust_process_gn_actual_json=args.pct_rust_process_gn_actual_json,
        require_pct_rust_process_gn_golden=(
            args.require_pct_rust_process_gn_golden or args.strict
        ),
        pct_gpmp_math_rust_json=args.pct_gpmp_math_rust_json,
        pct_gpmp_math_baseline_json=args.pct_gpmp_math_baseline_json,
        require_pct_gpmp_math=args.require_pct_gpmp_math or args.strict,
        pct_gpmp_optimizer_compare_json=args.pct_gpmp_optimizer_compare_json,
        require_pct_gpmp_optimizer_compare=(
            args.require_pct_gpmp_optimizer_compare or args.strict
        ),
        pct_rust_runtime_acceptance_json=args.pct_rust_runtime_acceptance_json,
        pct_native_rust_parity_json=args.pct_native_rust_parity_json,
        require_pct_native_rust_parity=args.require_pct_native_rust_parity
        or args.strict,
        camera_lidar_abi_smoke_json=args.camera_lidar_abi_smoke_json,
        require_camera_lidar_abi_smoke=args.require_camera_lidar_abi_smoke
        or args.strict,
    )
    if args.json:
        print(json.dumps(status, indent=2, sort_keys=True))
    else:
        print_text_report(status)
    return 0 if status["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
