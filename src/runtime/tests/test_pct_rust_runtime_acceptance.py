from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "bench" / "pct_rust_runtime_acceptance.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("pct_rust_runtime_acceptance", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _passing_smoke(nonlinear_optimizer: str) -> dict:
    return {
        "nonlinear_optimizer": nonlinear_optimizer,
        "actual": {
            "ok": True,
            "path_count": 91,
            "goal_error_m": 0.0,
            "runtime": {"backend": "rust_process"},
            "diagnostics": {
                "last_optimizer_nonlinear_optimizer": nonlinear_optimizer,
                "last_optimizer_linear_solver": "block_tridiagonal",
                "last_optimizer_linear_solve_fallbacks": 0,
                "last_optimizer_call_mode": "ffi",
            },
        },
        "comparison": {"verdict": "pass"},
    }


def _passing_payload(module) -> dict:
    return {
        "runtime_smokes": {
            "levenberg_marquardt": _passing_smoke("levenberg_marquardt"),
            "gauss_newton": _passing_smoke("gauss_newton"),
        },
        "optimizer_compare": {
            "payload": {
                "summary": {
                    "verdict": "pass",
                    "failed_checks": [],
                }
            }
        },
        "migration_status": {
            "ok": True,
            "claims": {
                claim: True for claim in module.REQUIRED_STATUS_CLAIMS
            },
        },
    }


def test_evaluate_acceptance_requires_runtime_golden_performance_and_status_claims() -> None:
    module = _load_module()
    payload = _passing_payload(module)

    result = module.evaluate_acceptance(payload)

    assert result["verdict"] == "pass"
    assert result["failed_checks"] == []

    payload["runtime_smokes"]["gauss_newton"]["actual"]["diagnostics"][
        "last_optimizer_linear_solve_fallbacks"
    ] = 1
    payload["optimizer_compare"]["payload"]["summary"]["verdict"] = "fail"
    payload["migration_status"]["claims"]["pct_gpmp_optimizer_performance"] = False

    result = module.evaluate_acceptance(payload)

    assert result["verdict"] == "fail"
    assert "gauss_newton:linear_solver_fallback" in result["failed_checks"]
    assert "optimizer_compare_failed" in result["failed_checks"]
    assert (
        "migration_status_claim_false:pct_gpmp_optimizer_performance"
        in result["failed_checks"]
    )


def test_artifact_path_accepts_repo_relative_and_absolute_paths() -> None:
    module = _load_module()
    relative = "tools/bench/pct_rust_process_smoke.py"
    absolute = SCRIPT.resolve()

    assert module._artifact_path(relative) == ROOT / relative
    assert module._artifact_path(str(absolute)) == absolute


def test_parser_defaults_run_lm_gn_runtime_and_performance_gate() -> None:
    module = _load_module()
    args = module.build_parser().parse_args([])

    assert args.linear_solver == "block_tridiagonal"
    assert args.state_count == 96
    assert args.repeats == 3
    assert args.min_speedup == 1.25
