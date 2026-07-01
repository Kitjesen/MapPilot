from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "bench" / "pct_gpmp_optimizer_compare.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("pct_gpmp_optimizer_compare", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_request_matches_wnoj_and_wnoa_state_shapes() -> None:
    module = _load_module()

    wnoj = module.build_request(
        mode="wnoj",
        state_count=8,
        linear_solver="block_tridiagonal",
        nonlinear_optimizer="levenberg_marquardt",
        max_iterations=5,
    )
    wnoa = module.build_request(
        mode="wnoa",
        state_count=8,
        linear_solver="dense",
        nonlinear_optimizer="gauss_newton",
        max_iterations=5,
    )

    assert wnoj["mode"] == "wnoj"
    assert len(wnoj["states"]) == 8
    assert len(wnoj["states"][0]) == 6
    assert wnoj["config"]["linear_solver"] == "block_tridiagonal"
    assert wnoj["config"]["nonlinear_optimizer"] == "levenberg_marquardt"
    assert wnoa["mode"] == "wnoa"
    assert len(wnoa["states"][0]) == 4
    assert wnoa["config"]["linear_solver"] == "dense"
    assert wnoa["config"]["nonlinear_optimizer"] == "gauss_newton"


def test_evaluate_requires_sparse_solver_speedup_and_cost_match() -> None:
    module = _load_module()
    payload = {
        "cases": [
            {
                "mode": "wnoj",
                "nonlinear_optimizer": "levenberg_marquardt",
                "state_count": 96,
                "speedup": 1.5,
                "final_cost_delta_abs": 1e-9,
                "dense": {
                    "reported_linear_solver": "dense",
                    "reported_nonlinear_optimizer": "levenberg_marquardt",
                },
                "block_tridiagonal": {
                    "reported_linear_solver": "block_tridiagonal",
                    "reported_nonlinear_optimizer": "levenberg_marquardt",
                    "linear_solve_fallbacks": 0,
                },
            },
            {
                "mode": "wnoa",
                "nonlinear_optimizer": "levenberg_marquardt",
                "state_count": 96,
                "speedup": 1.5,
                "final_cost_delta_abs": 1e-9,
                "dense": {
                    "reported_linear_solver": "dense",
                    "reported_nonlinear_optimizer": "levenberg_marquardt",
                },
                "block_tridiagonal": {
                    "reported_linear_solver": "block_tridiagonal",
                    "reported_nonlinear_optimizer": "levenberg_marquardt",
                    "linear_solve_fallbacks": 0,
                },
            },
            {
                "mode": "wnoj",
                "nonlinear_optimizer": "gauss_newton",
                "state_count": 96,
                "speedup": 1.5,
                "final_cost_delta_abs": 1e-9,
                "dense": {
                    "reported_linear_solver": "dense",
                    "reported_nonlinear_optimizer": "gauss_newton",
                },
                "block_tridiagonal": {
                    "reported_linear_solver": "block_tridiagonal",
                    "reported_nonlinear_optimizer": "gauss_newton",
                    "linear_solve_fallbacks": 0,
                },
            },
            {
                "mode": "wnoa",
                "nonlinear_optimizer": "gauss_newton",
                "state_count": 96,
                "speedup": 1.5,
                "final_cost_delta_abs": 1e-9,
                "dense": {
                    "reported_linear_solver": "dense",
                    "reported_nonlinear_optimizer": "gauss_newton",
                },
                "block_tridiagonal": {
                    "reported_linear_solver": "block_tridiagonal",
                    "reported_nonlinear_optimizer": "gauss_newton",
                    "linear_solve_fallbacks": 0,
                },
            },
        ]
    }

    assert module.evaluate(
        payload,
        min_speedup=1.25,
        final_cost_abs_tol=1e-6,
    ) == {
        "verdict": "pass",
        "failed_checks": [],
        "min_speedup": 1.25,
        "final_cost_abs_tol": 1e-6,
        "required_nonlinear_optimizers": [
            "levenberg_marquardt",
            "gauss_newton",
        ],
    }

    payload["cases"][0]["speedup"] = 1.0
    payload["cases"][0]["block_tridiagonal"]["linear_solve_fallbacks"] = 1
    result = module.evaluate(payload, min_speedup=1.25, final_cost_abs_tol=1e-6)

    assert result["verdict"] == "fail"
    assert "wnoj:levenberg_marquardt:96:speedup_below_threshold" in result["failed_checks"]
    assert "wnoj:levenberg_marquardt:96:block_solver_fallbacks" in result["failed_checks"]


def test_evaluate_requires_lm_and_gn_for_both_state_models() -> None:
    module = _load_module()
    payload = {
        "cases": [
            {
                "mode": "wnoj",
                "nonlinear_optimizer": "levenberg_marquardt",
                "state_count": 96,
                "speedup": 1.5,
                "final_cost_delta_abs": 0.0,
                "dense": {
                    "reported_linear_solver": "dense",
                    "reported_nonlinear_optimizer": "levenberg_marquardt",
                },
                "block_tridiagonal": {
                    "reported_linear_solver": "block_tridiagonal",
                    "reported_nonlinear_optimizer": "levenberg_marquardt",
                    "linear_solve_fallbacks": 0,
                },
            }
        ]
    }

    result = module.evaluate(payload, min_speedup=1.25, final_cost_abs_tol=1e-6)

    assert result["verdict"] == "fail"
    assert "wnoa:levenberg_marquardt:missing_case" in result["failed_checks"]
    assert "wnoj:gauss_newton:missing_case" in result["failed_checks"]
    assert "wnoa:gauss_newton:missing_case" in result["failed_checks"]
