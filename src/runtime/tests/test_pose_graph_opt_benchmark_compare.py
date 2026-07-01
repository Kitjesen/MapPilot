from __future__ import annotations

import importlib.util
import pytest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "bench" / "pose_graph_opt_compare.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("pose_graph_opt_compare", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_pose_graph_opt_compare_cases_reports_time_and_cost_delta():
    module = _load_module()
    rust = {
        "cases": [
            {
                "case": "pgo_loop",
                "poses": 16,
                "elapsed_ms": 2.0,
                "final_cost": 1e-9,
                "status": 0,
                "converged": True,
                "written": 16,
                "final_residual_rms": 1e-6,
                "final_residual_max": 2e-6,
                "iterations": 4,
                "accepted_steps": 3,
            }
        ]
    }
    baseline = {
        "cases": [
            {
                "case": "pgo_loop",
                "poses": 16,
                "elapsed_ms": 1.0,
                "final_cost": 3e-9,
                "status": 0,
                "converged": True,
                "written": 16,
                "final_residual_rms": 3e-6,
                "final_residual_max": 5e-6,
                "iterations": 5,
                "accepted_steps": 4,
            }
        ]
    }

    comparison = module.compare_cases(rust, baseline)

    assert len(comparison) == 1
    item = comparison[0]
    assert item["case"] == "pgo_loop"
    assert item["poses"] == 16
    assert item["rust_elapsed_ms"] == 2.0
    assert item["baseline_elapsed_ms"] == 1.0
    assert item["rust_to_baseline_time_ratio"] == 2.0
    assert item["rust_final_cost"] == 1e-9
    assert item["baseline_final_cost"] == 3e-9
    assert item["final_cost_delta"] == pytest.approx(-2e-9)
    assert item["rust_status"] == 0
    assert item["baseline_status"] == 0
    assert item["rust_converged"] is True
    assert item["baseline_converged"] is True
    assert item["rust_written"] == 16
    assert item["baseline_written"] == 16
    assert item["rust_final_residual_rms"] == 1e-6
    assert item["baseline_final_residual_rms"] == 3e-6
    assert item["final_residual_rms_delta"] == pytest.approx(-2e-6)
    assert item["rust_final_residual_max"] == 2e-6
    assert item["baseline_final_residual_max"] == 5e-6
    assert item["final_residual_max_delta"] == pytest.approx(-3e-6)
    assert item["rust_iterations"] == 4
    assert item["baseline_iterations"] == 5
    assert item["rust_accepted_steps"] == 3
    assert item["baseline_accepted_steps"] == 4


def test_pose_graph_opt_compare_cases_reports_pose_rmse_when_available():
    module = _load_module()
    rust = {
        "cases": [
            {
                "case": "pgo_loop",
                "poses": 2,
                "elapsed_ms": 2.0,
                "final_cost": 0.0,
                "optimized_poses": [
                    {"t_xyz": [0.0, 0.0, 0.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0]},
                    {"t_xyz": [1.0, 0.0, 0.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0]},
                ],
            }
        ]
    }
    baseline = {
        "cases": [
            {
                "case": "pgo_loop",
                "poses": 2,
                "elapsed_ms": 1.0,
                "final_cost": 0.0,
                "optimized_poses": [
                    {"t_xyz": [0.0, 0.0, 0.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0]},
                    {
                        "t_xyz": [1.1, 0.0, 0.0],
                        "q_wxyz": [0.9999875000260416, 0.0, 0.0, 0.004999979166692708],
                    },
                ],
            }
        ]
    }

    comparison = module.compare_cases(rust, baseline)

    assert comparison[0]["pose_translation_rmse"] == pytest.approx((0.01 / 2) ** 0.5)
    assert comparison[0]["pose_rotation_rmse_rad"] == pytest.approx((0.01**2 / 2) ** 0.5)


def test_pose_graph_opt_compare_summary_passes_with_thresholds():
    module = _load_module()
    rust = {
        "cases": [
            {
                "case": "pgo_loop",
                "poses": 2,
                "factors": 2,
                "status": 0,
                "converged": True,
                "written": 2,
                "fixture_hash": "fixture-a",
                "elapsed_ms": 2.0,
                "final_cost": 1e-9,
                "final_residual_rms": 1e-7,
                "final_residual_max": 2e-7,
                "optimized_poses": [
                    {"t_xyz": [0.0, 0.0, 0.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0]},
                    {"t_xyz": [1.0, 0.0, 0.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0]},
                ],
            }
        ]
    }
    baseline = {
        "cases": [
            {
                "case": "pgo_loop",
                "poses": 2,
                "factors": 2,
                "status": 0,
                "converged": True,
                "written": 2,
                "fixture_hash": "fixture-a",
                "elapsed_ms": 1.0,
                "final_cost": 2e-9,
                "final_residual_rms": 2e-7,
                "final_residual_max": 4e-7,
                "optimized_poses": [
                    {"t_xyz": [0.0, 0.0, 0.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0]},
                    {"t_xyz": [1.0, 0.0, 0.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0]},
                ],
            }
        ]
    }

    comparison = module.compare_cases(rust, baseline)
    summary = module.summarize_comparison(
        rust,
        baseline,
        comparison,
        module.comparison_tolerances(),
    )

    assert summary["verdict"] == "pass"
    assert summary["matched_cases"] == [{"case": "pgo_loop", "poses": 2}]
    assert summary["missing_in_rust"] == []
    assert summary["missing_in_baseline"] == []
    assert summary["failed_checks"] == []
    assert {
        (check["metric"], check["status"])
        for check in summary["checks"]
        if check["metric"] in {"rust_status", "baseline_status", "factors_match"}
    } == {
        ("rust_status", "pass"),
        ("baseline_status", "pass"),
        ("factors_match", "pass"),
    }
    assert any(
        check["metric"] == "case_fixture_hash_match" and check["status"] == "pass"
        for check in summary["checks"]
    )


def test_pose_graph_opt_compare_summary_fails_threshold_and_missing_case():
    module = _load_module()
    rust = {
        "cases": [
            {
                "case": "pgo_loop",
                "poses": 2,
                "factors": 3,
                "status": 1,
                "converged": False,
                "written": 1,
                "fixture_hash": "rust-fixture",
                "elapsed_ms": 10.0,
                "final_cost": 5e-3,
            },
            {
                "case": "rust_only",
                "poses": 3,
                "elapsed_ms": 1.0,
                "final_cost": 0.0,
            },
        ]
    }
    baseline = {
        "cases": [
            {
                "case": "pgo_loop",
                "poses": 2,
                "factors": 2,
                "status": 0,
                "converged": True,
                "written": 2,
                "fixture_hash": "baseline-fixture",
                "elapsed_ms": 1.0,
                "final_cost": 0.0,
            },
            {
                "case": "baseline_only",
                "poses": 4,
                "elapsed_ms": 1.0,
                "final_cost": 0.0,
            },
        ]
    }

    comparison = module.compare_cases(rust, baseline)
    summary = module.summarize_comparison(
        rust,
        baseline,
        comparison,
        module.comparison_tolerances(
            overrides={"rust_to_baseline_time_ratio_max": 2.0},
        ),
    )

    assert summary["verdict"] == "fail"
    assert summary["missing_in_rust"] == [{"case": "baseline_only", "poses": 4}]
    assert summary["missing_in_baseline"] == [{"case": "rust_only", "poses": 3}]
    failed_metrics = {check["metric"] for check in summary["failed_checks"]}
    assert "final_cost_delta" in failed_metrics
    assert "rust_to_baseline_time_ratio" in failed_metrics
    assert "rust_status" in failed_metrics
    assert "rust_converged" in failed_metrics
    assert "rust_written" in failed_metrics
    assert "factors_match" in failed_metrics
    assert "case_fixture_hash_match" in failed_metrics
    assert "missing_in_rust" in failed_metrics
    assert "missing_in_baseline" in failed_metrics


def test_pose_graph_opt_compare_summary_fails_when_pose_outputs_are_missing():
    module = _load_module()
    rust = {
        "cases": [
            {
                "case": "pgo_loop",
                "poses": 2,
                "factors": 1,
                "status": 0,
                "converged": True,
                "written": 2,
                "fixture_hash": "same-fixture",
                "elapsed_ms": 1.0,
                "final_cost": 0.0,
                "final_residual_rms": 0.0,
                "final_residual_max": 0.0,
            }
        ]
    }
    baseline = {
        "cases": [
            {
                "case": "pgo_loop",
                "poses": 2,
                "factors": 1,
                "status": 0,
                "converged": True,
                "written": 2,
                "fixture_hash": "same-fixture",
                "elapsed_ms": 1.0,
                "final_cost": 0.0,
                "final_residual_rms": 0.0,
                "final_residual_max": 0.0,
            }
        ]
    }

    summary = module.summarize_comparison(
        rust,
        baseline,
        module.compare_cases(rust, baseline),
        module.comparison_tolerances(),
    )

    assert summary["verdict"] == "fail"
    failed_metrics = {check["metric"] for check in summary["failed_checks"]}
    assert "rust_optimized_pose_count" in failed_metrics
    assert "baseline_optimized_pose_count" in failed_metrics


def test_pose_graph_opt_compare_summary_checks_fixture_identity():
    module = _load_module()
    rust = {
        "fixture_hash": "rust-hash",
        "cases": [
            {"case": "pgo_loop", "poses": 2, "elapsed_ms": 1.0, "final_cost": 0.0}
        ],
    }
    baseline = {
        "fixture_hash": "baseline-hash",
        "cases": [
            {"case": "pgo_loop", "poses": 2, "elapsed_ms": 1.0, "final_cost": 0.0}
        ],
    }

    summary = module.summarize_comparison(
        rust,
        baseline,
        module.compare_cases(rust, baseline),
        module.comparison_tolerances(),
        expected_fixture_hash="expected-hash",
    )

    failed_metrics = {check["metric"] for check in summary["failed_checks"]}
    assert summary["fixture_hashes"] == {
        "expected": "expected-hash",
        "rust": "rust-hash",
        "baseline": "baseline-hash",
    }
    assert {
        "rust_fixture_hash",
        "baseline_fixture_hash",
        "rust_baseline_fixture_hash_match",
    } <= failed_metrics


def test_pose_graph_opt_compare_tolerances_use_fixture_then_cli_overrides():
    module = _load_module()
    fixture = {
        "baseline_tolerances": {
            "final_cost_delta_abs_max": 0.5,
            "pose_translation_rmse_max": 0.25,
        }
    }

    tolerances = module.comparison_tolerances(
        fixture=fixture,
        overrides={
            "final_cost_delta_abs_max": 0.1,
            "pose_rotation_rmse_rad_max": 0.05,
        },
    )

    assert tolerances["final_cost_delta_abs_max"] == 0.1
    assert tolerances["pose_translation_rmse_max"] == 0.25
    assert tolerances["pose_rotation_rmse_rad_max"] == 0.05


def test_pose_graph_opt_compare_suite_uses_per_fixture_hashes_and_tolerances(tmp_path):
    module = _load_module()
    fixture = {
        "case": "pgo_loop",
        "poses": [
            {"t_xyz": [0.0, 0.0, 0.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0]},
            {"t_xyz": [1.0, 0.0, 0.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0]},
        ],
        "priors": [],
        "betweens": [],
        "baseline_tolerances": {"final_cost_delta_abs_max": 0.01},
    }
    fixture_path = tmp_path / "pgo_loop_2.json"
    fixture_path.write_text(module.json.dumps(fixture), encoding="utf-8")
    expected_hash = module.fixture_hash(fixture)
    expected_hashes, tolerances_by_case, fixture_records = (
        module.load_suite_fixture_expectations(tmp_path)
    )
    rust = {
        "cases": [
            {
                "case": "pgo_loop",
                "poses": 2,
                "factors": 0,
                "fixture_hash": expected_hash,
                "status": 0,
                "converged": True,
                "written": 2,
                "elapsed_ms": 1.0,
                "final_cost": 0.005,
                "optimized_poses": fixture["poses"],
            }
        ]
    }
    baseline = {
        "cases": [
            {
                "case": "pgo_loop",
                "poses": 2,
                "factors": 0,
                "fixture_hash": expected_hash,
                "status": 0,
                "converged": True,
                "written": 2,
                "elapsed_ms": 1.0,
                "final_cost": 0.0,
                "optimized_poses": fixture["poses"],
            }
        ]
    }

    summary = module.summarize_comparison(
        rust,
        baseline,
        module.compare_cases(rust, baseline),
        module.comparison_tolerances(),
        expected_fixture_hashes_by_case=expected_hashes,
        tolerances_by_case=tolerances_by_case,
    )

    assert fixture_records == [
        {"path": fixture_path.as_posix(), "case": "pgo_loop", "poses": 2, "fixture_hash": expected_hash}
    ]
    assert summary["verdict"] == "pass"
    assert summary["tolerances_by_case"][0]["tolerances"]["final_cost_delta_abs_max"] == 0.01
    identity_metrics = {
        (check["metric"], check["status"])
        for check in summary["identity_checks"]
        if check["metric"] in {"rust_case_fixture_hash", "baseline_case_fixture_hash"}
    }
    assert identity_metrics == {
        ("rust_case_fixture_hash", "pass"),
        ("baseline_case_fixture_hash", "pass"),
    }


def test_pose_graph_opt_compare_cli_accepts_precomputed_rust_result():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "--rust-result" in source
    assert "--fixture" in source
    assert "compare-suite" in source
    assert "--fixture-dir" in source
    assert "--enforce" in source
    assert "json.loads(args.rust_result.read_text" in source
    assert "else run_rust_benchmark(args.poses)" in source
    assert "summarize_comparison" in source
