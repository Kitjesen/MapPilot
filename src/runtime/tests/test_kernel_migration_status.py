from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
STATUS_SCRIPT = ROOT / "tools" / "validate" / "validate_kernel_migration_status.py"
GOLDEN = (
    ROOT
    / "src"
    / "nav"
    / "tests"
    / "planning_backends"
    / "fixtures"
    / "pct_preview"
    / "building2_9_smoke.json"
)
RUST_PROCESS_GOLDEN = (
    ROOT
    / "src"
    / "nav"
    / "tests"
    / "planning_backends"
    / "fixtures"
    / "pct_preview"
    / "rust_process_synthetic_smoke.json"
)
RUST_PROCESS_GN_GOLDEN = (
    ROOT
    / "src"
    / "nav"
    / "tests"
    / "planning_backends"
    / "fixtures"
    / "pct_preview"
    / "rust_process_synthetic_gn_smoke.json"
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "validate_kernel_migration_status",
        STATUS_SCRIPT,
    )
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _passing_pct_actual() -> dict:
    return {
        "schema": "lingtu.pct.preview.actual.v2",
        "ok": True,
        "planner": "pct",
        "path_count": 55,
        "path_distance_m": 17.82,
        "start": [2.0, 3.0, 0.0],
        "goal": [18.0, 11.0, 0.0],
        "first": [2.0, 3.0, 0.0],
        "last": [17.9, 11.05, 0.0],
        "goal_error_m": 0.12,
        "input": {
            "tomogram": "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/tomogram/building2_9.pickle",
            "tomogram_sha256": "cd75fdf87120f7a9da4a8208687473199702e6871d9e5f225f6ba6784f5637ee",
            "tomogram_size_bytes": 638628,
            "tomogram_data_shape": [5, 7, 97, 94],
            "tomogram_data_dtype": "float16",
            "obstacle_thr": 49.9,
            "start": [2.0, 3.0, 0.0],
            "goal": [18.0, 11.0, 0.0],
        },
        "path_samples": [
            {"index": 0, "point": [2.0, 3.0, 0.0]},
            {"index": 54, "point": [17.9, 11.05, 0.0]},
        ],
        "path": {
            "shape": [55, 3],
            "finite": True,
            "count": 55,
            "distance_m": 17.82,
            "goal_error_m": 0.12,
            "samples": {"by_index": []},
        },
        "diagnostics": {
            "last_path_mode": "optimized_trajectory",
            "last_optimizer_enabled": True,
            "last_optimizer_attempted": True,
        },
        "runtime": {"arch": "x86_64", "python": "py310", "lib_dir": "/native"},
    }


def _passing_pct_gpmp_math_result() -> dict:
    return {
        "schema": "lingtu.pct_gpmp_math.result.v1",
        "producer": "test",
        "cases": [
            {
                "case": "wnoj_positive_heading_rate",
                "mode": "wnoj",
                "state_order": ["x", "vx", "ax", "y", "vy", "ay"],
                "input": {
                    "qc": 0.1,
                    "delta": 1.0,
                    "tau": 0.35,
                    "x1": [0.2, 1.1, -0.4, -0.3, 0.7, 1.8],
                },
                "output": {
                    "prior_residual": [0.1, 0.2],
                    "heading_rate_residual": 0.5,
                },
            },
            {
                "case": "wnoj_negative_heading_rate",
                "mode": "wnoj",
                "state_order": ["x", "vx", "ax", "y", "vy", "ay"],
                "input": {
                    "qc": 0.1,
                    "delta": 1.0,
                    "tau": 0.35,
                    "x1": [0.2, 1.1, 0.4, -0.3, 0.7, -1.8],
                },
                "output": {
                    "prior_residual": [0.1, 0.2],
                    "heading_rate_residual": 0.5,
                },
            },
            {
                "case": "wnoj_heading_rate_inside_limit",
                "mode": "wnoj",
                "state_order": ["x", "vx", "ax", "y", "vy", "ay"],
                "input": {
                    "qc": 0.1,
                    "delta": 1.0,
                    "tau": 0.35,
                    "x1": [0.2, 1.1, 0.0, -0.3, 0.7, 0.0],
                },
                "output": {
                    "prior_residual": [0.1, 0.2],
                    "heading_rate_residual": 0.0,
                },
            },
            {
                "case": "wnoa_nominal",
                "mode": "wnoa",
                "state_order": ["x", "vx", "y", "vy"],
                "input": {"qc": 0.1, "delta": 1.0, "tau": 0.35},
                "output": {"prior_residual": [0.1]},
            },
        ],
    }


def _passing_pct_gpmp_optimizer_compare_result() -> dict:
    cases = []
    for mode, speedup in (("wnoj", 1.6), ("wnoa", 2.1)):
        for nonlinear_optimizer in ("levenberg_marquardt", "gauss_newton"):
            cases.append(
                {
                    "mode": mode,
                    "nonlinear_optimizer": nonlinear_optimizer,
                    "state_count": 96,
                    "speedup": speedup,
                    "final_cost_delta_abs": 0.0,
                    "dense": {
                        "reported_linear_solver": "dense",
                        "reported_nonlinear_optimizer": nonlinear_optimizer,
                        "linear_solve_fallbacks": 0,
                    },
                    "block_tridiagonal": {
                        "reported_linear_solver": "block_tridiagonal",
                        "reported_nonlinear_optimizer": nonlinear_optimizer,
                        "linear_solve_fallbacks": 0,
                    },
                }
            )
    return {
        "schema": "lingtu.pct_gpmp_optimizer_compare.v1",
        "optimizer_bin": "gpmp_optimize",
        "required_nonlinear_optimizers": [
            "levenberg_marquardt",
            "gauss_newton",
        ],
        "cases": cases,
        "summary": {
            "verdict": "pass",
            "failed_checks": [],
            "min_speedup": 1.25,
            "final_cost_abs_tol": 1e-6,
            "required_nonlinear_optimizers": [
                "levenberg_marquardt",
                "gauss_newton",
            ],
        },
    }


def _passing_pct_rust_runtime_acceptance_result(
    *,
    lm_actual_json: Path,
    gn_actual_json: Path,
    optimizer_compare_json: Path,
) -> dict:
    return {
        "schema": "lingtu.pct_rust_runtime_acceptance.v1",
        "optimizer_bin": "target/release/gpmp_optimize.exe",
        "work_dir": str(lm_actual_json.parent),
        "runtime_smokes": {
            "levenberg_marquardt": {
                "nonlinear_optimizer": "levenberg_marquardt",
                "linear_solver": "block_tridiagonal",
                "actual_json": str(lm_actual_json),
                "golden": str(RUST_PROCESS_GOLDEN),
                "actual": _passing_pct_rust_process_actual(),
                "comparison": {"verdict": "pass", "failed_checks": []},
                "summary": {
                    "ok": True,
                    "comparison": "pass",
                    "backend": "rust_process",
                    "call_mode": "ffi",
                    "path_count": 91,
                    "goal_error_m": 0.0000039,
                    "fallbacks": 0,
                },
            },
            "gauss_newton": {
                "nonlinear_optimizer": "gauss_newton",
                "linear_solver": "block_tridiagonal",
                "actual_json": str(gn_actual_json),
                "golden": str(RUST_PROCESS_GN_GOLDEN),
                "actual": _passing_pct_rust_process_gn_actual(),
                "comparison": {"verdict": "pass", "failed_checks": []},
                "summary": {
                    "ok": True,
                    "comparison": "pass",
                    "backend": "rust_process",
                    "call_mode": "ffi",
                    "path_count": 91,
                    "goal_error_m": 0.0000039,
                    "fallbacks": 0,
                },
            },
        },
        "optimizer_compare": {
            "json": str(optimizer_compare_json),
            "payload": _passing_pct_gpmp_optimizer_compare_result(),
        },
        "migration_status": {
            "ok": True,
            "claim_allowed": False,
            "claims": {
                "pct_gpmp_obstacle_optimizer_kernel": True,
                "pct_gpmp_rust_process_runtime": True,
                "pct_rust_process_golden_parity": True,
                "pct_rust_process_gn_golden_parity": True,
                "pct_gpmp_optimizer_performance": True,
            },
        },
        "summary": {
            "verdict": "pass",
            "failed_checks": [],
            "required_status_claims": [
                "pct_gpmp_obstacle_optimizer_kernel",
                "pct_gpmp_rust_process_runtime",
                "pct_rust_process_golden_parity",
                "pct_rust_process_gn_golden_parity",
                "pct_gpmp_optimizer_performance",
            ],
        },
    }


def _passing_pct_native_rust_parity_result() -> dict:
    return {
        "schema": "lingtu.pct.native_rust_parity.v1",
        "tomogram": "building2_9.pickle",
        "start": [2.0, 3.0, 0.0],
        "goal": [18.0, 11.0, 0.0],
        "native": {"ok": True},
        "rust_process": {"ok": True},
        "summary": {
            "verdict": "pass",
            "failed_checks": [],
            "checks": [],
            "tolerances": {
                "path_count_delta_max": 8,
                "path_distance_abs_tol": 2.0,
                "goal_error_abs_tol": 1.0,
                "sample_abs_tol": 3.0,
            },
        },
    }


def _passing_pct_rust_process_actual() -> dict:
    return {
        "schema": "lingtu.pct.preview.actual.v2",
        "ok": True,
        "planner": "pct",
        "path_count": 91,
        "path_distance_m": 10.579477,
        "start": [-5.0, 0.0, 0.0],
        "goal": [5.0, 3.0, 0.0],
        "first": [-5.0, 0.0000039, 0.0],
        "last": [5.0, 2.9999961, 0.0],
        "goal_error_m": 0.0000039,
        "input": {
            "tomogram": "/tmp/lingtu_pct_rust_process/tomogram.pickle",
            "tomogram_data_shape": [5, 1, 13, 15],
            "tomogram_data_dtype": "float32",
            "obstacle_thr": 49.9,
            "start": [-5.0, 0.0, 0.0],
            "goal": [5.0, 3.0, 0.0],
        },
        "path_samples": [
            {"index": 0, "point": [-5.0, 0.0000039, 0.0]},
            {"index": 45, "point": [0.0, 1.5000008, 0.0]},
            {"index": 90, "point": [5.0, 2.9999961, 0.0]},
        ],
        "path": {
            "shape": [91, 3],
            "finite": True,
            "count": 91,
            "distance_m": 10.579477,
            "goal_error_m": 0.0000039,
            "samples": {"by_index": []},
        },
        "diagnostics": {
            "last_path_mode": "rust_optimized_trajectory",
            "last_optimizer_enabled": True,
            "last_optimizer_attempted": True,
            "last_optimizer_accepted": True,
            "last_optimizer_nonlinear_optimizer": "levenberg_marquardt",
            "last_optimizer_linear_solver": "block_tridiagonal",
            "last_optimizer_linear_solve_fallbacks": 0,
            "last_optimizer_input_states": 11,
            "last_optimizer_output_states": 91,
            "last_optimizer_trajectory_expanded": True,
            "last_optimizer_interpolation_steps": 8,
            "last_optimizer_call_mode": "ffi",
            "last_raw_path_count": 11,
        },
        "optimizer_accessors": {
            "available": True,
            "source": "planner",
            "native_wrapper_compatible": True,
            "finite": True,
            "row_count": 91,
            "state_dim": 6,
            "result_matrix_shape": [91, 6],
            "layers_shape": [91],
            "heights_shape": [91],
            "ceilings_shape": [91],
            "opt_init_value_shape": [6, 91],
            "opt_init_layer_shape": [91],
            "heading_rate_shape": [91],
        },
        "runtime": {
            "backend": "rust_process",
            "native_binary_format": "rust_cdylib",
            "rust_optimizer_call_mode": "ffi",
            "arch": "x86_64",
            "python": "py313",
            "lib_dir": "/rust",
        },
    }


def _passing_pct_rust_process_gn_actual() -> dict:
    payload = json.loads(json.dumps(_passing_pct_rust_process_actual()))
    payload["diagnostics"]["last_optimizer_nonlinear_optimizer"] = "gauss_newton"
    payload["diagnostics"]["last_optimizer_iterations"] = 2
    payload["diagnostics"]["last_optimizer_accepted_steps"] = 1
    return payload


def _passing_camera_lidar_abi_smoke_result() -> dict:
    checks = {
        "abi_version": True,
        "pose3_size": True,
        "ct_gicp_size": True,
        "ct_gicp_source_size": True,
        "ct_gicp_target_size": True,
        "linearization_size": True,
        "optimizer_config_size": True,
        "optimizer_result_size": True,
        "return_code": True,
        "used_correspondences": True,
        "cost": True,
        "gradient": True,
        "rhs": True,
        "hessian": True,
        "optimizer_return_code": True,
        "optimizer_used_correspondences": True,
        "optimizer_cost_reduction": True,
        "optimizer_final_cost_finite": True,
        "optimizer_pose0_translation": True,
        "dynamic_optimizer_return_code": True,
        "dynamic_optimizer_used_correspondences": True,
        "dynamic_optimizer_cost_reduction": True,
        "dynamic_optimizer_final_cost_finite": True,
        "dynamic_optimizer_pose0_translation": True,
    }
    return {
        "schema": "lingtu.camera_lidar_optimizer_abi_smoke.v1",
        "ok": True,
        "library": "target/debug/lingtu_camera_lidar_optimizer.dll",
        "checks": checks,
        "result": {
            "abi_version": 2,
            "return_code": 0,
            "cost": 2.0,
            "used_correspondences": 1,
            "gradient_tx0": 4.0,
            "rhs_tx0": -4.0,
            "hessian_tx0_tx0": 4.0,
            "optimizer_return_code": 0,
            "optimizer_initial_cost": 7.0,
            "optimizer_final_cost": 0.0,
            "optimizer_pose0_translation": [1.0, 2.0, 3.0],
            "dynamic_optimizer_return_code": 0,
            "dynamic_optimizer_initial_cost": 7.0,
            "dynamic_optimizer_final_cost": 0.0,
            "dynamic_optimizer_pose0_translation": [1.0, 2.0, 3.0],
        },
        "summary": {"ok": True, "failed_checks": []},
    }


def test_status_default_is_windows_safe_static_gate() -> None:
    module = _load_module()

    status = module.build_status(ROOT)

    assert status["schema"] == "lingtu.kernel_migration_status.v1"
    assert status["ok"] is True
    assert status["claims"]["pgo_hba_pose_graph_opt_replacement"] is True
    assert status["claims"]["pct_gpmp_math_kernel_parity"] is False
    assert status["claims"]["pct_gpmp_obstacle_optimizer_kernel"] is True
    assert status["claims"]["pct_gpmp_rust_process_runtime"] is True
    assert status["claims"]["pct_gpmp_docker_gtsam_free"] is True
    assert status["claims"]["pct_gpmp_server_setup_default_rust"] is True
    assert status["claims"]["pct_gpmp_optimizer_performance"] is False
    assert status["claims"]["pct_native_rust_parity"] is False
    assert status["claims"]["pct_rust_process_golden_parity"] is False
    assert status["claims"]["pct_rust_process_gn_golden_parity"] is False
    assert status["claims"]["camera_lidar_ct_icp_kernel"] is True
    assert status["claims"]["camera_lidar_ct_gicp_kernel"] is True
    assert status["claims"]["camera_lidar_c_abi"] is True
    assert status["claims"]["camera_lidar_cpp_rust_abi_bridge"] is True
    assert status["claims"]["camera_lidar_dynamic_integrator_rust_runtime"] is True
    assert status["claims"]["camera_lidar_default_rust_required"] is True
    assert status["claims"]["camera_lidar_default_library_source_gtsam_free"] is True
    assert status["claims"]["camera_lidar_docker_gtsam_free"] is True
    assert status["claims"]["camera_lidar_legacy_gtsam_factors_removed"] is True
    assert status["claims"]["camera_lidar_abi_smoke"] is False
    assert status["claims"]["full_gtsam_replacement"] is False
    assert set(status["pose_graph_opt"]["covered_surfaces"]) >= {"slam_pgo", "slam_hba"}
    assert status["pct_gpmp_contract"]["ok"] is True
    assert status["pct_gpmp_contract"]["rust_math_kernel"]["crate"] == (
        "lingtu_gpmp_trajectory_optimizer"
    )
    assert status["camera_lidar_calibration_contract"]["ok"] is True
    assert status["camera_lidar_calibration_contract"]["rust_kernel"]["crate"] == (
        "lingtu_camera_lidar_optimizer"
    )
    camera_lidar_capabilities = set(
        status["camera_lidar_calibration_contract"]["rust_kernel"]["covered_capabilities"]
    )
    assert "Rust-owned nearest-neighbor correspondence search and max-distance rejection" in (
        camera_lidar_capabilities
    )
    assert "C ABI for CT-GICP nearest-neighbor correspondence construction calls" in (
        camera_lidar_capabilities
    )
    assert "Rust-owned dynamic CT-GICP correspondence rebuild and two-pose optimization loop" in (
        camera_lidar_capabilities
    )
    assert "C ABI for dynamic CT-GICP scan optimization calls" in (
        camera_lidar_capabilities
    )
    assert "dynamic point cloud integrator delegates CT-GICP correspondence rebuild and optimization loop to one Rust C ABI call" in (
        camera_lidar_capabilities
    )
    assert "camera-LiDAR calibration Docker images use ROS base images, Rust/Cargo, and explicit Ceres/Iridescence builds without GTSAM" in (
        camera_lidar_capabilities
    )
    assert status["pct_golden_readiness"]["status"] == "not_requested"
    assert status["pct_rust_process_golden_readiness"]["status"] == "not_requested"
    assert status["pct_rust_process_gn_golden_readiness"]["status"] == "not_requested"
    assert status["pct_gpmp_math_readiness"]["status"] == "not_requested"
    assert status["pct_gpmp_optimizer_compare_readiness"]["status"] == "not_requested"
    assert status["pct_rust_runtime_acceptance_readiness"]["status"] == "not_requested"
    assert status["pct_native_rust_parity_readiness"]["status"] == "not_requested"
    assert status["camera_lidar_abi_smoke_readiness"]["status"] == "not_requested"


def test_status_requires_pct_actual_when_strict() -> None:
    module = _load_module()

    status = module.build_status(ROOT, require_pct_golden=True)

    assert status["ok"] is False
    assert status["pct_golden_readiness"]["status"] == "missing_actual"
    assert "pct_golden_actual_missing" in status["blockers"]


def test_status_can_include_pct_golden_compare(tmp_path: Path) -> None:
    module = _load_module()
    actual_path = tmp_path / "actual.json"
    actual_path.write_text(json.dumps(_passing_pct_actual()), encoding="utf-8")

    status = module.build_status(
        ROOT,
        pct_golden=GOLDEN,
        pct_actual_json=actual_path,
        require_pct_golden=True,
    )

    assert status["ok"] is True
    assert status["pct_golden_readiness"]["status"] == "pass"
    assert status["pct_golden_readiness"]["failed_checks"] == []
    assert status["claims"]["full_gtsam_replacement"] is False


def test_status_requires_pct_rust_process_actual_when_requested() -> None:
    module = _load_module()

    status = module.build_status(ROOT, require_pct_rust_process_golden=True)

    assert status["ok"] is False
    assert status["pct_rust_process_golden_readiness"]["status"] == "missing_actual"
    assert "pct_rust_process_golden_actual_missing" in status["blockers"]


def test_status_can_include_pct_rust_process_golden_compare(tmp_path: Path) -> None:
    module = _load_module()
    actual_path = tmp_path / "rust_process_actual.json"
    actual_path.write_text(
        json.dumps(_passing_pct_rust_process_actual()),
        encoding="utf-8",
    )

    status = module.build_status(
        ROOT,
        pct_rust_process_golden=RUST_PROCESS_GOLDEN,
        pct_rust_process_actual_json=actual_path,
        require_pct_rust_process_golden=True,
    )

    assert status["ok"] is True
    assert status["pct_rust_process_golden_readiness"]["status"] == "pass"
    assert status["pct_rust_process_golden_readiness"]["failed_checks"] == []
    assert status["claims"]["pct_rust_process_golden_parity"] is True
    assert status["claims"]["pct_rust_process_gn_golden_parity"] is False
    assert status["claims"]["full_gtsam_replacement"] is False


def test_status_requires_pct_rust_process_gn_actual_when_requested() -> None:
    module = _load_module()

    status = module.build_status(ROOT, require_pct_rust_process_gn_golden=True)

    assert status["ok"] is False
    assert status["pct_rust_process_gn_golden_readiness"]["status"] == "missing_actual"
    assert "pct_rust_process_gn_golden_actual_missing" in status["blockers"]


def test_status_can_include_pct_rust_process_gn_golden_compare(tmp_path: Path) -> None:
    module = _load_module()
    actual_path = tmp_path / "rust_process_gn_actual.json"
    actual_path.write_text(
        json.dumps(_passing_pct_rust_process_gn_actual()),
        encoding="utf-8",
    )

    status = module.build_status(
        ROOT,
        pct_rust_process_gn_golden=RUST_PROCESS_GN_GOLDEN,
        pct_rust_process_gn_actual_json=actual_path,
        require_pct_rust_process_gn_golden=True,
    )

    assert status["ok"] is True
    assert status["pct_rust_process_gn_golden_readiness"]["status"] == "pass"
    assert status["pct_rust_process_gn_golden_readiness"]["failed_checks"] == []
    assert status["claims"]["pct_rust_process_gn_golden_parity"] is True
    assert status["claims"]["pct_rust_process_golden_parity"] is False
    assert status["claims"]["full_gtsam_replacement"] is False


def test_status_requires_pct_gpmp_math_when_requested() -> None:
    module = _load_module()

    status = module.build_status(ROOT, require_pct_gpmp_math=True)

    assert status["ok"] is False
    assert status["pct_gpmp_math_readiness"]["status"] == "missing_compare_inputs"
    assert "pct_gpmp_math_compare_inputs_missing" in status["blockers"]
    assert status["claims"]["pgo_hba_pose_graph_opt_replacement"] is True
    assert status["claims"]["pct_gpmp_math_kernel_parity"] is False
    assert status["claims"]["pct_gpmp_obstacle_optimizer_kernel"] is True
    assert status["claims"]["pct_gpmp_rust_process_runtime"] is True
    assert status["claims"]["pct_gpmp_docker_gtsam_free"] is True
    assert status["claims"]["pct_gpmp_server_setup_default_rust"] is True
    assert status["claims"]["pct_gpmp_optimizer_performance"] is False
    assert status["claims"]["pct_native_rust_parity"] is False
    assert status["claims"]["camera_lidar_ct_icp_kernel"] is True
    assert status["claims"]["camera_lidar_ct_gicp_kernel"] is True
    assert status["claims"]["camera_lidar_c_abi"] is True
    assert status["claims"]["camera_lidar_cpp_rust_abi_bridge"] is True
    assert status["claims"]["camera_lidar_dynamic_integrator_rust_runtime"] is True
    assert status["claims"]["camera_lidar_default_rust_required"] is True
    assert status["claims"]["camera_lidar_default_library_source_gtsam_free"] is True
    assert status["claims"]["camera_lidar_docker_gtsam_free"] is True
    assert status["claims"]["camera_lidar_legacy_gtsam_factors_removed"] is True


def test_status_can_include_pct_gpmp_math_compare(tmp_path: Path) -> None:
    module = _load_module()
    rust_path = tmp_path / "rust.json"
    baseline_path = tmp_path / "baseline.json"
    payload = _passing_pct_gpmp_math_result()
    rust_path.write_text(json.dumps(payload), encoding="utf-8")
    baseline_path.write_text(json.dumps(payload), encoding="utf-8")

    status = module.build_status(
        ROOT,
        pct_gpmp_math_rust_json=rust_path,
        pct_gpmp_math_baseline_json=baseline_path,
        require_pct_gpmp_math=True,
    )

    assert status["ok"] is True
    assert status["pct_gpmp_math_readiness"]["status"] == "pass"
    assert status["pct_gpmp_math_readiness"]["failed_checks"] == []
    assert status["pct_gpmp_math_readiness"]["matched_cases"] == [
        "wnoa_nominal",
        "wnoj_heading_rate_inside_limit",
        "wnoj_negative_heading_rate",
        "wnoj_positive_heading_rate",
    ]
    assert status["claims"]["pct_gpmp_math_kernel_parity"] is True
    assert status["claims"]["pct_gpmp_obstacle_optimizer_kernel"] is True
    assert status["claims"]["pct_gpmp_rust_process_runtime"] is True
    assert status["claims"]["pct_gpmp_docker_gtsam_free"] is True
    assert status["claims"]["pct_gpmp_server_setup_default_rust"] is True
    assert status["claims"]["pct_gpmp_optimizer_performance"] is False
    assert status["claims"]["pct_native_rust_parity"] is False
    assert status["claims"]["camera_lidar_ct_icp_kernel"] is True
    assert status["claims"]["camera_lidar_ct_gicp_kernel"] is True
    assert status["claims"]["camera_lidar_c_abi"] is True
    assert status["claims"]["camera_lidar_cpp_rust_abi_bridge"] is True
    assert status["claims"]["camera_lidar_dynamic_integrator_rust_runtime"] is True
    assert status["claims"]["camera_lidar_default_rust_required"] is True
    assert status["claims"]["camera_lidar_default_library_source_gtsam_free"] is True
    assert status["claims"]["camera_lidar_docker_gtsam_free"] is True
    assert status["claims"]["camera_lidar_legacy_gtsam_factors_removed"] is True
    assert status["claims"]["full_gtsam_replacement"] is False


def test_status_rejects_pct_gpmp_math_without_required_cases(tmp_path: Path) -> None:
    module = _load_module()
    rust_path = tmp_path / "rust.json"
    baseline_path = tmp_path / "baseline.json"
    payload = _passing_pct_gpmp_math_result()
    payload["cases"] = [
        case
        for case in payload["cases"]
        if case["case"] in {"wnoj_positive_heading_rate", "wnoa_nominal"}
    ]
    rust_path.write_text(json.dumps(payload), encoding="utf-8")
    baseline_path.write_text(json.dumps(payload), encoding="utf-8")

    status = module.build_status(
        ROOT,
        pct_gpmp_math_rust_json=rust_path,
        pct_gpmp_math_baseline_json=baseline_path,
        require_pct_gpmp_math=True,
    )

    assert status["ok"] is False
    assert status["pct_gpmp_math_readiness"]["status"] == "fail"
    assert "pct_gpmp_math_compare_failed" in status["blockers"]
    failed = set(status["pct_gpmp_math_readiness"]["failed_checks"])
    assert "wnoj_negative_heading_rate:missing_required_case" in failed
    assert "wnoj_heading_rate_inside_limit:missing_required_case" in failed


def test_status_requires_pct_gpmp_optimizer_compare_when_requested() -> None:
    module = _load_module()

    status = module.build_status(ROOT, require_pct_gpmp_optimizer_compare=True)

    assert status["ok"] is False
    assert status["pct_gpmp_optimizer_compare_readiness"]["status"] == "missing_result"
    assert "pct_gpmp_optimizer_compare_missing" in status["blockers"]
    assert status["claims"]["pct_gpmp_optimizer_performance"] is False


def test_status_can_include_pct_gpmp_optimizer_compare(tmp_path: Path) -> None:
    module = _load_module()
    compare_path = tmp_path / "optimizer_compare.json"
    compare_path.write_text(
        json.dumps(_passing_pct_gpmp_optimizer_compare_result()),
        encoding="utf-8",
    )

    status = module.build_status(
        ROOT,
        pct_gpmp_optimizer_compare_json=compare_path,
        require_pct_gpmp_optimizer_compare=True,
    )

    assert status["ok"] is True
    assert status["pct_gpmp_optimizer_compare_readiness"]["status"] == "pass"
    assert status["pct_gpmp_optimizer_compare_readiness"]["failed_checks"] == []
    assert status["claims"]["pct_gpmp_optimizer_performance"] is True
    assert status["claims"]["pct_native_rust_parity"] is False
    assert status["claims"]["full_gtsam_replacement"] is False


def test_status_can_include_pct_rust_runtime_acceptance(tmp_path: Path) -> None:
    module = _load_module()
    lm_actual_path = tmp_path / "pct_rust_process_levenberg_marquardt.json"
    gn_actual_path = tmp_path / "pct_rust_process_gauss_newton.json"
    compare_path = tmp_path / "pct_gpmp_optimizer_compare.json"
    acceptance_path = tmp_path / "pct_rust_runtime_acceptance.json"
    lm_actual_path.write_text(
        json.dumps(_passing_pct_rust_process_actual()),
        encoding="utf-8",
    )
    gn_actual_path.write_text(
        json.dumps(_passing_pct_rust_process_gn_actual()),
        encoding="utf-8",
    )
    compare_path.write_text(
        json.dumps(_passing_pct_gpmp_optimizer_compare_result()),
        encoding="utf-8",
    )
    acceptance_path.write_text(
        json.dumps(
            _passing_pct_rust_runtime_acceptance_result(
                lm_actual_json=lm_actual_path,
                gn_actual_json=gn_actual_path,
                optimizer_compare_json=compare_path,
            )
        ),
        encoding="utf-8",
    )

    status = module.build_status(
        ROOT,
        pct_rust_runtime_acceptance_json=acceptance_path,
        require_pct_rust_process_golden=True,
        require_pct_rust_process_gn_golden=True,
        require_pct_gpmp_optimizer_compare=True,
    )

    assert status["ok"] is True
    assert status["pct_rust_runtime_acceptance_readiness"]["status"] == "pass"
    assert status["pct_rust_process_golden_readiness"]["status"] == "pass"
    assert status["pct_rust_process_gn_golden_readiness"]["status"] == "pass"
    assert status["pct_gpmp_optimizer_compare_readiness"]["status"] == "pass"
    assert status["claims"]["pct_rust_process_golden_parity"] is True
    assert status["claims"]["pct_rust_process_gn_golden_parity"] is True
    assert status["claims"]["pct_gpmp_optimizer_performance"] is True
    assert status["claims"]["pct_native_rust_parity"] is False
    assert status["claims"]["full_gtsam_replacement"] is False


def test_status_rejects_failed_pct_rust_runtime_acceptance(tmp_path: Path) -> None:
    module = _load_module()
    acceptance_path = tmp_path / "pct_rust_runtime_acceptance.json"
    payload = _passing_pct_rust_runtime_acceptance_result(
        lm_actual_json=tmp_path / "missing_lm.json",
        gn_actual_json=tmp_path / "missing_gn.json",
        optimizer_compare_json=tmp_path / "missing_compare.json",
    )
    payload["summary"] = {
        "verdict": "fail",
        "failed_checks": ["optimizer_compare_failed"],
    }
    acceptance_path.write_text(json.dumps(payload), encoding="utf-8")

    status = module.build_status(
        ROOT,
        pct_rust_runtime_acceptance_json=acceptance_path,
    )

    assert status["ok"] is False
    assert status["pct_rust_runtime_acceptance_readiness"]["status"] == "fail"
    assert "pct_rust_runtime_acceptance_failed" in status["blockers"]
    failed = set(status["pct_rust_runtime_acceptance_readiness"]["failed_checks"])
    assert "optimizer_compare_failed" in failed
    assert "levenberg_marquardt_actual_json:missing_file" in failed
    assert "gauss_newton_actual_json:missing_file" in failed
    assert "optimizer_compare_json:missing_file" in failed


def test_status_rejects_optimizer_compare_summary_without_required_cases(tmp_path: Path) -> None:
    module = _load_module()
    compare_path = tmp_path / "optimizer_compare.json"
    payload = _passing_pct_gpmp_optimizer_compare_result()
    payload["cases"] = [
        {
            "mode": "wnoj",
            "nonlinear_optimizer": "levenberg_marquardt",
            "state_count": 96,
            "speedup": 1.6,
            "final_cost_delta_abs": 0.0,
            "dense": {
                "reported_linear_solver": "dense",
                "reported_nonlinear_optimizer": "levenberg_marquardt",
            },
            "block_tridiagonal": {
                "reported_linear_solver": "dense",
                "reported_nonlinear_optimizer": "levenberg_marquardt",
                "linear_solve_fallbacks": 1,
            },
        }
    ]
    compare_path.write_text(json.dumps(payload), encoding="utf-8")

    status = module.build_status(
        ROOT,
        pct_gpmp_optimizer_compare_json=compare_path,
        require_pct_gpmp_optimizer_compare=True,
    )

    assert status["ok"] is False
    assert status["pct_gpmp_optimizer_compare_readiness"]["status"] == "fail"
    failed = set(status["pct_gpmp_optimizer_compare_readiness"]["failed_checks"])
    assert "wnoa:levenberg_marquardt:missing_case" in failed
    assert "wnoj:gauss_newton:missing_case" in failed
    assert "wnoa:gauss_newton:missing_case" in failed
    assert "wnoj:levenberg_marquardt:96:block_solver_not_reported" in failed
    assert "wnoj:levenberg_marquardt:96:block_solver_fallbacks" in failed
    assert status["claims"]["pct_gpmp_optimizer_performance"] is False


def test_status_requires_pct_native_rust_parity_when_requested() -> None:
    module = _load_module()

    status = module.build_status(ROOT, require_pct_native_rust_parity=True)

    assert status["ok"] is False
    assert status["pct_native_rust_parity_readiness"]["status"] == "missing_result"
    assert "pct_native_rust_parity_missing" in status["blockers"]
    assert status["claims"]["pct_native_rust_parity"] is False


def test_status_can_include_pct_native_rust_parity(tmp_path: Path) -> None:
    module = _load_module()
    parity_path = tmp_path / "native_rust_parity.json"
    parity_path.write_text(
        json.dumps(_passing_pct_native_rust_parity_result()),
        encoding="utf-8",
    )

    status = module.build_status(
        ROOT,
        pct_native_rust_parity_json=parity_path,
        require_pct_native_rust_parity=True,
    )

    assert status["ok"] is True
    assert status["pct_native_rust_parity_readiness"]["status"] == "pass"
    assert status["pct_native_rust_parity_readiness"]["failed_checks"] == []
    assert status["claims"]["pct_native_rust_parity"] is True
    assert status["claims"]["full_gtsam_replacement"] is False


def test_status_requires_camera_lidar_abi_smoke_when_requested() -> None:
    module = _load_module()

    status = module.build_status(ROOT, require_camera_lidar_abi_smoke=True)

    assert status["ok"] is False
    assert status["camera_lidar_abi_smoke_readiness"]["status"] == "missing_result"
    assert "camera_lidar_abi_smoke_missing" in status["blockers"]
    assert status["claims"]["camera_lidar_abi_smoke"] is False


def test_status_can_include_camera_lidar_abi_smoke(tmp_path: Path) -> None:
    module = _load_module()
    smoke_path = tmp_path / "camera_lidar_abi_smoke.json"
    smoke_path.write_text(
        json.dumps(_passing_camera_lidar_abi_smoke_result()),
        encoding="utf-8",
    )

    status = module.build_status(
        ROOT,
        camera_lidar_abi_smoke_json=smoke_path,
        require_camera_lidar_abi_smoke=True,
    )

    assert status["ok"] is True
    assert status["camera_lidar_abi_smoke_readiness"]["status"] == "pass"
    assert status["camera_lidar_abi_smoke_readiness"]["failed_checks"] == []
    assert status["claims"]["camera_lidar_abi_smoke"] is True
    assert status["claims"]["full_gtsam_replacement"] is False


def test_status_rejects_camera_lidar_abi_smoke_failed_check(tmp_path: Path) -> None:
    module = _load_module()
    smoke_path = tmp_path / "camera_lidar_abi_smoke.json"
    payload = _passing_camera_lidar_abi_smoke_result()
    payload["ok"] = False
    payload["checks"]["optimizer_cost_reduction"] = False
    payload["summary"] = {
        "ok": False,
        "failed_checks": ["optimizer_cost_reduction"],
    }
    smoke_path.write_text(json.dumps(payload), encoding="utf-8")

    status = module.build_status(
        ROOT,
        camera_lidar_abi_smoke_json=smoke_path,
        require_camera_lidar_abi_smoke=True,
    )

    assert status["ok"] is False
    assert status["camera_lidar_abi_smoke_readiness"]["status"] == "fail"
    assert "camera_lidar_abi_smoke_failed" in status["blockers"]
    assert status["camera_lidar_abi_smoke_readiness"]["failed_checks"] == [
        "optimizer_cost_reduction"
    ]
