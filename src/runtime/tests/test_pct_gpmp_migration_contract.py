from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
CONTRACT_SCRIPT = ROOT / "tools" / "validate" / "validate_pct_gpmp_migration_contract.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "validate_pct_gpmp_migration_contract",
        CONTRACT_SCRIPT,
    )
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_contract_declares_pct_gpmp_outside_pose_graph_opt_surface() -> None:
    module = _load_module()

    contract = module.build_contract(ROOT)

    assert contract["schema"] == "lingtu.pct_gpmp_migration_contract.v1"
    assert contract["ok"] is True
    assert contract["covered_by_pose_graph_opt"] is False
    assert "Vector4/Vector6" in contract["reason_not_covered_by_pose_graph_opt"]
    assert contract["runtime_boundary"]["entrypoint"] == "load_pct_planner_runtime"
    assert contract["runtime_boundary"]["selector_env"] == "LINGTU_PCT_PLANNER_RUNTIME"
    assert (
        contract["runtime_boundary"]["legacy_native_allow_env"]
        == "LINGTU_PCT_ALLOW_LEGACY_GTSAM_NATIVE"
    )
    assert contract["runtime_boundary"]["default_runtime"] == "rust_process"
    assert contract["runtime_boundary"]["implemented_runtimes"] == ["native", "rust_process"]
    assert contract["runtime_boundary"]["future_runtimes"] == []
    assert contract["rust_math_kernel"]["crate"] == "lingtu_gpmp_trajectory_optimizer"
    assert "WNOJ Vector6 process Q/Phi/QInverse" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "WNOJ/WNOA obstacle residuals and Jacobians" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "WNOJ/WNOA interpolated obstacle chain-rule Jacobians" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "WNOJ/WNOA dense LM/GN batch optimizer for portable small graphs" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "WNOJ/WNOA block-tridiagonal sparse LM/GN batch optimizer for trajectory-chain graphs" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "WNOJ/WNOA faer sparse Cholesky LM/GN batch optimizer for non-chain graphs" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "PCT runtime accepts explicit sparse GPMP linear solver requests" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Rust process runtime can select LM or GN nonlinear optimization" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Rust process runtime validates optimizer schema, shapes, finite values, and solver report before accepting output" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Legacy PCT adapter runtime enters through the same native/Rust runtime selector" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Rust process native-like GPMP optimizer result accessors" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Rust process optimizer returns native-like interpolated trajectory output" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Rust process planner consumes expanded optimizer trajectory output for returned paths and accessors" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Rust process runtime callable from PCT runtime on Windows/lightweight hosts" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Rust GPMP optimizer exposes an in-process C ABI for lower-overhead runtime calls" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "PCT runtime auto-prefers Rust FFI and falls back to gpmp_optimize process when needed" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "PCT default runtime uses Rust process on Linux/Windows/lightweight hosts" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "PCT auto runtime prefers Rust process when the optimizer binary is available" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "PCT auto runtime never falls back to legacy Linux/GTSAM native modules" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "PCT native/GTSAM runtime requires an explicit legacy allow gate" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "PCT runtime resolves packaged Rust optimizer artifacts from pct/runtime/rust/<arch>" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "PCT legacy native loader no longer uses bundled planner/lib/x86_64 artifacts" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Build path generates the PCT Rust optimizer binary for rust_process runtime" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "PCT Docker build packages Rust GPMP optimizer artifacts without GTSAM native modules" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Server setup builds and installs PCT Rust GPMP runtime artifacts by default" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "PCT standalone native CMake skips C++ native modules by default" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "PCT standalone native CMake skips GTSAM by default" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Legacy Linux/GTSAM native build helper requires an explicit comparison flag" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Original PCT planner and third-party GTSAM build scripts require an explicit legacy flag" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "PCT preview/golden validates native wrapper optimizer accessor shape parity" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Linux native/rust_process parity gate compares optimizer accessor output shape parity" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Windows-safe dense-vs-block-tridiagonal performance and cost gate" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "Linux same-input native-vs-rust_process PCT parity gate" in (
        contract["rust_math_kernel"]["covered_capabilities"]
    )
    assert "general sparse LM/GN solver for arbitrary non-chain factor graphs" not in (
        contract["rust_math_kernel"]["not_covered_yet"]
    )
    assert "GTSAM-free PCT Rust optimizer deployment packaging" in (
        contract["required_capability_groups"]
    )
    assert "server setup default PCT runtime is Rust, not GTSAM native" in (
        contract["required_capability_groups"]
    )
    assert contract["forbidden_source_violations"] == []


def test_contract_covers_python_wrapper_output_parity_requirements() -> None:
    module = _load_module()

    contract = module.build_contract(ROOT)
    python_contract = contract["python_contract"]

    assert python_contract["planner_class"] == "TomogramPlanner"
    assert "plan(start_pos, end_pos" in python_contract["plan_signature"]
    assert python_contract["returned_path_shape"] == "Nx3 world coordinates [x, y, z]"
    assert {
        "get_result_matrix",
        "get_layers",
        "get_heights",
        "get_opt_init_value",
        "get_opt_init_layer",
    }.issubset(set(python_contract["required_result_fields"]))


def test_contract_checks_include_wnoj_wnoa_and_lm_solver_sources() -> None:
    module = _load_module()

    contract = module.build_contract(ROOT)
    checks = {item["id"]: item for item in contract["source_checks"]}

    for required_id in (
        "vector6_graph",
        "vector6_gp_prior",
        "vector6_interpolate_obstacle",
        "vector6_heading_rate",
        "vector6_lm_optimizer",
        "vector4_graph",
        "vector4_gp_prior",
        "vector4_interpolate_obstacle",
        "vector4_lm_optimizer",
        "runtime_rust_process_supported",
        "runtime_default_rust_process",
        "runtime_legacy_native_allow_env",
        "runtime_native_runtime_allow_guard",
        "runtime_auto_prefers_rust_binary",
        "runtime_auto_never_falls_back_to_native",
        "runtime_packaged_rust_artifact_dirs",
        "runtime_packaged_rust_binary_search",
        "runtime_packaged_rust_library_search",
        "runtime_native_loader_avoids_original_lib_tree",
        "runtime_native_loader_uses_runnable_native_dir",
        "runtime_rust_process_planner",
        "runtime_rust_process_invocation",
        "runtime_rust_ffi_library_resolver",
        "runtime_rust_ffi_wrapper",
        "runtime_rust_ffi_invocation",
        "runtime_rust_optimizer_response_validation",
        "runtime_rust_optimizer_diagnostics",
        "runtime_rust_optimizer_output_state_diagnostics",
        "runtime_rust_optimizer_expanded_trajectory_diagnostics",
        "runtime_rust_optimizer_call_mode_diagnostics",
        "runtime_rust_linear_solver_config",
        "runtime_rust_sparse_linear_solver_config",
        "runtime_rust_linear_solver_diagnostics",
        "runtime_rust_nonlinear_optimizer_config",
        "runtime_rust_nonlinear_optimizer_diagnostics",
        "preview_optimizer_accessor_report",
        "preview_optimizer_accessors_json",
        "runtime_rust_result_matrix_accessor",
        "runtime_rust_layers_accessor",
        "runtime_rust_heights_accessor",
        "runtime_rust_opt_init_value_accessor",
        "runtime_rust_opt_init_layer_accessor",
        "runtime_rust_optimizer_result_cache",
        "runtime_rust_expanded_trajectory_validation",
        "runtime_rust_expanded_trajectory_consumption",
        "runtime_rust_wnoj_optimizer_view",
        "runtime_rust_wnoa_optimizer_view",
        "runtime_rust_get_trajectory_optimizer_wnoj",
        "runtime_rust_get_trajectory_optimizer",
        "runtime_rust_get_ceilings",
        "runtime_rust_get_heading_rate",
        "runtime_rust_wnoa_gp_prior_test",
        "rust_wnoj_process_model",
        "rust_wnoa_process_model",
        "rust_heading_rate_factor_math",
        "rust_dense_elevation_map",
        "rust_obstacle_factor_math",
        "rust_interpolated_obstacle_factor_math",
        "rust_wnoj_batch_problem",
        "rust_wnoj_batch_optimizer",
        "rust_wnoa_batch_problem",
        "rust_dense_batch_optimizers",
        "rust_nonlinear_optimizer_kind",
        "rust_gauss_newton_optimizer",
        "rust_gauss_newton_dense_sparse_tests",
        "rust_block_tridiagonal_system",
        "rust_block_tridiagonal_solver_config",
        "rust_sparse_cholesky_solver",
        "rust_sparse_solver_config",
        "rust_sparse_non_chain_tests",
        "rust_process_optimizer_binary",
        "rust_process_wnoa_optimizer_binary",
        "rust_optimizer_c_abi_version",
        "rust_optimizer_c_abi_json_entrypoint",
        "rust_optimizer_c_abi_free_json",
        "rust_optimizer_expanded_trajectory_output",
        "rust_optimizer_initial_trajectory_output",
        "rust_kernel_build_script_gpmp_target",
        "ros_workspace_builds_rust_gpmp_default",
        "ros_workspace_rust_kernel_build_toggle",
        "server_setup_pct_legacy_flag_default_off",
        "server_setup_pct_default_rust_runtime",
        "server_setup_pct_builds_gpmp_kernel",
        "server_setup_pct_installs_packaged_rust_artifacts",
        "server_setup_pct_legacy_gtsam_explicit_only",
        "server_setup_pct_legacy_gtsam_flag_forwarded",
        "pct_docker_build_uses_rust_base",
        "pct_docker_build_copies_gpmp_kernel",
        "pct_docker_builds_gpmp_release_artifacts",
        "pct_docker_outputs_gpmp_binary",
        "pct_docker_outputs_gpmp_cdylib",
        "pct_build_all_platforms_outputs_rust_artifacts",
        "pct_build_all_platforms_uses_repo_root_context",
        "pct_native_cmake_native_modules_option_off",
        "pct_native_cmake_legacy_gtsam_option_off",
        "pct_native_cmake_legacy_gtsam_default_off",
        "pct_native_cmake_gtsam_find_guard",
        "pct_legacy_native_build_requires_explicit_gtsam_flag",
        "pct_legacy_native_build_enables_cmake_legacy_flag",
        "pct_legacy_native_build_enables_native_modules",
        "pct_legacy_planner_build_requires_explicit_gtsam_flag",
        "pct_legacy_planner_build_enables_cmake_legacy_flag",
        "pct_legacy_planner_build_enables_native_modules",
        "pct_legacy_thirdparty_build_requires_explicit_gtsam_flag",
        "pct_gpmp_math_acceptance_gate",
        "pct_gpmp_math_acceptance_required_cases",
        "pct_gpmp_math_acceptance_status_claim",
        "rust_optimizer_dense_sparse_compare_gate",
        "rust_optimizer_compare_speedup_gate",
        "rust_optimizer_compare_lm_gn_default",
        "rust_optimizer_compare_missing_lm_gn_cases",
        "rust_process_smoke_selects_optimizer_golden",
        "rust_process_smoke_gn_golden",
        "pct_rust_runtime_acceptance_gate",
        "pct_rust_runtime_acceptance_lm_gn",
        "pct_rust_runtime_acceptance_status_claims",
        "pct_native_rust_parity_gate",
        "pct_native_rust_parity_subprocess_isolation",
        "pct_native_rust_parity_optimizer_accessor_flags",
        "pct_native_rust_parity_optimizer_accessor_shapes",
    ):
        assert checks[required_id]["present"] is True
        assert checks[required_id]["line"] is not None

    assert checks["rust_obstacle_factor_math"]["count"] >= 2
    assert checks["rust_interpolated_obstacle_factor_math"]["count"] >= 2
    assert checks["rust_dense_batch_optimizers"]["count"] >= 2
    assert checks["rust_sparse_non_chain_tests"]["count"] >= 2
    assert checks["rust_optimizer_expanded_trajectory_output"]["count"] >= 2
    assert checks["rust_optimizer_initial_trajectory_output"]["count"] >= 2
    assert checks["runtime_rust_expanded_trajectory_consumption"]["count"] >= 2
    assert checks["preview_optimizer_accessors_json"]["count"] >= 2
    assert checks["pct_native_rust_parity_optimizer_accessor_shapes"]["count"] >= 2
    assert checks["runtime_packaged_rust_library_search"]["count"] >= 2
    assert checks["pct_docker_builds_gpmp_release_artifacts"]["count"] >= 2
    forbidden = {item["id"]: item for item in contract["forbidden_source_checks"]}
    for required_id in (
        "pct_docker_build_no_gtsam",
        "pct_docker_build_no_libgtsam",
        "pct_docker_build_no_native_planner_lib_tree",
    ):
        assert forbidden[required_id]["violated"] is False


def test_contract_reports_missing_required_source_check(tmp_path: Path) -> None:
    module = _load_module()
    runtime = tmp_path / "src/nav/services/plan/global_planner/algorithm/pct/runtime/runtime.py"
    runtime.parent.mkdir(parents=True)
    runtime.write_text("# missing runtime seam\n", encoding="utf-8")

    contract = module.build_contract(tmp_path)

    assert contract["ok"] is False
    missing = {item["id"] for item in contract["missing_required_checks"]}
    assert "runtime_selector_seam" in missing
