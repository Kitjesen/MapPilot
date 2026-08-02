#!/usr/bin/env python3
"""Validate the static migration contract for the PCT/GPMP optimizer.

This intentionally does not load the Linux native extension. It records the
current C++/Python boundary that a future Rust or process runtime must preserve.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

SCHEMA = "lingtu.pct_gpmp_migration_contract.v1"

RUNTIME_BACKEND = "src/nav/services/plan/global_planner/algorithm/pct/runtime/api.py"
RUNTIME_COMMON = "src/nav/services/plan/global_planner/algorithm/pct/runtime/common.py"
RUNTIME_LOADER = "src/nav/services/plan/global_planner/algorithm/pct/runtime/loader.py"
RUNTIME_NATIVE = "src/nav/services/plan/global_planner/algorithm/pct/runtime/native_runtime.py"
RUNTIME_TOMOGRAM = "src/nav/services/plan/global_planner/algorithm/pct/runtime/tomogram_planner.py"
RUNTIME_FFI = "src/nav/services/plan/global_planner/algorithm/pct/runtime/ffi.py"
RUNTIME_RUST_GPMP = "src/nav/services/plan/global_planner/algorithm/pct/runtime/rust_gpmp.py"
RUNTIME_BOUNDARY = RUNTIME_BACKEND
RUNTIME_PREVIEW = "src/nav/services/plan/global_planner/algorithm/pct/runtime/preview.py"
ROS_GLOBAL_PLANNER = "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/scripts/global_planner.py"
PYTHON_WRAPPER = "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/scripts/planner_wrapper.py"
PCT_NATIVE_CMAKE = "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/CMakeLists.txt"
PCT_DOCKER_BUILD = "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/Dockerfile.build"
PCT_BUILD_ALL_PLATFORMS_SCRIPT = "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/build_all_platforms.sh"
PCT_LEGACY_NATIVE_BUILD_SCRIPT = "src/nav/services/plan/global_planner/algorithm/pct/runtime/build_legacy_native_x86_64.sh"
PCT_LEGACY_PLANNER_BUILD_SCRIPT = "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/build.sh"
PCT_LEGACY_THIRDPARTY_BUILD_SCRIPT = "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/build_thirdparty.sh"
PYTHON_INTERFACE = (
    "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src/trajectory_optimization/"
    "python_interface.cc"
)
GPMP_ROOT = (
    "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src/trajectory_optimization/"
    "gpmp_optimizer"
)
RUST_GPMP_KERNEL = "src/kernels/planning/gpmp_trajectory_optimizer/src/lib.rs"
RUST_GPMP_OPTIMIZER_BIN = (
    "src/kernels/planning/gpmp_trajectory_optimizer/src/bin/gpmp_optimize.rs"
)
PCT_GPMP_MATH_ACCEPTANCE = "tools/bench/pct_gpmp_math_acceptance.py"
RUST_GPMP_OPTIMIZER_COMPARE = "tools/bench/pct_gpmp_optimizer_compare.py"
PCT_RUST_PROCESS_SMOKE = "tools/bench/pct_rust_process_smoke.py"
PCT_RUST_RUNTIME_ACCEPTANCE = "tools/bench/pct_rust_runtime_acceptance.py"
PCT_RUST_PROCESS_GN_GOLDEN = (
    "src/nav/tests/planning_backends/fixtures/pct_preview/rust_process_synthetic_gn_smoke.json"
)
PCT_NATIVE_RUST_PARITY = "tools/bench/pct_native_rust_parity.py"
RUST_KERNEL_BUILD_SCRIPT = "scripts/build/build_rust_kernels.py"
ROS_WORKSPACE_BUILD_SCRIPT = "scripts/build/build_ros_workspace.sh"
SERVER_SETUP_SCRIPT = "sim/scripts/setup_linux_validation_host.sh"


@dataclass(frozen=True)
class SourceCheck:
    id: str
    path: str
    pattern: str
    capability: str
    required: bool = True
    min_count: int = 1


@dataclass(frozen=True)
class ForbiddenSourceCheck:
    id: str
    path: str
    pattern: str
    capability: str


@dataclass(frozen=True)
class CheckResult:
    id: str
    path: str
    capability: str
    required: bool
    present: bool
    line: int | None
    count: int
    min_count: int

    def to_jsonable(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "path": self.path,
            "capability": self.capability,
            "required": self.required,
            "present": self.present,
            "line": self.line,
            "count": self.count,
            "min_count": self.min_count,
        }


@dataclass(frozen=True)
class ForbiddenCheckResult:
    id: str
    path: str
    capability: str
    pattern: str
    violated: bool
    line: int | None
    count: int

    def to_jsonable(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "path": self.path,
            "capability": self.capability,
            "pattern": self.pattern,
            "violated": self.violated,
            "line": self.line,
            "count": self.count,
        }


SOURCE_CHECKS: tuple[SourceCheck, ...] = (
    SourceCheck(
        "runtime_selector_seam",
        RUNTIME_LOADER,
        "def load_pct_planner_runtime(",
        "single Python seam for native/Rust/process PCT planner selection",
    ),
    SourceCheck(
        "runtime_rust_process_supported",
        RUNTIME_COMMON,
        'PCT_SUPPORTED_PLANNER_RUNTIMES = ("native", "rust_process")',
        "PCT runtime can select the Rust process runtime",
    ),
    SourceCheck(
        "runtime_default_rust_process",
        RUNTIME_COMMON,
        'PCT_DEFAULT_PLANNER_RUNTIME = "rust_process"',
        "PCT defaults to the Rust process optimizer instead of silently falling back to GTSAM native",
    ),
    SourceCheck(
        "runtime_legacy_native_allow_env",
        RUNTIME_COMMON,
        'PCT_LEGACY_NATIVE_ALLOW_ENV = "LINGTU_PCT_ALLOW_LEGACY_GTSAM_NATIVE"',
        "PCT legacy native/GTSAM runtime requires a separate explicit allow environment variable",
    ),
    SourceCheck(
        "runtime_native_runtime_allow_guard",
        RUNTIME_COMMON,
        'if runtime == "native" and not _legacy_native_runtime_allowed():',
        "PCT runtime rejects native/GTSAM runtime selection unless the legacy allow gate is enabled",
    ),
    SourceCheck(
        "runtime_auto_prefers_rust_binary",
        RUNTIME_COMMON,
        "def _auto_pct_planner_runtime",
        "PCT auto runtime prefers the Rust process optimizer when its binary is available",
    ),
    SourceCheck(
        "runtime_auto_never_falls_back_to_native",
        RUNTIME_COMMON,
        "compatibility selector and always resolves to Rust",
        "PCT auto runtime never falls back to legacy Linux/GTSAM native modules",
    ),
    SourceCheck(
        "runtime_packaged_rust_artifact_dirs",
        RUNTIME_COMMON,
        'runnable_root / "rust" / canonical_arch',
        "PCT runtime resolves packaged Rust optimizer artifacts from pct/runtime/rust/<arch>",
    ),
    SourceCheck(
        "runtime_packaged_rust_binary_search",
        RUNTIME_COMMON,
        "for artifact_dir in _candidate_rust_gpmp_artifact_dirs(repo_root)",
        "PCT Rust process binary resolver searches packaged deployment artifacts before crate target output",
    ),
    SourceCheck(
        "runtime_packaged_rust_library_search",
        RUNTIME_COMMON,
        "candidates.append(artifact_dir / name)",
        "PCT Rust FFI library resolver searches packaged deployment artifacts before crate target output",
        min_count=2,
    ),
    SourceCheck(
        "runtime_native_loader_avoids_original_lib_tree",
        RUNTIME_NATIVE,
        "loaded from the upstream planner/lib tree",
        "PCT legacy native loader no longer uses bundled planner/lib/x86_64 artifacts as a runtime fallback",
    ),
    SourceCheck(
        "runtime_native_loader_uses_runnable_native_dir",
        RUNTIME_NATIVE,
        'runnable_root / "native" / canonical_arch',
        "PCT legacy native loader uses pct_runtime/native artifacts unless an explicit env path is supplied",
    ),
    SourceCheck(
        "runtime_rust_process_planner",
        RUNTIME_TOMOGRAM,
        "class RustProcessTomogramPlanner",
        "Python planner replacement that uses Rust GPMP smoothing without Linux native GTSAM artifacts",
    ),
    SourceCheck(
        "runtime_rust_process_invocation",
        RUNTIME_TOMOGRAM,
        "subprocess.run(",
        "Rust GPMP optimizer process is invoked from the PCT runtime",
    ),
    SourceCheck(
        "runtime_rust_ffi_library_resolver",
        RUNTIME_COMMON,
        "def resolve_rust_gpmp_optimizer_library",
        "PCT runtime can resolve a Rust GPMP optimizer dynamic library for in-process calls",
    ),
    SourceCheck(
        "runtime_rust_ffi_wrapper",
        RUNTIME_FFI,
        "class RustGpmpOptimizerLibrary",
        "PCT runtime has a ctypes wrapper for the Rust GPMP optimizer C ABI",
    ),
    SourceCheck(
        "runtime_rust_ffi_invocation",
        RUNTIME_TOMOGRAM,
        "self.optimizer_library.optimize_json",
        "PCT runtime can invoke the Rust optimizer in-process without spawning gpmp_optimize",
    ),
    SourceCheck(
        "runtime_rust_optimizer_response_validation",
        RUNTIME_TOMOGRAM,
        "def _validate_optimizer_response",
        "PCT runtime validates Rust optimizer schema, shapes, finite values, and solver report before accepting output",
    ),
    SourceCheck(
        "runtime_rust_optimizer_diagnostics",
        RUNTIME_PREVIEW,
        "last_optimizer_final_cost",
        "PCT preview reports Rust optimizer cost/iteration diagnostics",
    ),
    SourceCheck(
        "runtime_rust_optimizer_output_state_diagnostics",
        RUNTIME_PREVIEW,
        "last_optimizer_output_states",
        "PCT preview reports how many trajectory states came back from the Rust optimizer",
    ),
    SourceCheck(
        "runtime_rust_optimizer_expanded_trajectory_diagnostics",
        RUNTIME_PREVIEW,
        "last_optimizer_trajectory_expanded",
        "PCT preview reports whether Rust optimizer output used native-like interpolated trajectory states",
    ),
    SourceCheck(
        "runtime_rust_optimizer_call_mode_diagnostics",
        RUNTIME_PREVIEW,
        "last_optimizer_call_mode",
        "PCT preview reports whether the Rust optimizer was called through FFI or process fallback",
    ),
    SourceCheck(
        "runtime_rust_linear_solver_config",
        RUNTIME_TOMOGRAM,
        "LINGTU_PCT_RUST_LINEAR_SOLVER",
        "PCT Rust process runtime can select the Rust linear solver",
    ),
    SourceCheck(
        "runtime_rust_sparse_linear_solver_config",
        RUNTIME_TOMOGRAM,
        "sparse_cholesky",
        "PCT runtime accepts explicit sparse linear solver requests and validates sparse Cholesky reports",
    ),
    SourceCheck(
        "runtime_rust_linear_solver_diagnostics",
        RUNTIME_PREVIEW,
        "last_optimizer_linear_solver",
        "PCT preview reports the Rust linear solver actually used",
    ),
    SourceCheck(
        "runtime_rust_nonlinear_optimizer_config",
        RUNTIME_TOMOGRAM,
        "LINGTU_PCT_RUST_NONLINEAR_OPTIMIZER",
        "PCT Rust process runtime can select LM or GN nonlinear optimization",
    ),
    SourceCheck(
        "runtime_rust_nonlinear_optimizer_diagnostics",
        RUNTIME_PREVIEW,
        "last_optimizer_nonlinear_optimizer",
        "PCT preview reports the Rust nonlinear optimizer actually used",
    ),
    SourceCheck(
        "preview_optimizer_accessor_report",
        RUNTIME_PREVIEW,
        "def _optimizer_accessor_report",
        "PCT preview records native wrapper optimizer accessor shape parity",
    ),
    SourceCheck(
        "preview_optimizer_accessors_json",
        RUNTIME_PREVIEW,
        '"optimizer_accessors": _optimizer_accessor_report',
        "PCT preview exports optimizer accessor parity into golden-comparable JSON",
        min_count=2,
    ),
    SourceCheck(
        "runtime_rust_result_matrix_accessor",
        RUNTIME_TOMOGRAM,
        "def get_result_matrix",
        "Rust process planner exposes native-compatible optimized result matrix",
    ),
    SourceCheck(
        "runtime_rust_layers_accessor",
        RUNTIME_TOMOGRAM,
        "def get_layers",
        "Rust process planner exposes native-compatible optimized layer vector",
    ),
    SourceCheck(
        "runtime_rust_heights_accessor",
        RUNTIME_TOMOGRAM,
        "def get_heights",
        "Rust process planner exposes native-compatible optimized height vector",
    ),
    SourceCheck(
        "runtime_rust_opt_init_value_accessor",
        RUNTIME_TOMOGRAM,
        "def get_opt_init_value",
        "Rust process planner exposes native-compatible optimizer initial values",
    ),
    SourceCheck(
        "runtime_rust_opt_init_layer_accessor",
        RUNTIME_TOMOGRAM,
        "def get_opt_init_layer",
        "Rust process planner exposes native-compatible optimizer initial layers",
    ),
    SourceCheck(
        "runtime_rust_optimizer_result_cache",
        RUNTIME_TOMOGRAM,
        "_record_optimizer_result",
        "Rust process planner stores optimizer response for wrapper output parity",
    ),
    SourceCheck(
        "runtime_rust_expanded_trajectory_validation",
        RUNTIME_TOMOGRAM,
        "trajectory_states",
        "Rust process planner validates expanded trajectory output from the optimizer process",
    ),
    SourceCheck(
        "runtime_rust_expanded_trajectory_consumption",
        RUNTIME_TOMOGRAM,
        '_response_field(response, "trajectory_states", "states")',
        "Rust process planner uses expanded trajectory output for returned paths and native-like accessors",
        min_count=2,
    ),
    SourceCheck(
        "runtime_rust_wnoj_optimizer_view",
        RUNTIME_RUST_GPMP,
        "class RustProcessGPMPOptimizer",
        "Rust process runtime exposes a native-like WNOJ optimizer result view",
    ),
    SourceCheck(
        "runtime_rust_wnoa_optimizer_view",
        RUNTIME_RUST_GPMP,
        "class RustProcessGPMPOptimizerWnoa",
        "Rust process runtime exposes a native-like WNOA optimizer result view",
    ),
    SourceCheck(
        "runtime_rust_get_trajectory_optimizer_wnoj",
        RUNTIME_TOMOGRAM,
        "def get_trajectory_optimizer_wnoj",
        "Rust process planner exposes the native WNOJ optimizer accessor",
    ),
    SourceCheck(
        "runtime_rust_get_trajectory_optimizer",
        RUNTIME_TOMOGRAM,
        "def get_trajectory_optimizer(self)",
        "Rust process planner exposes the native WNOA optimizer accessor",
    ),
    SourceCheck(
        "runtime_rust_get_ceilings",
        RUNTIME_RUST_GPMP,
        "def get_ceilings",
        "Rust process WNOJ optimizer view exposes ceiling output",
    ),
    SourceCheck(
        "runtime_rust_get_heading_rate",
        RUNTIME_RUST_GPMP,
        "def get_heading_rate",
        "Rust process WNOJ optimizer view exposes heading-rate output",
    ),
    SourceCheck(
        "runtime_rust_wnoa_gp_prior_test",
        RUNTIME_RUST_GPMP,
        "def gp_prior_test",
        "Rust process WNOA optimizer view implements gp_prior_test through the Rust optimizer",
    ),
    SourceCheck(
        "wrapper_plan_contract",
        PYTHON_WRAPPER,
        "def plan(self, start_pos, end_pos, start_height=0, end_height=0):",
        "TomogramPlanner plan(start_xy, goal_xy, start_h, goal_h) caller contract",
    ),
    SourceCheck(
        "wrapper_native_optimizer_selection",
        PYTHON_WRAPPER,
        "self.planner.get_trajectory_optimizer_wnoj()",
        "PCT wrapper retrieves the native WNOJ optimizer result object",
    ),
    SourceCheck(
        "wrapper_result_matrix",
        PYTHON_WRAPPER,
        "optimizer.get_result_matrix()",
        "optimized XY trajectory matrix returned to Python",
    ),
    SourceCheck(
        "wrapper_layers",
        PYTHON_WRAPPER,
        "optimizer.get_layers()",
        "optimized tomogram layer vector returned to Python",
    ),
    SourceCheck(
        "wrapper_heights",
        PYTHON_WRAPPER,
        "optimizer.get_heights()",
        "optimized height vector returned to Python",
    ),
    SourceCheck(
        "pybind_gpmp_optimizer",
        PYTHON_INTERFACE,
        'py::class_<GPMPOptimizer>(m, "GPMPOptimizer")',
        "pybind surface for WNOJ Vector6 optimizer",
    ),
    SourceCheck(
        "pybind_gpmp_optimizer_wnoa",
        PYTHON_INTERFACE,
        'py::class_<GPMPOptimizerWnoa>(m, "GPMPOptimizerWnoa")',
        "pybind surface for WNOA Vector4 optimizer",
    ),
    SourceCheck(
        "vector6_graph",
        f"{GPMP_ROOT}/gpmp_optimizer.cc",
        "auto graph = gtsam::NonlinearFactorGraph();",
        "WNOJ Vector6 nonlinear factor graph",
    ),
    SourceCheck(
        "vector6_values",
        f"{GPMP_ROOT}/gpmp_optimizer.cc",
        "gtsam::Values init_values;",
        "WNOJ Vector6 initial values container",
    ),
    SourceCheck(
        "vector6_gp_prior",
        f"{GPMP_ROOT}/gpmp_optimizer.cc",
        "graph.add(GPPriorFactor(last_x, this_x, dt, kQc));",
        "WNOJ Gaussian-process prior factor",
    ),
    SourceCheck(
        "vector6_interpolate_obstacle",
        f"{GPMP_ROOT}/gpmp_optimizer.cc",
        "graph.add(GPInterpolateObstacleFactor(",
        "WNOJ interpolated obstacle factor over DenseElevationMap",
    ),
    SourceCheck(
        "vector6_heading_rate",
        f"{GPMP_ROOT}/gpmp_optimizer.cc",
        "graph.add(GPInterpolateHeadingRateFactor(",
        "WNOJ heading-rate constraint factor",
    ),
    SourceCheck(
        "vector6_lm_optimizer",
        f"{GPMP_ROOT}/gpmp_optimizer.cc",
        "gtsam::LevenbergMarquardtOptimizer opt(graph, init_values, param);",
        "WNOJ batch Levenberg-Marquardt solve",
    ),
    SourceCheck(
        "vector4_graph",
        f"{GPMP_ROOT}/gpmp_optimizer_wnoa.cc",
        "auto graph = gtsam::NonlinearFactorGraph();",
        "WNOA Vector4 nonlinear factor graph",
    ),
    SourceCheck(
        "vector4_gp_prior",
        f"{GPMP_ROOT}/gpmp_optimizer_wnoa.cc",
        "graph.add(GPPriorFactorWnoa(last_x, this_x, dt, kQc));",
        "WNOA Gaussian-process prior factor",
    ),
    SourceCheck(
        "vector4_interpolate_obstacle",
        f"{GPMP_ROOT}/gpmp_optimizer_wnoa.cc",
        "graph.add(GPInterpolateObstacleFactorWnoa(",
        "WNOA interpolated obstacle factor over DenseElevationMap",
    ),
    SourceCheck(
        "vector4_lm_optimizer",
        f"{GPMP_ROOT}/gpmp_optimizer_wnoa.cc",
        "gtsam::LevenbergMarquardtOptimizer opt(graph, init_values, param);",
        "WNOA batch Levenberg-Marquardt solve",
    ),
    SourceCheck(
        "factor_vector6_type",
        f"{GPMP_ROOT}/factors/gp_prior_factor.h",
        "public gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::Vector6>",
        "custom binary Vector6 factor base",
    ),
    SourceCheck(
        "factor_vector4_type",
        f"{GPMP_ROOT}/factors_wnoa/gp_prior_factor.h",
        "public gtsam::NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4>",
        "custom binary Vector4 factor base",
    ),
    SourceCheck(
        "rust_wnoj_process_model",
        RUST_GPMP_KERNEL,
        "pub mod wnoj",
        "Rust WNOJ process model, prior residual, interpolation, and heading-rate math",
    ),
    SourceCheck(
        "rust_wnoa_process_model",
        RUST_GPMP_KERNEL,
        "pub mod wnoa",
        "Rust WNOA process model, prior residual, and interpolation math",
    ),
    SourceCheck(
        "rust_heading_rate_factor_math",
        RUST_GPMP_KERNEL,
        "pub fn heading_rate_residual",
        "Rust heading-rate residual matching the C++ piecewise factor rule",
    ),
    SourceCheck(
        "rust_dense_elevation_map",
        RUST_GPMP_KERNEL,
        "pub struct DenseElevationMap",
        "Rust DenseElevationMap safe layer/height/bilinear obstacle query support",
    ),
    SourceCheck(
        "rust_obstacle_factor_math",
        RUST_GPMP_KERNEL,
        "pub fn obstacle_residual_jacobian",
        "Rust WNOJ/WNOA obstacle residual and Jacobian matching the C++ squared violation rule",
        min_count=2,
    ),
    SourceCheck(
        "rust_interpolated_obstacle_factor_math",
        RUST_GPMP_KERNEL,
        "pub fn interpolate_obstacle_residual_jacobians",
        "Rust WNOJ/WNOA interpolated obstacle residual chain-rule Jacobians",
        min_count=2,
    ),
    SourceCheck(
        "rust_wnoj_batch_problem",
        RUST_GPMP_KERNEL,
        "pub struct WnojBatchProblem",
        "Rust WNOJ batch trajectory factor graph container",
    ),
    SourceCheck(
        "rust_wnoj_batch_optimizer",
        RUST_GPMP_KERNEL,
        "pub fn optimize(",
        "Rust dense LM/GN WNOJ batch optimizer for small/portable GPMP graphs",
    ),
    SourceCheck(
        "rust_wnoa_batch_problem",
        RUST_GPMP_KERNEL,
        "pub struct WnoaBatchProblem",
        "Rust WNOA batch trajectory factor graph container",
    ),
    SourceCheck(
        "rust_dense_batch_optimizers",
        RUST_GPMP_KERNEL,
        "pub fn optimize(",
        "Rust dense LM/GN WNOJ/WNOA batch optimizers for small/portable GPMP graphs",
        min_count=2,
    ),
    SourceCheck(
        "rust_nonlinear_optimizer_kind",
        RUST_GPMP_KERNEL,
        "pub enum NonlinearOptimizerKind",
        "Rust optimizer has an explicit LM/GN nonlinear optimizer selector",
    ),
    SourceCheck(
        "rust_gauss_newton_optimizer",
        RUST_GPMP_KERNEL,
        "GaussNewton",
        "Rust optimizer implements a Gauss-Newton branch without LM damping",
    ),
    SourceCheck(
        "rust_gauss_newton_dense_sparse_tests",
        RUST_GPMP_KERNEL,
        "gauss_newton_matches_dense_solver",
        "Rust tests compare dense and block-tridiagonal Gauss-Newton solves",
    ),
    SourceCheck(
        "rust_block_tridiagonal_system",
        RUST_GPMP_KERNEL,
        "struct BlockTridiagonalSystem",
        "Rust block-tridiagonal sparse linear system for trajectory-chain GPMP graphs",
    ),
    SourceCheck(
        "rust_block_tridiagonal_solver_config",
        RUST_GPMP_KERNEL,
        "BlockTridiagonal",
        "Rust optimizer can select block-tridiagonal sparse LM/GN solves",
    ),
    SourceCheck(
        "rust_sparse_cholesky_solver",
        RUST_GPMP_KERNEL,
        "fn solve_sparse_lm",
        "Rust faer sparse Cholesky LM/GN linear solve for general non-chain GPMP normal systems",
    ),
    SourceCheck(
        "rust_sparse_solver_config",
        RUST_GPMP_KERNEL,
        "Sparse",
        "Rust optimizer can select faer sparse Cholesky solves",
    ),
    SourceCheck(
        "rust_sparse_non_chain_tests",
        RUST_GPMP_KERNEL,
        "non_chain_graph",
        "Rust tests compare sparse Cholesky and dense solves on non-chain WNOJ/WNOA graphs",
        min_count=2,
    ),
    SourceCheck(
        "rust_process_optimizer_binary",
        RUST_GPMP_OPTIMIZER_BIN,
        "optimize_request_json",
        "Rust process optimizer binary entrypoint for PCT runtime calls",
    ),
    SourceCheck(
        "rust_process_wnoa_optimizer_binary",
        RUST_GPMP_KERNEL,
        "optimize_request_json",
        "Rust shared JSON optimizer entrypoint can dispatch WNOA optimization requests",
    ),
    SourceCheck(
        "rust_optimizer_c_abi_version",
        RUST_GPMP_KERNEL,
        "lingtu_gpmp_optimizer_abi_version",
        "Rust GPMP optimizer exposes a stable C ABI version",
    ),
    SourceCheck(
        "rust_optimizer_c_abi_json_entrypoint",
        RUST_GPMP_KERNEL,
        "lingtu_gpmp_optimizer_optimize_json",
        "Rust GPMP optimizer exposes a C ABI JSON optimization entrypoint",
    ),
    SourceCheck(
        "rust_optimizer_c_abi_free_json",
        RUST_GPMP_KERNEL,
        "lingtu_gpmp_optimizer_free_json",
        "Rust GPMP optimizer exposes a C ABI output buffer release function",
    ),
    SourceCheck(
        "rust_optimizer_expanded_trajectory_output",
        RUST_GPMP_KERNEL,
        "trajectory_states",
        "Rust optimizer emits native-like interpolated WNOJ/WNOA trajectory states",
        min_count=2,
    ),
    SourceCheck(
        "rust_optimizer_initial_trajectory_output",
        RUST_GPMP_KERNEL,
        "initial_trajectory_states",
        "Rust optimizer emits native-like interpolated initial trajectory states for wrapper parity",
        min_count=2,
    ),
    SourceCheck(
        "rust_kernel_build_script_gpmp_target",
        RUST_KERNEL_BUILD_SCRIPT,
        "gpmp_trajectory_optimizer",
        "Portable Rust kernel build script can build the PCT GPMP optimizer binary",
    ),
    SourceCheck(
        "ros_workspace_builds_rust_gpmp_default",
        ROS_WORKSPACE_BUILD_SCRIPT,
        'LINGTU_RUST_KERNEL_TARGETS:-gpmp_trajectory_optimizer,camera_lidar_optimizer',
        "ROS workspace build prepares the PCT Rust optimizer binary by default while sharing the portable Rust kernel build path",
    ),
    SourceCheck(
        "ros_workspace_rust_kernel_build_toggle",
        ROS_WORKSPACE_BUILD_SCRIPT,
        "LINGTU_BUILD_RUST_KERNELS",
        "ROS workspace build keeps an explicit escape hatch for legacy native/GTSAM comparisons",
    ),
    SourceCheck(
        "server_setup_pct_legacy_flag_default_off",
        SERVER_SETUP_SCRIPT,
        'PCT_BUILD_LEGACY_NATIVE="${LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE:-0}"',
        "Server setup keeps legacy PCT GTSAM native runtime disabled by default",
    ),
    SourceCheck(
        "server_setup_pct_default_rust_runtime",
        SERVER_SETUP_SCRIPT,
        "building PCT Rust GPMP runtime",
        "Server setup builds the Rust GPMP optimizer runtime by default for PCT",
    ),
    SourceCheck(
        "server_setup_pct_builds_gpmp_kernel",
        SERVER_SETUP_SCRIPT,
        "scripts/build/build_rust_kernels.py --target gpmp_trajectory_optimizer --release",
        "Server setup uses the shared portable Rust kernel build entrypoint for PCT",
    ),
    SourceCheck(
        "server_setup_pct_installs_packaged_rust_artifacts",
        SERVER_SETUP_SCRIPT,
        'algorithm/pct/runtime/rust/${arch}',
        "Server setup installs PCT Rust optimizer artifacts where runtime deployment lookup expects them",
    ),
    SourceCheck(
        "server_setup_pct_legacy_gtsam_explicit_only",
        SERVER_SETUP_SCRIPT,
        'if [[ "${PCT_BUILD_LEGACY_NATIVE}" == "1" ]]; then',
        "Server setup only builds legacy PCT GTSAM native artifacts behind an explicit flag",
    ),
    SourceCheck(
        "server_setup_pct_legacy_gtsam_flag_forwarded",
        SERVER_SETUP_SCRIPT,
        "LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=1",
        "Server setup forwards the explicit legacy GTSAM flag only inside the legacy branch",
    ),
    SourceCheck(
        "pct_docker_build_uses_rust_base",
        PCT_DOCKER_BUILD,
        "FROM rust:",
        "PCT Docker build uses the Rust toolchain image for portable GPMP optimizer artifacts",
    ),
    SourceCheck(
        "pct_docker_build_copies_gpmp_kernel",
        PCT_DOCKER_BUILD,
        "COPY src/kernels/planning/gpmp_trajectory_optimizer",
        "PCT Docker build copies the Rust GPMP optimizer kernel from the repository root",
    ),
    SourceCheck(
        "pct_docker_builds_gpmp_release_artifacts",
        PCT_DOCKER_BUILD,
        "cargo build --release",
        "PCT Docker build compiles release Rust GPMP optimizer binary and library artifacts",
        min_count=2,
    ),
    SourceCheck(
        "pct_docker_outputs_gpmp_binary",
        PCT_DOCKER_BUILD,
        "gpmp_optimize",
        "PCT Docker build outputs the Rust optimizer executable used by rust_process runtime",
    ),
    SourceCheck(
        "pct_docker_outputs_gpmp_cdylib",
        PCT_DOCKER_BUILD,
        "liblingtu_gpmp_trajectory_optimizer.so",
        "PCT Docker build outputs the Rust optimizer dynamic library used by FFI runtime",
    ),
    SourceCheck(
        "pct_build_all_platforms_outputs_rust_artifacts",
        PCT_BUILD_ALL_PLATFORMS_SCRIPT,
        "algorithm/pct/runtime/rust",
        "PCT multi-platform build script writes Rust optimizer deployment artifacts into pct/runtime/rust/<arch>",
    ),
    SourceCheck(
        "pct_build_all_platforms_uses_repo_root_context",
        PCT_BUILD_ALL_PLATFORMS_SCRIPT,
        "-f src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/Dockerfile.build",
        "PCT multi-platform build script builds from the repository root so the Rust kernel is available to Docker",
    ),
    SourceCheck(
        "pct_native_cmake_native_modules_option_off",
        PCT_NATIVE_CMAKE,
        "LINGTU_PCT_BUILD_NATIVE_MODULES",
        "PCT standalone native CMake exposes an explicit native module build option",
        min_count=3,
    ),
    SourceCheck(
        "pct_native_cmake_legacy_gtsam_option_off",
        PCT_NATIVE_CMAKE,
        "LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE",
        "PCT standalone native CMake exposes an explicit legacy GTSAM comparison option",
        min_count=2,
    ),
    SourceCheck(
        "pct_native_cmake_legacy_gtsam_default_off",
        PCT_NATIVE_CMAKE,
        "OFF",
        "PCT standalone native CMake defaults legacy GTSAM comparison modules off",
    ),
    SourceCheck(
        "pct_native_cmake_gtsam_find_guard",
        PCT_NATIVE_CMAKE,
        "if(LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE)",
        "PCT standalone native CMake only discovers GTSAM inside the explicit legacy block",
    ),
    SourceCheck(
        "pct_legacy_native_build_requires_explicit_gtsam_flag",
        PCT_LEGACY_NATIVE_BUILD_SCRIPT,
        'if [[ "${BUILD_LEGACY_NATIVE}" != "1" ]]; then',
        "Host native GTSAM build helper is legacy-only and fails unless explicitly enabled",
    ),
    SourceCheck(
        "pct_legacy_native_build_enables_cmake_legacy_flag",
        PCT_LEGACY_NATIVE_BUILD_SCRIPT,
        "-DLINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=ON",
        "Host native GTSAM build helper passes the explicit legacy CMake option",
    ),
    SourceCheck(
        "pct_legacy_native_build_enables_native_modules",
        PCT_LEGACY_NATIVE_BUILD_SCRIPT,
        "-DLINGTU_PCT_BUILD_NATIVE_MODULES=ON",
        "Host native GTSAM build helper explicitly enables native C++ modules",
    ),
    SourceCheck(
        "pct_legacy_planner_build_requires_explicit_gtsam_flag",
        PCT_LEGACY_PLANNER_BUILD_SCRIPT,
        'if [[ "${BUILD_LEGACY_NATIVE}" != "1" ]]; then',
        "Original PCT planner build script is legacy-only and fails unless explicitly enabled",
    ),
    SourceCheck(
        "pct_legacy_planner_build_enables_cmake_legacy_flag",
        PCT_LEGACY_PLANNER_BUILD_SCRIPT,
        "-DLINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=ON",
        "Original PCT planner build script passes the explicit legacy CMake option",
    ),
    SourceCheck(
        "pct_legacy_planner_build_enables_native_modules",
        PCT_LEGACY_PLANNER_BUILD_SCRIPT,
        "-DLINGTU_PCT_BUILD_NATIVE_MODULES=ON",
        "Original PCT planner build script explicitly enables native C++ modules",
    ),
    SourceCheck(
        "pct_legacy_thirdparty_build_requires_explicit_gtsam_flag",
        PCT_LEGACY_THIRDPARTY_BUILD_SCRIPT,
        'if [[ "${BUILD_LEGACY_NATIVE}" != "1" ]]; then',
        "Original PCT third-party GTSAM build script is legacy-only and fails unless explicitly enabled",
    ),
    SourceCheck(
        "rust_optimizer_dense_sparse_compare_gate",
        RUST_GPMP_OPTIMIZER_COMPARE,
        "Compare Rust GPMP dense and block-tridiagonal optimizer paths",
        "Windows-safe dense-vs-block-tridiagonal performance and cost gate",
    ),
    SourceCheck(
        "pct_gpmp_math_acceptance_gate",
        PCT_GPMP_MATH_ACCEPTANCE,
        "Run the PCT GPMP Rust-vs-legacy-math acceptance gate",
        "Windows-safe Rust-vs-legacy GPMP math acceptance gate",
    ),
    SourceCheck(
        "pct_gpmp_math_acceptance_required_cases",
        PCT_GPMP_MATH_ACCEPTANCE,
        "REQUIRED_CASES",
        "PCT GPMP math acceptance gate requires all WNOJ/WNOA formula cases",
        min_count=2,
    ),
    SourceCheck(
        "pct_gpmp_math_acceptance_status_claim",
        PCT_GPMP_MATH_ACCEPTANCE,
        "pct_gpmp_math_kernel_parity",
        "PCT GPMP math acceptance gate feeds artifacts into migration status",
        min_count=2,
    ),
    SourceCheck(
        "rust_optimizer_compare_speedup_gate",
        RUST_GPMP_OPTIMIZER_COMPARE,
        "speedup_below_threshold",
        "Rust optimizer compare gate fails when sparse speedup is below threshold",
    ),
    SourceCheck(
        "rust_optimizer_compare_lm_gn_default",
        RUST_GPMP_OPTIMIZER_COMPARE,
        'DEFAULT_NONLINEAR_OPTIMIZERS = ("levenberg_marquardt", "gauss_newton")',
        "Rust optimizer compare gate covers both LM and GN by default",
    ),
    SourceCheck(
        "rust_optimizer_compare_missing_lm_gn_cases",
        RUST_GPMP_OPTIMIZER_COMPARE,
        'failed.append(f"{required_mode}:{required_optimizer}:missing_case")',
        "Rust optimizer compare gate fails when a WNOJ/WNOA LM/GN case is missing",
    ),
    SourceCheck(
        "rust_process_smoke_selects_optimizer_golden",
        PCT_RUST_PROCESS_SMOKE,
        "DEFAULT_GOLDENS_BY_OPTIMIZER",
        "Rust process smoke selects a golden fixture for the requested nonlinear optimizer",
    ),
    SourceCheck(
        "rust_process_smoke_gn_golden",
        PCT_RUST_PROCESS_GN_GOLDEN,
        '"last_optimizer_nonlinear_optimizer": "gauss_newton"',
        "Rust process smoke has a Gauss-Newton runtime golden fixture",
    ),
    SourceCheck(
        "pct_rust_runtime_acceptance_gate",
        PCT_RUST_RUNTIME_ACCEPTANCE,
        "Run the Windows-safe PCT Rust runtime acceptance gate",
        "Windows-safe end-to-end PCT Rust runtime acceptance gate",
    ),
    SourceCheck(
        "pct_rust_runtime_acceptance_lm_gn",
        PCT_RUST_RUNTIME_ACCEPTANCE,
        'DEFAULT_NONLINEAR_OPTIMIZERS = ("levenberg_marquardt", "gauss_newton")',
        "PCT Rust runtime acceptance gate runs both LM and GN",
    ),
    SourceCheck(
        "pct_rust_runtime_acceptance_status_claims",
        PCT_RUST_RUNTIME_ACCEPTANCE,
        "REQUIRED_STATUS_CLAIMS",
        "PCT Rust runtime acceptance gate feeds artifacts into migration status claims",
        min_count=2,
    ),
    SourceCheck(
        "pct_native_rust_parity_gate",
        PCT_NATIVE_RUST_PARITY,
        "Compare PCT native preview output with the Rust process runtime",
        "Linux same-input native-vs-rust_process PCT parity gate",
    ),
    SourceCheck(
        "pct_native_rust_parity_subprocess_isolation",
        PCT_NATIVE_RUST_PARITY,
        "run_preview_runtime(",
        "PCT native/rust parity runs each runtime in an isolated subprocess",
    ),
    SourceCheck(
        "pct_native_rust_parity_optimizer_accessor_flags",
        PCT_NATIVE_RUST_PARITY,
        "optimizer_accessors:{runtime}_{field}",
        "PCT native/rust parity requires optimizer accessor availability, finite values, and wrapper compatibility",
    ),
    SourceCheck(
        "pct_native_rust_parity_optimizer_accessor_shapes",
        PCT_NATIVE_RUST_PARITY,
        "optimizer_accessors:{field}",
        "PCT native/rust parity compares optimizer accessor output shapes",
        min_count=2,
    ),
)


FORBIDDEN_SOURCE_CHECKS: tuple[ForbiddenSourceCheck, ...] = (
    ForbiddenSourceCheck(
        "pct_docker_build_no_gtsam",
        PCT_DOCKER_BUILD,
        "gtsam",
        "PCT default Docker build must not compile or package GTSAM",
    ),
    ForbiddenSourceCheck(
        "pct_docker_build_no_libgtsam",
        PCT_DOCKER_BUILD,
        "libgtsam",
        "PCT default Docker build must not copy libgtsam into deployment artifacts",
    ),
    ForbiddenSourceCheck(
        "pct_docker_build_no_native_planner_lib_tree",
        PCT_DOCKER_BUILD,
        "planner/lib/3rdparty",
        "PCT default Docker build must not enter the legacy native planner third-party tree",
    ),
)


def _repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[2]


def _find_occurrences(path: Path, pattern: str) -> tuple[int | None, int]:
    if not path.is_file():
        return None, 0
    first_line: int | None = None
    count = 0
    for index, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        if pattern in line:
            count += 1
            if first_line is None:
                first_line = index
    return first_line, count


def _find_occurrences_casefold(path: Path, pattern: str) -> tuple[int | None, int]:
    if not path.is_file():
        return None, 0
    needle = pattern.casefold()
    first_line: int | None = None
    count = 0
    for index, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        if needle in line.casefold():
            count += 1
            if first_line is None:
                first_line = index
    return first_line, count


def build_contract(repo_root: Path) -> dict[str, Any]:
    root = repo_root.resolve()
    checks: list[CheckResult] = []
    for check in SOURCE_CHECKS:
        line, count = _find_occurrences(root / check.path, check.pattern)
        checks.append(
            CheckResult(
                id=check.id,
                path=check.path,
                capability=check.capability,
                required=check.required,
                present=count >= check.min_count,
                line=line,
                count=count,
                min_count=check.min_count,
            )
        )
    forbidden_checks: list[ForbiddenCheckResult] = []
    for check in FORBIDDEN_SOURCE_CHECKS:
        line, count = _find_occurrences_casefold(root / check.path, check.pattern)
        forbidden_checks.append(
            ForbiddenCheckResult(
                id=check.id,
                path=check.path,
                capability=check.capability,
                pattern=check.pattern,
                violated=count > 0,
                line=line,
                count=count,
            )
        )

    missing = [result for result in checks if result.required and not result.present]
    forbidden_violations = [result for result in forbidden_checks if result.violated]
    return {
        "schema": SCHEMA,
        "repo_root": str(root),
        "covered_by_pose_graph_opt": False,
        "reason_not_covered_by_pose_graph_opt": (
            "PCT/GPMP optimizes Vector4/Vector6 trajectory states with custom "
            "Gaussian-process, obstacle, and heading-rate factors, not SE3 "
            "Pose3 prior/between factors."
        ),
        "runtime_boundary": {
            "path": RUNTIME_BOUNDARY,
            "entrypoint": "load_pct_planner_runtime",
            "selector_env": "LINGTU_PCT_PLANNER_RUNTIME",
            "legacy_native_allow_env": "LINGTU_PCT_ALLOW_LEGACY_GTSAM_NATIVE",
            "default_runtime": "rust_process",
            "implemented_runtimes": ["native", "rust_process"],
            "rust_optimizer_call_env": "LINGTU_GPMP_OPTIMIZER_CALL",
            "rust_optimizer_call_modes": ["auto", "ffi", "process"],
            "future_runtimes": [],
        },
        "rust_math_kernel": {
            "path": RUST_GPMP_KERNEL,
            "crate": "lingtu_gpmp_trajectory_optimizer",
            "covered_capabilities": [
                "WNOJ Vector6 process Q/Phi/QInverse",
                "WNOA Vector4 process Q/Phi/QInverse",
                "GP prior residual Phi*x1 - x2",
                "GP interpolation Lambda*x1 + Psi*x2",
                "WNOJ heading-rate residual and Jacobian",
                "DenseElevationMap safe layer/height/bilinear obstacle query semantics",
                "WNOJ/WNOA obstacle residuals and Jacobians",
                "WNOJ/WNOA interpolated obstacle chain-rule Jacobians",
                "WNOJ/WNOA dense LM/GN batch optimizer for portable small graphs",
                "WNOJ/WNOA block-tridiagonal sparse LM/GN batch optimizer for trajectory-chain graphs",
                "WNOJ/WNOA faer sparse Cholesky LM/GN batch optimizer for non-chain graphs",
                "Rust process runtime can select LM or GN nonlinear optimization",
                "PCT runtime accepts explicit sparse GPMP linear solver requests",
                "Rust process runtime callable from PCT runtime on Windows/lightweight hosts",
                "Rust GPMP optimizer exposes an in-process C ABI for lower-overhead runtime calls",
                "PCT runtime auto-prefers Rust FFI and falls back to gpmp_optimize process when needed",
                "PCT default runtime uses Rust process on Linux/Windows/lightweight hosts",
                "PCT auto runtime prefers Rust process when the optimizer binary is available",
                "PCT auto runtime never falls back to legacy Linux/GTSAM native modules",
                "PCT native/GTSAM runtime requires an explicit legacy allow gate",
                "PCT runtime resolves packaged Rust optimizer artifacts from pct/runtime/rust/<arch>",
                "PCT legacy native loader no longer uses bundled planner/lib/x86_64 artifacts",
                "Build path generates the PCT Rust optimizer binary for rust_process runtime",
                "PCT Docker build packages Rust GPMP optimizer artifacts without GTSAM native modules",
                "Server setup builds and installs PCT Rust GPMP runtime artifacts by default",
                "PCT standalone native CMake skips C++ native modules by default",
                "PCT standalone native CMake skips GTSAM by default",
                "Legacy Linux/GTSAM native build helper requires an explicit comparison flag",
                "Original PCT planner and third-party GTSAM build scripts require an explicit legacy flag",
                "Rust process runtime validates optimizer schema, shapes, finite values, and solver report before accepting output",
                "Legacy PCT adapter runtime enters through the same native/Rust runtime selector",
                "Rust process native-like GPMP optimizer result accessors",
                "Rust process optimizer returns native-like interpolated trajectory output",
                "Rust process planner consumes expanded optimizer trajectory output for returned paths and accessors",
                "PCT preview/golden validates native wrapper optimizer accessor shape parity",
                "Linux native/rust_process parity gate compares optimizer accessor output shape parity",
                "Windows-safe Rust-vs-legacy GPMP math acceptance gate",
                "Windows-safe dense-vs-block-tridiagonal performance and cost gate",
                "Windows-safe end-to-end PCT Rust runtime acceptance gate",
                "Linux same-input native-vs-rust_process PCT parity gate",
            ],
            "not_covered_yet": [
                "Linux native pybind GPMPOptimizer removal",
                "passing Linux native PCT parity artifact against existing GTSAM optimizer",
            ],
        },
        "python_contract": {
            "planner_class": "TomogramPlanner",
            "plan_signature": "plan(start_pos, end_pos, start_height=0, end_height=0)",
            "required_result_fields": [
                "get_result_matrix",
                "get_layers",
                "get_heights",
                "get_opt_init_value",
                "get_opt_init_layer",
            ],
            "returned_path_shape": "Nx3 world coordinates [x, y, z]",
        },
        "required_capability_groups": [
            "Vector6 WNOJ batch nonlinear factor graph",
            "Vector4 WNOA batch nonlinear factor graph",
            "custom GP prior factors with analytic Jacobians",
            "interpolated obstacle factors over DenseElevationMap",
            "heading-rate constraints for WNOJ",
            "LM/GN-style sparse nonlinear least-squares solve",
            "GTSAM-free PCT Rust optimizer deployment packaging",
            "server setup default PCT runtime is Rust, not GTSAM native",
            "native wrapper output parity for result matrix, layers, and heights",
            "native-like interpolated trajectory output from Rust process optimizer",
            "in-process Rust C ABI runtime path for GPMP optimization",
            "Windows-safe Rust-vs-legacy GPMP math acceptance gate",
            "golden-comparable preview output parity for optimizer accessors",
            "Windows-safe end-to-end PCT Rust runtime acceptance gate",
            "Linux native/rust_process optimizer accessor output parity",
        ],
        "source_checks": [result.to_jsonable() for result in checks],
        "forbidden_source_checks": [
            result.to_jsonable() for result in forbidden_checks
        ],
        "missing_required_checks": [result.to_jsonable() for result in missing],
        "forbidden_source_violations": [
            result.to_jsonable() for result in forbidden_violations
        ],
        "ok": not missing and not forbidden_violations,
    }


def print_text_report(contract: dict[str, Any]) -> None:
    status = "OK" if contract["ok"] else "FAILED"
    print(f"PCT/GPMP migration contract: {status}")
    print(f"  schema: {contract['schema']}")
    print(f"  covered_by_pose_graph_opt: {contract['covered_by_pose_graph_opt']}")
    print(f"  runtime seam: {contract['runtime_boundary']['path']}::{contract['runtime_boundary']['entrypoint']}")
    print("  required capability groups:")
    for group in contract["required_capability_groups"]:
        print(f"    - {group}")
    print("  source checks:")
    for item in contract["source_checks"]:
        marker = "ok" if item["present"] else "missing"
        line = item["line"] if item["line"] is not None else "-"
        print(f"    {marker}: {item['id']} ({item['path']}:{line})")
    if contract["forbidden_source_violations"]:
        print("  forbidden source violations:")
        for item in contract["forbidden_source_violations"]:
            line = item["line"] if item["line"] is not None else "-"
            print(f"    violation: {item['id']} ({item['path']}:{line})")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=_repo_root_from_script())
    parser.add_argument("--json", action="store_true", help="emit JSON instead of text")
    args = parser.parse_args(argv)

    contract = build_contract(args.repo_root)
    if args.json:
        print(json.dumps(contract, indent=2, sort_keys=True))
    else:
        print_text_report(contract)
    return 0 if contract["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
