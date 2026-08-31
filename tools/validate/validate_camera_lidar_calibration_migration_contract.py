#!/usr/bin/env python3
"""Validate the static migration contract for camera-LiDAR calibration GTSAM use."""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCHEMA = "lingtu.camera_lidar_calibration_migration_contract.v1"

CALIB_ROOT = "tools/calibration/camera_lidar/direct_visual_lidar_calibration"
CT_ICP_HEADER = f"{CALIB_ROOT}/include/vlcal/common/integrated_ct_icp_factor.hpp"
CT_GICP_HEADER = f"{CALIB_ROOT}/include/vlcal/common/integrated_ct_gicp_factor.hpp"
CT_ICP_IMPL = f"{CALIB_ROOT}/include/vlcal/common/integrated_ct_icp_factor_impl.hpp"
CT_GICP_IMPL = f"{CALIB_ROOT}/include/vlcal/common/integrated_ct_gicp_factor_impl.hpp"
RUST_BRIDGE = f"{CALIB_ROOT}/include/vlcal/common/rust_camera_lidar_optimizer.hpp"
RUST_GTSAM_HELPER = (
    f"{CALIB_ROOT}/include/vlcal/common/rust_camera_lidar_optimizer_gtsam.hpp"
)
OUTLIERS_TEST = f"{CALIB_ROOT}/src/test/outliers.cpp"
DYNAMIC_INTEGRATOR = f"{CALIB_ROOT}/src/vlcal/preprocess/dynamic_point_cloud_integrator.cpp"
DYNAMIC_INTEGRATOR_HEADER = f"{CALIB_ROOT}/include/vlcal/preprocess/dynamic_point_cloud_integrator.hpp"
VISUAL_CALIBRATION = f"{CALIB_ROOT}/src/vlcal/calib/visual_camera_calibration.cpp"
CALIB_CMAKE = f"{CALIB_ROOT}/CMakeLists.txt"
RUST_KERNEL = "src/kernels/calibration/camera_lidar_optimizer/src/lib.rs"
RUST_MANIFEST = "src/kernels/calibration/camera_lidar_optimizer/Cargo.toml"
RUST_ABI_SMOKE = "tools/bench/camera_lidar_optimizer_abi_smoke.py"
HUMBLE_DOCKERFILE = f"{CALIB_ROOT}/docker/humble/Dockerfile"
HUMBLE_SUPERGLUE_DOCKERFILE = f"{CALIB_ROOT}/docker/humble/Dockerfile_with_superglue"
JAZZY_DOCKERFILE = f"{CALIB_ROOT}/docker/jazzy/Dockerfile"
JAZZY_SUPERGLUE_DOCKERFILE = f"{CALIB_ROOT}/docker/jazzy/Dockerfile_with_superglue"
NOETIC_DOCKERFILE = f"{CALIB_ROOT}/docker/noetic/Dockerfile"
NOETIC_SUPERGLUE_DOCKERFILE = f"{CALIB_ROOT}/docker/noetic/Dockerfile_with_superglue"

DOCKERFILES: tuple[tuple[str, str], ...] = (
    ("humble", HUMBLE_DOCKERFILE),
    ("humble_superglue", HUMBLE_SUPERGLUE_DOCKERFILE),
    ("jazzy", JAZZY_DOCKERFILE),
    ("jazzy_superglue", JAZZY_SUPERGLUE_DOCKERFILE),
    ("noetic", NOETIC_DOCKERFILE),
    ("noetic_superglue", NOETIC_SUPERGLUE_DOCKERFILE),
)


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
    forbidden_pattern: str
    present: bool
    line: int | None
    count: int

    def to_jsonable(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "path": self.path,
            "capability": self.capability,
            "forbidden_pattern": self.forbidden_pattern,
            "present": self.present,
            "line": self.line,
            "count": self.count,
        }


SOURCE_CHECKS: tuple[SourceCheck, ...] = (
    SourceCheck(
        "calibration_cmake_gtsam_free_define",
        CALIB_CMAKE,
        "LINGTU_CAMERA_LIDAR_GTSAM_FREE_BUILD",
        "calibration build records a GTSAM-free compile mode",
    ),
    SourceCheck(
        "calibration_cmake_rust_optimizer_default_on",
        CALIB_CMAKE,
        "set(LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER_DEFAULT ON)",
        "default calibration build selects the Rust optimizer path",
    ),
    SourceCheck(
        "calibration_cmake_rust_optimizer_option",
        CALIB_CMAKE,
        "LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER",
        "calibration build exposes an explicit switch for legacy comparison runs",
    ),
    SourceCheck(
        "calibration_cmake_rust_optimizer_link",
        CALIB_CMAKE,
        "LINGTU_CAMERA_LIDAR_OPTIMIZER_LIBRARY",
        "calibration build can link a prebuilt Rust camera-LiDAR optimizer library",
    ),
    SourceCheck(
        "calibration_cmake_rust_optimizer_cargo_build",
        CALIB_CMAKE,
        "add_custom_target(camera_lidar_optimizer_rust",
        "calibration CMake builds the Rust camera-LiDAR optimizer when no prebuilt library is supplied",
    ),
    SourceCheck(
        "calibration_cmake_rust_optimizer_imported_target",
        CALIB_CMAKE,
        "add_library(lingtu_camera_lidar_optimizer STATIC IMPORTED)",
        "calibration CMake imports the cargo-built Rust static library",
    ),
    SourceCheck(
        "calibration_cmake_rust_optimizer_target_link",
        CALIB_CMAKE,
        "target_link_libraries(direct_visual_lidar_calibration lingtu_camera_lidar_optimizer",
        "calibration library links the Rust optimizer target in the default build path",
    ),
    SourceCheck(
        "calibration_cmake_rust_optimizer_required_option",
        CALIB_CMAKE,
        "LINGTU_CAMERA_LIDAR_REQUIRE_RUST_OPTIMIZER",
        "calibration build can require the Rust optimizer instead of allowing silent GTSAM LM fallback",
    ),
    SourceCheck(
        "calibration_cmake_rust_optimizer_required_define",
        CALIB_CMAKE,
        "target_compile_definitions(direct_visual_lidar_calibration PRIVATE LINGTU_CAMERA_LIDAR_REQUIRE_RUST_OPTIMIZER)",
        "calibration build forwards Rust-required mode into the dynamic integrator runtime",
    ),
    SourceCheck(
        "calibration_humble_docker_ros_base",
        HUMBLE_DOCKERFILE,
        "FROM ros:humble-ros-base",
        "Humble calibration Docker image uses an official ROS base instead of a GTSAM image",
    ),
    SourceCheck(
        "calibration_humble_superglue_docker_ros_base",
        HUMBLE_SUPERGLUE_DOCKERFILE,
        "FROM ros:humble-ros-base",
        "Humble+SuperGlue calibration Docker image uses an official ROS base instead of a GTSAM image",
    ),
    SourceCheck(
        "calibration_jazzy_docker_ros_base",
        JAZZY_DOCKERFILE,
        "FROM ros:jazzy-ros-base",
        "Jazzy calibration Docker image uses an official ROS base instead of a GTSAM image",
    ),
    SourceCheck(
        "calibration_jazzy_superglue_docker_ros_base",
        JAZZY_SUPERGLUE_DOCKERFILE,
        "FROM ros:jazzy-ros-base",
        "Jazzy+SuperGlue calibration Docker image uses an official ROS base instead of a GTSAM image",
    ),
    SourceCheck(
        "calibration_noetic_docker_ros_base",
        NOETIC_DOCKERFILE,
        "FROM ros:noetic-ros-base",
        "Noetic calibration Docker image uses an official ROS base instead of a GTSAM image",
    ),
    SourceCheck(
        "calibration_noetic_superglue_docker_ros_base",
        NOETIC_SUPERGLUE_DOCKERFILE,
        "FROM ros:noetic-ros-base",
        "Noetic+SuperGlue calibration Docker image uses an official ROS base instead of a GTSAM image",
    ),
    SourceCheck(
        "calibration_humble_docker_rust_toolchain",
        HUMBLE_DOCKERFILE,
        "rustc",
        "Humble calibration Docker image installs the Rust optimizer toolchain",
    ),
    SourceCheck(
        "calibration_humble_superglue_docker_rust_toolchain",
        HUMBLE_SUPERGLUE_DOCKERFILE,
        "rustc",
        "Humble+SuperGlue calibration Docker image installs the Rust optimizer toolchain",
    ),
    SourceCheck(
        "calibration_jazzy_docker_rust_toolchain",
        JAZZY_DOCKERFILE,
        "rustc",
        "Jazzy calibration Docker image installs the Rust optimizer toolchain",
    ),
    SourceCheck(
        "calibration_jazzy_superglue_docker_rust_toolchain",
        JAZZY_SUPERGLUE_DOCKERFILE,
        "rustc",
        "Jazzy+SuperGlue calibration Docker image installs the Rust optimizer toolchain",
    ),
    SourceCheck(
        "calibration_noetic_docker_rust_toolchain",
        NOETIC_DOCKERFILE,
        "rustc",
        "Noetic calibration Docker image installs the Rust optimizer toolchain",
    ),
    SourceCheck(
        "calibration_noetic_superglue_docker_rust_toolchain",
        NOETIC_SUPERGLUE_DOCKERFILE,
        "rustc",
        "Noetic+SuperGlue calibration Docker image installs the Rust optimizer toolchain",
    ),
    SourceCheck(
        "calibration_humble_docker_iridescence_build",
        HUMBLE_DOCKERFILE,
        "https://github.com/koide3/iridescence",
        "Humble calibration Docker image builds Iridescence explicitly instead of inheriting it from a GTSAM image",
    ),
    SourceCheck(
        "calibration_humble_superglue_docker_iridescence_build",
        HUMBLE_SUPERGLUE_DOCKERFILE,
        "https://github.com/koide3/iridescence",
        "Humble+SuperGlue calibration Docker image builds Iridescence explicitly instead of inheriting it from a GTSAM image",
    ),
    SourceCheck(
        "calibration_jazzy_docker_iridescence_build",
        JAZZY_DOCKERFILE,
        "https://github.com/koide3/iridescence",
        "Jazzy calibration Docker image builds Iridescence explicitly instead of inheriting it from a GTSAM image",
    ),
    SourceCheck(
        "calibration_jazzy_superglue_docker_iridescence_build",
        JAZZY_SUPERGLUE_DOCKERFILE,
        "https://github.com/koide3/iridescence",
        "Jazzy+SuperGlue calibration Docker image builds Iridescence explicitly instead of inheriting it from a GTSAM image",
    ),
    SourceCheck(
        "calibration_noetic_docker_iridescence_build",
        NOETIC_DOCKERFILE,
        "https://github.com/koide3/iridescence",
        "Noetic calibration Docker image builds Iridescence explicitly instead of inheriting it from a GTSAM image",
    ),
    SourceCheck(
        "calibration_noetic_superglue_docker_iridescence_build",
        NOETIC_SUPERGLUE_DOCKERFILE,
        "https://github.com/koide3/iridescence",
        "Noetic+SuperGlue calibration Docker image builds Iridescence explicitly instead of inheriting it from a GTSAM image",
    ),
    SourceCheck(
        "dynamic_integrator_header_uses_eigen_pose_storage",
        DYNAMIC_INTEGRATOR_HEADER,
        "ConcurrentQueue<std::tuple<Frame::ConstPtr, Eigen::Isometry3d, Eigen::Isometry3d>>",
        "dynamic point cloud integrator header stores poses with Eigen instead of exposing GTSAM Pose3",
    ),
    SourceCheck(
        "dynamic_integrator_cpp_rt_interpolation",
        DYNAMIC_INTEGRATOR,
        "interpolate_rt",
        "dynamic point cloud integrator replaces Pose3::interpolateRt with local rotation/translation interpolation",
        min_count=2,
    ),
    SourceCheck(
        "dynamic_integrator_rust_optimizer_runtime",
        DYNAMIC_INTEGRATOR,
        "optimize_dynamic_ct_gicp_with_rust",
        "dynamic point cloud integrator can directly call the Rust CT-GICP optimizer runtime",
    ),
    SourceCheck(
        "dynamic_integrator_rust_primary_pose_update",
        DYNAMIC_INTEGRATOR,
        "*optimized_begin = from_ffi_pose(result.pose0);",
        "dynamic point cloud integrator uses Rust optimizer output as the primary scan begin/end pose update",
    ),
    SourceCheck(
        "dynamic_integrator_direct_dynamic_optimizer_abi",
        DYNAMIC_INTEGRATOR,
        "lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose",
        "dynamic point cloud integrator delegates CT-GICP correspondence rebuild and optimization loop to one Rust C ABI call",
    ),
    SourceCheck(
        "dynamic_integrator_rust_required_runtime_guard",
        DYNAMIC_INTEGRATOR,
        "if (!optimized_with_rust)",
        "dynamic point cloud integrator enforces Rust optimizer success at runtime",
    ),
    SourceCheck(
        "dynamic_integrator_rust_required_no_silent_fallback",
        DYNAMIC_INTEGRATOR,
        "Dynamic point cloud integration requires the Rust CT-GICP optimizer",
        "dynamic point cloud integrator fails instead of silently falling back to a legacy optimizer",
    ),
    SourceCheck(
        "visual_calibration_uses_sophus_expmap",
        VISUAL_CALIBRATION,
        "Sophus::SE3d::exp",
        "visual camera calibration uses Sophus SE3 expmap for Nelder-Mead perturbations",
    ),
    SourceCheck(
        "visual_calibration_preserves_rotation_first_order",
        VISUAL_CALIBRATION,
        "expmap_rotation_first_order",
        "visual camera calibration preserves the legacy [rx, ry, rz, tx, ty, tz] tangent order",
        min_count=2,
    ),
    SourceCheck(
        "rust_camera_lidar_manifest",
        RUST_MANIFEST,
        "name = \"lingtu_camera_lidar_optimizer\"",
        "portable Rust camera-LiDAR optimizer crate exists",
    ),
    SourceCheck(
        "rust_camera_lidar_pose3",
        RUST_KERNEL,
        "pub struct Pose3",
        "Rust kernel provides SE3 pose math for calibration factors",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_icp_correspondence",
        RUST_KERNEL,
        "pub struct CtIcpCorrespondence",
        "Rust kernel defines fixed-correspondence CT-ICP inputs",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_icp_linearize",
        RUST_KERNEL,
        "pub fn linearize_ct_icp_fixed_correspondences",
        "Rust kernel linearizes CT-ICP point-to-plane residuals",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_icp_hessian",
        RUST_KERNEL,
        "pub fn hessian_12x12",
        "Rust kernel exports a two-pose Hessian assembly shape",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_icp_time_derivative_cache",
        RUST_KERNEL,
        "derivative_cache",
        "Rust kernel caches continuous-time endpoint derivatives by normalized timestamp",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_icp_jacobian_test",
        RUST_KERNEL,
        "ct_icp_jacobians_match_residual_finite_difference",
        "Rust tests check CT-ICP endpoint Jacobians against residual finite differences",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_gicp_correspondence",
        RUST_KERNEL,
        "pub struct CtGicpCorrespondence",
        "Rust kernel defines fixed-correspondence CT-GICP inputs with covariances",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_gicp_source_point",
        RUST_KERNEL,
        "pub struct CtGicpSourcePoint",
        "Rust kernel defines source points for CT-GICP nearest-neighbor correspondence search",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_gicp_target_point",
        RUST_KERNEL,
        "pub struct CtGicpTargetPoint",
        "Rust kernel defines target points for CT-GICP nearest-neighbor correspondence search",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_gicp_nearest_neighbor_builder",
        RUST_KERNEL,
        "pub fn build_ct_gicp_nearest_neighbor_correspondences",
        "Rust kernel owns CT-GICP nearest-neighbor correspondence search and max-distance rejection",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_gicp_linearize",
        RUST_KERNEL,
        "pub fn linearize_ct_gicp_fixed_correspondences",
        "Rust kernel linearizes CT-GICP Mahalanobis residuals",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_gicp_mahalanobis",
        RUST_KERNEL,
        "fn ct_gicp_mahalanobis",
        "Rust kernel builds fused CT-GICP covariance information matrices",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_gicp_jacobian_test",
        RUST_KERNEL,
        "ct_gicp_jacobians_match_cost_finite_difference_with_fixed_information",
        "Rust tests check CT-GICP endpoint gradients against cost finite differences",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_version",
        RUST_KERNEL,
        "pub extern \"C\" fn lingtu_camera_lidar_optimizer_abi_version",
        "Rust kernel exposes a stable C ABI version query",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_ct_icp",
        RUST_KERNEL,
        "pub extern \"C\" fn lingtu_camera_lidar_optimizer_linearize_ct_icp",
        "Rust kernel exposes CT-ICP linearization through C ABI",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_ct_gicp",
        RUST_KERNEL,
        "pub extern \"C\" fn lingtu_camera_lidar_optimizer_linearize_ct_gicp",
        "Rust kernel exposes CT-GICP linearization through C ABI",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_ct_gicp_correspondence_builder",
        RUST_KERNEL,
        "pub extern \"C\" fn lingtu_camera_lidar_optimizer_build_ct_gicp_correspondences",
        "Rust kernel exposes CT-GICP nearest-neighbor correspondence construction through C ABI",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_gicp_optimizer",
        RUST_KERNEL,
        "pub fn optimize_ct_gicp_two_pose",
        "Rust kernel optimizes the dynamic integrator two-pose CT-GICP problem",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_gicp_dynamic_optimizer",
        RUST_KERNEL,
        "pub fn optimize_ct_gicp_dynamic_two_pose",
        "Rust kernel owns the dynamic CT-GICP correspondence rebuild and two-pose optimization loop",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_ct_gicp_optimizer",
        RUST_KERNEL,
        "pub extern \"C\" fn lingtu_camera_lidar_optimizer_optimize_ct_gicp_two_pose",
        "Rust kernel exposes CT-GICP two-pose optimization through C ABI",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_ct_gicp_dynamic_optimizer",
        RUST_KERNEL,
        "pub extern \"C\" fn lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose",
        "Rust kernel exposes dynamic CT-GICP scan optimization through C ABI",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_test",
        RUST_KERNEL,
        "abi_linearizes_ct_gicp",
        "Rust tests verify C ABI CT-GICP output",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_optimizer_test",
        RUST_KERNEL,
        "abi_optimizes_ct_gicp_two_pose",
        "Rust tests verify C ABI CT-GICP optimizer output",
    ),
    SourceCheck(
        "rust_camera_lidar_dynamic_optimizer_test",
        RUST_KERNEL,
        "ct_gicp_dynamic_two_pose_optimizer_rebuilds_correspondences_and_recovers_translation",
        "Rust tests verify dynamic CT-GICP optimizer correspondence rebuild and pose recovery",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_dynamic_optimizer_test",
        RUST_KERNEL,
        "abi_optimizes_ct_gicp_dynamic_two_pose",
        "Rust tests verify the dynamic CT-GICP optimizer C ABI output",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_smoke_gate",
        RUST_ABI_SMOKE,
        "lingtu.camera_lidar_optimizer_abi_smoke.v1",
        "Windows-safe ctypes smoke gate loads the Rust camera-LiDAR optimizer library",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_smoke_checks_abi_version",
        RUST_ABI_SMOKE,
        "EXPECTED_ABI_VERSION = 2",
        "camera-LiDAR ABI smoke gate checks the stable ABI version",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_smoke_optimizes_ct_gicp",
        RUST_ABI_SMOKE,
        "lingtu_camera_lidar_optimizer_optimize_ct_gicp_two_pose",
        "camera-LiDAR ABI smoke gate exercises CT-GICP two-pose optimization",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_smoke_optimizes_dynamic_ct_gicp",
        RUST_ABI_SMOKE,
        "lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose",
        "camera-LiDAR ABI smoke gate exercises dynamic CT-GICP scan optimization",
    ),
    SourceCheck(
        "rust_camera_lidar_ct_gicp_nearest_neighbor_test",
        RUST_KERNEL,
        "ct_gicp_nearest_neighbor_correspondences_selects_closest_target",
        "Rust tests verify CT-GICP nearest-neighbor correspondence selection",
    ),
    SourceCheck(
        "rust_camera_lidar_abi_correspondence_builder_test",
        RUST_KERNEL,
        "abi_builds_ct_gicp_correspondences",
        "Rust tests verify C ABI CT-GICP correspondence builder output",
    ),
    SourceCheck(
        "cpp_rust_camera_lidar_bridge_header",
        RUST_BRIDGE,
        "struct LingtuCameraLidarTwoPoseLinearization",
        "C++ calibration bridge declares Rust ABI-compatible linearization structs",
    ),
    SourceCheck(
        "cpp_rust_camera_lidar_bridge_correspondence_builder",
        RUST_BRIDGE,
        "lingtu_camera_lidar_optimizer_build_ct_gicp_correspondences",
        "C++ calibration bridge declares the Rust CT-GICP correspondence builder C ABI",
    ),
    SourceCheck(
        "cpp_rust_camera_lidar_bridge_dynamic_optimizer",
        RUST_BRIDGE,
        "lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose",
        "C++ calibration bridge declares the dynamic CT-GICP optimizer C ABI",
    ),
    SourceCheck(
        "cpp_rust_camera_lidar_bridge_optimizer_structs",
        RUST_BRIDGE,
        "struct LingtuCameraLidarTwoPoseOptimizationResult",
        "C++ calibration bridge declares Rust ABI-compatible optimizer structs",
    ),
    SourceCheck(
        "cpp_rust_camera_lidar_bridge_abi_guard",
        RUST_BRIDGE,
        "abi_compatible",
        "C++ calibration bridge checks Rust ABI version and struct sizes before use",
    ),
)


FORBIDDEN_SOURCE_CHECKS: tuple[ForbiddenSourceCheck, ...] = (
    ForbiddenSourceCheck(
        "calibration_cmake_no_gtsam_find_package",
        CALIB_CMAKE,
        "find_package(GTSAM",
        "camera-LiDAR CMake does not discover or require GTSAM",
    ),
    ForbiddenSourceCheck(
        "calibration_cmake_no_gtsam_include_dirs",
        CALIB_CMAKE,
        "GTSAM_INCLUDE_DIRS",
        "camera-LiDAR CMake does not add GTSAM include paths",
    ),
    ForbiddenSourceCheck(
        "calibration_cmake_no_gtsam_libraries",
        CALIB_CMAKE,
        "GTSAM_LIBRARIES",
        "camera-LiDAR CMake does not link GTSAM libraries",
    ),
    ForbiddenSourceCheck(
        "dynamic_integrator_no_gtsam_include",
        DYNAMIC_INTEGRATOR,
        "#include <gtsam/",
        "dynamic point cloud integrator does not include GTSAM headers",
    ),
    ForbiddenSourceCheck(
        "dynamic_integrator_no_gtsam_namespace",
        DYNAMIC_INTEGRATOR,
        "gtsam::",
        "dynamic point cloud integrator does not call GTSAM APIs",
    ),
    ForbiddenSourceCheck(
        "dynamic_integrator_no_ct_gicp_factor_fallback",
        DYNAMIC_INTEGRATOR,
        "IntegratedCT_GICPFactor_",
        "dynamic point cloud integrator no longer routes fallback optimization through legacy factors",
    ),
    ForbiddenSourceCheck(
        "dynamic_integrator_no_lm_fallback_guard",
        DYNAMIC_INTEGRATOR,
        "LINGTU_CAMERA_LIDAR_HAS_GTSAM_LM_FALLBACK",
        "dynamic point cloud integrator has no legacy LM fallback compile path",
    ),
    ForbiddenSourceCheck(
        "dynamic_integrator_no_factor_graph",
        DYNAMIC_INTEGRATOR,
        "NonlinearFactorGraph",
        "dynamic point cloud integrator has no legacy nonlinear graph fallback",
    ),
    ForbiddenSourceCheck(
        "dynamic_integrator_no_legacy_lm_optimizer",
        DYNAMIC_INTEGRATOR,
        "LevenbergMarquardtOptimizer",
        "dynamic point cloud integrator has no legacy LM optimizer fallback",
    ),
    ForbiddenSourceCheck(
        "dynamic_integrator_no_cpp_ct_gicp_knn_search",
        DYNAMIC_INTEGRATOR,
        "target_ivox->knn_search",
        "dynamic point cloud integrator no longer owns CT-GICP nearest-neighbor search in the Rust runtime path",
    ),
    ForbiddenSourceCheck(
        "dynamic_integrator_no_cpp_dynamic_ct_gicp_outer_loop",
        DYNAMIC_INTEGRATOR,
        "for (int outer = 0; outer < 3; outer++)",
        "dynamic point cloud integrator no longer owns the CT-GICP correspondence rebuild optimization loop",
    ),
    ForbiddenSourceCheck(
        "dynamic_integrator_no_cpp_rust_correspondence_builder_helper",
        DYNAMIC_INTEGRATOR,
        "build_rust_correspondences",
        "dynamic point cloud integrator delegates correspondence rebuilds to the Rust dynamic optimizer ABI",
    ),
    ForbiddenSourceCheck(
        "cpp_rust_camera_lidar_bridge_no_gtsam_include",
        RUST_BRIDGE,
        "#include <gtsam/",
        "pure C++ Rust ABI bridge does not include GTSAM headers",
    ),
    ForbiddenSourceCheck(
        "cpp_rust_camera_lidar_bridge_no_gtsam_namespace",
        RUST_BRIDGE,
        "gtsam::",
        "pure C++ Rust ABI bridge does not call GTSAM APIs",
    ),
) + tuple(
    ForbiddenSourceCheck(
        f"calibration_{name}_docker_no_{suffix}",
        path,
        pattern,
        capability,
    )
    for name, path in DOCKERFILES
    for suffix, pattern, capability in (
        (
            "gtsam_base",
            "gtsam_docker",
            "camera-LiDAR calibration Docker image does not inherit from a GTSAM base image",
        ),
        (
            "legacy_gtsam_arg",
            "LINGTU_CAMERA_LIDAR_LEGACY_GTSAM_DOCKER",
            "camera-LiDAR calibration Docker image does not expose a legacy GTSAM build arg",
        ),
        (
            "apt_fast",
            "apt-fast",
            "camera-LiDAR calibration Docker image does not depend on apt-fast from the legacy base",
        ),
    )
)

REMOVED_GTSAM_BUILD_FILES: tuple[str, ...] = (
    f"{CALIB_ROOT}/cmake/FindGTSAM.cmake",
)

REMOVED_LEGACY_GTSAM_SOURCE_FILES: tuple[str, ...] = (
    CT_ICP_HEADER,
    CT_ICP_IMPL,
    CT_GICP_HEADER,
    CT_GICP_IMPL,
    RUST_GTSAM_HELPER,
    OUTLIERS_TEST,
)

DEFAULT_LIBRARY_FORBIDDEN_PATTERNS: tuple[str, ...] = (
    "#include <gtsam/",
    "gtsam::",
    "#include <vlcal/common/integrated_ct_",
    "IntegratedCT_",
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


def _direct_visual_lidar_library_sources(root: Path) -> list[str]:
    cmake = root / CALIB_CMAKE
    if not cmake.is_file():
        return []

    sources: list[str] = []
    in_target = False
    for raw_line in cmake.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not in_target:
            if line.startswith("add_library(direct_visual_lidar_calibration"):
                in_target = True
            continue

        if line == ")":
            break

        token = line.rstrip(")")
        if re.match(r"^src/.+\.(c|cc|cpp|cxx)$", token):
            sources.append(f"{CALIB_ROOT}/{token}")
    return sources


def _audit_default_library_sources(root: Path) -> dict[str, Any]:
    sources = _direct_visual_lidar_library_sources(root)
    violations: list[dict[str, Any]] = []
    for source in sources:
        path = root / source
        if not path.is_file():
            violations.append(
                {
                    "path": source,
                    "line": None,
                    "pattern": "<missing source file>",
                    "text": "",
                }
            )
            continue
        for line_number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
            for pattern in DEFAULT_LIBRARY_FORBIDDEN_PATTERNS:
                if pattern in line:
                    violations.append(
                        {
                            "path": source,
                            "line": line_number,
                            "pattern": pattern,
                            "text": line.strip()[:240],
                        }
                    )
    return {
        "target": "direct_visual_lidar_calibration",
        "source_count": len(sources),
        "sources": sources,
        "forbidden_patterns": list(DEFAULT_LIBRARY_FORBIDDEN_PATTERNS),
        "missing_source_list": not sources,
        "violations": violations,
        "ok": bool(sources) and not violations,
    }


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
        line, count = _find_occurrences(root / check.path, check.pattern)
        forbidden_checks.append(
            ForbiddenCheckResult(
                id=check.id,
                path=check.path,
                capability=check.capability,
                forbidden_pattern=check.pattern,
                present=count > 0,
                line=line,
                count=count,
            )
        )

    removed_files = [
        {
            "path": path,
            "removed": not (root / path).exists(),
        }
        for path in (*REMOVED_GTSAM_BUILD_FILES, *REMOVED_LEGACY_GTSAM_SOURCE_FILES)
    ]
    default_library_source_audit = _audit_default_library_sources(root)

    missing = [result for result in checks if result.required and not result.present]
    forbidden_present = [result for result in forbidden_checks if result.present]
    removed_missing = [item for item in removed_files if not item["removed"]]
    return {
        "schema": SCHEMA,
        "repo_root": str(root),
        "current_gtsam_surfaces": {
            "build_packaging": [],
            "dynamic_point_cloud_integrator": [],
            "legacy_factor_headers": [],
            "visual_camera_calibration": [],
        },
        "removed_legacy_gtsam_sources": list(REMOVED_LEGACY_GTSAM_SOURCE_FILES),
        "rust_kernel": {
            "path": RUST_KERNEL,
            "crate": "lingtu_camera_lidar_optimizer",
            "covered_capabilities": [
                "SE3 exp/log/compose/inverse/interpolate_rt for calibration factors",
                "CT-ICP fixed-correspondence point-to-plane residuals",
                "CT-ICP endpoint Jacobians through continuous-time SE3 interpolation",
                "CT-ICP two-pose Hessian/rhs assembly matching HessianFactor shape",
                "CT-GICP fixed-correspondence covariance-weighted Mahalanobis residuals",
                "CT-GICP endpoint Jacobians through continuous-time SE3 interpolation",
                "CT-GICP two-pose Hessian/rhs assembly matching HessianFactor shape",
                "Rust-owned nearest-neighbor correspondence search and max-distance rejection",
                "singular covariance rejection for CT-GICP fused information",
                "C ABI for CT-ICP/CT-GICP two-pose linearization calls",
                "C ABI for CT-GICP nearest-neighbor correspondence construction calls",
                "CT-GICP two-pose LM optimizer with prior and between constraints",
                "C ABI for CT-GICP two-pose optimization calls",
                "Rust-owned dynamic CT-GICP correspondence rebuild and two-pose optimization loop",
                "C ABI for dynamic CT-GICP scan optimization calls",
                "pure C++ Rust ABI header declares optimizer and correspondence entry points without GTSAM",
                "dynamic point cloud integrator delegates CT-GICP correspondence rebuild and optimization loop to one Rust C ABI call",
                "default calibration build is Rust-required and skips GTSAM discovery/linkage",
                "dynamic point cloud integrator has no GTSAM runtime fallback source path",
                "dynamic point cloud integrator no longer owns CT-GICP nearest-neighbor search in the Rust runtime path",
                "dynamic point cloud integrator no longer owns the CT-GICP correspondence rebuild optimization loop",
                "camera-LiDAR CMake has no GTSAM discovery/linkage path",
                "Rust-required calibration CMake path skips GTSAM discovery/linkage",
                "camera-LiDAR calibration Docker images use ROS base images, Rust/Cargo, and explicit Ceres/Iridescence builds without GTSAM",
                "legacy CT-ICP/CT-GICP GTSAM factor headers are removed from the repository",
                "legacy Rust-to-GTSAM helper header is removed from the repository",
                "legacy camera-LiDAR GTSAM test support source is removed from the repository",
                "dynamic point cloud integrator header stores Eigen poses instead of exposing GTSAM Pose3",
                "visual camera calibration Nelder-Mead perturbations use Sophus SE3 expmap without GTSAM",
                "default camera-LiDAR shared library source list is GTSAM-free and excludes legacy CT factor headers",
                "C++ Rust bridge guards ABI version and struct sizes before factor calls",
                "pure C++ Rust ABI header is GTSAM-free; legacy GTSAM helpers are removed",
                "timestamp-keyed endpoint derivative cache for repeated source times",
                "Windows-safe ctypes smoke gate loads the Rust camera-LiDAR optimizer library and exercises CT-GICP optimization",
            ],
            "not_covered_yet": [],
        },
        "required_capability_groups": [
            "classify calibration GTSAM build/runtime surfaces",
            "portable CT-ICP residual/Jacobian/Hessian math kernel",
            "portable CT-GICP Mahalanobis residual/Jacobian/Hessian math kernel",
            "stable C ABI entry points for calibration factor linearization",
            "stable C ABI entry point for CT-GICP Rust-owned correspondence construction",
            "stable C ABI entry point for CT-GICP two-pose calibration optimization",
            "stable C ABI entry point for dynamic CT-GICP scan optimization",
            "pure C++ Rust ABI bridge for runtime optimizer calls",
            "direct dynamic point cloud integrator runtime call to Rust ABI",
            "default calibration build path uses the Rust optimizer without GTSAM",
            "default calibration Docker images are GTSAM-free",
            "default direct_visual_lidar_calibration shared library source audit excludes legacy GTSAM factors",
            "legacy CT factor headers and GTSAM helper sources are removed",
            "dynamic point cloud integrator prevents silent runtime fallback",
            "Windows-safe camera-LiDAR Rust C ABI smoke verification",
        ],
        "source_checks": [result.to_jsonable() for result in checks],
        "forbidden_source_checks": [result.to_jsonable() for result in forbidden_checks],
        "default_library_source_audit": default_library_source_audit,
        "removed_gtsam_build_files": removed_files,
        "missing_required_checks": [result.to_jsonable() for result in missing],
        "forbidden_present_checks": [result.to_jsonable() for result in forbidden_present],
        "default_library_source_violations": default_library_source_audit["violations"],
        "missing_removed_gtsam_build_files": removed_missing,
        "ok": (
            not missing
            and not forbidden_present
            and not removed_missing
            and default_library_source_audit["ok"]
        ),
    }


def print_text_report(contract: dict[str, Any]) -> None:
    status = "OK" if contract["ok"] else "FAILED"
    print(f"Camera-LiDAR calibration migration contract: {status}")
    print(f"  schema: {contract['schema']}")
    print(f"  Rust kernel: {contract['rust_kernel']['path']}")
    print("  required capability groups:")
    for group in contract["required_capability_groups"]:
        print(f"    - {group}")
    print("  source checks:")
    for item in contract["source_checks"]:
        marker = "ok" if item["present"] else "missing"
        line = item["line"] if item["line"] is not None else "-"
        print(f"    {marker}: {item['id']} ({item['path']}:{line})")
    if contract["forbidden_present_checks"]:
        print("  forbidden source checks:")
        for item in contract["forbidden_present_checks"]:
            line = item["line"] if item["line"] is not None else "-"
            print(f"    present: {item['id']} ({item['path']}:{line})")
    source_audit = contract["default_library_source_audit"]
    source_status = "ok" if source_audit["ok"] else "failed"
    print(
        "  default library source audit: "
        f"{source_status} ({source_audit['source_count']} source file(s))"
    )
    for item in source_audit["violations"]:
        line = item["line"] if item["line"] is not None else "-"
        print(f"    violation: {item['path']}:{line} contains {item['pattern']}")
    if contract["missing_removed_gtsam_build_files"]:
        print("  files expected to be removed:")
        for item in contract["missing_removed_gtsam_build_files"]:
            print(f"    present: {item['path']}")


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
