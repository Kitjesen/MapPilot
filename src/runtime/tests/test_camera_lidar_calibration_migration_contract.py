from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
CONTRACT_SCRIPT = (
    ROOT / "tools" / "validate" / "validate_camera_lidar_calibration_migration_contract.py"
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "validate_camera_lidar_calibration_migration_contract",
        CONTRACT_SCRIPT,
    )
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_contract_declares_camera_lidar_outside_pose_graph_opt() -> None:
    module = _load_module()

    contract = module.build_contract(ROOT)

    assert contract["schema"] == "lingtu.camera_lidar_calibration_migration_contract.v1"
    assert contract["ok"] is True
    assert contract["covered_by_pose_graph_opt"] is False
    assert "continuous-time ICP/GICP" in contract["reason_not_covered_by_pose_graph_opt"]
    assert contract["rust_kernel"]["crate"] == "lingtu_camera_lidar_optimizer"
    assert "CT-ICP fixed-correspondence point-to-plane residuals" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "CT-GICP fixed-correspondence covariance-weighted Mahalanobis residuals" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "C ABI for CT-ICP/CT-GICP two-pose linearization calls" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "Rust-owned nearest-neighbor correspondence search and max-distance rejection" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "C ABI for CT-GICP nearest-neighbor correspondence construction calls" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "CT-GICP two-pose LM optimizer with prior and between constraints" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "C ABI for CT-GICP two-pose optimization calls" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "Rust-owned dynamic CT-GICP correspondence rebuild and two-pose optimization loop" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "C ABI for dynamic CT-GICP scan optimization calls" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "pure C++ Rust ABI header declares optimizer and correspondence entry points without GTSAM" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert (
        "dynamic point cloud integrator delegates CT-GICP correspondence "
        "rebuild and optimization loop to one Rust C ABI call"
    ) in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "dynamic point cloud integrator no longer owns CT-GICP nearest-neighbor search in the Rust runtime path" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "dynamic point cloud integrator no longer owns the CT-GICP correspondence rebuild optimization loop" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "default calibration build is Rust-required and skips GTSAM discovery/linkage" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "dynamic point cloud integrator has no GTSAM runtime fallback source path" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "camera-LiDAR CMake has no GTSAM discovery/linkage path" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "Rust-required calibration CMake path skips GTSAM discovery/linkage" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert (
        "camera-LiDAR calibration Docker images use ROS base images, Rust/Cargo, "
        "and explicit Ceres/Iridescence builds without GTSAM"
    ) in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "legacy CT-ICP/CT-GICP GTSAM factor headers are removed from the repository" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "legacy Rust-to-GTSAM helper header is removed from the repository" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "legacy camera-LiDAR GTSAM test support source is removed from the repository" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "dynamic point cloud integrator header stores Eigen poses instead of exposing GTSAM Pose3" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "visual camera calibration Nelder-Mead perturbations use Sophus SE3 expmap without GTSAM" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "default camera-LiDAR shared library source list is GTSAM-free and excludes legacy CT factor headers" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "C++ Rust bridge guards ABI version and struct sizes before factor calls" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "pure C++ Rust ABI header is GTSAM-free; legacy GTSAM helpers are removed" in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert (
        "Windows-safe ctypes smoke gate loads the Rust camera-LiDAR optimizer "
        "library and exercises CT-GICP optimization"
    ) in (
        contract["rust_kernel"]["covered_capabilities"]
    )
    assert "full removal of legacy dynamic_point_cloud_integrator GTSAM graph/LM fallback source path" not in (
        contract["rust_kernel"]["not_covered_yet"]
    )
    assert "Rust-owned nearest-neighbor correspondence search and max-distance rejection" not in (
        contract["rust_kernel"]["not_covered_yet"]
    )
    assert "removal of legacy CT-ICP/CT-GICP GTSAM factor headers" not in (
        contract["rust_kernel"]["not_covered_yet"]
    )
    assert "replacement or removal of legacy GTSAM-based calibration Docker images" not in (
        contract["rust_kernel"]["not_covered_yet"]
    )
    assert contract["current_gtsam_surfaces"]["build_packaging"] == []
    assert contract["current_gtsam_surfaces"]["legacy_factor_headers"] == []
    assert "default legacy calibration build still keeps GTSAM fallback for existing tools" not in (
        contract["rust_kernel"]["not_covered_yet"]
    )
    assert "visual_camera_calibration removal of GTSAM Pose3::Expmap" not in (
        contract["rust_kernel"]["not_covered_yet"]
    )


def test_contract_checks_current_gtsam_and_rust_kernel_sources() -> None:
    module = _load_module()

    contract = module.build_contract(ROOT)
    checks = {item["id"]: item for item in contract["source_checks"]}

    for required_id in (
        "calibration_cmake_gtsam_free_define",
        "calibration_cmake_rust_optimizer_default_on",
        "dynamic_integrator_rust_optimizer_runtime",
        "dynamic_integrator_direct_dynamic_optimizer_abi",
        "dynamic_integrator_rust_primary_pose_update",
        "dynamic_integrator_rust_required_runtime_guard",
        "dynamic_integrator_rust_required_no_silent_fallback",
        "dynamic_integrator_header_uses_eigen_pose_storage",
        "dynamic_integrator_cpp_rt_interpolation",
        "calibration_cmake_rust_optimizer_option",
        "calibration_cmake_rust_optimizer_link",
        "calibration_cmake_rust_optimizer_cargo_build",
        "calibration_cmake_rust_optimizer_imported_target",
        "calibration_cmake_rust_optimizer_target_link",
        "calibration_cmake_rust_optimizer_required_option",
        "calibration_cmake_rust_optimizer_required_define",
        "calibration_humble_docker_ros_base",
        "calibration_humble_superglue_docker_ros_base",
        "calibration_jazzy_docker_ros_base",
        "calibration_jazzy_superglue_docker_ros_base",
        "calibration_noetic_docker_ros_base",
        "calibration_noetic_superglue_docker_ros_base",
        "calibration_humble_docker_rust_toolchain",
        "calibration_humble_superglue_docker_rust_toolchain",
        "calibration_jazzy_docker_rust_toolchain",
        "calibration_jazzy_superglue_docker_rust_toolchain",
        "calibration_noetic_docker_rust_toolchain",
        "calibration_noetic_superglue_docker_rust_toolchain",
        "calibration_humble_docker_iridescence_build",
        "calibration_humble_superglue_docker_iridescence_build",
        "calibration_jazzy_docker_iridescence_build",
        "calibration_jazzy_superglue_docker_iridescence_build",
        "calibration_noetic_docker_iridescence_build",
        "calibration_noetic_superglue_docker_iridescence_build",
        "visual_calibration_uses_sophus_expmap",
        "visual_calibration_preserves_rotation_first_order",
        "rust_camera_lidar_manifest",
        "rust_camera_lidar_pose3",
        "rust_camera_lidar_ct_icp_correspondence",
        "rust_camera_lidar_ct_icp_linearize",
        "rust_camera_lidar_ct_icp_hessian",
        "rust_camera_lidar_ct_icp_time_derivative_cache",
        "rust_camera_lidar_ct_icp_jacobian_test",
        "rust_camera_lidar_ct_gicp_correspondence",
        "rust_camera_lidar_ct_gicp_source_point",
        "rust_camera_lidar_ct_gicp_target_point",
        "rust_camera_lidar_ct_gicp_nearest_neighbor_builder",
        "rust_camera_lidar_ct_gicp_linearize",
        "rust_camera_lidar_ct_gicp_mahalanobis",
        "rust_camera_lidar_ct_gicp_jacobian_test",
        "rust_camera_lidar_abi_version",
        "rust_camera_lidar_abi_ct_icp",
        "rust_camera_lidar_abi_ct_gicp",
        "rust_camera_lidar_abi_ct_gicp_correspondence_builder",
        "rust_camera_lidar_ct_gicp_optimizer",
        "rust_camera_lidar_ct_gicp_dynamic_optimizer",
        "rust_camera_lidar_abi_ct_gicp_optimizer",
        "rust_camera_lidar_abi_ct_gicp_dynamic_optimizer",
        "rust_camera_lidar_abi_test",
        "rust_camera_lidar_abi_optimizer_test",
        "rust_camera_lidar_dynamic_optimizer_test",
        "rust_camera_lidar_abi_dynamic_optimizer_test",
        "rust_camera_lidar_abi_smoke_gate",
        "rust_camera_lidar_abi_smoke_checks_abi_version",
        "rust_camera_lidar_abi_smoke_optimizes_ct_gicp",
        "rust_camera_lidar_abi_smoke_optimizes_dynamic_ct_gicp",
        "rust_camera_lidar_ct_gicp_nearest_neighbor_test",
        "rust_camera_lidar_abi_correspondence_builder_test",
        "cpp_rust_camera_lidar_bridge_header",
        "cpp_rust_camera_lidar_bridge_correspondence_builder",
        "cpp_rust_camera_lidar_bridge_dynamic_optimizer",
        "cpp_rust_camera_lidar_bridge_optimizer_structs",
        "cpp_rust_camera_lidar_bridge_abi_guard",
    ):
        assert checks[required_id]["present"] is True
        assert checks[required_id]["line"] is not None

    forbidden = {item["id"]: item for item in contract["forbidden_source_checks"]}
    for forbidden_id in (
        "calibration_cmake_no_gtsam_find_package",
        "calibration_cmake_no_gtsam_include_dirs",
        "calibration_cmake_no_gtsam_libraries",
        "dynamic_integrator_no_gtsam_include",
        "dynamic_integrator_no_gtsam_namespace",
        "dynamic_integrator_no_ct_gicp_factor_fallback",
        "dynamic_integrator_no_lm_fallback_guard",
        "dynamic_integrator_no_factor_graph",
        "dynamic_integrator_no_legacy_lm_optimizer",
        "dynamic_integrator_no_cpp_ct_gicp_knn_search",
        "dynamic_integrator_no_cpp_dynamic_ct_gicp_outer_loop",
        "dynamic_integrator_no_cpp_rust_correspondence_builder_helper",
        "cpp_rust_camera_lidar_bridge_no_gtsam_include",
        "cpp_rust_camera_lidar_bridge_no_gtsam_namespace",
        "calibration_humble_docker_no_gtsam_base",
        "calibration_humble_docker_no_legacy_gtsam_arg",
        "calibration_humble_docker_no_apt_fast",
        "calibration_humble_superglue_docker_no_gtsam_base",
        "calibration_humble_superglue_docker_no_legacy_gtsam_arg",
        "calibration_humble_superglue_docker_no_apt_fast",
        "calibration_jazzy_docker_no_gtsam_base",
        "calibration_jazzy_docker_no_legacy_gtsam_arg",
        "calibration_jazzy_docker_no_apt_fast",
        "calibration_jazzy_superglue_docker_no_gtsam_base",
        "calibration_jazzy_superglue_docker_no_legacy_gtsam_arg",
        "calibration_jazzy_superglue_docker_no_apt_fast",
        "calibration_noetic_docker_no_gtsam_base",
        "calibration_noetic_docker_no_legacy_gtsam_arg",
        "calibration_noetic_docker_no_apt_fast",
        "calibration_noetic_superglue_docker_no_gtsam_base",
        "calibration_noetic_superglue_docker_no_legacy_gtsam_arg",
        "calibration_noetic_superglue_docker_no_apt_fast",
    ):
        assert forbidden[forbidden_id]["present"] is False

    removed = {item["path"]: item for item in contract["removed_gtsam_build_files"]}
    assert removed[
        "tools/calibration/camera_lidar/direct_visual_lidar_calibration/cmake/FindGTSAM.cmake"
    ]["removed"] is True
    for path in (
        "tools/calibration/camera_lidar/direct_visual_lidar_calibration/include/vlcal/common/"
        "integrated_ct_icp_factor.hpp",
        "tools/calibration/camera_lidar/direct_visual_lidar_calibration/include/vlcal/common/"
        "integrated_ct_icp_factor_impl.hpp",
        "tools/calibration/camera_lidar/direct_visual_lidar_calibration/include/vlcal/common/"
        "integrated_ct_gicp_factor.hpp",
        "tools/calibration/camera_lidar/direct_visual_lidar_calibration/include/vlcal/common/"
        "integrated_ct_gicp_factor_impl.hpp",
        "tools/calibration/camera_lidar/direct_visual_lidar_calibration/include/vlcal/common/"
        "rust_camera_lidar_optimizer_gtsam.hpp",
        "tools/calibration/camera_lidar/direct_visual_lidar_calibration/src/test/outliers.cpp",
    ):
        assert removed[path]["removed"] is True


def test_pure_camera_lidar_rust_abi_header_stays_gtsam_free() -> None:
    header = (
        ROOT
        / "tools"
        / "calibration"
        / "camera_lidar"
        / "direct_visual_lidar_calibration"
        / "include"
        / "vlcal"
        / "common"
        / "rust_camera_lidar_optimizer.hpp"
    ).read_text(encoding="utf-8")

    assert "gtsam" not in header.lower()
    assert "LingtuCameraLidarTwoPoseOptimizationResult" in header
    assert "LingtuCameraLidarCtGicpSourcePoint" in header
    assert "LingtuCameraLidarCtGicpTargetPoint" in header
    assert "lingtu_camera_lidar_optimizer_build_ct_gicp_correspondences" in header
    assert "lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose" in header


def test_legacy_camera_lidar_gtsam_headers_and_support_are_removed() -> None:
    legacy_paths = [
        ROOT
        / "tools"
        / "calibration"
        / "camera_lidar"
        / "direct_visual_lidar_calibration"
        / "include"
        / "vlcal"
        / "common"
        / name
        for name in (
            "integrated_ct_icp_factor.hpp",
            "integrated_ct_icp_factor_impl.hpp",
            "integrated_ct_gicp_factor.hpp",
            "integrated_ct_gicp_factor_impl.hpp",
            "rust_camera_lidar_optimizer_gtsam.hpp",
        )
    ]
    legacy_paths.append(
        ROOT
        / "tools"
        / "calibration"
        / "camera_lidar"
        / "direct_visual_lidar_calibration"
        / "src"
        / "test"
        / "outliers.cpp"
    )

    for path in legacy_paths:
        assert not path.exists(), path


def test_calibration_cmake_can_skip_gtsam_in_rust_required_mode() -> None:
    cmake = (
        ROOT / "tools" / "calibration" / "camera_lidar" / "direct_visual_lidar_calibration" / "CMakeLists.txt"
    ).read_text(encoding="utf-8")

    assert "set(LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER_DEFAULT ON)" in cmake
    assert "add_custom_target(camera_lidar_optimizer_rust" in cmake
    assert "add_library(lingtu_camera_lidar_optimizer STATIC IMPORTED)" in cmake
    assert "target_link_libraries(direct_visual_lidar_calibration lingtu_camera_lidar_optimizer" in cmake
    assert "find_package(GTSAM" not in cmake
    assert "GTSAM_INCLUDE_DIRS" not in cmake
    assert "GTSAM_LIBRARIES" not in cmake

    unconditional_includes = cmake[
        cmake.index("target_include_directories(direct_visual_lidar_calibration PUBLIC") : cmake.index(
            "target_link_libraries(direct_visual_lidar_calibration"
        )
    ]
    assert "${GTSAM_INCLUDE_DIRS}" not in unconditional_includes
    assert (
        "target_compile_definitions(direct_visual_lidar_calibration PRIVATE "
        "LINGTU_CAMERA_LIDAR_GTSAM_FREE_BUILD)"
    ) in cmake


def test_default_camera_lidar_shared_library_sources_are_gtsam_free() -> None:
    module = _load_module()

    contract = module.build_contract(ROOT)
    audit = contract["default_library_source_audit"]

    assert audit["ok"] is True
    assert audit["source_count"] > 0
    assert audit["violations"] == []
    assert (
        "tools/calibration/camera_lidar/direct_visual_lidar_calibration/src/vlcal/preprocess/"
        "dynamic_point_cloud_integrator.cpp"
    ) in audit["sources"]
    assert not any("integrated_ct_" in source for source in audit["sources"])


def test_default_camera_lidar_shared_library_source_audit_rejects_gtsam(tmp_path: Path) -> None:
    module = _load_module()
    root = tmp_path / "tools/calibration/camera_lidar/direct_visual_lidar_calibration"
    root.mkdir(parents=True)
    (root / "CMakeLists.txt").write_text(
        "add_library(direct_visual_lidar_calibration SHARED\n"
        "  src/bad.cpp\n"
        ")\n"
        "set(LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER_DEFAULT ON)\n"
        "LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER\n"
        "LINGTU_CAMERA_LIDAR_OPTIMIZER_LIBRARY\n"
        "add_custom_target(camera_lidar_optimizer_rust)\n"
        "add_library(lingtu_camera_lidar_optimizer STATIC IMPORTED)\n"
        "target_link_libraries(direct_visual_lidar_calibration lingtu_camera_lidar_optimizer)\n"
        "LINGTU_CAMERA_LIDAR_REQUIRE_RUST_OPTIMIZER\n"
        "target_compile_definitions(direct_visual_lidar_calibration PRIVATE "
        "LINGTU_CAMERA_LIDAR_REQUIRE_RUST_OPTIMIZER)\n"
        "target_compile_definitions(direct_visual_lidar_calibration PRIVATE "
        "LINGTU_CAMERA_LIDAR_GTSAM_FREE_BUILD)\n",
        encoding="utf-8",
    )
    bad_source = root / "src/bad.cpp"
    bad_source.parent.mkdir()
    bad_source.write_text(
        "#include <vlcal/common/integrated_ct_gicp_factor.hpp>\n"
        "#include <gtsam/geometry/Pose3.h>\n"
        "void bad() { (void)sizeof(gtsam::Pose3); }\n",
        encoding="utf-8",
    )

    contract = module.build_contract(tmp_path)
    audit = contract["default_library_source_audit"]

    assert contract["ok"] is False
    assert audit["ok"] is False
    assert {
        item["pattern"] for item in contract["default_library_source_violations"]
    } >= {"#include <gtsam/", "gtsam::", "#include <vlcal/common/integrated_ct_"}


def test_contract_reports_missing_required_source_check(tmp_path: Path) -> None:
    module = _load_module()
    cmake = tmp_path / "tools/calibration/camera_lidar/direct_visual_lidar_calibration/CMakeLists.txt"
    cmake.parent.mkdir(parents=True)
    cmake.write_text("# no gtsam here\n", encoding="utf-8")

    contract = module.build_contract(tmp_path)

    assert contract["ok"] is False
    missing = {item["id"] for item in contract["missing_required_checks"]}
    forbidden = {item["id"] for item in contract["forbidden_present_checks"]}
    assert "calibration_cmake_no_gtsam_find_package" not in forbidden
    missing = {item["id"] for item in contract["missing_required_checks"]}
    assert "calibration_cmake_gtsam_free_define" in missing
