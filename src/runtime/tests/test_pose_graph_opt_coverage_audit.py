from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
AUDIT_SCRIPT = ROOT / "tools" / "validate" / "validate_pose_graph_opt_coverage.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "validate_pose_graph_opt_coverage",
        AUDIT_SCRIPT,
    )
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_classifier_rejects_gtsam_on_migrated_pgo_hba_surface() -> None:
    module = _load_module()

    assert (
        module.classify_path("src/localization/pgo/src/pgos/simple_pgo.cpp")
        == "covered_surface_forbidden"
    )
    assert (
        module.classify_path("src/localization/hba/src/hba/hba.cpp")
        == "covered_surface_forbidden"
    )
    assert (
        module.classify_path(
            "calibration/camera_lidar/direct_visual_lidar_calibration/src/vlcal/"
            "preprocess/dynamic_point_cloud_integrator.cpp"
        )
        == "covered_surface_forbidden"
    )


def test_classifier_keeps_optional_baseline_out_of_runtime_dependency_bucket() -> None:
    module = _load_module()

    assert (
        module.classify_path("tools/bench/pose_graph_opt_gtsam_baseline.py")
        == "optional_baseline_tool"
    )
    assert module.classify_path("tools/bench/pose_graph_opt_compare.py") == (
        "optional_baseline_tool"
    )
    assert module.classify_path("src/runtime/tests/test_pose_graph_opt_gtsam_baseline.py") == (
        "pose_graph_opt_tests"
    )


def test_classifier_keeps_migration_contract_tools_in_audit_bucket() -> None:
    module = _load_module()

    assert module.classify_path(
        "tools/validate/validate_pct_gpmp_migration_contract.py"
    ) == "coverage_audit"
    assert module.classify_path(
        "src/runtime/tests/test_pct_gpmp_migration_contract.py"
    ) == "coverage_audit"


def test_classifier_tracks_known_remaining_gtsam_surfaces() -> None:
    module = _load_module()

    assert module.classify_path(
        "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/CMakeLists.txt"
    ) == "remaining_dependency"
    assert module.classify_path(
        "src/nav/services/plan/global_planner/algorithm/pct/runtime/runtime.py"
    ) == "remaining_dependency"
    assert module.classify_path("scripts/build/build_ros_workspace.sh") == (
        "remaining_dependency_support"
    )
    assert module.classify_path("scripts/deploy/setup_server_ros_pct.sh") == (
        "remaining_dependency_support"
    )


def test_classifier_marks_vendored_tree_separately_from_active_dependencies() -> None:
    module = _load_module()

    assert module.classify_path(
        "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/gtsam/"
        "geometry/Pose3.h"
    ) == "vendored_third_party"


def test_scan_detects_unknown_and_migrated_surface_violations(tmp_path: Path) -> None:
    module = _load_module()
    (tmp_path / "src/localization/pgo").mkdir(parents=True)
    (tmp_path / "src/localization/pgo/bad.cpp").write_text(
        "#include <gtsam/geometry/Pose3.h>\n",
        encoding="utf-8",
    )
    (tmp_path / "src/other").mkdir(parents=True)
    (tmp_path / "src/other/new_backend.cpp").write_text(
        "find_package(GTSAM REQUIRED)\n",
        encoding="utf-8",
    )
    camera_lidar_source = (
        tmp_path
        / "calibration/camera_lidar/direct_visual_lidar_calibration/src/bad.cpp"
    )
    camera_lidar_source.parent.mkdir(parents=True)
    camera_lidar_source.write_text(
        "#include <gtsam/geometry/Pose3.h>\n",
        encoding="utf-8",
    )

    result = module.scan_repository(tmp_path)

    categories = {hit.path: hit.category for hit in result.hits}
    assert categories["src/localization/pgo/bad.cpp"] == "covered_surface_forbidden"
    assert categories["src/other/new_backend.cpp"] == "unknown"
    assert (
        categories["calibration/camera_lidar/direct_visual_lidar_calibration/src/bad.cpp"]
        == "covered_surface_forbidden"
    )
    assert {hit.category for hit in result.violations} == {
        "covered_surface_forbidden",
        "unknown",
    }


def test_scan_ignores_local_gtsam_ext_namespace(tmp_path: Path) -> None:
    module = _load_module()
    source = (
        tmp_path
        / "calibration/camera_lidar/direct_visual_lidar_calibration/include/vlcal/"
        "common/frame.hpp"
    )
    source.parent.mkdir(parents=True)
    source.write_text(
        "namespace gtsam_ext { struct Frame {}; }\n"
        "gtsam_ext::Frame* local_frame = nullptr;\n",
        encoding="utf-8",
    )

    result = module.scan_repository(tmp_path)

    assert result.hits == []
    assert result.violations == []


def test_scan_reports_vendored_tree_without_scanning_its_source(tmp_path: Path) -> None:
    module = _load_module()
    vendored_file = (
        tmp_path
        / "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/gtsam/"
        "geometry/Pose3.h"
    )
    vendored_file.parent.mkdir(parents=True)
    vendored_file.write_text("#include <gtsam/base/Matrix.h>\n", encoding="utf-8")

    result = module.scan_repository(tmp_path)

    assert result.vendored_tree_present is True
    assert result.category_counts == {"vendored_third_party": 1}
    assert result.violations == []


def test_scan_rejects_removed_pct_gtsam_runtime_artifacts(tmp_path: Path) -> None:
    module = _load_module()
    artifact = (
        tmp_path
        / "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/x86_64/libgtsam.so.4.1.1"
    )
    artifact.parent.mkdir(parents=True)
    artifact.write_bytes(b"\x00\x01not text")

    result = module.scan_repository(tmp_path)

    assert len(result.hits) == 1
    assert result.hits[0].path.endswith("libgtsam.so.4.1.1")
    assert result.hits[0].line == 0
    assert result.hits[0].category == "removed_legacy_runtime_artifact"
    assert result.violations == result.hits


def test_scan_rejects_removed_pct_native_runtime_artifacts(tmp_path: Path) -> None:
    module = _load_module()
    artifact = (
        tmp_path
        / "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/x86_64/"
        "a_star.cpython-310-x86_64-linux-gnu.so"
    )
    artifact.parent.mkdir(parents=True)
    artifact.write_bytes(b"\x00\x01not text")

    result = module.scan_repository(tmp_path)

    assert len(result.hits) == 1
    assert result.hits[0].path.endswith("a_star.cpython-310-x86_64-linux-gnu.so")
    assert result.hits[0].line == 0
    assert result.hits[0].category == "removed_legacy_runtime_artifact"
    assert result.violations == result.hits


def test_audit_json_exposes_capability_surface_matrix(tmp_path: Path) -> None:
    module = _load_module()

    result = module.scan_repository(tmp_path)
    payload = result.to_jsonable()
    surfaces = {surface["surface"]: surface for surface in payload["capability_surfaces"]}
    subsurfaces = {
        subsurface["subsurface"]: subsurface
        for subsurface in payload["dependency_subsurfaces"]
    }

    assert surfaces["slam_pgo"]["coverage"] == "covered_by_pose_graph_opt"
    assert surfaces["slam_hba"]["coverage"] == "covered_by_pose_graph_opt"
    assert "SE3 Pose3" in surfaces["slam_pgo"]["capabilities"]
    assert surfaces["pct_gpmp_global_planning"]["coverage"] == "remaining_dependency"
    assert "vector-state nonlinear factor graphs" in (
        surfaces["pct_gpmp_global_planning"]["capabilities"]
    )
    assert surfaces["camera_lidar_calibration"]["coverage"] == (
        "covered_by_camera_lidar_optimizer"
    )
    assert surfaces["camera_lidar_calibration"]["kernel"] == (
        "src/kernels/calibration/camera_lidar_optimizer"
    )
    assert "Rust-owned dynamic CT-GICP correspondence rebuild and optimization loop" in (
        surfaces["camera_lidar_calibration"]["capabilities"]
    )
    assert subsurfaces["pct_runtime_packaging"]["priority"] == "P0"
    assert subsurfaces["pct_gpmp_optimizer_core"]["priority"] == "P1"


def test_remaining_dependency_hits_must_map_to_capability_surface() -> None:
    module = _load_module()

    assert module.remaining_dependency_surface_for(
        "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/CMakeLists.txt"
    ) == "pct_gpmp_global_planning"
    assert module.remaining_dependency_surface_for(
        "src/nav/services/plan/global_planner/algorithm/pct/runtime/runtime.py"
    ) == "pct_gpmp_global_planning"
    assert (
        module.remaining_dependency_surface_for(
            "calibration/camera_lidar/direct_visual_lidar_calibration/CMakeLists.txt"
        )
        is None
    )
    assert module.remaining_dependency_surface_for("src/other/gtsam_user.cpp") is None


def test_remaining_dependency_subsurface_mapping_tracks_next_migration_packages() -> None:
    module = _load_module()

    assert module.remaining_dependency_subsurface_for(
        "src/nav/services/plan/global_planner/algorithm/pct/runtime/runtime.py"
    ) == "pct_runtime_packaging"
    assert module.remaining_dependency_subsurface_for(
        "src/nav/services/plan/global_planner/algorithm/pct/runtime/native/x86_64/libgtsam.so.4.1.1"
    ) == "pct_runtime_packaging"
    assert (
        module.remaining_dependency_subsurface_for(
            "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/x86_64/libgtsam.so.4.1.1"
        )
        is None
    )
    assert module.remaining_dependency_subsurface_for(
        "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src/trajectory_optimization/"
        "gpmp_optimizer/factors/gp_prior_factor.h"
    ) == "pct_gpmp_optimizer_core"
    assert module.remaining_dependency_subsurface_for(
        "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src/trajectory_optimization/"
        "trajectory_optimization/python_interface.cc"
    ) == "pct_python_binding_control"
    assert module.remaining_dependency_subsurface_for(
        "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/CMakeLists.txt"
    ) == "pct_build_packaging"
    assert module.remaining_dependency_subsurface_for(
        "calibration/camera_lidar/direct_visual_lidar_calibration/include/vlcal/common/"
        "integrated_ct_icp_factor_impl.hpp"
    ) is None
    assert module.remaining_dependency_subsurface_for(
        "calibration/camera_lidar/direct_visual_lidar_calibration/src/vlcal/preprocess/"
        "dynamic_point_cloud_integrator.cpp"
    ) is None
    assert module.remaining_dependency_subsurface_for(
        "calibration/camera_lidar/direct_visual_lidar_calibration/src/calibrate.cpp"
    ) is None
    assert module.remaining_dependency_subsurface_for(
        "calibration/camera_lidar/direct_visual_lidar_calibration/include/vlcal/common/"
        "frame.hpp"
    ) is None
    assert module.remaining_dependency_subsurface_for(
        "calibration/camera_lidar/direct_visual_lidar_calibration/src/test/outliers.cpp"
    ) is None


def test_dependency_surface_for_hit_sorts_support_after_capability_surfaces() -> None:
    module = _load_module()

    surfaces = [
        "remaining_dependency_support",
        "unmapped_remaining_dependency",
        "pct_gpmp_global_planning",
    ]

    assert sorted(surfaces, key=module.surface_sort_key) == [
        "pct_gpmp_global_planning",
        "remaining_dependency_support",
        "unmapped_remaining_dependency",
    ]


def test_remaining_dependency_support_is_not_capability_surface_mismatch(tmp_path: Path) -> None:
    module = _load_module()
    workflow = tmp_path / ".github/workflows/slam-aarch64-build.yml"
    workflow.parent.mkdir(parents=True)
    workflow.write_text("libgtsam-dev\n", encoding="utf-8")

    result = module.scan_repository(tmp_path)

    assert result.hits[0].category == "remaining_dependency_support"
    assert result.capability_surface_violations == []
    assert result.violations == []


def test_remaining_dependency_surface_summary_groups_hits_by_surface(tmp_path: Path) -> None:
    module = _load_module()
    pct = tmp_path / "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/CMakeLists.txt"
    pct_runtime = tmp_path / "src/nav/services/plan/global_planner/algorithm/pct/runtime/runtime.py"
    support = tmp_path / "scripts/build/build_ros_workspace.sh"
    pct.parent.mkdir(parents=True)
    pct_runtime.parent.mkdir(parents=True)
    support.parent.mkdir(parents=True)
    pct.write_text(
        "find_package(GTSAM REQUIRED)\n# libgtsam runtime note\n",
        encoding="utf-8",
    )
    pct_runtime.write_text("libgtsam.so.4.1.1\n", encoding="utf-8")
    support.write_text("GTSAM_DIR=/tmp/gtsam\n", encoding="utf-8")

    result = module.scan_repository(tmp_path)
    summary = result.remaining_dependency_surface_summary
    json_summary = result.to_jsonable()["remaining_dependency_surface_summary"]

    assert summary["pct_gpmp_global_planning"]["hit_count"] == 3
    assert summary["pct_gpmp_global_planning"]["path_count"] == 2
    assert summary["pct_gpmp_global_planning"]["representative_paths"][0] == {
        "path": "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/CMakeLists.txt",
        "hit_count": 2,
        "first_line": 1,
        "sample_text": "find_package(GTSAM REQUIRED)",
    }
    assert summary["remaining_dependency_support"]["hit_count"] == 1
    assert summary["remaining_dependency_support"]["paths"] == [
        "scripts/build/build_ros_workspace.sh"
    ]
    assert json_summary["pct_gpmp_global_planning"]["path_count"] == 2
    subsurface_summary = result.remaining_dependency_subsurface_summary
    assert subsurface_summary["pct_build_packaging"]["hit_count"] == 2
    assert subsurface_summary["pct_runtime_packaging"]["hit_count"] == 1
    assert result.to_jsonable()["remaining_dependency_subsurface_summary"][
        "pct_runtime_packaging"
    ]["priority"] == "P0"


def test_text_report_prints_representative_paths_by_surface(tmp_path: Path, capsys) -> None:
    module = _load_module()
    pct = tmp_path / "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/CMakeLists.txt"
    support = tmp_path / "scripts/build/build_ros_workspace.sh"
    pct.parent.mkdir(parents=True)
    support.parent.mkdir(parents=True)
    pct.write_text("find_package(GTSAM REQUIRED)\n", encoding="utf-8")
    support.write_text("GTSAM_DIR=/tmp/gtsam\n", encoding="utf-8")
    result = module.scan_repository(tmp_path)

    module.print_text_report(result, fail_on_remaining=False)

    output = capsys.readouterr().out
    assert "Remaining dependency summary:" in output
    assert "pct_gpmp_global_planning: 1 hit(s), 1 path(s)" in output
    assert "pct_build_packaging [P2]: 1 hit(s), 1 path(s)" in output
    assert "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/CMakeLists.txt" in output
    assert "remaining_dependency_support: 1 hit(s), 1 path(s)" in output
    assert "remaining_dependency_support [P0]: 1 hit(s), 1 path(s)" in output
    assert "scripts/build/build_ros_workspace.sh" in output
