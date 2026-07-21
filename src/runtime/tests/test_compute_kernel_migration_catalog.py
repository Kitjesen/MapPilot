from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

from kernels.catalog import KERNEL_TARGETS, first_wave_targets, target_by_key


ROOT = Path(__file__).resolve().parents[3]
COVERAGE_AUDIT = ROOT / "tools" / "validate" / "validate_pose_graph_opt_coverage.py"


def _load_coverage_audit():
    spec = importlib.util.spec_from_file_location(
        "validate_pose_graph_opt_coverage_for_catalog",
        COVERAGE_AUDIT,
    )
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_first_wave_targets_cover_deployment_blockers():
    first_wave = {target.key for target in first_wave_targets()}

    assert first_wave == {"path_safety", "gateway_pointcloud_codec"}


def test_every_kernel_has_stable_boundary_and_target_path():
    keys = set()
    for target in KERNEL_TARGETS:
        assert target.key not in keys
        keys.add(target.key)
        assert target.boundary in {"c_abi", "process_abi"}
        assert target.target_path.startswith("src/kernels/")
        assert (ROOT / target.target_path).exists()
        assert target.current_paths
        assert target.responsibility


def test_rust_candidates_are_portable_dependency_reduction_targets():
    rust_targets = {target.key for target in KERNEL_TARGETS if target.lane == "rust_candidate"}

    assert rust_targets == {
        "camera_lidar_calibration_optimizer",
        "gpmp_trajectory_optimizer",
        "path_safety",
        "pose_graph_optimizer",
    }


def test_pose_graph_optimizer_tracks_gtsam_replacement_surface():
    target = target_by_key("pose_graph_optimizer")

    assert target.boundary == "c_abi"
    assert target.lane == "rust_candidate"
    assert target.status == "contract_first"
    assert target.first_wave is False
    assert "src/localization/pgo/src/pgos/simple_pgo.cpp" in target.current_paths
    assert "src/localization/hba/src/hba/hba.cpp" in target.current_paths
    assert "PGO/HBA Pose3 prior/between" in target.notes
    assert "PCT/GPMP" in target.notes


def test_catalog_does_not_track_speculative_or_nav_kernel_owned_targets():
    for removed in {
        "local_planner",
        "terrain_analysis",
        "path_follower",
        "waypoint_tracker",
        "icp_localizer",
        "pct_adapter",
        "pct_global",
    }:
        with pytest.raises(KeyError):
            target_by_key(removed)



def test_remaining_gtsam_surfaces_have_explicit_migration_targets():
    gpmp = target_by_key("gpmp_trajectory_optimizer")
    calibration = target_by_key("camera_lidar_calibration_optimizer")

    assert gpmp.boundary == "process_abi"
    assert gpmp.lane == "rust_candidate"
    assert gpmp.status == "contract_first"
    assert "gpmp_optimizer" in gpmp.current_paths[0]
    assert "Not covered by pose_graph_opt" in gpmp.notes
    assert "WNOJ/WNOA process" in gpmp.notes
    assert "DenseElevationMap obstacle factors" in gpmp.notes
    assert calibration.boundary == "process_abi"
    assert calibration.lane == "rust_candidate"
    assert calibration.status == "contract_first"
    assert calibration.first_wave is False
    assert calibration.current_paths == (
        "calibration/camera_lidar/direct_visual_lidar_calibration",
    )
    assert "Offline calibration path" in calibration.notes
    assert "require Rust success" in calibration.notes
    assert "calls the Rust optimizer ABI directly" in calibration.notes
    assert "Visual calibration" in calibration.notes
    assert "Pose3::Expmap" in calibration.notes
    assert "dynamic integrator header" in calibration.notes
    assert "Eigen poses" in calibration.notes


def test_coverage_remaining_surfaces_are_backed_by_kernel_catalog_targets():
    audit = _load_coverage_audit()
    remaining_surfaces = {
        surface["surface"]
        for surface in audit.CAPABILITY_SURFACES
        if surface["coverage"] == "remaining_dependency"
    }
    catalog_paths = {
        path
        for target in KERNEL_TARGETS
        for path in target.current_paths
    }

    assert remaining_surfaces == {"pct_gpmp_global_planning"}
    covered_surfaces = {
        surface["surface"]: surface
        for surface in audit.CAPABILITY_SURFACES
        if str(surface["coverage"]).startswith("covered_by_")
    }
    assert covered_surfaces["camera_lidar_calibration"]["coverage"] == (
        "covered_by_camera_lidar_optimizer"
    )
    assert covered_surfaces["camera_lidar_calibration"]["kernel"] == (
        "src/kernels/calibration/camera_lidar_optimizer"
    )
    assert "src/nav/services/plan/global_planner/algorithm/pct/runtime" in catalog_paths
    assert (
        "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src/trajectory_optimization/gpmp_optimizer"
        in catalog_paths
    )
    assert "calibration/camera_lidar/direct_visual_lidar_calibration" in catalog_paths
