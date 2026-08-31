from __future__ import annotations

from pathlib import Path

from kernels.catalog import KERNEL_TARGETS, first_wave_targets, target_by_key

ROOT = Path(__file__).resolve().parents[3]


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
        "path_safety",
    }


def test_camera_lidar_migration_surface_has_a_kernel_target():
    calibration = target_by_key("camera_lidar_calibration_optimizer")

    assert calibration.boundary == "process_abi"
    assert calibration.lane == "rust_candidate"
    assert calibration.status == "contract_first"
    assert calibration.first_wave is False
    assert calibration.current_paths == (
        "tools/calibration/camera_lidar/direct_visual_lidar_calibration",
    )
    assert "Offline calibration path" in calibration.notes
    assert "require Rust success" in calibration.notes
    assert "calls the Rust optimizer ABI directly" in calibration.notes
    assert "Visual calibration" in calibration.notes
    assert "Pose3::Expmap" in calibration.notes
    assert "dynamic integrator header" in calibration.notes
    assert "Eigen poses" in calibration.notes


def test_camera_lidar_source_is_backed_by_kernel_catalog_target():
    catalog_paths = {
        path
        for target in KERNEL_TARGETS
        for path in target.current_paths
    }

    assert "tools/calibration/camera_lidar/direct_visual_lidar_calibration" in catalog_paths
