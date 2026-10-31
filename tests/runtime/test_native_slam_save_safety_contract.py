from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LTLIO_SOURCE = ROOT / "src" / "localization" / "slam" / "cpp" / "ltlio.cpp"
DRIVER_UNIT = ROOT / "scripts" / "deploy" / "thunder" / "lt-driver.service"


def test_ltlio_save_preserves_correction_and_raw_patch_evidence() -> None:
    source = LTLIO_SOURCE.read_text(encoding="utf-8")

    assert "pcl::io::savePCDFileBinary(capture.pcd.string(), capture.cloud)" in source
    assert "writeTrajectory(capture.pcd.parent_path(), capture.trajectory)" in source
    assert "sample.odom_body = slamPose(opt::compose_pose(correction, graphPose(sample.odom_body)))" in source
    assert "patches[i].pose = slamPose(pose)" in source
    assert "opt::rotate_vector(pose, point.x, point.y, point.z)" in source
    admission = source.split("Status startSaveMapAsync(", 1)[1].split(
        "bool saveMapAsyncInFlight()", 1
    )[0]
    for reason in (
        "online_mapping_busy_retry_save",
        "online_mapping_incomplete_cannot_save_corrected_map",
        "online_mapping_patch_identity_mismatch",
    ):
        assert admission.index(reason) < admission.index("SaveCapture capture;")
    assert "capture.pcd.parent_path(), capture.patches, capture.dropped_patches," in source
    assert 'map_dir / "scan_origin.txt"' in source
    assert "map.raw.pcd" not in source
    assert '#include "map_optimization' not in source
    assert '#include "loop_closure' not in source
    assert "hba_refine" not in source


def test_save_waits_for_the_first_loop_too() -> None:
    source = LTLIO_SOURCE.read_text(encoding="utf-8")
    admission = source.split("Status startSaveMapAsync(", 1)[1]
    assert "if (mode_ == SlamMode::Mapping && online_mapping_.busy())" in admission
    assert admission.index("online_mapping_busy_retry_save") < admission.index(
        "const bool corrected_save"
    )
    synchronous = source.split("Status saveMap(", 1)[1].split(
        "Status startSaveMapAsync(", 1
    )[0]
    assert 'while (!started.ok && started.message == "online_mapping_busy_retry_save")' in synchronous


def test_native_driver_consumes_backend_configuration_from_product_session() -> None:
    unit = DRIVER_UNIT.read_text(encoding="utf-8")

    assert "brainstem.service" not in unit
    assert "robot-brainstem.service" not in unit
    assert "EnvironmentFile=/run/lingtu/session.env" in unit
    assert "brainstem.env" not in unit
    assert "run_status_file_watchdog.sh" in unit
    assert "WatchdogSec=" in unit
