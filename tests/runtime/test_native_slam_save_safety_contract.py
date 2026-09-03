from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FASTLIO_SOURCE = ROOT / "src" / "localization" / "slam" / "cpp" / "fastlio.cpp"
DRIVER_UNIT = ROOT / "scripts" / "deploy" / "thunder" / "lt-driver.service"


def test_fastlio_save_has_no_optimization_facade() -> None:
    source = FASTLIO_SOURCE.read_text(encoding="utf-8")

    assert "builder_->saveMap(pcd.string())" in source
    assert "writeTrajectory(pcd.parent_path(), pose_history_)" in source
    assert "writePatchBundle(pcd.parent_path(), patches, patch_history_dropped_count_)" in source
    assert "map.raw.pcd" not in source
    assert "map_optimization" not in source
    assert "loop_closure" not in source
    assert "hba_refine" not in source


def test_native_driver_consumes_backend_configuration_from_product_session() -> None:
    unit = DRIVER_UNIT.read_text(encoding="utf-8")

    assert "brainstem.service" not in unit
    assert "robot-brainstem.service" not in unit
    assert "EnvironmentFile=/run/lingtu/session.env" in unit
    assert "brainstem.env" not in unit
    assert "run_status_file_watchdog.sh" in unit
    assert "WatchdogSec=" in unit
