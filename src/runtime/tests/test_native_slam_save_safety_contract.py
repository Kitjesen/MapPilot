from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
FASTLIO_SOURCE = ROOT / "src" / "localization" / "slam" / "cpp" / "fastlio.cpp"
DRIVER_UNIT = ROOT / "scripts" / "deploy" / "thunder" / "lingtu-driver.service"
MAP_PIPELINE_HEADER = (
    ROOT / "src" / "maps" / "include" / "lingtu" / "maps" / "build" / "pipeline.hpp"
)


def test_unverified_fastlio_save_time_loop_closure_is_opt_in() -> None:
    source = FASTLIO_SOURCE.read_text(encoding="utf-8")

    assert "bool map_optimization_enabled = false;" in source


def test_native_driver_treats_brainstem_as_remote_endpoint() -> None:
    unit = DRIVER_UNIT.read_text(encoding="utf-8")

    assert "brainstem.service" not in unit
    assert "robot-brainstem.service" not in unit
    assert "EnvironmentFile=/opt/lingtu/config/brainstem.env" in unit
    assert "EnvironmentFile=-/opt/lingtu/config/brainstem.env" not in unit
    assert "run_status_file_watchdog.sh" in unit
    assert "WatchdogSec=" in unit


def test_native_map_pipeline_also_defaults_to_no_secondary_optimizer() -> None:
    header = MAP_PIPELINE_HEADER.read_text(encoding="utf-8")

    assert 'std::string optimizer_strategy{"off"};' in header
