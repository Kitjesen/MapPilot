from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

import json
from pathlib import Path

from sim.scripts.saved_map_relocalize_contract_gate import run_gate


def test_saved_map_relocalize_runtime_gate_defaults_to_product_mujoco_live():
    from sim.scripts.saved_map_relocalize_runtime_gate import (
        _build_parser,
        _live_feed_timeout_s,
    )

    args = _build_parser().parse_args([])

    assert args.world == "map_metadata"
    assert args.duration_clock == "sim"
    assert args.imu_acc_mode == "finite_difference"
    assert args.scan_time_profile == "map_metadata"
    assert args.live_drive_source == "frontier"
    assert args.localizer_config == ""
    assert args.localizer_rough_score_thresh is None
    assert args.localizer_refine_score_thresh is None
    assert args.check_global_relocalize is False
    kidnapped = _build_parser().parse_args(["--check-global-relocalize"])
    assert kidnapped.check_global_relocalize is True
    assert kidnapped.kidnap_initial_x != 0.0
    assert kidnapped.min_global_map_odom_xy_m == 0.0
    assert kidnapped.bbs3d_num_threads >= 1
    assert kidnapped.bbs3d_timeout_ms >= 30000
    assert _live_feed_timeout_s(args) >= 120.0


def test_saved_map_relocalize_runtime_gate_uses_runtime_contract_boundaries():
    source = Path("sim/scripts/saved_map_relocalize_runtime_gate.py").read_text(
        encoding="utf-8",
    )

    assert "adapter_source_for_target" in source
    assert "TOPICS.saved_map_cloud" in source
    assert "TOPICS.relocalize_service" in source
    assert "TOPICS.global_relocalize_service" in source
    assert "MAP_TO_ODOM_LINK.parent" in source
    assert '"/nav/' not in source
    assert "'/nav/" not in source
    assert 'frame_id != "map"' not in source
    assert 'child_frame_id != "odom"' not in source


def test_saved_map_relocalize_runtime_gate_uses_same_source_metadata(tmp_path):
    from sim.scripts.saved_map_relocalize_runtime_gate import (
        _load_map_metadata,
        _resolve_live_world_arg,
        _resolve_scan_time_profile_arg,
    )

    map_pcd = tmp_path / "same_source_map/map.pcd"
    map_pcd.parent.mkdir()
    map_pcd.write_text("VERSION 0.7\nDATA ascii\n", encoding="ascii")
    metadata = {
        "world": "/tmp/large_terrain_scene.xml",
        "scan_time_profile": "instantaneous",
    }
    (map_pcd.parent / "metadata.json").write_text(
        json.dumps(metadata),
        encoding="utf-8",
    )

    loaded = _load_map_metadata(map_pcd)

    assert loaded == metadata
    assert _resolve_live_world_arg("map_metadata", loaded) == metadata["world"]
    assert _resolve_live_world_arg("industrial_park", loaded) == "industrial_park"
    assert _resolve_scan_time_profile_arg("map_metadata", loaded) == "instantaneous"
    assert _resolve_scan_time_profile_arg("synthetic_rolling", loaded) == "synthetic_rolling"


def test_saved_map_relocalize_runtime_gate_writes_localizer_threshold_config(tmp_path):
    from sim.scripts.saved_map_relocalize_runtime_gate import (
        _parse_localizer_thresholds,
        _write_localizer_runtime_config,
    )

    base = tmp_path / "localizer.yaml"
    base.write_text(
        "\n".join(
            [
                "rough_score_thresh: 0.2",
                "refine_score_thresh: 0.1",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    runtime = _write_localizer_runtime_config(
        base,
        tmp_path,
        rough_score_thresh=0.35,
        refine_score_thresh=0.35,
    )

    assert runtime == tmp_path / "localizer_runtime.yaml"
    assert _parse_localizer_thresholds(runtime) == {
        "rough_score_thresh": 0.35,
        "refine_score_thresh": 0.35,
    }


def test_saved_map_relocalize_latest_map_prefers_relocalization_sources(tmp_path, monkeypatch):
    import os

    import sim.scripts.saved_map_relocalize_runtime_gate as gate

    tare_old = (
        tmp_path
        / "artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live_old/tare/same_source_map/map.pcd"
    )
    tare_new = (
        tmp_path
        / "artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live_new/tare/same_source_map/map.pcd"
    )
    fastlio_newer = (
        tmp_path
        / "artifacts/server_sim_closure/mujoco_fastlio2_live/inspection/same_source_map/map.pcd"
    )
    for path in (tare_old, tare_new, fastlio_newer):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("VERSION 0.7\nDATA ascii\n", encoding="ascii")
    os.utime(tare_old, (100.0, 100.0))
    os.utime(tare_new, (200.0, 200.0))
    os.utime(fastlio_newer, (300.0, 300.0))

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    assert gate._resolve_latest_map() == tare_new


def test_saved_map_relocalize_gate_locks_navigation_contract():
    report = run_gate()

    assert report["ok"] is True
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["validation_level"] == "contract_only_no_runtime_relocalization"
    assert report["runtime_stage"] == "saved_map_relocalization"
    assert report["map_dependency"] == "saved_map_required"
    assert report["requires_saved_map"] is True
    assert report["runtime_relocalization_executed"] is False
    assert report["runtime_relocalization_validated"] is False
    assert "runtime_relocalization_validated" in report["forbidden_claims"]
    assert report["default_profiles"]["navigating"] == "localizer"
    localizer = report["contracts"]["localizer"]
    assert localizer["health_source"] == "localizer_health_topic"
    assert localizer["map_save_source"] == "active_map"
    assert localizer["saved_map_relocalization_supported"] is True
    assert localizer["recovery_method"] == "relocalize_service"
    for backend in ("fastlio2", "super_lio", "super_lio_relocation"):
        assert report["contracts"][backend]["saved_map_relocalization_supported"] is False
    assert report["plans"]["session_navigating_fastlio2"]["ensure"] == ("slam", "localizer")
    assert report["plans"]["switch_localizer"]["ensure"] == ("slam", "localizer")
    assert all(report["launch_services"].values())
    status = report["bridge_status"]["localizer"]
    assert status["backend"] == "localizer"
    assert status["localizer_health"] == "LOCKED"
    assert status["relocalization_state"] == "idle"
    assert status["saved_map_relocalization_supported"] is True
