from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

import hashlib
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
    assert args.duration == 12.0
    assert args.duration_clock == "sim"
    assert args.imu_acc_mode == "finite_difference"
    assert args.scan_time_profile == "map_metadata"
    assert args.live_drive_source == "fixed"
    assert args.fastlio_lidar_input == "timed_pointcloud2"
    assert args.fastlio_ieskf_max_iter == 10
    assert args.runtime_fault_confirm_samples == 6
    assert args.runtime_motion_fault_min_sim_m == 1.0
    assert args.localizer_config == ""
    assert args.localizer_rough_score_thresh is None
    assert args.localizer_refine_score_thresh is None
    assert args.preflight_only is False
    assert args.check_global_relocalize is False
    kidnapped = _build_parser().parse_args(["--check-global-relocalize"])
    assert kidnapped.check_global_relocalize is True
    assert kidnapped.kidnap_initial_x != 0.0
    assert kidnapped.min_global_map_odom_xy_m == 0.0
    assert kidnapped.bbs3d_num_threads >= 1
    assert kidnapped.bbs3d_timeout_ms >= 30000
    assert _live_feed_timeout_s(args) >= 120.0


def test_saved_map_relocalize_live_feed_uses_stable_fastlio_defaults(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    import sim.scripts.saved_map_relocalize_runtime_gate as gate

    class FakePopen:
        returncode = None

        def __init__(self, cmd, **kwargs):
            self.cmd = cmd
            stdout = kwargs.get("stdout")
            if stdout is not None:
                stdout.close()

    captured: dict[str, list[str]] = {}

    def fake_popen(cmd, **kwargs):
        captured["cmd"] = list(cmd)
        return FakePopen(cmd, **kwargs)

    args = gate._build_parser().parse_args(
        [
            "--run-dir",
            str(tmp_path / "run"),
            "--duration",
            "12",
        ]
    )
    monkeypatch.setattr(gate.subprocess, "Popen", fake_popen)

    proc = gate._start_live_feed(args, tmp_path / "run")

    assert isinstance(proc, FakePopen)
    command = " ".join(captured["cmd"])
    assert "--drive-source fixed" in command
    assert "--fastlio-lidar-input timed_pointcloud2" in command
    assert "--fastlio-ieskf-max-iter 10" in command
    assert "--runtime-fault-confirm-samples 6" in command
    assert "--runtime-motion-fault-min-sim-m 1.0" in command


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


def _saved_map_runtime_args(tmp_path: Path):
    from sim.scripts.saved_map_relocalize_runtime_gate import _build_parser

    map_pcd = tmp_path / "same_source_map/map.pcd"
    map_pcd.parent.mkdir(parents=True)
    map_text = "VERSION 0.7\nDATA ascii\n"
    map_pcd.write_text(map_text, encoding="ascii")
    (map_pcd.parent / "metadata.json").write_text(
        json.dumps(
            {
                "schema_version": "lingtu.same_source_map_artifacts.v1",
                "world": "/tmp/large_terrain_scene.xml",
                "scan_time_profile": "physical_rolling",
                "pcd": str(map_pcd),
                "pcd_sha256": hashlib.sha256(map_pcd.read_bytes()).hexdigest(),
                "point_count": 1,
            }
        ),
        encoding="utf-8",
    )
    localizer_config = tmp_path / "localizer.yaml"
    localizer_config.write_text(
        "rough_score_thresh: 0.2\nrefine_score_thresh: 0.1\n",
        encoding="utf-8",
    )
    return _build_parser().parse_args(
        [
            "--map-pcd",
            str(map_pcd),
            "--localizer-config",
            str(localizer_config),
            "--run-dir",
            str(tmp_path / "run"),
            "--json-out",
            str(tmp_path / "report.json"),
        ]
    )


def test_saved_map_relocalize_metadata_contract_accepts_saved_map_schema(
    tmp_path: Path,
):
    from sim.scripts.saved_map_relocalize_runtime_gate import _map_metadata_contract

    map_pcd = tmp_path / "same_source_map/map.pcd"
    map_pcd.parent.mkdir(parents=True)
    map_pcd.write_text("VERSION 0.7\nDATA ascii\n", encoding="ascii")
    (map_pcd.parent / "metadata.json").write_text(
        json.dumps(
            {
                "schema_version": "lingtu.saved_map_artifacts.v1",
                "world": "/tmp/large_terrain_scene.xml",
                "scan_time_profile": "physical_rolling",
                "pcd": str(map_pcd),
                "pcd_sha256": hashlib.sha256(map_pcd.read_bytes()).hexdigest(),
                "point_count": 1,
            }
        ),
        encoding="utf-8",
    )

    contract = _map_metadata_contract(map_pcd)

    assert contract["ok"] is True
    assert contract["checks"]["schema_version_known"] is True
    assert contract["schema_version"] == "lingtu.saved_map_artifacts.v1"


def test_saved_map_relocalize_runtime_gate_preflight_rejects_bad_metadata(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    import sim.scripts.saved_map_relocalize_runtime_gate as gate

    args = _saved_map_runtime_args(tmp_path)
    args.preflight_only = True
    metadata = Path(args.map_pcd).parent / "metadata.json"
    payload = json.loads(metadata.read_text(encoding="utf-8"))
    payload["pcd_sha256"] = "wrong-sha"
    metadata.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setattr(gate, "_load_ros_modules", lambda: tuple())

    report = gate.run_gate(args)

    assert report["ok"] is False
    assert report["execution_mode"] == "host_preflight_only"
    assert report["map_metadata_contract"]["ok"] is False
    assert report["map_metadata_contract"]["checks"][
        "map_pcd_sha256_matches_file"
    ] is False
    assert "map_metadata.map_pcd_sha256_matches_file is not true" in report[
        "blockers"
    ]


def test_saved_map_relocalize_runtime_gate_preflight_reports_ros_blocker(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    import sim.scripts.saved_map_relocalize_runtime_gate as gate

    args = _saved_map_runtime_args(tmp_path)
    args.preflight_only = True
    monkeypatch.setattr(
        gate,
        "_load_ros_modules",
        lambda: (_ for _ in ()).throw(RuntimeError("ROS unavailable")),
    )

    report = gate.run_gate(args)

    assert report["ok"] is False
    assert report["execution_mode"] == "host_preflight_only"
    assert report["map_metadata_contract"]["ok"] is True
    assert report["validation_only"] is True
    assert report["runtime_relocalization_executed"] is False
    assert report["runtime_relocalization_validated"] is False
    assert report["ros2_python"]["ok"] is False
    assert report["claim_boundary"] == "preflight_only_no_live_slam_or_relocalization"
    assert "ROS unavailable" in report["blockers"]


def test_saved_map_relocalize_runtime_gate_returns_host_guard_when_ros_missing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    import sim.scripts.saved_map_relocalize_runtime_gate as gate

    args = _saved_map_runtime_args(tmp_path)
    monkeypatch.setattr(
        gate,
        "_load_ros_modules",
        lambda: (_ for _ in ()).throw(RuntimeError("ROS unavailable")),
    )

    report = gate.run_gate(args)

    assert report["ok"] is False
    assert report["execution_mode"] == "host_guard"
    assert report["runtime_relocalization_executed"] is False
    assert report["runtime_relocalization_validated"] is False
    assert report["ros2_python"]["ok"] is False
    assert report["claim_boundary"] == "environment_blocked_no_runtime_relocalization"
    assert report["blockers"] == ["ROS unavailable"]


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


def test_saved_map_relocalize_latest_map_prefers_tomogram_backed_artifacts(
    tmp_path, monkeypatch
):
    import os

    import sim.scripts.saved_map_relocalize_runtime_gate as gate

    priority_without_tomogram = (
        tmp_path
        / "artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live_new/tare/same_source_map/map.pcd"
    )
    broader_with_tomogram = (
        tmp_path
        / "artifacts/server_sim_closure/moving_obstacle_sweep/children/fast-dense/run/same_source_map/map.pcd"
    )
    for path in (priority_without_tomogram, broader_with_tomogram):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("VERSION 0.7\nDATA ascii\n", encoding="ascii")
    (broader_with_tomogram.parent / "tomogram.pickle").write_bytes(b"tomogram")
    os.utime(priority_without_tomogram, (300.0, 300.0))
    os.utime(broader_with_tomogram, (200.0, 200.0))

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    assert gate._resolve_latest_map() == broader_with_tomogram


def test_saved_map_relocalize_latest_map_uses_newest_tomogram_backed_candidate(
    tmp_path, monkeypatch
):
    import os

    import sim.scripts.saved_map_relocalize_runtime_gate as gate

    priority_with_tomogram = (
        tmp_path
        / "artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live_new/tare/same_source_map/map.pcd"
    )
    broader_newer_with_tomogram = (
        tmp_path
        / "artifacts/server_sim_closure/moving_obstacle_sweep/children/fast-dense/run/same_source_map/map.pcd"
    )
    for path in (priority_with_tomogram, broader_newer_with_tomogram):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("VERSION 0.7\nDATA ascii\n", encoding="ascii")
        (path.parent / "tomogram.pickle").write_bytes(b"tomogram")
    os.utime(priority_with_tomogram, (200.0, 200.0))
    os.utime(broader_newer_with_tomogram, (300.0, 300.0))

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    assert gate._resolve_latest_map() == broader_newer_with_tomogram


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
    assert report["default_profiles"]["navigating"] == "native_dds"
    native = report["contracts"]["native_dds"]
    assert native["health_source"] == "slam_runtime"
    assert native["saved_map_relocalization_supported"] is True
    localizer = report["contracts"]["localizer"]
    assert localizer["health_source"] == "localizer_health_topic"
    assert localizer["map_save_source"] == "active_map"
    assert localizer["saved_map_relocalization_supported"] is True
    assert localizer["recovery_method"] == "relocalize_service"
    for backend in ("fastlio2", "super_lio", "super_lio_relocation"):
        assert report["contracts"][backend]["saved_map_relocalization_supported"] is False
    assert report["plans"]["session_navigating_native_dds"]["ensure"] == ("slam",)
    assert report["plans"]["switch_localizer"]["ensure"] == ("slam",)
    assert all(report["launch_services"].values())
    status = report["bridge_status"]["localizer"]
    assert status["backend"] == "localizer"
    assert status["localizer_health"] == "LOCKED"
    assert status["relocalization_state"] == "idle"
    assert status["saved_map_relocalization_supported"] is True
