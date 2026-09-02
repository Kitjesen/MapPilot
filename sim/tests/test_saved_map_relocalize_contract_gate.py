from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

pytestmark = [pytest.mark.sim]


def _saved_map_runtime_args(tmp_path: Path):
    from sim.scripts.mujoco.saved_map_relocalization import _build_parser

    map_pcd = tmp_path / "same_source_map/map.pcd"
    map_pcd.parent.mkdir(parents=True)
    map_pcd.write_text("VERSION 0.7\nDATA ascii\n", encoding="ascii")
    (map_pcd.parent / "metadata.json").write_text(
        json.dumps(
            {
                "schema_version": "lingtu.same_source_map_artifacts.v1",
                "world": "/tmp/large_terrain_scene.xml",
                "scan_time_profile": "physical_rolling",
                "pcd": str(map_pcd),
                "point_count": 1,
            }
        ),
        encoding="utf-8",
    )

    native_dir = tmp_path / "native"
    native_dir.mkdir()
    paths = {
        "slamd": native_dir / "slamd",
        "slamctl": native_dir / "slamctl",
        "config": native_dir / "fastlio2.yaml",
        "bridge": native_dir / "native_dds_sensors.py",
        "publisher": native_dir / "lingtu_mujoco_sensor_publisher",
    }
    for path in paths.values():
        path.write_text("placeholder\n", encoding="utf-8")

    return _build_parser().parse_args(
        [
            "--map-pcd", str(map_pcd),
            "--run-dir", str(tmp_path / "run"),
            "--json-out", str(tmp_path / "report.json"),
            "--slam-runtime-bin", str(paths["slamd"]),
            "--slam-control-bin", str(paths["slamctl"]),
            "--slam-config", str(paths["config"]),
            "--sensor-bridge", str(paths["bridge"]),
            "--publisher-bin", str(paths["publisher"]),
            "--domain-id", "231",
        ]
    )


class _FakePopen:
    returncode = None

    def __init__(self, cmd, **kwargs):
        self.cmd = list(cmd)
        stdout = kwargs.get("stdout")
        if stdout is not None:
            stdout.close()


def test_saved_map_relocalize_runtime_gate_defaults_to_native_runtime(
    monkeypatch: pytest.MonkeyPatch,
):
    from sim.scripts.mujoco.saved_map_relocalization import (
        _build_parser,
        _sensor_feed_timeout_s,
    )

    monkeypatch.delenv("LINGTU_DDS_DOMAIN_ID", raising=False)
    args = _build_parser().parse_args([])

    assert args.world == "map_metadata"
    assert args.duration == 12.0
    assert args.duration_clock == "sim"
    assert args.imu_acc_mode == "sensor"
    assert args.scan_time_profile == "map_metadata"
    assert args.slam_runtime_bin == ""
    assert args.slam_control_bin == ""
    assert args.domain_id == 231
    assert (args.initial_x, args.initial_y, args.initial_z, args.initial_yaw) == (
        0.0, 0.0, 0.0, 0.0
    )
    assert args.preflight_only is False
    assert args.check_global_relocalize is False
    assert (args.kidnap_start_x, args.kidnap_start_y, args.kidnap_start_z) == (
        3.0, 2.0, 0.0
    )
    assert _sensor_feed_timeout_s(args) >= 120.0
    assert args.min_global_map_odom_xy_m == 1.0


def test_saved_map_relocalize_runtime_gate_uses_environment_dds_domain(
    monkeypatch: pytest.MonkeyPatch,
):
    from sim.scripts.mujoco.saved_map_relocalization import _build_parser

    monkeypatch.setenv("LINGTU_DDS_DOMAIN_ID", "77")

    assert _build_parser().parse_args([]).domain_id == 77


@pytest.mark.parametrize("domain_id", [0, 232])
def test_saved_map_relocalize_preflight_rejects_out_of_range_dds_domain(
    tmp_path: Path,
    domain_id: int,
):
    import sim.scripts.mujoco.saved_map_relocalization as gate

    args = _saved_map_runtime_args(tmp_path)
    args.preflight_only = True
    args.domain_id = domain_id

    report = gate.run_gate(args)

    assert report["ok"] is False
    assert f"DDS domain id must be in [1, 231]: {domain_id}" in report["blockers"]


def test_saved_map_relocalize_starts_native_dds_sensor_feed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    import sim.scripts.mujoco.saved_map_relocalization as gate

    args = _saved_map_runtime_args(tmp_path)
    status_path = tmp_path / "run/slamd.status.json"
    captured: dict[str, list[str]] = {}

    def fake_popen(cmd, **kwargs):
        captured["cmd"] = list(cmd)
        return _FakePopen(cmd, **kwargs)

    monkeypatch.setattr(gate.subprocess, "Popen", fake_popen)
    proc = gate._start_sensor_feed(args, tmp_path / "run", status_path)

    assert isinstance(proc, _FakePopen)
    command = captured["cmd"]
    assert str(args.sensor_bridge) in command
    assert command[command.index("--drive-mode") + 1] == "kinematic"
    assert "--allow-kinematic-fastlio-acceptance" in command
    assert command[command.index("--domain-id") + 1] == "231"
    assert command[command.index("--slam-status-json") + 1] == str(status_path)
    assert "--require-slam-output" in command
    assert command[command.index("--publisher-bin") + 1] == str(args.publisher_bin)
    assert "--start" not in command
    assert "--start-anchor" not in command


def test_global_relocalize_starts_sensor_feed_from_kidnapped_position(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    import sim.scripts.mujoco.saved_map_relocalization as gate

    args = _saved_map_runtime_args(tmp_path)
    args.check_global_relocalize = True
    args.kidnap_start_x = 3.5
    args.kidnap_start_y = -2.25
    args.kidnap_start_z = 0.4
    status_path = tmp_path / "run/slamd.status.json"
    captured: dict[str, list[str]] = {}

    def fake_popen(cmd, **kwargs):
        captured["cmd"] = list(cmd)
        return _FakePopen(cmd, **kwargs)

    monkeypatch.setattr(gate.subprocess, "Popen", fake_popen)
    gate._start_sensor_feed(args, tmp_path / "run", status_path)

    command = captured["cmd"]
    assert command[command.index("--start") + 1] == "3.5,-2.25,0.4"
    assert command[command.index("--start-anchor") + 1] == "warmup"


def test_saved_map_relocalize_starts_slamd_in_localization_mode(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    import sim.scripts.mujoco.saved_map_relocalization as gate

    args = _saved_map_runtime_args(tmp_path)
    map_pcd = Path(args.map_pcd)
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    status_path = run_dir / "slamd.status.json"
    captured: dict[str, list[str]] = {}

    def fake_popen(cmd, **kwargs):
        captured["cmd"] = list(cmd)
        return _FakePopen(cmd, **kwargs)

    monkeypatch.setattr(gate.subprocess, "Popen", fake_popen)
    proc = gate._start_slamd(args, map_pcd, run_dir, status_path)

    assert isinstance(proc, _FakePopen)
    command = captured["cmd"]
    assert command[0] == str(args.slam_runtime_bin)
    assert command[command.index("--backend") + 1] == "fastlio2"
    assert command[command.index("--mode") + 1] == "localization"
    assert command[command.index("--map") + 1] == str(map_pcd)
    assert command[command.index("--config") + 1] == str(args.slam_config)
    assert command[command.index("--domain-id") + 1] == "231"
    assert command[command.index("--status-json") + 1] == str(status_path)


@pytest.mark.parametrize("global_relocalize", [False, True])
def test_saved_map_relocalize_calls_native_slamctl(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    global_relocalize: bool,
):
    import sim.scripts.mujoco.saved_map_relocalization as gate

    args = _saved_map_runtime_args(tmp_path)
    args.check_global_relocalize = global_relocalize
    captured: dict[str, list[str]] = {}

    class Completed:
        returncode = 0
        stdout = '{"success": true, "message": "accepted"}\n'
        stderr = ""

    def fake_run(command, **kwargs):
        captured["command"] = list(command)
        return Completed()

    monkeypatch.setattr(gate.subprocess, "run", fake_run)
    ok, response = gate._call_slamctl(args)

    assert ok is True
    assert response["success"] is True
    command = captured["command"]
    expected = "global-relocalize" if global_relocalize else "relocalize"
    assert command[:2] == [str(args.slam_control_bin), expected]
    assert command[command.index("--domain-id") + 1] == "231"
    if global_relocalize:
        assert "--x" not in command
        assert "--yaw" not in command
    else:
        assert command[command.index("--x") + 1] == "0.0"
        assert command[command.index("--y") + 1] == "0.0"
        assert command[command.index("--z") + 1] == "0.0"
        assert command[command.index("--yaw") + 1] == "0.0"


def test_saved_map_relocalize_runtime_gate_is_free_of_old_ros_localizer_surface():
    source = Path("sim/scripts/mujoco/saved_map_relocalization.py").read_text(
        encoding="utf-8"
    )

    assert "rclpy" not in source
    assert "localizer_node" not in source
    assert "ros2 run " + "localizer" not in source
    assert "_start_live_feed" not in source
    assert "localizer.yaml" not in source
    assert '"live_feed"' not in source
    assert "service-timeout" not in source


def test_saved_map_relocalize_runtime_gate_uses_same_source_metadata(tmp_path: Path):
    from sim.scripts.mujoco.saved_map_relocalization import (
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
        json.dumps(metadata), encoding="utf-8"
    )
    loaded = _load_map_metadata(map_pcd)

    assert loaded == metadata
    assert _resolve_live_world_arg("map_metadata", loaded) == metadata["world"]
    assert _resolve_live_world_arg("industrial_park", loaded) == "industrial_park"
    assert _resolve_scan_time_profile_arg("map_metadata", loaded) == "instantaneous"
    assert _resolve_scan_time_profile_arg("synthetic_rolling", loaded) == "synthetic_rolling"


def test_saved_map_relocalize_metadata_contract_accepts_saved_map_schema(
    tmp_path: Path,
):
    from sim.scripts.mujoco.saved_map_relocalization import _map_metadata_contract

    args = _saved_map_runtime_args(tmp_path)
    map_pcd = Path(args.map_pcd)
    metadata_path = map_pcd.parent / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["schema_version"] = "lingtu.saved_map_artifacts.v1"
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")

    contract = _map_metadata_contract(map_pcd)

    assert contract["ok"] is True
    assert contract["checks"]["schema_version_known"] is True
    assert contract["schema_version"] == "lingtu.saved_map_artifacts.v1"


def test_saved_map_relocalize_runtime_gate_preflight_accepts_native_artifacts(
    tmp_path: Path,
):
    import sim.scripts.mujoco.saved_map_relocalization as gate

    args = _saved_map_runtime_args(tmp_path)
    args.preflight_only = True
    report = gate.run_gate(args)

    assert report["ok"] is True
    assert report["schema_version"] == "lingtu.saved_map_relocalize_runtime.v2"
    assert report["execution_mode"] == "host_preflight_only"
    assert report["validation_only"] is True
    assert report["native_preflight"]["ok"] is True
    assert report["runtime_relocalization_executed"] is False
    assert report["runtime_relocalization_validated"] is False
    assert report["native_runtime"]["domain_id"] == 231
    assert [item["id"] for item in report["runtime_dataflow"]] == [
        "saved_map_artifact",
        "native_runtime_inputs",
    ]
    assert report["runtime_dataflow"][0]["ok"] is True
    assert report["runtime_dataflow"][1]["ok"] is True
    assert report["claim_boundary"] == "preflight_only_no_live_slam_or_relocalization"


def test_saved_map_relocalize_runtime_dataflow_is_readable_by_diagnostics(
    tmp_path: Path,
):
    import sim.scripts.mujoco.saved_map_relocalization as gate
    from sim.diagnostics.dataflow_report import runtime_dataflow_for_gate

    args = _saved_map_runtime_args(tmp_path)
    args.preflight_only = True
    report_path = tmp_path / "report.json"
    report_path.write_text(json.dumps(gate.run_gate(args)), encoding="utf-8")

    dataflow = runtime_dataflow_for_gate(
        "saved_map_relocalize",
        {"path": str(report_path)},
        {},
        root=tmp_path,
    )

    assert dataflow["checked"] is True, dataflow
    assert dataflow["ok"] is True
    assert dataflow["flow"]
    assert dataflow["failed_edges"] == []


@pytest.mark.parametrize(
    ("response", "expected"),
    [
        ({"success": True, "engine": "bbs3d_gicp"}, True),
        ({"success": False, "engine": "bbs3d_gicp"}, False),
        ({"success": True, "engine": "other", "message": "global success"}, False),
        ({"success": True, "message": "global relocalization accepted"}, False),
    ],
)
def test_bbs3d_success_requires_explicit_success_and_engine(
    response: dict[str, object], expected: bool
):
    from sim.scripts.mujoco.saved_map_relocalization import _bbs3d_succeeded

    assert _bbs3d_succeeded(response) is expected


def test_saved_map_relocalize_runtime_gate_preflight_rejects_empty_map(
    tmp_path: Path,
):
    import sim.scripts.mujoco.saved_map_relocalization as gate

    args = _saved_map_runtime_args(tmp_path)
    args.preflight_only = True
    Path(args.map_pcd).write_bytes(b"")
    report = gate.run_gate(args)

    assert report["ok"] is False
    assert report["execution_mode"] == "host_preflight_only"
    assert report["map_metadata_contract"]["checks"]["map_pcd_format_ok"] is False
    assert "map_metadata.map_pcd_format_ok is not true" in report["blockers"]


@pytest.mark.parametrize(
    ("missing_attr", "blocker_fragment"),
    [
        ("slam_runtime_bin", "slamd not found"),
        ("slam_control_bin", "slamctl not found"),
        ("slam_config", "FastLIO native config not found"),
        ("sensor_bridge", "native DDS sensor bridge not found"),
        ("publisher_bin", "native DDS publisher not found"),
    ],
)
def test_saved_map_relocalize_runtime_gate_preflight_reports_missing_native_artifact(
    tmp_path: Path,
    missing_attr: str,
    blocker_fragment: str,
):
    import sim.scripts.mujoco.saved_map_relocalization as gate

    args = _saved_map_runtime_args(tmp_path)
    args.preflight_only = True
    setattr(args, missing_attr, str(tmp_path / "missing" / missing_attr))
    report = gate.run_gate(args)

    assert report["ok"] is False
    assert report["native_preflight"]["ok"] is False
    assert any(blocker_fragment in blocker for blocker in report["blockers"])


def test_saved_map_relocalize_latest_map_uses_newest_generic_native_artifact(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    import sim.scripts.mujoco.saved_map_relocalization as gate

    old_map = tmp_path / "artifacts/sim_diagnostics/native_slam_capture/old/same_source_map/map.pcd"
    new_map = tmp_path / "artifacts/sim_diagnostics/native_slam_capture/new/same_source_map/map.pcd"
    for path in (old_map, new_map):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("VERSION 0.7\nDATA ascii\n", encoding="ascii")
    os.utime(old_map, (100.0, 100.0))
    os.utime(new_map, (200.0, 200.0))
    monkeypatch.setattr(gate, "ROOT", tmp_path)

    assert gate._resolve_latest_map() == new_map
