from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from scripts.diagnostics import dds_probe
from scripts.diagnostics.dds_probe import TopicStats


def test_dds_probe_stats_extracts_frame_and_point_count() -> None:
    stats = TopicStats()
    msg = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        width=3,
        height=2,
    )

    stats.observe(msg)
    stats.observe(msg)

    assert stats.samples == 2
    assert stats.frame_id == "map"
    assert stats.points == 6
    assert stats.hz() >= 0.0


def test_dds_probe_uses_native_helper_not_cyclonedds_python() -> None:
    source = Path("scripts/diagnostics/dds_probe.py").read_text(encoding="utf-8")
    native = Path("tools/diagnostics/dds_probe.cpp").read_text(encoding="utf-8")
    build = Path("scripts/build/build_dds_probe.sh").read_text(encoding="utf-8")

    assert "runtime.adapters.dds.reader" not in source
    assert "DDSReader" not in source
    assert "cyclonedds-python" in source
    assert "dds_create_reader" in native
    assert "lingtu_dds_FinalVelocityCommand_desc" in native
    assert "lingtu_dds_MapObservation_desc" in native
    assert "lingtu_dds_MapCollisionLayer_desc" in native
    assert "lingtu::message::kMapsLocalCollision.topic" in native
    assert "lingtu_dds_ExplorationExecutionGrid_desc" in native
    assert "lingtu::message::kNavExplorationExecutionSnapshot.topic" in native
    assert "lingtu_dds_SlamMapSnapshotRequest_desc" in native
    assert "lingtu::message::kSlamMapSnapshotRequest.topic" in native
    assert "lingtu_dds_SlamMapSnapshotAck_desc" in native
    assert "lingtu::message::kSlamMapSnapshotAck.topic" in native
    assert "kSlamMapCommand" not in native
    assert "kSlamMapEvent" not in native
    assert r'\"reset_epoch\"' in native
    assert r'\"max_gap_s\"' in native
    assert "messages.idl" in build
    assert '${LINGTU_CYCLONEDDS_PREFIX}/bin/idlc' in build
    assert 'env LD_LIBRARY_PATH="${IDLC_LIBRARY_PATH}"' in build
    assert '-L"${LINGTU_CYCLONEDDS_PREFIX}/lib"' in build
    assert '-Wl,-rpath,"${LINGTU_CYCLONEDDS_PREFIX}/lib"' in build


def test_dds_probe_parses_native_json(monkeypatch, tmp_path) -> None:
    fake_bin = tmp_path / "lingtu_dds_probe"
    fake_bin.write_text("", encoding="utf-8")
    monkeypatch.setattr(dds_probe, "_ensure_native_probe", lambda: fake_bin)

    class Result:
        returncode = 0
        stderr = ""
        stdout = json.dumps(
            [
                {
                    "topic": "/lidar/raw_frame",
                    "samples": 10,
                    "first_ts": 100.0,
                    "last_ts": 101.0,
                    "hz": 9.5,
                    "max_gap_s": 0.12,
                    "frame_id": "livox_frame",
                    "points": 20064,
                    "reset_epoch": 3,
                    "observation_sequence": 42,
                    "generation": 9,
                    "live": True,
                }
            ]
        )

    calls = []

    def fake_run(command, **_kwargs):
        calls.append(command)
        return Result()

    monkeypatch.setattr(dds_probe.subprocess, "run", fake_run)

    stats = dds_probe.probe(("/lidar/raw_frame",), seconds=1.0, domain_id=7)

    assert calls[0][0] == str(fake_bin)
    assert "--json" in calls[0]
    assert "--domain" in calls[0]
    assert "7" in calls[0]
    assert stats["/lidar/raw_frame"].samples == 10
    assert stats["/lidar/raw_frame"].frame_id == "livox_frame"
    assert stats["/lidar/raw_frame"].points == 20064
    assert stats["/lidar/raw_frame"].hz() == 9.5
    assert stats["/lidar/raw_frame"].max_gap_s == 0.12
    assert stats["/lidar/raw_frame"].reset_epoch == 3
    assert stats["/lidar/raw_frame"].observation_sequence == 42
    assert stats["/lidar/raw_frame"].generation == 9
    assert stats["/lidar/raw_frame"].live is True


def test_dds_probe_rejects_legacy_dog_odometry_topic(monkeypatch, tmp_path) -> None:
    fake_bin = tmp_path / "lingtu_dds_probe"
    fake_bin.write_text("", encoding="utf-8")
    monkeypatch.setattr(dds_probe, "_ensure_native_probe", lambda: fake_bin)

    with pytest.raises(ValueError, match="rt/driver/odometry"):
        dds_probe.probe(("rt/nav/dog_odometry",), seconds=1.0, domain_id=0)


def test_managed_release_probe_never_builds_during_readiness(monkeypatch, tmp_path) -> None:
    missing = tmp_path / "release" / "lingtu_dds_probe"
    monkeypatch.setenv("LINGTU_DDS_PROBE_BIN", str(missing))
    monkeypatch.setenv("LINGTU_DDS_PROBE_ALLOW_BUILD", "0")

    with pytest.raises(FileNotFoundError, match="forbid readiness-time compilation"):
        dds_probe._ensure_native_probe()
