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
    native = Path("scripts/diagnostics/native/dds_probe.cpp").read_text(encoding="utf-8")
    build = Path("scripts/build/build_dds_probe.sh").read_text(encoding="utf-8")

    assert "runtime.adapters.dds.reader" not in source
    assert "DDSReader" not in source
    assert "cyclonedds-python" in source
    assert "dds_create_reader" in native
    assert "lingtu_dds_FinalVelocityCommand_desc" in native
    assert "lingtu_slam.idl" in build


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
                    "frame_id": "livox_frame",
                    "points": 20064,
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


def test_dds_probe_rejects_legacy_dog_odometry_topic(monkeypatch, tmp_path) -> None:
    fake_bin = tmp_path / "lingtu_dds_probe"
    fake_bin.write_text("", encoding="utf-8")
    monkeypatch.setattr(dds_probe, "_ensure_native_probe", lambda: fake_bin)

    with pytest.raises(ValueError, match="rt/driver/odometry"):
        dds_probe.probe(("rt/nav/dog_odometry",), seconds=1.0, domain_id=0)
