from __future__ import annotations

import json

import pytest

import diagnostics.field.dds_readiness as readiness


class _Stat:
    def __init__(
        self,
        *,
        samples: int = 0,
        rate_hz: float = 0.0,
        first_ts: float = 0.0,
        last_ts: float = 0.0,
        frame_id: str = "",
        points: int | None = None,
    ) -> None:
        self.samples = samples
        self._rate_hz = rate_hz
        self.first_ts = first_ts
        self.last_ts = last_ts
        self.frame_id = frame_id
        self.points = points

    def hz(self) -> float:
        return self._rate_hz


def test_collect_readiness_requires_product_topics() -> None:
    with pytest.raises(ValueError, match="topics must be provided"):
        readiness.collect_readiness()


def test_cli_requires_topics() -> None:
    with pytest.raises(SystemExit) as error:
        readiness.main(["--seconds", "1"])

    assert error.value.code == 2


def test_collect_readiness_keeps_real_probe_observations(monkeypatch) -> None:
    topic = "rt/lidar/raw_frame"
    stat = _Stat(
        samples=50,
        rate_hz=10.0,
        first_ts=100.0,
        last_ts=105.0,
        frame_id="lidar_link",
        points=24000,
    )
    monkeypatch.setattr(readiness, "_load_probe", lambda: lambda *_args, **_kwargs: {topic: stat})

    report = readiness.collect_readiness((topic,), seconds=5.0, domain_id=7)

    assert report == {
        "schema_version": readiness.SCHEMA_VERSION,
        "collected_at": report["collected_at"],
        "domain_id": 7,
        "duration_s": 5.0,
        "ok": True,
        "missing": [],
        "topics": {
            topic: {
                "samples": 50,
                "rate_hz": 10.0,
                "first_ts": 100.0,
                "last_ts": 105.0,
                "frame_id": "lidar_link",
                "points": 24000,
            }
        },
    }


def test_collect_readiness_reports_missing_and_unknown_topics(monkeypatch) -> None:
    live = "rt/imu/raw"
    unknown = "rt/not/a/topic"
    monkeypatch.setattr(readiness, "_load_probe", lambda: lambda *_args, **_kwargs: {live: _Stat()})

    report = readiness.collect_readiness((live, unknown))

    assert report["ok"] is False
    assert report["missing"] == [live, unknown]
    assert report["topics"][unknown]["error"] == "no typed DDS contract"


def test_collect_readiness_reports_probe_failure(monkeypatch) -> None:
    def fail_probe(*_args, **_kwargs):
        raise RuntimeError("native probe unavailable")

    monkeypatch.setattr(readiness, "_load_probe", lambda: fail_probe)

    report = readiness.collect_readiness(("rt/imu/raw",))

    assert report["ok"] is False
    assert report["topics"]["rt/imu/raw"]["error"] == "DDS probe failed: native probe unavailable"


def test_cli_writes_the_same_report(monkeypatch, tmp_path) -> None:
    report = {
        "schema_version": readiness.SCHEMA_VERSION,
        "collected_at": 1.0,
        "domain_id": 9,
        "duration_s": 3.5,
        "ok": True,
        "missing": [],
        "topics": {},
    }
    captured = {}

    def collect(topics, *, seconds, domain_id):
        captured.update(topics=topics, seconds=seconds, domain_id=domain_id)
        return report

    monkeypatch.setattr(readiness, "collect_readiness", collect)
    output = tmp_path / "readiness.json"

    exit_code = readiness.main(["--seconds", "3.5", "--domain", "9", "--topics", "rt/imu/raw", "--json", str(output)])

    assert exit_code == 0
    assert captured == {"topics": ["rt/imu/raw"], "seconds": 3.5, "domain_id": 9}
    assert json.loads(output.read_text(encoding="utf-8")) == report
