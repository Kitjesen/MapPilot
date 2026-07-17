"""Tests for the field readiness collector (no real DDS required)."""

from __future__ import annotations

import json
from types import SimpleNamespace

import pytest

import diagnostics.field.field_readiness_collector as collector_mod
from diagnostics.field.field_readiness_collector import (
    DEFAULT_TOPICS,
    FIELD_READINESS_SCHEMA_VERSION,
    FieldReadinessCollector,
    collect_field_readiness,
    main,
)


class _FakeStat:
    """Minimal stand-in for dds_probe.TopicStats used by mocked probe()."""

    def __init__(
        self,
        samples: int = 0,
        rate: float = 0.0,
        frame_id: str = "",
        last_ts: float = 0.0,
        points: int | None = None,
    ) -> None:
        self.samples = samples
        self._rate = rate
        self.frame_id = frame_id
        self.last_ts = last_ts
        self.points = points

    def hz(self) -> float:
        return self._rate


def _fake_probe_factory(sample_map: dict[str, _FakeStat]):
    """Return a probe() replacement keyed by the names collect() passes in."""

    def _fake_probe(topics, *, seconds, domain_id):
        return {name: sample_map.get(name, _FakeStat()) for name in topics}

    return _fake_probe


# ── schema / structure ───────────────────────────────────────────────────


def test_report_structure_and_schema_version() -> None:
    collector = FieldReadinessCollector(topics=("rt/camera/color",))
    results = {"rt/camera/color": {"samples": 10, "rate_hz": 5.0, "ok": True}}
    report = collector.build_report(results, duration_s=5.0)

    assert report["schema_version"] == FIELD_READINESS_SCHEMA_VERSION
    assert report["schema"] == FIELD_READINESS_SCHEMA_VERSION
    for key in ("ts", "domain_id", "duration_s", "ok", "topics", "missing", "summary"):
        assert key in report
    assert report["duration_s"] == 5.0
    assert report["topics"] == results


def test_default_topics_are_native_wire_names() -> None:
    assert DEFAULT_TOPICS
    assert all(name.startswith("rt/") for name in DEFAULT_TOPICS)


# ── header validation ─────────────────────────────────────────────────────


def test_validate_headers_alive_stat() -> None:
    collector = FieldReadinessCollector()
    stat = _FakeStat(samples=3, frame_id="camera", last_ts=123.0)
    headers = collector._validate_headers(stat)

    assert headers["frame_id"] == "camera"
    assert headers["has_frame_id"] is True
    assert headers["has_timestamp"] is True


def test_validate_headers_dead_stat() -> None:
    collector = FieldReadinessCollector()
    stat = _FakeStat(samples=0, frame_id="", last_ts=0.0)
    headers = collector._validate_headers(stat)

    assert headers["frame_id"] == ""
    assert headers["has_frame_id"] is False
    assert headers["has_timestamp"] is False


# ── build_report computation ───────────────────────────────────────────────


def test_build_report_ok_missing_summary() -> None:
    collector = FieldReadinessCollector(topics=("a", "b", "c"))
    results = {
        "a": {"samples": 100, "ok": True},
        "b": {"samples": 0, "ok": False},
        "c": {"samples": 5, "ok": True},
    }
    report = collector.build_report(results, duration_s=3.0)

    assert report["summary"] == {"total": 3, "alive": 2, "dead": 1}
    assert report["missing"] == ["b"]
    assert report["ok"] is False


def test_build_report_all_alive_is_ok() -> None:
    collector = FieldReadinessCollector(topics=("a", "b"))
    results = {
        "a": {"samples": 10, "ok": True},
        "b": {"samples": 20, "ok": True},
    }
    report = collector.build_report(results, duration_s=1.0)

    assert report["summary"] == {"total": 2, "alive": 2, "dead": 0}
    assert report["missing"] == []
    assert report["ok"] is True


# ── collect() with mocked probe ────────────────────────────────────────────


def test_collect_reuses_probe_and_maps_results(monkeypatch) -> None:
    topics = ("rt/camera/color", "rt/slam/odometry")
    collector = FieldReadinessCollector(topics=topics)

    # Resolve topics deterministically so the fake probe key matches.
    monkeypatch.setattr(collector_mod, "_probe_topic_name", lambda topic: topic)
    sample_map = {
        "rt/camera/color": _FakeStat(samples=132, rate=26.4, frame_id="camera", last_ts=9.0),
        "rt/slam/odometry": _FakeStat(samples=0),
    }
    monkeypatch.setattr(collector_mod, "_load_probe", lambda: _fake_probe_factory(sample_map))

    results = collector.collect(seconds=1.0)

    assert results["rt/camera/color"]["samples"] == 132
    assert results["rt/camera/color"]["rate_hz"] == 26.4
    assert results["rt/camera/color"]["frame_id"] == "camera"
    assert results["rt/camera/color"]["ok"] is True
    assert results["rt/slam/odometry"]["ok"] is False


def test_collect_marks_unresolvable_topics_dead(monkeypatch) -> None:
    collector = FieldReadinessCollector(topics=("rt/bogus/topic",))
    monkeypatch.setattr(collector_mod, "_probe_topic_name", lambda topic: None)
    monkeypatch.setattr(collector_mod, "_load_probe", lambda: _fake_probe_factory({}))

    results = collector.collect(seconds=1.0)

    assert results["rt/bogus/topic"]["ok"] is False
    assert results["rt/bogus/topic"]["error"] == "no typed DDS contract"


def test_collect_degrades_when_dds_unavailable(monkeypatch) -> None:
    collector = FieldReadinessCollector(topics=("rt/camera/color",))

    def _boom():
        raise RuntimeError("native probe missing")

    monkeypatch.setattr(collector_mod, "_load_probe", _boom)

    results = collector.collect(seconds=1.0)
    report = collector.build_report(results, duration_s=1.0)

    assert results["rt/camera/color"]["ok"] is False
    assert "error" in report
    assert "Native DDS probe unavailable" in report["error"]
    assert report["ok"] is False


def test_collect_field_readiness_end_to_end(monkeypatch) -> None:
    monkeypatch.setattr(collector_mod, "_probe_topic_name", lambda topic: topic)
    sample_map = {"rt/camera/color": _FakeStat(samples=50, rate=25.0, frame_id="camera", last_ts=1.0)}
    monkeypatch.setattr(collector_mod, "_load_probe", lambda: _fake_probe_factory(sample_map))

    report = collect_field_readiness(topics=("rt/camera/color",), seconds=2.0, domain_id=7)

    assert report["schema_version"] == FIELD_READINESS_SCHEMA_VERSION
    assert report["domain_id"] == 7
    assert report["ok"] is True


# ── CLI argument parsing ────────────────────────────────────────────────────


def test_cli_parses_seconds_domain_topics(monkeypatch, tmp_path) -> None:
    captured = {}

    def _fake_collect(topics=None, *, seconds, domain_id):
        captured["topics"] = topics
        captured["seconds"] = seconds
        captured["domain_id"] = domain_id
        return {
            "schema_version": FIELD_READINESS_SCHEMA_VERSION,
            "ok": True,
            "topics": {},
            "missing": [],
            "summary": {"total": 0, "alive": 0, "dead": 0},
            "domain_id": domain_id,
            "duration_s": seconds,
        }

    monkeypatch.setattr(collector_mod, "collect_field_readiness", _fake_collect)

    out_file = tmp_path / "readiness.json"
    rc = main(
        [
            "--seconds",
            "3.5",
            "--domain",
            "9",
            "--topics",
            "rt/camera/color",
            "rt/imu/raw",
            "--json",
            str(out_file),
        ]
    )

    assert rc == 0
    assert captured["seconds"] == 3.5
    assert captured["domain_id"] == 9
    assert captured["topics"] == ["rt/camera/color", "rt/imu/raw"]
    assert out_file.exists()
    written = json.loads(out_file.read_text(encoding="utf-8"))
    assert written["schema_version"] == FIELD_READINESS_SCHEMA_VERSION


def test_cli_defaults(monkeypatch) -> None:
    captured = {}

    def _fake_collect(topics=None, *, seconds, domain_id):
        captured["topics"] = topics
        captured["seconds"] = seconds
        captured["domain_id"] = domain_id
        return {"ok": False, "summary": {"total": 1, "alive": 0, "dead": 1}, "topics": {}, "missing": []}

    monkeypatch.setattr(collector_mod, "collect_field_readiness", _fake_collect)

    rc = main([])

    assert rc == 1
    assert captured["seconds"] == 5.0
    assert captured["domain_id"] == 0
    assert captured["topics"] is None


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
