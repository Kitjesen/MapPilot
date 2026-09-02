# ruff: noqa: D101, D102, D103, S603, S607
"""Sample native DDS topics without cyclonedds-python and report activity."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from message.topics import TOPIC_SPECS, dds_topic_name

SCHEMA_VERSION = "lingtu.field_readiness_report.v1"
_WIRE_TOPICS = frozenset(dds_topic_name(topic) for topic in TOPIC_SPECS)
ROOT = Path(__file__).resolve().parents[3]
LEGACY_TOPIC_REPLACEMENTS = {
    "/nav/dog_odometry": "rt/driver/odometry",
    "rt/nav/dog_odometry": "rt/driver/odometry",
}


@dataclass
class TopicStats:
    samples: int = 0
    first_ts: float = 0.0
    last_ts: float = 0.0
    frame_id: str = ""
    points: int | None = None
    measured_hz: float | None = None
    max_gap_s: float = 0.0
    reset_epoch: int | None = None
    observation_sequence: int | None = None
    generation: int | None = None
    live: bool | None = None

    def observe(self, msg: Any) -> None:
        now = time.time()
        if not self.samples:
            self.first_ts = now
        self.samples += 1
        self.last_ts = now
        header = getattr(msg, "header", None)
        self.frame_id = str(getattr(header, "frame_id", self.frame_id) or self.frame_id)
        self.points = _point_count(msg, self.points)

    def hz(self) -> float:
        if self.measured_hz is not None:
            return self.measured_hz
        if self.samples < 2:
            return 0.0
        return (self.samples - 1) / max(self.last_ts - self.first_ts, 1e-9)


def _point_count(msg: Any, default: int | None = None) -> int | None:
    if hasattr(msg, "point_num"):
        return int(msg.point_num)
    if hasattr(msg, "width") and hasattr(msg, "height"):
        return int(msg.width) * int(msg.height)
    poses = getattr(msg, "poses", None)
    return len(poses) if poses is not None else default


def probe(topics: tuple[str, ...], *, seconds: float, domain_id: int) -> dict[str, TopicStats]:
    _reject_legacy_topics(topics)
    binary = _ensure_native_probe()
    result = subprocess.run(
        [str(binary), "--json", "--seconds", str(seconds), "--domain", str(domain_id), *topics],
        check=False,
        capture_output=True,
        text=True,
        timeout=max(5.0, float(seconds) + 10.0),
    )
    if result.returncode not in (0, 1):
        raise RuntimeError(result.stderr.strip() or f"native DDS probe failed: {result.returncode}")
    try:
        rows = json.loads(result.stdout or "[]")
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"native DDS probe returned invalid JSON: {exc}") from exc

    stats = {
        str(row.get("topic", "")): TopicStats(
            samples=int(row.get("samples", 0) or 0),
            first_ts=float(row.get("first_ts", 0.0) or 0.0),
            last_ts=float(row.get("last_ts", 0.0) or 0.0),
            frame_id=str(row.get("frame_id", "") or ""),
            points=_optional_int(row.get("points")),
            measured_hz=_optional_float(row.get("hz")),
            max_gap_s=float(row.get("max_gap_s", 0.0) or 0.0),
            reset_epoch=_optional_int(row.get("reset_epoch")),
            observation_sequence=_optional_int(row.get("observation_sequence")),
            generation=_optional_int(row.get("generation")),
            live=row.get("live") if isinstance(row.get("live"), bool) else None,
        )
        for row in rows
    }
    return {topic: stats.get(topic, TopicStats()) for topic in topics}


def _reject_legacy_topics(topics: tuple[str, ...]) -> None:
    for topic in topics:
        if replacement := LEGACY_TOPIC_REPLACEMENTS.get(topic):
            raise ValueError(f"{topic} is legacy; use {replacement}")


def _optional_int(value: Any) -> int | None:
    try:
        return None if value is None else int(value)
    except (TypeError, ValueError):
        return None


def _optional_float(value: Any) -> float | None:
    try:
        return None if value is None else float(value)
    except (TypeError, ValueError):
        return None


def _native_probe_path() -> Path:
    if configured := os.environ.get("LINGTU_DDS_PROBE_BIN"):
        return Path(configured).expanduser()
    suffix = ".exe" if os.name == "nt" else ""
    return ROOT / "build" / "dds_probe" / f"lingtu_dds_probe{suffix}"


def _ensure_native_probe() -> Path:
    binary = _native_probe_path()
    if binary.exists():
        return binary
    if os.environ.get("LINGTU_DDS_PROBE_ALLOW_BUILD", "1").strip().lower() not in {"1", "true", "yes", "on"}:
        raise FileNotFoundError(
            f"native DDS probe is not packaged at {binary}; managed releases forbid readiness-time compilation"
        )
    build_script = ROOT / "scripts" / "build" / "build_dds_probe.sh"
    if not build_script.exists():
        raise FileNotFoundError(f"native DDS probe build script is missing: {build_script}")
    result = subprocess.run(
        ["bash", str(build_script)],
        check=False,
        capture_output=True,
        text=True,
        cwd=str(ROOT),
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "failed to build native DDS probe")
    if not binary.exists():
        raise FileNotFoundError(f"native DDS probe binary is missing after build: {binary}")
    return binary


def _dead_topic(error: str | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "samples": 0,
        "rate_hz": 0.0,
        "first_ts": 0.0,
        "last_ts": 0.0,
        "frame_id": "",
    }
    if error:
        result["error"] = error
    return result


def _topic_result(stat: Any) -> dict[str, Any]:
    if stat is None:
        return _dead_topic()
    try:
        rate_hz = float(stat.hz())
    except (AttributeError, TypeError, ValueError):
        rate_hz = 0.0
    result: dict[str, Any] = {
        "samples": int(getattr(stat, "samples", 0) or 0),
        "rate_hz": round(rate_hz, 2),
        "first_ts": float(getattr(stat, "first_ts", 0.0) or 0.0),
        "last_ts": float(getattr(stat, "last_ts", 0.0) or 0.0),
        "frame_id": str(getattr(stat, "frame_id", "") or ""),
    }
    points = getattr(stat, "points", None)
    if points is not None:
        try:
            result["points"] = int(points)
        except (TypeError, ValueError):
            pass
    return result


def collect_readiness(
    topics: Sequence[str] | None = None,
    *,
    seconds: float = 5.0,
    domain_id: int = 0,
) -> dict[str, Any]:
    """Run one native probe and return its per-topic observations."""

    selected = tuple(topics or ())
    if not selected:
        raise ValueError("topics must be provided for the Product being checked")
    valid = tuple(topic for topic in selected if topic in _WIRE_TOPICS)
    invalid = set(selected) - set(valid)
    results = {topic: _dead_topic("no typed DDS contract") for topic in invalid}

    if valid:
        try:
            observed = probe(valid, seconds=seconds, domain_id=domain_id)
        except Exception as exc:
            error = f"DDS probe failed: {exc}"
            results.update({topic: _dead_topic(error) for topic in valid})
        else:
            results.update({topic: _topic_result(observed.get(topic)) for topic in valid})

    ordered_results = {topic: results[topic] for topic in selected}
    missing = [topic for topic, item in ordered_results.items() if item["samples"] <= 0]
    return {
        "schema_version": SCHEMA_VERSION,
        "collected_at": time.time(),
        "domain_id": int(domain_id),
        "duration_s": float(seconds),
        "ok": bool(selected) and not missing,
        "missing": missing,
        "topics": ordered_results,
    }


def _format_report(report: dict[str, Any]) -> str:
    topics = report["topics"]
    alive = len(topics) - len(report["missing"])
    lines = [
        f"DDS readiness: {'OK' if report['ok'] else 'FAIL'} "
        f"({alive}/{len(topics)} active, domain={report['domain_id']}, duration={report['duration_s']}s)",
        f"  {'topic':32} {'samples':>7} {'rate_hz':>8} {'frame':16}",
    ]
    for topic, item in topics.items():
        lines.append(f"  {topic:32} {item['samples']:7d} {item['rate_hz']:8.2f} {item['frame_id'][:16]:16}")
    if report["missing"]:
        lines.append(f"  missing: {', '.join(report['missing'])}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    """Run the one-shot DDS readiness CLI."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=5.0, help="sampling window")
    parser.add_argument("--domain", type=int, default=0, help="CycloneDDS domain id")
    parser.add_argument("--topics", nargs="+", required=True, help="canonical rt/... topics required by this Product")
    parser.add_argument("--json", type=Path, help="optional JSON report path")
    args = parser.parse_args(argv)

    report = collect_readiness(args.topics, seconds=args.seconds, domain_id=args.domain)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(_format_report(report))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
