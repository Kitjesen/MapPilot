#!/usr/bin/env python
"""Probe LingTu typed DDS topics without ROS2 or cyclonedds-python."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from runtime.runtime_interface import TOPICS

DEFAULT_TOPICS = (
    TOPICS.lidar_scan,
    TOPICS.imu,
    TOPICS.odometry,
    TOPICS.registered_cloud,
    TOPICS.map_cloud,
    TOPICS.localization_health,
)

LEGACY_TOPIC_REPLACEMENTS = {
    "/nav/dog_odometry": TOPICS.driver_odometry,
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
        span = max(self.last_ts - self.first_ts, 1e-9)
        return (self.samples - 1) / span


def _point_count(msg: Any, default: int | None = None) -> int | None:
    if hasattr(msg, "point_num"):
        return int(msg.point_num)
    if hasattr(msg, "width") and hasattr(msg, "height"):
        return int(msg.width) * int(msg.height)
    poses = getattr(msg, "poses", None)
    if poses is not None:
        return len(poses)
    return default


def probe(topics: tuple[str, ...], *, seconds: float, domain_id: int) -> dict[str, TopicStats]:
    _reject_legacy_topics(topics)
    binary = _ensure_native_probe()
    command = [
        str(binary),
        "--json",
        "--seconds",
        str(seconds),
        "--domain",
        str(domain_id),
        *topics,
    ]
    result = subprocess.run(
        command,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode not in (0, 1):
        stderr = result.stderr.strip()
        raise RuntimeError(stderr or f"native DDS probe failed: {result.returncode}")
    try:
        rows = json.loads(result.stdout or "[]")
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"native DDS probe returned invalid JSON: {exc}") from exc

    stats: dict[str, TopicStats] = {}
    for row in rows:
        topic = str(row.get("topic", ""))
        item = TopicStats(
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
        stats[topic] = item
    return {topic: stats.get(topic, TopicStats()) for topic in topics}


def _reject_legacy_topics(topics: tuple[str, ...]) -> None:
    for topic in topics:
        replacement = LEGACY_TOPIC_REPLACEMENTS.get(topic)
        if replacement is not None:
            raise ValueError(f"{topic} is legacy; use {replacement}")


def _optional_int(value: Any) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _native_probe_path() -> Path:
    configured = os.environ.get("LINGTU_DDS_PROBE_BIN")
    if configured:
        return Path(configured).expanduser()
    suffix = ".exe" if os.name == "nt" else ""
    return ROOT / "build" / "dds_probe" / f"lingtu_dds_probe{suffix}"


def _ensure_native_probe() -> Path:
    binary = _native_probe_path()
    if binary.exists():
        return binary
    allow_build = os.environ.get("LINGTU_DDS_PROBE_ALLOW_BUILD", "1").strip().lower()
    if allow_build not in {"1", "true", "yes", "on"}:
        raise FileNotFoundError(
            "native DDS probe is not packaged at "
            f"{binary}; managed releases forbid readiness-time compilation"
        )
    build_script = ROOT / "scripts" / "build" / "build_dds_probe.sh"
    if not build_script.exists():
        raise FileNotFoundError(f"native DDS probe build script is missing: {build_script}")
    result = subprocess.run(
        ["bash", str(build_script)],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=str(ROOT),
    )
    if result.returncode != 0:
        stderr = result.stderr.strip()
        raise RuntimeError(stderr or "failed to build native DDS probe")
    if not binary.exists():
        raise FileNotFoundError(f"native DDS probe binary is missing after build: {binary}")
    return binary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--json", action="store_true", help="emit machine-readable stats")
    parser.add_argument("topics", nargs="*", default=list(DEFAULT_TOPICS))
    args = parser.parse_args(argv)

    try:
        stats = probe(tuple(args.topics), seconds=args.seconds, domain_id=args.domain)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    ok = all(item.samples > 0 for item in stats.values())
    if args.json:
        print(
            json.dumps(
                [
                    {
                        "topic": topic,
                        "samples": item.samples,
                        "hz": item.hz(),
                        "max_gap_s": item.max_gap_s,
                        "frame_id": item.frame_id,
                        "points": item.points,
                        **(
                            {
                                "reset_epoch": item.reset_epoch,
                                "observation_sequence": item.observation_sequence,
                                "generation": item.generation,
                            }
                            if item.reset_epoch is not None
                            else {}
                        ),
                        **({"live": item.live} if item.live is not None else {}),
                    }
                    for topic, item in stats.items()
                ],
                separators=(",", ":"),
            )
        )
        return 0 if ok else 1
    print(f"{'topic':36} {'samples':>7} {'hz':>7} {'frame':16} {'points':>8} {'age_s':>7}")
    now = time.time()
    for topic, item in stats.items():
        age = now - item.last_ts if item.last_ts else 0.0
        points = "" if item.points is None else str(item.points)
        print(f"{topic:36} {item.samples:7d} {item.hz():7.2f} {item.frame_id[:16]:16} {points:>8} {age:7.2f}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
