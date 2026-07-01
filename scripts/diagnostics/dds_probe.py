#!/usr/bin/env python
"""Probe LingTu typed DDS topics without ROS2 CLI."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from message.dds import dds_type_for_topic  # noqa: E402
from runtime.dds import DDSReader  # noqa: E402
from runtime.runtime_interface import TOPICS  # noqa: E402


DEFAULT_TOPICS = (
    TOPICS.lidar_scan,
    TOPICS.imu,
    TOPICS.odometry,
    TOPICS.registered_cloud,
    TOPICS.map_cloud,
    TOPICS.localization_health,
)


@dataclass
class TopicStats:
    samples: int = 0
    first_ts: float = 0.0
    last_ts: float = 0.0
    frame_id: str = ""
    points: int | None = None

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
        if self.samples < 2:
            return 0.0
        span = max(self.last_ts - self.first_ts, 1e-9)
        return (self.samples - 1) / span


def _point_count(msg: Any, default: int | None = None) -> int | None:
    if hasattr(msg, "point_num"):
        return int(getattr(msg, "point_num"))
    if hasattr(msg, "width") and hasattr(msg, "height"):
        return int(getattr(msg, "width")) * int(getattr(msg, "height"))
    poses = getattr(msg, "poses", None)
    if poses is not None:
        return len(poses)
    return default


def probe(topics: tuple[str, ...], *, seconds: float, domain_id: int) -> dict[str, TopicStats]:
    reader = DDSReader(domain_id=domain_id)
    stats = {topic: TopicStats() for topic in topics}
    for topic in topics:
        dds_type = dds_type_for_topic(topic)
        if dds_type is None:
            raise ValueError(f"no typed DDS contract for {topic}")
        reader.subscribe(topic, dds_type, lambda msg, topic=topic: stats[topic].observe(msg))
    reader.spin_background()
    try:
        time.sleep(seconds)
    finally:
        reader.stop()
    return stats


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("topics", nargs="*", default=list(DEFAULT_TOPICS))
    args = parser.parse_args(argv)

    stats = probe(tuple(args.topics), seconds=args.seconds, domain_id=args.domain)
    print(f"{'topic':36} {'samples':>7} {'hz':>7} {'frame':16} {'points':>8} {'age_s':>7}")
    ok = True
    now = time.time()
    for topic, item in stats.items():
        ok = ok and item.samples > 0
        age = now - item.last_ts if item.last_ts else 0.0
        points = "" if item.points is None else str(item.points)
        print(f"{topic:36} {item.samples:7d} {item.hz():7.2f} {item.frame_id[:16]:16} {points:>8} {age:7.2f}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
