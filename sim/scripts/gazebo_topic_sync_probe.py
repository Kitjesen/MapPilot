#!/usr/bin/env python3
"""Probe Gazebo odometry and point-cloud topic timing contracts."""

from __future__ import annotations

import argparse
import json
import statistics
import time
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class TopicStats:
    topic: str
    samples: int = 0
    frames: set[str] = field(default_factory=set)
    child_frames: set[str] = field(default_factory=set)
    stamp_sec: list[float] = field(default_factory=list)
    receive_sec: list[float] = field(default_factory=list)
    point_counts: list[int] = field(default_factory=list)

    def add(self, *, frame_id: str, stamp_sec: float, child_frame_id: str = "", point_count: int = 0) -> None:
        self.samples += 1
        self.frames.add(frame_id)
        if child_frame_id:
            self.child_frames.add(child_frame_id)
        self.stamp_sec.append(stamp_sec)
        self.receive_sec.append(time.monotonic())
        if point_count:
            self.point_counts.append(point_count)

    def rate_hz(self) -> float:
        if self.samples < 2:
            return 0.0
        dt = max(self.receive_sec[-1] - self.receive_sec[0], 1e-9)
        return (self.samples - 1) / dt

    def stamp_monotonic(self) -> bool:
        if len(self.stamp_sec) < 2:
            return True
        return all(b >= a for a, b in zip(self.stamp_sec, self.stamp_sec[1:]))

    def as_dict(self) -> dict:
        return {
            "samples": self.samples,
            "rate_hz": round(self.rate_hz(), 3),
            "frames": sorted(self.frames),
            "child_frames": sorted(self.child_frames),
            "stamp_first_sec": self.stamp_sec[0] if self.stamp_sec else None,
            "stamp_last_sec": self.stamp_sec[-1] if self.stamp_sec else None,
            "stamp_monotonic": self.stamp_monotonic(),
            "point_count_last": self.point_counts[-1] if self.point_counts else None,
        }


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _nearest_skews(a: list[float], b: list[float]) -> list[float]:
    if not a or not b:
        return []
    b_sorted = sorted(b)
    skews: list[float] = []
    for stamp in a:
        lo = 0
        hi = len(b_sorted)
        while lo < hi:
            mid = (lo + hi) // 2
            if b_sorted[mid] < stamp:
                lo = mid + 1
            else:
                hi = mid
        candidates = []
        if lo < len(b_sorted):
            candidates.append(abs(b_sorted[lo] - stamp))
        if lo > 0:
            candidates.append(abs(b_sorted[lo - 1] - stamp))
        if candidates:
            skews.append(min(candidates))
    return skews


def _skew_report(reference: TopicStats, other: TopicStats) -> dict:
    skews = _nearest_skews(reference.stamp_sec, other.stamp_sec)
    if not skews:
        return {"samples": 0, "max_sec": None, "mean_sec": None, "p95_sec": None}
    sorted_skews = sorted(skews)
    p95_idx = min(len(sorted_skews) - 1, int(0.95 * (len(sorted_skews) - 1)))
    return {
        "samples": len(skews),
        "max_sec": round(max(skews), 6),
        "mean_sec": round(statistics.fmean(skews), 6),
        "p95_sec": round(sorted_skews[p95_idx], 6),
    }


def run_probe(args: argparse.Namespace) -> dict:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2

    rclpy.init(args=None)
    node = Node("lingtu_gazebo_topic_sync_probe")
    stats = {
        "/slam/odometry": TopicStats("/slam/odometry"),
        "/slam/map_cloud": TopicStats("/slam/map_cloud"),
        "/slam/registered_cloud": TopicStats("/slam/registered_cloud"),
    }

    def on_odom(msg: Odometry) -> None:
        stats["/slam/odometry"].add(
            frame_id=str(msg.header.frame_id),
            child_frame_id=str(msg.child_frame_id),
            stamp_sec=_stamp_sec(msg.header.stamp),
        )

    def on_map_cloud(msg: PointCloud2) -> None:
        stats["/slam/map_cloud"].add(
            frame_id=str(msg.header.frame_id),
            stamp_sec=_stamp_sec(msg.header.stamp),
            point_count=int(msg.width) * int(msg.height),
        )

    def on_registered_cloud(msg: PointCloud2) -> None:
        stats["/slam/registered_cloud"].add(
            frame_id=str(msg.header.frame_id),
            stamp_sec=_stamp_sec(msg.header.stamp),
            point_count=int(msg.width) * int(msg.height),
        )

    node.create_subscription(Odometry, "/slam/odometry", on_odom, 10)
    node.create_subscription(PointCloud2, "/slam/map_cloud", on_map_cloud, 10)
    node.create_subscription(PointCloud2, "/slam/registered_cloud", on_registered_cloud, 10)

    deadline = time.monotonic() + args.timeout_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(item.samples >= args.min_samples for item in stats.values()):
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    report = {
        "schema_version": "lingtu.gazebo_topic_sync_probe.v1",
        "ok": True,
        "simulation_only": True,
        "topics": {name: item.as_dict() for name, item in stats.items()},
        "skew_to_odometry_sec": {
            "/slam/map_cloud": _skew_report(stats["/slam/odometry"], stats["/slam/map_cloud"]),
            "/slam/registered_cloud": _skew_report(stats["/slam/odometry"], stats["/slam/registered_cloud"]),
        },
        "errors": [],
    }
    expected_frames = {
        "/slam/odometry": {"odom"},
        "/slam/map_cloud": {"odom"},
        "/slam/registered_cloud": {"body"},
    }
    for topic, topic_stats in stats.items():
        if topic_stats.samples < args.min_samples:
            report["errors"].append(f"{topic} samples {topic_stats.samples} < {args.min_samples}")
        if not topic_stats.stamp_monotonic():
            report["errors"].append(f"{topic} header timestamps are not monotonic")
        if not expected_frames[topic].issubset(topic_stats.frames):
            report["errors"].append(f"{topic} expected frame {sorted(expected_frames[topic])}, got {sorted(topic_stats.frames)}")
    if "body" not in stats["/slam/odometry"].child_frames:
        report["errors"].append("/slam/odometry expected child_frame_id body")
    report["ok"] = not report["errors"]
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument("--min-samples", type=int, default=3)
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()

    report = run_probe(args)
    payload = json.dumps(report, ensure_ascii=False, indent=2)
    print(payload)
    if args.json_out:
        path = Path(args.json_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(payload + "\n", encoding="utf-8")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
