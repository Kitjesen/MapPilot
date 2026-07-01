#!/usr/bin/env python3
"""Collect Gazebo/LingTu point-cloud geometry diagnostics.

This script is intentionally read-only. It samples the raw Gazebo lidar cloud,
normalized LingTu clouds, and odometry, then reports frame/range statistics plus
the exact near-field obstacle predicate used by localPlanner.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class TopicStats:
    samples: int = 0
    frame_id: str | None = None
    point_count_last: int = 0
    min_xyz: list[float] = field(default_factory=lambda: [math.inf, math.inf, math.inf])
    max_xyz: list[float] = field(default_factory=lambda: [-math.inf, -math.inf, -math.inf])
    near_field_hits_last: int = 0
    near_field_hits_max: int = 0
    obstacle_hits_last: int = 0
    obstacle_hits_max: int = 0

    def as_dict(self) -> dict[str, Any]:
        return {
            "samples": self.samples,
            "frame_id": self.frame_id,
            "point_count_last": self.point_count_last,
            "min_xyz": [None if math.isinf(v) else v for v in self.min_xyz],
            "max_xyz": [None if math.isinf(v) else v for v in self.max_xyz],
            "near_field_hits_last": self.near_field_hits_last,
            "near_field_hits_max": self.near_field_hits_max,
            "obstacle_hits_last": self.obstacle_hits_last,
            "obstacle_hits_max": self.obstacle_hits_max,
        }


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--json-out")
    args = parser.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    rclpy.init()
    node = Node("lingtu_gazebo_cloud_stats")
    topics = {
        "/lingtu/gazebo/raw/lidar_points": TopicStats(),
        "/slam/registered_cloud": TopicStats(),
        "/slam/map_cloud": TopicStats(),
        "/nav/terrain_map": TopicStats(),
        "/nav/terrain_map_ext": TopicStats(),
        "/slam/cumulative_map_cloud": TopicStats(),
    }
    odom = {
        "samples": 0,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "yaw": 0.0,
    }

    def on_odom(msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        odom["samples"] += 1
        odom["x"] = float(pos.x)
        odom["y"] = float(pos.y)
        odom["z"] = float(pos.z)
        odom["yaw"] = _yaw_from_quat(
            float(ori.x),
            float(ori.y),
            float(ori.z),
            float(ori.w),
        )

    def make_cloud_cb(topic: str):
        def on_cloud(msg: PointCloud2) -> None:
            stats = topics[topic]
            stats.samples += 1
            stats.frame_id = msg.header.frame_id
            names = {field.name for field in msg.fields}
            if not {"x", "y", "z"} <= names:
                return
            fields = ["x", "y", "z"]
            has_intensity = "intensity" in names
            if has_intensity:
                fields.append("intensity")
            point_count = 0
            near_hits = 0
            obstacle_hits = 0
            cy = math.cos(-float(odom["yaw"]))
            sy = math.sin(-float(odom["yaw"]))
            ox = float(odom["x"])
            oy = float(odom["y"])
            oz = float(odom["z"])
            for raw in point_cloud2.read_points(
                msg,
                field_names=fields,
                skip_nans=True,
            ):
                x, y, z = float(raw[0]), float(raw[1]), float(raw[2])
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                point_count += 1
                stats.min_xyz[0] = min(stats.min_xyz[0], x)
                stats.min_xyz[1] = min(stats.min_xyz[1], y)
                stats.min_xyz[2] = min(stats.min_xyz[2], z)
                stats.max_xyz[0] = max(stats.max_xyz[0], x)
                stats.max_xyz[1] = max(stats.max_xyz[1], y)
                stats.max_xyz[2] = max(stats.max_xyz[2], z)
                if topic.startswith("/nav/terrain_map"):
                    dx = x - ox
                    dy = y - oy
                    bx = dx * cy - dy * sy
                    by = dx * sy + dy * cy
                    height = float(raw[3]) if has_intensity else max(0.0, z - oz)
                    if bx > 0.0 and bx < 0.50 and abs(by) < 0.25:
                        near_hits += 1
                        if height > 0.20:
                            obstacle_hits += 1
            stats.point_count_last = point_count
            stats.near_field_hits_last = near_hits
            stats.near_field_hits_max = max(stats.near_field_hits_max, near_hits)
            stats.obstacle_hits_last = obstacle_hits
            stats.obstacle_hits_max = max(stats.obstacle_hits_max, obstacle_hits)

        return on_cloud

    node.create_subscription(Odometry, "/slam/odometry", on_odom, 10)
    for topic in topics:
        node.create_subscription(PointCloud2, topic, make_cloud_cb(topic), 10)

    deadline = time.monotonic() + max(0.1, args.duration_sec)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    report = {
        "schema_version": "lingtu.gazebo_cloud_stats.v1",
        "odom": odom,
        "topics": {topic: stats.as_dict() for topic, stats in topics.items()},
    }
    payload = json.dumps(report, indent=2, sort_keys=True)
    print(payload)
    if args.json_out:
        path = Path(args.json_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(payload + "\n", encoding="utf-8")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
