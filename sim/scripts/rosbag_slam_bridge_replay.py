#!/usr/bin/env python3
"""Offline ROS2 bag replay gate for SlamBridgeModule.

This script does not start robot services, does not publish ROS topics, and does
not send motion commands. It reads a rosbag2 sqlite/mcap bag, deserializes real
ROS2 messages, feeds selected odometry/cloud/health topics into
``SlamBridgeModule`` rclpy callbacks, and writes a JSON report.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


SLAM_ODOM_TOPICS = {
    "/Odometry",
    "/aft_mapped_to_init",
    "/test/odometry",
    "/slam/odometry",
}
SLAM_CLOUD_TOPICS = {
    "/cloud_registered",
    "/cloud_registered_body",
    "/test/map_cloud",
    "/test/registered_cloud",
    "/slam/map_cloud",
    "/slam/registered_cloud",
}
RAW_ODOM_TOPICS = {"/state_SDK", "/odom", "/odometry"}
RAW_CLOUD_TOPICS = {"/lidar/raw_frame", "/livox/lidar", "/velodyne_points"}
DEFAULT_ODOM_CANDIDATES = [
    "/slam/odometry",
    "/test/odometry",
    "/Odometry",
    "/aft_mapped_to_init",
    "/state_SDK",
    "/odom",
    "/odometry",
]
DEFAULT_CLOUD_CANDIDATES = [
    "/slam/map_cloud",
    "/test/map_cloud",
    "/cloud_registered",
    "/test/registered_cloud",
    "/cloud_registered_body",
    "/slam/registered_cloud",
    "/lidar/raw_frame",
    "/velodyne_points",
]


def _load_ros_modules():
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on ROS2 env
        raise RuntimeError(
            "ROS2 Python modules are unavailable. Source /opt/ros/humble/setup.bash "
            "or run in a ROS2 Humble environment."
        ) from exc
    return rosbag2_py, deserialize_message, get_message


def _choose_topic(topics: dict[str, str], candidates: list[str], explicit: str) -> str:
    if explicit:
        if explicit not in topics:
            raise ValueError(f"requested topic {explicit!r} not in bag")
        return explicit
    for topic in candidates:
        if topic in topics:
            return topic
    return ""


def _topic_class(odom_topic: str, cloud_topic: str) -> dict[str, bool]:
    odom_is_slam = odom_topic in SLAM_ODOM_TOPICS
    cloud_is_slam = cloud_topic in SLAM_CLOUD_TOPICS
    odom_is_raw = odom_topic in RAW_ODOM_TOPICS
    cloud_is_raw = cloud_topic in RAW_CLOUD_TOPICS
    return {
        "slam_algorithm_output_verified": bool(odom_is_slam and cloud_is_slam),
        "raw_sensor_bag_only": bool((odom_is_raw or cloud_is_raw) and not (odom_is_slam and cloud_is_slam)),
    }


def _ros2_packages() -> dict[str, Any]:
    ros2 = shutil.which("ros2")
    if not ros2:
        return {"ros2_cli": "", "available": [], "missing": ["ros2"]}
    try:
        proc = subprocess.run(
            [ros2, "pkg", "list"],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=10,
        )
    except Exception as exc:
        return {"ros2_cli": ros2, "available": [], "missing": ["ros2 pkg list"], "error": str(exc)}
    pkgs = set(proc.stdout.splitlines())
    wanted = ["pointlio", "fastlio2", "super_lio", "localizer"]
    return {
        "ros2_cli": ros2,
        "available": sorted(pkg for pkg in wanted if pkg in pkgs),
        "missing": sorted(pkg for pkg in wanted if pkg not in pkgs),
        "stderr": proc.stderr.strip()[-500:],
    }


def _stamp_ns(msg: Any, fallback: int) -> int:
    try:
        stamp = msg.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except Exception:
        return int(fallback)


def _odom_xyz(msg: Any) -> list[float] | None:
    try:
        p = msg.pose.pose.position
        return [float(p.x), float(p.y), float(p.z)]
    except Exception:
        return None


def _cloud_points(msg: Any) -> int:
    try:
        return int(msg.width) * int(msg.height)
    except Exception:
        return 0


def run_replay(
    *,
    bag: Path,
    odom_topic: str = "",
    cloud_topic: str = "",
    health_topic: str = "",
    backend_profile: str = "bridge",
    max_odom: int = 200,
    max_cloud: int = 50,
    max_bag_seconds: float = 30.0,
) -> dict[str, Any]:
    if not bag.exists():
        raise FileNotFoundError(str(bag))

    rosbag2_py, deserialize_message, get_message = _load_ros_modules()
    from localization.bridge import SlamBridgeModule

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected_odom = _choose_topic(topics, DEFAULT_ODOM_CANDIDATES, odom_topic)
    selected_cloud = _choose_topic(topics, DEFAULT_CLOUD_CANDIDATES, cloud_topic)
    selected_health = health_topic if health_topic in topics else ""
    if not selected_odom or not selected_cloud:
        raise ValueError(
            "bag must contain odometry and point cloud topics; "
            f"found topics={sorted(topics)}"
        )

    msg_types = {
        topic: get_message(type_name)
        for topic, type_name in topics.items()
        if topic in {selected_odom, selected_cloud, selected_health}
    }

    bridge = SlamBridgeModule(
        backend_profile=backend_profile,
        odom_timeout=2.0,
        cloud_timeout=2.0,
        localizer_health_timeout=2.0,
        watchdog_hz=50,
    )
    odom_out = []
    cloud_out = []
    statuses = []
    bridge.odometry._add_callback(odom_out.append)
    bridge.map_cloud._add_callback(cloud_out.append)
    bridge.localization_status._add_callback(statuses.append)

    counts = {"odom_in": 0, "cloud_in": 0, "health_in": 0}
    first_stamp: int | None = None
    last_storage_stamp: int | None = None
    last_header_stamp: int | None = None
    first_odom_xyz: list[float] | None = None
    last_odom_xyz: list[float] | None = None
    cloud_point_counts: list[int] = []
    started = time.time()

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if first_stamp is None:
            first_stamp = int(stamp)
        if max_bag_seconds > 0 and (int(stamp) - first_stamp) / 1e9 > max_bag_seconds:
            break
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        last_storage_stamp = int(stamp)
        last_header_stamp = _stamp_ns(msg, int(stamp))

        if topic == selected_odom and counts["odom_in"] < max_odom:
            counts["odom_in"] += 1
            xyz = _odom_xyz(msg)
            if xyz is not None:
                first_odom_xyz = first_odom_xyz or xyz
                last_odom_xyz = xyz
            bridge._on_rclpy_odom(msg)
            if bridge._odom_worker_thread is not None:
                bridge._odom_worker_thread.join(timeout=0.25)
        elif topic == selected_cloud and counts["cloud_in"] < max_cloud:
            counts["cloud_in"] += 1
            cloud_point_counts.append(_cloud_points(msg))
            bridge._process_rclpy_cloud(msg)
        elif selected_health and topic == selected_health:
            counts["health_in"] += 1
            bridge._on_rclpy_localization_health(msg)

        if counts["odom_in"] >= max_odom and counts["cloud_in"] >= max_cloud:
            break

    bridge.start()
    try:
        deadline = time.time() + 1.5
        while time.time() < deadline and not any(s.get("state") == "TRACKING" for s in statuses):
            time.sleep(0.02)
    finally:
        bridge.stop()

    states = [str(item.get("state")) for item in statuses]
    moved_m = None
    if first_odom_xyz and last_odom_xyz:
        moved_m = math.dist(first_odom_xyz, last_odom_xyz)
    topic_flags = _topic_class(selected_odom, selected_cloud)
    final_status = statuses[-1] if statuses else {}
    ok = bool(
        counts["odom_in"] > 0
        and counts["cloud_in"] > 0
        and len(odom_out) > 0
        and len(cloud_out) > 0
        and any(state == "TRACKING" for state in states)
    )
    return {
        "ok": ok,
        "real_rosbag_replay_verified": True,
        **topic_flags,
        "backend_profile": backend_profile,
        "bag": str(bag),
        "topics": topics,
        "selected_topics": {
            "odometry": selected_odom,
            "point_cloud": selected_cloud,
            "localization_health": selected_health or None,
        },
        "counts": counts,
        "outputs": {
            "odometry": len(odom_out),
            "map_cloud": len(cloud_out),
            "localization_status": len(statuses),
        },
        "states_seen": states,
        "final_status": final_status,
        "first_odom_xyz": first_odom_xyz,
        "last_odom_xyz": last_odom_xyz,
        "moved_m": moved_m,
        "cloud_points": {
            "min": min(cloud_point_counts) if cloud_point_counts else 0,
            "max": max(cloud_point_counts) if cloud_point_counts else 0,
            "samples": cloud_point_counts[:5],
        },
        "bag_time_s": (
            round((last_storage_stamp - first_stamp) / 1e9, 3)
            if first_stamp is not None and last_storage_stamp is not None
            else 0.0
        ),
        "first_storage_stamp_ns": first_stamp,
        "last_storage_stamp_ns": last_storage_stamp,
        "last_header_stamp_ns": last_header_stamp,
        "wall_time_s": round(time.time() - started, 3),
        "ros2_slam_packages": _ros2_packages(),
        "limitations": [
            "This is offline rosbag CDR replay into SlamBridgeModule callbacks.",
            "If selected topics are raw /lidar/raw_frame + /state_SDK, this does not prove Fast-LIO2/Super-LIO algorithm output.",
            "SLAM algorithm output is verified only when replaying /Odometry plus /cloud_registered or remapped equivalents.",
        ],
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="rosbag2 directory")
    parser.add_argument("--odom-topic", default="")
    parser.add_argument("--cloud-topic", default="")
    parser.add_argument("--health-topic", default="")
    parser.add_argument("--backend-profile", default="bridge")
    parser.add_argument("--max-odom", type=int, default=200)
    parser.add_argument("--max-cloud", type=int, default=50)
    parser.add_argument("--max-bag-seconds", type=float, default=30.0)
    parser.add_argument("--json-out", default="")
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_replay(
        bag=Path(args.bag),
        odom_topic=args.odom_topic,
        cloud_topic=args.cloud_topic,
        health_topic=args.health_topic,
        backend_profile=args.backend_profile,
        max_odom=args.max_odom,
        max_cloud=args.max_cloud,
        max_bag_seconds=args.max_bag_seconds,
    )
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    return 0 if report.get("ok") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
