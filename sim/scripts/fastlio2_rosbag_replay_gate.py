#!/usr/bin/env python3
"""Run Fast-LIO2 against a ROS2 bag and verify LingTu bridge outputs.

This gate is offline and non-motion: it starts only the Fast-LIO2 ROS2 node,
publishes selected IMU and PointCloud2 messages from a rosbag2 file, subscribes
to Fast-LIO2's `/Odometry` and `/cloud_registered`, and feeds those real
algorithm outputs into `SlamBridgeModule`.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
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


def _load_ros_modules():
    try:
        import rclpy  # type: ignore
        import rosbag2_py  # type: ignore
        from nav_msgs.msg import Odometry  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
        from sensor_msgs.msg import PointCloud2  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on ROS2 env
        raise RuntimeError(
            "ROS2 Python modules are unavailable. Source /opt/ros/humble/setup.bash "
            "and the LingTu install/setup.bash before running this gate."
        ) from exc
    return rclpy, rosbag2_py, deserialize_message, get_message, Odometry, PointCloud2


def _write_fastlio2_config(
    path: Path,
    *,
    imu_topic: str,
    lidar_topic: str,
    lidar_type: int = 3,
    scan_line: int = 4,
    timestamp_unit: int = 3,
    livox_scan_window: float | None = None,
    r_il: tuple[float, ...] | list[float] | None = None,
    t_il: tuple[float, ...] | list[float] | None = None,
    imu_static_acc_thresh: float = 0.04,
    imu_static_gyro_thresh: float = 0.001,
    zupt_min_static_frames: int = 5,
    degeneracy_max_update_dof: int = 2,
    degeneracy_max_condition: float = 50_000.0,
    max_update_translation_m: float = 0.5,
    max_update_rotation_rad: float = 0.35,
    reject_nonconverged_update: bool = True,
    reject_degenerate_nonconverged_update: bool = True,
) -> None:
    r_il_values = list(r_il) if r_il is not None else [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    t_il_values = list(t_il) if t_il is not None else [0.0, 0.0, 0.0]
    if len(r_il_values) != 9:
        raise ValueError("r_il must contain 9 row-major rotation values")
    if len(t_il_values) != 3:
        raise ValueError("t_il must contain 3 translation values")
    r_il_text = ", ".join(f"{float(value):.9g}" for value in r_il_values)
    t_il_text = ", ".join(f"{float(value):.9g}" for value in t_il_values)
    lines = [
        f"imu_topic: {imu_topic}",
        f"lidar_topic: {lidar_topic}",
        "body_frame: body",
        "world_frame: odom",
        "print_time_cost: false",
        "",
        "# Generic PointCloud2 replay. The source bag is MID-360 raw cloud,",
        "# but this gate verifies algorithm I/O without Livox CustomMsg.",
        f"lidar_type: {int(lidar_type)}",
        f"scan_line: {int(scan_line)}",
        f"timestamp_unit: {int(timestamp_unit)}",
        "acc_scale: 1.0",
    ]
    if livox_scan_window is not None:
        lines.append(f"livox_scan_window: {float(livox_scan_window):.6f}")
    lines.extend(
        [
            "",
            "lidar_filter_num: 4",
            "lidar_min_range: 0.5",
            "lidar_max_range: 30.0",
            "scan_resolution: 0.15",
            "map_resolution: 0.3",
            "",
            "cube_len: 2000",
            "det_range: 60",
            "move_thresh: 1.5",
            "max_map_points: 1000000",
            "stationary_thresh: 0.05",
            "",
            "na: 0.01",
            "ng: 0.01",
            "nba: 0.0001",
            "nbg: 0.0001",
            "",
            "imu_init_num: 20",
            "near_search_num: 5",
            "ieskf_max_iter: 5",
            f"degeneracy_max_update_dof: {int(degeneracy_max_update_dof)}",
            f"degeneracy_max_condition: {float(degeneracy_max_condition):.9g}",
            f"max_update_translation_m: {float(max_update_translation_m):.9g}",
            f"max_update_rotation_rad: {float(max_update_rotation_rad):.9g}",
            "reject_nonconverged_update: "
            f"{str(bool(reject_nonconverged_update)).lower()}",
            "reject_degenerate_nonconverged_update: "
            f"{str(bool(reject_degenerate_nonconverged_update)).lower()}",
            "time_diff_lidar_to_imu: 0.0",
            "",
            "gravity_align: true",
            "esti_il: false",
            f"r_il: [{r_il_text}]",
            f"t_il: [{t_il_text}]",
            "lidar_cov_inv: 1000.0",
            "",
            f"imu_static_acc_thresh: {float(imu_static_acc_thresh):.9g}",
            f"imu_static_gyro_thresh: {float(imu_static_gyro_thresh):.9g}",
            f"zupt_min_static_frames: {int(zupt_min_static_frames)}",
            "zupt_sigma_v: 0.02",
            "zupt_sigma_pos: 0.1",
            "",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def _odom_xyz(msg: Any) -> list[float] | None:
    try:
        p = msg.pose.pose.position
        return [float(p.x), float(p.y), float(p.z)]
    except Exception:
        return None


def _tail(path: Path, limit: int = 4000) -> str:
    if not path.exists():
        return ""
    data = path.read_bytes()
    return data[-limit:].decode("utf-8", errors="replace")


def run_gate(
    *,
    bag: Path,
    imu_topic: str,
    lidar_topic: str,
    max_imu: int,
    max_cloud: int,
    max_bag_seconds: float,
    playback_rate: float,
    startup_sleep: float,
    settle_sleep: float,
    work_dir: Path,
    backend_profile: str,
) -> dict[str, Any]:
    if not bag.exists():
        raise FileNotFoundError(str(bag))

    rclpy, rosbag2_py, deserialize_message, get_message, Odometry, PointCloud2 = _load_ros_modules()
    from localization.bridge import SlamBridgeModule

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in (imu_topic, lidar_topic) if topic not in topics]
    if missing:
        raise ValueError(f"bag is missing required topics: {missing}; found={sorted(topics)}")

    work_dir.mkdir(parents=True, exist_ok=True)
    config_path = work_dir / "fastlio2_replay.yaml"
    log_path = work_dir / "fastlio2_node.log"
    _write_fastlio2_config(config_path, imu_topic=imu_topic, lidar_topic=lidar_topic)

    ros2 = shutil.which("ros2")
    if not ros2:
        raise RuntimeError("ros2 CLI is not available")

    rclpy.init(args=None)
    node = rclpy.create_node("fastlio2_rosbag_replay_gate")
    bridge = SlamBridgeModule(
        backend_profile=backend_profile,
        odom_timeout=2.0,
        cloud_timeout=2.0,
        localizer_health_timeout=2.0,
        watchdog_hz=20,
    )

    odom_out: list[Any] = []
    cloud_out: list[Any] = []
    bridge_statuses: list[dict[str, Any]] = []
    first_odom_xyz: list[float] | None = None
    last_odom_xyz: list[float] | None = None

    def on_odom(msg: Any) -> None:
        nonlocal first_odom_xyz, last_odom_xyz
        odom_out.append(msg)
        xyz = _odom_xyz(msg)
        if xyz is not None:
            first_odom_xyz = first_odom_xyz or xyz
            last_odom_xyz = xyz
        bridge._on_rclpy_odom(msg)
        worker = getattr(bridge, "_odom_worker_thread", None)
        if worker is not None:
            worker.join(timeout=0.1)

    def on_cloud(msg: Any) -> None:
        cloud_out.append(msg)
        bridge._process_rclpy_cloud(msg)

    bridge.localization_status._add_callback(bridge_statuses.append)
    node.create_subscription(Odometry, "/Odometry", on_odom, 100)
    node.create_subscription(PointCloud2, "/cloud_registered", on_cloud, 10)

    msg_types = {
        imu_topic: get_message(topics[imu_topic]),
        lidar_topic: get_message(topics[lidar_topic]),
    }
    publishers = {
        imu_topic: node.create_publisher(msg_types[imu_topic], imu_topic, 100),
        lidar_topic: node.create_publisher(msg_types[lidar_topic], lidar_topic, 10),
    }

    process: subprocess.Popen[str] | None = None
    log_file = log_path.open("w", encoding="utf-8")
    counts = {"imu_published": 0, "cloud_published": 0}
    first_storage_stamp: int | None = None
    last_storage_stamp: int | None = None
    previous_publish_stamp: int | None = None
    subscription_counts = {"imu": 0, "cloud": 0}
    started = time.time()
    try:
        bridge.start()
        process = subprocess.Popen(
            [
                ros2,
                "run",
                "fastlio2",
                "lio_node",
                "--ros-args",
                "-p",
                f"config_path:={config_path}",
            ],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            env=os.environ.copy(),
            start_new_session=True,
        )
        deadline = time.time() + startup_sleep
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if process.poll() is not None:
                break
            subscription_counts = {
                "imu": publishers[imu_topic].get_subscription_count(),
                "cloud": publishers[lidar_topic].get_subscription_count(),
            }
            if subscription_counts["imu"] > 0 and subscription_counts["cloud"] > 0:
                break

        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if first_storage_stamp is None:
                first_storage_stamp = int(stamp)
            if max_bag_seconds > 0 and (int(stamp) - first_storage_stamp) / 1e9 > max_bag_seconds:
                break
            if topic not in msg_types:
                continue
            if topic == imu_topic and counts["imu_published"] >= max_imu:
                continue
            if topic == lidar_topic and counts["cloud_published"] >= max_cloud:
                continue
            if playback_rate > 0 and previous_publish_stamp is not None:
                delay_s = (int(stamp) - previous_publish_stamp) / 1e9 / playback_rate
                deadline = time.time() + max(0.0, delay_s)
                while time.time() < deadline:
                    rclpy.spin_once(node, timeout_sec=min(0.02, max(0.0, deadline - time.time())))
            msg = deserialize_message(data, msg_types[topic])
            publishers[topic].publish(msg)
            if topic == imu_topic:
                counts["imu_published"] += 1
            else:
                counts["cloud_published"] += 1
            last_storage_stamp = int(stamp)
            previous_publish_stamp = int(stamp)
            rclpy.spin_once(node, timeout_sec=0.0)

            if counts["imu_published"] >= max_imu and counts["cloud_published"] >= max_cloud:
                break

        deadline = time.time() + settle_sleep
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        bridge.stop()
        node.destroy_node()
        rclpy.shutdown()
        log_file.close()

    states = [str(item.get("state")) for item in bridge_statuses]
    moved_m = math.dist(first_odom_xyz, last_odom_xyz) if first_odom_xyz and last_odom_xyz else None
    process_returncode = process.returncode if process is not None else None
    ok = bool(
        counts["imu_published"] > 0
        and counts["cloud_published"] > 0
        and len(odom_out) > 0
        and len(cloud_out) > 0
        and any(state == "TRACKING" for state in states)
    )
    return {
        "ok": ok,
        "algorithm": "fastlio2",
        "slam_algorithm_output_verified": len(odom_out) > 0 and len(cloud_out) > 0,
        "raw_sensor_bag_only": False,
        "real_rosbag_replay_verified": True,
        "bridge_verified": any(state == "TRACKING" for state in states),
        "bag": str(bag),
        "topics": topics,
        "selected_topics": {"imu": imu_topic, "point_cloud": lidar_topic},
        "counts": counts,
        "subscription_counts_at_start": subscription_counts,
        "outputs": {
            "fastlio2_odometry": len(odom_out),
            "fastlio2_cloud_registered": len(cloud_out),
            "bridge_localization_status": len(bridge_statuses),
        },
        "states_seen": states,
        "final_bridge_status": bridge_statuses[-1] if bridge_statuses else {},
        "first_odom_xyz": first_odom_xyz,
        "last_odom_xyz": last_odom_xyz,
        "moved_m": moved_m,
        "bag_time_s": (
            round((last_storage_stamp - first_storage_stamp) / 1e9, 3)
            if first_storage_stamp is not None and last_storage_stamp is not None
            else 0.0
        ),
        "first_storage_stamp_ns": first_storage_stamp,
        "last_storage_stamp_ns": last_storage_stamp,
        "wall_time_s": round(time.time() - started, 3),
        "process_returncode": process_returncode,
        "config_path": str(config_path),
        "log_path": str(log_path),
        "log_tail": _tail(log_path),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--imu-topic", default="/imu/raw")
    parser.add_argument("--lidar-topic", default="/lidar/raw_frame")
    parser.add_argument("--max-imu", type=int, default=3000)
    parser.add_argument("--max-cloud", type=int, default=30)
    parser.add_argument("--max-bag-seconds", type=float, default=8.0)
    parser.add_argument("--playback-rate", type=float, default=0.0)
    parser.add_argument("--startup-sleep", type=float, default=2.0)
    parser.add_argument("--settle-sleep", type=float, default=3.0)
    parser.add_argument("--backend-profile", default="fastlio2")
    parser.add_argument("--work-dir", default="artifacts/fastlio2_rosbag_replay")
    parser.add_argument("--json-out", default="")
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_gate(
        bag=Path(args.bag),
        imu_topic=args.imu_topic,
        lidar_topic=args.lidar_topic,
        max_imu=args.max_imu,
        max_cloud=args.max_cloud,
        max_bag_seconds=args.max_bag_seconds,
        playback_rate=args.playback_rate,
        startup_sleep=args.startup_sleep,
        settle_sleep=args.settle_sleep,
        work_dir=Path(args.work_dir),
        backend_profile=args.backend_profile,
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
