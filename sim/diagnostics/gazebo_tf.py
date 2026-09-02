#!/usr/bin/env python3
"""Read-only ROS 2 smoke test for the LingTu Gazebo/TF contract.

The script does not publish goals, cmd_vel, or any robot-control topic. It only
listens for TF and odometry, then verifies that the runtime exposes the
canonical map->odom->body frame chain declared by runtime.runtime_interface.
With --require-sensors it also verifies the normalized Gazebo point-cloud topics
used by maps and local planning. Use --require-camera when the server has a
working headless rendering backend and camera evidence is part of the gate.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass


@dataclass
class TfSmokeResult:
    ok: bool
    samples: int
    odometry_seen: bool
    odometry_frame_id: str | None
    odometry_child_frame_id: str | None
    topic_samples: dict[str, int]
    topic_frames: dict[str, str | None]
    point_counts: dict[str, int]
    errors: list[str]

    def as_dict(self) -> dict:
        return {
            "schema_version": "lingtu.gazebo_runtime_smoke.v1",
            "ok": self.ok,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "samples": self.samples,
            "odometry_seen": self.odometry_seen,
            "odometry_frame_id": self.odometry_frame_id,
            "odometry_child_frame_id": self.odometry_child_frame_id,
            "topic_samples": self.topic_samples,
            "topic_frames": self.topic_frames,
            "point_counts": self.point_counts,
            "errors": self.errors,
        }


def _finite_transform(transform) -> bool:
    trans = transform.transform.translation
    rot = transform.transform.rotation
    values = (trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w)
    return all(math.isfinite(float(value)) for value in values)


def run_smoke(timeout_sec: float, min_samples: int, require_sensors: bool, require_camera: bool) -> TfSmokeResult:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init(args=None)
    node = Node("lingtu_tf_contract_smoke")
    buffer = Buffer()
    TransformListener(buffer, node)

    odom_state = {
        "seen": False,
        "frame_id": None,
        "child_frame_id": None,
    }
    topic_samples: dict[str, int] = {}
    topic_frames: dict[str, str | None] = {}
    point_counts: dict[str, int] = {}

    def on_odom(msg: Odometry) -> None:
        odom_state["seen"] = True
        odom_state["frame_id"] = msg.header.frame_id
        odom_state["child_frame_id"] = msg.child_frame_id

    node.create_subscription(Odometry, "/slam/odometry", on_odom, 10)

    def count_topic(name: str, frame_id: str | None, points: int | None = None) -> None:
        topic_samples[name] = topic_samples.get(name, 0) + 1
        topic_frames[name] = frame_id
        if points is not None:
            point_counts[name] = max(point_counts.get(name, 0), points)

    def on_map_cloud(msg: PointCloud2) -> None:
        count_topic("/slam/map_cloud", msg.header.frame_id, int(msg.width) * int(msg.height))

    def on_registered_cloud(msg: PointCloud2) -> None:
        count_topic("/slam/registered_cloud", msg.header.frame_id, int(msg.width) * int(msg.height))

    def on_color(msg: Image) -> None:
        count_topic("/camera/color/image_raw", msg.header.frame_id)

    def on_depth(msg: Image) -> None:
        count_topic("/camera/depth/image_raw", msg.header.frame_id)

    def on_info(msg: CameraInfo) -> None:
        count_topic("/camera/color/camera_info", msg.header.frame_id)

    if require_sensors:
        node.create_subscription(PointCloud2, "/slam/map_cloud", on_map_cloud, 10)
        node.create_subscription(PointCloud2, "/slam/registered_cloud", on_registered_cloud, 10)
    if require_camera:
        node.create_subscription(Image, "/camera/color/image_raw", on_color, 10)
        node.create_subscription(Image, "/camera/depth/image_raw", on_depth, 10)
        node.create_subscription(CameraInfo, "/camera/color/camera_info", on_info, 10)

    errors: list[str] = []
    samples = 0
    deadline = time.monotonic() + timeout_sec
    last_error = ""

    def required_observations_ready() -> bool:
        if samples < min_samples:
            return False
        if require_sensors:
            for topic in ("/slam/map_cloud", "/slam/registered_cloud"):
                if topic_samples.get(topic, 0) <= 0 or point_counts.get(topic, 0) <= 0:
                    return False
        if require_camera:
            for topic in (
                "/camera/color/image_raw",
                "/camera/depth/image_raw",
                "/camera/color/camera_info",
            ):
                if topic_samples.get(topic, 0) <= 0:
                    return False
        return True

    try:
        while time.monotonic() < deadline and not required_observations_ready():
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                map_to_odom = buffer.lookup_transform("map", "odom", rclpy.time.Time())
                odom_to_body = buffer.lookup_transform("odom", "body", rclpy.time.Time())
                body_to_lidar = buffer.lookup_transform("body", "lidar_link", rclpy.time.Time())
                body_to_camera = buffer.lookup_transform("body", "camera_link", rclpy.time.Time())
            except TransformException as exc:
                last_error = str(exc)
                continue
            if not _finite_transform(map_to_odom):
                errors.append("map->odom transform has non-finite values")
                continue
            if not _finite_transform(odom_to_body):
                errors.append("odom->body transform has non-finite values")
                continue
            if not _finite_transform(body_to_lidar):
                errors.append("body->lidar_link transform has non-finite values")
                continue
            if not _finite_transform(body_to_camera):
                errors.append("body->camera_link transform has non-finite values")
                continue
            samples += 1
            time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if samples < min_samples:
        errors.append(
            f"tf chain map->odom->body available samples {samples}/{min_samples}; last_error={last_error}"
        )
    if not odom_state["seen"]:
        errors.append("/slam/odometry was not observed")
    else:
        if odom_state["frame_id"] != "odom":
            errors.append(
                f"/slam/odometry.header.frame_id expected 'odom', got {odom_state['frame_id']!r}"
            )
        if odom_state["child_frame_id"] != "body":
            errors.append(
                f"/slam/odometry.child_frame_id expected 'body', got {odom_state['child_frame_id']!r}"
            )
    if require_sensors:
        expected_frames = {
            "/slam/map_cloud": "odom",
            "/slam/registered_cloud": "body",
        }
        for topic, frame_id in expected_frames.items():
            if topic_samples.get(topic, 0) <= 0:
                errors.append(f"{topic} was not observed")
                continue
            if topic_frames.get(topic) != frame_id:
                errors.append(f"{topic} frame expected {frame_id!r}, got {topic_frames.get(topic)!r}")
        for topic in ("/slam/map_cloud", "/slam/registered_cloud"):
            if point_counts.get(topic, 0) <= 0:
                errors.append(f"{topic} had no points")
    if require_camera:
        expected_frames = {
            "/camera/color/image_raw": "camera_link",
            "/camera/depth/image_raw": "camera_link",
            "/camera/color/camera_info": "camera_link",
        }
        for topic, frame_id in expected_frames.items():
            if topic_samples.get(topic, 0) <= 0:
                errors.append(f"{topic} was not observed")
                continue
            if topic_frames.get(topic) != frame_id:
                errors.append(f"{topic} frame expected {frame_id!r}, got {topic_frames.get(topic)!r}")

    return TfSmokeResult(
        ok=not errors,
        samples=samples,
        odometry_seen=bool(odom_state["seen"]),
        odometry_frame_id=odom_state["frame_id"],
        odometry_child_frame_id=odom_state["child_frame_id"],
        topic_samples=topic_samples,
        topic_frames=topic_frames,
        point_counts=point_counts,
        errors=errors,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--min-samples", type=int, default=3)
    parser.add_argument("--require-sensors", action="store_true")
    parser.add_argument("--require-camera", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()

    try:
        result = run_smoke(args.timeout_sec, args.min_samples, args.require_sensors, args.require_camera)
    except ImportError as exc:
        result = TfSmokeResult(
            ok=False,
            samples=0,
            odometry_seen=False,
            odometry_frame_id=None,
            odometry_child_frame_id=None,
            topic_samples={},
            topic_frames={},
            point_counts={},
            errors=[f"ROS 2 Python dependencies unavailable: {exc}"],
        )

    payload = json.dumps(result.as_dict(), ensure_ascii=False, indent=2)
    if args.json:
        print(payload)
    else:
        status = "PASSED" if result.ok else "FAILED"
        print(f"{status}: LingTu TF contract smoke")
        print(payload)
    if args.json_out:
        from pathlib import Path

        path = Path(args.json_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(payload + "\n", encoding="utf-8")
    return 0 if result.ok else 1


if __name__ == "__main__":
    sys.exit(main())
