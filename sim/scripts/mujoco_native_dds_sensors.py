#!/usr/bin/env python3
"""Publish MuJoCo MID-360/IMU raw sensors to the native DDS SLAM boundary.

This script is a simulator sensor bridge, not SLAM. It never estimates pose,
builds a map, or relocalizes. The expected SLAM/runtime consumer is the native
C++ ``lingtu_slam_cyclone_runtime`` process.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drivers.sim.mujoco.driver import _xyzi_to_livox_frame
from drivers.sim.mujoco.runtime import (
    DEFAULT_MID360_PATTERN,
    DEFAULT_MID360_SAMPLES_PER_FRAME,
    build_engine,
    parse_start,
    resolve_world,
)
from drivers.sim.mujoco.sensors import specific_force_body, world_xyzi_to_sensor_xyzi
from runtime.msgs.geometry import Quaternion, Vector3
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu
from runtime.runtime_interface import TOPICS, topic_default_frame_id


NATIVE_SLAM_RUNTIME = "lingtu_slam_cyclone_runtime"
NATIVE_SENSOR_PUBLISHER = "livox_sdk2_stream --stdin-records --dds"
LIDAR_FRAME_ID = topic_default_frame_id(TOPICS.lidar_scan)
IMU_FRAME_ID = topic_default_frame_id(TOPICS.imu)
_MAGIC = b"LTU1"
_RECORD_CLOUD = 1
_RECORD_IMU = 2
_HEADER = struct.Struct("<4sB3xQIII")
_IMU_PAYLOAD = struct.Struct("<ffffff")
_MID360_ACCEL_MPS2_PER_G = 9.80665
REQUIRED_SLAM_OUTPUT_TOPICS = (
    TOPICS.odometry,
    TOPICS.map_cloud,
    TOPICS.localization_health,
)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="industrial_park")
    parser.add_argument("--start", default="", help="Optional start pose x,y,z")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--publish-hz", type=float, default=10.0, help="LiDAR frame rate.")
    parser.add_argument("--imu-hz", type=float, default=50.0)
    parser.add_argument("--imu-acc-mode", choices=["finite_difference", "gravity_only"], default="finite_difference")
    parser.add_argument("--min-localization-quality", type=float, default=0.5)
    parser.add_argument("--max-odom-abs-m", type=float, default=100.0)
    parser.add_argument("--max-odom-z-abs-m", type=float, default=20.0)
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--publisher-bin", default=os.environ.get("LINGTU_MUJOCO_NATIVE_DDS_PUBLISHER_BIN", ""))
    parser.add_argument("--slam-status-json", default=os.environ.get("LINGTU_SLAM_STATUS_JSON", ""))
    parser.add_argument("--drive-mode", choices=["kinematic", "policy"], default="kinematic")
    parser.add_argument("--drive-vx", type=float, default=0.10)
    parser.add_argument("--drive-vy", type=float, default=0.0)
    parser.add_argument("--drive-wz", type=float, default=0.04)
    parser.add_argument("--policy-path", default=os.environ.get("LINGTU_MUJOCO_NATIVE_DDS_POLICY_PATH", ""))
    parser.add_argument("--n-rays", type=int, default=6400)
    parser.add_argument("--mujoco-memory", default="64M")
    parser.add_argument("--mid360-pattern", type=Path, default=DEFAULT_MID360_PATTERN)
    parser.add_argument("--mid360-samples-per-frame", type=int, default=DEFAULT_MID360_SAMPLES_PER_FRAME)
    parser.add_argument("--lidar-backend", choices=["mujoco_lidar", "ray_caster_lidar"], default="mujoco_lidar")
    parser.add_argument("--mujoco-lidar-backend", choices=["cpu", "taichi", "warp", "jax"], default="cpu")
    parser.add_argument("--allow-legacy-lidar-fallback", action="store_true")
    parser.add_argument("--max-points", type=int, default=15000)
    parser.add_argument(
        "--require-slam-output",
        action="store_true",
        help="Fail unless native C++ SLAM publishes odometry, map cloud, and health.",
    )
    parser.add_argument("--json-out", default="")
    return parser


def _runtime_imu_from_state(
    state: Any,
    ts: float,
    prev_velocity: Any = None,
    dt: float = 0.0,
    acc_mode: str = "finite_difference",
) -> Imu:
    quat = np.asarray(getattr(state, "orientation", (0.0, 0.0, 0.0, 1.0)), dtype=float)
    gyro = np.asarray(getattr(state, "imu_gyro", (0.0, 0.0, 0.0)), dtype=float)
    accel = specific_force_body(state, prev_velocity, dt, mode=acc_mode)
    return Imu(
        orientation=Quaternion(
            float(quat[0]),
            float(quat[1]),
            float(quat[2]),
            float(quat[3]),
        ),
        angular_velocity=Vector3(float(gyro[0]), float(gyro[1]), float(gyro[2])),
        linear_acceleration=Vector3(float(accel[0]), float(accel[1]), float(accel[2])),
        ts=float(ts),
        frame_id=IMU_FRAME_ID,
    )


def _bounded_points(points: Any, max_points: int) -> Any:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[0] <= 0:
        return np.zeros((0, 4), dtype=np.float32)
    if pts.shape[1] == 3:
        intensity = np.full((pts.shape[0], 1), 100.0, dtype=np.float32)
        pts = np.hstack([pts, intensity])
    elif pts.shape[1] > 4:
        pts = pts[:, :4]
    if max_points > 0 and pts.shape[0] > max_points:
        stride = int(np.ceil(pts.shape[0] / float(max_points)))
        pts = pts[::stride][:max_points]
    return pts.astype(np.float32, copy=False)


def _make_report(
    *,
    ok: bool,
    duration_s: float,
    domain_id: int,
    sensor_counts: Counter[str],
    slam_counts: Counter[str],
    require_slam_output: bool,
    remaining_gaps: list[str],
    publisher: str = "",
    slam_status: dict[str, Any] | None = None,
    error: str = "",
) -> dict[str, Any]:
    return {
        "schema_version": "lingtu.mujoco_native_dds_sensors.v1",
        "ok": bool(ok),
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "no_python_slam": True,
        "python_role": "mujoco_sensor_dds_adapter_only",
        "native_sensor_publisher": publisher,
        "localization_runtime_expected": NATIVE_SLAM_RUNTIME,
        "domain_id": int(domain_id),
        "duration_s": float(duration_s),
        "published": dict(sensor_counts),
        "observed_slam_outputs": dict(slam_counts),
        "slam_status": slam_status or {},
        "require_slam_output": bool(require_slam_output),
        "required_slam_output_topics": list(REQUIRED_SLAM_OUTPUT_TOPICS),
        "sensor_topics": {
            TOPICS.lidar_scan: {"dds_topic": "rt/lidar/raw_frame", "frame_id": LIDAR_FRAME_ID},
            TOPICS.imu: {"dds_topic": "rt/imu/raw", "frame_id": IMU_FRAME_ID},
        },
        "remaining_gaps": list(remaining_gaps),
        "error": error,
    }


def _remaining_gaps(
    *,
    sensor_counts: Counter[str],
    slam_counts: Counter[str],
    require_slam_output: bool,
) -> list[str]:
    gaps: list[str] = []
    for topic in (TOPICS.lidar_scan, TOPICS.imu):
        if sensor_counts.get(topic, 0) <= 0:
            gaps.append(f"sensor_not_published:{topic}")
    if require_slam_output:
        for topic in REQUIRED_SLAM_OUTPUT_TOPICS:
            if slam_counts.get(topic, 0) <= 0:
                gaps.append(f"native_slam_output_missing:{topic}")
    return gaps


def _slam_health_gaps(
    status: dict[str, Any],
    *,
    min_quality: float,
    max_odom_abs_m: float,
    max_odom_z_abs_m: float,
) -> list[str]:
    if not status:
        return []
    gaps: list[str] = []
    state = str(status.get("state") or "").upper()
    if state != "TRACKING":
        gaps.append(f"native_slam_not_tracking:{state or 'unknown'}")
    quality = float(status.get("localization_quality") or 0.0)
    if quality < float(min_quality):
        gaps.append(f"native_slam_quality_low:{quality:.3f}")
    pose = ((status.get("odometry") or {}).get("pose") or {})
    coords = {
        axis: float(pose.get(axis) or 0.0)
        for axis in ("x", "y", "z")
    }
    if any(not math.isfinite(value) for value in coords.values()):
        gaps.append("native_slam_odom_not_finite")
    elif any(abs(value) > float(max_odom_abs_m) for value in coords.values()):
        gaps.append(
            "native_slam_odom_out_of_bounds:"
            f"x={coords['x']:.3f},y={coords['y']:.3f},z={coords['z']:.3f}"
        )
    if math.isfinite(coords["z"]) and abs(coords["z"]) > float(max_odom_z_abs_m):
        gaps.append(f"native_slam_odom_z_out_of_bounds:{coords['z']:.3f}")
    return gaps


def _publisher_candidates(value: str) -> list[Path]:
    if value:
        return [Path(value).expanduser()]
    if os.name == "nt":
        return [
            ROOT / "build" / "livox_sdk2_stream" / "Debug" / "livox_sdk2_stream.exe",
            ROOT / "build" / "livox_sdk2_stream" / "Release" / "livox_sdk2_stream.exe",
        ]
    return [
        ROOT / "build" / "livox_sdk2_stream" / "livox_sdk2_stream",
    ]


def _resolve_publisher_bin(value: str) -> Path:
    for candidate in _publisher_candidates(value):
        if candidate.exists():
            return candidate.resolve()
    raise FileNotFoundError(
        "native DDS sensor publisher missing. Build it with: "
        "LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS=ON bash scripts/build/build_livox_sdk2_stream.sh"
    )


def _start_native_publisher(args: argparse.Namespace) -> subprocess.Popen[bytes]:
    publisher = _resolve_publisher_bin(str(args.publisher_bin or ""))
    command = [
        str(publisher),
        "--stdin-records",
        "--dds",
        "--domain-id",
        str(int(args.domain_id)),
        "--lidar-frame",
        LIDAR_FRAME_ID,
        "--imu-frame",
        IMU_FRAME_ID,
    ]
    return subprocess.Popen(
        command,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
    )


def _write_record(stream: Any, record_type: int, timestamp_ns: int, sequence: int, payload: bytes, count: int) -> None:
    stream.write(_HEADER.pack(_MAGIC, int(record_type), int(timestamp_ns), int(sequence), int(count), len(payload)))
    stream.write(payload)


def _write_native_scan(stream: Any, scan: Any) -> None:
    payload = np.asarray(scan.points).tobytes()
    _write_record(
        stream,
        _RECORD_CLOUD,
        int(scan.timestamp_ns),
        int(scan.sequence),
        payload,
        int(scan.point_count),
    )


def _write_native_imu(stream: Any, imu: Imu, sequence: int) -> None:
    acc_scale = _MID360_ACCEL_MPS2_PER_G
    payload = _IMU_PAYLOAD.pack(
        float(imu.angular_velocity.x),
        float(imu.angular_velocity.y),
        float(imu.angular_velocity.z),
        float(imu.linear_acceleration.x) / acc_scale,
        float(imu.linear_acceleration.y) / acc_scale,
        float(imu.linear_acceleration.z) / acc_scale,
    )
    _write_record(stream, _RECORD_IMU, int(float(imu.ts) * 1_000_000_000), sequence, payload, 1)


def _slam_status_counts(path: str) -> tuple[Counter[str], dict[str, Any]]:
    if not path:
        return Counter(), {}
    status_path = Path(path)
    if not status_path.exists():
        return Counter(), {}
    status = json.loads(status_path.read_text(encoding="utf-8"))
    counts: Counter[str] = Counter()
    if bool(status.get("has_odom")):
        counts[TOPICS.odometry] += 1
    if int(status.get("registered_points") or 0) > 0:
        counts[TOPICS.registered_cloud] += 1
    if int(status.get("map_points") or 0) > 0:
        counts[TOPICS.map_cloud] += 1
    if status.get("state"):
        counts[TOPICS.localization_health] += 1
    if float(status.get("localization_quality") or 0.0) > 0.0:
        counts[TOPICS.localization_quality] += 1
    return counts, status


def run(args: argparse.Namespace) -> dict[str, Any]:
    from sim.engine.core.engine import VelocityCommand

    duration_s = max(0.0, float(args.duration))
    lidar_hz = max(0.1, float(args.publish_hz))
    imu_hz = max(lidar_hz, float(args.imu_hz))
    lidar_period_s = 1.0 / lidar_hz
    imu_period_s = 1.0 / imu_hz
    scan_duration_ns = int(1_000_000_000 * lidar_period_s)
    sensor_counts: Counter[str] = Counter()
    slam_counts: Counter[str] = Counter()
    publisher_path = _resolve_publisher_bin(str(args.publisher_bin or ""))
    publisher = _start_native_publisher(args)

    engine = None
    try:
        engine = build_engine(
            world=resolve_world(str(args.world)),
            drive_mode=str(args.drive_mode),
            n_rays=int(args.n_rays),
            start=parse_start(str(args.start or "")),
            mujoco_memory=str(args.mujoco_memory),
            mid360_pattern=args.mid360_pattern,
            mid360_samples_per_frame=int(args.mid360_samples_per_frame),
            lidar_backend=str(args.lidar_backend),
            mujoco_lidar_backend=str(args.mujoco_lidar_backend),
            require_mature_lidar_backend=True,
            allow_legacy_lidar_fallback=bool(args.allow_legacy_lidar_fallback),
            policy_path=str(args.policy_path or "") or None,
        )
        cmd = VelocityCommand(
            linear_x=float(args.drive_vx),
            linear_y=float(args.drive_vy),
            angular_z=float(args.drive_wz),
        )
        deadline = time.monotonic() + duration_s
        sequence = 0
        imu_sequence = 0
        prev_imu_s = None
        prev_velocity = None
        next_lidar_s = 0.0
        while time.monotonic() < deadline:
            loop_start = time.monotonic()
            state = engine.step(cmd)
            ts = time.time()
            imu_dt = 0.0 if prev_imu_s is None else max(0.0, ts - prev_imu_s)
            imu = _runtime_imu_from_state(
                state,
                ts,
                prev_velocity,
                imu_dt,
                str(args.imu_acc_mode),
            )
            if publisher.stdin is None:
                raise RuntimeError("native publisher stdin closed")
            _write_native_imu(publisher.stdin, imu, imu_sequence)
            sensor_counts[TOPICS.imu] += 1
            imu_sequence += 1
            prev_imu_s = ts
            prev_velocity = np.asarray(state.linear_velocity, dtype=np.float64).copy()
            if loop_start >= next_lidar_s:
                world_points = _bounded_points(engine.get_lidar_points(), int(args.max_points))
                sensor_points = world_xyzi_to_sensor_xyzi(engine, world_points)
                scan = _xyzi_to_livox_frame(
                    sensor_points,
                    timestamp_ns=int(ts * 1_000_000_000),
                    sequence=sequence,
                    frame_id=LIDAR_FRAME_ID,
                    scan_duration_ns=scan_duration_ns,
                )
                _write_native_scan(publisher.stdin, scan)
                sensor_counts[TOPICS.lidar_scan] += 1
                sequence += 1
                next_lidar_s = loop_start + lidar_period_s
            publisher.stdin.flush()
            if publisher.poll() is not None:
                raise RuntimeError(f"native DDS sensor publisher exited: {publisher.returncode}")
            time.sleep(max(0.0, imu_period_s - (time.monotonic() - loop_start)))
    finally:
        if publisher.stdin is not None:
            publisher.stdin.close()
        try:
            publisher.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            publisher.terminate()
            publisher.wait(timeout=3.0)
        if engine is not None:
            engine.close()

    status_counts, slam_status = _slam_status_counts(str(args.slam_status_json or ""))
    slam_counts.update(status_counts)
    gaps = _remaining_gaps(
        sensor_counts=sensor_counts,
        slam_counts=slam_counts,
        require_slam_output=bool(args.require_slam_output),
    )
    if args.require_slam_output and not str(args.slam_status_json or ""):
        gaps.append("slam_status_json_not_configured")
    if args.require_slam_output:
        gaps.extend(
            _slam_health_gaps(
                slam_status,
                min_quality=float(args.min_localization_quality),
                max_odom_abs_m=float(args.max_odom_abs_m),
                max_odom_z_abs_m=float(args.max_odom_z_abs_m),
            )
        )
    return _make_report(
        ok=not gaps,
        duration_s=duration_s,
        domain_id=int(args.domain_id),
        sensor_counts=sensor_counts,
        slam_counts=slam_counts,
        require_slam_output=bool(args.require_slam_output),
        remaining_gaps=gaps,
        publisher=str(publisher_path),
        slam_status=slam_status,
    )


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        report = run(args)
    except Exception as exc:
        report = _make_report(
            ok=False,
            duration_s=float(getattr(args, "duration", 0.0) or 0.0),
            domain_id=int(getattr(args, "domain_id", 0) or 0),
            sensor_counts=Counter(),
            slam_counts=Counter(),
            require_slam_output=bool(getattr(args, "require_slam_output", False)),
            remaining_gaps=[f"mujoco_native_dds_sensor_bridge_failed:{type(exc).__name__}"],
            publisher=str(getattr(args, "publisher_bin", "") or NATIVE_SENSOR_PUBLISHER),
            error=str(exc),
        )
    text = json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
