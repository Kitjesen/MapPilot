#!/usr/bin/env python3
"""Publish MuJoCo MID-360/IMU raw sensors to the native DDS SLAM boundary.

This script is a simulator sensor bridge, not SLAM. It never estimates pose,
builds a map, or relocalizes. The optional navigation fixture publishes MuJoCo
ground truth as native DDS navigation inputs so navigation can be accepted
independently from SLAM accuracy and rolling-scan performance.
"""

from __future__ import annotations

import argparse
import errno
import hmac
import json
import math
import os
import queue
import re
import secrets
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Sequence

if TYPE_CHECKING:
    from sim.compat.engine.core.engine import VelocityCommand

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sim.scripts.mujoco import native_sensor_records as _sensor_records  # noqa: E402
from sim.scripts.mujoco.async_jsonl_writer import AsyncJsonlWriter  # noqa: E402

from drivers.sim.mujoco.driver import _xyzi_to_livox_frame  # noqa: E402
from drivers.sim.mujoco.runtime import (  # noqa: E402
    DEFAULT_MID360_PATTERN,
    DEFAULT_MID360_SAMPLES_PER_FRAME,
    build_engine,
    draw_navigation_paths,
    focus_presentation_viewer,
    launch_presentation_viewer,
    parse_start,
    resolve_world,
)
from drivers.sim.mujoco.sensors import (  # noqa: E402
    NAVIGATION_FIXTURE_GROUND_RESOLUTION_M,
    NAVIGATION_FIXTURE_GROUND_Y_HALF_M,
    angle_delta_rad,
    navigation_fixture_registered_body_points,
    projected_gravity_body,
    quat_xyzw_to_matrix,
    specific_force_body,
    world_xyzi_to_body_xyzi,
    world_xyzi_to_sensor_xyzi,
    yaw_from_quat_xyzw,
)
from runtime.msgs.geometry import Quaternion, Vector3  # noqa: E402
from runtime.msgs.numpy_compat import np  # noqa: E402
from runtime.msgs.sensor import Imu  # noqa: E402
from runtime.runtime_interface import TOPICS, topic_default_frame_id  # noqa: E402

NATIVE_SLAM_RUNTIME = "slamd"
NATIVE_SENSOR_PUBLISHER = "lingtu_mujoco_sensor_publisher --stdin-records --dds"
LIDAR_FRAME_ID = topic_default_frame_id(TOPICS.lidar_scan)
IMU_FRAME_ID = topic_default_frame_id(TOPICS.imu)
_MAGIC = _sensor_records.LTU1_MAGIC
_RECORD_CLOUD = _sensor_records.RECORD_CLOUD
_RECORD_IMU = _sensor_records.RECORD_IMU
_RECORD_ODOM_PRIOR = _sensor_records.RECORD_ODOM_PRIOR
_RECORD_REGISTERED_CLOUD = _sensor_records.RECORD_REGISTERED_CLOUD
_HEADER = _sensor_records.HEADER
_IMU_PAYLOAD = _sensor_records.IMU_PAYLOAD
_ODOM_PRIOR_PAYLOAD = _sensor_records.ODOM_PRIOR_PAYLOAD
_MID360_ACCEL_MPS2_PER_G = _sensor_records.MID360_ACCEL_MPS2_PER_G
POINT_DTYPE = _sensor_records.POINT_DTYPE
KINEMATIC_LEGACY_IMU_ACC_AXIS_SCALE = (-0.43, 1.0, 1.0)
# MuJoCo kinematic drive sets base velocity directly. Its finite-difference X
# acceleration is a control artifact, not a physical IMU force for Fast-LIO.
KINEMATIC_SIM_HARDWARE_IMU_ACC_AXIS_SCALE = (0.0, 1.0, 1.0)
_THUNDERV4_POLICY_DIR = (
    ROOT / "sim" / "packages" / "controllers" / "doso" / "thunder_v4" / "locomotion" / "policy"
)
DEFAULT_THUNDERV4_ONNX_POLICY = _THUNDERV4_POLICY_DIR / "policy_1119.onnx"
REQUIRED_SLAM_OUTPUT_TOPICS = (
    TOPICS.odometry,
    TOPICS.map_cloud,
    TOPICS.localization_health,
)
SIM_HARDWARE_CLOCK = "sim_hardware"
DEFAULT_IMU_ACC_CONDITIONING = "realistic"
# MuJoCo contact impulses are substantially sharper than the mechanically
# filtered signal observed by a lidar-mounted IMU. These values retain body
# acceleration while rejecting solver impulses that destabilize deskew/LIO.
DEFAULT_IMU_ACC_LOWPASS_HZ = 5.0
DEFAULT_IMU_ACC_MAX_DYNAMIC_MPS2 = 1.5
DEFAULT_IMU_ACC_MAX_SLEW_MPS3 = 30.0
DEFAULT_SIM_HARDWARE_MAX_LAG_S = 0.05
DEFAULT_SIM_HARDWARE_CATCH_UP_YIELD_STEPS = 40
DEFAULT_NATIVE_CLOCK_SYNC_SAMPLES = 5
WSL_CONTROL_TIMEOUT_S = 2.0
MAX_NATIVE_CLOCK_SYNC_RTT_S = 0.10
DEFAULT_ODOM_PRIOR_VELOCITY_WINDOW_S = 0.10
DEFAULT_PARENT_DIAGNOSTICS_PERIOD_S = 0.5
DEFAULT_PUBLISHER_WRITE_MODE = "sync"
DEFAULT_ASYNC_PUBLISHER_MAX_BYTES = 1_048_576
DEFAULT_ASYNC_PUBLISHER_MAX_RECORDS = 512
DEFAULT_ASYNC_PUBLISHER_MAX_BATCHES = 256
DEFAULT_ASYNC_PUBLISHER_OLDEST_S = 0.5
DEFAULT_ASYNC_PUBLISHER_SHUTDOWN_S = 2.0
DEFAULT_NATIVE_PUBLISHER_READY_S = 10.0
DEFAULT_DRIVER_BRIDGE_TRANSPORT_READY_S = 10.0
NATIVE_SENSOR_PUBLISHER_READY_SCHEMA = "lingtu.mujoco_sensor_publisher.ready.v1"
PARENT_DIAGNOSTICS_SCHEMA = "lingtu.mujoco.parent_sensor_diagnostics.v1"
_PARENT_DIAGNOSTIC_RECORD_TYPES = (
    "cloud",
    "imu",
    "odom_prior",
    "registered_cloud",
)
_PARENT_DIAGNOSTIC_HISTOGRAM_BOUNDS_US = (
    10,
    25,
    50,
    100,
    250,
    500,
    1_000,
    2_500,
    5_000,
    10_000,
    25_000,
    50_000,
    100_000,
    250_000,
    500_000,
    1_000_000,
)
EXTERNAL_ARM_SCHEMA = "lingtu.mujoco.external_arm.v1"
EXTERNAL_ARM_STATUS_SCHEMA = "lingtu.mujoco.external_arm_status.v1"
DEFAULT_EXTERNAL_ARM_TIMEOUT_S = 60.0
_EXTERNAL_ARM_MAX_BYTES = 4096
_EXTERNAL_ARM_STATUS_PERIOD_S = 0.25


def _relative_times_for_scan(
    point_count: int,
    lidar_period_s: float,
    *,
    scan_time_profile: str,
) -> np.ndarray:
    """Return per-point times matching how this validation scan was produced."""

    count = max(0, int(point_count))
    profile = str(scan_time_profile or "synthetic_rolling").strip().lower()
    if profile == "instantaneous":
        return np.zeros(count, dtype=np.float32)
    if profile == "synthetic_rolling":
        return np.linspace(
            0.0,
            float(lidar_period_s),
            num=count,
            endpoint=False,
            dtype=np.float32,
        )
    raise ValueError(f"unsupported scan_time_profile: {scan_time_profile}")


def _physical_rolling_scan_from_samples(
    samples: Sequence[tuple[float, np.ndarray, np.ndarray, int]],
    *,
    scan_start_s: float,
    scan_end_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, int]:
    """Build one scan from subscans captured at their actual simulation times."""

    start = float(scan_start_s)
    end = float(scan_end_s)
    eps = 1e-6
    sensor_chunks: list[np.ndarray] = []
    world_chunks: list[np.ndarray] = []
    time_chunks: list[np.ndarray] = []
    moving_point_count = 0
    selected_subscans = 0
    for sim_time_s, cloud_sensor, cloud_world, moving_count in samples:
        t = float(sim_time_s)
        if t < start - eps or t > end + eps:
            continue
        sensor_pts = np.asarray(cloud_sensor, dtype=np.float32)
        world_pts = np.asarray(cloud_world, dtype=np.float32)
        if sensor_pts.size == 0 or sensor_pts.ndim != 2 or sensor_pts.shape[1] < 4:
            continue
        if world_pts.size == 0 or world_pts.ndim != 2 or world_pts.shape[1] < 4:
            world_pts = np.zeros((len(sensor_pts), 4), dtype=np.float32)
        sensor_chunks.append(sensor_pts[:, :4])
        world_chunks.append(world_pts[:, :4])
        relative_t = max(0.0, min(float(end - start), t - start))
        time_chunks.append(np.full(len(sensor_pts), relative_t, dtype=np.float32))
        moving_point_count += max(0, int(moving_count))
        selected_subscans += 1
    if not sensor_chunks:
        return (
            np.zeros((0, 4), dtype=np.float32),
            np.zeros((0, 4), dtype=np.float32),
            np.zeros(0, dtype=np.float32),
            0,
            0,
        )
    return (
        np.vstack(sensor_chunks).astype(np.float32, copy=False),
        np.vstack(world_chunks).astype(np.float32, copy=False),
        np.concatenate(time_chunks).astype(np.float32, copy=False),
        int(moving_point_count),
        int(selected_subscans),
    )


def _dds_domain_id(value: str) -> int:
    try:
        domain_id = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("DDS domain id must be an integer") from exc
    if not 0 <= domain_id <= 232:
        raise argparse.ArgumentTypeError("DDS domain id must be between 0 and 232")
    return domain_id


def _write_atomic_json_object(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{threading.get_ident()}.{time.monotonic_ns()}.tmp")
    try:
        temporary.write_text(
            json.dumps(payload, allow_nan=False, ensure_ascii=True, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        for attempt in range(25):
            try:
                os.replace(temporary, path)
                break
            except OSError as exc:
                transient = (
                    isinstance(exc, PermissionError)
                    or exc.errno in {errno.EACCES, errno.EBUSY, errno.EPERM}
                    or getattr(exc, "winerror", None) in {5, 32, 33}
                )
                if not transient or attempt == 24:
                    raise
                time.sleep(0.01)
    finally:
        temporary.unlink(missing_ok=True)


class _FixedMicrosecondHistogram:
    def __init__(self) -> None:
        self._counts = [0] * (len(_PARENT_DIAGNOSTIC_HISTOGRAM_BOUNDS_US) + 1)
        self._count = 0
        self._total_us = 0.0
        self._max_us = 0.0

    def observe(self, duration_us: float) -> None:
        value = max(0.0, float(duration_us))
        bucket = len(_PARENT_DIAGNOSTIC_HISTOGRAM_BOUNDS_US)
        for index, upper_bound in enumerate(_PARENT_DIAGNOSTIC_HISTOGRAM_BOUNDS_US):
            if value <= upper_bound:
                bucket = index
                break
        self._counts[bucket] += 1
        self._count += 1
        self._total_us += value
        self._max_us = max(self._max_us, value)

    def snapshot(self) -> dict[str, Any]:
        return {
            "bounds_us": list(_PARENT_DIAGNOSTIC_HISTOGRAM_BOUNDS_US),
            "counts": list(self._counts),
            "count": self._count,
            "total_us": self._total_us,
            "max_us": self._max_us,
        }


class ParentSensorDiagnostics:
    """Bounded, opt-in parent-side sensor scheduling diagnostics."""

    def __init__(self, path: Path, *, period_s: float = DEFAULT_PARENT_DIAGNOSTICS_PERIOD_S) -> None:
        period = float(period_s)
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("--parent-diagnostics-period-s must be positive and finite")
        self.path = Path(path).expanduser().resolve()
        self.period_s = period
        self._lock = threading.RLock()
        self.started_wall_time_ns = time.time_ns()
        self.started_monotonic_ns = time.monotonic_ns()
        self._next_publish_monotonic_ns = self.started_monotonic_ns + int(period * 1_000_000_000)
        self._sequence = 0
        self._diagnostic_write_failures = 0
        self.last_published_reason = ""
        self.stop_requested = False
        self.final_reason = "final"
        self._previous_signal_handlers: dict[int, Any] = {}
        self._record_types: dict[str, dict[str, Any]] = {
            name: {
                "scheduled": 0,
                "catchup_dropped_before_generation": 0,
                "generated": 0,
                "enqueue_attempt": 0,
                "enqueue_success": 0,
                "enqueue_error": 0,
                "enqueue_full": 0,
                "enqueue_bytes": 0,
                "enqueue_duration_us": _FixedMicrosecondHistogram(),
                "pipe_write_attempt": 0,
                "pipe_write_success": 0,
                "pipe_write_error": 0,
                "payload_bytes": 0,
                "pipe_bytes": 0,
                "pipe_write_duration_us": _FixedMicrosecondHistogram(),
            }
            for name in _PARENT_DIAGNOSTIC_RECORD_TYPES
        }
        self._pending_flush = {name: {"records": 0, "bytes": 0} for name in _PARENT_DIAGNOSTIC_RECORD_TYPES}
        self._flush = {
            "attempt": 0,
            "success": 0,
            "error": 0,
            "duration_us": _FixedMicrosecondHistogram(),
            "records_since_flush": {"last": 0, "total": 0, "max": 0},
            "bytes_since_flush": {"last": 0, "total": 0, "max": 0},
            "record_types": {
                name: {
                    "attempt": 0,
                    "success": 0,
                    "error": 0,
                    "duration_us": _FixedMicrosecondHistogram(),
                    "records_since_flush": {"last": 0, "total": 0, "max": 0},
                    "bytes_since_flush": {"last": 0, "total": 0, "max": 0},
                }
                for name in _PARENT_DIAGNOSTIC_RECORD_TYPES
            },
        }
        self._deadline_skip = {name: 0 for name in _PARENT_DIAGNOSTIC_RECORD_TYPES}
        self._scheduler_pacing: dict[str, Any] = {
            "final_lag_s": 0.0,
            "max_lag_observed_s": 0.0,
            "max_consecutive_steps": 0,
            "catch_up_events": 0,
            "catch_up_yields": 0,
        }
        self._async_queue: dict[str, Any] = {
            "enabled": False,
            "limit_bytes": 0,
            "limit_records": 0,
            "limit_batches": 0,
            "oldest_limit_s": 0.0,
            "current_bytes": 0,
            "current_records": 0,
            "current_batches": 0,
            "max_bytes": 0,
            "max_records": 0,
            "max_batches": 0,
            "oldest_age_s": 0.0,
            "enqueued_batch_sequence": 0,
            "enqueued_record_sequence": 0,
            "written_batch_sequence": 0,
            "written_record_sequence": 0,
            "batch_sequence_lag": 0,
            "record_sequence_lag": 0,
            "undrained_bytes": 0,
            "undrained_records": 0,
            "undrained_batches": 0,
            "writer_alive": False,
            "cleanup_reason": "sync",
            "fatal_reason": "",
        }
        self.force_publish("startup")

    def handle_stop_signal(self, signum: int, _frame: Any) -> None:
        self.stop_requested = True
        if signum == signal.SIGTERM:
            self.final_reason = "signal:SIGTERM"
        elif signum == signal.SIGINT:
            self.final_reason = "signal:SIGINT"
        else:
            self.final_reason = "signal:unknown"

    def install_signal_handlers(self) -> None:
        for signum in (signal.SIGTERM, signal.SIGINT):
            self._previous_signal_handlers[int(signum)] = signal.getsignal(signum)
            signal.signal(signum, self.handle_stop_signal)

    def restore_signal_handlers(self) -> None:
        for signum, handler in self._previous_signal_handlers.items():
            signal.signal(signum, handler)
        self._previous_signal_handlers.clear()

    def _record(self, record_type: str) -> dict[str, Any]:
        try:
            return self._record_types[str(record_type)]
        except KeyError as exc:
            raise ValueError(f"unknown parent diagnostic record type: {record_type}") from exc

    def record_scheduled(self, record_type: str, count: int = 1) -> None:
        with self._lock:
            self._record(record_type)["scheduled"] += max(0, int(count))

    def record_catchup_drop(self, record_type: str, count: int = 1) -> None:
        with self._lock:
            self._record(record_type)["catchup_dropped_before_generation"] += max(0, int(count))

    def record_generated(self, record_type: str, count: int = 1) -> None:
        with self._lock:
            self._record(record_type)["generated"] += max(0, int(count))

    def record_enqueue_result(
        self,
        records: tuple[_SerializedPublisherRecord, ...],
        *,
        success: bool,
        full: bool,
        duration_us: float,
        queue_stats: dict[str, Any] | None = None,
    ) -> None:
        with self._lock:
            for record in records:
                counters = self._record(record.diagnostic_record_type)
                counters["enqueue_attempt"] += 1
                counters["enqueue_success" if success else "enqueue_error"] += 1
                if full:
                    counters["enqueue_full"] += 1
                if success:
                    counters["enqueue_bytes"] += record.wire_bytes
                counters["enqueue_duration_us"].observe(duration_us)
            if queue_stats is not None:
                for key in self._async_queue:
                    if key in queue_stats:
                        self._async_queue[key] = queue_stats[key]

    def configure_async_queue(
        self,
        *,
        max_bytes: int,
        max_records: int,
        max_batches: int,
        oldest_s: float,
    ) -> None:
        with self._lock:
            self._async_queue.update(
                {
                    "enabled": True,
                    "limit_bytes": int(max_bytes),
                    "limit_records": int(max_records),
                    "limit_batches": int(max_batches),
                    "oldest_limit_s": float(oldest_s),
                    "cleanup_reason": "running",
                }
            )

    def update_async_queue(self, stats: dict[str, Any]) -> None:
        with self._lock:
            for key in self._async_queue:
                if key in stats:
                    self._async_queue[key] = stats[key]

    def record_pipe_write_attempt(self, record_type: str) -> None:
        with self._lock:
            self._record(record_type)["pipe_write_attempt"] += 1

    def record_pipe_write_success(
        self,
        record_type: str,
        *,
        payload_bytes: int,
        pipe_bytes: int,
        duration_us: float,
    ) -> None:
        with self._lock:
            counters = self._record(record_type)
            counters["pipe_write_success"] += 1
            counters["payload_bytes"] += max(0, int(payload_bytes))
            counters["pipe_bytes"] += max(0, int(pipe_bytes))
            counters["pipe_write_duration_us"].observe(duration_us)
            pending = self._pending_flush[record_type]
            pending["records"] += 1
            pending["bytes"] += max(0, int(pipe_bytes))

    def record_pipe_write_error(self, record_type: str, *, duration_us: float) -> None:
        with self._lock:
            counters = self._record(record_type)
            counters["pipe_write_error"] += 1
            counters["pipe_write_duration_us"].observe(duration_us)

    def record_deadline_skip(self, record_type: str, count: int = 1) -> None:
        with self._lock:
            self._record(record_type)
            self._deadline_skip[record_type] += max(0, int(count))

    def update_scheduler_pacing(self, pacing: dict[str, Any]) -> None:
        with self._lock:
            self._scheduler_pacing = {
                "final_lag_s": float(pacing.get("final_lag_s") or 0.0),
                "max_lag_observed_s": float(pacing.get("max_lag_observed_s") or 0.0),
                "max_consecutive_steps": int(pacing.get("max_consecutive_steps") or 0),
                "catch_up_events": int(pacing.get("catch_up_events") or 0),
                "catch_up_yields": int(pacing.get("catch_up_yields") or 0),
                "forced_sensor_observations": int(
                    pacing.get("forced_sensor_observations") or 0
                ),
                "forced_lidar_observations": int(
                    pacing.get("forced_lidar_observations") or 0
                ),
            }

    @staticmethod
    def _update_since_flush_metric(metric: dict[str, int], value: int, *, committed: bool) -> None:
        amount = max(0, int(value))
        metric["last"] = amount
        metric["max"] = max(metric["max"], amount)
        if committed:
            metric["total"] += amount

    def record_flush(self, *, success: bool, duration_us: float) -> None:
        with self._lock:
            records = sum(item["records"] for item in self._pending_flush.values())
            pipe_bytes = sum(item["bytes"] for item in self._pending_flush.values())
            self._flush["attempt"] += 1
            self._flush["success" if success else "error"] += 1
            self._flush["duration_us"].observe(duration_us)
            self._update_since_flush_metric(
                self._flush["records_since_flush"],
                records,
                committed=success,
            )
            self._update_since_flush_metric(
                self._flush["bytes_since_flush"],
                pipe_bytes,
                committed=success,
            )
            for name, pending in self._pending_flush.items():
                if pending["records"] <= 0:
                    continue
                counters = self._flush["record_types"][name]
                counters["attempt"] += 1
                counters["success" if success else "error"] += 1
                counters["duration_us"].observe(duration_us)
                self._update_since_flush_metric(
                    counters["records_since_flush"],
                    pending["records"],
                    committed=success,
                )
                self._update_since_flush_metric(
                    counters["bytes_since_flush"],
                    pending["bytes"],
                    committed=success,
                )
            if success:
                for pending in self._pending_flush.values():
                    pending["records"] = 0
                    pending["bytes"] = 0

    def snapshot(self, reason: str) -> dict[str, Any]:
        with self._lock:
            return self._snapshot_unlocked(reason)

    def _snapshot_unlocked(self, reason: str) -> dict[str, Any]:
        pending_records = sum(item["records"] for item in self._pending_flush.values())
        pending_bytes = sum(item["bytes"] for item in self._pending_flush.values())
        return {
            "schema_version": PARENT_DIAGNOSTICS_SCHEMA,
            "sequence": self._sequence + 1,
            "reason": str(reason),
            "period_s": self.period_s,
            "started_wall_time_ns": self.started_wall_time_ns,
            "updated_wall_time_ns": time.time_ns(),
            "diagnostic_write_failures": self._diagnostic_write_failures,
            "record_types": {
                name: {
                    **{
                        key: value
                        for key, value in counters.items()
                        if key not in {"enqueue_duration_us", "pipe_write_duration_us"}
                    },
                    "enqueue_duration_us": counters["enqueue_duration_us"].snapshot(),
                    "pipe_write_duration_us": counters["pipe_write_duration_us"].snapshot(),
                }
                for name, counters in self._record_types.items()
            },
            "flush": {
                "attempt": self._flush["attempt"],
                "success": self._flush["success"],
                "error": self._flush["error"],
                "duration_us": self._flush["duration_us"].snapshot(),
                "records_since_flush": {
                    **self._flush["records_since_flush"],
                    "current": pending_records,
                },
                "bytes_since_flush": {
                    **self._flush["bytes_since_flush"],
                    "current": pending_bytes,
                },
                "record_types": {
                    name: {
                        "attempt": counters["attempt"],
                        "success": counters["success"],
                        "error": counters["error"],
                        "duration_us": counters["duration_us"].snapshot(),
                        "records_since_flush": {
                            **counters["records_since_flush"],
                            "current": self._pending_flush[name]["records"],
                        },
                        "bytes_since_flush": {
                            **counters["bytes_since_flush"],
                            "current": self._pending_flush[name]["bytes"],
                        },
                    }
                    for name, counters in self._flush["record_types"].items()
                },
            },
            "scheduler": {
                "deadline_skip": dict(self._deadline_skip),
                "pacing": dict(self._scheduler_pacing),
            },
            "async_queue": dict(self._async_queue),
        }

    def maybe_publish(self) -> bool:
        with self._lock:
            if time.monotonic_ns() < self._next_publish_monotonic_ns:
                return False
            return self.force_publish("periodic")

    def publish_due(self) -> bool:
        with self._lock:
            return time.monotonic_ns() >= self._next_publish_monotonic_ns

    def force_publish(self, reason: str) -> bool:
        with self._lock:
            try:
                _write_atomic_json_object(self.path, self._snapshot_unlocked(reason))
            except (OSError, TypeError, ValueError):
                self._diagnostic_write_failures += 1
                return False
            self._sequence += 1
            self.last_published_reason = str(reason)
            self._next_publish_monotonic_ns = time.monotonic_ns() + int(self.period_s * 1_000_000_000)
            return True


def _strict_json_object(raw: bytes) -> dict[str, Any]:
    if not raw or len(raw) > _EXTERNAL_ARM_MAX_BYTES:
        raise ValueError("external_arm_json_size_invalid")

    def reject_constant(_: str) -> None:
        raise ValueError("external_arm_json_constant_invalid")

    def reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        value: dict[str, Any] = {}
        for key, item in pairs:
            if key in value:
                raise ValueError("external_arm_json_duplicate_key")
            value[key] = item
        return value

    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=reject_duplicate_keys,
            parse_constant=reject_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("external_arm_json_invalid") from exc
    if not isinstance(value, dict):
        raise ValueError("external_arm_json_object_required")
    return value


def _validate_external_arm_payload(
    payload: dict[str, Any],
    *,
    expected_token: str,
    expected_domain_id: int,
    expected_scenario: str,
) -> str:
    expected_keys = {"schema", "arm", "token", "domain_id", "scenario"}
    if set(payload) != expected_keys:
        return "external_arm_keys_invalid"
    if payload.get("schema") != EXTERNAL_ARM_SCHEMA:
        return "external_arm_schema_invalid"
    if payload.get("arm") is not True:
        return "external_arm_value_invalid"
    token = payload.get("token")
    if not isinstance(token, str) or not hmac.compare_digest(token, expected_token):
        return "external_arm_token_mismatch"
    domain_id = payload.get("domain_id")
    if type(domain_id) is not int or domain_id != expected_domain_id:
        return "external_arm_domain_mismatch"
    scenario = payload.get("scenario")
    if not isinstance(scenario, str) or scenario != expected_scenario:
        return "external_arm_scenario_mismatch"
    return ""


def _external_arm_config_from_args(args: argparse.Namespace) -> dict[str, Any] | None:
    arm_file_value = str(getattr(args, "external_arm_file", "") or "").strip()
    token = str(getattr(args, "external_arm_token", "") or "")
    scenario = str(getattr(args, "external_arm_scenario", "") or "").strip()
    status_value = str(getattr(args, "external_arm_status_json", "") or "").strip()
    if not arm_file_value:
        if token or scenario or status_value:
            raise ValueError("--external-arm-file is required when external-arm options are set")
        return None
    if not token or len(token) > 256:
        raise ValueError("--external-arm-token must contain 1..256 characters")
    if not scenario or len(scenario) > 128:
        raise ValueError("--external-arm-scenario must contain 1..128 characters")
    timeout_s = float(getattr(args, "external_arm_timeout_s", DEFAULT_EXTERNAL_ARM_TIMEOUT_S))
    if not math.isfinite(timeout_s) or timeout_s <= 0.0:
        raise ValueError("--external-arm-timeout-s must be finite and positive")
    arm_file = Path(arm_file_value).expanduser().resolve()
    domain_id = int(args.domain_id)
    if not 0 <= domain_id <= 232:
        raise ValueError("external-arm DDS domain must be in [0, 232]")
    status_json = Path(status_value).expanduser().resolve() if status_value else None
    if status_json == arm_file:
        raise ValueError("external arm and status paths must be distinct")
    return {
        "arm_file": arm_file,
        "token": token,
        "domain_id": domain_id,
        "scenario": scenario,
        "timeout_s": timeout_s,
        "status_json": status_json,
    }


def _prepare_external_arm_files(config: dict[str, Any]) -> None:
    """Remove only the two exact per-run rendezvous artifacts before startup."""

    for key in ("arm_file", "status_json"):
        path = config.get(key)
        if path is not None:
            Path(path).unlink(missing_ok=True)


class ExternalArmGate:
    """Fail-closed, single-transition rendezvous for externally started motion."""

    def __init__(
        self,
        *,
        arm_file: Path,
        token: str,
        domain_id: int,
        scenario: str,
        timeout_s: float,
        status_json: Path | None = None,
        started_wall_s: float | None = None,
    ) -> None:
        self.arm_file = arm_file.resolve()
        self.expected_token = token
        self.domain_id = int(domain_id)
        self.scenario = scenario
        self.timeout_s = float(timeout_s)
        self.status_json = status_json.resolve() if status_json is not None else None
        self.started_wall_s = time.monotonic() if started_wall_s is None else float(started_wall_s)
        self.state = "waiting"
        self.last_error = ""
        self.arm_observed_sim_time_s: float | None = None
        self._wait_elapsed_wall_s = 0.0
        self._observations = 0
        self._read_failures = 0
        self._status_write_failures = 0
        self._next_status_write_wall_s = self.started_wall_s
        if not self._publish_status(self.started_wall_s, force=True):
            self.state = "invalid"
            self.last_error = "external_arm_status_write_failed"

    @property
    def acknowledged(self) -> bool:
        return self.state == "armed"

    @property
    def failed(self) -> bool:
        return self.state in {"invalid", "timed_out"}

    @property
    def failure_gap(self) -> str:
        if self.state == "timed_out":
            return "external_arm_timeout"
        if self.state == "invalid":
            return self.last_error or "external_arm_invalid"
        return ""

    def _elapsed(self, monotonic_now_s: float) -> float:
        if self.state == "waiting":
            return max(0.0, float(monotonic_now_s) - self.started_wall_s)
        return self._wait_elapsed_wall_s

    def snapshot(self, *, monotonic_now_s: float | None = None) -> dict[str, Any]:
        now = time.monotonic() if monotonic_now_s is None else float(monotonic_now_s)
        return {
            "schema": EXTERNAL_ARM_STATUS_SCHEMA,
            "enabled": True,
            "state": self.state,
            "acknowledged": self.acknowledged,
            "domain_id": self.domain_id,
            "duration_clock": "sim",
            "anchored_before_arm": True,
            "sensor_publication_before_arm": True,
            "scenario": self.scenario,
            "timeout_s": self.timeout_s,
            "wait_elapsed_wall_s": min(self.timeout_s, self._elapsed(now)),
            "arm_observed_sim_time_s": self.arm_observed_sim_time_s,
            "observations": self._observations,
            "read_failures": self._read_failures,
            "status_write_failures": self._status_write_failures,
            "last_error": self.last_error[:160],
        }

    def _publish_status(self, monotonic_now_s: float, *, force: bool = False) -> bool:
        if self.status_json is None:
            return True
        if not force and monotonic_now_s + 1e-9 < self._next_status_write_wall_s:
            return True
        try:
            _write_atomic_json_object(
                self.status_json,
                self.snapshot(monotonic_now_s=monotonic_now_s),
            )
        except (OSError, ValueError):
            self._status_write_failures += 1
            return False
        self._next_status_write_wall_s = monotonic_now_s + _EXTERNAL_ARM_STATUS_PERIOD_S
        return True

    def _transition(
        self,
        state: str,
        *,
        monotonic_now_s: float,
        sim_time_s: float | None = None,
        error: str = "",
    ) -> None:
        self.state = state
        self.last_error = error
        self._wait_elapsed_wall_s = max(0.0, monotonic_now_s - self.started_wall_s)
        if state == "armed":
            self.arm_observed_sim_time_s = float(sim_time_s) if sim_time_s is not None else None
        if not self._publish_status(monotonic_now_s, force=True) and state == "armed":
            self.state = "invalid"
            self.last_error = "external_arm_status_write_failed"

    def poll(self, *, sim_time_s: float, monotonic_now_s: float | None = None) -> str:
        now = time.monotonic() if monotonic_now_s is None else float(monotonic_now_s)
        if self.state != "waiting":
            return self.state
        self._observations += 1
        if now - self.started_wall_s >= self.timeout_s:
            self._transition(
                "timed_out",
                monotonic_now_s=now,
                error="external_arm_timeout",
            )
            return self.state
        if not self.arm_file.is_file():
            self._publish_status(now)
            return self.state
        try:
            payload = _strict_json_object(self.arm_file.read_bytes())
        except OSError:
            self._read_failures += 1
            self.last_error = "external_arm_read_failed"
            self._publish_status(now)
            return self.state
        except ValueError as exc:
            error = str(exc)
            self._transition("invalid", monotonic_now_s=now, error=error)
            return self.state
        error = _validate_external_arm_payload(
            payload,
            expected_token=self.expected_token,
            expected_domain_id=self.domain_id,
            expected_scenario=self.scenario,
        )
        if error:
            self._transition("invalid", monotonic_now_s=now, error=error)
            return self.state
        self._transition(
            "armed",
            monotonic_now_s=now,
            sim_time_s=sim_time_s,
        )
        return self.state


def _external_arm_disabled_report() -> dict[str, Any]:
    return {
        "schema": EXTERNAL_ARM_STATUS_SCHEMA,
        "enabled": False,
        "state": "disabled",
        "acknowledged": False,
    }


def _write_motion_complete_marker(
    marker_path: str | Path,
    *,
    sim_time_s: float,
    goal_reached: bool,
) -> None:
    marker = Path(marker_path).expanduser().resolve()
    marker.parent.mkdir(parents=True, exist_ok=True)
    temporary = marker.with_name(f".{marker.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(
            {
                "complete": True,
                "goal_reached": bool(goal_reached),
                "sim_time_s": float(sim_time_s),
            },
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, marker)


class OdomPriorVelocityEstimator:
    """Estimate map-frame velocity from a short pose window.

    MuJoCo generalized velocity can contain solver-scale contact impulses that
    are not present in the base pose trajectory.  The simulation-only odometry
    prior therefore uses the same observable quantity as odometry: position
    change over time.  Values are intentionally not clipped so real pose jumps
    still exercise the native SLAM prior safety gates.
    """

    def __init__(self, *, window_s: float = DEFAULT_ODOM_PRIOR_VELOCITY_WINDOW_S) -> None:
        self.window_s = max(0.02, float(window_s))
        self._samples: list[tuple[float, np.ndarray]] = []
        self._last_velocity = np.zeros(3, dtype=np.float64)
        self._last_valid = False
        self._max_speed_mps = 0.0
        self._max_velocity = np.zeros(3, dtype=np.float64)
        self._max_velocity_stamp_s = 0.0
        self._max_xy_speed_mps = 0.0
        self._max_xy_velocity = np.zeros(3, dtype=np.float64)
        self._max_xy_velocity_stamp_s = 0.0

    def update(self, position: Any, timestamp_s: float) -> tuple[np.ndarray, bool]:
        stamp = float(timestamp_s)
        pose = np.asarray(position, dtype=np.float64).reshape(-1)
        if pose.size < 3 or not math.isfinite(stamp) or not np.all(np.isfinite(pose[:3])):
            self._samples.clear()
            self._last_velocity.fill(0.0)
            self._last_valid = False
            return np.zeros(3, dtype=np.float64), False

        if self._samples and stamp <= self._samples[-1][0]:
            self._samples.clear()
        self._samples.append((stamp, pose[:3].copy()))

        cutoff = stamp - self.window_s
        first_inside = next(
            (index for index, (sample_stamp, _) in enumerate(self._samples) if sample_stamp >= cutoff),
            len(self._samples) - 1,
        )
        keep_from = max(0, first_inside - 1)
        if keep_from:
            self._samples = self._samples[keep_from:]

        first_stamp, first_pose = self._samples[0]
        span_s = stamp - first_stamp
        min_span_s = max(0.02, 0.5 * self.window_s)
        if span_s + 1e-12 < min_span_s:
            self._last_velocity.fill(0.0)
            self._last_valid = False
            return np.zeros(3, dtype=np.float64), False

        # Use robust centres of the older and newer half-windows. A MuJoCo
        # contact solve can displace the reported base pose for one 5 ms tick;
        # endpoint differencing turns that single sample into a false multi-m/s
        # velocity. Medians reject that impulse while a persistent teleport or
        # genuinely fast trajectory still moves the newer half-window centre.
        split = max(1, len(self._samples) // 2)
        older = self._samples[:split]
        newer = self._samples[split:]
        if not newer:
            self._last_velocity.fill(0.0)
            self._last_valid = False
            return np.zeros(3, dtype=np.float64), False
        older_stamp = float(np.median([sample_stamp for sample_stamp, _ in older]))
        newer_stamp = float(np.median([sample_stamp for sample_stamp, _ in newer]))
        robust_span_s = newer_stamp - older_stamp
        if robust_span_s <= 1e-12:
            self._last_velocity.fill(0.0)
            self._last_valid = False
            return np.zeros(3, dtype=np.float64), False
        older_pose = np.median(np.stack([sample_pose for _, sample_pose in older]), axis=0)
        newer_pose = np.median(np.stack([sample_pose for _, sample_pose in newer]), axis=0)
        velocity = (newer_pose - older_pose) / robust_span_s
        self._last_velocity = velocity.astype(np.float64, copy=True)
        self._last_valid = True
        speed_mps = float(np.linalg.norm(velocity))
        if speed_mps > self._max_speed_mps:
            self._max_speed_mps = speed_mps
            self._max_velocity = self._last_velocity.copy()
            self._max_velocity_stamp_s = stamp
        xy_speed_mps = float(np.linalg.norm(velocity[:2]))
        if xy_speed_mps > self._max_xy_speed_mps:
            self._max_xy_speed_mps = xy_speed_mps
            self._max_xy_velocity = self._last_velocity.copy()
            self._max_xy_velocity_stamp_s = stamp
        return self._last_velocity.copy(), True

    def stats(self) -> dict[str, Any]:
        return {
            "source": "robust_pose_window",
            "window_s": self.window_s,
            "sample_count": len(self._samples),
            "valid": self._last_valid,
            "velocity_mps": self._last_velocity.astype(float).tolist(),
            "max_speed_mps": self._max_speed_mps,
            "max_velocity_mps": self._max_velocity.astype(float).tolist(),
            "max_velocity_stamp_s": self._max_velocity_stamp_s,
            "max_xy_speed_mps": self._max_xy_speed_mps,
            "max_xy_velocity_mps": self._max_xy_velocity.astype(float).tolist(),
            "max_xy_velocity_stamp_s": self._max_xy_velocity_stamp_s,
        }


class RuntimeStageProfiler:
    """Low-overhead wall-time profiler for the simulation sensor loop."""

    def __init__(self, *, slow_threshold_s: float = 0.05, max_slow_events: int = 32) -> None:
        self.slow_threshold_s = max(0.0, float(slow_threshold_s))
        self.max_slow_events = max(1, int(max_slow_events))
        self._stages: dict[str, dict[str, float | int]] = {}
        self._slow_events: list[dict[str, float | str]] = []

    def record(self, stage: str, elapsed_s: float, sim_time_s: float) -> None:
        elapsed = max(0.0, float(elapsed_s))
        name = str(stage)
        stats = self._stages.setdefault(
            name,
            {
                "count": 0,
                "total_s": 0.0,
                "max_s": 0.0,
                "max_sim_time_s": 0.0,
            },
        )
        stats["count"] = int(stats["count"]) + 1
        stats["total_s"] = float(stats["total_s"]) + elapsed
        if elapsed > float(stats["max_s"]):
            stats["max_s"] = elapsed
            stats["max_sim_time_s"] = float(sim_time_s)
        if elapsed >= self.slow_threshold_s:
            self._slow_events.append({"stage": name, "elapsed_s": elapsed, "sim_time_s": float(sim_time_s)})
            self._slow_events.sort(key=lambda event: float(event["elapsed_s"]), reverse=True)
            del self._slow_events[self.max_slow_events :]

    def stats(self) -> dict[str, Any]:
        stages: dict[str, Any] = {}
        for name, values in sorted(self._stages.items()):
            count = int(values["count"])
            total_s = float(values["total_s"])
            stages[name] = {
                **values,
                "mean_s": total_s / float(count) if count > 0 else 0.0,
            }
        return {
            "slow_threshold_s": self.slow_threshold_s,
            "stages": stages,
            "slow_events": list(self._slow_events),
        }


def _gravity_specific_force_body(state: Any) -> np.ndarray:
    return -projected_gravity_body(state) * _MID360_ACCEL_MPS2_PER_G


class SimImuSignalConditioner:
    """Sensor-side conditioning for MuJoCo accelerometer samples.

    This keeps Fast-LIO on the normal accelerometer route while modeling the
    finite bandwidth and saturation envelope that a real IMU/driver chain has.
    """

    def __init__(
        self,
        *,
        lowpass_hz: float = DEFAULT_IMU_ACC_LOWPASS_HZ,
        max_dynamic_accel_mps2: float = DEFAULT_IMU_ACC_MAX_DYNAMIC_MPS2,
        max_slew_rate_mps3: float = DEFAULT_IMU_ACC_MAX_SLEW_MPS3,
    ) -> None:
        self.lowpass_hz = max(0.0, float(lowpass_hz))
        self.max_dynamic_accel_mps2 = max(0.0, float(max_dynamic_accel_mps2))
        self.max_slew_rate_mps3 = max(0.0, float(max_slew_rate_mps3))
        self._prev_accel: np.ndarray | None = None
        self._count = 0
        self._dynamic_clipped_count = 0
        self._slew_limited_count = 0
        self._raw_dynamic_max_mps2 = 0.0
        self._conditioned_dynamic_max_mps2 = 0.0

    def condition(self, accel: Any, *, state: Any, dt_s: float) -> np.ndarray:
        raw = np.asarray(accel, dtype=np.float64).reshape(3)
        gravity = _gravity_specific_force_body(state)
        dynamic = raw - gravity
        raw_dynamic_norm = float(np.linalg.norm(dynamic))
        self._raw_dynamic_max_mps2 = max(self._raw_dynamic_max_mps2, raw_dynamic_norm)

        if self.max_dynamic_accel_mps2 > 0.0 and raw_dynamic_norm > self.max_dynamic_accel_mps2:
            dynamic = dynamic * (self.max_dynamic_accel_mps2 / max(raw_dynamic_norm, 1e-12))
            self._dynamic_clipped_count += 1
        target = gravity + dynamic

        dt = max(0.0, float(dt_s))
        if self._prev_accel is not None and self.max_slew_rate_mps3 > 0.0 and dt > 0.0:
            delta = target - self._prev_accel
            delta_norm = float(np.linalg.norm(delta))
            max_delta = self.max_slew_rate_mps3 * dt
            if delta_norm > max_delta:
                target = self._prev_accel + delta * (max_delta / max(delta_norm, 1e-12))
                self._slew_limited_count += 1

        if self._prev_accel is not None and self.lowpass_hz > 0.0 and dt > 0.0:
            tau = 1.0 / (2.0 * math.pi * self.lowpass_hz)
            alpha = dt / (tau + dt)
            target = self._prev_accel + alpha * (target - self._prev_accel)

        self._prev_accel = target.astype(np.float64, copy=True)
        conditioned_dynamic_norm = float(np.linalg.norm(target - gravity))
        self._conditioned_dynamic_max_mps2 = max(
            self._conditioned_dynamic_max_mps2,
            conditioned_dynamic_norm,
        )
        self._count += 1
        return target.astype(np.float64, copy=False)

    def stats(self) -> dict[str, Any]:
        return {
            "enabled": True,
            "lowpass_hz": self.lowpass_hz,
            "max_dynamic_accel_mps2": self.max_dynamic_accel_mps2,
            "max_slew_rate_mps3": self.max_slew_rate_mps3,
            "sample_count": self._count,
            "dynamic_clipped_count": self._dynamic_clipped_count,
            "slew_limited_count": self._slew_limited_count,
            "raw_dynamic_max_mps2": self._raw_dynamic_max_mps2,
            "conditioned_dynamic_max_mps2": self._conditioned_dynamic_max_mps2,
        }


def _parse_xyz_triplet(value: str, label: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in str(value).split(",")]
    if len(parts) != 3:
        raise ValueError(f"{label} must contain exactly three comma-separated values")
    parsed = tuple(float(part) for part in parts)
    if not all(math.isfinite(item) for item in parsed):
        raise ValueError(f"{label} values must be finite")
    return parsed


@dataclass
class LinearMocapMotion:
    """Deterministic test-only motion for one MuJoCo mocap body."""

    mocap_id: int
    body_name: str
    start_xyz: tuple[float, float, float]
    end_xyz: tuple[float, float, float]
    start_s: float
    duration_s: float
    updates: int = 0
    motion_start_wall_s: float | None = None
    motion_complete_wall_s: float | None = None
    last_elapsed_s: float = 0.0
    last_alpha: float = 0.0
    last_xyz: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        if self.mocap_id < 0:
            raise ValueError("mocap_id must be non-negative")
        if not self.body_name:
            raise ValueError("mocap body name is required")
        if not math.isfinite(self.start_s) or self.start_s < 0.0:
            raise ValueError("mocap motion start must be finite and non-negative")
        if not math.isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("mocap motion duration must be positive and finite")
        if not all(math.isfinite(value) for value in (*self.start_xyz, *self.end_xyz)):
            raise ValueError("mocap motion positions must be finite")

    @classmethod
    def attach(
        cls,
        model: Any,
        *,
        body_name: str,
        start_xyz: tuple[float, float, float],
        end_xyz: tuple[float, float, float],
        start_s: float,
        duration_s: float,
    ) -> "LinearMocapMotion":
        import mujoco

        body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name))
        if body_id < 0:
            raise ValueError(f"MuJoCo mocap body not found: {body_name}")
        mocap_id = int(model.body_mocapid[body_id])
        if mocap_id < 0:
            raise ValueError(f"MuJoCo body is not mocap-controlled: {body_name}")
        return cls(
            mocap_id=mocap_id,
            body_name=body_name,
            start_xyz=start_xyz,
            end_xyz=end_xyz,
            start_s=float(start_s),
            duration_s=float(duration_s),
        )

    def update(self, data: Any, elapsed_s: float, *, wall_s: float | None = None) -> None:
        elapsed = max(0.0, float(elapsed_s))
        alpha = min(1.0, max(0.0, (elapsed - self.start_s) / self.duration_s))
        position = tuple(start + (end - start) * alpha for start, end in zip(self.start_xyz, self.end_xyz, strict=True))
        data.mocap_pos[self.mocap_id] = position
        data.mocap_quat[self.mocap_id] = (1.0, 0.0, 0.0, 0.0)
        now = time.time() if wall_s is None else float(wall_s)
        if alpha > 0.0 and self.motion_start_wall_s is None:
            self.motion_start_wall_s = now
        if alpha >= 1.0 and self.motion_complete_wall_s is None:
            self.motion_complete_wall_s = now
        self.updates += 1
        self.last_elapsed_s = elapsed
        self.last_alpha = alpha
        self.last_xyz = position

    def stats(self) -> dict[str, Any]:
        return {
            "enabled": True,
            "body_name": self.body_name,
            "start_xyz": list(self.start_xyz),
            "end_xyz": list(self.end_xyz),
            "start_s": self.start_s,
            "duration_s": self.duration_s,
            "updates": self.updates,
            "motion_started": self.motion_start_wall_s is not None,
            "motion_completed": self.motion_complete_wall_s is not None,
            "motion_start_wall_s": self.motion_start_wall_s,
            "motion_complete_wall_s": self.motion_complete_wall_s,
            "last_elapsed_s": self.last_elapsed_s,
            "last_alpha": self.last_alpha,
            "last_xyz": list(self.last_xyz) if self.last_xyz is not None else None,
        }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="industrial_park")
    parser.add_argument(
        "--mocap-motion-body",
        default="",
        help="Optional test-only mocap body moved on a deterministic linear trajectory.",
    )
    parser.add_argument("--mocap-motion-start", default="0,0,0")
    parser.add_argument("--mocap-motion-end", default="0,0,0")
    parser.add_argument("--mocap-motion-start-s", type=float, default=0.0)
    parser.add_argument("--mocap-motion-duration-s", type=float, default=1.0)
    parser.add_argument("--start", default="", help="Optional start pose x,y,z")
    parser.add_argument(
        "--start-yaw-deg",
        type=float,
        default=None,
        help="Optional initial body yaw in world degrees.",
    )
    parser.add_argument(
        "--start-anchor",
        choices=["off", "warmup", "run"],
        default="off",
        help=(
            "Keep the MuJoCo base at the configured start pose during SLAM "
            "initialization, or for the complete no-motion run. Motion "
            "acceptance must use warmup so the base is released before driving."
        ),
    )
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--settle-s",
        type=float,
        default=3.0,
        help=(
            "Advance MuJoCo with a zero command before starting the simulated "
            "hardware clock or publishing sensors, keeping Fast-LIO IMU "
            "initialization away from model drop/settle transients."
        ),
    )
    parser.add_argument(
        "--warmup-s",
        type=float,
        default=2.0,
        help="Publish stationary IMU/LiDAR before driving so Fast-LIO can initialize gyro bias at rest.",
    )
    parser.add_argument(
        "--drive-ramp-s",
        type=float,
        default=5.0,
        help="Ramp kinematic cmd_vel after warmup to avoid an unrealistic IMU acceleration step.",
    )
    parser.add_argument("--publish-hz", type=float, default=10.0, help="LiDAR frame rate.")
    parser.add_argument("--imu-hz", type=float, default=200.0)
    parser.add_argument(
        "--publish-odom-prior",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Publish MuJoCo odometry to /slam/odom_prior for simulation-only "
            "diagnostics. Real hardware configs keep this path off, and "
            "Fast-LIO simulation acceptance must not depend on it."
        ),
    )
    parser.add_argument(
        "--navigation-fixture",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Publish MuJoCo truth odometry, identity map-to-odom TF, healthy "
            "localization state, and a body-frame registered cloud through the "
            "native DDS publisher. This is only for navigation functional acceptance."
        ),
    )
    parser.add_argument(
        "--navigation-fixture-cloud-points",
        type=int,
        default=4000,
        help="Maximum body-frame registered-cloud points per navigation fixture frame.",
    )
    parser.add_argument(
        "--navigation-fixture-ground-resolution-m",
        type=float,
        default=NAVIGATION_FIXTURE_GROUND_RESOLUTION_M,
        help="Synthetic ground sample spacing for navigation functional acceptance.",
    )
    parser.add_argument(
        "--navigation-fixture-ground-y-half-m",
        type=float,
        default=NAVIGATION_FIXTURE_GROUND_Y_HALF_M,
        help=(
            "Half-width of synthetic traversable ground around the robot. "
            "Navigation acceptance scenes should match this to their drivable floor."
        ),
    )
    parser.add_argument(
        "--navigation-fixture-raw-overlay",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Overlay MuJoCo raw body-frame returns on top of synthetic fixture ground. "
            "Disable for free-space control acceptance so self/side-wall returns cannot masquerade as obstacles."
        ),
    )
    parser.add_argument(
        "--imu-acc-mode",
        choices=["sensor", "finite_difference", "gravity_only"],
        default="sensor",
        help=(
            "IMU specific-force source. The native DDS MuJoCo default is "
            "sensor so Fast-LIO receives the same raw accelerometer route as "
            "real hardware. Use gravity_only only as a diagnostic fallback; "
            "finite_difference is legacy."
        ),
    )
    parser.add_argument(
        "--imu-acc-conditioning",
        choices=["realistic", "off"],
        default=DEFAULT_IMU_ACC_CONDITIONING,
        help=(
            "Condition MuJoCo accelerometer samples before publishing raw IMU. "
            "The default keeps sensor mode on the real Fast-LIO route while "
            "adding sensor-bandwidth low-pass, dynamic acceleration clipping, "
            "and slew limiting to suppress non-physical contact impulses."
        ),
    )
    parser.add_argument(
        "--imu-acc-lowpass-hz",
        type=float,
        default=DEFAULT_IMU_ACC_LOWPASS_HZ,
        help="First-order accelerometer low-pass cutoff used by realistic conditioning.",
    )
    parser.add_argument(
        "--imu-acc-max-dynamic-mps2",
        type=float,
        default=DEFAULT_IMU_ACC_MAX_DYNAMIC_MPS2,
        help="Maximum body-frame dynamic acceleration magnitude before clipping in realistic conditioning.",
    )
    parser.add_argument(
        "--imu-acc-max-slew-mps3",
        type=float,
        default=DEFAULT_IMU_ACC_MAX_SLEW_MPS3,
        help="Maximum accelerometer slew rate before clipping in realistic conditioning.",
    )
    parser.add_argument(
        "--imu-acc-axis-scale",
        default="auto",
        help=(
            "Comma-separated XYZ scale applied to simulated IMU specific force before DDS publication. "
            "'auto' uses the calibrated MuJoCo kinematic profile."
        ),
    )
    parser.add_argument(
        "--imu-gyro-axis-scale",
        default="1,1,1",
        help="Comma-separated XYZ scale applied to simulated IMU angular velocity before DDS publication.",
    )
    parser.add_argument(
        "--scan-time-profile",
        choices=["instantaneous", "synthetic_rolling", "physical_rolling"],
        default="physical_rolling",
    )
    parser.add_argument(
        "--physical-rolling-sample-mode",
        choices=["full_frame", "subscan"],
        default="subscan",
        help=(
            "How physical_rolling collects MuJoCo LiDAR samples. subscan allocates "
            "the MID-360 scan pattern across IMU samples and matches rolling-scan "
            "timing. full_frame is a legacy diagnostic compatibility mode and must "
            "not be used as saved-map acceptance evidence."
        ),
    )
    parser.add_argument(
        "--timestamp-clock",
        choices=[SIM_HARDWARE_CLOCK, "sim", "wall"],
        default=SIM_HARDWARE_CLOCK,
        help=(
            "Clock profile used for DDS LiDAR/IMU timestamps. sim_hardware is the "
            "product MuJoCo bridge mode: LiDAR scan timing, IMU sampling, and "
            "publish pacing share one simulated hardware clock. wall/sim are "
            "legacy diagnostics."
        ),
    )
    parser.add_argument(
        "--sim-hardware-realtime-factor",
        type=float,
        default=1.0,
        help="Wall pacing factor for sim_hardware mode. 1.0 means one simulated second per wall second.",
    )
    parser.add_argument(
        "--sim-hardware-max-lag-s",
        type=float,
        default=DEFAULT_SIM_HARDWARE_MAX_LAG_S,
        help=(
            "Maximum simulated-hardware lag behind its wall pacing target before "
            "intermediate sensor ticks are dropped while MuJoCo dynamics continue."
        ),
    )
    parser.add_argument(
        "--odom-prior-velocity-window-s",
        type=float,
        default=DEFAULT_ODOM_PRIOR_VELOCITY_WINDOW_S,
        help=(
            "Pose-difference window used for the simulation-only odometry prior velocity. "
            "This rejects MuJoCo contact-solver qvel impulses without clipping real pose jumps."
        ),
    )
    parser.add_argument(
        "--sim-hardware-catch-up-yield-steps",
        type=int,
        default=DEFAULT_SIM_HARDWARE_CATCH_UP_YIELD_STEPS,
        help=(
            "Yield to peer DDS processes after this many consecutive catch-up "
            "physics steps. Physics is never time-jumped."
        ),
    )
    parser.add_argument(
        "--parent-diagnostics-json",
        default="",
        help="Optional rolling atomic JSON snapshot of parent-side sensor scheduling diagnostics.",
    )
    parser.add_argument(
        "--parent-diagnostics-period-s",
        type=float,
        default=DEFAULT_PARENT_DIAGNOSTICS_PERIOD_S,
        help="Rolling parent-side diagnostics snapshot period when --parent-diagnostics-json is set.",
    )
    parser.add_argument(
        "--publisher-write-mode",
        choices=["sync", "async_fifo"],
        default=DEFAULT_PUBLISHER_WRITE_MODE,
        help="Native publisher stdin mode; sync preserves the default blocking write path.",
    )
    parser.add_argument(
        "--async-publisher-max-bytes",
        type=int,
        default=DEFAULT_ASYNC_PUBLISHER_MAX_BYTES,
    )
    parser.add_argument(
        "--async-publisher-max-records",
        type=int,
        default=DEFAULT_ASYNC_PUBLISHER_MAX_RECORDS,
    )
    parser.add_argument(
        "--async-publisher-max-batches",
        type=int,
        default=DEFAULT_ASYNC_PUBLISHER_MAX_BATCHES,
    )
    parser.add_argument(
        "--async-publisher-oldest-s",
        type=float,
        default=DEFAULT_ASYNC_PUBLISHER_OLDEST_S,
    )
    parser.add_argument(
        "--async-publisher-shutdown-s",
        type=float,
        default=DEFAULT_ASYNC_PUBLISHER_SHUTDOWN_S,
    )
    parser.add_argument(
        "--imu-timestamp-clock",
        choices=["", SIM_HARDWARE_CLOCK, "sim", "wall"],
        default="",
        help="Optional IMU timestamp clock override. Empty inherits --timestamp-clock.",
    )
    parser.add_argument(
        "--lidar-timestamp-clock",
        choices=["", SIM_HARDWARE_CLOCK, "sim", "wall"],
        default="",
        help="Optional LiDAR scan timestamp clock override. Empty inherits --timestamp-clock.",
    )
    parser.add_argument("--min-localization-quality", type=float, default=0.5)
    parser.add_argument("--max-odom-abs-m", type=float, default=100.0)
    parser.add_argument("--max-odom-z-abs-m", type=float, default=20.0)
    parser.add_argument(
        "--min-sim-motion-for-odom-check-m",
        type=float,
        default=0.5,
        help="When requiring SLAM output, fail if MuJoCo moves at least this far but SLAM odometry barely moves.",
    )
    parser.add_argument(
        "--min-slam-motion-ratio",
        type=float,
        default=0.50,
        help="Minimum SLAM odom XY motion divided by MuJoCo XY motion for the motion-consistency gate.",
    )
    parser.add_argument(
        "--max-slam-motion-ratio",
        type=float,
        default=1.6,
        help="Maximum SLAM odom XY motion divided by MuJoCo XY motion for the motion-consistency gate.",
    )
    parser.add_argument(
        "--max-slam-map-xy-error-m",
        type=float,
        default=0.35,
        help="Maximum map-frame localization XY error when map tracking is active.",
    )
    parser.add_argument(
        "--min-sim-yaw-for-odom-check-rad",
        type=float,
        default=0.20,
        help="When MuJoCo yaw changes by at least this much, compare SLAM odom yaw against it.",
    )
    parser.add_argument(
        "--max-slam-yaw-error-rad",
        type=float,
        default=0.15,
        help="Maximum absolute SLAM odom yaw error relative to MuJoCo yaw delta.",
    )
    parser.add_argument("--domain-id", type=_dds_domain_id, default=0)
    parser.add_argument(
        "--external-arm-file",
        default="",
        help=(
            "Optional atomic JSON rendezvous. While enabled, the bridge keeps "
            "the base anchored and publishes stationary sensors until a valid arm is observed."
        ),
    )
    parser.add_argument(
        "--external-arm-token",
        default="",
        help="Opaque per-run token required by --external-arm-file.",
    )
    parser.add_argument(
        "--external-arm-scenario",
        default="",
        help="Exact acceptance scenario expected in the external arm document.",
    )
    parser.add_argument(
        "--external-arm-timeout-s",
        type=float,
        default=DEFAULT_EXTERNAL_ARM_TIMEOUT_S,
        help="Wall-clock deadline for a valid external arm document.",
    )
    parser.add_argument(
        "--external-arm-status-json",
        default="",
        help=(
            "Optional atomic status snapshot for the external-arm handshake. "
            "The opaque token is never written to this file."
        ),
    )
    parser.add_argument("--publisher-bin", default=os.environ.get("LINGTU_MUJOCO_NATIVE_DDS_PUBLISHER_BIN", ""))
    parser.add_argument("--slam-status-json", default=os.environ.get("LINGTU_SLAM_STATUS_JSON", ""))
    parser.add_argument("--drive-mode", choices=["kinematic", "policy"], default="policy")
    parser.add_argument(
        "--viewer",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Open a passive MuJoCo viewer. The viewer is presentation-only; "
            "motion still comes from --command-source and the physics loop."
        ),
    )
    parser.add_argument(
        "--viewer-hz",
        type=float,
        default=30.0,
        help="Maximum passive-viewer refresh rate.",
    )
    parser.add_argument(
        "--command-source",
        choices=["profile", "dds"],
        default="profile",
        help=(
            "Velocity source for the MuJoCo engine. profile uses the deterministic "
            "diagnostic command below; dds consumes the final typed-DDS /nav/cmd_vel "
            "through the native physical driver bridge and is required for native "
            "Product acceptance."
        ),
    )
    parser.add_argument(
        "--driver-bridge-bin",
        default=os.environ.get("LINGTU_MUJOCO_DRIVER_BRIDGE_BIN", ""),
        help=(
            "Verified native C++ physical driver bridge artifact used by "
            "--command-source dds. No build-directory discovery is performed."
        ),
    )
    parser.add_argument("--driver-bridge-pid-file", default="")
    parser.add_argument(
        "--driver-expected-host-boot-id",
        default=os.environ.get("LINGTU_HOST_BOOT_ID", ""),
        help="Exact FinalVelocityCommand producer identity accepted by the driver bridge.",
    )
    parser.add_argument("--driver-max-linear-mps", type=float, default=1.0)
    parser.add_argument("--driver-max-angular-rps", type=float, default=1.0)
    parser.add_argument("--driver-command-timeout-ms", type=int, default=200)
    parser.add_argument("--driver-heartbeat-timeout-ms", type=int, default=500)
    parser.add_argument("--driver-apply-timeout-ms", type=int, default=500)
    parser.add_argument("--publisher-pid-file", default="")
    parser.add_argument(
        "--require-cmd-vel",
        action="store_true",
        help="Fail the sensor/locomotion report unless a non-zero DDS command was consumed.",
    )
    parser.add_argument(
        "--allow-kinematic-fastlio-acceptance",
        action="store_true",
        help=(
            "Allow require-slam-output gates to pass with kinematic drive. "
            "Default false because kinematic cmd_vel pose injection is not a "
            "hardware-equivalent LiDAR/IMU source for Fast-LIO saved-map claims."
        ),
    )
    parser.add_argument(
        "--drive-profile",
        choices=["arc", "box_explore", "box_explore_gentle"],
        default="arc",
        help=(
            "Kinematic command profile. arc preserves the historical constant "
            "cmd_vel path; box_explore alternates forward runs and turns to "
            "exercise long-map accumulation."
        ),
    )
    parser.add_argument("--drive-vx", type=float, default=0.10)
    parser.add_argument("--drive-vy", type=float, default=0.0)
    parser.add_argument("--drive-wz", type=float, default=0.04)
    parser.add_argument(
        "--physics-integrator",
        choices=["model", "euler", "rk4", "implicit", "implicitfast"],
        default="model",
        help=(
            "MuJoCo integration scheme for this run. model preserves the XML "
            "setting; navigation functional gates may select Euler explicitly "
            "while retaining the model timestep and 50 Hz policy cadence."
        ),
    )
    parser.add_argument(
        "--physics-timestep-s",
        type=float,
        default=0.0,
        help=("Override the loaded MuJoCo model timestep for this simulation run. Zero preserves the model value."),
    )
    parser.add_argument(
        "--policy-cpu-threads",
        type=int,
        default=1,
        help=(
            "CPU threads used by the small gait policy. Product simulation "
            "defaults to one to avoid starving native DDS processes."
        ),
    )
    parser.add_argument("--policy-path", default=os.environ.get("LINGTU_MUJOCO_NATIVE_DDS_POLICY_PATH", ""))
    parser.add_argument("--mujoco-memory", default="64M")
    parser.add_argument("--mid360-pattern", type=Path, default=DEFAULT_MID360_PATTERN)
    parser.add_argument("--mid360-samples-per-frame", type=int, default=DEFAULT_MID360_SAMPLES_PER_FRAME)
    parser.add_argument("--lidar-backend", choices=["mujoco_lidar", "ray_caster_lidar"], default="mujoco_lidar")
    parser.add_argument("--mujoco-lidar-backend", choices=["cpu", "taichi", "warp", "jax"], default="cpu")
    parser.add_argument("--max-points", type=int, default=DEFAULT_MID360_SAMPLES_PER_FRAME)
    parser.add_argument(
        "--require-slam-output",
        action="store_true",
        help="Fail unless native C++ SLAM publishes odometry, map cloud, and health.",
    )
    parser.add_argument(
        "--motion-log",
        default="",
        help=(
            "Optional JSONL path for periodic MuJoCo ground-truth, native nav, and live LiDAR visualization samples."
        ),
    )
    parser.add_argument(
        "--motion-log-hz",
        type=float,
        default=24.0,
        help="Simulation-time sampling rate for --motion-log.",
    )
    parser.add_argument(
        "--motion-log-lidar-points",
        type=int,
        default=640,
        help="Maximum live world-frame LiDAR points stored in each motion-log sample.",
    )
    parser.add_argument(
        "--nav-status-json",
        default="",
        help="Native nav endpoint status snapshot used only for acceptance visualization logging.",
    )
    parser.add_argument(
        "--stop-on-nav-goal-reached",
        action="store_true",
        help="End the simulation cleanly once native navigation reports goal_reached.",
    )
    parser.add_argument(
        "--motion-complete-marker",
        default="",
        help="Atomic marker written after the motion interval and before process cleanup.",
    )
    parser.add_argument("--json-out", default="")
    return parser


def _runtime_imu_from_state(
    state: Any,
    ts: float,
    prev_velocity: Any = None,
    dt: float = 0.0,
    acc_mode: str = "finite_difference",
    acc_conditioner: SimImuSignalConditioner | None = None,
) -> Imu:
    quat = np.asarray(getattr(state, "orientation", (0.0, 0.0, 0.0, 1.0)), dtype=float)
    gyro = np.asarray(getattr(state, "imu_gyro", (0.0, 0.0, 0.0)), dtype=float)
    accel = specific_force_body(state, prev_velocity, dt, mode=acc_mode)
    if acc_conditioner is not None:
        accel = acc_conditioner.condition(accel, state=state, dt_s=dt)
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


def _resolve_imu_acc_axis_scale(
    value: str,
    *,
    drive_mode: str,
    acc_mode: str,
    timestamp_clock: str = "",
) -> tuple[tuple[float, float, float], str]:
    text = str(value or "auto").strip().lower()
    if text == "auto":
        if str(drive_mode).strip().lower() == "kinematic" and str(acc_mode).strip().lower() == "finite_difference":
            if str(timestamp_clock).strip().lower() == SIM_HARDWARE_CLOCK:
                return KINEMATIC_SIM_HARDWARE_IMU_ACC_AXIS_SCALE, "auto_kinematic_sim_hardware"
            return KINEMATIC_LEGACY_IMU_ACC_AXIS_SCALE, "auto_kinematic_legacy_split"
        return (1.0, 1.0, 1.0), "auto_identity"

    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 3:
        raise ValueError("--imu-acc-axis-scale must be 'auto' or three comma-separated numbers")
    try:
        x, y, z = (float(part) for part in parts)
    except ValueError as exc:
        raise ValueError("--imu-acc-axis-scale contains a non-numeric value") from exc
    return (x, y, z), "explicit"


def _apply_imu_acc_axis_scale(imu: Imu, scale: tuple[float, float, float]) -> None:
    imu.linear_acceleration.x *= float(scale[0])
    imu.linear_acceleration.y *= float(scale[1])
    imu.linear_acceleration.z *= float(scale[2])


def _resolve_axis_scale(value: str, name: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in str(value or "").split(",")]
    if len(parts) != 3:
        raise ValueError(f"{name} must contain three comma-separated numbers")
    try:
        values = tuple(float(part) for part in parts)
    except ValueError as exc:
        raise ValueError(f"{name} contains a non-numeric value") from exc
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"{name} values must be finite")
    return values  # type: ignore[return-value]


def _apply_imu_gyro_axis_scale(imu: Imu, scale: tuple[float, float, float]) -> None:
    imu.angular_velocity.x *= float(scale[0])
    imu.angular_velocity.y *= float(scale[1])
    imu.angular_velocity.z *= float(scale[2])


def _drive_command_for_profile(
    profile: str,
    elapsed_s: float,
    *,
    drive_vx: float,
    drive_vy: float,
    drive_wz: float,
) -> tuple[float, float, float]:
    name = str(profile or "arc").strip().lower()
    if name == "arc":
        return float(drive_vx), float(drive_vy), float(drive_wz)
    if name == "box_explore_gentle":
        forward_vx = max(abs(float(drive_vx)), 0.10)
        turn_wz = math.copysign(max(abs(float(drive_wz)), 0.20), float(drive_wz) if drive_wz else 1.0)
        phase_s = max(0.0, float(elapsed_s)) % 28.0
        if phase_s < 9.0:
            return forward_vx, 0.0, 0.0
        if phase_s < 13.0:
            return 0.0, 0.0, turn_wz
        if phase_s < 22.0:
            return forward_vx, 0.0, 0.0
        return 0.0, 0.0, turn_wz
    if name == "box_explore":
        forward_vx = max(abs(float(drive_vx)), 0.10)
        turn_wz = math.copysign(max(abs(float(drive_wz)), 0.35), float(drive_wz) if drive_wz else 1.0)
        phase_s = max(0.0, float(elapsed_s)) % 24.0
        if phase_s < 7.5:
            return forward_vx, 0.0, 0.0
        if phase_s < 12.0:
            return 0.0, 0.0, turn_wz
        if phase_s < 19.5:
            return forward_vx, 0.0, 0.0
        return 0.0, 0.0, turn_wz
    raise ValueError(f"unknown drive profile: {profile}")


class SimulatedHardwareClock:
    """One simulated sensor clock with optional wall pacing.

    Timestamps advance in simulated seconds. Wall time only throttles publication
    so native consumers see a realistic sensor stream instead of a bursty replay.
    """

    def __init__(
        self,
        *,
        sim_start_s: float,
        wall_epoch_s: float,
        monotonic_start_s: float,
        realtime_factor: float = 1.0,
    ) -> None:
        if not math.isfinite(float(realtime_factor)) or float(realtime_factor) <= 0.0:
            raise ValueError("--sim-hardware-realtime-factor must be positive and finite")
        self.sim_start_s = float(sim_start_s)
        self.wall_epoch_s = float(wall_epoch_s) - self.sim_start_s
        self.monotonic_start_s = float(monotonic_start_s)
        self.realtime_factor = float(realtime_factor)

    def timestamp_s(self, sim_time_s: float) -> float:
        return self.wall_epoch_s + float(sim_time_s)

    def monotonic_deadline_s(self, sim_time_s: float) -> float:
        sim_elapsed_s = max(0.0, float(sim_time_s) - self.sim_start_s)
        return self.monotonic_start_s + sim_elapsed_s / self.realtime_factor

    def expected_sim_time_s(self, monotonic_now_s: float | None = None) -> float:
        now_s = time.monotonic() if monotonic_now_s is None else float(monotonic_now_s)
        wall_elapsed_s = max(0.0, now_s - self.monotonic_start_s)
        return self.sim_start_s + wall_elapsed_s * self.realtime_factor

    def lag_s(self, sim_time_s: float, monotonic_now_s: float | None = None) -> float:
        return self.expected_sim_time_s(monotonic_now_s) - float(sim_time_s)

    def sleep_until(self, sim_time_s: float) -> None:
        sleep_s = self.monotonic_deadline_s(sim_time_s) - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)


def _select_native_wall_clock_alignment(
    samples: Sequence[tuple[int, int, int]],
    *,
    max_rtt_s: float = MAX_NATIVE_CLOCK_SYNC_RTT_S,
) -> dict[str, Any]:
    """Select the lowest-uncertainty native wall-clock sample."""

    valid: list[tuple[int, int]] = []
    for local_before_ns, native_wall_ns, local_after_ns in samples:
        if local_before_ns <= 0 or native_wall_ns <= 0 or local_after_ns < local_before_ns:
            continue
        rtt_ns = local_after_ns - local_before_ns
        midpoint_ns = local_before_ns + rtt_ns // 2
        valid.append((rtt_ns, native_wall_ns - midpoint_ns))
    if not valid:
        raise RuntimeError("native wall-clock synchronization returned no valid samples")
    rtt_ns, offset_ns = min(valid, key=lambda value: value[0])
    if rtt_ns > int(float(max_rtt_s) * 1_000_000_000):
        raise RuntimeError(f"native wall-clock synchronization uncertainty too high: rtt_ms={rtt_ns / 1_000_000.0:.3f}")
    return {
        "source": "native_wall_midpoint_lowest_rtt",
        "sample_count": len(valid),
        "rtt_ms": rtt_ns / 1_000_000.0,
        "uncertainty_ms": rtt_ns / 2_000_000.0,
        "native_minus_local_s": offset_ns / 1_000_000_000.0,
    }


class SimHardwareCatchUpController:
    """Drop only sensor observations while advancing every MuJoCo physics tick.

    A fixed simulated-hardware epoch is useful only while simulated time stays
    close to the wall pacing target. Under host load, publishing every overdue
    observation preserves sample count but makes every source timestamp stale.
    This controller instead accounts for omitted observations and lets the
    normal small-step dynamics catch back up without changing timestamp rules.
    """

    def __init__(
        self,
        *,
        clock: SimulatedHardwareClock,
        max_lag_s: float = DEFAULT_SIM_HARDWARE_MAX_LAG_S,
        yield_every_steps: int = DEFAULT_SIM_HARDWARE_CATCH_UP_YIELD_STEPS,
    ) -> None:
        if not math.isfinite(float(max_lag_s)) or float(max_lag_s) < 0.0:
            raise ValueError("--sim-hardware-max-lag-s must be non-negative and finite")
        if int(yield_every_steps) <= 0:
            raise ValueError("--sim-hardware-catch-up-yield-steps must be positive")
        self.clock = clock
        self.max_lag_s = float(max_lag_s)
        self.yield_every_steps = int(yield_every_steps)
        self.catch_up_events = 0
        self.dropped_imu_ticks = 0
        self.dropped_lidar_subscan_ticks = 0
        self.dropped_lidar_frames = 0
        self.fast_static_clock_ticks = 0
        self.dynamic_catch_up_ticks = 0
        self.catch_up_yields = 0
        self.forced_sensor_observations = 0
        self.forced_lidar_observations = 0
        self.max_lag_observed_s = 0.0
        self.max_consecutive_steps = 0
        self._consecutive_steps = 0
        self._catching_up = False

    def should_drop_sensor_tick(
        self,
        sim_time_s: float,
        *,
        monotonic_now_s: float | None = None,
    ) -> bool:
        lag_s = self.clock.lag_s(sim_time_s, monotonic_now_s)
        self.max_lag_observed_s = max(self.max_lag_observed_s, lag_s)
        if lag_s <= self.max_lag_s:
            self._catching_up = False
            self._consecutive_steps = 0
            return False
        if not self._catching_up:
            self.catch_up_events += 1
        self._catching_up = True
        self._consecutive_steps += 1
        self.max_consecutive_steps = max(self.max_consecutive_steps, self._consecutive_steps)
        self.dropped_imu_ticks += 1
        return True

    def force_sensor_observation(self, *, reason: str) -> None:
        """Keep a due coherent sensor frame even while wall-clock catch-up is active."""

        if self.dropped_imu_ticks > 0:
            self.dropped_imu_ticks -= 1
        self.forced_sensor_observations += 1
        if reason == "lidar_due":
            self.forced_lidar_observations += 1
        self._consecutive_steps = 0

    def record_lidar_subscan_drop(self) -> None:
        self.dropped_lidar_subscan_ticks += 1

    def record_lidar_frame_drops(self, count: int) -> None:
        self.dropped_lidar_frames += max(0, int(count))

    def record_catch_up_step(self, *, fast_static_clock: bool) -> None:
        if fast_static_clock:
            self.fast_static_clock_ticks += 1
        else:
            self.dynamic_catch_up_ticks += 1

    def yield_if_due(self) -> None:
        if self._consecutive_steps > 0 and self._consecutive_steps % self.yield_every_steps == 0:
            self.catch_up_yields += 1
            time.sleep(0.0)

    def stats(self, sim_time_s: float, *, monotonic_now_s: float | None = None) -> dict[str, Any]:
        final_lag_s = self.clock.lag_s(sim_time_s, monotonic_now_s)
        return {
            "enabled": True,
            "strategy": "small_step_dynamics_preserve_lidar_deadlines",
            "max_lag_s": self.max_lag_s,
            "yield_every_steps": self.yield_every_steps,
            "catch_up_events": self.catch_up_events,
            "catch_up_yields": self.catch_up_yields,
            "forced_sensor_observations": self.forced_sensor_observations,
            "forced_lidar_observations": self.forced_lidar_observations,
            "dropped_imu_ticks": self.dropped_imu_ticks,
            "dropped_lidar_subscan_ticks": self.dropped_lidar_subscan_ticks,
            "dropped_lidar_frames": self.dropped_lidar_frames,
            "fast_static_clock_ticks": self.fast_static_clock_ticks,
            "dynamic_catch_up_ticks": self.dynamic_catch_up_ticks,
            "max_lag_observed_s": self.max_lag_observed_s,
            "max_consecutive_steps": self.max_consecutive_steps,
            "final_lag_s": final_lag_s,
        }


def _advance_skipped_lidar_deadlines(
    *,
    next_lidar_sim_s: float,
    lidar_period_s: float,
    sim_time_s: float,
) -> tuple[float, int]:
    """Advance scheduled LiDAR frames crossed by a sensor-drop physics step."""

    next_due_s = float(next_lidar_sim_s)
    period_s = float(lidar_period_s)
    if not math.isfinite(period_s) or period_s <= 0.0:
        raise ValueError("lidar period must be positive and finite")
    dropped = 0
    while sim_time_s + 1e-9 >= next_due_s:
        next_due_s += period_s
        dropped += 1
    return next_due_s, dropped


def _sensor_timestamp_s(
    *,
    clock: str,
    sim_clock_epoch_s: float,
    sim_time_s: float,
    wall_time_s: float,
) -> float:
    if str(clock or "sim") == "wall":
        return float(wall_time_s)
    return float(sim_clock_epoch_s) + float(sim_time_s)


def _scan_start_timestamp_s(
    *,
    clock: str,
    sim_clock_epoch_s: float,
    scan_start_sim_s: float,
    wall_scan_start_s: float,
) -> float:
    if str(clock or "sim") == "wall":
        return float(wall_scan_start_s)
    return float(sim_clock_epoch_s) + float(scan_start_sim_s)


def _uses_unified_sim_hardware_clock(*, timestamp_clock: str, imu_clock: str, lidar_clock: str) -> bool:
    return (
        str(timestamp_clock) == SIM_HARDWARE_CLOCK
        and str(imu_clock) == SIM_HARDWARE_CLOCK
        and str(lidar_clock) == SIM_HARDWARE_CLOCK
    )


def _scan_stamp_sim_time_s(*, scan_time_profile: str, scan_start_sim_s: float, scan_end_sim_s: float) -> float:
    if str(scan_time_profile or "").strip().lower() == "instantaneous":
        return float(scan_end_sim_s)
    return float(scan_start_sim_s)


def _scan_stamp_wall_time_s(*, scan_time_profile: str, wall_scan_start_s: float, wall_scan_end_s: float) -> float:
    if str(scan_time_profile or "").strip().lower() == "instantaneous":
        return float(wall_scan_end_s)
    return float(wall_scan_start_s)


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


_MOTION_LOG_LIDAR_MAX_Z_ABOVE_ROBOT_M = 1.8
_MOTION_LOG_LIDAR_LOCAL_RADIUS_M = 4.0
_MOTION_LOG_LIDAR_LOCAL_SHARE = 0.75


def _motion_log_lidar_sample(
    points: Any,
    *,
    max_points: int,
    robot_xyz: Any,
) -> np.ndarray:
    """Keep a dense local scan while retaining some wider scene context."""

    pts = _bounded_points(points, 0)
    if pts.shape[0] <= 0 or max_points <= 0:
        return np.zeros((0, 4), dtype=np.float32)
    robot = np.asarray(robot_xyz, dtype=np.float64).reshape(-1)
    if robot.size < 3 or not np.isfinite(robot[:3]).all():
        robot = np.zeros(3, dtype=np.float64)
    finite = np.isfinite(pts[:, :3]).all(axis=1)
    below_presentation_ceiling = pts[:, 2] <= float(robot[2]) + _MOTION_LOG_LIDAR_MAX_Z_ABOVE_ROBOT_M
    visible = pts[finite & below_presentation_ceiling]
    if visible.shape[0] <= max_points:
        return visible.astype(np.float32, copy=False)

    delta_xy = visible[:, :2].astype(np.float64) - robot[:2]
    local_mask = np.einsum("ij,ij->i", delta_xy, delta_xy) <= (_MOTION_LOG_LIDAR_LOCAL_RADIUS_M**2)
    local = visible[local_mask]
    context = visible[~local_mask]
    target_local = min(
        len(local),
        int(math.ceil(max_points * _MOTION_LOG_LIDAR_LOCAL_SHARE)),
    )
    target_context = min(len(context), max_points - target_local)
    target_local = min(len(local), max_points - target_context)
    sampled_local = _bounded_points(local, target_local)
    sampled_context = _bounded_points(context, target_context)
    if sampled_local.shape[0] == 0:
        return sampled_context
    if sampled_context.shape[0] == 0:
        return sampled_local
    return np.vstack([sampled_local, sampled_context]).astype(np.float32, copy=False)


def _rolling_subscan_sample_count(
    *,
    frame_samples: int,
    imu_period_s: float,
    lidar_period_s: float,
) -> int:
    if lidar_period_s <= 0.0:
        return max(1, int(frame_samples))
    fraction = max(0.0, float(imu_period_s)) / float(lidar_period_s)
    return max(1, int(math.ceil(max(1, int(frame_samples)) * fraction)))


def _step_engine_for_sensor_tick(engine: Any, cmd: VelocityCommand, imu_period_s: float) -> Any:
    step_sensor_tick = getattr(engine, "step_sensor_tick", None)
    if callable(step_sensor_tick):
        return step_sensor_tick(cmd, dt_s=float(imu_period_s))
    return engine.step(cmd)


def _step_static_engine_for_sensor_tick(engine: Any, imu_period_s: float) -> Any:
    step_static = getattr(engine, "step_static_sensor_tick", None)
    if not callable(step_static):
        raise RuntimeError("MuJoCo engine does not implement static sensor stepping")
    return step_static(dt_s=float(imu_period_s))


def _advance_static_engine_clock_for_dropped_tick(engine: Any, imu_period_s: float) -> float:
    advance_clock = getattr(engine, "advance_static_sensor_clock", None)
    if not callable(advance_clock):
        raise RuntimeError("MuJoCo engine does not implement fast anchored clock advance")
    return float(advance_clock(dt_s=float(imu_period_s)))


def _start_anchor_active(mode: str, *, motion_started: bool) -> bool:
    return mode == "run" or (mode == "warmup" and not motion_started)


def _external_arm_drive_elapsed_s(
    gate: ExternalArmGate | None,
    *,
    sim_time_s: float,
) -> float:
    if gate is None or not gate.acknowledged or gate.arm_observed_sim_time_s is None:
        return 0.0
    return max(0.0, float(sim_time_s) - gate.arm_observed_sim_time_s)


def _sensor_anchor_active(
    mode: str,
    *,
    motion_started: bool,
    external_arm_gate: ExternalArmGate | None,
) -> bool:
    waiting_for_external_arm = external_arm_gate is not None and not external_arm_gate.acknowledged
    return waiting_for_external_arm or _start_anchor_active(mode, motion_started=motion_started)


def _driver_bridge_anchor_state(
    mode: str,
    *,
    motion_started: bool,
    driving: bool,
    external_arm_gate: ExternalArmGate | None,
    command_norm: float,
) -> tuple[bool, bool]:
    """Resolve anchoring before a bridge command can become physical evidence."""

    anchor_active = _sensor_anchor_active(
        mode,
        motion_started=motion_started,
        external_arm_gate=external_arm_gate,
    )
    external_arm_ready = external_arm_gate is None or external_arm_gate.acknowledged
    release_warmup = mode == "warmup" and bool(driving) and external_arm_ready
    if release_warmup and not motion_started:
        # The locomotion policy must settle under a zero command before the
        # first navigation command. Releasing only on the first non-zero sample
        # cold-starts the controller and can make the robot collapse while the
        # planner is still aligning its heading.
        motion_started = True
        anchor_active = _sensor_anchor_active(
            mode,
            motion_started=motion_started,
            external_arm_gate=external_arm_gate,
        )
    if float(command_norm) <= 1e-4:
        return anchor_active, motion_started
    if anchor_active and not release_warmup:
        raise RuntimeError("native driver bridge produced motion while the MuJoCo Product remained physically anchored")
    motion_started = True
    return (
        _sensor_anchor_active(
            mode,
            motion_started=motion_started,
            external_arm_gate=external_arm_gate,
        ),
        motion_started,
    )


def _anchor_position_after_policy_settle(
    requested_start: Sequence[float] | None,
    settled_position: Sequence[float],
    *,
    policy_settled: bool,
) -> np.ndarray:
    """Keep requested map x/y while anchoring a policy robot at its physical standing height."""

    anchor = np.asarray(settled_position, dtype=np.float64).copy()
    if requested_start is None:
        return anchor
    requested = np.asarray(requested_start, dtype=np.float64)
    anchor[:2] = requested[:2]
    if not policy_settled:
        anchor[2] = requested[2]
    return anchor


def _resolve_policy_path_for_drive(drive_mode: str, value: str) -> Path | None:
    text = str(value or "").strip()
    if text:
        candidate = Path(text).expanduser()
        if not candidate.is_absolute():
            candidate = (ROOT / candidate).resolve()
        return candidate
    if str(drive_mode or "").strip().lower() != "policy":
        return None
    if DEFAULT_THUNDERV4_ONNX_POLICY.exists():
        return DEFAULT_THUNDERV4_ONNX_POLICY.resolve()
    return None


def _configure_policy_cpu_threads(
    policy_path: Path | None,
    requested_threads: int,
) -> dict[str, Any]:
    threads = int(requested_threads)
    if not 1 <= threads <= 8:
        raise ValueError("--policy-cpu-threads must be in [1, 8]")
    suffix = policy_path.suffix.lower() if policy_path is not None else ""
    if suffix == ".onnx":
        return {
            "backend": "onnxruntime",
            "requested_cpu_threads": threads,
            "active_cpu_threads": threads,
            "active_interop_threads": 1,
        }
    if suffix not in {".pt", ".pth", ".jit"}:
        return {
            "backend": "not_torchscript",
            "requested_cpu_threads": threads,
            "active_cpu_threads": None,
            "active_interop_threads": None,
        }

    import torch

    torch.set_num_threads(threads)
    active_interop = int(torch.get_num_interop_threads())
    if active_interop != 1:
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError as exc:
            raise RuntimeError("Torch inter-op threads must be configured before policy work starts") from exc

    return {
        "backend": "torchscript",
        "requested_cpu_threads": threads,
        "active_cpu_threads": int(torch.get_num_threads()),
        "active_interop_threads": int(torch.get_num_interop_threads()),
    }


def _lexical_wsl_unc_path(path: Path) -> str | None:
    """Convert a WSL UNC path without requiring the share to be reachable."""

    raw = str(path).replace("/", "\\")
    extended_prefix = "\\\\?\\UNC\\"
    if raw.lower().startswith(extended_prefix.lower()):
        raw = "\\\\" + raw[len(extended_prefix) :]
    if not raw.startswith("\\\\"):
        return None
    parts = [part for part in raw[2:].split("\\") if part]
    if len(parts) < 2 or parts[0].lower() not in {"wsl.localhost", "wsl$"}:
        return None
    return "/" + "/".join(parts[2:])


def _wsl_path(path: Path) -> str:
    if os.name != "nt":
        return str(path.resolve())
    # UNC conversion is lexical on purpose: Path.resolve() contacts the WSL
    # share and raises WinError 64 when the distribution is stopped.
    wsl_unc = _lexical_wsl_unc_path(path)
    if wsl_unc is not None:
        return wsl_unc
    resolved = path.resolve()
    wsl_unc = _lexical_wsl_unc_path(resolved)
    if wsl_unc is not None:
        return wsl_unc
    drive = resolved.drive.rstrip(":").lower()
    if not drive:
        raise ValueError(f"cannot convert path to WSL form: {resolved}")
    relative = resolved.as_posix().split(":", 1)[1].lstrip("/")
    return f"/mnt/{drive}/{relative}"


def _linux_binary_command(binary: Path, *args: str) -> list[str]:
    if os.name != "nt" or binary.suffix.lower() == ".exe":
        return [str(binary), *args]
    wsl = shutil.which("wsl.exe") or shutil.which("wsl")
    if not wsl:
        raise FileNotFoundError("wsl.exe is required to run Linux native DDS binaries from Windows")
    return [wsl, "-e", _wsl_path(binary), *args]


def _managed_wsl_command(
    command: list[str],
    pid_file: Path,
    *,
    clock_handshake: bool = False,
) -> list[str]:
    """Wrap a WSL launch so the Linux exec PID is externally owned."""

    if os.name != "nt" or len(command) < 3 or command[1] != "-e":
        return command
    pid_file.parent.mkdir(parents=True, exist_ok=True)
    pid_file.unlink(missing_ok=True)
    if clock_handshake:
        script = (
            'pid_file="$1"; shift; echo "$$" > "$pid_file"; '
            'printf "LINGTU_CLOCK_READY\\n"; '
            f'i=0; while [ "$i" -lt {DEFAULT_NATIVE_CLOCK_SYNC_SAMPLES} ]; do '
            "IFS= read -r request || exit 71; "
            '[ "$request" = "LINGTU_CLOCK_SAMPLE" ] || exit 72; '
            "date +%s%N; i=$((i + 1)); done; "
            "IFS= read -r request || exit 73; "
            '[ "$request" = "LINGTU_CLOCK_START" ] || exit 74; '
            'exec "$@" >/dev/null'
        )
    else:
        script = 'pid_file="$1"; shift; echo "$$" > "$pid_file"; exec "$@"'
    return [
        command[0],
        "-e",
        "bash",
        "-lc",
        script,
        "lingtu-managed",
        _wsl_path(pid_file),
        *command[2:],
    ]


def _read_process_line(
    stream: Any,
    *,
    timeout_s: float,
    label: str,
) -> bytes:
    """Read one child-process line without allowing a failed handshake to hang."""

    results: queue.Queue[bytes | BaseException] = queue.Queue(maxsize=1)

    def read_line() -> None:
        try:
            results.put(stream.readline())
        except BaseException as exc:  # pragma: no cover - platform pipe failure
            results.put(exc)

    threading.Thread(target=read_line, daemon=True).start()
    try:
        value = results.get(timeout=max(0.01, float(timeout_s)))
    except queue.Empty as exc:
        raise RuntimeError(f"{label} timed out") from exc
    if isinstance(value, BaseException):
        raise RuntimeError(f"{label} failed: {value}") from value
    if not value:
        raise RuntimeError(f"{label} closed before response")
    return value


def _synchronize_managed_native_clock(
    process: subprocess.Popen[bytes],
) -> dict[str, Any]:
    """Synchronize after WSL startup and before binary sensor records begin."""

    if process.stdin is None or process.stdout is None:
        raise RuntimeError("native clock handshake pipes unavailable")
    ready = _read_process_line(
        process.stdout,
        timeout_s=10.0,
        label="native clock ready handshake",
    ).strip()
    if ready != b"LINGTU_CLOCK_READY":
        raise RuntimeError(f"invalid native clock ready handshake: {ready!r}")

    samples: list[tuple[int, int, int]] = []
    for _ in range(DEFAULT_NATIVE_CLOCK_SYNC_SAMPLES):
        local_before_ns = time.time_ns()
        process.stdin.write(b"LINGTU_CLOCK_SAMPLE\n")
        process.stdin.flush()
        response = _read_process_line(
            process.stdout,
            timeout_s=2.0,
            label="native clock sample handshake",
        ).strip()
        local_after_ns = time.time_ns()
        try:
            native_wall_ns = int(response)
        except ValueError as exc:
            raise RuntimeError(f"invalid native clock sample: {response!r}") from exc
        samples.append((local_before_ns, native_wall_ns, local_after_ns))

    alignment = _select_native_wall_clock_alignment(samples)
    process.stdin.write(b"LINGTU_CLOCK_START\n")
    process.stdin.flush()
    process.stdout.close()
    return {
        **alignment,
        "source": "managed_native_process_handshake",
    }


def _read_linux_pid(pid_file: Path, timeout_s: float = 2.0) -> int | None:
    deadline = time.monotonic() + max(0.0, timeout_s)
    while time.monotonic() <= deadline:
        try:
            pid = int(pid_file.read_text(encoding="ascii").strip())
            if pid > 0:
                return pid
        except (OSError, ValueError):
            pass
        time.sleep(0.02)
    return None


def _wsl_pid_alive(pid: int | None) -> bool:
    if os.name != "nt" or pid is None or pid <= 0:
        return False
    wsl = shutil.which("wsl.exe") or shutil.which("wsl")
    if not wsl:
        return False
    try:
        proc = subprocess.run(
            [wsl, "-e", "kill", "-0", str(pid)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=WSL_CONTROL_TIMEOUT_S,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        # Cleanup verification must fail closed. An unavailable WSL control
        # path is not evidence that the Linux process exited.
        return True
    return proc.returncode == 0


def _signal_wsl_pid(pid: int | None, signal_name: str) -> bool:
    if os.name != "nt" or pid is None or pid <= 0:
        return False
    wsl = shutil.which("wsl.exe") or shutil.which("wsl")
    if not wsl:
        return False
    try:
        proc = subprocess.run(
            [wsl, "-e", "kill", f"-{signal_name}", str(pid)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=WSL_CONTROL_TIMEOUT_S,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return proc.returncode == 0


def _wait_wsl_pid_exit(pid: int | None, timeout_s: float) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_s)
    while time.monotonic() <= deadline:
        if not _wsl_pid_alive(pid):
            return True
        time.sleep(0.05)
    return not _wsl_pid_alive(pid)


def _terminate_wsl_pid(
    pid: int | None,
    *,
    term_timeout_s: float = 3.0,
    kill_timeout_s: float = 2.0,
) -> dict[str, Any]:
    """Terminate one explicitly owned Linux PID and return auditable evidence."""

    if os.name != "nt" or pid is None or pid <= 0:
        return {
            "linux_pid": pid,
            "owned_pid": False,
            "alive_before_cleanup": False,
            "term_sent": False,
            "kill_sent": False,
            "alive_after_cleanup": False,
            "clean": os.name != "nt",
            "errors": ["linux_pid_handshake_missing"] if os.name == "nt" else [],
        }

    errors: list[str] = []
    alive_before = _wsl_pid_alive(pid)
    term_sent = False
    kill_sent = False
    if alive_before:
        term_sent = _signal_wsl_pid(pid, "TERM")
        if not term_sent:
            errors.append("term_signal_failed")
        if not _wait_wsl_pid_exit(pid, term_timeout_s):
            kill_sent = _signal_wsl_pid(pid, "KILL")
            if not kill_sent:
                errors.append("kill_signal_failed")
            _wait_wsl_pid_exit(pid, kill_timeout_s)
    alive_after = _wsl_pid_alive(pid)
    if alive_after:
        errors.append("linux_process_still_alive")
    return {
        "linux_pid": pid,
        "owned_pid": True,
        "alive_before_cleanup": alive_before,
        "term_sent": term_sent,
        "kill_sent": kill_sent,
        "alive_after_cleanup": alive_after,
        "clean": not alive_after,
        "errors": errors,
    }


def _publisher_process_kind(process: subprocess.Popen[Any]) -> str:
    kind = str(getattr(process, "_lingtu_publisher_kind", "") or "").strip().lower()
    if kind in {"local", "wsl"}:
        return kind
    return "wsl" if getattr(process, "_lingtu_linux_pid", None) else "local"


def _terminate_local_publisher(
    process: subprocess.Popen[Any],
    *,
    term_timeout_s: float = 3.0,
    kill_timeout_s: float = 2.0,
) -> dict[str, Any]:
    pid = int(getattr(process, "_lingtu_owned_pid", getattr(process, "pid", 0)) or 0)
    errors: list[str] = []
    try:
        alive_before = process.poll() is None
    except OSError as exc:
        alive_before = True
        errors.append(f"local_poll_failed:{type(exc).__name__}:{exc}")
    term_sent = False
    kill_sent = False
    if alive_before:
        try:
            process.terminate()
            term_sent = True
            process.wait(timeout=max(0.1, float(term_timeout_s)))
        except subprocess.TimeoutExpired:
            try:
                process.kill()
                kill_sent = True
                process.wait(timeout=max(0.1, float(kill_timeout_s)))
            except (OSError, subprocess.TimeoutExpired) as exc:
                errors.append(f"local_kill_failed:{type(exc).__name__}:{exc}")
        except OSError as exc:
            errors.append(f"local_terminate_failed:{type(exc).__name__}:{exc}")
    try:
        alive_after = process.poll() is None
    except OSError as exc:
        alive_after = True
        errors.append(f"local_poll_failed:{type(exc).__name__}:{exc}")
    if alive_after:
        errors.append("local_process_still_alive")
    return {
        "process_kind": "local",
        "pid": pid,
        "owned_pid": pid > 0,
        "alive_before_cleanup": alive_before,
        "term_sent": term_sent,
        "kill_sent": kill_sent,
        "alive_after_cleanup": alive_after,
        "clean": not alive_after and not errors,
        "errors": errors,
    }


def _terminate_native_publisher_process(
    process: subprocess.Popen[Any],
) -> dict[str, Any]:
    if _publisher_process_kind(process) == "local":
        return _terminate_local_publisher(process)
    cleanup = _terminate_wsl_pid(getattr(process, "_lingtu_linux_pid", None))
    cleanup.update(
        {
            "process_kind": "wsl",
            "pid": cleanup.get("linux_pid"),
        }
    )
    return cleanup


def _stop_relay(process: subprocess.Popen[Any], timeout_s: float = 2.0) -> list[str]:
    """Stop the Windows WSL relay without assuming it owns the Linux child."""

    errors: list[str] = []
    if process.poll() is not None:
        return errors
    try:
        process.terminate()
        process.wait(timeout=max(0.1, timeout_s))
    except subprocess.TimeoutExpired:
        try:
            process.kill()
            process.wait(timeout=max(0.1, timeout_s))
        except (OSError, subprocess.TimeoutExpired) as exc:
            errors.append(f"relay_kill_failed:{type(exc).__name__}:{exc}")
    except OSError as exc:
        errors.append(f"relay_terminate_failed:{type(exc).__name__}:{exc}")
    return errors


def _resolve_driver_bridge_bin(value: str) -> Path:
    raw = str(value or "").strip()
    if not raw:
        raise FileNotFoundError(
            "--driver-bridge-bin is required for --command-source dds; "
            "the Product acceptance runner must pass its verified artifact"
        )
    candidate = Path(raw).expanduser()
    if not candidate.is_file():
        raise FileNotFoundError(f"MuJoCo driver bridge binary not found: {candidate}")
    return candidate.resolve()


def _wait_for_driver_bridge_transport(
    process: subprocess.Popen[Any],
    ready_file: Path,
    *,
    timeout_s: float = DEFAULT_DRIVER_BRIDGE_TRANSPORT_READY_S,
) -> None:
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    last_value = ""
    while time.monotonic() <= deadline:
        try:
            if ready_file.exists():
                last_value = ready_file.read_text(encoding="utf-8").strip()
                if last_value == "ready":
                    return
        except OSError as exc:
            last_value = f"{type(exc).__name__}: {exc}"
        returncode = process.poll()
        if returncode is not None:
            raise RuntimeError(f"native driver bridge exited before transport readiness with return code {returncode}")
        time.sleep(0.02)
    detail = last_value or "ready marker not written"
    raise RuntimeError(f"native driver bridge transport readiness timed out after {float(timeout_s):.1f}s ({detail})")


def _linear_command_direction(
    vx: float,
    vy: float,
    *,
    epsilon: float = 1e-4,
) -> str:
    """Classify translational body-frame intent for acceptance diagnostics."""

    if math.hypot(float(vx), float(vy)) <= float(epsilon):
        return "idle"
    if float(vx) > float(epsilon):
        return "forward"
    if float(vx) < -float(epsilon):
        return "reverse"
    return "lateral"


@dataclass(frozen=True)
class DriverBridgeCommand:
    bridge_command_seq: int
    kind: str
    producer_boot_id: str
    output_sequence: int
    walk_x: float
    walk_y: float
    walk_z: float


@dataclass(frozen=True)
class PreparedDriverBridgeStep:
    velocity: VelocityCommand
    protocol: DriverBridgeCommand | None

    def __iter__(self):
        yield self.velocity
        yield self.protocol


def _safe_protocol_token(value: str, *, allow_empty: bool = False) -> bool:
    if not value:
        return allow_empty
    if len(value) > 128 or not value.isascii() or not value[0].isalnum():
        return False
    return all(character.isalnum() or character in "._:@-" for character in value)


def _parse_positive_protocol_int(value: str, field: str) -> int:
    if not value or not value.isascii() or not value.isdecimal():
        raise ValueError(f"{field} must be a decimal integer")
    parsed = int(value)
    if parsed <= 0 or parsed > (1 << 64) - 1:
        raise ValueError(f"{field} must be a positive uint64")
    return parsed


def _parse_protocol_uint64(value: str, field: str) -> int:
    if not value or not value.isascii() or not value.isdecimal():
        raise ValueError(f"{field} must be a decimal integer")
    parsed = int(value)
    if parsed < 0 or parsed > (1 << 64) - 1:
        raise ValueError(f"{field} must be a uint64")
    return parsed


def _driver_producer_matches_host(producer: str, expected_host_boot_id: str) -> bool:
    if (
        not _safe_protocol_token(producer)
        or len(producer) >= 128
        or not producer.startswith(f"{expected_host_boot_id}:")
    ):
        return False
    suffix = producer[len(expected_host_boot_id) + 1 :]
    fields = suffix.split(":")
    if len(fields) != 2:
        return False
    try:
        _parse_positive_protocol_int(fields[0], "producer_pid")
        _parse_positive_protocol_int(fields[1], "producer_start_boottime_ns")
    except ValueError:
        return False
    return True


def _parse_protocol_float(value: str, field: str) -> float:
    if (
        not value.isascii()
        or re.fullmatch(
            r"[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?",
            value,
        )
        is None
    ):
        raise ValueError(f"{field} must be a finite decimal")
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ValueError(f"{field} must be a finite decimal") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{field} must be finite")
    return parsed


def _format_protocol_float(value: float) -> str:
    if not math.isfinite(value):
        raise ValueError("driver bridge protocol values must be finite")
    if value == 0.0:
        return "0"
    return format(value, ".17g")


class NativeDriverBridge:
    """Own the physical MuJoCo side of the native driver V2 boundary."""

    _COMMAND_PREFIX = "LT_DRIVER_COMMAND_V2"
    _READY_PREFIX = "LT_DRIVER_READY_V2"
    _FAULT_PREFIX = "LT_DRIVER_FAULT_V2"
    _STOPPED_PREFIX = "LT_DRIVER_STOPPED_V2"
    _PID_PREFIX = "LT_PID_V1"
    _KINDS = {
        "activation_zero",
        "nav",
        "deactivate_zero",
        "writer_fault_zero",
        "safety_zero",
    }
    _FAULTS = {
        "protocol_violation",
        "controller_eof",
        "heartbeat_timeout",
        "apply_timeout",
        "writer_missing",
        "writer_ambiguous",
        "command_sequence_overflow",
    }

    def __init__(
        self,
        *,
        binary: Path,
        domain_id: int,
        expected_host_boot_id: str,
        max_linear_mps: float,
        max_angular_rps: float,
        pid_file: Path | None = None,
        command_timeout_ms: int = 200,
        heartbeat_timeout_ms: int = 500,
        apply_timeout_ms: int = 500,
    ) -> None:
        self.binary = binary.resolve()
        self.domain_id = int(domain_id)
        self.expected_host_boot_id = str(expected_host_boot_id)
        self.max_linear_mps = float(max_linear_mps)
        self.max_angular_rps = float(max_angular_rps)
        if not _safe_protocol_token(self.expected_host_boot_id):
            raise ValueError("expected_host_boot_id must be a safe non-empty token")
        if not math.isfinite(self.max_linear_mps) or self.max_linear_mps <= 0.0:
            raise ValueError("max_linear_mps must be finite and positive")
        if not math.isfinite(self.max_angular_rps) or self.max_angular_rps <= 0.0:
            raise ValueError("max_angular_rps must be finite and positive")
        timeouts = {
            "command_timeout_ms": command_timeout_ms,
            "heartbeat_timeout_ms": heartbeat_timeout_ms,
            "apply_timeout_ms": apply_timeout_ms,
        }
        for name, value in timeouts.items():
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        self.command_timeout_ms = command_timeout_ms
        self.heartbeat_timeout_ms = heartbeat_timeout_ms
        self.apply_timeout_ms = apply_timeout_ms
        self.bridge_boot_id = secrets.token_hex(16)
        self.controller_boot_id = secrets.token_hex(16)
        self._condition = threading.Condition()
        self._control_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._commands: deque[DriverBridgeCommand] = deque()
        self._inflight: DriverBridgeCommand | None = None
        self._current_velocity = (0.0, 0.0, 0.0)
        self._control_seq = 1
        self._last_command_seq = 0
        self._last_step_seq = 0
        self._heartbeat_refreshes = 0
        self._last_ready_sequence = 0
        self._last_ready_producer_boot_id = ""
        self._last_ready_output_sequence = 0
        self._ready_observed = False
        self._observed_nav_ack_sequence = 0
        self._observed_nav_ack_producer_boot_id = ""
        self._observed_nav_ack_output_sequence = 0
        self._ready = False
        self._fault = ""
        self._reported_pid: int | None = None
        self._deactivate_requested = False
        self._deactivate_applied = False
        self._deactivate_command_seq = 0
        self._deactivate_applied_step_seq = 0
        self._stopped_evidence: dict[str, Any] = {}
        self._waited_clean_exit = False
        self._closed = False
        self._samples = 0
        self._nonzero_samples = 0
        self._forward_linear_samples = 0
        self._reverse_linear_samples = 0
        self._lateral_linear_samples = 0
        self._parse_errors = 0
        self._stderr_tail: list[str] = []
        self._failed_before_close = False
        self._terminated_by_parent = False
        self._process_kind = "local"
        self._owned_pid: int | None = None
        self._linux_pid: int | None = None
        self._cleanup: dict[str, Any] = {}
        requested_pid_file = pid_file.expanduser().resolve() if pid_file is not None else None
        runtime_dir = (
            requested_pid_file.parent
            if requested_pid_file is not None
            else (ROOT / "artifacts" / "mujoco_native_dds").resolve()
        )
        runtime_dir.mkdir(parents=True, exist_ok=True)
        runtime_stem = (
            requested_pid_file.stem if requested_pid_file is not None else f"driver_bridge_{os.getpid()}_{id(self)}"
        )
        self._ready_file = (runtime_dir / f"{runtime_stem}.ready").resolve()
        self._ready_file.unlink(missing_ok=True)
        ready_argument = (
            _wsl_path(self._ready_file)
            if os.name == "nt" and self.binary.suffix.lower() != ".exe"
            else str(self._ready_file)
        )
        command = _linux_binary_command(
            self.binary,
            "--domain-id",
            str(self.domain_id),
            "--ready-file",
            ready_argument,
            "--bridge-boot-id",
            self.bridge_boot_id,
            "--expected-host-boot-id",
            self.expected_host_boot_id,
            "--max-linear-mps",
            _format_protocol_float(self.max_linear_mps),
            "--max-angular-rps",
            _format_protocol_float(self.max_angular_rps),
            "--command-timeout-ms",
            str(self.command_timeout_ms),
            "--heartbeat-timeout-ms",
            str(self.heartbeat_timeout_ms),
            "--apply-timeout-ms",
            str(self.apply_timeout_ms),
        )
        managed_wsl = os.name == "nt" and len(command) >= 3 and command[1] == "-e"
        self._process_kind = "wsl" if managed_wsl else "local"
        self._pid_file = requested_pid_file
        if managed_wsl:
            if self._pid_file is None:
                self._pid_file = (runtime_dir / f"{runtime_stem}.pid").resolve()
            self._pid_file.parent.mkdir(parents=True, exist_ok=True)
            self._pid_file.unlink(missing_ok=True)
            launch_command = _managed_wsl_command(command, self._pid_file)
        else:
            self._pid_file = None
            launch_command = command
        self.process = subprocess.Popen(
            launch_command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="strict",
            bufsize=1,
        )
        self.process._lingtu_publisher_kind = self._process_kind
        self.process._lingtu_owned_pid = int(self.process.pid)
        self.process._lingtu_linux_pid = None
        self._owned_pid = int(self.process.pid)
        self._stdout_thread = threading.Thread(target=self._read_stdout, daemon=True)
        self._stderr_thread = threading.Thread(target=self._read_stderr, daemon=True)
        self._stdout_thread.start()
        self._stderr_thread.start()
        if managed_wsl and self._pid_file is not None:
            linux_pid = _read_linux_pid(self._pid_file, timeout_s=10.0)
            if linux_pid is None:
                relay_errors = _stop_relay(self.process)
                self._stdout_thread.join(timeout=1.0)
                self._stderr_thread.join(timeout=1.0)
                detail = ";".join(relay_errors) if relay_errors else "pid_file_not_written"
                raise RuntimeError(f"driver bridge Linux PID handshake failed: {detail}")
            with self._condition:
                self._linux_pid = linux_pid
                self._owned_pid = linux_pid
                if self._reported_pid is not None and self._reported_pid != linux_pid:
                    self._fault = "pid_mismatch"
            self.process._lingtu_owned_pid = linux_pid
            self.process._lingtu_linux_pid = linux_pid
        try:
            _wait_for_driver_bridge_transport(
                self.process,
                self._ready_file,
                timeout_s=DEFAULT_DRIVER_BRIDGE_TRANSPORT_READY_S,
            )
            self._write_control_line(
                f"LT_DRIVER_ACTIVATE_V2\t{self.bridge_boot_id}\t{self.controller_boot_id}\t{self._control_seq}"
            )
        except BaseException:
            _terminate_native_publisher_process(self.process)
            if managed_wsl:
                _stop_relay(self.process)
            self._stdout_thread.join(timeout=1.0)
            self._stderr_thread.join(timeout=1.0)
            try:
                self._ready_file.unlink(missing_ok=True)
            except OSError:
                pass
            raise

    @classmethod
    def parse_pid_line(cls, line: str) -> int | None:
        parts = line.rstrip("\r\n").split("\t")
        if len(parts) != 2 or parts[0] != cls._PID_PREFIX:
            return None
        try:
            pid = int(parts[1])
        except ValueError:
            return None
        return pid if pid > 0 else None

    def _write_control_line(self, line: str) -> None:
        if len(line.encode("ascii")) > 512 or "\n" in line or "\r" in line:
            raise ValueError("driver bridge control line is invalid")
        stream = self.process.stdin
        if stream is None or stream.closed:
            raise RuntimeError("native driver bridge stdin is closed")
        with self._write_lock:
            stream.write(line + "\n")
            stream.flush()

    @classmethod
    def _parse_command_parts(
        cls,
        parts: list[str],
        *,
        bridge_boot_id: str,
        controller_boot_id: str,
    ) -> DriverBridgeCommand:
        if len(parts) != 10:
            raise ValueError("LT_DRIVER_COMMAND_V2 field count mismatch")
        if parts[1] != bridge_boot_id or parts[2] != controller_boot_id:
            raise ValueError("driver bridge command identity mismatch")
        sequence = _parse_positive_protocol_int(parts[3], "bridge_command_seq")
        kind = parts[4]
        if kind not in cls._KINDS:
            raise ValueError("unknown driver bridge command kind")
        producer = "" if parts[5] == "-" else parts[5]
        output_sequence = _parse_protocol_uint64(parts[6], "output_sequence")
        walk = tuple(
            _parse_protocol_float(parts[index], field) for index, field in ((7, "walk_x"), (8, "walk_y"), (9, "walk_z"))
        )
        if any(abs(value) > 1.0 for value in walk):
            raise ValueError("normalized driver walk exceeds [-1, 1]")
        if kind == "nav":
            if not _safe_protocol_token(producer) or output_sequence <= 0:
                raise ValueError("nav command output identity is incomplete")
        elif producer or output_sequence != 0 or any(value != 0.0 or math.copysign(1.0, value) < 0.0 for value in walk):
            raise ValueError("internal driver zero is not exact")
        return DriverBridgeCommand(
            sequence,
            kind,
            producer,
            output_sequence,
            walk[0],
            walk[1],
            walk[2],
        )

    @classmethod
    def parse_stdout_line(
        cls,
        line: str,
        *,
        bridge_boot_id: str,
        controller_boot_id: str,
    ) -> DriverBridgeCommand:
        raw = line.rstrip("\r\n")
        if not raw or len(raw.encode("utf-8")) > 512:
            raise ValueError("driver bridge stdout line is empty or oversized")
        parts = raw.split("\t")
        if not parts or parts[0] != cls._COMMAND_PREFIX:
            raise ValueError("stdout record is not LT_DRIVER_COMMAND_V2")
        return cls._parse_command_parts(
            parts,
            bridge_boot_id=bridge_boot_id,
            controller_boot_id=controller_boot_id,
        )

    def _parse_command(self, parts: list[str]) -> DriverBridgeCommand:
        return self._parse_command_parts(
            parts,
            bridge_boot_id=self.bridge_boot_id,
            controller_boot_id=self.controller_boot_id,
        )

    def _handle_stdout_line(self, line: str) -> None:
        raw = line.rstrip("\r\n")
        if not raw or len(raw.encode("utf-8")) > 512:
            raise ValueError("driver bridge stdout line is empty or oversized")
        pid = self.parse_pid_line(raw)
        if pid is not None:
            with self._condition:
                expected = self._linux_pid if self._process_kind == "wsl" else int(self.process.pid)
                if expected is not None and pid != expected:
                    raise ValueError("driver bridge PID identity mismatch")
                self._reported_pid = pid
            return
        parts = raw.split("\t")
        prefix = parts[0]
        with self._condition:
            if prefix == self._COMMAND_PREFIX:
                command = self._parse_command(parts)
                if command.kind == "nav" and not _driver_producer_matches_host(
                    command.producer_boot_id,
                    self.expected_host_boot_id,
                ):
                    raise ValueError("driver bridge command producer identity mismatch")
                if command.bridge_command_seq <= self._last_command_seq or self._commands or self._inflight:
                    raise ValueError("driver bridge command sequence is stale or concurrent")
                self._last_command_seq = command.bridge_command_seq
                self._commands.append(command)
                self._condition.notify_all()
                return
            if prefix == self._READY_PREFIX:
                if len(parts) != 6 or parts[1] != self.bridge_boot_id or parts[2] != self.controller_boot_id:
                    raise ValueError("driver bridge READY identity mismatch")
                accepted = _parse_positive_protocol_int(parts[3], "accepted_sequence")
                producer = "" if parts[4] == "-" else parts[4]
                output = _parse_protocol_uint64(parts[5], "accepted_output_sequence")
                if accepted <= self._last_ready_sequence:
                    raise ValueError("driver bridge READY sequence did not advance")
                if bool(producer) != bool(output) or (producer and not _safe_protocol_token(producer)):
                    raise ValueError("driver bridge READY output identity is incomplete")
                if producer and not _driver_producer_matches_host(
                    producer,
                    self.expected_host_boot_id,
                ):
                    raise ValueError("driver bridge READY producer identity mismatch")
                self._last_ready_sequence = accepted
                self._last_ready_producer_boot_id = producer
                self._last_ready_output_sequence = output
                self._ready_observed = True
                if producer:
                    self._observed_nav_ack_sequence = accepted
                    self._observed_nav_ack_producer_boot_id = producer
                    self._observed_nav_ack_output_sequence = output
                self._ready = True
                self._condition.notify_all()
                return
            if prefix == self._STOPPED_PREFIX:
                if len(parts) != 6 or parts[1] != self.bridge_boot_id or parts[2] != self.controller_boot_id:
                    raise ValueError("driver bridge STOPPED identity mismatch")
                command_seq = _parse_positive_protocol_int(parts[3], "bridge_command_seq")
                applied_step_seq = _parse_positive_protocol_int(parts[4], "applied_step_seq")
                if parts[5] != "deactivate_zero":
                    raise ValueError("driver bridge STOPPED kind mismatch")
                if not self._deactivate_requested or not self._deactivate_applied:
                    raise ValueError("driver bridge STOPPED arrived before physical deactivation")
                if command_seq != self._deactivate_command_seq or applied_step_seq != self._deactivate_applied_step_seq:
                    raise ValueError("driver bridge STOPPED physical evidence mismatch")
                if self._stopped_evidence:
                    raise ValueError("driver bridge STOPPED was duplicated")
                self._stopped_evidence = {
                    "bridge_boot_id": self.bridge_boot_id,
                    "controller_boot_id": self.controller_boot_id,
                    "bridge_command_seq": command_seq,
                    "applied_step_seq": applied_step_seq,
                    "kind": "deactivate_zero",
                }
                # STOPPED is a terminal safety state. Historical execution
                # evidence remains available separately, while current motion
                # authority and output acknowledgement are cleared.
                self._ready = False
                self._last_ready_sequence = 0
                self._last_ready_producer_boot_id = ""
                self._last_ready_output_sequence = 0
                self._condition.notify_all()
                return
            if prefix == self._FAULT_PREFIX:
                if (
                    len(parts) != 4
                    or parts[1] != self.bridge_boot_id
                    or parts[2]
                    not in {
                        self.controller_boot_id,
                        "-",
                    }
                    or parts[3] not in self._FAULTS
                ):
                    raise ValueError("driver bridge FAULT is malformed")
                self._fault = parts[3]
                self._ready = False
                self._condition.notify_all()
                return
        raise ValueError("unknown native driver bridge stdout record")

    def _read_stdout(self) -> None:
        stream = self.process.stdout
        if stream is None:
            return
        try:
            for line in stream:
                self._handle_stdout_line(line)
        except (UnicodeError, ValueError, OSError):
            with self._condition:
                self._parse_errors += 1
                self._fault = "protocol_violation"
                self._ready = False
                self._condition.notify_all()

    def _read_stderr(self) -> None:
        stream = self.process.stderr
        if stream is None:
            return
        for line in stream:
            with self._condition:
                self._stderr_tail.append(line.rstrip())
                del self._stderr_tail[:-20]

    def _raise_if_failed_locked(self) -> None:
        if self._fault:
            raise RuntimeError(f"native driver bridge fault: {self._fault}")
        returncode = self.process.poll()
        if returncode is not None and returncode != 0:
            self._failed_before_close = True
            raise RuntimeError(f"native driver bridge exited with return code {returncode}")

    def _write_control_or_fail(self, line: str) -> None:
        try:
            self._write_control_line(line)
        except (OSError, RuntimeError, UnicodeError, ValueError):
            with self._condition:
                self._failed_before_close = True
                self._ready = False
                self._condition.notify_all()
            raise

    def prepare_step(self, *, wait_for_command_s: float = 0.0) -> PreparedDriverBridgeStep:
        from sim.compat.engine.core.engine import VelocityCommand

        with self._condition:
            self._raise_if_failed_locked()
            if self._inflight is not None:
                raise RuntimeError("previous driver bridge command has not been physically applied")
            wait_s = max(0.0, float(wait_for_command_s))
            if not self._commands and (self._last_command_seq == 0 or wait_s > 0.0):
                deadline = time.monotonic() + max(
                    wait_s,
                    self.apply_timeout_ms / 1000.0 if self._last_command_seq == 0 else 0.0,
                )
                while not self._commands and not self._fault and self.process.poll() is None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    self._condition.wait(timeout=remaining)
                self._raise_if_failed_locked()
                if not self._commands and self._last_command_seq == 0:
                    raise RuntimeError("activation zero was not received before apply deadline")
            command = self._commands.popleft() if self._commands else None
            self._inflight = command
            if command is None:
                physical = self._current_velocity
            else:
                physical = (
                    command.walk_x * self.max_linear_mps,
                    command.walk_y * self.max_linear_mps,
                    command.walk_z * self.max_angular_rps,
                )
            return PreparedDriverBridgeStep(
                velocity=VelocityCommand(
                    linear_x=physical[0],
                    linear_y=physical[1],
                    angular_z=physical[2],
                ),
                protocol=command,
            )

    def heartbeat(self, step_seq: int) -> None:
        with self._control_lock:
            with self._condition:
                self._raise_if_failed_locked()
                if isinstance(step_seq, bool) or step_seq <= self._last_step_seq:
                    raise ValueError("controller step_seq must advance")
                self._last_step_seq = int(step_seq)
                self._control_seq += 1
                line = (
                    f"LT_DRIVER_HEARTBEAT_V2\t{self.bridge_boot_id}\t"
                    f"{self.controller_boot_id}\t{self._control_seq}\t{step_seq}"
                )
            self._write_control_or_fail(line)

    def refresh_heartbeat(self) -> bool:
        """Refresh controller liveness without claiming a new physics step."""

        with self._control_lock:
            with self._condition:
                self._raise_if_failed_locked()
                if self._last_step_seq <= 0 or self._deactivate_requested:
                    return False
                self._control_seq += 1
                self._heartbeat_refreshes += 1
                line = (
                    f"LT_DRIVER_HEARTBEAT_V2\t{self.bridge_boot_id}\t"
                    f"{self.controller_boot_id}\t{self._control_seq}\t{self._last_step_seq}"
                )
            self._write_control_or_fail(line)
        return True

    def complete_step(self, command: DriverBridgeCommand, *, step_seq: int) -> None:
        with self._control_lock:
            with self._condition:
                self._raise_if_failed_locked()
                if self._inflight != command:
                    raise ValueError("APPLIED command does not match the in-flight bridge command")
                if isinstance(step_seq, bool) or step_seq <= self._last_step_seq:
                    raise ValueError("controller step_seq must advance")
                producer = command.producer_boot_id or "-"
                applied_line = (
                    f"LT_DRIVER_APPLIED_V2\t{self.bridge_boot_id}\t{self.controller_boot_id}\t"
                    f"{command.bridge_command_seq}\t{command.kind}\t{producer}\t"
                    f"{command.output_sequence}\t{_format_protocol_float(command.walk_x)}\t"
                    f"{_format_protocol_float(command.walk_y)}\t"
                    f"{_format_protocol_float(command.walk_z)}\t{step_seq}"
                )
                self._current_velocity = (
                    command.walk_x * self.max_linear_mps,
                    command.walk_y * self.max_linear_mps,
                    command.walk_z * self.max_angular_rps,
                )
                self._inflight = None
                if command.kind == "nav":
                    self._samples += 1
                    if any(abs(value) > 1e-4 for value in self._current_velocity):
                        self._nonzero_samples += 1
                    direction = _linear_command_direction(
                        self._current_velocity[0],
                        self._current_velocity[1],
                    )
                    if direction == "forward":
                        self._forward_linear_samples += 1
                    elif direction == "reverse":
                        self._reverse_linear_samples += 1
                    elif direction == "lateral":
                        self._lateral_linear_samples += 1
                if command.kind == "deactivate_zero":
                    self._deactivate_applied = True
                    self._deactivate_command_seq = command.bridge_command_seq
                    self._deactivate_applied_step_seq = int(step_seq)
                self._last_step_seq = int(step_seq)
                self._control_seq += 1
                heartbeat_line = (
                    f"LT_DRIVER_HEARTBEAT_V2\t{self.bridge_boot_id}\t"
                    f"{self.controller_boot_id}\t{self._control_seq}\t{step_seq}"
                )
            self._write_control_or_fail(applied_line)
            self._write_control_or_fail(heartbeat_line)

    def begin_deactivate(self) -> None:
        with self._control_lock:
            with self._condition:
                self._raise_if_failed_locked()
                if self._deactivate_requested:
                    return
                if self._inflight is not None:
                    raise RuntimeError("cannot deactivate with an un-applied driver command")
                self._control_seq += 1
                line = f"LT_DRIVER_DEACTIVATE_V2\t{self.bridge_boot_id}\t{self.controller_boot_id}\t{self._control_seq}"
                self._deactivate_requested = True
            self._write_control_or_fail(line)

    def wait_stopped(self, *, timeout_s: float = 3.0) -> None:
        timeout_s = max(0.01, float(timeout_s))
        deadline = time.monotonic() + timeout_s
        with self._condition:
            if not self._deactivate_applied:
                raise RuntimeError("driver bridge cannot stop cleanly before physical deactivate zero")
            while not self._stopped_evidence and not self._fault:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(timeout=remaining)
            self._raise_if_failed_locked()
            if not self._stopped_evidence:
                raise RuntimeError("native driver bridge STOPPED evidence was not received")
        returncode = self.process.wait(timeout=max(0.01, deadline - time.monotonic()))
        if returncode != 0:
            self._failed_before_close = True
            raise RuntimeError(f"native driver bridge clean stop returned {returncode}")
        self._waited_clean_exit = True

    def stats(self) -> dict[str, Any]:
        with self._condition:
            return {
                "transport": "cpp_typed_dds_physical_bridge_v2",
                "topic": "/nav/cmd_vel",
                "dds_topic": "rt/nav/cmd_vel",
                "binary": str(self.binary),
                "bridge_boot_id": self.bridge_boot_id,
                "controller_boot_id": self.controller_boot_id,
                "transport_ready": True,
                "driver_ready": self._ready,
                "driver_ready_observed": self._ready_observed,
                "accepted_sequence": self._last_ready_sequence,
                "accepted_producer_boot_id": self._last_ready_producer_boot_id,
                "accepted_output_sequence": self._last_ready_output_sequence,
                "observed_output_ack": {
                    "accepted_sequence": self._observed_nav_ack_sequence,
                    "producer_boot_id": self._observed_nav_ack_producer_boot_id,
                    "output_sequence": self._observed_nav_ack_output_sequence,
                },
                "stopped_evidence": dict(self._stopped_evidence),
                "fault": self._fault,
                "samples": self._samples,
                "nonzero_samples": self._nonzero_samples,
                "forward_linear_samples": self._forward_linear_samples,
                "reverse_linear_samples": self._reverse_linear_samples,
                "lateral_linear_samples": self._lateral_linear_samples,
                "parse_errors": self._parse_errors,
                "heartbeat_refreshes": self._heartbeat_refreshes,
                "last_physical_step_seq": self._last_step_seq,
                "last_command": {
                    "vx": self._current_velocity[0],
                    "vy": self._current_velocity[1],
                    "wz": self._current_velocity[2],
                },
                "process_returncode": self.process.poll(),
                "process_kind": self._process_kind,
                "owned_pid": self._owned_pid,
                "linux_pid": self._linux_pid,
                "failed_before_close": self._failed_before_close,
                "terminated_by_parent": self._terminated_by_parent,
                "process_cleanup": dict(self._cleanup),
                "stderr_tail": list(self._stderr_tail),
            }

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        returncode = self.process.poll()
        if returncode is None:
            self._terminated_by_parent = True
            self._cleanup = _terminate_native_publisher_process(self.process)
        elif returncode != 0:
            self._failed_before_close = True
            self._cleanup = _terminate_native_publisher_process(self.process)
        else:
            self._cleanup = _terminate_native_publisher_process(self.process)
        if self.process.stdin is not None and not self.process.stdin.closed:
            self.process.stdin.close()
        if self._process_kind == "wsl":
            relay_errors = _stop_relay(self.process, timeout_s=3.0)
            if relay_errors:
                self._cleanup.setdefault("errors", []).extend(relay_errors)
                self._cleanup["clean"] = False
        self._stdout_thread.join(timeout=1.0)
        self._stderr_thread.join(timeout=1.0)
        try:
            self._ready_file.unlink(missing_ok=True)
        except OSError as exc:
            self._cleanup.setdefault("errors", []).append(
                f"driver_bridge_ready_file_cleanup_failed:{type(exc).__name__}:{exc}"
            )
            self._cleanup["clean"] = False
        if self._pid_file is not None:
            try:
                self._pid_file.unlink(missing_ok=True)
            except OSError as exc:
                self._cleanup.setdefault("errors", []).append(
                    f"driver_bridge_pid_cleanup_failed:{type(exc).__name__}:{exc}"
                )
                self._cleanup["clean"] = False
        physical_clean = (
            self._deactivate_applied
            and bool(self._stopped_evidence)
            and self._waited_clean_exit
            and self.process.poll() == 0
            and not self._terminated_by_parent
        )
        self._cleanup["clean"] = bool(self._cleanup.get("clean")) and physical_clean


class DriverHeartbeat:
    """Keep the controller lease alive while sensor work runs between physics steps."""

    def __init__(self, bridge: NativeDriverBridge) -> None:
        self._bridge = bridge
        self._period_s = bridge.heartbeat_timeout_ms / 3000.0
        self._stop = threading.Event()
        self._error: BaseException | None = None
        self._thread = threading.Thread(
            target=self._run,
            name="mujoco-driver-heartbeat",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.wait(self._period_s):
            try:
                self._bridge.refresh_heartbeat()
            except BaseException as exc:
                self._error = exc
                return

    def raise_if_failed(self) -> None:
        if self._error is not None:
            detail = f"{type(self._error).__name__}: {self._error}"
            raise RuntimeError(f"native driver heartbeat refresh failed: {detail}") from self._error

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=max(1.0, self._period_s * 2.0))
        if self._thread.is_alive():
            raise RuntimeError("native driver heartbeat thread did not stop")
        self.raise_if_failed()


def _step_with_driver_bridge(
    engine: Any,
    bridge: Any,
    prepared: Any,
    *,
    imu_period_s: float,
    step_seq: int,
) -> tuple[Any, int]:
    """Run one real controller tick before acknowledging its driver command."""

    velocity = prepared.velocity
    state = _step_engine_for_sensor_tick(engine, velocity, imu_period_s)
    return state, _ack_driver_bridge_step(bridge, prepared, step_seq=step_seq)


def _step_anchored_with_driver_bridge(
    engine: Any,
    bridge: Any,
    prepared: Any,
    *,
    imu_period_s: float,
    step_seq: int,
) -> tuple[Any, int]:
    """Advance anchored sensors and acknowledge the applied zero without running policy dynamics."""

    velocity = prepared.velocity
    command_norm = math.sqrt(
        float(velocity.linear_x) ** 2
        + float(velocity.linear_y) ** 2
        + float(velocity.angular_z) ** 2
    )
    if command_norm > 1e-4:
        raise RuntimeError("anchored driver step must be an exact physical stop")
    state = _step_static_engine_for_sensor_tick(engine, imu_period_s)
    return state, _ack_driver_bridge_step(bridge, prepared, step_seq=step_seq)


def _ack_driver_bridge_step(bridge: Any, prepared: Any, *, step_seq: int) -> int:
    """Publish physical-step evidence after the corresponding simulation step succeeded."""

    next_step_seq = int(step_seq) + 1
    protocol = getattr(prepared, "protocol", prepared)
    if protocol is None:
        bridge.heartbeat(next_step_seq)
    else:
        bridge.complete_step(protocol, step_seq=next_step_seq)
    return next_step_seq


def _deactivate_driver_bridge(
    engine: Any,
    bridge: NativeDriverBridge,
    *,
    imu_period_s: float,
    step_seq: int,
) -> int:
    """Physically apply the bridge's terminal zero before accepting clean exit."""

    prepared = bridge.prepare_step()
    if prepared.protocol is not None:
        _, step_seq = _step_with_driver_bridge(
            engine,
            bridge,
            prepared,
            imu_period_s=imu_period_s,
            step_seq=step_seq,
        )
    bridge.begin_deactivate()
    deadline = time.monotonic() + max(3.0, bridge.apply_timeout_ms / 1000.0)
    while True:
        remaining_s = deadline - time.monotonic()
        if remaining_s <= 0.0:
            raise RuntimeError("driver bridge physical deactivate zero timed out")
        prepared = bridge.prepare_step(wait_for_command_s=remaining_s)
        if prepared.protocol is None:
            raise RuntimeError("driver bridge did not issue a physical deactivate zero")
        if prepared.protocol.kind != "deactivate_zero":
            raise RuntimeError("driver bridge emitted a non-deactivate command after shutdown began")
        _, step_seq = _step_with_driver_bridge(
            engine,
            bridge,
            prepared,
            imu_period_s=imu_period_s,
            step_seq=step_seq,
        )
        break
    bridge.wait_stopped(timeout_s=3.0)
    return step_seq


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
    motion: dict[str, Any] | None = None,
    scan_time_profile: str = "",
    timestamp_clock: str = "",
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
        "motion": motion or {},
        "scan_time_profile": scan_time_profile,
        "timestamp_clock": timestamp_clock,
        "require_slam_output": bool(require_slam_output),
        "required_slam_output_topics": list(REQUIRED_SLAM_OUTPUT_TOPICS),
        "sensor_topics": {
            TOPICS.lidar_scan: {"dds_topic": "rt/lidar/raw_frame", "frame_id": LIDAR_FRAME_ID},
            TOPICS.imu: {"dds_topic": "rt/imu/raw", "frame_id": IMU_FRAME_ID},
            TOPICS.odom_prior: {"dds_topic": "rt/slam/odom_prior", "frame_id": "odom"},
        },
        "remaining_gaps": list(remaining_gaps),
        "error": error,
    }


def _remaining_gaps(
    *,
    sensor_counts: Counter[str],
    slam_counts: Counter[str],
    require_slam_output: bool,
    require_odom_prior: bool = True,
    drive_mode: str = "",
    allow_kinematic_fastlio_acceptance: bool = False,
) -> list[str]:
    gaps: list[str] = []
    for topic in (TOPICS.lidar_scan, TOPICS.imu):
        if sensor_counts.get(topic, 0) <= 0:
            gaps.append(f"sensor_not_published:{topic}")
    if require_odom_prior and sensor_counts.get(TOPICS.odom_prior, 0) <= 0:
        gaps.append(f"sensor_not_published:{TOPICS.odom_prior}")
    if require_slam_output:
        if str(drive_mode or "").strip().lower() == "kinematic" and not bool(allow_kinematic_fastlio_acceptance):
            gaps.append("kinematic_fastlio_acceptance_disabled")
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
    pose = (status.get("odometry") or {}).get("pose") or {}
    coords = {axis: float(pose.get(axis) or 0.0) for axis in ("x", "y", "z")}
    if any(not math.isfinite(value) for value in coords.values()):
        gaps.append("native_slam_odom_not_finite")
    elif any(abs(value) > float(max_odom_abs_m) for value in coords.values()):
        gaps.append(f"native_slam_odom_out_of_bounds:x={coords['x']:.3f},y={coords['y']:.3f},z={coords['z']:.3f}")
    if math.isfinite(coords["z"]) and abs(coords["z"]) > float(max_odom_z_abs_m):
        gaps.append(f"native_slam_odom_z_out_of_bounds:{coords['z']:.3f}")
    return gaps


def _slam_odom_pose(status: dict[str, Any]) -> dict[str, Any]:
    return (status.get("odometry") or {}).get("pose") or {}


def _slam_odom_xy_m(status: dict[str, Any], *, reference_xy: Any = None) -> float:
    pose = _slam_odom_pose(status)
    try:
        x = float(pose.get("x") or 0.0)
        y = float(pose.get("y") or 0.0)
    except (TypeError, ValueError):
        return float("nan")
    if reference_xy is not None and bool(status.get("odom_prior_enabled")):
        try:
            ref = np.asarray(reference_xy, dtype=np.float64)
            if ref.size >= 2:
                x -= float(ref[0])
                y -= float(ref[1])
        except (TypeError, ValueError):
            pass
    return float(math.hypot(x, y))


def _slam_odom_yaw_rad(status: dict[str, Any], *, reference_yaw: float | None = None) -> float:
    pose = _slam_odom_pose(status)
    try:
        qx = float(pose.get("qx") or 0.0)
        qy = float(pose.get("qy") or 0.0)
        qz = float(pose.get("qz") or 0.0)
        qw = float(pose.get("qw") if pose.get("qw") is not None else 1.0)
    except (TypeError, ValueError):
        return float("nan")
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = float(math.atan2(siny_cosp, cosy_cosp))
    if reference_yaw is not None and bool(status.get("odom_prior_enabled")):
        return float(angle_delta_rad(yaw, float(reference_yaw)))
    return yaw


def _multiply_quaternions_xyzw(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = [float(value) for value in lhs]
    rx, ry, rz, rw = [float(value) for value in rhs]
    result = np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(result))
    return result / norm if norm > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


def _slam_map_pose(status: dict[str, Any]) -> dict[str, float]:
    transform = status.get("map_odom_tf") or {}
    odom_pose = _slam_odom_pose(status)
    if transform.get("valid") is not True or not odom_pose:
        return {}
    try:
        map_translation = np.array(
            [transform["tx"], transform["ty"], transform["tz"]],
            dtype=np.float64,
        )
        map_quaternion = np.array(
            [transform["qx"], transform["qy"], transform["qz"], transform["qw"]],
            dtype=np.float64,
        )
        odom_translation = np.array(
            [odom_pose["x"], odom_pose["y"], odom_pose["z"]],
            dtype=np.float64,
        )
        odom_quaternion = np.array(
            [odom_pose["qx"], odom_pose["qy"], odom_pose["qz"], odom_pose["qw"]],
            dtype=np.float64,
        )
    except (KeyError, TypeError, ValueError):
        return {}
    if not all(
        np.isfinite(values).all() for values in (map_translation, map_quaternion, odom_translation, odom_quaternion)
    ):
        return {}
    position = map_translation + quat_xyzw_to_matrix(map_quaternion) @ odom_translation
    quaternion = _multiply_quaternions_xyzw(map_quaternion, odom_quaternion)
    return {
        "x": float(position[0]),
        "y": float(position[1]),
        "z": float(position[2]),
        "qx": float(quaternion[0]),
        "qy": float(quaternion[1]),
        "qz": float(quaternion[2]),
        "qw": float(quaternion[3]),
    }


def _motion_report(
    *,
    sim_start_position: Any,
    sim_end_position: Any,
    sim_start_yaw: float | None,
    sim_end_yaw: float | None,
    sim_path_length_xy_m: float = 0.0,
    sim_abs_yaw_delta_rad: float = 0.0,
    slam_status: dict[str, Any],
    min_sim_motion_for_odom_check_m: float,
    min_slam_motion_ratio: float,
    max_slam_motion_ratio: float,
    min_sim_yaw_for_odom_check_rad: float,
    max_slam_yaw_error_rad: float,
) -> dict[str, Any]:
    if sim_start_position is None or sim_end_position is None:
        return {
            "available": False,
            "reason": "mujoco_state_not_sampled",
        }
    start = np.asarray(sim_start_position, dtype=np.float64)
    end = np.asarray(sim_end_position, dtype=np.float64)
    delta = end[:3] - start[:3]
    sim_xy_m = float(math.hypot(float(delta[0]), float(delta[1])))
    slam_xy_m = _slam_odom_xy_m(slam_status, reference_xy=start[:2])
    slam_yaw_rad = _slam_odom_yaw_rad(slam_status, reference_yaw=sim_start_yaw)
    slam_map_pose = _slam_map_pose(slam_status)
    slam_map_xy_error_m = None
    slam_map_yaw_error_rad = None
    if slam_map_pose:
        slam_map_xy_error_m = float(
            math.hypot(
                float(slam_map_pose["x"]) - float(end[0]),
                float(slam_map_pose["y"]) - float(end[1]),
            )
        )
        if sim_end_yaw is not None:
            map_yaw = yaw_from_quat_xyzw(
                np.array(
                    [
                        slam_map_pose["qx"],
                        slam_map_pose["qy"],
                        slam_map_pose["qz"],
                        slam_map_pose["qw"],
                    ],
                    dtype=np.float64,
                )
            )
            slam_map_yaw_error_rad = float(angle_delta_rad(map_yaw, float(sim_end_yaw)))
    ratio = (slam_xy_m / sim_xy_m) if sim_xy_m > 1e-9 and math.isfinite(slam_xy_m) else None
    yaw_delta = None
    yaw_error = None
    if sim_start_yaw is not None and sim_end_yaw is not None:
        yaw_delta = float(angle_delta_rad(float(sim_end_yaw), float(sim_start_yaw)))
        if math.isfinite(slam_yaw_rad):
            yaw_error = float(angle_delta_rad(float(slam_yaw_rad), yaw_delta))
    return {
        "available": True,
        "sim_start_xyz": [float(v) for v in start[:3]],
        "sim_end_xyz": [float(v) for v in end[:3]],
        "sim_delta_xyz": [float(v) for v in delta[:3]],
        "sim_xy_m": sim_xy_m,
        "sim_path_length_xy_m": float(sim_path_length_xy_m),
        "sim_yaw_delta_rad": yaw_delta,
        "sim_abs_yaw_delta_rad": float(sim_abs_yaw_delta_rad),
        "slam_odom_xy_m": slam_xy_m,
        "slam_to_sim_xy_ratio": ratio,
        "slam_odom_yaw_rad": slam_yaw_rad,
        "slam_to_sim_yaw_error_rad": yaw_error,
        "slam_map_pose": slam_map_pose or None,
        "slam_map_xy_error_m": slam_map_xy_error_m,
        "slam_map_yaw_error_rad": slam_map_yaw_error_rad,
        "track_against_map": slam_status.get("track_against_map") or {},
        "min_sim_motion_for_odom_check_m": float(min_sim_motion_for_odom_check_m),
        "min_slam_motion_ratio": float(min_slam_motion_ratio),
        "max_slam_motion_ratio": float(max_slam_motion_ratio),
        "min_sim_yaw_for_odom_check_rad": float(min_sim_yaw_for_odom_check_rad),
        "max_slam_yaw_error_rad": float(max_slam_yaw_error_rad),
    }


def _slam_motion_gaps(
    motion: dict[str, Any],
    *,
    min_sim_motion_for_odom_check_m: float,
    min_slam_motion_ratio: float,
    max_slam_motion_ratio: float,
    min_sim_yaw_for_odom_check_rad: float = 0.20,
    max_slam_yaw_error_rad: float = 0.15,
    max_slam_map_xy_error_m: float = 0.35,
) -> list[str]:
    if not motion.get("available"):
        return []
    gaps: list[str] = []
    min_sim_motion = max(0.0, float(min_sim_motion_for_odom_check_m))
    sim_xy_m = float(motion.get("sim_xy_m") or 0.0)
    map_error = motion.get("slam_map_xy_error_m")
    tracking = motion.get("track_against_map") or {}
    map_localization_valid = (
        map_error is not None
        and math.isfinite(float(map_error))
        and int(tracking.get("successes") or 0) > 0
        and bool(tracking.get("enabled"))
        and int(tracking.get("consecutive_failures") or 0) == 0
    )
    if map_localization_valid:
        if float(map_error) > max(0.0, float(max_slam_map_xy_error_m)):
            gaps.append(
                "native_slam_map_pose_mismatch:"
                f"xy_error={float(map_error):.3f},max_error={float(max_slam_map_xy_error_m):.3f}"
            )
        map_yaw_error = motion.get("slam_map_yaw_error_rad")
        if map_yaw_error is not None and math.isfinite(float(map_yaw_error)):
            if abs(float(map_yaw_error)) > max(0.0, float(max_slam_yaw_error_rad)):
                gaps.append(
                    "native_slam_map_yaw_mismatch:"
                    f"yaw_error={abs(float(map_yaw_error)):.3f},"
                    f"max_error={float(max_slam_yaw_error_rad):.3f}"
                )
    elif min_sim_motion > 0.0 and sim_xy_m >= min_sim_motion:
        slam_xy_m = float(motion.get("slam_odom_xy_m") or 0.0)
        min_slam_xy_m = sim_xy_m * max(0.0, float(min_slam_motion_ratio))
        if not math.isfinite(slam_xy_m):
            gaps.append("native_slam_motion_not_finite")
        elif slam_xy_m < min_slam_xy_m:
            gaps.append(
                "native_slam_motion_mismatch:"
                f"sim_xy={sim_xy_m:.3f},slam_xy={slam_xy_m:.3f},min_slam_xy={min_slam_xy_m:.3f}"
            )
        else:
            max_ratio = max(0.0, float(max_slam_motion_ratio))
            if max_ratio > 0.0:
                max_slam_xy_m = sim_xy_m * max_ratio
                if slam_xy_m > max_slam_xy_m:
                    gaps.append(
                        "native_slam_motion_overshoot:"
                        f"sim_xy={sim_xy_m:.3f},slam_xy={slam_xy_m:.3f},max_slam_xy={max_slam_xy_m:.3f}"
                    )
    min_sim_yaw = max(0.0, float(min_sim_yaw_for_odom_check_rad))
    sim_yaw = motion.get("sim_yaw_delta_rad")
    yaw_error = (
        motion.get("slam_map_yaw_error_rad") if map_localization_valid else motion.get("slam_to_sim_yaw_error_rad")
    )
    if sim_yaw is not None and abs(float(sim_yaw)) >= min_sim_yaw:
        if yaw_error is None or not math.isfinite(float(yaw_error)):
            gaps.append("native_slam_yaw_not_finite")
        elif abs(float(yaw_error)) > max(0.0, float(max_slam_yaw_error_rad)):
            gaps.append(
                "native_slam_yaw_mismatch:"
                f"sim_yaw={float(sim_yaw):.3f},slam_yaw={float(motion.get('slam_odom_yaw_rad') or 0.0):.3f},"
                f"yaw_error={float(yaw_error):.3f},max_error={float(max_slam_yaw_error_rad):.3f}"
            )
    return gaps


def _publisher_candidates(value: str) -> list[Path]:
    if value:
        return [Path(value).expanduser()]
    if os.name == "nt":
        return [
            ROOT / "build" / "windows-native-dds-adapter" / "Release" / "lingtu_mujoco_sensor_publisher.exe",
        ]
    return [
        ROOT / "build" / "mujoco_native_dds" / "lingtu_mujoco_sensor_publisher",
    ]


def _resolve_publisher_bin(value: str) -> Path:
    for candidate in _publisher_candidates(value):
        if candidate.exists():
            return candidate.resolve()
    raise FileNotFoundError(
        "native DDS sensor publisher missing. Build it with: "
        "cmake -S sim/adapters/dds -B build/mujoco_native_dds "
        "-DLINGTU_MUJOCO_NATIVE_DDS_BUILD_RUNTIME=ON && "
        "cmake --build build/mujoco_native_dds"
    )


def _is_portable_sensor_publisher(path: Path) -> bool:
    return path.stem.lower() == "lingtu_mujoco_sensor_publisher"


def _publisher_runtime_base(args: argparse.Namespace) -> Path:
    report_value = str(getattr(args, "json_out", "") or "").strip()
    if report_value:
        return Path(report_value).expanduser().resolve().parent
    return (ROOT / "artifacts").resolve()


def _wait_for_publisher_ready(
    process: subprocess.Popen[Any],
    ready_file: Path,
    *,
    timeout_s: float = DEFAULT_NATIVE_PUBLISHER_READY_S,
) -> None:
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    last_error = "ready marker not written"
    while time.monotonic() <= deadline:
        try:
            if ready_file.exists():
                if ready_file.stat().st_size > 4096:
                    raise RuntimeError("DDS readiness marker exceeds 4096 bytes")
                payload = json.loads(ready_file.read_text(encoding="utf-8"))
                if not isinstance(payload, dict):
                    raise RuntimeError("DDS readiness marker must be a JSON object")
                if payload.get("ready") is not True:
                    raise RuntimeError("DDS readiness marker does not confirm readiness")
                if payload.get("schema") != NATIVE_SENSOR_PUBLISHER_READY_SCHEMA:
                    raise RuntimeError("DDS readiness marker schema mismatch")
                return
        except (OSError, json.JSONDecodeError) as exc:
            last_error = f"{type(exc).__name__}: {exc}"
        returncode = process.poll()
        if returncode is not None:
            raise RuntimeError(f"native sensor publisher exited before DDS readiness with return code {returncode}")
        time.sleep(0.02)
    raise RuntimeError(f"native sensor publisher DDS readiness timed out after {float(timeout_s):.1f}s ({last_error})")


def _should_restamp_native_records(args: argparse.Namespace) -> bool:
    if bool(getattr(args, "navigation_fixture", False)):
        return False
    timestamp_clock = str(getattr(args, "timestamp_clock", "") or "")
    imu_clock = str(getattr(args, "imu_timestamp_clock", "") or timestamp_clock)
    lidar_clock = str(getattr(args, "lidar_timestamp_clock", "") or timestamp_clock)
    return not _uses_unified_sim_hardware_clock(
        timestamp_clock=timestamp_clock,
        imu_clock=imu_clock,
        lidar_clock=lidar_clock,
    )


def _start_native_publisher(args: argparse.Namespace) -> subprocess.Popen[bytes]:
    publisher = _resolve_publisher_bin(str(args.publisher_bin or ""))
    publisher_args = [
        "--stdin-records",
        "--dds",
        "--domain-id",
        str(int(args.domain_id)),
        "--lidar-frame",
        LIDAR_FRAME_ID,
        "--imu-frame",
        IMU_FRAME_ID,
    ]
    if bool(getattr(args, "navigation_fixture", False)):
        publisher_args.append("--navigation-fixture")
    if _should_restamp_native_records(args):
        publisher_args.append("--restamp-stdin-records")
    ready_file: Path | None = None
    if _is_portable_sensor_publisher(publisher):
        ready_file = (_publisher_runtime_base(args) / f"mujoco_sensor_publisher_{os.getpid()}.ready.json").resolve()
        ready_file.parent.mkdir(parents=True, exist_ok=True)
        ready_file.unlink(missing_ok=True)
        ready_file.with_name(ready_file.name + ".tmp").unlink(missing_ok=True)
        ready_argument = (
            _wsl_path(ready_file) if os.name == "nt" and publisher.suffix.lower() != ".exe" else str(ready_file)
        )
        publisher_args.extend(["--ready-file", ready_argument])
    command = _linux_binary_command(publisher, *publisher_args)
    managed_wsl = os.name == "nt" and len(command) >= 3 and command[1] == "-e"
    clock_handshake = managed_wsl
    pid_file_value = str(getattr(args, "publisher_pid_file", "") or "")
    if pid_file_value:
        pid_file = Path(pid_file_value).expanduser().resolve()
    else:
        pid_file = (_publisher_runtime_base(args) / f"mujoco_sensor_publisher_{os.getpid()}.pid").resolve()
    launch_command = _managed_wsl_command(
        command,
        pid_file,
        clock_handshake=clock_handshake,
    )
    process = subprocess.Popen(
        launch_command,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE if clock_handshake else subprocess.DEVNULL,
    )
    if not managed_wsl:
        process._lingtu_publisher_kind = "local"
        process._lingtu_owned_pid = int(process.pid)
        process._lingtu_linux_pid_file = None
        process._lingtu_linux_pid = None
        process._lingtu_ready_file = ready_file
        process._lingtu_native_clock_alignment = {
            "source": "shared_local_wall_clock",
            "sample_count": 1,
            "rtt_ms": 0.0,
            "uncertainty_ms": 0.0,
            "native_minus_local_s": 0.0,
        }
        if ready_file is not None:
            try:
                _wait_for_publisher_ready(
                    process,
                    ready_file,
                    timeout_s=DEFAULT_NATIVE_PUBLISHER_READY_S,
                )
            except BaseException:
                _terminate_native_publisher_process(process)
                raise
        return process
    linux_pid = _read_linux_pid(pid_file, timeout_s=10.0)
    if linux_pid is None:
        relay_errors = _stop_relay(process)
        detail = ";".join(relay_errors) if relay_errors else "pid_file_not_written"
        raise RuntimeError(f"sensor publisher Linux PID handshake failed: {detail}")
    try:
        process._lingtu_native_clock_alignment = _synchronize_managed_native_clock(process)
    except BaseException:
        _stop_relay(process)
        raise
    process._lingtu_publisher_kind = "wsl"
    process._lingtu_owned_pid = linux_pid
    process._lingtu_linux_pid_file = pid_file
    process._lingtu_linux_pid = linux_pid
    process._lingtu_ready_file = ready_file
    if ready_file is not None:
        try:
            _wait_for_publisher_ready(
                process,
                ready_file,
                timeout_s=DEFAULT_NATIVE_PUBLISHER_READY_S,
            )
        except BaseException:
            _terminate_native_publisher_process(process)
            _stop_relay(process)
            raise
    return process


def _finish_native_publisher(
    process: subprocess.Popen[bytes],
    *,
    close_stdin: bool = True,
    termination_cleanup: dict[str, Any] | None = None,
) -> dict[str, Any]:
    pid_file = getattr(process, "_lingtu_linux_pid_file", None)
    ready_file = getattr(process, "_lingtu_ready_file", None)
    errors: list[str] = []
    try:
        returncode_before_cleanup = process.poll()
    except OSError as exc:
        returncode_before_cleanup = None
        errors.append(f"publisher_poll_failed:{type(exc).__name__}:{exc}")
    prior_parent_termination = bool(
        termination_cleanup and (termination_cleanup.get("term_sent") or termination_cleanup.get("kill_sent"))
    )
    try:
        if close_stdin and process.stdin is not None and not process.stdin.closed:
            process.stdin.close()
    except OSError as exc:
        errors.append(f"publisher_stdin_close_failed:{type(exc).__name__}:{exc}")
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        pass
    except OSError as exc:
        errors.append(f"publisher_wait_failed:{type(exc).__name__}:{exc}")

    cleanup = (
        dict(termination_cleanup) if termination_cleanup is not None else _terminate_native_publisher_process(process)
    )
    if _publisher_process_kind(process) == "wsl":
        relay_errors = _stop_relay(process)
        if relay_errors:
            errors.extend(relay_errors)
    if ready_file is not None:
        try:
            Path(ready_file).unlink(missing_ok=True)
        except OSError as exc:
            errors.append(f"publisher_ready_file_cleanup_failed:{type(exc).__name__}:{exc}")
    if errors:
        cleanup.setdefault("errors", []).extend(errors)
        cleanup["clean"] = False
    terminated_by_parent = prior_parent_termination or bool(cleanup.get("term_sent") or cleanup.get("kill_sent"))
    returncode = process.poll()
    failed_before_cleanup = returncode not in {None, 0} and not terminated_by_parent
    if failed_before_cleanup:
        cleanup.setdefault("errors", []).append(f"publisher_exited_nonzero:returncode={returncode}")
        cleanup["clean"] = False
    if terminated_by_parent:
        exit_context = "parent_requested_termination"
    elif returncode_before_cleanup is None:
        exit_context = "stdin_eof_exit"
    else:
        exit_context = "natural_exit"
    cleanup.update(
        {
            "pid_file": str(pid_file or ""),
            "ready_file": str(ready_file or ""),
            "returncode_before_cleanup": returncode_before_cleanup,
            "returncode": returncode,
            "exit_context": exit_context,
            "failed_before_cleanup": failed_before_cleanup,
            "terminated_by_parent": terminated_by_parent,
        }
    )
    return cleanup


class AsyncPublisherError(RuntimeError):
    """Fail-closed async native publisher error surfaced on the main thread."""


@dataclass(frozen=True)
class _SerializedPublisherRecord:
    diagnostic_record_type: str
    header: bytes
    payload: bytes

    @property
    def wire_bytes(self) -> int:
        return len(self.header) + len(self.payload)


class AsyncPublisherBatch:
    """One main-loop record group kept adjacent in the writer FIFO."""

    def __init__(self) -> None:
        self._records: list[_SerializedPublisherRecord] = []
        self._sealed = False

    def append(
        self,
        *,
        diagnostic_record_type: str,
        header: bytes,
        payload: bytes,
    ) -> None:
        if self._sealed:
            raise RuntimeError("async publisher batch is already enqueued")
        self._records.append(
            _SerializedPublisherRecord(
                diagnostic_record_type=str(diagnostic_record_type),
                header=bytes(header),
                payload=bytes(payload),
            )
        )

    def _seal(self) -> tuple[_SerializedPublisherRecord, ...]:
        if self._sealed:
            raise RuntimeError("async publisher batch is already enqueued")
        if not self._records:
            raise ValueError("async publisher batch must contain at least one record")
        self._sealed = True
        return tuple(self._records)


@dataclass(frozen=True)
class _QueuedPublisherBatch:
    sequence: int
    first_record_sequence: int
    last_record_sequence: int
    enqueued_monotonic_ns: int
    records: tuple[_SerializedPublisherRecord, ...]
    wire_bytes: int


class AsyncFifoPublisher:
    """One bounded FIFO whose sole writer thread owns stream I/O and close."""

    _DRAIN_GROUP_MIN_BATCHES = 8
    _DRAIN_GROUP_MAX_BATCHES = 64
    _DRAIN_GROUP_MAX_BYTES = 512 * 1024

    def __init__(
        self,
        stream: Any,
        *,
        parent_diagnostics: ParentSensorDiagnostics | None = None,
        max_bytes: int = DEFAULT_ASYNC_PUBLISHER_MAX_BYTES,
        max_records: int = DEFAULT_ASYNC_PUBLISHER_MAX_RECORDS,
        max_batches: int = DEFAULT_ASYNC_PUBLISHER_MAX_BATCHES,
        oldest_s: float = DEFAULT_ASYNC_PUBLISHER_OLDEST_S,
        monotonic_ns: Any = time.monotonic_ns,
    ) -> None:
        if int(max_bytes) <= 0 or int(max_records) <= 0 or int(max_batches) <= 0:
            raise ValueError("async publisher queue limits must be positive")
        if not math.isfinite(float(oldest_s)) or float(oldest_s) <= 0.0:
            raise ValueError("async publisher oldest age must be positive and finite")
        self._stream = stream
        self._parent_diagnostics = parent_diagnostics
        self._max_bytes = int(max_bytes)
        self._max_records = int(max_records)
        self._max_batches = int(max_batches)
        self._oldest_ns = int(float(oldest_s) * 1_000_000_000)
        self._monotonic_ns = monotonic_ns
        self._condition = threading.Condition()
        self._queue: deque[_QueuedPublisherBatch] = deque()
        self._inflight_batch: _QueuedPublisherBatch | None = None
        self._current_bytes = 0
        self._current_records = 0
        self._current_batches = 0
        self._max_observed_bytes = 0
        self._max_observed_records = 0
        self._max_observed_batches = 0
        self._stop_requested = False
        self._cleanup_reason = "running"
        self._failure: BaseException | None = None
        self._failure_context = ""
        self._failure_undrained_bytes: int | None = None
        self._failure_undrained_records: int | None = None
        self._failure_undrained_batches: int | None = None
        self._stream_closed = False
        self._enqueued_batch_sequence = 0
        self._enqueued_record_sequence = 0
        self._written_batch_sequence = 0
        self._written_record_sequence = 0
        self._writer_finished = False
        if self._parent_diagnostics is not None:
            self._parent_diagnostics.configure_async_queue(
                max_bytes=self._max_bytes,
                max_records=self._max_records,
                max_batches=self._max_batches,
                oldest_s=self._oldest_ns / 1_000_000_000.0,
            )
        self._thread = threading.Thread(
            target=self._writer_main,
            name="mujoco-native-dds-writer",
            daemon=True,
        )
        self._thread.start()
        self._update_queue_diagnostics()

    def _raise_failure_locked(self) -> None:
        if self._failure is None:
            return
        context = self._failure_context or "writer_failed"
        error = AsyncPublisherError(
            f"async native publisher {context}: {type(self._failure).__name__}: {self._failure}"
        )
        raise error from self._failure

    def _set_failure_locked(self, context: str, error: BaseException) -> None:
        if self._failure is None:
            self._failure = error
            self._failure_context = str(context)
            self._cleanup_reason = str(context)
            self._failure_undrained_bytes = self._current_bytes
            self._failure_undrained_records = self._current_records
            self._failure_undrained_batches = self._current_batches
        self._stop_requested = True
        self._condition.notify_all()

    def _oldest_age_s_locked(self, now_ns: int) -> float:
        oldest_ns: int | None = None
        if self._inflight_batch is not None:
            oldest_ns = self._inflight_batch.enqueued_monotonic_ns
        if self._queue:
            queued_ns = self._queue[0].enqueued_monotonic_ns
            oldest_ns = queued_ns if oldest_ns is None else min(oldest_ns, queued_ns)
        if oldest_ns is None:
            return 0.0
        return max(0, int(now_ns) - oldest_ns) / 1_000_000_000.0

    def _check_oldest_locked(self, now_ns: int) -> None:
        oldest_age_s = self._oldest_age_s_locked(now_ns)
        if oldest_age_s <= self._oldest_ns / 1_000_000_000.0:
            return
        self._set_failure_locked(
            "queue_oldest_age",
            AsyncPublisherError(f"queue_oldest_age:{oldest_age_s:.9f}s>{self._oldest_ns / 1_000_000_000.0:.9f}s"),
        )

    def _fail_queue_full_locked(self, limit: str) -> None:
        error = AsyncPublisherError(f"queue_full:{limit}")
        self._set_failure_locked(f"queue_full:{limit}", error)
        self._raise_failure_locked()

    def enqueue(self, batch: AsyncPublisherBatch) -> None:
        records = batch._seal()
        wire_bytes = sum(record.wire_bytes for record in records)
        now_ns = int(self._monotonic_ns())
        enqueue_started_ns = time.monotonic_ns()
        try:
            with self._condition:
                self._raise_failure_locked()
                self._check_oldest_locked(now_ns)
                self._raise_failure_locked()
                if self._stop_requested:
                    raise AsyncPublisherError("async native publisher is stopping")
                if self._current_bytes + wire_bytes > self._max_bytes:
                    self._fail_queue_full_locked("bytes")
                if self._current_records + len(records) > self._max_records:
                    self._fail_queue_full_locked("records")
                if self._current_batches + 1 > self._max_batches:
                    self._fail_queue_full_locked("batches")
                sequence = self._enqueued_batch_sequence + 1
                first_record_sequence = self._enqueued_record_sequence + 1
                last_record_sequence = self._enqueued_record_sequence + len(records)
                self._queue.append(
                    _QueuedPublisherBatch(
                        sequence=sequence,
                        first_record_sequence=first_record_sequence,
                        last_record_sequence=last_record_sequence,
                        enqueued_monotonic_ns=now_ns,
                        records=records,
                        wire_bytes=wire_bytes,
                    )
                )
                self._current_bytes += wire_bytes
                self._current_records += len(records)
                self._current_batches += 1
                self._max_observed_bytes = max(self._max_observed_bytes, self._current_bytes)
                self._max_observed_records = max(self._max_observed_records, self._current_records)
                self._max_observed_batches = max(self._max_observed_batches, self._current_batches)
                self._enqueued_batch_sequence = sequence
                self._enqueued_record_sequence = last_record_sequence
                self._check_oldest_locked(int(self._monotonic_ns()))
                self._raise_failure_locked()
                self._condition.notify()
        except BaseException:
            if self._parent_diagnostics is not None:
                stats = self.stats()
                self._parent_diagnostics.record_enqueue_result(
                    records,
                    success=False,
                    full=str(stats["fatal_reason"]).startswith("queue_full:"),
                    duration_us=(time.monotonic_ns() - enqueue_started_ns) / 1_000.0,
                    queue_stats=stats,
                )
            raise
        if self._parent_diagnostics is not None:
            self._parent_diagnostics.record_enqueue_result(
                records,
                success=True,
                full=False,
                duration_us=(time.monotonic_ns() - enqueue_started_ns) / 1_000.0,
                queue_stats=self.stats(),
            )

    def raise_if_failed(self) -> None:
        try:
            with self._condition:
                self._check_oldest_locked(int(self._monotonic_ns()))
                self._raise_failure_locked()
        except BaseException:
            self._update_queue_diagnostics()
            raise

    def request_stop(self, reason: str = "requested") -> None:
        with self._condition:
            if not self._stop_requested:
                self._cleanup_reason = str(reason)
            self._stop_requested = True
            self._condition.notify_all()
        self._update_queue_diagnostics()

    def mark_shutdown_timeout(self) -> None:
        with self._condition:
            self._set_failure_locked(
                "shutdown_timeout",
                AsyncPublisherError("shutdown_timeout"),
            )
        self._update_queue_diagnostics()

    def join(self, *, timeout_s: float | None = None) -> bool:
        self._thread.join(timeout=None if timeout_s is None else max(0.0, float(timeout_s)))
        return not self._thread.is_alive()

    @property
    def writer_alive(self) -> bool:
        return self._thread.is_alive()

    def stats(self) -> dict[str, Any]:
        with self._condition:
            return self._stats_locked()

    def _stats_locked(self) -> dict[str, Any]:
        oldest_age_s = self._oldest_age_s_locked(int(self._monotonic_ns()))
        return {
            "enqueued_batch_sequence": self._enqueued_batch_sequence,
            "enqueued_record_sequence": self._enqueued_record_sequence,
            "written_batch_sequence": self._written_batch_sequence,
            "written_record_sequence": self._written_record_sequence,
            "batch_sequence_lag": self._enqueued_batch_sequence - self._written_batch_sequence,
            "record_sequence_lag": self._enqueued_record_sequence - self._written_record_sequence,
            "current_batches": self._current_batches,
            "current_records": self._current_records,
            "current_bytes": self._current_bytes,
            "max_batches": self._max_observed_batches,
            "max_records": self._max_observed_records,
            "max_bytes": self._max_observed_bytes,
            "oldest_age_s": oldest_age_s,
            "undrained_batches": (
                self._current_batches if self._failure_undrained_batches is None else self._failure_undrained_batches
            ),
            "undrained_records": (
                self._current_records if self._failure_undrained_records is None else self._failure_undrained_records
            ),
            "undrained_bytes": (
                self._current_bytes if self._failure_undrained_bytes is None else self._failure_undrained_bytes
            ),
            "writer_alive": not self._writer_finished and self._thread.is_alive(),
            "cleanup_reason": self._cleanup_reason,
            "fatal_reason": self._failure_context,
        }

    def _update_queue_diagnostics(self) -> None:
        if self._parent_diagnostics is not None:
            self._parent_diagnostics.update_async_queue(self.stats())

    def _set_failure(self, context: str, error: BaseException) -> None:
        with self._condition:
            self._set_failure_locked(context, error)
        self._update_queue_diagnostics()

    def _take_next_batches(self) -> tuple[_QueuedPublisherBatch, ...] | None:
        with self._condition:
            while not self._queue and not self._stop_requested:
                self._condition.wait()
            if self._failure is not None or not self._queue:
                return None
            batches = [self._queue.popleft()]
            group_bytes = batches[0].wire_bytes
            if len(self._queue) + 1 >= self._DRAIN_GROUP_MIN_BATCHES:
                while self._queue and len(batches) < self._DRAIN_GROUP_MAX_BATCHES:
                    next_batch = self._queue[0]
                    if group_bytes + next_batch.wire_bytes > self._DRAIN_GROUP_MAX_BYTES:
                        break
                    batches.append(self._queue.popleft())
                    group_bytes += next_batch.wire_bytes
            self._inflight_batch = batches[0]
            return tuple(batches)

    def _complete_batches(self, batches: tuple[_QueuedPublisherBatch, ...]) -> None:
        if not batches:
            raise ValueError("completed publisher group must not be empty")
        with self._condition:
            if self._inflight_batch is batches[0]:
                self._inflight_batch = None
            self._current_bytes -= sum(batch.wire_bytes for batch in batches)
            self._current_records -= sum(len(batch.records) for batch in batches)
            self._current_batches -= len(batches)
            self._written_batch_sequence = batches[-1].sequence
            self._written_record_sequence = batches[-1].last_record_sequence
            self._condition.notify_all()
        self._update_queue_diagnostics()

    def _close_stream_once(self) -> None:
        if self._stream_closed:
            return
        self._stream_closed = True
        self._stream.close()

    def _writer_main(self) -> None:
        try:
            while True:
                batches = self._take_next_batches()
                if batches is None:
                    break
                for batch in batches:
                    for record in batch.records:
                        _write_serialized_record(
                            self._stream,
                            record,
                            parent_diagnostics=self._parent_diagnostics,
                        )
                _flush_native_publisher(
                    self._stream,
                    parent_diagnostics=self._parent_diagnostics,
                )
                self._complete_batches(batches)
        except BaseException as exc:
            self._set_failure("writer_error", exc)
        finally:
            try:
                self._close_stream_once()
            except BaseException as exc:
                self._set_failure("writer_close_error", exc)
            with self._condition:
                self._writer_finished = True
                if self._cleanup_reason == "running":
                    self._cleanup_reason = "writer_stopped"
            self._update_queue_diagnostics()


def _shutdown_async_native_publisher(
    publisher: AsyncFifoPublisher,
    process: subprocess.Popen[bytes],
    *,
    timeout_s: float = DEFAULT_ASYNC_PUBLISHER_SHUTDOWN_S,
) -> dict[str, Any]:
    timeout = float(timeout_s)
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("async publisher shutdown timeout must be positive and finite")
    publisher.request_stop("shutdown")
    if publisher.join(timeout_s=timeout):
        return {
            "timed_out": False,
            "joined_after_terminate": False,
            "timeout_stats": None,
            "termination": None,
            "queue": publisher.stats(),
        }

    publisher.mark_shutdown_timeout()
    timeout_stats = publisher.stats()
    termination = _terminate_native_publisher_process(process)
    joined_after_terminate = publisher.join(timeout_s=timeout)
    return {
        "timed_out": True,
        "joined_after_terminate": joined_after_terminate,
        "timeout_stats": timeout_stats,
        "termination": termination,
        "queue": publisher.stats(),
    }


def _cleanup_native_publisher(
    process: subprocess.Popen[bytes],
    *,
    async_publisher: AsyncFifoPublisher | None,
    async_shutdown_s: float = DEFAULT_ASYNC_PUBLISHER_SHUTDOWN_S,
) -> dict[str, Any]:
    if async_publisher is None:
        return _finish_native_publisher(process)

    async_cleanup = _shutdown_async_native_publisher(
        async_publisher,
        process,
        timeout_s=async_shutdown_s,
    )
    cleanup = _finish_native_publisher(
        process,
        close_stdin=False,
        termination_cleanup=async_cleanup.get("termination"),
    )
    cleanup["async_writer"] = async_cleanup
    async_errors: list[str] = []
    if bool(async_cleanup.get("timed_out")):
        async_errors.append("async_writer_shutdown_timeout")
    if bool(async_cleanup.get("timed_out")) and not bool(async_cleanup.get("joined_after_terminate")):
        async_errors.append("async_writer_still_alive_after_terminate")
    fatal_reason = str((async_cleanup.get("queue") or {}).get("fatal_reason") or "")
    if fatal_reason:
        async_errors.append(f"async_writer_fatal:{fatal_reason}")
    if async_errors:
        cleanup.setdefault("errors", []).extend(async_errors)
        cleanup["clean"] = False
    return cleanup


def _native_sensor_publisher_gaps(cleanup: dict[str, Any]) -> list[str]:
    gaps: list[str] = []
    if bool(cleanup.get("failed_before_cleanup")) and not bool(cleanup.get("terminated_by_parent")):
        gaps.append("native_sensor_publisher_failed")
    if cleanup.get("clean") is not True:
        gaps.append("native_sensor_publisher_cleanup_failed")
    return gaps


def _write_serialized_record(
    stream: Any,
    record: _SerializedPublisherRecord,
    *,
    parent_diagnostics: ParentSensorDiagnostics | None = None,
) -> None:
    if parent_diagnostics is None:
        stream.write(record.header)
        stream.write(record.payload)
        return
    parent_diagnostics.record_pipe_write_attempt(record.diagnostic_record_type)
    started_ns = time.monotonic_ns()
    try:
        stream.write(record.header)
        stream.write(record.payload)
    except Exception:
        parent_diagnostics.record_pipe_write_error(
            record.diagnostic_record_type,
            duration_us=(time.monotonic_ns() - started_ns) / 1_000.0,
        )
        raise
    parent_diagnostics.record_pipe_write_success(
        record.diagnostic_record_type,
        payload_bytes=len(record.payload),
        pipe_bytes=record.wire_bytes,
        duration_us=(time.monotonic_ns() - started_ns) / 1_000.0,
    )


def _write_record(
    stream: Any,
    record_type: int,
    timestamp_ns: int,
    sequence: int,
    payload: bytes,
    count: int,
    *,
    parent_diagnostics: ParentSensorDiagnostics | None = None,
    diagnostic_record_type: str = "",
    async_batch: AsyncPublisherBatch | None = None,
) -> None:
    _publish_encoded_record(
        stream,
        _sensor_records.encode_record(
            record_type,
            timestamp_ns=timestamp_ns,
            sequence=sequence,
            payload=payload,
            count=count,
        ),
        parent_diagnostics=parent_diagnostics,
        diagnostic_record_type=diagnostic_record_type,
        async_batch=async_batch,
    )


def _publish_encoded_record(
    stream: Any,
    encoded: _sensor_records.EncodedSensorRecord,
    *,
    parent_diagnostics: ParentSensorDiagnostics | None = None,
    diagnostic_record_type: str,
    async_batch: AsyncPublisherBatch | None = None,
) -> None:
    record = _SerializedPublisherRecord(
        diagnostic_record_type=str(diagnostic_record_type),
        header=encoded.header,
        payload=encoded.payload,
    )
    if async_batch is not None:
        async_batch.append(
            diagnostic_record_type=record.diagnostic_record_type,
            header=record.header,
            payload=record.payload,
        )
        return
    _write_serialized_record(
        stream,
        record,
        parent_diagnostics=parent_diagnostics,
    )


def _flush_native_publisher(
    stream: Any,
    *,
    parent_diagnostics: ParentSensorDiagnostics | None = None,
) -> None:
    if parent_diagnostics is None:
        stream.flush()
        return
    started_ns = time.monotonic_ns()
    try:
        stream.flush()
    except Exception:
        parent_diagnostics.record_flush(
            success=False,
            duration_us=(time.monotonic_ns() - started_ns) / 1_000.0,
        )
        raise
    parent_diagnostics.record_flush(
        success=True,
        duration_us=(time.monotonic_ns() - started_ns) / 1_000.0,
    )


def _begin_native_publisher_batch(
    async_publisher: AsyncFifoPublisher | None,
) -> AsyncPublisherBatch | None:
    if async_publisher is None:
        return None
    async_publisher.raise_if_failed()
    return AsyncPublisherBatch()


def _commit_native_publisher_batch(
    stream: Any,
    batch: AsyncPublisherBatch | None,
    *,
    async_publisher: AsyncFifoPublisher | None,
    parent_diagnostics: ParentSensorDiagnostics | None = None,
) -> None:
    if async_publisher is None:
        if batch is not None:
            raise RuntimeError("sync publisher received an async batch")
        _flush_native_publisher(
            stream,
            parent_diagnostics=parent_diagnostics,
        )
        return
    if batch is None:
        raise RuntimeError("async publisher batch is missing")
    async_publisher.raise_if_failed()
    async_publisher.enqueue(batch)
    async_publisher.raise_if_failed()


def _write_native_scan(
    stream: Any,
    scan: Any,
    *,
    parent_diagnostics: ParentSensorDiagnostics | None = None,
    async_batch: AsyncPublisherBatch | None = None,
) -> None:
    _publish_encoded_record(
        stream,
        _sensor_records.encode_scan(scan),
        parent_diagnostics=parent_diagnostics,
        diagnostic_record_type="cloud",
        async_batch=async_batch,
    )


def _write_native_imu(
    stream: Any,
    imu: Imu,
    sequence: int,
    *,
    parent_diagnostics: ParentSensorDiagnostics | None = None,
    async_batch: AsyncPublisherBatch | None = None,
) -> None:
    _publish_encoded_record(
        stream,
        _sensor_records.encode_imu(imu, sequence=sequence),
        parent_diagnostics=parent_diagnostics,
        diagnostic_record_type="imu",
        async_batch=async_batch,
    )


def _write_native_odom_prior(
    stream: Any,
    state: Any,
    timestamp_s: float,
    sequence: int,
    *,
    velocity: Any | None = None,
    has_velocity: bool = True,
    parent_diagnostics: ParentSensorDiagnostics | None = None,
    async_batch: AsyncPublisherBatch | None = None,
) -> None:
    _publish_encoded_record(
        stream,
        _sensor_records.encode_odom_prior(
            state,
            timestamp_s=timestamp_s,
            sequence=sequence,
            velocity=velocity,
            has_velocity=has_velocity,
        ),
        parent_diagnostics=parent_diagnostics,
        diagnostic_record_type="odom_prior",
        async_batch=async_batch,
    )


def _write_native_registered_cloud(
    stream: Any,
    points_xyzi_body: Any,
    *,
    timestamp_ns: int,
    sequence: int,
    parent_diagnostics: ParentSensorDiagnostics | None = None,
    async_batch: AsyncPublisherBatch | None = None,
) -> None:
    _publish_encoded_record(
        stream,
        _sensor_records.encode_registered_cloud(
            points_xyzi_body,
            timestamp_ns=timestamp_ns,
            sequence=sequence,
        ),
        parent_diagnostics=parent_diagnostics,
        diagnostic_record_type="registered_cloud",
        async_batch=async_batch,
    )


def _read_json_object(path: str | Path) -> dict[str, Any]:
    if not path:
        return {}
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _native_nav_goal_reached(status: dict[str, Any]) -> bool:
    local = status.get("last_local") or {}
    return bool(local.get("goal_reached")) or local.get("reason") == "goal_reached"


def _slam_status_counts(path: str) -> tuple[Counter[str], dict[str, Any]]:
    if not path:
        return Counter(), {}
    status_path = Path(path)
    if not status_path.exists():
        return Counter(), {}
    status = _read_json_object(status_path)
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


def _acceptance_contacts(model: Any, data: Any) -> list[dict[str, Any]]:
    """Return contacts involving scenario geoms for acceptance evidence."""

    import mujoco

    contacts: list[dict[str, Any]] = []
    for index in range(int(getattr(data, "ncon", 0))):
        contact = data.contact[index]
        geom1 = str(
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1))
            or ""
        )
        geom2 = str(
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2))
            or ""
        )
        if not (geom1.startswith("acceptance_") or geom2.startswith("acceptance_")):
            continue
        contacts.append(
            {
                "geom1": geom1,
                "geom2": geom2,
                "distance_m": float(contact.dist),
            }
        )
    return contacts


def run(args: argparse.Namespace) -> dict[str, Any]:
    from sim.compat.engine.core.engine import VelocityCommand

    duration_s = max(0.0, float(args.duration))
    settle_s = max(0.0, float(args.settle_s))
    warmup_s = max(0.0, float(args.warmup_s))
    requested_start = parse_start(str(args.start or ""))
    requested_start_yaw_deg = getattr(args, "start_yaw_deg", None)
    if requested_start_yaw_deg is not None and not math.isfinite(
        float(requested_start_yaw_deg)
    ):
        raise ValueError("--start-yaw-deg must be finite")
    start_anchor = str(args.start_anchor)
    drive_ramp_s = max(0.0, float(args.drive_ramp_s))
    imu_acc_axis_scale, imu_acc_axis_scale_source = _resolve_imu_acc_axis_scale(
        str(args.imu_acc_axis_scale),
        drive_mode=str(args.drive_mode),
        acc_mode=str(args.imu_acc_mode),
        timestamp_clock=str(args.timestamp_clock),
    )
    imu_gyro_axis_scale = _resolve_axis_scale(str(args.imu_gyro_axis_scale), "--imu-gyro-axis-scale")
    lidar_hz = max(0.1, float(args.publish_hz))
    imu_hz = max(lidar_hz, float(args.imu_hz))
    lidar_period_s = 1.0 / lidar_hz
    imu_period_s = 1.0 / imu_hz
    scan_duration_ns = int(1_000_000_000 * lidar_period_s)
    navigation_fixture = bool(getattr(args, "navigation_fixture", False))
    external_arm_config = _external_arm_config_from_args(args)
    if external_arm_config is not None:
        _prepare_external_arm_files(external_arm_config)
    external_arm_gate: ExternalArmGate | None = None
    external_arm_failure_gap = ""
    motion_interval_completed = False
    motion_complete_marker_value = str(getattr(args, "motion_complete_marker", "") or "")
    motion_complete_marker = (
        Path(motion_complete_marker_value).expanduser().resolve() if motion_complete_marker_value else None
    )
    if motion_complete_marker is not None:
        motion_complete_marker.unlink(missing_ok=True)
    if navigation_fixture and not bool(args.publish_odom_prior):
        raise ValueError("--navigation-fixture requires --publish-odom-prior")
    if navigation_fixture and str(args.scan_time_profile) != "instantaneous":
        raise ValueError("--navigation-fixture requires --scan-time-profile instantaneous")
    navigation_fixture_cloud_points = max(
        1,
        int(getattr(args, "navigation_fixture_cloud_points", 4000) or 4000),
    )
    navigation_fixture_ground_y_half_m = float(
        getattr(
            args,
            "navigation_fixture_ground_y_half_m",
            NAVIGATION_FIXTURE_GROUND_Y_HALF_M,
        )
    )
    if not math.isfinite(navigation_fixture_ground_y_half_m) or navigation_fixture_ground_y_half_m <= 0.0:
        raise ValueError("navigation fixture ground y half-width must be positive and finite")
    navigation_fixture_ground_resolution_m = float(
        getattr(
            args,
            "navigation_fixture_ground_resolution_m",
            NAVIGATION_FIXTURE_GROUND_RESOLUTION_M,
        )
    )
    if (
        not math.isfinite(navigation_fixture_ground_resolution_m)
        or navigation_fixture_ground_resolution_m <= 0.0
        or navigation_fixture_ground_resolution_m > NAVIGATION_FIXTURE_GROUND_RESOLUTION_M
    ):
        raise ValueError("navigation fixture ground resolution must be in (0, 0.2]")
    navigation_fixture_ground_report: dict[str, Any] = {
        "enabled": navigation_fixture,
        "synthetic_ground_points": 0,
        "raw_body_points": 0,
        "raw_overlay_points": 0,
        "raw_obstacle_overlay_points": 0,
        "raw_overlay_enabled": bool(getattr(args, "navigation_fixture_raw_overlay", True)),
        "published_points": 0,
        "max_points": navigation_fixture_cloud_points,
        "resolution_m": navigation_fixture_ground_resolution_m,
        "y_half_m": navigation_fixture_ground_y_half_m,
    }
    imu_acc_conditioner: SimImuSignalConditioner | None = None
    if str(args.imu_acc_mode) == "sensor" and str(args.imu_acc_conditioning) == "realistic":
        imu_acc_conditioner = SimImuSignalConditioner(
            lowpass_hz=float(args.imu_acc_lowpass_hz),
            max_dynamic_accel_mps2=float(args.imu_acc_max_dynamic_mps2),
            max_slew_rate_mps3=float(args.imu_acc_max_slew_mps3),
        )
    sensor_counts: Counter[str] = Counter()
    slam_counts: Counter[str] = Counter()
    parent_diagnostics: ParentSensorDiagnostics | None = getattr(
        args,
        "_parent_diagnostics",
        None,
    )
    driver_bridge_path: Path | None = None
    if str(args.command_source) == "dds":
        driver_bridge_path = _resolve_driver_bridge_bin(str(args.driver_bridge_bin or ""))
        if not _safe_protocol_token(str(args.driver_expected_host_boot_id or "")):
            raise ValueError("--driver-expected-host-boot-id must be a safe non-empty token for --command-source dds")
    publisher_path = _resolve_publisher_bin(str(args.publisher_bin or ""))
    publisher = _start_native_publisher(args)
    publisher_write_mode = str(
        getattr(args, "publisher_write_mode", DEFAULT_PUBLISHER_WRITE_MODE) or DEFAULT_PUBLISHER_WRITE_MODE
    )
    if publisher_write_mode not in {"sync", "async_fifo"}:
        _finish_native_publisher(publisher)
        raise ValueError(f"unsupported publisher write mode: {publisher_write_mode}")
    async_publisher: AsyncFifoPublisher | None = None
    async_publisher_shutdown_s = float(getattr(args, "async_publisher_shutdown_s", DEFAULT_ASYNC_PUBLISHER_SHUTDOWN_S))
    if not math.isfinite(async_publisher_shutdown_s) or async_publisher_shutdown_s <= 0.0:
        _finish_native_publisher(publisher)
        raise ValueError("--async-publisher-shutdown-s must be positive and finite")
    publisher_cleanup: dict[str, Any] = {}
    native_clock_alignment: dict[str, Any] = {}
    cleanup_errors: list[str] = []
    driver_bridge: NativeDriverBridge | None = None
    driver_heartbeat: DriverHeartbeat | None = None
    driver_bridge_stats: dict[str, Any] = {
        "transport": "deterministic_profile",
        "samples": 0,
        "nonzero_samples": 0,
    }
    try:
        if publisher_write_mode == "async_fifo":
            if publisher.stdin is None:
                raise RuntimeError("native publisher stdin closed")
            async_publisher = AsyncFifoPublisher(
                publisher.stdin,
                parent_diagnostics=parent_diagnostics,
                max_bytes=int(getattr(args, "async_publisher_max_bytes", DEFAULT_ASYNC_PUBLISHER_MAX_BYTES)),
                max_records=int(getattr(args, "async_publisher_max_records", DEFAULT_ASYNC_PUBLISHER_MAX_RECORDS)),
                max_batches=int(getattr(args, "async_publisher_max_batches", DEFAULT_ASYNC_PUBLISHER_MAX_BATCHES)),
                oldest_s=float(getattr(args, "async_publisher_oldest_s", DEFAULT_ASYNC_PUBLISHER_OLDEST_S)),
            )
    except Exception:
        _cleanup_native_publisher(
            publisher,
            async_publisher=async_publisher,
            async_shutdown_s=async_publisher_shutdown_s,
        )
        raise

    motion_log_path = str(getattr(args, "motion_log", "") or "")
    motion_log_writer: AsyncJsonlWriter | None = None
    motion_log_samples = 0
    motion_log_writer_diagnostics = {
        "submitted": 0,
        "written": 0,
        "dropped": 0,
        "failures": 0,
        "max_pending": 0,
    }
    planner_debug_path = ""
    planner_debug_writer: AsyncJsonlWriter | None = None
    planner_debug_samples = 0
    planner_debug_writer_diagnostics = {
        "submitted": 0,
        "written": 0,
        "dropped": 0,
        "failures": 0,
        "max_pending": 0,
    }
    planner_debug_id = 0
    last_planner_debug_key: tuple[float, float] | None = None
    motion_log_hz = max(1.0, float(getattr(args, "motion_log_hz", 24.0) or 24.0))
    motion_log_period_s = 1.0 / motion_log_hz
    motion_log_lidar_points = max(
        0,
        int(getattr(args, "motion_log_lidar_points", 640) or 0),
    )
    if motion_log_path:
        log_path = Path(motion_log_path)
        motion_log_writer = AsyncJsonlWriter(
            log_path,
            max_pending=2048,
            flush_every=64,
        )
        planner_debug_log_path = log_path.with_name(f"{log_path.stem}_planner_debug.jsonl")
        planner_debug_path = str(planner_debug_log_path)
        planner_debug_writer = AsyncJsonlWriter(
            planner_debug_log_path,
            max_pending=1024,
            flush_every=16,
        )

    engine = None
    viewer = None
    viewer_closed_early = False
    viewer_overlay = {"point_count": 0, "global_segments": 0, "local_segments": 0}
    policy_loaded = False
    policy_runtime_report: dict[str, Any] = {
        "backend": "unconfigured",
        "requested_cpu_threads": int(args.policy_cpu_threads),
        "active_cpu_threads": None,
        "active_interop_threads": None,
    }
    physics_timestep_requested_s = 0.0
    pacing_controller: SimHardwareCatchUpController | None = None
    pacing_stats: dict[str, Any] = {
        "enabled": False,
        "strategy": "legacy_split_or_wall_clock",
    }
    last_sim_time_s = 0.0
    sim_start_s = 0.0
    goal_reached_early = False
    motion_started_sim_s: float | None = None
    lidar_backend_report: dict[str, Any] = {}
    physics_integrator_requested = str(getattr(args, "physics_integrator", "model") or "model")
    physics_integrator_active = "unloaded"
    physics_timestep_s = 0.0
    runtime_stage_profiler = RuntimeStageProfiler()
    mocap_motion: LinearMocapMotion | None = None
    try:
        policy_path = _resolve_policy_path_for_drive(str(args.drive_mode), str(args.policy_path or ""))
        policy_runtime_report = _configure_policy_cpu_threads(policy_path, int(args.policy_cpu_threads))
        physics_timestep_requested_s = float(args.physics_timestep_s)
        if not math.isfinite(physics_timestep_requested_s):
            raise ValueError("--physics-timestep-s must be finite")
        engine = build_engine(
            world=resolve_world(str(args.world)),
            drive_mode=str(args.drive_mode),
            start=requested_start,
            mujoco_memory=str(args.mujoco_memory),
            mid360_pattern=args.mid360_pattern,
            mid360_samples_per_frame=int(args.mid360_samples_per_frame),
            lidar_backend=str(args.lidar_backend),
            mujoco_lidar_backend=str(args.mujoco_lidar_backend),
            require_product_lidar_backend=True,
            policy_path=policy_path,
            policy_cpu_threads=int(args.policy_cpu_threads),
            max_linear_vel=(float(args.driver_max_linear_mps) if str(args.command_source) == "dds" else None),
            max_angular_vel=(float(args.driver_max_angular_rps) if str(args.command_source) == "dds" else None),
        )
        if bool(getattr(args, "viewer", False)):
            viewer = launch_presentation_viewer(engine.model, engine.data)
        if physics_integrator_requested != "model":
            engine.set_physics_integrator(physics_integrator_requested)
        if physics_timestep_requested_s != 0.0:
            engine.set_physics_timestep(physics_timestep_requested_s)
        mocap_body = str(getattr(args, "mocap_motion_body", "") or "").strip()
        if mocap_body:
            mocap_motion = LinearMocapMotion.attach(
                engine.model,
                body_name=mocap_body,
                start_xyz=_parse_xyz_triplet(
                    str(args.mocap_motion_start),
                    "--mocap-motion-start",
                ),
                end_xyz=_parse_xyz_triplet(
                    str(args.mocap_motion_end),
                    "--mocap-motion-end",
                ),
                start_s=float(args.mocap_motion_start_s),
                duration_s=float(args.mocap_motion_duration_s),
            )
            mocap_motion.update(engine.data, 0.0)
        physics_integrator_active = str(engine.physics_integrator)
        physics_timestep_s = float(engine.dt)
        lidar_backend_report = engine.get_lidar_backend_report()
        policy_loaded = bool(getattr(engine, "has_policy", False))
        hold_cmd = VelocityCommand()
        policy_settled_before_anchor = bool(
            policy_loaded and start_anchor in {"warmup", "run"} and settle_s > 0.0
        )
        if policy_settled_before_anchor:
            settle_end_s = float(getattr(engine, "sim_time", 0.0)) + settle_s
            while float(getattr(engine, "sim_time", 0.0)) + 1e-9 < settle_end_s:
                if parent_diagnostics is not None:
                    if parent_diagnostics.stop_requested:
                        break
                    parent_diagnostics.maybe_publish()
                _step_engine_for_sensor_tick(engine, hold_cmd, imu_period_s)

        initial_state = engine.get_robot_state()
        if viewer is not None:
            focus_presentation_viewer(viewer, initial_state.position, initialize=True)
            viewer.sync()
        anchor_position = _anchor_position_after_policy_settle(
            requested_start,
            initial_state.position,
            policy_settled=policy_settled_before_anchor,
        )
        anchor_orientation = np.asarray(initial_state.orientation, dtype=np.float64)
        if requested_start_yaw_deg is not None:
            yaw_half = 0.5 * math.radians(float(requested_start_yaw_deg))
            anchor_orientation = np.asarray(
                [0.0, 0.0, math.sin(yaw_half), math.cos(yaw_half)],
                dtype=np.float64,
            )

        def apply_start_anchor() -> Any:
            engine.set_robot_pose(anchor_position, anchor_orientation)
            return engine.get_robot_state()

        motion_started = False
        motion_started_sim_s = None
        if _start_anchor_active(start_anchor, motion_started=motion_started):
            apply_start_anchor()
        if settle_s > 0.0 and not policy_settled_before_anchor:
            settle_end_s = float(getattr(engine, "sim_time", 0.0)) + settle_s
            while float(getattr(engine, "sim_time", 0.0)) + 1e-9 < settle_end_s:
                if parent_diagnostics is not None:
                    if parent_diagnostics.stop_requested:
                        break
                    parent_diagnostics.maybe_publish()
                if _start_anchor_active(start_anchor, motion_started=motion_started):
                    _step_static_engine_for_sensor_tick(engine, imu_period_s)
                else:
                    engine.step(hold_cmd)

        timestamp_clock = str(args.timestamp_clock)
        imu_timestamp_clock = str(args.imu_timestamp_clock or timestamp_clock)
        lidar_timestamp_clock = str(args.lidar_timestamp_clock or timestamp_clock)
        sim_start_s = float(getattr(engine, "sim_time", 0.0))
        native_clock_alignment = dict(publisher._lingtu_native_clock_alignment)
        hardware_clock = SimulatedHardwareClock(
            sim_start_s=sim_start_s,
            wall_epoch_s=(time.time() + float(native_clock_alignment["native_minus_local_s"])),
            monotonic_start_s=time.monotonic(),
            realtime_factor=float(args.sim_hardware_realtime_factor),
        )
        sim_clock_epoch_s = hardware_clock.wall_epoch_s
        unified_sim_hardware_clock = _uses_unified_sim_hardware_clock(
            timestamp_clock=timestamp_clock,
            imu_clock=imu_timestamp_clock,
            lidar_clock=lidar_timestamp_clock,
        )
        dds_closed_loop = str(args.command_source) == "dds"
        if unified_sim_hardware_clock and (not navigation_fixture or dds_closed_loop):
            pacing_controller = SimHardwareCatchUpController(
                clock=hardware_clock,
                max_lag_s=float(args.sim_hardware_max_lag_s),
                yield_every_steps=int(args.sim_hardware_catch_up_yield_steps),
            )
        elif unified_sim_hardware_clock:
            pacing_stats = {
                "enabled": False,
                "strategy": "navigation_fixture_complete_observation_stream",
                "drop_sensor_ticks": False,
            }
        last_sim_time_s = sim_start_s
        drive_vx = float(args.drive_vx)
        drive_vy = float(args.drive_vy)
        drive_wz = float(args.drive_wz)
        drive_profile = str(args.drive_profile)
        run_start_s = time.monotonic()
        drive_start_s = run_start_s + warmup_s
        deadline = drive_start_s + duration_s
        if external_arm_config is not None:
            external_arm_gate = ExternalArmGate(
                **external_arm_config,
                started_wall_s=run_start_s,
            )
        sequence = 0
        imu_sequence = 0
        odom_prior_sequence = 0
        registered_cloud_sequence = 0
        odom_prior_velocity_estimator = OdomPriorVelocityEstimator(window_s=float(args.odom_prior_velocity_window_s))
        mujoco_truth_velocity_estimator = OdomPriorVelocityEstimator(
            window_s=float(args.odom_prior_velocity_window_s)
        )
        prev_imu_s = None
        prev_velocity = None
        next_lidar_s = 0.0
        sim_start_position = None
        sim_end_position = None
        sim_start_yaw = None
        sim_end_yaw = None
        sim_path_length_xy_m = 0.0
        sim_abs_yaw_delta_rad = 0.0
        prev_drive_position = None
        prev_drive_yaw = None
        subscan_samples: list[tuple[float, Any, Any, int]] = []
        latest_world_points = np.zeros((0, 4), dtype=np.float32)
        next_motion_log_sim_s = sim_start_s
        next_nav_goal_check_sim_s = sim_start_s
        rolling_subscan_samples: int | None = None
        if str(args.physical_rolling_sample_mode) == "subscan":
            rolling_subscan_samples = _rolling_subscan_sample_count(
                frame_samples=int(args.mid360_samples_per_frame),
                imu_period_s=imu_period_s,
                lidar_period_s=lidar_period_s,
            )
        next_lidar_sim_s = sim_start_s
        viewer_period_s = 1.0 / max(1.0, float(getattr(args, "viewer_hz", 30.0) or 30.0))
        next_viewer_sim_s = sim_start_s
        viewer_overlay_period_s = 0.1
        next_viewer_overlay_sim_s = sim_start_s
        previous_loop_end_wall_s: float | None = None
        driver_step_seq = 0
        if str(args.command_source) == "dds":
            if driver_bridge_path is None:
                raise RuntimeError("driver bridge artifact was not resolved")
            driver_bridge = NativeDriverBridge(
                binary=driver_bridge_path,
                domain_id=int(args.domain_id),
                expected_host_boot_id=str(args.driver_expected_host_boot_id),
                max_linear_mps=float(args.driver_max_linear_mps),
                max_angular_rps=float(args.driver_max_angular_rps),
                pid_file=(
                    Path(str(args.driver_bridge_pid_file)).expanduser().resolve()
                    if str(args.driver_bridge_pid_file or "")
                    else None
                ),
                command_timeout_ms=int(args.driver_command_timeout_ms),
                heartbeat_timeout_ms=int(args.driver_heartbeat_timeout_ms),
                apply_timeout_ms=int(args.driver_apply_timeout_ms),
            )
            driver_heartbeat = DriverHeartbeat(driver_bridge)
            driver_heartbeat.start()
        while True:
            if driver_heartbeat is not None:
                driver_heartbeat.raise_if_failed()
            publisher_batch = _begin_native_publisher_batch(async_publisher)
            if parent_diagnostics is not None:
                if parent_diagnostics.stop_requested:
                    break
                if parent_diagnostics.publish_due():
                    if pacing_controller is not None:
                        parent_diagnostics.update_scheduler_pacing(
                            pacing_controller.stats(
                                last_sim_time_s,
                                monotonic_now_s=time.monotonic(),
                            )
                        )
                    parent_diagnostics.maybe_publish()
            loop_start = time.monotonic()
            if previous_loop_end_wall_s is not None:
                runtime_stage_profiler.record(
                    "inter_loop_gap",
                    loop_start - previous_loop_end_wall_s,
                    float(getattr(engine, "sim_time", 0.0)),
                )
            command_stage_start = time.monotonic()
            drop_sensor_tick = False
            sim_time_before_step_s = float(getattr(engine, "sim_time", 0.0))
            if external_arm_gate is not None:
                external_arm_gate.poll(
                    sim_time_s=sim_time_before_step_s,
                    monotonic_now_s=loop_start,
                )
                if external_arm_gate.failed:
                    external_arm_failure_gap = external_arm_gate.failure_gap
                    break
                driving = external_arm_gate.acknowledged
                drive_elapsed_s = _external_arm_drive_elapsed_s(
                    external_arm_gate,
                    sim_time_s=sim_time_before_step_s,
                )
                if driving and drive_elapsed_s >= duration_s:
                    motion_interval_completed = True
                    break
                if unified_sim_hardware_clock:
                    drop_sensor_tick = bool(
                        pacing_controller
                        and pacing_controller.should_drop_sensor_tick(
                            sim_time_before_step_s,
                            monotonic_now_s=loop_start,
                        )
                    )
            if external_arm_gate is None and unified_sim_hardware_clock:
                sim_elapsed_before_step_s = max(0.0, float(getattr(engine, "sim_time", 0.0)) - sim_start_s)
                anchored_dds_motion = driver_bridge is not None and start_anchor == "warmup"
                if anchored_dds_motion:
                    drive_elapsed_s = (
                        max(0.0, sim_time_before_step_s - motion_started_sim_s)
                        if motion_started_sim_s is not None
                        else 0.0
                    )
                    if motion_started_sim_s is not None and drive_elapsed_s >= duration_s:
                        motion_interval_completed = True
                        break
                elif sim_elapsed_before_step_s >= warmup_s + duration_s:
                    motion_interval_completed = True
                    break
                driving = sim_elapsed_before_step_s >= warmup_s
                if not anchored_dds_motion:
                    drive_elapsed_s = max(0.0, sim_elapsed_before_step_s - warmup_s)
                drop_sensor_tick = bool(
                    pacing_controller
                    and pacing_controller.should_drop_sensor_tick(
                        float(getattr(engine, "sim_time", 0.0)),
                        monotonic_now_s=loop_start,
                    )
                )
            elif external_arm_gate is None:
                if loop_start >= deadline:
                    motion_interval_completed = True
                    break
                driving = loop_start >= drive_start_s
                drive_elapsed_s = max(0.0, loop_start - drive_start_s) if driving else 0.0
            prepared_driver_step: PreparedDriverBridgeStep | None = None
            if driver_bridge is not None:
                prepared_driver_step = driver_bridge.prepare_step()
                cmd = prepared_driver_step.velocity
            elif driving:
                ramp = 1.0 if drive_ramp_s <= 0.0 else min(1.0, drive_elapsed_s / drive_ramp_s)
                profile_vx, profile_vy, profile_wz = _drive_command_for_profile(
                    drive_profile,
                    drive_elapsed_s,
                    drive_vx=drive_vx,
                    drive_vy=drive_vy,
                    drive_wz=drive_wz,
                )
                cmd = VelocityCommand(
                    linear_x=profile_vx * ramp,
                    linear_y=profile_vy * ramp,
                    angular_z=profile_wz * ramp,
                )
            else:
                cmd = hold_cmd
            command_norm = math.sqrt(float(cmd.linear_x) ** 2 + float(cmd.linear_y) ** 2 + float(cmd.angular_z) ** 2)
            if driver_bridge is not None:
                anchor_active, motion_started = _driver_bridge_anchor_state(
                    start_anchor,
                    motion_started=motion_started,
                    driving=driving,
                    external_arm_gate=external_arm_gate,
                    command_norm=command_norm,
                )
                if motion_started and motion_started_sim_s is None:
                    motion_started_sim_s = sim_time_before_step_s
            elif driving:
                motion_started = True
                anchor_active = _sensor_anchor_active(
                    start_anchor,
                    motion_started=motion_started,
                    external_arm_gate=external_arm_gate,
                )
            else:
                anchor_active = _sensor_anchor_active(
                    start_anchor,
                    motion_started=motion_started,
                    external_arm_gate=external_arm_gate,
                )
            runtime_stage_profiler.record(
                "command_input",
                time.monotonic() - command_stage_start,
                float(getattr(engine, "sim_time", 0.0)),
            )
            physics_stage_start = time.monotonic()
            if mocap_motion is not None:
                mocap_motion.update(
                    engine.data,
                    drive_elapsed_s if driving else 0.0,
                    wall_s=time.time(),
                )
            fast_static_clock = anchor_active and drop_sensor_tick and driver_bridge is None
            if driver_bridge is not None:
                if prepared_driver_step is None:
                    raise RuntimeError("driver bridge step was not prepared")
                if anchor_active:
                    state, driver_step_seq = _step_anchored_with_driver_bridge(
                        engine,
                        driver_bridge,
                        prepared_driver_step,
                        imu_period_s=imu_period_s,
                        step_seq=driver_step_seq,
                    )
                else:
                    state, driver_step_seq = _step_with_driver_bridge(
                        engine,
                        driver_bridge,
                        prepared_driver_step,
                        imu_period_s=imu_period_s,
                        step_seq=driver_step_seq,
                    )
            elif fast_static_clock:
                sim_time_s = _advance_static_engine_clock_for_dropped_tick(
                    engine,
                    imu_period_s,
                )
                state = None
            elif anchor_active:
                state = _step_static_engine_for_sensor_tick(engine, imu_period_s)
            else:
                state = _step_engine_for_sensor_tick(engine, cmd, imu_period_s)
            sim_time_s = float(getattr(engine, "sim_time", sim_time_s if state is None else 0.0))
            runtime_stage_profiler.record(
                "physics_step",
                time.monotonic() - physics_stage_start,
                sim_time_s,
            )
            last_sim_time_s = sim_time_s
            if viewer is not None and sim_time_s + 1e-9 >= next_viewer_sim_s:
                if not viewer.is_running():
                    viewer_closed_early = True
                    break
                if sim_time_s + 1e-9 >= next_viewer_overlay_sim_s:
                    viewer_overlay = draw_navigation_paths(
                        viewer,
                        _read_json_object(str(getattr(args, "nav_status_json", "") or "")),
                        point_cloud=latest_world_points,
                    )
                    next_viewer_overlay_sim_s = sim_time_s + viewer_overlay_period_s
                focus_presentation_viewer(
                    viewer,
                    anchor_position if state is None else state.position,
                )
                viewer.sync()
                next_viewer_sim_s = sim_time_s + viewer_period_s
            if state is None:
                position = anchor_position.copy()
                yaw = yaw_from_quat_xyzw(anchor_orientation)
            else:
                position = np.asarray(state.position, dtype=np.float64).copy()
                yaw = yaw_from_quat_xyzw(state.orientation)
            physical_drive_active = driving and (driver_bridge is None or motion_started)
            if physical_drive_active and sim_start_position is None:
                sim_start_position = position.copy()
                sim_start_yaw = float(yaw)
                prev_drive_position = position.copy()
                prev_drive_yaw = float(yaw)
            if physical_drive_active:
                if prev_drive_position is not None:
                    drive_delta = position[:2] - prev_drive_position[:2]
                    sim_path_length_xy_m += float(math.hypot(float(drive_delta[0]), float(drive_delta[1])))
                if prev_drive_yaw is not None:
                    sim_abs_yaw_delta_rad += abs(float(angle_delta_rad(float(yaw), float(prev_drive_yaw))))
                prev_drive_position = position.copy()
                prev_drive_yaw = float(yaw)
                sim_end_position = position.copy()
                sim_end_yaw = float(yaw)
            if parent_diagnostics is not None:
                parent_diagnostics.record_scheduled("imu")
                if bool(args.publish_odom_prior):
                    parent_diagnostics.record_scheduled("odom_prior")
            if (
                drop_sensor_tick
                and pacing_controller is not None
                and unified_sim_hardware_clock
                and sim_time_s + 1e-9 >= next_lidar_sim_s
            ):
                pacing_controller.force_sensor_observation(reason="lidar_due")
                drop_sensor_tick = False
            if drop_sensor_tick:
                catch_up_stage_start = time.monotonic()
                if pacing_controller is not None:
                    pacing_controller.record_catch_up_step(
                        fast_static_clock=fast_static_clock,
                    )
                imu_sequence += 1
                if bool(args.publish_odom_prior):
                    odom_prior_sequence += 1
                if str(args.scan_time_profile) == "physical_rolling" and pacing_controller is not None:
                    pacing_controller.record_lidar_subscan_drop()
                next_lidar_sim_s, dropped_lidar_frames = _advance_skipped_lidar_deadlines(
                    next_lidar_sim_s=next_lidar_sim_s,
                    lidar_period_s=lidar_period_s,
                    sim_time_s=sim_time_s,
                )
                sequence += dropped_lidar_frames
                if parent_diagnostics is not None:
                    parent_diagnostics.record_catchup_drop("imu")
                    if bool(args.publish_odom_prior):
                        parent_diagnostics.record_catchup_drop("odom_prior")
                    if dropped_lidar_frames > 0:
                        parent_diagnostics.record_scheduled("cloud", dropped_lidar_frames)
                        parent_diagnostics.record_catchup_drop("cloud", dropped_lidar_frames)
                        parent_diagnostics.record_deadline_skip("cloud", dropped_lidar_frames)
                        if navigation_fixture:
                            parent_diagnostics.record_scheduled(
                                "registered_cloud",
                                dropped_lidar_frames,
                            )
                            parent_diagnostics.record_catchup_drop(
                                "registered_cloud",
                                dropped_lidar_frames,
                            )
                            parent_diagnostics.record_deadline_skip(
                                "registered_cloud",
                                dropped_lidar_frames,
                            )
                if pacing_controller is not None:
                    pacing_controller.record_lidar_frame_drops(dropped_lidar_frames)
                    pacing_controller.yield_if_due()
                if publisher.poll() is not None:
                    raise RuntimeError(f"native DDS sensor publisher exited: {publisher.returncode}")
                loop_end = time.monotonic()
                runtime_stage_profiler.record(
                    "catch_up_bookkeeping",
                    loop_end - catch_up_stage_start,
                    sim_time_s,
                )
                runtime_stage_profiler.record("loop_total", loop_end - loop_start, sim_time_s)
                previous_loop_end_wall_s = loop_end
                continue
            imu_publish_stage_start = time.monotonic()
            wall_ts_s = time.time()
            sensor_ts_s = _sensor_timestamp_s(
                clock=imu_timestamp_clock,
                sim_clock_epoch_s=sim_clock_epoch_s,
                sim_time_s=sim_time_s,
                wall_time_s=wall_ts_s,
            )
            imu_dt = 0.0 if prev_imu_s is None else max(0.0, sensor_ts_s - prev_imu_s)
            imu = _runtime_imu_from_state(
                state,
                sensor_ts_s,
                prev_velocity,
                imu_dt,
                str(args.imu_acc_mode),
                acc_conditioner=imu_acc_conditioner,
            )
            _apply_imu_acc_axis_scale(imu, imu_acc_axis_scale)
            _apply_imu_gyro_axis_scale(imu, imu_gyro_axis_scale)
            if parent_diagnostics is not None:
                parent_diagnostics.record_generated("imu")
            if publisher.stdin is None:
                raise RuntimeError("native publisher stdin closed")
            _write_native_imu(
                publisher.stdin,
                imu,
                imu_sequence,
                parent_diagnostics=parent_diagnostics,
                async_batch=publisher_batch,
            )
            sensor_counts[TOPICS.imu] += 1
            imu_sequence += 1
            if physical_drive_active:
                mujoco_truth_velocity_estimator.update(state.position, sensor_ts_s)
            if bool(args.publish_odom_prior):
                odom_prior_velocity, odom_prior_velocity_valid = odom_prior_velocity_estimator.update(
                    state.position,
                    sensor_ts_s,
                )
                if parent_diagnostics is not None:
                    parent_diagnostics.record_generated("odom_prior")
                _write_native_odom_prior(
                    publisher.stdin,
                    state,
                    sensor_ts_s,
                    odom_prior_sequence,
                    velocity=odom_prior_velocity,
                    has_velocity=odom_prior_velocity_valid,
                    parent_diagnostics=parent_diagnostics,
                    async_batch=publisher_batch,
                )
                sensor_counts[TOPICS.odom_prior] += 1
                odom_prior_sequence += 1
            prev_imu_s = sensor_ts_s
            prev_velocity = np.asarray(state.linear_velocity, dtype=np.float64).copy()
            runtime_stage_profiler.record(
                "imu_prior_publish",
                time.monotonic() - imu_publish_stage_start,
                sim_time_s,
            )
            if str(args.scan_time_profile) == "physical_rolling":
                lidar_subscan_stage_start = time.monotonic()
                sample_world_points = _bounded_points(
                    engine.get_lidar_points(sample_count=rolling_subscan_samples),
                    int(args.max_points),
                )
                sample_sensor_points = world_xyzi_to_sensor_xyzi(engine, sample_world_points)
                subscan_samples.append((sim_time_s, sample_sensor_points, sample_world_points, 0))
                min_sample_time = sim_time_s - lidar_period_s * 1.5
                subscan_samples = [sample for sample in subscan_samples if sample[0] >= min_sample_time]
                runtime_stage_profiler.record(
                    "lidar_subscan",
                    time.monotonic() - lidar_subscan_stage_start,
                    sim_time_s,
                )
            lidar_due = (
                sim_time_s + 1e-9 >= next_lidar_sim_s if unified_sim_hardware_clock else loop_start >= next_lidar_s
            )
            if lidar_due:
                if parent_diagnostics is not None:
                    parent_diagnostics.record_scheduled("cloud")
                    if navigation_fixture:
                        parent_diagnostics.record_scheduled("registered_cloud")
                lidar_frame_stage_start = time.monotonic()
                scan_start_sim_s = max(0.0, sim_time_s - lidar_period_s)
                scan_start_wall_s = wall_ts_s - min(lidar_period_s, max(0.0, sim_time_s - scan_start_sim_s))
                scan_stamp_sim_s = _scan_stamp_sim_time_s(
                    scan_time_profile=str(args.scan_time_profile),
                    scan_start_sim_s=scan_start_sim_s,
                    scan_end_sim_s=sim_time_s,
                )
                scan_stamp_wall_s = _scan_stamp_wall_time_s(
                    scan_time_profile=str(args.scan_time_profile),
                    wall_scan_start_s=scan_start_wall_s,
                    wall_scan_end_s=wall_ts_s,
                )
                scan_start_timestamp_s = _scan_start_timestamp_s(
                    clock=lidar_timestamp_clock,
                    sim_clock_epoch_s=sim_clock_epoch_s,
                    scan_start_sim_s=scan_stamp_sim_s,
                    wall_scan_start_s=scan_stamp_wall_s,
                )
                if str(args.scan_time_profile) == "physical_rolling":
                    sensor_points, world_points, relative_times_s, _moving_count, _subscan_count = (
                        _physical_rolling_scan_from_samples(
                            subscan_samples,
                            scan_start_s=scan_start_sim_s,
                            scan_end_s=sim_time_s,
                        )
                    )
                    if int(args.max_points) > 0 and len(sensor_points) > int(args.max_points):
                        stride = int(np.ceil(len(sensor_points) / float(args.max_points)))
                        sensor_points = sensor_points[::stride][: int(args.max_points)]
                        world_points = world_points[::stride][: int(args.max_points)]
                        relative_times_s = relative_times_s[::stride][: int(args.max_points)]
                else:
                    world_points = _bounded_points(engine.get_lidar_points(), int(args.max_points))
                    sensor_points = world_xyzi_to_sensor_xyzi(engine, world_points)
                    relative_times_s = _relative_times_for_scan(
                        len(sensor_points),
                        lidar_period_s,
                        scan_time_profile=str(args.scan_time_profile),
                    )
                latest_world_points = np.asarray(world_points, dtype=np.float32)
                scan = _xyzi_to_livox_frame(
                    sensor_points,
                    timestamp_ns=int(scan_start_timestamp_s * 1_000_000_000),
                    sequence=sequence,
                    frame_id=LIDAR_FRAME_ID,
                    scan_duration_ns=scan_duration_ns,
                    offset_time_ns=(relative_times_s * 1_000_000_000).astype(np.uint64),
                )
                if parent_diagnostics is not None:
                    parent_diagnostics.record_generated("cloud")
                _write_native_scan(
                    publisher.stdin,
                    scan,
                    parent_diagnostics=parent_diagnostics,
                    async_batch=publisher_batch,
                )
                sensor_counts[TOPICS.lidar_scan] += 1
                sequence += 1
                if navigation_fixture:
                    body_points, navigation_fixture_ground_report = navigation_fixture_registered_body_points(
                        world_xyzi_to_body_xyzi(state, world_points),
                        state,
                        max_points=navigation_fixture_cloud_points,
                        raw_overlay_enabled=bool(getattr(args, "navigation_fixture_raw_overlay", True)),
                        ground_resolution_m=navigation_fixture_ground_resolution_m,
                        ground_y_half_m=navigation_fixture_ground_y_half_m,
                    )
                    if parent_diagnostics is not None:
                        parent_diagnostics.record_generated("registered_cloud")
                    _write_native_registered_cloud(
                        publisher.stdin,
                        body_points,
                        timestamp_ns=int(scan_start_timestamp_s * 1_000_000_000),
                        sequence=registered_cloud_sequence,
                        parent_diagnostics=parent_diagnostics,
                        async_batch=publisher_batch,
                    )
                    sensor_counts[TOPICS.registered_cloud] += 1
                    registered_cloud_sequence += 1
                if unified_sim_hardware_clock:
                    next_lidar_sim_s += lidar_period_s
                    skipped_lidar_deadlines = 0
                    while next_lidar_sim_s <= sim_time_s - 1e-9:
                        next_lidar_sim_s += lidar_period_s
                        skipped_lidar_deadlines += 1
                    if parent_diagnostics is not None and skipped_lidar_deadlines > 0:
                        parent_diagnostics.record_scheduled("cloud", skipped_lidar_deadlines)
                        parent_diagnostics.record_deadline_skip("cloud", skipped_lidar_deadlines)
                        if navigation_fixture:
                            parent_diagnostics.record_scheduled(
                                "registered_cloud",
                                skipped_lidar_deadlines,
                            )
                            parent_diagnostics.record_deadline_skip(
                                "registered_cloud",
                                skipped_lidar_deadlines,
                            )
                else:
                    next_lidar_s = loop_start + lidar_period_s
                runtime_stage_profiler.record(
                    "lidar_frame_publish",
                    time.monotonic() - lidar_frame_stage_start,
                    sim_time_s,
                )
            nav_status_snapshot: dict[str, Any] | None = None
            goal_reached_this_tick = False
            if (
                bool(getattr(args, "stop_on_nav_goal_reached", False))
                and driving
                and sim_time_s + 1e-9 >= next_nav_goal_check_sim_s
            ):
                goal_check_stage_start = time.monotonic()
                nav_status_snapshot = _read_json_object(str(getattr(args, "nav_status_json", "") or ""))
                goal_reached_this_tick = _native_nav_goal_reached(nav_status_snapshot)
                runtime_stage_profiler.record(
                    "goal_status_check",
                    time.monotonic() - goal_check_stage_start,
                    sim_time_s,
                )
                next_nav_goal_check_sim_s = sim_time_s + 0.1

            motion_log_due = sim_time_s + 1e-9 >= next_motion_log_sim_s
            if motion_log_writer is not None and (motion_log_due or goal_reached_this_tick):
                motion_log_stage_start = time.monotonic()
                nav_status = nav_status_snapshot
                if nav_status is None:
                    nav_status = _read_json_object(str(getattr(args, "nav_status_json", "") or ""))
                local_planner_debug = nav_status.get("local_candidates") or {}
                local_planner_metrics = nav_status.get("local_planner_debug") or {}
                local_map_debug = nav_status.get("local_map") or {}
                try:
                    nav_status_stamp_s = float(nav_status.get("stamp_s") or 0.0)
                except (TypeError, ValueError):
                    nav_status_stamp_s = 0.0
                try:
                    planner_snapshot_stamp_s = float(local_planner_debug.get("timestamp_s") or 0.0)
                except (TypeError, ValueError):
                    planner_snapshot_stamp_s = 0.0
                planner_debug_key = (nav_status_stamp_s, planner_snapshot_stamp_s)
                if planner_debug_writer is not None and planner_debug_key != last_planner_debug_key:
                    next_planner_debug_id = planner_debug_id + 1
                    if planner_debug_writer.submit(
                        {
                            "id": next_planner_debug_id,
                            "nav_status_stamp_s": nav_status_stamp_s,
                            "local_planner_debug": local_planner_debug,
                            "local_planner_metrics": local_planner_metrics,
                            "local_map": local_map_debug,
                            "global_path": nav_status.get("global_path") or [],
                            "local_path": nav_status.get("local_path") or [],
                            "last_local": nav_status.get("last_local") or {},
                            "input_gate": nav_status.get("input_gate") or {},
                            "dynamic_objects": nav_status.get("dynamic_objects") or [],
                            "timing_ms": nav_status.get("timing_ms") or {},
                            "status_snapshot_writer": nav_status.get("status_snapshot_writer") or {},
                        }
                    ):
                        planner_debug_id = next_planner_debug_id
                        planner_debug_samples += 1
                        last_planner_debug_key = planner_debug_key
                display_points = (
                    _motion_log_lidar_sample(
                        latest_world_points,
                        max_points=motion_log_lidar_points,
                        robot_xyz=position,
                    )
                    if motion_log_lidar_points > 0
                    else np.zeros((0, 4), dtype=np.float32)
                )
                if motion_log_writer.submit(
                    {
                        "t": float(sensor_ts_s),
                        "sim_time_s": float(sim_time_s),
                        "x": float(position[0]),
                        "y": float(position[1]),
                        "z": float(position[2]),
                        "yaw": float(yaw),
                        "driving": bool(driving),
                        "start_xyz": [float(value) for value in parse_start(str(args.start or ""))],
                        "start_yaw_deg": (
                            float(requested_start_yaw_deg)
                            if requested_start_yaw_deg is not None
                            else None
                        ),
                        "qpos": np.asarray(engine.data.qpos, dtype=np.float64).tolist(),
                        "cmd": [float(cmd.linear_x), float(cmd.linear_y), float(cmd.angular_z)],
                        "global_path": nav_status.get("global_path") or [],
                        "local_path": nav_status.get("local_path") or [],
                        "planner_debug_id": planner_debug_id or None,
                        "nav_status_stamp_s": nav_status_stamp_s,
                        "local_reason": str((nav_status.get("last_local") or {}).get("reason") or ""),
                        "local_diagnostics": nav_status.get("last_local") or {},
                        "input_gate": nav_status.get("input_gate") or {},
                        "odom_prior_velocity": odom_prior_velocity_estimator.stats(),
                        "lidar_world": display_points[:, :3].astype(float).tolist(),
                        "acceptance_contacts": _acceptance_contacts(
                            engine.model,
                            engine.data,
                        ),
                    }
                ):
                    motion_log_samples += 1
                if motion_log_due:
                    next_motion_log_sim_s += motion_log_period_s
                    while next_motion_log_sim_s <= sim_time_s - 1e-9:
                        next_motion_log_sim_s += motion_log_period_s
                runtime_stage_profiler.record(
                    "motion_log",
                    time.monotonic() - motion_log_stage_start,
                    sim_time_s,
                )
            if goal_reached_this_tick:
                if async_publisher is not None:
                    publisher_flush_stage_start = time.monotonic()
                    _commit_native_publisher_batch(
                        publisher.stdin,
                        publisher_batch,
                        async_publisher=async_publisher,
                        parent_diagnostics=parent_diagnostics,
                    )
                    if publisher.poll() is not None:
                        raise RuntimeError(f"native DDS sensor publisher exited: {publisher.returncode}")
                    runtime_stage_profiler.record(
                        "publisher_flush",
                        time.monotonic() - publisher_flush_stage_start,
                        sim_time_s,
                    )
                goal_reached_early = True
                loop_end = time.monotonic()
                runtime_stage_profiler.record("loop_total", loop_end - loop_start, sim_time_s)
                previous_loop_end_wall_s = loop_end
                break
            publisher_flush_stage_start = time.monotonic()
            _commit_native_publisher_batch(
                publisher.stdin,
                publisher_batch,
                async_publisher=async_publisher,
                parent_diagnostics=parent_diagnostics,
            )
            if publisher.poll() is not None:
                raise RuntimeError(f"native DDS sensor publisher exited: {publisher.returncode}")
            runtime_stage_profiler.record(
                "publisher_flush",
                time.monotonic() - publisher_flush_stage_start,
                sim_time_s,
            )
            pacing_sleep_stage_start = time.monotonic()
            if unified_sim_hardware_clock:
                hardware_clock.sleep_until(sim_time_s)
            else:
                time.sleep(max(0.0, imu_period_s - (time.monotonic() - loop_start)))
            loop_end = time.monotonic()
            runtime_stage_profiler.record(
                "pacing_sleep",
                loop_end - pacing_sleep_stage_start,
                sim_time_s,
            )
            runtime_stage_profiler.record("loop_total", loop_end - loop_start, sim_time_s)
            previous_loop_end_wall_s = loop_end
        if motion_complete_marker is not None and (
            external_arm_gate is None or motion_interval_completed or goal_reached_early
        ):
            _write_motion_complete_marker(
                motion_complete_marker,
                sim_time_s=last_sim_time_s,
                goal_reached=goal_reached_early,
            )
    finally:
        if pacing_controller is not None:
            pacing_stats = pacing_controller.stats(
                last_sim_time_s,
                monotonic_now_s=time.monotonic(),
            )
        if parent_diagnostics is not None:
            parent_diagnostics.update_scheduler_pacing(pacing_stats)
        if driver_bridge is not None:
            try:
                if driver_heartbeat is not None:
                    try:
                        driver_heartbeat.stop()
                    except Exception as exc:
                        cleanup_errors.append(
                            f"driver_heartbeat_cleanup_failed:{type(exc).__name__}:{exc}"
                        )
                if engine is None:
                    raise RuntimeError("MuJoCo engine unavailable for physical driver shutdown")
                driver_step_seq = _deactivate_driver_bridge(
                    engine,
                    driver_bridge,
                    imu_period_s=imu_period_s,
                    step_seq=driver_step_seq,
                )
            except Exception as exc:
                cleanup_errors.append(f"driver_bridge_physical_shutdown_failed:{type(exc).__name__}:{exc}")
            finally:
                try:
                    driver_bridge.close()
                except Exception as exc:
                    cleanup_errors.append(f"driver_bridge_cleanup_failed:{type(exc).__name__}:{exc}")
                driver_bridge_stats = driver_bridge.stats()
        if motion_log_writer is not None:
            try:
                motion_log_writer_diagnostics = motion_log_writer.close()
            except Exception as exc:
                motion_log_writer_diagnostics = motion_log_writer.diagnostics()
                cleanup_errors.append(f"motion_log_cleanup_failed:{type(exc).__name__}:{exc}")
        if planner_debug_writer is not None:
            try:
                planner_debug_writer_diagnostics = planner_debug_writer.close()
            except Exception as exc:
                planner_debug_writer_diagnostics = planner_debug_writer.diagnostics()
                cleanup_errors.append(f"planner_debug_log_cleanup_failed:{type(exc).__name__}:{exc}")
        try:
            publisher_cleanup = _cleanup_native_publisher(
                publisher,
                async_publisher=async_publisher,
                async_shutdown_s=async_publisher_shutdown_s,
            )
        except Exception as exc:
            cleanup_errors.append(f"publisher_cleanup_failed:{type(exc).__name__}:{exc}")
            publisher_cleanup = {
                "linux_pid": getattr(publisher, "_lingtu_linux_pid", None),
                "clean": False,
                "errors": [cleanup_errors[-1]],
            }
        if parent_diagnostics is not None:
            parent_diagnostics.force_publish(parent_diagnostics.final_reason)
        if viewer is not None:
            try:
                viewer.close()
            except Exception as exc:
                cleanup_errors.append(f"mujoco_viewer_cleanup_failed:{type(exc).__name__}:{exc}")
        if engine is not None:
            try:
                engine.close()
            except Exception as exc:
                cleanup_errors.append(f"mujoco_cleanup_failed:{type(exc).__name__}:{exc}")

    status_counts, slam_status = _slam_status_counts(str(args.slam_status_json or ""))
    slam_counts.update(status_counts)
    motion = _motion_report(
        sim_start_position=sim_start_position,
        sim_end_position=sim_end_position,
        sim_start_yaw=sim_start_yaw,
        sim_end_yaw=sim_end_yaw,
        sim_path_length_xy_m=sim_path_length_xy_m,
        sim_abs_yaw_delta_rad=sim_abs_yaw_delta_rad,
        slam_status=slam_status,
        min_sim_motion_for_odom_check_m=float(args.min_sim_motion_for_odom_check_m),
        min_slam_motion_ratio=float(args.min_slam_motion_ratio),
        max_slam_motion_ratio=float(args.max_slam_motion_ratio),
        min_sim_yaw_for_odom_check_rad=float(args.min_sim_yaw_for_odom_check_rad),
        max_slam_yaw_error_rad=float(args.max_slam_yaw_error_rad),
    )
    gaps = _remaining_gaps(
        sensor_counts=sensor_counts,
        slam_counts=slam_counts,
        require_slam_output=bool(args.require_slam_output),
        require_odom_prior=bool(args.publish_odom_prior),
        drive_mode=str(args.drive_mode),
        allow_kinematic_fastlio_acceptance=bool(args.allow_kinematic_fastlio_acceptance),
    )
    if external_arm_gate is not None:
        external_gap = external_arm_failure_gap or external_arm_gate.failure_gap
        if external_gap:
            gaps.append(external_gap)
        elif not external_arm_gate.acknowledged:
            gaps.append("external_arm_acknowledgement_missing")
        elif not (motion_interval_completed or goal_reached_early):
            gaps.append("external_arm_motion_interval_incomplete")
    if str(args.command_source) == "dds":
        observed_output_ack = driver_bridge_stats.get("observed_output_ack")
        observed_output_ack = observed_output_ack if isinstance(observed_output_ack, dict) else {}
        if bool(driver_bridge_stats.get("failed_before_close")):
            gaps.append("native_driver_bridge_failed")
        if (driver_bridge_stats.get("process_cleanup") or {}).get("clean") is not True:
            gaps.append("native_driver_bridge_cleanup_failed")
        if driver_bridge_stats.get("driver_ready_observed") is not True:
            gaps.append("native_driver_bridge_ready_never_observed")
        if not driver_bridge_stats.get("stopped_evidence"):
            gaps.append("native_driver_bridge_stopped_evidence_missing")
        if (
            driver_bridge_stats.get("driver_ready") is not False
            or int(driver_bridge_stats.get("accepted_sequence") or 0) != 0
            or str(driver_bridge_stats.get("accepted_producer_boot_id") or "")
            or int(driver_bridge_stats.get("accepted_output_sequence") or 0) != 0
        ):
            gaps.append("native_driver_bridge_terminal_authority_not_cleared")
        if bool(args.require_cmd_vel) and int(driver_bridge_stats.get("nonzero_samples") or 0) <= 0:
            gaps.append("native_cmd_vel_nonzero_missing")
        if bool(args.require_cmd_vel) and (
            not str(observed_output_ack.get("producer_boot_id") or "")
            or int(observed_output_ack.get("output_sequence") or 0) <= 0
        ):
            gaps.append("native_driver_bridge_output_ack_missing")
    gaps.extend(_native_sensor_publisher_gaps(publisher_cleanup))
    gaps.extend(cleanup_errors)
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
        gaps.extend(
            _slam_motion_gaps(
                motion,
                min_sim_motion_for_odom_check_m=float(args.min_sim_motion_for_odom_check_m),
                min_slam_motion_ratio=float(args.min_slam_motion_ratio),
                max_slam_motion_ratio=float(args.max_slam_motion_ratio),
                min_sim_yaw_for_odom_check_rad=float(args.min_sim_yaw_for_odom_check_rad),
                max_slam_yaw_error_rad=float(args.max_slam_yaw_error_rad),
                max_slam_map_xy_error_m=float(args.max_slam_map_xy_error_m),
            )
        )
    report = _make_report(
        ok=not gaps,
        duration_s=duration_s,
        domain_id=int(args.domain_id),
        sensor_counts=sensor_counts,
        slam_counts=slam_counts,
        require_slam_output=bool(args.require_slam_output),
        remaining_gaps=gaps,
        publisher=str(publisher_path),
        slam_status=slam_status,
        motion=motion,
        scan_time_profile=str(args.scan_time_profile),
        timestamp_clock=str(args.timestamp_clock),
    )
    report["warmup_s"] = warmup_s
    report["external_arm"] = (
        external_arm_gate.snapshot() if external_arm_gate is not None else _external_arm_disabled_report()
    )
    report["motion_interval_completed"] = motion_interval_completed
    report["start_anchor"] = start_anchor
    report["start_anchor_xyz"] = [float(value) for value in anchor_position[:3]]
    report["start_anchor_yaw_deg"] = (
        float(requested_start_yaw_deg)
        if requested_start_yaw_deg is not None
        else None
    )
    report["settle_s"] = settle_s
    report["drive_ramp_s"] = drive_ramp_s
    report["drive_duration_s"] = duration_s
    report["motion_started_sim_s"] = motion_started_sim_s
    report["drive_mode"] = str(args.drive_mode)
    report["command_source"] = str(args.command_source)
    report["viewer"] = {
        "enabled": bool(getattr(args, "viewer", False)),
        "refresh_hz": float(getattr(args, "viewer_hz", 30.0) or 30.0),
        "closed_early": viewer_closed_early,
        "control_authority": "none_presentation_only",
        "navigation_overlay": viewer_overlay,
    }
    report["cmd_vel"] = driver_bridge_stats
    report["native_sensor_publisher_process"] = publisher_cleanup
    report["policy_loaded"] = policy_loaded
    report["drive_profile"] = str(args.drive_profile)
    report["policy_path"] = str(policy_path) if policy_path is not None else ""
    report["policy_runtime"] = policy_runtime_report
    report["physics_integrator_requested"] = physics_integrator_requested
    report["physics_timestep_requested_s"] = physics_timestep_requested_s
    report["physics_integrator"] = physics_integrator_active
    report["physics_timestep_s"] = physics_timestep_s
    report["imu_acc_mode"] = str(args.imu_acc_mode)
    if str(args.imu_acc_mode) == "gravity_only":
        sensor_model = "fastlio_gravity_only_imu"
    elif str(args.imu_acc_mode) == "sensor" and imu_acc_conditioner is not None:
        sensor_model = "mujoco_accelerometer_conditioned_imu"
    elif str(args.imu_acc_mode) == "sensor":
        sensor_model = "mujoco_accelerometer_raw_imu"
    else:
        sensor_model = "legacy_finite_difference_imu"
    report["sim_hardware_sensor_model"] = sensor_model
    report["imu_acc_conditioning"] = str(args.imu_acc_conditioning)
    if imu_acc_conditioner is not None:
        report["imu_acc_conditioner"] = imu_acc_conditioner.stats()
    else:
        report["imu_acc_conditioner"] = {"enabled": False}
    report["allow_kinematic_fastlio_acceptance"] = bool(args.allow_kinematic_fastlio_acceptance)
    report["publish_odom_prior"] = bool(args.publish_odom_prior)
    report["navigation_fixture"] = navigation_fixture
    report["navigation_fixture_cloud_points"] = navigation_fixture_cloud_points
    report["navigation_fixture_ground_coverage"] = navigation_fixture_ground_report
    report["mocap_motion"] = mocap_motion.stats() if mocap_motion is not None else {"enabled": False}
    report["odom_prior_velocity_source"] = "robust_pose_window"
    report["odom_prior_velocity_window_s"] = float(args.odom_prior_velocity_window_s)
    report["odom_prior_velocity"] = odom_prior_velocity_estimator.stats()
    report["mujoco_truth_velocity"] = mujoco_truth_velocity_estimator.stats()
    report["physical_rolling_sample_mode"] = str(args.physical_rolling_sample_mode)
    report["imu_acc_axis_scale"] = [float(v) for v in imu_acc_axis_scale]
    report["imu_acc_axis_scale_source"] = imu_acc_axis_scale_source
    report["imu_gyro_axis_scale"] = [float(v) for v in imu_gyro_axis_scale]
    report["imu_timestamp_clock"] = str(args.imu_timestamp_clock or args.timestamp_clock)
    report["lidar_timestamp_clock"] = str(args.lidar_timestamp_clock or args.timestamp_clock)
    report["clock_profile"] = "sim_hardware" if unified_sim_hardware_clock else "legacy_split_or_wall"
    report["sim_hardware_realtime_factor"] = float(args.sim_hardware_realtime_factor)
    report["native_clock_alignment"] = native_clock_alignment
    report["sim_hardware_pacing"] = pacing_stats
    report["sim_elapsed_s"] = max(0.0, float(last_sim_time_s) - float(sim_start_s))
    report["effective_publish_hz"] = {
        str(topic): (float(count) / report["sim_elapsed_s"] if report["sim_elapsed_s"] > 0.0 else 0.0)
        for topic, count in sensor_counts.items()
    }
    report["lidar_backend"] = lidar_backend_report
    report["motion_log"] = motion_log_path
    report["motion_log_samples"] = int(motion_log_samples)
    report["motion_log_writer"] = motion_log_writer_diagnostics
    report["motion_log_hz"] = float(motion_log_hz)
    report["motion_log_lidar_points"] = int(motion_log_lidar_points)
    report["motion_log_lidar_presentation"] = {
        "sample_after_vertical_filter": True,
        "max_z_above_robot_m": _MOTION_LOG_LIDAR_MAX_Z_ABOVE_ROBOT_M,
        "local_radius_m": _MOTION_LOG_LIDAR_LOCAL_RADIUS_M,
        "local_share": _MOTION_LOG_LIDAR_LOCAL_SHARE,
    }
    report["planner_debug_log"] = planner_debug_path
    report["planner_debug_samples"] = int(planner_debug_samples)
    report["planner_debug_writer"] = planner_debug_writer_diagnostics
    report["goal_reached_early"] = goal_reached_early
    report["runtime_stage_timing"] = runtime_stage_profiler.stats()
    return report


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    parent_diagnostics: ParentSensorDiagnostics | None = None
    try:
        parent_diagnostics_path = str(getattr(args, "parent_diagnostics_json", "") or "")
        if parent_diagnostics_path:
            parent_diagnostics = ParentSensorDiagnostics(
                Path(parent_diagnostics_path),
                period_s=float(args.parent_diagnostics_period_s),
            )
            setattr(args, "_parent_diagnostics", parent_diagnostics)
            parent_diagnostics.install_signal_handlers()
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
            scan_time_profile=str(getattr(args, "scan_time_profile", "") or ""),
            timestamp_clock=str(getattr(args, "timestamp_clock", "") or ""),
            error=str(exc),
        )
    finally:
        if parent_diagnostics is not None:
            if parent_diagnostics.last_published_reason != parent_diagnostics.final_reason:
                parent_diagnostics.force_publish(parent_diagnostics.final_reason)
            parent_diagnostics.restore_signal_handlers()
    text = json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
