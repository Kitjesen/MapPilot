"""C++ SLAM status-file adapter for the field runtime."""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from collections import Counter
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from runtime.backend_status import BackendStatus
from runtime.module import Module
from runtime.msgs.geometry import Pose, Quaternion, Transform, Vector3
from runtime.msgs.gnss import GnssOdom
from runtime.msgs.map import MapObservationFrame
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import Imu, PointCloud2
from runtime.registry import register
from runtime.runtime_interface import TOPICS, body_frame_id, topic_default_frame_id
from runtime.stream import In, Out
from runtime.tf import FrameTree

logger = logging.getLogger(__name__)

STATUS_SNAPSHOT_SCHEMA = "lingtu.slam.status_snapshot.v1"
DEFAULT_STATUS_SNAPSHOT_PATH = "/tmp/lingtu_slam_status.json"
DEFAULT_CLOUD_SNAPSHOT_DIR = "/dev/shm/lingtu_slam"
STATUS_SNAPSHOT_HEALTH_SOURCE = "slam_runtime"
DEFAULT_MAX_REASONABLE_SPEED_MPS = 5.0
SOURCE_RESET_REASONS = {
    "reset",
    "sensor_time_jump_reset_mapping",
    "sensor_time_jump_relocalization_required",
}


@register(
    "localization_adapter",
    "cpp_slam_status",
    description="Read LingTu C++ SLAM status snapshots into Module ports",
)
@register(
    "localization_adapter",
    "native_slam_status",
    description="Alias for cpp_slam_status",
)
class CppSlamStatusAdapterModule(Module, layer=1):
    """Bridge the C++ SLAM runtime status snapshot into LingTu ports."""

    lidar_scan: Out[PointCloud2]
    imu: Out[Imu]
    registered_cloud: Out[PointCloud2]
    map_observation: Out[MapObservationFrame]
    map_cloud: Out[PointCloud2]
    saved_map: Out[PointCloud2]
    odometry: Out[Odometry]
    localization_quality: Out[float]
    alive: Out[bool]
    localization_status: Out[dict]
    gnss_fusion_health: Out[dict]
    map_odom_tf: Out[dict]
    map_frame_jump_event: Out[dict]
    scene_mode: Out[str]

    visual_odom: In[Odometry]
    gnss_odom: In[GnssOdom]

    def __init__(
        self,
        backend_profile: str = "bridge",
        status_snapshot_path: str | None = None,
        cloud_snapshot_dir: str | None = None,
        status_snapshot_interval_s: float | None = None,
        lidar_scan_snapshot_interval_s: float | None = None,
        status_snapshot_stale_after_s: float | None = None,
        max_reasonable_speed_mps: float | None = None,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._backend_profile = str(backend_profile or "bridge")
        self._status_snapshot_path = str(
            status_snapshot_path or os.environ.get("LINGTU_SLAM_STATUS_JSON") or DEFAULT_STATUS_SNAPSHOT_PATH
        )
        self._cloud_snapshot_dir = str(
            cloud_snapshot_dir or os.environ.get("LINGTU_SLAM_CLOUD_SNAPSHOT_DIR") or DEFAULT_CLOUD_SNAPSHOT_DIR
        )
        interval = (
            status_snapshot_interval_s
            if status_snapshot_interval_s is not None
            else os.environ.get("LINGTU_SLAM_STATUS_INTERVAL_S", "0.05")
        )
        self._status_snapshot_interval_s = max(0.05, float(interval))
        lidar_scan_interval = (
            lidar_scan_snapshot_interval_s
            if lidar_scan_snapshot_interval_s is not None
            else os.environ.get("LINGTU_SLAM_LIDAR_SCAN_INTERVAL_S", "0.02")
        )
        self._lidar_scan_snapshot_interval_s = max(0.01, float(lidar_scan_interval))
        stale_after = (
            status_snapshot_stale_after_s
            if status_snapshot_stale_after_s is not None
            else os.environ.get("LINGTU_SLAM_STATUS_STALE_AFTER_S", "0.5")
        )
        self._status_snapshot_stale_after_s = max(0.1, float(stale_after))
        speed_limit = (
            max_reasonable_speed_mps
            if max_reasonable_speed_mps is not None
            else os.environ.get(
                "LINGTU_SLAM_MAX_REASONABLE_SPEED_MPS",
                str(DEFAULT_MAX_REASONABLE_SPEED_MPS),
            )
        )
        try:
            self._max_reasonable_speed_mps = max(0.1, float(speed_limit))
        except (TypeError, ValueError):
            self._max_reasonable_speed_mps = DEFAULT_MAX_REASONABLE_SPEED_MPS
        self._status_snapshot_thread: threading.Thread | None = None
        self._lidar_scan_snapshot_thread: threading.Thread | None = None
        self._status_snapshot_stop = threading.Event()
        self._status_snapshot_mtime_ns = 0
        self._cloud_snapshot_mtime_ns: dict[str, int] = {}
        self._status_snapshot_schema = ""
        self._status_snapshot_stale = True
        self._status_snapshot_diverged = False
        self._stale_status_published = False
        self._message_counts: Counter[str] = Counter()
        self._decode_errors: Counter[str] = Counter()
        self._last_message_ts = 0.0
        self._last_status_ts = 0.0
        self._last_map_odom_tf: dict[str, Any] | None = None
        self._latest_observation_status: dict[str, Any] | None = None
        self._latest_registered_cloud: PointCloud2 | None = None
        self._last_observation_sequence = 0
        self._last_observation_runtime_id = ""
        self._last_observation_source_epoch: int | None = None
        self._source_progress_runtime_id: str | None = None
        self._source_progress_epoch: int | None = None
        self._last_source_observation_sequence = 0
        self._last_source_observation_stamp_s = 0.0
        self._last_source_progress_monotonic = 0.0
        self._source_data_stale = False
        self._frame_tree = kw.get("frame_tree") or FrameTree.from_robot_config()
        self._backend_status = BackendStatus.configured_as("cpp_slam_status")

    def setup(self) -> None:
        self._status_snapshot_stop.clear()
        self._status_snapshot_thread = threading.Thread(
            target=self._status_snapshot_loop,
            name="cpp_slam_status_reader",
            daemon=True,
        )
        self._status_snapshot_thread.start()
        self._lidar_scan_snapshot_thread = threading.Thread(
            target=self._lidar_scan_snapshot_loop,
            name="cpp_slam_lidar_scan_reader",
            daemon=True,
        )
        self._lidar_scan_snapshot_thread.start()

    def stop(self) -> None:
        self._status_snapshot_stop.set()
        thread = self._status_snapshot_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._status_snapshot_thread = None
        thread = self._lidar_scan_snapshot_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._lidar_scan_snapshot_thread = None
        super().stop()

    def health(self) -> dict[str, Any]:
        snapshot_age_s = time.time() - self._last_status_ts if self._last_status_ts else None
        source_data_age_s = (
            time.monotonic() - self._last_source_progress_monotonic
            if self._last_source_progress_monotonic
            else None
        )
        return {
            **self._backend_status.as_health_fields(),
            "transport": "cpp_status_snapshot",
            "snapshot_backend": "file",
            "status_snapshot_contract": STATUS_SNAPSHOT_SCHEMA,
            "status_snapshot_path": self._status_snapshot_path,
            "cloud_snapshot_dir": self._cloud_snapshot_dir,
            "status_snapshot_schema": self._status_snapshot_schema,
            "status_snapshot_stale_after_s": self._status_snapshot_stale_after_s,
            "lidar_scan_snapshot_interval_s": self._lidar_scan_snapshot_interval_s,
            "status_snapshot_age_s": snapshot_age_s,
            "status_snapshot_stale": self._status_snapshot_stale,
            "status_snapshot_diverged": self._status_snapshot_diverged,
            "source_data_age_s": source_data_age_s,
            "source_data_stale": self._source_data_stale,
            "source_epoch": self._source_progress_epoch,
            "max_reasonable_speed_mps": self._max_reasonable_speed_mps,
            "payload_contract": "status_and_cloud_snapshots",
            "cloud_payloads": "binary_pointcloud2_snapshot",
            "message_counts": dict(self._message_counts),
            "decode_errors": dict(self._decode_errors),
            "last_message_ts": self._last_message_ts,
            "last_observation_sequence": self._last_observation_sequence,
        }

    def _status_snapshot_loop(self) -> None:
        path = Path(self._status_snapshot_path)
        while not self._status_snapshot_stop.wait(self._status_snapshot_interval_s):
            self._poll_cloud_snapshots()
            try:
                stat = path.stat()
            except OSError:
                self._publish_stale_if_needed()
                continue
            mtime_ns = int(getattr(stat, "st_mtime_ns", int(stat.st_mtime * 1e9)))
            if mtime_ns == self._status_snapshot_mtime_ns:
                self._publish_stale_if_needed()
                continue
            self._status_snapshot_mtime_ns = mtime_ns
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
                if isinstance(payload, Mapping):
                    schema = str(payload.get("schema_version") or "")
                    self._status_snapshot_schema = schema
                    if schema != STATUS_SNAPSHOT_SCHEMA:
                        self._record_decode_error("slam_status_schema")
                        self._publish_stale_if_needed()
                        continue
                    self._publish_status_snapshot(payload)
            except Exception:
                self._record_decode_error("slam_status_snapshot")
            self._publish_stale_if_needed()

    def _lidar_scan_snapshot_loop(self) -> None:
        while not self._status_snapshot_stop.wait(self._lidar_scan_snapshot_interval_s):
            self._poll_one_cloud_snapshot(
                "lidar_scan",
                "lidar_scan.bin",
                TOPICS.lidar_scan,
            )

    def _publish_status_snapshot(self, payload: Mapping[str, Any]) -> None:
        reported_state = str(payload.get("state") or "UNKNOWN").upper()
        stamp_s = _float(payload.get("stamp_s"), time.time())
        source_data_age_s = self._update_source_progress(payload)
        quality = _float(payload.get("localization_quality", payload.get("confidence")), 0.0)
        fastlio_velocity = payload.get("fastlio_velocity")
        fastlio_speed_mps: float | None = None
        runaway_velocity = False
        if isinstance(fastlio_velocity, Mapping):
            components = (
                _float(fastlio_velocity.get("x"), math.nan),
                _float(fastlio_velocity.get("y"), math.nan),
                _float(fastlio_velocity.get("z"), math.nan),
            )
            if all(math.isfinite(value) for value in components):
                fastlio_speed_mps = math.sqrt(sum(value * value for value in components))
                runaway_velocity = fastlio_speed_mps > self._max_reasonable_speed_mps
            else:
                runaway_velocity = True
        source_stalled = (
            self._source_data_stale
            and bool(payload.get("alive", False))
            and reported_state not in {"LOST", "FAILED", "DIVERGED"}
        )
        state = reported_state
        reason = str(payload.get("reason") or "")
        if source_stalled:
            state = "STALE"
            reason = "slam_runtime_source_stale"
        if runaway_velocity:
            state = "DIVERGED"
            reason = "fastlio_velocity_out_of_bounds"
        if runaway_velocity or source_stalled:
            quality = 0.0
        alive = bool(payload.get("alive", False)) and state not in {"LOST", "FAILED", "DIVERGED", "STALE"}
        reported_fastlio_velocity = dict(fastlio_velocity) if isinstance(fastlio_velocity, Mapping) else None
        if source_stalled:
            reported_fastlio_velocity = None
        fastlio_degeneracy = _fastlio_degeneracy(payload.get("fastlio_degeneracy"))
        status = {
            "runtime_instance_id": str(payload.get("runtime_instance_id") or ""),
            "source_epoch": _int(payload.get("source_epoch")),
            "state": state,
            "reported_state": reported_state,
            "confidence": 0.0 if runaway_velocity or source_stalled else _float(payload.get("confidence"), quality),
            "quality": quality,
            "backend": str(payload.get("backend") or "cpp_cyclone_slam"),
            "mode": str(payload.get("mode") or ""),
            "backend_profile": self._backend_profile,
            "health_source": STATUS_SNAPSHOT_HEALTH_SOURCE,
            "source": str(payload.get("source") or "cpp_cyclone_slam"),
            "reason": reason,
            "ts": stamp_s,
            "alive": alive,
            "has_odom": bool(payload.get("has_odom", False)) and not (runaway_velocity or source_stalled),
            "source_data_age_s": source_data_age_s,
            "source_data_stale": source_stalled,
            "fastlio_velocity": reported_fastlio_velocity,
            "fastlio_speed_mps": None if source_stalled else fastlio_speed_mps,
            "fastlio_degeneracy": fastlio_degeneracy,
            "max_reasonable_speed_mps": self._max_reasonable_speed_mps,
            "status_target_hz": _float(payload.get("status_target_hz"), 0.0),
            "imu_input_hz": 0.0 if source_stalled else _float(payload.get("imu_input_hz"), 0.0),
            "lidar_input_hz": 0.0 if source_stalled else _float(payload.get("lidar_input_hz"), 0.0),
            "slam_tick_hz": 0.0 if source_stalled else _float(payload.get("slam_tick_hz"), 0.0),
            "processed_scan_hz": 0.0 if source_stalled else _float(payload.get("processed_scan_hz"), 0.0),
            "registered_points": _int(payload.get("registered_points")),
            "observation_sequence": _int(payload.get("observation_sequence")),
            "map_points": _int(payload.get("map_points")),
            "saved_map_points": _int(payload.get("saved_map_points")),
            "registered_cloud_frame_id": str(payload.get("registered_cloud_frame_id") or ""),
            "map_cloud_frame_id": str(payload.get("map_cloud_frame_id") or ""),
            "saved_map_cloud_frame_id": str(payload.get("saved_map_cloud_frame_id") or ""),
            "scan_start_s": _float(payload.get("scan_start_s"), 0.0),
            "scan_end_s": _float(payload.get("scan_end_s"), 0.0),
            "last_imu_s": _float(payload.get("last_imu_s"), 0.0),
            "imu_batch": _int(payload.get("imu_batch")),
            "sync_wait_count": _int(payload.get("sync_wait_count")),
            "imu_rollback_count": _int(payload.get("imu_rollback_count")),
            "lidar_rollback_count": _int(payload.get("lidar_rollback_count")),
            "map_loaded": bool(payload.get("map_loaded", False)),
            "map_frame_jump": bool(payload.get("map_frame_jump", False)),
            "map_frame_jump_sequence": _int(payload.get("map_frame_jump_sequence")),
            "scene_mode": str(payload.get("scene_mode") or "unknown"),
            "buffers": {
                "imu": _int(payload.get("imu_buffer")),
                "lidar": _int(payload.get("lidar_buffer")),
                "imu_batch": _int(payload.get("imu_batch")),
                "dropped_lidar_frames": _int(payload.get("dropped_lidar_frames")),
                "dropped_imu_frames": _int(payload.get("dropped_imu_frames")),
                "sync_wait_count": _int(payload.get("sync_wait_count")),
                "imu_rollback_count": _int(payload.get("imu_rollback_count")),
                "lidar_rollback_count": _int(payload.get("lidar_rollback_count")),
            },
            "relocalization_supported": bool(payload.get("relocalization_supported", False)),
            "saved_map_relocalization_supported": bool(payload.get("saved_map_relocalization_supported", False)),
            "relocalization_state": str(payload.get("relocalization_state") or "unsupported"),
            "last_relocalization_message": str(payload.get("last_relocalization_message") or ""),
            "relocalization_quality": _float(payload.get("relocalization_quality"), -1.0),
            "relocalization_map_body": payload.get("relocalization_map_body"),
            "relocalization_refine_backend": str(payload.get("relocalization_refine_backend") or ""),
            "relocalization_refine_iterations": _int(payload.get("relocalization_refine_iterations")),
            "relocalization_refine_inliers": _int(payload.get("relocalization_refine_inliers")),
            "relocalization_refine_converged": bool(payload.get("relocalization_refine_converged", False)),
            "relocalization_refine_pos_cov_trace": _float(payload.get("relocalization_refine_pos_cov_trace"), -1.0),
            "restart_recovery_supported": False,
            "map_save_supported": True,
            "map_save_source": "native_slam_dds_control",
        }
        if fastlio_degeneracy is not None:
            status.update(
                {
                    "degeneracy_detected": fastlio_degeneracy["detected"],
                    "degenerate_dof_count": fastlio_degeneracy["degenerate_dof_count"],
                    "condition_number": fastlio_degeneracy["condition_number"],
                    "min_eigenvalue": fastlio_degeneracy["min_eigenvalue"],
                    "max_eigenvalue": fastlio_degeneracy["max_eigenvalue"],
                    "effective_ratio": fastlio_degeneracy["effective_ratio"],
                    "pos_cov_trace": fastlio_degeneracy["pos_cov_trace"],
                    "ieskf_iter_num": fastlio_degeneracy["iter_num"],
                    "ieskf_converged": fastlio_degeneracy["converged"],
                }
            )
        runtime_id = str(status.get("runtime_instance_id") or "").strip()
        source_epoch_present = payload.get("source_epoch") is not None
        source_epoch = _int(payload.get("source_epoch")) if source_epoch_present else None
        source_epoch_changed = (
            source_epoch is not None
            and self._last_observation_source_epoch is not None
            and source_epoch != self._last_observation_source_epoch
        )
        source_reset = (
            _source_reset_reason(payload)
            and _int(payload.get("observation_sequence")) < self._last_observation_sequence
        )
        if (
            (runtime_id and runtime_id != self._last_observation_runtime_id)
            or source_epoch_changed
            or source_reset
        ):
            self._last_observation_runtime_id = runtime_id
            self._last_observation_sequence = 0
        if source_epoch is not None:
            self._last_observation_source_epoch = source_epoch
        if isinstance(payload.get("map_odom_tf"), Mapping):
            status["map_odom_tf"] = dict(payload["map_odom_tf"])
        if isinstance(payload.get("gnss_fusion_health"), Mapping):
            status["gnss_fusion_health"] = dict(payload["gnss_fusion_health"])
        if status["map_frame_jump"]:
            jump_event = self._map_frame_jump_event(status, stamp_s)
            if jump_event:
                status["map_frame_jump_event"] = jump_event

        self._status_snapshot_diverged = runaway_velocity
        odom = None if runaway_velocity or source_stalled else _odometry_from_status_snapshot(payload)
        if odom is not None:
            self._frame_tree.update_odometry(odom)
            self.odometry.publish(odom)
            self._record_message(TOPICS.odometry)
        self.localization_status.publish(status)
        self.localization_quality.publish(quality)
        self.alive.publish(alive)
        if isinstance(status.get("map_odom_tf"), dict):
            tf = dict(status["map_odom_tf"])
            self._update_map_odom_tf(tf)
            self.map_odom_tf.publish(tf)
        if isinstance(status.get("gnss_fusion_health"), dict):
            self.gnss_fusion_health.publish(dict(status["gnss_fusion_health"]))
        if isinstance(status.get("map_frame_jump_event"), dict):
            self.map_frame_jump_event.publish(dict(status["map_frame_jump_event"]))
        self.scene_mode.publish(str(status["scene_mode"]))
        if runaway_velocity or source_stalled:
            self._latest_observation_status = None
            self._latest_registered_cloud = None
        else:
            self._latest_observation_status = dict(payload)
            self._publish_map_observation_if_ready()
        self._last_status_ts = time.time()
        self._record_message(TOPICS.localization_health)
        self._status_snapshot_stale = False
        self._stale_status_published = False

    def _update_source_progress(self, payload: Mapping[str, Any]) -> float:
        runtime_id = str(payload.get("runtime_instance_id") or "").strip()
        source_epoch_present = payload.get("source_epoch") is not None
        source_epoch = _int(payload.get("source_epoch")) if source_epoch_present else None
        sequence = _int(payload.get("observation_sequence"))
        observation = payload.get("state_estimation_at_scan")
        if isinstance(observation, Mapping):
            observation_stamp_s = _float(observation.get("stamp_s"), 0.0)
        else:
            observation_stamp_s = _float(payload.get("scan_end_s"), 0.0)
            if observation_stamp_s <= 0.0:
                observation_stamp_s = _float(payload.get("stamp_s"), 0.0)

        now = time.monotonic()
        first_snapshot = self._source_progress_runtime_id is None
        runtime_changed = not first_snapshot and runtime_id != self._source_progress_runtime_id
        source_epoch_changed = (
            source_epoch is not None
            and self._source_progress_epoch is not None
            and source_epoch != self._source_progress_epoch
        )
        source_rollback = (
            sequence < self._last_source_observation_sequence
            or observation_stamp_s + 1e-6 < self._last_source_observation_stamp_s
        )
        explicit_source_reset = _source_reset_reason(payload) and source_rollback
        source_boundary_changed = runtime_changed or source_epoch_changed or explicit_source_reset
        has_source_marker = sequence > 0 or observation_stamp_s > 0.0
        marker_changed = (
            sequence != self._last_source_observation_sequence
            or abs(observation_stamp_s - self._last_source_observation_stamp_s) > 1e-6
        )
        progressed = (
            first_snapshot
            or sequence > self._last_source_observation_sequence
            or observation_stamp_s > self._last_source_observation_stamp_s + 1e-6
            or (source_boundary_changed and has_source_marker and marker_changed)
        )
        self._source_progress_runtime_id = runtime_id
        if source_epoch is not None:
            self._source_progress_epoch = source_epoch
        if progressed:
            self._last_source_observation_sequence = sequence
            self._last_source_observation_stamp_s = observation_stamp_s
            self._last_source_progress_monotonic = now
            self._source_data_stale = False
            return 0.0

        if source_boundary_changed:
            self._last_source_observation_sequence = sequence
            self._last_source_observation_stamp_s = observation_stamp_s

        age_s = max(0.0, now - self._last_source_progress_monotonic)
        self._source_data_stale = age_s > self._status_snapshot_stale_after_s
        return age_s

    def _map_frame_jump_event(self, status: Mapping[str, Any], stamp_s: float) -> dict[str, Any] | None:
        tf = status.get("map_odom_tf")
        if not isinstance(tf, Mapping) or tf.get("valid") is False:
            return {
                "type": "map_frame_jump",
                "source": status["source"],
                "ts": stamp_s,
                "reason": status["reason"],
            }

        current = dict(tf)
        previous = self._last_map_odom_tf
        event: dict[str, Any] = {
            "type": "map_frame_jump",
            "source": status["source"],
            "ts": stamp_s,
            "reason": status["reason"],
            "new_xyz": [
                _float(current.get("tx"), 0.0),
                _float(current.get("ty"), 0.0),
                _float(current.get("tz"), 0.0),
            ],
        }
        if isinstance(previous, Mapping):
            prev_xyz, prev_yaw = _tf_xyz_yaw(previous)
            next_xyz, next_yaw = _tf_xyz_yaw(current)
            event.update(
                {
                    "dt_m": round(_xyz_distance(prev_xyz, next_xyz), 4),
                    "dyaw_deg": round(_yaw_delta_deg(prev_yaw, next_yaw), 2),
                    "old_xyz": [round(v, 4) for v in prev_xyz],
                    "new_xyz": [round(v, 4) for v in next_xyz],
                }
            )
        else:
            event.update(
                {
                    "dt_m": 0.0,
                    "dyaw_deg": 0.0,
                    "old_xyz": None,
                    "baseline_missing": True,
                }
            )
        return event

    def _poll_cloud_snapshots(self) -> None:
        for attr_name, filename, topic in (
            ("registered_cloud", "registered_cloud.bin", TOPICS.registered_cloud),
            ("map_cloud", "map_cloud.bin", TOPICS.map_cloud),
            ("saved_map", "saved_map_cloud.bin", TOPICS.saved_map_cloud),
        ):
            if self._status_snapshot_diverged and attr_name in {
                "registered_cloud",
                "map_cloud",
            }:
                continue
            self._poll_one_cloud_snapshot(attr_name, filename, topic)

    def _poll_one_cloud_snapshot(
        self,
        attr_name: str,
        filename: str,
        topic: str,
    ) -> None:
        path = Path(self._cloud_snapshot_dir) / filename
        try:
            stat = path.stat()
        except OSError:
            return
        mtime_ns = int(getattr(stat, "st_mtime_ns", int(stat.st_mtime * 1e9)))
        if mtime_ns == self._cloud_snapshot_mtime_ns.get(filename):
            return
        self._cloud_snapshot_mtime_ns[filename] = mtime_ns
        try:
            cloud = PointCloud2.decode(path.read_bytes())
        except Exception:
            self._record_decode_error(f"cloud_snapshot:{filename}")
            return
        getattr(self, attr_name).publish(cloud)
        if attr_name == "registered_cloud":
            self._latest_registered_cloud = cloud
            self._publish_map_observation_if_ready()
        self._record_message(topic)

    def _publish_map_observation_if_ready(self) -> None:
        status = self._latest_observation_status
        cloud = self._latest_registered_cloud
        if not isinstance(status, Mapping) or cloud is None:
            return
        sequence = _int(status.get("observation_sequence"))
        if sequence <= self._last_observation_sequence:
            return
        scan_state = status.get("state_estimation_at_scan")
        map_odom_raw = status.get("map_odom_tf")
        if not isinstance(scan_state, Mapping) or not isinstance(map_odom_raw, Mapping):
            return
        scan_stamp = _float(scan_state.get("stamp_s"), 0.0)
        if scan_stamp <= 0.0 or abs(float(cloud.ts) - scan_stamp) > 1e-4:
            return
        scan_pose = scan_state.get("pose")
        if not isinstance(scan_pose, Mapping) or map_odom_raw.get("valid") is not True:
            return
        odom_frame = str(scan_state.get("frame_id") or "")
        sensor_frame = str(scan_state.get("child_frame_id") or "")
        if not odom_frame or not sensor_frame or str(cloud.frame_id) != sensor_frame:
            return
        map_frame = str(map_odom_raw.get("frame_id") or "")
        if not map_frame or str(map_odom_raw.get("child_frame_id") or "") != odom_frame:
            return
        map_odom = Transform(
            translation=Vector3(
                _float(map_odom_raw.get("tx"), 0.0),
                _float(map_odom_raw.get("ty"), 0.0),
                _float(map_odom_raw.get("tz"), 0.0),
            ),
            rotation=Quaternion(
                _float(map_odom_raw.get("qx"), 0.0),
                _float(map_odom_raw.get("qy"), 0.0),
                _float(map_odom_raw.get("qz"), 0.0),
                _float(map_odom_raw.get("qw"), 1.0),
            ),
            frame_id=map_frame,
            child_frame_id=odom_frame,
            ts=scan_stamp,
        )
        odom_sensor = Transform(
            translation=Vector3(
                _float(scan_pose.get("x"), 0.0),
                _float(scan_pose.get("y"), 0.0),
                _float(scan_pose.get("z"), 0.0),
            ),
            rotation=Quaternion(
                _float(scan_pose.get("qx"), 0.0),
                _float(scan_pose.get("qy"), 0.0),
                _float(scan_pose.get("qz"), 0.0),
                _float(scan_pose.get("qw"), 1.0),
            ),
            frame_id=odom_frame,
            child_frame_id=sensor_frame,
            ts=scan_stamp,
        )
        try:
            map_sensor = map_odom + odom_sensor
            observation = MapObservationFrame(
                points=cloud.points,
                sequence=sequence,
                ts=scan_stamp,
                frame_id=map_frame,
                sensor_frame_id=sensor_frame,
                sensor_origin=map_sensor.translation,
                map_sensor_pose=Pose(map_sensor.translation, map_sensor.rotation),
                map_sensor_transform=map_sensor,
                pose_quality={
                    "confidence": _float(status.get("confidence"), 0.0),
                    "localization_quality": _float(status.get("localization_quality"), 0.0),
                    "state": str(status.get("state") or ""),
                    "reason": str(status.get("reason") or ""),
                },
                source="cpp_slam_status:registered_cloud_body",
            )
        except (TypeError, ValueError):
            self._record_decode_error("map_observation")
            return
        self.map_observation.publish(observation)
        self._last_observation_sequence = sequence

    def _publish_stale_if_needed(self) -> None:
        if not self._last_status_ts:
            return
        age_s = time.time() - self._last_status_ts
        if age_s <= self._status_snapshot_stale_after_s or self._stale_status_published:
            return
        self._status_snapshot_stale = True
        self._stale_status_published = True
        latest = self._latest_observation_status or {}
        status = {
            "runtime_instance_id": str(latest.get("runtime_instance_id") or ""),
            "source_epoch": _int(latest.get("source_epoch")),
            "observation_sequence": _int(latest.get("observation_sequence")),
            "map_frame_jump_sequence": _int(latest.get("map_frame_jump_sequence")),
            "state": "STALE",
            "confidence": 0.0,
            "quality": 0.0,
            "backend": "slam_runtime",
            "backend_profile": self._backend_profile,
            "health_source": STATUS_SNAPSHOT_HEALTH_SOURCE,
            "source": "slam_runtime_adapter",
            "reason": "slam_runtime_status_stale",
            "ts": time.time(),
            "alive": False,
            "has_odom": False,
            "status_snapshot_age_s": age_s,
        }
        self.localization_status.publish(status)
        self.localization_quality.publish(0.0)
        self.alive.publish(False)
        self._record_message(TOPICS.localization_health)

    def _record_message(self, topic: str) -> None:
        self._message_counts[topic] += 1
        self._last_message_ts = time.time()

    def _record_decode_error(self, topic: str) -> None:
        self._decode_errors[topic] += 1
        logger.debug("C++ SLAM status decode failed for %s", topic, exc_info=True)

    def _update_map_odom_tf(self, tf: Mapping[str, Any]) -> None:
        if not tf or not tf.get("valid", False):
            return
        self._frame_tree.set_transform(
            Transform(
                translation=Vector3(float(tf["tx"]), float(tf["ty"]), float(tf["tz"])),
                rotation=Quaternion(
                    float(tf["qx"]),
                    float(tf["qy"]),
                    float(tf["qz"]),
                    float(tf["qw"]),
                ),
                frame_id=topic_default_frame_id(TOPICS.map_cloud),
                child_frame_id=topic_default_frame_id(TOPICS.odometry),
                ts=float(tf.get("ts") or time.time()),
            ),
            authority="cpp_slam_status",
        )
        self._last_map_odom_tf = dict(tf)


def _source_reset_reason(payload: Mapping[str, Any]) -> bool:
    reason = str(payload.get("reason") or "").strip().lower()
    return reason in SOURCE_RESET_REASONS


def _fastlio_degeneracy(raw: Any) -> dict[str, Any] | None:
    if not isinstance(raw, Mapping):
        return None

    detected = bool(raw.get("detected", False))
    dof_count = max(0, _int(raw.get("degenerate_dof_count")))
    condition_number = _float(raw.get("condition_number"), 0.0)
    converged = bool(raw.get("converged", False))

    return {
        "detected": detected,
        "degenerate_dof_count": dof_count,
        "condition_number": condition_number,
        "min_eigenvalue": _float(raw.get("min_eigenvalue"), 0.0),
        "max_eigenvalue": _float(raw.get("max_eigenvalue"), 0.0),
        "effective_ratio": _float(raw.get("effective_ratio"), 0.0),
        "pos_cov_trace": _float(raw.get("pos_cov_trace"), 0.0),
        "iter_num": _int(raw.get("iter_num")),
        "converged": converged,
    }


def _odometry_from_status_snapshot(payload: Mapping[str, Any]) -> Odometry | None:
    if not bool(payload.get("has_odom", False)):
        return None
    raw_odom = payload.get("odometry")
    raw_pose = raw_odom.get("pose") if isinstance(raw_odom, Mapping) else None
    if not isinstance(raw_odom, Mapping) or not isinstance(raw_pose, Mapping):
        return None
    return Odometry(
        pose=Pose(
            position=Vector3(
                _float(raw_pose.get("x"), 0.0),
                _float(raw_pose.get("y"), 0.0),
                _float(raw_pose.get("z"), 0.0),
            ),
            orientation=Quaternion(
                _float(raw_pose.get("qx"), 0.0),
                _float(raw_pose.get("qy"), 0.0),
                _float(raw_pose.get("qz"), 0.0),
                _float(raw_pose.get("qw"), 1.0),
            ),
        ),
        ts=_float(payload.get("stamp_s"), 0.0),
        frame_id=str(raw_odom.get("frame_id") or topic_default_frame_id(TOPICS.odometry)),
        child_frame_id=str(raw_odom.get("child_frame_id") or body_frame_id()),
    )


def _tf_xyz_yaw(tf: Mapping[str, Any]) -> tuple[tuple[float, float, float], float]:
    xyz = (
        _float(tf.get("tx"), 0.0),
        _float(tf.get("ty"), 0.0),
        _float(tf.get("tz"), 0.0),
    )
    qx = _float(tf.get("qx"), 0.0)
    qy = _float(tf.get("qy"), 0.0)
    qz = _float(tf.get("qz"), 0.0)
    qw = _float(tf.get("qw"), 1.0)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return xyz, math.degrees(math.atan2(siny_cosp, cosy_cosp))


def _xyz_distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]))


def _yaw_delta_deg(a: float, b: float) -> float:
    delta = (b - a + 180.0) % 360.0 - 180.0
    return abs(delta)


def _float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default
