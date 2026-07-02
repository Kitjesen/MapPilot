"""C++ SLAM status-file adapter for the field runtime."""

from __future__ import annotations

import json
import logging
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
        status_snapshot_stale_after_s: float | None = None,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._backend_profile = str(backend_profile or "bridge")
        self._status_snapshot_path = str(
            status_snapshot_path
            or os.environ.get("LINGTU_SLAM_STATUS_JSON")
            or DEFAULT_STATUS_SNAPSHOT_PATH
        )
        self._cloud_snapshot_dir = str(
            cloud_snapshot_dir
            or os.environ.get("LINGTU_SLAM_CLOUD_SNAPSHOT_DIR")
            or DEFAULT_CLOUD_SNAPSHOT_DIR
        )
        interval = (
            status_snapshot_interval_s
            if status_snapshot_interval_s is not None
            else os.environ.get("LINGTU_SLAM_STATUS_INTERVAL_S", "0.05")
        )
        self._status_snapshot_interval_s = max(0.05, float(interval))
        stale_after = (
            status_snapshot_stale_after_s
            if status_snapshot_stale_after_s is not None
            else os.environ.get("LINGTU_SLAM_STATUS_STALE_AFTER_S", "0.5")
        )
        self._status_snapshot_stale_after_s = max(0.1, float(stale_after))
        self._status_snapshot_thread: threading.Thread | None = None
        self._status_snapshot_stop = threading.Event()
        self._status_snapshot_mtime_ns = 0
        self._cloud_snapshot_mtime_ns: dict[str, int] = {}
        self._status_snapshot_schema = ""
        self._status_snapshot_stale = True
        self._stale_status_published = False
        self._message_counts: Counter[str] = Counter()
        self._decode_errors: Counter[str] = Counter()
        self._last_message_ts = 0.0
        self._last_status_ts = 0.0
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

    def stop(self) -> None:
        self._status_snapshot_stop.set()
        thread = self._status_snapshot_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._status_snapshot_thread = None
        super().stop()

    def health(self) -> dict[str, Any]:
        snapshot_age_s = time.time() - self._last_status_ts if self._last_status_ts else None
        return {
            **self._backend_status.as_health_fields(),
            "transport": "cpp_status_snapshot",
            "snapshot_backend": "file",
            "status_snapshot_contract": STATUS_SNAPSHOT_SCHEMA,
            "status_snapshot_path": self._status_snapshot_path,
            "cloud_snapshot_dir": self._cloud_snapshot_dir,
            "status_snapshot_schema": self._status_snapshot_schema,
            "status_snapshot_stale_after_s": self._status_snapshot_stale_after_s,
            "status_snapshot_age_s": snapshot_age_s,
            "status_snapshot_stale": self._status_snapshot_stale,
            "payload_contract": "status_and_cloud_snapshots",
            "cloud_payloads": "binary_pointcloud2_snapshot",
            "message_counts": dict(self._message_counts),
            "decode_errors": dict(self._decode_errors),
            "last_message_ts": self._last_message_ts,
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

    def _publish_status_snapshot(self, payload: Mapping[str, Any]) -> None:
        state = str(payload.get("state") or "UNKNOWN").upper()
        stamp_s = _float(payload.get("stamp_s"), time.time())
        quality = _float(payload.get("localization_quality", payload.get("confidence")), 0.0)
        alive = bool(payload.get("alive", False)) and state not in {"LOST", "FAILED", "DIVERGED"}
        status = {
            "state": state,
            "confidence": _float(payload.get("confidence"), quality),
            "quality": quality,
            "backend": str(payload.get("backend") or "cpp_cyclone_slam"),
            "backend_profile": self._backend_profile,
            "health_source": STATUS_SNAPSHOT_HEALTH_SOURCE,
            "source": str(payload.get("source") or "cpp_cyclone_slam"),
            "reason": str(payload.get("reason") or ""),
            "ts": stamp_s,
            "alive": alive,
            "has_odom": bool(payload.get("has_odom", False)),
            "status_target_hz": _float(payload.get("status_target_hz"), 0.0),
            "imu_input_hz": _float(payload.get("imu_input_hz"), 0.0),
            "lidar_input_hz": _float(payload.get("lidar_input_hz"), 0.0),
            "slam_tick_hz": _float(payload.get("slam_tick_hz"), 0.0),
            "processed_scan_hz": _float(payload.get("processed_scan_hz"), 0.0),
            "registered_points": _int(payload.get("registered_points")),
            "map_points": _int(payload.get("map_points")),
            "saved_map_points": _int(payload.get("saved_map_points")),
            "registered_cloud_frame_id": str(
                payload.get("registered_cloud_frame_id") or ""
            ),
            "map_cloud_frame_id": str(payload.get("map_cloud_frame_id") or ""),
            "saved_map_cloud_frame_id": str(
                payload.get("saved_map_cloud_frame_id") or ""
            ),
            "scan_start_s": _float(payload.get("scan_start_s"), 0.0),
            "scan_end_s": _float(payload.get("scan_end_s"), 0.0),
            "last_imu_s": _float(payload.get("last_imu_s"), 0.0),
            "imu_batch": _int(payload.get("imu_batch")),
            "sync_wait_count": _int(payload.get("sync_wait_count")),
            "imu_rollback_count": _int(payload.get("imu_rollback_count")),
            "lidar_rollback_count": _int(payload.get("lidar_rollback_count")),
            "map_loaded": bool(payload.get("map_loaded", False)),
            "map_frame_jump": bool(payload.get("map_frame_jump", False)),
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
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": False,
            "map_save_supported": True,
            "map_save_source": "native_slam_dds_control",
        }
        if isinstance(payload.get("map_odom_tf"), Mapping):
            status["map_odom_tf"] = dict(payload["map_odom_tf"])
        if isinstance(payload.get("gnss_fusion_health"), Mapping):
            status["gnss_fusion_health"] = dict(payload["gnss_fusion_health"])
        if status["map_frame_jump"]:
            status["map_frame_jump_event"] = {
                "source": status["source"],
                "ts": stamp_s,
                "reason": status["reason"],
            }

        odom = _odometry_from_status_snapshot(payload)
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
        self._last_status_ts = time.time()
        self._record_message(TOPICS.localization_health)
        self._status_snapshot_stale = False
        self._stale_status_published = False

    def _poll_cloud_snapshots(self) -> None:
        base = Path(self._cloud_snapshot_dir)
        for attr_name, filename, topic in (
            ("registered_cloud", "registered_cloud.bin", TOPICS.registered_cloud),
            ("map_cloud", "map_cloud.bin", TOPICS.map_cloud),
            ("saved_map", "saved_map_cloud.bin", TOPICS.saved_map_cloud),
        ):
            path = base / filename
            try:
                stat = path.stat()
            except OSError:
                continue
            mtime_ns = int(getattr(stat, "st_mtime_ns", int(stat.st_mtime * 1e9)))
            if mtime_ns == self._cloud_snapshot_mtime_ns.get(filename):
                continue
            self._cloud_snapshot_mtime_ns[filename] = mtime_ns
            try:
                cloud = PointCloud2.decode(path.read_bytes())
            except Exception:
                self._record_decode_error(f"cloud_snapshot:{filename}")
                continue
            getattr(self, attr_name).publish(cloud)
            self._record_message(topic)

    def _publish_stale_if_needed(self) -> None:
        if not self._last_status_ts:
            return
        age_s = time.time() - self._last_status_ts
        if age_s <= self._status_snapshot_stale_after_s or self._stale_status_published:
            return
        self._status_snapshot_stale = True
        self._stale_status_published = True
        status = {
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
