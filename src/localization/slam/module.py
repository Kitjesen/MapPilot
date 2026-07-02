"""Native SLAM Module contract.

SlamModule is an L1 pose and map producer. It does not own navigation planning
outputs: no global_path, local_path, waypoint, cmd_vel, or generic path port.
"""

from __future__ import annotations

import logging
import importlib
import math
import struct
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

from runtime.module import Module, rpc, skill
from runtime.registry import register
from runtime.runtime_interface import (
    TOPICS,
    map_frame_id,
    odom_frame_id,
    topic_default_frame_id,
)
from runtime.stream import In, Out
from runtime.msgs.gnss import GnssOdom
from runtime.msgs.geometry import Pose, Quaternion, Vector3
from runtime.msgs.map import MapCloudFrame
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu, PointCloud2

logger = logging.getLogger(__name__)


NATIVE_SLAM_BINDING_SCHEMA = "lingtu.slam.native.v2"

SLAM_STATES = {
    "UNCONFIGURED",
    "INITIALIZING",
    "MAPPING",
    "LOCALIZING",
    "TRACKING",
    "DEGRADED",
    "LOST",
    "FAILED",
}


def _buffer_status(outputs: dict[str, Any]) -> dict[str, int]:
    return {
        "imu": int(outputs.get("imu_buffer", 0)),
        "lidar": int(outputs.get("lidar_buffer", 0)),
        "dropped_lidar_frames": int(outputs.get("dropped_lidar_frames", 0)),
        "dropped_imu_frames": int(outputs.get("dropped_imu_frames", 0)),
    }


def _sync_status(outputs: dict[str, Any]) -> dict[str, float | int]:
    return {
        "scan_start_s": float(outputs.get("scan_start_s", 0.0)),
        "scan_end_s": float(outputs.get("scan_end_s", 0.0)),
        "last_imu_s": float(outputs.get("last_imu_s", 0.0)),
        "imu_batch": int(outputs.get("imu_batch", 0)),
        "wait_count": int(outputs.get("sync_wait_count", 0)),
        "imu_rollback_count": int(outputs.get("imu_rollback_count", 0)),
        "lidar_rollback_count": int(outputs.get("lidar_rollback_count", 0)),
    }


@register(
    "localization_adapter",
    "native_slam",
    description="Native LingTu SLAM module contract",
)
@register("slam_module", "native_slam", description="Native LingTu SLAM module")
class SlamModule(Module, layer=1):
    """Native SLAM/localization boundary for LingTu.

    The module exposes the same downstream contract as the old bridge, plus
    registered_cloud and state_estimation_at_scan. Navigation path ports are
    intentionally absent.
    """

    visual_odom: In[Odometry]
    gnss_odom: In[GnssOdom]
    lidar_raw_scan: In[Any]
    lidar_scan_in: In[PointCloud2]
    lidar_imu: In[Imu]

    odometry: Out[Odometry]
    registered_cloud: Out[PointCloud2]
    map_cloud_frame: Out[MapCloudFrame]
    map_cloud: Out[PointCloud2]
    saved_map: Out[PointCloud2]
    localization_status: Out[dict]
    localization_quality: Out[float]
    alive: Out[bool]
    map_odom_tf: Out[dict]
    map_frame_jump_event: Out[dict]
    gnss_fusion_health: Out[dict]
    scene_mode: Out[str]

    # Diagnostics when this module directly owns a LiDAR source.
    lidar_scan: Out[PointCloud2]
    imu: Out[Imu]
    state_estimation_at_scan: Out[Odometry]

    def __init__(
        self,
        backend_profile: str = "fastlio2",
        mode: str | None = None,
        map_path: str = "",
        config_path: str = "",
        tick_hz: float = 10.0,
        publish_state_estimation_at_scan: bool = False,
        **config: Any,
    ) -> None:
        super().__init__(**config)
        self._backend_profile = str(backend_profile or "fastlio2").strip().lower()
        self._mode = _mode_from_profile(self._backend_profile, mode)
        self._map_path = str(map_path or "")
        self._config_path = str(config_path or _default_slam_config_path(self._backend_profile))
        self._tick_hz = max(0.1, float(tick_hz))
        self._publish_scan_state = bool(publish_state_estimation_at_scan)
        self._runner = _load_runner(self._backend_profile, self._mode, self._map_path)
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.RLock()
        self._queue_lock = threading.Lock()
        self._imu_queue: deque[Imu] = deque()
        self._lidar_queue: deque[tuple[str, Any]] = deque()
        self._max_imu_queue = max(1, int(config.get("imu_queue_size", 4000)))
        self._max_lidar_queue = max(1, int(config.get("lidar_queue_size", 64)))
        self._dropped_imu = 0
        self._dropped_lidar = 0
        self._last_outputs: dict[str, Any] = {}
        self._last_status: dict[str, Any] = {}
        self._last_gnss_health = _default_gnss_health()
        self._last_scene_mode = "unknown"

    def setup(self) -> None:
        self.visual_odom.set_policy("latest")
        self.gnss_odom.set_policy("latest")
        self.visual_odom.subscribe(self._on_visual_odom)
        self.gnss_odom.subscribe(self._on_gnss_odom)
        self.lidar_raw_scan.subscribe(self.feed_raw_lidar)
        self.lidar_scan_in.subscribe(self.feed_lidar)
        self.lidar_imu.subscribe(self.feed_imu)

    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._configure_runner()
        self._publish_contract_snapshot()
        self._thread = threading.Thread(
            target=self._tick_loop,
            name="slam_module_tick",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._thread = None
        with self._queue_lock:
            self._imu_queue.clear()
            self._lidar_queue.clear()
        super().stop()

    @rpc
    def set_mode(self, mode: str, map_path: str = "") -> dict[str, Any]:
        """Switch SLAM mode without changing the Module graph."""

        normalized = _normalize_mode(mode)
        with self._lock:
            self._mode = normalized
            if map_path:
                self._map_path = map_path
            result = self._runner.setMode(normalized, self._map_path)
            self._publish_contract_snapshot(reason=result.get("message", "set_mode"))
            return dict(result)

    @rpc
    def set_initial_pose(
        self,
        x: float,
        y: float,
        z: float = 0.0,
        yaw: float = 0.0,
    ) -> dict[str, Any]:
        """Set the initial map-frame pose guess."""

        pose = _pose_dict(float(x), float(y), float(z), float(yaw))
        with self._lock:
            result = self._runner.setInitialPose(pose)
            self._publish_contract_snapshot(reason=result.get("message", "initial_pose"))
            return dict(result)

    @skill
    def relocalize(
        self,
        x: float | None = None,
        y: float | None = None,
        z: float = 0.0,
        yaw: float = 0.0,
    ) -> dict[str, Any]:
        """Relocalize against the loaded map, optionally with a pose guess."""

        guess = None
        if x is not None and y is not None:
            guess = _pose_dict(float(x), float(y), float(z), float(yaw))
        with self._lock:
            result = self._runner.relocalize(guess)
            self._publish_contract_snapshot(reason=result.get("message", "relocalize"))
            return dict(result)

    @rpc
    def save_map(self, path: str) -> dict[str, Any]:
        """Save current SLAM map artifacts. Writes map.pcd at minimum."""

        with self._lock:
            result = self._runner.saveMap(path)
            if result.get("ok"):
                self._publish_saved_map_from_path(str(result.get("map_pcd") or path))
            self._publish_contract_snapshot(reason=result.get("message", "save_map"))
            return dict(result)

    @rpc
    def load_map(self, path: str) -> dict[str, Any]:
        """Load a saved map artifact for localization."""

        with self._lock:
            result = self._runner.loadMap(path)
            if result.get("ok"):
                self._publish_saved_map_from_path(str(result.get("map_pcd") or path))
            self._publish_contract_snapshot(reason=result.get("message", "load_map"))
            return dict(result)

    @rpc
    def slam_status(self) -> dict[str, Any]:
        """Return the latest native SLAM status contract."""

        with self._lock:
            if not self._last_status:
                self._publish_contract_snapshot()
            else:
                outputs = dict(self._runner.outputs())
                self._last_status["input_queue"] = self._queue_snapshot()
                self._last_status["buffers"] = _buffer_status(outputs)
                self._last_status["sync"] = _sync_status(outputs)
            return dict(self._last_status)

    def health(self) -> dict[str, Any]:
        """Compatibility health view used by REPL and Gateway diagnostics."""

        status = self.slam_status()
        return {
            "slam": {
                "alive": status.get("alive", False),
                "state": status.get("state", "UNCONFIGURED"),
                "confidence": status.get("confidence", 0.0),
                "reason": status.get("reason", ""),
                "backend": self._backend_profile,
                "mode": self._mode,
            },
            "localization": status,
            "gnss_fusion_health": dict(self._last_gnss_health),
        }

    def _gnss_health_snapshot(self) -> dict[str, Any]:
        """Compatibility hook for old REPL/Gateway status code."""

        return dict(self._last_gnss_health)

    @rpc
    def set_gnss_fusion(self, enabled: bool) -> dict[str, Any]:
        """Enable/disable GNSS fusion health reporting in the native contract."""

        with self._lock:
            self._last_gnss_health["enabled"] = bool(enabled)
            self._last_gnss_health["reason"] = (
                "gnss_fusion_enabled" if enabled else "gnss_fusion_disabled"
            )
            self.gnss_fusion_health.publish(dict(self._last_gnss_health))
            return {"ok": True, "enabled": bool(enabled)}

    @rpc
    def relock_gnss_alignment(self) -> dict[str, Any]:
        """Compatibility relock hook; native backends handle true relock."""

        with self._lock:
            self._last_gnss_health["alignment_locked"] = False
            self._last_gnss_health["relock_count"] = int(
                self._last_gnss_health.get("relock_count", 0)
            ) + 1
            self._last_gnss_health["reason"] = "gnss_relock_requested"
            self.gnss_fusion_health.publish(dict(self._last_gnss_health))
            return {"ok": True, "reason": "gnss_relock_requested"}

    def feed_imu(self, msg: Imu) -> dict[str, Any]:
        """Queue one IMU sample from the sensor callback path."""

        size = self._enqueue(self._imu_queue, msg, self._max_imu_queue, "imu")
        return {"ok": True, "message": "imu_queued", "queue": size}

    def feed_lidar(self, cloud: PointCloud2) -> dict[str, Any]:
        """Queue one LiDAR frame from the sensor callback path."""

        size = self._enqueue(self._lidar_queue, ("cloud", cloud), self._max_lidar_queue, "lidar")
        return {"ok": True, "message": "lidar_queued", "queue": size}

    def feed_raw_lidar(self, frame: Any) -> dict[str, Any]:
        """Queue one lossless Livox frame from the sensor callback path."""

        if getattr(frame, "points", None) is None:
            return {"ok": False, "message": "unsupported_raw_lidar_frame"}
        size = self._enqueue(self._lidar_queue, ("raw", frame), self._max_lidar_queue, "lidar")
        return {"ok": True, "message": "lidar_queued", "queue": size}

    def _enqueue(self, queue: deque, item: Any, limit: int, kind: str) -> int:
        with self._queue_lock:
            if len(queue) >= limit:
                queue.popleft()
                if kind == "imu":
                    self._dropped_imu += 1
                else:
                    self._dropped_lidar += 1
            queue.append(item)
            return len(queue)

    def _drain_inputs(self) -> None:
        with self._lock:
            self._drain_inputs_locked()

    def _drain_inputs_locked(self) -> None:
        with self._queue_lock:
            imu_samples = list(self._imu_queue)
            lidar_frames = list(self._lidar_queue)
            self._imu_queue.clear()
            self._lidar_queue.clear()

        for msg in imu_samples:
            self.imu.publish(msg)
            self._runner.feedImu(_imu_to_dict(msg))

        for kind, item in lidar_frames:
            if kind == "cloud":
                self.lidar_scan.publish(item)
                self._runner.feedLidar(_cloud_to_dict(item))
                continue
            payload = _raw_lidar_to_dict(item)
            if payload is None:
                self._dropped_lidar += 1
                continue
            cloud = _cloud_from_raw_lidar(item)
            if cloud is not None:
                self.lidar_scan.publish(cloud)
            self._runner.feedLidar(payload)

    def _configure_runner(self) -> None:
        result = self._runner.configure(
            {
                "backend": self._backend_profile,
                "mode": self._mode,
                "map_path": self._map_path,
                "config_path": self._config_path,
                "publish_state_estimation_at_scan": self._publish_scan_state,
            }
        )
        if result.get("ok"):
            self._runner.setMode(self._mode, self._map_path)
        else:
            logger.warning("SlamModule backend configure failed: %s", result)

    def _tick_loop(self) -> None:
        interval = 1.0 / self._tick_hz
        while not self._stop_event.wait(interval):
            with self._lock:
                self._drain_inputs_locked()
                result = self._runner.tick()
                self._publish_outputs(reason=result.get("message", "tick"))

    def _on_visual_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._runner.feedVisualOdom(_odom_to_dict(msg))

    def _on_gnss_odom(self, msg: GnssOdom) -> None:
        with self._lock:
            self._runner.feedGnss(_gnss_to_dict(msg))
            self._last_gnss_health = _gnss_health_from_msg(msg)
            self.gnss_fusion_health.publish(dict(self._last_gnss_health))

    def _publish_contract_snapshot(self, reason: str = "startup") -> None:
        self._publish_outputs(reason=reason, force_health=True)

    def _publish_outputs(self, reason: str, force_health: bool = False) -> None:
        outputs = dict(self._runner.outputs())
        self._last_outputs = outputs
        state = str(outputs.get("state") or "INITIALIZING")
        if state not in SLAM_STATES:
            state = "INITIALIZING"
        alive = bool(outputs.get("alive", False))
        quality = _clamp01(outputs.get("localization_quality", outputs.get("confidence", 0.0)))
        status = {
            "schema_version": "lingtu.slam.status.v1",
            "state": state,
            "confidence": quality,
            "reason": str(outputs.get("reason") or reason),
            "backend": self._backend_profile,
            "mode": self._mode,
            "alive": alive,
            "map_loaded": bool(outputs.get("map_loaded", False)),
            "map_frame_jump": bool(outputs.get("map_frame_jump", False)),
            "localization_quality": quality,
            "scene_mode": str(outputs.get("scene_mode") or self._last_scene_mode),
            "saved_map_relocalization_supported": bool(
                outputs.get("saved_map_relocalization_supported", True)
            ),
            "buffers": _buffer_status(outputs),
            "sync": _sync_status(outputs),
            "input_queue": self._queue_snapshot(),
            "stamp_s": float(outputs.get("stamp_s", time.time())),
        }
        self._last_status = status

        odom = _odometry_from_output(outputs.get("odometry_odom_body"))
        if odom is not None:
            self.odometry.publish(odom)
        scan_odom = _odometry_from_output(outputs.get("state_estimation_at_scan"))
        if self._publish_scan_state and scan_odom is not None:
            self.state_estimation_at_scan.publish(scan_odom)
        registered = _cloud_from_output(
            outputs.get("registered_cloud_body"),
            topic_default_frame_id(TOPICS.registered_cloud),
        )
        if registered is not None:
            self.registered_cloud.publish(registered)
        map_cloud = _cloud_from_output(
            outputs.get("map_cloud_map"),
            topic_default_frame_id(TOPICS.map_cloud),
        )
        if map_cloud is not None:
            self.map_cloud_frame.publish(
                MapCloudFrame.from_pointcloud2(
                    map_cloud,
                    mode="FULL",
                    source=f"native_slam:{self._backend_profile}",
                )
            )
            self.map_cloud.publish(map_cloud)
        saved_map_cloud = _cloud_from_output(
            outputs.get("saved_map_cloud_map"),
            topic_default_frame_id(TOPICS.saved_map_cloud),
        )
        if saved_map_cloud is not None:
            self.saved_map.publish(saved_map_cloud)

        tf = outputs.get("map_odom_tf") or _identity_tf_dict()
        self.map_odom_tf.publish(dict(tf))
        if bool(outputs.get("map_frame_jump", False)):
            self.map_frame_jump_event.publish(
                {
                    "type": "map_frame_jump",
                    "stamp_s": status["stamp_s"],
                    "reason": status["reason"],
                }
            )
        self.localization_status.publish(dict(status))
        self.localization_quality.publish(quality)
        self.alive.publish(alive)
        scene_mode = str(outputs.get("scene_mode") or self._last_scene_mode)
        self._last_scene_mode = scene_mode
        self.scene_mode.publish(scene_mode)
        if force_health:
            self.gnss_fusion_health.publish(dict(self._last_gnss_health))

    def _queue_snapshot(self) -> dict[str, int]:
        with self._queue_lock:
            return {
                "lidar": len(self._lidar_queue),
                "imu": len(self._imu_queue),
                "dropped_lidar": self._dropped_lidar,
                "dropped_imu": self._dropped_imu,
            }

    def _publish_saved_map_from_path(self, path: str) -> None:
        pcd = _resolve_map_pcd(path)
        if not pcd.exists():
            return
        points = _read_ascii_pcd_xyz(pcd)
        self.saved_map.publish(
            PointCloud2(
                points=points,
                frame_id=topic_default_frame_id(TOPICS.saved_map_cloud),
            )
        )


class _PythonSlamRunner:
    """Small in-process fallback that enforces the SLAM contract.

    It is not a LIO algorithm. It keeps startup/tests ROS-free while the C++
    Fast-LIO2/Point-LIO implementations bind to the same API.
    """

    def __init__(self, backend: str, mode: str, map_path: str) -> None:
        self.backend = backend
        self.mode = mode
        self.map_path = map_path
        self.configured = False
        self.alive = False
        self.map_loaded = False
        self.reason = "unconfigured"
        self.state = "UNCONFIGURED"
        self.imu_buffer = 0
        self.lidar_buffer = 0
        self.dropped_lidar_frames = 0
        self.dropped_imu_frames = 0
        self.scan_start_s = 0.0
        self.scan_end_s = 0.0
        self.last_imu_s = 0.0
        self.last_lidar_s = 0.0
        self.last_lidar_frame: dict[str, Any] | None = None
        self.imu_batch = 0
        self.sync_wait_count = 0
        self.imu_rollback_count = 0
        self.lidar_rollback_count = 0
        self.last_odom: dict[str, Any] | None = None
        self.pose_history: list[dict[str, Any]] = []
        self.last_map_pcd = ""

    def configure(self, config: dict[str, Any]) -> dict[str, Any]:
        self.backend = str(config.get("backend") or self.backend)
        self.mode = _normalize_mode(config.get("mode") or self.mode)
        self.map_path = str(config.get("map_path") or self.map_path)
        self.configured = True
        self.alive = True
        self.state = "MAPPING" if self.mode == "mapping" else "LOCALIZING"
        self.reason = (
            "pointlio_algorithm_pending_ros_node_extraction"
            if self.backend == "pointlio"
            else "python_contract_backend"
        )
        return {"ok": True, "message": self.reason}

    def setMode(self, mode: str, map_path: str) -> dict[str, Any]:
        self.mode = _normalize_mode(mode)
        if map_path:
            self.map_path = str(map_path)
        self.state = "MAPPING" if self.mode == "mapping" else "LOCALIZING"
        self.reason = "mode_set"
        return {"ok": True, "message": self.reason, "mode": self.mode}

    def feedImu(self, sample: dict[str, Any]) -> dict[str, Any]:
        try:
            stamp_s = float(sample.get("stamp_s", 0.0) or 0.0)
        except (TypeError, ValueError):
            stamp_s = float("nan")
        if not _finite_values(stamp_s, sample.get("angular_velocity", ()), sample.get("linear_acceleration", ())):
            self.dropped_imu_frames += 1
            return {"ok": False, "message": "invalid_imu_sample"}
        if self.last_imu_s > 0.0 and stamp_s < self.last_imu_s:
            self.imu_rollback_count += 1
            self.reason = "imu_time_rollback"
        self.last_imu_s = stamp_s
        self.imu_buffer += 1
        return {"ok": True, "message": "imu_accepted"}

    def feedLidar(self, frame: dict[str, Any]) -> dict[str, Any]:
        if int(frame.get("num_points", 0)) < 0:
            self.dropped_lidar_frames += 1
            return {"ok": False, "message": "invalid_lidar_frame"}
        try:
            stamp_s = float(frame.get("stamp_s", 0.0) or 0.0)
        except (TypeError, ValueError):
            stamp_s = float("nan")
        if not math.isfinite(stamp_s):
            self.dropped_lidar_frames += 1
            return {"ok": False, "message": "invalid_lidar_frame"}
        if self.last_lidar_s > 0.0 and stamp_s < self.last_lidar_s:
            self.lidar_rollback_count += 1
            self.reason = "lidar_time_rollback"
        self.last_lidar_s = stamp_s
        self.scan_start_s = stamp_s
        self.scan_end_s = _scan_end_s(frame)
        self.imu_batch = self.imu_buffer
        if self.last_imu_s > 0.0 and self.last_imu_s < self.scan_end_s:
            self.sync_wait_count += 1
        self.lidar_buffer += 1
        self.last_lidar_frame = frame
        return {"ok": True, "message": "lidar_accepted"}

    def feedGnss(self, sample: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, "message": "gnss_accepted", "fix_type": sample.get("fix_type")}

    def feedVisualOdom(self, odom: dict[str, Any]) -> dict[str, Any]:
        self.last_odom = odom
        self.pose_history.append(odom)
        if len(self.pose_history) > 10000:
            self.pose_history = self.pose_history[-10000:]
        return {"ok": True, "message": "visual_odom_accepted"}

    def setInitialPose(self, pose: dict[str, Any]) -> dict[str, Any]:
        self.last_odom = {"pose": pose, "stamp_s": time.time()}
        self.pose_history.append(self.last_odom)
        return {"ok": True, "message": "initial_pose_set"}

    def relocalize(self, guess: dict[str, Any] | None) -> dict[str, Any]:
        if guess is not None:
            self.last_odom = {"pose": guess, "stamp_s": time.time()}
            self.pose_history.append(self.last_odom)
        if not self.map_loaded and self.mode == "localization":
            return {"ok": False, "message": "map_not_loaded"}
        self.state = "TRACKING"
        self.reason = "relocalized"
        return {"ok": True, "message": self.reason}

    def tick(self) -> dict[str, Any]:
        if not self.configured:
            return {"ok": False, "message": "unconfigured"}
        if self.backend == "pointlio":
            self.reason = "pointlio_algorithm_pending_ros_node_extraction"
            self.state = "DEGRADED"
            return {"ok": True, "message": self.reason}
        self.reason = "tracking" if self.last_odom else "waiting_for_sensor_data"
        if self.last_odom:
            self.state = "TRACKING"
        return {"ok": True, "message": self.reason}

    def saveMap(self, path: str) -> dict[str, Any]:
        pcd = _resolve_map_pcd(path)
        pcd.parent.mkdir(parents=True, exist_ok=True)
        _write_empty_ascii_pcd(pcd)
        if self.pose_history:
            trajectory_path = pcd.parent / "trajectory.txt"
            trajectory_path.write_text(
                "\n".join(_pose_line(item) for item in self.pose_history) + "\n",
                encoding="utf-8",
            )
        patches_dir = pcd.parent / "patches"
        patches_dir.mkdir(exist_ok=True)
        poses_txt = ""
        if self.last_lidar_frame is not None and self.last_odom is not None:
            patch_name = "latest_scan.pcd"
            if _write_lidar_patch_pcd(patches_dir / patch_name, self.last_lidar_frame):
                (pcd.parent / "poses.txt").write_text(
                    _patch_pose_line(patch_name, self.last_odom) + "\n",
                    encoding="utf-8",
                )
                poses_txt = str(pcd.parent / "poses.txt")
        self.map_loaded = True
        self.last_map_pcd = str(pcd)
        return {
            "ok": True,
            "message": "map_saved",
            "map_pcd": str(pcd),
            "poses_txt": poses_txt,
            "trajectory_txt": str(pcd.parent / "trajectory.txt") if self.pose_history else "",
            "patches_dir": str(pcd.parent / "patches"),
        }

    def loadMap(self, path: str) -> dict[str, Any]:
        pcd = _resolve_map_pcd(path)
        if not pcd.exists():
            return {"ok": False, "message": "map_pcd_missing", "map_pcd": str(pcd)}
        self.map_loaded = True
        self.last_map_pcd = str(pcd)
        return {"ok": True, "message": "map_loaded", "map_pcd": str(pcd)}

    def outputs(self) -> dict[str, Any]:
        confidence = 0.0 if self.backend == "pointlio" else 0.8 if self.last_odom else 0.0
        odom = self.last_odom
        return {
            "state": self.state,
            "stamp_s": time.time(),
            "confidence": confidence,
            "reason": self.reason,
            "odometry_odom_body": odom,
            "state_estimation_at_scan": odom,
            "registered_cloud_body": None,
            "map_cloud_map": None,
            "saved_map_cloud_map": None,
            "map_odom_tf": _identity_tf_dict(),
            "alive": self.alive,
            "map_loaded": self.map_loaded,
            "map_frame_jump": False,
            "localization_quality": confidence,
            "gnss_fusion_health": _default_gnss_health(),
            "scene_mode": "unknown",
            "scan_start_s": self.scan_start_s,
            "scan_end_s": self.scan_end_s,
            "last_imu_s": self.last_imu_s,
            "imu_batch": self.imu_batch,
            "sync_wait_count": self.sync_wait_count,
            "imu_rollback_count": self.imu_rollback_count,
            "lidar_rollback_count": self.lidar_rollback_count,
            "imu_buffer": self.imu_buffer,
            "lidar_buffer": self.lidar_buffer,
            "dropped_lidar_frames": self.dropped_lidar_frames,
            "dropped_imu_frames": self.dropped_imu_frames,
            "saved_map_relocalization_supported": True,
        }

    def reset(self) -> dict[str, Any]:
        self.__init__(self.backend, self.mode, self.map_path)
        return {"ok": True, "message": "reset"}


def _load_runner(backend: str, mode: str, map_path: str) -> Any:
    native = _load_native_slam_binding()
    if native is not None:
        try:
            return native.SlamRunner(backend, mode, map_path)
        except Exception as exc:  # pragma: no cover - defensive around optional native extension
            logger.warning("native SLAM runner failed to initialize, using Python contract runner: %s", exc)
    return _PythonSlamRunner(backend, mode, map_path)


def _load_native_slam_binding() -> Any | None:
    try:
        native = importlib.import_module("localization.slam._native")
    except Exception as exc:
        logger.debug("native SLAM binding is not available: %s", exc)
        return None

    schema = str(getattr(native, "NATIVE_SLAM_BINDING_SCHEMA", ""))
    if schema != NATIVE_SLAM_BINDING_SCHEMA:
        logger.warning(
            "ignoring incompatible native SLAM binding schema %r; expected %s",
            schema,
            NATIVE_SLAM_BINDING_SCHEMA,
        )
        return None
    if not hasattr(native, "SlamRunner"):
        logger.warning("ignoring native SLAM binding without SlamRunner")
        return None
    return native


def _mode_from_profile(profile: str, explicit: str | None) -> str:
    if explicit:
        return _normalize_mode(explicit)
    if profile in {"localizer", "genz", "super_lio_relocation", "relocation"}:
        return "localization"
    return "mapping"


def _normalize_mode(mode: Any) -> str:
    value = str(mode or "").strip().lower()
    if value in {"nav", "navigate", "localize", "localizer", "localization"}:
        return "localization"
    if value in {"map", "mapping", "slam", "fastlio2", "pointlio"}:
        return "mapping"
    return "localization" if value == "tracking" else "mapping"


def _default_slam_config_path(profile: str) -> str:
    normalized = str(profile or "").strip().lower()
    if normalized == "pointlio":
        path = Path(__file__).resolve().parents[1] / "pointlio" / "config" / "mid360.yaml"
        return str(path) if path.exists() else ""
    if normalized not in {"fastlio2", "localizer"}:
        return ""
    path = Path(__file__).resolve().parents[1] / "fastlio2" / "config" / "mid360_s100p.yaml"
    return str(path) if path.exists() else ""


def _clamp01(value: Any) -> float:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(f):
        return 0.0
    return max(0.0, min(1.0, f))


def _identity_tf_dict() -> dict[str, Any]:
    return {
        "frame_id": map_frame_id(),
        "child_frame_id": odom_frame_id(),
        "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "stamp_s": time.time(),
    }


def _default_gnss_health() -> dict[str, Any]:
    return {
        "enabled": False,
        "alignment_locked": False,
        "last_fix_type": "NONE",
        "last_gnss_age_s": float("inf"),
        "last_residual_m": 0.0,
        "relock_count": 0,
        "reason": "gnss_not_connected",
    }


def _gnss_health_from_msg(msg: GnssOdom) -> dict[str, Any]:
    age = max(0.0, time.time() - float(msg.ts))
    fix_type = getattr(msg.fix_type, "name", str(msg.fix_type))
    return {
        "enabled": True,
        "alignment_locked": bool(getattr(msg.fix_type, "is_usable", False)),
        "last_fix_type": fix_type,
        "last_gnss_age_s": age,
        "last_residual_m": 0.0,
        "relock_count": 0,
        "horizontal_std_m": float(msg.horizontal_std_m),
        "reason": "gnss_sample_received",
    }


def _pose_dict(x: float, y: float, z: float, yaw: float) -> dict[str, Any]:
    q = Quaternion.from_yaw(yaw)
    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": q.to_dict(),
    }


def _odom_to_dict(msg: Odometry) -> dict[str, Any]:
    return {
        "pose": msg.pose.to_dict(),
        "twist": msg.twist.to_dict(),
        "stamp_s": float(msg.ts),
        "frame_id": msg.frame_id,
        "child_frame_id": msg.child_frame_id,
    }


def _imu_to_dict(msg: Imu) -> dict[str, Any]:
    return {
        "orientation": msg.orientation.to_dict(),
        "orientation_covariance": list(msg.orientation_covariance),
        "angular_velocity": msg.angular_velocity.to_list(),
        "angular_velocity_covariance": list(msg.angular_velocity_covariance),
        "linear_acceleration": msg.linear_acceleration.to_list(),
        "linear_acceleration_covariance": list(msg.linear_acceleration_covariance),
        "stamp_s": float(msg.ts),
        "frame_id": msg.frame_id,
    }


def _cloud_to_dict(msg: PointCloud2) -> dict[str, Any]:
    return {
        "num_points": int(msg.num_points),
        "stamp_s": float(msg.ts),
        "frame_id": msg.frame_id,
        "points": msg.points,
    }


def _raw_lidar_to_dict(frame: Any) -> dict[str, Any] | None:
    points = getattr(frame, "points", None)
    if points is None:
        return None
    point_count = int(getattr(frame, "point_count", len(points)))
    stamp_ns = int(getattr(frame, "timestamp_ns", 0) or 0)
    field_names = getattr(getattr(points, "dtype", None), "names", None)
    return {
        "num_points": point_count,
        "stamp_s": stamp_ns * 1e-9 if stamp_ns > 0 else time.time(),
        "frame_id": str(getattr(frame, "frame_id", "lidar")),
        "format": "livox_xyzi_offset_line_tag",
        "points": points,
        "x": points["x"] if field_names else None,
        "y": points["y"] if field_names else None,
        "z": points["z"] if field_names else None,
        "intensity": points["intensity"] if field_names else None,
        "offset_time_ns": points["offset_time_ns"] if field_names else None,
        "line": points["line"] if field_names else None,
        "tag": points["tag"] if field_names else None,
    }


def _cloud_from_raw_lidar(frame: Any) -> PointCloud2 | None:
    to_xyzi = getattr(frame, "to_xyzi", None)
    if not callable(to_xyzi):
        return None
    stamp_ns = int(getattr(frame, "timestamp_ns", 0) or 0)
    ts = stamp_ns * 1e-9 if stamp_ns > 0 else time.time()
    return PointCloud2.from_numpy(
        to_xyzi(),
        ts=ts,
        frame_id=str(getattr(frame, "frame_id", "lidar")),
    )


def _gnss_to_dict(msg: GnssOdom) -> dict[str, Any]:
    return {
        "position_enu": [float(msg.east), float(msg.north), float(msg.up)],
        "velocity_enu": [float(msg.ve), float(msg.vn), float(msg.vu)],
        "cov_diag": [float(msg.cov_e), float(msg.cov_n), float(msg.cov_u)],
        "fix_type": getattr(msg.fix_type, "name", str(msg.fix_type)),
        "stamp_s": float(msg.ts),
        "frame_id": msg.frame_id,
    }


def _finite_values(*values: Any) -> bool:
    for value in values:
        if isinstance(value, (list, tuple)):
            if not _finite_values(*value):
                return False
            continue
        try:
            if not math.isfinite(float(value)):
                return False
        except (TypeError, ValueError):
            return False
    return True


def _scan_end_s(frame: dict[str, Any]) -> float:
    stamp_s = float(frame.get("stamp_s", 0.0) or 0.0)
    offsets = frame.get("offset_time_ns")
    if offsets is None:
        return stamp_s
    try:
        if hasattr(offsets, "max"):
            max_offset_ns = float(offsets.max())
        else:
            max_offset_ns = float(max(offsets))
    except (TypeError, ValueError):
        return stamp_s
    if not math.isfinite(max_offset_ns) or max_offset_ns < 0.0:
        return stamp_s
    return stamp_s + max_offset_ns * 1e-9


def _odometry_from_output(raw: Any) -> Odometry | None:
    if raw is None:
        return None
    if isinstance(raw, Odometry):
        return raw
    if not isinstance(raw, dict):
        return None
    pose_raw = raw.get("pose") or raw
    position = pose_raw.get("position", {}) if isinstance(pose_raw, dict) else {}
    orientation = pose_raw.get("orientation", {}) if isinstance(pose_raw, dict) else {}
    pose = Pose(
        position=Vector3.from_dict(position),
        orientation=Quaternion.from_dict(orientation),
    )
    return Odometry(
        pose=pose,
        ts=float(raw.get("stamp_s", raw.get("ts", time.time()))),
        frame_id=str(raw.get("frame_id", "odom")),
        child_frame_id=str(raw.get("child_frame_id", "body")),
    )


def _cloud_from_output(raw: Any, frame_id: str) -> PointCloud2 | None:
    if raw is None:
        return None
    if isinstance(raw, PointCloud2):
        return raw
    if isinstance(raw, dict):
        points = raw.get("points")
        if points is None:
            return None
        return PointCloud2(points=points, frame_id=str(raw.get("frame_id") or frame_id))
    return None


def _resolve_map_pcd(path: str) -> Path:
    p = Path(path)
    if p.suffix.lower() == ".pcd":
        return p
    return p / "map.pcd"


def _write_empty_ascii_pcd(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS x y z intensity",
                "SIZE 4 4 4 4",
                "TYPE F F F F",
                "COUNT 1 1 1 1",
                "WIDTH 0",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                "POINTS 0",
                "DATA ascii",
                "",
            ]
        ),
        encoding="utf-8",
    )


def _write_lidar_patch_pcd(path: Path, frame: dict[str, Any]) -> bool:
    points = frame.get("points")
    if points is None:
        return False
    arr = np.asarray(points)
    if arr.size == 0:
        return False
    names = getattr(arr.dtype, "names", None)
    if names:
        required = {"x", "y", "z"}
        if not required.issubset(set(names)):
            return False
        xyz = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float32)
        intensity = (
            np.asarray(arr["intensity"], dtype=np.float32)
            if "intensity" in names
            else np.zeros((arr.shape[0],), dtype=np.float32)
        )
    else:
        arr = np.asarray(arr, dtype=np.float32)
        if arr.ndim != 2 or arr.shape[1] < 3:
            return False
        xyz = arr[:, :3].astype(np.float32, copy=False)
        intensity = (
            arr[:, 3].astype(np.float32, copy=False)
            if arr.shape[1] > 3
            else np.zeros((arr.shape[0],), dtype=np.float32)
        )
    valid = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity)
    xyz = xyz[valid]
    intensity = intensity[valid]
    if xyz.shape[0] == 0:
        return False

    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {xyz.shape[0]}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {xyz.shape[0]}\n"
        "DATA binary\n"
    )
    with path.open("wb") as fh:
        fh.write(header.encode("ascii"))
        for point, value in zip(xyz, intensity):
            fh.write(
                struct.pack(
                    "<ffff",
                    float(point[0]),
                    float(point[1]),
                    float(point[2]),
                    float(value),
                )
            )
    return True


def _read_ascii_pcd_xyz(path: Path) -> list[list[float]]:
    try:
        lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    except OSError:
        return []
    data_idx = None
    for idx, line in enumerate(lines):
        if line.strip().lower().startswith("data "):
            data_idx = idx + 1
            break
    if data_idx is None:
        return []
    points: list[list[float]] = []
    for line in lines[data_idx:]:
        parts = line.split()
        if len(parts) < 3:
            continue
        try:
            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
        except ValueError:
            continue
    return points


def _pose_line(item: dict[str, Any]) -> str:
    pose = item.get("pose", {})
    pos = pose.get("position", {})
    ori = pose.get("orientation", {})
    stamp = float(item.get("stamp_s", time.time()))
    return (
        f"{stamp:.9f} "
        f"{float(pos.get('x', 0.0)):.9f} "
        f"{float(pos.get('y', 0.0)):.9f} "
        f"{float(pos.get('z', 0.0)):.9f} "
        f"{float(ori.get('x', 0.0)):.9f} "
        f"{float(ori.get('y', 0.0)):.9f} "
        f"{float(ori.get('z', 0.0)):.9f} "
        f"{float(ori.get('w', 1.0)):.9f}"
    )


def _patch_pose_line(patch_name: str, item: dict[str, Any]) -> str:
    pose = item.get("pose", {})
    pos = pose.get("position", {})
    ori = pose.get("orientation", {})
    return (
        f"{patch_name} "
        f"{float(pos.get('x', 0.0)):.9f} "
        f"{float(pos.get('y', 0.0)):.9f} "
        f"{float(pos.get('z', 0.0)):.9f} "
        f"{float(ori.get('w', 1.0)):.9f} "
        f"{float(ori.get('x', 0.0)):.9f} "
        f"{float(ori.get('y', 0.0)):.9f} "
        f"{float(ori.get('z', 0.0)):.9f}"
    )
