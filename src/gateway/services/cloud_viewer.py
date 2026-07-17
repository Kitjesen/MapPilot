"""Point-cloud viewer cache and WebSocket fan-out service.

GatewayModule owns HTTP routing and session state.  This service owns the
browser-facing point-cloud buffers: accumulated map cache, current scan cache,
binary fan-out queues, and reset/debug snapshots.
"""

from __future__ import annotations

import asyncio
import os
import threading
import time
from typing import Any, Callable

from gateway.services.cloud_scene_cache import CloudSceneCache
from gateway.services.cloud_ws import CloudWs
from gateway.services.traffic import DEFAULT_CLOUD_QUEUE_MAXSIZE
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2
from runtime.runtime_interface import (
    TOPICS,
    normalize_frame_id,
    runtime_topic_allowed_frame_ids,
    topic_default_frame_id,
)
from runtime.utils.binary_codec import encode_pointcloud

_MAX_BROWSER_CLOUD_POINTS = 1_000_000


def _as_layer_dict(layer: Any) -> dict[str, Any] | None:
    if isinstance(layer, dict):
        return dict(layer)
    to_dict = getattr(layer, "to_dict", None)
    if callable(to_dict):
        try:
            value = to_dict()
        except TypeError:
            value = to_dict(include_payload=True)
        if isinstance(value, dict):
            return dict(value)
    return None


def _finite_label(value: Any) -> int | None:
    try:
        label = int(value)
    except (TypeError, ValueError):
        return None
    if 0 <= label <= 0xFFFF:
        return label
    return None


def _parse_color(value: Any) -> tuple[int, int, int] | None:
    if isinstance(value, str):
        raw = value.strip()
        if raw.startswith("#"):
            raw = raw[1:]
        if len(raw) == 6:
            try:
                return (
                    int(raw[0:2], 16),
                    int(raw[2:4], 16),
                    int(raw[4:6], 16),
                )
            except ValueError:
                return None
        return None
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        try:
            rgb = [float(value[0]), float(value[1]), float(value[2])]
        except (TypeError, ValueError):
            return None
        if all(0.0 <= channel <= 1.0 for channel in rgb):
            return tuple(int(round(channel * 255.0)) for channel in rgb)  # type: ignore[return-value]
        if all(0.0 <= channel <= 255.0 for channel in rgb):
            return tuple(int(round(channel)) for channel in rgb)  # type: ignore[return-value]
    return None


def _palette_entries(raw: Any) -> dict[int, dict[str, Any]]:
    if not isinstance(raw, dict):
        return {}
    entries: dict[int, dict[str, Any]] = {}
    for key, value in raw.items():
        label = _finite_label(key)
        if label is None:
            continue
        if isinstance(value, dict):
            color = _parse_color(value.get("color") or value.get("rgb") or value.get("hex"))
            name = value.get("name") or value.get("label")
        else:
            color = _parse_color(value)
            name = None
        item: dict[str, Any] = {}
        if color is not None:
            item["color"] = "#{:02x}{:02x}{:02x}".format(*color)
        if name is not None:
            item["name"] = str(name)
        if item:
            entries[label] = item
    return entries


def _fallback_label_color(label: int) -> tuple[int, int, int]:
    hue = ((label * 47) % 360) / 360.0
    x = (1.0 - abs((hue * 6.0) % 2.0 - 1.0)) * 0.75
    if hue < 1 / 6:
        r, g, b = 0.75, x, 0.18
    elif hue < 2 / 6:
        r, g, b = x, 0.75, 0.18
    elif hue < 3 / 6:
        r, g, b = 0.18, 0.75, x
    elif hue < 4 / 6:
        r, g, b = 0.18, x, 0.75
    elif hue < 5 / 6:
        r, g, b = x, 0.18, 0.75
    else:
        r, g, b = 0.75, 0.18, x
    return int(r * 255), int(g * 255), int(b * 255)


def _semantic_layer_info(layer: dict[str, Any], point_count: int) -> dict[str, Any] | None:
    raw_labels = layer.get("labels")
    if raw_labels is None:
        raw_labels = layer.get("semantic_labels")
    if raw_labels is None:
        raw_labels = layer.get("label_ids")
    if raw_labels is None:
        return None
    if hasattr(raw_labels, "tolist"):
        raw_labels = raw_labels.tolist()
    if not isinstance(raw_labels, (list, tuple)):
        return None
    labels: list[int] = []
    for item in raw_labels:
        label = _finite_label(item)
        if label is None:
            return None
        labels.append(label)
    if len(labels) != point_count:
        return None

    raw_confidence = layer.get("confidence")
    if raw_confidence is None:
        raw_confidence = layer.get("confidences")
    confidence: list[float] | None = None
    if raw_confidence is not None:
        if hasattr(raw_confidence, "tolist"):
            raw_confidence = raw_confidence.tolist()
        if isinstance(raw_confidence, (list, tuple)) and len(raw_confidence) == point_count:
            confidence = []
            for item in raw_confidence:
                try:
                    value = float(item)
                except (TypeError, ValueError):
                    value = 0.0
                confidence.append(max(0.0, min(1.0, value)) if np.isfinite(value) else 0.0)

    palette = _palette_entries(layer.get("palette") or layer.get("label_palette"))
    return {
        "labels": labels,
        "confidence": confidence,
        "palette": palette,
        "taxonomy": layer.get("taxonomy") or layer.get("label_taxonomy"),
        "taxonomy_version": layer.get("taxonomy_version"),
    }


def _semantic_colors(info: dict[str, Any] | None) -> np.ndarray | None:
    if not info:
        return None
    labels = info.get("labels")
    if not isinstance(labels, list):
        return None
    palette = info.get("palette") if isinstance(info.get("palette"), dict) else {}
    colors = np.zeros((len(labels), 3), dtype=np.uint8)
    for index, label in enumerate(labels):
        entry = (palette.get(label) or palette.get(str(label))) if palette else None
        color = _parse_color(entry.get("color")) if isinstance(entry, dict) else None
        colors[index] = color or _fallback_label_color(label)
    return colors


def _semantic_event_layer(layer: dict[str, Any], info: dict[str, Any] | None) -> dict[str, Any]:
    item = {
        key: value
        for key, value in layer.items()
        if key
        not in {
            "payload",
            "cloud",
            "grid",
            "labels",
            "semantic_labels",
            "label_ids",
            "confidence",
            "confidences",
        }
    }
    item["has_labels"] = bool(info)
    if info:
        labels = info.get("labels") if isinstance(info.get("labels"), list) else []
        item["label_count"] = len(labels)
        item["labels"] = labels
        if info.get("confidence") is not None:
            item["confidence"] = info["confidence"]
        item["palette"] = info.get("palette") or {}
        item["taxonomy"] = info.get("taxonomy")
        item["taxonomy_version"] = info.get("taxonomy_version")
    return item


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


class CloudViewerService:
    """Owns live point-cloud viewer state independent of GatewayModule."""

    _MAP_ATTRS = frozenset(
        {
            "_map_points",
            "_map_point_keys",
            "_map_point_stale_counts",
            "_map_cloud_lock",
            "_map_cloud_count",
            "_map_voxel_size",
            "_inv_map_voxel_size",
            "_voxel_hits",
            "_voxel_min_hits",
            "_voxel_key_offset",
            "_map_viewer_stale_grace",
            "_map_viewer_max_stale_drops",
            "_map_viewer_max_points",
            "_last_clean_map_layer_ts",
            "_last_map_scene_ts",
            "_clean_map_layer_prefer_s",
        }
    )
    _CLOUD_ATTRS = {
        "_cloud_lock": "_lock",
        "_latest_cloud_buf": "_latest_buf",
        "_cloud_seq": "_seq",
        "_cloud_subs": "_subs",
        "_cloud_sub_loops": "_sub_loops",
        "_cloud_queue_maxsize": "_queue_maxsize",
        "_cloud_published_frames": "_published_frames",
        "_cloud_dropped_frames": "_dropped_frames",
        "_cloud_max_depth_seen": "_max_depth_seen",
        "_latest_cloud_meta": "_latest_meta",
    }
    _SCAN_ATTRS = {
        "_scan_lock": "_lock",
        "_latest_scan_buf": "_latest_buf",
        "_scan_seq": "_seq",
        "_scan_subs": "_subs",
        "_scan_sub_loops": "_sub_loops",
        "_scan_queue_maxsize": "_queue_maxsize",
        "_scan_published_frames": "_published_frames",
        "_scan_dropped_frames": "_dropped_frames",
        "_scan_max_depth_seen": "_max_depth_seen",
        "_latest_scan_meta": "_latest_meta",
    }

    def __getattr__(self, name: str) -> Any:
        if name in self._MAP_ATTRS:
            return getattr(self._map_cache, name)
        mapped = self._CLOUD_ATTRS.get(name)
        if mapped is not None:
            return getattr(self._cloud_ws, mapped)
        mapped = self._SCAN_ATTRS.get(name)
        if mapped is not None:
            return getattr(self._scan_ws, mapped)
        raise AttributeError(f"{type(self).__name__!s} object has no attribute {name!r}")

    def __setattr__(self, name: str, value: Any) -> None:
        if name in type(self)._MAP_ATTRS and "_map_cache" in self.__dict__:
            if name == "_map_voxel_size":
                self.__dict__["_map_cache"].set_voxel_size(float(value))
            else:
                setattr(self.__dict__["_map_cache"], name, value)
            return
        mapped = type(self)._CLOUD_ATTRS.get(name)
        if mapped is not None and "_cloud_ws" in self.__dict__:
            setattr(self.__dict__["_cloud_ws"], mapped, value)
            return
        mapped = type(self)._SCAN_ATTRS.get(name)
        if mapped is not None and "_scan_ws" in self.__dict__:
            setattr(self.__dict__["_scan_ws"], mapped, value)
            return
        super().__setattr__(name, value)

    def __init__(
        self,
        *,
        queue_put_latest: Callable[
            [asyncio.Queue, bytes, asyncio.AbstractEventLoop | None, Callable[[bool, int], None]],
            None,
        ],
        current_loop: Callable[[], asyncio.AbstractEventLoop | None],
        push_event: Callable[[dict[str, Any]], None],
        session_mode: Callable[[], str],
        product_session: Callable[[], str],
        active_session_map: Callable[[], str | None],
        saved_active_map: Callable[[], str | None],
        queue_maxsize: int = DEFAULT_CLOUD_QUEUE_MAXSIZE,
    ) -> None:
        self._queue_put_latest = queue_put_latest
        self._current_loop = current_loop
        self._push_event = push_event
        self._session_mode = session_mode
        self._product_session = product_session
        self._active_session_map = active_session_map
        self._saved_active_map = saved_active_map
        self._scene_lock = threading.RLock()

        self._map_cache = CloudSceneCache()
        self._cloud_ws = CloudWs(
            queue_put_latest=queue_put_latest,
            current_loop=current_loop,
            queue_maxsize=queue_maxsize,
        )
        self._scan_ws = CloudWs(
            queue_put_latest=queue_put_latest,
            current_loop=current_loop,
            queue_maxsize=queue_maxsize,
        )
        self._cloud_viewer_min_interval_s = 1.0 / max(0.1, _env_float("LINGTU_CLOUD_VIEWER_MAX_HZ", 4.0))
        self._cloud_viewer_force_interval_s = max(
            0.2,
            _env_float("LINGTU_CLOUD_VIEWER_FORCE_S", 1.0),
        )
        self._cloud_viewer_min_point_delta = max(
            0,
            int(os.environ.get("LINGTU_CLOUD_VIEWER_MIN_POINT_DELTA", "256")),
        )
        self._last_view_cloud_publish_ts = 0.0
        self._last_view_cloud_publish_cache_points = 0

        self._last_lidar_scan_ts = 0.0
        self._last_slam_map_scan_ts = 0.0
        self._viewer_frame_id: str | None = None
        self._scene_map_id: str | None = None
        self._scene_epoch = 1
        self._wire_cloud_sequence = 0
        self._wire_scan_sequence = 0
        self._scan_incompatible_frame_drops = 0
        self._last_incompatible_scan_frame_id: str | None = None
        self._map_incompatible_frame_drops = 0
        self._last_incompatible_map_frame_id: str | None = None
        self._slam_map_scan_prefer_s = max(
            0.1,
            _env_float("LINGTU_SLAM_MAP_SCAN_PREFER_S", 0.5),
        )
        self._scan_viewer_min_interval_s = 1.0 / max(0.1, _env_float("LINGTU_SCAN_VIEWER_MAX_HZ", 10.0))
        self._scan_viewer_voxel_size = max(
            0.01,
            _env_float("LINGTU_SCAN_VIEWER_VOXEL_SIZE", self._map_voxel_size),
        )
        self._scan_viewer_max_points = min(
            _MAX_BROWSER_CLOUD_POINTS,
            max(
                1000,
                int(os.environ.get("LINGTU_SCAN_VIEWER_MAX_POINTS", "15000")),
            ),
        )
        self._cloud_abs_limit_m = max(
            500.0,
            _env_float("LINGTU_CLOUD_ABS_LIMIT_M", 10000.0),
        )
        self._last_scan_publish_ts = 0.0

    def viewer_scan_allowed_frames(self) -> set[str]:
        frames = runtime_topic_allowed_frame_ids(None).get(
            TOPICS.map_cloud,
            (topic_default_frame_id(TOPICS.map_cloud), topic_default_frame_id(TOPICS.odometry)),
        )
        return {frame for frame in (normalize_frame_id(item) for item in frames) if frame}

    def _next_wire_sequence(self, stream_kind: str) -> int:
        if stream_kind == "scan":
            self._wire_scan_sequence = (self._wire_scan_sequence + 1) & 0xFFFFFFFF
            return self._wire_scan_sequence
        self._wire_cloud_sequence = (self._wire_cloud_sequence + 1) & 0xFFFFFFFF
        return self._wire_cloud_sequence

    def is_viewer_scan_frame_compatible(self, cloud: PointCloud2) -> bool:
        frame_id = normalize_frame_id(getattr(cloud, "frame_id", None))
        if frame_id is None or frame_id not in self.viewer_scan_allowed_frames():
            return False
        return self._viewer_frame_id is None or frame_id == self._viewer_frame_id

    def _accept_map_frame(self, cloud: PointCloud2) -> bool:
        frame_id = normalize_frame_id(getattr(cloud, "frame_id", None))
        if frame_id is None or frame_id not in self.viewer_scan_allowed_frames():
            self._map_incompatible_frame_drops += 1
            self._last_incompatible_map_frame_id = frame_id
            return False
        if self._viewer_frame_id is None:
            self._viewer_frame_id = frame_id
            return True
        if frame_id != self._viewer_frame_id:
            self._map_incompatible_frame_drops += 1
            self._last_incompatible_map_frame_id = frame_id
            return False
        return True

    def on_lidar_scan(self, cloud: PointCloud2) -> None:
        with self._scene_lock:
            self._on_lidar_scan_locked(cloud)

    def _on_lidar_scan_locked(self, cloud: PointCloud2) -> None:
        now = time.time()
        self._last_lidar_scan_ts = now
        if not self.is_viewer_scan_frame_compatible(cloud):
            self._scan_incompatible_frame_drops += 1
            self._last_incompatible_scan_frame_id = normalize_frame_id(getattr(cloud, "frame_id", None))
            return
        if self._last_slam_map_scan_ts > 0.0 and now - self._last_slam_map_scan_ts <= self._slam_map_scan_prefer_s:
            return
        self.handle_scan_cloud(cloud, source="lidar_scan")

    def on_map_cloud(self, cloud: PointCloud2) -> None:
        with self._scene_lock:
            self._on_map_cloud_locked(cloud)

    def _on_map_cloud_locked(self, cloud: PointCloud2) -> None:
        if not self._accept_map_frame(cloud):
            return
        now = time.time()
        self._last_slam_map_scan_ts = now
        self.handle_scan_cloud(
            cloud,
            source="slam_map_cloud",
            fallback=self._last_lidar_scan_ts > 0.0,
        )
        if (
            self._clean_map_layer_prefer_s > 0.0
            and self._last_clean_map_layer_ts > 0.0
            and now - self._last_clean_map_layer_ts <= self._clean_map_layer_prefer_s
        ):
            return
        self.handle_view_cloud(cloud, source="slam_map_cloud", authoritative=False)

    def on_map_scene(self, frame: Any) -> None:
        with self._scene_lock:
            self._on_map_scene_locked(frame)

    def _on_map_scene_locked(self, frame: Any) -> None:
        now = time.time()
        self._last_map_scene_ts = now
        if isinstance(frame, dict):
            layers = list(frame.get("layers") or [])
            frame_dict = dict(frame)
        else:
            layers = list(getattr(frame, "layers", []) or [])
            frame_dict = frame.to_dict(include_payload=False)
        consumed = 0
        event_layers: list[dict[str, Any]] = []
        for layer in layers:
            layer_dict = _as_layer_dict(layer)
            if layer_dict is None:
                continue
            layer_type = str(layer_dict.get("type") or layer_dict.get("layer_type") or "")
            source = str(layer_dict.get("source") or layer_dict.get("id") or "maps.scene")
            payload = layer_dict.get("payload") or layer_dict.get("cloud")
            semantic_info = None
            if layer_type == "pointcloud" and isinstance(payload, PointCloud2):
                if not self._accept_map_frame(payload):
                    continue
                point_count = int(len(payload.points)) if payload.points is not None else 0
                semantic_info = _semantic_layer_info(layer_dict, point_count)
                self._last_clean_map_layer_ts = now
                self.handle_scan_cloud(payload, source=source, fallback=True)
                self.handle_view_cloud(
                    payload,
                    source=source,
                    authoritative=True,
                    colors=_semantic_colors(semantic_info),
                )
                consumed += 1
            event_layers.append(_semantic_event_layer(layer_dict, semantic_info))
        if consumed or event_layers:
            self._push_event(
                {
                    "type": "map_scene",
                    "schema_version": frame_dict.get("schema_version"),
                    "source": frame_dict.get("source") or "maps.scene",
                    "frame_id": frame_dict.get("frame_id"),
                    "sequence": frame_dict.get("sequence"),
                    "map_id": frame_dict.get("map_id"),
                    "metadata": frame_dict.get("metadata") or {},
                    "layers": event_layers or frame_dict.get("layers") or [],
                    "consumed_pointcloud_layers": consumed,
                }
            )

    def on_voxel_cloud(self, cloud: PointCloud2) -> None:
        with self._scene_lock:
            self._on_voxel_cloud_locked(cloud)

    def _on_voxel_cloud_locked(self, cloud: PointCloud2) -> None:
        if not self._accept_map_frame(cloud):
            return
        if time.time() - self._last_map_scene_ts <= 0.25:
            return
        self._last_clean_map_layer_ts = time.time()
        self.handle_scan_cloud(cloud, source="voxel_cloud", fallback=True)
        self.handle_view_cloud(cloud, source="voxel_cloud", authoritative=True)

    def handle_scan_cloud(
        self,
        cloud: PointCloud2,
        *,
        source: str,
        fallback: bool = False,
    ) -> None:
        with self._scene_lock:
            self._handle_scan_cloud_locked(
                cloud,
                source=source,
                fallback=fallback,
            )

    def _handle_scan_cloud_locked(
        self,
        cloud: PointCloud2,
        *,
        source: str,
        fallback: bool = False,
    ) -> None:
        now = time.time()
        latest_scan_source = str(self._latest_scan_meta.get("source") or "")
        priority = {"lidar_scan": 0, "voxel_cloud": 1, "slam_map_cloud": 2}
        source_priority = priority.get(source, 0)
        latest_priority = priority.get(latest_scan_source, 0)
        preferred_overrides_lower = source_priority > latest_priority
        if fallback and now - self._last_scan_publish_ts < 1.0 and not preferred_overrides_lower:
            return
        min_interval_s = self._scan_viewer_min_interval_s
        if source == "lidar_scan":
            min_interval_s *= 0.5
        if now - self._last_scan_publish_ts < min_interval_s and not preferred_overrides_lower:
            return

        pts = cloud.points
        if pts is None or len(pts) == 0:
            return
        pts = pts[:, :3].astype(np.float32)
        finite = np.isfinite(pts).all(axis=1)
        bounded = (np.abs(pts) < self._cloud_abs_limit_m).all(axis=1)
        pts = pts[finite & bounded]
        if len(pts) == 0:
            return

        if source == "lidar_scan":
            scan_pts = pts
        else:
            scan_pts = self.voxel_downsample(pts, self._scan_viewer_voxel_size)
        if len(scan_pts) > self._scan_viewer_max_points:
            stride = max(1, len(scan_pts) // self._scan_viewer_max_points)
            scan_pts = scan_pts[::stride][: self._scan_viewer_max_points]
        if len(scan_pts) == 0:
            return

        z_values = scan_pts[:, 2]
        frame_id = (
            normalize_frame_id(getattr(cloud, "frame_id", None))
            or self._viewer_frame_id
            or topic_default_frame_id(TOPICS.map_cloud)
        )
        try:
            stamp_s = float(getattr(cloud, "ts", 0.0) or now)
        except (TypeError, ValueError):
            stamp_s = now
        if not np.isfinite(stamp_s):
            stamp_s = now
        wire_sequence = self._next_wire_sequence("scan")
        buf = encode_pointcloud(
            scan_pts[:, :3],
            frame_id=frame_id,
            epoch=self._scene_epoch,
            stamp_s=stamp_s,
            sequence=wire_sequence,
            stream_kind="scan",
        )
        self.publish_scan_frame(
            buf,
            metadata={
                "point_count": int(len(scan_pts)),
                "source": source,
                "fallback": bool(fallback),
                "frame_id": frame_id,
                "epoch": self._scene_epoch,
                "wire_sequence": wire_sequence,
                "stamp_s": stamp_s,
                "session_mode": self._session_mode(),
                "z_min": float(np.min(z_values)),
                "z_max": float(np.max(z_values)),
            },
        )
        self._last_scan_publish_ts = now

    def handle_view_cloud(
        self,
        cloud: PointCloud2,
        *,
        source: str,
        authoritative: bool,
        colors: np.ndarray | None = None,
    ) -> None:
        with self._scene_lock:
            self._handle_view_cloud_locked(
                cloud,
                source=source,
                authoritative=authoritative,
                colors=colors,
            )

    def _handle_view_cloud_locked(
        self,
        cloud: PointCloud2,
        *,
        source: str,
        authoritative: bool,
        colors: np.ndarray | None = None,
    ) -> None:
        map_frame_count = self._map_cache.record_frame()
        cloud_points = cloud.points
        if cloud_points is None or len(cloud_points) == 0:
            return
        raw_pts = cloud_points[:, :3].astype(np.float32)
        finite = np.isfinite(raw_pts).all(axis=1)
        bounded = (np.abs(raw_pts) < self._cloud_abs_limit_m).all(axis=1)
        valid = finite & bounded
        pts = raw_pts[valid]
        if len(pts) == 0:
            return
        point_colors: np.ndarray | None = None
        if colors is not None and len(colors) == len(raw_pts):
            colors = np.asarray(colors, dtype=np.uint8)
            point_colors = colors[valid]
        mode = self._session_mode()
        aligned_color_points: np.ndarray | None = None
        aligned_colors: np.ndarray | None = None
        with self._map_cloud_lock:
            if authoritative:
                clean_pts, clean_indices = self.voxel_downsample_with_indices(
                    pts,
                    self._map_voxel_size,
                )
                if mode == "navigating" or self._map_points is None:
                    self.replace_map_points(clean_pts)
                    if point_colors is not None:
                        aligned_color_points = clean_pts
                        aligned_colors = point_colors[clean_indices]
                else:
                    self.merge_map_points_incremental(clean_pts, replace_xy_columns=True)
            elif mode == "navigating" or self._map_points is None:
                sampled_pts, sampled_indices = self.voxel_downsample_with_indices(
                    pts,
                    self._map_voxel_size,
                )
                self.replace_map_points(sampled_pts)
                if point_colors is not None:
                    aligned_color_points = sampled_pts
                    aligned_colors = point_colors[sampled_indices]
            else:
                sampled_pts = self.voxel_downsample(pts, self._map_voxel_size)
                self.merge_map_points_incremental(sampled_pts)
                self._map_cache.record_hits(pts)

        if not authoritative and map_frame_count % 2 != 0:
            return

        pts_all = self._map_cache.snapshot_points()
        if pts_all is None:
            return

        max_send = 60_000
        if len(pts_all) > max_send:
            scale = (len(pts_all) / max_send) ** (1 / 3)
            coarse_voxel = self._map_voxel_size * max(1.0, scale)
            pts_send = self.voxel_downsample(pts_all, coarse_voxel)
            if len(pts_send) > max_send:
                stride = max(1, len(pts_send) // max_send)
                pts_send = pts_send[::stride][:max_send]
        else:
            pts_send = pts_all

        cache_point_count = int(len(pts_all))
        publish_ts = time.time()
        if not self.should_publish_view_cloud(
            mode=mode,
            cache_points=cache_point_count,
            now=publish_ts,
        ):
            return

        z_min = z_max = None
        if len(pts_send) > 0:
            z_values = pts_send[:, 2]
            z_min = float(np.min(z_values))
            z_max = float(np.max(z_values))
        send_colors = None
        if (
            aligned_colors is not None
            and aligned_color_points is not None
            and len(pts_send) == len(pts_all) == len(aligned_colors)
            and np.array_equal(pts_all, aligned_color_points)
        ):
            send_colors = aligned_colors
        frame_id = (
            self._viewer_frame_id
            or normalize_frame_id(getattr(cloud, "frame_id", None))
            or topic_default_frame_id(TOPICS.map_cloud)
        )
        try:
            stamp_s = float(getattr(cloud, "ts", 0.0) or publish_ts)
        except (TypeError, ValueError):
            stamp_s = publish_ts
        if not np.isfinite(stamp_s):
            stamp_s = publish_ts
        wire_sequence = self._next_wire_sequence("map")
        buf = encode_pointcloud(
            pts_send[:, :3],
            colors=send_colors,
            frame_id=frame_id,
            epoch=self._scene_epoch,
            stamp_s=stamp_s,
            sequence=wire_sequence,
            stream_kind="map",
        )
        seq = self.publish_cloud_frame(
            buf,
            metadata={
                "point_count": int(len(pts_send)),
                "cache_points": cache_point_count,
                "source": source,
                "frame_id": frame_id,
                "epoch": self._scene_epoch,
                "wire_sequence": wire_sequence,
                "stamp_s": stamp_s,
                "clean_layer": bool(authoritative),
                "semantic_colors": bool(send_colors is not None),
                "session_mode": mode,
                "z_min": z_min,
                "z_max": z_max,
            },
        )
        self._push_event(
            {
                "type": "map_cloud",
                "count": int(len(pts_send)),
                "seq": seq,
                "bytes": len(buf),
                "source": source,
                "clean_layer": bool(authoritative),
                "semantic_colors": bool(send_colors is not None),
                "z_min": z_min,
                "z_max": z_max,
            }
        )
        self._last_view_cloud_publish_ts = publish_ts
        self._last_view_cloud_publish_cache_points = cache_point_count

    def should_publish_view_cloud(self, *, mode: str, cache_points: int, now: float) -> bool:
        last_ts = self._last_view_cloud_publish_ts
        if last_ts <= 0.0:
            return True
        elapsed = max(0.0, now - last_ts)
        if elapsed < self._cloud_viewer_min_interval_s:
            return False
        if mode in ("mapping", "exploring"):
            delta = cache_points - self._last_view_cloud_publish_cache_points
            if delta < self._cloud_viewer_min_point_delta and elapsed < self._cloud_viewer_force_interval_s:
                return False
        return True

    @staticmethod
    def voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
        sampled, _ = CloudViewerService.voxel_downsample_with_indices(pts, voxel)
        return sampled

    @staticmethod
    def voxel_downsample_with_indices(
        pts: np.ndarray,
        voxel: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        if len(pts) == 0:
            return pts, np.empty((0,), dtype=np.int64)
        keys = (pts[:, :3] / voxel).astype(np.int32)
        _, indices = np.unique(keys, axis=0, return_index=True)
        indices = np.sort(indices).astype(np.int64, copy=False)
        return pts[indices], indices

    def pack_voxel_keys(self, pts_xyz: np.ndarray) -> np.ndarray:
        return self._map_cache.pack_voxel_keys(pts_xyz)

    def pack_xy_voxel_keys(self, pts_xyz: np.ndarray) -> np.ndarray:
        return self._map_cache.pack_xy_voxel_keys(pts_xyz)

    def replace_map_points(self, pts_xyz: np.ndarray) -> None:
        self._map_cache.replace_points(pts_xyz)

    def update_clean_observations_and_mark_stale(self, pts_xyz: np.ndarray) -> None:
        self._map_cache.update_clean_points(pts_xyz)

    def merge_map_points_incremental(
        self,
        pts_xyz: np.ndarray,
        *,
        replace_xy_columns: bool = False,
    ) -> None:
        self._map_cache.merge_points(pts_xyz, replace_xy_columns=replace_xy_columns)

    def publish_cloud_frame(self, buf: bytes, *, metadata: dict[str, Any] | None = None) -> int:
        return self._cloud_ws.publish(buf, metadata=metadata)

    def cloud_subscribe(self) -> tuple[asyncio.Queue, bytes | None]:
        return self._cloud_ws.subscribe()

    def cloud_unsubscribe(self, q: asyncio.Queue) -> None:
        self._cloud_ws.unsubscribe(q)

    def record_cloud_delivery(self, dropped: bool, depth: int) -> None:
        self._cloud_ws.record_delivery(dropped, depth)

    def publish_scan_frame(self, buf: bytes, *, metadata: dict[str, Any] | None = None) -> int:
        return self._scan_ws.publish(buf, metadata=metadata)

    def scan_subscribe(self) -> tuple[asyncio.Queue, bytes | None]:
        return self._scan_ws.subscribe()

    def scan_unsubscribe(self, q: asyncio.Queue) -> None:
        self._scan_ws.unsubscribe(q)

    def record_scan_delivery(self, dropped: bool, depth: int) -> None:
        self._scan_ws.record_delivery(dropped, depth)

    def traffic_snapshot(self) -> dict[str, Any]:
        cloud = self._cloud_ws.traffic()
        scan = self._scan_ws.traffic()
        scan.update(
            {
                "overlay_allowed_frames": sorted(self.viewer_scan_allowed_frames()),
                "viewer_frame_id": self._viewer_frame_id,
                "scene_epoch": self._scene_epoch,
                "incompatible_frame_drops": self._scan_incompatible_frame_drops,
                "last_incompatible_frame_id": self._last_incompatible_scan_frame_id,
                "map_incompatible_frame_drops": self._map_incompatible_frame_drops,
                "last_incompatible_map_frame_id": self._last_incompatible_map_frame_id,
            }
        )
        return {"cloud": cloud, "scan": scan}

    def cache_point_count(self) -> int:
        return self._map_cache.point_count()

    def cache_frames_seen(self) -> int:
        return self._map_cache.frames_seen()

    def map_points_array(self) -> Any:
        return self._map_cache.copy_points()

    def map_summary(self) -> dict[str, Any]:
        return {
            "live_points": self.cache_point_count(),
            "live_cloud_frames": self.cache_frames_seen(),
            "has_live_cloud": self.cache_point_count() > 0,
        }

    def configure(
        self,
        *,
        map_voxel_size: float | None = None,
        voxel_min_hits: int | None = None,
        map_viewer_stale_grace: int | None = None,
        map_viewer_max_points: int | None = None,
        cloud_viewer_min_interval_s: float | None = None,
        cloud_viewer_force_interval_s: float | None = None,
        cloud_viewer_min_point_delta: int | None = None,
        clean_map_layer_prefer_s: float | None = None,
        scan_viewer_min_interval_s: float | None = None,
        slam_map_scan_prefer_s: float | None = None,
        cloud_queue_maxsize: int | None = None,
    ) -> None:
        if map_voxel_size is not None:
            self._map_cache.set_voxel_size(float(map_voxel_size))
            self._scan_viewer_voxel_size = max(0.02, float(map_voxel_size))
        if voxel_min_hits is not None:
            self._map_cache._voxel_min_hits = int(voxel_min_hits)
        if map_viewer_stale_grace is not None:
            self._map_cache._map_viewer_stale_grace = max(1, int(map_viewer_stale_grace))
        if map_viewer_max_points is not None:
            self._map_cache._map_viewer_max_points = max(1, int(map_viewer_max_points))
        if cloud_viewer_min_interval_s is not None:
            self._cloud_viewer_min_interval_s = max(0.0, float(cloud_viewer_min_interval_s))
        if cloud_viewer_force_interval_s is not None:
            self._cloud_viewer_force_interval_s = max(0.0, float(cloud_viewer_force_interval_s))
        if cloud_viewer_min_point_delta is not None:
            self._cloud_viewer_min_point_delta = max(0, int(cloud_viewer_min_point_delta))
        if clean_map_layer_prefer_s is not None:
            self._map_cache._clean_map_layer_prefer_s = max(0.0, float(clean_map_layer_prefer_s))
        if scan_viewer_min_interval_s is not None:
            self._scan_viewer_min_interval_s = max(0.0, float(scan_viewer_min_interval_s))
        if slam_map_scan_prefer_s is not None:
            self._slam_map_scan_prefer_s = max(0.0, float(slam_map_scan_prefer_s))
        if cloud_queue_maxsize is not None:
            size = max(1, int(cloud_queue_maxsize))
            self._cloud_ws._queue_maxsize = size
            self._scan_ws._queue_maxsize = size

    def viewer_config(self) -> dict[str, Any]:
        return {
            "map_voxel_size": self._map_voxel_size,
            "inv_map_voxel_size": self._inv_map_voxel_size,
            "voxel_min_hits": self._voxel_min_hits,
            "map_viewer_stale_grace": self._map_viewer_stale_grace,
            "map_viewer_max_points": self._map_viewer_max_points,
            "cloud_viewer_min_interval_s": self._cloud_viewer_min_interval_s,
            "cloud_viewer_force_interval_s": self._cloud_viewer_force_interval_s,
            "cloud_viewer_min_point_delta": self._cloud_viewer_min_point_delta,
            "clean_map_layer_prefer_s": self._clean_map_layer_prefer_s,
            "scan_viewer_min_interval_s": self._scan_viewer_min_interval_s,
            "slam_map_scan_prefer_s": self._slam_map_scan_prefer_s,
            "cloud_queue_maxsize": self.cloud_queue_maxsize(),
            "scan_queue_maxsize": self.scan_queue_maxsize(),
        }

    def clean_map_layer_prefer_s(self) -> float:
        return float(self._clean_map_layer_prefer_s)

    def mark_clean_map_layer_recent(self, ts: float | None = None) -> None:
        self._last_clean_map_layer_ts = time.time() if ts is None else float(ts)

    def adjust_last_view_publish_ts(self, delta_s: float) -> None:
        self._last_view_cloud_publish_ts += float(delta_s)

    def adjust_last_slam_map_scan_ts(self, delta_s: float) -> None:
        self._last_slam_map_scan_ts += float(delta_s)

    def cloud_queue_maxsize(self) -> int:
        return int(self._cloud_ws._queue_maxsize)

    def scan_queue_maxsize(self) -> int:
        return int(self._scan_ws._queue_maxsize)

    def cloud_published_frames(self) -> int:
        return int(self._cloud_ws.traffic().get("published_frames", 0))

    def scan_published_frames(self) -> int:
        return int(self._scan_ws.traffic().get("published_frames", 0))

    def latest_cloud_metadata(self) -> dict[str, Any]:
        _, latest = self._cloud_ws.latest_debug()
        return dict(latest)

    def scene_identity(self) -> dict[str, Any]:
        """Return one atomic identity for binding non-live map layers."""

        current_map_id = self._current_saved_active_map()
        with self._scene_lock:
            if current_map_id != self._scene_map_id:
                self._clear_locked(
                    reason="active_map_identity_changed",
                    scene_map_id=current_map_id,
                )
            return {
                "protocol_version": 2,
                "frame_id": self._viewer_frame_id or topic_default_frame_id(TOPICS.map_cloud),
                "map_id": self._scene_map_id,
                "epoch": self._scene_epoch,
                "sequence": self._wire_cloud_sequence,
                "stamp_s": time.time(),
            }

    def map_points_snapshot(self, *, max_points: int = 80000) -> dict[str, Any]:
        """Return the browser-facing accumulated map cloud as JSON rows."""
        with self._scene_lock:
            return self._map_points_snapshot_locked(max_points=max_points)

    def _map_points_snapshot_locked(self, *, max_points: int) -> dict[str, Any]:
        try:
            limit = int(max_points)
        except (TypeError, ValueError):
            limit = 80000
        limit = max(1, limit)

        pts = self._map_cache.copy_points()

        snapshot_ts = time.time()
        if pts is None or len(pts) == 0:
            return {
                "schema_version": 1,
                "protocol_version": 2,
                "count": 0,
                "layout": "xyz_rows",
                "frame_id": self._viewer_frame_id or topic_default_frame_id(TOPICS.map_cloud),
                "epoch": self._scene_epoch,
                "sequence": self._wire_cloud_sequence,
                "stamp_s": snapshot_ts,
                "stream_kind": "reset",
                "source": "live_map_cloud",
                "points": [],
                "ts": snapshot_ts,
            }

        if len(pts) > limit:
            idx = np.random.choice(len(pts), limit, replace=False)
            pts = pts[idx]

        return {
            "schema_version": 1,
            "protocol_version": 2,
            "count": len(pts),
            "layout": "xyz_rows",
            "frame_id": self._viewer_frame_id or topic_default_frame_id(TOPICS.map_cloud),
            "epoch": self._scene_epoch,
            "sequence": self._wire_cloud_sequence,
            "stamp_s": snapshot_ts,
            "stream_kind": "map",
            "source": "live_map_cloud",
            "bounds": {
                "x": [float(pts[:, 0].min()), float(pts[:, 0].max())],
                "y": [float(pts[:, 1].min()), float(pts[:, 1].max())],
                "z": [float(pts[:, 2].min()), float(pts[:, 2].max())],
            },
            "points": pts[:, :3].tolist(),
            "ts": snapshot_ts,
        }

    def debug_snapshot(self) -> dict[str, Any]:
        cache_points = self._map_cache.point_count()
        frames_seen = self._map_cache.frames_seen()
        has_latest, latest = self._cloud_ws.latest_debug()
        has_latest_scan, latest_scan = self._scan_ws.latest_debug()
        lidar_scan_age_s = (
            round(max(0.0, time.time() - self._last_lidar_scan_ts), 3) if self._last_lidar_scan_ts > 0.0 else None
        )
        slam_map_scan_age_s = (
            round(max(0.0, time.time() - self._last_slam_map_scan_ts), 3) if self._last_slam_map_scan_ts > 0.0 else None
        )
        session_mode = self._session_mode()
        return {
            "session_mode": session_mode,
            "product_session": self._product_session(),
            "active_map": self._active_session_map() if session_mode == "navigating" else None,
            "saved_active_map": self._saved_active_map(),
            "cache_points": cache_points,
            "frames_seen": frames_seen,
            "viewer_voxel_size_m": self._map_voxel_size,
            "abs_limit_m": self._cloud_abs_limit_m,
            "min_hits": self._voxel_min_hits,
            "viewer_max_hz": round(1.0 / self._cloud_viewer_min_interval_s, 3)
            if self._cloud_viewer_min_interval_s > 0
            else None,
            "viewer_min_point_delta": self._cloud_viewer_min_point_delta,
            "viewer_force_s": self._cloud_viewer_force_interval_s,
            "has_latest_binary_frame": has_latest,
            "latest_frame": latest,
            "latest_map_scene_age_s": round(max(0.0, time.time() - self._last_map_scene_ts), 3)
            if self._last_map_scene_ts > 0.0
            else None,
            "scan_max_hz": round(1.0 / self._scan_viewer_min_interval_s, 3)
            if self._scan_viewer_min_interval_s > 0
            else None,
            "scan_voxel_size_m": self._scan_viewer_voxel_size,
            "scan_max_points": self._scan_viewer_max_points,
            "scan_source_priority": ["slam_map_cloud", "voxel_cloud", "lidar_scan"],
            "viewer_frame_id": self._viewer_frame_id,
            "scene_epoch": self._scene_epoch,
            "scan_overlay_allowed_frames": sorted(self.viewer_scan_allowed_frames()),
            "scan_incompatible_frame_drops": self._scan_incompatible_frame_drops,
            "last_incompatible_scan_frame_id": self._last_incompatible_scan_frame_id,
            "map_incompatible_frame_drops": self._map_incompatible_frame_drops,
            "last_incompatible_map_frame_id": self._last_incompatible_map_frame_id,
            "slam_map_scan_prefer_s": self._slam_map_scan_prefer_s,
            "last_lidar_scan_age_s": lidar_scan_age_s,
            "last_slam_map_scan_age_s": slam_map_scan_age_s,
            "has_latest_scan_frame": has_latest_scan,
            "latest_scan_frame": latest_scan,
        }

    def _current_saved_active_map(self) -> str | None:
        try:
            value = self._saved_active_map()
        except Exception:
            return None
        normalized = str(value or "").strip()
        return normalized or None

    def clear(self, reason: str = "manual_reset") -> int:
        scene_map_id = self._current_saved_active_map()
        with self._scene_lock:
            return self._clear_locked(reason=reason, scene_map_id=scene_map_id)

    def _clear_locked(self, reason: str, *, scene_map_id: str | None) -> int:
        reset_frame_id = self._viewer_frame_id or topic_default_frame_id(TOPICS.map_cloud)
        self._map_cache.clear()
        self._scene_epoch = (self._scene_epoch + 1) & 0xFFFFFFFF
        if self._scene_epoch == 0:
            self._scene_epoch = 1
        self._scene_map_id = scene_map_id
        self._viewer_frame_id = None
        self._scan_incompatible_frame_drops = 0
        self._last_incompatible_scan_frame_id = None
        self._map_incompatible_frame_drops = 0
        self._last_incompatible_map_frame_id = None
        self._last_lidar_scan_ts = 0.0
        self._last_slam_map_scan_ts = 0.0
        self._last_clean_map_layer_ts = 0.0
        self._last_map_scene_ts = 0.0
        self._last_view_cloud_publish_ts = 0.0
        self._last_view_cloud_publish_cache_points = 0
        self._last_scan_publish_ts = 0.0

        now = time.time()
        cloud_wire_sequence = self._next_wire_sequence("map")
        cloud_reset = encode_pointcloud(
            np.empty((0, 3), dtype=np.float32),
            frame_id=reset_frame_id,
            epoch=self._scene_epoch,
            stamp_s=now,
            sequence=cloud_wire_sequence,
            stream_kind="reset",
        )
        seq = self.publish_cloud_frame(
            cloud_reset,
            metadata={
                "point_count": 0,
                "source": "reset",
                "reset": True,
                "reason": reason,
                "frame_id": reset_frame_id,
                "map_id": self._scene_map_id,
                "epoch": self._scene_epoch,
                "wire_sequence": cloud_wire_sequence,
                "stamp_s": now,
            },
        )
        scan_wire_sequence = self._next_wire_sequence("scan")
        scan_reset = encode_pointcloud(
            np.empty((0, 3), dtype=np.float32),
            frame_id=reset_frame_id,
            epoch=self._scene_epoch,
            stamp_s=now,
            sequence=scan_wire_sequence,
            stream_kind="reset",
        )
        self.publish_scan_frame(
            scan_reset,
            metadata={
                "point_count": 0,
                "source": "reset",
                "reset": True,
                "reason": reason,
                "frame_id": reset_frame_id,
                "map_id": self._scene_map_id,
                "epoch": self._scene_epoch,
                "wire_sequence": scan_wire_sequence,
                "stamp_s": now,
            },
        )
        self._push_event(
            {
                "type": "map_cloud",
                "points": [],
                "count": 0,
                "seq": seq,
                "reset": True,
                "reason": reason,
                "frame_id": reset_frame_id,
                "map_id": self._scene_map_id,
                "epoch": self._scene_epoch,
            }
        )
        return seq
