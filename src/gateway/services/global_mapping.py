"""Read the native whole-map preview; no mapping or accumulation in Python."""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

from gateway.services.native_status import read_json_snapshot
from runtime.msgs.sensor import PointCloud2

_epoch_lock = threading.Lock()
_source_epoch: int | None = None
_viewer_epoch = 0


def _wire_epoch(source: int | None) -> int:
    global _source_epoch, _viewer_epoch
    with _epoch_lock:
        if source != _source_epoch:
            _source_epoch = source
            _viewer_epoch = (_viewer_epoch + 1) & 0xFFFFFFFF
        return _viewer_epoch


def global_mapping_points(max_points: int = 80000) -> dict[str, Any]:
    session = os.environ.get("LINGTU_SESSION_ROOT", "").strip()
    default_status = (
        str(Path(session) / "slam.status.json")
        if session and os.environ.get("LINGTU_ENV") == "sim"
        else "/tmp/lingtu_slam_status.json"
    )
    status_path = Path(os.environ.get("LINGTU_SLAM_STATUS_JSON") or default_status)
    cloud_dir = Path(os.environ.get("LINGTU_SLAM_CLOUD_SNAPSHOT_DIR") or "/dev/shm/lingtu_slam")
    status = read_json_snapshot(str(status_path)) or {}
    metadata = read_json_snapshot(str(cloud_dir / "global_map_cloud.meta.json"))
    if not isinstance(metadata, dict):
        metadata = {}
    live = status.get("global_mapping")
    if isinstance(live, dict):
        metadata = {
            **metadata,
            "busy": bool(live.get("busy", False)),
            "dropped_frames": live.get("dropped_frames", 0),
            "state": live.get("state", metadata.get("state")),
        }
    empty = dict(
        count=0,
        points=[],
        source="global_mapping_preview",
        layout="xyz_rows",
        frame_id=metadata.get("frame_id") or "map",
        epoch=_wire_epoch(status.get("source_epoch")),
        sequence=metadata.get("revision", 0),
        stamp_s=metadata.get("stamp_s", 0),
        stream_kind="map",
        global_mapping=metadata,
    )
    try:
        if (
            time.time() - status_path.stat().st_mtime > 5.0
            or (status.get("global_mapping") or {}).get("state") == "inactive"
            or not metadata.get("points")
            or metadata.get("source_epoch") != status.get("source_epoch")
        ):
            return empty
        cloud = PointCloud2.decode((cloud_dir / "global_map_cloud.bin").read_bytes())
        if (
            cloud.frame_id != metadata.get("frame_id")
            or abs(float(cloud.ts) - float(metadata.get("stamp_s", 0))) > 1e-5
        ):
            return empty
        points = cloud.points[:, :3]
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            return empty
        bounds = {"min": points.min(axis=0).tolist(), "max": points.max(axis=0).tolist()}
        limit = max(1, min(int(max_points), 200000))
        if len(points) > limit:
            points = points[np.linspace(0, len(points) - 1, limit, dtype=np.int64)]
        return {**empty, "count": len(points), "points": points.tolist(), "bounds": bounds}
    except (OSError, ValueError, TypeError, AttributeError):
        return empty
