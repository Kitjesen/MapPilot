"""Pose recovery and localization reset helpers for GatewayModule."""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from typing import Any

from runtime.msgs.geometry import Quaternion, Transform, Vector3
from runtime.runtime_interface import TOPICS, topic_default_frame_id

logger = logging.getLogger(__name__)

LAST_POSE_PATH = os.path.expanduser("~/.lingtu/last_nav_pose.json")
LAST_POSE_MAX_AGE_S = 7 * 24 * 3600


def persist_last_nav_pose(
    map_name: str,
    x: float,
    y: float,
    yaw: float,
    quality: float | None,
    *,
    path: str = LAST_POSE_PATH,
) -> None:
    """Atomically snapshot the last successful relocalize pose to disk."""
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        payload = {
            "map_name": map_name,
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
            "quality": float(quality) if quality is not None else None,
            "saved_at": time.time(),
        }
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(payload, f)
        os.replace(tmp, path)
        logger.info(
            "gateway: persisted last_nav_pose map=%s (%.2f, %.2f, yaw=%.2f)",
            map_name,
            x,
            y,
            yaw,
        )
    except Exception as e:
        logger.warning("gateway: persist last_nav_pose failed: %s", e)


def load_last_nav_pose(
    map_name: str,
    *,
    path: str = LAST_POSE_PATH,
    max_age_s: float = LAST_POSE_MAX_AGE_S,
) -> dict | None:
    """Return persisted pose if map matches and snapshot is fresh."""
    try:
        if not os.path.isfile(path):
            return None
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        if data.get("map_name") != map_name:
            return None
        age = time.time() - float(data.get("saved_at", 0.0))
        if age > max_age_s:
            logger.info("gateway: last_nav_pose too old (%.1fh), ignoring", age / 3600)
            return None
        return data
    except Exception as e:
        logger.debug("gateway: load last_nav_pose failed: %s", e)
        return None


def spawn_auto_relocalize(gw: Any, map_name: str) -> None:
    """Replay persisted saved-map relocalization in a background thread."""
    data = gw._load_last_nav_pose(map_name)
    if data is None:
        return
    x, y, yaw = data["x"], data["y"], data["yaw"]

    def _worker() -> None:
        time.sleep(2.5)
        try:
            pcd_path = gw._map_artifact_path_from_maps_service(
                map_name,
                "source_pointcloud",
            )
            if pcd_path is None or not os.path.isfile(pcd_path):
                logger.warning(
                    "auto-relocalize: source point cloud unavailable from maps service: %s",
                    map_name,
                )
                return
            if not gw.localization.available:
                logger.warning("auto-relocalize: relocalization service unavailable")
                return
            result = gw.localization.relocalize_saved_map_with_env(
                pcd_path,
                x,
                y,
                yaw,
                timeout_s=20.0,
            )
            logger.info(
                "auto-relocalize: map=%s pose=(%.2f,%.2f,yaw=%.2f) ok=%s",
                map_name,
                x,
                y,
                yaw,
                result.success,
            )
        except Exception as e:
            logger.warning("auto-relocalize worker failed: %s", e)

    threading.Thread(
        target=_worker,
        daemon=True,
        name="gateway-auto-reloc",
    ).start()


def handle_map_odom_tf(gw: Any, tf: dict) -> None:
    """Cache localizer map->odom transform in FrameTree and matrix form."""
    if not tf or not tf.get("valid", False):
        return
    gw._blackbox.record("tf", tf)
    try:
        tx, ty, tz = float(tf["tx"]), float(tf["ty"]), float(tf["tz"])
        qx, qy, qz, qw = (
            float(tf["qx"]),
            float(tf["qy"]),
            float(tf["qz"]),
            float(tf["qw"]),
        )
        ts = float(tf.get("ts") or time.time())
        transform = Transform(
            translation=Vector3(tx, ty, tz),
            rotation=Quaternion(qx, qy, qz, qw),
            frame_id=topic_default_frame_id(TOPICS.map_cloud),
            child_frame_id=topic_default_frame_id(TOPICS.odometry),
            ts=ts,
        )
        gw._frame_tree.set_transform(transform)
        gw._T_map_odom = transform.to_matrix()
        gw._has_map_odom_tf = True
    except Exception as e:
        logger.debug("gateway: _on_map_odom_tf parse failed: %s", e)


def clear_localization_runtime_cache(
    gw: Any,
    *,
    reason: str = "localization_restart",
) -> None:
    """Clear stale pose, map-cloud, and odometry UI state after localization reset."""
    with gw._state_lock:
        gw._runtime_cache.clear()
    gw._cloud_viewer.clear(reason=reason)
    gw.push_event(
        {
            "type": "odometry",
            "data": None,
            "reset": True,
            "reason": reason,
        }
    )
