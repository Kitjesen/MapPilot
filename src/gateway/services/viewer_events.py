"""SSE viewer-event serialization helpers."""

from __future__ import annotations

import base64
import logging
import math
from typing import Any

from gateway.services.sse import should_emit_raster
from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)

def xyz_point(point: Any) -> tuple[float, float, float] | None:
    if hasattr(point, "x") and hasattr(point, "y"):
        return (
            float(getattr(point, "x", 0.0)),
            float(getattr(point, "y", 0.0)),
            float(getattr(point, "z", 0.0)),
        )
    try:
        values = list(point)
    except TypeError:
        return None
    if len(values) < 2:
        return None
    return (
        float(values[0]),
        float(values[1]),
        float(values[2]) if len(values) > 2 else 0.0,
    )


def _path_identity(path: Any) -> dict[str, Any]:
    """Return source identity without inventing a map frame."""
    frame_id = getattr(path, "frame_id", None)
    if not frame_id:
        header = getattr(path, "header", None)
        frame_id = getattr(header, "frame_id", None)
    frame_id = str(frame_id).strip() if frame_id else None
    stamp_s = getattr(path, "ts", None)
    if stamp_s is None:
        header = getattr(path, "header", None)
        stamp = getattr(header, "stamp", None)
        to_sec = getattr(stamp, "to_sec", None)
        if callable(to_sec):
            stamp_s = to_sec()
    try:
        stamp_s = float(stamp_s) if stamp_s is not None else None
    except (TypeError, ValueError):
        stamp_s = None
    receive_sequence = getattr(path, "receive_sequence", None)
    if receive_sequence is None:
        receive_sequence = getattr(path, "sequence", None)
    try:
        receive_sequence = int(receive_sequence) if receive_sequence is not None else None
    except (TypeError, ValueError):
        receive_sequence = None
    return {
        "frame_id": frame_id,
        "stamp_s": stamp_s,
        "receive_sequence": receive_sequence,
    }


def _path_points(path: Any) -> list[dict[str, float | str | None]]:
    poses = getattr(path, "poses", path) or []
    identity = _path_identity(path)
    points: list[dict[str, float | str | None]] = []
    for point in poses:
        xyz = xyz_point(point)
        if xyz is None:
            continue
        item: dict[str, float | str | None] = {
            "x": xyz[0],
            "y": xyz[1],
            "z": xyz[2],
        }
        if identity["frame_id"] is not None:
            item["frame_id"] = identity["frame_id"]
        if identity["stamp_s"] is not None:
            item["ts"] = identity["stamp_s"]
        points.append(item)
    return points


def handle_global_path(gw: Any, path: Any) -> None:
    points = _path_points(path)
    identity = _path_identity(path)
    with gw._state_lock:
        gw._last_path = points
        gw._last_path_meta = identity
    gw.push_event({"type": "global_path", "points": points, **identity})


def handle_local_path(gw: Any, path: Any) -> None:
    gw._local_path_throttle = getattr(gw, "_local_path_throttle", 0) + 1
    try:
        points = _path_points(path)
        # Empty paths are control-plane clears and must not be sampled away.
        if points and gw._local_path_throttle % 5 != 0:
            return
        identity = _path_identity(path)
        with gw._state_lock:
            gw._last_local_path = points
            gw._last_local_path_meta = identity
        gw.push_event({"type": "local_path", "points": points, **identity})
    except Exception as exc:
        logger.debug("_on_local_path: failed to build points: %s", exc)


def handle_native_traversability(gw: Any, data: dict[str, Any]) -> None:
    """Publish native control-risk cells through the HostBus boundary."""

    grid = data.get("grid")
    if grid is None or not should_emit_raster(gw, "native_traversability"):
        return
    try:
        # Do not cast arbitrary input directly to uint8: values such as 256 or
        # 300 would wrap before the 0..100 contract check and could be shown as
        # a valid low-risk cell.  Validate in a wide numeric type first, then
        # encode the bounded control-risk values.
        raw = np.asarray(grid, dtype=np.float64)
        if (
            not np.all(np.isfinite(raw))
            or np.any(raw < 0)
            or np.any(raw > 100)
            or np.any(raw != np.rint(raw))
        ):
            raise ValueError("native traversability cells outside integer 0..100")
        arr = np.ascontiguousarray(raw.astype(np.uint8))
        if arr.ndim != 2 or arr.size <= 0 or arr.size > 1_000_000:
            raise ValueError("native traversability grid dimensions invalid")
        frame_id = str(data.get("frame_id") or "").strip()
        stamp_s = float(data.get("stamp_s", 0.0))
        reset_epoch = int(data.get("reset_epoch", 0))
        sequence = int(data.get("sequence", 0))
        resolution = float(data.get("resolution", 0.0))
        origin_raw = data.get("origin")
        yaw = float(data.get("yaw", 0.0))
        if (
            frame_id != "map"
            or not math.isfinite(stamp_s)
            or stamp_s <= 0.0
            or reset_epoch <= 0
            or sequence <= 0
            or not math.isfinite(resolution)
            or resolution <= 0.0
            or not isinstance(origin_raw, (list, tuple))
            or len(origin_raw) < 3
            or not all(math.isfinite(float(value)) for value in origin_raw[:3])
            or not math.isfinite(yaw)
            or abs(yaw) > 1e-6
            or np.any(arr > 100)
        ):
            raise ValueError("native traversability identity or transform invalid")
        gw.push_event(
            {
                "type": "native_traversability",
                "grid_b64": base64.b64encode(arr.tobytes(order="C")).decode(),
                "rows": int(arr.shape[0]),
                "cols": int(arr.shape[1]),
                "resolution": resolution,
                "origin": [float(value) for value in origin_raw[:3]],
                "yaw": yaw,
                "frame_id": frame_id,
                "stamp_s": stamp_s,
                "reset_epoch": reset_epoch,
                "sequence": sequence,
                "source": "native_nav_client",
                "control_authority": True,
                "value_semantics": "control_risk_0_100",
                "identity_verified": True,
            }
        )
    except Exception as exc:
        logger.debug("_on_native_traversability serialize failed: %s", exc)
