"""SSE viewer-event serialization helpers."""

from __future__ import annotations

import base64
import logging
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


def handle_global_path(gw: Any, path: Any) -> None:
    points = []
    for point in getattr(path, "poses", path) or []:
        xyz = xyz_point(point)
        if xyz is not None:
            points.append({"x": xyz[0], "y": xyz[1], "z": xyz[2]})
    with gw._state_lock:
        gw._last_path = points
    gw.push_event({"type": "global_path", "points": points})


def handle_local_path(gw: Any, path: Any) -> None:
    gw._local_path_throttle = getattr(gw, "_local_path_throttle", 0) + 1
    if gw._local_path_throttle % 5 != 0:
        return
    try:
        points = (
            [
                {
                    "x": float(p.pose.position.x),
                    "y": float(p.pose.position.y),
                    "z": float(getattr(p.pose.position, "z", 0.0)),
                }
                for p in path.poses
            ]
            if hasattr(path, "poses")
            else []
        )
        with gw._state_lock:
            gw._last_local_path = points
        gw.push_event({"type": "local_path", "points": points})
    except Exception as exc:
        logger.debug("_on_local_path: failed to build points: %s", exc)


def handle_costmap(gw: Any, costmap: dict[str, Any]) -> None:
    gw._costmap_throttle += 1
    if gw._costmap_throttle % 5 != 0:
        return
    grid = costmap.get("grid")
    if grid is None:
        return
    if not should_emit_raster(gw, "costmap"):
        return
    try:
        grid_u8 = np.clip(grid, 0, 100).astype(np.uint8)
        rows = int(grid_u8.shape[0])
        cols = int(grid_u8.shape[1]) if grid_u8.ndim >= 2 else rows
        origin = [float(v) for v in costmap.get("origin", [0.0, 0.0])]
        gw.push_event(
            {
                "type": "costmap",
                "grid_b64": base64.b64encode(grid_u8.tobytes()).decode(),
                "rows": rows,
                "cols": cols,
                "resolution": float(costmap.get("resolution", 0.1)),
                "origin": origin,
                "yaw": 0.0,
            }
        )
    except Exception as exc:
        logger.debug("_on_costmap serialize failed: %s", exc)


def handle_slope_grid(gw: Any, data: dict[str, Any]) -> None:
    grid = data.get("grid")
    if grid is None:
        return
    if not should_emit_raster(gw, "slope_grid"):
        return
    try:
        arr = np.asarray(grid)
        rows = int(arr.shape[0])
        cols = int(arr.shape[1]) if arr.ndim >= 2 else rows
        origin = [float(v) for v in data.get("origin", [0.0, 0.0])]
        event: dict[str, Any] = {
            "type": "slope_grid",
            "available": True,
            "rows": rows,
            "cols": cols,
            "resolution": float(data.get("resolution", 0.2)),
            "origin": origin,
            "yaw": 0.0,
        }
        if gw._sse_slope_payload_enabled:
            grid_u8 = np.clip(arr * (255.0 / 90.0), 0, 255).astype(np.uint8)
            event.update(
                {
                    "payload": "inline",
                    "encoding": "uint8_slope_degrees_0_90",
                    "grid_b64": base64.b64encode(grid_u8.tobytes()).decode(),
                }
            )
        else:
            event.update(
                {
                    "payload": "omitted",
                    "reason": "inline_payload_disabled",
                    "encoding": "uint8_slope_degrees_0_90",
                }
            )
        gw.push_event(event)
    except Exception as exc:
        logger.debug("_on_slope_grid serialize failed: %s", exc)
