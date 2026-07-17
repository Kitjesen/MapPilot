"""Traffic policy helpers for Gateway realtime channels."""

from __future__ import annotations

import asyncio
import json
import time
from collections.abc import Mapping
from typing import Any

DEFAULT_SSE_QUEUE_MAXSIZE = 128
DEFAULT_CLOUD_QUEUE_MAXSIZE = 2
SSE_EVENT_SCHEMA_VERSION = 1
SSE_RETRY_MS = 3000
DEFAULT_SSE_RASTER_MIN_INTERVAL_S = 1.0
DEFAULT_SSE_SLOPE_PAYLOAD_ENABLED = False
SSE_EVENT_TYPES = (
    "snapshot",
    "ping",
    "odometry",
    "slam_status",
    "map_cloud",
    "saved_map",
    "scene_graph",
    "safety",
    "mission",
    "navigation_status",
    "lease",
    "command_ack",
    "eval",
    "dialogue",
    "gnss_fusion",
    "slam_diag",
    "slam_drift",
    "session",
    "exploring",
    "tare_stats",
    "exploration_supervisor",
    "traversable_frontiers",
    "frontier_candidate",
    "global_path",
    "local_path",
    "costmap",
    "slope_grid",
    "agent_message",
)
SSE_DIAGNOSTIC_EVENT_TYPES = (
    "gnss_fusion",
    "slam_diag",
    "slam_drift",
    "tare_stats",
    "exploration_supervisor",
)
SSE_LEGACY_EVENT_TYPES = ("heartbeat",)

DROP_OLDEST_POLICY = "drop_oldest"

RECOMMENDED_CLIENT_RATES_HZ: dict[str, float] = {
    "bootstrap": 0.0,
    "state": 1.0,
    "session": 1.0,
    "health": 0.2,
    "path": 2.0,
    "scene_graph": 1.0,
    "devices": 0.2,
}


def put_latest(queue: asyncio.Queue, item: Any) -> bool:
    """Put item without blocking, dropping one old item if the queue is full."""
    try:
        queue.put_nowait(item)
        return False
    except asyncio.QueueFull:
        pass

    try:
        queue.get_nowait()
    except asyncio.QueueEmpty:
        pass

    try:
        queue.put_nowait(item)
    except asyncio.QueueFull:
        return True
    return True


def normalize_sse_event(
    event: Any,
    *,
    event_id: int | None = None,
    now: float | None = None,
) -> dict[str, Any]:
    """Return the stable SSE event envelope used by App/Web clients."""
    payload = dict(event) if isinstance(event, Mapping) else {"data": event}
    if not isinstance(payload.get("type"), str) or not payload.get("type"):
        payload["type"] = "event"
    payload.setdefault("schema_version", SSE_EVENT_SCHEMA_VERSION)
    payload.setdefault("ts", time.time() if now is None else now)
    if event_id is not None:
        payload["event_id"] = int(event_id)
    return payload


def format_sse_message(
    event: Mapping[str, Any],
    *,
    event_id: int | None = None,
    retry_ms: int | None = None,
) -> str:
    """Format a Server-Sent Event without a named event type.

    Browser EventSource delivers unnamed events through ``onmessage``. Keeping
    the event type inside JSON preserves the existing web hook behavior while
    still giving clients ``Last-Event-ID`` support.
    """
    parts: list[str] = []
    if retry_ms is not None:
        parts.append(f"retry: {int(retry_ms)}")
    resolved_id = event_id
    if resolved_id is None and event.get("event_id") is not None:
        try:
            resolved_id = int(event["event_id"])
        except (TypeError, ValueError):
            resolved_id = None
    if resolved_id is not None:
        parts.append(f"id: {resolved_id}")
    parts.append(f"data: {json.dumps(dict(event), separators=(',', ':'))}")
    return "\n".join(parts) + "\n\n"


def snapshot(gw: Any) -> dict[str, Any]:
    """Return one consistent traffic snapshot across SSE and binary streams."""
    with gw._sse_lock:
        queue_depths = [queue.qsize() for queue in gw._sse_queues]
        sse = {
            "clients": len(gw._sse_queues),
            "queue_maxsize": gw._sse_queue_maxsize,
            "queue_depths": queue_depths,
            "max_depth_seen": gw._sse_max_depth_seen,
            "latest_event_id": gw._sse_event_seq,
            "published_events": gw._sse_published_events,
            "dropped_events": gw._sse_dropped_events,
            "suppressed_events": dict(gw._sse_suppressed_events),
            "raster_min_interval_s": gw._sse_raster_min_interval_s,
            "slope_grid_inline": gw._sse_slope_payload_enabled,
            "drop_policy": DROP_OLDEST_POLICY,
        }
    realtime = gw._cloud_viewer.traffic_snapshot()
    return {
        "sse": sse,
        "cloud": realtime["cloud"],
        "scan": realtime["scan"],
        "recommended_client_rates_hz": dict(RECOMMENDED_CLIENT_RATES_HZ),
    }
