"""Traffic policy helpers for Gateway realtime channels."""

from __future__ import annotations

import asyncio
import json
import math
import time
from collections.abc import Mapping
from typing import Any

DEFAULT_SSE_QUEUE_MAXSIZE = 128
DEFAULT_CLOUD_QUEUE_MAXSIZE = 2
SSE_EVENT_SCHEMA_VERSION = 1
SSE_RETRY_MS = 3000
DEFAULT_SSE_RASTER_MIN_INTERVAL_S = 1.0
SSE_EVENT_TYPES = (
    "snapshot",
    "ping",
    "odometry",
    "joint_state",
    "slam_status",
    "map_cloud",
    "map_scene",
    "scene_graph",
    "visual_servo_status",
    "safety",
    "navigation_status",
    "inspection_task_event",
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
    "global_path",
    "local_path",
    "native_traversability",
    "agent_message",
)
SSE_DIAGNOSTIC_EVENT_TYPES = (
    "gnss_fusion",
    "slam_diag",
    "slam_drift",
    "tare_stats",
    "exploration_supervisor",
)
DROP_OLDEST_POLICY = "drop_oldest"
SSE_LATEST_STATE_TYPES = frozenset({
    "odometry", "joint_state", "slam_status", "map_cloud", "map_scene",
    "scene_graph", "visual_servo_status", "safety", "navigation_status",
    "lease", "gnss_fusion", "slam_diag", "slam_drift", "exploring",
    "tare_stats", "exploration_supervisor", "global_path", "local_path",
    "native_traversability",
})

RECOMMENDED_CLIENT_RATES_HZ: dict[str, float] = {
    "bootstrap": 0.0,
    "state": 1.0,
    "session": 1.0,
    "health": 0.2,
    "path": 2.0,
    "scene_graph": 1.0,
}


def put_latest(queue: asyncio.Queue, item: Any) -> bool:
    """Keep current state and give one-shot events priority under pressure."""
    if not isinstance(item, Mapping):
        if queue.full():
            queue.get_nowait()
            queue.put_nowait(item)
            return True
        queue.put_nowait(item)
        return False

    if item.get("type") not in SSE_LATEST_STATE_TYPES and not queue.full():
        queue.put_nowait(item)
        return False

    items: list[Any] = []
    while not queue.empty():
        items.append(queue.get_nowait())
    dropped = put_bounded(items, item, queue.maxsize)
    for queued in items:
        queue.put_nowait(queued)
    return dropped


def put_bounded(items: list[Any], item: Any, capacity: int) -> bool:
    """Apply the SSE policy before or after an event-loop handoff."""
    event_type = item.get("type") if isinstance(item, Mapping) else None
    is_state = event_type in SSE_LATEST_STATE_TYPES
    if is_state:
        for index, queued in enumerate(items):
            if isinstance(queued, Mapping) and queued.get("type") == event_type:
                del items[index]
                items.append(item)
                return True
    if len(items) >= capacity:
        state_index = next(
            (index for index, queued in enumerate(items)
             if isinstance(queued, Mapping) and queued.get("type") in SSE_LATEST_STATE_TYPES),
            None,
        )
        if is_state and state_index is None:
            return True
        del items[state_index if state_index is not None else 0]
        items.append(item)
        return True
    items.append(item)
    return False


def prepare_sse_delivery(event: Mapping[str, Any], *, now: float | None = None) -> dict[str, Any] | None:
    """Refresh display sample age at dequeue, without rebasing its source clock."""
    if event.get("type") != "joint_state":
        return dict(event)
    stamp = event.get("stamp")
    if not isinstance(stamp, (float, int)) or not math.isfinite(stamp) or stamp <= 0:
        return None
    age_s = (time.time() if now is None else now) - stamp
    if age_s > 2.0 or age_s < -1.0:
        return None
    return {**event, "source_age_s": max(0.0, age_s)}


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
    from gateway.services.sse import handoff_stats

    with gw._sse_lock:
        queue_depths = [queue.qsize() for queue in gw._sse_queues]
        handoff_depths = [handoff_stats(queue)[0] for queue in gw._sse_queues]
        sse = {
            "clients": len(gw._sse_queues),
            "queue_maxsize": gw._sse_queue_maxsize,
            "queue_depths": queue_depths,
            "handoff_depths": handoff_depths,
            "max_depth_seen": gw._sse_max_depth_seen,
            "latest_event_id": gw._sse_event_seq,
            "published_events": gw._sse_published_events,
            "dropped_events": gw._sse_dropped_events,
            "suppressed_events": dict(gw._sse_suppressed_events),
            "raster_min_interval_s": gw._sse_raster_min_interval_s,
            "drop_policy": DROP_OLDEST_POLICY,
        }
    realtime = gw._cloud_viewer.traffic_snapshot()
    return {
        "sse": sse,
        "cloud": realtime["cloud"],
        "scan": realtime["scan"],
        "recommended_client_rates_hz": dict(RECOMMENDED_CLIENT_RATES_HZ),
    }
