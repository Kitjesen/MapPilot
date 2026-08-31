"""Server-sent event queue helpers for GatewayModule."""

from __future__ import annotations

import asyncio
import copy
import time
from typing import Any, Callable

from gateway.services.traffic import normalize_sse_event, put_latest


def _elevation_layer(event: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(event, dict) or event.get("type") != "map_scene":
        return None
    layers = event.get("layers")
    if not isinstance(layers, list):
        return None
    for layer in layers:
        if not isinstance(layer, dict):
            continue
        if layer.get("id") == "maps.elevation" or layer.get("topic") == "/maps/elevation":
            return layer
    return None


def _elevation_cohort(event: dict[str, Any] | None) -> tuple[Any, ...] | None:
    layer = _elevation_layer(event)
    if layer is None:
        return None
    metadata = event.get("metadata") if isinstance(event, dict) else None
    metadata = metadata if isinstance(metadata, dict) else {}
    producer_boot_id = str(layer.get("producer_boot_id") or metadata.get("producer_boot_id") or "").strip()
    frame_id = str(layer.get("frame_id") or event.get("frame_id") or "").strip()
    reset_epoch = layer.get("reset_epoch", metadata.get("reset_epoch"))
    origin = layer.get("origin")
    if not producer_boot_id or not frame_id or reset_epoch is None or not isinstance(origin, list):
        return None
    return (
        producer_boot_id,
        frame_id,
        reset_epoch,
        event.get("map_id"),
        layer.get("rows"),
        layer.get("cols"),
        layer.get("resolution"),
        tuple(origin),
        layer.get("yaw"),
        layer.get("downsample_factor"),
    )


def _update_latest_elevation_locked(gw: Any, event: dict[str, Any]) -> None:
    layer = _elevation_layer(event)
    if layer is None:
        return
    grid_b64 = layer.get("grid_b64")
    if isinstance(grid_b64, str) and grid_b64:
        rows = layer.get("rows")
        cols = layer.get("cols")
        cell_count = rows * cols if isinstance(rows, int) and isinstance(cols, int) else 0
        expected_b64_length = 4 * ((cell_count * 4 + 2) // 3)
        if (
            _elevation_cohort(event) is None
            or cell_count <= 0
            or cell_count > 131_072
            or len(grid_b64) != expected_b64_length
        ):
            gw._latest_elevation_event = None
            return
        cached = {key: copy.deepcopy(value) for key, value in event.items() if key != "layers"}
        cached["layers"] = [copy.deepcopy(layer)]
        gw._latest_elevation_event = cached
        return
    if layer.get("retain_previous") is True:
        cached = getattr(gw, "_latest_elevation_event", None)
        if _elevation_cohort(cached) == _elevation_cohort(event) and _elevation_cohort(event) is not None:
            return
    gw._latest_elevation_event = None


def _latest_elevation_replay_locked(gw: Any) -> dict[str, Any] | None:
    cached = getattr(gw, "_latest_elevation_event", None)
    if not isinstance(cached, dict):
        return None
    replay = copy.deepcopy(cached)
    replay.pop("event_id", None)
    return normalize_sse_event(replay)


def _subscriber_accepts_locked(
    gw: Any,
    q: asyncio.Queue,
    event_type: str,
    *,
    elevation_payload: bool = False,
) -> bool:
    """Return whether one subscriber wants an event/payload.

    ``None`` means the legacy all-events stream.  The HTTP route always stores
    an explicit capability for elevation so a normal dashboard connection can
    receive map metadata without receiving the large raster body.
    """
    event_types = getattr(gw, "_sse_queue_event_types", {}).get(q)
    if event_types is not None and event_type not in event_types:
        return False
    if elevation_payload and not getattr(gw, "_sse_queue_elevation_payload", {}).get(q, False):
        return False
    return True


def _event_for_subscriber(
    payload: dict[str, Any],
    *,
    include_elevation_payload: bool,
) -> dict[str, Any]:
    """Strip only heavy elevation bytes for clients that did not opt in."""
    if include_elevation_payload or payload.get("type") != "map_scene":
        return payload
    layers = payload.get("layers")
    if not isinstance(layers, list):
        return payload
    changed = False
    filtered_layers: list[Any] = []
    for layer in layers:
        if not isinstance(layer, dict):
            filtered_layers.append(layer)
            continue
        is_elevation = layer.get("id") == "maps.elevation" or layer.get("topic") == "/maps/elevation"
        if not is_elevation or not layer.get("grid_b64"):
            filtered_layers.append(layer)
            continue
        filtered = dict(layer)
        filtered.pop("grid_b64", None)
        filtered["payload"] = "omitted"
        filtered["reason"] = "client_not_subscribed"
        filtered["retain_previous"] = False
        filtered_layers.append(filtered)
        changed = True
    if not changed:
        return payload
    result = dict(payload)
    result["layers"] = filtered_layers
    return result


def running_loop_or_none() -> asyncio.AbstractEventLoop | None:
    try:
        return asyncio.get_running_loop()
    except RuntimeError:
        return None


def call_queue_put_latest(
    q: asyncio.Queue,
    item: Any,
    loop: asyncio.AbstractEventLoop | None,
    record: Callable[[bool, int], None],
) -> None:
    def _put_and_record() -> None:
        dropped = put_latest(q, item)
        record(dropped, q.qsize())

    if loop is not None and loop.is_running():
        try:
            loop.call_soon_threadsafe(_put_and_record)
            return
        except RuntimeError:
            pass
    _put_and_record()


def record_delivery(gw: Any, dropped: bool, depth: int) -> None:
    with gw._sse_lock:
        if dropped:
            gw._sse_dropped_events += 1
        gw._sse_max_depth_seen = max(gw._sse_max_depth_seen, depth)


def should_emit_raster(gw: Any, event_type: str) -> bool:
    with gw._sse_lock:
        if not any(_subscriber_accepts_locked(gw, q, event_type) for q in gw._sse_queues):
            return False
        interval = max(0.0, float(gw._sse_raster_min_interval_s))
        now = time.monotonic()
        last = gw._sse_raster_last_emit.get(event_type)
        if interval > 0 and last is not None and now - last < interval:
            gw._sse_suppressed_events[event_type] = gw._sse_suppressed_events.get(event_type, 0) + 1
            return False
        gw._sse_raster_last_emit[event_type] = now
        return True


def push_event(gw: Any, event: dict) -> None:
    with gw._sse_lock:
        _update_latest_elevation_locked(gw, event)
        gw._sse_event_seq += 1
        event_id = gw._sse_event_seq
        subscribers = [
            (
                q,
                gw._sse_queue_loops.get(q),
                getattr(gw, "_sse_queue_elevation_payload", {}).get(q, False),
            )
            for q in gw._sse_queues
            if _subscriber_accepts_locked(gw, q, str(event.get("type") or "event"))
        ]
        gw._sse_published_events += 1
    payload = normalize_sse_event(event, event_id=event_id)
    for q, loop, include_elevation_payload in subscribers:
        subscriber_payload = _event_for_subscriber(
            payload,
            include_elevation_payload=include_elevation_payload,
        )
        call_queue_put_latest(
            q,
            subscriber_payload,
            loop,
            lambda dropped, depth: record_delivery(gw, dropped, depth),
        )


def subscribe_with_event_id(
    gw: Any,
    *,
    event_types: set[str] | None = None,
    include_elevation_payload: bool = False,
) -> tuple[asyncio.Queue, int]:
    q: asyncio.Queue = asyncio.Queue(maxsize=gw._sse_queue_maxsize)
    loop = running_loop_or_none()
    with gw._sse_lock:
        gw._sse_event_seq += 1
        event_id = gw._sse_event_seq
        gw._sse_queues.append(q)
        gw._sse_queue_loops[q] = loop
        gw._sse_queue_event_types[q] = set(event_types) if event_types is not None else None
        gw._sse_queue_elevation_payload[q] = bool(include_elevation_payload)
        replay = _latest_elevation_replay_locked(gw)
        if replay is not None and not _subscriber_accepts_locked(gw, q, "map_scene"):
            replay = None
        if replay is not None:
            replay = _event_for_subscriber(
                replay,
                include_elevation_payload=include_elevation_payload,
            )
    if replay is not None:
        call_queue_put_latest(
            q,
            replay,
            loop,
            lambda dropped, depth: record_delivery(gw, dropped, depth),
        )
    return q, event_id


def subscribe(
    gw: Any,
    *,
    event_types: set[str] | None = None,
    include_elevation_payload: bool = False,
) -> asyncio.Queue:
    q: asyncio.Queue = asyncio.Queue(maxsize=gw._sse_queue_maxsize)
    loop = running_loop_or_none()
    with gw._sse_lock:
        gw._sse_queues.append(q)
        gw._sse_queue_loops[q] = loop
        gw._sse_queue_event_types[q] = set(event_types) if event_types is not None else None
        gw._sse_queue_elevation_payload[q] = bool(include_elevation_payload)
        replay = _latest_elevation_replay_locked(gw)
        if replay is not None and not _subscriber_accepts_locked(gw, q, "map_scene"):
            replay = None
        if replay is not None:
            replay = _event_for_subscriber(
                replay,
                include_elevation_payload=include_elevation_payload,
            )
    if replay is not None:
        call_queue_put_latest(
            q,
            replay,
            loop,
            lambda dropped, depth: record_delivery(gw, dropped, depth),
        )
    return q


def unsubscribe(gw: Any, q: asyncio.Queue) -> None:
    with gw._sse_lock:
        try:
            gw._sse_queues.remove(q)
        except ValueError:
            pass
        gw._sse_queue_loops.pop(q, None)
        getattr(gw, "_sse_queue_event_types", {}).pop(q, None)
        getattr(gw, "_sse_queue_elevation_payload", {}).pop(q, None)
