"""Server-sent event queue helpers for GatewayModule."""

from __future__ import annotations

import asyncio
import time
from typing import Any, Callable

from gateway.services.traffic import normalize_sse_event, put_latest


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
        if not gw._sse_queues:
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
        gw._sse_event_seq += 1
        event_id = gw._sse_event_seq
        subscribers = [(q, gw._sse_queue_loops.get(q)) for q in gw._sse_queues]
        gw._sse_published_events += 1
    payload = normalize_sse_event(event, event_id=event_id)
    for q, loop in subscribers:
        call_queue_put_latest(
            q,
            payload,
            loop,
            lambda dropped, depth: record_delivery(gw, dropped, depth),
        )


def next_event_id(gw: Any) -> int:
    with gw._sse_lock:
        gw._sse_event_seq += 1
        return gw._sse_event_seq


def subscribe_with_event_id(gw: Any) -> tuple[asyncio.Queue, int]:
    q: asyncio.Queue = asyncio.Queue(maxsize=gw._sse_queue_maxsize)
    loop = running_loop_or_none()
    with gw._sse_lock:
        gw._sse_event_seq += 1
        event_id = gw._sse_event_seq
        gw._sse_queues.append(q)
        gw._sse_queue_loops[q] = loop
    return q, event_id


def subscribe(gw: Any) -> asyncio.Queue:
    q: asyncio.Queue = asyncio.Queue(maxsize=gw._sse_queue_maxsize)
    loop = running_loop_or_none()
    with gw._sse_lock:
        gw._sse_queues.append(q)
        gw._sse_queue_loops[q] = loop
    return q


def unsubscribe(gw: Any, q: asyncio.Queue) -> None:
    with gw._sse_lock:
        try:
            gw._sse_queues.remove(q)
        except ValueError:
            pass
        gw._sse_queue_loops.pop(q, None)
