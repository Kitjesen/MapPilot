"""Binary point-cloud WebSocket channel."""

from __future__ import annotations

import asyncio
import threading
import time
from typing import Any, Callable

from gateway.services.traffic import DROP_OLDEST_POLICY


class CloudWs:
    """Stores the latest binary cloud frame and fans it out to WS clients."""

    def __init__(
        self,
        *,
        queue_put_latest: Callable[
            [asyncio.Queue, bytes, asyncio.AbstractEventLoop | None, Callable[[bool, int], None]],
            None,
        ],
        current_loop: Callable[[], asyncio.AbstractEventLoop | None],
        queue_maxsize: int,
    ) -> None:
        self._queue_put_latest = queue_put_latest
        self._current_loop = current_loop
        self._lock = threading.Lock()
        self._latest_buf: bytes | None = None
        self._seq = 0
        self._subs: list[asyncio.Queue] = []
        self._sub_loops: dict[asyncio.Queue, asyncio.AbstractEventLoop | None] = {}
        self._queue_maxsize = queue_maxsize
        self._published_frames = 0
        self._dropped_frames = 0
        self._max_depth_seen = 0
        self._latest_meta: dict[str, Any] = {}

    def publish(self, buf: bytes, *, metadata: dict[str, Any] | None = None) -> int:
        now = time.time()
        with self._lock:
            self._latest_buf = buf
            self._seq += 1
            seq = self._seq
            meta = dict(metadata or {})
            meta.update({"seq": seq, "bytes": len(buf), "ts": now})
            self._latest_meta = meta
            subscribers = [(q, self._sub_loops.get(q)) for q in self._subs]
            self._published_frames += 1
        for q, loop in subscribers:
            self._queue_put_latest(q, buf, loop, self.record_delivery)
        return seq

    def subscribe(self) -> tuple[asyncio.Queue, bytes | None]:
        q: asyncio.Queue = asyncio.Queue(maxsize=self._queue_maxsize)
        loop = self._current_loop()
        with self._lock:
            self._subs.append(q)
            self._sub_loops[q] = loop
            latest = self._latest_buf
        return q, latest

    def unsubscribe(self, q: asyncio.Queue) -> None:
        with self._lock:
            try:
                self._subs.remove(q)
            except ValueError:
                pass
            self._sub_loops.pop(q, None)

    def has_subscribers(self) -> bool:
        with self._lock:
            return bool(self._subs)

    def record_delivery(self, dropped: bool, depth: int) -> None:
        with self._lock:
            if dropped:
                self._dropped_frames += 1
            self._max_depth_seen = max(self._max_depth_seen, depth)

    def traffic(self) -> dict[str, Any]:
        with self._lock:
            depths = [q.qsize() for q in self._subs]
            latest = dict(self._latest_meta)
            if latest.get("ts"):
                latest["age_s"] = round(max(0.0, time.time() - float(latest["ts"])), 3)
            return {
                "clients": len(self._subs),
                "queue_maxsize": self._queue_maxsize,
                "queue_depths": depths,
                "max_depth_seen": self._max_depth_seen,
                "published_frames": self._published_frames,
                "dropped_frames": self._dropped_frames,
                "drop_policy": DROP_OLDEST_POLICY,
                "latest_seq": self._seq,
                "latest_frame": latest,
            }

    def latest_debug(self) -> tuple[bool, dict[str, Any]]:
        with self._lock:
            latest = dict(self._latest_meta)
            has_latest = self._latest_buf is not None
        if latest.get("ts"):
            latest["age_s"] = round(max(0.0, time.time() - float(latest["ts"])), 3)
        return has_latest, latest
