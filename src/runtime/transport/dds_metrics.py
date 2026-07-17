"""Lightweight per-topic DDS observability metrics.

Collects message count, rate, latency, byte volume, and drop count for DDS
topics. All collection is opt-in via the ``LINGTU_DDS_METRICS`` environment
variable and adds zero overhead when disabled.

Design constraints:
  * Zero external dependencies (stdlib only).
  * Thread-safe via a single ``threading.Lock`` per collector.
  * Metrics collection paths are wrapped in ``try/except`` so a bug in
    observability code can never affect normal DDS communication.
  * When ``LINGTU_DDS_METRICS`` is not set to ``1``/``true``/``yes`` the
    helper functions return immediately, keeping the hot path clean.

Usage from transport code::

    from runtime.transport.dds_metrics import record_publish, record_receive

    # On publish
    record_publish("my_topic", size_bytes=128)

    # On receive (timestamp extracted from the message if possible)
    record_receive("my_topic", msg)

The global singleton is accessible via :func:`get_global_metrics` so Gateway
or other modules can expose the snapshot over REST/SSE.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from collections import deque
from typing import Any

logger = logging.getLogger(__name__)

# Environment variable that enables DDS metrics collection. Default is off to
# guarantee zero overhead in production.
_ENV_VAR = "LINGTU_DDS_METRICS"

# Default sliding-window size for rate calculation.
_DEFAULT_WINDOW_SIZE = 200

# ── environment variable helper ──────────────────────────────────────────

_env_cache: bool | None = None
_env_lock = threading.Lock()


def metrics_enabled() -> bool:
    """Return True when ``LINGTU_DDS_METRICS=1`` is set in the environment.

    The result is cached after the first read for performance. Tests that
    need to flip the switch should call :func:`reset_env_cache`.
    """
    global _env_cache
    if _env_cache is not None:
        return _env_cache
    with _env_lock:
        if _env_cache is not None:
            return _env_cache
        raw = os.environ.get(_ENV_VAR, "").strip().lower()
        _env_cache = raw in ("1", "true", "yes", "on")
    return _env_cache


def reset_env_cache() -> None:
    """Clear the cached environment-variable lookup (test helper)."""
    global _env_cache
    with _env_lock:
        _env_cache = None


# ── per-topic stats ──────────────────────────────────────────────────────


class _TopicStats:
    """Mutable per-topic statistics (not thread-safe on its own)."""

    __slots__ = (
        "_last_seq",
        "_timestamps",
        "bytes_total",
        "drop_count",
        "last_latency_ms",
        "msg_count",
        "publish_bytes",
        "publish_count",
        "receive_bytes",
        "receive_count",
    )

    def __init__(self, window_size: int) -> None:
        self.msg_count: int = 0
        self.publish_count: int = 0
        self.receive_count: int = 0
        self.bytes_total: int = 0
        self.publish_bytes: int = 0
        self.receive_bytes: int = 0
        self.drop_count: int = 0
        self.last_latency_ms: float = 0.0
        # Sliding window of receive timestamps (monotonic) for rate calc.
        self._timestamps: deque[float] = deque(maxlen=window_size)
        self._last_seq: int | None = None

    def rate_hz(self) -> float:
        """Compute message rate from the sliding window of receive times."""
        n = len(self._timestamps)
        if n < 2:
            return 0.0
        span = self._timestamps[-1] - self._timestamps[0]
        if span <= 0.0:
            return 0.0
        return (n - 1) / span


# ── metrics collector ────────────────────────────────────────────────────


class DDSMetrics:
    """Per-topic DDS metrics collector.

    Thread-safe. Use :meth:`record_publish` on the send path and
    :meth:`record_receive` on the receive path. Call :meth:`snapshot` to get a
    dict of all topic metrics, and :meth:`reset` to clear.
    """

    def __init__(self, window_size: int = _DEFAULT_WINDOW_SIZE) -> None:
        self._window_size = window_size
        self._lock = threading.Lock()
        self._topics: dict[str, _TopicStats] = {}

    def _get_or_create(self, topic: str) -> _TopicStats:
        stats = self._topics.get(topic)
        if stats is None:
            stats = _TopicStats(self._window_size)
            self._topics[topic] = stats
        return stats

    def record_publish(self, topic: str, size_bytes: int = 0) -> None:
        """Record one published message on *topic*."""
        try:
            with self._lock:
                stats = self._get_or_create(topic)
                stats.msg_count += 1
                stats.publish_count += 1
                stats.bytes_total += size_bytes
                stats.publish_bytes += size_bytes
        except Exception:
            logger.debug("DDSMetrics.record_publish error", exc_info=True)

    def record_receive(
        self,
        topic: str,
        *,
        timestamp: float | None = None,
        size_bytes: int = 0,
        seq: int | None = None,
    ) -> None:
        """Record one received message on *topic*.

        Parameters
        ----------
        topic:
            Topic name.
        timestamp:
            Publish-side wall-clock timestamp (``time.time()`` seconds). When
            provided, the end-to-end latency is computed as
            ``now - timestamp``.
        size_bytes:
            Best-effort payload size in bytes.
        seq:
            Optional sequence number. When provided and a gap is detected
            (seq > last_seq + 1), the difference is added to ``drop_count``.
        """
        try:
            now_mono = time.monotonic()
            latency_ms = 0.0
            if timestamp is not None:
                try:
                    now_wall = time.time()
                    latency_ms = max(0.0, (now_wall - float(timestamp)) * 1000.0)
                except (TypeError, ValueError):
                    latency_ms = 0.0

            with self._lock:
                stats = self._get_or_create(topic)
                stats.msg_count += 1
                stats.receive_count += 1
                stats.bytes_total += size_bytes
                stats.receive_bytes += size_bytes
                stats.last_latency_ms = latency_ms
                stats._timestamps.append(now_mono)

                if seq is not None:
                    last = stats._last_seq
                    if last is not None and seq > last + 1:
                        stats.drop_count += seq - last - 1
                    stats._last_seq = seq
        except Exception:
            logger.debug("DDSMetrics.record_receive error", exc_info=True)

    def snapshot(self) -> dict[str, dict[str, Any]]:
        """Return a deep-copy snapshot of all topic metrics.

        Each topic maps to a dict with keys:
        ``msg_count``, ``publish_count``, ``receive_count``,
        ``msg_rate_hz``, ``last_latency_ms``, ``bytes_total``,
        ``publish_bytes``, ``receive_bytes``, ``drop_count``.
        """
        with self._lock:
            result: dict[str, dict[str, Any]] = {}
            for topic, stats in self._topics.items():
                result[topic] = {
                    "msg_count": stats.msg_count,
                    "publish_count": stats.publish_count,
                    "receive_count": stats.receive_count,
                    "msg_rate_hz": round(stats.rate_hz(), 2),
                    "last_latency_ms": round(stats.last_latency_ms, 3),
                    "bytes_total": stats.bytes_total,
                    "publish_bytes": stats.publish_bytes,
                    "receive_bytes": stats.receive_bytes,
                    "drop_count": stats.drop_count,
                }
            return result

    def reset(self) -> None:
        """Clear all collected metrics."""
        with self._lock:
            self._topics.clear()


# ── global singleton ─────────────────────────────────────────────────────

_global_metrics: DDSMetrics | None = None
_global_lock = threading.Lock()


def get_global_metrics() -> DDSMetrics:
    """Return the process-wide :class:`DDSMetrics` singleton."""
    global _global_metrics
    if _global_metrics is None:
        with _global_lock:
            if _global_metrics is None:
                _global_metrics = DDSMetrics()
    return _global_metrics


# ── timestamp extraction ─────────────────────────────────────────────────


def extract_timestamp(msg: Any) -> float | None:
    """Best-effort extraction of a wall-clock publish timestamp from *msg*.

    Handles:
      * Objects with ``.timestamp`` (e.g. ``RawMessage``).
      * Objects with ``.header.stamp`` carrying ``.sec``/``.nanosec``
        (ROS2-style DDS IDL structs).
      * Objects with ``.ts`` (JSON envelope decoded message).
      * Dicts with ``"ts"`` or ``"timestamp"`` keys.

    Returns ``None`` when no timestamp can be found.
    """
    try:
        # RawMessage and similar: direct .timestamp attribute
        ts = getattr(msg, "timestamp", None)
        if ts is not None:
            return float(ts)

        # JSON envelope: .ts attribute
        ts = getattr(msg, "ts", None)
        if ts is not None:
            return float(ts)

        # ROS2-style header.stamp (sec + nanosec)
        header = getattr(msg, "header", None)
        if header is not None:
            stamp = getattr(header, "stamp", None)
            if stamp is not None:
                sec = getattr(stamp, "sec", 0) or 0
                nanosec = getattr(stamp, "nanosec", 0) or 0
                if sec or nanosec:
                    return float(sec) + float(nanosec) * 1e-9

        if isinstance(msg, dict):
            ts = msg.get("ts") or msg.get("timestamp")
            if ts is not None:
                return float(ts)
    except Exception:
        pass
    return None


# ── integration helpers ──────────────────────────────────────────────────


def record_publish(topic: str, size_bytes: int = 0) -> None:
    """Record a publish event when metrics are enabled (no-op otherwise)."""
    if not metrics_enabled():
        return
    try:
        get_global_metrics().record_publish(topic, size_bytes=size_bytes)
    except Exception:
        logger.debug("record_publish failed", exc_info=True)


def record_receive(topic: str, msg: Any = None, size_bytes: int = 0) -> None:
    """Record a receive event when metrics are enabled (no-op otherwise).

    Attempts to extract a publish timestamp from *msg* for latency
    computation. If *msg* is ``None`` only the count is incremented.
    """
    if not metrics_enabled():
        return
    try:
        timestamp = extract_timestamp(msg) if msg is not None else None
        get_global_metrics().record_receive(topic, timestamp=timestamp, size_bytes=size_bytes)
    except Exception:
        logger.debug("record_receive failed", exc_info=True)
