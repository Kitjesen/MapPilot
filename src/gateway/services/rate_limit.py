"""Sliding-window rate limiting middleware for Gateway.

Pure ASGI middleware (consistent with auth.py) that enforces per-client
request rate limits.  Limits are configurable via environment variables and
differentiated by endpoint category:

  - Control endpoints (goal, stop, cmd_vel, mode): stricter limits
  - Query endpoints (state, health, status): moderate limits
  - SSE/WebSocket: exempt (long-lived connections)

When a limit is exceeded the middleware returns HTTP 429 with a
``Retry-After`` header.  WebSocket upgrade requests are never rate-limited.

Environment variables:
  LINGTU_RATE_LIMIT_ENABLED   - "1"/"true" to enable (default: enabled)
  LINGTU_RATE_LIMIT_CONTROL   - max req/s for control endpoints (default: 10)
  LINGTU_RATE_LIMIT_QUERY     - max req/s for query endpoints (default: 30)
  LINGTU_RATE_LIMIT_WINDOW_S  - sliding window size (default: 1.0)
  LINGTU_RATE_LIMIT_BURST     - burst multiplier (default: 3)
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from collections import defaultdict
from typing import Any

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

_DEFAULT_CONTROL_RPS = 10.0
_DEFAULT_QUERY_RPS = 30.0
_DEFAULT_WINDOW_S = 1.0
_DEFAULT_BURST = 3

# Paths subject to control rate limits (prefix match)
_CONTROL_PREFIXES = (
    "/api/v1/goal",
    "/api/v1/cmd_vel",
    "/api/v1/stop",
    "/api/v1/estop/reset",
    "/api/v1/mode",
    "/api/v1/instruction",
    "/api/v1/navigation/cancel",
    "/api/v1/lease",
    "/api/v1/driver/swap",
    "/api/v1/runtime/backend",
)

# Paths exempt from rate limiting (long-lived or probes)
_EXEMPT_PREFIXES = (
    "/ws/",
    "/api/v1/events",
    "/docs",
    "/redoc",
    "/openapi.json",
)
_EXEMPT_EXACT = {"/health", "/ready", "/"}


def _is_control_path(path: str) -> bool:
    if any(path.startswith(prefix) for prefix in _CONTROL_PREFIXES):
        return True
    return (
        path.startswith("/api/v1/navigation/tasks/")
        and path.rsplit("/", 1)[-1] in {"cancel", "pause", "resume"}
    )


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default


def _env_enabled(name: str, default: bool = True) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in {"0", "false", "no", "off"}


# ---------------------------------------------------------------------------
# Sliding window counter
# ---------------------------------------------------------------------------


class _SlidingWindowCounter:
    """Thread-safe sliding window request counter per client key."""

    __slots__ = ("_lock", "_max_requests", "_window_s", "_windows")

    def __init__(self, window_s: float, max_requests: float) -> None:
        self._lock = threading.Lock()
        self._windows: dict[str, list[float]] = defaultdict(list)
        self._window_s = window_s
        self._max_requests = max_requests

    def allow(self, key: str) -> tuple[bool, float]:
        """Return (allowed, retry_after_s)."""
        now = time.monotonic()
        cutoff = now - self._window_s
        with self._lock:
            timestamps = self._windows[key]
            # Prune expired entries
            while timestamps and timestamps[0] < cutoff:
                timestamps.pop(0)
            if len(timestamps) >= self._max_requests:
                oldest = timestamps[0] if timestamps else now
                retry_after = self._window_s - (now - oldest)
                return False, max(0.1, retry_after)
            timestamps.append(now)
            return True, 0.0

    def cleanup(self, max_age_s: float = 60.0) -> int:
        """Remove stale client entries. Returns count of removed entries."""
        now = time.monotonic()
        cutoff = now - max_age_s
        removed = 0
        with self._lock:
            stale_keys = [
                key
                for key, timestamps in self._windows.items()
                if not timestamps or timestamps[-1] < cutoff
            ]
            for key in stale_keys:
                del self._windows[key]
                removed += 1
        return removed


# ---------------------------------------------------------------------------
# Middleware
# ---------------------------------------------------------------------------


class RateLimitMiddleware:
    """Pure ASGI middleware for per-client sliding-window rate limiting.

    Differentiates between control and query endpoints. WebSocket and SSE
    connections are exempt.
    """

    def __init__(
        self,
        app: Any,
        *,
        enabled: bool | None = None,
        control_rps: float | None = None,
        query_rps: float | None = None,
        window_s: float | None = None,
        burst: float | None = None,
    ) -> None:
        self.app = app
        self._enabled = (
            _env_enabled("LINGTU_RATE_LIMIT_ENABLED")
            if enabled is None
            else enabled
        )
        resolved_window = (
            _env_float("LINGTU_RATE_LIMIT_WINDOW_S", _DEFAULT_WINDOW_S)
            if window_s is None
            else window_s
        )
        resolved_burst = (
            _env_float("LINGTU_RATE_LIMIT_BURST", _DEFAULT_BURST)
            if burst is None
            else burst
        )
        resolved_control = (
            _env_float("LINGTU_RATE_LIMIT_CONTROL", _DEFAULT_CONTROL_RPS)
            if control_rps is None
            else control_rps
        )
        resolved_query = (
            _env_float("LINGTU_RATE_LIMIT_QUERY", _DEFAULT_QUERY_RPS)
            if query_rps is None
            else query_rps
        )

        self._control_counter = _SlidingWindowCounter(
            window_s=resolved_window,
            max_requests=resolved_control * resolved_burst,
        )
        self._query_counter = _SlidingWindowCounter(
            window_s=resolved_window,
            max_requests=resolved_query * resolved_burst,
        )
        self._rejected_total = 0
        self._stats_lock = threading.Lock()

        if self._enabled:
            logger.info(
                "Rate limiting enabled: control=%.0f req/s, query=%.0f req/s, "
                "window=%.1fs, burst=x%.0f",
                resolved_control,
                resolved_query,
                resolved_window,
                resolved_burst,
            )

    async def __call__(self, scope: dict, receive: Any, send: Any) -> None:
        """Apply the configured rate limit to one ASGI request."""
        if not self._enabled or scope["type"] not in ("http",):
            return await self.app(scope, receive, send)

        path = scope.get("path", "")

        # Exempt paths
        if path in _EXEMPT_EXACT:
            return await self.app(scope, receive, send)
        for prefix in _EXEMPT_PREFIXES:
            if path.startswith(prefix):
                return await self.app(scope, receive, send)

        # Only rate-limit /api/ paths
        if not path.startswith("/api/"):
            return await self.app(scope, receive, send)

        client_key = self._client_key(scope)
        is_control = _is_control_path(path)
        counter = self._control_counter if is_control else self._query_counter

        allowed, retry_after = counter.allow(client_key)
        if allowed:
            return await self.app(scope, receive, send)

        with self._stats_lock:
            self._rejected_total += 1

        await self._reject(send, retry_after, is_control)

    # ------------------------------------------------------------------

    @staticmethod
    def _client_key(scope: dict) -> str:
        """Extract client identifier (IP-based)."""
        client = scope.get("client")
        if client:
            return str(client[0])
        return "unknown"

    @staticmethod
    async def _reject(send: Any, retry_after: float, is_control: bool) -> None:
        """Return 429 Too Many Requests."""
        category = "control" if is_control else "query"
        body = json.dumps(
            {
                "error": "rate_limited",
                "message": f"Too many {category} requests. Slow down.",
                "retry_after_s": round(retry_after, 2),
            }
        ).encode()
        await send(
            {
                "type": "http.response.start",
                "status": 429,
                "headers": [
                    (b"content-type", b"application/json; charset=utf-8"),
                    (b"content-length", str(len(body)).encode()),
                    (b"retry-after", str(int(retry_after) + 1).encode()),
                ],
            }
        )
        await send({"type": "http.response.body", "body": body})

    def stats_snapshot(self) -> dict[str, Any]:
        """Return rate limiter statistics for /api/v1/metrics."""
        with self._stats_lock:
            return {
                "enabled": self._enabled,
                "rejected_total": self._rejected_total,
            }
