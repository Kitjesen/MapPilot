"""WebSocket connection registry and heartbeat monitoring.

Provides a centralized registry for tracking active WebSocket connections,
detecting idle/dead connections, and exposing connection metrics for
diagnostics endpoints.

Environment variables:
  LINGTU_WS_HEARTBEAT_INTERVAL_S  - server ping interval (default: 15.0)
  LINGTU_WS_HEARTBEAT_TIMEOUT_S   - max time without pong (default: 30.0)
  LINGTU_WS_MAX_IDLE_S            - max idle time before cleanup (default: 120.0)
"""

from __future__ import annotations

import logging
import os
import threading
import time
from typing import Any

logger = logging.getLogger(__name__)

_DEFAULT_HEARTBEAT_INTERVAL_S = 15.0
_DEFAULT_HEARTBEAT_TIMEOUT_S = 30.0
_DEFAULT_MAX_IDLE_S = 120.0


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default


class WebSocketConnectionInfo:
    """Metadata for a single WebSocket connection."""

    __slots__ = (
        "conn_id",
        "endpoint",
        "client_id",
        "client_ip",
        "connected_at",
        "last_activity",
        "last_pong",
        "messages_rx",
        "messages_tx",
        "bytes_tx",
    )

    def __init__(
        self,
        conn_id: str,
        endpoint: str,
        client_id: str = "unknown",
        client_ip: str | None = None,
    ) -> None:
        now = time.monotonic()
        self.conn_id = conn_id
        self.endpoint = endpoint
        self.client_id = client_id
        self.client_ip = client_ip
        self.connected_at = now
        self.last_activity = now
        self.last_pong = now
        self.messages_rx = 0
        self.messages_tx = 0
        self.bytes_tx = 0

    def touch(self) -> None:
        """Update last activity timestamp."""
        self.last_activity = time.monotonic()

    def record_rx(self) -> None:
        self.messages_rx += 1
        self.touch()

    def record_tx(self, nbytes: int = 0) -> None:
        self.messages_tx += 1
        self.bytes_tx += nbytes
        self.touch()

    def record_pong(self) -> None:
        self.last_pong = time.monotonic()

    def idle_seconds(self) -> float:
        return time.monotonic() - self.last_activity

    def since_pong(self) -> float:
        return time.monotonic() - self.last_pong

    def to_dict(self) -> dict[str, Any]:
        return {
            "conn_id": self.conn_id,
            "endpoint": self.endpoint,
            "client_id": self.client_id,
            "client_ip": self.client_ip,
            "connected_at": self.connected_at,
            "idle_s": round(self.idle_seconds(), 1),
            "since_pong_s": round(self.since_pong(), 1),
            "messages_rx": self.messages_rx,
            "messages_tx": self.messages_tx,
            "bytes_tx": self.bytes_tx,
        }


class WebSocketRegistry:
    """Thread-safe registry of active WebSocket connections.

    Provides:
      - Connection tracking with metadata
      - Idle connection detection
      - Heartbeat timeout detection
      - Metrics for diagnostics endpoints
    """

    def __init__(
        self,
        *,
        heartbeat_interval_s: float | None = None,
        heartbeat_timeout_s: float | None = None,
        max_idle_s: float | None = None,
    ) -> None:
        self._lock = threading.Lock()
        self._connections: dict[str, WebSocketConnectionInfo] = {}
        self._heartbeat_interval_s = (
            _env_float("LINGTU_WS_HEARTBEAT_INTERVAL_S", _DEFAULT_HEARTBEAT_INTERVAL_S)
            if heartbeat_interval_s is None
            else heartbeat_interval_s
        )
        self._heartbeat_timeout_s = (
            _env_float("LINGTU_WS_HEARTBEAT_TIMEOUT_S", _DEFAULT_HEARTBEAT_TIMEOUT_S)
            if heartbeat_timeout_s is None
            else heartbeat_timeout_s
        )
        self._max_idle_s = (
            _env_float("LINGTU_WS_MAX_IDLE_S", _DEFAULT_MAX_IDLE_S)
            if max_idle_s is None
            else max_idle_s
        )
        # Cumulative stats
        self._total_connections = 0
        self._total_disconnections = 0
        self._total_timeout_disconnects = 0

    def register(
        self,
        conn_id: str,
        endpoint: str,
        client_id: str = "unknown",
        client_ip: str | None = None,
    ) -> WebSocketConnectionInfo:
        """Register a new WebSocket connection."""
        info = WebSocketConnectionInfo(conn_id, endpoint, client_id, client_ip)
        with self._lock:
            self._connections[conn_id] = info
            self._total_connections += 1
        logger.debug(
            "WS registered: %s endpoint=%s client=%s",
            conn_id,
            endpoint,
            client_id,
        )
        return info

    def unregister(self, conn_id: str) -> None:
        """Remove a connection from the registry."""
        with self._lock:
            if conn_id in self._connections:
                del self._connections[conn_id]
                self._total_disconnections += 1
        logger.debug("WS unregistered: %s", conn_id)

    def get(self, conn_id: str) -> WebSocketConnectionInfo | None:
        with self._lock:
            return self._connections.get(conn_id)

    def find_stale_connections(self) -> list[str]:
        """Return conn_ids that have exceeded heartbeat timeout or max idle."""
        stale: list[str] = []
        with self._lock:
            for conn_id, info in self._connections.items():
                if info.since_pong() > self._heartbeat_timeout_s:
                    stale.append(conn_id)
                elif info.idle_seconds() > self._max_idle_s:
                    stale.append(conn_id)
        return stale

    def mark_timeout_disconnect(self, conn_id: str) -> None:
        """Record a timeout-triggered disconnect for metrics."""
        with self._lock:
            self._total_timeout_disconnects += 1
            if conn_id in self._connections:
                del self._connections[conn_id]
                self._total_disconnections += 1

    def snapshot(self) -> dict[str, Any]:
        """Return registry metrics for diagnostics endpoints."""
        with self._lock:
            by_endpoint: dict[str, int] = {}
            total_idle_s = 0.0
            for info in self._connections.values():
                by_endpoint[info.endpoint] = by_endpoint.get(info.endpoint, 0) + 1
                total_idle_s += info.idle_seconds()
            active_count = len(self._connections)
            return {
                "active_connections": active_count,
                "by_endpoint": by_endpoint,
                "total_connections": self._total_connections,
                "total_disconnections": self._total_disconnections,
                "timeout_disconnects": self._total_timeout_disconnects,
                "avg_idle_s": round(total_idle_s / active_count, 1) if active_count else 0.0,
                "heartbeat_interval_s": self._heartbeat_interval_s,
                "heartbeat_timeout_s": self._heartbeat_timeout_s,
                "max_idle_s": self._max_idle_s,
            }

    def connections_detail(self) -> list[dict[str, Any]]:
        """Return detailed info for all active connections."""
        with self._lock:
            return [info.to_dict() for info in self._connections.values()]
