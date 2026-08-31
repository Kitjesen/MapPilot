"""WebSocket connection registry used by Gateway diagnostics."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

logger = logging.getLogger(__name__)

class WebSocketConnectionInfo:
    """Metadata for a single WebSocket connection."""

    __slots__ = (
        "client_id",
        "client_ip",
        "conn_id",
        "connected_at",
        "endpoint",
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

    def to_dict(self) -> dict[str, Any]:
        """Return only connection facts observed by Gateway."""

        return {
            "conn_id": self.conn_id,
            "endpoint": self.endpoint,
            "client_id": self.client_id,
            "client_ip": self.client_ip,
            "connected_at": self.connected_at,
            "connected_s": round(time.monotonic() - self.connected_at, 1),
        }


class WebSocketRegistry:
    """Thread-safe registry of connections that Gateway actually observes."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._connections: dict[str, WebSocketConnectionInfo] = {}
        self._total_connections = 0
        self._total_disconnections = 0

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
        """Return one registered connection, if present."""

        with self._lock:
            return self._connections.get(conn_id)

    def snapshot(self) -> dict[str, Any]:
        """Return registry metrics for diagnostics endpoints."""
        with self._lock:
            by_endpoint: dict[str, int] = {}
            for info in self._connections.values():
                by_endpoint[info.endpoint] = by_endpoint.get(info.endpoint, 0) + 1
            active_count = len(self._connections)
            return {
                "active_connections": active_count,
                "by_endpoint": by_endpoint,
                "total_connections": self._total_connections,
                "total_disconnections": self._total_disconnections,
            }

    def connections_detail(self) -> list[dict[str, Any]]:
        """Return detailed info for all active connections."""
        with self._lock:
            return [info.to_dict() for info in self._connections.values()]
