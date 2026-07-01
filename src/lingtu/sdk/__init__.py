"""LingTu Python SDK -- programmatic robot control and state inspection.

Provides typed clients for the Gateway REST API and MCP JSON-RPC tool
interface.  The sync client (``LingTuClient``) uses only the Python stdlib;
the async client (``AsyncLingTuClient``) requires ``aiohttp``.
"""

from __future__ import annotations

from lingtu.sdk.client import (
    CommandResult,
    HealthStatus,
    LingTuClient,
    MapInfo,
    MapList,
    NavigationStatus,
    Position,
    RobotState,
    SessionInfo,
)
from lingtu.sdk.config import LingTuConfig

__all__ = [
    "CommandResult",
    "HealthStatus",
    "LingTuClient",
    "LingTuConfig",
    "MapInfo",
    "MapList",
    "NavigationStatus",
    "Position",
    "RobotState",
    "SessionInfo",
]


def __getattr__(name: str):
    """Load optional SDK clients only when they are requested."""

    if name == "AsyncLingTuClient":
        from lingtu.sdk.async_client import AsyncLingTuClient

        return AsyncLingTuClient
    raise AttributeError(f"module 'lingtu.sdk' has no attribute {name!r}")
