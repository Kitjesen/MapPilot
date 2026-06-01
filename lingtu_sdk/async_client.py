"""Asynchronous HTTP client for the LingTu robot Gateway.

Requires the ``aiohttp`` package.
"""

from __future__ import annotations

import json
from typing import Any

try:
    import aiohttp
    import aiohttp.client_exceptions
except ImportError:  # pragma: no cover
    msg = (
        "The async LingTu client requires aiohttp. "
        "Install it with: pip install lingtu-sdk[aiohttp]"
    )
    raise ImportError(msg) from None


class AsyncLingTuClient:
    """Asynchronous client for LingTu robot control and state inspection.

    Usage::

        async with AsyncLingTuClient("192.168.66.190", 5050) as robot:
            state = await robot.state()
            await robot.go(10.0, 5.0)
            await robot.stop()
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5050,
        api_key: str | None = None,
        timeout: float = 10.0,
    ) -> None:
        self.base_url = f"http://{host}:{port}"
        self._api_key = api_key
        self._timeout = aiohttp.ClientTimeout(total=timeout)
        self._session: aiohttp.ClientSession | None = None

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    async def __aenter__(self) -> AsyncLingTuClient:
        self._session = aiohttp.ClientSession(self._timeout)
        return self

    async def __aexit__(self, *args: object) -> None:
        await self.close()

    async def close(self) -> None:
        if self._session is not None:
            await self._session.close()
            self._session = None

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    async def go(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        """Navigate to map coordinates."""
        return await self._post("/api/v1/goal", {"x": x, "y": y, "yaw": yaw})

    async def go_to(self, target: str) -> dict[str, Any]:
        """Navigate by semantic label or natural-language instruction."""
        return await self._post("/api/v1/instruction", {"text": target})

    async def stop(self) -> dict[str, Any]:
        """Emergency stop — immediately halt all motion."""
        return await self._post("/api/v1/stop")

    async def cancel(self, reason: str = "client_cancel") -> dict[str, Any]:
        """Gracefully cancel the current navigation mission."""
        return await self._post("/api/v1/navigation/cancel", {"reason": reason})

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    async def state(self) -> dict[str, Any]:
        """Get full robot state."""
        return await self._get("/api/v1/state")

    async def health(self) -> dict[str, Any]:
        """Get system health overview."""
        return await self._get("/api/v1/health")

    async def position(self) -> dict[str, float]:
        """Get current position as {x, y, z, yaw}."""
        s = await self.state()
        o = s.get("odometry", {})
        return {
            "x": o.get("x", 0.0),
            "y": o.get("y", 0.0),
            "z": o.get("z", 0.0),
            "yaw": o.get("yaw", 0.0),
        }

    async def session(self) -> dict[str, Any]:
        """Get current session state."""
        return await self._get("/api/v1/session")

    # ------------------------------------------------------------------
    # Velocity control
    # ------------------------------------------------------------------

    async def drive(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> dict[str, Any]:
        """Send direct velocity command."""
        return await self._post("/api/v1/cmd_vel", {"vx": vx, "vy": vy, "wz": wz})

    # ------------------------------------------------------------------
    # Maps
    # ------------------------------------------------------------------

    async def maps(self) -> dict[str, Any]:
        """List saved maps on the robot."""
        return await self._get("/api/v1/slam/maps")

    async def save_map(self, name: str | None = None) -> dict[str, Any]:
        """Save the current SLAM map."""
        body: dict[str, Any] = {}
        if name:
            body["name"] = name
        return await self._post("/api/v1/map/save", body)

    async def use_map(self, name: str) -> dict[str, Any]:
        """Activate a saved map for navigation."""
        return await self._post("/api/v1/map/activate", {"name": name})

    async def rename_map(self, old_name: str, new_name: str) -> dict[str, Any]:
        """Rename a saved map."""
        return await self._post("/api/v1/map/rename", {"old_name": old_name, "new_name": new_name})

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------

    async def set_mode(self, mode: str) -> dict[str, Any]:
        """Set robot operating mode (manual | autonomous | estop)."""
        return await self._post("/api/v1/mode", {"mode": mode})

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------

    async def scene(self) -> dict[str, Any]:
        """Get the current scene graph."""
        return await self._get("/api/v1/scene_graph")

    async def locations(self) -> dict[str, Any]:
        """List tagged navigation locations."""
        return await self._get("/api/v1/locations")

    async def tag_location(
        self,
        name: str,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        use_current_pose: bool = False,
    ) -> dict[str, Any]:
        """Create or update a tagged navigation location."""
        body: dict[str, Any] = {"name": name, "use_current_pose": use_current_pose}
        if x is not None:
            body["x"] = x
        if y is not None:
            body["y"] = y
        if yaw is not None:
            body["yaw"] = yaw
        return await self._post("/api/v1/locations", body)

    # ------------------------------------------------------------------
    # Lease (control ownership)
    # ------------------------------------------------------------------

    async def acquire_lease(self, client_id: str, ttl: float = 30.0) -> dict[str, Any]:
        return await self._post("/api/v1/lease", {
            "action": "acquire", "client_id": client_id, "ttl": ttl,
        })

    async def release_lease(self, client_id: str) -> dict[str, Any]:
        return await self._post("/api/v1/lease", {
            "action": "release", "client_id": client_id,
        })

    async def renew_lease(self, client_id: str, ttl: float = 30.0) -> dict[str, Any]:
        return await self._post("/api/v1/lease", {
            "action": "renew", "client_id": client_id, "ttl": ttl,
        })

    # ------------------------------------------------------------------
    # Session lifecycle
    # ------------------------------------------------------------------

    async def start_session(
        self,
        mode: str,
        map_name: str | None = None,
        slam_profile: str | None = None,
    ) -> dict[str, Any]:
        body: dict[str, Any] = {"mode": mode}
        if map_name:
            body["map_name"] = map_name
        if slam_profile:
            body["slam_profile"] = slam_profile
        return await self._post("/api/v1/session/start", body)

    async def end_session(self) -> dict[str, Any]:
        return await self._post("/api/v1/session/end")

    # ------------------------------------------------------------------
    # Driver swap (Phase 2)
    # ------------------------------------------------------------------

    async def swap_driver(
        self,
        driver: str,
        config: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        body: dict[str, Any] = {"driver": driver}
        if config:
            body["config"] = config
        return await self._post("/api/v1/driver/swap", body)

    # ------------------------------------------------------------------
    # Internal HTTP helpers
    # ------------------------------------------------------------------

    async def _get(self, path: str) -> dict[str, Any]:
        s = self._require_session()
        headers: dict[str, str] = {}
        if self._api_key:
            headers["X-API-Key"] = self._api_key
        async with s.get(self.base_url + path, headers=headers) as resp:
            return await resp.json()

    async def _post(self, path: str, data: dict[str, Any] | None = None) -> dict[str, Any]:
        s = self._require_session()
        headers: dict[str, str] = {"Content-Type": "application/json"}
        if self._api_key:
            headers["X-API-Key"] = self._api_key
        async with s.post(
            self.base_url + path,
            data=json.dumps(data or {}).encode("utf-8"),
            headers=headers,
        ) as resp:
            return await resp.json()

    def _require_session(self) -> aiohttp.ClientSession:
        if self._session is None:
            raise RuntimeError(
                "AsyncLingTuClient session is not open. "
                "Use `async with AsyncLingTuClient(...) as robot:` or "
                "call `await robot.__aenter__()` first."
            )
        return self._session
