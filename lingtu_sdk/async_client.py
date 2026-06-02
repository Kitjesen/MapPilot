"""Asynchronous HTTP client for the LingTu robot Gateway.

Requires the ``aiohttp`` package.

Usage::

    async with AsyncLingTuClient("192.168.66.190", 5050) as robot:
        state = await robot.state()
        await robot.go(10.0, 5.0)
        await robot.stop()
"""

from __future__ import annotations

import json
import time
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

from lingtu_sdk.client import (
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
        """Connect to a LingTu robot Gateway.

        Args:
            host: Robot IP or hostname.
            port: Gateway HTTP port (default 5050).
            api_key: Optional shared secret sent as ``X-API-Key`` header.
            timeout: Default HTTP request timeout in seconds (default 10.0).

        Example::

            async with AsyncLingTuClient("192.168.66.190") as robot:
                ...
        """
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
        """Close the underlying HTTP session."""
        if self._session is not None:
            await self._session.close()
            self._session = None

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    async def go(self, x: float, y: float, yaw: float = 0.0) -> CommandResult:
        """Navigate to map coordinates.

        Args:
            x: X coordinate in metres (map frame).
            y: Y coordinate in metres.
            yaw: Heading in radians (default 0).

        Example::

            await robot.go(10.0, 5.0)
        """
        return await self._command("/api/v1/goal", {"x": x, "y": y, "yaw": yaw})

    async def go_to(self, target: str) -> CommandResult:
        """Navigate by semantic label or natural-language instruction.

        Args:
            target: Location name, object description, or instruction text.

        Example::

            await robot.go_to("会议室")
        """
        return await self._command("/api/v1/instruction", {"text": target})

    async def stop(self) -> CommandResult:
        """Emergency stop -- immediately halt all motion.

        Example::

            await robot.stop()
        """
        return await self._command("/api/v1/stop")

    async def cancel(self, reason: str = "client_cancel") -> CommandResult:
        """Gracefully cancel the current navigation mission.

        Args:
            reason: Cancel reason string (default ``"client_cancel"``).
        """
        return await self._command("/api/v1/navigation/cancel", {"reason": reason})

    async def navigate_click(self, x: float, y: float, yaw: float = 0.0) -> CommandResult:
        """Navigate to a map-viewer click point."""
        return await self._command("/api/v1/navigate/click", {"x": x, "y": y, "yaw": yaw})

    async def batch_go(self, waypoints: list[tuple[float, float, float]]) -> list[CommandResult]:
        """Navigate through a sequence of waypoints (blocking).

        Each waypoint is visited in order.  The coroutine returns only after
        the final waypoint has been reached (or any one fails).

        Args:
            waypoints: List of ``(x, y, yaw)`` tuples.

        Example::

            await robot.batch_go([
                (10.0, 5.0, 0.0),
                (15.0, 8.0, 1.57),
            ])
        """
        results: list[CommandResult] = []
        for i, (x, y, yaw) in enumerate(waypoints):
            result = await self.go(x, y, yaw)
            results.append(result)
            if not result.ok:
                break
            if i < len(waypoints) - 1:
                await self.wait_until_arrived()
        return results

    async def wait_until_arrived(
        self,
        timeout: float = 120.0,
        poll_interval: float = 0.5,
        distance_threshold: float = 0.3,
    ) -> NavigationStatus:
        """Wait (async) until the current navigation mission completes.

        Polls ``navigation_status()`` every *poll_interval* seconds.
        Raises :class:`TimeoutError` if the mission does not complete
        within *timeout* seconds.

        Args:
            timeout: Maximum wait time in seconds (default 120).
            poll_interval: Seconds between status checks (default 0.5).
            distance_threshold: Metres from goal considered "arrived" (default 0.3).

        Raises:
            TimeoutError: If the mission does not finish in time.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = await self.navigation_status()
            state = status.state
            if state in ("idle", "completed", "failed", "cancelled"):
                return status
            if status.distance_to_goal is not None and status.distance_to_goal <= distance_threshold:
                return status
            await asyncio_sleep(poll_interval)

        raise TimeoutError(
            f"Navigation did not complete within {timeout}s "
            f"(state={status.state}, dist={status.distance_to_goal:.2f}m)"
        )

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    async def state(self) -> RobotState:
        """Get full robot state: odometry, mission, safety, mode, etc."""
        raw = await self._get("/api/v1/state")
        return LingTuClient._parse_state(raw)

    async def health(self) -> HealthStatus:
        """Get system health overview (modules, sensors, SLAM rate, etc.)."""
        raw = await self._get("/api/v1/health")
        return HealthStatus(
            modules_ok=raw.get("modules_ok", 0),
            modules_fail=raw.get("modules_fail", 0),
            modules_total=raw.get("modules_ok", 0) + raw.get("modules_fail", 0),
            slam_rate=raw.get("slam_rate", 0.0),
            mode=raw.get("mode", ""),
            session=raw.get("session", ""),
            raw=raw,
        )

    async def position(self) -> Position:
        """Get current position as ``Position``.

        Convenience wrapper around ``state()``.
        """
        return (await self.state()).odometry

    async def session(self) -> SessionInfo:
        """Get current session state (mode, SLAM profile, active map, etc.)."""
        raw = await self._get("/api/v1/session")
        return SessionInfo(
            mode=raw.get("mode", "idle"),
            active_map=raw.get("active_map"),
            slam_profile=raw.get("slam_profile"),
            raw=raw,
        )

    async def navigation_status(self) -> NavigationStatus:
        """Get current navigation mission status."""
        raw = await self._get("/api/v1/navigation/status")
        goal_raw = raw.get("goal", {})
        return NavigationStatus(
            state=raw.get("state", "idle"),
            distance_to_goal=raw.get("distance_to_goal", 0.0),
            time_elapsed=raw.get("time_elapsed", 0.0),
            goal=Position(
                x=goal_raw.get("x", 0.0),
                y=goal_raw.get("y", 0.0),
                z=goal_raw.get("z", 0.0),
                yaw=goal_raw.get("yaw", 0.0),
            ),
            raw=raw,
        )

    async def localization_status(self) -> dict[str, Any]:
        """Get localization health: alignment, residual, fix quality."""
        return await self._get("/api/v1/localization/status")

    async def path(self) -> dict[str, Any]:
        """Get the latest planned global path as a list of poses."""
        return await self._get("/api/v1/path")

    # ------------------------------------------------------------------
    # Velocity control
    # ------------------------------------------------------------------

    async def drive(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> CommandResult:
        """Send direct velocity command to the robot.

        Args:
            vx: Linear velocity forward (m/s).
            vy: Linear velocity sideways (m/s).
            wz: Angular velocity (rad/s).
        """
        return await self._command("/api/v1/cmd_vel", {"vx": vx, "vy": vy, "wz": wz})

    # ------------------------------------------------------------------
    # Maps
    # ------------------------------------------------------------------

    async def maps(self) -> MapList:
        """List saved maps on the robot."""
        raw = await self._get("/api/v1/slam/maps")
        return LingTuClient._parse_map_list(raw)

    async def save_map(self, name: str | None = None) -> CommandResult:
        """Save the current SLAM map.

        Args:
            name: Map name.  If omitted the server auto-generates one.
        """
        body: dict[str, Any] = {}
        if name:
            body["name"] = name
        return await self._command("/api/v1/map/save", body)

    async def use_map(self, name: str) -> CommandResult:
        """Activate a saved map for navigation."""
        return await self._command("/api/v1/map/activate", {"name": name})

    async def rename_map(self, old_name: str, new_name: str) -> CommandResult:
        """Rename a saved map."""
        return await self._command("/api/v1/map/rename", {"old_name": old_name, "new_name": new_name})

    async def restore_map(self, name: str) -> CommandResult:
        """Restore a map.pcd from DUFOMap pre-filter backup."""
        return await self._command("/api/v1/map/restore_predufo", {"name": name})

    async def reset_map_cloud(self) -> CommandResult:
        """Clear the accumulated map cloud (visualisation only)."""
        return await self._command("/api/v1/map_cloud/reset")

    async def map_points(self) -> dict[str, Any]:
        """Get current live map point cloud as JSON."""
        return await self._get("/api/v1/map/points")

    async def saved_map_points(self, name: str) -> dict[str, Any]:
        """Get a saved map point cloud as JSON."""
        return await self._get(f"/api/v1/maps/{name}/points")

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------

    async def set_mode(self, mode: str) -> CommandResult:
        """Set robot operating mode (manual | autonomous | estop)."""
        return await self._command("/api/v1/mode", {"mode": mode})

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------

    async def scene(self) -> dict[str, Any]:
        """Get the current scene graph (detected objects, relations, regions)."""
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
    ) -> CommandResult:
        """Create or update a tagged navigation location.

        Args:
            name: Location label.
            x: X coordinate (required unless ``use_current_pose`` is true).
            y: Y coordinate.
            yaw: Optional heading.
            use_current_pose: If true, use the robot's current odometry pose.
        """
        body: dict[str, Any] = {"name": name, "use_current_pose": use_current_pose}
        if x is not None:
            body["x"] = x
        if y is not None:
            body["y"] = y
        if yaw is not None:
            body["yaw"] = yaw
        return await self._command("/api/v1/locations", body)

    async def delete_location(self, name: str) -> CommandResult:
        """Delete a tagged navigation location."""
        return await self._command(f"/api/v1/locations/{name}")

    # ------------------------------------------------------------------
    # Camera
    # ------------------------------------------------------------------

    async def camera_snapshot(self) -> bytes:
        """Fetch a JPEG camera snapshot from the robot.

        Returns raw JPEG bytes.
        """
        s = self._require_session()
        headers: dict[str, str] = {}
        if self._api_key:
            headers["X-API-Key"] = self._api_key
        async with s.get(self.base_url + "/api/v1/camera/snapshot", headers=headers) as resp:
            return await resp.read()

    # ------------------------------------------------------------------
    # SLAM
    # ------------------------------------------------------------------

    async def slam_status(self) -> dict[str, Any]:
        """Get SLAM service status."""
        return await self._get("/api/v1/slam/status")

    async def slam_switch(self, profile: str) -> CommandResult:
        """Hot-switch the SLAM profile."""
        return await self._command("/api/v1/slam/switch", {"profile": profile})

    async def slam_relocalize(self, x: float | None = None, y: float | None = None, yaw: float | None = None) -> CommandResult:
        """Relocalize against a saved map with optional pose guess."""
        body: dict[str, Any] = {}
        if x is not None:
            body["x"] = x
        if y is not None:
            body["y"] = y
        if yaw is not None:
            body["yaw"] = yaw
        return await self._command("/api/v1/slam/relocalize", body)

    async def slam_auto_relocalize(self) -> CommandResult:
        """Global relocalize via 3D-BBS (no pose guess required)."""
        return await self._command("/api/v1/slam/auto_relocalize")

    # ------------------------------------------------------------------
    # Exploration
    # ------------------------------------------------------------------

    async def explore_start(self) -> CommandResult:
        """Start autonomous frontier exploration."""
        return await self._command("/api/v1/explore/start")

    async def explore_stop(self) -> CommandResult:
        """Stop autonomous frontier exploration."""
        return await self._command("/api/v1/explore/stop")

    async def explore_status(self) -> dict[str, Any]:
        """Get exploration status."""
        return await self._get("/api/v1/explore/status")

    # ------------------------------------------------------------------
    # Rosbag recording
    # ------------------------------------------------------------------

    async def bag_start(self, name: str | None = None) -> CommandResult:
        """Start rosbag recording."""
        body: dict[str, Any] = {}
        if name:
            body["name"] = name
        return await self._command("/api/v1/bag/start", body)

    async def bag_stop(self) -> CommandResult:
        """Stop rosbag recording."""
        return await self._command("/api/v1/bag/stop")

    async def bag_status(self) -> dict[str, Any]:
        """Get rosbag recording status."""
        return await self._get("/api/v1/bag/status")

    # ------------------------------------------------------------------
    # Temporal memory
    # ------------------------------------------------------------------

    async def memory_temporal(self) -> dict[str, Any]:
        """Query temporal entity observations."""
        return await self._get("/api/v1/memory/temporal")

    async def memory_temporal_semantic(self, query: str) -> dict[str, Any]:
        """Semantic similarity search over temporal observations."""
        return await self._post("/api/v1/memory/temporal/semantic", {"query": query})

    # ------------------------------------------------------------------
    # Lease (control ownership)
    # ------------------------------------------------------------------

    async def acquire_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Acquire the control lease (only one client at a time)."""
        return await self._command("/api/v1/lease", {
            "action": "acquire", "client_id": client_id, "ttl": ttl,
        })

    async def release_lease(self, client_id: str) -> CommandResult:
        """Release the control lease."""
        return await self._command("/api/v1/lease", {
            "action": "release", "client_id": client_id,
        })

    async def renew_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Renew the control lease before it expires."""
        return await self._command("/api/v1/lease", {
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
    ) -> CommandResult:
        """Enter a robot session mode (mapping | navigating | exploring)."""
        body: dict[str, Any] = {"mode": mode}
        if map_name:
            body["map_name"] = map_name
        if slam_profile:
            body["slam_profile"] = slam_profile
        return await self._command("/api/v1/session/start", body)

    async def end_session(self) -> CommandResult:
        """End the current robot session."""
        return await self._command("/api/v1/session/end")

    # ------------------------------------------------------------------
    # Driver swap
    # ------------------------------------------------------------------

    async def swap_driver(self, driver: str, config: dict[str, Any] | None = None) -> CommandResult:
        """Hot-swap the robot driver at runtime.

        Args:
            driver: Driver name (e.g. ``"thunder"``, ``"stub"``).
            config: Optional driver-specific configuration.
        """
        body: dict[str, Any] = {"driver": driver}
        if config:
            body["config"] = config
        return await self._command("/api/v1/driver/swap", body)

    async def swap(self, driver: str, config: dict[str, Any] | None = None) -> CommandResult:
        """Alias for :meth:`swap_driver`.

        Shorter name for interactive use.
        """
        return await self.swap_driver(driver, config)

    # ------------------------------------------------------------------
    # Runtime backend
    # ------------------------------------------------------------------

    async def switch_backend(self, category: str, backend: str, config: dict[str, Any] | None = None) -> CommandResult:
        """Switch a runtime backend (non-motion)."""
        body: dict[str, Any] = {"category": category, "backend": backend}
        if config:
            body["config"] = config
        return await self._command("/api/v1/runtime/backend", body)

    # ------------------------------------------------------------------
    # App / system info
    # ------------------------------------------------------------------

    async def capabilities(self) -> dict[str, Any]:
        """Get the API capability manifest."""
        return await self._get("/api/v1/app/capabilities")

    async def bootstrap(self) -> dict[str, Any]:
        """Get the bootstrap snapshot (full system state for web clients)."""
        return await self._get("/api/v1/app/bootstrap")

    async def devices(self) -> dict[str, Any]:
        """Get the hardware device registry status."""
        return await self._get("/api/v1/devices")

    async def readiness(self) -> dict[str, Any]:
        """Get client readiness snapshot."""
        return await self._get("/api/v1/readiness")

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    async def field_check(self) -> CommandResult:
        """Run a read-only product field readiness check."""
        return await self._command("/api/v1/diagnostics/field-check")

    async def runtime_contract(self) -> dict[str, Any]:
        """Get the canonical runtime interface contract."""
        return await self._get("/api/v1/diagnostics/runtime-contract")

    # ------------------------------------------------------------------
    # Auth
    # ------------------------------------------------------------------

    async def auth_login(self, api_key: str) -> dict[str, Any]:
        """Login with an API key."""
        return await self._post("/api/v1/auth/login", {"api_key": api_key})

    async def auth_check(self) -> dict[str, Any]:
        """Check if authentication is required."""
        return await self._get("/api/v1/auth/check")

    # ------------------------------------------------------------------
    # Internal HTTP helpers
    # ------------------------------------------------------------------

    async def _command(self, path: str, data: dict[str, Any] | None = None) -> CommandResult:
        raw = await self._post(path, data)
        ok = raw.get("ok", False) or raw.get("success", False)
        if not ok and "error" not in raw and "message" not in raw:
            ok = True  # no error signal = success
        return CommandResult(
            ok=ok,
            message=raw.get("message", raw.get("reason", raw.get("error", ""))),
            raw=raw,
        )

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


# ---------------------------------------------------------------------------
# asyncio sleep helper — Python 3.13+ asyncio.sleep is fine, just an alias
# ---------------------------------------------------------------------------
import asyncio

asyncio_sleep = asyncio.sleep
