"""Asynchronous HTTP client for the LingTu robot Gateway.

Requires the ``aiohttp`` package.

Usage::

    async with AsyncLingTuClient("192.168.66.190", 5050) as robot:
        state = await robot.state()
        await robot.go(10.0, 5.0)
        await robot.stop()
"""

from __future__ import annotations

import asyncio
import json
import tempfile
import time
from pathlib import Path
from typing import Any
from urllib.parse import quote

try:
    import aiohttp
    import aiohttp.client_exceptions
except ImportError:  # pragma: no cover
    msg = "The async LingTu client requires aiohttp. Install it with: pip install 'lingtu[sdk-async]'"
    raise ImportError(msg) from None

from lingtu.sdk.client import (
    CommandResult,
    HealthStatus,
    LingTuClient,
    MapList,
    NavigationStatus,
    Position,
    RobotState,
    SessionInfo,
    _connection_error_payload,
    _mapping,
    _navigation_failure_reason,
    _navigation_state,
    _navigation_wait_outcome,
    _new_request_id,
    _parse_command_result,
    _parse_navigation_status,
    _save_operation_failure_reason,
    _save_response_is_connection_unknown,
    _validate_save_operation_identity,
    _validated_map_save_receipt,
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
        self._session = aiohttp.ClientSession(timeout=self._timeout)
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

    async def go(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Navigate to map coordinates.

        Args:
            x: X coordinate in metres (map frame).
            y: Y coordinate in metres.
            yaw: Heading in radians (default 0).

        Example::

            await robot.go(10.0, 5.0)
        """
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        result = await self._command(
            "/api/v1/goal",
            {"x": x, "y": y, "yaw": yaw, "request_id": resolved_request_id},
        )
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

    async def go_to(self, target: str) -> CommandResult:
        """Navigate by semantic label or natural-language instruction.

        Args:
            target: Location name, object description, or instruction text.

        Example::

            await robot.go_to("浼氳瀹?)
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
        for x, y, yaw in waypoints:
            baseline = await self.navigation_status()
            result = await self.go(x, y, yaw)
            results.append(result)
            if not result.ok:
                break
            await self.wait_until_arrived(
                request_id=result.request_id,
                expected_goal=(x, y, yaw),
                baseline=baseline,
            )
        return results

    async def wait_until_arrived(
        self,
        timeout: float = 120.0,
        poll_interval: float = 0.5,
        distance_threshold: float = 0.3,
        *,
        request_id: str | None = None,
        expected_goal: tuple[float, float, float] | None = None,
        baseline: NavigationStatus | None = None,
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
            RuntimeError: If the mission fails or is cancelled.
        """
        deadline = time.monotonic() + timeout
        active_seen = False
        status = baseline or NavigationStatus()
        while time.monotonic() < deadline:
            status = await self.navigation_status()
            active_seen, outcome = _navigation_wait_outcome(
                status,
                request_id=request_id,
                expected_goal=expected_goal,
                baseline=baseline,
                active_seen=active_seen,
                distance_threshold=distance_threshold,
            )
            if outcome == "success":
                return status
            if outcome == "failure":
                state = _navigation_state(status)
                raise RuntimeError(f"Navigation {state}: {_navigation_failure_reason(status)}")
            await asyncio_sleep(poll_interval)

        distance = "unknown" if status.distance_to_goal is None else f"{status.distance_to_goal:.2f}m"
        raise TimeoutError(f"Navigation did not complete within {timeout}s (state={status.state}, dist={distance})")

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
        return _parse_navigation_status(raw)

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

    async def save_map(
        self,
        name: str | None = None,
        *,
        optimization: str | None = None,
        request_id: str | None = None,
    ) -> CommandResult:
        """Queue a durable save of the current SLAM map."""
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        body: dict[str, Any] = {"request_id": resolved_request_id}
        if name:
            body["name"] = name
        if optimization is not None:
            body["optimization"] = optimization
        result = await self._command("/api/v1/map/save", body)
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

    async def save_map_and_wait(
        self,
        name: str | None = None,
        *,
        optimization: str | None = None,
        request_id: str | None = None,
        timeout: float = 300.0,
        poll_interval: float = 1.0,
    ) -> dict[str, Any]:
        """Save the current map and wait for its durable terminal result."""
        deadline = time.monotonic() + timeout
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        while True:
            receipt = await self.save_map(
                name,
                optimization=optimization,
                request_id=resolved_request_id,
            )
            operation_id, submission_unknown = _validated_map_save_receipt(
                receipt,
                resolved_request_id,
            )
            if receipt.ok and receipt.accepted and operation_id:
                break
            if not submission_unknown:
                raise RuntimeError("Map save response did not include operation_id")
            if time.monotonic() >= deadline:
                raise TimeoutError("Map save admission remained unknown until the timeout")
            await asyncio_sleep(poll_interval)
        state = ""
        while time.monotonic() < deadline:
            response = await self.get_map_operation(operation_id)
            _validate_save_operation_identity(response, operation_id)
            operation = _mapping(response.get("operation"))
            state = str(operation.get("state") or "").strip().upper()
            if state == "SUCCEEDED":
                return response
            if state in {"FAILED", "CANCELLED", "CANCELED"}:
                raise RuntimeError(f"Map save {state}: {_save_operation_failure_reason(response)}")
            reason_code = str(operation.get("reason_code") or response.get("reason_code") or "").strip().lower()
            transient = reason_code == "operation_not_found" or _save_response_is_connection_unknown(response)
            explicit_error = (
                response.get("ok") is False
                or response.get("success") is False
                or response.get("error") is not None
                or response.get("detail") is not None
                or operation.get("error") is not None
                or operation.get("detail") is not None
                or state in {"ERROR", "REJECTED", "NOT_FOUND"}
            )
            if explicit_error and not transient:
                raise RuntimeError(f"Map save status query failed: {_save_operation_failure_reason(response)}")
            await asyncio_sleep(poll_interval)
        raise TimeoutError(f"Map save did not complete within {timeout}s (state={state or 'UNKNOWN'})")

    async def get_map_operation(self, operation_id: str) -> dict[str, Any]:
        """Return the durable state of one map-save operation."""
        encoded_operation_id = quote(operation_id, safe="")
        return await self._get(f"/api/v1/maps/operations/{encoded_operation_id}")

    async def cancel_map_operation(self, operation_id: str) -> CommandResult:
        """Request cancellation of one map-save operation."""
        encoded_operation_id = quote(operation_id, safe="")
        return await self._command(f"/api/v1/maps/operations/{encoded_operation_id}/cancel")

    async def retry_map_operation(self, operation_id: str) -> CommandResult:
        """Retry one failed map-save operation."""
        encoded_operation_id = quote(operation_id, safe="")
        return await self._command(f"/api/v1/maps/operations/{encoded_operation_id}/retry")

    async def download_map_pcd(self, name: str, target: str | Path) -> Path:
        """Stream a saved map PCD artifact to ``target`` atomically."""
        destination = Path(target)
        encoded_name = quote(name, safe="")
        headers: dict[str, str] = {}
        if self._api_key:
            headers["X-API-Key"] = self._api_key
        session = self._require_session()
        temporary_path: Path | None = None
        try:
            async with session.get(
                self.base_url + f"/api/v1/maps/{encoded_name}/pcd",
                headers=headers,
            ) as response:
                response.raise_for_status()
                with tempfile.NamedTemporaryFile(
                    mode="wb",
                    dir=destination.parent,
                    prefix=f".{destination.name}.",
                    suffix=".part",
                    delete=False,
                ) as output:
                    temporary_path = Path(output.name)
                    async for chunk in response.content.iter_chunked(1024 * 1024):
                        if chunk:
                            await asyncio.to_thread(output.write, chunk)
            temporary_path.replace(destination)
        except BaseException:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)
            raise
        return destination

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

    async def slam_relocalize(
        self,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
    ) -> CommandResult:
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
    # Native MCAP recording
    # ------------------------------------------------------------------

    async def recording_start(
        self,
        prefix: str | None = None,
        *,
        duration: int = 600,
    ) -> CommandResult:
        """Start a native MCAP recording session."""
        body: dict[str, Any] = {"duration": duration}
        if prefix:
            body["prefix"] = prefix
        return await self._command("/api/v1/recordings/start", body)

    async def recording_stop(self) -> CommandResult:
        """Stop the active native MCAP recording session."""
        return await self._command("/api/v1/recordings/stop")

    async def recording_status(self) -> dict[str, Any]:
        """Get native MCAP recording status."""
        return await self._get("/api/v1/recordings/status")

    async def bag_start(self, name: str | None = None) -> CommandResult:
        """Deprecated alias for :meth:`recording_start`."""
        return await self.recording_start(name)

    async def bag_stop(self) -> CommandResult:
        """Deprecated alias for :meth:`recording_stop`."""
        return await self.recording_stop()

    async def bag_status(self) -> dict[str, Any]:
        """Deprecated alias for :meth:`recording_status`."""
        return await self.recording_status()

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
        return await self._command(
            "/api/v1/lease",
            {
                "action": "acquire",
                "client_id": client_id,
                "ttl": ttl,
            },
        )

    async def release_lease(self, client_id: str) -> CommandResult:
        """Release the control lease."""
        return await self._command(
            "/api/v1/lease",
            {
                "action": "release",
                "client_id": client_id,
            },
        )

    async def renew_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Renew the control lease before it expires."""
        return await self._command(
            "/api/v1/lease",
            {
                "action": "renew",
                "client_id": client_id,
                "ttl": ttl,
            },
        )

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
        return _parse_command_result(raw)

    async def _get(self, path: str) -> dict[str, Any]:
        s = self._require_session()
        headers: dict[str, str] = {}
        if self._api_key:
            headers["X-API-Key"] = self._api_key
        try:
            async with s.get(self.base_url + path, headers=headers) as resp:
                return await _read_json_response(resp, path)
        except (aiohttp.ClientConnectionError, TimeoutError) as exc:
            return _connection_error_payload(path, exc)

    async def _post(self, path: str, data: dict[str, Any] | None = None) -> dict[str, Any]:
        s = self._require_session()
        headers: dict[str, str] = {"Content-Type": "application/json"}
        if self._api_key:
            headers["X-API-Key"] = self._api_key
        try:
            async with s.post(
                self.base_url + path,
                data=json.dumps(data or {}).encode("utf-8"),
                headers=headers,
            ) as resp:
                return await _read_json_response(resp, path)
        except (aiohttp.ClientConnectionError, TimeoutError) as exc:
            return _connection_error_payload(path, exc)

    def _require_session(self) -> aiohttp.ClientSession:
        if self._session is None:
            raise RuntimeError(
                "AsyncLingTuClient session is not open. "
                "Use `async with AsyncLingTuClient(...) as robot:` or "
                "call `await robot.__aenter__()` first."
            )
        return self._session


# ---------------------------------------------------------------------------
# asyncio sleep helper; Python 3.13+ asyncio.sleep is fine, just an alias.
# ---------------------------------------------------------------------------
asyncio_sleep = asyncio.sleep


async def _read_json_response(resp: aiohttp.ClientResponse, path: str) -> dict[str, Any]:
    """Parse an async response without classifying server payloads as offline."""

    try:
        return await resp.json()
    except (aiohttp.ContentTypeError, json.JSONDecodeError, ValueError) as exc:
        try:
            detail = await resp.text()
        except Exception:
            detail = str(exc)
        return {
            "ok": False,
            "success": False,
            "error": f"HTTP {getattr(resp, 'status', 'unknown')}",
            "detail": detail,
            "path": path,
        }
