"""Asynchronous adapter for the synchronous LingTu Gateway client."""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any

from lingtu.sdk.client import (
    CommandResult,
    HealthStatus,
    LingTuClient,
    MapList,
    NavigationStatus,
    Pose2D,
    Position,
    RobotState,
    SessionInfo,
)


class AsyncLocalizationClient:
    """Non-blocking adapter for the synchronous localization facade."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

    async def relocalize(
        self,
        map_name: str,
        *,
        initial_pose: Pose2D,
        request_id: str | None = None,
    ) -> CommandResult:
        """Relocalize from a caller-provided pose seed."""
        return await asyncio.to_thread(
            self._client.localization.relocalize,
            map_name,
            initial_pose=initial_pose,
            request_id=request_id,
        )

    async def global_relocalize(
        self,
        map_name: str,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Relocalize without a pose seed."""
        return await asyncio.to_thread(
            self._client.localization.global_relocalize,
            map_name,
            request_id=request_id,
        )

    async def start_map_tracking(
        self,
        map_name: str,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Start continuous alignment against the active map."""
        return await asyncio.to_thread(
            self._client.localization.start_map_tracking,
            map_name,
            request_id=request_id,
        )


class AsyncLingTuClient:
    """Expose the canonical synchronous SDK without blocking the event loop."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5050,
        api_key: str | None = None,
        timeout: float = 10.0,
    ) -> None:
        self._client = LingTuClient(host, port, api_key=api_key, timeout=timeout)

    @property
    def localization(self) -> AsyncLocalizationClient:
        """Access localization operations without exposing backend names."""
        return AsyncLocalizationClient(self._client)

    async def __aenter__(self) -> AsyncLingTuClient:
        return self

    async def __aexit__(self, *_args: object) -> None:
        await self.close()

    async def close(self) -> None:
        """Close resources owned by the canonical client."""
        await asyncio.to_thread(self._client.close)

    async def go(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Navigate to map coordinates."""
        return await asyncio.to_thread(
            self._client.go,
            x,
            y,
            yaw,
            request_id=request_id,
        )

    async def go_to(self, target: str) -> CommandResult:
        """Navigate by semantic target."""
        return await asyncio.to_thread(self._client.go_to, target)

    async def follow_person(self, target_id: str) -> CommandResult:
        """Follow the person currently published with ``target_id``."""
        return await asyncio.to_thread(self._client.follow_person, target_id)

    async def stop_following(self) -> CommandResult:
        """Stop the active person-following task."""
        return await asyncio.to_thread(self._client.stop_following)

    async def stop(self) -> CommandResult:
        """Stop robot motion."""
        return await asyncio.to_thread(self._client.stop)

    async def cancel(self, reason: str = "client_cancel") -> CommandResult:
        """Cancel the current navigation mission."""
        return await asyncio.to_thread(self._client.cancel, reason)

    async def navigate_click(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
    ) -> CommandResult:
        """Navigate to a map-viewer click point."""
        return await asyncio.to_thread(self._client.navigate_click, x, y, yaw)

    async def batch_go(
        self,
        waypoints: list[tuple[float, float, float]],
    ) -> list[CommandResult]:
        """Navigate through a sequence of waypoints."""
        return await asyncio.to_thread(self._client.batch_go, waypoints)

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
        """Wait until the current navigation mission completes."""
        return await asyncio.to_thread(
            self._client.wait_until_arrived,
            timeout,
            poll_interval,
            distance_threshold,
            request_id=request_id,
            expected_goal=expected_goal,
            baseline=baseline,
        )

    async def state(self) -> RobotState:
        """Return the full robot state."""
        return await asyncio.to_thread(self._client.state)

    async def health(self) -> HealthStatus:
        """Return system health."""
        return await asyncio.to_thread(self._client.health)

    async def position(self) -> Position:
        """Return the current map-frame position."""
        return await asyncio.to_thread(self._client.position)

    async def session(self) -> SessionInfo:
        """Return the current session state."""
        return await asyncio.to_thread(self._client.session)

    async def navigation_status(self) -> NavigationStatus:
        """Return the current navigation status."""
        return await asyncio.to_thread(self._client.navigation_status)

    async def localization_status(self) -> dict[str, Any]:
        """Return localization health."""
        return await asyncio.to_thread(self._client.localization_status)

    async def path(self) -> dict[str, Any]:
        """Return the latest planned path."""
        return await asyncio.to_thread(self._client.path)

    async def maps(self) -> MapList:
        """List saved maps."""
        return await asyncio.to_thread(self._client.maps)

    async def save_map(
        self,
        name: str | None = None,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Submit a durable map-save operation."""
        return await asyncio.to_thread(
            self._client.save_map,
            name,
            request_id=request_id,
        )

    async def save_map_and_wait(
        self,
        name: str | None = None,
        *,
        request_id: str | None = None,
        timeout: float = 300.0,
        poll_interval: float = 1.0,
    ) -> dict[str, Any]:
        """Submit a map save and wait for its terminal result."""
        return await asyncio.to_thread(
            self._client.save_map_and_wait,
            name,
            request_id=request_id,
            timeout=timeout,
            poll_interval=poll_interval,
        )

    async def get_map_operation(self, operation_id: str) -> dict[str, Any]:
        """Return one map-save operation."""
        return await asyncio.to_thread(self._client.get_map_operation, operation_id)

    async def cancel_map_operation(self, operation_id: str) -> CommandResult:
        """Cancel one map-save operation."""
        return await asyncio.to_thread(self._client.cancel_map_operation, operation_id)

    async def retry_map_operation(self, operation_id: str) -> CommandResult:
        """Retry one map-save operation."""
        return await asyncio.to_thread(self._client.retry_map_operation, operation_id)

    async def download_map_pcd(self, name: str, target: str | Path) -> Path:
        """Download a saved point cloud."""
        return await asyncio.to_thread(self._client.download_map_pcd, name, target)

    async def rename_map(self, old_name: str, new_name: str) -> CommandResult:
        """Rename a saved map."""
        return await asyncio.to_thread(self._client.rename_map, old_name, new_name)

    async def reset_map_cloud(self) -> CommandResult:
        """Clear the live visualization cloud."""
        return await asyncio.to_thread(self._client.reset_map_cloud)

    async def map_points(self) -> dict[str, Any]:
        """Return live map points."""
        return await asyncio.to_thread(self._client.map_points)

    async def saved_map_points(self, name: str) -> dict[str, Any]:
        """Return points from a saved map."""
        return await asyncio.to_thread(self._client.saved_map_points, name)

    async def set_mode(self, mode: str) -> CommandResult:
        """Set robot operating mode."""
        return await asyncio.to_thread(self._client.set_mode, mode)

    async def scene(self) -> dict[str, Any]:
        """Return the current scene graph."""
        return await asyncio.to_thread(self._client.scene)

    async def locations(self) -> dict[str, Any]:
        """List tagged locations."""
        return await asyncio.to_thread(self._client.locations)

    async def tag_location(
        self,
        name: str,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        use_current_pose: bool = False,
    ) -> CommandResult:
        """Create or update a tagged location."""
        return await asyncio.to_thread(
            self._client.tag_location,
            name,
            x,
            y,
            yaw,
            use_current_pose,
        )

    async def delete_location(self, name: str) -> CommandResult:
        """Delete a tagged location."""
        return await asyncio.to_thread(self._client.delete_location, name)

    async def camera_snapshot(self) -> bytes:
        """Return the latest camera image."""
        return await asyncio.to_thread(self._client.camera_snapshot)

    async def slam_status(self) -> dict[str, Any]:
        """Return SLAM status."""
        return await asyncio.to_thread(self._client.slam_status)

    async def explore_start(self) -> CommandResult:
        """Start frontier exploration."""
        return await asyncio.to_thread(self._client.explore_start)

    async def explore_stop(self) -> CommandResult:
        """Stop frontier exploration."""
        return await asyncio.to_thread(self._client.explore_stop)

    async def explore_status(self) -> dict[str, Any]:
        """Return exploration status."""
        return await asyncio.to_thread(self._client.explore_status)

    async def recording_start(
        self,
        prefix: str | None = None,
        *,
        duration: int = 600,
        capture_profile: str | None = None,
        task_id: str | None = None,
        camera: bool | None = None,
        minimum_free_gib: int | None = None,
    ) -> CommandResult:
        """Start a native recording."""
        return await asyncio.to_thread(
            self._client.recording_start,
            prefix,
            duration=duration,
            capture_profile=capture_profile,
            task_id=task_id,
            camera=camera,
            minimum_free_gib=minimum_free_gib,
        )

    async def recording_stop(self) -> CommandResult:
        """Stop the active recording."""
        return await asyncio.to_thread(self._client.recording_stop)

    async def recording_status(self) -> dict[str, Any]:
        """Return recording status."""
        return await asyncio.to_thread(self._client.recording_status)

    async def memory_temporal(self) -> dict[str, Any]:
        """Return temporal observations."""
        return await asyncio.to_thread(self._client.memory_temporal)

    async def memory_temporal_semantic(self, query: str) -> dict[str, Any]:
        """Search temporal observations."""
        return await asyncio.to_thread(self._client.memory_temporal_semantic, query)

    async def acquire_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Acquire the control lease."""
        return await asyncio.to_thread(self._client.acquire_lease, client_id, ttl)

    async def release_lease(self, client_id: str) -> CommandResult:
        """Release the control lease."""
        return await asyncio.to_thread(self._client.release_lease, client_id)

    async def renew_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Renew the control lease."""
        return await asyncio.to_thread(self._client.renew_lease, client_id, ttl)

    async def capabilities(self) -> dict[str, Any]:
        """Return the API capability manifest."""
        return await asyncio.to_thread(self._client.capabilities)

    async def bootstrap(self) -> dict[str, Any]:
        """Return the application bootstrap snapshot."""
        return await asyncio.to_thread(self._client.bootstrap)

    async def readiness(self) -> dict[str, Any]:
        """Return client readiness."""
        return await asyncio.to_thread(self._client.readiness)

    async def field_check(self) -> CommandResult:
        """Run the read-only field check."""
        return await asyncio.to_thread(self._client.field_check)

    async def runtime_contract(self) -> dict[str, Any]:
        """Return the runtime interface contract."""
        return await asyncio.to_thread(self._client.runtime_contract)

    async def auth_login(self, api_key: str) -> dict[str, Any]:
        """Authenticate with an API key."""
        return await asyncio.to_thread(self._client.auth_login, api_key)

    async def auth_check(self) -> dict[str, Any]:
        """Return authentication requirements."""
        return await asyncio.to_thread(self._client.auth_check)
