"""Asynchronous adapter for the synchronous LingTu Gateway client."""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any

from lingtu.sdk.async_domains import (
    AsyncLocalizationClient,
    AsyncMapClient,
    AsyncNavigationClient,
    AsyncObservationClient,
    AsyncSystemClient,
    AsyncTaskClient,
)
from lingtu.sdk.client import (
    CommandResult,
    HealthStatus,
    LingTuClient,
    MapList,
    NavigationStatus,
    Position,
    RobotState,
    SessionInfo,
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
    def navigation(self) -> AsyncNavigationClient:
        """Navigation, localization and exploration."""
        return AsyncNavigationClient(self._client)

    @property
    def map(self) -> AsyncMapClient:
        """Saved maps, artifacts and named places."""
        return AsyncMapClient(self._client)

    @property
    def tasks(self) -> AsyncTaskClient:
        """Instructions, follow and inspection tasks."""
        return AsyncTaskClient(self._client)

    @property
    def observations(self) -> AsyncObservationClient:
        """Scene, camera, history and native recording."""
        return AsyncObservationClient(self._client)

    @property
    def system(self) -> AsyncSystemClient:
        """System status, diagnostics and operator control."""
        return AsyncSystemClient(self._client)

    @property
    def localization(self) -> AsyncLocalizationClient:
        """Compatibility entry for the original localization commands."""
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

    async def send_instruction(
        self,
        text: str,
        *,
        run_id: str | None = None,
        control_only: bool = False,
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Submit text or scoped controls through the synchronous HTTP client."""
        return await asyncio.to_thread(
            self._client.send_instruction,
            text,
            run_id=run_id,
            control_only=control_only,
            request_id=request_id,
            client_id=client_id,
        )

    async def get_agent_run(self, run_id: str) -> dict[str, Any]:
        """Read the referenced run without inferring a terminal state."""
        return await asyncio.to_thread(self._client.get_agent_run, run_id)

    async def cancel_agent_run(
        self,
        run_id: str,
        *,
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Request cancellation of only the referenced Agent run."""
        return await asyncio.to_thread(
            self._client.cancel_agent_run,
            run_id,
            request_id=request_id,
            client_id=client_id,
        )

    async def start_follow(
        self,
        *,
        description: str | None = None,
        target_id: str | None = None,
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Start one owned Follow execution."""
        return await asyncio.to_thread(
            self._client.start_follow,
            description=description,
            target_id=target_id,
            request_id=request_id,
            client_id=client_id,
        )

    async def get_follow_task(
        self,
        follow_task_id: str,
        product_session_id: str,
    ) -> dict[str, Any]:
        """Read one Follow execution using its stable owner identity."""
        return await asyncio.to_thread(
            self._client.get_follow_task,
            follow_task_id,
            product_session_id,
        )

    async def cancel_follow_task(
        self,
        follow_task_id: str,
        product_session_id: str,
        *,
        reason: str = "operator_cancel",
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Cancel exactly one Follow in its original Product run."""
        return await asyncio.to_thread(
            self._client.cancel_follow_task,
            follow_task_id,
            product_session_id,
            reason=reason,
            request_id=request_id,
            client_id=client_id,
        )

    async def follow_person(
        self,
        target_id: str,
        *,
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Start owned Follow for the published person ``target_id``."""
        return await asyncio.to_thread(
            self._client.follow_person,
            target_id,
            request_id=request_id,
            client_id=client_id,
        )

    async def stop_following(
        self,
        follow_task_id: str,
        product_session_id: str,
        *,
        reason: str = "operator_cancel",
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Cancel one referenced Follow execution."""
        return await asyncio.to_thread(
            self._client.stop_following,
            follow_task_id,
            product_session_id,
            reason=reason,
            request_id=request_id,
            client_id=client_id,
        )

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

    async def wait_until_arrived(
        self,
        task_id: str | None = None,
        timeout: float = 120.0,
        poll_interval: float = 0.5,
    ) -> NavigationStatus:
        """Wait until an identified or currently observed navigation task completes."""
        return await asyncio.to_thread(
            self._client.wait_until_arrived,
            task_id,
            timeout,
            poll_interval,
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

    async def navigation_task_status(self, task_id: str) -> dict[str, Any]:
        """Return lifecycle evidence for one stable navigation task identity."""
        return await asyncio.to_thread(self._client.navigation_task_status, task_id)

    async def localization_status(self) -> dict[str, Any]:
        """Return localization health."""
        return await asyncio.to_thread(self._client.localization_status)

    async def path(self) -> dict[str, Any]:
        """Return the latest planned path."""
        return await asyncio.to_thread(self._client.path)

    async def inspection_routes(self, map_id: str | None = None) -> dict[str, Any]:
        """List stored inspection routes, optionally for one map."""
        return await asyncio.to_thread(self._client.inspection_routes, map_id)

    async def start_inspection(
        self,
        route_id: str,
        *,
        map_id: str | None = None,
        revision: int = 0,
        request_id: str | None = None,
    ) -> CommandResult:
        """Submit a stored route for native inspection execution."""
        return await asyncio.to_thread(
            self._client.start_inspection,
            route_id,
            map_id=map_id,
            revision=revision,
            request_id=request_id,
        )

    async def inspection_task(self, task_id: str) -> dict[str, Any]:
        """Return the fact-backed state of one inspection task."""
        return await asyncio.to_thread(self._client.inspection_task, task_id)

    async def pause_inspection(
        self,
        task_id: str,
        *,
        reason: str = "operator_pause",
        request_id: str | None = None,
    ) -> CommandResult:
        """Request a pause for one inspection task."""
        return await asyncio.to_thread(
            self._client.pause_inspection,
            task_id,
            reason=reason,
            request_id=request_id,
        )

    async def resume_inspection(
        self,
        task_id: str,
        *,
        reason: str = "operator_resume",
        request_id: str | None = None,
    ) -> CommandResult:
        """Request resumption of one paused inspection task."""
        return await asyncio.to_thread(
            self._client.resume_inspection,
            task_id,
            reason=reason,
            request_id=request_id,
        )

    async def cancel_inspection(
        self,
        task_id: str,
        *,
        reason: str = "operator_cancel",
        request_id: str | None = None,
    ) -> CommandResult:
        """Request cancellation of one inspection task."""
        return await asyncio.to_thread(
            self._client.cancel_inspection,
            task_id,
            reason=reason,
            request_id=request_id,
        )

    async def inspection_report(self, task_id: str) -> dict[str, Any]:
        """Return the business report for one inspection task."""
        return await asyncio.to_thread(self._client.inspection_report, task_id)

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
