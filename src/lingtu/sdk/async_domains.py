"""Typed asynchronous adapters for the canonical synchronous domain clients."""

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


class AsyncNavigationClient:
    """Run navigation and localization without blocking the event loop."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

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
            self._client.navigation.go,
            x,
            y,
            yaw,
            request_id=request_id,
        )

    async def go_to_click(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
    ) -> CommandResult:
        """Navigate to a map-viewer click point."""
        return await asyncio.to_thread(self._client.navigation.go_to_click, x, y, yaw)

    async def cancel(self, reason: str = "client_cancel") -> CommandResult:
        """Cancel the current navigation mission."""
        return await asyncio.to_thread(self._client.navigation.cancel, reason)

    async def wait_until_arrived(
        self,
        task_id: str | None = None,
        timeout: float = 120.0,
        poll_interval: float = 0.5,
    ) -> NavigationStatus:
        """Wait until an identified or currently observed navigation task completes."""
        return await asyncio.to_thread(
            self._client.navigation.wait_until_arrived,
            task_id,
            timeout,
            poll_interval,
        )

    async def status(self) -> NavigationStatus:
        """Return the current navigation status."""
        return await asyncio.to_thread(self._client.navigation.status)

    async def task(self, task_id: str) -> dict[str, Any]:
        """Return lifecycle evidence for one stable navigation task identity."""
        return await asyncio.to_thread(self._client.navigation.task, task_id)

    async def path(self) -> dict[str, Any]:
        """Return the latest planned path."""
        return await asyncio.to_thread(self._client.navigation.path)

    async def start_exploration(self) -> CommandResult:
        """Start frontier exploration."""
        return await asyncio.to_thread(self._client.navigation.start_exploration)

    async def stop_exploration(self) -> CommandResult:
        """Stop frontier exploration."""
        return await asyncio.to_thread(self._client.navigation.stop_exploration)

    async def exploration_status(self) -> dict[str, Any]:
        """Return exploration status."""
        return await asyncio.to_thread(self._client.navigation.exploration_status)

    async def relocalize(
        self,
        map_name: str,
        *,
        initial_pose: Pose2D,
        request_id: str | None = None,
    ) -> CommandResult:
        """Relocalize from a caller-provided pose seed."""
        return await asyncio.to_thread(
            self._client.navigation.relocalize,
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
            self._client.navigation.global_relocalize,
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
            self._client.navigation.start_map_tracking,
            map_name,
            request_id=request_id,
        )

    async def localization_status(self) -> dict[str, Any]:
        """Return localization health."""
        return await asyncio.to_thread(self._client.navigation.localization_status)

    async def position(self) -> Position:
        """Return the current map-frame position."""
        return await asyncio.to_thread(self._client.navigation.position)


class AsyncMapClient:
    """Run saved-map operations without blocking the event loop."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

    async def list(self) -> MapList:
        """List saved maps."""
        return await asyncio.to_thread(self._client.map.list)

    async def save(
        self,
        name: str | None = None,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Submit a durable map-save operation."""
        return await asyncio.to_thread(
            self._client.map.save,
            name,
            request_id=request_id,
        )

    async def save_and_wait(
        self,
        name: str | None = None,
        *,
        request_id: str | None = None,
        timeout: float = 300.0,
        poll_interval: float = 1.0,
    ) -> dict[str, Any]:
        """Submit a map save and wait for its terminal result."""
        return await asyncio.to_thread(
            self._client.map.save_and_wait,
            name,
            request_id=request_id,
            timeout=timeout,
            poll_interval=poll_interval,
        )

    async def operation(self, operation_id: str) -> dict[str, Any]:
        """Return one map-save operation."""
        return await asyncio.to_thread(self._client.map.operation, operation_id)

    async def cancel_operation(self, operation_id: str) -> CommandResult:
        """Cancel one map-save operation."""
        return await asyncio.to_thread(self._client.map.cancel_operation, operation_id)

    async def retry_operation(self, operation_id: str) -> CommandResult:
        """Retry one map-save operation."""
        return await asyncio.to_thread(self._client.map.retry_operation, operation_id)

    async def download_pcd(self, name: str, target: str | Path) -> Path:
        """Download a saved point cloud."""
        return await asyncio.to_thread(self._client.map.download_pcd, name, target)

    async def rename(self, old_name: str, new_name: str) -> CommandResult:
        """Rename a saved map."""
        return await asyncio.to_thread(self._client.map.rename, old_name, new_name)

    async def reset_cloud(self) -> CommandResult:
        """Clear the live visualization cloud."""
        return await asyncio.to_thread(self._client.map.reset_cloud)

    async def points(self) -> dict[str, Any]:
        """Return live map points."""
        return await asyncio.to_thread(self._client.map.points)

    async def saved_points(self, name: str) -> dict[str, Any]:
        """Return points from a saved map."""
        return await asyncio.to_thread(self._client.map.saved_points, name)

    async def locations(self) -> dict[str, Any]:
        """List tagged locations."""
        return await asyncio.to_thread(self._client.map.locations)

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
            self._client.map.tag_location,
            name,
            x,
            y,
            yaw,
            use_current_pose,
        )

    async def delete_location(self, name: str) -> CommandResult:
        """Delete a tagged location."""
        return await asyncio.to_thread(self._client.map.delete_location, name)


class AsyncTaskClient:
    """Run instructions, follow and inspection without blocking the event loop."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

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
            self._client.tasks.send_instruction,
            text,
            run_id=run_id,
            control_only=control_only,
            request_id=request_id,
            client_id=client_id,
        )

    async def get_agent_run(self, run_id: str) -> dict[str, Any]:
        """Read the referenced run without inferring a terminal state."""
        return await asyncio.to_thread(self._client.tasks.get_agent_run, run_id)

    async def cancel_agent_run(
        self,
        run_id: str,
        *,
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Request cancellation of only the referenced Agent run."""
        return await asyncio.to_thread(
            self._client.tasks.cancel_agent_run,
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
            self._client.tasks.start_follow,
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
            self._client.tasks.get_follow_task,
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
            self._client.tasks.cancel_follow_task,
            follow_task_id,
            product_session_id,
            reason=reason,
            request_id=request_id,
            client_id=client_id,
        )

    async def inspection_routes(self, map_id: str | None = None) -> dict[str, Any]:
        """List stored inspection routes, optionally for one map."""
        return await asyncio.to_thread(self._client.tasks.inspection_routes, map_id)

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
            self._client.tasks.start_inspection,
            route_id,
            map_id=map_id,
            revision=revision,
            request_id=request_id,
        )

    async def inspection_task(self, task_id: str) -> dict[str, Any]:
        """Return the fact-backed state of one inspection task."""
        return await asyncio.to_thread(self._client.tasks.inspection_task, task_id)

    async def pause_inspection(
        self,
        task_id: str,
        *,
        reason: str = "operator_pause",
        request_id: str | None = None,
    ) -> CommandResult:
        """Request a pause for one inspection task."""
        return await asyncio.to_thread(
            self._client.tasks.pause_inspection,
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
            self._client.tasks.resume_inspection,
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
            self._client.tasks.cancel_inspection,
            task_id,
            reason=reason,
            request_id=request_id,
        )

    async def inspection_report(self, task_id: str) -> dict[str, Any]:
        """Return the business report for one inspection task."""
        return await asyncio.to_thread(self._client.tasks.inspection_report, task_id)


class AsyncObservationClient:
    """Run observation queries and recording without blocking the event loop."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

    async def scene(self) -> dict[str, Any]:
        """Return the current scene graph."""
        return await asyncio.to_thread(self._client.observations.scene)

    async def camera_snapshot(self) -> bytes:
        """Return the latest camera image."""
        return await asyncio.to_thread(self._client.observations.camera_snapshot)

    async def temporal_memory(self) -> dict[str, Any]:
        """Return temporal observations."""
        return await asyncio.to_thread(self._client.observations.temporal_memory)

    async def search_memory(self, query: str) -> dict[str, Any]:
        """Search temporal observations."""
        return await asyncio.to_thread(self._client.observations.search_memory, query)

    async def start_recording(
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
            self._client.observations.start_recording,
            prefix,
            duration=duration,
            capture_profile=capture_profile,
            task_id=task_id,
            camera=camera,
            minimum_free_gib=minimum_free_gib,
        )

    async def stop_recording(self) -> CommandResult:
        """Stop the active recording."""
        return await asyncio.to_thread(self._client.observations.stop_recording)

    async def recording_status(self) -> dict[str, Any]:
        """Return recording status."""
        return await asyncio.to_thread(self._client.observations.recording_status)


class AsyncSystemClient:
    """Run system diagnostics and operator control without blocking the event loop."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

    async def state(self) -> RobotState:
        """Return the full robot state."""
        return await asyncio.to_thread(self._client.system.state)

    async def health(self) -> HealthStatus:
        """Return system health."""
        return await asyncio.to_thread(self._client.system.health)

    async def session(self) -> SessionInfo:
        """Return the current session state."""
        return await asyncio.to_thread(self._client.system.session)

    async def set_mode(self, mode: str) -> CommandResult:
        """Set robot operating mode."""
        return await asyncio.to_thread(self._client.system.set_mode, mode)

    async def capabilities(self) -> dict[str, Any]:
        """Return the API capability manifest."""
        return await asyncio.to_thread(self._client.system.capabilities)

    async def bootstrap(self) -> dict[str, Any]:
        """Return the application bootstrap snapshot."""
        return await asyncio.to_thread(self._client.system.bootstrap)

    async def readiness(self) -> dict[str, Any]:
        """Return client readiness."""
        return await asyncio.to_thread(self._client.system.readiness)

    async def field_check(self) -> CommandResult:
        """Run the read-only field check."""
        return await asyncio.to_thread(self._client.system.field_check)

    async def runtime_contract(self) -> dict[str, Any]:
        """Return the runtime interface contract."""
        return await asyncio.to_thread(self._client.system.runtime_contract)

    async def slam_status(self) -> dict[str, Any]:
        """Return SLAM status."""
        return await asyncio.to_thread(self._client.system.slam_status)

    async def emergency_stop(self) -> CommandResult:
        """Stop robot motion."""
        return await asyncio.to_thread(self._client.system.emergency_stop)

    async def acquire_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Acquire the control lease."""
        return await asyncio.to_thread(self._client.system.acquire_lease, client_id, ttl)

    async def release_lease(self, client_id: str) -> CommandResult:
        """Release the control lease."""
        return await asyncio.to_thread(self._client.system.release_lease, client_id)

    async def renew_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Renew the control lease."""
        return await asyncio.to_thread(self._client.system.renew_lease, client_id, ttl)


class AsyncLocalizationClient:
    """Compatibility entry for the original localization commands."""

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
