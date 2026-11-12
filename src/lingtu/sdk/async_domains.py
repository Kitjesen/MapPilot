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
        self, x: float, y: float, yaw: float | None = None, *,
        z: float = 0.0, task_id: str | None = None, request_id: str | None = None,
        acceptance_radius_m: float | None = None, max_speed_mps: float | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Delegate to the navigation domain."""
        return await asyncio.to_thread(self._client.navigation.go, x, y, yaw, z=z, task_id=task_id, request_id=request_id,
            acceptance_radius_m=acceptance_radius_m, max_speed_mps=max_speed_mps,
            client_id=client_id)

    async def go_to_click(
        self, x: float, y: float, yaw: float | None = None, *,
        z: float = 0.0, task_id: str | None = None, request_id: str | None = None,
        acceptance_radius_m: float | None = None, max_speed_mps: float | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Delegate to the navigation domain."""
        return await asyncio.to_thread(self._client.navigation.go_to_click, x, y, yaw, z=z, task_id=task_id, request_id=request_id,
            acceptance_radius_m=acceptance_radius_m, max_speed_mps=max_speed_mps,
            client_id=client_id)

    async def select_planner(
        self, name: str, *, request_id: str | None = None, client_id: str = "sdk",
    ) -> CommandResult:
        """Select the local planner through the synchronous navigation contract."""
        return await asyncio.to_thread(
            self._client.navigation.select_planner, name, request_id=request_id, client_id=client_id,
        )

    async def cancel(
        self, reason: str = "client_cancel", *, task_id: str | None = None,
        request_id: str | None = None, client_id: str = "sdk",
    ) -> CommandResult:
        """Cancel without issuing an emergency stop."""
        return await asyncio.to_thread(self._client.navigation.cancel, reason, task_id=task_id, request_id=request_id, client_id=client_id)

    async def pause(
        self, task_id: str, *, reason: str = "operator_pause",
        request_id: str | None = None, client_id: str = "sdk",
    ) -> CommandResult:
        """Delegate to the synchronous navigation contract."""
        return await asyncio.to_thread(
            self._client.navigation.pause, task_id, reason=reason,
            request_id=request_id, client_id=client_id,
        )

    async def resume(
        self, task_id: str, *, reason: str = "operator_resume",
        request_id: str | None = None, client_id: str = "sdk",
    ) -> CommandResult:
        """Delegate to the synchronous navigation contract."""
        return await asyncio.to_thread(
            self._client.navigation.resume, task_id, reason=reason,
            request_id=request_id, client_id=client_id,
        )

    async def set_speed_limit(
        self, task_id: str, max_speed_mps: float, *,
        request_id: str | None = None, client_id: str = "sdk",
    ) -> CommandResult:
        """Delegate to the synchronous navigation contract."""
        return await asyncio.to_thread(
            self._client.navigation.set_speed_limit, task_id, max_speed_mps,
            request_id=request_id, client_id=client_id,
        )

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

    async def goal_status(self, request_id: str) -> dict[str, Any]:
        """Reconcile a goal submission using its caller-selected request identity."""
        return await asyncio.to_thread(self._client.navigation.goal_status, request_id)

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
        auto_process: bool = True,
    ) -> CommandResult:
        """Submit a durable map-save operation."""
        return await asyncio.to_thread(
            self._client.map.save,
            name,
            request_id=request_id,
            auto_process=auto_process,
        )

    async def save_and_wait(
        self,
        name: str | None = None,
        *,
        request_id: str | None = None,
        timeout: float = 300.0,
        poll_interval: float = 1.0,
        auto_process: bool = True,
        wait_for_source: bool = False,
    ) -> dict[str, Any]:
        """Submit a map save and wait for its terminal result."""
        return await asyncio.to_thread(
            self._client.map.save_and_wait,
            name,
            request_id=request_id,
            timeout=timeout,
            poll_interval=poll_interval,
            auto_process=auto_process,
            wait_for_source=wait_for_source,
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

    async def operations(self, limit: int = 100) -> dict[str, Any]:
        """List durable save operations, including archived sources."""
        return await asyncio.to_thread(self._client.map.operations, limit)

    async def reoptimize_operation(self, operation_id: str, *, request_id: str | None = None) -> CommandResult:
        """Realign this operation's archived scans and rebuild its navigation map."""
        return await asyncio.to_thread(self._client.map.reoptimize_operation, operation_id, request_id=request_id)

    async def process_operation(self, operation_id: str) -> CommandResult:
        """Process an existing saved source without requesting another SLAM snapshot."""
        return await asyncio.to_thread(self._client.map.process_operation, operation_id)

    async def download_source(self, operation_id: str, target: str | Path) -> Path:
        """Stream this operation's archived source PCD, including after processing fails."""
        return await asyncio.to_thread(self._client.map.download_source, operation_id, target)

    async def download_octomap(self, name: str, target: str | Path) -> Path:
        """Stream the saved OctoMap artifact to ``target`` atomically."""
        return await asyncio.to_thread(self._client.map.download_octomap, name, target)

    async def surface(
        self,
        name: str,
        *,
        headroom_m: float | None = None,
        radius_m: float | None = None,
        max_hole_area_m2: float | None = None,
        max_slope: float | None = None,
    ) -> dict[str, Any]:
        """Preview observed and inferred support surfaces using native geometry."""
        return await asyncio.to_thread(
            self._client.map.surface,
            name,
            headroom_m=headroom_m,
            radius_m=radius_m,
            max_hole_area_m2=max_hole_area_m2,
            max_slope=max_slope,
        )

    async def no_go(self, name: str) -> dict[str, Any]:
        """Read independent 3D no-go regions and their content epoch."""
        return await asyncio.to_thread(self._client.map.no_go, name)

    async def set_no_go(self, name: str, region: dict[str, Any], *, expected_content_epoch: int) -> CommandResult:
        """Add or replace a region with ``id``, ``min`` and ``max`` XYZ bounds."""
        return await asyncio.to_thread(
            self._client.map.set_no_go, name, region, expected_content_epoch=expected_content_epoch
        )

    async def delete_no_go(self, name: str, region_id: str, *, expected_content_epoch: int) -> CommandResult:
        """Delete one no-go region against the version returned by ``no_go``."""
        return await asyncio.to_thread(
            self._client.map.delete_no_go, name, region_id, expected_content_epoch=expected_content_epoch
        )

    async def preview_edits(
        self, name: str, operations: list[dict[str, Any]], *, expected_source_content_epoch: int
    ) -> dict[str, Any]:
        """Preview native point-cloud edit operations against a saved source version."""
        return await asyncio.to_thread(
            self._client.map.preview_edits, name, operations, expected_source_content_epoch=expected_source_content_epoch
        )

    async def save_as(
        self, name: str, new_name: str, operations: list[dict[str, Any]], *, expected_source_content_epoch: int
    ) -> CommandResult:
        """Save edited source as another map. Pass an empty operation list to copy."""
        return await asyncio.to_thread(
            self._client.map.save_as,
            name,
            new_name,
            operations,
            expected_source_content_epoch=expected_source_content_epoch,
        )

    async def crop(
        self, name: str, bounds: dict[str, Any], *, invert: bool = False, voxel_size: float = 0.0
    ) -> CommandResult:
        """Crop the saved source in place, invalidating derived navigation artifacts."""
        return await asyncio.to_thread(self._client.map.crop, name, bounds, invert=invert, voxel_size=voxel_size)

    async def edit_voxels(
        self, name: str, x: float, y: float, z: float, *, state: str, radius_m: float = 0.2, shape: str = "sphere"
    ) -> CommandResult:
        """Edit saved OctoMap occupancy. State is occupied, free, preblocked or traversable."""
        return await asyncio.to_thread(
            self._client.map.edit_voxels, name, x, y, z, state=state, radius_m=radius_m, shape=shape
        )

    async def voxel_edits(self, name: str) -> dict[str, Any]:
        """Read the saved OctoMap edit overlay."""
        return await asyncio.to_thread(self._client.map.voxel_edits, name)

    async def mark_zone(
        self, name: str, x: float, y: float, z: float, *, state: str,
        radius_m: float = 0.2, shape: str = "sphere",
    ) -> CommandResult:
        """Mark a saved OctoMap zone through native mapd."""
        return await asyncio.to_thread(
            self._client.map.mark_zone, name, x, y, z, state=state, radius_m=radius_m, shape=shape,
        )

    async def build_occupancy(self, name: str) -> CommandResult:
        """Build a 2D occupancy artifact from the saved source."""
        return await asyncio.to_thread(self._client.map.build_occupancy, name)

    async def validate_plan(
        self, name: str, x: float, y: float, *, z: float = 0.0,
        frame_id: str = "map", acceptance_radius_m: float | None = None,
    ) -> dict[str, Any]:
        """Preview a route on the active saved map without publishing a motion goal."""
        return await asyncio.to_thread(
            self._client.map.validate_plan, name, x, y, z=z,
            frame_id=frame_id, acceptance_radius_m=acceptance_radius_m,
        )

    async def build_octomap(
        self,
        name: str,
        *,
        resolution: float | None = None,
        frame_id: str | None = None,
        source_profile: str | None = None,
        data_source: str | None = None,
        slam_source: str | None = None,
        localization_source: str | None = None,
        mapping_source: str | None = None,
    ) -> CommandResult:
        """Rebuild the saved source's OctoMap using native mapd."""
        return await asyncio.to_thread(
            self._client.map.build_octomap,
            name,
            resolution=resolution,
            frame_id=frame_id,
            source_profile=source_profile,
            data_source=data_source,
            slam_source=slam_source,
            localization_source=localization_source,
            mapping_source=mapping_source,
        )

    async def import_pcd(
        self, name: str, source_path: str, *, voxel_size: float = 0.0, bounds: dict[str, Any] | None = None
    ) -> CommandResult:
        """Import a PCD already in the Gateway's map exchange directory."""
        return await asyncio.to_thread(self._client.map.import_pcd, name, source_path, voxel_size=voxel_size, bounds=bounds)

    async def delete(self, name: str) -> CommandResult:
        """Delete a saved map. Native mapd rejects deletion of an active map."""
        return await asyncio.to_thread(self._client.map.delete, name)


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
        expected_product_session_id: str | None = None,
        expected_env: str | None = None,
    ) -> CommandResult:
        """Submit a stored route for native inspection execution."""
        return await asyncio.to_thread(
            self._client.tasks.start_inspection,
            route_id,
            map_id=map_id,
            revision=revision,
            request_id=request_id,
            expected_product_session_id=expected_product_session_id,
            expected_env=expected_env,
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
        expected_product_session_id: str | None = None,
        expected_env: str | None = None,
    ) -> CommandResult:
        """Request a pause for one inspection task."""
        return await asyncio.to_thread(
            self._client.tasks.pause_inspection,
            task_id,
            reason=reason,
            request_id=request_id,
            expected_product_session_id=expected_product_session_id,
            expected_env=expected_env,
        )

    async def resume_inspection(
        self,
        task_id: str,
        *,
        reason: str = "operator_resume",
        request_id: str | None = None,
        expected_product_session_id: str | None = None,
        expected_env: str | None = None,
    ) -> CommandResult:
        """Request resumption of one paused inspection task."""
        return await asyncio.to_thread(
            self._client.tasks.resume_inspection,
            task_id,
            reason=reason,
            request_id=request_id,
            expected_product_session_id=expected_product_session_id,
            expected_env=expected_env,
        )

    async def cancel_inspection(
        self,
        task_id: str,
        *,
        reason: str = "operator_cancel",
        request_id: str | None = None,
        expected_product_session_id: str | None = None,
        expected_env: str | None = None,
    ) -> CommandResult:
        """Request cancellation of one inspection task."""
        return await asyncio.to_thread(
            self._client.tasks.cancel_inspection,
            task_id,
            reason=reason,
            request_id=request_id,
            expected_product_session_id=expected_product_session_id,
            expected_env=expected_env,
        )

    async def inspection_report(self, task_id: str) -> dict[str, Any]:
        """Return the business report for one inspection task."""
        return await asyncio.to_thread(self._client.tasks.inspection_report, task_id)

    async def inspection_route(self, route_id: str, *, map_id: str | None = None) -> dict[str, Any]:
        """Read one stored native inspection route."""
        return await asyncio.to_thread(
            self._client.tasks.inspection_route, route_id, map_id=map_id,
        )

    async def save_inspection_route(self, route: dict[str, Any]) -> dict[str, Any]:
        """Create or update a route with its map epoch and revision."""
        return await asyncio.to_thread(
            self._client.tasks.save_inspection_route, route,
        )

    async def delete_inspection_route(self, route_id: str, *, map_id: str | None = None) -> dict[str, Any]:
        """Delete one stored route through the native inspection owner."""
        return await asyncio.to_thread(
            self._client.tasks.delete_inspection_route, route_id, map_id=map_id,
        )

    async def inspection_tasks(
        self, *, map_id: str | None = None, route_id: str | None = None,
        include_terminal: bool = False, limit: int = 20,
    ) -> dict[str, Any]:
        """List retained inspection task projections with server-side filters."""
        return await asyncio.to_thread(
            self._client.tasks.inspection_tasks,
            map_id=map_id,
            route_id=route_id,
            include_terminal=include_terminal,
            limit=limit,
        )

    async def list_inspection_evidence(self, *, limit: int = 20) -> dict[str, Any]:
        """List recent evidence verified by the inspection store."""
        return await asyncio.to_thread(
            self._client.tasks.list_inspection_evidence, limit=limit,
        )

    async def inspection_evidence(self, evidence_id: str) -> dict[str, Any]:
        """Read one verified evidence manifest."""
        return await asyncio.to_thread(
            self._client.tasks.inspection_evidence, evidence_id,
        )

    async def review_inspection_evidence(self, evidence_id: str, verdict: str, *, note: str = "") -> dict[str, Any]:
        """Record a human finding while retaining the original capture result."""
        return await asyncio.to_thread(
            self._client.tasks.review_inspection_evidence, evidence_id, verdict, note=note,
        )

    async def download_inspection_artifact(self, evidence_id: str, kind: str, target: str | Path) -> Path:
        """Download one verified evidence artifact by kind."""
        return await asyncio.to_thread(
            self._client.tasks.download_inspection_artifact, evidence_id, kind, target,
        )

    async def download_inspection_measurement(self, evidence_id: str, sensor_id: str, target: str | Path) -> Path:
        """Download a verified measurement using its sensor identity."""
        return await asyncio.to_thread(
            self._client.tasks.download_inspection_measurement, evidence_id, sensor_id, target,
        )


class AsyncObservationClient:
    """Run observation queries and recording without blocking the event loop."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

    async def scene(self) -> dict[str, Any]:
        """Return the current scene graph."""
        return await asyncio.to_thread(self._client.observations.scene)

    async def camera_snapshot(self, *, viewer_token: str | None = None) -> bytes:
        """Return the latest camera image."""
        return await asyncio.to_thread(self._client.observations.camera_snapshot, viewer_token=viewer_token)

    async def temporal_memory(
        self,
        *,
        label: str | None = None,
        since: str | None = None,
        near_x: float | None = None,
        near_y: float | None = None,
        radius: float | None = None,
        limit: int = 100,
    ) -> dict[str, Any]:
        """Return temporal observations with server-side filters."""
        return await asyncio.to_thread(
            self._client.observations.temporal_memory,
            label=label,
            since=since,
            near_x=near_x,
            near_y=near_y,
            radius=radius,
            limit=limit,
        )

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
        topics: list[str] | None = None,
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
            topics=topics,
        )

    async def stop_recording(self, *, expected_session_id: str | None = None) -> CommandResult:
        """Stop the current recording, optionally requiring its session identity."""
        return await asyncio.to_thread(self._client.observations.stop_recording, expected_session_id=expected_session_id)

    async def recording_status(self) -> dict[str, Any]:
        """Return recording status."""
        return await asyncio.to_thread(self._client.observations.recording_status)

    async def search_memory_embedding(
        self, embedding: list[float], *, since: str | None = None, top_k: int = 10, label: str | None = None
    ) -> dict[str, Any]:
        """Search temporal memory with a vector from the store's embedding model."""
        return await asyncio.to_thread(
            self._client.observations.search_memory_embedding, embedding, since=since, top_k=top_k, label=label
        )

    async def camera_status(self, *, viewer_token: str | None = None) -> dict[str, Any]:
        """Return the current camera status."""
        return await asyncio.to_thread(self._client.observations.camera_status, viewer_token=viewer_token)

    async def media_status(self, *, viewer_token: str | None = None) -> dict[str, Any]:
        """Return the current media status."""
        return await asyncio.to_thread(self._client.observations.media_status, viewer_token=viewer_token)

    async def camera_diagnostics(self, *, viewer_token: str | None = None) -> dict[str, Any]:
        """Return the current camera diagnostics."""
        return await asyncio.to_thread(self._client.observations.camera_diagnostics, viewer_token=viewer_token)

    async def recordings(self, *, limit: int = 100) -> dict[str, Any]:
        """List retained recording sessions and available disk space."""
        return await asyncio.to_thread(
            self._client.observations.recordings, limit=limit,
        )

    async def recording(self, session_id: str) -> dict[str, Any]:
        """Read one recording's public manifest and declared artifact list."""
        return await asyncio.to_thread(
            self._client.observations.recording, session_id,
        )

    async def recording_catalog(self) -> dict[str, Any]:
        """Read selectable recording topics from the native catalog."""
        return await asyncio.to_thread(
            self._client.observations.recording_catalog,
        )

    async def recording_streams(self) -> dict[str, Any]:
        """Read available recording streams and their capabilities."""
        return await asyncio.to_thread(
            self._client.observations.recording_streams,
        )

    async def download_recording_file(self, session_id: str, artifact_path: str, target: str | Path) -> Path:
        """Download a declared artifact, preserving its nested relative path."""
        return await asyncio.to_thread(
            self._client.observations.download_recording_file, session_id, artifact_path, target,
        )

    async def download_recording_archive(self, session_id: str, target: str | Path) -> Path:
        """Download a terminal recording as a tar archive."""
        return await asyncio.to_thread(
            self._client.observations.download_recording_archive, session_id, target,
        )

    async def recording_summary(self, session_id: str) -> dict[str, Any]:
        """Read recorded DDS topics, counts and missing selected topics."""
        return await asyncio.to_thread(
            self._client.observations.recording_summary, session_id,
        )

    async def validate_recording(self, session_id: str) -> dict[str, Any]:
        """Ask the native decoder to validate the retained DDS artifact."""
        return await asyncio.to_thread(
            self._client.observations.validate_recording, session_id,
        )

    async def delete_recording(self, session_id: str) -> dict[str, Any]:
        """Delete one recording through its owning service."""
        return await asyncio.to_thread(
            self._client.observations.delete_recording, session_id,
        )

    async def recording_series(
        self, session_id: str, topic: str, *, field: str | None = None,
        start: float = 0.0, end: float | None = None, max_points: int = 2000,
    ) -> dict[str, Any]:
        """Read a bounded numeric series without replaying DDS messages."""
        return await asyncio.to_thread(
            self._client.observations.recording_series,
            session_id,
            topic,
            field=field,
            start=start,
            end=end,
            max_points=max_points,
        )

    async def recording_scene_info(self, session_id: str) -> dict[str, Any]:
        """Read recorded scene timing and available preview streams."""
        return await asyncio.to_thread(
            self._client.observations.recording_scene_info, session_id,
        )

    async def recording_scene(
        self, session_id: str, *, start: float = 0.0, end: float = 5.0,
        max_frames: int = 32, max_points: int = 2048, max_poses: int = 2000,
        cloud_topic: str | None = None, pose_topic: str | None = None,
        path_topics: list[str] | None = None,
    ) -> dict[str, Any]:
        """Read a bounded recorded scene with its original topic selection."""
        return await asyncio.to_thread(
            self._client.observations.recording_scene,
            session_id,
            start=start,
            end=end,
            max_frames=max_frames,
            max_points=max_points,
            max_poses=max_poses,
            cloud_topic=cloud_topic,
            pose_topic=pose_topic,
            path_topics=path_topics,
        )

    async def recording_camera_frame(
        self, session_id: str, t: float, segment: int, *, camera_id: str = "color", width: int = 960,
    ) -> bytes:
        """Read one recorded JPEG, returning empty bytes when no frame is available."""
        return await asyncio.to_thread(
            self._client.observations.recording_camera_frame, session_id, t, segment, camera_id=camera_id, width=width,
        )


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

    async def stop_motion(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Delegate stop motion to the native motion owner."""
        return await asyncio.to_thread(
            self._client.system.stop_motion, request_id=request_id, client_id=client_id,
        )

    async def resume_motion(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Release the native motion hold through its existing owner."""
        return await asyncio.to_thread(self._client.system.resume_motion, request_id=request_id, client_id=client_id)

    async def clear_emergency_stop(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Delegate clear emergency stop to the native motion owner."""
        return await asyncio.to_thread(
            self._client.system.clear_emergency_stop, request_id=request_id, client_id=client_id,
        )

    async def stand_up(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Delegate stand up to the native motion owner."""
        return await asyncio.to_thread(
            self._client.system.stand_up, request_id=request_id, client_id=client_id,
        )

    async def sit_down(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Delegate sit down to the native motion owner."""
        return await asyncio.to_thread(
            self._client.system.sit_down, request_id=request_id, client_id=client_id,
        )

    async def enable_motors(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Delegate enable motors to the native motion owner."""
        return await asyncio.to_thread(
            self._client.system.enable_motors, request_id=request_id, client_id=client_id,
        )

    async def disable_motors(self, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Delegate disable motors to the native motion owner."""
        return await asyncio.to_thread(
            self._client.system.disable_motors, request_id=request_id, client_id=client_id,
        )

    async def switch_profile(self, name: str, *, request_id: str | None = None, client_id: str = "sdk") -> CommandResult:
        """Delegate switch profile to the native motion owner."""
        return await asyncio.to_thread(
            self._client.system.switch_profile, name, request_id=request_id, client_id=client_id,
        )

    async def product_status(self) -> dict[str, Any]:
        """Read the current ProductControl session and latest switch operation."""
        return await asyncio.to_thread(
            self._client.system.product_status,
        )

    async def switch_product(
        self, product: str, *, expected_product_session_id: str,
        map_name: str | None = None, initial_pose: dict[str, float] | None = None,
        request_id: str | None = None,
    ) -> dict[str, Any]:
        """Submit one durable switch operation to ProductControl."""
        return await asyncio.to_thread(
            self._client.system.switch_product,
            product,
            expected_product_session_id=expected_product_session_id,
            map_name=map_name,
            initial_pose=initial_pose,
            request_id=request_id,
        )

    async def product_operation(self, request_id: str) -> dict[str, Any]:
        """Read a retained switch result by its submitted request identity."""
        return await asyncio.to_thread(
            self._client.system.product_operation, request_id,
        )

    async def stop_product(self, expected_product_session_id: str, *, request_id: str | None = None) -> dict[str, Any]:
        """Submit a durable stop for the selected Product session."""
        return await asyncio.to_thread(self._client.system.stop_product, expected_product_session_id, request_id=request_id)


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
