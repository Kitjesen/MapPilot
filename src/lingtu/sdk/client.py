"""Gateway connection settings with named domain clients.

Prefer robot.navigation, robot.map, robot.tasks, robot.observations and robot.system.
Existing flat methods remain available and delegate to the same implementation.
Each request uses urllib; the root owns settings, not a persistent TCP session.
Uses only the Python standard library.
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, cast

from ._common import _connection_error_payload
from .localization import LocalizationClient as LocalizationClient
from .maps import MapClient as MapClient
from .models import CommandResult as CommandResult
from .models import HealthStatus as HealthStatus
from .models import MapInfo as MapInfo
from .models import MapList as MapList
from .models import NavigationControlState as NavigationControlState
from .models import NavigationGoalAdmission as NavigationGoalAdmission
from .models import NavigationMotionState as NavigationMotionState
from .models import NavigationStatus as NavigationStatus
from .models import NavigationTask as NavigationTask
from .models import Pose2D as Pose2D
from .models import Position as Position
from .models import RobotState as RobotState
from .models import SessionInfo as SessionInfo
from .models import _parse_command_result
from .navigation import NavigationClient as NavigationClient
from .observations import ObservationClient as ObservationClient
from .system import SystemClient as SystemClient
from .tasks import TaskClient as TaskClient


class LingTuClient:
    """Shared Gateway configuration and access to the public domain clients."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5050,
        api_key: str | None = None,
        timeout: float = 10.0,
    ) -> None:
        self.base_url = f"http://{host}:{port}"
        self._api_key = api_key
        self._timeout = timeout
        self._navigation = NavigationClient(self)
        self._map = MapClient(self)
        self._localization = LocalizationClient(self)
        self._observations = ObservationClient(self)
        self._system = SystemClient(self)
        self._tasks = TaskClient(self)

    @property
    def navigation(self) -> NavigationClient:
        """Navigation, localization, execution evidence and exploration."""
        return self._navigation

    @property
    def map(self) -> MapClient:
        """Saved maps, durable save operations, artifacts and named places."""
        return self._map

    @property
    def localization(self) -> LocalizationClient:
        """Legacy localization commands; prefer navigation for new callers."""
        return self._localization

    @property
    def observations(self) -> ObservationClient:
        """Current observations, retained history and native data recording."""
        return self._observations

    @property
    def system(self) -> SystemClient:
        """System state, diagnostics, emergency stop and operator control leases."""
        return self._system

    @property
    def tasks(self) -> TaskClient:
        """Instructions, follow and inspection with explicit task identities."""
        return self._tasks

    def __enter__(self) -> LingTuClient:
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    def go(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Compatibility alias for navigation.go()."""
        return self.navigation.go(x, y, yaw, request_id=request_id)

    def go_to(self, target: str) -> CommandResult:
        """Navigate by semantic label or natural-language instruction."""
        return self.send_instruction(target)

    def send_instruction(
        self,
        text: str,
        *,
        run_id: str | None = None,
        control_only: bool = False,
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Compatibility alias for tasks.send_instruction()."""
        return self.tasks.send_instruction(
            text, run_id=run_id, control_only=control_only, request_id=request_id, client_id=client_id
        )

    def get_agent_run(self, run_id: str) -> dict[str, Any]:
        """Compatibility alias for tasks.get_agent_run()."""
        return self.tasks.get_agent_run(run_id)

    def cancel_agent_run(
        self,
        run_id: str,
        *,
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Compatibility alias for tasks.cancel_agent_run()."""
        return self.tasks.cancel_agent_run(run_id, request_id=request_id, client_id=client_id)

    def start_follow(
        self,
        *,
        description: str | None = None,
        target_id: str | None = None,
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Compatibility alias for tasks.start_follow()."""
        return self.tasks.start_follow(
            description=description, target_id=target_id, request_id=request_id, client_id=client_id
        )

    def get_follow_task(
        self,
        follow_task_id: str,
        product_session_id: str,
    ) -> dict[str, Any]:
        """Compatibility alias for tasks.get_follow_task()."""
        return self.tasks.get_follow_task(follow_task_id, product_session_id)

    def cancel_follow_task(
        self,
        follow_task_id: str,
        product_session_id: str,
        *,
        reason: str = "operator_cancel",
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Compatibility alias for tasks.cancel_follow_task()."""
        return self.tasks.cancel_follow_task(
            follow_task_id, product_session_id, reason=reason, request_id=request_id, client_id=client_id
        )

    def follow_person(
        self,
        target_id: str,
        *,
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Start owned Follow for the published person ``target_id``."""
        return self.start_follow(
            target_id=target_id,
            request_id=request_id,
            client_id=client_id,
        )

    def stop_following(
        self,
        follow_task_id: str,
        product_session_id: str,
        *,
        reason: str = "operator_cancel",
        request_id: str | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Cancel one referenced Follow without issuing an emergency stop."""
        return self.cancel_follow_task(
            follow_task_id,
            product_session_id,
            reason=reason,
            request_id=request_id,
            client_id=client_id,
        )

    def stop(self) -> CommandResult:
        """Compatibility alias for system.emergency_stop()."""
        return self.system.emergency_stop()

    def cancel(self, reason: str = "client_cancel") -> CommandResult:
        """Compatibility alias for navigation.cancel()."""
        return self.navigation.cancel(reason)

    def navigate_click(self, x: float, y: float, yaw: float = 0.0) -> CommandResult:
        """Compatibility alias for navigation.go_to_click()."""
        return self.navigation.go_to_click(x, y, yaw)

    def wait_until_arrived(
        self,
        task_id: str | None = None,
        timeout: float = 120.0,
        poll_interval: float = 0.5,
    ) -> NavigationStatus:
        """Compatibility alias for navigation.wait_until_arrived()."""
        return self.navigation.wait_until_arrived(task_id, timeout, poll_interval)

    def state(self) -> RobotState:
        """Compatibility alias for system.state()."""
        return self.system.state()

    def health(self) -> HealthStatus:
        """Compatibility alias for system.health()."""
        return self.system.health()

    def position(self) -> Position:
        """Compatibility alias for navigation.position()."""
        return self.navigation.position()

    def session(self) -> SessionInfo:
        """Compatibility alias for system.session()."""
        return self.system.session()

    def navigation_status(self) -> NavigationStatus:
        """Compatibility alias for navigation.status()."""
        return self.navigation.status()

    def navigation_task_status(self, task_id: str) -> dict[str, Any]:
        """Compatibility alias for navigation.task()."""
        return self.navigation.task(task_id)

    def localization_status(self) -> dict[str, Any]:
        """Compatibility alias for navigation.localization_status()."""
        return self.navigation.localization_status()

    def path(self) -> dict[str, Any]:
        """Compatibility alias for navigation.path()."""
        return self.navigation.path()

    def inspection_routes(self, map_id: str | None = None) -> dict[str, Any]:
        """Compatibility alias for tasks.inspection_routes()."""
        return self.tasks.inspection_routes(map_id)

    def start_inspection(
        self,
        route_id: str,
        *,
        map_id: str | None = None,
        revision: int = 0,
        request_id: str | None = None,
    ) -> CommandResult:
        """Compatibility alias for tasks.start_inspection()."""
        return self.tasks.start_inspection(route_id, map_id=map_id, revision=revision, request_id=request_id)

    def inspection_task(self, task_id: str) -> dict[str, Any]:
        """Compatibility alias for tasks.inspection_task()."""
        return self.tasks.inspection_task(task_id)

    def pause_inspection(
        self,
        task_id: str,
        *,
        reason: str = "operator_pause",
        request_id: str | None = None,
    ) -> CommandResult:
        """Compatibility alias for tasks.pause_inspection()."""
        return self.tasks.pause_inspection(task_id, reason=reason, request_id=request_id)

    def resume_inspection(
        self,
        task_id: str,
        *,
        reason: str = "operator_resume",
        request_id: str | None = None,
    ) -> CommandResult:
        """Compatibility alias for tasks.resume_inspection()."""
        return self.tasks.resume_inspection(task_id, reason=reason, request_id=request_id)

    def cancel_inspection(
        self,
        task_id: str,
        *,
        reason: str = "operator_cancel",
        request_id: str | None = None,
    ) -> CommandResult:
        """Compatibility alias for tasks.cancel_inspection()."""
        return self.tasks.cancel_inspection(task_id, reason=reason, request_id=request_id)

    def inspection_report(self, task_id: str) -> dict[str, Any]:
        """Compatibility alias for tasks.inspection_report()."""
        return self.tasks.inspection_report(task_id)

    def maps(self) -> MapList:
        """Compatibility alias for map.list()."""
        return self.map.list()

    def save_map(
        self,
        name: str | None = None,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Compatibility alias for map.save()."""
        return self.map.save(name, request_id=request_id)

    def save_map_and_wait(
        self,
        name: str | None = None,
        *,
        request_id: str | None = None,
        timeout: float = 300.0,
        poll_interval: float = 1.0,
    ) -> dict[str, Any]:
        """Compatibility alias for map.save_and_wait()."""
        return self.map.save_and_wait(name, request_id=request_id, timeout=timeout, poll_interval=poll_interval)

    def get_map_operation(self, operation_id: str) -> dict[str, Any]:
        """Compatibility alias for map.operation()."""
        return self.map.operation(operation_id)

    def cancel_map_operation(self, operation_id: str) -> CommandResult:
        """Compatibility alias for map.cancel_operation()."""
        return self.map.cancel_operation(operation_id)

    def retry_map_operation(self, operation_id: str) -> CommandResult:
        """Compatibility alias for map.retry_operation()."""
        return self.map.retry_operation(operation_id)

    def download_map_pcd(self, name: str, target: str | Path) -> Path:
        """Compatibility alias for map.download_pcd()."""
        return self.map.download_pcd(name, target)

    def rename_map(self, old_name: str, new_name: str) -> CommandResult:
        """Compatibility alias for map.rename()."""
        return self.map.rename(old_name, new_name)

    def reset_map_cloud(self) -> CommandResult:
        """Compatibility alias for map.reset_cloud()."""
        return self.map.reset_cloud()

    def map_points(self) -> dict[str, Any]:
        """Compatibility alias for map.points()."""
        return self.map.points()

    def saved_map_points(self, name: str) -> dict[str, Any]:
        """Compatibility alias for map.saved_points()."""
        return self.map.saved_points(name)

    def set_mode(self, mode: str) -> CommandResult:
        """Compatibility alias for system.set_mode()."""
        return self.system.set_mode(mode)

    def scene(self) -> dict[str, Any]:
        """Compatibility alias for observations.scene()."""
        return self.observations.scene()

    def locations(self) -> dict[str, Any]:
        """Compatibility alias for map.locations()."""
        return self.map.locations()

    def tag_location(
        self,
        name: str,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        use_current_pose: bool = False,
    ) -> CommandResult:
        """Compatibility alias for map.tag_location()."""
        return self.map.tag_location(name, x, y, yaw, use_current_pose)

    def delete_location(self, name: str) -> CommandResult:
        """Compatibility alias for map.delete_location()."""
        return self.map.delete_location(name)

    def camera_snapshot(self) -> bytes:
        """Compatibility alias for observations.camera_snapshot()."""
        return self.observations.camera_snapshot()

    def slam_status(self) -> dict[str, Any]:
        """Compatibility alias for system.slam_status()."""
        return self.system.slam_status()

    def explore_start(self) -> CommandResult:
        """Compatibility alias for navigation.start_exploration()."""
        return self.navigation.start_exploration()

    def explore_stop(self) -> CommandResult:
        """Compatibility alias for navigation.stop_exploration()."""
        return self.navigation.stop_exploration()

    def explore_status(self) -> dict[str, Any]:
        """Compatibility alias for navigation.exploration_status()."""
        return self.navigation.exploration_status()

    def recording_start(
        self,
        prefix: str | None = None,
        *,
        duration: int = 600,
        capture_profile: str | None = None,
        task_id: str | None = None,
        camera: bool | None = None,
        minimum_free_gib: int | None = None,
    ) -> CommandResult:
        """Compatibility alias for observations.start_recording()."""
        return self.observations.start_recording(
            prefix,
            duration=duration,
            capture_profile=capture_profile,
            task_id=task_id,
            camera=camera,
            minimum_free_gib=minimum_free_gib,
        )

    def recording_stop(self) -> CommandResult:
        """Compatibility alias for observations.stop_recording()."""
        return self.observations.stop_recording()

    def recording_status(self) -> dict[str, Any]:
        """Compatibility alias for observations.recording_status()."""
        return self.observations.recording_status()

    def memory_temporal(self) -> dict[str, Any]:
        """Compatibility alias for observations.temporal_memory()."""
        return self.observations.temporal_memory()

    def memory_temporal_semantic(self, query: str) -> dict[str, Any]:
        """Compatibility alias for observations.search_memory()."""
        return self.observations.search_memory(query)

    def acquire_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Compatibility alias for system.acquire_lease()."""
        return self.system.acquire_lease(client_id, ttl)

    def release_lease(self, client_id: str) -> CommandResult:
        """Compatibility alias for system.release_lease()."""
        return self.system.release_lease(client_id)

    def renew_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Compatibility alias for system.renew_lease()."""
        return self.system.renew_lease(client_id, ttl)

    def capabilities(self) -> dict[str, Any]:
        """Compatibility alias for system.capabilities()."""
        return self.system.capabilities()

    def bootstrap(self) -> dict[str, Any]:
        """Compatibility alias for system.bootstrap()."""
        return self.system.bootstrap()

    def readiness(self) -> dict[str, Any]:
        """Compatibility alias for system.readiness()."""
        return self.system.readiness()

    def field_check(self) -> CommandResult:
        """Compatibility alias for system.field_check()."""
        return self.system.field_check()

    def runtime_contract(self) -> dict[str, Any]:
        """Compatibility alias for system.runtime_contract()."""
        return self.system.runtime_contract()

    def _command(
        self,
        path: str,
        data: dict[str, Any] | None = None,
        *,
        method: str = "POST",
    ) -> CommandResult:
        raw = self._request_json(path, data, method=method)
        return _parse_command_result(raw)

    def _get(self, path: str) -> dict[str, Any]:
        url = self.base_url + path
        req = urllib.request.Request(url)
        if self._api_key:
            req.add_header("X-API-Key", self._api_key)
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as r:
                return cast(dict[str, Any], json.loads(r.read()))
        except urllib.error.HTTPError as exc:
            body = exc.read()
            try:
                detail = json.loads(body)
            except json.JSONDecodeError:
                detail = body.decode(errors="replace")
            raise RuntimeError(f"GET {path} failed with HTTP {exc.code}: {detail}") from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise ConnectionError(f"GET {path} failed: {exc}") from exc

    def _request_json(
        self,
        path: str,
        data: dict[str, Any] | None = None,
        *,
        method: str = "POST",
    ) -> dict[str, Any]:
        url = self.base_url + path
        body = json.dumps(data or {}).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=body,
            headers={"Content-Type": "application/json"},
            method=method,
        )
        if self._api_key:
            req.add_header("X-API-Key", self._api_key)
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as r:
                return cast(dict[str, Any], json.loads(r.read()))
        except urllib.error.HTTPError as exc:
            err_body = exc.read()
            try:
                return cast(dict[str, Any], json.loads(err_body))
            except json.JSONDecodeError:
                return {"error": f"HTTP {exc.code}", "detail": err_body.decode(errors="replace")}
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            return _connection_error_payload(path, exc)

    def close(self) -> None:
        """Close any held resources (no-op for the stdlib client)."""
        pass
