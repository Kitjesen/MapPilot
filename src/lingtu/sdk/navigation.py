"""Navigation, localization, execution evidence and exploration."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any
from urllib.parse import quote

from ._common import _new_request_id
from .models import (
    CommandResult,
    NavigationStatus,
    NavigationTask,
    Pose2D,
    Position,
    _mapping,
    _parse_navigation_status,
)

if TYPE_CHECKING:
    from .client import LingTuClient


_NAVIGATION_FAILURE_STATES = frozenset({"FAILED", "CANCELLED"})


class NavigationClient:
    """Navigation, localization, execution evidence and exploration."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

    def go(
        self, x: float, y: float, yaw: float | None = None, *,
        z: float = 0.0, task_id: str | None = None, request_id: str | None = None,
        acceptance_radius_m: float | None = None, max_speed_mps: float | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Navigate in the active map; omitted yaw requests position only."""
        return self._goal("/api/v1/goal", x, y, yaw, z=z, task_id=task_id, request_id=request_id,
            acceptance_radius_m=acceptance_radius_m, max_speed_mps=max_speed_mps,
            client_id=client_id)

    def go_to_click(
        self, x: float, y: float, yaw: float | None = None, *,
        z: float = 0.0, task_id: str | None = None, request_id: str | None = None,
        acceptance_radius_m: float | None = None, max_speed_mps: float | None = None,
        client_id: str = "sdk",
    ) -> CommandResult:
        """Navigate to a 3D map click. Yaw is radians, omitted yaw requests position only."""
        return self._goal("/api/v1/navigate/click", x, y, yaw, z=z, task_id=task_id, request_id=request_id,
            acceptance_radius_m=acceptance_radius_m, max_speed_mps=max_speed_mps,
            client_id=client_id)

    def _goal(
        self, path: str, x: float, y: float, yaw: float | None, *, z: float,
        task_id: str | None, request_id: str | None, acceptance_radius_m: float | None,
        max_speed_mps: float | None, client_id: str,
    ) -> CommandResult:
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        body = {"x": x, "y": y, "z": z, "yaw": yaw,
                "request_id": resolved_request_id, "client_id": client_id}
        body.update({key: value for key, value in {
            "task_id": task_id, "acceptance_radius_m": acceptance_radius_m,
            "max_speed_mps": max_speed_mps,
        }.items() if value is not None})
        result = self._client._command(path, body)
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

    def select_planner(
        self, name: str, *, request_id: str | None = None, client_id: str = "sdk",
    ) -> CommandResult:
        """Select SCAN/CMU while idle; success means the native switch completed."""
        return self._client._command("/api/v1/navigation/planner", {
            "name": name, "request_id": request_id or _new_request_id(), "client_id": client_id,
        })

    def cancel(
        self, reason: str = "client_cancel", *, task_id: str | None = None,
        request_id: str | None = None, client_id: str = "sdk",
    ) -> CommandResult:
        """Cancel an identified task, or the current task when omitted."""
        if task_id is not None:
            return self._control("cancel", task_id, reason, request_id, client_id)
        body = {"reason": reason, "client_id": client_id}
        if request_id is not None:
            body["request_id"] = request_id
        return self._client._command("/api/v1/navigation/cancel", body)

    def pause(
        self, task_id: str, *, reason: str = "operator_pause",
        request_id: str | None = None, client_id: str = "sdk",
    ) -> CommandResult:
        """Request pause; the receipt does not confirm physical standstill."""
        return self._control("pause", task_id, reason, request_id, client_id)

    def resume(
        self, task_id: str, *, reason: str = "operator_resume",
        request_id: str | None = None, client_id: str = "sdk",
    ) -> CommandResult:
        """Continue the same task without submitting a new goal."""
        return self._control("resume", task_id, reason, request_id, client_id)

    def set_speed_limit(
        self, task_id: str, max_speed_mps: float, *,
        request_id: str | None = None, client_id: str = "sdk",
    ) -> CommandResult:
        """Set the active task speed limit through native admission."""
        return self._control("speed-limit", task_id, "", request_id, client_id,
                             max_speed_mps=max_speed_mps)

    def _control(
        self, action: str, task_id: str, reason: str, request_id: str | None,
        client_id: str, **fields: float,
    ) -> CommandResult:
        task_id = str(task_id).strip()
        if not task_id:
            raise ValueError("task_id is required")
        body = {"task_id": task_id, "request_id": request_id or _new_request_id(),
                "client_id": client_id, "reason": reason, **fields}
        return self._client._command(
            f"/api/v1/navigation/tasks/{quote(task_id, safe='')}/{action}", body,
        )

    def wait_until_arrived(
        self,
        task_id: str | None = None,
        timeout: float = 120.0,
        poll_interval: float = 0.5,
    ) -> NavigationStatus:
        """Block until a navigation task completes.

        A task identity uses the task ledger. Without one, only the current v3
        task state is observed; no goal or legacy-status correlation is used.
        """
        normalized_task_id = str(task_id or "").strip()
        if normalized_task_id:
            return self._wait_for_navigation_task(
                normalized_task_id,
                timeout=timeout,
                poll_interval=poll_interval,
            )

        deadline = time.monotonic() + timeout
        last_state = "UNKNOWN"
        while time.monotonic() < deadline:
            result = self.status()
            last_state = result.task.state.upper()
            if last_state == "SUCCESS":
                return result
            if last_state in _NAVIGATION_FAILURE_STATES:
                raise RuntimeError(f"Navigation {last_state}: {result.task.reason or 'unknown reason'}")
            time.sleep(poll_interval)

        raise TimeoutError(f"Navigation did not complete within {timeout}s (state={last_state})")

    def status(self) -> NavigationStatus:
        """Get current navigation mission status."""
        raw = self._client._get("/api/v1/navigation/status")
        return _parse_navigation_status(raw)

    def task(self, task_id: str) -> dict[str, Any]:
        """Get lifecycle evidence for one stable navigation task identity."""
        normalized = str(task_id or "").strip()
        if not normalized:
            raise ValueError("task_id is required")
        return self._client._get(f"/api/v1/navigation/tasks/{quote(normalized, safe='')}")

    def path(self) -> dict[str, Any]:
        """Get the latest planned global path as a list of poses."""
        return self._client._get("/api/v1/path")

    def goal_status(self, request_id: str) -> dict[str, Any]:
        """Reconcile a goal submission using its caller-selected request identity."""
        return self._client._get(f"/api/v1/navigation/goals/{quote(request_id, safe='')}")

    def start_exploration(self) -> CommandResult:
        """Start autonomous frontier exploration."""
        return self._client._command("/api/v1/explore/start")

    def stop_exploration(self) -> CommandResult:
        """Stop autonomous frontier exploration."""
        return self._client._command("/api/v1/explore/stop")

    def exploration_status(self) -> dict[str, Any]:
        """Get exploration status."""
        return self._client._get("/api/v1/explore/status")

    def _wait_for_navigation_task(
        self,
        task_id: str,
        *,
        timeout: float,
        poll_interval: float,
    ) -> NavigationStatus:
        deadline = time.monotonic() + timeout
        last_state = "UNKNOWN"
        while time.monotonic() < deadline:
            payload = self.task(task_id)
            status_payload = _mapping(payload.get("status"))
            observed_task_id = str(status_payload.get("task_id") or payload.get("task_id") or "").strip()
            identity_matches = observed_task_id == task_id
            if payload.get("found") is True and status_payload and identity_matches:
                last_state = str(
                    status_payload.get("lifecycle_state_name") or status_payload.get("state_name") or "UNKNOWN"
                ).upper()
                result = NavigationStatus(
                    task=NavigationTask(
                        state=last_state,
                        task_id=task_id,
                        reason=str(
                            status_payload.get("failure_reason")
                            or status_payload.get("reason")
                            or payload.get("reason")
                            or ""
                        ),
                    ),
                    raw=dict(payload),
                )
                if last_state == "SUCCESS":
                    return result
                if last_state in _NAVIGATION_FAILURE_STATES:
                    reason = str(
                        status_payload.get("failure_reason")
                        or status_payload.get("reason")
                        or payload.get("reason")
                        or "unknown reason"
                    )
                    raise RuntimeError(f"Navigation {last_state}: {reason}")
            time.sleep(poll_interval)

        raise TimeoutError(f"Navigation task {task_id} did not complete within {timeout}s (state={last_state})")

    def relocalize(
        self,
        map_name: str,
        *,
        initial_pose: Pose2D,
        request_id: str | None = None,
    ) -> CommandResult:
        """Relocalize on the active map from a caller-provided pose seed."""
        body: dict[str, Any] = {
            "mode": "seeded",
            "map_name": map_name,
            "initial_pose": {
                "x": initial_pose.x,
                "y": initial_pose.y,
                "yaw": initial_pose.yaw,
            },
        }
        if request_id is not None:
            body["request_id"] = request_id
        return self._client._command("/api/v1/localization/relocalizations", body)

    def global_relocalize(
        self,
        map_name: str,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Relocalize on the active map without a pose seed."""
        body: dict[str, Any] = {"mode": "global", "map_name": map_name}
        if request_id is not None:
            body["request_id"] = request_id
        return self._client._command("/api/v1/localization/relocalizations", body)

    def start_map_tracking(
        self,
        map_name: str,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Start continuous alignment against the active map."""
        body: dict[str, Any] = {"map_name": map_name}
        if request_id is not None:
            body["request_id"] = request_id
        return self._client._command("/api/v1/localization/map-tracking", body)

    def localization_status(self) -> dict[str, Any]:
        """Get localization health: alignment, residual, fix quality."""
        return self._client._get("/api/v1/localization/status")

    def position(self) -> Position:
        """Get the current map-frame position."""
        position = self._client.system.state().odometry
        if position is None:
            raise RuntimeError("current map-frame position is unavailable")
        return position
