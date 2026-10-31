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
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Navigate to map coordinates."""
        resolved_request_id = str(request_id or "").strip() or _new_request_id()
        result = self._client._command(
            "/api/v1/goal",
            {"x": x, "y": y, "yaw": yaw, "request_id": resolved_request_id},
        )
        if result.request_id is None:
            result.request_id = resolved_request_id
        return result

    def go_to_click(self, x: float, y: float, yaw: float = 0.0) -> CommandResult:
        """Navigate to a map-viewer click point."""
        return self._client._command("/api/v1/navigate/click", {"x": x, "y": y, "yaw": yaw})

    def cancel(self, reason: str = "client_cancel") -> CommandResult:
        """Gracefully cancel the current navigation mission."""
        return self._client._command("/api/v1/navigation/cancel", {"reason": reason})

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
