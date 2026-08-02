"""ROS-free building mission orchestration above native navigation."""

from __future__ import annotations

import logging
import math
from collections.abc import Callable
from typing import Protocol

from nav.building.model import (
    ActiveFloor,
    BuildingMissionPhase,
    BuildingMissionRequest,
    BuildingMissionStatus,
    GoalProgress,
    PoseTarget,
)

logger = logging.getLogger(__name__)


class BuildingNavigationPort(Protocol):
    """Native navigation capability required by building orchestration."""

    def autonomy_ready(self) -> tuple[bool, str]:
        """Report whether the mutually exclusive native mode is autonomy."""

    def send_goal(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        request_id: str | None = None,
    ) -> None:
        """Submit one map-frame goal and wait for its business ACK."""

    def observe_goal(
        self,
        *,
        request_id: str,
        map_id: str,
        target: PoseTarget,
    ) -> GoalProgress:
        """Observe only completion correlated to the supplied goal."""

    def cancel(self, reason: str = "cancel", *, request_id: str | None = None) -> None:
        """Cancel active native navigation."""

    def stop(self, reason: str = "stop", *, request_id: str | None = None) -> None:
        """Clear active motion without latching estop."""


class BuildingMissionPort(Protocol):
    """Stable high-level entry used by Open-RMF and native callers alike."""

    def submit(self, request: BuildingMissionRequest) -> tuple[bool, str]:
        """Admit one floor-aware mission without exposing velocity control."""


class FloorTransitionPort(Protocol):
    """Optional connector transition capability used for cross-floor work."""

    def start(
        self,
        request: BuildingMissionRequest,
        *,
        source_floor: ActiveFloor,
    ) -> tuple[bool, str]:
        """Start a configured cross-floor transition."""

    def tick(self) -> tuple[str, str]:
        """Return pending, executing, succeeded, failed, or blocked plus reason."""

    def cancel(self, *, reason: str) -> None:
        """Abort transition motion and release connector ownership."""


class BuildingMissionOrchestrator:
    """Execute one floor-aware mission while preserving native mode ownership."""

    def __init__(
        self,
        *,
        navigation: BuildingNavigationPort,
        active_floor: ActiveFloor | Callable[[], ActiveFloor],
        transition_executor: FloorTransitionPort | None = None,
        target_binding_validator: Callable[[BuildingMissionRequest], tuple[bool, str]] | None = None,
    ) -> None:
        self._navigation = navigation
        self._active_floor_provider = active_floor if callable(active_floor) else lambda: active_floor
        self._transition_executor = transition_executor
        self._target_binding_validator = target_binding_validator
        self._request: BuildingMissionRequest | None = None
        self._goal_request_id = ""
        self._motion_active = False
        self._status = BuildingMissionStatus(BuildingMissionPhase.IDLE)

    def submit(self, request: BuildingMissionRequest) -> tuple[bool, str]:
        """Admit a same-floor goal or start an explicit connector transition."""

        if self._status.active:
            return False, "building_mission_busy"
        validation_error = self._validate_request(request)
        if validation_error:
            return self._reject(request, validation_error)
        if self._has_target_binding(request) and self._target_binding_validator is None:
            return self._reject(request, "target_binding_validator_unavailable")
        ready, readiness_reason = self._autonomy_ready()
        if not ready:
            return self._reject(
                request,
                f"autonomy_not_ready:{readiness_reason or 'unknown'}",
            )

        try:
            active_floor = self._active_floor_provider()
        except Exception:
            return self._reject(request, "active_floor_unavailable")
        if not isinstance(active_floor, ActiveFloor):
            return self._reject(request, "active_floor_invalid")
        self._request = request
        if request.target_floor != active_floor:
            if self._transition_executor is None:
                return self._reject(request, "floor_transition_executor_unavailable")
            try:
                accepted, reason = self._transition_executor.start(
                    request,
                    source_floor=active_floor,
                )
            except Exception:
                return self._reject(request, "floor_transition_start_error")
            if accepted is not True:
                return self._reject(request, str(reason or "floor_transition_rejected"))
            self._status = BuildingMissionStatus(
                phase=BuildingMissionPhase.FLOOR_TRANSITION,
                request_id=request.request_id,
                reason=str(reason or "floor_transition_started"),
                target_floor=request.target_floor,
            )
            return True, str(reason or "floor_transition_started")

        if request.travel_mode in {"stairs", "elevator"}:
            return self._reject(request, "connector_transition_not_required")

        return self._dispatch_target_goal(request)

    def tick(self) -> BuildingMissionStatus:
        """Advance the active mission by one deterministic orchestration step."""

        if not self._status.active or self._request is None:
            return self._status
        ready, readiness_reason = self._autonomy_ready()
        if not ready:
            return self._abort(f"autonomy_not_ready:{readiness_reason or 'unknown'}")

        if self._status.phase is BuildingMissionPhase.FLOOR_TRANSITION:
            return self._tick_transition()
        return self._tick_target_goal()

    def cancel(self, reason: str = "cancel") -> BuildingMissionStatus:
        """Cancel active navigation/transition and never restore stale motion."""

        if self._status.active:
            self._cancel_motion(reason=f"building_mission_cancel:{reason or 'cancel'}")
        self._status = BuildingMissionStatus(
            phase=BuildingMissionPhase.CANCELLED,
            request_id=self._request.request_id if self._request is not None else "",
            reason=str(reason or "cancel"),
            target_floor=self._request.target_floor if self._request is not None else None,
        )
        self._motion_active = False
        return self._status

    def status(self) -> BuildingMissionStatus:
        """Return the latest immutable mission status."""

        return self._status

    def _tick_transition(self) -> BuildingMissionStatus:
        transition_executor = self._transition_executor
        request = self._request
        if transition_executor is None or request is None:
            return self._abort("building_mission_state_invalid")
        try:
            progress, reason = transition_executor.tick()
        except Exception:
            return self._abort("floor_transition_tick_error")
        progress = str(progress).strip().lower()
        if progress in {"pending", "executing"}:
            self._status = BuildingMissionStatus(
                phase=BuildingMissionPhase.FLOOR_TRANSITION,
                request_id=request.request_id,
                reason=str(reason or progress),
                target_floor=request.target_floor,
                transition_phase=str(reason or progress),
            )
            return self._status
        if progress == "succeeded":
            try:
                active_floor = self._active_floor_provider()
            except Exception:
                return self._abort("active_floor_unavailable")
            if not isinstance(active_floor, ActiveFloor):
                return self._abort("active_floor_invalid")
            if active_floor != request.target_floor:
                return self._abort("floor_transition_identity_unverified")
            accepted, _reason = self._dispatch_target_goal(request)
            if not accepted:
                return self._status
            self._status = BuildingMissionStatus(
                phase=BuildingMissionPhase.TARGET_NAVIGATION,
                request_id=request.request_id,
                reason="floor_transition_complete; target_navigation_started",
                target_floor=request.target_floor,
            )
            return self._status
        return self._abort(str(reason or f"floor_transition_{progress or 'blocked'}"))

    def _tick_target_goal(self) -> BuildingMissionStatus:
        request = self._request
        if request is None:
            return self._abort("building_mission_state_invalid")
        try:
            progress = self._navigation.observe_goal(
                request_id=self._goal_request_id,
                map_id=request.map_id,
                target=request.target,
            )
            progress = progress if isinstance(progress, GoalProgress) else GoalProgress(str(progress).lower())
        except (TypeError, ValueError):
            return self._abort("native_navigation_goal_status_invalid")
        except Exception:
            return self._abort("native_navigation_goal_status_error")

        if progress in {GoalProgress.PENDING, GoalProgress.EXECUTING}:
            return self._status
        if progress is GoalProgress.SUCCEEDED:
            self._motion_active = False
            self._status = BuildingMissionStatus(
                phase=BuildingMissionPhase.SUCCEEDED,
                request_id=request.request_id,
                reason="building_navigation_goal_reached",
                target_floor=request.target_floor,
            )
            return self._status
        return self._abort(f"native_navigation_goal_{progress.value}")

    def _dispatch_target_goal(self, request: BuildingMissionRequest) -> tuple[bool, str]:
        self._goal_request_id = f"{request.request_id}:goal"
        binding_error = self._validate_target_binding(request)
        if binding_error:
            return self._reject(request, binding_error)
        try:
            self._navigation.send_goal(
                request.target.x,
                request.target.y,
                request.target.z,
                request.target.yaw,
                request_id=self._goal_request_id,
            )
        except Exception:
            return self._reject(request, "native_navigation_goal_error")
        self._motion_active = True
        self._status = BuildingMissionStatus(
            phase=BuildingMissionPhase.TARGET_NAVIGATION,
            request_id=request.request_id,
            reason="building_navigation_goal_accepted",
            target_floor=request.target_floor,
        )
        return True, "building_navigation_goal_accepted"

    def _abort(self, reason: str) -> BuildingMissionStatus:
        self._cancel_motion(reason=f"building_mission_abort:{reason}")
        self._status = BuildingMissionStatus(
            phase=BuildingMissionPhase.FAILED,
            request_id=self._request.request_id if self._request is not None else "",
            reason=reason,
            target_floor=self._request.target_floor if self._request is not None else None,
        )
        self._motion_active = False
        return self._status

    def _cancel_motion(self, *, reason: str) -> None:
        if self._status.phase is BuildingMissionPhase.FLOOR_TRANSITION and self._transition_executor is not None:
            try:
                self._transition_executor.cancel(reason=reason)
            except Exception:
                logger.debug("Failed to cancel floor transition", exc_info=True)
        if self._motion_active and self._request is not None:
            try:
                self._navigation.cancel(
                    reason,
                    request_id=f"{self._request.request_id}:cancel",
                )
            except Exception:
                logger.debug("Failed to cancel native navigation", exc_info=True)

    def _reject(self, request: BuildingMissionRequest, reason: str) -> tuple[bool, str]:
        self._request = request
        self._motion_active = False
        self._status = BuildingMissionStatus(
            phase=BuildingMissionPhase.FAILED,
            request_id=str(getattr(request, "request_id", "")),
            reason=reason,
            target_floor=getattr(request, "target_floor", None),
        )
        return False, reason

    def _autonomy_ready(self) -> tuple[bool, str]:
        try:
            ready, reason = self._navigation.autonomy_ready()
        except Exception:
            return False, "native_autonomy_status_error"
        return ready is True, str(reason or "")

    def _validate_target_binding(self, request: BuildingMissionRequest) -> str:
        if not self._has_target_binding(request):
            return ""
        validator = self._target_binding_validator
        if validator is None:
            return "target_binding_validator_unavailable"
        try:
            valid, reason = validator(request)
        except Exception:
            return "target_binding_validation_error"
        if valid is not True:
            return str(reason or "target_binding_validation_failed")
        return ""

    @staticmethod
    def _has_target_binding(request: BuildingMissionRequest) -> bool:
        return request.map_version is not None or bool(request.version_id) or bool(request.map_pcd_sha256)

    @staticmethod
    def _validate_request(request: BuildingMissionRequest) -> str:
        if not isinstance(request, BuildingMissionRequest):
            return "invalid_building_mission"
        if not request.request_id.strip():
            return "request_id_required"
        if not request.building_id.strip() or not request.floor_id.strip() or not request.map_id.strip():
            return "building_floor_identity_required"
        if request.travel_mode not in {"any", "stairs", "elevator"}:
            return "invalid_travel_mode"
        if request.target.frame_id != "map":
            return "unsupported_navigation_frame"
        if not all(
            math.isfinite(value)
            for value in (
                request.target.x,
                request.target.y,
                request.target.z,
                request.target.yaw,
            )
        ):
            return "invalid_navigation_target"
        return ""
