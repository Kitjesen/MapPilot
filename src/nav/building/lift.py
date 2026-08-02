"""Deterministic ROS-free lift transition executor."""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable
from typing import Protocol

from nav.building.model import (
    ActiveFloor,
    BuildingMissionRequest,
    GoalProgress,
    LiftState,
    LiftTransitionPhase,
    LiftTransitionPlan,
    LiftTransitionStatus,
    PoseTarget,
)
from nav.building.orchestrator import BuildingNavigationPort

logger = logging.getLogger(__name__)


class LiftCommandPort(Protocol):
    """Building-controller adapter boundary; no PLC protocol leaks inward."""

    def request(
        self,
        *,
        lift_id: str,
        destination_floor_id: str,
        session_id: str,
    ) -> tuple[bool, str]:
        """Claim/renew one session and request a destination floor."""

    def release(self, *, lift_id: str, session_id: str) -> tuple[bool, str]:
        """Release a previously claimed lift session."""

    def snapshot(self, lift_id: str) -> LiftState | None:
        """Return the latest transport-normalized lift state."""


class FloorLocalizationPort(Protocol):
    """Atomic map activation plus native saved-map relocalization boundary."""

    def active_floor(self) -> ActiveFloor:
        """Return floor identity backed by the active native map."""

    def switch_and_relocalize(
        self,
        floor: ActiveFloor,
        seed: PoseTarget,
    ) -> tuple[bool, str]:
        """Activate the target map and request native relocalization."""

    def is_localized(self, floor: ActiveFloor) -> bool:
        """Verify fresh tracking on the target floor after the request."""


class StaticLiftTransitionCatalog:
    """Explicit directed lift routes; reverse travel needs its own plan."""

    def __init__(self, plans: list[LiftTransitionPlan] | tuple[LiftTransitionPlan, ...]) -> None:
        self._plans: dict[tuple[ActiveFloor, ActiveFloor], LiftTransitionPlan] = {}
        for plan in plans:
            key = (plan.source_floor, plan.target_floor)
            if key in self._plans:
                raise ValueError(
                    f"duplicate lift transition route: {plan.source_floor.map_id}->{plan.target_floor.map_id}"
                )
            self._plans[key] = plan

    def resolve(
        self,
        source_floor: ActiveFloor,
        target_floor: ActiveFloor,
    ) -> LiftTransitionPlan | None:
        """Return the exact directed route for a source/target floor pair."""

        return self._plans.get((source_floor, target_floor))


class LiftTransitionService:
    """Resolve a floor pair and expose LiftTransitionExecutor to orchestration."""

    def __init__(
        self,
        *,
        catalog: StaticLiftTransitionCatalog,
        executor: LiftTransitionExecutor,
    ) -> None:
        self._catalog = catalog
        self._executor = executor

    def start(
        self,
        request: BuildingMissionRequest,
        *,
        source_floor: ActiveFloor,
    ) -> tuple[bool, str]:
        """Resolve and start the configured transition for one mission."""

        if request.travel_mode not in {"any", "elevator"}:
            return False, "lift_transition_travel_mode_unsupported"
        plan = self._catalog.resolve(source_floor, request.target_floor)
        if plan is None:
            return False, "floor_transition_route_unavailable"
        if request.connector_id and request.connector_id != plan.lift_id:
            return False, "lift_connector_mismatch"
        accepted, reason = self._executor.start(plan, request_id=request.request_id)
        if accepted is not True:
            return False, str(reason or "lift_transition_rejected")
        return True, str(reason or "lift_transition_started")

    def tick(self) -> tuple[str, str]:
        """Expose executor progress through the building-mission port contract."""

        status = self._executor.tick()
        if status.phase is LiftTransitionPhase.SUCCEEDED:
            return "succeeded", status.reason
        if status.phase in {
            LiftTransitionPhase.FAILED,
            LiftTransitionPhase.CANCELLED,
        }:
            return "failed", status.reason
        if status.phase is LiftTransitionPhase.IDLE:
            return "blocked", "lift_transition_not_started"
        return "executing", status.phase.value.lower()

    def cancel(self, *, reason: str) -> None:
        """Cancel the active transition and release connector ownership."""

        self._executor.cancel(reason=reason)

    def status(self) -> LiftTransitionStatus:
        """Return the executor's latest immutable status."""

        return self._executor.status()


class LiftTransitionExecutor:
    """Call, enter, ride, relocalize, and exit one configured lift."""

    def __init__(
        self,
        *,
        navigation: BuildingNavigationPort,
        lift: LiftCommandPort,
        floor_localization: FloorLocalizationPort,
        clock: Callable[[], float] = time.monotonic,
        max_lift_state_age_s: float = 1.0,
        step_timeout_s: float = 45.0,
        ride_timeout_s: float = 180.0,
    ) -> None:
        self._navigation = navigation
        self._lift = lift
        self._floors = floor_localization
        self._clock = clock
        self._max_lift_state_age_s = max(0.05, float(max_lift_state_age_s))
        self._step_timeout_s = max(1.0, float(step_timeout_s))
        self._ride_timeout_s = max(self._step_timeout_s, float(ride_timeout_s))
        self._plan: LiftTransitionPlan | None = None
        self._request_id = ""
        self._session_id = ""
        self._goal_request_id = ""
        self._phase_started_s = 0.0
        self._lift_claimed = False
        self._motion_active = False
        self._status = LiftTransitionStatus(LiftTransitionPhase.IDLE)

    def start(
        self,
        plan: LiftTransitionPlan,
        *,
        request_id: str,
    ) -> tuple[bool, str]:
        """Start with a source-lobby goal; all later actions advance in tick()."""

        if self._status.active:
            return False, "lift_transition_busy"
        reason = self._validate_plan(plan, request_id=request_id)
        if reason:
            self._set_terminal(LiftTransitionPhase.FAILED, reason)
            return False, reason
        ready, readiness_reason = self._autonomy_ready()
        if not ready:
            reason = f"autonomy_not_ready:{readiness_reason or 'unknown'}"
            self._set_terminal(LiftTransitionPhase.FAILED, reason)
            return False, reason
        if self._floors.active_floor() != plan.source_floor:
            self._set_terminal(LiftTransitionPhase.FAILED, "source_floor_identity_mismatch")
            return False, "source_floor_identity_mismatch"

        self._plan = plan
        self._request_id = str(request_id).strip()
        self._session_id = f"{self._request_id}:lift"
        self._lift_claimed = False
        if not self._send_goal(plan.source_lobby, suffix="approach"):
            return False, self._status.reason
        self._set_phase(LiftTransitionPhase.APPROACH_SOURCE, "approaching_source_lobby")
        return True, "lift_transition_started"

    def tick(self) -> LiftTransitionStatus:
        """Advance exactly one transition step from fresh, correlated evidence."""

        if not self._status.active or self._plan is None:
            if self._lift_claimed and self._status.phase in {
                LiftTransitionPhase.FAILED,
                LiftTransitionPhase.CANCELLED,
            }:
                return self._retry_release()
            return self._status
        ready, readiness_reason = self._autonomy_ready()
        if not ready:
            return self._fail(f"autonomy_not_ready:{readiness_reason or 'unknown'}")
        if self._phase_timed_out():
            return self._fail(f"{self._status.phase.value.lower()}_timeout")

        phase = self._status.phase
        if phase is LiftTransitionPhase.APPROACH_SOURCE:
            return self._tick_approach()
        if phase is LiftTransitionPhase.WAIT_SOURCE_DOOR:
            return self._tick_wait_source()
        if phase is LiftTransitionPhase.ENTER_CABIN:
            return self._tick_enter()
        if phase is LiftTransitionPhase.RIDE:
            return self._tick_ride()
        if phase is LiftTransitionPhase.VERIFY_TARGET_LOCALIZATION:
            return self._tick_localization()
        if phase is LiftTransitionPhase.EXIT_CABIN:
            return self._tick_exit()
        return self._fail("invalid_lift_transition_phase")

    def cancel(self, *, reason: str = "cancel") -> LiftTransitionStatus:
        """Stop robot motion, release our lift session, and stay terminal."""

        if self._status.active:
            self._stop_and_cancel(reason=f"lift_transition_cancel:{reason or 'cancel'}")
            released, release_reason = self._release_lift()
            final_reason = str(reason or "cancel")
            if not released:
                final_reason += f";lift_release_pending:{release_reason or 'unconfirmed'}"
        else:
            final_reason = str(reason or "cancel")
        self._set_terminal(LiftTransitionPhase.CANCELLED, final_reason)
        return self._status

    def status(self) -> LiftTransitionStatus:
        """Return the latest immutable transition status."""

        return self._status

    def _tick_approach(self) -> LiftTransitionStatus:
        plan = self._plan
        if plan is None:
            return self._fail("lift_transition_plan_missing")
        progress = self._observe_goal(plan.source_floor.map_id, plan.source_lobby)
        if progress in {GoalProgress.PENDING, GoalProgress.EXECUTING}:
            return self._status
        if progress is not GoalProgress.SUCCEEDED:
            return self._fail(f"source_approach_{progress.value}")
        self._motion_active = False
        accepted, reason = self._request_lift(plan.source_floor.floor_id)
        if not accepted:
            return self._fail(reason or "lift_source_request_rejected")
        self._set_phase(LiftTransitionPhase.WAIT_SOURCE_DOOR, "waiting_source_door")
        return self._status

    def _tick_wait_source(self) -> LiftTransitionStatus:
        plan = self._plan
        if plan is None:
            return self._fail("lift_transition_plan_missing")
        state, error = self._validated_lift_state()
        if error:
            return self._fail(error)
        if state is None:
            return self._fail("lift_state_missing")
        if state.current_floor_id != plan.source_floor.floor_id:
            return self._status_with_reason("waiting_lift_at_source_floor")
        if self._normal(state.motion_state) != "stopped":
            return self._status_with_reason("waiting_lift_to_stop_at_source")
        if self._normal(state.door_state) != "open":
            return self._status_with_reason("waiting_source_door_open")
        if not self._send_goal(plan.source_cabin, suffix="enter"):
            return self._status
        self._set_phase(LiftTransitionPhase.ENTER_CABIN, "entering_lift_cabin")
        return self._status

    def _tick_enter(self) -> LiftTransitionStatus:
        plan = self._plan
        if plan is None:
            return self._fail("lift_transition_plan_missing")
        error = self._door_motion_error(
            required_floor_id=plan.source_floor.floor_id,
            label="source",
        )
        if error:
            return self._fail(error)
        progress = self._observe_goal(plan.source_floor.map_id, plan.source_cabin)
        if progress in {GoalProgress.PENDING, GoalProgress.EXECUTING}:
            return self._status
        if progress is not GoalProgress.SUCCEEDED:
            return self._fail(f"lift_entry_{progress.value}")
        self._motion_active = False
        try:
            self._navigation.stop(
                "lift_ride_hold",
                request_id=f"{self._session_id}:hold",
            )
        except Exception:
            return self._fail("lift_ride_hold_error")
        accepted, reason = self._request_lift(plan.target_floor.floor_id)
        if not accepted:
            return self._fail(reason or "lift_destination_request_rejected")
        self._set_phase(LiftTransitionPhase.RIDE, "riding_to_target_floor")
        return self._status

    def _tick_ride(self) -> LiftTransitionStatus:
        plan = self._plan
        if plan is None:
            return self._fail("lift_transition_plan_missing")
        state, error = self._validated_lift_state()
        if error:
            return self._fail(error)
        if state is None:
            return self._fail("lift_state_missing")
        if state.current_floor_id != plan.target_floor.floor_id:
            return self._status_with_reason("lift_in_transit")
        if self._normal(state.motion_state) != "stopped":
            return self._status_with_reason("waiting_lift_to_stop_at_target")
        if self._normal(state.door_state) != "open":
            return self._status_with_reason("waiting_target_door_open")
        try:
            switched, reason = self._floors.switch_and_relocalize(
                plan.target_floor,
                plan.target_cabin,
            )
        except Exception:
            return self._fail("target_map_relocalization_error")
        if switched is not True:
            return self._fail(str(reason or "target_map_relocalization_failed"))
        self._set_phase(
            LiftTransitionPhase.VERIFY_TARGET_LOCALIZATION,
            "waiting_target_localization_verification",
        )
        return self._status

    def _tick_localization(self) -> LiftTransitionStatus:
        plan = self._plan
        if plan is None:
            return self._fail("lift_transition_plan_missing")
        error = self._door_motion_error(
            required_floor_id=plan.target_floor.floor_id,
            label="target",
        )
        if error:
            return self._fail(error)
        try:
            localized = self._floors.is_localized(plan.target_floor)
        except Exception:
            return self._fail("target_localization_status_error")
        if localized is not True:
            return self._status_with_reason("waiting_target_localization_verification")
        if not self._send_goal(plan.target_lobby, suffix="exit"):
            return self._status
        self._set_phase(LiftTransitionPhase.EXIT_CABIN, "exiting_lift_cabin")
        return self._status

    def _tick_exit(self) -> LiftTransitionStatus:
        plan = self._plan
        if plan is None:
            return self._fail("lift_transition_plan_missing")
        error = self._door_motion_error(
            required_floor_id=plan.target_floor.floor_id,
            label="target",
        )
        if error:
            return self._fail(error)
        progress = self._observe_goal(plan.target_floor.map_id, plan.target_lobby)
        if progress in {GoalProgress.PENDING, GoalProgress.EXECUTING}:
            return self._status
        if progress is not GoalProgress.SUCCEEDED:
            return self._fail(f"lift_exit_{progress.value}")
        self._motion_active = False
        released, reason = self._release_lift()
        if not released:
            return self._fail(reason or "lift_release_failed")
        self._set_terminal(LiftTransitionPhase.SUCCEEDED, "lift_transition_complete")
        return self._status

    def _send_goal(self, target: PoseTarget, *, suffix: str) -> bool:
        self._goal_request_id = f"{self._session_id}:{suffix}"
        try:
            self._navigation.send_goal(
                target.x,
                target.y,
                target.z,
                target.yaw,
                request_id=self._goal_request_id,
            )
        except Exception:
            self._fail(f"lift_{suffix}_goal_error")
            return False
        self._motion_active = True
        return True

    def _observe_goal(self, map_id: str, target: PoseTarget) -> GoalProgress:
        try:
            progress = self._navigation.observe_goal(
                request_id=self._goal_request_id,
                map_id=map_id,
                target=target,
            )
            return progress if isinstance(progress, GoalProgress) else GoalProgress(str(progress).lower())
        except (TypeError, ValueError):
            return GoalProgress.BLOCKED
        except Exception:
            return GoalProgress.FAILED

    def _request_lift(self, destination_floor_id: str) -> tuple[bool, str]:
        plan = self._plan
        if plan is None:
            return False, "lift_transition_plan_missing"
        try:
            accepted, reason = self._lift.request(
                lift_id=plan.lift_id,
                destination_floor_id=destination_floor_id,
                session_id=self._session_id,
            )
        except Exception:
            return False, "lift_request_error"
        if accepted is True:
            self._lift_claimed = True
            return True, str(reason or "")
        return False, str(reason or "")

    def _release_lift(self) -> tuple[bool, str]:
        if not self._lift_claimed or self._plan is None:
            return True, "lift_not_claimed"
        try:
            released, reason = self._lift.release(
                lift_id=self._plan.lift_id,
                session_id=self._session_id,
            )
        except Exception:
            return False, "lift_release_error"
        if released is True:
            self._lift_claimed = False
            return True, str(reason or "")
        return False, str(reason or "")

    def _validated_lift_state(self) -> tuple[LiftState | None, str]:
        plan = self._plan
        if plan is None:
            return None, "lift_transition_plan_missing"
        try:
            state = self._lift.snapshot(plan.lift_id)
        except Exception:
            return None, "lift_state_error"
        if state is None:
            return None, "lift_state_missing"
        if state.lift_id != plan.lift_id:
            return None, "lift_identity_mismatch"
        if not math.isfinite(float(state.stamp_s)):
            return None, "lift_state_timestamp_invalid"
        age_s = float(self._clock()) - float(state.stamp_s)
        if age_s < -0.5 or age_s > self._max_lift_state_age_s:
            return None, "lift_state_stale"
        if state.available is not True:
            return None, "lift_unavailable"
        if state.session_id != self._session_id:
            return None, "lift_session_mismatch"
        return state, ""

    def _door_motion_error(self, *, required_floor_id: str, label: str) -> str:
        state, error = self._validated_lift_state()
        if error:
            return error
        if state is None:
            return "lift_state_missing"
        if state.current_floor_id != required_floor_id:
            return f"{label}_lift_floor_changed"
        if self._normal(state.motion_state) != "stopped":
            return f"{label}_lift_not_stopped"
        if self._normal(state.door_state) != "open":
            return f"{label}_lift_door_not_open"
        return ""

    def _phase_timed_out(self) -> bool:
        timeout_s = self._ride_timeout_s if self._status.phase is LiftTransitionPhase.RIDE else self._step_timeout_s
        return float(self._clock()) - self._phase_started_s > timeout_s

    def _fail(self, reason: str) -> LiftTransitionStatus:
        self._stop_and_cancel(reason=f"lift_transition_abort:{reason}")
        released, release_reason = self._release_lift()
        if not released:
            reason += f";lift_release_pending:{release_reason or 'unconfirmed'}"
        self._set_terminal(LiftTransitionPhase.FAILED, reason)
        return self._status

    def _retry_release(self) -> LiftTransitionStatus:
        released, _reason = self._release_lift()
        if not released:
            self._set_terminal(self._status.phase, self._status.reason)
            return self._status
        self._status = LiftTransitionStatus(
            phase=self._status.phase,
            request_id=self._request_id,
            session_id=self._session_id,
            reason=f"{self._status.reason};lift_released_on_retry",
            release_pending=False,
        )
        return self._status

    def _stop_and_cancel(self, *, reason: str) -> None:
        if self._motion_active:
            try:
                self._navigation.cancel(
                    reason,
                    request_id=f"{self._session_id}:cancel",
                )
            except Exception:
                logger.debug("Failed to cancel lift-step navigation", exc_info=True)
        try:
            self._navigation.stop(
                reason,
                request_id=f"{self._session_id}:stop",
            )
        except Exception:
            logger.debug("Failed to publish lift transition stop", exc_info=True)
        self._motion_active = False

    def _set_phase(self, phase: LiftTransitionPhase, reason: str) -> None:
        self._phase_started_s = float(self._clock())
        self._status = LiftTransitionStatus(
            phase=phase,
            request_id=self._request_id,
            session_id=self._session_id,
            reason=reason,
            release_pending=False,
        )

    def _set_terminal(self, phase: LiftTransitionPhase, reason: str) -> None:
        self._motion_active = False
        self._status = LiftTransitionStatus(
            phase=phase,
            request_id=self._request_id,
            session_id=self._session_id,
            reason=reason,
            release_pending=self._lift_claimed,
        )

    def _status_with_reason(self, reason: str) -> LiftTransitionStatus:
        self._status = LiftTransitionStatus(
            phase=self._status.phase,
            request_id=self._request_id,
            session_id=self._session_id,
            reason=reason,
            release_pending=self._status.release_pending,
        )
        return self._status

    def _autonomy_ready(self) -> tuple[bool, str]:
        try:
            ready, reason = self._navigation.autonomy_ready()
        except Exception:
            return False, "native_autonomy_status_error"
        return ready is True, str(reason or "")

    @staticmethod
    def _normal(value: str) -> str:
        return str(value or "").strip().lower()

    @staticmethod
    def _validate_plan(plan: LiftTransitionPlan, *, request_id: str) -> str:
        if not isinstance(plan, LiftTransitionPlan):
            return "invalid_lift_transition_plan"
        if not str(request_id).strip():
            return "request_id_required"
        if not plan.lift_id.strip():
            return "lift_id_required"
        if plan.source_floor == plan.target_floor:
            return "lift_transition_requires_distinct_floors"
        if plan.source_floor.building_id != plan.target_floor.building_id:
            return "cross_building_lift_transition_unsupported"
        for target in (
            plan.source_lobby,
            plan.source_cabin,
            plan.target_cabin,
            plan.target_lobby,
        ):
            if target.frame_id != "map":
                return "unsupported_navigation_frame"
            if not all(math.isfinite(value) for value in (target.x, target.y, target.z, target.yaw)):
                return "invalid_navigation_target"
        return ""
