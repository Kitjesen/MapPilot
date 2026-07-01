"""Mission FSM runtime helpers for Navigation."""

from __future__ import annotations

import logging

from nav.mission.model.state import (
    MISSION_TRANSITIONS,
    MissionEvent,
    MissionEventInput,
    MissionMode,
    MissionState,
    MissionStateInput,
    build_transition_rejection,
    coerce_mission_event,
    coerce_mission_state,
    default_phase_reason,
    mission_mode_for_event,
    mission_state_name,
    next_mission_state,
)
from nav.mission.model.status import MissionStatus, build_mission_status
from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


class NavigationFsmMixin:
    def _set_state(self, state: MissionStateInput, *, reason: str = "") -> bool:
        # Snapshot shared state under lock for cross-thread consistency
        with self._nav_lock:
            current_state_raw = self._state
            replan_count = self._replan_count
            goal = self._goal
            failure_reason = self._failure_reason

        current_state = coerce_mission_state(current_state_raw)
        requested_state = coerce_mission_state(state)
        if requested_state is None:
            self._reject_state_transition(
                current_state_raw,
                state,
                reason=reason or "unknown mission state",
            )
            return False
        if current_state is None:
            self._reject_state_transition(
                current_state_raw,
                requested_state,
                reason="invalid current mission state",
            )
            return False

        allowed = MISSION_TRANSITIONS.get(current_state, frozenset())
        if requested_state != current_state and requested_state not in allowed:
            self._reject_state_transition(
                current_state,
                requested_state,
                reason=reason or "illegal mission state transition",
                allowed=allowed,
            )
            return False

        phase_reason = (
            reason.strip()
            if isinstance(reason, str) and reason.strip()
            else default_phase_reason(current_state, requested_state)
        )

        # Write new state under lock
        with self._nav_lock:
            self._state = requested_state
            self._phase_reason = phase_reason
            self._last_rejected_transition = None

        self._publish_state_status(
            requested_state,
            phase_reason=phase_reason,
            replan_count=replan_count,
            goal=goal,
            failure_reason=failure_reason,
        )
        return True

    def _apply_event(self, event: MissionEventInput, *, reason: str = "") -> bool:
        parsed_event = coerce_mission_event(event)
        if parsed_event is None:
            self._reject_state_transition(
                self._get_state(),
                None,
                reason=f"unknown mission event: {event!r}",
            )
            return False
        with self._nav_lock:
            current_state = self._state
            previous_state = self._previous_state

        try:
            next_state = next_mission_state(
                current_state,
                parsed_event,
                previous_state=previous_state,
            )
        except ValueError as exc:
            self._reject_state_transition(
                current_state,
                None,
                reason=str(exc),
            )
            return False

        with self._nav_lock:
            mode = mission_mode_for_event(parsed_event)
            if mode is not None:
                self._mission_mode = mode
            if parsed_event is MissionEvent.STOP:
                self._mission_mode = MissionMode.NONE
            if parsed_event is MissionEvent.PAUSE:
                self._previous_state = current_state
            elif parsed_event in {
                MissionEvent.RESUME,
                MissionEvent.CANCEL,
                MissionEvent.STOP,
                MissionEvent.PATH_COMPLETE,
                MissionEvent.PLAN_FAILED,
                MissionEvent.RECOVERY_FAILED,
                MissionEvent.FRAME_ERROR,
            }:
                self._previous_state = None

        return self._set_state(
            next_state,
            reason=reason or parsed_event.value.lower(),
        )

    def _publish_state_status(
        self,
        state: MissionStateInput,
        *,
        phase_reason: str,
        replan_count: int,
        goal: np.ndarray | None,
        failure_reason: str,
    ) -> None:
        self.planner_status.publish(mission_state_name(state))
        status: MissionStatus = build_mission_status(
            state=state,
            mission_mode=self._mission_mode,
            phase_reason=phase_reason,
            last_rejected_transition=self._last_rejected_transition,
            replan_count=replan_count,
            tracker=self._tracker,
            planning_frame_id=self._planning_frame_id,
            odom_frame_id=self._odom_frame_id,
            costmap_frame_id=self._costmap_frame_id,
            goal_frame_id=self._goal_frame_id,
            goal=goal,
            robot_pos=self._robot_pos,
            speed_scale=self._speed_scale,
            speed_policy_reason=self._speed_policy_reason,
            planner_svc=self._planner_svc,
            replan_on_costmap_update=self._replan_on_costmap_update,
            last_plan_report=self._current_plan_report(),
            global_plan=self._last_global_plan,
            direct_goal_fallback=self._direct_goal_fallback_status,
            path_start_anchor=self._path_start_anchor_status,
            allow_path_start_insert=self._allow_path_start_insert,
            external_strategy_path_control=self._external_strategy_path_control,
            using_external_strategy_path=self._using_external_strategy_path,
            accept_partial_goal_progress=self._accept_partial_goal_progress,
            degeneracy=self._degen_level,
            failure_reason=failure_reason,
            localization_paused=self._paused_for_localization,
            localization_recovery_motion_hold=self._localization_recovery_motion_hold,
        )
        self.mission_status.publish(status)

    def force_state(self, state: MissionStateInput, *, reason: str) -> bool:
        """Force a mission state for diagnostics and explicit recovery code."""

        requested_state = coerce_mission_state(state)
        if requested_state is None:
            self._reject_state_transition(
                self._get_state(),
                state,
                reason=reason or "unknown forced mission state",
            )
            return False

        phase_reason = (
            f"force_state: {reason.strip()}"
            if isinstance(reason, str) and reason.strip()
            else "force_state"
        )
        with self._nav_lock:
            self._state = requested_state
            self._phase_reason = phase_reason
            self._last_rejected_transition = None
            replan_count = self._replan_count
            goal = self._goal
            failure_reason = self._failure_reason

        self._publish_state_status(
            requested_state,
            phase_reason=phase_reason,
            replan_count=replan_count,
            goal=goal,
            failure_reason=failure_reason,
        )
        return True

    def _reject_state_transition(
        self,
        current_state: MissionStateInput | None,
        requested_state: MissionStateInput | None,
        *,
        reason: str,
        allowed: frozenset[MissionState] | None = None,
    ) -> None:
        payload = build_transition_rejection(
            current_state=current_state,
            requested_state=requested_state,
            reason=reason,
            allowed=allowed,
        )
        with self._nav_lock:
            self._last_rejected_transition = payload
        logger.warning(
            "Navigation: rejected illegal state transition %s -> %s "
            "(allowed: %s, reason=%s)",
            payload["current_state"],
            payload["requested_state"],
            payload["allowed_states"],
            reason,
        )
        self.adapter_status.publish(payload)

    def _get_state(self) -> MissionState:
        """Thread-safe read of current mission state.

        Acquires the nav lock to ensure a consistent snapshot of self._state.
        Use this everywhere outside _set_state() to avoid cross-thread races
        with the recovery motion thread and plan preview executor.
        """
        with self._nav_lock:
            parsed_state = coerce_mission_state(self._state)
        if parsed_state is None:
            raise ValueError(f"invalid mission state: {self._state!r}")
        return parsed_state

    def _set_state_locked(self, new_state: MissionStateInput) -> None:
        """Thread-safe write of mission state without publish side-effects.

        Only sets the internal _state field under lock. For full FSM transitions
        (including legality checks and port publishing), use _set_state() instead.
        """
        parsed_state = coerce_mission_state(new_state)
        if parsed_state is None:
            raise ValueError(f"invalid mission state: {new_state!r}")
        with self._nav_lock:
            self._state = parsed_state

    def _get_goal(self) -> np.ndarray | None:
        """Thread-safe read of current navigation goal."""
        with self._nav_lock:
            _g = self._goal
            return _g.copy() if _g is not None else None

    def _set_goal(self, goal: np.ndarray | None) -> None:
        """Thread-safe write of navigation goal."""
        with self._nav_lock:
            self._goal = goal

    def _get_replan_count(self) -> int:
        """Thread-safe read of replan count."""
        with self._nav_lock:
            return self._replan_count

    def _set_replan_count(self, value: int) -> None:
        """Thread-safe write of replan count."""
        with self._nav_lock:
            self._replan_count = value

    def _is_path_execution_state(self) -> bool:
        return self._get_state() in (
            MissionState.EXECUTING,
            MissionState.PATROLLING,
        )

    def _is_planning_or_path_state(self) -> bool:
        return self._get_state() in (
            MissionState.PLANNING,
            MissionState.EXECUTING,
            MissionState.PATROLLING,
        )

    def _get_failure_reason(self) -> str:
        """Thread-safe read of failure reason."""
        with self._nav_lock:
            return self._failure_reason

    def _set_failure_reason(self, reason: str) -> None:
        """Thread-safe write of failure reason."""
        with self._nav_lock:
            self._failure_reason = reason
