"""Mission lifecycle state for navigation."""

from __future__ import annotations

import time
from enum import Enum
from typing import Literal, TypedDict, cast


class MissionState(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    PAUSED = "PAUSED"
    RECOVERING = "RECOVERING"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"
    # Legacy public states. New mission flow uses mode=PATROL and
    # event=STUCK with lifecycle state EXECUTING/RECOVERING.
    PATROLLING = "PATROLLING"
    STUCK = "STUCK"


class MissionMode(str, Enum):
    NONE = "NONE"
    SINGLE_GOAL = "SINGLE_GOAL"
    PATROL = "PATROL"
    EXPLORE = "EXPLORE"
    EXTERNAL_PATH = "EXTERNAL_PATH"


class MissionEvent(str, Enum):
    GOAL_ACCEPTED = "GOAL_ACCEPTED"
    PATROL_ACCEPTED = "PATROL_ACCEPTED"
    EXPLORE_ACCEPTED = "EXPLORE_ACCEPTED"
    EXTERNAL_PATH_ACCEPTED = "EXTERNAL_PATH_ACCEPTED"
    REPLAN_REQUESTED = "REPLAN_REQUESTED"
    PLAN_OK = "PLAN_OK"
    PLAN_FAILED = "PLAN_FAILED"
    WAYPOINT_REACHED = "WAYPOINT_REACHED"
    PATH_COMPLETE = "PATH_COMPLETE"
    STUCK = "STUCK"
    RECOVERY_OK = "RECOVERY_OK"
    RECOVERY_FAILED = "RECOVERY_FAILED"
    PAUSE = "PAUSE"
    RESUME = "RESUME"
    CANCEL = "CANCEL"
    STOP = "STOP"
    FRAME_ERROR = "FRAME_ERROR"


MissionStateName = Literal[
    "IDLE",
    "PLANNING",
    "EXECUTING",
    "PAUSED",
    "RECOVERING",
    "SUCCESS",
    "FAILED",
    "CANCELLED",
    "PATROLLING",
    "STUCK",
]
MissionModeName = Literal["NONE", "SINGLE_GOAL", "PATROL", "EXPLORE", "EXTERNAL_PATH"]
MissionStateInput = MissionState | MissionStateName | str
MissionEventInput = MissionEvent | str

MISSION_TRANSITIONS: dict[MissionState, frozenset[MissionState]] = {
    MissionState.IDLE: frozenset(
        [
            MissionState.PLANNING,
            MissionState.EXECUTING,
            MissionState.FAILED,
            MissionState.CANCELLED,
            MissionState.PATROLLING,
        ]
    ),
    MissionState.PLANNING: frozenset(
        [
            MissionState.EXECUTING,
            MissionState.PAUSED,
            MissionState.FAILED,
            MissionState.IDLE,
            MissionState.CANCELLED,
            MissionState.PATROLLING,
        ]
    ),
    MissionState.EXECUTING: frozenset(
        [
            MissionState.SUCCESS,
            MissionState.FAILED,
            MissionState.RECOVERING,
            MissionState.PAUSED,
            MissionState.IDLE,
            MissionState.PLANNING,
            MissionState.CANCELLED,
            MissionState.PATROLLING,
            MissionState.STUCK,
        ]
    ),
    MissionState.PAUSED: frozenset(
        [
            MissionState.IDLE,
            MissionState.PLANNING,
            MissionState.EXECUTING,
            MissionState.RECOVERING,
            MissionState.FAILED,
            MissionState.CANCELLED,
        ]
    ),
    MissionState.RECOVERING: frozenset(
        [
            MissionState.IDLE,
            MissionState.PLANNING,
            MissionState.EXECUTING,
            MissionState.FAILED,
            MissionState.CANCELLED,
        ]
    ),
    MissionState.SUCCESS: frozenset(
        [
            MissionState.IDLE,
            MissionState.PLANNING,
            MissionState.PATROLLING,
        ]
    ),
    MissionState.FAILED: frozenset(
        [
            MissionState.IDLE,
            MissionState.PLANNING,
            MissionState.EXECUTING,
            MissionState.PATROLLING,
        ]
    ),
    MissionState.CANCELLED: frozenset(
        [
            MissionState.IDLE,
            MissionState.PLANNING,
            MissionState.EXECUTING,
            MissionState.PATROLLING,
        ]
    ),
    MissionState.PATROLLING: frozenset(
        [
            MissionState.EXECUTING,
            MissionState.SUCCESS,
            MissionState.FAILED,
            MissionState.RECOVERING,
            MissionState.PAUSED,
            MissionState.IDLE,
            MissionState.PLANNING,
            MissionState.CANCELLED,
            MissionState.STUCK,
        ]
    ),
    MissionState.STUCK: frozenset(
        [
            MissionState.IDLE,
            MissionState.PLANNING,
            MissionState.EXECUTING,
            MissionState.RECOVERING,
            MissionState.CANCELLED,
        ]
    ),
}

_START_MODES: dict[MissionEvent, MissionMode] = {
    MissionEvent.GOAL_ACCEPTED: MissionMode.SINGLE_GOAL,
    MissionEvent.PATROL_ACCEPTED: MissionMode.PATROL,
    MissionEvent.EXPLORE_ACCEPTED: MissionMode.EXPLORE,
    MissionEvent.EXTERNAL_PATH_ACCEPTED: MissionMode.EXTERNAL_PATH,
}


class MissionTransitionRejected(TypedDict):
    event: Literal["mission_state_transition_rejected"]
    current_state: str
    requested_state: str
    allowed_states: list[str]
    reason: str
    ts: float


def coerce_mission_state(state: MissionStateInput | None) -> MissionState | None:
    if state is None:
        return None
    if isinstance(state, MissionState):
        return state
    try:
        return MissionState(str(state))
    except ValueError:
        return None


def coerce_mission_event(event: MissionEventInput | None) -> MissionEvent | None:
    if event is None:
        return None
    if isinstance(event, MissionEvent):
        return event
    try:
        return MissionEvent(str(event))
    except ValueError:
        return None


def mission_state_value(state: MissionStateInput | None) -> str:
    parsed = coerce_mission_state(state)
    if parsed is not None:
        return parsed.value
    return "" if state is None else str(state)


def mission_state_name(state: MissionStateInput) -> MissionStateName:
    parsed = coerce_mission_state(state)
    if parsed is None:
        raise ValueError(f"invalid mission state: {state!r}")
    return cast(MissionStateName, parsed.value)


def mission_mode_for_event(event: MissionEventInput | None) -> MissionMode | None:
    parsed = coerce_mission_event(event)
    if parsed is None:
        return None
    return _START_MODES.get(parsed)


def next_mission_state(
    current: MissionStateInput,
    event: MissionEventInput,
    *,
    previous_state: MissionStateInput | None = None,
) -> MissionState:
    parsed_current = coerce_mission_state(current)
    parsed_event = coerce_mission_event(event)
    if parsed_current is None:
        raise ValueError(f"invalid mission state: {current!r}")
    if parsed_event is None:
        raise ValueError(f"invalid mission event: {event!r}")

    if parsed_event in _START_MODES:
        return MissionState.PLANNING
    if parsed_event is MissionEvent.REPLAN_REQUESTED:
        return MissionState.PLANNING
    if parsed_event is MissionEvent.PLAN_OK:
        return MissionState.EXECUTING
    if parsed_event in {MissionEvent.PLAN_FAILED, MissionEvent.FRAME_ERROR}:
        return MissionState.FAILED
    if parsed_event is MissionEvent.WAYPOINT_REACHED:
        return parsed_current
    if parsed_event is MissionEvent.PATH_COMPLETE:
        return MissionState.SUCCESS
    if parsed_event is MissionEvent.STUCK:
        return MissionState.RECOVERING
    if parsed_event is MissionEvent.RECOVERY_OK:
        return MissionState.PLANNING
    if parsed_event is MissionEvent.RECOVERY_FAILED:
        return MissionState.FAILED
    if parsed_event is MissionEvent.PAUSE:
        return MissionState.PAUSED
    if parsed_event is MissionEvent.RESUME:
        return coerce_mission_state(previous_state) or MissionState.EXECUTING
    if parsed_event is MissionEvent.CANCEL:
        return MissionState.CANCELLED
    if parsed_event is MissionEvent.STOP:
        return MissionState.IDLE
    raise ValueError(f"unhandled mission event: {event!r}")


def default_phase_reason(
    current_state: MissionStateInput,
    state: MissionStateInput,
) -> str:
    current_name = mission_state_value(current_state)
    state_name = mission_state_value(state)
    if state_name == current_name:
        return f"state refresh: {state_name}"
    defaults = {
        MissionState.IDLE: "mission idle or cleared",
        MissionState.PLANNING: "goal accepted; planning global path",
        MissionState.EXECUTING: "global path ready; tracking waypoint",
        MissionState.PAUSED: "mission paused",
        MissionState.RECOVERING: "stuck; running recovery",
        MissionState.SUCCESS: "path complete",
        MissionState.FAILED: "mission failed",
        MissionState.CANCELLED: "mission cancelled",
        MissionState.PATROLLING: "patrol path ready; tracking patrol waypoint",
        MissionState.STUCK: "stuck after recovery attempts",
    }
    parsed_state = coerce_mission_state(state)
    return defaults.get(parsed_state, f"{current_name} -> {state_name}")


def build_transition_rejection(
    *,
    current_state: MissionStateInput | None,
    requested_state: MissionStateInput | None,
    reason: str,
    allowed: frozenset[MissionState] | None = None,
    timestamp: float | None = None,
) -> MissionTransitionRejected:
    current = coerce_mission_state(current_state)
    allowed_states = sorted(
        mission_state_value(item)
        for item in (allowed if allowed is not None else MISSION_TRANSITIONS.get(current, frozenset()))
    )
    return {
        "event": "mission_state_transition_rejected",
        "current_state": mission_state_value(current_state),
        "requested_state": mission_state_value(requested_state),
        "allowed_states": allowed_states,
        "reason": reason,
        "ts": time.time() if timestamp is None else timestamp,
    }
