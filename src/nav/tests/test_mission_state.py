from nav.mission.model.state import (
    MissionEvent,
    MissionMode,
    MissionState,
    mission_mode_for_event,
    next_mission_state,
)


def test_mission_lifecycle_separates_patrol_mode_from_state() -> None:
    assert mission_mode_for_event(MissionEvent.PATROL_ACCEPTED) is MissionMode.PATROL
    assert next_mission_state(MissionState.IDLE, MissionEvent.PATROL_ACCEPTED) is MissionState.PLANNING
    assert next_mission_state(MissionState.PLANNING, MissionEvent.PLAN_OK) is MissionState.EXECUTING


def test_stuck_is_event_not_terminal_state() -> None:
    assert next_mission_state(MissionState.EXECUTING, MissionEvent.STUCK) is MissionState.RECOVERING
    assert next_mission_state(MissionState.RECOVERING, MissionEvent.RECOVERY_OK) is MissionState.PLANNING


def test_replan_and_waypoint_events_keep_lifecycle_small() -> None:
    assert next_mission_state(MissionState.EXECUTING, MissionEvent.REPLAN_REQUESTED) is MissionState.PLANNING
    assert next_mission_state(MissionState.EXECUTING, MissionEvent.WAYPOINT_REACHED) is MissionState.EXECUTING
