from __future__ import annotations

import pytest

from nav.building import (
    ActiveFloor,
    BuildingMissionRequest,
    GoalProgress,
    LiftState,
    LiftTransitionExecutor,
    LiftTransitionPhase,
    LiftTransitionPlan,
    LiftTransitionService,
    PoseTarget,
    StaticLiftTransitionCatalog,
)


class FakeNavigationPort:
    def __init__(self) -> None:
        self.ready = (True, "autonomy_ready")
        self.progress: dict[str, GoalProgress] = {}
        self.goals: list[tuple[float, float, float, float, str | None]] = []
        self.cancels: list[tuple[str, str | None]] = []
        self.stops: list[tuple[str, str | None]] = []

    def autonomy_ready(self) -> tuple[bool, str]:
        return self.ready

    def send_goal(self, x, y, z, yaw, *, request_id=None) -> None:
        self.goals.append((x, y, z, yaw, request_id))

    def observe_goal(self, *, request_id, map_id, target):
        return self.progress.get(request_id, GoalProgress.PENDING)

    def cancel(self, reason="cancel", *, request_id=None) -> None:
        self.cancels.append((reason, request_id))

    def stop(self, reason="stop", *, request_id=None) -> None:
        self.stops.append((reason, request_id))


class FakeLiftPort:
    def __init__(self) -> None:
        self.state: LiftState | None = None
        self.requests: list[tuple[str, str, str]] = []
        self.releases: list[tuple[str, str]] = []
        self.request_ack = True
        self.release_ok = True

    def request(self, *, lift_id, destination_floor_id, session_id):
        self.requests.append((lift_id, destination_floor_id, session_id))
        return self.request_ack, "lift_request_accepted"

    def release(self, *, lift_id, session_id):
        self.releases.append((lift_id, session_id))
        return self.release_ok, "lift_released" if self.release_ok else "lift_release_unconfirmed"

    def snapshot(self, lift_id):
        assert lift_id == "lift-a"
        return self.state


class FakeFloorLocalizationPort:
    def __init__(self, floor: ActiveFloor) -> None:
        self.floor = floor
        self.localized = True
        self.switch_ok = True
        self.switch_calls: list[tuple[ActiveFloor, PoseTarget]] = []

    def active_floor(self) -> ActiveFloor:
        return self.floor

    def switch_and_relocalize(self, floor: ActiveFloor, seed: PoseTarget):
        self.switch_calls.append((floor, seed))
        if not self.switch_ok:
            return False, "native_relocalization_failed"
        self.floor = floor
        self.localized = False
        return True, "native_relocalization_requested"

    def is_localized(self, floor: ActiveFloor) -> bool:
        return self.floor == floor and self.localized


def _floor(number: int) -> ActiveFloor:
    return ActiveFloor(
        building_id="factory-a",
        floor_id=f"floor-{number}",
        map_id=f"factory-a-floor-{number}",
    )


def _pose(x: float, y: float, z: float = 0.0) -> PoseTarget:
    return PoseTarget(frame_id="map", x=x, y=y, z=z, yaw=0.0)


def _plan(*, reverse: bool = False) -> LiftTransitionPlan:
    source_floor = _floor(2) if reverse else _floor(1)
    target_floor = _floor(1) if reverse else _floor(2)
    source_z = 3.6 if reverse else 0.0
    target_z = 0.0 if reverse else 3.6
    return LiftTransitionPlan(
        lift_id="lift-a",
        source_floor=source_floor,
        target_floor=target_floor,
        source_lobby=_pose(4.0, 1.0, source_z),
        source_cabin=_pose(5.0, 1.0, source_z),
        target_cabin=_pose(5.0, 1.0, target_z),
        target_lobby=_pose(6.0, 1.0, target_z),
    )


def _executor(clock=lambda: 100.0, *, initial_floor: ActiveFloor | None = None):
    navigation = FakeNavigationPort()
    lift = FakeLiftPort()
    floors = FakeFloorLocalizationPort(initial_floor or _floor(1))
    executor = LiftTransitionExecutor(
        navigation=navigation,
        lift=lift,
        floor_localization=floors,
        clock=clock,
        max_lift_state_age_s=1.0,
        step_timeout_s=30.0,
        ride_timeout_s=120.0,
    )
    return executor, navigation, lift, floors


def _lift_state(
    *,
    floor_id: str,
    session_id: str = "building-task-1:lift",
    door_state: str = "open",
    motion_state: str = "stopped",
    stamp_s: float = 100.0,
    available: bool = True,
) -> LiftState:
    return LiftState(
        lift_id="lift-a",
        available=available,
        current_floor_id=floor_id,
        destination_floor_id=floor_id,
        door_state=door_state,
        motion_state=motion_state,
        session_id=session_id,
        stamp_s=stamp_s,
    )


def test_lift_transition_runs_approach_call_enter_ride_relocalize_exit() -> None:
    executor, navigation, lift, floors = _executor()

    assert executor.start(_plan(), request_id="building-task-1") == (
        True,
        "lift_transition_started",
    )
    assert executor.status().phase is LiftTransitionPhase.APPROACH_SOURCE
    assert navigation.goals[-1][-1] == "building-task-1:lift:approach"

    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    assert executor.tick().phase is LiftTransitionPhase.WAIT_SOURCE_DOOR
    assert lift.requests == [("lift-a", "floor-1", "building-task-1:lift")]

    lift.state = _lift_state(floor_id="floor-1")
    assert executor.tick().phase is LiftTransitionPhase.ENTER_CABIN
    assert navigation.goals[-1][-1] == "building-task-1:lift:enter"

    navigation.progress["building-task-1:lift:enter"] = GoalProgress.SUCCEEDED
    assert executor.tick().phase is LiftTransitionPhase.RIDE
    assert navigation.stops == [("lift_ride_hold", "building-task-1:lift:hold")]
    assert lift.requests[-1] == ("lift-a", "floor-2", "building-task-1:lift")

    lift.state = _lift_state(floor_id="floor-2")
    assert executor.tick().phase is LiftTransitionPhase.VERIFY_TARGET_LOCALIZATION
    assert floors.switch_calls == [(_floor(2), _plan().target_cabin)]

    floors.localized = True
    assert executor.tick().phase is LiftTransitionPhase.EXIT_CABIN
    assert navigation.goals[-1][-1] == "building-task-1:lift:exit"

    navigation.progress["building-task-1:lift:exit"] = GoalProgress.SUCCEEDED
    status = executor.tick()
    assert status.phase is LiftTransitionPhase.SUCCEEDED
    assert status.reason == "lift_transition_complete"
    assert lift.releases == [("lift-a", "building-task-1:lift")]


@pytest.mark.parametrize("malformed_ready", ["true", 1, None])
def test_lift_transition_rejects_malformed_autonomy_readiness(
    malformed_ready,
) -> None:
    executor, navigation, _lift, _floors = _executor()
    navigation.ready = (malformed_ready, "malformed_autonomy_readiness")

    accepted, reason = executor.start(_plan(), request_id="building-task-1")

    assert accepted is False
    assert reason == "autonomy_not_ready:malformed_autonomy_readiness"
    assert navigation.goals == []


@pytest.mark.parametrize("malformed_ack", ["true", 1, None])
def test_lift_transition_rejects_malformed_lift_request_ack(
    malformed_ack,
) -> None:
    executor, navigation, lift, _floors = _executor()
    lift.request_ack = malformed_ack
    assert executor.start(_plan(), request_id="building-task-1")[0] is True
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED

    status = executor.tick()

    assert status.phase is LiftTransitionPhase.FAILED
    assert "lift_request_accepted" in status.reason
    assert all(goal[-1] != "building-task-1:lift:enter" for goal in navigation.goals)


@pytest.mark.parametrize("malformed_ack", ["true", 1, None])
def test_lift_transition_keeps_release_pending_for_malformed_release_ack(
    malformed_ack,
) -> None:
    executor, navigation, lift, _floors = _executor()
    assert executor.start(_plan(), request_id="building-task-1")[0] is True
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    assert executor.tick().phase is LiftTransitionPhase.WAIT_SOURCE_DOOR
    lift.release_ok = malformed_ack

    status = executor.cancel(reason="operator_cancel")

    assert status.phase is LiftTransitionPhase.CANCELLED
    assert status.release_pending is True
    assert "lift_release_pending" in status.reason


def test_reverse_lift_transition_runs_floor_two_to_floor_one() -> None:
    plan = _plan(reverse=True)
    executor, navigation, lift, floors = _executor(initial_floor=_floor(2))

    assert executor.start(plan, request_id="building-task-1") == (
        True,
        "lift_transition_started",
    )
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    assert executor.tick().phase is LiftTransitionPhase.WAIT_SOURCE_DOOR
    assert lift.requests[-1] == ("lift-a", "floor-2", "building-task-1:lift")

    lift.state = _lift_state(floor_id="floor-2")
    assert executor.tick().phase is LiftTransitionPhase.ENTER_CABIN
    navigation.progress["building-task-1:lift:enter"] = GoalProgress.SUCCEEDED
    assert executor.tick().phase is LiftTransitionPhase.RIDE
    assert lift.requests[-1] == ("lift-a", "floor-1", "building-task-1:lift")

    lift.state = _lift_state(floor_id="floor-1")
    assert executor.tick().phase is LiftTransitionPhase.VERIFY_TARGET_LOCALIZATION
    assert floors.switch_calls == [(_floor(1), plan.target_cabin)]
    floors.localized = True
    assert executor.tick().phase is LiftTransitionPhase.EXIT_CABIN
    navigation.progress["building-task-1:lift:exit"] = GoalProgress.SUCCEEDED
    assert executor.tick().phase is LiftTransitionPhase.SUCCEEDED
    assert floors.floor == _floor(1)
    assert lift.releases == [("lift-a", "building-task-1:lift")]


def test_lift_transition_fails_closed_on_stale_lift_state() -> None:
    executor, navigation, lift, _floors = _executor()
    executor.start(_plan(), request_id="building-task-1")
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    executor.tick()
    lift.state = _lift_state(floor_id="floor-1", stamp_s=90.0)

    status = executor.tick()

    assert status.phase is LiftTransitionPhase.FAILED
    assert status.reason == "lift_state_stale"
    assert all(goal[-1] != "building-task-1:lift:enter" for goal in navigation.goals)
    assert lift.releases == [("lift-a", "building-task-1:lift")]


@pytest.mark.parametrize(
    ("state", "expected_reason"),
    [
        (None, "lift_state_missing"),
        (_lift_state(floor_id="floor-1", available=False), "lift_unavailable"),
        (_lift_state(floor_id="floor-1", session_id="other-session"), "lift_session_mismatch"),
    ],
)
def test_lift_transition_fails_closed_on_missing_unavailable_or_hijacked_state(
    state: LiftState | None,
    expected_reason: str,
) -> None:
    executor, navigation, lift, _floors = _executor()
    executor.start(_plan(), request_id="building-task-1")
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    executor.tick()
    lift.state = state

    status = executor.tick()

    assert status.phase is LiftTransitionPhase.FAILED
    assert status.reason == expected_reason
    assert lift.releases == [("lift-a", "building-task-1:lift")]


@pytest.mark.parametrize("malformed_available", ["true", 1, None])
def test_lift_transition_rejects_malformed_lift_availability(
    malformed_available,
) -> None:
    executor, navigation, lift, _floors = _executor()
    assert executor.start(_plan(), request_id="building-task-1")[0] is True
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    assert executor.tick().phase is LiftTransitionPhase.WAIT_SOURCE_DOOR
    lift.state = _lift_state(
        floor_id="floor-1",
        available=malformed_available,
    )

    status = executor.tick()

    assert status.phase is LiftTransitionPhase.FAILED
    assert status.reason == "lift_unavailable"
    assert all(goal[-1] != "building-task-1:lift:enter" for goal in navigation.goals)


def test_lift_transition_aborts_if_door_closes_while_robot_enters() -> None:
    executor, navigation, lift, _floors = _executor()
    executor.start(_plan(), request_id="building-task-1")
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    executor.tick()
    lift.state = _lift_state(floor_id="floor-1")
    executor.tick()
    lift.state = _lift_state(floor_id="floor-1", door_state="closed")

    status = executor.tick()

    assert status.phase is LiftTransitionPhase.FAILED
    assert status.reason == "source_lift_door_not_open"
    assert navigation.cancels[-1][1] == "building-task-1:lift:cancel"


def test_lift_transition_does_not_exit_until_target_localization_is_verified() -> None:
    executor, navigation, lift, floors = _executor()
    executor.start(_plan(), request_id="building-task-1")
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    executor.tick()
    lift.state = _lift_state(floor_id="floor-1")
    executor.tick()
    navigation.progress["building-task-1:lift:enter"] = GoalProgress.SUCCEEDED
    executor.tick()
    lift.state = _lift_state(floor_id="floor-2")
    executor.tick()

    status = executor.tick()

    assert status.phase is LiftTransitionPhase.VERIFY_TARGET_LOCALIZATION
    assert all(goal[-1] != "building-task-1:lift:exit" for goal in navigation.goals)
    assert floors.localized is False

    floors.localized = True
    assert executor.tick().phase is LiftTransitionPhase.EXIT_CABIN
    assert navigation.goals[-1][-1] == "building-task-1:lift:exit"


@pytest.mark.parametrize("malformed_ack", ["true", 1, None])
def test_lift_transition_rejects_malformed_relocalization_ack(
    malformed_ack,
) -> None:
    executor, navigation, lift, floors = _executor()
    assert executor.start(_plan(), request_id="building-task-1")[0] is True
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    assert executor.tick().phase is LiftTransitionPhase.WAIT_SOURCE_DOOR
    lift.state = _lift_state(floor_id="floor-1")
    assert executor.tick().phase is LiftTransitionPhase.ENTER_CABIN
    navigation.progress["building-task-1:lift:enter"] = GoalProgress.SUCCEEDED
    assert executor.tick().phase is LiftTransitionPhase.RIDE

    def switch_and_relocalize(floor, seed):
        floors.switch_calls.append((floor, seed))
        return malformed_ack, "malformed_relocalization_ack"

    floors.switch_and_relocalize = switch_and_relocalize
    lift.state = _lift_state(floor_id="floor-2")

    status = executor.tick()

    assert status.phase is LiftTransitionPhase.FAILED
    assert "malformed_relocalization_ack" in status.reason
    assert all(goal[-1] != "building-task-1:lift:exit" for goal in navigation.goals)


@pytest.mark.parametrize("malformed_status", ["true", 1, None])
def test_lift_transition_does_not_treat_malformed_localization_status_as_verified(
    malformed_status,
) -> None:
    executor, navigation, lift, floors = _executor()
    assert executor.start(_plan(), request_id="building-task-1")[0] is True
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    assert executor.tick().phase is LiftTransitionPhase.WAIT_SOURCE_DOOR
    lift.state = _lift_state(floor_id="floor-1")
    assert executor.tick().phase is LiftTransitionPhase.ENTER_CABIN
    navigation.progress["building-task-1:lift:enter"] = GoalProgress.SUCCEEDED
    assert executor.tick().phase is LiftTransitionPhase.RIDE
    lift.state = _lift_state(floor_id="floor-2")
    assert executor.tick().phase is LiftTransitionPhase.VERIFY_TARGET_LOCALIZATION
    floors.localized = malformed_status

    status = executor.tick()

    assert status.phase is LiftTransitionPhase.VERIFY_TARGET_LOCALIZATION
    assert all(goal[-1] != "building-task-1:lift:exit" for goal in navigation.goals)


@pytest.mark.parametrize(
    ("door_state", "motion_state", "expected_reason"),
    [
        ("closed", "stopped", "target_lift_door_not_open"),
        ("open", "moving", "target_lift_not_stopped"),
    ],
)
def test_lift_transition_aborts_if_target_lift_becomes_unsafe_during_exit(
    door_state: str,
    motion_state: str,
    expected_reason: str,
) -> None:
    executor, navigation, lift, floors = _executor()
    executor.start(_plan(), request_id="building-task-1")
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    executor.tick()
    lift.state = _lift_state(floor_id="floor-1")
    executor.tick()
    navigation.progress["building-task-1:lift:enter"] = GoalProgress.SUCCEEDED
    executor.tick()
    lift.state = _lift_state(floor_id="floor-2")
    executor.tick()
    floors.localized = True
    assert executor.tick().phase is LiftTransitionPhase.EXIT_CABIN
    lift.state = _lift_state(
        floor_id="floor-2",
        door_state=door_state,
        motion_state=motion_state,
    )

    status = executor.tick()

    assert status.phase is LiftTransitionPhase.FAILED
    assert status.reason == expected_reason
    assert navigation.cancels[-1][1] == "building-task-1:lift:cancel"


def test_lift_transition_map_switch_failure_never_dispatches_exit_goal() -> None:
    executor, navigation, lift, floors = _executor()
    executor.start(_plan(), request_id="building-task-1")
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    executor.tick()
    lift.state = _lift_state(floor_id="floor-1")
    executor.tick()
    navigation.progress["building-task-1:lift:enter"] = GoalProgress.SUCCEEDED
    executor.tick()
    floors.switch_ok = False
    lift.state = _lift_state(floor_id="floor-2")

    status = executor.tick()

    assert status.phase is LiftTransitionPhase.FAILED
    assert status.reason == "native_relocalization_failed"
    assert all(goal[-1] != "building-task-1:lift:exit" for goal in navigation.goals)


def test_lift_transition_aborts_on_native_mode_loss_during_ride() -> None:
    executor, navigation, lift, _floors = _executor()
    executor.start(_plan(), request_id="building-task-1")
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    executor.tick()
    lift.state = _lift_state(floor_id="floor-1")
    executor.tick()
    navigation.progress["building-task-1:lift:enter"] = GoalProgress.SUCCEEDED
    executor.tick()
    navigation.ready = (False, "native_control_mode_teleop")

    status = executor.tick()

    assert status.phase is LiftTransitionPhase.FAILED
    assert status.reason == "autonomy_not_ready:native_control_mode_teleop"
    assert lift.releases == [("lift-a", "building-task-1:lift")]


def test_failed_transition_reports_and_retries_lift_release_pending() -> None:
    executor, navigation, lift, _floors = _executor()
    executor.start(_plan(), request_id="building-task-1")
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    executor.tick()
    lift.release_ok = False
    lift.state = _lift_state(floor_id="floor-1", stamp_s=90.0)

    status = executor.tick()

    assert status.phase is LiftTransitionPhase.FAILED
    assert status.release_pending is True
    assert "lift_release_pending" in status.reason

    lift.release_ok = True
    status = executor.tick()
    assert status.phase is LiftTransitionPhase.FAILED
    assert status.release_pending is False
    assert "lift_released_on_retry" in status.reason


def test_lift_ride_timeout_stops_and_releases_session() -> None:
    now = [100.0]
    executor, navigation, lift, _floors = _executor(clock=lambda: now[0])
    executor.start(_plan(), request_id="building-task-1")
    navigation.progress["building-task-1:lift:approach"] = GoalProgress.SUCCEEDED
    executor.tick()
    lift.state = _lift_state(floor_id="floor-1", stamp_s=100.0)
    executor.tick()
    navigation.progress["building-task-1:lift:enter"] = GoalProgress.SUCCEEDED
    executor.tick()

    now[0] = 221.0
    status = executor.tick()

    assert status.phase is LiftTransitionPhase.FAILED
    assert status.reason == "ride_timeout"
    assert lift.releases == [("lift-a", "building-task-1:lift")]


def test_lift_transition_service_resolves_only_configured_directed_route() -> None:
    executor, _navigation, _lift, _floors = _executor()
    service = LiftTransitionService(
        catalog=StaticLiftTransitionCatalog([_plan()]),
        executor=executor,
    )
    request = BuildingMissionRequest(
        request_id="building-task-1",
        source="native_test",
        fleet_name="",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-2",
        map_id="factory-a-floor-2",
        target=_pose(12.0, -3.0, 3.6),
    )

    assert service.start(request, source_floor=_floor(1)) == (
        True,
        "lift_transition_started",
    )

    reverse = BuildingMissionRequest(
        **{
            **request.__dict__,
            "request_id": "building-task-reverse",
            "floor_id": "floor-1",
            "map_id": "factory-a-floor-1",
        }
    )
    reverse_service = LiftTransitionService(
        catalog=StaticLiftTransitionCatalog([_plan()]),
        executor=_executor()[0],
    )
    assert reverse_service.start(reverse, source_floor=_floor(2)) == (
        False,
        "floor_transition_route_unavailable",
    )


@pytest.mark.parametrize("malformed_ack", ["true", 1, None])
def test_lift_transition_service_rejects_malformed_executor_start_ack(
    malformed_ack,
) -> None:
    class MalformedExecutor:
        def start(self, plan, *, request_id):
            return malformed_ack, "malformed_transition_ack"

    service = LiftTransitionService(
        catalog=StaticLiftTransitionCatalog([_plan()]),
        executor=MalformedExecutor(),
    )
    request = BuildingMissionRequest(
        request_id="building-task-1",
        source="native_test",
        fleet_name="",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id="floor-2",
        map_id="factory-a-floor-2",
        target=_pose(12.0, -3.0, 3.6),
    )

    accepted, reason = service.start(request, source_floor=_floor(1))

    assert accepted is False
    assert reason == "malformed_transition_ack"
