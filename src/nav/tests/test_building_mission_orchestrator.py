from __future__ import annotations

from nav.building import (
    ActiveFloor,
    BuildingMissionOrchestrator,
    BuildingMissionPhase,
    BuildingMissionRequest,
    GoalProgress,
    PoseTarget,
)


class FakeNavigationPort:
    def __init__(self) -> None:
        self.ready = (True, "autonomy_ready")
        self.progress = GoalProgress.PENDING
        self.goals: list[tuple[float, float, float, float, str | None]] = []
        self.cancels: list[tuple[str, str | None]] = []
        self.stops: list[tuple[str, str | None]] = []

    def autonomy_ready(self) -> tuple[bool, str]:
        return self.ready

    def send_goal(self, x, y, z, yaw, *, request_id=None) -> None:
        self.goals.append((x, y, z, yaw, request_id))

    def observe_goal(self, *, request_id, map_id, target):
        return self.progress

    def cancel(self, reason="cancel", *, request_id=None) -> None:
        self.cancels.append((reason, request_id))

    def stop(self, reason="stop", *, request_id=None) -> None:
        self.stops.append((reason, request_id))


class FakeTransitionPort:
    def __init__(self) -> None:
        self.starts = []
        self.progress = ("executing", "riding_to_target_floor")
        self.cancels: list[str] = []

    def start(self, request, *, source_floor):
        self.starts.append((request, source_floor))
        return True, "lift_transition_started"

    def tick(self):
        return self.progress

    def cancel(self, *, reason):
        self.cancels.append(reason)


def _mission(*, floor_id: str = "floor-1", map_id: str = "factory-a-floor-1") -> BuildingMissionRequest:
    return BuildingMissionRequest(
        request_id="building-task-1",
        source="native_test",
        fleet_name="",
        robot_name="thunder-01",
        building_id="factory-a",
        floor_id=floor_id,
        map_id=map_id,
        target=PoseTarget(frame_id="map", x=12.0, y=-3.0, z=0.0, yaw=0.5),
    )


def _active_floor() -> ActiveFloor:
    return ActiveFloor(
        building_id="factory-a",
        floor_id="floor-1",
        map_id="factory-a-floor-1",
    )


def test_same_floor_mission_dispatches_one_correlated_native_goal() -> None:
    navigation = FakeNavigationPort()
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=_active_floor,
    )

    accepted, reason = orchestrator.submit(_mission())

    assert (accepted, reason) == (True, "building_navigation_goal_accepted")
    assert navigation.goals == [(12.0, -3.0, 0.0, 0.5, "building-task-1:goal")]
    assert orchestrator.status().phase is BuildingMissionPhase.TARGET_NAVIGATION

    navigation.progress = GoalProgress.EXECUTING
    assert orchestrator.tick().phase is BuildingMissionPhase.TARGET_NAVIGATION

    navigation.progress = GoalProgress.SUCCEEDED
    status = orchestrator.tick()
    assert status.phase is BuildingMissionPhase.SUCCEEDED
    assert status.reason == "building_navigation_goal_reached"


def test_cross_floor_mission_fails_closed_without_transition_executor() -> None:
    navigation = FakeNavigationPort()
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=_active_floor,
    )

    accepted, reason = orchestrator.submit(
        _mission(floor_id="floor-2", map_id="factory-a-floor-2")
    )

    assert (accepted, reason) == (False, "floor_transition_executor_unavailable")
    assert navigation.goals == []
    assert orchestrator.status().phase is BuildingMissionPhase.FAILED


def test_cross_floor_mission_runs_transition_then_dispatches_target_goal() -> None:
    navigation = FakeNavigationPort()
    transition = FakeTransitionPort()
    active = [_active_floor()]
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=lambda: active[0],
        transition_executor=transition,
    )
    request = _mission(floor_id="floor-2", map_id="factory-a-floor-2")

    assert orchestrator.submit(request) == (True, "lift_transition_started")
    assert transition.starts == [(request, _active_floor())]
    assert orchestrator.tick().phase is BuildingMissionPhase.FLOOR_TRANSITION
    assert navigation.goals == []

    active[0] = request.target_floor
    transition.progress = ("succeeded", "lift_transition_complete")
    status = orchestrator.tick()

    assert status.phase is BuildingMissionPhase.TARGET_NAVIGATION
    assert navigation.goals == [(12.0, -3.0, 0.0, 0.5, "building-task-1:goal")]
    navigation.progress = GoalProgress.SUCCEEDED
    assert orchestrator.tick().phase is BuildingMissionPhase.SUCCEEDED


def test_cross_floor_transition_cannot_complete_without_target_floor_identity() -> None:
    navigation = FakeNavigationPort()
    transition = FakeTransitionPort()
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=_active_floor,
        transition_executor=transition,
    )
    assert orchestrator.submit(
        _mission(floor_id="floor-2", map_id="factory-a-floor-2")
    )[0]
    transition.progress = ("succeeded", "lift_transition_complete")

    status = orchestrator.tick()

    assert status.phase is BuildingMissionPhase.FAILED
    assert status.reason == "floor_transition_identity_unverified"
    assert navigation.goals == []
    assert transition.cancels == [
        "building_mission_abort:floor_transition_identity_unverified"
    ]


def test_building_mission_rejects_when_active_floor_is_unavailable() -> None:
    navigation = FakeNavigationPort()

    def unavailable_floor() -> ActiveFloor:
        raise RuntimeError("native active map unavailable")

    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=unavailable_floor,
    )

    assert orchestrator.submit(_mission()) == (False, "active_floor_unavailable")
    assert navigation.goals == []
    assert orchestrator.status().phase is BuildingMissionPhase.FAILED


def test_transition_aborts_when_target_floor_identity_becomes_unavailable() -> None:
    navigation = FakeNavigationPort()
    transition = FakeTransitionPort()
    floor_reads = [_active_floor()]

    def active_floor() -> ActiveFloor:
        if floor_reads:
            return floor_reads.pop(0)
        raise RuntimeError("native active map unavailable")

    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=active_floor,
        transition_executor=transition,
    )
    request = _mission(floor_id="floor-2", map_id="factory-a-floor-2")
    assert orchestrator.submit(request)[0] is True
    transition.progress = ("succeeded", "lift_transition_complete")

    status = orchestrator.tick()

    assert status.phase is BuildingMissionPhase.FAILED
    assert status.reason == "active_floor_unavailable"
    assert navigation.goals == []
    assert transition.cancels == [
        "building_mission_abort:active_floor_unavailable"
    ]


def test_building_mission_rejects_when_native_autonomy_is_not_ready() -> None:
    navigation = FakeNavigationPort()
    navigation.ready = (False, "native_control_mode_teleop_avoid")
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=_active_floor,
    )

    accepted, reason = orchestrator.submit(_mission())

    assert accepted is False
    assert reason == "autonomy_not_ready:native_control_mode_teleop_avoid"
    assert navigation.goals == []
    assert orchestrator.status().phase is BuildingMissionPhase.FAILED


def test_active_building_mission_aborts_on_control_authority_loss() -> None:
    navigation = FakeNavigationPort()
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=_active_floor,
    )
    assert orchestrator.submit(_mission())[0] is True
    navigation.progress = GoalProgress.EXECUTING
    orchestrator.tick()

    navigation.ready = (False, "operator_takeover_latched")
    status = orchestrator.tick()

    assert status.phase is BuildingMissionPhase.FAILED
    assert status.reason == "autonomy_not_ready:operator_takeover_latched"
    assert navigation.cancels == [
        ("building_mission_abort:autonomy_not_ready:operator_takeover_latched", "building-task-1:cancel")
    ]


def test_floor_transition_aborts_on_control_authority_loss() -> None:
    navigation = FakeNavigationPort()
    transition = FakeTransitionPort()
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=_active_floor,
        transition_executor=transition,
    )
    assert orchestrator.submit(
        _mission(floor_id="floor-2", map_id="factory-a-floor-2")
    )[0]
    navigation.ready = (False, "native_control_mode_teleop_avoid")

    status = orchestrator.tick()

    assert status.phase is BuildingMissionPhase.FAILED
    assert status.reason == "autonomy_not_ready:native_control_mode_teleop_avoid"
    assert transition.cancels == [
        "building_mission_abort:autonomy_not_ready:native_control_mode_teleop_avoid"
    ]
    assert navigation.goals == []


def test_building_mission_rejects_non_map_or_non_finite_target() -> None:
    navigation = FakeNavigationPort()
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=_active_floor,
    )
    invalid_frame = BuildingMissionRequest(
        **{**_mission().__dict__, "target": PoseTarget("odom", 1.0, 2.0, 0.0, 0.0)}
    )
    non_finite = BuildingMissionRequest(
        **{**_mission().__dict__, "target": PoseTarget("map", float("nan"), 2.0, 0.0, 0.0)}
    )

    assert orchestrator.submit(invalid_frame) == (False, "unsupported_navigation_frame")
    assert orchestrator.submit(non_finite) == (False, "invalid_navigation_target")
    assert navigation.goals == []
