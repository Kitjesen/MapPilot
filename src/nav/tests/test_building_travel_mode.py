from __future__ import annotations

import pytest

from nav.building import (
    ActiveFloor,
    BuildingMissionOrchestrator,
    BuildingMissionPhase,
    BuildingMissionRequest,
    LiftTransitionPlan,
    LiftTransitionService,
    PoseTarget,
    StaticLiftTransitionCatalog,
)


class FakeNavigation:
    def __init__(self) -> None:
        self.goals: list[tuple[float, float, float, float, str | None]] = []

    def autonomy_ready(self) -> tuple[bool, str]:
        return True, "autonomy_ready"

    def send_goal(self, x, y, z, yaw, *, request_id=None) -> None:
        self.goals.append((x, y, z, yaw, request_id))

    def observe_goal(self, *, request_id, map_id, target):
        return "pending"

    def cancel(self, reason="cancel", *, request_id=None) -> None:
        return None

    def stop(self, reason="stop", *, request_id=None) -> None:
        return None


class FakeLiftExecutor:
    def __init__(self) -> None:
        self.starts: list[tuple[LiftTransitionPlan, str]] = []

    def start(self, plan: LiftTransitionPlan, *, request_id: str) -> tuple[bool, str]:
        self.starts.append((plan, request_id))
        return True, "lift_transition_started"


class FakeTransition:
    def __init__(self) -> None:
        self.starts: list[tuple[BuildingMissionRequest, ActiveFloor]] = []
        self.progress = ("executing", "transition_in_progress")

    def start(
        self,
        request: BuildingMissionRequest,
        *,
        source_floor: ActiveFloor,
    ) -> tuple[bool, str]:
        self.starts.append((request, source_floor))
        return True, "floor_transition_started"

    def tick(self) -> tuple[str, str]:
        return self.progress

    def cancel(self, *, reason: str) -> None:
        return None


def _floor(number: int) -> ActiveFloor:
    return ActiveFloor(
        building_id="factory-a",
        floor_id=f"floor-{number}",
        map_id=f"factory-a-floor-{number}",
    )


def _pose(x: float = 1.0, *, z: float = 0.0) -> PoseTarget:
    return PoseTarget(frame_id="map", x=x, y=2.0, z=z, yaw=0.0)


def _mission(
    *,
    floor: ActiveFloor | None = None,
    travel_mode: str = "any",
    connector_id: str = "",
    map_version: int | None = None,
    version_id: str = "",
    map_pcd_sha256: str = "",
) -> BuildingMissionRequest:
    target_floor = floor or _floor(1)
    return BuildingMissionRequest(
        request_id="building-task-1",
        source="native_test",
        fleet_name="",
        robot_name="thunder-01",
        building_id=target_floor.building_id,
        floor_id=target_floor.floor_id,
        map_id=target_floor.map_id,
        target=_pose(z=3.6 if target_floor == _floor(2) else 0.0),
        travel_mode=travel_mode,
        connector_id=connector_id,
        map_version=map_version,
        version_id=version_id,
        map_pcd_sha256=map_pcd_sha256,
    )


def _plan() -> LiftTransitionPlan:
    return LiftTransitionPlan(
        lift_id="lift-a",
        source_floor=_floor(1),
        target_floor=_floor(2),
        source_lobby=_pose(1.0),
        source_cabin=_pose(2.0),
        target_cabin=_pose(2.0, z=3.6),
        target_lobby=_pose(3.0, z=3.6),
    )


def test_building_mission_request_keeps_legacy_constructor_and_new_defaults() -> None:
    request = BuildingMissionRequest(
        "building-task-1",
        "native_test",
        "",
        "thunder-01",
        "factory-a",
        "floor-1",
        "factory-a-floor-1",
        _pose(),
    )

    assert request.travel_mode == "any"
    assert request.connector_id == ""
    assert request.place_id == ""
    assert request.map_version is None
    assert request.version_id == ""
    assert request.map_pcd_sha256 == ""


@pytest.mark.parametrize("travel_mode", ["stairs", "elevator"])
def test_same_floor_explicit_connector_mode_is_not_silently_ignored(
    travel_mode: str,
) -> None:
    navigation = FakeNavigation()
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=_floor(1),
    )

    assert orchestrator.submit(_mission(travel_mode=travel_mode)) == (
        False,
        "connector_transition_not_required",
    )
    assert navigation.goals == []


def test_orchestrator_rejects_unknown_travel_mode() -> None:
    navigation = FakeNavigation()
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=_floor(1),
    )

    assert orchestrator.submit(_mission(travel_mode="teleport")) == (
        False,
        "invalid_travel_mode",
    )
    assert navigation.goals == []


def test_bound_cross_floor_request_requires_validator_before_transition() -> None:
    navigation = FakeNavigation()
    transition = FakeTransition()
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=_floor(1),
        transition_executor=transition,
    )

    assert orchestrator.submit(
        _mission(
            floor=_floor(2),
            map_version=4,
            version_id="factory-a-floor-2:v4",
            map_pcd_sha256="sha256-current",
        )
    ) == (False, "target_binding_validator_unavailable")
    assert transition.starts == []
    assert navigation.goals == []


def test_bound_same_floor_request_validates_immediately_before_goal() -> None:
    navigation = FakeNavigation()
    validations: list[BuildingMissionRequest] = []

    def validate(request: BuildingMissionRequest) -> tuple[bool, str]:
        assert navigation.goals == []
        validations.append(request)
        return True, "target_binding_current"

    request = _mission(map_version=4)
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=_floor(1),
        target_binding_validator=validate,
    )

    assert orchestrator.submit(request) == (True, "building_navigation_goal_accepted")
    assert validations == [request]
    assert navigation.goals == [(1.0, 2.0, 0.0, 0.0, "building-task-1:goal")]


def test_cross_floor_revalidates_binding_after_transition_before_final_goal() -> None:
    navigation = FakeNavigation()
    transition = FakeTransition()
    active_floor = [_floor(1)]
    validations: list[BuildingMissionRequest] = []

    def validate(request: BuildingMissionRequest) -> tuple[bool, str]:
        validations.append(request)
        return False, "target_map_version_changed"

    request = _mission(floor=_floor(2), map_version=4)
    orchestrator = BuildingMissionOrchestrator(
        navigation=navigation,
        active_floor=lambda: active_floor[0],
        transition_executor=transition,
        target_binding_validator=validate,
    )

    assert orchestrator.submit(request) == (True, "floor_transition_started")
    active_floor[0] = _floor(2)
    transition.progress = ("succeeded", "floor_transition_complete")

    status = orchestrator.tick()

    assert status.reason == "target_map_version_changed"
    assert status.phase is BuildingMissionPhase.FAILED
    assert validations == [request]
    assert navigation.goals == []


@pytest.mark.parametrize("travel_mode", ["any", "elevator"])
def test_lift_service_accepts_compatible_travel_modes(travel_mode: str) -> None:
    plan = _plan()
    executor = FakeLiftExecutor()
    service = LiftTransitionService(
        catalog=StaticLiftTransitionCatalog([plan]),
        executor=executor,
    )

    assert service.start(
        _mission(floor=_floor(2), travel_mode=travel_mode),
        source_floor=_floor(1),
    ) == (True, "lift_transition_started")
    assert executor.starts == [(plan, "building-task-1")]


def test_lift_service_rejects_stairs_mode_without_starting_executor() -> None:
    executor = FakeLiftExecutor()
    service = LiftTransitionService(
        catalog=StaticLiftTransitionCatalog([_plan()]),
        executor=executor,
    )

    assert service.start(
        _mission(floor=_floor(2), travel_mode="stairs"),
        source_floor=_floor(1),
    ) == (False, "lift_transition_travel_mode_unsupported")
    assert executor.starts == []


@pytest.mark.parametrize(
    ("connector_id", "expected"),
    [
        ("lift-a", (True, "lift_transition_started")),
        ("lift-b", (False, "lift_connector_mismatch")),
    ],
)
def test_lift_service_honors_explicit_connector_id(
    connector_id: str,
    expected: tuple[bool, str],
) -> None:
    plan = _plan()
    executor = FakeLiftExecutor()
    service = LiftTransitionService(
        catalog=StaticLiftTransitionCatalog([plan]),
        executor=executor,
    )

    assert (
        service.start(
            _mission(floor=_floor(2), travel_mode="elevator", connector_id=connector_id),
            source_floor=_floor(1),
        )
        == expected
    )
    assert executor.starts == ([(plan, "building-task-1")] if expected[0] else [])
