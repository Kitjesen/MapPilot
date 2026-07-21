"""Admission tests for the map-bound building navigation service."""

from __future__ import annotations

import json
from copy import deepcopy
from typing import Any

import pytest

from nav.building import BuildingMissionRequest, BuildingService


class FakeMapsAPI:
    def __init__(self, *, active: str = "hq-floor-1") -> None:
        self.active = active
        self.records = {
            map_id: {
                "map_id": map_id,
                "version": version,
                "version_id": f"{map_id}:v{version}",
                "state": "READY",
                "scope": {"frame_id": "map"},
                "artifacts": [
                    {
                        "type": "POINTCLOUD",
                        "name": "map_pcd",
                        "hash": f"sha-{map_id}",
                    }
                ],
            }
            for map_id, version in (("hq-floor-1", 2), ("hq-floor-6", 7))
        }
        self.edges: list[dict[str, Any]] = [
            {
                "from": "hq-floor-1",
                "to": "hq-floor-6",
                "type": "elevator",
                "bidirectional": True,
                "connector_id": "lift-east",
            }
        ]
        self.transitions: list[dict[str, str]] = [
            {
                "from_map_id": "hq-floor-1",
                "to_map_id": "hq-floor-6",
                "edge_id": "hq-floor-1->hq-floor-6",
            }
        ]
        self.calls: list[tuple[Any, ...]] = []

    def get_active_map(self) -> dict[str, Any]:
        self.calls.append(("get_active_map",))
        return {"success": True, "active": self.active}

    def get_record(self, map_id: str) -> dict[str, Any]:
        self.calls.append(("get_record", map_id))
        return {"success": True, "record": deepcopy(self.records[map_id])}

    def get_map_points(self, map_id: str, *, max_points: int = 0) -> dict[str, Any]:
        self.calls.append(("get_map_points", map_id, max_points))
        record = self.records[map_id]
        return {
            "success": True,
            "map_id": map_id,
            "version_id": record["version_id"],
            "map_pcd_sha256": record["artifacts"][0]["hash"],
            "frame_id": "map",
            "points": [],
        }

    def shortest_route(self, command: dict[str, str]) -> dict[str, Any]:
        self.calls.append(("shortest_route", dict(command)))
        return {
            "success": bool(self.transitions),
            "found": bool(self.transitions),
            "transitions": deepcopy(self.transitions),
        }

    def list_map_graph(self) -> dict[str, Any]:
        self.calls.append(("list_map_graph",))
        return {"success": True, "edges": deepcopy(self.edges)}


class MapsModuleWrapper:
    def __init__(self, api: FakeMapsAPI) -> None:
        self.api = api


class FakeMissionPort:
    def __init__(self, result: tuple[bool, str] = (True, "building_mission_accepted")) -> None:
        self.result = result
        self.requests: list[BuildingMissionRequest] = []
        self.cancellations: list[str] = []

    def submit(self, request: BuildingMissionRequest) -> tuple[bool, str]:
        self.requests.append(request)
        return self.result

    def cancel(self, reason: str):
        self.cancellations.append(reason)
        return None


def _command(
    *,
    map_id: str = "hq-floor-1",
    floor_id: str = "floor-1",
    travel_mode: str = "any",
    connector_id: str = "",
) -> dict[str, Any]:
    version = 2 if map_id == "hq-floor-1" else 7
    return {
        "action": "building_navigate",
        "schema_version": "lingtu.building_goal.v1",
        "request_id": "voice-turn-1",
        "source": "askme.voice",
        "fleet_name": "",
        "robot_name": "thunder-01",
        "travel_mode": travel_mode,
        "connector_id": connector_id,
        "target": {
            "building_id": "hq",
            "floor_id": floor_id,
            "map_id": map_id,
            "place_id": "place-company-a",
            "frame_id": "map",
            "x": 6.0,
            "y": -1.5,
            "z": 0.0,
            "yaw": 1.2,
            "map_version": version,
            "version_id": f"{map_id}:v{version}",
            "map_pcd_sha256": f"sha-{map_id}",
        },
    }


def _service(
    maps: FakeMapsAPI | None = None,
    mission: FakeMissionPort | None = None,
) -> tuple[BuildingService, FakeMapsAPI, FakeMissionPort]:
    maps = maps or FakeMapsAPI()
    mission = mission or FakeMissionPort()
    service = BuildingService(mission_port=mission)
    service.on_system_modules({"maps.service": MapsModuleWrapper(maps)})
    return service, maps, mission


def test_same_map_goal_revalidates_binding_and_submits_typed_mission() -> None:
    service, maps, mission = _service()
    statuses: list[dict[str, Any]] = []
    service.building_status.subscribe(statuses.append)

    result = service.submit(_command())

    assert result["accepted"] is True
    assert result["success"] is True
    assert result["reason"] == "building_mission_accepted"
    assert statuses == [result]
    assert maps.calls == [
        ("get_active_map",),
        ("get_record", "hq-floor-1"),
        ("get_map_points", "hq-floor-1", 1),
    ]
    request = mission.requests[0]
    assert request.request_id == "voice-turn-1"
    assert request.target.frame_id == "map"
    assert (request.target.x, request.target.y, request.target.z, request.target.yaw) == (
        6.0,
        -1.5,
        0.0,
        1.2,
    )
    assert request.place_id == "place-company-a"
    assert request.map_version == 2
    assert request.version_id == "hq-floor-1:v2"
    assert request.map_pcd_sha256 == "sha-hq-floor-1"


def test_cross_map_goal_requires_one_verified_transition() -> None:
    service, maps, mission = _service()

    result = service.submit(_command(map_id="hq-floor-6", floor_id="floor-6"))

    assert result["accepted"] is True
    assert mission.requests[0].map_id == "hq-floor-6"
    assert ("shortest_route", {"from": "hq-floor-1", "to": "hq-floor-6"}) in maps.calls


def test_cross_map_explicit_elevator_requires_matching_direct_graph_edge() -> None:
    maps = FakeMapsAPI()
    maps.edges = [
        {
            "from": "hq-floor-6",
            "to": "hq-floor-1",
            "type": "lift",
            "bidirectional": True,
            "connector_id": "lift-east",
        }
    ]
    service, _, mission = _service(maps=maps)

    accepted = service.submit(
        _command(
            map_id="hq-floor-6",
            floor_id="floor-6",
            travel_mode="elevator",
            connector_id="lift-east",
        )
    )
    assert accepted["accepted"] is True
    assert mission.requests[0].travel_mode == "elevator"
    assert mission.requests[0].connector_id == "lift-east"

    wrong_connector = service.submit(
        _command(
            map_id="hq-floor-6",
            floor_id="floor-6",
            travel_mode="elevator",
            connector_id="lift-west",
        )
    )
    assert wrong_connector["accepted"] is False
    assert wrong_connector["reason"] == "connector_route_unavailable"

    maps.edges[0]["type"] = "stairs"
    rejected = service.submit(
        _command(
            map_id="hq-floor-6",
            floor_id="floor-6",
            travel_mode="elevator",
            connector_id="lift-east",
        )
    )
    assert rejected["accepted"] is False
    assert rejected["reason"] == "connector_route_unavailable"
    assert len(mission.requests) == 1


def test_any_mode_with_connector_id_still_requires_matching_direct_edge() -> None:
    service, maps, mission = _service()

    result = service.submit(
        _command(
            map_id="hq-floor-6",
            floor_id="floor-6",
            travel_mode="any",
            connector_id="lift-west",
        )
    )

    assert result["accepted"] is False
    assert result["reason"] == "connector_route_unavailable"
    assert ("list_map_graph",) in maps.calls
    assert mission.requests == []


@pytest.mark.parametrize(
    ("travel_mode", "connector_id"),
    [("elevator", "lift-east"), ("stairs", "stairs-east")],
)
def test_explicit_mode_without_id_selects_unique_typed_connector(
    travel_mode: str,
    connector_id: str,
) -> None:
    maps = FakeMapsAPI()
    maps.edges = [
        {
            "from": "hq-floor-1",
            "to": "hq-floor-6",
            "type": travel_mode,
            "bidirectional": True,
            "connector_id": connector_id,
        }
    ]
    service, _, mission = _service(maps=maps)

    result = service.submit(
        _command(
            map_id="hq-floor-6",
            floor_id="floor-6",
            travel_mode=travel_mode,
        )
    )

    assert result["accepted"] is True
    assert mission.requests[0].connector_id == connector_id


def test_explicit_mode_without_id_rejects_ambiguous_typed_connectors() -> None:
    maps = FakeMapsAPI()
    maps.edges = [
        {
            "from": "hq-floor-1",
            "to": "hq-floor-6",
            "type": "elevator",
            "bidirectional": True,
            "connector_id": connector_id,
        }
        for connector_id in ("lift-east", "lift-west")
    ]
    service, _, mission = _service(maps=maps)

    result = service.submit(
        _command(
            map_id="hq-floor-6",
            floor_id="floor-6",
            travel_mode="elevator",
        )
    )

    assert result["accepted"] is False
    assert result["reason"] == "connector_route_unavailable"
    assert mission.requests == []


def test_native_map_edge_id_is_not_treated_as_a_lift_identity() -> None:
    maps = FakeMapsAPI()
    maps.edges = [
        {
            "from": "hq-floor-1",
            "to": "hq-floor-6",
            "type": "elevator",
            "bidirectional": True,
        }
    ]
    service, _, mission = _service(maps=maps)

    result = service.submit(
        _command(
            map_id="hq-floor-6",
            floor_id="floor-6",
            travel_mode="elevator",
        )
    )

    assert result["accepted"] is False
    assert result["reason"] == "connector_route_unavailable"
    assert mission.requests == []


def test_same_map_explicit_connector_fails_closed() -> None:
    service, _, mission = _service()

    result = service.submit(_command(travel_mode="stairs"))

    assert result["accepted"] is False
    assert result["reason"] == "connector_transition_not_required"
    assert mission.requests == []


@pytest.mark.parametrize(
    ("mutate", "reason"),
    [
        (lambda command: command.update(schema_version="building_navigate.v1"), "invalid_schema_version"),
        (lambda command: command.update(travel_mode="teleport"), "invalid_travel_mode"),
        (lambda command: command["target"].update(frame_id="odom"), "unsupported_frame"),
        (lambda command: command["target"].update(x=float("nan")), "invalid_x"),
        (lambda command: command["target"].update(map_version=True), "invalid_map_version"),
    ],
)
def test_invalid_goal_schema_never_reaches_maps_or_mission(mutate, reason: str) -> None:
    service, maps, mission = _service()
    command = _command()
    mutate(command)

    result = service.submit(command)

    assert result["accepted"] is False
    assert result["reason"] == reason
    assert maps.calls == []
    assert mission.requests == []


def test_stale_target_binding_fails_before_route_or_mission() -> None:
    service, maps, mission = _service()
    command = _command()
    command["target"]["map_pcd_sha256"] = "stale-sha"

    result = service.submit(command)

    assert result["accepted"] is False
    assert result["reason"] == "target_map_binding_mismatch"
    assert "map_pcd_sha256" in result["message"]
    assert not any(call[0] == "shortest_route" for call in maps.calls)
    assert mission.requests == []


def test_multiple_transitions_and_missing_connector_edge_fail_closed() -> None:
    maps = FakeMapsAPI()
    maps.transitions.append(
        {
            "from_map_id": "hq-floor-3",
            "to_map_id": "hq-floor-6",
            "edge_id": "hq-floor-3->hq-floor-6",
        }
    )
    service, _, mission = _service(maps=maps)

    result = service.submit(_command(map_id="hq-floor-6", floor_id="floor-6"))

    assert result["accepted"] is False
    assert result["reason"] == "building_route_not_direct"
    assert mission.requests == []


def test_missing_capabilities_and_mission_rejection_are_statuses_not_exceptions() -> None:
    command = _command()
    no_maps = BuildingService(mission_port=FakeMissionPort())
    assert no_maps.submit(command)["reason"] == "maps_unavailable"

    maps = FakeMapsAPI()
    no_mission = BuildingService()
    no_mission.on_system_modules({"maps.service": MapsModuleWrapper(maps)})
    assert no_mission.submit(command)["reason"] == "mission_unavailable"

    rejecting_mission = FakeMissionPort((False, "building_mission_busy"))
    service, _, _ = _service(mission=rejecting_mission)
    rejected = service.submit(command)
    assert rejected["accepted"] is False
    assert rejected["reason"] == "building_mission_busy"


def test_input_ports_parse_json_and_cancel_through_mission_owner() -> None:
    service, _, mission = _service()
    statuses: list[dict[str, Any]] = []
    service.building_status.subscribe(statuses.append)
    service.setup()

    service.mission_request._deliver(json.dumps(_command()))
    service.cancel_request._deliver(json.dumps({"request_id": "voice-turn-cancel", "reason": "operator_cancel"}))

    assert len(mission.requests) == 1
    assert mission.cancellations == ["operator_cancel"]
    assert statuses[-1]["action"] == "building_cancel"
    assert statuses[-1]["request_id"] == "voice-turn-cancel"
    assert statuses[-1]["accepted"] is True
