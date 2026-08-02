"""Focused tests for building navigation command dispatch."""

from __future__ import annotations

import json

import pytest

from nav.services.goals import GoalService


class FakeBuildingModule:
    def __init__(self, result: dict | None = None) -> None:
        self.result = (
            {
                "accepted": True,
                "success": True,
                "reason": "mission_accepted",
                "message": "building mission accepted",
            }
            if result is None
            else result
        )
        self.calls: list[dict] = []

    def submit(self, command: dict) -> dict:
        self.calls.append(command)
        return dict(self.result)


def _service(building: object | None = None) -> tuple[GoalService, list[dict], list[object]]:
    service = GoalService(building_module="nav.building")
    if building is not None:
        service.on_system_modules({"nav.building": building})
    service.setup()
    statuses: list[dict] = []
    goals: list[object] = []
    service.goal_status.subscribe(statuses.append)
    service.goal_pose.subscribe(goals.append)
    return service, statuses, goals


@pytest.mark.parametrize("action", ["building_navigate", "building"])
def test_building_command_delegates_complete_command_with_request_id(action: str) -> None:
    building = FakeBuildingModule()
    service, statuses, goals = _service(building)
    command = {
        "action": action,
        "target_place_id": "place-company-6f",
        "travel_mode": "elevator",
        "request_id": "voice-42",
    }

    service.goal_command._deliver(json.dumps(command))

    expected_command = {**command, "action": "building_navigate"}
    assert building.calls == [expected_command]
    assert building.calls[0] is not command
    assert goals == []
    assert statuses[-1]["request_id"] == "voice-42"
    assert statuses[-1]["action"] == action
    assert statuses[-1]["accepted"] is True
    assert statuses[-1]["success"] is True
    assert statuses[-1]["reason"] == "mission_accepted"
    assert statuses[-1]["message"] == "building mission accepted"


def test_building_command_generates_request_id_for_forwarded_copy() -> None:
    building = FakeBuildingModule()
    service, statuses, _goals = _service(building)
    command = {
        "action": "building_navigate",
        "schema_version": "lingtu.building_goal.v1",
        "target": {"place_id": "place-company-6f"},
    }

    service.goal_command._deliver(json.dumps(command))

    assert "request_id" not in command
    assert building.calls[0]["request_id"].startswith("nav-")
    assert statuses[-1]["request_id"] == building.calls[0]["request_id"]


def test_building_command_fails_closed_when_module_is_unavailable() -> None:
    service, statuses, goals = _service()

    service.goal_command._deliver(json.dumps({"action": "building_navigate", "request_id": "voice-43"}))

    assert goals == []
    assert statuses[-1]["success"] is False
    assert statuses[-1]["accepted"] is False
    assert statuses[-1]["reason"] == "building_module_unavailable"
    assert statuses[-1]["request_id"] == "voice-43"


def test_building_command_fails_closed_when_submit_is_missing() -> None:
    service, statuses, goals = _service(object())

    service.goal_command._deliver(json.dumps({"action": "building", "request_id": "voice-44"}))

    assert goals == []
    assert statuses[-1]["success"] is False
    assert statuses[-1]["accepted"] is False
    assert statuses[-1]["reason"] == "building_submit_unavailable"


def test_building_command_publishes_rejection_from_building_module() -> None:
    building = FakeBuildingModule(
        {
            "accepted": False,
            "success": False,
            "reason": "connector_unavailable",
            "message": "requested elevator is unavailable",
        }
    )
    service, statuses, goals = _service(building)

    service.goal_command._deliver(json.dumps({"action": "building_navigate", "request_id": "voice-45"}))

    assert goals == []
    assert statuses[-1]["success"] is False
    assert statuses[-1]["accepted"] is False
    assert statuses[-1]["reason"] == "connector_unavailable"
    assert statuses[-1]["message"] == "requested elevator is unavailable"


@pytest.mark.parametrize(
    "result",
    [
        {"accepted": "false", "success": "false"},
        {"accepted": 1, "success": True},
        {"success": "true"},
        {},
    ],
)
def test_building_command_rejects_malformed_acknowledgement(result: dict) -> None:
    building = FakeBuildingModule(result)
    service, statuses, goals = _service(building)

    service.goal_command._deliver(
        json.dumps({"action": "building_navigate", "request_id": "voice-malformed"})
    )

    assert goals == []
    assert statuses[-1]["success"] is False
    assert statuses[-1]["accepted"] is False
    assert statuses[-1]["reason"] == "invalid_building_ack"


def test_building_command_fails_closed_when_submit_raises() -> None:
    class FailingBuildingModule:
        def submit(self, command: dict) -> dict:
            del command
            raise ValueError("lift controller offline")

    service, statuses, goals = _service(FailingBuildingModule())

    service.goal_command._deliver(json.dumps({"action": "building_navigate", "request_id": "voice-46"}))

    assert goals == []
    assert statuses[-1]["success"] is False
    assert statuses[-1]["accepted"] is False
    assert statuses[-1]["reason"] == "building_dispatch_error"
    assert statuses[-1]["message"] == "lift controller offline"
