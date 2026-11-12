"""Public domain-client composition, compatibility and command contracts."""

from __future__ import annotations

import inspect
import io
import json
from unittest.mock import Mock

import pytest

import lingtu.sdk as sdk
from lingtu.sdk import client as legacy

DOMAINS = (
    ("navigation", "NavigationClient"),
    ("map", "MapClient"),
    ("tasks", "TaskClient"),
    ("observations", "ObservationClient"),
    ("system", "SystemClient"),
)


@pytest.mark.parametrize(("property_name", "class_name"), (*DOMAINS, ("localization", "LocalizationClient")))
def test_exported_domain_interfaces_share_configuration_and_match_async(
    property_name,
    class_name,
    monkeypatch,
):
    def unexpected_http(*args, **kwargs):
        pytest.fail("Constructing a client must not make HTTP requests")

    monkeypatch.setattr("urllib.request.urlopen", unexpected_http)
    robot = sdk.LingTuClient("robot.local", 6060, api_key="test-key", timeout=2.5)
    domain_type = getattr(sdk, class_name)
    domain = getattr(robot, property_name)
    assert isinstance(domain, domain_type)
    assert getattr(robot, property_name) is domain
    assert domain._client is robot
    assert domain_type(robot)._client is robot
    assert class_name in sdk.__all__

    async_type = getattr(sdk, f"Async{class_name}")
    assert f"Async{class_name}" in sdk.__all__
    sync_methods = {
        name: method for name, method in vars(domain_type).items() if not name.startswith("_") and callable(method)
    }
    async_methods = {
        name: method for name, method in vars(async_type).items() if not name.startswith("_") and callable(method)
    }
    assert sync_methods.keys() == async_methods.keys()
    for name, method in sync_methods.items():
        assert inspect.iscoroutinefunction(async_methods[name])
        assert inspect.signature(method) == inspect.signature(async_methods[name]), name


def test_domain_calls_and_legacy_calls_keep_http_configuration_and_receipts(monkeypatch):
    payload = {
        "ok": True,
        "accepted": True,
        "task_id": "nav-1",
        "execution_confirmed": False,
        "motor_confirmed": False,
    }
    transport = Mock(side_effect=lambda *args, **kwargs: io.BytesIO(json.dumps(payload).encode()))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    robot = sdk.LingTuClient("robot.local", 6060, api_key="test-key", timeout=2.5)

    grouped = robot.navigation.go(1.0, 2.0, 0.5, request_id="goal-1")
    flat = robot.go(1.0, 2.0, 0.5, request_id="goal-1")

    assert grouped == flat
    assert grouped.task_id == "nav-1" and grouped.request_id == "goal-1"
    assert grouped.raw["motor_confirmed"] is False
    assert transport.call_count == 2
    for request_call in transport.call_args_list:
        request = request_call.args[0]
        assert request.full_url == "http://robot.local:6060/api/v1/goal"
        assert request.get_header("X-api-key") == "test-key"
        assert request_call.kwargs["timeout"] == 2.5
        assert json.loads(request.data) == {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5, "request_id": "goal-1", "client_id": "sdk"}


def test_navigation_cancel_and_emergency_stop_remain_distinct(monkeypatch):
    transport = Mock(side_effect=lambda *args, **kwargs: io.BytesIO(b'{"accepted":true}'))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    robot = sdk.LingTuClient()

    assert robot.navigation.cancel("operator_cancel").accepted
    assert robot.system.emergency_stop().accepted

    cancel_request, stop_request = [entry.args[0] for entry in transport.call_args_list]
    assert cancel_request.full_url.endswith("/api/v1/navigation/cancel")
    assert json.loads(cancel_request.data) == {"reason": "operator_cancel", "client_id": "sdk"}
    assert stop_request.full_url.endswith("/api/v1/stop")
    assert json.loads(stop_request.data) == {}


def test_standalone_map_client_preserves_named_place_encoding(monkeypatch):
    transport = Mock(return_value=io.BytesIO(b'{"accepted":true}'))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    connection = sdk.LingTuClient("robot.local", api_key="test-key")

    result = sdk.MapClient(connection).delete_location("floor 1/door")

    assert result.accepted
    request = transport.call_args.args[0]
    assert request.get_method() == "DELETE"
    assert request.full_url == "http://robot.local:5050/api/v1/locations/floor%201%2Fdoor"
    assert request.get_header("X-api-key") == "test-key"


def test_original_response_imports_refer_to_the_same_public_types():
    for name in (
        "CommandResult",
        "HealthStatus",
        "MapInfo",
        "MapList",
        "NavigationControlState",
        "NavigationGoalAdmission",
        "NavigationMotionState",
        "NavigationStatus",
        "NavigationTask",
        "Pose2D",
        "Position",
        "RobotState",
        "SessionInfo",
        "LocalizationClient",
    ):
        assert getattr(legacy, name) is getattr(sdk, name), name


@pytest.mark.parametrize("click", [False, True])
@pytest.mark.parametrize("yaw", [None, 0.0, 1.2])
def test_goal_fields_reach_real_gateway_model(monkeypatch, click, yaw):
    from gateway.navigation.goals import construct_goal_from_request
    from gateway.schemas import ClickNavRequest, GoalRequest
    captured = []
    def transport(request, **kwargs):
        captured.append(json.loads(request.data))
        return io.BytesIO(b'{"accepted":true}')
    monkeypatch.setattr("urllib.request.urlopen", transport)
    robot = sdk.LingTuClient()
    operation = robot.navigation.go_to_click if click else robot.navigation.go
    operation(2, 3, yaw, z=4.5, task_id="task-3d", request_id="goal-3d",
              max_speed_mps=0.3, acceptance_radius_m=0.12)
    model = (ClickNavRequest if click else GoalRequest)(**captured[0])
    goal = construct_goal_from_request(model)
    assert (goal.x, goal.y, goal.z, goal.yaw) == (2, 3, 4.5, yaw)
    assert goal.max_speed_mps == 0.3 and goal.acceptance_radius_m == 0.12


@pytest.mark.parametrize("action", ["pause", "resume", "cancel", "set_speed_limit"])
def test_navigation_controls_keep_task_identity(monkeypatch, action):
    transport = Mock(return_value=io.BytesIO(b'{"accepted":true,"execution_confirmed":false}'))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    nav = sdk.LingTuClient().navigation
    if action == "cancel":
        result = nav.cancel(task_id="task/1", request_id="control-1")
    elif action == "set_speed_limit":
        result = nav.set_speed_limit("task/1", 0.2, request_id="control-1")
    else:
        result = getattr(nav, action)("task/1", request_id="control-1")
    request = transport.call_args.args[0]
    assert request.full_url.endswith("/tasks/task%2F1/" + ("speed-limit" if action == "set_speed_limit" else action))
    assert json.loads(request.data)["task_id"] == "task/1"
    assert json.loads(request.data)["request_id"] == "control-1"
    assert result.raw["execution_confirmed"] is False
