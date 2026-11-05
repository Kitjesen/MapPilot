from __future__ import annotations

from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.gateway_module import GatewayModule
from gateway.navigation.routes import register_navigation_routes


@pytest.fixture
def navigation():
    gateway = GatewayModule()
    gateway._navigation_state = {"authority": "estop", "ts": 1.0}
    gateway._session_mode = "mapping"
    calls = []
    result = {"accepted": True}

    def submit(goal, *, task_id, request_id, action, **_limits):
        calls.append((goal, task_id, request_id, action))
        ack = {"accepted": result["accepted"], "task_id": task_id,
               "request_id": request_id, "reason": result.get("reason", "")}
        return {"task_id": task_id, "request_id": request_id, **result,
                **({} if result.get("admission_unconfirmed") else {"native_ack": ack})}

    gateway._goals = SimpleNamespace(submit_goal=submit)
    app = FastAPI()
    register_navigation_routes(app, gateway)
    with TestClient(app) as client:
        yield client, gateway, calls, result


@pytest.mark.parametrize("path", ["/api/v1/goal", "/api/v1/navigate/click"])
def test_cached_readiness_does_not_override_native_acceptance(navigation, path):
    client, _gateway, calls, _result = navigation
    response = client.post(path, json={"x": 1, "y": 2, "client_id": "web", "request_id": "r-1"})
    assert response.status_code == 200, response.text
    assert response.json()["command"]["accepted"] is True
    assert len(calls) == 1


@pytest.mark.parametrize("path", ["/api/v1/goal", "/api/v1/navigate/click"])
def test_native_rejection_is_returned_when_cached_state_disagrees(navigation, path, monkeypatch):
    client, gateway, calls, result = navigation
    gateway._navigation_state = {"authority": "none"}
    gateway._session_mode = "navigating"
    monkeypatch.setattr("gateway.navigation.status.evaluate_navigation_gate",
                        lambda _gw: {"can_accept_goal": True, "blockers": []})
    result.update(accepted=False, message="goal rejected", reason="estop_latched")
    response = client.post(path, json={"x": 1, "y": 2, "client_id": "web", "request_id": "r-2"})
    assert response.status_code == 409
    assert len(calls) == 1
    assert response.json()["error"] == "native_command_rejected"
    assert response.json()["detail"]["reason_code"] == "estop_latched"
    assert response.json()["detail"]["native_ack"]["accepted"] is False


def test_unknown_native_outcome_is_not_reported_as_a_rejection(navigation):
    client, _gateway, calls, result = navigation
    result.update(accepted=False, admission_unconfirmed=True, message="ACK timeout")
    response = client.post("/api/v1/goal", json={"x": 1, "y": 2, "client_id": "web", "request_id": "r-3"})
    assert response.status_code == 503
    body = response.json()
    assert body["error"] == "native_command_unconfirmed"
    assert body.get("command") is None
    assert body["detail"]["task_id"] == calls[0][1]
    assert body["detail"]["request_id"] == "r-3"


def test_control_lease_still_blocks_a_different_page(navigation):
    client, gateway, calls, _result = navigation
    assert gateway._lease.acquire("owner", 30)
    response = client.post("/api/v1/goal", json={"x": 1, "y": 2, "client_id": "other", "request_id": "r-4"})
    assert response.status_code == 403
    assert response.json()["error"] == "control_lease"
    assert calls == []


def test_goal_openapi_describes_lease_refusal_and_unconfirmed_admission(navigation):
    client, _gateway, _calls, _result = navigation
    schema = client.get("/openapi.json").json()
    paths = schema["paths"]
    assert "native_ack" in schema["components"]["schemas"]["ControlCommandResponse"]["properties"]
    assert "native_ack" in schema["components"]["schemas"]["GatewayCommandErrorDetail"]["properties"]
    for path in ("/api/v1/goal", "/api/v1/navigate/click"):
        responses = paths[path]["post"]["responses"]
        assert {"200", "403", "409", "503"} <= responses.keys()
        assert responses["503"]["content"]["application/json"]["schema"]["$ref"].endswith(
            "/GatewayErrorResponse"
        )


@pytest.mark.parametrize("outcome", ["accepted", "rejected", "timeout"])
def test_http_goal_preserves_real_goal_service_native_receipt(outcome):
    from nav.services.goals import GoalService
    from runtime.msgs.nav import NavigationCommandKind, NavigationCommandReceipt

    class NativeCommands:
        calls = 0

        def send_goal(self, x, y, z, yaw, *, task_id, request_id):
            del x, y, z, yaw
            self.calls += 1
            if outcome == "timeout":
                raise TimeoutError("native ACK timeout")
            return NavigationCommandReceipt(
                accepted=outcome == "accepted", kind=int(NavigationCommandKind.GOAL),
                task_id=task_id, request_id=request_id, endpoint_timestamp_s=1.0,
                reason="planning" if outcome == "accepted" else "estop_latched",
            )

    commands = NativeCommands()
    goals = GoalService()
    goals.on_system_modules({"nav.commands": commands})
    goals.setup()
    gateway = GatewayModule()
    gateway._session_mode = "mapping"
    gateway._goals = goals
    app = FastAPI()
    register_navigation_routes(app, gateway)
    body = {"x": 1, "y": 2, "client_id": "web", "task_id": "task-1", "request_id": "request-1"}
    try:
        with TestClient(app) as client:
            response = client.post("/api/v1/goal", json=body)
            if outcome == "accepted":
                assert response.status_code == 200
                assert response.json()["native_ack"]["accepted"] is True
            elif outcome == "rejected":
                assert response.status_code == 409
                assert response.json()["detail"]["native_ack"]["reason"] == "estop_latched"
            else:
                assert response.status_code == 503
                assert response.json().get("command") is None
                assert client.post("/api/v1/goal", json=body).status_code == 503
            assert commands.calls == 1
    finally:
        goals.stop()
