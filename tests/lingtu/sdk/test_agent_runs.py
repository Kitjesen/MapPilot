"""Agent HTTP contracts shared by the synchronous and asynchronous SDKs."""

import inspect
import io
import json
import urllib.error
from unittest.mock import Mock

import pytest
from fastapi.testclient import TestClient

from decision.modules.agent_planner import AgentPlannerModule, AgentRun
from gateway.gateway_module import GatewayModule
from gateway.schemas import AgentTaskCancelRequest, InstructionRequest
from lingtu.sdk import AsyncLingTuClient, LingTuClient


@pytest.fixture(params=[LingTuClient, AsyncLingTuClient], ids=["sync", "async"])
def robot(request):
    return request.param("127.0.0.1", api_key="test-key", timeout=2.5)


async def invoke(method, *args, **kwargs):
    result = method(*args, **kwargs)
    return await result if inspect.isawaitable(result) else result


@pytest.fixture
def http(monkeypatch):
    def respond(payload, status=200):
        content = json.dumps(payload).encode("utf-8")

        def urlopen(request, *, timeout):
            if status >= 400:
                raise urllib.error.HTTPError(
                    request.full_url, status, "rejected", {}, io.BytesIO(content),
                )
            return io.BytesIO(content)

        mocked = Mock(side_effect=urlopen)
        monkeypatch.setattr("urllib.request.urlopen", mocked)
        return mocked

    return respond


@pytest.mark.asyncio
async def test_instruction_preserves_controls_and_owner_receipt(robot, http):
    payload = {
        "ok": True, "accepted": True, "operation": "query", "executor": "agent",
        "run_id": "agent-current", "state": "running", "request_id": "query-1",
        "task_id": "navigation-child", "execution_confirmed": False,
        "motor_confirmed": False, "stop_confirmed": None,
        "task_status": {"run_id": "agent-current", "instruction": "follow red"},
    }
    transport = http(payload)

    result = await invoke(
        robot.send_instruction, "现在在干什么", run_id="agent-current",
        control_only=True, request_id="query-1", client_id="operator-console",
    )

    request = transport.call_args.args[0]
    assert request.full_url == "http://127.0.0.1:5050/api/v1/instruction"
    assert request.get_method() == "POST"
    assert request.get_header("X-api-key") == "test-key"
    assert transport.call_args.kwargs["timeout"] == 2.5
    body = json.loads(request.data)
    assert body == {
        "text": "现在在干什么", "run_id": "agent-current", "control_only": True,
        "request_id": "query-1", "client_id": "operator-console",
    }
    assert InstructionRequest.model_validate(body).run_id == "agent-current"
    assert result.ok and result.accepted
    assert result.run_id == "agent-current"
    assert result.task_id == "navigation-child"
    assert result.request_id == "query-1"
    assert result.raw == payload


@pytest.mark.asyncio
@pytest.mark.parametrize("method_name", ["send_instruction", "go_to"])
async def test_plain_instruction_generates_request_identity(robot, http, method_name):
    transport = http({"ok": True, "accepted": True, "run_id": "agent-new"})

    result = await invoke(getattr(robot, method_name), "find a bottle")

    request = transport.call_args.args[0]
    body = json.loads(request.data)
    assert request.full_url.endswith("/api/v1/instruction")
    assert body["text"] == "find a bottle"
    assert body["control_only"] is False
    assert body.get("run_id") is None
    assert body["client_id"] == "sdk"
    assert len(body["request_id"]) == 26
    assert result.request_id == body["request_id"]
    assert result.run_id == "agent-new"
    assert transport.call_count == 1


@pytest.mark.asyncio
async def test_semantic_fallback_does_not_invent_an_agent_run(robot, http):
    payload = {"ok": True, "accepted": True, "executor": "semantic"}
    http(payload)

    result = await invoke(robot.send_instruction, "find a bottle")

    assert result.accepted
    assert result.run_id is None
    assert result.raw == payload


@pytest.mark.asyncio
@pytest.mark.parametrize("reason", ["not_current", "agent_unavailable"])
async def test_query_preserves_not_found_without_inventing_completion(robot, http, reason):
    payload = {"found": False, "run_id": "agent-old", "reason": reason}
    transport = http(payload)

    result = await invoke(robot.get_agent_run, "agent-old")

    assert result == payload
    request = transport.call_args.args[0]
    assert request.full_url.endswith("/api/v1/agent/runs/agent-old")
    assert request.get_method() == "GET" and request.data is None
    assert request.get_header("X-api-key") == "test-key"
    assert transport.call_count == 1


@pytest.mark.asyncio
async def test_cancel_is_scoped_and_does_not_claim_physical_stop(robot, http):
    payload = {
        "ok": True, "accepted": True, "run_id": "agent-current",
        "cancel_requested": True, "stop_confirmed": False,
        "execution_confirmed": False, "stage": "submitted",
    }
    transport = http(payload)

    result = await invoke(
        robot.cancel_agent_run, "agent-current",
        request_id="cancel-1", client_id="operator-console",
    )

    request = transport.call_args.args[0]
    assert request.full_url.endswith("/api/v1/agent/runs/agent-current/cancel")
    assert request.get_method() == "POST"
    body = json.loads(request.data)
    assert body == {"request_id": "cancel-1", "client_id": "operator-console"}
    assert AgentTaskCancelRequest.model_validate(body).request_id == "cancel-1"
    assert result.run_id == "agent-current"
    assert result.request_id == "cancel-1"
    assert result.accepted and result.ok
    assert result.raw == payload
    assert transport.call_count == 1


@pytest.mark.asyncio
@pytest.mark.parametrize("status", [200, 409])
async def test_cancel_rejection_retains_owner_reason(robot, http, status):
    payload = {
        "ok": False, "accepted": False, "run_id": "agent-old",
        "cancel_requested": False, "reason": "not_current",
    }
    transport = http(payload, status)

    result = await invoke(robot.cancel_agent_run, "agent-old")

    assert not result.ok and not result.accepted
    assert result.run_id == "agent-old"
    assert result.message == "not_current"
    assert result.raw == payload
    assert result.request_id == json.loads(transport.call_args.args[0].data)["request_id"]
    assert transport.call_count == 1


@pytest.mark.asyncio
async def test_instruction_conflict_keeps_current_run_and_reason(robot, http):
    payload = {
        "ok": False, "accepted": False, "run_id": "agent-current",
        "reason": "task_active", "state": "running",
    }
    transport = http(payload, 409)

    result = await invoke(robot.send_instruction, "follow blue", request_id="change-1")

    assert not result.ok and not result.accepted
    assert result.run_id == "agent-current"
    assert result.message == "task_active"
    assert result.raw == payload
    assert transport.call_count == 1


@pytest.mark.asyncio
async def test_query_http_error_is_exposed(robot, http):
    http({"detail": "not found"}, 404)

    with pytest.raises(RuntimeError, match=r"GET .*agent-1.*HTTP 404.*not found"):
        await invoke(robot.get_agent_run, "agent-1")


@pytest.mark.asyncio
async def test_query_connection_error_is_exposed(robot, monkeypatch):
    monkeypatch.setattr("urllib.request.urlopen", Mock(side_effect=TimeoutError("offline")))

    with pytest.raises(ConnectionError, match=r"agent-1.*offline"):
        await invoke(robot.get_agent_run, "agent-1")


@pytest.mark.asyncio
async def test_cancel_connection_error_stays_an_unconfirmed_failure(robot, monkeypatch):
    transport = Mock(side_effect=urllib.error.URLError("offline"))
    monkeypatch.setattr("urllib.request.urlopen", transport)

    result = await invoke(robot.cancel_agent_run, "agent-1", request_id="cancel-1")

    assert not result.ok and not result.accepted
    assert result.run_id is None
    assert result.request_id == "cancel-1"
    assert result.raw["error"] == "robot not reachable"
    assert result.raw["path"].endswith("/agent-1/cancel")
    assert "stop_confirmed" not in result.raw
    assert transport.call_count == 1


@pytest.mark.asyncio
@pytest.mark.parametrize("method_name", ["get_agent_run", "cancel_agent_run"])
async def test_scoped_operation_requires_a_run_id(robot, monkeypatch, method_name):
    transport = Mock()
    monkeypatch.setattr("urllib.request.urlopen", transport)

    with pytest.raises(ValueError, match="run_id is required"):
        await invoke(getattr(robot, method_name), " ")

    transport.assert_not_called()


@pytest.mark.asyncio
async def test_sdk_controls_round_trip_through_existing_gateway(robot, monkeypatch):
    gateway = GatewayModule()
    gateway.setup()
    owner = AgentPlannerModule()
    owner._active_run = AgentRun("follow red")
    run_id = owner._active_run.run_id
    gateway._all_modules = {"AgentPlannerModule": owner}
    try:
        with TestClient(gateway._app) as server:
            def urlopen(request, *, timeout):
                response = server.request(
                    request.get_method(), request.full_url, content=request.data,
                    headers=dict(request.header_items()),
                )
                if response.status_code >= 400:
                    raise urllib.error.HTTPError(
                        request.full_url, response.status_code, "rejected", {},
                        io.BytesIO(response.content),
                    )
                return io.BytesIO(response.content)

            monkeypatch.setattr("urllib.request.urlopen", urlopen)
            queried = await invoke(
                robot.send_instruction, "现在在干什么", run_id=run_id, control_only=True,
            )
            assert queried.ok and queried.run_id == run_id
            assert queried.raw["operation"] == "query"
            assert queried.raw["task_status"]["instruction"] == "follow red"
            status = await invoke(robot.get_agent_run, run_id)
            assert status["found"] and status["run_id"] == run_id

            rejected = await invoke(robot.cancel_agent_run, "old-run")
            assert not rejected.accepted
            assert rejected.raw["reason"] == "not_current"
            assert owner._active_run.state == "running"

            cancelled = await invoke(robot.cancel_agent_run, run_id)
            assert cancelled.ok and cancelled.run_id == run_id
            assert cancelled.raw["cancel_requested"] is True
            assert cancelled.raw["execution_confirmed"] is False
            status = await invoke(robot.get_agent_run, run_id)
            assert status["state"] == "cancelled"
            assert owner._active_run.run_id == run_id
    finally:
        gateway.stop()
