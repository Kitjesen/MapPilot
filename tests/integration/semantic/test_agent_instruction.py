"""Ordinary HTTP instructions cross the real Agent loop into owned domain tasks.

The model, native navigation endpoint, and vision verdicts are controlled test
doubles.  These tests prove task ownership and state flow; they are not model,
geometry, simulator, or robot evidence.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import json
import threading
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient
from tests.decision.test_semantic_target_verification import harness
from tests.gateway.test_gateway_commands import _endpoint, _payload

from decision.modules.agent_planner import AgentPlannerModule
from decision.modules.visual_servo import VisualServoModule
from gateway.gateway_module import GatewayModule
from gateway.schemas import AgentTaskCancelRequest, InstructionRequest
from runtime.msgs.geometry import Vector3
from runtime.msgs.semantic import Detection3D


def _tool(name: str, **arguments) -> dict:
    if name == "done":
        arguments.setdefault("success", True)
    return {
        "tool_calls": [{
            "id": f"test-{name}",
            "function": {"name": name, "arguments": json.dumps(arguments)},
        }]
    }


def _scripted_model(*responses: dict):
    remaining = iter(responses)
    calls: list[list[dict]] = []

    async def chat_with_tools(messages, **_kwargs):
        calls.append(messages)
        return next(remaining)

    return SimpleNamespace(chat_with_tools=chat_with_tools), calls


@pytest.fixture
def app(harness, monkeypatch):
    monkeypatch.setattr(AgentPlannerModule, "_init_llm", lambda self: None)
    agent = AgentPlannerModule(llm_backend="mock", timeout=3.0)
    agent.setup()
    agent.on_system_modules({"SemanticPlannerModule": harness.module})

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {
        "AgentPlannerModule": agent,
        "SemanticPlannerModule": harness.module,
    }
    yield SimpleNamespace(gateway=gateway, agent=agent, semantic=harness)
    gateway.stop()
    agent.stop()


def test_find_water_bottle_requires_owned_motion_and_visual_confirmation(app):
    """The real AgentLoop cannot report success at native arrival alone."""
    h = app.semantic
    h.module._goal_resolver.fast_resolve.return_value = SimpleNamespace(
        confidence=1.0,
        candidate_id="bottle-1",
        action="navigate",
    )
    h.observe(100.01, object_id="bottle-1", label="bottle")
    model, calls = _scripted_model(
        _tool("find_object", target="water bottle"),
        _tool("done", summary="water bottle visually confirmed"),
    )
    app.agent._llm_client = model
    h.goal_sent.clear()

    submit = _endpoint(app.gateway, "/api/v1/instruction")
    receipt = asyncio.run(submit(InstructionRequest(
        text="帮我找一瓶水",
        request_id="find-water-1",
        client_id="integration-test",
    )))
    run = app.agent._active_run

    assert receipt["executor"] == "agent"
    assert receipt["run_id"] == run.run_id
    assert h.goal_sent.wait(1.0)
    assert len(calls) == 1
    assert run.motion_attempted is True
    assert run.motion_success is False

    h.reach()
    assert run.future.done() is False
    h.observe(100.1, object_id="bottle-1", label="bottle")
    h.await_request()
    h.reply("match")
    assert run.future.done() is False
    h.observe(100.7, object_id="bottle-1", label="bottle")
    h.await_request()
    h.reply("match")
    run.future.result(timeout=2.0)

    query = _endpoint(app.gateway, "/api/v1/agent/runs/{run_id}")
    status = asyncio.run(query(run.run_id))
    assert len(calls) == 2
    assert status["found"] is True
    assert status["state"] == "done"
    assert status["motion_success"] is True
    assert status["navigation"]["success"] is True


def test_http_controls_preserve_real_owner_state_with_safety_stop(app):
    """Serialize idle/running/terminal controls through the actual HTTP app."""
    servo = VisualServoModule()
    app.semantic.module.on_system_modules({**app.semantic.modules, "VisualServoModule": servo})
    follow_started = threading.Event()
    servo.servo_status.subscribe(lambda _status: follow_started.set())
    model, calls = _scripted_model(_tool("follow_person", description="person in red"))
    app.agent._llm_client = model

    with TestClient(app.gateway._app) as client:
        request_number = 0

        def send(text, run_id="", *, control_only=False):
            nonlocal request_number
            request_number += 1
            return client.post("/api/v1/instruction", json={
                "text": text, "run_id": run_id, "control_only": control_only,
                "request_id": f"wire-control-{request_number}", "client_id": "local-asgi",
            })

        app.gateway._navigation_state = {"authority": "estop", "hold_reason": "operator_estop"}
        idle = send("当前状态", control_only=True)
        assert idle.status_code == 200
        assert idle.json()["operation"] == "query"
        assert idle.json()["accepted"] is True
        assert idle.json()["task_status"]["state"] == "idle"
        assert idle.json()["task_status"]["found"] is False
        assert app.agent._active_run is None

        blocked = send("跟随红衣服的人")
        assert blocked.status_code == 409
        assert blocked.json()["error"] == "safety_stop"
        assert app.agent._active_run is None

        app.gateway._navigation_state = None
        control_only = send("跟随红衣服的人", control_only=True)
        assert control_only.status_code == 409
        assert control_only.json()["accepted"] is False
        assert control_only.json()["reason"] == "motion_start_blocked"
        assert app.agent._active_run is None

        started = send("跟随红衣服的人")
        assert started.status_code == 200
        assert started.json()["operation"] == "start"
        assert started.json()["accepted"] is True
        run = app.agent._active_run
        assert started.json()["run_id"] == run.run_id
        assert follow_started.wait(1.0)
        follow = servo.get_servo_status()

        app.gateway._navigation_state = {"authority": "estop", "hold_reason": "operator_estop"}
        for text, operation in [("现在在干什么", "query"), ("继续跟着他", "keep")]:
            response = send(text, run.run_id, control_only=True)
            assert response.status_code == 200
            payload = response.json()
            assert payload["operation"] == operation and payload["accepted"] is True
            assert payload["run_id"] == run.run_id
            assert payload["task_status"]["found"] is True
            assert payload["task_status"]["instruction"] == "跟随红衣服的人"
            assert payload["task_status"]["navigation"]["follow_task_id"] == follow["follow_task_id"]
            assert payload["execution_confirmed"] is payload["motor_confirmed"] is False

        assert send("帮我找一瓶水", run.run_id).status_code == 409
        stale = send("停下", "old-run").json()
        assert stale["accepted"] is False and stale["reason"] == "not_current"
        assert run.cancelled is False

        cancelled = send("结束当前任务", run.run_id, control_only=True)
        assert cancelled.status_code == 200
        assert cancelled.json()["operation"] == "cancel"
        assert cancelled.json()["cancel_requested"] is True
        assert cancelled.json()["stop_confirmed"] is True
        assert cancelled.json()["task_status"]["found"] is True
        assert cancelled.json()["task_status"]["state"] == "cancelled"

        terminal = send("为什么停止", run.run_id, control_only=True).json()
        assert terminal["operation"] == "query"
        assert terminal["run_id"] == run.run_id
        assert terminal["task_status"]["state"] == "cancelled"
        unsupported = send("恢复任务", run.run_id).json()
        assert unsupported["accepted"] is False
        assert unsupported["reason"] == "unsupported_control"
        assert app.agent._active_run is run
        assert len(calls) == 1


def test_follow_cancel_and_stale_run_id_cannot_affect_replacement(app):
    """HTTP cancellation is scoped to the current Agent and its owned Follow."""
    app.agent._timeout = 0.03
    servo = VisualServoModule()
    app.semantic.module.on_system_modules({**app.semantic.modules, "VisualServoModule": servo})
    follow_started = threading.Event()
    servo.servo_status.subscribe(lambda _status: follow_started.set())
    model, calls = _scripted_model(_tool("follow_person", description="person in red"))
    app.agent._llm_client = model

    submit = _endpoint(app.gateway, "/api/v1/instruction")
    cancel = _endpoint(app.gateway, "/api/v1/agent/runs/{run_id}/cancel")
    first = asyncio.run(submit(InstructionRequest(
        text="跟随红衣服的人",
        request_id="follow-1",
        client_id="integration-test",
    )))
    old_run = app.agent._active_run
    assert follow_started.wait(1.0)
    follow = servo.get_servo_status()
    assert follow["follow_task_id"]
    with pytest.raises(concurrent.futures.TimeoutError):
        old_run.future.result(timeout=0.08)
    assert old_run.state == "running"

    for text, operation in [("现在在干什么", "query"), ("继续跟着他", "keep")]:
        control = _payload(asyncio.run(submit(InstructionRequest(
            text=text, run_id=first["run_id"],
            request_id=f"follow-{operation}", client_id="integration-test",
        ))))
        assert control["accepted"] is True
        assert control["operation"] == operation
        assert control["run_id"] == first["run_id"]
        assert control["task_status"]["found"] is True
        assert control["task_status"]["navigation"]["follow_task_id"] == follow["follow_task_id"]
        assert app.agent._active_run is old_run
        assert len(calls) == 1

    refused = _payload(asyncio.run(submit(InstructionRequest(
        text="帮我找一瓶水", run_id=first["run_id"],
        request_id="busy-search", client_id="integration-test",
    ))))
    assert refused["accepted"] is False
    assert refused["reason"] == "task_active"
    assert refused["run_id"] == first["run_id"]
    assert len(calls) == 1

    cancelled = _payload(asyncio.run(submit(InstructionRequest(
        text="停下", run_id=first["run_id"],
        request_id="cancel-follow-1", client_id="integration-test",
    ))))
    assert cancelled["operation"] == "cancel"
    assert cancelled["task_status"]["found"] is True
    assert cancelled["cancel_requested"] is True
    assert cancelled["stop_confirmed"] is True
    task = servo.get_follow_task(follow["follow_task_id"], follow["product_session_id"])
    assert task["terminal"] is True
    assert task["state"] == "cancelled"

    replacement_model, _replacement_calls = _scripted_model(
        _tool("done", summary="status returned"),
    )
    app.agent._llm_client = replacement_model
    second = asyncio.run(submit(InstructionRequest(
        text="报告状态",
        request_id="status-1",
        client_id="integration-test",
    )))
    new_run = app.agent._active_run
    new_run.future.result(timeout=2.0)
    stale = _payload(asyncio.run(cancel(
        old_run.run_id,
        AgentTaskCancelRequest(request_id="stale-cancel", client_id="integration-test"),
    )))

    assert first["run_id"] != second["run_id"]
    assert stale["cancel_requested"] is False
    assert stale["reason"] == "not_current"
    assert new_run.cancelled is False
    assert new_run.state == "done"


def test_external_follow_cancel_reaches_agent_without_model_restart(app):
    """A domain cancellation is terminal failure and cannot restart Follow."""
    servo = VisualServoModule()
    app.semantic.module.on_system_modules({**app.semantic.modules, "VisualServoModule": servo})
    follow_started = threading.Event()
    servo.servo_status.subscribe(lambda _status: follow_started.set())
    model, calls = _scripted_model(
        _tool("follow_person", description="person in red"),
        _tool("follow_person", description="person in blue"),
    )
    app.agent._llm_client = model

    submit = _endpoint(app.gateway, "/api/v1/instruction")
    receipt = asyncio.run(submit(InstructionRequest(
        text="跟随红衣服的人",
        request_id="external-cancel-follow",
        client_id="integration-test",
    )))
    run = app.agent._active_run
    assert follow_started.wait(1.0)
    ref = servo.get_servo_status()

    cancelled = servo.cancel_follow_task(
        ref["follow_task_id"],
        ref["product_session_id"],
        reason="operator_cancel",
    )
    assert cancelled["terminal"] is True
    owned = app.semantic.module.owned_instruction_status(run.run_id)
    assert owned == {
        "owned": True,
        "terminal": True,
        "success": False,
        "state": "cancelled",
        "follow_task_id": ref["follow_task_id"],
        "product_session_id": ref["product_session_id"],
        "phase": "cancelled",
        "reason": "operator_cancel",
        "result": "cancelled",
        "children": {},
        "details": owned["details"],
    }
    run.future.result(timeout=2.0)
    assert receipt["run_id"] == run.run_id
    assert len(calls) == 1
    assert run.cancelled is True
    assert run.state == "cancelled"
    assert servo.get_follow_task(ref["follow_task_id"], ref["product_session_id"])["state"] == "cancelled"

    replacement_model, _replacement_calls = _scripted_model(
        _tool("done", summary="status returned"),
    )
    app.agent._llm_client = replacement_model
    submit_status = asyncio.run(submit(InstructionRequest(
        text="报告状态",
        request_id="after-external-cancel",
        client_id="integration-test",
    )))
    replacement = app.agent._active_run
    replacement.future.result(timeout=2.0)
    assert submit_status["run_id"] != run.run_id
    assert replacement.state == "done"
    assert servo.cancel_follow_task(
        ref["follow_task_id"], ref["product_session_id"], reason="late_old_cancel",
    )["reason"] == "already_terminal"
    assert replacement.state == "done"


def test_follow_me_without_bound_visual_identity_requests_clarification(app):
    servo = VisualServoModule()
    app.semantic.module.on_system_modules({**app.semantic.modules, "VisualServoModule": servo})
    messages = []
    app.semantic.module.agent_message.subscribe(messages.append)
    model, calls = _scripted_model(
        _tool("follow_person", description="me"),
        _tool("done", success=False, summary="A visual target description is required."),
    )
    app.agent._llm_client = model

    submit = _endpoint(app.gateway, "/api/v1/instruction")
    receipt = _payload(asyncio.run(submit(InstructionRequest(
        text="跟随我", request_id="unbound-speaker", client_id="integration-test",
    ))))
    run = app.agent._active_run
    run.future.result(timeout=2.0)

    assert receipt["run_id"] == run.run_id
    assert len(calls) == 2
    assert any(
        message["role"] == "tool" and "follow_target_ambiguous" in message["content"]
        for message in calls[1]
    )
    assert servo.get_servo_status()["mode"] == "idle"
    assert not servo._follow_tasks
    assert len(app.semantic.goals) == 1
    assert any(message["phase"] == "follow_target" and "衣着" in message["text"] for message in messages)
    assert run.state == "failed"
    assert run.motion_success is False


def test_unknown_follow_reference_is_terminal_failure(app):
    """A missing retained Follow cannot be interpreted as successful motion."""
    servo = VisualServoModule()
    app.semantic.module.on_system_modules({**app.semantic.modules, "VisualServoModule": servo})
    app.semantic.module._instruction_owner = "agent-missing-follow"
    app.semantic.module._visual_handoff = True
    app.semantic.module._follow_ref = {
        "follow_task_id": "missing-task",
        "product_session_id": servo._product_session_id,
    }

    status = app.semantic.module.owned_instruction_status("agent-missing-follow")

    assert status["owned"] is True
    assert status["terminal"] is True
    assert status["success"] is False
    assert status["state"] == "FOLLOW_UNKNOWN"
    assert status["reason"] == "follow_task_not_found"

