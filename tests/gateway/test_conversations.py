"""Conversation HTTP messages bypass motion gates until an owner tool executes."""

import asyncio
import base64
import json
import threading
from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from decision.llm.client import LLMConfig
from decision.modules.agent_planner import AgentPlannerModule
from decision.tasks.sdk_agent import TurnResult
from gateway.routes.commands import register_command_routes
from gateway.services.commands import CommandJournal, run_control_command


@pytest.fixture
def gateway(tmp_path, monkeypatch):
    agent = AgentPlannerModule(conversation_db_path=str(tmp_path / "chat.db"))
    agent._llm_client = SimpleNamespace(config=LLMConfig())
    monkeypatch.setattr(agent, "conversation_available", lambda: True)
    gw = SimpleNamespace(_all_modules={"AgentPlannerModule": agent}, _navigation_state=None,
                         _command_journal=CommandJournal(), _lease=None)
    gw._state_lock = threading.RLock()
    gw._publish_command_ack = lambda *args, **kwargs: None
    gw.push_event = lambda event: None
    gw._run_control_command = lambda command, body, action, **kwargs: run_control_command(
        gw, command, body, action, **kwargs)
    app = FastAPI()
    register_command_routes(app, gw)
    with TestClient(app) as client:
        yield SimpleNamespace(agent=agent, gw=gw, client=client)
    agent.stop()


def finish(gateway, receipt):
    import time

    path = f"/api/v1/agent/conversations/{receipt['conversation_id']}/turns/{receipt['turn_id']}"
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        turn = gateway.client.get(path).json()
        if turn["state"] != "running":
            return turn
        time.sleep(0.01)
    pytest.fail("turn did not finish")


def test_queue_http_edits_cancels_and_rejects_started_turn(gateway):
    store = gateway.agent._conversation_store()
    cid = store.create()["conversation_id"]
    first, _ = store.accept(cid, "active", "working", None)
    queued, _ = store.accept(cid, "waiting", "later", None, enqueue=True)
    path = f"/api/v1/agent/conversations/{cid}"
    queue_path = f"{path}/queue/{queued['turn_id']}"
    assert gateway.client.post(f"{queue_path}/promote").status_code == 200
    assert gateway.client.patch(queue_path, json={"text": "updated"}).status_code == 200
    assert gateway.client.get(path).json()["messages"][-1]["text"] == "updated"
    assert gateway.client.patch(queue_path, json={"text": " "}).status_code == 400
    assert gateway.client.delete(queue_path).json()["state"] == "cancelled"
    assert gateway.client.patch(f"{path}/queue/{first['turn_id']}", json={"text": "late"}).status_code == 409
    assert gateway.client.get(f"{path}?before_seq=0").status_code == 422


def test_http_persists_messages_and_receipts_without_starting_motion(gateway, monkeypatch):
    async def runner(text, history, **kwargs):
        return TurnResult("reply", "Hello", [*history, {"role": "user", "content": text},
                                            {"role": "assistant", "content": "Hello"}])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    created = gateway.client.post("/api/v1/agent/conversations", json={})
    assert created.status_code == 201
    cid = created.json()["conversation_id"]
    path = f"/api/v1/agent/conversations/{cid}"
    body = {"message_id": "retry", "text": "hello"}
    accepted = gateway.client.post(f"{path}/messages", json=body)
    assert accepted.status_code == 202 and accepted.json()["state"] == "running"
    assert accepted.json()["started_at"] > 0
    turn = finish(gateway, accepted.json())
    assert turn["kind"] == "reply" and "run_id" not in turn
    assert turn["finished_at"] >= turn["started_at"]
    replay = gateway.client.post(f"{path}/messages", json=body).json()
    assert replay["turn_id"] == turn["turn_id"]
    snapshot = gateway.client.get(path).json()
    assert snapshot["messages"][0]["id"] == "retry"
    assert [message["role"] for message in snapshot["messages"]] == ["user", "assistant"]
    assert all(message["created_at"] > 0 for message in snapshot["messages"])
    assert snapshot["server_time"] >= turn["finished_at"]
    assert snapshot["active_run"] is None
    assert gateway.client.post(f"{path}/messages", json={**body, "text": "changed"}).status_code == 409


def test_agent_missing_and_history_identity_errors_are_explicit(gateway):
    assert gateway.client.get("/api/v1/agent/conversations/missing").status_code == 404
    assert gateway.client.post("/api/v1/agent/conversations/x/messages", json={
        "message_id": "blank", "text": " "}).status_code == 422
    gateway.gw._all_modules = {}
    response = gateway.client.post("/api/v1/agent/conversations", json={})
    assert response.status_code == 503 and response.json()["detail"]["reason"] == "agent_unavailable"


def test_http_keeps_selected_quote_separate_from_question(gateway, monkeypatch):
    inputs = []

    async def runner(text, history, **kwargs):
        inputs.append(text)
        return TurnResult("reply", "第一段。\n\n路线会显示在地图上。", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = gateway.client.post("/api/v1/agent/conversations").json()["conversation_id"]
    path = f"/api/v1/agent/conversations/{cid}"
    first = gateway.client.post(f"{path}/messages", json={"message_id": "one", "text": "介绍一下"})
    finish(gateway, first.json())
    original = gateway.client.get(path).json()["messages"][-1]
    body = {"message_id": "two", "text": "是什么意思？", "reply_to_message_id": original["id"],
            "reply_quote": "路线会显示在地图上。"}
    response = gateway.client.post(f"{path}/messages", json=body)
    assert response.status_code == 202
    turn = finish(gateway, response.json())
    assert turn["reply_quote"] == body["reply_quote"]
    assert "第一段" not in inputs[-1] and body["reply_quote"] in inputs[-1]
    snapshot = gateway.client.get(path).json()
    assert snapshot["messages"][-2]["text"] == body["text"]
    assert snapshot["turns"][-1]["reply_quote"] == body["reply_quote"]
    assert gateway.client.post(f"{path}/messages", json=body).json()["turn_id"] == turn["turn_id"]
    assert gateway.client.post(f"{path}/messages", json={**body, "reply_quote": "第一段。"}).status_code == 409
    for invalid in ({"reply_quote": " "}, {"reply_quote": 3}, {"reply_to_message_id": None}):
        assert gateway.client.post(f"{path}/messages", json={**body, "message_id": "invalid", **invalid}).status_code in (400, 422)


def test_safety_stop_allows_chat_but_blocks_tool_motion(gateway, monkeypatch):
    gateway.gw._navigation_state = {"authority": "estop"}

    async def runner(text, history, invoke_tool, **kwargs):
        if text == "move":
            result = json.loads(await invoke_tool("navigate_to", {"x": 1, "y": 2}, "call"))
            assert result["reason"] == "safety_stop"
        return TurnResult("reply", "reply", [*history, {"role": "user", "content": text}])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = gateway.client.post("/api/v1/agent/conversations").json()["conversation_id"]
    for index, text in enumerate(["hello", "move"]):
        response = gateway.client.post(f"/api/v1/agent/conversations/{cid}/messages",
                                       json={"message_id": str(index), "text": text, "permission_mode": "full_access"})
        assert response.status_code == 202
        turn = finish(gateway, response.json())
        assert turn["kind"] == ("reply" if text == "hello" else "failed")
    assert gateway.agent._active_run is None


def test_full_length_message_id_rechecks_motion_guard_for_each_tool(gateway, monkeypatch):
    calls = []

    async def motion(**kwargs):
        calls.append(kwargs)
        kwargs["run"].motion_attempted = kwargs["run"].motion_success = True
        return "native reached"

    async def runner(text, history, invoke_tool, **kwargs):
        await invoke_tool("navigate_to", {"x": 1, "y": 2}, "first")
        gateway.gw._navigation_state = {"authority": "estop"}
        rejected = json.loads(await invoke_tool("navigate_to", {"x": 3, "y": 4}, "second"))
        assert rejected["reason"] == "safety_stop"
        return TurnResult("reply", "Stopped.", [])

    monkeypatch.setattr(gateway.agent, "_tool_navigate_to", motion)
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = gateway.client.post("/api/v1/agent/conversations").json()["conversation_id"]
    receipt = gateway.client.post(f"/api/v1/agent/conversations/{cid}/messages", json={
        "message_id": "m" * 128, "text": "move twice", "permission_mode": "full_access"}).json()
    assert finish(gateway, receipt)["failure_reason"] == "safety_stop"
    assert len(calls) == 1 and gateway.agent._active_run.state == "failed"


def test_http_two_turns_use_real_sdk_and_keep_clarification_context(gateway, monkeypatch):
    pytest.importorskip("agents")
    from functools import partial

    from decision.llm.client import LLMConfig
    from decision.tasks.sdk_agent import run_turn
    from tests.decision.test_sdk_agent import FakeModel, call, message

    model = FakeModel([
        [call("ask_user", {"question": "Which room?"}, "clarify")],
        [call("get_agent_status", {}, "status")],
        [message("No motion task is active.")],
    ])
    gateway.agent._llm_client.config = LLMConfig()
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", partial(run_turn, model=model))
    cid = gateway.client.post("/api/v1/agent/conversations").json()["conversation_id"]
    path = f"/api/v1/agent/conversations/{cid}"
    for index, text in enumerate(["Tell me about the room", "The blue room"]):
        receipt = gateway.client.post(f"{path}/messages", json={"message_id": str(index), "text": text})
        assert receipt.status_code == 202
        turn = finish(gateway, receipt.json())
        assert turn["state"] == "completed", turn
        assert turn["kind"] == ("clarification" if index == 0 else "reply")
    assert any(item.get("type") == "function_call_output" and "Which room?" in item["output"]
               for item in model.inputs[1])
    assert sum(item.get("content") == "Tell me about the room" for item in model.inputs[1]) == 1
    snapshot = gateway.client.get(path).json()
    assert [item["kind"] for item in snapshot["messages"] if item["role"] == "assistant"] == ["clarification", "reply"]
    assert any(item["role"] == "tool" and item["kind"] == "get_agent_status" for item in snapshot["messages"])
    assert gateway.agent._active_run is None


def test_http_options_attachments_and_selection_errors(gateway, monkeypatch):
    async def runner(text, history, **kwargs):
        assert kwargs["attachments"][0]["data"] == b"a note"
        return TurnResult("reply", "Read it.", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    options = gateway.client.get("/api/v1/agent/options")
    assert options.status_code == 200 and options.json()["available"] is True
    assert options.json()["default_model"] == "gpt-4o-mini"
    cid = gateway.client.post("/api/v1/agent/conversations").json()["conversation_id"]
    path = f"/api/v1/agent/conversations/{cid}"
    body = {"attachment_id": "file", "name": "notes.txt", "mime_type": "text/plain",
            "data_base64": base64.b64encode(b"a note").decode()}
    upload = gateway.client.post(f"{path}/attachments", json=body)
    assert upload.status_code == 201 and upload.json()["size"] == 6
    assert gateway.client.post(f"{path}/attachments", json=body).json() == upload.json()
    assert gateway.client.post(f"{path}/attachments", json={**body, "name": "changed.txt"}).status_code == 409
    invalid = gateway.client.post(f"{path}/attachments", json={**body, "attachment_id": "bad",
                                                               "data_base64": "invalid"})
    assert invalid.status_code == 400
    assert gateway.client.post(f"{path}/messages", json={"message_id": "failed-file",
        "attachment_ids": ["bad"]}).status_code == 404
    message = {"message_id": "file-message", "attachment_ids": ["file"]}
    accepted = gateway.client.post(f"{path}/messages", json=message)
    assert accepted.status_code == 202 and finish(gateway, accepted.json())["kind"] == "reply"
    assert gateway.client.get(path).json()["messages"][0]["attachments"] == [upload.json()]
    assert gateway.client.post(f"{path}/messages", json={"message_id": "wrong", "text": "hi",
                                                         "model": "unsupported-model"}).status_code == 400
    assert gateway.client.post(f"{path}/messages", json={"message_id": "wrong", "text": "hi",
                                                         "reasoning": "unsupported"}).status_code == 400
    assert gateway.client.post(f"{path}/messages", json={**message, "attachment_ids": ["missing"]}).status_code == 404
    assert gateway.client.post(f"{path}/messages", json={**message, "attachment_ids": ["file"] * 5}).status_code == 422
    gateway.gw._all_modules = {}
    assert gateway.client.get("/api/v1/agent/options").status_code == 503


def test_long_conversation_input_keeps_full_text_and_motion_guard(gateway, monkeypatch):
    text = "move " + "x" * 7995
    seen = []
    gateway.gw._navigation_state = {"authority": "estop"}

    async def runner(text, history, invoke_tool, **kwargs):
        seen.append(text)
        rejection = json.loads(await invoke_tool("navigate_to", {"x": 1, "y": 2}, "call"))
        assert rejection["reason"] == "safety_stop"
        return TurnResult("reply", "Stopped.", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = gateway.client.post("/api/v1/agent/conversations").json()["conversation_id"]
    path = f"/api/v1/agent/conversations/{cid}"
    accepted = gateway.client.post(f"{path}/messages", json={"message_id": "long", "text": text,
                                                           "permission_mode": "full_access"})
    assert accepted.status_code == 202
    assert finish(gateway, accepted.json())["failure_reason"] == "safety_stop"
    assert seen == [text] and gateway.client.get(path).json()["messages"][0]["text"] == text
    assert gateway.client.post(f"{path}/messages", json={"message_id": "too-long", "text": text + "x"}).status_code == 422


def test_http_approval_is_scoped_and_rechecks_motion_safety(gateway, monkeypatch):
    import time

    async def runner(text, history, invoke_tool, **kwargs):
        result = json.loads(await invoke_tool("navigate_to", {"x": 1, "y": 2}, "motion"))
        assert result["reason"] == "safety_stop"
        return TurnResult("reply", "Stopped.", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    options = gateway.client.get("/api/v1/agent/options").json()
    assert options["permission_modes"] == ["read_only", "default", "full_access"]
    assert options["default_permission_mode"] == "default"
    cid = gateway.client.post("/api/v1/agent/conversations").json()["conversation_id"]
    path = f"/api/v1/agent/conversations/{cid}"
    receipt = gateway.client.post(f"{path}/messages", json={"message_id": "move", "text": "go there"}).json()
    deadline = time.monotonic() + 3
    approvals = []
    while time.monotonic() < deadline and not approvals:
        approvals = gateway.client.get(path).json()["approvals"]
        time.sleep(0.01)
    assert approvals and gateway.agent._active_run is None
    approval_id = approvals[0]["approval_id"]
    other = gateway.client.post("/api/v1/agent/conversations").json()["conversation_id"]
    assert gateway.client.post(f"/api/v1/agent/conversations/{other}/approvals/{approval_id}",
                               json={"approved": True}).status_code == 404
    assert gateway.client.post(f"{path}/approvals/{approval_id}", json={"approved": "yes"}).status_code == 422
    gateway.gw._navigation_state = {"authority": "estop"}
    decision = gateway.client.post(f"{path}/approvals/{approval_id}", json={"approved": True})
    assert decision.status_code == 200 and decision.json()["state"] == "approved"
    assert finish(gateway, receipt)["failure_reason"] == "safety_stop"
    assert gateway.agent._active_run is None
    assert gateway.client.post(f"{path}/approvals/{approval_id}", json={"approved": True}).json() == decision.json()
    assert gateway.client.post(f"{path}/messages", json={"message_id": "invalid", "text": "hi",
                                                       "permission_mode": "unknown"}).status_code == 422


def test_codex_routes_preserve_login_identity_and_hide_credentials(gateway, monkeypatch, tmp_path):
    import httpx

    from decision.llm import codex

    def handle(request):
        assert request.url.path.endswith("/deviceauth/usercode")
        return httpx.Response(200, json={"user_code": "TEST-CODE", "device_auth_id": "private-device", "interval": 5})

    account = codex.CodexAccount(tmp_path / "account.json", transport=httpx.MockTransport(handle))
    monkeypatch.setattr(codex, "account", account)
    path = "/api/v1/agent/connections/codex"
    response = gateway.client.get(path)
    assert response.status_code == 200 and response.json()["connected"] is False
    assert response.headers["cache-control"] == "no-store"
    login = gateway.client.post(f"{path}/login").json()
    assert "private-device" not in json.dumps(login)
    assert gateway.client.post(f"{path}/login").json()["id"] == login["id"]
    assert gateway.client.post(f"{path}/login/{login['id']}/poll").json()["state"] == "pending"
    assert gateway.client.delete(f"{path}/login/{login['id']}").json()["state"] == "cancelled"
    assert gateway.client.post(f"{path}/login/other/poll").status_code == 404
    assert gateway.client.delete(path).json()["connected"] is False
    assert gateway.agent._active_run is None
