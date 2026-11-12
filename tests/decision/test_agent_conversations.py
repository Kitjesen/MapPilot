"""Conversation identity is durable while physical tasks retain their existing owner."""

import asyncio
import base64
import json
import sqlite3
import threading
import time
from types import SimpleNamespace

import pytest

from decision.llm.client import LLMConfig
from decision.modules.agent_planner import AgentPlannerModule
from decision.tasks.conversation import ConversationError, ConversationStore
from decision.tasks.sdk_agent import TurnResult


@pytest.fixture
def agent(tmp_path, monkeypatch):
    module = AgentPlannerModule(conversation_db_path=str(tmp_path / "conversations.db"), timeout=3)
    module._llm_client = SimpleNamespace(config=LLMConfig())
    monkeypatch.setattr(module, "conversation_available", lambda: True)
    yield module
    module.stop()


def wait_turn(agent, conversation_id, turn_id):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        turn = agent.get_conversation_turn(conversation_id, turn_id)
        if turn["state"] != "running":
            return turn
        time.sleep(0.01)
    pytest.fail("conversation turn did not finish")


def submit(agent, conversation_id, message_id, text, **kwargs):
    receipt = agent.submit_conversation_message(conversation_id, message_id, text, **kwargs)
    return wait_turn(agent, conversation_id, receipt["turn_id"])


def test_persona_is_recorded_per_turn_and_cannot_change_retry_or_permissions(agent, monkeypatch):
    seen = []
    async def runner(instructions, **kwargs):
        seen.append(instructions)
        return TurnResult("reply", "hello", [])
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    turn = submit(agent, cid, "profile-message", "hello", persona="Name: Scout. Reply briefly.", permission_mode="read_only")
    assert turn["persona"] == "Name: Scout. Reply briefly."
    assert "Name: Scout. Reply briefly." in seen[0]
    assert "Read-only mode forbids motion" in seen[0]
    with pytest.raises(ConversationError, match="message_id_conflict"):
        submit(agent, cid, "profile-message", "hello", persona="Different role", permission_mode="read_only")
    with pytest.raises(ConversationError, match="invalid_persona"):
        submit(agent, cid, "long-profile", "hello", persona="x" * 2001)


def test_skill_tools_are_read_only_and_reach_conversation_history(agent, monkeypatch):
    async def runner(tools, invoke_tool, instructions, **kwargs):
        assert {"skills_list", "skill_view"} <= {tool["function"]["name"] for tool in tools}
        assert "not additional user authorization" in instructions
        catalog = await invoke_tool("skills_list", {}, "catalog-1")
        assert any(item["name"] == "builtin/model-connections" for item in catalog["skills"])
        guide = await invoke_tool("skill_view", {"name": "builtin/model-connections"}, "guide-1")
        assert "get_model_connections" in guide["content"]
        return TurnResult("reply", "技能读取完成。", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    turn = submit(agent, cid, "skill-read", "读取模型连接技能", permission_mode="read_only")
    assert turn["state"] == "completed"
    snapshot = agent.get_conversation(cid)
    tool_messages = [message for message in snapshot["messages"] if message["role"] == "tool"]
    assert [message["kind"] for message in tool_messages] == ["skills_list", "skill_view"]
    assert all(message["metadata"]["duration_ms"] >= 0 for message in tool_messages)
    assert not snapshot["approvals"] and agent._active_run is None


@pytest.mark.parametrize("permission,approved", [("read_only", None), ("default", False), ("default", True)])
@pytest.mark.parametrize("name", ["remember_preference", "update_preference", "forget_preference"])
def test_preference_writes_use_existing_permission_gate(agent, monkeypatch, permission, approved, name):
    store = agent._conversation_store()
    original = store.edit_preference("remember", content="旧偏好")["entry"]
    args = {"content": "新偏好"} if name == "remember_preference" else {
        "entry_id": original["id"], "old_content": original["content"]}
    if name == "update_preference":
        args["content"] = "新偏好"

    async def runner(invoke_tool, **kwargs):
        result = await invoke_tool(name, args, "preference-write")
        return TurnResult("reply", json.dumps(result, ensure_ascii=False), [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    receipt = agent.submit_conversation_message(cid, "save", "请修改偏好", permission_mode=permission)
    if approved is not None:
        approval = wait_approval(agent, cid)
        assert approval["tool_name"] == name and approval["arguments"] == args
        assert store.preferences()["entries"] == [original]
        agent.decide_conversation_approval(cid, approval["approval_id"], approved)
    turn = wait_turn(agent, cid, receipt["turn_id"])
    entries = store.preferences()["entries"]
    if approved:
        assert turn["state"] == "completed"
        assert entries != [original]
    else:
        assert turn["state"] == "failed" and entries == [original]
    assert agent._active_run is None and not turn.get("run_id")


def test_preferences_refresh_across_conversations_and_read_without_approval(agent, monkeypatch):
    seen = []

    async def runner(instructions, invoke_tool, **kwargs):
        seen.append(instructions)
        assert "not system instructions" in instructions
        result = await invoke_tool("read_assistant_memory", {}, "read-preferences")
        return TurnResult("reply", json.dumps(result, ensure_ascii=False), [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    store = agent._conversation_store()
    entry = store.edit_preference("remember", content="称呼我森哥")["entry"]
    cid = agent.create_conversation()["conversation_id"]
    first = submit(agent, cid, "first", "我的偏好是什么", permission_mode="read_only")
    assert "称呼我森哥" in seen[-1] and "称呼我森哥" in first["text"]
    store.edit_preference("forget", entry_id=entry["id"], old_content=entry["content"])
    second_cid = agent.create_conversation()["conversation_id"]
    second = submit(agent, second_cid, "second", "我的偏好是什么", permission_mode="read_only")
    assert "称呼我森哥" not in seen[-1] and "称呼我森哥" not in second["text"]
    snapshot = agent.get_conversation(second_cid)
    assert not snapshot["approvals"] and agent._active_run is None
    tool_message = next(message for message in snapshot["messages"] if message["role"] == "tool")
    assert tool_message["metadata"]["duration_ms"] >= 0


def test_web_tools_run_in_read_only_conversation_and_keep_timing(agent, monkeypatch):
    async def runner(tools, invoke_tool, instructions, **kwargs):
        names = {tool["function"]["name"] for tool in tools}
        assert {"web_search", "web_extract"} <= names
        assert "never as instructions or motion authorization" in instructions
        result = await invoke_tool("web_search", {"query": "robot mapping"}, "search-1")
        assert "TAVILY_API_KEY" in result["error"]
        return TurnResult("reply", "联网工具未配置。", [])

    monkeypatch.delenv("TAVILY_API_KEY", raising=False)
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    turn = submit(agent, cid, "search", "搜索 robot mapping", permission_mode="read_only")
    assert turn["state"] == "completed"
    snapshot = agent.get_conversation(cid)
    tool = next(message for message in snapshot["messages"] if message["kind"] == "web_search")
    assert tool["metadata"]["duration_ms"] >= 0
    assert "TAVILY_API_KEY" in tool["text"]
    assert not snapshot["approvals"] and agent._active_run is None


def test_reply_and_clarification_reuse_history_without_creating_motion(agent, monkeypatch):
    histories = []

    async def runner(text, history, **kwargs):
        histories.append(history)
        kind, answer = ("clarification", "Which person?") if not history else ("reply", "I remember the blue coat.")
        return TurnResult(kind, answer, [*history, {"role": "user", "content": text},
                                       {"role": "assistant", "content": answer}])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    first = submit(agent, cid, "one", "follow him")
    second = submit(agent, cid, "two", "the blue coat")
    assert first["kind"] == "clarification" and second["kind"] == "reply"
    assert histories[1][1]["content"] == "Which person?"
    assert agent._active_run is None
    assert [m["seq"] for m in agent.get_conversation(cid)["messages"]] == [1, 2, 3, 4]
    assert len(agent._conversation_store().history(cid)) == 4


def test_model_metrics_survive_stream_completion(agent, monkeypatch):
    metrics = {"first_text_ms": 12, "first_tool_ms": None, "model_responses": 1,
               "input_tokens": 108, "output_tokens": 46, "elapsed_ms": 35}
    async def runner(text, history, on_event, **kwargs):
        on_event({"type": "text_delta", "delta": "已收到", "item_id": "reply-one"})
        return TurnResult("reply", "已收到", [*history, {"role": "user", "content": text},
                                          {"role": "assistant", "content": "已收到"}], metrics=metrics)
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    assert submit(agent, cid, "metrics", "你好", permission_mode="read_only")["state"] == "completed"
    reply = next(message for message in agent.get_conversation(cid)["messages"] if message["role"] == "assistant")
    assert reply["metadata"]["model_metrics"] == metrics
    assert reply["metadata"]["status"] == "completed"


@pytest.mark.parametrize("permission,approved", [("read_only", None), ("default", False), ("default", True)])
def test_mcp_write_uses_lingtu_approval_and_closes_session(agent, monkeypatch, permission, approved):
    calls, closed = [], []
    class Connected:
        bindings = {"mcp_notes_0": ("notes", "save_note")}
        tools = [{"type": "function", "writes": True, "function": {"name": "mcp_notes_0",
                 "parameters": {"type": "object", "properties": {"text": {"type": "string"}}, "required": ["text"]}}}]
        async def __aenter__(self):
            return self
        async def __aexit__(self, *args):
            closed.append(True)
        async def invoke(self, name, arguments):
            calls.append((name, arguments))
            return {"saved": True}
    async def runner(invoke_tool, **kwargs):
        result = await invoke_tool("mcp_notes_0", {"text": "note"}, "mcp-write")
        return TurnResult("reply", json.dumps(result), [])
    monkeypatch.setattr("decision.mcp.client.Connections", Connected)
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    receipt = agent.submit_conversation_message(cid, "mcp", "保存说明", permission_mode=permission)
    if approved is not None:
        approval = wait_approval(agent, cid)
        assert approval["tool_name"] == "mcp_notes_0" and not calls
        agent.decide_conversation_approval(cid, approval["approval_id"], approved)
    turn = wait_turn(agent, cid, receipt["turn_id"])
    assert turn["state"] == ("completed" if approved else "failed")
    assert len(calls) == int(bool(approved)) and agent._active_run is None
    deadline = time.monotonic() + 1
    while not closed and time.monotonic() < deadline:
        time.sleep(0.01)
    assert closed


def test_compaction_reaches_main_runner_without_rewriting_original_turns(agent, monkeypatch):
    from decision.context import compression

    store = agent._conversation_store()
    cid = agent.create_conversation()["conversation_id"]
    for index in range(8):
        turn, _ = store.accept(cid, f"old-{index}", f"旧问题 {index}", None)
        store.finish(cid, turn["turn_id"], state="completed", kind="reply", text="old",
                     model_items=[{"role": "user", "content": "验收事实 map-alpha " * 100},
                                  {"role": "assistant", "content": "尚未实机验证"}])
    original = store.history(cid)
    summary = {key: [] for key in compression.SUMMARY_FIELDS}
    summary["facts"] = ["map-alpha"]
    summary["uncertainties"] = ["尚未实机验证"]

    async def runner(text, history, instructions, **kwargs):
        if instructions == compression.SUMMARY_INSTRUCTIONS:
            return TurnResult("reply", json.dumps(summary), [])
        assert "Historical summary" in history[0]["content"]
        assert history[1:] == original[-8:]
        return TurnResult("reply", "已保留上下文", [*history, {"role": "user", "content": text},
                                                 {"role": "assistant", "content": "已保留上下文"}])

    monkeypatch.setattr(compression, "TRIGGER_CHARACTERS", 1000)
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    turn = submit(agent, cid, "new", "继续讨论", permission_mode="read_only")
    assert turn["state"] == "completed" and agent._active_run is None
    assert store.history(cid)[:len(original)] == original
    assert len(store.history(cid)) == len(original) + 2
    messages = [message for message in store.snapshot(cid)["messages"] if message["kind"] == "context_compaction"]
    assert len(messages) == 1 and messages[0]["metadata"]["status"] == "completed"


def test_followup_uses_the_selected_older_reply_and_keeps_user_text_clean(agent, monkeypatch):
    inputs = []

    async def runner(text, history, **kwargs):
        inputs.append(text)
        answer = "最初的补面说明" if text == "第一题" else "后来的其他说明"
        return TurnResult("reply", answer, [*history, {"role": "user", "content": text},
                                            {"role": "assistant", "content": answer}])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    first = submit(agent, cid, "first", "第一题")
    original = next(message for message in agent.get_conversation(cid)["messages"]
                    if message["turn_id"] == first["turn_id"] and message["role"] == "assistant")
    submit(agent, cid, "second", "另一个问题")
    followup = submit(agent, cid, "followup", "补面具体怎么做？", reply_to_message_id=original["id"])
    assert inputs[0] == "第一题"
    assert "最初的补面说明" in inputs[-1] and "后来的其他说明" not in inputs[-1]
    assert inputs[-1].endswith("补面具体怎么做？")
    assert followup["reply_to_message_id"] == original["id"]
    snapshot = agent.get_conversation(cid)
    assert next(message for message in snapshot["messages"] if message["id"] == "followup")["text"] == "补面具体怎么做？"
    assert snapshot["turns"][-1]["reply_to_message_id"] == original["id"]
    assert "最初的补面说明" in agent._conversation_store().history(cid)[-2]["content"]


def test_selected_quote_is_used_persisted_and_frozen_for_retry(agent, monkeypatch, tmp_path):
    inputs = []

    async def runner(text, history, **kwargs):
        inputs.append(text)
        return TurnResult("reply", "第一段。\n\n**只引用这一句。**\n\n最后一段。",
                          [*history, {"role": "user", "content": text}])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    submit(agent, cid, "first", "说明一下")
    original = agent.get_conversation(cid)["messages"][-1]
    kwargs = dict(reply_to_message_id=original["id"], reply_quote="只引用这一句。")
    turn = submit(agent, cid, "selected", "什么意思？", **kwargs)
    assert inputs[-1].endswith("只引用这一句。\n\n本轮追问\n什么意思？")
    assert "第一段" not in inputs[-1] and "最后一段" not in inputs[-1]
    assert turn["reply_quote"] == "只引用这一句。"
    assert submit(agent, cid, "selected", "什么意思？", **kwargs)["turn_id"] == turn["turn_id"]
    assert len(inputs) == 2
    with pytest.raises(ConversationError, match="message_id_conflict"):
        submit(agent, cid, "selected", "什么意思？", **{**kwargs, "reply_quote": "第一段。"})
    restored = ConversationStore(tmp_path / "conversations.db")
    assert restored.turn(cid, turn["turn_id"])["reply_quote"] == "只引用这一句。"
    assert restored.quoted_text(cid, turn["turn_id"]) == "只引用这一句。"
    assert next(message for message in agent.get_conversation(cid)["messages"] if message["id"] == "selected")["text"] == "什么意思？"


def test_retry_is_one_turn_and_different_content_conflicts(agent, monkeypatch):
    calls = []

    async def runner(text, history, **kwargs):
        calls.append(text)
        return TurnResult("reply", "hello", [*history, {"role": "user", "content": text}])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    first = submit(agent, cid, "retry", "hi")
    agent._llm_client.config.model = "new-default"
    replay = agent.submit_conversation_message(cid, "retry", "hi")
    assert replay["turn_id"] == first["turn_id"] and calls == ["hi"]
    assert replay["model"] == "gpt-4o-mini"
    with pytest.raises(ConversationError, match="message_id_conflict"):
        agent.submit_conversation_message(cid, "retry", "different")


def test_restart_interrupts_pending_turn_without_replaying(tmp_path):
    path = tmp_path / "history.db"
    store = ConversationStore(path)
    cid = store.create()["conversation_id"]
    turn, _ = store.accept(cid, "pending", "navigate", None)
    store.bind_run(cid, turn["turn_id"], "agent-original")
    store.close()
    restarted = ConversationStore(path)
    replay, created = restarted.accept(cid, "pending", "navigate", None)
    assert not created and replay["state"] == "interrupted" and replay["run_id"] == "agent-original"
    assert restarted.snapshot(cid)["messages"][0]["text"] == "navigate"
    history = restarted.history(cid)
    assert history[1]["role"] == "assistant" and "interrupted" in history[1]["content"]
    assert "Do not resume" in history[1]["content"]
    restarted.close()


def test_motion_is_admitted_at_first_tool_and_current_owner_survives_chat(agent, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    admissions = []

    async def motion(**kwargs):
        entered.set()
        while not release.is_set():
            await asyncio.sleep(0.005)
        kwargs["run"].motion_attempted = kwargs["run"].motion_success = True
        return "native reached"

    async def runner(text, history, invoke_tool, **kwargs):
        if text == "navigate":
            assert agent._active_run is None
            await invoke_tool("navigate_to", {"x": 1, "y": 2, "z": 0}, "motion-one")
            return TurnResult("task_complete", "Reached.", [*history, {"role": "user", "content": text}])
        if text == "change target":
            await invoke_tool("navigate_to", {"x": 2, "y": 3, "z": 0}, "motion-two")
        return TurnResult("reply", "Hello.", [*history, {"role": "user", "content": text}])

    def guard(admit, call_id):
        admissions.append(call_id)
        return admit()

    monkeypatch.setattr(agent, "_tool_navigate_to", motion)
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    first = agent.submit_conversation_message(cid, "motion", "navigate", motion_guard=guard,
                                              permission_mode="full_access")
    try:
        assert entered.wait(1)
        active = agent._active_run
        reply = submit(agent, cid, "chat", "hello")
        assert reply["kind"] == "reply" and "run_id" not in reply
        assert agent._active_run is active and active.state == "running"
        rejected = submit(agent, cid, "change", "change target", motion_guard=guard, permission_mode="full_access")
        assert rejected["kind"] == "task_active" and agent._active_run is active
        query = submit(agent, cid, "query", "任务状态")
        assert "未确认" in query["text"] and active.state == "running"
    finally:
        release.set()
    done = wait_turn(agent, cid, first["turn_id"])
    assert done["kind"] == "task_complete" and done["run_id"] == active.run_id
    assert active.state == "done" and admissions == ["motion-one", "motion-two"]


def test_motion_without_gateway_guard_and_completion_without_evidence_are_rejected(agent, monkeypatch):
    async def runner(text, history, invoke_tool, **kwargs):
        if text == "move":
            output = json.loads(await invoke_tool("navigate_to", {"x": 1, "y": 2}, "one"))
            assert output["reason"] == "motion_guard_unavailable"
        return TurnResult("task_complete", "I completed it", [*history, {"role": "user", "content": text}])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    assert submit(agent, cid, "one", "move", permission_mode="full_access")["state"] == "failed"
    assert submit(agent, cid, "two", "claim")["state"] == "failed"
    assert agent._active_run is None


def test_thinking_rejects_parallel_turn_and_cancel_prevents_late_motion(agent, monkeypatch):
    entered, release, returned = threading.Event(), threading.Event(), threading.Event()
    admissions = []

    async def runner(text, history, invoke_tool, **kwargs):
        entered.set()
        while not release.is_set():
            try:
                await asyncio.sleep(0.005)
            except asyncio.CancelledError:
                pass
        try:
            await invoke_tool("navigate_to", {"x": 1, "y": 2}, "late")
        finally:
            returned.set()
        return TurnResult("reply", "late", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    first = agent.submit_conversation_message(cid, "first", "move", motion_guard=lambda *args: admissions.append(args))
    assert entered.wait(1)
    try:
        with pytest.raises(ConversationError, match="turn_active"):
            agent.submit_conversation_message(cid, "parallel", "hi")
        cancelled = submit(agent, cid, "cancel", "停止")
        assert "已取消" in cancelled["text"]
        assert agent.get_conversation_turn(cid, first["turn_id"])["state"] == "cancelled"
    finally:
        release.set()
    assert returned.wait(1)
    assert not admissions and agent._active_run is None


@pytest.mark.parametrize("arguments", [
    {"x": "one", "y": 2}, {"x": True, "y": 2}, {"x": float("inf"), "y": 2},
    {"x": 1, "y": 2, "unexpected": 3},
])
def test_invalid_external_tool_arguments_do_not_admit_motion(agent, monkeypatch, arguments):
    admissions = []

    async def runner(text, history, invoke_tool, **kwargs):
        result = json.loads(await invoke_tool("navigate_to", arguments, "invalid"))
        assert "error" in result
        return TurnResult("clarification", "Need valid coordinates.", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    turn = submit(agent, cid, "invalid", "go there", motion_guard=lambda *args: admissions.append(args))
    assert turn["kind"] == "clarification" and not admissions and agent._active_run is None


def test_failed_tool_reply_does_not_leave_a_running_owner(agent, monkeypatch):
    async def runner(text, history, invoke_tool, **kwargs):
        await invoke_tool("navigate_to", {"x": 1, "y": 2}, "no-pose")
        return TurnResult("reply", "A fresh pose is required.", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    turn = submit(agent, cid, "no-pose", "go there", motion_guard=lambda admit, call_id: admit(),
                  permission_mode="full_access")
    assert turn["kind"] == "reply" and agent._active_run.state == "failed"
    assert agent._active_run.failure_reason == "motion_not_completed"


def test_attachment_selection_reaches_runner_and_retry_compares_options(agent, monkeypatch):
    calls = []

    async def runner(text, history, **kwargs):
        calls.append(kwargs)
        return TurnResult("reply", "Read the attachment.", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    encoded = base64.b64encode(b"attachment content").decode()
    receipt = agent.upload_conversation_attachment(cid, "file-one", "notes.txt", "text/plain", encoded)
    assert receipt == {"id": "file-one", "name": "notes.txt", "mime_type": "text/plain", "size": 18}
    first = submit(agent, cid, "attached", "", model="gpt-4o-mini", attachment_ids=["file-one"])
    assert first["model"] == "gpt-4o-mini" and first["reasoning"] == "default"
    assert calls[0]["selected_model"] == "gpt-4o-mini"
    assert calls[0]["attachments"][0]["data"] == b"attachment content"
    assert agent.submit_conversation_message(cid, "attached", "", model="gpt-4o-mini",
                                              attachment_ids=["file-one"])["turn_id"] == first["turn_id"]
    with pytest.raises(ConversationError, match="message_id_conflict"):
        agent.submit_conversation_message(cid, "attached", "", attachment_ids=["file-one"])
    snapshot = agent.get_conversation(cid)
    assert snapshot["messages"][0]["attachments"] == [receipt]
    assert '"data":' not in json.dumps(snapshot) and "attachment content" not in json.dumps(snapshot)


def test_attachment_retry_skips_parsing_and_is_bound_to_conversation(agent, monkeypatch):
    cid = agent.create_conversation()["conversation_id"]
    encoded = base64.b64encode(b"notes").decode()
    receipt = agent.upload_conversation_attachment(cid, "file", "notes.txt", "application/octet-stream", encoded)
    assert receipt["mime_type"] == "text/plain"

    def fail_parse(*args):
        pytest.fail("a stored attachment must not be parsed again")

    monkeypatch.setattr("decision.tasks.conversation_inputs.prepare_attachment", fail_parse)
    assert agent.upload_conversation_attachment(cid, "file", "notes.txt", "text/plain", encoded) == receipt
    assert agent.upload_conversation_attachment(cid, "file", "notes.txt", "application/octet-stream", encoded) == receipt
    with pytest.raises(ConversationError, match="attachment_id_conflict"):
        agent.upload_conversation_attachment(cid, "file", "changed.txt", "text/plain", encoded)
    with pytest.raises(ConversationError, match="attachment_id_conflict"):
        agent.upload_conversation_attachment(cid, "file", "notes.txt", "text/plain",
                                             base64.b64encode(b"different").decode())
    other = agent.create_conversation()["conversation_id"]
    with pytest.raises(ConversationError, match="attachment_not_found"):
        agent.submit_conversation_message(other, "message", "read", attachment_ids=["file"])


def test_restart_replays_attachment_content_and_migrates_old_text_turns(tmp_path):
    path = tmp_path / "old.db"
    with sqlite3.connect(path) as old:
        old.executescript("""
            CREATE TABLE conversations (id TEXT PRIMARY KEY);
            INSERT INTO conversations VALUES ('old');
            CREATE TABLE turns (
                ordinal INTEGER PRIMARY KEY AUTOINCREMENT,
                conversation_id TEXT NOT NULL, turn_id TEXT UNIQUE NOT NULL,
                message_id TEXT NOT NULL, input TEXT NOT NULL, reply_to TEXT,
                state TEXT NOT NULL, kind TEXT, text TEXT, run_id TEXT, failure_reason TEXT,
                model_items TEXT NOT NULL DEFAULT '[]', UNIQUE(conversation_id, message_id)
            );
            INSERT INTO turns (conversation_id,turn_id,message_id,input,state)
            VALUES ('old','old-turn','old-message','hello','completed');
        """)
    store = ConversationStore(path)
    replay, created = store.accept("old", "old-message", "hello", None)
    assert not created and replay["turn_id"] == "old-turn"
    assert "started_at" not in replay and "finished_at" not in replay
    store.save_attachment("old", "file", {"name": "notes.txt", "mime_type": "text/plain", "size": 5,
        "data": b"notes", "content": [{"type": "input_text", "text": "notes"}]})
    store.accept("old", "attached", "read", None, attachment_ids=["file"], resolved_model="gpt-4o-mini")
    store.close()
    restarted = ConversationStore(path)
    history = restarted.history("old")
    assert history[2]["content"] == [{"type": "input_text", "text": "read"},
                                      {"type": "input_text", "text": "notes"}]
    assert "Do not resume" in history[3]["content"]
    assert restarted.snapshot("old")["turns"][1]["model"] == "gpt-4o-mini"
    restarted.close()


@pytest.mark.parametrize("change", [
    {"model": "other"}, {"reasoning": "low"}, {"attachment_ids": ["second", "first"]},
])
def test_store_retry_compares_all_selected_inputs(tmp_path, change):
    store = ConversationStore(tmp_path / "retry.db")
    cid = store.create()["conversation_id"]
    selection = {"model": "chosen", "reasoning": "default", "attachment_ids": ["first", "second"]}
    first, _ = store.accept(cid, "message", "hello", None, **selection)
    replay, created = store.accept(cid, "message", "hello", None, **selection)
    assert not created and replay["turn_id"] == first["turn_id"]
    with pytest.raises(ConversationError, match="message_id_conflict"):
        store.accept(cid, "message", "hello", None, **{**selection, **change})
    store.close()


def wait_approval(agent, conversation_id):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        approvals = agent.get_conversation(conversation_id)["approvals"]
        if approvals:
            return approvals[-1]
        time.sleep(0.01)
    pytest.fail("approval request did not arrive")


@pytest.mark.parametrize("permission_mode,reason", [
    ("read_only", "permission_read_only"), ("full_access", None),
])
def test_permission_mode_controls_motion_entry(agent, monkeypatch, permission_mode, reason):
    calls = []

    async def motion(**kwargs):
        calls.append(kwargs)
        kwargs["run"].motion_attempted = kwargs["run"].motion_success = True
        return "reached"

    async def runner(text, history, invoke_tool, **kwargs):
        result = await invoke_tool("navigate_to", {"x": 1, "y": 2}, "motion")
        if reason:
            assert json.loads(result)["reason"] == reason
        return TurnResult("reply", "Processed.", [])

    monkeypatch.setattr(agent, "_tool_navigate_to", motion)
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    turn = submit(agent, cid, "motion", "go there", permission_mode=permission_mode,
                  motion_guard=lambda admit, call_id: admit())
    assert turn["permission_mode"] == permission_mode
    assert len(calls) == (0 if reason else 1)
    assert agent.get_conversation(cid)["approvals"] == []


@pytest.mark.parametrize("approved", [True, False])
def test_default_waits_for_specific_approval(agent, monkeypatch, approved):
    calls = []

    async def motion(**kwargs):
        calls.append(kwargs)
        kwargs["run"].motion_attempted = kwargs["run"].motion_success = True
        return "reached"

    async def runner(text, history, invoke_tool, **kwargs):
        result = await invoke_tool("navigate_to", {"x": 1, "y": 2}, "motion")
        if not approved:
            assert json.loads(result)["reason"] == "approval_denied"
        return TurnResult("reply", "Processed.", [])

    monkeypatch.setattr(agent, "_tool_navigate_to", motion)
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    turn = agent.submit_conversation_message(cid, "motion", "go there", motion_guard=lambda admit, call_id: admit())
    approval = wait_approval(agent, cid)
    assert not calls and agent._active_run is None
    assert approval["turn_id"] == turn["turn_id"] and approval["arguments"] == {"x": 1, "y": 2}
    receipt = agent.decide_conversation_approval(cid, approval["approval_id"], approved)
    assert receipt["state"] == ("approved" if approved else "denied")
    assert agent.decide_conversation_approval(cid, approval["approval_id"], approved) == receipt
    with pytest.raises(ConversationError, match="conflict"):
        agent.decide_conversation_approval(cid, approval["approval_id"], not approved)
    done = wait_turn(agent, cid, turn["turn_id"])
    assert len(calls) == int(approved)
    assert done["state"] == ("completed" if approved else "failed")


def test_cancelled_approval_cannot_admit_late_motion(agent, monkeypatch):
    calls = []

    async def runner(text, history, invoke_tool, **kwargs):
        await invoke_tool("navigate_to", {"x": 1, "y": 2}, "motion")
        return TurnResult("reply", "late", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    turn = agent.submit_conversation_message(cid, "motion", "go there", motion_guard=lambda *args: calls.append(args))
    approval = wait_approval(agent, cid)
    query = submit(agent, cid, "query", "任务状态", permission_mode="read_only")
    assert query["state"] == "completed"
    submit(agent, cid, "cancel", "停止", permission_mode="read_only")
    assert wait_turn(agent, cid, turn["turn_id"])["state"] == "cancelled"
    with pytest.raises(ConversationError, match=r"conflict|closed"):
        agent.decide_conversation_approval(cid, approval["approval_id"], True)
    assert not calls and agent.get_conversation(cid)["approvals"][0]["state"] == "cancelled"


def test_approval_restart_and_mode_retry_preserve_decisions(tmp_path):
    path = tmp_path / "permissions.db"
    store = ConversationStore(path)
    cid = store.create()["conversation_id"]
    turn, _ = store.accept(cid, "message", "move", None, permission_mode="default")
    approval = store.request_approval(cid, turn["turn_id"], "call", "navigate_to", {"x": 1, "y": 2})
    with pytest.raises(ConversationError, match="message_id_conflict"):
        store.accept(cid, "message", "move", None, permission_mode="full_access")
    store.close()
    restarted = ConversationStore(path)
    assert restarted.approval(cid, approval["approval_id"])["state"] == "cancelled"
    with pytest.raises(ConversationError, match=r"conflict|closed"):
        restarted.decide_approval(cid, approval["approval_id"], True)
    restarted.close()


def test_finished_timeout_cancels_pending_approval(agent, monkeypatch):
    async def runner(text, history, invoke_tool, **kwargs):
        with pytest.raises(asyncio.TimeoutError):
            await asyncio.wait_for(invoke_tool("navigate_to", {"x": 1, "y": 2}, "motion"), timeout=0.1)
        return TurnResult("failed", "Agent turn timed out", [], "Agent turn timed out")

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    turn = submit(agent, cid, "timeout", "go there")
    assert turn["state"] == "failed"
    approval = agent.get_conversation(cid)["approvals"][0]
    assert approval["state"] == "cancelled"
    with pytest.raises(ConversationError, match=r"conflict|closed"):
        agent.decide_conversation_approval(cid, approval["approval_id"], True)


@pytest.mark.parametrize("permission,expected_calls", [("read_only", 0), ("full_access", 1)])
def test_workbench_writes_use_conversation_permissions(agent, monkeypatch, permission, expected_calls):
    called = []

    async def invoke(name, args):
        called.append((name, args))
        return {"success": True}

    async def runner(text, history, invoke_tool, **kwargs):
        result = await invoke_tool("save_point", {"name": "入口"}, "call-map")
        return TurnResult("reply", str(result), history)

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    receipt = agent.submit_conversation_message(cid, "map-tool", "保存入口", permission_mode=permission,
        domain_tools=[{"type": "function", "writes": True, "function": {"name": "save_point", "parameters": {
            "type": "object", "properties": {"name": {"type": "string"}}, "required": ["name"]}}}],
        invoke_domain_tool=invoke)
    turn = wait_turn(agent, cid, receipt["turn_id"])
    assert len(called) == expected_calls
    assert turn["state"] == ("failed" if permission == "read_only" else "completed")
    assert not turn.get("run_id")


def test_context_is_part_of_request_identity_and_model_history_is_portable(tmp_path):
    store = ConversationStore(tmp_path / "context.db")
    cid = store.create()["conversation_id"]
    context = {"map_id": "garage", "content_epoch": 1, "selection": None}
    receipt, _ = store.accept(cid, "same", "补齐", None, resolved_model="kimi-k3", map_context=context)
    with pytest.raises(ConversationError, match="message_id_conflict"):
        store.accept(cid, "same", "补齐", None, map_context={**context, "content_epoch": 2})
    items = [{"type": "reasoning", "id": "provider-secret-reasoning", "summary": []},
             {"type": "function_call", "name": "inspect_saved_map", "call_id": "call1", "arguments": "{}",
              "id": "provider-id", "provider_data": {"reasoning_content": "private"}},
             {"type": "function_call_output", "call_id": "call1", "output": "saved"}]
    store.finish(cid, receipt["turn_id"], state="completed", kind="reply", text="完成", model_items=items)
    portable = store.history(cid, model="gpt-6.1-sol")
    assert len(portable) == 2 and "provider_data" not in portable[0] and "id" not in portable[0]
    assert portable[1]["output"] == "saved"
    assert store.history(cid, model="kimi-k3") == items
    store.close()



def test_attachment_processing_uses_selected_provider_not_default(agent, monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "test-only")
    seen = []

    def prepare(config, name, mime_type, data):
        seen.append((config.backend, config.api_key_env))
        return {"name": name, "mime_type": mime_type, "data": b"hello", "size": 5,
                "content": [{"type": "input_text", "text": "hello"}]}

    monkeypatch.setattr("decision.tasks.conversation_inputs.prepare_attachment", prepare)
    cid = agent.create_conversation()["conversation_id"]
    agent.upload_conversation_attachment(cid, "file", "notes.txt", "text/plain", "aGVsbG8=", model="gpt-6.1-sol")
    assert seen == [("openai", "OPENAI_API_KEY")]



def test_tool_progress_updates_one_message_and_persists_elapsed_time(agent, monkeypatch):
    async def runner(text, history, invoke_tool, **kwargs):
        await invoke_tool("get_agent_status", {}, "timed-call")
        return TurnResult("reply", "已查询", history)

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    receipt = agent.submit_conversation_message(cid, "time-check", "查询")
    wait_turn(agent, cid, receipt["turn_id"])
    messages = agent._conversation_store().snapshot(cid)["messages"]
    tools = [message for message in messages if message["role"] == "tool"]
    assert len(tools) == 1
    assert tools[0]["metadata"]["tool_call_id"] == "timed-call"
    assert tools[0]["metadata"]["status"] == "completed"
    assert tools[0]["metadata"]["duration_ms"] >= 0
    assert "执行中" not in tools[0]["text"]


def test_clarification_choices_survive_history_refresh(agent, monkeypatch):
    async def runner(text, history, **kwargs):
        return TurnResult("clarification", "哪一类？", history, choices=["地面", "墙面"])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    receipt = agent.submit_conversation_message(cid, "choice-check", "补齐这里")
    wait_turn(agent, cid, receipt["turn_id"])
    answer = agent._conversation_store().snapshot(cid)["messages"][-1]
    assert answer["text"] == "哪一类？"
    assert answer["metadata"]["choices"] == ["地面", "墙面"]



def test_connection_card_reads_real_catalog_without_secrets(agent, monkeypatch):
    async def runner(text, history, invoke_tool, **kwargs):
        result = await invoke_tool("get_model_connections", {}, "connections")
        assert result["models"][0]["configured"] is False
        return TurnResult("reply", "需要配置", history)

    monkeypatch.setattr(agent, "get_conversation_options", lambda: {"models": [
        {"id": "gpt-6.1-sol", "label": "GPT", "configured": False, "key_env": "OPENAI_API_KEY"}]})
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    receipt = agent.submit_conversation_message(cid, "connections", "查看模型连接", permission_mode="read_only")
    turn = wait_turn(agent, cid, receipt["turn_id"])
    assert turn["state"] == "completed" and not turn.get("run_id")



def test_switching_models_replays_output_messages_through_real_sdk_converter(tmp_path):
    from agents.models.chatcmpl_converter import Converter

    store = ConversationStore(tmp_path / "model-switch.db")
    cid = store.create()["conversation_id"]
    receipt, _ = store.accept(cid, "one", "问题", None, resolved_model="kimi-k2.6")
    store.finish(cid, receipt["turn_id"], state="completed", kind="reply", text="回答", model_items=[
        {"type": "message", "role": "assistant", "id": "provider-message", "status": "completed",
         "content": [{"type": "output_text", "text": "第一段", "annotations": [], "logprobs": []},
                     {"type": "output_text", "text": "第二段", "annotations": []}]}])
    history = store.history(cid, model="kimi-k3.1-pro-highspeed")
    converted = Converter.items_to_messages(history)
    assert converted[0]["content"] == "第一段\n第二段"
    store.close()


@pytest.mark.parametrize("permission", ["read_only", "default", "full_access"])
def test_robot_analysis_uses_existing_permissions_and_no_motion_owner(agent, monkeypatch, permission):
    from gateway.services.conversation_tools import workbench_tools

    tools, invoke = workbench_tools(SimpleNamespace())
    async def runner(invoke_tool, **kwargs):
        result = await invoke_tool("run_python", {"code": "print(6 * 7)"}, "analysis")
        return TurnResult("reply", json.dumps(result), [])
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    receipt = agent.submit_conversation_message(cid, "analysis", "计算", permission_mode=permission,
                                               domain_tools=tools, invoke_domain_tool=invoke)
    if permission == "default":
        approval = wait_approval(agent, cid)
        assert approval["tool_name"] == "run_python" and approval["arguments"]["code"] == "print(6 * 7)"
        agent.decide_conversation_approval(cid, approval["approval_id"], True)
    turn = wait_turn(agent, cid, receipt["turn_id"])
    assert agent._active_run is None and not turn.get("run_id")
    if permission == "read_only":
        assert "permission_read_only" in turn["text"]
    else:
        assert "42" in turn["text"]

def test_specialist_events_persist_one_receipt_each_without_motion(agent, monkeypatch):
    async def runner(text, history, on_event, **kwargs):
        event = {'type': 'specialist', 'id': 'call-maps', 'name': '地图助手', 'task': text, 'status': 'running'}
        on_event(event)
        on_event({**event, 'status': 'completed', 'summary': '检查完成', 'evidence': [], 'duration_ms': 50})
        return TurnResult('reply', '汇总', [*history, {'role': 'user', 'content': text}])
    monkeypatch.setattr('decision.tasks.sdk_agent.run_turn', runner)
    cid = agent.create_conversation()['conversation_id']
    receipt = submit(agent, cid, 'specialist-review', '协作检查', permission_mode='read_only')
    assert receipt['state'] == 'completed' and not receipt.get('run_id')
    rows = [m for m in agent._conversation_store().snapshot(cid)['messages'] if m['kind'] == 'specialist_review']
    assert len(rows) == 1
    assert rows[0]['metadata']['status'] == 'completed'
    assert json.loads(rows[0]['text'])['summary'] == '检查完成'
