"""Public streamed replies retain one identity through persistence and terminal states."""

import asyncio
import threading
import time

import pytest

from decision.tasks.conversation import ConversationError, ConversationStore
from decision.tasks.sdk_agent import TurnResult
from tests.decision.test_agent_conversations import agent as agent
from tests.decision.test_agent_conversations import submit, wait_turn


def test_streamed_reply_is_coalesced_recoverable_and_finalized_once(agent, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    events = []
    agent.agent_message.subscribe(events.append)

    async def runner(on_event, **kwargs):
        on_event({"type": "text_delta", "item_id": "answer", "delta": "开始"})
        for _ in range(30):
            on_event({"type": "text_delta", "item_id": "answer", "delta": "。"})
        on_event({"type": "reasoning_delta", "delta": "PRIVATE_REASONING"})
        entered.set()
        while not release.is_set():
            await asyncio.sleep(0.005)
        return TurnResult("reply", "真实完整结果", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    receipt = agent.submit_conversation_message(cid, "stream", "查一下")
    assert entered.wait(1)
    try:
        deadline = time.monotonic() + 1
        while len(events) < 2 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert len(events) == 2
        assert [event["text"] for event in events] == ["开始", "开始" + "。" * 30]
        assert events[0]["id"] == events[1]["id"]
        assert events[1]["revision"] > events[0]["revision"]
        snapshot = agent.get_conversation(cid)
        assert snapshot["messages"][-1]["text"] == events[1]["text"]
        assert snapshot["messages"][-1]["metadata"]["status"] == "streaming"
        with pytest.raises(ConversationError, match="share_empty"):
            agent.share_conversation(cid)
    finally:
        release.set()
    assert wait_turn(agent, cid, receipt["turn_id"])["state"] == "completed"
    messages = [row for row in agent.get_conversation(cid)["messages"] if row["role"] == "assistant"]
    assert len(messages) == 1
    assert messages[0]["text"] == "真实完整结果"
    assert messages[0]["metadata"]["status"] == "completed"
    assert messages[0]["id"] == events[0]["id"] == events[-1]["id"]
    assert events[-1]["text"] == "真实完整结果" and len(events) == 3
    assert all("PRIVATE_REASONING" not in event["text"] for event in events)
    assert agent.submit_conversation_message(cid, "stream", "查一下")["turn_id"] == receipt["turn_id"]
    assert len(agent.get_conversation(cid)["messages"]) == 2


@pytest.mark.parametrize("outcome", ["clarification", "failed", "exception"])
def test_partial_stream_uses_authoritative_terminal_result(agent, monkeypatch, outcome):
    events = []
    agent.agent_message.subscribe(events.append)

    async def runner(on_event, **kwargs):
        on_event({"type": "text_delta", "delta": "未完成的前缀"})
        if outcome == "exception":
            raise RuntimeError("offline error")
        return TurnResult(outcome, "真实终结结果", [], "failed" if outcome == "failed" else "", ["左", "右"])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    turn = submit(agent, cid, "terminal", "检查")
    answer = agent.get_conversation(cid)["messages"][-1]
    assert answer["id"] == events[0]["id"] == events[-1]["id"]
    assert answer["kind"] == turn["kind"]
    assert answer["metadata"]["status"] == turn["state"]
    assert answer["text"] == turn["text"] and "未完成的前缀" not in answer["text"]
    assert len(agent.get_conversation(cid)["messages"]) == 2
    if outcome == "clarification":
        assert answer["metadata"]["choices"] == ["左", "右"]


def test_new_sdk_output_replaces_previous_segment_and_final_flushes_pending_text(agent, monkeypatch):
    events = []
    agent.agent_message.subscribe(events.append)

    async def runner(on_event, **kwargs):
        on_event({"type": "text_delta", "item_id": "before", "delta": "先查状态"})
        on_event({"type": "tool_start", "name": "inspect"})
        on_event({"type": "tool_end", "output": "PRIVATE_TOOL"})
        on_event({"type": "text_delta", "item_id": "after", "delta": "查到"})
        on_event({"type": "text_delta", "item_id": "after", "delta": "结果"})
        await asyncio.sleep(0.1)
        return TurnResult("reply", "查到结果", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    submit(agent, cid, "segments", "查询")
    assert [event["text"] for event in events] == ["先查状态", "查到结果", "查到结果"]
    assert len({event["id"] for event in events}) == 1
    assert events[-1]["metadata"]["status"] == "completed"


def test_cancel_updates_the_partial_message_without_a_later_stream_write(agent, monkeypatch):
    entered = threading.Event()
    events = []
    agent.agent_message.subscribe(events.append)

    async def runner(on_event, **kwargs):
        on_event({"type": "text_delta", "delta": "正在"})
        on_event({"type": "text_delta", "delta": "处理"})
        entered.set()
        await asyncio.Event().wait()

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    receipt = agent.submit_conversation_message(cid, "stream", "检查")
    assert entered.wait(1)
    submit(agent, cid, "cancel", "停止")
    time.sleep(0.1)
    answer = next(row for row in agent.get_conversation(cid)["messages"] if row["id"] == events[0]["id"])
    assert answer["metadata"]["status"] == "cancelled"
    assert answer["text"] == "本轮执行已取消。"
    assert agent.get_conversation_turn(cid, receipt["turn_id"])["state"] == "cancelled"
    stream_events = [event for event in events if event["id"] == answer["id"]]
    assert stream_events[-1]["metadata"]["status"] == "cancelled"
    assert len(stream_events) == 2


def test_restart_closes_streaming_message_without_replaying_it(tmp_path):
    path = tmp_path / "stream.db"
    store = ConversationStore(path)
    cid = store.create()["conversation_id"]
    turn, _ = store.accept(cid, "pending", "检查", None)
    partial = store.append(cid, turn["turn_id"], "assistant", "已收到的部分", kind="reply", metadata={"status": "streaming"})
    store.close()
    restored = ConversationStore(path)
    answer = restored.snapshot(cid)["messages"][-1]
    assert answer["id"] == partial["id"] and answer["seq"] == partial["seq"]
    assert answer["metadata"]["status"] == "interrupted"
    assert answer["text"] == "已收到的部分" and answer["revision"] > partial["revision"]
    assert restored.turn(cid, turn["turn_id"])["state"] == "interrupted"
    assert not restored.accept(cid, "pending", "检查", None)[1]
    restored.close()
