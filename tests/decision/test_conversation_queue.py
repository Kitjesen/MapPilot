import asyncio
import threading
import time

import pytest

from decision.tasks.conversation import ConversationError, ConversationStore
from decision.tasks.sdk_agent import TurnResult
from tests.decision.test_agent_conversations import agent


def test_queue_edit_claim_cancel_and_restart(tmp_path):
    path = tmp_path / "queue.db"
    store = ConversationStore(path)
    cid = store.create()["conversation_id"]
    first, _ = store.accept(cid, "one", "first", None)
    second, _ = store.accept(cid, "two", "second", None, enqueue=True)
    third, _ = store.accept(cid, "three", "third", None, enqueue=True)
    assert second["state"] == "queued" and "started_at" not in second
    assert store.claim_queued(cid, second["turn_id"]) is None
    assert all(item.get("content") != "second" for item in store.history(cid))
    store.edit_queued(cid, second["turn_id"], "edited")
    store.edit_queued(cid, third["turn_id"], None)
    assert store.turn(cid, third["turn_id"])["state"] == "cancelled"
    store.finish(cid, first["turn_id"], state="completed", kind="reply", text="done", model_items=[])
    assert store.claim_queued(cid, second["turn_id"]) == "edited"
    with pytest.raises(ConversationError, match="queue_closed_conflict"):
        store.edit_queued(cid, second["turn_id"], "too late")
    fourth, _ = store.accept(cid, "four", "fourth", None, enqueue=True)
    store.close()
    store = ConversationStore(path)
    assert store.turn(cid, fourth["turn_id"])["state"] == "interrupted"
    store.close()


def test_queue_dispatches_edited_input_once_in_order(agent, monkeypatch):
    release = threading.Event()
    calls = []

    async def runner(text, history, **kwargs):
        calls.append(text)
        if text == "first":
            while not release.is_set():
                await asyncio.sleep(.01)
        return TurnResult("reply", "done", [{"role": "user", "content": text}])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    agent.submit_conversation_message(cid, "one", "first", enqueue=True)
    second = agent.submit_conversation_message(cid, "two", "second", enqueue=True)
    duplicate = agent.submit_conversation_message(cid, "two", "second", enqueue=True)
    third = agent.submit_conversation_message(cid, "three", "third", enqueue=True)
    assert duplicate["turn_id"] == second["turn_id"]
    agent.edit_queued_conversation(cid, second["turn_id"], "edited")
    agent.edit_queued_conversation(cid, third["turn_id"], None)
    release.set()
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        if agent.get_conversation_turn(cid, second["turn_id"])["state"] == "completed":
            break
        time.sleep(.01)
    assert agent.get_conversation_turn(cid, second["turn_id"])["state"] == "completed"
    assert calls == ["first", "edited"]


def test_bounded_snapshot_keeps_queue_and_tool_revisions(tmp_path):
    store = ConversationStore(tmp_path / "history.db")
    cid = store.create()["conversation_id"]
    first, _ = store.accept(cid, "one", "first", None)
    pending, _ = store.accept(cid, "two", "later", None, enqueue=True)
    for index in range(210):
        last = store.append(cid, first["turn_id"], "tool", str(index))
    latest = store.snapshot(cid, limit=200)
    assert len(latest["messages"]) == 201
    assert any(item["id"] == pending["message_id"] for item in latest["messages"])
    cursor = latest["history_cursor"]
    assert cursor > 1
    older = store.snapshot(cid, before_seq=cursor, limit=200)
    assert older["history_cursor"] is None
    assert older["messages"][0]["id"] == "one"
    updated = store.update_message(cid, last["id"], "complete", {"status": "completed"})
    assert updated["revision"] == 2
    store.close()


def test_promote_changes_next_turn_without_interrupting_active(tmp_path):
    store = ConversationStore(tmp_path / "order.db")
    cid = store.create()["conversation_id"]
    first, _ = store.accept(cid, "one", "first", None)
    second, _ = store.accept(cid, "two", "second", None, enqueue=True)
    third, _ = store.accept(cid, "three", "third", None, enqueue=True)
    promoted = store.promote_queued(cid, third["turn_id"])
    assert promoted["queue_position"] < second["queue_position"]
    assert store.claim_queued(cid, third["turn_id"]) is None
    assert store.turn(cid, first["turn_id"])["state"] == "running"
    store.finish(cid, first["turn_id"], state="completed", kind="reply", text="done", model_items=[])
    assert store.claim_queued(cid, second["turn_id"]) is None
    assert store.claim_queued(cid, third["turn_id"]) == "third"
    store.finish(cid, third["turn_id"], state="completed", kind="reply", text="done", model_items=[])
    assert store.claim_queued(cid, second["turn_id"]) == "second"
    assert [item["content"] for item in store.history(cid) if item["role"] == "user"] == ['first', 'third', 'second']
    store.close()
