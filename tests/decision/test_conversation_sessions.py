"""Persistent conversation selection and revocable frozen shares."""

import json
import sqlite3

import pytest

from decision.tasks.conversation import ConversationError, ConversationStore
from tests.decision.test_agent_conversations import agent as agent


def completed_turn(store, conversation_id, message_id, text, answer="Answered."):
    turn, _ = store.accept(conversation_id, message_id, text, None)
    store.append(conversation_id, turn["turn_id"], "assistant", answer)
    store.finish(conversation_id, turn["turn_id"], state="completed", kind="reply", text=answer, model_items=[])
    return turn


def test_reply_timing_survives_retry_and_reload(tmp_path, monkeypatch):
    now = [1000.0]
    monkeypatch.setattr("decision.tasks.conversation.time.time", lambda: now[0])
    path = tmp_path / "timing.db"
    store = ConversationStore(path)
    cid = store.create()["conversation_id"]
    accepted, _ = store.accept(cid, "request", "hello", None)
    assert accepted["started_at"] == 1000.0
    assert "finished_at" not in accepted
    now[0] = 1004.0
    assert store.snapshot(cid)["server_time"] == 1004.0
    reply = store.append(cid, accepted["turn_id"], "assistant", "hello back")
    assert reply["created_at"] == 1004.0
    now[0] = 1008.5
    store.finish(cid, accepted["turn_id"], state="completed", kind="reply", text="hello back", model_items=[])
    now[0] = 1010.0
    replay, created = store.accept(cid, "request", "hello", None)
    assert not created and replay["started_at"] == 1000.0 and replay["finished_at"] == 1008.5
    store.finish(cid, accepted["turn_id"], state="completed", kind="reply", text="hello back", model_items=[])
    store.close()
    reloaded = ConversationStore(path)
    snapshot = reloaded.snapshot(cid)
    assert snapshot["turns"][0]["finished_at"] - snapshot["turns"][0]["started_at"] == 8.5
    assert [message["created_at"] for message in snapshot["messages"]] == [1000.0, 1004.0]
    reloaded.close()


def test_old_messages_keep_unknown_times(tmp_path):
    path = tmp_path / "old-times.db"
    store = ConversationStore(path)
    cid = store.create()["conversation_id"]
    completed_turn(store, cid, "old", "old request")
    store.close()
    with sqlite3.connect(path) as db:
        db.executescript("""
            ALTER TABLE messages RENAME TO newer_messages;
            CREATE TABLE messages (
                conversation_id TEXT NOT NULL, id TEXT NOT NULL, seq INTEGER NOT NULL,
                role TEXT NOT NULL, text TEXT NOT NULL, turn_id TEXT NOT NULL, run_id TEXT, kind TEXT,
                PRIMARY KEY(conversation_id,seq)
            );
            INSERT INTO messages SELECT conversation_id,id,seq,role,text,turn_id,run_id,kind FROM newer_messages;
            DROP TABLE newer_messages;
            UPDATE turns SET started_at=NULL,finished_at=NULL;
        """)
    reloaded = ConversationStore(path)
    snapshot = reloaded.snapshot(cid)
    assert all(message["created_at"] is None for message in snapshot["messages"])
    assert "started_at" not in snapshot["turns"][0] and "finished_at" not in snapshot["turns"][0]
    new_turn, _ = reloaded.accept(cid, "new", "new request", None)
    assert new_turn["started_at"] > 0
    assert reloaded.snapshot(cid)["messages"][-1]["created_at"] > 0
    reloaded.close()


def test_metadata_list_rename_archive_and_restore(tmp_path):
    store = ConversationStore(tmp_path / "sessions.db")
    cid = store.create()["conversation_id"]
    metadata = store.metadata(cid)
    assert metadata["title"] == "新对话" and not metadata["archived"]
    assert metadata["created_at"] == metadata["updated_at"] > 0
    turn, _ = store.accept(cid, "first", "First\n  topic", None)
    assert store.metadata(cid)["title"] == "First topic"
    with pytest.raises(ConversationError, match="conversation_active"):
        store.update(cid, archived=True)
    store.finish(cid, turn["turn_id"], state="completed", kind="reply", text="done", model_items=[])
    archived = store.update(cid, title="Renamed", archived=True)
    assert archived["archived"] and archived["title"] == "Renamed"
    assert store.list()["conversations"] == []
    assert store.list(True)["conversations"] == [archived]
    with pytest.raises(ConversationError, match="conversation_archived"):
        store.accept(cid, "new", "hello", None)
    assert store.accept(cid, "first", "First\n  topic", None)[1] is False
    store.update(cid, archived=False)
    completed_turn(store, cid, "new", "second topic")
    assert store.metadata(cid)["title"] == "Renamed"
    store.close()


def test_old_metadata_migration_preserves_history_and_manual_title(tmp_path):
    path = tmp_path / "old.db"
    store = ConversationStore(path)
    cid = store.create()["conversation_id"]
    completed_turn(store, cid, "old", "Original topic")
    store.close()
    with sqlite3.connect(path) as db:
        db.executescript("""
            ALTER TABLE conversations RENAME TO old_conversations;
            CREATE TABLE conversations (id TEXT PRIMARY KEY);
            INSERT INTO conversations SELECT id FROM old_conversations;
            DROP TABLE old_conversations;
        """)
    migrated = ConversationStore(path)
    assert migrated.metadata(cid)["title"] == "Original topic"
    assert migrated.snapshot(cid)["messages"][0]["text"] == "Original topic"
    migrated.update(cid, title="Customer title")
    migrated.close()
    restarted = ConversationStore(path)
    assert restarted.metadata(cid)["title"] == "Customer title"
    restarted.close()


def test_share_freezes_only_terminal_public_text_and_can_be_revoked(tmp_path):
    path = tmp_path / "shares.db"
    store = ConversationStore(path)
    cid = store.create()["conversation_id"]
    with pytest.raises(ConversationError, match="share_empty"):
        store.create_share(cid)
    first = completed_turn(store, cid, "first", "A user question", "A public answer")
    store.append(cid, first["turn_id"], "tool", "PRIVATE_TOOL_OUTPUT", kind="navigate_to")
    store.finish(cid, first["turn_id"], state="completed", kind="reply", text="A public answer",
                 model_items=[{"private": "MODEL_ITEMS"}])
    pending, _ = store.accept(cid, "pending", "Unfinished question", None)
    store.request_approval(cid, pending["turn_id"], "tool", "navigate_to", {"x": 1, "y": 2})
    store.accept(cid, "queued", "Queued private question", None, enqueue=True)
    receipt = store.create_share(cid)
    assert receipt["message_count"] == 2 and len(receipt["token"]) >= 40
    shared = store.shared(receipt["token"])
    assert set(shared) == {"title", "created_at", "messages"}
    assert all(set(message) == {"role", "text", "seq"} for message in shared["messages"])
    assert [message["role"] for message in shared["messages"]] == ["user", "assistant"]
    assert "PRIVATE_TOOL_OUTPUT" not in json.dumps(shared) and "MODEL_ITEMS" not in json.dumps(shared)
    assert "Unfinished question" not in json.dumps(shared)
    assert "Queued private question" not in json.dumps(shared)
    store.finish(cid, pending["turn_id"], state="failed", kind="failed", text="Failed.", model_items=[])
    store.append(cid, pending["turn_id"], "assistant", "Failed.")
    store.update(cid, title="Changed later")
    assert store.create_share(cid) == receipt and store.shared(receipt["token"]) == shared
    store.close()
    restarted = ConversationStore(path)
    assert restarted.shared(receipt["token"]) == shared
    assert restarted.revoke_share(cid) == {"revoked": True}
    assert restarted.revoke_share(cid) == {"revoked": True}
    with pytest.raises(ConversationError, match="share_not_found"):
        restarted.shared(receipt["token"])
    newer = restarted.create_share(cid)
    assert newer["token"] != receipt["token"] and newer["message_count"] == 5
    assert restarted.shared(newer["token"])["title"] == "Changed later"
    restarted.close()


def test_archive_rejects_unconfirmed_motion_for_own_conversation(agent, monkeypatch):
    store = agent._conversation_store()
    cid = store.create()["conversation_id"]
    turn = completed_turn(store, cid, "first", "move")
    store.bind_run(cid, turn["turn_id"], "owned-run")
    monkeypatch.setattr(agent, "get_agent_status", lambda: json.dumps({
        "run_id": "owned-run", "state": "failed", "stop_confirmed": False,
    }))
    with pytest.raises(ConversationError, match="conversation_active"):
        agent.update_conversation(cid, archived=True)
    other = agent.create_conversation()["conversation_id"]
    assert agent.update_conversation(other, archived=True)["archived"]
    assert agent._active_run is None
    monkeypatch.setattr(agent, "get_agent_status", lambda: json.dumps({
        "run_id": "owned-run", "state": "failed", "stop_confirmed": True,
    }))
    assert agent.update_conversation(cid, archived=True)["archived"]
