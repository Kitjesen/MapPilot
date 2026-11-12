"""Durable preferences share conversation storage, not robot spatial memory."""

import pytest

from decision.tasks.conversation import ConversationStore


def test_preferences_persist_update_and_forget_without_erasing_history(tmp_path):
    path = tmp_path / "conversations.sqlite3"
    store = ConversationStore(path)
    conversation = store.create()
    entry = store.edit_preference("remember", content="请称呼我森哥。 ")["entry"]
    duplicate = store.edit_preference("remember", content="请称呼我森哥。")
    assert duplicate["entry"] == entry and not duplicate["created"]
    store.close()

    store = ConversationStore(path)
    assert store.preferences()["entries"] == [entry]
    changed = store.edit_preference("update", entry_id=entry["id"], old_content=entry["content"], content="回复简洁。")
    assert changed["saved"]
    assert store.edit_preference("forget", entry_id=entry["id"], old_content=entry["content"])["error"] == "preference_changed"
    assert store.edit_preference("update", entry_id=entry["id"], old_content=entry["content"], content="旧请求")["error"] == "preference_changed"
    assert store.preferences()["entries"] == [changed["entry"]]
    assert store.edit_preference("forget", entry_id=entry["id"], old_content="回复简洁。")["removed"]
    assert store.preferences()["entries"] == []
    assert store.metadata(conversation["conversation_id"])
    store.close()


@pytest.mark.parametrize("content", ["", "   ", "x" * 1001, None])
def test_invalid_preference_does_not_write(tmp_path, content):
    store = ConversationStore(tmp_path / "test.db")
    assert "error" in store.edit_preference("remember", content=content)
    assert store.preferences()["entries"] == []
    store.close()


def test_full_memory_does_not_evict_existing_preferences(tmp_path):
    store = ConversationStore(tmp_path / "test.db")
    for index in range(12):
        assert store.edit_preference("remember", content=f"{index:02}" + "x" * 998)["saved"]
    before = store.preferences()
    assert before["characters"] == 12000
    assert store.edit_preference("remember", content="new")["error"] == "preference_memory_full"
    assert store.preferences() == before
    first = before["entries"][0]
    assert store.edit_preference("update", entry_id=first["id"], old_content=first["content"], content="short")["saved"]
    assert store.edit_preference("remember", content="new")["saved"]
    store.close()
