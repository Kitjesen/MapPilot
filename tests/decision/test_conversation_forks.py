"""Conversation branches copy a closed public prefix, never runnable work or private replay."""

import json
import sqlite3

import pytest

from decision.tasks.conversation import ConversationError, ConversationStore
from decision.tasks.sdk_agent import TurnResult
from tests.decision.test_agent_conversations import agent as agent
from tests.decision.test_agent_conversations import submit


@pytest.fixture
def store(tmp_path):
    history = ConversationStore(tmp_path / "forks.db")
    yield history
    history.close()


def complete(store, cid, message_id, text="Question", answer="Answer", *, metadata=None, kind="reply", **kwargs):
    turn, _ = store.accept(cid, message_id, text, kwargs.pop("reply_to", None), **kwargs)
    reply = store.append(cid, turn["turn_id"], "assistant", answer, kind=kind, metadata=metadata)
    store.finish(cid, turn["turn_id"], state="completed", kind=kind, text=answer,
                 model_items=[{"type": "reasoning", "private": "PRIVATE_REPLAY"}])
    return turn, reply


def test_fork_copies_the_full_prefix_beyond_the_snapshot_page_and_persists_origin(store):
    cid = store.create()["conversation_id"]
    replies = [complete(store, cid, f"question-{index}", text=f"Question {index}", answer=f"Answer {index}")[1]
               for index in range(130)]
    store.update(cid, title="源" * 80, archived=True)
    source = store.snapshot(cid)
    branch = store.fork(cid, replies[109]["id"])
    copied = store.snapshot(branch["conversation_id"])
    assert len(copied["messages"]) == 220 and len(copied["turns"]) == 110
    assert copied["messages"][0]["text"] == "Question 0" and copied["messages"][-1]["text"] == "Answer 109"
    assert [message["seq"] for message in copied["messages"]] == list(range(1, 221))
    assert not {message["id"] for message in source["messages"]} & {message["id"] for message in copied["messages"]}
    assert not {turn["turn_id"] for turn in source["turns"]} & {turn["turn_id"] for turn in copied["turns"]}
    assert branch["title"].endswith(" · 分支") and len(branch["title"]) == 80 and not branch["archived"]
    assert branch["forked_from"] == {"conversation_id": cid, "message_id": replies[109]["id"],
                                     "title": "源" * 80, "seq": replies[109]["seq"]}
    assert store.metadata(branch["conversation_id"]) == branch
    assert "forked_from" not in store.metadata(cid)
    assert all(row["model_items"] == "[]" for row in store._db.execute(
        "SELECT model_items FROM turns WHERE conversation_id=?", (branch["conversation_id"],)))
    assert len(store.fork(cid, replies[0]["id"])["forked_from"]) == 4
    # A fresh reader runs the same schema migration as Host restart.
    reloaded = ConversationStore(store._db.execute("PRAGMA database_list").fetchone()["file"])
    assert reloaded.metadata(branch["conversation_id"]) == branch
    reloaded.close()
    assert {key: value for key, value in store.snapshot(cid).items() if key != "server_time"} == {
        key: value for key, value in source.items() if key != "server_time"}


def test_fork_preserves_plan_references_and_attachment_content_without_private_or_active_state(store):
    cid = store.create()["conversation_id"]
    attachments = [
        {"id": "document", "name": "notes.txt", "mime_type": "text/plain", "data": b"document bytes",
         "content": [{"type": "input_text", "text": "DOCUMENT_REFERENCE"}]},
        {"id": "image", "name": "camera.png", "mime_type": "image/png", "data": b"image bytes",
         "content": [{"type": "input_image", "image_url": "data:image/png;base64,YWJj"}]},
    ]
    for item in attachments:
        store.save_attachment(cid, item["id"], {**item, "size": len(item["data"])})
    plan = {"title": "只读排查", "steps": [{"title": "读取定位状态"}]}
    first, answer = complete(store, cid, "first", text="看附件", answer="# 只读排查", kind="plan", attachment_ids=["document", "image"],
                             metadata={"status": "completed", "plan": plan, "private_trace": "PRIVATE_METADATA"})
    store.bind_run(cid, first["turn_id"], "historical-run")
    second, point = complete(store, cid, "second", text="解释这一句", reply_to=answer["id"], reply_quote="读取定位状态")
    pending, _ = store.accept(cid, "pending", "FUTURE_QUESTION", None)
    store.request_approval(cid, pending["turn_id"], "pending-call", "navigate_to", {"x": 1, "y": 2})
    store.append(cid, pending["turn_id"], "tool", "PRIVATE_TOOL")
    store.accept(cid, "queued", "FUTURE_QUEUE", None, enqueue=True)
    store.create_share(cid)
    ordinal = store._db.execute("SELECT ordinal FROM turns WHERE turn_id=?", (second["turn_id"],)).fetchone()[0]
    assert store.save_context(cid, pending["turn_id"], 0, ordinal, "SOURCE_COMPRESSION", None)
    branch = store.fork(cid, point["id"])
    target = branch["conversation_id"]
    copied = store.snapshot(target)
    assert len(copied["messages"]) == 4 and copied["share"] is None and copied["approvals"] == []
    assert all(message["run_id"] is None for message in copied["messages"])
    assert all("run_id" not in turn for turn in copied["turns"])
    assert copied["messages"][1]["metadata"] == {"status": "completed", "plan": plan}
    assert copied["messages"][1]["kind"] == "plan"
    assert copied["turns"][1]["reply_to_message_id"] == copied["messages"][1]["id"]
    assert copied["turns"][1]["reply_quote"] == "读取定位状态"
    assert store.context_checkpoint(target) == {"through_ordinal": 0, "summary": ""}
    assert "PRIVATE" not in json.dumps(copied) and "FUTURE" not in json.dumps(copied)
    for item in attachments:
        assert store.attachment(target, item["id"])["data"] == item["data"]
        assert store.attachment(target, item["id"])["content"] == item["content"]
    history = store.history(target)
    assert history[0]["content"][1:] == [*attachments[0]["content"], *attachments[1]["content"]]
    assert "读取定位状态" in history[2]["content"] and "本轮追问\n解释这一句" in history[2]["content"]
    assert store.turn(cid, pending["turn_id"])["state"] == "running"
    assert store.approval(cid, store.snapshot(cid)["approvals"][0]["approval_id"])["state"] == "pending"


@pytest.mark.parametrize("quote", [None, "当时的摘录"])
def test_streamed_tool_reply_and_earlier_queued_input_do_not_cross_the_selected_completion(store, monkeypatch, quote):
    now = [100.0]
    monkeypatch.setattr("decision.tasks.conversation.time.time", lambda: now[0])
    cid = store.create()["conversation_id"]
    first, _ = store.accept(cid, "first", "慢速任务", None)
    streamed = store.append(cid, first["turn_id"], "assistant", "当时的摘录", kind="reply", metadata={"status": "streaming"})
    queued, _ = store.accept(cid, "queued", "以后才执行的问题", None, enqueue=True)
    now[0] = 102
    _, point = complete(store, cid, "control", text="状态追问", reply_to=streamed["id"],
                        reply_quote=quote, control=True)
    now[0] = 104
    store.append(cid, first["turn_id"], "tool", "INTERNAL_TOOL_AFTER_STREAM")
    store.finish(cid, first["turn_id"], state="completed", kind="reply", text="未来才完成的回答", model_items=[])
    now[0] = 106
    assert store.claim_queued(cid, queued["turn_id"]) is not None
    store.append(cid, queued["turn_id"], "assistant", "未来的队列答案")
    store.finish(cid, queued["turn_id"], state="completed", kind="reply", text="未来的队列答案", model_items=[])
    branch = store.fork(cid, point["id"])
    copied = store.snapshot(branch["conversation_id"])
    assert [message["text"] for message in copied["messages"]] == ["状态追问", "Answer"]
    assert [message["seq"] for message in copied["messages"]] == [1, 2]
    assert "reply_to_message_id" not in copied["turns"][0]
    assert store.quoted_text(branch["conversation_id"], copied["turns"][0]["turn_id"]) == quote
    if quote:
        assert quote in store.history(branch["conversation_id"])[0]["content"]
    assert "未来" not in json.dumps(store.history(branch["conversation_id"]), ensure_ascii=False)
    first_branch = store.fork(cid, streamed["id"])
    assert [message["text"] for message in store.snapshot(first_branch["conversation_id"])["messages"]] == [
        "慢速任务", "未来才完成的回答"]
    # Old rows without completion times use the public sequence boundary alone.
    with store._db:
        store._db.execute("UPDATE turns SET finished_at=NULL WHERE conversation_id=?", (cid,))
    assert store.fork(cid, streamed["id"])["forked_from"]["message_id"] == streamed["id"]


def test_fork_keeps_execution_order_when_queue_acceptance_order_differs(store, monkeypatch):
    now = [100.0]
    monkeypatch.setattr("decision.tasks.conversation.time.time", lambda: now[0])
    cid = store.create()["conversation_id"]
    first, _ = store.accept(cid, "first", "First", None)
    second, _ = store.accept(cid, "second", "Accepted second", None, enqueue=True)
    third, _ = store.accept(cid, "third", "Runs second", None, enqueue=True)
    store.append(cid, first["turn_id"], "assistant", "First answer")
    store.finish(cid, first["turn_id"], state="completed", kind="reply", text="First answer", model_items=[])
    store.promote_queued(cid, third["turn_id"])
    now[0] = 102
    assert store.claim_queued(cid, third["turn_id"]) == "Runs second"
    store.append(cid, third["turn_id"], "assistant", "Second answer")
    store.finish(cid, third["turn_id"], state="completed", kind="reply", text="Second answer", model_items=[])
    now[0] = 104
    assert store.claim_queued(cid, second["turn_id"]) == "Accepted second"
    point = store.append(cid, second["turn_id"], "assistant", "Last answer")
    store.finish(cid, second["turn_id"], state="completed", kind="reply", text="Last answer", model_items=[])
    branch = store.fork(cid, point["id"])
    assert [message["text"] for message in store.snapshot(branch["conversation_id"])["messages"]] == [
        "First", "First answer", "Runs second", "Second answer", "Accepted second", "Last answer"]


def test_fork_rejects_missing_non_assistant_unfinished_and_nonfinal_points(store):
    cid = store.create()["conversation_id"]
    turn, first = complete(store, cid, "question")
    final = store.append(cid, turn["turn_id"], "assistant", "Final answer")
    tool = store.append(cid, turn["turn_id"], "tool", "Private")
    pending, _ = store.accept(cid, "pending", "Pending", None)
    partial = store.append(cid, pending["turn_id"], "assistant", "Partial")
    for message_id in ("question", first["id"], tool["id"], partial["id"]):
        with pytest.raises(ConversationError, match="fork_point_conflict"):
            store.fork(cid, message_id)
    for source, message_id, reason in (("missing", final["id"], "conversation_not_found"),
                                       (cid, "missing", "message_not_found")):
        with pytest.raises(ConversationError, match=reason):
            store.fork(source, message_id)
    assert len(store.list()["conversations"]) == 1


@pytest.mark.parametrize("table", ["messages", "attachments"])
def test_fork_copy_is_one_atomic_transaction(store, table):
    cid = store.create()["conversation_id"]
    store.save_attachment(cid, "file", {"name": "file.txt", "mime_type": "text/plain", "data": b"bytes", "size": 5, "content": []})
    _, point = complete(store, cid, "first", attachment_ids=["file"])
    tables = ("conversations", "turns", "messages", "attachments", "approvals", "conversation_shares", "conversation_context")
    counts = {name: store._db.execute(f"SELECT COUNT(*) FROM {name}").fetchone()[0] for name in tables}
    with store._db:
        store._db.execute(f"""CREATE TRIGGER fail_fork BEFORE INSERT ON {table}
            WHEN (SELECT forked_from FROM conversations WHERE id=NEW.conversation_id) IS NOT NULL
            BEGIN SELECT RAISE(ABORT, 'fork_copy_failed'); END""")
    with pytest.raises(sqlite3.IntegrityError, match="fork_copy_failed"):
        store.fork(cid, point["id"])
    assert {name: store._db.execute(f"SELECT COUNT(*) FROM {name}").fetchone()[0] for name in tables} == counts
    assert not store._db.in_transaction and store.attachment(cid, "file")["data"] == b"bytes"


def test_continuing_a_branch_uses_plain_history_and_preserves_the_source_owner(agent, monkeypatch):
    store = agent._conversation_store()
    cid = agent.create_conversation()["conversation_id"]
    _, point = complete(store, cid, "first", text="Historical question", answer="Historical answer")
    pending, _ = store.accept(cid, "pending", "Owned source task", None)
    owner = agent._new_run("Owned source task")
    store.bind_run(cid, pending["turn_id"], owner.run_id)
    seen = []

    async def runner(history, **kwargs):
        seen.append(history)
        assert "historical discussion" in kwargs["instructions"]
        return TurnResult("reply", "New analysis", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    branch = agent.fork_conversation(cid, point["id"])
    assert agent._loop is None and not agent._conversation_turn_futures
    assert submit(agent, branch["conversation_id"], "new-question", "换个角度分析")["state"] == "completed"
    assert seen == [[{"role": "user", "content": "Historical question"},
                     {"role": "assistant", "content": "Historical answer"}]]
    assert agent._active_run is owner and store.turn(cid, pending["turn_id"])["state"] == "running"
