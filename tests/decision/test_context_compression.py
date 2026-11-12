"""Context checkpoints never replace the original transcript or authorize tools."""

import asyncio
import json
from types import SimpleNamespace

import pytest

from decision.context import compression
from decision.context.transcript import serialize
from decision.llm.client import LLMConfig
from decision.tasks.conversation import ConversationStore


def seed(store, count=8, text="旧材料" * 100):
    cid = store.create()["conversation_id"]
    turns = []
    for index in range(count):
        turn, _ = store.accept(cid, f"input-{index}", f"地图 map-alpha {index}: {text}", None, resolved_model="kimi-k3")
        items = [{"role": "user", "content": f"问题 {index}"},
                 {"type": "function_call", "call_id": f"call-{index}", "name": "inspect", "arguments": "{}"},
                 {"type": "function_call_output", "call_id": f"call-{index}", "output": text},
                 {"role": "assistant", "content": f"回答 {index}，尚未实机验证"}]
        store.append(cid, turn["turn_id"], "tool", f"map-alpha receipt {index}", kind="inspect")
        store.finish(cid, turn["turn_id"], state="completed", kind="reply", text=f"回答 {index}", model_items=items)
        turns.append(turn)
    active, _ = store.accept(cid, "active", "继续", None, resolved_model="kimi-k3")
    return cid, active["turn_id"], turns


def result(text=None, kind="reply"):
    summary = {key: [] for key in compression.SUMMARY_FIELDS}
    summary["facts"] = ["地图 map-alpha；停车点 P-17。"]
    summary["uncertainties"] = ["尚未实机验证。"]
    return SimpleNamespace(kind=kind, text=text if text is not None else json.dumps(summary, ensure_ascii=False))


async def prepare(store, cid, active, events):
    return await compression.prepare_context(store, cid, active, config=LLMConfig(), selected_model="kimi-k3",
                                             overhead="", on_event=events.append, reasoning="low")


@pytest.mark.asyncio
async def test_checkpoint_keeps_tail_pairs_and_history_and_survives_restart(tmp_path, monkeypatch):
    path = tmp_path / "chat.db"
    store = ConversationStore(path)
    cid, active, turns = seed(store)
    original = store.history(cid, exclude_turn_id=active, model="kimi-k3")
    calls = []

    async def summarize(**kwargs):
        calls.append(kwargs)
        assert not kwargs["tools"] and kwargs["allow_specialists"] is False
        assert kwargs["reasoning"] == "low"
        assert "call-0" in kwargs["text"] and "尚未实机验证" in kwargs["text"]
        return result()

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", summarize)
    monkeypatch.setattr(compression, "TRIGGER_CHARACTERS", 1000)
    events = []
    compact = await prepare(store, cid, active, events)
    assert len(compact) == 1 + 4 * 4 and compact[1:] == original[-16:]
    assert store.history(cid, exclude_turn_id=active, model="kimi-k3") == original
    assert events[-1]["status"] == "completed"
    assert events[-1]["after_characters"] < events[-1]["before_characters"]
    assert store.context_checkpoint(cid)["through_ordinal"] > 0
    assert await prepare(store, cid, active, []) == compact and len(calls) == 1
    assert store.search_history(cid, "map-alpha")["matches"]
    restored = store.read_context_turn(cid, turns[0]["turn_id"])
    assert "call-0" in restored["text"]
    store.close()
    store = ConversationStore(path)
    monkeypatch.setattr(compression, "TRIGGER_CHARACTERS", 100000)
    resumed = await prepare(store, cid, active, [])
    assert resumed == compact and len(calls) == 1
    store.close()


@pytest.mark.asyncio
@pytest.mark.parametrize("bad", ["not JSON", "{}", None])
async def test_failure_preserves_history_and_checkpoint(tmp_path, monkeypatch, bad):
    store = ConversationStore(tmp_path / "chat.db")
    cid, active, _ = seed(store)
    original = store.history(cid, exclude_turn_id=active, model="kimi-k3")
    async def summarize(**_kwargs):
        return result(text=bad, kind="failed" if bad is None else "reply")
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", summarize)
    monkeypatch.setattr(compression, "TRIGGER_CHARACTERS", 1000)
    events = []
    assert await prepare(store, cid, active, events) == original
    assert store.context_checkpoint(cid)["through_ordinal"] == 0
    assert events[-1]["history_preserved"] is True
    store.close()


@pytest.mark.asyncio
async def test_cancel_cannot_commit_partial_summary(tmp_path, monkeypatch):
    store = ConversationStore(tmp_path / "chat.db")
    cid, active, _ = seed(store)
    entered = asyncio.Event()
    async def summarize(**_kwargs):
        entered.set()
        await asyncio.Event().wait()
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", summarize)
    monkeypatch.setattr(compression, "TRIGGER_CHARACTERS", 1000)
    events = []
    task = asyncio.create_task(prepare(store, cid, active, events))
    await asyncio.wait_for(entered.wait(), 1)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert store.context_checkpoint(cid)["through_ordinal"] == 0
    assert events[-1]["status"] == "cancelled"
    store.close()


@pytest.mark.asyncio
async def test_multi_page_failure_and_timeout_do_not_publish_partial_checkpoint(tmp_path, monkeypatch):
    store = ConversationStore(tmp_path / "chat.db")
    cid, active, _ = seed(store)
    monkeypatch.setattr(compression, "TRIGGER_CHARACTERS", 1000)
    monkeypatch.setattr(compression, "SOURCE_PAGE_CHARACTERS", 2000)
    calls = []
    async def summarize(**kwargs):
        calls.append(kwargs)
        return result() if len(calls) == 1 else result(text="invalid")
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", summarize)
    original = store.history(cid, exclude_turn_id=active, model="kimi-k3")
    assert await prepare(store, cid, active, []) == original
    assert len(calls) == 2 and "P-17" in calls[1]["text"]
    assert store.context_checkpoint(cid)["through_ordinal"] == 0
    async def slow(**_kwargs):
        await asyncio.sleep(1)
        return result()
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", slow)
    monkeypatch.setattr(compression, "TIMEOUT_SECONDS", 0.01)
    assert await prepare(store, cid, active, []) == original
    assert store.context_checkpoint(cid)["through_ordinal"] == 0
    store.close()


@pytest.mark.asyncio
async def test_pending_turn_blocks_prefix_and_cross_chat_recovery(tmp_path, monkeypatch):
    store = ConversationStore(tmp_path / "chat.db")
    cid, active, turns = seed(store)
    with store._db:
        store._db.execute("UPDATE turns SET state='running' WHERE turn_id=?", (turns[0]["turn_id"],))
    async def unexpected(**_kwargs):
        pytest.fail("Must not summarize across an unfinished turn")
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", unexpected)
    monkeypatch.setattr(compression, "TRIGGER_CHARACTERS", 1000)
    assert await prepare(store, cid, active, []) == store.history(cid, exclude_turn_id=active, model="kimi-k3")
    other = store.create()["conversation_id"]
    assert store.read_context_turn(other, turns[0]["turn_id"])["error"] == "turn_not_found"
    assert store.search_history(other, "map-alpha")["matches"] == []
    assert not store.save_context(cid, active, 0, 4, "stale", "kimi-k3")
    store.close()


def test_recovery_pages_do_not_drop_text_or_encode_images(tmp_path):
    store = ConversationStore(tmp_path / "chat.db")
    cid, active, turns = seed(store, count=1, text="a" * 30000)
    first = store.read_context_turn(cid, turns[0]["turn_id"])
    pages = [first["text"]]
    while first["next_offset"] is not None:
        first = store.read_context_turn(cid, turns[0]["turn_id"], first["next_offset"])
        pages.append(first["text"])
    assert "a" * 30000 in "".join(pages)
    assert "SECRET_BASE64" not in serialize({"type": "input_image", "image_url": "data:image/png;base64,SECRET_BASE64"})
    assert "error" in store.read_context_turn(cid, active, -1)
    store.close()


def test_cancelled_turn_retains_completed_tool_receipts_for_summary(tmp_path):
    store = ConversationStore(tmp_path / "chat.db")
    cid = store.create()["conversation_id"]
    turn, _ = store.accept(cid, "one", "读取相机", None)
    store.append(cid, turn["turn_id"], "tool", json.dumps({"status": "captured", "timestamp": 123,
                 "image_data_url": "SECRET_BASE64"}), kind="camera")
    store.finish(cid, turn["turn_id"], state="cancelled", kind="control", text="已取消", model_items=[])
    recovered = store.read_context_turn(cid, turn["turn_id"])["text"]
    assert "captured" in recovered and "123" in recovered and "SECRET_BASE64" not in recovered
    store.close()


@pytest.mark.asyncio
async def test_summary_omits_only_exact_duplicate_turn_text(tmp_path, monkeypatch):
    store = ConversationStore(tmp_path / "chat.db")
    cid, active, turns = seed(store)
    marker = "KEEP_SOURCE_ONCE"
    with store._db:
        store._db.execute("UPDATE turns SET input=?,text=?,model_items=? WHERE turn_id=?",
            (marker, "ONE_REPLY", json.dumps([{"role": "user", "content": marker},
                                              {"role": "assistant", "content": "ONE_REPLY"}]), turns[0]["turn_id"]))
    captured = []
    async def summarize(**kwargs):
        captured.append(json.loads(kwargs["text"])["source_page"])
        return result()
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", summarize)
    monkeypatch.setattr(compression, "TRIGGER_CHARACTERS", 1000)
    await prepare(store, cid, active, [])
    text = "\n".join(captured)
    assert text.count(marker) == 1 and text.count("ONE_REPLY") == 1
    assert "问题 1" in text and "地图 map-alpha 1" in text
    assert marker in store.read_context_turn(cid, turns[0]["turn_id"])["text"]
    store.close()


@pytest.mark.asyncio
async def test_complete_json_fence_is_accepted_but_prose_is_not(tmp_path, monkeypatch):
    store = ConversationStore(tmp_path / "chat.db")
    cid, active, _ = seed(store)
    monkeypatch.setattr(compression, "TRIGGER_CHARACTERS", 1000)
    async def prose(**_kwargs):
        return result(text="Here is the answer:\n" + result().text)
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", prose)
    await prepare(store, cid, active, [])
    assert store.context_checkpoint(cid)["through_ordinal"] == 0
    async def fenced(**_kwargs):
        return result(text="```json\n" + result().text + "\n```")
    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", fenced)
    await prepare(store, cid, active, [])
    assert "P-17" in store.context_checkpoint(cid)["summary"]
    store.close()
