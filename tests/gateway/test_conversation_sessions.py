"""Conversation metadata and read-only share HTTP contracts."""

from decision.tasks.sdk_agent import TurnResult
from tests.gateway.test_conversations import finish
from tests.gateway.test_conversations import gateway as gateway


def test_http_lists_renames_archives_restores_and_freezes_shares(gateway, monkeypatch):
    async def runner(text, history, **kwargs):
        return TurnResult("reply", "Synthetic answer", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    client = gateway.client
    cid = client.post("/api/v1/agent/conversations", json={}).json()["conversation_id"]
    path = f"/api/v1/agent/conversations/{cid}"
    metadata = client.get("/api/v1/agent/conversations").json()["conversations"]
    assert len(metadata) == 1 and metadata[0]["conversation_id"] == cid
    assert client.post(f"{path}/share").status_code == 422
    first = client.post(f"{path}/messages", json={"message_id": "first", "text": "Synthetic question"})
    assert first.status_code == 202
    assert finish(gateway, first.json())["state"] == "completed"
    snapshot = client.get(path).json()
    assert snapshot["metadata"]["title"] == "Synthetic question" and snapshot["share"] is None
    renamed = client.patch(path, json={"title": "Customer session"})
    assert renamed.status_code == 200 and renamed.json()["title"] == "Customer session"
    assert client.patch(path, json={"title": " "}).status_code == 422
    assert client.patch(path, json={"archived": "yes"}).status_code == 422
    shared = client.post(f"{path}/share")
    assert shared.status_code == 200
    receipt = shared.json()
    public_path = f"/api/v1/agent/shares/{receipt['token']}"
    frozen = client.get(public_path).json()
    assert frozen["title"] == "Customer session" and len(frozen["messages"]) == 2
    assert set(frozen) == {"title", "created_at", "messages"}
    assert client.get(path).json()["share"] == receipt
    assert client.post(f"{path}/share").json() == receipt
    second = client.post(f"{path}/messages", json={"message_id": "second", "text": "Later question"})
    finish(gateway, second.json())
    assert client.get(public_path).json() == frozen
    archived = client.patch(path, json={"archived": True})
    assert archived.status_code == 200 and archived.json()["archived"]
    assert client.get("/api/v1/agent/conversations").json()["conversations"] == []
    assert client.get("/api/v1/agent/conversations?archived=true").json()["conversations"][0]["conversation_id"] == cid
    rejected = client.post(f"{path}/messages", json={"message_id": "new", "text": "Do something"})
    assert rejected.status_code == 409 and rejected.json()["detail"]["reason"] == "conversation_archived"
    assert client.patch(path, json={"archived": False}).status_code == 200
    assert client.delete(f"{path}/share").json() == {"revoked": True}
    assert client.delete(f"{path}/share").json() == {"revoked": True}
    assert client.get(public_path).status_code == 404
    newer = client.post(f"{path}/share").json()
    assert newer["token"] != receipt["token"] and newer["message_count"] == 4
    assert client.patch("/api/v1/agent/conversations/missing", json={"title": "New"}).status_code == 404
    assert gateway.agent._active_run is None


def test_http_archive_rejects_running_turn_without_changing_owner(gateway):
    client = gateway.client
    cid = client.post("/api/v1/agent/conversations").json()["conversation_id"]
    gateway.agent._conversation_store().accept(cid, "running", "Pending", None)
    response = client.patch(f"/api/v1/agent/conversations/{cid}", json={"archived": True})
    assert response.status_code == 409 and response.json()["detail"]["reason"] == "conversation_active"
    assert client.get(f"/api/v1/agent/conversations/{cid}").json()["metadata"]["archived"] is False
    assert gateway.agent._active_run is None


def test_http_search_matches_public_bodies_with_literal_queries_and_archive_scope(gateway):
    store = gateway.agent._conversation_store()
    cid = store.create()["conversation_id"]
    turn, _ = store.accept(cid, "question", "检查中文通道 ROUTE 和字面 %_", None)
    answer = store.append(cid, turn["turn_id"], "assistant", "开头" * 180 + "中文通道检查完成")
    store.append(cid, turn["turn_id"], "tool", "PRIVATE_TOOL")
    store.append(cid, turn["turn_id"], "thinking", "PRIVATE_THINKING")
    store.finish(cid, turn["turn_id"], state="completed", kind="reply", text="完成", model_items=[])
    store.update(cid, title="只存在于标题")
    client = gateway.client
    path = "/api/v1/agent/conversations/search"
    rows = client.get(path, params={"q": "中文通道"}).json()["results"]
    assert len(rows) == 2
    assert rows[0]["message_id"] == answer["id"] and rows[0]["seq"] == answer["seq"]
    assert rows[0]["conversation_id"] == cid and rows[0]["title"] == "只存在于标题"
    assert rows[0]["role"] == "assistant" and "中文通道" in rows[0]["snippet"]
    for query in ("route", "%_"):
        result = client.get(path, params={"q": query}).json()["results"]
        assert len(result) == 1 and result[0]["message_id"] == "question"
    for query in ("PRIVATE_TOOL", "PRIVATE_THINKING", "只存在于标题", "   ", "x' OR 1=1--"):
        assert client.get(path, params={"q": query}).json() == {"results": []}
    for params in ({}, {"q": ""}, {"q": "x" * 201}, {"q": "x", "archived": "invalid"}):
        assert client.get(path, params=params).status_code == 422
    store.update(cid, archived=True)
    assert client.get(path, params={"q": "中文通道"}).json() == {"results": []}
    assert len(client.get(path, params={"q": "中文通道", "archived": "true"}).json()["results"]) == 2
    assert store.turn(cid, turn["turn_id"])["state"] == "completed"


def test_http_search_limits_message_hits_without_creating_a_turn(gateway):
    store = gateway.agent._conversation_store()
    cid = store.create()["conversation_id"]
    turn, _ = store.accept(cid, "question", "needle", None)
    for index in range(40):
        store.append(cid, turn["turn_id"], "assistant", f"needle {index}")
    store.finish(cid, turn["turn_id"], state="completed", kind="reply", text="done", model_items=[])
    response = gateway.client.get("/api/v1/agent/conversations/search", params={"q": "needle"})
    assert response.status_code == 200 and len(response.json()["results"]) == 30
    assert len(store.snapshot(cid)["turns"]) == 1 and gateway.agent._active_run is None
