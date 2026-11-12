"""Fork HTTP creates an independent conversation without model or task dispatch."""

import pytest

from tests.decision.test_conversation_forks import complete
from tests.gateway.test_conversations import gateway as gateway


def test_http_forks_an_archived_source_and_reads_its_copied_attachment(gateway):
    agent, client = gateway.agent, gateway.client
    store = agent._conversation_store()
    cid = store.create()["conversation_id"]
    store.save_attachment(cid, "file", {"name": "notes.txt", "mime_type": "text/plain",
                                        "data": b"copied bytes", "size": 12,
                                        "content": [{"type": "input_text", "text": "reference"}]})
    _, point = complete(store, cid, "first", attachment_ids=["file"])
    store.update(cid, archived=True)
    response = client.post(f"/api/v1/agent/conversations/{cid}/fork", json={"message_id": point["id"]})
    assert response.status_code == 201
    branch = response.json()
    assert branch["conversation_id"] != cid and branch["archived"] is False
    assert branch["forked_from"]["message_id"] == point["id"]
    path = f"/api/v1/agent/conversations/{branch['conversation_id']}"
    snapshot = client.get(path).json()
    assert snapshot["metadata"] == branch and snapshot["share"] is None and snapshot["approvals"] == []
    assert len(snapshot["messages"]) == 2
    assert client.get(f"{path}/attachments/file").content == b"copied bytes"
    assert client.get(f"/api/v1/agent/conversations/{cid}").json()["metadata"]["archived"] is True
    assert agent._loop is None and not agent._conversation_turn_futures and agent._active_run is None


@pytest.mark.parametrize("body", [{}, {"message_id": ""}, {"message_id": "  "}, {"message_id": "x" * 129},
                                  {"message_id": "answer", "restore_motion": True}, {"message_id": 1}])
def test_http_fork_validates_the_selected_message_without_creating_a_branch(gateway, body):
    cid = gateway.agent.create_conversation()["conversation_id"]
    assert gateway.client.post(f"/api/v1/agent/conversations/{cid}/fork", json=body).status_code == 422
    assert len(gateway.agent.list_conversations()["conversations"]) == 1


def test_http_fork_distinguishes_missing_history_from_invalid_points(gateway):
    store, client = gateway.agent._conversation_store(), gateway.client
    cid = store.create()["conversation_id"]
    _, point = complete(store, cid, "question")
    pending, _ = store.accept(cid, "pending", "Pending", None)
    partial = store.append(cid, pending["turn_id"], "assistant", "Still running")
    for source, message_id, status, reason in (("missing", point["id"], 404, "conversation_not_found"),
                                               (cid, "missing", 404, "message_not_found"),
                                               (cid, "question", 409, "fork_point_conflict"),
                                               (cid, partial["id"], 409, "fork_point_conflict")):
        response = client.post(f"/api/v1/agent/conversations/{source}/fork", json={"message_id": message_id})
        assert response.status_code == status and response.json()["detail"]["reason"] == reason
    assert len(store.list()["conversations"]) == 1 and store.turn(cid, pending["turn_id"])["state"] == "running"
