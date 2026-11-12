"""User-facing Agent messages retain their actual execution owner across SSE."""

import asyncio
from types import SimpleNamespace

import pytest

from gateway.services.event_handlers import handle_agent_message
from tests.decision.test_agent_task_lifecycle import agent as agent
from tests.decision.test_agent_task_lifecycle import script, tool
from tests.decision.test_semantic_target_verification import harness as harness


def test_agent_start_and_result_messages_keep_the_run_id_in_gateway_events(agent):
    script(agent, tool("done", summary="status returned"))
    agent.module.run_agent_task("current status")
    run = agent.module._active_run
    run.future.result(timeout=2)
    assert len(agent.chats) >= 2
    assert all(message["run_id"] == run.run_id for message in agent.chats)
    events = []
    gateway = SimpleNamespace(push_event=events.append)
    for message in agent.chats:
        handle_agent_message(gateway, message)
    assert all(event["run_id"] == run.run_id for event in events)


def test_superseded_say_cannot_publish_as_the_new_run(agent):
    old = agent.module._new_run("current status")
    current = agent.module._new_run("battery level")
    with pytest.raises(asyncio.CancelledError):
        agent.module._tool_say("old task message", run=old)
    assert not agent.chats
    agent.module._tool_say("current task message", run=current)
    assert agent.chats[-1]["run_id"] == current.run_id


def test_unowned_domain_message_is_not_assigned_to_an_agent():
    events = []
    handle_agent_message(SimpleNamespace(push_event=events.append), {"text": "domain update"})
    assert events[0]["run_id"] == ""


def test_persisted_conversation_message_keeps_identity_revision_and_state_in_sse():
    events = []
    message = {"conversation_id": "chat", "id": "answer", "turn_id": "turn", "seq": 7,
               "revision": 3, "role": "assistant", "text": "完整累积内容", "kind": "reply", "created_at": 12,
               "metadata": {"status": "streaming"}, "private_model_items": "not public"}
    handle_agent_message(SimpleNamespace(push_event=events.append), message)
    assert {key: events[0][key] for key in message if key != "private_model_items"} == {
        key: value for key, value in message.items() if key != "private_model_items"}
    assert events[0]["type"] == "agent_message" and "private_model_items" not in events[0]


def test_semantic_message_retains_the_instruction_owner(harness):
    events = []
    harness.module.agent_message.subscribe(events.append)
    harness.module._instruction_owner = "agent-semantic"
    harness.module._chat("thinking", "Checking candidate", phase="target_verification")
    assert events[-1]["run_id"] == "agent-semantic"
