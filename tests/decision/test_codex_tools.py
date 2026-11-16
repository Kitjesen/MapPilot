"""LingTu owned tools over the native Codex stdio protocol."""

import asyncio
import json
from pathlib import Path
from unittest.mock import AsyncMock

import pytest

pytest.importorskip("agents")

from tests.decision.codex_fixture import codex_server
from tests.decision.test_sdk_agent import TOOL, turn


def tool_responses(peer):
    return [item["result"] for item in peer.messages()
            if not item.get("method") and "result" in item]


@pytest.mark.asyncio
@pytest.mark.parametrize("reasoning", ["default", "low", "medium", "high", "xhigh"])
async def test_codex_reasoning_selection_reaches_native_turn(codex_server, reasoning):
    result = await turn(None, selected_model="codex:gpt-6.1-sol", reasoning=reasoning)
    assert result.kind == "reply"
    request = next(item for item in codex_server.messages() if item.get("method") == "turn/start")
    assert request["params"]["effort"] == ("medium" if reasoning == "default" else reasoning)


@pytest.mark.asyncio
async def test_codex_asks_with_options_and_interrupts_native_turn(codex_server):
    codex_server.scenario.update(tool="ask_user", arguments={
        "question": "选择目标？", "options": ["入口", "出口"]})
    invoke = AsyncMock()
    result = await turn(None, selected_model="codex:gpt-6.1-sol", invoke_tool=invoke)
    assert result.kind == "clarification" and result.text == "选择目标？"
    assert result.choices == ["入口", "出口"]
    invoke.assert_not_awaited()
    assert tool_responses(codex_server)[0]["success"] is True
    # A following RPC drains the interrupt acknowledgement through the same reader.
    assert codex_server.server.status()["connected"] is True
    interrupt = next(item for item in codex_server.messages() if item.get("method") == "turn/interrupt")
    assert interrupt["params"] == {"threadId": "thread-1", "turnId": "turn-1"}


@pytest.mark.asyncio
async def test_codex_plan_uses_existing_plan_semantics(codex_server):
    codex_server.scenario.update(tool="propose_plan", arguments={
        "title": "巡检", "steps": [{"title": "检查入口"}]})
    result = await turn(None, selected_model="codex:gpt-6.1-sol")
    assert result.kind == "plan"
    assert result.plan == {"title": "巡检", "steps": [{"title": "检查入口"}]}
    assert tool_responses(codex_server)[0]["success"] is True


@pytest.mark.asyncio
@pytest.mark.parametrize("reason,kind,success", [(None, "task_complete", True), ("robot still moving", "reply", False)])
async def test_codex_done_checks_runtime_completion(codex_server, reason, kind, success):
    codex_server.scenario.update(tool="done", arguments={"success": True, "summary": "已完成"})
    checks = []

    def check():
        checks.append(True)
        return reason

    result = await turn(None, selected_model="codex:gpt-6.1-sol", completion_check=check)
    assert result.kind == kind and checks == [True]
    assert tool_responses(codex_server)[0]["success"] is success


@pytest.mark.asyncio
async def test_codex_unknown_tool_is_rejected(codex_server):
    codex_server.scenario["tool"] = "unregistered_motion"
    invoke = AsyncMock()
    result = await turn(None, selected_model="codex:gpt-6.1-sol", invoke_tool=invoke)
    assert result.kind == "reply"
    invoke.assert_not_awaited()
    response = tool_responses(codex_server)[0]
    assert response["success"] is False
    assert "not registered" in response["contentItems"][0]["text"]


@pytest.mark.asyncio
async def test_codex_tool_error_is_reported_and_not_reexecuted(codex_server):
    codex_server.scenario["duplicate"] = True
    invoke = AsyncMock(side_effect=RuntimeError("camera unavailable"))
    result = await turn(None, selected_model="codex:gpt-6.1-sol", invoke_tool=invoke)
    assert result.kind == "reply"
    invoke.assert_awaited_once()
    responses = tool_responses(codex_server)
    assert len(responses) == 2 and responses[0] == responses[1]
    assert responses[0]["success"] is False
    assert json.loads(responses[0]["contentItems"][0]["text"])["error"] == "camera unavailable"


@pytest.mark.asyncio
async def test_codex_camera_tool_returns_native_image_and_replays_history(codex_server):
    image = "data:image/jpeg;base64,YWJj"
    invoke = AsyncMock(return_value={"success": True, "image_data_url": image})
    result = await turn(None, selected_model="codex:gpt-6.1-sol", invoke_tool=invoke)
    response = tool_responses(codex_server)[0]
    assert response["success"] is True
    assert response["contentItems"][1] == {"type": "inputImage", "imageUrl": image}
    followup = await turn(None, selected_model="codex:gpt-6.1-sol", invoke_tool=invoke, history=result.model_items)
    assert followup.kind == "reply"
    inputs = [item["params"]["input"] for item in codex_server.messages() if item.get("method") == "turn/start"][-1]
    assert any(part["type"] == "localImage" for part in inputs)
    assert all(not Path(part["path"]).exists() for part in inputs if part["type"] == "localImage")


@pytest.mark.asyncio
async def test_codex_new_turn_ignores_late_cancelled_thread_events(codex_server):
    codex_server.scenario.update(wait_for_interrupt_first=True, late_previous_events=True)
    started = asyncio.Event()

    async def pending(*args):
        started.set()
        await asyncio.Future()

    running = asyncio.create_task(turn(None, selected_model="codex:gpt-6.1-sol", invoke_tool=pending))
    await asyncio.wait_for(started.wait(), timeout=10)
    running.cancel()
    with pytest.raises(asyncio.CancelledError):
        await asyncio.wait_for(running, timeout=10)
    events = []
    result = await turn(None, selected_model="codex:gpt-6.1-sol", on_event=events.append)
    assert result.kind == "reply" and result.text == "地图已检查"
    assert not any(event.get("delta") == "OLD CANCELLED REPLY"
                   or event.get("params", {}).get("threadId") == "thread-1" for event in events)


@pytest.mark.asyncio
async def test_codex_cancel_interrupts_motion_and_settles_owned_tool(codex_server):
    codex_server.scenario.update(tool="navigate", arguments={"target": "entry"},
                                 duplicate_pending=True, wait_for_interrupt=True)
    started, settled = asyncio.Event(), asyncio.Event()

    async def motion(*args):
        started.set()
        try:
            await asyncio.Future()
        finally:
            settled.set()

    invoke = AsyncMock(side_effect=motion)
    tool = {"type": "function", "function": {**TOOL["function"], "name": "navigate"}}
    running = asyncio.create_task(turn(None, selected_model="codex:gpt-6.1-sol", tools=[tool], invoke_tool=invoke))
    await asyncio.wait_for(started.wait(), timeout=10)
    running.cancel()
    with pytest.raises(asyncio.CancelledError):
        await asyncio.wait_for(running, timeout=10)
    await asyncio.wait_for(settled.wait(), timeout=2)
    invoke.assert_awaited_once_with("navigate", {"target": "entry"}, "call_1")
    assert codex_server.server.status()["connected"] is True
    interrupts = [item for item in codex_server.messages() if item.get("method") == "turn/interrupt"]
    assert len(interrupts) == 1
    assert interrupts[0]["params"] == {"threadId": "thread-1", "turnId": "turn-1"}
