"""Offline contracts against the real optional SDK runner."""

import asyncio
import json
from unittest.mock import AsyncMock

import pytest

pytest.importorskip("agents")
from agents.models.interface import Model
from openai.types.responses import (
    Response,
    ResponseCompletedEvent,
    ResponseFunctionToolCall,
    ResponseOutputMessage,
    ResponseOutputText,
    ResponseTextDeltaEvent,
)

from decision.llm.client import LLMConfig
from decision.tasks.sdk_agent import run_turn


def message(text):
    return ResponseOutputMessage(
        id="message",
        type="message",
        role="assistant",
        status="completed",
        content=[ResponseOutputText(type="output_text", text=text, annotations=[])],
    )


def call(name, args, call_id="call_1"):
    return ResponseFunctionToolCall(
        id=call_id,
        type="function_call",
        call_id=call_id,
        name=name,
        arguments=json.dumps(args),
        status="completed",
    )


class FakeModel(Model):
    def __init__(self, responses, *, delay=0, usage=None):
        self.responses = iter(responses)
        self.inputs = []
        self.settings = []
        self.tools = []
        self.delay = delay
        self.usage = usage
        self.started = asyncio.Event()
        self.settled = asyncio.Event()

    async def get_response(self, *args, **kwargs):
        raise AssertionError("Expected streamed execution")

    async def stream_response(self, system_instructions, input, model_settings, *args, **kwargs):
        self.inputs.append(input)
        self.settings.append(model_settings)
        self.tools.append(kwargs.get("tools", args[0] if args else []))
        self.started.set()
        try:
            await asyncio.sleep(self.delay)
            output = next(self.responses)
            if isinstance(output, Exception):
                raise output
            for item in output:
                if item.type == "message":
                    yield ResponseTextDeltaEvent(
                        type="response.output_text.delta",
                        delta=item.content[0].text,
                        item_id=item.id,
                        output_index=0,
                        content_index=0,
                        sequence_number=0,
                        logprobs=[],
                    )
            yield ResponseCompletedEvent(
                type="response.completed",
                sequence_number=1,
                response=Response.model_construct(
                    id="response",
                    object="response",
                    model="offline",
                    created_at=0,
                    status="completed",
                    output=output,
                    usage=self.usage,
                ),
            )
        finally:
            self.settled.set()


TOOL = {
    "type": "function",
    "function": {
        "name": "inspect",
        "description": "Read task status",
        "parameters": {
            "type": "object",
            "properties": {"target": {"type": "string"}},
            "required": ["target"],
        },
    },
}


async def turn(model, **kwargs):
    options = dict(
        history=[],
        instructions="Help the operator",
        tools=[TOOL],
        invoke_tool=AsyncMock(return_value={"ok": True}),
        config=LLMConfig(),
        on_event=lambda _event: None,
        completion_check=lambda: None,
        model=model,
    )
    options.update(kwargs)
    return await run_turn("hello", **options)


@pytest.mark.asyncio
async def test_reply_and_history():
    events = []
    first = await turn(FakeModel([[message("Hello")]]), on_event=events.append)
    assert first.kind == "reply"
    assert first.text == "Hello"
    assert events == [{"type": "text_delta", "delta": "Hello", "item_id": "message"}]
    model = FakeModel([[message("Again")]])
    second = await turn(model, history=first.model_items)
    assert model.inputs[0] == [*first.model_items, {"role": "user", "content": "hello"}]
    assert len(second.model_items) == len(first.model_items) + 2
    assert model.settings[0].parallel_tool_calls is False
    assert first.metrics["first_text_ms"] is not None
    assert first.metrics["input_tokens"] is None


@pytest.mark.asyncio
async def test_metrics_count_provider_usage_across_tool_rounds():
    from openai.types.responses.response_usage import ResponseUsage

    usage = ResponseUsage(input_tokens=100, output_tokens=20, total_tokens=120,
                          input_tokens_details={"cached_tokens": 0, "cache_write_tokens": 0}, output_tokens_details={"reasoning_tokens": 0})
    result = await turn(FakeModel([[call("inspect", {"target": "map"})], [message("Verified")]], usage=usage))
    assert result.kind == "reply"
    assert result.metrics["model_responses"] == 2
    assert result.metrics["input_tokens"] == 200 and result.metrics["output_tokens"] == 40
    assert result.metrics["first_tool_ms"] is not None
    assert result.metrics["first_text_ms"] <= result.metrics["elapsed_ms"]


@pytest.mark.asyncio
async def test_in_loop_budget_changes_model_view_not_history_or_tool_execution():
    from openai.types.responses.response_usage import ResponseUsage

    usage = ResponseUsage(input_tokens=25000, output_tokens=20, total_tokens=25020,
        input_tokens_details={"cached_tokens": 0, "cache_write_tokens": 0}, output_tokens_details={"reasoning_tokens": 0})
    model = FakeModel([[call("inspect", {"target": "first"}, "one")],
                       [call("inspect", {"target": "second"}, "two")], [message("done")]], usage=usage)
    invoke = AsyncMock(side_effect=["x" * 5000, "y" * 5000])
    events = []
    result = await turn(model, invoke_tool=invoke, conversation_turn_id="turn-source", on_event=events.append)
    assert result.kind == "reply" and invoke.await_count == 2
    sent = [item for item in model.inputs[-1] if item.get("type") == "function_call_output"]
    assert "read_conversation_turn" in sent[0]["output"] and sent[1]["output"] == "y" * 5000
    stored = [item for item in result.model_items if item.get("type") == "function_call_output"]
    assert stored[0]["output"] == "x" * 5000
    assert any(event["type"] == "context_budget" and event["provider_input_tokens"] == 25000 for event in events)


@pytest.mark.asyncio
async def test_selected_model_settings_and_attachments_reach_sdk():
    model = FakeModel([[message("The attachment says 42")]])
    attachment = {"content": [{"type": "input_text", "text": "Uploaded document: 42"}]}
    result = await turn(model, config=LLMConfig(backend="kimi", model="kimi-k2.6"),
                        selected_model="kimi-k3", reasoning="high", attachments=[attachment])
    assert result.kind == "reply"
    assert model.settings[0].extra_body == {"reasoning_effort": "high"}
    assert model.inputs[0][-1]["content"][-1]["text"] == "Uploaded document: 42"
    assert result.model_items[0]["content"][-1]["text"] == "Uploaded document: 42"


@pytest.mark.parametrize("tool_output", [{"state": "running"}, '{"state": "running"}'])
@pytest.mark.asyncio
async def test_tool_output_reaches_model(tool_output):
    invoke = AsyncMock(return_value=tool_output)
    events = []
    model = FakeModel([[call("inspect", {"target": "task"})], [message("Still running")]])
    result = await turn(model, invoke_tool=invoke, on_event=events.append)
    assert result.kind == "reply"
    invoke.assert_awaited_once_with("inspect", {"target": "task"}, "call_1")
    output = next(item for item in model.inputs[1] if item.get("type") == "function_call_output")
    assert json.loads(output["output"]) == {"state": "running"}
    assert [event["type"] for event in events] == ["tool_start", "tool_end", "text_delta"]


@pytest.mark.asyncio
async def test_public_text_deltas_keep_message_identity_across_tool_rounds():
    before, after = message("Checking first"), message("Final answer")
    before.id, after.id = "before-tool", "after-tool"
    events = []
    result = await turn(FakeModel([[before, call("inspect", {"target": "task"})], [after]]), on_event=events.append)
    deltas = [event for event in events if event["type"] == "text_delta"]
    assert [(event["item_id"], event["delta"]) for event in deltas] == [
        ("before-tool", "Checking first"), ("after-tool", "Final answer")]
    assert result.text == "Final answer"


@pytest.mark.asyncio
async def test_clarification_ends_turn():
    model = FakeModel([[call("ask_user", {"question": "Which room?"})]])
    result = await turn(model)
    assert result.kind == "clarification"
    assert result.text == "Which room?"
    assert len(model.inputs) == 1
    assert result.model_items[-1]["type"] == "function_call_output"


@pytest.mark.asyncio
async def test_plan_is_a_structured_read_only_terminal_result():
    invoke = AsyncMock()
    model = FakeModel([[call("propose_plan", {
        "title": "  通道检查方案  ", "summary": "  先只读观察。  ",
        "steps": [{"title": "  检查地图  ", "detail": "  读取定位状态。\n保留观测证据。  "},
                  {"title": "确认风险"}],
    }), call("inspect", {"target": "task"}, "after-plan")]])
    result = await turn(model, invoke_tool=invoke)
    assert result.kind == "plan" and result.failure_reason == "" and result.choices is None
    assert result.plan == {"title": "通道检查方案", "summary": "先只读观察。", "steps": [
        {"title": "检查地图", "detail": "读取定位状态。\n保留观测证据。"}, {"title": "确认风险"}]}
    assert result.text.startswith("# 通道检查方案\n\n> 方案草稿")
    assert "1. **检查地图**\n\n   读取定位状态。\n   保留观测证据。" in result.text
    assert "2. **确认风险**" in result.text
    invoke.assert_not_awaited()
    assert len(model.inputs) == 1


@pytest.mark.parametrize("plan", [
    {"title": " ", "steps": [{"title": "观察"}]},
    {"title": "x" * 121, "steps": [{"title": "观察"}]},
    {"title": 1, "steps": [{"title": "观察"}]},
    {"title": "方案", "steps": []},
    {"title": "方案", "steps": [{"title": "观察"}] * 13},
    {"title": "方案", "steps": "观察"},
    {"title": "方案", "steps": ["观察"]},
    {"title": "方案", "steps": [{}]},
    {"title": "方案", "steps": [{"title": "x" * 161}]},
    {"title": "方案", "steps": [{"title": "观察", "detail": "x" * 2001}]},
    {"title": "方案", "steps": [{"title": "观察", "detail": False}]},
    {"title": "方案", "summary": "x" * 1001, "steps": [{"title": "观察"}]},
    {"title": "方案", "summary": "  ", "steps": [{"title": "观察"}]},
    {"title": "方案", "status": "executed", "steps": [{"title": "观察"}]},
    {"title": "方案", "steps": [{"title": "观察", "permission": "approved"}]},
])
@pytest.mark.asyncio
async def test_invalid_plan_reports_tool_error_and_allows_a_normal_reply(plan):
    invoke = AsyncMock()
    model = FakeModel([[call("propose_plan", plan)], [message("请补全方案内容。")]])
    result = await turn(model, invoke_tool=invoke)
    assert result.kind == "reply" and result.plan is None
    assert result.text == "请补全方案内容。" and len(model.inputs) == 2
    output = next(item for item in model.inputs[1] if item.get("type") == "function_call_output")
    assert "error" in json.loads(output["output"])
    invoke.assert_not_awaited()


@pytest.mark.asyncio
async def test_specialist_cannot_receive_the_terminal_plan_tool():
    model = FakeModel([[message("只读检查摘要")]])
    result = await turn(model, allow_specialists=False)
    assert result.kind == "reply" and result.plan is None
    assert "propose_plan" not in {tool.name for tool in model.tools[0]}


@pytest.mark.asyncio
async def test_completion_requires_runtime_evidence():
    model = FakeModel([[call("done", {"success": True, "summary": "Arrived"})], [message("Still moving")]])
    result = await turn(model, completion_check=lambda: "Navigation has not finished")
    assert result.kind == "reply"
    assert result.text == "Still moving"
    output = model.inputs[1][-1]
    assert json.loads(output["output"])["error"] == "Navigation has not finished"


@pytest.mark.parametrize("success,kind", [(True, "task_complete"), (False, "failed")])
@pytest.mark.asyncio
async def test_done(success, kind):
    result = await turn(FakeModel([[call("done", {"success": success, "summary": "Finished"})]]))
    assert result.kind == kind
    assert result.text == "Finished"


@pytest.mark.parametrize("arguments", [[], {}, {"success": "yes", "summary": "Done"}])
@pytest.mark.asyncio
async def test_bad_arguments_return_error(arguments):
    invoke = AsyncMock()
    name = "done" if "success" in arguments else "inspect"
    model = FakeModel([[call(name, arguments)], [message("Correcting arguments")]])
    result = await turn(model, invoke_tool=invoke)
    assert result.kind == "reply"
    assert "error" in json.loads(model.inputs[1][-1]["output"])
    invoke.assert_not_awaited()


@pytest.mark.asyncio
async def test_cancel_slow_model_prevents_tool():
    invoke = AsyncMock()
    model = FakeModel([[call("inspect", {"target": "task"})]], delay=60)
    task = asyncio.create_task(turn(model, invoke_tool=invoke))
    await asyncio.wait_for(model.started.wait(), timeout=5)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert model.settled.is_set()
    invoke.assert_not_awaited()


@pytest.mark.asyncio
async def test_timeout_settles_model():
    invoke = AsyncMock()
    model = FakeModel([[call("inspect", {"target": "task"})]], delay=60)
    result = await turn(model, invoke_tool=invoke, timeout=0.02)
    assert result.kind == "failed"
    assert result.failure_reason == "Agent turn timed out"
    # The SDK can time out while preparing tools, before entering the model.
    assert not model.started.is_set() or model.settled.is_set()
    invoke.assert_not_awaited()


@pytest.mark.asyncio
async def test_failure_keeps_completed_tool_history():
    model = FakeModel([[call("inspect", {"target": "task"})], RuntimeError("Model unavailable")])
    result = await turn(model)
    assert result.kind == "failed"
    assert result.failure_reason == "Model unavailable"
    output = next(item for item in result.model_items if item.get("type") == "function_call_output")
    assert json.loads(output["output"]) == {"ok": True}


@pytest.mark.asyncio
async def test_invalid_output_text_replay_keeps_diagnostics_out_of_reply():
    from agents.models.chatcmpl_converter import Converter

    class ReplayModel(FakeModel):
        async def stream_response(self, system_instructions, input, model_settings, *args, **kwargs):
            Converter.items_to_messages(input)
            async for event in super().stream_response(system_instructions, input, model_settings, *args, **kwargs):
                yield event

    invoke = AsyncMock()
    history = [{"role": "assistant", "content": [
        {"type": "output_text", "text": "| 地图 | 状态 |\n| --- | --- |\n| test6 | READY |", "annotations": []}]}]
    result = await turn(ReplayModel([[message("unused")]]), history=history, invoke_tool=invoke)
    assert result.kind == "failed"
    assert result.text == "对话内容解析失败，本轮未完成。请重试。"
    assert result.failure_reason.startswith("Unknown content:")
    assert "output_text" in result.failure_reason
    assert "annotations" not in result.text
    invoke.assert_not_awaited()


@pytest.mark.asyncio
async def test_cancel_running_tool_prevents_next_tool():
    started = asyncio.Event()
    settled = asyncio.Event()
    invoked = []

    async def invoke(name, args, call_id):
        invoked.append(call_id)
        started.set()
        try:
            await asyncio.sleep(60)
        finally:
            settled.set()

    model = FakeModel(
        [
            [
                call("inspect", {"target": "one"}, "call_1"),
                call("inspect", {"target": "two"}, "call_2"),
            ]
        ]
    )
    task = asyncio.create_task(turn(model, invoke_tool=invoke))
    await asyncio.wait_for(started.wait(), timeout=5)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert settled.is_set()
    assert invoked == ["call_1"]


@pytest.mark.asyncio
@pytest.mark.parametrize("selected,backend,url,key", [
    ("gpt-6.1-sol", "responses", "https://api.openai.com/v1", "OPENAI_API_KEY"),
    ("claude-sonnet-5-5", "chat", "https://api.anthropic.com/v1/", "ANTHROPIC_API_KEY"),
])
async def test_selected_provider_routes_sdk_and_tools(monkeypatch, selected, backend, url, key):
    captured = []
    model = FakeModel([[call("inspect", {"target": "map"})], [message("ready")]])

    def factory(kind):
        def create(**kwargs):
            captured.append((kind, kwargs["model"], str(kwargs["openai_client"].base_url).rstrip("/")))
            return model
        return create

    monkeypatch.setenv(key, "test-only")
    monkeypatch.setattr("agents.models.openai_responses.OpenAIResponsesModel", factory("responses"))
    monkeypatch.setattr("agents.models.openai_chatcompletions.OpenAIChatCompletionsModel", factory("chat"))
    invoked = AsyncMock(return_value={"ok": True})
    result = await turn(None, config=LLMConfig(backend="kimi", model="kimi-k3"),
                        selected_model=selected, invoke_tool=invoked)
    assert result.kind == "reply" and result.text == "ready"
    assert captured == [(backend, selected, url.rstrip("/"))]
    invoked.assert_awaited_once_with("inspect", {"target": "map"}, "call_1")


@pytest.mark.asyncio
async def test_clarification_returns_choices_and_preserves_freeform_question():
    result = await turn(FakeModel([[call("ask_user", {"question": "要补地面还是墙面？", "options": ["地面", "墙面"]})]]))
    assert result.kind == "clarification"
    assert result.choices == ["地面", "墙面"]
    assert result.text == "要补地面还是墙面？"


@pytest.mark.asyncio
async def test_codex_direct_responses_retains_owned_tools_and_history(monkeypatch):
    from decision.llm import codex

    model = FakeModel([[call("inspect", {"target": "map"})], [message("地图已检查")]])
    captured = []
    monkeypatch.setattr(codex.account, "access_token", lambda: "private-test-token")

    def create(**kwargs):
        captured.append((kwargs["model"], str(kwargs["openai_client"].base_url)))
        return model

    monkeypatch.setattr("agents.models.openai_responses.OpenAIResponsesModel", create)
    invoke = AsyncMock(return_value={"ok": True})
    history = [{"role": "user", "content": "上一次"}, {"role": "assistant", "content": "已记录"}]
    result = await turn(None, selected_model="codex:gpt-6.1-sol", invoke_tool=invoke, history=history)
    assert captured == [("gpt-6.1-sol", "https://chatgpt.com/backend-api/codex/")]
    invoke.assert_awaited_once_with("inspect", {"target": "map"}, "call_1")
    assert result.kind == "reply" and result.text == "地图已检查"
    assert result.model_items[:len(history)] == history
    assert all(setting.store is False for setting in model.settings)
    assert all(setting.temperature is None for setting in model.settings)


@pytest.mark.asyncio
async def test_codex_actual_sdk_http_stream_carries_token_and_tool_result(monkeypatch):
    import httpx
    import openai

    from decision.llm import codex

    requests = []
    original = openai.AsyncOpenAI

    def respond(request):
        requests.append(request)
        output = [call("inspect", {"target": "map"})] if len(requests) == 1 else [message("已检查")]
        event = ResponseCompletedEvent(type="response.completed", sequence_number=1,
            response=Response.model_construct(id=f"response-{len(requests)}", object="response", model="gpt-6.1-sol",
                created_at=0, status="completed", output=output, usage=None))
        return httpx.Response(200, headers={"content-type": "text/event-stream"},
                              text="data: " + event.model_dump_json() + "\n\n")

    monkeypatch.setattr(openai, "AsyncOpenAI", lambda **kwargs: original(**kwargs,
                        http_client=httpx.AsyncClient(transport=httpx.MockTransport(respond))))
    monkeypatch.setattr(codex.account, "access_token", lambda: "private-http-test-token")
    invoke = AsyncMock(return_value={"ok": True})
    result = await turn(None, selected_model="codex:gpt-6.1-sol", invoke_tool=invoke)
    assert result.text == "已检查"
    assert len(requests) == 2
    for request in requests:
        assert str(request.url) == "https://chatgpt.com/backend-api/codex/responses"
        assert request.headers["authorization"] == "Bearer private-http-test-token"
        body = json.loads(request.content)
        assert body["stream"] is True and body["store"] is False
        assert body["model"] == "gpt-6.1-sol" and body["instructions"] == "Help the operator"
        assert "temperature" not in body and "max_output_tokens" not in body
    assert any(item.get("type") == "function_call_output" for item in json.loads(requests[1].content)["input"])
    invoke.assert_awaited_once()


@pytest.mark.asyncio
async def test_invalid_clarification_options_do_not_finish_turn():
    model = FakeModel([[call("ask_user", {"question": "哪一种？", "options": ["地面", "地面"]})], [message("请描述范围")]])
    result = await turn(model)
    assert result.kind == "reply" and result.choices is None
    assert any(item.get("type") == "function_call_output" and "distinct" in item["output"] for item in result.model_items)


@pytest.mark.asyncio
async def test_camera_tool_is_real_image_content_for_the_next_model_call():
    image = "data:image/jpeg;base64,YWJj"
    model = FakeModel([[call("inspect", {"target": "camera"})], [message("Camera received")]])
    result = await turn(model, config=LLMConfig(backend="kimi", model="kimi-k3"),
                        invoke_tool=AsyncMock(return_value={"success": True, "image_data_url": image}))
    assert result.kind == "reply"
    output = next(item for item in model.inputs[1] if item.get("type") == "function_call_output")
    assert json.loads(output["output"])["success"]
    stored = next(item for item in result.model_items if item.get("type") == "function_call_output")
    assert stored["output"][1] == {"type": "input_image", "image_url": image, "detail": "auto"}
    from agents.models.chatcmpl_converter import Converter
    messages = Converter.items_to_messages(model.inputs[1])
    assert any(part.get("type") == "image_url" and part["image_url"]["url"] == image
               for item in messages if isinstance(item.get("content"), list) for part in item["content"])

@pytest.mark.asyncio
async def test_manager_runs_bounded_specialists_and_synthesizes_reports():
    from copy import deepcopy
    read_tools = []
    for name in ('list_saved_maps', 'get_agent_status'):
        tool = deepcopy(TOOL)
        tool['function']['name'] = name
        read_tools.append(tool)
    events = []
    model = FakeModel([[call('review_with_specialists', {'task': '检查地图与观测'})],
                       [message('地图检查摘要')], [message('观测检查摘要')], [message('汇总完成')]])
    result = await turn(model, tools=read_tools, on_event=events.append)
    assert result.kind == 'reply' and result.text == '汇总完成'
    progress = [event for event in events if event['type'] == 'specialist']
    assert [event['status'] for event in progress].count('running') == 2
    assert [event['status'] for event in progress].count('completed') == 2
    output = next(item for item in result.model_items if item.get('type') == 'function_call_output')
    assert len(json.loads(output['output'])['reviews']) == 2

@pytest.mark.asyncio
async def test_specialist_batch_cannot_repeat_in_one_manager_turn():
    from copy import deepcopy
    read_tools = []
    for name in ('list_saved_maps', 'get_agent_status'):
        item = deepcopy(TOOL)
        item['function']['name'] = name
        read_tools.append(item)
    model = FakeModel([[call('review_with_specialists', {'task': '检查'}, 'batch-one')],
                       [message('地图摘要')], [message('观测摘要')],
                       [call('review_with_specialists', {'task': '再检查'}, 'batch-two')], [message('沿用已有结果')]])
    events = []
    result = await turn(model, tools=read_tools, on_event=events.append)
    assert result.kind == 'reply'
    assert len([event for event in events if event['type'] == 'specialist' and event['status'] == 'running']) == 2
    outputs = [json.loads(item['output']) for item in result.model_items if item.get('type') == 'function_call_output']
    assert 'Only one specialist review' in outputs[-1]['error']
