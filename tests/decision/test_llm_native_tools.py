"""Native function calling through the real SDK and an in-memory HTTP transport."""

import asyncio
import json
from types import SimpleNamespace

import pytest

from decision.llm.client import LLMConfig, LLMError, MoonshotClient, OpenAIClient
from decision.tasks.agent import AgentLoop

openai = pytest.importorskip("openai")
try:
    from openai._base_client import httpx
except ImportError:
    from openai._base_client import httpx2 as httpx


def _event(delta, finish_reason=None):
    payload = {
        "id": "reply", "object": "chat.completion.chunk", "created": 1, "model": "test",
        "choices": [{"index": 0, "delta": delta, "finish_reason": finish_reason}],
    }
    return f"data: {json.dumps(payload)}\n\n".encode()


def _call(name, arguments, call_id="call_inspect"):
    return {
        "id": call_id, "type": "function",
        "function": {"name": name, "arguments": arguments},
    }


def _reply(call, *, reasoning="", content=None, finish_reason="tool_calls"):
    delta = {"role": "assistant", "tool_calls": [{"index": 0, **call}]}
    if reasoning:
        delta["reasoning_content"] = reasoning
    if content is not None:
        delta["content"] = content
    return _event(delta) + _event({}, finish_reason) + b"data: [DONE]\n\n"


def _context():
    return {
        "robot_x": 0.0, "robot_y": 0.0, "nav_status": "IDLE", "visible_objects": "none",
        "memory_context": "none", "camera_image": None, "camera_available": False,
    }


_INSPECT_TOOLS = [{
    "type": "function", "function": {
        "name": "inspect_task", "description": "Inspect a retained task.",
        "parameters": {
            "type": "object", "properties": {"text": {"type": "string"}}, "required": ["text"],
        },
    },
}]


@pytest.fixture
def native_transport(monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "test-key-never-sent-to-network")
    monkeypatch.setenv("MOONSHOT_API_KEY", "test-key-never-sent-to-network")
    wire = SimpleNamespace(requests=[], constructors=[], replies=[], status=200, stream=None)
    constructor = openai.AsyncOpenAI

    def handle(request):
        wire.requests.append(json.loads(request.content))
        if wire.status != 200:
            return httpx.Response(wire.status, json={"error": {
                "message": "test server unavailable", "type": "server_error",
            }})
        if wire.stream is not None:
            return httpx.Response(200, stream=wire.stream, headers={"content-type": "text/event-stream"})
        assert wire.replies, "Unexpected extra model request"
        return httpx.Response(200, content=wire.replies.pop(0), headers={"content-type": "text/event-stream"})

    def create(**kwargs):
        wire.constructors.append(kwargs.copy())
        return constructor(**kwargs, http_client=httpx.AsyncClient(transport=httpx.MockTransport(handle)))

    monkeypatch.setattr(openai, "AsyncOpenAI", create)
    return wire


def _client(model="kimi-k3"):
    cls = OpenAIClient if model == "gpt-4o-mini" else MoonshotClient
    return cls(LLMConfig(model=model, max_retries=0))


@pytest.mark.parametrize("model", ["gpt-4o-mini", "kimi-k2.6", "kimi-k3"])
def test_native_request_sends_tool_definitions_as_protocol_parameters(native_transport, model):
    native_transport.replies = [_reply(_call("inspect_task", '{"text":"door"}'))]
    client = _client(model)
    messages = [{"role": "user", "content": "Inspect the door task"}]

    async def exercise():
        try:
            await client.chat_with_tools(messages, tools=_INSPECT_TOOLS)
        finally:
            await client.close()

    asyncio.run(exercise())
    body = native_transport.requests[0]
    assert body["messages"] == messages
    assert body["tools"] == _INSPECT_TOOLS
    assert body["tool_choice"] == "auto"
    assert body["parallel_tool_calls"] is False
    assert body["stream"] is True
    assert body["model"] == model
    assert native_transport.constructors[0]["max_retries"] == 0
    if model == "kimi-k3":
        assert body["reasoning_effort"] == "low"
        assert body["max_completion_tokens"] == 4096
        assert "temperature" not in body
    elif model == "kimi-k2.6":
        assert body["thinking"] == {"type": "disabled"}
        assert "temperature" not in body


def test_native_stream_reassembles_the_complete_assistant_message(native_transport):
    native_transport.replies = [
        _event({"role": "assistant", "reasoning_content": "Check ", "content": "I will ", "tool_calls": [{
            "index": 0, "id": "call_inspect", "type": "function",
            "function": {"name": "inspect_task", "arguments": '{"text":'},
        }]})
        + _event({"reasoning_content": "the retained task.", "content": "inspect.", "tool_calls": [{
            "index": 0, "function": {"arguments": '"水瓶"}'},
        }]})
        + _event({}, "tool_calls") + b"data: [DONE]\n\n"
    ]
    client = _client()

    async def exercise():
        try:
            return await client.chat_with_tools([{"role": "user", "content": "Inspect"}], tools=_INSPECT_TOOLS)
        finally:
            await client.close()

    message = asyncio.run(exercise())
    assert message["role"] == "assistant"
    assert message["content"] == "I will inspect."
    assert message["reasoning_content"] == "Check the retained task."
    assert message["tool_calls"] == [_call("inspect_task", '{"text":"水瓶"}')]


def test_plain_chat_still_returns_only_the_final_text(native_transport):
    native_transport.replies = [
        _event({"role": "assistant", "reasoning_content": "Private planning."})
        + _event({"content": "Final answer."}, "stop") + b"data: [DONE]\n\n"
    ]
    client = _client()

    async def exercise():
        try:
            return await client.chat([{"role": "user", "content": "Describe status"}])
        finally:
            await client.close()

    assert asyncio.run(exercise()) == "Final answer."
    assert "tools" not in native_transport.requests[0]


def test_agent_preserves_native_assistant_and_matching_results_across_rounds(native_transport):
    first = _call("query_memory", '{"text":"door"}', "call_first")
    second = _call("query_memory", '{"text":"chair"}', "call_second")
    native_transport.replies = [
        _reply(first, reasoning="First private reasoning.", content="Checking the door."),
        _reply(second, reasoning="Second private reasoning."),
        _reply(_call("done", '{"summary":"Inspection finished","success":true}', "call_done")),
    ]
    client = _client()
    handled = []
    loop = AgentLoop(
        llm_client=client, tool_registry={}, tool_list=[], context_fn=_context,
        tool_handlers={"query_memory": lambda text: handled.append(text) or f"Retained: {text}"},
    )

    async def exercise():
        try:
            return await loop.run("Inspect door and chair memory")
        finally:
            await client.close()

    state = asyncio.run(exercise())
    assert state.completed and state.failure_reason == ""
    assert state.summary == "Inspection finished"
    assert handled == ["door", "chair"]
    assert len(native_transport.requests) == 3
    for request, expected_calls in zip(native_transport.requests[1:], ([first], [first, second])):
        assistants = [message for message in request["messages"] if message["role"] == "assistant"]
        assert [message["tool_calls"][0] for message in assistants] == expected_calls
        assert assistants[0]["reasoning_content"] == "First private reasoning."
        assert assistants[0]["content"] == "Checking the door."
        results = [message for message in request["messages"] if message["role"] == "tool"]
        assert [message["tool_call_id"] for message in results] == [call["id"] for call in expected_calls]
        assert [message["content"] for message in results] == [
            f"Retained: {json.loads(call['function']['arguments'])['text']}" for call in expected_calls
        ]
    second_assistant = [
        message for message in native_transport.requests[2]["messages"] if message["role"] == "assistant"
    ][1]
    assert second_assistant["reasoning_content"] == "Second private reasoning."
    system_prompt = native_transport.requests[0]["messages"][0]["content"]
    assert "Respond with JSON only" not in system_prompt
    assert '"tool": "name"' not in system_prompt
    assert native_transport.requests[0]["tools"] == loop._tools


def test_native_reasoning_is_not_spoken_to_the_operator(native_transport):
    native_transport.replies = [
        _reply(_call("say", '{"text":"Task is idle."}'), reasoning="Private internal planning."),
        _reply(_call("done", '{"summary":"Status reported","success":true}', "call_done")),
    ]
    client = _client()
    spoken = []
    loop = AgentLoop(
        llm_client=client, tool_registry={}, tool_list=[], context_fn=_context,
        tool_handlers={"say": lambda text: spoken.append(text) or "Spoken"},
    )

    async def exercise():
        try:
            return await loop.run("Report current status")
        finally:
            await client.close()

    state = asyncio.run(exercise())
    assert spoken == ["Task is idle."]
    assert state.summary == "Status reported"


@pytest.mark.parametrize("finish_reason", ["length", None, "stop"])
def test_unfinished_tool_reply_cannot_execute_a_handler(native_transport, finish_reason):
    native_transport.replies = [_reply(
        _call("stop_servo", "{}"), finish_reason=finish_reason,
    )]
    client = _client()
    calls = []
    loop = AgentLoop(
        llm_client=client, tool_registry={}, tool_list=[], context_fn=_context,
        tool_handlers={"stop_servo": lambda: calls.append("stop")}, max_steps=1,
    )

    async def exercise():
        try:
            return await loop.run("Stop my task")
        finally:
            await client.close()

    state = asyncio.run(exercise())
    assert not calls
    assert not loop._tool_call_audit
    assert state.failure_reason == "llm_error"


def test_multiple_native_calls_are_rejected_before_any_handler_runs(native_transport):
    native_transport.replies = [
        _event({"role": "assistant", "tool_calls": [
            {"index": 0, **_call("stop_servo", "{}", "call_one")},
            {"index": 1, **_call("stop_servo", "{}", "call_two")},
        ]}) + _event({}, "tool_calls") + b"data: [DONE]\n\n"
    ]
    client = _client()
    calls = []
    loop = AgentLoop(
        llm_client=client, tool_registry={}, tool_list=[], context_fn=_context,
        tool_handlers={"stop_servo": lambda: calls.append("stop")}, max_steps=1,
    )

    async def exercise():
        try:
            return await loop.run("Stop my task")
        finally:
            await client.close()

    state = asyncio.run(exercise())
    assert state.failure_reason == "llm_error"
    assert not calls
    assert not loop._tool_call_audit


@pytest.mark.parametrize("arguments", ['{"broken"', "[]", "null", '"stop"', "false"])
def test_invalid_native_arguments_cannot_become_a_no_argument_handler_call(native_transport, arguments):
    native_transport.replies = [_reply(_call("stop_servo", arguments))]
    client = _client()
    calls = []
    loop = AgentLoop(
        llm_client=client, tool_registry={}, tool_list=[], context_fn=_context,
        tool_handlers={"stop_servo": lambda: calls.append("stop")}, max_steps=1,
    )

    async def exercise():
        try:
            return await loop.run("Stop my task")
        finally:
            await client.close()

    state = asyncio.run(exercise())
    assert not calls
    assert state.failure_reason
    if state.failure_reason != "llm_error":
        assert any("VALIDATION_ERROR" in message.get("content", "") for message in state.messages
                   if message["role"] == "tool")


@pytest.mark.parametrize("cancel_method", ["explicit", "deadline"])
def test_cancelled_native_request_closes_the_incomplete_http_stream(native_transport, cancel_method):
    client = _client()

    async def exercise():
        reading, closed = asyncio.Event(), asyncio.Event()

        class PendingReply(httpx.AsyncByteStream):
            async def __aiter__(self):
                yield _event({"tool_calls": [{
                    "index": 0, "id": "call_pending", "type": "function",
                    "function": {"name": "inspect_task", "arguments": '{"text":'},
                }]})
                reading.set()
                await asyncio.Event().wait()

            async def aclose(self):
                closed.set()

        native_transport.stream = PendingReply()
        request = asyncio.create_task(client.chat_with_tools(
            [{"role": "user", "content": "Inspect task"}], tools=_INSPECT_TOOLS,
        ))
        try:
            await asyncio.wait_for(reading.wait(), timeout=1.0)
            if cancel_method == "explicit":
                request.cancel()
                with pytest.raises(asyncio.CancelledError):
                    await request
            else:
                with pytest.raises(asyncio.TimeoutError):
                    await asyncio.wait_for(request, timeout=0.02)
            assert closed.is_set()
        finally:
            request.cancel()
            await asyncio.gather(request, return_exceptions=True)
            await client.close()

    asyncio.run(exercise())


def test_native_request_does_not_add_sdk_retries(native_transport):
    native_transport.status = 503
    client = _client()

    async def exercise():
        try:
            with pytest.raises(LLMError):
                await client.chat_with_tools([{"role": "user", "content": "Inspect"}], tools=_INSPECT_TOOLS)
        finally:
            await client.close()

    asyncio.run(exercise())
    assert len(native_transport.requests) == 1
    assert native_transport.constructors[0]["max_retries"] == 0
