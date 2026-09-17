"""Exercise the real OpenAI SDK against an in-memory HTTP transport, never an API."""

import asyncio
import json
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from decision.llm.client import LLMConfig, LLMError, MoonshotClient, OpenAIClient
from decision.modules.llm import LLMModule, LLMRequest
from decision.semantic_navigation.observation import ObjectTarget
from decision.semantic_navigation.verification import VerificationSample, verification_messages
from decision.vision.vlm_scene import VLMSceneAgent
from tests.decision.test_semantic_target_verification import harness

openai = pytest.importorskip("openai")
# Use the transport library of the installed SDK (the project permits SDK >= 1).
try:
    from openai._base_client import httpx
except ImportError:
    from openai._base_client import httpx2 as httpx


def event(delta):
    payload = {"id": "reply", "object": "chat.completion.chunk", "created": 1,
               "model": "test", "choices": [{"index": 0, "delta": delta}]}
    return f"data: {json.dumps(payload)}\n\n".encode()


@pytest.fixture
def transport(monkeypatch):
    monkeypatch.setenv("MOONSHOT_API_KEY", "test-key-never-sent-to-network")
    monkeypatch.setenv("OPENAI_API_KEY", "test-key-never-sent-to-network")
    wire = SimpleNamespace(requests=[], constructors=[], text='{"verdict":"match"}',
                           reasoning="", error=False, error_message="test model rejected",
                           reply_stream=None, on_request=lambda: None)
    constructor = openai.AsyncOpenAI

    def handle(request):
        wire.requests.append(request)
        wire.on_request()
        if wire.error:
            return httpx.Response(400, json={"error": {
                "message": wire.error_message, "type": "invalid_request"}})
        if wire.reply_stream is not None:
            return httpx.Response(200, stream=wire.reply_stream, headers={"content-type": "text/event-stream"})
        body = json.loads(request.content)
        if body.get("stream"):
            data = event({"reasoning_content": wire.reasoning})
            data += event({"content": wire.text}) + b"data: [DONE]\n\n"
            return httpx.Response(200, content=data, headers={"content-type": "text/event-stream"})
        return httpx.Response(200, json={"id": "reply", "created": 1, "model": body["model"],
                                       "object": "chat.completion", "choices": [{"index": 0, "message": {
                                           "role": "assistant", "content": wire.text,
                                           "reasoning_content": wire.reasoning}}]})

    def create(**kwargs):
        wire.constructors.append(kwargs.copy())
        return constructor(**kwargs, max_retries=0, http_client=httpx.AsyncClient(transport=httpx.MockTransport(handle)))

    monkeypatch.setattr(openai, "AsyncOpenAI", create)
    return wire


def image_messages():
    target = ObjectTarget("chair-1", "chair", (2.0, 0.0, 1.0))
    sample = VerificationSample(101.0, target.position, (1.5, 0.0, 0.3),
                                (4, 4, 24, 24), np.zeros((32, 32, 3), dtype=np.uint8))
    return verification_messages("Find the red chair beside the desk", target, sample)


@pytest.mark.parametrize("backend", ["kimi", "moonshot"])
def test_kimi_defaults_accept_verification_images_using_platform_contract(transport, backend):
    module = LLMModule(backend=backend)
    client = module._create_client()
    assert client.config.model == "kimi-k2.6"
    assert client.config.base_url == "https://api.moonshot.cn/v1"
    assert client.supports_vision is True
    assert VLMSceneAgent(client)._has_vision is True
    messages = image_messages()

    async def run():
        try:
            assert await client.chat(messages, temperature=0.0) == transport.text
        finally:
            await client.close()

    asyncio.run(run())
    request = transport.requests[0]
    body = json.loads(request.content)
    assert str(request.url) == "https://api.moonshot.cn/v1/chat/completions"
    assert body["messages"] == messages
    assert body["model"] == "kimi-k2.6"
    assert body["thinking"] == {"type": "disabled"}
    assert "temperature" not in body
    assert "claude-code" not in request.headers["user-agent"]


def test_single_image_preserves_explicit_model_and_final_content(transport):
    client = OpenAIClient(LLMConfig(model="gpt-4o-mini", max_retries=0))
    transport.text = ""
    transport.reasoning = "This is private reasoning, not a final verdict"

    async def run():
        try:
            result = await client.chat_with_image("Check target", "dGVzdA==", temperature=0.0)
            assert result == ""
        finally:
            await client.close()

    asyncio.run(run())
    body = json.loads(transport.requests[0].content)
    assert body["model"] == "gpt-4o-mini"
    assert body["temperature"] == 0.0


@pytest.mark.parametrize("via_helper", [False, True])
def test_unverified_moonshot_model_cannot_send_image_requests(transport, via_helper):
    client = MoonshotClient(LLMConfig(model="custom-text-model", max_retries=0))
    assert client.config.model == "custom-text-model"
    assert client.supports_vision is False

    async def run():
        try:
            with pytest.raises(LLMError, match="vision"):
                if via_helper:
                    await client.chat_with_image("Check target", "dGVzdA==")
                else:
                    await client.chat(image_messages())
        finally:
            await client.close()

    asyncio.run(run())
    assert not transport.requests


@pytest.mark.parametrize("error", [False, True])
def test_module_reports_resolved_model_and_entire_request_latency(transport, monkeypatch, error):
    import decision.modules.llm as module_source

    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(module_source, "time", SimpleNamespace(monotonic=lambda: clock.now, time=lambda: 100.0))
    transport.error = error
    transport.on_request = lambda: setattr(clock, "now", 10.125)
    module = LLMModule(backend="kimi")
    received, replies = threading.Event(), []
    module.response.subscribe(lambda response: (replies.append(response), received.set()))
    module.setup()
    try:
        module._client.config.max_retries = 0
        module.request._deliver(LLMRequest(messages=image_messages(), request_id="verify-1", temperature=0.0))
        assert received.wait(3.0)
        response = replies[0]
        assert response.request_id == "verify-1"
        assert response.model == "kimi-k2.6"
        assert response.ok is not error
        assert response.latency_ms == pytest.approx(125.0)
        assert module.health()["llm"]["model"] == "kimi-k2.6"
        if not error:
            assert module.health()["llm"]["avg_ms"] == pytest.approx(125.0)
    finally:
        asyncio.run_coroutine_threadsafe(module.client.close(), module._loop).result(timeout=3.0)
        module.stop()


def test_moonshot_alias_checks_the_same_key_at_preflight(monkeypatch):
    monkeypatch.delenv("MOONSHOT_API_KEY", raising=False)
    assert "MOONSHOT_API_KEY" in (LLMModule(backend="moonshot").preflight() or "")


def test_stream_is_closed_when_the_module_deadline_cancels_a_reply(transport):
    closed = threading.Event()

    class PendingReply(httpx.AsyncByteStream):
        async def __aiter__(self):
            yield event({"reasoning_content": "still processing"})
            await asyncio.Event().wait()

        async def aclose(self):
            closed.set()

    transport.reply_stream = PendingReply()
    client = OpenAIClient(LLMConfig(max_retries=0))

    async def run():
        try:
            with pytest.raises(asyncio.TimeoutError):
                await asyncio.wait_for(client.chat(image_messages()), timeout=0.1)
            assert closed.is_set()
        finally:
            await client.close()

    asyncio.run(run())


def test_exhausted_temperature_retry_reports_an_error_instead_of_empty_success(transport):
    transport.error = True
    transport.error_message = "invalid temperature"
    client = OpenAIClient(LLMConfig(max_retries=0))

    async def run():
        try:
            with pytest.raises(LLMError, match="invalid temperature"):
                await client.chat([{"role": "user", "content": "test"}])
        finally:
            await client.close()

    asyncio.run(run())
    assert len(transport.requests) == 1


def test_empty_timeout_exception_is_an_error_and_counts_as_transient(transport):
    class PendingReply(httpx.AsyncByteStream):
        async def __aiter__(self):
            yield event({"reasoning_content": "still processing"})
            await asyncio.Event().wait()

    transport.reply_stream = PendingReply()
    module = LLMModule(backend="kimi", timeout_sec=0.1)
    received, replies = threading.Event(), []
    module.response.subscribe(lambda response: (replies.append(response), received.set()))
    module.setup()
    try:
        module.request._deliver(LLMRequest.simple("test", request_id="timeout-1"))
        assert received.wait(3.0)
        assert replies[0].ok is False
        assert "TimeoutError" in replies[0].error
        assert replies[0].latency_ms >= 50.0
        assert module.health()["llm"]["circuit_breaker"] == "closed"
        assert module.health()["llm"]["consecutive_failures"] == 1
    finally:
        asyncio.run_coroutine_threadsafe(module.client.close(), module._loop).result(timeout=3.0)
        module.stop()


def test_provider_switch_does_not_inherit_another_providers_endpoint_and_model(monkeypatch):
    monkeypatch.setenv("CUSTOM_TEST_KEY", "not-a-real-key")
    module = LLMModule(backend="openai", model="custom-openai", api_key_env="CUSTOM_TEST_KEY",
                       base_url="https://example.invalid/v1")
    module.setup()
    try:
        assert module.reconfigure_backend("llm", "kimi")["ok"]
        assert module.client.config.model == "kimi-k2.6"
        assert module.client.config.base_url == "https://api.moonshot.cn/v1"
        assert module.client.config.api_key_env == "MOONSHOT_API_KEY"
        assert module.reconfigure_backend("llm", "moonshot", model="custom-kimi",
                                          base_url="https://custom.invalid/v1")["ok"]
        assert module.reconfigure_backend("llm", "kimi", temperature=0.5)["ok"]
        assert module.client.config.model == "custom-kimi"
        assert module.client.config.base_url == "https://custom.invalid/v1"
    finally:
        module.stop()


def test_stop_cancels_pending_request_and_closes_sdk_client(transport):
    reading, closed = threading.Event(), threading.Event()

    class PendingReply(httpx.AsyncByteStream):
        async def __aiter__(self):
            yield event({"reasoning_content": "still processing"})
            reading.set()
            await asyncio.Event().wait()

        async def aclose(self):
            closed.set()

    transport.reply_stream = PendingReply()
    module = LLMModule(backend="kimi")
    module.setup()
    thread = module._loop_thread
    module.request._deliver(LLMRequest.simple("test"))
    assert reading.wait(3.0)
    sdk = module.client._client
    module.stop()
    assert not thread.is_alive()
    assert closed.is_set()
    assert sdk.is_closed()
    assert module._in_flight == 0


def test_reconfigure_closes_the_previous_sdk_client(transport):
    module = LLMModule(backend="kimi")
    module.setup()
    try:
        asyncio.run_coroutine_threadsafe(module.client.chat([{"role": "user", "content": "test"}]),
                                        module._loop).result(timeout=3.0)
        sdk = module.client._client
        assert module.reconfigure_backend("llm", "openai")["ok"]
        assert sdk.is_closed()
    finally:
        module.stop()


@pytest.mark.parametrize("verdict,frames,expected", [
    ("match", 2, "COMPLETED"),
    ("mismatch", 1, "TARGET_MISMATCH"),
    ("uncertain", 3, "TARGET_UNCONFIRMED"),
    ("api_error", 1, "TARGET_VERIFICATION_UNAVAILABLE"),
])
def test_native_reached_to_verification_uses_sdk_and_complete_instruction(harness, transport, verdict, frames, expected):
    h = harness
    if verdict == "uncertain":
        h.modules["nav.commands"].preview_plan = lambda *args: {"feasible": False}
    transport.error = verdict == "api_error"
    transport.text = json.dumps({"target_id": "chair-1", "verdict": verdict, "reason": "test visual evidence"})
    module = LLMModule(backend="kimi")
    module.setup()
    module.client.config.max_retries = 0
    h.modules["LLMModule"] = module
    h.module.on_system_modules(h.modules)
    h.module.llm_request.subscribe(module.request._deliver)
    module.response.subscribe(h.module.llm_response._deliver)
    received = threading.Event()
    module.response.subscribe(lambda response: received.set())
    try:
        h.reach()
        assert h.statuses[-1] == "VERIFYING_TARGET"
        for index in range(frames):
            received.clear()
            h.observe(100.1 + index * 0.6)
            assert received.wait(3.0)
        if verdict == "uncertain":
            assert h.completed.wait(1.0)
        assert h.statuses[-1] == expected
        assert len(h.goals) == 1
        assert len(transport.requests) == frames
        for request in transport.requests:
            body = json.loads(request.content)
            assert body["model"] == "kimi-k2.6"
            assert body["thinking"] == {"type": "disabled"}
            content = body["messages"][1]["content"]
            assert json.loads(content[0]["text"])["instruction"] == "find the red chair"
            assert [part["type"] for part in content] == ["text", "image_url", "image_url"]
        if verdict != "api_error":
            assert all(item["model"] == "kimi-k2.6" for item in h.module._verification.evidence)
    finally:
        module.stop()
