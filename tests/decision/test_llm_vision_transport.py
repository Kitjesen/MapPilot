"""Exercise the real OpenAI SDK against an in-memory HTTP transport, never an API."""

import asyncio
import json
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from decision.llm.client import LLMConfig, LLMError, MoonshotClient, OpenAIClient
from decision.modules.llm import LLMModule, LLMRequest
from decision.modules.visual_servo import VisualServoModule
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


def event(delta, finish_reason=None):
    payload = {"id": "reply", "object": "chat.completion.chunk", "created": 1,
               "model": "test", "choices": [{"index": 0, "delta": delta, "finish_reason": finish_reason}]}
    return f"data: {json.dumps(payload)}\n\n".encode()


@pytest.fixture
def transport(monkeypatch):
    monkeypatch.setenv("MOONSHOT_API_KEY", "test-key-never-sent-to-network")
    monkeypatch.setenv("OPENAI_API_KEY", "test-key-never-sent-to-network")
    wire = SimpleNamespace(requests=[], constructors=[], text='{"verdict":"match"}', texts=[],
                           reasoning="", error=False, error_status=400, error_message="test model rejected",
                           reply_stream=None, finish_reason=None, on_request=lambda: None)
    constructor = openai.AsyncOpenAI

    def handle(request):
        wire.requests.append(request)
        wire.on_request()
        if wire.error:
            return httpx.Response(wire.error_status, json={"error": {
                "message": wire.error_message, "type": "invalid_request"}})
        if wire.reply_stream is not None:
            return httpx.Response(200, stream=wire.reply_stream, headers={"content-type": "text/event-stream"})
        body = json.loads(request.content)
        response_text = wire.texts.pop(0) if wire.texts else wire.text
        if body.get("stream"):
            data = event({"reasoning_content": wire.reasoning})
            data += event({"content": response_text}, wire.finish_reason) + b"data: [DONE]\n\n"
            return httpx.Response(200, content=data, headers={"content-type": "text/event-stream"})
        return httpx.Response(200, json={"id": "reply", "created": 1, "model": body["model"],
                                       "object": "chat.completion", "choices": [{"index": 0, "message": {
                                           "role": "assistant", "content": response_text,
                                           "reasoning_content": wire.reasoning}}]})

    def create(**kwargs):
        wire.constructors.append(kwargs.copy())
        return constructor(**kwargs, http_client=httpx.AsyncClient(transport=httpx.MockTransport(handle)))

    monkeypatch.setattr(openai, "AsyncOpenAI", create)
    return wire


@pytest.mark.parametrize("max_retries", [0, 1])
def test_application_retry_limit_counts_actual_http_attempts(transport, monkeypatch, max_retries):
    async def skip_retry_delay(_seconds):
        pass

    monkeypatch.setattr("decision.llm.client.asyncio.sleep", skip_retry_delay)
    transport.error = True
    transport.error_status = 500
    client = OpenAIClient(LLMConfig(max_retries=max_retries))

    async def run():
        try:
            with pytest.raises(LLMError, match=f"after {max_retries + 1} attempts"):
                await client.chat([{"role": "user", "content": "test"}])
        finally:
            await client.close()

    asyncio.run(run())
    assert len(transport.requests) == max_retries + 1


def test_text_tool_fallback_sends_valid_history_on_the_next_http_request(transport):
    from decision.tasks.agent import AgentLoop

    transport.texts = [
        '{"tool":"find_object","args":{"target":"a bottle of water"}}',
        '{"tool":"done","args":{"summary":"test result","success":true}}',
    ]
    calls = []
    client = OpenAIClient(LLMConfig(max_retries=0))
    loop = AgentLoop(
        llm_client=SimpleNamespace(chat=client.chat),
        tool_registry={},
        tool_list=[],
        context_fn=lambda: {
            "robot_x": 0.0, "robot_y": 0.0, "nav_status": "IDLE", "visible_objects": "none",
        },
        tool_handlers={"find_object": lambda target: calls.append(target) or "test observation"},
    )

    async def run():
        try:
            return await loop.run("find a bottle of water")
        finally:
            await client.close()

    state = asyncio.run(run())
    assert state.completed and not state.failure_reason
    assert calls == ["a bottle of water"]
    assert len(transport.requests) == 2
    messages = json.loads(transport.requests[1].content)["messages"]
    assistant = next(message for message in messages if message["role"] == "assistant")
    tool_call = assistant["tool_calls"][0]
    assert tool_call["type"] == "function"
    assert tool_call["function"]["name"] == "find_object"
    assert json.loads(tool_call["function"]["arguments"]) == {"target": "a bottle of water"}
    result = next(message for message in messages if message["role"] == "tool")
    assert result["tool_call_id"] == tool_call["id"]
    assert result["content"] == "test observation"


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


@pytest.mark.parametrize("backend", ["kimi", "moonshot"])
def test_k3_vision_uses_low_effort_and_returns_only_final_content(transport, backend):
    transport.reasoning = "Considering candidates 1 and 3 before deciding"
    transport.text = "2"
    module = LLMModule(backend=backend, model="kimi-k3")
    client = module._create_client()
    assert client.supports_vision is True
    messages = image_messages()

    async def run():
        try:
            assert await client.chat(messages, temperature=0.0) == "2"
        finally:
            await client.close()

    asyncio.run(run())
    body = json.loads(transport.requests[0].content)
    assert body["model"] == "kimi-k3"
    assert body["messages"] == messages
    assert body["reasoning_effort"] == "low"
    assert body["max_completion_tokens"] == 4096
    assert "thinking" not in body
    assert "temperature" not in body
    assert "max_tokens" not in body


def test_k3_truncated_person_number_cannot_lock_a_target(transport):
    transport.text = "1"
    transport.finish_reason = "length"
    module = LLMModule(backend="kimi", model="kimi-k3")
    module.setup()
    servo = VisualServoModule()
    servo.on_system_modules({"LLMModule": module})
    servo._on_servo_target("follow:gray shirt and pink bag")
    try:
        servo._run_vlm_select_sync(
            servo._target_label, [np.zeros((32, 24, 3), dtype=np.uint8)],
            [{"id": "person-a", "position": [3.0, 0.0, 1.0]}],
            servo._intent_generation,
        )
        assert len(transport.requests) == 1
        assert not servo._person_tracker.target_selected
        assert servo._follow_select_method == "unavailable"
    finally:
        servo.stop()
        module.stop()


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


@pytest.mark.parametrize(
    "reply,expected_id,requests_per_selection",
    [("2", "person-b", 2), ("0", None, 1), ("api_error", None, 1)],
)
def test_follow_selection_reuses_the_host_api_loop_and_sends_numbered_crops(
    transport, reply, expected_id, requests_per_selection,
):
    loops = []
    transport.on_request = lambda: loops.append(asyncio.get_running_loop())
    module = LLMModule(backend="kimi", model="kimi-k3", timeout_sec=1.0)
    module.setup()
    servo = VisualServoModule()
    servo.on_system_modules({"LLMModule": module})
    persons = [
        {"id": "person-a", "label": "person", "position": [3.0, 0.0, 1.0]},
        {"id": "person-b", "label": "person", "position": [4.0, 1.0, 1.0]},
    ]
    crops = [np.zeros((32, 24, 3), dtype=np.uint8), np.full((32, 24, 3), 127, dtype=np.uint8)]
    goals = []
    servo.goal_pose._add_callback(goals.append)
    try:
        module.client.config.max_retries = 0
        # Warm the same SDK instance through its existing Host request loop.
        asyncio.run_coroutine_threadsafe(
            module.client.chat([{"role": "user", "content": "test"}]),
            module._loop,
        ).result(timeout=3.0)
        transport.text = reply
        if reply == "2":
            transport.texts = ["2", "MATCH", "2", "MATCH"]
        transport.reasoning = "Candidate 1 is not the requested person"
        transport.error = reply == "api_error"
        for _ in range(2):
            servo._on_servo_target("stop")
            servo._on_servo_target("follow:gray shirt and pink bag")
            servo._run_vlm_select_sync(
                servo._target_label, crops, persons, servo._intent_generation,
            )
            selected = servo._person_tracker.status()
            assert (selected["id"] if selected else None) == expected_id
        expected_requests = 1 + 2 * requests_per_selection
        assert len(transport.requests) == expected_requests
        assert loops == [module._loop] * expected_requests
        # A selection alone must wait for current map-frame detections to move.
        assert goals == []
        selection_requests = transport.requests[1::requests_per_selection]
        for request in selection_requests:
            body = json.loads(request.content)
            assert body["model"] == "kimi-k3"
            assert body["reasoning_effort"] == "low"
            assert "thinking" not in body
            assert "temperature" not in body
            content = body["messages"][0]["content"]
            assert "gray shirt and pink bag" in content[0]["text"]
            assert content[1]["text"] == "Person 1:"
            assert content[3]["text"] == "Person 2:"
            images = [part["image_url"]["url"] for part in content if part["type"] == "image_url"]
            assert len(images) == 2
            assert all(value.startswith("data:image/jpeg;base64,") for value in images)
            assert images[0] != images[1]
        if reply == "2":
            for request in transport.requests[2::2]:
                content = json.loads(request.content)["messages"][0]["content"]
                assert "Target description: gray shirt and pink bag" in content[0]["text"]
                assert any(part.get("text") == "Current selected candidate crop:" for part in content)
                assert len([part for part in content if part["type"] == "image_url"]) == 1
    finally:
        servo.stop()
        module.stop()


def test_follow_reselection_requires_pair_match_against_original_crop(transport):
    module = LLMModule(backend="kimi", model="kimi-k3", timeout_sec=1.0)
    module.setup()
    servo = VisualServoModule()
    servo.on_system_modules({"LLMModule": module})
    persons = [
        {"id": "person-a", "label": "person", "position": [3.0, 0.0, 1.0]},
        {"id": "person-b", "label": "person", "position": [4.0, 1.0, 1.0]},
    ]
    crops = [np.zeros((32, 24, 3), dtype=np.uint8), np.full((32, 24, 3), 127, dtype=np.uint8)]
    try:
        module.client.config.max_retries = 0
        servo._on_servo_target("follow:gray shirt and pink bag")
        transport.texts = ["1", "MATCH"]
        servo._run_vlm_select_sync(
            servo._target_label, crops, persons, servo._intent_generation,
        )
        assert servo._person_tracker.status()["id"] == "person-a"
        original_reference = servo._follow_reference_crop.copy()

        servo._person_tracker.reset()
        transport.texts = ["2", "OTHER"]
        request_start = len(transport.requests)
        servo._run_vlm_select_sync(
            servo._target_label, crops, persons, servo._intent_generation,
        )

        assert not servo._person_tracker.target_selected
        np.testing.assert_array_equal(servo._follow_reference_crop, original_reference)
        selection_content = json.loads(
            transport.requests[request_start].content
        )["messages"][0]["content"]
        assert selection_content[0]["text"].startswith("Reference image")
        assert selection_content[1]["type"] == "image_url"
        verification_content = json.loads(
            transport.requests[request_start + 1].content
        )["messages"][0]["content"]
        assert any(part.get("text") == "Original target crop:" for part in verification_content)
        assert len([part for part in verification_content if part["type"] == "image_url"]) == 2
    finally:
        servo.stop()
        module.stop()


def test_kimi_checks_the_description_even_with_one_confirmed_person(transport):
    transport.text = "0"
    module = LLMModule(backend="kimi", model="kimi-k3")
    module.setup()
    servo = VisualServoModule()
    servo.on_system_modules({"LLMModule": module})
    servo._latest_bgr = np.zeros((64, 64, 3), dtype=np.uint8)
    servo._on_servo_target("follow:purple jacket and yellow hat")
    done = threading.Event()
    select = servo._run_vlm_select_sync

    def finish(*args):
        try:
            select(*args)
        finally:
            done.set()

    servo._run_vlm_select_sync = finish
    try:
        servo._try_select_follow_target(
            [{"id": "person-a", "label": "person", "position": [3.0, 0.0, 1.0],
              "bbox": [0, 0, 32, 48]}],
            single_person_confirmed=True,
        )
        assert done.wait(3.0)
        assert len(transport.requests) == 1
        assert not servo._person_tracker.target_selected
        assert servo._follow_select_method == "unavailable"
    finally:
        servo.stop()
        module.stop()


def test_follow_does_not_advertise_a_vision_api_without_its_key(monkeypatch):
    monkeypatch.delenv("MOONSHOT_API_KEY", raising=False)
    module = LLMModule(backend="kimi")
    module.setup()
    try:
        servo = VisualServoModule()
        servo.on_system_modules({"LLMModule": module})
        assert not servo.can_select_follow_target()
    finally:
        module.stop()


@pytest.mark.parametrize("shutdown_host", [False, True])
def test_stop_discards_a_late_person_reply_and_host_shutdown_closes_it(transport, shutdown_host):
    reading, closed = threading.Event(), threading.Event()
    release = asyncio.Event()

    class PendingReply(httpx.AsyncByteStream):
        async def __aiter__(self):
            yield event({"reasoning_content": "waiting"})
            reading.set()
            await release.wait()
            yield event({"content": "1"}) + b"data: [DONE]\n\n"

        async def aclose(self):
            closed.set()

    transport.reply_stream = PendingReply()
    module = LLMModule(backend="kimi", model="kimi-k3")
    module.setup()
    servo = VisualServoModule()
    servo.on_system_modules({"LLMModule": module})
    servo._on_servo_target("follow:gray shirt and pink bag")
    worker = threading.Thread(target=servo._run_vlm_select_sync, args=(
        servo._target_label, [np.zeros((32, 24, 3), dtype=np.uint8)],
        [{"id": "person-a", "position": [3.0, 0.0, 1.0]}], servo._intent_generation,
    ))
    worker.start()
    try:
        assert reading.wait(3.0)
        servo._on_servo_target("stop")
        sdk = module.client._client
        if shutdown_host:
            module.stop()
        else:
            module._loop.call_soon_threadsafe(release.set)
            worker.join(timeout=3.0)
    finally:
        if module._loop is not None:
            module.stop()
        worker.join(timeout=3.0)
        servo.stop()
    assert not worker.is_alive()
    assert sdk.is_closed()
    assert closed.is_set()
    assert servo._mode == "idle"
    assert not servo._person_tracker.target_selected


def test_follow_api_deadline_releases_selection_and_closes_the_stream(transport):
    reading, closed = threading.Event(), threading.Event()

    class PendingReply(httpx.AsyncByteStream):
        async def __aiter__(self):
            yield event({"reasoning_content": "waiting"})
            reading.set()
            await asyncio.Event().wait()

        async def aclose(self):
            closed.set()

    transport.reply_stream = PendingReply()
    module = LLMModule(backend="kimi", model="kimi-k3", timeout_sec=0.2)
    module.setup()
    servo = VisualServoModule()
    servo.on_system_modules({"LLMModule": module})
    servo._on_servo_target("follow:gray shirt and pink bag")
    worker = threading.Thread(target=servo._run_vlm_select_sync, args=(
        servo._target_label, [np.zeros((32, 24, 3), dtype=np.uint8)],
        [{"id": "person-a", "position": [3.0, 0.0, 1.0]}], servo._intent_generation,
    ))
    worker.start()
    try:
        assert reading.wait(3.0)
        worker.join(timeout=3.0)
        assert not worker.is_alive()
        assert closed.is_set()
        assert module._loop.is_running()
        assert not servo._person_tracker.target_selected
        assert not servo._follow_select_pending
        assert servo._follow_select_method == "unavailable"
    finally:
        servo.stop()
        module.stop()
        worker.join(timeout=3.0)


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
