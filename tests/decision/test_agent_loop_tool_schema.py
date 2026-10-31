import asyncio
import json

from decision.tasks.agent import AGENT_TOOLS, AgentLoop, AgentState, skills_to_openai_tools


class _DummyLLM:
    async def chat(self, *args, **kwargs):
        return {"content": ""}


def _context():
    return {
        "robot_x": 0.0,
        "robot_y": 0.0,
        "visible_objects": "none",
        "nav_status": "IDLE",
        "memory_context": "none",
        "camera_image": None,
        "camera_available": False,
    }


def test_query_memory_schema_accepts_text_argument():
    loop = AgentLoop(
        llm_client=_DummyLLM(),
        tool_registry={},
        tool_list=[],
        context_fn=_context,
        tool_handlers={"query_memory": lambda text: text},
    )

    assert loop._validate_tool_call("query_memory", {"text": "backpack"}) is None


def test_model_request_has_its_own_total_deadline_and_is_cancelled():
    class SlowModel:
        cancelled = False

        async def chat_with_tools(self, messages, tools=None):
            try:
                await asyncio.sleep(10)
            finally:
                self.cancelled = True

    model = SlowModel()
    loop = AgentLoop(
        llm_client=model, tool_registry={}, tool_list=[], context_fn=_context,
        timeout=2, llm_timeout=0.02,
    )
    state = asyncio.run(loop.run("find a bottle of water"))
    assert state.completed and state.failure_reason == "llm_timeout"
    assert "0.02s" in state.summary
    assert model.cancelled
    assert not loop._tool_call_audit


def test_task_deadline_can_expire_before_the_model_request_deadline():
    class SlowModel:
        async def chat_with_tools(self, messages, tools=None):
            await asyncio.sleep(10)

    loop = AgentLoop(
        llm_client=SlowModel(), tool_registry={}, tool_list=[], context_fn=_context,
        timeout=0.02, llm_timeout=2,
    )
    state = asyncio.run(loop.run("find a bottle of water"))
    assert state.completed and state.failure_reason == "timeout"
    assert not loop._tool_call_audit


def test_sustained_follow_wait_is_ended_by_cancellation_not_task_deadline():
    started = asyncio.Event()

    async def follow_person(description):
        started.set()
        await asyncio.Event().wait()

    loop = AgentLoop(
        llm_client=_ToolCallLLM([
            {
                "function": {
                    "name": "follow_person",
                    "arguments": '{"description":"person"}',
                },
                "id": "follow",
            }
        ]),
        tool_registry={},
        tool_list=[],
        tool_handlers={"follow_person": follow_person},
        context_fn=_context,
        timeout=0.02,
        llm_timeout=1.0,
    )

    async def exercise():
        task = asyncio.create_task(loop.run("follow me"))
        await asyncio.wait_for(started.wait(), timeout=1.0)
        await asyncio.sleep(0.05)
        assert task.done() is False
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            return
        raise AssertionError("Follow Agent task ignored explicit cancellation")

    asyncio.run(exercise())


def test_done_requires_explicit_boolean_outcome():
    loop = _agent_loop()
    assert loop._validate_tool_call("done", {"summary": "not found"}) is not None
    assert loop._validate_tool_call("done", {"summary": "not found", "success": "false"}) is not None
    assert loop._validate_tool_call("done", {"summary": "not found", "success": False}) is None


class _ToolCallLLM:
    def __init__(self, tool_calls):
        self._tool_calls = iter(tool_calls)

    async def chat_with_tools(self, messages, tools=None):
        call = next(
            self._tool_calls,
            {"function": {"name": "done", "arguments": '{"summary": "end", "success": true}'}, "id": "done"},
        )
        return {"tool_calls": [call]}


def _agent_loop(*tool_calls):
    return AgentLoop(
        llm_client=_ToolCallLLM(tool_calls),
        tool_registry={},
        tool_list=[],
        context_fn=_context,
        max_steps=5,
        timeout=30.0,
    )


def test_unknown_tool_is_audited_and_fed_back_to_the_llm():
    loop = _agent_loop(
        {"function": {"name": "fly_to_moon", "arguments": "{}"}, "id": "bad"},
        {"function": {"name": "done", "arguments": '{"summary": "recovered", "success": true}'}, "id": "done"},
    )
    state = asyncio.run(loop.run("go somewhere"))

    rejected = [entry for entry in loop._tool_call_audit if entry["tool_name"] == "fly_to_moon"]
    assert rejected
    assert "VALIDATION_ERROR" in rejected[0]["result_summary"]
    assert any(
        message.get("role") == "tool" and "fly_to_moon" in message.get("content", "")
        for message in state.messages
    )


def test_invalid_tool_arguments_are_audited():
    loop = _agent_loop(
        {"function": {"name": "navigate_to", "arguments": "{}"}, "id": "bad"},
        {"function": {"name": "done", "arguments": '{"summary": "ok", "success": true}'}, "id": "done"},
    )
    asyncio.run(loop.run("navigate"))

    rejected = [entry for entry in loop._tool_call_audit if entry["tool_name"] == "navigate_to"]
    assert rejected
    assert "VALIDATION_ERROR" in rejected[0]["result_summary"]


def test_invalid_calls_count_toward_max_steps():
    calls = [
        {"function": {"name": "nonexistent_tool", "arguments": "{}"}, "id": f"bad-{index}"}
        for index in range(10)
    ]
    loop = _agent_loop(*calls)
    state = asyncio.run(loop.run("loop forever"))

    assert state.step <= loop._max_steps
    assert loop._tool_call_audit


def test_discovered_handler_and_schema_take_precedence_together():
    calls = []
    loop = AgentLoop(
        llm_client=_ToolCallLLM([]),
        tool_registry={"query_memory": lambda query: calls.append(query) or "found"},
        tool_list=[{"name": "query_memory", "description": "Query memory", "inputSchema": {
            "type": "object", "properties": {"query": {"type": "string"}}, "required": ["query"],
        }}],
        context_fn=_context,
        tool_handlers={"query_memory": lambda text: "wrong handler"},
    )
    descriptor = next(tool["function"] for tool in loop._tools if tool["function"]["name"] == "query_memory")
    assert descriptor["parameters"]["required"] == ["query"]
    assert loop._validate_tool_call("query_memory", {"query": "door"}) is None
    assert loop._validate_tool_call("query_memory", {"text": "door"}) is not None
    from decision.tasks.agent import AgentState
    asyncio.run(loop._execute_tool({"function": {"name": "query_memory", "arguments": '{"query":"door"}'},
                                   "id": "query"}, AgentState()))
    assert calls == ["door"]


def test_skill_metadata_is_visible_to_openai_tool_callers():
    tools = skills_to_openai_tools([
        {
            "name": "inspect_task",
            "description": "Read retained task state.",
            "inputSchema": {
                "type": "object",
                "properties": {"task_id": {"type": "string"}},
                "required": ["task_id"],
            },
            "outputSchema": {
                "type": "object",
                "properties": {"found": {"type": "boolean"}},
                "required": ["found"],
            },
            "annotations": {"readOnlyHint": True},
        }
    ])

    function = tools[0]["function"]
    assert function["parameters"]["required"] == ["task_id"]
    assert 'Returns JSON matching: {"type":"object"' in function["description"]
    assert "Read-only: does not change robot or task state." in function["description"]


def test_mutating_skill_keeps_motion_authority_warning():
    tools = skills_to_openai_tools([
        {
            "name": "start_task",
            "description": "Start a task.",
            "inputSchema": {"type": "object", "properties": {}},
            "annotations": {"readOnlyHint": False},
        }
    ])

    description = tools[0]["function"]["description"]
    assert "Changes runtime state" in description
    assert "task ownership and motion gates still apply" in description


def test_owned_follow_tools_state_real_completion_and_cancel_scope():
    descriptions = {
        tool["function"]["name"]: tool["function"]["description"]
        for tool in AGENT_TOOLS
    }

    assert "wait for its real terminal result" in descriptions["follow_person"]
    assert "only this Agent run's owned motion" in descriptions["stop_servo"]
    assert "not that physical motion has stopped" in descriptions["stop_servo"]


def test_structured_tool_results_are_valid_json_for_the_model():
    for payload in ({"状态": "正常", "found": True}, ["水瓶", "桌子"]):
        loop = AgentLoop(
            llm_client=_ToolCallLLM([]),
            tool_registry={"inspect_task": lambda value=payload: value},
            tool_list=[{
                "name": "inspect_task",
                "description": "Inspect task.",
                "inputSchema": {"type": "object", "properties": {}},
            }],
            context_fn=_context,
        )
        state = AgentState()

        result = asyncio.run(loop._execute_tool(
            {"function": {"name": "inspect_task", "arguments": "{}"}, "id": "inspect"},
            state,
        ))

        assert json.loads(result) == payload
        tool_message = next(message for message in state.messages if message.get("role") == "tool")
        assert tool_message["content"] == result
