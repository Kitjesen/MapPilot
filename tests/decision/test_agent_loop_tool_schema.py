import asyncio

from decision.tasks.agent import AgentLoop


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
