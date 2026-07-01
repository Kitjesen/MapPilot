from decision.tasking.agent_loop import AgentLoop


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
