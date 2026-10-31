import json

import pytest

from runtime.module import Module, skill
from runtime.skills import discover_skills


def test_bare_skill_remains_compatible():
    class M(Module):
        @skill
        def ping(self) -> str:
            """Ping."""
            return "pong"
    info = M().get_skill_infos()[0]
    assert json.loads(info.args_schema)["description"] == "Ping."


def test_rich_metadata_and_deepcopy_are_exported():
    params = {"target": "Target label"}
    output = {"type": "object", "properties": {"task_id": {"type": "string"}}}
    class M(Module):
        @skill(description="Start a target task", parameters=params,
               output_schema=output, read_only=False)
        def start(self, target: str) -> dict:
            return {}
    tool = M().get_skill_infos()[0].to_tool()
    assert tool["inputSchema"]["properties"]["target"]["description"] == "Target label"
    assert tool["outputSchema"]["type"] == "object"
    assert tool["annotations"] == {"readOnlyHint": False}
    params["target"] = "changed"
    output["properties"]["task_id"]["type"] = "integer"
    fresh = M().get_skill_infos()[0].to_tool()
    assert fresh["inputSchema"]["properties"]["target"]["description"] == "Target label"
    assert fresh["outputSchema"]["properties"]["task_id"]["type"] == "string"
    tool["inputSchema"]["properties"]["target"]["description"] = "changed export"
    assert M().get_skill_infos()[0].to_tool()["inputSchema"]["properties"]["target"]["description"] == "Target label"


def test_unknown_parameter_description_is_author_error():
    class M(Module):
        @skill(parameters={"missing": "bad"})
        def start(self, target: str) -> str:
            return ""
    with pytest.raises(ValueError, match="missing"):
        M().get_skill_infos()


def test_discovery_aliases_collisions_and_explicit_fallback():
    class M(Module):
        @skill
        def ping(self) -> str:
            return ""
    m = M()
    registry, tools = discover_skills({"a": m, "alias": m})
    assert list(registry) == ["ping"] and len(tools) == 1
    with pytest.raises(ValueError, match="duplicate skill 'ping'"):
        discover_skills({"one": M(), "two": M()})
    registry, _ = discover_skills({"one": M(), "two": M()}, exclude={"ping"})
    assert registry == {}
    fallback, peer = M(), M()
    registry, _ = discover_skills({"fallback": fallback, "peer": peer},
                                  fallback_owner=fallback, fallback_names={"ping"})
    assert registry["ping"].__self__ is peer


def test_invalid_skill_metadata_is_rejected():
    with pytest.raises(TypeError):
        skill(read_only="yes")
    with pytest.raises(ValueError):
        skill(output_schema={"properties": {}})
    with pytest.raises(TypeError):
        skill(parameters={"target": 1})
