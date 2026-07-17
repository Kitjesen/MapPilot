from __future__ import annotations

import json

from runtime.module import Module, skill


class StructuredSkills(Module):
    @skill
    def start_route(
        self,
        waypoints: list[dict[str, float]],
        loop: bool = False,
        labels: list[str] | None = None,
    ) -> str:
        """Start a structured waypoint route."""

        del waypoints, loop, labels
        return "ok"


def test_skill_schema_preserves_arrays_objects_and_optional_types() -> None:
    module = StructuredSkills()
    info = next(item for item in module.get_skill_infos() if item.func_name == "start_route")
    schema = json.loads(info.args_schema)

    waypoints = schema["properties"]["waypoints"]
    assert waypoints["type"] == "array"
    assert waypoints["items"]["type"] == "object"
    assert waypoints["items"]["additionalProperties"]["type"] == "number"
    assert schema["properties"]["loop"] == {"type": "boolean", "default": False}
    assert schema["properties"]["labels"]["type"] == "array"
    assert schema["properties"]["labels"]["items"]["type"] == "string"
    assert schema["properties"]["labels"]["default"] is None
    assert schema["required"] == ["waypoints"]
