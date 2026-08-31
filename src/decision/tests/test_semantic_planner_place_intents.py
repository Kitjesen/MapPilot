"""Named-place navigation stays inside the active map."""

from __future__ import annotations

import json
from typing import Any

from decision.modules.semantic_planner import SemanticPlannerModule
from memory.spatial.places import PLACE_SCHEMA_VERSION


class _MapQuery:
    def __init__(self, active: str, maps: dict[str, dict[str, Any]]) -> None:
        self.active = active
        self.maps = maps

    def service(self, action: str, **arguments: Any) -> dict[str, Any]:
        map_id = str(arguments.get("map_id") or self.active)
        if action == "get_active_map":
            return {"success": True, "active": self.active}
        if action == "list_maps":
            return {"success": True, "maps": [{"map_id": item} for item in self.maps]}
        if action == "get_record":
            return {
                "success": True,
                "record": {"map_id": map_id, "content_epoch": 7, "state": "READY", "frame_id": "map"},
            }
        if action == "list_poi":
            return {"success": True, "pois": self.maps.get(map_id, {})}
        raise AssertionError(action)


def _poi(*, map_id: str, x: float = 3.0) -> dict[str, Any]:
    return {
        "x": x,
        "y": 4.0,
        "z": 0.2,
        "yaw": 1.0,
        "frame_id": "map",
        "tags": {
            "schema_version": PLACE_SCHEMA_VERSION,
            "place_id": "acme",
            "aliases": ["某公司"],
            "kind": "company",
            "building_id": "main",
            "floor_id": "floor-6",
            "content_epoch": 7,
            "source": "test",
            "confidence": 0.95,
        },
    }


def _planner(query: _MapQuery, *, navigation: bool = True) -> SemanticPlannerModule:
    module = SemanticPlannerModule(llm_backend="mock", map_query=query)
    module.on_system_modules({"nav.goals": object()} if navigation else {})
    return module


def _collect(port):
    values = []
    port._add_callback(values.append)
    return values


def test_named_place_dispatches_native_goal_on_active_map() -> None:
    module = _planner(_MapQuery("map-a", {"map-a": {"某公司": _poi(map_id="map-a")}}))
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("带我到某公司")

    payload = json.loads(commands[0])
    assert payload["action"] == "goto"
    assert (payload["x"], payload["y"], payload["z"], payload["yaw"]) == (3.0, 4.0, 0.2, 1.0)
    assert statuses[-1] == "PLACE_GOAL_DISPATCHED"


def test_named_place_rejects_cross_map_result() -> None:
    module = _planner(
        _MapQuery("map-a", {"map-a": {}, "map-b": {"某公司": _poi(map_id="map-b")}})
    )
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("带我到某公司")

    assert not commands
    assert statuses[-1] == "CROSS_MAP_NAVIGATION_UNSUPPORTED"


def test_named_place_requires_navigation_service() -> None:
    module = _planner(
        _MapQuery("map-a", {"map-a": {"某公司": _poi(map_id="map-a")}}),
        navigation=False,
    )
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("带我到某公司")

    assert not commands
    assert statuses[-1] == "NAVIGATION_SERVICE_REQUIRED"
