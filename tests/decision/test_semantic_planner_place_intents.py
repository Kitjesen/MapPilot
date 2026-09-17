"""Named-place navigation stays inside the active map."""

from __future__ import annotations

import json
import threading
from types import SimpleNamespace
from typing import Any

import pytest

from decision.modules.llm import LLMResponse
from decision.modules.semantic_planner import SemanticPlannerModule
from memory.spatial.places import PLACE_SCHEMA_VERSION
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.semantic import SceneGraph


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


def test_owned_place_refusal_is_terminal_and_cleared_by_new_instruction() -> None:
    module = _planner(
        _MapQuery("map-a", {"map-a": {}, "map-b": {"某公司": _poi(map_id="map-b")}})
    )
    result = module.submit_owned_instruction("带我到某公司", "agent-owner")
    assert result["owned"] is True
    assert result["terminal"] is True
    assert result["success"] is False
    assert result["state"] == "CROSS_MAP_NAVIGATION_UNSUPPORTED"

    module._map_query.maps["map-a"] = {"某公司": _poi(map_id="map-a")}
    result = module.submit_owned_instruction("带我到某公司", "agent-owner")
    assert result["terminal"] is False
    assert result["state"] == "dispatching"


def test_owned_floor_only_request_finishes_with_clarification() -> None:
    module = _planner(_MapQuery("map-a", {"map-a": {}}))
    result = module.submit_owned_instruction("去六楼", "agent-owner")
    assert result["terminal"] is True
    assert result["success"] is False
    assert result["state"] == "PLACE_CLARIFICATION_REQUIRED"


@pytest.mark.parametrize("lookup_error", [False, True])
def test_late_place_lookup_cannot_fail_replacement_instruction(lookup_error) -> None:
    module = _planner(_MapQuery("map-a", {"map-a": {}}))
    entered, release = threading.Event(), threading.Event()

    def resolve(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        if lookup_error:
            raise RuntimeError("map endpoint unavailable")
        return SimpleNamespace(status="stale_map", place=None)

    module._place_catalog = SimpleNamespace(resolve=resolve)
    statuses = _collect(module.planner_status)
    module._replace_instruction("old-owner")
    request_id = module._register_symbolic_llm_request("去六楼某公司")
    response = LLMResponse(request_id=request_id, text=json.dumps({
        "action": "navigate", "target_query": "某公司", "floor_id": "六楼", "travel_mode": "stairs",
    }))
    worker = threading.Thread(target=module._on_llm_response, args=(response,))
    worker.start()
    try:
        assert entered.wait(1)
        before = module.submit_owned_pose(
            PoseStamped(Pose(Vector3(1, 2, 0.3)), frame_id="map"), "new target", "new-owner",
        )
        status_count = len(statuses)
        release.set()
        worker.join(1)
        assert not worker.is_alive()
        assert module.owned_instruction_status("new-owner") == before
        assert len(statuses) == status_count
    finally:
        release.set()
        worker.join(2)


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


@pytest.mark.parametrize("same_map", [True, False])
def test_handled_place_intent_is_not_reinterpreted_on_scene_update(same_map) -> None:
    from unittest.mock import Mock

    place_map = "map-a" if same_map else "map-b"
    module = _planner(_MapQuery("map-a", {"map-a": {}, place_map: {"某公司": _poi(map_id=place_map)}}))
    module._goal_resolver = Mock()
    module._try_resolve = Mock()
    module._on_instruction("带我到某公司")

    module._on_scene_graph(SceneGraph(objects=[], regions=[]))

    module._try_resolve.assert_not_called()
