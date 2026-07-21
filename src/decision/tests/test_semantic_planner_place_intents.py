"""SemanticPlanner place-intent grounding tests."""

from __future__ import annotations

import json
import math
import time
from typing import Any

import pytest

from decision.modules.llm import LLMResponse
from decision.modules.semantic_planner import SemanticPlannerModule
from maps.places import PLACE_SCHEMA_VERSION


def _collect(port_out):
    items = []
    port_out._add_callback(items.append)
    return items


class _FakeMapsModule:
    def __init__(self, *, active_map: str = "map-a", pois: dict[str, dict[str, Any]] | None = None) -> None:
        self._active_map = active_map
        self.api = _FakeMapsApi(active_map=active_map, pois=pois or {})


class _FakeMapsApi:
    def __init__(self, *, active_map: str, pois: dict[str, dict[str, Any]]) -> None:
        self._active_map = active_map
        self._pois = pois

    def list_maps(self) -> dict[str, Any]:
        return {
            "success": True,
            "maps": [{"map_id": map_id} for map_id in sorted(self._pois)],
        }

    def get_record(self, map_id: str) -> dict[str, Any]:
        return {
            "success": True,
            "record": {
                "map_id": map_id,
                "version": 7,
                "state": "READY",
                "version_id": f"{map_id}:v7",
                "map_pcd_sha256": f"hash-{map_id}",
                "frame_id": "map",
            },
        }

    def get_map_points(self, map_id: str, *, max_points: int = 0) -> dict[str, Any]:
        return {
            "success": True,
            "map_id": map_id,
            "version_id": f"{map_id}:v7",
            "map_pcd_sha256": f"hash-{map_id}",
            "frame_id": "map",
            "points": [],
        }

    def get_active_map(self) -> dict[str, Any]:
        return {"success": True, "active": self._active_map}

    def poi_list(self, map_id: str = "") -> dict[str, Any]:
        return {"success": True, "pois": dict(self._pois.get(map_id or self._active_map, {}))}


def _place_poi(
    *,
    x: float = 1.0,
    y: float = 2.0,
    z: float = 0.3,
    yaw: float | None = 1.2,
    floor_id: str = "floor-6",
    map_id: str = "map-a",
    place_id: str = "acme",
    aliases: tuple[str, ...] = ("某公司",),
    connector_id: str = "",
    version_id: str | None = None,
    map_hash: str | None = None,
) -> dict[str, Any]:
    return {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "frame_id": "map",
        "tags": {
            "schema_version": PLACE_SCHEMA_VERSION,
            "place_id": place_id,
            "aliases": list(aliases),
            "kind": "company",
            "building_id": "main",
            "floor_id": floor_id,
            "connector_id": connector_id,
            "map_version": 7,
            "version_id": version_id or f"{map_id}:v7",
            "map_pcd_sha256": map_hash or f"hash-{map_id}",
            "source": "test",
            "confidence": 0.95,
        },
    }


def _planner_with_maps(
    maps_module: _FakeMapsModule,
    *,
    nav_goals: bool = False,
    nav_building: bool = False,
) -> SemanticPlannerModule:
    module = SemanticPlannerModule(llm_backend="mock")
    modules: dict[str, object] = {"maps.service": maps_module}
    if nav_goals:
        modules["nav.goals"] = object()
    if nav_building:
        modules["nav.building"] = object()
    module.on_system_modules(modules)
    return module


def test_place_intent_dispatches_active_map_building_mission_without_scene_graph() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(
            pois={
                "map-a": {
                    "某公司": _place_poi(x=3.0, y=4.0, z=0.2, yaw=math.pi / 2),
                }
            }
        ),
        nav_goals=True,
        nav_building=True,
    )
    goals = _collect(module.goal_pose)
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("带我到6楼某公司")

    assert statuses[-1] == "BUILDING_MISSION_DISPATCHED"
    assert not goals
    assert len(commands) == 1
    payload = json.loads(commands[0])
    assert payload["travel_mode"] == "any"
    assert payload["target"]["map_id"] == "map-a"
    assert payload["target"]["x"] == 3.0
    assert payload["target"]["y"] == 4.0
    assert payload["target"]["z"] == 0.2
    assert math.isclose(payload["target"]["yaw"], math.pi / 2)


@pytest.mark.parametrize(
    ("nav_goals", "nav_building"),
    [(False, False), (True, False), (False, True)],
)
def test_active_map_place_fails_closed_without_complete_building_runtime(
    nav_goals: bool,
    nav_building: bool,
) -> None:
    module = _planner_with_maps(
        _FakeMapsModule(pois={"map-a": {"某公司": _place_poi()}}),
        nav_goals=nav_goals,
        nav_building=nav_building,
    )
    goals = _collect(module.goal_pose)
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("带我到6楼某公司")

    assert not goals
    assert not commands
    assert statuses[-1] == "CONNECTOR_RUNTIME_REQUIRED"


def test_place_intent_prefers_active_map_when_alias_exists_on_multiple_floors() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(
            active_map="map-a",
            pois={
                "map-a": {"某公司一层": _place_poi(place_id="active", x=3.0)},
                "map-b": {
                    "某公司六层": _place_poi(
                        map_id="map-b",
                        place_id="remote",
                        x=30.0,
                    )
                },
            },
        ),
        nav_goals=True,
        nav_building=True,
    )
    goals = _collect(module.goal_pose)
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("带我到6楼某公司")

    assert not goals
    assert statuses[-1] == "BUILDING_MISSION_DISPATCHED"
    assert len(commands) == 1
    payload = json.loads(commands[0])
    assert payload["target"]["place_id"] == "active"
    assert payload["target"]["map_id"] == "map-a"
    assert payload["target"]["x"] == 3.0


def test_place_intent_fails_closed_for_cross_map_without_building_runtime() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(
            active_map="map-a",
            pois={
                "map-a": {},
                "map-b": {"某公司": _place_poi(map_id="map-b")},
            },
        )
    )
    goals = _collect(module.goal_pose)
    statuses = _collect(module.planner_status)

    module._on_instruction("去六层某公司")

    assert not goals
    assert statuses[-1] == "CONNECTOR_RUNTIME_REQUIRED"


def test_place_intent_dispatches_cross_map_building_mission() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(
            active_map="map-a",
            pois={
                "map-a": {},
                "map-b": {
                    "某公司": _place_poi(
                        map_id="map-b",
                        connector_id="lift-east",
                        x=8.0,
                        y=9.0,
                        z=0.4,
                        yaw=0.6,
                    )
                },
            },
        ),
        nav_goals=True,
        nav_building=True,
    )
    goals = _collect(module.goal_pose)
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("去六层某公司")

    assert not goals
    assert statuses[-1] == "BUILDING_MISSION_DISPATCHED"
    assert len(commands) == 1
    payload = json.loads(commands[0])
    assert payload["action"] == "building_navigate"
    assert payload["schema_version"] == "lingtu.building_goal.v1"
    assert payload["request_id"].startswith("semantic-")
    assert payload["source"] == "semantic"
    assert payload["travel_mode"] == "any"
    assert payload["connector_id"] == "lift-east"
    assert payload["target"] == {
        "place_id": "acme",
        "name": "某公司",
        "building_id": "main",
        "floor_id": "floor-6",
        "map_id": "map-b",
        "frame_id": "map",
        "x": 8.0,
        "y": 9.0,
        "z": 0.4,
        "yaw": 0.6,
        "map_version": 7,
        "version_id": "map-b:v7",
        "map_pcd_sha256": "hash-map-b",
    }


def test_place_intent_refuses_stale_place() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(
            pois={
                "map-a": {
                    "某公司": _place_poi(version_id="map-a:old", map_hash="old-hash"),
                }
            }
        )
    )
    goals = _collect(module.goal_pose)
    statuses = _collect(module.planner_status)

    module._on_instruction("去6楼某公司")

    assert not goals
    assert statuses[-1] == "PLACE_STALE_MAP"


def test_place_intent_refuses_ambiguous_place() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(
            pois={
                "map-a": {
                    "某公司A": _place_poi(place_id="a", aliases=("某公司",), x=1.0),
                    "某公司B": _place_poi(place_id="b", aliases=("某公司",), x=2.0),
                }
            }
        )
    )
    goals = _collect(module.goal_pose)
    statuses = _collect(module.planner_status)

    module._on_instruction("去6楼某公司")

    assert not goals
    assert statuses[-1] == "PLACE_AMBIGUOUS"


@pytest.mark.parametrize("instruction", ["走楼梯去6楼某公司", "坐电梯去6楼某公司"])
@pytest.mark.parametrize(
    ("nav_goals", "nav_building"),
    [(False, False), (True, False), (False, True)],
)
def test_explicit_connector_mode_fails_closed_until_building_runtime_exists(
    instruction: str,
    nav_goals: bool,
    nav_building: bool,
) -> None:
    module = _planner_with_maps(
        _FakeMapsModule(pois={"map-a": {"某公司": _place_poi()}}),
        nav_goals=nav_goals,
        nav_building=nav_building,
    )
    goals = _collect(module.goal_pose)
    statuses = _collect(module.planner_status)

    module._on_instruction(instruction)

    assert not goals
    assert statuses[-1] == "CONNECTOR_RUNTIME_REQUIRED"


@pytest.mark.parametrize(
    ("instruction", "travel_mode", "connector_id"),
    [
        ("走楼梯去6楼某公司", "stairs", "stairs-west"),
        ("坐电梯去6楼某公司", "elevator", "lift-east"),
    ],
)
def test_explicit_connector_mode_dispatches_building_mission_when_available(
    instruction: str,
    travel_mode: str,
    connector_id: str,
) -> None:
    module = _planner_with_maps(
        _FakeMapsModule(
            pois={
                "map-a": {
                    "某公司": _place_poi(connector_id=connector_id),
                }
            }
        ),
        nav_goals=True,
        nav_building=True,
    )
    goals = _collect(module.goal_pose)
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction(instruction)

    assert not goals
    assert statuses[-1] == "BUILDING_MISSION_DISPATCHED"
    assert len(commands) == 1
    payload = json.loads(commands[0])
    assert payload["action"] == "building_navigate"
    assert payload["travel_mode"] == travel_mode
    assert payload["connector_id"] == connector_id
    assert payload["target"]["map_id"] == "map-a"
    assert payload["target"]["floor_id"] == "floor-6"


def test_unrelated_instruction_still_uses_existing_scene_graph_path() -> None:
    module = _planner_with_maps(_FakeMapsModule(pois={"map-a": {}}))
    calls = []
    module._latest_sg = "{}"
    module._try_resolve = lambda instruction, sg_json: calls.append((instruction, sg_json))
    statuses = _collect(module.planner_status)

    module._on_instruction("find the red chair")

    assert calls == [("find the red chair", "{}")]
    assert statuses[0] == "PROCESSING"


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("开始展厅导览A", {"action": "inspection", "route_id": "展厅导览A"}),
        ("暂停导览", {"action": "inspection_pause", "reason": "semantic_pause"}),
        ("继续", {"action": "inspection_resume", "reason": "semantic_resume"}),
        ("取消导览", {"action": "inspection_cancel", "reason": "semantic_cancel"}),
    ],
)
def test_tour_command_dispatches_symbolic_native_inspection_command(
    text: str,
    expected: dict[str, str],
) -> None:
    module = _planner_with_maps(_FakeMapsModule(pois={"map-a": {}}), nav_goals=True)
    goals = _collect(module.goal_pose)
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction(text)

    time.sleep(0.01)
    assert not goals
    assert [json.loads(command) for command in commands] == [expected]
    assert statuses[-1] == "TOUR_COMMAND_DISPATCHED"


def test_tour_command_fails_closed_without_goal_service() -> None:
    module = _planner_with_maps(_FakeMapsModule(pois={"map-a": {}}))
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("暂停导览")

    assert commands == []
    assert statuses[-1] == "TOUR_COMMAND_UNAVAILABLE"


def test_rule_navigation_intent_does_not_emit_symbolic_llm_request() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(pois={"map-a": {"某公司": _place_poi()}}),
        nav_goals=True,
        nav_building=True,
    )
    requests = _collect(module.llm_request)

    module._on_instruction("坐电梯去6楼某公司")

    assert requests == []


def test_weak_navigation_phrase_emits_symbolic_llm_request_without_legacy_goal() -> None:
    module = _planner_with_maps(_FakeMapsModule(pois={"map-a": {"某公司": _place_poi()}}))
    requests = _collect(module.llm_request)
    goals = _collect(module.goal_pose)
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)
    module._latest_sg = "{}"
    module._try_resolve = lambda *_args: pytest.fail("weak symbolic request must not use legacy resolver")

    module._on_instruction("劳驾领路去那家六楼的公司")

    assert len(requests) == 1
    assert requests[0].caller == "SemanticPlannerModule.symbolic_intent"
    assert requests[0].temperature == 0.0
    assert requests[0].request_id.startswith("semantic-intent-")
    assert "Never include coordinates" in requests[0].messages[1]["content"]
    assert not goals
    assert not commands
    assert statuses[-1] == "SYMBOLIC_LLM_PENDING"


def test_symbolic_llm_response_with_coordinates_is_rejected_fail_closed() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(pois={"map-a": {"某公司": _place_poi()}}),
        nav_goals=True,
        nav_building=True,
    )
    requests = _collect(module.llm_request)
    goals = _collect(module.goal_pose)
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("劳驾领路去那家六楼的公司")
    module._on_llm_response(
        LLMResponse(
            text='```json\n{"action":"navigate","target_query":"某公司","floor_id":"六楼","x":1.0}\n```',
            request_id=requests[0].request_id,
        )
    )

    assert not goals
    assert not commands
    assert statuses[-1] == "SYMBOLIC_LLM_REJECTED"


def test_symbolic_llm_bad_json_is_rejected_fail_closed() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(pois={"map-a": {"某公司": _place_poi()}}),
        nav_goals=True,
        nav_building=True,
    )
    requests = _collect(module.llm_request)
    goals = _collect(module.goal_pose)
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("劳驾领路去那家六楼的公司")
    module._on_llm_response(
        LLMResponse(
            text='{"action":"navigate","target_query":"某公司"',
            request_id=requests[0].request_id,
        )
    )

    assert not goals
    assert not commands
    assert statuses[-1] == "SYMBOLIC_LLM_REJECTED"


def test_symbolic_llm_navigate_response_dispatches_building_command() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(pois={"map-a": {"某公司": _place_poi(x=6.0, y=7.0, connector_id="lift-east")}}),
        nav_goals=True,
        nav_building=True,
    )
    requests = _collect(module.llm_request)
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("劳驾领路去那家六楼的公司")
    module._on_llm_response(
        LLMResponse(
            text='{"action":"navigate","target_query":"某公司","floor_id":"六楼","travel_mode":"elevator","confidence":0.82}',
            request_id=requests[0].request_id,
        )
    )

    assert statuses[-1] == "BUILDING_MISSION_DISPATCHED"
    assert len(commands) == 1
    payload = json.loads(commands[0])
    assert payload["action"] == "building_navigate"
    assert payload["travel_mode"] == "elevator"
    assert payload["connector_id"] == "lift-east"
    assert payload["target"]["x"] == 6.0
    assert payload["target"]["y"] == 7.0


def test_symbolic_llm_clarification_response_does_not_execute() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(pois={"map-a": {"某公司": _place_poi()}}),
        nav_goals=True,
        nav_building=True,
    )
    requests = _collect(module.llm_request)
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("想去六楼看看")
    module._on_llm_response(
        LLMResponse(
            text='{"action":"navigate","target_query":"","needs_clarification":true,"confidence":0.6,"reason":"place_required"}',
            request_id=requests[0].request_id,
        )
    )

    assert not commands
    assert statuses[-1] == "PLACE_CLARIFICATION_REQUIRED"


def test_single_word_movement_object_navigation_stays_on_legacy_path() -> None:
    module = _planner_with_maps(_FakeMapsModule(pois={"map-a": {}}))
    requests = _collect(module.llm_request)
    calls = []
    module._latest_sg = "{}"
    module._try_resolve = lambda instruction, sg_json: calls.append((instruction, sg_json))

    module._on_instruction("去找红色椅子")

    assert requests == []
    assert calls == [("去找红色椅子", "{}")]


def test_symbolic_llm_ungrounded_place_falls_back_to_legacy_resolver() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(pois={"map-a": {}}),
        nav_goals=True,
        nav_building=True,
    )
    requests = _collect(module.llm_request)
    commands = _collect(module.nav_command)
    calls = []
    module._latest_sg = "{}"
    module._try_resolve = lambda instruction, sg_json: calls.append((instruction, sg_json))

    module._on_instruction("劳驾领路去六楼那个红椅子")
    module._on_llm_response(
        LLMResponse(
            text='{"action":"navigate","target_query":"红椅子","travel_mode":"any","confidence":0.77}',
            request_id=requests[0].request_id,
        )
    )

    assert not commands
    assert calls == [("劳驾领路去六楼那个红椅子", "{}")]


def test_symbolic_llm_epoch_guard_drops_response_if_new_instruction_arrives_before_publish() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(pois={"map-a": {"某公司": _place_poi(x=6.0, y=7.0)}}),
        nav_goals=True,
        nav_building=True,
    )
    requests = _collect(module.llm_request)
    commands = _collect(module.nav_command)
    calls = []
    module._latest_sg = "{}"
    module._try_resolve = lambda instruction, sg_json: calls.append((instruction, sg_json))
    original_parser = module._parse_symbolic_llm_json

    def parse_and_interrupt(text: str) -> dict[str, Any] | None:
        payload = original_parser(text)
        module._on_instruction("find the red chair")
        return payload

    module._parse_symbolic_llm_json = parse_and_interrupt
    module._on_instruction("劳驾领路去那家六楼的公司")
    module._on_llm_response(
        LLMResponse(
            text='{"action":"navigate","target_query":"某公司","floor_id":"六楼","travel_mode":"any","confidence":0.8}',
            request_id=requests[0].request_id,
        )
    )

    assert not commands
    assert calls == [("find the red chair", "{}")]


def test_symbolic_llm_drops_stale_response_after_new_instruction() -> None:
    module = _planner_with_maps(
        _FakeMapsModule(pois={"map-a": {"某公司": _place_poi(x=9.0)}}),
        nav_goals=True,
        nav_building=True,
    )
    requests = _collect(module.llm_request)
    commands = _collect(module.nav_command)
    statuses = _collect(module.planner_status)

    module._on_instruction("劳驾领路去那家六楼的公司")
    stale_request_id = requests[0].request_id
    module._on_instruction("麻烦领路去六楼某公司那边")
    current_request_id = requests[1].request_id

    legal = '{"action":"navigate","target_query":"某公司","floor_id":"六楼","travel_mode":"any","confidence":0.8}'
    module._on_llm_response(LLMResponse(text=legal, request_id=stale_request_id))
    assert not commands
    assert statuses[-1] == "SYMBOLIC_LLM_PENDING"

    module._on_llm_response(LLMResponse(text=legal, request_id=current_request_id))
    assert len(commands) == 1
    assert statuses[-1] == "BUILDING_MISSION_DISPATCHED"
