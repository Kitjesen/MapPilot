"""Public task-decomposition entrypoint stays before semantic resolution."""

from __future__ import annotations

import json
from types import SimpleNamespace
from unittest.mock import Mock

from decision.goals.resolver import GoalResolver
from decision.modules.semantic_planner import SemanticPlannerModule
from decision.tasks.decomposition import TaskDecomposer
from runtime.msgs.geometry import Vector3
from runtime.msgs.semantic import Detection3D, Relation, SceneGraph


def _planner() -> SemanticPlannerModule:
    planner = SemanticPlannerModule(llm_backend="mock", map_query=SimpleNamespace())
    planner._task_decomposer = TaskDecomposer()
    return planner


def test_decompose_task_returns_symbolic_plan_for_rule_parse() -> None:
    payload = json.loads(_planner().decompose_task("去会议室"))

    assert payload["status"] == "parsed"
    assert payload["schema_version"] == "task_plan.v1"
    assert payload["subgoals"][0]["action"] == "navigate"
    assert payload["subgoals"][0]["target"] == "会议室"
    assert "x" not in payload["subgoals"][0]["parameters"]


def test_decompose_task_requests_llm_for_complex_plan() -> None:
    payload = json.loads(_planner().decompose_task("先去会议室，然后找到桌子旁边的箱子"))

    assert payload["status"] == "needs_llm"
    assert payload["subgoals"] == []
    assert payload["decomposition_prompt"][0]["role"] == "system"
    assert "Do not resolve room/object IDs" in payload["decomposition_prompt"][0]["content"]


def test_complex_instruction_stops_before_single_goal_shortcut() -> None:
    planner = _planner()
    statuses = []
    plans = []
    planner.planner_status.subscribe(statuses.append)
    planner.task_plan.subscribe(plans.append)

    planner._process_instruction("先去会议室，然后找到桌子旁边的箱子")

    assert statuses[-1] == "TASK_DECOMPOSITION_REQUIRED"
    assert plans[-1]["status"] == "needs_llm"
    assert planner._current_instruction == ""


def test_decompose_task_rejects_blank_instruction() -> None:
    payload = json.loads(_planner().decompose_task("  "))

    assert payload == {"status": "invalid", "error": "instruction must not be blank"}


def _scene() -> SceneGraph:
    return SceneGraph(
        ts=100.0,
        frame_id="map",
        graph_level="hierarchical",
        map_id="building",
        content_epoch=7,
        objects=[
            Detection3D(id="desk-1", label="desk", position=Vector3(2.0, 0.0, 0.7), ts=100.0),
            Detection3D(id="box-1", label="box", position=Vector3(2.4, 0.1, 0.4), ts=100.0),
        ],
        relations=[Relation(subject_id="box-1", predicate="near", object_id="desk-1", confidence=0.9)],
        rooms=[
            {
                "room_id": 1,
                "name": "office",
                "center": {"x": 2.0, "y": 0.0, "z": 0.0},
                "object_ids": ["desk-1", "box-1"],
                "group_ids": [],
            }
        ],
    )


def test_resolve_task_step_grounds_active_room_without_dispatch(monkeypatch) -> None:
    planner = _planner()
    planner._current_scene_graph = _scene()
    monkeypatch.setattr(planner, "_map_sample_is_stale", lambda _: False)
    planner._goal_resolver = SimpleNamespace(
        maybe_reload_kg=lambda: None,
        fast_resolve=Mock(
            return_value=SimpleNamespace(
                is_valid=True,
                confidence=0.8,
                candidate_id=-1,
                target_label="office",
                target_x=2.0,
                target_y=0.0,
                target_z=0.0,
                frame_id="map",
                path="fast",
                reasoning="room center",
            )
        ),
    )
    dispatched = []
    planner.nav_command.subscribe(dispatched.append)

    payload = json.loads(
        planner.resolve_task_step(
            json.dumps(
                {
                    "instruction": "先去会议室，然后找到箱子",
                    "subgoals": [{"step_id": 0, "action": "navigate", "target": "office"}],
                }
            )
        )
    )

    assert payload["status"] == "resolved"
    assert payload["query"] == "office"
    assert payload["map_id"] == "building"
    assert payload["content_epoch"] == 7
    assert payload["frame_id"] == "map"
    assert planner._goal_resolver.fast_resolve.call_args.args[0] == "office"
    assert dispatched == []


def test_resolve_task_step_preserves_relation_query_and_map_identity(monkeypatch) -> None:
    planner = _planner()
    planner._current_scene_graph = _scene()
    monkeypatch.setattr(planner, "_map_sample_is_stale", lambda _: False)
    planner._goal_resolver = SimpleNamespace(
        fast_resolve=Mock(
            return_value=SimpleNamespace(
                is_valid=True,
                confidence=0.9,
                candidate_id="box-1",
                target_label="box",
                target_x=2.4,
                target_y=0.1,
                target_z=0.4,
                frame_id="map",
                path="fast",
            )
        )
    )

    payload = json.loads(
        planner.resolve_task_step(
            json.dumps(
                {
                    "instruction": "先去会议室，然后找到桌子旁边的箱子",
                    "current_step": 1,
                    "subgoals": [
                        {"step_id": 0, "action": "navigate", "target": "office", "status": "completed"},
                        {
                            "step_id": 1,
                            "action": "find",
                            "target": "box",
                            "parameters": {"relation": "near", "reference": "desk"},
                        },
                    ],
                }
            )
        )
    )

    assert payload["status"] == "resolved"
    assert payload["query"] == "box near desk"
    assert payload["candidate_id"] == "box-1"
    assert payload["map_id"] == "building"
    assert planner._goal_resolver.fast_resolve.call_args.args[0] == "box near desk"


def test_goal_resolver_reads_scene_relation_predicate() -> None:
    resolver = GoalResolver.__new__(GoalResolver)
    resolver._fast_path_attempts = 0
    resolver._fast_path_hits = 0
    resolver._resolve_times = []
    resolver._max_resolve_history = 100
    resolver._fast_path_threshold = 0.75
    resolver._scene_cache = None

    result = resolver.fast_resolve("box near table", _scene().to_json())

    assert result is not None
    assert result.candidate_id == "box-1"
    assert "spatial=0.3" in result.reasoning


def test_goal_resolver_matches_chinese_room_alias_to_room_type() -> None:
    resolver = GoalResolver.__new__(GoalResolver)
    resolver._fast_path_attempts = 0
    resolver._fast_path_hits = 0
    resolver._resolve_times = []
    resolver._max_resolve_history = 100
    resolver._fast_path_threshold = 0.75
    resolver._scene_cache = None
    scene = _scene()
    scene.rooms[0]["name"] = "meeting_room"

    result = resolver.fast_resolve("会议室", scene.to_json())

    assert result is not None
    assert result.target_label == "meeting_room"


def test_resolve_task_step_rejects_stale_navigation_map(monkeypatch) -> None:
    planner = _planner()
    planner._current_scene_graph = _scene()
    planner._navigation_context = ("boot-1", "other-map", 8)
    monkeypatch.setattr(planner, "_map_sample_is_stale", lambda _: False)
    resolver = Mock()
    planner._goal_resolver = SimpleNamespace(fast_resolve=resolver)

    payload = json.loads(
        planner.resolve_task_step(
            json.dumps(
                {
                    "instruction": "去办公室",
                    "subgoals": [{"action": "navigate", "target": "office"}],
                }
            )
        )
    )

    assert payload["status"] == "stale_scene_map"
    assert payload["active_map_id"] == "other-map"
    resolver.assert_not_called()


def test_resolve_task_step_requires_current_step_to_match_status(monkeypatch) -> None:
    planner = _planner()
    planner._current_scene_graph = _scene()
    monkeypatch.setattr(planner, "_map_sample_is_stale", lambda _: False)

    payload = json.loads(
        planner.resolve_task_step(
            json.dumps(
                {
                    "instruction": "先去办公室，然后找箱子",
                    "current_step": 1,
                    "subgoals": [
                        {"action": "navigate", "target": "office"},
                        {"action": "find", "target": "box"},
                    ],
                }
            )
        )
    )

    assert payload["status"] == "invalid"
    assert payload["error"] == "current_step_does_not_match_active_subgoal"
