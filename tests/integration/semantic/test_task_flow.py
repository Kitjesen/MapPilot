"""Offline contract between hierarchical scene semantics and native navigation."""

from __future__ import annotations

import json
import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from decision.modules.semantic_planner import SemanticPlannerModule
from decision.tasks.decomposition import SubGoalAction, TaskDecomposer
from nav.commands.module import Commands
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.semantic import Detection3D, Relation, SceneGraph, SemanticObservation
from runtime.msgs.sensor import Image, ImageFormat


def _hierarchical_scene() -> SceneGraph:
    return SceneGraph(
        ts=100.0,
        frame_id="map",
        graph_level="hierarchical",
        graph_version="3.0",
        map_id="building",
        content_epoch=7,
        objects=[
            Detection3D(id="desk-1", label="desk", position=Vector3(2.0, 0.0, 0.7), ts=100.0),
            Detection3D(id="box-1", label="box", position=Vector3(2.4, 0.1, 0.4), ts=100.0),
        ],
        relations=[Relation(subject_id="box-1", predicate="near", object_id="desk-1", confidence=0.9)],
        floors=[{"floor_id": 0, "room_ids": [1], "object_ids": ["desk-1", "box-1"]}],
        rooms=[{"room_id": 1, "name": "office", "center": {"x": 2.0, "y": 0.0, "z": 0.0},
                "object_ids": ["desk-1", "box-1"], "group_ids": [4]}],
        groups=[{"group_id": 4, "room_id": 1, "name": "furniture", "object_ids": ["desk-1", "box-1"]}],
        hierarchy_edges=[
            {"parent_type": "floor", "parent_id": 0, "child_type": "room", "child_id": 1},
            {"parent_type": "room", "parent_id": 1, "child_type": "group", "child_id": 4},
            {"parent_type": "group", "parent_id": 4, "child_type": "object", "child_id": "desk-1"},
            {"parent_type": "group", "parent_id": 4, "child_type": "object", "child_id": "box-1"},
        ],
        topology_edges=[{"from_room": 1, "to_room": 1, "type": "internal"}],
    )


def test_hierarchy_can_ground_a_symbolic_task_without_coordinates() -> None:
    scene = _hierarchical_scene()

    assert scene.validate_hierarchy() == ()
    assert scene.map_id == "building"
    assert scene.content_epoch == 7
    assert scene.get_object_by_id("box-1").label == "box"
    assert scene.relations[0].subject_id == "box-1"
    assert scene.relations[0].predicate == "near"

    plan = TaskDecomposer().parse_decomposition_response(
        "去办公室，找到桌子旁边的箱子，并停在它前面",
        {
            "subgoals": [
                {"action": "navigate", "target": "office"},
                {"action": "find", "target": "box near desk"},
                {"action": "approach", "target": "box near desk"},
                {"action": "verify", "target": "box near desk"},
            ]
        },
    )

    assert [step.action for step in plan.subgoals] == [
        SubGoalAction.NAVIGATE, SubGoalAction.FIND, SubGoalAction.APPROACH, SubGoalAction.VERIFY,
    ]
    assert all("x" not in step.parameters and "y" not in step.parameters for step in plan.subgoals)
    assert plan.subgoals[1].target == "box near desk"


@pytest.fixture
def navigation_harness(monkeypatch):
    monkeypatch.setattr("decision.modules.semantic_planner.time.time", lambda: 100.0)
    monkeypatch.setattr(SemanticPlannerModule, "_init_backends", lambda self: None)
    preview = Mock(return_value={
        "feasible": True, "start_valid": True, "frame_id": "map",
        "path": [{"x": 0.0, "y": 0.0, "z": 0.3}, {"x": 1.5, "y": 0.0, "z": 0.3}],
    })
    monkeypatch.setattr("nav.commands.module.get_native_navigation_client",
                        lambda **_: SimpleNamespace(preview_plan=preview))

    module = SemanticPlannerModule(llm_backend="mock", map_query=SimpleNamespace())
    module._goal_resolver = SimpleNamespace(
        maybe_reload_kg=lambda: None,
        fast_resolve=Mock(return_value=SimpleNamespace(
            confidence=1.0, candidate_id="box-1", action="navigate",
        )),
    )
    module._task_decomposer = None
    module._frontier_scorer = None
    module._action_executor = None
    module.setup()
    module.on_system_modules({"nav.commands": Commands()})
    scene = _hierarchical_scene()
    module.observation._deliver(SemanticObservation(
        product_session_id=module._product_session_id, sequence=1, source_ts=100.0,
        source_generation=10, map_id=scene.map_id, content_epoch=scene.content_epoch,
        status="positive", scene_graph=scene, detections=tuple(scene.objects),
        robot_pose=PoseStamped(Pose(Vector3(0.0, 0.0, 0.3)), frame_id="map", ts=100.0),
        image=Image(np.zeros((32, 32, 3), dtype=np.uint8), format=ImageFormat.BGR, ts=100.0),
    ))

    commands, statuses, threads = [], [], []
    module.nav_command.subscribe(lambda raw: commands.append(json.loads(raw)))
    module.planner_status.subscribe(statuses.append)
    thread_class = threading.Thread

    def worker(**kwargs):
        thread = thread_class(**kwargs)
        threads.append(thread)
        return thread

    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread", worker)

    yield SimpleNamespace(module=module, preview=preview, commands=commands, statuses=statuses, threads=threads)

    module.stop()
    for thread in threads:
        thread.join(2.0)
        assert not thread.is_alive()


def test_native_preview_is_required_before_semantic_goal_execution(navigation_harness) -> None:
    harness = navigation_harness
    harness.module.instruction._deliver("找到桌子旁边的箱子")
    for thread in harness.threads:
        thread.join(2.0)

    assert harness.preview.call_count == 1
    goal = harness.commands[0]
    assert goal["action"] == "goto"
    assert goal["source"] == "semantic"
    assert goal["z"] == pytest.approx(0.3)
    assert math.hypot(goal["x"] - 2.4, goal["y"] - 0.1) == pytest.approx(0.5)
    assert (goal["x"], goal["y"]) != pytest.approx((2.4, 0.1))


def test_rejected_native_previews_never_dispatch_object_center(navigation_harness) -> None:
    harness = navigation_harness
    harness.preview.return_value = {"feasible": False, "reason": "goal_blocked"}
    harness.module.instruction._deliver("找到桌子旁边的箱子")
    for thread in harness.threads:
        thread.join(2.0)

    assert harness.preview.call_count == 8
    assert harness.commands == []
    assert harness.statuses[-1] == "OBSERVATION_PATH_BLOCKED"
