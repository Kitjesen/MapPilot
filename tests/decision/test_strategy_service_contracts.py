"""Use concrete strategies so permissive service mocks cannot hide API drift."""

import asyncio
import json
import threading
from types import SimpleNamespace
from unittest.mock import AsyncMock

import numpy as np
import pytest

from decision.frontiers.scorer import FrontierScorer
from decision.goals.resolver import GoalResolver
from decision.llm.client import LLMConfig
from decision.modules.semantic_planner import SemanticPlannerModule
from decision.tasks.actions import ActionExecutor
from decision.tasks.services import ActionExecutionService, FrontierExplorationService, GoalResolutionService
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.nav import NavigationGoalState, NavigationGoalStatus
from runtime.msgs.semantic import Detection3D, SceneGraph


@pytest.fixture
def resolver(tmp_path):
    return GoalResolver(primary_config=LLMConfig(backend="mock"), save_dir=str(tmp_path), fast_path_threshold=0.5)


def _chairs():
    return SceneGraph(objects=[
        Detection3D(id=str(i), label="chair", confidence=0.99, position=Vector3(x, 0, 0))
        for i, x in [(1, 1.0), (2, 9.0)]
    ])


def test_goal_service_preserves_robot_position_for_target_selection(resolver):
    result = GoalResolutionService(resolver).resolve_fast("find chair", _chairs().to_json(), {"x": 8, "y": 0})
    assert result is not None
    assert result.target_x == 9.0


def test_planner_preserves_robot_position_for_target_selection(resolver, observation_backend):
    module = SemanticPlannerModule(fast_path_threshold=0.5)
    module._goal_resolver = resolver
    module.on_system_modules(observation_backend)
    module._on_robot_pose(PoseStamped(pose=Pose(position=Vector3(8, 0, 0)), frame_id="map"))
    module._on_scene_graph(_chairs())
    goals = []
    module.nav_command.subscribe(lambda raw: goals.append(json.loads(raw)))

    module._continue_scene_resolution("find chair")

    assert len(goals) == 1
    assert goals[0]["x"] == 8.5
    assert module._active_goal.target.position == (9.0, 0.0, 0.0)


def test_retained_nearby_track_cannot_hide_current_object(resolver, observation_backend):
    module = SemanticPlannerModule(fast_path_threshold=0.5)
    module._goal_resolver = resolver
    module.on_system_modules(observation_backend)
    module._on_robot_pose(PoseStamped(Pose(Vector3(8, 0, 0)), frame_id="map"))
    scene = _chairs()
    scene.objects[1].ts -= 10.0
    module._on_scene_graph(scene)
    module._continue_scene_resolution("find chair")
    assert module._active_goal.target.object_id == "1"
    assert module._active_goal.target.position == (1.0, 0.0, 0.0)
    assert len(module._current_scene_graph.objects) == 2


def test_async_resolution_calls_real_resolver_entrypoint(resolver):
    resolver._call_with_fallback = AsyncMock(return_value=json.dumps({
        "action": "explore", "target": {"x": 2, "y": 1, "z": 0}, "confidence": 0.4,
    }))
    result = asyncio.run(GoalResolutionService(resolver).resolve_slow("find quokka", '{"objects":[]}'))
    assert result is not None
    assert result.action == "explore"
    resolver._call_with_fallback.assert_awaited_once()


def _frontier_grid():
    grid = np.full((12, 12), -1, dtype=np.int8)
    grid[3:9, 3:9] = 0
    return grid


def test_frontier_service_evaluates_and_remembers_failure_with_real_scorer():
    scorer = FrontierScorer(tsp_reorder=False)
    service = FrontierExplorationService(scorer)
    args = (_frontier_grid(), 0.1, 0.0, 0.0, np.array([0.6, 0.6]))
    first = service.evaluate(*args, instruction="find chair")
    assert first is not None
    first_score = first.score

    service.record_failure(first.center_world)
    second = service.evaluate(*args, instruction="find chair")

    assert second is not None
    assert second.score < first_score


def test_planner_dispatches_actual_frontier_coordinates():
    scorer = FrontierScorer()
    scorer.update_costmap(_frontier_grid(), 0.1, 2.0, 3.0)
    scorer.extract_frontiers(np.array([2.6, 3.6]))
    scorer.score_frontiers("explore", np.array([2.6, 3.6]))
    expected = scorer.get_best_frontier()
    assert expected is not None
    module = SemanticPlannerModule()
    module._frontier_scorer = scorer
    goals, servo = [], []
    module.nav_command.subscribe(lambda raw: goals.append(json.loads(raw)))
    module.servo_target._add_callback(servo.append)

    module._explore_frontier("find chair")

    assert len(goals) == 1
    assert (goals[0]["x"], goals[0]["y"]) == pytest.approx(expected.center_world)
    assert servo == []


def test_action_service_uses_real_executor_for_positions_and_standoff():
    executor = ActionExecutor(approach_distance=0.5)
    service = ActionExecutionService(executor)
    target, robot = np.array([4.0, 0.0, 0.2]), np.array([0.0, 0.0, 0.2])
    goal = service.navigate(target, robot)
    approach = service.approach(target, robot, stop_distance=1.0)
    look = service.look_around(robot)

    assert (goal.target_x, goal.target_y, goal.target_z) == (4.0, 0.0, 0.2)
    assert approach.target_x == 3.0
    assert executor.approach_distance == 0.5
    assert look.command_type == "velocity"
    assert look.angular_z != 0.0


def test_action_service_recovery_uses_real_executor_and_failure_count():
    service = ActionExecutionService(ActionExecutor())
    strategy = asyncio.run(service.recover("find chair", {"failure_count": 3}))
    assert strategy == "abort"


def test_action_service_can_await_synchronous_executor_recovery():
    llm = AsyncMock()
    llm.chat.return_value = '{"action":"expand_search","reason":"target absent","params":{}}'
    service = ActionExecutionService(ActionExecutor(), llm)
    strategy = asyncio.run(service.recover("find chair", {"current_labels": ["table"]}))
    assert strategy == "expand_search"
    llm.chat.assert_awaited_once()


def test_planner_recovery_receives_configured_model_client(resolver, monkeypatch, observation_backend):
    module = SemanticPlannerModule()
    module._goal_resolver = resolver
    module.on_system_modules(observation_backend)
    module._on_robot_pose(PoseStamped(Pose(), frame_id="map"))
    module._on_scene_graph(_chairs())
    module._continue_scene_resolution("find chair")
    observed = []

    def recover(**kwargs):
        observed.append(kwargs["llm_client"])
        return "abort"

    def immediate_worker(*, target, args, **kwargs):
        return SimpleNamespace(start=lambda: target(*args))

    module._action_executor = SimpleNamespace(lera_recover=recover)
    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread", immediate_worker)
    module._on_navigation_goal_status(NavigationGoalStatus(
        ts=100.0, frame_id="map", boot_id="nav-test", sequence=1, state=NavigationGoalState.FAILED,
        task_id=module._active_goal.task_id, request_id=module._active_goal.request_id,
    ))

    assert observed == [resolver._primary]


@pytest.fixture
def observation_backend(monkeypatch):
    thread_class = threading.Thread
    def worker(*, target, args, name, **kwargs):
        if name == "semantic-observation":
            return SimpleNamespace(start=lambda: target(*args))
        return thread_class(target=target, args=args, name=name, **kwargs)
    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread", worker)
    return {"nav.commands": SimpleNamespace(preview_plan=lambda x, y, z: {
        "feasible": True, "start_valid": True, "frame_id": "map", "path": [{"x": x, "y": y, "z": z}],
    })}
