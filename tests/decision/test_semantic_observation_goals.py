"""Observation goals cross the real RPC surface; only the native endpoint is fake."""

import json
import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from decision.modules.semantic_planner import SemanticPlannerModule
from nav.commands.module import Commands
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.nav import NavigationState
from runtime.msgs.semantic import Detection3D, SceneGraph


def _scene(*, x=2.0, z=1.2, object_ts=100.0):
    return SceneGraph(ts=100.0, frame_id="map", objects=[
        Detection3D(id="chair-1", label="chair", position=Vector3(x, 0, z), ts=object_ts),
    ])


def _feasible(x, y, z):
    return {"feasible": True, "start_valid": True, "frame_id": "map",
            "path": [{"x": 0, "y": 0, "z": z}, {"x": x, "y": y, "z": z}]}


@pytest.fixture
def harness(monkeypatch):
    monkeypatch.setattr("decision.modules.semantic_planner.time.time", lambda: 100.0)
    monkeypatch.setattr(SemanticPlannerModule, "_init_backends", lambda self: None)
    client = SimpleNamespace(preview_plan=Mock(side_effect=_feasible))
    monkeypatch.setattr("nav.commands.module.get_native_navigation_client", lambda **kw: client)
    module = SemanticPlannerModule(llm_backend="mock", map_query=SimpleNamespace())
    module._goal_resolver = SimpleNamespace(
        maybe_reload_kg=lambda: None,
        fast_resolve=Mock(return_value=SimpleNamespace(confidence=1.0, candidate_id="chair-1", action="navigate")),
    )
    module._task_decomposer = None
    module._frontier_scorer = None
    module._action_executor = None
    module.setup()
    module.on_system_modules({"nav.commands": Commands()})
    module.robot_pose._deliver(PoseStamped(Pose(Vector3(0, 0, 0.3)), frame_id="map", ts=100.0))
    module.scene_graph._deliver(_scene())
    commands, statuses, servo = [], [], []
    module.nav_command.subscribe(lambda raw: commands.append(json.loads(raw)))
    module.planner_status.subscribe(statuses.append)
    module.servo_target.subscribe(servo.append)
    threads = []
    thread_class = threading.Thread

    def worker(**kwargs):
        thread = thread_class(**kwargs)
        threads.append(thread)
        return thread

    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread", worker)

    def join():
        for thread in tuple(threads):
            thread.join(2.0)
            assert not thread.is_alive(), "observation query did not finish"

    yield SimpleNamespace(module=module, client=client, commands=commands, statuses=statuses, servo=servo, join=join)
    module.stop()
    join()
    assert all(port._publish_errors == 0 for port in module.ports_out.values())


def test_object_center_and_robot_observation_pose_are_distinct(harness):
    h = harness
    h.module.instruction._deliver("find chair")
    h.join()
    goal, = h.commands
    assert (goal["x"], goal["y"], goal["z"], goal["yaw"]) == pytest.approx((1.5, 0, 0.3, 0))
    assert h.module._active_goal.to_dict()["object_target"] == {
        "id": "chair-1", "label": "chair", "position": [2.0, 0.0, 1.2],
    }
    assert h.client.preview_plan.call_count == 1
    assert h.servo == []


def test_blocked_near_side_tries_another_native_checked_position(harness):
    h = harness
    def preview(x, y, z):
        if abs(y) < 0.01:
            return {"feasible": False, "reason": "goal_blocked"}
        return _feasible(x, y, z)
    h.client.preview_plan.side_effect = preview
    h.module.instruction._deliver("find chair")
    h.join()
    goal, = h.commands
    assert abs(goal["y"]) > 0.1
    assert math.hypot(goal["x"] - 2, goal["y"]) == pytest.approx(0.5)
    assert h.client.preview_plan.call_count == 2


@pytest.mark.parametrize("response", [
    {"feasible": False, "reason": "goal_blocked"},
    {"feasible": True, "start_valid": False, "frame_id": "map", "path": [{}]},
    {"feasible": True, "start_valid": True, "frame_id": "odom", "path": [{}]},
    {"feasible": True, "start_valid": True, "frame_id": "map", "path": []},
])
def test_rejected_previews_never_fall_back_to_object_center(harness, response):
    h = harness
    h.client.preview_plan.side_effect = lambda *args: response
    h.module.instruction._deliver("find chair")
    h.join()
    assert h.client.preview_plan.call_count == 8
    assert h.commands == h.servo == []
    assert h.statuses[-1] == "OBSERVATION_PATH_BLOCKED"
    h.module.scene_graph._deliver(_scene())
    assert h.client.preview_plan.call_count == 8


@pytest.mark.parametrize("reason", ["navigation_busy", "planner_busy", "odometry_not_ready", "map_odom_tf_not_ready"])
def test_busy_native_planner_defers_without_trying_all_candidates(harness, reason):
    h = harness
    h.client.preview_plan.side_effect = lambda *args: {"reason": reason}
    h.module.instruction._deliver("find chair")
    h.join()
    assert h.client.preview_plan.call_count == 1
    assert h.commands == h.servo == []
    assert h.statuses[-1] == "OBSERVATION_PLANNER_WAITING"


def test_preview_timeout_does_not_issue_motion(harness):
    h = harness
    h.client.preview_plan.side_effect = TimeoutError("native preview unavailable")
    h.module.instruction._deliver("find chair")
    h.join()
    assert h.commands == h.servo == []
    assert h.statuses[-1] == "OBSERVATION_PLANNER_UNAVAILABLE"


def test_missing_rpc_does_not_issue_motion(harness):
    h = harness
    h.module.on_system_modules({})
    h.module.instruction._deliver("find chair")
    assert h.commands == h.servo == []
    assert h.statuses[-1] == "OBSERVATION_PLANNER_UNAVAILABLE"


@pytest.mark.parametrize("object_ts", [90.0, 110.0])
def test_fresh_graph_cannot_renew_stale_object_evidence(harness, object_ts):
    h = harness
    h.module.scene_graph._deliver(_scene(object_ts=object_ts))
    h.module.instruction._deliver("find chair")
    assert h.commands == h.servo == []
    h.client.preview_plan.assert_not_called()
    assert h.statuses[-1] == "TARGET_NOT_OBSERVED"


@pytest.mark.parametrize("change", ["instruction", "stop", "map", "restart", "moved", "disappeared", "pose_expired"])
def test_blocking_preview_cannot_use_superseded_context(harness, change):
    h = harness
    entered, release = threading.Event(), threading.Event()
    h.module.navigation_state._deliver(NavigationState(boot_id="nav-a", sequence=1, map_id="map-a"))
    def preview(x, y, z):
        entered.set()
        assert release.wait(2.0)
        return _feasible(x, y, z)
    h.client.preview_plan.side_effect = preview
    try:
        h.module.instruction._deliver("find chair")
        assert entered.wait(1.0), "camera callback must not wait for the native RPC"
        if change == "instruction":
            h.module.instruction._deliver("follow person")
        elif change == "stop":
            h.module.stop()
        elif change in {"map", "restart"}:
            h.module.navigation_state._deliver(NavigationState(
                boot_id="nav-b" if change == "restart" else "nav-a", sequence=2,
                map_id="map-b" if change == "map" else "map-a",
            ))
        elif change == "moved":
            h.module.scene_graph._deliver(_scene(x=3))
        elif change == "disappeared":
            h.module.scene_graph._deliver(SceneGraph(ts=100.0, frame_id="map"))
        elif change == "pose_expired":
            h.module.robot_pose._deliver(PoseStamped(Pose(), frame_id="map", ts=90.0))
    finally:
        release.set()
        h.join()
    assert not [cmd for cmd in h.commands if cmd["action"] == "goto"]


def test_stationary_object_does_not_replan_as_robot_approaches(harness):
    h = harness
    h.module.instruction._deliver("find chair")
    h.join()
    h.module.robot_pose._deliver(PoseStamped(Pose(Vector3(0.5, 0.2, 0.3)), frame_id="map", ts=100.0))
    h.module.scene_graph._deliver(_scene())
    h.join()
    assert len(h.commands) == 1
    assert h.client.preview_plan.call_count == 1


def test_knowledge_prior_is_not_treated_as_observed_object(harness):
    h = harness
    h.module._goal_resolver.fast_resolve.return_value.action = "explore"
    h.module._explore_frontier = Mock()
    h.module.instruction._deliver("find chair")
    h.module._explore_frontier.assert_called_once_with("find chair")
    h.client.preview_plan.assert_not_called()
    assert h.commands == []
