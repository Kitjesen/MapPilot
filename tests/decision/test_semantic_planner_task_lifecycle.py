"""Exercise task changes through planner inputs, including delayed recovery."""

import json
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from decision.modules.semantic_planner import SemanticPlannerModule
from nav.services.goals import GoalService
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.nav import (
    NavigationCommandKind,
    NavigationCommandReceipt,
    NavigationGoalState,
    NavigationGoalStatus,
    NavigationLifecycle,
    NavigationState,
)
from runtime.msgs.semantic import Detection3D, SceneGraph


def _scene(ts=100.0, x=2):
    return SceneGraph(
        ts=ts,
        frame_id="map",
        objects=[Detection3D(id="chair-1", label="chair", position=Vector3(x, 0, 0))],
    )


@pytest.fixture
def planner(monkeypatch):
    thread_class = threading.Thread
    def worker(*, target, args, name, **kwargs):
        if name == "semantic-observation":
            return SimpleNamespace(start=lambda: target(*args))
        return thread_class(target=target, args=args, name=name, **kwargs)
    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread", worker)
    monkeypatch.setattr("decision.modules.semantic_planner.time.time", lambda: 100.0)
    monkeypatch.setattr(SemanticPlannerModule, "_init_backends", lambda self: None)
    module = SemanticPlannerModule(llm_backend="mock", map_query=SimpleNamespace(), lera_cooldown=0.0)
    module._goal_resolver = SimpleNamespace(
        maybe_reload_kg=lambda: None,
        fast_resolve=Mock(return_value=SimpleNamespace(confidence=1.0, candidate_id="chair-1", position=[2, 0, 0], frame_id="map")),
    )
    module._task_decomposer = None
    module._action_executor = None
    module._frontier_scorer = None
    commands = _Commands()
    service = GoalService(command_module="nav.commands")
    service.on_system_modules({"nav.commands": commands})
    service.setup()
    module.setup()
    module.on_system_modules({"nav.commands": commands})
    module.nav_command.subscribe(service.goal_command._deliver)
    service.goal_status.subscribe(module.goal_status._deliver)
    service.task_status.subscribe(module.navigation_goal_status._deliver)
    module._test_service = service
    module._test_commands = commands
    goals, cancels, statuses, servo = [], [], [], []
    def collect(raw):
        cmd = json.loads(raw)
        (goals if cmd["action"] == "goto" else cancels).append(cmd)
    module.nav_command.subscribe(collect)
    module.planner_status._add_callback(statuses.append)
    module.servo_target._add_callback(servo.append)
    module.robot_pose._deliver(PoseStamped(Pose(), ts=100.0, frame_id="map"))
    module._on_scene_graph(_scene())
    yield module, goals, cancels, statuses, servo
    module.stop()
    for owner in (module, service):
        assert all(port._publish_errors == 0 for port in owner.ports_out.values())
    service.stop()


class _Commands:
    mode = "accept"
    on_send = None

    def preview_plan(self, x, y, z):
        return {"feasible": True, "start_valid": True, "frame_id": "map", "path": [{"x": x, "y": y, "z": z}]}

    def send_goal(self, x, y, z, yaw, *, task_id, request_id):
        if self.on_send is not None:
            self.on_send(task_id, request_id)
        if self.mode == "timeout":
            raise TimeoutError("ACK unavailable")
        return NavigationCommandReceipt(
            accepted=self.mode != "reject", kind=NavigationCommandKind.GOAL,
            task_id=task_id, request_id=request_id, endpoint_timestamp_s=100.0,
            reason="localization_unavailable" if self.mode == "reject" else "accepted",
        )

    def cancel_task(self, task_id, reason, *, request_id):
        return NavigationCommandReceipt(
            accepted=True, kind=NavigationCommandKind.TASK_CANCEL,
            task_id=task_id, request_id=request_id, endpoint_timestamp_s=100.0,
            reason="cancel_requested",
        )


def _native(module, state, *, command=None, sequence=1, request_id=None, boot_id="test-nav"):
    task = module._active_goal
    command = command or {"task_id": task.task_id, "request_id": task.request_id}
    status = NavigationGoalStatus(
        ts=100.0, frame_id="map", boot_id=boot_id, sequence=sequence,
        task_id=command["task_id"], request_id=request_id or command["request_id"],
        state=state, goal_epoch=1,
    )
    module._test_service.navigation_goal_status._deliver(status)
    return status


def _failed():
    return NavigationState(boot_id="test-nav", sequence=1, lifecycle_state=NavigationLifecycle.FAILED)


def test_follow_handoff_cannot_resurrect_previous_scene_goal(planner):
    module, goals, _, _, servo = planner
    module._on_instruction("find chair")
    assert len(goals) == 1

    module._on_instruction("follow person")
    module._goal_resolver.fast_resolve.return_value.position = [3, 0, 0]
    module._on_scene_graph(_scene())

    assert servo[-1] == "follow:person"
    assert len(goals) == 1
    assert module._current_goal_pose is None


def test_new_unresolved_instruction_cannot_reuse_previous_goal(planner):
    module, goals, _, _, _ = planner
    module._on_instruction("find chair")
    module._latest_sg = None
    module._current_scene_graph = None

    module._on_instruction("find extinguisher")

    assert len(goals) == 1
    assert module._current_goal_pose is None


def test_cached_scene_is_checked_when_instruction_arrives(planner, monkeypatch):
    module, goals, _, statuses, _ = planner
    monkeypatch.setattr("decision.modules.semantic_planner.time.time", lambda: 105.0)

    module._on_instruction("find chair")

    assert goals == []
    module._goal_resolver.fast_resolve.assert_not_called()
    assert statuses[-1] == "WAITING_FOR_FRESH_SCENE_GRAPH"


def test_idle_planner_does_not_recover_another_navigation_task(planner, monkeypatch):
    module, goals, cancels, _, servo = planner
    thread = Mock()
    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread", thread)

    module._on_navigation_state(_failed())

    thread.assert_not_called()
    assert not goals and not cancels and not servo


@pytest.mark.parametrize("outcome", ["abort", "retry_different_path", "error"])
def test_delayed_recovery_does_not_change_replacement_instruction(planner, monkeypatch, outcome):
    module, goals, cancels, statuses, _ = planner
    module._on_instruction("find chair")
    entered, release = threading.Event(), threading.Event()
    workers = []
    thread_class = threading.Thread

    def recovery(**kwargs):
        entered.set()
        assert release.wait(2.0)
        if outcome == "error":
            raise RuntimeError("old recovery failed")
        return outcome

    def capture_thread(**kwargs):
        worker = thread_class(**kwargs)
        if kwargs.get("name") != "semantic-observation":
            workers.append(worker)
        return worker

    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread", capture_thread)
    module._action_executor = SimpleNamespace(lera_recover=recovery)
    try:
        _native(module, NavigationGoalState.FAILED)
        assert entered.wait(2.0)
        module._on_instruction("find table")
        published_count = len(goals)
        latest_status = statuses[-1]
    finally:
        release.set()
        for worker in workers:
            worker.join(timeout=2.0)
            assert not worker.is_alive()

    assert module._current_instruction == "find table"
    assert len(goals) == published_count
    assert cancels == []
    assert statuses[-1] == latest_status
    assert not module._lera_running


def test_goal_identity_reaches_the_native_service(planner):
    module, goals, _, statuses, _ = planner
    module.instruction._deliver("find chair")
    goal = goals[-1]
    assert goal["task_id"] != goal["request_id"]
    task = module._test_service.get_task(goal["task_id"])
    assert task["target"]["x"] == 1.5
    assert module._active_goal.target.position == (2.0, 0.0, 0.0)
    assert statuses[-1] == "NAVIGATION_ACCEPTED"


@pytest.mark.parametrize("binding", ["current", "other_map", "other_epoch", "unbound"])
def test_memory_goal_requires_current_map_provenance(planner, binding):
    module, goals, _, statuses, _ = planner
    module.navigation_state._deliver(NavigationState(
        boot_id="test-nav", sequence=1, map_id="office", map_content_epoch=2,
    ))
    best = {"x": 5.0, "y": 6.0, "z": 0.3, "score": 0.9, "navigable": True,
            "map_id": "office", "map_content_epoch": 2, "frame_id": "map"}
    if binding == "other_map":
        best["map_id"] = "yard"
    elif binding == "other_epoch":
        best["map_content_epoch"] = 3
    elif binding == "unbound":
        best.pop("map_id")
    module._backends.vector_memory = SimpleNamespace(
        query_location=lambda _: json.dumps({
            "found": True, "best": best, "navigable": True,
            "semantic_encoder_ready": True, "degraded": False,
        }),
        get_memory_stats=lambda: json.dumps({"semantic_encoder_ready": True, "degraded": False}),
    )
    assert module._try_vector_memory("find chair") is (binding == "current")
    if binding == "current":
        assert goals[-1]["x"] == 5.0
        assert goals[-1]["z"] == 0.3
    else:
        assert goals == []
        assert statuses[-1] == "VECTOR_MEMORY_QUERY_ONLY"


def test_rejected_goal_is_not_reported_as_resolved_or_repeated(planner):
    module, goals, _, statuses, _ = planner
    module._test_commands.mode = "reject"
    module.instruction._deliver("find chair")
    module._goal_resolver.fast_resolve.return_value.position = [3, 0, 0]
    module.scene_graph._deliver(_scene())
    assert len(goals) == 1
    assert statuses[-1] == "NAVIGATION_REJECTED"
    assert module._active_goal.terminal


def test_lost_ack_cannot_start_another_goal_or_servo(planner):
    module, goals, _, statuses, servo = planner
    module._test_commands.mode = "timeout"
    module.instruction._deliver("find chair")
    module._goal_resolver.fast_resolve.return_value = None
    module.scene_graph._deliver(_scene())
    assert len(goals) == 1
    assert servo == []
    assert statuses[-1] == "NAVIGATION_UNCONFIRMED"


@pytest.mark.parametrize("purpose,expected", [
    ("place", "COMPLETED"), ("object", "TARGET_VERIFICATION_REQUIRED"),
    ("memory", "TARGET_VERIFICATION_REQUIRED"), ("frontier", "TARGET_VERIFICATION_REQUIRED"),
])
def test_arrival_ends_geometric_task_without_inventing_object_verification(planner, purpose, expected):
    module, goals, _, statuses, _ = planner
    module._current_instruction = "find chair"
    module._dispatch_navigation_goal(PoseStamped(Pose(Vector3(2, 0, 0)), frame_id="map"),
                                     instruction="find chair", purpose=purpose)
    _native(module, NavigationGoalState.REACHED)
    module._goal_resolver.fast_resolve.return_value.position = [3, 0, 0]
    module.scene_graph._deliver(_scene())
    assert len(goals) == 1
    assert statuses[-1] == expected
    assert module._active_goal.terminal
    assert module._current_instruction == ""


def test_cancel_ack_is_not_a_terminal_event(planner):
    module, goals, cancels, statuses, _ = planner
    module.instruction._deliver("find chair")
    module._cancel_owned_goals("operator_cancel")
    assert cancels[-1]["task_id"] == goals[-1]["task_id"]
    assert cancels[-1]["request_id"] != goals[-1]["request_id"]
    assert statuses[-1] == "CANCELLING"
    assert not module._active_goal.terminal
    _native(module, NavigationGoalState.CANCELLED, request_id=cancels[-1]["request_id"])
    assert statuses[-1] == "CANCELLED"
    assert module._active_goal.terminal
    assert module._navigation_goals == {}


def test_external_cancel_with_a_new_request_id_ends_owned_task(planner):
    module, goals, _, statuses, _ = planner
    module.instruction._deliver("find chair")
    result = module._test_service.submit_cancel("web_cancel", task_id=goals[-1]["task_id"], request_id="web-cancel")
    assert result["accepted"] is True
    _native(module, NavigationGoalState.CANCELLED, request_id="web-cancel")
    module.scene_graph._deliver(_scene())
    assert statuses[-1] == "CANCELLED"
    assert len(goals) == 1


def test_old_task_completion_cannot_end_the_new_instruction(planner):
    module, goals, cancels, statuses, _ = planner
    module.instruction._deliver("find chair")
    old = goals[-1]
    module.instruction._deliver("find table")
    new = goals[-1]
    assert cancels[-1]["task_id"] == old["task_id"]
    assert old["task_id"] != new["task_id"]
    previous_status = statuses[-1]
    _native(module, NavigationGoalState.REACHED, command=old)
    assert module._current_instruction == "find table"
    assert module._active_goal.task_id == new["task_id"]
    assert statuses[-1] == previous_status


def test_terminal_event_before_ack_stays_terminal(planner):
    module, _, _, statuses, _ = planner
    module._test_commands.on_send = lambda task_id, request_id: _native(
        module, NavigationGoalState.REACHED, command={"task_id": task_id, "request_id": request_id},
    )
    module.instruction._deliver("find chair")
    assert statuses[-1] == "TARGET_VERIFICATION_UNAVAILABLE"
    assert module._active_goal.terminal


def test_lifecycle_before_lost_ack_stays_authoritative(planner):
    module, _, _, statuses, _ = planner
    module._test_commands.mode = "timeout"
    module._test_commands.on_send = lambda task_id, request_id: _native(
        module, NavigationGoalState.PATH_ACTIVE, command={"task_id": task_id, "request_id": request_id},
    )
    module.instruction._deliver("find chair")
    assert statuses[-1] == "NAVIGATION_PATH_ACTIVE"
    assert module._active_goal.state == "path_active"


def test_older_lifecycle_event_is_not_forwarded(planner):
    module, _, _, statuses, _ = planner
    forwarded = []
    module._test_service.task_status.subscribe(forwarded.append)
    module.instruction._deliver("find chair")
    _native(module, NavigationGoalState.PATH_ACTIVE, sequence=3)
    _native(module, NavigationGoalState.PLANNING, sequence=2)
    assert [event.sequence for event in forwarded] == [3]
    assert statuses[-1] == "NAVIGATION_PATH_ACTIVE"


def test_untracked_native_event_is_not_forwarded(planner):
    module, _, _, statuses, _ = planner
    module.instruction._deliver("find chair")
    _native(module, NavigationGoalState.REACHED, command={"task_id": "web-task", "request_id": "web-request"})
    assert statuses[-1] == "NAVIGATION_ACCEPTED"
    assert module._current_instruction == "find chair"


def test_native_recovery_is_not_interrupted_by_semantic_recovery(planner, monkeypatch):
    module, goals, _, _, _ = planner
    module.instruction._deliver("find chair")
    worker = Mock()
    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread", worker)
    module.navigation_state._deliver(NavigationState(
        boot_id="test-nav", sequence=1, lifecycle_state=NavigationLifecycle.RECOVERING,
        active_task_id=goals[-1]["task_id"], active_request_id=goals[-1]["request_id"],
    ))
    worker.assert_not_called()
    assert len(goals) == 1


@pytest.mark.parametrize("lifecycle", [NavigationLifecycle.RECOVERING, NavigationLifecycle.PAUSED])
def test_native_pause_or_recovery_defers_scene_goal_updates(planner, lifecycle):
    module, goals, _, _, servo = planner
    module.instruction._deliver("find chair")
    original = goals[-1]
    module.navigation_state._deliver(NavigationState(
        boot_id="test-nav", sequence=1, lifecycle_state=lifecycle,
        active_task_id=original["task_id"], active_request_id=original["request_id"],
    ))
    module.scene_graph._deliver(_scene(x=3))
    assert len(goals) == 1
    assert servo == []
    module.navigation_state._deliver(NavigationState(
        boot_id="test-nav", sequence=2, lifecycle_state=NavigationLifecycle.EXECUTING,
        active_task_id=original["task_id"], active_request_id=original["request_id"],
    ))
    module.scene_graph._deliver(_scene(x=3))
    assert len(goals) == 2


def test_goal_pause_cannot_be_undone_by_scene_update(planner):
    module, goals, _, _, _ = planner
    module.instruction._deliver("find chair")
    _native(module, NavigationGoalState.PAUSED)
    module.scene_graph._deliver(_scene(x=3))
    assert len(goals) == 1
    _native(module, NavigationGoalState.PATH_ACTIVE, sequence=2)
    module.scene_graph._deliver(_scene(x=3))
    assert len(goals) == 2


@pytest.mark.parametrize("changed", [
    {"map_id": "map-b", "map_content_epoch": 1},
    {"map_id": "map-a", "map_content_epoch": 2},
    {"boot_id": "restarted-nav", "map_id": "map-a", "map_content_epoch": 1},
])
def test_map_or_native_restart_invalidates_cached_goal(planner, changed):
    module, goals, cancels, statuses, _ = planner
    module.navigation_state._deliver(NavigationState(
        boot_id="test-nav", sequence=1, map_id="map-a", map_content_epoch=1,
    ))
    module.instruction._deliver("find chair")
    module.navigation_state._deliver(NavigationState(**{"boot_id": "test-nav", "sequence": 2, **changed}))
    module.scene_graph._deliver(_scene())
    assert len(goals) == 1
    assert cancels[-1]["task_id"] == goals[-1]["task_id"]
    assert statuses[-1] == "NAVIGATION_CONTEXT_CHANGED"
    assert module._current_instruction == ""


def test_failed_goal_retries_only_with_fresh_scene(planner, monkeypatch):
    module, goals, _, statuses, _ = planner
    module.instruction._deliver("find chair")
    monkeypatch.setattr("decision.modules.semantic_planner.time.time", lambda: 105.0)
    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread",
                        lambda *, target, args, **kw: SimpleNamespace(start=lambda: target(*args)))
    _native(module, NavigationGoalState.FAILED)
    assert len(goals) == 1
    assert statuses[-1] == "WAITING_FOR_FRESH_SCENE_GRAPH"
    module.robot_pose._deliver(PoseStamped(Pose(), ts=105.0, frame_id="map"))
    module.scene_graph._deliver(_scene(ts=105.0))
    assert len(goals) == 2
    assert goals[-1]["task_id"] != goals[0]["task_id"]


def test_unknown_recovery_strategy_cannot_restart_on_next_scene(planner, monkeypatch):
    module, goals, _, statuses, _ = planner
    module.instruction._deliver("find chair")
    module._action_executor = SimpleNamespace(lera_recover=lambda **kw: "unexpected-model-response")
    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread",
                        lambda *, target, args, **kw: SimpleNamespace(start=lambda: target(*args)))
    _native(module, NavigationGoalState.FAILED)
    module._goal_resolver.fast_resolve.return_value.position = [3, 0, 0]
    module.scene_graph._deliver(_scene())
    assert statuses[-1] == "ABORTED"
    assert len(goals) == 1


def test_failed_cancelled_goal_cannot_restart_on_next_scene(planner):
    module, goals, _, statuses, _ = planner
    module.instruction._deliver("find chair")
    module._cancel_owned_goals("operator_cancel")
    _native(module, NavigationGoalState.FAILED)
    module._goal_resolver.fast_resolve.return_value.position = [3, 0, 0]
    module.scene_graph._deliver(_scene())
    assert len(goals) == 1
    assert statuses[-1] == "NAVIGATION_FAILED"


def test_new_goal_stops_prior_visual_following(planner):
    module, goals, _, _, servo = planner
    module.instruction._deliver("follow person")
    module.instruction._deliver("find chair")
    assert servo == ["follow:person", "stop"]
    assert len(goals) == 1


def test_visual_fallback_is_a_handoff_not_a_repeated_command(planner):
    module, _, _, _, servo = planner
    module._goal_resolver.fast_resolve.return_value = None
    module.instruction._deliver("find chair")
    module.scene_graph._deliver(_scene())
    module.scene_graph._deliver(_scene())
    assert servo == ["find:find chair"]
    module.stop()
    assert servo[-1] == "stop"


def test_old_geometric_goal_cannot_overwrite_visual_handoff_status(planner):
    module, goals, _, statuses, _ = planner
    module.instruction._deliver("find chair")
    old = goals[-1]
    module._goal_resolver.fast_resolve.return_value = None
    module.instruction._deliver("find another chair")
    _native(module, NavigationGoalState.CANCELLED, command=old)
    assert statuses[-1] == "VISUAL_SERVO"


def test_repeated_terminal_event_does_not_launch_duplicate_recovery(planner, monkeypatch):
    module, _, _, _, _ = planner
    module.instruction._deliver("find chair")
    worker = Mock()
    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread", worker)
    _native(module, NavigationGoalState.FAILED)
    _native(module, NavigationGoalState.FAILED)
    worker.assert_called_once()
    assert module._lera_count == 1


def test_delayed_recovery_cannot_publish_after_stop(planner, monkeypatch):
    module, goals, _, _, _ = planner
    module.instruction._deliver("find chair")
    entered, release = threading.Event(), threading.Event()
    workers = []
    thread_class = threading.Thread

    def recover(**kwargs):
        entered.set()
        assert release.wait(2.0)
        return "retry_different_path"

    def capture(**kwargs):
        worker = thread_class(**kwargs)
        workers.append(worker)
        return worker

    module._action_executor = SimpleNamespace(lera_recover=recover)
    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread", capture)
    try:
        _native(module, NavigationGoalState.FAILED)
        assert entered.wait(2.0)
        module.stop()
        counts = (module.nav_command.msg_count, module.servo_target.msg_count, module.planner_status.msg_count)
    finally:
        release.set()
        for worker in workers:
            worker.join(2.0)
            assert not worker.is_alive()
    assert counts == (module.nav_command.msg_count, module.servo_target.msg_count, module.planner_status.msg_count)
    assert len(goals) == 1


def test_failure_during_cooldown_is_delayed_not_lost(planner, monkeypatch):
    module, goals, _, _, _ = planner
    module.instruction._deliver("find chair")
    waits = []
    module._lera_cooldown = 5.0
    module._last_lera_time = 1000.0
    monkeypatch.setattr("decision.modules.semantic_planner.time.monotonic", lambda: 1001.0)
    monkeypatch.setattr("decision.modules.semantic_planner.threading.Event", lambda: SimpleNamespace(wait=waits.append))
    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread",
                        lambda *, target, args, **kw: SimpleNamespace(start=lambda: target(*args)))
    _native(module, NavigationGoalState.FAILED)
    assert waits == [4.0]
    assert len(goals) == 2


def test_retry_failure_before_worker_returns_is_not_lost(planner, monkeypatch):
    module, goals, _, statuses, _ = planner
    module.instruction._deliver("find chair")
    module._action_executor = SimpleNamespace(
        lera_recover=lambda **kw: "abort" if kw["failure_count"] >= 3 else "retry_different_path",
    )
    module._test_commands.on_send = lambda task_id, request_id: _native(
        module, NavigationGoalState.FAILED, command={"task_id": task_id, "request_id": request_id},
    )
    monkeypatch.setattr("decision.modules.semantic_planner.threading.Thread",
                        lambda *, target, args, **kw: SimpleNamespace(start=lambda: target(*args)))
    _native(module, NavigationGoalState.FAILED)
    assert len(goals) == 3
    assert statuses[-1] == "ABORTED"
    assert module._current_instruction == ""


@pytest.mark.parametrize("frame,stamp,x", [
    ("odom", 100.0, 2.0), ("map", 95.0, 2.0), ("map", 100.0, float("nan")),
])
def test_scene_grounding_requires_fresh_finite_map_pose(planner, frame, stamp, x):
    module, goals, _, statuses, servo = planner
    module.robot_pose._deliver(PoseStamped(Pose(Vector3(x, 0, 0)), ts=stamp, frame_id=frame))
    module.instruction._deliver("find chair")
    module._goal_resolver.fast_resolve.assert_not_called()
    assert goals == []
    assert servo == []
    assert statuses[-1] == "WAITING_FOR_MAP_POSE"


def test_map_pose_expiry_is_checked_again_when_resolving(planner, monkeypatch):
    module, goals, _, statuses, _ = planner
    monkeypatch.setattr("decision.modules.semantic_planner.time.time", lambda: 105.0)
    module.scene_graph._deliver(_scene(ts=105.0))
    module.instruction._deliver("find chair")
    assert goals == []
    assert statuses[-1] == "WAITING_FOR_MAP_POSE"
