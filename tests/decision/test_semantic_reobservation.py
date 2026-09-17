"""Bounded view/candidate changes through real semantic and native task services."""

import asyncio
import json
import math
import threading
import time
from types import SimpleNamespace

import pytest

from decision.goals.resolver import GoalResolver
from decision.llm.client import LLMConfig
from decision.modules.agent_planner import AgentPlannerModule
from runtime.msgs.geometry import Vector3
from runtime.msgs.nav import NavigationCommandKind, NavigationCommandReceipt, NavigationGoalState, NavigationState
from runtime.msgs.semantic import Detection3D
from tests.decision.test_semantic_target_verification import harness


def observe(h, stamp, *, candidates=1):
    task = h.module._active_goal
    extras = [Detection3D(id=f"chair-{index}", label="chair", confidence=0.95,
                          position=Vector3(index + 1.0, 0, 1), ts=stamp, bbox_2d=[4, 4, 28, 28])
              for index in range(1, candidates + 1) if f"chair-{index}" != task.target.object_id]
    h.observe(stamp, robot_position=(task.pose.x, task.pose.y, task.pose.z),
              object_id=task.target.object_id, object_x=task.target.position[0], extra_objects=extras)


def inconclusive_view(h, start):
    h.goal_sent.clear()
    for offset in (0.0, 0.6, 1.2):
        observe(h, start + offset)
        h.await_request()
        h.reply("uncertain")


def alternate_resolver(h):
    def resolve(instruction, scene, **kwargs):
        assert instruction == "find the red chair"
        objects = [obj for obj in json.loads(scene)["objects"] if obj["id"] not in kwargs.get("excluded_object_ids", set())]
        return SimpleNamespace(confidence=1.0, candidate_id=objects[0]["id"], action="navigate")
    h.module._goal_resolver.fast_resolve.side_effect = resolve


def test_new_detection_does_not_replace_an_object_still_being_approached(harness):
    h = harness
    h.module._goal_resolver.fast_resolve.return_value = SimpleNamespace(
        confidence=1.0, candidate_id="chair-2", action="navigate")
    observe(h, 100.1, candidates=2)
    assert len(h.goals) == 1
    assert h.module._active_goal.target.object_id == "chair-1"
    assert len(h.module._object_candidates) == 1


@pytest.mark.parametrize("result", [None, SimpleNamespace(confidence=0.1),
                                    SimpleNamespace(confidence=1.0, action="explore")])
def test_temporary_recognition_failure_keeps_admitted_object_task(harness, result):
    h = harness
    task = h.module._active_goal
    servo = []
    h.module.servo_target.subscribe(servo.append)
    h.module._goal_resolver.fast_resolve.return_value = result
    observe(h, 100.1)
    assert h.module._active_goal is task
    assert not task.cancel_request_id
    assert servo == []
    assert len(h.goals) == 1


def test_nearby_alternative_has_its_own_navigation_and_verification(harness):
    h = harness
    alternate_resolver(h)
    h.reach()
    h.observe(100.1, extra_objects=[Detection3D(
        id="chair-2", label="chair", confidence=0.95, position=Vector3(2.02, 0, 1),
        ts=100.1, bbox_2d=[4, 4, 28, 28],
    )])
    h.await_request()
    h.goal_sent.clear()
    h.reply("mismatch")
    assert h.goal_sent.wait(1.0), "a different object must not be suppressed by pose deduplication"
    assert h.module._active_goal.target.object_id == "chair-2"
    assert len(h.goals) == 2
    assert abs(h.goals[1]["x"] - h.goals[0]["x"]) < 0.05
    h.reach()
    assert h.module._verification.target.object_id == "chair-2"


def test_public_instruction_skill_uses_the_same_task_lifecycle(harness):
    h = harness
    old = h.module._active_goal
    h.goal_sent.clear()
    response = json.loads(h.module.send_instruction("find the red chair"))
    assert response["status"] == "sent"
    assert h.goal_sent.wait(1.0)
    assert old.cancel_request_id
    assert h.module._active_goal is not old
    assert h.module._active_goal.target.object_id == "chair-1"


def test_agent_object_tool_reaches_native_preview_through_semantic_owner(harness):
    h = harness
    agent = AgentPlannerModule(llm_backend="mock")
    agent.on_system_modules({"SemanticPlannerModule": h.module})
    checked = []
    preview = h.modules["nav.commands"].preview_plan

    def record_preview(*args):
        checked.append(args)
        return preview(*args)

    h.modules["nav.commands"].preview_plan = record_preview
    h.goal_sent.clear()
    async def navigate():
        run = agent._new_run("find the red chair")
        result = asyncio.create_task(agent._tool_navigate_to_object("the red chair", run=run))
        assert await asyncio.to_thread(h.goal_sent.wait, 1.0)
        assert checked
        assert "goal_pose" not in agent.ports_out
        assert h.module._active_goal.target.object_id == "chair-1"
        assert h.module._active_goal.pose.z == 0.3
        h.reach()
        assert not result.done(), "native arrival is not visual confirmation"
        for stamp in (100.1, 100.7):
            h.observe(stamp)
            h.await_request()
            h.reply("match")
        assert json.loads(await asyncio.wait_for(result, 1.0))["success"] is True
    asyncio.run(navigate())
    agent.stop()


def test_new_view_retains_budget_and_waits_for_native_arrival(harness):
    h = harness
    h.reach()
    state = h.module._verification
    deadline = state.deadline
    first_task = state.task_id
    inconclusive_view(h, 100.1)
    assert h.goal_sent.wait(1.0)
    assert len(h.goals) == 2
    goal = h.goals[-1]
    assert goal["acceptance_radius_m"] == 0.15
    assert h.modules["nav.commands"].goals[-1]["acceptance_radius_m"] == 0.15
    assert math.hypot(goal["x"] - h.goals[0]["x"], goal["y"] - h.goals[0]["y"]) >= 0.30
    assert state.state == "repositioning", h.module._active_goal.to_dict()
    assert state.task_id != first_task
    assert state.deadline == deadline
    observe(h, 102.0)
    assert len(h.requests) == 3
    h.reach()
    assert h.module._verification is state
    assert state.attempts == 3 and state.view_attempts == 0
    for stamp in (102.1, 102.7):
        observe(h, stamp)
        h.await_request()
        h.reply("match")
    assert h.statuses[-1] == "COMPLETED"
    assert len(state.evidence) == 5
    assert {item["view_index"] for item in state.evidence} == {0, 1}
    assert state.deadline == deadline


def test_three_views_and_nine_calls_are_the_per_object_limit(harness):
    h = harness
    h.reach()
    state = h.module._verification
    for index in range(3):
        inconclusive_view(h, 100.1 + 3 * index)
        if index < 2:
            assert h.goal_sent.wait(1.0)
            observe(h, 102.0 + 3 * index)
            h.reach()
    assert h.completed.wait(1.0)
    assert h.statuses[-1] == "TARGET_UNCONFIRMED"
    assert state.reason == "observation_view_budget_exhausted"
    assert state.attempts == 9
    assert len(state.viewpoints) == len(h.goals) == 3
    observe(h, 110.0)
    assert len(h.requests) == 9


def test_mismatch_skips_old_candidate_and_stops_after_three_objects(harness):
    h = harness
    alternate_resolver(h)
    for index in range(3):
        h.reach()
        state = h.module._verification
        observe(h, 100.1 + index, candidates=4)
        h.await_request()
        h.goal_sent.clear()
        h.reply("mismatch")
        assert state.state == "mismatch"
        if index < 2:
            assert h.goal_sent.wait(1.0)
            assert h.module._active_goal.target.object_id == f"chair-{index + 2}"
            assert h.module._active_goal.instruction == "find the red chair"
            assert json.loads(h.module.get_planner_status())["state"] == "NAVIGATION_ACCEPTED"
    assert len(h.goals) == 3
    assert h.module._object_candidates == {f"chair-{i}": "mismatch" for i in range(1, 4)}
    history = json.loads(h.module.get_planner_status())["object_verifications"]
    assert set(history) == {f"chair-{i}" for i in range(1, 4)}
    assert all(result["evidence"][0]["verdict"] == "mismatch" for result in history.values())
    assert h.statuses[-1] == "TARGET_MISMATCH"
    assert "COMPLETED" not in h.statuses


def test_concrete_goal_resolver_selects_an_alternative_from_filtered_scene(harness, tmp_path):
    h = harness
    h.module._goal_resolver = GoalResolver(primary_config=LLMConfig(backend="mock"), save_dir=str(tmp_path))
    h.reach()
    observe(h, 100.1, candidates=2)
    h.await_request()
    h.goal_sent.clear()
    h.reply("mismatch")
    assert h.goal_sent.wait(1.0)
    assert h.module._active_goal.target.object_id == "chair-2"
    assert h.module._active_goal.instruction == "find the red chair"


def test_excluded_candidates_remain_available_as_relational_landmarks(tmp_path):
    resolver = GoalResolver(primary_config=LLMConfig(backend="mock"), save_dir=str(tmp_path))
    scene = {"frame_id": "map", "objects": [
        {"id": "chair", "label": "chair", "confidence": 0.99, "position": {"x": 1, "y": 0, "z": 0}},
        {"id": "table", "label": "table", "confidence": 0.99, "position": {"x": 2, "y": 0, "z": 0}},
    ], "relations": [{"subject_id": "chair", "object_id": "table", "relation": "near"}]}
    result = resolver.fast_resolve("find the chair near the table", json.dumps(scene), excluded_object_ids={"table"})
    assert result.candidate_id == "chair"
    scene["relations"] = []
    without_relation = resolver.fast_resolve("find the chair near the table", json.dumps(scene), excluded_object_ids={"table"})
    assert result.confidence > without_relation.confidence


def test_blocked_alternative_is_skipped_within_same_candidate_limit(harness):
    h = harness
    alternate_resolver(h)
    native_preview = h.modules["nav.commands"].preview_plan
    checked = []

    def preview(x, y, z):
        checked.append((x, y, z))
        return {"feasible": False} if x < 3.51 else native_preview(x, y, z)

    h.modules["nav.commands"].preview_plan = preview
    h.reach()
    observe(h, 100.1, candidates=4)
    h.await_request()
    h.goal_sent.clear()
    h.reply("mismatch")
    assert h.goal_sent.wait(1.0)
    assert h.module._active_goal.target.object_id == "chair-3"
    assert len(checked) > 8
    assert h.module._object_candidates["chair-2"] == "path_blocked"
    assert len(h.module._object_candidates) == 3
    assert len(h.goals) == 2


@pytest.mark.parametrize("change", ["stop", "instruction", "map", "restart", "timeout"])
@pytest.mark.parametrize("alternative", [False, True])
def test_delayed_view_preview_cannot_dispatch_after_invalidation(harness, change, alternative):
    h = harness
    entered, release, finished = threading.Event(), threading.Event(), threading.Event()
    preview = h.modules["nav.commands"].preview_plan
    select = h.module._select_observation_goal

    def delayed(*args):
        entered.set()
        assert release.wait(2.0)
        return preview(*args)

    def selecting(*args):
        try:
            select(*args)
        finally:
            finished.set()

    h.modules["nav.commands"].preview_plan = delayed
    h.module._select_observation_goal = selecting
    h.reach()
    if alternative:
        alternate_resolver(h)
        observe(h, 100.1, candidates=2)
        h.await_request()
        h.reply("mismatch")
    else:
        inconclusive_view(h, 100.1)
    assert entered.wait(1.0)
    state = h.module._verification
    try:
        if change == "stop":
            h.module.stop()
        elif change == "instruction":
            h.module.instruction._deliver("follow person")
        elif change == "timeout":
            state.deadline = time.monotonic() - 1.0
            h.module._expire_target_verification(state)
        else:
            h.module.navigation_state._deliver(NavigationState(
                boot_id="other" if change == "restart" else "test-nav", sequence=2,
                map_id="other" if change == "map" else "map-a"))
    finally:
        release.set()
        assert finished.wait(1.0)
    assert len(h.goals) == 1
    assert state.terminal


def test_deadline_during_motion_requests_cancel_and_preserves_timeout_outcome(harness):
    h = harness
    commands = []
    h.module.nav_command.subscribe(lambda raw: commands.append(json.loads(raw)))
    h.reach()
    inconclusive_view(h, 100.1)
    assert h.goal_sent.wait(1.0)
    state = h.module._verification
    state.deadline = time.monotonic() - 1.0
    h.module._expire_target_verification(state)
    assert commands[-1]["action"] == "cancel"
    assert commands[-1]["task_id"] == h.module._active_goal.task_id
    assert h.module._active_goal.cancel_request_id
    assert not h.module._active_goal.terminal
    h.reach(NavigationGoalState.CANCELLED)
    assert h.statuses[-1] == "TARGET_VERIFICATION_TIMEOUT"
    observe(h, 103.0)
    assert len(h.goals) == 2 and len(h.requests) == 3


def test_native_failure_during_reobservation_does_not_start_unbounded_lera(harness):
    h = harness
    h.reach()
    inconclusive_view(h, 100.1)
    assert h.goal_sent.wait(1.0)
    h.reach(NavigationGoalState.FAILED)
    assert h.statuses[-1] == "TARGET_UNCONFIRMED"
    assert h.module._verification.reason == "observation_navigation_failed"
    assert len(h.goals) == 2
    assert h.module._failure_count == 0


def test_busy_native_planner_retries_without_renewing_verification_deadline(harness):
    h = harness
    busy = threading.Event()
    preview = h.modules["nav.commands"].preview_plan

    def temporarily_busy(*args):
        busy.set()
        return {"feasible": False, "reason": "planner_busy"}

    h.modules["nav.commands"].preview_plan = temporarily_busy
    h.reach()
    state = h.module._verification
    deadline = state.deadline
    inconclusive_view(h, 100.1)
    assert busy.wait(1.0)
    assert len(h.goals) == 1
    h.modules["nav.commands"].preview_plan = preview
    h.module._observation_retry_after = 0.0
    observe(h, 102.0)
    assert h.goal_sent.wait(1.0)
    assert state.deadline == deadline
    assert len(h.goals) == 2


@pytest.mark.parametrize("admission", ["rejected", "unconfirmed"])
def test_failed_view_admission_never_produces_another_motion_retry(harness, admission):
    h = harness

    def send_goal(**kwargs):
        if admission == "unconfirmed":
            raise TimeoutError("receipt was lost")
        return NavigationCommandReceipt(accepted=False, kind=NavigationCommandKind.GOAL,
                                        task_id=kwargs["task_id"], request_id=kwargs["request_id"],
                                        reason="test rejection", endpoint_timestamp_s=100.0)

    h.modules["nav.commands"].send_goal = send_goal
    h.reach()
    inconclusive_view(h, 100.1)
    assert h.goal_sent.wait(1.0)
    assert h.module._verification.state == "unavailable"
    if admission == "unconfirmed":
        assert h.module._active_goal.cancel_request_id
        h.reach(NavigationGoalState.CANCELLED)
    else:
        assert h.module._active_goal.terminal
    observe(h, 103.0)
    assert len(h.goals) == 2
    assert len(h.requests) == 3


def test_alternative_waits_for_native_planner_then_continues_same_candidate(harness):
    h = harness
    alternate_resolver(h)
    finished = threading.Event()
    select = h.module._select_observation_goal
    preview = h.modules["nav.commands"].preview_plan

    def selecting(*args):
        try:
            select(*args)
        finally:
            finished.set()

    h.module._select_observation_goal = selecting
    h.modules["nav.commands"].preview_plan = lambda *args: {"reason": "navigation_busy"}
    h.reach()
    observe(h, 100.1, candidates=2)
    h.await_request()
    h.goal_sent.clear()
    h.reply("mismatch")
    assert finished.wait(1.0)
    assert h.module._object_candidates["chair-2"] == "waiting_path"
    assert json.loads(h.module.get_planner_status())["state"] == "OBSERVATION_PLANNER_WAITING"
    assert len(h.goals) == 1
    h.modules["nav.commands"].preview_plan = preview
    h.module._observation_retry_after = 0.0
    observe(h, 100.7, candidates=2)
    assert h.goal_sent.wait(1.0)
    assert len(h.goals) == 2
    assert len(h.module._object_candidates) == 2
    assert h.module._active_goal.target.object_id == "chair-2"


def test_alternative_preview_wait_expires_without_another_camera_frame(harness):
    h = harness
    alternate_resolver(h)
    h.module._verification_timeout_s = 0.2
    finished = threading.Event()
    select = h.module._select_observation_goal

    def selecting(*args):
        try:
            select(*args)
        finally:
            finished.set()

    h.module._select_observation_goal = selecting
    h.modules["nav.commands"].preview_plan = lambda *args: {"reason": "navigation_busy"}
    h.reach()
    observe(h, 100.1, candidates=2)
    h.await_request()
    h.reply("mismatch")
    assert finished.wait(0.1)
    assert h.module._object_candidates["chair-2"] == "waiting_path"
    h.completed.clear()
    assert h.completed.wait(1.0)
    assert h.module._object_candidates["chair-2"] == "path_wait_expired"
    assert json.loads(h.module.get_planner_status())["state"] == "TARGET_MISMATCH"
    assert len(h.goals) == 1
