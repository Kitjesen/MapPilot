"""Late model replies and native motion cross real Agent/Semantic/GoalService owners."""

import asyncio
import concurrent.futures
import json
import threading
from types import SimpleNamespace

import pytest

from decision.modules.agent_planner import AgentPlannerModule
from runtime.msgs.nav import (
    NavigationCommandKind,
    NavigationCommandReceipt,
    NavigationGoalState,
    NavigationGoalStatus,
    NavigationState,
)
from tests.decision.test_semantic_target_verification import harness


def tool(name, **args):
    if name == "done":
        args.setdefault("success", True)
    return {"tool_calls": [{"id": name, "function": {"name": name, "arguments": json.dumps(args)}}]}


@pytest.fixture
def agent(harness, monkeypatch):
    h = harness
    monkeypatch.setattr(AgentPlannerModule, "_init_llm", lambda self: None)
    module = AgentPlannerModule(llm_backend="mock", timeout=3)
    module.setup()
    module.on_system_modules({"SemanticPlannerModule": h.module})
    module.navigation_state._deliver(NavigationState(boot_id="test-nav", map_id="map-a", sequence=1))
    statuses, cancels, chats = [], [], []
    module.planner_status.subscribe(statuses.append)
    module.agent_message.subscribe(chats.append)
    h.module.nav_command.subscribe(lambda raw: cancels.append(json.loads(raw))
                                   if json.loads(raw)["action"] == "cancel" else None)
    yield SimpleNamespace(module=module, h=h, statuses=statuses, cancels=cancels, chats=chats)
    module.stop()


def script(agent, *responses):
    remaining = iter(responses)
    calls = []

    async def chat(messages, **kwargs):
        calls.append(messages)
        return next(remaining, tool("done", summary="complete"))

    agent.module._llm_client = SimpleNamespace(chat_with_tools=chat)
    return calls


def test_coordinate_tool_waits_for_correlated_native_arrival_before_done(agent):
    calls = script(agent, tool("navigate_to", x=1, y=2, z=0.3))
    h = agent.h
    h.goal_sent.clear()
    agent.module.run_agent_task("visit coordinate")
    run = agent.module._active_run
    assert h.goal_sent.wait(1)
    assert len(calls) == 1
    assert not run.future.done()
    assert "AGENT_DONE" not in agent.statuses
    h.reach()
    run.future.result(timeout=2)
    assert len(calls) == 2
    assert agent.statuses[-1] == "AGENT_DONE"
    assert run.motion_success


@pytest.mark.parametrize("change", ["replace", "map", "restart", "cancel", "stop"])
def test_late_model_reply_cannot_issue_motion_or_finish_old_run(agent, change):
    entered, release, cancelled, returned = (threading.Event() for _ in range(4))
    async def chat(messages, **kwargs):
        if messages[1]["content"] != "old instruction":
            return tool("done", summary="new instruction complete")
        entered.set()
        while not release.is_set():
            try:
                await asyncio.sleep(0.005)
            except asyncio.CancelledError:
                # Even a provider that returns after cancellation must not resurrect a task.
                cancelled.set()
        returned.set()
        return tool("navigate_to", x=9, y=9, z=0.3)

    agent.module._llm_client = SimpleNamespace(chat_with_tools=chat)
    before = len(agent.h.goals)
    agent.module.run_agent_task("old instruction")
    old = agent.module._active_run
    assert entered.wait(1)
    stopping = None
    try:
        if change == "replace":
            agent.module.run_agent_task("new instruction")
            agent.module._active_run.future.result(timeout=1)
        elif change in {"map", "restart"}:
            agent.module.navigation_state._deliver(NavigationState(
                boot_id="other-nav" if change == "restart" else "test-nav", sequence=2,
                map_id="map-b" if change == "map" else "map-a"))
        elif change == "cancel":
            response = json.loads(agent.module.cancel_agent_task(old.run_id))
            assert response == {"cancel_requested": True, "stop_confirmed": False}
        else:
            stopping = threading.Thread(target=agent.module.stop)
            stopping.start()
        assert cancelled.wait(1)
        release.set()
        assert returned.wait(1)
        if stopping is not None:
            stopping.join(2)
            assert not stopping.is_alive()
        with pytest.raises(concurrent.futures.CancelledError):
            old.future.result(timeout=1)
        assert len(agent.h.goals) == before
        assert not any("old instruction" in chat["text"] and chat["phase"] == "agent_done"
                       for chat in agent.chats)
        if change == "replace":
            assert agent.statuses[-1] == "AGENT_DONE"
            assert agent.module._active_run.state == "done"
    finally:
        release.set()
        if stopping is not None:
            stopping.join(2)


def test_task_timeout_requests_cancellation_of_its_actual_native_goal(agent):
    script(agent, tool("navigate_to", x=1, y=2, z=0.3))
    agent.module._timeout = 0.1
    agent.h.goal_sent.clear()
    agent.module.run_agent_task("visit coordinate")
    run = agent.module._active_run
    assert agent.h.goal_sent.wait(1)
    task_id = agent.h.goals[-1]["task_id"]
    run.future.result(timeout=2)
    assert agent.statuses[-1] == "AGENT_FAILED"
    assert any(command["task_id"] == task_id for command in agent.cancels)
    before = list(agent.statuses)
    agent.h.reach()
    assert agent.statuses == before
    assert "AGENT_DONE" not in agent.statuses


def test_manual_semantic_instruction_cannot_be_cancelled_by_old_agent(agent):
    script(agent, tool("navigate_to", x=1, y=2, z=0.3))
    h = agent.h
    h.goal_sent.clear()
    agent.module.run_agent_task("visit coordinate")
    old = agent.module._active_run
    assert h.goal_sent.wait(1)
    h.goal_sent.clear()
    h.module.send_instruction("find the red chair")
    assert h.goal_sent.wait(1)
    manual = h.module._active_goal
    old.future.result(timeout=2)
    assert agent.statuses[-1] == "AGENT_CANCELLED"
    assert not manual.cancel_request_id
    agent.module.cancel_agent_task(old.run_id)
    assert not manual.cancel_request_id
    assert all(command["task_id"] != manual.task_id for command in agent.cancels)


def test_stale_cancel_id_does_not_cancel_new_agent_run(agent):
    script(agent, tool("done", summary="first complete"))
    old_id = json.loads(agent.module.run_agent_task("first"))["run_id"]
    agent.module._active_run.future.result(timeout=1)
    script(agent, tool("navigate_to", x=1, y=2, z=0.3))
    agent.h.goal_sent.clear()
    agent.module.run_agent_task("second")
    new = agent.module._active_run
    assert agent.h.goal_sent.wait(1)
    assert json.loads(agent.module.cancel_agent_task(old_id))["cancel_requested"] is False
    assert not new.cancelled
    assert not agent.h.module._active_goal.cancel_request_id


def test_rejected_native_goal_cannot_be_reported_done_by_model(agent):
    agent.h.modules["nav.commands"].send_goal = lambda *args, **kwargs: NavigationCommandReceipt(
        accepted=False, kind=NavigationCommandKind.GOAL, task_id=kwargs["task_id"],
        request_id=kwargs["request_id"], reason="blocked", endpoint_timestamp_s=100.0)
    script(agent, tool("navigate_to", x=1, y=2, z=0.3))
    agent.module._max_steps = 2
    agent.module.run_agent_task("visit coordinate")
    agent.module._active_run.future.result(timeout=2)
    assert agent.statuses[-1] == "AGENT_FAILED"
    assert "AGENT_DONE" not in agent.statuses


def test_model_reporting_incomplete_task_is_not_reported_success(agent):
    script(agent, tool("done", summary="The requested object could not be found", success=False))
    agent.module.run_agent_task("find the missing chair")
    run = agent.module._active_run
    state = run.future.result(timeout=2)
    assert state.failure_reason == "task_incomplete"
    assert run.state == "failed"
    assert agent.statuses[-1] == "AGENT_FAILED"
    assert "AGENT_DONE" not in agent.statuses


@pytest.mark.parametrize("motion_before_takeover", [False, True])
def test_manual_takeover_while_model_pending_prevents_next_motion(agent, motion_before_takeover):
    entered, release = threading.Event(), threading.Event()
    calls = 0

    async def chat(messages, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 1 and motion_before_takeover:
            return tool("navigate_to", x=1, y=2, z=0.3)
        entered.set()
        while not release.is_set():
            await asyncio.sleep(0.005)
        return tool("navigate_to", x=9, y=9, z=0.3)

    agent.module._llm_client = SimpleNamespace(chat_with_tools=chat)
    h = agent.h
    h.goal_sent.clear()
    agent.module.run_agent_task("visit two coordinates")
    run = agent.module._active_run
    try:
        if motion_before_takeover:
            assert h.goal_sent.wait(1)
            h.reach()
        assert entered.wait(1)
        h.goal_sent.clear()
        h.module.send_instruction("find the red chair")
        assert h.goal_sent.wait(1)
        manual = h.module._active_goal
        count = len(h.goals)
        release.set()
        run.future.result(timeout=2)
        assert len(h.goals) == count
        assert not manual.cancel_request_id
        assert agent.statuses[-1] == "AGENT_CANCELLED"
    finally:
        release.set()


def test_motion_aliases_use_the_same_owned_navigation(agent):
    from decision.modules.visual_servo import VisualServoModule
    from decision.modules.vla import VLAModule
    from memory.modules.tagged_locations_module import TaggedLocationsModule
    from nav.skills.skills_module import NavSkills

    agent.module.on_system_modules({"SemanticPlannerModule": agent.h.module,
                                   "nav.skills": NavSkills(), "VisualServoModule": VisualServoModule(),
                                   "VLAModule": VLAModule(), "TaggedLocationsModule": TaggedLocationsModule()})
    for name in ("navigate_to_deg", "find_object", "follow_person", "stop_servo", "vla_navigate", "go_to_tag"):
        assert name not in agent.module._agent_tool_registry
    assert "get_vla_status" in agent.module._agent_tool_registry
    script(agent, tool("navigate_to_deg", x=1, y=2, z=0.3, yaw_deg=90))
    agent.h.goal_sent.clear()
    agent.module.run_agent_task("visit coordinate")
    run = agent.module._active_run
    assert agent.h.goal_sent.wait(1)
    assert agent.h.goals[-1]["yaw"] == pytest.approx(1.57079632679)
    agent.h.reach()
    run.future.result(timeout=2)
    assert agent.statuses[-1] == "AGENT_DONE"


def test_owned_follow_handoff_stops_on_agent_cancel(agent):
    servo, sent = [], threading.Event()
    agent.h.module.servo_target.subscribe(lambda command: (servo.append(command), sent.set()))
    script(agent, tool("follow_person", description="person"))
    agent.module.run_agent_task("follow person")
    run = agent.module._active_run
    assert sent.wait(1)
    assert servo[0].startswith("follow:")
    agent.module.cancel_agent_task(run.run_id)
    assert servo[-1] == "stop"
    assert "AGENT_DONE" not in agent.statuses


def test_replacing_an_executing_agent_cancels_only_its_old_task(agent):
    h = agent.h
    script(agent, tool("navigate_to", x=1, y=2, z=0.3))
    h.goal_sent.clear()
    agent.module.run_agent_task("first")
    old_run = agent.module._active_run
    assert h.goal_sent.wait(1)
    old_goal = h.goals[-1]
    script(agent, tool("navigate_to", x=4, y=5, z=0.3))
    h.goal_sent.clear()
    agent.module.run_agent_task("second")
    new_run = agent.module._active_run
    assert h.goal_sent.wait(1)
    new_goal = h.goals[-1]
    assert any(command["task_id"] == old_goal["task_id"] for command in agent.cancels)
    assert all(command["task_id"] != new_goal["task_id"] for command in agent.cancels)
    for sequence, goal in enumerate((old_goal, new_goal), start=1):
        h.modules["nav.goals"].navigation_goal_status._deliver(NavigationGoalStatus(
            ts=h.clock.now, frame_id="map", boot_id="test-nav", sequence=sequence, goal_epoch=sequence,
            task_id=goal["task_id"], request_id=goal["request_id"], state=NavigationGoalState.REACHED))
        if sequence == 1:
            assert not new_run.future.done()
            assert "AGENT_DONE" not in agent.statuses
    new_run.future.result(timeout=2)
    assert old_run.cancelled
    assert agent.statuses[-1] == "AGENT_DONE"


def test_repeated_agent_runs_keep_one_model_event_loop(agent):
    loops = []
    async def chat(*args, **kwargs):
        loops.append(asyncio.get_running_loop())
        return tool("done", summary="greeting complete")
    agent.module._llm_client = SimpleNamespace(chat_with_tools=chat)
    for instruction in ("first greeting", "second greeting"):
        agent.module.run_agent_task(instruction)
        agent.module._active_run.future.result(timeout=1)
    assert len(loops) == 2 and loops[0] is loops[1]
    assert not loops[0].is_closed()
    agent.module.stop()
    assert loops[0].is_closed()


def test_agent_waits_through_candidate_change_and_visual_confirmation(agent):
    from runtime.msgs.geometry import Vector3
    from runtime.msgs.semantic import Detection3D

    h = agent.h
    def resolve(instruction, scene, **kwargs):
        objects = [obj for obj in json.loads(scene)["objects"] if obj["id"] not in kwargs.get("excluded_object_ids", set())]
        return SimpleNamespace(confidence=1.0, candidate_id=objects[0]["id"], action="navigate")
    h.module._goal_resolver.fast_resolve.side_effect = resolve
    calls = script(agent, tool("navigate_to_object", label="red chair"))
    h.goal_sent.clear()
    agent.module.run_agent_task("find the red chair")
    run = agent.module._active_run
    assert h.goal_sent.wait(1)
    h.reach()
    h.observe(100.1, extra_objects=[Detection3D(id="chair-2", label="chair", confidence=0.95,
                                             position=Vector3(3, 0, 1), ts=100.1, bbox_2d=[4, 4, 28, 28])])
    h.await_request()
    h.goal_sent.clear()
    h.reply("mismatch")
    assert h.goal_sent.wait(1)
    assert len(calls) == 1
    assert not run.future.done()
    target = h.module._active_goal
    assert target.target.object_id == "chair-2"
    h.reach()
    for stamp in (100.2, 100.8):
        h.observe(stamp, object_id="chair-2", object_x=3,
                  robot_position=(target.pose.x, target.pose.y, target.pose.z))
        h.await_request()
        h.reply("match")
    run.future.result(timeout=2)
    status = json.loads(agent.module.get_agent_status())
    assert status["state"] == "done"
    assert status["motion_success"]
    assert status["navigation"]["confirmations"] == 2
    assert status["navigation"]["task_id"] == target.task_id


def test_blank_or_closed_submission_does_not_return_an_old_run_identity(agent):
    script(agent, tool("done", summary="complete"))
    agent.module.run_agent_task("first")
    agent.module._active_run.future.result(timeout=1)
    assert json.loads(agent.module.run_agent_task(" ")) == {"run_id": "", "state": "unavailable"}
    agent.module.stop()
    assert json.loads(agent.module.run_agent_task("another")) == {"run_id": "", "state": "unavailable"}
