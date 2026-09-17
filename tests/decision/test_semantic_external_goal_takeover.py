"""A goal accepted through the operator API supersedes pending semantic work."""

import asyncio
import threading
from types import SimpleNamespace

import pytest

from gateway.navigation.commands import submit_goal
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.nav import NavigationCommandKind, NavigationCommandReceipt
from tests.decision.test_agent_task_lifecycle import agent, tool
from tests.decision.test_semantic_target_verification import harness


@pytest.mark.parametrize("already_moved", [False, True])
def test_operator_goal_prevents_late_agent_motion(agent, already_moved):
    entered, release = threading.Event(), threading.Event()
    calls = 0

    async def chat(messages, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 1 and already_moved:
            return tool("navigate_to", x=1, y=2, z=0.3)
        if calls > (2 if already_moved else 1):
            return tool("done", summary="done")
        entered.set()
        while not release.is_set():
            await asyncio.sleep(0.005)
        return tool("navigate_to", x=9, y=9, z=0.3)

    agent.module._llm_client = SimpleNamespace(chat_with_tools=chat)
    h = agent.h
    h.goal_sent.clear()
    agent.module.run_agent_task("visit coordinates")
    run = agent.module._active_run
    try:
        if already_moved:
            assert h.goal_sent.wait(1)
            h.reach()
        assert entered.wait(1)
        assert submit_goal(
            SimpleNamespace(_goals=h.modules["nav.goals"]),
            PoseStamped(Pose(Vector3(4, 5, 0.3)), frame_id="map"),
            task_id="operator-task", request_id="operator-request",
        )["accepted"]
        count = len(h.goals)
        release.set()
        run.future.result(timeout=1)
        assert len(h.goals) == count
        assert agent.statuses[-1] == "AGENT_CANCELLED"
        assert all(command["task_id"] != "operator-task" for command in agent.cancels)
    finally:
        release.set()


@pytest.mark.parametrize("kind", ["rejected", "replay", "visual_servo"])
def test_non_takeover_goal_ack_preserves_semantic_owner(harness, kind):
    h = harness
    module = h.module
    if kind == "replay":
        pose = PoseStamped(Pose(Vector3(4, 5, 0.3)), frame_id="map")
        h.modules["nav.goals"].submit_goal(pose, task_id="old-task", request_id="old-request", action="goal")
        module.submit_owned_instruction("find the red chair", "agent-owner")
        revision = module.instruction_revision()
        h.modules["nav.goals"].submit_goal(pose, task_id="old-task", request_id="old-request", action="goal")
    else:
        module.submit_owned_instruction("find the red chair", "agent-owner")
        revision = module.instruction_revision()
        if kind == "rejected":
            h.modules["nav.commands"].send_goal = lambda *args, **kwargs: NavigationCommandReceipt(
                accepted=False, kind=NavigationCommandKind.GOAL, task_id=kwargs["task_id"],
                request_id=kwargs["request_id"], reason="blocked", endpoint_timestamp_s=100.0,
            )
        else:
            module._visual_handoff = True
        h.modules["nav.goals"].submit_goal(
            PoseStamped(Pose(Vector3(4, 5, 0.3)), frame_id="map"),
            task_id="other-task", request_id="other-request",
            action="visual_servo" if kind == "visual_servo" else "goal",
        )
    assert module.instruction_revision() == revision
    assert module.owned_instruction_status("agent-owner")["owned"] is True


def test_operator_goal_invalidates_in_flight_observation_preview(harness, monkeypatch):
    h = harness
    entered, release, returned = threading.Event(), threading.Event(), threading.Event()

    def preview(x, y, z):
        entered.set()
        assert release.wait(2)
        return {"feasible": True, "start_valid": True, "frame_id": "map", "path": [{"x": x, "y": y, "z": z}]}

    select = h.module._select_observation_goal

    def select_and_signal(*args):
        try:
            select(*args)
        finally:
            returned.set()

    monkeypatch.setattr(h.module, "_select_observation_goal", select_and_signal)
    h.modules["nav.commands"].preview_plan = preview
    h.module.submit_owned_instruction("find the red chair", "agent-owner")
    try:
        assert entered.wait(1)
        submit_goal(
            SimpleNamespace(_goals=h.modules["nav.goals"]),
            PoseStamped(Pose(Vector3(4, 5, 0.3)), frame_id="map"),
            task_id="operator-task", request_id="operator-request",
        )
        count = len(h.goals)
        release.set()
        assert returned.wait(1)
        assert h.module._pending_observation is None
        assert h.module.owned_instruction_status("agent-owner")["state"] == "SUPERSEDED"
        assert len(h.goals) == count
    finally:
        release.set()
