"""Agent motion tools must preserve map height and semantic goal ownership."""

import asyncio
import json
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from decision.modules.agent_planner import AgentPlannerModule
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.nav import NavigationLifecycle, NavigationState


@pytest.fixture
def agent(monkeypatch):
    monkeypatch.setattr(AgentPlannerModule, "_init_llm", lambda self: None)
    module = AgentPlannerModule(llm_backend="mock")
    module.setup()
    goals, servo = [], []
    completed = {"owned": True, "terminal": True, "success": True, "state": "reached"}
    planner = SimpleNamespace(
        instruction_revision=lambda: 1,
        submit_owned_pose=lambda pose, instruction, owner, **kwargs: goals.append(pose) or completed,
        submit_owned_instruction=Mock(return_value=completed),
        owned_instruction_status=lambda owner: completed,
        cancel_owned_instruction=Mock(return_value=True),
    )
    module.on_system_modules({"SemanticPlannerModule": planner})
    module._new_run("test")
    yield SimpleNamespace(module=module, goals=goals, servo=servo, planner=planner)
    module.stop()


def test_explicit_height_matches_the_advertised_navigation_tool(agent):
    asyncio.run(agent.module._tool_navigate_to(1.0, 2.0, z=1.7, yaw=0.4))
    assert len(agent.goals) == 1
    assert agent.goals[0].z == 1.7
    assert agent.goals[0].yaw == pytest.approx(0.4)


def test_omitted_height_uses_fresh_map_pose_not_zero(agent):
    agent.module.robot_pose._deliver(PoseStamped(Pose(Vector3(0, 0, 1.3)), frame_id="map", ts=time.time()))
    asyncio.run(agent.module._tool_navigate_to(1.0, 2.0))
    assert agent.goals[0].z == 1.3


@pytest.mark.parametrize("pose", [None, PoseStamped(Pose(Vector3(0, 0, 1.3)), frame_id="odom"),
                                  PoseStamped(Pose(Vector3(0, 0, 1.3)), frame_id="map", ts=1.0)])
def test_unknown_map_height_does_not_send_an_invented_floor(agent, pose):
    if pose is not None:
        agent.module.robot_pose._deliver(pose)
    assert "map pose" in asyncio.run(agent.module._tool_navigate_to(1.0, 2.0)).lower()
    assert agent.goals == []


def test_map_change_invalidates_cached_floor(agent):
    agent.module.navigation_state._deliver(NavigationState(boot_id="nav", map_id="a", sequence=1))
    agent.module.robot_pose._deliver(PoseStamped(Pose(Vector3(0, 0, 1.3)), frame_id="map", ts=time.time()))
    agent.module.navigation_state._deliver(NavigationState(boot_id="nav", map_id="b", sequence=2))
    agent.module._new_run("after map change")
    asyncio.run(agent.module._tool_navigate_to(1.0, 2.0))
    assert agent.goals == []


def test_object_tool_delegates_complete_description_to_semantic_planner(agent):
    result = asyncio.run(agent.module._tool_navigate_to_object("red chair near the door"))
    agent.planner.submit_owned_instruction.assert_called_once_with(
        "find red chair near the door", agent.module._active_run.run_id, expected_revision=1)
    assert json.loads(result)["success"] is True
    assert agent.goals == agent.servo == []


def test_object_tool_without_semantic_planner_does_not_bypass_preview(agent):
    agent.module.on_system_modules({})
    assert "unavailable" in asyncio.run(agent.module._tool_navigate_to_object("chair")).lower()
    assert agent.goals == agent.servo == []


@pytest.mark.parametrize("failure", ["model_error", "empty_response", "max_steps", "timeout"])
def test_agent_failure_cannot_be_reported_as_done(agent, failure):
    statuses = []
    agent.module.planner_status.subscribe(statuses.append)

    async def reply(*args, **kwargs):
        if failure == "model_error":
            raise RuntimeError("provider unavailable")
        return {} if failure == "empty_response" else {"content": "still thinking"}

    agent.module._llm_client = SimpleNamespace(chat_with_tools=reply)
    agent.module._max_steps = 1
    agent.module._timeout = -1 if failure == "timeout" else 30
    agent.module._run_agent_loop_sync("inspect")
    assert statuses[-1] == "AGENT_FAILED"
    assert "AGENT_DONE" not in statuses


def test_model_reply_after_deadline_cannot_issue_a_navigation_goal(agent):
    statuses = []
    agent.module.planner_status.subscribe(statuses.append)

    async def reply(*args, **kwargs):
        await asyncio.sleep(0.05)
        return {"tool_calls": [{"function": {"name": "navigate_to", "arguments": '{"x":1,"y":2,"z":0.3}'},
                                "id": "late"}]}

    agent.module._llm_client = SimpleNamespace(chat_with_tools=reply)
    agent.module._timeout = 0.01
    agent.module._run_agent_loop_sync("inspect")
    assert agent.goals == []
    assert statuses[-1] == "AGENT_FAILED"


@pytest.mark.parametrize("state", [NavigationLifecycle.RECOVERING, NavigationLifecycle.FAILED])
def test_native_failure_and_recovery_remain_authoritative_in_agent_context(agent, state):
    agent.module.navigation_state._deliver(NavigationState(
        boot_id="nav", map_id="a", sequence=2, lifecycle_state=int(state)))
    agent.module.mission_status._deliver({"state": "EXECUTING"})
    agent.module.navigation_state._deliver(NavigationState(
        boot_id="nav", map_id="a", sequence=1, lifecycle_state=int(NavigationLifecycle.PLANNING)))
    assert agent.module._agent_context()["nav_status"] == state.name
