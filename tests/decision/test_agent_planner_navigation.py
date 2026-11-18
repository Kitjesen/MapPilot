"""Agent motion tools must preserve map height and semantic goal ownership."""

import asyncio
import json
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from decision.modules.agent_planner import AgentPlannerModule
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.nav import (
    InspectionTaskEvent,
    InspectionTaskEventKind,
    InspectionTaskState,
    NavigationGoalState,
    NavigationGoalStatus,
    NavigationLifecycle,
    NavigationState,
)


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


def test_single_target_find_preserves_the_users_original_instruction(agent):
    run = agent.module._new_run("帮我找一瓶水")

    result = asyncio.run(agent.module._tool_find_object("chair", run=run))

    agent.planner.submit_owned_instruction.assert_called_once_with(
        "帮我找一瓶水", run.run_id, expected_revision=1,
    )
    assert json.loads(result)["success"] is True
    assert run.requires_object_confirmation is True
    assert run.object_confirmed is True


def test_follow_selection_does_not_use_object_search_confirmation_gate(agent):
    run = agent.module._new_run("跟随我")

    assert run.requires_execution_evidence is True
    assert run.requires_object_confirmation is False
    assert run.object_query_instruction == ""


def test_conversation_follow_returns_after_follow_admission_without_semantic_planner(agent):
    from decision.modules.visual_servo import VisualServoModule

    servo = VisualServoModule(product_session_id="tracking-session")
    agent.module.on_system_modules({"VisualServoModule": servo})
    run = agent.module._active_run
    result = asyncio.run(agent.module._tool_start_follow_person("person in red", run=run))
    payload = json.loads(result)
    assert payload["accepted"] is True and payload["phase"] == "selecting"
    assert payload["execution_confirmed"] is False
    assert payload["run_id"] == run.run_id
    assert run.follow_ref == {"follow_task_id": payload["follow_task_id"], "product_session_id": "tracking-session"}
    assert run.persistent_task and run.follow_submitted and run.motion_submitted
    agent.planner.submit_owned_instruction.assert_not_called()

    again = json.loads(asyncio.run(agent.module._tool_start_follow_person("person in blue", run=run)))
    assert again["reason"] == "task_active"
    assert run.follow_ref["follow_task_id"] == payload["follow_task_id"]
    assert json.loads(asyncio.run(agent.module._submit_motion(run, instruction="find chair")))["reason"] == "task_active"


def test_domain_navigation_task_requires_terminal_status_before_completion(agent):
    run = agent.module._new_run("导航到目标点")

    AgentPlannerModule._record_domain_task(
        run,
        "start_navigation",
        {"accepted": True, "task_id": "nav-task-1"},
    )
    assert run.motion_attempted and run.motion_submitted and run.persistent_task
    assert agent.module._completion_refusal(run)

    AgentPlannerModule._record_domain_task(
        run,
        "get_navigation_status",
        {"found": True, "task_id": "nav-task-1", "status": {"terminal": False, "state_name": "EXECUTING"}},
    )
    assert run.persistent_task and not run.motion_success

    AgentPlannerModule._record_domain_task(
        run,
        "get_navigation_status",
        {"found": True, "task_id": "nav-task-1", "status": {"terminal": True, "state_name": "SUCCESS"}},
    )
    assert not run.persistent_task and run.motion_success
    assert agent.module._completion_refusal(run) is None


def test_domain_inspection_task_uses_terminal_status_for_completion(agent):
    run = agent.module._new_run("执行巡检")

    AgentPlannerModule._record_domain_task(
        run,
        "start_inspection_route",
        {"ok": True, "accepted": True, "task_id": "inspection-task-1"},
    )
    assert run.domain_task_type == "inspection" and run.persistent_task

    AgentPlannerModule._record_domain_task(
        run,
        "get_inspection_status",
        {"found": True, "task_id": "inspection-task-1", "terminal": True,
         "latest_event": {"state_name": "SUCCEEDED"}},
    )
    assert not run.persistent_task and run.motion_success


@pytest.mark.parametrize(
    ("state", "expected_state", "success"),
    [
        (NavigationGoalState.REACHED, "done", True),
        (NavigationGoalState.FAILED, "failed", False),
        (NavigationGoalState.CANCELLED, "cancelled", False),
    ],
)
def test_native_navigation_terminal_event_closes_domain_run(agent, state, expected_state, success):
    run = agent.module._new_run("导航到目标点")
    run.domain_task_id = "nav-task-1"
    run.domain_task_type = "navigation"
    run.motion_attempted = run.motion_submitted = True
    run.persistent_task = True
    messages = []
    agent.module.agent_message.subscribe(messages.append)

    status = NavigationGoalStatus(
        boot_id="nav-boot",
        sequence=1,
        task_id="nav-task-1",
        request_id="nav-request-1",
        state=int(state),
    )
    agent.module._on_navigation_goal_status(status)
    agent.module._on_navigation_goal_status(status)

    assert run.state == expected_state
    assert run.motion_success is success
    assert run.persistent_task is False
    assert run.domain_completion_reported is True
    assert len([message for message in messages if message.get("phase") == "navigation_result"]) == 1


def test_native_inspection_terminal_event_closes_domain_run(agent):
    run = agent.module._new_run("执行巡检")
    run.domain_task_id = "inspection-task-1"
    run.domain_task_type = "inspection"
    run.motion_attempted = run.motion_submitted = True
    run.persistent_task = True
    messages = []
    agent.module.agent_message.subscribe(messages.append)

    event = InspectionTaskEvent(
        boot_id="nav-boot",
        event_sequence=1,
        kind=int(InspectionTaskEventKind.STATE_CHANGED),
        task_id="inspection-task-1",
        request_id="inspection-request-1",
        state=int(InspectionTaskState.SUCCEEDED),
        map_id="field-map",
        map_content_epoch=1,
        route_id="route-1",
        route_revision=1,
    )
    agent.module._on_inspection_task_event(event)
    agent.module._on_inspection_task_event(event)

    assert run.state == "done"
    assert run.motion_success is True
    assert run.persistent_task is False
    assert len([message for message in messages if message.get("phase") == "inspection_result"]) == 1


def test_terminal_event_cached_before_navigation_receipt_is_replayed(agent):
    run = agent.module._new_run("导航到目标点")
    messages = []
    agent.module.agent_message.subscribe(messages.append)
    agent.module._on_navigation_goal_status(NavigationGoalStatus(
        boot_id="nav-boot",
        sequence=1,
        task_id="nav-task-early",
        request_id="nav-request-early",
        state=int(NavigationGoalState.REACHED),
    ))

    AgentPlannerModule._record_domain_task(
        run,
        "start_navigation",
        {"accepted": True, "task_id": "nav-task-early"},
    )
    agent.module._sync_cached_domain_task(run)

    assert run.state == "done"
    assert run.motion_success is True
    assert run.domain_completion_reported is True
    assert len([message for message in messages if message.get("phase") == "navigation_result"]) == 1


@pytest.mark.parametrize("description,loaded,reason", [
    ("person in red", False, "visual_servo_unavailable"),
    ("", True, "follow_target_required"),
])
def test_rejected_follow_never_claims_persistent_ownership(agent, description, loaded, reason):
    from decision.modules.visual_servo import VisualServoModule

    agent.module.on_system_modules({"VisualServoModule": VisualServoModule()} if loaded else {})
    run = agent.module._active_run
    receipt = json.loads(asyncio.run(agent.module._tool_start_follow_person(description, run=run)))
    assert receipt["accepted"] is False and receipt["reason"] == reason
    assert not run.persistent_task and not run.motion_submitted and run.follow_ref is None


def test_busy_follow_receipt_cannot_transfer_another_operators_task_to_agent(agent):
    from decision.modules.visual_servo import VisualServoModule

    servo = VisualServoModule()
    other = servo.start_follow(description="person in blue")
    agent.module.on_system_modules({"VisualServoModule": servo})
    run = agent.module._active_run
    receipt = json.loads(asyncio.run(agent.module._tool_start_follow_person("person in red", run=run)))
    assert receipt["reason"] == "follow_busy"
    assert run.follow_ref is None and not run.motion_submitted
    agent.module.cancel_agent_task(run.run_id)
    assert servo.get_follow_task(other["follow_task_id"], other["product_session_id"])["state"] == "running"


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
