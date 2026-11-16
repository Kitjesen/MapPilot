"""Follow ownership through real Host ports and the navigation task ledger."""

import json
import time
from types import SimpleNamespace

import pytest

from decision.llm.client import LLMConfig
from decision.modules.agent_planner import AgentPlannerModule
from decision.modules.visual_servo import VisualServoModule
from decision.tasks.sdk_agent import TurnResult
from nav.services.goals import GoalService
from runtime.msgs.nav import NavigationCommandKind, NavigationCommandReceipt, NavigationGoalState, NavigationGoalStatus
from runtime.msgs.numpy_compat import np


class Commands:
    def __init__(self):
        self.goals = []
        self.cancels = []

    def send_goal(self, x, y, z, yaw, *, task_id, request_id):
        self.goals.append((task_id, request_id))
        return self.receipt(task_id, request_id, NavigationCommandKind.GOAL)

    def cancel_task(self, task_id, reason, *, request_id):
        self.cancels.append((task_id, request_id))
        return self.receipt(task_id, request_id, NavigationCommandKind.TASK_CANCEL)

    @staticmethod
    def receipt(task_id, request_id, kind):
        return NavigationCommandReceipt(
            accepted=True, kind=int(kind), task_id=task_id, request_id=request_id,
            endpoint_timestamp_s=100.0, reason="accepted",
        )


@pytest.fixture
def chain():
    commands = Commands()
    goals = GoalService(product_session_id="session-a")
    servo = VisualServoModule()
    goals.on_system_modules({"nav.commands": commands})
    servo.on_system_modules({"nav.goals": goals})
    servo.goal_pose.subscribe(goals.visual_goal_request._deliver)
    servo.goal_cancel.subscribe(goals.visual_cancel_request._deliver)
    goals.goal_status.subscribe(servo.goal_status._deliver)
    goals.task_status.subscribe(servo.navigation_goal_status._deliver)
    goals.setup()
    servo.setup()
    yield servo, goals, commands
    servo.stop()
    goals.stop()


def start(servo, name="person-a", request_id=""):
    receipt = servo.start_follow(target_id=name, request_id=request_id, client_id="test")
    assert receipt["accepted"] is True
    return receipt


def view(servo, ref):
    return servo.get_follow_task(ref["follow_task_id"], ref["product_session_id"])


def cancel(servo, ref):
    return servo.cancel_follow_task(ref["follow_task_id"], ref["product_session_id"])


def publish(servo, x=3):
    servo._last_goal_time = 0
    assert servo._publish_goal_from_3d(np.array([x, 0.0, 0.0]))


def native(goals, ids, state, sequence):
    event = NavigationGoalStatus(
        task_id=ids[0], request_id=ids[1], boot_id="nav-boot", sequence=sequence,
        state=int(state), goal_epoch=1, reason=state.name.lower(),
    )
    goals.navigation_goal_status._deliver(event)
    return event


@pytest.fixture
def chat(chain, tmp_path, monkeypatch):
    servo, goals, _ = chain
    monkeypatch.setattr(AgentPlannerModule, "_init_llm", lambda self: None)
    agent = AgentPlannerModule(conversation_db_path=str(tmp_path / "chat.db"))
    agent._llm_client = SimpleNamespace(config=LLMConfig())
    monkeypatch.setattr(agent, "conversation_available", lambda: True)
    agent.on_system_modules({"VisualServoModule": servo})
    agent.setup()
    goals.task_status.subscribe(agent.navigation_goal_status._deliver)
    cid = agent.create_conversation()["conversation_id"]
    yield agent, cid
    agent.stop()


def chat_turn(chat, text, **kwargs):
    agent, cid = chat
    receipt = agent.submit_conversation_message(
        cid, f"message-{time.monotonic_ns()}", text,
        motion_guard=lambda admit, call_id: admit(), **kwargs,
    )
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        turn = agent.get_conversation_turn(cid, receipt["turn_id"])
        if turn["state"] != "running":
            return turn
        time.sleep(0.01)
    pytest.fail("conversation turn did not finish")


def test_conversation_follow_survives_reply_queries_and_stops_after_native_feedback(chat, chain, monkeypatch):
    agent, cid = chat
    servo, goals, commands = chain
    receipts = []

    async def runner(text, invoke_tool, **kwargs):
        if text == "跟随穿红衣服的人":
            receipts.append(json.loads(await invoke_tool("follow_person", {"description": "穿红衣服的人"}, "start")))
        return TurnResult("reply", "已收到。", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    turn = chat_turn(chat, "跟随穿红衣服的人", permission_mode="full_access")
    assert turn["state"] == "completed"
    ref = receipts[-1]
    assert ref["accepted"] and ref["phase"] == "selecting"
    assert not ref["execution_confirmed"]
    assert turn["run_id"] == ref["run_id"]
    assert agent.get_conversation(cid)["turns"][-1]["run_id"] == ref["run_id"]
    assert json.loads(agent.get_agent_status())["state"] == "running"

    chat_turn(chat, "你好", permission_mode="read_only")
    query = chat_turn(chat, "跟随状态", permission_mode="read_only")
    assert "正在选择人员" in query["text"]
    assert len(receipts) == 1 and len(servo._follow_tasks) == 1
    status = json.loads(agent.get_agent_status())
    assert status["navigation"]["follow_task_id"] == ref["follow_task_id"]
    assert status["navigation"]["product_session_id"] == ref["product_session_id"]

    publish(servo)
    child = commands.goals[-1]
    cancelled = chat_turn(chat, "停止跟随", permission_mode="read_only")
    assert "停止尚未确认" in cancelled["text"]
    assert commands.cancels[-1][0] == child[0]
    assert not json.loads(agent.get_agent_status())["stop_confirmed"]
    servo._on_goal_status({
        "action": "visual_servo_stop", "task_id": child[0],
        "accepted": False, "reason": "temporarily_unavailable",
    })
    cancel_count = len(commands.cancels)
    chat_turn(chat, "停止跟随", permission_mode="read_only")
    assert len(commands.cancels) == cancel_count + 1
    assert commands.cancels[-1][0] == child[0]
    blocked = chat_turn(chat, "跟随穿红衣服的人", permission_mode="full_access")
    assert blocked["state"] == "failed" and receipts[-1]["reason"] == "stop_unconfirmed"
    assert len(servo._follow_tasks) == 1

    native(goals, child, NavigationGoalState.CANCELLED, 1)
    final = json.loads(agent.get_agent_status())
    assert final["state"] == "cancelled" and final["stop_confirmed"]
    assert final["native_navigation_status"]["task_id"] == child[0]
    assert view(servo, ref)["terminal"]
    next_turn = chat_turn(chat, "跟随穿红衣服的人", permission_mode="full_access")
    assert next_turn["state"] == "completed" and receipts[-1]["accepted"]
    assert receipts[-1]["follow_task_id"] != ref["follow_task_id"]
    assert json.loads(agent.cancel_agent_task(ref["run_id"]))["reason"] == "not_current"
    assert view(servo, receipts[-1])["state"] == "running"


@pytest.mark.parametrize("mode", ["read_only", "full_access"])
def test_conversation_model_tools_keep_permissions_and_exact_follow_owner(chat, chain, monkeypatch, mode):
    servo, _, _ = chain
    results = []

    async def runner(text, invoke_tool, **kwargs):
        if text == "start":
            results.append(json.loads(await invoke_tool("follow_person", {"description": "person in red"}, "start")))
        else:
            current = json.loads(await invoke_tool("get_agent_status", {}, "query"))
            results.append(json.loads(await invoke_tool("cancel_agent_task", {"run_id": current["run_id"]}, "cancel")))
        return TurnResult("reply", "已收到。", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    denied = chat_turn(chat, "start", permission_mode="read_only")
    assert denied["state"] == "failed" and results[-1]["reason"] == "permission_read_only"
    assert len(servo._follow_tasks) == 0
    chat_turn(chat, "start", permission_mode="full_access")
    ref = results[-1]
    chat_turn(chat, "让刚才的跟随结束吧", permission_mode=mode)
    if mode == "read_only":
        assert results[-1]["reason"] == "permission_read_only"
        assert view(servo, ref)["state"] == "running"
    else:
        assert results[-1]["cancel_requested"] and results[-1]["stop_confirmed"]
        assert view(servo, ref)["state"] == "cancelled"


def test_conversation_follow_unknown_and_external_cancel_are_not_success(chat, chain, monkeypatch):
    agent, _ = chat
    servo, _, _ = chain

    async def runner(invoke_tool, **kwargs):
        await invoke_tool("follow_person", {"description": "person in red"}, "start")
        return TurnResult("reply", "已收到。", [])

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    chat_turn(chat, "start", permission_mode="full_access")
    ref = agent._active_run.follow_ref
    with monkeypatch.context() as patch:
        patch.setattr(servo, "get_follow_task", lambda **kwargs: {"found": False, "reason": "follow_task_not_found"})
        status = json.loads(agent.get_agent_status())
        assert status["navigation"]["state"] == "FOLLOW_UNKNOWN"
        assert not status["stop_confirmed"] and not status["navigation"]["terminal"]
    servo.cancel_follow_task(**ref)
    status = json.loads(agent.get_agent_status())
    assert status["state"] == "cancelled" and status["stop_confirmed"]
    assert not status["motion_success"]


def test_conversation_failure_cancels_its_accepted_follow(chat, chain, monkeypatch):
    agent, _ = chat
    servo, _, _ = chain

    async def runner(invoke_tool, **kwargs):
        receipt = json.loads(await invoke_tool("follow_person", {"description": "person in red"}, "start"))
        assert receipt["accepted"]
        raise RuntimeError("provider disconnected after admission")

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    turn = chat_turn(chat, "start", permission_mode="full_access")
    assert turn["state"] == "failed"
    assert servo.get_follow_task(**agent._active_run.follow_ref)["state"] == "cancelled"
    assert json.loads(agent.get_agent_status())["stop_confirmed"]


@pytest.mark.parametrize("description", ["我", "他", "她", "那个人", "me", "him", "her", "ME！"])
def test_unbound_person_reference_does_not_start_follow(chain, description):
    servo, _, commands = chain

    receipt = servo.start_follow(description=description)

    assert receipt["accepted"] is False
    assert receipt["reason"] == "follow_target_ambiguous"
    assert "follow_task_id" not in receipt
    assert servo.get_servo_status()["mode"] == "idle"
    assert commands.goals == []


def test_person_reference_with_explicit_tracker_identity_can_start(chain):
    servo, _, _ = chain

    receipt = servo.start_follow(description="我", target_id="selected-person")

    assert receipt["accepted"] is True
    assert view(servo, receipt)["details"]["target_id"] == "selected-person"


def test_follow_cancel_waits_for_all_owned_native_children(chain):
    servo, goals, commands = chain
    ref = start(servo)
    publish(servo)
    first = commands.goals[-1]
    publish(servo, 4)
    second = commands.goals[-1]
    assert first != second
    assert view(servo, ref)["state"] == "running"
    assert cancel(servo, ref)["accepted"]
    assert {item[0] for item in commands.cancels} == {first[0], second[0]}
    assert view(servo, ref)["state"] == "cancelling"
    assert view(servo, ref)["terminal"] is False
    assert servo.start_follow(target_id="new-person")["reason"] == "follow_busy"
    native(goals, first, NavigationGoalState.CANCELLED, 1)
    assert view(servo, ref)["terminal"] is False
    native(goals, second, NavigationGoalState.CANCELLED, 2)
    assert view(servo, ref)["state"] == "cancelled"
    assert view(servo, ref)["terminal"] is True
    assert view(servo, ref)["available_operations"] == []


def test_old_cancel_admission_and_native_events_cannot_change_new_follow(chain):
    servo, goals, commands = chain
    old = start(servo)
    publish(servo)
    old_goal = commands.goals[-1]
    cancel(servo, old)
    old_terminal = native(goals, old_goal, NavigationGoalState.CANCELLED, 1)
    old_snapshot = view(servo, old)
    current = start(servo, "person-b")
    publish(servo, 4)
    current_goal = commands.goals[-1]
    native(goals, current_goal, NavigationGoalState.PATH_ACTIVE, 2)
    before = view(servo, current)
    cancelled_count = len(commands.cancels)

    assert cancel(servo, old)["reason"] == "already_terminal"
    servo.goal_status._deliver({
        "action": "visual_servo", "accepted": True,
        "task_id": old_goal[0], "request_id": old_goal[1], "state": "accepted",
    })
    servo.goal_status._deliver({
        "action": "visual_servo_stop", "accepted": False,
        "task_id": old_goal[0], "reason": "old_cancel_failed",
    })
    servo.navigation_goal_status._deliver(old_terminal)
    native(goals, old_goal, NavigationGoalState.PATH_ACTIVE, 1)

    after = view(servo, current)
    assert len(commands.cancels) == cancelled_count
    assert after["follow_task_id"] == current["follow_task_id"] != old["follow_task_id"]
    assert after["state"] == before["state"] == "running"
    assert after["details"]["navigation_task_id"] == current_goal[0]
    assert after["details"]["navigation_state"] == "path_active"
    assert view(servo, old) == old_snapshot


def test_cancel_without_motion_is_terminal_but_not_physical_stop_evidence(chain):
    servo, _, commands = chain
    ref = start(servo)
    receipt = cancel(servo, ref)
    assert receipt["accepted"] and receipt["terminal"]
    assert receipt["execution_confirmed"] is False
    assert commands.goals == commands.cancels == []
    assert view(servo, ref)["children"] == {}


def test_session_and_unscoped_cancel_cannot_mutate_follow(chain):
    servo, _, commands = chain
    ref = start(servo)
    publish(servo)
    assert servo.cancel_follow_task(ref["follow_task_id"], "session-old")["accepted"] is False
    assert servo.get_follow_task(ref["follow_task_id"], "session-old")["found"] is False
    assert servo.stop_servo()["reason"] == "follow_task_reference_required"
    servo._on_servo_target("stop")
    assert commands.cancels == []
    assert view(servo, ref)["state"] == "running"


def test_start_retry_preserves_task_identity_and_cannot_restart_terminal_task(chain):
    servo, _, _ = chain
    first = start(servo, request_id="request-one")
    retry = start(servo, request_id="request-one")
    assert retry["follow_task_id"] == first["follow_task_id"]
    assert servo.start_follow(target_id="different", request_id="request-one", client_id="test")["reason"] == "request_id_conflict"
    cancel(servo, first)
    assert start(servo, request_id="request-one")["state"] == "cancelled"
    assert start(servo, request_id="request-two")["follow_task_id"] != first["follow_task_id"]


def test_dict_terminal_cannot_complete_follow_and_late_ack_cannot_roll_back_native_progress(chain):
    servo, goals, commands = chain
    ref = start(servo)
    publish(servo)
    ids = commands.goals[-1]
    native(goals, ids, NavigationGoalState.PATH_ACTIVE, 1)
    servo.goal_status._deliver({
        "action": "visual_servo", "task_id": ids[0], "request_id": ids[1],
        "accepted": True, "state": "accepted",
    })
    assert view(servo, ref)["details"]["navigation_state"] == "path_active"
    cancel(servo, ref)
    servo.goal_status._deliver({
        "action": "visual_servo", "task_id": ids[0], "request_id": ids[1],
        "accepted": True, "state": "cancelled", "terminal": True, "source": "native_goal_status",
    })
    assert view(servo, ref)["state"] == "cancelling"
    native(goals, ids, NavigationGoalState.CANCELLED, 2)
    assert view(servo, ref)["state"] == "cancelled"


def test_follow_reached_child_does_not_end_continuous_task(chain):
    servo, goals, commands = chain
    ref = start(servo)
    publish(servo)
    native(goals, commands.goals[-1], NavigationGoalState.REACHED, 1)
    assert view(servo, ref)["state"] == "running"
    assert view(servo, ref)["terminal"] is False
    publish(servo, 4)
    assert len(commands.goals) == 2


def test_goal_ack_arriving_after_cancel_is_cancelled_by_id():
    servo = VisualServoModule(product_session_id="session-a")
    ref = start(servo)
    requests, cancels = [], []
    servo.goal_pose.subscribe(requests.append)
    servo.goal_cancel.subscribe(cancels.append)
    publish(servo)
    cancel(servo, ref)
    request = requests[-1]
    assert view(servo, ref)["terminal"] is False
    servo._on_goal_status({
        "action": "visual_servo", "task_id": request.task_id, "request_id": request.request_id,
        "accepted": True, "state": "accepted",
    })
    assert cancels[-1].task_ids == (request.task_id,)
    assert view(servo, ref)["state"] == "cancelling"


def test_semantic_old_handoff_cancellation_keeps_later_operator_follow(chain):
    from decision.modules.semantic_planner import SemanticPlannerModule

    servo, _, _ = chain
    planner = SemanticPlannerModule()
    planner.on_system_modules({"VisualServoModule": servo})
    planner._process_instruction("follow person in red")
    old = planner._follow_ref.copy()
    cancel(servo, old)
    current = start(servo, "person-b")
    planner._stop_visual_handoff()
    assert view(servo, current)["state"] == "running"
    assert view(servo, old)["state"] == "cancelled"


def test_cancel_rejection_remains_pending_and_explicit_retry_is_scoped(chain):
    servo, goals, commands = chain
    ref = start(servo)
    publish(servo)
    child = commands.goals[-1]
    cancel(servo, ref)
    servo._on_goal_status({
        "action": "visual_servo_stop", "task_id": child[0],
        "accepted": False, "reason": "temporarily_unavailable",
    })
    assert view(servo, ref)["terminal"] is False
    assert view(servo, ref)["details"]["navigation_state"] == "unknown"
    before = len(commands.cancels)
    cancel(servo, ref)
    assert len(commands.cancels) == before + 1
    assert commands.cancels[-1][0] == child[0]
    native(goals, child, NavigationGoalState.CANCELLED, 1)
    assert view(servo, ref)["terminal"] is True


def test_mcp_discovery_queries_and_cancels_the_same_owner(chain):
    import json

    from gateway.mcp_server import MCPServerModule

    servo, _, _ = chain
    mcp = MCPServerModule()
    mcp.on_system_modules({"VisualServoModule": servo})
    tools = mcp._tool_registry
    ref = tools["start_follow"](target_id="person-a")
    query = tools["get_follow_task"](ref["follow_task_id"], ref["product_session_id"])
    assert query["state"] == "running"
    tools["cancel_follow_task"](ref["follow_task_id"], ref["product_session_id"])
    assert view(servo, ref)["state"] == "cancelled"
    schema = next(info for info in servo.get_skill_infos() if info.func_name == "cancel_follow_task")
    assert json.loads(schema.args_schema)["required"] == ["follow_task_id", "product_session_id"]
