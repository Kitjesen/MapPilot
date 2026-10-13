"""Follow ownership through real Host ports and the navigation task ledger."""

import pytest

from decision.modules.visual_servo import VisualServoModule
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
