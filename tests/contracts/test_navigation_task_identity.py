"""Product contracts for stable navigation task identity."""

from dataclasses import fields
from pathlib import Path

from message.dds_types import (
    NavigationCommandAck,
    NavigationCommandRequest,
    NavigationGoalStatus,
    NavigationState,
)
from message.dds_types_generated import (
    NavigationCommandAck as GeneratedNavigationCommandAck,
)
from message.dds_types_generated import (
    NavigationCommandRequest as GeneratedNavigationCommandRequest,
)
from message.dds_types_generated import (
    NavigationGoalStatus as GeneratedNavigationGoalStatus,
)
from message.dds_types_generated import NavigationState as GeneratedNavigationState
from runtime.msgs.nav import (
    NavigationCommandKind,
    NavigationCommandReceipt,
    NavigationControlMode,
    NavigationGoalState,
    NavigationLifecycle,
)
from runtime.msgs.nav import (
    NavigationGoalStatus as RuntimeNavigationGoalStatus,
)
from runtime.msgs.nav import NavigationState as RuntimeNavigationState

REPO_ROOT = Path(__file__).resolve().parents[2]


def _idl_fields(struct_name: str) -> list[str]:
    idl = (REPO_ROOT / "src/message/idl/lingtu_slam.idl").read_text(encoding="utf-8")
    body = idl.split(f"struct {struct_name} {{", 1)[1].split("};", 1)[0]
    return [line.rstrip(";").split()[-1] for line in body.splitlines() if ";" in line]


def _field_names(message_type: type) -> list[str]:
    return [field.name for field in fields(message_type)]


def test_navigation_task_identity_is_distinct_from_command_attempt_identity() -> None:
    expected = {
        "NavigationCommandRequest": [
            "header",
            "client_id",
            "task_id",
            "request_id",
            "kind",
            "goal",
            "velocity",
            "reason",
        ],
        "NavigationCommandAck": [
            "header",
            "task_id",
            "request_id",
            "kind",
            "accepted",
            "reason",
        ],
        "NavigationGoalStatus": [
            "header",
            "boot_id",
            "event_sequence",
            "task_id",
            "request_id",
            "state",
            "goal_epoch",
            "reason",
        ],
        "NavigationState": [
            "header",
            "boot_id",
            "state_sequence",
            "control_mode",
            "lifecycle_state",
            "active_task_id",
            "active_request_id",
            "goal_epoch",
            "map_id",
            "map_version",
            "map_hash",
            "planning_state",
            "execution_state",
            "recovery_state",
            "progress",
            "authority",
            "hold_reason",
            "failure_code",
        ],
    }
    runtime_types = {
        "NavigationCommandRequest": NavigationCommandRequest,
        "NavigationCommandAck": NavigationCommandAck,
        "NavigationGoalStatus": NavigationGoalStatus,
        "NavigationState": NavigationState,
    }
    generated_types = {
        "NavigationCommandRequest": GeneratedNavigationCommandRequest,
        "NavigationCommandAck": GeneratedNavigationCommandAck,
        "NavigationGoalStatus": GeneratedNavigationGoalStatus,
        "NavigationState": GeneratedNavigationState,
    }

    for name, expected_fields in expected.items():
        assert _idl_fields(name) == expected_fields
        assert _field_names(runtime_types[name]) == expected_fields
        assert _field_names(generated_types[name]) == expected_fields


def test_navigation_task_identity_uses_existing_dds_topics() -> None:
    topics = (REPO_ROOT / "config/runtime_graph/topics.yaml").read_text(encoding="utf-8")

    assert "/nav/command/request" in topics
    assert "/nav/command/ack" in topics
    assert "/nav/goal/status" in topics
    assert "/nav/state" in topics
    assert "/navigation/tasks/request" not in topics
    assert "/navigation/tasks/ack" not in topics


def test_runtime_navigation_messages_preserve_both_identities() -> None:
    status = RuntimeNavigationGoalStatus(
        ts=1.0,
        frame_id="map",
        boot_id="navd-boot",
        sequence=7,
        task_id="navigation-task-1",
        request_id="goal-attempt-2",
        state=int(NavigationGoalState.PATH_ACTIVE),
        goal_epoch=3,
        reason="path_active",
    )
    state = RuntimeNavigationState(
        ts=2.0,
        frame_id="map",
        boot_id="navd-boot",
        sequence=8,
        control_mode=int(NavigationControlMode.AUTONOMY),
        lifecycle_state=int(NavigationLifecycle.EXECUTING),
        active_task_id="navigation-task-1",
        active_request_id="goal-attempt-2",
        goal_epoch=3,
    )

    assert status.to_dict()["task_id"] == "navigation-task-1"
    assert status.to_dict()["request_id"] == "goal-attempt-2"
    assert state.to_dict()["active_task_id"] == "navigation-task-1"
    assert state.to_dict()["active_request_id"] == "goal-attempt-2"


def test_navigation_command_receipt_separates_task_and_attempt() -> None:
    receipt = NavigationCommandReceipt(
        accepted=True,
        kind=int(NavigationCommandKind.GOAL),
        task_id="navigation-task-1",
        request_id="goal-attempt-2",
        endpoint_timestamp_s=3.5,
        reason="planning_started",
    )

    assert receipt.task_id != receipt.request_id
    assert receipt.to_dict() == {
        "accepted": True,
        "kind": int(NavigationCommandKind.GOAL),
        "kind_name": "GOAL",
        "task_id": "navigation-task-1",
        "request_id": "goal-attempt-2",
        "endpoint_timestamp_s": 3.5,
        "reason": "planning_started",
    }
