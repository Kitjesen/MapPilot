"""Tests for nav.goals.

All tests are pure Python and do not require ROS2 or hardware.
"""

from __future__ import annotations

import json

import pytest

from nav.services.goals import GoalService
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import (
    NavigationCommandKind,
    NavigationCommandReceipt,
)


@pytest.fixture
def goal_service():
    mod = GoalService()
    mod.setup()
    goals: list[PoseStamped] = []
    patrols: list[list] = []
    cancels: list[str] = []
    statuses: list[dict] = []
    mod.goal_pose.subscribe(goals.append)
    mod.patrol_goals.subscribe(patrols.append)
    mod.cancel.subscribe(cancels.append)
    mod.goal_status.subscribe(statuses.append)
    mod._test_goals = goals
    mod._test_patrols = patrols
    mod._test_cancels = cancels
    mod._test_statuses = statuses
    return mod


def _cmd(mod: GoalService, cmd: dict) -> dict:
    mod._test_statuses.clear()
    mod._on_command(json.dumps(cmd))
    assert mod._test_statuses, "no status published"
    return mod._test_statuses[-1]


class TestGoalService:
    def test_port_types(self, goal_service):
        summary = goal_service.port_summary()

        assert summary["ports_in"]["goal_command"]["type"] == "str"
        assert summary["ports_in"]["goal_request"]["type"] == "PoseStamped"
        assert summary["ports_in"]["cancel_request"]["type"] == "str"
        assert summary["ports_out"]["goal_pose"]["type"] == "PoseStamped"
        assert summary["ports_out"]["patrol_goals"]["type"] == "list"
        assert summary["ports_out"]["cancel"]["type"] == "str"
        assert summary["ports_out"]["goal_status"]["type"] == "dict"

    def test_goto_command_publishes_map_goal(self, goal_service):
        status = _cmd(
            goal_service,
            {"action": "goto", "x": 1.0, "y": 2.0, "z": 0.3, "yaw": 0.5},
        )

        assert status["success"] is True
        assert len(goal_service._test_goals) == 1
        goal = goal_service._test_goals[-1]
        assert goal.frame_id == "map"
        assert goal.x == pytest.approx(1.0)
        assert goal.y == pytest.approx(2.0)
        assert goal.z == pytest.approx(0.3)
        assert goal.yaw == pytest.approx(0.5)

    def test_goal_request_republishes_normalized_goal(self, goal_service):
        goal = PoseStamped(
            pose=Pose(
                position=Vector3(3.0, 4.0, 0.2),
                orientation=Quaternion.from_yaw(0.25),
            ),
            frame_id="/map",
        )

        goal_service.goal_request._deliver(goal)

        assert goal_service._test_statuses[-1]["success"] is True
        assert len(goal_service._test_goals) == 1
        assert goal_service._test_goals[-1].frame_id == "map"
        assert goal_service._test_goals[-1].x == pytest.approx(3.0)

    def test_goal_request_rejects_wrong_frame(self, goal_service):
        goal = PoseStamped(pose=Pose(1.0, 2.0, 0.0), frame_id="odom")

        goal_service.goal_request._deliver(goal)

        assert goal_service._test_statuses[-1]["success"] is False
        assert goal_service._test_goals == []

    def test_patrol_command_publishes_waypoints(self, goal_service):
        status = _cmd(
            goal_service,
            {
                "action": "patrol",
                "loop": True,
                "waypoints": [{"x": 1.0, "y": 2.0}, [3.0, 4.0, 0.2]],
            },
        )

        assert status["success"] is True
        assert len(goal_service._test_patrols) == 1
        assert goal_service._test_patrols[-1] == [
            {"x": 1.0, "y": 2.0, "z": 0.0, "frame_id": "map", "loop": True},
            {"x": 3.0, "y": 4.0, "z": 0.2, "frame_id": "map", "loop": True},
        ]

    def test_patrol_command_rejects_wrong_frame(self, goal_service):
        status = _cmd(
            goal_service,
            {
                "action": "patrol",
                "waypoints": [{"x": 1.0, "y": 2.0, "frame_id": "odom"}],
            },
        )

        assert status["success"] is False
        assert goal_service._test_patrols == []

    def test_cancel_command_publishes_reason(self, goal_service):
        status = _cmd(goal_service, {"action": "cancel", "reason": "operator"})

        assert status["success"] is True
        assert goal_service._test_cancels == ["operator"]

    def test_cancel_request_publishes_reason(self, goal_service):
        goal_service.cancel_request._deliver("api_cancel")

        assert goal_service._test_statuses[-1]["success"] is True
        assert goal_service._test_cancels == ["api_cancel"]

    def test_native_goal_uses_assembled_command_capability(self):
        class FakeCommands:
            def __init__(self) -> None:
                self.goals = []

            def send_goal(self, x, y, z, yaw, *, task_id=None, request_id=None):
                self.goals.append((x, y, z, yaw, task_id, request_id))
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="accepted",
                )

        commands = FakeCommands()
        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": commands})
        service.setup()
        emitted = []
        statuses = []
        service.goal_pose.subscribe(emitted.append)
        service.goal_status.subscribe(statuses.append)

        service.goal_request._deliver(
            PoseStamped(
                pose=Pose(
                    position=Vector3(1.0, 2.0, 0.3),
                    orientation=Quaternion.from_yaw(0.4),
                ),
                frame_id="map",
            )
        )

        assert emitted == []
        assert commands.goals[0][:4] == (1.0, 2.0, 0.3, pytest.approx(0.4))
        assert str(commands.goals[0][4]).startswith("nav-task-")
        generated_task_id = str(commands.goals[0][4])
        assert len(generated_task_id) == len("nav-task-") + 32
        int(generated_task_id.removeprefix("nav-task-"), 16)
        assert generated_task_id != str(commands.goals[0][5])
        assert str(commands.goals[0][5]).startswith("nav-")
        assert statuses[-1]["success"] is True
        assert statuses[-1]["sink"] == "native_dds"

    def test_native_task_receipts_preserve_task_identity_across_goal_and_cancel(self):
        class FakeCommands:
            def __init__(self) -> None:
                self.calls = []

            def send_goal(
                self,
                x,
                y,
                z,
                yaw,
                *,
                task_id=None,
                request_id=None,
            ):
                self.calls.append(("goal", task_id, request_id, x, y, z, yaw))
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="accepted",
                )

            def cancel_task(self, task_id, reason, *, request_id=None):
                self.calls.append(("cancel", task_id, request_id, reason))
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.CANCEL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=101.0,
                    reason="cancel_requested",
                )

        commands = FakeCommands()
        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": commands})
        service.setup()
        goal = PoseStamped(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
            frame_id="map",
        )

        accepted = service.submit_goal(
            goal,
            task_id="navigation-task-1",
            request_id="goal-attempt-1",
        )
        cancel_requested = service.submit_cancel(
            "operator_cancel",
            task_id="navigation-task-1",
            request_id="cancel-attempt-1",
        )

        assert accepted["accepted"] is True
        assert accepted["task_id"] == "navigation-task-1"
        assert accepted["request_id"] == "goal-attempt-1"
        assert accepted["state"] == "accepted"
        assert accepted["native_request_id"] == "goal-attempt-1"
        assert accepted["native_ack"]["accepted"] is True
        assert cancel_requested["accepted"] is True
        assert cancel_requested["task_id"] == "navigation-task-1"
        assert cancel_requested["request_id"] == "cancel-attempt-1"
        assert cancel_requested["state"] == "cancel_requested"
        assert "cancelled" not in str(cancel_requested).lower()
        assert cancel_requested["native_request_id"] == "cancel-attempt-1"
        assert cancel_requested["native_ack"]["accepted"] is True
        assert commands.calls == [
            ("goal", "navigation-task-1", "goal-attempt-1", 1.0, 2.0, 0.0, 0.0),
            (
                "cancel",
                "navigation-task-1",
                "cancel-attempt-1",
                "operator_cancel",
            ),
        ]

    def test_native_cancel_without_task_identity_fails_closed(self):
        class FakeCommands:
            def __init__(self) -> None:
                self.reasons = []

            def cancel_task(self, task_id, reason, *, request_id=None):
                self.reasons.append((reason, task_id, request_id))
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.CANCEL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=101.0,
                    reason="cancel_requested",
                )

        commands = FakeCommands()
        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": commands})
        service.setup()
        emitted = []
        statuses = []
        service.cancel.subscribe(emitted.append)
        service.goal_status.subscribe(statuses.append)

        service.cancel_request._deliver("operator_cancel")

        assert emitted == []
        assert commands.reasons == []
        assert statuses[-1]["success"] is False
        assert "task_id is required" in statuses[-1]["message"]
        assert statuses[-1]["sink"] == "native_dds"
        assert statuses[-1]["state"] == "rejected"

    def test_goal_rejects_equal_task_and_request_identity(self):
        service = GoalService()
        service.setup()
        emitted = []
        service.goal_pose.subscribe(emitted.append)

        status = service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="same-identity",
            request_id="same-identity",
        )

        assert emitted == []
        assert status["accepted"] is False
        assert "must be distinct" in status["message"]

    @pytest.mark.parametrize("invalid_ack", [None, "queued", 1, {}, {"accepted": True}])
    def test_native_goal_malformed_ack_is_not_reported_as_accepted(
        self,
        invalid_ack,
    ):
        class MalformedCommands:
            def send_goal(self, x, y, z, yaw, *, task_id=None, request_id=None):
                return invalid_ack

        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": MalformedCommands()})
        service.setup()
        statuses = []
        service.goal_status.subscribe(statuses.append)

        service.goal_request._deliver(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            )
        )

        assert statuses[-1]["success"] is False
        assert statuses[-1]["accepted"] is False
        assert "invalid task receipt" in statuses[-1]["message"]

    def test_native_goal_boolean_is_not_a_business_ack(self):
        class RejectingCommands:
            def send_goal(self, x, y, z, yaw, *, task_id=None, request_id=None) -> bool:
                return False

        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": RejectingCommands()})
        service.setup()
        statuses = []
        service.goal_status.subscribe(statuses.append)

        service.goal_request._deliver(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            )
        )

        assert statuses[-1]["success"] is False
        assert statuses[-1]["accepted"] is False
        assert "send_goal" in statuses[-1]["message"]
        assert "invalid task receipt" in statuses[-1]["message"]

    def test_native_cancel_boolean_is_not_a_business_ack(self):
        class RejectingCommands:
            def cancel_task(self, task_id, reason, *, request_id=None) -> bool:
                return False

        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": RejectingCommands()})
        service.setup()

        result = service.submit_cancel(
            "operator_cancel",
            task_id="navigation-task-rejected",
            request_id="cancel-rejected",
        )

        assert result["success"] is False
        assert result["accepted"] is False
        assert result["request_id"] == "cancel-rejected"
        assert "cancel" in result["message"]
        assert "invalid task receipt" in result["message"]

    def test_native_inspection_uses_assembled_command_capability(self):
        class FakeCommands:
            def __init__(self) -> None:
                self.starts = []

            def start_inspection(self, route_id, *, revision=0, request_id=None) -> bool:
                self.starts.append((route_id, revision, request_id))
                return True

        commands = FakeCommands()
        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": commands})
        service.setup()
        patrols = []
        statuses = []
        service.patrol_goals.subscribe(patrols.append)
        service.goal_status.subscribe(statuses.append)

        service._on_command(
            json.dumps(
                {
                    "action": "inspection",
                    "route_id": "daily-route",
                    "route_revision": (1 << 64) - 1,
                    "request_id": "inspection-42",
                }
            )
        )

        assert patrols == []
        assert commands.starts == [("daily-route", (1 << 64) - 1, "inspection-42")]
        assert statuses[-1] == {
            "request_id": "inspection-42",
            "action": "inspection",
            "success": True,
            "accepted": True,
            "state": "accepted",
            "message": "inspection command dispatched",
            "route_id": "daily-route",
            "route_revision": (1 << 64) - 1,
            "sink": "native_dds",
        }

    @pytest.mark.parametrize("revision", [-1, 1 << 64])
    def test_native_inspection_rejects_revision_outside_uint64(self, revision):
        class FakeCommands:
            calls = 0

            def start_inspection(self, **kwargs):
                del kwargs
                self.calls += 1
                return True

        commands = FakeCommands()
        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": commands})
        service.setup()
        statuses = []
        service.goal_status.subscribe(statuses.append)

        service._on_command(
            json.dumps(
                {
                    "action": "inspection",
                    "route_id": "daily-route",
                    "revision": revision,
                }
            )
        )

        assert commands.calls == 0
        assert statuses[-1]["success"] is False
        assert statuses[-1]["message"] == "inspection route revision must be between 0 and UINT64_MAX"

    def test_invalid_json_reports_error(self, goal_service):
        goal_service._on_command("not-json")

        assert goal_service._test_statuses[-1]["success"] is False
        assert goal_service._test_goals == []
