"""Tests for nav.goals.

All tests are pure Python and do not require ROS2 or hardware.
"""

from __future__ import annotations

import json
import sqlite3

import pytest

from nav.services.goals import GoalService
from nav.services.task_ledger import NavigationTaskLedger
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import (
    NavigationCommandKind,
    NavigationCommandReceipt,
    NavigationGoalState,
    NavigationGoalStatus,
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
    @pytest.mark.parametrize(
        "legacy_column",
        [
            "_".join(("product", "fingerprint")),
            "_".join(("runtime", "manifest", "fingerprint")),
        ],
    )
    def test_task_ledger_migrates_legacy_runtime_identity_column(
        self,
        tmp_path,
        legacy_column,
    ):
        db_path = tmp_path / "legacy-tasks.sqlite3"
        with sqlite3.connect(db_path) as connection:
            connection.execute(
                f"""
                CREATE TABLE navigation_tasks (
                    task_id TEXT PRIMARY KEY,
                    state TEXT NOT NULL,
                    admission TEXT NOT NULL DEFAULT 'unconfirmed',
                    admission_reason TEXT NOT NULL DEFAULT '',
                    evidence_status TEXT NOT NULL DEFAULT 'unavailable',
                    state_source TEXT NOT NULL DEFAULT 'none',
                    state_observed_at REAL,
                    terminal INTEGER NOT NULL DEFAULT 0,
                    reason TEXT NOT NULL DEFAULT '',
                    source TEXT NOT NULL DEFAULT '',
                    observed_only INTEGER NOT NULL DEFAULT 0,
                    target_json TEXT NOT NULL,
                    {legacy_column} TEXT NOT NULL DEFAULT '',
                    map_identity_json TEXT NOT NULL,
                    created_at REAL NOT NULL,
                    updated_at REAL NOT NULL,
                    terminal_at REAL,
                    endpoint_boot_id TEXT NOT NULL DEFAULT '',
                    active_request_id TEXT NOT NULL DEFAULT '',
                    cancel_requested_at REAL,
                    cancel_request_id TEXT NOT NULL DEFAULT '',
                    cancel_reason TEXT NOT NULL DEFAULT '',
                    can_resume INTEGER NOT NULL DEFAULT 0,
                    last_goal_status_json TEXT,
                    last_navigation_state_json TEXT
                )
                """
            )
            connection.execute(
                f"""
                INSERT INTO navigation_tasks (
                    task_id, state, target_json, {legacy_column},
                    map_identity_json, created_at, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                ("legacy-task", "unknown", "{}", "legacy-runtime", "{}", 1.0, 1.0),
            )

        ledger = NavigationTaskLedger(db_path)
        row = ledger._conn.execute(
            """
            SELECT run_plan_fingerprint
            FROM navigation_tasks WHERE task_id = ?
            """,
            ("legacy-task",),
        ).fetchone()

        assert row["run_plan_fingerprint"] == "legacy-runtime"
        ledger.close()

    def test_port_types(self, goal_service):
        summary = goal_service.port_summary()

        assert summary["ports_in"]["goal_command"]["type"] == "str"
        assert summary["ports_in"]["goal_request"]["type"] == "PoseStamped"
        assert summary["ports_in"]["cancel_request"]["type"] == "str"
        assert summary["ports_in"]["navigation_goal_status"]["type"] == "NavigationGoalStatus"
        assert summary["ports_out"]["goal_pose"]["type"] == "PoseStamped"
        assert summary["ports_out"]["patrol_goals"]["type"] == "list"
        assert summary["ports_out"]["cancel"]["type"] == "str"
        assert summary["ports_out"]["goal_status"]["type"] == "dict"

    @pytest.mark.parametrize(
        "retired_name",
        [
            "_".join(("product", "fingerprint")),
            "_".join(("runtime", "manifest", "fingerprint")),
        ],
    )
    def test_goal_service_rejects_retired_fingerprint_kwargs(self, retired_name):
        with pytest.raises(TypeError, match="run_plan_fingerprint"):
            GoalService(**{retired_name: "retired"})

    def test_native_goal_status_updates_live_task_history(self, goal_service):
        goal_service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-live-status",
            request_id="request-live-status",
        )
        published_statuses = len(goal_service._test_statuses)

        goal_service.navigation_goal_status._deliver(
            NavigationGoalStatus(
                ts=100.0,
                frame_id="map",
                boot_id="navd-boot-live",
                sequence=1,
                task_id="task-live-status",
                request_id="request-live-status",
                state=int(NavigationGoalState.PATH_ACTIVE),
                goal_epoch=1,
                reason="path_active",
            )
        )

        task = goal_service.get_task("task-live-status")
        assert task is not None
        assert task["execution_state"] == "executing"
        assert task["state_source"] == "native_goal_status"
        assert task["evidence_status"] == "fresh"
        assert task["terminal"] is False
        assert len(goal_service._test_statuses) == published_statuses

    def test_foreign_native_goal_status_does_not_create_task(self, goal_service):
        goal_service.navigation_goal_status._deliver(
            NavigationGoalStatus(
                ts=101.0,
                frame_id="map",
                boot_id="navd-boot-live",
                sequence=2,
                task_id="task-owned-by-another-service",
                request_id="request-owned-by-another-service",
                state=int(NavigationGoalState.PATH_ACTIVE),
                goal_epoch=1,
                reason="path_active",
            )
        )

        assert goal_service.get_task("task-owned-by-another-service") is None
        assert goal_service.navigation_goal_status.callback_errors == 0

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
                    kind=int(NavigationCommandKind.TASK_CANCEL),
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
                    kind=int(NavigationCommandKind.TASK_CANCEL),
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

            def start_inspection_task(
                self,
                task_id,
                route_id,
                *,
                revision=0,
                request_id=None,
            ) -> bool:
                self.starts.append((task_id, route_id, revision, request_id))
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
                    "task_id": "inspection-task-42",
                    "route_id": "daily-route",
                    "route_revision": (1 << 64) - 1,
                    "request_id": "inspection-42",
                }
            )
        )

        assert patrols == []
        assert commands.starts == [
            (
                "inspection-task-42",
                "daily-route",
                (1 << 64) - 1,
                "inspection-42",
            )
        ]
        assert statuses[-1] == {
            "request_id": "inspection-42",
            "action": "inspection",
            "success": True,
            "accepted": True,
            "state": "accepted",
            "message": "inspection command dispatched",
            "task_id": "inspection-task-42",
            "route_id": "daily-route",
            "route_revision": (1 << 64) - 1,
            "sink": "native_dds",
        }

    @pytest.mark.parametrize("revision", [-1, 1 << 64])
    def test_native_inspection_rejects_revision_outside_uint64(self, revision):
        class FakeCommands:
            calls = 0

            def start_inspection_task(self, **kwargs):
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

    def test_task_attempt_replay_does_not_dispatch_native_goal_twice(self, tmp_path):
        class FakeCommands:
            def __init__(self) -> None:
                self.goals = []

            def send_goal(self, x, y, z, yaw, *, task_id, request_id):
                self.goals.append((task_id, request_id, x, y, z, yaw))
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="planning_started",
                )

            def read_navigation_state(self):
                return {
                    "boot_id": "navd-boot-a",
                    "active_task_id": "task-replay",
                    "active_request_id": "goal-attempt-replay",
                }

            def get_navigation_task_status(self, task_id):
                del task_id
                return None

        commands = FakeCommands()
        service = GoalService(
            command_module="nav.commands",
            task_ledger_path=str(tmp_path / "tasks.sqlite3"),
        )
        service.on_system_modules({"nav.commands": commands})
        service.setup()
        goal = PoseStamped(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
            frame_id="map",
        )

        first = service.submit_goal(
            goal,
            task_id="task-replay",
            request_id="goal-attempt-replay",
        )
        status_count = service.goal_status.msg_count
        second = service.submit_goal(
            goal,
            task_id="task-replay",
            request_id="goal-attempt-replay",
        )

        assert first["accepted"] is True
        assert second["accepted"] is True
        assert second["replay"] is True
        assert len(commands.goals) == 1
        assert service.goal_status.msg_count == status_count + 1
        task = service.get_task("task-replay")
        assert task is not None
        assert task["admission"] == "accepted"
        assert task["execution_state"] is None
        assert len(task["attempts"]) == 1
        service.stop()

    def test_goal_replay_lookup_is_read_only_and_never_dispatches(self):
        class FakeCommands:
            def __init__(self) -> None:
                self.goals = []

            def send_goal(self, x, y, z, yaw, *, task_id, request_id):
                self.goals.append((task_id, request_id, x, y, z, yaw))
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="planning_started",
                )

        commands = FakeCommands()
        service = GoalService(
            command_module="nav.commands",
            run_plan_fingerprint="runtime-a",
            map_identity={"map_id": "yard", "version": 7},
        )
        service.on_system_modules({"nav.commands": commands})
        service.setup()
        original = PoseStamped(
            pose=Pose(
                position=Vector3(1.0, 2.0, 0.0),
                orientation=Quaternion.from_yaw(0.25),
            ),
            frame_id="/map",
        )
        service.submit_goal(
            original,
            task_id="task-probe",
            request_id="goal-attempt-probe",
            action="goto",
        )
        before = service.get_task("task-probe")
        status_count = service.goal_status.msg_count

        replay = service.lookup_goal_replay(
            PoseStamped(
                pose=Pose(
                    position=Vector3(1.0, 2.0, 0.0),
                    orientation=Quaternion.from_yaw(0.25),
                ),
                frame_id="map",
            ),
            task_id="task-probe",
            request_id="goal-attempt-probe",
            action="goto",
        )

        assert replay is not None
        assert replay["accepted"] is True
        assert replay["replay"] is True
        assert replay["task_id"] == "task-probe"
        assert replay["request_id"] == "goal-attempt-probe"
        assert replay["native_request_id"] == "goal-attempt-probe"
        assert replay["native_ack"]["accepted"] is True
        assert len(commands.goals) == 1
        assert service.get_task("task-probe") == before
        assert service.goal_status.msg_count == status_count
        service.stop()

    def test_goal_replay_lookup_returns_none_without_both_stable_ids(self):
        service = GoalService()
        service.setup()

        assert (
            service.lookup_goal_replay(
                object(),
                task_id="",
                request_id="goal-attempt",
                action="goto",
            )
            is None
        )
        assert (
            service.lookup_goal_replay(
                object(),
                task_id="task-id",
                request_id="",
                action="goto",
            )
            is None
        )
        rejected = service.lookup_goal_replay(
            object(),
            task_id="task-id",
            request_id="goal-attempt",
            action="goto",
        )
        assert rejected is not None
        assert rejected["accepted"] is False
        assert service.goal_status.msg_count == 0
        service.stop()

    def test_goal_replay_lookup_conflict_is_rejected_without_redispatch(self):
        class FakeCommands:
            calls = 0

            def send_goal(self, x, y, z, yaw, *, task_id, request_id):
                del x, y, z, yaw
                self.calls += 1
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="planning_started",
                )

        commands = FakeCommands()
        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": commands})
        service.setup()
        service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-probe-conflict",
            request_id="goal-attempt-probe-conflict",
        )
        status_count = service.goal_status.msg_count

        rejected = service.lookup_goal_replay(
            PoseStamped(
                pose=Pose(position=Vector3(9.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-probe-conflict",
            request_id="goal-attempt-probe-conflict",
            action="goto",
        )

        assert rejected is not None
        assert rejected["accepted"] is False
        assert rejected["state"] == "rejected"
        assert "conflict" in rejected["message"]
        assert commands.calls == 1
        assert service.goal_status.msg_count == status_count
        service.stop()

    def test_goal_replay_lookup_rejects_missing_product_and_map_context(self):
        class FakeCommands:
            calls = 0

            def send_goal(self, x, y, z, yaw, *, task_id, request_id):
                del x, y, z, yaw
                self.calls += 1
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="planning_started",
                )

        commands = FakeCommands()
        service = GoalService(
            command_module="nav.commands",
            run_plan_fingerprint="runtime-a",
            map_identity={"map_id": "yard", "version": 7},
        )
        service.on_system_modules({"nav.commands": commands})
        service.setup()
        goal = PoseStamped(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
            frame_id="map",
        )
        service.submit_goal(
            goal,
            task_id="task-probe-context",
            request_id="goal-attempt-probe-context",
        )
        service._run_plan_fingerprint = ""
        service._map_identity = None
        status_count = service.goal_status.msg_count

        rejected = service.lookup_goal_replay(
            goal,
            task_id="task-probe-context",
            request_id="goal-attempt-probe-context",
            action="goto",
        )

        assert rejected is not None
        assert rejected["accepted"] is False
        assert rejected["state"] == "rejected"
        assert "different RunPlan" in rejected["message"]
        assert commands.calls == 1
        assert service.goal_status.msg_count == status_count
        service.stop()

    def test_goal_replay_lookup_fails_closed_when_history_is_unavailable(self, monkeypatch):
        class FakeCommands:
            calls = 0

            def send_goal(self, *args, **kwargs):
                del args, kwargs
                self.calls += 1
                raise AssertionError("replay lookup must not dispatch")

        commands = FakeCommands()
        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": commands})
        service.setup()

        def fail_lookup(*args, **kwargs):
            del args, kwargs
            raise OSError("database unavailable")

        monkeypatch.setattr(service._task_ledger, "lookup_admission", fail_lookup)
        status_count = service.goal_status.msg_count
        rejected = service.lookup_goal_replay(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-probe-unavailable",
            request_id="goal-attempt-probe-unavailable",
            action="goto",
        )

        assert rejected is not None
        assert rejected["accepted"] is False
        assert rejected["state"] == "rejected"
        assert rejected["task_id"] == "task-probe-unavailable"
        assert rejected["request_id"] == "goal-attempt-probe-unavailable"
        assert "task history unavailable" in rejected["message"]
        assert "not dispatched" in rejected["message"]
        assert commands.calls == 0
        assert service.goal_status.msg_count == status_count
        service.stop()

    def test_task_terminal_status_survives_goal_service_restart(self, tmp_path):
        class FakeCommands:
            def __init__(self, *, terminal=False) -> None:
                self.terminal = terminal
                self.goals = []

            def send_goal(self, x, y, z, yaw, *, task_id, request_id):
                self.goals.append((task_id, request_id))
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="planning_started",
                )

            def read_navigation_state(self):
                return {
                    "boot_id": "navd-boot-a",
                    "active_task_id": "" if self.terminal else "task-persisted",
                    "active_request_id": "" if self.terminal else "goal-attempt-persisted",
                }

            def get_navigation_task_status(self, task_id):
                if not self.terminal:
                    return None
                return {
                    "ts": 120.0,
                    "frame_id": "map",
                    "boot_id": "navd-boot-a",
                    "sequence": 9,
                    "task_id": task_id,
                    "request_id": "goal-attempt-persisted",
                    "state": int(NavigationGoalState.REACHED),
                    "goal_epoch": 1,
                    "reason": "goal_reached",
                }

        db_path = tmp_path / "tasks.sqlite3"
        first_commands = FakeCommands()
        first_service = GoalService(
            command_module="nav.commands",
            task_ledger_path=str(db_path),
        )
        first_service.on_system_modules({"nav.commands": first_commands})
        first_service.setup()
        first_service.submit_goal(
            PoseStamped(pose=Pose(position=Vector3(3.0, 4.0, 0.0)), frame_id="map"),
            task_id="task-persisted",
            request_id="goal-attempt-persisted",
        )
        first_service.stop()

        second_commands = FakeCommands(terminal=True)
        second_service = GoalService(
            command_module="nav.commands",
            task_ledger_path=str(db_path),
        )
        second_service.on_system_modules({"nav.commands": second_commands})
        second_service.setup()

        task = second_service.get_task("task-persisted")

        assert task is not None
        assert task["execution_state"] == "reached"
        assert task["terminal"] is True
        assert task["execution_reason"] == "goal_reached"
        assert second_commands.goals == []
        second_service.stop()

    def test_endpoint_boot_change_marks_evidence_without_replaying_motion(self, tmp_path):
        db_path = tmp_path / "tasks.sqlite3"
        ledger = NavigationTaskLedger(db_path, clock=lambda: 100.0)
        ledger.admit(
            "task-old-boot",
            "goal-old-boot",
            "goal",
            {"x": 1.0, "y": 2.0},
            target={"x": 1.0, "y": 2.0},
        )
        ledger.record_admission_result(
            "task-old-boot",
            "goal-old-boot",
            accepted=True,
            reason="planning_started",
            endpoint_boot_id="navd-boot-old",
        )
        ledger.close()

        class RestartedCommands:
            calls = 0

            def read_navigation_state(self):
                return {
                    "boot_id": "navd-boot-new",
                    "active_task_id": "",
                    "active_request_id": "",
                }

            def get_navigation_task_status(self, task_id):
                del task_id
                return None

            def send_goal(self, *args, **kwargs):
                self.calls += 1
                raise AssertionError("restart reconciliation must not replay motion")

        commands = RestartedCommands()
        service = GoalService(
            command_module="nav.commands",
            task_ledger_path=str(db_path),
        )
        service.on_system_modules({"nav.commands": commands})
        service.setup()

        task = service.get_task("task-old-boot")

        assert task is not None
        assert task["admission"] == "accepted"
        assert task["execution_state"] is None
        assert task["execution_reason"] == ""
        assert task["evidence_status"] == "boot_changed"
        assert task["terminal"] is False
        assert commands.calls == 0
        service.stop()

    def test_native_timeout_is_unconfirmed_and_replay_never_redispatches(self):
        class SideEffectThenTimeout:
            calls = 0

            def send_goal(self, *args, **kwargs):
                del args, kwargs
                self.calls += 1
                raise TimeoutError

        commands = SideEffectThenTimeout()
        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": commands})
        service.setup()
        goal = PoseStamped(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
            frame_id="map",
        )

        first = service.submit_goal(
            goal,
            task_id="task-timeout",
            request_id="request-timeout",
        )
        replay = service.submit_goal(
            goal,
            task_id="task-timeout",
            request_id="request-timeout",
        )
        task = service.get_task("task-timeout")

        assert commands.calls == 1
        assert first["accepted"] is False
        assert first["state"] == "unknown"
        assert first["admission_unconfirmed"] is True
        assert replay["replay"] is True
        assert replay["admission_confirmed"] is False
        assert replay["admission_unconfirmed"] is True
        assert task is not None
        assert task["admission"] == "unconfirmed"
        assert task["execution_state"] is None
        assert task["terminal"] is False
        assert task["attempts"][0]["accepted"] is None
        service.stop()

    def test_cancel_timeout_is_unconfirmed_and_replay_never_redispatches(self):
        class SideEffectThenTimeout:
            cancel_calls = 0

            def send_goal(self, x, y, z, yaw, *, task_id, request_id):
                del x, y, z, yaw
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="planning_started",
                )

            def cancel_task(self, task_id, reason, *, request_id):
                del task_id, reason, request_id
                self.cancel_calls += 1
                raise TimeoutError("ACK timeout after cancel write")

        commands = SideEffectThenTimeout()
        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": commands})
        service.setup()
        service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-cancel-timeout",
            request_id="goal-cancel-timeout",
        )

        first = service.submit_cancel(
            "operator",
            task_id="task-cancel-timeout",
            request_id="cancel-timeout",
        )
        replay = service.submit_cancel(
            "operator",
            task_id="task-cancel-timeout",
            request_id="cancel-timeout",
        )
        task = service.get_task("task-cancel-timeout")

        assert commands.cancel_calls == 1
        assert first["state"] == "unknown"
        assert first["admission_unconfirmed"] is True
        assert replay["replay"] is True
        assert replay["admission_confirmed"] is False
        assert replay["admission_unconfirmed"] is True
        assert task is not None
        assert task["terminal"] is False
        assert task["cancel_requested"] is False
        assert task["attempts"][-1]["accepted"] is None
        service.stop()

    def test_dispatch_success_survives_history_write_failure(self, monkeypatch):
        class FakeCommands:
            calls = 0

            def send_goal(self, x, y, z, yaw, *, task_id, request_id):
                del x, y, z, yaw
                self.calls += 1
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="planning_started",
                )

        commands = FakeCommands()
        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": commands})
        service.setup()

        def fail_history(_receipt):
            raise OSError("disk unavailable")

        monkeypatch.setattr(service._task_ledger, "record_native_ack", fail_history)
        result = service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-history-failure",
            request_id="request-history-failure",
        )

        assert commands.calls == 1
        assert result["accepted"] is True
        assert result["history_recorded"] is False
        assert "history" in result["history_warning"]
        service.stop()

    def test_history_admission_failure_prevents_motion_dispatch(self, monkeypatch):
        class FakeCommands:
            calls = 0

            def send_goal(self, *args, **kwargs):
                del args, kwargs
                self.calls += 1
                raise AssertionError("dispatch must not run")

        commands = FakeCommands()
        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": commands})
        service.setup()

        def fail_admission(*args, **kwargs):
            del args, kwargs
            raise OSError("ledger unavailable")

        monkeypatch.setattr(service._task_ledger, "admit", fail_admission)
        result = service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-no-history",
            request_id="request-no-history",
        )

        assert commands.calls == 0
        assert result["accepted"] is False
        assert "task history unavailable" in result["message"]
        service.stop()

    def test_explicit_native_rejection_is_admission_not_execution_terminal(self):
        class RejectingCommands:
            def send_goal(self, x, y, z, yaw, *, task_id, request_id):
                del x, y, z, yaw
                return NavigationCommandReceipt(
                    accepted=False,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="goal_outside_map",
                )

        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": RejectingCommands()})
        service.setup()
        result = service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-rejected",
            request_id="request-rejected",
        )
        task = service.get_task("task-rejected")

        assert result["accepted"] is False
        assert task is not None
        assert task["admission"] == "rejected"
        assert task["admission_reason"] == "goal_outside_map"
        assert task["execution_state"] is None
        assert task["terminal"] is False
        service.stop()

    def test_get_and_list_tasks_are_read_only(self):
        class Commands:
            def send_goal(self, x, y, z, yaw, *, task_id, request_id):
                del x, y, z, yaw
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="accepted",
                )

            def read_navigation_state(self):
                raise AssertionError("task reads must not poll native state")

            def get_navigation_task_status(self, task_id):
                del task_id
                raise AssertionError("task reads must not poll native status")

        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": Commands()})
        service.setup()
        service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-read-only",
            request_id="request-read-only",
        )
        before = service._task_ledger.get_task("task-read-only")

        detail = service.get_task("task-read-only")
        listed = service.list_tasks()
        after = service._task_ledger.get_task("task-read-only")

        assert detail == before
        assert listed == [before]
        assert after == before
        service.stop()

    def test_native_status_unavailable_does_not_change_execution_on_read(self):
        class UnavailableCommands:
            def send_goal(self, x, y, z, yaw, *, task_id, request_id):
                del x, y, z, yaw
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="planning_started",
                )

            def read_navigation_state(self):
                raise RuntimeError("status unavailable")

            def get_navigation_task_status(self, task_id):
                del task_id
                raise RuntimeError("status unavailable")

        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": UnavailableCommands()})
        service.setup()
        service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-status-unavailable",
            request_id="request-status-unavailable",
        )

        task = service.get_task("task-status-unavailable")

        assert task is not None
        assert task["admission"] == "accepted"
        assert task["execution_state"] is None
        assert task["execution_reason"] == ""
        assert task["evidence_status"] == "unavailable"
        assert task["terminal"] is False
        service.stop()

    def test_list_tasks_active_only_excludes_terminal_history(self):
        service = GoalService()
        service.setup()
        active = service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-active",
            request_id="request-active",
        )
        assert active["accepted"] is True
        service._task_ledger.admit(
            "task-terminal",
            "request-terminal",
            "goal",
            {"x": 3.0, "y": 4.0},
            target={"x": 3.0, "y": 4.0},
        )
        service._task_ledger.record_admission_result(
            "task-terminal",
            "request-terminal",
            accepted=False,
            reason="goal_outside_map",
        )

        tasks = service.list_tasks(active_only=True)

        assert [task["task_id"] for task in tasks] == ["task-active"]
        service.stop()

    def test_conflicting_reuse_of_request_identity_never_redispatches(self):
        class FakeCommands:
            calls = 0

            def send_goal(self, x, y, z, yaw, *, task_id, request_id):
                del x, y, z, yaw
                self.calls += 1
                return NavigationCommandReceipt(
                    accepted=True,
                    kind=int(NavigationCommandKind.GOAL),
                    task_id=task_id,
                    request_id=request_id,
                    endpoint_timestamp_s=100.0,
                    reason="planning_started",
                )

        commands = FakeCommands()
        service = GoalService(command_module="nav.commands")
        service.on_system_modules({"nav.commands": commands})
        service.setup()
        first = service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-conflict",
            request_id="request-conflict",
        )
        conflict = service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(9.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-conflict",
            request_id="request-conflict",
        )

        assert first["accepted"] is True
        assert conflict["accepted"] is False
        assert "conflict" in conflict["message"]
        assert commands.calls == 1
        service.stop()

    def test_local_cancel_is_persisted_nonterminal_and_replay_safe(self):
        service = GoalService()
        service.setup()
        cancellations: list[str] = []
        service.cancel.subscribe(cancellations.append)
        service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
                frame_id="map",
            ),
            task_id="task-local-cancel",
            request_id="goal-local-cancel",
        )

        first = service.submit_cancel(
            "operator",
            task_id="task-local-cancel",
            request_id="cancel-local-cancel",
        )
        status_count = service.goal_status.msg_count
        replay = service.submit_cancel(
            "operator",
            task_id="task-local-cancel",
            request_id="cancel-local-cancel",
        )
        task = service.get_task("task-local-cancel")

        assert cancellations == ["operator"]
        assert first["state"] == "cancel_requested"
        assert first["history_recorded"] is True
        assert replay["replay"] is True
        assert replay["state"] == "cancel_requested"
        assert service.goal_status.msg_count == status_count + 1
        assert task is not None
        assert task["cancel_requested"] is True
        assert task["terminal"] is False
        service.stop()
