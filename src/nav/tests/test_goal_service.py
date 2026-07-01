"""Tests for nav.goals.

All tests are pure Python and do not require ROS2 or hardware.
"""

from __future__ import annotations

import json

import pytest

from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from nav.services.goals import GoalService


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

    def test_invalid_json_reports_error(self, goal_service):
        goal_service._on_command("not-json")

        assert goal_service._test_statuses[-1]["success"] is False
        assert goal_service._test_goals == []
