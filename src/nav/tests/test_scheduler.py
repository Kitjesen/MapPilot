"""Tests for TaskSchedulerModule -- schedule CRUD, firing logic, dedup.

All tests are pure-Python, no ROS2 / hardware required.
"""

from __future__ import annotations

import json
from datetime import datetime

import pytest

from nav.services.scheduler import TaskSchedulerModule


# -- fixtures ------------------------------------------------------------------


@pytest.fixture
def task_scheduler(tmp_path):
    """TaskSchedulerModule with a temp schedule file."""
    sched_file = tmp_path / "schedules.yaml"
    mod = TaskSchedulerModule(schedule_file=str(sched_file))
    mod.setup()
    tasks: list[dict] = []
    patrol_commands: list[str] = []
    mod.scheduled_task.subscribe(lambda t: tasks.append(t))
    mod.patrol_command.subscribe(lambda c: patrol_commands.append(c))
    mod._test_tasks = tasks
    mod._test_patrol_commands = patrol_commands
    return mod


def _cmd(mod, cmd: dict) -> dict:
    """Send a JSON command and return the last published task."""
    mod._test_tasks.clear()
    mod._on_command(json.dumps(cmd))
    assert len(mod._test_tasks) > 0, "no response published"
    return mod._test_tasks[-1]


# -- tests ---------------------------------------------------------------------


class TestTaskSchedulerModule:
    """Schedule CRUD, firing, and double-fire prevention."""

    def test_instantiation(self, task_scheduler):
        """Module initialises with empty schedules dict."""
        assert isinstance(task_scheduler._schedules, dict)
        assert len(task_scheduler._schedules) == 0

    def test_port_types(self, task_scheduler):
        """Verify In/Out port registration and types via port_summary."""
        s = task_scheduler.port_summary()
        assert s["ports_in"]["schedule_command"]["type"] == "str"
        assert s["ports_out"]["scheduled_task"]["type"] == "dict"
        assert s["ports_out"]["patrol_command"]["type"] == "str"

    def test_add_and_list(self, task_scheduler):
        """add a schedule and verify it appears in list."""
        resp = _cmd(
            task_scheduler,
            {"action": "add", "name": "morning", "patrol_route": "route_a", "hour": 8, "minute": 0},
        )
        assert resp["success"] is True

        resp = _cmd(task_scheduler, {"action": "list"})
        assert resp["success"] is True
        assert len(resp["schedules"]) == 1
        assert resp["schedules"][0]["patrol_route"] == "route_a"

    def test_add_missing_name(self, task_scheduler):
        """add with empty name fails."""
        resp = _cmd(task_scheduler, {"action": "add", "name": "", "patrol_route": "r"})
        assert resp["success"] is False

    def test_add_missing_route(self, task_scheduler):
        """add with empty patrol_route fails."""
        resp = _cmd(task_scheduler, {"action": "add", "name": "s", "patrol_route": ""})
        assert resp["success"] is False

    def test_remove_schedule(self, task_scheduler):
        """add then remove a schedule."""
        _cmd(task_scheduler, {"action": "add", "name": "temp", "patrol_route": "r"})
        resp = _cmd(task_scheduler, {"action": "remove", "name": "temp"})
        assert resp["success"] is True
        resp = _cmd(task_scheduler, {"action": "list"})
        assert len(resp["schedules"]) == 0

    def test_remove_nonexistent(self, task_scheduler):
        """remove a non-existent schedule fails."""
        resp = _cmd(task_scheduler, {"action": "remove", "name": "ghost"})
        assert resp["success"] is False

    def test_enable_disable(self, task_scheduler):
        """enable and disable a schedule."""
        _cmd(task_scheduler, {"action": "add", "name": "s1", "patrol_route": "r1"})
        resp = _cmd(task_scheduler, {"action": "disable", "name": "s1"})
        assert resp["success"] is True
        assert task_scheduler._schedules["s1"]["enabled"] is False

        resp = _cmd(task_scheduler, {"action": "enable", "name": "s1"})
        assert resp["success"] is True
        assert task_scheduler._schedules["s1"]["enabled"] is True

    def test_enable_nonexistent(self, task_scheduler):
        """enable a non-existent schedule fails."""
        resp = _cmd(task_scheduler, {"action": "enable", "name": "nope"})
        assert resp["success"] is False

    def test_check_schedules_fires(self, task_scheduler):
        """check_schedules fires a matching schedule."""
        task_scheduler._schedules["test"] = {
            "patrol_route": "r_test",
            "hour": 14,
            "minute": 30,
            "weekdays": [0, 1, 2, 3, 4, 5, 6],
            "enabled": True,
        }
        task_scheduler._test_tasks.clear()
        task_scheduler._test_patrol_commands.clear()
        now = datetime(2026, 4, 6, 14, 30, 0)  # Monday
        fired = task_scheduler.check_schedules(now=now)
        assert len(fired) == 1
        assert fired[0]["patrol_route"] == "r_test"
        assert fired[0]["event"] == "schedule_fired"
        assert fired[0]["patrol_command"] == {"action": "start", "name": "r_test"}
        assert json.loads(task_scheduler._test_patrol_commands[-1]) == {
            "action": "start",
            "name": "r_test",
        }

    def test_check_schedules_no_double_fire(self, task_scheduler):
        """check_schedules does not fire the same schedule twice in the same minute."""
        task_scheduler._schedules["test"] = {
            "patrol_route": "r_test",
            "hour": 14,
            "minute": 30,
            "weekdays": [0, 1, 2, 3, 4, 5, 6],
            "enabled": True,
        }
        now = datetime(2026, 4, 6, 14, 30, 0)
        task_scheduler.check_schedules(now=now)
        task_scheduler._test_tasks.clear()
        fired = task_scheduler.check_schedules(now=now)
        assert len(fired) == 0

    def test_check_schedules_disabled_skipped(self, task_scheduler):
        """disabled schedules are not fired."""
        task_scheduler._schedules["test"] = {
            "patrol_route": "r_test",
            "hour": 14,
            "minute": 30,
            "weekdays": [0, 1, 2, 3, 4, 5, 6],
            "enabled": False,
        }
        now = datetime(2026, 4, 6, 14, 30, 0)
        fired = task_scheduler.check_schedules(now=now)
        assert len(fired) == 0

    def test_check_schedules_wrong_weekday(self, task_scheduler):
        """schedule does not fire on a non-matching weekday."""
        task_scheduler._schedules["test"] = {
            "patrol_route": "r_test",
            "hour": 14,
            "minute": 30,
            "weekdays": [2],  # Wednesday only
            "enabled": True,
        }
        now = datetime(2026, 4, 6, 14, 30, 0)  # Monday (weekday=0)
        fired = task_scheduler.check_schedules(now=now)
        assert len(fired) == 0

    def test_unknown_action(self, task_scheduler):
        """unknown action returns error."""
        resp = _cmd(task_scheduler, {"action": "xyz"})
        assert resp["success"] is False
