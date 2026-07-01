"""Tests for PatrolManagerModule -- patrol route CRUD and start/stop.

All tests are pure-Python, no ROS2 / hardware required.
"""

from __future__ import annotations

import json
import os

import pytest

from runtime.msgs.nav import Odometry
from runtime.msgs.geometry import Pose
from nav.services.patrol import PatrolManagerModule


# -- fixtures ------------------------------------------------------------------


@pytest.fixture
def patrol_manager(tmp_path):
    """PatrolManagerModule with a temp routes directory."""
    routes_dir = tmp_path / "routes"
    mod = PatrolManagerModule(routes_dir=str(routes_dir))
    mod.setup()
    statuses: list[str] = []
    goals: list[list] = []
    mod.patrol_status.subscribe(lambda s: statuses.append(s))
    mod.patrol_goals.subscribe(lambda g: goals.append(g))
    mod._test_statuses = statuses
    mod._test_goals = goals
    return mod


def _cmd(mod, cmd: dict) -> dict:
    """Send a JSON command and return the last published status."""
    mod._test_statuses.clear()
    mod._on_command(json.dumps(cmd))
    assert len(mod._test_statuses) > 0, "no response published"
    return json.loads(mod._test_statuses[-1])


# -- tests ---------------------------------------------------------------------


class TestPatrolManagerModule:
    """Patrol route CRUD and patrol start/stop."""

    def test_instantiation(self, patrol_manager):
        """Module creates routes directory on init."""
        assert os.path.isdir(patrol_manager._routes_dir)
        assert patrol_manager._active_route is None

    def test_port_types(self, patrol_manager):
        """Verify In/Out port registration and types via port_summary."""
        s = patrol_manager.port_summary()
        assert s["ports_in"]["patrol_command"]["type"] == "str"
        assert s["ports_in"]["odometry"]["type"] == "Odometry"
        assert s["ports_out"]["patrol_goals"]["type"] == "list"
        assert s["ports_out"]["patrol_status"]["type"] == "str"

    def test_save_and_list(self, patrol_manager):
        """save a route and verify it appears in list."""
        wps = [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]
        resp = _cmd(patrol_manager, {"action": "save", "name": "route_a", "waypoints": wps})
        assert resp["success"] is True

        resp = _cmd(patrol_manager, {"action": "list"})
        assert resp["success"] is True
        assert len(resp["routes"]) == 1
        assert resp["routes"][0]["waypoint_count"] == 2

    def test_save_empty_waypoints(self, patrol_manager):
        """save with empty waypoints fails."""
        resp = _cmd(patrol_manager, {"action": "save", "name": "empty", "waypoints": []})
        assert resp["success"] is False

    def test_save_missing_name(self, patrol_manager):
        """save with empty name fails."""
        resp = _cmd(patrol_manager, {"action": "save", "name": "", "waypoints": [{"x": 0}]})
        assert resp["success"] is False

    def test_rejects_path_traversal_route_name(self, patrol_manager):
        resp = _cmd(
            patrol_manager,
            {"action": "save", "name": "../escape", "waypoints": [{"x": 0, "y": 0}]},
        )
        assert resp["success"] is False
        assert "invalid route name" in resp["message"]

    def test_load_route(self, patrol_manager):
        """save then load a route."""
        wps = [{"x": 1.0, "y": 2.0}]
        _cmd(patrol_manager, {"action": "save", "name": "r1", "waypoints": wps})
        resp = _cmd(patrol_manager, {"action": "load", "name": "r1"})
        assert resp["success"] is True
        assert "route" in resp

    def test_load_nonexistent(self, patrol_manager):
        """load a non-existent route fails."""
        resp = _cmd(patrol_manager, {"action": "load", "name": "nope"})
        assert resp["success"] is False

    def test_delete_route(self, patrol_manager):
        """save then delete a route."""
        _cmd(patrol_manager, {"action": "save", "name": "del_me", "waypoints": [{"x": 0}]})
        resp = _cmd(patrol_manager, {"action": "delete", "name": "del_me"})
        assert resp["success"] is True
        resp = _cmd(patrol_manager, {"action": "list"})
        assert len(resp["routes"]) == 0

    def test_delete_nonexistent(self, patrol_manager):
        """delete a non-existent route fails."""
        resp = _cmd(patrol_manager, {"action": "delete", "name": "ghost"})
        assert resp["success"] is False

    def test_start_patrol(self, patrol_manager):
        """start patrol publishes waypoints and sets active route."""
        wps = [{"x": 1.0, "y": 2.0}]
        _cmd(patrol_manager, {"action": "save", "name": "patrol1", "waypoints": wps})
        patrol_manager._test_goals.clear()
        resp = _cmd(patrol_manager, {"action": "start", "name": "patrol1"})
        assert resp["success"] is True
        assert len(patrol_manager._test_goals) == 1
        assert patrol_manager._active_route == "patrol1"

    def test_start_patrol_preserves_saved_loop_flag(self, patrol_manager):
        wps = [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]
        _cmd(
            patrol_manager,
            {"action": "save", "name": "loop1", "waypoints": wps, "loop": True},
        )
        patrol_manager._test_goals.clear()

        resp = _cmd(patrol_manager, {"action": "start", "name": "loop1"})

        assert resp["success"] is True
        assert patrol_manager._test_goals[-1][0]["loop"] is True
        assert patrol_manager._test_goals[-1][1]["loop"] is True

    def test_start_patrol_nonexistent(self, patrol_manager):
        """start on a non-existent route fails."""
        resp = _cmd(patrol_manager, {"action": "start", "name": "ghost"})
        assert resp["success"] is False

    def test_stop_patrol(self, patrol_manager):
        """stop patrol clears active route."""
        resp = _cmd(patrol_manager, {"action": "stop"})
        assert resp["success"] is True
        assert patrol_manager._active_route is None

    def test_on_odom_updates_pose(self, patrol_manager):
        """odometry updates internal current_pose."""
        odom = Odometry(pose=Pose(7.0, 8.0, 9.0))
        patrol_manager._on_odom(odom)
        assert patrol_manager._current_pose["x"] == 7.0
        assert patrol_manager._current_pose["y"] == 8.0
        assert patrol_manager._current_pose["z"] == 9.0

    def test_unknown_action(self, patrol_manager):
        """unknown action returns error."""
        resp = _cmd(patrol_manager, {"action": "xyz"})
        assert resp["success"] is False
