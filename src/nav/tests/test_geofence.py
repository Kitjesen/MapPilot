"""Tests for GeofenceManagerModule -- fence CRUD, point-in-polygon, intrusion.

All tests are pure-Python, no ROS2 / hardware required.
"""

from __future__ import annotations

import json

import pytest

from runtime.msgs.nav import Odometry
from runtime.msgs.geometry import Pose
from nav.services.geofence import GeofenceManagerModule


# -- fixtures ------------------------------------------------------------------


@pytest.fixture
def geofence_manager(tmp_path):
    """GeofenceManagerModule with a temp fence file."""
    fence_file = tmp_path / "fences.yaml"
    mod = GeofenceManagerModule(geofence_file=str(fence_file))
    mod.setup()
    alerts: list[dict] = []
    stops: list[int] = []
    mod.geofence_alert.subscribe(lambda a: alerts.append(a))
    mod.stop_cmd.subscribe(lambda s: stops.append(s))
    mod._test_alerts = alerts
    mod._test_stops = stops
    return mod


def _cmd(mod, cmd: dict) -> dict:
    """Send a JSON command and return the last published alert."""
    mod._test_alerts.clear()
    mod._on_command(json.dumps(cmd))
    assert len(mod._test_alerts) > 0, "no response published"
    return mod._test_alerts[-1]


# -- tests ---------------------------------------------------------------------


class TestGeofenceManagerModule:
    """Geofence CRUD, point-in-polygon, intrusion detection."""

    def test_instantiation(self, geofence_manager):
        """Module initialises with empty fences dict."""
        assert isinstance(geofence_manager._fences, dict)
        assert len(geofence_manager._fences) == 0

    def test_port_types(self, geofence_manager):
        """Verify In/Out port registration and types via port_summary."""
        s = geofence_manager.port_summary()
        assert s["ports_in"]["odometry"]["type"] == "Odometry"
        assert s["ports_in"]["geofence_command"]["type"] == "str"
        assert s["ports_out"]["geofence_alert"]["type"] == "dict"
        assert s["ports_out"]["stop_cmd"]["type"] == "int"

    def test_add_fence(self, geofence_manager):
        """add a valid polygon fence."""
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        resp = _cmd(geofence_manager, {"action": "add", "name": "zone1", "polygon": polygon})
        assert resp["success"] is True
        assert "zone1" in geofence_manager._fences

    def test_add_fence_too_few_vertices(self, geofence_manager):
        """add with fewer than 3 vertices fails."""
        resp = _cmd(geofence_manager, {"action": "add", "name": "bad", "polygon": [[0, 0], [1, 1]]})
        assert resp["success"] is False
        assert "vertices" in resp["message"]

    def test_add_fence_rejects_non_finite_vertex(self, geofence_manager):
        resp = _cmd(
            geofence_manager,
            {"action": "add", "name": "bad", "polygon": [[0, 0], [1, "nan"], [1, 1]]},
        )
        assert resp["success"] is False
        assert "finite" in resp["message"]

    def test_add_fence_missing_name(self, geofence_manager):
        """add with empty name fails."""
        resp = _cmd(
            geofence_manager,
            {"action": "add", "name": "", "polygon": [[0, 0], [1, 0], [1, 1]]},
        )
        assert resp["success"] is False

    def test_list_fences(self, geofence_manager):
        """list returns added fences."""
        _cmd(geofence_manager, {"action": "add", "name": "z1", "polygon": [[0, 0], [1, 0], [1, 1], [0, 1]]})
        resp = _cmd(geofence_manager, {"action": "list"})
        assert resp["success"] is True
        assert len(resp["fences"]) == 1
        assert resp["fences"][0]["vertex_count"] == 4

    def test_remove_fence(self, geofence_manager):
        """add then remove a fence."""
        _cmd(geofence_manager, {"action": "add", "name": "rm_me", "polygon": [[0, 0], [1, 0], [1, 1]]})
        resp = _cmd(geofence_manager, {"action": "remove", "name": "rm_me"})
        assert resp["success"] is True
        resp = _cmd(geofence_manager, {"action": "list"})
        assert len(resp["fences"]) == 0

    def test_remove_nonexistent(self, geofence_manager):
        """remove a non-existent fence fails."""
        resp = _cmd(geofence_manager, {"action": "remove", "name": "ghost"})
        assert resp["success"] is False

    def test_enable_disable(self, geofence_manager):
        """enable and disable a fence."""
        _cmd(geofence_manager, {"action": "add", "name": "f1", "polygon": [[0, 0], [1, 0], [1, 1]]})
        resp = _cmd(geofence_manager, {"action": "disable", "name": "f1"})
        assert resp["success"] is True
        assert geofence_manager._fences["f1"]["enabled"] is False

        resp = _cmd(geofence_manager, {"action": "enable", "name": "f1"})
        assert resp["success"] is True
        assert geofence_manager._fences["f1"]["enabled"] is True

    def test_clear_fences(self, geofence_manager):
        """clear removes all fences."""
        _cmd(geofence_manager, {"action": "add", "name": "a", "polygon": [[0, 0], [1, 0], [1, 1]]})
        _cmd(geofence_manager, {"action": "add", "name": "b", "polygon": [[0, 0], [1, 0], [1, 1]]})
        resp = _cmd(geofence_manager, {"action": "clear"})
        assert resp["success"] is True
        assert "2" in resp["message"]

    # -- point-in-polygon --

    def test_point_in_polygon_inside(self):
        """ray-casting detects a point inside the polygon."""
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        assert GeofenceManagerModule._point_in_polygon(5, 5, poly) is True

    def test_point_in_polygon_outside(self):
        """ray-casting detects a point outside the polygon."""
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        assert GeofenceManagerModule._point_in_polygon(15, 15, poly) is False

    def test_point_in_polygon_too_few(self):
        """polygon with fewer than 3 vertices returns False."""
        assert GeofenceManagerModule._point_in_polygon(0, 0, [[0, 0]]) is False

    # -- intrusion detection --

    def test_check_intrusion_inside(self, geofence_manager):
        """robot inside an enabled fence triggers intrusion alert."""
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        geofence_manager._fences["zone"] = {"polygon": polygon, "enabled": True}
        geofence_manager._on_odom(Odometry(pose=Pose(5.0, 5.0, 0.0)))
        geofence_manager._test_alerts.clear()
        alerts = geofence_manager.check_intrusion()
        assert len(alerts) == 1
        assert alerts[0]["type"] == "intrusion"

    def test_health_has_no_stop_side_effect(self, geofence_manager):
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        geofence_manager._fences["zone"] = {"polygon": polygon, "enabled": True}
        geofence_manager._on_odom(Odometry(pose=Pose(5.0, 5.0, 0.0)))
        geofence_manager._test_alerts.clear()
        geofence_manager._test_stops.clear()

        health = geofence_manager.health()

        assert health["violations"] == 1
        assert geofence_manager._test_alerts == []
        assert geofence_manager._test_stops == []

    def test_check_intrusion_outside(self, geofence_manager):
        """robot outside all fences triggers no alert."""
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        geofence_manager._fences["zone"] = {"polygon": polygon, "enabled": True}
        geofence_manager._on_odom(Odometry(pose=Pose(50.0, 50.0, 0.0)))
        alerts = geofence_manager.check_intrusion()
        assert len(alerts) == 0

    def test_check_intrusion_disabled_fence(self, geofence_manager):
        """disabled fence does not trigger intrusion."""
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        geofence_manager._fences["zone"] = {"polygon": polygon, "enabled": False}
        geofence_manager._on_odom(Odometry(pose=Pose(5.0, 5.0, 0.0)))
        alerts = geofence_manager.check_intrusion()
        assert len(alerts) == 0

    def test_unknown_action(self, geofence_manager):
        """unknown action returns error."""
        resp = _cmd(geofence_manager, {"action": "foobar"})
        assert resp["success"] is False
