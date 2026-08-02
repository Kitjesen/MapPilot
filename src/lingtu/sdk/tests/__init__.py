"""Test LingTu SDK client with mocked HTTP."""

from __future__ import annotations

import json
import unittest
import urllib.error
from unittest.mock import patch

import lingtu.sdk as sdk_pkg
from lingtu.sdk import (
    CommandResult,
    LingTuClient,
    MapList,
    NavigationStatus,
    Position,
    RobotState,
    SessionInfo,
)


class TestLingTuClient(unittest.TestCase):
    """Suite of mocked HTTP tests for the sync client."""

    def setUp(self) -> None:
        self.robot = LingTuClient()

    def test_public_all_keeps_optional_async_out_of_star_import(self) -> None:
        self.assertNotIn("AsyncLingTuClient", sdk_pkg.__all__)
        self.assertIn("LingTuClient", sdk_pkg.__all__)

    def _mock_http(self, mock_urlopen, response: dict) -> None:
        """Set up a mock that returns the given JSON dict."""
        mock_urlopen.return_value.__enter__.return_value.read.return_value = (
            json.dumps(response).encode("utf-8")
        )

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_go(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True, "message": "goal set"})
        r = self.robot.go(10.0, 5.0)
        self.assertIsInstance(r, CommandResult)
        self.assertTrue(r.ok)
        self.assertEqual(r.message, "goal set")

    @patch("urllib.request.urlopen")
    def test_go_to(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.go_to("meeting room")
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_stop(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.stop()
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_cancel(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.cancel()
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_navigate_click(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.navigate_click(10.0, 5.0)
        self.assertIsInstance(r, CommandResult)

    # ------------------------------------------------------------------
    # Drive
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_drive(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.drive(vx=0.5, wz=0.3)
        self.assertIsInstance(r, CommandResult)

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_state(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {
            "mode": "autonomous",
            "odometry": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 1.57},
            "mission_state": "navigating",
        })
        s = self.robot.state()
        self.assertIsInstance(s, RobotState)
        self.assertEqual(s.mode, "autonomous")
        self.assertIsInstance(s.odometry, Position)
        self.assertEqual(s.odometry.x, 1.0)
        self.assertEqual(s.odometry.y, 2.0)

    @patch("urllib.request.urlopen")
    def test_position(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {
            "mode": "idle",
            "odometry": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        })
        p = self.robot.position()
        self.assertIsInstance(p, Position)
        self.assertEqual(p.x, 1.0)
        self.assertEqual(p.y, 2.0)

    @patch("urllib.request.urlopen")
    def test_health(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {
            "modules_ok": 8,
            "modules_fail": 0,
            "slam_rate": 10.0,
            "mode": "autonomous",
            "session": "navigating",
        })
        h = self.robot.health()
        self.assertEqual(h.modules_ok, 8)
        self.assertEqual(h.modules_total, 8)

    @patch("urllib.request.urlopen")
    def test_health_unreachable_returns_error_payload(self, mock_urlopen) -> None:
        mock_urlopen.side_effect = urllib.error.URLError("connection refused")

        h = self.robot.health()

        self.assertEqual(h.modules_ok, 0)
        self.assertEqual(h.raw["error"], "robot not reachable")
        self.assertIn("/api/v1/health", h.raw["path"])

    @patch("urllib.request.urlopen")
    def test_session(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {
            "mode": "navigating",
            "active_map": "factory_01",
            "slam_profile": "localizer",
        })
        s = self.robot.session()
        self.assertIsInstance(s, SessionInfo)
        self.assertEqual(s.mode, "navigating")
        self.assertEqual(s.active_map, "factory_01")

    # ------------------------------------------------------------------
    # Navigation status
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_navigation_status(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {
            "state": "navigating",
            "distance_to_goal": 5.0,
            "time_elapsed": 3.5,
            "goal": {"x": 10.0, "y": 5.0, "z": 0.0, "yaw": 0.0},
        })
        ns = self.robot.navigation_status()
        self.assertIsInstance(ns, NavigationStatus)
        self.assertEqual(ns.state, "navigating")
        self.assertEqual(ns.distance_to_goal, 5.0)
        self.assertEqual(ns.goal.x, 10.0)

    # ------------------------------------------------------------------
    # Maps
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_maps(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {
            "active_map": "factory_01",
            "maps": [
                {"name": "factory_01", "has_pcd": True, "is_active": True},
                {"name": "lab_01", "has_pcd": True, "is_active": False},
            ],
        })
        ml = self.robot.maps()
        self.assertIsInstance(ml, MapList)
        self.assertEqual(ml.active_map, "factory_01")
        self.assertEqual(len(ml.maps), 2)
        self.assertEqual(ml.maps[0].name, "factory_01")
        self.assertTrue(ml.maps[0].has_pcd)

    # ------------------------------------------------------------------
    # Save / use / rename map
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_save_map(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True, "message": "map saved"})
        r = self.robot.save_map("test_map")
        self.assertTrue(r.ok)

    @patch("urllib.request.urlopen")
    def test_use_map(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.use_map("test_map")
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_rename_map(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.rename_map("old", "new")
        self.assertIsInstance(r, CommandResult)

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_set_mode(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.set_mode("autonomous")
        self.assertIsInstance(r, CommandResult)

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_scene(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"objects": []})
        sg = self.robot.scene()
        self.assertIsInstance(sg, dict)
        self.assertEqual(sg.get("objects"), [])

    @patch("urllib.request.urlopen")
    def test_locations(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"locations": []})
        locs = self.robot.locations()
        self.assertIsInstance(locs, dict)

    @patch("urllib.request.urlopen")
    def test_tag_location(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.tag_location("test", use_current_pose=True)
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_delete_location(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.delete_location("test")
        self.assertIsInstance(r, CommandResult)

    # ------------------------------------------------------------------
    # Camera
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_camera_snapshot(self, mock_urlopen) -> None:
        mock_urlopen.return_value.__enter__.return_value.read.return_value = b"fake_jpeg_bytes"
        jpg = self.robot.camera_snapshot()
        self.assertEqual(jpg, b"fake_jpeg_bytes")

    # ------------------------------------------------------------------
    # SLAM
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_slam_status(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"state": "mapping"})
        st = self.robot.slam_status()
        self.assertEqual(st.get("state"), "mapping")

    @patch("urllib.request.urlopen")
    def test_slam_switch(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.slam_switch("localizer")
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_slam_relocalize(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.slam_relocalize()
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_slam_auto_relocalize(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.slam_auto_relocalize()
        self.assertIsInstance(r, CommandResult)

    # ------------------------------------------------------------------
    # Exploration
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_explore(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.explore_start()
        self.assertIsInstance(r, CommandResult)
        r = self.robot.explore_stop()
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_explore_status(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"active": True})
        st = self.robot.explore_status()
        self.assertTrue(st.get("active"))

    # ------------------------------------------------------------------
    # Bag
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_recording(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.recording_start("test")
        self.assertIsInstance(r, CommandResult)
        r = self.robot.recording_stop()
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_recording_status(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"recording": True})
        st = self.robot.recording_status()
        self.assertTrue(st.get("recording"))

    # ------------------------------------------------------------------
    # Memory
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_memory_temporal(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"observations": []})
        mem = self.robot.memory_temporal()
        self.assertIsInstance(mem, dict)

    @patch("urllib.request.urlopen")
    def test_memory_temporal_semantic(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"results": []})
        mem = self.robot.memory_temporal_semantic("where was the backpack?")
        self.assertIsInstance(mem, dict)

    # ------------------------------------------------------------------
    # Lease
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_lease(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True, "message": "acquired"})
        r = self.robot.acquire_lease("my_app")
        self.assertIsInstance(r, CommandResult)
        self.assertTrue(r.ok)

    @patch("urllib.request.urlopen")
    def test_renew_release_lease(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.renew_lease("my_app")
        self.assertIsInstance(r, CommandResult)
        r = self.robot.release_lease("my_app")
        self.assertIsInstance(r, CommandResult)

    # ------------------------------------------------------------------
    # Session lifecycle
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_start_end_session(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True, "message": "started"})
        r = self.robot.start_session("navigating", map_name="factory_01")
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_end_session(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.end_session()
        self.assertIsInstance(r, CommandResult)

    # ------------------------------------------------------------------
    # Driver swap
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_swap_driver(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {
            "success": True, "message": "Swapped to stub", "swap_time_ms": 42,
        })
        r = self.robot.swap_driver("stub")
        self.assertIsInstance(r, CommandResult)
        self.assertTrue(r.ok)

    @patch("urllib.request.urlopen")
    def test_swap_alias(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"success": True, "message": "ok"})
        r = self.robot.swap("stub")
        self.assertIsInstance(r, CommandResult)

    # ------------------------------------------------------------------
    # Switch backend
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_switch_backend(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.switch_backend("planner", "pct")
        self.assertIsInstance(r, CommandResult)

    # ------------------------------------------------------------------
    # App endpoints
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_capabilities(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"version": "1.0"})
        caps = self.robot.capabilities()
        self.assertEqual(caps.get("version"), "1.0")

    @patch("urllib.request.urlopen")
    def test_bootstrap(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"mode": "idle"})
        b = self.robot.bootstrap()
        self.assertIsInstance(b, dict)

    @patch("urllib.request.urlopen")
    def test_devices(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"devices": []})
        d = self.robot.devices()
        self.assertIsInstance(d, dict)

    @patch("urllib.request.urlopen")
    def test_readiness(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ready": True})
        r = self.robot.readiness()
        self.assertIsInstance(r, dict)

    # ------------------------------------------------------------------
    # Auth
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_auth_login(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True, "token": "abc"})
        r = self.robot.auth_login("my-key")
        self.assertEqual(r.get("token"), "abc")

    @patch("urllib.request.urlopen")
    def test_auth_check(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"auth_required": True})
        ac = self.robot.auth_check()
        self.assertTrue(ac.get("auth_required"))

    # ------------------------------------------------------------------
    # Map operations
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_restore_map(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.restore_map("test")
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_reset_map_cloud(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.reset_map_cloud()
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_map_points(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"points": [], "count": 0})
        pts = self.robot.map_points()
        self.assertEqual(pts.get("count"), 0)

    # ------------------------------------------------------------------
    # Navigation status / path / localization
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_localization_status(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"fix_quality": "fixed"})
        loc = self.robot.localization_status()
        self.assertEqual(loc.get("fix_quality"), "fixed")

    @patch("urllib.request.urlopen")
    def test_path(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"path": []})
        p = self.robot.path()
        self.assertEqual(p.get("path"), [])

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_field_check(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True, "message": "all good"})
        r = self.robot.field_check()
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_command_unreachable_returns_failed_result(self, mock_urlopen) -> None:
        mock_urlopen.side_effect = urllib.error.URLError("no route to host")

        r = self.robot.go(1.0, 2.0)

        self.assertIsInstance(r, CommandResult)
        self.assertFalse(r.ok)
        self.assertEqual(r.message, "robot not reachable")
        self.assertEqual(r.raw["path"], "/api/v1/goal")

    @patch("urllib.request.urlopen")
    def test_runtime_contract(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"contract_version": 1})
        rc = self.robot.runtime_contract()
        self.assertIsInstance(rc, dict)

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_context_manager(self, mock_urlopen) -> None:
        with LingTuClient() as r:
            self.assertIsNotNone(r)


if __name__ == "__main__":
    unittest.main()
