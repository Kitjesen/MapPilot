"""Test LingTu SDK client with mocked HTTP."""

from __future__ import annotations

import json
import unittest
import urllib.error
from pathlib import Path
from unittest.mock import Mock, call, patch

import lingtu.sdk as sdk_pkg
from lingtu.sdk import (
    CommandResult,
    LingTuClient,
    MapList,
    NavigationStatus,
    Pose2D,
    Position,
    RobotState,
    SessionInfo,
)


class TestLingTuClient(unittest.TestCase):
    """Suite of mocked HTTP tests for the sync client."""

    def setUp(self) -> None:
        self.robot = LingTuClient()

    def test_public_all_exports_both_clients(self) -> None:
        self.assertIn("AsyncLingTuClient", sdk_pkg.__all__)
        self.assertIn("LingTuClient", sdk_pkg.__all__)
        self.assertIn("LocalizationClient", sdk_pkg.__all__)
        self.assertIn("AsyncLocalizationClient", sdk_pkg.__all__)
        self.assertIn("Pose2D", sdk_pkg.__all__)

    def test_removed_direct_runtime_mutations_are_not_sdk_methods(self) -> None:
        for name in ("use_map", "restore_map", "start_session", "end_session", "drive"):
            self.assertFalse(hasattr(LingTuClient, name), name)
        async_source = (
            Path(__file__).resolve().parents[3] / "src/lingtu/sdk/async_client.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("async def use_map", async_source)
        self.assertNotIn("async def restore_map", async_source)
        self.assertNotIn("async def start_session", async_source)
        self.assertNotIn("async def end_session", async_source)
        self.assertNotIn("async def drive", async_source)

    def test_removed_slam_relocalization_aliases_are_not_sdk_methods(self) -> None:
        self.assertFalse(hasattr(LingTuClient, "slam_relocalize"))
        self.assertFalse(hasattr(LingTuClient, "slam_auto_relocalize"))

    def _mock_http(self, mock_urlopen, response: dict) -> None:
        """Set up a mock that returns the given JSON dict."""
        mock_urlopen.return_value.__enter__.return_value.read.return_value = (
            json.dumps(response).encode("utf-8")
        )

    def test_simple_get_routes(self) -> None:
        routes = (
            (self.robot.scene, "/api/v1/scene_graph"),
            (self.robot.locations, "/api/v1/locations"),
            (self.robot.slam_status, "/api/v1/slam/status"),
            (self.robot.explore_status, "/api/v1/explore/status"),
            (self.robot.recording_status, "/api/v1/recordings/status"),
            (self.robot.memory_temporal, "/api/v1/memory/temporal"),
            (self.robot.capabilities, "/api/v1/app/capabilities"),
            (self.robot.bootstrap, "/api/v1/app/bootstrap"),
            (self.robot.readiness, "/api/v1/readiness"),
            (self.robot.auth_check, "/api/v1/auth/check"),
            (self.robot.map_points, "/api/v1/map/points"),
            (self.robot.localization_status, "/api/v1/localization/status"),
            (self.robot.path, "/api/v1/path"),
            (self.robot.runtime_contract, "/api/v1/diagnostics/runtime-contract"),
        )
        for invoke, path in routes:
            with self.subTest(path=path):
                self.robot._get = Mock(return_value={})
                self.assertEqual(invoke(), {})
                self.robot._get.assert_called_once_with(path)

    def test_simple_command_routes(self) -> None:
        routes = (
            (lambda: self.robot.rename_map("old", "new"), "/api/v1/map/rename", {"old_name": "old", "new_name": "new"}),
            (lambda: self.robot.set_mode("autonomous"), "/api/v1/mode", {"mode": "autonomous"}),
            (
                lambda: self.robot.follow_person("person_01"),
                "/api/v1/visual_servo",
                {"mode": "follow", "target_id": "person_01"},
            ),
            (
                self.robot.stop_following,
                "/api/v1/visual_servo",
                {"mode": "stop"},
            ),
            (lambda: self.robot.delete_location("test"), "/api/v1/locations/test", None),
            (self.robot.explore_start, "/api/v1/explore/start", None),
            (self.robot.explore_stop, "/api/v1/explore/stop", None),
            (self.robot.reset_map_cloud, "/api/v1/map_cloud/reset", None),
            (self.robot.field_check, "/api/v1/diagnostics/field-check", None),
        )
        for invoke, path, payload in routes:
            with self.subTest(path=path):
                expected = CommandResult(ok=True)
                self.robot._command = Mock(return_value=expected)
                self.assertIs(invoke(), expected)
                if payload is None:
                    self.robot._command.assert_called_once_with(path)
                else:
                    self.robot._command.assert_called_once_with(path, payload)

    def test_slam_hot_switch_is_not_exposed(self) -> None:
        self.assertFalse(hasattr(self.robot, "slam_switch"))

    def test_seeded_relocalization_uses_domain_api(self) -> None:
        expected = CommandResult(ok=True)
        self.robot._command = Mock(return_value=expected)

        result = self.robot.localization.relocalize(
            "factory",
            initial_pose=Pose2D(x=1.0, y=2.0, yaw=0.5),
        )

        self.assertIs(result, expected)
        self.robot._command.assert_called_once_with(
            "/api/v1/localization/relocalizations",
            {
                "mode": "seeded",
                "map_name": "factory",
                "initial_pose": {"x": 1.0, "y": 2.0, "yaw": 0.5},
            },
        )

    def test_global_relocalization_uses_domain_api(self) -> None:
        expected = CommandResult(ok=True)
        self.robot._command = Mock(return_value=expected)

        result = self.robot.localization.global_relocalize("factory")

        self.assertIs(result, expected)
        self.robot._command.assert_called_once_with(
            "/api/v1/localization/relocalizations",
            {"mode": "global", "map_name": "factory"},
        )

    def test_map_tracking_uses_domain_api(self) -> None:
        expected = CommandResult(ok=True)
        self.robot._command = Mock(return_value=expected)

        result = self.robot.localization.start_map_tracking("factory")

        self.assertIs(result, expected)
        self.robot._command.assert_called_once_with(
            "/api/v1/localization/map-tracking",
            {"map_name": "factory"},
        )

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_go(self, mock_urlopen) -> None:
        self._mock_http(
            mock_urlopen,
            {
                "ok": True,
                "status": "planning_started",
                "message": "goal set",
                "command": {"accepted": True, "request_id": "goal-123"},
            },
        )
        r = self.robot.go(10.0, 5.0, request_id="goal-123")
        self.assertIsInstance(r, CommandResult)
        self.assertTrue(r.ok)
        self.assertTrue(r.accepted)
        self.assertEqual(r.message, "goal set")
        self.assertEqual(r.request_id, "goal-123")
        self.assertEqual(r.stage, "planning_started")

        request = mock_urlopen.call_args.args[0]
        self.assertEqual(json.loads(request.data), {
            "x": 10.0,
            "y": 5.0,
            "yaw": 0.0,
            "request_id": "goal-123",
        })

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
    def test_health_unreachable_is_not_reported_as_zero_health(self, mock_urlopen) -> None:
        mock_urlopen.side_effect = urllib.error.URLError("connection refused")

        with self.assertRaises(ConnectionError):
            self.robot.health()

    @patch("urllib.request.urlopen")
    def test_session(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {
            "mode": "navigating",
            "active_map": "factory_01",
            "slam_profile": "native_dds",
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
            "state": "EXECUTING",
            "target": {
                "distance_to_goal_m": 5.0,
                "goal": {"x": 10.0, "y": 5.0, "z": 0.0, "yaw": 0.0},
            },
            "mission": {"raw": {"time_elapsed": 3.5}},
        })
        ns = self.robot.navigation_status()
        self.assertIsInstance(ns, NavigationStatus)
        self.assertEqual(ns.state, "EXECUTING")
        self.assertEqual(ns.distance_to_goal, 5.0)
        self.assertEqual(ns.time_elapsed, 3.5)
        self.assertEqual(ns.goal.x, 10.0)

    @patch("urllib.request.urlopen")
    def test_navigation_status_missing_distance_stays_unknown(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {
            "state": "IDLE",
            "target": {"goal": None, "distance_to_goal_m": None},
        })

        ns = self.robot.navigation_status()

        self.assertIsNone(ns.distance_to_goal)

    def test_wait_until_arrived_ignores_stale_terminal_until_new_mission_is_active(self) -> None:
        baseline = NavigationStatus(
            state="SUCCESS",
            goal=Position(10.0, 5.0),
            raw={
                "mission": {"raw": {"ts": 1.0}},
                "target": {"goal": {"x": 10.0, "y": 5.0}},
            },
        )
        stale = NavigationStatus(
            state="SUCCESS",
            goal=Position(10.0, 5.0),
            raw={
                "mission": {"raw": {"ts": 1.0}},
                "target": {"goal": {"x": 10.0, "y": 5.0}},
            },
        )
        active = NavigationStatus(
            state="PLANNING",
            goal=Position(10.0, 5.0),
            raw={
                "mission": {"raw": {"ts": 2.0}},
                "target": {"goal": {"x": 10.0, "y": 5.0}},
            },
        )
        completed = NavigationStatus(
            state="SUCCESS",
            distance_to_goal=0.0,
            goal=Position(10.0, 5.0),
            raw={
                "mission": {"raw": {"ts": 3.0}},
                "target": {"goal": {"x": 10.0, "y": 5.0}},
            },
        )
        self.robot.navigation_status = Mock(side_effect=[stale, active, completed])

        result = self.robot.wait_until_arrived(
            timeout=1.0,
            poll_interval=0.0,
            expected_goal=(10.0, 5.0, 0.0),
            baseline=baseline,
        )

        self.assertIs(result, completed)
        self.assertEqual(self.robot.navigation_status.call_count, 3)

    def test_wait_until_arrived_ignores_unrelated_terminal_transition(self) -> None:
        baseline = NavigationStatus(
            state="IDLE",
            raw={"mission": {"raw": {"ts": 1.0}}},
        )
        unrelated = NavigationStatus(
            state="SUCCESS",
            goal=Position(99.0, 99.0),
            raw={
                "mission": {"raw": {"ts": 2.0}},
                "target": {"goal": {"x": 99.0, "y": 99.0}},
            },
        )
        wrong_yaw = NavigationStatus(
            state="SUCCESS",
            goal=Position(7.0, 8.0, yaw=1.5),
            raw={
                "mission": {"raw": {"ts": 2.5}},
                "target": {"goal": {"x": 7.0, "y": 8.0, "yaw": 1.5}},
            },
        )
        active = NavigationStatus(
            state="EXECUTING",
            goal=Position(7.0, 8.0, yaw=0.0),
            raw={
                "mission": {"raw": {"ts": 3.0}},
                "target": {"goal": {"x": 7.0, "y": 8.0, "yaw": 0.0}},
            },
        )
        completed = NavigationStatus(
            state="SUCCESS",
            goal=Position(7.0, 8.0, yaw=0.0),
            raw={
                "mission": {"raw": {"ts": 4.0}},
                "target": {"goal": {"x": 7.0, "y": 8.0, "yaw": 0.0}},
            },
        )
        self.robot.navigation_status = Mock(
            side_effect=[unrelated, wrong_yaw, active, completed]
        )

        result = self.robot.wait_until_arrived(
            timeout=1.0,
            poll_interval=0.0,
            expected_goal=(7.0, 8.0, 0.0),
            baseline=baseline,
        )

        self.assertIs(result, completed)
        self.assertEqual(self.robot.navigation_status.call_count, 4)

    @patch(
        "lingtu.sdk.client.time.monotonic",
        side_effect=[0.0, 0.1, 0.2, 0.3, 1.1],
    )
    def test_wait_until_arrived_requires_explicit_success(
        self,
        _monotonic,
    ) -> None:
        self.robot.navigation_status = Mock(
            side_effect=[
                NavigationStatus(state="EXECUTING"),
                NavigationStatus(state="IDLE", distance_to_goal=0.0),
                NavigationStatus(state="ARRIVED", distance_to_goal=0.0),
            ]
        )

        with self.assertRaises(TimeoutError):
            self.robot.wait_until_arrived(
                timeout=1.0,
                poll_interval=0.0,
                expected_goal=(7.0, 8.0, 0.0),
            )

    def test_wait_until_arrived_raises_for_terminal_failure(self) -> None:
        for state in ("FAILED", "CANCELLED"):
            with self.subTest(state=state):
                self.robot.navigation_status = Mock(
                    side_effect=[
                        NavigationStatus(state="EXECUTING", goal=Position(2.0, 3.0)),
                        NavigationStatus(
                            state=state,
                            goal=Position(2.0, 3.0),
                            raw={
                                "failure_reason": "planner_failed",
                                "target": {"goal": {"x": 2.0, "y": 3.0}},
                            },
                        ),
                    ]
                )

                with self.assertRaisesRegex(RuntimeError, f"{state}.*planner_failed"):
                    self.robot.wait_until_arrived(
                        timeout=1.0,
                        poll_interval=0.0,
                        expected_goal=(2.0, 3.0, 0.0),
                    )
    def test_wait_until_arrived_accepts_immediate_success_with_matching_request_id(self) -> None:
        baseline = NavigationStatus(
            state="IDLE",
            raw={"mission": {"raw": {"ts": 1.0}}},
        )
        completed = NavigationStatus(
            state="SUCCESS",
            goal=Position(7.0, 8.0),
            request_id="goal-1",
            raw={
                "request_id": "goal-1",
                "mission": {"raw": {"ts": 2.0}},
                "target": {"goal": {"x": 7.0, "y": 8.0, "yaw": 0.0}},
            },
        )
        self.robot.navigation_status = Mock(return_value=completed)

        result = self.robot.wait_until_arrived(
            timeout=1.0,
            poll_interval=0.0,
            request_id="goal-1",
            expected_goal=(7.0, 8.0, 0.0),
            baseline=baseline,
        )

        self.assertIs(result, completed)

    def test_wait_until_arrived_ignores_same_goal_terminal_for_other_request(self) -> None:
        baseline = NavigationStatus(
            state="IDLE",
            raw={"mission": {"raw": {"ts": 1.0}}},
        )
        other = NavigationStatus(
            state="SUCCESS",
            goal=Position(7.0, 8.0),
            request_id="goal-other",
            raw={
                "request_id": "goal-other",
                "mission": {"raw": {"ts": 2.0}},
                "target": {"goal": {"x": 7.0, "y": 8.0, "yaw": 0.0}},
            },
        )
        active = NavigationStatus(
            state="EXECUTING",
            goal=Position(7.0, 8.0),
            request_id="goal-1",
            raw={
                "request_id": "goal-1",
                "mission": {"raw": {"ts": 3.0}},
                "target": {"goal": {"x": 7.0, "y": 8.0, "yaw": 0.0}},
            },
        )
        completed = NavigationStatus(
            state="SUCCESS",
            goal=Position(7.0, 8.0),
            request_id="goal-1",
            raw={
                "request_id": "goal-1",
                "mission": {"raw": {"ts": 4.0}},
                "target": {"goal": {"x": 7.0, "y": 8.0, "yaw": 0.0}},
            },
        )
        self.robot.navigation_status = Mock(side_effect=[other, active, completed])

        result = self.robot.wait_until_arrived(
            timeout=1.0,
            poll_interval=0.0,
            request_id="goal-1",
            expected_goal=(7.0, 8.0, 0.0),
            baseline=baseline,
        )

        self.assertIs(result, completed)
        self.assertEqual(self.robot.navigation_status.call_count, 3)

    def test_wait_until_arrived_reports_failure_that_precedes_first_active_poll(self) -> None:
        baseline = NavigationStatus(
            state="IDLE",
            raw={"mission": {"raw": {"ts": 1.0}}},
        )
        self.robot.navigation_status = Mock(
            return_value=NavigationStatus(
                state="FAILED",
                goal=Position(7.0, 8.0),
                raw={
                    "failure_reason": "goal_rejected",
                    "mission": {"raw": {"ts": 2.0}},
                    "target": {"goal": {"x": 7.0, "y": 8.0}},
                },
            )
        )

        with self.assertRaisesRegex(RuntimeError, "FAILED.*goal_rejected"):
            self.robot.wait_until_arrived(
                timeout=1.0,
                poll_interval=0.0,
                expected_goal=(7.0, 8.0, 0.0),
                baseline=baseline,
            )

    def test_batch_go_waits_for_every_waypoint_including_final(self) -> None:
        first_baseline = NavigationStatus(state="IDLE", raw={"mission": {"raw": {"ts": 1.0}}})
        second_baseline = NavigationStatus(state="SUCCESS", raw={"mission": {"raw": {"ts": 2.0}}})
        self.robot.navigation_status = Mock(side_effect=[first_baseline, second_baseline])
        self.robot.go = Mock(
            side_effect=[
                CommandResult(ok=True, accepted=True, request_id="goal-1"),
                CommandResult(ok=True, accepted=True, request_id="goal-2"),
            ]
        )
        self.robot.wait_until_arrived = Mock(
            side_effect=[
                NavigationStatus(state="SUCCESS"),
                NavigationStatus(state="SUCCESS"),
            ]
        )

        results = self.robot.batch_go([(1.0, 2.0, 0.1), (3.0, 4.0, 0.2)])

        self.assertEqual(len(results), 2)
        self.assertEqual(
            self.robot.wait_until_arrived.call_args_list,
            [
                call(
                    request_id="goal-1",
                    expected_goal=(1.0, 2.0, 0.1),
                    baseline=first_baseline,
                ),
                call(
                    request_id="goal-2",
                    expected_goal=(3.0, 4.0, 0.2),
                    baseline=second_baseline,
                ),
            ],
        )

    @patch("urllib.request.urlopen")
    def test_command_without_explicit_success_signal_is_not_accepted(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"status": "queued"})

        result = self.robot.go(1.0, 2.0, request_id="goal-no-ack")

        self.assertFalse(result.ok)
        self.assertFalse(result.accepted)

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
        request = mock_urlopen.call_args.args[0]
        self.assertEqual(request.full_url, "http://127.0.0.1:5050/api/v1/slam/maps")

    @patch("urllib.request.urlopen")
    def test_maps_prefers_current_active_contract(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {
            "active": "warehouse",
            "active_map": "legacy_map",
            "maps": [
                {"name": "warehouse", "has_pcd": True},
                {"name": "legacy_map", "has_pcd": True},
            ],
        })

        maps = self.robot.maps()

        self.assertEqual(maps.active_map, "warehouse")
        self.assertTrue(maps.maps[0].is_active)
        self.assertFalse(maps.maps[1].is_active)


    # ------------------------------------------------------------------
    # Save / use / rename map
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_save_map(self, mock_urlopen) -> None:
        self._mock_http(
            mock_urlopen,
            {
                "ok": True,
                "accepted": True,
                "request_id": "save-test-001",
                "message": "map save queued",
            },
        )

        r = self.robot.save_map(
            "test_map",
            request_id="save-test-001",
        )

        self.assertTrue(r.ok)
        self.assertEqual(r.request_id, "save-test-001")
        request = mock_urlopen.call_args.args[0]
        self.assertEqual(
            json.loads(request.data),
            {
                "name": "test_map",
                "request_id": "save-test-001",
            },
        )

    @patch("urllib.request.urlopen")
    def test_save_map_generates_request_id_when_omitted(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True, "accepted": True})

        result = self.robot.save_map("test_map")

        request = mock_urlopen.call_args.args[0]
        request_id = json.loads(request.data)["request_id"]
        self.assertRegex(request_id, r"^[0-7][0-9A-HJKMNP-TV-Z]{25}$")
        self.assertEqual(result.request_id, request_id)

    @patch("urllib.request.urlopen")
    def test_save_map_does_not_expose_internal_job_id(
        self,
        mock_urlopen,
    ) -> None:
        self._mock_http(
            mock_urlopen,
            {
                "ok": True,
                "accepted": True,
                "request_id": "request_1",
                "job_id": "internal_job_1",
            },
        )

        result = self.robot.save_map("test_map", request_id="request_1")

        self.assertEqual(result.request_id, "request_1")
        self.assertIsNone(result.operation_id)

    def test_save_map_and_wait_returns_immediate_success(self) -> None:
        receipt = CommandResult(
            ok=True,
            accepted=True,
            request_id="request_1",
            operation_id="operation_1",
            raw={"request_id": "request_1", "operation_id": "operation_1"},
        )
        completed = {
            "ok": True,
            "request_id": "request_1",
            "operation_id": "operation_1",
            "operation": {
                "operation_id": "operation_1",
                "state": "SUCCEEDED",
                "map_id": "test_map",
            },
        }
        self.robot.save_map = Mock(return_value=receipt)
        self.robot.get_map_operation = Mock(return_value=completed)

        result = self.robot.save_map_and_wait(
            "test_map",
            request_id="request_1",
            poll_interval=0.0,
        )

        self.assertIs(result, completed)
        self.robot.save_map.assert_called_once_with(
            "test_map",
            request_id="request_1",
        )
        self.robot.get_map_operation.assert_called_once_with("operation_1")

    def test_save_map_and_wait_uses_server_operation_id(self) -> None:
        receipt = CommandResult(
            ok=True,
            accepted=True,
            request_id="request_1",
            operation_id="operation_9",
            raw={
                "request_id": "request_1",
                "operation_id": "operation_9",
            },
        )
        completed = {
            "ok": True,
            "operation_id": "operation_9",
            "operation": {
                "operation_id": "operation_9",
                "state": "SUCCEEDED",
                "map_id": "test_map",
            },
        }
        self.robot.save_map = Mock(return_value=receipt)
        self.robot.get_map_operation = Mock(return_value=completed)

        result = self.robot.save_map_and_wait(
            "test_map",
            request_id="request_1",
            poll_interval=0.0,
        )

        self.assertIs(result, completed)
        self.robot.get_map_operation.assert_called_once_with("operation_9")

    def test_legacy_save_job_methods_are_not_public(self) -> None:
        self.assertFalse(hasattr(self.robot, "get_save_map_job"))
        self.assertFalse(hasattr(self.robot, "cancel_save_map_job"))
        self.assertFalse(hasattr(self.robot, "retry_save_map_job"))

    def test_save_map_and_wait_polls_operation_with_generated_request_id(self) -> None:
        completed = {
            "ok": True,
            "operation_id": "operation_generated",
            "operation": {
                "operation_id": "operation_generated",
                "state": "SUCCEEDED",
            },
        }

        def admit_save(_name, *, request_id):
            return CommandResult(
                ok=True,
                accepted=True,
                request_id=request_id,
                operation_id="operation_generated",
                raw={
                    "request_id": request_id,
                    "operation_id": "operation_generated",
                },
            )

        self.robot.save_map = Mock(side_effect=admit_save)
        self.robot.get_map_operation = Mock(
            side_effect=[
                {
                    "ok": True,
                    "operation_id": "operation_generated",
                    "operation": {
                        "operation_id": "operation_generated",
                        "state": "RUNNING",
                    },
                },
                completed,
            ]
        )

        result = self.robot.save_map_and_wait("test_map", poll_interval=0.0)

        resolved_id = self.robot.save_map.call_args.kwargs["request_id"]
        self.assertRegex(resolved_id, r"^[0-7][0-9A-HJKMNP-TV-Z]{25}$")
        self.assertIs(result, completed)
        self.assertEqual(
            self.robot.get_map_operation.call_args_list,
            [call("operation_generated"), call("operation_generated")],
        )

    def test_save_map_and_wait_fails_fast_when_submission_is_rejected(self) -> None:
        self.robot.save_map = Mock(
            return_value=CommandResult(
                ok=False,
                accepted=False,
                message="invalid map name",
                request_id="save-test-001",
                raw={
                    "ok": False,
                    "reason_code": "invalid_map_name",
                    "message": "invalid map name",
                },
            )
        )
        self.robot.get_map_operation = Mock()

        with self.assertRaisesRegex(
            RuntimeError,
            "request rejected.*invalid_map_name",
        ):
            self.robot.save_map_and_wait(
                "bad/name",
                request_id="save-test-001",
                timeout=0.01,
                poll_interval=0.0,
            )

        self.robot.get_map_operation.assert_not_called()

    def test_save_map_and_wait_fails_fast_on_explicit_status_query_error(self) -> None:
        self.robot.save_map = Mock(
            return_value=CommandResult(
                ok=True,
                accepted=True,
                request_id="request_1",
                operation_id="operation_1",
                raw={"request_id": "request_1", "operation_id": "operation_1"},
            )
        )
        self.robot.get_map_operation = Mock(
            return_value={
                "ok": False,
                "success": False,
                "operation_id": "operation_1",
                "operation": {
                    "operation_id": "operation_1",
                    "reason_code": "forbidden",
                    "message": "invalid API key",
                },
            }
        )

        with self.assertRaisesRegex(
            RuntimeError,
            "status query failed.*forbidden",
        ):
            self.robot.save_map_and_wait(
                "test_map",
                request_id="request_1",
                poll_interval=0.0,
            )

        self.robot.get_map_operation.assert_called_once_with("operation_1")

    def test_save_map_and_wait_rejects_mismatched_request_identity(self) -> None:
        self.robot.save_map = Mock(
            return_value=CommandResult(
                ok=True,
                accepted=True,
                request_id="request_1",
                operation_id="operation_1",
                raw={"request_id": "different-request", "operation_id": "operation_1"},
            )
        )
        self.robot.get_map_operation = Mock()

        with self.assertRaisesRegex(RuntimeError, "request identity mismatch"):
            self.robot.save_map_and_wait(
                "test_map",
                request_id="request_1",
                poll_interval=0.0,
            )

        self.robot.get_map_operation.assert_not_called()

    def test_save_map_and_wait_rejects_mismatched_operation_identity(self) -> None:
        self.robot.save_map = Mock(
            return_value=CommandResult(
                ok=True,
                accepted=True,
                request_id="request_1",
                operation_id="operation_1",
                raw={"request_id": "request_1", "operation_id": "operation_1"},
            )
        )
        self.robot.get_map_operation = Mock(
            return_value={
                "operation_id": "different-operation",
                "operation": {
                    "operation_id": "different-operation",
                    "state": "SUCCEEDED",
                },
            }
        )

        with self.assertRaisesRegex(RuntimeError, "identity mismatch"):
            self.robot.save_map_and_wait(
                "test_map",
                request_id="request_1",
                poll_interval=0.0,
            )

    def test_save_map_and_wait_raises_with_failed_operation_reason(self) -> None:
        self.robot.save_map = Mock(
            return_value=CommandResult(
                ok=True,
                accepted=True,
                request_id="request_1",
                operation_id="operation_1",
                raw={"request_id": "request_1", "operation_id": "operation_1"},
            )
        )
        self.robot.get_map_operation = Mock(
            return_value={
                "operation_id": "operation_1",
                "operation": {
                    "operation_id": "operation_1",
                    "state": "FAILED",
                    "reason_code": "snapshot_failed",
                },
            }
        )

        with self.assertRaisesRegex(RuntimeError, "FAILED.*snapshot_failed"):
            self.robot.save_map_and_wait(
                "test_map",
                request_id="request_1",
                timeout=0.01,
                poll_interval=0.0,
            )

    def test_save_map_and_wait_raises_for_cancelled_operation_spellings(self) -> None:
        for state in ("CANCELLED", "CANCELED"):
            with self.subTest(state=state):
                self.robot.save_map = Mock(
                    return_value=CommandResult(
                        ok=True,
                        accepted=True,
                        request_id="request_1",
                        operation_id="operation_1",
                        raw={
                            "request_id": "request_1",
                            "operation_id": "operation_1",
                        },
                    )
                )
                self.robot.get_map_operation = Mock(
                    return_value={
                        "operation_id": "operation_1",
                        "operation": {
                            "operation_id": "operation_1",
                            "state": state,
                            "message": "operator cancelled",
                        },
                    }
                )

                with self.assertRaisesRegex(
                    RuntimeError,
                    f"{state}.*operator cancelled",
                ):
                    self.robot.save_map_and_wait(
                        "test_map",
                        request_id="request_1",
                        timeout=0.01,
                        poll_interval=0.0,
                    )

    def test_save_map_and_wait_recovers_from_post_timeout_and_delayed_admission(self) -> None:
        completed = {
            "ok": True,
            "operation_id": "operation_recovered",
            "operation": {
                "operation_id": "operation_recovered",
                "state": "SUCCEEDED",
                "map_id": "test_map",
            },
        }
        self.robot.save_map = Mock(
            side_effect=[
                CommandResult(
                    ok=False,
                    accepted=False,
                    message="robot not reachable",
                    raw={"ok": False, "error": "robot not reachable"},
                ),
                CommandResult(
                    ok=True,
                    accepted=True,
                    operation_id="operation_recovered",
                    raw={"operation_id": "operation_recovered"},
                ),
            ]
        )
        self.robot.get_map_operation = Mock(
            side_effect=[
                {
                    "ok": False,
                    "operation_id": "operation_recovered",
                    "operation": {
                        "operation_id": "operation_recovered",
                        "state": "NOT_FOUND",
                        "reason_code": "operation_not_found",
                    },
                },
                {
                    "ok": False,
                    "success": False,
                    "operation_id": "operation_recovered",
                    "error": "robot not reachable",
                },
                completed,
            ]
        )

        result = self.robot.save_map_and_wait("test_map", poll_interval=0.0)

        first_call, second_call = self.robot.save_map.call_args_list
        resolved_id = first_call.kwargs["request_id"]
        self.assertRegex(resolved_id, r"^[0-7][0-9A-HJKMNP-TV-Z]{25}$")
        self.assertEqual(second_call.kwargs["request_id"], resolved_id)
        self.assertIs(result, completed)
        self.assertEqual(
            self.robot.get_map_operation.call_args_list,
            [
                call("operation_recovered"),
                call("operation_recovered"),
                call("operation_recovered"),
            ],
        )

    @patch("lingtu.sdk.client.time.monotonic", side_effect=[10.0, 10.1, 11.1])
    def test_save_map_and_wait_times_out_with_last_state(self, _monotonic) -> None:
        self.robot.save_map = Mock(
            return_value=CommandResult(
                ok=True,
                accepted=True,
                request_id="request_1",
                operation_id="operation_1",
                raw={"request_id": "request_1", "operation_id": "operation_1"},
            )
        )
        self.robot.get_map_operation = Mock(
            return_value={
                "operation_id": "operation_1",
                "operation": {
                    "operation_id": "operation_1",
                    "state": "RUNNING",
                },
            }
        )

        with self.assertRaisesRegex(TimeoutError, "within 1.0s.*RUNNING"):
            self.robot.save_map_and_wait(
                "test_map",
                request_id="request_1",
                timeout=1.0,
                poll_interval=0.0,
            )

        self.robot.get_map_operation.assert_called_once_with("operation_1")

    @patch("urllib.request.urlopen")
    def test_get_map_operation(self, mock_urlopen) -> None:
        self._mock_http(
            mock_urlopen,
            {
                "ok": True,
                "operation_id": "operation-1",
                "operation": {"state": "RUNNING"},
            },
        )

        operation = self.robot.get_map_operation("operation-1")

        self.assertEqual(operation["operation"]["state"], "RUNNING")
        request = mock_urlopen.call_args.args[0]
        self.assertEqual(
            request.full_url,
            "http://127.0.0.1:5050/api/v1/maps/operations/operation-1",
        )

    @patch("urllib.request.urlopen")
    def test_cancel_map_operation(self, mock_urlopen) -> None:
        self._mock_http(
            mock_urlopen,
            {"ok": True, "success": True, "accepted": True, "message": "cancelled"},
        )

        result = self.robot.cancel_map_operation("operation-1")

        self.assertTrue(result.ok)
        request = mock_urlopen.call_args.args[0]
        self.assertEqual(
            request.full_url,
            "http://127.0.0.1:5050/api/v1/maps/operations/operation-1/cancel",
        )
        self.assertEqual(request.method, "POST")

    @patch("urllib.request.urlopen")
    def test_retry_map_operation(self, mock_urlopen) -> None:
        self._mock_http(
            mock_urlopen,
            {"ok": True, "success": True, "accepted": True, "message": "queued"},
        )

        result = self.robot.retry_map_operation("operation-1")

        self.assertTrue(result.ok)
        request = mock_urlopen.call_args.args[0]
        self.assertEqual(
            request.full_url,
            "http://127.0.0.1:5050/api/v1/maps/operations/operation-1/retry",
        )
        self.assertEqual(request.method, "POST")

    @patch("urllib.request.urlopen")
    def test_download_map_pcd_streams_with_authentication(self, mock_urlopen) -> None:
        response = mock_urlopen.return_value.__enter__.return_value
        response.read.side_effect = [b"pcd-", b"bytes", b""]
        robot = LingTuClient(api_key="secret")

        target = Path.cwd() / ".sdk-test-sync-download-success.pcd"
        try:
            result = robot.download_map_pcd("factory_01", target)
            self.assertEqual(result, target)
            self.assertEqual(target.read_bytes(), b"pcd-bytes")
        finally:
            target.unlink(missing_ok=True)

        request = mock_urlopen.call_args.args[0]
        self.assertEqual(
            request.full_url,
            "http://127.0.0.1:5050/api/v1/maps/factory_01/pcd",
        )
        self.assertEqual(dict(request.header_items())["X-api-key"], "secret")
        self.assertGreaterEqual(response.read.call_count, 3)

    @patch("urllib.request.urlopen")
    def test_download_map_pcd_preserves_target_on_interrupted_stream(
        self,
        mock_urlopen,
    ) -> None:
        response = mock_urlopen.return_value.__enter__.return_value
        response.read.side_effect = [b"partial", OSError("connection lost")]
        target = Path.cwd() / ".sdk-test-sync-download.pcd"
        target.write_bytes(b"previous-map")

        try:
            with self.assertRaisesRegex(OSError, "connection lost"):
                self.robot.download_map_pcd("factory_01", target)

            self.assertEqual(target.read_bytes(), b"previous-map")
            self.assertEqual(
                list(target.parent.glob(f".{target.name}.*.part")),
                [],
            )
        finally:
            target.unlink(missing_ok=True)
            for partial in target.parent.glob(f".{target.name}.*.part"):
                partial.unlink(missing_ok=True)

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_tag_location(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.tag_location("test", use_current_pose=True)
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

    # ------------------------------------------------------------------
    # Exploration
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Native recording
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_recording(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})
        r = self.robot.recording_start(
            "test",
            duration=60,
            capture_profile="evidence",
            task_id="inspection-task-1",
            camera=True,
            minimum_free_gib=12,
        )
        self.assertIsInstance(r, CommandResult)
        request = mock_urlopen.call_args.args[0]
        self.assertTrue(request.full_url.endswith("/api/v1/recordings/start"))
        self.assertEqual(
            json.loads(request.data),
            {
                "duration": 60,
                "prefix": "test",
                "capture_profile": "evidence",
                "task_id": "inspection-task-1",
                "camera": True,
                "minimum_free_gib": 12,
            },
        )
        r = self.robot.recording_stop()
        self.assertIsInstance(r, CommandResult)

    @patch("urllib.request.urlopen")
    def test_recording_sensors_omits_task_id(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True})

        self.robot.recording_start(capture_profile="sensors")

        request = mock_urlopen.call_args.args[0]
        self.assertEqual(
            json.loads(request.data),
            {"duration": 600, "capture_profile": "sensors"},
        )

    # ------------------------------------------------------------------
    # Memory
    # ------------------------------------------------------------------

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
    # App endpoints
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Auth
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_auth_login(self, mock_urlopen) -> None:
        self._mock_http(mock_urlopen, {"ok": True, "token": "abc"})
        r = self.robot.auth_login("my-key")
        self.assertEqual(r.get("token"), "abc")
        request = mock_urlopen.call_args.args[0]
        self.assertEqual(json.loads(request.data), {"key": "my-key"})

    # ------------------------------------------------------------------
    # Map operations
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Navigation status / path / localization
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_command_unreachable_returns_failed_result(self, mock_urlopen) -> None:
        mock_urlopen.side_effect = urllib.error.URLError("no route to host")

        r = self.robot.go(1.0, 2.0)

        self.assertIsInstance(r, CommandResult)
        self.assertFalse(r.ok)
        self.assertEqual(r.message, "robot not reachable")
        self.assertEqual(r.raw["path"], "/api/v1/goal")

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    @patch("urllib.request.urlopen")
    def test_context_manager(self, mock_urlopen) -> None:
        with LingTuClient() as r:
            self.assertIsNotNone(r)


if __name__ == "__main__":
    unittest.main()
