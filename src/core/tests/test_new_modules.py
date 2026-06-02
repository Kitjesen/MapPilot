"""Tests for 7 new unified modules — NavigationModule, SafetyRingModule,
AutonomyModule, GatewayModule, MCPServerModule, RerunModule, SemanticPlannerModule.

All external deps mocked. No GPU/FastAPI/Rerun/C++ required.
"""

import json
import math
import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from core.msgs.nav import Odometry

# ============================================================================
# NavigationModule
# ============================================================================

class TestNavigationModule(unittest.TestCase):

    def _make(self, **kw):
        from nav.navigation_module import NavigationModule
        return NavigationModule(planner="astar", **kw)

    def test_ports_in(self):
        m = self._make()
        self.assertIn("goal_pose", m.ports_in)
        self.assertIn("odometry", m.ports_in)
        self.assertIn("instruction", m.ports_in)
        self.assertIn("stop_signal", m.ports_in)
        self.assertIn("cancel", m.ports_in)

    def test_ports_out(self):
        m = self._make()
        self.assertIn("waypoint", m.ports_out)
        self.assertIn("global_path", m.ports_out)
        self.assertIn("clear_path", m.ports_out)
        self.assertIn("planner_status", m.ports_out)
        self.assertIn("mission_status", m.ports_out)

    def test_default_waypoint_z_threshold_matches_step_limit(self):
        m = self._make()

        self.assertAlmostEqual(m._tracker._z_threshold, 0.25)

    def test_set_state_publishes(self):
        m = self._make()
        statuses = []
        m.planner_status._add_callback(statuses.append)
        m._set_state("PLANNING")
        self.assertEqual(statuses[-1], "PLANNING")

    def test_on_stop_clears(self):
        m = self._make()
        clears = []
        recovery = []
        m.clear_path._add_callback(clears.append)
        m.recovery_cmd_vel._add_callback(recovery.append)
        # Path now lives in the WaypointTracker service
        m._tracker._path = [np.array([1, 2, 0])]
        m._on_stop(2)
        self.assertEqual(m._tracker.path_length, 0)
        self.assertEqual(m._state, "IDLE")
        self.assertEqual(clears, [True])
        self.assertTrue(recovery[-1].is_zero())

    def test_on_cancel_clears_active_mission(self):
        m = self._make()
        statuses = []
        clears = []
        recovery = []
        m.mission_status._add_callback(statuses.append)
        m.clear_path._add_callback(clears.append)
        m.recovery_cmd_vel._add_callback(recovery.append)
        m._state = "EXECUTING"
        m._goal = np.array([5.0, 6.0, 0.0])
        m._tracker.reset(
            [np.array([2.0, 3.0, 0.0]), np.array([5.0, 6.0, 0.0])],
            np.array([0.0, 0.0, 0.0]),
        )

        m._on_cancel("operator_cancel")

        self.assertEqual(m._state, "CANCELLED")
        self.assertEqual(m._tracker.path_length, 0)
        self.assertEqual(m._failure_reason, "cancelled: operator_cancel")
        self.assertEqual(statuses[-1]["state"], "CANCELLED")
        self.assertEqual(statuses[-1]["remaining_waypoints"], 0)
        self.assertIsNone(statuses[-1]["goal"])
        self.assertEqual(clears, [True])
        self.assertTrue(recovery[-1].is_zero())

    def test_cancel_clears_stale_teleop_resume_state(self):
        m = self._make()
        m._paused_for_teleop = True
        m._pre_teleop_goal = np.array([5.0, 6.0, 0.0])
        m._pre_teleop_state = "EXECUTING"
        m._state = "EXECUTING"
        m._plan = MagicMock()

        m._on_cancel("operator_cancel")
        m._on_teleop_active(False)

        self.assertIsNone(m._pre_teleop_goal)
        self.assertIsNone(m._pre_teleop_state)
        m._plan.assert_not_called()

    def test_stop_clears_stale_teleop_resume_state(self):
        m = self._make()
        m._paused_for_teleop = True
        m._pre_teleop_goal = np.array([5.0, 6.0, 0.0])
        m._pre_teleop_state = "EXECUTING"
        m._state = "EXECUTING"
        m._plan = MagicMock()

        m._on_stop(2)
        m._on_teleop_active(False)

        self.assertIsNone(m._pre_teleop_goal)
        self.assertIsNone(m._pre_teleop_state)
        m._plan.assert_not_called()

    def test_recovery_motion_returns_without_sleeping_in_caller(self):
        m = self._make()
        m._state = "EXECUTING"

        with patch("nav.navigation_module.time.sleep", side_effect=AssertionError("blocking sleep")):
            m._execute_recovery_motion(post_action="none")
            m._request_recovery_stop()

    def test_downsample(self):
        m = self._make(downsample_dist=2.0)
        path = [(0, 0, 0), (0.5, 0, 0), (1, 0, 0), (3, 0, 0), (5, 0, 0)]
        goal = np.array(path[-1])
        # Downsample now lives in GlobalPlannerService
        result = m._planner_svc._downsample(path, goal)
        self.assertLess(len(result), len(path))

    def test_on_odom_updates_pos(self):
        m = self._make()
        odom = Odometry(
            pose=Pose(position=Vector3(1, 2, 0)),
            frame_id="map",
            ts=time.time(),
        )
        m._on_odom(odom)
        self.assertAlmostEqual(m._robot_pos[0], 1.0)
        self.assertAlmostEqual(m._robot_pos[1], 2.0)

    def test_on_odom_rejects_non_planning_frame_without_transform(self):
        m = self._make()
        events = []
        m.adapter_status._add_callback(events.append)

        m._on_odom(Odometry(
            pose=Pose(position=Vector3(1, 2, 0)),
            frame_id="odom",
            ts=time.time(),
        ))

        self.assertEqual(m._odom_frame_id, "odom")
        np.testing.assert_array_equal(m._robot_pos, np.zeros(3))
        self.assertTrue(
            any(event.get("event") == "frame_mismatch" for event in events)
        )

    def test_on_odom_rejects_non_contract_frame(self):
        m = self._make()
        events = []
        m.adapter_status._add_callback(events.append)
        m._state = "EXECUTING"

        m._on_odom(Odometry(
            pose=Pose(position=Vector3(1, 2, 0)),
            frame_id="camera_link",
            ts=time.time(),
        ))

        self.assertEqual(m._state, "FAILED")
        self.assertEqual(events[-1]["event"], "navigation_blocked")
        self.assertEqual(events[-1]["source"], "odometry")
        self.assertEqual(events[-1]["expected_frame"], "map")
        self.assertEqual(events[-1]["received_frame"], "camera_link")

    def test_on_odom_updates_yaw_and_tracker(self):
        from nav.waypoint_tracker import TrackerStatus

        m = self._make()
        m._state = "EXECUTING"
        m._tracker.update = MagicMock(return_value=TrackerStatus(0, 1))

        yaw = math.pi / 2.0
        odom = Odometry(
            pose=Pose(
                position=Vector3(1.0, 2.0, 0.0),
                orientation=Quaternion.from_yaw(yaw),
            ),
            frame_id="map",
            ts=time.time(),
        )
        m._on_odom(odom)

        self.assertAlmostEqual(m._robot_yaw, yaw)
        m._tracker.update.assert_called_once()
        self.assertAlmostEqual(m._tracker.update.call_args.args[1], yaw)
        status = json.loads(m.get_navigation_status())
        self.assertAlmostEqual(status["position"]["yaw"], round(yaw, 3))

    def test_repeated_patrol_goals_do_not_reset_active_exploration_path(self):
        m = self._make()
        m._plan = MagicMock()
        goals = [{"x": 1.0, "y": 2.0, "z": 0.0}, {"x": 3.0, "y": 4.0, "z": 0.0}]

        m._on_patrol_goals(goals)
        self.assertEqual(m._state, "PATROLLING")
        self.assertEqual(m._plan.call_count, 1)

        m._state = "EXECUTING"
        m._plan.reset_mock()
        m._on_patrol_goals(goals)

        self.assertEqual(m._state, "EXECUTING")
        m._plan.assert_not_called()

    def test_repeated_patrol_goals_restart_after_terminal_state(self):
        m = self._make()
        m._plan = MagicMock()
        goals = [{"x": 1.0, "y": 2.0, "z": 0.0}, {"x": 3.0, "y": 4.0, "z": 0.0}]

        m._on_patrol_goals(goals)
        self.assertEqual(m._plan.call_count, 1)

        m._state = "SUCCESS"
        m._plan.reset_mock()
        m._on_patrol_goals(goals)

        self.assertEqual(m._state, "PATROLLING")
        self.assertEqual(m._plan.call_count, 1)

    def test_external_strategy_path_control_executes_path_without_global_replan(self):
        m = self._make(
            external_strategy_path_control=True,
            waypoint_threshold=0.2,
            final_waypoint_threshold=0.2,
        )
        m._plan = MagicMock()
        global_paths = []
        waypoints = []
        events = []
        m.global_path._add_callback(global_paths.append)
        m.waypoint._add_callback(waypoints.append)
        m.adapter_status._add_callback(events.append)

        goals = [
            {"x": 1.0, "y": 0.0, "z": 0.0},
            {"x": 2.0, "y": 0.0, "z": 0.0},
            {"x": 3.0, "y": 0.0, "z": 0.0},
        ]
        m._on_patrol_goals(goals)

        self.assertEqual(m._state, "EXECUTING")
        self.assertTrue(m._using_external_strategy_path)
        m._plan.assert_not_called()
        self.assertEqual(len(global_paths[-1]), 4)
        self.assertAlmostEqual(global_paths[-1][0][0], 0.0)
        self.assertAlmostEqual(waypoints[-1].pose.position.x, 1.0)
        self.assertTrue(
            any(e.get("event") == "external_strategy_path_control" for e in events)
        )

    def test_external_strategy_path_control_does_not_replan_on_costmap(self):
        m = self._make(
            external_strategy_path_control=True,
            waypoint_threshold=0.2,
            final_waypoint_threshold=0.2,
        )
        m._on_patrol_goals([
            {"x": 1.0, "y": 0.0, "z": 0.0},
            {"x": 2.0, "y": 0.0, "z": 0.0},
        ])
        m._plan = MagicMock()
        m._last_costmap_replan_time = 0.0

        m._on_costmap({
            "grid": np.zeros((5, 5), dtype=np.float32),
            "resolution": 0.2,
            "origin": [0.0, 0.0],
        })

        m._plan.assert_not_called()

    def test_external_strategy_path_control_does_not_replan_on_stuck(self):
        from nav.waypoint_tracker import EV_STUCK, TrackerStatus

        m = self._make(
            external_strategy_path_control=True,
            waypoint_threshold=0.2,
            final_waypoint_threshold=0.2,
            max_replan_count=3,
        )
        events = []
        global_paths = []
        waypoints = []
        m.adapter_status._add_callback(events.append)
        m.global_path._add_callback(global_paths.append)
        m.waypoint._add_callback(waypoints.append)
        m._on_patrol_goals([
            {"x": 1.0, "y": 0.0, "z": 0.0},
            {"x": 2.0, "y": 0.0, "z": 0.0},
            {"x": 3.0, "y": 0.0, "z": 0.0},
        ])
        m._plan = MagicMock()
        # Mock _execute_recovery_motion to call _finish_recovery_motion directly
        # (which publishes the event), instead of starting a long-running thread.
        m._execute_recovery_motion = MagicMock(
            side_effect=lambda post_action: m._finish_recovery_motion(post_action, "stuck")
        )
        m._tracker.update = MagicMock(
            return_value=TrackerStatus(0, 3, event=EV_STUCK)
        )

        m._on_odom(Odometry(
            pose=Pose(position=Vector3(0.0, 0.0, 0.0)),
            frame_id="map",
            ts=time.time(),
        ))

        m._execute_recovery_motion.assert_called_once()
        m._plan.assert_not_called()
        self.assertEqual(m._state, "EXECUTING")
        self.assertTrue(m._using_external_strategy_path)
        self.assertEqual(len(global_paths[-1]), 4)
        self.assertAlmostEqual(waypoints[-1].pose.position.x, 1.0)
        self.assertTrue(
            any(
                e.get("event") == "external_strategy_path_stuck_recovery"
                for e in events
            )
        )

    def test_external_strategy_path_reports_strategy_not_stale_pct(self):
        m = self._make(
            external_strategy_path_control=True,
            waypoint_threshold=0.2,
            final_waypoint_threshold=0.2,
        )
        m._planner_svc._last_plan_report = {
            "primary_planner": "pct",
            "selected_planner": "pct",
            "fallback_reason": "old failure",
        }
        statuses = []
        m.mission_status._add_callback(statuses.append)

        m._on_patrol_goals([
            {"x": 1.0, "y": 0.0, "z": 0.0},
            {"x": 2.0, "y": 0.0, "z": 0.0},
        ])

        report = statuses[-1]["last_plan_report"]
        self.assertEqual(report["primary_planner"], "tare_external")
        self.assertEqual(report["selected_planner"], "external_strategy_path")
        self.assertIs(report["fallback_used"], False)

    def test_layer(self):
        m = self._make()
        self.assertEqual(m.layer, 5)

    def test_health(self):
        m = self._make()
        h = m.health()
        self.assertIn("navigation", h)
        self.assertEqual(h["navigation"]["planner"], "astar")

    def test_no_map_uses_direct_goal_fallback(self):
        # Direct-goal fallback is off by default since Wave 1 — explicitly opt
        # in for this legacy-behaviour test.
        m = self._make(enable_ros2_bridge=False, allow_direct_goal_fallback=True)

        class _NoMapBackend:
            _grid = None

            def plan(self, start, goal, **kwargs):
                return []

        m._planner_svc._backend = _NoMapBackend()
        m._robot_pos = np.array([0.0, 0.0, 0.0])
        waypoints = []
        statuses = []
        m.waypoint._add_callback(waypoints.append)
        m.adapter_status._add_callback(statuses.append)

        m._on_goal(PoseStamped(
            pose=Pose(position=Vector3(4.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="map", ts=time.time(),
        ))

        self.assertEqual(m._state, "EXECUTING")
        self.assertEqual(len(waypoints), 1)
        self.assertAlmostEqual(waypoints[0].pose.position.x, 4.0)
        self.assertAlmostEqual(waypoints[0].pose.position.y, 2.0)
        self.assertTrue(any(s.get("event") == "direct_goal_fallback" for s in statuses))

    def test_empty_path_with_map_still_fails(self):
        m = self._make(enable_ros2_bridge=False)

        class _MappedBackend:
            _grid = np.zeros((10, 10), dtype=np.float32)

            def plan(self, start, goal, **kwargs):
                return []

        m._planner_svc._backend = _MappedBackend()
        m._robot_pos = np.array([0.0, 0.0, 0.0])

        m._on_goal(PoseStamped(
            pose=Pose(position=Vector3(4.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="map", ts=time.time(),
        ))

        self.assertEqual(m._state, "FAILED")
        self.assertIn("empty path", m._failure_reason)

    def test_empty_path_with_map_can_use_explicit_direct_goal_fallback(self):
        m = self._make(
            enable_ros2_bridge=False,
            allow_direct_goal_fallback=True,
            direct_goal_fallback_on_planner_failure=True,
        )

        class _MappedBackend:
            _grid = np.zeros((10, 10), dtype=np.float32)

            def plan(self, start, goal, **kwargs):
                return []

        m._planner_svc._backend = _MappedBackend()
        m._robot_pos = np.array([0.0, 0.0, 0.0])
        waypoints = []
        statuses = []
        missions = []
        m.waypoint._add_callback(waypoints.append)
        m.adapter_status._add_callback(statuses.append)
        m.mission_status._add_callback(missions.append)

        m._on_goal(PoseStamped(
            pose=Pose(position=Vector3(4.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="map", ts=time.time(),
        ))

        self.assertEqual(m._state, "EXECUTING")
        self.assertEqual(len(waypoints), 1)
        self.assertAlmostEqual(waypoints[0].pose.position.x, 4.0)
        self.assertAlmostEqual(waypoints[0].pose.position.y, 2.0)
        self.assertTrue(any(s.get("event") == "direct_goal_fallback" for s in statuses))
        self.assertTrue(missions[-1]["direct_goal_fallback"]["used"])
        self.assertIn("empty path", missions[-1]["direct_goal_fallback"]["reason"])

    def test_duplicate_goal_update_is_ignored_while_executing(self):
        # Requires direct-goal fallback to reach EXECUTING without a real map.
        m = self._make(enable_ros2_bridge=False, allow_direct_goal_fallback=True)

        class _NoMapBackend:
            _grid = None

            def plan(self, start, goal, **kwargs):
                return []

        m._planner_svc._backend = _NoMapBackend()
        m._robot_pos = np.array([0.0, 0.0, 0.0])
        events = []
        m.adapter_status._add_callback(events.append)

        pose = PoseStamped(
            pose=Pose(position=Vector3(4.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="map", ts=time.time(),
        )
        m._on_goal(pose)
        first_path_len = m._tracker.path_length
        first_state = m._state
        m._on_goal(pose)

        self.assertEqual(first_state, "EXECUTING")
        self.assertEqual(m._tracker.path_length, first_path_len)
        self.assertTrue(any(e.get("event") == "goal_update_ignored" for e in events))

    def test_same_xy_different_z_goal_update_is_not_ignored(self):
        m = self._make(enable_ros2_bridge=False, allow_direct_goal_fallback=True)

        class _NoMapBackend:
            _grid = None

            def plan(self, start, goal, **kwargs):
                return []

        m._planner_svc._backend = _NoMapBackend()
        m._robot_pos = np.array([0.0, 0.0, 0.0])
        events = []
        m.adapter_status._add_callback(events.append)

        m._on_goal(PoseStamped(
            pose=Pose(position=Vector3(4.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="map", ts=time.time(),
        ))
        events.clear()

        m._on_goal(PoseStamped(
            pose=Pose(position=Vector3(4.0, 2.0, 3.0), orientation=Quaternion()),
            frame_id="map", ts=time.time(),
        ))

        self.assertAlmostEqual(m._goal[2], 3.0)
        self.assertFalse(any(e.get("event") == "goal_update_ignored" for e in events))

    def test_navigate_to_skill_accepts_explicit_z(self):
        m = self._make(enable_ros2_bridge=False, allow_direct_goal_fallback=True)
        m._robot_pos = np.array([1.0, 2.0, 1.25])

        result = json.loads(m.navigate_to(4.0, 5.0, yaw=0.25, z=2.5))

        self.assertEqual(result["goal"], [4.0, 5.0, 2.5])
        self.assertAlmostEqual(m._goal[2], 2.5)
        self.assertAlmostEqual(m._active_path_terminal_goal[2], 2.5)

    def test_navigate_to_skill_defaults_z_to_current_floor(self):
        m = self._make(enable_ros2_bridge=False, allow_direct_goal_fallback=True)
        m._on_odom(Odometry(
            pose=Pose(position=Vector3(1.0, 2.0, 1.75)),
            frame_id="map",
            ts=time.time(),
        ))

        result = json.loads(m.navigate_to(4.0, 5.0))

        self.assertEqual(result["goal"], [4.0, 5.0, 1.75])
        self.assertAlmostEqual(m._goal[2], 1.75)

    def test_rejects_non_map_goal_frame(self):
        m = self._make(enable_ros2_bridge=False, allow_direct_goal_fallback=True)
        events = []
        statuses = []
        waypoints = []
        m.adapter_status._add_callback(events.append)
        m.mission_status._add_callback(statuses.append)
        m.waypoint._add_callback(waypoints.append)

        m._on_goal(PoseStamped(
            pose=Pose(position=Vector3(4.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="odom", ts=time.time(),
        ))

        self.assertIsNone(m._goal)
        self.assertEqual(m._state, "IDLE")
        self.assertEqual(waypoints, [])
        self.assertEqual(events[-1]["event"], "goal_rejected")
        self.assertEqual(events[-1]["expected_frame"], "map")
        self.assertEqual(events[-1]["received_frame"], "odom")
        self.assertEqual(statuses[-1]["planning_frame_id"], "map")
        self.assertEqual(
            statuses[-1]["failure_reason"],
            "unsupported goal_pose frame 'odom'; expected 'map'",
        )

    def test_rejects_non_finite_goal_coordinates(self):
        m = self._make(enable_ros2_bridge=False, allow_direct_goal_fallback=True)
        events = []
        statuses = []
        waypoints = []
        m.adapter_status._add_callback(events.append)
        m.mission_status._add_callback(statuses.append)
        m.waypoint._add_callback(waypoints.append)

        m._on_goal(PoseStamped(
            pose=Pose(position=Vector3(float("nan"), 2.0, 0.0), orientation=Quaternion()),
            frame_id="map", ts=time.time(),
        ))

        self.assertIsNone(m._goal)
        self.assertEqual(m._state, "IDLE")
        self.assertEqual(waypoints, [])
        self.assertEqual(events[-1]["event"], "goal_rejected")
        self.assertEqual(events[-1]["reason"], "invalid_coordinates")
        self.assertEqual(events[-1]["source"], "goal_pose")
        self.assertEqual(statuses[-1]["failure_reason"], "invalid goal_pose coordinates")

    def test_mission_status_exposes_coordinate_frames(self):
        m = self._make()
        statuses = []
        m.mission_status._add_callback(statuses.append)

        m._on_odom(Odometry(
            pose=Pose(position=Vector3(1, 2, 0)),
            frame_id="map",
            ts=time.time(),
        ))
        m._set_state("IDLE")

        self.assertEqual(statuses[-1]["frame_id"], "map")
        self.assertEqual(statuses[-1]["planning_frame_id"], "map")
        self.assertEqual(statuses[-1]["odom_frame_id"], "map")


# ============================================================================
# SafetyRingModule
# ============================================================================

class TestSafetyRingModule(unittest.TestCase):

    def _make(self):
        from nav.safety_ring_module import SafetyRingModule
        return SafetyRingModule()

    def test_ports_in(self):
        m = self._make()
        self.assertIn("odometry", m.ports_in)
        self.assertIn("path", m.ports_in)
        self.assertIn("cmd_vel", m.ports_in)

    def test_ports_out(self):
        m = self._make()
        self.assertIn("stop_cmd", m.ports_out)
        self.assertIn("safety_state", m.ports_out)
        self.assertIn("execution_eval", m.ports_out)
        self.assertIn("dialogue_state", m.ports_out)

    def test_cross_track_no_path(self):
        m = self._make()
        self.assertAlmostEqual(m._cross_track_error(), 0.0)

    def test_distance_to_goal_no_goal(self):
        m = self._make()
        self.assertEqual(m._distance_to_goal(), float("inf"))

    def test_on_odom_updates_xy(self):
        m = self._make()
        m.setup()
        odom = Odometry(pose=Pose(position=Vector3(3, 4, 0)), ts=time.time())
        m._on_odom(odom)
        self.assertAlmostEqual(m._robot_xy[0], 3.0)

    def test_layer_is_0(self):
        m = self._make()
        self.assertEqual(m.layer, 0)

    def test_health(self):
        m = self._make()
        h = m.health()
        self.assertIn("safety_ring", h)


# ============================================================================
# TerrainModule / LocalPlannerModule / PathFollowerModule
# ============================================================================

class TestTerrainModule(unittest.TestCase):

    def test_ports(self):
        from base_autonomy.modules.terrain_module import TerrainModule
        m = TerrainModule(backend="simple")
        self.assertIn("odometry", m.ports_in)
        self.assertIn("map_cloud", m.ports_in)
        self.assertIn("terrain_map", m.ports_out)
        self.assertIn("traversability", m.ports_out)
        self.assertIn("alive", m.ports_out)

    def test_layer(self):
        from base_autonomy.modules.terrain_module import TerrainModule
        m = TerrainModule(backend="simple")
        self.assertEqual(m.layer, 2)

    def test_health(self):
        from base_autonomy.modules.terrain_module import TerrainModule
        m = TerrainModule(backend="simple")
        h = m.health()
        self.assertIn("terrain", h)
        self.assertEqual(h["terrain"]["backend"], "simple")


class TestLocalPlannerModule(unittest.TestCase):

    def test_ports(self):
        from base_autonomy.modules.local_planner_module import LocalPlannerModule
        m = LocalPlannerModule(backend="simple")
        self.assertIn("odometry", m.ports_in)
        self.assertIn("terrain_map", m.ports_in)
        self.assertIn("waypoint", m.ports_in)
        self.assertIn("global_path", m.ports_in)
        self.assertIn("clear_path", m.ports_in)
        self.assertIn("map_frame_jump_event", m.ports_in)
        self.assertIn("local_path", m.ports_out)

    def test_layer(self):
        from base_autonomy.modules.local_planner_module import LocalPlannerModule
        m = LocalPlannerModule(backend="simple")
        self.assertEqual(m.layer, 2)

    def test_storage_inputs_keep_latest_only(self):
        from base_autonomy.modules.local_planner_module import LocalPlannerModule
        m = LocalPlannerModule(backend="simple")
        m.setup()

        self.assertEqual(m.terrain_map._policy, "latest")
        self.assertEqual(m.boundary._policy, "latest")
        self.assertEqual(m.added_obstacles._policy, "latest")
        self.assertEqual(m.esdf._policy, "latest")

    def test_clear_path_drops_waypoint_and_publishes_empty_path(self):
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        m = LocalPlannerModule(backend="simple")
        paths = []
        m.local_path._add_callback(paths.append)
        m._latest_waypoint = PoseStamped(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
            frame_id="map",
        )

        m._on_clear_path(True)

        self.assertIsNone(m._latest_waypoint)
        self.assertEqual(len(paths), 1)
        self.assertEqual(len(paths[-1].poses), 0)
        self.assertEqual(paths[-1].frame_id, "map")

    def test_global_path_biases_simple_backend_toward_corridor(self):
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        m = LocalPlannerModule(backend="simple")
        paths = []
        m.local_path._add_callback(paths.append)
        m._on_odom(Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0))))
        m._on_global_path([
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 3.0, 0.0]),
            np.array([3.0, 3.0, 0.0]),
        ])

        m._on_waypoint(PoseStamped(
            pose=Pose(position=Vector3(3.0, 3.0, 0.0)),
            frame_id="odom",
        ))

        self.assertEqual(paths[-1].frame_id, "odom")
        last = paths[-1].poses[-1]
        self.assertAlmostEqual(last.x, 0.0)
        self.assertAlmostEqual(last.y, 3.0)

    def test_map_frame_jump_drops_local_planner_state(self):
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        m = LocalPlannerModule(backend="simple")
        paths = []
        m.local_path._add_callback(paths.append)
        m._latest_waypoint = PoseStamped(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
            frame_id="map",
        )
        m._global_path_points = np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 0.0]])

        m._on_map_frame_jump({"dt_m": 1.2})

        self.assertIsNone(m._latest_waypoint)
        self.assertIsNone(m._global_path_points)
        self.assertEqual(len(paths[-1].poses), 0)
        self.assertEqual(paths[-1].frame_id, "map")

    def test_nanobind_empty_path_triggers_safety_stop(self):
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        class EmptyResult:
            path = []
            slow_down = 0
            near_field_stop = False
            path_found = False
            recovery_state = 0

        class FakeCore:
            def set_vehicle(self, *args):
                pass

            def set_goal(self, *args):
                pass

            def plan(self, *args):
                return EmptyResult()

        m = LocalPlannerModule(backend="nanobind")
        m._core = FakeCore()
        m._robot_pos = np.array([0.0, 0.0, 0.0], dtype=float)
        m._global_path_points = np.array([[5.0, 0.0, 0.0]], dtype=float)
        m._latest_waypoint = PoseStamped(
            pose=Pose(position=Vector3(5.0, 0.0, 0.0)),
            frame_id="map",
        )
        paths = []
        hints = []
        m.local_path._add_callback(paths.append)
        m.control_hint._add_callback(hints.append)

        m._run_nanobind(1.0)

        self.assertEqual(len(paths), 1)
        self.assertEqual(paths[-1].poses, [])
        self.assertTrue(hints[-1]["safety_stop"])
        self.assertEqual(hints[-1]["reason"], "no_local_path")

    def test_nanobind_direct_track_fallback_keeps_trackable_path(self):
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        class EmptyResult:
            path = []
            slow_down = 0
            near_field_stop = False
            path_found = False
            recovery_state = 0

        class FakeCore:
            def set_vehicle(self, *args):
                pass

            def set_goal(self, *args):
                pass

            def plan(self, *args):
                return EmptyResult()

        m = LocalPlannerModule(
            backend="nanobind",
            allow_direct_track_fallback=True,
            direct_track_fallback_min_distance_m=0.3,
        )
        m._core = FakeCore()
        m._robot_pos = np.array([0.0, 0.0, 0.0], dtype=float)
        m._latest_waypoint = PoseStamped(
            pose=Pose(position=Vector3(1.0, 0.0, 0.0)),
            frame_id="map",
        )
        paths = []
        hints = []
        m.local_path._add_callback(paths.append)
        m.control_hint._add_callback(hints.append)

        m._run_nanobind(1.0)

        self.assertGreaterEqual(len(paths[-1].poses), 2)
        self.assertFalse(hints[-1]["safety_stop"])
        self.assertEqual(hints[-1]["reason"], "direct_track_fallback:no_local_path")
        self.assertEqual(m.health()["local_planner"]["last_local_path_points"], 2)

    def test_nanobind_direct_track_fallback_respects_near_field_stop(self):
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        class EmptyResult:
            path = []
            slow_down = 0
            near_field_stop = True
            path_found = False
            recovery_state = 0

        class FakeCore:
            def set_vehicle(self, *args):
                pass

            def set_goal(self, *args):
                pass

            def plan(self, *args):
                return EmptyResult()

        m = LocalPlannerModule(backend="nanobind", allow_direct_track_fallback=True)
        m._core = FakeCore()
        m._robot_pos = np.array([0.0, 0.0, 0.0], dtype=float)
        m._latest_waypoint = PoseStamped(
            pose=Pose(position=Vector3(1.0, 0.0, 0.0)),
            frame_id="map",
        )
        paths = []
        hints = []
        m.local_path._add_callback(paths.append)
        m.control_hint._add_callback(hints.append)

        m._run_nanobind(1.0)

        self.assertEqual(paths[-1].poses, [])
        self.assertTrue(hints[-1]["safety_stop"])
        self.assertEqual(hints[-1]["reason"], "no_local_path")

    def test_nanobind_ignore_near_field_stop_keeps_trackable_path_active(self):
        from base_autonomy.modules.local_planner_module import LocalPlannerModule

        class Pt:
            def __init__(self, x, y, z=0.0):
                self.x = x
                self.y = y
                self.z = z

        class Result:
            path = [Pt(0.0, 0.0), Pt(1.0, 0.0)]
            slow_down = 0
            near_field_stop = True
            path_found = True
            recovery_state = 0

        class FakeCore:
            def set_vehicle(self, *args):
                pass

            def set_goal(self, *args):
                pass

            def plan(self, *args):
                return Result()

        m = LocalPlannerModule(
            backend="nanobind",
            ignore_near_field_stop=True,
        )
        m._core = FakeCore()
        m._robot_pos = np.array([0.0, 0.0, 0.0], dtype=float)
        m._latest_waypoint = PoseStamped(
            pose=Pose(position=Vector3(1.0, 0.0, 0.0)),
            frame_id="map",
        )
        hints = []
        m.control_hint._add_callback(hints.append)

        m._run_nanobind(1.0)

        self.assertTrue(hints[-1]["near_field_stop"])
        self.assertFalse(hints[-1]["safety_stop"])
        self.assertEqual(hints[-1]["reason"], "nanobind")


class TestPathFollowerModule(unittest.TestCase):

    def test_ports(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule
        m = PathFollowerModule(backend="pid")
        self.assertIn("odometry", m.ports_in)
        self.assertIn("local_path", m.ports_in)
        self.assertIn("map_frame_jump_event", m.ports_in)
        self.assertIn("cmd_vel", m.ports_out)

    def test_layer(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule
        m = PathFollowerModule(backend="pid")
        self.assertEqual(m.layer, 2)


# ============================================================================
# GatewayModule
# ============================================================================

@unittest.skipUnless(
    __import__("importlib").util.find_spec("pydantic"),
    "pydantic not installed",
)
class TestGatewayModule(unittest.TestCase):

    def _make(self):
        from gateway.gateway_module import GatewayModule
        return GatewayModule(port=5050)

    def test_ports_in(self):
        m = self._make()
        # Assert named ports exist rather than count (count drifts as
        # modules grow — just check the contract we actually rely on).
        self.assertGreaterEqual(len(m.ports_in), 10)
        self.assertIn("odometry", m.ports_in)
        self.assertIn("map_cloud", m.ports_in)
        self.assertIn("scene_graph", m.ports_in)
        self.assertIn("safety_state", m.ports_in)
        self.assertIn("global_path", m.ports_in)
        self.assertIn("costmap", m.ports_in)
        self.assertIn("slope_grid", m.ports_in)

    def test_ports_out(self):
        m = self._make()
        self.assertGreaterEqual(len(m.ports_out), 6)
        self.assertIn("instruction", m.ports_out)
        self.assertIn("mode_cmd", m.ports_out)
        self.assertIn("goal_pose", m.ports_out)
        self.assertIn("cmd_vel", m.ports_out)
        self.assertIn("stop_cmd", m.ports_out)
        self.assertIn("cancel", m.ports_out)

    def test_on_odometry_caches(self):
        m = self._make()
        odom = Odometry(pose=Pose(position=Vector3(1, 2, 0)), ts=time.time())
        m._on_odometry(odom)
        self.assertIsNotNone(m._odom)
        self.assertAlmostEqual(m._odom["x"], 1.0)

    def test_check_lease_open(self):
        m = self._make()
        self.assertTrue(m._lease.check("anyone"))

    def test_check_lease_held(self):
        m = self._make()
        m._lease.acquire("client_a", ttl=30.0)
        self.assertTrue(m._lease.check("client_a"))
        self.assertFalse(m._lease.check("client_b"))

    def test_layer(self):
        m = self._make()
        self.assertEqual(m.layer, 6)

    def test_health(self):
        m = self._make()
        h = m.health()
        self.assertIn("gateway", h)


# ============================================================================
# MCPServerModule
# ============================================================================

class TestMCPServerModule(unittest.TestCase):

    def _make(self):
        from gateway.mcp_server import MCPServerModule
        return MCPServerModule(port=8090)

    def test_ports_in(self):
        m = self._make()
        self.assertIn("odometry", m.ports_in)
        self.assertIn("scene_graph", m.ports_in)
        self.assertIn("safety_state", m.ports_in)
        self.assertIn("mission_status", m.ports_in)

    def test_ports_out(self):
        m = self._make()
        self.assertIn("goal_pose", m.ports_out)
        self.assertIn("cmd_vel", m.ports_out)
        self.assertIn("stop_cmd", m.ports_out)
        self.assertIn("instruction", m.ports_out)
        self.assertIn("mode_cmd", m.ports_out)

    def test_tools_discovered_via_skill(self):
        """@skill methods on MCPServerModule itself appear in _tool_registry after on_system_modules."""
        import json

        from core.module import Module, skill

        class FakeNav(Module, layer=5):
            @skill
            def navigate_to(self, x: float, y: float) -> str:
                """Go to map coordinates (x, y)."""
                return json.dumps({"status": "navigating", "goal": [x, y]})

        m = self._make()
        m.on_system_modules({"MCPServerModule": m, "NavigationModule": FakeNav()})
        # Built-in MCP tools + navigation skills when NavigationModule is present
        self.assertGreaterEqual(len(m._tool_list), 12)
        names = [t["name"] for t in m._tool_list]
        self.assertIn("emergency_stop", names)
        self.assertIn("navigate_to", names)
        self.assertIn("get_health", names)
        self.assertIn("get_config", names)

    def test_stop_via_skill(self):
        import json
        m = self._make()
        stops = []
        m.stop_cmd._add_callback(stops.append)
        result = json.loads(m.emergency_stop())
        self.assertEqual(result["status"], "emergency_stopped")
        self.assertEqual(stops, [2])

    def test_get_position_no_odom(self):
        import json
        m = self._make()
        result = json.loads(m.get_robot_position())
        self.assertIn("error", result)

    def test_get_position_with_odom(self):
        import json
        m = self._make()
        m._odom = {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}
        result = json.loads(m.get_robot_position())
        self.assertAlmostEqual(result["x"], 1.0)

    def test_tag_location_via_skill(self):
        import json

        from memory.modules.tagged_locations_module import TaggedLocationsModule
        m = self._make()
        m._odom = {"x": 5.0, "y": 3.0, "z": 0.0}
        tl = TaggedLocationsModule()
        m._tagged_locations_mod = tl
        result = json.loads(m.tag_location("office"))
        self.assertEqual(result["tagged"], "office")
        self.assertIsNotNone(tl.store.query("office"))

    def test_navigate_to_resolves_to_navigation_named_module(self):
        import json

        from core.module import Module, skill

        class FakeNav(Module, layer=5):
            def __init__(self):
                super().__init__()
                self.calls = []

            @skill
            def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> str:
                """Navigate to (x, y) in map frame."""
                self.calls.append((x, y, yaw))
                return json.dumps({"status": "navigating", "goal": [x, y], "yaw": yaw})

        m = self._make()
        nav = FakeNav()
        m.on_system_modules({"MCPServerModule": m, "NavigationModule": nav})
        fn = m._tool_registry.get("navigate_to")
        self.assertIsNotNone(fn)
        self.assertEqual(getattr(fn, "__self__", None), nav)
        result = json.loads(fn(5.0, 3.0))
        self.assertEqual(result["status"], "navigating")
        self.assertEqual(nav.calls, [(5.0, 3.0, 0.0)])

    def test_set_mode_via_skill(self):
        import json
        m = self._make()
        result = json.loads(m.set_mode("autonomous"))
        self.assertEqual(result["mode"], "autonomous")

    def test_set_mode_estop_publishes_stop_and_zero_twist(self):
        import json
        m = self._make()
        result = json.loads(m.set_mode("estop"))
        self.assertEqual(result["mode"], "estop")
        self.assertEqual(m.mode_cmd.msg_count, 1)
        self.assertEqual(m.stop_cmd.msg_count, 1)
        self.assertEqual(m.cmd_vel.msg_count, 1)

    def test_set_mode_invalid(self):
        import json
        m = self._make()
        result = json.loads(m.set_mode("unknown"))
        self.assertIn("error", result)

    def test_query_memory_degraded_vector_hit_is_query_only(self):
        import json

        class _VectorMemory:
            def query_location(self, text):
                return {
                    "found": True,
                    "best": {
                        "x": 1.0,
                        "y": 2.0,
                        "labels": "backpack",
                        "score": 0.95,
                    },
                    "results": [{
                        "x": 1.0,
                        "y": 2.0,
                        "labels": "backpack",
                        "score": 0.95,
                        "navigable": False,
                    }],
                    "encoder_type": "lexical_hash",
                    "semantic_encoder_ready": False,
                    "degraded": True,
                    "navigable": False,
                }

        m = self._make()
        m._vector_memory_mod = _VectorMemory()

        result = json.loads(m.query_memory("find backpack"))

        self.assertEqual(result["count"], 1)
        self.assertTrue(result["results"][0]["query_only"])
        self.assertNotIn("position", result["results"][0])

    def test_get_health_no_handle(self):
        import json
        m = self._make()
        result = json.loads(m.get_health())
        self.assertIn("error", result)

    def test_health(self):
        m = self._make()
        m.on_system_modules({"MCPServerModule": m})
        h = m.health()
        self.assertIn("mcp", h)
        self.assertEqual(h["mcp"]["tools"], len(m._tool_list))
        self.assertGreaterEqual(h["mcp"]["tools"], 13)


# ============================================================================
# RerunModule
# ============================================================================

class TestRerunModule(unittest.TestCase):

    def _make(self):
        from core.rerun_module import RerunModule
        return RerunModule(web_port=9090)

    def test_ports_in(self):
        m = self._make()
        self.assertIn("odometry", m.ports_in)
        self.assertIn("global_path", m.ports_in)
        self.assertIn("scene_graph", m.ports_in)
        self.assertIn("safety_state", m.ports_in)
        self.assertIn("waypoint", m.ports_in)
        self.assertIn("cmd_vel", m.ports_in)

    def test_trajectory_empty_initially(self):
        m = self._make()
        self.assertEqual(len(m._trajectory), 0)

    def test_layer(self):
        m = self._make()
        self.assertEqual(m.layer, 6)

    def test_health(self):
        m = self._make()
        h = m.health()
        self.assertIn("rerun", h)
        self.assertEqual(h["rerun"]["trajectory_len"], 0)


# ============================================================================
# SLAMModule
# ============================================================================

class TestSLAMModule(unittest.TestCase):

    def test_ports(self):
        from slam.slam_module import SLAMModule
        m = SLAMModule(backend="fastlio2")
        self.assertIn("alive", m.ports_out)
        self.assertNotIn("odometry", m.ports_out)
        self.assertNotIn("map_cloud", m.ports_out)

    def test_layer(self):
        from slam.slam_module import SLAMModule
        m = SLAMModule(backend="fastlio2")
        self.assertEqual(m.layer, 1)

    def test_backends_registered(self):
        # Import SLAMModule to trigger @register decorators
        from slam.slam_module import SLAMModule  # noqa: F401
        from core.registry import list_plugins
        backends = list_plugins("slam")
        self.assertIn("fastlio2", backends)
        self.assertIn("pointlio", backends)

    def test_health(self):
        from slam.slam_module import SLAMModule
        m = SLAMModule(backend="pointlio")
        h = m.health()
        self.assertIn("slam", h)
        self.assertEqual(h["slam"]["backend"], "pointlio")


# ============================================================================
# SemanticPlannerModule
# ============================================================================

class TestSemanticPlannerModule(unittest.TestCase):

    def _make(self):
        import os
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "semantic", "planner"))
        from semantic.planner.semantic_planner_module import SemanticPlannerModule
        return SemanticPlannerModule()

    def test_ports_in(self):
        m = self._make()
        self.assertIn("instruction", m.ports_in)
        self.assertIn("scene_graph", m.ports_in)
        self.assertIn("odometry", m.ports_in)
        self.assertIn("detections", m.ports_in)
        self.assertIn("mission_status", m.ports_in)

    def test_ports_out(self):
        m = self._make()
        self.assertIn("goal_pose", m.ports_out)
        self.assertIn("planner_status", m.ports_out)
        self.assertIn("task_plan", m.ports_out)
        self.assertIn("cancel", m.ports_out)

    def test_decompose_fallback(self):
        m = self._make()
        result = m._decompose("go to the kitchen")
        self.assertIn("subtasks", result)

    def test_layer(self):
        m = self._make()
        self.assertEqual(m.layer, 4)

    def test_health(self):
        m = self._make()
        h = m.health()
        self.assertIn("semantic_planner", h)
        self.assertIn("lera_triggers", h["semantic_planner"])

    # -- LERa integration tests -----------------------------------------------

    def test_lera_ignored_when_executing(self):
        """Non-terminal nav states must not trigger LERa."""
        m = self._make()
        statuses = []
        m.planner_status._add_callback(statuses.append)
        m._on_mission_status({"state": "EXECUTING"})
        self.assertNotIn("RECOVERING", statuses)

    def test_lera_triggers_on_stuck(self):
        """STUCK state must publish RECOVERING and increment lera_count."""
        import time as _time
        m = self._make()
        statuses = []
        m.planner_status._add_callback(statuses.append)
        m._lera_cooldown = 0  # disable cooldown for test
        m._on_mission_status({"state": "STUCK"})
        # RECOVERING is published synchronously before thread launch.
        self.assertIn("RECOVERING", statuses)
        self.assertEqual(m._lera_count, 1)
        # Wait briefly for background thread to finish and clear _lera_running.
        for _ in range(20):
            with m._lera_lock:
                if not m._lera_running:
                    break
            _time.sleep(0.05)

    def test_lera_cooldown_prevents_storm(self):
        """Second STUCK within cooldown window must be silently dropped."""
        m = self._make()
        counts = []
        m.planner_status._add_callback(lambda s: counts.append(s) if s == "RECOVERING" else None)
        m._lera_cooldown = 999  # very long cooldown
        m._last_nav_state = ""  # first trigger allowed
        m._on_mission_status({"state": "STUCK"})
        m._last_nav_state = ""  # reset so state-change check passes
        m._on_mission_status({"state": "STUCK"})  # must be dropped by cooldown
        self.assertEqual(len(counts), 1)

    def test_lera_abort_publishes_cancel(self):
        """abort strategy must publish 'lera_abort' on cancel port."""
        m = self._make()
        cancels = []
        m.cancel._add_callback(cancels.append)
        m._dispatch_recovery("abort")
        self.assertEqual(cancels, ["lera_abort"])

    def test_lera_retry_republishes_goal(self):
        """retry_different_path must republish the cached goal_pose."""
        m = self._make()
        goals = []
        m.goal_pose._add_callback(goals.append)
        pose = PoseStamped(
            pose=Pose(position=Vector3(3, 4, 0), orientation=Quaternion(0, 0, 0, 1)),
            frame_id="map", ts=0.0,
        )
        m._current_goal_pose = pose
        m._dispatch_recovery("retry_different_path")
        self.assertEqual(len(goals), 1)
        self.assertAlmostEqual(goals[0].pose.position.x, 3.0)

    def test_lera_abort_clears_state(self):
        """abort must reset failure_count and current_instruction."""
        m = self._make()
        m._failure_count = 3
        m._current_instruction = "go to kitchen"
        m._dispatch_recovery("abort")
        self.assertEqual(m._failure_count, 0)
        self.assertEqual(m._current_instruction, "")

    def test_lera_no_duplicate_trigger_same_state(self):
        """Consecutive publishes of the same state must only trigger LERa once."""
        m = self._make()
        m._lera_cooldown = 0
        m._on_mission_status({"state": "STUCK"})
        count_after_first = m._lera_count
        m._on_mission_status({"state": "STUCK"})  # same state, must be dropped
        self.assertEqual(m._lera_count, count_after_first)

    def test_fast_resolve_uses_target_xyz_when_position_missing(self):
        m = self._make()

        class _Result:
            confidence = 0.9
            target_x = 16.25
            target_y = 2.9
            target_z = 0.937
            frame_id = 'map'

        class _Resolver:
            def maybe_reload_kg(self):
                return None

            def fast_resolve(self, instruction, sg_json):
                return _Result()

        m._goal_resolver = _Resolver()
        goals = []
        m.goal_pose._add_callback(goals.append)

        m._try_resolve('find the stairs', '{"objects": []}')

        self.assertEqual(len(goals), 1)
        self.assertAlmostEqual(goals[0].pose.position.x, 16.25)
        self.assertAlmostEqual(goals[0].pose.position.y, 2.9)
        self.assertAlmostEqual(goals[0].pose.position.z, 0.937)

    def test_degraded_vector_memory_cannot_publish_goal_pose(self):
        class _VectorMemory:
            def query_location(self, text):
                return {
                    "found": True,
                    "best": {
                        "x": 1.0,
                        "y": 2.0,
                        "labels": "backpack",
                        "score": 0.95,
                    },
                    "encoder_type": "lexical_hash",
                    "semantic_encoder_ready": False,
                    "degraded": True,
                    "navigable": False,
                }

            def get_memory_stats(self):
                return {
                    "encoder_type": "lexical_hash",
                    "semantic_encoder_ready": False,
                    "degraded": True,
                }

        m = self._make()
        m._vector_memory = _VectorMemory()
        goals = []
        statuses = []
        m.goal_pose._add_callback(goals.append)
        m.planner_status._add_callback(statuses.append)

        self.assertFalse(m._try_vector_memory("find backpack"))
        self.assertEqual(goals, [])
        self.assertIn("VECTOR_MEMORY_QUERY_ONLY", statuses)

    def test_degraded_vector_memory_blocks_frontier_fallback(self):
        class _VectorMemory:
            def query_location(self, text):
                return {
                    "found": True,
                    "best": {
                        "x": 1.0,
                        "y": 2.0,
                        "labels": "backpack",
                        "score": 0.95,
                    },
                    "encoder_type": "lexical_hash",
                    "semantic_encoder_ready": False,
                    "degraded": True,
                    "navigable": False,
                }

            def get_memory_stats(self):
                return {
                    "encoder_type": "lexical_hash",
                    "semantic_encoder_ready": False,
                    "degraded": True,
                }

        class _Frontier:
            position = [9.0, 9.0, 0.0]

        class _FrontierScorer:
            def get_best_frontier(self):
                return _Frontier()

        m = self._make()
        m._goal_resolver = None
        m._frontier_scorer = _FrontierScorer()
        m._vector_memory = _VectorMemory()
        goals = []
        statuses = []
        m.goal_pose._add_callback(goals.append)
        m.planner_status._add_callback(statuses.append)

        m._try_resolve("find backpack", '{"objects": []}')

        self.assertEqual(goals, [])
        self.assertIn("VECTOR_MEMORY_QUERY_ONLY", statuses)
        self.assertNotIn("EXPLORING", statuses)

    def test_vector_memory_missing_contract_fields_fails_closed(self):
        class _VectorMemory:
            def query_location(self, text):
                return {
                    "found": True,
                    "best": {
                        "x": 1.0,
                        "y": 2.0,
                        "labels": "legacy",
                        "score": 0.99,
                    },
                }

            def get_memory_stats(self):
                return {
                    "encoder_type": "mobileclip",
                    "semantic_encoder_ready": True,
                    "degraded": False,
                }

        m = self._make()
        m._vector_memory = _VectorMemory()
        goals = []
        m.goal_pose._add_callback(goals.append)

        self.assertFalse(m._try_vector_memory("find legacy target"))
        self.assertEqual(goals, [])

    def test_vector_memory_stats_failure_fails_closed(self):
        class _VectorMemory:
            def query_location(self, text):
                return {
                    "found": True,
                    "best": {
                        "x": 1.0,
                        "y": 2.0,
                        "labels": "charger",
                        "score": 0.99,
                    },
                    "encoder_type": "mobileclip",
                    "semantic_encoder_ready": True,
                    "degraded": False,
                    "navigable": True,
                }

            def get_memory_stats(self):
                raise RuntimeError("stats unavailable")

        m = self._make()
        m._vector_memory = _VectorMemory()
        goals = []
        m.goal_pose._add_callback(goals.append)

        self.assertFalse(m._try_vector_memory("find charger"))
        self.assertEqual(goals, [])

    def test_degraded_agent_memory_tool_withholds_coordinates(self):
        class _VectorMemory:
            def query_location(self, text):
                return {
                    "found": True,
                    "best": {
                        "x": 1.0,
                        "y": 2.0,
                        "labels": "backpack",
                        "score": 0.95,
                    },
                    "encoder_type": "lexical_hash",
                    "semantic_encoder_ready": False,
                    "degraded": True,
                    "navigable": False,
                }

            def get_memory_stats(self):
                return {
                    "encoder_type": "lexical_hash",
                    "semantic_encoder_ready": False,
                    "degraded": True,
                }

        m = self._make()
        m._vector_memory = _VectorMemory()

        response = m._tool_query_memory("find backpack")

        self.assertIn("coordinates are withheld", response)
        self.assertNotIn("(1.0, 2.0)", response)

    def test_semantic_vector_memory_can_publish_goal_pose(self):
        class _VectorMemory:
            def query_location(self, text):
                return {
                    "found": True,
                    "best": {
                        "x": 6.0,
                        "y": 7.0,
                        "z": 1.25,
                        "labels": "charger",
                        "score": 0.95,
                    },
                    "encoder_type": "mobileclip",
                    "semantic_encoder_ready": True,
                    "degraded": False,
                    "navigable": True,
                }

            def get_memory_stats(self):
                return {
                    "encoder_type": "mobileclip",
                    "semantic_encoder_ready": True,
                    "degraded": False,
                }

        m = self._make()
        m._vector_memory = _VectorMemory()
        goals = []
        statuses = []
        m.goal_pose._add_callback(goals.append)
        m.planner_status._add_callback(statuses.append)

        self.assertTrue(m._try_vector_memory("find charger"))
        self.assertEqual(len(goals), 1)
        self.assertAlmostEqual(goals[0].pose.position.x, 6.0)
        self.assertAlmostEqual(goals[0].pose.position.y, 7.0)
        self.assertAlmostEqual(goals[0].pose.position.z, 1.25)
        self.assertIn("VECTOR_MEMORY", statuses)

    def test_scene_graph_stored_as_object(self):
        """_on_scene_graph must persist the SceneGraph object, not just JSON."""
        from core.msgs.semantic import Detection3D, SceneGraph
        m = self._make()
        sg = SceneGraph(objects=[Detection3D(label="chair", confidence=0.9)])
        m._on_scene_graph(sg)
        self.assertIsNotNone(m._current_scene_graph)
        self.assertEqual(m._current_scene_graph.objects[0].label, "chair")


# ============================================================================
# MissionLoggerModule
# ============================================================================

class TestMissionLoggerModule(unittest.TestCase):

    def _make(self, tmp_path=None):
        import tempfile

        from memory.modules.mission_logger_module import MissionLoggerModule
        log_dir = tmp_path or tempfile.mkdtemp(prefix="lingtu_test_missions_")
        return MissionLoggerModule(log_dir=log_dir)

    def test_ports(self):
        m = self._make()
        self.assertIn("mission_status", m.ports_in)
        self.assertIn("odometry", m.ports_in)
        self.assertEqual(len(m.ports_out), 0)

    def test_empty_list(self):
        import json
        m = self._make()
        # @skill list_missions returns JSON str
        result = json.loads(m.list_missions())
        self.assertEqual(result["missions"], [])

    def test_empty_list_raw(self):
        m = self._make()
        self.assertEqual(m._list_missions_raw(), [])

    def test_empty_stats(self):
        m = self._make()
        s = m._stats_raw()
        self.assertEqual(s["total"], 0)
        self.assertEqual(s["success"], 0)
        self.assertEqual(s["failed"], 0)

    def test_empty_stats_get_stats_compat(self):
        m = self._make()
        s = m.get_stats()
        self.assertEqual(s["total"], 0)

    def test_mission_lifecycle_saved(self):
        import tempfile
        tmp = tempfile.mkdtemp(prefix="lingtu_test_missions_")
        m = self._make(tmp_path=tmp)

        m._on_mission_status({"state": "PLANNING", "goal": "kitchen"})
        self.assertIsNotNone(m._current)
        self.assertEqual(m._current["goal"], "kitchen")

        from core.msgs.geometry import Pose, Vector3
        def _odom(x, y):
            return Odometry(pose=Pose(position=Vector3(x=x, y=y, z=0.0)))
        m._on_odom(_odom(1.0, 2.0))
        odom2 = _odom(4.0, 6.0)
        m._on_odom(odom2)

        m._on_mission_status({"state": "SUCCESS"})
        self.assertIsNone(m._current)

        from pathlib import Path as PPath
        files = list(PPath(tmp).glob("*.json"))
        self.assertEqual(len(files), 1)
        import json
        data = json.loads(files[0].read_text())
        self.assertEqual(data["result"], "SUCCESS")
        self.assertEqual(data["goal"], "kitchen")
        self.assertGreater(data["distance_m"], 0)

    def test_list_and_stats_after_missions(self):
        import tempfile
        tmp = tempfile.mkdtemp(prefix="lingtu_test_missions_")
        m = self._make(tmp_path=tmp)

        for result in ("SUCCESS", "FAILED", "SUCCESS"):
            m._on_mission_status({"state": "PLANNING", "goal": "test"})
            m._on_mission_status({"state": result})

        import json
        missions_raw = m._list_missions_raw()
        self.assertEqual(len(missions_raw), 3)
        # @skill JSON path
        missions_json = json.loads(m.list_missions())
        self.assertEqual(len(missions_json["missions"]), 3)
        s = m._stats_raw()
        self.assertEqual(s["total"], 3)
        self.assertEqual(s["success"], 2)
        self.assertEqual(s["failed"], 1)

    def test_replan_count_tracked(self):
        import tempfile
        tmp = tempfile.mkdtemp(prefix="lingtu_test_missions_")
        m = self._make(tmp_path=tmp)

        m._on_mission_status({"state": "PLANNING", "goal": "go far"})
        m._on_mission_status({"state": "REPLANNING"})
        m._on_mission_status({"state": "REPLANNING"})
        m._on_mission_status({"state": "SUCCESS"})

        import json
        missions = m._list_missions_raw()
        self.assertEqual(missions[0]["replan_count"], 2)
        # also verify via @skill JSON path
        missions_json = json.loads(m.list_missions())
        self.assertEqual(missions_json["missions"][0]["replan_count"], 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
