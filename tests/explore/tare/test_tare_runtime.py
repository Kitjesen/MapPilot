"""Tests for the TARE exploration bridge and the exploration stack factory.

Runs without any C++ binary or DDS transport; relies on the in-process policy
path, and exercises the
contracts (ports, skills, waypoint to PoseStamped conversion).
"""

from __future__ import annotations

import ast
import os
import unittest
from pathlib import Path

from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import Odometry
from runtime.runtime_interface import TOPICS

ROOT = Path(__file__).resolve().parents[3]
TARE_ENDPOINT_BOUNDARY_FILES = (
    ROOT / "src/lingtu/assembly/stacks/exploration.py",
    ROOT / "src/explore/tare/module.py",
    ROOT / "src/explore/tare/supervisor.py",
)


def _absolute_imports(tree: ast.AST) -> list[tuple[int, str]]:
    imports: list[tuple[int, str]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.extend((node.lineno, alias.name) for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.append((node.lineno, node.module))
    return imports


def _call_name(node: ast.Call) -> str:
    parts: list[str] = []
    current: ast.AST = node.func
    while isinstance(current, ast.Attribute):
        parts.append(current.attr)
        current = current.value
    if isinstance(current, ast.Name):
        parts.append(current.id)
    else:
        return ""
    return ".".join(reversed(parts))


def test_tare_endpoint_bridge_does_not_launch_ros2_or_native_modules():
    forbidden_import_prefixes = (
        "runtime.adapters.ros2",
        "runtime.adapters.ros2.native_module",
        "rclpy",
        "subprocess",
    )
    forbidden_calls = {
        "NativeModule",
        "NativeModuleConfig",
        "subprocess.Popen",
        "subprocess.run",
        "subprocess.call",
        "subprocess.check_call",
        "subprocess.check_output",
    }
    violations: list[str] = []

    for path in TARE_ENDPOINT_BOUNDARY_FILES:
        rel = path.relative_to(ROOT).as_posix()
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        for lineno, imported in _absolute_imports(tree):
            if any(imported == prefix or imported.startswith(f"{prefix}.") for prefix in forbidden_import_prefixes):
                violations.append(f"{rel}:{lineno}: imports {imported}")
        for node in ast.walk(tree):
            if isinstance(node, ast.Name) and node.id in {
                "NativeModule",
                "NativeModuleConfig",
            }:
                violations.append(f"{rel}:{node.lineno}: references {node.id}")
            elif isinstance(node, ast.Call):
                name = _call_name(node)
                if name in forbidden_calls or name.startswith("subprocess."):
                    violations.append(f"{rel}:{node.lineno}: calls {name}")

    assert violations == []


def _odom(x: float, y: float, z: float = 0.0) -> Odometry:
    return Odometry(
        pose=Pose(
            position=Vector3(x=x, y=y, z=z),
            orientation=Quaternion(),
        ),
        frame_id="map",
    )


class TestTAREWaypointEmission(unittest.TestCase):
    def test_emit_waypoint_publishes_pose_stamped(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(auto_start=False)
        received: list[PoseStamped] = []
        m.exploration_goal._add_callback(received.append)

        m._emit_waypoint(3.0, 4.0, 0.0, frame_id="map")

        self.assertEqual(len(received), 1)
        pose = received[0]
        self.assertIsInstance(pose, PoseStamped)
        self.assertAlmostEqual(pose.pose.position.x, 3.0)
        self.assertAlmostEqual(pose.pose.position.y, 4.0)
        self.assertAlmostEqual(pose.pose.position.z, 0.0)
        self.assertEqual(pose.frame_id, "map")

    def test_emit_waypoint_can_override_output_frame_for_live_contract(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(auto_start=False, goal_frame_id="odom")
        received: list[PoseStamped] = []
        m.exploration_goal._add_callback(received.append)

        m._emit_waypoint(3.0, 4.0, 0.0, frame_id="map")

        self.assertEqual(len(received), 1)
        self.assertEqual(received[0].frame_id, "odom")
        self.assertEqual(m._last_goal_candidates, [(3.0, 4.0)])

    def test_far_waypoint_is_suppressed_when_distance_limit_is_enabled(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(
            auto_start=False,
            max_waypoint_distance_m=8.0,
        )
        received: list[PoseStamped] = []
        m.exploration_goal._add_callback(received.append)
        m._on_odom(_odom(0.0, 0.0))

        m._emit_waypoint(12.0, 0.0, 0.0, frame_id="odom")
        self.assertIn("waypoint_distance", m._last_waypoint_reject_reason)
        m._emit_waypoint(5.0, 0.0, 0.0, frame_id="odom")

        self.assertEqual(len(received), 1)
        self.assertAlmostEqual(received[0].pose.position.x, 5.0)
        self.assertEqual(m._suppressed_far_waypoint_count, 1)
        self.assertEqual(m._last_waypoint_reject_reason, "")

    def test_hold_active_goal_suppresses_waypoint_churn_until_terminal(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(
            auto_start=False,
            hold_active_goal_until_terminal=True,
        )
        received: list[PoseStamped] = []
        m.exploration_goal._add_callback(received.append)

        m._emit_waypoint(1.0, 0.0, 0.0, frame_id="map")
        m._emit_waypoint(2.0, 0.0, 0.0, frame_id="map")

        self.assertEqual(len(received), 1)
        self.assertEqual(received[0].pose.position.x, 1.0)
        self.assertEqual(m._suppressed_waypoint_count, 1)

        m._on_navigation_status(
            {
                "state": "FAILED",
                "goal": [1.0, 0.0, 0.0],
                "ts": 10.0,
            }
        )
        m._emit_waypoint(2.0, 0.0, 0.0, frame_id="map")

        self.assertEqual(len(received), 2)
        self.assertEqual(received[-1].pose.position.x, 2.0)

    def test_waypoint_count_increments(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(auto_start=False)
        self.assertEqual(m._waypoint_count, 0)
        m._emit_waypoint(1.0, 1.0, 0.0)
        m._emit_waypoint(2.0, 2.0, 0.0)
        self.assertEqual(m._waypoint_count, 2)
        self.assertGreater(m._last_waypoint_ts, 0.0)

    def test_path_strategy_mode_keeps_waypoint_as_diagnostic_only(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(
            auto_start=False,
            prefer_path_strategy=True,
            path_strategy_fallback_to_waypoint=False,
        )
        received: list[PoseStamped] = []
        m.exploration_goal._add_callback(received.append)

        m._emit_waypoint(3.0, 4.0, 0.0, frame_id="map")

        self.assertEqual(m._waypoint_count, 1)
        self.assertEqual(received, [])

    def test_suppressed_waypoint_does_not_replace_active_strategy_goal_candidates(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(
            auto_start=False,
            prefer_path_strategy=True,
            path_goal_min_distance_m=0.5,
            path_strategy_fallback_to_waypoint=False,
        )
        m._on_odom(_odom(0.0, 0.0))
        m._publish_strategy_path_if_ready(
            [
                {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
                {"x": 1.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
                {"x": 2.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
            ]
        )

        self.assertEqual(m._last_goal_candidates, [(1.0, 0.0), (2.0, 0.0)])

        m._emit_waypoint(9.0, 9.0, 0.0, frame_id="map")
        m._on_navigation_status(
            {
                "state": "SUCCESS",
                "goal": [1.0, 0.0, 0.0],
                "ts": 10.0,
            }
        )

        self.assertEqual(m._last_goal_candidates, [(1.0, 0.0), (2.0, 0.0)])
        self.assertEqual(m._navigation_success_count, 1)

    def test_path_strategy_filters_near_robot_points(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(
            auto_start=False,
            prefer_path_strategy=True,
            path_goal_min_distance_m=0.5,
            path_goal_spacing_m=0.75,
        )
        pts = [
            {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
            {"x": 0.2, "y": 0.0, "z": 0.0, "frame_id": "map"},
            {"x": 1.1, "y": 0.0, "z": 0.0, "frame_id": "map"},
            {"x": 1.4, "y": 0.0, "z": 0.0, "frame_id": "map"},
            {"x": 2.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
        ]
        m._on_odom(_odom(0.0, 0.0))

        filtered = m._strategy_goals_from_path(pts)

        self.assertEqual(
            [(p["x"], p["y"], p["z"]) for p in filtered],
            [(1.1, 0.0, 0.0), (2.0, 0.0, 0.0)],
        )

    def test_default_path_topic_uses_tare_local_strategy_path(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(auto_start=False)

        self.assertEqual(m._path_topic, "/exploration/local_path")

    def test_path_is_diagnostic_only_unless_strategy_mode_enabled(self):
        from types import SimpleNamespace

        from explore.tare.module import TAREExplorerModule

        msg = SimpleNamespace(
            poses=[
                SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(
                            x=0.0,
                            y=0.0,
                            z=0.0,
                        )
                    )
                ),
                SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(
                            x=1.0,
                            y=0.0,
                            z=0.0,
                        )
                    )
                ),
            ]
        )
        m = TAREExplorerModule(auto_start=False, prefer_path_strategy=False)
        received: list[list[dict]] = []
        m.exploration_path._add_callback(received.append)

        m._process_dds_path(
            SimpleNamespace(
                valid=True,
                frame_id="map",
                poses=[
                    SimpleNamespace(
                        x=pose.pose.position.x,
                        y=pose.pose.position.y,
                        z=pose.pose.position.z,
                    )
                    for pose in msg.poses
                ],
            )
        )

        self.assertEqual(m._path_count, 1)
        self.assertEqual(received, [])

    def test_path_strategy_mode_emits_filtered_strategy_path(self):
        from types import SimpleNamespace

        from explore.tare.module import TAREExplorerModule

        msg = SimpleNamespace(
            poses=[
                SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(
                            x=0.0,
                            y=0.0,
                            z=0.0,
                        )
                    )
                ),
                SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(
                            x=0.2,
                            y=0.0,
                            z=0.0,
                        )
                    )
                ),
                SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(
                            x=1.2,
                            y=0.0,
                            z=0.0,
                        )
                    )
                ),
                SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(
                            x=2.2,
                            y=0.0,
                            z=0.0,
                        )
                    )
                ),
            ]
        )
        m = TAREExplorerModule(
            auto_start=False,
            prefer_path_strategy=True,
            path_goal_min_distance_m=1.0,
        )
        received: list[list[dict]] = []
        m.exploration_path._add_callback(received.append)
        m._on_odom(_odom(0.0, 0.0))

        m._process_dds_path(
            SimpleNamespace(
                valid=True,
                frame_id="map",
                poses=[
                    SimpleNamespace(
                        x=pose.pose.position.x,
                        y=pose.pose.position.y,
                        z=pose.pose.position.z,
                    )
                    for pose in msg.poses
                ],
            )
        )

        self.assertEqual(m._path_count, 1)
        self.assertEqual(len(received), 1)
        self.assertEqual(received[-1][0]["x"], 1.2)
        self.assertEqual(received[-1][1]["x"], 2.2)

    def test_path_strategy_reanchors_loop_to_current_odometry(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(
            auto_start=False,
            prefer_path_strategy=True,
            path_goal_min_distance_m=0.5,
            path_goal_spacing_m=0.75,
            path_start_tolerance_m=1.0,
            path_max_goal_count=4,
        )
        pts = [
            {"x": 2.2, "y": -0.2, "z": 0.75, "frame_id": "map"},
            {"x": 5.8, "y": 1.0, "z": 0.75, "frame_id": "map"},
            {"x": 6.2, "y": 1.0, "z": 0.75, "frame_id": "map"},
            {"x": 6.6, "y": 1.0, "z": 0.75, "frame_id": "map"},
            {"x": 7.0, "y": 1.0, "z": 0.75, "frame_id": "map"},
            {"x": 7.4, "y": 1.0, "z": 0.75, "frame_id": "map"},
            {"x": 8.2, "y": 1.0, "z": 0.75, "frame_id": "map"},
        ]
        m._on_odom(_odom(6.82, 1.54, 0.75))

        strategy = m._strategy_goals_from_path(pts)

        self.assertGreaterEqual(len(strategy), 2)
        self.assertNotEqual((strategy[0]["x"], strategy[0]["y"]), (2.2, -0.2))
        self.assertEqual((strategy[0]["x"], strategy[0]["y"]), (7.0, 1.0))

    def test_path_strategy_rejects_paths_not_anchored_near_odometry(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(
            auto_start=False,
            prefer_path_strategy=True,
            path_start_tolerance_m=0.5,
        )
        pts = [
            {"x": 10.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
            {"x": 11.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
        ]
        m._on_odom(_odom(0.0, 0.0))

        strategy = m._strategy_goals_from_path(pts)

        self.assertEqual(strategy, [])
        self.assertEqual(m._last_strategy_path_reject_reason, "path_not_near_odom")

    def test_navigation_status_updates_tare_goal_result_counters(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(auto_start=False)
        m._emit_waypoint(3.0, 4.0, 0.0, frame_id="map")

        m._on_navigation_status(
            {
                "state": "SUCCESS",
                "goal": [3.05, 3.95, 0.0],
                "ts": 10.0,
            }
        )

        self.assertEqual(m._navigation_terminal_count, 1)
        self.assertEqual(m._navigation_success_count, 1)
        self.assertEqual(m._navigation_failure_count, 0)
        self.assertEqual(m._last_navigation_status["state"], "SUCCESS")

    def test_navigation_status_success_overrides_same_goal_failure(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(auto_start=False)
        m._emit_waypoint(3.0, 4.0, 0.0, frame_id="map")

        m._on_navigation_status(
            {
                "state": "FAILED",
                "goal": [3.05, 3.95, 0.0],
                "failure_reason": "empty_path",
                "ts": 10.0,
            }
        )
        m._on_navigation_status(
            {
                "state": "SUCCESS",
                "goal": [3.05, 3.95, 0.0],
                "ts": 12.0,
            }
        )

        self.assertEqual(m._navigation_terminal_count, 1)
        self.assertEqual(m._navigation_success_count, 1)
        self.assertEqual(m._navigation_failure_count, 0)
        self.assertEqual(m._last_navigation_status["state"], "SUCCESS")

    def test_navigation_status_ignores_failure_after_same_goal_success(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(auto_start=False)
        m._emit_waypoint(3.0, 4.0, 0.0, frame_id="map")

        m._on_navigation_status(
            {
                "state": "SUCCESS",
                "goal": [3.05, 3.95, 0.0],
                "ts": 10.0,
            }
        )
        m._on_navigation_status(
            {
                "state": "FAILED",
                "goal": [3.05, 3.95, 0.0],
                "failure_reason": "late_duplicate",
                "ts": 12.0,
            }
        )

        self.assertEqual(m._navigation_terminal_count, 1)
        self.assertEqual(m._navigation_success_count, 1)
        self.assertEqual(m._navigation_failure_count, 0)
        self.assertEqual(m._last_navigation_status["state"], "SUCCESS")

    def test_navigation_status_for_other_goal_is_ignored(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(auto_start=False, navigation_goal_match_tolerance_m=0.5)
        m._emit_waypoint(3.0, 4.0, 0.0, frame_id="map")

        m._on_navigation_status(
            {
                "state": "FAILED",
                "goal": [10.0, 10.0, 0.0],
                "failure_reason": "planner_failed",
                "ts": 10.0,
            }
        )

        self.assertEqual(m._navigation_terminal_count, 0)
        self.assertEqual(m._navigation_failure_count, 0)


class TestTAREStatsSnapshot(unittest.TestCase):
    def test_stats_schema(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(auto_start=False)
        received: list[dict] = []
        m.tare_stats._add_callback(received.append)
        m._publish_stats()
        self.assertEqual(len(received), 1)
        snap = received[-1]
        for key in (
            "alive",
            "started",
            "healthy",
            "waypoint_count",
            "waypoint_age_s",
            "last_runtime_ms",
            "finished",
            "navigation_terminal_count",
            "navigation_success_count",
            "navigation_failure_count",
            "last_navigation_status",
            "configured_backend",
            "backend",
            "degraded",
            "degraded_reason",
        ):
            self.assertIn(key, snap)
        self.assertEqual(snap["configured_backend"], "tare")
        self.assertEqual(snap["backend"], "tare")
        self.assertFalse(snap["degraded"])
        self.assertEqual(snap["degraded_reason"], "")

    def test_stats_report_unhealthy_before_first_waypoint(self):
        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(auto_start=False, way_point_timeout_s=1.0)
        received: list[dict] = []
        m.tare_stats._add_callback(received.append)
        m._publish_stats()
        self.assertFalse(received[-1]["healthy"])
        # inf encoded as large number in dict form
        self.assertEqual(received[-1]["waypoint_count"], 0)

    def test_cyclonedds_domain_uses_ros_domain_id(self):
        from explore.tare.module import TAREExplorerModule

        old = os.environ.get("ROS_DOMAIN_ID")
        try:
            os.environ["ROS_DOMAIN_ID"] = "73"
            self.assertEqual(TAREExplorerModule._ros_domain_id(), 73)
            os.environ["ROS_DOMAIN_ID"] = "bad"
            self.assertEqual(TAREExplorerModule._ros_domain_id(), 0)
        finally:
            if old is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = old


class TestTARESkills(unittest.TestCase):
    def test_get_tare_status_returns_json(self):
        import json

        from explore.tare.module import TAREExplorerModule

        m = TAREExplorerModule(auto_start=False)
        parsed = json.loads(m.get_tare_status())
        self.assertIn("alive", parsed)
        self.assertIn("waypoint_count", parsed)
        self.assertEqual(parsed["configured_backend"], "tare")
        self.assertEqual(parsed["backend"], "tare")
        self.assertFalse(parsed["degraded"])
        self.assertEqual(parsed["degraded_reason"], "")


# Stack factory


def _module_names(bp) -> set[str]:
    """Return the set of registered module names/aliases in a Blueprint."""
    return {e.name for e in bp._entries}


class TestExplorationStackFactory(unittest.TestCase):
    def test_none_returns_empty_blueprint(self):
        from lingtu.assembly.stacks.exploration import exploration

        bp = exploration(backend="none")
        self.assertEqual(len(bp._entries), 0)

    def test_empty_backend_returns_empty(self):
        from lingtu.assembly.stacks.exploration import exploration

        bp = exploration(backend="")
        self.assertEqual(len(bp._entries), 0)

    def test_unknown_backend_raises(self):
        from lingtu.assembly.stacks.exploration import exploration

        with self.assertRaises(ValueError):
            exploration(backend="bogus")

    def test_tare_external_adds_bridge_without_native_binary(self):
        from lingtu.assembly.stacks.exploration import exploration

        bp = exploration(backend="tare_external", auto_start=False)
        names = _module_names(bp)

        self.assertIn("TAREExplorerModule", names)
        self.assertIn("ExplorationSupervisorModule", names)
        self.assertNotIn("tare_explorer", names)
        tare = next(e for e in bp._entries if e.name == "TAREExplorerModule")
        self.assertIs(tare.config["prefer_path_strategy"], False)
        self.assertEqual(tare.config["configured_backend"], "tare_external")

    def test_full_stack_forwards_external_tare_supervisor_timeout(self):
        from lingtu.assembly.products.host import host_blueprint

        bp = host_blueprint(
            robot="stub",
            slam_profile="none",
            llm="mock",
            planner_backend="astar",
            enable_native=False,
            enable_semantic=False,
            enable_gateway=False,
            run_startup_checks=False,
            exploration_backend="tare_external",
            exploration_auto_start=False,
            tare_warn_timeout_s=45.0,
            tare_fallback_timeout_s=180.0,
            tare_supervisor_hz=2.0,
        )

        supervisor = next(e for e in bp._entries if e.name == "ExplorationSupervisorModule")
        self.assertEqual(supervisor.config["warn_timeout_s"], 45.0)
        self.assertEqual(supervisor.config["fallback_timeout_s"], 180.0)
        self.assertEqual(supervisor.config["poll_hz"], 2.0)

    def test_tare_adds_bridge_without_native_binary(self):
        """LingTu TARE is in-process by default; no native binary required."""
        from lingtu.assembly.stacks.exploration import exploration

        bp = exploration(backend="tare", auto_start=False)
        names = _module_names(bp)

        self.assertIn("TAREExplorerModule", names)
        self.assertIn("ExplorationSupervisorModule", names)
        self.assertNotIn("TAREPlannerNativeModule", names)
        tare = next(e for e in bp._entries if e.name == "TAREExplorerModule")
        self.assertEqual(tare.config["configured_backend"], "tare")

    def test_native_tare_owner_mounts_no_python_goal_producer(self):
        from lingtu.assembly.stacks.exploration import exploration

        bp = exploration(backend="tare", owner="native", auto_start=False)

        self.assertEqual(_module_names(bp), set())

    def test_native_field_stack_mounts_command_client_not_python_tare(self):
        from lingtu.assembly.products.host import host_blueprint

        bp = host_blueprint(
            robot="stub",
            slam_profile="none",
            llm="mock",
            planner_backend="astar",
            enable_native=False,
            enable_semantic=False,
            enable_gateway=False,
            run_startup_checks=False,
            native_navigation_endpoint="lingtu-nav-dds",
            command_output_mode="endpoint_only",
            exploration_backend="tare",
            exploration_auto_start=False,
        )
        names = _module_names(bp)

        self.assertIn("nav.commands", names)
        self.assertNotIn("TAREExplorerModule", names)
        self.assertNotIn("ExplorationSupervisorModule", names)


# --- TARE source boundary ----------------------------------------------------


class TestTareTopicContract(unittest.TestCase):
    def test_tare_planner_source_tree_is_not_vendored(self):
        self.assertFalse((ROOT / "src/explore/tare/planner").exists())
        self.assertTrue((ROOT / "src/explore/tare/policy.py").exists())

    def test_tare_remaps_cover_external_runtime_contract(self):
        """External compatibility remaps stay small and data-only."""
        from explore.tare.topics import TARE_REMAPS

        remaps = TARE_REMAPS
        self.assertEqual(remaps, TARE_REMAPS)
        self.assertEqual(remaps.get("/state_estimation"), TOPICS.odometry)
        self.assertEqual(remaps.get("/state_estimation_at_scan"), TOPICS.odometry)
        self.assertEqual(remaps.get("/registered_scan"), TOPICS.map_cloud)
        self.assertEqual(remaps.get("/terrain_map"), "/nav/terrain_map")
        self.assertEqual(remaps.get("/terrain_map_ext"), "/nav/terrain_map_ext")
        self.assertEqual(remaps.get("/way_point"), "/exploration/way_point")
        self.assertEqual(remaps.get("/global_path"), "/exploration/global_path")
        self.assertEqual(remaps.get("/local_path"), "/exploration/local_path")
        self.assertEqual(remaps.get("/runtime_breakdown"), "/exploration/runtime_breakdown")
        self.assertEqual(
            remaps.get("/to_nearest_global_subspace_path"),
            "/exploration/to_nearest_global_subspace_path",
        )
        self.assertEqual(remaps.get("/start_exploration"), "/exploration/start")


if __name__ == "__main__":
    unittest.main()
