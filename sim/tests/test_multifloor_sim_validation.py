from __future__ import annotations

import pickle

import pytest

pytest.importorskip("rclpy", reason="Needs ROS2 runtime")
from runtime.tests.numpy_guard import import_numpy_or_skip

np = import_numpy_or_skip()

pytestmark = [pytest.mark.sim]

from nav.services.plan.global_planner.algorithm.pct.runtime.api import prepare_tomogram_for_pct
from sim.engine.scenarios.multifloor_assets import build_multifloor_assets
import sim.scripts.multifloor_nav_validation as multifloor_nav_validation
from sim.scripts.multifloor_nav_validation import (
    ROUTE_CASES,
    _native_pct_gate,
    _frontier_probe_costmap,
    _pct_runtime_evidence,
    _nav_path_from_points,
    _odom_at_path_start,
    _path_initial_yaw,
    _pose_stamped,
    _prepare_local_path_for_follower,
    _required_planner_ok,
    _split_bridge_motion_segments,
    run_command_flow,
    run_frontier_exploration_probe,
    run_frontier_navigation_closed_loop_probe,
    run_global_planner,
    run_lidar_localization_contract,
    run_mujoco_bridge_loop,
    run_route_case,
    run_route_matrix,
    run_route_planner,
    validate_transition_against_metadata,
)


def test_multifloor_assets_have_floor_transition_gateway(tmp_path):
    assets = build_multifloor_assets(tmp_path)

    assert assets.scene_xml.exists()
    assert assets.tomogram.exists()
    assert assets.map_pcd.exists()
    assert assets.metadata.exists()
    assert assets.floors == (0.0, 2.5)

    with assets.tomogram.open("rb") as fh:
        raw = pickle.load(fh)
    assert raw["grid_info"]["axis_order"] == "builder_xy"
    assert raw["floor_transitions"][0]["kind"] == "stairs"

    prepared = prepare_tomogram_for_pct(assets.tomogram)
    with prepared.open("rb") as fh:
        normalized = pickle.load(fh)
    trav = normalized["data"][0]
    elev_g = normalized["data"][3]
    mask_t = (trav[:-1] <= 49.9) & (trav[1:] <= 49.9)
    mask_g = np.abs(elev_g[1:] - elev_g[:-1]) < 0.5
    assert int(np.sum(mask_t & mask_g)) > 0


def test_multifloor_assets_publish_same_source_saved_map_metadata(tmp_path):
    from runtime.same_source_map_artifacts import validate_saved_map_artifact_dir

    assets = build_multifloor_assets(tmp_path)

    result = validate_saved_map_artifact_dir(
        assets.tomogram.parent,
        require_tomogram=True,
        expected_source_profile="multifloor_synthetic_assets",
        expected_data_source="synthetic_multifloor_geometry",
        expected_frame_id="map",
    )

    assert result["ok"] is True, result["blockers"]
    assert result["checked_required_artifacts"] == ["map_pcd", "tomogram"]
    assert result["artifacts"]["map_pcd"]["sha256_ok"] is True
    assert result["artifacts"]["tomogram"]["sha256_ok"] is True


def test_multifloor_lidar_localization_and_frontier_contracts():
    localization = run_lidar_localization_contract()
    exploration = run_frontier_exploration_probe()

    assert localization["ok"] is True
    assert localization["odom_frame"] == "map"
    assert all(sample["localization_state"] == "LOCKED" for sample in localization["samples"])
    assert all(sample["localization_state"] == "LOCKED" for sample in localization["robustness_samples"])
    assert all(sample["localization_state"] == "LOST" for sample in localization["rejection_samples"])
    assert localization["replay_source"] == "deterministic_simulated_scan_pose_sequence"
    assert localization["not_fastlio2_rosbag"] is True
    assert localization["real_fastlio2_rosbag_verified"] is False
    assert [frame["localization_state"] for frame in localization["frames"]] == [
        "LOCKED",
        "LOCKED",
        "LOST",
        "RECOVERED",
        "LOCKED",
    ]
    assert [item["to"] for item in localization["transitions"]] == ["LOST", "RECOVERED", "LOCKED"]
    assert localization["drift_m"] > 0.8
    assert localization["recovery_count"] == 1
    assert localization["final_state"] == "LOCKED"
    assert exploration["ok"] is True
    assert exploration["closed_loop"] is False
    assert exploration["probe_mode"] == "candidate_only"
    assert exploration["frontier_count"] >= 1
    assert exploration["goal_count"] >= 1


def test_frontier_navigation_closed_loop_probe_reveals_map_and_replays_tracking(tmp_path):
    report = run_frontier_navigation_closed_loop_probe(output_dir=tmp_path / "frontier_loop", rounds=2)

    assert report["ok"] is True
    assert report["closed_loop"] is True
    assert report["probe_mode"] == "frontier_navigation_closed_loop"
    assert report["simulation_only"] is True
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["requested_rounds"] == 2
    assert report["rounds_completed"] == 2
    assert report["goal_reached_events"] == 2
    assert report["known_cells_delta"] > 0
    assert len(report["rounds"]) == 2

    known_after = report["known_cells_initial"]
    for item in report["rounds"]:
        assert item["ok"] is True
        assert item["goal"]["frontier"]
        assert item["goal"]["navigation"]
        assert item["goal"]["safe_goal_tolerance"] == pytest.approx(1.2)
        assert item["goal"]["safe"]
        assert len(item["path"]) >= 2
        assert item["path_validation"]["ok"] is True
        assert item["command_flow"]["ok"] is True
        assert item["command_flow"]["cmd_vel_sent_to_hardware"] is False
        assert item["tracking_replay"]["ok"] is True
        assert item["tracking_replay"]["cmd_vel_sent_to_hardware"] is False
        assert item["goal_reached"] is True
        assert item["map_update"]["known_cells_delta"] > 0
        assert item["map_update"]["known_cells_after"] > known_after
        known_after = item["map_update"]["known_cells_after"]


def test_frontier_probe_costmap_starts_on_known_free_space():
    costmap = _frontier_probe_costmap()
    grid = costmap["grid"]
    x_idx = int(round((ROUTE_CASES["same_floor"].start[0] - costmap["origin_x"]) / costmap["resolution"]))
    y_idx = int(round((ROUTE_CASES["same_floor"].start[1] - costmap["origin_y"]) / costmap["resolution"]))

    assert grid[y_idx, x_idx] == 0


def test_command_flow_replay_faces_the_first_path_segment():
    reverse_path = [
        [3.0, 0.0, 0.0],
        [2.6, -0.4, 0.0],
        [2.2, -0.8, 0.0],
    ]

    assert _path_initial_yaw(reverse_path) == pytest.approx(-2.35619449)
    assert _odom_at_path_start(reverse_path).yaw == pytest.approx(-2.35619449)


def test_command_flow_reports_requested_and_effective_algorithm_backends():
    report = run_command_flow(
        [
            [0.0, 0.0, 0.0],
            [0.8, 0.0, 0.0],
            [1.2, 0.0, 0.0],
        ],
        local_planner_backend="simple",
    )

    assert report["ok"] is True
    assert report["local_planner_backend"] == "simple"
    assert report["local_planner_backend_requested"] == "simple"
    assert report["local_planner_backend_actual"] == "simple"
    assert report["path_follower_backend_requested"] == "pid"
    assert report["path_follower_backend_actual"] == "pid"
    assert report["algorithm_backends"]["local_planner"]["requested"] == "simple"
    assert report["algorithm_backends"]["local_planner"]["backend_actual"] == "simple"
    assert report["algorithm_backends"]["local_planner"]["exercised_by"] == "command_flow"
    assert report["algorithm_backends"]["path_follower"]["requested"] == "pid"
    assert report["algorithm_backends"]["path_follower"]["backend_actual"] == "pid"
    assert report["algorithm_backends"]["path_follower"]["exercised_by"] == "command_flow"


def test_frontier_loop_gate_rejects_candidate_only_probe(tmp_path):
    candidate_only = run_frontier_exploration_probe()

    report = run_route_case(
        case=ROUTE_CASES["same_floor"],
        output_dir=tmp_path / "case_candidate_only",
        planners=["astar"],
        downsample_dist=0.5,
        skip_mujoco=True,
        frontier_loop=True,
        shared_exploration=candidate_only,
    )

    assert candidate_only["ok"] is True
    assert candidate_only["closed_loop"] is False
    assert report["exploration"]["probe_mode"] == "candidate_only"
    assert report["passed"] is False


def test_route_matrix_can_use_frontier_closed_loop_report(tmp_path, monkeypatch):
    def fake_frontier_loop(**kwargs):
        return {
            "ok": True,
            "closed_loop": True,
            "probe_mode": "frontier_navigation_closed_loop",
            "requested_rounds": kwargs["rounds"],
            "rounds_completed": kwargs["rounds"],
            "known_cells_delta": 10,
            "goal_reached_events": kwargs["rounds"],
            "rounds": [
                {
                    "ok": True,
                    "goal": {"frontier": [1.0, 0.0, 0.0], "navigation": [1.0, 0.0, 0.0]},
                    "path": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                    "path_validation": {"ok": True},
                    "command_flow": {"ok": True},
                    "tracking_replay": {"ok": True},
                    "goal_reached": True,
                    "map_update": {"known_cells_delta": 10},
                }
            ],
        }

    monkeypatch.setattr(
        "sim.scripts.multifloor_nav_validation.run_frontier_navigation_closed_loop_probe",
        fake_frontier_loop,
    )

    report = run_route_matrix(
        output_dir=tmp_path / "matrix_frontier_loop",
        planners=["astar"],
        downsample_dist=0.5,
        skip_mujoco=True,
        frontier_loop=True,
        frontier_rounds=2,
        routes=("same_floor",),
    )

    assert report["passed"] is True
    assert report["frontier_loop_enabled"] is True
    assert report["frontier_rounds"] == 2
    assert report["exploration"]["closed_loop"] is True
    assert report["cases"][0]["exploration"]["probe_mode"] == "frontier_navigation_closed_loop"


def test_multifloor_same_floor_astar_drives_memory_only_command_flow(tmp_path):
    assets = build_multifloor_assets(tmp_path, goal=(1.9, -1.25, 0.0))
    plan = run_route_planner(
        planner="astar",
        route="same_floor",
        tomogram=assets.tomogram,
        start=assets.start,
        goal=assets.goal,
        downsample_dist=0.5,
    )

    assert plan["feasible"] is True
    assert plan["count"] >= 2
    assert plan["z_min"] == 0.0
    assert plan["z_max"] == 0.0

    case = run_route_case(
        case=ROUTE_CASES["same_floor"],
        output_dir=tmp_path / "case",
        planners=["astar"],
        downsample_dist=0.5,
        skip_mujoco=True,
    )
    assert case["passed"] is True
    assert case["validation_level"] == "kinematic_module_ports"
    assert case["physical_gait_verified"] is False
    assert case["slam_verified"] is False
    assert case["real_lidar_verified"] is False
    assert case["cross_floor_physical_verified"] is False
    assert case["native_pct_gate"]["required"] is False
    assert case["native_pct_gate"]["ok"] is True
    assert case["planning"][0]["path_validation"]["ok"] is True
    assert case["tracking_replay"]["ok"] is True
    assert case["tracking_replay"]["cmd_vel_sent_to_hardware"] is False
    assert case["tracking_replay"]["speed_limit_ok"] is True
    assert case["tracking_replay"]["stop_ok"] is True
    assert case["tracking_replay"]["tracking_error_ok"] is True

    command_flow = run_command_flow(plan["path"])
    assert command_flow["ok"] is True
    assert command_flow["driver_used"] is False
    assert command_flow["goal_sent"] is False
    assert command_flow["cmd_vel_sent_to_hardware"] is False
    assert command_flow["production_local_planner_verified"] is False
    assert command_flow["seen"]["local_path"] >= 1
    assert command_flow["seen"]["mux_cmd"] >= 1


def test_multifloor_upper_floor_astar_reaches_requested_goal(tmp_path):
    case = ROUTE_CASES["upper_floor"]
    assets = build_multifloor_assets(tmp_path, start=case.start, goal=case.goal)
    plan = run_route_planner(
        planner="astar",
        route=case.route_mode,
        tomogram=assets.tomogram,
        start=case.start,
        goal=case.goal,
        downsample_dist=0.5,
    )

    assert plan["feasible"] is True
    assert plan["error"] is None
    assert plan["safe_goal_distance_m"] == pytest.approx(0.0)
    assert plan["z_min"] >= 2.0
    assert plan["z_max"] == pytest.approx(2.5)
    assert plan["last"][:2] == pytest.approx(list(case.goal[:2]))

    report = run_route_case(
        case=case,
        output_dir=tmp_path / "upper_case",
        planners=["astar"],
        downsample_dist=0.5,
        skip_mujoco=True,
    )
    assert report["passed"] is True
    assert report["planning"][0]["path_validation"]["ok"] is True
    assert report["command_flow"]["ok"] is True
    assert report["command_flow"]["cmd_vel_sent_to_hardware"] is False
    assert report["tracking_replay"]["ok"] is True
    assert report["tracking_replay"]["cmd_vel_sent_to_hardware"] is False


def test_pct_missing_runtime_or_tomogram_is_blocked_not_passed(tmp_path):
    plan = run_global_planner(
        planner="pct",
        tomogram=tmp_path / "missing_tomogram.pickle",
        start=(0.0, 0.0, 0.0),
        goal=(1.0, 0.0, 0.0),
        downsample_dist=0.5,
    )

    assert plan["feasible"] is False
    assert plan["status"] == "blocked"
    assert plan["blocked"] is True
    assert plan["native_backend_used"] is False
    assert isinstance(plan["native_runtime"], dict)

    gate = _native_pct_gate(
        planners=["pct"],
        planning=[plan],
        case=ROUTE_CASES["same_floor"],
    )

    assert gate["ok"] is False
    assert gate["status"] == "blocked"
    assert gate["blocked"] is True


def test_pct_runtime_evidence_exposes_host_blocker_details(monkeypatch):
    def fake_inspect_pct_runtime(_root):
        return {
            "ok": False,
            "canonical_arch": "x86_64",
            "python_tag": "cpython-313",
            "known_good_python_tag": "cpython-310",
            "python_abi_matches_known_good": False,
            "platform_system": "Windows",
            "os_name": "nt",
            "native_binary_format": "linux_elf",
            "host_platform_supported": False,
            "host_platform_blocker": "linux_elf extensions are not runnable on Windows",
            "candidate_diagnostics": [{"path": "pct.cpython-310-aarch64-linux-gnu.so"}],
            "recommended_build_command": "bash scripts/build_pct.sh",
        }

    monkeypatch.setattr(
        multifloor_nav_validation,
        "inspect_pct_runtime",
        fake_inspect_pct_runtime,
    )

    evidence = _pct_runtime_evidence()

    assert evidence["ok"] is False
    assert evidence["platform_system"] == "Windows"
    assert evidence["host_platform_supported"] is False
    assert evidence["known_good_python_tag"] == "cpython-310"
    assert evidence["python_abi_matches_known_good"] is False
    assert evidence["native_binary_format"] == "linux_elf"
    assert evidence["candidate_diagnostics"][0]["path"].endswith(".so")
    assert evidence["recommended_build_command"] == "bash scripts/build_pct.sh"


def test_partial_global_plan_is_kept_for_diagnostics_but_not_feasible(monkeypatch, tmp_path):
    class FakeBackend:
        available = True

    class FakeGlobalPlanner:
        def __init__(self, **_: object) -> None:
            self._backend = FakeBackend()
            self.last_plan_report = {
                "reached_goal": False,
                "safe_goal": [1.0, 0.0, 0.0],
            }

        def setup(self) -> None:
            return None

        def plan(self, *_: object, **__: object) -> tuple[list[np.ndarray], float]:
            return [np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])], 4.2

    monkeypatch.setattr(
        "sim.scripts.multifloor_nav_validation.GlobalPlanner",
        FakeGlobalPlanner,
    )

    tomogram = tmp_path / "diagnostic.tomogram"
    tomogram.write_bytes(b"diagnostic")

    plan = run_global_planner(
        planner="astar",
        tomogram=tomogram,
        start=(0.0, 0.0, 0.0),
        goal=(2.0, 0.0, 0.0),
        downsample_dist=0.5,
    )

    assert plan["reached_goal"] is False
    assert plan["feasible"] is False
    assert plan["ok"] is False
    assert plan["status"] == "fail"
    assert plan["path"] == [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
    assert plan["error"] == "planner returned partial path without reaching goal"


def test_far_projected_safe_goal_is_not_counted_as_requested_goal(monkeypatch, tmp_path):
    class FakeBackend:
        available = True

    class FakeGlobalPlanner:
        def __init__(self, **_: object) -> None:
            self._backend = FakeBackend()
            self.last_plan_report = {
                "reached_goal": True,
                "safe_goal": [1.0, 0.0, 0.0],
            }

        def setup(self) -> None:
            return None

        def plan(self, *_: object, **__: object) -> tuple[list[np.ndarray], float]:
            return [np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])], 4.2

    monkeypatch.setattr(
        "sim.scripts.multifloor_nav_validation.GlobalPlanner",
        FakeGlobalPlanner,
    )

    tomogram = tmp_path / "diagnostic.tomogram"
    tomogram.write_bytes(b"diagnostic")

    plan = run_global_planner(
        planner="astar",
        tomogram=tomogram,
        start=(0.0, 0.0, 0.0),
        goal=(3.0, 0.0, 0.0),
        downsample_dist=0.5,
        safe_goal_tolerance=0.0,
    )

    assert plan["backend_reached_goal"] is True
    assert plan["reached_goal"] is False
    assert plan["feasible"] is False
    assert plan["ok"] is False
    assert plan["status"] == "fail"
    assert plan["safe_goal_distance_m"] == pytest.approx(2.0)
    assert plan["error"] == "planner safe goal is too far from requested goal"


def test_pct_global_plan_reports_effective_planner_after_fallback(monkeypatch, tmp_path):
    class NativePCTBackend:
        available = True
        _load_error = ""

    class AstarFallbackBackend:
        available = True
        _load_error = ""

    class FakeGlobalPlanner:
        def __init__(self, planner_name: str, **_: object) -> None:
            self._planner_name = planner_name
            self._backend = NativePCTBackend()
            self._fallback_backend = AstarFallbackBackend()
            self.last_plan_report = {}

        def setup(self) -> None:
            return None

        def plan(self, start, goal, **_: object) -> tuple[list[np.ndarray], float]:
            self.last_plan_report = {
                "primary_planner": self._planner_name,
                "selected_planner": "astar",
                "fallback_reason": "pct path_safety failed",
                "policy": "fallback_astar",
                "rejected_plans": [
                    {"planner": self._planner_name, "reason": "unsafe_primary_path"}
                ],
                "reached_goal": True,
                "safe_goal": [float(goal[0]), float(goal[1]), 0.0],
            }
            return [
                np.array([float(start[0]), float(start[1]), 0.0]),
                np.array([float(goal[0]), float(goal[1]), 0.0]),
            ], 3.4

    monkeypatch.setattr(
        "sim.scripts.multifloor_nav_validation.GlobalPlanner",
        FakeGlobalPlanner,
    )
    monkeypatch.setattr(
        multifloor_nav_validation,
        "_pct_runtime_evidence",
        lambda: {"ok": True, "missing": []},
    )
    tomogram = tmp_path / "diagnostic.tomogram"
    tomogram.write_bytes(b"diagnostic")

    plan = run_global_planner(
        planner="pct",
        tomogram=tomogram,
        start=(0.0, 0.0, 0.0),
        goal=(1.0, 0.0, 0.0),
        downsample_dist=0.5,
    )

    assert plan["planner"] == "pct"
    assert plan["planner_requested"] == "pct"
    assert plan["selected_planner"] == "astar"
    assert plan["fallback_reason"] == "pct path_safety failed"
    assert plan["plan_safety_policy"] == "fallback_astar"
    assert plan["rejected_plans"][0]["planner"] == "pct"
    assert plan["backend_requested_class"] == "NativePCTBackend"
    assert plan["planner_class"] == "AstarFallbackBackend"
    assert plan["native_backend_used"] is False

    gate = _native_pct_gate(
        planners=["pct"],
        planning=[{**plan, "path_validation": {"ok": True}}],
        case=ROUTE_CASES["same_floor"],
    )

    assert gate["ok"] is False
    assert gate["planner_requested"] == "pct"
    assert gate["selected_planner"] == "astar"
    assert gate["native_backend_used"] is False


def test_multifloor_astar_matrix_covers_lower_floor_routes(tmp_path):
    report = run_route_matrix(
        output_dir=tmp_path / "matrix",
        planners=["astar"],
        downsample_dist=0.5,
        skip_mujoco=True,
        routes=("same_floor", "lower_approach"),
    )

    assert report["passed"] is True
    assert report["validation_level"] == "kinematic_module_ports"
    assert report["physical_gait_verified"] is False
    assert report["slam_verified"] is False
    assert report["case_count"] == 2
    assert report["passed_count"] == 2
    assert report["cmd_vel_sent_to_hardware"] is False
    assert all(case["command_flow"]["cmd_vel_sent_to_hardware"] is False for case in report["cases"])
    assert all(case["tracking_replay"]["cmd_vel_sent_to_hardware"] is False for case in report["cases"])
    assert all(case["tracking_replay"]["ok"] is True for case in report["cases"])
    assert all(case["planning"][0]["path_validation"]["ok"] is True for case in report["cases"])


def test_multifloor_bridge_loop_splits_vertical_transition_segments():
    path = [
        [0.0, 0.0, 0.0],
        [0.8, 0.0, 0.0],
        [1.6, 0.0, 0.0],
        [1.6, 0.0, 2.5],
        [2.4, 0.0, 2.5],
        [3.2, 0.0, 2.5],
    ]

    segments, transition_edges = _split_bridge_motion_segments(path)

    assert transition_edges == 1
    assert len(segments) == 2
    assert segments[0][0] == path[0]
    assert segments[0][-1] == path[2]
    assert segments[1][0] == path[3]
    assert segments[1][-1] == path[-1]


def test_bridge_path_preparation_removes_near_start_dense_points():
    path = _nav_path_from_points([
        [0.00, 0.0, 0.0],
        [0.05, 0.0, 0.0],
        [0.12, 0.0, 0.0],
        [0.24, 0.0, 0.0],
        [0.36, 0.0, 0.0],
        [0.48, 0.0, 0.0],
    ])

    prepared = _prepare_local_path_for_follower(
        path,
        current_xy=np.asarray([0.0, 0.0], dtype=float),
        min_xy_step=0.12,
        min_start_dist=0.20,
    )

    assert len(prepared.poses) >= 2
    assert prepared.poses[0].pose.position.x >= 0.20
    assert prepared.poses[-1].pose.position.x == 0.48


def test_path_follower_nav_kernel_lookahead_stays_beyond_stop_threshold():
    from nav.local.path_follower import PathFollower

    follower = PathFollower(backend="nav_kernel", lookahead=0.8, goal_tolerance=0.25)
    follower.setup()
    try:
        if follower._backend != "nav_kernel" or follower._nc_params is None:
            pytest.skip("LingTu native navigation kernel path follower backend is not available")
        assert follower._nc_params.stop_dis_thre == pytest.approx(0.25)
        assert follower._nc_params.min_look_ahead_dis > follower._nc_params.stop_dis_thre
        assert follower._nc_params.base_look_ahead_dis >= follower._nc_params.min_look_ahead_dis
        assert follower._nc_params.max_look_ahead_dis >= follower._nc_params.min_look_ahead_dis
    finally:
        follower.stop()


def test_bridge_loop_rejects_zero_cmd_vel_segment_even_with_message_counts(monkeypatch):
    def fake_segment(*args, **kwargs):
        return {
            "ok": True,
            "simulation_only": True,
            "cmd_vel_sent_to_hardware": False,
            "odom_count": 5,
            "lidar_cloud_count": 1,
            "local_path_count": 12,
            "path_follower_cmd_count": 12,
            "mux_cmd_count": 12,
            "max_linear_x": 0.0,
            "max_linear_y": 0.0,
            "max_linear_xy": 0.0,
            "mean_abs_linear_x": 0.0,
            "mean_abs_linear_y": 0.0,
            "mean_abs_linear_xy": 0.0,
            "nonzero_linear_cmd_count": 0,
            "nonzero_linear_y_cmd_count": 0,
            "nonzero_linear_xy_cmd_count": 0,
            "moved_m": 0.0,
        }

    monkeypatch.setattr(
        "sim.scripts.multifloor_nav_validation._run_mujoco_bridge_segment",
        fake_segment,
    )

    report = run_mujoco_bridge_loop(
        [[0.0, 0.0, 0.0], [0.8, 0.0, 0.0]],
        scene_xml=__file__,
        timeout_s=0.2,
        local_planner_backend="nanobind",
    )

    assert report["ok"] is False
    assert report["segments"][0]["mux_cmd_count"] == 12


def test_local_planner_short_simple_path_still_has_two_points():
    from nav.services.plan.local_planner.service import LocalPlanner

    planner = LocalPlanner(backend="simple")
    start = np.asarray([1.0, 2.0, 0.0], dtype=float)
    goal = np.asarray([1.2, 2.1, 0.0], dtype=float)

    points = planner._straight_line(start, goal, step=0.5)

    assert len(points) == 2
    assert np.allclose(points[0], start)
    assert np.allclose(points[-1], goal)


def test_nanobind_local_planner_skips_untrackable_placeholder_path():
    from nav.services.plan.local_planner.service import LocalPlanner

    class FakeVec:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    class FakeResult:
        path = [FakeVec(0.0, 0.0, 0.0)]
        path_found = False
        recovery_state = 0

    class FakeCore:
        def set_vehicle(self, *args):
            pass

        def set_goal(self, *args):
            pass

        def plan(self, *args):
            return FakeResult()

    planner = LocalPlanner(backend="nanobind")
    planner._core = FakeCore()
    planner._latest_waypoint = _pose_stamped([1.0, 0.0, 0.0])
    planner._robot_pos = np.asarray([0.0, 0.0, 0.0], dtype=float)
    published = []
    planner.local_path._add_callback(published.append)

    planner._run_nanobind(1.0)

    assert len(published) == 1
    assert published[0].poses == []


def test_route_case_can_gate_on_mujoco_bridge_loop_without_hardware(tmp_path, monkeypatch):
    captured = {}

    def fake_bridge_loop(path_points, *, scene_xml, timeout_s, max_speed, local_planner_backend):
        captured["path_count"] = len(path_points)
        captured["scene_xml"] = scene_xml
        captured["timeout_s"] = timeout_s
        captured["max_speed"] = max_speed
        captured["local_planner_backend"] = local_planner_backend
        return {
            "ok": True,
            "simulation_only": True,
            "cmd_vel_sent_to_hardware": False,
            "production_local_planner_verified": False,
            "segment_count": 1,
            "segments": [{"ok": True, "odom_count": 5, "mux_cmd_count": 3}],
        }

    monkeypatch.setattr(
        "sim.scripts.multifloor_nav_validation.run_mujoco_bridge_loop",
        fake_bridge_loop,
    )

    report = run_route_case(
        case=ROUTE_CASES["same_floor"],
        output_dir=tmp_path / "case_bridge",
        planners=["astar"],
        downsample_dist=0.5,
        skip_mujoco=True,
        bridge_loop=True,
        bridge_loop_timeout_s=0.2,
        bridge_loop_max_speed=0.22,
    )

    assert report["passed"] is True
    assert report["simulation_motion"] is True
    assert report["simulated_driver_used"] is True
    assert report["real_robot_motion"] is False
    assert report["physical_gait_verified"] is False
    assert report["cross_floor_physical_verified"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["mujoco_bridge_loop"]["ok"] is True
    assert report["mujoco_bridge_loop"]["cmd_vel_sent_to_hardware"] is False
    assert captured["path_count"] >= 2
    assert captured["scene_xml"].exists()
    assert captured["timeout_s"] == 0.2
    assert captured["max_speed"] == 0.22
    assert captured["local_planner_backend"] == "simple"


def test_route_case_can_require_production_local_planner(tmp_path):
    report = run_route_case(
        case=ROUTE_CASES["same_floor"],
        output_dir=tmp_path / "case_require_local",
        planners=["astar"],
        downsample_dist=0.5,
        skip_mujoco=True,
        local_planner_backend="simple",
        require_production_local_planner=True,
    )

    assert report["command_flow"]["ok"] is True
    assert report["production_local_planner_required"] is True
    assert report["production_local_planner_verified"] is False
    assert report["passed"] is False


def test_cross_floor_route_composes_pct_segments_and_metadata_transition(tmp_path, monkeypatch):
    assets = build_multifloor_assets(tmp_path)

    def fake_global_planner(*, planner, tomogram, start, goal, downsample_dist):
        return {
            "planner": planner,
            "ok": True,
            "feasible": True,
            "backend_available": True,
            "count": 2,
            "distance_m": 1.0,
            "plan_ms": 1.0,
            "wall_ms": 1.0,
            "z_min": min(start[2], goal[2]),
            "z_max": max(start[2], goal[2]),
            "start": list(start),
            "goal": list(goal),
            "first": list(start),
            "last": list(goal),
            "path": [list(start), list(goal)],
            "error": None,
        }

    monkeypatch.setattr(
        "sim.scripts.multifloor_nav_validation.run_global_planner",
        fake_global_planner,
    )
    plan = run_route_planner(
        planner="pct",
        route="cross_floor",
        tomogram=assets.tomogram,
        start=assets.start,
        goal=assets.goal,
        downsample_dist=0.5,
    )

    assert plan["feasible"] is True
    assert plan["route_strategy"] == "floor_graph_pct_segments"
    assert plan["planner_scope"] == "floor_graph_pct_composition"
    assert plan["native_pct_single_plan_verified"] is False
    assert plan["cross_floor_physical_verified"] is False
    assert plan["transition_motion_verified"] is False
    assert plan["native_pct_required_segments"] == 2
    assert plan["native_pct_feasible_segments"] == 0
    assert [segment["name"] for segment in plan["segments"]] == [
        "floor_0_to_stairs",
        "floor_1_from_stairs",
    ]
    assert plan["z_max"] >= 2.5
    transition = validate_transition_against_metadata(assets.tomogram, plan["transition"])
    assert transition["ok"] is True
    assert transition["kind"] == "stairs"


def test_required_planner_gate_rejects_missing_validation_and_non_native_pct():
    case = ROUTE_CASES["upper_floor"]
    valid_native_pct = {
        "planner": "pct",
        "feasible": True,
        "backend_available": True,
        "native_backend_used": True,
        "path_validation": {"ok": True},
        "planner_scope": "native_pct_single_route",
        "count": 3,
    }

    assert _required_planner_ok(planners=["pct"], planning=[valid_native_pct], case=case) is True

    missing_path_validation = dict(valid_native_pct)
    missing_path_validation.pop("path_validation")
    assert _required_planner_ok(planners=["pct"], planning=[missing_path_validation], case=case) is False

    non_native_pct = dict(valid_native_pct)
    non_native_pct["native_backend_used"] = False
    assert _required_planner_ok(planners=["pct"], planning=[non_native_pct], case=case) is False

    gate = _native_pct_gate(planners=["pct"], planning=[non_native_pct], case=case)
    assert gate["required"] is True
    assert gate["ok"] is False
    assert gate["native_backend_used"] is False


def test_cross_floor_gate_requires_transition_validation_and_native_segments():
    case = ROUTE_CASES["cross_floor"]
    cross_floor_plan = {
        "planner": "pct",
        "feasible": True,
        "backend_available": True,
        "native_backend_used": True,
        "route_strategy": "floor_graph_pct_segments",
        "planner_scope": "floor_graph_pct_composition",
        "native_pct_required_segments": 2,
        "native_pct_feasible_segments": 2,
        "z_max": 2.5,
        "path_validation": {"ok": True},
        "transition": [[0.0, 0.0, 0.0], [0.0, 0.0, 2.5]],
        "transition_validation": {"ok": True},
    }

    assert _required_planner_ok(planners=["pct"], planning=[cross_floor_plan], case=case) is True

    missing_transition_validation = dict(cross_floor_plan)
    missing_transition_validation.pop("transition_validation")
    assert _required_planner_ok(
        planners=["pct"],
        planning=[missing_transition_validation],
        case=case,
    ) is False
