from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import pytest

pytestmark = [pytest.mark.sim]

from sim.scripts.native_pct_mujoco_gate import (
    PctRoute,
    _build_safe_follow_path,
    _default_local_planner_path_folder,
    _load_pct_route,
    _local_path_obstacle_evidence,
    _moving_obstacle_boxes,
    _native_node_commands,
    _omni_cart_cmd,
    _path_is_robot_frame,
    _path_window_by_distance,
    _ros_path_to_points,
    _sample_lidar_points,
    _select_local_target,
    _summarize_local_path_obstacle_evidence,
    _trajectory_correctness,
)


def _write_source_report(tmp_path: Path, *, native_backend_used: bool = True) -> Path:
    scene_xml = tmp_path / "scene.xml"
    scene_xml.write_text("<mujoco><worldbody/></mujoco>\n", encoding="utf-8")
    tomogram = tmp_path / "tomogram.pickle"
    tomogram.write_bytes(b"pct test tomogram")
    metadata = {
        "obstacles": [
            {
                "name": "pillar_lower",
                "floor_id": 0,
                "position": [0.85, -1.55, 0.35],
                "half_size": [0.18, 0.18, 0.35],
            }
        ]
    }
    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
    report = {
        "cases": [
            {
                "route": "same_floor",
                "assets": {
                    "scene_xml": str(scene_xml),
                    "tomogram": str(tomogram),
                    "metadata": str(metadata_path),
                    "start": [-0.4, -2.1, 0.0],
                    "goal": [1.9, -1.25, 0.0],
                },
                "planning": [
                    {
                        "planner": "pct",
                        "backend_class": "_PCTBackend",
                        "native_backend_used": native_backend_used,
                        "native_runtime": {"ok": True},
                        "plan_ms": 12.3,
                        "goal": [1.9, -1.25, 0.0],
                        "path": [
                            [-0.2, -2.2, 0.6],
                            [0.4, -2.0, 0.6],
                            [1.9, -1.25, 0.0],
                        ],
                    }
                ],
            }
        ]
    }
    source = tmp_path / "report.json"
    source.write_text(json.dumps(report), encoding="utf-8")
    return source


def test_load_pct_route_requires_native_backend(tmp_path: Path) -> None:
    source = _write_source_report(tmp_path, native_backend_used=False)

    with pytest.raises(ValueError, match="native backend"):
        _load_pct_route(source, route="same_floor")


def test_load_pct_route_extracts_showcase_inputs(tmp_path: Path) -> None:
    source = _write_source_report(tmp_path)

    route = _load_pct_route(source, route="same_floor")

    assert route.route == "same_floor"
    assert route.scene_xml.exists()
    assert route.start == [-0.4, -2.1, 0.55]
    assert route.goal == [1.9, -1.25, 0.0]
    assert len(route.path) == 3
    assert route.plan["backend_class"] == "_PCTBackend"


def test_load_pct_route_rejects_known_unsafe_source_path(tmp_path: Path) -> None:
    source = _write_source_report(tmp_path)
    data = json.loads(source.read_text(encoding="utf-8"))
    data["cases"][0]["planning"][0]["path_safety"] = {
        "ok": False,
        "blocked_sample_count": 2,
    }
    source.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(ValueError, match="failed path_safety"):
        _load_pct_route(source, route="same_floor")


def test_native_node_commands_launch_only_local_planner_stack(tmp_path: Path) -> None:
    local_planner_cmd, path_follower_cmd = _native_node_commands(
        path_folder=tmp_path / "paths",
        max_speed=0.4,
        autonomy_speed=0.35,
        goal_clear_range=0.45,
        lookahead=0.55,
        stop_distance=0.30,
        obstacle_aware=True,
        check_rot_obstacle=False,
    )

    joined = " ".join(local_planner_cmd + path_follower_cmd)
    assert local_planner_cmd[:4] == ["ros2", "run", "local_planner", "localPlanner"]
    assert path_follower_cmd[:4] == ["ros2", "run", "local_planner", "pathFollower"]
    assert "autonomyMode:=true" in joined
    assert "checkObstacle:=true" in joined
    assert "checkRotObstacle:=false" in joined
    assert "Gateway" not in joined
    assert "driver" not in joined.lower()


def test_default_local_planner_path_folder_supports_merge_install(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    from sim.scripts import native_pct_mujoco_gate as mod

    merge_paths = tmp_path / "install/share/local_planner/paths"
    merge_paths.mkdir(parents=True)
    (merge_paths / "startPaths.ply").write_text("ply\n", encoding="utf-8")
    (merge_paths / "paths.ply").write_text("ply\n", encoding="utf-8")
    source_paths = tmp_path / "src/base_autonomy/local_planner/paths"
    source_paths.mkdir(parents=True)
    (source_paths / "startPaths.ply").write_text("ply\n", encoding="utf-8")
    (source_paths / "paths.ply").write_text("ply\n", encoding="utf-8")
    monkeypatch.setattr(mod, "ROOT", tmp_path)

    assert _default_local_planner_path_folder() == merge_paths


def test_ros_path_to_points_extracts_native_local_path() -> None:
    class Position:
        x = 1.25
        y = -2.5
        z = 0.6

    class Pose:
        position = Position()

    class PoseStamped:
        pose = Pose()

    class PathMsg:
        poses = [PoseStamped()]

    assert _ros_path_to_points(PathMsg()) == [[1.25, -2.5, 0.6]]


def test_path_frame_detection_treats_empty_native_path_as_robot_frame() -> None:
    assert _path_is_robot_frame("")
    assert _path_is_robot_frame("vehicle")
    assert not _path_is_robot_frame("map")
    assert not _path_is_robot_frame("/odom")


def test_path_window_by_distance_keeps_long_front_view() -> None:
    path = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [4.5, 0.0, 0.0]]

    window = _path_window_by_distance(path, start_idx=1, lookahead_m=2.2)

    assert window == path[1:]


def test_sample_lidar_points_pads_intensity_and_downsamples() -> None:
    class Engine:
        def get_lidar_points(self):
            return [[float(i), float(i + 1), 0.0] for i in range(10)]

    pts = _sample_lidar_points(Engine(), 4)

    assert pts.shape == (4, 4)
    assert pts[:, 3].tolist() == [1.0, 1.0, 1.0, 1.0]
    assert pts[:, 0].tolist() == [0.0, 3.0, 6.0, 9.0]


def test_moving_obstacle_boxes_support_density_and_phase_offsets() -> None:
    route = PctRoute(
        route="unit",
        source_report=Path("report.json"),
        scene_xml=Path("scene.xml"),
        start=[0.0, 0.0, 0.0],
        goal=[4.0, 0.0, 0.0],
        path=[[float(idx), 0.0, 0.0] for idx in range(5)],
        plan={},
        case={},
    )
    args = argparse.Namespace(
        moving_obstacle_mode="route_crossing",
        moving_obstacle_route_ratio=0.5,
        moving_obstacle_count=3,
        moving_obstacle_route_ratio_step=0.1,
        moving_obstacle_lateral_phase_step_rad=math.pi / 2.0,
        moving_obstacle_start_s=0.0,
        moving_obstacle_duration_s=10.0,
        moving_obstacle_period_s=8.0,
        moving_obstacle_lateral_amplitude_m=1.0,
        moving_obstacle_along_amplitude_m=0.0,
        moving_obstacle_radius_m=0.12,
        moving_obstacle_height_m=0.4,
    )

    boxes = _moving_obstacle_boxes(route, args, elapsed_s=2.0)

    assert len(boxes) == 3
    assert [box["name"] for box in boxes] == [
        "moving_crossing_obstacle_0",
        "moving_crossing_obstacle_1",
        "moving_crossing_obstacle_2",
    ]
    assert [box["half_size"] for box in boxes] == [[0.12, 0.12, 0.2]] * 3
    assert boxes[0]["position"][0] < boxes[1]["position"][0] < boxes[2]["position"][0]
    assert boxes[0]["position"][1] != boxes[1]["position"][1]


def test_omni_cart_tracker_uses_native_local_path_without_saturating_yaw() -> None:
    class State:
        position = [0.0, 0.0, 0.0]
        orientation = [0.0, 0.0, 0.0, 1.0]

    cmd, debug = _omni_cart_cmd(
        state=State(),
        local_path=[[0.0, 0.0, 0.0], [0.4, 0.05, 0.0], [1.2, 0.1, 0.0]],
        local_path_frame_id="body",
        fallback_target=[3.0, 0.0, 0.0],
        lookahead_m=1.0,
        min_speed=0.12,
        max_speed=0.35,
        max_lateral_speed=0.30,
        yaw_gain=0.45,
        max_yaw_rate=0.22,
        yaw_deadband=0.12,
    )

    assert debug["source"] == "omni_local_path_tracker"
    assert cmd.linear_x > 0.30
    assert abs(cmd.linear_y) < 0.05
    assert abs(cmd.angular_z) <= 0.22


def test_select_local_target_falls_back_to_global_waypoint() -> None:
    class State:
        position = [1.0, 2.0, 0.0]
        orientation = [0.0, 0.0, 0.0, 1.0]

    local, world = _select_local_target(
        state=State(),
        local_path=[],
        local_path_frame_id="",
        fallback_target=[3.0, 2.5, 0.0],
        lookahead_m=1.0,
    )

    assert local.tolist() == [2.0, 0.5]
    assert world.tolist() == [3.0, 2.5]


def test_trajectory_correctness_checks_progress_and_lateral_error() -> None:
    reference = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
    trail = [(0.0, 0.05), (0.8, 0.04), (1.6, -0.03), (2.4, 0.02), (3.0, 0.0)]

    report = _trajectory_correctness(
        trail=trail,
        reference_path=reference,
        max_p95_error_m=0.20,
        max_progress_regressions=0,
        min_route_progress_ratio=0.98,
    )

    assert report["ok"] is True
    assert report["final_progress_ratio"] == 1.0
    assert report["p95_lateral_error_m"] < 0.08


def test_trajectory_correctness_rejects_wrong_side_track() -> None:
    reference = [[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
    trail = [(0.0, 1.5), (1.0, 1.5), (2.0, 1.5), (3.0, 1.5)]

    report = _trajectory_correctness(
        trail=trail,
        reference_path=reference,
        max_p95_error_m=0.50,
        max_progress_regressions=0,
        min_route_progress_ratio=0.90,
    )

    assert report["ok"] is False
    assert report["p95_lateral_error_m"] > 1.0


def test_waypoint_safety_filter_skips_unsafe_points_without_auto_detour(tmp_path: Path) -> None:
    route = _load_pct_route(_write_source_report(tmp_path), route="same_floor")

    path, report = _build_safe_follow_path(
        route,
        enabled=True,
        clearance=0.40,
        extra_margin=0.18,
        sample_step_m=0.05,
    )

    assert report["enabled"] is True
    assert report["inserted_count"] == 0
    assert report["auto_detour_enabled"] is False
    assert report["skipped_unsafe_waypoint_count"] >= 1
    assert report["avoidance_clearance_m"] > report["clearance_m"]
    assert report["insertions"] == []
    assert len(path) < len(route.path)


def _route_with_center_obstacle(tmp_path: Path) -> PctRoute:
    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(
        json.dumps(
            {
                "obstacles": [
                    {
                        "name": "center_box",
                        "floor_id": 0,
                        "position": [1.0, 0.0, 0.35],
                        "half_size": [0.20, 0.20, 0.35],
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    return PctRoute(
        route="unit",
        source_report=tmp_path / "report.json",
        scene_xml=tmp_path / "scene.xml",
        start=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0, 0.0],
        path=[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        plan={},
        case={"assets": {"metadata": str(metadata_path)}},
    )


def test_local_path_obstacle_evidence_flags_path_into_obstacle(tmp_path: Path) -> None:
    class State:
        position = [0.0, 0.0, 0.0]
        orientation = [0.0, 0.0, 0.0, 1.0]

    report = _local_path_obstacle_evidence(
        local_path=[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        local_path_frame_id="body",
        state=State(),
        route=_route_with_center_obstacle(tmp_path),
        robot_radius=0.30,
        sample_step_m=0.05,
    )

    assert report["ok"] is False
    assert report["collision"] is True
    assert report["points_into_obstacle"] is True
    assert report["nearest_obstacle"] == "center_box"


def test_local_path_obstacle_evidence_accepts_detour(tmp_path: Path) -> None:
    class State:
        position = [0.0, 0.0, 0.0]
        orientation = [0.0, 0.0, 0.0, 1.0]

    report = _local_path_obstacle_evidence(
        local_path=[[0.0, 0.75, 0.0], [1.0, 0.75, 0.0], [2.0, 0.75, 0.0]],
        local_path_frame_id="body",
        state=State(),
        route=_route_with_center_obstacle(tmp_path),
        robot_radius=0.30,
        sample_step_m=0.05,
    )

    assert report["ok"] is True
    assert report["collision"] is False
    assert report["points_into_obstacle"] is False
    assert report["min_clearance_minus_robot_radius_m"] > 0.0


def test_local_path_obstacle_summary_requires_native_path_when_obstacle_aware() -> None:
    summary = _summarize_local_path_obstacle_evidence(
        samples=[],
        path_count=0,
        stop_samples=[],
        slow_down_samples=[],
        obstacle_aware=True,
    )

    assert summary["ok"] is False
    assert summary["reasons"] == ["local_path_missing"]
