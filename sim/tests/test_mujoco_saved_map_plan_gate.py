from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

from sim.scripts.mujoco import saved_map_plan_gate
from sim.scripts.mujoco.saved_map_plan_gate import (
    BUILDING_DEFAULT_GOAL,
    BUILDING_DEFAULT_START,
    INDUSTRIAL_PARK_DEFAULT_GOAL,
    INDUSTRIAL_PARK_DEFAULT_START,
    MULTILEVEL_SCENE_PRESETS,
    STAIRS3D_BODY_REFERENCE_HEIGHT_M,
    STAIRS3D_BODY_SUPPORT_DEPTH_M,
    STAIRS3D_DEFAULT_GOAL,
    STAIRS3D_DEFAULT_START,
    STAIRS3D_MAX_SLOPE,
    STAIRS3D_RISE_M,
    STAIRS3D_STEP_COUNT,
    STAIRS3D_STEP_MAX_M,
    STAIRS3D_TREAD_M,
    _stairs3d_geoms,
    build_parser,
    building_scene_clip,
    collect_mujoco_lidar_points,
    effective_free_layers_above,
    effective_start_goal,
    effective_support_dilation_cells,
    filter_points_to_scene,
    generate_building_points,
    generate_industrial_park_points,
    generate_points,
    generate_scene_xml,
    multifloor_mapping_trajectory,
    planner_constraint_overrides,
    run_gate,
    scene_acceptance,
    stair_scene_default_start_goal,
    stair_scene_profile,
    write_ascii_pcd,
)


def test_generate_points_writes_ascii_pcd(tmp_path: Path) -> None:
    points = generate_points(length=1.0, width=1.0, spacing=0.5)
    pcd = tmp_path / "map.pcd"

    write_ascii_pcd(pcd, points)

    text = pcd.read_text(encoding="ascii")
    assert "FIELDS x y z" in text
    assert f"POINTS {len(points)}" in text
    assert "DATA ascii" in text
    assert len(points) > 0


def test_stairs3d_steps_are_quadruped_reasonable() -> None:
    geoms = _stairs3d_geoms()
    up = [g for g in geoms if g["name"].startswith("step_up_")]
    down = [g for g in geoms if g["name"].startswith("step_down_")]

    assert len(up) == STAIRS3D_STEP_COUNT
    assert len(down) == STAIRS3D_STEP_COUNT
    up_tops = [round(float(g["pos"][2]) + float(g["size"][2]), 3) for g in up]
    down_tops = [round(float(g["pos"][2]) + float(g["size"][2]), 3) for g in down]
    assert up_tops == [round(STAIRS3D_RISE_M * i, 3) for i in range(1, STAIRS3D_STEP_COUNT + 1)]
    assert down_tops == [round(STAIRS3D_RISE_M * (STAIRS3D_STEP_COUNT - i), 3) for i in range(0, STAIRS3D_STEP_COUNT)]
    assert max(b - a for a, b in zip([0.0, *up_tops[:-1]], up_tops)) <= STAIRS3D_STEP_MAX_M
    assert all(round(float(g["size"][0]) * 2.0, 3) == STAIRS3D_TREAD_M for g in [*up, *down])
    assert STAIRS3D_DEFAULT_START == [0.75, 0.0, 0.55]
    assert [round(v, 3) for v in STAIRS3D_DEFAULT_GOAL] == [7.10, 0.0, 1.75]


def test_goal_marker_is_visual_only_and_excluded_from_lidar() -> None:
    xml = saved_map_plan_gate._stairs3d_scene_xml("multifloor_stack_3")

    assert 'name="goal_marker"' in xml
    assert 'rgba="0.1 0.32 1 0.75" group="5"' in xml


def test_multifloor_stair_bridges_overlap_floor_by_robot_edge_clearance() -> None:
    geoms = saved_map_plan_gate._multifloor_stack_geoms()
    bridge = next(g for g in geoms if g["name"] == "floor_stair_bridge_bottom_l1")
    bridge_y_min = float(bridge["pos"][1]) - float(bridge["size"][1])
    floor_edge_y = 1.75

    assert bridge_y_min <= floor_edge_y - saved_map_plan_gate.STAIRS3D_EDGE_CLEARANCE_M
    posts = [g for g in geoms if str(g["name"]).startswith("rail_stair_") and str(g["name"]).endswith("_post")]
    handrails = [g for g in geoms if str(g["name"]).endswith("_handrail")]
    assert len(posts) == 64
    assert len(handrails) == 64
    assert all(math.isclose(float(g["size"][2]) * 2.0, 1.08) for g in posts)


def test_multifloor_transfer_platform_starts_after_the_stair_run() -> None:
    geoms = saved_map_plan_gate._multifloor_stack_geoms()
    transfer = next(g for g in geoms if g["name"] == "floor_stair_transfer_l1_to_l2")
    profile = stair_scene_profile("multifloor_stack_3")
    run_end_x = (
        3.90
        - 2.15
        + int(round(float(profile["level_height_m"]) / float(profile["rise_m"]))) * float(profile["tread_m"])
        + float(profile["mid_landing_length_m"])
    )
    transfer_min_x = float(transfer["pos"][0]) - float(transfer["size"][0])

    assert transfer_min_x >= run_end_x - 1e-9


def test_multifloor_mapping_trajectory_covers_every_level_and_stair_center() -> None:
    trajectory = multifloor_mapping_trajectory("multifloor_stack_3")

    assert len(trajectory) >= 80
    assert min(point[2] for point in trajectory) <= 0.55
    assert max(point[2] for point in trajectory) >= 4.15

    climbing_segments = []
    for first, second in zip(trajectory, trajectory[1:]):
        dx = float(second[0]) - float(first[0])
        dy = float(second[1]) - float(first[1])
        dz = float(second[2]) - float(first[2])
        if abs(dz) <= 1e-9:
            continue
        dxy = math.hypot(dx, dy)
        assert dxy > 0.0
        assert abs(dz) / dxy <= STAIRS3D_MAX_SLOPE + 1e-6
        climbing_segments.append((first, second))

    assert climbing_segments
    climbing_y = [0.5 * (a[1] + b[1]) for a, b in climbing_segments]
    assert any(abs(value - 3.94) < 0.15 for value in climbing_y)
    assert any(abs(value - 5.49) < 0.15 for value in climbing_y)


def test_saved_map_gate_uses_current_octoplanner3d_map_path_contract() -> None:
    source = Path(saved_map_plan_gate.__file__).read_text(encoding="utf-8")

    assert 'map_path=str(map_dir / "octomap.ot")' in source
    assert "tomogram_path=" not in source


def test_stairs3d_defaults_to_23cm_planner_step_limit() -> None:
    args = argparse.Namespace(
        scene_preset="stairs3d",
        planner_no_ground_support=False,
        planner_clearance_cells=-1,
        planner_preblocked_cells=-1,
        planner_robot_radius=0.0,
        planner_goal_tolerance_m=0.0,
        planner_goal_xy_tolerance_m=0.0,
        planner_goal_z_tolerance_m=0.0,
        planner_max_step_height_m=0.0,
        map_source="synthetic_hits",
        resolution=0.18,
    )

    overrides = planner_constraint_overrides(args)

    assert overrides["max_step_height"] == STAIRS3D_STEP_MAX_M
    assert overrides["max_slope"] == STAIRS3D_MAX_SLOPE
    assert overrides["strict_direct_ground_support"] is False
    assert overrides["ground_support_xy_radius_cells"] == 1
    assert overrides["ground_support_depth_cells"] == math.ceil(STAIRS3D_BODY_SUPPORT_DEPTH_M / 0.18)
    assert math.isclose(overrides["support_height_m"], STAIRS3D_BODY_REFERENCE_HEIGHT_M)
    assert math.isclose(overrides["support_height_tolerance_m"], 0.099)
    assert overrides["support_patch_radius_cells"] == 0
    assert overrides["support_patch_min_samples"] == 0
    assert math.isclose(overrides["body_clearance_below_m"], 0.18)
    assert math.isclose(overrides["body_clearance_above_m"], 0.30)
    assert overrides["lowest_traversable_only"] is False


def test_live_multifloor_map_scales_voxel_layers_from_metric_robot_contract() -> None:
    args = argparse.Namespace(
        scene_preset="multifloor_stack_3",
        map_source="mujoco_lidar",
        resolution=0.09,
        support_dilation_cells=-1,
        free_layers_above=-1,
    )

    assert effective_support_dilation_cells(args, args.scene_preset) == 2
    assert effective_free_layers_above(args, args.scene_preset) == math.ceil(0.55 / 0.09)


def test_named_multilevel_scene_profiles_have_explicit_step_semantics() -> None:
    expected = {
        "stair_easy": (0.15, False),
        "stair_limit_23cm": (0.23, False),
        "stair_blocked_30cm": (0.30, True),
        "multifloor_two_connectors": (0.20, False),
        "multifloor_stack_3": (0.113, False),
        "rough_terrain_traversability": (0.18, False),
    }

    for preset, (rise, blocked) in expected.items():
        profile = stair_scene_profile(preset)
        assert round(float(profile["rise_m"]), 3) == rise
        assert bool(profile["blocked"]) is blocked
        start, goal = stair_scene_default_start_goal(preset)
        assert len(start) == 3
        assert goal[2] > start[2]


def test_scene_parser_exposes_planning_acceptance_suite() -> None:
    parser = build_parser()
    scene_action = next(action for action in parser._actions if action.dest == "scene_preset")

    assert set(MULTILEVEL_SCENE_PRESETS).issubset(set(scene_action.choices))


def test_multifloor_stack_scene_has_three_levels_and_side_connectors() -> None:
    geoms = _stairs3d_geoms("multifloor_stack_3")
    floors = [g for g in geoms if g["name"].startswith("floor_level_")]
    step_treads = [g for g in geoms if g["name"].startswith("step_stack_")]
    risers = [g for g in geoms if g["name"].startswith("riser_stack_")]
    stair_handrails = [g for g in geoms if g["name"].endswith("_handrail")]
    stair_posts = [g for g in geoms if g["name"].endswith("_post")]
    start, goal = stair_scene_default_start_goal("multifloor_stack_3")

    assert len(floors) == 3
    assert len(step_treads) >= 20
    assert len(risers) == len(step_treads)
    assert len(stair_handrails) == len(step_treads) * 2
    assert len(stair_posts) == len(step_treads) * 2
    assert all(float(g["size"][2]) <= 0.03 for g in step_treads)
    assert all(float(g["size"][2]) > 0.05 for g in risers)
    assert all(float(g["size"][2]) <= 0.04 for g in stair_handrails)
    assert all(math.isclose(float(g["size"][2]), 0.54) for g in stair_posts)
    assert start == [7.00, -1.20, 0.55]
    assert goal[2] > 3.0


def test_blocked_stair_scene_rejects_unexpected_goal_reach() -> None:
    acceptance = scene_acceptance(
        "stair_blocked_30cm",
        {"ok": True, "reached_goal": True, "path": [[0.0, 0.0, 0.0], [1.0, 0.0, 1.0]]},
    )

    assert acceptance["ok"] is False
    assert "blocked_scene_unexpectedly_reached_goal" in acceptance["blockers"]


def test_multilevel_scene_rejects_vertical_air_climb_path() -> None:
    acceptance = scene_acceptance(
        "multifloor_stack_3",
        {"ok": True, "reached_goal": True, "path": [[1.0, 1.0, 0.5], [1.0, 1.0, 0.68]]},
    )

    assert acceptance["ok"] is False
    assert "path_has_vertical_floor_transition" in acceptance["blockers"]


def test_multilevel_scene_rejects_too_steep_path_segment() -> None:
    acceptance = scene_acceptance(
        "multifloor_stack_3",
        {"ok": True, "reached_goal": True, "path": [[1.0, 1.0, 0.5], [1.18, 1.0, 0.68]]},
    )

    assert acceptance["ok"] is False
    assert "path_exceeds_stair_slope" in acceptance["blockers"]


def _valid_three_floor_stair_path() -> list[list[float]]:
    profile = stair_scene_profile("multifloor_stack_3")
    rise = float(profile["rise_m"])
    tread = float(profile["tread_m"])
    steps = int(round(float(profile["level_height_m"]) / rise))
    landing_after = int(profile["mid_landing_after_steps"])
    landing_length = float(profile["mid_landing_length_m"])

    def progress(index: int) -> float:
        return index * tread + (landing_length if index > landing_after else 0.0)

    path = [[7.0, -1.2, 0.55], [1.75, 3.94, 0.55]]
    for index in range(1, steps + 1):
        if index == landing_after + 1:
            path.append(
                [
                    1.75 + landing_after * tread + landing_length,
                    3.94,
                    0.55 + landing_after * rise,
                ]
            )
        path.append([1.75 + progress(index), 3.94, 0.55 + index * rise])
    second_start = 1.75 + steps * tread + landing_length
    path.append([second_start, 5.49, 2.35])
    for index in range(1, steps + 1):
        if index == landing_after + 1:
            path.append(
                [
                    second_start - landing_after * tread - landing_length,
                    5.49,
                    2.35 + landing_after * rise,
                ]
            )
        path.append([second_start - progress(index), 5.49, 2.35 + index * rise])
    path.append([7.4, 2.8, 4.15])
    return path


def test_multifloor_scene_accepts_two_complete_opposed_stair_runs() -> None:
    acceptance = scene_acceptance(
        "multifloor_stack_3",
        {"ok": True, "reached_goal": True, "path": _valid_three_floor_stair_path()},
    )

    assert acceptance["ok"] is True
    assert acceptance["topology"]["ok"] is True
    assert [item["climbing_segments"] for item in acceptance["topology"]["stairs"]] == [16, 16]


def test_multifloor_scene_tolerates_one_voxel_floor_height_jitter() -> None:
    path = _valid_three_floor_stair_path()
    path.insert(1, [6.7, -1.2, 0.64])

    acceptance = scene_acceptance(
        "multifloor_stack_3",
        {"ok": True, "reached_goal": True, "path": path},
    )

    assert acceptance["ok"] is True


def test_multifloor_scene_rejects_second_stair_climbing_in_wrong_direction() -> None:
    path = _valid_three_floor_stair_path()
    profile = stair_scene_profile("multifloor_stack_3")
    rise = float(profile["rise_m"])
    tread = float(profile["tread_m"])
    landing_after = int(profile["mid_landing_after_steps"])
    landing_length = float(profile["mid_landing_length_m"])
    for point in path:
        if abs(float(point[1]) - 5.49) > 1e-6 or float(point[2]) <= 2.35:
            continue
        step_index = int(round((float(point[2]) - 2.35) / rise))
        point[0] = 1.75 + step_index * tread
        if step_index > landing_after:
            point[0] += landing_length

    acceptance = scene_acceptance(
        "multifloor_stack_3",
        {"ok": True, "reached_goal": True, "path": path},
    )

    assert acceptance["ok"] is False
    assert "stair_2_climbs_against_expected_direction" in acceptance["blockers"]


def test_multifloor_scene_rejects_major_climb_off_both_stairs() -> None:
    path = _valid_three_floor_stair_path()
    path.insert(1, [6.58, -1.2, 0.73])

    acceptance = scene_acceptance(
        "multifloor_stack_3",
        {"ok": True, "reached_goal": True, "path": path},
    )

    assert acceptance["ok"] is False
    assert "major_climb_outside_stair_corridor" in acceptance["blockers"]


def test_filter_points_to_scene_keeps_only_generated_world_bounds() -> None:
    points = [
        (0.0, 0.0, 0.0),
        (1.5, 0.9, 0.8),
        (4.0, 0.0, 0.0),
        (0.0, 2.0, 0.0),
        (0.0, 0.0, 2.0),
    ]

    kept = filter_points_to_scene(points, length=2.0, width=2.0, margin=0.5)

    assert kept == [(0.0, 0.0, 0.0), (1.5, 0.9, 0.8)]


def test_generate_corridor_scene_floor_covers_start_margin(tmp_path: Path) -> None:
    scene = tmp_path / "scene.xml"

    generate_scene_xml(scene, length=1.4, width=1.8, scene_preset="corridor")

    text = scene.read_text(encoding="utf-8")
    assert 'name="floor"' in text
    assert 'pos="0.700 0 -0.025"' in text
    assert 'size="1.700 0.900 0.025"' in text


def test_generate_building_points_contains_two_floors_and_stairs() -> None:
    points = generate_building_points(spacing=0.5, hits_per_cell=1)

    assert len(points) > 1000
    assert any(abs(z - 0.0) < 0.05 for _, _, z in points)
    assert any(abs(z - 3.5) < 0.05 for _, _, z in points)
    assert any(0.5 < z < 3.0 for x, y, z in points if 15.0 <= x <= 17.5 and 0.8 <= y <= 5.0)


def test_industrial_park_preset_generates_full_route_support_without_visual_markers(
    tmp_path: Path,
) -> None:
    parser = build_parser()
    scene_action = next(action for action in parser._actions if action.dest == "scene_preset")
    assert "industrial_park" in scene_action.choices

    scene = tmp_path / "industrial_park.xml"
    generate_scene_xml(scene, length=60.0, width=40.0, scene_preset="industrial_park")
    assert scene.read_bytes() == saved_map_plan_gate.INDUSTRIAL_PARK_SCENE_XML.read_bytes()

    points = generate_industrial_park_points(spacing=0.5, hits_per_cell=2)
    assert len(points) > 20_000
    assert any(
        abs(x - INDUSTRIAL_PARK_DEFAULT_GOAL[0]) <= 0.26
        and abs(y - INDUSTRIAL_PARK_DEFAULT_GOAL[1]) <= 0.26
        and abs(z) <= 0.02
        for x, y, z in points
    )
    start, goal = effective_start_goal(
        argparse.Namespace(
            scene_preset="industrial_park",
            start=list(saved_map_plan_gate.CORRIDOR_DEFAULT_START),
            goal=list(saved_map_plan_gate.CORRIDOR_DEFAULT_GOAL),
        )
    )
    assert start == INDUSTRIAL_PARK_DEFAULT_START
    assert goal == INDUSTRIAL_PARK_DEFAULT_GOAL


def test_building_scene_clip_keeps_upper_floor_points() -> None:
    clip = building_scene_clip()
    kept = filter_points_to_scene(
        [(18.0, 11.0, 3.5), (18.0, 11.0, 7.8), (30.0, 11.0, 3.5)],
        length=3.0,
        width=1.8,
        **clip,
    )

    assert kept == [(18.0, 11.0, 3.5)]


def test_planner_constraint_overrides_are_explicit() -> None:
    args = argparse.Namespace(
        planner_no_ground_support=True,
        planner_clearance_cells=0,
        planner_preblocked_cells=0,
        planner_robot_radius=0.1,
        planner_goal_tolerance_m=0.0,
        planner_goal_xy_tolerance_m=0.0,
        planner_goal_z_tolerance_m=0.0,
    )

    overrides = planner_constraint_overrides(args)

    assert overrides == {
        "require_ground_support": False,
        "obstacle_clearance_radius_cells": 0,
        "obstacle_clearance_weight": 0.0,
        "preblocked_costmap_radius_cells": 0,
        "enable_preblocked_costmap": False,
        "robot_radius": 0.1,
    }


def test_mujoco_lidar_map_source_defaults_to_mid360_product_backend() -> None:
    args = build_parser().parse_args([])

    assert args.lidar_backend == "mujoco_lidar"
    assert args.mujoco_lidar_backend == "cpu"
    assert args.allow_legacy_lidar_fallback is False
    assert args.mid360_pattern.name == "mid360.npy"
    assert args.mid360_samples_per_frame == 15000


def test_mujoco_lidar_collection_uses_product_engine(monkeypatch, tmp_path: Path) -> None:
    from drivers.sim.mujoco import runtime as mujoco_runtime

    calls = {}

    class FakeEngine:
        def __init__(self) -> None:
            self.sim_time = 0.0
            self.closed = False

        def get_lidar_backend_report(self) -> dict:
            return {
                "backend": "mujoco_lidar",
                "product_backend": True,
                "product_lidar_backend_verified": True,
                "fallback_used": False,
            }

        def step(self, cmd) -> SimpleNamespace:
            self.sim_time += 0.1
            return SimpleNamespace(position=[self.sim_time, 0.0, 0.55])

        def get_lidar_points(self) -> list[tuple[float, float, float, float]]:
            return [(self.sim_time, 0.0, 0.2, 120.0)]

        def close(self) -> None:
            self.closed = True

    def fake_build_engine(**kwargs):
        calls.update(kwargs)
        return FakeEngine()

    monkeypatch.setattr(mujoco_runtime, "build_engine", fake_build_engine)

    points, report = collect_mujoco_lidar_points(
        tmp_path / "scene.xml",
        scans=2,
        duration_s=0.0,
        timeout_s=1.0,
        vx=0.1,
        wz=0.0,
        publish_hz=10.0,
        n_rays=6400,
        mid360_pattern=tmp_path / "mid360.npy",
        mid360_samples_per_frame=15000,
        lidar_backend="mujoco_lidar",
        mujoco_lidar_backend="cpu",
        allow_legacy_lidar_fallback=False,
        mujoco_memory="64M",
        trajectory_support_radius_m=0.0,
    )

    assert len(points) == 2
    assert report["ok"] is True
    assert report["source"] == "build_engine.get_lidar_points"
    assert report["lidar_backend"]["backend"] == "mujoco_lidar"
    assert calls["lidar_backend"] == "mujoco_lidar"
    assert calls["mujoco_lidar_backend"] == "cpu"
    assert calls["require_product_lidar_backend"] is True
    assert calls["allow_legacy_lidar_fallback"] is False


def test_mujoco_lidar_collection_can_add_trajectory_support(monkeypatch, tmp_path: Path) -> None:
    from drivers.sim.mujoco import runtime as mujoco_runtime

    class FakeEngine:
        def __init__(self) -> None:
            self.sim_time = 0.0

        def get_lidar_backend_report(self) -> dict:
            return {"backend": "mujoco_lidar", "product_backend": True}

        def step(self, cmd) -> SimpleNamespace:
            self.sim_time += 0.1
            return SimpleNamespace(position=[1.0 + self.sim_time, 2.0, 0.55])

        def get_lidar_points(self) -> list[tuple[float, float, float, float]]:
            return [(1.0 + self.sim_time, 2.0, 0.2, 120.0)]

        def close(self) -> None:
            pass

    monkeypatch.setattr(mujoco_runtime, "build_engine", lambda **kwargs: FakeEngine())

    points, report = collect_mujoco_lidar_points(
        tmp_path / "scene.xml",
        scans=2,
        duration_s=0.0,
        timeout_s=1.0,
        vx=0.1,
        wz=0.0,
        publish_hz=10.0,
        n_rays=6400,
        mid360_pattern=tmp_path / "mid360.npy",
        mid360_samples_per_frame=15000,
        lidar_backend="mujoco_lidar",
        mujoco_lidar_backend="cpu",
        allow_legacy_lidar_fallback=False,
        mujoco_memory="64M",
        trajectory_support_radius_m=0.2,
        trajectory_support_spacing_m=0.1,
    )

    assert report["lidar_point_count"] == 2
    assert report["trajectory_support_point_count"] > 0
    assert len(points) == report["point_count"]
    assert report["point_count"] > report["lidar_point_count"]


def test_mujoco_lidar_trajectory_support_is_opt_in() -> None:
    args = build_parser().parse_args([])

    assert args.trajectory_support_radius_m == 0.0


def test_gate_reports_missing_converter(tmp_path: Path) -> None:
    args = argparse.Namespace(
        out_dir=str(tmp_path / "gate"),
        scene_preset="corridor",
        length=1.0,
        width=1.0,
        spacing=0.5,
        hits_per_cell=4,
        map_source="synthetic_hits",
        lidar_scans=1,
        lidar_duration=0.0,
        lidar_timeout=1.0,
        lidar_vx=0.0,
        lidar_wz=0.0,
        resolution=0.2,
        converter="",
        no_env_converter=True,
        converter_timeout=5.0,
        planner_executable="",
        planner_timeout=1.0,
        planner_no_ground_support=False,
        planner_clearance_cells=-1,
        planner_preblocked_cells=-1,
        planner_robot_radius=0.0,
        planner_goal_tolerance_m=0.0,
        planner_goal_xy_tolerance_m=0.0,
        planner_goal_z_tolerance_m=0.0,
        start=[0.0, 0.0, 0.0],
        goal=[0.5, 0.0, 0.0],
        skip_plan=True,
    )

    report = run_gate(args)

    assert report["ok"] is False
    assert report["build"]["status"] == "missing_converter"
    assert (tmp_path / "gate" / "same_source_map" / "map.pcd").is_file()


def test_gate_builds_artifact_with_fake_converter_when_plan_skipped(tmp_path: Path) -> None:
    fake = tmp_path / "fake_converter.py"
    fake.write_text(
        """
import argparse
p = argparse.ArgumentParser()
p.add_argument("--input")
p.add_argument("--output")
p.add_argument("--resolution")
p.add_argument("--support-dilation-cells")
p.add_argument("--free-layers-above")
p.add_argument("--free-dilation-cells")
p.add_argument("--frame")
a = p.parse_args()
open(a.output, "wb").write(b"fake octomap")
""".strip(),
        encoding="utf-8",
    )
    args = argparse.Namespace(
        out_dir=str(tmp_path / "gate"),
        scene_preset="corridor",
        length=1.0,
        width=1.0,
        spacing=0.5,
        hits_per_cell=4,
        map_source="synthetic_hits",
        lidar_scans=1,
        lidar_duration=0.0,
        lidar_timeout=1.0,
        lidar_vx=0.0,
        lidar_wz=0.0,
        resolution=0.2,
        converter=[sys.executable, str(fake)],
        no_env_converter=True,
        converter_timeout=5.0,
        planner_executable="",
        planner_timeout=1.0,
        planner_no_ground_support=False,
        planner_clearance_cells=-1,
        planner_preblocked_cells=-1,
        planner_robot_radius=0.0,
        planner_goal_tolerance_m=0.0,
        planner_goal_xy_tolerance_m=0.0,
        planner_goal_z_tolerance_m=0.0,
        start=[0.0, 0.0, 0.0],
        goal=[0.5, 0.0, 0.0],
        skip_plan=True,
    )

    report = run_gate(args)

    assert report["ok"] is True
    assert report["build"]["status"] == "built"
    assert report["artifact_gate"]["ok"] is True
    assert report["active_map"]["success"] is True
    assert (tmp_path / "gate" / "active_map.txt").read_text(encoding="utf-8").strip() == "same_source_map"
    metadata = json.loads((tmp_path / "gate" / "same_source_map" / "metadata.json").read_text())
    assert metadata["data_source"] == "mujoco"


def test_building_gate_builds_multifloor_artifact_with_fake_converter(tmp_path: Path) -> None:
    fake = tmp_path / "fake_converter.py"
    fake.write_text(
        """
import argparse
p = argparse.ArgumentParser()
p.add_argument("--input")
p.add_argument("--output")
p.add_argument("--resolution")
p.add_argument("--support-dilation-cells")
p.add_argument("--free-layers-above")
p.add_argument("--free-dilation-cells")
p.add_argument("--frame")
a = p.parse_args()
open(a.output, "wb").write(b"fake octomap")
""".strip(),
        encoding="utf-8",
    )
    args = argparse.Namespace(
        out_dir=str(tmp_path / "building_gate"),
        scene_preset="building",
        length=3.0,
        width=1.8,
        spacing=0.5,
        hits_per_cell=1,
        map_source="synthetic_hits",
        lidar_scans=1,
        lidar_duration=0.0,
        lidar_timeout=1.0,
        lidar_vx=0.0,
        lidar_wz=0.0,
        resolution=0.2,
        converter=[sys.executable, str(fake)],
        no_env_converter=True,
        converter_timeout=5.0,
        planner_executable="",
        planner_timeout=1.0,
        planner_no_ground_support=False,
        planner_clearance_cells=-1,
        planner_preblocked_cells=-1,
        planner_robot_radius=0.0,
        planner_goal_tolerance_m=0.0,
        planner_goal_xy_tolerance_m=0.0,
        planner_goal_z_tolerance_m=0.0,
        start=[0.0, 0.0, 0.0],
        goal=[2.4, 0.0, 0.0],
        skip_plan=True,
    )

    report = run_gate(args)

    assert report["ok"] is True
    assert report["scene_preset"] == "building"
    assert report["build"]["status"] == "built"
    assert report["artifact_gate"]["ok"] is True
    assert report["start"] == BUILDING_DEFAULT_START
    assert report["goal"] == BUILDING_DEFAULT_GOAL
    text = (tmp_path / "building_gate" / "same_source_map" / "map.pcd").read_text(encoding="ascii")
    assert " 3.500000" in text
    metadata = json.loads((tmp_path / "building_gate" / "same_source_map" / "metadata.json").read_text())
    assert metadata["source_profile"] == "mujoco_building_saved_map_gate"


def test_building_default_start_goal_are_explicit() -> None:
    args = argparse.Namespace(
        scene_preset="building",
        start=[0.0, 0.0, 0.0],
        goal=[2.4, 0.0, 0.0],
    )

    assert effective_start_goal(args) == (BUILDING_DEFAULT_START, BUILDING_DEFAULT_GOAL)
