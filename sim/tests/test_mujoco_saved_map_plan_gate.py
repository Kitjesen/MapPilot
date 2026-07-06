from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

from sim.scripts.mujoco.saved_map_plan_gate import (
    BUILDING_DEFAULT_GOAL,
    BUILDING_DEFAULT_START,
    building_scene_clip,
    build_parser,
    collect_mujoco_lidar_points,
    effective_start_goal,
    filter_points_to_scene,
    generate_building_points,
    generate_points,
    planner_constraint_overrides,
    run_gate,
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


def test_generate_building_points_contains_two_floors_and_stairs() -> None:
    points = generate_building_points(spacing=0.5, hits_per_cell=1)

    assert len(points) > 1000
    assert any(abs(z - 0.0) < 0.05 for _, _, z in points)
    assert any(abs(z - 3.5) < 0.05 for _, _, z in points)
    assert any(0.5 < z < 3.0 for x, y, z in points if 15.0 <= x <= 17.5 and 0.8 <= y <= 5.0)


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
    )

    overrides = planner_constraint_overrides(args)

    assert overrides == {
        "require_ground_support": False,
        "obstacle_clearance_radius_cells": 0,
        "preblocked_costmap_radius_cells": 0,
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
    )

    assert len(points) == 2
    assert report["ok"] is True
    assert report["source"] == "build_engine.get_lidar_points"
    assert report["lidar_backend"]["backend"] == "mujoco_lidar"
    assert calls["lidar_backend"] == "mujoco_lidar"
    assert calls["mujoco_lidar_backend"] == "cpu"
    assert calls["require_product_lidar_backend"] is True
    assert calls["allow_legacy_lidar_fallback"] is False


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
        start=[0.0, 0.0, 0.0],
        goal=[0.5, 0.0, 0.0],
        skip_plan=True,
    )

    report = run_gate(args)

    assert report["ok"] is True
    assert report["build"]["status"] == "built"
    assert report["artifact_gate"]["ok"] is True
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
