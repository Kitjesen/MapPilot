from __future__ import annotations

import json
import struct
from pathlib import Path

import pytest

from sim.scripts.mujoco.saved_map_quality_gate import (
    DEFAULT_WORLD_XML,
    _apply_overlay_alignment_xy_m,
    _cell_centers_xy_m,
    evaluate_saved_map_quality,
    expected_obstacle_cells,
    read_pcd_xyz,
)


def _write_world(path: Path) -> None:
    path.write_text(
        """<mujoco model="quality_gate_test">
  <worldbody>
    <geom name="ground" type="box" pos="3 2 -0.05" size="3 2 0.05"/>
    <geom name="wall_test" type="box" pos="3 2 0.80" size="1.0 0.1 0.80"/>
    <geom name="roof_test" type="box" pos="3 2 3.2" size="3 2 0.04"/>
    <body name="robot_placeholder" pos="1 2 0.50">
      <freejoint name="robot_root"/>
      <geom name="chassis" type="box" size="0.4 0.3 0.1"/>
    </body>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )


def _write_ascii_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {len(points)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(points)}",
            "DATA ascii",
        ]
    )
    body = "\n".join(f"{x:.4f} {y:.4f} {z:.4f}" for x, y, z in points)
    path.write_text(header + "\n" + body + "\n", encoding="ascii")


def _points_from_cells(cells: set[tuple[int, int]], cell_m: float, repeats: int = 3) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for ix, iy in sorted(cells):
        x = (ix + 0.5) * cell_m
        y = (iy + 0.5) * cell_m
        for n in range(repeats):
            points.append((x + n * 0.005, y, 0.8))
    return points


def _transform_points(
    points: list[tuple[float, float, float]],
    *,
    yaw_deg: float,
    tx: float,
    ty: float,
) -> list[tuple[float, float, float]]:
    import math

    theta = math.radians(yaw_deg)
    c = math.cos(theta)
    s = math.sin(theta)
    out: list[tuple[float, float, float]] = []
    for x, y, z in points:
        out.append((c * x - s * y + tx, s * x + c * y + ty, z))
    return out


def test_quality_gate_accepts_cells_near_scene_obstacle(tmp_path: Path) -> None:
    world = tmp_path / "world.xml"
    pcd = tmp_path / "map.pcd"
    _write_world(world)
    expected, _ = expected_obstacle_cells(world, cell_m=0.2, z_min=0.3, z_max=1.6)
    _write_ascii_pcd(pcd, _points_from_cells(expected, 0.2))

    report, _, _ = evaluate_saved_map_quality(
        pcd_path=pcd,
        world_xml=world,
        min_candidate_cells=1,
    )

    assert report["ok"] is True
    assert report["remaining_gaps"] == []
    assert report["world"]["parsed_obstacle_geoms"] == 1
    assert report["scene_overlay"]["candidate_cells_within_near_distance_ratio"] == pytest.approx(1.0)


def test_quality_gate_allows_bounded_global_map_frame_alignment(tmp_path: Path) -> None:
    world = tmp_path / "world.xml"
    pcd = tmp_path / "map.pcd"
    _write_world(world)
    expected, _ = expected_obstacle_cells(world, cell_m=0.2, z_min=0.3, z_max=1.6)
    shifted_points = _transform_points(
        _points_from_cells(expected, 0.2),
        yaw_deg=6.0,
        tx=0.8,
        ty=-0.6,
    )
    _write_ascii_pcd(pcd, shifted_points)

    report, _, _ = evaluate_saved_map_quality(
        pcd_path=pcd,
        world_xml=world,
        min_candidate_cells=1,
    )

    assert report["ok"] is True
    assert report["quality_basis"] == "aligned_scene_overlay"
    assert report["scene_overlay"]["applied"] is True
    assert abs(report["scene_overlay"]["yaw_deg"]) > 0.0
    assert report["scene_overlay"]["candidate_cells_farther_than_far_distance_ratio"] < 0.15
    assert report["raw_scene_overlay"]["candidate_cells_within_near_distance_ratio"] < 1.0


def test_quality_gate_plot_points_use_aligned_overlay_metrics() -> None:
    xy = _cell_centers_xy_m({(0, 0)}, cell_m=1.0)

    aligned = _apply_overlay_alignment_xy_m(
        xy,
        {
            "applied": True,
            "yaw_deg": 90.0,
            "translation_xy_m": [1.0, 2.0],
        },
    )

    assert aligned.shape == (1, 2)
    assert aligned[0, 0] == pytest.approx(0.5)
    assert aligned[0, 1] == pytest.approx(2.5)


def test_quality_gate_rejects_far_ghost_cells(tmp_path: Path) -> None:
    world = tmp_path / "world.xml"
    pcd = tmp_path / "map.pcd"
    _write_world(world)
    expected, _ = expected_obstacle_cells(world, cell_m=0.2, z_min=0.3, z_max=1.6)
    ghost = {(ix + 80, iy + 40) for ix, iy in expected}
    _write_ascii_pcd(pcd, _points_from_cells(expected | ghost, 0.2))

    report, _, _ = evaluate_saved_map_quality(
        pcd_path=pcd,
        world_xml=world,
        min_candidate_cells=1,
    )

    assert report["ok"] is False
    assert any(gap.startswith("map_quality_obstacle_cells_not_near_scene") for gap in report["remaining_gaps"])
    assert any(gap.startswith("map_quality_far_ghost_cells") for gap in report["remaining_gaps"])
    assert report["scene_overlay"]["candidate_cells_farther_than_far_distance_ratio"] > 0.4


def test_quality_gate_reads_binary_pcd_xyz_fields(tmp_path: Path) -> None:
    pcd = tmp_path / "binary.pcd"
    points = [
        (1.0, 2.0, 3.0, 10.0, 0.0, 0.0, 1.0, 0.0),
        (4.0, 5.0, 6.0, 20.0, 0.0, 1.0, 0.0, 0.0),
    ]
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z intensity normal_x normal_y normal_z curvature",
            "SIZE 4 4 4 4 4 4 4 4",
            "TYPE F F F F F F F F",
            "COUNT 1 1 1 1 1 1 1 1",
            "WIDTH 2",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            "POINTS 2",
            "DATA binary",
        ]
    )
    pcd.write_bytes((header + "\n").encode("ascii") + b"".join(struct.pack("<ffffffff", *row) for row in points))

    xyz, meta = read_pcd_xyz(pcd)

    assert meta["data"] == "binary"
    assert meta["points_header"] == 2
    assert xyz.tolist() == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]


def test_industrial_park_expected_obstacles_match_product_scene() -> None:
    cells, meta = expected_obstacle_cells(DEFAULT_WORLD_XML, cell_m=0.2, z_min=0.3, z_max=1.6)

    assert meta["parsed_obstacle_geoms"] >= 40
    assert meta["start_xy_scene_m"] == [3.0, 4.0]
    assert len(cells) > 800
    json.dumps(meta)
