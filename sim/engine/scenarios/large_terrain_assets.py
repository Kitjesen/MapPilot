"""Deterministic large-terrain assets for navigation validation."""

from __future__ import annotations

import json
import pickle
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from maps.artifacts import build_saved_map_metadata, sha256_file


@dataclass(frozen=True)
class TerrainBox:
    name: str
    position: tuple[float, float, float]
    half_size: tuple[float, float, float]
    rgba: tuple[float, float, float, float]
    kind: str = "obstacle"


@dataclass(frozen=True)
class TerrainZone:
    name: str
    center: tuple[float, float]
    half_size: tuple[float, float]
    cost: float
    rgba: tuple[float, float, float, float]
    kind: str


@dataclass(frozen=True)
class TerrainRoute:
    name: str
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    description: str
    min_routed_distance_m: float


@dataclass(frozen=True)
class LargeTerrainAssets:
    scene_xml: Path
    tomogram: Path
    map_pcd: Path
    metadata: Path
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    routes: tuple[TerrainRoute, ...]


DEFAULT_START = (-9.5, -5.6, 0.0)
DEFAULT_GOAL = (9.4, 5.4, 0.0)
DEFAULT_RESOLUTION = 0.2
DEFAULT_ORIGIN = (-12.0, -8.0)
DEFAULT_SHAPE_XY = (121, 81)  # x cells, y cells; builder_xy convention.
DEFAULT_SLICE_DH = 0.5


def large_terrain_localization_landmarks() -> list[TerrainBox]:
    """Return planner-visible vertical geometry that improves LiDAR observability."""

    return [
        TerrainBox(
            name="localization_start_south_panel",
            position=(-9.40, -6.65, 0.90),
            half_size=(0.75, 0.08, 0.90),
            rgba=(0.74, 0.70, 0.30, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_start_west_panel",
            position=(-10.55, -5.25, 0.90),
            half_size=(0.08, 0.70, 0.90),
            rgba=(0.74, 0.70, 0.30, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_start_north_panel",
            position=(-8.65, -4.20, 0.90),
            half_size=(0.52, 0.08, 0.90),
            rgba=(0.74, 0.70, 0.30, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_start_south_plinth",
            position=(-8.00, -6.15, 0.42),
            half_size=(0.38, 0.24, 0.42),
            rgba=(0.62, 0.67, 0.40, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_start_west_plinth",
            position=(-10.15, -4.45, 0.50),
            half_size=(0.32, 0.22, 0.50),
            rgba=(0.62, 0.67, 0.40, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_start_corner_plinth",
            position=(-10.30, -6.35, 0.58),
            half_size=(0.30, 0.30, 0.58),
            rgba=(0.62, 0.67, 0.40, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_lane_north_panel",
            position=(-7.85, -3.70, 0.90),
            half_size=(0.70, 0.08, 0.90),
            rgba=(0.74, 0.70, 0.30, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_lane_south_plinth",
            position=(-7.15, -7.15, 0.55),
            half_size=(0.34, 0.24, 0.55),
            rgba=(0.62, 0.67, 0.40, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_lane_east_panel",
            position=(-6.30, -6.40, 0.90),
            half_size=(0.08, 0.60, 0.90),
            rgba=(0.74, 0.70, 0.30, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_lane_north_plinth",
            position=(-6.80, -3.95, 0.60),
            half_size=(0.30, 0.28, 0.60),
            rgba=(0.62, 0.67, 0.40, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_midfield_south_panel",
            position=(-3.20, -6.55, 0.90),
            half_size=(0.70, 0.08, 0.90),
            rgba=(0.74, 0.70, 0.30, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_midfield_north_panel",
            position=(-3.80, 6.30, 0.90),
            half_size=(0.70, 0.08, 0.90),
            rgba=(0.74, 0.70, 0.30, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_gate_south_panel",
            position=(-1.75, -1.45, 0.90),
            half_size=(0.08, 0.55, 0.90),
            rgba=(0.74, 0.70, 0.30, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_gate_south_plinth",
            position=(-2.35, -0.35, 0.45),
            half_size=(0.28, 0.28, 0.45),
            rgba=(0.62, 0.67, 0.40, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_gate_north_panel",
            position=(1.75, 2.75, 0.90),
            half_size=(0.08, 0.55, 0.90),
            rgba=(0.74, 0.70, 0.30, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_goal_south_panel",
            position=(7.85, -5.75, 0.90),
            half_size=(0.72, 0.08, 0.90),
            rgba=(0.74, 0.70, 0.30, 1.0),
            kind="localization_landmark",
        ),
        TerrainBox(
            name="localization_goal_north_panel",
            position=(9.65, 3.85, 0.90),
            half_size=(0.08, 0.72, 0.90),
            rgba=(0.74, 0.70, 0.30, 1.0),
            kind="localization_landmark",
        ),
    ]


def large_terrain_routes() -> list[TerrainRoute]:
    return [
        TerrainRoute(
            name="terrain_short",
            start=(-9.5, -5.6, 0.0),
            goal=(-5.9, -4.2, 0.0),
            description="short smoke route around a guide block",
            min_routed_distance_m=3.5,
        ),
        TerrainRoute(
            name="terrain_long",
            start=DEFAULT_START,
            goal=DEFAULT_GOAL,
            description="20m-class route through the main field and narrow gate",
            min_routed_distance_m=22.0,
        ),
        TerrainRoute(
            name="terrain_narrow_gap",
            start=(-7.8, -5.2, 0.0),
            goal=(6.8, 4.6, 0.0),
            description="route that must pass the central narrow gate",
            min_routed_distance_m=17.5,
        ),
        TerrainRoute(
            name="terrain_slope_bypass",
            start=(-8.8, 4.8, 0.0),
            goal=(8.8, 4.8, 0.0),
            description="upper-field route near a sloped/high-cost band",
            min_routed_distance_m=17.0,
        ),
        TerrainRoute(
            name="terrain_complex_slalom",
            start=(-9.6, 5.9, 0.0),
            goal=(9.3, -5.7, 0.0),
            description="long diagonal route through slalom blocks, central gate, ditch, and boulder field",
            min_routed_distance_m=23.0,
        ),
        TerrainRoute(
            name="terrain_patrol_loop",
            start=(-9.5, -5.6, 0.0),
            goal=(-9.5, -5.6, 0.0),
            description="route catalog entry for future multi-waypoint patrol validation",
            min_routed_distance_m=0.0,
        ),
    ]


def large_terrain_boxes() -> list[TerrainBox]:
    """Return obstacle geometry for a larger outdoor-style navigation field."""

    return [
        TerrainBox(
            name="west_boundary",
            position=(-11.4, 0.0, 0.45),
            half_size=(0.08, 7.4, 0.45),
            rgba=(0.36, 0.39, 0.42, 1.0),
            kind="boundary",
        ),
        TerrainBox(
            name="east_boundary",
            position=(11.4, 0.0, 0.45),
            half_size=(0.08, 7.4, 0.45),
            rgba=(0.36, 0.39, 0.42, 1.0),
            kind="boundary",
        ),
        TerrainBox(
            name="south_boundary",
            position=(0.0, -7.45, 0.45),
            half_size=(11.4, 0.08, 0.45),
            rgba=(0.36, 0.39, 0.42, 1.0),
            kind="boundary",
        ),
        TerrainBox(
            name="north_boundary",
            position=(0.0, 7.45, 0.45),
            half_size=(11.4, 0.08, 0.45),
            rgba=(0.36, 0.39, 0.42, 1.0),
            kind="boundary",
        ),
        TerrainBox(
            name="central_wall_lower",
            position=(0.0, -3.75, 0.55),
            half_size=(0.14, 2.20, 0.55),
            rgba=(0.50, 0.52, 0.58, 1.0),
        ),
        TerrainBox(
            name="central_wall_upper",
            position=(0.0, 4.9, 0.55),
            half_size=(0.14, 2.55, 0.55),
            rgba=(0.50, 0.52, 0.58, 1.0),
        ),
        TerrainBox(
            name="gate_left_post",
            position=(-0.48, 1.45, 0.55),
            half_size=(0.18, 0.45, 0.55),
            rgba=(0.58, 0.48, 0.36, 1.0),
        ),
        TerrainBox(
            name="gate_right_post",
            position=(0.48, 1.45, 0.55),
            half_size=(0.18, 0.45, 0.55),
            rgba=(0.58, 0.48, 0.36, 1.0),
        ),
        TerrainBox(
            name="boulder_a",
            position=(-6.7, -3.2, 0.35),
            half_size=(0.45, 0.40, 0.35),
            rgba=(0.38, 0.34, 0.30, 1.0),
        ),
        TerrainBox(
            name="boulder_b",
            position=(-4.2, -1.7, 0.35),
            half_size=(0.50, 0.35, 0.35),
            rgba=(0.38, 0.34, 0.30, 1.0),
        ),
        TerrainBox(
            name="boulder_c",
            position=(3.6, 0.1, 0.35),
            half_size=(0.42, 0.42, 0.35),
            rgba=(0.38, 0.34, 0.30, 1.0),
        ),
        TerrainBox(
            name="goal_yard_block",
            position=(7.4, 2.1, 0.38),
            half_size=(0.55, 0.28, 0.38),
            rgba=(0.46, 0.37, 0.30, 1.0),
        ),
        TerrainBox(
            name="slalom_upper_a",
            position=(-7.35, 4.85, 0.40),
            half_size=(0.34, 0.45, 0.40),
            rgba=(0.34, 0.42, 0.50, 1.0),
        ),
        TerrainBox(
            name="slalom_upper_b",
            position=(-5.55, 2.85, 0.38),
            half_size=(0.50, 0.28, 0.38),
            rgba=(0.34, 0.42, 0.50, 1.0),
        ),
        TerrainBox(
            name="slalom_mid_a",
            position=(-2.42, 0.42, 0.36),
            half_size=(0.44, 0.30, 0.36),
            rgba=(0.34, 0.42, 0.50, 1.0),
        ),
        TerrainBox(
            name="slalom_mid_b",
            position=(2.65, -0.75, 0.36),
            half_size=(0.48, 0.30, 0.36),
            rgba=(0.34, 0.42, 0.50, 1.0),
        ),
        TerrainBox(
            name="slalom_lower_a",
            position=(5.35, -4.35, 0.38),
            half_size=(0.58, 0.44, 0.38),
            rgba=(0.34, 0.42, 0.50, 1.0),
        ),
        TerrainBox(
            name="slalom_lower_b",
            position=(7.25, -2.35, 0.38),
            half_size=(0.44, 0.56, 0.38),
            rgba=(0.34, 0.42, 0.50, 1.0),
        ),
        *large_terrain_localization_landmarks(),
    ]


def large_terrain_zones() -> list[TerrainZone]:
    return [
        TerrainZone(
            name="rough_gravel_patch",
            center=(-3.5, 3.9),
            half_size=(2.3, 1.1),
            cost=18.0,
            rgba=(0.42, 0.36, 0.30, 0.48),
            kind="rough",
        ),
        TerrainZone(
            name="slope_ramp_band",
            center=(4.8, 4.1),
            half_size=(2.0, 1.0),
            cost=12.0,
            rgba=(0.47, 0.60, 0.42, 0.42),
            kind="slope",
        ),
        TerrainZone(
            name="ditch_no_go",
            center=(4.1, -3.1),
            half_size=(2.0, 0.32),
            cost=100.0,
            rgba=(0.20, 0.25, 0.30, 0.70),
            kind="no_go",
        ),
    ]


def _format_vec(values: tuple[float, ...]) -> str:
    return " ".join(f"{v:.4f}".rstrip("0").rstrip(".") for v in values)


def _scene_xml(start: tuple[float, float, float], goal: tuple[float, float, float]) -> str:
    geoms: list[str] = []
    for zone in large_terrain_zones():
        geoms.append(
            f'    <geom name="{zone.name}" type="box" '
            f'size="{zone.half_size[0]:.4f} {zone.half_size[1]:.4f} 0.015" '
            f'pos="{zone.center[0]:.4f} {zone.center[1]:.4f} 0.018" '
            f'rgba="{_format_vec(zone.rgba)}" contype="0" conaffinity="0" group="2"/>\n'
        )
    for box in large_terrain_boxes():
        geoms.append(
            f'    <geom name="{box.name}" type="box" '
            f'size="{_format_vec(box.half_size)}" pos="{_format_vec(box.position)}" '
            f'rgba="{_format_vec(box.rgba)}" contype="1" conaffinity="1" group="1"/>\n'
        )
    goal_marker = (goal[0], goal[1], max(0.16, goal[2] + 0.16))
    return f"""<mujoco model="lingtu_large_terrain_nav">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <visual>
    <global offwidth="1600" offheight="1000"/>
    <headlight ambient="0.58 0.58 0.56" diffuse="0.78 0.78 0.72"/>
    <quality shadowsize="2048"/>
    <map znear="0.01" zfar="120"/>
  </visual>
  <asset>
    <material name="field_mat" rgba=".50 .54 .56 1"/>
  </asset>
  <worldbody>
    <body name="robot_placeholder" pos="{_format_vec(start)}"/>
    <light pos="2 -7 10" dir="-0.25 0.45 -1" diffuse="0.92 0.90 0.84" castshadow="false"/>
    <geom name="field_floor" type="plane" size="13 9 0.1" material="field_mat"
          contype="1" conaffinity="1" condim="3" friction="1 0.5 0.5" group="1"/>
{"".join(geoms)}    <geom name="goal_marker" type="sphere" size="0.18"
          pos="{_format_vec(goal_marker)}" contype="0" conaffinity="0"
          rgba="0.1 0.34 1 0.75" group="1"/>
  </worldbody>
</mujoco>
"""


def _world_grid(
    shape_xy: tuple[int, int],
    origin: tuple[float, float],
    resolution: float,
) -> tuple[np.ndarray, np.ndarray]:
    x_cells, y_cells = shape_xy
    xs = origin[0] + np.arange(x_cells, dtype=np.float32) * resolution
    ys = origin[1] + np.arange(y_cells, dtype=np.float32) * resolution
    return np.meshgrid(xs, ys, indexing="ij")


def _rect_mask(
    xx: np.ndarray,
    yy: np.ndarray,
    *,
    center: tuple[float, float],
    half: tuple[float, float],
) -> np.ndarray:
    return (
        (xx >= center[0] - half[0])
        & (xx <= center[0] + half[0])
        & (yy >= center[1] - half[1])
        & (yy <= center[1] + half[1])
    )


def _mark_box(
    trav: np.ndarray,
    xx: np.ndarray,
    yy: np.ndarray,
    *,
    box: TerrainBox,
    obstacle_thr: float,
    inflation: float,
) -> None:
    mask = _rect_mask(
        xx,
        yy,
        center=(box.position[0], box.position[1]),
        half=(box.half_size[0] + inflation, box.half_size[1] + inflation),
    )
    hard_obstacle_cost = max(float(obstacle_thr), 100.0)
    trav[mask] = hard_obstacle_cost


def _mark_zone(trav: np.ndarray, xx: np.ndarray, yy: np.ndarray, *, zone: TerrainZone) -> None:
    mask = _rect_mask(xx, yy, center=zone.center, half=zone.half_size)
    if zone.cost >= 49.9:
        trav[mask] = np.maximum(trav[mask], zone.cost)
    else:
        trav[mask] = np.maximum(trav[mask], zone.cost)


def _map_frame_box(box: TerrainBox, origin_world_xy: tuple[float, float]) -> TerrainBox:
    return TerrainBox(
        name=box.name,
        position=(
            float(box.position[0]) - float(origin_world_xy[0]),
            float(box.position[1]) - float(origin_world_xy[1]),
            float(box.position[2]),
        ),
        half_size=box.half_size,
        rgba=box.rgba,
        kind=box.kind,
    )


def _map_frame_zone(zone: TerrainZone, origin_world_xy: tuple[float, float]) -> TerrainZone:
    return TerrainZone(
        name=zone.name,
        center=(
            float(zone.center[0]) - float(origin_world_xy[0]),
            float(zone.center[1]) - float(origin_world_xy[1]),
        ),
        half_size=zone.half_size,
        cost=zone.cost,
        rgba=zone.rgba,
        kind=zone.kind,
    )


def _map_frame_route(route: TerrainRoute, origin_world_xy: tuple[float, float]) -> TerrainRoute:
    return TerrainRoute(
        name=route.name,
        start=(
            float(route.start[0]) - float(origin_world_xy[0]),
            float(route.start[1]) - float(origin_world_xy[1]),
            float(route.start[2]),
        ),
        goal=(
            float(route.goal[0]) - float(origin_world_xy[0]),
            float(route.goal[1]) - float(origin_world_xy[1]),
            float(route.goal[2]),
        ),
        description=route.description,
        min_routed_distance_m=route.min_routed_distance_m,
    )


def _tomogram(
    *,
    resolution: float,
    origin: tuple[float, float],
    shape_xy: tuple[int, int],
    slice_dh: float,
    obstacle_thr: float,
    inflation: float,
    map_frame_origin_world_xy: tuple[float, float] = (0.0, 0.0),
) -> dict[str, Any]:
    trav = np.full(shape_xy, 1.0, dtype=np.float32)
    xx, yy = _world_grid(shape_xy, origin, resolution)
    for zone in large_terrain_zones():
        _mark_zone(trav, xx, yy, zone=_map_frame_zone(zone, map_frame_origin_world_xy))
    for box in large_terrain_boxes():
        _mark_box(
            trav,
            xx,
            yy,
            box=_map_frame_box(box, map_frame_origin_world_xy),
            obstacle_thr=obstacle_thr,
            inflation=inflation,
        )

    data = np.zeros((5, 1, shape_xy[0], shape_xy[1]), dtype=np.float32)
    data[0, 0] = trav
    data[3, 0] = 0.0
    data[4, 0] = 2.2
    center = [
        origin[0] + shape_xy[0] * resolution / 2.0,
        origin[1] + shape_xy[1] * resolution / 2.0,
    ]
    return {
        "data": data,
        "resolution": float(resolution),
        "center": center,
        "origin": list(origin),
        "slice_h0": 0.0,
        "slice_dh": float(slice_dh),
        "grid_info": {
            "axis_order": "builder_xy",
            "origin": list(origin),
            "resolution": float(resolution),
            "shape_xy": [int(shape_xy[0]), int(shape_xy[1])],
        },
        "meta": {
            "source": "sim_large_terrain_geometry",
            "obstacle_thr": float(obstacle_thr),
            "inflation_m": float(inflation),
            "map_frame_origin_world_xy": list(map_frame_origin_world_xy),
        },
    }


def _sample_box_surfaces(box: TerrainBox, step: float = 0.16) -> np.ndarray:
    cx, cy, cz = box.position
    hx, hy, hz = box.half_size
    xs = np.arange(cx - hx, cx + hx + step * 0.5, step)
    ys = np.arange(cy - hy, cy + hy + step * 0.5, step)
    zs = np.arange(max(0.0, cz - hz), cz + hz + step * 0.5, step)
    pts: list[tuple[float, float, float]] = []
    for x in xs:
        for y in ys:
            pts.append((float(x), float(y), float(cz + hz)))
    for z in zs:
        for y in ys:
            pts.append((float(cx - hx), float(y), float(z)))
            pts.append((float(cx + hx), float(y), float(z)))
        for x in xs:
            pts.append((float(x), float(cy - hy), float(z)))
            pts.append((float(x), float(cy + hy), float(z)))
    return np.asarray(pts, dtype=np.float32)


def _sample_zone_points(zone: TerrainZone, step: float = 0.35) -> np.ndarray:
    xs = np.arange(zone.center[0] - zone.half_size[0], zone.center[0] + zone.half_size[0] + step * 0.5, step)
    ys = np.arange(zone.center[1] - zone.half_size[1], zone.center[1] + zone.half_size[1] + step * 0.5, step)
    pts = [(float(x), float(y), 0.03) for x in xs for y in ys]
    return np.asarray(pts, dtype=np.float32)


def sample_large_terrain_map_points(
    step: float = 0.16,
    *,
    map_frame_origin_world_xy: tuple[float, float] = (0.0, 0.0),
) -> np.ndarray:
    blocks = [
        _sample_box_surfaces(_map_frame_box(box, map_frame_origin_world_xy), step=step) for box in large_terrain_boxes()
    ]
    blocks.extend(
        _sample_zone_points(_map_frame_zone(zone, map_frame_origin_world_xy)) for zone in large_terrain_zones()
    )
    points = np.concatenate(blocks, axis=0)
    return np.asarray(points[np.all(np.isfinite(points), axis=1)], dtype=np.float32)


def _write_ascii_pcd(path: Path, *, map_frame_origin_world_xy: tuple[float, float]) -> None:
    points = sample_large_terrain_map_points(
        map_frame_origin_world_xy=map_frame_origin_world_xy,
    )
    lines = [
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
    lines.extend(f"{x:.4f} {y:.4f} {z:.4f}" for x, y, z in points)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_large_terrain_assets(
    output_dir: str | Path,
    *,
    start: tuple[float, float, float] = DEFAULT_START,
    goal: tuple[float, float, float] = DEFAULT_GOAL,
    resolution: float = DEFAULT_RESOLUTION,
    origin: tuple[float, float] = DEFAULT_ORIGIN,
    shape_xy: tuple[int, int] = DEFAULT_SHAPE_XY,
    slice_dh: float = DEFAULT_SLICE_DH,
    obstacle_thr: float = 49.9,
    inflation: float = 0.16,
    map_frame_origin_world_xy: tuple[float, float] | None = None,
) -> LargeTerrainAssets:
    out = Path(output_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    scene_xml = out / "large_terrain_scene.xml"
    tomogram_path = out / "tomogram.pickle"
    map_pcd = out / "map.pcd"
    metadata = out / "metadata.json"
    routes = tuple(large_terrain_routes())
    map_frame_origin_world_xy = tuple(float(value) for value in (map_frame_origin_world_xy or (0.0, 0.0)))
    tomogram_origin = (
        float(origin[0]) - float(map_frame_origin_world_xy[0]),
        float(origin[1]) - float(map_frame_origin_world_xy[1]),
    )
    map_frame = (
        "world"
        if abs(map_frame_origin_world_xy[0]) < 1e-9 and abs(map_frame_origin_world_xy[1]) < 1e-9
        else "start_odom"
        if (
            abs(map_frame_origin_world_xy[0] - float(start[0])) < 1e-9
            and abs(map_frame_origin_world_xy[1] - float(start[1])) < 1e-9
        )
        else "offset_odom"
    )

    scene_xml.write_text(_scene_xml(start, goal), encoding="utf-8")
    tomogram_payload = _tomogram(
        resolution=resolution,
        origin=tomogram_origin,
        shape_xy=shape_xy,
        slice_dh=slice_dh,
        obstacle_thr=obstacle_thr,
        inflation=inflation,
        map_frame_origin_world_xy=map_frame_origin_world_xy,
    )
    with tomogram_path.open("wb") as fh:
        pickle.dump(tomogram_payload, fh, protocol=pickle.HIGHEST_PROTOCOL)
    _write_ascii_pcd(map_pcd, map_frame_origin_world_xy=map_frame_origin_world_xy)
    map_sha = sha256_file(map_pcd)
    tomogram_sha = sha256_file(tomogram_path)
    point_count = int(len(sample_large_terrain_map_points(map_frame_origin_world_xy=map_frame_origin_world_xy)))
    metadata_payload = build_saved_map_metadata(
        source_profile="large_terrain_synthetic_assets",
        data_source="synthetic_large_terrain_geometry",
        slam_source="synthetic_large_terrain_map_cloud",
        localization_source="synthetic_large_terrain_ground_truth",
        mapping_source="synthetic_large_terrain_geometry_to_map_pcd_and_tomogram",
        frame_id="map",
        artifacts={
            "map_pcd": {
                "path": str(map_pcd),
                "sha256": map_sha,
                "source_profile": "large_terrain_synthetic_assets",
                "data_source": "synthetic_large_terrain_geometry",
                "slam_source": "synthetic_large_terrain_map_cloud",
                "frame_id": "map",
                "point_count": point_count,
            },
            "tomogram": {
                "path": str(tomogram_path),
                "sha256": tomogram_sha,
                "source_map_sha256": map_sha,
                "source_profile": "large_terrain_synthetic_assets",
                "data_source": "synthetic_large_terrain_geometry",
                "frame_id": "map",
                "shape": list(np.asarray(tomogram_payload["data"]).shape),
            },
        },
        extra_metadata={
            "name": "large_terrain_field",
            "source": "synthetic_sim_geometry",
            "scene_xml": str(scene_xml),
            "tomogram": str(tomogram_path),
            "map_pcd": str(map_pcd),
            "start": list(start),
            "goal": list(goal),
            "map_frame": map_frame,
            "map_frame_origin_world_xy": list(map_frame_origin_world_xy),
            "resolution": resolution,
            "origin": list(tomogram_origin),
            "world_origin": list(origin),
            "shape_xy": list(shape_xy),
            "slice_dh": slice_dh,
            "inflation_m": inflation,
            "obstacle_thr": obstacle_thr,
            "obstacles": [asdict(box) for box in large_terrain_boxes()],
            "map_frame_obstacles": [
                asdict(_map_frame_box(box, map_frame_origin_world_xy)) for box in large_terrain_boxes()
            ],
            "terrain_zones": [asdict(zone) for zone in large_terrain_zones()],
            "map_frame_terrain_zones": [
                asdict(_map_frame_zone(zone, map_frame_origin_world_xy)) for zone in large_terrain_zones()
            ],
            "routes": [asdict(route) for route in routes],
            "map_frame_routes": [asdict(_map_frame_route(route, map_frame_origin_world_xy)) for route in routes],
        },
    )
    metadata.write_text(
        json.dumps(metadata_payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return LargeTerrainAssets(
        scene_xml=scene_xml,
        tomogram=tomogram_path,
        map_pcd=map_pcd,
        metadata=metadata,
        start=start,
        goal=goal,
        routes=routes,
    )
