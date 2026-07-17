"""Deterministic multi-floor assets for navigation simulation validation."""

from __future__ import annotations

import json
import pickle
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from maps.artifacts import build_saved_map_metadata, sha256_file


@dataclass(frozen=True)
class MultiFloorBox:
    name: str
    position: tuple[float, float, float]
    half_size: tuple[float, float, float]
    rgba: tuple[float, float, float, float]
    floor_id: int | None = None


@dataclass(frozen=True)
class FloorTransition:
    name: str
    floor_from: int
    floor_to: int
    kind: str
    lower_xy: tuple[float, float]
    upper_xy: tuple[float, float]
    width_m: float


@dataclass(frozen=True)
class MultiFloorAssets:
    scene_xml: Path
    tomogram: Path
    map_pcd: Path
    metadata: Path
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    floors: tuple[float, ...]


DEFAULT_START = (-0.4, -2.1, 0.0)
DEFAULT_GOAL = (6.7, 2.1, 2.5)
DEFAULT_RESOLUTION = 0.2
DEFAULT_ORIGIN = (-1.2, -4.0)
DEFAULT_SHAPE_XY = (48, 42)  # x cells, y cells; LingTu builder convention.
DEFAULT_FLOORS = (0.0, 2.5)
DEFAULT_SLICE_DH = 0.5


def floor_transitions() -> list[FloorTransition]:
    return [
        FloorTransition(
            name="north_stair",
            floor_from=0,
            floor_to=1,
            kind="stairs",
            lower_xy=(2.2, -1.6),
            upper_xy=(4.3, 1.4),
            width_m=1.0,
        )
    ]


def multifloor_boxes() -> list[MultiFloorBox]:
    """Return coarse geometry for a two-floor indoor validation scene."""

    return [
        MultiFloorBox(
            name="lower_floor_plate",
            position=(1.2, -2.0, -0.04),
            half_size=(2.4, 1.35, 0.04),
            rgba=(0.70, 0.72, 0.68, 1.0),
            floor_id=0,
        ),
        MultiFloorBox(
            name="upper_floor_plate",
            position=(5.5, 2.0, 2.46),
            half_size=(2.3, 1.35, 0.04),
            rgba=(0.68, 0.72, 0.78, 1.0),
            floor_id=1,
        ),
        MultiFloorBox(
            name="lower_room_wall_west",
            position=(-0.95, -2.0, 0.45),
            half_size=(0.06, 1.35, 0.45),
            rgba=(0.50, 0.50, 0.56, 1.0),
            floor_id=0,
        ),
        MultiFloorBox(
            name="lower_room_wall_south",
            position=(1.2, -3.35, 0.45),
            half_size=(2.15, 0.06, 0.45),
            rgba=(0.50, 0.50, 0.56, 1.0),
            floor_id=0,
        ),
        MultiFloorBox(
            name="upper_room_wall_east",
            position=(7.75, 2.0, 2.95),
            half_size=(0.06, 1.35, 0.45),
            rgba=(0.50, 0.50, 0.56, 1.0),
            floor_id=1,
        ),
        MultiFloorBox(
            name="upper_room_wall_north",
            position=(5.5, 3.35, 2.95),
            half_size=(2.20, 0.06, 0.45),
            rgba=(0.50, 0.50, 0.56, 1.0),
            floor_id=1,
        ),
        MultiFloorBox(
            name="stair_volume",
            position=(3.25, -0.1, 1.25),
            half_size=(0.65, 2.00, 1.25),
            rgba=(0.78, 0.58, 0.32, 1.0),
        ),
        MultiFloorBox(
            name="pillar_lower",
            position=(0.85, -1.55, 0.35),
            half_size=(0.18, 0.18, 0.35),
            rgba=(0.45, 0.40, 0.35, 1.0),
            floor_id=0,
        ),
        MultiFloorBox(
            name="pillar_upper",
            position=(5.9, 1.55, 2.85),
            half_size=(0.18, 0.18, 0.35),
            rgba=(0.45, 0.40, 0.35, 1.0),
            floor_id=1,
        ),
    ]


def _format_vec(values: tuple[float, ...]) -> str:
    return " ".join(f"{v:.4f}".rstrip("0").rstrip(".") for v in values)


def _scene_xml(start: tuple[float, float, float], goal: tuple[float, float, float]) -> str:
    geoms: list[str] = []
    for box in multifloor_boxes():
        if box.name == "stair_volume":
            geoms.append(
                f'    <geom name="{box.name}" type="box" '
                f'size="{_format_vec(box.half_size)}" pos="{_format_vec(box.position)}" '
                f'rgba="{_format_vec(box.rgba)}" contype="1" conaffinity="1" group="1"/>\n'
            )
            continue
        geoms.append(
            f'    <geom name="{box.name}" type="box" '
            f'size="{_format_vec(box.half_size)}" pos="{_format_vec(box.position)}" '
            f'rgba="{_format_vec(box.rgba)}" contype="1" conaffinity="1" group="1"/>\n'
        )
    return f"""<mujoco model="lingtu_multifloor_nav">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <headlight ambient="0.42 0.42 0.42"/>
    <quality shadowsize="2048"/>
    <map znear="0.01" zfar="80"/>
  </visual>
  <asset>
    <material name="ground_mat" rgba=".62 .65 .62 1"/>
  </asset>
  <worldbody>
    <body name="robot_placeholder" pos="{_format_vec(start)}"/>
    <light pos="5 -6 8" dir="-0.4 0.35 -1" diffuse="0.9 0.88 0.82" castshadow="false"/>
    <geom name="ground_reference" type="plane" size="12 8 0.1" material="ground_mat"
          contype="1" conaffinity="1" condim="3" friction="1 0.5 0.5" group="1"/>
{"".join(geoms)}    <geom name="goal_marker" type="sphere" size="0.16"
          pos="{_format_vec(goal)}" contype="0" conaffinity="0"
          rgba="0.1 0.35 1 0.75" group="1"/>
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


def _segment_mask(
    xx: np.ndarray,
    yy: np.ndarray,
    *,
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
) -> np.ndarray:
    ax, ay = start
    bx, by = end
    vx = bx - ax
    vy = by - ay
    denom = max(vx * vx + vy * vy, 1e-9)
    t = np.clip(((xx - ax) * vx + (yy - ay) * vy) / denom, 0.0, 1.0)
    px = ax + t * vx
    py = ay + t * vy
    return (xx - px) ** 2 + (yy - py) ** 2 <= (width * 0.5) ** 2


def _mark_box_obstacles(
    trav: np.ndarray,
    xx: np.ndarray,
    yy: np.ndarray,
    *,
    box: MultiFloorBox,
    origin_z: float,
    slice_dh: float,
    obstacle_thr: float,
    inflation: float,
) -> None:
    if box.name in {"lower_floor_plate", "upper_floor_plate", "stair_volume"}:
        return
    mask = _rect_mask(
        xx,
        yy,
        center=(box.position[0], box.position[1]),
        half=(box.half_size[0] + inflation, box.half_size[1] + inflation),
    )
    z_min = box.position[2] - box.half_size[2]
    z_max = box.position[2] + box.half_size[2]
    first_slice = int(np.floor((z_min - origin_z) / slice_dh))
    last_slice = int(np.ceil((z_max - origin_z) / slice_dh))
    first_slice = int(np.clip(first_slice, 0, trav.shape[0] - 1))
    last_slice = int(np.clip(last_slice, 0, trav.shape[0] - 1))
    for target_slice in range(first_slice, last_slice + 1):
        trav[target_slice, mask] = obstacle_thr


def _tomogram(
    *,
    floors: tuple[float, ...],
    resolution: float,
    origin: tuple[float, float],
    shape_xy: tuple[int, int],
    slice_dh: float,
    obstacle_thr: float,
    inflation: float,
) -> dict[str, Any]:
    n_slices = int(round((max(floors) - 0.0) / slice_dh)) + 1
    trav = np.full((n_slices, shape_xy[0], shape_xy[1]), obstacle_thr, dtype=np.float32)
    # Keep elevation valid even for blocked cells. The original PCT planner
    # detects vertical gateways from traversability changes plus adjacent
    # elevation continuity; NaN obstacle elevations suppress gateway detection.
    elev_g = np.zeros_like(trav, dtype=np.float32)
    elev_c = np.zeros_like(trav, dtype=np.float32)
    for slice_idx in range(n_slices):
        nominal = slice_idx * slice_dh * 0.9
        elev_g[slice_idx, :, :] = nominal
        elev_c[slice_idx, :, :] = nominal + 2.2
    xx, yy = _world_grid(shape_xy, origin, resolution)

    lower = _rect_mask(xx, yy, center=(1.2, -2.0), half=(2.25, 1.20))
    upper = _rect_mask(xx, yy, center=(5.5, 2.0), half=(2.10, 1.15))
    stair = _segment_mask(xx, yy, start=(2.2, -1.6), end=(4.3, 1.4), width=1.0)

    def mark_free(slice_idx: int, mask: np.ndarray, height: float) -> None:
        trav[slice_idx, mask] = 0.0
        elev_g[slice_idx, mask] = height
        elev_c[slice_idx, mask] = height + 2.2

    mark_free(0, lower | stair, floors[0])
    top_slice = n_slices - 1
    mark_free(top_slice, upper | stair, float(floors[-1]))
    for slice_idx in range(1, top_slice):
        height = slice_idx * slice_dh
        mark_free(slice_idx, stair, height)

    for box in multifloor_boxes():
        _mark_box_obstacles(
            trav,
            xx,
            yy,
            box=box,
            origin_z=0.0,
            slice_dh=slice_dh,
            obstacle_thr=obstacle_thr,
            inflation=inflation,
        )

    data = np.zeros((5, n_slices, shape_xy[0], shape_xy[1]), dtype=np.float32)
    data[0] = trav
    data[1] = 0.0
    data[2] = 0.0
    data[3] = elev_g
    data[4] = elev_c
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
        "floor_heights": list(floors),
        "floor_transitions": [asdict(t) for t in floor_transitions()],
        "grid_info": {
            "axis_order": "builder_xy",
            "origin": list(origin),
            "resolution": float(resolution),
            "shape_xy": [int(shape_xy[0]), int(shape_xy[1])],
        },
        "meta": {
            "source": "sim_multifloor_geometry",
            "obstacle_thr": float(obstacle_thr),
            "inflation_m": float(inflation),
        },
    }


def _sample_box_surfaces(box: MultiFloorBox, step: float = 0.12) -> np.ndarray:
    cx, cy, cz = box.position
    hx, hy, hz = box.half_size
    xs = np.arange(cx - hx, cx + hx + step * 0.5, step)
    ys = np.arange(cy - hy, cy + hy + step * 0.5, step)
    zs = np.arange(cz - hz, cz + hz + step * 0.5, step)
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


def sample_multifloor_map_points(step: float = 0.12) -> np.ndarray:
    points = np.concatenate([_sample_box_surfaces(box, step=step) for box in multifloor_boxes()], axis=0)
    return np.asarray(points[np.all(np.isfinite(points), axis=1)], dtype=np.float32)


def _write_ascii_pcd(path: Path) -> None:
    points = sample_multifloor_map_points()
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


def build_multifloor_assets(
    output_dir: str | Path,
    *,
    start: tuple[float, float, float] = DEFAULT_START,
    goal: tuple[float, float, float] = DEFAULT_GOAL,
    floors: tuple[float, ...] = DEFAULT_FLOORS,
    resolution: float = DEFAULT_RESOLUTION,
    origin: tuple[float, float] = DEFAULT_ORIGIN,
    shape_xy: tuple[int, int] = DEFAULT_SHAPE_XY,
    slice_dh: float = DEFAULT_SLICE_DH,
    obstacle_thr: float = 49.9,
    inflation: float = 0.10,
) -> MultiFloorAssets:
    out = Path(output_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    scene_xml = out / "multifloor_scene.xml"
    tomogram_path = out / "tomogram.pickle"
    map_pcd = out / "map.pcd"
    metadata = out / "metadata.json"

    scene_xml.write_text(_scene_xml(start, goal), encoding="utf-8")
    tomogram_payload = _tomogram(
        floors=floors,
        resolution=resolution,
        origin=origin,
        shape_xy=shape_xy,
        slice_dh=slice_dh,
        obstacle_thr=obstacle_thr,
        inflation=inflation,
    )
    with tomogram_path.open("wb") as fh:
        pickle.dump(tomogram_payload, fh, protocol=pickle.HIGHEST_PROTOCOL)
    _write_ascii_pcd(map_pcd)
    map_sha = sha256_file(map_pcd)
    tomogram_sha = sha256_file(tomogram_path)
    point_count = int(len(sample_multifloor_map_points()))
    metadata_payload = build_saved_map_metadata(
        source_profile="multifloor_synthetic_assets",
        data_source="synthetic_multifloor_geometry",
        slam_source="synthetic_multifloor_map_cloud",
        localization_source="synthetic_multifloor_ground_truth",
        mapping_source="synthetic_multifloor_geometry_to_map_pcd_and_tomogram",
        frame_id="map",
        artifacts={
            "map_pcd": {
                "path": map_pcd.relative_to(out).as_posix(),
                "sha256": map_sha,
                "source_profile": "multifloor_synthetic_assets",
                "data_source": "synthetic_multifloor_geometry",
                "slam_source": "synthetic_multifloor_map_cloud",
                "frame_id": "map",
                "point_count": point_count,
            },
            "tomogram": {
                "path": tomogram_path.relative_to(out).as_posix(),
                "sha256": tomogram_sha,
                "source_map_sha256": map_sha,
                "source_profile": "multifloor_synthetic_assets",
                "data_source": "synthetic_multifloor_geometry",
                "frame_id": "map",
                "shape": list(np.asarray(tomogram_payload["data"]).shape),
            },
        },
        extra_metadata={
            "name": "multifloor_stairs",
            "source": "synthetic_sim_geometry",
            "scene_xml": str(scene_xml),
            "tomogram": str(tomogram_path),
            "map_pcd": str(map_pcd),
            "start": list(start),
            "goal": list(goal),
            "floors": list(floors),
            "resolution": resolution,
            "origin": list(origin),
            "shape_xy": list(shape_xy),
            "slice_dh": slice_dh,
            "inflation_m": inflation,
            "transitions": [asdict(t) for t in floor_transitions()],
            "obstacles": [asdict(box) for box in multifloor_boxes()],
        },
    )
    metadata.write_text(
        json.dumps(metadata_payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return MultiFloorAssets(
        scene_xml=scene_xml,
        tomogram=tomogram_path,
        map_pcd=map_pcd,
        metadata=metadata,
        start=start,
        goal=goal,
        floors=floors,
    )
