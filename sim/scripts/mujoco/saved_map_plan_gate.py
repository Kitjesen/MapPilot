#!/usr/bin/env python3
"""Generate a tiny MuJoCo saved map and validate OctoPlanner3D planning."""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from nav.services.map_layers.map_artifact_builder import (  # noqa: E402
    MapArtifactBuilder,
    MapArtifactBuilderConfig,
)
from nav.services.plan.global_planner.algorithm.octoplanner3d_planner import (  # noqa: E402
    OctoPlanner3DPlanner,
)
from runtime.same_source_map_artifacts import validate_saved_map_artifact_dir  # noqa: E402

BUILDING_SCENE_XML = ROOT / "sim" / "worlds" / "mujoco" / "building_scene.xml"
CORRIDOR_DEFAULT_START = [0.0, 0.0, 0.0]
CORRIDOR_DEFAULT_GOAL = [2.4, 0.0, 0.0]
BUILDING_DEFAULT_START = [2.0, 3.0, 0.5]
BUILDING_DEFAULT_GOAL = [18.0, 11.0, 4.0]
BUILDING_MAP_PREFIXES = ("floor_", "wall_", "stair_", "step_", "obs_")


def _frange(start: float, stop: float, step: float) -> list[float]:
    values: list[float] = []
    n = 0
    while True:
        value = start + n * step
        if value > stop + step * 0.5:
            return values
        values.append(round(value, 6))
        n += 1


def generate_scene_xml(
    path: Path,
    *,
    length: float,
    width: float,
    scene_preset: str = "corridor",
) -> None:
    if scene_preset == "building":
        shutil.copyfile(BUILDING_SCENE_XML, path)
        return
    if scene_preset != "corridor":
        raise ValueError(f"unsupported scene preset: {scene_preset}")
    path.write_text(
        f"""<mujoco model="lingtu_mujoco_saved_map_gate">
  <worldbody>
    <geom name="floor" type="box" pos="{length / 2:.3f} 0 -0.025" size="{length / 2:.3f} {width / 2:.3f} 0.025"/>
    <geom name="left_rail" type="box" pos="{length / 2:.3f} {width / 2:.3f} 0.4" size="{length / 2:.3f} 0.05 0.4"/>
    <geom name="right_rail" type="box" pos="{length / 2:.3f} {-width / 2:.3f} 0.4" size="{length / 2:.3f} 0.05 0.4"/>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )


def generate_points(
    *,
    length: float,
    width: float,
    spacing: float,
    hits_per_cell: int = 4,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    offsets = _hit_offsets(spacing, max(1, int(hits_per_cell)))
    xs = _frange(-0.2, length + 0.2, spacing)
    ys = _frange(-width / 2.0, width / 2.0, spacing)
    for x in xs:
        for y in ys:
            points.extend((x + dx, y + dy, dz) for dx, dy, dz in offsets)
    for y in (-width / 2.0, width / 2.0):
        for x in xs:
            for z in _frange(0.2, 0.8, spacing):
                points.extend((x + dx, y + dy, z + dz) for dx, dy, dz in offsets)
    return points


def _parse_vec(text: str | None, expected: int) -> list[float]:
    if not text:
        return []
    values = [float(v) for v in text.split()]
    if len(values) != expected or not all(math.isfinite(v) for v in values):
        return []
    return values


def _included_building_geom(name: str) -> bool:
    return name.startswith(BUILDING_MAP_PREFIXES)


def _append_hits(
    points: list[tuple[float, float, float]],
    x: float,
    y: float,
    z: float,
    offsets: list[tuple[float, float, float]],
) -> None:
    points.extend((x + dx, y + dy, z + dz) for dx, dy, dz in offsets)


def _sample_box_surfaces(
    points: list[tuple[float, float, float]],
    *,
    pos: list[float],
    size: list[float],
    spacing: float,
    offsets: list[tuple[float, float, float]],
) -> None:
    cx, cy, cz = pos
    hx, hy, hz = size
    x0, x1 = cx - hx, cx + hx
    y0, y1 = cy - hy, cy + hy
    z0, z1 = cz - hz, cz + hz
    for x in _frange(x0, x1, spacing):
        for y in _frange(y0, y1, spacing):
            _append_hits(points, x, y, z1, offsets)
    for z in _frange(z0, z1, spacing):
        for y in _frange(y0, y1, spacing):
            _append_hits(points, x0, y, z, offsets)
            _append_hits(points, x1, y, z, offsets)
        for x in _frange(x0, x1, spacing):
            _append_hits(points, x, y0, z, offsets)
            _append_hits(points, x, y1, z, offsets)


def _sample_cylinder_surfaces(
    points: list[tuple[float, float, float]],
    *,
    pos: list[float],
    size: list[float],
    spacing: float,
    offsets: list[tuple[float, float, float]],
) -> None:
    if len(size) < 2:
        return
    cx, cy, cz = pos
    radius, half_height = size[:2]
    z0, z1 = cz - half_height, cz + half_height
    angle_step = max(0.12, min(math.pi / 8.0, spacing / max(radius, 1e-6)))
    angles = _frange(0.0, math.tau, angle_step)
    for r in _frange(0.0, radius, max(spacing, radius)):
        for angle in angles:
            x = cx + math.cos(angle) * r
            y = cy + math.sin(angle) * r
            _append_hits(points, x, y, z1, offsets)
    for z in _frange(z0, z1, spacing):
        for angle in angles:
            x = cx + math.cos(angle) * radius
            y = cy + math.sin(angle) * radius
            _append_hits(points, x, y, z, offsets)


def _building_map_geoms() -> list[dict[str, Any]]:
    root = ET.parse(BUILDING_SCENE_XML).getroot()
    geoms: list[dict[str, Any]] = []
    for geom in root.findall(".//geom"):
        name = geom.attrib.get("name", "")
        if not _included_building_geom(name):
            continue
        geom_type = geom.attrib.get("type", "box")
        pos = _parse_vec(geom.attrib.get("pos"), 3)
        size_len = 2 if geom_type == "cylinder" else 3
        size = _parse_vec(geom.attrib.get("size"), size_len)
        if not pos or not size:
            continue
        geoms.append({"name": name, "type": geom_type, "pos": pos, "size": size})
    return geoms


def generate_building_points(
    *,
    spacing: float,
    hits_per_cell: int = 4,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    offsets = _hit_offsets(spacing, max(1, int(hits_per_cell)))
    for geom in _building_map_geoms():
        if geom["type"] == "box":
            _sample_box_surfaces(
                points,
                pos=geom["pos"],
                size=geom["size"],
                spacing=spacing,
                offsets=offsets,
            )
        elif geom["type"] == "cylinder":
            _sample_cylinder_surfaces(
                points,
                pos=geom["pos"],
                size=geom["size"],
                spacing=spacing,
                offsets=offsets,
            )
    return points


def _hit_offsets(spacing: float, count: int) -> list[tuple[float, float, float]]:
    base = max(0.001, min(0.02, spacing * 0.05))
    pattern = [
        (0.0, 0.0, 0.0),
        (base, 0.0, 0.0),
        (0.0, base, 0.0),
        (base, base, 0.0),
        (-base, 0.0, 0.0),
        (0.0, -base, 0.0),
    ]
    return pattern[:count] if count <= len(pattern) else pattern + [pattern[-1]] * (count - len(pattern))


def write_ascii_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
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
    body = "\n".join(f"{x:.6f} {y:.6f} {z:.6f}" for x, y, z in points)
    path.write_text(header + "\n" + body + "\n", encoding="ascii")


def filter_points_to_scene(
    points: list[tuple[float, float, float]],
    *,
    length: float,
    width: float,
    margin: float = 0.5,
    z_min: float = -0.2,
    z_max: float = 1.2,
    x_min: float | None = None,
    x_max: float | None = None,
    y_min: float | None = None,
    y_max: float | None = None,
) -> list[tuple[float, float, float]]:
    x_min = -margin if x_min is None else x_min
    x_max = length + margin if x_max is None else x_max
    y_min = -(width / 2.0 + margin) if y_min is None else y_min
    y_max = width / 2.0 + margin if y_max is None else y_max
    return [
        (x, y, z)
        for x, y, z in points
        if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max
    ]


def building_scene_clip(margin: float = 0.5) -> dict[str, float]:
    x_min = y_min = z_min = float("inf")
    x_max = y_max = z_max = float("-inf")
    for geom in _building_map_geoms():
        x, y, z = geom["pos"]
        size = geom["size"]
        if geom["type"] == "cylinder":
            radius, half_height = size[:2]
            bounds = (x - radius, x + radius, y - radius, y + radius, z - half_height, z + half_height)
        else:
            hx, hy, hz = size
            bounds = (x - hx, x + hx, y - hy, y + hy, z - hz, z + hz)
        gx0, gx1, gy0, gy1, gz0, gz1 = bounds
        x_min, x_max = min(x_min, gx0), max(x_max, gx1)
        y_min, y_max = min(y_min, gy0), max(y_max, gy1)
        z_min, z_max = min(z_min, gz0), max(z_max, gz1)
    return {
        "x_min": x_min - margin,
        "x_max": x_max + margin,
        "y_min": y_min - margin,
        "y_max": y_max + margin,
        "z_min": z_min - margin,
        "z_max": z_max + margin,
        "margin": margin,
    }


def collect_mujoco_lidar_points(
    scene_xml: Path,
    *,
    scans: int,
    duration_s: float,
    timeout_s: float,
    vx: float,
    wz: float,
    publish_hz: float,
    n_rays: int,
    mid360_pattern: Path | None,
    mid360_samples_per_frame: int,
    lidar_backend: str,
    mujoco_lidar_backend: str,
    allow_legacy_lidar_fallback: bool,
    mujoco_memory: str,
) -> tuple[list[tuple[float, float, float]], dict[str, Any]]:
    from drivers.sim.mujoco.runtime import (
        DEFAULT_MID360_PATTERN,
        build_engine,
    )
    from sim.engine.core.engine import VelocityCommand

    engine = None
    clouds: list[Any] = []
    odoms: list[Any] = []
    backend_report: dict[str, Any] = {}
    try:
        engine = build_engine(
            world=scene_xml.resolve(),
            drive_mode="kinematic",
            n_rays=int(n_rays),
            start=None,
            mujoco_memory=mujoco_memory,
            mid360_pattern=mid360_pattern or DEFAULT_MID360_PATTERN,
            mid360_samples_per_frame=int(mid360_samples_per_frame),
            lidar_backend=lidar_backend,
            mujoco_lidar_backend=mujoco_lidar_backend,
            require_product_lidar_backend=not bool(allow_legacy_lidar_fallback),
            allow_legacy_lidar_fallback=bool(allow_legacy_lidar_fallback),
        )
        backend_report = engine.get_lidar_backend_report()
    except Exception as exc:
        if engine is not None:
            engine.close()
        return [], {
            "ok": False,
            "scan_count": 0,
            "point_count": 0,
            "error": f"{type(exc).__name__}: {exc}",
            "source": "build_engine.get_lidar_points",
            "requested_backend": str(lidar_backend),
            "mujoco_lidar_backend": str(mujoco_lidar_backend),
            "allow_legacy_lidar_fallback": bool(allow_legacy_lidar_fallback),
        }

    cmd = VelocityCommand(linear_x=float(vx), linear_y=0.0, angular_z=float(wz))
    scan_period_s = 1.0 / max(1e-6, float(publish_hz))
    next_scan_s = 0.0
    scan_attempts = 0
    started = time.time()
    deadline = started + max(0.1, timeout_s)
    try:
        while time.time() < deadline and (
            scan_attempts < max(1, scans) or float(engine.sim_time) < max(0.0, duration_s)
        ):
            state = engine.step(cmd)
            odoms.append(state)
            if float(engine.sim_time) + 1e-9 < next_scan_s:
                continue
            cloud = engine.get_lidar_points()
            scan_attempts += 1
            next_scan_s += scan_period_s
            if cloud is not None and len(cloud) > 0:
                clouds.append(cloud)
    finally:
        engine.step(VelocityCommand())
        engine.close()

    points: list[tuple[float, float, float]] = []
    for cloud in clouds:
        for point in cloud:
            if len(point) >= 3:
                points.append((float(point[0]), float(point[1]), float(point[2])))
    last_odom = odoms[-1] if odoms else None
    return points, {
        "ok": bool(points),
        "scan_count": scan_attempts,
        "nonempty_scan_count": len(clouds),
        "point_count": len(points),
        "odom_count": len(odoms),
        "final_pose": (
            {
                "x": float(last_odom.position[0]),
                "y": float(last_odom.position[1]),
                "z": float(last_odom.position[2]),
                "frame_id": "odom",
            }
            if last_odom is not None
            else None
        ),
        "cmd_vx": float(vx),
        "cmd_wz": float(wz),
        "duration_s": float(duration_s),
        "timeout_s": float(timeout_s),
        "publish_hz": float(publish_hz),
        "source": "build_engine.get_lidar_points",
        "lidar_backend": backend_report,
        "requested_backend": str(lidar_backend),
        "mujoco_lidar_backend": str(mujoco_lidar_backend),
        "allow_legacy_lidar_fallback": bool(allow_legacy_lidar_fallback),
        "mid360_pattern": str(mid360_pattern or DEFAULT_MID360_PATTERN),
        "mid360_samples_per_frame": int(mid360_samples_per_frame),
    }

def _finite_triplet(values: list[float], name: str) -> list[float]:
    if len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError(f"{name} must contain exactly three finite numbers")
    return [float(v) for v in values]


def planner_constraint_overrides(args: argparse.Namespace) -> dict[str, Any]:
    overrides: dict[str, Any] = {}
    if args.planner_no_ground_support:
        overrides["require_ground_support"] = False
    if args.planner_clearance_cells >= 0:
        overrides["obstacle_clearance_radius_cells"] = args.planner_clearance_cells
    if args.planner_preblocked_cells >= 0:
        overrides["preblocked_costmap_radius_cells"] = args.planner_preblocked_cells
    if args.planner_robot_radius > 0:
        overrides["robot_radius"] = args.planner_robot_radius
    return overrides


def _same_triplet(values: list[float], expected: list[float]) -> bool:
    if len(values) != 3:
        return False
    return all(abs(float(a) - float(b)) <= 1e-9 for a, b in zip(values, expected))


def effective_start_goal(args: argparse.Namespace) -> tuple[list[float], list[float]]:
    start = _finite_triplet(list(args.start), "start")
    goal = _finite_triplet(list(args.goal), "goal")
    if getattr(args, "scene_preset", "corridor") == "building":
        if _same_triplet(start, CORRIDOR_DEFAULT_START):
            start = list(BUILDING_DEFAULT_START)
        if _same_triplet(goal, CORRIDOR_DEFAULT_GOAL):
            goal = list(BUILDING_DEFAULT_GOAL)
    return start, goal


def run_gate(args: argparse.Namespace) -> dict[str, Any]:
    scene_preset = getattr(args, "scene_preset", "corridor")
    out_dir = Path(args.out_dir or ROOT / "artifacts" / "mujoco_saved_map_plan_gate" / time.strftime("%Y%m%d_%H%M%S"))
    map_dir = out_dir / "same_source_map"
    map_dir.mkdir(parents=True, exist_ok=True)
    scene_xml = out_dir / "scene.xml"
    map_pcd = map_dir / "map.pcd"

    generate_scene_xml(scene_xml, length=args.length, width=args.width, scene_preset=scene_preset)
    scan_report: dict[str, Any] = {"source": args.map_source, "scene_preset": scene_preset}
    if args.map_source == "mujoco_lidar":
        points, scan_report = collect_mujoco_lidar_points(
            scene_xml,
            scans=args.lidar_scans,
            duration_s=args.lidar_duration,
            timeout_s=args.lidar_timeout,
            vx=args.lidar_vx,
            wz=args.lidar_wz,
            publish_hz=getattr(args, "lidar_publish_hz", 10.0),
            n_rays=getattr(args, "n_rays", 6400),
            mid360_pattern=getattr(args, "mid360_pattern", None),
            mid360_samples_per_frame=getattr(args, "mid360_samples_per_frame", 15000),
            lidar_backend=getattr(args, "lidar_backend", "mujoco_lidar"),
            mujoco_lidar_backend=getattr(args, "mujoco_lidar_backend", "cpu"),
            allow_legacy_lidar_fallback=getattr(args, "allow_legacy_lidar_fallback", False),
            mujoco_memory=getattr(args, "mujoco_memory", "64M"),
        )
        raw_point_count = len(points)
        clip: dict[str, Any] = {"length": args.length, "width": args.width}
        if scene_preset == "building":
            clip.update(building_scene_clip())
        points = filter_points_to_scene(points, **clip)
        scan_report["raw_point_count"] = raw_point_count
        scan_report["clipped_point_count"] = len(points)
        scan_report["scene_preset"] = scene_preset
        scan_report["scene_clip"] = clip
    elif scene_preset == "building":
        points = generate_building_points(
            spacing=args.spacing,
            hits_per_cell=args.hits_per_cell,
        )
    else:
        points = generate_points(
            length=args.length,
            width=args.width,
            spacing=args.spacing,
            hits_per_cell=args.hits_per_cell,
        )
    write_ascii_pcd(map_pcd, points)

    builder = MapArtifactBuilder(
        MapArtifactBuilderConfig(
            converter_command=args.converter or None,
            use_env_converter=not args.no_env_converter,
            resolution=args.resolution,
            frame_id="map",
            source_profile=f"mujoco_{scene_preset}_saved_map_gate",
            data_source="mujoco",
            slam_source="mujoco_synthetic_map",
            localization_source="mujoco_synthetic_scan",
            mapping_source=f"mujoco_{scene_preset}_saved_map_plan_gate",
            timeout_sec=args.converter_timeout,
        )
    )
    build = builder.build_for_saved_map(map_dir).to_dict()
    artifact_gate = validate_saved_map_artifact_dir(
        map_dir,
        require_octomap=True,
        expected_frame_id="map",
        expected_data_source="mujoco",
    )

    plan: dict[str, Any] = {"skipped": bool(args.skip_plan)}
    if artifact_gate.get("ok") is True and not args.skip_plan:
        planner = OctoPlanner3DPlanner(
            tomogram_path=str(map_dir / "octomap.ot"),
            executable_path=args.planner_executable or None,
            timeout_s=args.planner_timeout,
        )
        planner.configure_constraints(planner_constraint_overrides(args))
        start, goal = effective_start_goal(args)
        path = planner.plan(start, goal)
        plan = {
            "ok": bool(path) and bool(planner._last_plan_reached_goal),
            "available": planner.available,
            "path_count": len(path),
            "reached_goal": bool(planner._last_plan_reached_goal),
            "error": planner._last_plan_error,
            "diagnostics": planner._last_plan_diagnostics,
            "constraint_overrides": planner_constraint_overrides(args),
        }

    ok = build.get("ok") is True and artifact_gate.get("ok") is True and (
        bool(args.skip_plan) or plan.get("ok") is True
    )
    return {
        "schema_version": "lingtu.mujoco_saved_map_plan_gate.v1",
        "ok": ok,
        "out_dir": str(out_dir),
        "scene_preset": scene_preset,
        "scene_xml": str(scene_xml),
        "map_dir": str(map_dir),
        "map_pcd": str(map_pcd),
        "point_count": len(points),
        "start": effective_start_goal(args)[0],
        "goal": effective_start_goal(args)[1],
        "scan": scan_report,
        "build": build,
        "artifact_gate": artifact_gate,
        "plan": plan,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
    }


def build_parser() -> argparse.ArgumentParser:
    from drivers.sim.mujoco.runtime import (
        DEFAULT_MID360_PATTERN,
        DEFAULT_MID360_SAMPLES_PER_FRAME,
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default="")
    parser.add_argument(
        "--scene-preset",
        choices=("corridor", "building"),
        default="corridor",
        help="corridor keeps the tiny gate map; building reuses sim/worlds/mujoco/building_scene.xml",
    )
    parser.add_argument("--length", type=float, default=3.0)
    parser.add_argument("--width", type=float, default=1.8)
    parser.add_argument("--spacing", type=float, default=0.2)
    parser.add_argument("--hits-per-cell", type=int, default=4)
    parser.add_argument(
        "--map-source",
        choices=("synthetic_hits", "mujoco_lidar"),
        default="synthetic_hits",
    )
    parser.add_argument("--lidar-scans", type=int, default=3)
    parser.add_argument("--lidar-duration", type=float, default=3.0)
    parser.add_argument("--lidar-timeout", type=float, default=8.0)
    parser.add_argument("--lidar-vx", type=float, default=0.2)
    parser.add_argument("--lidar-wz", type=float, default=0.0)
    parser.add_argument("--lidar-publish-hz", type=float, default=10.0)
    parser.add_argument("--n-rays", type=int, default=6400)
    parser.add_argument("--mujoco-memory", default="64M")
    parser.add_argument("--mid360-pattern", type=Path, default=DEFAULT_MID360_PATTERN)
    parser.add_argument("--mid360-samples-per-frame", type=int, default=DEFAULT_MID360_SAMPLES_PER_FRAME)
    parser.add_argument("--lidar-backend", choices=("mujoco_lidar", "ray_caster_lidar"), default="mujoco_lidar")
    parser.add_argument("--mujoco-lidar-backend", choices=("cpu", "taichi", "warp", "jax"), default="cpu")
    parser.add_argument("--allow-legacy-lidar-fallback", action="store_true")
    parser.add_argument("--resolution", type=float, default=0.2)
    parser.add_argument("--converter", default="")
    parser.add_argument("--no-env-converter", action="store_true")
    parser.add_argument("--converter-timeout", type=float, default=60.0)
    parser.add_argument("--planner-executable", default="")
    parser.add_argument("--planner-timeout", type=float, default=30.0)
    parser.add_argument("--planner-no-ground-support", action="store_true")
    parser.add_argument("--planner-clearance-cells", type=int, default=-1)
    parser.add_argument("--planner-preblocked-cells", type=int, default=-1)
    parser.add_argument("--planner-robot-radius", type=float, default=0.0)
    parser.add_argument("--start", nargs=3, type=float, default=CORRIDOR_DEFAULT_START)
    parser.add_argument("--goal", nargs=3, type=float, default=CORRIDOR_DEFAULT_GOAL)
    parser.add_argument("--skip-plan", action="store_true")
    parser.add_argument("--json-out", default="")
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    report = run_gate(args)
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    return 0 if report["ok"] or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
