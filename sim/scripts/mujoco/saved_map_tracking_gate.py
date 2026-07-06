#!/usr/bin/env python3
"""Validate saved-map global planning and MuJoCo path tracking without ROS."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from runtime.msgs.numpy_compat import np  # noqa: E402
from nav.services.plan.global_planner.algorithm.octoplanner3d_planner import (  # noqa: E402
    OctoPlanner3DPlanner,
)
from sim.scripts.mujoco import saved_map_plan_gate as plan_gate  # noqa: E402

PATH_LIBRARY = ROOT / "src" / "nav" / "services" / "plan" / "local_planner" / "paths"


def _wrap_angle_rad(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_quat_xyzw(quat: Any) -> float:
    values = np.asarray(quat, dtype=float).reshape(-1)
    if values.size < 4:
        return 0.0
    x, y, z, w = [float(v) for v in values[:4]]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _distance2d(a: list[float] | tuple[float, ...], b: list[float] | tuple[float, ...]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _map_to_body(
    point_map: list[float],
    position_map: list[float],
    yaw: float,
) -> list[float]:
    dx = float(point_map[0]) - float(position_map[0])
    dy = float(point_map[1]) - float(position_map[1])
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    return [
        c * dx + s * dy,
        -s * dx + c * dy,
        float(point_map[2]) - float(position_map[2]),
    ]


def _select_target_index(
    path: list[list[float]],
    position_map: list[float],
    *,
    cursor: int,
    lookahead_m: float,
    waypoint_reached_m: float,
) -> int:
    if not path:
        return 0
    start = min(max(0, int(cursor)), len(path) - 1)
    nearest = start
    nearest_distance = _distance2d(path[start], position_map)
    search_end = min(len(path), start + 12)
    for idx in range(start + 1, search_end):
        distance = _distance2d(path[idx], position_map)
        if distance < nearest_distance:
            nearest = idx
            nearest_distance = distance
    while nearest + 1 < len(path) and _distance2d(path[nearest], position_map) < waypoint_reached_m:
        nearest += 1
    for idx in range(nearest, len(path)):
        if _distance2d(path[idx], position_map) >= lookahead_m:
            return idx
    return len(path) - 1


def _as_vec3(nav_kernel: Any, point: list[float] | tuple[float, ...]) -> Any:
    return nav_kernel.Vec3(float(point[0]), float(point[1]), float(point[2]))


def _build_local_planner(nav_kernel: Any, args: argparse.Namespace) -> Any:
    params = nav_kernel.LocalPlannerParams()
    params.check_obstacle = bool(args.check_obstacle)
    params.use_terrain_analysis = bool(args.check_obstacle)
    params.use_traversability_cost = False
    params.autonomy_speed = float(args.max_speed)
    params.max_speed = max(float(args.max_speed), 0.01)
    params.vehicle_length = float(args.vehicle_length_m)
    params.vehicle_width = float(args.vehicle_width_m)
    params.sensor_offset_x = float(args.sensor_offset_x_m)
    params.sensor_offset_y = float(args.sensor_offset_y_m)
    planner = nav_kernel.LocalPlanner(params)
    if not planner.load_paths(str(args.path_library_dir)):
        raise RuntimeError(f"failed to load local planner path library: {args.path_library_dir}")
    return planner


def _build_path_follower(nav_kernel: Any, args: argparse.Namespace) -> tuple[Any, Any]:
    params = nav_kernel.PathFollowerParams()
    params.max_speed = float(args.max_speed)
    params.max_accel = max(0.01, float(args.max_accel))
    params.stop_dis_thre = max(0.01, float(args.goal_tolerance_m))
    min_lookahead = max(0.2, params.stop_dis_thre + 0.05)
    params.base_look_ahead_dis = max(float(args.local_lookahead_m) * 0.2, min_lookahead)
    params.min_look_ahead_dis = min_lookahead
    params.max_look_ahead_dis = max(min(float(args.local_lookahead_m), 2.0), min_lookahead)
    params.look_ahead_ratio = 0.5
    params.yaw_rate_gain = float(args.yaw_rate_gain)
    params.stop_yaw_rate_gain = float(args.stop_yaw_rate_gain)
    params.max_yaw_rate = math.degrees(max(0.01, float(args.max_yaw_rate)))
    params.switch_time_thre = 1.0
    params.dir_diff_thre = float(args.dir_diff_thre)
    params.omni_dir_goal_thre = 1.0
    params.omni_dir_diff_thre = 1.5
    params.slow_dwn_dis_thre = 1.0
    params.two_way_drive = bool(args.two_way_drive)
    params.no_rot_at_goal = True
    return params, nav_kernel.PathFollowerState()


def _load_pcd_xyz(path: Path, *, limit: int = 20000) -> list[list[float]]:
    if not path.is_file():
        return []
    points: list[list[float]] = []
    in_data = False
    for line in path.read_text(encoding="ascii", errors="ignore").splitlines():
        text = line.strip()
        if not text:
            continue
        if not in_data:
            if text.lower().startswith("data "):
                in_data = True
            continue
        parts = text.split()
        if len(parts) < 3:
            continue
        try:
            point = [float(parts[0]), float(parts[1]), float(parts[2])]
        except ValueError:
            continue
        if all(math.isfinite(v) for v in point):
            points.append(point)
        if len(points) >= limit:
            break
    return points


def _nearby_pcd_obstacles(
    points: list[list[float]],
    position: list[float],
    *,
    radius_m: float,
    max_points: int,
) -> Any:
    if not points or max_points <= 0:
        return np.zeros((0, 4), dtype=np.float32)
    radius2 = float(radius_m) * float(radius_m)
    rows: list[list[float]] = []
    stride = max(1, len(points) // max(max_points * 4, 1))
    for point in points[::stride]:
        dx = float(point[0]) - float(position[0])
        dy = float(point[1]) - float(position[1])
        if dx * dx + dy * dy > radius2:
            continue
        height = float(point[2]) - float(position[2])
        rows.append([float(point[0]), float(point[1]), float(point[2]), height])
        if len(rows) >= max_points:
            break
    return np.asarray(rows, dtype=np.float32)


def _path_length_xy(points: list[list[float]]) -> float:
    total = 0.0
    for prev, cur in zip(points, points[1:]):
        total += _distance2d(prev, cur)
    return total


def _extract_or_replan_global_path(
    plan_report: dict[str, Any],
    args: argparse.Namespace,
) -> tuple[list[list[float]], dict[str, Any]]:
    plan = plan_report.get("plan") or {}
    path = [[float(v) for v in point[:3]] for point in plan.get("path", [])]
    if path:
        return path, {"source": "saved_map_plan_gate_report"}
    map_dir = Path(str(plan_report.get("map_dir") or ""))
    octomap_path = map_dir / "octomap.ot"
    start = plan_report.get("start") or args.start
    goal = plan_report.get("goal") or args.goal
    planner = OctoPlanner3DPlanner(
        tomogram_path=str(octomap_path),
        executable_path=args.planner_executable or None,
        timeout_s=args.planner_timeout,
    )
    planner.configure_constraints(plan_gate.planner_constraint_overrides(args))
    replanned = planner.plan(start, goal)
    json_path = plan_gate.jsonable_path(replanned) if hasattr(plan_gate, "jsonable_path") else [
        [float(v) for v in point[:3]] for point in replanned
    ]
    return json_path, {
        "source": "tracking_gate_replan",
        "ok": bool(json_path) and bool(planner._last_plan_reached_goal),
        "path_count": len(json_path),
        "reached_goal": bool(planner._last_plan_reached_goal),
        "error": planner._last_plan_error,
        "diagnostics": planner._last_plan_diagnostics,
    }


def _write_trajectory_csv(path: Path, samples: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "t",
        "x",
        "y",
        "z",
        "yaw",
        "target_index",
        "target_x",
        "target_y",
        "target_z",
        "cmd_vx",
        "cmd_vy",
        "cmd_wz",
        "local_path_points",
        "reason",
    ]
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for sample in samples:
            writer.writerow({key: sample.get(key, "") for key in fields})


def _svg_polyline(
    points: list[list[float]],
    project: Any,
    *,
    color: str,
    width: float,
    opacity: float = 1.0,
) -> str:
    if len(points) < 2:
        return ""
    text = " ".join(f"{project(p)[0]:.2f},{project(p)[1]:.2f}" for p in points)
    return (
        f'<polyline points="{text}" fill="none" stroke="{color}" '
        f'stroke-width="{width:.2f}" opacity="{opacity:.3f}" '
        'stroke-linecap="round" stroke-linejoin="round"/>'
    )


def _write_preview_svg(
    path: Path,
    *,
    pcd_points: list[list[float]],
    global_path: list[list[float]],
    trajectory: list[list[float]],
    start: list[float],
    goal: list[float],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    all_points = [*pcd_points[:3000], *global_path, *trajectory, start, goal]
    xs = [float(p[0]) for p in all_points] or [0.0, 1.0]
    ys = [float(p[1]) for p in all_points] or [0.0, 1.0]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    if abs(max_x - min_x) < 1e-6:
        max_x += 1.0
        min_x -= 1.0
    if abs(max_y - min_y) < 1e-6:
        max_y += 1.0
        min_y -= 1.0
    width, height, margin = 1000.0, 700.0, 40.0
    scale = min((width - 2.0 * margin) / (max_x - min_x), (height - 2.0 * margin) / (max_y - min_y))

    def project(point: list[float]) -> tuple[float, float]:
        x = margin + (float(point[0]) - min_x) * scale
        y = height - margin - (float(point[1]) - min_y) * scale
        return x, y

    pcd_step = max(1, len(pcd_points) // 2500)
    pcd_svg = []
    for point in pcd_points[::pcd_step][:2500]:
        x, y = project(point)
        pcd_svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="1.1" fill="#9aa0a6" opacity="0.45"/>')
    sx, sy = project(start)
    gx, gy = project(goal)
    body = "\n  ".join(
        [
            *pcd_svg,
            _svg_polyline(global_path, project, color="#1a73e8", width=3.0, opacity=0.95),
            _svg_polyline(trajectory, project, color="#d93025", width=2.4, opacity=0.9),
            f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="6" fill="#188038"/>',
            f'<circle cx="{gx:.2f}" cy="{gy:.2f}" r="6" fill="#202124"/>',
            '<text x="48" y="36" font-family="Arial" font-size="16" fill="#202124">'
            "gray=PCD, blue=global_path, red=MuJoCo trajectory</text>",
        ]
    )
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" '
        f'viewBox="0 0 {width:.0f} {height:.0f}">\n'
        '<rect width="100%" height="100%" fill="#ffffff"/>\n  '
        f"{body}\n</svg>\n"
    )
    path.write_text(svg, encoding="utf-8")


def _plan_args(args: argparse.Namespace, out_dir: Path) -> argparse.Namespace:
    return SimpleNamespace(
        out_dir=str(out_dir),
        scene_preset=args.scene_preset,
        length=args.length,
        width=args.width,
        spacing=args.spacing,
        hits_per_cell=args.hits_per_cell,
        map_source=args.map_source,
        lidar_scans=args.lidar_scans,
        lidar_duration=args.lidar_duration,
        lidar_timeout=args.lidar_timeout,
        lidar_vx=args.lidar_vx,
        lidar_wz=args.lidar_wz,
        lidar_publish_hz=args.lidar_publish_hz,
        n_rays=args.n_rays,
        mujoco_memory=args.mujoco_memory,
        mid360_pattern=args.mid360_pattern,
        mid360_samples_per_frame=args.mid360_samples_per_frame,
        lidar_backend=args.lidar_backend,
        mujoco_lidar_backend=args.mujoco_lidar_backend,
        allow_legacy_lidar_fallback=args.allow_legacy_lidar_fallback,
        resolution=args.resolution,
        converter=args.converter,
        no_env_converter=args.no_env_converter,
        converter_timeout=args.converter_timeout,
        planner_executable=args.planner_executable,
        planner_timeout=args.planner_timeout,
        planner_no_ground_support=args.planner_no_ground_support,
        planner_clearance_cells=args.planner_clearance_cells,
        planner_preblocked_cells=args.planner_preblocked_cells,
        planner_robot_radius=args.planner_robot_radius,
        start=args.start,
        goal=args.goal,
        skip_plan=False,
    )


def run_tracking_gate(args: argparse.Namespace) -> dict[str, Any]:
    out_dir = Path(args.out_dir or ROOT / "artifacts" / "mujoco_saved_map_tracking_gate" / time.strftime("%Y%m%d_%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=True)
    plan_dir = out_dir / "plan"
    plan_report = plan_gate.run_gate(_plan_args(args, plan_dir))
    plan = plan_report.get("plan") or {}
    global_path, path_source = _extract_or_replan_global_path(plan_report, args)
    if plan_report.get("ok") is not True or not global_path:
        report = {
            "schema_version": "lingtu.mujoco_saved_map_tracking_gate.v1",
            "ok": False,
            "stage": "global_plan",
            "simulation_only": True,
            "uses_ros": False,
            "out_dir": str(out_dir),
            "plan_report": plan_report,
            "path_source": path_source,
            "blockers": ["global_plan_failed_or_empty"],
        }
        _write_report(args, out_dir, report)
        return report

    from drivers.sim.mujoco.runtime import build_engine
    from sim.engine.core.engine import VelocityCommand

    try:
        import lingtu_nav_kernel as nav_kernel
    except Exception as exc:
        report = {
            "schema_version": "lingtu.mujoco_saved_map_tracking_gate.v1",
            "ok": False,
            "stage": "nav_kernel_import",
            "simulation_only": True,
            "uses_ros": False,
            "out_dir": str(out_dir),
            "plan_report": plan_report,
            "blockers": [f"lingtu_nav_kernel import failed: {type(exc).__name__}: {exc}"],
        }
        _write_report(args, out_dir, report)
        return report

    local_planner = _build_local_planner(nav_kernel, args)
    follower_params, follower_state = _build_path_follower(nav_kernel, args)
    pcd_points = _load_pcd_xyz(Path(plan_report["map_pcd"]), limit=args.preview_pcd_points)

    start = [float(v) for v in plan_report["start"][:3]]
    goal = [float(v) for v in plan_report["goal"][:3]]
    mujoco_start = [
        float(start[0]),
        float(start[1]),
        float(args.mujoco_start_z if args.mujoco_start_z is not None else max(start[2], 0.55)),
    ]
    engine = build_engine(
        world=Path(plan_report["scene_xml"]),
        drive_mode="kinematic",
        n_rays=int(args.n_rays),
        start=mujoco_start,
        mujoco_memory=str(args.mujoco_memory),
        mid360_pattern=None if args.allow_golden_spiral_lidar else args.mid360_pattern,
        mid360_samples_per_frame=int(args.mid360_samples_per_frame),
        lidar_backend=str(args.lidar_backend),
        mujoco_lidar_backend=str(args.mujoco_lidar_backend),
        require_product_lidar_backend=not bool(args.allow_legacy_lidar_fallback),
        allow_legacy_lidar_fallback=bool(args.allow_legacy_lidar_fallback),
    )

    cursor = 0
    samples: list[dict[str, Any]] = []
    trajectory: list[list[float]] = []
    local_path_points_total = 0
    nonzero_cmd_count = 0
    near_field_stop_count = 0
    last_reason = "not_started"
    arrived = False
    timeout = False
    cmd = VelocityCommand()
    start_wall = time.monotonic()
    start_sim = float(getattr(engine, "sim_time", 0.0))
    next_tick = start_sim

    try:
        while True:
            now_wall = time.monotonic()
            sim_time = float(getattr(engine, "sim_time", 0.0))
            elapsed_sim = sim_time - start_sim
            if elapsed_sim >= float(args.duration):
                timeout = True
                break
            if float(args.max_wall_time_s) > 0.0 and now_wall - start_wall >= float(args.max_wall_time_s):
                timeout = True
                break

            state = engine.get_robot_state()
            position = [float(state.position[0]), float(state.position[1]), float(state.position[2])]
            yaw = _yaw_from_quat_xyzw(state.orientation)
            trajectory.append(position)
            if _distance2d(position, goal) <= float(args.goal_tolerance_m):
                arrived = True
                last_reason = "goal_reached"
                break

            if sim_time + 1e-9 >= next_tick:
                cursor = _select_target_index(
                    global_path,
                    position,
                    cursor=cursor,
                    lookahead_m=float(args.global_lookahead_m),
                    waypoint_reached_m=float(args.waypoint_reached_m),
                )
                target = global_path[cursor]
                obstacles = (
                    _nearby_pcd_obstacles(
                        pcd_points,
                        position,
                        radius_m=float(args.obstacle_radius_m),
                        max_points=int(args.max_obstacle_points),
                    )
                    if args.check_obstacle
                    else np.zeros((0, 4), dtype=np.float32)
                )
                local = local_planner.plan_frame_without_grid(
                    position[0],
                    position[1],
                    position[2],
                    yaw,
                    float(target[0]),
                    float(target[1]),
                    obstacles,
                    sim_time,
                )
                local_path = list(local.path)
                if local.near_field_stop:
                    near_field_stop_count += 1
                    cmd = VelocityCommand()
                    last_reason = "near_field_stop"
                else:
                    if len(local_path) < 2:
                        target_body = _map_to_body(target, position, yaw)
                        if _distance2d(target_body, [0.0, 0.0, 0.0]) > 0.05:
                            local_path = [
                                nav_kernel.Vec3(0.0, 0.0, 0.0),
                                _as_vec3(nav_kernel, target_body),
                            ]
                    if len(local_path) >= 2:
                        control = nav_kernel.compute_control(
                            nav_kernel.Vec3(0.0, 0.0, 0.0),
                            0.0,
                            local_path,
                            1.0,
                            sim_time,
                            1.0,
                            0,
                            follower_params,
                            follower_state,
                        )
                        cmd = VelocityCommand(
                            linear_x=float(control.cmd.vx),
                            linear_y=float(control.cmd.vy),
                            angular_z=float(control.cmd.wz),
                        )
                        last_reason = "control_ready" if bool(local.path_found) else "fallback_tracking"
                    else:
                        cmd = VelocityCommand()
                        last_reason = "no_local_path"
                local_path_points_total += len(local_path)
                if abs(cmd.linear_x) + abs(cmd.linear_y) + abs(cmd.angular_z) > 1e-4:
                    nonzero_cmd_count += 1
                samples.append(
                    {
                        "t": round(float(elapsed_sim), 4),
                        "x": round(position[0], 5),
                        "y": round(position[1], 5),
                        "z": round(position[2], 5),
                        "yaw": round(float(yaw), 5),
                        "target_index": int(cursor),
                        "target_x": round(float(target[0]), 5),
                        "target_y": round(float(target[1]), 5),
                        "target_z": round(float(target[2]), 5),
                        "cmd_vx": round(float(cmd.linear_x), 5),
                        "cmd_vy": round(float(cmd.linear_y), 5),
                        "cmd_wz": round(float(cmd.angular_z), 5),
                        "local_path_points": int(len(local_path)),
                        "reason": last_reason,
                    }
                )
                next_tick = sim_time + 1.0 / max(1e-6, float(args.tick_hz))

            engine.step(cmd)
    finally:
        try:
            engine.step(VelocityCommand())
            engine.close()
        except Exception:
            pass

    trajectory_csv = out_dir / "trajectory.csv"
    preview_svg = out_dir / "preview.svg"
    _write_trajectory_csv(trajectory_csv, samples)
    _write_preview_svg(
        preview_svg,
        pcd_points=pcd_points,
        global_path=global_path,
        trajectory=trajectory,
        start=start,
        goal=goal,
    )

    final_position = trajectory[-1] if trajectory else mujoco_start
    final_error_m = _distance2d(final_position, goal)
    ok = (
        plan_report.get("ok") is True
        and bool(global_path)
        and nonzero_cmd_count > 0
        and len(trajectory) >= 2
        and (arrived or final_error_m <= float(args.acceptance_radius_m))
    )
    report = {
        "schema_version": "lingtu.mujoco_saved_map_tracking_gate.v1",
        "ok": bool(ok),
        "stage": "tracking",
        "simulation_only": True,
        "uses_ros": False,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "out_dir": str(out_dir),
        "scene_preset": args.scene_preset,
        "dataflow": [
            "map.pcd",
            "octomap.ot",
            "OctoPlanner3D global_path",
            "lingtu_nav_kernel LocalPlanner",
            "lingtu_nav_kernel PathFollower",
            "MuJoCo kinematic robot",
        ],
        "limitations": (
            ["building_scene_tracking_is_flat_kinematic"]
            if args.scene_preset == "building"
            else []
        ),
        "artifacts": {
            "map_pcd": plan_report.get("map_pcd"),
            "map_dir": plan_report.get("map_dir"),
            "scene_xml": plan_report.get("scene_xml"),
            "trajectory_csv": str(trajectory_csv),
            "preview_svg": str(preview_svg),
        },
        "plan": {
            "ok": bool(plan.get("ok")),
            "path_source": path_source,
            "path_count": int(len(global_path)),
            "path_length_m": round(_path_length_xy(global_path), 4),
            "reached_goal": bool(plan.get("reached_goal")),
            "diagnostics": plan.get("diagnostics", {}),
        },
        "tracking": {
            "arrived": bool(arrived),
            "timeout": bool(timeout),
            "samples": int(len(samples)),
            "trajectory_points": int(len(trajectory)),
            "trajectory_length_m": round(_path_length_xy(trajectory), 4),
            "final_error_m": round(float(final_error_m), 4),
            "nonzero_cmd_count": int(nonzero_cmd_count),
            "near_field_stop_count": int(near_field_stop_count),
            "local_path_points_avg": (
                round(float(local_path_points_total / max(1, len(samples))), 2)
                if samples
                else 0.0
            ),
            "last_reason": last_reason,
            "tick_hz": float(args.tick_hz),
            "max_speed": float(args.max_speed),
            "check_obstacle": bool(args.check_obstacle),
        },
        "plan_report": plan_report,
    }
    _write_report(args, out_dir, report)
    return report


def _write_report(args: argparse.Namespace, out_dir: Path, report: dict[str, Any]) -> None:
    text = json.dumps(report, indent=2, sort_keys=True)
    report_path = Path(args.json_out) if args.json_out else out_dir / "report.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(text + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    from drivers.sim.mujoco.runtime import (
        DEFAULT_MID360_PATTERN,
        DEFAULT_MID360_SAMPLES_PER_FRAME,
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default="")
    parser.add_argument("--scene-preset", choices=("corridor", "building"), default="corridor")
    parser.add_argument("--length", type=float, default=3.0)
    parser.add_argument("--width", type=float, default=1.8)
    parser.add_argument("--spacing", type=float, default=0.2)
    parser.add_argument("--hits-per-cell", type=int, default=4)
    parser.add_argument("--map-source", choices=("synthetic_hits", "mujoco_lidar"), default="synthetic_hits")
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
    parser.add_argument("--allow-golden-spiral-lidar", action="store_true")
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
    parser.add_argument("--start", nargs=3, type=float, default=plan_gate.CORRIDOR_DEFAULT_START)
    parser.add_argument("--goal", nargs=3, type=float, default=plan_gate.CORRIDOR_DEFAULT_GOAL)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--max-wall-time-s", type=float, default=90.0)
    parser.add_argument("--tick-hz", type=float, default=20.0)
    parser.add_argument("--global-lookahead-m", type=float, default=1.2)
    parser.add_argument("--local-lookahead-m", type=float, default=1.5)
    parser.add_argument("--waypoint-reached-m", type=float, default=0.45)
    parser.add_argument("--goal-tolerance-m", type=float, default=0.25)
    parser.add_argument("--acceptance-radius-m", type=float, default=0.45)
    parser.add_argument("--max-speed", type=float, default=0.35)
    parser.add_argument("--max-accel", type=float, default=1.0)
    parser.add_argument("--max-yaw-rate", type=float, default=0.8)
    parser.add_argument("--yaw-rate-gain", type=float, default=7.5)
    parser.add_argument("--stop-yaw-rate-gain", type=float, default=7.5)
    parser.add_argument("--dir-diff-thre", type=float, default=0.1)
    parser.add_argument("--two-way-drive", action="store_true")
    parser.add_argument("--check-obstacle", action="store_true")
    parser.add_argument("--obstacle-radius-m", type=float, default=5.0)
    parser.add_argument("--max-obstacle-points", type=int, default=2500)
    parser.add_argument("--vehicle-length-m", type=float, default=0.8)
    parser.add_argument("--vehicle-width-m", type=float, default=0.45)
    parser.add_argument("--sensor-offset-x-m", type=float, default=0.0)
    parser.add_argument("--sensor-offset-y-m", type=float, default=0.0)
    parser.add_argument("--mujoco-start-z", type=float, default=None)
    parser.add_argument("--path-library-dir", type=Path, default=PATH_LIBRARY)
    parser.add_argument("--preview-pcd-points", type=int, default=20000)
    parser.add_argument("--json-out", default="")
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    report = run_tracking_gate(args)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report.get("ok") is True or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
