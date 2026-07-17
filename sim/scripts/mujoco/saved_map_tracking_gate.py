#!/usr/bin/env python3
"""Validate saved-map global planning and MuJoCo path tracking without ROS."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sim.scripts.mujoco import saved_map_plan_gate as plan_gate

from nav.kernel.loader import require_nav_kernel
from nav.services.plan.global_planner.algorithm.octoplanner3d_planner import (
    OctoPlanner3DPlanner,
)
from runtime.msgs.numpy_compat import np

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
    if hasattr(params, "use_traversability_cost"):
        params.use_traversability_cost = bool(args.use_traversability_grid)
    if hasattr(params, "traversability_near_field_stop"):
        params.traversability_near_field_stop = bool(args.traversability_near_field_stop)
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


def _make_scene_traversability_grid(scene_preset: str) -> dict[str, Any]:
    if scene_preset not in plan_gate.MULTILEVEL_SCENE_PRESETS:
        return {"ok": False, "reason": "scene_has_no_builtin_traversability_grid"}

    resolution = 0.20
    clip = plan_gate.stairs3d_scene_clip(margin=0.6, scene_preset=scene_preset)
    origin_x, origin_y = float(clip["x_min"]), float(clip["y_min"])
    cols = max(1, int(math.ceil((float(clip["x_max"]) - origin_x) / resolution)))
    rows = max(1, int(math.ceil((float(clip["y_max"]) - origin_y) / resolution)))
    grid = np.full((rows, cols), 100.0, dtype=np.float32)

    def mark_rect(cx: float, cy: float, hx: float, hy: float, risk: float) -> None:
        x0 = int(max(0, math.floor((cx - hx - origin_x) / resolution)))
        x1 = int(min(cols - 1, math.ceil((cx + hx - origin_x) / resolution)))
        y0 = int(max(0, math.floor((cy - hy - origin_y) / resolution)))
        y1 = int(min(rows - 1, math.ceil((cy + hy - origin_y) / resolution)))
        grid[y0 : y1 + 1, x0 : x1 + 1] = np.minimum(grid[y0 : y1 + 1, x0 : x1 + 1], risk)

    def mark_segment(ax: float, ay: float, bx: float, by: float, width: float, risk: float) -> None:
        xs = origin_x + np.arange(cols, dtype=np.float32) * resolution
        ys = origin_y + np.arange(rows, dtype=np.float32) * resolution
        xx, yy = np.meshgrid(xs, ys)
        vx, vy = bx - ax, by - ay
        denom = max(vx * vx + vy * vy, 1e-9)
        t = np.clip(((xx - ax) * vx + (yy - ay) * vy) / denom, 0.0, 1.0)
        px = ax + t * vx
        py = ay + t * vy
        mask = (xx - px) ** 2 + (yy - py) ** 2 <= (width * 0.5) ** 2
        grid[mask] = np.minimum(grid[mask], risk)

    step_rise = float(plan_gate.stair_scene_profile(scene_preset)["rise_m"])
    for geom in plan_gate._stairs3d_geoms(scene_preset):
        name = str(geom.get("name", ""))
        if geom.get("type") != "box":
            if name.startswith("obs_") or name.startswith("wall_"):
                radius = float(geom["size"][0])
                mark_rect(float(geom["pos"][0]), float(geom["pos"][1]), radius + 0.24, radius + 0.24, 95.0)
            continue
        cx, cy = float(geom["pos"][0]), float(geom["pos"][1])
        hx, hy = float(geom["size"][0]), float(geom["size"][1])
        if name.startswith("floor_"):
            mark_rect(cx, cy, hx, hy, 0.0)
        elif name.startswith("step_") or name.startswith("riser_"):
            risk = 92.0 if step_rise > plan_gate.STAIRS3D_STEP_MAX_M else 28.0 + max(0.0, step_rise - 0.15) * 160.0
            mark_rect(cx, cy, hx, hy, risk)
        elif name.startswith("rough_patch_"):
            mark_rect(cx, cy, hx + 0.05, hy + 0.05, 65.0)
        elif name.startswith("obs_") or name.startswith("wall_"):
            mark_rect(cx, cy, hx + 0.12, hy + 0.12, 95.0)
    return {
        "ok": True,
        "source": f"builtin_{scene_preset}_traversability_grid",
        "grid": grid,
        "rows": rows,
        "cols": cols,
        "resolution": resolution,
        "origin": [origin_x, origin_y],
        "risk_semantics": {
            "0": "flat traversable floor",
            "28-70": "stair or rough terrain traversable with penalty",
            "92": "step exceeds product max height and should be rejected",
            "95": "obstacle",
            "100": "unknown/outside walkable corridor",
        },
    }


def _sync_scene_traversability_grid(local_planner: Any, args: argparse.Namespace) -> dict[str, Any]:
    if not bool(args.use_traversability_grid):
        return {"enabled": False}
    payload = _make_scene_traversability_grid(str(args.scene_preset))
    if payload.get("ok") is not True:
        return {"enabled": True, "ok": False, "reason": payload.get("reason", "grid_unavailable")}
    if not hasattr(local_planner, "set_traversability_grid"):
        return {"enabled": True, "ok": False, "reason": "local_planner_missing_set_traversability_grid"}
    grid = np.ascontiguousarray(payload["grid"], dtype=np.float32)
    try:
        local_planner.set_traversability_grid(
            grid,
            float(payload["resolution"]),
            float(payload["origin"][0]),
            float(payload["origin"][1]),
        )
    except TypeError:
        local_planner.set_traversability_grid(
            grid,
            int(payload["rows"]),
            int(payload["cols"]),
            float(payload["resolution"]),
            float(payload["origin"][0]),
            float(payload["origin"][1]),
        )
    return {
        "enabled": True,
        "ok": True,
        "source": payload["source"],
        "rows": int(payload["rows"]),
        "cols": int(payload["cols"]),
        "resolution": float(payload["resolution"]),
        "origin": payload["origin"],
        "risk_semantics": payload["risk_semantics"],
    }


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


def _inject_walking_person(scene_xml: Path, out_dir: Path, args: argparse.Namespace) -> tuple[Path, dict[str, Any]]:
    if not bool(args.dynamic_person):
        return scene_xml, {"enabled": False}
    tree = ET.parse(scene_xml)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        return scene_xml, {"enabled": True, "ok": False, "reason": "worldbody_missing"}
    start = [float(v) for v in args.person_start[:3]]
    person = ET.Element("body", {"name": "walking_person", "mocap": "true", "pos": f"{start[0]} {start[1]} {start[2]}"})
    ET.SubElement(
        person,
        "geom",
        {
            "name": "walking_person_body",
            "type": "capsule",
            "fromto": "0 0 0.05 0 0 1.25",
            "size": "0.20",
            "rgba": "1 0.12 0.08 0.88",
            "contype": "1",
            "conaffinity": "1",
            "group": "1",
        },
    )
    ET.SubElement(
        person,
        "geom",
        {
            "name": "walking_person_head",
            "type": "sphere",
            "pos": "0 0 1.48",
            "size": "0.18",
            "rgba": "1 0.22 0.12 0.92",
            "contype": "1",
            "conaffinity": "1",
            "group": "1",
        },
    )
    worldbody.append(person)
    dynamic_scene = out_dir / "scene_with_walking_person.xml"
    tree.write(dynamic_scene, encoding="utf-8", xml_declaration=False)
    return dynamic_scene, {
        "enabled": True,
        "ok": True,
        "scene_xml": str(dynamic_scene),
        "person_start": start,
        "person_end": [float(v) for v in args.person_end[:3]],
        "person_speed_mps": float(args.person_speed_mps),
    }


def _update_walking_person(engine: Any, args: argparse.Namespace, elapsed_s: float) -> list[float] | None:
    if not bool(args.dynamic_person):
        return None
    try:
        import mujoco

        body_id = mujoco.mj_name2id(engine.model, mujoco.mjtObj.mjOBJ_BODY, "walking_person")
        if body_id < 0:
            return None
        mocap_id = int(engine.model.body_mocapid[body_id])
        if mocap_id < 0:
            return None
        a = np.asarray(args.person_start[:3], dtype=np.float64)
        b = np.asarray(args.person_end[:3], dtype=np.float64)
        distance = float(np.linalg.norm(b[:2] - a[:2]))
        period = max(1.0, 2.0 * distance / max(0.05, float(args.person_speed_mps)))
        phase = (float(elapsed_s) % period) / period
        u = phase * 2.0 if phase <= 0.5 else 2.0 - phase * 2.0
        pos = a + (b - a) * float(u)
        engine.data.mocap_pos[mocap_id][:] = pos
        engine.data.mocap_quat[mocap_id][:] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(engine.model, engine.data)
        return [float(pos[0]), float(pos[1]), float(pos[2])]
    except Exception:
        return None


def _trajectory_clearance_to_obstacles(
    trajectory: list[list[float]],
    obstacles: list[list[float]],
    *,
    min_body_z_offset: float = -0.10,
    max_body_z_offset: float = 0.85,
) -> dict[str, Any]:
    if not trajectory or not obstacles:
        return {"ok": False, "reason": "trajectory_or_obstacles_empty"}
    obstacle_arr = np.asarray(obstacles, dtype=np.float32)
    min_distance = float("inf")
    min_sample: dict[str, Any] | None = None
    checked = 0
    for idx, pose in enumerate(trajectory):
        p = np.asarray(pose[:3], dtype=np.float32)
        dz = obstacle_arr[:, 2] - p[2]
        mask = (dz >= float(min_body_z_offset)) & (dz <= float(max_body_z_offset))
        if not np.any(mask):
            continue
        checked += int(np.count_nonzero(mask))
        subset = obstacle_arr[mask]
        dxy = np.linalg.norm(subset[:, :2] - p[:2], axis=1)
        local_min_idx = int(np.argmin(dxy))
        local_min = float(dxy[local_min_idx])
        if local_min < min_distance:
            obstacle = subset[local_min_idx]
            min_distance = local_min
            min_sample = {
                "trajectory_index": int(idx),
                "robot": [float(pose[0]), float(pose[1]), float(pose[2])],
                "obstacle": [float(obstacle[0]), float(obstacle[1]), float(obstacle[2])],
            }
    if not math.isfinite(min_distance):
        return {"ok": False, "reason": "no_same_body_height_obstacles", "checked_points": checked}
    return {
        "ok": True,
        "min_xy_clearance_m": round(float(min_distance), 4),
        "checked_points": checked,
        "min_sample": min_sample,
    }


def _load_xyz_points(path: Path, *, limit: int = 50000) -> list[list[float]]:
    if not path.is_file():
        return []
    points: list[list[float]] = []
    for line in path.read_text(encoding="ascii", errors="ignore").splitlines():
        text = line.strip()
        if not text or text.startswith("#"):
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


def _resolve_repo_path(path: Any) -> Path:
    candidate = Path(str(path))
    if candidate.is_absolute():
        return candidate
    return ROOT / candidate


def _dump_octomap_occupied_points(
    octomap_path: Path,
    out_dir: Path,
    *,
    load_limit: int,
) -> tuple[list[list[float]], dict[str, Any]]:
    output = out_dir / "octomap_occupied.xyz"
    info: dict[str, Any] = {
        "source": "octomap_occupied",
        "path": str(output),
        "enabled": False,
        "ok": False,
        "points": 0,
    }
    exe = ROOT / "build" / "octoplanner3d_headless" / "octoplanner3d_dump_octomap"
    octomap_path = _resolve_repo_path(octomap_path)
    if not octomap_path.is_file():
        info["error"] = f"octomap_missing: {octomap_path}"
        return [], info
    if not exe.is_file():
        info["error"] = f"dump_tool_missing: {exe}"
        return [], info

    output.parent.mkdir(parents=True, exist_ok=True)
    if os.name == "nt":
        launcher = shutil.which("wsl.exe") or shutil.which("wsl")
        if not launcher:
            info["error"] = "wsl_launcher_missing"
            return [], info
        root_wsl = plan_gate._wsl_path(ROOT)
        exe_wsl = plan_gate._wsl_path(exe)
        input_wsl = plan_gate._wsl_path(octomap_path)
        output_wsl = plan_gate._wsl_path(output)
        command = [
            launcher,
            "bash",
            "-lc",
            (
                f"cd {root_wsl} && "
                "LD_LIBRARY_PATH=src/nav/services/plan/global_planner/algorithm/OctoPlanner3D/lib "
                f"{exe_wsl} --input {input_wsl} --output {output_wsl}"
            ),
        ]
    else:
        env = os.environ.copy()
        lib_dir = ROOT / "src" / "nav" / "services" / "plan" / "global_planner" / "algorithm" / "OctoPlanner3D" / "lib"
        env["LD_LIBRARY_PATH"] = f"{lib_dir}:{env.get('LD_LIBRARY_PATH', '')}"
        command = [str(exe), "--input", str(octomap_path), "--output", str(output)]

    info["enabled"] = True
    try:
        proc = subprocess.run(
            command,
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=30.0,
            env=None if os.name == "nt" else env,
            check=False,
        )
    except Exception as exc:
        info["error"] = f"{type(exc).__name__}: {exc}"
        return [], info

    info["returncode"] = int(proc.returncode)
    info["stdout"] = (proc.stdout or "").strip()[-600:]
    info["stderr"] = (proc.stderr or "").strip()[-600:]
    if proc.returncode != 0:
        info["error"] = "dump_tool_failed"
        return [], info

    points = _load_xyz_points(output, limit=load_limit)
    info["ok"] = bool(points)
    info["points"] = int(len(points))
    if not points:
        info["error"] = "dump_output_empty"
    return points, info


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


def _nearby_cloud_obstacles(
    points: Any,
    position: list[float],
    *,
    radius_m: float,
    max_points: int,
) -> Any:
    """Build local-planner obstacle rows from a live world-frame point cloud."""

    if max_points <= 0:
        return np.zeros((0, 4), dtype=np.float32)
    arr = np.asarray(points, dtype=np.float32)
    if arr.ndim != 2 or arr.shape[1] < 3 or len(arr) == 0:
        return np.zeros((0, 4), dtype=np.float32)
    xyz = arr[:, :3]
    finite = np.isfinite(xyz).all(axis=1)
    if not np.any(finite):
        return np.zeros((0, 4), dtype=np.float32)
    xyz = xyz[finite]
    dxy = xyz[:, :2] - np.asarray(position[:2], dtype=np.float32)
    mask = np.sum(dxy * dxy, axis=1) <= float(radius_m) * float(radius_m)
    if not np.any(mask):
        return np.zeros((0, 4), dtype=np.float32)
    xyz = xyz[mask]
    if len(xyz) > max_points:
        stride = max(1, int(math.ceil(len(xyz) / float(max_points))))
        xyz = xyz[::stride][:max_points]
    height = xyz[:, 2:3] - np.float32(position[2])
    return np.ascontiguousarray(np.concatenate([xyz, height], axis=1), dtype=np.float32)


def _path_length_xy(points: list[list[float]]) -> float:
    total = 0.0
    for prev, cur in zip(points, points[1:]):
        total += _distance2d(prev, cur)
    return total


def _vec3_xyz(point: Any) -> list[float]:
    if hasattr(point, "x") and hasattr(point, "y"):
        return [
            float(point.x),
            float(point.y),
            float(getattr(point, "z", 0.0)),
        ]
    return [
        float(point[0]),
        float(point[1]),
        float(point[2]) if len(point) > 2 else 0.0,
    ]


def _body_path_to_map(
    local_path: list[Any],
    position: list[float],
    yaw: float,
) -> list[list[float]]:
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    out: list[list[float]] = []
    for point in local_path:
        bx, by, bz = _vec3_xyz(point)
        out.append(
            [
                float(position[0]) + c * bx - s * by,
                float(position[1]) + s * bx + c * by,
                float(position[2]) + bz,
            ]
        )
    return out


def _pcd_xy_points(points: list[tuple[float, float, float]], limit: int = 1400) -> list[tuple[float, float]]:
    if not points:
        return []
    step = max(1, int(math.ceil(len(points) / max(1, int(limit)))))
    out: list[tuple[float, float]] = []
    for x, y, _z in points[::step]:
        out.append((float(x), float(y)))
    return out


def _write_navigation_video(
    *,
    args: argparse.Namespace,
    plan_report: dict[str, Any],
    snapshots: list[dict[str, Any]],
    start: list[float],
    goal: list[float],
    global_path: list[list[float]],
    pcd_points: list[tuple[float, float, float]],
    tracking_scene_xml: Path | None = None,
) -> dict[str, Any]:
    video_out = str(getattr(args, "video_out", "") or "")
    if not video_out:
        return {"path": "", "enabled": False, "frames": 0, "exists": False}
    from sim.scripts.mujoco.record_policy_nav_video import _render_replay

    output = Path(video_out)
    scene_xml = tracking_scene_xml or Path(str(plan_report["scene_xml"]))
    if not scene_xml.is_absolute():
        scene_xml = (ROOT / scene_xml).resolve()
    try:
        rendered = _render_replay(
            world=str(scene_xml),
            policy_path=str(args.policy_path or ""),
            drive_mode=str(args.drive_mode),
            output=output,
            snapshots=snapshots,
            start_xy=(float(start[0]), float(start[1])),
            goal_xy=(float(goal[0]), float(goal[1])),
            global_path=[(float(p[0]), float(p[1])) for p in global_path],
            obstacle_points=_pcd_xy_points(pcd_points),
            width=int(args.video_width),
            height=int(args.video_height),
            fps=float(args.video_fps),
            show_telemetry=not (
                bool(getattr(args, "video_clean", False)) or bool(getattr(args, "video_no_telemetry", False))
            ),
            show_inset=not (bool(getattr(args, "video_clean", False)) or bool(getattr(args, "video_no_inset", False))),
            show_robot_focus=not bool(getattr(args, "video_no_robot_focus", False)),
        )
    except Exception as exc:
        return {
            "path": str(output),
            "enabled": True,
            "frames": 0,
            "exists": output.is_file(),
            "error": str(exc),
            "source": "saved_map_tracking_gate_replay",
        }
    return {
        "path": str(output),
        "enabled": True,
        "frames": int(rendered),
        "exists": output.is_file(),
        "fps": float(args.video_fps),
        "width": int(args.video_width),
        "height": int(args.video_height),
        "source": "saved_map_tracking_gate_replay",
    }


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
        map_path=str(octomap_path),
        executable_path=args.planner_executable or None,
        timeout_s=args.planner_timeout,
    )
    planner.configure_constraints(plan_gate.planner_constraint_overrides(args))
    replanned = planner.plan(start, goal)
    json_path = (
        plan_gate.jsonable_path(replanned)
        if hasattr(plan_gate, "jsonable_path")
        else [[float(v) for v in point[:3]] for point in replanned]
    )
    return json_path, {
        "source": "tracking_gate_replan",
        "ok": bool(json_path) and bool(planner._last_plan_reached_goal),
        "path_count": len(json_path),
        "reached_goal": bool(planner._last_plan_reached_goal),
        "error": planner._last_plan_error,
        "diagnostics": planner._last_plan_diagnostics,
    }


def _append_requested_terminal_goal(
    global_path: list[list[float]],
    path_source: dict[str, Any],
    requested_goal: list[float],
) -> tuple[list[list[float]], dict[str, Any]]:
    if not global_path or _distance2d(global_path[-1], requested_goal) <= 0.05:
        return global_path, path_source
    terminal = [
        float(requested_goal[0]),
        float(requested_goal[1]),
        float(global_path[-1][2]) if len(global_path[-1]) >= 3 else float(requested_goal[2]),
    ]
    return [*global_path, terminal], {
        **path_source,
        "terminal_goal_appended": True,
        "terminal_goal_xy": [round(float(requested_goal[0]), 4), round(float(requested_goal[1]), 4)],
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


def _write_local_path_jsonl(path: Path, records: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for record in records:
            f.write(json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n")


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


def _path_debug_projector(
    *,
    pcd_points: list[list[float]],
    global_path: list[list[float]],
    trajectory: list[list[float]],
    local_records: list[dict[str, Any]],
    start: list[float],
    goal: list[float],
    width: int,
    height: int,
) -> Any:
    local_points: list[list[float]] = []
    for record in local_records[:: max(1, len(local_records) // 80)]:
        local_points.extend(record.get("local_path_map") or [])
    all_points = [*pcd_points, *global_path, *trajectory, *local_points, start, goal]
    xs = [float(p[0]) for p in all_points] or [0.0, 1.0]
    ys = [float(p[1]) for p in all_points] or [0.0, 1.0]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    pad_x = max(1.0, (max_x - min_x) * 0.08)
    pad_y = max(1.0, (max_y - min_y) * 0.08)
    min_x -= pad_x
    max_x += pad_x
    min_y -= pad_y
    max_y += pad_y
    scale = min(width / max(1e-6, max_x - min_x), height / max(1e-6, max_y - min_y))
    used_w = (max_x - min_x) * scale
    used_h = (max_y - min_y) * scale
    off_x = (width - used_w) * 0.5
    off_y = (height - used_h) * 0.5

    def project(point: list[float] | tuple[float, float]) -> tuple[int, int]:
        x = off_x + (float(point[0]) - min_x) * scale
        y = height - (off_y + (float(point[1]) - min_y) * scale)
        return int(round(x)), int(round(y))

    return project


def _draw_polyline_cv(
    frame: Any,
    points: list[list[float]],
    project: Any,
    color: tuple[int, int, int],
    width: int,
) -> None:
    if len(points) < 2:
        return
    import cv2

    pts = np.asarray([project(point) for point in points], dtype=np.int32)
    cv2.polylines(frame, [pts], isClosed=False, color=color, thickness=width, lineType=cv2.LINE_AA)


def _write_path_debug_video(
    path: Path,
    *,
    pcd_points: list[list[float]],
    global_path: list[list[float]],
    trajectory: list[list[float]],
    local_records: list[dict[str, Any]],
    start: list[float],
    goal: list[float],
    fps: float,
    width: int,
    height: int,
) -> dict[str, Any]:
    if not local_records:
        return {"path": str(path), "enabled": False, "exists": False, "frames": 0}
    import cv2

    path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        return {
            "path": str(path),
            "enabled": True,
            "exists": False,
            "frames": 0,
            "error": "video_writer_open_failed",
        }

    project = _path_debug_projector(
        pcd_points=pcd_points,
        global_path=global_path,
        trajectory=trajectory,
        local_records=local_records,
        start=start,
        goal=goal,
        width=width,
        height=height,
    )
    pcd_step = max(1, len(pcd_points) // 12000)
    pcd_xy = [project(point) for point in pcd_points[::pcd_step]]
    frames = 0
    try:
        stride = max(1, int(round(20.0 / max(1.0, fps))))
        trail: list[list[float]] = []
        for record in local_records[::stride]:
            frame = np.full((height, width, 3), 255, dtype=np.uint8)
            for px, py in pcd_xy:
                if 0 <= px < width and 0 <= py < height:
                    frame[py, px] = (178, 178, 178)
            _draw_polyline_cv(frame, global_path, project, (20, 128, 255), 3)
            trail.append([float(record["x"]), float(record["y"]), float(record.get("z", 0.0))])
            _draw_polyline_cv(frame, trail, project, (30, 80, 230), 3)
            _draw_polyline_cv(frame, record.get("local_path_map") or [], project, (30, 190, 60), 4)
            sx, sy = project(start)
            gx, gy = project(goal)
            rx, ry = project([float(record["x"]), float(record["y"]), 0.0])
            cv2.circle(frame, (sx, sy), 7, (40, 140, 40), -1, cv2.LINE_AA)
            cv2.circle(frame, (gx, gy), 9, (80, 40, 230), -1, cv2.LINE_AA)
            cv2.circle(frame, (rx, ry), 8, (0, 190, 255), -1, cv2.LINE_AA)
            cv2.putText(
                frame,
                "gray=OctoMap occupied voxels  orange=global  green=local  red=executed",
                (24, 38),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                (20, 20, 20),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                (
                    f"t={float(record['t']):.2f}s  "
                    f"local_pts={len(record.get('local_path_map') or [])}  "
                    f"obs={int(record.get('obstacle_points', 0))}  "
                    f"source={record.get('obstacle_source', '')}  "
                    f"reason={record.get('reason', '')}"
                ),
                (24, 72),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.64,
                (20, 20, 20),
                2,
                cv2.LINE_AA,
            )
            writer.write(frame)
            frames += 1
    finally:
        writer.release()
    return {
        "path": str(path),
        "enabled": True,
        "exists": path.is_file(),
        "frames": int(frames),
        "fps": float(fps),
        "width": int(width),
        "height": int(height),
    }


def _product_acceptance_blockers(args: argparse.Namespace, report: dict[str, Any]) -> list[str]:
    blockers: list[str] = []
    plan = report.get("plan") if isinstance(report.get("plan"), dict) else {}
    plan_report = report.get("plan_report") if isinstance(report.get("plan_report"), dict) else {}
    if not plan and isinstance(plan_report.get("plan"), dict):
        plan = plan_report["plan"]
    diagnostics = plan.get("diagnostics") if isinstance(plan.get("diagnostics"), dict) else {}
    constraints = diagnostics.get("constraints") if isinstance(diagnostics.get("constraints"), dict) else {}
    tracking = report.get("tracking") if isinstance(report.get("tracking"), dict) else {}
    artifacts = report.get("artifacts") if isinstance(report.get("artifacts"), dict) else {}
    video = report.get("video") if isinstance(report.get("video"), dict) else {}
    if not artifacts and plan_report:
        artifacts = {
            "map_pcd": plan_report.get("map_pcd"),
            "map_dir": plan_report.get("map_dir"),
            "scene_xml": plan_report.get("scene_xml"),
        }

    if str(getattr(args, "map_source", "")) != "mujoco_lidar":
        blockers.append("product acceptance requires map-source=mujoco_lidar")
    if str(getattr(args, "drive_mode", "")) != "policy":
        blockers.append("product acceptance requires drive-mode=policy")
    if not bool(getattr(args, "check_obstacle", False)):
        blockers.append("product acceptance requires local obstacle checking")
    if bool(getattr(args, "check_obstacle", False)) and str(getattr(args, "local_obstacle_source", "")) != "live_lidar":
        blockers.append("product acceptance requires local-obstacle-source=live_lidar")
    if bool(getattr(args, "planner_no_ground_support", False)):
        blockers.append("product acceptance forbids planner-no-ground-support")
    if constraints.get("require_ground_support") is not True:
        blockers.append("product acceptance requires OctoPlanner3D ground support")
    if report.get("policy_loaded") is not True:
        blockers.append("product acceptance requires a loaded ThunderV4 policy")
    if report.get("uses_ros") is not False:
        blockers.append("product acceptance must remain ROS-free")
    if int(plan.get("path_count") or 0) <= 1:
        blockers.append("product acceptance requires a non-trivial global path")
    if int(tracking.get("nonzero_cmd_count") or 0) <= 0:
        blockers.append("product acceptance requires nonzero follower commands")
    if not bool(tracking.get("arrived")) and float(tracking.get("final_error_m") or 999.0) > float(
        getattr(args, "acceptance_radius_m", 0.0)
    ):
        blockers.append("product acceptance requires arrival or final error within radius")
    for key in ("map_pcd", "map_dir", "scene_xml", "trajectory_csv", "preview_svg"):
        if not artifacts.get(key):
            blockers.append(f"product acceptance missing artifact: {key}")
    if str(getattr(args, "video_out", "") or ""):
        if video.get("exists") is not True or int(video.get("frames") or 0) <= 0:
            blockers.append("product acceptance requested video but video evidence is missing")
    clearance = report.get("clearance") if isinstance(report.get("clearance"), dict) else {}
    min_required_clearance = float(getattr(args, "min_clearance_m", 0.0) or 0.0)
    if min_required_clearance > 0.0:
        if clearance.get("ok") is not True:
            blockers.append("product acceptance requires trajectory clearance check")
        elif float(clearance.get("min_xy_clearance_m") or 0.0) < min_required_clearance:
            blockers.append(f"product acceptance requires trajectory clearance >= {min_required_clearance:.2f}m")
    return blockers


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
        trajectory_support_radius_m=args.trajectory_support_radius_m,
        trajectory_support_spacing_m=args.trajectory_support_spacing_m,
        resolution=args.resolution,
        support_dilation_cells=args.support_dilation_cells,
        free_layers_above=args.free_layers_above,
        free_dilation_cells=args.free_dilation_cells,
        converter=args.converter,
        no_env_converter=args.no_env_converter,
        converter_timeout=args.converter_timeout,
        planner_executable=args.planner_executable,
        planner_timeout=args.planner_timeout,
        planner_no_ground_support=args.planner_no_ground_support,
        planner_clearance_cells=args.planner_clearance_cells,
        planner_preblocked_cells=args.planner_preblocked_cells,
        planner_robot_radius=args.planner_robot_radius,
        planner_goal_tolerance_m=args.planner_goal_tolerance_m,
        planner_goal_xy_tolerance_m=args.planner_goal_xy_tolerance_m,
        planner_goal_z_tolerance_m=args.planner_goal_z_tolerance_m,
        planner_max_step_height_m=args.planner_max_step_height_m,
        start=args.start,
        goal=args.goal,
        skip_plan=False,
    )


def run_tracking_gate(args: argparse.Namespace) -> dict[str, Any]:
    out_dir = Path(
        args.out_dir or ROOT / "artifacts" / "mujoco_saved_map_tracking_gate" / time.strftime("%Y%m%d_%H%M%S")
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    plan_dir = out_dir / "plan"
    plan_report = plan_gate.run_gate(_plan_args(args, plan_dir))
    plan = plan_report.get("plan") or {}
    global_path, path_source = _extract_or_replan_global_path(plan_report, args)
    requested_goal = [float(v) for v in (plan_report.get("goal") or args.goal)[:3]]
    global_path, path_source = _append_requested_terminal_goal(global_path, path_source, requested_goal)
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
        if bool(args.product_acceptance):
            product_blockers = _product_acceptance_blockers(args, report)
            report["acceptance"] = {
                "mode": "product",
                "product_ready": False,
                "product_blockers": product_blockers,
                "requires_ground_support": True,
                "requires_mujoco_lidar_map": True,
                "requires_policy_motion": True,
                "requires_video": bool(str(getattr(args, "video_out", "") or "")),
            }
            report["blockers"] = [*report["blockers"], *product_blockers]
        _write_report(args, out_dir, report)
        return report

    from sim.engine.core.engine import VelocityCommand

    from drivers.sim.mujoco.runtime import build_engine

    try:
        nav_kernel = require_nav_kernel(
            required_symbols=(
                "Vec3",
                "LocalPlannerParams",
                "LocalPlanner",
                "PathFollowerParams",
                "PathFollowerState",
                "compute_control",
            ),
            context="MuJoCo saved-map tracking gate",
            anchor=__file__,
        )
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
    traversability_grid_status = _sync_scene_traversability_grid(local_planner, args)
    follower_params, follower_state = _build_path_follower(nav_kernel, args)
    pcd_points = _load_pcd_xyz(Path(plan_report["map_pcd"]), limit=args.preview_pcd_points)
    octomap_path = (
        plan.get("diagnostics", {}).get("runtime_map_path")
        or plan.get("diagnostics", {}).get("map_path")
        or plan_report.get("octomap_path")
        or plan_report.get("build", {}).get("octomap_path")
    )
    octomap_points, path_debug_map = (
        _dump_octomap_occupied_points(
            Path(str(octomap_path)),
            out_dir,
            load_limit=max(int(args.preview_pcd_points), 50000),
        )
        if octomap_path
        else (
            [],
            {
                "source": "octomap_occupied",
                "enabled": False,
                "ok": False,
                "points": 0,
                "error": "octomap_path_missing",
            },
        )
    )
    path_debug_points = octomap_points if octomap_points else pcd_points
    if octomap_points:
        path_debug_map["used_for_path_debug"] = True
    else:
        path_debug_map["used_for_path_debug"] = False
        path_debug_map["fallback_source"] = "map_pcd_preview"

    start = [float(v) for v in plan_report["start"][:3]]
    goal = [float(v) for v in plan_report["goal"][:3]]
    mujoco_start = [
        float(start[0]),
        float(start[1]),
        float(args.mujoco_start_z if args.mujoco_start_z is not None else max(start[2], 0.55)),
    ]
    tracking_scene_xml, dynamic_person_status = _inject_walking_person(
        Path(plan_report["scene_xml"]),
        out_dir,
        args,
    )
    engine = build_engine(
        world=tracking_scene_xml,
        drive_mode=str(args.drive_mode),
        policy_path=str(args.policy_path or ""),
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
    video_snapshots: list[dict[str, Any]] = []
    video_enabled = bool(str(getattr(args, "video_out", "") or ""))
    local_path_records: list[dict[str, Any]] = []
    person_trace: list[dict[str, Any]] = []

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

            person_pos = _update_walking_person(engine, args, elapsed_sim)
            if person_pos is not None:
                person_trace.append(
                    {
                        "t": round(float(elapsed_sim), 4),
                        "x": round(float(person_pos[0]), 5),
                        "y": round(float(person_pos[1]), 5),
                        "z": round(float(person_pos[2]), 5),
                    }
                )

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
                obstacle_source = "disabled"
                if args.check_obstacle:
                    obstacle_source = str(args.local_obstacle_source)
                    if obstacle_source == "live_lidar":
                        obstacles = _nearby_cloud_obstacles(
                            engine.get_lidar_points(),
                            position,
                            radius_m=float(args.obstacle_radius_m),
                            max_points=int(args.max_obstacle_points),
                        )
                    else:
                        obstacles = _nearby_pcd_obstacles(
                            pcd_points,
                            position,
                            radius_m=float(args.obstacle_radius_m),
                            max_points=int(args.max_obstacle_points),
                        )
                else:
                    obstacles = np.zeros((0, 4), dtype=np.float32)
                obstacle_flat = np.ascontiguousarray(obstacles, dtype=np.float32).ravel()
                local = local_planner.plan_frame_without_grid(
                    position[0],
                    position[1],
                    position[2],
                    yaw,
                    float(target[0]),
                    float(target[1]),
                    obstacle_flat,
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
                local_path_map = _body_path_to_map(local_path, position, yaw)
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
                        "obstacle_source": obstacle_source,
                        "obstacle_points": int(len(obstacles)),
                        "reason": last_reason,
                    }
                )
                local_path_records.append(
                    {
                        "t": round(float(elapsed_sim), 4),
                        "x": round(position[0], 5),
                        "y": round(position[1], 5),
                        "z": round(position[2], 5),
                        "yaw": round(float(yaw), 5),
                        "target_index": int(cursor),
                        "target": [
                            round(float(target[0]), 5),
                            round(float(target[1]), 5),
                            round(float(target[2]), 5),
                        ],
                        "cmd": [
                            round(float(cmd.linear_x), 5),
                            round(float(cmd.linear_y), 5),
                            round(float(cmd.angular_z), 5),
                        ],
                        "obstacle_source": obstacle_source,
                        "obstacle_points": int(len(obstacles)),
                        "local_path_map": [
                            [round(float(p[0]), 5), round(float(p[1]), 5), round(float(p[2]), 5)]
                            for p in local_path_map
                        ],
                        "reason": last_reason,
                    }
                )
                if video_enabled and getattr(engine, "data", None) is not None:
                    video_snapshots.append(
                        {
                            "t": float(elapsed_sim),
                            "x": float(position[0]),
                            "y": float(position[1]),
                            "z": float(position[2]),
                            "dist": float(_distance2d(position, goal)),
                            "state": str(last_reason),
                            "cmd": [float(cmd.linear_x), float(cmd.angular_z)],
                            "qpos": np.asarray(engine.data.qpos, dtype=np.float64).copy().tolist(),
                            "local_path_map": local_path_map,
                            "person": person_pos,
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
    local_path_jsonl = out_dir / "local_path_timeseries.jsonl"
    _write_trajectory_csv(trajectory_csv, samples)
    _write_local_path_jsonl(local_path_jsonl, local_path_records)
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
    video = _write_navigation_video(
        args=args,
        plan_report=plan_report,
        snapshots=video_snapshots,
        start=start,
        goal=goal,
        global_path=global_path,
        pcd_points=pcd_points,
        tracking_scene_xml=tracking_scene_xml,
    )
    path_video_out = str(getattr(args, "path_video_out", "") or "")
    path_debug_video = _write_path_debug_video(
        Path(path_video_out) if path_video_out else out_dir / "local_path_debug.mp4",
        pcd_points=path_debug_points,
        global_path=global_path,
        trajectory=trajectory,
        local_records=local_path_records,
        start=start,
        goal=goal,
        fps=float(args.video_fps),
        width=int(args.video_width),
        height=int(args.video_height),
    )
    clearance = _trajectory_clearance_to_obstacles(
        trajectory,
        path_debug_points,
    )
    if clearance.get("ok") is True:
        clearance["min_required_clearance_m"] = float(args.min_clearance_m)
        clearance["meets_required_clearance"] = float(clearance["min_xy_clearance_m"]) >= float(args.min_clearance_m)
    gate_ok = (
        plan_report.get("ok") is True
        and bool(global_path)
        and nonzero_cmd_count > 0
        and len(trajectory) >= 2
        and (arrived or final_error_m <= float(args.acceptance_radius_m))
    )
    report = {
        "schema_version": "lingtu.mujoco_saved_map_tracking_gate.v1",
        "ok": bool(gate_ok),
        "stage": "tracking",
        "simulation_only": True,
        "uses_ros": False,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "out_dir": str(out_dir),
        "scene_preset": args.scene_preset,
        "drive_mode": str(args.drive_mode),
        "policy_loaded": bool(getattr(engine, "has_policy", False)),
        "policy_path": str(getattr(engine, "policy_path", args.policy_path or "")),
        "dataflow": [
            "map.pcd",
            "octomap.ot",
            "OctoPlanner3D global_path",
            "lingtu_nav_kernel LocalPlanner",
            "lingtu_nav_kernel PathFollower",
            ("MuJoCo RL policy robot" if str(args.drive_mode) == "policy" else "MuJoCo kinematic robot"),
        ],
        "limitations": (
            [
                *(
                    ["building_scene_tracking_is_flat_kinematic"]
                    if args.scene_preset == "building" and str(args.drive_mode) == "kinematic"
                    else []
                ),
                *(
                    ["policy_not_loaded"]
                    if str(args.drive_mode) == "policy" and not bool(getattr(engine, "has_policy", False))
                    else []
                ),
            ]
        ),
        "artifacts": {
            "map_pcd": plan_report.get("map_pcd"),
            "map_dir": plan_report.get("map_dir"),
            "scene_xml": plan_report.get("scene_xml"),
            "tracking_scene_xml": str(tracking_scene_xml),
            "trajectory_csv": str(trajectory_csv),
            "preview_svg": str(preview_svg),
            "local_path_jsonl": str(local_path_jsonl),
            "local_path_debug_video": path_debug_video.get("path") if path_debug_video.get("enabled") else "",
            "path_debug_map_points": path_debug_map.get("path") if path_debug_map.get("ok") else "",
            "video_mp4": video.get("path") if video.get("enabled") else "",
        },
        "video": video,
        "path_debug_video": path_debug_video,
        "path_debug_map": path_debug_map,
        "dynamic_person": {
            **dynamic_person_status,
            "trace_samples": len(person_trace),
            "trace_preview": person_trace[:5],
        },
        "traversability_grid": traversability_grid_status,
        "clearance": clearance,
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
                round(float(local_path_points_total / max(1, len(samples))), 2) if samples else 0.0
            ),
            "last_reason": last_reason,
            "tick_hz": float(args.tick_hz),
            "max_speed": float(args.max_speed),
            "check_obstacle": bool(args.check_obstacle),
            "local_obstacle_source": str(args.local_obstacle_source) if bool(args.check_obstacle) else "disabled",
            "obstacle_points_avg": (
                round(
                    float(sum(int(row.get("obstacle_points", 0)) for row in samples) / max(1, len(samples))),
                    2,
                )
                if samples
                else 0.0
            ),
            "obstacle_points_max": (max(int(row.get("obstacle_points", 0)) for row in samples) if samples else 0),
        },
        "plan_report": plan_report,
    }
    product_blockers = _product_acceptance_blockers(args, report)
    report["acceptance"] = {
        "mode": "product" if bool(args.product_acceptance) else "demo",
        "product_ready": bool(not product_blockers),
        "product_blockers": product_blockers,
        "requires_ground_support": True,
        "requires_mujoco_lidar_map": True,
        "requires_policy_motion": True,
        "requires_video": bool(str(getattr(args, "video_out", "") or "")),
    }
    if bool(args.product_acceptance) and product_blockers:
        report["ok"] = False
        report["blockers"] = [*report.get("blockers", []), *product_blockers]
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
    parser.add_argument("--scene-preset", choices=plan_gate.SCENE_PRESETS, default="corridor")
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
    parser.add_argument("--trajectory-support-radius-m", type=float, default=0.0)
    parser.add_argument("--trajectory-support-spacing-m", type=float, default=0.1)
    parser.add_argument("--resolution", type=float, default=0.2)
    parser.add_argument("--support-dilation-cells", type=int, default=1)
    parser.add_argument("--free-layers-above", type=int, default=3)
    parser.add_argument("--free-dilation-cells", type=int, default=1)
    parser.add_argument("--converter", default="")
    parser.add_argument("--no-env-converter", action="store_true")
    parser.add_argument("--converter-timeout", type=float, default=60.0)
    parser.add_argument("--planner-executable", default="")
    parser.add_argument("--planner-timeout", type=float, default=30.0)
    parser.add_argument("--planner-no-ground-support", action="store_true")
    parser.add_argument("--planner-clearance-cells", type=int, default=-1)
    parser.add_argument("--planner-preblocked-cells", type=int, default=-1)
    parser.add_argument("--planner-robot-radius", type=float, default=0.0)
    parser.add_argument("--planner-goal-tolerance-m", type=float, default=0.0)
    parser.add_argument("--planner-goal-xy-tolerance-m", type=float, default=0.0)
    parser.add_argument("--planner-goal-z-tolerance-m", type=float, default=0.0)
    parser.add_argument(
        "--planner-max-step-height-m",
        type=float,
        default=0.0,
        help="Override OctoPlanner3D max_step_height. Multilevel scenes default to 0.23m.",
    )
    parser.add_argument("--start", nargs=3, type=float, default=plan_gate.CORRIDOR_DEFAULT_START)
    parser.add_argument("--goal", nargs=3, type=float, default=plan_gate.CORRIDOR_DEFAULT_GOAL)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--max-wall-time-s", type=float, default=90.0)
    parser.add_argument(
        "--drive-mode",
        choices=("policy", "kinematic"),
        default="policy",
        help=(
            "MuJoCo actuation mode. Use policy for RL gait validation; "
            "kinematic is only a deterministic planning/tracking smoke mode."
        ),
    )
    parser.add_argument(
        "--policy-path",
        default="",
        help="Optional explicit ThunderV4 policy path. Empty uses the sim driver default policy.",
    )
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
    parser.add_argument(
        "--use-traversability-grid",
        action="store_true",
        help=(
            "Feed a scene traversability risk grid into the C++ LocalPlanner. "
            "Implemented for named multilevel MuJoCo scene presets; other presets report grid_unavailable."
        ),
    )
    parser.add_argument(
        "--traversability-near-field-stop",
        action="store_true",
        help="Allow high traversability risk directly in front of the body to trigger near-field stop.",
    )
    parser.add_argument(
        "--local-obstacle-source",
        choices=("live_lidar", "saved_map"),
        default="live_lidar",
        help=(
            "Obstacle source for the local planner when --check-obstacle is enabled. "
            "Use live_lidar for product validation; saved_map is only a static regression mode."
        ),
    )
    parser.add_argument("--obstacle-radius-m", type=float, default=5.0)
    parser.add_argument("--max-obstacle-points", type=int, default=2500)
    parser.add_argument("--vehicle-length-m", type=float, default=0.8)
    parser.add_argument("--vehicle-width-m", type=float, default=0.45)
    parser.add_argument("--sensor-offset-x-m", type=float, default=0.0)
    parser.add_argument("--sensor-offset-y-m", type=float, default=0.0)
    parser.add_argument(
        "--dynamic-person",
        action="store_true",
        help="Inject a moving mocap person into the tracking scene only; the person is not baked into the static OctoMap.",
    )
    parser.add_argument("--person-start", nargs=3, type=float, default=[6.2, -1.0, 1.2])
    parser.add_argument("--person-end", nargs=3, type=float, default=[6.2, 0.9, 1.2])
    parser.add_argument("--person-speed-mps", type=float, default=0.45)
    parser.add_argument(
        "--min-clearance-m",
        type=float,
        default=0.45,
        help="Minimum same-body-height XY clearance required for product acceptance.",
    )
    parser.add_argument("--mujoco-start-z", type=float, default=None)
    parser.add_argument("--path-library-dir", type=Path, default=PATH_LIBRARY)
    parser.add_argument("--preview-pcd-points", type=int, default=20000)
    parser.add_argument("--video-out", default="")
    parser.add_argument("--video-fps", type=float, default=20.0)
    parser.add_argument("--video-width", type=int, default=960)
    parser.add_argument("--video-height", type=int, default=540)
    parser.add_argument(
        "--path-video-out",
        default="",
        help="Optional top-down diagnostic video showing map, global path, local path, and executed trail.",
    )
    parser.add_argument(
        "--video-clean",
        action="store_true",
        help="Hide telemetry text and the route inset for product recordings.",
    )
    parser.add_argument("--video-no-telemetry", action="store_true")
    parser.add_argument("--video-no-inset", action="store_true")
    parser.add_argument("--video-no-robot-focus", action="store_true")
    parser.add_argument("--json-out", default="")
    parser.add_argument(
        "--product-acceptance",
        action="store_true",
        help=(
            "Apply the product MuJoCo navigation gate: MuJoCo LiDAR map, "
            "policy drive, OctoPlanner3D ground support, nonzero commands, "
            "and arrival. Demo runs may omit this while the support-map issue "
            "is being fixed."
        ),
    )
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    report = run_tracking_gate(args)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report.get("ok") is True or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
