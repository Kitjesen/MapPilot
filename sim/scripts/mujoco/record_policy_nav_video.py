#!/usr/bin/env python3
"""Record a simulation-only MuJoCo policy navigation run to MP4.

This script never talks to real robot services. It runs the in-process
sim_mujoco driver through the LingTu full stack, sends one map-frame goal, and
records an offscreen MuJoCo view with navigation telemetry overlaid.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def _load_policy_metadata(policy_path: str) -> dict[str, Any]:
    if not policy_path:
        return {"path": "", "exists": False}
    path = Path(policy_path)
    meta: dict[str, Any] = {"path": str(path), "exists": path.exists()}
    if not path.exists():
        return meta
    try:
        import hashlib

        meta["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    except Exception as exc:
        meta["sha256_error"] = str(exc)
    try:
        import onnxruntime as ort

        sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        meta["input"] = [{"name": inp.name, "shape": list(inp.shape), "type": inp.type} for inp in sess.get_inputs()]
        meta["output"] = [{"name": out.name, "shape": list(out.shape), "type": out.type} for out in sess.get_outputs()]
    except Exception as exc:
        meta["onnx_error"] = str(exc)
    return meta


def _put_text(
    frame: np.ndarray,
    text: str,
    y: int,
    color=(255, 255, 255),
    *,
    x: int = 28,
    scale: float = 0.72,
    thickness: int = 2,
) -> None:
    import cv2

    (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    pad = 8
    cv2.rectangle(
        frame,
        (max(0, x - pad), max(0, y - th - pad)),
        (min(frame.shape[1] - 1, x + tw + pad), min(frame.shape[0] - 1, y + baseline + pad)),
        (15, 20, 24),
        -1,
    )
    cv2.putText(
        frame,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def _draw_inset(
    frame: np.ndarray,
    *,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    trail: list[tuple[float, float]],
    global_path: list[tuple[float, float]] | None = None,
    obstacle_points: list[tuple[float, float]] | None = None,
) -> None:
    import cv2

    h, w = frame.shape[:2]
    inset_w = max(360, min(520, int(w * 0.27)))
    inset_h = max(240, min(360, int(h * 0.33)))
    x0, y0 = w - inset_w - 28, h - inset_h - 28
    x1, y1 = w - 28, y0 + inset_h
    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x1, y1), (12, 16, 18), -1)
    cv2.addWeighted(overlay, 0.86, frame, 0.14, 0.0, frame)
    cv2.rectangle(frame, (x0, y0), (x1, y1), (235, 235, 235), 2)
    path_points = global_path or []
    map_points = obstacle_points or []
    points = [start_xy, goal_xy, *trail, *path_points, *map_points]
    min_x = min(p[0] for p in points) - 0.3
    max_x = max(p[0] for p in points) + 0.3
    min_y = min(p[1] for p in points) - 0.3
    max_y = max(p[1] for p in points) + 0.3
    sx = (x1 - x0 - 30) / max(max_x - min_x, 0.1)
    sy = (y1 - y0 - 30) / max(max_y - min_y, 0.1)
    scale = min(sx, sy)

    def project(p: tuple[float, float]) -> tuple[int, int]:
        px = x0 + 15 + int((p[0] - min_x) * scale)
        py = y1 - 15 - int((p[1] - min_y) * scale)
        return px, py

    if map_points:
        for p in map_points[:: max(1, len(map_points) // 1800)]:
            cv2.circle(frame, project(p), 1, (92, 92, 120), -1)
    if len(path_points) >= 2:
        pts = np.array([project(p) for p in path_points], dtype=np.int32)
        cv2.polylines(frame, [pts], isClosed=False, color=(255, 140, 40), thickness=4)
        for point in pts:
            cv2.circle(frame, tuple(int(v) for v in point), 4, (255, 170, 70), -1)
    if len(trail) >= 2:
        pts = np.array([project(p) for p in trail], dtype=np.int32)
        cv2.polylines(frame, [pts], isClosed=False, color=(40, 245, 80), thickness=4)
    cv2.circle(frame, project(start_xy), 8, (80, 220, 80), -1)
    cv2.circle(frame, project(goal_xy), 10, (70, 70, 255), -1)
    if trail:
        cv2.circle(frame, project(trail[-1]), 10, (40, 230, 255), -1)
    cv2.putText(
        frame,
        "map / path / trail",
        (x0 + 14, y0 + 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (245, 245, 245),
        2,
        cv2.LINE_AA,
    )
    legend_y = y1 - 34
    cv2.line(frame, (x0 + 18, legend_y), (x0 + 62, legend_y), (255, 140, 40), 4)
    cv2.putText(
        frame, "global", (x0 + 68, legend_y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (245, 245, 245), 1, cv2.LINE_AA
    )
    cv2.line(frame, (x0 + 154, legend_y), (x0 + 194, legend_y), (40, 245, 80), 4)
    cv2.putText(
        frame, "trail", (x0 + 202, legend_y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (245, 245, 245), 1, cv2.LINE_AA
    )
    cv2.circle(frame, (x0 + 276, legend_y), 6, (70, 70, 255), -1)
    cv2.putText(
        frame, "goal", (x0 + 290, legend_y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (245, 245, 245), 1, cv2.LINE_AA
    )


def _path_points(path: Any) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for p in path or []:
        try:
            if isinstance(p, dict):
                x = float(p.get("x"))
                y = float(p.get("y"))
            else:
                x = float(p[0])
                y = float(p[1])
        except Exception:
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y))
    return points


def _path_distance(points: list[tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])))


def _yaw_from_mj_qpos(qpos: np.ndarray) -> float | None:
    if qpos.size < 7:
        return None
    qw, qx, qy, qz = (float(v) for v in qpos[3:7])
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm <= 1e-9:
        return None
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _draw_robot_focus(frame: np.ndarray, qpos: np.ndarray) -> None:
    import cv2

    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    color = (0, 245, 255)
    shadow = (10, 15, 18)
    cv2.circle(frame, (cx, cy), 52, shadow, 5, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 52, color, 3, cv2.LINE_AA)

    yaw = _yaw_from_mj_qpos(qpos)
    if yaw is not None:
        # The camera azimuth is fixed at 135 degrees. Rotate the world yaw into
        # the screen plane so the marker gives a stable heading cue.
        screen_yaw = yaw - math.radians(135.0)
        end = (
            int(cx + math.cos(screen_yaw) * 66),
            int(cy - math.sin(screen_yaw) * 66),
        )
        cv2.arrowedLine(frame, (cx, cy), end, color, 5, cv2.LINE_AA, tipLength=0.24)

    cv2.rectangle(frame, (cx + 58, cy + 24), (cx + 194, cy + 60), shadow, -1)
    cv2.putText(
        frame,
        "robot",
        (cx + 78, cy + 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.68,
        color,
        2,
        cv2.LINE_AA,
    )


def _boost_robot_replay_materials(model: Any) -> None:
    # The ThunderV4 mesh material is almost white; in a white indoor scene it
    # disappears in recorded evidence. This is a replay-only visual override.
    try:
        import mujoco
    except Exception:
        mujoco = None
    try:
        geom_group = model.geom_group
        geom_rgba = model.geom_rgba
        ngeom = int(model.ngeom)
    except Exception:
        return
    transparent_prefixes = (
        "floor_2",
        "wall_2",
        "obs_2f",
        "stair_wall",
        "step_",
        "rail_",
    )
    for geom_id in range(ngeom):
        try:
            name = ""
            if mujoco is not None:
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
            if name.startswith(transparent_prefixes):
                geom_rgba[geom_id][:] = [0.62, 0.62, 0.62, 0.10]
                continue
            if int(geom_group[geom_id]) != 1:
                continue
            geom_rgba[geom_id][:] = [0.04, 0.24, 0.95, 1.0]
        except Exception:
            continue


def _load_tomogram_obstacle_points(
    tomogram: str,
    *,
    obstacle_thr: float = 49.9,
    max_points: int = 1400,
) -> list[tuple[float, float]]:
    if not tomogram:
        return []
    import pickle

    path = Path(tomogram)
    if not path.exists():
        return []
    try:
        with path.open("rb") as fh:
            raw = pickle.load(fh)
    except Exception:
        return []
    if not isinstance(raw, dict):
        return []
    data = raw.get("data")
    res = float(raw.get("resolution", 0.2))
    if data is not None and hasattr(data, "ndim") and data.ndim == 4:
        grid = np.asarray(data[0, 0], dtype=np.float32)
        center = np.asarray(raw.get("center", [0, 0])[:2], dtype=np.float64)
        h, w = grid.shape
        origin = center - np.array([w * res / 2.0, h * res / 2.0])
    else:
        grid_obj = raw.get("grid", raw.get("traversability"))
        if grid_obj is None:
            return []
        grid = np.asarray(grid_obj, dtype=np.float32)
        origin = np.asarray(raw.get("origin", [0, 0])[:2], dtype=np.float64)
    rows, cols = np.where(grid >= obstacle_thr)
    if len(rows) == 0:
        return []
    step = max(1, int(math.ceil(len(rows) / max_points)))
    return [
        (float(origin[0] + col * res), float(origin[1] + row * res)) for row, col in zip(rows[::step], cols[::step])
    ]


def _render_replay(
    *,
    world: str,
    policy_path: str,
    drive_mode: str,
    output: Path,
    snapshots: list[dict[str, Any]],
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    global_path: list[tuple[float, float]],
    obstacle_points: list[tuple[float, float]],
    width: int,
    height: int,
    fps: float,
    show_telemetry: bool = True,
    show_inset: bool = True,
    show_robot_focus: bool = True,
) -> int:
    import cv2

    import mujoco
    from drivers.sim.mujoco.driver import MujocoDriverModule

    if not snapshots:
        raise RuntimeError("No navigation snapshots captured; nothing to render")

    driver = MujocoDriverModule(
        world=world,
        render=False,
        enable_camera=False,
        drive_mode=drive_mode,
        policy_path=policy_path,
    )
    driver.setup()
    engine = driver._engine
    renderer = None
    writer = None
    try:
        if engine is None or engine.model is None or engine.data is None:
            raise RuntimeError("Replay MuJoCo engine is not available")
        _boost_robot_replay_materials(engine.model)
        engine.model.vis.global_.offwidth = max(int(engine.model.vis.global_.offwidth), width)
        engine.model.vis.global_.offheight = max(int(engine.model.vis.global_.offheight), height)
        renderer = mujoco.Renderer(engine.model, height=height, width=width)
        scene_option = mujoco.MjvOption()
        scene_option.geomgroup[:] = 1
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        route_points = [
            start_xy,
            goal_xy,
            *global_path,
            *[(float(s["x"]), float(s["y"])) for s in snapshots],
        ]
        xs = [p[0] for p in route_points]
        ys = [p[1] for p in route_points]
        extent = max(max(xs) - min(xs), max(ys) - min(ys), 0.8)
        camera.distance = max(10.0, min(15.0, extent * 0.72))
        camera.elevation = -82.0
        camera.azimuth = 135.0
        camera.lookat[0] = start_xy[0]
        camera.lookat[1] = start_xy[1]
        camera.lookat[2] = 0.85

        output.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(output),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (width, height),
        )
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open video writer: {output}")

        replay_trail: list[tuple[float, float]] = []
        for snap in snapshots:
            qpos = np.asarray(snap["qpos"], dtype=np.float64)
            n_qpos = min(qpos.size, engine.data.qpos.size)
            engine.data.qpos[:n_qpos] = qpos[:n_qpos]
            person = snap.get("person")
            if person is not None:
                try:
                    body_id = mujoco.mj_name2id(engine.model, mujoco.mjtObj.mjOBJ_BODY, "walking_person")
                    if body_id >= 0:
                        mocap_id = int(engine.model.body_mocapid[body_id])
                        if mocap_id >= 0:
                            engine.data.mocap_pos[mocap_id][:] = np.asarray(person[:3], dtype=np.float64)
                            engine.data.mocap_quat[mocap_id][:] = [1.0, 0.0, 0.0, 0.0]
                except Exception:
                    pass
            engine.data.qvel[:] = 0.0
            mujoco.mj_forward(engine.model, engine.data)

            x = float(snap["x"])
            y = float(snap["y"])
            replay_trail.append((x, y))
            camera.lookat[0] = x
            camera.lookat[1] = y
            camera.lookat[2] = max(0.75, float(snap.get("z", 0.5)) + 0.45)
            renderer.update_scene(engine.data, camera, scene_option=scene_option)
            frame = renderer.render().copy()
            cmd = snap["cmd"]
            if show_telemetry:
                _put_text(frame, "LingTu MuJoCo navigation acceptance", 42, (255, 255, 255), scale=0.82)
                _put_text(
                    frame,
                    f"state={snap['state']}   t={snap['t']:.1f}s   dist={snap['dist']:.3f}m   global_pts={len(global_path)}",
                    82,
                    (0, 255, 255),
                    scale=0.66,
                )
                _put_text(
                    frame,
                    f"robot=({x:.2f},{y:.2f})   goal=({goal_xy[0]:.2f},{goal_xy[1]:.2f})   cmd_vx={cmd[0]:.2f} cmd_wz={cmd[1]:.2f}",
                    120,
                    (230, 230, 230),
                    scale=0.58,
                )
            if show_robot_focus:
                _draw_robot_focus(frame, qpos)
            if show_inset:
                _draw_inset(
                    frame,
                    start_xy=start_xy,
                    goal_xy=goal_xy,
                    trail=replay_trail,
                    global_path=global_path,
                    obstacle_points=obstacle_points,
                )
            writer.write(frame[:, :, ::-1])
        return len(snapshots)
    finally:
        if writer is not None:
            writer.release()
        if renderer is not None:
            renderer.close()
        driver.stop()


def record_full_stack_nav(
    *,
    world: str,
    policy_path: str,
    drive_mode: str,
    tomogram: str,
    goal_xy: tuple[float, float] | None,
    enable_map_modules: bool,
    output: Path,
    goal_distance: float,
    duration: float,
    fps: float,
    width: int,
    height: int,
    waypoint_threshold: float,
    final_waypoint_threshold: float,
    downsample_dist: float,
    path_goal_tolerance: float,
    path_min_speed: float,
    path_max_speed: float,
    safe_goal_tolerance: float,
    nav_max_angular_z: float,
    success_settle: float,
) -> dict[str, Any]:
    from sim.scripts.policy_nav_smoke import (
        PRODUCTION_GLOBAL_PLANNER_BACKEND,
        PRODUCTION_LOCAL_PLANNER_BACKEND,
        PRODUCTION_PATH_FOLLOWER_BACKEND,
    )

    from lingtu.assembly.profile_builder import build_system_for_profile
    from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3

    system = build_system_for_profile(
        "sim",
        dict(
            robot="sim_mujoco",
            world=world,
            slam_profile="none",
            detector="sim_scene",
            llm="mock",
            planner_backend=PRODUCTION_GLOBAL_PLANNER_BACKEND,
            enable_native=False,
            enable_semantic=False,
            enable_gateway=False,
            enable_map_modules=enable_map_modules,
            render=False,
            python_autonomy_backend=PRODUCTION_LOCAL_PLANNER_BACKEND,
            python_path_follower_backend=PRODUCTION_PATH_FOLLOWER_BACKEND,
            drive_mode=drive_mode,
            policy_path=policy_path,
            tomogram=tomogram,
            max_angular_vel=nav_max_angular_z,
            waypoint_threshold=waypoint_threshold,
            final_waypoint_threshold=final_waypoint_threshold,
            downsample_dist=downsample_dist,
            path_follower_goal_tolerance=path_goal_tolerance,
            path_follower_min_speed=path_min_speed,
            path_follower_max_speed=path_max_speed,
            safe_goal_tolerance=safe_goal_tolerance,
            run_startup_checks=False,
        ),
    )

    driver = system.get_module("MujocoDriverModule")
    ogm = system.get_module("OccupancyGridModule") if enable_map_modules else None
    nav = system.get_module("nav.mission")
    mux = system.get_module("nav.velocity_mux")

    seen = {"costmap": 0, "waypoints": 0, "mux_cmd": 0}
    odom: list[tuple[float, float, float]] = []
    last_mux: list[tuple[float, float]] = []
    trail: list[tuple[float, float]] = []
    global_path_points: list[tuple[float, float]] = []
    latest_global_path_points: list[tuple[float, float]] = []

    if ogm is not None:
        ogm.costmap._add_callback(lambda _: seen.__setitem__("costmap", seen["costmap"] + 1))
    nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))

    def _record_global_path(path: Any) -> None:
        nonlocal global_path_points, latest_global_path_points
        points = _path_points(path)
        if points:
            if not global_path_points:
                global_path_points = points
            latest_global_path_points = points

    nav.global_path._add_callback(_record_global_path)
    mux.driver_cmd_vel._add_callback(
        lambda m: (
            seen.__setitem__("mux_cmd", seen["mux_cmd"] + 1),
            last_mux.append((float(m.linear.x), float(m.angular.z))),
        )
    )
    driver.odometry._add_callback(
        lambda m: odom.append((float(m.pose.position.x), float(m.pose.position.y), float(m.pose.position.z)))
    )

    snapshots: list[dict[str, Any]] = []
    result: dict[str, Any] | None = None
    system.start()
    try:
        warm_deadline = time.time() + min(10.0, max(3.0, duration * 0.25))
        while time.time() < warm_deadline and ((ogm is not None and seen["costmap"] == 0) or not odom):
            time.sleep(0.1)
        if not odom:
            raise RuntimeError("No odometry from sim_mujoco policy mode")
        engine = driver._engine
        if engine is None or engine.model is None or engine.data is None:
            raise RuntimeError("MuJoCo engine is not available")

        start = odom[-1]
        if goal_xy is None:
            goal_x = start[0] + goal_distance
            goal_y = start[1]
        else:
            goal_x, goal_y = goal_xy
        nav.goal_pose._deliver(
            PoseStamped(
                pose=Pose(
                    position=Vector3(goal_x, goal_y, 0.0),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                frame_id="map",
                ts=time.time(),
            )
        )
        plan_deadline = time.time() + 3.0
        while time.time() < plan_deadline and not global_path_points:
            time.sleep(0.05)

        deadline = time.time() + duration
        next_frame = time.time()
        frame_period = 1.0 / max(fps, 1.0)
        started_at = time.time()
        success_at: float | None = None
        dist_at_success: float | None = None
        success_pose: tuple[float, float, float] | None = None

        while time.time() < deadline:
            now = time.time()
            if odom:
                x, y, z = odom[-1]
                if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.02:
                    trail.append((x, y))
                dist_to_goal = math.hypot(goal_x - x, goal_y - y)
            else:
                x, y, z = start
                dist_to_goal = math.hypot(goal_x - x, goal_y - y)

            if now >= next_frame:
                state = str(getattr(nav, "_state", ""))
                cmd = last_mux[-1] if last_mux else (0.0, 0.0)
                snapshots.append(
                    {
                        "t": time.time() - started_at,
                        "x": float(x),
                        "y": float(y),
                        "z": float(z),
                        "dist": float(dist_to_goal),
                        "state": state,
                        "cmd": [float(cmd[0]), float(cmd[1])],
                        "qpos": engine.data.qpos.copy().tolist(),
                    }
                )
                next_frame += frame_period

            if str(getattr(nav, "_state", "")) == "SUCCESS":
                if success_at is None:
                    success_at = time.time()
                    dist_at_success = dist_to_goal
                    success_pose = odom[-1] if odom else None
                if time.time() - success_at >= success_settle:
                    break
            time.sleep(0.02)

        elapsed = time.time() - started_at
        end = odom[-1] if odom else start
        direct_distance = math.hypot(goal_x - start[0], goal_y - start[1])
        global_distance = _path_distance(global_path_points)
        latest_global_distance = _path_distance(latest_global_path_points)
        result = {
            "output": str(output),
            "frames": len(snapshots),
            "fps": fps,
            "elapsed_s": elapsed,
            "world": world,
            "drive_mode": drive_mode,
            "tomogram": tomogram,
            "global_planner_backend_requested": PRODUCTION_GLOBAL_PLANNER_BACKEND,
            "local_planner_backend_requested": PRODUCTION_LOCAL_PLANNER_BACKEND,
            "path_follower_backend_requested": PRODUCTION_PATH_FOLLOWER_BACKEND,
            "policy": _load_policy_metadata(str(getattr(driver, "_policy_path", policy_path))),
            "start": [float(v) for v in start[:3]],
            "goal": [float(goal_x), float(goal_y), 0.0],
            "end": [float(v) for v in end[:3]],
            "success_seen": success_at is not None,
            "dist_at_success_m": dist_at_success,
            "success_end": [float(v) for v in success_pose[:3]] if success_pose else None,
            "dist_to_goal_m": math.hypot(goal_x - end[0], goal_y - end[1]),
            "nav_state": str(getattr(nav, "_state", "")),
            "seen": seen,
            "global_path": {
                "count": len(global_path_points),
                "distance_m": global_distance,
                "direct_distance_m": direct_distance,
                "detour_ratio": (global_distance / direct_distance if direct_distance > 1e-6 else None),
                "max_y": max((p[1] for p in global_path_points), default=None),
                "last": list(global_path_points[-1]) if global_path_points else None,
            },
            "latest_global_path": {
                "count": len(latest_global_path_points),
                "distance_m": latest_global_distance,
                "last": list(latest_global_path_points[-1]) if latest_global_path_points else None,
            },
        }
    finally:
        system.stop()
    if result is None:
        raise RuntimeError("Navigation run did not produce a result")
    obstacle_points = _load_tomogram_obstacle_points(tomogram)
    rendered_frames = _render_replay(
        world=world,
        policy_path=policy_path,
        drive_mode=drive_mode,
        output=output,
        snapshots=snapshots,
        start_xy=(float(result["start"][0]), float(result["start"][1])),
        goal_xy=(float(result["goal"][0]), float(result["goal"][1])),
        global_path=global_path_points,
        obstacle_points=obstacle_points,
        width=width,
        height=height,
        fps=fps,
    )
    result["frames"] = rendered_frames
    return result


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="open_field")
    parser.add_argument("--policy", default="", help="Explicit ONNX policy path")
    parser.add_argument("--drive-mode", choices=("policy", "kinematic"), default="policy")
    parser.add_argument("--tomogram", default="", help="Explicit tomogram.pickle for global planning")
    parser.add_argument(
        "--scenario",
        choices=("none", "corridor_gap"),
        default="none",
        help="Build and use a deterministic obstacle scene plus matching tomogram",
    )
    parser.add_argument("--scenario-dir", default="")
    parser.add_argument(
        "--use-live-costmap",
        action="store_true",
        help="Keep live map modules enabled for scenario runs; default scenario mode uses static tomogram only",
    )
    parser.add_argument("--output", default="artifacts/policy_nav_video.mp4")
    parser.add_argument("--summary-out", default="")
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--goal-distance", type=float, default=1.0)
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--fps", type=float, default=25.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--nav-max-angular-z", type=float, default=0.2)
    parser.add_argument("--nav-waypoint-threshold", type=float, default=0.25)
    parser.add_argument("--nav-final-waypoint-threshold", type=float, default=0.06)
    parser.add_argument("--nav-downsample-dist", type=float, default=0.25)
    parser.add_argument("--nav-path-goal-tolerance", type=float, default=0.08)
    parser.add_argument("--nav-path-min-speed", type=float, default=0.05)
    parser.add_argument("--nav-path-max-speed", type=float, default=0.25)
    parser.add_argument("--nav-safe-goal-tolerance", type=float, default=0.0)
    parser.add_argument("--success-settle", type=float, default=0.8)
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    world = args.world
    tomogram = args.tomogram
    goal_xy = None
    scenario_meta: dict[str, Any] | None = None
    if args.scenario == "corridor_gap":
        from sim.engine.scenarios.nav_corridor_assets import build_corridor_gap_assets

        scenario_dir = Path(args.scenario_dir) if args.scenario_dir else Path(args.output).parent / "corridor_gap"
        assets = build_corridor_gap_assets(scenario_dir)
        world = str(assets.scene_xml)
        tomogram = str(assets.tomogram)
        goal_xy = (float(assets.goal[0]), float(assets.goal[1]))
        scenario_meta = {
            "name": "corridor_gap",
            "scene_xml": str(assets.scene_xml),
            "tomogram": str(assets.tomogram),
            "map_pcd": str(assets.map_pcd),
            "metadata": str(assets.metadata),
            "start": list(assets.start),
            "goal": list(assets.goal),
        }
    if args.goal_x is not None or args.goal_y is not None:
        if args.goal_x is None or args.goal_y is None:
            raise SystemExit("--goal-x and --goal-y must be provided together")
        goal_xy = (float(args.goal_x), float(args.goal_y))
    result = record_full_stack_nav(
        world=world,
        policy_path=args.policy,
        drive_mode=args.drive_mode,
        tomogram=tomogram,
        goal_xy=goal_xy,
        enable_map_modules=(args.scenario == "none" or args.use_live_costmap),
        output=Path(args.output),
        goal_distance=args.goal_distance,
        duration=args.duration,
        fps=args.fps,
        width=args.width,
        height=args.height,
        waypoint_threshold=args.nav_waypoint_threshold,
        final_waypoint_threshold=args.nav_final_waypoint_threshold,
        downsample_dist=args.nav_downsample_dist,
        path_goal_tolerance=args.nav_path_goal_tolerance,
        path_min_speed=args.nav_path_min_speed,
        path_max_speed=args.nav_path_max_speed,
        safe_goal_tolerance=args.nav_safe_goal_tolerance,
        nav_max_angular_z=args.nav_max_angular_z,
        success_settle=args.success_settle,
    )
    if scenario_meta is not None:
        result["scenario"] = scenario_meta
    output = json.dumps(result, indent=2, sort_keys=True)
    print(output)
    if args.summary_out:
        Path(args.summary_out).write_text(output + "\n", encoding="utf-8")
    return 0 if result.get("success_seen") and Path(result["output"]).exists() else 1


if __name__ == "__main__":
    raise SystemExit(main())
