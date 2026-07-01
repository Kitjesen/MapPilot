#!/usr/bin/env python3
"""Run MuJoCo policy-mode gait and navigation smoke checks.

This script is intentionally simulation-only. It uses the in-process
sim_mujoco driver and never talks to real robot services.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import traceback
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.msgs.numpy_compat import np


def _rpy_from_xyzw(q: Any) -> tuple[float, float, float]:
    x, y, z, w = [float(v) for v in q]
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"min": None, "max": None, "avg": None}
    return {
        "min": float(min(values)),
        "max": float(max(values)),
        "avg": float(sum(values) / len(values)),
    }


def _command_stats(values: list[tuple[float, float]]) -> dict[str, Any]:
    if not values:
        return {
            "count": 0,
            "nonzero_count": 0,
            "linear_x": _stats([]),
            "abs_linear_x": _stats([]),
            "angular_z": _stats([]),
            "abs_angular_z": _stats([]),
        }
    linear = [float(v[0]) for v in values]
    angular = [float(v[1]) for v in values]
    return {
        "count": len(values),
        "nonzero_count": sum(1 for vx, wz in values if abs(vx) > 1e-4 or abs(wz) > 1e-4),
        "linear_x": _stats(linear),
        "abs_linear_x": _stats([abs(v) for v in linear]),
        "angular_z": _stats(angular),
        "abs_angular_z": _stats([abs(v) for v in angular]),
    }


_FOOT_BODY_NAMES = ("FR_foot", "FL_foot", "RR_foot", "RL_foot")


def _mujoco_name(mujoco: Any, model: Any, obj_type: Any, idx: int) -> str:
    try:
        name = mujoco.mj_id2name(model, obj_type, int(idx))
    except Exception:
        return ""
    return str(name or "")


def _contact_snapshot(engine: Any) -> dict[str, Any]:
    """Return one MuJoCo contact sample focused on physical gait stability."""
    model = getattr(engine, "model", None)
    data = getattr(engine, "data", None)
    if model is None or data is None:
        return {
            "available": False,
            "ncon": 0,
            "feet": [],
            "support_count": 0,
            "non_foot_ground_contacts": 0,
            "max_normal_force": 0.0,
        }
    try:
        import mujoco  # type: ignore
    except Exception:
        return {
            "available": False,
            "ncon": int(getattr(data, "ncon", 0) or 0),
            "feet": [],
            "support_count": 0,
            "non_foot_ground_contacts": 0,
            "max_normal_force": 0.0,
        }

    feet: set[str] = set()
    non_foot_ground_contacts = 0
    max_normal_force = 0.0
    ncon = int(getattr(data, "ncon", 0) or 0)
    for idx in range(ncon):
        try:
            contact = data.contact[idx]
        except Exception:
            break
        geom_ids = (int(contact.geom1), int(contact.geom2))
        body_ids = [int(model.geom_bodyid[geom_id]) for geom_id in geom_ids]
        body_names = [
            _mujoco_name(mujoco, model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            for body_id in body_ids
        ]
        foot_names = [
            body_name for body_name in body_names if body_name in _FOOT_BODY_NAMES
        ]
        touches_world = any(body_id == 0 for body_id in body_ids)
        if foot_names and touches_world:
            feet.update(foot_names)
            force = np.zeros(6, dtype=np.float64)
            try:
                mujoco.mj_contactForce(model, data, idx, force)
                max_normal_force = max(max_normal_force, abs(float(force[0])))
            except Exception:
                pass
        if touches_world and not foot_names:
            robot_body_contacts = [
                name for name, body_id in zip(body_names, body_ids)
                if body_id != 0 and name
            ]
            if robot_body_contacts:
                non_foot_ground_contacts += 1

    return {
        "available": True,
        "ncon": ncon,
        "feet": sorted(feet),
        "support_count": len(feet),
        "non_foot_ground_contacts": non_foot_ground_contacts,
        "max_normal_force": max_normal_force,
    }


def _contact_summary(samples: list[dict[str, Any]]) -> dict[str, Any]:
    if not samples:
        return {
            "sample_count": 0,
            "available_sample_count": 0,
            "contact_sample_count": 0,
            "foot_contact_sample_count": 0,
            "unique_feet": [],
            "unique_feet_count": 0,
            "per_foot_contact_samples": {},
            "support_count": _stats([]),
            "max_support_count": 0,
            "non_foot_ground_contacts": 0,
            "max_normal_force": 0.0,
        }

    support_counts: list[float] = []
    unique_feet: set[str] = set()
    per_foot: dict[str, int] = {name: 0 for name in _FOOT_BODY_NAMES}
    contact_sample_count = 0
    foot_contact_sample_count = 0
    available_sample_count = 0
    non_foot_ground_contacts = 0
    max_normal_force = 0.0
    for sample in samples:
        if bool(sample.get("available")):
            available_sample_count += 1
        if int(sample.get("ncon") or 0) > 0:
            contact_sample_count += 1
        feet = [str(name) for name in sample.get("feet") or []]
        if feet:
            foot_contact_sample_count += 1
        unique_feet.update(feet)
        for name in feet:
            per_foot[name] = per_foot.get(name, 0) + 1
        support_counts.append(float(sample.get("support_count") or 0))
        non_foot_ground_contacts += int(sample.get("non_foot_ground_contacts") or 0)
        max_normal_force = max(max_normal_force, float(sample.get("max_normal_force") or 0.0))

    return {
        "sample_count": len(samples),
        "available_sample_count": available_sample_count,
        "contact_sample_count": contact_sample_count,
        "foot_contact_sample_count": foot_contact_sample_count,
        "unique_feet": sorted(unique_feet),
        "unique_feet_count": len(unique_feet),
        "per_foot_contact_samples": {
            key: value for key, value in sorted(per_foot.items()) if value > 0
        },
        "support_count": _stats(support_counts),
        "max_support_count": int(max(support_counts)) if support_counts else 0,
        "non_foot_ground_contacts": non_foot_ground_contacts,
        "max_normal_force": max_normal_force,
    }


def _contact_stability_ok(result: dict[str, Any]) -> bool:
    contacts = result.get("contacts") or {}
    return (
        int(contacts.get("available_sample_count") or 0) > 0
        and int(contacts.get("foot_contact_sample_count") or 0) >= 3
        and int(contacts.get("unique_feet_count") or 0) >= 2
        and int(contacts.get("max_support_count") or 0) >= 2
        and int(contacts.get("non_foot_ground_contacts") or 0) == 0
        and float(contacts.get("max_normal_force") or 0.0) > 0.0
    )


def _xyz_from_any(value: Any) -> list[float] | None:
    if hasattr(value, "pose"):
        value = value.pose.position
    elif hasattr(value, "position"):
        value = value.position
    if all(hasattr(value, attr) for attr in ("x", "y")):
        return [
            float(value.x),
            float(value.y),
            float(getattr(value, "z", 0.0)),
        ]
    if isinstance(value, dict):
        if "position" in value:
            return _xyz_from_any(value["position"])
        try:
            return [
                float(value.get("x", 0.0)),
                float(value.get("y", 0.0)),
                float(value.get("z", 0.0)),
            ]
        except Exception:
            return None
    try:
        arr = np.asarray(value, dtype=float).reshape(-1)
    except Exception:
        return None
    if arr.size < 2 or not np.all(np.isfinite(arr[:2])):
        return None
    out = [0.0, 0.0, 0.0]
    for idx in range(min(arr.size, 3)):
        out[idx] = float(arr[idx])
    return out


def _path_summary(path: Any) -> dict[str, Any]:
    points_src = getattr(path, "poses", path)
    points: list[list[float]] = []
    try:
        iterator = iter(points_src)
    except TypeError:
        iterator = iter(())
    for item in iterator:
        point = _xyz_from_any(item)
        if point is not None:
            points.append(point)
    return {
        "count": len(points),
        "first": points[0] if points else None,
        "last": points[-1] if points else None,
        "frame_id": str(getattr(path, "frame_id", "") or ""),
    }


def _safe_float(value: Any) -> float:
    try:
        return float(value)
    except Exception:
        return 0.0


def _angle_delta_rad(start: float, end: float) -> float:
    return math.atan2(math.sin(end - start), math.cos(end - start))


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
        meta["input"] = [
            {"name": inp.name, "shape": list(inp.shape), "type": inp.type}
            for inp in sess.get_inputs()
        ]
        meta["output"] = [
            {"name": out.name, "shape": list(out.shape), "type": out.type}
            for out in sess.get_outputs()
        ]
    except Exception as exc:
        meta["onnx_error"] = str(exc)
    return meta


def _free_costmap_for_goal(
    start: tuple[float, float, float] | list[float],
    goal: tuple[float, float, float] | list[float],
    *,
    resolution: float,
    margin: float,
) -> dict[str, Any]:
    sx, sy = float(start[0]), float(start[1])
    gx, gy = float(goal[0]), float(goal[1])
    min_x = min(sx, gx) - margin
    max_x = max(sx, gx) + margin
    min_y = min(sy, gy) - margin
    max_y = max(sy, gy) + margin
    width = max(32, int(math.ceil((max_x - min_x) / resolution)) + 1)
    height = max(32, int(math.ceil((max_y - min_y) / resolution)) + 1)
    return {
        "grid": np.zeros((height, width), dtype=np.float32),
        "resolution": float(resolution),
        "origin": [float(min_x), float(min_y)],
        "height": height,
        "width": width,
        "frame_id": "map",
        "source": "policy_nav_smoke_free_space",
        "ts": time.time(),
    }


def _ensure_nav_planner_map(
    nav: Any,
    *,
    start: tuple[float, float, float] | list[float],
    goal: tuple[float, float, float] | list[float],
    wait_s: float,
    inject_if_missing: bool,
    resolution: float,
    margin: float,
) -> dict[str, Any]:
    deadline = time.time() + max(0.0, wait_s)
    while time.time() < deadline:
        if bool(getattr(nav._planner_svc, "has_map", False)):
            return {
                "planner_has_map": True,
                "source": "live_stack_costmap",
                "in_msg_count": int(getattr(nav.costmap, "msg_count", 0)),
                "in_deliver_count": int(getattr(nav.costmap, "deliver_count", 0)),
            }
        time.sleep(0.05)

    before_msg_count = int(getattr(nav.costmap, "msg_count", 0))
    if not inject_if_missing:
        return {
            "planner_has_map": bool(getattr(nav._planner_svc, "has_map", False)),
            "source": "missing",
            "in_msg_count": before_msg_count,
            "in_deliver_count": int(getattr(nav.costmap, "deliver_count", 0)),
        }

    injected = _free_costmap_for_goal(
        start,
        goal,
        resolution=resolution,
        margin=margin,
    )
    nav.costmap._deliver(injected)
    return {
        "planner_has_map": bool(getattr(nav._planner_svc, "has_map", False)),
        "source": "injected_free_space",
        "in_msg_count": int(getattr(nav.costmap, "msg_count", 0)),
        "in_deliver_count": int(getattr(nav.costmap, "deliver_count", 0)),
        "injected": {
            "resolution": float(injected["resolution"]),
            "origin": list(injected["origin"]),
            "height": int(injected["height"]),
            "width": int(injected["width"]),
            "frame_id": injected["frame_id"],
        },
        "in_msg_count_before_inject": before_msg_count,
    }


def _failed_check(mode: str, exc: BaseException, *, policy_path: str = "") -> dict[str, Any]:
    result = {
        "mode": mode,
        "passed": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "policy_loaded": False,
        "policy_backend": "unloaded",
        "policy_path": policy_path,
        "error": str(exc),
        "traceback": traceback.format_exc(limit=8),
    }
    result["policy"] = _load_policy_metadata(policy_path)
    return result


def run_direct_policy(
    *,
    world: str,
    duration: float,
    linear_x: float,
    angular_z: float,
    direct_mode: str = "walk",
    policy_path: str = "",
) -> dict[str, Any]:
    from drivers.sim.mujoco_driver_module import MujocoDriverModule
    from sim.engine.core.engine import VelocityCommand

    driver = MujocoDriverModule(
        world=world,
        render=False,
        enable_camera=False,
        drive_mode="policy",
        policy_path=policy_path,
    )
    driver.setup()
    engine = driver._engine
    if engine is None:
        raise RuntimeError("MujocoDriverModule setup failed")

    try:
        start = engine.get_robot_state()
        start_xy = np.array(start.position[:2], dtype=float)
        _, _, start_yaw = _rpy_from_xyzw(start.orientation)
        dt = max(0.001, _safe_float(engine.control_dt))
        steps = max(1, int(duration / dt))
        z_values: list[float] = []
        roll_values: list[float] = []
        pitch_values: list[float] = []
        speed_values: list[float] = []
        contact_samples: list[dict[str, Any]] = []
        finite = True
        ctrl_abs_max = 0.0
        if direct_mode == "stand":
            cmd_linear_x = 0.0
            cmd_angular_z = 0.0
        elif direct_mode == "turn":
            cmd_linear_x = 0.0
            cmd_angular_z = angular_z if abs(angular_z) > 1e-9 else 0.4
        else:
            cmd_linear_x = linear_x
            cmd_angular_z = angular_z
        cmd = VelocityCommand(linear_x=cmd_linear_x, angular_z=cmd_angular_z)

        for _ in range(steps):
            state = engine.step(cmd)
            contact_samples.append(_contact_snapshot(engine))
            finite = (
                finite
                and bool(np.isfinite(state.position).all())
                and bool(np.isfinite(state.orientation).all())
                and bool(np.isfinite(state.linear_velocity).all())
            )
            roll, pitch, _ = _rpy_from_xyzw(state.orientation)
            z_values.append(float(state.position[2]))
            roll_values.append(abs(roll))
            pitch_values.append(abs(pitch))
            speed_values.append(float(np.linalg.norm(state.linear_velocity[:2])))
            if engine.data is not None:
                finite = (
                    finite
                    and bool(np.isfinite(engine.data.qpos).all())
                    and bool(np.isfinite(engine.data.qvel).all())
                    and bool(np.isfinite(engine.data.ctrl).all())
                )
                ctrl_abs_max = max(ctrl_abs_max, float(np.max(np.abs(engine.data.ctrl))))

        end = engine.get_robot_state()
        _, _, end_yaw = _rpy_from_xyzw(end.orientation)
        moved = float(np.linalg.norm(np.array(end.position[:2], dtype=float) - start_xy))
        yaw_delta = _angle_delta_rad(start_yaw, end_yaw)
        return {
            "mode": "direct_policy",
            "direct_mode": direct_mode,
            "drive_mode": str(getattr(engine, "drive_mode", "")),
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "policy_backend": "onnx" if engine.has_policy else "unloaded",
            "world": world,
            "duration_s": duration,
            "command_linear_x": float(cmd_linear_x),
            "command_angular_z": float(cmd_angular_z),
            "policy_loaded": bool(engine.has_policy),
            "policy_path": str(driver._policy_path),
            "frames": {
                "command": "base_link",
                "odometry": "map",
                "cmd_vel": "base_link",
            },
            "finite": finite,
            "moved_m": moved,
            "yaw_delta_rad": float(yaw_delta),
            "yaw_delta_abs_rad": float(abs(yaw_delta)),
            "z": _stats(z_values),
            "roll_abs": _stats(roll_values),
            "pitch_abs": _stats(pitch_values),
            "speed_xy": _stats(speed_values),
            "contacts": _contact_summary(contact_samples),
            "ctrl_abs_max": ctrl_abs_max,
            "start": [float(v) for v in start.position[:3]],
            "end": [float(v) for v in end.position[:3]],
        }
    finally:
        if driver._engine is not None:
            driver._engine.close()
            driver._engine = None


def run_full_stack_nav(
    *,
    world: str,
    duration: float,
    goal_distance: float,
    policy_path: str = "",
    local_planner_backend: str = "simple",
    path_follower_backend: str = "pid",
    nav_max_angular_z: float = 0.2,
    waypoint_threshold: float = 0.25,
    final_waypoint_threshold: float = 0.06,
    downsample_dist: float = 0.25,
    path_goal_tolerance: float = 0.08,
    path_min_speed: float = 0.05,
    path_max_speed: float = 0.25,
    post_success_settle: float = 0.0,
    safe_goal_tolerance: float = 0.0,
    nav_costmap_wait: float = 2.0,
    inject_free_costmap: bool = True,
    free_costmap_resolution: float = 0.10,
    free_costmap_margin: float = 3.0,
) -> dict[str, Any]:
    from core.blueprints.profile_builder import build_system_for_profile
    from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3

    system = build_system_for_profile("sim", dict(
        robot="sim_mujoco",
        world=world,
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        render=False,
        python_autonomy_backend=local_planner_backend,
        python_path_follower_backend=path_follower_backend,
        drive_mode="policy",
        policy_path=policy_path,
        max_angular_vel=nav_max_angular_z,
        path_follower_max_yaw_rate=nav_max_angular_z,
        waypoint_threshold=waypoint_threshold,
        final_waypoint_threshold=final_waypoint_threshold,
        downsample_dist=downsample_dist,
        path_follower_goal_tolerance=path_goal_tolerance,
        path_follower_min_speed=path_min_speed,
        path_follower_max_speed=path_max_speed,
        safe_goal_tolerance=safe_goal_tolerance,
        run_startup_checks=False,
    ))

    driver = system.get_module("MujocoDriverModule")
    ogm = system.get_module("OccupancyGridModule")
    nav = system.get_module("NavigationModule")
    local_planner = system.get_module("LocalPlannerModule")
    path_follower = system.get_module("PathFollowerModule")
    mux = system.get_module("CmdVelMux")

    seen = {
        "costmap": 0,
        "waypoints": 0,
        "local_path": 0,
        "path_follower_cmd": 0,
        "mux_cmd": 0,
        "direct_fallback": 0,
    }
    odom: list[tuple[float, float, float]] = []
    last_mux: list[tuple[float, float]] = []
    path_follower_cmds: list[tuple[float, float]] = []
    global_path_summaries: list[dict[str, Any]] = []
    waypoint_points: list[list[float]] = []
    local_path_summaries: list[dict[str, Any]] = []
    control_hint_reasons: dict[str, int] = {}
    control_hint_safety_stop_count = 0
    last_control_hint: dict[str, Any] | None = None
    map_readiness: dict[str, Any] = {
        "planner_has_map": False,
        "source": "not_checked",
    }

    def _record_global_path(path: Any) -> None:
        global_path_summaries.append(_path_summary(path))

    def _record_waypoint(wp: Any) -> None:
        seen["waypoints"] += 1
        point = _xyz_from_any(wp)
        if point is not None:
            waypoint_points.append(point)

    def _record_local_path(path: Any) -> None:
        seen["local_path"] += 1
        local_path_summaries.append(_path_summary(path))

    def _record_control_hint(hint: Any) -> None:
        nonlocal control_hint_safety_stop_count, last_control_hint
        if not isinstance(hint, dict):
            return
        reason = str(hint.get("reason") or "")
        control_hint_reasons[reason] = control_hint_reasons.get(reason, 0) + 1
        if bool(hint.get("safety_stop") or hint.get("near_field_stop")):
            control_hint_safety_stop_count += 1
        last_control_hint = dict(hint)

    ogm.costmap._add_callback(lambda _: seen.__setitem__("costmap", seen["costmap"] + 1))
    nav.global_path._add_callback(_record_global_path)
    nav.waypoint._add_callback(_record_waypoint)
    nav.adapter_status._add_callback(
        lambda e: seen.__setitem__(
            "direct_fallback",
            seen["direct_fallback"] + (1 if e.get("event") == "direct_goal_fallback" else 0),
        )
    )
    local_planner.local_path._add_callback(_record_local_path)
    local_planner.control_hint._add_callback(_record_control_hint)
    path_follower.cmd_vel._add_callback(
        lambda m: (
            seen.__setitem__("path_follower_cmd", seen["path_follower_cmd"] + 1),
            path_follower_cmds.append((float(m.linear.x), float(m.angular.z))),
        )
    )
    mux.driver_cmd_vel._add_callback(
        lambda m: (
            seen.__setitem__("mux_cmd", seen["mux_cmd"] + 1),
            last_mux.append((float(m.linear.x), float(m.angular.z))),
        )
    )
    driver.odometry._add_callback(
        lambda m: odom.append(
            (float(m.pose.position.x), float(m.pose.position.y), float(m.pose.position.z))
        )
    )

    system.start()
    try:
        warm_deadline = time.time() + min(10.0, max(3.0, duration * 0.25))
        while time.time() < warm_deadline and (seen["costmap"] == 0 or not odom):
            time.sleep(0.1)
        if not odom:
            raise RuntimeError("No odometry from sim_mujoco policy mode")

        engine = driver._engine
        start = odom[-1]
        goal_x = start[0] + goal_distance
        goal_y = start[1]
        goal = (goal_x, goal_y, 0.0)
        map_readiness = _ensure_nav_planner_map(
            nav,
            start=start,
            goal=goal,
            wait_s=nav_costmap_wait,
            inject_if_missing=inject_free_costmap,
            resolution=free_costmap_resolution,
            margin=free_costmap_margin,
        )
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

        deadline = time.time() + duration
        started_at = time.time()
        success_at: float | None = None
        success_pose: tuple[float, float, float] | None = None
        dist_at_success: float | None = None
        moved_at_success: float | None = None
        moved = 0.0
        dist_to_goal = math.hypot(goal_x - start[0], goal_y - start[1])
        z_values: list[float] = []
        roll_values: list[float] = []
        pitch_values: list[float] = []
        contact_samples: list[dict[str, Any]] = []
        finite = True
        while time.time() < deadline:
            time.sleep(0.1)
            if odom:
                x, y, z = odom[-1]
                moved = math.hypot(x - start[0], y - start[1])
                dist_to_goal = math.hypot(goal_x - x, goal_y - y)
                z_values.append(z)
                finite = finite and all(math.isfinite(v) for v in odom[-1])
            if engine is not None:
                state = engine.get_robot_state()
                roll, pitch, _ = _rpy_from_xyzw(state.orientation)
                roll_values.append(abs(roll))
                pitch_values.append(abs(pitch))
                contact_samples.append(_contact_snapshot(engine))
                finite = (
                    finite
                    and bool(np.isfinite(state.position).all())
                    and bool(np.isfinite(state.orientation).all())
                    and bool(np.isfinite(state.linear_velocity).all())
                )
            if str(getattr(nav, "_state", "")) == "SUCCESS":
                if success_at is None:
                    success_at = time.time()
                    success_pose = odom[-1] if odom else None
                    dist_at_success = dist_to_goal
                    moved_at_success = moved
                if time.time() - success_at >= post_success_settle:
                    break

        return {
            "mode": "full_stack_policy_nav",
            "drive_mode": str(getattr(engine, "drive_mode", "")) if engine is not None else "",
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "world": world,
            "duration_s": duration,
            "elapsed_s": time.time() - started_at,
            "goal_distance_m": goal_distance,
            "local_planner_backend_requested": local_planner_backend,
            "local_planner_backend_actual": str(getattr(local_planner, "_backend", "")),
            "path_follower_backend_requested": path_follower_backend,
            "path_follower_backend_actual": str(getattr(path_follower, "_backend", "")),
            "nav_max_angular_z": nav_max_angular_z,
            "waypoint_threshold_m": waypoint_threshold,
            "final_waypoint_threshold_m": final_waypoint_threshold,
            "downsample_dist_m": downsample_dist,
            "path_goal_tolerance_m": path_goal_tolerance,
            "path_min_speed_mps": path_min_speed,
            "path_max_speed_mps": path_max_speed,
            "post_success_settle_s": post_success_settle,
            "safe_goal_tolerance_m": safe_goal_tolerance,
            "nav_costmap_wait_s": nav_costmap_wait,
            "inject_free_costmap_if_missing": inject_free_costmap,
            "costmap_readiness": map_readiness,
            "success_seen": success_at is not None,
            "dist_at_success_m": dist_at_success,
            "moved_at_success_m": moved_at_success,
            "success_end": [float(v) for v in success_pose[:3]] if success_pose else None,
            "settled_dist_delta_m": (
                float(dist_to_goal - dist_at_success) if dist_at_success is not None else None
            ),
            "policy_loaded": bool(engine.has_policy) if engine is not None else False,
            "policy_backend": "onnx" if engine is not None and engine.has_policy else "unloaded",
            "policy_path": str(driver._policy_path),
            "frames": {
                "goal": "map",
                "global_path": (
                    str((global_path_summaries[-1] or {}).get("frame_id", ""))
                    if global_path_summaries
                    else ""
                ),
                "waypoint": "map",
                "local_path": (
                    str((local_path_summaries[-1] or {}).get("frame_id", ""))
                    if local_path_summaries
                    else ""
                ),
                "cmd_vel": "base_link",
                "driver_cmd_vel": "base_link",
                "odometry": "map",
            },
            "finite": finite,
            "seen": seen,
            "local_path_nonempty_count": sum(
                1 for summary in local_path_summaries if int(summary.get("count") or 0) >= 2
            ),
            "local_path_empty_count": sum(
                1 for summary in local_path_summaries if int(summary.get("count") or 0) < 2
            ),
            "global_path": global_path_summaries[-1] if global_path_summaries else None,
            "last_waypoint": waypoint_points[-1] if waypoint_points else None,
            "last_nonempty_local_path": next(
                (
                    summary
                    for summary in reversed(local_path_summaries)
                    if int(summary.get("count") or 0) >= 2
                ),
                None,
            ),
            "last_local_path": local_path_summaries[-1] if local_path_summaries else None,
            "control_hint_reasons": control_hint_reasons,
            "control_hint_safety_stop_count": control_hint_safety_stop_count,
            "last_control_hint": last_control_hint,
            "path_follower_cmd_stats": _command_stats(path_follower_cmds),
            "mux_cmd_stats": _command_stats(last_mux),
            "moved_m": moved,
            "dist_to_goal_m": dist_to_goal,
            "z": _stats(z_values),
            "roll_abs": _stats(roll_values),
            "pitch_abs": _stats(pitch_values),
            "contacts": _contact_summary(contact_samples),
            "nav_state": str(getattr(nav, "_state", "")),
            "last_mux": list(last_mux[-1]) if last_mux else None,
            "start": [float(v) for v in start[:3]],
            "end": [float(v) for v in odom[-1][:3]] if odom else None,
        }
    finally:
        system.stop()


def _passes_direct_common(result: dict[str, Any]) -> bool:
    return (
        result.get("drive_mode") == "policy"
        and bool(result.get("policy_loaded"))
        and bool(result.get("finite"))
        and (result.get("z", {}).get("min") or 0.0) > 0.35
        and (result.get("z", {}).get("max") or 999.0) < 0.55
        and (result.get("roll_abs", {}).get("max") or 999.0) < 0.35
        and (result.get("pitch_abs", {}).get("max") or 999.0) < 0.35
        and _contact_stability_ok(result)
    )


def _passes_direct(
    result: dict[str, Any],
    min_motion: float,
    *,
    direct_mode: str = "walk",
    max_stand_drift: float = 0.05,
    min_turn_yaw: float = 0.35,
    max_turn_drift: float = 0.10,
) -> bool:
    if not _passes_direct_common(result):
        return False
    moved = float(result.get("moved_m", 0.0))
    if direct_mode == "stand":
        return moved <= max_stand_drift
    if direct_mode == "turn":
        yaw_delta = float(
            result.get(
                "yaw_delta_abs_rad",
                abs(float(result.get("yaw_delta_rad", 0.0))),
            )
        )
        return yaw_delta >= min_turn_yaw and moved <= max_turn_drift
    return moved >= min_motion


def _passes_nav(result: dict[str, Any], min_motion: float, max_dist_to_goal: float) -> bool:
    seen = result.get("seen", {})
    map_ready = int(seen.get("costmap", 0)) > 0 or bool(
        (result.get("costmap_readiness") or {}).get("planner_has_map")
    )
    dist_for_gate = result.get("dist_at_success_m")
    if dist_for_gate is None:
        dist_for_gate = result.get("dist_to_goal_m", 999.0)
    close_enough = bool(result.get("success_seen")) or float(dist_for_gate) <= max_dist_to_goal
    return (
        result.get("drive_mode") == "policy"
        and bool(result.get("policy_loaded"))
        and bool(result.get("finite"))
        and map_ready
        and close_enough
        and int(seen.get("waypoints", 0)) > 0
        and int(seen.get("local_path", 0)) > 0
        and int(seen.get("path_follower_cmd", 0)) > 3
        and int(seen.get("mux_cmd", 0)) > 3
        and int(seen.get("direct_fallback", 0)) == 0
        and float(result.get("moved_m", 0.0)) >= min_motion
        and (result.get("z", {}).get("min") or 0.0) > 0.35
        and (result.get("z", {}).get("max") or 999.0) < 0.55
        and (result.get("roll_abs", {}).get("max") or 999.0) < 0.35
        and (result.get("pitch_abs", {}).get("max") or 999.0) < 0.35
        and _contact_stability_ok(result)
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="open_field")
    parser.add_argument("--direct-duration", type=float, default=6.0)
    parser.add_argument("--nav-duration", type=float, default=18.0)
    parser.add_argument("--goal-distance", type=float, default=1.0)
    parser.add_argument("--linear-x", type=float, default=0.2)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument(
        "--direct-mode",
        choices=("walk", "stand", "turn"),
        default="walk",
        help="Direct policy gate mode: walk motion, stand drift, or turn yaw response",
    )
    parser.add_argument(
        "--policy",
        "--policy-path",
        dest="policy",
        default="",
        help="Explicit ONNX policy path. If omitted, MujocoDriverModule searches its default candidates.",
    )
    parser.add_argument(
        "--nav-local-planner-backend",
        choices=("simple", "nanobind", "cmu", "cmu_py"),
        default="simple",
    )
    parser.add_argument(
        "--nav-path-follower-backend",
        choices=("pid", "nav_core", "pure_pursuit"),
        default="pid",
    )
    parser.add_argument(
        "--nav-max-angular-z",
        type=float,
        default=0.2,
        help="Full-stack policy-mode angular velocity clamp in rad/s",
    )
    parser.add_argument("--nav-waypoint-threshold", type=float, default=0.25)
    parser.add_argument("--nav-final-waypoint-threshold", type=float, default=0.06)
    parser.add_argument("--nav-downsample-dist", type=float, default=0.25)
    parser.add_argument("--nav-path-goal-tolerance", type=float, default=0.08)
    parser.add_argument("--nav-path-min-speed", type=float, default=0.05)
    parser.add_argument("--nav-path-max-speed", type=float, default=0.25)
    parser.add_argument("--nav-post-success-settle", type=float, default=0.0)
    parser.add_argument("--nav-safe-goal-tolerance", type=float, default=0.0)
    parser.add_argument(
        "--nav-costmap-wait",
        type=float,
        default=2.0,
        help="Wait for NavigationModule planner map after odometry/costmap warmup",
    )
    parser.add_argument(
        "--no-nav-inject-free-costmap",
        action="store_true",
        help="Disable simulation-only free-space costmap injection when live map is late",
    )
    parser.add_argument("--nav-free-costmap-resolution", type=float, default=0.10)
    parser.add_argument("--nav-free-costmap-margin", type=float, default=3.0)
    parser.add_argument("--min-direct-motion", type=float, default=0.20)
    parser.add_argument("--max-stand-drift", type=float, default=0.05)
    parser.add_argument("--min-turn-yaw", type=float, default=0.35)
    parser.add_argument("--max-turn-drift", type=float, default=0.10)
    parser.add_argument("--min-nav-motion", type=float, default=0.20)
    parser.add_argument("--max-nav-dist-to-goal", type=float, default=0.10)
    parser.add_argument("--direct-only", action="store_true")
    parser.add_argument("--nav-only", action="store_true")
    parser.add_argument("--json-out", default="")
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()

    if args.direct_only and args.nav_only:
        parser.error("--direct-only and --nav-only are mutually exclusive")

    results: dict[str, Any] = {
        "schema_version": "lingtu.policy_nav_smoke.v1",
        "world": args.world,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "checks": [],
    }
    direct_result = None
    nav_result = None
    if not args.nav_only:
        try:
            direct_result = run_direct_policy(
                world=args.world,
                duration=args.direct_duration,
                linear_x=args.linear_x,
                angular_z=args.angular_z,
                direct_mode=args.direct_mode,
                policy_path=args.policy,
            )
            direct_result["passed"] = _passes_direct(
                direct_result,
                args.min_direct_motion,
                direct_mode=args.direct_mode,
                max_stand_drift=args.max_stand_drift,
                min_turn_yaw=args.min_turn_yaw,
                max_turn_drift=args.max_turn_drift,
            )
            direct_result["policy"] = _load_policy_metadata(str(direct_result.get("policy_path", "")))
        except Exception as exc:
            direct_result = _failed_check("direct_policy", exc, policy_path=args.policy)
        results["checks"].append(direct_result)

    if not args.direct_only:
        try:
            nav_result = run_full_stack_nav(
                world=args.world,
                duration=args.nav_duration,
                goal_distance=args.goal_distance,
                policy_path=args.policy,
                local_planner_backend=args.nav_local_planner_backend,
                path_follower_backend=args.nav_path_follower_backend,
                nav_max_angular_z=args.nav_max_angular_z,
                waypoint_threshold=args.nav_waypoint_threshold,
                final_waypoint_threshold=args.nav_final_waypoint_threshold,
                downsample_dist=args.nav_downsample_dist,
                path_goal_tolerance=args.nav_path_goal_tolerance,
                path_min_speed=args.nav_path_min_speed,
                path_max_speed=args.nav_path_max_speed,
                post_success_settle=args.nav_post_success_settle,
                safe_goal_tolerance=args.nav_safe_goal_tolerance,
                nav_costmap_wait=args.nav_costmap_wait,
                inject_free_costmap=not args.no_nav_inject_free_costmap,
                free_costmap_resolution=args.nav_free_costmap_resolution,
                free_costmap_margin=args.nav_free_costmap_margin,
            )
            nav_result["passed"] = _passes_nav(
                nav_result,
                args.min_nav_motion,
                args.max_nav_dist_to_goal,
            )
            nav_result["policy"] = _load_policy_metadata(str(nav_result.get("policy_path", "")))
        except Exception as exc:
            nav_result = _failed_check("full_stack_policy_nav", exc, policy_path=args.policy)
        results["checks"].append(nav_result)

    results["passed"] = all(bool(check.get("passed")) for check in results["checks"])
    output = json.dumps(results, indent=2, sort_keys=True)
    print(output)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(output + "\n", encoding="utf-8")
    return 0 if results["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
