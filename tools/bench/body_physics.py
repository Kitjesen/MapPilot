"""Exercise native body-route execution against the packaged Thunder policy.

Python owns MuJoCo physics and evidence only. Native C++ owns the route,
SCAN, follower and checked commands. This is Thunder component evidence,
not Go2 hardware evidence or a complete Product run.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from sim.compat.engine.core.engine import VelocityCommand
from sim.scripts.mujoco.stair_policy_probe import rpy
from tools.bench.body_export import native_body_text
from tools.bench.body_scenarios import prepare_world, read_dynamic_boxes

from drivers.sim.mujoco.runtime import build_engine

WORLD = ROOT / "sim/compat/engine/worlds/flat_showcase.xml"
# Keep this historical body-route comparison fixed to its measured 4998 baseline.
POLICY = ROOT / "sim/packages/controllers/doso/thunder_v4/locomotion/policy/policy_4998.onnx"
ROBOT = ROOT / "sim/packages/robots/doso/thunder_v4/mjcf/thunderv4.xml"
THUNDER_SUPPORT_BODIES = ("FL_foot", "FR_foot", "RL_foot", "RR_foot")
DEFAULT_GOAL_OFFSET = (1.2, 0.0)
DEFAULT_GOAL_YAW_OFFSET_DEG = 90.0


def requested_route(position: Any, yaw: float, goal_offset: Any = DEFAULT_GOAL_OFFSET,
                    goal_yaw_offset_deg: float = DEFAULT_GOAL_YAW_OFFSET_DEG) -> list[list[float]]:
    """Keep measured start, world-XY translation, and final in-place heading."""
    if not all(math.isfinite(value) for value in (*position, yaw, *goal_offset, goal_yaw_offset_deg)):
        raise ValueError("Route position, heading, and offsets must be finite")
    dx, dy = goal_offset
    start = [float(value) for value in position]
    endpoint = [start[0] + dx, start[1] + dy, start[2]]
    return [[*start, yaw], [*endpoint, yaw], [*endpoint, yaw + math.radians(goal_yaw_offset_deg)]]


def robot_body_ids(model: Any) -> set[int]:
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    result = set()
    for body in range(1, model.nbody):
        ancestor = body
        while ancestor > 0 and ancestor != base:
            ancestor = int(model.body_parentid[ancestor])
        if ancestor == base:
            result.add(body)
    return result


def contacts(engine: Any, support_bodies: tuple[str, ...] = THUNDER_SUPPORT_BODIES,
             robot_ids: set[int] | None = None) -> dict[str, Any]:
    """Separate permissible sole contact, loaded support, and world collisions."""
    model, data = engine.model, engine.data
    robot_ids = robot_ids if robot_ids is not None else robot_body_ids(model)
    feet, support_geoms, blocked = set(), set(), []
    max_force = 0.0
    for index in range(data.ncon):
        contact = data.contact[index]
        geoms = (int(contact.geom1), int(contact.geom2))
        bodies = tuple(int(model.geom_bodyid[g]) for g in geoms)
        robot_side = tuple(b in robot_ids for b in bodies)
        if robot_side[0] == robot_side[1]:
            continue
        side = 0 if robot_side[0] else 1
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bodies[side]) or ""
        normal = np.asarray(contact.frame[:3]) * (-1 if side == 0 else 1)
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, force)
        max_force = max(max_force, float(force[0]))
        if float(force[0]) <= 1e-3 and float(contact.dist) > 0:
            continue
        if name in support_bodies and normal[2] >= .98 and contact.dist >= -.005:
            # An unloading sole may retain legal soft contact with zero force.
            # It is allowed to touch, but must not be required as load bearing.
            if float(force[0]) > 1e-3:
                feet.add(name)
                support_geoms.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM,
                                                   geoms[1 - side]) or "")
        else:
            blocked.append({"body": name, "geom_ids": list(geoms),
                            "geom_names": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ""
                                           for geom in geoms],
                            "position_m": contact.pos.tolist(),
                            "efc_address": int(contact.efc_address), "exclude": int(contact.exclude),
                            "normal": normal.tolist(), "distance_m": float(contact.dist),
                            "normal_force_n": float(force[0])})
    return {"feet": sorted(feet), "support_geoms": sorted(support_geoms),
            "non_foot_contacts": blocked, "max_normal_force_n": max_force}


def part_snapshot(engine: Any, parts: list[dict[str, Any]], state: Any) -> list[float]:
    """Bake current joints and roll/pitch into a yaw-only native body frame."""
    # mj_step has integrated qpos after computing its geom transforms.
    mujoco.mj_kinematics(engine.model, engine.data)
    yaw = rpy(state.orientation)[2]
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])
    ids = [p["geom_id"] for p in parts]
    centers = (np.asarray(engine.data.geom_xpos[ids]) - state.position) @ rotation
    axes = np.einsum("ij,njk->nik", rotation.T,
                     np.asarray(engine.data.geom_xmat[ids]).reshape(-1, 3, 3))
    result = []
    for part, center, matrix in zip(parts, centers, axes):
        if part["source_type"] == "sphere":
            matrix = np.eye(3)
        result.extend([*center, *matrix.ravel(), 0])
    return result


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def capture_bad_substep(engine: Any, output: Path, blocked: list[dict[str, Any]]) -> None:
    """Retain failure state and the exact cached geometry used by contacts."""
    model, data = engine.model, engine.data
    geom_ids = sorted({geom for contact in blocked for geom in contact["geom_ids"]})
    write_json(output, {
        "mujoco_version": mujoco.__version__, "sim_time_s": float(engine.sim_time),
        "data_time_s": float(data.time), "qpos": data.qpos.tolist(), "qvel": data.qvel.tolist(),
        "ctrl": data.ctrl.tolist(), "qacc_warmstart": data.qacc_warmstart.tolist(),
        "mocap_pos": data.mocap_pos.tolist(), "mocap_quat": data.mocap_quat.tolist(),
        "disableflags": int(model.opt.disableflags), "enableflags": int(model.opt.enableflags),
        "contacts": blocked,
        "contact_geometries": [{"id": geom, "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom),
                                "type": int(model.geom_type[geom]), "size": model.geom_size[geom].tolist(),
                                "position": data.geom_xpos[geom].tolist(),
                                "rotation": data.geom_xmat[geom].reshape(3, 3).tolist()}
                               for geom in geom_ids],
        "scope": "first failed physical substep; qpos/qvel after integration, cached contact geom transforms retained separately because mj_step contacts can precede final qpos integration; no pose inference",
    })


def export_body(engine: Any, output: Path,
                support_bodies: tuple[str, ...] = THUNDER_SUPPORT_BODIES) -> dict[str, Any]:
    """Keep every physical robot geom and enclose each primitive conservatively."""
    model, data = engine.model, engine.data
    mujoco.mj_kinematics(model, data)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    state = engine.get_robot_state()
    yaw = rpy(state.orientation)[2]
    cosine, sine = math.cos(yaw), math.sin(yaw)
    base_rotation = np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])
    base_position = np.asarray(state.position)
    parts: list[dict[str, Any]] = []
    for geom in range(model.ngeom):
        body = int(model.geom_bodyid[geom])
        ancestor = body
        while ancestor > 0 and ancestor != base:
            ancestor = int(model.body_parentid[ancestor])
        if ancestor != base or not (model.geom_contype[geom] or model.geom_conaffinity[geom]):
            continue
        kind = int(model.geom_type[geom])
        size = np.asarray(model.geom_size[geom], dtype=float)
        rotation = base_rotation.T @ np.asarray(data.geom_xmat[geom]).reshape(3, 3)
        center = base_rotation.T @ (np.asarray(data.geom_xpos[geom]) - base_position)
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            half = size.copy()
            source = "box"
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            half = np.array([size[0], size[0], size[1]])
            source = "cylinder"
        elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            half = np.repeat(size[0], 3)
            rotation = np.eye(3)
            source = "sphere"
        elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
            half = np.array([size[0], size[0], size[0] + size[1]])
            source = "capsule"
        else:
            raise ValueError(f"Robot collision geom {geom} has unsupported primitive type {kind}")
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
        extent = np.abs(rotation) @ half
        exact_world_extent = np.abs(np.asarray(data.geom_xmat[geom]).reshape(3, 3)) @ half
        if source == "cylinder":
            world_rotation = np.asarray(data.geom_xmat[geom]).reshape(3, 3)
            exact_world_extent = (size[0] * np.sqrt(np.sum(world_rotation[:, :2] ** 2, axis=1))
                                  + size[1] * np.abs(world_rotation[:, 2]))
        elif source == "sphere":
            exact_world_extent = np.repeat(size[0], 3)
        parts.append({"link": f"{name}_{geom}", "body": name, "geom_id": geom,
                      "source_type": source, "center": center.tolist(),
                      "rotation": rotation.tolist(), "half_size": half.tolist(),
                      "radius_m": float(size[0]) if source != "box" else 0.0,
                      "half_length_m": float(size[1]) if source in {"cylinder", "capsule"} else 0.0,
                      "support_contact": name in support_bodies,
                      "local_min": (center - extent).tolist(),
                      "local_max": (center + extent).tolist(),
                      "primitive_world_min_z_m": float(data.geom_xpos[geom][2] - exact_world_extent[2])})
    output.write_text(native_body_text(parts), encoding="utf-8")
    minimum = min(p["local_min"][2] for p in parts)
    report = {
        "robot": "doso/thunder_v4", "robot_xml": str(ROBOT), "policy": str(POLICY),
        "world": str(WORLD), "primitive_count": len(parts), "parts": parts,
        "frame": "yaw-only frame at measured state.position; roll/pitch included in row-major part axes",
        "snapshot_time_s": float(engine.sim_time), "base_position": base_position.tolist(),
        "qpos": data.qpos.tolist(), "qvel": data.qvel.tolist(),
        "minimum_local_z_m": minimum,
        "primitive_minimum_world_z_m": min(p["primitive_world_min_z_m"] for p in parts),
        "minimum_world_z_yaw_only_m": float(base_position[2]) + minimum,
        "minimum_world_z_with_20mm_local_margin_m": min(
            float(base_position[2]) + p["center"][2]
            - float(np.abs(np.asarray(p["rotation"])[2]) @ (np.asarray(p["half_size"]) + 0.02))
            for p in parts),
        "contacts": contacts(engine, support_bodies), "physical_drive": "ONNX policy + MuJoCo dynamics",
        "support_contact_bodies": list(support_bodies),
        "scope": "Thunder component test; runtime joint and roll/pitch snapshot; no Go2 policy or field deployment",
    }
    write_json(output.with_suffix(".json"), report)
    return report


def native_reader(stream: Any, pending: queue.Queue, log: Path) -> None:
    with log.open("w", encoding="utf-8") as output:
        for line in stream:
            output.write(line)
            output.flush()
            if line.startswith("@command "):
                try:
                    pending.put(json.loads(line[len("@command "):]))
                except json.JSONDecodeError as exc:
                    pending.put(exc)
        pending.put(EOFError("Native executor closed stdout"))


def render(engine: Any, renderer: Any, writer: Any, *, route: Any = None,
           measured: Any = (), requested: Any = (), elapsed_s: float = 0.0) -> None:
    import cv2
    from tools.bench.body_motion_view import draw_route_overlay, hide_showcase_markers

    hide_showcase_markers(engine.model)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.6, 0.0, 0.3]
    camera.distance, camera.azimuth, camera.elevation = 3.2, 90, -32
    options = mujoco.MjvOption()
    options.geomgroup[5] = 1
    renderer.update_scene(engine.data, camera=camera, scene_option=options)
    draw_route_overlay(renderer, route, np.asarray(measured), np.asarray(requested))
    frame = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
    label = "Blue: native global route" if route is not None else "Requested points only; native global route pending"
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 52), (28, 34, 38), -1)
    cv2.putText(frame, f"Actual policy physics | t={elapsed_s:.2f} s | {label}", (14, 21),
                cv2.FONT_HERSHEY_SIMPLEX, .45, (235, 237, 240), 1, cv2.LINE_AA)
    cv2.putText(frame, "Green: measured base trail | route and trail projected onto floor", (14, 43),
                cv2.FONT_HERSHEY_SIMPLEX, .43, (205, 216, 224), 1, cv2.LINE_AA)
    writer.write(frame)


def run(args: argparse.Namespace) -> dict[str, Any]:
    args.out.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    cpu_started = time.process_time()
    wall_timing_s: dict[str, float] = defaultdict(float)
    native_environment = None
    native_control_overrides = {}
    robot_xml, policy = ROBOT, POLICY
    if args.native is not None:
        from lingtu.assembly.compile_plan import compile_run_plan

        # Geometry selection must not also select another environment or controller tuning.
        plan = compile_run_plan("nav", "sim", robot="doso/thunder_v4", local_planner="scan",
                                env_config={"backend": "mujoco", "localization": "truth"})
        simulation = plan.simulation
        robot_xml = ROOT / simulation["physics_plan"]["robots"][0]["model"]["mjcf"]
        policy = ROOT / simulation["control_plan"]["controllers"][0]["policy"]["artifact"]
        native_environment = dict(os.environ)
        native_environment.pop("LINGTU_NAV_BODY_FILE", None)
        plan_environment = plan.native_process_environment
        native_environment.update(plan_environment)
        if args.navigation_body == "obb":
            native_environment.pop("LINGTU_NAV_BODY_FILE", None)
        native_control_overrides = {
            key: value for key, value in native_environment.items()
            if key.startswith(("LINGTU_NAV_", "LINGTU_TELEOP_")) and plan_environment.get(key) != value
        }
        write_json(args.out / "navigation-config.json", {
            "run_plan": plan.as_dict(), "environment": plan_environment,
            "navigation_body": args.navigation_body,
            "effective_body_file": native_environment.get("LINGTU_NAV_BODY_FILE"),
            "native_control_overrides": native_control_overrides,
            "scope": "component fixture overrides world, spawn, and sensor input; no Product lifecycle is started",
        })
    dynamic_boxes = read_dynamic_boxes(getattr(args, "dynamic_obstacles", None))
    effective_world = prepare_world(args.world, args.scene, dynamic_boxes, args.out / "effective-world.xml")
    phase_started = time.perf_counter()
    engine = build_engine(world=effective_world, drive_mode="policy", start=[0.0, 0.0, 0.58],
                          mujoco_memory="64M", robot_xml=robot_xml, policy_path=policy, policy_cpu_threads=1,
                          lidar_backend="mujoco_lidar", lidar_body_name="lidar1_link",
                          lidar_site_name="lidar1_link_site", mid360_samples_per_frame=32)
    if args.ccd_tolerance is not None:
        engine.model.opt.ccd_tolerance = args.ccd_tolerance
    wall_timing_s["engine_initialization"] += time.perf_counter() - phase_started
    process = renderer = writer = None
    render_route = None
    render_route_loaded = False
    samples: list[dict[str, Any]] = []
    physics_steps = blocked_steps = unloaded_frames = 0
    first_bad_substep_captured = False
    snapshot_costs_ms: list[float] = []
    result = "initialization_only"
    try:
        mocap_ids = []
        for box in dynamic_boxes:
            body_id = mujoco.mj_name2id(engine.model, mujoco.mjtObj.mjOBJ_BODY, box.name)
            if body_id < 0 or int(engine.model.body_mocapid[body_id]) < 0:
                raise ValueError(f"Fixture mocap body missing: {box.name}")
            mocap_ids.append(int(engine.model.body_mocapid[body_id]))

        def update_obstacles(elapsed_s: float) -> list[dict[str, Any]]:
            observed = []
            for box, mocap_id in zip(dynamic_boxes, mocap_ids):
                position, velocity = box.at(elapsed_s)
                engine.data.mocap_pos[mocap_id] = position
                observed.append({"name": box.name, "position": position, "half_size": list(box.half_size),
                                 "velocity": velocity})
            if dynamic_boxes:
                mujoco.mj_forward(engine.model, engine.data)
            return observed

        update_obstacles(0.0)
        zero = VelocityCommand(0.0, 0.0, 0.0)
        phase_started = time.perf_counter()
        for _ in range(round(args.settle / args.dt)):
            engine.step_sensor_tick(zero, args.dt)
        wall_timing_s["initial_settle"] += time.perf_counter() - phase_started
        phase_started = time.perf_counter()
        report = export_body(engine, args.out / "body.txt")
        report.update(world=str(effective_world), robot_xml=str(robot_xml), policy=str(policy))
        write_json(args.out / "body.json", report)
        parts = report["parts"]
        body_ids = robot_body_ids(engine.model)
        base_body_id = mujoco.mj_name2id(engine.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        state = engine.get_robot_state()
        yaw = rpy(state.orientation)[2]
        route = requested_route(state.position, yaw, args.goal_offset, args.goal_yaw_offset_deg)
        if args.waypoints:
            offsets = json.loads(args.waypoints.read_text(encoding="utf-8-sig"))
            if len(offsets) < 2 or offsets[0] != [0, 0, 0]:
                raise ValueError("Waypoints must start at [0,0,0] and contain at least two points")
            route = []
            for dx, dy, heading_deg in offsets:
                if not all(math.isfinite(value) for value in (dx, dy, heading_deg)):
                    raise ValueError("Waypoint offsets must be finite")
                route.append([float(state.position[0]) + dx, float(state.position[1]) + dy,
                              float(state.position[2]), yaw + math.radians(heading_deg)])
        route_path = args.out / "route.txt"
        route_path.write_text(str(len(route)) + "\n" + "\n".join(
            " ".join(format(v, ".17g") for v in pose) for pose in route) + "\n", encoding="utf-8")
        write_json(args.out / "initialization.json", {**report, "requested_route": route,
                                                      "rpy": rpy(state.orientation)})
        wall_timing_s["body_export_and_request"] += time.perf_counter() - phase_started
        if args.native is None:
            timings = []
            for _ in range(100):
                snapshot_started = time.perf_counter()
                part_snapshot(engine, parts, state)
                timings.append((time.perf_counter() - snapshot_started) * 1000)
            return {"result": result, "parts": report["primitive_count"],
                    "base_z_m": float(state.position[2]),
                    "actual_primitive_minimum_world_z_m": report["primitive_minimum_world_z_m"],
                    "loaded_support_bodies": report["contacts"]["feet"],
                    "non_foot_world_contacts": len(report["contacts"]["non_foot_contacts"]),
                    "snapshot_cost_ms": {"mean": float(np.mean(timings)),
                                         "p95": float(np.percentile(timings, 95)), "max": max(timings)},
                    "body_floor_minimum_m": report["minimum_world_z_yaw_only_m"],
                    "body_floor_minimum_with_margin_m": report["minimum_world_z_with_20mm_local_margin_m"]}
        command = [str(args.native), "--map", str(args.map), "--body", str(args.out / "body.txt"),
                   "--route", str(route_path), "--out", str(args.out / "native"), "--external",
                   "--yaw-tolerance-deg", str(args.goal_yaw_tolerance_deg)]
        if args.navigation_body == "obb":
            command.append("--obb")
        if args.use_route:
            command.append("--use-route")
        if args.keep_turn_stages:
            command.append("--keep-turn-stages")
        if args.scene:
            command += ["--scene", str(args.scene)]
        write_json(args.out / "invocation.json", {"command": command, "dt_s": args.dt,
                                                  "native_control_overrides": native_control_overrides,
                                                  "physics_dt_s": engine.dt,
                                                  "robot": "doso/thunder_v4",
                                                  "robot_xml": str(robot_xml),
                                                  "policy": str(policy),
                                                  "world": str(args.world),
                                                  "effective_world": str(effective_world),
                                                    "scene_geometry": ("native scene boxes; flat floor uses equivalent bounded flat heightfield" if args.scene
                                                                       else "world geometry and separately loaded native map; qualify only the inspected common scene extent"),
                                                    "physics_collision_backend": ("libccd" if int(engine.model.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD) else "nativeccd"),
                                                    "physics_ccd_tolerance": float(engine.model.opt.ccd_tolerance),
                                                  "dynamic_obstacles": str(args.dynamic_obstacles) if getattr(args, "dynamic_obstacles", None) else None,
                                                  "obstacle_input_scope": "deterministic fixture ground truth; not LiDAR detections",
                                                  "scene": str(args.scene) if args.scene else None,
                                                  "waypoints": str(args.waypoints) if args.waypoints else None,
                                                  "settle_s": args.settle,
                                                  "mujoco_version": mujoco.__version__,
                                                  "wall_paced": args.pace,
                                                  "navigation_body": args.navigation_body,
                                                  "goal_yaw_tolerance_deg": args.goal_yaw_tolerance_deg,
                                                  "duration_s": args.duration})
        phase_started = time.perf_counter()
        native_started = phase_started
        first_reply_wall_s = first_motion_wall_s = None
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, encoding="utf-8", bufsize=1,
                                   env=native_environment)
        pending: queue.Queue = queue.Queue()
        reader = threading.Thread(target=native_reader, args=(process.stdout, pending, args.out / "native.log"), daemon=True)
        reader.start()
        wall_timing_s["native_spawn"] += time.perf_counter() - phase_started
        video_error = ""
        phase_started = time.perf_counter()
        if args.video:
            try:
                import cv2

                renderer = mujoco.Renderer(engine.model, height=540, width=960)
                writer = cv2.VideoWriter(str(args.out / "physics.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (960, 540))
                if not writer.isOpened():
                    raise RuntimeError("MP4 writer could not open")
            except Exception as exc:
                video_error = f"{type(exc).__name__}: {exc}"
        wall_timing_s["video_setup"] += time.perf_counter() - phase_started
        result = "duration_elapsed"
        execution_start_time = float(engine.sim_time)
        terminal: dict[str, Any] | None = None
        terminal_time = 0.0
        for tick in range(math.ceil(args.duration / args.dt)):
            tick_started = time.perf_counter()
            phase_started = time.perf_counter()
            obstacle_observations = update_obstacles(float(engine.sim_time) - execution_start_time)
            state = engine.get_robot_state()
            angle = rpy(state.orientation)
            current_contacts = contacts(engine, robot_ids=body_ids)
            wall_timing_s["observation_and_current_contacts"] += time.perf_counter() - phase_started
            snapshot_started = time.perf_counter()
            snapshot = (part_snapshot(engine, parts, state)
                        if args.navigation_body == "parts" else angle[:2])
            snapshot_elapsed = time.perf_counter() - snapshot_started
            snapshot_costs_ms.append(snapshot_elapsed * 1000)
            wall_timing_s["body_snapshot"] += snapshot_elapsed
            unloaded_frames += not bool(current_contacts["feet"])
            # Free-joint angular qvel is expressed in the rotating body frame.
            world_angular = np.asarray(engine.data.xmat[base_body_id]).reshape(3, 3) @ state.angular_velocity
            fields = [float(engine.sim_time), *state.position, angle[2],
                      *state.linear_velocity, float(world_angular[2])]
            if terminal is None:
                phase_started = time.perf_counter()
                write_error = None
                try:
                    geometry_marker = f" P {len(parts)} " if args.navigation_body == "parts" else " R "
                    obstacle_fields = [value for box in obstacle_observations
                                       for value in (*box["position"], *box["half_size"], *box["velocity"])]
                    dynamic_message = (f" D {len(dynamic_boxes)} "
                                       + " ".join(format(float(v), ".17g") for v in obstacle_fields)) if dynamic_boxes else ""
                    process.stdin.write(" ".join(format(float(v), ".17g") for v in fields)
                                        + geometry_marker
                                        + " ".join(format(float(v), ".17g") for v in snapshot) + dynamic_message + "\n")
                    process.stdin.flush()
                except OSError as exc:
                    # Windows can report errno 22 when startup rejection closes stdin.
                    write_error = exc
                wall_timing_s["encode_and_send_state"] += time.perf_counter() - phase_started
                phase_started = time.perf_counter()
                response = pending.get(timeout=120)
                wall_timing_s["native_first_response_wait" if tick == 0 else "native_response_wait"] += time.perf_counter() - phase_started
                if isinstance(response, Exception):
                    raise response
                if first_reply_wall_s is None:
                    first_reply_wall_s = time.perf_counter() - native_started
                if first_motion_wall_s is None and any(abs(float(response.get(k, 0))) > 1e-6
                                                       for k in ("vx", "vy", "wz")):
                    first_motion_wall_s = time.perf_counter() - native_started
                if write_error is not None and not response.get("done"):
                    raise write_error
                if renderer is not None and not render_route_loaded:
                    from tools.bench.body_motion_view import read_route

                    render_route = read_route(args.out / "native/global.csv")
                    render_route_loaded = True
                if response.get("done"):
                    terminal, terminal_time = response, float(engine.sim_time)
            else:
                response = terminal
            motion = VelocityCommand(float(response.get("vx", 0)), float(response.get("vy", 0)),
                                     float(response.get("wz", 0)))
            substep_blocked: list[dict[str, Any]] = []
            # The public engine keeps policy cadence while each physics step is observed.
            for _ in range(max(1, round(args.dt / engine.dt))):
                phase_started = time.perf_counter()
                update_obstacles(float(engine.sim_time) - execution_start_time)
                state = engine.step_sensor_tick(motion, engine.dt)
                wall_timing_s["physics_policy_and_returned_state"] += time.perf_counter() - phase_started
                phase_started = time.perf_counter()
                observed_contacts = contacts(engine, robot_ids=body_ids)
                wall_timing_s["substep_contact_checks"] += time.perf_counter() - phase_started
                physics_steps += 1
                if observed_contacts["non_foot_contacts"]:
                    if not first_bad_substep_captured:
                        capture_bad_substep(engine, args.out / "first-bad-substep.json",
                                            observed_contacts["non_foot_contacts"])
                        first_bad_substep_captured = True
                    blocked_steps += 1
                    substep_blocked.extend({"t": float(engine.sim_time), **contact}
                                           for contact in observed_contacts["non_foot_contacts"])
            sample = {"t": float(engine.sim_time), "position": state.position.tolist(),
                      "velocity": state.linear_velocity.tolist(), "rpy": rpy(state.orientation),
                      "joint_positions": state.joint_positions.tolist(),
                      "command": response, **observed_contacts,
                      "dynamic_obstacle_observations": obstacle_observations,
                      "dynamic_obstacles": [{"name": box.name,
                                              "position": engine.data.mocap_pos[mocap_id].tolist(),
                                              "half_size": list(box.half_size)}
                                             for box, mocap_id in zip(dynamic_boxes, mocap_ids)],
                      "non_foot_contacts": substep_blocked,
                      "loaded_feet_at_command": current_contacts["feet"]}
            samples.append(sample)
            if renderer is not None and writer is not None and tick % max(1, round(0.1 / args.dt)) == 0:
                phase_started = time.perf_counter()
                render(engine, renderer, writer, route=render_route,
                       measured=[entry["position"] for entry in samples], requested=route,
                       elapsed_s=float(engine.sim_time) - execution_start_time)
                wall_timing_s["video_render"] += time.perf_counter() - phase_started
            if not np.isfinite(engine.data.qpos).all() or not np.isfinite(engine.data.qvel).all():
                result = "nonfinite_physics"
                break
            if max(abs(v) for v in sample["rpy"][:2]) > 0.9:
                result = "tipped"
                break
            if sample["non_foot_contacts"]:
                result = "non_foot_world_contact"
                break
            if args.pace:
                # Async SCAN runs on wall time. Do not let faster geometry
                # queries advance its feedback clock faster than real time.
                # Slow ticks are not followed by accelerated catch-up ticks.
                wait_started = time.perf_counter()
                remaining = args.dt - (wait_started - tick_started)
                if remaining > 0:
                    time.sleep(remaining)
                wall_timing_s["pacing_wait"] += time.perf_counter() - wait_started
            if response.get("done"):
                result = ("native_goal_reached" if response.get("status") == "goal_reached"
                          else str(response.get("status") or "native_done"))
                if float(engine.sim_time) - terminal_time >= 0.8:
                    break
        final = engine.get_robot_state()
        elapsed_wall = time.perf_counter() - started
        wall_timing_s["other_loop_and_setup"] = elapsed_wall - sum(wall_timing_s.values())
        report = {"result": result, "robot": "doso/thunder_v4", "robot_xml": str(robot_xml), "policy": str(policy),
                  "keep_turn_stages": args.keep_turn_stages,
                  "wall_paced": args.pace,
                  "navigation_body": args.navigation_body,
                  "effective_world": str(effective_world),
                  "physics_collision_backend": ("libccd" if int(engine.model.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD) else "nativeccd"),
                  "physics_ccd_tolerance": float(engine.model.opt.ccd_tolerance),
                  "dynamic_obstacle_count": len(dynamic_boxes),
                  "obstacle_input_scope": "deterministic fixture ground truth; not LiDAR detections",
                  "goal_yaw_tolerance_deg": args.goal_yaw_tolerance_deg,
                  "first_reply_wall_s": first_reply_wall_s,
                  "first_motion_wall_s": first_motion_wall_s,
                  "samples": len(samples), "wall_time_s": elapsed_wall,
                  "wall_timing_s": dict(wall_timing_s),
                  "python_process_cpu_s": time.process_time() - cpu_started,
                  "timing_scope": "run entry to summary capture; excludes module imports, final log writing/cleanup and later video replay; native waits include native work and IPC scheduling, not additional time to add to native CSV timers",
                  "sim_time_s": float(engine.sim_time), "final_position": final.position.tolist(),
                  "execution_start_time_s": execution_start_time,
                  "execution_duration_s": float(engine.sim_time) - execution_start_time,
                  "final_rpy": rpy(final.orientation),
                  "goal_xy_error_m": float(np.linalg.norm(np.asarray(route[-1][:2]) - final.position[:2])),
                  "goal_yaw_error_rad": abs(math.remainder(route[-1][3] - rpy(final.orientation)[2], 2 * math.pi)),
                  "actual_xy_displacement_m": float(np.linalg.norm(final.position[:2] - np.asarray(route[0][:2]))),
                  "max_command_xy_mps": max((math.hypot(s["command"].get("vx", 0), s["command"].get("vy", 0))
                                              for s in samples), default=0.0),
                  "non_foot_contact_samples": sum(bool(s["non_foot_contacts"]) for s in samples),
                  "physics_substeps_checked": physics_steps,
                  "physics_dt_s": engine.dt,
                  "non_foot_contact_substeps": blocked_steps,
                  "no_loaded_support_frames": unloaded_frames,
                  "snapshot_cost_ms": {"mean": float(np.mean(snapshot_costs_ms)),
                                       "p95": float(np.percentile(snapshot_costs_ms, 95)),
                                       "max": max(snapshot_costs_ms)},
                  "last_native": samples[-1]["command"] if samples else None,
                  "video_error": video_error,
                  "scope": "actual MuJoCo policy physics with native component executor; not Go2 or full Product"}
        last_reason = str(report["last_native"].get("reason", "")) if report["last_native"] else ""
        report["failure_stage"] = ("global_planning" if result == "global_failed" else
                                   "final_body_or_support_safety" if last_reason == "final_body_or_support_blocked" else
                                   "none" if result == "native_goal_reached" else "execution")
        return report
    finally:
        with (args.out / "states.jsonl").open("w", encoding="utf-8") as output:
            for sample in samples:
                output.write(json.dumps(sample) + "\n")
        if process is not None:
            try:
                process.stdin.close()
            except OSError as exc:
                if exc.errno not in (22, 32):
                    raise
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=10)
        if writer is not None:
            writer.release()
        if renderer is not None:
            renderer.close()
        engine.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--native", type=Path)
    parser.add_argument("--map", type=Path)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--dynamic-obstacles", type=Path,
                        help="Fixture JSON boxes with start/end XYZ, half_size, start_s and duration_s; ground truth input")
    parser.add_argument("--world", type=Path, default=WORLD)
    parser.add_argument("--ccd-tolerance", type=float,
                        help="Explicit MuJoCo collision numerical tolerance for a matched physics experiment; default keeps the model setting")
    parser.add_argument("--waypoints", type=Path,
                        help="JSON [world-dx,world-dy,yaw-offset-deg] points from the settled measured pose")
    parser.add_argument("--goal-offset", nargs=2, type=float, default=DEFAULT_GOAL_OFFSET, metavar=("DX", "DY"),
                        help="World-frame XY offset from the settled measured start, in metres")
    parser.add_argument("--goal-yaw-offset-deg", type=float, default=DEFAULT_GOAL_YAW_OFFSET_DEG,
                        help="Final heading offset from the settled measured yaw, in degrees")
    parser.add_argument("--goal-yaw-tolerance-deg", type=float, default=3.0)
    parser.add_argument("--navigation-body", choices=("parts", "obb"), default="parts",
                        help="Native geometry: articulated snapshot or production-configured single OBB with measured attitude")
    parser.add_argument("--use-route", action="store_true")
    parser.add_argument("--keep-turn-stages", action="store_true",
                        help="Benchmark ablation: retain original stop/turn stages with unchanged final checks")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--pace", action="store_true",
                        help="Keep each control step at least dt wall seconds for async-planner comparisons")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--dt", type=float, default=0.02)
    args = parser.parse_args()
    if args.ccd_tolerance is not None and (not math.isfinite(args.ccd_tolerance) or args.ccd_tolerance <= 0):
        parser.error("--ccd-tolerance must be finite and positive")
    if not all(math.isfinite(value) for value in (*args.goal_offset, args.goal_yaw_offset_deg)):
        parser.error("--goal-offset and --goal-yaw-offset-deg must be finite")
    if not math.isfinite(args.goal_yaw_tolerance_deg) or not 0 < args.goal_yaw_tolerance_deg <= 180:
        parser.error("--goal-yaw-tolerance-deg must be in (0, 180]")
    if args.native is not None and args.map is None:
        parser.error("--native requires --map")
    if args.waypoints and not args.use_route:
        parser.error("--waypoints requires --use-route so intermediate points reach the executor")
    try:
        summary = run(args)
    except Exception as exc:
        args.out.mkdir(parents=True, exist_ok=True)
        write_json(args.out / "summary.json", {"result": "harness_error", "error": f"{type(exc).__name__}: {exc}"})
        raise
    write_json(args.out / "summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
