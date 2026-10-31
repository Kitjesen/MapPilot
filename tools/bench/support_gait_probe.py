"""Offline Thunder policy capability probes, with native predicates replayed separately.

These runs do not execute a navigation Product. Commands are controlled experimental
inputs to MuJoCo; no hardware command path or navigation safety rule is changed.
"""
from __future__ import annotations

import argparse
import copy
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from tools.bench.body_physics import (
    POLICY, WORLD, contacts, export_body, part_snapshot,
    robot_body_ids, write_json, capture_bad_substep,
)
from tools.bench.body_export import native_body_text
from drivers.sim.mujoco.runtime import build_engine
from sim.compat.engine.core.engine import VelocityCommand
from sim.scripts.mujoco.stair_policy_probe import rpy


def cases() -> dict:
    result = {"flat_forward": {}, "flat_turn": {"turn": True}}
    for width in (.10, .30, .50):
        result[f"slot_stand_{round(width*100):03}"] = {"slot": width, "stand": True}
    result["slot_walk_010"] = {"slot": .10}
    for width in (.10, .30):
        result[f"gap_walk_{round(width*100):03}"] = {"gap": width}
    for width in (.60, .80):
        result[f"corridor_{round(width*100):03}"] = {"corridor": width}
    return result


def scene(config: dict, output: Path) -> Path:
    # Thick analytic boxes avoid the previous finite-heightfield fixture issue.
    floors = [[-1.5, -1.2, -.3, 3, 1.2, 0]]
    if "slot" in config:
        half = config["slot"] / 2
        floors = [[-1.5, -1.2, -.3, 3, -half, 0], [-1.5, half, -.3, 3, 1.2, 0]]
    if "gap" in config:
        half = config["gap"] / 2
        floors = [[-1.5, -1.2, -.3, .8-half, 1.2, 0], [.8+half, -1.2, -.3, 3, 1.2, 0]]
    boxes = floors[:]
    if "corridor" in config:
        half = config["corridor"] / 2
        boxes += [[.6, -1.2, 0, 2.7, -half, 1.2], [.6, half, 0, 2.7, 1.2, 1.2]]
    values = [.05, -1.5, -1.2, -.3, 3, 1.2, 1.5, len(boxes), *(v for box in boxes for v in box)]
    (output / "scene.txt").write_text(" ".join(map(str, values)), encoding="utf-8")
    tree = ET.parse(WORLD)
    world = tree.getroot().find("worldbody")
    for geom in list(world.findall("geom")):
        world.remove(geom)
    for i, box in enumerate(boxes):
        low, high = np.array(box[:3]), np.array(box[3:])
        ET.SubElement(world, "geom", name=f"fixture_{i}", type="box",
                      pos=" ".join(map(str, (low+high)/2)), size=" ".join(map(str, (high-low)/2)),
                      contype="1", conaffinity="1", condim="3", rgba="0.55 0.60 0.64 1")
    path = output / "world.xml"
    tree.write(path, encoding="utf-8")
    return path


def primitive_bounds(parts: list[dict]) -> tuple[np.ndarray, np.ndarray]:
    lows, highs = [], []
    for part in parts:
        center = np.array(part["center"])
        rotation = np.array(part["rotation"])
        kind = part["source_type"]
        if kind == "box":
            extent = np.abs(rotation) @ np.array(part["half_size"])
        elif kind == "sphere":
            extent = np.full(3, part["radius_m"])
        else:
            axis = rotation[:, 2]
            radial = np.sqrt(np.maximum(0, 1-axis*axis)) if kind == "cylinder" else np.ones(3)
            extent = part["radius_m"]*radial + part["half_length_m"]*np.abs(axis)
        lows.append(center-extent)
        highs.append(center+extent)
    return np.min(lows, axis=0), np.max(highs, axis=0)


def run_case(name: str, config: dict, root: Path, native: Path, video: bool) -> dict:
    output = root / name
    output.mkdir(parents=True, exist_ok=False)
    (output / "snapshots").mkdir()
    world = scene(config, output)
    engine = build_engine(world=world, drive_mode="policy", start=[0, 0, .58],
                          mujoco_memory="64M", policy_path=POLICY, policy_cpu_threads=1,
                          lidar_backend="mujoco_lidar", lidar_body_name="lidar1_link",
                          lidar_site_name="lidar1_link_site", mid360_samples_per_frame=32)
    renderer = writer = None
    samples, probes = [], []
    bad_substeps = total_substeps = 0
    result = "duration_elapsed"
    try:
        zero = VelocityCommand(0, 0, 0)
        ids = robot_body_ids(engine.model)
        # All scenarios start from the same state and policy; no settling reset.
        with (output / "settle-states.jsonl").open("w", encoding="utf-8") as settling:
            for tick in range(round(2/engine.dt)):
                engine.step_sensor_tick(zero, engine.dt)
                if tick % max(1, round(.02/engine.dt)) == 0:
                    state = engine.get_robot_state()
                    observed = contacts(engine, robot_ids=ids)
                    settling.write(json.dumps({"t": float(engine.sim_time)-2,
                        "position": state.position.tolist(), "loaded_feet": observed["feet"],
                        "qpos": engine.data.qpos.tolist(), "qvel": engine.data.qvel.tolist()})+"\n")
        nominal = export_body(engine, output / "nominal.body2")
        nominal["world"] = str(world)
        write_json(output / "nominal.json", nominal)
        parts = nominal["parts"]
        nominal_low, nominal_high = primitive_bounds(parts)
        write_json(output / "case.json", {"name": name, "config": config, "robot": "doso/thunder_v4",
                   "policy": str(POLICY), "world": str(world), "mujoco_version": mujoco.__version__,
                   "physics_dt_s": engine.dt, "settle_s": 2, "duration_limit_s": 10,
                   "forward_command_mps": .25, "turn_command_radps": .30,
                   "native_resolution_m": .05, "native_support_height": "settled base Z",
                   "native_support_tolerance_m": .05,
                   "scope": "policy capability, not native navigation execution or Go2 evidence"})
        if video:
            import cv2
            renderer = mujoco.Renderer(engine.model, height=540, width=960)
            writer = cv2.VideoWriter(str(output / "physics.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (960, 540))
            if not writer.isOpened():
                raise RuntimeError("video writer failed")
        start = engine.get_robot_state().position.copy()
        start_time = float(engine.sim_time)
        reached_at = None
        with (output / "states.jsonl").open("w", encoding="utf-8") as states:
            for tick in range(501):
                state = engine.get_robot_state()
                t = float(engine.sim_time)-start_time
                angles = rpy(state.orientation)
                observed = contacts(engine, robot_ids=ids)
                snapshot = part_snapshot(engine, parts, state)
                current = copy.deepcopy(parts)
                for i, part in enumerate(current):
                    row = snapshot[i*13:(i+1)*13]
                    part["center"] = row[:3]
                    part["rotation"] = np.array(row[3:12]).reshape(3, 3).tolist()
                low, high = primitive_bounds(current)
                if not config.get("stand") and not config.get("turn") and state.position[0]-start[0] >= 1.6 and reached_at is None:
                    reached_at = t
                cmd = zero if config.get("stand") or reached_at is not None or (config.get("turn") and t>=6) else (
                    VelocityCommand(0, 0, .30) if config.get("turn") else VelocityCommand(.25, 0, 0))
                sample = {"t": t, "position": state.position.tolist(), "rpy": angles,
                          "velocity": state.linear_velocity.tolist(), "joints": state.joint_positions.tolist(),
                          "qpos": engine.data.qpos.tolist(), "qvel": engine.data.qvel.tolist(),
                          "loaded_feet": observed["feet"], "contacts": observed["non_foot_contacts"],
                          "max_normal_force_n": observed["max_normal_force_n"],
                          "command": [cmd.linear_x, cmd.linear_y, cmd.angular_z],
                          "body_low": low.tolist(), "body_high": high.tolist()}
                samples.append(sample)
                states.write(json.dumps(sample)+"\n")
                if tick % 10 == 0 or bad_substeps:
                    file = f"snapshots/{tick:04}.body2"
                    (output/file).write_text(native_body_text(current), encoding="utf-8")
                    probes.append(" ".join(map(str, [t, *state.position, angles[2], file])))
                if renderer is not None and tick % 5 == 0:
                    import cv2
                    camera = mujoco.MjvCamera()
                    camera.lookat[:] = [max(.3, float(state.position[0])), 0, .2]
                    camera.distance, camera.azimuth, camera.elevation = 2.8, 125, -28
                    options = mujoco.MjvOption()
                    options.geomgroup[5] = 1
                    renderer.update_scene(engine.data, camera=camera, scene_option=options)
                    frame = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
                    cv2.putText(frame, f"{name} | policy physics | t={t:.2f}s", (15, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, .6, (30, 30, 30), 1)
                    writer.write(frame)
                if not np.isfinite(engine.data.qpos).all() or max(abs(a) for a in angles[:2]) > .9:
                    result = "tipped_or_nonfinite"
                    break
                # Every fixture's lowest solid surface is z=-0.3 m. A base below
                # it has fallen out of the scene, even if roll/pitch stays small.
                if state.position[2] < -.3:
                    result = "fell_during_settle" if tick == 0 else "fell_below_fixture"
                    break
                if observed["non_foot_contacts"] or bad_substeps:
                    result = "contact_violation"
                    break
                if (reached_at is not None and t-reached_at>=1) or (config.get("stand") and t>=8) or (config.get("turn") and t>=8):
                    result = "distance_completed" if reached_at is not None else "hold_completed" if config.get("stand") else "turn_completed"
                    break
                for _ in range(max(1, round(.02/engine.dt))):
                    engine.step_sensor_tick(cmd, engine.dt)
                    contact = contacts(engine, robot_ids=ids)
                    total_substeps += 1
                    if contact["non_foot_contacts"]:
                        if bad_substeps == 0:
                            capture_bad_substep(engine, output / "first-contact.json", contact["non_foot_contacts"])
                        bad_substeps += 1
                if bad_substeps:
                    result = "contact_violation"
                    # Capture the resulting pose on the next tick before stopping.
                    continue
        (output / "probe.txt").write_text("\n".join(probes)+"\n", encoding="utf-8")
        if bad_substeps:
            result = "contact_violation"
        report = {"name": name, "result": result, "duration_s": samples[-1]["t"],
                  "net_x_m": samples[-1]["position"][0]-float(start[0]),
                  "max_abs_roll_pitch_rad": max(max(abs(v) for v in s["rpy"][:2]) for s in samples),
                  "nominal_width_m": float(nominal_high[1]-nominal_low[1]),
                  "max_width_m": max(s["body_high"][1]-s["body_low"][1] for s in samples),
                  "lateral_envelope_exceeded_frames": int(sum(s["body_low"][1]<nominal_low[1]-.001 or s["body_high"][1]>nominal_high[1]+.001 for s in samples)),
                  "four_loaded_feet_frames": sum(len(s["loaded_feet"])==4 for s in samples),
                  "samples": len(samples), "illegal_contact_substeps": bad_substeps,
                  "physics_substeps": total_substeps, "video": video,
                  "scope": "Thunder policy/MuJoCo capability only; native predicates replayed separately"}
        write_json(output / "summary.json", report)
    finally:
        if writer is not None:
            writer.release()
        if renderer is not None:
            renderer.close()
        engine.close()
    with (output / "native.log").open("w", encoding="utf-8") as log:
        subprocess.run([str(native.resolve()), str(output.resolve())], stdout=log, stderr=subprocess.STDOUT, check=True)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--case", choices=list(cases()))
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    for name, config in cases().items():
        if args.case and name != args.case:
            continue
        print(json.dumps(run_case(name, config, args.out, args.native, args.video)), flush=True)
