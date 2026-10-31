"""Measure packaged Thunder policies on real MuJoCo stair contacts.

This bounded locomotion probe is not a native-navigation test.
The canonical engine owns observations, gains, inference and physical stepping.
The probe initializes a spawn once and never moves or resets the base afterward.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import onnxruntime

import mujoco

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from sim.catalog import resolve_robot_default_policy
from sim.compat.engine.core.engine import VelocityCommand

from drivers.sim.mujoco.runtime import build_engine

WORLD = ROOT / "sim/compat/engine/worlds/thunderv4_stair_showcase.xml"
POLICIES = ROOT / "sim/packages/controllers/doso/thunder_v4/locomotion/policy"
POLICY_PATHS = {
    "4998": POLICIES / "policy_4998.onnx",
    "1119": POLICIES / "policy_1119.onnx",
    "cts_moe": resolve_robot_default_policy(ROOT)["artifact"],
}
FEET = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"}


def resolve_policy(value: str) -> Path:
    """Select a packaged policy alias or an explicitly supplied ONNX path."""
    return POLICY_PATHS[value] if value in POLICY_PATHS else Path(value).expanduser().resolve()


def rpy(q: Any) -> list[float]:
    x, y, z, w = (float(v) for v in q)
    return [
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    ]


def contacts(engine: Any) -> dict[str, Any]:
    model, data = engine.model, engine.data
    feet: set[str] = set()
    support_geoms: set[str] = set()
    body_contacts: list[dict[str, Any]] = []
    max_force = 0.0
    for index in range(int(data.ncon)):
        c = data.contact[index]
        body_ids = [int(model.geom_bodyid[c.geom1]), int(model.geom_bodyid[c.geom2])]
        if 0 not in body_ids:
            continue
        names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "" for i in body_ids}
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, force)
        max_force = max(max_force, abs(float(force[0])))
        touching_feet = names & FEET
        if touching_feet:
            feet.update(touching_feet)
            for geom in (c.geom1, c.geom2):
                if int(model.geom_bodyid[geom]) == 0:
                    support_geoms.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom)) or "")
        else:
            body_contacts.append({
                "bodies": sorted(names),
                "geoms": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(i)) for i in (c.geom1, c.geom2)],
                "normal_force_n": float(force[0]),
                "distance_m": float(c.dist),
            })
    return {"feet": sorted(feet), "support_geoms": sorted(support_geoms),
            "non_foot_contacts": body_contacts, "max_normal_force_n": max_force}


def render(engine: Any, renderer: Any, destination: Path) -> None:
    import cv2

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [2.5, 0.0, 0.55]
    camera.distance, camera.azimuth, camera.elevation = 10.0, 90.0, -21.0
    options = mujoco.MjvOption()
    options.geomgroup[5] = 1
    renderer.update_scene(engine.data, camera=camera, scene_option=options)
    pixels = renderer.render()
    if not cv2.imwrite(str(destination), cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)):
        raise RuntimeError(f"Could not write {destination}")


def heading_command(state: Any, ascent: bool, max_speed: float) -> VelocityCommand:
    """Use simulation truth only to keep this probe on the stair centerline."""
    x, y, _ = state.position
    desired_yaw = math.atan2(-float(y), 0.75 if ascent else -0.75)
    yaw_error = math.remainder(desired_yaw - rpy(state.orientation)[2], 2.0 * math.pi)
    remaining = (5.85 - float(x)) if ascent else (float(x) + 0.85)
    speed = min(max_speed, max(0.06, 0.8 * remaining))
    speed *= max(0.2, math.cos(yaw_error))
    return VelocityCommand(speed, 0.0, max(-0.6, min(0.6, 2.0 * yaw_error)))


def probe(policy: str, direction: str, args: argparse.Namespace) -> dict[str, Any]:
    selected_policy = resolve_policy(policy)
    label = policy if policy in POLICY_PATHS else selected_policy.stem
    out = args.output / f"{label}_{direction}"
    out.mkdir(parents=True, exist_ok=True)
    ascent = direction == "up"
    spawn = [-1.0, 0.0, 0.58] if ascent else [5.9, 0.0, 1.27]
    orientation = [1.0, 0.0, 0.0, 0.0] if ascent else [0.0, 0.0, 0.0, 1.0]
    started = time.perf_counter()
    engine = build_engine(
        world=WORLD, drive_mode="policy", start=spawn,
        start_orientation_wxyz=orientation, mujoco_memory="64M",
        policy_path=POLICY_PATHS["4998"] if args.handoff else selected_policy, policy_cpu_threads=1,
        lidar_backend="mujoco_lidar", lidar_body_name="lidar1_link",
        lidar_site_name="lidar1_link_site",
        mid360_samples_per_frame=32,
    )
    renderer = None
    render_error = ""
    samples: list[dict[str, Any]] = []
    reached_at: float | None = None
    entry_switch_at: float | None = None
    restore_at: float | None = None
    quiet_since: float | None = None
    entry_ready = not args.handoff
    switch_events: list[dict[str, Any]] = []
    restore_requested = args.handoff or args.exit_restore
    result = "duration_elapsed"
    try:
        try:
            renderer = mujoco.Renderer(engine.model, height=540, width=960)
            render(engine, renderer, out / "initial.png")
        except Exception as exc:
            render_error = f"{type(exc).__name__}: {exc}"
            if renderer is not None:
                renderer.close()
                renderer = None
        control_dt = float(engine._control_dt)
        start_state = engine.get_robot_state()
        for index in range(math.ceil((args.duration + args.settle) / control_dt)):
            elapsed = index * control_dt
            prior = engine.get_robot_state()
            quiet = np.linalg.norm(prior.linear_velocity) <= 0.04 and np.linalg.norm(prior.angular_velocity) <= 0.12
            quiet_since = (float(engine.sim_time) if quiet_since is None else quiet_since) if quiet else None
            confirmed_quiet = quiet_since is not None and float(engine.sim_time) - quiet_since >= 0.2
            if args.handoff and elapsed >= args.settle and entry_switch_at is None and confirmed_quiet:
                before = engine.data.qpos.copy()
                old_path = engine.active_policy_path
                changed = engine.switch_policy(str(selected_policy))
                entry_switch_at = float(engine.sim_time)
                quiet_since = None
                switch_events.append({"phase": "entry", "t": entry_switch_at, "before": old_path,
                                      "after": engine.active_policy_path, "changed": changed,
                                      "qpos_unchanged": bool(np.array_equal(before, engine.data.qpos))})
            if args.handoff and entry_switch_at is not None and not entry_ready:
                entry_ready = float(engine.sim_time) - entry_switch_at >= 2.0 and confirmed_quiet
            moving = elapsed >= args.settle and entry_ready and reached_at is None
            command = VelocityCommand(args.speed if moving else 0.0, 0.0, 0.0)
            if moving and args.heading_hold:
                command = heading_command(engine.get_robot_state(), ascent, args.speed)
            state = engine.step(command)
            angle = rpy(state.orientation)
            contact = contacts(engine)
            sample = {
                "t": float(engine.sim_time), "phase": "drive" if moving else "settle",
                "command_vx": command.linear_x, "command_wz": command.angular_z,
                "active_policy": engine.active_policy_path,
                "position": np.asarray(state.position).tolist(),
                "velocity": np.asarray(state.linear_velocity).tolist(),
                "rpy": angle, **contact,
            }
            samples.append(sample)
            if not np.isfinite(engine.data.qpos).all() or not np.isfinite(engine.data.qvel).all():
                result = "nonfinite_physics"
                break
            if max(abs(angle[0]), abs(angle[1])) > 0.9:
                result = "tipped"
                break
            if contact["non_foot_contacts"]:
                result = "non_foot_world_contact"
                break
            x, y, z = state.position
            if -0.4 <= x <= 5.6 and abs(y) > 2.0:
                result = "left_stair_width"
                break
            on_exit = x >= 5.7 if ascent else x <= -0.7
            platform_z = 0.72 if ascent else 0.03
            if on_exit and 0.25 < z - platform_z < 0.65 and moving:
                reached_at = float(engine.sim_time)
            if reached_at is not None and float(engine.sim_time) - reached_at >= 2.0:
                within_platform = (5.4 <= x <= 6.3 if ascent else -1.15 <= x <= -0.55) and abs(y) < 1.8
                if restore_requested and restore_at is None:
                    if not within_platform:
                        result = "exit_not_settled_on_platform"
                        break
                    if confirmed_quiet:
                        before = engine.data.qpos.copy()
                        old_path = engine.active_policy_path
                        changed = engine.switch_policy(str(POLICY_PATHS["4998"]))
                        restore_at = float(engine.sim_time)
                        quiet_since = None
                        switch_events.append({"phase": "exit", "t": restore_at, "before": old_path,
                                              "after": engine.active_policy_path, "changed": changed,
                                              "qpos_unchanged": bool(np.array_equal(before, engine.data.qpos))})
                elif not restore_requested or (restore_at is not None and float(engine.sim_time) - restore_at >= 2.0 and confirmed_quiet):
                    result = ("exit_reached_and_settled"
                              if within_platform and np.linalg.norm(state.linear_velocity[:2]) < 0.08
                              else "exit_not_settled_on_platform")
                    break
        if renderer is not None:
            render(engine, renderer, out / "final.png")
        final = engine.get_robot_state()
        report = {
            "schema": "lingtu.stair-policy-probe.v1",
            "scope": "truth_heading_feedback_locomotion_only" if args.heading_hold else "fixed_velocity_physical_locomotion_only",
            "navigation_test": False, "profile_switch_test": restore_requested,
            "entry_switch_requested": args.handoff, "exit_restore_requested": restore_requested,
            "switch_events": switch_events,
            "result": result, "passed": result == "exit_reached_and_settled",
            "policy": str(selected_policy),
            "loaded_policy": str(engine._policy_path),
            "policy_runner": type(engine._policy).__name__,
            "policy_observation_dim": getattr(engine._robot_cfg, "obs_dim", None),
            "policy_history_len": getattr(engine._robot_cfg, "history_len", None),
            "kp": np.asarray(engine._robot_cfg.torque_kp).tolist(),
            "kd": np.asarray(engine._robot_cfg.torque_kd).tolist(),
            "physics_timestep_s": float(engine.model.opt.timestep),
            "control_timestep_s": control_dt,
            "drive_mode": str(engine._drive_mode), "direction": direction,
            "spawn": spawn, "speed_mps": args.speed, "world": str(WORLD),
            "start": np.asarray(start_state.position).tolist(),
            "end": np.asarray(final.position).tolist(),
            "displacement": (np.asarray(final.position) - start_state.position).tolist(),
            "sim_duration_s": float(engine.sim_time), "wall_duration_s": time.perf_counter() - started,
            "max_abs_roll_rad": max(abs(s["rpy"][0]) for s in samples),
            "max_abs_pitch_rad": max(abs(s["rpy"][1]) for s in samples),
            "non_foot_contact_samples": sum(bool(s["non_foot_contacts"]) for s in samples),
            "foot_contact_samples": sum(bool(s["feet"]) for s in samples),
            "observed_foot_support_geoms": sorted({g for s in samples for g in s["support_geoms"]}),
            "x_range_m": [min(s["position"][0] for s in samples), max(s["position"][0] for s in samples)],
            "z_range_m": [min(s["position"][2] for s in samples), max(s["position"][2] for s in samples)],
            "render_error": render_error, "hardware_commands": False,
            "base_pose_edits_after_initialization": 0,
            "timeline": str(out / "timeline.jsonl"),
        }
        (out / "timeline.jsonl").write_text("".join(json.dumps(s) + "\n" for s in samples), encoding="utf-8")
        (out / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        return report
    finally:
        if renderer is not None:
            renderer.close()
        engine.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "build/stairs-mujoco-20261012/baseline")
    parser.add_argument("--duration", type=float, default=50.0)
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--speed", type=float, default=0.35)
    parser.add_argument("--heading-hold", action="store_true", help="Keep the centerline with explicit MuJoCo truth velocity-command feedback; no navd.")
    parser.add_argument("--handoff", action="store_true", help="Actually load 4998, switch to the selected policy at rest, restore 4998 on the exit platform.")
    parser.add_argument("--exit-restore", action="store_true", help="Isolate exit restoration after direct initialization with the selected policy.")
    parser.add_argument("--policies", nargs="+", default=["cts_moe"],
                        help="Packaged aliases 4998, 1119, cts_moe, or explicit ONNX paths.")
    parser.add_argument("--directions", nargs="+", choices=("up", "down"), default=["up", "down"])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    source_paths = [
        "sim/scripts/mujoco/stair_policy_probe.py",
        "sim/compat/engine/mujoco/engine.py", "sim/runtime/control/policy_loader.py",
        "sim/runtime/control/thunder/cts_policy.py",
        "sim/runtime/control/thunder/legacy_policy.py", "sim/runtime/control/thunder/joints.py",
        "sim/compat/engine/core/robot.py", "src/drivers/sim/mujoco/runtime.py",
        "sim/runtime/control/thunder/flat.py", "sim/runtime/control/thunder/legacy_controller.py",
        "sim/compat/engine/worlds/thunderv4_stair_showcase.xml",
    ]
    for source in source_paths:
        target = args.output / "source" / source
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / source, target)
    report: dict[str, Any] = {
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "mujoco_version": mujoco.__version__, "onnxruntime_version": onnxruntime.__version__,
        "cases": [],
    }
    for policy in args.policies:
        for direction in args.directions:
            print(f"Running {policy} {direction}", flush=True)
            try:
                case = probe(policy, direction, args)
            except Exception as exc:
                case = {"policy": policy, "direction": direction, "passed": False, "error": f"{type(exc).__name__}: {exc}"}
                import traceback
                traceback.print_exc()
            report["cases"].append(case)
            (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            print(json.dumps(case), flush=True)
    return 0 if all(case["passed"] for case in report["cases"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
