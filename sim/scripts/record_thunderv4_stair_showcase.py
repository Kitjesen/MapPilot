#!/usr/bin/env python3
"""Record the ThunderV4 policy on the stair showcase scene.

This script is simulation-only. It validates the locomotion policy directly:

    cmd_vel -> MuJoCo policy -> ThunderV4 motion

It does not start navigation, DDS, Gateway, or real robot services.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drivers.sim.mujoco.driver import MujocoDriverModule
from sim.engine.core.engine import VelocityCommand
from sim.scripts.policy_nav_smoke import (
    _contact_snapshot,
    _contact_summary,
    _rpy_from_xyzw,
)


def _jsonable_position(state: Any) -> list[float]:
    return [float(v) for v in state.position]


def _render_camera(camera: Any, mode: str, pos: list[float]) -> None:
    if mode == "side":
        camera.lookat[:] = [pos[0] + 0.35, pos[1], max(0.58, pos[2] + 0.02)]
        camera.distance = 4.15
        camera.azimuth = 90.0
        camera.elevation = -7.5
        return

    camera.lookat[:] = [pos[0] + 0.55, pos[1], max(0.62, pos[2] + 0.05)]
    camera.distance = 3.35
    camera.azimuth = 112.0
    camera.elevation = -16.0


def run(args: argparse.Namespace) -> dict[str, Any]:
    import cv2
    import mujoco

    video_path = Path(args.video).resolve()
    json_path = Path(args.json_out).resolve()
    frame_path = Path(args.frame_out).resolve()
    video_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    frame_path.parent.mkdir(parents=True, exist_ok=True)

    driver = MujocoDriverModule(
        world=args.world,
        drive_mode="policy",
        render=False,
        enable_camera=False,
        sim_rate=100.0,
        publish_rate=20.0,
    )
    driver.setup()
    engine = driver._engine
    if engine is None:
        raise RuntimeError("MuJoCo engine missing after setup")

    model = engine.model
    data = engine.data
    control_dt = float(getattr(engine, "_control_dt", 0.02))

    settle_steps = int(args.settle_s / control_dt)
    for _ in range(settle_steps):
        engine.step(VelocityCommand(0.0, 0.0, 0.0))

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE

    writer = cv2.VideoWriter(
        str(video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(args.fps),
        (args.width, args.height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"VideoWriter failed: {video_path}")

    render_every = max(1, round((1.0 / args.fps) / control_dt))
    command = VelocityCommand(args.linear_x, 0.0, 0.0)
    total_steps = int(args.duration_s / control_dt)
    start = engine.get_robot_state()
    start_position = _jsonable_position(start)

    samples: list[dict[str, float]] = []
    contacts: list[dict[str, Any]] = []
    frame_count = 0
    mid_frame_written = False
    finite = True
    max_roll = 0.0
    max_pitch = 0.0

    try:
        for step_idx in range(total_steps):
            engine.step(command)
            state = engine.get_robot_state()
            pos = _jsonable_position(state)
            roll, pitch, yaw = _rpy_from_xyzw(state.orientation)

            if step_idx % 3 == 0:
                samples.append(
                    {
                        "t": round((step_idx + 1) * control_dt, 3),
                        "x": pos[0],
                        "y": pos[1],
                        "z": pos[2],
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw,
                    }
                )
                contacts.append(_contact_snapshot(engine))
                max_roll = max(max_roll, abs(roll))
                max_pitch = max(max_pitch, abs(pitch))
                finite = finite and all(math.isfinite(v) for v in pos + [roll, pitch, yaw])

            if step_idx % render_every == 0:
                _render_camera(camera, args.camera, pos)
                renderer.update_scene(data, camera=camera)
                frame = renderer.render()
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                writer.write(bgr)
                frame_count += 1
                if not mid_frame_written and step_idx >= total_steps // 2:
                    cv2.imwrite(str(frame_path), bgr)
                    mid_frame_written = True
    finally:
        writer.release()
        renderer.close()

    end = engine.get_robot_state()
    end_position = _jsonable_position(end)
    xs = [s["x"] for s in samples]
    ys = [s["y"] for s in samples]
    zs = [s["z"] for s in samples]
    body_hits = sum(int(c.get("non_foot_ground_contacts", 0) or 0) for c in contacts)
    reached_mid_landing = max(xs) >= args.mid_x and max(zs) >= args.mid_z
    reached_upper_landing = max(xs) >= args.upper_x and max(zs) >= args.upper_z

    result = {
        "scenario": "thunderv4_stair_showcase",
        "world": args.world,
        "camera": args.camera,
        "video": str(video_path),
        "frame_mid": str(frame_path),
        "duration_s": float(args.duration_s),
        "video_fps": float(args.fps),
        "frame_count": frame_count,
        "command": {
            "linear_x": float(args.linear_x),
            "linear_y": 0.0,
            "angular_z": 0.0,
        },
        "start_position": start_position,
        "end_position": end_position,
        "x_progress_m": end_position[0] - start_position[0],
        "z_gain_m": end_position[2] - start_position[2],
        "x_max_m": max(xs),
        "z_max_m": max(zs),
        "y_abs_max_m": max(abs(v) for v in ys),
        "roll_abs_max_rad": max_roll,
        "pitch_abs_max_rad": max_pitch,
        "reached_mid_landing": reached_mid_landing,
        "reached_upper_landing": reached_upper_landing,
        "body_ground_contact_events": body_hits,
        "finite": finite,
        "pass": bool(
            finite
            and reached_upper_landing
            and max_roll < args.max_roll
            and max_pitch < args.max_pitch
            and body_hits == 0
        ),
        "contacts": _contact_summary(contacts),
        "sample_count": len(samples),
        "samples_tail": samples[-10:],
    }
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    engine.close()
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default="stair_showcase")
    parser.add_argument("--camera", choices=("side", "follow"), default="side")
    parser.add_argument("--linear-x", type=float, default=1.0)
    parser.add_argument("--duration-s", type=float, default=18.0)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--fps", type=float, default=25.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument(
        "--video",
        default=str(ROOT / "artifacts" / "mujoco_thunderv4_stair_showcase_vx10_side.mp4"),
    )
    parser.add_argument(
        "--json-out",
        default=str(ROOT / "artifacts" / "mujoco_thunderv4_stair_showcase_vx10_side.json"),
    )
    parser.add_argument(
        "--frame-out",
        default=str(ROOT / "artifacts" / "mujoco_thunderv4_stair_showcase_vx10_side_frame_mid.jpg"),
    )
    parser.add_argument("--mid-x", type=float, default=2.35)
    parser.add_argument("--mid-z", type=float, default=0.72)
    parser.add_argument("--upper-x", type=float, default=5.25)
    parser.add_argument("--upper-z", type=float, default=1.00)
    parser.add_argument("--max-roll", type=float, default=0.45)
    parser.add_argument("--max-pitch", type=float, default=0.45)
    return parser.parse_args()


if __name__ == "__main__":
    print(json.dumps(run(parse_args()), ensure_ascii=False, indent=2))
