#!/usr/bin/env python3
"""Replay a formal teleop_avoid acceptance run in the MuJoCo viewer."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np

from drivers.sim.mujoco.runtime import (
    _append_path_segments,
    build_engine,
    draw_navigation_paths,
    focus_presentation_viewer,
    launch_presentation_viewer,
)
from sim.engine.mujoco.lidar import (
    ROBOT_COLLISION_GEOM_GROUP,
    ROBOT_VISUAL_GEOM_GROUP,
)


ROOT = Path(__file__).resolve().parents[3]
PRESENTATION_HIDDEN_GEOM_GROUP = 2


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _resolve_asset(plan_path: Path, value: Any) -> Path:
    candidate = Path(str(value or "")).expanduser()
    if not str(candidate):
        raise ValueError("RunPlan asset path is empty")
    if candidate.is_absolute():
        return candidate.resolve()
    repository_candidate = (ROOT / candidate).resolve()
    if repository_candidate.exists():
        return repository_candidate
    return (plan_path.parent / candidate).resolve()


def _trajectory(report: dict[str, Any]) -> list[list[float]]:
    values = ((report.get("evidence") or {}).get("motion") or {}).get("trajectory") or []
    result: list[list[float]] = []
    for item in values:
        if not isinstance(item, list) or len(item) < 5:
            continue
        row = [float(value) for value in item[:5]]
        if all(math.isfinite(value) for value in row):
            result.append(row)
    if len(result) < 2:
        raise ValueError("acceptance report has no replayable motion trajectory")
    return result


def _representative_local_path(report: dict[str, Any]) -> list[list[float]]:
    best: list[list[float]] = []
    best_score = (-1.0, -1)
    for item in ((report.get("scenario") or {}).get("timeline") or []):
        nav = item.get("nav") if isinstance(item, dict) else None
        path = nav.get("local_path") if isinstance(nav, dict) else None
        if not isinstance(path, list):
            continue
        points: list[list[float]] = []
        for point in path:
            if not isinstance(point, list) or len(point) < 2:
                continue
            xyz = [float(point[0]), float(point[1]), float(point[2] if len(point) > 2 else 0.0)]
            if all(math.isfinite(value) for value in xyz):
                points.append(xyz)
        lateral = max((abs(point[1]) for point in points), default=0.0)
        score = (lateral, len(points))
        if score > best_score:
            best = points
            best_score = score
    return best


def _run_plan_path(report: dict[str, Any], override: Path | None) -> Path:
    if override is not None:
        return override.expanduser().resolve()
    reported = Path(str((report.get("scenario") or {}).get("run_plan") or ""))
    if reported.is_file():
        return reported.resolve()
    raise ValueError("the original RunPlan was retired; pass --run-plan with the preserved plan")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--run-plan", type=Path)
    parser.add_argument("--speed", type=float, default=1.5)
    parser.add_argument("--loop", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--snapshot-only", action="store_true")
    return parser


def _render_snapshot(
    *,
    engine: Any,
    samples: list[list[float]],
    local_path: list[list[float]],
    output: Path,
) -> None:
    import mujoco
    from PIL import Image, ImageDraw

    sample = min(samples, key=lambda item: abs(item[1] - 2.0))
    _, x, y, z, yaw = sample
    engine.set_robot_pose(
        np.asarray([x, y, z], dtype=np.float64),
        np.asarray(
            [0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)],
            dtype=np.float64,
        ),
    )
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    start = np.asarray([samples[0][1], samples[0][2], samples[0][3]], dtype=np.float64)
    goal = np.asarray([samples[-1][1], samples[-1][2], samples[-1][3]], dtype=np.float64)
    center = 0.5 * (start + goal)
    camera.lookat[:] = [center[0], center[1], max(0.30, center[2] * 0.55)]
    camera.distance = max(7.5, float(np.linalg.norm(goal[:2] - start[:2])) * 1.35)
    camera.azimuth = 90.0
    camera.elevation = -89.0
    option = mujoco.MjvOption()
    option.geomgroup[:] = 1
    option.geomgroup[ROBOT_COLLISION_GEOM_GROUP] = 0
    option.geomgroup[ROBOT_VISUAL_GEOM_GROUP] = 1
    option.geomgroup[PRESENTATION_HIDDEN_GEOM_GROUP] = 0
    renderer = mujoco.Renderer(engine.model, height=480, width=640)
    try:
        renderer.update_scene(engine.data, camera=camera, scene_option=option)
        executed_path = [np.asarray([row[1], row[2], row[3]], dtype=np.float64) for row in samples]
        planned_path = [np.asarray(point, dtype=np.float64) for point in local_path]
        _append_path_segments(
            mujoco,
            renderer.scene,
            executed_path,
            radius=0.025,
            rgba=(0.90, 0.56, 0.12, 0.95),
            z_offset=0.06,
            max_segments=160,
        )
        _append_path_segments(
            mujoco,
            renderer.scene,
            planned_path,
            radius=0.035,
            rgba=(0.18, 0.82, 0.38, 0.98),
            z_offset=0.10,
            max_segments=80,
        )
        image = Image.fromarray(renderer.render())
    finally:
        renderer.close()
    draw = ImageDraw.Draw(image)
    draw.rounded_rectangle((22, 20, 390, 92), radius=12, fill=(8, 12, 18, 210))
    draw.line((42, 45, 105, 45), fill=(230, 143, 31), width=7)
    draw.text((120, 35), "Executed trajectory", fill=(245, 245, 245))
    draw.line((42, 70, 105, 70), fill=(46, 209, 97), width=7)
    draw.text((120, 60), "CMU local plan", fill=(245, 245, 245))
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def _hide_presentation_occluders(model: Any) -> None:
    import mujoco

    geom_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "overhead_reference",
    )
    if geom_id >= 0:
        model.geom_group[geom_id] = PRESENTATION_HIDDEN_GEOM_GROUP


def _make_robot_readable(model: Any) -> None:
    import mujoco

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_id < 0:
        return
    for geom_id in range(int(model.ngeom)):
        cursor = int(model.geom_bodyid[geom_id])
        while cursor > 0 and cursor != base_id:
            cursor = int(model.body_parentid[cursor])
        if cursor != base_id:
            continue
        name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or "").lower()
        if name.endswith("_foot_visual"):
            model.geom_rgba[geom_id][3] = 0.0
        elif "wheel" in name:
            model.geom_rgba[geom_id] = np.asarray([0.03, 0.03, 0.04, 1.0])
        else:
            model.geom_rgba[geom_id] = np.asarray([0.10, 0.18, 0.24, 1.0])


def main() -> int:
    args = _parser().parse_args()
    report_path = args.report.expanduser().resolve()
    report = _load_object(report_path)
    plan_path = _run_plan_path(report, args.run_plan)
    plan = _load_object(plan_path)
    simulation = ((plan.get("launch") or {}).get("simulation") or {})
    physics = simulation.get("physics_plan") or {}
    robots = physics.get("robots") or []
    if len(robots) != 1:
        raise ValueError("replay requires exactly one robot in the RunPlan")
    robot = robots[0]
    control = simulation.get("control_plan") or {}
    controllers = control.get("controllers") or []
    if len(controllers) != 1:
        raise ValueError("replay requires exactly one controller in the RunPlan")

    world = _resolve_asset(plan_path, (physics.get("world") or {}).get("mjcf"))
    robot_xml = _resolve_asset(plan_path, (robot.get("model") or {}).get("mjcf"))
    policy = _resolve_asset(plan_path, (controllers[0].get("policy") or {}).get("artifact"))
    timestep_s = float((physics.get("global_policy") or {}).get("timestep_s") or 0.005)
    samples = _trajectory(report)
    local_path = _representative_local_path(report)
    executed_path = [[row[1], row[2], row[3]] for row in samples]
    replay_speed = max(0.1, float(args.speed))

    print(f"loading MuJoCo replay world: {world}", flush=True)
    engine = build_engine(
        world=world,
        robot_xml=robot_xml,
        drive_mode="policy",
        policy_path=policy,
        start=executed_path[0],
        mujoco_memory="",
        physics_timestep_s=timestep_s,
        require_product_lidar_backend=False,
    )
    print("MuJoCo replay model loaded", flush=True)
    _hide_presentation_occluders(engine.model)
    _make_robot_readable(engine.model)
    if args.snapshot is not None:
        _render_snapshot(
            engine=engine,
            samples=samples,
            local_path=local_path,
            output=args.snapshot.expanduser().resolve(),
        )
        print(f"MuJoCo replay snapshot written: {args.snapshot}", flush=True)
        if bool(args.snapshot_only):
            engine.close()
            return 0
    viewer = launch_presentation_viewer(engine.model, engine.data)
    print("MuJoCo replay viewer opened", flush=True)
    focus_presentation_viewer(viewer, executed_path[0], initialize=True)
    viewer.opt.geomgroup[PRESENTATION_HIDDEN_GEOM_GROUP] = 0
    draw_navigation_paths(
        viewer,
        {
            "global_path": executed_path,
            "local_path": local_path,
        },
    )

    try:
        while viewer.is_running():
            for index, sample in enumerate(samples):
                if not viewer.is_running():
                    break
                _, x, y, z, yaw = sample
                orientation = np.asarray(
                    [0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)],
                    dtype=np.float64,
                )
                position = np.asarray([x, y, z], dtype=np.float64)
                engine.set_robot_pose(position, orientation)
                focus_presentation_viewer(viewer, position)
                viewer.sync()
                if index + 1 < len(samples):
                    step_delta = max(1.0, samples[index + 1][0] - sample[0])
                    time.sleep(min(0.20, step_delta * timestep_s / replay_speed))
            if not bool(args.loop):
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.03)
                break
            time.sleep(0.8)
    finally:
        engine.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
