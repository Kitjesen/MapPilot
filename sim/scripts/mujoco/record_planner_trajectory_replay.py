#!/usr/bin/env python3
"""Record the public LingTu and DimOS paths as a MuJoCo 3D replay.

This is a kinematic presentation replay.  The robot base is placed at each
trajectory sample while the ThunderV4 nominal standing joints remain fixed.
It shows route shape, elevation changes, and relative path length.  It does
not run the locomotion policy or prove physical stair navigation.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.scripts.mujoco.diagnose_octoplanner3d_map_quality import load_pcd_xyz


MODEL_PATH = ROOT / "sim" / "packages" / "robots" / "doso" / "thunder_v4" / "mjcf" / "thunderv4.xml"
LINGTU_PATH = ROOT / "build" / "official-global-planner-navd-positive" / "building_cross_building_modeled" / "path.xyz"
DIMOS_PATH = ROOT / "build" / "dimos-public-current" / "building-go2" / "building_cross_building_modeled.txt"
PCD_PATH = ROOT / "sim" / "evaluation" / "data" / "octoplanner3d" / "building2_9.pcd"

SURFACE_TO_BASE_M = 0.30
LINGTU_COLOR = (0.12, 0.50, 1.00, 0.88)
DIMOS_COLOR = (1.00, 0.42, 0.10, 0.88)
LINGTU_FADE = (0.12, 0.50, 1.00, 0.32)
DIMOS_FADE = (1.00, 0.42, 0.10, 0.32)


def _read_path(path: Path, *, z_offset: float = 0.0) -> np.ndarray:
    rows: list[list[float]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        values = raw.split()
        if len(values) < 3:
            continue
        try:
            point = [float(values[0]), float(values[1]), float(values[2]) + z_offset]
        except ValueError:
            continue
        if np.isfinite(point).all():
            rows.append(point)
    result = np.asarray(rows, dtype=np.float64)
    if len(result) < 2:
        raise ValueError(f"trajectory needs at least two finite points: {path}")
    return result


def _sample_point_cloud(points: np.ndarray, max_points: int) -> np.ndarray:
    finite = points[np.isfinite(points).all(axis=1)]
    if len(finite) <= max_points:
        return finite.astype(np.float64, copy=False)
    indices = np.linspace(0, len(finite) - 1, int(max_points), dtype=np.int64)
    return finite[indices].astype(np.float64, copy=False)


def _arc_lengths(points: np.ndarray) -> np.ndarray:
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    if cumulative[-1] <= 1e-9:
        raise ValueError("trajectory has no measurable length")
    return cumulative


def _resample(points: np.ndarray, count: int) -> np.ndarray:
    distances = _arc_lengths(points)
    targets = np.linspace(0.0, float(distances[-1]), max(2, int(count)))
    result = np.column_stack(
        [np.interp(targets, distances, points[:, axis]) for axis in range(3)]
    )
    return result


def _yaw_at(points: np.ndarray, index: int) -> float:
    if index == 0:
        delta = points[1] - points[0]
    elif index == len(points) - 1:
        delta = points[-1] - points[-2]
    else:
        delta = points[index + 1] - points[index - 1]
    return math.atan2(float(delta[1]), float(delta[0]))


def _yaw_quaternion(yaw: float) -> np.ndarray:
    return np.asarray([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)], dtype=np.float64)


def _next_geom(scene: mujoco.MjvScene) -> mujoco.MjvGeom | None:
    if int(scene.ngeom) >= int(scene.maxgeom):
        return None
    geom = scene.geoms[int(scene.ngeom)]
    scene.ngeom += 1
    return geom


def _append_capsule(
    scene: mujoco.MjvScene,
    start: np.ndarray,
    end: np.ndarray,
    color: tuple[float, float, float, float],
    radius: float = 0.025,
) -> None:
    if float(np.linalg.norm(end - start)) < 1e-6:
        return
    geom = _next_geom(scene)
    if geom is None:
        return
    rgba = np.asarray(color, dtype=np.float32)
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.asarray([radius, 0.0, 0.0], dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(-1),
        rgba,
    )
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, start, end)
    geom.rgba[:] = rgba


def _append_path(scene: mujoco.MjvScene, points: np.ndarray, color: tuple[float, float, float, float]) -> None:
    if len(points) < 2:
        return
    limit = 180
    if len(points) > limit + 1:
        indices = np.linspace(0, len(points) - 1, limit + 1, dtype=np.int32)
        points = points[indices]
    for start, end in zip(points[:-1], points[1:]):
        _append_capsule(scene, start, end, color)


def _append_marker(scene: mujoco.MjvScene, point: np.ndarray, color: tuple[float, float, float, float]) -> None:
    geom = _next_geom(scene)
    if geom is None:
        return
    rgba = np.asarray(color, dtype=np.float32)
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.asarray([0.075, 0.0, 0.0], dtype=np.float64),
        point,
        np.eye(3, dtype=np.float64).reshape(-1),
        rgba,
    )
    geom.rgba[:] = rgba


def _append_local_point_cloud(
    scene: mujoco.MjvScene,
    points: np.ndarray,
    center: np.ndarray,
    *,
    radius: float,
    max_points: int,
) -> None:
    if len(points) == 0:
        return
    local = points[np.linalg.norm(points - center, axis=1) <= 12.0]
    if len(local) > max_points:
        local = local[np.linspace(0, len(local) - 1, max_points, dtype=np.int64)]
    rgba = np.asarray([0.48, 0.62, 0.72, 0.34], dtype=np.float32)
    identity = np.eye(3, dtype=np.float64).reshape(-1)
    size = np.asarray([radius, 0.0, 0.0], dtype=np.float64)
    for point in local:
        geom = _next_geom(scene)
        if geom is None:
            break
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            size,
            point,
            identity,
            rgba,
        )
        geom.rgba[:] = rgba


def _draw_routes(
    renderer: mujoco.Renderer,
    lingtu: np.ndarray,
    dimos: np.ndarray,
    active: str,
    point_cloud: np.ndarray,
    robot_surface_point: np.ndarray,
) -> None:
    scene = renderer.scene
    _append_local_point_cloud(
        scene,
        point_cloud,
        robot_surface_point,
        radius=0.018,
        max_points=2200,
    )
    _append_path(scene, lingtu, LINGTU_COLOR if active == "lingtu" else LINGTU_FADE)
    _append_path(scene, dimos, DIMOS_COLOR if active == "dimos" else DIMOS_FADE)
    _append_marker(scene, lingtu[0], LINGTU_COLOR)
    _append_marker(scene, lingtu[-1], LINGTU_COLOR)
    _append_marker(scene, dimos[0], DIMOS_COLOR)
    _append_marker(scene, dimos[-1], DIMOS_COLOR)


def _set_robot_pose(model: mujoco.MjModel, data: mujoco.MjData, point: np.ndarray, yaw: float) -> None:
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qpos[0:3] = point + np.asarray([0.0, 0.0, SURFACE_TO_BASE_M], dtype=np.float64)
    data.qpos[3:7] = _yaw_quaternion(yaw)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def _camera_for(point: np.ndarray, yaw: float, *, side: float = 1.0) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = point + np.asarray([0.0, 0.0, 0.55], dtype=np.float64)
    camera.distance = 4.6
    camera.azimuth = math.degrees(yaw) + 125.0 * side
    camera.elevation = -18.0
    return camera


def _render(
    renderer: mujoco.Renderer,
    data: mujoco.MjData,
    route_point: np.ndarray,
    yaw: float,
    lingtu: np.ndarray,
    dimos: np.ndarray,
    active: str,
    point_cloud: np.ndarray,
) -> np.ndarray:
    camera = _camera_for(route_point, yaw)
    renderer.update_scene(data, camera=camera)
    _draw_routes(renderer, lingtu, dimos, active, point_cloud, route_point)
    return renderer.render().copy()


def _label(frame_bgr: np.ndarray, title: str, color: tuple[int, int, int]) -> np.ndarray:
    cv2.rectangle(frame_bgr, (20, 18), (540, 82), (248, 248, 248), -1)
    cv2.putText(frame_bgr, title, (38, 57), cv2.FONT_HERSHEY_SIMPLEX, 0.95, color, 2, cv2.LINE_AA)
    cv2.rectangle(frame_bgr, (14, frame_bgr.shape[0] - 42), (455, frame_bgr.shape[0] - 10), (248, 248, 248), -1)
    cv2.putText(frame_bgr, "MuJoCo 3D trajectory replay | kinematic", (22, frame_bgr.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (80, 80, 80), 1, cv2.LINE_AA)
    return frame_bgr


def _open_writer(path: Path, fps: float, width: int, height: int) -> cv2.VideoWriter:
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"could not open video writer: {path}")
    return writer


def _summary(name: str, points: np.ndarray, duration_s: float, frames: int) -> dict[str, object]:
    return {
        "name": name,
        "points": int(len(points)),
        "length_m": float(_arc_lengths(points)[-1]),
        "height_min_m": float(points[:, 2].min()),
        "height_max_m": float(points[:, 2].max()),
        "duration_s": float(duration_s),
        "frames": int(frames),
    }


def record(args: argparse.Namespace) -> dict[str, object]:
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    lingtu = _read_path(Path(args.lingtu))
    dimos = _read_path(Path(args.dimos), z_offset=float(args.dimos_z_offset))
    pcd_path = Path(args.pcd).resolve()
    pcd_points = load_pcd_xyz(pcd_path)
    point_cloud = _sample_point_cloud(pcd_points, int(args.pcd_sample_points))
    lingtu_length = float(_arc_lengths(lingtu)[-1])
    dimos_length = float(_arc_lengths(dimos)[-1])
    frame_count = max(2, round(float(args.duration) * float(args.fps)))
    lingtu_samples = _resample(lingtu, frame_count)
    dimos_samples = _resample(dimos, frame_count)

    model = mujoco.MjModel.from_xml_path(str(Path(args.model).resolve()))
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), int(args.width))
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), int(args.height))
    renderer = mujoco.Renderer(model, height=int(args.height), width=int(args.width), max_geom=8000)
    lingtu_data = mujoco.MjData(model)
    dimos_data = mujoco.MjData(model)
    lingtu_video = output / "lingtu_kinematic_replay.mp4"
    dimos_video = output / "dimos_kinematic_replay.mp4"
    compare_video = output / "trajectory_compare_mujoco.mp4"
    mid_frame = output / "trajectory_compare_mujoco.png"
    writers = [
        _open_writer(lingtu_video, args.fps, args.width, args.height),
        _open_writer(dimos_video, args.fps, args.width, args.height),
        _open_writer(compare_video, args.fps, args.width * 2, args.height),
    ]
    try:
        for index in range(frame_count):
            lp = lingtu_samples[index]
            dp = dimos_samples[index]
            ly = _yaw_at(lingtu_samples, index)
            dy = _yaw_at(dimos_samples, index)
            _set_robot_pose(model, lingtu_data, lp, ly)
            _set_robot_pose(model, dimos_data, dp, dy)
            lingtu_bgr = cv2.cvtColor(
                _render(renderer, lingtu_data, lp, ly, lingtu, dimos, "lingtu", point_cloud),
                cv2.COLOR_RGB2BGR,
            )
            dimos_bgr = cv2.cvtColor(
                _render(renderer, dimos_data, dp, dy, lingtu, dimos, "dimos", point_cloud),
                cv2.COLOR_RGB2BGR,
            )
            lingtu_bgr = _label(lingtu_bgr, f"LingTu | {lingtu_length:.2f} m", (190, 100, 30))
            dimos_bgr = _label(dimos_bgr, f"DimOS | {dimos_length:.2f} m", (30, 90, 210))
            writers[0].write(lingtu_bgr)
            writers[1].write(dimos_bgr)
            compare = np.hstack((lingtu_bgr, dimos_bgr))
            writers[2].write(compare)
            if index == frame_count // 2:
                cv2.imwrite(str(mid_frame), compare)
    finally:
        for writer in writers:
            writer.release()
        renderer.close()

    metadata = {
        "kind": "mujoco_kinematic_trajectory_replay",
        "model": str(Path(args.model).resolve()),
        "point_cloud": str(pcd_path),
        "point_cloud_source_points": int(len(pcd_points)),
        "point_cloud_render_sample_points": int(len(point_cloud)),
        "surface_to_base_offset_m": SURFACE_TO_BASE_M,
        "dimos_z_offset_m": float(args.dimos_z_offset),
        "fps": float(args.fps),
        "resolution": [int(args.width), int(args.height)],
        "outputs": {
            "lingtu": str(lingtu_video),
            "dimos": str(dimos_video),
            "comparison": str(compare_video),
            "mid_frame": str(mid_frame),
        },
        "routes": {
            "lingtu": _summary("LingTu", lingtu, args.duration, frame_count),
            "dimos": _summary("DimOS", dimos, args.duration, frame_count),
        },
        "limitations": [
            "The base pose is replayed kinematically at each path sample.",
            "The ThunderV4 locomotion policy is not running.",
            "This does not validate dynamic contacts, stair stability, or closed-loop navigation.",
        ],
    }
    metadata_path = output / "trajectory_compare_mujoco.json"
    metadata["outputs"]["metadata"] = str(metadata_path)
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    return metadata


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=str(MODEL_PATH))
    parser.add_argument("--lingtu", default=str(LINGTU_PATH))
    parser.add_argument("--dimos", default=str(DIMOS_PATH))
    parser.add_argument("--pcd", default=str(PCD_PATH))
    parser.add_argument("--pcd-sample-points", type=int, default=120_000)
    parser.add_argument("--dimos-z-offset", type=float, default=0.35)
    parser.add_argument("--output", default=str(ROOT / "build" / "planner-compare-20261114" / "mujoco-replay"))
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--fps", type=float, default=24.0)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    return parser.parse_args()


if __name__ == "__main__":
    result = record(parse_args())
    print(json.dumps(result, ensure_ascii=False, indent=2))
