#!/usr/bin/env python3
"""Record ThunderV4 policy motion with real MuJoCo-LiDAR MID-360 hit points."""

from __future__ import annotations

import argparse
import importlib
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sim.compat.engine.core.engine import VelocityCommand  # noqa: E402

from drivers.sim.mujoco.runtime import build_engine  # noqa: E402

cv2: Any
imageio: Any
mujoco: Any
np: Any


def _write_world(path: Path) -> None:
    path.write_text(
        """<mujoco model="thunderv4_mid360_policy_world">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <size memory="96M"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.18 0.18 0.20" diffuse="0.30 0.30 0.32" specular="0.02 0.02 0.02"/>
    <quality shadowsize="4096"/>
    <map znear="0.01" zfar="100"/>
  </visual>
  <worldbody>
    <light name="key" pos="-3 -4 7" dir="0.35 0.35 -1" diffuse="0.55 0.53 0.50" castshadow="true"/>
    <geom name="floor" type="plane" size="18 18 0.1" rgba="0.43 0.42 0.48 1"
          conaffinity="1" condim="3" friction="1 0.5 0.5" group="1"/>
    <geom name="left_wall_near" type="box" pos="-0.6 1.65 1.05" size="3.2 0.10 1.05"
          rgba="0.52 0.52 0.56 1" conaffinity="1" condim="3" group="1"/>
    <geom name="right_rack_near" type="box" pos="-0.4 -2.05 0.95" size="3.0 0.12 0.95"
          rgba="0.34 0.28 0.22 1" conaffinity="1" condim="3" group="1"/>
    <geom name="front_machine_left" type="box" pos="1.25 0.95 0.78" size="0.65 0.42 0.78"
          rgba="0.22 0.34 0.42 1" conaffinity="1" condim="3" group="1"/>
    <geom name="front_machine_right" type="box" pos="1.85 -1.65 0.70" size="0.72 0.36 0.70"
          rgba="0.26 0.32 0.36 1" conaffinity="1" condim="3" group="1"/>
    <geom name="crate_stack_a" type="box" pos="-1.0 0.90 0.32" size="0.36 0.36 0.32"
          rgba="0.48 0.34 0.20 1" conaffinity="1" condim="3" group="1"/>
    <geom name="crate_stack_b" type="box" pos="-0.62 0.90 0.86" size="0.32 0.32 0.24"
          rgba="0.44 0.30 0.18 1" conaffinity="1" condim="3" group="1"/>
    <geom name="barrel_left" type="cylinder" pos="0.10 1.05 0.42" size="0.28 0.42"
          rgba="0.70 0.56 0.14 1" conaffinity="1" condim="3" group="1"/>
    <geom name="barrel_right" type="cylinder" pos="0.35 -1.25 0.42" size="0.28 0.42"
          rgba="0.65 0.48 0.12 1" conaffinity="1" condim="3" group="1"/>
    <geom name="overhead_beam_a" type="box" pos="-0.6 0.0 2.25" size="2.8 0.06 0.08"
          rgba="0.36 0.38 0.40 1" conaffinity="1" condim="3" group="1"/>
    <geom name="overhead_beam_b" type="box" pos="1.6 0.0 2.55" size="0.06 2.0 0.08"
          rgba="0.36 0.38 0.40 1" conaffinity="1" condim="3" group="1"/>
    <geom name="pipe_left" type="cylinder" pos="-1.0 0.0 2.75" size="0.045 1.85"
          euler="1.5708 0 0" rgba="0.28 0.30 0.32 1" conaffinity="1" condim="3" group="1"/>
    <geom name="pipe_right" type="cylinder" pos="1.0 -0.2 2.85" size="0.04 1.65"
          euler="1.5708 0 0" rgba="0.28 0.30 0.32 1" conaffinity="1" condim="3" group="1"/>
    <geom name="far_panel" type="box" pos="3.4 0.15 1.05" size="0.08 1.20 1.05"
          rgba="0.46 0.48 0.50 1" conaffinity="1" condim="3" group="1"/>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )


def _is_descendant(model: Any, body_id: int, root_body_id: int) -> bool:
    cursor = int(body_id)
    root = int(root_body_id)
    while cursor >= 0:
        if cursor == root:
            return True
        if cursor == 0:
            return False
        cursor = int(model.body_parentid[cursor])
    return False


def _make_robot_readable(model: Any, base_id: int) -> None:
    if base_id < 0:
        return
    for gid in range(int(model.ngeom)):
        if not _is_descendant(model, int(model.geom_bodyid[gid]), base_id):
            continue
        name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or "").lower()
        if name.endswith("_foot_visual"):
            model.geom_rgba[gid][3] = 0.0
        elif "wheel" in name:
            model.geom_rgba[gid] = np.array([0.03, 0.03, 0.035, 1.0], dtype=np.float32)
        else:
            model.geom_rgba[gid] = np.array([0.14, 0.14, 0.16, 1.0], dtype=np.float32)


def _next_geom(scene: Any) -> Any | None:
    if int(scene.ngeom) >= int(scene.maxgeom):
        return None
    geom = scene.geoms[int(scene.ngeom)]
    scene.ngeom += 1
    return geom


def _add_sphere(scene: Any, pos: np.ndarray, radius: float, rgba: tuple[float, ...]) -> bool:
    geom = _next_geom(scene)
    if geom is None:
        return False
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.asarray([radius, radius, radius], dtype=np.float64),
        np.asarray(pos, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(-1),
        np.asarray(rgba, dtype=np.float32),
    )
    return True


def _add_capsule(scene: Any, start: np.ndarray, end: np.ndarray, radius: float) -> None:
    start = np.asarray(start, dtype=np.float64)
    end = np.asarray(end, dtype=np.float64)
    if not np.isfinite(start).all() or not np.isfinite(end).all():
        return
    if float(np.linalg.norm(end - start)) < 1e-4:
        return
    geom = _next_geom(scene)
    if geom is None:
        return
    rgba = np.asarray([0.02, 0.02, 0.05, 0.38], dtype=np.float32)
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.asarray([radius, 0.0, 0.0], dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(-1),
        rgba,
    )
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, float(radius), start, end)
    geom.rgba[:] = rgba


def _draw_scan_hits(scene: Any, scan: np.ndarray, *, max_points: int) -> int:
    if scan is None or len(scan) == 0:
        return 0
    xyz = np.asarray(scan[:, :3], dtype=np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) == 0:
        return 0
    if len(xyz) > max_points:
        xyz = xyz[np.linspace(0, len(xyz) - 1, max_points).astype(np.int64)]
    shown = 0
    for point in xyz:
        if _add_sphere(scene, point, 0.011, (1.0, 0.0, 0.02, 0.70)):
            shown += 1
    return shown


def _draw_trail(scene: Any, trail: list[np.ndarray]) -> None:
    if len(trail) < 2:
        return
    points = trail[-100:]
    step = max(1, len(points) // 45)
    sampled = points[::step]
    for start, end in zip(sampled, sampled[1:]):
        _add_capsule(scene, start, end, 0.012)


def _wheel_clearance(model: Any, data: Any) -> dict[str, float]:
    result: dict[str, float] = {}
    for name in ("FR_wheel", "FL_wheel", "RR_wheel", "RL_wheel"):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if gid < 0:
            continue
        radius = float(model.geom_size[gid][0])
        result[name] = float(data.geom_xpos[gid][2] - radius)
    return result


def run(args: argparse.Namespace) -> dict[str, Any]:
    global cv2, imageio, mujoco, np
    cv2 = importlib.import_module("cv2")
    imageio = importlib.import_module("imageio.v2")
    mujoco = importlib.import_module("mujoco")
    np = importlib.import_module("numpy")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir or ROOT / "artifacts" / f"mujoco_thunderv4_mid360_policy_{stamp}").resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    world_path = out_dir / "thunderv4_mid360_policy_world.xml"
    video_path = out_dir / "thunderv4_mid360_policy_browser.mp4"
    gif_path = out_dir / "thunderv4_mid360_policy_preview.gif"
    first_frame = out_dir / "first_frame.png"
    mid_frame = out_dir / "mid_frame.png"
    report_path = out_dir / "report.json"
    _write_world(world_path)

    engine = build_engine(
        world=world_path,
        drive_mode="policy",
        start=[float(v) for v in args.start.split(",")],
        mujoco_memory="96M",
        mid360_samples_per_frame=int(args.samples_per_frame),
        lidar_backend="mujoco_lidar",
        mujoco_lidar_backend="cpu",
        require_product_lidar_backend=True,
    )
    model = engine.model
    data = engine.data
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    _make_robot_readable(model, base_id)

    renderer = mujoco.Renderer(model, height=args.height, width=args.width, max_geom=30000)
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

    control_dt = float(getattr(engine, "control_dt", 0.02))
    render_every = max(1, round((1.0 / args.fps) / control_dt))
    preview: list[np.ndarray] = []
    trail: list[np.ndarray] = []
    scan_sizes: list[int] = []
    shown_sizes: list[int] = []
    intensity_values: list[np.ndarray] = []
    imu_gyro_values: list[np.ndarray] = []
    imu_gravity_values: list[np.ndarray] = []
    imu_accel_values: list[np.ndarray] = []
    contacts: list[int] = []
    clearances: list[dict[str, float]] = []
    base_z: list[float] = []
    frames = 0

    for _ in range(int(args.settle_s / control_dt)):
        engine.step(VelocityCommand(0.0, 0.0, 0.0))
    start_pos = np.asarray(engine.get_robot_state().position, dtype=float).copy()

    try:
        for step in range(int(args.duration_s / control_dt)):
            t = step * control_dt
            cmd = VelocityCommand(
                float(args.vx),
                0.0,
                float(args.wz_amp) * math.sin(t * float(args.wz_freq)),
            )
            engine.step(cmd)
            if step % render_every != 0:
                continue

            state = engine.get_robot_state()
            pos = np.asarray(state.position, dtype=float)
            scan = engine.get_lidar_points()
            scan_sizes.append(int(len(scan)))
            if scan is not None and len(scan) > 0 and scan.shape[1] >= 4:
                intensity_values.append(np.asarray(scan[:, 3], dtype=np.float64))
            imu_gyro_values.append(np.asarray(state.imu_gyro, dtype=np.float64))
            imu_gravity_values.append(np.asarray(state.imu_projected_gravity, dtype=np.float64))
            imu_accel_values.append(
                np.asarray(
                    getattr(state, "imu_linear_acceleration", np.zeros(3, dtype=np.float64)),
                    dtype=np.float64,
                )
            )
            contacts.append(int(data.ncon))
            base_z.append(float(pos[2]))
            clearances.append(_wheel_clearance(model, data))
            trail.append(np.asarray([pos[0], pos[1], 0.025], dtype=np.float64))

            camera.lookat[:] = [float(pos[0]) + 0.05, float(pos[1]), 0.25]
            camera.distance = float(args.camera_distance)
            camera.azimuth = float(args.camera_azimuth)
            camera.elevation = float(args.camera_elevation)
            renderer.update_scene(data, camera=camera)
            shown_sizes.append(_draw_scan_hits(renderer.scene, scan, max_points=int(args.max_visual_points)))
            _draw_trail(renderer.scene, trail)

            frame = renderer.render()
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            writer.write(bgr)
            if frames == 0:
                cv2.imwrite(str(first_frame), bgr)
            if frames == args.mid_frame_index:
                cv2.imwrite(str(mid_frame), bgr)
            if frames % max(1, int(args.gif_stride)) == 0:
                preview.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            frames += 1
    finally:
        writer.release()
        renderer.close()

    if preview:
        imageio.mimsave(gif_path, preview, duration=1.0 / float(args.gif_fps))
    if not mid_frame.exists() and first_frame.exists():
        import shutil

        shutil.copyfile(first_frame, mid_frame)

    end_pos = np.asarray(engine.get_robot_state().position, dtype=float).copy()
    backend = engine.get_lidar_backend_report()
    clearance_values = [v for item in clearances for v in item.values()]
    intensities = np.concatenate(intensity_values) if intensity_values else np.zeros((0,), dtype=np.float64)
    imu_last_gyro = imu_gyro_values[-1] if imu_gyro_values else np.zeros(3, dtype=np.float64)
    imu_last_gravity = imu_gravity_values[-1] if imu_gravity_values else np.zeros(3, dtype=np.float64)
    imu_last_accel = imu_accel_values[-1] if imu_accel_values else np.zeros(3, dtype=np.float64)
    report = {
        "scenario": "thunderv4_mid360_policy_real_hits",
        "world": str(world_path),
        "video": str(video_path),
        "gif": str(gif_path),
        "first_frame": str(first_frame),
        "mid_frame": str(mid_frame),
        "duration_s": float(args.duration_s),
        "fps": float(args.fps),
        "frame_count": int(frames),
        "motion": "ThunderV4 ONNX policy through MuJoCo physics",
        "policy_used": bool(engine.has_policy),
        "policy_class": type(engine._policy).__name__ if engine._policy else None,
        "start_position": [float(v) for v in start_pos],
        "end_position": [float(v) for v in end_pos],
        "xy_motion_m": float(np.linalg.norm(end_pos[:2] - start_pos[:2])),
        "base_z_min_m": float(min(base_z) if base_z else 0.0),
        "base_z_max_m": float(max(base_z) if base_z else 0.0),
        "contacts_min": int(min(contacts) if contacts else 0),
        "contacts_max": int(max(contacts) if contacts else 0),
        "contacts_mean": float(np.mean(contacts) if contacts else 0.0),
        "visible_wheel_clearance_min_m": float(min(clearance_values) if clearance_values else 0.0),
        "scan_count": int(len(scan_sizes)),
        "scan_points_min": int(min(scan_sizes) if scan_sizes else 0),
        "scan_points_max": int(max(scan_sizes) if scan_sizes else 0),
        "scan_points_mean": float(np.mean(scan_sizes) if scan_sizes else 0.0),
        "scene_density": "dense_local_industrial",
        "intensity": {
            "model": "180/(1+(range_m/25)^2)+N(0,3), clipped to [1,255]",
            "count": int(len(intensities)),
            "min": float(np.min(intensities)) if len(intensities) else 0.0,
            "p50": float(np.percentile(intensities, 50)) if len(intensities) else 0.0,
            "mean": float(np.mean(intensities)) if len(intensities) else 0.0,
            "p95": float(np.percentile(intensities, 95)) if len(intensities) else 0.0,
            "max": float(np.max(intensities)) if len(intensities) else 0.0,
        },
        "imu": {
            "engine_state_available": bool(imu_gyro_values and imu_gravity_values),
            "gyro_last_rad_s": [float(v) for v in imu_last_gyro],
            "projected_gravity_last": [float(v) for v in imu_last_gravity],
            "projected_gravity_norm_last": float(np.linalg.norm(imu_last_gravity)),
            "linear_acceleration_last_m_s2": [float(v) for v in imu_last_accel],
            "mujoco_driver_publishes_runtime_imu_topic": True,
            "recorder_publishes_runtime_topics": False,
        },
        "slam_readiness": {
            "lidar_xyzi_available": bool(scan_sizes and max(scan_sizes) > 0),
            "imu_state_available_in_engine": bool(imu_gyro_values and imu_gravity_values),
            "runtime_raw_imu_topic_available": True,
            "ready_for_direct_slam": False,
            "reason": "MuJoCo IMU is available through MujocoDriverModule. Direct Fast-LIO SLAM still needs an explicit raw LiDAR frame/time-sync feed from the sim driver or portable DDS adapter.",
        },
        "shown_points_mean": float(np.mean(shown_sizes) if shown_sizes else 0.0),
        "lidar_backend": backend,
        "overlay": {
            "style": "clean red real MuJoCo-LiDAR hit points",
            "point_source": "engine.get_lidar_points() XYZI raycast hits",
            "rays_drawn": False,
            "ground_projected_pattern": False,
            "trajectory": "continuous capsule line",
            "visual_subsample_only": True,
        },
    }
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    engine.close()
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default="")
    parser.add_argument("--duration-s", type=float, default=9.0)
    parser.add_argument("--settle-s", type=float, default=1.0)
    parser.add_argument("--fps", type=float, default=18.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--samples-per-frame", type=int, default=15000)
    parser.add_argument("--max-visual-points", type=int, default=2400)
    parser.add_argument("--start", default="-1.5,-0.8,0.55")
    parser.add_argument("--vx", type=float, default=0.38)
    parser.add_argument("--wz-amp", type=float, default=0.13)
    parser.add_argument("--wz-freq", type=float, default=0.55)
    parser.add_argument("--camera-distance", type=float, default=6.2)
    parser.add_argument("--camera-azimuth", type=float, default=126.0)
    parser.add_argument("--camera-elevation", type=float, default=-57.0)
    parser.add_argument("--gif-stride", type=int, default=4)
    parser.add_argument("--gif-fps", type=float, default=4.5)
    parser.add_argument("--mid-frame-index", type=int, default=72)
    return parser.parse_args()


def main() -> int:
    print(json.dumps(run(parse_args()), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
