"""MuJoCo live-runtime helpers used by LingTu simulation bridges.

This module owns the product-side MuJoCo scene resolution and engine
construction. Validation scripts may call it, but they should not duplicate
runtime construction logic in ``sim/scripts``.
"""

from __future__ import annotations

import math
import tempfile
from contextlib import nullcontext
from itertools import pairwise
from pathlib import Path
from typing import Any

import numpy as np
from sim.engine.mujoco.lidar import (
    ROBOT_COLLISION_GEOM_GROUP,
    ROBOT_VISUAL_GEOM_GROUP,
)

ROOT = Path(__file__).resolve().parents[4]
SIM_ROOT = ROOT / "sim"
DEFAULT_MID360_PATTERN = SIM_ROOT / "assets" / "livox" / "mid360.npy"
DEFAULT_MID360_SAMPLES_PER_FRAME = 20000
POLICY_1119_PHYSICS_TIMESTEP_S = 0.005


def launch_presentation_viewer(model: Any, data: Any) -> Any:
    """Open the operator viewer without MuJoCo's model-inspection sidebars."""

    import mujoco.viewer

    return mujoco.viewer.launch_passive(
        model,
        data,
        show_left_ui=False,
        show_right_ui=False,
    )


def focus_presentation_viewer(
    viewer: Any,
    robot_position: Any,
    *,
    initialize: bool = False,
) -> None:
    """Keep a passive MuJoCo viewer centered on the robot.

    Initial framing chooses a useful quadruped demonstration view. Later calls
    update only the focal point so mouse-controlled orbit and zoom remain under
    the operator's control.
    """

    position = [float(value) for value in robot_position]
    if len(position) < 3:
        raise ValueError("robot_position must contain x, y, and z")
    viewer.cam.lookat[:] = position[:3]
    if initialize:
        # LiDAR keeps robot meshes and collision primitives in separate groups.
        # The operator view shows only the meshes; physics is unaffected.
        if hasattr(viewer, "opt") and len(viewer.opt.geomgroup) > 0:
            viewer.opt.geomgroup[ROBOT_COLLISION_GEOM_GROUP] = 0
            viewer.opt.geomgroup[ROBOT_VISUAL_GEOM_GROUP] = 1
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 30.0
        viewer.cam.elevation = -20.0


def _navigation_path_points(value: Any) -> list[np.ndarray]:
    points: list[np.ndarray] = []
    for item in value or []:
        try:
            point = np.asarray(item[:3], dtype=np.float64)
        except (IndexError, TypeError, ValueError):
            continue
        if point.shape == (3,) and np.isfinite(point).all():
            points.append(point)
    return points


def _append_path_segments(
    mujoco: Any,
    scene: Any,
    points: list[np.ndarray],
    *,
    radius: float,
    rgba: tuple[float, float, float, float],
    z_offset: float,
    max_segments: int,
) -> int:
    if len(points) < 2:
        return 0
    if len(points) - 1 > max_segments:
        indices = np.linspace(0, len(points) - 1, max_segments + 1, dtype=np.int32)
        points = [points[int(index)] for index in indices]
    color = np.asarray(rgba, dtype=np.float32)
    added = 0
    for start, end in pairwise(points):
        if int(scene.ngeom) >= int(scene.maxgeom):
            break
        start = np.asarray(start, dtype=np.float64).copy()
        end = np.asarray(end, dtype=np.float64).copy()
        start[2] += z_offset
        end[2] += z_offset
        if float(np.linalg.norm(end - start)) < 1e-5:
            continue
        geom = scene.geoms[int(scene.ngeom)]
        scene.ngeom += 1
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.asarray([radius, 0.0, 0.0], dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.eye(3, dtype=np.float64).reshape(-1),
            color,
        )
        mujoco.mjv_connector(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            float(radius),
            start,
            end,
        )
        geom.rgba[:] = color
        added += 1
    return added


def _point_cloud_xyz(value: Any, max_points: int) -> np.ndarray:
    try:
        points = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return np.empty((0, 3), dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 3:
        return np.empty((0, 3), dtype=np.float64)
    points = points[:, :3]
    points = points[np.isfinite(points).all(axis=1)]
    limit = max(0, int(max_points))
    if limit == 0:
        return np.empty((0, 3), dtype=np.float64)
    if len(points) > limit:
        indices = np.linspace(0, len(points) - 1, limit, dtype=np.int32)
        points = points[indices]
    return points


def _navigation_obstacle_points(status: dict[str, Any]) -> Any:
    local_map = status.get("local_map")
    if not isinstance(local_map, dict) or local_map.get("enabled") is not True:
        return None
    if local_map.get("obstacle_points_fresh") is False:
        return np.empty((0, 3), dtype=np.float64)
    if local_map.get("frame_id") != "map":
        return np.empty((0, 3), dtype=np.float64)
    return local_map.get("obstacle_points")


def _append_point_cloud(
    mujoco: Any,
    scene: Any,
    points: np.ndarray,
    *,
    radius: float,
    rgba: tuple[float, float, float, float],
) -> int:
    color = np.asarray(rgba, dtype=np.float32)
    size = np.asarray([radius, 0.0, 0.0], dtype=np.float64)
    identity = np.eye(3, dtype=np.float64).reshape(-1)
    added = 0
    for point in points:
        if int(scene.ngeom) >= int(scene.maxgeom):
            break
        geom = scene.geoms[int(scene.ngeom)]
        scene.ngeom += 1
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            size,
            point,
            identity,
            color,
        )
        geom.rgba[:] = color
        added += 1
    return added


def draw_navigation_paths(
    viewer: Any,
    nav_status: Any,
    *,
    point_cloud: Any = None,
    max_point_count: int = 600,
) -> dict[str, int]:
    """Draw current paths and a bounded native local-obstacle sample."""

    import mujoco

    status = nav_status if isinstance(nav_status, dict) else {}
    lock = viewer.lock() if callable(getattr(viewer, "lock", None)) else nullcontext()
    with lock:
        scene = viewer.user_scn
        scene.ngeom = 0
        global_segments = _append_path_segments(
            mujoco,
            scene,
            _navigation_path_points(status.get("global_path")),
            radius=0.025,
            rgba=(0.90, 0.56, 0.12, 0.95),
            z_offset=0.06,
            max_segments=120,
        )
        local_segments = _append_path_segments(
            mujoco,
            scene,
            _navigation_path_points(status.get("local_path")),
            radius=0.035,
            rgba=(0.18, 0.82, 0.38, 0.98),
            z_offset=0.10,
            max_segments=80,
        )
        native_obstacles = _navigation_obstacle_points(status)
        point_count = _append_point_cloud(
            mujoco,
            scene,
            _point_cloud_xyz(
                native_obstacles if native_obstacles is not None else point_cloud,
                max_point_count,
            ),
            radius=0.010,
            rgba=(0.16, 0.72, 1.0, 0.80),
        )
    return {
        "point_count": point_count,
        "global_segments": global_segments,
        "local_segments": local_segments,
    }


def resolve_mid360_pattern(path: Path | str | None) -> Path | None:
    """Resolve the official MID-360 pattern asset, or return ``None`` explicitly."""

    if path is None or str(path).strip() == "":
        return None
    candidate = Path(path).expanduser()
    if not candidate.is_absolute():
        candidate = (ROOT / candidate).resolve()
    if not candidate.is_file():
        raise FileNotFoundError(
            f"MID-360 scan pattern not found: {candidate}. "
            "Pass --mid360-pattern or --allow-golden-spiral-lidar explicitly."
        )
    return candidate


def resolve_world(world: str) -> Path:
    """Resolve a registered MuJoCo world name or filesystem path."""

    from drivers.sim.mujoco.driver import _WORLDS_DIR, WORLDS

    candidate = Path(world)
    if candidate.exists():
        return candidate.resolve()
    mapped = WORLDS.get(world, world)
    path = (_WORLDS_DIR / mapped).resolve()
    if not path.exists():
        raise FileNotFoundError(f"MuJoCo world not found: {world} -> {path}")
    return path


def scene_start(scene_xml: Path) -> list[float] | None:
    """Return the robot start pose declared by a LingTu MuJoCo scene."""

    from drivers.sim.mujoco.driver import _scene_placeholder_start

    start = _scene_placeholder_start(scene_xml)
    if start is not None:
        return start
    try:
        import xml.etree.ElementTree as ET

        root = ET.fromstring(scene_xml.read_text(encoding="utf-8", errors="ignore"))
        worldbody = root.find("worldbody")
        if worldbody is None:
            return None
        for body in worldbody.findall("body"):
            if body.attrib.get("name") != "base_link":
                continue
            parts = [float(v) for v in body.attrib.get("pos", "").split()]
            if len(parts) >= 3:
                return parts[:3]
    except (ET.ParseError, AttributeError, KeyError, ValueError, TypeError):
        return None
    return None


def parse_start(value: str) -> list[float] | None:
    """Parse CLI start pose formatted as ``x,y,z``."""

    if not value:
        return None
    parts = [float(item.strip()) for item in value.replace(";", ",").split(",") if item.strip()]
    if len(parts) != 3:
        raise ValueError("--start must be formatted as x,y,z")
    return parts


def scene_with_memory(scene_xml: Path, memory: str) -> Path:
    """Return a temporary scene with a MuJoCo ``<size memory=...>`` element."""

    if not memory:
        return scene_xml
    text = scene_xml.read_text(encoding="utf-8", errors="ignore")
    if "<size " in text:
        return scene_xml
    marker_start = text.find("<mujoco")
    if marker_start < 0:
        return scene_xml
    marker_end = text.find(">", marker_start)
    if marker_end < 0:
        return scene_xml
    patched = text[: marker_end + 1] + f'\n  <size memory="{memory}"/>' + text[marker_end + 1 :]
    tmp = tempfile.NamedTemporaryFile(
        suffix=".xml",
        prefix=f"{scene_xml.stem}_memory_",
        dir=str(scene_xml.parent),
        mode="w",
        encoding="utf-8",
        delete=False,
    )
    tmp.write(patched)
    tmp.close()
    return Path(tmp.name)


def build_engine(
    *,
    world: Path,
    drive_mode: str,
    start: list[float] | None,
    mujoco_memory: str,
    start_orientation_wxyz: list[float] | None = None,
    camera_configs: list[Any] | None = None,
    robot_xml: Path | None = None,
    base_body_name: str = "base_link",
    lidar_body_name: str = "lidar_link",
    lidar_site_name: str = "lidar_site",
    physics_timestep_s: float | None = None,
    leg_joint_names: list[str] | None = None,
    controller_actuator_names: list[str] | None = None,
    mid360_pattern: Path | str | None = DEFAULT_MID360_PATTERN,
    mid360_samples_per_frame: int = DEFAULT_MID360_SAMPLES_PER_FRAME,
    lidar_backend: str = "mujoco_lidar",
    mujoco_lidar_backend: str = "cpu",
    require_product_lidar_backend: bool = True,
    policy_path: Path | str | None = None,
    policy_freq_hz: float | None = None,
    policy_cpu_threads: int | None = None,
    max_linear_vel: float | None = None,
    max_angular_vel: float | None = None,
    initial_keyframe: str | None = None,
):
    """Build the canonical in-process MuJoCo engine for live LingTu gates."""

    from sim.engine.core.robot import RobotConfig
    from sim.engine.core.sensor import LidarConfig
    from sim.engine.core.world import WorldConfig
    from sim.engine.mujoco.engine import MuJoCoEngine

    robot_cfg = RobotConfig.default_thunder_v4()
    robot_cfg.resolve_paths(base_dir=str(SIM_ROOT))
    if robot_xml is not None:
        robot_cfg.robot_xml = str(robot_xml)
    robot_cfg.base_body_name = str(base_body_name)
    robot_cfg.lidar_body_name = str(lidar_body_name)
    if leg_joint_names is not None:
        robot_cfg.leg_joint_names = list(leg_joint_names)
    if controller_actuator_names is not None:
        declared = tuple(controller_actuator_names)
        physical = tuple(robot_cfg.leg_joint_names)
        if not declared or len(declared) != len(set(declared)) or set(declared) != set(physical):
            raise ValueError("controller actuator channels do not match the MuJoCo robot joints")
    if policy_path is not None and str(policy_path).strip():
        candidate = Path(policy_path).expanduser()
        if not candidate.is_absolute():
            candidate = (ROOT / candidate).resolve()
        robot_cfg.policy_onnx = str(candidate)
    if policy_freq_hz is not None:
        if not math.isfinite(float(policy_freq_hz)) or float(policy_freq_hz) <= 0.0:
            raise ValueError("policy_freq_hz must be finite and positive")
        robot_cfg.policy_freq_hz = float(policy_freq_hz)
    if policy_cpu_threads is not None:
        threads = int(policy_cpu_threads)
        if not 1 <= threads <= 8:
            raise ValueError("policy_cpu_threads must be in [1, 8]")
        robot_cfg.policy_cpu_threads = threads
    if max_linear_vel is not None:
        if not math.isfinite(float(max_linear_vel)) or float(max_linear_vel) <= 0.0:
            raise ValueError("max_linear_vel must be finite and positive")
        robot_cfg.max_linear_vel = float(max_linear_vel)
    if max_angular_vel is not None:
        if not math.isfinite(float(max_angular_vel)) or float(max_angular_vel) <= 0.0:
            raise ValueError("max_angular_vel must be finite and positive")
        robot_cfg.max_angular_vel = float(max_angular_vel)
    start = start or scene_start(world)
    if start is not None:
        robot_cfg.init_position = [float(v) for v in start[:3]]
    if start_orientation_wxyz is not None:
        if len(start_orientation_wxyz) != 4:
            raise ValueError("start_orientation_wxyz must contain four values")
        orientation = [float(value) for value in start_orientation_wxyz]
        if not all(math.isfinite(value) for value in orientation):
            raise ValueError("start_orientation_wxyz must be finite")
        norm = math.sqrt(sum(value * value for value in orientation))
        if abs(norm - 1.0) > 1e-6:
            raise ValueError("start_orientation_wxyz must be normalized")
        robot_cfg.init_orientation_wxyz = orientation
    pattern_path = resolve_mid360_pattern(mid360_pattern)

    load_world = scene_with_memory(world, mujoco_memory)
    engine = MuJoCoEngine(
        robot_config=robot_cfg,
        world_config=WorldConfig(scene_xml=str(load_world)),
        lidar_config=LidarConfig(
            body_name=robot_cfg.lidar_body_name,
            exclude_body_name=robot_cfg.base_body_name,
            geom_group=0,
            add_noise=True,
            backend=str(lidar_backend),
            mujoco_lidar_backend=str(mujoco_lidar_backend),
            require_product_backend=bool(require_product_lidar_backend),
            site_name=str(lidar_site_name),
            mid360_npy_path=str(pattern_path) if pattern_path is not None else None,
            samples_per_frame=int(mid360_samples_per_frame),
        ),
        camera_configs=camera_configs or [],
        headless=True,
        drive_mode=drive_mode,
        initial_keyframe=initial_keyframe,
    )
    try:
        engine.load(str(load_world))
        effective_timestep_s = physics_timestep_s
        if (
            effective_timestep_s is None
            and str(drive_mode).strip().lower() == "policy"
            and Path(robot_cfg.policy_onnx).name.lower() == "policy_1119.onnx"
        ):
            effective_timestep_s = POLICY_1119_PHYSICS_TIMESTEP_S
        if effective_timestep_s is not None:
            engine.set_physics_timestep(float(effective_timestep_s))
        engine.reset()
    finally:
        if load_world != world:
            load_world.unlink(missing_ok=True)
    return engine
