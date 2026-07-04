"""MujocoDriverModule exposes MuJoCo simulation as a Module.

Wraps sim/engine/mujoco/engine.py directly in-process.
No TCP bridge, no separate process. Data flows through In/Out ports.

Provides: odometry, lidar_cloud, map_cloud, imu, height_rays, camera_image,
depth_image
Consumes: cmd_vel, stop_signal

Usage::

    from drivers.sim.mujoco.driver import MujocoDriverModule
    bp.add(MujocoDriverModule,
           world="building_scene",
           render=True,
           sim_rate=50.0)
"""

from __future__ import annotations

import logging
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

from runtime.module import Module
from runtime.msgs.numpy_compat import np
from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat, Imu, PointCloud2
from runtime.registry import register
from runtime.runtime_interface import FRAMES, TOPICS, topic_default_frame_id
from runtime.stream import In, Out
from message.livox_frame import POINT_DTYPE, LivoxPointFrame

logger = logging.getLogger(__name__)

# Resolve sim/ directory relative to this file
_SIM_ROOT = Path(__file__).resolve().parents[4] / "sim"
_WORLDS_DIR = _SIM_ROOT / "worlds" / "mujoco"
_THUNDER_MJCF = _SIM_ROOT / "robots" / "thunderv4" / "mjcf" / "thunderv4.xml"
_THUNDERV4_POLICY = (
    _SIM_ROOT
    / "robots"
    / "thunderv4"
    / "policy"
    / "pose_flat_low_kpkd_microterrain_model29600_policy.pt"
)
_LEGACY_ROBOTS_DIR = _SIM_ROOT / "robots" / "nova_dog"
_ROBOT_XML = _THUNDER_MJCF
_BRAINSTEM_POLICY_NAME = "policy_251119.onnx"
_EXPLICIT_POLICY_CANDIDATES = (
    _LEGACY_ROBOTS_DIR / "model" / _BRAINSTEM_POLICY_NAME,
    _SIM_ROOT.parent / "model" / _BRAINSTEM_POLICY_NAME,
    _SIM_ROOT.parent.parent / "brainstem" / "model" / _BRAINSTEM_POLICY_NAME,
    _SIM_ROOT.parent.parent / "brainstem" / "han_dog" / "model" / _BRAINSTEM_POLICY_NAME,
    _SIM_ROOT.parent.parent / "brainstem" / "sim" / "model" / _BRAINSTEM_POLICY_NAME,
    _LEGACY_ROBOTS_DIR / "policy.onnx",
    # Legacy 76-D policy kept last: it needs its original observation contract.
    _LEGACY_ROBOTS_DIR / "thunder_policy.onnx",
)
_POLICY_CANDIDATES: tuple[Path, ...] = (_THUNDERV4_POLICY,)
_POLICY_ONNX = _EXPLICIT_POLICY_CANDIDATES[-1]
_DEFAULT_START_POS = (0.0, 0.0, 0.55)
MUJOCO_MODULE_ODOM_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
MUJOCO_MODULE_BODY_FRAME_ID = topic_default_frame_id(TOPICS.registered_cloud)
# In-process MuJoCo does not own a map->odom localizer; its live map cloud is odom-frame.
MUJOCO_MODULE_MAP_CLOUD_FRAME_ID = MUJOCO_MODULE_ODOM_FRAME_ID
MUJOCO_MODULE_CAMERA_FRAME_ID = FRAMES.camera
MUJOCO_MODULE_IMU_FRAME_ID = topic_default_frame_id(TOPICS.imu)
MUJOCO_MODULE_LIDAR_FRAME_ID = topic_default_frame_id(TOPICS.lidar_scan)
_NUMPY_RUNTIME_AVAILABLE: bool | None = None


def _first_existing_path(paths: tuple[Path, ...]) -> str:
    for path in paths:
        if path.exists():
            return str(path)
    return ""


def _resolve_sim_path(path: str) -> str:
    if not path:
        return ""
    candidate = Path(path).expanduser()
    if candidate.is_absolute():
        return str(candidate)

    candidate_paths: list[Path] = []
    parts = candidate.parts
    if parts and parts[0].lower() == "sim":
        candidate_paths.append(_SIM_ROOT.parent / candidate)
        if len(parts) > 1:
            candidate_paths.append(_SIM_ROOT / Path(*parts[1:]))
    elif parts and parts[0].lower() == "model":
        candidate_paths.append(_SIM_ROOT / "robots" / "nova_dog" / candidate)
        candidate_paths.append(_SIM_ROOT.parent / candidate)
    else:
        candidate_paths.append(_SIM_ROOT / candidate)
        candidate_paths.append(_SIM_ROOT.parent / candidate)
    candidate_paths.append(Path.cwd() / candidate)

    for resolved in [p.resolve() for p in candidate_paths]:
        if resolved.exists():
            return str(resolved)
    return str(candidate_paths[0].resolve())


def _numpy_runtime_available() -> bool:
    """Return whether NumPy can be imported without crashing this interpreter."""

    global _NUMPY_RUNTIME_AVAILABLE
    if _NUMPY_RUNTIME_AVAILABLE is not None:
        return _NUMPY_RUNTIME_AVAILABLE

    try:
        result = subprocess.run(
            [sys.executable, "-c", "import numpy"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired):
        _NUMPY_RUNTIME_AVAILABLE = False
    else:
        _NUMPY_RUNTIME_AVAILABLE = result.returncode == 0
    return _NUMPY_RUNTIME_AVAILABLE


def _is_default_start_pos(start_pos: tuple) -> bool:
    return tuple(float(v) for v in start_pos[:3]) == _DEFAULT_START_POS


def _scene_placeholder_start(scene_xml: Path) -> list[float] | None:
    try:
        import xml.etree.ElementTree as ET

        root = ET.fromstring(scene_xml.read_text(encoding="utf-8", errors="ignore"))
        worldbody = root.find("worldbody")
        if worldbody is None:
            return None
        for body in worldbody.findall("body"):
            if body.attrib.get("name") != "robot_placeholder":
                continue
            pos_str = body.attrib.get("pos", "")
            parts = [float(v) for v in pos_str.split()]
            if len(parts) >= 3:
                return parts[:3]
    except (ET.ParseError, AttributeError, ValueError, TypeError):
        logger.debug("Failed to parse robot_placeholder pose from %s", scene_xml, exc_info=True)
    return None


def _quat_xyzw_to_rotation_matrix(quat: np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=float).reshape(4)
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12:
        return np.eye(3, dtype=float)
    x, y, z, w = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _world_points_to_body_frame(
    points: np.ndarray,
    position_xyz: np.ndarray,
    orientation_xyzw: np.ndarray,
) -> np.ndarray:
    cloud = np.asarray(points, dtype=np.float32).copy()
    if cloud.ndim != 2 or cloud.shape[1] < 3:
        return cloud
    rotation_body_to_world = _quat_xyzw_to_rotation_matrix(orientation_xyzw)
    relative_world = cloud[:, :3].astype(float) - np.asarray(position_xyz, dtype=float).reshape(3)
    cloud[:, :3] = (relative_world @ rotation_body_to_world).astype(np.float32)
    return cloud


def _world_points_to_lidar_frame(
    points: np.ndarray,
    engine: Any,
    position_xyz: np.ndarray,
    orientation_xyzw: np.ndarray,
) -> np.ndarray:
    cloud = np.asarray(points, dtype=np.float32).copy()
    if cloud.ndim != 2 or cloud.shape[1] < 3:
        return cloud
    try:
        import mujoco

        model = getattr(engine, "_model", None)
        data = getattr(engine, "_data", None)
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "lidar_site")
        if site_id >= 0:
            pos = np.asarray(data.site_xpos[site_id], dtype=float).reshape(3)
            rot = np.asarray(data.site_xmat[site_id], dtype=float).reshape(3, 3)
            cloud[:, :3] = ((cloud[:, :3].astype(float) - pos) @ rot).astype(np.float32)
            return cloud
    except Exception:
        pass
    return _world_points_to_body_frame(cloud, position_xyz, orientation_xyzw)


def _xyzi_to_livox_frame(
    points: np.ndarray,
    *,
    timestamp_ns: int,
    sequence: int,
    frame_id: str,
    scan_duration_ns: int,
) -> LivoxPointFrame:
    xyzi = np.asarray(points, dtype=np.float32)
    count = 0 if xyzi.ndim != 2 else int(xyzi.shape[0])
    raw = np.zeros(count, dtype=POINT_DTYPE)
    if count:
        raw["x"] = xyzi[:, 0]
        raw["y"] = xyzi[:, 1]
        raw["z"] = xyzi[:, 2]
        raw["intensity"] = xyzi[:, 3] if xyzi.shape[1] >= 4 else 0.0
        raw["offset_time_ns"] = np.linspace(
            0,
            max(0, int(scan_duration_ns) - 1),
            count,
            dtype=np.uint32,
        )
    frame = LivoxPointFrame(points=raw, timestamp_ns=int(timestamp_ns), sequence=int(sequence))
    frame.frame_id = str(frame_id)
    return frame


# Known worlds
WORLDS = {
    "building": "building_scene.xml",
    "building_scene": "building_scene.xml",
    "factory": "factory_scene.xml",
    "factory_scene": "factory_scene.xml",
    "flat_showcase": "flat_showcase.xml",
    "industrial_park": "industrial_park_scene.xml",
    "industrial_park_scene": "industrial_park_scene.xml",
    "industrial_demo": "industrial_demo_scene.xml",
    "industrial_demo_scene": "industrial_demo_scene.xml",
    "open_field": "open_field.xml",
    "spiral": "spiral_terrain.xml",
    "spiral_terrain": "spiral_terrain.xml",
    "stair_showcase": "thunderv4_stair_showcase.xml",
    "thunderv4_stair_showcase": "thunderv4_stair_showcase.xml",
}


@register("driver", "sim_mujoco", description="MuJoCo sim driver (in-process, dimos-style)")
@register("driver_protocol", "mujoco_inproc", description="MuJoCo in-process simulation driver")
class MujocoDriverModule(Module, layer=1):
    """MuJoCo simulation running inside the Module framework.

    Steps physics at sim_rate Hz, publishes sensor data through ports.
    cmd_vel controls the robot via RL policy to joint PD control.
    """

    # -- Inputs --
    cmd_vel: In[Twist]
    stop_signal: In[int]

    # -- Outputs --
    odometry: Out[Odometry]
    lidar_cloud: Out[PointCloud2]
    map_cloud: Out[PointCloud2]
    raw_scan: Out[Any]
    imu: Out[Imu]
    camera_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraIntrinsics]
    height_rays: Out[dict]
    alive: Out[bool]
    robot_state: Out[dict]

    def __init__(
        self,
        world: str = "building_scene",
        render: bool = False,
        enable_camera: bool = False,
        sim_rate: float = 50.0,
        policy_path: str = "",
        robot_xml: str = "",
        drive_mode: str = "",
        max_linear_vel: float | None = None,
        max_angular_vel: float | None = None,
        start_pos: tuple = _DEFAULT_START_POS,
        obstacles: list | None = None,
        odom_frame_id: str = MUJOCO_MODULE_ODOM_FRAME_ID,
        map_cloud_frame_id: str = MUJOCO_MODULE_MAP_CLOUD_FRAME_ID,
        child_frame_id: str = MUJOCO_MODULE_BODY_FRAME_ID,
        lidar_publish_every: int = 5,
        height_ray_publish_every: int = 5,
        enable_height_rays: bool = True,
        **kw,
    ):
        super().__init__(**kw)
        self._world_name = world
        self._render = render
        self._enable_camera = bool(enable_camera or render)
        self._sim_rate = sim_rate
        self._drive_mode = (
            drive_mode or os.environ.get("LINGTU_SIM_DRIVE_MODE", "policy")
        ).strip().lower()
        if self._drive_mode not in {"policy", "kinematic"}:
            raise ValueError(f"Unsupported MuJoCo drive_mode: {self._drive_mode}")
        default_policy = _first_existing_path(_POLICY_CANDIDATES)
        self._policy_path = (
            _resolve_sim_path(policy_path) if policy_path else default_policy
        )
        if self._drive_mode == "kinematic":
            self._policy_path = ""
        self._max_linear_vel = max_linear_vel
        self._max_angular_vel = max_angular_vel
        self._robot_xml = _resolve_sim_path(robot_xml) if robot_xml else str(_ROBOT_XML)
        self._start_pos = start_pos
        self._obstacles = obstacles or []
        self._odom_frame_id = odom_frame_id
        self._map_cloud_frame_id = map_cloud_frame_id
        self._child_frame_id = child_frame_id
        self._lidar_publish_every = max(1, int(lidar_publish_every))
        self._height_ray_publish_every = max(1, int(height_ray_publish_every))
        self._enable_height_rays = bool(enable_height_rays)

        self._engine = None
        self._sim_thread: threading.Thread | None = None
        self._running = False
        self._cmd_lock = threading.Lock()
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._stopped = False
        self._camera_warned = False

    def setup(self) -> None:
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self.stop_signal.subscribe(self._on_stop)

        try:
            if not _numpy_runtime_available():
                logger.error(
                    "MujocoDriverModule: NumPy runtime unavailable; "
                    "simulation engine not loaded"
                )
                self._engine = None
                return

            # Add the repo root to path so `sim.engine` imports work.
            repo_root = str(_SIM_ROOT.parent)
            if repo_root not in sys.path:
                sys.path.insert(0, repo_root)

            from sim.engine.core.robot import RobotConfig
            from sim.engine.core.sensor import CameraConfig, DiscreteRayConfig, LidarConfig
            from sim.engine.core.world import WorldConfig
            from sim.engine.mujoco.engine import MuJoCoEngine

            # Resolve world XML
            world_file = WORLDS.get(self._world_name, self._world_name)
            world_path = _WORLDS_DIR / world_file
            if not world_path.exists():
                logger.error("World not found: %s", world_path)
                return
            if not Path(self._robot_xml).exists():
                logger.error("Robot XML not found: %s", self._robot_xml)
                return

            robot_cfg = RobotConfig.default_thunder_v3()
            robot_cfg.resolve_paths(base_dir=str(_SIM_ROOT))
            robot_cfg.robot_xml = self._robot_xml
            if self._max_linear_vel is not None:
                robot_cfg.max_linear_vel = float(self._max_linear_vel)
            if self._max_angular_vel is not None:
                robot_cfg.max_angular_vel = float(self._max_angular_vel)
            scene_start = _scene_placeholder_start(world_path)
            start_pos = (
                scene_start if scene_start is not None
                and _is_default_start_pos(self._start_pos)
                else list(self._start_pos)
            )
            robot_cfg.init_position = [float(v) for v in start_pos[:3]]
            if self._drive_mode == "kinematic":
                robot_cfg.policy_onnx = ""
            elif self._policy_path:
                robot_cfg.policy_onnx = self._policy_path

            from sim.engine.core.world import ObstacleConfig
            obs_cfgs = []
            for o in self._obstacles:
                if isinstance(o, dict):
                    obs_cfgs.append(ObstacleConfig(**o))
                else:
                    obs_cfgs.append(o)
            world_cfg = WorldConfig(scene_xml=str(world_path), obstacles=obs_cfgs)

            # Camera capture is independent from the on-screen viewer so the
            # semantic stack can run in headless simulation.
            camera_cfgs = []
            if self._enable_camera:
                camera_cfg = CameraConfig(
                    name="front_camera", width=640, height=480,
                    fovy=60.0, render_depth=True)
                camera_cfgs = [camera_cfg]

            self._engine = MuJoCoEngine(
                robot_config=robot_cfg,
                world_config=world_cfg,
                lidar_config=LidarConfig(
                    body_name=robot_cfg.lidar_body_name,
                    geom_group=0,
                ),
                camera_configs=camera_cfgs,
                headless=not self._render,
                drive_mode=self._drive_mode,
                discrete_ray_config=DiscreteRayConfig(),
            )
            # Let MuJoCoEngine decide whether to use the scene directly or merge
            # the selected world geometry into the robot model.
            self._engine.load(str(world_path))
            self._engine.reset()  # stabilize + warm up policy history
            logger.info(
                "MujocoDriverModule: loaded world '%s', robot at %s, drive_mode=%s",
                self._world_name,
                tuple(robot_cfg.init_position),
                self._drive_mode,
            )

        except ImportError as e:
            self._engine = None
            logger.error("MujocoDriverModule: MuJoCo not available: %s", e)
        except Exception as e:
            self._engine = None
            logger.error("MujocoDriverModule: setup failed: %s", e)

    def start(self):
        super().start()
        if self._engine is None:
            self.alive.publish(False)
            return

        self._running = True
        self._sim_thread = threading.Thread(
            target=self._sim_loop, name="mujoco_sim", daemon=True)
        self._sim_thread.start()
        self.alive.publish(True)
        self._publish_robot_state()
        logger.info("MujocoDriverModule: sim loop started at %.0f Hz", self._sim_rate)

    def stop(self):
        self._running = False
        if self._sim_thread:
            self._sim_thread.join(timeout=3.0)
            self._sim_thread = None
        if self._engine:
            self._engine.close()
            self._engine = None
        self.alive.publish(False)
        self._publish_robot_state()
        super().stop()

    def _on_cmd_vel(self, twist: Twist):
        if self._stopped:
            return
        # MuJoCo engine: linear_x = forward, linear_y = lateral (verified by Step 2 test)
        # Nav stack: vx = forward, vy = lateral; same convention, direct passthrough.
        with self._cmd_lock:
            self._cmd_vx = twist.linear.x if hasattr(twist.linear, 'x') else 0.0
            self._cmd_vy = twist.linear.y if hasattr(twist.linear, 'y') else 0.0
            self._cmd_wz = twist.angular.z if hasattr(twist.angular, 'z') else 0.0

    def _on_stop(self, level: int):
        if level >= 1:
            with self._cmd_lock:
                self._cmd_vx = 0.0
                self._cmd_vy = 0.0
                self._cmd_wz = 0.0
            self._stopped = True
            return
        self._stopped = False

    def _sim_loop(self):
        """Step physics and publish sensor data."""
        from sim.engine.core.engine import VelocityCommand

        dt = 1.0 / self._sim_rate
        step_count = 0

        while self._running and self._engine:
            t0 = time.monotonic()

            try:
                # Step physics with current command
                with self._cmd_lock:
                    cmd_vx = self._cmd_vx
                    cmd_vy = self._cmd_vy
                    cmd_wz = self._cmd_wz
                cmd = VelocityCommand(
                    linear_x=cmd_vx, linear_y=cmd_vy, angular_z=cmd_wz)
                state = self._engine.step(cmd)

                # Publish odometry
                ts = time.time()
                quat = state.orientation  # [x, y, z, w]
                self.odometry.publish(Odometry(
                    pose=Pose(
                        position=Vector3(
                            float(state.position[0]),
                            float(state.position[1]),
                            float(state.position[2])),
                        orientation=Quaternion(
                            float(quat[0]), float(quat[1]),
                            float(quat[2]), float(quat[3]))),
                    twist=Twist(
                        linear=Vector3(
                            float(state.linear_velocity[0]),
                            float(state.linear_velocity[1]),
                            float(state.linear_velocity[2])),
                        angular=Vector3(
                            float(state.angular_velocity[0]),
                            float(state.angular_velocity[1]),
                            float(state.angular_velocity[2]))),
                    ts=ts,
                    frame_id=self._odom_frame_id,
                    child_frame_id=self._child_frame_id,
                ))

                gyro = np.asarray(getattr(state, "imu_gyro", (0.0, 0.0, 0.0)), dtype=float)
                accel = np.asarray(
                    getattr(state, "imu_linear_acceleration", (0.0, 0.0, 0.0)),
                    dtype=float,
                )
                self.imu.publish(Imu(
                    orientation=Quaternion(
                        float(quat[0]), float(quat[1]),
                        float(quat[2]), float(quat[3])),
                    angular_velocity=Vector3(
                        float(gyro[0]), float(gyro[1]), float(gyro[2])),
                    linear_acceleration=Vector3(
                        float(accel[0]), float(accel[1]), float(accel[2])),
                    ts=ts,
                    frame_id=MUJOCO_MODULE_IMU_FRAME_ID,
                ))

                # Publish LiDAR (default every 5th step = ~10 Hz). Nav-only
                # gates can raise this interval after the first scan so heavy
                # raycasting does not starve the motion loop.
                if step_count % self._lidar_publish_every == 0:
                    try:
                        pts = self._engine.get_lidar_points()
                        if pts is not None and len(pts) > 0:
                            pts_world = pts.astype(np.float32)
                            pts_body = _world_points_to_body_frame(
                                pts_world,
                                state.position,
                                state.orientation,
                            )
                            pts_lidar = _world_points_to_lidar_frame(
                                pts_world,
                                self._engine,
                                state.position,
                                state.orientation,
                            )
                            scan_duration_s = (
                                self._lidar_publish_every
                                / max(float(self._sim_rate), 1e-6)
                            )
                            self.raw_scan.publish(_xyzi_to_livox_frame(
                                pts_lidar,
                                timestamp_ns=int(max(0.0, ts - scan_duration_s) * 1_000_000_000),
                                sequence=step_count // self._lidar_publish_every,
                                frame_id=MUJOCO_MODULE_LIDAR_FRAME_ID,
                                scan_duration_ns=int(1_000_000_000 * scan_duration_s),
                            ))
                            self.lidar_cloud.publish(PointCloud2(
                                points=pts_body,
                                frame_id=MUJOCO_MODULE_BODY_FRAME_ID,
                                ts=ts,
                            ))
                            world_cloud = PointCloud2(
                                points=pts_world,
                                frame_id=self._map_cloud_frame_id,
                                ts=ts,
                            )
                            self.map_cloud.publish(world_cloud)
                    except Exception:
                        logger.debug("mujoco_driver: cloud publish error", exc_info=True)

                # Publish fixed-pattern terrain rays for policy/terrain evidence.
                if self._enable_height_rays and step_count % self._height_ray_publish_every == 0:
                    try:
                        rays = self._engine.get_discrete_rays()
                        self.height_rays.publish({
                            "pattern": rays.pattern,
                            "heights": rays.heights,
                            "points_body": rays.points_body,
                            "points_world": rays.points_world,
                            "valid_mask": rays.valid_mask,
                            "metadata": rays.metadata,
                            "ts": ts,
                            "frame_id": MUJOCO_MODULE_BODY_FRAME_ID,
                        })
                    except Exception:
                        logger.debug("mujoco_driver: height-ray publish error", exc_info=True)

                # Publish camera (every 10th step = ~5 Hz)
                if step_count % 10 == 0:
                    try:
                        cam = self._engine.get_camera_data("front_camera")
                        if cam is not None:
                            h, w = (cam.rgb.shape[:2] if cam.rgb is not None
                                    else cam.depth.shape[:2])
                            self.camera_image.publish(Image(
                                data=cam.rgb,
                                format=ImageFormat.RGB,
                                ts=ts,
                                frame_id=MUJOCO_MODULE_CAMERA_FRAME_ID,
                            ))
                            if cam.depth is not None:
                                self.depth_image.publish(Image(
                                    data=cam.depth,
                                    format=ImageFormat.DEPTH_F32,
                                    ts=ts,
                                    frame_id=MUJOCO_MODULE_CAMERA_FRAME_ID,
                                ))
                            fx, fy, cx, cy = cam.intrinsics
                            self.camera_info.publish(CameraIntrinsics(
                                fx=float(fx),
                                fy=float(fy),
                                cx=float(cx),
                                cy=float(cy),
                                width=int(w),
                                height=int(h),
                                depth_scale=1.0,
                            ))
                    except Exception as e:
                        if not self._camera_warned:
                            logger.warning("MujocoDriverModule: camera publish failed: %s", e)
                            self._camera_warned = True

                step_count += 1

            except Exception as e:
                logger.error("MujocoDriverModule: sim step error: %s", e)
                break

            # Rate control
            elapsed = time.monotonic() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info("MujocoDriverModule: sim loop ended after %d steps", step_count)

    # -- Robot state --------------------------------------------------

    def _publish_robot_state(self) -> None:
        """Publish sim operational state."""
        from drivers.sim import build_sim_robot_state
        self.robot_state.publish(build_sim_robot_state())

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["mujoco"] = {
            "world": self._world_name,
            "running": self._running,
            "has_engine": self._engine is not None,
            "sim_rate": self._sim_rate,
            "render": self._render,
            "drive_mode": self._drive_mode,
            "lidar_publish_every": self._lidar_publish_every,
            "height_ray_publish_every": self._height_ray_publish_every,
            "height_rays_enabled": self._enable_height_rays,
        }
        return info
