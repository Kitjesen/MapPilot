"""MujocoDriverModule — MuJoCo simulation as a Module (dimos-style).

Wraps sim/engine/mujoco/engine.py directly in-process.
No TCP bridge, no separate process. Data flows through In/Out ports.

Provides: odometry, lidar_cloud, camera_image, depth_image, imu
Consumes: cmd_vel, stop_signal

Usage::

    from drivers.sim.mujoco_driver_module import MujocoDriverModule
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

from core.module import Module
from core.msgs.numpy_compat import np
from core.msgs.geometry import Pose, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from core.msgs.sensor import CameraIntrinsics, Image, ImageFormat, PointCloud2
from core.registry import register
from core.runtime_interface import FRAMES, TOPICS, topic_default_frame_id
from core.stream import In, Out

logger = logging.getLogger(__name__)

# Resolve sim/ directory relative to this file
_SIM_ROOT = Path(__file__).resolve().parents[3] / "sim"
_WORLDS_DIR = _SIM_ROOT / "worlds" / "mujoco"
_THUNDER_MJCF = _SIM_ROOT / "assets" / "mjcf" / "thunder_v3_lingtu.xml"
_LEGACY_ROBOTS_DIR = _SIM_ROOT / "robots" / "nova_dog"
_ROBOT_XML = _THUNDER_MJCF
_BRAINSTEM_POLICY_NAME = "policy_251119.onnx"
_POLICY_CANDIDATES = (
    _LEGACY_ROBOTS_DIR / "model" / _BRAINSTEM_POLICY_NAME,
    _SIM_ROOT.parent / "model" / _BRAINSTEM_POLICY_NAME,
    _SIM_ROOT.parent.parent / "brainstem" / "model" / _BRAINSTEM_POLICY_NAME,
    _SIM_ROOT.parent.parent / "brainstem" / "han_dog" / "model" / _BRAINSTEM_POLICY_NAME,
    _SIM_ROOT.parent.parent / "brainstem" / "sim" / "model" / _BRAINSTEM_POLICY_NAME,
    _LEGACY_ROBOTS_DIR / "policy.onnx",
    # Legacy 76-D policy kept last: it needs its original observation contract.
    _LEGACY_ROBOTS_DIR / "thunder_policy.onnx",
)
_POLICY_ONNX = _POLICY_CANDIDATES[-1]
_DEFAULT_START_POS = (0.0, 0.0, 0.55)
MUJOCO_MODULE_ODOM_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
MUJOCO_MODULE_BODY_FRAME_ID = topic_default_frame_id(TOPICS.registered_cloud)
# In-process MuJoCo does not own a map->odom localizer; its live map cloud is odom-frame.
MUJOCO_MODULE_MAP_CLOUD_FRAME_ID = MUJOCO_MODULE_ODOM_FRAME_ID
MUJOCO_MODULE_CAMERA_FRAME_ID = FRAMES.camera
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


# Known worlds
WORLDS = {
    "building": "building_scene.xml",
    "building_scene": "building_scene.xml",
    "factory": "factory_scene.xml",
    "factory_scene": "factory_scene.xml",
    "industrial_park": "industrial_park_scene.xml",
    "industrial_park_scene": "industrial_park_scene.xml",
    "industrial_demo": "industrial_demo_scene.xml",
    "industrial_demo_scene": "industrial_demo_scene.xml",
    "open_field": "open_field.xml",
    "spiral": "spiral_terrain.xml",
    "spiral_terrain": "spiral_terrain.xml",
}


@register("driver", "sim_mujoco", description="MuJoCo sim driver (in-process, dimos-style)")
@register("driver_protocol", "mujoco_inproc", description="MuJoCo in-process simulation driver")
class MujocoDriverModule(Module, layer=1):
    """MuJoCo simulation running inside the Module framework.

    Steps physics at sim_rate Hz, publishes sensor data through ports.
    cmd_vel controls the robot via RL policy → joint PD control.
    """

    # -- Inputs --
    cmd_vel: In[Twist]
    stop_signal: In[int]

    # -- Outputs --
    odometry: Out[Odometry]
    lidar_cloud: Out[PointCloud2]
    map_cloud: Out[PointCloud2]
    camera_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraIntrinsics]
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
        child_frame_id: str = MUJOCO_MODULE_BODY_FRAME_ID,
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
        self._child_frame_id = child_frame_id

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
            from sim.engine.core.sensor import CameraConfig, LidarConfig
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
            if self._drive_mode == "policy" and self._policy_path:
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
        # Nav stack: vx = forward, vy = lateral — same convention, direct passthrough
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
        """Main simulation loop — steps physics and publishes sensor data."""
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

                # Publish LiDAR (every 5th step = ~10 Hz)
                if step_count % 5 == 0:
                    try:
                        pts = self._engine.get_lidar_points()
                        if pts is not None and len(pts) > 0:
                            pts_world = pts.astype(np.float32)
                            pts_body = _world_points_to_body_frame(
                                pts_world,
                                state.position,
                                state.orientation,
                            )
                            self.lidar_cloud.publish(PointCloud2(
                                points=pts_body,
                                frame_id=MUJOCO_MODULE_BODY_FRAME_ID,
                                ts=ts,
                            ))
                            world_cloud = PointCloud2(
                                points=pts_world,
                                frame_id=MUJOCO_MODULE_MAP_CLOUD_FRAME_ID,
                                ts=ts,
                            )
                            self.map_cloud.publish(world_cloud)
                    except Exception:
                        logger.debug("mujoco_driver: cloud publish error", exc_info=True)

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
        }
        return info
