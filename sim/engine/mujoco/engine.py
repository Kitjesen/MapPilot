"""MuJoCo simulation engine implementation
# Extracted from src/drivers/sim/nova_nav_bridge.py

Core logic sourced from nova_nav_bridge.py:
  - MuJoCo model loading and scene XML generation
  - Physics stepping loop (mujoco.mj_step)
  - cmd_vel -> ONNX policy -> joint control
  - IMU / joint state reading (get_imu, get_joint_state)
  - LiDAR scan wrapper
"""

import math
import os
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from sim.engine.core.engine import (
    CameraData,
    DiscreteRayData,
    RobotState,
    SimEngine,
    VelocityCommand,
)
from sim.engine.core.robot import THUNDER_V3_JOINT_NAMES, RobotConfig
from sim.engine.core.sensor import CameraConfig, DiscreteRayConfig, IMUConfig, LidarConfig
from sim.engine.core.world import SimWorld, WorldConfig
from sim.engine.mujoco.camera import MuJoCoCamera
from sim.engine.mujoco.lidar import MuJoCoLidar
from sim.engine.mujoco.robot_controller import (
    DART_TO_MJ,
    MJ_TO_DART,
    STANDING_POSE,
    PolicyRunner,
    load_policy_runner,
)

DEFAULT_LEG_JOINT_NAMES = THUNDER_V3_JOINT_NAMES


def _freeze_scene_euler_orientations(scene_root: Any, mujoco_module: Any) -> None:
    """Preserve scene-local Euler semantics before merging MJCF compiler scopes."""

    compiler = scene_root.find("compiler")
    angle_unit = str(compiler.attrib.get("angle", "degree") if compiler is not None else "degree").strip().lower()
    euler_sequence = str(compiler.attrib.get("eulerseq", "xyz") if compiler is not None else "xyz").strip()
    worldbody = scene_root.find("worldbody")
    if worldbody is None:
        return

    for element in worldbody.iter():
        encoded = element.attrib.get("euler")
        if encoded is None:
            continue
        values = np.fromstring(encoded, sep=" ", dtype=np.float64)
        if values.size != 3 or not np.isfinite(values).all():
            name = element.attrib.get("name", "")
            raise ValueError(f"invalid MJCF Euler orientation on {element.tag} {name!r}: {encoded!r}")
        if angle_unit.startswith("deg"):
            values = np.deg2rad(values)
        elif not angle_unit.startswith("rad"):
            raise ValueError(f"unsupported MJCF compiler angle unit: {angle_unit!r}")

        quaternion = np.zeros(4, dtype=np.float64)
        mujoco_module.mju_euler2Quat(quaternion, values, euler_sequence)
        element.attrib["quat"] = " ".join(f"{value:.17g}" for value in quaternion)
        del element.attrib["euler"]


class MuJoCoEngine(SimEngine):
    """MuJoCo simulation engine.

    Implements the SimEngine abstract interface, wrapping:
      - Scene loading (WorldConfig + RobotConfig)
      - Physics stepping (500Hz physics, 50Hz policy)
      - ONNX gait policy (PolicyRunner)
      - LiDAR scanning (MuJoCoLidar)
      - Camera rendering (MuJoCoCamera)
    """

    def __init__(
        self,
        robot_config: RobotConfig | None = None,
        world_config: WorldConfig | None = None,
        lidar_config: LidarConfig | None = None,
        camera_configs: list[CameraConfig] | None = None,
        imu_config: IMUConfig | None = None,
        headless: bool = True,
        drive_mode: str = "policy",
        discrete_ray_config: DiscreteRayConfig | None = None,
    ) -> None:
        """Initialize MuJoCo engine (model not loaded; call load() before use).

        Args:
            robot_config: robot configuration, defaults to Thunder v3
            world_config: scene configuration, defaults to flat ground
            lidar_config: LiDAR configuration, defaults to MID-360
            camera_configs: list of camera configurations
            imu_config: IMU configuration
            headless: True=headless mode, False=launch MuJoCo viewer
            drive_mode: "policy" for ONNX gait control, "kinematic" for
                headless navigation validation that applies cmd_vel directly
                to the floating base.
        """
        super().__init__()

        self._robot_cfg = robot_config or RobotConfig.default_nova_dog()
        self._world_cfg = world_config or WorldConfig()
        self._lidar_cfg = lidar_config or LidarConfig()
        self._camera_cfgs = camera_configs or []
        self._imu_cfg = imu_config or IMUConfig()
        self._discrete_ray_cfg = discrete_ray_config or DiscreteRayConfig()
        self._headless = headless
        self._drive_mode = (drive_mode or "policy").strip().lower()
        if self._drive_mode not in {"policy", "kinematic"}:
            raise ValueError(f"Unsupported MuJoCo drive_mode: {drive_mode}")

        # MuJoCo core objects (initialized after load())
        self._model = None
        self._data = None

        # Sensors
        self._lidar: MuJoCoLidar | None = None
        self._cameras: dict[str, MuJoCoCamera] = {}

        # Policy controller
        self._policy: Any | None = None
        self._policy_path: str = ""
        self._policy_idle_hold = False
        self._policy_idle_cmd_eps = 1e-4

        # Joint ID lists (populated after load())
        self._leg_joint_ids: list[int] = []
        self._leg_actuator_ids: list[int] = []
        self._base_body_id: int = 0
        self._lidar_body_id: int = 0
        self._leg_actuator_offset: int = 0
        self._leg_control_mode: str = "position"
        self._last_leg_targets_mj: np.ndarray | None = None
        self._root_qposadr: int = 0
        self._root_dofadr: int = 0

        # cmd_vel (thread-safe)
        self._cmd_vel = np.zeros(3, dtype=np.float64)  # [vx, vy, wz]
        self._cmd_vel_time = 0.0
        self._lock = threading.Lock()
        self._data_lock = threading.RLock()

        # Physics parameters
        self._physics_dt: float = 0.002  # read from model.opt.timestep
        policy_hz = max(float(getattr(self._robot_cfg, "policy_freq_hz", 50.0) or 50.0), 1.0)
        self._control_dt: float = 1.0 / policy_hz
        self._last_policy_update_sim_time: float = -self._control_dt
        self._sensor_tick_residual_s: float = 0.0

        # Background physics thread
        self._stop_event = threading.Event()
        self._sim_thread: threading.Thread | None = None

    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    # Lifecycle
    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def load(self, xml_path: str = "", **kwargs: Any) -> None:
        """Load scene and initialize all subsystems.

        Args:
            xml_path: scene XML path. If empty, generates from WorldConfig.
            **kwargs: extra parameters (ignored, for interface compatibility)
        """
        import mujoco

        # Resolve robot XML path
        self._robot_cfg.resolve_paths()
        robot_xml = self._robot_cfg.robot_xml

        if xml_path:
            # Check whether XML contains a ground plane (geom in worldbody)
            # If it is a bare robot.xml (no ground), auto-wrap with a scene
            _xml_content = Path(xml_path).read_text(encoding="utf-8", errors="ignore")
            # Only consider it grounded if it explicitly contains a floor/plane
            _has_floor = (
                "floor" in _xml_content.lower() or 'name="ground"' in _xml_content or 'type="plane"' in _xml_content
            )
            _has_actuators = "<actuator>" in _xml_content
            if _has_floor and _has_actuators:
                # Scene already has robot (actuators present) 鈥?use as-is
                self._model = mujoco.MjModel.from_xml_path(xml_path)
            elif _has_floor and not _has_actuators:
                # Scene has floor but no robot 鈥?merge robot into scene using
                # structured XML editing so nested placeholder bodies are
                # removed safely.
                import copy
                import xml.etree.ElementTree as ET

                scene_root = ET.fromstring(_xml_content)
                robot_root = ET.fromstring(Path(robot_xml).read_text(encoding="utf-8", errors="ignore"))

                scene_worldbody = scene_root.find("worldbody")
                robot_worldbody = robot_root.find("worldbody")
                if scene_worldbody is None or robot_worldbody is None:
                    self._model = mujoco.MjModel.from_xml_path(xml_path)
                else:
                    # Scene and robot files may declare different compiler
                    # eulerseq values. Once scene nodes are copied under the
                    # robot root, raw Euler triples would silently inherit the
                    # robot convention and describe different geometry.
                    _freeze_scene_euler_orientations(scene_root, mujoco)
                    scene_asset = scene_root.find("asset")
                    robot_asset = robot_root.find("asset")
                    scene_size = scene_root.find("size")
                    scene_visual = scene_root.find("visual")
                    if scene_size is not None and robot_root.find("size") is None:
                        insert_at = 0
                        for idx, child in enumerate(list(robot_root)):
                            if child.tag == "compiler":
                                insert_at = idx + 1
                                break
                        robot_root.insert(insert_at, copy.deepcopy(scene_size))

                    if scene_asset is not None and len(scene_asset) > 0:
                        if robot_asset is None:
                            robot_asset = ET.Element("asset")
                            robot_root.append(robot_asset)
                        existing_asset_keys = {(child.tag, child.attrib.get("name")) for child in list(robot_asset)}
                        for child in list(scene_asset):
                            asset_key = (child.tag, child.attrib.get("name"))
                            if asset_key in existing_asset_keys:
                                continue
                            robot_asset.append(copy.deepcopy(child))
                            existing_asset_keys.add(asset_key)

                    if scene_visual is not None:
                        robot_visual = robot_root.find("visual")
                        if robot_visual is not None:
                            visual_index = list(robot_root).index(robot_visual)
                            robot_root.remove(robot_visual)
                            robot_root.insert(visual_index, copy.deepcopy(scene_visual))
                        else:
                            robot_root.insert(0, copy.deepcopy(scene_visual))

                    for child in list(robot_worldbody):
                        if child.tag != "geom":
                            continue
                        geom_name = child.attrib.get("name", "")
                        geom_type = child.attrib.get("type", "")
                        if geom_name in {"ground", "floor"} or geom_type == "plane":
                            robot_worldbody.remove(child)

                    for child in list(scene_worldbody):
                        if child.tag == "body" and child.attrib.get("name") in {"robot_placeholder", "base_link"}:
                            continue
                        robot_worldbody.append(copy.deepcopy(child))

                    _merged = ET.tostring(robot_root, encoding="unicode")
                    _robot_dir = str(Path(robot_xml).parent)
                    _tmp = tempfile.NamedTemporaryFile(
                        suffix=".xml", delete=False, dir=_robot_dir, mode="w", encoding="utf-8"
                    )
                    _tmp.write(_merged)
                    _tmp.close()
                    try:
                        self._model = mujoco.MjModel.from_xml_path(_tmp.name)
                    finally:
                        Path(_tmp.name).unlink(missing_ok=True)
            else:
                # Auto-inject ground: insert floor geom after <worldbody>
                _robot_dir = str(Path(xml_path).parent)
                _floor_geom = (
                    '    <geom name="floor" type="plane" size="50 50 0.1"'
                    ' conaffinity="1" condim="3" friction="1 0.5 0.5"/>\n'
                )
                _patched = _xml_content.replace(
                    "<worldbody>",
                    "<worldbody>\n" + _floor_geom,
                    1,  # only replace the first occurrence
                )
                _tmp = tempfile.NamedTemporaryFile(
                    suffix=".xml", delete=False, dir=_robot_dir, mode="w", encoding="utf-8"
                )
                _tmp.write(_patched)
                _tmp.close()
                try:
                    self._model = mujoco.MjModel.from_xml_path(_tmp.name)
                finally:
                    Path(_tmp.name).unlink(missing_ok=True)
        elif self._world_cfg.scene_xml and Path(self._world_cfg.scene_xml).exists():
            self._model = mujoco.MjModel.from_xml_path(self._world_cfg.scene_xml)
        else:
            # Generate scene XML dynamically from WorldConfig
            world = SimWorld(self._world_cfg)
            # MuJoCo is sensitive to Windows-style include paths. Use a normalized
            # absolute POSIX path inside the generated XML so headless startup
            # resolves robot.xml consistently.
            scene_xml_str = world.get_scene_xml(Path(robot_xml).resolve().as_posix())
            # Write to temp file (MuJoCo needs file path to handle <include>)
            robot_dir = str(Path(robot_xml).parent)
            tmp = tempfile.NamedTemporaryFile(suffix=".xml", delete=False, dir=robot_dir, mode="w", encoding="utf-8")
            tmp.write(scene_xml_str)
            tmp.close()
            try:
                self._model = mujoco.MjModel.from_xml_path(tmp.name)
            finally:
                Path(tmp.name).unlink(missing_ok=True)

        self._data = mujoco.MjData(self._model)
        self._physics_dt = float(self._model.opt.timestep)

        # Resolve body/joint IDs
        self._resolve_ids()
        valid_leg_joints = len([jid for jid in self._leg_joint_ids if jid >= 0])
        valid_leg_actuators = len([aid for aid in self._leg_actuator_ids if aid >= 0])
        if valid_leg_joints > 0 and valid_leg_actuators == 0 and self._model is not None:
            self._leg_actuator_offset = max(0, int(self._model.nu) - valid_leg_joints)
        else:
            self._leg_actuator_offset = self._robot_cfg.leg_act_offset
        self._leg_control_mode = self._resolve_leg_control_mode()

        # Initialize LiDAR
        self._lidar = MuJoCoLidar(self._model, self._data, self._lidar_cfg)

        # Initialize cameras
        for cam_cfg in self._camera_cfgs:
            try:
                self._cameras[cam_cfg.name] = MuJoCoCamera(self._model, cam_cfg)
            except Exception as e:
                print(f"[MuJoCoEngine] Warning: {e}")

        # Initialize policy. Kinematic mode intentionally bypasses the gait
        # policy so route-level navigation tests are not gated by RL checkpoints.
        policy_path = self._robot_cfg.policy_onnx
        self._policy_path = str(policy_path or "")
        if self._drive_mode == "kinematic":
            print("[MuJoCoEngine] Kinematic drive mode enabled, policy disabled")
            self._policy = None
        elif policy_path:
            if Path(policy_path).exists():
                self._policy = load_policy_runner(policy_path)
                wheel_limit = getattr(self._policy, "wheel_torque_limit", None)
                if wheel_limit is not None:
                    self._robot_cfg.torque_limit[12:] = [float(wheel_limit)] * 4
            else:
                print(f"[MuJoCoEngine] Policy not found at {policy_path}, running PD-only mode")
                self._policy = None
        else:
            print("[MuJoCoEngine] No policy configured, running PD-only mode")
            self._policy = None

        self._running = True
        print(
            f"[MuJoCoEngine] Loaded. dt={self._physics_dt:.4f}s, "
            f"joints={valid_leg_joints}, "
            f"ctrl_offset={self._leg_actuator_offset}, "
            f"leg_ctrl={self._leg_control_mode}, "
            f"drive_mode={self._drive_mode}, "
            f"cameras={list(self._cameras.keys())}"
        )

    def _resolve_ids(self) -> None:
        """Resolve body/joint IDs in MuJoCo.
        # Extracted from src/drivers/sim/nova_nav_bridge.py NavBridge.__init__
        """
        import mujoco

        self._base_body_id, resolved_base = self._resolve_body_id(
            self._robot_cfg.base_body_name,
            aliases=["trunk", "base_link", "robot_placeholder"],
        )
        self._robot_cfg.base_body_name = resolved_base

        self._lidar_body_id, resolved_lidar = self._resolve_body_id(
            self._lidar_cfg.body_name,
            aliases=[self._robot_cfg.base_body_name, "trunk", "lidar_link"],
        )
        self._lidar_cfg.body_name = resolved_lidar
        self._resolve_root_joint_adrs()
        leg_joint_names = self._robot_cfg.leg_joint_names or DEFAULT_LEG_JOINT_NAMES
        self._leg_joint_ids = [self._resolve_joint_id(name) for name in leg_joint_names]
        self._leg_actuator_ids = self._resolve_leg_actuator_ids(self._leg_joint_ids)
        missing = [n for n, jid in zip(leg_joint_names, self._leg_joint_ids) if jid < 0]
        if missing:
            print(f"[MuJoCoEngine] Warning: joints not found: {missing}")
        missing_actuators = [
            n
            for n, jid, aid in zip(leg_joint_names, self._leg_joint_ids, self._leg_actuator_ids)
            if jid >= 0 and aid < 0
        ]
        if missing_actuators:
            print(f"[MuJoCoEngine] Warning: actuators not found: {missing_actuators}")

    def _resolve_root_joint_adrs(self) -> None:
        """Resolve floating-base qpos/qvel addresses for kinematic drive."""
        import mujoco

        self._root_qposadr = 0
        self._root_dofadr = 0
        if self._model is None:
            return

        for jid in range(int(self._model.njnt)):
            if int(self._model.jnt_type[jid]) == int(mujoco.mjtJoint.mjJNT_FREE):
                self._root_qposadr = int(self._model.jnt_qposadr[jid])
                self._root_dofadr = int(self._model.jnt_dofadr[jid])
                return

    def _resolve_body_id(self, preferred_name: str, aliases: list[str]) -> tuple[int, str]:
        """Resolve a MuJoCo body name with a few safe fallbacks."""
        import mujoco

        candidates = []
        for name in [preferred_name, *aliases]:
            if name and name not in candidates:
                candidates.append(name)

        for name in candidates:
            body_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id >= 0:
                if name != preferred_name:
                    print(f'[MuJoCoEngine] Body "{preferred_name}" not found, using "{name}" instead')
                return body_id, name

        print(f"[MuJoCoEngine] Warning: bodies not found for {candidates}, falling back to world body")
        return 0, preferred_name or "world"

    def _resolve_joint_id(self, preferred_name: str) -> int:
        """Resolve a joint name, accepting legacy lower-case Thunder aliases."""
        import mujoco

        for name in self._joint_name_candidates(preferred_name):
            joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id >= 0:
                return joint_id
        return -1

    @staticmethod
    def _joint_name_candidates(name: str) -> list[str]:
        candidates = []
        for candidate in (
            name,
            MuJoCoEngine._legacy_joint_name(name),
            MuJoCoEngine._thunder_joint_name(name),
        ):
            if candidate and candidate not in candidates:
                candidates.append(candidate)
        return candidates

    @staticmethod
    def _legacy_joint_name(name: str) -> str:
        parts = name.split("_", 1)
        if len(parts) != 2:
            return name.lower()
        return f"{parts[0].lower()}_{parts[1]}"

    @staticmethod
    def _thunder_joint_name(name: str) -> str:
        parts = name.split("_", 1)
        if len(parts) != 2:
            return name
        prefix = parts[0].upper()
        if prefix in {"FR", "FL", "RR", "RL"}:
            return f"{prefix}_{parts[1]}"
        return name

    def _resolve_leg_actuator_ids(self, joint_ids: list[int]) -> list[int]:
        """Resolve control slots by joint ID so actuator XML order can vary."""
        actuator_ids: list[int] = []
        if self._model is None:
            return [-1 for _ in joint_ids]

        for joint_id in joint_ids:
            actuator_id = -1
            if joint_id >= 0:
                for idx in range(int(self._model.nu)):
                    try:
                        if int(self._model.actuator_trnid[idx, 0]) == joint_id:
                            actuator_id = idx
                            break
                    except Exception:
                        break
            actuator_ids.append(actuator_id)
        return actuator_ids

    def _resolve_leg_control_mode(self) -> str:
        requested = getattr(self._robot_cfg, "leg_control_mode", "auto")
        requested = (requested or "auto").strip().lower()
        if requested in {"position", "torque"}:
            return requested
        if requested != "auto":
            raise ValueError(f"Unsupported leg_control_mode: {requested}")

        valid_actuators = [idx for idx in self._leg_actuator_ids if idx >= 0]
        if not valid_actuators or self._model is None:
            return "position"

        # MuJoCo <motor> actuators have direct gain and no position/velocity
        # bias term; position actuators have non-zero bias parameters.
        for actuator_id in valid_actuators:
            gain = float(self._model.actuator_gainprm[actuator_id, 0])
            bias = np.asarray(self._model.actuator_biasprm[actuator_id, :3], dtype=np.float64)
            if not np.isclose(gain, 1.0) or not np.allclose(bias, 0.0):
                return "position"
        return "torque"

    def reset(self) -> RobotState:
        """Reset simulation to initial state."""
        import mujoco

        mujoco.mj_resetData(self._model, self._data)

        # Set initial position
        pos = self._robot_cfg.init_position
        q_wxyz = self._robot_cfg.init_orientation_wxyz
        self._data.qpos[0:3] = pos
        self._data.qpos[3] = q_wxyz[0]  # w
        self._data.qpos[4] = q_wxyz[1]  # x
        self._data.qpos[5] = q_wxyz[2]  # y
        self._data.qpos[6] = q_wxyz[3]  # z

        # Set initial standing joint angles
        self._apply_standing_pose()

        # Zero arm joints ctrl (required by original bridge)
        offset = self._leg_actuator_offset
        for i in range(offset):
            if i < len(self._data.ctrl):
                self._data.ctrl[i] = 0.0

        mujoco.mj_forward(self._model, self._data)

        # Stabilization phase: hold the standing pose before enabling policy.
        # (policy outputs bad actions from initial state and would fling the robot)
        _policy_backup = self._policy
        self._policy = None  # temporarily disable policy
        warmup_seconds = 0.1 if self._leg_control_mode == "torque" else 2.0
        warmup_steps = max(1, round(warmup_seconds / max(self._physics_dt, 1e-6)))
        standing_mj = self._robot_cfg.standing_pose_array[DART_TO_MJ]
        for _ in range(warmup_steps):
            self._write_leg_ctrl(standing_mj)
            mujoco.mj_step(self._model, self._data)
        self._policy = _policy_backup  # restore policy

        self._sim_time = 0.0
        self._last_policy_update_sim_time = -self._control_dt
        self._sensor_tick_residual_s = 0.0

        # Reset policy history (use stabilized real sensor data)
        if self._policy is not None:
            self._policy.reset()
            gyro, pg = self._get_imu()
            jp, jv = self._get_joint_state()
            self._policy.warm_up(gyro, pg, jp, jv)
            self._policy_idle_hold = False

        # Update LiDAR data reference
        if self._lidar is not None:
            self._lidar.update_data(self._data)

        with self._lock:
            self._cmd_vel[:] = 0.0
            self._cmd_vel_time = 0.0

        return self.get_robot_state()

    def _apply_standing_pose(self) -> None:
        """Write standing pose to joint ctrl (used during initialization)."""
        if not self._leg_joint_ids:
            return
        import mujoco

        standing_dart = self._robot_cfg.standing_pose_array
        standing_mj = standing_dart[self._robot_cfg.dart_to_mj_array]

        for i, jid in enumerate(self._leg_joint_ids):
            if jid < 0:
                continue
            qadr = self._model.jnt_qposadr[jid]
            self._data.qpos[qadr] = standing_mj[i]
        self._write_leg_ctrl(standing_mj)

    def close(self) -> None:
        """Release all resources."""
        self._running = False
        self._stop_event.set()
        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=2.0)
        for cam in self._cameras.values():
            cam.close()
        self._cameras.clear()
        print("[MuJoCoEngine] Closed.")

    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    # Simulation stepping
    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def step(self, cmd: VelocityCommand | None = None) -> RobotState:
        """Advance one control cycle (policy_dt = 0.02s = 10 physics steps).

        Args:
            cmd: velocity command; if given, updates internal cmd_vel.

        Returns:
            robot state after stepping
        """
        import mujoco

        if cmd is not None:
            self.set_cmd_vel(cmd)

        with self._data_lock:
            # Watchdog: zero out cmd_vel if no command within 200ms
            self._watchdog_cmd_vel()

            # Run policy inference, write to ctrl
            self._step_policy()
            self._last_policy_update_sim_time = self._sim_time

            # Physics stepping (policy_dt / physics_dt steps)
            n_sub = max(1, round(self._control_dt / self._physics_dt))
            if self._drive_mode == "kinematic":
                for _ in range(n_sub):
                    self._apply_kinematic_cmd(integrate_dt=self._physics_dt)
                    mj_forward = getattr(mujoco, "mj_forward", None)
                    if callable(mj_forward):
                        mj_forward(self._model, self._data)
            else:
                for _ in range(n_sub):
                    self._refresh_leg_torque_ctrl()
                    mujoco.mj_step(self._model, self._data)
            self._sim_time += self._control_dt

        return self.get_robot_state()

    def step_sensor_tick(
        self,
        cmd: VelocityCommand | None = None,
        dt_s: float | None = None,
    ) -> RobotState:
        """Advance one simulated sensor sample while keeping policy at control rate."""
        import mujoco

        if cmd is not None:
            self.set_cmd_vel(cmd)

        target_dt = max(float(dt_s or self._physics_dt), self._physics_dt)
        desired_steps = (target_dt + self._sensor_tick_residual_s) / self._physics_dt
        n_sub = max(1, int(math.floor(desired_steps + 0.5)))
        actual_dt = n_sub * self._physics_dt
        self._sensor_tick_residual_s += target_dt - actual_dt

        with self._data_lock:
            for _ in range(n_sub):
                if self._sim_time - self._last_policy_update_sim_time >= self._control_dt - 1e-9:
                    self._watchdog_cmd_vel()
                    self._step_policy()
                    self._last_policy_update_sim_time = self._sim_time

                if self._drive_mode == "kinematic":
                    self._apply_kinematic_cmd(integrate_dt=self._physics_dt)
                    mj_forward = getattr(mujoco, "mj_forward", None)
                    if callable(mj_forward):
                        mj_forward(self._model, self._data)
                else:
                    self._refresh_leg_torque_ctrl()
                    mujoco.mj_step(self._model, self._data)
                self._sim_time += self._physics_dt

        return self.get_robot_state()

    def step_static_sensor_tick(self, dt_s: float | None = None) -> RobotState:
        """Advance sensor time without integrating robot dynamics.

        This is used while a navigation run is waiting for localization and a
        global plan. It avoids the artificial acceleration impulses caused by
        stepping gravity and teleporting the free joint back every IMU tick.
        """
        import mujoco

        target_dt = max(float(dt_s or self._physics_dt), self._physics_dt)
        desired_steps = (target_dt + self._sensor_tick_residual_s) / self._physics_dt
        n_sub = max(1, int(math.floor(desired_steps + 0.5)))
        actual_dt = n_sub * self._physics_dt
        self._sensor_tick_residual_s += target_dt - actual_dt

        with self._data_lock:
            self._data.qvel[:] = 0.0
            if hasattr(self._data, "qacc"):
                self._data.qacc[:] = 0.0
            self._sim_time += actual_dt
            self._data.time = self._sim_time
            mujoco.mj_forward(self._model, self._data)

        return self.get_robot_state()

    def advance_static_sensor_clock(self, dt_s: float | None = None) -> float:
        """Advance an anchored sensor clock without recomputing MuJoCo state.

        This is the fast path for an intentionally dropped sensor observation.
        The caller guarantees that the robot pose is anchored, so neither
        dynamics nor derived sensor values need to be evaluated for this tick.
        The next published static tick uses :meth:`step_static_sensor_tick`,
        which runs ``mj_forward`` before any state or sensor sample is read.
        """

        target_dt = max(float(dt_s or self._physics_dt), self._physics_dt)
        desired_steps = (target_dt + self._sensor_tick_residual_s) / self._physics_dt
        n_sub = max(1, int(math.floor(desired_steps + 0.5)))
        actual_dt = n_sub * self._physics_dt
        self._sensor_tick_residual_s += target_dt - actual_dt

        with self._data_lock:
            self._data.qvel[:] = 0.0
            if hasattr(self._data, "qacc"):
                self._data.qacc[:] = 0.0
            self._sim_time += actual_dt
            self._data.time = self._sim_time
            return self._sim_time

    def step_physics(self, n_steps: int = 1) -> None:
        """Pure physics step, no policy inference."""
        import mujoco

        with self._data_lock:
            for _ in range(n_steps):
                mujoco.mj_step(self._model, self._data)
            self._sim_time += self._physics_dt * n_steps

    def _watchdog_cmd_vel(self) -> None:
        """Zero out cmd_vel if watchdog timeout exceeded.
        # Extracted from src/drivers/sim/nova_nav_bridge.py _watchdog_cmd_vel
        """
        with self._lock:
            if self._cmd_vel_time > 0 and time.time() - self._cmd_vel_time > self._robot_cfg.cmd_vel_watchdog_sec:
                self._cmd_vel[:] = 0.0

    def _step_policy(self) -> None:
        """Run one policy inference step, write action to MuJoCo ctrl.
        # Extracted from src/drivers/sim/nova_nav_bridge.py NavBridge.step_policy
        """
        with self._lock:
            direction = self._cmd_vel.copy()

        gyro, pg = self._get_imu()
        jp, jv = self._get_joint_state()

        if self._policy is not None:
            idle_command = self._is_idle_policy_command(direction)
            if idle_command and not bool(getattr(self._policy, "run_at_idle", False)):
                self._policy_idle_hold = True
                standing_dart = self._robot_cfg.standing_pose_array
                self._write_leg_ctrl(standing_dart[DART_TO_MJ])
                return
            if self._policy_idle_hold:
                self._policy.reset()
                self._policy.warm_up(gyro, pg, jp, jv)
                self._policy_idle_hold = False
            obs = self._policy.build_obs(gyro, pg, direction, jp, jv)
            action_dart = self._policy.infer(obs)  # (16,) Dart order
            if idle_command and bool(getattr(self._policy, "zero_wheels_at_idle", False)):
                action_dart[12:] = 0.0
            # Dart -> MuJoCo order
            action_mj = action_dart[DART_TO_MJ]
            self._write_leg_ctrl(action_mj)
        else:
            # No policy: hold standing pose
            standing_dart = self._robot_cfg.standing_pose_array
            action_mj = standing_dart[DART_TO_MJ]
            self._write_leg_ctrl(action_mj)

    def _is_idle_policy_command(self, direction: np.ndarray) -> bool:
        return bool(np.linalg.norm(direction) <= self._policy_idle_cmd_eps)

    def _apply_kinematic_cmd(self, *, integrate_dt: float = 0.0) -> None:
        """Apply cmd_vel directly to the free joint for deterministic nav smoke tests."""
        if self._model is None or self._data is None:
            return

        with self._lock:
            vx_body, vy_body, wz = self._cmd_vel.copy()

        qadr = self._root_qposadr
        dadr = self._root_dofadr
        if qadr + 7 > len(self._data.qpos) or dadr + 6 > len(self._data.qvel):
            return

        quat = self._data.qpos[qadr + 3 : qadr + 7]
        yaw = self._yaw_from_wxyz(quat)
        c, s = np.cos(yaw), np.sin(yaw)
        vx_world = vx_body * c - vy_body * s
        vy_world = vx_body * s + vy_body * c

        dt = max(0.0, float(integrate_dt))
        if dt > 0.0:
            self._data.qpos[qadr + 0] += vx_world * dt
            self._data.qpos[qadr + 1] += vy_world * dt
            yaw += float(wz) * dt
            c, s = np.cos(yaw), np.sin(yaw)
            vx_world = vx_body * c - vy_body * s
            vy_world = vx_body * s + vy_body * c

        self._data.qvel[dadr + 0] = vx_world
        self._data.qvel[dadr + 1] = vy_world
        self._data.qvel[dadr + 2] = 0.0
        self._data.qvel[dadr + 3] = 0.0
        self._data.qvel[dadr + 4] = 0.0
        self._data.qvel[dadr + 5] = wz

        # Keep the simplified simulator upright and at nominal body height.
        self._data.qpos[qadr + 2] = float(self._robot_cfg.init_position[2])
        cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
        self._data.qpos[qadr + 3] = cy
        self._data.qpos[qadr + 4] = 0.0
        self._data.qpos[qadr + 5] = 0.0
        self._data.qpos[qadr + 6] = sy

    @staticmethod
    def _yaw_from_wxyz(quat_wxyz: np.ndarray) -> float:
        w, x, y, z = [float(v) for v in quat_wxyz[:4]]
        return float(
            np.arctan2(
                2.0 * (w * z + x * y),
                1.0 - 2.0 * (y * y + z * z),
            )
        )

    def _write_leg_ctrl(self, targets_mj: np.ndarray) -> None:
        """Write joint command in MuJoCo joint order, independent of XML actuator order."""
        ctrl_values = np.asarray(targets_mj, dtype=np.float64)
        if self._leg_control_mode == "torque":
            self._last_leg_targets_mj = ctrl_values.copy()
            ctrl_values = self._compute_leg_torques(ctrl_values)

        wrote_by_actuator = False
        for i, actuator_id in enumerate(self._leg_actuator_ids):
            if actuator_id < 0 or i >= len(ctrl_values):
                continue
            if actuator_id < len(self._data.ctrl):
                self._data.ctrl[actuator_id] = ctrl_values[i]
                wrote_by_actuator = True

        if wrote_by_actuator:
            return

        offset = self._leg_actuator_offset
        ctrl_span = min(len(ctrl_values), max(0, len(self._data.ctrl) - offset))
        self._data.ctrl[offset : offset + ctrl_span] = ctrl_values[:ctrl_span]

    def _refresh_leg_torque_ctrl(self) -> None:
        if self._leg_control_mode != "torque" or self._last_leg_targets_mj is None:
            return
        ctrl_values = self._compute_leg_torques(self._last_leg_targets_mj)
        wrote_by_actuator = False
        for i, actuator_id in enumerate(self._leg_actuator_ids):
            if actuator_id < 0 or i >= len(ctrl_values):
                continue
            if actuator_id < len(self._data.ctrl):
                self._data.ctrl[actuator_id] = ctrl_values[i]
                wrote_by_actuator = True
        if wrote_by_actuator:
            return
        offset = self._leg_actuator_offset
        ctrl_span = min(len(ctrl_values), max(0, len(self._data.ctrl) - offset))
        self._data.ctrl[offset : offset + ctrl_span] = ctrl_values[:ctrl_span]

    def _compute_leg_torques(self, targets_mj: np.ndarray) -> np.ndarray:
        joint_pos_mj, joint_vel_mj = self._get_joint_state()
        kp_mj = self._robot_cfg.torque_kp_array[DART_TO_MJ]
        kd_mj = self._robot_cfg.torque_kd_array[DART_TO_MJ]
        limit_mj = self._robot_cfg.torque_limit_array[DART_TO_MJ]

        torques = np.zeros(16, dtype=np.float64)
        position_mask = kp_mj > 0.0
        velocity_mask = ~position_mask
        torques[position_mask] = (
            kp_mj[position_mask] * (targets_mj[position_mask] - joint_pos_mj[position_mask])
            - kd_mj[position_mask] * joint_vel_mj[position_mask]
        )
        torques[velocity_mask] = kd_mj[velocity_mask] * (targets_mj[velocity_mask] - joint_vel_mj[velocity_mask])
        return np.clip(torques, -limit_mj, limit_mj)

    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    # State reading
    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def get_robot_state(self, robot_id: str = "robot_0") -> RobotState:
        """Read current robot state snapshot."""
        with self._data_lock:
            return self._get_robot_state_unlocked(robot_id)

    def _get_robot_state_unlocked(self, robot_id: str = "robot_0") -> RobotState:
        del robot_id
        qadr = self._root_qposadr
        dadr = self._root_dofadr
        pos = self._data.qpos[qadr : qadr + 3].copy()
        quat_wxyz = self._data.qpos[qadr + 3 : qadr + 7].copy()
        # MuJoCo quaternion w,x,y,z -> ROS x,y,z,w
        orientation = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
        linear_vel = self._data.qvel[dadr : dadr + 3].copy()
        angular_vel = self._data.qvel[dadr + 3 : dadr + 6].copy()

        jp, jv = self._get_joint_state()
        gyro, pg = self._get_imu()
        linear_acceleration = self._get_imu_linear_acceleration(pg)

        return RobotState(
            position=pos,
            orientation=orientation,
            linear_velocity=linear_vel,
            angular_velocity=angular_vel,
            joint_positions=jp,
            joint_velocities=jv,
            imu_gyro=gyro,
            imu_projected_gravity=pg,
            imu_linear_acceleration=linear_acceleration,
        )

    def _get_imu(self):
        """Extract IMU data (gyroscope + projected gravity).
        # Extracted from src/drivers/sim/nova_nav_bridge.py get_imu()
        """
        sensor_imu = self._get_sensor_imu()
        if sensor_imu is not None:
            return sensor_imu

        body_id = self._base_body_id
        R = self._data.xmat[body_id].reshape(3, 3)  # body-to-world
        dadr = self._root_dofadr
        omega_world = self._data.qvel[dadr + 3 : dadr + 6]
        gyroscope = R.T @ omega_world  # world -> body frame

        gravity_world = np.array([0.0, 0.0, -1.0])
        projected_gravity = R.T @ gravity_world

        return gyroscope.astype(np.float64), projected_gravity.astype(np.float64)

    def _sensor_data(self, *names: str) -> np.ndarray | None:
        for name in names:
            try:
                data = np.asarray(self._data.sensor(name).data, dtype=np.float64)
            except Exception:
                continue
            return data
        return None

    def _get_sensor_imu(self) -> tuple[np.ndarray, np.ndarray] | None:
        imu_quat = self._sensor_data("lidar-orientation", "orientation")
        gyro_local = self._sensor_data("lidar-angular-velocity", "angular-velocity")
        if imu_quat is None or gyro_local is None:
            return None
        if imu_quat.size != 4 or gyro_local.size != 3:
            return None
        if not np.isfinite(imu_quat).all() or np.linalg.norm(imu_quat) <= 1e-9:
            return None

        imu_R = self._quat_wxyz_to_mat(imu_quat)
        qpos = getattr(self._data, "qpos", None)
        if qpos is None:
            base_R = imu_R
        else:
            base_quat = np.asarray(
                qpos[self._root_qposadr + 3 : self._root_qposadr + 7],
                dtype=np.float64,
            )
            base_R = self._quat_wxyz_to_mat(base_quat)
        gravity_world = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        projected_gravity = imu_R.T @ gravity_world
        gyro_world = imu_R @ gyro_local.astype(np.float64)
        gyro_base = base_R.T @ gyro_world
        return gyro_base.astype(np.float64), projected_gravity.astype(np.float64)

    def _get_imu_linear_acceleration(self, projected_gravity: np.ndarray) -> np.ndarray:
        accel = self._sensor_data("lidar-linear-acceleration", "linear-acceleration")
        if accel is None:
            accel = -np.asarray(projected_gravity, dtype=np.float64) * 9.80665
        if accel.size != 3 or not np.isfinite(accel).all():
            return np.zeros(3, dtype=np.float64)
        return accel.astype(np.float64)

    @staticmethod
    def _quat_wxyz_to_mat(quat_wxyz: np.ndarray) -> np.ndarray:
        q = np.asarray(quat_wxyz, dtype=np.float64)
        norm = np.linalg.norm(q)
        if norm <= 1e-12:
            return np.eye(3, dtype=np.float64)
        w, x, y, z = q / norm
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def _get_joint_state(self):
        """Read position and velocity of 16 leg joints.
        # Extracted from src/drivers/sim/nova_nav_bridge.py get_joint_state()
        """
        valid_ids = [j for j in self._leg_joint_ids if j >= 0]
        if not valid_ids:
            return np.zeros(16), np.zeros(16)

        pos = np.array(
            [self._data.qpos[self._model.jnt_qposadr[j]] if j >= 0 else 0.0 for j in self._leg_joint_ids],
            dtype=np.float64,
        )
        vel = np.array(
            [self._data.qvel[self._model.jnt_dofadr[j]] if j >= 0 else 0.0 for j in self._leg_joint_ids],
            dtype=np.float64,
        )

        # Pad to 16 if some joints are missing
        if len(pos) < 16:
            pos = np.pad(pos, (0, 16 - len(pos)))
            vel = np.pad(vel, (0, 16 - len(vel)))
        return pos, vel

    def get_camera_data(self, camera_name: str = "front_camera") -> CameraData | None:
        """Read camera frame (RGB + depth)."""
        if camera_name not in self._cameras:
            return None
        with self._data_lock:
            return self._cameras[camera_name].render(self._data)

    def get_lidar_points(self, sample_count: int | None = None) -> np.ndarray:
        """Read current LiDAR point cloud.

        Returns:
            (N, 4) float32 XYZI world frame, or (N, 3) from livox_mid360 module
        """
        if self._lidar is None:
            return np.zeros((0, 4), dtype=np.float32)
        with self._data_lock:
            return self._lidar.scan(sample_count=sample_count)

    def get_lidar_backend_report(self) -> dict:
        """Return JSON-ready LiDAR backend evidence for validation reports."""
        if self._lidar is None:
            return {
                "backend": "none",
                "product_backend": False,
                "product_lidar_backend_verified": False,
                "fallback_used": False,
                "error": "lidar_not_initialized",
            }
        report = getattr(self._lidar, "backend_report", None)
        if callable(report):
            return report()
        return {
            "backend": getattr(self._lidar, "backend", "unknown"),
            "product_backend": False,
            "product_lidar_backend_verified": False,
            "fallback_used": False,
            "error": "backend_report_unavailable",
        }

    def get_discrete_rays(self, config: DiscreteRayConfig | None = None) -> DiscreteRayData:
        """Return IsaacLab-style fixed-pattern terrain ray observations."""
        import mujoco

        with self._data_lock:
            return self._get_discrete_rays_locked(config)

    def _get_discrete_rays_locked(self, config: DiscreteRayConfig | None = None) -> DiscreteRayData:
        cfg = config or self._discrete_ray_cfg
        if cfg.pattern != "grid":
            raise ValueError(f"unsupported discrete ray pattern: {cfg.pattern}")
        if self._model is None or self._data is None:
            return DiscreteRayData(
                heights=np.zeros((0,), dtype=np.float32),
                points_body=np.zeros((0, 3), dtype=np.float32),
                points_world=np.zeros((0, 3), dtype=np.float32),
                valid_mask=np.zeros((0,), dtype=bool),
                pattern=cfg.pattern,
                metadata={"reason": "engine_not_loaded"},
            )

        x_count = max(1, int(cfg.x_count))
        y_count = max(1, int(cfg.y_count))
        xs = np.linspace(float(cfg.x_min), float(cfg.x_max), x_count)
        ys = np.linspace(float(cfg.y_min), float(cfg.y_max), y_count)
        sample_body = np.array(
            [[x, y, float(cfg.ray_start_z)] for x in xs for y in ys],
            dtype=np.float64,
        )
        n_samples = int(sample_body.shape[0])

        body_id = self._base_body_id
        body_pos = self._data.xpos[body_id].copy()
        rmat = self._data.xmat[body_id].reshape(3, 3).copy()
        starts_world = body_pos + sample_body @ rmat.T

        direction_body = np.asarray(cfg.direction_body, dtype=np.float64).reshape(3)
        norm = float(np.linalg.norm(direction_body))
        if norm <= 1e-12:
            raise ValueError("discrete ray direction must be non-zero")
        direction_body = direction_body / norm
        direction_world = direction_body @ rmat.T

        geomgroup = np.zeros(6, dtype=np.uint8)
        for group in cfg.geom_group_mask:
            idx = int(group)
            if 0 <= idx < len(geomgroup):
                geomgroup[idx] = 1
        if not np.any(geomgroup):
            geomgroup[:] = 1

        hits_world = np.full((n_samples, 3), np.nan, dtype=np.float32)
        hits_body = np.full((n_samples, 3), np.nan, dtype=np.float32)
        heights = np.full((n_samples,), np.nan, dtype=np.float32)
        valid = np.zeros((n_samples,), dtype=bool)
        ray_length = max(0.01, float(cfg.ray_length))

        for i, start in enumerate(starts_world):
            geom_id = np.array([-1], dtype=np.int32)
            dist = float(
                mujoco.mj_ray(
                    self._model,
                    self._data,
                    start,
                    direction_world,
                    geomgroup,
                    1,
                    body_id,
                    geom_id,
                )
            )
            if not (0.0 <= dist <= ray_length):
                continue
            geom = int(geom_id[0])
            if geom >= 0:
                geom_body = int(self._model.geom_bodyid[geom])
                if self._is_descendant_body(geom_body, body_id):
                    continue
            hit_world = start + direction_world * dist
            hit_body = (hit_world - body_pos) @ rmat
            hits_world[i] = hit_world.astype(np.float32)
            hits_body[i] = hit_body.astype(np.float32)
            heights[i] = float(body_pos[2] - hit_world[2])
            valid[i] = True

        return DiscreteRayData(
            heights=heights,
            points_body=hits_body,
            points_world=hits_world,
            valid_mask=valid,
            pattern=cfg.pattern,
            metadata={
                "sample_count": n_samples,
                "valid_count": int(valid.sum()),
                "x_count": x_count,
                "y_count": y_count,
                "ray_start_z": float(cfg.ray_start_z),
                "ray_length": ray_length,
                "direction_body": [float(v) for v in direction_body],
            },
        )

    def _is_descendant_body(self, body_id: int, root_body_id: int) -> bool:
        """Return whether ``body_id`` is ``root_body_id`` or its child body."""
        if self._model is None:
            return False
        cursor = int(body_id)
        root = int(root_body_id)
        while cursor >= 0:
            if cursor == root:
                return True
            if cursor == 0:
                return False
            cursor = int(self._model.body_parentid[cursor])
        return False

    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    # Control interface
    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def set_cmd_vel(self, cmd: VelocityCommand, robot_id: str = "robot_0") -> None:
        """Set velocity command (thread-safe)."""
        with self._lock:
            self._cmd_vel[0] = np.clip(cmd.linear_x, -self._robot_cfg.max_linear_vel, self._robot_cfg.max_linear_vel)
            self._cmd_vel[1] = np.clip(cmd.linear_y, -self._robot_cfg.max_linear_vel, self._robot_cfg.max_linear_vel)
            self._cmd_vel[2] = np.clip(cmd.angular_z, -self._robot_cfg.max_angular_vel, self._robot_cfg.max_angular_vel)
            self._cmd_vel_time = time.time()

    def set_joint_positions(self, positions: np.ndarray) -> None:
        """Directly set joint target positions (bypasses ONNX policy)."""
        if len(positions) < 16:
            raise ValueError(f"Expected 16 joint positions, got {len(positions)}")
        self._write_leg_ctrl(np.asarray(positions[:16], dtype=np.float64))

    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    # Scene manipulation
    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def set_robot_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """Teleport robot to a given pose (ROS quaternion x,y,z,w)."""
        import mujoco

        self._data.qpos[0:3] = position[:3]
        # ROS x,y,z,w -> MuJoCo w,x,y,z
        self._data.qpos[3] = orientation[3]  # w
        self._data.qpos[4] = orientation[0]  # x
        self._data.qpos[5] = orientation[1]  # y
        self._data.qpos[6] = orientation[2]  # z
        # Zero velocities
        self._data.qvel[:] = 0.0
        mujoco.mj_forward(self._model, self._data)

    def add_obstacle(
        self, name: str, shape: str, size: list[float], position: list[float], rgba: list[float] | None = None
    ) -> None:
        """Dynamically add obstacle (MuJoCo does not support runtime geom addition).

        Note: MuJoCo does not support modifying model structure during physics stepping.
        Adding an obstacle requires a reload via load(). This method queues the request
        in world_config; it takes effect on next reset()+load().
        """
        from sim.engine.core.world import ObstacleConfig

        obs = ObstacleConfig(
            name=name,
            shape=shape,
            size=size,
            position=position,
            rgba=rgba or [0.7, 0.7, 0.7, 1.0],
        )
        self._world_cfg.obstacles.append(obs)
        print(f'[MuJoCoEngine] Obstacle "{name}" queued 鈥?call load() to apply.')

    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    # Properties
    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    @property
    def physics_integrator(self) -> str:
        """Return the active MuJoCo integrator name."""
        if self._model is None:
            return "unloaded"
        import mujoco

        active = int(self._model.opt.integrator)
        options = {
            "euler": int(mujoco.mjtIntegrator.mjINT_EULER),
            "rk4": int(mujoco.mjtIntegrator.mjINT_RK4),
            "implicit": int(mujoco.mjtIntegrator.mjINT_IMPLICIT),
            "implicitfast": int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
        }
        for name, value in options.items():
            if active == value:
                return name
        return f"unknown:{active}"

    def set_physics_integrator(self, name: str) -> str:
        """Override the loaded model integrator without changing its timestep."""
        if self._model is None:
            raise RuntimeError("MuJoCo model is not loaded")
        normalized = str(name or "").strip().lower()
        if normalized == "model":
            return self.physics_integrator
        import mujoco

        options = {
            "euler": mujoco.mjtIntegrator.mjINT_EULER,
            "rk4": mujoco.mjtIntegrator.mjINT_RK4,
            "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
            "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
        }
        if normalized not in options:
            raise ValueError(f"unsupported MuJoCo integrator: {name}")
        self._model.opt.integrator = options[normalized]
        return self.physics_integrator

    @property
    def dt(self) -> float:
        return self._physics_dt

    @property
    def control_dt(self) -> float:
        return self._control_dt

    @property
    def model(self):
        """MuJoCo MjModel (for direct access)."""
        return self._model

    @property
    def data(self):
        """MuJoCo MjData (for direct access)."""
        return self._data

    @property
    def has_policy(self) -> bool:
        return self._policy is not None

    @property
    def policy_path(self) -> str:
        return self._policy_path

    def remove_robot(self, robot_id: str) -> None:
        """Single-robot engine stub.

        Args:
            robot_id: robot identifier to remove

        Raises:
            ValueError: if robot_id is not the expected single robot
        """
        if robot_id not in ("robot_0", self._robot_cfg.name or "robot_0"):
            raise ValueError(f"Unknown robot: {robot_id}")
        self.close()

    @property
    def drive_mode(self) -> str:
        return self._drive_mode

    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    # Background physics thread (optional, for async stepping)
    # 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def start_background_sim(self) -> None:
        """Run physics stepping in a background thread (separate from rendering)."""
        if self._sim_thread and self._sim_thread.is_alive():
            return
        self._stop_event.clear()
        self._sim_thread = threading.Thread(
            target=self._background_sim_loop,
            name="MuJoCoSimThread",
            daemon=True,
        )
        self._sim_thread.start()

    def stop_background_sim(self) -> None:
        """Stop the background physics thread."""
        self._stop_event.set()
        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=3.0)
        self._sim_thread = None

    def _background_sim_loop(self) -> None:
        """Background physics loop: 50Hz policy + 500Hz physics.
        # Extracted from src/drivers/sim/nova_nav_bridge.py NavBridge.spin()
        """
        import mujoco

        last_policy = 0.0
        print(
            f"[MuJoCoEngine] Background sim started: "
            f"physics={1 / self._physics_dt:.0f}Hz, "
            f"policy={1 / self._control_dt:.0f}Hz"
        )

        while not self._stop_event.is_set():
            t0 = time.time()

            with self._lock:
                pass  # acquire lock pattern

            with self._data_lock:
                if self._drive_mode == "kinematic":
                    self._apply_kinematic_cmd(integrate_dt=self._physics_dt)
                    mj_forward = getattr(mujoco, "mj_forward", None)
                    if callable(mj_forward):
                        mj_forward(self._model, self._data)
                else:
                    mujoco.mj_step(self._model, self._data)
                self._sim_time += self._physics_dt

                if self._sim_time - last_policy >= self._control_dt:
                    self._watchdog_cmd_vel()
                    self._step_policy()
                    last_policy = self._sim_time

            elapsed = time.time() - t0
            sleep_t = self._physics_dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

        print("[MuJoCoEngine] Background sim stopped.")
