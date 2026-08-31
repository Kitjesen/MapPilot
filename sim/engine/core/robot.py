"""Robot configuration — RobotConfig."""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from runtime.runtime_interface import FRAMES

THUNDER_V4_JOINT_NAMES = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FR_foot_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FL_foot_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RR_foot_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RL_foot_joint",
]


@dataclass
class RobotConfig:
    """Thunder V4 robot simulation configuration.

    Single source of truth: physical parameters, initial pose, policy path,
    and joint mapping are all defined here.
    This simulation model is separate from the field RobotConfig.
    """

    # Model files
    robot_xml: str = ""  # robot.xml path (relative to sim/ or absolute)
    policy_onnx: str = ""  # policy.onnx path

    # Initial pose
    init_position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.55])
    init_orientation_wxyz: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])

    # Physical parameters (from global CLAUDE.md memory)
    pd_kp: float = 65.0  # position gain (hip)
    pd_kv: float = 5.0  # velocity gain (damping)
    leg_control_mode: str = "auto"  # auto, position, or torque
    torque_kp: list[float] = field(
        default_factory=lambda: [70.0, 100.0, 120.0] * 4 + [0.0] * 4
    )
    torque_kd: list[float] = field(
        default_factory=lambda: [15.0, 15.0, 20.0] * 4 + [1.0] * 4
    )
    torque_limit: list[float] = field(
        default_factory=lambda: [
            120.0,
            120.0,
            120.0,
            120.0,
            120.0,
            120.0,
            120.0,
            120.0,
            120.0,
            120.0,
            120.0,
            120.0,
            17.0,
            17.0,
            17.0,
            17.0,
        ]
    )

    # Policy parameters (consistent with brainstem StandardObservationBuilder)
    # Extracted from src/drivers/sim/nova_nav_bridge.py
    action_scale: list[float] = field(
        default_factory=lambda: [
            0.125,
            0.25,
            0.25,  # FR: hip, thigh, calf
            0.125,
            0.25,
            0.25,  # FL
            0.125,
            0.25,
            0.25,  # RR
            0.125,
            0.25,
            0.25,  # RL
            5.0,
            5.0,
            5.0,
            5.0,  # foot
        ]
    )
    imu_gyro_scale: float = 0.25
    joint_vel_scale: float = 0.05
    policy_freq_hz: float = 50.0  # policy_1119: 200 Hz physics with decimation 4
    policy_cpu_threads: int = 1
    obs_dim: int = 57
    history_len: int = 5

    # Standing pose (Dart order, use as-is, no sign flip)
    standing_pose: list[float] = field(
        default_factory=lambda: [
            -0.1,
            -0.8,
            1.8,  # FR: hip, thigh, calf
            0.1,
            0.8,
            -1.8,  # FL
            0.1,
            0.8,
            -1.8,  # RR
            -0.1,
            -0.8,
            1.8,  # RL
            0.0,
            0.0,
            0.0,
            0.0,  # foot
        ]
    )

    # Joint names (MuJoCo order)
    leg_joint_names: list[str] = field(default_factory=lambda: THUNDER_V4_JOINT_NAMES.copy())

    # Body names
    # Thunder V4 uses base_link as the root body. The LingTu runtime MJCF adds
    # a lidar_link body for the MID-360 sensor pose.
    base_body_name: str = FRAMES.model_base
    lidar_body_name: str = FRAMES.lidar

    # Current Thunder V4 runtime MJCF has one actuator per leg/wheel joint.
    leg_act_offset: int = 0

    # Joint order mapping (from nova_nav_bridge.py original constants)
    # MuJoCo (4+4+4+4): FR(h,t,c,f), FL(...), RR(...), RL(...)
    # Dart   (3+3+3+3+4): FR(h,t,c), FL(h,t,c), RR, RL, FR_f, FL_f, RR_f, RL_f
    mj_to_dart: list[int] = field(default_factory=lambda: [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7, 11, 15])
    dart_to_mj: list[int] = field(default_factory=lambda: [0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15])

    # Velocity command limits
    max_linear_vel: float = 1.0  # m/s
    max_angular_vel: float = 1.0  # rad/s
    cmd_vel_watchdog_sec: float = 0.2  # zero-out timeout

    def resolve_paths(self, base_dir: str | None = None) -> "RobotConfig":
        """Resolve relative paths to absolute paths.

        Args:
            base_dir: base directory; defaults to sim/ directory
        """
        if base_dir is None:
            base_dir = str(Path(__file__).resolve().parents[2])

        base = Path(base_dir)
        if self.robot_xml and not Path(self.robot_xml).is_absolute():
            self.robot_xml = str(base / self.robot_xml)
        if self.policy_onnx and not Path(self.policy_onnx).is_absolute():
            self.policy_onnx = str(base / self.policy_onnx)
        return self

    @property
    def standing_pose_array(self) -> np.ndarray:
        return np.array(self.standing_pose, dtype=np.float64)

    @property
    def action_scale_array(self) -> np.ndarray:
        return np.array(self.action_scale, dtype=np.float64)

    @property
    def torque_kp_array(self) -> np.ndarray:
        return np.array(self.torque_kp, dtype=np.float64)

    @property
    def torque_kd_array(self) -> np.ndarray:
        return np.array(self.torque_kd, dtype=np.float64)

    @property
    def torque_limit_array(self) -> np.ndarray:
        return np.array(self.torque_limit, dtype=np.float64)

    @property
    def mj_to_dart_array(self) -> np.ndarray:
        return np.array(self.mj_to_dart, dtype=np.int32)

    @property
    def dart_to_mj_array(self) -> np.ndarray:
        return np.array(self.dart_to_mj, dtype=np.int32)

    @classmethod
    def default_thunder_v4(cls) -> "RobotConfig":
        """Return current Thunder simulation config (paths resolved at runtime)."""
        cfg = cls()
        cfg.robot_xml = "robots/doso/thunder_v4/mjcf/thunderv4.xml"
        cfg.policy_onnx = "controllers/doso/thunder_v4/locomotion/policy/policy_1119.onnx"
        return cfg
