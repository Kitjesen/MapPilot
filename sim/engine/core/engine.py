"""Simulation engine abstract base class — engine-agnostic unified interface."""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np


@dataclass
class RobotState:
    """Complete robot motion state snapshot."""

    position: np.ndarray          # [x, y, z] world frame
    orientation: np.ndarray       # [x, y, z, w] quaternion (ROS convention)
    linear_velocity: np.ndarray   # [vx, vy, vz] world frame
    angular_velocity: np.ndarray  # [wx, wy, wz] world frame
    joint_positions: np.ndarray   # (16,) leg joint positions rad, MuJoCo order
    joint_velocities: np.ndarray  # (16,) leg joint velocities rad/s
    imu_gyro: np.ndarray          # (3,) body-frame gyroscope rad/s
    imu_projected_gravity: np.ndarray  # (3,) body-frame projected gravity (normalized)
    imu_linear_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=np.float64)
        self.orientation = np.asarray(self.orientation, dtype=np.float64)
        self.linear_velocity = np.asarray(self.linear_velocity, dtype=np.float64)
        self.angular_velocity = np.asarray(self.angular_velocity, dtype=np.float64)
        self.joint_positions = np.asarray(self.joint_positions, dtype=np.float64)
        self.joint_velocities = np.asarray(self.joint_velocities, dtype=np.float64)
        self.imu_gyro = np.asarray(self.imu_gyro, dtype=np.float64)
        self.imu_projected_gravity = np.asarray(self.imu_projected_gravity, dtype=np.float64)
        self.imu_linear_acceleration = np.asarray(self.imu_linear_acceleration, dtype=np.float64)


@dataclass
class CameraData:
    """Camera frame data."""

    rgb: np.ndarray       # (H, W, 3) uint8
    depth: np.ndarray     # (H, W) float32 in meters
    intrinsics: tuple     # (fx, fy, cx, cy)
    timestamp: float = 0.0

    @property
    def K(self) -> np.ndarray:
        """Compatibility helper for bridge code expecting a 3x3 intrinsic matrix."""
        fx, fy, cx, cy = self.intrinsics
        return np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )


@dataclass
class DiscreteRayData:
    """Fixed-pattern ray/height observation snapshot."""

    heights: np.ndarray       # (N,) base-height minus hit z, NaN when invalid
    points_body: np.ndarray   # (N, 3) hit points in body frame, NaN when invalid
    points_world: np.ndarray  # (N, 3) hit points in world frame, NaN when invalid
    valid_mask: np.ndarray    # (N,) bool hit validity
    pattern: str = "grid"
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        self.heights = np.asarray(self.heights, dtype=np.float32)
        self.points_body = np.asarray(self.points_body, dtype=np.float32)
        self.points_world = np.asarray(self.points_world, dtype=np.float32)
        self.valid_mask = np.asarray(self.valid_mask, dtype=bool)


@dataclass
class VelocityCommand:
    """Velocity command (corresponds to ROS TwistStamped)."""

    linear_x: float = 0.0   # forward velocity m/s
    linear_y: float = 0.0   # lateral velocity m/s
    angular_z: float = 0.0  # yaw rate rad/s


class SimEngine(ABC):
    """Simulation engine abstract base class.

    All concrete engines (MuJoCo, Isaac, Gazebo, etc.) implement this interface.
    Upper-layer code depends only on SimEngine, not on the underlying implementation.
    """

    def __init__(self) -> None:
        self._running: bool = False
        self._sim_time: float = 0.0
        self._state: Optional[RobotState] = None

    # ──────────────────────────────────────────────────────────────
    # Lifecycle
    # ──────────────────────────────────────────────────────────────

    @abstractmethod
    def load(self, xml_path: str, **kwargs: Any) -> None:
        """Load scene XML / URDF.

        Args:
            xml_path: path to scene description file
            **kwargs: engine-specific parameters
        """

    @abstractmethod
    def reset(self) -> RobotState:
        """Reset simulation to initial state, return initial robot state."""

    @abstractmethod
    def close(self) -> None:
        """Release simulation resources."""

    # ──────────────────────────────────────────────────────────────
    # Simulation stepping
    # ──────────────────────────────────────────────────────────────

    @abstractmethod
    def step(self, cmd: Optional[VelocityCommand] = None) -> RobotState:
        """Advance one control cycle (physics + policy).

        Args:
            cmd: velocity command; None keeps the previous command

        Returns:
            robot state after stepping
        """

    @abstractmethod
    def step_physics(self, n_steps: int = 1) -> None:
        """Pure physics step, no policy inference.

        Args:
            n_steps: number of steps to advance
        """

    # ──────────────────────────────────────────────────────────────
    # State reading
    # ──────────────────────────────────────────────────────────────

    def get_robot_state(self, robot_id: str = "robot_0") -> RobotState:
        """Read current robot state.

        Args:
            robot_id: robot identifier (defaults to "robot_0" for single-robot)

        Returns:
            RobotState for the given robot
        """
        if robot_id == "robot_0":
            if self._state is None:
                raise RuntimeError("No state available. Call reset() or step() first.")
            return self._state
        raise ValueError(f"Unknown robot_id: {robot_id}")

    @abstractmethod
    def get_camera_data(self, camera_name: str = "front_camera") -> Optional[CameraData]:
        """Read camera frame (RGB + depth).

        Args:
            camera_name: camera name as defined in MuJoCo XML

        Returns:
            CameraData or None if camera does not exist
        """

    @abstractmethod
    def get_lidar_points(self) -> np.ndarray:
        """Read current LiDAR scan point cloud.

        Returns:
            (N, 3) float32 world-frame point cloud, or (N, 4) with intensity
        """

    @abstractmethod
    def get_discrete_rays(self, config: Any | None = None) -> DiscreteRayData:
        """Read fixed-pattern terrain ray/height observations.

        Returns:
            DiscreteRayData with heights, body/world hit points, and validity mask.
        """

    # ──────────────────────────────────────────────────────────────
    # Control interface
    # ──────────────────────────────────────────────────────────────

    @abstractmethod
    def set_cmd_vel(self, cmd: VelocityCommand, robot_id: str = "robot_0") -> None:
        """Set velocity command (async, applied on next step())."""

    @abstractmethod
    def set_joint_positions(self, positions: np.ndarray) -> None:
        """Directly set joint target positions (bypasses ONNX policy).

        Args:
            positions: (16,) joint target positions, MuJoCo order
        """

    # ──────────────────────────────────────────────────────────────
    # Scene manipulation
    # ──────────────────────────────────────────────────────────────

    @abstractmethod
    def set_robot_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """Teleport robot to a given pose.

        Args:
            position: [x, y, z]
            orientation: [x, y, z, w] quaternion
        """

    @abstractmethod
    def add_obstacle(self, name: str, shape: str, size: List[float],
                     position: List[float], rgba: Optional[List[float]] = None) -> None:
        """Dynamically add an obstacle at runtime.

        Args:
            name: obstacle name (unique)
            shape: "box" | "sphere" | "cylinder"
            size: shape params (box: [lx,ly,lz], sphere: [r], cylinder: [r,h])
            position: [x, y, z]
            rgba: color [r, g, b, a], defaults to gray
        """

    # ──────────────────────────────────────────────────────────────
    # Multi-robot management
    # ──────────────────────────────────────────────────────────────

    def add_robot(self, robot_model: Any, init_pose: Optional[Any] = None,
                  robot_id: Optional[str] = None) -> str:
        """Add a robot to the simulation.

        Args:
            robot_model: robot model/configuration identifier or object
            init_pose: initial pose (engine-specific format); None uses default
            robot_id: optional explicit identifier; auto-generated if None

        Returns:
            robot_id assigned to the added robot

        Raises:
            NotImplementedError: if the engine does not support multi-robot
        """
        raise NotImplementedError(
            f"{type(self).__name__} does not support add_robot"
        )

    @abstractmethod
    def remove_robot(self, robot_id: str) -> None:
        """Remove a robot from the simulation.

        Args:
            robot_id: robot identifier to remove

        Raises:
            ValueError: if robot_id does not exist
        """

    def list_robots(self) -> list[str]:
        """List all active robot identifiers.

        Returns:
            list of active robot IDs (default: ["robot_0"] for single-robot engines)
        """
        return ["robot_0"]

    # ──────────────────────────────────────────────────────────────
    # Properties
    # ──────────────────────────────────────────────────────────────

    @property
    def sim_time(self) -> float:
        """Simulation time in seconds."""
        return self._sim_time

    @property
    def is_running(self) -> bool:
        """Whether the simulation is running."""
        return self._running

    @property
    @abstractmethod
    def dt(self) -> float:
        """Simulation physics timestep in seconds."""

    @property
    @abstractmethod
    def control_dt(self) -> float:
        """Policy control period in seconds (typically physics_dt x n_substeps)."""
