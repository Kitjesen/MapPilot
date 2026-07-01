"""Robot model definitions for multi-robot simulation.

RobotModel is the canonical description of a robot type -- its
kinematics, sensors, and controller parameters -- along with a registry
pattern mirroring the LingTu robot_profile / backend system.

Usage:
    from sim.engine.core.robot_model import NOVA_DOG_MODEL, register_robot

    register_robot(NOVA_DOG_MODEL)
    model = get_robot("nova_dog")
    print(model.model_type)  # "quadruped"

    # Discover all robot packages under sim/robots/
    count = auto_discover("/path/to/sim")
"""

from __future__ import annotations

from dataclasses import dataclass, field
from importlib import import_module
from logging import getLogger
from pathlib import Path
from typing import ClassVar

_LOG = getLogger(__name__)


# 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
# Nested configuration dataclasses
# 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€


@dataclass(frozen=True)
class JointConfig:
    """Joint configuration for a robot model.

    Attributes:
        num_legs: Number of legs (0 for wheeled/humanoid without legs).
        joint_names: Ordered joint names matching the MJCF actuator order.
        standing_pose: Default joint angles at standstill (radians).
        action_scale: Per-joint action scaling factor for policy output.
    """

    num_legs: int = 0
    joint_names: tuple[str, ...] = ()
    standing_pose: tuple[float, ...] = ()
    action_scale: tuple[float, ...] = ()


@dataclass(frozen=True)
class SensorLayout:
    """Sensor body/site names referenced in the MJCF model.

    Attributes:
        lidar_body: Name of the LiDAR body/site (empty string if none).
        camera_bodies: One or more camera body names ordered by convention.
        imu_body: Name of the IMU body/site (empty string if none).
    """

    lidar_body: str = ""
    camera_bodies: tuple[str, ...] = ()
    imu_body: str = ""


@dataclass(frozen=True)
class ControllerParams:
    """Low-level controller and policy inference parameters.

    Attributes:
        policy_freq_hz: Inference frequency of the RL policy (0 if none).
        obs_dim: Observation vector dimension fed to the policy.
        pd_kp: Proportional gain for joint-level PD control.
        pd_kv: Derivative gain for joint-level PD control.
        max_linear_vel: Maximum forward/backward velocity (m/s).
        max_angular_vel: Maximum yaw rate (rad/s).
    """

    policy_freq_hz: float = 50.0
    obs_dim: int = 57
    pd_kp: float = 65.0
    pd_kv: float = 5.0
    max_linear_vel: float = 1.0
    max_angular_vel: float = 1.0


# 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
# RobotModel -- frozen value-object descriptor
# 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€


@dataclass(frozen=True)
class RobotModel:
    """Canonical descriptor for a robot type in multi-robot simulation.

    Every robot instance in a sim scene is backed by a RobotModel.
    Fields are immutable -- a model is a value object shared across
    all instances of that type.

    Attributes:
        name: Unique identifier (e.g. ``"nova_dog"``, ``"omni_cart"``).
        model_type: Kinematic category: ``"quadruped"`` | ``"wheeled"`` | ``"humanoid"``.
        mjcf_path: Path to MJCF XML, **relative to the sim/ directory**.
        policy_onnx: Optional path to an ONNX policy checkpoint.
        init_position: Spawn position ``(x, y, z)`` in world frame.
        init_orientation: Spawn orientation ``(w, x, y, z)`` quaternion (ROS convention).
        joints: Joint definitions and configuration.
        sensors: Sensor body/site names in the MJCF model.
        controller: Low-level controller and policy parameters.
    """

    name: str = ""
    model_type: str = ""
    mjcf_path: str = ""
    policy_onnx: str = ""
    init_position: tuple[float, float, float] = (0.0, 0.0, 0.55)
    init_orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    joints: JointConfig = field(default_factory=JointConfig)
    sensors: SensorLayout = field(default_factory=SensorLayout)
    controller: ControllerParams = field(default_factory=ControllerParams)


# 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
# Registry (class-level singleton dict)
# 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€


class _RobotRegistry:
    """Internal registry for RobotModel definitions.

    Mirrors ``src/runtime/registry.py``'s decorator pattern but uses an
    explicit register/get/list API rather than class decorators.
    """

    _models: ClassVar[dict[str, RobotModel]] = {}

    @classmethod
    def register(cls, model: RobotModel) -> None:
        """Register a RobotModel by its ``.name``.

        Idempotent for the same model object 鈥?re-registering the exact
        same instance is a no-op.  Registering a *different* model under
        an existing name raises ``KeyError``.

        Raises:
            ValueError: Model name is empty.
            KeyError: A *different* model is already registered under this name.
        """
        if not model.name:
            raise ValueError("RobotModel.name must be a non-empty string")
        existing = cls._models.get(model.name)
        if existing is model:
            return  # idempotent: same object, skip
        if existing is not None:
            raise KeyError(
                f"Robot model {model.name!r} is already registered"
            )
        cls._models[model.name] = model
        _LOG.info("Registered robot model: %s (%s)", model.name, model.model_type)

    @classmethod
    def get(cls, name: str) -> RobotModel:
        """Look up a registered model by name.

        Raises:
            KeyError: No model with that name is registered.
        """
        if name not in cls._models:
            available = "', '".join(cls._models)
            raise KeyError(
                f"Unknown robot model {name!r}. Available: ['{available}']"
            )
        return cls._models[name]

    @classmethod
    def list(cls) -> list[str]:
        """Return sorted list of all registered model names."""
        return sorted(cls._models)

    @classmethod
    def discover(cls, base_dir: str) -> int:
        """Scan ``sim/robots/*/__init__.py`` and import each package.

        Each ``__init__.py`` is expected to call ``register_robot()``
        for one or more models.

        Args:
            base_dir: Absolute path to the ``sim/`` root directory.

        Returns:
            Number of robot packages successfully imported.
        """
        robots_dir = Path(base_dir) / "robots"
        if not robots_dir.is_dir():
            return 0

        count = 0
        for child in sorted(robots_dir.iterdir()):
            if not child.is_dir():
                continue
            init_file = child / "__init__.py"
            if not init_file.is_file():
                continue
            module_name = f"sim.robots.{child.name}"
            try:
                import_module(module_name)
                count += 1
            except Exception as exc:
                _LOG.warning(
                    "Failed to import robot package %s: %s",
                    module_name,
                    exc,
                )
        return count


# Public API -- module-level functions delegating to _RobotRegistry
register_robot = _RobotRegistry.register
get_robot = _RobotRegistry.get
list_robots = _RobotRegistry.list
auto_discover = _RobotRegistry.discover


# 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
# Built-in concrete models (auto-registered at import time)
# 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

NOVA_DOG_MODEL = RobotModel(
    name="nova_dog",
    model_type="quadruped",
    mjcf_path="robots/nova_dog/robot_with_camera.xml",
    joints=JointConfig(
        num_legs=4,
        joint_names=(
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
        ),
        # Note: foot joints (suffix _foot_joint) are limited="false" in MJCF
        # (free rotation, no position bounds).  standing_pose=0.0 for them is
        # a placeholder 鈥?not enforced.  action_scale=5.0 is clamped by the
        # position actuator ctrlrange [-1.0, 1.0] (see scaffold_robot.py).
        standing_pose=(
            -0.1, -0.8, 1.8,   # FR: hip, thigh, calf
            0.1, 0.8, -1.8,    # FL
            0.1, 0.8, -1.8,    # RR
            -0.1, -0.8, 1.8,   # RL
            0.0, 0.0, 0.0, 0.0,  # foot (unconstrained, placeholder)
        ),
        action_scale=(
            0.125, 0.25, 0.25,   # FR: hip, thigh, calf
            0.125, 0.25, 0.25,   # FL
            0.125, 0.25, 0.25,   # RR
            0.125, 0.25, 0.25,   # RL
            5.0, 5.0, 5.0, 5.0,  # foot
        ),
    ),
    sensors=SensorLayout(
        lidar_body="lidar_link",
        camera_bodies=("front_camera",),
        imu_body="imu",
    ),
    controller=ControllerParams(
        policy_freq_hz=50.0,
        obs_dim=57,
        pd_kp=65.0,
        pd_kv=5.0,
        max_linear_vel=1.0,
        max_angular_vel=1.0,
    ),
)

OMNI_CART_MODEL = RobotModel(
    name="omni_cart",
    model_type="wheeled",
    mjcf_path="robots/omni_cart/omni_cart.xml",
    init_position=(0.0, 0.0, 0.08),
    init_orientation=(1.0, 0.0, 0.0, 0.0),
    joints=JointConfig(
        num_legs=0,
        joint_names=(),
        standing_pose=(),
        action_scale=(),
    ),
    sensors=SensorLayout(
        lidar_body="lidar_link",
        camera_bodies=("front_camera",),
        imu_body="base_link",
    ),
    controller=ControllerParams(
        policy_freq_hz=0.0,
        obs_dim=0,
        pd_kp=0.0,
        pd_kv=0.0,
        max_linear_vel=2.0,
        max_angular_vel=3.0,
    ),
)

# Register built-in models so they are available on import
_RobotRegistry.register(NOVA_DOG_MODEL)
_RobotRegistry.register(OMNI_CART_MODEL)
