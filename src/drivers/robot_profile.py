"""RobotProfile — typed dataclass for robot physical and configuration parameters.

Provides structured, importable robot specifications (kinematics, control limits,
safety, power, connection, sensors) with a simple in-memory registry.

Usage::

    from drivers.robot_profile import (
        THUNDER_V3_PROFILE, STUB_PROFILE,
        register_profile, get_profile, list_profiles,
    )
    register_profile(THUNDER_V3_PROFILE)
    p = get_profile("thunder_v3")
    print(p.control_limits.max_linear_speed)   # 1.0
    n = load_profiles_from_yaml("config/robot_config.yaml")
    p2 = RobotProfile.from_config({"robot": {"id": "custom"}, "speed": ...})
"""

from __future__ import annotations

from dataclasses import dataclass, field

import yaml

# ──────────────────────────────────────────────────────────────────────
# Nested dataclasses
# ──────────────────────────────────────────────────────────────────────


@dataclass
class RobotKinematics:
    """Physical dimensions and locomotion characteristics.

    locomotion_type: ``quadruped``, ``biped``, ``wheeled``, ``tracked``, ``none``.
    body_length/width/height: Vehicle dimensions (m).
    mass_kg: Total mass.
    max_slope_deg: Maximum traversable slope.
    ground_clearance_m: Minimum ground clearance.
    """

    locomotion_type: str = "quadruped"
    body_length: float = 1.0
    body_width: float = 0.6
    body_height: float = 0.5
    mass_kg: float = 55.0
    max_slope_deg: float = 30.0
    ground_clearance_m: float = 0.15


@dataclass
class ControlLimits:
    """Velocity, acceleration, and rate limits.

    max_linear_speed: Maximum forward/backward speed (m/s).
    max_angular_speed: Maximum yaw rate (rad/s).
    max_linear_accel: Maximum linear acceleration (m/s^2).
    max_decel: Maximum linear deceleration (m/s^2). Defaults to max_linear_accel.
    max_yaw_rate: Maximum yaw rate (deg/s), used by some planners.
    """

    max_linear_speed: float = 1.0
    max_angular_speed: float = 1.0
    max_linear_accel: float = 1.0
    max_decel: float = 1.0
    max_yaw_rate: float = 45.0


@dataclass
class SafetyLimits:
    """Safety thresholds for obstacle avoidance and deadman monitoring.

    stop_distance: Emergency stop trigger distance (m).
    slow_distance: Deceleration trigger distance (m).
    obstacle_height_threshold: Min obstacle height (m).
    tilt_limit_deg: Max chassis tilt.
    deadman_timeout_ms: Deadman timeout (ms).
    cmd_vel_timeout_ms: Velocity command watchdog (ms).
    """

    stop_distance: float = 0.8
    slow_distance: float = 2.0
    obstacle_height_threshold: float = 0.2
    tilt_limit_deg: float = 30.0
    deadman_timeout_ms: float = 300.0
    cmd_vel_timeout_ms: float = 200.0


@dataclass
class PowerSpec:
    """Battery specification and voltage thresholds.

    battery_capacity_wh: Total capacity (Wh).
    voltage_nominal/min/max/critical: Voltage levels.
    """

    battery_capacity_wh: float = 1000.0
    voltage_nominal: float = 48.0
    voltage_min: float = 42.0
    voltage_max: float = 54.6
    voltage_critical: float = 40.0


@dataclass
class ConnectionConfig:
    """Driver-level connection parameters.

    protocol: Transport (``grpc``, ``ros2``, ``serial``).
    host: Remote hostname or IP.
    port: Remote port.
    reconnect_interval_s: Reconnection interval.
    cmd_vel_timeout_ms: Velocity command watchdog (ms).
    auto_enable: Enable motors at startup.
    auto_standup: Stand robot at startup.
    control_rate_hz: Control loop frequency.
    """

    protocol: str = "grpc"
    host: str = "127.0.0.1"
    port: int = 13145
    reconnect_interval_s: float = 3.0
    cmd_vel_timeout_ms: float = 200.0
    auto_enable: bool = False
    auto_standup: bool = False
    control_rate_hz: float = 50.0


@dataclass
class MountSpec:
    """A single sensor with its extrinsics.

    id: Unique identifier (e.g. ``lidar_front``).
    type: ``lidar``, ``camera``, ``imu``, ``gnss``, ``depth``.
    model: Sensor model string.
    frame_id: ROS TF frame.
    x/y/z/roll/pitch/yaw: Extrinsic transform (m / rad, ZYX Euler).
    intrinsics: Sensor-specific parameters (fx/fy/cx/cy, publish_freq, ...).
    """

    id: str
    type: str
    model: str = ""
    frame_id: str = ""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    intrinsics: dict[str, float] | None = None


# ──────────────────────────────────────────────────────────────────────
# Top-level RobotProfile dataclass
# ──────────────────────────────────────────────────────────────────────


@dataclass
class RobotProfile:
    """Complete robot specification aggregating all sub-configs.

    id: Unique profile identifier (registry key).
    brand/model/description: Robot identity.
    kinematics/control_limits/safety_limits: Core specs.
    power/connection/sensors: Power, driver, and sensor config.
    """

    id: str
    brand: str = ""
    model: str = ""
    description: str = ""
    kinematics: RobotKinematics = field(default_factory=RobotKinematics)
    control_limits: ControlLimits = field(default_factory=ControlLimits)
    safety_limits: SafetyLimits = field(default_factory=SafetyLimits)
    power: PowerSpec = field(default_factory=PowerSpec)
    connection: ConnectionConfig = field(default_factory=ConnectionConfig)
    sensors: list[MountSpec] = field(default_factory=list)

    # ── composition ────────────────────────────────────────────────

    def merge_with(self, other: RobotProfile) -> RobotProfile:
        """Return a new profile with *other*'s non-empty fields overriding this one's.

        Sub-dataclasses are replaced whole; ``sensors`` is replaced if non-empty.
        """
        return RobotProfile(
            id=other.id or self.id,
            brand=other.brand or self.brand,
            model=other.model or self.model,
            description=other.description or self.description,
            kinematics=other.kinematics,
            control_limits=other.control_limits,
            safety_limits=other.safety_limits,
            power=other.power,
            connection=other.connection,
            sensors=other.sensors or self.sensors,
        )

    # ── factory ────────────────────────────────────────────────────

    @classmethod
    def from_config(cls, config: dict) -> RobotProfile:
        """Build a ``RobotProfile`` from a dict shaped like ``robot_config.yaml``.

        Fields not present fall back to dataclass defaults.
        """
        robot = config.get("robot", {})
        geometry = config.get("geometry", {})
        speed = config.get("speed", {})
        safety = config.get("safety", {})
        driver = config.get("driver", {})
        control = config.get("control", {})
        lidar_cfg = config.get("lidar", {})
        camera_cfg = config.get("camera", {})
        gnss_cfg = config.get("gnss", {})

        kinematics = RobotKinematics(
            body_length=geometry.get("vehicle_length", 1.0),
            body_width=geometry.get("vehicle_width", 0.6),
            body_height=geometry.get("vehicle_height", 0.5),
            mass_kg=geometry.get("mass_kg", 55.0),
            max_slope_deg=safety.get("tilt_limit_deg", 30.0),
            ground_clearance_m=geometry.get("ground_clearance_m", 0.15),
        )

        control_limits = ControlLimits(
            max_linear_speed=speed.get("max_linear", 1.0),
            max_angular_speed=speed.get("max_angular", 1.0),
            max_linear_accel=control.get("max_accel", 1.0),
            max_decel=control.get("max_decel", 1.0),
            max_yaw_rate=control.get("max_yaw_rate", 45.0),
        )

        safety_limits = SafetyLimits(
            stop_distance=safety.get("stop_distance", 0.8),
            slow_distance=safety.get("slow_distance", 2.0),
            obstacle_height_threshold=safety.get("obstacle_height_thre", 0.2),
            tilt_limit_deg=safety.get("tilt_limit_deg", 30.0),
            deadman_timeout_ms=safety.get("deadman_timeout_ms", 300.0),
            cmd_vel_timeout_ms=safety.get("cmd_vel_timeout_ms", 200.0),
        )

        connection = ConnectionConfig(
            protocol=driver.get("protocol", "grpc"),
            host=driver.get("dog_host", "127.0.0.1"),
            port=driver.get("dog_port", 13145),
            reconnect_interval_s=driver.get("reconnect_interval", 3.0),
            cmd_vel_timeout_ms=safety.get("cmd_vel_timeout_ms", 200.0),
            auto_enable=driver.get("auto_enable", False),
            auto_standup=driver.get("auto_standup", False),
            control_rate_hz=driver.get("control_rate", 50.0),
        )

        sensors: list[MountSpec] = []

        # Lidar
        sensors.append(MountSpec(
            id="lidar_front",
            type="lidar",
            model="Livox MID-360",
            frame_id=lidar_cfg.get("frame_id", "livox_frame"),
            x=lidar_cfg.get("offset_x", 0.0),
            y=lidar_cfg.get("offset_y", 0.0),
            z=lidar_cfg.get("offset_z", 0.0),
            roll=lidar_cfg.get("roll", 0.0),
            pitch=lidar_cfg.get("pitch", 0.0),
            yaw=lidar_cfg.get("yaw", 0.0),
            intrinsics={"publish_freq": lidar_cfg.get("publish_freq", 10.0)},
        ))

        # Camera
        cam_ints: dict[str, float] = {}
        for k in ("fx", "fy", "cx", "cy", "depth_scale",
                   "dist_k1", "dist_k2", "dist_k3", "dist_p1", "dist_p2",
                   "width", "height", "rotate"):
            v = camera_cfg.get(k)
            if v is not None:
                cam_ints[k] = v

        sensors.append(MountSpec(
            id="camera_front",
            type="camera",
            model="Depth Camera",
            frame_id="camera_link",
            x=camera_cfg.get("position_x", 0.15),
            y=camera_cfg.get("position_y", 0.0),
            z=camera_cfg.get("position_z", 0.45),
            roll=camera_cfg.get("roll", 0.0),
            pitch=camera_cfg.get("pitch", 0.0),
            yaw=camera_cfg.get("yaw", 0.0),
            intrinsics=cam_ints or None,
        ))

        # GNSS
        if gnss_cfg.get("enabled", False):
            ant = gnss_cfg.get("antenna_offset", {})
            sensors.append(MountSpec(
                id="gnss_main",
                type="gnss",
                model=gnss_cfg.get("model", "WTRTK-980"),
                frame_id="gnss_link",
                x=ant.get("x", 0.0),
                y=ant.get("y", 0.0),
                z=ant.get("z", 0.45),
            ))

        return cls(
            id=robot.get("id", "robot_001"),
            brand="Nova",
            model=robot.get("hw_id", "dog_v2"),
            description=robot.get("firmware_version", ""),
            kinematics=kinematics,
            control_limits=control_limits,
            safety_limits=safety_limits,
            connection=connection,
            sensors=sensors,
        )


# ──────────────────────────────────────────────────────────────────────
# Profile registry
# ──────────────────────────────────────────────────────────────────────

_PROFILES: dict[str, RobotProfile] = {}


def register_profile(profile: RobotProfile) -> None:
    """Register *profile* by its ``.id``, overwriting any existing entry."""
    _PROFILES[profile.id] = profile


def get_profile(name: str) -> RobotProfile:
    """Look up a registered profile by *name*.

    Raises:
        KeyError: If *name* is not registered.
    """
    if name not in _PROFILES:
        raise KeyError(
            f"Unknown robot profile {name!r}. Available: {sorted(_PROFILES)}"
        )
    return _PROFILES[name]


def list_profiles() -> list[str]:
    """Return sorted list of registered profile ids."""
    return sorted(_PROFILES)


def load_profiles_from_yaml(path: str) -> int:
    """Load a profile from a YAML file and register it.

    Returns:
        Number of profiles loaded (currently always 1).
    """
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError(
            f"Expected a dict at top level of {path}, got {type(data).__name__}"
        )

    profile = RobotProfile.from_config(data)
    register_profile(profile)
    return 1


# ──────────────────────────────────────────────────────────────────────
# Concrete robot profiles
# ──────────────────────────────────────────────────────────────────────

THUNDER_V3_PROFILE = RobotProfile(
    id="thunder_v3",
    brand="Nova",
    model="Dog V3",
    description="Thunder quadruped for the S100P platform (RDK X5, Nash BPU).",
    sensors=[
        MountSpec(
            id="lidar_front",
            type="lidar",
            model="Livox MID-360",
            frame_id="livox_frame",
            x=-0.011,
            y=-0.02329,
            z=0.04412,
            intrinsics={"publish_freq": 10.0},
        ),
        MountSpec(
            id="camera_front",
            type="camera",
            model="Depth Camera",
            frame_id="camera_link",
            x=0.15,
            y=0.0,
            z=0.45,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            intrinsics={
                "fx": 615.0, "fy": 615.0,
                "cx": 320.0, "cy": 240.0,
                "width": 640, "height": 480,
                "depth_scale": 0.001,
                "rotate": 270,
            },
        ),
        MountSpec(
            id="gnss_main",
            type="gnss",
            model="WTRTK-980",
            frame_id="gnss_link",
            z=0.45,
        ),
    ],
)

STUB_PROFILE = RobotProfile(
    id="stub",
    brand="Nova",
    model="Stub",
    description="Minimal profile for framework testing -- no hardware assumptions.",
    kinematics=RobotKinematics(
        locomotion_type="none",
        body_length=0.0,
        body_width=0.0,
        body_height=0.0,
        mass_kg=0.0,
        max_slope_deg=0.0,
        ground_clearance_m=0.0,
    ),
    control_limits=ControlLimits(
        max_linear_speed=0.0,
        max_angular_speed=0.0,
        max_linear_accel=0.0,
        max_decel=0.0,
        max_yaw_rate=0.0,
    ),
    safety_limits=SafetyLimits(
        stop_distance=0.0,
        slow_distance=0.0,
        obstacle_height_threshold=0.0,
        tilt_limit_deg=0.0,
        deadman_timeout_ms=0.0,
        cmd_vel_timeout_ms=0.0,
    ),
    connection=ConnectionConfig(
        protocol="none",
        host="",
        port=0,
        reconnect_interval_s=0.0,
        cmd_vel_timeout_ms=0.0,
        auto_enable=False,
        auto_standup=False,
        control_rate_hz=0.0,
    ),
    sensors=[],
)
