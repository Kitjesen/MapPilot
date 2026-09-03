"""Unified configuration loader.

Reads the selected robot model's RobotConfig.
Modules access config via: cfg = load_config(); cfg.speed.max_speed

ProductControl sets ``LINGTU_CONFIG_PATH`` for a selected real robot. Without
an explicit path, this module returns neutral development defaults.
"""

from __future__ import annotations

import ipaddress
import math
import os
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import numpy as np

import yaml

from .msgs.numpy_compat import np, numpy_import_is_safe


@dataclass
class SpeedConfig:
    """Velocity limits for the robot."""

    max_linear: float = 1.0
    max_angular: float = 1.0
    max_speed: float = 0.875
    autonomy_speed: float = 0.875


@dataclass
class GeometryConfig:
    """Physical dimensions and sensor offsets."""

    vehicle_height: float = 0.5
    vehicle_width: float = 0.6
    vehicle_length: float = 1.0
    collision_cylinder_radius: float = 0.40
    collision_cylinder_offset: float = 0.25
    collision_hard_margin: float = 0.15
    collision_clearance_below: float = 0.25
    collision_clearance_above: float = 0.35
    sensor_offset_x: float = 0.3
    sensor_offset_y: float = 0.0


@dataclass
class DriverConfig:
    """Physical robot driver selected by the real environment."""

    backend: str = ""
    target: str = ""
    network_interface: str = ""
    network_address: str = ""
    probe_ip: str = ""
    dog_host: str = "127.0.0.1"
    dog_port: int = 13145
    control_rate: float = 50.0
    auto_enable: bool = False
    auto_standup: bool = False
    reconnect_interval: float = 3.0
    slam_reset_interval: float = 5.0
    tls_ca_file: str = ""
    tls_cert_file: str = ""
    tls_key_file: str = ""
    tls_server_name: str = ""


@dataclass
class SafetyConfig:
    """Safety thresholds and timeouts."""

    obstacle_height_thre: float = 0.2
    ground_height_thre: float = 0.1
    stop_distance: float = 0.8
    slow_distance: float = 2.0
    deadman_timeout_ms: float = 300.0
    cmd_vel_timeout_ms: float = 200.0
    tilt_limit_deg: float = 30.0


@dataclass
class CameraConfig:
    """Camera mounting extrinsics and default intrinsics.

    Extrinsics define the body→camera static transform (position + rotation).
    Intrinsics are factory defaults; runtime CameraInfo from ROS2 overrides them.
    Distortion uses Brown-Conrady (plumb_bob) model: k1, k2, p1, p2, k3.
    """

    # Extrinsics: camera position in body frame (m)
    position_x: float = 0.15
    position_y: float = 0.0
    position_z: float = 0.45
    # Extrinsics: camera rotation relative to body forward (rad)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    # Default intrinsics (overridden by ROS2 CameraInfo at runtime)
    fx: float = 615.0
    fy: float = 615.0
    cx: float = 320.0
    cy: float = 240.0
    width: int = 640
    height: int = 480
    depth_scale: float = 0.001
    # Image rotation to apply (degrees: 0, 90, 180, 270).
    # When the camera is mounted sideways the image is rotated to upright
    # and intrinsics (fx/fy/cx/cy/width/height) are swapped accordingly
    # so downstream 3D projection stays correct. Default 0 = no rotation.
    rotate: int = 0
    # Distortion coefficients (Brown-Conrady / plumb_bob)
    dist_k1: float = 0.0
    dist_k2: float = 0.0
    dist_p1: float = 0.0
    dist_p2: float = 0.0
    dist_k3: float = 0.0

    @property
    def T_camera_body(self) -> np.ndarray | list[list[float]]:
        """4x4 camera→body static transform from factory calibration.

        Builds rotation from ZYX Euler angles (yaw, pitch, roll in radians).
        Translation is the camera position in body frame.
        Pure numpy — no cv2 dependency.
        """
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        # ZYX extrinsic convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        rows = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
        if not numpy_import_is_safe():
            return [
                [rows[0][0], rows[0][1], rows[0][2], self.position_x],
                [rows[1][0], rows[1][1], rows[1][2], self.position_y],
                [rows[2][0], rows[2][1], rows[2][2], self.position_z],
                [0.0, 0.0, 0.0, 1.0],
            ]
        R = np.array(rows, dtype=np.float64)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [self.position_x, self.position_y, self.position_z]
        return T


@dataclass
class LidarConfig:
    """LiDAR sensor mounting extrinsics and publish settings.

    offset_x/y/z + roll/pitch/yaw define the body→lidar static transform
    used by static_transform_publisher and C++ coordinate transforms.
    """

    frame_id: str = "livox_frame"
    publish_freq: float = 10.0
    # Network (Livox MID-360). These are consumed by the Livox driver JSON generator.
    network_interface: str = ""
    lidar_ip: str = ""
    host_ip: str = ""
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    camera_offset_z: float = 0.0


@dataclass
class GnssAntennaOffset:
    """Antenna mounting offset in body frame (metres)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class GnssQualityConfig:
    """Quality settings consumed by the native GNSS/localization path."""

    min_sat_used: int = 8
    max_hdop: float = 2.5
    max_age_s: float = 2.0
    require_fix_type: int = 1
    allow_float: bool = True


@dataclass
class GnssFusionRuntime:
    """Runtime fusion parameters consumed by the selected SLAM implementation.

    Keys are passed with a ``gnss_`` prefix by the SLAM stack.
    """

    backend: str = "fastlio2_gnss"  # reserved for future factor-graph backends
    factor_weight_fix: float = 1.0
    factor_weight_float: float = 0.3
    factor_weight_single: float = 0.05
    enabled: bool = False
    alpha_healthy: float = 0.05
    alpha_degraded: float = 0.5
    rtk_float_scale: float = 0.3
    max_age_s: float = 2.0
    max_std_m: float = 1.0
    residual_warn_m: float = 5.0
    residual_warn_duration_s: float = 10.0
    residual_warn_ratio: float = 0.7


@dataclass
class GnssConfig:
    """Typed view of the ``gnss:`` section of robot_config.yaml.

    Non-typed sub-sections (origin, rtcm, topic_fix, etc.) stay in
    ``RobotConfig.raw['gnss']`` for loose access. This config owns the
    safety-critical bits (antenna offset, quality gate, fusion weights).
    """

    enabled: bool = False
    model: str = "WTRTK-980"
    device: str = "/dev/wtrtk980"
    baud: int = 115200
    antenna_offset: GnssAntennaOffset = field(default_factory=GnssAntennaOffset)
    quality: GnssQualityConfig = field(default_factory=GnssQualityConfig)
    fusion: GnssFusionRuntime = field(default_factory=GnssFusionRuntime)


@dataclass
class TrackingConfig:
    """3D instance-tracking fusion thresholds (USS-Nav style matching)."""

    sem_threshold: float = 0.75
    geo_weak_threshold: float = 0.1
    geo_strong_threshold: float = 0.5
    geo_point_dist_tau: float = 0.05
    candidate_radius: float = 2.0
    fov_half_angle: float = 0.52
    fov_max_range: float = 5.0
    merge_distance: float = 0.5
    iou_threshold: float = 0.3
    clip_threshold: float = 0.75
    max_objects: int = 200
    stale_timeout: float = 300.0
    max_views: int = 300

    # Stage 1a matching improvements (gated, default OFF to preserve behavior)
    use_hungarian_matching: bool = False  # global-optimal Hungarian matching
    enable_dedup_merge: bool = False  # post-matching duplicate merge
    dedup_distance: float = 0.3  # m — max distance to consider a duplicate
    dedup_clip_threshold: float = 0.85  # min CLIP cosine sim to merge
    dedup_time_window: float = 5.0  # s — max last_seen diff to merge


@dataclass
class EncoderConfig:
    """Image/text encoder runtime parameters."""

    clip_cache_size: int = 1000
    clip_batch_size: int = 32
    clip_model_name: str = "ViT-B/32"
    clip_multi_scale: bool = False


@dataclass
class DetectorConfig:
    """Detector-related runtime defaults."""

    confidence_threshold: float = 0.3
    iou_threshold: float = 0.45
    max_detections: int = 64
    min_box_size_px: int = 12
    model_size: str = "l"
    device: str = ""
    model_path: str = ""


@dataclass
class PerceptionConfig:
    """Perception pipeline tunables.

    Defaults mirror the values consumed by perception.module,
    instance_tracker.py, clip_encoder.py, and reconstruction_module.py so
    behavior is unchanged when no override is present in robot_config.yaml.
    """

    default_classes: str = "door . chair . person . desk . stairs . elevator . sign"
    skip_frames: int = 1
    max_depth: float = 6.0
    min_depth: float = 0.3
    laplacian_threshold: float = 100.0
    max_rgbd_skew_s: float = 0.05
    max_odom_age_s: float = 0.10
    max_map_odom_age_s: float = 0.50
    dynamic_labels: frozenset[str] = field(
        default_factory=lambda: frozenset(
            {
                "person",
                "people",
                "pedestrian",
                "car",
                "vehicle",
                "truck",
                "bus",
                "bicycle",
                "motorcycle",
                "dog",
                "cat",
                "animal",
            }
        )
    )
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    encoder: EncoderConfig = field(default_factory=EncoderConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)


@dataclass
class RobotConfig:
    """Top-level physical robot configuration.

    Sub-configs hold typed fields for the most commonly accessed sections.
    The full parsed YAML dict is available via ``raw`` for sections not
    explicitly modelled.
    """

    speed: SpeedConfig = field(default_factory=SpeedConfig)
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    driver: DriverConfig = field(default_factory=DriverConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    gnss: GnssConfig = field(default_factory=GnssConfig)
    perception: PerceptionConfig = field(default_factory=PerceptionConfig)
    raw: dict[str, Any] = field(default_factory=dict)


def _fill_dataclass(cls, data: dict[str, Any]):
    """Instantiate a dataclass, ignoring keys not present in its fields."""
    field_names = {f.name for f in cls.__dataclass_fields__.values()}
    filtered = {k: v for k, v in data.items() if k in field_names}
    return cls(**filtered)


def _fill_gnss_config(data: dict[str, Any]) -> GnssConfig:
    """Build GnssConfig from the nested gnss section of robot_config.yaml.
    Nested antenna_offset / quality / fusion subsections are filled
    recursively; unknown keys are tolerated (forward-compat)."""
    if not isinstance(data, dict):
        return GnssConfig()
    antenna = _fill_dataclass(GnssAntennaOffset, data.get("antenna_offset") or {})
    quality = _fill_dataclass(GnssQualityConfig, data.get("quality") or {})
    fusion = _fill_dataclass(GnssFusionRuntime, data.get("fusion") or {})
    return GnssConfig(
        enabled=bool(data.get("enabled", False)),
        model=str(data.get("model", "WTRTK-980")),
        device=str(data.get("device", "/dev/wtrtk980")),
        baud=int(data.get("baud", 115200)),
        antenna_offset=antenna,
        quality=quality,
        fusion=fusion,
    )


def _fill_perception_config(data: dict[str, Any]) -> PerceptionConfig:
    """Build PerceptionConfig from the nested perception section of robot_config.yaml.

    Unknown keys are ignored; missing nested sections fall back to typed defaults.
    """
    if not isinstance(data, dict):
        data = {}

    tracking = _fill_dataclass(TrackingConfig, data.get("tracking", {}))
    encoder = _fill_dataclass(EncoderConfig, data.get("encoder", {}))
    detector = _fill_dataclass(DetectorConfig, data.get("detector", {}))

    default_dynamic_labels = frozenset(
        {
            "person",
            "people",
            "pedestrian",
            "car",
            "vehicle",
            "truck",
            "bus",
            "bicycle",
            "motorcycle",
            "dog",
            "cat",
            "animal",
        }
    )
    dyn_labels = data.get("dynamic_labels")
    dynamic_labels = frozenset(dyn_labels) if isinstance(dyn_labels, (list, tuple, set)) else default_dynamic_labels

    return PerceptionConfig(
        default_classes=str(data.get("default_classes", "door . chair . person . desk . stairs . elevator . sign")),
        skip_frames=int(data.get("skip_frames", 1)),
        max_depth=float(data.get("max_depth", 6.0)),
        min_depth=float(data.get("min_depth", 0.3)),
        laplacian_threshold=float(data.get("laplacian_threshold", 100.0)),
        max_rgbd_skew_s=float(data.get("max_rgbd_skew_s", 0.05)),
        max_odom_age_s=float(data.get("max_odom_age_s", 0.10)),
        max_map_odom_age_s=float(data.get("max_map_odom_age_s", 0.50)),
        dynamic_labels=dynamic_labels,
        tracking=tracking,
        encoder=encoder,
        detector=detector,
    )


def load_config(path: str | None = None) -> RobotConfig:
    """Load robot config from YAML.

    Resolution order for the config file path:
    1. Explicit ``path`` argument.
    2. ``LINGTU_CONFIG_PATH`` environment variable.

    Returns a neutral ``RobotConfig`` when neither path is set or the selected
    file is missing.
    """
    config_path = path or os.environ.get("LINGTU_CONFIG_PATH")

    if config_path:
        try:
            with open(config_path, encoding="utf-8") as fh:
                raw = yaml.safe_load(fh) or {}
        except (FileNotFoundError, OSError):
            raw = {}
    else:
        raw = {}

    cfg = RobotConfig(
        speed=_fill_dataclass(SpeedConfig, raw.get("speed", {})),
        geometry=_fill_dataclass(GeometryConfig, raw.get("geometry", {})),
        driver=_fill_dataclass(DriverConfig, raw.get("driver", {})),
        safety=_fill_dataclass(SafetyConfig, raw.get("safety", {})),
        lidar=_fill_dataclass(LidarConfig, raw.get("lidar", {})),
        camera=_fill_dataclass(CameraConfig, raw.get("camera", {})),
        gnss=_fill_gnss_config(raw.get("gnss", {})),
        perception=_fill_perception_config(raw.get("perception", {})),
        raw=raw,
    )

    errors = validate_config(cfg)
    if errors:
        import logging

        _logger = logging.getLogger(__name__)
        for err in errors:
            _logger.warning("Config validation: %s", err)

    return cfg


def validate_config(cfg: RobotConfig) -> list[str]:
    """Validate robot config, return list of error messages (empty = OK).

    Checks required ranges for safety-critical parameters.
    """
    errors = []

    # Speed limits must be positive
    if cfg.speed.max_linear <= 0:
        errors.append(f"speed.max_linear must be > 0, got {cfg.speed.max_linear}")
    if cfg.speed.max_angular <= 0:
        errors.append(f"speed.max_angular must be > 0, got {cfg.speed.max_angular}")

    # Geometry must be positive
    if cfg.geometry.vehicle_width <= 0:
        errors.append(f"geometry.vehicle_width must be > 0, got {cfg.geometry.vehicle_width}")
    if cfg.geometry.vehicle_length <= 0:
        errors.append(f"geometry.vehicle_length must be > 0, got {cfg.geometry.vehicle_length}")
    if cfg.geometry.vehicle_height <= 0:
        errors.append(f"geometry.vehicle_height must be > 0, got {cfg.geometry.vehicle_height}")
    if cfg.geometry.collision_cylinder_radius <= 0:
        errors.append(
            "geometry.collision_cylinder_radius must be > 0, got "
            f"{cfg.geometry.collision_cylinder_radius}"
        )
    if cfg.geometry.collision_cylinder_offset < 0:
        errors.append(
            "geometry.collision_cylinder_offset must be >= 0, got "
            f"{cfg.geometry.collision_cylinder_offset}"
        )
    if cfg.geometry.collision_hard_margin < 0:
        errors.append(
            "geometry.collision_hard_margin must be >= 0, got "
            f"{cfg.geometry.collision_hard_margin}"
        )
    if cfg.geometry.collision_clearance_below < 0 or cfg.geometry.collision_clearance_above < 0:
        errors.append("geometry collision clearances must be non-negative")

    # Safety distances must be positive and ordered
    if cfg.safety.stop_distance <= 0:
        errors.append(f"safety.stop_distance must be > 0, got {cfg.safety.stop_distance}")
    if cfg.safety.slow_distance <= cfg.safety.stop_distance:
        errors.append(
            f"safety.slow_distance ({cfg.safety.slow_distance}) must be > stop_distance ({cfg.safety.stop_distance})"
        )

    # The native loop must be able to execute its 100 ms control-lease refresh.
    if cfg.driver.backend and cfg.driver.backend not in {"go2", "doso"}:
        errors.append(
            "driver.backend must be one of go2, doso, "
            f"got {cfg.driver.backend!r}"
        )
    if cfg.driver.backend == "go2":
        if not cfg.driver.network_interface.strip():
            errors.append("driver.network_interface is required for the go2 backend")
        network = None
        try:
            network = ipaddress.ip_interface(cfg.driver.network_address)
            if network.version != 4:
                raise ValueError
        except ValueError:
            errors.append(
                "driver.network_address must be an IPv4 interface CIDR for the go2 backend"
            )
        probe = None
        try:
            probe = ipaddress.ip_address(cfg.driver.probe_ip)
            if probe.version != 4:
                raise ValueError
        except ValueError:
            errors.append("driver.probe_ip must be an IPv4 address for the go2 backend")
        if network is not None and probe is not None and probe not in network.network:
            errors.append("driver.probe_ip must be in the same subnet as driver.network_address")
    if not math.isfinite(cfg.driver.control_rate) or cfg.driver.control_rate < 10.0:
        errors.append(f"driver.control_rate must be finite and >= 10 Hz, got {cfg.driver.control_rate}")

    # GNSS — only check when enabled; unset robots skip these
    if cfg.gnss.enabled:
        if cfg.gnss.quality.min_sat_used < 0:
            errors.append(f"gnss.quality.min_sat_used must be ≥ 0, got {cfg.gnss.quality.min_sat_used}")
        if cfg.gnss.quality.max_hdop <= 0:
            errors.append(f"gnss.quality.max_hdop must be > 0, got {cfg.gnss.quality.max_hdop}")
        if cfg.gnss.quality.max_age_s <= 0:
            errors.append(f"gnss.quality.max_age_s must be > 0, got {cfg.gnss.quality.max_age_s}")
        if not 0.0 <= cfg.gnss.fusion.alpha_healthy <= 1.0:
            errors.append(f"gnss.fusion.alpha_healthy must be in [0, 1], got {cfg.gnss.fusion.alpha_healthy}")
        if not 0.0 <= cfg.gnss.fusion.alpha_degraded <= 1.0:
            errors.append(f"gnss.fusion.alpha_degraded must be in [0, 1], got {cfg.gnss.fusion.alpha_degraded}")
        if cfg.gnss.fusion.residual_warn_m <= 0:
            errors.append(f"gnss.fusion.residual_warn_m must be > 0, got {cfg.gnss.fusion.residual_warn_m}")
        if not 0.0 < cfg.gnss.fusion.residual_warn_ratio <= 1.0:
            errors.append(
                f"gnss.fusion.residual_warn_ratio must be in (0, 1], got {cfg.gnss.fusion.residual_warn_ratio}"
            )

    # Camera intrinsics must be positive
    if cfg.camera.fx <= 0 or cfg.camera.fy <= 0:
        errors.append(f"camera.fx/fy must be > 0, got fx={cfg.camera.fx}, fy={cfg.camera.fy}")
    if cfg.camera.width <= 0 or cfg.camera.height <= 0:
        errors.append(f"camera.width/height must be > 0, got {cfg.camera.width}x{cfg.camera.height}")

    return errors


# ---------------------------------------------------------------------------
# Singleton accessor
# ---------------------------------------------------------------------------

_config: RobotConfig | None = None


def get_config() -> RobotConfig:
    """Return the singleton ``RobotConfig``, loading on first call."""
    global _config
    if _config is None:
        _config = load_config()
    return _config


def reset_config() -> None:
    """Clear the singleton so the next ``get_config()`` reloads from disk.

    Primarily useful for tests.
    """
    global _config
    _config = None
