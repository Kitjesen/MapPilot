"""Sensor configuration — CameraConfig / LidarConfig / IMUConfig."""

from dataclasses import dataclass
from math import pi
from typing import Optional, Tuple

from runtime.runtime_interface import LIDAR_EXTRINSICS

_MUJOCO_LIDAR = LIDAR_EXTRINSICS["mujoco_thunder_v3"]


@dataclass
class CameraConfig:
    """Camera configuration (corresponds to MuJoCo camera element).

    MuJoCo camera parameter reference:
      https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera
    """

    name: str = "front_camera"  # camera name in MuJoCo XML
    width: int = 640  # render width px
    height: int = 480  # render height px
    fovy: float = 60.0  # vertical field of view degrees (MuJoCo default 45)
    render_depth: bool = True  # whether to render depth image
    depth_near: float = 0.1  # near clipping distance m
    depth_far: float = 10.0  # far clipping distance m
    fps: float = 30.0  # desired frame rate Hz

    @property
    def intrinsics(self) -> Tuple[float, float, float, float]:
        """Compute camera intrinsics (fx, fy, cx, cy).

        Simplified calculation based on MuJoCo fovy and resolution
        (equivalent to OpenCV pinhole model).
        """
        import math

        fy = self.height / (2.0 * math.tan(math.radians(self.fovy) / 2.0))
        # MuJoCo camera aspect ratio is determined by width/height; fovx matches automatically
        fovx = 2.0 * math.degrees(math.atan(math.tan(math.radians(self.fovy) / 2.0) * self.width / self.height))
        fx = self.width / (2.0 * math.tan(math.radians(fovx) / 2.0))
        cx = self.width / 2.0
        cy = self.height / 2.0
        return (fx, fy, cx, cy)


@dataclass
class LidarConfig:
    """LiDAR configuration (Livox MID-360 parameters).

    Reference: sim/sensors/livox_mid360.py parameter definitions.
    """

    body_name: str = _MUJOCO_LIDAR.child  # MuJoCo mounting body name
    sensor_name: str = "lidar_mid360"  # ray_caster plugin sensor name (method A)

    range_min: float = 0.10  # minimum valid range m
    # MID-360 reaches 70 m only on 80% targets. MuJoCo currently has no
    # calibrated material reflectivity model, so the nominal profile uses the
    # official 10%-reflectivity daylight range instead of granting 70 m to
    # every geometry.
    range_max: float = 40.0  # conservative nominal range m
    add_noise: bool = True  # whether to add range noise
    noise_std: float = 0.02  # radial range-noise standard deviation m
    range_noise_near_std_m: float = 0.03  # official envelope at 0.2 m
    range_noise_far_std_m: float = 0.02  # official envelope at 10 m
    range_noise_near_m: float = 0.2
    range_noise_far_m: float = 10.0
    angle_noise_std_rad: float = 0.10 / 180.0 * pi  # below official 1-sigma limit
    # Dropout is a stress-injection control, not a published MID-360 nominal.
    pixel_dropout_prob: float = 0.0
    distance_dropout_prob_at_max: float = 0.0
    # With no calibrated material model, assume a conservative 10% Lambertian
    # target: MID-360 maps 0..100% diffuse reflectivity into raw 0..150.
    intensity_base: float = 15.0
    intensity_range_scale_m: float = 25.0  # distance falloff scale
    intensity_noise_std: float = 0.0  # stress-only when non-zero
    intensity_min: float = 1.0
    intensity_max: float = 255.0
    fps: float = 10.0  # scan frequency Hz

    # Backend selection. Both supported backends produce real MuJoCo ray hits.
    backend: str = "auto"  # auto | mujoco_lidar | ray_caster_lidar
    mujoco_lidar_backend: str = "cpu"  # cpu | taichi | warp | jax
    require_product_backend: bool = False
    site_name: str = "lidar_site"  # MuJoCo site used by MuJoCo-LiDAR/plugin paths

    # OmniPerception real scan mode
    mid360_npy_path: Optional[str] = None  # if not None, use real scan mode
    samples_per_frame: int = 20000  # samples per frame (real scan mode)

    # Geom filter (only detect environment geoms, skip robot body)
    geom_group: int = 1  # MuJoCo geomgroup bitmask
    # Root of the complete robot subtree excluded from ray queries.  This is
    # intentionally separate from body_name: a sensor mounted on a child body
    # must not see the rest of its own robot. Appended to preserve the existing
    # positional dataclass surface.
    exclude_body_name: str | None = None


@dataclass
class DiscreteRayConfig:
    """Fixed-pattern ray/height observation configuration.

    This is intentionally separate from LiDAR. It models the IsaacLab-style
    terrain samples used by policies: a fixed body-frame grid/ring of points
    that casts rays, normally downward, and returns heights plus hit points.
    """

    pattern: str = "grid"
    x_min: float = -0.60
    x_max: float = 0.80
    x_count: int = 11
    y_min: float = -0.50
    y_max: float = 0.50
    y_count: int = 11
    ray_start_z: float = 0.60
    ray_length: float = 3.0
    direction_body: tuple[float, float, float] = (0.0, 0.0, -1.0)
    geom_group_mask: tuple[int, ...] = (0, 1)


@dataclass
class IMUConfig:
    """IMU configuration (extracted from MuJoCo freejoint state).

    Output format consistent with brainstem ImuService.
    """

    body_name: str = "base_link"  # IMU mounting body (typically base_link)
    gyro_scale: float = 0.25  # gyroscope scale factor (matches brainstem)
    freq_hz: float = 200.0  # IMU output frequency (readable every step in MuJoCo)
    add_noise: bool = False  # whether to add IMU noise
    gyro_noise_std: float = 0.01  # gyroscope noise rad/s
    accel_noise_std: float = 0.05  # accelerometer noise m/s^2
