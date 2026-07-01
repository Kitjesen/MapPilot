"""Sensor configuration — CameraConfig / LidarConfig / IMUConfig."""
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from runtime.runtime_interface import LIDAR_EXTRINSICS


_MUJOCO_LIDAR = LIDAR_EXTRINSICS["mujoco_thunder_v3"]


@dataclass
class CameraConfig:
    """Camera configuration (corresponds to MuJoCo camera element).

    MuJoCo camera parameter reference:
      https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera
    """

    name: str = "front_camera"      # camera name in MuJoCo XML
    width: int = 640                # render width px
    height: int = 480               # render height px
    fovy: float = 60.0              # vertical field of view degrees (MuJoCo default 45)
    render_depth: bool = True       # whether to render depth image
    depth_near: float = 0.1         # near clipping distance m
    depth_far: float = 10.0         # far clipping distance m
    fps: float = 30.0               # desired frame rate Hz

    @property
    def intrinsics(self) -> Tuple[float, float, float, float]:
        """Compute camera intrinsics (fx, fy, cx, cy).

        Simplified calculation based on MuJoCo fovy and resolution
        (equivalent to OpenCV pinhole model).
        """
        import math
        fy = self.height / (2.0 * math.tan(math.radians(self.fovy) / 2.0))
        # MuJoCo camera aspect ratio is determined by width/height; fovx matches automatically
        fovx = 2.0 * math.degrees(math.atan(math.tan(math.radians(self.fovy) / 2.0)
                                             * self.width / self.height))
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

    # Scan parameters (method B fallback)
    n_rays: int = 6400              # rays per frame (real MID-360 ~20000/frame)
    vfov_min_deg: float = -7.0      # vertical FoV lower bound degrees
    vfov_max_deg: float = 52.0      # vertical FoV upper bound degrees
    range_min: float = 0.10         # minimum valid range m
    range_max: float = 70.0         # maximum valid range m
    add_noise: bool = True          # whether to add range noise
    noise_std: float = 0.02         # range noise standard deviation m
    fps: float = 10.0               # scan frequency Hz

    # Backend selection. Strict/product gates should request `mujoco_lidar` or
    # `ray_caster_lidar` and set `require_mature_backend=True` so legacy
    # mj_multiRay cannot silently pass as a production LiDAR backend.
    backend: str = "auto"  # auto | mujoco_lidar | ray_caster_lidar | mj_multiray
    mujoco_lidar_backend: str = "cpu"  # cpu | taichi | warp | jax
    require_mature_backend: bool = False
    allow_legacy_fallback: bool = True
    site_name: str = "lidar_site"  # MuJoCo site used by MuJoCo-LiDAR/plugin paths

    # OmniPerception real scan mode
    mid360_npy_path: Optional[str] = None  # if not None, use real scan mode
    samples_per_frame: int = 20000          # samples per frame (real scan mode)

    # Geom filter (only detect environment geoms, skip robot body)
    geom_group: int = 1             # MuJoCo geomgroup bitmask


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

    body_name: str = "base_link"    # IMU mounting body (typically base_link)
    gyro_scale: float = 0.25        # gyroscope scale factor (matches brainstem)
    freq_hz: float = 200.0          # IMU output frequency (readable every step in MuJoCo)
    add_noise: bool = False         # whether to add IMU noise
    gyro_noise_std: float = 0.01    # gyroscope noise rad/s
    accel_noise_std: float = 0.05   # accelerometer noise m/s^2
