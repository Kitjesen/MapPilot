"""Calibrated simulation rays. Camera transforms never mutate physics bodies.

Optical coordinates are x right, y down, z forward. Pixel coordinates refer to
image edges: the centre of pixel [row, column] is (column + .5, row + .5).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from .samples import SensorSampleStamp


def _finite(values: object, size: int, name: str) -> None:
    array = np.asarray(values, dtype=float)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f"{name} must contain {size} finite values")


def _image_size(width: int, height: int) -> None:
    if type(width) is not int or type(height) is not int or min(width, height) <= 0:
        raise ValueError("image width and height must be positive integers")


@dataclass(frozen=True, slots=True)
class PinholeCalibration:
    width: int
    height: int
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float

    def __post_init__(self) -> None:
        _image_size(self.width, self.height)
        _finite((self.fx_px, self.fy_px, self.cx_px, self.cy_px), 4, "pinhole calibration")
        if min(self.fx_px, self.fy_px) <= 0:
            raise ValueError("pinhole focal lengths must be positive")


@dataclass(frozen=True, slots=True)
class FisheyeCalibration:
    """Equidistant lens r=f*theta, bounded by both FOV and a calibrated disk."""

    width: int
    height: int
    focal_px: float
    cx_px: float
    cy_px: float
    fov_deg: float
    valid_radius_px: float

    def __post_init__(self) -> None:
        _image_size(self.width, self.height)
        _finite((self.focal_px, self.cx_px, self.cy_px, self.fov_deg, self.valid_radius_px), 5, "fisheye calibration")
        if min(self.focal_px, self.valid_radius_px) <= 0 or not 0 < self.fov_deg < 360:
            raise ValueError("fisheye focal length/radius must be positive and FOV must be in (0, 360) degrees")


@dataclass(frozen=True, slots=True)
class PanoramaCalibration:
    """Equirectangular angular bounds, with the upper latitude at image top."""

    width: int
    height: int
    longitude_min_deg: float = -180
    longitude_max_deg: float = 180
    latitude_min_deg: float = -90
    latitude_max_deg: float = 90

    def __post_init__(self) -> None:
        _image_size(self.width, self.height)
        _finite(
            (self.longitude_min_deg, self.longitude_max_deg, self.latitude_min_deg, self.latitude_max_deg),
            4,
            "panorama bounds",
        )
        if not 0 < self.longitude_max_deg - self.longitude_min_deg <= 360:
            raise ValueError("panorama longitude span must be in (0, 360] degrees")
        if not -90 <= self.latitude_min_deg < self.latitude_max_deg <= 90:
            raise ValueError("panorama latitude bounds must be ordered within [-90, 90] degrees")


CameraCalibration = PinholeCalibration | FisheyeCalibration | PanoramaCalibration


@dataclass(frozen=True, slots=True)
class CameraPose:
    """Child-to-parent rigid transform, using a wxyz quaternion."""

    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        _finite(self.position_m, 3, "position_m")
        _finite(self.orientation_wxyz, 4, "orientation_wxyz")
        if not any(self.orientation_wxyz):
            raise ValueError("orientation_wxyz must be nonzero")


@dataclass(frozen=True, slots=True)
class GimbalPose:
    """Pan right about optical +y, then tilt up about the panned optical +x."""

    pan_deg: float = 0.0
    tilt_deg: float = 0.0
    pan_limits_deg: tuple[float, float] = (-180.0, 180.0)
    tilt_limits_deg: tuple[float, float] = (-90.0, 90.0)

    def __post_init__(self) -> None:
        _finite((self.pan_deg, self.tilt_deg), 2, "gimbal angles")
        for angle, limits, name in (
            (self.pan_deg, self.pan_limits_deg, "pan"),
            (self.tilt_deg, self.tilt_limits_deg, "tilt"),
        ):
            _finite(limits, 2, f"{name}_limits_deg")
            if not limits[0] <= angle <= limits[1]:
                raise ValueError(f"{name} angle must lie within its ordered limits")


def _rotation(pose: CameraPose) -> np.ndarray:
    w, x, y, z = np.asarray(pose.orientation_wxyz, dtype=float) / np.linalg.norm(pose.orientation_wxyz)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _quaternion_product(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def compose_camera_pose(base_pose: CameraPose, mount_pose: CameraPose, gimbal: GimbalPose) -> CameraPose:
    """Compose base * mount * pan * tilt without modifying authoritative poses."""
    pan, tilt = np.deg2rad([gimbal.pan_deg, gimbal.tilt_deg]) / 2
    base_q = np.asarray(base_pose.orientation_wxyz) / np.linalg.norm(base_pose.orientation_wxyz)
    mount_q = np.asarray(mount_pose.orientation_wxyz) / np.linalg.norm(mount_pose.orientation_wxyz)
    orientation = _quaternion_product(
        _quaternion_product(_quaternion_product(base_q, mount_q), np.array([np.cos(pan), 0, np.sin(pan), 0])),
        np.array([np.cos(tilt), np.sin(tilt), 0, 0]),
    )
    position = np.asarray(base_pose.position_m) + _rotation(base_pose) @ mount_pose.position_m
    return CameraPose(tuple(position), tuple(orientation))


@dataclass(frozen=True, slots=True)
class CameraRaySample:
    stamp: SensorSampleStamp
    origins_m: np.ndarray
    directions_world: np.ndarray
    valid_mask: np.ndarray
    calibration_metadata: dict
    simulation: bool = True


def project_local_directions(
    calibration: CameraCalibration, directions_local: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Project optical-frame directions to edge-based (u,v), returning validity.

    Full panoramas wrap the back-facing seam onto the left image edge. Invalid
    directions (including zero/NaN rays and out-of-image projections) return zero.
    """
    directions = np.asarray(directions_local, dtype=float)
    if directions.ndim < 1 or directions.shape[-1] != 3:
        raise ValueError("directions_local must have a final xyz axis")
    norm = np.linalg.norm(directions, axis=-1)
    valid = np.isfinite(directions).all(axis=-1) & (norm > 0)
    unit = np.divide(directions, norm[..., None], out=np.zeros_like(directions), where=valid[..., None])
    x, y, z = np.moveaxis(unit, -1, 0)
    if isinstance(calibration, PanoramaCalibration):
        longitude = np.rad2deg(np.arctan2(x, z))
        longitude = (longitude - calibration.longitude_min_deg) % 360 + calibration.longitude_min_deg
        latitude = np.rad2deg(np.arctan2(-y, np.hypot(x, z)))
        u = (
            (longitude - calibration.longitude_min_deg)
            / (calibration.longitude_max_deg - calibration.longitude_min_deg)
            * calibration.width
        )
        v = (
            (calibration.latitude_max_deg - latitude)
            / (calibration.latitude_max_deg - calibration.latitude_min_deg)
            * calibration.height
        )
    elif isinstance(calibration, FisheyeCalibration):
        transverse = np.hypot(x, y)
        theta = np.arctan2(transverse, z)
        radius = theta * calibration.focal_px
        scale = np.divide(radius, transverse, out=np.zeros_like(radius), where=transverse > 0)
        u, v = x * scale + calibration.cx_px, y * scale + calibration.cy_px
        valid &= (radius <= calibration.valid_radius_px + 1e-12) & (
            theta <= np.deg2rad(calibration.fov_deg / 2) + 1e-12
        )
    else:
        valid &= z > 0
        u = np.divide(x, z, out=np.zeros_like(x), where=z != 0) * calibration.fx_px + calibration.cx_px
        v = np.divide(y, z, out=np.zeros_like(y), where=z != 0) * calibration.fy_px + calibration.cy_px
    valid &= (u >= 0) & (u < calibration.width) & (v >= 0) & (v < calibration.height)
    uv = np.stack((u, v), axis=-1)
    return np.where(valid[..., None], uv, 0), valid


def calibrated_rays(
    calibration: CameraCalibration,
    stamp: SensorSampleStamp,
    *,
    base_pose: CameraPose = CameraPose(),
    mount_pose: CameraPose = CameraPose(),
    gimbal: GimbalPose = GimbalPose(),
) -> CameraRaySample:
    """Generate one ray per pixel at the caller's simulation sample stamp."""
    if not isinstance(stamp, SensorSampleStamp):
        raise ValueError("stamp must be a SensorSampleStamp")
    u, v = np.meshgrid(np.arange(calibration.width) + 0.5, np.arange(calibration.height) + 0.5)
    valid = np.ones(u.shape, dtype=bool)
    if isinstance(calibration, PanoramaCalibration):
        longitude = np.deg2rad(
            calibration.longitude_min_deg
            + u / calibration.width * (calibration.longitude_max_deg - calibration.longitude_min_deg)
        )
        latitude = np.deg2rad(
            calibration.latitude_max_deg
            - v / calibration.height * (calibration.latitude_max_deg - calibration.latitude_min_deg)
        )
        directions = np.stack(
            (np.cos(latitude) * np.sin(longitude), -np.sin(latitude), np.cos(latitude) * np.cos(longitude)), axis=-1
        )
        model = "equirectangular"
    elif isinstance(calibration, FisheyeCalibration):
        x, y = u - calibration.cx_px, v - calibration.cy_px
        radius = np.hypot(x, y)
        theta = radius / calibration.focal_px
        scale = np.divide(np.sin(theta), radius, out=np.zeros_like(theta), where=radius > 0)
        directions = np.stack((x * scale, y * scale, np.cos(theta)), axis=-1)
        valid = (radius <= calibration.valid_radius_px) & (theta <= np.deg2rad(calibration.fov_deg / 2))
        directions[~valid] = 0
        model = "equidistant_fisheye"
    else:
        directions = np.stack(
            ((u - calibration.cx_px) / calibration.fx_px, (v - calibration.cy_px) / calibration.fy_px, np.ones_like(u)),
            axis=-1,
        )
        directions /= np.linalg.norm(directions, axis=-1, keepdims=True)
        model = "pinhole"
    projection = {"model": model, **asdict(calibration)}
    if isinstance(calibration, PanoramaCalibration):
        projection["angle_convention"] = (
            "longitude=atan2(x,z),latitude=atan2(-y,hypot(x,z)); u increases longitude,v decreases latitude"
        )
    camera_pose = compose_camera_pose(base_pose, mount_pose, gimbal)
    world_directions = directions @ _rotation(camera_pose).T
    return CameraRaySample(
        stamp,
        np.broadcast_to(camera_pose.position_m, directions.shape).copy(),
        world_directions,
        valid,
        {
            "simulation": True,
            "projection": projection,
            "optical_axes": {"x": "right", "y": "down", "z": "forward"},
            "pixel_coordinates": "u=column+0.5,v=row+0.5; image edges at 0,width and 0,height",
            "transform_composition": "base_pose * mount_pose * Ry(pan) * Rx(tilt)",
            "gimbal": asdict(gimbal),
            "mount_pose": asdict(mount_pose),
            "base_pose": asdict(base_pose),
            "camera_pose_world": asdict(camera_pose),
        },
    )
