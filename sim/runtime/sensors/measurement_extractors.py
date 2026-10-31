"""Owner-thread measurement endpoints bound to one leased physics snapshot."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from contextlib import AbstractContextManager
from dataclasses import asdict, dataclass, field, replace
from typing import Any

from .acoustic_scene import AcousticScene
from .acoustics import MicrophoneSensor
from .camera_geometry import CameraCalibration, CameraPose, GimbalPose, calibrated_rays
from .contracts import ScheduledSensorSample
from .measurement_control import MeasurementNoise
from .measurement_scene import MujocoRasterizer, thermal_calibration
from .samples import SensorSampleStamp
from .semantic_camera import SemanticCameraConfig, SemanticLabel, semantic_from_geom_ids
from .thermal import ThermalConfig, ThermalFrame, ThermalSensor, ThermalSurface

PhysicsRead = Callable[[], AbstractContextManager[tuple[Any, Any]]]


@dataclass(frozen=True, slots=True)
class MountedCamera:
    """Optical frame relative to a physical body, not a UI spectator camera."""

    body_name: str
    calibration: CameraCalibration
    mount_pose: CameraPose = field(default_factory=CameraPose)
    gimbal: GimbalPose = field(default_factory=GimbalPose)
    max_distance_m: float = 100.0

    def __post_init__(self) -> None:
        if not self.body_name or self.body_name != self.body_name.strip():
            raise ValueError("camera body_name must be a nonempty name")
        if not math.isfinite(self.max_distance_m) or self.max_distance_m <= 0.001:
            raise ValueError("camera max_distance_m must exceed .001 m")


def _sample_stamp(request: ScheduledSensorSample, snapshot: Mapping[str, Any]) -> SensorSampleStamp:
    stamp = SensorSampleStamp.from_scheduled(request)
    for field_name in ("session_id", "model_generation", "reset_generation"):
        if snapshot.get(field_name) != getattr(stamp, field_name):
            raise ValueError(f"measurement snapshot {field_name} differs from request")
    actual = snapshot.get("sim_time_ns")
    if type(actual) is not int or actual < request.deadline_ns:
        raise ValueError("measurement snapshot time must be at or after its deadline")
    return replace(stamp, sim_time_ns=actual)


def _camera_raster(model: Any, data: Any, camera: MountedCamera, stamp: SensorSampleStamp):
    import mujoco

    if round(float(data.time) * 1e9) != stamp.sim_time_ns:
        raise ValueError("physics lease time differs from measurement source time")
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, camera.body_name)
    if body_id < 0:
        raise ValueError(f"camera body not found: {camera.body_name}")
    mujoco.mj_kinematics(model, data)
    base = CameraPose(tuple(data.xpos[body_id]), tuple(data.xquat[body_id]))
    rays = calibrated_rays(
        camera.calibration, stamp, base_pose=base, mount_pose=camera.mount_pose, gimbal=camera.gimbal
    )
    # World body 0 must not be excluded: its static surfaces are observable.
    raster = MujocoRasterizer(model, data, stamp).cast(
        rays, max_distance_m=camera.max_distance_m, body_exclude=body_id if body_id > 0 else -1
    )
    return rays, raster


def _geom_ids(model: Any, declarations: Mapping[str, Any]) -> dict[int, Any]:
    import mujoco

    result = {}
    for name, value in declarations.items():
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            raise ValueError(f"measurement geom not found: {name}")
        result[geom_id] = value
    return result


class ThermalCameraExtractor:
    """Per-pixel thermal sampling under the caller's existing physics lock.

    Source time is the accepted snapshot time, never a past scheduling deadline.
    ThermalConfig intrinsics must agree with MountedCamera; world pose metadata
    is refreshed per frame. Surface names are resolved against each model lease.
    """

    def __init__(
        self,
        read_state: PhysicsRead,
        camera: MountedCamera,
        config: ThermalConfig,
        surfaces: Mapping[str, ThermalSurface],
    ):
        self.read_state = read_state
        self.camera = camera
        self.config = config
        self.surfaces = dict(surfaces)
        self.max_rate_hz: float | None = None

    def __call__(self, request: ScheduledSensorSample, snapshot: Mapping[str, Any]) -> ThermalFrame:
        if request.stream.stream_kind != "thermal":
            raise ValueError("thermal extractor requires a thermal stream")
        stamp = _sample_stamp(request, snapshot)
        with self.read_state() as (model, data):
            rays, raster = _camera_raster(model, data, self.camera, stamp)
            calibration = thermal_calibration(rays, self.config.camera_calibration["calibration_id"])
            fields = ("model", "width", "height", "fx", "fy", "cx", "cy", "parameters")
            if any(self.config.camera_calibration.get(key) != calibration.get(key) for key in fields):
                raise ValueError("thermal and mounted camera calibration disagree")
            surfaces = _geom_ids(model, self.surfaces)
            sensor = ThermalSensor(replace(self.config, camera_calibration=calibration), surfaces)
            frame = sensor.measure(
                stamp,
                source_stamp=raster.stamp,
                geom_ids=raster.geom_ids,
                distances_m=raster.distances_m,
                valid_rays=raster.valid_mask,
            )
        return replace(
            frame,
            metadata={
                **frame.metadata,
                "scheduled_deadline_ns": request.deadline_ns,
                "mount_body_name": self.camera.body_name,
                "surface_names": dict(zip(map(str, surfaces), self.surfaces, strict=True)),
            },
        )

    def configure_noise(self, noise: MeasurementNoise) -> None:
        self.config = replace(self.config, noise=noise)

    def measurement_settings(self) -> dict:
        return {
            "noise": self.config.noise.to_dict(),
            "camera": asdict(self.camera),
            "temperature_mode": self.config.temperature_mode,
            "background_temperature_k": self.config.background_temperature_k,
            "transmittance": self.config.transmittance,
        }

    def set_noise_enabled(self, enabled: bool) -> None:
        self.configure_noise(replace(self.config.noise, enabled=enabled))

    def set_gimbal(self, pose: GimbalPose) -> None:
        if not isinstance(pose, GimbalPose):
            raise ValueError("gimbal pose must be calibrated and within its limits")
        self.camera = replace(self.camera, gimbal=pose)

    def set_gimbal_angles(self, pan_deg: float, tilt_deg: float) -> None:
        self.set_gimbal(replace(self.camera.gimbal, pan_deg=pan_deg, tilt_deg=tilt_deg))

    def set_rate_hz(self, value: float) -> None:
        # This extractor is on-demand; the shared scheduler owns its cadence.
        if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError("sample rate must be finite and positive")
        if self.max_rate_hz is not None and value > self.max_rate_hz:
            raise ValueError("sample rate exceeds the physics owner rate")


class SemanticCameraExtractor:
    """Map nearest physical geometry to declared class and instance IDs."""

    def __init__(self, read_state: PhysicsRead, camera: MountedCamera, labels: Mapping[str, tuple[int, int]]):
        self.read_state, self.camera, self.labels = read_state, camera, dict(labels)
        self.noise = MeasurementNoise("label")
        self.max_rate_hz: float | None = None

    def __call__(self, request: ScheduledSensorSample, snapshot: Mapping[str, Any]):
        if request.stream.stream_kind not in {"semantic", "instance"}:
            raise ValueError("semantic extractor requires a label stream")
        stamp = _sample_stamp(request, snapshot)
        with self.read_state() as (model, data):
            rays, raster = _camera_raster(model, data, self.camera, stamp)
            labels = tuple(SemanticLabel(geom_id, *pair) for geom_id, pair in _geom_ids(model, self.labels).items())
            config = SemanticCameraConfig(self.camera.calibration, labels, self.noise)
            return semantic_from_geom_ids(config, stamp, raster.geom_ids, source_stamp=raster.stamp, rays=rays)

    def configure_noise(self, noise: MeasurementNoise) -> None:
        SemanticCameraConfig(self.camera.calibration, (), noise)
        self.noise = noise

    def measurement_settings(self) -> dict:
        return {"noise": self.noise.to_dict(), "camera": asdict(self.camera), "ground_truth": True}

    def set_noise_enabled(self, enabled: bool) -> None:
        self.configure_noise(replace(self.noise, enabled=enabled))

    def set_gimbal(self, pose: GimbalPose) -> None:
        if not isinstance(pose, GimbalPose):
            raise ValueError("gimbal pose must be calibrated and within its limits")
        self.camera = replace(self.camera, gimbal=pose)

    def set_gimbal_angles(self, pan_deg: float, tilt_deg: float) -> None:
        self.set_gimbal(replace(self.camera.gimbal, pan_deg=pan_deg, tilt_deg=tilt_deg))

    def set_rate_hz(self, value: float) -> None:
        if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError("sample rate must be finite and positive")
        if self.max_rate_hz is not None and value > self.max_rate_hz:
            raise ValueError("sample rate exceeds the physics owner rate")


class AcousticExtractor:
    """A scene pressure block sampled under the same physics read lease."""

    def __init__(self, read_state: PhysicsRead, scene: AcousticScene):
        self.read_state, self.scene = read_state, scene

    def __call__(self, request: ScheduledSensorSample, snapshot: Mapping[str, Any]):
        if request.stream.stream_kind != "acoustic":
            raise ValueError("acoustic extractor requires an acoustic stream")
        self.set_rate_hz(float(request.stream.rate_hz))
        stamp = _sample_stamp(request, snapshot)
        with self.read_state() as (model, data):
            frame = self.scene.sample(model, data, stamp)
        return replace(frame, metadata={**frame.metadata, "scheduled_deadline_ns": request.deadline_ns})

    def configure_noise(self, noise: MeasurementNoise) -> None:
        self.scene.sensor = MicrophoneSensor(replace(self.scene.sensor.config, noise=noise))

    def measurement_settings(self) -> dict:
        return {
            "noise": self.scene.sensor.config.noise.to_dict(),
            "microphone": asdict(self.scene.sensor.config),
            "receiver_body_name": self.scene.receiver_body_name,
        }

    def set_noise_enabled(self, enabled: bool) -> None:
        self.configure_noise(replace(self.scene.sensor.config.noise, enabled=enabled))

    def set_rate_hz(self, value: float) -> None:
        config = self.scene.sensor.config
        if isinstance(value, bool) or value != config.sample_rate_hz / config.block_samples:
            raise ValueError("acoustic cadence must equal sample_rate_hz / block_samples; reconfigure block first")
