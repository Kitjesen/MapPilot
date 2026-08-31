"""Typed, transport-neutral sensor sample contracts."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TypeAlias

from .contracts import ScheduledSensorSample

Vector3: TypeAlias = tuple[float, float, float]
QuaternionWxyz: TypeAlias = tuple[float, float, float, float]
Covariance: TypeAlias = tuple[float, ...]


def _text(value: object, field: str) -> None:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{field} must be a non-empty trimmed string")


def _non_negative_integer(value: object, field: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")


def _finite_tuple(value: object, size: int, field: str) -> None:
    if not isinstance(value, tuple) or len(value) != size:
        raise ValueError(f"{field} must be a {size}-value tuple")
    for index, item in enumerate(value):
        if (
            isinstance(item, bool)
            or not isinstance(item, (int, float))
            or not math.isfinite(item)
        ):
            raise ValueError(f"{field}[{index}] must be finite")


def _optional_finite_tuple(
    value: object | None, size: int, field: str
) -> None:
    if value is not None:
        _finite_tuple(value, size, field)


@dataclass(frozen=True, slots=True)
class SensorSampleStamp:
    """Generation and simulation-clock identity shared by sensor samples."""

    session_id: str
    instance_id: str
    sensor_id: str
    frame_id: str
    model_generation: int
    reset_generation: int
    sequence: int
    sim_time_ns: int

    def __post_init__(self) -> None:
        """Validate identity fields without changing caller-provided values."""

        _text(self.session_id, "session_id")
        _text(self.instance_id, "instance_id")
        _text(self.sensor_id, "sensor_id")
        _text(self.frame_id, "frame_id")
        _non_negative_integer(self.model_generation, "model_generation")
        _non_negative_integer(self.reset_generation, "reset_generation")
        _non_negative_integer(self.sequence, "sequence")
        _non_negative_integer(self.sim_time_ns, "sim_time_ns")

    @classmethod
    def from_scheduled(cls, scheduled: ScheduledSensorSample) -> SensorSampleStamp:
        """Build an exact stamp from one scheduler request."""

        return cls(
            session_id=scheduled.session_id,
            instance_id=scheduled.stream.instance_id,
            sensor_id=scheduled.sensor_id,
            frame_id=scheduled.stream.frame_id,
            model_generation=scheduled.model_generation,
            reset_generation=scheduled.reset_generation,
            sequence=scheduled.sequence,
            sim_time_ns=scheduled.deadline_ns,
        )


@dataclass(frozen=True, slots=True)
class ImuSample:
    """Measured IMU values; unavailable fields remain explicitly ``None``."""

    stamp: SensorSampleStamp
    orientation_wxyz: QuaternionWxyz | None
    angular_velocity_rps: Vector3 | None
    linear_acceleration_mps2: Vector3 | None
    orientation_covariance: Covariance | None = None
    angular_velocity_covariance: Covariance | None = None
    linear_acceleration_covariance: Covariance | None = None

    def __post_init__(self) -> None:
        """Reject malformed measurements without estimating missing values."""

        if not isinstance(self.stamp, SensorSampleStamp):
            raise ValueError("stamp must be a SensorSampleStamp")
        _optional_finite_tuple(self.orientation_wxyz, 4, "orientation_wxyz")
        if self.orientation_wxyz is not None and not any(self.orientation_wxyz):
            raise ValueError("orientation_wxyz must be non-zero when available")
        _optional_finite_tuple(
            self.angular_velocity_rps, 3, "angular_velocity_rps"
        )
        _optional_finite_tuple(
            self.linear_acceleration_mps2, 3, "linear_acceleration_mps2"
        )
        _optional_finite_tuple(
            self.orientation_covariance, 9, "orientation_covariance"
        )
        _optional_finite_tuple(
            self.angular_velocity_covariance,
            9,
            "angular_velocity_covariance",
        )
        _optional_finite_tuple(
            self.linear_acceleration_covariance,
            9,
            "linear_acceleration_covariance",
        )
        if (
            self.orientation_wxyz is None
            and self.angular_velocity_rps is None
            and self.linear_acceleration_mps2 is None
        ):
            raise ValueError("an IMU sample must contain at least one measured value")


@dataclass(frozen=True, slots=True)
class TruthOdometrySample:
    """MuJoCo truth pose and only the velocities actually made available."""

    stamp: SensorSampleStamp
    position_m: Vector3
    orientation_wxyz: QuaternionWxyz
    linear_velocity_mps: Vector3 | None
    angular_velocity_rps: Vector3 | None
    pose_covariance: Covariance | None = None
    twist_covariance: Covariance | None = None

    def __post_init__(self) -> None:
        """Validate truth values without adding covariance or missing velocity."""

        if not isinstance(self.stamp, SensorSampleStamp):
            raise ValueError("stamp must be a SensorSampleStamp")
        _finite_tuple(self.position_m, 3, "position_m")
        _finite_tuple(self.orientation_wxyz, 4, "orientation_wxyz")
        if not any(self.orientation_wxyz):
            raise ValueError("orientation_wxyz must be non-zero")
        _optional_finite_tuple(self.linear_velocity_mps, 3, "linear_velocity_mps")
        _optional_finite_tuple(
            self.angular_velocity_rps, 3, "angular_velocity_rps"
        )
        _optional_finite_tuple(self.pose_covariance, 36, "pose_covariance")
        _optional_finite_tuple(self.twist_covariance, 36, "twist_covariance")


@dataclass(frozen=True, slots=True)
class LivoxPointSample:
    """One simulation Livox point with explicit conservative metadata."""

    x: float
    y: float
    z: float
    reflectivity: int
    offset_time_ns: int
    tag: int = 0
    line: int = 0

    def __post_init__(self) -> None:
        _finite_tuple((self.x, self.y, self.z), 3, "livox_point.xyz")
        for field, value, maximum in (
            ("reflectivity", self.reflectivity, 255),
            ("offset_time_ns", self.offset_time_ns, (1 << 32) - 1),
            ("tag", self.tag, 255),
            ("line", self.line, 255),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or value < 0
                or value > maximum
            ):
                raise ValueError(f"{field} must be an unsigned integer <= {maximum}")


@dataclass(frozen=True, slots=True)
class Mid360FrameSample:
    """One accepted MuJoCo MID-360 frame for the sim LivoxFrame DDS topic."""

    stamp: SensorSampleStamp
    points: tuple[LivoxPointSample, ...]
    scan_time_profile: str
    reflectivity_semantics: str = "explicit_conservative_proxy"
    unknown_line_representation: str = "line_0_unknown_physical_channel"

    def __post_init__(self) -> None:
        if not isinstance(self.stamp, SensorSampleStamp):
            raise ValueError("stamp must be a SensorSampleStamp")
        if not self.points:
            raise ValueError("MID-360 frame must contain at least one real raycast point")
        if not isinstance(self.points, tuple) or not all(
            isinstance(point, LivoxPointSample) for point in self.points
        ):
            raise ValueError("points must be a tuple of LivoxPointSample")
        previous_offset = -1
        for point in self.points:
            if point.offset_time_ns < previous_offset:
                raise ValueError("point offset_time_ns must be non-decreasing")
            previous_offset = point.offset_time_ns
        if self.scan_time_profile not in {
            "physical_rolling/subscan",
            "instantaneous_geometry/scheduled_offsets",
        }:
            raise ValueError("scan_time_profile must name the Mid360 timing qualification")
        if self.reflectivity_semantics != "explicit_conservative_proxy":
            raise ValueError("reflectivity must be identified as an explicit conservative proxy")
        if self.unknown_line_representation != "line_0_unknown_physical_channel":
            raise ValueError("line=0 must be represented as an unknown physical channel")
