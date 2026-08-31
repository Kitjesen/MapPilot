"""Strict conversion from Physics Runtime snapshots to typed sensor samples."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any, cast

from .contracts import ScheduledSensorSample
from .samples import ImuSample, SensorSampleStamp, TruthOdometrySample


class SensorSampleError(ValueError):
    """Raised when source truth cannot satisfy one scheduled sample exactly."""


def _integer(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise SensorSampleError(f"{field} must be a non-negative integer")
    return value


def _vector(value: Any, size: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise SensorSampleError(f"{field} must contain exactly {size} values")
    if len(value) != size:
        raise SensorSampleError(f"{field} must contain exactly {size} values")
    result: list[float] = []
    for index, item in enumerate(value):
        if (
            isinstance(item, bool)
            or not isinstance(item, (int, float))
            or not math.isfinite(item)
        ):
            raise SensorSampleError(f"{field}[{index}] must be finite numeric data")
        result.append(float(item))
    return tuple(result)


def truth_odometry_from_snapshot(
    scheduled: ScheduledSensorSample,
    snapshot: Mapping[str, Any],
) -> TruthOdometrySample:
    """Create truth odometry only from an identity-matched, exact-time snapshot."""

    if scheduled.stream.stream_kind != "truth_odom":
        raise SensorSampleError("scheduled stream is not truth_odom")
    if snapshot.get("session_id") != scheduled.session_id:
        raise SensorSampleError("snapshot session_id does not match the schedule")
    if _integer(snapshot.get("model_generation"), "snapshot.model_generation") != (
        scheduled.model_generation
    ):
        raise SensorSampleError("snapshot model_generation does not match the schedule")
    if _integer(snapshot.get("reset_generation"), "snapshot.reset_generation") != (
        scheduled.reset_generation
    ):
        raise SensorSampleError("snapshot reset_generation does not match the schedule")
    sim_time_ns = _integer(snapshot.get("sim_time_ns"), "snapshot.sim_time_ns")
    if sim_time_ns != scheduled.deadline_ns:
        raise SensorSampleError(
            "truth odometry requires a snapshot at the exact scheduled deadline"
        )

    bodies = snapshot.get("bodies")
    if not isinstance(bodies, list):
        raise SensorSampleError("snapshot.bodies must be a list")
    matches = [
        body
        for body in bodies
        if isinstance(body, Mapping)
        and body.get("stable_id") == scheduled.stream.frame_id
    ]
    if len(matches) != 1:
        raise SensorSampleError(
            f"snapshot must contain exactly one body {scheduled.stream.frame_id!r}"
        )
    body = matches[0]
    position = cast(
        tuple[float, float, float],
        _vector(body.get("position_m"), 3, "body.position_m"),
    )
    orientation = cast(
        tuple[float, float, float, float],
        _vector(body.get("quaternion_wxyz"), 4, "body.quaternion_wxyz"),
    )
    if not any(orientation):
        raise SensorSampleError("body.quaternion_wxyz must be non-zero")
    linear_velocity = cast(
        tuple[float, float, float],
        _vector(
            body.get("linear_velocity_mps"),
            3,
            "body.linear_velocity_mps",
        ),
    )
    angular_velocity = cast(
        tuple[float, float, float],
        _vector(
            body.get("angular_velocity_rps"),
            3,
            "body.angular_velocity_rps",
        ),
    )
    return TruthOdometrySample(
        stamp=SensorSampleStamp.from_scheduled(scheduled),
        position_m=position,
        orientation_wxyz=orientation,
        linear_velocity_mps=linear_velocity,
        angular_velocity_rps=angular_velocity,
    )


def _validate_snapshot_identity(
    scheduled: ScheduledSensorSample, snapshot: Mapping[str, Any]
) -> int:
    """Validate the immutable snapshot identity and return its exact time."""

    if snapshot.get("session_id") != scheduled.session_id:
        raise SensorSampleError("snapshot session_id does not match the schedule")
    if _integer(snapshot.get("model_generation"), "snapshot.model_generation") != (
        scheduled.model_generation
    ):
        raise SensorSampleError("snapshot model_generation does not match the schedule")
    if _integer(snapshot.get("reset_generation"), "snapshot.reset_generation") != (
        scheduled.reset_generation
    ):
        raise SensorSampleError("snapshot reset_generation does not match the schedule")
    sim_time_ns = _integer(snapshot.get("sim_time_ns"), "snapshot.sim_time_ns")
    if sim_time_ns != scheduled.deadline_ns:
        raise SensorSampleError(
            "IMU requires a snapshot at the exact scheduled deadline"
        )
    return sim_time_ns


def _sensor_values(
    sensors: Any,
    *,
    frame_id: str,
    semantic: str,
    accepted_types: frozenset[str],
) -> tuple[float, ...]:
    """Select one MuJoCo sensor by exact source frame and semantic type."""

    if not isinstance(sensors, list):
        raise SensorSampleError("snapshot.sensors must be a list")
    matches = [
        sensor
        for sensor in sensors
        if isinstance(sensor, Mapping)
        and sensor.get("source_stable_id") == frame_id
        and sensor.get("sensor_type") in accepted_types
    ]
    if len(matches) != 1:
        raise SensorSampleError(
            f"snapshot must contain exactly one {semantic} sensor for {frame_id!r}"
        )
    return _vector(matches[0].get("values"), 4 if semantic == "framequat" else 3,
                   f"sensor.{semantic}.values")


def imu_from_snapshot(
    scheduled: ScheduledSensorSample,
    snapshot: Mapping[str, Any],
) -> ImuSample:
    """Extract MuJoCo IMU truth from the scheduled frame's sensor values.

    The extractor deliberately never consults body velocity or finite
    differences.  Orientation, gyro, and specific force must be exported by
    the physics snapshot as typed MuJoCo sensor values.
    """

    if scheduled.stream.stream_kind != "imu":
        raise SensorSampleError("scheduled stream is not imu")
    _validate_snapshot_identity(scheduled, snapshot)
    frame_id = scheduled.stream.frame_id
    orientation = _sensor_values(
        snapshot.get("sensors"),
        frame_id=frame_id,
        semantic="framequat",
        accepted_types=frozenset({"framequat", "orientation"}),
    )
    gyro = _sensor_values(
        snapshot.get("sensors"),
        frame_id=frame_id,
        semantic="gyro",
        accepted_types=frozenset({"gyro", "angular-velocity"}),
    )
    acceleration = _sensor_values(
        snapshot.get("sensors"),
        frame_id=frame_id,
        semantic="accelerometer",
        accepted_types=frozenset({"accelerometer", "linear-acceleration"}),
    )
    return ImuSample(
        stamp=SensorSampleStamp.from_scheduled(scheduled),
        orientation_wxyz=orientation,  # type: ignore[arg-type]
        angular_velocity_rps=gyro,  # type: ignore[arg-type]
        linear_acceleration_mps2=acceleration,  # type: ignore[arg-type]
    )
