"""Canonical LTU1 sensor-record encoding for the MuJoCo native DDS seam."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Any

from runtime.msgs.sensor import POINT_DTYPE
from runtime.msgs.numpy_compat import np

LTU1_MAGIC = b"LTU1"
RECORD_CLOUD = 1
RECORD_IMU = 2
RECORD_ODOM_PRIOR = 3
RECORD_REGISTERED_CLOUD = 4
RECORD_CAMERA = 5
MID360_ACCEL_MPS2_PER_G = 9.80665

HEADER = struct.Struct("<4sB3xQIII")
CAMERA_RECORD_HEADER = struct.Struct("<4sHHIIIIddddddIddddd")
IMU_PAYLOAD = struct.Struct("<ffffff")
ODOM_PRIOR_PAYLOAD = struct.Struct("<ddddddddddB7x")
MAX_SENSOR_RECORD_PAYLOAD_BYTES = 256 * 1024 * 1024
MAX_CAMERA_RECORD_PAYLOAD_BYTES = 128 * 1024 * 1024
MAX_CAMERA_TIMESTAMP_S = (1 << 31) - 1
_UINT8_MAX = (1 << 8) - 1
_UINT32_MAX = (1 << 32) - 1
_UINT64_MAX = (1 << 64) - 1


@dataclass(frozen=True)
class EncodedSensorRecord:
    """One immutable LTU1 header and payload ready for a byte stream."""

    record_type: int
    header: bytes
    payload: bytes

    @property
    def wire(self) -> bytes:
        """Return the complete record as one immutable byte string."""

        return self.header + self.payload

    @property
    def wire_bytes(self) -> int:
        """Return the encoded header-plus-payload byte count without copying."""

        return len(self.header) + len(self.payload)


def _unsigned_int(name: str, value: Any, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if value < 0 or value > maximum:
        raise ValueError(f"{name} must be in range 0..{maximum}")
    return value


def _seconds_to_timestamp_ns(name: str, value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a real number")
    seconds = float(value)
    if not math.isfinite(seconds):
        raise ValueError(f"{name} must be finite")
    if seconds < 0.0 or seconds > _UINT64_MAX / 1_000_000_000:
        raise ValueError(f"{name} is outside the LTU1 uint64 nanosecond range")
    return int(seconds * 1_000_000_000)


def encode_record(
    record_type: int,
    *,
    timestamp_ns: int,
    sequence: int,
    count: int,
    payload: bytes | bytearray | memoryview,
) -> EncodedSensorRecord:
    """Encode one LTU1 record after checking every fixed-width wire field."""

    encoded_type = _unsigned_int("record_type", record_type, _UINT8_MAX)
    if encoded_type not in {
        RECORD_CLOUD,
        RECORD_IMU,
        RECORD_ODOM_PRIOR,
        RECORD_REGISTERED_CLOUD,
        RECORD_CAMERA,
    }:
        raise ValueError(f"unsupported LTU1 record_type: {encoded_type}")
    encoded_timestamp = _unsigned_int("timestamp_ns", timestamp_ns, _UINT64_MAX)
    encoded_sequence = _unsigned_int("sequence", sequence, _UINT32_MAX)
    encoded_count = _unsigned_int("count", count, _UINT32_MAX)
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError("payload must be bytes-like")
    encoded_payload = bytes(payload)
    if len(encoded_payload) > MAX_SENSOR_RECORD_PAYLOAD_BYTES:
        raise ValueError("LTU1 payload exceeds the 256 MiB parser safety limit")
    header = HEADER.pack(
        LTU1_MAGIC,
        encoded_type,
        encoded_timestamp,
        encoded_sequence,
        encoded_count,
        len(encoded_payload),
    )
    return EncodedSensorRecord(encoded_type, header, encoded_payload)


def _camera_header(
    intrinsics: Any,
    *,
    kind: int,
    timestamp_s: int | float,
    channels: int,
    image_format: int,
    payload_size: int,
) -> tuple[bytes, int]:
    timestamp_ns = _seconds_to_timestamp_ns("timestamp_s", timestamp_s)
    if timestamp_ns <= 0:
        raise ValueError("timestamp_s must be a positive wall-clock time")
    if float(timestamp_s) > MAX_CAMERA_TIMESTAMP_S:
        raise ValueError("timestamp_s exceeds the LTOB v2 range")
    if payload_size > MAX_CAMERA_RECORD_PAYLOAD_BYTES:
        raise ValueError("camera payload exceeds the LTOB v2 safety limit")
    width = _unsigned_int("intrinsics.width", intrinsics.width, 16_384)
    height = _unsigned_int("intrinsics.height", intrinsics.height, 16_384)
    if width == 0 or height == 0:
        raise ValueError("camera dimensions must be positive")
    values = (
        float(intrinsics.fx),
        float(intrinsics.fy),
        float(intrinsics.cx),
        float(intrinsics.cy),
        *(float(getattr(intrinsics, name, 0.0)) for name in (
            "dist_k1",
            "dist_k2",
            "dist_p1",
            "dist_p2",
            "dist_k3",
        )),
    )
    fx, fy, cx, cy, *distortion = values
    if not all(math.isfinite(value) for value in values):
        raise ValueError("camera intrinsics must be finite")
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    if not 0.0 <= cx < width or not 0.0 <= cy < height:
        raise ValueError("camera principal point must be inside the image")
    return (
        CAMERA_RECORD_HEADER.pack(
            b"LTOB",
            2,
            kind,
            width,
            height,
            channels,
            image_format,
            float(timestamp_s),
            fx,
            fy,
            cx,
            cy,
            0.001,
            payload_size,
            *distortion,
        ),
        timestamp_ns,
    )


def encode_camera_intrinsics(
    intrinsics: Any,
    *,
    timestamp_s: int | float,
    sequence: int,
) -> EncodedSensorRecord:
    """Encode canonical LTOB v2 camera calibration inside one LTU1 record."""

    payload, timestamp_ns = _camera_header(
        intrinsics,
        kind=1,
        timestamp_s=timestamp_s,
        channels=0,
        image_format=0,
        payload_size=0,
    )
    return encode_record(
        RECORD_CAMERA,
        timestamp_ns=timestamp_ns,
        sequence=sequence,
        count=1,
        payload=payload,
    )


def encode_camera_rgb(
    rgb: Any,
    *,
    intrinsics: Any,
    timestamp_s: int | float,
    sequence: int,
) -> EncodedSensorRecord:
    """Encode one contiguous HxWx3 uint8 RGB frame as canonical LTOB v2."""

    image = np.asarray(rgb)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("camera RGB must be an HxWx3 uint8 array")
    if not bool(image.flags.c_contiguous):
        raise ValueError("camera RGB must be C-contiguous")
    if image.shape[:2] != (intrinsics.height, intrinsics.width):
        raise ValueError("camera RGB dimensions must match intrinsics")
    image_payload = image.tobytes(order="C")
    header, timestamp_ns = _camera_header(
        intrinsics,
        kind=2,
        timestamp_s=timestamp_s,
        channels=3,
        image_format=1,
        payload_size=len(image_payload),
    )
    return encode_record(
        RECORD_CAMERA,
        timestamp_ns=timestamp_ns,
        sequence=sequence,
        count=1,
        payload=header + image_payload,
    )


def encode_camera_depth(
    depth_m: Any,
    *,
    intrinsics: Any,
    timestamp_s: int | float,
    sequence: int,
) -> EncodedSensorRecord:
    """Encode one HxW metric depth image as little-endian uint16 millimetres."""

    depth = np.asarray(depth_m)
    if depth.ndim != 2 or depth.dtype.kind not in "fiu" or depth.dtype.kind == "b":
        raise ValueError("camera depth must be an HxW numeric array in metres")
    if depth.shape != (intrinsics.height, intrinsics.width):
        raise ValueError("camera depth dimensions must match intrinsics")
    if not bool(np.isfinite(depth).all()):
        raise ValueError("camera depth must contain only finite values")
    if bool((depth < 0.0).any()):
        raise ValueError("camera depth must be nonnegative")
    if bool((depth > 65.535).any()):
        raise ValueError("camera depth exceeds uint16 millimetre range")
    millimeters = np.floor(depth.astype(np.float64) * 1000.0 + 0.5).astype("<u2")
    image_payload = millimeters.tobytes(order="C")
    header, timestamp_ns = _camera_header(
        intrinsics,
        kind=3,
        timestamp_s=timestamp_s,
        channels=1,
        image_format=3,
        payload_size=len(image_payload),
    )
    return encode_record(
        RECORD_CAMERA,
        timestamp_ns=timestamp_ns,
        sequence=sequence,
        count=1,
        payload=header + image_payload,
    )


def encode_imu(imu: Any, *, sequence: int) -> EncodedSensorRecord:
    """Encode one MID-360-compatible IMU sample in gyro/rad-s and accel/g."""

    timestamp_ns = _seconds_to_timestamp_ns("imu.ts", imu.ts)
    values = (
        float(imu.angular_velocity.x),
        float(imu.angular_velocity.y),
        float(imu.angular_velocity.z),
        float(imu.linear_acceleration.x) / MID360_ACCEL_MPS2_PER_G,
        float(imu.linear_acceleration.y) / MID360_ACCEL_MPS2_PER_G,
        float(imu.linear_acceleration.z) / MID360_ACCEL_MPS2_PER_G,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("LTU1 IMU samples must be finite")
    payload = IMU_PAYLOAD.pack(*values)
    return encode_record(
        RECORD_IMU,
        timestamp_ns=timestamp_ns,
        sequence=sequence,
        count=1,
        payload=payload,
    )


def _encode_point_frame(scan: Any, *, record_type: int) -> EncodedSensorRecord:
    points = np.asarray(scan.points)
    if points.ndim != 1 or points.dtype != POINT_DTYPE:
        raise ValueError("LTU1 point records require a one-dimensional POINT_DTYPE array")
    count = _unsigned_int("point_count", scan.point_count, _UINT32_MAX)
    if count != int(points.shape[0]):
        raise ValueError("point_count must equal the number of encoded Livox points")
    for field in ("x", "y", "z", "intensity"):
        if not bool(np.isfinite(points[field]).all()):
            raise ValueError(f"LTU1 point field {field} must contain only finite values")
    payload = points.tobytes(order="C")
    if len(payload) != count * POINT_DTYPE.itemsize:
        raise ValueError("LTU1 point payload size does not match point_count")
    return encode_record(
        record_type,
        timestamp_ns=scan.timestamp_ns,
        sequence=scan.sequence,
        count=count,
        payload=payload,
    )


def encode_scan(scan: Any) -> EncodedSensorRecord:
    """Encode one canonical raw Livox point frame."""

    return _encode_point_frame(scan, record_type=RECORD_CLOUD)


def encode_odom_prior(
    state: Any,
    *,
    timestamp_s: int | float,
    sequence: int,
    velocity: Any | None = None,
    has_velocity: bool = True,
) -> EncodedSensorRecord:
    """Encode the optional MuJoCo truth odometry prior used by native SLAM."""

    if not isinstance(has_velocity, bool):
        raise TypeError("has_velocity must be bool")
    position = np.asarray(
        getattr(state, "position", (0.0, 0.0, 0.0)), dtype=np.float64
    ).reshape(-1)
    orientation = np.asarray(
        getattr(state, "orientation", (0.0, 0.0, 0.0, 1.0)), dtype=np.float64
    ).reshape(-1)
    if velocity is None:
        velocity = getattr(state, "linear_velocity", (0.0, 0.0, 0.0))
    linear_velocity = np.asarray(velocity, dtype=np.float64).reshape(-1)
    if position.size < 3:
        position = np.pad(position, (0, 3 - position.size))
    if orientation.size < 4:
        orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    if linear_velocity.size < 3:
        linear_velocity = np.pad(
            linear_velocity, (0, 3 - linear_velocity.size)
        )
    values = (
        float(position[0]),
        float(position[1]),
        float(position[2]),
        float(orientation[0]),
        float(orientation[1]),
        float(orientation[2]),
        float(orientation[3]),
        float(linear_velocity[0]),
        float(linear_velocity[1]),
        float(linear_velocity[2]),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("LTU1 odom prior values must be finite")
    payload = ODOM_PRIOR_PAYLOAD.pack(*values, int(has_velocity))
    return encode_record(
        RECORD_ODOM_PRIOR,
        timestamp_ns=_seconds_to_timestamp_ns("timestamp_s", timestamp_s),
        sequence=sequence,
        count=1,
        payload=payload,
    )


def encode_registered_cloud(
    points_xyzi_body: Any,
    *,
    timestamp_ns: int,
    sequence: int,
) -> EncodedSensorRecord:
    """Encode body-frame XYZ[I] points as a registered Livox cloud."""

    points = np.asarray(points_xyzi_body, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(
            f"expected body-frame XYZI cloud shape (N, >=3), got {points.shape}"
        )
    count = int(points.shape[0])
    _unsigned_int("point_count", count, _UINT32_MAX)
    values = points[:, :4] if points.shape[1] >= 4 else points[:, :3]
    if not bool(np.isfinite(values).all()):
        raise ValueError("registered cloud coordinates and intensity must be finite")
    raw = np.zeros(count, dtype=POINT_DTYPE)
    if count:
        raw["x"] = points[:, 0]
        raw["y"] = points[:, 1]
        raw["z"] = points[:, 2]
        if points.shape[1] >= 4:
            raw["intensity"] = points[:, 3]
    payload = raw.tobytes(order="C")
    if len(payload) != count * POINT_DTYPE.itemsize:
        raise ValueError("LTU1 registered-cloud payload size mismatch")
    return encode_record(
        RECORD_REGISTERED_CLOUD,
        timestamp_ns=timestamp_ns,
        sequence=sequence,
        count=count,
        payload=payload,
    )
