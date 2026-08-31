from __future__ import annotations

import struct
import types
from io import BytesIO

import pytest

from runtime.msgs.sensor import POINT_DTYPE
from runtime.msgs.geometry import Quaternion, Vector3
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu
from sim.scripts.mujoco.native_sensor_records import (
    HEADER,
    IMU_PAYLOAD,
    LTU1_MAGIC,
    MID360_ACCEL_MPS2_PER_G,
    ODOM_PRIOR_PAYLOAD,
    RECORD_CAMERA,
    RECORD_CLOUD,
    encode_camera_depth,
    encode_camera_intrinsics,
    encode_camera_rgb,
    encode_imu,
    encode_odom_prior,
    encode_record,
    encode_registered_cloud,
    encode_scan,
)


def test_encode_camera_intrinsics_wraps_exact_ltob_v2_record() -> None:
    intrinsics = types.SimpleNamespace(
        width=4,
        height=3,
        fx=2.5,
        fy=2.75,
        cx=1.5,
        cy=1.0,
        dist_k1=0.0,
        dist_k2=0.0,
        dist_p1=0.0,
        dist_p2=0.0,
        dist_k3=0.0,
    )

    encoded = encode_camera_intrinsics(
        intrinsics,
        timestamp_s=1.5,
        sequence=7,
    )

    ltob = struct.pack(
        "<4sHHIIIIddddddIddddd",
        b"LTOB",
        2,
        1,
        4,
        3,
        0,
        0,
        1.5,
        2.5,
        2.75,
        1.5,
        1.0,
        0.001,
        0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    assert len(ltob) == 116
    assert encoded.record_type == RECORD_CAMERA
    assert encoded.header == HEADER.pack(
        LTU1_MAGIC,
        RECORD_CAMERA,
        1_500_000_000,
        7,
        1,
        len(ltob),
    )
    assert encoded.payload == ltob


def test_encode_camera_rgb_preserves_contiguous_rgb8_payload() -> None:
    intrinsics = types.SimpleNamespace(
        width=2,
        height=1,
        fx=2.0,
        fy=2.0,
        cx=1.0,
        cy=0.0,
    )
    rgb = np.array([[[1, 2, 3], [250, 251, 252]]], dtype=np.uint8)

    encoded = encode_camera_rgb(
        rgb,
        intrinsics=intrinsics,
        timestamp_s=2.25,
        sequence=8,
    )

    ltob_header = struct.pack(
        "<4sHHIIIIddddddIddddd",
        b"LTOB",
        2,
        2,
        2,
        1,
        3,
        1,
        2.25,
        2.0,
        2.0,
        1.0,
        0.0,
        0.001,
        6,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    assert encoded.header == HEADER.pack(
        LTU1_MAGIC,
        RECORD_CAMERA,
        2_250_000_000,
        8,
        1,
        122,
    )
    assert encoded.payload == ltob_header + bytes((1, 2, 3, 250, 251, 252))


def test_encode_camera_depth_converts_metric_values_to_little_endian_mm() -> None:
    intrinsics = types.SimpleNamespace(
        width=3,
        height=1,
        fx=2.0,
        fy=2.0,
        cx=1.0,
        cy=0.0,
    )
    depth_m = np.array([[0.0, 1.234, 65.535]], dtype=np.float64)

    encoded = encode_camera_depth(
        depth_m,
        intrinsics=intrinsics,
        timestamp_s=3.5,
        sequence=9,
    )

    header = struct.pack(
        "<4sHHIIIIddddddIddddd",
        b"LTOB",
        2,
        3,
        3,
        1,
        1,
        3,
        3.5,
        2.0,
        2.0,
        1.0,
        0.0,
        0.001,
        6,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    millimeters = struct.pack("<HHH", 0, 1234, 65535)
    assert encoded.header == HEADER.pack(
        LTU1_MAGIC,
        RECORD_CAMERA,
        3_500_000_000,
        9,
        1,
        122,
    )
    assert encoded.payload == header + millimeters


@pytest.mark.parametrize(
    "rgb",
    [
        np.zeros((2, 3), dtype=np.uint8),
        np.zeros((1, 2, 4), dtype=np.uint8),
        np.zeros((1, 2, 3), dtype=np.float32),
        np.zeros((1, 4, 3), dtype=np.uint8)[:, ::2, :],
    ],
)
def test_encode_camera_rgb_rejects_noncanonical_arrays(rgb: object) -> None:
    intrinsics = types.SimpleNamespace(
        width=2,
        height=1,
        fx=2.0,
        fy=2.0,
        cx=1.0,
        cy=0.0,
    )

    with pytest.raises(ValueError):
        encode_camera_rgb(
            rgb,
            intrinsics=intrinsics,
            timestamp_s=1.0,
            sequence=0,
        )


@pytest.mark.parametrize(
    "depth",
    [
        np.zeros((1, 2, 1), dtype=np.float32),
        np.zeros((1, 2), dtype=np.bool_),
        np.array([[0.0, float("nan")]], dtype=np.float64),
        np.array([[0.0, float("inf")]], dtype=np.float64),
        np.array([[0.0, -0.001]], dtype=np.float64),
        np.array([[0.0, 65.536]], dtype=np.float64),
    ],
)
def test_encode_camera_depth_rejects_invalid_metric_values(depth: object) -> None:
    intrinsics = types.SimpleNamespace(
        width=2,
        height=1,
        fx=2.0,
        fy=2.0,
        cx=1.0,
        cy=0.0,
    )

    with pytest.raises(ValueError):
        encode_camera_depth(
            depth,
            intrinsics=intrinsics,
            timestamp_s=1.0,
            sequence=0,
        )


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("fx", 0.0),
        ("fy", -1.0),
        ("cx", -0.1),
        ("cx", 2.0),
        ("cy", -0.1),
        ("cy", 1.0),
        ("dist_k1", float("nan")),
    ],
)
def test_encode_camera_intrinsics_rejects_values_outside_image(
    field: str,
    value: float,
) -> None:
    values = {
        "width": 2,
        "height": 1,
        "fx": 2.0,
        "fy": 2.0,
        "cx": 1.0,
        "cy": 0.0,
        "dist_k1": 0.0,
    }
    values[field] = value

    with pytest.raises(ValueError):
        encode_camera_intrinsics(
            types.SimpleNamespace(**values),
            timestamp_s=1.0,
            sequence=0,
        )


def test_encode_camera_rejects_nonpositive_wall_timestamp() -> None:
    intrinsics = types.SimpleNamespace(
        width=2,
        height=1,
        fx=2.0,
        fy=2.0,
        cx=1.0,
        cy=0.0,
    )

    with pytest.raises(ValueError, match="positive wall-clock"):
        encode_camera_intrinsics(intrinsics, timestamp_s=0.0, sequence=0)


def test_encode_imu_matches_the_ltu1_wire_contract_exactly() -> None:
    imu = Imu(
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.25, -0.5, 1.0),
        linear_acceleration=Vector3(9.80665, 0.0, -19.6133),
        ts=1.5,
        frame_id="imu_link",
    )

    encoded = encode_imu(imu, sequence=7)

    expected_header = bytes.fromhex(
        "4c545531"  # LTU1
        "02"  # IMU record
        "000000"  # reserved
        "002f685900000000"  # 1_500_000_000 ns, little endian
        "07000000"  # sequence
        "01000000"  # one sample
        "18000000"  # 24-byte payload
    )
    expected_payload = struct.pack("<ffffff", 0.25, -0.5, 1.0, 1.0, 0.0, -2.0)
    assert encoded.header == expected_header
    assert encoded.payload == expected_payload
    assert encoded.wire == expected_header + expected_payload


def test_encode_record_matches_the_ltu1_point_wire_contract_exactly() -> None:
    point = struct.pack("<ffffIBBH", 1.0, -2.0, 3.5, 42.0, 500, 6, 7, 0)

    encoded = encode_record(
        RECORD_CLOUD,
        timestamp_ns=123,
        sequence=4,
        count=1,
        payload=point,
    )

    assert encoded.wire == (
        bytes.fromhex(
            "4c545531010000007b00000000000000040000000100000018000000"
        )
        + point
    )


def test_encode_scan_preserves_the_canonical_livox_point_layout() -> None:
    points = np.zeros(1, dtype=POINT_DTYPE)
    points[0] = (1.0, -2.0, 3.5, 42.0, 500, 6, 7, 0)
    scan = types.SimpleNamespace(
        points=points,
        timestamp_ns=123,
        sequence=4,
        point_count=1,
    )

    encoded = encode_scan(scan)

    assert encoded.record_type == RECORD_CLOUD
    assert encoded.payload == struct.pack(
        "<ffffIBBH", 1.0, -2.0, 3.5, 42.0, 500, 6, 7, 0
    )


def test_encode_odom_prior_matches_the_native_struct_layout() -> None:
    state = types.SimpleNamespace(
        position=np.array([1.0, 2.0, 0.3], dtype=np.float64),
        orientation=np.array([0.0, 0.0, 0.5, 0.8660254], dtype=np.float64),
        linear_velocity=np.array([0.4, 0.0, -0.1], dtype=np.float64),
    )

    encoded = encode_odom_prior(
        state,
        timestamp_s=12.5,
        sequence=9,
        has_velocity=True,
    )

    assert encoded.header == bytes.fromhex(
        "4c5455310300000000dd0ee902000000090000000100000058000000"
    )
    assert encoded.payload == struct.pack(
        "<ddddddddddB7x",
        1.0,
        2.0,
        0.3,
        0.0,
        0.0,
        0.5,
        0.8660254,
        0.4,
        0.0,
        -0.1,
        1,
    )


def test_encode_registered_cloud_builds_canonical_body_frame_points() -> None:
    points = np.array(
        [[1.0, 2.0, 0.3, 11.0], [-0.5, 0.25, 1.2, 42.0]],
        dtype=np.float32,
    )

    encoded = encode_registered_cloud(
        points,
        timestamp_ns=12_500_000_000,
        sequence=10,
    )

    first = struct.pack("<ffffIBBH", 1.0, 2.0, 0.3, 11.0, 0, 0, 0, 0)
    second = struct.pack("<ffffIBBH", -0.5, 0.25, 1.2, 42.0, 0, 0, 0, 0)
    assert encoded.record_type == 4
    assert encoded.payload == first + second


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("record_type", True),
        ("timestamp_ns", -1),
        ("timestamp_ns", 1 << 64),
        ("sequence", 1.0),
        ("sequence", 1 << 32),
        ("count", False),
        ("count", 1 << 32),
    ],
)
def test_encode_record_rejects_noncanonical_fixed_width_fields(
    field: str, value: object
) -> None:
    values: dict[str, object] = {
        "record_type": RECORD_CLOUD,
        "timestamp_ns": 1,
        "sequence": 1,
        "count": 0,
        "payload": b"",
    }
    values[field] = value

    with pytest.raises((TypeError, ValueError)):
        encode_record(**values)  # type: ignore[arg-type]


def test_typed_encoders_reject_nonfinite_or_inconsistent_sensor_values() -> None:
    imu = Imu(
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(float("nan"), 0.0, 0.0),
        linear_acceleration=Vector3(0.0, 0.0, 9.80665),
        ts=1.0,
        frame_id="imu_link",
    )
    with pytest.raises(ValueError, match="finite"):
        encode_imu(imu, sequence=1)

    points = np.zeros(1, dtype=POINT_DTYPE)
    inconsistent_scan = types.SimpleNamespace(
        points=points,
        timestamp_ns=1,
        sequence=1,
        point_count=2,
    )
    with pytest.raises(ValueError, match="point_count"):
        encode_scan(inconsistent_scan)

    points_xyzi = np.array([[float("inf"), 0.0, 0.0, 1.0]], dtype=np.float32)
    with pytest.raises(ValueError, match="finite"):
        encode_registered_cloud(points_xyzi, timestamp_ns=1, sequence=1)


def test_legacy_writer_delegates_to_the_same_codec_without_wire_drift() -> None:
    from sim.scripts.mujoco import native_dds_sensors as bridge

    imu = Imu(
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.25, -0.5, 1.0),
        linear_acceleration=Vector3(MID360_ACCEL_MPS2_PER_G, 0.0, 0.0),
        ts=1.5,
        frame_id="imu_link",
    )
    stream = BytesIO()

    bridge._write_native_imu(stream, imu, sequence=7)

    assert stream.getvalue() == encode_imu(imu, sequence=7).wire
    assert bridge._MAGIC is LTU1_MAGIC
    assert bridge._HEADER is HEADER
    assert bridge._IMU_PAYLOAD is IMU_PAYLOAD
    assert bridge._ODOM_PRIOR_PAYLOAD is ODOM_PRIOR_PAYLOAD
