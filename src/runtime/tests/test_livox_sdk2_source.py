from __future__ import annotations

import io
from pathlib import Path

import pytest

from drivers.real.lidar.frames import POINT_DTYPE, LivoxPointFrame
from drivers.real.lidar.sdk2_stream_source import (
    _HEADER,
    _IMU_PAYLOAD,
    _MAGIC,
    _RECORD_CLOUD,
    _RECORD_IMU,
    _parse_record,
)
from drivers.real.lidar.source import create_lidar_source
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu


def test_create_lidar_source_selects_sdk2_source() -> None:
    source = create_lidar_source(ip="192.168.1.115")

    assert source.__class__.__name__ == "Sdk2Source"
    assert source.ip == "192.168.1.115"


def test_sdk2_cloud_record_preserves_livox_metadata() -> None:
    points = np.zeros(2, dtype=POINT_DTYPE)
    points["x"] = [1.0, 2.0]
    points["y"] = [3.0, 4.0]
    points["z"] = [5.0, 6.0]
    points["intensity"] = [7.0, 8.0]
    points["offset_time_ns"] = [0, 1250]
    points["tag"] = [11, 12]
    points["line"] = [0, 3]
    payload = points.tobytes()
    header = _HEADER.pack(_MAGIC, _RECORD_CLOUD, 1_234_000_000, 42, len(points), len(payload))

    frame = _parse_record(io.BytesIO(payload), header)

    assert isinstance(frame, LivoxPointFrame)
    assert frame.timestamp_ns == 1_234_000_000
    assert frame.sequence == 42
    assert frame.point_count == 2
    assert frame.points["offset_time_ns"].tolist() == [0, 1250]
    assert frame.points["tag"].tolist() == [11, 12]
    assert frame.points["line"].tolist() == [0, 3]
    assert frame.to_xyzi().shape == (2, 4)


def test_sdk2_imu_record_maps_to_runtime_imu() -> None:
    payload = _IMU_PAYLOAD.pack(0.1, 0.2, 0.3, 9.7, 0.0, -0.1)
    header = _HEADER.pack(_MAGIC, _RECORD_IMU, 2_500_000_000, 7, 1, len(payload))

    imu = _parse_record(io.BytesIO(payload), header)

    assert isinstance(imu, Imu)
    assert imu.ts == pytest.approx(2.5)
    assert imu.angular_velocity.x == pytest.approx(0.1)
    assert imu.angular_velocity.y == pytest.approx(0.2)
    assert imu.angular_velocity.z == pytest.approx(0.3)
    assert imu.linear_acceleration.x == pytest.approx(9.7)


def test_sdk2_stream_declares_optional_native_dds_publisher() -> None:
    cmake = Path("src/drivers/real/lidar/sdk2_stream/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    main = Path("src/drivers/real/lidar/sdk2_stream/main.cpp").read_text(
        encoding="utf-8"
    )
    build_script = Path("scripts/build/build_livox_sdk2_stream.sh").read_text(
        encoding="utf-8"
    )

    assert "LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS" in cmake
    assert "CycloneDDS::ddsc" in cmake
    assert "CycloneDDS-CXX" not in cmake
    assert "src/message/idl/lingtu_slam.idl" in cmake
    assert "LINGTU_LIVOX_SDK2_STREAM_HAS_DDS=1" in cmake
    assert "#include \"dds/dds.h\"" in main
    assert "dds_create_writer" in main
    assert "lingtu_dds_LivoxFrame_desc" in main
    assert "lingtu_dds_Imu_desc" in main
    assert "kLidarRawPacket" in main
    assert "ScanAccumulator" in main
    assert "--publish-freq" in main
    assert "--dds" in main
    assert "LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS" in build_script
    assert 'LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS:-ON' in build_script
    assert "LINGTU_CYCLONEDDS_PREFIX" in build_script
