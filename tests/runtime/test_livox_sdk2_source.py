from __future__ import annotations

import io
from pathlib import Path

import pytest

from drivers.real.lidar.impl.livox.sdk2_stream_source import (
    _HEADER,
    _IMU_PAYLOAD,
    _MAGIC,
    _RECORD_CLOUD,
    _RECORD_IMU,
    _parse_record,
)
from drivers.real.lidar.native.sdk import create_lidar_source
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import POINT_DTYPE, Imu, LivoxPointFrame


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
    cmake = Path("src/drivers/real/lidar/sdk2_stream/CMakeLists.txt").read_text(encoding="utf-8")
    main = Path("src/drivers/real/lidar/sdk2_stream/main.cpp").read_text(encoding="utf-8")
    dds_module = Path("src/drivers/real/lidar/native/dds_module.cpp").read_text(encoding="utf-8")
    build_script = Path("scripts/build/build_livox_sdk2_stream.sh").read_text(encoding="utf-8")

    assert "LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS" in cmake
    assert "CycloneDDS::ddsc" in cmake
    assert "CycloneDDS-CXX" not in cmake
    assert "src/message/idl/messages.idl" in cmake
    assert "LINGTU_LIVOX_SDK2_STREAM_HAS_DDS=1" in cmake
    assert '#include "native/dds_module.hpp"' in main
    assert '#include "dds/dds.h"' not in main
    assert "dds_create_writer" in dds_module
    assert "lingtu_dds_LivoxFrame_desc" in dds_module
    assert "lingtu_dds_Imu_desc" in dds_module
    assert "kLidarRawPacket" in dds_module
    assert "qos_for_topic(lingtu::message::kLidarRawFrame.dds_topic)" in dds_module
    assert "qos_for_topic(lingtu::message::kLidarRawPacket.dds_topic)" in dds_module
    assert "dds_create_writer(publisher_, lidar_topic_, lidar_qos.get(), nullptr)" in dds_module
    assert (
        "dds_create_writer(publisher_, raw_packet_topic_, raw_packet_qos.get(), nullptr)"
        in dds_module
    )
    assert "make_qos(lingtu::dds::QosProfile::SensorStream)" in dds_module
    assert "dds_create_writer(publisher_, imu_topic_, sensor_qos.get(), nullptr)" in dds_module
    assert (
        "dds_create_writer(publisher_, odom_prior_topic_, sensor_qos.get(), nullptr)"
        in dds_module
    )
    assert "ScanAccumulator" in main
    assert "--publish-freq" in main
    assert "--stdin-records" in main
    assert "run_stdin_records" in main
    assert "--stdin-records requires --dds" in main
    assert "--dds" in main
    assert "LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS" in build_script
    assert "LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS:-ON" in build_script
    assert '-DLIVOX_SDK2_DIR="$ROOT/src/drivers/real/lidar/deps/livox/Livox-SDK2"' in build_script
    assert "LINGTU_CYCLONEDDS_PREFIX" in build_script


def test_native_lidar_dds_write_diagnostics_are_explicit_and_opt_in() -> None:
    dds_module = Path("src/drivers/real/lidar/native/dds_module.cpp").read_text(
        encoding="utf-8"
    )

    assert "LINGTU_LIVOX_DDS_WRITE_DIAGNOSTICS" in dds_module
    assert 'std::strcmp(value, "1") == 0' in dds_module
    assert "dds_get_qos(writer, qos.get())" in dds_module
    assert '\\"profile\\":\\"RawLidarStream\\"' in dds_module
    assert "dds_qget_reliability" in dds_module
    assert "dds_qget_durability" in dds_module
    assert "dds_qget_history" in dds_module
    assert "dds_qget_lifespan" in dds_module
    assert "dds_qget_resource_limits" in dds_module
    assert "std::chrono::steady_clock::now()" in dds_module
    assert "const dds_return_t write_rc = dds_write(writer, &msg)" in dds_module
    assert "dds_get_publication_matched_status(writer, &matched_status)" in dds_module
    assert "kLivoxPointPayloadBytes = 24" in dds_module
    assert "checked(write_rc, \"dds_write(livox)\")" in dds_module
