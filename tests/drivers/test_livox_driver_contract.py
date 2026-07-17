from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / "src" / "drivers"
SDK_ROOT = ROOT / "real" / "lidar" / "deps" / "livox" / "Livox-SDK2"


def test_custom_sdk2_tcp_endpoint_is_removed() -> None:
    assert not (SDK_ROOT / "lingtu_tcp").exists()
    assert not (ROOT / "real" / "lidar" / "sdk2_source.py").exists()
    assert not (ROOT / "real" / "lidar" / "wire.py").exists()


def test_lingtu_lidar_uses_official_livox_dds_message_contract() -> None:
    real_lidar = ROOT / "real" / "lidar"
    dds = (real_lidar / "compat" / "dds.py").read_text(encoding="utf-8-sig")
    source = (real_lidar / "native" / "sdk.py").read_text(encoding="utf-8-sig")

    assert not (real_lidar / "_dds.py").exists()
    assert not (real_lidar / "source.py").exists()
    assert "livox_ros_driver2::msg::dds_::CustomMsg_" in dds
    assert "sdk2_tcp" not in source
    assert "Sdk2TcpSource" not in source


def test_sdk2_stream_enables_livox_imu_without_ros() -> None:
    stream = (ROOT / "real" / "lidar" / "sdk2_stream" / "main.cpp").read_text(encoding="utf-8-sig")
    module = (ROOT / "real" / "lidar" / "native" / "module.hpp").read_text(encoding="utf-8-sig")

    assert "#include <rclcpp" not in stream
    assert "SetLivoxLidarImuDataCallback(imu_callback" in stream
    assert "EnableLivoxLidarImuData(handle" in stream
    assert "--imu-publish-freq" in stream
    assert "g_imu_min_interval_ns" in stream
    assert 'std::string lidar_frame{"lidar_link"}' in module


def test_sdk2_stream_has_bounded_timestamp_paced_replay() -> None:
    stream = (ROOT / "real" / "lidar" / "sdk2_stream" / "main.cpp").read_text(encoding="utf-8-sig")
    module = (ROOT / "real" / "lidar" / "native" / "module.hpp").read_text(encoding="utf-8-sig")

    assert "--validate-records" in stream
    assert "--replay-rate" in stream
    assert "kMaxReplayPayloadBytes" in stream
    assert "std::this_thread::sleep_until" in stream
    assert "stdin cloud payload truncated" in stream
    assert "validate_records" in module
    assert "replay_rate" in module
