from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2] / "src" / "drivers"
SDK_ROOT = ROOT / "real" / "lidar" / "Livox-SDK2"


def test_custom_sdk2_tcp_endpoint_is_removed() -> None:
    assert not (SDK_ROOT / "lingtu_tcp").exists()
    assert not (ROOT / "real" / "lidar" / "sdk2_source.py").exists()
    assert not (ROOT / "real" / "lidar" / "wire.py").exists()


def test_lingtu_lidar_uses_official_livox_dds_message_contract() -> None:
    dds = (ROOT / "real" / "lidar" / "_dds.py").read_text(encoding="utf-8-sig")
    source = (ROOT / "real" / "lidar" / "source.py").read_text(encoding="utf-8-sig")

    assert "livox_ros_driver2::msg::dds_::CustomMsg_" in dds
    assert "sdk2_tcp" not in source
    assert "Sdk2TcpSource" not in source


def test_sdk2_stream_enables_livox_imu_without_ros() -> None:
    stream = (ROOT / "real" / "lidar" / "sdk2_stream" / "main.cpp").read_text(
        encoding="utf-8-sig"
    )

    assert "#include <rclcpp" not in stream
    assert "SetLivoxLidarImuDataCallback(imu_callback" in stream
    assert "EnableLivoxLidarImuData(handle" in stream
