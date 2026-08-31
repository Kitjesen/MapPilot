from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_livox_vendor_dependencies_are_below_deps() -> None:
    real = ROOT / "src" / "drivers" / "real" / "lidar"
    ros2_adapter = ROOT / "src" / "drivers" / "adapters" / "ros2" / "lidar"

    assert not (real / "Livox-SDK2").exists()
    assert not (real / "livox_ros_driver2").exists()
    assert (real / "deps" / "livox" / "Livox-SDK2").is_dir()
    assert not (real / "deps" / "livox" / "livox_ros_driver2").exists()
    assert not ros2_adapter.exists()


def test_livox_sdk2_build_points_to_deps() -> None:
    cmake = (ROOT / "src" / "drivers" / "real" / "lidar" / "sdk2_stream" / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "../deps/livox/Livox-SDK2" in cmake
    assert "../native/dds_module.cpp" in cmake


def test_livox_product_stream_is_cpp_and_ros_free() -> None:
    real = ROOT / "src" / "drivers" / "real" / "lidar"
    stream = (real / "sdk2_stream" / "main.cpp").read_text(encoding="utf-8-sig")
    dds_module = (real / "native" / "dds_module.cpp").read_text(encoding="utf-8")

    assert "int main(" in stream
    assert "livox_lidar_api.h" in stream
    assert "native/dds_module.hpp" in stream
    assert "dds_create_participant" not in stream
    assert "dds_create_participant" in dds_module
    assert "#include <rclcpp" not in stream
    assert "#include <rclcpp" not in dds_module
    assert "ros2 run" not in stream
    assert "livox_ros_driver2_node" not in stream
