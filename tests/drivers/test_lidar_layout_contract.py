from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_real_and_sim_lidar_share_layout() -> None:
    real = ROOT / "src" / "drivers" / "real" / "lidar"
    sim = ROOT / "src" / "drivers" / "sim" / "lidar"

    real_root_files = {path.name for path in real.glob("*.py")}
    assert real_root_files == {"__init__.py", "module.py"}

    for base in (real, sim):
        assert (base / "__init__.py").is_file()
        assert (base / "module.py").is_file()
        assert (base / "native").is_dir()
        assert (base / "impl").is_dir()
        assert (base / "deps").is_dir()

    assert (real / "api" / "frames.py").is_file()
    assert (real / "api" / "frame_stream.py").is_file()
    assert not (real / "compat").exists()
    assert (real / "impl" / "livox" / "sdk2_stream_source.py").is_file()
    assert (real / "native" / "model.py").is_file()
    assert (real / "native" / "sdk.py").is_file()
    assert (real / "native" / "module.hpp").is_file()
    assert (real / "native" / "dds_module.hpp").is_file()
    assert (real / "native" / "dds_module.cpp").is_file()
    assert (sim / "impl" / "mujoco" / "lidar.py").is_file()
    assert (sim / "native" / "sdk.py").is_file()


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
