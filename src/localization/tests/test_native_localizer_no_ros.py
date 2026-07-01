from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
NATIVE_DIR = ROOT / "src" / "localization" / "native_localizer"
FASTLIO2_DIR = ROOT / "src" / "localization" / "fastlio2"


def test_native_localizer_build_files_do_not_depend_on_ros():
    forbidden = (
        "ament_cmake",
        "rclcpp",
        "sensor_msgs",
        "nav_msgs",
        "tf2_ros",
        "std_srvs",
        "pcl_conversions",
    )

    for path in (
        NATIVE_DIR / "CMakeLists.txt",
        NATIVE_DIR / "localizer_cli.cpp",
    ):
        text = path.read_text(encoding="utf-8")
        hits = [token for token in forbidden if token in text]
        assert hits == []


def test_fastlio2_core_cmake_does_not_require_ros():
    text = (FASTLIO2_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    core_section = text.split("if(NOT FASTLIO2_BUILD_ROS_NODE)", 1)[0]

    assert "add_library(fastlio2_core" in core_section
    assert "ament_cmake" not in core_section
    assert "rclcpp" not in core_section
    assert "sensor_msgs" not in core_section
