from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
LOCALIZER_DIR = ROOT / "src" / "localization" / "localizer"
SLAM_CMAKE = ROOT / "src" / "localization" / "slam" / "cpp" / "CMakeLists.txt"
FASTLIO2_DIR = ROOT / "src" / "localization" / "fastlio2"


def test_localizer_is_the_single_ros_free_algorithm_owner():
    forbidden = (
        "ament_cmake",
        "ament_package",
        "rclcpp",
        "sensor_msgs",
        "nav_msgs",
        "tf2_ros",
        "std_srvs",
        "pcl_conversions",
    )

    for path in (
        LOCALIZER_DIR / "CMakeLists.txt",
        LOCALIZER_DIR / "src" / "localizer_cli.cpp",
    ):
        text = path.read_text(encoding="utf-8")
        hits = [token for token in forbidden if token in text]
        assert hits == []

    cmake = (LOCALIZER_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "add_library(localizer" in cmake
    assert "add_executable(localizer_cli" in cmake
    assert "target_link_libraries(localizer_cli PRIVATE localizer)" in cmake
    assert "lingtu_localizer_core" not in cmake


def test_slam_links_localizer_instead_of_recompiling_its_sources():
    cmake = SLAM_CMAKE.read_text(encoding="utf-8")

    assert 'add_subdirectory("${LOCALIZER_DIR}"' in cmake
    assert "PRIVATE localizer" in cmake
    assert '"${LOCALIZER_DIR}/src/localizers/commons.cpp"' not in cmake
    assert '"${LOCALIZER_DIR}/src/localizers/icp_localizer.cpp"' not in cmake
    assert '"${LOCALIZER_DIR}/src/localizers/bbs3d_global_localizer.cpp"' not in cmake


def test_retired_ros_localizer_surfaces_stay_removed():
    for path in (
        ROOT / "src" / "localization" / "native_localizer",
        LOCALIZER_DIR / "package.xml",
        LOCALIZER_DIR / "src" / "localizer_node.cpp",
        LOCALIZER_DIR / "config",
        LOCALIZER_DIR / "rviz",
    ):
        assert not path.exists(), path


def test_native_slam_ci_tracks_and_uploads_the_current_localizer_targets():
    workflow = (ROOT / ".github" / "workflows" / "slam-aarch64-build.yml").read_text(
        encoding="utf-8"
    )

    assert workflow.count("src/localization/localizer/**") == 2
    assert "build/slam_core/slamd" in workflow
    assert "build/slam_core/slamctl" in workflow
    assert "messages_cyclone_runtime" not in workflow
    assert "messages_control" not in workflow


def test_fastlio2_is_a_native_core_without_ros_surfaces():
    cmake = (FASTLIO2_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "add_library(fastlio2_core" in cmake

    forbidden = (
        "FASTLIO2_BUILD_ROS_NODE",
        "ament_cmake",
        "ament_package",
        "rclcpp",
        "sensor_msgs",
        "nav_msgs",
        "tf2_ros",
        "livox_ros_driver2",
        "pcl_conversions",
    )
    source_text = "\n".join(
        path.read_text(encoding="utf-8", errors="replace")
        for path in FASTLIO2_DIR.rglob("*")
        if path.is_file()
    )
    assert [token for token in forbidden if token in source_text] == []

    for retired in (
        FASTLIO2_DIR / "package.xml",
        FASTLIO2_DIR / "COLCON_IGNORE",
        FASTLIO2_DIR / "rviz",
        FASTLIO2_DIR / "src" / "lio_node.cpp",
        FASTLIO2_DIR / "src" / "utils.cpp",
        FASTLIO2_DIR / "src" / "utils.h",
        FASTLIO2_DIR / "config" / "lio_velodyne.yaml",
    ):
        assert not retired.exists(), retired


def test_retired_pointlio_source_tree_is_removed():
    assert not (ROOT / "src" / "localization" / "pointlio").exists()
