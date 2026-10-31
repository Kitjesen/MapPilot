from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RELOCALIZATION_DIR = ROOT / "src" / "localization" / "relocalization"
SLAM_CMAKE = ROOT / "src" / "localization" / "slam" / "cpp" / "CMakeLists.txt"
LTLIO_DIR = ROOT / "src" / "localization" / "ltlio"


def test_relocalization_is_the_single_ros_free_algorithm_owner():
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
        RELOCALIZATION_DIR / "CMakeLists.txt",
        RELOCALIZATION_DIR / "src" / "relocalization_cli.cpp",
    ):
        text = path.read_text(encoding="utf-8")
        hits = [token for token in forbidden if token in text]
        assert hits == []

    cmake = (RELOCALIZATION_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "add_library(relocalization" in cmake
    assert "add_executable(relocalization_cli" in cmake
    assert "target_link_libraries(relocalization_cli PRIVATE relocalization)" in cmake
    assert "lingtu_localizer_core" not in cmake


def test_slam_links_relocalization_instead_of_recompiling_its_sources():
    cmake = SLAM_CMAKE.read_text(encoding="utf-8")

    assert 'add_subdirectory("${RELOCALIZATION_DIR}"' in cmake
    assert "PRIVATE relocalization" in cmake
    assert '"${RELOCALIZATION_DIR}/src/localizers/commons.cpp"' not in cmake
    assert '"${RELOCALIZATION_DIR}/src/localizers/icp_localizer.cpp"' not in cmake
    assert '"${RELOCALIZATION_DIR}/src/localizers/bbs3d_global_localizer.cpp"' not in cmake


def test_retired_ros_relocalization_surfaces_stay_removed():
    for path in (
        ROOT / "src" / "localization" / "native_localizer",
        RELOCALIZATION_DIR / "package.xml",
        RELOCALIZATION_DIR / "src" / "localizer_node.cpp",
        RELOCALIZATION_DIR / "config",
        RELOCALIZATION_DIR / "rviz",
    ):
        assert not path.exists(), path


def test_native_slam_ci_tracks_and_uploads_the_current_relocalization_targets():
    workflow = (ROOT / ".github" / "workflows" / "slam-aarch64-build.yml").read_text(
        encoding="utf-8"
    )

    assert workflow.count("src/localization/relocalization/**") == 2
    assert "build/slam_core/slamd" in workflow
    assert "build/slam_core/slamctl" in workflow
    assert "messages_cyclone_runtime" not in workflow
    assert "messages_control" not in workflow


def test_ltlio_is_a_native_core_without_ros_surfaces():
    cmake = (LTLIO_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "add_library(ltlio_core" in cmake

    forbidden = (
        "LTLIO_BUILD_ROS_NODE",
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
        for path in LTLIO_DIR.rglob("*")
        if path.is_file()
    )
    assert [token for token in forbidden if token in source_text] == []

    for retired in (
        LTLIO_DIR / "package.xml",
        LTLIO_DIR / "COLCON_IGNORE",
        LTLIO_DIR / "rviz",
        LTLIO_DIR / "src" / "lio_node.cpp",
        LTLIO_DIR / "src" / "utils.cpp",
        LTLIO_DIR / "src" / "utils.h",
        LTLIO_DIR / "config" / "lio_velodyne.yaml",
    ):
        assert not retired.exists(), retired


def test_retired_pointlio_source_tree_is_removed():
    assert not (ROOT / "src" / "localization" / "pointlio").exists()
