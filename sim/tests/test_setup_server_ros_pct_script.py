from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "deploy" / "setup_server_ros_pct.sh"


def _script_text() -> str:
    return SCRIPT.read_text(encoding="utf-8")


def test_setup_server_ros_pct_ignores_deprecated_ros2_local_planner_package():
    text = _script_text()

    assert "RUN_ROS2_LOCAL_PLANNER" in text
    assert "build_ros2_local_planner_runtime" in text
    assert "LINGTU_RUN_ROS2_LOCAL_PLANNER=1 is deprecated and ignored" in text
    assert "nav_kernel/nanobind is the production local-planning runtime" in text
    assert "--packages-up-to local_planner" not in text
    assert '|| "${RUN_ROS2_LOCAL_PLANNER}" == "1"' not in text
    assert "build_ros2_local_planner_runtime \"${distro}\"" in text


def test_setup_server_ros_pct_builds_ros2_fastlio2_package():
    text = _script_text()

    assert "RUN_ROS2_FASTLIO2" in text
    assert "build_ros2_fastlio2_runtime" in text
    assert "colcon build" in text
    assert "--packages-up-to fastlio2" in text
    assert "--merge-install" in text
    assert "build_ros2_fastlio2_runtime \"${distro}\"" in text


def test_setup_server_ros_pct_does_not_verify_legacy_local_planner_executables():
    text = _script_text()

    assert "verify_ros2_local_planner_runtime" not in text
    assert "ros2 pkg executables local_planner" not in text
    assert "ros2 pkg executables pct_adapters" not in text
    assert "localPlanner pathFollower" not in text
    assert "local_planner executable missing" not in text
    assert "pct_adapters executable missing: pct_path_adapter" not in text


def test_setup_server_ros_pct_verifies_fastlio2_executable_and_livox_dependency():
    text = _script_text()

    assert "verify_ros2_fastlio2_runtime" in text
    assert "ros2 pkg executables fastlio2" in text
    assert "fastlio2[[:space:]]+lio_node" in text
    assert "ros2 pkg prefix livox_ros_driver2" in text
    assert "fastlio2 executable missing: lio_node" in text


def test_setup_server_ros_pct_verifies_mid360_pattern_asset():
    text = _script_text()

    assert "verify_mid360_pattern_asset" in text
    assert "sim/assets/livox/mid360.npy" in text
    assert "MID360_PATTERN_SHA256" in text
    assert "MID-360 pattern SHA256 mismatch" in text
    assert "verify_mid360_pattern_asset" in text.split("run_verification", 1)[0]
