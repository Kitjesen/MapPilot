from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NATIVE_SCRIPT = ROOT / "sim" / "scripts" / "setup_linux_validation_host.sh"
ROS_COMPAT_SCRIPT = (
    ROOT / "scripts" / "compat" / "ros2" / "setup_fastlio2_validation_host.sh"
)
LEGACY_SCRIPT = ROOT / "scripts" / "deploy" / "setup_server_ros_pct.sh"
LEGACY_LOCAL_PLANNER_ENV = "LINGTU_RUN_ROS2_LOCAL_PLANNER"


def _script_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_setup_entrypoints_are_split_and_legacy_entry_is_removed():
    assert NATIVE_SCRIPT.is_file()
    assert ROS_COMPAT_SCRIPT.is_file()
    assert not LEGACY_SCRIPT.exists()


def test_native_validation_host_setup_keeps_only_native_gates():
    text = _script_text(NATIVE_SCRIPT)

    assert "activate_conda_env" in text
    assert "install_system_deps" in text
    assert "install_python_deps" in text
    assert "build_pct_runtime" in text
    assert "build_nav_kernel_runtime" in text
    assert "verify_mid360_pattern_asset" in text
    assert "sim/scripts/multifloor_nav_validation.py" in text
    assert "sim/scripts/routecheck_preflight_gate.py" in text
    assert "sim/scripts/server_sim_closure.py" in text


def test_native_validation_host_setup_has_no_ros_compatibility_lane():
    text = _script_text(NATIVE_SCRIPT)
    forbidden = (
        "LINGTU_INSTALL_ROS2",
        "LINGTU_RUN_ROS2_FASTLIO2",
        LEGACY_LOCAL_PLANNER_ENV,
        "source_ros_if_present",
        "/opt/ros",
        "colcon",
        "rosdep",
        "ros2 pkg",
        "FishROS",
    )

    for token in forbidden:
        assert token not in text


def test_ros_compat_setup_builds_and_verifies_fastlio2():
    text = _script_text(ROS_COMPAT_SCRIPT)

    assert "detect_ros_distro" in text
    assert "install_ros2_official" in text
    assert "install_ros2_fishros" in text
    assert "source_ros_if_present" in text
    assert "colcon build" in text
    assert "--packages-up-to fastlio2" in text
    assert "ros2 pkg executables fastlio2" in text
    assert "fastlio2[[:space:]]+lio_node" in text
    assert "ros2 pkg prefix livox_ros_driver2" in text


def test_ros_compat_setup_has_no_native_validation_lane():
    text = _script_text(ROS_COMPAT_SCRIPT).lower()

    for token in (
        "run_pct",
        "run_mujoco",
        "run_nav_kernel",
        "run_multifloor",
        "run_routecheck_preflight",
        "pct",
        "mujoco",
        "nav_kernel",
        "multifloor",
        "routecheck",
    ):
        assert token not in text


def test_retired_ros_local_planner_setup_lane_is_not_migrated():
    assert LEGACY_LOCAL_PLANNER_ENV not in _script_text(NATIVE_SCRIPT)
    assert LEGACY_LOCAL_PLANNER_ENV not in _script_text(ROS_COMPAT_SCRIPT)