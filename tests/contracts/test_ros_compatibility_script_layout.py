"""Contracts for explicitly quarantined ROS2 compatibility scripts."""

# ruff: noqa: S101 - pytest contracts use assert statements by design.

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ROS2_COMPAT_ROOT = ROOT / "scripts" / "compat" / "ros2"


def test_ros2_bag_converter_lives_only_in_compatibility_namespace() -> None:
    """Offline ROS2 bag conversion must not look like a default script entrypoint."""

    legacy = ROOT / "scripts" / "datasets" / "ros2_bag_to_normalized_jsonl.py"
    isolated = ROS2_COMPAT_ROOT / "datasets" / "ros2_bag_to_normalized_jsonl.py"

    assert not legacy.exists()
    assert isolated.is_file()


def test_ros2_live_perception_demos_live_only_in_compatibility_namespace() -> None:
    """ROS camera demos must be visibly separate from Product perception scripts."""

    for filename in ("live_detect.py", "live_track.py"):
        legacy = ROOT / "scripts" / "perception" / filename
        isolated = ROS2_COMPAT_ROOT / "perception" / filename

        assert not legacy.exists()
        assert isolated.is_file()


def test_native_rerun_viewers_remain_outside_ros2_compatibility_namespace() -> None:
    """Native Gateway and CycloneDDS viewers must not be mislabeled as ROS2 tools."""

    native = ROOT / "scripts" / "visualization" / "rerun_live.py"
    isolated = ROS2_COMPAT_ROOT / "visualization" / "rerun_live.py"
    native_content = native.read_text(encoding="utf-8")

    assert native.is_file()
    assert not isolated.exists()
    assert (ROOT / "scripts" / "visualization" / "rerun_gateway_live.py").is_file()
    assert "DDSReader" in native_content
    assert "import rclpy" not in native_content


def test_scripts_index_separates_default_commands_from_ros2_compatibility() -> None:
    """The canonical scripts index must route ROS2 tooling through one boundary."""

    scripts_index = (ROOT / "scripts" / "README.md").read_text(encoding="utf-8")
    common_commands = scripts_index.split("## Common Commands", maxsplit=1)[1].split(
        "\n## ",
        maxsplit=1,
    )[0]
    compatibility_index = ROS2_COMPAT_ROOT / "README.md"

    assert "Legacy ROS" not in common_commands
    assert "--ros2" not in common_commands
    assert "scripts/compat/ros2/README.md" in scripts_index
    assert compatibility_index.is_file()

    compatibility = compatibility_index.read_text(encoding="utf-8")
    for migrated in (
        "datasets/ros2_bag_to_normalized_jsonl.py",
        "hardware/record_bag.sh",
        "perception/live_detect.py",
        "perception/live_track.py",
    ):
        assert migrated in compatibility
    for deferred in (
        "scripts/build/build_ros_workspace.sh",
        "scripts/build/clone_orbbec_ros2.sh",
        "scripts/deploy/thunder/ros2-env.sh",
    ):
        assert deferred in compatibility

    assert not (ROOT / "scripts" / "hardware" / "record_bag.sh").exists()


def test_native_release_excludes_ros2_compatibility_namespace() -> None:
    """Explicit compatibility tools must not ship in the native Product artifact."""

    package_script = (ROOT / "scripts" / "deploy" / "package_native_release.sh").read_text(
        encoding="utf-8"
    )

    assert "--exclude='/scripts/compat/ros2/***'" in package_script

def test_retired_ota_tree_is_absent_and_native_release_is_canonical() -> None:
    """The removed ROS OTA implementation must not return beside native releases."""

    ota_root = ROOT / "scripts" / "ota"
    package_script = (ROOT / "scripts" / "deploy" / "package_native_release.sh").read_text(
        encoding="utf-8"
    )
    installer = (ROOT / "scripts" / "deploy" / "install_native_release.sh").read_text(
        encoding="utf-8"
    )
    scripts_index = (ROOT / "scripts" / "README.md").read_text(encoding="utf-8")
    release_guide = (ROOT / "docs" / "04-deployment" / "OTA_GUIDE.md").read_text(
        encoding="utf-8"
    )

    assert not ota_root.exists()
    assert 'INSTALLER_SOURCE="${SCRIPT_ROOT}/scripts/deploy/install_native_release.sh"' in package_script
    assert "ProductControl" in installer
    assert "scripts/ota/" not in scripts_index
    assert "scripts/deploy/package_native_release.sh" in release_guide
    for retired in ("build_nav_package.sh", "deploy_to_robot.sh", "generate_manifest.py"):
        assert retired not in scripts_index
        assert retired not in release_guide


def test_legacy_s100p_ros2_systemd_boot_seam_is_absent() -> None:
    """Retired ROS2 systemd installers and units must not return."""

    legacy_root = ROOT / "scripts" / "deploy" / "s100p"
    retired_paths = (
        legacy_root / "install_services.sh",
        legacy_root / "lidar.service",
        legacy_root / "slam.service",
        legacy_root / "slam_pgo.service",
        legacy_root / "localizer.service",
        legacy_root / "genz_icp.service",
        legacy_root / "hba.service",
    )

    assert not [path.relative_to(ROOT) for path in retired_paths if path.exists()]
