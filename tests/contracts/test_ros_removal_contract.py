"""Keep retired ROS2 compatibility entrypoints out of the Product repository."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_ros2_compatibility_entrypoints_are_absent() -> None:
    assert not (ROOT / "scripts/build/build_ros_workspace.sh").exists()
    compat = ROOT / "scripts/compat/ros2"
    assert not [
        path
        for pattern in ("*.py", "*.sh", "*.md")
        for path in compat.rglob(pattern)
    ]


def test_native_field_tools_do_not_offer_ros2_modes() -> None:
    doctor = (ROOT / "src/diagnostics/field/doctor.py").read_text(encoding="utf-8")
    collector = (ROOT / "scripts/gates/real_runtime_evidence_collect.py").read_text(
        encoding="utf-8"
    )

    for text in (doctor, collector):
        assert "--ros2" not in text
        assert "import rclpy" not in text
        assert "from tf2_ros" not in text


def test_native_release_packager_has_no_ros2_exclusion_policy() -> None:
    package = (ROOT / "scripts/deploy/package_native_release.sh").read_text(encoding="utf-8")

    assert "scripts/compat/ros2" not in package
    assert "build_ros_workspace.sh" not in package
