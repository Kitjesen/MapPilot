from __future__ import annotations

from tools.validate.validate_architecture_boundaries import (
    validate_ros_coupling_touchpoints,
    validate_ros_import_boundaries,
)


def test_ros_imports_stay_inside_explicit_compat_boundaries() -> None:
    violations, scanned, classified = validate_ros_import_boundaries()

    assert scanned > 0
    assert classified == 0
    assert violations == [], "\n".join(violations)


def test_ros_cli_and_setup_coupling_stays_inside_compat_allowlist() -> None:
    violations, scanned = validate_ros_coupling_touchpoints()

    assert scanned > 0
    assert violations == [], "\n".join(violations)
