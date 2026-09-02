"""Regression guards for the native field Product runtime.

ROS2 compatibility, replay, calibration, simulation, and diagnostic tools are
intentionally retained outside this Product-owned runtime tree.
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_product_source_tree_has_no_owned_ros2_adapter_packages() -> None:
    retired_paths = (
        "src/drivers/adapters/ros2",
        "src/localization/adapters/ros2",
        "src/perception/adapters/ros2",
        "src/runtime/adapters/ros2",
        "src/runtime/adapters/navigation_io.py",
        "src/lingtu/assembly/stacks/navigation_io.py",
        "src/nav/adapters/dds/nav",
        "src/lingtu/ros2_plugin_seed.py",
        "src/lingtu/ros2_shutdown.py",
        "src/localization/bridge.py",
        "src/localization/relocalization.py",
        "src/localization/fastlio2_nav_bridge.py",
    )

    leftovers = [path for path in retired_paths if (ROOT / path).exists()]
    assert leftovers == []


def test_retired_ros2_tare_adapter_stays_removed() -> None:
    retired_paths = ("src/nav/adapters/ros2/tare_bridge.py",)

    leftovers = [path for path in retired_paths if (ROOT / path).exists()]
    assert leftovers == []


def test_product_plugin_catalog_has_no_ros2_compat_loader() -> None:
    source = (ROOT / "src/lingtu/assembly/plugins.py").read_text(encoding="utf-8")

    assert "ros2_plugin_seed" not in source
    assert "LINGTU_ENABLE_ROS2_COMPAT" not in source
    assert "LINGTU_ENABLE_LEGACY_ROS2_SERVICES" not in source


def test_product_control_has_no_ros2_runtime_hook() -> None:
    source = (ROOT / "src/lingtu/control.py").read_text(encoding="utf-8")

    assert "ros2_shutdown" not in source
    assert "shutdown_ros2_runtime" not in source


def test_product_python_has_no_cyclonedds_runtime() -> None:
    users = []
    for path in (ROOT / "src").rglob("*.py"):
        if "/tests/" in path.as_posix():
            continue
        source = path.read_text(encoding="utf-8-sig")
        if "import cyclonedds" in source or "from cyclonedds" in source:
            users.append(path.relative_to(ROOT).as_posix())
    assert users == []
