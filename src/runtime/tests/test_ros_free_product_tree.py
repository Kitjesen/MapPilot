"""Regression guards for the ROS-free LingTu product runtime."""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def test_product_source_tree_has_no_owned_ros2_adapter_packages() -> None:
    retired_paths = (
        "src/drivers/adapters/ros2",
        "src/localization/adapters/ros2",
        "src/perception/adapters/ros2",
        "src/runtime/adapters/ros2",
        "src/runtime/adapters/navigation_io.py",
        "src/runtime/blueprints/stacks/navigation_io.py",
        "src/nav/adapters/dds/nav",
        "src/lingtu/ros2_plugin_seed.py",
        "src/lingtu/ros2_shutdown.py",
        "src/localization/bridge.py",
        "src/localization/relocalization.py",
        "src/localization/fastlio2_nav_bridge.py",
    )

    leftovers = [path for path in retired_paths if (ROOT / path).exists()]
    assert leftovers == []


def test_product_plugin_catalog_has_no_ros2_compat_loader() -> None:
    source = (ROOT / "src/lingtu/plugin_seed.py").read_text(encoding="utf-8")

    assert "ros2_plugin_seed" not in source
    assert "LINGTU_ENABLE_ROS2_COMPAT" not in source
    assert "LINGTU_ENABLE_LEGACY_ROS2_SERVICES" not in source


def test_cli_shutdown_has_no_ros2_runtime_hook() -> None:
    source = (ROOT / "cli/main.py").read_text(encoding="utf-8")

    assert "ros2_shutdown" not in source
    assert "shutdown_ros2_runtime" not in source


def test_python_dds_reader_dependencies_are_explicit_and_bounded() -> None:
    """Do not let the optional cyclonedds-python surface spread silently."""

    expected = {
        "src/drivers/real/imu/dds_module.py",
        "src/drivers/real/lidar/compat/dds.py",
        "src/drivers/real/lidar/compat/dds_adapter.py",
        "src/localization/gnss_module.py",
        "src/nav/adapters/dds/tare_bridge.py",
    }
    users = set()
    for path in (ROOT / "src").rglob("*.py"):
        relative = path.relative_to(ROOT).as_posix()
        if "/tests/" in relative or relative == "src/runtime/adapters/dds/reader.py":
            continue
        source = path.read_text(encoding="utf-8-sig")
        if "runtime.adapters.dds.reader" not in source:
            continue
        tree = ast.parse(source, filename=str(path))
        if any(
            isinstance(node, ast.ImportFrom) and node.module == "runtime.adapters.dds.reader" for node in ast.walk(tree)
        ):
            users.add(relative)

    assert users == expected
