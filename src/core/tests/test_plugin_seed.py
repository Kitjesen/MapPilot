from __future__ import annotations

from pathlib import Path

import pytest

pytestmark = [pytest.mark.sim]

import sys

from core.module import Module
from core.registry import clear, get, list_plugins, register, restore, snapshot


def _restore_import_state(module_names: set[str], before: set[str]) -> None:
    """Remove seed-imported modules that were not loaded before this test."""
    for module_name in module_names:
        if module_name in before:
            continue
        sys.modules.pop(module_name, None)


def test_builtin_plugin_seed_restores_core_plugin_surfaces_after_clear():
    from core.plugin_seed import registered_plugin_catalog_names
    from lingtu_runtime.plugin_seed import BUILTIN_PLUGIN_MODULES, seed_builtin_plugins

    assert "lingtu_builtin" in registered_plugin_catalog_names()

    saved = snapshot()
    seed_modules = {module for modules in BUILTIN_PLUGIN_MODULES.values() for module in modules}
    modules_before = set(sys.modules)
    try:
        clear()

        report = seed_builtin_plugins(
            groups=(
                "driver",
                "driver_ros2",
                "lidar",
                "map",
                "map_ros2",
                "planner_backend",
                "navigation",
                "navigation_ros2",
                "navigation_lcm",
                "autonomy",
                "slam",
                "slam_ros2",
                "slam_lcm",
                "exploration",
                "exploration_ros2",
                "perception",
                "reconstruction",
                "llm",
            ),
            reload_loaded=True,
        )

        assert set(BUILTIN_PLUGIN_MODULES) >= {
            "driver",
            "driver_ros2",
            "lidar",
            "map",
            "map_ros2",
            "planner_backend",
            "navigation",
            "navigation_ros2",
            "navigation_lcm",
            "autonomy",
            "slam",
            "slam_ros2",
            "slam_lcm",
            "exploration",
            "exploration_ros2",
            "perception",
            "reconstruction",
            "llm",
        }
        assert set(report) == {"loaded", "failed"}

        assert {"stub", "thunder", "sim_mujoco", "sim_ros2", "nova_dog"} <= set(
            list_plugins("driver")
        )
        assert {"lidar_mid360"} <= set(list_plugins("driver"))
        assert {"mid360"} <= set(list_plugins("lidar"))
        assert {
            "occupancy_grid",
            "voxel",
            "esdf",
            "elevation",
            "traversability_cost",
            "ros2_grid_bridge",
            "manager",
        } <= set(list_plugins("map"))
        assert {"pct", "astar"} <= set(list_plugins("planner_backend"))
        assert {"nanobind", "native", "cmu", "simple"} <= set(list_plugins("terrain"))
        assert {"nanobind", "cmu", "cmu_py", "simple"} <= set(
            list_plugins("local_planner")
        )
        assert {"nav_core", "pure_pursuit", "pid"} <= set(
            list_plugins("path_follower")
        )
        assert {"fastlio2", "pointlio", "localizer", "genz"} <= set(
            list_plugins("slam")
        )
        assert {"ros2_slam_bridge"} <= set(list_plugins("localization_adapter"))
        assert {"default"} <= set(list_plugins("slam_bridge"))
        assert get("localization_adapter", "ros2_slam_bridge") is get(
            "slam_bridge",
            "default",
        )
        assert {"depth"} <= set(list_plugins("visual_odom"))
        seed_builtin_plugins(groups=("sim_lidar",), reload_loaded=True)
        assert {"pointcloud"} <= set(list_plugins("sim_lidar"))
        assert {"tare", "supervisor"} <= set(list_plugins("exploration"))
        assert {
            "default",
            "traversable_frontier",
            "ros2_path_bridge",
            "lcm_navigation_command_bridge",
            "lcm_path_command_bridge",
        } <= set(list_plugins("navigation"))
        assert {"wavefront_frontier"} <= set(list_plugins("exploration"))
        assert {"scene"} <= set(list_plugins("perception"))
        assert {"pluggable"} <= set(list_plugins("encoder"))
        assert {"default", "dataset_recorder", "keyframe_exporter"} <= set(
            list_plugins("reconstruction")
        )
        assert {"yoloe", "yolo_world", "bpu", "sim_scene"} <= set(
            list_plugins("detector")
        )
        assert {"clip", "mobileclip"} <= set(list_plugins("encoder"))
        assert {"openai", "claude", "qwen", "moonshot", "mock"} <= set(
            list_plugins("llm_client")
        )
    finally:
        restore(saved)
        _restore_import_state(seed_modules, modules_before)


def test_builtin_plugin_seed_can_seed_one_group_without_loading_unrelated_groups():
    from lingtu_runtime.plugin_seed import BUILTIN_PLUGIN_MODULES, seed_builtin_plugins

    saved = snapshot()
    seed_modules = {module for modules in BUILTIN_PLUGIN_MODULES.values() for module in modules}
    modules_before = set(sys.modules)
    try:
        clear()

        seed_builtin_plugins(groups=("planner_backend",), reload_loaded=True)

        assert {"pct", "astar"} <= set(list_plugins("planner_backend"))
        assert list_plugins("driver") == []
        assert list_plugins("detector") == []
    finally:
        restore(saved)
        _restore_import_state(seed_modules, modules_before)


def test_builtin_plugin_seed_preserves_preexisting_plugin_registrations():
    from lingtu_runtime.plugin_seed import BUILTIN_PLUGIN_MODULES, seed_builtin_plugins

    saved = snapshot()
    seed_modules = {module for modules in BUILTIN_PLUGIN_MODULES.values() for module in modules}
    modules_before = set(sys.modules)
    try:
        clear()

        @register("map", "occupancy_grid")
        class FakeOccupancyGrid(Module, layer=2):
            pass

        seed_builtin_plugins(groups=("map",), reload_loaded=True)

        assert get("map", "occupancy_grid") is FakeOccupancyGrid
        assert {"voxel", "esdf", "elevation", "traversability_cost"} <= set(
            list_plugins("map")
        )
    finally:
        restore(saved)
        _restore_import_state(seed_modules, modules_before)


def test_driver_plugin_seed_does_not_mutate_sys_path():
    from lingtu_runtime.plugin_seed import BUILTIN_PLUGIN_MODULES, seed_builtin_plugins

    saved = snapshot()
    seed_modules = {module for modules in BUILTIN_PLUGIN_MODULES.values() for module in modules}
    modules_before = set(sys.modules)
    sys_path_before = list(sys.path)
    try:
        clear()

        seed_builtin_plugins(groups=("driver",), reload_loaded=True)

        assert sys.path == sys_path_before
        assert {"stub", "thunder", "sim_mujoco", "nova_dog"} <= set(list_plugins("driver"))
        assert "sim_ros2" not in list_plugins("driver")

        seed_builtin_plugins(groups=("driver_ros2",), reload_loaded=True)
        assert {"sim_ros2"} <= set(list_plugins("driver"))
    finally:
        sys.path[:] = sys_path_before
        restore(saved)
        _restore_import_state(seed_modules, modules_before)


def test_slam_plugin_seed_does_not_import_cv2_for_registration_only():
    from lingtu_runtime.plugin_seed import BUILTIN_PLUGIN_MODULES, seed_builtin_plugins

    saved = snapshot()
    seed_modules = {module for modules in BUILTIN_PLUGIN_MODULES.values() for module in modules}
    modules_before = set(sys.modules)
    cv2_before = sys.modules.get("cv2")
    had_cv2 = "cv2" in sys.modules
    try:
        clear()
        sys.modules.pop("cv2", None)

        seed_builtin_plugins(groups=("slam",), reload_loaded=True)

        assert "cv2" not in sys.modules
        assert {"depth"} <= set(list_plugins("visual_odom"))
    finally:
        if had_cv2:
            sys.modules["cv2"] = cv2_before
        else:
            sys.modules.pop("cv2", None)
        restore(saved)
        _restore_import_state(seed_modules, modules_before)


def test_builtin_plugin_seed_default_groups_skip_optional_runtime_surfaces():
    from lingtu_runtime.plugin_seed import (
        BUILTIN_PLUGIN_MODULES,
        DEFAULT_BUILTIN_PLUGIN_GROUPS,
        seed_builtin_plugins,
    )

    saved = snapshot()
    seed_modules = {module for modules in BUILTIN_PLUGIN_MODULES.values() for module in modules}
    modules_before = set(sys.modules)
    try:
        clear()

        seed_builtin_plugins(reload_loaded=True)

        assert "driver" in DEFAULT_BUILTIN_PLUGIN_GROUPS
        assert "planner_backend" in DEFAULT_BUILTIN_PLUGIN_GROUPS
        assert "map_save_adapter" not in DEFAULT_BUILTIN_PLUGIN_GROUPS
        assert "camera_ros2" not in DEFAULT_BUILTIN_PLUGIN_GROUPS
        assert not any(group.endswith("_ros2") for group in DEFAULT_BUILTIN_PLUGIN_GROUPS)
        assert "navigation_lcm" not in DEFAULT_BUILTIN_PLUGIN_GROUPS
        assert "slam_lcm" not in DEFAULT_BUILTIN_PLUGIN_GROUPS
        assert "gateway" not in DEFAULT_BUILTIN_PLUGIN_GROUPS
        assert "visualization" not in DEFAULT_BUILTIN_PLUGIN_GROUPS
        assert "webrtc" not in DEFAULT_BUILTIN_PLUGIN_GROUPS
        assert {"stub", "thunder"} <= set(list_plugins("driver"))
        assert "sim_ros2" not in list_plugins("driver")
        assert "ros2_grid_bridge" not in list_plugins("map")
        assert "ros2_path_bridge" not in list_plugins("navigation")
        assert "lcm_path_command_bridge" not in list_plugins("navigation")
        assert "ros2_slam_bridge" not in list_plugins("localization_adapter")
        assert "lcm_endpoint" not in list_plugins("localization_adapter")
        assert list_plugins("map_save_adapter") == []
        assert {"pct", "astar"} <= set(list_plugins("planner_backend"))
        assert {"ring", "cmd_vel_mux", "geofence"} <= set(list_plugins("safety"))
        assert list_plugins("gateway") == []
        assert list_plugins("webrtc") == []
    finally:
        restore(saved)
        _restore_import_state(seed_modules, modules_before)


def test_core_plugin_seed_has_no_lingtu_product_catalog() -> None:
    import core.plugin_seed as plugin_seed

    source = Path(plugin_seed.__file__).read_text(encoding="utf-8-sig")

    assert "BUILTIN_PLUGIN_MODULES" not in source
    assert "lingtu_runtime" not in source
    assert "compat.ros2" not in source
    assert "drivers.real.thunder" not in source
    assert "nav.occupancy_grid_module" not in source
    assert hasattr(plugin_seed, "seed_plugin_modules")
