from __future__ import annotations

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
    from core.plugin_seed import BUILTIN_PLUGIN_MODULES, seed_builtin_plugins

    saved = snapshot()
    seed_modules = {module for modules in BUILTIN_PLUGIN_MODULES.values() for module in modules}
    modules_before = set(sys.modules)
    try:
        clear()

        report = seed_builtin_plugins(
            groups=(
                "driver",
                "lidar",
                "map",
                "planner_backend",
                "navigation",
                "autonomy",
                "slam",
                "exploration",
                "perception",
                "reconstruction",
                "llm",
            ),
            reload_loaded=True,
        )

        assert set(BUILTIN_PLUGIN_MODULES) >= {
            "driver",
            "lidar",
            "map",
            "planner_backend",
            "navigation",
            "autonomy",
            "slam",
            "exploration",
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
        assert {"default"} <= set(list_plugins("slam_bridge"))
        assert {"depth"} <= set(list_plugins("visual_odom"))
        seed_builtin_plugins(groups=("sim_lidar",), reload_loaded=True)
        assert {"pointcloud"} <= set(list_plugins("sim_lidar"))
        assert {"tare", "supervisor"} <= set(list_plugins("exploration"))
        assert {"default", "traversable_frontier", "ros2_path_bridge"} <= set(
            list_plugins("navigation")
        )
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
    from core.plugin_seed import BUILTIN_PLUGIN_MODULES, seed_builtin_plugins

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
    from core.plugin_seed import BUILTIN_PLUGIN_MODULES, seed_builtin_plugins

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
    from core.plugin_seed import BUILTIN_PLUGIN_MODULES, seed_builtin_plugins

    saved = snapshot()
    seed_modules = {module for modules in BUILTIN_PLUGIN_MODULES.values() for module in modules}
    modules_before = set(sys.modules)
    sys_path_before = list(sys.path)
    try:
        clear()

        seed_builtin_plugins(groups=("driver",), reload_loaded=True)

        assert sys.path == sys_path_before
        assert {"stub", "thunder", "sim_mujoco", "sim_ros2", "nova_dog"} <= set(
            list_plugins("driver")
        )
    finally:
        sys.path[:] = sys_path_before
        restore(saved)
        _restore_import_state(seed_modules, modules_before)


def test_slam_plugin_seed_does_not_import_cv2_for_registration_only():
    from core.plugin_seed import BUILTIN_PLUGIN_MODULES, seed_builtin_plugins

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
    from core.plugin_seed import (
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
        assert "gateway" not in DEFAULT_BUILTIN_PLUGIN_GROUPS
        assert "visualization" not in DEFAULT_BUILTIN_PLUGIN_GROUPS
        assert "webrtc" not in DEFAULT_BUILTIN_PLUGIN_GROUPS
        assert {"stub", "thunder"} <= set(list_plugins("driver"))
        assert {"pct", "astar"} <= set(list_plugins("planner_backend"))
        assert {"ring", "cmd_vel_mux", "geofence"} <= set(list_plugins("safety"))
        assert list_plugins("gateway") == []
        assert list_plugins("webrtc") == []
    finally:
        restore(saved)
        _restore_import_state(seed_modules, modules_before)
