from __future__ import annotations

import ast
from pathlib import Path

import pytest

from runtime.blueprints.products.thunder import (
    thunder_basic_blueprint,
    thunder_basic_config,
    thunder_blueprint,
    thunder_explore_config,
    thunder_lite_blueprint,
    thunder_lite_config,
    thunder_map_config,
    thunder_nav_config,
)
from runtime.profiles.catalog.runtime_paths import _resolve_octoplanner3d_map
from lingtu.plugin_seed import install_builtin_plugin_catalog

ROOT = Path(__file__).resolve().parents[3]
install_builtin_plugin_catalog()


def _entry_names(bp) -> set[str]:
    return {entry.name for entry in bp._entries}


def _wire_set(bp) -> set[str]:
    return {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }


def test_thunder_product_configs_use_robot_name_not_board_name():
    for config in (
        thunder_basic_config(),
        thunder_lite_config(),
        thunder_map_config(),
        thunder_nav_config(),
        thunder_explore_config(),
    ):
        assert config["robot"] == "thunder"
        assert config["dog_host"] == "127.0.0.1"
        assert config["auto_enable"] is False
        assert config["auto_standup"] is False


def test_thunder_product_configs_lock_core_runtime_modes():
    basic = thunder_basic_config()
    lite = thunder_lite_config()
    assert basic["runtime_mode"] == "lite"
    assert basic["slam_profile"] == "none"
    assert basic["planner"] == "direct"
    assert basic["enable_semantic"] is False
    assert basic["python_autonomy_backend"] == "simple"
    assert basic["python_path_follower_backend"] == "pid"
    assert basic["enable_gateway"] is False
    assert basic["enable_map_modules"] is False
    assert basic["enable_gnss"] is False
    assert basic["manage_external_services"] is False
    assert basic["run_startup_checks"] is False
    assert lite == basic

    nav = thunder_nav_config()
    assert nav["slam_profile"] == "localizer"
    assert nav["localization_adapter"] == "cpp_slam_status"
    assert "endpoint_contract" not in nav
    assert nav["planner"] == "octoplanner3d"
    assert nav["tomogram"] == _resolve_octoplanner3d_map()
    assert nav["plan_safety_policy"] == "reject"
    assert nav["fallback_planner_name"] == ""
    assert nav["preview_timeout"] == pytest.approx(30.0)
    assert nav["octoplanner3d_timeout_s"] == pytest.approx(30.0)
    assert nav["waypoint_threshold"] == pytest.approx(0.20)
    assert nav["final_waypoint_threshold"] == pytest.approx(0.10)
    assert nav["local_planner_allow_direct_track_fallback"] is True
    assert nav["local_planner_direct_track_fallback_min_distance_m"] == pytest.approx(0.05)
    assert nav["local_planner_min_trackable_local_path_m"] == pytest.approx(0.05)
    assert nav["path_follower_goal_tolerance"] == pytest.approx(0.05)
    assert nav["path_follower_lookahead"] == pytest.approx(0.35)
    assert nav["path_follower_max_speed"] == pytest.approx(0.20)
    assert nav["path_follower_min_speed"] == pytest.approx(0.08)
    assert nav["path_follower_native_max_accel"] == pytest.approx(10.0)
    assert nav["enable_native"] is True
    assert nav["terrain_backend"] == "nanobind"
    assert nav["terrain_strict_native"] is True
    assert nav["local_planner_backend"] == "nanobind"
    assert nav["path_follower_backend"] == "nav_kernel"
    assert nav["octoplanner3d_robot_radius"] == pytest.approx(0.25)
    assert nav["octoplanner3d_require_ground_support"] is True
    assert nav["octoplanner3d_strict_direct_ground_support"] is False
    assert nav["octoplanner3d_ground_support_xy_radius_cells"] == 1
    assert nav["octoplanner3d_ground_support_depth_cells"] == 1
    assert nav["octoplanner3d_max_step_height"] == pytest.approx(0.45)
    assert nav["octoplanner3d_max_slope"] == pytest.approx(0.0)
    assert nav["enable_semantic"] is True
    assert "lidar_transport" not in nav
    assert "lidar_endpoint_host" not in nav
    assert "lidar_endpoint_port" not in nav

    mapping = thunder_map_config()
    assert mapping["slam_profile"] == "fastlio2"
    assert mapping["planner"] == "octoplanner3d"
    assert mapping["tomogram"] == _resolve_octoplanner3d_map()
    assert mapping["plan_safety_policy"] == "reject"
    assert mapping["fallback_planner_name"] == ""
    assert mapping["octoplanner3d_robot_radius"] == pytest.approx(0.25)
    assert mapping["octoplanner3d_max_slope"] == pytest.approx(0.0)
    assert mapping["enable_native"] is True
    assert mapping["terrain_backend"] == "nanobind"
    assert mapping["terrain_strict_native"] is True
    assert mapping["local_planner_backend"] == "nanobind"
    assert mapping["path_follower_backend"] == "nav_kernel"
    assert mapping["enable_semantic"] is False
    assert "lidar_transport" not in mapping

    explore = thunder_explore_config()
    assert explore["slam_profile"] == "fastlio2"
    assert explore["planner"] == "octoplanner3d"
    assert explore["tomogram"] == _resolve_octoplanner3d_map()
    assert explore["plan_safety_policy"] == "reject"
    assert explore["fallback_planner_name"] == ""
    assert explore["octoplanner3d_robot_radius"] == pytest.approx(0.25)
    assert explore["octoplanner3d_require_ground_support"] is True
    assert explore["octoplanner3d_max_slope"] == pytest.approx(0.0)
    assert explore["enable_native"] is True
    assert explore["terrain_backend"] == "nanobind"
    assert explore["terrain_strict_native"] is True
    assert explore["local_planner_backend"] == "nanobind"
    assert explore["path_follower_backend"] == "nav_kernel"
    assert explore["enable_frontier"] is True
    assert explore["enable_traversable_frontier"] is True
    assert explore["exploration_backend"] == "none"
    assert "lidar_transport" not in explore


def test_thunder_product_configs_accept_local_overrides():
    config = thunder_nav_config(llm="mock", run_startup_checks=False)

    assert config["llm"] == "mock"
    assert config["run_startup_checks"] is False


def test_thunder_blueprint_accepts_resolved_config() -> None:
    config = thunder_nav_config(
        llm="mock",
        run_startup_checks=False,
        manage_external_services=False,
    )

    bp = thunder_blueprint(config, enable_gateway=False)
    names = _entry_names(bp)

    assert "ThunderDriver" in names
    assert "SlamAdapterModule" in names
    assert "SlamBridgeModule" not in names
    assert "GatewayModule" not in names


def test_thunder_product_blueprint_uses_stack_composition_directly() -> None:
    path = ROOT / "src" / "runtime" / "blueprints" / "products" / "thunder.py"
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module)

    assert "runtime.blueprints.stacks.composition" in imports


def test_thunder_basic_blueprint_builds_from_product_entrypoint() -> None:
    bp = thunder_basic_blueprint()
    names = _entry_names(bp)
    wires = _wire_set(bp)

    assert "ThunderDriver" in names
    assert "nav.mission" in names
    assert "nav.safety" in names
    assert "ExternalServiceManagerModule" not in names
    assert "nav.safety.stop_cmd->ThunderDriver.stop_signal" in wires
    assert "nav.safety.stop_cmd->nav.mission.stop_signal" in wires


def test_thunder_lite_blueprint_is_minimal_no_ros_product_entrypoint() -> None:
    bp = thunder_lite_blueprint()
    names = _entry_names(bp)
    wires = _wire_set(bp)

    assert {"ThunderDriver", "nav.mission", "nav.safety", "nav.velocity_mux"} <= names
    assert "SlamAdapterModule" not in names
    assert "SlamBridgeModule" not in names
    assert "SLAMModule" not in names
    assert "GnssModule" not in names
    assert "GnssBridgeModule" not in names
    assert "GatewayModule" not in names
    assert "MCPServerModule" not in names
    assert "ExternalServiceManagerModule" not in names
    assert "DeviceManager" not in names
    assert "OccupancyGridModule" not in names
    assert "nav.maps" not in names
    assert "SemanticPlannerModule" not in names
    assert "nav.safety.stop_cmd->ThunderDriver.stop_signal" in wires
    assert "nav.velocity_mux.driver_cmd_vel->ThunderDriver.cmd_vel" in wires

    configs = {entry.name: entry.config for entry in bp._entries}
    assert configs["nav.terrain"]["backend"] == "simple"
    assert configs["nav.local_planner"]["backend"] == "simple"
    assert configs["nav.path_follower"]["backend"] == "pid"
    assert configs["nav.local_planner"]["backend"] not in {"nanobind", "cmu", "cmu_py"}
    assert configs["nav.path_follower"]["backend"] not in {"nav_kernel", "pure_pursuit"}

    slam_entries = [
        entry
        for entry in bp._entries
        if getattr(entry.module_cls, "__module__", "").startswith("localization.")
    ]
    assert slam_entries == []


def test_thunder_lite_runtime_rejects_legacy_endpoint_flags() -> None:
    config = thunder_lite_config(enable_endpoint_command_bridge=True)

    with pytest.raises(ValueError, match="enable_endpoint_command_bridge=True"):
        thunder_blueprint(config)


@pytest.mark.parametrize(
    "override",
    (
        {"slam_profile": "fastlio2"},
        {"enable_gateway": True},
        {"enable_semantic": True},
        {"enable_map_modules": True},
        {"enable_nav_in": True},
        {"enable_nav_out": True},
        {"enable_ros2_camera_bridge": True},
        {"enable_ros2_rerun_bridge": True},
        {"run_startup_checks": True},
    ),
)
def test_thunder_lite_runtime_rejects_field_runtime_overrides(override) -> None:
    config = thunder_lite_config(**override)

    with pytest.raises(ValueError, match="Thunder Lite runtime"):
        thunder_blueprint(config)


def test_manual_smoke_entrypoints_use_profile_builder() -> None:
    for rel_path in (
        "tests/scripts/smoke/s100p_start.py",
        "tests/scripts/smoke/mapping.py",
        "tests/scripts/smoke/nav_planning.py",
        "tests/scripts/smoke/mcp_full.py",
        "scripts/visualization/run_rerun_mapping.py",
    ):
        text = (ROOT / rel_path).read_text(encoding="utf-8-sig")

        assert "blueprint_for_resolved_profile" in text


def test_manual_smoke_docs_use_thunder_product_name() -> None:
    for rel_path in (
        "tests/scripts/README.md",
        "tests/scripts/smoke/README.md",
        "tests/scripts/smoke/s100p_start.py",
        "tests/scripts/smoke/mapping.py",
        "tests/scripts/smoke/nav_planning.py",
        "tests/scripts/smoke/mcp_full.py",
    ):
        text = (ROOT / rel_path).read_text(encoding="utf-8-sig")

        assert "S100P" not in text
