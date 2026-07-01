from __future__ import annotations

import ast
from pathlib import Path

from core.blueprints.products.thunder import (
    thunder_basic_blueprint,
    thunder_basic_config,
    thunder_blueprint,
    thunder_explore_config,
    thunder_lite_blueprint,
    thunder_lite_config,
    thunder_map_config,
    thunder_nav_config,
)
from lingtu_runtime.plugin_seed import install_builtin_plugin_catalog

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
    assert basic["slam_profile"] == "none"
    assert basic["planner"] == "astar"
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
    assert nav["planner"] == "pct"
    assert nav["enable_semantic"] is True

    mapping = thunder_map_config()
    assert mapping["slam_profile"] == "fastlio2"
    assert mapping["planner"] == "astar"
    assert mapping["enable_semantic"] is False

    explore = thunder_explore_config()
    assert explore["slam_profile"] == "fastlio2"
    assert explore["enable_frontier"] is True
    assert explore["enable_traversable_frontier"] is True
    assert explore["exploration_backend"] == "none"


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
    assert "SlamBridgeModule" in names
    assert "GatewayModule" not in names


def test_thunder_product_blueprints_do_not_import_full_stack_compat_entry() -> None:
    path = ROOT / "src" / "core" / "blueprints" / "products" / "thunder.py"
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module)

    assert "core.blueprints.full_stack" not in imports


def test_thunder_basic_blueprint_builds_from_product_entrypoint() -> None:
    bp = thunder_basic_blueprint()
    names = _entry_names(bp)
    wires = _wire_set(bp)

    assert "ThunderDriver" in names
    assert "NavigationModule" in names
    assert "SafetyRingModule" in names
    assert "ExternalServiceManagerModule" not in names
    assert "SafetyRingModule.stop_cmd->ThunderDriver.stop_signal" in wires
    assert "SafetyRingModule.stop_cmd->NavigationModule.stop_signal" in wires


def test_thunder_lite_blueprint_is_minimal_no_ros_product_entrypoint() -> None:
    bp = thunder_lite_blueprint()
    names = _entry_names(bp)
    wires = _wire_set(bp)

    assert {"ThunderDriver", "NavigationModule", "SafetyRingModule", "CmdVelMux"} <= names
    assert "SlamBridgeModule" not in names
    assert "SLAMModule" not in names
    assert "GnssModule" not in names
    assert "GnssBridgeModule" not in names
    assert "GatewayModule" not in names
    assert "MCPServerModule" not in names
    assert "ExternalServiceManagerModule" not in names
    assert "DeviceManager" not in names
    assert "OccupancyGridModule" not in names
    assert "MapManagerModule" not in names
    assert "SemanticPlannerModule" not in names
    assert "SafetyRingModule.stop_cmd->ThunderDriver.stop_signal" in wires
    assert "CmdVelMux.driver_cmd_vel->ThunderDriver.cmd_vel" in wires

    configs = {entry.name: entry.config for entry in bp._entries}
    assert configs["TerrainModule"]["backend"] == "simple"
    assert configs["LocalPlannerModule"]["backend"] == "simple"
    assert configs["PathFollowerModule"]["backend"] == "pid"

    slam_entries = [
        entry
        for entry in bp._entries
        if getattr(entry.module_cls, "__module__", "").startswith("slam.")
    ]
    assert slam_entries == []


def test_manual_smoke_entrypoints_use_profile_builder() -> None:
    for rel_path in (
        "tests/scripts/smoke/s100p_start.py",
        "tests/scripts/smoke/mapping.py",
        "tests/scripts/smoke/nav_planning.py",
        "tests/scripts/smoke/mcp_full.py",
        "scripts/visualization/run_rerun_mapping.py",
    ):
        text = (ROOT / rel_path).read_text(encoding="utf-8-sig")

        assert "core.blueprints.full_stack" not in text
        assert "full_stack_blueprint(" not in text
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
