from __future__ import annotations

from core.blueprints.profile_graph import resolve_profile_config as graph_resolve_config
from core.blueprints.runtime_endpoint import resolve_runtime_run_spec
from core.runtime.resolver import (
    PROFILE_ALIASES,
    canonical_profile_name,
    resolve_profile_config,
    resolve_runtime_config,
)


def test_resolver_applies_default_hardware_endpoint_after_robot_defaults() -> None:
    resolved = resolve_runtime_config("nav")

    assert resolved.runtime_endpoint == "thunder_field"
    assert resolved.robot_preset == "thunder"
    assert "slam_profile" not in resolved.product_config
    assert resolved.robot_config["slam_profile"] == "localizer"
    assert resolved.endpoint_config["slam_profile"] == "bridge"
    assert resolved.endpoint_config["localization_adapter"] == "lcm_endpoint"
    assert resolved.endpoint_config["endpoint_ingress_adapter"] == "lcm_endpoint"
    assert resolved.endpoint_config["endpoint_egress_adapter"] == "lcm_endpoint"
    assert resolved.endpoint_config["enable_robot_driver"] is False
    assert resolved.endpoint_config["command_output_mode"] == "endpoint_only"
    assert resolved.endpoint_config["hardware_control_boundary"] == "lcm_endpoint_source"
    assert resolved.endpoint_config["enable_endpoint_command_bridge"] is True
    assert resolved.endpoint_config["enable_endpoint_path_bridge"] is True
    assert resolved.config["slam_profile"] == "bridge"
    assert resolved.config["localization_adapter"] == "lcm_endpoint"
    assert resolved.config["endpoint_ingress_adapter"] == "lcm_endpoint"
    assert resolved.config["endpoint_egress_adapter"] == "lcm_endpoint"
    assert resolved.config["enable_robot_driver"] is False
    assert resolved.config["command_output_mode"] == "endpoint_only"
    assert resolved.config["hardware_control_boundary"] == "lcm_endpoint_source"
    assert resolved.config["enable_endpoint_command_bridge"] is True
    assert resolved.config["enable_endpoint_path_bridge"] is True
    assert resolved.config["_runtime_endpoint"] == "thunder_field"
    assert resolved.config["robot"] == "thunder"
    assert resolved.config["detector"] == "bpu"
    spec = resolve_runtime_run_spec("nav", resolved.config)
    assert spec.endpoint == "thunder_field"
    assert spec.module_transport == "local"
    assert spec.endpoint_transport == "lcm"
    assert spec.endpoint_contract == "thunder_field_lcm_v1"
    assert spec.localization_adapter == "lcm_endpoint"
    assert spec.env["LINGTU_MODULE_TRANSPORT"] == "local"
    assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == "lcm"
    assert spec.env["LINGTU_ENDPOINT_CONTRACT"] == "thunder_field_lcm_v1"
    assert spec.env["LINGTU_LOCALIZATION_ADAPTER"] == "lcm_endpoint"
    assert spec.env["LINGTU_ENABLE_ROBOT_DRIVER"] == "0"
    assert spec.env["LINGTU_COMMAND_OUTPUT_MODE"] == "endpoint_only"
    assert spec.env["LINGTU_HARDWARE_CONTROL_BOUNDARY"] == "lcm_endpoint_source"


def test_resolver_canonicalizes_thunder_field_endpoint_alias() -> None:
    resolved = resolve_runtime_config(
        "nav",
        runtime_endpoint_name="thunder-field",
    )
    spec = resolve_runtime_run_spec("nav", resolved.config)

    assert resolved.runtime_endpoint == "thunder_field"
    assert resolved.robot_preset == "thunder"
    assert resolved.config["_runtime_endpoint"] == "thunder_field"
    assert spec.endpoint == "thunder_field"
    assert spec.data_source == "thunder_field"
    assert spec.runtime_contract == "thunder_field"
    assert spec.endpoint_transport == "lcm"
    assert spec.endpoint_contract == "thunder_field_lcm_v1"
    assert spec.localization_adapter == "lcm_endpoint"
    assert spec.env["LINGTU_ENABLE_ROBOT_DRIVER"] == "0"
    assert spec.env["LINGTU_COMMAND_OUTPUT_MODE"] == "endpoint_only"


def test_resolver_keeps_legacy_real_endpoint_alias_for_compatibility() -> None:
    resolved = resolve_runtime_config(
        "nav",
        runtime_endpoint_name="real_s100p",
    )

    assert resolved.runtime_endpoint == "thunder_field"
    assert resolved.robot_preset == "thunder"


def test_resolver_endpoint_layer_overrides_for_compatibility_runtime() -> None:
    resolved = resolve_runtime_config(
        "explore",
        runtime_endpoint_name="mujoco_live",
    )

    assert resolved.robot_preset == "sim_gazebo"
    assert resolved.runtime_endpoint == "mujoco_live"
    assert resolved.product_config["planner"] == "pct"
    assert resolved.endpoint_config["planner"] == "astar"
    assert resolved.config["planner"] == "astar"
    assert resolved.config["robot"] == "sim_ros2"
    assert resolved.config["_runtime_endpoint"] == "mujoco_live"
    assert resolved.config["_endpoint_data_source"] == "mujoco_fastlio2_live"
    assert resolved.endpoint_config["_module_transport"] == "local"
    assert resolved.endpoint_config["_endpoint_transport"] == "local"


def test_resolver_user_overrides_win_after_endpoint_layer() -> None:
    config = resolve_profile_config(
        "explore",
        runtime_endpoint="mujoco_live",
        overrides={
            "planner": "pct",
            "enable_gateway": False,
        },
        llm="openai",
    )

    assert config["planner"] == "pct"
    assert config["llm"] == "openai"
    assert config["enable_gateway"] is False
    assert config["_runtime_endpoint"] == "mujoco_live"


def test_resolver_allows_explicit_module_transport_override() -> None:
    config = resolve_profile_config("nav", module_transport="lcm")
    spec = resolve_runtime_run_spec("nav", config)

    assert config["_module_transport"] == "local"
    assert config["module_transport"] == "lcm"
    assert spec.module_transport == "lcm"
    assert spec.endpoint_transport == "lcm"
    assert spec.env["LINGTU_MODULE_TRANSPORT"] == "lcm"
    assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == "lcm"


def test_resolver_cli_metadata_mode_keeps_launch_contracts_not_selection_key() -> None:
    config = resolve_profile_config(
        "sim_mujoco_live",
        include_profile_metadata=True,
    )

    assert config["_desc"] == "MuJoCo raw MID-360 + Fast-LIO live simulation"
    assert config["_external_launcher"] == "sim/scripts/launch_mujoco_fastlio2_live.sh"
    assert config["_runtime_contract"] == "mujoco_fastlio2_live"
    assert "_default_robot" not in config


def test_resolver_robot_preset_argument_applies_robot_layer() -> None:
    config = resolve_profile_config("nav", robot_preset="stub")

    assert config["robot"] == "stub"
    assert config["detector"] == "yoloe"
    assert config["slam_profile"] == "none"
    assert "_runtime_endpoint" not in config


def test_profile_graph_uses_runtime_resolver_entrypoint() -> None:
    assert graph_resolve_config is resolve_profile_config


def test_thunder_product_aliases_resolve_to_canonical_profiles() -> None:
    assert PROFILE_ALIASES["thunder-lite"] == "lite"
    assert PROFILE_ALIASES["thunder-basic"] == "lite"
    assert PROFILE_ALIASES["thunder-nav"] == "nav"
    assert canonical_profile_name("thunder-lite") == "lite"
    assert canonical_profile_name("thunder-explore") == "explore"

    resolved = resolve_runtime_config("thunder-nav")
    alias_config = resolve_profile_config("thunder-nav")
    canonical_config = resolve_profile_config("nav")

    assert resolved.profile == "nav"
    assert resolved.config["enable_robot_driver"] is False
    assert resolved.config["command_output_mode"] == "endpoint_only"
    assert alias_config == canonical_config


def test_thunder_lite_alias_resolves_to_lightweight_runtime_contract() -> None:
    resolved = resolve_runtime_config("thunder-lite")
    config = resolve_profile_config("thunder-lite")
    spec = resolve_runtime_run_spec("lite", config)

    assert resolved.profile == "lite"
    assert resolved.runtime_endpoint == "thunder_lite"
    assert resolved.robot_preset == "thunder"
    assert config["robot"] == "thunder"
    assert config["slam_profile"] == "none"
    assert config["enable_semantic"] is False
    assert config["python_autonomy_backend"] == "simple"
    assert config["python_path_follower_backend"] == "pid"
    assert config["enable_gateway"] is False
    assert config["enable_map_modules"] is False
    assert config["enable_gnss"] is False
    assert spec.endpoint == "thunder_lite"
    assert spec.data_source == "thunder_lite_local"
    assert spec.runtime_contract == "thunder_lite_local"
    assert spec.module_transport == "local"
    assert spec.endpoint_transport == "local"
    assert spec.launcher is None
    assert spec.slam_source == "none"
    assert spec.localization_source == "none"
    assert spec.mapping_source == "none"
    assert list(spec.topic_allowed_frame_ids) == ["/nav/cmd_vel"]
    assert [stage["name"] for stage in spec.resolved_runtime_data_flow] == [
        "endpoint_adapter",
        "command_boundary",
    ]
