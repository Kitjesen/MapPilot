from __future__ import annotations

from dataclasses import replace

import pytest

import runtime.introspection.profile_graph as profile_graph
from runtime.profiles.catalog.endpoints import RUNTIME_ENDPOINTS, RuntimeEndpointError
from runtime.profiles.catalog.runtime_paths import (
    DEFAULT_PLANNING_FRAME_ID,
    RUNTIME_ODOM_FRAME_ID,
)
from runtime.profiles.endpoints import resolve_runtime_run_spec
from runtime.profiles.resolver import (
    PROFILE_ALIASES,
    canonical_profile_name,
    resolve_profile_config,
    resolve_runtime_config,
)


def test_resolver_applies_default_hardware_endpoint_after_robot_defaults() -> None:
    resolved = resolve_runtime_config("nav")

    assert resolved.runtime_endpoint == "thunder_field"
    assert resolved.robot_preset == "thunder"
    assert "_default_robot" not in resolved.product_config
    assert "slam_profile" not in resolved.product_config
    assert "command_output_mode" not in resolved.product_config
    assert "localization_adapter" not in resolved.product_config
    assert {"slam_profile", "detector", "encoder"}.isdisjoint(resolved.robot_config)
    assert resolved.robot_runtime_config["slam_profile"] == "localizer"
    assert resolved.robot_runtime_config["detector"] == "bpu"
    assert resolved.robot_runtime_config["encoder"] == "mobileclip"
    assert "slam_profile" not in resolved.endpoint_config
    assert resolved.endpoint_config["localization_adapter"] == "cpp_slam_status"
    assert "nav_in_adapter" not in resolved.endpoint_config
    assert "nav_out_adapter" not in resolved.endpoint_config
    assert resolved.endpoint_config["native_navigation_endpoint"] == "lingtu-nav-dds"
    assert resolved.endpoint_config["enable_robot_driver"] is False
    assert resolved.endpoint_config["enable_hw"] is False
    assert resolved.endpoint_config["enable_lidar"] is False
    assert resolved.endpoint_config["enable_camera"] is True
    assert resolved.endpoint_config["camera_backend"] == "dds"
    assert resolved.endpoint_config["command_output_mode"] == "endpoint_only"
    assert resolved.endpoint_config["hardware_control_boundary"] == "driver"
    assert "enable_nav_in" not in resolved.endpoint_config
    assert "enable_nav_out" not in resolved.endpoint_config
    assert resolved.endpoint_config["enable_map_out"] is False
    assert resolved.config["slam_profile"] == "localizer"
    assert resolved.config["localization_adapter"] == "cpp_slam_status"
    assert "nav_in_adapter" not in resolved.config
    assert "nav_out_adapter" not in resolved.config
    assert resolved.config["native_navigation_endpoint"] == "lingtu-nav-dds"
    assert resolved.config["enable_robot_driver"] is False
    assert resolved.config["enable_hw"] is False
    assert resolved.config["enable_lidar"] is False
    assert resolved.config["enable_camera"] is True
    assert resolved.config["camera_backend"] == "dds"
    assert resolved.config["command_output_mode"] == "endpoint_only"
    assert resolved.config["hardware_control_boundary"] == "driver"
    assert "enable_nav_in" not in resolved.config
    assert "enable_nav_out" not in resolved.config
    assert resolved.config["enable_map_out"] is False
    assert resolved.config["_runtime_endpoint"] == "thunder_field"
    assert resolved.config["robot"] == "thunder"
    assert resolved.config["detector"] == "bpu"
    spec = resolve_runtime_run_spec("nav", resolved.config)
    assert spec.endpoint == "thunder_field"
    assert spec.module_transport == "local"
    assert spec.endpoint_transport == "dds"
    assert spec.endpoint_contract == "thunder_field_dds_v1"
    assert spec.route_contract == "robot"
    assert spec.localization_adapter == "cpp_slam_status"
    assert spec.global_planner == "octoplanner3d"
    assert spec.fallback_global_planners == ()
    assert spec.planner_latency_budget_ms == 800
    assert spec.plan_safety_policy == "reject"
    assert spec.autonomy_backends == {
        "terrain_backend": "nanobind",
        "local_planner_backend": "nanobind",
        "path_follower_backend": "nav_kernel",
    }
    assert spec.env["LINGTU_MODULE_TRANSPORT"] == "local"
    assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == "dds"
    assert spec.env["LINGTU_ENDPOINT_CONTRACT"] == "thunder_field_dds_v1"
    assert spec.env["LINGTU_ROUTE_CONTRACT"] == "robot"
    assert spec.env["LINGTU_LOCALIZATION_ADAPTER"] == "cpp_slam_status"
    assert spec.env["LINGTU_ENABLE_ROBOT_DRIVER"] == "0"
    assert spec.env["LINGTU_COMMAND_OUTPUT_MODE"] == "endpoint_only"
    assert spec.env["LINGTU_HARDWARE_CONTROL_BOUNDARY"] == "driver"


def test_thunder_field_run_specs_never_manage_sensor_or_driver_services() -> None:
    for profile in RUNTIME_ENDPOINTS["thunder_field"].supported_profiles:
        resolved = resolve_runtime_config(profile)
        spec = resolve_runtime_run_spec(profile, resolved.config)

        assert resolved.runtime_endpoint == "thunder_field"
        assert resolved.config["enable_robot_driver"] is False
        assert resolved.config["enable_hw"] is False
        assert resolved.config["enable_lidar"] is False
        assert resolved.config["enable_imu"] is False
        assert "enable_nav_in" not in resolved.config
        assert "enable_nav_out" not in resolved.config
        assert resolved.config["enable_map_out"] is False
        assert resolved.config["localization_adapter"] == "cpp_slam_status"
        assert resolved.config["native_navigation_endpoint"] == "lingtu-nav-dds"
        assert "lidar_backend" not in resolved.config
        assert "imu_backend" not in resolved.config

        assert spec.endpoint == "thunder_field"
        assert spec.data_source == "thunder_field"
        assert spec.endpoint_transport == "dds"
        assert spec.endpoint_contract == "thunder_field_dds_v1"
        assert spec.localization_adapter == "cpp_slam_status"
        assert spec.env["LINGTU_ENABLE_ROBOT_DRIVER"] == "0"
        assert spec.env["LINGTU_COMMAND_OUTPUT_MODE"] == "endpoint_only"
        assert spec.env["LINGTU_HARDWARE_CONTROL_BOUNDARY"] == "driver"


def test_resolver_layers_keep_product_robot_endpoint_override_precedence() -> None:
    resolved = resolve_runtime_config("map")

    assert resolved.product_config["slam_profile"] == "fastlio2"
    assert resolved.robot_config["robot"] == "thunder"
    assert resolved.robot_runtime_config["slam_profile"] == "localizer"
    assert resolved.endpoint_config["_runtime_endpoint"] == "thunder_field"
    assert "slam_profile" not in resolved.endpoint_config
    assert resolved.config["slam_profile"] == "fastlio2"

    overridden = resolve_runtime_config(
        "map",
        overrides={
            "slam_profile": "manual_override",
            "planner_latency_budget_ms": 450,
        },
    )

    assert overridden.config["slam_profile"] == "manual_override"
    assert overridden.config["planner_latency_budget_ms"] == 450


def test_map_profile_default_field_endpoint_does_not_manage_lidar() -> None:
    resolved = resolve_runtime_config("map")

    assert resolved.runtime_endpoint == "thunder_field"
    assert resolved.config["slam_profile"] == "fastlio2"
    assert resolved.config["enable_robot_driver"] is False
    assert resolved.config["enable_lidar"] is False
    assert resolved.config["enable_imu"] is False
    assert resolved.config["localization_adapter"] == "cpp_slam_status"


def test_managed_map_profile_uses_local_lidar_when_endpoint_is_bypassed() -> None:
    config = resolve_profile_config("map", robot_preset="thunder")

    assert "_runtime_endpoint" not in config
    assert config["robot"] == "thunder"
    assert config["slam_profile"] == "fastlio2"
    assert "enable_lidar" not in config
    assert "lidar_backend" not in config
    assert "enable_imu" not in config


def test_resolver_layers_expose_endpoint_adapter_boundary() -> None:
    resolved = resolve_runtime_config("nav")

    assert resolved.product_config["planner"] == "octoplanner3d"
    assert "dog_host" not in resolved.product_config
    assert "nav_out_adapter" not in resolved.product_config
    assert resolved.robot_config["dog_host"] == "127.0.0.1"
    assert "nav_out_adapter" not in resolved.robot_config
    assert resolved.endpoint_config["_endpoint_transport"] == "dds"
    assert "nav_out_adapter" not in resolved.endpoint_config
    assert resolved.endpoint_config["native_navigation_endpoint"] == "lingtu-nav-dds"
    assert "slam_profile" not in resolved.endpoint_config
    assert resolved.config["planner"] == "octoplanner3d"
    assert resolved.config["dog_host"] == "127.0.0.1"
    assert "nav_out_adapter" not in resolved.config
    assert resolved.config["native_navigation_endpoint"] == "lingtu-nav-dds"


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
    assert spec.endpoint_transport == "dds"
    assert spec.endpoint_contract == "thunder_field_dds_v1"
    assert spec.route_contract == "robot"
    assert spec.localization_adapter == "cpp_slam_status"
    assert spec.env["LINGTU_ENABLE_ROBOT_DRIVER"] == "0"
    assert spec.env["LINGTU_COMMAND_OUTPUT_MODE"] == "endpoint_only"


def test_resolver_keeps_legacy_real_endpoint_alias_for_compatibility() -> None:
    resolved = resolve_runtime_config(
        "nav",
        runtime_endpoint_name="real_s100p",
    )

    assert resolved.runtime_endpoint == "thunder_field"
    assert resolved.robot_preset == "thunder"


@pytest.mark.parametrize(
    "endpoint",
    ["windows-fastlio2", "ubuntu-portable", "portable-lio", "portable_fastlio2"],
)
def test_removed_portable_fastlio2_endpoint_aliases_fail_closed(endpoint: str) -> None:
    with pytest.raises(RuntimeEndpointError, match="unknown runtime endpoint"):
        resolve_runtime_config("map", runtime_endpoint_name=endpoint)


def test_product_endpoint_matrix_rejects_endpoint_catalog_drift(monkeypatch) -> None:
    endpoint = RUNTIME_ENDPOINTS["cmu_unity"]
    monkeypatch.setitem(
        RUNTIME_ENDPOINTS,
        "cmu_unity",
        replace(endpoint, supported_profiles=(*endpoint.supported_profiles, "nav")),
    )

    with pytest.raises(RuntimeEndpointError, match="not a product endpoint"):
        resolve_runtime_config("nav", runtime_endpoint_name="cmu_unity")


def test_resolver_endpoint_layer_overrides_for_compatibility_runtime() -> None:
    resolved = resolve_runtime_config(
        "explore",
        runtime_endpoint_name="mujoco_live",
    )

    assert resolved.robot_preset == "sim_endpoint"
    assert resolved.runtime_endpoint == "mujoco_live"
    assert resolved.product_config["planner"] == "octoplanner3d"
    assert resolved.endpoint_config["planner"] == "octoplanner3d"
    assert resolved.config["planner"] == "octoplanner3d"
    assert resolved.config["robot"] == "sim_endpoint"
    assert resolved.config["_runtime_endpoint"] == "mujoco_live"
    assert resolved.config["_endpoint_data_source"] == "mujoco_fastlio2_live"
    assert resolved.endpoint_config["_module_transport"] == "local"
    assert resolved.endpoint_config["_endpoint_transport"] == "local"
    assert resolved.endpoint_config["enable_map_out"] is False
    assert "enable_nav_out" not in resolved.endpoint_config
    assert resolved.config["enable_map_out"] is False
    assert "enable_nav_out" not in resolved.config


def test_resolver_preserves_intentional_tare_frame_endpoint_override() -> None:
    resolved = resolve_runtime_config(
        "tare_explore",
        runtime_endpoint_name="mujoco_live",
    )

    assert resolved.product_config["planning_frame_id"] == DEFAULT_PLANNING_FRAME_ID
    assert resolved.endpoint_config["planning_frame_id"] == RUNTIME_ODOM_FRAME_ID
    assert resolved.endpoint_config["occupancy_frame_id"] == RUNTIME_ODOM_FRAME_ID
    assert resolved.endpoint_config["exploration_backend"] == "tare"
    assert resolved.endpoint_config["enable_map_out"] is False
    assert "enable_nav_out" not in resolved.endpoint_config
    assert resolved.config["planning_frame_id"] == RUNTIME_ODOM_FRAME_ID
    assert resolved.config["exploration_backend"] == "tare"
    assert resolved.config["enable_map_out"] is False
    assert "enable_nav_out" not in resolved.config


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
    config = resolve_profile_config("nav", module_transport="shm")
    spec = resolve_runtime_run_spec("nav", config)

    assert config["_module_transport"] == "local"
    assert config["module_transport"] == "shm"
    assert spec.module_transport == "shm"
    assert spec.endpoint_transport == "dds"
    assert spec.env["LINGTU_MODULE_TRANSPORT"] == "shm"
    assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == "dds"


def test_sim_nav_is_explicit_legacy_sim_lidar_opt_in() -> None:
    sim_nav = resolve_profile_config("sim_nav")
    sim = resolve_profile_config("sim")

    assert sim_nav["enable_legacy_sim_lidar"] is True
    assert sim.get("enable_legacy_sim_lidar") is not True


def test_portable_mujoco_is_explicit_legacy_driver_sensor_opt_in() -> None:
    config = resolve_profile_config("portable_mujoco")
    sim = resolve_profile_config("sim")

    assert config["use_driver_lidar"] is True
    assert config["use_driver_imu"] is True
    assert sim.get("use_driver_lidar") is not True
    assert sim.get("use_driver_imu") is not True


def test_resolver_cli_metadata_mode_keeps_launch_contracts_not_selection_key() -> None:
    config = resolve_profile_config(
        "sim_mujoco_live",
        include_profile_metadata=True,
    )

    assert config["_desc"] == (
        "Legacy MuJoCo Module harness for downstream nav/map wiring; not real-equivalent native DDS closure"
    )
    assert config["_external_launcher"] == "sim/scripts/mujoco/launch_fastlio2_live.sh"
    assert config["_runtime_contract"] == "mujoco_fastlio2_live"
    assert "_default_robot" not in config


def test_resolver_robot_preset_argument_applies_robot_layer() -> None:
    config = resolve_profile_config("nav", robot_preset="stub")

    assert config["robot"] == "stub"
    assert config["detector"] == "yoloe"
    assert config["encoder"] == "mobileclip"
    assert config["slam_profile"] == "none"
    assert "_runtime_endpoint" not in config


def test_profile_graph_uses_runtime_resolver_entrypoint() -> None:
    assert profile_graph.resolve_profile_config is resolve_profile_config


def test_thunder_product_aliases_resolve_to_canonical_profiles() -> None:
    assert PROFILE_ALIASES["thunder-lite"] == "lite"
    assert PROFILE_ALIASES["thunder-basic"] == "lite"
    assert PROFILE_ALIASES["thunder-nav"] == "nav"
    assert canonical_profile_name("thunder-lite") == "lite"
    assert canonical_profile_name("thunder-explore") == "tare_explore"

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
    assert config["runtime_mode"] == "lite"
    assert config["slam_profile"] == "none"
    assert config["planner"] == "direct"
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
    assert spec.route_contract is None
    assert spec.launcher is None
    assert spec.slam_source == "none"
    assert spec.localization_source == "none"
    assert spec.mapping_source == "none"
    assert list(spec.topic_allowed_frame_ids) == ["/nav/cmd_vel"]
    assert [stage["name"] for stage in spec.resolved_runtime_data_flow] == [
        "endpoint_adapter",
        "command_boundary",
    ]
