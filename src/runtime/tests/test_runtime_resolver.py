from __future__ import annotations

import pytest

import lingtu.assembly.graph as profile_graph
from lingtu.assembly.products import (
    FIELD_PRODUCT_HOST_DEFAULTS,
    resolve_product_host_config,
    resolve_product_host_runtime,
)
from runtime.profiles.catalog.runtime_paths import DEFAULT_PLANNING_FRAME_ID
from runtime.profiles.profile_adapters import resolve_runtime_run_spec
from runtime.profiles.resolver import (
    canonical_profile_name,
    resolve_profile_config,
    resolve_runtime_config,
)


def _resolve_named_runtime(name: str, **kwargs):
    if name in FIELD_PRODUCT_HOST_DEFAULTS:
        return resolve_product_host_runtime(
            name,
            kwargs.pop("env", "real"),
            **kwargs,
        )
    return resolve_runtime_config(name, **kwargs)


def _resolve_named_config(name: str, **kwargs):
    if name in FIELD_PRODUCT_HOST_DEFAULTS:
        return resolve_product_host_config(
            name,
            kwargs.pop("env", "real"),
            **kwargs,
        )
    return resolve_profile_config(name, **kwargs)


def test_profile_resolver_rejects_field_products() -> None:
    with pytest.raises(KeyError, match="unknown profile: nav"):
        resolve_runtime_config("nav")


def test_real_env_applies_native_host_communication_config() -> None:
    resolved = _resolve_named_runtime("nav")

    assert resolved.product == "nav"
    assert resolved.env == "real"
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
    assert resolved.config["_env"] == "real"
    assert "_profile_adapter" not in resolved.config
    assert "_driver_backend" not in resolved.config
    assert resolved.config["robot"] == "thunder"
    assert resolved.config["detector"] == "bpu"
    with pytest.raises(
        ValueError,
        match="Product RunPlans cannot be resolved through Profile adapters",
    ):
        resolve_runtime_run_spec("nav", resolved.config)

def test_real_product_hosts_never_manage_sensor_or_driver_services() -> None:
    for product in FIELD_PRODUCT_HOST_DEFAULTS:
        resolved = _resolve_named_runtime(product)
        assert resolved.env == "real"
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

        with pytest.raises(
            ValueError,
            match="Product RunPlans cannot be resolved through Profile adapters",
        ):
            resolve_runtime_run_spec(product, resolved.config)

def test_product_defaults_win_before_env_and_operator_overrides_win_last() -> None:
    resolved = _resolve_named_runtime("map")

    assert resolved.config["slam_profile"] == "fastlio2"
    assert resolved.config["robot"] == "thunder"

    overridden = _resolve_named_runtime(
        "map",
        overrides={
            "slam_profile": "manual_override",
            "planner_latency_budget_ms": 450,
        },
    )

    assert overridden.config["slam_profile"] == "manual_override"
    assert overridden.config["planner_latency_budget_ms"] == 450


def test_map_product_real_env_does_not_manage_lidar() -> None:
    resolved = _resolve_named_runtime("map")

    assert resolved.env == "real"
    assert resolved.config["slam_profile"] == "fastlio2"
    assert resolved.config["enable_robot_driver"] is False
    assert resolved.config["enable_lidar"] is False
    assert resolved.config["enable_imu"] is False
    assert resolved.config["localization_adapter"] == "cpp_slam_status"


def test_field_product_rejects_profile_driver_backend_bypass() -> None:
    with pytest.raises(TypeError, match="reserved Product resolution key"):
        resolve_product_host_config("map", "real", driver_backend="thunder")


def test_real_env_exposes_communication_adapter_boundary() -> None:
    resolved = _resolve_named_runtime("nav")

    assert resolved.config["planner"] == "octoplanner3d"
    assert resolved.config["dog_host"] == "127.0.0.1"
    assert resolved.config["_endpoint_transport"] == "dds"
    assert resolved.config["_endpoint_contract"] == "thunder_dds_v1"
    assert "nav_out_adapter" not in resolved.config
    assert resolved.config["native_navigation_endpoint"] == "lingtu-nav-dds"


@pytest.mark.parametrize(
    "env",
    ["legacy_real_target", "mujoco_live", "real_s100p", "portable_fastlio2"],
)
def test_product_resolution_rejects_endpoint_names_as_env(env: str) -> None:
    with pytest.raises(ValueError, match="unknown Env"):
        _resolve_named_runtime("map", env=env)


def test_sim_host_backend_applies_environment_adapter_config() -> None:
    resolved = _resolve_named_runtime(
        "explore",
        env="sim",
        env_config={"backend": "mujoco_host"},
    )

    assert resolved.env == "sim"
    assert resolved.config["planner"] == "octoplanner3d"
    assert resolved.config["robot"] == "sim_endpoint"
    assert resolved.config["_env"] == "sim"
    assert resolved.config["_env_backend"] == "mujoco_host"
    assert resolved.config["_module_transport"] == "local"
    assert resolved.config["_endpoint_transport"] == "local"
    assert resolved.config["enable_map_out"] is False
    assert "enable_nav_out" not in resolved.config


def test_explore_map_variant_preserves_saved_map_host_configuration() -> None:
    resolved = _resolve_named_runtime(
        "explore",
        env="sim",
        env_config={"backend": "mujoco_host"},
        product_variant="map",
    )

    assert resolved.product == "explore"
    assert resolved.config["_product_variant"] == "map"
    assert resolved.config["slam_profile"] == "localizer"
    assert resolved.config["map_artifact_gate_required"] is True
    assert resolved.config["planning_frame_id"] == DEFAULT_PLANNING_FRAME_ID
    assert resolved.config["exploration_backend"] == "tare"
    assert resolved.config["enable_map_out"] is False
    assert "enable_nav_out" not in resolved.config


def test_resolver_user_overrides_win_after_endpoint_layer() -> None:
    config = _resolve_named_config(
        "explore",
        env="sim",
        env_config={"backend": "mujoco_host"},
        overrides={
            "planner": "pct",
            "enable_gateway": False,
        },
        llm="openai",
    )

    assert config["planner"] == "pct"
    assert config["llm"] == "openai"
    assert config["enable_gateway"] is False
    assert config["_env"] == "sim"
    assert config["_env_backend"] == "mujoco_host"


def test_resolver_does_not_mirror_hw_config_into_retired_keys() -> None:
    config = _resolve_named_config("dev", overrides={"enable_hw": False})
    legacy_config = _resolve_named_config(
        "dev",
        overrides={"enable_device_manager": False},
    )

    assert config["enable_hw"] is False
    assert "enable_device_manager" not in config
    assert "enable_hw" not in legacy_config


def test_product_transport_override_still_cannot_enter_profile_adapter_resolver() -> None:
    config = _resolve_named_config("nav", module_transport="shm")

    assert config["_module_transport"] == "local"
    assert config["module_transport"] == "shm"
    with pytest.raises(
        ValueError,
        match="Product RunPlans cannot be resolved through Profile adapters",
    ):
        resolve_runtime_run_spec("nav", config)

def test_sim_nav_enables_the_python_sim_lidar_source() -> None:
    sim_nav = _resolve_named_config("sim_nav")
    sim = _resolve_named_config("sim")

    assert sim_nav["enable_sim_lidar"] is True
    assert sim.get("enable_sim_lidar") is not True


def test_portable_mujoco_is_explicit_legacy_driver_sensor_opt_in() -> None:
    config = _resolve_named_config("portable_mujoco")
    sim = _resolve_named_config("sim")

    assert config["use_driver_lidar"] is True
    assert config["use_driver_imu"] is True
    assert sim.get("use_driver_lidar") is not True
    assert sim.get("use_driver_imu") is not True


def test_resolver_cli_metadata_mode_keeps_launch_contracts_not_selection_key() -> None:
    config = _resolve_named_config(
        "sim_mujoco_live",
        include_profile_metadata=True,
    )

    assert config["_desc"] == (
        "Legacy MuJoCo Module harness for downstream nav/map wiring; not real-equivalent native DDS closure"
    )
    assert config["_external_launcher"] == "sim/scripts/mujoco/launch_fastlio2_live.sh"
    assert config["_runtime_contract"] == "mujoco_fastlio2_live"
    assert "_driver_backend" not in config


def test_profile_rejects_public_driver_backend_selection() -> None:
    with pytest.raises(ValueError, match="cannot select a driver backend"):
        resolve_profile_config("dev", robot="stub")
    with pytest.raises(ValueError, match="cannot select a driver backend"):
        resolve_profile_config("dev", driver_backend="stub")


def test_profile_graph_uses_runtime_resolver_entrypoint() -> None:
    assert profile_graph.resolve_profile_config is resolve_profile_config


def test_retired_and_field_product_names_fail_closed() -> None:
    assert canonical_profile_name("lite") == "lite"
    assert canonical_profile_name("thunder-lite") == "thunder-lite"
    assert canonical_profile_name("thunder-explore") == "thunder-explore"

    for retired_name in ("thunder-lite", "thunder-basic"):
        with pytest.raises(KeyError, match=f"unknown profile: {retired_name}"):
            resolve_runtime_config(retired_name)
    with pytest.raises(KeyError, match="unknown profile: thunder-nav"):
        resolve_runtime_config("thunder-nav")
    with pytest.raises(ValueError, match="Unknown Product"):
        resolve_product_host_config("thunder-nav", "real")


def test_lite_profile_resolves_to_lightweight_runtime_contract() -> None:
    resolved = _resolve_named_runtime("lite")
    config = _resolve_named_config("lite")
    spec = resolve_runtime_run_spec("lite", config)

    assert resolved.profile == "lite"
    assert resolved.profile_adapter == "thunder_lite"
    assert resolved.driver_backend == "thunder"
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
    assert spec.adapter == "thunder_lite"
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
