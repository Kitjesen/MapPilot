from __future__ import annotations

import ast
from pathlib import Path

import pytest

import runtime.profiles.catalog as runtime_catalog
from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.assembly.products.host_defaults import (
    FIELD_PRODUCT_HOST_DEFAULTS,
    FIELD_PRODUCT_NAMES,
    FIELD_PRODUCT_VARIANT_HOST_DEFAULTS,
)
from lingtu.assembly.stacks.driver import DriverBackend
from runtime.graph.loader import load_runtime_graph
from runtime.profiles.binding_policy import (
    LEGACY_SENSOR_BINDING_KEYS,
    ROS2_RERUN_BRIDGE_ENABLE_KEYS,
    resolved_autonomy_backend_selection,
    ros2_autonomy_backend_violations,
    ros2_runtime_binding_violations,
)
from runtime.profiles.catalog.driver_backends import (
    CANONICAL_DRIVER_BACKENDS,
    CANONICAL_DRIVER_PROTOCOLS,
    DRIVER_BACKENDS,
    DRIVER_PROTOCOLS,
    driver_backend_defaults,
    driver_backend_module_name,
    driver_backend_names,
    driver_backend_protocol,
)
from runtime.profiles.catalog.driver_catalog import (
    DRIVER_CATALOG_SCHEMA_VERSION,
    driver_catalog,
    driver_catalog_backends,
    driver_catalog_modules,
    driver_catalog_path,
    driver_catalog_protocols,
    driver_catalog_runtime_defaults,
)
from runtime.profiles.catalog.driver_runtime_defaults import (
    CANONICAL_DRIVER_RUNTIME_DEFAULTS,
    DRIVER_RUNTIME_DEFAULTS,
    driver_runtime_defaults,
)
from runtime.profiles.catalog.host_defaults import (
    HOST_PROFILE_DEFAULTS,
    HOST_PROFILE_SNAPSHOT_NAMES,
)
from runtime.profiles.catalog.local_host_defaults import (
    LOCAL_HOST_DEFAULTS,
    LOCAL_PROFILE_NAMES,
)
from runtime.profiles.catalog.profile_adapter_configs import (
    MUJOCO_LIVE_CONFIG,
)
from runtime.profiles.catalog.profile_adapters import PROFILE_ADAPTERS as CATALOG_ADAPTERS
from runtime.profiles.catalog.profile_adapters import (
    PROFILE_ADAPTERS as RUNTIME_CATALOG_ADAPTERS,
)
from runtime.profiles.catalog.profile_adapters import profile_adapter_names_for_profile
from runtime.profiles.catalog.runtime_paths import (
    DEFAULT_GATEWAY_PORT,
    DEFAULT_PLANNING_FRAME_ID,
    DEFAULT_SAMPLE_OCTOPLANNER3D_MAP,
    RUNTIME_MAP_FRAME_ID,
    RUNTIME_ODOM_FRAME_ID,
    _resolve_octoplanner3d_map,
)
from runtime.profiles.catalog.simulation_profiles import (
    SIMULATION_ENTRYPOINT_PROFILES,
    SIMULATION_PROFILES,
)
from runtime.profiles.profile_adapters import (
    PROFILE_ADAPTERS as COMPAT_ADAPTERS,
)
from runtime.profiles.profile_adapters import (
    PROFILE_ADAPTERS as PROFILE_ADAPTERS_RUNTIME,
)
from runtime.profiles.profile_adapters import profile_adapter_names
from runtime.profiles.profile_adapters import (
    resolve_runtime_run_spec as resolve_runtime_run_spec_compat,
)
from runtime.profiles.profile_adapters import (
    resolve_runtime_run_spec as resolve_runtime_run_spec_runtime,
)
from runtime.profiles.resolver import resolve_runtime_config

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
ALL_NAMED_HOST_DEFAULTS = {
    **HOST_PROFILE_DEFAULTS,
    **FIELD_PRODUCT_HOST_DEFAULTS,
}


def _resolve_named_runtime(name: str):
    if name in FIELD_PRODUCT_HOST_DEFAULTS:
        return resolve_product_host_runtime(name, "real")
    return resolve_runtime_config(name)


def test_product_names_and_host_defaults_follow_runtime_graph() -> None:
    expected = tuple(load_runtime_graph().products)

    assert FIELD_PRODUCT_NAMES == expected
    assert tuple(FIELD_PRODUCT_HOST_DEFAULTS) == expected
    assert "lite" not in FIELD_PRODUCT_NAMES
    assert "super_lio" not in FIELD_PRODUCT_NAMES
    assert "super_lio_relocation" not in FIELD_PRODUCT_NAMES


def test_host_default_categories_are_explicit_and_disjoint() -> None:
    groups = (
        set(FIELD_PRODUCT_HOST_DEFAULTS),
        set(LOCAL_HOST_DEFAULTS),
        set(SIMULATION_ENTRYPOINT_PROFILES),
    )

    for index, group in enumerate(groups):
        for other in groups[index + 1 :]:
            assert group.isdisjoint(other)

    assert LOCAL_PROFILE_NAMES == ("lite",)
    assert set(HOST_PROFILE_DEFAULTS) == set().union(*groups[1:])
    assert set(FIELD_PRODUCT_HOST_DEFAULTS).isdisjoint(HOST_PROFILE_DEFAULTS)


@pytest.mark.parametrize(
    "profile",
    ("super_lio", "super_lio_relocation", "relocation"),
)
def test_super_lio_implementation_names_are_not_host_profiles(profile: str) -> None:
    assert profile not in HOST_PROFILE_DEFAULTS
    with pytest.raises(KeyError, match=f"unknown profile: {profile}"):
        resolve_runtime_config(profile)



@pytest.mark.parametrize("profile", ("sim_gazebo", "sim_industrial", "sim_cmu_tare"))
def test_removed_broken_sim_profiles_fail_closed(profile: str) -> None:
    assert profile not in HOST_PROFILE_DEFAULTS
    assert profile not in SIMULATION_ENTRYPOINT_PROFILES
    assert profile not in SIMULATION_PROFILES
    assert profile_adapter_names_for_profile(profile) == ()
    with pytest.raises(KeyError, match=f"unknown profile: {profile}"):
        resolve_runtime_config(profile)

def test_runtime_profile_catalog_is_single_source() -> None:
    assert CATALOG_ADAPTERS is RUNTIME_CATALOG_ADAPTERS
    assert runtime_catalog.DRIVER_BACKENDS is DRIVER_BACKENDS
    assert runtime_catalog.DRIVER_PROTOCOLS is DRIVER_PROTOCOLS


def test_profile_adapter_resolver_reexports_adapter_catalog() -> None:
    assert COMPAT_ADAPTERS is CATALOG_ADAPTERS
    assert COMPAT_ADAPTERS is PROFILE_ADAPTERS_RUNTIME
    assert resolve_runtime_run_spec_compat is resolve_runtime_run_spec_runtime
    assert profile_adapter_names_for_profile("nav") == ()
    assert CATALOG_ADAPTERS["thunder_lite"].driver_backend == "thunder"
    assert CATALOG_ADAPTERS["thunder_lite"].data_source == "thunder_lite_local"
    assert CATALOG_ADAPTERS["thunder_lite"].config_overrides == {
        "enable_hw": False,
    }
    assert "thunder_dds" not in CATALOG_ADAPTERS
    assert "portable_fastlio2" not in CATALOG_ADAPTERS


def test_profile_adapter_names_expose_only_canonical_names() -> None:
    adapter_names = profile_adapter_names()

    assert adapter_names == tuple(CATALOG_ADAPTERS)
    assert "thunder-lite" not in adapter_names
    assert "thunder-basic" not in adapter_names


def test_driver_protocols_share_internal_driver_backend_names() -> None:
    assert set(DRIVER_PROTOCOLS) == set(DRIVER_BACKENDS)
    assert set(DriverBackend.known_backends()) == set(DRIVER_BACKENDS)
    assert set(DRIVER_RUNTIME_DEFAULTS) == set(DRIVER_BACKENDS)


def test_driver_catalog_exposes_only_canonical_backends() -> None:
    expected = {
        "stub",
        "sim",
        "sim_endpoint",

        "thunder",
        "thunder_remote",
    }

    assert set(CANONICAL_DRIVER_BACKENDS) == expected
    assert set(CANONICAL_DRIVER_PROTOCOLS) == expected
    assert set(CANONICAL_DRIVER_RUNTIME_DEFAULTS) == expected
    assert set(DRIVER_BACKENDS) == expected
    assert set(DRIVER_PROTOCOLS) == expected
    assert set(DRIVER_RUNTIME_DEFAULTS) == expected
    assert set(driver_backend_names()) == expected
    assert set(DriverBackend.known_backends()) == expected
    assert driver_backend_module_name("thunder") == "ThunderDriver"


@pytest.mark.parametrize("name", ["s100p", "navigate", "sim_gazebo"])
def test_removed_driver_backend_aliases_fail_closed(name: str) -> None:
    with pytest.raises(KeyError):
        driver_backend_defaults(name)
    with pytest.raises(KeyError):
        driver_backend_protocol(name)
    with pytest.raises(KeyError):
        driver_backend_module_name(name)
    with pytest.raises(KeyError):
        driver_runtime_defaults(name)
    with pytest.raises(KeyError):
        DriverBackend(name)


def test_thunder_driver_defaults_remain_canonical() -> None:
    assert DRIVER_BACKENDS["thunder"]["robot"] == "thunder"
    assert DRIVER_BACKENDS["thunder"]["dog_host"] == "127.0.0.1"
    assert DRIVER_BACKENDS["thunder_remote"]["dog_host"] == "192.168.66.13"


def test_thunder_driver_backends_are_sourced_from_driver_catalog() -> None:
    catalog = driver_catalog("thunder")

    assert driver_catalog_path("thunder").name == "thunder.yaml"
    assert catalog["schema_version"] == DRIVER_CATALOG_SCHEMA_VERSION
    assert "compat_aliases" not in catalog
    assert driver_catalog_backends("thunder") == {
        "thunder": CANONICAL_DRIVER_BACKENDS["thunder"],
        "thunder_remote": CANONICAL_DRIVER_BACKENDS["thunder_remote"],
    }
    assert driver_catalog_protocols("thunder") == {
        "thunder": CANONICAL_DRIVER_PROTOCOLS["thunder"],
        "thunder_remote": CANONICAL_DRIVER_PROTOCOLS["thunder_remote"],
    }
    assert driver_catalog_modules("thunder") == {
        "thunder": "ThunderDriver",
        "thunder_remote": "ThunderDriver",
    }
    assert driver_catalog_runtime_defaults("thunder") == {
        "thunder": CANONICAL_DRIVER_RUNTIME_DEFAULTS["thunder"],
        "thunder_remote": CANONICAL_DRIVER_RUNTIME_DEFAULTS["thunder_remote"],
    }


def test_driver_defaults_do_not_own_upper_stack_defaults() -> None:
    upper_stack_fields = {"slam_profile", "detector", "encoder"}

    for backend, config in DRIVER_BACKENDS.items():
        assert upper_stack_fields.isdisjoint(config), backend

    assert DRIVER_RUNTIME_DEFAULTS["thunder"]["slam_profile"] == "localizer"
    assert DRIVER_RUNTIME_DEFAULTS["thunder"]["detector"] == "bpu"
    assert DRIVER_RUNTIME_DEFAULTS["stub"]["slam_profile"] == "none"
    assert DRIVER_RUNTIME_DEFAULTS["stub"]["detector"] == "yoloe"


def test_host_default_categories_are_unambiguous() -> None:
    assert set(HOST_PROFILE_SNAPSHOT_NAMES) <= set(HOST_PROFILE_DEFAULTS)
    assert set(SIMULATION_ENTRYPOINT_PROFILES) == set(SIMULATION_PROFILES)

    assert "nav" in FIELD_PRODUCT_NAMES
    assert "lite" in LOCAL_PROFILE_NAMES
    assert "sim" in SIMULATION_PROFILES


def test_physical_host_defaults_do_not_own_driver_backend_defaults() -> None:
    groups = (
        FIELD_PRODUCT_HOST_DEFAULTS,
        LOCAL_HOST_DEFAULTS,
    )
    for defaults in groups:
        for profile, config in defaults.items():
            assert "_driver_backend" not in config, profile
            assert "dog_host" not in config, profile
            assert "dog_port" not in config, profile
            assert "auto_enable" not in config, profile
            assert "auto_standup" not in config, profile


def test_product_support_lives_only_in_env_catalog() -> None:
    graph = load_runtime_graph()

    assert profile_adapter_names_for_profile("nav") == ()
    assert profile_adapter_names_for_profile("lite") == ("thunder_lite",)
    assert profile_adapter_names_for_profile("super_lio") == ()
    assert set(graph.envs["real"]["supported_products"]) == set(FIELD_PRODUCT_NAMES)
    assert set(graph.envs["sim"]["supported_products"]) < set(FIELD_PRODUCT_NAMES)
    for adapter in CATALOG_ADAPTERS.values():
        assert set(adapter.supported_profiles) <= set(HOST_PROFILE_DEFAULTS)
        assert not hasattr(adapter, "supported_products")
        assert not hasattr(adapter, "product_overrides")


def test_field_product_layers_do_not_write_legacy_planner_backend() -> None:
    for profile in FIELD_PRODUCT_NAMES:
        assert "planner_backend" not in FIELD_PRODUCT_HOST_DEFAULTS[profile], profile

    graph = load_runtime_graph()
    assert "planner_backend" not in graph.envs["real"]["host_config"]
    for backend, implementation in graph.envs["sim"]["backends"].items():
        assert "planner_backend" not in implementation["host_config"], backend


def test_runtime_paths_own_shared_runtime_defaults() -> None:
    assert DEFAULT_GATEWAY_PORT == 5050
    assert DEFAULT_PLANNING_FRAME_ID == RUNTIME_MAP_FRAME_ID
    assert RUNTIME_MAP_FRAME_ID == "map"
    assert RUNTIME_ODOM_FRAME_ID == "odom"
    assert DEFAULT_SAMPLE_OCTOPLANNER3D_MAP.endswith("result_cleaned.bt")

    octo_map = Path(_resolve_octoplanner3d_map()).as_posix()
    assert octo_map.endswith((".bt", ".ot", ".octomap", ".pcd"))


def test_catalog_profiles_consume_shared_runtime_defaults() -> None:
    assert HOST_PROFILE_DEFAULTS["lite"]["gateway_port"] == DEFAULT_GATEWAY_PORT
    assert HOST_PROFILE_DEFAULTS["lite"]["planning_frame_id"] == DEFAULT_PLANNING_FRAME_ID
    assert FIELD_PRODUCT_HOST_DEFAULTS["nav"]["gateway_port"] == DEFAULT_GATEWAY_PORT
    assert HOST_PROFILE_DEFAULTS["sim"]["map_path"] == _resolve_octoplanner3d_map()
    assert MUJOCO_LIVE_CONFIG["gateway_port"] == DEFAULT_GATEWAY_PORT
    assert MUJOCO_LIVE_CONFIG["planning_frame_id"] == DEFAULT_PLANNING_FRAME_ID
    assert MUJOCO_LIVE_CONFIG["enable_map_out"] is False
    assert "enable_nav_out" not in MUJOCO_LIVE_CONFIG

    assert (
        CATALOG_ADAPTERS["mujoco_live"].profile_overrides["sim_mujoco_octo_live"]["map_path"]
        == _resolve_octoplanner3d_map()
    )


def test_nav_product_does_not_own_field_bridge_policy() -> None:
    assert "slam_profile" not in FIELD_PRODUCT_HOST_DEFAULTS["nav"]
    assert "endpoint_transport" not in FIELD_PRODUCT_HOST_DEFAULTS["nav"]
    assert profile_adapter_names_for_profile("nav") == ()


def _ros_bridge_keys(config: dict) -> list[str]:
    return [key for key in config if key.startswith("enable_ros2")]


def test_host_defaults_do_not_own_ros_bridge_switches() -> None:
    for profile, defaults in HOST_PROFILE_DEFAULTS.items():
        assert _ros_bridge_keys(defaults) == [], profile


def test_host_defaults_do_not_use_retired_hw_config_keys() -> None:
    compat_keys = {"enable_device_manager", "device_manager_bridge"}
    for profile, config in HOST_PROFILE_DEFAULTS.items():
        assert compat_keys.isdisjoint(config), profile


def test_host_defaults_do_not_enable_ros2_rerun_bridge() -> None:
    for profile, config in HOST_PROFILE_DEFAULTS.items():
        for key in ROS2_RERUN_BRIDGE_ENABLE_KEYS:
            assert key not in config, profile


def test_host_defaults_do_not_default_to_ros2_autonomy_backends() -> None:
    for profile, config in HOST_PROFILE_DEFAULTS.items():
        enable_native = bool(config.get("enable_native", False))
        assert ros2_autonomy_backend_violations(config, enable_native=enable_native) == [], profile


def test_default_field_runtime_configs_do_not_select_ros2_bindings() -> None:
    for profile in FIELD_PRODUCT_NAMES:
        resolved = resolve_product_host_runtime(profile, "real")
        config = resolved.config
        enable_native = bool(config.get("enable_native", False))
        assert ros2_runtime_binding_violations(config, enable_native=enable_native) == [], profile

    mapped_explore = resolve_product_host_runtime(
        "explore",
        "real",
        product_variant="map",
    )
    assert ros2_runtime_binding_violations(
        mapped_explore.config,
        enable_native=bool(mapped_explore.config.get("enable_native", False)),
    ) == []


def test_physical_host_configs_use_ros_free_autonomy_backends() -> None:
    expected = {
        "teleop": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "teleop_avoid": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "lite": {
            "terrain_backend": "simple",
            "local_planner_backend": "simple",
            "path_follower_backend": "pid",
        },
        "map": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "nav": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "tracking": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "inspection": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
        "explore": {
            "terrain_backend": "nanobind",
            "local_planner_backend": "nanobind",
            "path_follower_backend": "nav_kernel",
        },
    }

    physical_profiles = (
        set(FIELD_PRODUCT_NAMES)
        | set(LOCAL_PROFILE_NAMES)
    )
    assert set(expected) == physical_profiles

    for profile in expected:
        resolved = _resolve_named_runtime(profile)
        config = resolved.config
        enable_native = bool(config.get("enable_native", False))

        assert resolved_autonomy_backend_selection(config, enable_native=enable_native) == expected[profile]

    mapped_explore = resolve_product_host_runtime(
        "explore",
        "real",
        product_variant="map",
    )
    assert resolved_autonomy_backend_selection(
        mapped_explore.config,
        enable_native=bool(mapped_explore.config.get("enable_native", False)),
    ) == expected["explore"]


def test_mapped_teleop_profiles_enable_cmd_vel_collision_monitor() -> None:
    enabled_profiles = set()
    physical_profiles = (
        *FIELD_PRODUCT_NAMES,
        *LOCAL_PROFILE_NAMES,
    )
    for profile in physical_profiles:
        resolved = _resolve_named_runtime(profile)
        if bool(resolved.config.get("cmd_vel_mux_collision_monitor", False)):
            enabled_profiles.add(profile)

    assert enabled_profiles == {
        "teleop_avoid",
        "map",
        "nav",
        "explore",
    }

    mapped_explore = resolve_product_host_runtime(
        "explore",
        "real",
        product_variant="map",
    )
    assert mapped_explore.config["cmd_vel_mux_collision_monitor"] is True


def test_octoplanner3d_products_use_supported_map_formats() -> None:
    configs = (
        ALL_NAMED_HOST_DEFAULTS["tracking"],
        ALL_NAMED_HOST_DEFAULTS["nav"],
        ALL_NAMED_HOST_DEFAULTS["explore"],
        FIELD_PRODUCT_VARIANT_HOST_DEFAULTS["explore"]["map"],
    )
    for config in configs:
        assert config["planner"] == "octoplanner3d"
        assert config.get("fallback_planner_name", "") == ""
        assert Path(config["map_path"]).suffix.lower() in {
            ".bt",
            ".ot",
            ".octomap",
            ".pcd",
        }


def test_profile_adapters_do_not_own_ros_bridge_switches() -> None:
    for adapter_name, adapter in CATALOG_ADAPTERS.items():
        assert _ros_bridge_keys(dict(adapter.config_overrides)) == [], adapter_name
        for profile, overrides in adapter.profile_overrides.items():
            assert _ros_bridge_keys(dict(overrides)) == [], (adapter_name, profile)


def test_real_profile_adapters_do_not_select_legacy_driver_sensor_paths() -> None:
    for adapter_name, adapter in CATALOG_ADAPTERS.items():
        if adapter.simulation_only:
            continue
        adapter_config = dict(adapter.config_overrides)
        for key in LEGACY_SENSOR_BINDING_KEYS:
            assert key not in adapter_config, adapter_name
        for profile, overrides in adapter.profile_overrides.items():
            profile_config = dict(overrides)
            for key in LEGACY_SENSOR_BINDING_KEYS:
                assert key not in profile_config, (adapter_name, profile)


def test_real_products_do_not_select_legacy_driver_sensor_paths() -> None:
    field_products = {
        "map",
        "nav",
        "explore",
    }
    for product in field_products:
        config = dict(ALL_NAMED_HOST_DEFAULTS[product])
        for key in LEGACY_SENSOR_BINDING_KEYS:
            assert key not in config, product

    for key in LEGACY_SENSOR_BINDING_KEYS:
        assert key not in FIELD_PRODUCT_VARIANT_HOST_DEFAULTS["explore"]["map"]


def test_profile_adapters_do_not_enable_ros2_rerun_bridge() -> None:
    for adapter_name, adapter in CATALOG_ADAPTERS.items():
        adapter_config = dict(adapter.config_overrides)
        for key in ROS2_RERUN_BRIDGE_ENABLE_KEYS:
            assert key not in adapter_config, adapter_name
        for profile, overrides in adapter.profile_overrides.items():
            profile_config = dict(overrides)
            for key in ROS2_RERUN_BRIDGE_ENABLE_KEYS:
                assert key not in profile_config, (adapter_name, profile)


def test_product_blueprints_import_host_defaults_from_their_direct_owner() -> None:
    path = SRC / "lingtu" / "assembly" / "products" / "thunder.py"
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))

    modules: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.append(node.module)

    assert not any(module.startswith("cli.") for module in modules)
    assert "runtime.profiles.catalog.runtime_paths" in modules
    assert "lingtu.assembly.catalog.runtime_paths" not in modules


def test_profile_graph_does_not_hardcode_removed_driver_backend_aliases() -> None:
    source = (SRC / "lingtu" / "assembly" / "graph.py").read_text(
        encoding="utf-8-sig"
    )

    assert '"s100p": "ThunderDriver"' not in source
    assert '"navigate": "ThunderDriver"' not in source
    assert "driver_backend_module_name" in source


def test_catalog_octoplanner3d_map_prefers_active_saved_map(monkeypatch, tmp_path) -> None:
    monkeypatch.delenv("LINGTU_OCTOPLANNER3D_MAP", raising=False)
    monkeypatch.delenv("NAV_OCTOMAP", raising=False)
    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))
    active = tmp_path / "warehouse"
    active.mkdir()
    (tmp_path / "active_map.txt").write_text("warehouse\n", encoding="utf-8")
    active_map = active / "octomap.ot"
    active_map.write_bytes(b"# OctoMap OcTree file\n")

    assert Path(_resolve_octoplanner3d_map()) == active_map


def test_catalog_octoplanner3d_map_env_override_wins(monkeypatch, tmp_path) -> None:
    active = tmp_path / "warehouse"
    active.mkdir()
    (tmp_path / "active_map.txt").write_text("warehouse\n", encoding="utf-8")
    (active / "octomap.ot").write_bytes(b"# OctoMap OcTree file\n")
    override = tmp_path / "override.bt"
    override.write_bytes(b"# OctoMap OcTree binary file\n")

    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))
    monkeypatch.setenv("LINGTU_OCTOPLANNER3D_MAP", str(override))
    monkeypatch.delenv("NAV_OCTOMAP", raising=False)

    assert Path(_resolve_octoplanner3d_map()) == override
