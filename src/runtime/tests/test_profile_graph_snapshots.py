import builtins

import pytest

pytestmark = [pytest.mark.sim]

from lingtu.assembly.graph import (
    blueprint_for_profile,
    graph_for_product,
    graph_for_profile,
    resolve_profile_config,
)
from lingtu.assembly.products import (
    FIELD_PRODUCT_HOST_DEFAULTS,
    FIELD_PRODUCT_NAMES,
    resolve_product_host_config,
)
from lingtu.assembly.profile_builder import (
    blueprint_from_run_plan,
    compile_run_plan,
)
from lingtu.assembly.stacks.navigation import (
    autonomy_stack_config,
    frontier_module_config,
    navigation_config,
)
from runtime.contracts.simulation import (
    CANONICAL_NAV_TOPICS,
    SIMULATION_RUNTIME_CONTRACTS,
    runtime_contracts_for_profile,
    simulation_runtime_contract,
)
from runtime.graph.loader import load_runtime_graph, resolve_product_variant_spec
from runtime.profiles.binding_policy import (
    LEGACY_SENSOR_BINDING_KEYS,
    legacy_sensor_binding_violations,
    resolved_autonomy_backend_selection,
)
from runtime.profiles.catalog.driver_backends import DRIVER_BACKENDS
from runtime.profiles.catalog.host_defaults import (
    HOST_PROFILE_DEFAULTS,
    HOST_PROFILE_SNAPSHOT_NAMES,
)
from runtime.profiles.catalog.local_host_defaults import LOCAL_PROFILE_NAMES
from runtime.profiles.catalog.simulation_profiles import SIMULATION_PROFILES
from runtime.profiles.product_lifecycle import (
    OPERATOR_PRODUCT_LIFECYCLES,
    product_lifecycle,
    product_transition_plan,
)
from runtime.profiles.profile_adapters import (
    profile_adapter,
    profile_adapter_names,
    resolve_runtime_run_spec,
)
from runtime.runtime_interface import (
    DATA_SOURCE_CONTRACTS,
    RUNTIME_DATA_FLOW,
    TOPICS,
    product_data_source,
    profile_data_source,
)


def _resolve_selection_config(
    name: str,
    *,
    env: str = "real",
    env_config: dict[str, object] | None = None,
    profile_adapter: str | None = None,
    product_variant: str | None = None,
) -> dict[str, object]:
    if name in FIELD_PRODUCT_HOST_DEFAULTS:
        if profile_adapter is not None:
            raise ValueError("Products select real or sim Env, not a runtime endpoint")
        return resolve_product_host_config(
            name,
            env,
            product_variant=product_variant,
            env_config=env_config,
        )
    if product_variant is not None:
        raise ValueError("local Profiles do not have Product variants")
    return resolve_profile_config(name, profile_adapter=profile_adapter)


def _graph_for_selection(
    name: str,
    *,
    env: str = "real",
    env_config: dict[str, object] | None = None,
    product_variant: str | None = None,
    **kwargs,
):
    if name in FIELD_PRODUCT_HOST_DEFAULTS:
        return graph_for_product(
            name,
            env=env,
            env_config=env_config,
            product_variant=product_variant,
            **kwargs,
        )
    if product_variant is not None:
        raise ValueError("local Profiles do not have Product variants")
    return graph_for_profile(name, **kwargs)


def _product_blueprint(product: str, *, env: str = "real"):
    config = resolve_product_host_config(product, env)
    plan = compile_run_plan(product, env, config)
    return blueprint_from_run_plan(plan)


REAL_LOCALIZATION_SELECTIONS = (
    ("teleop_avoid", None),
    ("map", None),
    ("tracking", None),
    ("nav", None),
    ("inspection", None),
    ("explore", "live"),
    ("explore", "map"),
)

SIMULATION_PROFILE_RUNTIME_MATRIX = {
    "stub": {
        "data_source": "in_process_stub",
        "profile_contracts": (),
        "external_launcher": None,
        "runtime_contract": None,
    },
    "dev": {
        "data_source": "in_process_stub",
        "profile_contracts": (),
        "external_launcher": None,
        "runtime_contract": None,
    },
    "sim_nav": {
        "data_source": "in_process_stub",
        "profile_contracts": (),
        "external_launcher": None,
        "runtime_contract": None,
    },
    "sim": {
        "data_source": "mujoco_module_graph",
        "profile_contracts": (),
        "external_launcher": None,
        "runtime_contract": None,
    },
    "portable_mujoco": {
        "data_source": "mujoco_module_graph",
        "profile_contracts": (),
        "external_launcher": None,
        "runtime_contract": None,
    },
    "sim_mujoco_live": {
        "data_source": "mujoco_fastlio2_live",
        "profile_contracts": ("mujoco_fastlio2_live",),
        "external_launcher": "sim/scripts/mujoco/launch_fastlio2_live.sh",
        "runtime_contract": "mujoco_fastlio2_live",
    },
    "sim_mujoco_octo_live": {
        "data_source": "mujoco_fastlio2_live",
        "profile_contracts": ("mujoco_fastlio2_live",),
        "external_launcher": "sim/scripts/mujoco/launch_fastlio2_live.sh",
        "runtime_contract": "mujoco_fastlio2_live",
    },
}


def test_product_modes_are_locked_to_product_declarations():
    expected = {
        "teleop": "teleop",
        "teleop_avoid": "teleop_avoid",
        "map": "mapping",
        "tracking": "tracking",
        "nav": "navigation",
        "inspection": "inspection",
        "explore": "exploration",
    }

    products = load_runtime_graph().products
    for product, mode in expected.items():
        assert products[product]["product_mode"] == mode
        assert "product_mode" not in FIELD_PRODUCT_HOST_DEFAULTS[product]
        assert product_data_source(product).data_source in DATA_SOURCE_CONTRACTS


def test_runtime_data_flow_stages_define_operator_relevant_contract():
    for stage in RUNTIME_DATA_FLOW:
        assert stage.producer, stage.name
        assert stage.consumers, stage.name
        assert stage.frequency, stage.name
        assert stage.transport_policy, stage.name


def test_product_modes_start_and_forbid_expected_module_groups():
    for profile, product in load_runtime_graph().products.items():
        modules = set(graph_for_product(profile, env="real").modules)
        assert set(product.get("forbidden_modules", ())).isdisjoint(modules), profile


def test_runtime_product_modes_start_and_forbid_expected_module_groups():
    from lingtu.plugin_seed import install_builtin_plugin_catalog

    install_builtin_plugin_catalog()
    for profile, product in load_runtime_graph().products.items():
        graph = graph_for_product(
            profile,
            env="real",
            mode="runtime",
            run_startup_checks=False,
        )
        modules = set(graph.modules)
        assert set(product.get("forbidden_modules", ())).isdisjoint(modules), profile


def test_product_lifecycles_do_not_duplicate_compiled_product_data():
    forbidden = {
        "processes",
        "required_topics",
        "required_capabilities",
        "forbidden_modules",
        "native_nav",
        "required_modules",
        "required_wires",
    }
    for lifecycle in OPERATOR_PRODUCT_LIFECYCLES.values():
        payload = lifecycle.as_dict()
        assert forbidden.isdisjoint(payload)
        assert payload["product"] == lifecycle.product
        assert "profile" not in payload


def test_product_lifecycles_lock_operator_session_vocabulary():
    expected = {
        "teleop": "teleop",
        "teleop_avoid": "teleop_avoid",
        "map": "mapping",
        "tracking": "tracking",
        "nav": "navigation",
        "inspection": "inspection",
        "explore": "exploration",
    }

    assert {
        profile: contract.product_session
        for profile, contract in OPERATOR_PRODUCT_LIFECYCLES.items()
    } == expected
    for profile, contract in OPERATOR_PRODUCT_LIFECYCLES.items():
        assert contract.as_dict()["product_session"] == expected[profile]

    mapped_explore = product_lifecycle("explore", product_variant="map")
    assert mapped_explore.product_session == "exploration"
    assert mapped_explore.slam_mode == "localization"
    assert mapped_explore.requires_map is True


def test_endpoint_only_modes_move_collision_avoidance_out_of_python_mux():
    from lingtu.plugin_seed import install_builtin_plugin_catalog

    install_builtin_plugin_catalog()
    mapped_teleop_selections = (
        ("teleop_avoid", None),
        ("map", None),
        ("nav", None),
        ("explore", "live"),
        ("explore", "map"),
    )
    collision_costmap_wire = "TraversabilityCostModule.fused_cost->nav.velocity_mux.collision_costmap"
    for mode in ("static", "runtime"):
        kwargs = (
            {
                "mode": "runtime",
                "run_startup_checks": False,
            }
            if mode == "runtime"
            else {}
        )
        teleop_avoid = graph_for_product("teleop_avoid", env="real", **kwargs)
        wires = _wire_set(teleop_avoid)
        modules = set(teleop_avoid.modules)

        assert "nav.local_planner" not in modules
        assert "nav.path_follower" not in modules
        assert "nav.velocity_mux" not in modules
        assert not any("nav.velocity_mux" in wire for wire in wires)
        assert "TraversabilityCostModule.fused_cost->nav.mission.costmap" not in wires

    for profile, product_variant in mapped_teleop_selections:
        profile_graph = _graph_for_selection(
            profile,
            product_variant=product_variant,
        )
        label = f"{profile}/{product_variant}" if product_variant else profile
        assert "nav.velocity_mux" not in profile_graph.modules, label
        assert not any(
            "nav.velocity_mux" in wire for wire in _wire_set(profile_graph)
        ), label

    for profile in ("teleop", "tracking", "inspection"):
        profile_wires = _wire_set(graph_for_product(profile, env="real"))
        assert collision_costmap_wire not in profile_wires, profile

def test_visual_servo_profiles_wire_gateway_hot_entry():
    visual_servo_selections = (
        ("nav", None),
        ("explore", "live"),
        ("explore", "map"),
        ("inspection", None),
    )
    hot_entry_wire = "GatewayModule.servo_target->VisualServoModule.servo_target"

    for profile, product_variant in visual_servo_selections:
        graph = _graph_for_selection(profile, product_variant=product_variant)
        label = f"{profile}/{product_variant}" if product_variant else profile
        assert "VisualServoModule" in graph.modules, label
        assert hot_entry_wire in _wire_set(graph), label

    for profile in ("teleop", "teleop_avoid", "map", "tracking"):
        graph = graph_for_product(profile, env="real")
        assert "VisualServoModule" not in graph.modules, profile
        assert hot_entry_wire not in _wire_set(graph), profile


def test_product_mode_switch_contracts_require_cold_restart():
    plan = product_transition_plan("tracking", "inspection")
    assert plan["current"]["product"] == "tracking"
    assert plan["target"]["product"] == "inspection"
    assert plan["same_graph_candidate"] is False
    assert plan["online_hot_switch_supported"] is False
    assert plan["required_lifecycle"] == "cold_restart"

    teleop_to_nav = product_transition_plan("teleop", "nav")
    assert teleop_to_nav["same_graph_candidate"] is False
    assert teleop_to_nav["required_lifecycle"] == "cold_restart"


def test_top_level_blueprint_api_exposes_all_stack_factories():
    import lingtu.assembly as blueprints

    for name in (
        "driver",
        "exploration",
        "gateway",
        "lidar",
        "maps",
        "memory",
        "navigation",
        "perception",
        "planner",
        "safety",
        "sim_lidar",
        "slam",
    ):
        assert callable(getattr(blueprints, name))
        assert name in blueprints.__all__


def _wire_set(graph):
    return {wire.as_snapshot() for wire in graph.explicit_wires}


def test_static_profile_graph_does_not_import_runtime_message_stack(monkeypatch):
    real_import = builtins.__import__

    def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "numpy" or name.startswith("runtime.msgs"):
            raise AssertionError(f"static graph imported runtime message stack: {name}")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", guarded_import)

    graph = graph_for_product("explore", env="real")
    assert "nav.mission" not in graph.modules
    assert "host.bus" in graph.modules
    assert "nav.out" not in graph.modules
    assert "ThunderDriver" not in graph.modules


def test_profile_graphs_compile_for_primary_profiles():
    for profile in HOST_PROFILE_SNAPSHOT_NAMES:
        graph = _graph_for_selection(profile)
        config = _resolve_selection_config(profile)

        product = load_runtime_graph().products.get(profile, {})
        uses_native_navigation = bool(config.get("native_navigation_endpoint"))
        product_forbids_mission = "nav.mission" in product.get(
            "forbidden_modules",
            (),
        )
        if uses_native_navigation or product_forbids_mission:
            # Field navigation belongs to navd. Teleop/map contracts also
            # explicitly forbid the Python mission stack.
            assert "nav.mission" not in graph.modules
        else:
            assert "nav.mission" in graph.modules
        if config.get("enable_navigation", True):
            assert "nav.skills" in graph.modules
            assert "nav.localization_monitor" in graph.modules
        if config.get("command_output_mode") == "endpoint_only":
            assert "nav.safety" not in graph.modules
            assert "nav.velocity_mux" not in graph.modules
            assert "GeofenceManagerModule" not in graph.modules
            if config.get("enable_gateway", True) and config.get("enable_teleop", True):
                assert "CameraJpegRelayModule" in graph.modules
                assert "TeleopModule" not in graph.modules
        else:
            assert "nav.safety" in graph.modules
        if config.get("enable_gateway", True):
            assert "GatewayModule" in graph.modules
            assert "MCPServerModule" in graph.modules
        else:
            assert "GatewayModule" not in graph.modules
            assert "MCPServerModule" not in graph.modules
        assert not graph.dangling_wires(), profile


def test_runtime_product_materializes_blueprint_from_run_plan():
    from lingtu.plugin_seed import install_builtin_plugin_catalog

    install_builtin_plugin_catalog()
    config = _resolve_selection_config("nav")
    bp = _product_blueprint("nav")
    modules = {entry.name for entry in bp._entries}
    wires = {f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}" for wire in bp._wires}

    assert "nav.out" not in modules
    assert "nav.local_planner" not in modules
    assert "nav.path_follower" not in modules
    assert config.get("native_navigation_endpoint") == "lingtu-nav-dds"
    assert "ThunderDriver" not in modules
    # "nav" uses the native cpp_slam_status localization adapter.
    assert "SlamAdapterModule" in modules
    assert "nav.safety" not in modules
    assert not any(wire.startswith("nav.safety.") for wire in wires)
    assert "GatewayModule.stop_cmd->nav.mission.stop_signal" not in wires
    assert "host.bus.global_path->GatewayModule.global_path" in wires
    assert "host.bus.local_path->GatewayModule.local_path" in wires
    assert "nav.velocity_mux.driver_cmd_vel->nav.out.cmd_vel" not in wires
    assert "nav.velocity_mux.driver_cmd_vel->ThunderDriver.cmd_vel" not in wires


def test_product_identity_is_not_injected_into_host_module_config():
    local_blueprint = blueprint_for_profile("sim_nav")
    product_blueprint = _product_blueprint("nav")

    local_gateway = next(
        entry for entry in local_blueprint._entries if entry.name == "GatewayModule"
    )
    product_gateway = next(
        entry for entry in product_blueprint._entries if entry.name == "GatewayModule"
    )

    assert local_gateway.config.get("product") is None
    assert product_gateway.config.get("product") is None


def test_simulation_profiles_match_runtime_data_source_matrix():
    assert set(SIMULATION_PROFILE_RUNTIME_MATRIX) == set(SIMULATION_PROFILES)

    for profile, expected in SIMULATION_PROFILE_RUNTIME_MATRIX.items():
        config = _resolve_selection_config(profile)
        binding = profile_data_source(profile)
        source = DATA_SOURCE_CONTRACTS[binding.data_source]
        contracts = runtime_contracts_for_profile(profile)

        assert binding.data_source == expected["data_source"], profile
        assert source.provider != "hardware", profile
        assert tuple(contract.name for contract in contracts) == expected["profile_contracts"], profile
        assert HOST_PROFILE_DEFAULTS[profile].get("_external_launcher") == expected["external_launcher"], profile
        assert HOST_PROFILE_DEFAULTS[profile].get("_runtime_contract") == expected["runtime_contract"], profile
        assert "_external_launcher" not in config, profile
        assert "_runtime_contract" not in config, profile
        for contract in contracts:
            assert contract.data_source_contract == binding.data_source, profile
            assert contract.simulation_only is True, profile


def test_local_simulation_profile_adapters_generate_coherent_runtime_run_specs():
    # Gazebo and CMU Unity are Product env backends, not local Profile
    # adapters. Only adapters that can launch a local Profile belong here.
    for endpoint_name in ("mujoco_live",):
        endpoint = profile_adapter(endpoint_name)
        source = DATA_SOURCE_CONTRACTS[endpoint.data_source]

        assert endpoint.simulation_only is True
        assert source.provider != "hardware"
        assert endpoint.runtime_contract

        for profile in endpoint.supported_profiles:
            config = _resolve_selection_config(profile, profile_adapter=endpoint_name)
            spec = resolve_runtime_run_spec(profile, config)

            assert spec.adapter == endpoint_name, profile
            assert spec.data_source == endpoint.data_source, profile
            assert spec.runtime_contract == endpoint.runtime_contract, profile
            assert spec.simulation_only is True, profile
            assert spec.command_sink == source.command_sink, profile
            if endpoint.external_launcher:
                assert spec.launcher == endpoint.external_launcher, profile
                assert spec.launcher_args == endpoint.default_actions[profile], profile
            else:
                assert spec.launcher is None, profile
                assert spec.launcher_args == (), profile
                assert spec.as_command() == [], profile
            assert spec.env["LINGTU_PROFILE"] == profile
            assert spec.env["LINGTU_PROFILE_ADAPTER"] == endpoint_name
            assert spec.env["LINGTU_DATA_SOURCE"] == endpoint.data_source
            assert spec.env["LINGTU_MODULE_TRANSPORT"] == endpoint.module_transport
            assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == endpoint.endpoint_transport
            assert spec.env["LINGTU_RUNTIME_CONTRACT"] == endpoint.runtime_contract
            assert spec.env["LINGTU_COMMAND_SINK"] == source.command_sink
            assert spec.env["LINGTU_SIMULATION_ONLY"] == "1"


def test_profile_graph_snapshot_locks_safety_gateway_and_mux_edges():
    for profile in HOST_PROFILE_SNAPSHOT_NAMES:
        graph = _graph_for_selection(profile)
        wires = _wire_set(graph)
        modules = set(graph.modules)

        driver = next(
            (
                module
                for module in modules
                if module.endswith("Driver") or module.endswith("DriverModule") or module.endswith("DogModule")
            ),
            None,
        )

        if {"nav.safety", "nav.mission"} <= modules:
            assert "nav.safety.stop_cmd->nav.mission.stop_signal" in wires
        elif "nav.safety" not in modules:
            assert not any(wire.startswith("nav.safety.") for wire in wires)
        if "GatewayModule" in modules:
            if "host.bus" in modules:
                assert "GatewayModule.stop_cmd->nav.mission.stop_signal" not in wires
                assert "host.bus.global_path->GatewayModule.global_path" in wires
                assert "host.bus.local_path->GatewayModule.local_path" in wires
                assert "nav.mission.mission_status->GatewayModule.mission_status" not in wires
            elif "nav.mission" in modules:
                assert "GatewayModule.stop_cmd->nav.mission.stop_signal" in wires
            if "nav.velocity_mux" in modules:
                assert "GatewayModule.cmd_vel->nav.velocity_mux.teleop_cmd_vel" in wires
            else:
                assert "GatewayModule.cmd_vel->nav.velocity_mux.teleop_cmd_vel" not in wires
            if "nav.mission" in modules and "host.bus" not in modules:
                assert "nav.mission.mission_status->GatewayModule.mission_status" in wires
            if "nav.goals" in modules and "nav.mission" in modules:
                assert "GatewayModule.goal_pose->nav.goals.goal_request" in wires
                assert "GatewayModule.cancel->nav.goals.cancel_request" in wires
                assert "nav.goals.goal_pose->nav.mission.goal_pose" in wires
                assert "nav.goals.cancel->nav.mission.cancel" in wires
            elif "nav.mission" in modules:
                assert "GatewayModule.cancel->nav.mission.cancel" in wires
        if "MCPServerModule" in modules:
            if "nav.mission" in modules and "host.bus" not in modules:
                assert "MCPServerModule.stop_cmd->nav.mission.stop_signal" in wires
            if "nav.velocity_mux" in modules:
                assert "MCPServerModule.cmd_vel->nav.velocity_mux.teleop_cmd_vel" in wires
            else:
                assert "MCPServerModule.cmd_vel->nav.velocity_mux.teleop_cmd_vel" not in wires
            if "nav.mission" in modules and "host.bus" not in modules:
                assert "nav.mission.mission_status->MCPServerModule.mission_status" in wires
            if "nav.goals" in modules and "nav.mission" in modules:
                assert "MCPServerModule.goal_pose->nav.goals.goal_request" in wires
        if "GeofenceManagerModule" in modules and "nav.mission" in modules:
            assert "GeofenceManagerModule.stop_cmd->nav.mission.stop_signal" in wires
        if driver is not None:
            if "nav.safety" in modules:
                assert f"nav.safety.stop_cmd->{driver}.stop_signal" in wires
            if "GatewayModule" in modules:
                assert f"GatewayModule.stop_cmd->{driver}.stop_signal" in wires
            if "MCPServerModule" in modules:
                assert f"MCPServerModule.stop_cmd->{driver}.stop_signal" in wires
            if "GeofenceManagerModule" in modules:
                assert f"GeofenceManagerModule.stop_cmd->{driver}.stop_signal" in wires
            if "nav.velocity_mux" in modules:
                assert f"nav.velocity_mux.driver_cmd_vel->{driver}.cmd_vel" in wires
        else:
            assert "nav.out" not in modules
            assert f"nav.velocity_mux.driver_cmd_vel->nav.out.cmd_vel@{TOPICS.cmd_vel}" not in wires
        if {"nav.mission", "nav.local_planner"} <= modules:
            assert "nav.mission.clear_path->nav.local_planner.clear_path" in wires
            assert "nav.mission.global_path->nav.local_planner.global_path" in wires
        if {"nav.local_planner", "nav.path_follower"} <= modules:
            assert "nav.local_planner.control_hint->nav.path_follower.control_hint" in wires


def test_profile_graph_snapshot_locks_mapping_and_algorithm_edges():
    for profile in (
        "stub",
        "dev",
        "sim",
        "sim_mujoco_live",
        "map",
        "nav",
        "explore",
    ):
        graph = _graph_for_selection(profile)
        wires = _wire_set(graph)
        modules = set(graph.modules)

        if {"OccupancyGridModule", "TraversabilityCostModule"} <= modules:
            assert "OccupancyGridModule.costmap->TraversabilityCostModule.costmap" in wires
        if {"ESDFModule", "TraversabilityCostModule"} <= modules:
            assert "ESDFModule.esdf->TraversabilityCostModule.esdf" in wires
        if {"TraversabilityCostModule", "nav.mission"} <= modules:
            assert "TraversabilityCostModule.fused_cost->nav.mission.costmap" in wires
        if {"nav.terrain", "nav.mission"} <= modules:
            assert "nav.terrain.traversability->nav.mission.traversability" in wires
        if {"TraversabilityCostModule", "GatewayModule"} <= modules:
            assert "TraversabilityCostModule.fused_cost->GatewayModule.costmap" in wires
        if {"SemanticMapperModule", "SemanticPlannerModule"} <= modules:
            assert "SemanticMapperModule.topo_summary->SemanticPlannerModule.topo_summary" in wires
            assert "SemanticMapperModule.room_graph->SemanticPlannerModule.room_graph" in wires


# Profiles that assemble the full map + autonomy chain (occupancy/ESDF ->
# traversability -> nav, plus terrain -> local planner -> path follower).
_NAV_CONTRACT_PROFILES = (
    "stub",
    "dev",
    "sim",
    "portable_mujoco",
)


def test_navigation_compute_contract_layer_main_edges():
    """NAV COMPUTE CONTRACT: the four-layer main chain must be wired.

    docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md §2/§7.
      L5 global -> L2 gating -> L2 local -> L2 control.
    """
    for profile in _NAV_CONTRACT_PROFILES:
        wires = _wire_set(graph_for_profile(profile))

        # L2 safety gating: fused_cost (risk grid) -> global gate only.
        assert "TraversabilityCostModule.fused_cost->nav.mission.costmap" in wires, profile
        # L2 local planning: terrain_map (local geometry) is the main local input.
        assert "nav.terrain.terrain_map->nav.local_planner.terrain_map" in wires, profile
        assert "nav.terrain.traversability->nav.local_planner.traversability" in wires, profile
        # L5 -> L2 command dispatch (global_path + waypoint as staged goals).
        assert "nav.mission.global_path->nav.local_planner.global_path" in wires, profile
        assert "nav.mission.waypoint->nav.local_planner.waypoint" in wires, profile
        # L2 control tracking: local_path -> path follower -> cmd_vel.
        assert "nav.local_planner.local_path->nav.path_follower.local_path" in wires, profile


def test_navigation_compute_contract_forbids_role_drift_edges():
    """NAV COMPUTE CONTRACT §3/§7: forbidden edges must never appear.

    - fused_cost/costmap must NOT feed the local planner's primary scoring.
    - terrain_map (local geometry) must NOT feed global strategy.
    """
    for profile in HOST_PROFILE_SNAPSHOT_NAMES:
        wires = _wire_set(graph_for_profile(profile))
        for wire in wires:
            assert not wire.startswith("TraversabilityCostModule.fused_cost->LocalPlanner"), (
                f"{profile}: costmap must not be a local-planner input ({wire})"
            )
            assert not wire.startswith("nav.terrain.terrain_map->Navigation"), (
                f"{profile}: terrain_map must not drive global planning ({wire})"
            )


def test_nav_profile_routes_slam_adapter_state_to_field_host_consumers():
    graph = graph_for_product("nav", env="real")
    wires = _wire_set(graph)

    health = TOPICS.localization_health
    quality = TOPICS.localization_quality

    assert graph.name == "nav"
    assert graph.source_kind == "product"
    assert graph.env == "real"
    assert "nav.mission" not in graph.modules
    assert "nav.local_planner" not in graph.modules
    assert "nav.path_follower" not in graph.modules
    assert "DepthVisualOdomModule" not in graph.modules
    assert f"SlamAdapterModule.localization_status->nav.safety.localization_status@{health}" not in wires
    assert f"SlamAdapterModule.localization_status->nav.mission.localization_status@{health}" not in wires
    assert (
        f"SlamAdapterModule.localization_status->DepthVisualOdomModule.localization_status@{health}"
        not in wires
    )
    assert f"SlamAdapterModule.localization_status->GatewayModule.localization_status@{health}" in wires
    assert f"SlamAdapterModule.localization_quality->GatewayModule.localization_quality@{quality}" in wires
    assert f"SlamAdapterModule.lidar_scan->GatewayModule.lidar_scan[local]@{TOPICS.lidar_scan}" in wires
    assert "SlamAdapterModule.map_odom_tf->GatewayModule.map_odom_tf" in wires
    assert not any("->nav.mission." in wire for wire in wires)
    assert not graph.dangling_wires()


def test_real_products_use_native_command_boundary_by_default():
    for product in FIELD_PRODUCT_NAMES:
        config = _resolve_selection_config(product)
        graph = graph_for_product(product, env="real")
        wires = _wire_set(graph)

        assert config["_env"] == "real"
        assert config["_endpoint_transport"] == "dds"
        assert config["_endpoint_contract"] == "thunder_dds_v1"
        assert config["localization_adapter"] == "cpp_slam_status"
        assert config["enable_hw"] is False
        assert config["enable_robot_driver"] is False
        assert config["enable_lidar"] is False
        assert config["enable_imu"] is False
        assert "lidar_backend" not in config
        assert "imu_backend" not in config
        assert config["native_navigation_endpoint"] == "lingtu-nav-dds"
        assert "enable_nav_in" not in config
        assert "enable_nav_out" not in config
        assert config["enable_map_out"] is False
        assert config["enable_map_layers"] is False
        assert config["enable_visual_backup"] is False
        assert legacy_sensor_binding_violations(config) == []
        for key in LEGACY_SENSOR_BINDING_KEYS:
            assert key not in config or config[key] is False, (product, key)
        assert "nav.in" not in graph.modules
        assert "nav.out" not in graph.modules
        assert "map.out" not in graph.modules
        assert "OccupancyGridModule" not in graph.modules
        assert "VoxelGridModule" not in graph.modules
        assert "ESDFModule" not in graph.modules
        assert "ElevationMapModule" not in graph.modules
        assert "TraversabilityCostModule" not in graph.modules
        if config.get("enable_map_modules", True):
            assert "maps.service" in graph.modules
        else:
            assert "maps.service" not in graph.modules
        assert "nav.terrain" not in graph.modules
        assert "nav.local_planner" not in graph.modules
        assert "nav.path_follower" not in graph.modules
        assert "hw" not in graph.modules
        assert "lidar" not in graph.modules
        assert "imu" not in graph.modules
        assert "ThunderDriver" not in graph.modules
        assert "LidarModule" not in graph.modules
        assert "MujocoDriverModule" not in graph.modules
        assert "GnssBridgeModule" not in graph.modules
        assert "ROS2SimDriverModule" not in graph.modules
        assert "nav.velocity_mux" not in graph.modules
        assert not any("nav.velocity_mux" in wire for wire in wires)
        if product != "teleop":
            assert "SlamAdapterModule" in graph.modules
        assert not any(
            wire.startswith(
                (
                    "lidar.",
                    "imu.",
                    "LidarModule.",
                    "ThunderDriver.",
                    "MujocoDriverModule.",
                )
            )
            for wire in wires
        )
        assert f"nav.velocity_mux.driver_cmd_vel->nav.out.cmd_vel@{TOPICS.cmd_vel}" not in wires
        assert "nav.velocity_mux.driver_cmd_vel->ThunderDriver.cmd_vel" not in wires
        assert "nav.mission.global_path->nav.local_planner.global_path" not in wires
        assert "nav.mission.waypoint->nav.local_planner.waypoint" not in wires
        assert "nav.terrain.terrain_map->nav.local_planner.terrain_map" not in wires
        assert "nav.local_planner.local_path->nav.path_follower.local_path" not in wires
        assert "nav.path_follower.cmd_vel->nav.velocity_mux.path_follower_cmd_vel" not in wires

        has_mission_stack = "nav.mission" in graph.modules
        if has_mission_stack:
            assert f"nav.mission.global_path->nav.out.global_path@{TOPICS.global_path}" not in wires
            assert f"nav.local_planner.local_path->nav.out.local_path@{TOPICS.local_path}" not in wires
            assert f"nav.mission.waypoint->nav.out.waypoint@{TOPICS.nav_way_point}" not in wires
            assert f"SlamAdapterModule.odometry->nav.mission.odometry@{TOPICS.odometry}" in wires
            assert (
                "SlamAdapterModule.localization_status->nav.mission.localization_status"
                f"@{TOPICS.localization_health}" in wires
            )
        else:
            # "map" runs mapping-only: its contract forbids nav.mission and
            # nav.local_planner, so it has no navigation execution outputs.
            assert "nav.local_planner" not in graph.modules


def test_map_product_rejects_profile_driver_backend_bypass():
    with pytest.raises(TypeError, match="reserved Product resolution key"):
        graph_for_product("map", env="real", driver_backend="thunder")


def test_real_dds_semantic_selections_use_dds_camera_role():
    for profile, product_variant in (
        ("nav", None),
        ("inspection", None),
        ("explore", "live"),
        ("explore", "map"),
    ):
        config = _resolve_selection_config(
            profile,
            product_variant=product_variant,
        )
        graph = _graph_for_selection(
            profile,
            product_variant=product_variant,
        )
        wires = _wire_set(graph)

        assert config["_env"] == "real"
        assert config["camera_backend"] == "dds"
        assert config["enable_camera"] is True
        assert "camera" in graph.modules
        assert "camera.color_image->PerceptionModule.color_image" in wires
        assert "camera.depth_image->PerceptionModule.depth_image" in wires
        assert "camera.camera_info->PerceptionModule.camera_info" in wires


def test_real_map_product_keeps_dds_camera_without_semantic_stack():
    config = _resolve_selection_config("map")
    graph = graph_for_product("map", env="real")
    wires = _wire_set(graph)

    assert config["enable_semantic"] is False
    assert config["camera_backend"] == "dds"
    assert config["enable_camera"] is True
    assert "camera" in graph.modules
    assert "PerceptionModule" not in graph.modules
    assert "camera.color_image->CameraJpegRelayModule.color_image" in wires


def test_static_profile_graph_honors_gnss_backend_without_hw_bridge(monkeypatch):
    class FakeConfig:
        raw = {
            "gnss": {
                "enabled": True,
                "model": "WTRTK-980",
                "device": "/dev/wtrtk980",
                "gnss_backend": "replay",
            }
        }

    monkeypatch.setattr("runtime.config.get_config", lambda: FakeConfig())

    graph = graph_for_product(
        "nav",
        env="real",
        enable_gnss=True,
        gnss_backend="replay",
    )

    assert "gnss" in graph.modules
    assert "GnssBridgeModule" not in graph.modules


def test_static_profile_graph_omits_native_wtrtk980_python_module(monkeypatch):
    class FakeConfig:
        raw = {
            "gnss": {
                "enabled": True,
                "model": "WTRTK-980",
                "device": "/dev/wtrtk980",
                "gnss_backend": "wtrtk980",
            }
        }

    monkeypatch.setattr("runtime.config.get_config", lambda: FakeConfig())

    graph = graph_for_product(
        "nav",
        env="real",
        enable_gnss=True,
        gnss_backend="wtrtk980",
    )

    assert "GnssModule" not in graph.modules
    assert "GnssBridgeModule" not in graph.modules


@pytest.mark.sim
def test_portable_mujoco_profile_is_no_ros_planning_sensor_entry():
    graph = graph_for_profile("portable_mujoco")
    wires = _wire_set(graph)
    config = _resolve_selection_config("portable_mujoco")
    source = DATA_SOURCE_CONTRACTS[profile_data_source("portable_mujoco").data_source]

    assert config["robot"] == "sim_mujoco"
    assert config["world"] == "open_field"
    assert config["drive_mode"] == "kinematic"
    assert config["slam_profile"] == "none"
    assert config["planner"] == "octoplanner3d"
    assert config["enable_native"] is False
    assert config["enable_semantic"] is False
    assert config["enable_gateway"] is False
    assert config["enable_camera"] is True
    assert config["use_driver_camera"] is True
    assert config["planner_profile"]["latency_budget_ms"] == 250
    assert source.name == "mujoco_module_graph"
    assert source.provider == "mujoco"
    assert TOPICS.camera_color in source.normalized_outputs
    assert TOPICS.camera_depth in source.normalized_outputs
    assert TOPICS.camera_info in source.normalized_outputs
    assert TOPICS.height_rays in source.normalized_outputs
    assert source.algorithm_context_outputs == (TOPICS.height_rays,)
    assert "fixed_terrain_height_rays" in source.owns
    assert "MujocoDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "GatewayModule" not in graph.modules
    assert "MCPServerModule" not in graph.modules
    assert "camera" not in graph.modules
    assert "MujocoDriverModule.odometry->nav.mission.odometry" in wires
    assert "MujocoDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "nav.mission.global_path->nav.local_planner.global_path" in wires
    assert "nav.local_planner.local_path->nav.path_follower.local_path" in wires
    assert "nav.velocity_mux.driver_cmd_vel->MujocoDriverModule.cmd_vel" in wires
    assert not graph.dangling_wires()


@pytest.mark.sim
def test_sim_mujoco_live_profile_is_raw_fastlio_simulation_entry():
    graph = graph_for_profile("sim_mujoco_live")
    wires = _wire_set(graph)
    config = _resolve_selection_config("sim_mujoco_live")

    assert config["robot"] == "sim_endpoint"
    assert config["slam_profile"] == "none"
    assert HOST_PROFILE_DEFAULTS["sim_mujoco_live"]["_external_launcher"] == (
        "sim/scripts/mujoco/launch_fastlio2_live.sh"
    )
    assert HOST_PROFILE_DEFAULTS["sim_mujoco_live"]["_runtime_contract"] == "mujoco_fastlio2_live"
    assert config["planning_frame_id"] == "map"
    assert config["enable_frontier"] is True
    assert config["enable_traversable_frontier"] is True
    assert config["exploration_backend"] == "none"
    assert config["enable_native"] is False
    assert config["enable_semantic"] is False
    assert config["enable_teleop"] is False
    assert config["run_startup_checks"] is False
    assert config["cloud_topic"] == TOPICS.map_cloud
    assert "SimEndpointDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "WavefrontFrontierExplorer" in graph.modules
    assert "TraversableFrontierModule" in graph.modules
    assert "nav.mission" in graph.modules
    assert "MujocoDriverModule" not in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "SimEndpointDriverModule.odometry->nav.mission.odometry" in wires
    assert "SimEndpointDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "SimEndpointDriverModule.map_cloud->nav.terrain.map_cloud" in wires
    assert "WavefrontFrontierExplorer.exploration_goal->nav.mission.goal_pose" in wires
    assert "nav.mission.mission_status->WavefrontFrontierExplorer.navigation_status" in wires
    assert "TraversableFrontierModule.traversable_frontiers->GatewayModule.traversable_frontiers" in wires
    assert "TraversableFrontierModule.frontier_candidate->GatewayModule.frontier_candidate" in wires
    assert "TraversableFrontierModule.frontier_candidate->nav.mission.goal_pose" not in wires
    assert not graph.dangling_wires()


@pytest.mark.sim
def test_sim_mujoco_octo_live_profile_is_octoplanner3d_closed_loop_entry():
    graph = graph_for_profile("sim_mujoco_octo_live")
    wires = _wire_set(graph)
    config = _resolve_selection_config("sim_mujoco_octo_live")

    assert config["robot"] == "sim_endpoint"
    assert config["slam_profile"] == "none"
    assert HOST_PROFILE_DEFAULTS["sim_mujoco_octo_live"]["_external_launcher"] == (
        "sim/scripts/mujoco/launch_fastlio2_live.sh"
    )
    assert HOST_PROFILE_DEFAULTS["sim_mujoco_octo_live"]["_runtime_contract"] == "mujoco_fastlio2_live"
    assert config["planner"] == "octoplanner3d"
    assert config["planner_backend"] == "octoplanner3d"
    assert config["map_path"].endswith((".ot", ".bt"))
    assert config["plan_safety_policy"] == "reject"
    assert config["fallback_planner_name"] == ""
    assert config["planning_frame_id"] == "map"
    assert config["enable_frontier"] is False
    assert config["enable_traversable_frontier"] is False
    assert config["exploration_backend"] == "none"
    assert config["enable_native"] is False
    assert config["enable_semantic"] is False
    assert config["enable_teleop"] is False
    assert config["local_planner_allow_direct_track_fallback"] is False
    assert config["local_planner_ignore_near_field_stop"] is False
    assert config["run_startup_checks"] is False
    assert config["cloud_topic"] == TOPICS.map_cloud
    assert "SimEndpointDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "WavefrontFrontierExplorer" not in graph.modules
    assert "TraversableFrontierModule" not in graph.modules
    assert "MujocoDriverModule" not in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "SimEndpointDriverModule.odometry->nav.mission.odometry" in wires
    assert "SimEndpointDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "SimEndpointDriverModule.map_cloud->nav.terrain.map_cloud" in wires
    assert "nav.mission.global_path->nav.local_planner.global_path" in wires
    assert "nav.local_planner.local_path->nav.path_follower.local_path" in wires
    assert not graph.dangling_wires()


def test_product_explore_can_run_in_sim_mujoco_host_env():
    config = _resolve_selection_config(
        "explore",
        env="sim",
        env_config={"backend": "mujoco_host"},
    )
    graph = graph_for_product(
        "explore",
        env="sim",
        env_config={"backend": "mujoco_host"},
    )

    assert config["robot"] == "sim_endpoint"
    assert config["_env"] == "sim"
    assert config["_env_backend"] == "mujoco_host"
    assert config["_endpoint_transport"] == "local"
    assert config["planning_frame_id"] == "map"
    assert config["enable_frontier"] is True
    assert config["enable_traversable_frontier"] is True
    assert config["exploration_backend"] == "none"
    assert "SimEndpointDriverModule" in graph.modules
    assert "ROS2SimDriverModule" not in graph.modules
    assert "WavefrontFrontierExplorer" in graph.modules
    assert "TraversableFrontierModule" in graph.modules
    assert "nav.mission" in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert not graph.dangling_wires()


def test_sim_mujoco_host_launcher_is_env_implementation_data():
    implementation = load_runtime_graph().envs["sim"]["backends"]["mujoco_host"]

    assert implementation["process_control"] == "external_runner"
    assert implementation["runner"] == "sim/scripts/mujoco/launch_fastlio2_live.sh"
    assert implementation["supported_products"] == ["map", "explore"]


def test_rosbag_replay_profile_adapter_is_removed():
    from runtime.profiles.catalog.profile_adapters import PROFILE_ADAPTERS

    assert "replay" not in PROFILE_ADAPTERS
    assert "rosbag_fastlio2_replay" not in DATA_SOURCE_CONTRACTS
    assert "rosbag_fastlio2_replay" not in SIMULATION_RUNTIME_CONTRACTS


def test_real_product_config_carries_dds_communication_boundary():
    config = _resolve_selection_config("nav")

    assert config["_env"] == "real"
    assert config["_module_transport"] == "local"
    assert config["_endpoint_transport"] == "dds"
    assert config["_endpoint_contract"] == "thunder_dds_v1"
    assert config["hardware_control_boundary"] == "driver"


def test_explore_map_variant_can_run_in_sim_mujoco_host_env():
    config = _resolve_selection_config(
        "explore",
        env="sim",
        product_variant="map",
        env_config={"backend": "mujoco_host"},
    )

    assert config["robot"] == "sim_endpoint"
    assert config["_env"] == "sim"
    assert config["_env_backend"] == "mujoco_host"
    assert config["planner"] == "octoplanner3d"
    assert config["map_path"].endswith((".ot", ".bt"))
    assert config["map_artifact_gate_required"] is True
    assert "planner_backend" not in config
    assert config["planning_frame_id"] == "map"
    assert config["exploration_backend"] == "tare"
    assert config["enable_map_out"] is False
    assert "enable_nav_out" not in config
    assert config["enable_frontier"] is False
    assert "hold_active_goal_until_terminal" not in config


def test_explore_map_variant_rejects_retired_cmu_unity_sim_backend():
    with pytest.raises(ValueError, match="unsupported sim backend 'cmu_unity'"):
        _resolve_selection_config(
            "explore",
            env="sim",
            product_variant="map",
            env_config={"backend": "cmu_unity"},
        )

def test_sim_backend_rejects_unsupported_product_pairing():
    with pytest.raises(ValueError, match="does not support Product 'nav'"):
        _resolve_selection_config(
            "nav",
            env="sim",
            env_config={"backend": "gazebo"},
        )

def test_sim_profiles_keep_autonomy_inside_module_graph():
    for profile in ("sim", "sim_nav"):
        config = _resolve_selection_config(profile)
        nav_config = dict(config)
        enable_native = nav_config.pop("enable_native", False)
        nav_config.pop("planner", "octoplanner3d")
        nav_config.pop("tomogram", "")

        autonomy_config = autonomy_stack_config(enable_native, **nav_config)

        assert enable_native is False
        assert config["python_autonomy_backend"] == "nanobind"
        assert config["python_path_follower_backend"] == "nav_kernel"
        assert (autonomy_config["terrain_backend"] or autonomy_config["backend"]) == "nanobind"
        assert autonomy_config["backend"] == "nanobind"
        assert autonomy_config["path_follower_backend"] == "nav_kernel"


def test_real_robot_profiles_do_not_auto_actuate_on_startup():
    for profile, product_variant in (
        ("lite", None),
        ("map", None),
        ("nav", None),
        ("explore", "live"),
        ("explore", "map"),
    ):
        config = _resolve_selection_config(
            profile,
            product_variant=product_variant,
        )

        assert config["robot"] == "thunder"
        assert config["auto_enable"] is False
        assert config["auto_standup"] is False


def test_lite_profile_graph_stays_lightweight_and_python_only():
    config = _resolve_selection_config("lite")
    graph = graph_for_profile(
        "lite",
        mode="runtime",
        run_startup_checks=False,
    )
    modules = set(graph.modules)

    assert {
        "ThunderDriver",
        "nav.mission",
        "nav.terrain",
        "nav.local_planner",
        "nav.path_follower",
        "nav.safety",
        "nav.velocity_mux",
    } <= modules
    assert {
        "hw",
        "lidar",
        "SLAMModule",
        "SlamAdapterModule",
        "DepthVisualOdomModule",
        "OccupancyGridModule",
        "VoxelGridModule",
        "ESDFModule",
        "ElevationMapModule",
        "TraversabilityCostModule",
        "maps.service",
        "camera",
        "PerceptionModule",
        "ReconstructionModule",
        "SemanticPlannerModule",
        "LLMModule",
        "GatewayModule",
        "MCPServerModule",
        "TeleopModule",
        "RerunBridgeModule",
        "map.out",
        "nav.out",
    }.isdisjoint(modules)
    assert config["enable_hw"] is False
    assert not graph.dangling_wires()

    bp = blueprint_for_profile(
        "lite",
        run_startup_checks=False,
    )
    configs = {entry.name: entry.config for entry in bp._entries}
    assert configs["nav.mission"]["planner"] == "direct"
    assert configs["nav.mission"].get("tomogram", "") == ""
    assert configs["nav.terrain"]["backend"] == "simple"
    assert configs["nav.local_planner"]["backend"] == "simple"
    assert configs["nav.path_follower"]["backend"] == "pid"


def test_navigation_profiles_use_localization_odometry_for_runtime_consumers():
    for profile, product_variant in REAL_LOCALIZATION_SELECTIONS:
        source = "SlamAdapterModule"
        graph = _graph_for_selection(
            profile,
            product_variant=product_variant,
        )
        wires = _wire_set(graph)
        slam_topic = source in {"SlamModule", "SlamAdapterModule"}
        odom_suffix = f"@{TOPICS.odometry}" if slam_topic else ""
        map_suffix = f"@{TOPICS.map_cloud}" if slam_topic else ""
        observation_suffix = f"@{TOPICS.map_observation}" if slam_topic else ""
        health_suffix = f"@{TOPICS.localization_health}" if slam_topic else ""
        quality_suffix = f"@{TOPICS.localization_quality}" if slam_topic else ""
        has_mission_stack = "nav.mission" in graph.modules

        assert source in graph.modules
        assert f"{source}.odometry->GatewayModule.odometry{odom_suffix}" in wires
        if "nav.safety" in graph.modules:
            assert f"{source}.odometry->nav.safety.odometry{odom_suffix}" in wires
        else:
            assert f"{source}.odometry->nav.safety.odometry{odom_suffix}" not in wires
        for consumer in (
            "OccupancyGridModule",
            "VoxelGridModule",
            "ElevationMapModule",
        ):
            assert consumer not in graph.modules
            assert (
                f"{source}.map_observation->{consumer}.map_observation"
                f"{observation_suffix}"
            ) not in wires
            assert f"{source}.map_cloud->{consumer}.map_cloud{map_suffix}" not in wires
        assert f"{source}.map_cloud->GatewayModule.map_cloud{map_suffix}" in wires
        assert f"{source}.lidar_scan->GatewayModule.lidar_scan[local]@{TOPICS.lidar_scan}" in wires
        if "nav.safety" in graph.modules:
            assert f"{source}.localization_status->nav.safety.localization_status{health_suffix}" in wires
        else:
            assert f"{source}.localization_status->nav.safety.localization_status{health_suffix}" not in wires
        assert f"{source}.localization_status->GatewayModule.localization_status{health_suffix}" in wires
        assert f"{source}.localization_quality->GatewayModule.localization_quality{quality_suffix}" in wires

        if has_mission_stack:
            assert f"{source}.odometry->nav.mission.odometry{odom_suffix}" in wires
            assert f"{source}.localization_status->nav.mission.localization_status{health_suffix}" in wires
        if "nav.path_follower" in graph.modules:
            assert f"{source}.odometry->nav.path_follower.odometry{odom_suffix}" in wires
        if "nav.local_planner" in graph.modules:
            assert f"{source}.odometry->nav.local_planner.odometry{odom_suffix}" in wires
        if "nav.terrain" in graph.modules:
            assert f"{source}.map_cloud->nav.terrain.map_cloud{map_suffix}" in wires
        if "maps.service" in graph.modules:
            assert f"{source}.map_cloud->maps.service.map_cloud{map_suffix}" in wires
        else:
            assert f"{source}.map_cloud->maps.service.map_cloud{map_suffix}" not in wires

        assert "DepthVisualOdomModule" not in graph.modules
        assert (
            f"{source}.localization_status->DepthVisualOdomModule.localization_status"
            f"{health_suffix}"
        ) not in wires


def test_profile_groups_make_simulation_boundary_explicit():
    assert set(HOST_PROFILE_SNAPSHOT_NAMES) == set(SIMULATION_PROFILES)
    assert set(LOCAL_PROFILE_NAMES) == {"lite"}
    assert "lite" not in HOST_PROFILE_SNAPSHOT_NAMES
    assert "tare_explore" not in FIELD_PRODUCT_NAMES
    assert "portable_mujoco" in SIMULATION_PROFILES
    assert "sim_mujoco_live" in SIMULATION_PROFILES
    assert "sim_mujoco_octo_live" in SIMULATION_PROFILES


def test_field_product_graphs_do_not_include_legacy_native_modules():
    for profile in FIELD_PRODUCT_NAMES:
        graph = graph_for_product(profile, env="real")
        native_modules = [module_name for module_name in graph.modules if module_name.endswith("NativeModule")]
        assert native_modules == [], profile


def test_explore_map_variant_delegates_tare_policy_to_native_endpoint():
    graph = graph_for_product(
        "explore",
        env="real",
        product_variant="map",
    )
    wires = _wire_set(graph)
    product = resolve_product_variant_spec(
        "explore",
        load_runtime_graph().products["explore"],
        product_variant="map",
    )

    assert {module_name for module_name in graph.modules if module_name.endswith("NativeModule")} == set()
    assert "nav.commands" in graph.modules
    assert "WavefrontFrontierExplorer" not in graph.modules
    assert "TraversableFrontierModule" not in graph.modules
    assert "TAREPlannerNativeModule" not in graph.modules
    assert "TAREExplorerModule" not in graph.modules
    assert "ExplorationSupervisorModule" not in graph.modules
    assert "nav.terrain" not in graph.modules
    assert "nav.local_planner" not in graph.modules
    assert "nav.path_follower" not in graph.modules
    assert not any("TAREExplorerModule" in wire for wire in wires)
    assert product["autonomy_owner"] == "explore_endpoint"
    assert "explore" in product["processes"]


def test_tare_mujoco_host_backend_builds_the_product_host_contract():
    graph = graph_for_product(
        "explore",
        env="sim",
        product_variant="map",
        env_config={"backend": "mujoco_host"},
    )

    assert "nav.mission" in graph.modules
    assert "TAREExplorerModule" in graph.modules
    assert "ExplorationSupervisorModule" in graph.modules

def test_only_sanctioned_external_simulator_profiles_are_first_class():
    external_profiles = {
        profile
        for profile, config in HOST_PROFILE_DEFAULTS.items()
        if config.get("_external_launcher")
    }

    assert external_profiles == {
        "sim_mujoco_live",
        "sim_mujoco_octo_live",
    }


def test_profile_driver_defaults_resolve_through_driver_stack():
    from lingtu.assembly.stacks.driver import DriverBackend

    profile_backends = {
        config.get("_driver_backend", "stub")
        for config in HOST_PROFILE_DEFAULTS.values()
    }
    endpoint_backends = {
        profile_adapter(endpoint_name).driver_backend
        for endpoint_name in profile_adapter_names()
    }
    used_backends = profile_backends | endpoint_backends

    assert used_backends <= set(DRIVER_BACKENDS)
    assert set(DRIVER_BACKENDS) == set(DriverBackend.known_backends())


def test_products_do_not_enable_simulation_bypass_flags():
    forbidden_keys = {
        "allow_direct_goal_fallback",
        "direct_goal_fallback_on_planner_failure",
        "local_planner_allow_direct_track_fallback",
        "local_planner_ignore_near_field_stop",
        "safety_stop_wiring",
        "safety_cmd_vel_timeout_ms",
        "latch_stop_signal",
    }

    for product in FIELD_PRODUCT_NAMES:
        config = _resolve_selection_config(product)
        assert forbidden_keys.isdisjoint(config), product


def test_simulation_runtime_contracts_lock_simulator_boundary():
    gazebo = simulation_runtime_contract("gazebo_industrial")
    cmu_baseline = simulation_runtime_contract("cmu_unity_baseline")
    fastlio = simulation_runtime_contract("mujoco_fastlio2_live")

    assert gazebo.profile is None
    assert gazebo.world == "sim/worlds/gazebo/lingtu_gazebo_industrial_park.sdf"
    assert gazebo.launch_script == "sim/scripts/launch_lingtu_gazebo_industrial_demo.sh"
    assert gazebo.rviz_config == "sim/planning/lingtu_industrial_demo.rviz"
    assert gazebo.adapter_script == "sim/engine/bridge/gazebo_runtime_adapter.py"
    assert gazebo.data_source_contract == "gazebo_industrial"
    assert gazebo.command_topic == DATA_SOURCE_CONTRACTS["gazebo_industrial"].command_sink
    assert gazebo.canonical_topics == CANONICAL_NAV_TOPICS
    assert gazebo.native_topics == DATA_SOURCE_CONTRACTS["gazebo_industrial"].source_outputs
    assert set(DATA_SOURCE_CONTRACTS["gazebo_industrial"].algorithm_entry_outputs) <= set(
        gazebo.required_runtime_topics
    )
    assert gazebo.required_path_topics == (TOPICS.global_path, TOPICS.local_path)
    assert gazebo.required_map_growth_topics == (TOPICS.exploration_grid,)
    assert gazebo.required_scan_topics == (TOPICS.registered_cloud,)
    assert gazebo.simulation_only is True
    assert gazebo.isolated_domain_required is True
    assert "global_planning" in gazebo.lingtu_owns
    assert "local_planning" in gazebo.lingtu_owns
    assert "path_following" in gazebo.lingtu_owns
    assert "sensor_rendering" in gazebo.simulator_owns
    assert "global_planning" not in gazebo.simulator_owns
    assert gazebo.contract_role == "lingtu_full_stack_delivery_demo"
    assert gazebo.runtime_stage == "no_saved_map_live_mapping_smoke"
    assert gazebo.map_dependency == "gazebo_live_lidar_occupancy"
    assert gazebo.world_sensor_owner == "gazebo"
    assert gazebo.slam_source == DATA_SOURCE_CONTRACTS["gazebo_industrial"].slam_source
    assert gazebo.localization_source == DATA_SOURCE_CONTRACTS["gazebo_industrial"].localization_source
    assert gazebo.mapping_source == DATA_SOURCE_CONTRACTS["gazebo_industrial"].mapping_source
    assert gazebo.slam_validated is False
    assert gazebo.requires_live_slam is False
    assert gazebo.requires_saved_map is False
    assert gazebo.requires_tomogram is False
    assert gazebo.required_slam_topics == ()
    assert gazebo.cmd_vel_owner == "lingtu_cmd_vel_mux_to_gazebo_adapter"
    assert "real_robot_readiness" in gazebo.forbidden_claims
    assert "lingtu_fastlio_mapping_validated" in gazebo.forbidden_claims

    assert cmu_baseline.profile is None
    assert cmu_baseline.contract_role == "baseline_reference"
    assert cmu_baseline.runtime_stage == "external_baseline_no_lingtu_claim"
    assert cmu_baseline.map_dependency == "cmu_unity_live_registered_scan"
    assert cmu_baseline.launch_script.startswith("external:")
    assert cmu_baseline.adapter_script is None
    assert cmu_baseline.data_source_contract == "cmu_unity_external"
    assert cmu_baseline.command_topic == "/cmd_vel"
    assert cmu_baseline.exploration_owner == "cmu_tare_far"
    assert cmu_baseline.local_planning_owner == "cmu_local_planner"
    assert cmu_baseline.path_following_owner == "cmu_path_follower"
    assert cmu_baseline.cmd_vel_owner == "cmu_path_follower_to_vehicle_simulator"
    assert cmu_baseline.slam_source == "none"
    assert cmu_baseline.localization_source == "cmu_unity_state_estimation"
    assert cmu_baseline.mapping_source == "cmu_unity_registered_scan_and_terrain_map_ext"
    assert cmu_baseline.slam_validated is False
    assert cmu_baseline.requires_saved_map is False
    assert "/path" in cmu_baseline.required_path_topics
    assert "cmu_local_planning" in cmu_baseline.simulator_owns
    assert "lingtu_planning_validated" in cmu_baseline.forbidden_claims
    assert "lingtu_product" in cmu_baseline.forbidden_claims
    assert "lingtu_product" + "_profile" not in cmu_baseline.forbidden_claims
    assert "lingtu_slam_localization_validated" in cmu_baseline.forbidden_claims

    with pytest.raises(
        KeyError,
        match="unknown simulation runtime contract: cmu_unity_external",
    ):
        simulation_runtime_contract("cmu_unity_external")
    assert fastlio.provider == "mujoco"
    assert fastlio.profile == "sim_mujoco_live"
    assert fastlio.world == "sim/worlds/mujoco/industrial_park_scene.xml"
    assert fastlio.launch_script == "sim/scripts/mujoco/launch_fastlio2_live.sh"
    assert fastlio.data_source_contract == "mujoco_fastlio2_live"
    assert set(fastlio.canonical_topics) == set(DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"].source_outputs) | set(
        DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"].algorithm_entry_outputs
    )
    assert "/Odometry" in fastlio.native_topics
    assert "/cloud_map" in fastlio.native_topics
    assert "/Odometry" not in fastlio.canonical_topics
    assert "/cloud_map" not in fastlio.canonical_topics
    assert fastlio.runtime_stage == "no_saved_map_live_slam_optional_exploration"
    assert fastlio.map_dependency == "none_raw_lidar_imu"
    assert fastlio.required_slam_topics == (
        *DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"].source_outputs,
        "/Odometry",
        "/cloud_map",
    )
    assert fastlio.slam_source == DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"].slam_source
    assert fastlio.localization_source == DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"].localization_source
    assert fastlio.mapping_source == DATA_SOURCE_CONTRACTS["mujoco_fastlio2_live"].mapping_source
    assert fastlio.slam_validated is True
    assert fastlio.requires_live_slam is True
    assert fastlio.requires_saved_map is False
    assert fastlio.requires_tomogram is False
    assert fastlio.exploration_owner == "lingtu_frontier_optional"
    assert fastlio.cmd_vel_owner == "mujoco_velocity_adapter_or_fixed_gate_motion"
    assert "fastlio2_lidar_imu_mapping_localization" in fastlio.validated_claims
    assert "canonical_nav_output_relay" in fastlio.validated_claims
    assert "lingtu_navigation_validated" not in fastlio.forbidden_claims

    assert runtime_contracts_for_profile("sim_industrial") == ()
    assert runtime_contracts_for_profile("sim_cmu_tare") == ()
    assert runtime_contracts_for_profile("sim_mujoco_live") == (fastlio,)
    assert runtime_contracts_for_profile("sim_mujoco_octo_live") == (fastlio,)
    assert "gazebo_industrial" in SIMULATION_RUNTIME_CONTRACTS
    assert "cmu_unity_baseline" in SIMULATION_RUNTIME_CONTRACTS
    assert "cmu_unity_external" not in SIMULATION_RUNTIME_CONTRACTS
    assert "mujoco_fastlio2_live" in SIMULATION_RUNTIME_CONTRACTS
    assert "rosbag_fastlio2_replay" not in SIMULATION_RUNTIME_CONTRACTS
    assert "portable_fastlio2_replay" not in SIMULATION_RUNTIME_CONTRACTS

    for profile in HOST_PROFILE_DEFAULTS:
        binding = profile_data_source(profile)
        assert binding.profile == profile
        assert binding.data_source in DATA_SOURCE_CONTRACTS

    with pytest.raises(ValueError):
        profile_data_source("sim_industrial")
    with pytest.raises(ValueError):
        profile_data_source("sim_cmu_tare")
    assert profile_data_source("sim_mujoco_live").data_source == "mujoco_fastlio2_live"
    assert profile_data_source("sim_mujoco_octo_live").data_source == "mujoco_fastlio2_live"
    assert profile_data_source("sim").data_source == "mujoco_module_graph"
    assert profile_data_source("lite").data_source == "thunder_lite_local"
    assert product_data_source("map").data_source == "thunder"


def test_lingtu_simulation_runtime_contracts_mirror_data_source_boundaries():
    for contract in SIMULATION_RUNTIME_CONTRACTS.values():
        if contract.contract_role == "baseline_reference":
            continue
        source = DATA_SOURCE_CONTRACTS[contract.data_source_contract]
        required_runtime = set(contract.required_runtime_topics)

        assert set(source.algorithm_entry_outputs) <= required_runtime, contract.name
        assert set(source.algorithm_context_outputs) <= required_runtime, contract.name
        assert set(source.source_outputs) <= set(contract.native_topics), contract.name
        assert contract.slam_source == source.slam_source, contract.name
        assert contract.localization_source == source.localization_source, contract.name
        assert contract.mapping_source == source.mapping_source, contract.name
        if source.command_sink.startswith("/"):
            assert contract.command_topic == source.command_sink, contract.name


def test_real_robot_profiles_plan_in_client_map_frame():
    selections = (
        ("map", None),
        ("nav", None),
        ("explore", "live"),
        ("explore", "map"),
    )
    for profile, product_variant in selections:
        config = _resolve_selection_config(
            profile,
            product_variant=product_variant,
        )
        nav_config = dict(config)
        planner = nav_config.pop("planner", "octoplanner3d")
        nav_config.pop("tomogram", "")
        map_path = nav_config.pop("map_path", "")
        nav_config.pop("enable_native", False)
        nav_entry_config = navigation_config(
            planner,
            map_path,
            **nav_config,
        )

        assert nav_entry_config["planning_frame_id"] == "map"


def test_navigation_plan_safety_policy_is_profile_visible():
    sim_config = _resolve_selection_config("sim")
    sim_nav_config = dict(sim_config)
    sim_planner = sim_nav_config.pop("planner", "octoplanner3d")
    sim_nav_config.pop("tomogram", "")
    sim_map_path = sim_nav_config.pop("map_path", "")
    sim_nav_config.pop("planner_backend", None)
    sim_nav_config.pop("enable_native", False)
    sim_entry_config = navigation_config(
        sim_planner,
        sim_map_path,
        **sim_nav_config,
    )

    assert sim_entry_config["plan_safety_policy"] == "reject"
    assert sim_entry_config["fallback_planner_name"] == ""

    for profile, product_variant in (
        ("nav", None),
        ("explore", "live"),
        ("explore", "map"),
    ):
        config = _resolve_selection_config(
            profile,
            product_variant=product_variant,
        )
        nav_config = dict(config)
        planner = nav_config.pop("planner", "octoplanner3d")
        nav_config.pop("tomogram", "")
        map_path = nav_config.pop("map_path", "")
        nav_config.pop("planner_backend", None)
        nav_config.pop("enable_native", False)
        nav_entry_config = navigation_config(planner, map_path, **nav_config)

        assert nav_entry_config["plan_safety_policy"] == "reject"
        assert nav_entry_config["fallback_planner_name"] == ""


def test_navigation_forwards_frontier_reachability_tuning():
    frontier_config = frontier_module_config(
        enable_frontier=True,
        frontier_approach_max_target_distance_m=2.4,
        frontier_approach_goal_max_distance_m=6.0,
        frontier_reachable_goal_radius=1.6,
    )

    assert frontier_config["approach_max_target_distance_m"] == 2.4
    assert frontier_config["approach_goal_max_distance_m"] == 6.0
    assert frontier_config["reachable_goal_radius"] == 1.6


def test_navigation_forwards_path_follower_drive_mode():
    config = autonomy_stack_config(
        False,
        path_follower_two_way_drive=False,
        path_follower_native_max_accel=8.0,
    )

    assert config["path_follower_config"]["two_way_drive"] is False
    assert config["path_follower_config"]["native_max_accel"] == 8.0


def test_navigation_forwards_path_follower_safety_guard_config():
    config = autonomy_stack_config(
        False,
        path_follower_use_incl_rate_to_slow=True,
        path_follower_incl_rate_thre=90.0,
        path_follower_slow_rate_1=0.2,
        path_follower_slow_rate_2=0.4,
        path_follower_slow_rate_3=0.7,
        path_follower_slow_time_1=1.5,
        path_follower_slow_time_2=2.5,
        path_follower_use_incl_to_stop=True,
        path_follower_incl_thre=35.0,
        path_follower_stop_time=4.0,
    )

    assert config["path_follower_config"] == {
        "use_incl_rate_to_slow": True,
        "incl_rate_thre": 90.0,
        "slow_rate_1": 0.2,
        "slow_rate_2": 0.4,
        "slow_rate_3": 0.7,
        "slow_time_1": 1.5,
        "slow_time_2": 2.5,
        "use_incl_to_stop": True,
        "incl_thre": 35.0,
        "stop_time": 4.0,
    }


def test_navigation_forwards_independent_autonomy_backends():
    config = autonomy_stack_config(
        False,
        terrain_backend="simple",
        local_planner_backend="cmu_py",
        path_follower_backend="pid",
    )

    assert config["terrain_backend"] == "simple"
    assert config["backend"] == "cmu_py"
    assert config["path_follower_backend"] == "pid"


def test_autonomy_stack_config_matches_runtime_backend_policy_selection():
    runtime_config = {
        "terrain_backend": "simple",
        "python_autonomy_backend": "simple",
        "python_path_follower_backend": "pid",
        "path_follower_two_way_drive": False,
    }

    stack_config = autonomy_stack_config(False, **runtime_config)
    backend_selection = resolved_autonomy_backend_selection(
        runtime_config,
        enable_native=False,
    )

    assert stack_config["terrain_backend"] == backend_selection["terrain_backend"]
    assert stack_config["backend"] == backend_selection["local_planner_backend"]
    assert stack_config["path_follower_backend"] == backend_selection["path_follower_backend"]
    assert stack_config["path_follower_config"]["two_way_drive"] is False
