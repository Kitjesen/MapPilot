# ruff: noqa: S101

from __future__ import annotations

import json
from argparse import Namespace
from copy import deepcopy

import pytest

from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.assembly.profile_builder import (
    blueprint_for_resolved_product,
    blueprint_for_resolved_profile,
    blueprint_from_run_plan,
    compile_run_plan,
)
from lingtu.run_plan import RUN_PLAN_SCHEMA, RunPlan
from runtime.blueprint import Blueprint
from runtime.graph import RuntimeGraph, load_runtime_graph
from runtime.profiles.resolver import resolve_runtime_config

FIELD_PRODUCTS = tuple(sorted(load_runtime_graph().products))


def _compile_real(
    product: str,
    *,
    product_variant: str | None = None,
    overrides: dict | None = None,
    graph: RuntimeGraph | None = None,
) -> RunPlan:
    resolved = resolve_product_host_runtime(
        product,
        "real",
        product_variant=product_variant,
        overrides=overrides,
    )
    return compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
        product_variant=resolved.product_variant,
        graph=graph,
    )


@pytest.mark.parametrize("product", FIELD_PRODUCTS)
def test_every_field_product_declares_one_blueprint_host_contract(product: str) -> None:
    manifest = _compile_real(product)
    blueprint = blueprint_from_run_plan(manifest)

    assert manifest.process_control == "systemd"
    assert manifest.modules == blueprint.module_names
    assert manifest.critical_modules == blueprint.required_module_names
    assert manifest.schema_version == RUN_PLAN_SCHEMA
    assert manifest.has_process("host")


def test_field_product_compiles_module_graph_and_processes_together() -> None:
    product = _compile_real("nav")
    blueprint = blueprint_from_run_plan(product)

    assert product.product == "nav"
    assert product.env == "real"
    assert product.process_control == "systemd"
    assert product.has_process("slam")
    assert product.has_process("maps")
    assert product.has_process("nav")
    assert product.has_process("driver")
    assert product.host_config["enable_map_layers"] is False
    assert "maps.service" in product.modules
    assert not {
        "OccupancyGridModule",
        "VoxelGridModule",
        "SemanticMapModule",
        "ESDFModule",
        "ElevationMapModule",
        "TraversabilityCostModule",
        "map.out",
    }.intersection(product.modules)
    assert "/slam/odometry" in product.required_topics
    assert "/slam/map_observation" in product.required_topics
    assert "/nav/state" in product.required_topics
    assert "/nav/cmd_vel" in product.required_topics
    assert "final_cmd_vel_single_writer" in product.required_capabilities
    assert "DepthVisualOdomModule" not in product.modules
    assert "nav.mission" not in product.modules
    assert "nav.local_planner" not in product.modules
    assert "nav.path_follower" not in product.modules
    assert "host.bus" in product.modules
    assert "nav.commands" in product.modules
    assert "nav.goals" in product.modules
    assert product.critical_modules == (
        "host.bus",
        "SlamAdapterModule",
        "maps.service",
        "nav.commands",
        "nav.goals",
        "GatewayModule",
    )
    assert blueprint.required_module_names == product.critical_modules
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in blueprint._wires
    }
    assert "host.bus.navigation_goal_status->nav.goals.navigation_goal_status" in wires
    assert "nav.mission" not in product.critical_modules


def test_explore_product_launches_the_native_exploration_contract() -> None:
    product = _compile_real("explore")

    assert product.has_process("explore")
    assert {
        "/nav/exploration/command",
        "/nav/exploration/ack",
        "/nav/exploration_snapshot",
        "/nav/exploration_execution_snapshot",
        "/nav/exploration_segment/request",
        "/nav/exploration_segment/ack",
        "/nav/exploration_segment/status",
    } <= set(product.required_topics)


def test_explore_variants_compile_distinct_complete_run_plans(tmp_path) -> None:
    live = _compile_real("explore", product_variant="live")
    saved_map = _compile_real("explore", product_variant="map")

    assert live.product == saved_map.product == "explore"
    assert live.product_variant == "live"
    assert saved_map.product_variant == "map"
    assert live.contracts == ("lingtu.product.explore.v1",)
    assert saved_map.contracts == ("lingtu.product.explore.map.v1",)
    assert live.lifecycle["product_variant"] == "live"
    assert saved_map.lifecycle["product_variant"] == "map"
    assert live.lifecycle["slam_mode"] == "mapping"
    assert live.lifecycle["requires_map"] is False
    assert saved_map.lifecycle["slam_mode"] == "localization"
    assert saved_map.lifecycle["requires_map"] is True
    assert live.native_nav["allow_teleop_takeover"] is False
    assert saved_map.native_nav["allow_teleop_takeover"] is True
    assert live.fingerprint != saved_map.fingerprint

    loaded = RunPlan.load(saved_map.write(tmp_path / "explore-map.json"))

    assert loaded == saved_map
    assert loaded.as_dict()["identity"]["product_variant"] == "map"

    tampered = saved_map.as_dict()
    tampered["identity"]["product_variant"] = "live"
    with pytest.raises(ValueError, match="variant does not match"):
        RunPlan.from_dict(tampered)


def test_explore_compile_rejects_host_config_from_another_variant() -> None:
    resolved = resolve_product_host_runtime(
        "explore",
        "real",
        product_variant="map",
    )

    with pytest.raises(ValueError, match="Host config variant does not match"):
        compile_run_plan(
            resolved.product,
            resolved.env,
            resolved.config,
            product_variant="live",
        )


@pytest.mark.parametrize("mismatched_surface", ["host", "lifecycle"])
def test_run_plan_create_rejects_mismatched_variant_identity(
    mismatched_surface: str,
) -> None:
    plan = _compile_real("explore", product_variant="map")
    host_config = plan.host_config
    lifecycle = plan.lifecycle
    if mismatched_surface == "host":
        host_config["_product_variant"] = "live"
    else:
        lifecycle["product_variant"] = "live"

    with pytest.raises(ValueError, match="variant does not match"):
        RunPlan.create(
            product=plan.product,
            product_variant=plan.product_variant,
            env=plan.env,
            process_control=plan.process_control,
            modules=plan.modules,
            processes=plan.processes,
            available_processes=plan.available_processes,
            stop_targets=plan.stop_targets,
            contracts=plan.contracts,
            critical_modules=plan.critical_modules,
            route_contract=plan.route_contract,
            module_transport=plan.module_transport,
            host_config=host_config,
            lifecycle=lifecycle,
            compiled_against=plan.compiled_against,
            native_process_environment=plan.native_process_environment,
            parameter_profile=plan.parameter_profile,
            parameter_overrides=plan.parameter_overrides,
        )


def test_explore_compile_reads_parameter_profile_from_selected_variant() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    products["explore"]["variants"]["live"]["parameter_profile"] = (
        "rolling-explore-live"
    )
    products["explore"]["variants"]["map"]["parameter_profile"] = (
        "rolling-explore-map"
    )
    variant_graph = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=graph.envs,
    )
    resolved = resolve_product_host_runtime(
        "explore",
        "real",
        product_variant="map",
    )

    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
        product_variant=resolved.product_variant,
        graph=variant_graph,
    )

    assert plan.parameter_profile == "rolling-explore-map"


def test_field_product_rejects_python_layers_beside_native_maps_process() -> None:
    overrides = {
            "enable_map_layers": True,
            "enable_map_out": True,
            "map_out_adapter": "dds_map_output",
        }

    with pytest.raises(ValueError, match="Product validation failed") as exc_info:
        _compile_real("nav", overrides=overrides)

    detail = str(exc_info.value)
    assert "native maps process" in detail
    assert "native_maps_layers_enabled" in detail
    assert "native_maps_module_conflict" in detail
    for conflict in (
        "OccupancyGridModule",
        "VoxelGridModule",
        "SemanticMapModule",
        "ESDFModule",
        "ElevationMapModule",
        "TraversabilityCostModule",
        "map.out",
    ):
        assert conflict in detail


def test_native_maps_owner_validation_uses_process_role_not_systemd_target() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["real"]["processes"]["maps"]["target"] = (
        "renamed-native-maps.service"
    )
    renamed_target_graph = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )
    overrides = {
            "enable_map_layers": True,
            "enable_map_out": True,
            "map_out_adapter": "dds_map_output",
        }

    with pytest.raises(ValueError, match="Product validation failed") as exc_info:
        _compile_real(
            "nav",
            overrides=overrides,
            graph=renamed_target_graph,
        )

    assert "native_maps_layers_enabled" in str(exc_info.value)


def test_local_profile_does_not_compile_to_run_plan() -> None:
    resolved = resolve_runtime_config("sim_nav")

    with pytest.raises(ValueError, match="is not a Product"):
        compile_run_plan(resolved.profile, "sim", resolved.config, env_config={"backend": "mujoco_host"})


def test_sim_host_product_uses_its_env_host_graph_not_real_module_guards() -> None:
    resolved = resolve_product_host_runtime(
        "explore",
        "sim",
        env_config={"backend": "mujoco_host"},
    )

    manifest = compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
        env_config={"backend": "mujoco_host"},
    )

    assert manifest.env == "sim"
    assert manifest.process_control == "external_runner"
    assert manifest.critical_modules == ()
    assert "SimEndpointDriverModule" in manifest.modules
    assert "nav.mission" in manifest.modules


def test_resolved_blueprint_apis_reject_the_other_selection_kind() -> None:
    with pytest.raises(ValueError, match="is a Product"):
        blueprint_for_resolved_profile("nav", {})
    with pytest.raises(ValueError, match="Unknown Product"):
        blueprint_for_resolved_product("sim_nav", {})


def test_teleop_avoid_uses_native_operator_motion_not_python_session_module() -> None:
    product = _compile_real("teleop_avoid")

    assert product.product == "teleop_avoid"
    assert product.process_control == "systemd"
    assert product.native_nav["control_mode"] == "teleop_avoid"
    assert product.native_nav["teleop_local_planner"] is True
    assert product.native_nav["check_obstacle"] is True
    assert product.native_nav["use_traversability_cost"] is True
    assert "/nav/operator_motion/control" in product.required_topics
    assert "/nav/operator_motion/sample" in product.required_topics
    assert "/nav/operator_motion/ack" in product.required_topics
    assert "/nav/operator_motion/status" in product.required_topics
    assert "operator_motion_typed_dds_interface" in product.required_capabilities
    assert "native_operator_motion_authority" in product.required_capabilities
    assert "/nav/command/request" in product.required_topics
    assert "/nav/command/ack" in product.required_topics
    assert "operator.motion" not in product.modules
    assert "operator.motion" not in product.critical_modules
    assert "maps.service" not in product.modules
    assert "maps.service" not in product.critical_modules
    assert product.has_process("maps")


@pytest.mark.parametrize("product_name", ["teleop", "teleop_avoid"])
def test_operator_motion_product_passes_compiled_control_boundary_to_gateway(
    monkeypatch,
    product_name: str,
) -> None:
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    monkeypatch.delenv("LINGTU_HARDWARE_CONTROL_BOUNDARY", raising=False)
    monkeypatch.delenv("LINGTU_PRODUCT", raising=False)
    monkeypatch.delenv("LINGTU_PROFILE", raising=False)
    product = _compile_real(product_name)
    blueprint = blueprint_from_run_plan(product)

    gateway = next(
        entry for entry in blueprint._entries if entry.name == "GatewayModule"
    )
    assert gateway.config["command_output_mode"] == "endpoint_only"
    assert gateway.config["hardware_control_boundary"] == "driver"
    assert gateway.config["run_plan"] is product


@pytest.mark.parametrize(
    ("product_name", "require_map_scene"),
    [("teleop", False), ("teleop_avoid", True)],
)
def test_product_topics_control_host_bus_map_scene_requirement(
    product_name: str,
    require_map_scene: bool,
) -> None:
    product = _compile_real(product_name)
    blueprint = blueprint_from_run_plan(product)

    host_bus = next(
        entry for entry in blueprint._entries if entry.name == "host.bus"
    )
    assert host_bus.config["require_map_scene"] is require_map_scene


def test_map_run_plan_contains_host_blueprint_contract() -> None:
    product = _compile_real("map")
    blueprint = blueprint_from_run_plan(product)
    payload = product.as_dict()

    assert product.modules == blueprint.module_names
    assert product.critical_modules == blueprint.required_module_names
    assert set(payload) == {"identity", "launch", "host", "checks"}
    assert payload["identity"]["schema"] == RUN_PLAN_SCHEMA
    assert payload["identity"]["product"] == "map"
    assert payload["identity"]["env"] == "real"
    assert "endpoint" not in payload
    assert payload["host"]["expected_modules"] == list(product.modules)
    assert payload["checks"]["critical_modules"] == list(
        product.critical_modules
    )
    assert payload["host"]["config"] == json.loads(
        json.dumps(product.host_config)
    )
    assert [
        process["name"]
        for process in payload["launch"]["process_catalog"]["selected"]
    ] == [
        "lidar",
        "slam",
        "maps",
        "nav",
        "driver",
        "camera",
        "host",
    ]
    assert product.process("camera").target == "lingtu-camera-dds.service"
    fingerprint = payload["identity"]["fingerprint"]
    assert isinstance(fingerprint, str)
    assert len(fingerprint) == 64
    assert fingerprint == fingerprint.lower()
    assert all(character in "0123456789abcdef" for character in fingerprint)


def test_map_run_plan_fingerprint_covers_host_config_and_rejects_tamper(
    tmp_path,
) -> None:
    operational_overrides = {
        "slam_profile": "fastlio2",
        "localization_adapter": "cpp_slam_status",
        "map_dir": "/data/lingtu/maps",
        "data_dir": "/data/lingtu",
        "map_save_adapter": "native_slam_dds",
        "map_save_timeout_sec": 42.5,
        "source_profile": "map",
        "data_source": "thunder",
        "map_artifact_converter_command": "lingtu-octomap-build",
        "octomap_converter_command": "octomap-convert",
        "octomap_build_mode": "external_pcl_converter",
        "octomap_resolution": 0.18,
        "octomap_free_layers_above": 4,
        "octomap_free_dilation_cells": 2,
        "octomap_build_timeout_sec": 77.0,
        "build_octomap_on_save": True,
        "map_prune_command": "lingtu-prune",
        "dynamic_filter_command": "lingtu-dynamic-filter",
        "map_opt": "pgo",
        "map_optimization": "pgo",
        "map_opt_command": "lingtu-map-opt",
        "map_optimization_command": "lingtu-map-optimization",
        "map_opt_timeout_sec": 88.0,
        "map_opt_required": True,
        "semantic_taxonomy_path": "/opt/lingtu/config/semantic_taxonomy.json",
        "semantic_save_timeout_sec": 5.0,
        "octomap_editor_command": "lingtu-octomap-edit",
        "octomap_edit_timeout_sec": 12.0,
        "gateway_port": 5051,
        "mcp_port": 8091,
        "enable_gateway": True,
        "enable_teleop": True,
        "enable_camera": True,
        "camera_backend": "dds",
        "camera_jpeg_quality": 82,
        "camera_fps": 18,
        "startup_timeout_s": 33.0,
        "readiness_poll_interval_s": 0.75,
        "stop_timeout_s": 9.0,
        "runtime_failure_grace_s": 6.0,
        "native_navigation_endpoint": "lingtu-nav-dds",
        "planning_frame_id": "map",
        "planner": "direct",
        "planner_latency_budget_ms": 250,
        "command_output_mode": "endpoint_only",
        "hardware_control_boundary": "driver",
    }
    resolved = resolve_product_host_runtime(
        "map",
        "real",
        overrides=operational_overrides,
    )
    product = compile_run_plan(resolved.product, resolved.env, resolved.config)
    repeated = compile_run_plan(resolved.product, resolved.env, resolved.config)
    payload = product.as_dict()

    assert payload["identity"]["schema"] == RUN_PLAN_SCHEMA
    assert payload["identity"]["fingerprint"] == repeated.fingerprint
    host_config = payload["host"]["config"]
    expected_host_config = {
        key: resolved.config[key]
        for key in (
            "_endpoint_transport",
            "_endpoint_contract",
            "slam_profile",
            "localization_adapter",
            "map_dir",
            "data_dir",
            "map_save_adapter",
            "map_save_timeout_sec",
            "source_profile",
            "data_source",
            "map_artifact_converter_command",
            "octomap_converter_command",
            "octomap_build_mode",
            "octomap_resolution",
            "octomap_free_layers_above",
            "octomap_free_dilation_cells",
            "octomap_build_timeout_sec",
            "build_octomap_on_save",
            "map_prune_command",
            "dynamic_filter_command",
            "map_opt",
            "map_optimization",
            "map_opt_command",
            "map_optimization_command",
            "map_opt_timeout_sec",
            "map_opt_required",
            "semantic_taxonomy_path",
            "semantic_save_timeout_sec",
            "octomap_editor_command",
            "octomap_edit_timeout_sec",
            "gateway_port",
            "mcp_port",
            "enable_gateway",
            "enable_teleop",
            "enable_camera",
            "camera_backend",
            "camera_jpeg_quality",
            "camera_fps",
            "startup_timeout_s",
            "readiness_poll_interval_s",
            "stop_timeout_s",
            "runtime_failure_grace_s",
            "native_navigation_endpoint",
            "planning_frame_id",
            "planner",
            "planner_latency_budget_ms",
            "command_output_mode",
            "hardware_control_boundary",
        )
    }
    assert {key: host_config[key] for key in expected_host_config} == expected_host_config
    assert host_config == json.loads(json.dumps(product.host_config))
    staging_keys = (
        "_endpoint_transport",
        "_endpoint_contract",
        "command_output_mode",
        "hardware_control_boundary",
    )
    assert {key: host_config[key] for key in staging_keys} == {
        key: resolved.config[key] for key in staging_keys
    }
    path = product.write(tmp_path / "map-product.json")
    loaded = RunPlan.load(path)
    assert loaded.fingerprint == product.fingerprint
    assert dict(loaded.host_config) == host_config

    tampered = json.loads(path.read_text(encoding="utf-8"))
    tampered["host"]["config"]["_endpoint_transport"] = "shm"
    path.write_text(json.dumps(tampered), encoding="utf-8")

    with pytest.raises(ValueError, match="fingerprint mismatch"):
        RunPlan.load(path)


def test_product_contract_is_serializable_without_starting_runtime() -> None:
    product = _compile_real("nav")

    payload = product.as_dict()

    assert payload["identity"]["schema"] == RUN_PLAN_SCHEMA
    assert payload["identity"]["env"] == "real"
    assert "endpoint" not in payload
    assert [
        process["name"]
        for process in payload["launch"]["process_catalog"]["selected"]
    ] == [
        "lidar",
        "slam",
        "maps",
        "traversability",
        "nav",
        "driver",
        "host",
    ]
    assert len(payload["identity"]["fingerprint"]) == 64
    assert payload["launch"]["process_catalog"]["available"]
    assert not hasattr(product, "plan")
    assert payload["host"]["route_contract"] == "robot"
    assert payload["host"]["module_transport"] == "local"
    assert payload["host"]["config"]["_env"] == "real"
    assert "_profile_adapter" not in payload["host"]["config"]
    assert payload["checks"]["critical_modules"] == list(
        product.critical_modules
    )
    assert payload["checks"]["contracts"] == ["lingtu.product.nav.v1"]
    assert "required_capabilities" not in payload["checks"]
    assert "required_topics" not in payload["checks"]
    assert "nav.mission" not in payload["checks"]["critical_modules"]


def test_product_and_env_parameter_declarations_are_preserved_not_resolved() -> None:
    graph = load_runtime_graph()
    products = deepcopy(graph.products)
    envs = deepcopy(graph.envs)
    products["nav"]["parameter_profile"] = "field-nav-defaults"
    envs["real"]["parameter_overrides"] = {
        "future_parameter": {"value": 7, "unit": "cells"}
    }
    parameter_graph = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=products,
        envs=envs,
    )

    product = _compile_real("nav", graph=parameter_graph)
    payload = product.as_dict()

    assert product.parameter_profile == "field-nav-defaults"
    assert product.parameter_overrides == {
        "future_parameter": {"value": 7, "unit": "cells"}
    }
    assert payload["launch"]["parameter_profile"] == "field-nav-defaults"
    assert payload["launch"]["parameter_overrides"] == product.parameter_overrides
    assert set(payload["checks"]) == {"contracts", "critical_modules"}
    assert "future_parameter" not in payload["checks"]
    assert "future_parameter" not in payload["launch"][
        "native_process_environment"
    ]


def test_compiled_run_plan_returns_defensive_config_copies() -> None:
    product = _compile_real("nav")

    config_copy = product.host_config
    config_copy["gateway_port"] = 9999
    native_copy = product.native_nav
    native_copy["control_mode"] = "teleop"

    assert product.host_config["gateway_port"] != 9999
    assert product.native_nav["control_mode"] == "autonomy"


def test_product_compile_defers_startup_preflight(monkeypatch) -> None:
    import lingtu.assembly.products.thunder as thunder

    calls: list[str] = []
    monkeypatch.setattr(
        thunder,
        "_run_startup_preflight",
        lambda **_: calls.append("preflight"),
    )
    _compile_real("nav")

    assert calls == []


def test_blueprint_runs_deferred_checks_only_when_building() -> None:
    calls: list[str] = []
    blueprint = Blueprint().before_build(lambda: calls.append("preflight"))

    assert calls == []
    blueprint.build()

    assert calls == ["preflight"]


def test_run_plan_round_trip_and_tamper_rejection(tmp_path) -> None:
    product = _compile_real("nav")
    path = product.write(tmp_path / "product.json")

    loaded = RunPlan.load(path)

    assert loaded.fingerprint == product.fingerprint
    assert loaded.process("host").target == "lingtu.service"
    assert loaded.required_capabilities == product.required_capabilities
    assert json.dumps(dict(loaded.host_config), sort_keys=True) == json.dumps(
        product.host_config,
        sort_keys=True,
    )

    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["checks"]["contracts"] = ["lingtu.product.tracking.v1"]
    path.write_text(json.dumps(payload), encoding="utf-8")

    try:
        RunPlan.load(path)
    except ValueError as exc:
        assert "fingerprint mismatch" in str(exc)
    else:
        raise AssertionError("edited Product capability contract must fail closed")


def test_host_uses_run_plan_without_recompiling_product(monkeypatch, tmp_path) -> None:
    import cli.main as main_mod

    plan = _compile_real("nav")
    plan_path = plan.write(tmp_path / "run-plan.json")
    sentinel = object()
    monkeypatch.setenv("LINGTU_RUN_PLAN", str(plan_path))
    monkeypatch.setenv("LINGTU_RUN_PLAN_FINGERPRINT", plan.fingerprint)
    monkeypatch.setenv("LINGTU_SYSTEMD_UNIT", "lingtu.service")
    monkeypatch.setattr(RunPlan, "build", lambda _plan: sentinel)

    system = main_mod._build_host_system(plan.product, plan.host_config)

    assert system is sentinel


def test_systemd_host_requires_published_run_plan(monkeypatch) -> None:
    import cli.main as main_mod

    resolved = resolve_product_host_runtime("nav", "real")
    monkeypatch.setenv("LINGTU_SYSTEMD_UNIT", "lingtu.service")
    monkeypatch.delenv("LINGTU_RUN_PLAN", raising=False)
    monkeypatch.delenv("LINGTU_RUN_PLAN", raising=False)
    monkeypatch.delenv("LINGTU_RUN_PLAN_FINGERPRINT", raising=False)

    with pytest.raises(RuntimeError, match="LINGTU_RUN_PLAN"):
        main_mod._load_run_plan_for_host(resolved.product)


def test_systemd_host_requires_published_run_plan_fingerprint(
    monkeypatch,
    tmp_path,
) -> None:
    import cli.main as main_mod

    plan = _compile_real("nav")
    plan_path = plan.write(tmp_path / "run-plan.json")
    monkeypatch.setenv("LINGTU_SYSTEMD_UNIT", "lingtu.service")
    monkeypatch.setenv("LINGTU_RUN_PLAN", str(plan_path))
    monkeypatch.delenv("LINGTU_RUN_PLAN_FINGERPRINT", raising=False)

    with pytest.raises(RuntimeError, match="LINGTU_RUN_PLAN_FINGERPRINT"):
        main_mod._load_run_plan_for_host(plan.product)


def test_managed_host_uses_run_plan_config_before_profile_resolution(
    monkeypatch,
    tmp_path,
) -> None:
    import cli.main as main_mod

    plan = _compile_real("nav")
    plan_path = plan.write(tmp_path / "run-plan.json")
    monkeypatch.setenv("LINGTU_SYSTEMD_UNIT", "lingtu.service")
    monkeypatch.setenv("LINGTU_RUN_PLAN", str(plan_path))
    monkeypatch.setenv("LINGTU_RUN_PLAN_FINGERPRINT", plan.fingerprint)
    monkeypatch.setattr(
        main_mod,
        "_resolve_config",
        lambda *_args, **_kwargs: pytest.fail(
            "managed Host must not re-resolve Profile data"
        ),
    )

    loaded_plan, host_config = main_mod._resolve_host_startup(
        plan.product,
        Namespace(),
    )

    assert loaded_plan is not None
    assert loaded_plan.fingerprint == plan.fingerprint
    assert host_config == plan.host_config


def test_run_plan_materializes_the_declared_host_blueprint() -> None:
    plan = _compile_real("nav")
    blueprint = blueprint_from_run_plan(plan)

    assert blueprint.module_names == plan.modules
    assert blueprint.required_module_names == plan.critical_modules
    gateway_entry = next(
        entry for entry in blueprint._entries if entry.name == "GatewayModule"
    )
    goals_entry = next(
        entry for entry in blueprint._entries if entry.name == "nav.goals"
    )
    assert gateway_entry.config["run_plan"] is plan
    assert gateway_entry.config["run_plan"].fingerprint == plan.fingerprint
    assert gateway_entry.config["run_plan_fingerprint"] == plan.fingerprint
    assert goals_entry.config["run_plan_fingerprint"] == plan.fingerprint


def test_host_rejects_config_different_from_run_plan(monkeypatch, tmp_path) -> None:
    import cli.main as main_mod

    plan = _compile_real("nav")
    plan_path = plan.write(tmp_path / "run-plan.json")
    changed = dict(plan.host_config)
    changed["gateway_port"] = int(changed["gateway_port"]) + 1
    monkeypatch.setenv("LINGTU_RUN_PLAN", str(plan_path))
    monkeypatch.setenv("LINGTU_RUN_PLAN_FINGERPRINT", plan.fingerprint)

    with pytest.raises(RuntimeError, match="Host configuration does not match"):
        main_mod._build_host_system(plan.product, changed)
