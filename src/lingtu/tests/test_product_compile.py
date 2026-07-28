# ruff: noqa: S101

from __future__ import annotations

import json

import pytest

from lingtu.assembly.profile_builder import blueprint_from_manifest, compile_product
from lingtu.product import ProductManifest
from runtime.blueprint import Blueprint
from runtime.profiles.resolver import resolve_runtime_config


def test_field_product_compiles_module_graph_and_processes_together() -> None:
    resolved = resolve_runtime_config("nav")

    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    assert product.profile == "nav"
    assert product.endpoint == "thunder_field"
    assert product.process_control == "launcher"
    assert product.has_process("slam")
    assert product.has_process("maps")
    assert product.has_process("nav")
    assert product.has_process("driver")
    assert product.process("maps").target == "mapd.service"
    assert product.config["enable_map_layers"] is False
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
    assert product.blueprint is not None
    assert product.blueprint.required_module_names == product.critical_modules
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in product.blueprint._wires
    }
    assert "host.bus.navigation_goal_status->nav.goals.navigation_goal_status" in wires
    assert "nav.mission" not in product.critical_modules


def test_field_product_rejects_python_map_layers_beside_native_mapd() -> None:
    resolved = resolve_runtime_config(
        "nav",
        overrides={
            "enable_map_layers": True,
            "enable_map_out": True,
            "map_out_adapter": "dds_map_output",
        },
    )

    with pytest.raises(ValueError, match="product validation failed") as exc_info:
        compile_product(
            resolved.profile,
            resolved.config,
            endpoint=resolved.runtime_endpoint,
        )

    detail = str(exc_info.value)
    assert "mapd.service" in detail
    assert "real_profile_mapd_layers_enabled" in detail
    assert "real_profile_mapd_module_conflict" in detail
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

def test_local_profile_compiles_without_deployment_processes() -> None:
    resolved = resolve_runtime_config("sim_nav")

    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    assert product.profile == "sim_nav"
    assert product.processes == ()
    assert product.process_control == "module"
    assert "nav.mission" in product.modules


def test_teleop_avoid_uses_native_operator_motion_not_python_session_module() -> None:
    resolved = resolve_runtime_config("teleop_avoid")

    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    assert product.profile == "teleop_avoid"
    assert product.process_control == "launcher"
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


@pytest.mark.parametrize("profile", ["teleop", "teleop_avoid"])
def test_operator_motion_product_passes_compiled_control_boundary_to_gateway(
    monkeypatch,
    profile: str,
) -> None:
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    monkeypatch.delenv("LINGTU_HARDWARE_CONTROL_BOUNDARY", raising=False)
    monkeypatch.delenv("LINGTU_PRODUCT_PROFILE", raising=False)
    monkeypatch.delenv("LINGTU_PROFILE", raising=False)
    resolved = resolve_runtime_config(profile)

    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    assert product.blueprint is not None
    gateway = next(
        entry for entry in product.blueprint._entries if entry.name == "GatewayModule"
    )
    assert gateway.config["command_output_mode"] == "endpoint_only"
    assert gateway.config["hardware_control_boundary"] == "driver"
    assert gateway.config["product_profile"] == profile
    assert "_product_profile" not in product.config


@pytest.mark.parametrize(
    ("profile", "require_map_scene"),
    [("teleop", False), ("teleop_avoid", True)],
)
def test_product_topics_control_host_bus_map_scene_requirement(
    profile: str,
    require_map_scene: bool,
) -> None:
    resolved = resolve_runtime_config(profile)
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    assert product.blueprint is not None
    host_bus = next(
        entry for entry in product.blueprint._entries if entry.name == "host.bus"
    )
    assert host_bus.config["require_map_scene"] is require_map_scene


def test_map_field_product_manifest_contains_only_process_contract() -> None:
    resolved = resolve_runtime_config("map")
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )
    payload = product.manifest().as_dict()

    assert product.blueprint is None
    with pytest.raises(RuntimeError, match="process-only"):
        _ = product.modules
    assert payload["profile"] == "map"
    assert payload["endpoint"] == "thunder_field"
    assert [process["name"] for process in payload["processes"]] == [
        "lidar",
        "slam",
        "maps",
        "nav",
        "driver",
        "camera",
        "host",
    ]
    assert product.process("camera").target == "lingtu-camera-dds.service"
    fingerprint = payload["fingerprint"]
    assert isinstance(fingerprint, str)
    assert len(fingerprint) == 64
    assert fingerprint == fingerprint.lower()
    assert all(character in "0123456789abcdef" for character in fingerprint)

    forbidden_keys = {
        "blueprint",
        "modules",
        "critical_modules",
        "module_transport",
        "host_config",
        "wires",
    }

    def collect_keys(value: object) -> set[str]:
        if isinstance(value, dict):
            mapping_keys = set(value)
            for nested_value in value.values():
                mapping_keys.update(collect_keys(nested_value))
            return mapping_keys
        if isinstance(value, list):
            sequence_keys: set[str] = set()
            for nested_value in value:
                sequence_keys.update(collect_keys(nested_value))
            return sequence_keys
        return set()

    present_forbidden_keys = forbidden_keys.intersection(collect_keys(payload))
    assert not present_forbidden_keys


@pytest.mark.parametrize("flag", ["enable_gateway", "enable_teleop", "enable_camera"])
def test_map_v5_rejects_disabling_required_control_plane_capability(flag: str) -> None:
    resolved = resolve_runtime_config("map", overrides={flag: False})

    with pytest.raises(ValueError, match=f"requires {flag}=true"):
        compile_product(
            resolved.profile,
            resolved.config,
            endpoint=resolved.runtime_endpoint,
        )


def test_map_v5_manifest_fingerprint_covers_host_process_config_and_rejects_tamper(
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
        "data_source": "thunder_field",
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
        "manage_session_services": False,
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
    resolved = resolve_runtime_config("map", overrides=operational_overrides)
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )
    repeated = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )
    payload = product.manifest().as_dict()

    assert payload["schema_version"] == "lingtu.product.v5"
    assert payload["fingerprint"] == repeated.fingerprint
    host_process = next(
        process for process in payload["processes"] if process["name"] == "host"
    )
    assert host_process["application"] == "map_control_plane"
    host_config = host_process["config"]
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
            "manage_session_services",
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
    assert host_config == expected_host_config
    staging_keys = (
        "_endpoint_transport",
        "_endpoint_contract",
        "command_output_mode",
        "hardware_control_boundary",
    )
    assert {key: host_config[key] for key in staging_keys} == {
        key: resolved.config[key] for key in staging_keys
    }
    assert all(
        value is None or isinstance(value, str | int | float | bool)
        for value in host_config.values()
    )
    assert all(
        "module" not in key.lower()
        and "wire" not in key.lower()
        and key != "host_config"
        for key in host_config
    )
    assert all(
        not (
            isinstance(value, str)
            and (value.endswith("Module") or value.endswith(".service"))
        )
        for value in host_config.values()
    )

    path = product.manifest().write(tmp_path / "map-product.json")
    loaded = ProductManifest.load(path)
    assert loaded.fingerprint == product.fingerprint
    assert loaded.process("host").application == "map_control_plane"
    assert dict(loaded.process("host").config) == host_config
    with pytest.raises(TypeError):
        loaded.process("host").config["_endpoint_transport"] = "shm"  # type: ignore[index]

    tampered = json.loads(path.read_text(encoding="utf-8"))
    tampered_host = next(
        process for process in tampered["processes"] if process["name"] == "host"
    )
    tampered_host["config"]["_endpoint_transport"] = "shm"
    path.write_text(json.dumps(tampered), encoding="utf-8")

    with pytest.raises(ValueError, match="fingerprint mismatch"):
        ProductManifest.load(path)


def test_product_contract_is_serializable_without_starting_runtime() -> None:
    resolved = resolve_runtime_config("nav")
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    payload = product.as_dict()

    assert payload["schema_version"] == "lingtu.product.v4"
    assert payload["endpoint"] == "thunder_field"
    assert [process["name"] for process in payload["processes"]] == [
        "lidar",
        "slam",
        "maps",
        "traversability",
        "nav",
        "driver",
        "host",
    ]
    assert len(payload["fingerprint"]) == 64
    assert payload["known_processes"]
    assert not hasattr(product, "plan")
    assert payload["route_contract"] == "robot"
    assert payload["module_transport"] == "local"
    assert payload["host_config"]["_runtime_endpoint"] == "thunder_field"
    assert payload["critical_modules"] == list(product.critical_modules)
    assert payload["required_capabilities"] == list(product.required_capabilities)
    assert "nav.mission" not in payload["critical_modules"]


def test_product_compile_defers_startup_preflight(monkeypatch) -> None:
    import lingtu.assembly.products.thunder as thunder

    calls: list[str] = []
    monkeypatch.setattr(
        thunder,
        "_run_startup_preflight",
        lambda **_: calls.append("preflight"),
    )
    resolved = resolve_runtime_config("nav")

    compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    assert calls == []


def test_blueprint_runs_deferred_checks_only_when_building() -> None:
    calls: list[str] = []
    blueprint = Blueprint().before_build(lambda: calls.append("preflight"))

    assert calls == []
    blueprint.build()

    assert calls == ["preflight"]


def test_product_manifest_round_trip_and_tamper_rejection(tmp_path) -> None:
    resolved = resolve_runtime_config("nav")
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )
    path = product.manifest().write(tmp_path / "product.json")

    loaded = ProductManifest.load(path)

    assert loaded.fingerprint == product.fingerprint
    assert loaded.process("host").target == "lingtu.service"
    assert loaded.required_capabilities == product.required_capabilities
    assert json.dumps(loaded.host_config, sort_keys=True) == json.dumps(
        product.config,
        sort_keys=True,
    )

    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["required_capabilities"].append("forged_runtime_capability")
    path.write_text(json.dumps(payload), encoding="utf-8")

    try:
        ProductManifest.load(path)
    except ValueError as exc:
        assert "fingerprint mismatch" in str(exc)
    else:
        raise AssertionError("edited Product capability contract must fail closed")


def test_host_uses_manifest_without_recompiling_product(monkeypatch, tmp_path) -> None:
    import cli.main as main_mod

    resolved = resolve_runtime_config("nav")
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )
    manifest_path = product.manifest().write(tmp_path / "product.json")
    sentinel = object()
    monkeypatch.setenv("LINGTU_PRODUCT_MANIFEST", str(manifest_path))
    monkeypatch.setenv("LINGTU_PRODUCT_FINGERPRINT", product.fingerprint)
    monkeypatch.setenv("LINGTU_SYSTEMD_UNIT", "lingtu.service")
    monkeypatch.setattr(ProductManifest, "build", lambda _manifest: sentinel)

    system = main_mod._build_host_system(product.profile, dict(product.config))

    assert system is sentinel


def test_manifest_materializes_the_declared_host_blueprint() -> None:
    resolved = resolve_runtime_config("nav")
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )

    blueprint = blueprint_from_manifest(product.manifest())

    assert blueprint.module_names == product.modules
    assert blueprint.required_module_names == product.critical_modules


def test_host_rejects_config_different_from_manifest(monkeypatch, tmp_path) -> None:
    import cli.main as main_mod

    resolved = resolve_runtime_config("nav")
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )
    manifest_path = product.manifest().write(tmp_path / "product.json")
    changed = dict(product.config)
    changed["gateway_port"] = int(changed["gateway_port"]) + 1
    monkeypatch.setenv("LINGTU_PRODUCT_MANIFEST", str(manifest_path))
    monkeypatch.setenv("LINGTU_PRODUCT_FINGERPRINT", product.fingerprint)

    with pytest.raises(RuntimeError, match="Host configuration does not match"):
        main_mod._build_host_system(product.profile, changed)
