#!/usr/bin/env python3
"""Validate the Thunder field endpoint-only deployment contract."""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any

ROOT_DIR = Path(__file__).resolve().parents[2]
SRC_DIR = ROOT_DIR / "src"
SERVICE_PATH = ROOT_DIR / "scripts" / "deploy" / "thunder" / "lingtu-thunder-lcm-endpoint.service"
DEPLOY_PATH = ROOT_DIR / "scripts" / "deploy" / "deploy_thunder.sh"
RUNTIME_ENV_PATH = ROOT_DIR / "scripts" / "deploy" / "thunder" / "runtime-env.sh"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from compat.lcm.contracts import endpoint_contract  # noqa: E402
from compat.lcm.endpoint_runner import _expand_source_specs  # noqa: E402
from core.blueprints.profile_builder import blueprint_for_resolved_profile  # noqa: E402
from core.blueprints.runtime_endpoint import resolve_runtime_run_spec  # noqa: E402
from core.runtime.resolver import resolve_runtime_config  # noqa: E402
from core.runtime_interface import THUNDER_FIELD_RUNTIME_CONTRACT, TOPICS  # noqa: E402
from lingtu_runtime.plugin_seed import install_builtin_plugin_catalog  # noqa: E402

install_builtin_plugin_catalog()

EXPECTED_PROFILE = "thunder-nav"
EXPECTED_CANONICAL_PROFILE = "nav"
EXPECTED_ENDPOINT = "thunder_field"
EXPECTED_CONTRACT = "thunder_field_lcm_v1"
EXPECTED_HARDWARE_BOUNDARY = "lcm_endpoint_source"
EXPECTED_COMMAND_MODE = "endpoint_only"
FIELD_PRODUCT_GRAPH_PROFILES = ("thunder-map", "thunder-nav", "thunder-explore")
FORBIDDEN_PRODUCT_MODULE_PREFIXES = ("compat.ros2",)

EXPECTED_SPEC = {
    "endpoint": EXPECTED_ENDPOINT,
    "data_source": THUNDER_FIELD_RUNTIME_CONTRACT,
    "runtime_contract": THUNDER_FIELD_RUNTIME_CONTRACT,
    "robot_preset": "thunder",
    "module_transport": "local",
    "endpoint_transport": "lcm",
    "endpoint_contract": EXPECTED_CONTRACT,
    "localization_adapter": "lcm_endpoint",
    "simulation_only": False,
    "command_sink": "hardware_driver_after_cmd_vel_mux",
}

EXPECTED_CONFIG = {
    "enable_robot_driver": False,
    "command_output_mode": EXPECTED_COMMAND_MODE,
    "hardware_control_boundary": EXPECTED_HARDWARE_BOUNDARY,
    "localization_adapter": "lcm_endpoint",
    "endpoint_ingress_adapter": "lcm_endpoint",
    "endpoint_egress_adapter": "lcm_endpoint",
    "enable_endpoint_command_bridge": True,
    "enable_endpoint_path_bridge": True,
}

EXPECTED_SERVICE_ENV = {
    "LINGTU_PROFILE": EXPECTED_PROFILE,
    "LINGTU_MODULE_TRANSPORT": "local",
    "LINGTU_ENDPOINT": EXPECTED_ENDPOINT,
    "LINGTU_ENDPOINT_TRANSPORT": "lcm",
    "LINGTU_ENDPOINT_CONTRACT": EXPECTED_CONTRACT,
    "LINGTU_ENDPOINT_SOURCES": "thunder_field",
    "LINGTU_ENABLE_ROBOT_DRIVER": "0",
    "LINGTU_COMMAND_OUTPUT_MODE": EXPECTED_COMMAND_MODE,
    "LINGTU_HARDWARE_CONTROL_BOUNDARY": EXPECTED_HARDWARE_BOUNDARY,
}

EXPECTED_ENTRY_CLASSES = {
    "SlamBridgeModule": "LCMLocalizationAdapterModule",
    "EndpointCommandBridgeModule": "LCMNavigationCommandBridgeModule",
    "EndpointPathBridgeModule": "LCMPathCommandBridgeModule",
}

REQUIRED_WIRES = {
    "EndpointCommandBridgeModule.goal_pose->NavigationModule.goal_pose",
    "EndpointCommandBridgeModule.cancel->NavigationModule.cancel",
    "EndpointCommandBridgeModule.instruction->NavigationModule.instruction",
    "NavigationModule.global_path->EndpointPathBridgeModule.global_path",
    "LocalPlannerModule.local_path->EndpointPathBridgeModule.local_path",
    "NavigationModule.waypoint->EndpointPathBridgeModule.waypoint",
    "CmdVelMux.driver_cmd_vel->EndpointPathBridgeModule.cmd_vel",
}

FORBIDDEN_WIRES = {
    "CmdVelMux.driver_cmd_vel->ThunderDriver.cmd_vel",
    "SafetyRingModule.stop_cmd->ThunderDriver.stop_signal",
}

EXPECTED_BINDING_DIRECTIONS = {
    TOPICS.lidar_scan: "endpoint_to_lingtu",
    TOPICS.imu: "endpoint_to_lingtu",
    TOPICS.odometry: "endpoint_to_lingtu",
    TOPICS.registered_cloud: "endpoint_to_lingtu",
    TOPICS.map_cloud: "endpoint_to_lingtu",
    TOPICS.goal_pose: "endpoint_to_lingtu",
    TOPICS.cancel: "endpoint_to_lingtu",
    TOPICS.semantic_instruction: "endpoint_to_lingtu",
    TOPICS.global_path: "lingtu_to_endpoint",
    TOPICS.local_path: "lingtu_to_endpoint",
    TOPICS.nav_way_point: "lingtu_to_endpoint",
    TOPICS.cmd_vel: "lingtu_to_endpoint",
}


def validate(profile: str = EXPECTED_PROFILE) -> dict[str, Any]:
    """Validate Thunder field runtime, endpoint, graph, and service contracts."""

    blockers: list[str] = []
    checked_graph_profiles: set[str] = set()
    checked_files = {
        "src/core/blueprints/catalog/endpoints.py",
        "src/core/blueprints/products/thunder.py",
        "src/compat/lcm/contracts.py",
        "src/compat/lcm/endpoint_runner.py",
    }

    resolved = resolve_runtime_config(
        profile,
        overrides={
            "run_startup_checks": False,
            "manage_external_services": False,
        },
    )
    config = dict(resolved.config)
    spec = resolve_runtime_run_spec(resolved.profile, config)

    _validate_runtime_layers(resolved, spec, config, blockers)
    _validate_endpoint_contract(blockers)
    _validate_blueprint_graph(
        resolved.profile,
        config,
        blockers,
        checked_graph_profiles,
        graph_label=profile,
    )
    _validate_core_field_product_graphs(profile, blockers, checked_graph_profiles)
    _validate_runtime_env_file(blockers, checked_files)
    _validate_service_file(blockers, checked_files)
    _validate_deploy_script(blockers, checked_files)

    return {
        "ok": not blockers,
        "profile": profile,
        "canonical_profile": resolved.profile,
        "endpoint": spec.endpoint,
        "contract": spec.endpoint_contract,
        "runtime_contract": spec.runtime_contract,
        "checked_graph_profiles": sorted(checked_graph_profiles),
        "blockers": blockers,
        "checked_files": sorted(checked_files),
    }


def _validate_runtime_layers(
    resolved: Any,
    spec: Any,
    config: Mapping[str, Any],
    blockers: list[str],
) -> None:
    if resolved.runtime_endpoint != EXPECTED_ENDPOINT:
        blockers.append(
            f"runtime endpoint expected {EXPECTED_ENDPOINT!r}, got {resolved.runtime_endpoint!r}"
        )

    expected_spec = {**EXPECTED_SPEC, "profile": resolved.profile}
    for field, expected in expected_spec.items():
        actual = getattr(spec, field)
        if actual != expected:
            blockers.append(f"runtime spec {field} expected {expected!r}, got {actual!r}")

    for field, expected in EXPECTED_CONFIG.items():
        actual = config.get(field)
        if actual != expected:
            blockers.append(f"profile config {field} expected {expected!r}, got {actual!r}")

    expected_env = {
        "LINGTU_PROFILE": resolved.profile,
        "LINGTU_ENDPOINT": EXPECTED_ENDPOINT,
        "LINGTU_DATA_SOURCE": THUNDER_FIELD_RUNTIME_CONTRACT,
        "LINGTU_RUNTIME_CONTRACT": THUNDER_FIELD_RUNTIME_CONTRACT,
        "LINGTU_MODULE_TRANSPORT": "local",
        "LINGTU_ENDPOINT_TRANSPORT": "lcm",
        "LINGTU_ENDPOINT_CONTRACT": EXPECTED_CONTRACT,
        "LINGTU_LOCALIZATION_ADAPTER": "lcm_endpoint",
        "LINGTU_ENABLE_ROBOT_DRIVER": "0",
        "LINGTU_COMMAND_OUTPUT_MODE": EXPECTED_COMMAND_MODE,
        "LINGTU_HARDWARE_CONTROL_BOUNDARY": EXPECTED_HARDWARE_BOUNDARY,
    }
    for key, expected in expected_env.items():
        actual = spec.env.get(key)
        if actual != expected:
            blockers.append(f"runtime env {key} expected {expected!r}, got {actual!r}")


def _validate_endpoint_contract(blockers: list[str]) -> None:
    contract = endpoint_contract(EXPECTED_CONTRACT)
    if contract.runtime_contract != THUNDER_FIELD_RUNTIME_CONTRACT:
        blockers.append(
            f"endpoint contract runtime expected {THUNDER_FIELD_RUNTIME_CONTRACT!r}, "
            f"got {contract.runtime_contract!r}"
        )
    if contract.transport != "lcm":
        blockers.append(f"endpoint contract transport expected 'lcm', got {contract.transport!r}")

    for topic, expected_direction in EXPECTED_BINDING_DIRECTIONS.items():
        try:
            binding = contract.binding_for_topic(topic)
        except KeyError:
            blockers.append(f"endpoint contract missing topic {topic}")
            continue
        if binding.direction != expected_direction:
            blockers.append(
                f"endpoint contract {topic} direction expected {expected_direction!r}, "
                f"got {binding.direction!r}"
            )
        if binding.payload_format != "lingtu.transport.json.v1":
            blockers.append(
                f"endpoint contract {topic} payload format must be "
                "lingtu.transport.json.v1"
            )


def _validate_core_field_product_graphs(
    requested_profile: str,
    blockers: list[str],
    checked_graph_profiles: set[str],
) -> None:
    for product_profile in FIELD_PRODUCT_GRAPH_PROFILES:
        if product_profile == requested_profile:
            continue
        resolved = resolve_runtime_config(
            product_profile,
            overrides={
                "run_startup_checks": False,
                "manage_external_services": False,
            },
        )
        _validate_blueprint_graph(
            resolved.profile,
            dict(resolved.config),
            blockers,
            checked_graph_profiles,
            graph_label=product_profile,
        )


def _validate_blueprint_graph(
    profile: str,
    config: Mapping[str, Any],
    blockers: list[str],
    checked_graph_profiles: set[str],
    *,
    graph_label: str,
) -> None:
    if graph_label in checked_graph_profiles:
        return
    checked_graph_profiles.add(graph_label)
    bp = blueprint_for_resolved_profile(profile, config)
    entry_classes = {
        entry.name: getattr(entry.module_cls, "__name__", str(entry.module_cls))
        for entry in bp._entries
    }
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }

    if "ThunderDriver" in entry_classes:
        blockers.append(
            f"{graph_label} graph must not include in-process ThunderDriver"
        )
    for entry in bp._entries:
        module_name = getattr(entry.module_cls, "__module__", "")
        if module_name.startswith(FORBIDDEN_PRODUCT_MODULE_PREFIXES):
            class_name = getattr(entry.module_cls, "__name__", str(entry.module_cls))
            blockers.append(
                f"{graph_label} graph contains ROS compatibility module "
                f"{entry.name} ({module_name}.{class_name})"
            )
    for module_name, expected_class in EXPECTED_ENTRY_CLASSES.items():
        actual = entry_classes.get(module_name)
        if actual != expected_class:
            blockers.append(
                f"{graph_label} graph entry {module_name} expected "
                f"{expected_class}, got {actual or '<missing>'}"
            )
    for wire in sorted(REQUIRED_WIRES):
        if wire not in wires:
            blockers.append(f"{graph_label} graph missing required wire: {wire}")
    for wire in sorted(FORBIDDEN_WIRES):
        if wire in wires:
            blockers.append(
                f"{graph_label} graph contains forbidden in-process driver wire: {wire}"
            )


def _validate_service_file(blockers: list[str], checked_files: set[str]) -> None:
    rel_path = "scripts/deploy/thunder/lingtu-thunder-lcm-endpoint.service"
    checked_files.add(rel_path)
    if not SERVICE_PATH.is_file():
        blockers.append(f"{rel_path}: missing")
        return

    text = SERVICE_PATH.read_text(encoding="utf-8-sig")
    env = _parse_systemd_environment(text)
    for key, expected in EXPECTED_SERVICE_ENV.items():
        actual = env.get(key)
        if actual != expected:
            blockers.append(f"{rel_path}: {key} expected {expected!r}, got {actual!r}")

    source_value = str(env.get("LINGTU_ENDPOINT_SOURCES") or "")
    sources = {item.strip() for item in source_value.split(",") if item.strip()}
    if "thunder_field" not in sources:
        blockers.append(f"{rel_path}: LINGTU_ENDPOINT_SOURCES must include thunder_field")
    expanded_sources = _expand_source_specs(sorted(sources))
    if "thunder_brainstem" not in expanded_sources:
        blockers.append(f"{rel_path}: thunder_field source group must include thunder_brainstem")
    if "ros2-env.sh" in text or "/opt/ros" in text:
        blockers.append(f"{rel_path}: must not source ROS in the Thunder LCM endpoint service")
    if "run_lcm_endpoint_service.py" not in text:
        blockers.append(f"{rel_path}: must execute run_lcm_endpoint_service.py")
    if "--contract" not in text or "--source" not in text:
        blockers.append(f"{rel_path}: must pass endpoint contract and sources to runner")


def _validate_runtime_env_file(blockers: list[str], checked_files: set[str]) -> None:
    rel_path = "scripts/deploy/thunder/runtime-env.sh"
    checked_files.add(rel_path)
    if not RUNTIME_ENV_PATH.is_file():
        blockers.append(f"{rel_path}: missing")
        return

    text = RUNTIME_ENV_PATH.read_text(encoding="utf-8-sig", errors="ignore")
    if "/opt/ros" in text or "ros2-env.sh" in text:
        blockers.append(f"{rel_path}: product runtime environment must not source ROS")
    defaults = _parse_shell_default_env(text)
    expected_defaults = {
        "LINGTU_MODULE_TRANSPORT": "local",
        "LINGTU_ENDPOINT_TRANSPORT": "local",
        "LINGTU_SIMULATION_ONLY": "0",
    }
    for key, expected in expected_defaults.items():
        actual = defaults.get(key)
        if actual != expected:
            blockers.append(f"{rel_path}: {key} default expected {expected!r}, got {actual!r}")


def _validate_deploy_script(blockers: list[str], checked_files: set[str]) -> None:
    rel_path = "scripts/deploy/deploy_thunder.sh"
    checked_files.add(rel_path)
    if not DEPLOY_PATH.is_file():
        blockers.append(f"{rel_path}: missing")
        return

    text = DEPLOY_PATH.read_text(encoding="utf-8-sig")
    required_markers = (
        "LINGTU_DEPLOY_PROFILE:-thunder-nav",
        "LINGTU_ENDPOINT:=thunder_field",
        "LINGTU_ENDPOINT_TRANSPORT:=lcm",
        "LINGTU_ENDPOINT_CONTRACT:=thunder_field_lcm_v1",
        "LINGTU_MODULE_TRANSPORT:=local",
        "SOURCE_ROS2=0",
        "ros2|sim_ros2|*-ros2|ros-compat|legacy",
    )
    for marker in required_markers:
        if marker not in text:
            blockers.append(f"{rel_path}: missing marker {marker!r}")
    if '[ -f "/opt/ros/humble/setup.bash" ]' in text:
        blockers.append(f"{rel_path}: must not source ROS by default")
    if "git reset --hard" in text:
        blockers.append(f"{rel_path}: must not perform destructive git reset")


def _parse_systemd_environment(text: str) -> dict[str, str]:
    env: dict[str, str] = {}
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line.startswith("Environment="):
            continue
        item = line.split("=", 1)[1].strip().strip('"')
        key, _, value = item.partition("=")
        if key:
            env[key] = value
    return env


def _parse_shell_default_env(text: str) -> dict[str, str]:
    defaults: dict[str, str] = {}
    pattern = re.compile(r':\s*"\$\{([A-Za-z_][A-Za-z0-9_]*):=([^}]*)\}"')
    for raw_line in text.splitlines():
        match = pattern.search(raw_line.strip())
        if match:
            defaults[match.group(1)] = match.group(2)
    return defaults


def main(argv: list[str] | None = None) -> int:
    """Run the Thunder field deployment validator CLI."""

    parser = argparse.ArgumentParser(description="Validate Thunder field deployment boundaries")
    parser.add_argument(
        "--profile",
        default=EXPECTED_PROFILE,
        help=(
            "Product profile graph to validate; the standalone endpoint "
            "service defaults are still checked against thunder-nav"
        ),
    )
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    args = parser.parse_args(argv)

    result = validate(profile=str(args.profile))
    if args.json:
        print(json.dumps(result, ensure_ascii=True, indent=2, sort_keys=True))
    elif result["ok"]:
        checked = ", ".join(result["checked_files"])
        print(f"Thunder field deployment check: OK ({checked})")
    else:
        print("Thunder field deployment check: FAIL")
        for blocker in result["blockers"]:
            print(f"  ERROR: {blocker}")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
