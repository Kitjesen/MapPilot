#!/usr/bin/env python3
"""Validate the Thunder native field deployment contract."""

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
SERVICE_PATH = ROOT_DIR / "scripts" / "deploy" / "thunder" / "lingtu-driver.service"
DRIVER_INSTALLER_PATH = ROOT_DIR / "scripts" / "deploy" / "thunder" / "install_driver_service.sh"
DEPLOY_PATH = ROOT_DIR / "scripts" / "deploy" / "deploy_thunder.sh"
RUNTIME_ENV_PATH = ROOT_DIR / "scripts" / "deploy" / "thunder" / "runtime-env.sh"
NATIVE_MOTION_CI_PATH = ROOT_DIR / ".github" / "workflows" / "native-motion-build.yml"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from lingtu.plugin_seed import install_builtin_plugin_catalog
from lingtu.assembly.profile_builder import blueprint_for_resolved_profile
from runtime.endpoints.dds.contracts import endpoint_contract
from runtime.profiles.endpoints import resolve_runtime_run_spec
from runtime.profiles.resolver import resolve_runtime_config
from runtime.runtime_interface import THUNDER_FIELD_RUNTIME_CONTRACT, TOPICS

install_builtin_plugin_catalog()

EXPECTED_PROFILE = "nav"
EXPECTED_CANONICAL_PROFILE = "nav"
EXPECTED_ENDPOINT = "thunder_field"
EXPECTED_CONTRACT = "thunder_field_dds_v1"
EXPECTED_HARDWARE_BOUNDARY = "driver"
EXPECTED_COMMAND_MODE = "endpoint_only"
FIELD_PRODUCT_GRAPH_PROFILES = ("map", "nav", "tare_explore")
FORBIDDEN_PRODUCT_MODULE_PREFIXES = ("runtime.adapters.ros2",)

EXPECTED_SPEC = {
    "endpoint": EXPECTED_ENDPOINT,
    "data_source": THUNDER_FIELD_RUNTIME_CONTRACT,
    "runtime_contract": THUNDER_FIELD_RUNTIME_CONTRACT,
    "robot_preset": "thunder",
    "module_transport": "local",
    "endpoint_transport": "dds",
    "endpoint_contract": EXPECTED_CONTRACT,
    "localization_adapter": "cpp_slam_status",
    "simulation_only": False,
    "command_sink": "driver",
}

EXPECTED_CONFIG = {
    "enable_robot_driver": False,
    "command_output_mode": EXPECTED_COMMAND_MODE,
    "hardware_control_boundary": EXPECTED_HARDWARE_BOUNDARY,
    "localization_adapter": "cpp_slam_status",
    "native_navigation_endpoint": "lingtu-nav-dds",
    "enable_map_out": False,
}

EXPECTED_SERVICE_ENV = {
    "LINGTU_DRIVER_BIN": "/opt/lingtu/current/build/driver/lingtu_driver",
    "LINGTU_DDS_DOMAIN_ID": "0",
    "LINGTU_DRIVER_CMD_TIMEOUT_MS": "200",
    "LINGTU_DRIVER_STATUS_FILE": "/dev/shm/lingtu/driver_status.json",
}

LEGACY_MOTION_SINK_UNITS = (
    "lingtu-thunder-dds-endpoint.service",
    "thunder-dds-endpoint.service",
    "dds-endpoint.service",
    "lingtu-thunder-lite.service",
)

EXPECTED_ENTRY_CLASSES = {
    "SlamAdapterModule": "CppSlamStatusAdapterModule",
}

REQUIRED_WIRES = {
    "SlamAdapterModule.odometry->GatewayModule.odometry",
    "SlamAdapterModule.map_cloud->maps.service.map_cloud",
    "SlamAdapterModule.localization_status->GatewayModule.localization_status",
}

FORBIDDEN_ENDPOINT_ONLY_ENTRIES = {"nav.velocity_mux"}

MISSION_WIRES = {
    "GatewayModule.goal_pose->nav.goals.goal_request",
    "GatewayModule.cancel->nav.goals.cancel_request",
    "nav.goals.goal_pose->nav.mission.goal_pose",
    "nav.goals.cancel->nav.mission.cancel",
    "nav.mission.global_path->GatewayModule.global_path",
}

TARE_WIRES = {
    "OccupancyGridModule.exploration_grid->TAREExplorerModule.exploration_grid",
    "SlamAdapterModule.odometry->TAREExplorerModule.odometry",
    "TAREExplorerModule.exploration_goal->nav.mission.goal_pose",
    "TAREExplorerModule.exploration_path->nav.mission.patrol_goals",
    "nav.mission.mission_status->TAREExplorerModule.navigation_status",
}

FORBIDDEN_WIRES = {
    "nav.velocity_mux.driver_cmd_vel->ThunderDriver.cmd_vel",
    "nav.safety.stop_cmd->ThunderDriver.stop_signal",
}

EXPECTED_BINDING_DIRECTIONS = {
    TOPICS.lidar_scan: "endpoint_to_lingtu",
    TOPICS.imu: "endpoint_to_lingtu",
    TOPICS.odometry: "endpoint_to_lingtu",
    TOPICS.registered_cloud: "endpoint_to_lingtu",
    TOPICS.map_cloud: "endpoint_to_lingtu",
    TOPICS.goal_pose: "lingtu_to_endpoint",
    TOPICS.cancel: "lingtu_to_endpoint",
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
        "src/runtime/profiles/catalog/endpoints.py",
        "src/lingtu/assembly/products/thunder.py",
        "src/runtime/endpoints/dds/contracts.py",
        "src/runtime/endpoints/dds/endpoint_runner.py",
        "src/runtime/endpoints/dds/endpoint_service.py",
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
    _validate_driver_installer(blockers, checked_files)
    _validate_deploy_script(blockers, checked_files)
    _validate_native_motion_ci(blockers, checked_files)

    return {
        "ok": not blockers,
        "profile": profile,
        "canonical_profile": resolved.profile,
        "endpoint": spec.endpoint,
        "contract": EXPECTED_CONTRACT,
        "runtime_endpoint_contract": spec.endpoint_contract,
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
        blockers.append(f"runtime endpoint expected {EXPECTED_ENDPOINT!r}, got {resolved.runtime_endpoint!r}")

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
        "LINGTU_ENDPOINT_TRANSPORT": "dds",
        "LINGTU_LOCALIZATION_ADAPTER": "cpp_slam_status",
        "LINGTU_ENABLE_ROBOT_DRIVER": "0",
        "LINGTU_COMMAND_OUTPUT_MODE": EXPECTED_COMMAND_MODE,
        "LINGTU_HARDWARE_CONTROL_BOUNDARY": EXPECTED_HARDWARE_BOUNDARY,
    }
    for key, expected in expected_env.items():
        actual = spec.env.get(key)
        if actual != expected:
            blockers.append(f"runtime env {key} expected {expected!r}, got {actual!r}")
    if spec.env.get("LINGTU_ENDPOINT_CONTRACT") != EXPECTED_CONTRACT:
        blockers.append(f"runtime env LINGTU_ENDPOINT_CONTRACT expected {EXPECTED_CONTRACT!r}")


def _validate_endpoint_contract(blockers: list[str]) -> None:
    contract = endpoint_contract(EXPECTED_CONTRACT)
    if contract.runtime_contract != THUNDER_FIELD_RUNTIME_CONTRACT:
        blockers.append(
            f"endpoint contract runtime expected {THUNDER_FIELD_RUNTIME_CONTRACT!r}, got {contract.runtime_contract!r}"
        )
    if contract.transport != "dds":
        blockers.append(f"endpoint contract transport expected 'dds', got {contract.transport!r}")

    for topic, expected_direction in EXPECTED_BINDING_DIRECTIONS.items():
        try:
            binding = contract.binding_for_topic(topic)
        except KeyError:
            blockers.append(f"endpoint contract missing topic {topic}")
            continue
        if binding.direction != expected_direction:
            blockers.append(
                f"endpoint contract {topic} direction expected {expected_direction!r}, got {binding.direction!r}"
            )
        if binding.payload_format != "dds.idl.v1":
            blockers.append(f"endpoint contract {topic} payload format must be dds.idl.v1")
        if not binding.idl_type or not binding.cpp_type:
            blockers.append(f"endpoint contract {topic} must declare IDL and C++ message types")


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
    entry_classes = {entry.name: getattr(entry.module_cls, "__name__", str(entry.module_cls)) for entry in bp._entries}
    wires = {f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}" for wire in bp._wires}

    if "ThunderDriver" in entry_classes:
        blockers.append(f"{graph_label} graph must not include in-process ThunderDriver")
    if config.get("command_output_mode") == "endpoint_only":
        for entry_name in sorted(FORBIDDEN_ENDPOINT_ONLY_ENTRIES & entry_classes.keys()):
            blockers.append(f"{graph_label} graph contains endpoint-only Python control module {entry_name}")
    for entry in bp._entries:
        module_name = getattr(entry.module_cls, "__module__", "")
        if module_name.startswith(FORBIDDEN_PRODUCT_MODULE_PREFIXES):
            class_name = getattr(entry.module_cls, "__name__", str(entry.module_cls))
            blockers.append(
                f"{graph_label} graph contains ROS compatibility module {entry.name} ({module_name}.{class_name})"
            )
    for module_name, expected_class in EXPECTED_ENTRY_CLASSES.items():
        actual = entry_classes.get(module_name)
        if actual != expected_class:
            blockers.append(
                f"{graph_label} graph entry {module_name} expected {expected_class}, got {actual or '<missing>'}"
            )
    required_wires = set(REQUIRED_WIRES)
    if profile in {"nav", "tracking", "inspection", "tare_explore"}:
        required_wires |= MISSION_WIRES
    if profile == "tare_explore":
        required_wires |= TARE_WIRES
    for wire in sorted(required_wires):
        if wire not in wires:
            blockers.append(f"{graph_label} graph missing required wire: {wire}")
    for old_entry in ("nav.in", "nav.out", "SlamBridgeModule"):
        if old_entry in entry_classes:
            blockers.append(f"{graph_label} graph must not include old field entry {old_entry}")
    for wire in sorted(FORBIDDEN_WIRES):
        if wire in wires:
            blockers.append(f"{graph_label} graph contains forbidden in-process driver wire: {wire}")


def _validate_service_file(blockers: list[str], checked_files: set[str]) -> None:
    rel_path = "scripts/deploy/thunder/lingtu-driver.service"
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

    if "ros2-env.sh" in text or "/opt/ros" in text:
        blockers.append(f"{rel_path}: must not source ROS in the native driver service")
    if "python" in text.lower():
        blockers.append(f"{rel_path}: product driver must not execute Python")
    if "run_driver.sh" not in text:
        blockers.append(f"{rel_path}: must execute run_driver.sh")
    if "EnvironmentFile=/opt/lingtu/config/brainstem.env" not in text:
        blockers.append(
            f"{rel_path}: must require the remote Brainstem endpoint environment"
        )
    if "run_driver.sh --require-remote" not in text:
        blockers.append(f"{rel_path}: field driver must fail closed without a remote Brainstem host")
    if "run_status_file_watchdog.sh" not in text:
        blockers.append(f"{rel_path}: must bridge the driver status heartbeat to systemd")
    if "lingtu-nav-dds.service" not in text:
        blockers.append(f"{rel_path}: must order after the native nav command producer")

    directives = _parse_systemd_directives(text)
    service_types = set(directives.get("Type", ()))
    after_units = set(directives.get("After", ()))
    required_units = set(directives.get("Requires", ()))
    wanted_units = set(directives.get("Wants", ()))
    conflict_units = set(directives.get("Conflicts", ()))
    if "notify" not in service_types:
        blockers.append(f"{rel_path}: status-heartbeat watchdog requires Type=notify")
    for local_brainstem_unit in ("brainstem.service", "robot-brainstem.service"):
        if local_brainstem_unit in after_units | required_units | wanted_units:
            blockers.append(
                f"{rel_path}: remote Brainstem must not be modeled as local unit {local_brainstem_unit}"
            )
    for unit in LEGACY_MOTION_SINK_UNITS:
        if unit not in conflict_units:
            blockers.append(f"{rel_path}: must conflict with legacy motion sink {unit}")
    if "WatchdogSec=" not in text:
        blockers.append(f"{rel_path}: must configure a systemd watchdog timeout")


def _validate_driver_installer(blockers: list[str], checked_files: set[str]) -> None:
    rel_path = "scripts/deploy/thunder/install_driver_service.sh"
    checked_files.add(rel_path)
    if not DRIVER_INSTALLER_PATH.is_file():
        blockers.append(f"{rel_path}: missing")
        return

    text = DRIVER_INSTALLER_PATH.read_text(encoding="utf-8-sig")
    if 'install_catalog_service.sh" driver' not in text:
        blockers.append(f"{rel_path}: must install the catalog driver service")
    if "systemctl disable --now" not in text:
        blockers.append(f"{rel_path}: must stop and disable legacy motion sinks")
    for unit in LEGACY_MOTION_SINK_UNITS:
        if f'"{unit}"' not in text:
            blockers.append(f"{rel_path}: must disable legacy motion sink {unit}")


def _validate_native_motion_ci(blockers: list[str], checked_files: set[str]) -> None:
    rel_path = ".github/workflows/native-motion-build.yml"
    checked_files.add(rel_path)
    if not NATIVE_MOTION_CI_PATH.is_file():
        blockers.append(f"{rel_path}: missing")
        return

    text = NATIVE_MOTION_CI_PATH.read_text(encoding="utf-8-sig")
    required_markers = (
        "scripts/build/build_driver.sh",
        "scripts/build/build_nav_endpoint.sh",
        "LINGTU_DRIVER_RUN_TESTS: '1'",
        "LINGTU_NAV_ENDPOINT_RUN_TESTS: '1'",
    )
    for marker in required_markers:
        if marker not in text:
            blockers.append(f"{rel_path}: missing marker {marker!r}")


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
        "LINGTU_DEPLOY_PROFILE:-nav",
        "LINGTU_ENDPOINT:=thunder_field",
        "LINGTU_ENDPOINT_TRANSPORT:=dds",
        "LINGTU_MODULE_TRANSPORT:=local",
        "SOURCE_ROS2=0",
        "ros2|sim_ros2|*-ros2|ros-compat|legacy",
        "scripts/build/build_driver.sh",
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


def _parse_systemd_directives(text: str) -> dict[str, tuple[str, ...]]:
    directives: dict[str, list[str]] = {}
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        directives.setdefault(key, []).extend(value.split())
    return {key: tuple(values) for key, values in directives.items()}


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
            "service defaults are checked against the canonical nav profile"
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
