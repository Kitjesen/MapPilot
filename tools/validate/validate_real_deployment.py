#!/usr/bin/env python3
"""Validate the native deployment contract for ``env=real``."""

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
SERVICE_PATH = ROOT_DIR / "scripts" / "deploy" / "thunder" / "lt-driver.service"
DRIVER_INSTALLER_PATH = ROOT_DIR / "scripts" / "deploy" / "thunder" / "install_driver_service.sh"
DEPLOY_PATH = ROOT_DIR / "scripts" / "deploy" / "deploy_robot.sh"
RUNTIME_ENV_PATH = ROOT_DIR / "scripts" / "deploy" / "thunder" / "runtime-env.sh"
NATIVE_MOTION_CI_PATH = ROOT_DIR / ".github" / "workflows" / "native-motion-build.yml"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from lingtu.assembly.compiler import blueprint_for_resolved_product  # noqa: E402
from lingtu.assembly.plugins import install_builtin_plugin_catalog  # noqa: E402
from lingtu.assembly.products import resolve_product_host_runtime  # noqa: E402
from lingtu.control import ProductControl  # noqa: E402
from runtime.endpoints.dds.contracts import endpoint_contract  # noqa: E402
from runtime.runtime_interface import (  # noqa: E402
    FIELD_DATA_SOURCE,
    REAL_RUNTIME_CONTRACT,
    TOPICS,
)

install_builtin_plugin_catalog()

EXPECTED_PRODUCT = "nav"
EXPECTED_CANONICAL_PRODUCT = "nav"
EXPECTED_CONTRACT = "field_dds_v1"
EXPECTED_HARDWARE_BOUNDARY = "driver"
EXPECTED_COMMAND_MODE = "endpoint_only"
FIELD_PRODUCT_GRAPH_SPECS = (
    ("map", None),
    ("nav", None),
    ("explore", "live"),
    ("explore", "map"),
)
FORBIDDEN_PRODUCT_MODULE_PREFIXES = ("runtime.adapters.ros2",)

EXPECTED_CONFIG = {
    "enable_robot_driver": False,
    "command_output_mode": EXPECTED_COMMAND_MODE,
    "hardware_control_boundary": EXPECTED_HARDWARE_BOUNDARY,
    "localization_adapter": "cpp_slam_status",
    "native_navigation_endpoint": "lingtu-nav-dds",
}

EXPECTED_SERVICE_ENV = {
    "LINGTU_DRIVER_BIN": "/opt/lingtu/current/bin/lingtu_driver",
    "LINGTU_DDS_DOMAIN_ID": "0",
    "LINGTU_DRIVER_STATUS_FILE": "/dev/shm/lingtu/driver_status.json",
}

LEGACY_MOTION_SINK_UNITS = (
    "lingtu-thunder-dds-endpoint.service",
    "thunder-dds-endpoint.service",
    "dds-endpoint.service",
)

EXPECTED_ENTRY_CLASSES = {
    "SlamAdapterModule": "CppSlamStatusAdapterModule",
}

REQUIRED_WIRES = {
    "SlamAdapterModule.odometry->GatewayModule.odometry",
    "SlamAdapterModule.localization_status->GatewayModule.localization_status",
}

NAV_PRODUCT_HOST_WIRES = {
    "host.bus.global_path->GatewayModule.global_path",
}

NAV_PRODUCT_HOST_ENTRIES = {
    "nav.commands": "Commands",
    "nav.goals": "GoalService",
}

FORBIDDEN_GATEWAY_COMMAND_WIRES = {
    "GatewayModule.goal_pose->nav.goals.goal_request",
    "GatewayModule.cancel->nav.goals.cancel_request",
}

EXPECTED_BINDING_DIRECTIONS = {
    TOPICS.lidar_scan: "endpoint_to_lingtu",
    TOPICS.imu: "endpoint_to_lingtu",
    TOPICS.odometry: "endpoint_to_lingtu",
    TOPICS.registered_cloud: "endpoint_to_lingtu",
    TOPICS.map_cloud: "endpoint_to_lingtu",
    TOPICS.nav_command_request: "lingtu_to_endpoint",
    TOPICS.nav_command_ack: "endpoint_to_lingtu",
    TOPICS.global_path: "lingtu_to_endpoint",
    TOPICS.local_path: "lingtu_to_endpoint",
    TOPICS.nav_way_point: "lingtu_to_endpoint",
    TOPICS.cmd_vel: "lingtu_to_endpoint",
}


def validate(product: str = EXPECTED_PRODUCT) -> dict[str, Any]:
    """Validate field runtime, endpoint, graph, and service contracts."""

    blockers: list[str] = []
    checked_graph_products: set[str] = set()
    checked_files = {
        "src/lingtu/control.py",
        "src/lingtu/run_plan.py",
        "src/lingtu/assembly/products/thunder.py",
        "src/runtime/endpoints/dds/contracts.py",
    }

    plan = ProductControl(env="real", process_env={})._resolve(product)
    config = dict(plan.host_config)

    _validate_runtime_layers(plan, config, blockers)
    _validate_endpoint_contract(blockers)
    _validate_blueprint_graph(
        plan.product,
        config,
        blockers,
        checked_graph_products,
        graph_label=product,
    )
    _validate_core_field_product_graphs(product, blockers, checked_graph_products)
    _validate_runtime_env_file(blockers, checked_files)
    _validate_service_file(blockers, checked_files)
    _validate_driver_installer(blockers, checked_files)
    _validate_deploy_script(blockers, checked_files)
    _validate_native_motion_ci(blockers, checked_files)

    return {
        "ok": not blockers,
        "product": product,
        "canonical_product": plan.product,
        "env": plan.env,
        "contract": EXPECTED_CONTRACT,
        "endpoint_contract": EXPECTED_CONTRACT,
        "runtime_contract": REAL_RUNTIME_CONTRACT,
        "run_plan_processes": [process.name for process in plan.processes],
        "checked_graph_products": sorted(checked_graph_products),
        "blockers": blockers,
        "checked_files": sorted(checked_files),
    }


def _validate_runtime_layers(
    plan: Any,
    config: Mapping[str, Any],
    blockers: list[str],
) -> None:
    if plan.env != "real":
        blockers.append(f"RunPlan Env expected 'real', got {plan.env!r}")
    if plan.process_control != "systemd":
        blockers.append(f"RunPlan controller expected 'systemd', got {plan.process_control!r}")
    if not plan.has_process("host"):
        blockers.append("RunPlan must declare the Host process")
    if plan.product in {"nav", "explore"} and not plan.has_process("nav"):
        blockers.append(f"RunPlan for {plan.product!r} must declare the native nav process")
    process_by_name = {
        process.name: process
        for process in getattr(plan, "processes", ())
    }
    nav_process = process_by_name.get("nav")
    driver_process = process_by_name.get("driver")
    if (
        nav_process is not None
        and driver_process is not None
        and nav_process.order >= driver_process.order
    ):
        blockers.append(
            "RunPlan must start the native nav command producer before the driver"
        )

    for field, expected in EXPECTED_CONFIG.items():
        actual = config.get(field)
        if actual != expected:
            blockers.append(f"RunPlan Host config {field} expected {expected!r}, got {actual!r}")
    if "robot" in config:
        blockers.append("RunPlan Host config must not carry the legacy robot plugin selector")

    native_env = plan.native_process_environment
    expected_native_env = {"LINGTU_PRODUCT": plan.product}
    for key, expected in expected_native_env.items():
        actual = native_env.get(key)
        if actual != expected:
            blockers.append(f"RunPlan native env {key} expected {expected!r}, got {actual!r}")
    required_native_keys = {
        "LINGTU_NAV_CONTROL_MODE",
        "LINGTU_NAV_PUBLISH_CMD_VEL",
        "LINGTU_NAV_CHECK_OBSTACLE",
        "LINGTU_NAV_USE_TRAVERSABILITY_COST",
        "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER",
        "LINGTU_TELEOP_LOCAL_PLANNER",
    }
    for key in sorted(required_native_keys):
        if key not in native_env:
            blockers.append(f"RunPlan native env missing {key}")
    for key in sorted(required_native_keys - {"LINGTU_NAV_CONTROL_MODE"}):
        value = native_env.get(key)
        if value not in {"0", "1"}:
            blockers.append(f"RunPlan native env {key} must be '0' or '1', got {value!r}")
    if plan.product in {"nav", "explore"} and native_env.get("LINGTU_NAV_CONTROL_MODE") != "autonomy":
        blockers.append(
            "RunPlan native env LINGTU_NAV_CONTROL_MODE expected 'autonomy' "
            f"for {plan.product!r}, got {native_env.get('LINGTU_NAV_CONTROL_MODE')!r}"
        )
def _validate_endpoint_contract(blockers: list[str]) -> None:
    contract = endpoint_contract(EXPECTED_CONTRACT)
    if contract.runtime_contract != REAL_RUNTIME_CONTRACT:
        blockers.append(
            f"endpoint contract runtime expected {REAL_RUNTIME_CONTRACT!r}, got {contract.runtime_contract!r}"
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
        if not binding.idl_type:
            blockers.append(f"endpoint contract {topic} must declare an IDL message type")


def _validate_core_field_product_graphs(
    requested_product: str,
    blockers: list[str],
    checked_graph_products: set[str],
) -> None:
    for product, product_variant in FIELD_PRODUCT_GRAPH_SPECS:
        if product == requested_product and product_variant is None:
            continue
        resolved = resolve_product_host_runtime(
            product,
            "real",
            product_variant=product_variant,
            overrides={
                "run_startup_checks": False,
            },
        )
        graph_label = (
            f"{product}:{product_variant}"
            if product_variant is not None
            else product
        )
        _validate_blueprint_graph(
            resolved.product,
            dict(resolved.config),
            blockers,
            checked_graph_products,
            graph_label=graph_label,
        )


def _validate_blueprint_graph(
    product: str,
    config: Mapping[str, Any],
    blockers: list[str],
    checked_graph_products: set[str],
    *,
    graph_label: str,
) -> None:
    if graph_label in checked_graph_products:
        return
    checked_graph_products.add(graph_label)
    bp = blueprint_for_resolved_product(product, config)
    entry_classes = {entry.name: getattr(entry.module_cls, "__name__", str(entry.module_cls)) for entry in bp._entries}
    wires = {f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}" for wire in bp._wires}

    if "ThunderDriver" in entry_classes:
        blockers.append(f"{graph_label} graph must not include in-process ThunderDriver")
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
    if product in {"nav", "tracking", "inspection", "explore"}:
        required_wires |= NAV_PRODUCT_HOST_WIRES
        for entry_name, expected_class in NAV_PRODUCT_HOST_ENTRIES.items():
            actual = entry_classes.get(entry_name)
            if actual != expected_class:
                blockers.append(
                    f"{graph_label} graph entry {entry_name} expected {expected_class}, "
                    f"got {actual or '<missing>'}"
                )
    for wire in sorted(required_wires):
        if wire not in wires:
            blockers.append(f"{graph_label} graph missing required wire: {wire}")
    for wire in sorted(FORBIDDEN_GATEWAY_COMMAND_WIRES & wires):
        blockers.append(f"{graph_label} graph contains retired Gateway command wire: {wire}")
    for old_entry in ("nav.in", "nav.out", "SlamBridgeModule"):
        if old_entry in entry_classes:
            blockers.append(f"{graph_label} graph must not include old field entry {old_entry}")


def _validate_service_file(blockers: list[str], checked_files: set[str]) -> None:
    rel_path = "scripts/deploy/thunder/lt-driver.service"
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
    if "EnvironmentFile=/run/lingtu/session.env" not in text:
        blockers.append(f"{rel_path}: must consume the Product session environment")
    if "require_product_session.sh driver" not in text:
        blockers.append(f"{rel_path}: must reject startup without the Product session")
    if "brainstem.env" in text:
        blockers.append(f"{rel_path}: must not hard-code a Thunder-only environment file")
    if "run_status_file_watchdog.sh" not in text:
        blockers.append(f"{rel_path}: must bridge the driver status heartbeat to systemd")
    directives = _parse_systemd_directives(text)
    service_types = set(directives.get("Type", ()))
    after_units = set(directives.get("After", ()))
    required_units = set(directives.get("Requires", ()))
    wanted_units = set(directives.get("Wants", ()))
    conflict_units = set(directives.get("Conflicts", ()))
    if "lt-nav.service" in after_units | required_units | wanted_units:
        blockers.append(
            f"{rel_path}: Product role ordering belongs to RunPlan/ProductControl, not systemd unit dependencies"
        )
    if "notify" not in service_types:
        blockers.append(f"{rel_path}: status-heartbeat watchdog requires Type=notify")
    for local_brainstem_unit in ("brainstem.service", "robot-brainstem.service"):
        if local_brainstem_unit in after_units | required_units | wanted_units:
            blockers.append(f"{rel_path}: remote Brainstem must not be modeled as local unit {local_brainstem_unit}")
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
        "LINGTU_ENV": "real",
        "LINGTU_DATA_SOURCE": FIELD_DATA_SOURCE,
        "LINGTU_RUNTIME_CONTRACT": REAL_RUNTIME_CONTRACT,
        "LINGTU_MODULE_TRANSPORT": "local",
        "LINGTU_ENDPOINT_TRANSPORT": "local",
        "LINGTU_SIMULATION_ONLY": "0",
    }
    for key, expected in expected_defaults.items():
        actual = defaults.get(key)
        if actual != expected:
            blockers.append(f"{rel_path}: {key} default expected {expected!r}, got {actual!r}")


def _validate_deploy_script(blockers: list[str], checked_files: set[str]) -> None:
    rel_path = "scripts/deploy/deploy_robot.sh"
    checked_files.add(rel_path)
    if not DEPLOY_PATH.is_file():
        blockers.append(f"{rel_path}: missing")
        return

    text = DEPLOY_PATH.read_text(encoding="utf-8-sig")
    required_markers = (
        "LINGTU_DEPLOY_PRODUCT",
        "from lingtu.assembly.deployment import product_native_build_scripts",
        "scripts/build/build_driver.sh",
        "scripts/deploy/thunder/install_services.sh",
        'scripts/lingtu" --robot "${ROBOT}" --env real switch',
        "LINGTU_DEPLOY_ROBOT",
        "LINGTU_DEPLOY_MAP",
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
    """Run the real-env deployment validator CLI."""

    parser = argparse.ArgumentParser(description="Validate env=real deployment boundaries")
    parser.add_argument(
        "--product",
        default=EXPECTED_PRODUCT,
        help=(
            "Real-env Product Host graph to validate; the standalone endpoint "
            "service defaults are checked against the canonical nav Product"
        ),
    )
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    args = parser.parse_args(argv)

    result = validate(product=str(args.product))
    if args.json:
        print(json.dumps(result, ensure_ascii=True, indent=2, sort_keys=True))
    elif result["ok"]:
        checked = ", ".join(result["checked_files"])
        print(f"Real deployment check: OK ({checked})")
    else:
        print("Real deployment check: FAIL")
        for blocker in result["blockers"]:
            print(f"  ERROR: {blocker}")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
