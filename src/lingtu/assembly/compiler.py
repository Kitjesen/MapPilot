"""Compile Products into immutable RunPlans and Host blueprints."""

from __future__ import annotations

import re
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from lingtu.assembly.binding_policy import (
    endpoint_contract_for_config,
    endpoint_transport_for_config,
)
from lingtu.assembly.native_nav import compile_native_nav_config, mapd_environment
from lingtu.assembly.products.configuration import (
    EnvSpec,
    resolve_product_host_runtime,
)
from lingtu.assembly.simulation import compile_simulation_snapshot
from lingtu.products import product_name
from lingtu.run_plan import RunPlan
from runtime.blueprint import Blueprint
from runtime.contracts.product_runtime import resolve_product_spec_contracts
from runtime.graph import (
    ProcessSpec,
    load_runtime_graph,
    resolve_processes,
    resolve_stop_before_start,
    valid_process_name,
)
from runtime.graph.loader import RuntimeGraph

_CMU_PROFILES = {
    "doso/thunder_v4": "thunder",
    "unitree/go2": "go2",
}

_SIM_TRUTH_SLAM_CONFIG = "src/localization/fastlio2/config/sim_mid360.yaml"


def blueprint_for_resolved_product(
    product: str,
    config: Mapping[str, Any],
) -> Blueprint:
    """Return the Host Blueprint for an already resolved field Product."""

    product_name(product)

    from lingtu.assembly.products import host_blueprint

    return host_blueprint(config)


def compile_run_plan(
    product: str,
    env: str,
    *,
    robot: str | None = None,
    product_variant: str | None = None,
    local_planner: str | None = None,
    env_config: Mapping[str, Any] | None = None,
    overrides: Mapping[str, Any] | None = None,
    graph: RuntimeGraph | None = None,
) -> RunPlan:
    """Compile one Product without starting Modules or native processes."""

    try:
        resolved_product = product_name(product)
    except ValueError as exc:
        raise ValueError(f"{product!r} is not a Product") from exc
    graph = graph or load_runtime_graph()
    env_name = _env_name(env)
    resolved = resolve_product_host_runtime(
        resolved_product,
        env_name,
        robot=robot,
        product_variant=product_variant,
        local_planner=local_planner,
        env_config=env_config,
        overrides=overrides,
        graph=graph,
    )
    resolved_config = dict(resolved.config)
    product_spec = resolved.product_spec
    env_spec = resolved.env_spec
    implementation = env_spec.implementation
    process_control = str(
        implementation.get("process_control") or "module"
    ).strip()
    resolved_product_variant = resolved.product_variant
    runtime_contracts = resolve_product_spec_contracts(
        resolved_product,
        product_spec,
    )
    required_topics = runtime_contracts.topics

    validate_route_contract_for_resolved_config(resolved_config)
    route_contract = route_contract_name_for_resolved_config(resolved_config)

    processes: tuple[ProcessSpec, ...] = ()
    available_processes: tuple[ProcessSpec, ...] = ()
    process_conflicts: tuple[str, ...] = ()
    support_processes: tuple[str, ...] = ()
    if process_control in {"systemd", "subprocess"}:
        (
            processes,
            available_processes,
            process_conflicts,
            support_processes,
        ) = resolve_processes(
            resolved_product,
            env_name,
            graph=graph,
            env_config=(
                {"backend": env_spec.config.backend}
                if env_spec.config.backend is not None
                else None
            ),
            implementation=implementation,
            product_spec=product_spec,
        )
    required_capabilities = runtime_contracts.capabilities
    product_roles = _product_roles(product_spec, product=resolved_product)
    has_camera = "camera" in product_roles
    resolved_config["enable_camera"] = has_camera
    resolved_config["use_driver_camera"] = False
    if has_camera:
        resolved_config["camera_backend"] = "dds"
    else:
        resolved_config.pop("camera_backend", None)
    native_nav = product_spec.get("native_nav")
    if native_nav is None:
        native_nav = {}
    if not isinstance(native_nav, Mapping):
        raise ValueError(f"Product {resolved_product!r} native_nav must be a mapping")

    backend = env_spec.config.backend or ""
    simulation = compile_simulation_snapshot(
        env=env_name,
        process_control=process_control,
        backend=backend,
        robot=resolved.robot,
        implementation=implementation,
        repository_root=_REPO_ROOT,
        product=resolved_product,
        viewer=env_spec.config.viewer,
    )
    if env_name == "sim" and has_camera:
        resolved_config.setdefault("detector", "sim_scene")
        if resolved_config["detector"] == "sim_scene":
            world_mjcf = simulation["physics_plan"]["world"]["mjcf"]
            resolved_config.setdefault("world", Path(world_mjcf).name)
            scenario_plan = simulation.get("scenario_plan")
            if isinstance(scenario_plan, Mapping):
                resolved_config["scenario_entities"] = [
                    entity
                    for entity in scenario_plan.get("entities", ())
                    if isinstance(entity, Mapping)
                    and entity.get("entity_type") != "robot"
                ]
    # Critical Host modules are Product semantics. An Env may select concrete
    # processes, but it must not weaken the Product's startup barrier.
    critical_modules = _string_tuple(product_spec.get("critical_modules"))
    blueprint_config = dict(resolved_config)
    blueprint_config["_product_required_topics"] = required_topics
    blueprint_config["_product_required_capabilities"] = required_capabilities
    blueprint = blueprint_for_resolved_product(resolved_product, blueprint_config)
    if route_contract:
        blueprint.route_contract(route_contract)
    if critical_modules:
        blueprint.require_modules(*critical_modules)
    from lingtu.assembly.validation import validate_product

    issues = validate_product(
        resolved_product,
        product_spec=product_spec,
        implementation=implementation,
        config=resolved_config,
        module_names=blueprint.module_names,
        env_name=env_name,
        processes=processes,
    )
    if issues:
        detail = "; ".join(f"{issue.code}: {issue.message}" for issue in issues)
        raise ValueError(f"Product validation failed: {detail}")

    stop_before_start = resolve_stop_before_start(
        implementation,
        available_processes,
        process_conflicts,
        owner=f"env {env_name}",
    )
    lifecycle = _compile_lifecycle(resolved_product, product_spec)
    compiled_native_nav = compile_native_nav_config(
        resolved_product,
        {
            **resolved_config,
            "native_control_mode": lifecycle["native_control_mode"],
            "native_nav": dict(native_nav),
        },
    )
    native_nav_contract = dict(compiled_native_nav.native_nav)
    del native_nav_contract["octoplanner3d"]
    del native_nav_contract["recovery"]
    del native_nav_contract["smoothing"]
    native_process_environment = _native_process_environment(
        product=resolved_product,
        env_spec=env_spec,
        native_environment=compiled_native_nav.environment,
        lifecycle=lifecycle,
        roles=product_roles,
        process_control=process_control,
    )
    parameter_profile = _optional_parameter_profile(
        product_spec.get("parameter_profile"),
        product=resolved_product,
    )
    parameter_overrides = _parameter_overrides(
        env=env_name,
        implementation=implementation,
        env_spec=graph.envs.get(env_name),
    )
    return RunPlan.create(
        product=resolved_product,
        product_variant=resolved_product_variant,
        env=env_name,
        robot=resolved.robot,
        process_control=process_control,
        modules=blueprint.module_names,
        processes=processes,
        available_processes=available_processes,
        support_processes=support_processes,
        stop_before_start=stop_before_start,
        contracts=runtime_contracts.contract_ids,
        critical_modules=critical_modules,
        native_nav=native_nav_contract,
        native_process_environment=native_process_environment,
        route_contract=route_contract,
        host_config=resolved_config,
        lifecycle=lifecycle,
        parameter_profile=parameter_profile,
        parameter_overrides=parameter_overrides,
        simulation=simulation,
    )


def blueprint_from_run_plan(plan: RunPlan) -> Blueprint:
    """Recreate only the Host graph declared by a RunPlan."""

    config = dict(plan.host_config)
    config_env = _env_name(config.get("_env"))
    if config_env != plan.env:
        raise ValueError(
            f"RunPlan Host env mismatch: plan={plan.env!r} config={config_env!r}"
        )
    route_contract = route_contract_name_for_resolved_config(config)
    if route_contract != plan.route_contract:
        raise ValueError(
            "RunPlan Host route contract mismatch: "
            f"plan={plan.route_contract!r} config={route_contract!r}"
        )
    validate_route_contract_for_resolved_config(config)
    assembly_config = dict(config)
    assembly_config["_product_required_topics"] = plan.required_topics
    assembly_config["_product_required_capabilities"] = plan.required_capabilities
    # Hand the resolved execution record directly to the managed Host.
    assembly_config["_run_plan"] = plan
    blueprint = blueprint_for_resolved_product(plan.product, assembly_config)
    if plan.route_contract:
        blueprint.route_contract(plan.route_contract)
    if plan.critical_modules:
        blueprint.require_modules(*plan.critical_modules)
    if blueprint.module_names != plan.modules:
        raise ValueError(
            "RunPlan Host modules do not match the current assembly: "
            f"plan={plan.modules!r} assembly={blueprint.module_names!r}"
        )
    return blueprint


def build_host_from_run_plan(plan: RunPlan) -> Any:
    """Build a Host from immutable Product data without resolving processes."""

    blueprint = blueprint_from_run_plan(plan)
    return blueprint.build()


def route_contract_name_for_resolved_config(config: Mapping[str, Any]) -> str | None:
    """Return the external route contract selected by a resolved config.

    Route contracts are boundary metadata. They validate endpoint topics and
    adapter ownership, but they do not change ModulePort delivery unless a
    Blueprint route contracts remain metadata and never select Module delivery.
    """

    endpoint_transport = endpoint_transport_for_config(config, default="").lower()
    endpoint_contract = endpoint_contract_for_config(config)
    if endpoint_transport == "dds" and endpoint_contract == "field_dds_v1":
        return "robot"
    return None


def validate_route_contract_for_resolved_config(config: Mapping[str, Any]) -> None:
    """Fail fast when a resolved endpoint references an invalid route contract."""

    route_contract = route_contract_name_for_resolved_config(config)
    if not route_contract:
        return

    from runtime.route_contract import load_route_contract, validate_route_contract

    contract = load_route_contract(route_contract)
    issues = validate_route_contract(contract)
    if issues:
        detail = "; ".join(f"{issue.code}:{issue.scope}: {issue.message}" for issue in issues)
        raise ValueError(f"invalid route contract '{route_contract}': {detail}")


def _env_name(value: Any) -> str:
    env = str(value or "").strip()
    if env not in {"real", "sim"}:
        raise ValueError(f"env must be 'real' or 'sim', received {env!r}")
    return env


def _native_process_environment(
    *,
    product: str,
    env_spec: EnvSpec,
    native_environment: Mapping[str, str],
    lifecycle: Mapping[str, Any],
    roles: tuple[str, ...],
    process_control: str,
) -> dict[str, str]:
    environment = dict(native_environment)
    if "maps" in roles:
        environment["LINGTU_MAPD_EXTENDED_LAYERS"] = "1" if product == "map" else "0"
    if "traversability" in roles:
        environment["LINGTU_EXPLORE_ROUTE"] = (
            "live" if lifecycle["slam_mode"] == "mapping" else "map"
        )
    if "maps" in roles:
        environment.update(
            mapd_environment(environment.get("LINGTU_NAV_LOCAL_PLANNER_BACKEND"))
        )
    if env_spec.name == "real":
        robot_config = env_spec.robot_config
        if robot_config is None:
            raise RuntimeError("real Env resolved without RobotConfig")
        driver = robot_config.driver
        speed = robot_config.speed
        safety = robot_config.safety
        values = {
            "LINGTU_CONFIG_PATH": (
                env_spec.robot_config_ref.relative_to(_REPO_ROOT).as_posix()
                if env_spec.robot_config_ref is not None
                else None
            ),
            "LINGTU_DRIVER_BACKEND": driver.backend,
            "LINGTU_DRIVER_TARGET": driver.target,
            "LINGTU_DRIVER_NETWORK_INTERFACE": driver.network_interface,
            "LINGTU_DRIVER_NETWORK_ADDRESS": driver.network_address,
            "LINGTU_DRIVER_PROBE_IP": driver.probe_ip,
            "LINGTU_DDS_NETWORK_INTERFACE": driver.network_interface,
            "LINGTU_DRIVER_MAX_LINEAR_MPS": speed.max_linear,
            "LINGTU_DRIVER_MAX_ANGULAR_RPS": speed.max_angular,
            "LINGTU_DRIVER_CMD_TIMEOUT_MS": safety.cmd_vel_timeout_ms,
            "LINGTU_DRIVER_POLL_HZ": driver.control_rate,
            "LINGTU_DRIVER_RECONNECT_MS": driver.reconnect_interval * 1000.0,
            "LINGTU_DRIVER_TLS_CA_FILE": driver.tls_ca_file,
            "LINGTU_DRIVER_TLS_CERT_FILE": driver.tls_cert_file,
            "LINGTU_DRIVER_TLS_KEY_FILE": driver.tls_key_file,
            "LINGTU_DRIVER_TLS_SERVER_NAME": driver.tls_server_name,
        }
        if driver.backend == "go2":
            interface = driver.network_interface.strip()
            if re.fullmatch(r"[A-Za-z0-9_.:-]+", interface) is None:
                raise ValueError("Go2 driver network interface is invalid")
            values["CYCLONEDDS_URI"] = (
                "<CycloneDDS><Domain><General><NetworkInterfaceAddress>"
                f"{interface}"
                "</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
            )
        if "slam" in roles:
            values["LINGTU_SLAM_CONFIG"] = env_spec.slam_config_ref
        if "lidar" in roles:
            values.update(
                {
                    "LINGTU_LIVOX_LIDAR_IP": env_spec.lidar_ip,
                    "LINGTU_LIVOX_HOST_IP": env_spec.lidar_host_ip,
                    "LINGTU_LIVOX_NET_IFACE": env_spec.lidar_network_interface,
                }
            )
        environment.update(
            {key: str(value) for key, value in values.items() if value is not None}
        )
    elif "slam" in roles:
        environment["LINGTU_SLAM_CONFIG"] = _SIM_TRUTH_SLAM_CONFIG
    cmu_planning_enabled = environment.get("LINGTU_NAV_LOCAL_PLANNER_BACKEND") == "cmu" and (
        "traversability" in roles or environment.get("LINGTU_TELEOP_LOCAL_PLANNER") == "1"
    )
    if cmu_planning_enabled:
        try:
            profile = _CMU_PROFILES[env_spec.robot]
        except KeyError as exc:
            raise ValueError(
                f"Robot {env_spec.robot!r} has no CMU path library"
            ) from exc
        root = (
            "build/nav_endpoint/cmu_paths"
            if env_spec.name == "real"
            else "src/nav/cpp/planning/local/cmu/paths"
        )
        environment["LINGTU_LOCAL_PLANNER_PATHS"] = f"{root}/{profile}"
        # CMU's collision threshold is calibrated for the upstream 5 cm
        # obstacle voxel size. Keep SCAN's rolling-map pipeline independent.
        environment["LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M"] = "0.05"
    if process_control == "subprocess":
        # SimProcessManager injects runtime identity after publishing the RunPlan.
        environment.pop("LINGTU_PRODUCT", None)
        if "slam" in roles:
            environment["LINGTU_SLAM_MODE"] = str(lifecycle["slam_mode"])
    return environment


_REPO_ROOT = Path(__file__).resolve().parents[3]


def _product_roles(
    product_spec: Mapping[str, Any],
    *,
    product: str,
) -> tuple[str, ...]:
    value = product_spec.get("processes")
    if not isinstance(value, list | tuple):
        raise ValueError(f"Product {product!r} must declare processes as a list")
    roles = tuple(value)
    if (
        not roles
        or any(not valid_process_name(role) for role in roles)
        or len(set(roles)) != len(roles)
    ):
        raise ValueError(f"Product {product!r} has invalid or duplicate process roles")
    return roles


def _optional_parameter_profile(value: Any, *, product: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"Product {product!r} parameter_profile must be a string")
    return value.strip()


def _compile_lifecycle(product: str, spec: Mapping[str, Any]) -> dict[str, Any]:
    """Copy the Product lifecycle fields needed by the runtime."""

    def required(name: str) -> str:
        value = str(spec.get(name) or "").strip()
        if not value:
            raise ValueError(f"Product {product!r} must declare {name}")
        return value

    return {
        "product": product,
        "product_variant": spec.get("product_variant"),
        "label": str(spec.get("label") or product.replace("_", " ").title()),
        "session_mode": required("session_mode"),
        "native_control_mode": required("native_control_mode"),
        "slam_mode": required("slam_mode"),
        "requires_map": bool(spec.get("requires_map", False)),
        "switch_policy": required("switch_policy"),
        "default_for_session_mode": bool(spec.get("default_for_session_mode", False)),
        "online_hot_switch_supported": bool(
            spec.get("online_hot_switch_supported", False)
        ),
    }


def _parameter_overrides(
    *,
    env: str,
    implementation: Mapping[str, Any],
    env_spec: Any,
) -> Mapping[str, Any]:
    if not isinstance(env_spec, Mapping):
        raise ValueError(f"unknown Runtime Graph env: {env}")
    value = implementation.get(
        "parameter_overrides",
        env_spec.get("parameter_overrides", {}),
    )
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"Env {env!r} parameter_overrides must be a mapping")
    return value


def _string_tuple(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, set | frozenset):
        return tuple(sorted(str(item) for item in value))
    if isinstance(value, list | tuple):
        return tuple(str(item) for item in value)
    return ()
