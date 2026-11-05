"""Compile Products into immutable RunPlans and Host builders."""

from __future__ import annotations

import math
import re
from collections.abc import Mapping
from dataclasses import replace
from pathlib import Path
from typing import Any

from lingtu.assembly.config.camera import camera_environment
from lingtu.assembly.config.endpoint_bindings import (
    endpoint_transport_for_config,
)
from lingtu.assembly.config.explore import explore_environment
from lingtu.assembly.config.lidar import lidar_environment
from lingtu.assembly.config.maps import mapd_environment
from lingtu.assembly.config.native_navigation import (
    NATIVE_NAV_ENVIRONMENT,
    compile_native_nav_config,
)
from lingtu.assembly.config.native_parameters import PARAMETER_SPECS, resolve_parameters
from lingtu.assembly.config.product import (
    EnvSpec,
    resolve_product_host_runtime,
)
from lingtu.assembly.config.simulation import compile_simulation_snapshot
from lingtu.assembly.config.slam import slam_environment
from lingtu.assembly.graph import (
    ProcessSpec,
    load_runtime_graph,
    resolve_processes,
    resolve_stop_before_start,
    valid_process_name,
)
from lingtu.assembly.graph.loader import RuntimeGraph, product_requirements
from lingtu.products import product_name
from lingtu.run_plan import RunPlan
from runtime.host import HostBuilder

_CMU_PROFILES = {
    "doso/thunder_v4": "thunder",
    "unitree/go2": "go2",
}

_SIM_TRUTH_SLAM_CONFIG = "src/localization/ltlio/config/sim_mid360.yaml"
_SIM_LTLIO_SLAM_CONFIG = "src/localization/ltlio/config/sim_mid360_slam.yaml"


def host_builder_for_resolved_product(
    product: str,
    config: Mapping[str, Any],
    *,
    required_topics: tuple[str, ...] = (),
    required_capabilities: tuple[str, ...] = (),
    run_plan: RunPlan | None = None,
) -> HostBuilder:
    """Return a Host builder for an already resolved field Product."""

    product_name(product)

    from lingtu.assembly.host.build import host_builder

    return host_builder(
        config,
        product=product,
        required_topics=required_topics,
        required_capabilities=required_capabilities,
        run_plan=run_plan,
    )


def compile_run_plan(
    product: str,
    env: str,
    *,
    robot: str | None = None,
    product_variant: str | None = None,
    local_planner: str | None = None,
    env_config: Mapping[str, Any] | None = None,
    overrides: Mapping[str, Any] | None = None,
    parameter_overrides: Mapping[str, Any] | None = None,
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
    required_topics, required_capabilities = product_requirements(resolved_product, product_spec)

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
        session_source=env_spec.config.session,
    )
    processes, available_processes = _bind_unreal_camera(simulation, processes, available_processes)
    processes, available_processes, measurements_enabled = _bind_measurements(
        simulation, processes, available_processes, resolved_config
    )
    if env_name == "sim":
        geometry = simulation["physics_plan"]["robots"][0].get("navigation_geometry", {})
        for name in ("collision_clearance_below_m", "collision_clearance_above_m"):
            if name in geometry:
                resolved_config.setdefault(name, geometry[name])
        for name in ("support_height_m", "support_height_tolerance_m"):
            if name in geometry:
                resolved_config.setdefault(f"octoplanner3d_{name}", geometry[name])
    if env_name == "sim" and has_camera:
        resolved_config["detector"] = "sim_scene"
        world_mjcf = simulation["physics_plan"]["world"]["mjcf"]
        resolved_config.setdefault("world", Path(world_mjcf).as_posix())
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
    if measurements_enabled:
        critical_modules = (*critical_modules, *(name for name in ("measurements", "sensor.controls") if name not in critical_modules))
    host_builder = host_builder_for_resolved_product(
        resolved_product,
        resolved_config,
        required_topics=required_topics,
        required_capabilities=required_capabilities,
    )
    if route_contract:
        host_builder.route_contract(route_contract)
    if critical_modules:
        host_builder.require_modules(*critical_modules)
    from lingtu.assembly.config.validation import validate_product

    issues = validate_product(
        resolved_product,
        product_spec=product_spec,
        implementation=implementation,
        config=resolved_config,
        module_names=host_builder.module_names,
        env_name=env_name,
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
            "maps": product_spec.get("maps"),
        },
        settings_directory=graph.root / "settings",
    )
    physical_limits = None
    if env_spec.robot_config is not None:
        speed = env_spec.robot_config.speed
        if isinstance(speed.max_linear, bool) or isinstance(speed.max_angular, bool):
            raise ValueError("RobotConfig speed.max_linear and speed.max_angular must be finite and positive")
        physical_limits = (float(speed.max_linear), float(speed.max_angular))
        if any(not math.isfinite(value) or value <= 0.0 for value in physical_limits):
            raise ValueError("RobotConfig speed.max_linear and speed.max_angular must be finite and positive")
        compiled_native_nav = compiled_native_nav.limit_motion(*physical_limits)
    native_nav_contract = dict(compiled_native_nav.native_nav)
    del native_nav_contract["octoplanner3d"]
    del native_nav_contract["recovery"]
    del native_nav_contract["smoothing"]
    process_environment = _process_environment(
        product=resolved_product,
        settings=product_spec,
        env_spec=env_spec,
        native_environment=compiled_native_nav.environment,
        lifecycle=lifecycle,
        roles=product_roles,
        process_control=process_control,
    )
    if env_name == "sim":
        body_file = simulation["physics_plan"]["robots"][0].get("navigation_geometry", {}).get("body_file")
        if body_file:
            process_environment["LINGTU_NAV_BODY_FILE"] = (_REPO_ROOT / body_file).resolve().as_posix()
    product_parameters = product_spec.get("parameters", {})
    if not isinstance(product_parameters, Mapping):
        raise ValueError(f"Product {resolved_product!r} parameters must be a mapping")
    # Product defaults express the per-axis budget (1 m/s), while a route or
    # acceptance override may intentionally lower the total path speed. Keep
    # the local follower axes inside that resolved route budget instead of
    # letting a stale 1 m/s axis value bypass a lower path cap.
    product_parameters = dict(product_parameters)
    env_parameters = _env_parameters(
        env=env_name,
        implementation=implementation,
        env_spec=graph.envs.get(env_name),
    )
    if compiled_native_nav.native_nav["local_planner"] == "scan":
        path_speed = compiled_native_nav.parameters["path_follower_max_speed_mps"]
        minimum_planner_speed = next(spec.minimum for spec in PARAMETER_SPECS
                                    if spec.name == "scan_planner.max_velocity_mps")
        # SCAN initializes above this floor, then applies the actual request speed.
        planner_speed = max(path_speed, minimum_planner_speed)
        for axis in ("scan_follower.max_vx_mps", "scan_follower.max_vy_mps"):
            if axis in product_parameters:
                product_parameters[axis] = min(float(product_parameters[axis]), path_speed)
        forward_limit = compiled_native_nav.parameters[
            "teleop_max_speed_mps"
            if compiled_native_nav.native_nav["control_mode"] == "teleop_avoid"
            else "path_follower_max_speed_mps"
        ]
        env_parameters = {
            "scan_planner.max_velocity_mps": planner_speed,
            "scan_planner.max_acceleration_mps2": compiled_native_nav.parameters["path_follower_max_accel_mps2"],
            "scan_follower.max_vx_mps": forward_limit,
            "scan_follower.max_yaw_rate_rad_s": compiled_native_nav.parameters["path_follower_max_yaw_rate_rad_s"],
            **env_parameters,
        }
    parameters = resolve_parameters(
        settings_directory=graph.root / "settings",
        env_overrides=env_parameters,
        product_parameters=product_parameters,
        session_overrides=parameter_overrides,
        map_publish_hz=_traversability_publish_hz(
            env=env_name,
            implementation=implementation,
            env_spec=graph.envs.get(env_name),
        ),
    ).as_dict()
    if physical_limits is not None and compiled_native_nav.native_nav["local_planner"] == "scan":
        max_linear_mps, max_angular_rps = physical_limits
        parameters["scan_planner.max_velocity_mps"] = max(
            minimum_planner_speed,
            min(parameters["scan_planner.max_velocity_mps"], math.hypot(max_linear_mps, max_linear_mps)),
        )
        for name, limit in (
            ("scan_follower.max_vx_mps", max_linear_mps),
            ("scan_follower.max_vy_mps", max_linear_mps),
            ("scan_follower.max_yaw_rate_rad_s", max_angular_rps),
        ):
            parameters[name] = min(parameters[name], limit)
    return RunPlan.create(
        product=resolved_product,
        product_variant=resolved_product_variant,
        env=env_name,
        robot=resolved.robot,
        process_control=process_control,
        modules=host_builder.module_names,
        processes=processes,
        available_processes=available_processes,
        support_processes=support_processes,
        stop_before_start=stop_before_start,
        required_topics=required_topics,
        required_capabilities=required_capabilities,
        critical_modules=critical_modules,
        native_nav=native_nav_contract,
        process_environment=process_environment,
        route_contract=route_contract,
        host_config=resolved_config,
        lifecycle=lifecycle,
        parameters=parameters,
        simulation=simulation,
    )


def _bind_unreal_camera(
    simulation: Mapping[str, Any], processes: tuple[ProcessSpec, ...], available: tuple[ProcessSpec, ...],
) -> tuple[tuple[ProcessSpec, ...], tuple[ProcessSpec, ...]]:
    """Bind the existing native camera publisher's optical frame, not a new protocol."""
    streams = simulation.get("sensor_plan", {}).get("streams", {})
    cameras = [stream for kind in ("rgb", "depth") for stream in streams.get(kind, [])]
    if not any(stream.get("source") == "unreal_camera" for stream in cameras):
        return processes, available
    if simulation["session"]["runtime"]["mode"] != "unreal" or any(
        (s.get("owner"), s.get("source"), s.get("transport")) != ("visual", "unreal_camera", "camera_shm")
        for s in cameras
    ):
        raise ValueError("UE camera source requires explicit Unreal mode and one source owner")
    frames = {s.get("frame_id") for s in cameras}
    if len(frames) != 1:
        raise ValueError("Product RGB-D endpoint requires one optical frame")
    frame = next(iter(frames))
    if not isinstance(frame, str) or not frame or frame != frame.strip() or "\0" in frame:
        raise ValueError("UE camera optical frame must be explicit")
    endpoints = [p for p in processes if "camera" in p.provides]
    if len(endpoints) != 1 or endpoints[0].manager != "direct" or endpoints[0].command is None:
        raise ValueError("UE camera requires one native camera endpoint with an optical frame")
    endpoint = endpoints[0]
    command = endpoint.command
    assert command is not None
    if Path(command.artifact.path).name not in {"lingtu_mujoco_sensor_publisher", "lingtu_mujoco_sensor_publisher.exe"}:
        raise ValueError("UE camera frame requires the native sensor publisher")
    positions = [i for i, arg in enumerate(command.argv) if arg.split("=", 1)[0] == "--camera-frame"]
    if positions:
        i = positions[0]
        if (len(positions) != 1 or command.argv[i] != "--camera-frame" or i + 1 >= len(command.argv)
                or command.argv[i + 1] != frame):
            raise ValueError("native camera frame conflicts with the SensorPlan optical frame")
    else:
        command = replace(command, argv=(*command.argv, "--camera-frame", frame))
    matches = [p for p in available if p.name == endpoint.name]
    if len(matches) != 1 or matches[0].command != endpoint.command:
        raise ValueError("native camera frame endpoint must match available_processes")
    return tuple(replace(p, command=command) if p.name == endpoint.name else p for p in processes), tuple(
        replace(p, command=command) if p.name == endpoint.name else p for p in available
    )


def _bind_measurements(
    simulation: Mapping[str, Any],
    processes: tuple[ProcessSpec, ...],
    available_processes: tuple[ProcessSpec, ...],
    config: dict[str, Any],
) -> tuple[tuple[ProcessSpec, ...], tuple[ProcessSpec, ...], bool]:
    """Bind declared measurement streams to the existing authenticated endpoint."""
    sensor_plan = simulation.get("sensor_plan", {})
    streams = sensor_plan.get("streams", {})
    kinds = [kind for kind in ("thermal", "acoustic", "semantic") if kind in streams]
    if not kinds:
        return processes, available_processes, False
    session_id = sensor_plan.get("session_id")
    if not isinstance(session_id, str) or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,62}", session_id) is None:
        raise ValueError("measurement sensor_plan.session_id must be an explicit session ID")
    scenario = simulation.get("scenario_plan")
    if scenario is None:
        model_generation = reset_generation = 0
    else:
        if not isinstance(scenario, Mapping):
            raise ValueError("measurement scenario_plan must be a mapping")
        raw_model_generation = scenario.get("model_generation")
        raw_reset_generation = scenario.get("reset_generation")
        if (not isinstance(raw_model_generation, int) or isinstance(raw_model_generation, bool)
                or raw_model_generation < 0 or not isinstance(raw_reset_generation, int)
                or isinstance(raw_reset_generation, bool) or raw_reset_generation < 0):
            raise ValueError("measurement scenario generations must be non-negative integers")
        model_generation = raw_model_generation
        reset_generation = raw_reset_generation
    sensor_ids: list[str] = []
    measurement_sensors: dict[str, dict[str, Any]] = {}
    for kind in kinds:
        declarations = streams[kind]
        if not isinstance(declarations, list | tuple) or not declarations:
            raise ValueError(f"measurement streams.{kind} must be a nonempty list")
        for stream in declarations:
            sensor_id = stream.get("sensor_id") if isinstance(stream, Mapping) else None
            if (
                not isinstance(sensor_id, str) or not sensor_id or sensor_id != sensor_id.strip()
                or "\0" in sensor_id or sensor_id in sensor_ids
            ):
                raise ValueError("measurement streams require explicit unique sensor IDs")
            identity = {}
            for field in ("instance_id", "frame_id"):
                value = stream.get(field)
                if not isinstance(value, str) or not value or value != value.strip() or "\0" in value:
                    raise ValueError(f"measurement streams.{kind}.{field} must be an explicit source identity")
                identity[field] = value
            measurement_sensors[sensor_id] = {
                "modality": kind,
                "session_id": session_id,
                **identity,
                "model_generation": model_generation,
                "reset_generation": reset_generation,
            }
            sensor_ids.append(sensor_id)
    cameras = [process for process in processes if "camera" in process.provides]
    if len(cameras) != 1 or cameras[0].manager != "direct":
        raise ValueError("measurement streams require one selected native camera endpoint")
    camera = cameras[0]
    command = camera.command
    if command is None:
        raise ValueError("measurement streams require one selected native camera endpoint")
    if Path(command.artifact.path).name not in {
        "lingtu_mujoco_sensor_publisher", "lingtu_mujoco_sensor_publisher.exe"
    }:
        raise ValueError("measurement camera endpoint must use the native sensor publisher")
    argv = command.argv
    options = [arg.split("=", 1)[0] for arg in argv if arg.startswith("--")]
    if len(set(options)) != len(options):
        raise ValueError("measurement camera endpoint contains duplicate arguments")

    def argument(flag: str, *, takes_value: bool = False, optional: bool = False) -> str | None:
        positions = [index for index, arg in enumerate(argv) if arg == flag or arg.startswith(flag + "=")]
        if not positions and optional:
            return None
        if len(positions) != 1 or argv[positions[0]] != flag:
            raise ValueError(f"measurement camera endpoint requires exactly one {flag}")
        index = positions[0]
        if takes_value:
            if index + 1 == len(argv) or argv[index + 1].startswith("--"):
                raise ValueError(f"measurement camera endpoint requires a value for {flag}")
            return argv[index + 1]
        return None

    argument("--local-endpoint")
    argument("--dds")
    if argument("--stream", takes_value=True) != "camera":
        raise ValueError("measurement camera endpoint requires --stream camera")
    domain = argument("--domain-id", takes_value=True)
    if domain is None or not domain.isdecimal() or not 0 <= int(domain) <= 232:
        raise ValueError("measurement camera endpoint requires a DDS domain in 0..232")
    if dict(command.env).get("LINGTU_DDS_DOMAIN_ID", domain) != domain:
        raise ValueError("measurement camera endpoint DDS domain conflicts with its environment")
    forbidden = {
        "--measurement-product-session-id", "--stdin-records", "--validate-records",
        "--restamp-stdin-records", "--ready-file", "--navigation-fixture", "--sim-lidar-raw-frame",
    }
    if any(arg.split("=", 1)[0] in forbidden for arg in argv):
        raise ValueError("measurement camera endpoint has incompatible input or Product identity arguments")
    existing_session = argument("--measurement-session-id", takes_value=True, optional=True)
    if existing_session is not None and existing_session != session_id:
        raise ValueError("measurement camera endpoint session conflicts with sensor_plan.session_id")
    if existing_session is None:
        command = replace(command, argv=(*argv, "--measurement-session-id", session_id))
    available = [process for process in available_processes if process.name == camera.name]
    if len(available) != 1 or available[0].command != camera.command:
        raise ValueError("measurement camera endpoint must match available_processes")

    def bind(records: tuple[ProcessSpec, ...]) -> tuple[ProcessSpec, ...]:
        return tuple(replace(process, command=command) if process.name == camera.name else process for process in records)

    config.update(
        enable_measurements=True,
        measurement_session_id=session_id,
        measurement_sensor_ids=sensor_ids,
        measurement_sensors=measurement_sensors,
    )
    return bind(processes), bind(available_processes), True


def host_builder_from_run_plan(plan: RunPlan) -> HostBuilder:
    """Recreate only the Host graph declared by a RunPlan."""

    config = dict(plan.host_config)
    route_contract = route_contract_name_for_resolved_config(config)
    if route_contract != plan.route_contract:
        raise ValueError(
            "RunPlan Host route contract mismatch: "
            f"plan={plan.route_contract!r} config={route_contract!r}"
        )
    validate_route_contract_for_resolved_config(config)
    host_builder = host_builder_for_resolved_product(
        plan.product,
        config,
        required_topics=plan.required_topics,
        required_capabilities=plan.required_capabilities,
        run_plan=plan,
    )
    if plan.route_contract:
        host_builder.route_contract(plan.route_contract)
    if plan.critical_modules:
        host_builder.require_modules(*plan.critical_modules)
    if host_builder.module_names != plan.modules:
        raise ValueError(
            "RunPlan Host modules do not match the current assembly: "
            f"plan={plan.modules!r} assembly={host_builder.module_names!r}"
        )
    return host_builder


def build_host_from_run_plan(plan: RunPlan) -> Any:
    """Build a Host from immutable Product data without resolving processes."""

    host_builder = host_builder_from_run_plan(plan)
    return host_builder.build()


def route_contract_name_for_resolved_config(config: Mapping[str, Any]) -> str | None:
    """Return the external route contract selected by a resolved config.

    Route contracts are boundary metadata. They validate endpoint topics and
    adapter ownership, but they do not change ModulePort delivery unless a
    HostBuilder route contracts remain metadata and never select Module delivery.
    """

    endpoint_transport = endpoint_transport_for_config(config, default="").lower()
    if endpoint_transport == "dds":
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


def _process_environment(
    *,
    product: str,
    settings: Mapping[str, Any],
    env_spec: EnvSpec,
    native_environment: Mapping[str, str],
    lifecycle: Mapping[str, Any],
    roles: tuple[str, ...],
    process_control: str,
) -> dict[str, str]:
    environment = {
        key: value
        for key, value in native_environment.items()
        if key not in NATIVE_NAV_ENVIRONMENT.values()
    }
    if "maps" in roles:
        environment["LINGTU_MAPD_EXTENDED_LAYERS"] = "1" if product == "map" else "0"
    if "traversability" in roles:
        environment["LINGTU_EXPLORE_ROUTE"] = (
            "live" if lifecycle["slam_mode"] == "mapping" else "map"
        )
    if "maps" in roles:
        environment.update(mapd_environment(settings["maps"], native_environment))
    if "slam" in roles:
        environment.update(slam_environment(settings["slam"], simulation=env_spec.name == "sim"))
    if "explore" in roles:
        environment.update(explore_environment(settings["explore"]))
    if env_spec.name == "real":
        environment["LINGTU_WEBRTC_URL"] = ""
        robot_config = env_spec.robot_config
        if robot_config is None:
            raise RuntimeError("real Env resolved without RobotConfig")
        if "camera" in roles:
            environment.update(camera_environment(settings["camera"], robot_config.camera, include_video="video" in roles))
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
            "LINGTU_DRIVER_TILT_LIMIT_DEG": safety.tilt_limit_deg,
            "LINGTU_DRIVER_POLL_HZ": driver.control_rate,
            "LINGTU_DRIVER_RECONNECT_MS": driver.reconnect_interval * 1000.0,
            "LINGTU_DRIVER_TLS_CA_FILE": driver.tls_ca_file,
            "LINGTU_DRIVER_TLS_CERT_FILE": driver.tls_cert_file,
            "LINGTU_DRIVER_TLS_KEY_FILE": driver.tls_key_file,
            "LINGTU_DRIVER_TLS_SERVER_NAME": driver.tls_server_name,
            "LINGTU_DRIVER_STAIRS_UP_PROFILE": driver.stairs_up_profile,
            "LINGTU_DRIVER_STAIRS_DOWN_PROFILE": driver.stairs_down_profile,
        }
        body_file = str(robot_config.geometry.navigation_body_file or "").strip()
        if body_file:
            if Path(body_file).is_absolute() or ".." in Path(body_file).parts:
                raise ValueError("geometry.navigation_body_file must be a relative path")
            if env_spec.robot_config_ref is None:
                raise RuntimeError("navigation body model requires RobotConfig path")
            body_path = (env_spec.robot_config_ref.parent / body_file).resolve()
            if not body_path.is_file():
                raise ValueError(f"navigation body model does not exist: {body_path}")
            values["LINGTU_NAV_BODY_FILE"] = body_path.as_posix()
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
            environment.update(lidar_environment(settings["lidar"], robot_config))
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
        environment["LINGTU_SLAM_CONFIG"] = (
            _SIM_TRUTH_SLAM_CONFIG
            if env_spec.config.localization == "truth"
            else _SIM_LTLIO_SLAM_CONFIG
        )
    cmu_planning_enabled = native_environment.get("LINGTU_NAV_LOCAL_PLANNER_BACKEND") == "cmu" and (
        "traversability" in roles or native_environment.get("LINGTU_TELEOP_LOCAL_PLANNER") == "1"
    )
    if cmu_planning_enabled:
        try:
            profile = _CMU_PROFILES[env_spec.robot]
        except KeyError as exc:
            raise ValueError(
                f"Robot {env_spec.robot!r} has no CMU path library"
            ) from exc
        root = (
            "share/lingtu/cmu_paths"
            if env_spec.name == "real"
            else "src/nav/cpp/planning/local/cmu/paths"
        )
        environment["LINGTU_LOCAL_PLANNER_PATHS"] = f"{root}/{profile}"
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


def _env_parameters(
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


def _traversability_publish_hz(
    *,
    env: str,
    implementation: Mapping[str, Any],
    env_spec: Any,
) -> float:
    if not isinstance(env_spec, Mapping):
        raise ValueError(f"unknown Runtime Graph env: {env}")
    raw = implementation.get(
        "traversability_publish_hz",
        env_spec.get("traversability_publish_hz"),
    )
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"Env {env!r} traversability_publish_hz must be positive"
        ) from exc
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"Env {env!r} traversability_publish_hz must be positive")
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
