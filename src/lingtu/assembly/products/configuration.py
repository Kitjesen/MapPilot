"""Resolve the Python Host portion of one Product in one Env."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, replace
from pathlib import Path
from types import MappingProxyType
from typing import Any

import yaml

from lingtu.assembly.native_nav import local_planner_name
from lingtu.products import ProductName, product_name
from runtime.config import RobotConfig, load_config
from runtime.graph.loader import RuntimeGraph, load_runtime_graph, resolve_product_variant_spec
from runtime.runtime_interface import lidar_extrinsic

from .host_defaults import (
    FIELD_PRODUCT_HOST_DEFAULTS,
    FIELD_PRODUCT_VARIANT_HOST_DEFAULTS,
)
from .runtime_paths import DEFAULT_GATEWAY_PORT, DEFAULT_PLANNING_FRAME_ID

_REPO_ROOT = Path(__file__).resolve().parents[4]
_ROBOT_MODELS_ROOT = _REPO_ROOT / "config" / "robots"
_ENV_NAMES = ("real", "sim")
_RESERVED_PRODUCT_OVERRIDE_KEYS = frozenset(
    {
        "env",
        "_env",
        "env_config",
        "backend",
        "_env_backend",
        "robot",
        "driver_backend",
        "_driver_backend",
        "driver_target",
        "driver_network_interface",
        "driver_network_address",
        "driver_probe_ip",
        "driver_control_rate",
        "driver_reconnect_interval",
        "driver_max_linear_mps",
        "driver_max_angular_rps",
        "driver_cmd_timeout_ms",
        "driver_tls_ca_file",
        "driver_tls_cert_file",
        "driver_tls_key_file",
        "driver_tls_server_name",
        "robot_config",
        "_robot_config_ref",
        "_slam_config_ref",
        "lidar_extrinsic_profile",
        "lidar_ip",
        "lidar_host_ip",
        "lidar_network_interface",
        "vehicle_length_m",
        "vehicle_width_m",
        "collision_hard_margin_m",
        "collision_cylinder_radius_m",
        "collision_cylinder_offset_m",
        "collision_clearance_below_m",
        "collision_clearance_above_m",
        "sensor_offset_x_m",
        "sensor_offset_y_m",
        "sensor_offset_z_m",
        "product_variant",
        "_product_variant",
        "endpoint_transport",
        "_endpoint_transport",
        "endpoint_contract",
        "_endpoint_contract",
        "slam_mode",
        "slam_profile",
    }
)

# These are Host implementation defaults for the real environment, not a
# physical RobotConfig and not a runtime selector. Product-owned values win.
_REAL_HOST_DEFAULTS: Mapping[str, Any] = MappingProxyType(
    {
        "detector": "bpu",
    }
)

_COMMON_HOST_DEFAULTS: Mapping[str, Any] = MappingProxyType(
    {
        "enable_gateway": True,
        "gateway_port": DEFAULT_GATEWAY_PORT,
        "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
    }
)

@dataclass(frozen=True)
class EnvConfig:
    """Internal implementation selection for an Env.

    ``backend`` is required only for ``sim``. It is configuration beneath the
    Env identity and never becomes a third runtime identity.
    """

    backend: str | None = None
    viewer: bool = False


@dataclass(frozen=True)
class EnvSpec:
    """Resolved Env implementation and its optional physical RobotConfig."""

    name: str
    robot: str
    config: EnvConfig
    implementation: Mapping[str, Any]
    robot_config_ref: Path | None = None
    robot_config: RobotConfig | None = None
    slam_config_ref: Path | None = None
    lidar_extrinsic_profile: str | None = None
    lidar_ip: str | None = None
    lidar_host_ip: str | None = None
    lidar_network_interface: str | None = None


@dataclass(frozen=True)
class ResolvedProductHostConfig:
    """Resolved Product Host config for one Robot, Product, and Env."""

    robot: str
    product: ProductName
    env: str
    config: Mapping[str, Any]
    product_spec: Mapping[str, Any]
    env_spec: EnvSpec
    product_variant: str | None = None


def resolve_env_spec(
    env: str,
    *,
    robot: str | None = None,
    env_config: EnvConfig | Mapping[str, Any] | None = None,
    graph: RuntimeGraph | None = None,
) -> EnvSpec:
    """Resolve and validate one public ``real`` or ``sim`` Env."""

    env_name = str(env).strip()
    if env_name not in _ENV_NAMES:
        choices = ", ".join(_ENV_NAMES)
        raise ValueError(f"unknown Env '{env_name}' (expected: {choices})")

    graph = graph or load_runtime_graph()
    robot_name, robot_dir, _ = _load_robot_model(robot)
    normalized = _normalize_env_config(
        env_name,
        env_config,
        default_backend=(
            _single_sim_backend(graph) if env_name == "sim" else None
        ),
    )
    selector_config = (
        {"backend": normalized.backend} if normalized.backend is not None else None
    )

    # Keep the runtime-graph import local so Product config remains cheap to import.
    from runtime.graph import resolve_env_implementation

    try:
        implementation = resolve_env_implementation(
            env_name,
            graph=graph,
            env_config=selector_config,
        )
    except (KeyError, ValueError) as exc:
        if env_name == "sim" and normalized.backend is not None:
            raise ValueError(
                f"unsupported sim backend '{normalized.backend}'"
            ) from exc
        raise

    if not isinstance(implementation, Mapping):
        raise TypeError(f"Env '{env_name}' implementation must be a mapping")

    implementation_copy = dict(implementation)
    if env_name == "real":
        robot_config_ref = _resolve_robot_config_ref(robot_name, robot_dir)
        robot_config = load_config(str(robot_config_ref))
        slam_config_ref = None
        lidar_extrinsic_profile = None
        lidar_ip = robot_config.lidar.lidar_ip
        lidar_host_ip = robot_config.lidar.host_ip
        lidar_network_interface = robot_config.lidar.network_interface
    else:
        robot_config_ref = None
        robot_config = None
        slam_config_ref = None
        lidar_extrinsic_profile = None
        lidar_ip = None
        lidar_host_ip = None
        lidar_network_interface = None

    return EnvSpec(
        name=env_name,
        robot=robot_name,
        config=normalized,
        implementation=MappingProxyType(implementation_copy),
        robot_config_ref=robot_config_ref,
        robot_config=robot_config,
        slam_config_ref=slam_config_ref,
        lidar_extrinsic_profile=lidar_extrinsic_profile,
        lidar_ip=lidar_ip,
        lidar_host_ip=lidar_host_ip,
        lidar_network_interface=lidar_network_interface,
    )


def resolve_product_host_runtime(
    product: str,
    env: str,
    *,
    robot: str | None = None,
    product_variant: str | None = None,
    local_planner: str | None = None,
    env_config: EnvConfig | Mapping[str, Any] | None = None,
    overrides: Mapping[str, Any] | None = None,
    graph: RuntimeGraph | None = None,
) -> ResolvedProductHostConfig:
    """Resolve one Product Host for exactly one public Env."""

    resolved_product_name = product_name(product)
    graph = graph or load_runtime_graph()
    resolved_product_spec = resolve_product_variant_spec(
        resolved_product_name,
        graph.products[resolved_product_name],
        product_variant=product_variant,
    )
    if local_planner is not None:
        native_nav = resolved_product_spec.get("native_nav")
        if not isinstance(native_nav, Mapping):
            raise ValueError(
                f"Product {resolved_product_name!r} does not use native local planning"
            )
        selectable = native_nav.get("local_planners")
        if not isinstance(selectable, (list, tuple)) or not selectable:
            raise ValueError(
                f"Product {resolved_product_name!r} does not declare selectable local planners"
            )
        selected = local_planner_name(local_planner)
        available = tuple(local_planner_name(value) for value in selectable)
        if selected not in available:
            raise ValueError(
                f"Product {resolved_product_name!r} does not support local planner {selected!r}"
            )
        resolved_product_spec["native_nav"] = {
            **native_nav,
            "local_planner": selected,
        }
    resolved_product_variant = resolved_product_spec.get("product_variant")
    product_processes = resolved_product_spec.get("processes")
    if not isinstance(product_processes, (list, tuple)):
        raise TypeError(
            f"Product {resolved_product_name!r} processes must be a list"
        )
    uses_lidar = "lidar" in product_processes
    resolved_env = resolve_env_spec(
        env,
        robot=robot,
        env_config=env_config,
        graph=graph,
    )
    if uses_lidar and resolved_env.name == "real":
        resolved_env = _with_real_mid360(resolved_env)
        if not all(
            (
                resolved_env.lidar_ip,
                resolved_env.lidar_host_ip,
                resolved_env.lidar_network_interface,
            )
        ):
            raise ValueError(
                f"Robot {resolved_env.robot!r} has incomplete MID-360 network "
                f"configuration for Product {resolved_product_name!r}"
            )
    _require_supported_product(
        resolved_product_name,
        resolved_env,
        product_variant=(
            str(resolved_product_variant)
            if resolved_product_variant is not None
            else None
        ),
    )
    native_nav = resolved_product_spec.get("native_nav")
    if isinstance(native_nav, Mapping):
        selected_local_planner = local_planner_name(
            native_nav.get("local_planner") or "cmu"
        )
        declared_local_planners = resolved_env.implementation.get("local_planners")
        if not isinstance(declared_local_planners, (list, tuple)):
            raise ValueError(
                f"Env {resolved_env.name!r} does not declare qualified local planners"
            )
        qualified_local_planners = tuple(
            local_planner_name(value) for value in declared_local_planners
        )
        if selected_local_planner not in qualified_local_planners:
            raise ValueError(
                f"Env {resolved_env.name!r} has not qualified local planner "
                f"{selected_local_planner!r}"
            )
    _reject_reserved_overrides(overrides)

    source_defaults = FIELD_PRODUCT_HOST_DEFAULTS[resolved_product_name]
    if resolved_product_variant is not None:
        source_defaults = FIELD_PRODUCT_VARIANT_HOST_DEFAULTS[
            resolved_product_name
        ][str(resolved_product_variant)]
    config = dict(_COMMON_HOST_DEFAULTS)
    config.update(source_defaults)

    if resolved_env.name == "real":
        _apply_defaults(config, _REAL_HOST_DEFAULTS)

    host_config = resolved_env.implementation.get("host_config")
    if not isinstance(host_config, Mapping):
        raise TypeError(
            f"Env '{resolved_env.name}' implementation must declare host_config"
        )
    config.update(host_config)

    slam_mode = str(resolved_product_spec.get("slam_mode") or "").strip().lower()
    if slam_mode not in {"mapping", "localization", "none"}:
        raise ValueError(
            f"Product {resolved_product_name!r} has unsupported slam_mode {slam_mode!r}"
        )
    config["slam_mode"] = slam_mode
    config["slam_profile"] = "none" if slam_mode == "none" else "native_dds"

    if resolved_env.robot_config is not None:
        robot_config = resolved_env.robot_config
        driver = robot_config.driver
        geometry = robot_config.geometry
        lidar = robot_config.lidar
        config.update(
            {
                "driver_backend": driver.backend,
                "vehicle_length_m": geometry.vehicle_length,
                "vehicle_width_m": geometry.vehicle_width,
                "collision_hard_margin_m": geometry.collision_hard_margin,
                "collision_cylinder_radius_m": geometry.collision_cylinder_radius,
                "collision_cylinder_offset_m": geometry.collision_cylinder_offset,
                "collision_clearance_below_m": geometry.collision_clearance_below,
                "collision_clearance_above_m": geometry.collision_clearance_above,
                "sensor_offset_x_m": lidar.offset_x,
                "sensor_offset_y_m": lidar.offset_y,
                "sensor_offset_z_m": lidar.offset_z,
            }
        )
        if uses_lidar:
            config["lidar_ip"] = resolved_env.lidar_ip

    config.update(dict(overrides or {}))
    config["_selection_kind"] = "product"
    config["_env"] = resolved_env.name
    if resolved_product_variant is not None:
        config["_product_variant"] = str(resolved_product_variant)
    if resolved_env.config.backend is not None:
        config["_env_backend"] = resolved_env.config.backend

    return ResolvedProductHostConfig(
        robot=resolved_env.robot,
        product=resolved_product_name,
        env=resolved_env.name,
        config=dict(config),
        product_spec=MappingProxyType(dict(resolved_product_spec)),
        env_spec=resolved_env,
        product_variant=(
            str(resolved_product_variant)
            if resolved_product_variant is not None
            else None
        ),
    )


def resolve_product_host_config(
    product: str,
    env: str,
    *,
    robot: str | None = None,
    product_variant: str | None = None,
    env_config: EnvConfig | Mapping[str, Any] | None = None,
    overrides: Mapping[str, Any] | None = None,
    **inline_overrides: Any,
) -> dict[str, Any]:
    """Return Blueprint kwargs for the Host inside one Product."""

    merged_overrides = dict(overrides or {})
    merged_overrides.update(inline_overrides)
    _reject_reserved_overrides(merged_overrides)
    resolved = resolve_product_host_runtime(
        product,
        env,
        robot=robot,
        product_variant=product_variant,
        env_config=env_config,
        overrides=merged_overrides,
    )
    return dict(resolved.config)


def _load_robot_model(
    robot: str | None,
) -> tuple[str, Path, Mapping[str, Any]]:
    models: dict[str, Path] = {}
    for path in sorted(_ROBOT_MODELS_ROOT.glob("*/*/model.yaml")):
        model_id = path.parent.relative_to(_ROBOT_MODELS_ROOT).as_posix()
        models[model_id] = path

    requested = str(robot or "").strip()
    if not requested:
        raise ValueError("Robot is required")
    path = models.get(requested)
    if path is None:
        raise ValueError(f"unknown Robot {requested!r}")
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise TypeError(f"Robot model must be a mapping: {path}")
    return requested, path.parent, payload


def _resolve_mid360_config(
    robot: str,
    robot_dir: Path,
    model: Mapping[str, Any],
) -> tuple[Path | None, str | None]:
    sensors = model.get("sensors")
    if sensors is None:
        return None, None
    if not isinstance(sensors, Mapping):
        raise TypeError(f"Robot {robot!r} must declare sensors")
    mid360 = sensors.get("mid360")
    if mid360 is None:
        return None, None
    if not isinstance(mid360, Mapping):
        raise TypeError(f"Robot {robot!r} must declare sensors.mid360")

    raw_config = mid360.get("fastlio2_config")
    if not isinstance(raw_config, str) or not raw_config.strip():
        raise TypeError(
            f"Robot {robot!r} sensors.mid360.fastlio2_config must be a string"
        )
    config_ref = (robot_dir / raw_config.strip()).resolve()
    if not config_ref.is_file():
        raise FileNotFoundError(
            f"Robot {robot!r} MID-360 Fast-LIO2 config not found: {config_ref}"
        )

    raw_profile = mid360.get("extrinsic_profile")
    if not isinstance(raw_profile, str) or not raw_profile.strip():
        raise TypeError(
            f"Robot {robot!r} sensors.mid360.extrinsic_profile must be a string"
        )
    return config_ref, raw_profile.strip()


def _with_real_mid360(env: EnvSpec) -> EnvSpec:
    _, robot_dir, model = _load_robot_model(env.robot)
    config_ref, profile = _resolve_mid360_config(env.robot, robot_dir, model)
    if config_ref is None or profile is None:
        raise ValueError(f"Robot {env.robot!r} has no MID-360 configuration")
    robot_config = env.robot_config
    if robot_config is None:
        raise RuntimeError("real Env resolved without RobotConfig")

    mount = lidar_extrinsic(profile)
    configured_mount = (
        robot_config.lidar.offset_x,
        robot_config.lidar.offset_y,
        robot_config.lidar.offset_z,
        robot_config.lidar.roll,
        robot_config.lidar.pitch,
        robot_config.lidar.yaw,
    )
    declared_mount = (
        mount.x,
        mount.y,
        mount.z,
        mount.roll,
        mount.pitch,
        mount.yaw,
    )
    if not all(
        math.isclose(configured, declared, abs_tol=1e-12)
        for configured, declared in zip(configured_mount, declared_mount)
    ):
        raise ValueError(
            f"Robot {env.robot!r} MID-360 extrinsic profile differs from RobotConfig"
        )
    return replace(
        env,
        slam_config_ref=config_ref,
        lidar_extrinsic_profile=profile,
    )


def _normalize_env_config(
    env: str,
    env_config: EnvConfig | Mapping[str, Any] | None,
    *,
    default_backend: str | None = None,
) -> EnvConfig:
    if env_config is None:
        normalized = EnvConfig()
    elif isinstance(env_config, EnvConfig):
        normalized = env_config
    elif isinstance(env_config, Mapping):
        unknown = sorted(set(env_config) - {"backend", "viewer"})
        if unknown:
            joined = ", ".join(str(key) for key in unknown)
            raise TypeError(f"unsupported env_config field(s): {joined}")
        normalized = EnvConfig(
            backend=env_config.get("backend"),
            viewer=env_config.get("viewer", False),
        )
    else:
        raise TypeError("env_config must be EnvConfig, a mapping, or None")

    if not isinstance(normalized.viewer, bool):
        raise TypeError("env_config.viewer must be bool")

    backend = normalized.backend or default_backend
    if backend is not None:
        if not isinstance(backend, str) or not backend.strip():
            raise TypeError("env_config.backend must be a non-empty string")
        normalized = replace(normalized, backend=backend.strip())

    if env == "real" and normalized.backend is not None:
        raise ValueError("real Env does not accept env_config.backend")
    if env == "real" and normalized.viewer:
        raise ValueError("real Env does not accept env_config.viewer")
    if env == "sim" and normalized.backend is None:
        raise ValueError("sim env_config.backend is required")
    return normalized


def _single_sim_backend(graph: RuntimeGraph) -> str | None:
    sim = graph.envs.get("sim")
    if not isinstance(sim, Mapping):
        return None
    backends = sim.get("backends")
    if not isinstance(backends, Mapping) or len(backends) != 1:
        return None
    return str(next(iter(backends))).strip() or None


def _resolve_robot_config_ref(robot: str, robot_dir: Path) -> Path:
    candidate = (robot_dir / "robot.yaml").resolve()
    if not candidate.is_file():
        raise FileNotFoundError(
            f"Robot {robot!r} has no real RobotConfig: {candidate}"
        )
    return candidate


def _require_supported_product(
    product: str,
    env: EnvSpec,
    *,
    product_variant: str | None,
) -> None:
    supported = env.implementation.get("supported_products")
    if not isinstance(supported, (list, tuple)) or not all(
        isinstance(item, str) for item in supported
    ):
        raise TypeError(
            f"Env '{env.name}' implementation must declare supported_products"
        )
    if product not in supported:
        backend = (
            f" backend '{env.config.backend}'"
            if env.config.backend is not None
            else ""
        )
        raise ValueError(
            f"Env '{env.name}'{backend} does not support Product '{product}'"
        )

    variant_limits = env.implementation.get("supported_product_variants")
    if variant_limits is None:
        return
    if not isinstance(variant_limits, Mapping):
        raise TypeError(
            f"Env '{env.name}' implementation supported_product_variants "
            "must be a mapping"
        )
    allowed = variant_limits.get(product)
    if allowed is None:
        return
    if (
        not isinstance(allowed, (list, tuple))
        or not allowed
        or not all(
            isinstance(item, str)
            and bool(item.strip())
            and item == item.strip()
            for item in allowed
        )
        or len(set(allowed)) != len(allowed)
    ):
        raise TypeError(
            f"Env '{env.name}' implementation Product '{product}' variant "
            "limits must be a non-empty string list"
        )
    if product_variant not in allowed:
        backend = (
            f" backend '{env.config.backend}'"
            if env.config.backend is not None
            else ""
        )
        raise ValueError(
            f"Env '{env.name}'{backend} does not support Product '{product}' "
            f"variant {product_variant!r}"
        )


def _apply_defaults(config: dict[str, Any], defaults: Mapping[str, Any]) -> None:
    for key, value in defaults.items():
        config.setdefault(key, value)


def _reject_reserved_overrides(overrides: Mapping[str, Any] | None) -> None:
    if not overrides:
        return
    reserved = sorted(
        key
        for key in overrides
        if key in _RESERVED_PRODUCT_OVERRIDE_KEYS or key.endswith("_preset")
    )
    if reserved:
        joined = ", ".join(reserved)
        raise TypeError(f"reserved Product resolution key(s): {joined}")


__all__ = [
    "EnvConfig",
    "EnvSpec",
    "ResolvedProductHostConfig",
    "resolve_env_spec",
    "resolve_product_host_config",
    "resolve_product_host_runtime",
]
