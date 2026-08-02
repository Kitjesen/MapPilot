"""Resolve the Python Host portion of one Product in one Env."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any

from runtime.config import RobotConfig, load_config
from runtime.graph.loader import load_runtime_graph, resolve_product_variant_spec
from runtime.profiles.product_lifecycle import ProductName, product_name
from runtime.profiles.resolver import finalize_host_config

from .host_defaults import (
    FIELD_PRODUCT_HOST_DEFAULTS,
    FIELD_PRODUCT_VARIANT_HOST_DEFAULTS,
)

_REPO_ROOT = Path(__file__).resolve().parents[4]
_CANONICAL_ROBOT_CONFIG = (_REPO_ROOT / "config" / "robot_config.yaml").resolve()
_ENV_NAMES = ("real", "sim")
_RESERVED_PRODUCT_OVERRIDE_KEYS = frozenset(
    {
        "env",
        "_env",
        "env_config",
        "backend",
        "_env_backend",
        "profile_adapter",
        "_profile_adapter",
        "robot",
        "driver_backend",
        "_driver_backend",
        "robot_config",
        "_robot_config_ref",
        "product_variant",
        "_product_variant",
        "endpoint_transport",
        "_endpoint_transport",
        "endpoint_contract",
        "_endpoint_contract",
    }
)

# These are Host implementation defaults for the real environment, not a
# physical RobotConfig and not a runtime selector. Product-owned values win.
_REAL_HOST_DEFAULTS: Mapping[str, Any] = MappingProxyType(
    {
        "slam_profile": "localizer",
        "detector": "bpu",
        "encoder": "mobileclip",
    }
)

_SIM_HOST_DEFAULTS: Mapping[str, Mapping[str, Any]] = MappingProxyType(
    {
        "mujoco_native": MappingProxyType({"slam_profile": "localizer"}),
        "mujoco_host": MappingProxyType({"slam_profile": "none", "llm": "mock"}),
        "gazebo": MappingProxyType({"slam_profile": "none", "llm": "mock"}),
    }
)


@dataclass(frozen=True)
class EnvConfig:
    """Internal implementation selection for an Env.

    ``backend`` is required only for ``sim``. It is configuration beneath the
    Env identity and never becomes a third runtime identity.
    """

    backend: str | None = None


@dataclass(frozen=True)
class EnvSpec:
    """Resolved Env implementation and its optional physical RobotConfig."""

    name: str
    config: EnvConfig
    implementation: Mapping[str, Any]
    robot_config_ref: Path | None = None
    robot_config: RobotConfig | None = None


@dataclass(frozen=True)
class ResolvedProductHostConfig:
    """Resolved Product Host config with Product and Env as its only identity."""

    product: ProductName
    env: str
    config: Mapping[str, Any]
    product_variant: str | None = None


def resolve_env_spec(
    env: str,
    *,
    env_config: EnvConfig | Mapping[str, Any] | None = None,
) -> EnvSpec:
    """Resolve and validate one public ``real`` or ``sim`` Env."""

    env_name = str(env).strip()
    if env_name not in _ENV_NAMES:
        choices = ", ".join(_ENV_NAMES)
        raise ValueError(f"unknown Env '{env_name}' (expected: {choices})")

    normalized = _normalize_env_config(env_name, env_config)
    selector_config = (
        {"backend": normalized.backend} if normalized.backend is not None else None
    )

    # Lazy import keeps local Host Profile resolution independent from Runtime
    # Graph loading and avoids treating an Env as a Profile catalog entry.
    from runtime.graph import resolve_env_implementation

    try:
        implementation = resolve_env_implementation(
            env_name,
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
        robot_config_ref = _canonical_robot_config_ref(implementation_copy)
        robot_config = load_config(str(robot_config_ref))
    else:
        robot_config_ref = None
        robot_config = None

    return EnvSpec(
        name=env_name,
        config=normalized,
        implementation=MappingProxyType(implementation_copy),
        robot_config_ref=robot_config_ref,
        robot_config=robot_config,
    )


def resolve_product_host_runtime(
    product: str,
    env: str,
    *,
    product_variant: str | None = None,
    env_config: EnvConfig | Mapping[str, Any] | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_product_metadata: bool = False,
) -> ResolvedProductHostConfig:
    """Resolve one Product Host for exactly one public Env."""

    resolved_product_name = product_name(product)
    resolved_product_spec = resolve_product_variant_spec(
        resolved_product_name,
        load_runtime_graph().products[resolved_product_name],
        product_variant=product_variant,
    )
    resolved_product_variant = resolved_product_spec.get("product_variant")
    resolved_env = resolve_env_spec(env, env_config=env_config)
    _require_supported_product(
        resolved_product_name,
        resolved_env,
        product_variant=(
            str(resolved_product_variant)
            if resolved_product_variant is not None
            else None
        ),
    )
    _reject_reserved_overrides(overrides)

    source_defaults = FIELD_PRODUCT_HOST_DEFAULTS[resolved_product_name]
    if resolved_product_variant is not None:
        source_defaults = FIELD_PRODUCT_VARIANT_HOST_DEFAULTS[
            resolved_product_name
        ][str(resolved_product_variant)]
    if include_product_metadata:
        config = dict(source_defaults)
    else:
        config = {
            key: value
            for key, value in source_defaults.items()
            if not key.startswith("_")
        }

    if resolved_env.name == "real":
        _apply_defaults(config, _REAL_HOST_DEFAULTS)
    else:
        backend = resolved_env.config.backend
        if backend is None:
            raise RuntimeError("sim Env resolved without an implementation backend")
        try:
            _apply_defaults(config, _SIM_HOST_DEFAULTS[backend])
        except KeyError as exc:
            raise ValueError(f"unsupported sim backend '{backend}'") from exc

    host_config = resolved_env.implementation.get("host_config")
    if not isinstance(host_config, Mapping):
        raise TypeError(
            f"Env '{resolved_env.name}' implementation must declare host_config"
        )
    config.update(host_config)

    if resolved_env.robot_config is not None:
        driver = resolved_env.robot_config.driver
        config.update(
            {
                "dog_host": driver.dog_host,
                "dog_port": driver.dog_port,
                "auto_enable": driver.auto_enable,
                "auto_standup": driver.auto_standup,
                "_robot_config_ref": "config/robot_config.yaml",
            }
        )

    config.update(dict(overrides or {}))
    config["_selection_kind"] = "product"
    config["_env"] = resolved_env.name
    if resolved_product_variant is not None:
        config["_product_variant"] = str(resolved_product_variant)
    if resolved_env.config.backend is not None:
        config["_env_backend"] = resolved_env.config.backend

    return ResolvedProductHostConfig(
        product=resolved_product_name,
        env=resolved_env.name,
        config=finalize_host_config(resolved_product_name, config),
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
    product_variant: str | None = None,
    env_config: EnvConfig | Mapping[str, Any] | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_product_metadata: bool = False,
    **inline_overrides: Any,
) -> dict[str, Any]:
    """Return Blueprint kwargs for the Host inside one Product."""

    merged_overrides = dict(overrides or {})
    merged_overrides.update(inline_overrides)
    _reject_reserved_overrides(merged_overrides)
    resolved = resolve_product_host_runtime(
        product,
        env,
        product_variant=product_variant,
        env_config=env_config,
        overrides=merged_overrides,
        include_product_metadata=include_product_metadata,
    )
    return dict(resolved.config)


def _normalize_env_config(
    env: str,
    env_config: EnvConfig | Mapping[str, Any] | None,
) -> EnvConfig:
    if env_config is None:
        normalized = EnvConfig()
    elif isinstance(env_config, EnvConfig):
        normalized = env_config
    elif isinstance(env_config, Mapping):
        unknown = sorted(set(env_config) - {"backend"})
        if unknown:
            joined = ", ".join(str(key) for key in unknown)
            raise TypeError(f"unsupported env_config field(s): {joined}")
        normalized = EnvConfig(backend=env_config.get("backend"))
    else:
        raise TypeError("env_config must be EnvConfig, a mapping, or None")

    backend = normalized.backend
    if backend is not None:
        if not isinstance(backend, str) or not backend.strip():
            raise TypeError("env_config.backend must be a non-empty string")
        normalized = EnvConfig(backend=backend.strip())

    if env == "real" and normalized.backend is not None:
        raise ValueError("real Env does not accept env_config.backend")
    if env == "sim" and normalized.backend is None:
        raise ValueError("sim env_config.backend is required")
    return normalized


def _canonical_robot_config_ref(implementation: Mapping[str, Any]) -> Path:
    raw_ref = implementation.get("robot_config_ref")
    if not isinstance(raw_ref, str) or not raw_ref.strip():
        raise ValueError("real Env must reference config/robot_config.yaml")
    candidate = (_REPO_ROOT / raw_ref).resolve()
    if candidate != _CANONICAL_ROBOT_CONFIG:
        raise ValueError(
            "real Env robot_config_ref must be config/robot_config.yaml"
        )
    if not candidate.is_file():
        raise FileNotFoundError(f"real Env RobotConfig not found: {candidate}")
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
