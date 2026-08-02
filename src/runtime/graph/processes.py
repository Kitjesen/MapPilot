"""Resolve Product process roles against one deployment Env implementation."""

from __future__ import annotations

import re
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from .loader import RuntimeGraph, load_runtime_graph

PROCESS_LIFECYCLES = frozenset({"mode", "persistent"})
_PROCESS_NAME = re.compile(r"[a-z][a-z0-9_]*\Z")
_SYSTEMD_TARGET = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.@:-]*\.service\Z")


@dataclass(frozen=True)
class ProcessSpec:
    """One logical product process resolved to a deployment target."""

    name: str
    manager: str
    target: str
    order: int
    timeout_s: int
    lifecycle: str

    def as_dict(self) -> dict[str, Any]:
        """Return the stable representation stored in a RunPlan."""

        return {
            "name": self.name,
            "manager": self.manager,
            "target": self.target,
            "order": self.order,
            "timeout_s": self.timeout_s,
            "lifecycle": self.lifecycle,
        }


def resolve_processes(
    product: str,
    env: str,
    *,
    graph: RuntimeGraph | None = None,
    env_config: Mapping[str, Any] | None = None,
) -> tuple[tuple[ProcessSpec, ...], tuple[ProcessSpec, ...], tuple[str, ...]]:
    """Return selected processes, all Env processes, and conflict targets.

    Non-systemd implementations own their own acceptance or external process
    transaction, so they return no ProductControl process records.
    """

    graph = graph or load_runtime_graph()
    product_spec = graph.products.get(product)
    if product_spec is None:
        raise ValueError(f"unknown Runtime Graph product: {product}")
    implementation = resolve_env_implementation(
        env,
        graph=graph,
        env_config=env_config,
    )
    supported_products = _name_tuple(
        implementation.get("supported_products"),
        owner=f"env {env}",
        field="supported_products",
        allow_empty=True,
    )
    if product not in supported_products:
        supported = ", ".join(supported_products) or "none"
        raise ValueError(
            f"env {env} implementation does not support Product {product!r}; "
            f"supported: {supported}"
        )

    process_control = str(implementation.get("process_control") or "").strip()
    if process_control != "systemd":
        return (), (), ()

    names = _process_names(product_spec.get("processes"), product=product)
    definitions = implementation.get("processes")
    if not isinstance(definitions, Mapping):
        raise ValueError(f"env {env} has no process mapping")

    manager = str(implementation.get("process_manager") or "").strip()
    available = tuple(
        sorted(
            (
                _resolve_process(
                    name,
                    value,
                    env=env,
                    manager=manager,
                )
                for name, value in definitions.items()
            ),
            key=lambda process: (process.order, process.name),
        )
    )
    orders = [process.order for process in available]
    if len(set(orders)) != len(orders):
        raise ValueError(f"env {env} has duplicate process order values")
    targets = [process.target for process in available]
    if len(set(targets)) != len(targets):
        raise ValueError(f"env {env} has duplicate process targets")

    by_name = {process.name: process for process in available}
    missing = sorted(set(names) - set(by_name))
    if missing:
        raise ValueError(
            f"env {env} does not map processes: {', '.join(missing)}"
        )

    conflicts = _target_names(
        implementation.get("conflicts"),
        env=env,
        manager=manager,
    )
    overlap = sorted(set(targets) & set(conflicts))
    if overlap:
        raise ValueError(
            f"env {env} process targets also appear as conflicts: {', '.join(overlap)}"
        )
    selected = tuple(process for process in available if process.name in names)
    return selected, available, conflicts


def resolve_env_implementation(
    env: str,
    *,
    graph: RuntimeGraph | None = None,
    env_config: Mapping[str, Any] | None = None,
) -> Mapping[str, Any]:
    """Return the selected implementation for one public Env.

    ``real`` has one physical implementation. ``sim`` deliberately requires
    ``env_config.backend`` because no simulator backend implements every
    Product.
    """

    graph = graph or load_runtime_graph()
    env_spec = graph.envs.get(env)
    if env_spec is None:
        raise ValueError(f"unknown Runtime Graph env: {env}")

    if env != "sim":
        selected_backend = ""
        if isinstance(env_config, Mapping):
            selected_backend = str(env_config.get("backend") or "").strip()
        if selected_backend:
            raise ValueError(f"env {env} does not accept a backend selector")
        return env_spec

    if not isinstance(env_config, Mapping):
        raise ValueError("env sim requires env_config.backend")
    backend = str(env_config.get("backend") or "").strip()
    if not backend:
        raise ValueError("env sim requires env_config.backend")
    backends = env_spec.get("backends")
    if not isinstance(backends, Mapping):
        raise ValueError("env sim has no backend implementations")
    implementation = backends.get(backend)
    if not isinstance(implementation, Mapping):
        available = ", ".join(sorted(str(name) for name in backends))
        raise ValueError(
            f"unknown backend {backend!r} for env sim; available: {available}"
        )
    return implementation


def _process_names(value: Any, *, product: str) -> tuple[str, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"product {product} must declare a process list")
    names = tuple(str(item).strip() for item in value)
    invalid = tuple(name for name in names if _PROCESS_NAME.fullmatch(name) is None)
    if not names or invalid:
        raise ValueError(f"product {product} has invalid process names: {invalid}")
    if len(set(names)) != len(names):
        raise ValueError(f"product {product} has duplicate process names")
    return names


def _name_tuple(
    value: Any,
    *,
    owner: str,
    field: str,
    allow_empty: bool = False,
) -> tuple[str, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"{owner} must declare {field} as a list")
    names = tuple(str(item).strip() for item in value)
    if (not allow_empty and not names) or any(not name for name in names):
        raise ValueError(f"{owner} has invalid {field}")
    if len(set(names)) != len(names):
        raise ValueError(f"{owner} has duplicate {field}")
    return names


def _target_names(value: Any, *, env: str, manager: str) -> tuple[str, ...]:
    if value is None:
        return ()
    if not isinstance(value, list | tuple):
        raise ValueError(f"env {env} conflicts must be a list")
    targets = tuple(str(item).strip() for item in value)
    if (
        any(not _valid_target(target, manager=manager) for target in targets)
        or len(set(targets)) != len(targets)
    ):
        raise ValueError(f"env {env} has invalid conflict targets")
    return targets


def _resolve_process(
    name: str,
    value: Any,
    *,
    env: str,
    manager: str,
) -> ProcessSpec:
    if not isinstance(value, Mapping):
        raise ValueError(f"env {env} does not map process {name}")
    if _PROCESS_NAME.fullmatch(name) is None:
        raise ValueError(f"env {env} has invalid process name {name!r}")

    if manager not in {"systemd", "direct", "external"}:
        raise ValueError(f"env {env} has invalid process manager {manager!r}")
    target = str(value.get("target") or "").strip()
    if not _valid_target(target, manager=manager):
        raise ValueError(f"env {env} process {name} has invalid target")
    lifecycle = str(value.get("lifecycle") or "").strip()
    if lifecycle not in PROCESS_LIFECYCLES:
        raise ValueError(
            f"env {env} process {name} has invalid lifecycle {lifecycle!r}"
        )
    raw_order = value.get("order")
    raw_timeout = value.get("timeout_s")
    if (
        isinstance(raw_order, bool)
        or not isinstance(raw_order, int)
        or isinstance(raw_timeout, bool)
        or not isinstance(raw_timeout, int)
    ):
        raise ValueError(
            f"env {env} process {name} needs integer order and timeout_s"
        )
    if raw_order < 0 or raw_timeout <= 0:
        raise ValueError(
            f"env {env} process {name} has invalid order or timeout_s"
        )
    return ProcessSpec(
        name=name,
        manager=manager,
        target=target,
        order=raw_order,
        timeout_s=raw_timeout,
        lifecycle=lifecycle,
    )


def _valid_target(value: str, *, manager: str) -> bool:
    if not value or value.startswith("-") or "|" in value:
        return False
    if any(char.isspace() for char in value):
        return False
    if manager == "systemd":
        return _SYSTEMD_TARGET.fullmatch(value) is not None
    return True
