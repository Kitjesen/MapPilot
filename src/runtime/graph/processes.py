"""Resolve product process roles against one deployment endpoint."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping

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
    application: str | None = None
    config: Mapping[str, Any] = field(default_factory=dict, hash=False)

    def __post_init__(self) -> None:
        """Normalize optional application metadata into immutable JSON data."""

        if self.application is not None:
            application = str(self.application).strip()
            if _PROCESS_NAME.fullmatch(application) is None:
                raise ValueError(f"invalid process application {self.application!r}")
            object.__setattr__(self, "application", application)
        object.__setattr__(self, "config", _immutable_json_mapping(self.config))

    def as_dict(self) -> dict[str, Any]:
        """Return the stable representation consumed by Launcher."""

        payload = {
            "name": self.name,
            "manager": self.manager,
            "target": self.target,
            "order": self.order,
            "timeout_s": self.timeout_s,
            "lifecycle": self.lifecycle,
        }
        if self.application is not None:
            payload["application"] = self.application
        if self.config:
            payload["config"] = _mutable_json_value(self.config)
        return payload


def resolve_processes(
    product: str,
    endpoint: str,
    *,
    graph: RuntimeGraph | None = None,
) -> tuple[tuple[ProcessSpec, ...], tuple[ProcessSpec, ...], tuple[str, ...]]:
    """Return selected processes, all endpoint processes, and conflict targets."""

    graph = graph or load_runtime_graph()
    product_spec = graph.products.get(product)
    if product_spec is None:
        raise ValueError(f"unknown Runtime Graph product: {product}")
    endpoint_spec = graph.endpoints.get(endpoint)
    if endpoint_spec is None:
        raise ValueError(f"unknown Runtime Graph endpoint: {endpoint}")
    process_control = str(endpoint_spec.get("process_control") or "").strip()
    if process_control != "launcher":
        owner = process_control or "unspecified"
        raise ValueError(
            f"endpoint {endpoint} is not Launcher-managed; process control is {owner}"
        )

    names = _process_names(product_spec.get("processes"), product=product)
    definitions = endpoint_spec.get("processes")
    if not isinstance(definitions, Mapping):
        raise ValueError(f"endpoint {endpoint} has no process mapping")

    manager = str(endpoint_spec.get("process_manager") or "").strip()
    available = tuple(
        sorted(
            (
                _resolve_process(
                    name,
                    value,
                    endpoint=endpoint,
                    manager=manager,
                )
                for name, value in definitions.items()
            ),
            key=lambda process: (process.order, process.name),
        )
    )
    orders = [process.order for process in available]
    if len(set(orders)) != len(orders):
        raise ValueError(f"endpoint {endpoint} has duplicate process order values")
    targets = [process.target for process in available]
    if len(set(targets)) != len(targets):
        raise ValueError(f"endpoint {endpoint} has duplicate process targets")

    by_name = {process.name: process for process in available}
    missing = sorted(set(names) - set(by_name))
    if missing:
        raise ValueError(
            f"endpoint {endpoint} does not map processes: {', '.join(missing)}"
        )

    conflicts = _target_names(
        endpoint_spec.get("conflicts"),
        endpoint=endpoint,
        manager=manager,
    )
    overlap = sorted(set(targets) & set(conflicts))
    if overlap:
        raise ValueError(
            f"endpoint {endpoint} process targets also appear as conflicts: {', '.join(overlap)}"
        )
    selected = tuple(process for process in available if process.name in names)
    return selected, available, conflicts


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


def _target_names(value: Any, *, endpoint: str, manager: str) -> tuple[str, ...]:
    if value is None:
        return ()
    if not isinstance(value, list | tuple):
        raise ValueError(f"endpoint {endpoint} conflicts must be a list")
    targets = tuple(str(item).strip() for item in value)
    if (
        any(not _valid_target(target, manager=manager) for target in targets)
        or len(set(targets)) != len(targets)
    ):
        raise ValueError(f"endpoint {endpoint} has invalid conflict targets")
    return targets


def _resolve_process(
    name: str,
    value: Any,
    *,
    endpoint: str,
    manager: str,
) -> ProcessSpec:
    if not isinstance(value, Mapping):
        raise ValueError(f"endpoint {endpoint} does not map process {name}")
    if _PROCESS_NAME.fullmatch(name) is None:
        raise ValueError(f"endpoint {endpoint} has invalid process name {name!r}")

    if manager not in {"systemd", "direct", "external"}:
        raise ValueError(f"endpoint {endpoint} has invalid process manager {manager!r}")
    target = str(value.get("target") or "").strip()
    if not _valid_target(target, manager=manager):
        raise ValueError(f"endpoint {endpoint} process {name} has invalid target")
    lifecycle = str(value.get("lifecycle") or "").strip()
    if lifecycle not in PROCESS_LIFECYCLES:
        raise ValueError(
            f"endpoint {endpoint} process {name} has invalid lifecycle {lifecycle!r}"
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
            f"endpoint {endpoint} process {name} needs integer order and timeout_s"
        )
    if raw_order < 0 or raw_timeout <= 0:
        raise ValueError(
            f"endpoint {endpoint} process {name} has invalid order or timeout_s"
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


def _immutable_json_mapping(value: Mapping[str, Any]) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError("process config must be a JSON object")
    try:
        encoded = json.dumps(
            {str(key): item for key, item in value.items()},
            allow_nan=False,
            ensure_ascii=True,
            sort_keys=True,
        )
        normalized = json.loads(encoded)
    except (TypeError, ValueError) as exc:
        raise ValueError("process config must contain JSON data") from exc
    return _immutable_json_value(normalized)


def _immutable_json_value(value: Any) -> Any:
    if isinstance(value, dict):
        return MappingProxyType(
            {str(key): _immutable_json_value(item) for key, item in value.items()}
        )
    if isinstance(value, list):
        return tuple(_immutable_json_value(item) for item in value)
    return value


def _mutable_json_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _mutable_json_value(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_mutable_json_value(item) for item in value]
    return value
