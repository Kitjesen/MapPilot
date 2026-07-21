"""Resolve product intent into a deterministic endpoint process plan."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any, Mapping

from .loader import RuntimeGraph, load_runtime_graph

PROCESS_LIFECYCLES = frozenset({"mode", "persistent"})
_PROCESS_NAME = re.compile(r"[a-z][a-z0-9_]*\Z")
_SYSTEMD_TARGET = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.@:-]*\.service\Z")


@dataclass(frozen=True)
class RuntimeProcess:
    """One logical process resolved to an endpoint deployment service."""

    name: str
    manager: str
    target: str
    order: int
    timeout_s: int
    lifecycle: str

    def as_dict(self) -> dict[str, Any]:
        """Return the stable JSON representation consumed by launchers."""

        return {
            "name": self.name,
            "manager": self.manager,
            "target": self.target,
            "order": self.order,
            "timeout_s": self.timeout_s,
            "lifecycle": self.lifecycle,
        }


@dataclass(frozen=True)
class RuntimePlan:
    """Executable process plan for one product on one endpoint."""

    product: str
    endpoint: str
    processes: tuple[RuntimeProcess, ...]
    available_processes: tuple[RuntimeProcess, ...] = ()
    conflicts: tuple[str, ...] = ()

    @property
    def managed_processes(self) -> tuple[RuntimeProcess, ...]:
        """Return processes restarted as part of a product mode switch."""

        return tuple(process for process in self.processes if process.lifecycle == "mode")

    @property
    def persistent_processes(self) -> tuple[RuntimeProcess, ...]:
        """Return processes kept alive across product mode switches."""

        return tuple(process for process in self.processes if process.lifecycle == "persistent")

    def has_process(self, name: str) -> bool:
        """Return whether the selected product requires *name*."""

        return any(process.name == name for process in self.processes)

    def process(self, name: str) -> RuntimeProcess:
        """Return one selected process or raise ``KeyError``."""

        for process in self.processes:
            if process.name == name:
                return process
        raise KeyError(name)

    def as_dict(self) -> dict[str, Any]:
        """Return the machine-readable runtime plan contract."""

        return {
            "schema_version": "lingtu.runtime_plan.v1",
            "product": self.product,
            "endpoint": self.endpoint,
            "processes": [process.as_dict() for process in self.processes],
            "known_targets": [process.target for process in self.available_processes],
            "stop_targets": list(self.stop_targets),
        }

    @property
    def stop_targets(self) -> tuple[str, ...]:
        """Return targets stopped before applying this product plan."""

        managed = tuple(
            process.target
            for process in reversed(self.available_processes)
            if process.lifecycle == "mode"
        )
        return tuple(dict.fromkeys((*managed, *self.conflicts)))


def build_runtime_plan(
    product: str,
    endpoint: str,
    *,
    graph: RuntimeGraph | None = None,
) -> RuntimePlan:
    """Resolve explicit product process names against an endpoint mapping."""

    graph = graph or load_runtime_graph()
    product_spec = graph.products.get(product)
    if product_spec is None:
        raise ValueError(f"unknown Runtime Graph product: {product}")
    endpoint_spec = graph.endpoints.get(endpoint)
    if endpoint_spec is None:
        raise ValueError(f"unknown Runtime Graph endpoint: {endpoint}")
    process_control = str(endpoint_spec.get("process_control") or "").strip()
    if process_control != "runtime_plan":
        owner = process_control or "unspecified"
        raise ValueError(
            f"endpoint {endpoint} is not RuntimePlan-managed; process control is {owner}"
        )

    names = _process_names(product_spec.get("processes"), product=product)
    definitions = endpoint_spec.get("processes")
    if not isinstance(definitions, Mapping):
        raise ValueError(f"endpoint {endpoint} has no process mapping")

    available = tuple(
        sorted(
            (
                _resolve_process(
                    name,
                    value,
                    endpoint=endpoint,
                    manager=str(endpoint_spec.get("process_manager") or "").strip(),
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
    manager = str(endpoint_spec.get("process_manager") or "").strip()
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
    return RuntimePlan(
        product=product,
        endpoint=endpoint,
        processes=tuple(process for process in available if process.name in names),
        available_processes=available,
        conflicts=conflicts,
    )


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
) -> RuntimeProcess:
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
    order = raw_order
    timeout_s = raw_timeout
    if order < 0 or timeout_s <= 0:
        raise ValueError(
            f"endpoint {endpoint} process {name} has invalid order or timeout_s"
        )
    return RuntimeProcess(
        name=name,
        manager=manager,
        target=target,
        order=order,
        timeout_s=timeout_s,
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
