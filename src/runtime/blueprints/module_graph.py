"""Serializable Module graph exported from a Blueprint.

Blueprint remains the runtime builder. ModuleGraph is the declaration contract
used by validators, profile snapshots, deployment planning, and future
portable runtime code generation.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True, order=True)
class ModuleSpec:
    """One declared Module slot in a ModuleGraph."""

    name: str
    module_class: str
    module_path: str
    alias: str | None = None
    config: Mapping[str, Any] = field(default_factory=dict, compare=False)
    preinstantiated: bool = False
    declared_ports: tuple[str, ...] = ()
    _module_cls: Any = field(default=None, compare=False, repr=False)
    _instance: Any = field(default=None, compare=False, repr=False)

    def declares_port(self, port_name: str) -> bool:
        """Return whether the Module declares a port with this name."""

        if port_name in self.declared_ports:
            return True
        if self._instance is not None:
            return port_name in getattr(self._instance, "all_ports", {})
        if self._module_cls is not None:
            return hasattr(self._module_cls, port_name)
        return False

    def to_manifest(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "module_class": self.module_class,
            "module_path": self.module_path,
            "alias": self.alias,
            "config": _json_ready(self.config),
            "preinstantiated": self.preinstantiated,
            "declared_ports": list(self.declared_ports),
        }


@dataclass(frozen=True, order=True)
class GraphWireSpec:
    """One explicit Out-to-In connection in a ModuleGraph."""

    out_module: str
    out_port: str
    in_module: str
    in_port: str
    transport: str | None = None
    topic: str | None = None

    def as_snapshot(self) -> str:
        wire = f"{self.out_module}.{self.out_port}->{self.in_module}.{self.in_port}"
        if self.transport:
            wire = f"{wire}[{self.transport}]"
        if self.topic:
            wire = f"{wire}@{self.topic}"
        return wire

    def to_manifest(self) -> dict[str, Any]:
        return {
            "out_module": self.out_module,
            "out_port": self.out_port,
            "in_module": self.in_module,
            "in_port": self.in_port,
            "transport": self.transport,
            "topic": self.topic,
        }


@dataclass(frozen=True)
class ModuleGraph:
    """Portable graph declaration exported by a Blueprint."""

    profile: str | None
    modules: tuple[ModuleSpec, ...]
    explicit_wires: tuple[GraphWireSpec, ...]
    auto_wire: bool = False
    global_config: Mapping[str, Any] = field(default_factory=dict)
    swap_config: Mapping[str, Any] | None = None

    @property
    def module_names(self) -> tuple[str, ...]:
        return tuple(module.name for module in self.modules)

    def module_spec(self, name: str) -> ModuleSpec | None:
        for module in self.modules:
            if module.name == name:
                return module
        return None

    def dangling_wires(self) -> tuple[GraphWireSpec, ...]:
        module_set = set(self.module_names)
        return tuple(
            wire
            for wire in self.explicit_wires
            if wire.out_module not in module_set or wire.in_module not in module_set
        )

    def as_snapshot(self) -> dict[str, list[str]]:
        return {
            "modules": sorted(self.module_names),
            "explicit_wires": sorted(wire.as_snapshot() for wire in self.explicit_wires),
        }

    def to_manifest(self) -> dict[str, Any]:
        return {
            "schema_version": "lingtu.module_graph.v1",
            "profile": self.profile,
            "modules": [module.to_manifest() for module in self.modules],
            "explicit_wires": [wire.to_manifest() for wire in self.explicit_wires],
            "auto_wire": self.auto_wire,
            "global_config": _json_ready(self.global_config),
            "swap_config": _json_ready(self.swap_config) if self.swap_config is not None else None,
        }

    @classmethod
    def from_blueprint(cls, blueprint: Any, *, profile: str | None = None) -> "ModuleGraph":
        """Export a ModuleGraph from a Blueprint-like object."""

        modules = tuple(
            ModuleSpec(
                name=entry.name,
                module_class=entry.module_cls.__name__,
                module_path=entry.module_cls.__module__,
                alias=entry.alias,
                config=dict(entry.config),
                preinstantiated=entry.instance is not None,
                declared_ports=_declared_ports(entry),
                _module_cls=entry.module_cls,
                _instance=entry.instance,
            )
            for entry in getattr(blueprint, "_entries", ())
        )
        explicit_wires = tuple(
            GraphWireSpec(
                out_module=wire.out_module,
                out_port=wire.out_port,
                in_module=wire.in_module,
                in_port=wire.in_port,
                transport=_transport_name(getattr(wire, "transport", None)),
                topic=_topic_name(getattr(wire, "topic", None)),
            )
            for wire in getattr(blueprint, "_wires", ())
        )
        return cls(
            profile=profile,
            modules=modules,
            explicit_wires=explicit_wires,
            auto_wire=bool(getattr(blueprint, "_auto_wired", False)),
            global_config=dict(getattr(blueprint, "_global_cfg", {})),
            swap_config=_optional_dict(getattr(blueprint, "_swap_config", None)),
        )


def _optional_dict(value: Any) -> dict[str, Any] | None:
    if value is None:
        return None
    if isinstance(value, Mapping):
        return dict(value)
    return {"value": value}


def _declared_ports(entry: Any) -> tuple[str, ...]:
    ports: list[str] = []
    if getattr(entry, "instance", None) is not None:
        ports.extend(getattr(entry.instance, "all_ports", {}).keys())
    module_cls = getattr(entry, "module_cls", None)
    if module_cls is not None:
        for cls in getattr(module_cls, "__mro__", (module_cls,)):
            ports.extend(getattr(cls, "__annotations__", {}).keys())
    return tuple(sorted(dict.fromkeys(str(port) for port in ports if port)))


def _transport_name(transport: Any) -> str | None:
    if transport is None:
        return None
    if isinstance(transport, str):
        return transport
    return getattr(transport, "backend_name", type(transport).__name__)


def _topic_name(topic: Any) -> str | None:
    if topic is None:
        return None
    value = str(topic).strip()
    return value or None


def _json_ready(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_json_ready(item) for item in value]
    if isinstance(value, list):
        return [_json_ready(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return repr(value)
