"""Runtime route contract data model."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping


class RouteBackend(str, Enum):
    """Supported route backends for canonical runtime topics."""

    LOCAL = "local"
    DDS = "dds"
    SHM = "shm"
    LCM = "lcm"
    ROS2 = "ros2"


@dataclass(frozen=True)
class PortBinding:
    """One runtime endpoint or Module port that publishes or consumes a topic."""

    owner: str
    port: str
    direction: str
    boundary: str = "module"
    required: bool = True

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> PortBinding:
        return cls(
            owner=str(data.get("owner") or data.get("module") or "").strip(),
            port=str(data.get("port") or "").strip(),
            direction=str(data.get("direction") or "").strip().lower(),
            boundary=str(data.get("boundary") or "module").strip().lower(),
            required=bool(data.get("required", True)),
        )

    def to_manifest(self) -> dict[str, Any]:
        return {
            "owner": self.owner,
            "port": self.port,
            "direction": self.direction,
            "boundary": self.boundary,
            "required": self.required,
        }


@dataclass(frozen=True)
class TopicContract:
    """One canonical runtime topic."""

    topic: str
    role: str
    schema: str
    frame: str
    producer: str
    consumers: tuple[str, ...] = ()
    port_bindings: tuple[PortBinding, ...] = ()
    real_equivalent_required: bool = False
    external_diagnostics_subscribable: bool = False

    @classmethod
    def from_mapping(cls, topic: str, data: Mapping[str, Any]) -> TopicContract:
        consumers = data.get("consumers") or ()
        if isinstance(consumers, str):
            consumers = (consumers,)
        raw_port_bindings = data.get("port_bindings") or data.get("blueprint_bindings") or ()
        if not isinstance(raw_port_bindings, list | tuple):
            raw_port_bindings = ()
        return cls(
            topic=str(topic),
            role=str(data.get("role") or ""),
            schema=str(data.get("schema") or ""),
            frame=str(data.get("frame") or "none"),
            producer=str(data.get("producer") or ""),
            consumers=tuple(str(item) for item in consumers),
            port_bindings=tuple(
                PortBinding.from_mapping(item) for item in raw_port_bindings if isinstance(item, Mapping)
            ),
            real_equivalent_required=bool(data.get("real_equivalent_required", False)),
            external_diagnostics_subscribable=bool(data.get("external_diagnostics_subscribable", False)),
        )

    def to_manifest(self) -> dict[str, Any]:
        return {
            "topic": self.topic,
            "role": self.role,
            "schema": self.schema,
            "frame": self.frame,
            "producer": self.producer,
            "consumers": list(self.consumers),
            "port_bindings": [binding.to_manifest() for binding in self.port_bindings],
            "real_equivalent_required": self.real_equivalent_required,
            "external_diagnostics_subscribable": self.external_diagnostics_subscribable,
        }


@dataclass(frozen=True)
class RouteSpec:
    """Transport selection for one runtime route."""

    name: str
    default: str = RouteBackend.LOCAL.value
    description: str = ""
    endpoint_contract: str | None = None
    routes: Mapping[str, str] = field(default_factory=dict)
    bindings: Mapping[str, Mapping[str, Mapping[str, Any]]] = field(default_factory=dict)

    def backend_for(self, topic: str) -> str:
        return str(self.routes.get(topic, self.default)).strip().lower()

    def binding_for(self, backend: str, topic: str) -> Mapping[str, Any]:
        by_backend = self.bindings.get(str(backend).strip().lower()) or {}
        return by_backend.get(str(topic), {})

    def to_manifest(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "default": self.default,
            "endpoint_contract": self.endpoint_contract,
            "routes": dict(self.routes),
            "bindings": {
                str(backend): {str(topic): dict(binding) for topic, binding in topic_bindings.items()}
                for backend, topic_bindings in self.bindings.items()
            },
        }


@dataclass(frozen=True)
class RouteContract:
    """Runtime topics plus one selected route."""

    name: str
    topics: Mapping[str, TopicContract]
    route: RouteSpec
    native_contract_topics: tuple[str, ...] = ()

    def topic(self, topic: str) -> TopicContract:
        return self.topics[str(topic)]

    def route_for(self, topic: str) -> str:
        return self.route.backend_for(str(topic))

    def binding_for(self, topic: str) -> Mapping[str, Any]:
        backend = self.route_for(topic)
        return self.route.binding_for(backend, str(topic))

    def to_manifest(self) -> dict[str, Any]:
        return {
            "schema_version": "lingtu.route_contract.v1",
            "name": self.name,
            "route": self.route.to_manifest(),
            "native_contract_topics": list(self.native_contract_topics),
            "topics": {topic: spec.to_manifest() for topic, spec in sorted(self.topics.items())},
        }
