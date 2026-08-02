"""Module graph wire declarations and delivery resolution."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

from runtime.transport.local import LocalTransport, Transport

if TYPE_CHECKING:
    from runtime.blueprint import Blueprint


class WireDelivery(str, Enum):
    """Supported delivery modes for one Module Out-to-In wire."""

    CALLBACK = "callback"
    LOCAL = "local"
    DDS = "dds"
    SHM = "shm"


@dataclass(frozen=True)
class WireSpec:
    """Connect one output port to one input port."""

    out_module: str
    out_port: str
    in_module: str
    in_port: str
    delivery: Any = None
    topic: str | None = None

    @property
    def delivery_spec(self) -> Any:
        """Return the effective delivery selector."""

        return self.delivery

    def label(self) -> str:
        label = f"{self.out_module}.{self.out_port}->{self.in_module}.{self.in_port}"
        if self.topic:
            label = f"{label}@{self.topic}"
        return label

    def apply(self, bp: "Blueprint") -> None:
        bp.wire(
            self.out_module,
            self.out_port,
            self.in_module,
            self.in_port,
            delivery=self.delivery,
            topic=self.topic,
        )


def wire_key(spec: WireSpec) -> tuple[str, str, str, str]:
    return (spec.out_module, spec.out_port, spec.in_module, spec.in_port)


def resolve_wire_delivery(spec: Any) -> Transport | None:
    """Resolve a delivery spec string or instance to a transport object."""

    if spec is None:
        return None
    if isinstance(spec, WireDelivery):
        spec = spec.value
    if isinstance(spec, str):
        normalized = spec.strip().lower()
        if normalized in {"", WireDelivery.CALLBACK.value}:
            return None
        if normalized in {WireDelivery.DDS.value, WireDelivery.SHM.value}:
            from runtime.transport.factory import create_transport_adapter

            return create_transport_adapter(normalized)
        if normalized == WireDelivery.LOCAL.value:
            return LocalTransport()
        raise ValueError(f"Unknown wire delivery spec: '{spec}'")
    return spec


def wire_delivery_cache_key(spec: Any) -> tuple[str, str]:
    """Return a stable per-build key for a delivery specification."""

    if isinstance(spec, WireDelivery):
        return ("name", spec.value)
    if isinstance(spec, str):
        return ("name", spec.strip().lower())
    return ("object", str(id(spec)))


def wire_delivery_name(spec_or_transport: Any) -> str:
    """Return a readable delivery/backend name for diagnostics."""

    if spec_or_transport is None:
        return WireDelivery.CALLBACK.value
    if isinstance(spec_or_transport, WireDelivery):
        return spec_or_transport.value
    if isinstance(spec_or_transport, str):
        return spec_or_transport.strip().lower()
    name = getattr(spec_or_transport, "backend_name", None) or getattr(
        spec_or_transport,
        "name",
        None,
    )
    return str(name or type(spec_or_transport).__name__)


def explicit_wire_topic(spec: WireSpec) -> str | None:
    if spec.topic is None:
        return None
    topic = str(spec.topic).strip()
    if not topic:
        raise ValueError(
            "wire(): topic cannot be empty for "
            f"{spec.out_module}.{spec.out_port}->{spec.in_module}.{spec.in_port}"
        )
    return topic


def default_wire_topic(spec: WireSpec) -> str:
    return explicit_wire_topic(spec) or f"/{spec.out_module}/{spec.out_port}"
