"""Shared wire specification primitives."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from runtime.blueprint import Blueprint


@dataclass(frozen=True)
class WireSpec:
    """Connect one output port to one input port in a Blueprint."""

    out_module: str
    out_port: str
    in_module: str
    in_port: str
    transport: Any = None
    topic: str | None = None

    def label(self) -> str:
        label = (
            f"{self.out_module}.{self.out_port}"
            f"->{self.in_module}.{self.in_port}"
        )
        if self.topic:
            label = f"{label}@{self.topic}"
        return label

    def apply(self, bp: Blueprint) -> None:
        bp.wire(
            self.out_module,
            self.out_port,
            self.in_module,
            self.in_port,
            transport=self.transport,
            topic=self.topic,
        )


def wire_key(spec: WireSpec) -> tuple[str, str, str, str]:
    return (spec.out_module, spec.out_port, spec.in_module, spec.in_port)


def wire_present_specs(bp: Blueprint, specs: tuple[WireSpec, ...]) -> None:
    names = {entry.name for entry in bp._entries}
    existing = {wire_key(wire) for wire in bp._wires}
    for spec in specs:
        if wire_key(spec) in existing:
            continue
        if (
            spec.out_module in names
            and spec.in_module in names
            and _module_declares(bp, spec.out_module, spec.out_port)
            and _module_declares(bp, spec.in_module, spec.in_port)
        ):
            spec.apply(bp)
            existing.add(wire_key(spec))


def _module_declares(bp: Blueprint, module_name: str, port_name: str) -> bool:
    for entry in bp._entries:
        if entry.name == module_name:
            return hasattr(entry.module_cls, port_name)
    return False
