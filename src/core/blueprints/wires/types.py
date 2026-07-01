"""Shared wire specification primitives."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from core.blueprint import Blueprint


@dataclass(frozen=True)
class WireSpec:
    """Connect one output port to one input port in a Blueprint."""

    out_module: str
    out_port: str
    in_module: str
    in_port: str
    transport: Any = None

    def label(self) -> str:
        return (
            f"{self.out_module}.{self.out_port}"
            f"->{self.in_module}.{self.in_port}"
        )

    def apply(self, bp: Blueprint) -> None:
        bp.wire(
            self.out_module,
            self.out_port,
            self.in_module,
            self.in_port,
            transport=self.transport,
        )


def wire_key(spec: WireSpec) -> tuple[str, str, str, str]:
    return (spec.out_module, spec.out_port, spec.in_module, spec.in_port)
