"""Shared wire specification helpers."""

from __future__ import annotations

from runtime.blueprint import Blueprint
from runtime.wiring import WireSpec, wire_key


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
