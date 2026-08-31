"""Opaque typed-DDS payloads captured at a simulation truth boundary."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class TypedDdsPayloadSample:
    """One serialized typed-DDS sample; DDS owns encoding, recording owns bytes."""

    topic: str
    type_name: str
    encoding: str
    model_generation: int
    reset_generation: int
    sequence: int
    sim_time_ns: int
    payload: bytes
    metadata: Mapping[str, object] | None = None


__all__ = ["TypedDdsPayloadSample"]
