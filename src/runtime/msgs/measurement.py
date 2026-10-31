"""Typed Host measurements; source identity is never a receipt/epoch clock."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Any, ClassVar

import numpy as np


def _finite_seconds(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be finite numeric seconds")
    try:
        seconds = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be finite numeric seconds") from exc
    if not isfinite(seconds):
        raise ValueError(f"{name} must be finite numeric seconds")
    return seconds


def _captured_at_s(metadata: dict[str, Any]) -> float | None:
    frame = metadata.get("frame_metadata", {})
    if not isinstance(frame, dict):
        raise ValueError("frame_metadata must be an object")
    clock = frame.get("source_clock", {})
    if not isinstance(clock, dict):
        raise ValueError("source_clock must be an object")
    if "mapped_epoch_s" not in clock:
        return None
    return _finite_seconds(clock["mapped_epoch_s"], "mapped_epoch_s")


def _validate_clocks(received_at_s: float, metadata: dict[str, Any]) -> None:
    if _finite_seconds(received_at_s, "received_at_s") < 0:
        raise ValueError("received_at_s must be nonnegative")
    _captured_at_s(metadata)


@dataclass(frozen=True)
class MeasurementIdentity:
    """Exact source identity (native IDL sample_sequence maps to sequence)."""

    product_session_id: str
    session_id: str
    instance_id: str
    sensor_id: str
    frame_id: str
    model_generation: int
    reset_generation: int
    sequence: int
    sim_time_ns: int


@dataclass(frozen=True)
class ThermalMeasurement:
    """Kelvin image, not a depth encoding or a hardware-calibration claim.

    Unmapped frames are displayable. Inspection must require captured_at_s
    before comparing this observation to RGB/epoch timestamps.
    """

    msg_name: ClassVar[str] = "measurement.ThermalMeasurement"
    identity: MeasurementIdentity
    temperature_k: np.ndarray
    valid_mask: np.ndarray
    metadata: dict[str, Any]
    received_at_s: float

    def __post_init__(self) -> None:
        _validate_clocks(self.received_at_s, self.metadata)

    @property
    def captured_at_s(self) -> float | None:
        """Only an explicit source-clock mapping can supply observation time."""
        return _captured_at_s(self.metadata)


@dataclass(frozen=True)
class AcousticMeasurement:
    """Channel-major pressure in Pa and optional calibrated voltage in V."""

    msg_name: ClassVar[str] = "measurement.AcousticMeasurement"
    identity: MeasurementIdentity
    pressure_pa: np.ndarray
    valid_mask: np.ndarray
    sample_rate_hz: int
    voltage_v: np.ndarray | None
    metadata: dict[str, Any]
    received_at_s: float

    def __post_init__(self) -> None:
        _validate_clocks(self.received_at_s, self.metadata)

    @property
    def captured_at_s(self) -> float | None:
        """None means display-only until the source clock is explicitly mapped."""
        return _captured_at_s(self.metadata)


@dataclass(frozen=True)
class SemanticMeasurement:
    """Physical class/instance labels, never detector predictions."""

    msg_name: ClassVar[str] = "measurement.SemanticMeasurement"
    identity: MeasurementIdentity
    class_ids: np.ndarray
    instance_ids: np.ndarray
    valid_mask: np.ndarray
    metadata: dict[str, Any]
    received_at_s: float

    def __post_init__(self) -> None:
        _validate_clocks(self.received_at_s, self.metadata)

    @property
    def captured_at_s(self) -> float | None:
        """Only an explicit mapping permits RGB/epoch timestamp comparison."""
        return _captured_at_s(self.metadata)
