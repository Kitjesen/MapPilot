"""Pure threshold observations, never task completion or industrial safety limits.

Thermal ROIs are half-open pixel rectangles (x0, y0, x1, y1). Equality with a
configured threshold is an anomaly. Simulation declarations remain provenance,
not evidence of calibrated hardware or a semantic fault diagnosis.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass
from math import isfinite
from numbers import Real
from types import MappingProxyType
from typing import Any, Literal

import numpy as np

Verdict = Literal["normal", "anomaly", "inconclusive"]


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) for key in value):
            raise ValueError("details require string keys")
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, Real) and isfinite(value):
        return float(value)
    raise ValueError("details require finite JSON-shaped values")


@dataclass(frozen=True)
class Assessment:
    """Immutable observation; unavailable statistics are None, never fake zero."""

    verdict: Verdict
    reason: str
    metrics: Mapping[str, Any]
    rule_details: Mapping[str, Any]
    provenance: Mapping[str, Any]
    calibration: Mapping[str, Any]

    def __post_init__(self) -> None:
        for name in ("metrics", "rule_details", "provenance", "calibration"):
            object.__setattr__(self, name, _freeze(getattr(self, name)))

    def to_dict(self) -> dict[str, Any]:
        """Return independent JSON-shaped details for the caller's finding.

        The caller still owns sensor/evidence binding, finding construction and
        persistence. This method does not import or call the evidence store.
        """
        return {name: _thaw(getattr(self, name)) for name in (
            "verdict", "reason", "metrics", "rule_details", "provenance", "calibration",
        )}


def _thaw(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {key: _thaw(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw(item) for item in value]
    return value


@dataclass(frozen=True)
class ThermalRule:
    """Caller-defined maximum K rule with explicit calibration/model identity."""

    max_kelvin: float
    roi: tuple[int, int, int, int]
    min_valid_coverage: float
    temperature_kind: Literal["apparent", "corrected"]
    calibration_id: str
    model_id: str
    model_version: str

    def __post_init__(self) -> None:
        _validate_common(self)
        _nonnegative("max_kelvin", self.max_kelvin)
        if self.temperature_kind not in ("apparent", "corrected"):
            raise ValueError("temperature_kind must be apparent or corrected")
        if (not isinstance(self.roi, (tuple, list)) or len(self.roi) != 4
                or any(type(value) is not int or value < 0 for value in self.roi)
                or self.roi[2] <= self.roi[0] or self.roi[3] <= self.roi[1]):
            raise ValueError("roi must be a nonempty half-open integer rectangle")
        object.__setattr__(self, "roi", tuple(self.roi))


@dataclass(frozen=True)
class AcousticRule:
    """Caller-defined RMS Pa rule, window duration and per-channel coverage."""

    rms_pa: float
    channels: tuple[int, ...]
    min_duration_s: float
    min_valid_coverage: float
    calibration_id: str
    model_id: str
    model_version: str

    def __post_init__(self) -> None:
        _validate_common(self)
        _nonnegative("rms_pa", self.rms_pa)
        _nonnegative("min_duration_s", self.min_duration_s)
        if self.min_duration_s == 0:
            raise ValueError("min_duration_s must be positive")
        if (not isinstance(self.channels, (tuple, list)) or not self.channels
                or any(type(channel) is not int or channel < 0 for channel in self.channels)
                or len(set(self.channels)) != len(self.channels)):
            raise ValueError("channels must be distinct nonnegative integer indices")
        object.__setattr__(self, "channels", tuple(self.channels))


def _nonnegative(name: str, value: Any) -> None:
    if isinstance(value, bool) or not isinstance(value, Real) or not isfinite(value) or value < 0:
        raise ValueError(f"{name} must be finite and nonnegative")


def _validate_common(rule: Any) -> None:
    _nonnegative("min_valid_coverage", rule.min_valid_coverage)
    if not 0 < rule.min_valid_coverage <= 1:
        raise ValueError("min_valid_coverage must be in (0, 1]")
    for name in ("calibration_id", "model_id", "model_version"):
        value = getattr(rule, name)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{name} must be nonempty")


def _metadata_reason(rule: Any, calibration: Any, provenance: Any) -> str | None:
    calibration, provenance = _metadata_snapshot(calibration), _metadata_snapshot(provenance)
    if calibration.get("calibration_id") != rule.calibration_id:
        return "calibration_missing_or_mismatched"
    if (provenance.get("model_id") != rule.model_id
            or provenance.get("model_version") != rule.model_version
            or not isinstance(provenance.get("qualification"), str)
            or not provenance["qualification"].strip()):
        return "provenance_missing_or_mismatched"
    return None


def _metadata_snapshot(value: Any) -> Mapping[str, Any]:
    try:
        return _freeze(value) if isinstance(value, Mapping) else {}
    except ValueError:
        return {}


def _assessment(rule: Any, verdict: Verdict, reason: str, metrics: dict,
                provenance: Any, calibration: Any, unit: str) -> Assessment:
    return Assessment(
        verdict, reason, metrics, asdict(rule) | {"unit": unit, "comparison": ">="},
        _metadata_snapshot(provenance), _metadata_snapshot(calibration),
    )


def _arrays(raw: Any, valid_mask: Any) -> tuple[np.ndarray, np.ndarray] | None:
    try:
        values, mask = np.asarray(raw), np.asarray(valid_mask)
        if (values.ndim != 2 or not all(values.shape) or values.dtype.kind not in "iuf"
                or mask.dtype.kind != "b" or mask.shape != values.shape):
            return None
        values = values.astype(np.float64)
        return values, mask & np.isfinite(values)
    except (TypeError, ValueError, OverflowError):
        return None


def evaluate_thermal(
    kelvin: Any, valid_mask: Any, rule: ThermalRule, *, unit: str | None = None,
    temperature_kind: str | None = None, calibration: Mapping[str, Any] | None = None,
    provenance: Mapping[str, Any] | None = None,
) -> Assessment:
    """Compare the maximum valid Kelvin value in the rule's ROI using >=.

    Require unit='K', matching apparent/corrected temperature_kind, calibration
    with calibration_id, and provenance with model_id/model_version/qualification.
    Nonfinite or negative Kelvin values are invalid even if their mask is true.
    Coverage is valid pixels / ROI pixels; below minimum remains inconclusive,
    even when the available pixels exceed the threshold. No correction is applied.
    Metadata must be finite JSON-shaped values; simulation declarations and model
    limitations are retained without claiming physical calibration.
    """
    reason = _metadata_reason(rule, calibration, provenance)
    if unit != "K":
        reason = "unit_missing_or_mismatched"
    elif temperature_kind != rule.temperature_kind:
        reason = "temperature_kind_missing_or_mismatched"
    if reason:
        return _assessment(rule, "inconclusive", reason, {}, provenance, calibration, "K")
    arrays = _arrays(kelvin, valid_mask)
    if arrays is None:
        return _assessment(rule, "inconclusive", "invalid_array_or_mask", {}, provenance, calibration, "K")
    frame, mask = arrays
    x0, y0, x1, y1 = rule.roi
    if x1 > frame.shape[1] or y1 > frame.shape[0]:
        return _assessment(rule, "inconclusive", "roi_out_of_bounds", {}, provenance, calibration, "K")
    values = frame[y0:y1, x0:x1]
    valid = mask[y0:y1, x0:x1] & (values >= 0)
    selected = values[valid]
    maximum = float(np.max(selected)) if selected.size else None
    mean = float(np.mean(selected / maximum) * maximum) if maximum else maximum
    coverage = float(np.mean(valid))
    metrics = {"max_kelvin": maximum, "mean_kelvin": mean,
               "valid_coverage": coverage, "valid_pixels": int(selected.size)}
    if maximum is None or coverage < rule.min_valid_coverage:
        return _assessment(rule, "inconclusive", "insufficient_valid_coverage", metrics, provenance, calibration, "K")
    return _assessment(rule,
        "anomaly" if maximum >= rule.max_kelvin else "normal", "threshold_evaluated",
        metrics,
        provenance, calibration, "K",
    )


def evaluate_acoustic(
    pressure_pa: Any, valid_mask: Any, rule: AcousticRule, *, fs: float | None = None,
    unit: str | None = None, calibration: Mapping[str, Any] | None = None,
    provenance: Mapping[str, Any] | None = None,
) -> Assessment:
    """Evaluate unweighted, non-DC-subtracted RMS Pa per selected channel.

    Input is channels x samples. Duration is the original sample window N/fs;
    validity is checked per channel, never pooled across channels. Any selected
    channel at or above threshold is anomalous, provided all quality gates pass.
    This is neither dBA nor evidence of a semantic equipment fault.
    """
    reason = _metadata_reason(rule, calibration, provenance)
    if unit != "Pa":
        reason = "unit_missing_or_mismatched"
    if reason:
        return _assessment(rule, "inconclusive", reason, {}, provenance, calibration, "Pa")
    if isinstance(fs, bool) or not isinstance(fs, Real) or not isfinite(fs) or fs <= 0:
        return _assessment(rule, "inconclusive", "invalid_sample_rate", {}, provenance, calibration, "Pa")
    arrays = _arrays(pressure_pa, valid_mask)
    if arrays is None:
        return _assessment(rule, "inconclusive", "invalid_array_or_mask", {}, provenance, calibration, "Pa")
    frame, valid = arrays
    if max(rule.channels) >= frame.shape[0]:
        return _assessment(rule, "inconclusive", "channels_unavailable", {}, provenance, calibration, "Pa")
    values, mask = frame[list(rule.channels)], valid[list(rule.channels)]
    rms = tuple(_rms(channel[usable]) for channel, usable in zip(values, mask))
    coverage = tuple(float(np.mean(usable)) for usable in mask)
    duration = values.shape[1] / fs
    metrics = {"rms_pa": rms, "duration_s": duration, "valid_coverage": coverage,
               "valid_samples": tuple(int(np.count_nonzero(usable)) for usable in mask)}
    if not isfinite(duration) or duration < rule.min_duration_s:
        return _assessment(rule, "inconclusive", "insufficient_duration", metrics, provenance, calibration, "Pa")
    if any(value is None for value in rms) or min(coverage) < rule.min_valid_coverage:
        return _assessment(rule, "inconclusive", "insufficient_valid_coverage", metrics, provenance, calibration, "Pa")
    return _assessment(
        rule, "anomaly" if max(rms) >= rule.rms_pa else "normal", "threshold_evaluated",
        metrics, provenance, calibration, "Pa",
    )


def _rms(values: np.ndarray) -> float | None:
    if not values.size:
        return None
    scale = float(np.max(np.abs(values)))
    return float(np.sqrt(np.mean((values / scale) ** 2)) * scale) if scale else 0.0
