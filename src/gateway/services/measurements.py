"""Read-only, latest-per-declared-sensor measurement projection.

Freshness is receipt freshness, not a calibration or epoch-alignment claim.
Raw arrays retain their units, dtype, mask, and NaNs.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from collections.abc import Collection
from dataclasses import asdict, replace
from typing import Any
from urllib.parse import quote

import numpy as np

from runtime.contracts.sensor_control import SensorControlResult
from runtime.msgs.measurement import AcousticMeasurement, SemanticMeasurement, ThermalMeasurement

Measurement = ThermalMeasurement | AcousticMeasurement | SemanticMeasurement


def finite_json(value: Any) -> Any:
    """Non-finite metadata is unknown (null), never a zero measurement."""
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(item) for item in value]
    if isinstance(value, np.ndarray):
        return finite_json(value.tolist())
    if isinstance(value, np.generic):
        return finite_json(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def measurement_arrays(sample: Measurement) -> dict[str, np.ndarray]:
    arrays = {"valid_mask": sample.valid_mask}
    if isinstance(sample, ThermalMeasurement):
        arrays["temperature_k"] = sample.temperature_k
    elif isinstance(sample, AcousticMeasurement):
        arrays["pressure_pa"] = sample.pressure_pa
        if sample.voltage_v is not None:
            arrays["voltage_v"] = sample.voltage_v
    else:
        arrays.update(class_ids=sample.class_ids, instance_ids=sample.instance_ids)
    return arrays


class LatestMeasurements:
    """Bounded by the declared sensor set; no history or discovery cache."""

    def __init__(self, sensor_ids: Collection[str], stale_timeout_s: float = 1.0) -> None:
        if isinstance(sensor_ids, str) or any(
            not isinstance(item, str) or not item or item != item.strip() or "/" in item or "\0" in item
            for item in sensor_ids
        ):
            raise ValueError("measurement_sensor_ids must contain nonempty trimmed sensor IDs without slashes")
        if isinstance(stale_timeout_s, bool) or not math.isfinite(stale_timeout_s) or stale_timeout_s <= 0:
            raise ValueError("measurement_stale_timeout_s must be finite positive seconds")
        self.sensor_ids = tuple(dict.fromkeys(sensor_ids))
        self.stale_timeout_s = float(stale_timeout_s)
        self._declared = frozenset(self.sensor_ids)
        self._latest: dict[str, tuple[Measurement, float, float]] = {}
        self._lock = threading.Lock()
        self._controls: dict[str, SensorControlResult] = {}
        self._control_seen: deque[str] = deque(maxlen=256)

    def apply_control(self, result: SensorControlResult) -> None:
        if not result.applied or result.sensor_id not in self._declared or type(result.settings.get("enabled")) is not bool:
            return
        with self._lock:
            previous = self._controls.get(result.sensor_id)
            if (result.request_id in self._control_seen
                    or (previous is not None and result.applied_sim_time_ns < previous.applied_sim_time_ns)):
                return
            record = self._latest.get(result.sensor_id)
            if record is not None:
                identity = record[0].identity
                if (result.product_session_id, result.session_id, result.model_generation, result.reset_generation) != (
                        identity.product_session_id, identity.session_id, identity.model_generation, identity.reset_generation):
                    return
            self._control_seen.append(result.request_id)
            if previous is not None and previous.settings.get("enabled") is False and result.settings["enabled"]:
                self._latest.pop(result.sensor_id, None)
            self._controls[result.sensor_id] = replace(result, settings=finite_json(result.settings))

    def observe(self, sample: Measurement) -> None:
        sensor_id = sample.identity.sensor_id
        if sensor_id not in self._declared:
            return
        # Own one stable snapshot, independent of a publisher's mutable arrays.
        arrays: dict[str, Any] = {name: array.copy() for name, array in measurement_arrays(sample).items()}
        sample = replace(sample, **arrays, metadata=finite_json(sample.metadata))
        with self._lock:
            self._latest[sensor_id] = (sample, time.monotonic(), time.time() - sample.received_at_s)

    def get(self, sensor_id: str) -> tuple[dict[str, Any], Measurement | None]:
        if sensor_id not in self._declared:
            raise KeyError(sensor_id)
        with self._lock:
            record = self._latest.get(sensor_id)
            control = self._controls.get(sensor_id)
        result: dict[str, Any] = dict(
            sensor_id=sensor_id,
            status="no_sample",
            fresh=False,
            stale=None,
            receipt_age_s=None,
            stale_timeout_s=self.stale_timeout_s,
            freshness_basis="received_at_s_then_monotonic",
            source_identity=None,
            modality=None,
            units=None,
            calibration=None,
            validity=None,
            received_at_s=None,
            captured_at_s=None,
            clock_mapped=False,
            metadata=None,
            controls=finite_json(control.settings) if control is not None else None,
            raw_url=f"/api/v1/measurements/{quote(sensor_id, safe='')}/raw",
        )
        disabled = control is not None and control.settings.get("enabled") is False
        if disabled:
            result["status"] = "disabled"
        if record is None:
            return result, None
        sample, observed, initial_age = record
        age = max(0.0, initial_age) + max(0.0, time.monotonic() - observed)
        clock_invalid = initial_age < -1.0
        stale = age > self.stale_timeout_s
        arrays = measurement_arrays(sample)
        valid = sample.valid_mask.astype(bool).copy()
        for name, values in arrays.items():
            if name != "valid_mask":
                valid &= np.isfinite(values)
        total, count = int(valid.size), int(valid.sum())
        modality = (
            "thermal"
            if isinstance(sample, ThermalMeasurement)
            else "acoustic"
            if isinstance(sample, AcousticMeasurement)
            else "semantic"
        )
        frame = sample.metadata.get("frame_metadata", {})
        calibration = frame.get("camera_calibration", frame.get("calibration"))
        if modality == "semantic":
            calibration = sample.metadata.get("calibration_metadata", sample.metadata.get("configuration"))
        descriptors: dict[str, Any] = {}
        for name, values in arrays.items():
            descriptor: dict[str, Any] = {"shape": list(values.shape), "dtype": values.dtype.str}
            if name != "valid_mask":
                selected = values[valid]
                descriptor.update(
                    min=selected.min().item() if count else None, max=selected.max().item() if count else None
                )
            descriptors[name] = descriptor
        result.update(
            status="disabled" if disabled else "invalid_clock" if clock_invalid else "stale" if stale else "live",
            fresh=not disabled and not stale and not clock_invalid,
            stale=stale,
            receipt_age_s=age,
            source_identity=asdict(sample.identity),
            modality=modality,
            units=sample.metadata.get("units"),
            calibration=calibration,
            validity=dict(
                status="invalid" if not count else "valid" if count == total else "partial",
                valid_elements=count,
                total_elements=total,
                valid_fraction=count / total if total else None,
            ),
            received_at_s=sample.received_at_s,
            captured_at_s=sample.captured_at_s,
            clock_mapped=sample.captured_at_s is not None,
            metadata=sample.metadata,
            arrays=descriptors,
        )
        if isinstance(sample, AcousticMeasurement):
            result["sample_rate_hz"] = sample.sample_rate_hz
        return finite_json(result), sample

    def snapshot(self) -> dict[str, Any]:
        return {
            "sensors": [self.get(sensor_id)[0] for sensor_id in self.sensor_ids],
            "status": "configured" if self.sensor_ids else "disabled",
            "read_only": True,
            "cache_policy": "latest_per_declared_sensor",
        }
