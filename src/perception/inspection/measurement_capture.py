"""Host DTO to inspection artifact conversion; no simulator or driver codec."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import asdict
from io import BytesIO
from math import isfinite
from typing import Any, cast

import numpy as np

from runtime.contracts.inspection_evidence import SUPPORTED_ACTIONS, InspectionFinding, MeasurementArtifact
from runtime.msgs.measurement import AcousticMeasurement, SemanticMeasurement, ThermalMeasurement

from .measurement_checks import AcousticRule, ThermalRule, evaluate_acoustic, evaluate_thermal

Measurement = ThermalMeasurement | AcousticMeasurement | SemanticMeasurement
Rule = ThermalRule | AcousticRule
MODALITIES = {ThermalMeasurement: "thermal", AcousticMeasurement: "acoustic", SemanticMeasurement: "semantic"}


def measurement_configuration(
    product_session_id: str, sensors: Mapping[str, Mapping[str, Any]],
    rules: Mapping[str, Mapping[str, Any]], selection: Mapping[str, Sequence[str]],
) -> tuple[dict[str, dict[str, Any]], dict[str, Rule], dict[str, tuple[str, ...]]]:
    """Validate inspection declarations, leaving DTO/wire validation upstream."""
    if sensors and (not isinstance(product_session_id, str) or not product_session_id.strip()):
        raise ValueError("measurement_sensors require an explicit product_session_id")
    bindings = {}
    for sensor_id, binding in sensors.items():
        if not isinstance(binding, Mapping) or binding.get("modality") not in MODALITIES.values():
            raise ValueError(f"invalid measurement modality: {sensor_id}")
        if any(not isinstance(binding.get(key), str) or not binding[key].strip()
               for key in ("session_id", "instance_id", "frame_id")):
            raise ValueError(f"measurement sensor requires explicit source identity: {sensor_id}")
        if any(type(binding.get(key)) is not int or binding[key] < 0
               for key in ("model_generation", "reset_generation")):
            raise ValueError(f"measurement sensor requires explicit model/reset generations: {sensor_id}")
        bindings[sensor_id] = dict(binding)
    parsed_rules: dict[str, Rule] = {}
    for sensor_id, rule in rules.items():
        if sensor_id not in bindings or bindings[sensor_id]["modality"] == "semantic":
            raise ValueError(f"measurement rule requires a declared thermal/acoustic sensor: {sensor_id}")
        rule_type = ThermalRule if bindings[sensor_id]["modality"] == "thermal" else AcousticRule
        parsed_rules[sensor_id] = rule_type(**rule)
    actions = {}
    for action, sensor_ids in selection.items():
        if (action not in SUPPORTED_ACTIONS or isinstance(sensor_ids, (str, bytes))
                or not isinstance(sensor_ids, Sequence) or len(set(sensor_ids)) != len(sensor_ids)
                or any(sensor_id not in bindings for sensor_id in sensor_ids)):
            raise ValueError(f"capture_sensor_ids requires distinct declared sensors and a supported action: {action}")
        if action in {"capture:thermal", "capture:acoustic"} and not any(
            action == f"capture:{bindings[sensor_id]['modality']}" for sensor_id in sensor_ids
        ):
            raise ValueError(f"{action} requires at least one matching declared sensor")
        actions[action] = tuple(sensor_ids)
    return bindings, parsed_rules, actions


class MeasurementUnavailable(ValueError):
    """A requested observation cannot be bound to this capture."""


def validate_measurement_identity(sample: Measurement, binding: Mapping[str, Any], product_session_id: str) -> None:
    identity = sample.identity
    if MODALITIES[type(sample)] != binding["modality"]:
        raise MeasurementUnavailable(f"measurement_modality_mismatch:{identity.sensor_id}")
    if (not product_session_id or identity.product_session_id != product_session_id
            or any(getattr(identity, key) != binding.get(key) for key in (
                "session_id", "instance_id", "frame_id", "model_generation", "reset_generation",
            ))):
        raise MeasurementUnavailable(f"measurement_identity_mismatch:{identity.sensor_id}")
    expected_stamp = asdict(identity)
    del expected_stamp["product_session_id"]
    for container in (sample.metadata, sample.metadata.get("frame_metadata", {})):
        for key in ("stamp", "source_stamp"):
            if key in container and container[key] != expected_stamp:
                raise MeasurementUnavailable(f"measurement_identity_mismatch:{identity.sensor_id}")


def validate_measurement_time(
    sample: Measurement, *, now: float, rgb_ts: float, odom_ts: float, max_age_s: float, max_skew_s: float,
) -> None:
    sensor_id = sample.identity.sensor_id
    captured = sample.captured_at_s
    if captured is None:
        raise MeasurementUnavailable(f"measurement_clock_unmapped:{sensor_id}")
    if any(not isfinite(ts) or ts <= 0 or not -max_skew_s <= now - ts <= max_age_s
           for ts in (captured, sample.received_at_s)):
        raise MeasurementUnavailable(f"measurement_stale:{sensor_id}")
    if abs(captured - rgb_ts) > max_skew_s:
        raise MeasurementUnavailable(f"measurement_rgb_unsynchronized:{sensor_id}")
    if abs(captured - odom_ts) > max_skew_s:
        raise MeasurementUnavailable(f"measurement_odometry_unsynchronized:{sensor_id}")


def capture_measurements(
    samples: Mapping[str, Measurement], sensor_ids: Sequence[str], rules: Mapping[str, Rule],
    *, action: str,
) -> tuple[tuple[MeasurementArtifact, ...], InspectionFinding | None]:
    artifacts = []
    findings: dict[str, InspectionFinding] = {}
    unassessed = False
    for sensor_id in sensor_ids:
        sample = samples[sensor_id]
        metadata = dict(sample.metadata)
        metadata.update(product_session_id=sample.identity.product_session_id, captured_at_s=sample.captured_at_s)
        modality = MODALITIES[type(sample)]
        arrays: dict[str, Any] = {"valid_mask": sample.valid_mask}
        if isinstance(sample, ThermalMeasurement):
            arrays["temperature_k"] = sample.temperature_k
        elif isinstance(sample, AcousticMeasurement):
            arrays["pressure_pa"] = sample.pressure_pa
            if sample.voltage_v is not None:
                arrays["voltage_v"] = sample.voltage_v
            metadata["sample_rate_hz"] = sample.sample_rate_hz
        else:
            arrays.update(class_ids=sample.class_ids, instance_ids=sample.instance_ids)
        stream = BytesIO()
        np.savez_compressed(stream, **arrays, metadata=np.array(json.dumps(metadata, allow_nan=False)))
        artifacts.append(MeasurementArtifact(
            modality=modality, sensor_id=sensor_id, payload=stream.getvalue(),
            media_type="application/x-npz", metadata=metadata,
        ))
        if (sensor_id in rules and action == f"capture:{modality}"
                and isinstance(sample, (ThermalMeasurement, AcousticMeasurement))):
            frame = metadata.get("frame_metadata", {})
            provenance = metadata.get("provenance", frame.get("provenance"))
            rule = rules[sensor_id]
            if isinstance(sample, ThermalMeasurement):
                temperature_kind = frame.get("temperature_kind")
                temperature_kind = {
                    "apparent_brightness": "apparent",
                    "emissivity_corrected_surface": "corrected",
                }.get(temperature_kind, temperature_kind)
                assessment = evaluate_thermal(
                    sample.temperature_k, sample.valid_mask, cast(ThermalRule, rule),
                    unit=metadata.get("units", {}).get("temperature_k"),
                    temperature_kind=temperature_kind,
                    calibration=metadata.get("calibration", frame.get("camera_calibration")),
                    provenance=provenance,
                )
            else:
                assessment = evaluate_acoustic(
                    sample.pressure_pa, sample.valid_mask, cast(AcousticRule, rule), fs=sample.sample_rate_hz,
                    unit=metadata.get("units", {}).get("pressure_pa"),
                    calibration=metadata.get("calibration", frame.get("calibration")), provenance=provenance,
                )
            findings[sensor_id] = InspectionFinding(
                observed_state=assessment.verdict, modality=modality, source="inspection_measurement",
                reason=assessment.reason, details=assessment.to_dict(),
                model_provenance={"model_id": rule.model_id, "model_version": rule.model_version},
            )
        elif action == f"capture:{modality}":
            unassessed = True
    if not findings or unassessed:
        return tuple(artifacts), None
    # Insufficient evidence cannot be hidden by another sensor's normal value.
    priority = {"inconclusive": 0, "anomaly": 1, "normal": 2}
    finding = min(findings.values(), key=lambda item: priority[item.observed_state])
    if len(findings) > 1:
        finding = InspectionFinding(
            observed_state=finding.observed_state, modality=finding.modality, source=finding.source,
            reason=finding.reason, model_provenance=finding.model_provenance,
            details=dict(finding.details) | {"sensor_assessments": {
                sensor_id: dict(item.details) for sensor_id, item in findings.items()
            }},
        )
    return tuple(artifacts), finding
