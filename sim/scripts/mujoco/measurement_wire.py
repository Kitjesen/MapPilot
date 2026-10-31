"""Typed raw measurement wire for the existing camera-owned LTU1 connection."""

from __future__ import annotations

import json
import struct

import numpy as np
from sim.runtime.recording.measurement_payload import MeasurementSample, measurement_components
from sim.runtime.sensors.acoustics import AcousticFrame
from sim.runtime.sensors.semantic_camera import SemanticCameraSample
from sim.runtime.sensors.thermal import ThermalFrame

from .native_sensor_records import RECORD_MEASUREMENT, EncodedSensorRecord, encode_record

MEASUREMENT_HEADER = struct.Struct("<4sHH6I4Q5H2x")
MAX_MEASUREMENT_BYTES = 32 * 1024 * 1024
MAX_METADATA_BYTES = 1024 * 1024


def _identifier(value: str) -> bytes:
    if not isinstance(value, str) or not value or value != value.strip() or "\0" in value:
        raise ValueError("measurement identities must be nonempty trimmed text without NUL")
    encoded = value.encode("utf-8")
    if len(encoded) > 255:
        raise ValueError("measurement identities must fit 255 UTF-8 bytes")
    return encoded


def encode_measurement(sample: MeasurementSample, *, product_session_id: str) -> EncodedSensorRecord:
    """Encode little-endian typed arrays, never NPZ or a depth/audio Image alias.

    The source stamp remains unchanged; the Product owner supplies its separate
    identity. The native authenticated endpoint verifies that binding.
    """
    if isinstance(sample, ThermalFrame):
        modality, kind, primary_name, secondary_name = "thermal", 1, "temperature_k", None
    elif isinstance(sample, AcousticFrame):
        modality, kind, primary_name, secondary_name = (
            "acoustic",
            2,
            "pressure_pa",
            "voltage_v" if sample.voltage_v is not None else None,
        )
    elif isinstance(sample, SemanticCameraSample):
        modality, kind, primary_name, secondary_name = "semantic", 3, "class_ids", "instance_ids"
    else:
        raise ValueError("unsupported raw measurement type")
    arrays, metadata = measurement_components(sample, modality=modality)
    stamp = sample.stamp
    identities = [
        _identifier(value)
        for value in (product_session_id, stamp.session_id, stamp.instance_id, stamp.sensor_id, stamp.frame_id)
    ]
    for name in ("model_generation", "reset_generation", "sequence", "sim_time_ns"):
        if not 0 <= getattr(stamp, name) < 1 << 64:
            raise ValueError(f"measurement {name} exceeds uint64")
    rows, columns = arrays[primary_name].shape
    count = int(rows) * int(columns)
    header_size = MEASUREMENT_HEADER.size + sum(map(len, identities))
    flags = int(kind == 2 and secondary_name is not None)
    sample_rate = sample.sample_rate_hz if isinstance(sample, AcousticFrame) else 0
    if not 0 <= sample_rate < 1 << 32:
        raise ValueError("measurement sample_rate_hz exceeds uint32")
    raw_size = count * (9 if secondary_name else 5)
    if not count or raw_size + header_size > MAX_MEASUREMENT_BYTES:
        raise ValueError("measurement dimensions exceed 32 MiB wire limit")
    raw_arrays = {}
    for name in (primary_name, secondary_name):
        if name is None:
            continue
        dtype = "<u4" if kind == 3 else "<f4"
        with np.errstate(over="ignore", invalid="ignore"):
            values = np.ascontiguousarray(arrays[name], dtype=dtype)
        if kind != 3 and not np.isfinite(values[arrays["valid_mask"]]).all():
            raise ValueError("measurement values do not fit finite float32")
        raw_arrays[name] = values
        metadata["arrays"][name]["dtype"] = dtype
    metadata["wire_encoding"] = "ltm1-le-v1"
    encoded_metadata = json.dumps(metadata, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
    if (
        not 0 < len(encoded_metadata) <= MAX_METADATA_BYTES
        or header_size + len(encoded_metadata) + raw_size > MAX_MEASUREMENT_BYTES
    ):
        raise ValueError("measurement metadata or payload exceeds wire limit")
    header = MEASUREMENT_HEADER.pack(
        b"LTM1",
        1,
        kind,
        header_size,
        len(encoded_metadata),
        rows,
        columns,
        sample_rate,
        flags,
        stamp.model_generation,
        stamp.reset_generation,
        stamp.sequence,
        stamp.sim_time_ns,
        *map(len, identities),
    )
    payload = header + b"".join(identities) + encoded_metadata + raw_arrays[primary_name].tobytes()
    payload += arrays["valid_mask"].astype(np.uint8).tobytes()
    if secondary_name:
        payload += raw_arrays[secondary_name].tobytes()
    return encode_record(
        RECORD_MEASUREMENT, timestamp_ns=stamp.sim_time_ns, sequence=stamp.sequence, count=count, payload=payload
    )
