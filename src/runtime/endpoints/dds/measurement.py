"""Decode C-ABI reader LTM1 bytes into Host messages, without Python DDS."""

from __future__ import annotations

import json
import struct
from dataclasses import asdict
from math import isfinite
from typing import Any

import numpy as np

from runtime.contracts.inspection_evidence import _validate_measurement_metadata
from runtime.msgs.measurement import AcousticMeasurement, MeasurementIdentity, SemanticMeasurement, ThermalMeasurement

_HEADER = struct.Struct("<4sHH6I4Q5H2x")
_MAX_PAYLOAD_BYTES = 32 * 1024 * 1024
_MAX_METADATA_BYTES = 1024 * 1024


def _validate_dimensions(calibration: Any, shape: tuple[int, ...]) -> None:
    if (
        not isinstance(calibration, dict)
        or type(calibration.get("height")) is not int
        or type(calibration.get("width")) is not int
        or (calibration["height"], calibration["width"]) != shape
    ):
        raise ValueError("measurement calibration dimensions must match array shape")


def _validate_values(values: np.ndarray, mask: np.ndarray, *, labels: bool = False, kelvin: bool = False) -> None:
    if labels:
        if (values[~mask] != 0).any() or (values[mask] == 0).any() or (values[mask] > 0x7FFFFFFF).any():
            raise ValueError("invalid semantic labels/mask")
    elif not np.isfinite(values[mask]).all() or not np.isnan(values[~mask]).all():
        raise ValueError("valid values must be finite; invalid values must be NaN")
    elif kelvin and (values[mask] <= 0).any():
        raise ValueError("valid Kelvin values must be strictly positive")


def _validate_metadata(
    metadata: Any,
    identity: MeasurementIdentity,
    modality: str,
    arrays: dict[str, np.ndarray],
    fs: int,
) -> None:
    _validate_measurement_metadata(metadata, identity.sensor_id, modality)
    if "stamp" not in metadata:
        raise ValueError("measurement metadata requires the wire source stamp")
    units = {"valid_mask": "1"}
    units.update(
        {
            name: {
                "temperature_k": "K",
                "pressure_pa": "Pa",
                "voltage_v": "V",
                "class_ids": "label",
                "instance_ids": "label",
            }[name]
            for name in arrays
            if name != "valid_mask"
        }
    )
    if metadata.get("units") != units:
        raise ValueError("measurement units must match native array units")
    specs = {name: {"dtype": array.dtype.str, "shape": list(array.shape)} for name, array in arrays.items()}
    if metadata.get("arrays") != specs or any(
        type(size) is not int for spec in metadata["arrays"].values() for size in spec["shape"]
    ):
        raise ValueError("measurement array dtype/shape does not match metadata")
    frame_metadata = metadata.get("frame_metadata", {})
    if not isinstance(frame_metadata, dict):
        raise ValueError("measurement frame_metadata must be an object")
    expected_stamp = asdict(identity)
    del expected_stamp["product_session_id"]
    for container in (metadata, frame_metadata):
        for name in ("stamp", "source_stamp"):
            if name not in container:
                continue
            stamp = container[name]
            if (
                not isinstance(stamp, dict)
                or stamp != expected_stamp
                or any(type(stamp[key]) is not type(value) for key, value in expected_stamp.items())
            ):
                raise ValueError("measurement source_stamp must match binary identity")
    if modality == "acoustic" and (type(metadata.get("sample_rate_hz")) is not int or metadata["sample_rate_hz"] != fs):
        raise ValueError("measurement sample_rate_hz does not match header")
    shape = arrays["valid_mask"].shape
    if modality == "thermal":
        if "temperature_unit" in frame_metadata and frame_metadata["temperature_unit"] != "K":
            raise ValueError("thermal temperature_unit must be K")
        if "camera_calibration" in frame_metadata:
            _validate_dimensions(frame_metadata["camera_calibration"], shape)
    elif modality == "acoustic":
        if "pressure_unit" in frame_metadata and frame_metadata["pressure_unit"] != "Pa":
            raise ValueError("acoustic pressure_unit must be Pa")
        if "sensitivity_volts_per_pa" in frame_metadata:
            sensitivity = frame_metadata["sensitivity_volts_per_pa"]
            if (sensitivity is not None) != ("voltage_v" in arrays):
                raise ValueError("acoustic voltage presence must match sensitivity calibration")
            if sensitivity is not None:
                try:
                    valid = not isinstance(sensitivity, bool) and isinstance(sensitivity, (int, float))
                    valid = valid and isfinite(sensitivity) and sensitivity > 0
                except OverflowError:
                    valid = False
                if not valid:
                    raise ValueError("acoustic sensitivity calibration must be finite and positive")
    else:
        configuration = metadata.get("configuration", {})
        if not isinstance(configuration, dict):
            raise ValueError("semantic configuration must be an object")
        if "calibration" in configuration:
            _validate_dimensions(configuration["calibration"], shape)
        calibration_metadata = metadata.get("calibration_metadata", {})
        if not isinstance(calibration_metadata, dict):
            raise ValueError("semantic calibration_metadata must be an object")
        if "projection" in calibration_metadata:
            projection = calibration_metadata["projection"]
            _validate_dimensions(projection, shape)
            if "calibration_model" in configuration and projection.get("model") != configuration["calibration_model"]:
                raise ValueError("semantic projection model must match calibration")


def measurement_from_record(
    payload: bytes,
    *,
    received_at_s: float,
    expected_product_session_id: str,
    expected_session_id: str,
) -> ThermalMeasurement | AcousticMeasurement | SemanticMeasurement:
    """Decode the inner LTM1 payload, not the reader-owned LTU1 envelope."""
    if not isinstance(payload, bytes) or not _HEADER.size <= len(payload) <= _MAX_PAYLOAD_BYTES:
        raise ValueError("LTM1 payload must be bytes within 76-byte/32 MiB limits")
    fields = _HEADER.unpack_from(payload)
    magic, version, kind, header_bytes, metadata_bytes, rows, columns, fs, flags = fields[:9]
    if magic != b"LTM1" or version != 1 or kind not in (1, 2, 3):
        raise ValueError("invalid LTM1 magic, version or modality")
    if not 0 < metadata_bytes <= _MAX_METADATA_BYTES:
        raise ValueError("LTM1 metadata must be within 1-byte/1 MiB limits")
    if not rows or not columns or (fs <= 0 if kind == 2 else fs != 0):
        raise ValueError("invalid LTM1 shape or sample rate")
    if flags > (1 if kind == 2 else 0) or payload[74:76] != b"\0\0":
        raise ValueError("unsupported LTM1 flags or reserved bytes")
    if fields[11] > 0xFFFFFFFF or fields[12] // 1_000_000_000 > 0x7FFFFFFF:
        raise ValueError("LTM1 sequence/time exceeds native LTU1/IDL range")
    lengths = fields[13:18]
    if any(not 0 < length <= 255 for length in lengths) or header_bytes != 76 + sum(lengths):
        raise ValueError("LTM1 header/identifier length mismatch")
    count = rows * columns
    stride = 9 if kind == 3 or flags else 5
    if count > _MAX_PAYLOAD_BYTES // stride or header_bytes + metadata_bytes + count * stride != len(payload):
        raise ValueError("LTM1 payload length/shape mismatch")
    cursor = _HEADER.size
    identifiers = []
    for length in lengths:
        identifier = payload[cursor : cursor + length].decode("utf-8")
        if not identifier or identifier != identifier.strip() or "\0" in identifier:
            raise ValueError("LTM1 identifier must be trimmed nonempty UTF-8 without NUL")
        identifiers.append(identifier)
        cursor += length
    identity = MeasurementIdentity(
        product_session_id=identifiers[0],
        session_id=identifiers[1],
        instance_id=identifiers[2],
        sensor_id=identifiers[3],
        frame_id=identifiers[4],
        model_generation=fields[9],
        reset_generation=fields[10],
        sequence=fields[11],
        sim_time_ns=fields[12],
    )
    if not expected_product_session_id or identity.product_session_id != expected_product_session_id:
        raise ValueError("measurement product session mismatch")
    if not expected_session_id or identity.session_id != expected_session_id:
        raise ValueError("measurement source session mismatch")
    metadata = json.loads(payload[header_bytes : header_bytes + metadata_bytes].decode("utf-8"))
    cursor = header_bytes + metadata_bytes
    count = rows * columns
    dtype = "<u4" if kind == 3 else "<f4"
    values = np.frombuffer(payload, dtype=dtype, count=count, offset=cursor).reshape(rows, columns)
    raw_mask = np.frombuffer(payload, dtype="u1", count=count, offset=cursor + count * 4).reshape(rows, columns)
    if (raw_mask > 1).any():
        raise ValueError("LTM1 mask must contain exactly 0 or 1")
    mask = raw_mask.astype(bool)
    mask.setflags(write=False)
    _validate_values(values, mask, labels=kind == 3, kelvin=kind == 1)
    if kind == 3:
        instances = np.frombuffer(payload, dtype="<u4", count=count, offset=cursor + count * 5).reshape(rows, columns)
        _validate_values(instances, mask, labels=True)
        _validate_metadata(
            metadata, identity, "semantic", {"class_ids": values, "instance_ids": instances, "valid_mask": mask}, fs
        )
        return SemanticMeasurement(identity, values, instances, mask, metadata, received_at_s)
    if kind == 2:
        voltage = (
            np.frombuffer(payload, dtype="<f4", count=count, offset=cursor + count * 5).reshape(rows, columns)
            if flags
            else None
        )
        if voltage is not None:
            _validate_values(voltage, mask)
        arrays = {"pressure_pa": values, "valid_mask": mask}
        if voltage is not None:
            arrays["voltage_v"] = voltage
        _validate_metadata(metadata, identity, "acoustic", arrays, fs)
        return AcousticMeasurement(identity, values, mask, fs, voltage, metadata, received_at_s)
    _validate_metadata(metadata, identity, "thermal", {"temperature_k": values, "valid_mask": mask}, fs)
    return ThermalMeasurement(identity, values, mask, metadata, received_at_s)
