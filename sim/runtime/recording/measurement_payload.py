"""Lossless raw measurement NPZ codec; lifecycle and storage belong to timeline.

Arrays are ordinary NPY entries, never object arrays/pickle. ``metadata`` is a
scalar Unicode JSON entry, also exposed on SensorPayloadSample for indexing.
Decoding accepts that sample or the bytes returned by timeline replay.
"""

from __future__ import annotations

import json
from collections.abc import Iterable
from dataclasses import asdict
from io import BytesIO
from typing import Any
from zipfile import BadZipFile

import numpy as np

from ..sensors.acoustics import AcousticFrame, MicrophoneConfig
from ..sensors.camera_geometry import FisheyeCalibration, PanoramaCalibration, PinholeCalibration
from ..sensors.measurement_control import MeasurementNoise
from ..sensors.samples import SensorSampleStamp
from ..sensors.semantic_camera import SemanticCameraConfig, SemanticCameraSample, SemanticLabel
from ..sensors.thermal import ThermalConfig, ThermalFrame
from .timeline import SensorPayloadSample

MEASUREMENT_PAYLOAD_SCHEMA = "lingtu.sim.measurement-payload.v1"
MEASUREMENT_PAYLOAD_ENCODING = "npz"
MEASUREMENT_PAYLOAD_MEDIA_TYPE = "application/vnd.lingtu.measurement+npz"
MeasurementSample = ThermalFrame | AcousticFrame | SemanticCameraSample
_CALIBRATIONS = {
    "pinhole": PinholeCalibration,
    "equidistant_fisheye": FisheyeCalibration,
    "equirectangular": PanoramaCalibration,
}


def _json(value: object) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _tuple_paths(value: object, path: tuple = ()) -> list[list]:
    """Retain tuple-valued camera poses without embedding Python objects."""
    paths = [list(path)] if isinstance(value, tuple) else []
    children: Iterable[tuple[Any, Any]]
    if isinstance(value, dict):
        children = value.items()
    elif isinstance(value, (list, tuple)):
        children = enumerate(value)
    else:
        return paths
    for key, child in children:
        paths.extend(_tuple_paths(child, (*path, key)))
    return paths


def _restore_tuples(document: dict) -> None:
    for path in reversed(document["metadata_tuple_paths"]):
        if not isinstance(path, list) or not path or path[0] not in ("frame_metadata", "calibration_metadata"):
            raise ValueError("invalid metadata tuple path")
        parent = document
        for key in path[:-1]:
            parent = parent[key]
        parent[path[-1]] = tuple(parent[path[-1]])


def _required(metadata: object, names: tuple[str, ...]) -> dict:
    if not isinstance(metadata, dict) or any(name not in metadata for name in names):
        raise ValueError("missing required measurement metadata: " + ", ".join(names))
    return metadata


def _frame(document: dict, arrays: dict[str, np.ndarray], modality: str) -> MeasurementSample:
    if document["schema"] != MEASUREMENT_PAYLOAD_SCHEMA or document["modality"] != modality:
        raise ValueError("measurement schema or modality mismatch")
    stamp = SensorSampleStamp(**document["stamp"])
    units = {"valid_mask": "1"}
    if modality == "thermal":
        units["temperature_k"] = "K"
    elif modality == "acoustic":
        units["pressure_pa"] = "Pa"
        if "voltage_v" in arrays:
            units["voltage_v"] = "V"
    elif modality == "semantic":
        units.update(class_ids="label", instance_ids="label")
    else:
        raise ValueError("unsupported measurement modality")
    if set(arrays) != set(units) or document["units"] != units:
        raise ValueError("missing/invalid arrays or units")
    mask = arrays["valid_mask"]
    if mask.dtype != np.bool_ or mask.ndim != 2 or min(mask.shape) <= 0:
        raise ValueError("valid_mask must be a nonempty 2D boolean array")
    specs = {}
    for name, array in arrays.items():
        if array.shape != mask.shape:
            raise ValueError("measurement array dimensions must match valid_mask")
        if name != "valid_mask":
            if array.dtype.kind not in ("iu" if modality == "semantic" else "iuf"):
                raise ValueError("measurement arrays must have a real numeric dtype (integer labels)")
            if np.isinf(array).any() or not np.isfinite(array[mask]).all():
                raise ValueError("valid measurements must be finite; NaN is allowed only under an invalid mask")
        specs[name] = {"dtype": array.dtype.str, "shape": list(array.shape)}
    if document["arrays"] != specs:
        raise ValueError("array dtype/shape does not match metadata")
    _restore_tuples(document)
    if modality == "semantic":
        config = document["configuration"]
        configuration = SemanticCameraConfig(
            _CALIBRATIONS[config["calibration_model"]](**config["calibration"]),
            tuple(SemanticLabel(**label) for label in config["labels"]),
            MeasurementNoise(**config["noise"]),
        )
        calibration = configuration.calibration
        calibration_metadata = _required(
            document["calibration_metadata"],
            (
                "simulation",
                "projection",
                "optical_axes",
                "pixel_coordinates",
                "transform_composition",
                "gimbal",
                "mount_pose",
                "base_pose",
                "camera_pose_world",
            ),
        )
        projection = calibration_metadata["projection"]
        if (
            mask.shape != (calibration.height, calibration.width)
            or any(projection.get(key) != value for key, value in asdict(calibration).items())
            or projection["model"] != config["calibration_model"]
        ):
            raise ValueError("semantic calibration must match array dimensions and projection")
        source_stamp = SensorSampleStamp(**document["source_stamp"])
        if source_stamp != stamp:
            raise ValueError("source_stamp must match measurement identity")
        if document["simulation"] is not True or document["ground_truth"] is not True:
            raise ValueError("semantic simulation and ground_truth must be explicit true flags")
        return SemanticCameraSample(
            stamp,
            source_stamp,
            arrays["class_ids"],
            arrays["instance_ids"],
            mask,
            document["calibration_metadata"],
            configuration,
            True,
            True,
        )
    if modality == "thermal":
        metadata = _required(
            document["frame_metadata"],
            (
                "temperature_unit",
                "camera_calibration",
                "source_stamp",
                "truth_model",
                "calibration_assumptions",
                "background_temperature_k",
                "atmospheric_temperature_k",
                "transmittance",
                "noise",
                "valid_temperature_bounds_k",
                "temperature_kind",
                "env",
                "qualification",
                "calibrated_lwir",
                "geometry_source",
                "noise_domain",
                "normalized_radiance_formula",
                "invalid_pixel_semantics",
            ),
        )
        config = ThermalConfig(
            metadata["camera_calibration"],
            metadata["background_temperature_k"],
            transmittance=metadata["transmittance"],
            atmospheric_temperature_k=metadata["atmospheric_temperature_k"],
            min_temperature_k=metadata["valid_temperature_bounds_k"][0],
            max_temperature_k=metadata["valid_temperature_bounds_k"][1],
            noise=MeasurementNoise(**metadata["noise"]),
        )
        camera = config.camera_calibration
        if mask.shape != (camera["height"], camera["width"]) or metadata["temperature_unit"] != "K":
            raise ValueError("thermal calibration dimensions/unit mismatch")
        if metadata["temperature_kind"] not in ("apparent_brightness", "emissivity_corrected_surface"):
            raise ValueError("thermal temperature_kind must identify the radiometric output")
        if SensorSampleStamp(**metadata["source_stamp"]) != stamp:
            raise ValueError("source_stamp must match measurement identity")
        assumptions = metadata["calibration_assumptions"]
        if (
            not isinstance(assumptions, list)
            or not assumptions
            or not all(isinstance(a, str) and a for a in assumptions)
        ):
            raise ValueError("thermal calibration assumptions must be explicit text")
        if not isinstance(metadata["truth_model"], str) or not metadata["truth_model"]:
            raise ValueError("thermal truth_model must be explicit")
        return ThermalFrame(stamp, arrays["temperature_k"], mask, metadata)
    metadata = _required(
        document["frame_metadata"],
        (
            "pressure_unit",
            "sources",
            "calibration",
            "propagation_model",
            "near_field_policy",
            "channel_positions_world_m",
            "sensitivity_volts_per_pa",
            "speed_of_sound_mps",
            "noise",
            "simulation",
            "first_sample_index",
            "sample_grid_origin_sim_time_ns",
            "blocked_paths",
            "noise_time_basis",
            "geometry_policy",
            "interpolation",
        ),
    )
    if metadata["pressure_unit"] != "Pa" or not all(
        isinstance(metadata[key], str) and metadata[key] for key in ("propagation_model", "near_field_policy")
    ):
        raise ValueError("acoustic pressure unit/model must be explicit")
    if not isinstance(metadata["sources"], list):
        raise ValueError("acoustic sources must be an explicit list (empty means known silence)")
    for source in metadata["sources"]:
        _required(
            source,
            (
                "source_id",
                "reference_distance_m",
                "sample_rate_hz",
                "start_sim_time_ns",
                "waveform_samples",
                "position_world_m",
                "source_sim_time_ns",
            ),
        )
        if source["source_sim_time_ns"] != stamp.sim_time_ns:
            raise ValueError("acoustic source geometry time must match measurement identity")
    config = MicrophoneConfig(
        document["sample_rate_hz"],
        mask.shape[1],
        tuple(tuple(p) for p in metadata["channel_positions_world_m"]),
        speed_of_sound_mps=metadata["speed_of_sound_mps"],
        sensitivity_volts_per_pa=metadata["sensitivity_volts_per_pa"],
        calibration_metadata=metadata["calibration"],
        noise=MeasurementNoise(**metadata["noise"]),
    )
    if len(config.channel_offsets_m) != mask.shape[0] or ("voltage_v" in arrays) != (
        config.sensitivity_volts_per_pa is not None
    ):
        raise ValueError("acoustic channels/voltage must match declared calibration")
    first_index = metadata["first_sample_index"]
    if (
        type(first_index) is not int
        or first_index < 0
        or metadata["sample_grid_origin_sim_time_ns"] != 0
        or first_index * 1_000_000_000 // config.sample_rate_hz != stamp.sim_time_ns
    ):
        raise ValueError("acoustic sample grid must match the first-sample stamp")
    return AcousticFrame(stamp, arrays["pressure_pa"], mask, config.sample_rate_hz, metadata, arrays.get("voltage_v"))


def _components(sample: MeasurementSample, modality: str) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    if modality == "thermal" and isinstance(sample, ThermalFrame):
        arrays = {"temperature_k": sample.temperature_k, "valid_mask": sample.valid_mask}
        units = {"temperature_k": "K", "valid_mask": "1"}
    elif modality == "acoustic" and isinstance(sample, AcousticFrame):
        arrays = {"pressure_pa": sample.pressure_pa, "valid_mask": sample.valid_mask}
        units = {"pressure_pa": "Pa", "valid_mask": "1"}
        if sample.voltage_v is not None:
            arrays["voltage_v"] = sample.voltage_v
            units["voltage_v"] = "V"
    elif modality == "semantic" and isinstance(sample, SemanticCameraSample):
        arrays = {"class_ids": sample.class_ids, "instance_ids": sample.instance_ids, "valid_mask": sample.valid_mask}
        units = {"class_ids": "label", "instance_ids": "label", "valid_mask": "1"}
    else:
        raise ValueError("modality must match a supported measurement frame")
    if not isinstance(sample.stamp, SensorSampleStamp):
        raise ValueError("stamp must be a SensorSampleStamp")
    if not all(isinstance(array, np.ndarray) for array in arrays.values()):
        raise ValueError("measurements must be NumPy arrays")
    document: dict[str, Any] = {
        "schema": MEASUREMENT_PAYLOAD_SCHEMA,
        "modality": modality,
        "stamp": asdict(sample.stamp),
        "units": units,
        "arrays": {name: {"dtype": array.dtype.str, "shape": list(array.shape)} for name, array in arrays.items()},
    }
    if isinstance(sample, SemanticCameraSample):
        calibration = sample.configuration.calibration
        model = next(name for name, cls in _CALIBRATIONS.items() if isinstance(calibration, cls))
        document.update(
            source_stamp=asdict(sample.source_stamp),
            calibration_metadata=sample.calibration_metadata,
            configuration={**asdict(sample.configuration), "calibration_model": model},
            simulation=sample.simulation,
            ground_truth=sample.ground_truth,
        )
        if "source_clock" in sample.calibration_metadata:
            document["frame_metadata"] = {"source_clock": sample.calibration_metadata["source_clock"]}
    else:
        document["frame_metadata"] = sample.metadata
    if isinstance(sample, AcousticFrame):
        document["sample_rate_hz"] = sample.sample_rate_hz
    document["metadata_tuple_paths"] = [
        path
        for key in ("frame_metadata", "calibration_metadata")
        if key in document
        for path in _tuple_paths(document[key], (key,))
    ]
    encoded_metadata = _json(document)
    _frame(json.loads(encoded_metadata), arrays, modality)
    return arrays, json.loads(encoded_metadata)


def measurement_components(sample: MeasurementSample, *, modality: str) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Validate typed raw arrays/metadata without allocating a compressed archive."""
    try:
        return _components(sample, modality)
    except (KeyError, TypeError, IndexError, AttributeError, StopIteration) as exc:
        raise ValueError(f"invalid measurement frame: {exc}") from exc


def _encode(sample: MeasurementSample, modality: str) -> SensorPayloadSample:
    arrays, document = measurement_components(sample, modality=modality)
    encoded_metadata = _json(document)
    stream = BytesIO()
    np.savez_compressed(stream, metadata=np.array(encoded_metadata), **arrays)
    return SensorPayloadSample(
        sample.stamp.sensor_id,
        modality,
        MEASUREMENT_PAYLOAD_ENCODING,
        MEASUREMENT_PAYLOAD_MEDIA_TYPE,
        sample.stamp.sequence,
        sample.stamp.sim_time_ns,
        stream.getvalue(),
        document,
    )


def encode_measurement_payload(sample: MeasurementSample, *, modality: str) -> SensorPayloadSample:
    """Encode thermal/acoustic/semantic raw frames; invalid input raises ValueError.

    Returns a SensorPayloadSample with encoding ``npz`` and complete JSON metadata.
    No recorder, file I/O, resampling, unit conversion or dtype coercion is added.
    """
    try:
        return _encode(sample, modality)
    except (KeyError, TypeError, IndexError, AttributeError, StopIteration) as exc:
        raise ValueError(f"invalid measurement frame: {exc}") from exc


def decode_measurement_payload(payload: bytes | SensorPayloadSample, *, modality: str) -> MeasurementSample:
    """Decode raw bytes or a timeline sample; malformed data raises ValueError."""
    raw = payload.payload if isinstance(payload, SensorPayloadSample) else payload
    try:
        with np.load(BytesIO(raw), allow_pickle=False) as archive:
            metadata = archive["metadata"]
            if metadata.ndim != 0 or metadata.dtype.kind != "U":
                raise ValueError("metadata must be scalar JSON text")
            document = json.loads(metadata.item())
            _json(document)
            arrays = {name: archive[name] for name in archive.files if name != "metadata"}
        if isinstance(payload, SensorPayloadSample):
            stamp = SensorSampleStamp(**document["stamp"])
            actual = (
                payload.sensor_id,
                payload.stream_kind,
                payload.encoding,
                payload.media_type,
                payload.sample_sequence,
                payload.sample_time_ns,
            )
            expected = (
                stamp.sensor_id,
                modality,
                MEASUREMENT_PAYLOAD_ENCODING,
                MEASUREMENT_PAYLOAD_MEDIA_TYPE,
                stamp.sequence,
                stamp.sim_time_ns,
            )
            if actual != expected or _json(payload.metadata) != _json(document):
                raise ValueError("timeline envelope identity/metadata does not match measurement payload")
        return _frame(document, arrays, modality)
    except (KeyError, TypeError, IndexError, AttributeError, OSError, EOFError, BadZipFile) as exc:
        raise ValueError(f"invalid measurement payload: {exc}") from exc
