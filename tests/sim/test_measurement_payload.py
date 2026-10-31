"""Lossless measurement payloads through the existing recording seam."""

from __future__ import annotations

import json
from dataclasses import asdict, replace
from io import BytesIO

import numpy as np
import pytest
from sim.runtime.recording.measurement_payload import (
    decode_measurement_payload,
    encode_measurement_payload,
)
from sim.runtime.recording.timeline import SensorPayloadSample, SimulationRecordingWriter
from sim.runtime.replay import SimulationReplay
from sim.runtime.sensors.acoustics import AcousticSource, MicrophoneConfig, MicrophoneSensor
from sim.runtime.sensors.camera_geometry import FisheyeCalibration, PanoramaCalibration, PinholeCalibration
from sim.runtime.sensors.samples import SensorSampleStamp
from sim.runtime.sensors.semantic_camera import SemanticCameraConfig, SemanticLabel, semantic_from_geom_ids
from sim.runtime.sensors.thermal import ThermalConfig, ThermalSensor, ThermalSurface


def _stamp(sensor_id="thermal"):
    return SensorSampleStamp("recording-test", "robot", sensor_id, "optical", 2, 3, 7, 1_000_000_000)


def _thermal():
    camera = dict(model="pinhole", width=2, height=2, fx=80, fy=80, cx=1, cy=1, calibration_id="sim-v1")
    stamp = _stamp()
    return ThermalSensor(ThermalConfig(camera, 280), {1: ThermalSurface(320, 0.8)}).measure(
        stamp,
        source_stamp=stamp,
        geom_ids=np.array([[1, -1], [1, 1]]),
        distances_m=np.ones((2, 2)),
        valid_rays=np.ones((2, 2), dtype=bool),
    )


def _assert_array(actual, expected):
    assert actual.dtype == expected.dtype
    assert actual.shape == expected.shape
    assert actual.tobytes() == expected.tobytes()


def test_thermal_round_trip_preserves_raw_nan_dtype_stamp_and_complete_metadata():
    original = replace(_thermal(), temperature_k=_thermal().temperature_k.astype(">f8"))
    encoded = encode_measurement_payload(original, modality="thermal")
    assert isinstance(encoded, SensorPayloadSample)
    assert (encoded.sensor_id, encoded.stream_kind) == (original.stamp.sensor_id, "thermal")
    assert (encoded.sample_sequence, encoded.sample_time_ns) == (7, 1_000_000_000)
    assert encoded.metadata["stamp"] == asdict(original.stamp)
    assert encoded.metadata["units"]["temperature_k"] == "K"
    restored = decode_measurement_payload(encoded, modality="thermal")
    assert type(restored) is type(original)
    assert restored.stamp == original.stamp
    assert restored.metadata == original.metadata
    _assert_array(restored.temperature_k, original.temperature_k)
    _assert_array(restored.valid_mask, original.valid_mask)
    with np.load(BytesIO(encoded.payload), allow_pickle=False) as archive:
        for name in archive.files:
            assert not archive[name].dtype.hasobject


def _acoustic(calibrated=True):
    stamp = _stamp("microphone")
    source = AcousticSource(stamp, "pump", np.array([0.2, -0.4, 0.1]), 1.0, 8_000, 1_000_000_000, (0, 0, 0))
    config = MicrophoneConfig(
        8_000,
        4,
        ((0, 0, 0), (1, 0, 0)),
        sensitivity_volts_per_pa=0.02 if calibrated else None,
        calibration_metadata={"calibration_id": "declared-sim", "response": "flat"} if calibrated else None,
    )
    return MicrophoneSensor(config).sample(stamp, (source,), blocked_paths=(("pump", 1),))


@pytest.mark.parametrize("calibrated", [False, True])
def test_acoustic_round_trip_keeps_channels_sample_grid_source_and_optional_voltage(calibrated):
    original = _acoustic(calibrated)
    encoded = encode_measurement_payload(original, modality="acoustic")
    restored = decode_measurement_payload(encoded.payload, modality="acoustic")
    assert type(restored) is type(original)
    assert restored.stamp == original.stamp
    assert restored.sample_rate_hz == original.sample_rate_hz
    assert restored.metadata == original.metadata
    assert encoded.metadata["units"]["pressure_pa"] == "Pa"
    _assert_array(restored.pressure_pa, original.pressure_pa)
    _assert_array(restored.valid_mask, original.valid_mask)
    if calibrated:
        assert encoded.metadata["units"]["voltage_v"] == "V"
        _assert_array(restored.voltage_v, original.voltage_v)
    else:
        assert restored.voltage_v is None


def _semantic(calibration):
    stamp = _stamp("semantic")
    config = SemanticCameraConfig(calibration, (SemanticLabel(1, 4, 12), SemanticLabel(2, 5, 13)))
    return semantic_from_geom_ids(config, stamp, np.array([[1, -1], [2, 1]]), source_stamp=stamp)


@pytest.mark.parametrize(
    "calibration",
    [
        PinholeCalibration(2, 2, 80, 80, 1, 1),
        FisheyeCalibration(2, 2, 1, 1, 1, 180, 1.5),
        PanoramaCalibration(2, 2),
    ],
)
def test_semantic_round_trip_restores_projection_labels_ground_truth_and_source_identity(calibration):
    original = _semantic(calibration)
    original = replace(original, class_ids=original.class_ids.astype(np.uint16))
    encoded = encode_measurement_payload(original, modality="semantic")
    restored = decode_measurement_payload(encoded, modality="semantic")
    assert type(restored) is type(original)
    assert restored.stamp == original.stamp
    assert restored.source_stamp == original.source_stamp
    assert restored.configuration == original.configuration
    assert restored.calibration_metadata == original.calibration_metadata
    assert restored.simulation is True and restored.ground_truth is True
    assert encoded.metadata["units"]["class_ids"] == "label"
    for name in ("class_ids", "instance_ids", "valid_mask"):
        _assert_array(getattr(restored, name), getattr(original, name))


def _encoded(modality):
    sample = {
        "thermal": _thermal,
        "acoustic": _acoustic,
        "semantic": lambda: _semantic(PinholeCalibration(2, 2, 80, 80, 1, 1)),
    }[modality]()
    return encode_measurement_payload(sample, modality=modality)


@pytest.mark.parametrize(
    "modality,damage",
    [
        ("thermal", "missing_array"),
        ("acoustic", "missing_array"),
        ("semantic", "missing_array"),
        ("thermal", "mask_dtype"),
        ("acoustic", "shape"),
        ("semantic", "numeric_dtype"),
        ("thermal", "object_array"),
        ("thermal", "infinite"),
        ("thermal", "valid_nan"),
        ("thermal", "missing_units"),
        ("acoustic", "wrong_units"),
        ("semantic", "missing_metadata"),
        ("thermal", "missing_metadata"),
        ("acoustic", "missing_metadata"),
        ("thermal", "bad_calibration"),
        ("thermal", "missing_assumptions"),
        ("acoustic", "missing_sources"),
        ("acoustic", "missing_model"),
        ("acoustic", "missing_calibration"),
        ("acoustic", "bad_rate"),
        ("semantic", "bad_calibration"),
        ("semantic", "source_identity"),
        ("thermal", "source_identity"),
        ("thermal", "missing_stamp"),
        ("semantic", "bad_flag"),
        ("thermal", "extra_array"),
        ("thermal", "dtype_manifest"),
        ("thermal", "metadata_nan"),
        ("thermal", "missing_temperature_kind"),
        ("acoustic", "missing_grid"),
        ("acoustic", "source_time"),
        ("semantic", "missing_pose"),
    ],
)
def test_decode_rejects_incomplete_or_invalid_measurements(modality, damage):
    encoded = _encoded(modality)
    with np.load(BytesIO(encoded.payload), allow_pickle=False) as archive:
        arrays = {key: archive[key] for key in archive.files if key != "metadata"}
        document = json.loads(archive["metadata"].item())
    name = {"thermal": "temperature_k", "acoustic": "pressure_pa", "semantic": "class_ids"}[modality]
    metadata = document.get("frame_metadata", document.get("calibration_metadata"))
    if damage == "missing_array":
        del arrays[name]
    elif damage == "mask_dtype":
        arrays["valid_mask"] = arrays["valid_mask"].astype(np.uint8)
    elif damage == "shape":
        arrays[name] = arrays[name].ravel()
    elif damage == "numeric_dtype":
        arrays[name] = arrays[name].astype(float)
    elif damage == "object_array":
        arrays[name] = arrays[name].astype(object)
    elif damage in ("infinite", "valid_nan"):
        arrays[name][0, 0] = np.inf if damage == "infinite" else np.nan
    elif damage == "missing_units":
        del document["units"]
    elif damage == "wrong_units":
        document["units"][name] = "dB"
    elif damage == "missing_metadata":
        del document["configuration" if modality == "semantic" else "frame_metadata"]
    elif damage == "bad_calibration":
        if modality == "semantic":
            document["configuration"]["calibration"]["width"] = 99
        else:
            metadata["camera_calibration"]["width"] = 99
    elif damage == "missing_assumptions":
        del metadata["calibration_assumptions"]
    elif damage == "missing_sources":
        del metadata["sources"]
    elif damage == "missing_model":
        del metadata["propagation_model"]
    elif damage == "missing_calibration":
        del metadata["calibration"]
    elif damage == "bad_rate":
        document["sample_rate_hz"] = 0
    elif damage == "source_identity":
        source = document["source_stamp"] if modality == "semantic" else metadata["source_stamp"]
        source["reset_generation"] += 1
    elif damage == "missing_stamp":
        del document["stamp"]["frame_id"]
    elif damage == "bad_flag":
        document["ground_truth"] = "true"
    elif damage == "extra_array":
        arrays["unlabelled"] = np.zeros((2, 2))
    elif damage == "dtype_manifest":
        document["arrays"][name]["dtype"] = "<f8"
    elif damage == "metadata_nan":
        metadata["unexpected"] = float("nan")
    elif damage == "missing_temperature_kind":
        del metadata["temperature_kind"]
    elif damage == "missing_grid":
        del metadata["first_sample_index"]
    elif damage == "source_time":
        metadata["sources"][0]["source_sim_time_ns"] += 1
    elif damage == "missing_pose":
        del metadata["mount_pose"]
    stream = BytesIO()
    np.savez_compressed(stream, metadata=np.array(json.dumps(document)), **arrays)
    with pytest.raises(ValueError):
        decode_measurement_payload(stream.getvalue(), modality=modality)


@pytest.mark.parametrize(
    "damage",
    [
        "object",
        "mask",
        "shape",
        "not_array",
        "unit",
        "missing_metadata",
        "stamp",
        "source_stamp",
        "non_json",
    ],
)
def test_encode_refuses_invalid_frames_before_creating_a_payload(damage):
    sample = _thermal()
    if damage == "object":
        sample = replace(sample, temperature_k=sample.temperature_k.astype(object))
    elif damage == "mask":
        sample = replace(sample, valid_mask=sample.valid_mask.astype(int))
    elif damage == "shape":
        sample = replace(sample, temperature_k=sample.temperature_k.ravel())
    elif damage == "not_array":
        sample = replace(sample, temperature_k=sample.temperature_k.tolist())
    elif damage == "unit":
        sample = replace(sample, metadata=dict(sample.metadata, temperature_unit="degC"))
    elif damage == "missing_metadata":
        sample = replace(sample, metadata={})
    elif damage == "stamp":
        sample = replace(sample, stamp=None)
    elif damage == "source_stamp":
        sample = replace(
            sample, metadata=dict(sample.metadata, source_stamp=asdict(replace(sample.stamp, sequence=99)))
        )
    elif damage == "non_json":
        sample = replace(sample, metadata=dict(sample.metadata, custom=object()))
    with pytest.raises(ValueError):
        encode_measurement_payload(sample, modality="thermal")


@pytest.mark.parametrize(
    "field,value",
    [
        ("sensor_id", "other"),
        ("stream_kind", "acoustic"),
        ("sample_sequence", 8),
        ("sample_time_ns", 1),
        ("encoding", "pickle"),
        ("media_type", "text/plain"),
        ("metadata", {}),
    ],
)
def test_decode_checks_timeline_envelope_identity_against_embedded_metadata(field, value):
    encoded = _encoded("thermal")
    with pytest.raises(ValueError):
        decode_measurement_payload(replace(encoded, **{field: value}), modality="thermal")


@pytest.mark.parametrize("modality", ["thermal", "acoustic", "semantic"])
def test_existing_timeline_writer_and_replay_store_npz_bytes_and_complete_metadata(tmp_path, modality):
    encoded = _encoded(modality)
    stamp = encoded.metadata["stamp"]
    snapshot = {
        "event": "snapshot",
        "session_id": stamp["session_id"],
        "model_generation": stamp["model_generation"],
        "reset_generation": stamp["reset_generation"],
        "sequence": 10,
        "physics_step": 20,
        "sim_time_ns": stamp["sim_time_ns"],
        "bodies": [],
    }
    with SimulationRecordingWriter(
        tmp_path,
        run_id="raw-measurement",
        session_id=stamp["session_id"],
        required_content={"sensor_payload"},
    ) as writer:
        writer.append(snapshot, sensor_payloads=[encoded])
    replay = SimulationReplay.open(tmp_path)
    reference = replay.frames[0].sensor_payloads[0]
    assert reference["metadata"] == encoded.metadata
    content = replay.read_sensor_payload(reference)
    assert content == encoded.payload
    restored = decode_measurement_payload(content, modality=modality)
    assert asdict(restored.stamp) == stamp
    with np.load(BytesIO(content), allow_pickle=False) as archive:
        assert all(not archive[name].dtype.hasobject for name in archive.files)


@pytest.mark.parametrize("modality", ["thermal", "acoustic", "semantic"])
def test_explicit_modality_cannot_relabel_a_measurement(modality):
    encoded = _encoded(modality)
    other = "acoustic" if modality == "thermal" else "thermal"
    with pytest.raises(ValueError):
        decode_measurement_payload(encoded.payload, modality=other)
    with pytest.raises(ValueError):
        encode_measurement_payload(decode_measurement_payload(encoded, modality=modality), modality=other)


@pytest.mark.parametrize("content", [b"not an NPZ", b"", b"PK\x03\x04broken"])
def test_invalid_archive_is_reported_as_value_error(content):
    with pytest.raises(ValueError):
        decode_measurement_payload(content, modality="thermal")
