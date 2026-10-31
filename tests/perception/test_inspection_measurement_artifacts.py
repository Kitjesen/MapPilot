"""Raw measurements share the inspection evidence transaction, not its verdict."""

from __future__ import annotations

import json
from dataclasses import FrozenInstanceError
from io import BytesIO
from pathlib import Path

import numpy as np
import pytest

from runtime.contracts.inspection_evidence import (
    EvidenceConflictError,
    EvidenceIntegrityError,
    EvidenceValidationError,
    InspectionEvidenceRequest,
    InspectionEvidenceStore,
    InspectionFinding,
    MeasurementArtifact,
)


def _request(action: str = "capture:overview", **overrides) -> InspectionEvidenceRequest:
    values = dict(
        task_id="task-a", route_id="route-a", route_revision=1, map_id="map-a",
        map_content_epoch=1, point_index=0, request_id="capture-a", action=action,
        requested_at_s=100.0, deadline_s=130.0,
    )
    return InspectionEvidenceRequest(**(values | overrides))


def _metadata(sensor_id: str = "sensor-a") -> dict:
    return {
        "units": {"temperature_k": "K", "pressure_pa": "Pa", "class_ids": "label"},
        "source_stamp": {
            "session_id": "session-a", "instance_id": "robot-a", "sensor_id": sensor_id,
            "frame_id": "sensor-frame", "model_generation": 1, "reset_generation": 2,
            "sequence": 3, "sim_time_ns": 100_000_000_000,
        },
        "calibration": {"calibration_id": "declared-test", "assumptions": ["test-only"]},
        "simulation": True, "ground_truth": True,
    }


def _measurement(modality: str = "thermal", sensor_id: str = "sensor-a", payload=None):
    if payload is None:
        stream = BytesIO()
        np.savez_compressed(stream, raw=np.array([[300.0, np.nan]], dtype=np.float32),
                            valid_mask=np.array([[True, False]]))
        payload = stream.getvalue()
    return MeasurementArtifact(
        modality=modality, sensor_id=sensor_id, payload=payload,
        media_type="application/x-npz", metadata=_metadata(sensor_id),
    )


def _capture(store, request, measurements=(), **kwargs):
    return store.persist(
        request, rgb_bytes=b"original-rgb", media_type="image/png",
        pose={"frame_id": "map", "x": 1.0, "y": 2.0, "yaw": 0.0},
        measurements=measurements, **kwargs,
    )


@pytest.mark.parametrize("modality", ["thermal", "acoustic", "semantic"])
def test_raw_measurement_roundtrip_with_rgb_pose_and_complete_metadata(tmp_path: Path, modality):
    measurement = _measurement(modality)
    saved = _capture(InspectionEvidenceStore(tmp_path), _request(), (measurement,))

    loaded = InspectionEvidenceStore(tmp_path).get(saved.request.request_id)
    records = loaded.manifest["persistence"]["artifacts"]
    assert {record["kind"] for record in records} == {"rgb", "pose", "measurement"}
    record = next(record for record in records if record["kind"] == "measurement")
    assert record["modality"] == modality
    assert record["sensor_id"] == measurement.sensor_id
    assert record["metadata"] == measurement.metadata
    assert record["media_type"] == "application/x-npz"
    assert record["bytes"] == len(measurement.payload)
    assert (loaded.evidence_dir / record["path"]).read_bytes() == measurement.payload
    assert (loaded.evidence_dir / "rgb.png").read_bytes() == b"original-rgb"
    assert (loaded.evidence_dir / "pose.json").is_file()
    assert not list(loaded.evidence_dir.parent.glob(".staging-*"))


@pytest.mark.parametrize("modality", ["thermal", "acoustic"])
def test_measurement_capture_requires_matching_modality_and_rgb_pose(tmp_path, modality):
    store = InspectionEvidenceStore(tmp_path)
    request = _request(f"capture:{modality}")
    matching = _measurement(modality)
    other = _measurement("acoustic" if modality == "thermal" else "thermal")
    for measurements in ((), (other,), (_measurement("semantic"),)):
        with pytest.raises(EvidenceValidationError, match="matching measurement"):
            _capture(store, request, measurements)
        assert list(store.requests_dir.iterdir()) == []
    for payload in (
        {"pose": {"frame_id": "map"}},
        {"rgb_bytes": b"rgb", "media_type": "image/png"},
        {"rgb_bytes": b"", "media_type": "image/png", "pose": {"frame_id": "map"}},
    ):
        with pytest.raises(EvidenceValidationError, match="RGB and pose"):
            store.persist(request, measurements=(matching,), **payload)
        assert list(store.requests_dir.iterdir()) == []
    saved = _capture(store, request, (matching,))
    assert saved.analysis_support == "unavailable"
    assert saved.analysis_verdict == "inconclusive"


@pytest.mark.parametrize("damage", ["units", "source_stamp", "calibration", "sensor_id", "frame_id", "sequence", "non_json"])
def test_measurement_metadata_requires_explicit_units_full_stamp_and_calibration(damage):
    metadata = _metadata()
    if damage in {"units", "source_stamp", "calibration"}:
        del metadata[damage]
    elif damage == "sensor_id":
        metadata["source_stamp"]["sensor_id"] = "different-sensor"
    elif damage == "frame_id":
        del metadata["source_stamp"]["frame_id"]
    elif damage == "sequence":
        metadata["source_stamp"]["sequence"] = -1
    else:
        metadata["calibration"]["bad"] = float("nan")
    with pytest.raises(EvidenceValidationError):
        MeasurementArtifact(modality="thermal", sensor_id="sensor-a", payload=b"opaque-npz",
                            media_type="application/x-npz", metadata=metadata)


def test_measurement_is_deeply_immutable_and_payload_is_opaque(tmp_path):
    metadata = _metadata()
    measurement = MeasurementArtifact(
        modality="thermal", sensor_id="sensor-a", payload=b"not parsed as an NPZ",
        media_type="application/x-npz", metadata=metadata,
    )
    expected = _metadata()
    metadata["calibration"]["assumptions"].append("mutated-input")
    measurement.metadata["source_stamp"]["sequence"] = 99
    measurement.metadata["calibration"]["assumptions"].append("mutated-output")
    with pytest.raises(FrozenInstanceError):
        measurement.payload = b"changed"
    result = _capture(InspectionEvidenceStore(tmp_path), _request("capture:thermal"), (measurement,))
    record = next(r for r in result.manifest["persistence"]["artifacts"] if r["kind"] == "measurement")
    assert record["metadata"] == expected
    assert (result.evidence_dir / record["path"]).read_bytes() == b"not parsed as an NPZ"


def test_duplicate_sensor_ids_are_rejected_before_any_commit(tmp_path):
    store = InspectionEvidenceStore(tmp_path)
    with pytest.raises(EvidenceValidationError, match="duplicate measurement sensor_id"):
        _capture(store, _request("capture:thermal"), (_measurement(), _measurement("acoustic")))
    assert list(store.requests_dir.iterdir()) == []
    result = _capture(store, _request("capture:thermal"),
                      (_measurement(), _measurement("thermal", "sensor-b"), _measurement("semantic", "sensor-c")))
    assert len(result.manifest["persistence"]["artifacts"]) == 5


def test_retries_preserve_first_raw_bytes_metadata_and_complete_request_identity(tmp_path):
    store = InspectionEvidenceStore(tmp_path)
    request = _request("capture:thermal")
    first = _capture(store, request, (_measurement(payload=b"first-raw"),))
    original = first.manifest_path.read_bytes()
    retry = _capture(store, request, (_measurement(payload=b"retry-raw"),))
    assert retry.manifest_path.read_bytes() == original
    assert store.persist(request).manifest_path.read_bytes() == original
    record = next(r for r in retry.manifest["persistence"]["artifacts"] if r["kind"] == "measurement")
    assert (retry.evidence_dir / record["path"]).read_bytes() == b"first-raw"
    for override in (
        {"task_id": "other"}, {"route_id": "other"}, {"route_revision": 2},
        {"map_id": "other"}, {"map_content_epoch": 2}, {"point_index": 2},
        {"action": "capture:acoustic"}, {"requested_at_s": 101.0}, {"deadline_s": 131.0},
    ):
        with pytest.raises(EvidenceConflictError):
            store.persist(_request(**({"action": "capture:thermal"} | override)))


@pytest.mark.parametrize("damage", [
    "missing_file", "length", "units", "source_stamp", "calibration", "duplicate_sensor",
    "missing_measurement", "wrong_modality", "missing_rgb", "missing_pose",
])
def test_read_and_retry_reject_incomplete_or_damaged_measurements(tmp_path, damage):
    store = InspectionEvidenceStore(tmp_path)
    saved = _capture(store, _request("capture:thermal"),
                     (_measurement(), _measurement("acoustic", "sensor-b")))
    manifest = json.loads(saved.manifest_path.read_text(encoding="utf-8"))
    records = manifest["persistence"]["artifacts"]
    measurement = next(r for r in records if r["kind"] == "measurement")
    path = saved.evidence_dir / measurement["path"]
    if damage == "missing_file":
        path.unlink()
    elif damage == "length":
        path.write_bytes(b"truncated")
    elif damage in {"units", "source_stamp", "calibration"}:
        del measurement["metadata"][damage]
    elif damage == "duplicate_sensor":
        other = records[-1]
        other["sensor_id"] = measurement["sensor_id"]
        other["metadata"]["source_stamp"]["sensor_id"] = measurement["sensor_id"]
    elif damage == "wrong_modality":
        measurement["modality"] = "semantic"
    else:
        kind = damage.removeprefix("missing_")
        manifest["persistence"]["artifacts"] = [r for r in records if r["kind"] != kind]
    saved.manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    for read in (lambda: store.get(saved.request.request_id), lambda: store.persist(saved.request)):
        with pytest.raises(EvidenceIntegrityError):
            read()


@pytest.mark.parametrize("modality", ["thermal", "acoustic", "semantic"])
def test_codec_metadata_and_vendor_npz_media_type_are_preserved_without_sim_imports(tmp_path, modality):
    generic = _metadata()
    metadata = {
        "schema": "lingtu.sim.measurement-payload.v1", "modality": modality,
        "stamp": generic["source_stamp"], "units": generic["units"],
        "arrays": {"raw": {"dtype": "<f4", "shape": [1, 2]}}, "metadata_tuple_paths": [],
    }
    if modality == "semantic":
        metadata.update(source_stamp=metadata["stamp"], calibration_metadata={"projection": "test"},
                        configuration={"calibration": {"width": 2, "height": 1}},
                        simulation=True, ground_truth=True)
    else:
        key = "camera_calibration" if modality == "thermal" else "calibration"
        metadata["frame_metadata"] = {key: generic["calibration"], "qualification": "test-only"}
    measurement = MeasurementArtifact(
        modality=modality, sensor_id="sensor-a", payload=b"producer-validated-opaque-npz",
        media_type="application/vnd.lingtu.measurement+npz", metadata=metadata,
    )
    result = _capture(InspectionEvidenceStore(tmp_path), _request(), (measurement,))
    record = next(r for r in result.manifest["persistence"]["artifacts"] if r["kind"] == "measurement")
    assert record["path"].endswith(".npz")
    assert record["media_type"] == measurement.media_type
    assert record["metadata"] == metadata


@pytest.mark.parametrize("modality", ["thermal", "acoustic"])
@pytest.mark.parametrize("verdict", ["normal", "anomaly", "inconclusive"])
def test_measurement_verdict_requires_explicit_caller_finding_and_model_provenance(tmp_path, modality, verdict):
    provenance = {"model_id": "test-analyzer", "model_version": "v1", "qualification": "test-only"}
    finding = InspectionFinding(
        observed_state=verdict, source="test-caller", modality=modality,
        model_provenance=provenance, reason="explicit caller result",
        details={"sensor_id": "sensor-a", "threshold_units": "K" if modality == "thermal" else "Pa"},
    )
    result = _capture(InspectionEvidenceStore(tmp_path), _request(f"capture:{modality}"),
                      (_measurement(modality),), finding=finding)
    assert result.analysis_verdict == verdict
    assert result.analysis_support == "measurement_observation"
    assert result.manifest["analysis"]["observation"]["model_provenance"] == provenance
    assert result.manifest["analysis"]["observation"]["modality"] == modality


@pytest.mark.parametrize("provenance", [None, {}, {"model_id": "test"}, {"model_id": "test", "model_version": ""}])
def test_measurement_finding_without_identified_model_is_invalid(provenance):
    with pytest.raises(EvidenceValidationError, match="provenance"):
        InspectionFinding(observed_state="normal", source="caller", modality="thermal", model_provenance=provenance)


@pytest.mark.parametrize("action", ["capture:acoustic", "capture:door_open", "capture:overview"])
def test_finding_for_another_modality_or_action_cannot_commit(tmp_path, action):
    store = InspectionEvidenceStore(tmp_path)
    finding = InspectionFinding(
        observed_state="normal", source="caller", modality="thermal",
        model_provenance={"model_id": "test", "model_version": "v1"},
    )
    with pytest.raises(EvidenceValidationError):
        _capture(store, _request(action), (_measurement("acoustic"),), finding=finding)
    assert list(store.requests_dir.iterdir()) == []


def test_recognition_shaped_json_is_not_an_inspection_finding(tmp_path):
    result = _capture(InspectionEvidenceStore(tmp_path), _request("capture:thermal"), (_measurement(),),
                      detections={"verdict": "normal", "model_id": "not-an-explicit-finding"})
    assert result.analysis_verdict == "inconclusive"


@pytest.mark.parametrize("override", [
    {"modality": "gas"}, {"sensor_id": "../outside"}, {"payload": b""},
    {"payload": bytearray(b"mutable")}, {"media_type": " "},
])
def test_invalid_measurement_contract_is_rejected(override):
    values = dict(modality="thermal", sensor_id="sensor-a", payload=b"raw", media_type="application/x-npz",
                  metadata=_metadata())
    with pytest.raises(EvidenceValidationError):
        MeasurementArtifact(**(values | override))


def test_atomic_publication_failure_leaves_no_partial_capture_and_can_retry(tmp_path, monkeypatch):
    store = InspectionEvidenceStore(tmp_path)
    measurements = (_measurement(), _measurement("acoustic", "sensor-b"), _measurement("semantic", "sensor-c"))
    rename = store._atomic_rename

    def fail_publication(source, destination):
        assert {path.suffix for path in source.iterdir()} == {".json", ".png", ".npz"}
        assert len(list(source.glob("measurement-*.npz"))) == 3
        raise OSError("injected publication failure")

    monkeypatch.setattr(store, "_atomic_rename", fail_publication)
    with pytest.raises(OSError, match="publication failure"):
        _capture(store, _request("capture:thermal"), measurements)
    assert list(store.requests_dir.iterdir()) == []
    monkeypatch.setattr(store, "_atomic_rename", rename)
    result = _capture(store, _request("capture:thermal"), measurements)
    assert store.get(result.request.request_id).manifest == result.manifest
