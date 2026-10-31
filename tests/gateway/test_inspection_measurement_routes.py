"""Measurement downloads select sensor identity within verified evidence."""

from __future__ import annotations

import errno
import json
from dataclasses import replace
from io import BytesIO
from types import SimpleNamespace

import numpy as np
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes.inspection import register_inspection_routes
from runtime.contracts.inspection_evidence import (
    InspectionEvidenceRequest,
    InspectionEvidenceStore,
    MeasurementArtifact,
)

BASE = "/api/v1/inspection/evidence"


@pytest.fixture
def capture(monkeypatch, tmp_path, request):
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_DIR", str(tmp_path))
    measurements = []
    for index, sensor_id in enumerate(("thermal-front", "thermal-rear")):
        modality = getattr(request, "param", ("thermal", "thermal"))[index]
        array_name, unit = {
            "thermal": ("temperature_k", "K"),
            "acoustic": ("pressure_pa", "Pa"),
            "semantic": ("class_ids", "label"),
        }[modality]
        stream = BytesIO()
        raw = np.array([[index + 1]], dtype=np.int32 if modality == "semantic" else np.float32)
        np.savez_compressed(stream, **{array_name: raw})
        measurements.append(MeasurementArtifact(
            modality=modality, sensor_id=sensor_id, payload=stream.getvalue(),
            media_type=("application/x-npz" if index == 0 else "application/vnd.lingtu.measurement+npz"),
            metadata={
                "units": {array_name: unit},
                "source_stamp": {
                    "session_id": "session-a", "instance_id": "robot-a", "sensor_id": sensor_id,
                    "frame_id": sensor_id, "model_generation": 1, "reset_generation": 2,
                    "sequence": index + 1, "sim_time_ns": 100_000_000_000,
                },
                "calibration": {"calibration_id": "test-only"},
            },
        ))
    saved = InspectionEvidenceStore(tmp_path).persist(
        InspectionEvidenceRequest(
            task_id="task-a", route_id="route-a", route_revision=1, map_id="map-a",
            map_content_epoch=1, point_index=0, request_id="evidence-a",
            action=("capture:thermal" if measurements[0].modality == "thermal" else "capture:acoustic"),
            requested_at_s=100.0, deadline_s=130.0,
        ),
        rgb_bytes=b"original-rgb", media_type="image/png",
        pose={"frame_id": "map", "x": 1.0, "y": 2.0},
        detections={"objects": []}, measurements=tuple(measurements),
    )
    app = FastAPI()
    register_inspection_routes(app, SimpleNamespace())
    with TestClient(app) as client:
        yield client, saved, measurements


def test_download_selects_each_sensor_not_first_measurement(capture):
    client, saved, measurements = capture
    for measurement in reversed(measurements):
        response = client.get(f"{BASE}/{saved.request.request_id}/measurements/{measurement.sensor_id}")
        assert response.status_code == 200
        assert response.content == measurement.payload
        assert response.headers["content-type"] == measurement.media_type


def test_detail_and_summary_expose_per_sensor_urls_and_original_metadata(capture):
    client, saved, measurements = capture
    detail = client.get(f"{BASE}/{saved.request.request_id}")
    listed = client.get(BASE)
    assert detail.status_code == listed.status_code == 200
    summary = detail.json()["evidence"]
    assert listed.json()["evidence"] == [summary]
    records = [record for record in summary["artifacts"] if record["kind"] == "measurement"]
    assert len(records) == 2
    assert {record["sensor_id"] for record in records} == {item.sensor_id for item in measurements}
    assert len({record["download_url"] for record in records}) == 2
    for record, measurement in zip(records, measurements):
        assert record == {
            "kind": "measurement", "modality": measurement.modality, "sensor_id": measurement.sensor_id,
            "bytes": len(measurement.payload), "media_type": measurement.media_type,
            "metadata": measurement.metadata,
            "download_url": f"{BASE}/{saved.request.request_id}/measurements/{measurement.sensor_id}",
        }
        assert client.get(record["download_url"]).content == measurement.payload
    assert all("path" not in record for record in summary["artifacts"])


@pytest.mark.parametrize("capture", [("acoustic", "semantic")], indirect=True)
def test_other_measurement_modalities_use_the_same_raw_download_contract(capture):
    client, saved, measurements = capture
    summary = client.get(f"{BASE}/{saved.request.request_id}").json()["evidence"]
    records = [record for record in summary["artifacts"] if record["kind"] == "measurement"]
    assert {record["modality"] for record in records} == {"acoustic", "semantic"}
    for record, measurement in zip(records, measurements):
        response = client.get(record["download_url"])
        assert response.status_code == 200
        assert response.content == measurement.payload
        assert response.headers["content-type"] == measurement.media_type
        assert record["metadata"] == measurement.metadata


@pytest.mark.parametrize("legacy", [False, True])
def test_existing_rgb_pose_detections_and_kind_rejection_unchanged(capture, legacy):
    client, saved, _ = capture
    if legacy:
        saved = InspectionEvidenceStore(saved.evidence_dir.parent.parent).persist(
            replace(saved.request, request_id="legacy-evidence", action="capture:overview"),
            rgb_bytes=b"original-rgb", media_type="image/png",
            pose={"frame_id": "map", "x": 1.0, "y": 2.0}, detections={"objects": []},
        )
    base = f"{BASE}/{saved.request.request_id}"
    rgb = client.get(f"{base}/artifacts/rgb")
    assert rgb.status_code == 200
    assert rgb.content == b"original-rgb"
    assert rgb.headers["content-type"] == "image/png"
    for kind, expected in (("pose", {"frame_id": "map", "x": 1.0, "y": 2.0}),
                           ("detections", {"objects": []})):
        response = client.get(f"{base}/artifacts/{kind}")
        assert response.status_code == 200
        assert response.headers["content-type"] == "application/json"
        assert response.json() == expected
    response = client.get(f"{base}/artifacts/measurement")
    assert response.status_code == 400
    assert response.json()["error"] == "inspection_evidence_artifact_kind_invalid"
    if legacy:
        assert client.get(f"{base}/measurements/thermal-front").status_code == 404


@pytest.mark.parametrize(("evidence_id", "sensor_id"), [
    ("missing-evidence", "thermal-front"), ("evidence-a", "missing-sensor"),
    ("bad$name", "thermal-front"), ("evidence-a", "bad$name"),
    ("evidence-a", "bad%5Cname"), ("evidence-a", "bad%2Fname"),
    ("evidence-a", "_invalid"), ("evidence-a", "a" * 129),
])
def test_unknown_or_malformed_identity_is_not_a_file_path(capture, evidence_id, sensor_id):
    client, _, _ = capture
    response = client.get(f"{BASE}/{evidence_id}/measurements/{sensor_id}")
    assert response.status_code == 404


@pytest.mark.parametrize("damage", [
    "missing_file", "length", "manifest_json", "units", "stamp", "duplicate_sensor", "path_escape",
])
def test_damaged_capture_blocks_download_detail_and_listing(capture, damage):
    client, saved, _ = capture
    manifest = json.loads(saved.manifest_path.read_text(encoding="utf-8"))
    records = [item for item in manifest["persistence"]["artifacts"] if item["kind"] == "measurement"]
    record = records[1]
    path = saved.evidence_dir / record["path"]
    if damage == "missing_file":
        path.unlink()
    elif damage == "length":
        path.write_bytes(b"truncated")
    elif damage == "units":
        del record["metadata"]["units"]
    elif damage == "stamp":
        del record["metadata"]["source_stamp"]
    elif damage == "duplicate_sensor":
        record["sensor_id"] = records[0]["sensor_id"]
        record["metadata"]["source_stamp"]["sensor_id"] = records[0]["sensor_id"]
    elif damage == "path_escape":
        record["path"] = "../outside.npz"
    saved.manifest_path.write_text(
        "{broken-json" if damage == "manifest_json" else json.dumps(manifest), encoding="utf-8",
    )
    base = f"{BASE}/{saved.request.request_id}"
    for suffix in ("", "/measurements/thermal-front", "/measurements/thermal-rear", "/artifacts/rgb"):
        response = client.get(f"{base}{suffix}")
        assert response.status_code == 409
        assert response.json()["error"] == "inspection_evidence_integrity_failed"
    listed = client.get(BASE)
    assert listed.status_code == 200
    assert listed.json()["evidence"] == []
    assert listed.json()["integrity_failures"] == 1


@pytest.mark.parametrize("damage", ["length", "missing_file"])
def test_file_damage_after_store_verification_is_still_blocked(capture, monkeypatch, damage):
    import gateway.routes.inspection as inspection

    client, _, _ = capture

    class DamagingStore(InspectionEvidenceStore):
        def get(self, request_id):
            result = super().get(request_id)
            record = next(item for item in result.manifest["persistence"]["artifacts"]
                          if item.get("sensor_id") == "thermal-rear")
            path = result.evidence_dir / record["path"]
            if damage == "length":
                path.write_bytes(b"replaced")
            else:
                path.unlink()
            return result

    monkeypatch.setattr(inspection, "InspectionEvidenceStore", DamagingStore)
    response = client.get(f"{BASE}/evidence-a/measurements/thermal-rear")
    assert response.status_code == 409
    assert response.json()["error"] == "inspection_evidence_integrity_failed"


def test_measurement_storage_read_failure_remains_unavailable(capture, monkeypatch):
    import gateway.routes.inspection as inspection

    client, _, _ = capture

    def fail_read(_path):
        raise OSError(errno.EIO, "injected storage failure")

    monkeypatch.setattr(inspection, "_read_regular_file_bytes", fail_read)
    response = client.get(f"{BASE}/evidence-a/measurements/thermal-rear")
    assert response.status_code == 503
    assert response.json()["error"] == "inspection_evidence_unavailable"
