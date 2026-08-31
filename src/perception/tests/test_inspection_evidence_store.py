from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any

import pytest

from perception.inspection import (
    EvidenceConflictError,
    EvidenceIntegrityError,
    EvidenceValidationError,
    InspectionEvidenceRequest,
    InspectionEvidenceStore,
    TrustedParkingObservation,
)


def _request(**overrides: Any) -> InspectionEvidenceRequest:
    values = {
        "run_id": "run-001",
        "route_id": "route-main",
        "route_revision": 7,
        "map_id": "field-map",
        "map_content_epoch": 4,
        "point_id": "point-01",
        "point_index": 0,
        "request_id": "request-001",
        "action": "capture:overview",
        "requested_at_s": 1_720_000_000.0,
        "deadline_s": 1_720_000_010.0,
    }
    values.update(overrides)
    return InspectionEvidenceRequest(**values)


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_overview_commit_persists_auditable_artifacts_and_manifest(tmp_path: Path) -> None:
    store = InspectionEvidenceStore(tmp_path)
    rgb = b"\xff\xd8inspection-frame\xff\xd9"

    result = store.persist(
        _request(),
        rgb_bytes=rgb,
        media_type="image/jpeg",
        pose={"frame_id": "map", "x": 1.25, "y": -0.5, "yaw": 0.2},
        detections=[{"label": "car", "confidence": 0.92}],
    )

    assert result.evidence_dir == tmp_path / "requests" / "request-001"
    assert result.manifest_path == result.evidence_dir / "manifest.json"
    assert result.persistence_status == "persisted"
    assert result.analysis_support == "not_applicable"
    assert result.analysis_verdict == "not_evaluated"
    assert (result.evidence_dir / "rgb.jpg").read_bytes() == rgb

    manifest = _load_json(result.manifest_path)
    assert manifest["schema_version"] == "lingtu.inspection.evidence.v1"
    assert manifest["request"] == {
        "action": "capture:overview",
        "deadline_s": 1_720_000_010.0,
        "map_id": "field-map",
        "map_content_epoch": 4,
        "point_id": "point-01",
        "point_index": 0,
        "request_id": "request-001",
        "route_id": "route-main",
        "route_revision": 7,
        "run_id": "run-001",
        "requested_at_s": 1_720_000_000.0,
    }
    assert manifest["persistence"]["persisted"] is True
    assert manifest["analysis"] == {
        "reason": "capture_only",
        "support": "not_applicable",
        "verdict": "not_evaluated",
    }

    artifact_by_name = {
        artifact["path"]: artifact for artifact in manifest["persistence"]["artifacts"]
    }
    assert set(artifact_by_name) == {"rgb.jpg", "pose.json", "detections.json"}
    for name, record in artifact_by_name.items():
        payload = (result.evidence_dir / name).read_bytes()
        assert record["bytes"] == len(payload)
        assert set(record) == {"kind", "path", "media_type", "bytes"}

    assert {path.name for path in result.evidence_dir.iterdir()} == {
        "manifest.json",
        "rgb.jpg",
        "pose.json",
        "detections.json",
    }
    assert not list((tmp_path / "requests").glob(".staging-*"))


def test_request_mapping_is_supported_and_rejects_unknown_fields(tmp_path: Path) -> None:
    store = InspectionEvidenceStore(tmp_path)
    request = {
        "run_id": "run-001",
        "route_id": "route-main",
        "route_revision": 7,
        "map_id": "field-map",
        "map_content_epoch": 4,
        "point_id": "point-01",
        "point_index": 0,
        "request_id": "request-mapping",
        "action": "capture:overview",
        "requested_at_s": 1_720_000_000.0,
        "deadline_s": 1_720_000_010.0,
    }

    result = store.persist(request)
    assert result.request.request_id == "request-mapping"

    with pytest.raises(EvidenceValidationError, match="unknown request fields"):
        store.persist({**request, "request_id": "request-extra", "path": "../escape"})


def test_same_request_id_is_idempotent_and_conflicting_identity_is_rejected(
    tmp_path: Path,
) -> None:
    store = InspectionEvidenceStore(tmp_path)
    request = _request(request_id="request-idempotent")
    first = store.persist(request, rgb_bytes=b"first", media_type="image/png")
    first_manifest = first.manifest_path.read_bytes()

    second = store.persist(request, rgb_bytes=b"different retry body", media_type="image/png")

    assert second.manifest_path.read_bytes() == first_manifest
    assert (second.evidence_dir / "rgb.png").read_bytes() == b"first"

    conflicting_fields = (
        {"run_id": "run-002"},
        {"route_id": "route-alternate"},
        {"route_revision": 8},
        {"map_id": "field-map-v2"},
        {"map_content_epoch": 5},
        {"point_id": "point-02"},
        {"point_index": 1},
        {"action": "capture:parking"},
        {"requested_at_s": 1_720_000_001.0},
        {"deadline_s": 1_720_000_011.0},
    )
    for override in conflicting_fields:
        with pytest.raises(EvidenceConflictError, match="request_id already committed"):
            store.persist(_request(request_id="request-idempotent", **override))


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("run_id", "../run"),
        ("route_id", "route/subroute"),
        ("map_id", "../field-map"),
        ("point_id", r"point\escape"),
        ("request_id", "."),
        ("request_id", ""),
        ("action", "capture:../../passwd"),
        ("action", "inspect"),
    ],
)
def test_identifiers_and_actions_reject_path_traversal(
    tmp_path: Path,
    field: str,
    value: str,
) -> None:
    store = InspectionEvidenceStore(tmp_path)
    values = {
        "run_id": "run-001",
        "route_id": "route-main",
        "route_revision": 7,
        "map_id": "field-map",
        "map_content_epoch": 4,
        "point_id": "point-01",
        "point_index": 0,
        "request_id": "request-safe",
        "action": "capture:overview",
        "requested_at_s": 1_720_000_000.0,
        "deadline_s": 1_720_000_010.0,
    }
    values[field] = value

    with pytest.raises(EvidenceValidationError):
        store.persist(values)

    assert not (tmp_path.parent / "run").exists()


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("route_revision", 0),
        ("route_revision", -1),
        ("route_revision", 2**64),
        ("route_revision", 1.5),
        ("route_revision", True),
        ("map_content_epoch", -1),
        ("map_content_epoch", 1.5),
        ("map_content_epoch", False),
        ("point_index", -1),
        ("point_index", 2**32),
        ("point_index", 0.5),
        ("point_index", True),
        ("requested_at_s", 0.0),
        ("requested_at_s", -1.0),
        ("requested_at_s", float("nan")),
        ("requested_at_s", float("inf")),
        ("requested_at_s", float("-inf")),
        ("deadline_s", 0.0),
        ("deadline_s", -1.0),
        ("deadline_s", float("nan")),
        ("deadline_s", float("inf")),
        ("deadline_s", float("-inf")),
        ("deadline_s", 1_720_000_000.0),
        ("deadline_s", 1_719_999_999.0),
    ],
)
def test_audit_identity_rejects_invalid_numeric_boundaries(
    tmp_path: Path,
    field: str,
    value: Any,
) -> None:
    store = InspectionEvidenceStore(tmp_path)

    with pytest.raises(EvidenceValidationError):
        store.persist(_request(**{field: value}))


def test_request_mapping_does_not_default_missing_audit_fields(tmp_path: Path) -> None:
    store = InspectionEvidenceStore(tmp_path)
    complete = _request(request_id="missing-audit-field").to_dict()

    for field in (
        "route_revision",
        "map_id",
        "map_content_epoch",
        "point_index",
        "requested_at_s",
        "deadline_s",
    ):
        incomplete = dict(complete)
        incomplete.pop(field)
        with pytest.raises(EvidenceValidationError, match="missing request fields"):
            store.persist(incomplete)


def test_parking_requires_explicit_trusted_observation_for_a_verdict(tmp_path: Path) -> None:
    store = InspectionEvidenceStore(tmp_path)

    unsupported = store.persist(
        _request(request_id="parking-unknown", action="capture:parking")
    )
    assert unsupported.analysis_support == "unavailable"
    assert unsupported.analysis_verdict == "inconclusive"
    assert unsupported.manifest["analysis"]["reason"] == "trusted_observation_missing"

    trusted = store.persist(
        _request(request_id="parking-trusted", action="capture:parking"),
        trusted_observation=TrustedParkingObservation(
            source="parking_detector_v1",
            verdict="violation",
            confidence=0.93,
            details={"vehicle_track_id": "track-7", "zone_id": "fire-lane"},
        ),
    )
    assert trusted.analysis_support == "trusted_observation"
    assert trusted.analysis_verdict == "violation"
    assert trusted.manifest["analysis"]["observation"]["source"] == "parking_detector_v1"

    with pytest.raises(EvidenceValidationError, match="TrustedParkingObservation"):
        store.persist(
            _request(request_id="parking-untrusted", action="capture:parking"),
            trusted_observation={"verdict": "violation"},  # type: ignore[arg-type]
        )


@pytest.mark.parametrize("action", ["capture:bin_full", "capture:plate_ocr"])
def test_unimplemented_analyzers_never_fabricate_a_result(
    tmp_path: Path,
    action: str,
) -> None:
    store = InspectionEvidenceStore(tmp_path)
    result = store.persist(_request(request_id=action.replace(":", "-"), action=action))

    assert result.analysis_support == "unavailable"
    assert result.analysis_verdict == "inconclusive"
    assert result.manifest["analysis"]["reason"] == "analyzer_not_integrated"


def test_media_extension_comes_only_from_explicit_media_type(tmp_path: Path) -> None:
    store = InspectionEvidenceStore(tmp_path)

    result = store.persist(
        _request(request_id="raw-rgb"),
        rgb_bytes=b"\x01\x02\x03",
        media_type="application/x-raw-rgb",
    )
    assert (result.evidence_dir / "rgb.rgb").read_bytes() == b"\x01\x02\x03"

    with pytest.raises(EvidenceValidationError, match="media_type"):
        store.persist(_request(request_id="missing-media"), rgb_bytes=b"pixels")
    with pytest.raises(EvidenceValidationError, match="unsupported media_type"):
        store.persist(
            _request(request_id="unsafe-media"),
            rgb_bytes=b"pixels",
            media_type="image/../../escape",
        )


def test_write_and_rename_interruptions_do_not_damage_committed_evidence(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import runtime.contracts.inspection_evidence as evidence_store_module

    store = InspectionEvidenceStore(tmp_path)
    committed = store.persist(_request(request_id="committed"), pose={"x": 1.0})
    committed_manifest = committed.manifest_path.read_bytes()

    original_write = evidence_store_module._write_bytes_fsync

    def fail_during_write(path: Path, payload: bytes) -> None:
        if path.name == "pose.json":
            raise OSError("simulated write interruption")
        original_write(path, payload)

    monkeypatch.setattr(evidence_store_module, "_write_bytes_fsync", fail_during_write)
    with pytest.raises(OSError, match="write interruption"):
        store.persist(_request(request_id="write-failed"), pose={"x": 2.0})
    assert not (tmp_path / "requests" / "write-failed").exists()
    assert committed.manifest_path.read_bytes() == committed_manifest

    monkeypatch.setattr(evidence_store_module, "_write_bytes_fsync", original_write)

    def fail_rename(_source: Path, _destination: Path) -> None:
        raise OSError("simulated rename interruption")

    monkeypatch.setattr(store, "_atomic_rename", fail_rename)
    with pytest.raises(OSError, match="rename interruption"):
        store.persist(_request(request_id="rename-failed"))
    assert not (tmp_path / "requests" / "rename-failed").exists()
    assert store.get("committed").manifest_path.read_bytes() == committed_manifest


def test_stale_staging_cleanup_preserves_recent_and_committed_directories(tmp_path: Path) -> None:
    store = InspectionEvidenceStore(tmp_path, staging_ttl_seconds=30.0)
    committed = store.persist(_request(request_id="keep-committed"))
    staging_parent = tmp_path / "requests"
    stale = staging_parent / ".staging-stale"
    recent = staging_parent / ".staging-recent"
    stale.mkdir()
    recent.mkdir()
    old = time.time() - 120.0
    os.utime(stale, (old, old))

    removed = store.cleanup_staging(now=time.time())

    assert stale in removed
    assert not stale.exists()
    assert recent.exists()
    assert committed.evidence_dir.exists()


def test_commit_fsyncs_staging_and_parent_directory(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import runtime.contracts.inspection_evidence as evidence_store_module

    calls: list[Path] = []
    monkeypatch.setattr(
        evidence_store_module,
        "_fsync_directory",
        lambda path: calls.append(Path(path)),
    )
    store = InspectionEvidenceStore(tmp_path)

    result = store.persist(_request(request_id="fsync-proof"))

    assert result.evidence_dir in calls
    assert tmp_path / "requests" in calls


def test_get_rejects_tampered_artifacts_and_manifest(tmp_path: Path) -> None:
    artifact_store = InspectionEvidenceStore(tmp_path / "artifact-case")
    artifact_result = artifact_store.persist(
        _request(request_id="tampered-artifact"),
        rgb_bytes=b"original",
        media_type="image/jpeg",
    )
    (artifact_result.evidence_dir / "rgb.jpg").write_bytes(b"tampered")

    with pytest.raises(EvidenceIntegrityError, match="artifact byte count mismatch"):
        artifact_store.get("tampered-artifact")

    manifest_store = InspectionEvidenceStore(tmp_path / "manifest-case")
    manifest_result = manifest_store.persist(_request(request_id="tampered-manifest"))
    manifest = _load_json(manifest_result.manifest_path)
    manifest["schema_version"] = "unsupported"
    manifest_result.manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(EvidenceIntegrityError, match="schema_version"):
        manifest_store.get("tampered-manifest")


def test_get_rejects_symlinked_manifest_and_artifact(tmp_path: Path) -> None:
    artifact_store = InspectionEvidenceStore(tmp_path / "artifact-symlink")
    artifact_result = artifact_store.persist(
        _request(request_id="symlink-artifact"),
        rgb_bytes=b"same-content",
        media_type="image/jpeg",
    )
    artifact = artifact_result.evidence_dir / "rgb.jpg"
    artifact_target = tmp_path / "artifact-target.jpg"
    artifact_target.write_bytes(artifact.read_bytes())
    artifact.unlink()
    try:
        os.symlink(artifact_target, artifact)
    except OSError as exc:
        pytest.skip(f"symlink creation unavailable: {exc}")

    with pytest.raises(EvidenceIntegrityError, match="artifact is not a regular file"):
        artifact_store.get("symlink-artifact")

    manifest_store = InspectionEvidenceStore(tmp_path / "manifest-symlink")
    manifest_result = manifest_store.persist(_request(request_id="symlink-manifest"))
    manifest = manifest_result.manifest_path
    manifest_target = tmp_path / "manifest-target.json"
    manifest_target.write_bytes(manifest.read_bytes())
    manifest.unlink()
    os.symlink(manifest_target, manifest)

    with pytest.raises(EvidenceIntegrityError, match="manifest is not a regular file"):
        manifest_store.get("symlink-manifest")
