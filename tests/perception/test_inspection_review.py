"""Human findings survive restarts without changing captured evidence."""

import json

import pytest

from runtime.contracts.inspection_evidence import EvidenceValidationError, InspectionEvidenceStore


def _capture(store):
    return store.persist(
        dict(task_id="task-a", route_id="route-a", route_revision=1, map_id="map-a",
             map_content_epoch=1, point_index=0, request_id="capture-a", action="capture:overview",
             requested_at_s=100., deadline_s=130.),
        rgb_bytes=b"saved-image", media_type="image/jpeg",
    )


def test_review_is_durable_and_does_not_rewrite_capture_or_machine_analysis(tmp_path):
    store = InspectionEvidenceStore(tmp_path)
    captured = _capture(store)
    original = captured.manifest_path.read_bytes()
    reviewed = store.review("capture-a", verdict="normal", note="Indicator is green")

    assert reviewed.manifest["review"]["verdict"] == "normal"
    assert reviewed.manifest["review"]["note"] == "Indicator is green"
    assert reviewed.manifest["review"]["reviewed_at"]
    assert reviewed.analysis_verdict == "not_evaluated"
    assert captured.manifest_path.read_bytes() == original
    assert (captured.evidence_dir / "rgb.jpg").read_bytes() == b"saved-image"
    restored = InspectionEvidenceStore(tmp_path).get("capture-a")
    assert restored.manifest["review"] == reviewed.manifest["review"]


def test_corrected_finding_preserves_review_history_and_duplicate_click_is_idempotent(tmp_path):
    store = InspectionEvidenceStore(tmp_path)
    captured = _capture(store)
    store.review("capture-a", verdict="anomaly", note="Needs another look")
    store.review("capture-a", verdict="normal", note="Checked the full image")
    store.review("capture-a", verdict="normal", note="Checked the full image")

    history = json.loads((captured.evidence_dir / "reviews.json").read_text())
    assert [item["verdict"] for item in history] == ["anomaly", "normal"]
    assert store.get("capture-a").manifest["review"]["verdict"] == "normal"


@pytest.mark.parametrize("verdict", ["", "success", "not_evaluated"])
def test_review_does_not_accept_capture_success_as_device_finding(tmp_path, verdict):
    store = InspectionEvidenceStore(tmp_path)
    _capture(store)
    with pytest.raises(EvidenceValidationError):
        store.review("capture-a", verdict=verdict)


def test_missing_evidence_cannot_receive_a_normal_finding(tmp_path):
    with pytest.raises(FileNotFoundError):
        InspectionEvidenceStore(tmp_path).review("missing", verdict="normal")
