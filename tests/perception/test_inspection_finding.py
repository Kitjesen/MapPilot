"""Visual observations are evaluated against the route's expected state."""

import pytest

from runtime.contracts.inspection_evidence import InspectionEvidenceStore, InspectionFinding


@pytest.mark.parametrize(("action", "observed", "verdict"), [
    ("capture:indicator_green", "green", "normal"),
    ("capture:indicator_green", "red", "anomaly"),
    ("capture:indicator_on", "green", "normal"),
    ("capture:indicator_off", "green", "anomaly"),
    ("capture:indicator_green", "on", "inconclusive"),
    ("capture:indicator_on", "closed", "inconclusive"),
    ("capture:door_closed", "closed", "normal"),
    ("capture:door_closed", "open", "anomaly"),
    ("capture:door_closed", "unknown", "inconclusive"),
])
def test_state_comparison_never_treats_unobserved_target_as_normal(tmp_path, action, observed, verdict):
    result = InspectionEvidenceStore(tmp_path).persist(
        dict(task_id="task-a", route_id="route-a", route_revision=1, map_id="map-a",
             map_content_epoch=1, point_index=0, request_id="capture-a", action=action,
             requested_at_s=100., deadline_s=130.),
        rgb_bytes=b"saved-image", media_type="image/jpeg",
        finding=InspectionFinding(observed_state=observed, source="test-reader"),
    )
    assert result.persistence_status == "persisted"
    assert result.analysis_support == "visual_observation"
    assert result.analysis_verdict == verdict
    assert result.manifest["analysis"]["observation"]["observed_state"] == observed


def test_unavailable_visual_reader_still_preserves_photo_for_human_review(tmp_path):
    result = InspectionEvidenceStore(tmp_path).persist(
        dict(task_id="task-a", route_id="route-a", route_revision=1, map_id="map-a",
             map_content_epoch=1, point_index=0, request_id="capture-a", action="capture:door_closed",
             requested_at_s=100., deadline_s=130.),
        rgb_bytes=b"saved-image", media_type="image/jpeg",
    )
    assert result.persistence_status == "persisted"
    assert result.analysis_verdict == "inconclusive"
    assert result.manifest["analysis"]["reason"] == "visual_reader_unavailable"
