"""A complete capture route does not imply normal equipment."""

import pytest

from gateway.schemas import InspectionTaskReportResponse
from gateway.services.inspection_report import build_inspection_task_report


def _report(*, review=None, analysis=None, evidence_status="VERIFIED"):
    route = dict(id="route-a", revision=1, map_id="map-a", map_content_epoch=1,
                 points=[dict(id="indicator", action="capture:overview")], loop_count=1)
    request = dict(task_id="task-a", route_id="route-a", route_revision=1, map_id="map-a",
                   map_content_epoch=1, point_index=0, action="capture:overview")
    evidence = dict(status=evidence_status, summary=dict(request=request, analysis=analysis or {
        "support": "not_applicable", "verdict": "not_evaluated", "reason": "capture_only",
    }))
    if review is not None:
        evidence["summary"]["review"] = dict(verdict=review)
    event = dict(kind=5, state=4, point_index=0, loop_index=0, evidence_id="capture-a")
    task = dict(task_id="task-a", current_state="SUCCESS", terminal=True,
                execution_confirmed=True, delivery=dict(history_complete=True),
                timeline=[event], latest_event=event)
    report = build_inspection_task_report(task, route, {"capture-a": evidence})
    InspectionTaskReportResponse.model_validate(report)
    return report


def test_saved_unanalysed_image_requires_review_even_when_capture_workflow_is_complete():
    report = _report()
    assert report["report_status"] == "COMPLETE"
    assert report["acceptance"] == "ACCEPTABLE"
    assert report["inspection_result"]["status"] == "NEEDS_REVIEW"
    assert report["inspection_result"]["unreviewed_points"] == 1


@pytest.mark.parametrize(("review", "state"), [
    ("normal", "NORMAL"), ("anomaly", "ANOMALY"), ("inconclusive", "INCONCLUSIVE"),
])
def test_human_finding_is_separate_from_successful_navigation(review, state):
    report = _report(review=review)
    assert report["execution"]["state"] == "SUCCESS"
    assert report["points"][0]["inspection_result"] == state
    assert report["inspection_result"]["status"] == state


def test_invalid_evidence_cannot_become_normal_from_a_review():
    report = _report(review="normal", evidence_status="INVALID")
    assert report["inspection_result"]["normal_points"] == 0
    assert report["inspection_result"]["not_inspected_points"] == 1


def test_supported_machine_finding_can_report_anomaly_and_human_review_is_recorded_separately():
    analysis = dict(support="trusted_observation", verdict="violation")
    assert _report(analysis=analysis)["inspection_result"]["status"] == "ANOMALY"
    assert _report(analysis=analysis, review="normal")["inspection_result"]["status"] == "NORMAL"
