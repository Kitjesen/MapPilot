from __future__ import annotations

import pytest

from gateway.services.event_handlers import handle_inspection_task_event
from gateway.services.inspection_task_lifecycle import (
    InspectionTaskJournalUnavailable,
    InspectionTaskTimeline,
)


def _event(
    *,
    task_id: str = "inspection-task-42",
    boot_id: str = "navd-boot-a",
    event_sequence: int = 1,
    kind: int = 2,
    state: int = 2,
    request_id: str = "inspection-request-42",
    reason: str = "planning_route",
) -> dict[str, object]:
    return {
        "ts": 1000.0 + event_sequence,
        "frame_id": "map",
        "boot_id": boot_id,
        "event_sequence": event_sequence,
        "kind": kind,
        "task_id": task_id,
        "request_id": request_id,
        "command_request_id": request_id,
        "state": state,
        "map_id": "field-map",
        "map_version": 7,
        "route_id": "route-a",
        "route_revision": 3,
        "point_index": 0,
        "point_count": 2,
        "loop_index": 0,
        "retry_count": 0,
        "point_id": "dock",
        "action": "capture:overview",
        "action_request_id": "",
        "evidence_id": "",
        "reason": reason,
    }


def test_submission_receipt_does_not_claim_native_execution() -> None:
    timeline = InspectionTaskTimeline()

    timeline.record_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-request-42",
        route_id="route-a",
        map_id="field-map",
        map_version=7,
        route_revision=3,
    )

    status = timeline.query("inspection-task-42")

    assert status["found"] is True
    assert status["current_state"] == "SUBMISSION_ACCEPTED_AWAITING_NATIVE_EVENT"
    assert status["state_source"] == "business_ack_only"
    assert status["execution_confirmed"] is False
    assert status["terminal"] is False
    assert status["can_resume"] is False
    assert status["available_actions"] == []


def test_native_events_are_the_only_execution_and_terminal_truth() -> None:
    timeline = InspectionTaskTimeline()
    timeline.record_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-request-42",
        route_id="route-a",
        map_id="field-map",
        map_version=7,
        route_revision=3,
    )

    assert timeline.observe(_event(event_sequence=1, kind=1, state=2)) is True
    planning = timeline.query("inspection-task-42")
    assert planning["current_state"] == "PLANNING"
    assert planning["state_source"] == "native_task_event"
    assert planning["execution_confirmed"] is True
    assert planning["terminal"] is False
    assert planning["available_actions"] == ["pause", "cancel"]
    assert planning["progress"] == {
        "known": True,
        "completed_points": 0,
        "point_count": 2,
        "current_point_number": 1,
        "current_point_id": "dock",
        "loop_number": 1,
        "retry_count": 0,
        "action": "capture:overview",
        "evidence_id": "",
    }

    assert timeline.observe(_event(event_sequence=2, state=12, reason="operator_pause")) is True
    pausing = timeline.query("inspection-task-42")
    assert pausing["current_state"] == "PAUSING"
    assert pausing["terminal"] is False
    assert pausing["can_resume"] is False

    assert timeline.observe(_event(event_sequence=3, state=5, reason="stopped")) is True
    paused = timeline.query("inspection-task-42")
    assert paused["current_state"] == "PAUSED"
    assert paused["terminal"] is False
    assert paused["can_resume"] is True
    assert paused["available_actions"] == ["resume", "cancel"]

    assert timeline.observe(_event(event_sequence=4, state=13, reason="operator_cancel")) is True
    cancelling = timeline.query("inspection-task-42")
    assert cancelling["current_state"] == "CANCELLING"
    assert cancelling["terminal"] is False

    assert timeline.observe(_event(event_sequence=5, state=9, reason="native_stop_confirmed")) is True
    cancelled = timeline.query("inspection-task-42")
    assert cancelled["current_state"] == "CANCELLED"
    assert cancelled["terminal"] is True
    assert cancelled["terminal_source"] == "native_task_event"
    assert cancelled["available_actions"] == []


def test_delivery_gaps_and_endpoint_restart_are_visible_not_silently_inferred() -> None:
    timeline = InspectionTaskTimeline()
    assert timeline.observe(_event(event_sequence=1, state=3)) is True
    assert timeline.observe(_event(event_sequence=3, state=5)) is True

    gapped = timeline.query("inspection-task-42")
    assert gapped["delivery"]["history_complete"] is False
    assert gapped["delivery"]["reason"] == "event_sequence_gap"
    assert gapped["available_actions"] == []

    assert timeline.observe(
        _event(
            task_id="inspection-task-next",
            boot_id="navd-boot-b",
            event_sequence=1,
            state=2,
        )
    ) is True
    interrupted = timeline.query("inspection-task-42")
    assert interrupted["terminal"] is False
    assert interrupted["delivery"]["continuity"] == "endpoint_restart_observed"
    assert interrupted["delivery"]["reason"] == "endpoint_restarted"
    assert interrupted["available_actions"] == []


def test_late_first_event_does_not_claim_a_complete_history() -> None:
    timeline = InspectionTaskTimeline()

    assert timeline.observe(_event(event_sequence=2, state=3)) is True

    status = timeline.query("inspection-task-42")
    assert status["current_state"] == "NAVIGATING"
    assert status["delivery"]["history_complete"] is False
    assert status["delivery"]["reason"] == "event_sequence_gap"
    assert status["available_actions"] == []


def test_retention_truncation_is_visible_as_incomplete_history() -> None:
    timeline = InspectionTaskTimeline(event_retention=2)

    assert timeline.observe(_event(event_sequence=1, state=2)) is True
    assert timeline.observe(_event(event_sequence=2, state=3)) is True
    assert timeline.observe(_event(event_sequence=3, state=4)) is True

    status = timeline.query("inspection-task-42")
    assert len(status["timeline"]) == 2
    assert status["delivery"]["history_complete"] is False
    assert status["delivery"]["reason"] == "event_retention_exceeded"
    assert status["available_actions"] == []


def test_endpoint_restart_makes_submission_only_task_outcome_unknown() -> None:
    timeline = InspectionTaskTimeline()
    timeline.record_submission(
        task_id="inspection-task-submitted",
        action="start",
        request_id="start-request-submitted",
        route_id="route-a",
        map_id="field-map",
        map_version=7,
        route_revision=3,
    )
    assert timeline.observe(_event(task_id="other-task", event_sequence=1)) is True
    assert timeline.observe(
        _event(
            task_id="new-boot-task",
            boot_id="navd-boot-b",
            event_sequence=1,
        )
    ) is True

    status = timeline.query("inspection-task-submitted")
    assert status["current_state"] == "INTERRUPTED_AWAITING_NATIVE_TRUTH"
    assert status["state_source"] == "continuity_monitor"
    assert status["execution_confirmed"] is False
    assert status["terminal"] is False
    assert status["reason"] == "endpoint_restarted"
    assert status["available_actions"] == []


def test_duplicate_or_out_of_order_native_events_do_not_replace_latest_truth() -> None:
    timeline = InspectionTaskTimeline()
    assert timeline.observe(_event(event_sequence=2, state=2, reason="planning")) is True
    assert timeline.observe(_event(event_sequence=1, state=9, reason="forged_terminal")) is False

    status = timeline.query("inspection-task-42")
    assert status["current_state"] == "PLANNING"
    assert status["reason"] == "planning"
    assert len(status["timeline"]) == 1


def test_native_event_cannot_redefine_a_frozen_task_identity() -> None:
    timeline = InspectionTaskTimeline()
    timeline.record_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-request-42",
        route_id="route-a",
        map_id="field-map",
        map_version=7,
        route_revision=3,
    )
    assert timeline.observe(_event(event_sequence=1, state=2)) is True
    conflicting_terminal = _event(
        event_sequence=2,
        state=7,
        reason="wrong_route_succeeded",
    )
    conflicting_terminal["route_id"] = "route-b"

    assert timeline.observe(conflicting_terminal) is False

    status = timeline.query("inspection-task-42")
    assert status["current_state"] == "PLANNING"
    assert status["terminal"] is False
    assert status["latest_event"]["event_sequence"] == 1
    assert status["delivery"]["history_complete"] is False
    assert status["delivery"]["continuity"] == "identity_conflict"
    assert status["delivery"]["reason"] == "native_event_identity_conflict"
    assert status["available_actions"] == []


def test_gateway_projects_native_event_and_emits_the_same_fact_to_sse() -> None:
    class Gateway:
        def __init__(self) -> None:
            self._inspection_task_timeline = InspectionTaskTimeline()
            self.events: list[dict[str, object]] = []

        def push_event(self, event: dict[str, object]) -> None:
            self.events.append(event)

    gateway = Gateway()
    native_event = _event(event_sequence=1, kind=1, state=2)

    handle_inspection_task_event(gateway, native_event)

    status = gateway._inspection_task_timeline.query("inspection-task-42")
    assert status["current_state"] == "PLANNING"
    assert gateway.events == [
        {
            "type": "inspection_task_event",
            "data": status["latest_event"],
        }
    ]


def test_terminal_task_fact_is_queryable_after_gateway_restart(tmp_path) -> None:
    journal = tmp_path / "inspection_tasks.json"
    timeline = InspectionTaskTimeline(journal_path=journal)
    timeline.record_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-request-42",
        route_id="route-a",
        map_id="field-map",
        map_version=7,
        route_revision=3,
        route_snapshot={
            "id": "route-a",
            "map_id": "field-map",
            "map_version": 7,
            "revision": 3,
            "loop_count": 1,
            "failure_policy": "stop",
            "points": [
                {
                    "id": "dock",
                    "action": "capture:overview",
                    "enabled": True,
                }
            ],
        },
    )
    assert timeline.observe(_event(event_sequence=1, kind=1, state=2)) is True
    assert timeline.observe(
        _event(event_sequence=2, state=9, reason="native_stop_confirmed")
    ) is True

    restored = InspectionTaskTimeline(journal_path=journal)
    status = restored.query("inspection-task-42")

    assert status["found"] is True
    assert status["current_state"] == "CANCELLED"
    assert status["state_source"] == "persisted_native_task_event"
    assert status["execution_confirmed"] is True
    assert status["terminal"] is True
    assert status["terminal_source"] == "native_task_event"
    assert status["route_snapshot"] == {
        "id": "route-a",
        "map_id": "field-map",
        "map_version": 7,
        "revision": 3,
        "loop_count": 1,
        "failure_policy": "stop",
        "points": [
            {
                "id": "dock",
                "action": "capture:overview",
                "enabled": True,
            }
        ],
    }
    assert status["delivery"]["retention"] == "durable_gateway_projection"
    assert restored.health()["journal"]["status"] == "ready"


def test_request_id_binding_survives_restart_before_native_admission(tmp_path) -> None:
    journal = tmp_path / "inspection_tasks.json"
    timeline = InspectionTaskTimeline(journal_path=journal)
    timeline.reserve_submission(
        task_id="inspection-task-42",
        action="pause",
        request_id="inspection-control-once",
        reason="operator_hold",
    )

    restored = InspectionTaskTimeline(journal_path=journal)
    restored.reserve_submission(
        task_id="inspection-task-42",
        action="pause",
        request_id="inspection-control-once",
        reason="operator_hold",
    )
    with pytest.raises(ValueError, match="different action"):
        restored.reserve_submission(
            task_id="inspection-task-42",
            action="cancel",
            request_id="inspection-control-once",
            reason="operator_hold",
        )

    health = restored.health()
    assert health["request_bindings"] == 1
    assert health["journal"]["status"] == "ready"


def test_request_id_binding_freezes_route_requirements_before_native_admission() -> None:
    timeline = InspectionTaskTimeline()
    route_snapshot = {
        "id": "route-a",
        "map_id": "field-map",
        "map_version": 7,
        "revision": 3,
        "loop_count": 1,
        "failure_policy": "stop",
        "points": [
            {
                "id": "dock",
                "action": "",
                "enabled": True,
            }
        ],
    }
    timeline.reserve_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-start-once",
        route_snapshot=route_snapshot,
    )
    changed_requirements = {
        **route_snapshot,
        "points": [
            {
                "id": "dock",
                "action": "capture:overview",
                "enabled": True,
            }
        ],
    }

    with pytest.raises(ValueError, match="different route_snapshot_sha256"):
        timeline.reserve_submission(
            task_id="inspection-task-42",
            action="start",
            request_id="inspection-start-once",
            route_snapshot=changed_requirements,
        )


def test_active_task_waits_for_native_reconciliation_after_gateway_restart(
    tmp_path,
) -> None:
    journal = tmp_path / "inspection_tasks.json"
    timeline = InspectionTaskTimeline(journal_path=journal)
    assert timeline.observe(_event(event_sequence=1, kind=1, state=3)) is True

    restored = InspectionTaskTimeline(journal_path=journal)
    recovered = restored.query("inspection-task-42")

    assert recovered["current_state"] == "RECOVERED_AWAITING_NATIVE_RECONCILIATION"
    assert recovered["state_source"] == "durable_task_projection"
    assert recovered["execution_confirmed"] is False
    assert recovered["terminal"] is False
    assert recovered["reason"] == "gateway_restarted_awaiting_native_reconciliation"
    assert recovered["latest_event"]["state_name"] == "NAVIGATING"
    assert recovered["delivery"]["continuity"] == (
        "gateway_restart_awaiting_native_reconciliation"
    )
    assert recovered["available_actions"] == []

    assert restored.observe(
        _event(event_sequence=2, state=5, reason="native_pause_confirmed")
    ) is True
    reconciled = restored.query("inspection-task-42")
    assert reconciled["current_state"] == "PAUSED"
    assert reconciled["state_source"] == "native_task_event"
    assert reconciled["execution_confirmed"] is True
    assert reconciled["available_actions"] == ["resume", "cancel"]


def test_corrupt_task_journal_is_visible_and_never_returns_a_forged_state(
    tmp_path,
) -> None:
    journal = tmp_path / "inspection_tasks.json"
    journal.write_text('{"body":{},"sha256":"not-valid"}\n', encoding="utf-8")

    restored = InspectionTaskTimeline(journal_path=journal)
    status = restored.query("inspection-task-42")

    assert status["found"] is False
    assert status["current_state"] == "UNKNOWN"
    assert status["execution_confirmed"] is False
    assert status["terminal"] is False
    assert status["reason"] == "task_journal_corrupt"
    assert status["delivery"]["retention"] == "durable_gateway_projection"
    assert restored.health()["journal"]["status"] == "corrupt"
    assert restored.health()["journal"]["error"]


def test_field_timeline_factory_uses_the_declared_durable_journal(
    monkeypatch,
    tmp_path,
) -> None:
    from gateway.services.inspection_task_lifecycle import (
        create_inspection_task_timeline,
    )

    journal = tmp_path / "field" / "inspection_tasks.json"
    monkeypatch.setenv("LINGTU_INSPECTION_TASK_JOURNAL", str(journal))
    first = create_inspection_task_timeline()
    assert first.observe(_event(event_sequence=1, kind=1, state=9)) is True

    restored = create_inspection_task_timeline()
    status = restored.query("inspection-task-42")

    assert status["terminal"] is True
    assert status["state_source"] == "persisted_native_task_event"
    assert restored.health()["retention"] == "durable_gateway_projection"


def test_journal_write_failure_is_visible_without_publishing_an_unstored_fact(
    monkeypatch,
    tmp_path,
) -> None:
    import gateway.services.inspection_task_lifecycle as lifecycle

    def fail_write(_path, _payload) -> None:
        raise OSError("disk full")

    monkeypatch.setattr(lifecycle, "_atomic_write", fail_write)

    class Gateway:
        def __init__(self) -> None:
            self._inspection_task_timeline = InspectionTaskTimeline(
                journal_path=tmp_path / "inspection_tasks.json"
            )
            self.events: list[dict[str, object]] = []

        def push_event(self, event: dict[str, object]) -> None:
            self.events.append(event)

    gateway = Gateway()
    handle_inspection_task_event(gateway, _event(event_sequence=1, kind=1, state=2))

    assert gateway.events == [
        {
            "type": "inspection_task_journal_error",
            "data": {
                "task_id": "inspection-task-42",
                "status": "write_failed",
                "error": "inspection task journal write failed: disk full",
            },
        }
    ]
    status = gateway._inspection_task_timeline.query("inspection-task-42")
    assert status["found"] is False
    assert status["reason"] == "task_journal_write_failed"
    with pytest.raises(InspectionTaskJournalUnavailable):
        gateway._inspection_task_timeline.list_tasks()
