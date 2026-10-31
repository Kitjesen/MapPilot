from __future__ import annotations

import json
import threading
import time

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
        "state": state,
        "map_id": "field-map",
        "map_content_epoch": 7,
        "route_id": "route-a",
        "route_revision": 3,
        "point_index": 0,
        "point_count": 2,
        "loop_index": 0,
        "retry_count": 0,
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
        map_content_epoch=7,
        route_revision=3,
    )

    status = timeline.query("inspection-task-42")

    assert status["found"] is True
    assert status["current_state"] is None
    assert status["state_available"] is False
    assert status["phase"] is None
    assert status["transition"] is None
    assert status["state_source"] == "business_ack_only"
    assert status["execution_confirmed"] is False
    assert status["terminal"] is False
    assert status["can_resume"] is False
    assert status["available_actions"] == []


def test_query_projects_internal_navigation_phase_as_public_execution_state() -> None:
    timeline = InspectionTaskTimeline()

    assert timeline.observe(_event(event_sequence=1, state=3)) is True

    status = timeline.query("inspection-task-42")

    assert status["current_state"] == "EXECUTING"
    assert status["phase"] == "NAVIGATING"
    assert status["transition"] is None


@pytest.mark.parametrize(
    ("enabled", "point_index", "completed", "current_number"),
    [
        ([False, True], 1, 0, 1),
        ([False, True, False, True], 3, 1, 2),
        ([True, False, True], 2, 1, 2),
    ],
)
def test_progress_counts_only_enabled_points_from_the_bound_route(
    enabled: list[bool], point_index: int, completed: int, current_number: int,
) -> None:
    timeline = InspectionTaskTimeline()
    timeline.record_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-request-42",
        route_snapshot={
            "id": "route-a", "map_id": "field-map", "map_content_epoch": 7,
            "revision": 3, "loop_count": 1,
            "points": [
                {"id": f"point-{index}", "enabled": value, "action": f"capture:point-{index}"}
                for index, value in enumerate(enabled)
            ],
        },
    )
    assert timeline.observe({
        **_event(state=3), "point_index": point_index, "point_count": sum(enabled),
    })

    progress = timeline.query("inspection-task-42")["progress"]

    assert progress["known"] is True
    assert progress["completed_points"] == completed
    assert progress["point_count"] == sum(enabled)
    assert progress["current_point_number"] == current_number
    assert progress["current_point_id"] == f"point-{point_index}"
    assert progress["action"] == f"capture:point-{point_index}"


def test_progress_preserves_point_phases_loop_reset_and_success_sentinel() -> None:
    timeline = InspectionTaskTimeline()
    timeline.record_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-request-42",
        route_snapshot={
            "id": "route-a", "map_id": "field-map", "map_content_epoch": 7,
            "revision": 3, "loop_count": 2,
            "points": [
                {"id": "disabled-first", "enabled": False},
                {"id": "first", "action": "capture:first"},
                {"id": "last", "action": "capture:last"},
                {"id": "disabled-last", "enabled": False},
            ],
        },
    )
    events = [
        (3, 1, 0, 0, 1, "first"),
        (10, 1, 0, 0, 1, "first"),
        (4, 1, 0, 0, 1, "first"),
        (11, 1, 0, 0, 1, "first"),
        (2, 2, 0, 1, 2, "last"),
        (2, 1, 1, 0, 1, "first"),
        (7, 2, 2, 2, None, ""),
    ]
    for sequence, (state, index, loop, completed, number, point_id) in enumerate(events, 1):
        assert timeline.observe({
            **_event(event_sequence=sequence, state=state),
            "point_index": index, "point_count": 2, "loop_index": loop,
        })
        status = timeline.query("inspection-task-42")
        progress = status["progress"]
        assert progress["completed_points"] == completed
        assert progress["point_count"] == 2
        assert progress["current_point_number"] == number
        assert progress["current_point_id"] == point_id
        assert progress["action"] == (f"capture:{point_id}" if point_id else "")
        assert progress["loop_number"] == (loop if state == 7 else loop + 1)
        assert status["terminal"] is (state == 7)


@pytest.mark.parametrize("state", [8, 9])
def test_failed_or_cancelled_progress_keeps_the_unfinished_enabled_point(state: int) -> None:
    timeline = InspectionTaskTimeline()
    timeline.record_submission(
        task_id="inspection-task-42", action="start", request_id="inspection-request-42",
        route_snapshot={
            "id": "route-a", "map_id": "field-map", "map_content_epoch": 7,
            "revision": 3, "loop_count": 2,
            "points": [
                {"id": "disabled", "enabled": False},
                {"id": "unfinished", "action": "capture:unfinished"},
            ],
        },
    )
    assert timeline.observe({
        **_event(state=state), "point_index": 1, "point_count": 1, "loop_index": 1,
    })
    status = timeline.query("inspection-task-42")
    progress = status["progress"]
    assert status["terminal"] is True
    assert progress["known"] is True
    assert progress["completed_points"] == 0
    assert progress["current_point_number"] == 1
    assert progress["current_point_id"] == "unfinished"
    assert progress["action"] == "capture:unfinished"
    assert progress["loop_number"] == 2


def test_progress_without_a_bound_route_does_not_treat_raw_index_as_completion() -> None:
    timeline = InspectionTaskTimeline()
    assert timeline.observe({**_event(state=3), "point_index": 1, "point_count": 1})
    progress = timeline.query("inspection-task-42")["progress"]
    assert progress["known"] is False
    assert progress["completed_points"] == 0
    assert progress["point_count"] == 1
    assert progress["current_point_number"] is None
    assert progress["current_point_id"] == ""


@pytest.mark.parametrize(
    ("native_state", "public_state", "phase"),
    [
        (1, "PLANNING", "VALIDATING"),
        (2, "PLANNING", "PLANNING"),
        (3, "EXECUTING", "NAVIGATING"),
        (4, "EXECUTING", "DWELLING"),
        (5, "PAUSED", "PAUSED"),
        (6, "RECOVERING", "RECOVERING"),
        (7, "SUCCESS", "SUCCEEDED"),
        (8, "FAILED", "FAILED"),
        (9, "CANCELLED", "CANCELLED"),
        (10, "EXECUTING", "SETTLING"),
        (11, "EXECUTING", "ACTION_PENDING"),
    ],
)
def test_confirmed_native_phases_use_the_canonical_public_task_states(
    native_state: int,
    public_state: str,
    phase: str,
) -> None:
    timeline = InspectionTaskTimeline()

    assert timeline.observe(_event(event_sequence=1, state=native_state)) is True

    status = timeline.query("inspection-task-42")

    assert status["current_state"] == public_state
    assert status["state_available"] is True
    assert status["phase"] == phase
    assert status["transition"] is None


def test_runtime_idle_is_rejected_as_an_inspection_task_state() -> None:
    timeline = InspectionTaskTimeline()

    assert timeline.observe(_event(event_sequence=1, state=0)) is False


def test_stopping_phases_keep_the_last_confirmed_public_state() -> None:
    timeline = InspectionTaskTimeline()
    assert timeline.observe(_event(event_sequence=1, state=2)) is True
    assert timeline.observe(_event(event_sequence=2, state=12)) is True

    pausing = timeline.query("inspection-task-42")

    assert pausing["current_state"] == "PLANNING"
    assert pausing["state_available"] is True
    assert pausing["phase"] == "PAUSING"
    assert pausing["transition"] == "PAUSE_REQUESTED"

    assert timeline.observe(_event(event_sequence=3, state=5)) is True
    assert timeline.observe(_event(event_sequence=4, state=13)) is True

    cancelling = timeline.query("inspection-task-42")

    assert cancelling["current_state"] == "PAUSED"
    assert cancelling["state_available"] is True
    assert cancelling["phase"] == "CANCELLING"
    assert cancelling["transition"] == "CANCEL_REQUESTED"


def test_transition_without_a_confirmed_predecessor_does_not_guess_state() -> None:
    timeline = InspectionTaskTimeline()

    assert timeline.observe(_event(event_sequence=2, state=12)) is True

    status = timeline.query("inspection-task-42")

    assert status["current_state"] is None
    assert status["state_available"] is False
    assert status["phase"] == "PAUSING"
    assert status["transition"] == "PAUSE_REQUESTED"
    assert status["delivery"]["history_complete"] is False


def test_native_events_are_the_only_execution_and_terminal_truth() -> None:
    timeline = InspectionTaskTimeline()
    timeline.record_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-request-42",
        route_id="route-a",
        map_id="field-map",
        map_content_epoch=7,
        route_revision=3,
    )

    assert timeline.observe(_event(event_sequence=1, kind=1, state=2)) is True
    planning = timeline.query("inspection-task-42")
    assert planning["current_state"] == "PLANNING"
    assert planning["phase"] == "PLANNING"
    assert planning["transition"] is None
    assert planning["state_source"] == "native_task_event"
    assert planning["execution_confirmed"] is True
    assert planning["terminal"] is False
    assert planning["available_actions"] == ["pause", "cancel"]
    assert planning["progress"] == {
        "known": False,
        "completed_points": 0,
        "point_count": 2,
        "current_point_number": None,
        "current_point_id": "",
        "loop_number": 1,
        "retry_count": 0,
        "action": "capture:overview",
        "evidence_id": "",
    }

    assert timeline.observe(_event(event_sequence=2, state=12, reason="operator_pause")) is True
    pausing = timeline.query("inspection-task-42")
    assert pausing["current_state"] == "PLANNING"
    assert pausing["phase"] == "PAUSING"
    assert pausing["transition"] == "PAUSE_REQUESTED"
    assert pausing["terminal"] is False
    assert pausing["can_resume"] is False

    assert timeline.observe(_event(event_sequence=3, state=5, reason="stopped")) is True
    paused = timeline.query("inspection-task-42")
    assert paused["current_state"] == "PAUSED"
    assert paused["phase"] == "PAUSED"
    assert paused["transition"] is None
    assert paused["terminal"] is False
    assert paused["can_resume"] is True
    assert paused["available_actions"] == ["resume", "cancel"]

    assert timeline.observe(_event(event_sequence=4, state=13, reason="operator_cancel")) is True
    cancelling = timeline.query("inspection-task-42")
    assert cancelling["current_state"] == "PAUSED"
    assert cancelling["phase"] == "CANCELLING"
    assert cancelling["transition"] == "CANCEL_REQUESTED"
    assert cancelling["terminal"] is False

    assert timeline.observe(_event(event_sequence=5, state=9, reason="native_stop_confirmed")) is True
    cancelled = timeline.query("inspection-task-42")
    assert cancelled["current_state"] == "CANCELLED"
    assert cancelled["phase"] == "CANCELLED"
    assert cancelled["transition"] is None
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
    assert gapped["available_actions"] == ["resume", "cancel"]

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
    assert status["current_state"] == "EXECUTING"
    assert status["phase"] == "NAVIGATING"
    assert status["delivery"]["history_complete"] is False
    assert status["delivery"]["reason"] == "event_sequence_gap"
    assert status["available_actions"] == ["pause", "cancel"]


def test_retention_truncation_is_visible_as_incomplete_history() -> None:
    timeline = InspectionTaskTimeline(event_retention=2)

    assert timeline.observe(_event(event_sequence=1, state=2)) is True
    assert timeline.observe(_event(event_sequence=2, state=3)) is True
    assert timeline.observe(_event(event_sequence=3, state=4)) is True

    status = timeline.query("inspection-task-42")
    assert len(status["timeline"]) == 2
    assert status["delivery"]["history_complete"] is False
    assert status["delivery"]["reason"] == "event_retention_exceeded"
    assert status["available_actions"] == ["pause", "cancel"]


@pytest.mark.parametrize(
    ("native_state", "reason"),
    [(3, "restart_resume_required"), (14, "restart_loop_wait_resume_required")],
)
def test_native_checkpoint_restart_preserves_history_and_resume(
    tmp_path, native_state: int, reason: str
) -> None:
    journal = tmp_path / "inspection_tasks.json"
    timeline = InspectionTaskTimeline(journal_path=journal)
    assert timeline.observe(_event(event_sequence=1, state=1))
    assert timeline.observe(_event(event_sequence=2, state=native_state))
    timeline.record_submission(
        task_id="unreconciled-task", action="start", request_id="unreconciled-start",
        route_id="route-a", map_id="field-map", map_content_epoch=7, route_revision=3,
    )
    restored = InspectionTaskTimeline(journal_path=journal)
    assert restored.query("inspection-task-42")["available_actions"] == []
    assert restored.observe(
        _event(boot_id="navd-boot-b", event_sequence=3, state=5, reason=reason)
    )
    status = restored.query("inspection-task-42")
    assert status["current_state"] == "PAUSED"
    assert status["available_actions"] == ["resume", "cancel"]
    assert status["delivery"]["continuity"] == "verified"
    assert status["delivery"]["history_complete"] is True
    assert [item["event_sequence"] for item in status["timeline"]] == [1, 2, 3]
    assert status["delivery"]["reason"] == ""
    unrelated = restored.query("unreconciled-task")
    assert unrelated["current_state"] is None
    assert unrelated["delivery"]["history_complete"] is False
    assert unrelated["available_actions"] == []


def test_terminal_checkpoint_replay_does_not_duplicate_or_truncate_history() -> None:
    timeline = InspectionTaskTimeline(event_retention=2)
    assert timeline.observe(_event(event_sequence=1, state=3))
    terminal = _event(event_sequence=2, state=9, reason="native_stop_confirmed")
    assert timeline.observe(terminal)
    assert timeline.observe({**terminal, "boot_id": "navd-boot-b"})
    status = timeline.query("inspection-task-42")
    assert status["current_state"] == "CANCELLED"
    assert status["delivery"]["history_complete"] is True
    assert len(status["timeline"]) == 2
    assert status["latest_event"]["boot_id"] == "navd-boot-b"
    assert status["available_actions"] == []
    assert timeline.observe({**terminal, "boot_id": "navd-boot-b"}) is False


def test_checkpoint_recovery_keeps_missing_history_but_allows_native_resume() -> None:
    timeline = InspectionTaskTimeline()
    assert timeline.observe(_event(event_sequence=1, state=3))
    assert timeline.observe(_event(event_sequence=3, state=3))
    assert timeline.observe(
        _event(boot_id="navd-boot-b", event_sequence=4, state=5,
               reason="restart_resume_required")
    )
    status = timeline.query("inspection-task-42")
    assert status["current_state"] == "PAUSED"
    assert status["available_actions"] == ["resume", "cancel"]
    assert status["delivery"]["history_complete"] is False
    assert status["delivery"]["reason"] == "event_sequence_gap"
    assert [item["event_sequence"] for item in status["timeline"]] == [1, 3, 4]


@pytest.mark.parametrize(
    ("native_state", "actions"),
    [(2, ["pause", "cancel"]), (3, ["pause", "cancel"]),
     (5, ["resume", "cancel"]), (14, ["pause", "cancel"])],
)
def test_missing_report_history_does_not_veto_fresh_native_controls(
    native_state: int, actions: list[str]
) -> None:
    timeline = InspectionTaskTimeline()
    assert timeline.observe(_event(event_sequence=8, state=native_state))
    status = timeline.query("inspection-task-42")
    assert status["state_available"] is True
    assert status["delivery"]["history_complete"] is False
    assert status["available_actions"] == actions


def test_waiting_next_loop_has_execution_phase_and_operator_controls() -> None:
    from gateway.schemas import InspectionTaskStatusResponse

    timeline = InspectionTaskTimeline()
    assert timeline.observe(_event(state=14, reason="waiting_next_loop"))
    status = timeline.query("inspection-task-42")
    assert status["current_state"] == "EXECUTING"
    assert status["phase"] == "WAITING_NEXT_LOOP"
    assert status["available_actions"] == ["pause", "cancel"]
    assert InspectionTaskStatusResponse.model_validate(status).phase == "WAITING_NEXT_LOOP"


def test_unknown_state_or_unbound_identity_does_not_offer_controls() -> None:
    timeline = InspectionTaskTimeline()
    assert timeline.observe(_event(state=99)) is False
    assert timeline.query("inspection-task-42")["available_actions"] == []
    assert timeline.observe(_event(event_sequence=2, state=12))
    unknown_current = timeline.query("inspection-task-42")
    assert unknown_current["current_state"] is None
    assert unknown_current["available_actions"] == []
    unbound = InspectionTaskTimeline()
    assert unbound.observe({**_event(state=5), "map_id": ""})
    assert unbound.query("inspection-task-42")["available_actions"] == []


def test_checkpoint_restart_cannot_rebind_identity_or_fill_a_sequence_gap() -> None:
    timeline = InspectionTaskTimeline()
    assert timeline.observe(_event(event_sequence=1, state=3))
    changed = _event(boot_id="navd-boot-b", event_sequence=2, state=5,
                     reason="restart_resume_required")
    changed["map_content_epoch"] = 8
    assert timeline.observe(changed) is False
    status = timeline.query("inspection-task-42")
    assert status["delivery"]["continuity"] == "identity_conflict"
    assert status["available_actions"] == []

    missing = InspectionTaskTimeline()
    assert missing.observe(_event(event_sequence=1, state=3))
    assert missing.observe(_event(boot_id="navd-boot-b", event_sequence=3, state=5,
                                 reason="restart_resume_required"))
    incomplete = missing.query("inspection-task-42")
    assert incomplete["delivery"]["history_complete"] is False
    assert incomplete["available_actions"] == ["resume", "cancel"]


def test_unrecognized_boot_transition_keeps_history_incomplete() -> None:
    timeline = InspectionTaskTimeline()
    assert timeline.observe(_event(event_sequence=1, state=3))
    assert timeline.observe(_event(boot_id="navd-boot-b", event_sequence=2, state=5,
                                  reason="operator_pause"))
    status = timeline.query("inspection-task-42")
    assert status["delivery"]["history_complete"] is False
    assert status["current_state"] == "PAUSED"
    assert status["available_actions"] == ["resume", "cancel"]


def test_endpoint_restart_makes_submission_only_task_outcome_unknown() -> None:
    timeline = InspectionTaskTimeline()
    timeline.record_submission(
        task_id="inspection-task-submitted",
        action="start",
        request_id="start-request-submitted",
        route_id="route-a",
        map_id="field-map",
        map_content_epoch=7,
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
    assert status["current_state"] is None
    assert status["state_available"] is False
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
        map_content_epoch=7,
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
        map_content_epoch=7,
        route_revision=3,
        route_snapshot={
            "id": "route-a",
            "map_id": "field-map",
            "map_content_epoch": 7,
            "revision": 3,
            "loop_count": 1,
            "loop_interval_s": 42.125,
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
        "map_content_epoch": 7,
        "revision": 3,
        "loop_count": 1,
        "loop_interval_s": 42.125,
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


def test_product_session_binding_survives_restart_and_is_immutable(tmp_path) -> None:
    journal = tmp_path / "inspection_tasks.json"
    timeline = InspectionTaskTimeline(journal_path=journal)
    timeline.record_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-start-session-a",
        product_session_id="session-a",
    )

    assert timeline.query("inspection-task-42")["product_session_id"] == "session-a"
    restored = InspectionTaskTimeline(journal_path=journal)
    assert restored.query("inspection-task-42")["product_session_id"] == "session-a"

    with pytest.raises(ValueError, match="product_session_id"):
        restored.record_submission(
            task_id="inspection-task-42",
            action="pause",
            request_id="inspection-pause-session-b",
            product_session_id="session-b",
        )


def test_legacy_journal_without_product_session_binding_remains_readable(
    tmp_path,
) -> None:
    journal = tmp_path / "inspection_tasks.json"
    timeline = InspectionTaskTimeline(journal_path=journal)
    timeline.record_submission(
        task_id="legacy-task",
        action="start",
        request_id="legacy-start",
    )
    payload = json.loads(journal.read_text(encoding="utf-8"))
    payload["tasks"][0].pop("product_session_id", None)
    payload["request_bindings"][0].pop("product_session_id", None)
    journal.write_text(json.dumps(payload), encoding="utf-8")

    restored = InspectionTaskTimeline(journal_path=journal)

    assert restored.health()["journal"]["status"] == "ready"
    assert restored.query("legacy-task")["product_session_id"] is None


def test_request_id_binding_freezes_route_requirements_before_native_admission() -> None:
    timeline = InspectionTaskTimeline()
    route_snapshot = {
        "id": "route-a",
        "map_id": "field-map",
        "map_content_epoch": 7,
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

    with pytest.raises(ValueError, match="different route_snapshot"):
        timeline.reserve_submission(
            task_id="inspection-task-42",
            action="start",
            request_id="inspection-start-once",
            route_snapshot=changed_requirements,
        )

    changed_geometry = {
        **route_snapshot,
        "points": [
            {
                "id": "dock",
                "x": 1.0,
                "y": 2.0,
                "z": 0.0,
                "action": "",
                "enabled": True,
            }
        ],
    }
    with pytest.raises(ValueError, match="different route_snapshot"):
        timeline.reserve_submission(
            task_id="inspection-task-42",
            action="start",
            request_id="inspection-start-once",
            route_snapshot=changed_geometry,
        )


def test_query_preserves_frozen_native_route_geometry_and_full_point_indices(
    tmp_path,
) -> None:
    journal = tmp_path / "inspection_tasks.json"
    timeline = InspectionTaskTimeline(journal_path=journal)
    route_snapshot = {
        "id": "route-a",
        "map_id": "field-map",
        "map_content_epoch": 7,
        "revision": 3,
        "loop_count": 1,
        "failure_policy": "stop",
        "points": [
            {
                "id": "disabled-staging",
                "x": 1.25,
                "y": -2.5,
                "z": 0.75,
                "yaw": 1.5,
                "frame_id": "map",
                "position_tolerance_m": 0.2,
                "dwell_s": 4.0,
                "action": "capture:staging",
                "enabled": False,
            },
            {
                "id": "live-target",
                "x": 8.0,
                "y": 9.0,
                "z": 1.0,
                "frame_id": "map",
                "position_tolerance_m": 0.35,
                "dwell_s": 0.0,
                "action": "capture:overview",
                "enabled": True,
            },
        ],
    }
    timeline.record_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-request-42",
        route_snapshot=route_snapshot,
    )
    route_snapshot["points"][1]["x"] = 999.0
    assert timeline.observe(
        {
            **_event(state=3),
            "point_index": 1,
            "point_count": 1,
        }
    )

    status = InspectionTaskTimeline(journal_path=journal).query("inspection-task-42")

    assert status["route_snapshot"]["points"] == [
        {
            "id": "disabled-staging",
            "action": "capture:staging",
            "enabled": False,
            "x": 1.25,
            "y": -2.5,
            "z": 0.75,
            "frame_id": "map",
            "yaw": 1.5,
            "position_tolerance_m": 0.2,
            "dwell_s": 4.0,
        },
        {
            "id": "live-target",
            "action": "capture:overview",
            "enabled": True,
            "x": 8.0,
            "y": 9.0,
            "z": 1.0,
            "frame_id": "map",
            "position_tolerance_m": 0.35,
            "dwell_s": 0.0,
        },
    ]
    assert status["progress"]["current_point_id"] == "live-target"
    assert status["progress"]["current_point_number"] == 1


def test_task_status_response_includes_route_snapshot() -> None:
    from gateway.schemas import InspectionTaskStatusResponse

    assert "route_snapshot" in InspectionTaskStatusResponse.model_fields
    timeline = InspectionTaskTimeline()
    timeline.record_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-request-42",
        route_snapshot={
            "id": "route-a",
            "map_id": "field-map",
            "map_content_epoch": 7,
            "revision": 3,
            "loop_count": 1,
            "failure_policy": "stop",
            "points": [
                {
                    "id": "dock",
                    "x": 1.0,
                    "y": 2.0,
                    "z": 3.0,
                    "action": "capture:overview",
                    "enabled": True,
                }
            ],
        },
    )

    response = InspectionTaskStatusResponse.model_validate(
        timeline.query("inspection-task-42")
    ).model_dump()

    assert response["route_snapshot"]["points"][0] == {
        "id": "dock",
        "action": "capture:overview",
        "enabled": True,
        "x": 1.0,
        "y": 2.0,
        "z": 3.0,
        "frame_id": "map",
    }


def test_legacy_route_snapshot_without_geometry_remains_without_a_target() -> None:
    timeline = InspectionTaskTimeline()
    timeline.record_submission(
        task_id="inspection-task-42",
        action="start",
        request_id="inspection-request-42",
        route_snapshot={
            "id": "route-a",
            "map_id": "field-map",
            "map_content_epoch": 7,
            "revision": 3,
            "loop_count": 1,
            "failure_policy": "stop",
            "points": [
                {
                    "id": "legacy-point",
                    "action": "capture:overview",
                    "enabled": True,
                }
            ],
        },
    )

    point = timeline.query("inspection-task-42")["route_snapshot"]["points"][0]

    assert point == {
        "id": "legacy-point",
        "action": "capture:overview",
        "enabled": True,
    }


@pytest.mark.parametrize(
    "geometry",
    [
        {"x": 1.0},
        {"x": 1.0, "y": 2.0},
        {"x": float("nan"), "y": 2.0, "z": 0.0},
        {"x": 1.0, "y": 2.0, "z": float("inf")},
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": float("-inf")},
        {
            "x": 1.0,
            "y": 2.0,
            "z": 0.0,
            "position_tolerance_m": float("nan"),
        },
        {"x": 1.0, "y": 2.0, "z": 0.0, "dwell_s": float("inf")},
    ],
)
def test_route_snapshot_rejects_partial_or_nonfinite_geometry(
    geometry: dict[str, float],
) -> None:
    timeline = InspectionTaskTimeline()
    route_snapshot = {
        "id": "route-a",
        "map_id": "field-map",
        "map_content_epoch": 7,
        "revision": 3,
        "loop_count": 1,
        "failure_policy": "stop",
        "points": [
            {
                "id": "bad-point",
                "action": "capture:overview",
                "enabled": True,
                **geometry,
            }
        ],
    }

    with pytest.raises(ValueError, match="route point"):
        timeline.record_submission(
            task_id="inspection-task-42",
            action="start",
            request_id="inspection-request-42",
            route_snapshot=route_snapshot,
        )


def test_active_task_waits_for_native_reconciliation_after_gateway_restart(
    tmp_path,
) -> None:
    journal = tmp_path / "inspection_tasks.json"
    timeline = InspectionTaskTimeline(journal_path=journal)
    assert timeline.observe(_event(event_sequence=1, kind=1, state=3)) is True

    restored = InspectionTaskTimeline(journal_path=journal)
    recovered = restored.query("inspection-task-42")

    assert recovered["current_state"] is None
    assert recovered["state_available"] is False
    assert recovered["phase"] == "NAVIGATING"
    assert recovered["transition"] is None
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
    journal.write_text('{"schema_version":"wrong"}\n', encoding="utf-8")

    restored = InspectionTaskTimeline(journal_path=journal)
    status = restored.query("inspection-task-42")

    assert status["found"] is False
    assert status["current_state"] is None
    assert status["state_available"] is False
    assert status["phase"] is None
    assert status["transition"] is None
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


def test_recording_identity_is_strictly_restored_with_the_task_journal(tmp_path) -> None:
    journal = tmp_path / "inspection_tasks.json"
    timeline = InspectionTaskTimeline(journal_path=journal)
    timeline.bind_recording(
        task_id="inspection-task-42",
        session_id="inspection-session-42",
        product_session_id="inspection-product-session-42",
    )

    restored = InspectionTaskTimeline(journal_path=journal)

    recording = restored.query("inspection-task-42")["recording"]
    assert recording["session_id"] == "inspection-session-42"
    assert recording["product_session_id"] == "inspection-product-session-42"
    assert recording["state"] == "recording"
    assert recording["error"] == ""
    assert recording["updated_at"] > 0.0


@pytest.mark.parametrize("native_state", [7, 8, 9])
def test_native_terminal_event_stops_the_bound_recording_once_in_background(native_state) -> None:
    stopped = threading.Event()

    class Recording:
        def __init__(self) -> None:
            self.calls: list[str] = []

        def stop(self, *, expected_session_id: str):
            self.calls.append(expected_session_id)
            stopped.set()

    class Gateway:
        def __init__(self) -> None:
            self._inspection_task_timeline = InspectionTaskTimeline()
            self._inspection_task_timeline.bind_recording(
                task_id="inspection-task-42",
                session_id="inspection-session-42",
                product_session_id="inspection-product-session-42",
            )
            self._recording = Recording()
            self.events: list[dict[str, object]] = []

        def push_event(self, event: dict[str, object]) -> None:
            self.events.append(event)

    gateway = Gateway()
    terminal = _event(event_sequence=1, state=native_state)

    handle_inspection_task_event(gateway, terminal)
    handle_inspection_task_event(gateway, terminal)

    assert stopped.wait(timeout=1.0)
    assert gateway._recording.calls == ["inspection-session-42"]
    deadline = time.time() + 1.0
    recording = gateway._inspection_task_timeline.query("inspection-task-42")["recording"]
    while recording["state"] != "completed" and time.time() < deadline:
        time.sleep(0.01)
        recording = gateway._inspection_task_timeline.query("inspection-task-42")["recording"]
    assert recording["state"] == "completed"


@pytest.mark.parametrize("crashed_after_claim", [False, True])
def test_restored_terminal_fact_finishes_recording_after_gateway_restart(
    tmp_path,
    crashed_after_claim,
) -> None:
    journal = tmp_path / "inspection_tasks.json"
    terminal = _event(event_sequence=1, state=7)
    timeline = InspectionTaskTimeline(journal_path=journal)
    timeline.bind_recording(
        task_id="inspection-task-42",
        session_id="inspection-session-42",
        product_session_id="inspection-product-session-42",
    )
    assert timeline.observe(terminal) is True
    if crashed_after_claim:
        assert timeline.claim_recording_stop("inspection-task-42") is not None

    stopped = threading.Event()

    class Recording:
        calls: list[str] = []

        def stop(self, *, expected_session_id: str):
            self.calls.append(expected_session_id)
            stopped.set()

    class Gateway:
        def __init__(self) -> None:
            self._inspection_task_timeline = InspectionTaskTimeline(journal_path=journal)
            self._recording = Recording()
            self.events: list[dict[str, object]] = []

        def push_event(self, event: dict[str, object]) -> None:
            self.events.append(event)

    gateway = Gateway()

    handle_inspection_task_event(gateway, terminal)
    handle_inspection_task_event(gateway, terminal)

    assert stopped.wait(timeout=1.0)
    assert gateway._recording.calls == ["inspection-session-42"]


def test_transient_recording_stop_failure_is_reclaimable_after_restart(tmp_path) -> None:
    from gateway.services.recording import NativeRecordingError

    journal = tmp_path / "inspection_tasks.json"
    terminal = _event(event_sequence=1, state=7)
    timeline = InspectionTaskTimeline(journal_path=journal)
    timeline.bind_recording(
        task_id="inspection-task-42",
        session_id="inspection-session-42",
        product_session_id="inspection-product-session-42",
    )

    stopped = threading.Event()

    class Recording:
        def stop(self, *, expected_session_id: str):
            stopped.set()
            raise NativeRecordingError(
                "native_recorder_timeout", "temporary timeout", status_code=504
            )

    gateway = type(
        "Gateway",
        (),
        {
            "_inspection_task_timeline": timeline,
            "_recording": Recording(),
            "push_event": lambda self, event: None,
        },
    )()
    handle_inspection_task_event(gateway, terminal)
    assert stopped.wait(timeout=1.0)
    deadline = time.time() + 1.0
    while timeline.query("inspection-task-42")["recording"]["state"] == "stopping" and time.time() < deadline:
        time.sleep(0.01)
    assert timeline.query("inspection-task-42")["recording"]["state"] == "recording"

    restored = InspectionTaskTimeline(journal_path=journal)
    claimed = restored.claim_recording_stop("inspection-task-42", recover_stopping=True)
    assert claimed is not None


def test_recording_session_mismatch_is_a_terminal_cleanup_failure() -> None:
    from gateway.services.recording import NativeRecordingError

    stopped = threading.Event()
    timeline = InspectionTaskTimeline()
    timeline.bind_recording(
        task_id="inspection-task-42",
        session_id="inspection-session-42",
        product_session_id="inspection-product-session-42",
    )

    class Recording:
        def stop(self, *, expected_session_id: str):
            stopped.set()
            raise NativeRecordingError(
                "recording_session_mismatch", "different active session", status_code=409
            )

    gateway = type(
        "Gateway",
        (),
        {
            "_inspection_task_timeline": timeline,
            "_recording": Recording(),
            "push_event": lambda self, event: None,
        },
    )()
    handle_inspection_task_event(gateway, _event(event_sequence=1, state=7))
    assert stopped.wait(timeout=1.0)
    deadline = time.time() + 1.0
    while timeline.query("inspection-task-42")["recording"]["state"] == "stopping" and time.time() < deadline:
        time.sleep(0.01)

    recording = timeline.query("inspection-task-42")["recording"]
    assert recording["state"] == "failed"
    assert recording["error"] == "recording_session_mismatch"


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
