"""Behavior tests for the durable navigation task ledger."""

from __future__ import annotations

import sqlite3

import pytest

from nav.services.task_ledger import NavigationTaskLedger, TaskLedgerConflict
from runtime.msgs.nav import (
    NavigationCommandKind,
    NavigationCommandReceipt,
    NavigationGoalState,
)


def test_admitted_task_survives_reopen(tmp_path):
    db_path = tmp_path / "navigation_tasks.sqlite3"
    ledger = NavigationTaskLedger(db_path)

    admission = ledger.admit(
        "task-001",
        "goal-attempt-001",
        "goal",
        {"x": 1.0, "y": 2.0, "yaw": 0.25},
        source="web",
        target={"x": 1.0, "y": 2.0, "yaw": 0.25},
        product_fingerprint="product-sha256",
        map_identity={"map_id": "yard", "version": 7, "hash": "map-sha256"},
    )
    ledger.close()

    reopened = NavigationTaskLedger(db_path)
    task = reopened.get_task("task-001")

    assert admission["replay"] is False
    assert admission["record"]["admission"] == "unconfirmed"
    assert admission["record"]["execution_state"] is None
    assert admission["record"]["evidence_status"] == "unavailable"
    assert task is not None
    assert task["task_id"] == "task-001"
    assert task["source"] == "web"
    assert task["target"] == {"x": 1.0, "y": 2.0, "yaw": 0.25}
    assert task["product_fingerprint"] == "product-sha256"
    assert task["map_identity"] == {
        "map_id": "yard",
        "version": 7,
        "hash": "map-sha256",
    }
    assert task["attempts"][0]["request_id"] == "goal-attempt-001"
    reopened.close()


def test_goal_admission_result_records_native_acceptance():
    ledger = NavigationTaskLedger(":memory:", clock=lambda: 100.0)
    ledger.admit(
        "task-accepted",
        "goal-attempt-accepted",
        "goal",
        {"x": 1.0, "y": 2.0},
        target={"x": 1.0, "y": 2.0},
    )

    task = ledger.record_admission_result(
        "task-accepted",
        "goal-attempt-accepted",
        accepted=True,
        reason="native_goal_accepted",
        endpoint_boot_id="navd-boot-a",
    )

    assert task["admission"] == "accepted"
    assert task["admission_reason"] == "native_goal_accepted"
    assert task["execution_state"] is None
    assert task["execution_reason"] == ""
    assert task["state_source"] == "none"
    assert task["evidence_status"] == "unavailable"
    assert task["endpoint_boot_id"] == "navd-boot-a"
    assert task["attempts"][0]["accepted"] is True
    assert task["attempts"][0]["reason"] == "native_goal_accepted"


def test_rejected_goal_ack_does_not_create_execution_terminal():
    ledger = NavigationTaskLedger(":memory:", clock=lambda: 100.0)
    ledger.admit(
        "task-rejected",
        "goal-attempt-rejected",
        "goal",
        {"x": 1.0, "y": 2.0},
        target={"x": 1.0, "y": 2.0},
    )

    task = ledger.record_admission_result(
        "task-rejected",
        "goal-attempt-rejected",
        accepted=False,
        reason="map_not_ready",
        endpoint_boot_id="navd-boot-a",
    )

    assert task["admission"] == "rejected"
    assert task["admission_reason"] == "map_not_ready"
    assert task["execution_state"] is None
    assert task["state_source"] == "none"
    assert task["terminal"] is False
    assert ledger.list_open() == []


def test_unknown_task_cancel_is_audited_without_blocking_cancel():
    ledger = NavigationTaskLedger(":memory:", clock=lambda: 200.0)

    admission = ledger.admit(
        "task-still-running-in-native",
        "cancel-attempt-after-host-restart",
        "cancel",
        {"reason": "operator_stop"},
        source="web",
    )
    task = ledger.record_admission_result(
        "task-still-running-in-native",
        "cancel-attempt-after-host-restart",
        accepted=True,
        reason="cancel_requested",
        endpoint_boot_id="navd-boot-existing",
    )

    assert admission["replay"] is False
    assert admission["record"]["state"] == "unknown"
    assert admission["record"]["observed_only"] is True
    assert task["execution_state"] is None
    assert task["cancel_requested"] is True
    assert task["terminal"] is False
    assert task["can_resume"] is False
    assert task["attempts"][0]["kind"] == "cancel"
    assert task["attempts"][0]["accepted"] is True
    ledger.close()
    ledger.close()


def test_attempt_identity_is_idempotent_and_task_target_is_immutable():
    ledger = NavigationTaskLedger(":memory:")
    with pytest.raises(ValueError, match="kind must be"):
        ledger.admit("task-invalid-kind", "request-invalid-kind", "pause", {})
    with pytest.raises(ValueError, match="reserved native clock-retry"):
        ledger.admit("task-reserved", "request-clock-retry-1", "goal", {}, target={})

    first = ledger.admit(
        "task-idempotent",
        "goal-attempt-1",
        "goal",
        {"target": {"x": 1.0, "y": 2.0}},
        target={"x": 1.0, "y": 2.0},
    )
    replay = ledger.admit(
        "task-idempotent",
        "goal-attempt-1",
        "goal",
        {"target": {"y": 2.0, "x": 1.0}},
        target={"y": 2.0, "x": 1.0},
    )

    assert first["replay"] is False
    assert replay["replay"] is True
    assert len(replay["record"]["attempts"]) == 1

    with pytest.raises(TaskLedgerConflict, match="different content"):
        ledger.admit(
            "task-idempotent",
            "goal-attempt-1",
            "goal",
            {"target": {"x": 99.0, "y": 2.0}},
            target={"x": 1.0, "y": 2.0},
        )
    with pytest.raises(TaskLedgerConflict, match="different target"):
        ledger.admit(
            "task-idempotent",
            "goal-attempt-1",
            "goal",
            {"target": {"x": 1.0, "y": 2.0}},
            target={"x": 9.0, "y": 2.0},
        )

    with pytest.raises(TaskLedgerConflict, match="different Product"):
        ledger.admit(
            "task-idempotent",
            "goal-attempt-1",
            "goal",
            {"target": {"x": 1.0, "y": 2.0}},
            target={"x": 1.0, "y": 2.0},
            product_fingerprint="different-product",
        )

    with pytest.raises(TaskLedgerConflict, match="different map"):
        ledger.admit(
            "task-idempotent",
            "goal-attempt-1",
            "goal",
            {"target": {"x": 1.0, "y": 2.0}},
            target={"x": 1.0, "y": 2.0},
            map_identity={"map_id": "different-map"},
        )

    retry = ledger.admit(
        "task-idempotent",
        "goal-attempt-2",
        "goal",
        {"target": {"x": 1.0, "y": 2.0}},
        target={"x": 1.0, "y": 2.0},
    )
    assert len(retry["record"]["attempts"]) == 2

    with pytest.raises(TaskLedgerConflict, match="different target"):
        ledger.admit(
            "task-idempotent",
            "goal-attempt-3",
            "goal",
            {"target": {"x": 5.0, "y": 2.0}},
            target={"x": 5.0, "y": 2.0},
        )
    ledger.close()


def test_lookup_admission_is_an_exact_read_only_replay_probe():
    clock_calls: list[float] = []

    def clock() -> float:
        value = 100.0 + len(clock_calls)
        clock_calls.append(value)
        return value

    ledger = NavigationTaskLedger(":memory:", clock=clock)
    ledger.admit(
        "task-lookup",
        "goal-attempt-lookup",
        "goal",
        {"yaw": 0.25, "x": 1.0, "y": 2.0},
        target={"frame_id": "map", "x": 1.0, "y": 2.0, "yaw": 0.25},
        product_fingerprint="product-sha256",
        map_identity={"map_id": "yard", "version": 7},
    )
    before = ledger.get_task("task-lookup")
    clock_call_count = len(clock_calls)

    replay = ledger.lookup_admission(
        "task-lookup",
        "goal-attempt-lookup",
        "goal",
        {"x": 1.0, "y": 2.0, "yaw": 0.25},
        target={"yaw": 0.25, "y": 2.0, "x": 1.0, "frame_id": "map"},
        product_fingerprint="product-sha256",
        map_identity={"version": 7, "map_id": "yard"},
    )
    after = ledger.get_task("task-lookup")

    assert replay == {"record": before, "replay": True}
    assert after == before
    assert len(clock_calls) == clock_call_count
    assert replay["record"]["updated_at"] == before["updated_at"]
    assert replay["record"]["events"] == before["events"]
    ledger.close()


def test_lookup_admission_returns_none_for_unknown_identity_without_writes():
    ledger = NavigationTaskLedger(":memory:", clock=lambda: 100.0)
    ledger.admit(
        "task-known",
        "goal-attempt-known",
        "goal",
        {"x": 1.0, "y": 2.0},
        target={"x": 1.0, "y": 2.0},
    )
    before = ledger.list_recent()

    assert (
        ledger.lookup_admission(
            "task-missing",
            "goal-attempt-missing",
            "goal",
            {"x": 1.0, "y": 2.0},
            target={"x": 1.0, "y": 2.0},
        )
        is None
    )
    assert (
        ledger.lookup_admission(
            "task-known",
            "goal-attempt-missing",
            "goal",
            {"x": 1.0, "y": 2.0},
            target={"x": 1.0, "y": 2.0},
        )
        is None
    )
    assert ledger.list_recent() == before
    ledger.close()


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"kind": "cancel"}, "different content"),
        ({"payload": {"x": 9.0, "y": 2.0}}, "different content"),
        ({"target": {"x": 9.0, "y": 2.0}}, "different target"),
        ({"product_fingerprint": "product-b"}, "different Product"),
        ({"product_fingerprint": ""}, "different Product"),
        ({"map_identity": {"map_id": "map-b"}}, "different map"),
        ({"map_identity": None}, "different map"),
    ],
)
def test_lookup_admission_rejects_conflicting_immutable_content(overrides, message):
    ledger = NavigationTaskLedger(":memory:", clock=lambda: 100.0)
    ledger.admit(
        "task-conflict",
        "goal-attempt-conflict",
        "goal",
        {"x": 1.0, "y": 2.0},
        target={"x": 1.0, "y": 2.0},
        product_fingerprint="product-a",
        map_identity={"map_id": "map-a"},
    )
    before = ledger.get_task("task-conflict")
    query = {
        "kind": "goal",
        "payload": {"x": 1.0, "y": 2.0},
        "target": {"x": 1.0, "y": 2.0},
        "product_fingerprint": "product-a",
        "map_identity": {"map_id": "map-a"},
    }
    query.update(overrides)

    with pytest.raises(TaskLedgerConflict, match=message):
        ledger.lookup_admission(
            "task-conflict",
            "goal-attempt-conflict",
            query.pop("kind"),
            query.pop("payload"),
            **query,
        )

    assert ledger.get_task("task-conflict") == before
    ledger.close()


def test_authoritative_goal_status_reaches_one_immutable_terminal():
    ledger = NavigationTaskLedger(":memory:")
    ledger.admit(
        "task-lifecycle",
        "goal-attempt-lifecycle",
        "goal",
        {"x": 3.0, "y": 4.0},
        target={"x": 3.0, "y": 4.0},
    )
    ledger.record_admission_result(
        "task-lifecycle",
        "goal-attempt-lifecycle",
        accepted=True,
        reason="accepted",
        endpoint_boot_id="navd-boot-a",
    )

    for sequence, state in enumerate(
        ("planning", "executing", "paused", "executing", "reached"),
        start=1,
    ):
        task = ledger.record_goal_status(
            {
                "timestamp_s": 100.0 + sequence,
                "boot_id": "navd-boot-a",
                "sequence": sequence,
                "task_id": "task-lifecycle",
                "request_id": "goal-attempt-lifecycle",
                "state": state,
                "reason": f"{state}-{sequence}",
            }
        )

    assert task["admission"] == "accepted"
    assert task["execution_state"] == "reached"
    assert task["execution_reason"] == "reached-5"
    assert task["state_source"] == "native_goal_status"
    assert task["evidence_status"] == "fresh"
    assert task["terminal"] is True
    assert task["terminal_at"] == pytest.approx(105.0)
    assert task["attempts"][0]["state"] == "accepted"
    terminal_events = [event for event in task["events"] if event["state"] in {"reached", "failed", "cancelled"}]
    assert [event["state"] for event in terminal_events] == ["reached"]

    duplicate = ledger.record_goal_status(
        {
            "timestamp_s": 105.0,
            "boot_id": "navd-boot-a",
            "sequence": 5,
            "task_id": "task-lifecycle",
            "request_id": "goal-attempt-lifecycle",
            "state": "reached",
            "reason": "reached-5",
        }
    )
    assert duplicate["execution_state"] == "reached"
    assert len(duplicate["events"]) == len(task["events"])

    with pytest.raises(TaskLedgerConflict, match="terminal"):
        ledger.record_goal_status(
            {
                "timestamp_s": 106.0,
                "boot_id": "navd-boot-a",
                "sequence": 6,
                "task_id": "task-lifecycle",
                "request_id": "goal-attempt-lifecycle",
                "state": "failed",
                "reason": "late_failure",
            }
        )
    ledger.close()


def test_authoritative_paused_goal_status_remains_nonterminal():
    ledger = NavigationTaskLedger(":memory:")
    ledger.admit(
        "task-paused",
        "goal-attempt-paused",
        "goal",
        {"x": 3.0, "y": 4.0},
        target={"x": 3.0, "y": 4.0},
    )
    ledger.record_admission_result(
        "task-paused",
        "goal-attempt-paused",
        accepted=True,
        reason="accepted",
        endpoint_boot_id="navd-boot-a",
    )

    paused = ledger.record_goal_status(
        {
            "timestamp_s": 106.0,
            "boot_id": "navd-boot-a",
            "sequence": 6,
            "task_id": "task-paused",
            "request_id": "goal-attempt-paused",
            "state": 6,
            "reason": "operator_pause",
        }
    )

    assert paused["execution_state"] == "paused"
    assert paused["execution_reason"] == "operator_pause"
    assert paused["terminal"] is False
    assert paused["state_source"] == "native_goal_status"
    assert [item["task_id"] for item in ledger.list_open()] == ["task-paused"]
    ledger.close()


def test_authoritative_goal_status_can_continue_under_a_new_endpoint_boot():
    ledger = NavigationTaskLedger(":memory:")
    ledger.admit(
        "task-new-boot",
        "goal-attempt-new-boot",
        "goal",
        {"x": 1.0, "y": 1.0},
        target={"x": 1.0, "y": 1.0},
    )
    ledger.record_admission_result(
        "task-new-boot",
        "goal-attempt-new-boot",
        accepted=True,
        reason="accepted",
        endpoint_boot_id="navd-boot-a",
    )
    ledger.record_goal_status(
        {
            "timestamp_s": 10.0,
            "boot_id": "navd-boot-a",
            "sequence": 5,
            "task_id": "task-new-boot",
            "request_id": "goal-attempt-new-boot",
            "state": "planning",
            "reason": "planning",
        }
    )

    task = ledger.record_goal_status(
        {
            "timestamp_s": 11.0,
            "boot_id": "navd-boot-b",
            "sequence": 1,
            "task_id": "task-new-boot",
            "request_id": "goal-attempt-new-boot",
            "state": "executing",
            "reason": "path_active_after_restart",
        }
    )

    assert task["admission"] == "accepted"
    assert task["execution_state"] == "executing"
    assert task["execution_reason"] == "path_active_after_restart"
    assert task["endpoint_boot_id"] == "navd-boot-b"
    assert task["state_source"] == "native_goal_status"
    assert task["evidence_status"] == "fresh"
    assert task["terminal"] is False
    ledger.close()


def test_cancel_ack_is_only_a_request_until_goal_status_confirms_cancelled():
    ledger = NavigationTaskLedger(":memory:")
    ledger.admit(
        "task-cancel",
        "goal-attempt-cancel",
        "goal",
        {"x": 1.0, "y": 1.0},
        target={"x": 1.0, "y": 1.0},
    )
    ledger.record_native_ack(
        NavigationCommandReceipt(
            accepted=True,
            kind=int(NavigationCommandKind.GOAL),
            task_id="task-cancel",
            request_id="goal-attempt-cancel",
            endpoint_timestamp_s=10.0,
            reason="accepted",
        ),
        endpoint_boot_id="navd-boot-cancel",
    )
    ledger.record_goal_status(
        {
            "timestamp_s": 10.5,
            "boot_id": "navd-boot-cancel",
            "sequence": 1,
            "task_id": "task-cancel",
            "request_id": "goal-attempt-cancel",
            "state": int(NavigationGoalState.PATH_ACTIVE),
            "reason": "path_active",
        }
    )
    ledger.admit(
        "task-cancel",
        "cancel-attempt-cancel",
        "cancel",
        {"reason": "operator_cancel"},
    )

    requested = ledger.record_native_ack(
        NavigationCommandReceipt(
            accepted=True,
            kind=int(NavigationCommandKind.CANCEL),
            task_id="task-cancel",
            request_id="cancel-attempt-cancel",
            endpoint_timestamp_s=11.0,
            reason="cancel_requested",
        ),
        endpoint_boot_id="navd-boot-cancel",
    )

    assert requested["admission"] == "accepted"
    assert requested["execution_state"] == "executing"
    assert requested["state_source"] == "native_goal_status"
    assert requested["cancel_requested"] is True
    assert requested["cancel_request_id"] == "cancel-attempt-cancel"
    assert requested["cancel_reason"] == "operator_cancel"
    assert requested["terminal"] is False
    assert requested["attempts"][1]["native_ack"]["accepted"] is True
    assert requested["events"][-1]["state"] == "cancel_requested"

    cancelled = ledger.record_goal_status(
        {
            "timestamp_s": 12.0,
            "boot_id": "navd-boot-cancel",
            "sequence": 2,
            "task_id": "task-cancel",
            "request_id": "goal-attempt-cancel",
            "state": int(NavigationGoalState.CANCELLED),
            "reason": "operator_cancelled",
        }
    )
    assert cancelled["execution_state"] == "cancelled"
    assert cancelled["execution_reason"] == "operator_cancelled"
    assert cancelled["evidence_status"] == "fresh"
    assert cancelled["terminal"] is True
    ledger.close()


def test_native_clock_retry_is_correlated_to_the_original_request_attempt():
    ledger = NavigationTaskLedger(":memory:")
    ledger.admit(
        "task-clock-retry",
        "goal-attempt-clock",
        "goal",
        {"x": 2.0, "y": 3.0},
        target={"x": 2.0, "y": 3.0},
    )

    accepted = ledger.record_native_ack(
        NavigationCommandReceipt(
            accepted=True,
            kind=int(NavigationCommandKind.GOAL),
            task_id="task-clock-retry",
            request_id="goal-attempt-clock-clock-retry-1",
            endpoint_timestamp_s=20.0,
            reason="accepted_after_clock_retry",
        ),
        endpoint_boot_id="navd-boot-clock",
    )

    assert accepted["active_request_id"] == ""
    assert accepted["attempts"][0]["request_id"] == "goal-attempt-clock"
    assert accepted["attempts"][0]["native_request_id"] == "goal-attempt-clock-clock-retry-1"

    executing = ledger.record_goal_status(
        {
            "timestamp_s": 21.0,
            "boot_id": "navd-boot-clock",
            "sequence": 1,
            "task_id": "task-clock-retry",
            "request_id": "goal-attempt-clock-clock-retry-1",
            "state": "executing",
            "reason": "path_active",
        }
    )
    assert executing["execution_state"] == "executing"
    assert executing["active_request_id"] == "goal-attempt-clock"
    assert executing["last_goal_status"]["request_id"].endswith("-clock-retry-1")
    ledger.close()


def _accepted_task(
    ledger: NavigationTaskLedger,
    task_id: str,
    *,
    boot_id: str,
) -> tuple[str, str]:
    request_id = f"{task_id}-goal-attempt"
    ledger.admit(
        task_id,
        request_id,
        "goal",
        {"x": 1.0, "y": 2.0},
        target={"x": 1.0, "y": 2.0},
    )
    ledger.record_admission_result(
        task_id,
        request_id,
        accepted=True,
        reason="accepted",
        endpoint_boot_id=boot_id,
    )
    return task_id, request_id


def test_navigation_state_records_evidence_without_changing_execution():
    ledger = NavigationTaskLedger(":memory:")
    task_id, request_id = _accepted_task(
        ledger,
        "task-state",
        boot_id="navd-boot-state",
    )

    task = ledger.record_navigation_state(
        {
            "timestamp_s": 30.0,
            "boot_id": "navd-boot-state",
            "sequence": 1,
            "lifecycle_state": 6,
            "active_task_id": task_id,
            "active_request_id": f"{request_id}-clock-retry-1",
            "map_id": "yard",
            "map_version": 8,
            "map_hash": "map-sha256-v8",
            "hold_reason": "",
            "failure_code": "",
        }
    )

    assert task is not None
    assert task["admission"] == "accepted"
    assert task["execution_state"] is None
    assert task["execution_reason"] == ""
    assert task["state_source"] == "none"
    assert task["evidence_status"] == "fresh"
    assert task["terminal"] is False
    assert task["active_request_id"] == request_id
    assert task["last_navigation_state"]["active_request_id"].endswith("-clock-retry-1")
    assert task["last_navigation_state"]["map_id"] == "yard"
    ledger.close()


def test_unavailable_endpoint_evidence_never_falsely_interrupts_task():
    now = [100.0]
    ledger = NavigationTaskLedger(":memory:", clock=lambda: now[0])
    task_id, _ = _accepted_task(
        ledger,
        "task-unavailable",
        boot_id="navd-boot-a",
    )

    reconciled = ledger.reconcile_endpoint(None)

    assert [task["task_id"] for task in reconciled] == [task_id]
    assert reconciled[0]["admission"] == "accepted"
    assert reconciled[0]["execution_state"] is None
    assert reconciled[0]["execution_reason"] == ""
    assert reconciled[0]["state_source"] == "none"
    assert reconciled[0]["evidence_status"] == "unavailable"
    assert reconciled[0]["terminal"] is False
    assert [task["task_id"] for task in ledger.list_open()] == [task_id]
    first_updated_at = reconciled[0]["updated_at"]
    first_event_count = len(reconciled[0]["events"])
    assert reconciled[0]["events"][-1]["reason"] == "endpoint_status_unavailable"
    now[0] = 200.0
    [duplicate] = ledger.reconcile_endpoint(None)
    assert duplicate["updated_at"] == first_updated_at
    assert len(duplicate["events"]) == first_event_count
    assert duplicate["last_navigation_state"] is None

    ledger.close()


def test_reconcile_marks_evidence_health_without_inventing_terminal():
    restarted = NavigationTaskLedger(":memory:")
    task_id, request_id = _accepted_task(
        restarted,
        "task-restarted",
        boot_id="navd-boot-a",
    )

    [boot_changed] = restarted.reconcile_endpoint(
        {
            "timestamp_s": 40.0,
            "boot_id": "navd-boot-b",
            "sequence": 1,
            "active_task_id": task_id,
            "active_request_id": request_id,
            "lifecycle_state": "executing",
        }
    )
    assert boot_changed["admission"] == "accepted"
    assert boot_changed["execution_state"] is None
    assert boot_changed["execution_reason"] == ""
    assert boot_changed["evidence_status"] == "boot_changed"
    assert boot_changed["terminal"] is False
    assert boot_changed["attempts"][0]["state"] == "accepted"
    assert boot_changed["attempts"][0]["reason"] == "accepted"
    restarted.close()

    inactive = NavigationTaskLedger(":memory:")
    task_id, _ = _accepted_task(
        inactive,
        "task-inactive",
        boot_id="navd-boot-same",
    )
    endpoint_idle = {
        "timestamp_s": 50.0,
        "boot_id": "navd-boot-same",
        "sequence": 2,
        "active_task_id": "",
        "active_request_id": "",
        "lifecycle_state": "idle",
    }

    [uncertain] = inactive.reconcile_endpoint(endpoint_idle)
    assert uncertain["admission"] == "accepted"
    assert uncertain["execution_state"] is None
    assert uncertain["evidence_status"] == "stale"
    assert uncertain["terminal"] is False
    assert uncertain["events"][-1]["reason"] == "endpoint_task_not_confirmed"

    [confirmed] = inactive.reconcile_endpoint(endpoint_idle, goal_statuses=[])
    assert confirmed["task_id"] == task_id
    assert confirmed["execution_state"] is None
    assert confirmed["evidence_status"] == "stale"
    assert confirmed["terminal"] is False
    assert confirmed["events"][-1]["reason"] == "endpoint_task_not_active"
    inactive.close()


def test_recent_and_open_queries_return_json_safe_task_records():
    ledger = NavigationTaskLedger(":memory:")
    _accepted_task(ledger, "task-open", boot_id="navd-boot-list")
    ledger.admit(
        "task-rejected",
        "task-rejected-attempt",
        "goal",
        {"x": 9.0, "y": 9.0},
        target={"x": 9.0, "y": 9.0},
    )
    ledger.record_admission_result(
        "task-rejected",
        "task-rejected-attempt",
        accepted=False,
        reason="map_not_ready",
    )

    recent = ledger.list_recent(limit=10)
    open_tasks = ledger.list_open()

    assert {task["task_id"] for task in recent} == {
        "task-open",
        "task-rejected",
    }
    assert [task["task_id"] for task in open_tasks] == ["task-open"]
    assert all(isinstance(task["attempts"], list) for task in recent)
    assert all(isinstance(task["events"], list) for task in recent)
    ledger.close()


def test_legacy_synthetic_terminal_is_not_migrated_as_native_execution(tmp_path):
    db_path = tmp_path / "legacy-navigation-tasks.sqlite3"
    connection = sqlite3.connect(db_path)
    connection.executescript(
        """
        CREATE TABLE navigation_tasks (
            task_id TEXT PRIMARY KEY,
            state TEXT NOT NULL,
            terminal INTEGER NOT NULL DEFAULT 0,
            reason TEXT NOT NULL DEFAULT '',
            source TEXT NOT NULL DEFAULT '',
            observed_only INTEGER NOT NULL DEFAULT 0,
            target_json TEXT NOT NULL,
            product_fingerprint TEXT NOT NULL DEFAULT '',
            map_identity_json TEXT NOT NULL,
            created_at REAL NOT NULL,
            updated_at REAL NOT NULL,
            terminal_at REAL,
            endpoint_boot_id TEXT NOT NULL DEFAULT '',
            active_request_id TEXT NOT NULL DEFAULT '',
            cancel_requested_at REAL,
            cancel_request_id TEXT NOT NULL DEFAULT '',
            cancel_reason TEXT NOT NULL DEFAULT '',
            can_resume INTEGER NOT NULL DEFAULT 0,
            last_goal_status_json TEXT,
            last_navigation_state_json TEXT
        );
        CREATE TABLE navigation_task_attempts (
            task_id TEXT NOT NULL,
            request_id TEXT NOT NULL,
            native_request_id TEXT NOT NULL DEFAULT '',
            kind TEXT NOT NULL,
            payload_json TEXT NOT NULL,
            state TEXT NOT NULL DEFAULT 'admitted',
            accepted INTEGER,
            reason TEXT NOT NULL DEFAULT '',
            endpoint_boot_id TEXT NOT NULL DEFAULT '',
            native_ack_json TEXT,
            created_at REAL NOT NULL,
            updated_at REAL NOT NULL,
            PRIMARY KEY (task_id, request_id)
        );
        INSERT INTO navigation_tasks (
            task_id, state, terminal, reason, target_json, map_identity_json,
            created_at, updated_at, terminal_at, endpoint_boot_id,
            active_request_id
        ) VALUES (
            'task-legacy', 'interrupted', 1, 'endpoint_restarted', '{}', '{}',
            10.0, 20.0, 20.0, 'navd-boot-a', 'goal-attempt-legacy'
        );
        INSERT INTO navigation_task_attempts (
            task_id, request_id, native_request_id, kind, payload_json,
            state, accepted, reason, endpoint_boot_id, native_ack_json,
            created_at, updated_at
        ) VALUES (
            'task-legacy', 'goal-attempt-legacy', 'goal-attempt-legacy',
            'goal', '{}', 'interrupted', 1, 'endpoint_restarted',
            'navd-boot-a',
            '{"accepted":true,"kind":"goal","reason":"accepted","request_id":"goal-attempt-legacy","task_id":"task-legacy","endpoint_timestamp_s":11.0}',
            10.0, 20.0
        );
        """
    )
    connection.commit()
    connection.close()

    ledger = NavigationTaskLedger(db_path)
    task = ledger.get_task("task-legacy")

    assert task is not None
    assert task["admission"] == "accepted"
    assert task["admission_reason"] == "accepted"
    assert task["execution_state"] is None
    assert task["execution_reason"] == ""
    assert task["terminal"] is False
    assert task["terminal_at"] is None
    assert task["active_request_id"] == ""
    assert task["state_source"] == "none"
    assert task["evidence_status"] == "boot_changed"
    assert task["attempts"][0]["state"] == "accepted"
    assert task["attempts"][0]["reason"] == "accepted"
    ledger.close()
