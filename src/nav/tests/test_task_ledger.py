"""Behavior tests for the durable navigation task ledger."""

from __future__ import annotations

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
    assert admission["record"]["state"] == "admitted"
    assert admission["record"]["terminal"] is False
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

    assert task["state"] == "accepted"
    assert task["terminal"] is False
    assert task["endpoint_boot_id"] == "navd-boot-a"
    assert task["attempts"][0]["accepted"] is True
    assert task["attempts"][0]["reason"] == "native_goal_accepted"


def test_unknown_task_cancel_is_audited_without_blocking_cancel():
    ledger = NavigationTaskLedger(":memory:", clock=lambda: 200.0)

    admission = ledger.admit(
        "task-still-running-in-native",
        "cancel-attempt-after-host-restart",
        "cancel",
        {"reason": "operator_stop"},
        source="web",
    )
    task = ledger.record_accepted(
        "task-still-running-in-native",
        "cancel-attempt-after-host-restart",
        reason="cancel_requested",
        endpoint_boot_id="navd-boot-existing",
    )

    assert admission["replay"] is False
    assert admission["record"]["state"] == "unknown"
    assert admission["record"]["observed_only"] is True
    assert task["state"] == "unknown"
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


def test_authoritative_goal_status_reaches_one_immutable_terminal():
    ledger = NavigationTaskLedger(":memory:")
    ledger.admit(
        "task-lifecycle",
        "goal-attempt-lifecycle",
        "goal",
        {"x": 3.0, "y": 4.0},
        target={"x": 3.0, "y": 4.0},
    )
    ledger.record_accepted(
        "task-lifecycle",
        "goal-attempt-lifecycle",
        reason="accepted",
        endpoint_boot_id="navd-boot-a",
    )

    for sequence, state in enumerate(
        ("planning", "executing", "recovering", "executing", "reached"),
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

    assert task["state"] == "reached"
    assert task["terminal"] is True
    assert task["can_resume"] is False
    assert task["terminal_at"] == pytest.approx(105.0)
    terminal_events = [
        event for event in task["events"] if event["state"] in {"reached", "failed", "cancelled", "interrupted"}
    ]
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
    assert duplicate["state"] == "reached"
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

    assert requested["state"] == "accepted"
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
            "sequence": 1,
            "task_id": "task-cancel",
            "request_id": "goal-attempt-cancel",
            "state": int(NavigationGoalState.CANCELLED),
            "reason": "operator_cancelled",
        }
    )
    assert cancelled["state"] == "cancelled"
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

    assert accepted["active_request_id"] == "goal-attempt-clock"
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
    assert executing["state"] == "executing"
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
    ledger.record_accepted(
        task_id,
        request_id,
        reason="accepted",
        endpoint_boot_id=boot_id,
    )
    return task_id, request_id


def test_navigation_state_records_execution_and_map_evidence():
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
            "lifecycle_state": 2,
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
    assert task["state"] == "executing"
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
    assert reconciled[0]["state"] == "unknown"
    assert reconciled[0]["reason"] == "endpoint_status_unavailable"
    assert reconciled[0]["terminal"] is False
    assert reconciled[0]["can_resume"] is False
    assert [task["task_id"] for task in ledger.list_open()] == [task_id]
    first_updated_at = reconciled[0]["updated_at"]
    first_event_count = len(reconciled[0]["events"])
    now[0] = 200.0
    [duplicate] = ledger.reconcile_endpoint(None)
    assert duplicate["updated_at"] == first_updated_at
    assert len(duplicate["events"]) == first_event_count
    assert duplicate["last_navigation_state"] is None

    ledger.close()


def test_reconcile_interrupts_only_with_explicit_restart_or_inactive_evidence():
    restarted = NavigationTaskLedger(":memory:")
    task_id, request_id = _accepted_task(
        restarted,
        "task-restarted",
        boot_id="navd-boot-a",
    )

    [interrupted] = restarted.reconcile_endpoint(
        {
            "timestamp_s": 40.0,
            "boot_id": "navd-boot-b",
            "sequence": 1,
            "active_task_id": task_id,
            "active_request_id": request_id,
            "lifecycle_state": "executing",
        }
    )
    assert interrupted["state"] == "interrupted"
    assert interrupted["reason"] == "endpoint_restarted"
    assert interrupted["terminal"] is True
    assert interrupted["can_resume"] is False
    restarted.close()
    assert interrupted["attempts"][0]["state"] == "interrupted"
    assert interrupted["attempts"][0]["reason"] == "endpoint_restarted"
    assert interrupted["events"][-1]["request_id"] == request_id

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
    assert uncertain["state"] == "unknown"
    assert uncertain["terminal"] is False
    assert uncertain["reason"] == "endpoint_task_not_confirmed"

    [confirmed] = inactive.reconcile_endpoint(endpoint_idle, goal_statuses=[])
    assert confirmed["task_id"] == task_id
    assert confirmed["state"] == "interrupted"
    assert confirmed["reason"] == "endpoint_task_not_active"
    assert confirmed["terminal"] is True
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
    ledger.record_rejected(
        "task-rejected",
        "task-rejected-attempt",
        "map_not_ready",
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
