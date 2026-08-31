from __future__ import annotations

import json
from pathlib import Path

import pytest

from gateway.services.explore_runs import (
    ExploreRunConflict,
    ExploreRunJournalUnavailable,
    ExploreRuns,
)

RUN_A = "01K1M9S4FX27T8XMY6QJNBAV3W"


def _context(*, session: str = "product-session-0001", route: str = "map") -> dict:
    return {
        "product_session_id": session,
        "route": route,
        "map": (
            {"map_id": "yard-a", "map_content_epoch": 7}
            if route == "map"
            else None
        ),
    }


def _event(
    *,
    state: str = "running",
    sequence: int = 1,
    boot_id: str = "explore-boot-a",
    stop_confirmed: bool = False,
) -> dict:
    return {
        "timestamp_s": 100.0 + sequence,
        "frame_id": "map",
        "boot_id": boot_id,
        "event_sequence": sequence,
        "kind": 2,
        "exploration_run_id": RUN_A,
        "start_request_id": "start-request-1",
        "command_request_id": "start-request-1",
        "product_session_id": "product-session-0001",
        "state": state,
        "route": "map",
        "map_id": "yard-a",
        "map_content_epoch": 7,
        "reason": f"exploration_{state}",
        "motion_stop_confirmed": stop_confirmed,
        "motion_stop_reason": "nav_stop_gate" if stop_confirmed else "",
    }


def _runs(path: Path | None = None) -> ExploreRuns:
    return ExploreRuns(journal_path=path, id_factory=lambda: RUN_A)


def test_start_request_is_durable_and_exact_retry_replays_same_run(tmp_path: Path) -> None:
    journal = tmp_path / "explore_runs.json"
    runs = _runs(journal)

    first = runs.reserve_start("start-request-1", **_context())
    replay = runs.reserve_start("start-request-1", **_context())

    assert first["replay"] is False
    assert replay["replay"] is True
    assert first["run"]["exploration_run_id"] == RUN_A
    assert first["run"]["exploration_run_id"] != "start-request-1"

    restored = _runs(journal)
    assert restored.query(RUN_A)["found"] is True
    assert restored.reserve_start("start-request-1", **_context())["replay"] is True


def test_request_id_cannot_be_reused_for_another_product_context() -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())

    with pytest.raises(ExploreRunConflict, match="different exploration context"):
        runs.reserve_start(
            "start-request-1", **_context(session="product-session-0002")
        )


def test_only_ordered_native_event_drives_execution_and_terminal_state() -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    admitted = runs.record_admission(RUN_A, accepted=True, reason="exploration_started")
    assert admitted["admission"] == "accepted"
    assert admitted["state"] is None
    assert admitted["state_available"] is False
    assert admitted["phase"] is None

    running = runs.observe_event(_event())
    assert running["state"] == "EXECUTING"
    assert running["state_available"] is True
    assert running["phase"] == "RUNNING"
    assert running["state_source"] == "native_exploration_run_event"
    assert running["terminal"] is False

    complete = runs.observe_event(
        _event(state="completed", sequence=2, stop_confirmed=True)
    )
    assert complete["state"] == "SUCCESS"
    assert complete["state_available"] is True
    assert complete["phase"] == "COMPLETED"
    assert complete["terminal"] is True
    assert complete["motion_stop"] == {
        "confirmed": True,
        "reason": "nav_stop_gate",
    }


@pytest.mark.parametrize(
    ("native_state", "kind", "stop_confirmed", "public_state", "terminal"),
    [
        ("admitted", 1, False, "PLANNING", False),
        ("paused", 2, True, "PAUSED", False),
        ("cancelled", 2, True, "CANCELLED", True),
        ("failed", 2, True, "FAILED", True),
    ],
)
def test_confirmed_native_phase_maps_to_canonical_public_state(
    native_state: str,
    kind: int,
    stop_confirmed: bool,
    public_state: str,
    terminal: bool,
) -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    runs.record_admission(RUN_A, accepted=True, reason="exploration_started")
    event = _event(state=native_state, stop_confirmed=stop_confirmed)
    event["kind"] = kind

    result = runs.observe_event(event)

    assert result["state"] == public_state
    assert result["state_available"] is True
    assert result["phase"] == native_state.upper()
    assert result["transition"] is None
    assert result["terminal"] is terminal


@pytest.mark.parametrize(
    ("transition_phase", "target_state"),
    [("pausing", "PAUSED"), ("cancelling", "CANCELLED")],
)
def test_transition_phase_keeps_last_confirmed_public_state(
    transition_phase: str,
    target_state: str,
) -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    runs.record_admission(RUN_A, accepted=True, reason="exploration_started")
    runs.observe_event(_event(state="running", sequence=1))
    event = _event(state=transition_phase, sequence=2)
    event["kind"] = 3
    event["motion_stop_reason"] = "driver_ack_pending"

    result = runs.observe_event(event)

    assert result["state"] == "EXECUTING"
    assert result["state_available"] is True
    assert result["phase"] == transition_phase.upper()
    assert result["transition"] == {
        "confirmed": False,
        "from_state": "EXECUTING",
        "phase": transition_phase.upper(),
        "reason": f"exploration_{transition_phase}",
        "target_state": target_state,
    }


def test_runtime_message_timestamp_projects_to_canonical_native_field() -> None:
    from runtime.msgs import ExplorationRunEvent, ExplorationRunEventKind
    from runtime.msgs.nav import ExplorationRunState

    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    event = ExplorationRunEvent(
        ts=101.0,
        frame_id="map",
        boot_id="explore-boot-a",
        event_sequence=1,
        kind=int(ExplorationRunEventKind.STATE_CHANGED),
        exploration_run_id=RUN_A,
        start_request_id="start-request-1",
        command_request_id="start-request-1",
        product_session_id="product-session-0001",
        state=int(ExplorationRunState.RUNNING),
        route="map",
        map_id="yard-a",
        map_content_epoch=7,
        reason="exploration_running",
    )

    result = runs.observe_event(event)

    assert result["state"] == "EXECUTING"
    assert result["phase"] == "RUNNING"
    assert result["last_native_event"]["timestamp_s"] == 101.0
    assert "ts" not in result["last_native_event"]


def test_mapping_timestamp_alias_is_canonicalized_and_conflicts_are_rejected() -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    event = _event()
    event["ts"] = event.pop("timestamp_s")

    result = runs.observe_event(event)

    assert result["last_native_event"]["timestamp_s"] == 101.0
    assert "ts" not in result["last_native_event"]

    second = _runs()
    second.reserve_start("start-request-1", **_context())
    conflicting = _event()
    conflicting["ts"] = 999.0
    with pytest.raises(ValueError, match="conflicting timestamp"):
        second.observe_event(conflicting)


def test_native_terminal_without_parking_evidence_is_rejected() -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    runs.record_admission(RUN_A, accepted=True, reason="exploration_started")

    with pytest.raises(ValueError, match="lacks confirmed motion-stop evidence"):
        runs.observe_event(_event(state="cancelled", stop_confirmed=False))

    assert runs.query(RUN_A)["terminal"] is False


def test_native_event_must_match_exact_map_content_epoch() -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    runs.record_admission(RUN_A, accepted=True, reason="exploration_started")
    event = _event()
    event["map_content_epoch"] = 8

    with pytest.raises(ExploreRunConflict, match="map_content_epoch mismatch"):
        runs.observe_event(event)

    unresolved = runs.query(RUN_A)
    assert unresolved["state"] is None
    assert unresolved["state_available"] is False


@pytest.mark.parametrize(
    ("kind", "state"),
    [
        (99, "running"),
        (1, "running"),
        (2, "admitted"),
        (3, "running"),
    ],
)
def test_invalid_native_event_kind_state_pair_is_rejected(
    kind: int, state: str
) -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    event = _event(state=state)
    event["kind"] = kind

    with pytest.raises(ValueError, match="event kind"):
        runs.observe_event(event)


def test_terminal_run_cannot_be_reopened_by_a_later_event() -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    runs.record_admission(RUN_A, accepted=True, reason="exploration_started")
    runs.observe_event(_event(state="completed", sequence=1, stop_confirmed=True))

    with pytest.raises(ExploreRunConflict, match="already terminal"):
        runs.observe_event(_event(state="running", sequence=2))

    result = runs.query(RUN_A)
    assert result["state"] == "SUCCESS"
    assert result["phase"] == "COMPLETED"
    assert result["terminal"] is True


def test_endpoint_restart_is_visible_but_does_not_invent_terminal_truth() -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    runs.record_admission(RUN_A, accepted=True, reason="exploration_started")
    runs.observe_event(_event())

    interrupted = runs.reconcile_runtime(
        product_session_id="product-session-0001",
        observed_boot_id="explore-boot-b",
    )

    assert interrupted is not None
    assert interrupted["exploration_run_id"] == RUN_A
    assert interrupted["state"] == "EXECUTING"
    assert interrupted["state_available"] is False
    assert interrupted["phase"] == "RUNNING"
    assert interrupted["continuity"] == {
        "reason": "endpoint_restarted_stop_pending",
        "replacement_boot_id": "explore-boot-b",
        "status": "interrupted",
    }
    assert interrupted["terminal"] is False
    assert interrupted["reason"] == "endpoint_restarted_stop_pending"
    assert interrupted["can_resume"] is False
    assert interrupted["motion_stop"]["confirmed"] is False


def test_endpoint_restart_reconciliation_is_idempotent(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    runs.record_admission(RUN_A, accepted=True, reason="exploration_started")
    runs.observe_event(_event())
    writes = 0
    original_persist = runs._persist

    def count_persist() -> None:
        nonlocal writes
        writes += 1
        original_persist()

    monkeypatch.setattr(runs, "_persist", count_persist)

    assert runs.reconcile_runtime(
        product_session_id="product-session-0001",
        observed_boot_id="explore-boot-b",
    ) is not None
    assert runs.reconcile_runtime(
        product_session_id="product-session-0001",
        observed_boot_id="explore-boot-b",
    ) is None
    assert writes == 1


def test_endpoint_restart_during_transition_marks_confirmed_state_unavailable() -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    runs.record_admission(RUN_A, accepted=True, reason="exploration_started")
    runs.observe_event(_event(state="running", sequence=1))
    pausing = _event(state="pausing", sequence=2)
    pausing["kind"] = 3
    pausing["motion_stop_reason"] = "driver_ack_pending"
    runs.observe_event(pausing)

    interrupted = runs.reconcile_runtime(
        product_session_id="product-session-0001",
        observed_boot_id="explore-boot-b",
    )

    assert interrupted is not None
    assert interrupted["state"] == "EXECUTING"
    assert interrupted["state_available"] is False
    assert interrupted["phase"] == "PAUSING"
    assert interrupted["continuity"]["status"] == "interrupted"


def test_interrupted_journal_restores_as_unavailable_continuity(
    tmp_path: Path,
) -> None:
    journal = tmp_path / "explore_runs.json"
    runs = _runs(journal)
    runs.reserve_start("start-request-1", **_context())
    runs.record_admission(RUN_A, accepted=True, reason="exploration_started")
    runs.observe_event(_event())

    body = json.loads(journal.read_text(encoding="utf-8"))
    record = body["runs"][0]
    record["state"] = "interrupted"
    record["state_source"] = "runtime_reconciliation"
    record["reason"] = "endpoint_restarted_stop_pending"
    record["replacement_boot_id"] = "explore-boot-b"
    journal.write_text(
        json.dumps(body, sort_keys=True, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )

    restored = _runs(journal).query(RUN_A)

    assert restored["state"] == "EXECUTING"
    assert restored["state_available"] is False
    assert restored["phase"] == "RUNNING"
    assert restored["continuity"] == {
        "reason": "endpoint_restarted_stop_pending",
        "replacement_boot_id": "explore-boot-b",
        "status": "interrupted",
    }


def test_rejected_admission_has_no_public_task_state() -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())

    rejected = runs.record_admission(
        RUN_A,
        accepted=False,
        reason="native_rejected",
    )

    assert rejected["admission"] == "rejected"
    assert rejected["state"] is None
    assert rejected["state_available"] is False
    assert rejected["phase"] is None
    assert rejected["transition"] is None


def test_corrupt_configured_journal_fails_closed(tmp_path: Path) -> None:
    journal = tmp_path / "explore_runs.json"
    journal.write_text(
        json.dumps(
            {
                "schema_version": "wrong",
                "runs": [],
            }
        ),
        encoding="utf-8",
    )

    runs = _runs(journal)
    assert runs.health()["journal"]["status"] == "corrupt"
    with pytest.raises(ExploreRunJournalUnavailable, match="schema"):
        runs.reserve_start("start-request-1", **_context())


def test_query_unknown_run_is_honest() -> None:
    missing = "01K1M9S4FX27T8XMY6QJNBAV3X"
    result = _runs().query(missing)
    assert result == {
        "schema_version": "lingtu.explore.run.v1",
        "found": False,
        "exploration_run_id": missing,
        "state": None,
        "state_available": False,
        "phase": None,
        "transition": None,
        "terminal": False,
        "reason": "exploration_run_not_found",
    }


def test_generated_run_id_must_be_canonical_ulid() -> None:
    runs = ExploreRuns(id_factory=lambda: "explore-task-start-request-1")
    with pytest.raises(ValueError, match="canonical 26-character uppercase ULID"):
        runs.reserve_start("start-request-1", **_context())
