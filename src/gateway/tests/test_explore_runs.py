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
            {"map_id": "yard-a", "map_version": 7, "artifact_hash": "a" * 64}
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
        "map_version": 7,
        "artifact_hash": "a" * 64,
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
    assert admitted["state"] == "submitted"

    running = runs.observe_event(_event())
    assert running["state"] == "running"
    assert running["state_source"] == "native_exploration_run_event"
    assert running["terminal"] is False

    complete = runs.observe_event(
        _event(state="completed", sequence=2, stop_confirmed=True)
    )
    assert complete["state"] == "completed"
    assert complete["terminal"] is True
    assert complete["motion_stop"] == {
        "confirmed": True,
        "reason": "nav_stop_gate",
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
        map_version=7,
        artifact_hash="a" * 64,
        reason="exploration_running",
    )

    result = runs.observe_event(event)

    assert result["state"] == "running"
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


def test_native_event_must_match_exact_map_artifact() -> None:
    runs = _runs()
    runs.reserve_start("start-request-1", **_context())
    runs.record_admission(RUN_A, accepted=True, reason="exploration_started")
    event = _event()
    event["artifact_hash"] = "b" * 64

    with pytest.raises(ExploreRunConflict, match="artifact_hash mismatch"):
        runs.observe_event(event)

    assert runs.query(RUN_A)["state"] == "submitted"


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
    assert result["state"] == "completed"
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
    assert interrupted["state"] == "interrupted"
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


def test_corrupt_configured_journal_fails_closed(tmp_path: Path) -> None:
    journal = tmp_path / "explore_runs.json"
    journal.write_text(
        json.dumps(
            {
                "body": {
                    "schema_version": "lingtu.explore.run.journal.v1",
                    "runs": [],
                },
                "sha256": "wrong",
            }
        ),
        encoding="utf-8",
    )

    runs = _runs(journal)
    assert runs.health()["journal"]["status"] == "corrupt"
    with pytest.raises(ExploreRunJournalUnavailable, match="integrity"):
        runs.reserve_start("start-request-1", **_context())


def test_query_unknown_run_is_honest() -> None:
    missing = "01K1M9S4FX27T8XMY6QJNBAV3X"
    result = _runs().query(missing)
    assert result == {
        "schema_version": "lingtu.explore.run.v1",
        "found": False,
        "exploration_run_id": missing,
        "state": "unknown",
        "terminal": False,
        "reason": "exploration_run_not_found",
    }


def test_generated_run_id_must_be_canonical_ulid() -> None:
    runs = ExploreRuns(id_factory=lambda: "explore-task-start-request-1")
    with pytest.raises(ValueError, match="canonical 26-character uppercase ULID"):
        runs.reserve_start("start-request-1", **_context())
