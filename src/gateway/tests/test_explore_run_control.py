from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

import gateway.services.exploration as exploration
from gateway.services.explore_runs import ExploreRunJournalUnavailable, ExploreRuns

RUN_ID = "01K1M9S4FX27T8XMY6QJNBAV3W"


class _Lock:
    def __init__(self, **_kwargs: Any) -> None:
        pass

    def __enter__(self) -> _Lock:
        return self

    def __exit__(self, *_args: Any) -> None:
        return None


class _Commands:
    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, Any]]] = []

    def _receipt(self, name: str, kwargs: dict[str, Any]) -> dict[str, Any]:
        self.calls.append((name, dict(kwargs)))
        return {
            "accepted": True,
            "exploration_run_id": kwargs["exploration_run_id"],
            "request_id": kwargs["request_id"],
            "reason": f"exploration_{name}",
            "duplicate": False,
        }

    def start_exploration(self, **kwargs: Any) -> dict[str, Any]:
        return self._receipt("start", kwargs)

    def pause_exploration(self, **kwargs: Any) -> dict[str, Any]:
        return self._receipt("pause", kwargs)

    def resume_exploration(self, **kwargs: Any) -> dict[str, Any]:
        return self._receipt("resume", kwargs)

    def stop_exploration(self, **kwargs: Any) -> dict[str, Any]:
        return self._receipt("finish", kwargs)


class _Gateway:
    def __init__(self, journal_path: Path | None = None) -> None:
        self._explore_runs = ExploreRuns(
            journal_path=journal_path,
            id_factory=lambda: RUN_ID,
        )
        self.events: list[dict[str, Any]] = []
        self._exploring = False

    def push_event(self, event: dict[str, Any]) -> None:
        self.events.append(event)

    @staticmethod
    def _exploration_start_readiness() -> dict[str, Any]:
        return {"can_start": True, "blockers": []}


def _binding() -> dict[str, Any]:
    return {
        "valid": True,
        "blockers": [],
        "session_id": "product-session-a",
        "variant": "map",
        "native_status": {
            "boot_id": "explore-boot-a",
            "active": False,
            "state": "idle",
            "paused": False,
            "pending_goal": None,
            "pending_segment": None,
            "map": {
                "map_id": "yard-a",
                "map_version": 7,
                "artifact_hash": "a" * 64,
            },
        },
    }


def _run_event(*, state: str = "running", sequence: int = 1) -> dict[str, Any]:
    stopped = state in {"paused", "completed", "cancelled", "failed"}
    return {
        "timestamp_s": 100.0 + sequence,
        "frame_id": "map",
        "boot_id": "explore-boot-a",
        "event_sequence": sequence,
        "kind": 2,
        "exploration_run_id": RUN_ID,
        "start_request_id": "request-a",
        "command_request_id": "request-a",
        "product_session_id": "product-session-a",
        "state": state,
        "route": "map",
        "map_id": "yard-a",
        "map_version": 7,
        "artifact_hash": "a" * 64,
        "reason": f"exploration_{state}",
        "motion_stop_confirmed": stopped,
        "motion_stop_reason": "nav_stop_gate" if stopped else "",
    }


@pytest.fixture
def configured(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> tuple[_Gateway, _Commands]:
    gw = _Gateway(tmp_path / "explore-runs.json")
    commands = _Commands()
    monkeypatch.setattr("lingtu.product_lock.ProductControlLock", _Lock)
    monkeypatch.setattr(exploration, "external_explore_binding", lambda _gw: _binding())
    monkeypatch.setattr(exploration, "_native_commands", lambda _gw: commands)
    monkeypatch.setattr(exploration, "_native_status", lambda: _binding()["native_status"])
    return gw, commands


def test_start_reserves_before_dispatch_and_exact_retry_does_not_restart(
    configured: tuple[_Gateway, _Commands],
) -> None:
    gw, commands = configured

    first = exploration.start_exploration_run(gw, "request-a")
    replay = exploration.start_exploration_run(gw, "request-a")

    assert first["exploration_run_id"] == RUN_ID
    assert first["accepted"] is True
    assert first["request_id"] == "request-a"
    assert replay["replay"] is True
    assert len(commands.calls) == 1
    assert commands.calls[0] == (
        "start",
        {
            "exploration_run_id": RUN_ID,
            "session_id": "product-session-a",
            "reason": "gateway_start",
            "request_id": "request-a",
        },
    )


def test_changed_context_with_same_request_conflicts(
    configured: tuple[_Gateway, _Commands], monkeypatch: pytest.MonkeyPatch
) -> None:
    gw, _commands = configured
    exploration.start_exploration_run(gw, "request-a")
    changed = _binding()
    changed["session_id"] = "product-session-b"
    monkeypatch.setattr(exploration, "external_explore_binding", lambda _gw: changed)

    with pytest.raises(exploration.ExplorationRunError) as captured:
        exploration.start_exploration_run(gw, "request-a")

    assert captured.value.code == "exploration_request_conflict"


def test_pause_is_bound_to_same_run_and_product_session(
    configured: tuple[_Gateway, _Commands],
) -> None:
    gw, commands = configured
    exploration.start_exploration_run(gw, "request-a")

    result = exploration.command_exploration_run(
        gw,
        RUN_ID,
        action="pause",
        request_id="pause-request-a",
    )

    assert result["accepted"] is True
    assert commands.calls[-1] == (
        "pause",
        {
            "exploration_run_id": RUN_ID,
            "session_id": "product-session-a",
            "reason": "operator_pause",
            "request_id": "pause-request-a",
        },
    )


def test_run_command_rejects_malformed_run_id_as_client_error(
    configured: tuple[_Gateway, _Commands],
) -> None:
    gw, commands = configured

    with pytest.raises(exploration.ExplorationRunError) as captured:
        exploration.command_exploration_run(
            gw,
            "not-a-run-id",
            action="pause",
            request_id="pause-request-a",
        )

    assert captured.value.code == "exploration_run_id_invalid"
    assert captured.value.status_code == 400
    assert commands.calls == []


def test_run_command_requires_a_durable_healthy_journal(
    configured: tuple[_Gateway, _Commands],
) -> None:
    _gw, commands = configured
    memory_only = _Gateway()

    with pytest.raises(exploration.ExplorationRunError) as captured:
        exploration.command_exploration_run(
            memory_only,
            RUN_ID,
            action="finish",
            request_id="finish-request-a",
        )

    assert captured.value.code == "exploration_run_journal_unavailable"
    assert captured.value.status_code == 503
    assert commands.calls == []


def test_new_run_is_blocked_while_previous_run_truth_is_unresolved(
    configured: tuple[_Gateway, _Commands], monkeypatch: pytest.MonkeyPatch
) -> None:
    gw, _commands = configured
    gw._exploring = False
    gw._session_pending = False
    gw._session_mode = "exploring"
    gw._explorer_available = lambda: True
    gw._explore_runs.reserve_start(
        "request-a",
        product_session_id="product-session-a",
        route="map",
        map={
            "map_id": "yard-a",
            "map_version": 7,
            "artifact_hash": "a" * 64,
        },
    )
    monkeypatch.setattr(exploration, "product_control_owns_explore", lambda _gw: True)
    monkeypatch.setattr(
        "gateway.services.runtime_status.build_navigation_status",
        lambda _gw: {
            "state": "ready",
            "can_accept_goal": True,
            "readiness": {
                "can_execute_autonomy": True,
                "blockers": [],
                "advisories": [],
            },
        },
    )

    readiness = exploration.exploration_start_readiness(gw)

    assert readiness["can_start"] is False
    assert "exploration_run_unresolved" in readiness["blockers"]
    assert readiness["run_projection"]["latest"]["exploration_run_id"] == RUN_ID


def test_endpoint_restart_marks_run_interrupted_and_blocks_new_start(
    configured: tuple[_Gateway, _Commands], monkeypatch: pytest.MonkeyPatch
) -> None:
    gw, commands = configured
    exploration.start_exploration_run(gw, "request-a")
    gw._explore_runs.observe_event(_run_event())
    restarted = _binding()
    restarted["native_status"]["boot_id"] = "explore-boot-b"
    monkeypatch.setattr(exploration, "external_explore_binding", lambda _gw: restarted)

    with pytest.raises(exploration.ExplorationRunError) as captured:
        exploration.start_exploration_run(gw, "request-b")

    assert captured.value.code == "exploration_run_interrupted"
    interrupted = gw._explore_runs.query(RUN_ID)
    assert interrupted["state"] == "interrupted"
    assert interrupted["terminal"] is False
    assert interrupted["motion_stop"]["confirmed"] is False
    assert [name for name, _kwargs in commands.calls] == ["start"]


def test_interrupted_run_command_never_targets_restarted_endpoint(
    configured: tuple[_Gateway, _Commands],
) -> None:
    gw, commands = configured
    exploration.start_exploration_run(gw, "request-a")
    gw._explore_runs.observe_event(_run_event())
    gw._explore_runs.reconcile_runtime(
        product_session_id="product-session-a",
        observed_boot_id="explore-boot-b",
    )

    with pytest.raises(exploration.ExplorationRunError) as captured:
        exploration.command_exploration_run(
            gw,
            RUN_ID,
            action="finish",
            request_id="finish-request-a",
        )

    assert captured.value.code == "exploration_run_interrupted"
    assert [name for name, _kwargs in commands.calls] == ["start"]


def test_typed_receipt_identity_mismatch_fails_closed(
    configured: tuple[_Gateway, _Commands], monkeypatch: pytest.MonkeyPatch
) -> None:
    gw, commands = configured

    def wrong(**kwargs: Any) -> dict[str, Any]:
        receipt = commands._receipt("start", kwargs)
        receipt["exploration_run_id"] = "01K1M9S4FX27T8XMY6QJNBAV3X"
        return receipt

    monkeypatch.setattr(commands, "start_exploration", wrong)

    with pytest.raises(exploration.ExplorationRunError) as captured:
        exploration.start_exploration_run(gw, "request-a")

    assert captured.value.code == "exploration_receipt_identity_mismatch"
    assert captured.value.detail["exploration_run_id"] == RUN_ID


def test_post_ack_journal_failure_requests_native_stop(
    configured: tuple[_Gateway, _Commands], monkeypatch: pytest.MonkeyPatch
) -> None:
    gw, commands = configured

    def fail_record(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
        raise ExploreRunJournalUnavailable("disk full")

    monkeypatch.setattr(gw._explore_runs, "record_admission", fail_record)

    with pytest.raises(exploration.ExplorationRunError) as captured:
        exploration.start_exploration_run(gw, "request-a")

    assert captured.value.code == "exploration_run_journal_unavailable"
    assert [name for name, _kwargs in commands.calls] == ["start", "finish"]
    stop = commands.calls[-1][1]
    assert stop["exploration_run_id"] == RUN_ID
    assert stop["session_id"] == "product-session-a"
    assert stop["reason"] == "exploration_journal_write_failed"
    assert len(stop["request_id"]) == 26
    assert captured.value.detail["compensating_stop"]["accepted"] is True


def test_product_start_requires_a_durable_journal(
    configured: tuple[_Gateway, _Commands],
) -> None:
    _gw, commands = configured
    memory_only = _Gateway()

    with pytest.raises(exploration.ExplorationRunError) as captured:
        exploration.start_exploration_run(memory_only, "request-a")

    assert captured.value.code == "exploration_run_journal_unavailable"
    assert commands.calls == []


def test_saved_map_route_cannot_save_as_a_new_mapping_result(
    configured: tuple[_Gateway, _Commands],
) -> None:
    gw, _commands = configured

    readiness = exploration.exploration_map_save_readiness(gw)

    assert readiness == {
        "can_save": False,
        "reason": "map_save_requires_live_exploration",
        "message": "Saved-map coverage exploration does not create a new mapping result.",
    }


def test_live_map_save_requires_native_idle_after_confirmed_run_terminal(
    configured: tuple[_Gateway, _Commands], monkeypatch: pytest.MonkeyPatch
) -> None:
    gw, _commands = configured
    binding = _binding()
    binding["variant"] = "live"
    binding["native_status"] = {
        **binding["native_status"],
        "active": True,
        "state": "executing",
        "pending_goal": {"request_id": "goal-a"},
        "map": None,
    }
    monkeypatch.setattr(exploration, "external_explore_binding", lambda _gw: binding)

    readiness = exploration.exploration_map_save_readiness(gw)

    assert readiness["can_save"] is False
    assert readiness["reason"] == "native_exploration_motion_not_idle"


def test_live_map_save_allows_no_run_only_when_native_endpoint_is_idle(
    configured: tuple[_Gateway, _Commands], monkeypatch: pytest.MonkeyPatch
) -> None:
    gw, _commands = configured
    binding = _binding()
    binding["variant"] = "live"
    binding["native_status"] = {
        **binding["native_status"],
        "map": None,
    }
    monkeypatch.setattr(exploration, "external_explore_binding", lambda _gw: binding)

    assert exploration.exploration_map_save_readiness(gw) == {
        "can_save": True,
        "reason": "exploration_safely_parked",
        "message": "Exploration is idle and any retained run has confirmed parking.",
    }


def test_native_run_event_is_persisted_before_exact_sse_projection(
    configured: tuple[_Gateway, _Commands],
) -> None:
    from gateway.services.event_handlers import handle_exploration_run_event

    gw, _commands = configured
    gw._explore_runs.reserve_start(
        "request-a",
        product_session_id="product-session-a",
        route="map",
        map={
            "map_id": "yard-a",
            "map_version": 7,
            "artifact_hash": "a" * 64,
        },
    )
    gw._explore_runs.record_admission(
        RUN_ID,
        accepted=True,
        reason="exploration_started",
    )
    event = _run_event()

    handle_exploration_run_event(gw, event)

    assert gw._explore_runs.query(RUN_ID)["state"] == "running"
    assert gw._exploring is True
    assert gw.events[-1] == {"type": "exploration_run_event", "data": event}


def test_native_run_event_journal_failure_requests_one_fail_safe_stop(
    configured: tuple[_Gateway, _Commands], monkeypatch: pytest.MonkeyPatch
) -> None:
    import gateway.services.command_boundary as command_boundary
    from gateway.services.event_handlers import handle_exploration_run_event

    gw, commands = configured

    def fail_observe(_event: object) -> dict[str, Any]:
        raise ExploreRunJournalUnavailable("disk full")

    monkeypatch.setattr(gw._explore_runs, "observe_event", fail_observe)
    monkeypatch.setattr(command_boundary, "navigation_commands", lambda _gw: commands)
    event = _run_event()

    handle_exploration_run_event(gw, event)
    handle_exploration_run_event(gw, event)

    assert [name for name, _kwargs in commands.calls] == ["finish"]
    stop = commands.calls[0][1]
    assert stop["exploration_run_id"] == RUN_ID
    assert stop["session_id"] == "product-session-a"
    assert stop["reason"] == "exploration_run_projection_failed"
    assert gw.events[0]["type"] == "exploration_run_journal_error"
    assert gw.events[0]["data"]["compensating_stop"]["accepted"] is True
    assert gw.events[1]["data"]["compensating_stop"]["reason"] == "already_requested"


def test_rejected_false_terminal_requests_stop_for_the_known_active_run(
    configured: tuple[_Gateway, _Commands], monkeypatch: pytest.MonkeyPatch
) -> None:
    import gateway.services.command_boundary as command_boundary
    from gateway.services.event_handlers import handle_exploration_run_event

    gw, commands = configured
    exploration.start_exploration_run(gw, "request-a")
    monkeypatch.setattr(command_boundary, "navigation_commands", lambda _gw: commands)
    event = _run_event(state="cancelled")
    event["motion_stop_confirmed"] = False
    event["motion_stop_reason"] = ""

    handle_exploration_run_event(gw, event)

    assert gw._explore_runs.query(RUN_ID)["terminal"] is False
    assert [name for name, _kwargs in commands.calls] == ["start", "finish"]
    stop = commands.calls[-1][1]
    assert stop["exploration_run_id"] == RUN_ID
    assert stop["session_id"] == "product-session-a"
    assert stop["reason"] == "exploration_run_projection_failed"
    assert gw.events[-1]["type"] == "exploration_run_event_rejected"
    assert gw.events[-1]["data"]["compensating_stop"]["accepted"] is True
