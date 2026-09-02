from __future__ import annotations

from threading import RLock

import pytest

from gateway.services.event_handlers import handle_navigation_goal_status
from gateway.services.navigation_lifecycle import (
    query_navigation_goal_status,
    query_navigation_task_status,
)
from runtime.msgs.nav import NavigationGoalState, NavigationGoalStatus


class _Gateway:
    def __init__(self) -> None:
        self._state_lock = RLock()
        self._navigation_goal_status_sequences: dict[str, int] = {}
        self._navigation_goal_status_by_task: dict[str, dict] = {}
        self._navigation_goal_status_by_request: dict[str, dict] = {}
        self._latest_navigation_goal_status: dict | None = None
        self.events: list[dict] = []

    def push_event(self, event: dict) -> None:
        self.events.append(event)


class _DurableGoals:
    def __init__(self, record: dict | None) -> None:
        self.record = record
        self.queries: list[str] = []

    def get_task(self, task_id: str) -> dict | None:
        self.queries.append(task_id)
        return self.record


def _status(
    *,
    sequence: int,
    task_id: str = "navigation-task-1",
    request_id: str = "goal-1",
    state: NavigationGoalState = NavigationGoalState.PATH_ACTIVE,
) -> NavigationGoalStatus:
    return NavigationGoalStatus(
        ts=42.0 + sequence,
        frame_id="map",
        boot_id="navd-boot",
        sequence=sequence,
        task_id=task_id,
        request_id=request_id,
        state=int(state),
        goal_epoch=3,
    )


def test_gateway_retains_terminal_status_by_request() -> None:
    gateway = _Gateway()

    handle_navigation_goal_status(gateway, _status(sequence=1))
    handle_navigation_goal_status(
        gateway,
        _status(sequence=2, state=NavigationGoalState.REACHED),
    )

    result = query_navigation_goal_status(gateway, "goal-1")

    assert result["found"] is True
    assert result["status"]["state_name"] == "SUCCESS"
    assert result["status"]["lifecycle_state_name"] == "SUCCESS"
    assert result["status"]["phase"] == "REACHED"
    assert result["status"]["terminal"] is True
    assert len(gateway.events) == 2
    assert gateway.events[-1]["data"]["state_name"] == "SUCCESS"


def test_gateway_retains_terminal_status_by_stable_task_identity() -> None:
    gateway = _Gateway()

    handle_navigation_goal_status(gateway, _status(sequence=1))
    handle_navigation_goal_status(
        gateway,
        _status(sequence=2, state=NavigationGoalState.REACHED),
    )

    result = query_navigation_task_status(gateway, "navigation-task-1")

    assert result["found"] is True
    assert result["task_id"] == "navigation-task-1"
    assert result["status"]["task_id"] == "navigation-task-1"
    assert result["status"]["request_id"] == "goal-1"
    assert result["status"]["state_name"] == "SUCCESS"
    assert result["status"]["phase"] == "REACHED"
    assert result["status"]["terminal"] is True


def test_gateway_deduplicates_replayed_goal_status_sequence() -> None:
    gateway = _Gateway()

    handle_navigation_goal_status(gateway, _status(sequence=2))
    handle_navigation_goal_status(
        gateway,
        _status(sequence=1, state=NavigationGoalState.REACHED),
    )

    result = query_navigation_goal_status(gateway, "goal-1")

    assert result["status"]["state_name"] == "EXECUTING"
    assert result["status"]["phase"] == "PATH_ACTIVE"
    assert len(gateway.events) == 1


def test_gateway_goal_status_query_is_read_only_and_explicit_when_unknown() -> None:
    gateway = _Gateway()

    missing = query_navigation_goal_status(gateway, "goal-missing")
    invalid = query_navigation_goal_status(gateway, " ")

    assert missing["found"] is False
    assert missing["reason"] == "request_status_unknown"
    assert invalid["found"] is False
    assert invalid["reason"] == "request_id_required"


@pytest.mark.parametrize(
    ("native_state", "public_state", "terminal"),
    [
        (NavigationGoalState.PLANNING, "PLANNING", False),
        (NavigationGoalState.PATH_ACTIVE, "EXECUTING", False),
        (NavigationGoalState.PAUSED, "PAUSED", False),
        (NavigationGoalState.REACHED, "SUCCESS", True),
        (NavigationGoalState.FAILED, "FAILED", True),
        (NavigationGoalState.CANCELLED, "CANCELLED", True),
    ],
)
def test_gateway_exposes_only_canonical_task_lifecycle_states(
    native_state: NavigationGoalState,
    public_state: str,
    terminal: bool,
) -> None:
    gateway = _Gateway()

    handle_navigation_goal_status(
        gateway,
        _status(sequence=1, state=native_state),
    )

    result = query_navigation_task_status(gateway, "navigation-task-1")
    status = result["status"]
    assert status["state_name"] == public_state
    assert status["lifecycle_state_name"] == public_state
    assert status["phase"] == native_state.name
    assert status["native_state"] == int(native_state)
    assert status["terminal"] is terminal
    assert status["task_id"] == "navigation-task-1"
    assert status["request_id"] == "goal-1"


def test_gateway_rejects_unknown_task_phase_instead_of_publishing_an_eighth_state() -> None:
    gateway = _Gateway()
    payload = _status(sequence=1).to_dict()
    payload["state_name"] = "WAITING_FOR_SOMETHING"

    handle_navigation_goal_status(gateway, payload)

    assert query_navigation_task_status(gateway, "navigation-task-1")["found"] is False
    assert gateway.events == []


def test_task_query_falls_back_to_retained_native_status_without_mutating_live_state() -> None:
    gateway = _Gateway()
    goals = _DurableGoals(
        {
            "task_id": "navigation-task-1",
            "evidence_status": "retained",
            "last_goal_status": _status(
                sequence=9,
                state=NavigationGoalState.REACHED,
            ).to_dict(),
        }
    )
    gateway._goals = goals

    result = query_navigation_task_status(gateway, "navigation-task-1")

    assert result["found"] is True
    assert result["source"] == "durable_native_goal_status"
    assert result["evidence_status"] == "retained"
    assert result["status"]["state_name"] == "SUCCESS"
    assert result["status"]["phase"] == "REACHED"
    assert goals.queries == ["navigation-task-1"]
    assert gateway._navigation_goal_status_by_task == {}
    assert gateway.events == []


def test_task_query_never_infers_terminal_state_from_ledger_summary() -> None:
    gateway = _Gateway()
    goals = _DurableGoals(
        {
            "task_id": "navigation-task-1",
            "state": "reached",
            "terminal": True,
            "evidence_status": "retained",
            "last_goal_status": None,
        }
    )
    gateway._goals = goals

    result = query_navigation_task_status(gateway, "navigation-task-1")

    assert result["found"] is False
    assert result["source"] == "durable_task_ledger"
    assert result["reason"] == "native_task_status_not_observed"
    assert result["status"] is None
    assert gateway.events == []
