from __future__ import annotations

from threading import RLock

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
    assert result["status"]["state_name"] == "REACHED"
    assert result["status"]["terminal"] is True
    assert len(gateway.events) == 2


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
    assert result["status"]["state_name"] == "REACHED"
    assert result["status"]["terminal"] is True


def test_gateway_deduplicates_replayed_goal_status_sequence() -> None:
    gateway = _Gateway()

    handle_navigation_goal_status(gateway, _status(sequence=2))
    handle_navigation_goal_status(
        gateway,
        _status(sequence=1, state=NavigationGoalState.REACHED),
    )

    result = query_navigation_goal_status(gateway, "goal-1")

    assert result["status"]["state_name"] == "PATH_ACTIVE"
    assert len(gateway.events) == 1


def test_gateway_goal_status_query_is_read_only_and_explicit_when_unknown() -> None:
    gateway = _Gateway()

    missing = query_navigation_goal_status(gateway, "goal-missing")
    invalid = query_navigation_goal_status(gateway, " ")

    assert missing["found"] is False
    assert missing["reason"] == "request_status_unknown"
    assert invalid["found"] is False
    assert invalid["reason"] == "request_id_required"
