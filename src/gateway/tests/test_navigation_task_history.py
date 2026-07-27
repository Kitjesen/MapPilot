from __future__ import annotations

import uuid
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")

from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes.status import register_status_routes
from nav.services.goals import GoalService
from runtime.msgs.geometry import Pose, PoseStamped, Vector3


def _task(task_id: str = "nav-task-001", *, terminal: bool = False) -> dict:
    return {
        "task_id": task_id,
        "target": {"x": 1.0, "y": 2.0, "yaw": 0.0},
        "product_fingerprint": "product-sha256",
        "map_identity": {"map_id": "yard", "map_version": "v3"},
        "admission": "accepted",
        "admission_reason": "accepted",
        "execution_state": "reached" if terminal else "executing",
        "execution_reason": "goal_reached" if terminal else "",
        "cancel_requested": False,
        "evidence_status": "fresh",
        "state_source": "native_goal_status",
        "state_observed_at": 101.0,
        "created_at": 100.0,
        "updated_at": 101.0,
    }


def _client(goals=None) -> TestClient:
    app = FastAPI()
    register_status_routes(app, SimpleNamespace(_goals=goals))
    return TestClient(app, raise_server_exceptions=False)


def test_navigation_task_detail_is_a_read_only_history_lookup() -> None:
    class Goals:
        calls: list[str] = []

        def get_task(self, task_id: str):
            self.calls.append(task_id)
            return _task(task_id)

    goals = Goals()
    response = _client(goals).get("/api/v1/navigation/tasks/nav-task-001")

    assert response.status_code == 200
    assert response.json() == {
        "schema_version": 2,
        "found": True,
        "task": _task(),
        "reason": None,
        "ts": response.json()["ts"],
    }
    assert goals.calls == ["nav-task-001"]


def test_navigation_task_detail_accepts_native_paused_state() -> None:
    class Goals:
        def get_task(self, task_id: str):
            task = _task(task_id)
            task["execution_state"] = "paused"
            task["execution_reason"] = "operator_pause"
            return task

    response = _client(Goals()).get("/api/v1/navigation/tasks/nav-task-paused")

    assert response.status_code == 200
    assert response.json()["task"]["execution_state"] == "paused"
    assert response.json()["task"]["execution_reason"] == "operator_pause"


def test_navigation_task_detail_reports_unknown_task_without_500() -> None:
    class Goals:
        def get_task(self, task_id: str):
            return None

    response = _client(Goals()).get("/api/v1/navigation/tasks/missing")

    assert response.status_code == 200
    assert response.json()["found"] is False
    assert response.json()["task"] is None
    assert response.json()["reason"] == "task_not_found"


def test_navigation_task_list_passes_user_filters_to_goal_service() -> None:
    class Goals:
        calls: list[tuple[int, bool]] = []

        def list_tasks(self, *, limit: int = 50, active_only: bool = False):
            self.calls.append((limit, active_only))
            return [_task("nav-task-active")]

    goals = Goals()
    response = _client(goals).get("/api/v1/navigation/tasks?limit=25&active_only=true")

    assert response.status_code == 200
    payload = response.json()
    assert payload["schema_version"] == 2
    assert payload["tasks"] == [_task("nav-task-active")]
    assert payload["count"] == 1
    assert payload["limit"] == 25
    assert payload["active_only"] is True
    assert payload["reason"] is None
    assert goals.calls == [(25, True)]


@pytest.mark.parametrize("limit", [0, 101])
def test_navigation_task_list_rejects_limits_outside_one_to_one_hundred(
    limit: int,
) -> None:
    response = _client(SimpleNamespace(list_tasks=lambda **_: [])).get(f"/api/v1/navigation/tasks?limit={limit}")

    assert response.status_code == 422


@pytest.mark.parametrize("goals", [None, SimpleNamespace()])
def test_navigation_task_routes_report_service_unavailable_without_500(goals) -> None:
    detail = _client(goals).get("/api/v1/navigation/tasks/nav-task-001")
    listing = _client(goals).get("/api/v1/navigation/tasks")

    assert detail.status_code == 200
    assert detail.json()["found"] is False
    assert detail.json()["reason"] == "navigation_task_service_unavailable"
    assert listing.status_code == 200
    assert listing.json()["tasks"] == []
    assert listing.json()["count"] == 0
    assert listing.json()["reason"] == "navigation_task_service_unavailable"


def test_navigation_task_routes_fail_closed_for_non_json_safe_service_records() -> None:
    class Goals:
        def get_task(self, task_id: str):
            return {**_task(task_id), "unsafe": object()}

        def list_tasks(self, *, limit: int = 50, active_only: bool = False):
            return [{**_task(), "updated_at": float("nan")}]

    detail = _client(Goals()).get("/api/v1/navigation/tasks/nav-task-001")
    listing = _client(Goals()).get("/api/v1/navigation/tasks")

    assert detail.status_code == 200
    assert detail.json()["found"] is False
    assert detail.json()["task"] is None
    assert detail.json()["reason"] == "navigation_task_record_invalid"
    assert listing.status_code == 200
    assert listing.json()["tasks"] == []
    assert listing.json()["reason"] == "navigation_task_record_invalid"


def test_navigation_task_routes_do_not_leak_json_safe_internal_fields() -> None:
    task = _task()
    task.update(
        {
            "state": "failed",
            "terminal": True,
            "reason": "host_inferred_failure",
            "can_resume": False,
        }
    )
    task["internal_db_path"] = "/var/lib/lingtu/navigation-tasks.sqlite3"
    task["attempts"] = [
        {
            "request_id": "request-001",
            "kind": "goal",
            "state": "accepted",
            "created_at": 100.0,
            "updated_at": 101.0,
            "internal_db_path": "/private/attempt.sqlite3",
        }
    ]
    task["events"] = [
        {
            "id": 1,
            "type": "accepted",
            "state": "accepted",
            "created_at": 101.0,
            "internal_db_path": "/private/events.sqlite3",
        }
    ]

    class Goals:
        def get_task(self, task_id: str):
            return {**task, "task_id": task_id}

        def list_tasks(self, *, limit: int = 50, active_only: bool = False):
            return [task]

    detail = _client(Goals()).get("/api/v1/navigation/tasks/nav-task-001")
    listing = _client(Goals()).get("/api/v1/navigation/tasks")

    assert detail.status_code == 200
    assert detail.json()["found"] is True
    assert listing.status_code == 200
    assert listing.json()["count"] == 1
    assert {"state", "terminal", "reason", "can_resume", "attempts", "events"}.isdisjoint(
        detail.json()["task"]
    )
    assert "internal_db_path" not in detail.text
    assert "internal_db_path" not in listing.text


def test_navigation_task_service_exceptions_are_reported_without_internal_details() -> None:
    class Goals:
        def get_task(self, task_id: str):
            raise RuntimeError("database path and secret details")

        def list_tasks(self, *, limit: int = 50, active_only: bool = False):
            raise RuntimeError("database path and secret details")

    detail = _client(Goals()).get("/api/v1/navigation/tasks/nav-task-001")
    listing = _client(Goals()).get("/api/v1/navigation/tasks")

    assert detail.status_code == 200
    assert detail.json()["reason"] == "navigation_task_service_unavailable"
    assert "secret" not in detail.text
    assert listing.status_code == 200
    assert listing.json()["reason"] == "navigation_task_service_unavailable"
    assert "secret" not in listing.text


def test_navigation_task_reads_are_offloaded_with_asyncio_to_thread(monkeypatch) -> None:
    import gateway.routes.status as status_routes

    calls: list[tuple[object, tuple, dict]] = []

    async def fake_to_thread(function, /, *args, **kwargs):
        calls.append((function, args, kwargs))
        return function(*args, **kwargs)

    class Goals:
        def get_task(self, task_id: str):
            return _task(task_id)

        def list_tasks(self, *, limit: int = 50, active_only: bool = False):
            return [_task()]

    monkeypatch.setattr(status_routes.asyncio, "to_thread", fake_to_thread)
    client = _client(Goals())

    assert client.get("/api/v1/navigation/tasks/nav-task-001").status_code == 200
    assert client.get("/api/v1/navigation/tasks").status_code == 200
    assert len(calls) == 2


def test_navigation_task_read_routes_do_not_shadow_cancel_route_shape() -> None:
    app = FastAPI()
    register_status_routes(app, SimpleNamespace(_goals=None))

    routes = {(route.path, method) for route in app.routes for method in getattr(route, "methods", set())}
    assert ("/api/v1/navigation/tasks", "GET") in routes
    assert ("/api/v1/navigation/tasks/{task_id}", "GET") in routes
    assert ("/api/v1/navigation/tasks/{task_id}/cancel", "GET") not in routes


def test_navigation_task_routes_read_real_goal_service_sqlite_history() -> None:
    db_path = Path.cwd() / f".test-navigation-tasks-{uuid.uuid4().hex}.sqlite3"
    service = GoalService(task_ledger_path=str(db_path))
    service.setup()
    try:
        receipt = service.submit_goal(
            PoseStamped(
                pose=Pose(position=Vector3(1.5, -0.5, 0.0)),
                frame_id="map",
            ),
            task_id="nav-task-real-service",
            request_id="request-real-service",
        )

        assert receipt["accepted"] is True
        with _client(service) as client:
            detail = client.get("/api/v1/navigation/tasks/nav-task-real-service")
            listing = client.get("/api/v1/navigation/tasks?active_only=true")

        assert detail.status_code == 200
        task = detail.json()["task"]
        assert detail.json()["found"] is True
        assert task["task_id"] == "nav-task-real-service"
        assert task["admission"] == "accepted"
        assert task["execution_state"] is None
        assert "attempts" not in task
        assert listing.status_code == 200
        assert listing.json()["reason"] is None
        assert [item["task_id"] for item in listing.json()["tasks"]] == ["nav-task-real-service"]
    finally:
        service.stop()
        for suffix in ("", "-shm", "-wal"):
            Path(f"{db_path}{suffix}").unlink(missing_ok=True)
