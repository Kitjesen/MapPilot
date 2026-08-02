# ruff: noqa: S101

from __future__ import annotations

import time

import pytest
from fastapi.testclient import TestClient

from gateway.gateway_module import GatewayModule
from gateway.services.event_handlers import handle_navigation_goal_status


class _TaskControls:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, str, str]] = []

    def submit_pause(
        self,
        reason: str,
        *,
        task_id: str | None = None,
        request_id: str | None = None,
    ) -> dict[str, object]:
        self.calls.append(("pause", str(task_id), str(request_id), reason))
        return {
            "accepted": True,
            "state": "pause_requested",
            "task_id": task_id,
            "request_id": request_id,
        }

    def submit_resume(
        self,
        reason: str,
        *,
        task_id: str | None = None,
        request_id: str | None = None,
    ) -> dict[str, object]:
        self.calls.append(("resume", str(task_id), str(request_id), reason))
        return {
            "accepted": True,
            "state": "resume_requested",
            "task_id": task_id,
            "request_id": request_id,
        }


def _native_status(
    *,
    sequence: int,
    state_name: str,
    request_id: str,
) -> dict[str, object]:
    return {
        "ts": time.time(),
        "frame_id": "map",
        "boot_id": "navd-boot-1",
        "sequence": sequence,
        "task_id": "navigation-task-1",
        "request_id": request_id,
        "state": sequence,
        "state_name": state_name,
        "goal_epoch": 7,
        "reason": "",
        "terminal": False,
    }


def test_task_pause_and_resume_ack_do_not_advance_state_before_native_events() -> None:
    controls = _TaskControls()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": controls})
    assert gateway._lease.acquire("web", 30.0) is True
    client = TestClient(gateway._app)

    handle_navigation_goal_status(
        gateway,
        _native_status(sequence=1, state_name="PATH_ACTIVE", request_id="goal-1"),
    )

    pause = client.post(
        "/api/v1/navigation/tasks/navigation-task-1/pause",
        json={
            "reason": "operator_pause",
            "request_id": "pause-1",
            "client_id": "web",
        },
    )

    assert pause.status_code == 202
    assert pause.json()["status"] == "pause_requested"
    assert pause.json()["task_id"] == "navigation-task-1"
    assert pause.json()["command"]["request_id"] == "pause-1"
    assert pause.json()["execution_confirmed"] is False
    assert pause.json()["final_output_confirmed"] is False
    assert (
        client.get("/api/v1/navigation/tasks/navigation-task-1").json()["status"]["state_name"]
        == "PATH_ACTIVE"
    )

    handle_navigation_goal_status(
        gateway,
        _native_status(sequence=2, state_name="PAUSED", request_id="pause-1"),
    )
    assert (
        client.get("/api/v1/navigation/tasks/navigation-task-1").json()["status"]["state_name"]
        == "PAUSED"
    )

    resume = client.post(
        "/api/v1/navigation/tasks/navigation-task-1/resume",
        json={
            "reason": "operator_resume",
            "request_id": "resume-1",
            "client_id": "web",
        },
    )

    assert resume.status_code == 202
    assert resume.json()["status"] == "resume_requested"
    assert resume.json()["task_id"] == "navigation-task-1"
    assert resume.json()["command"]["request_id"] == "resume-1"
    assert resume.json()["goal_reissue_required"] is False
    assert resume.json()["execution_confirmed"] is False
    assert resume.json()["final_output_confirmed"] is False
    assert (
        client.get("/api/v1/navigation/tasks/navigation-task-1").json()["status"]["state_name"]
        == "PAUSED"
    )

    handle_navigation_goal_status(
        gateway,
        _native_status(sequence=3, state_name="PATH_ACTIVE", request_id="resume-1"),
    )
    assert (
        client.get("/api/v1/navigation/tasks/navigation-task-1").json()["status"]["state_name"]
        == "PATH_ACTIVE"
    )
    assert controls.calls == [
        ("pause", "navigation-task-1", "pause-1", "operator_pause"),
        ("resume", "navigation-task-1", "resume-1", "operator_resume"),
    ]


@pytest.mark.parametrize("action", ["pause", "resume"])
def test_task_control_command_ack_event_uses_http_accepted_status(action: str) -> None:
    controls = _TaskControls()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": controls})
    assert gateway._lease.acquire("web", 30.0) is True
    queue = gateway._sse_subscribe()

    try:
        response = TestClient(gateway._app).post(
            f"/api/v1/navigation/tasks/navigation-task-1/{action}",
            json={"request_id": f"{action}-event-1", "client_id": "web"},
        )
        event = queue.get_nowait()
    finally:
        gateway._sse_unsubscribe(queue)

    assert response.status_code == 202
    assert event["type"] == "command_ack"
    assert event["data"]["status_code"] == 202
    assert event["data"]["command"]["name"] == f"navigation_task_{action}"



def test_task_pause_remains_available_during_safety_stop_and_foreign_lease() -> None:
    controls = _TaskControls()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": controls})
    assert gateway._lease.acquire("another-operator", 30.0) is True
    with gateway._state_lock:
        gateway._safety = {"level": 2}

    response = TestClient(gateway._app).post(
        "/api/v1/navigation/tasks/navigation-task-1/pause",
        json={
            "request_id": "pause-during-stop",
            "client_id": "web",
        },
    )

    assert response.status_code == 202
    assert response.json()["status"] == "pause_requested"
    assert controls.calls == [
        (
            "pause",
            "navigation-task-1",
            "pause-during-stop",
            "operator_pause",
        )
    ]


def test_task_resume_requires_the_active_control_lease() -> None:
    controls = _TaskControls()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": controls})
    assert gateway._lease.acquire("another-operator", 30.0) is True

    response = TestClient(gateway._app).post(
        "/api/v1/navigation/tasks/navigation-task-1/resume",
        json={
            "request_id": "resume-without-lease",
            "client_id": "web",
        },
    )

    assert response.status_code == 403
    assert response.json()["error"] == "control_lease"
    assert controls.calls == []


def test_task_resume_is_rejected_while_safety_stop_is_active() -> None:
    controls = _TaskControls()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": controls})
    assert gateway._lease.acquire("web", 30.0) is True
    with gateway._state_lock:
        gateway._safety = {"level": 2}

    response = TestClient(gateway._app).post(
        "/api/v1/navigation/tasks/navigation-task-1/resume",
        json={
            "request_id": "resume-during-stop",
            "client_id": "web",
        },
    )

    assert response.status_code == 409
    assert response.json()["error"] == "safety_stop"
    assert controls.calls == []



def test_pause_generates_a_delivery_request_id_when_the_client_omits_one() -> None:
    controls = _TaskControls()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": controls})

    response = TestClient(gateway._app).post(
        "/api/v1/navigation/tasks/navigation-task-1/pause",
        json={"client_id": "web"},
    )

    request_id = response.json()["command"]["request_id"]
    assert response.status_code == 202
    assert request_id.startswith("nav-pause-")
    assert request_id != "navigation-task-1"
    assert controls.calls == [
        ("pause", "navigation-task-1", request_id, "operator_pause")
    ]


@pytest.mark.parametrize("action", ["pause", "resume"])
def test_task_control_rejects_a_body_task_that_conflicts_with_the_route(
    action: str,
) -> None:
    controls = _TaskControls()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": controls})

    response = TestClient(gateway._app).post(
        f"/api/v1/navigation/tasks/navigation-task-1/{action}",
        json={
            "task_id": "another-navigation-task",
            "request_id": f"{action}-wrong-task",
            "client_id": "web",
        },
    )

    assert response.status_code == 409
    assert response.json()["error"] == "task_identity_conflict"
    assert controls.calls == []


def test_native_resume_rejection_preserves_the_paused_read_model() -> None:
    class RejectingControls(_TaskControls):
        def submit_resume(
            self,
            reason: str,
            *,
            task_id: str | None = None,
            request_id: str | None = None,
        ) -> dict[str, object]:
            self.calls.append(("resume", str(task_id), str(request_id), reason))
            return {
                "accepted": False,
                "state": "rejected",
                "task_id": task_id,
                "request_id": request_id,
                "message": "task_not_paused",
            }

    controls = RejectingControls()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": controls})
    assert gateway._lease.acquire("web", 30.0) is True
    client = TestClient(gateway._app)
    handle_navigation_goal_status(
        gateway,
        _native_status(sequence=1, state_name="PAUSED", request_id="pause-1"),
    )

    response = client.post(
        "/api/v1/navigation/tasks/navigation-task-1/resume",
        json={"request_id": "resume-rejected", "client_id": "web"},
    )

    assert response.status_code == 409
    assert response.json()["error"] == "native_command_rejected"
    assert (
        client.get("/api/v1/navigation/tasks/navigation-task-1").json()["status"]["state_name"]
        == "PAUSED"
    )


def test_unknown_task_rejection_does_not_create_task_read_model_state() -> None:
    class MissingTaskControls(_TaskControls):
        def submit_pause(
            self,
            reason: str,
            *,
            task_id: str | None = None,
            request_id: str | None = None,
        ) -> dict[str, object]:
            self.calls.append(("pause", str(task_id), str(request_id), reason))
            return {
                "accepted": False,
                "state": "rejected",
                "task_id": task_id,
                "request_id": request_id,
                "message": "task_not_active",
            }

    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": MissingTaskControls()})
    client = TestClient(gateway._app)

    response = client.post(
        "/api/v1/navigation/tasks/navigation-task-missing/pause",
        json={"request_id": "pause-missing", "client_id": "web"},
    )
    status = client.get("/api/v1/navigation/tasks/navigation-task-missing").json()

    assert response.status_code == 409
    assert response.json()["error"] == "native_command_rejected"
    assert status["found"] is False
    assert status["reason"] == "task_status_unknown"


@pytest.mark.parametrize(
    ("field", "invalid_value"),
    [
        ("task_id", "another-task"),
        ("request_id", "another-request"),
        ("state", "PAUSED"),
    ],
)
def test_task_pause_rejects_an_ack_with_the_wrong_identity_or_state(
    field: str,
    invalid_value: str,
) -> None:
    class InvalidAckControls(_TaskControls):
        def submit_pause(
            self,
            reason: str,
            *,
            task_id: str | None = None,
            request_id: str | None = None,
        ) -> dict[str, object]:
            result = super().submit_pause(
                reason,
                task_id=task_id,
                request_id=request_id,
            )
            result[field] = invalid_value
            return result

    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.goals": InvalidAckControls()})

    response = TestClient(gateway._app).post(
        "/api/v1/navigation/tasks/navigation-task-1/pause",
        json={"request_id": "pause-invalid-ack", "client_id": "web"},
    )

    assert response.status_code == 409
    assert response.json()["error"] == "native_command_rejected"

