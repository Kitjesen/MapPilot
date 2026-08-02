from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")


class _Store:
    routes: dict[tuple[str, str], dict] = {}
    deleted: list[tuple[str, str]] = []

    def __init__(self, map_root):
        self.map_root = map_root

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return None

    def put(self, route: dict):
        stored = {
            **route,
            "points": [
                {
                    "id": point["id"],
                    "x": point["x"],
                    "y": point["y"],
                    "z": point["z"],
                    "yaw": point["yaw"],
                    "tolerance": point["position_tolerance_m"],
                    "dwell": point["dwell_s"],
                    "action": point["action"],
                    "enabled": point["enabled"],
                }
                for point in route["points"]
            ],
        }
        self.routes[(route["map_id"], route["id"])] = stored
        return stored

    def get(self, map_id: str, route_id: str):
        return self.routes[(map_id, route_id)]

    def list(self, map_id: str):
        return {"routes": [route for (stored_map, _), route in self.routes.items() if stored_map == map_id]}

    def status(self):
        return {
            "state": "running",
            "route_id": "route-a",
            "current_point_index": 1,
            "point_count": 2,
        }

    def delete(self, map_id: str, route_id: str):
        self.deleted.append((map_id, route_id))
        self.routes.pop((map_id, route_id), None)


class _Client:
    def __init__(self):
        self.calls = []
        self.error: str | None = None
        self.accepted = True

    def _check(self):
        if self.error:
            raise RuntimeError(self.error)

    def start_task(self, task_id: str, route_id: str, *, revision: int = 0, request_id=None):
        self._check()
        self.calls.append(("task_start", task_id, route_id, revision, request_id))
        return self.accepted

    def pause_task(self, task_id: str, reason: str, *, request_id=None):
        self._check()
        self.calls.append(("task_pause", task_id, reason, request_id))
        return self.accepted

    def resume_task(self, task_id: str, reason: str, *, request_id=None):
        self._check()
        self.calls.append(("task_resume", task_id, reason, request_id))
        return self.accepted

    def cancel_task(self, task_id: str, reason: str, *, request_id=None):
        self._check()
        self.calls.append(("task_cancel", task_id, reason, request_id))
        return self.accepted


class _InspectionService:
    def __init__(self, client: _Client, store_type=_Store):
        self.client = client
        self.store_type = store_type

    def _store(self, method: str, *args):
        with self.store_type(Path(".")) as store:
            return getattr(store, method)(*args)

    def list_routes(self, map_id: str):
        return self._store("list", map_id)

    def put_route(self, route: dict):
        return self._store("put", route)

    def get_route(self, map_id: str, route_id: str):
        return self._store("get", map_id, route_id)

    def delete_route(self, map_id: str, route_id: str):
        self._store("delete", map_id, route_id)
        return True

    def status(self):
        return self._store("status")

    def start_task(self, task_id: str, route_id: str, revision: int = 0, request_id=None):
        return self.client.start_task(task_id, route_id, revision=revision, request_id=request_id)

    def pause_task(self, task_id: str, reason: str, request_id=None):
        return self.client.pause_task(task_id, reason, request_id=request_id)

    def resume_task(self, task_id: str, reason: str, request_id=None):
        return self.client.resume_task(task_id, reason, request_id=request_id)

    def cancel_task(self, task_id: str, reason: str, request_id=None):
        return self.client.cancel_task(task_id, reason, request_id=request_id)


def _client(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.routes.inspection as inspection

    _Store.routes = {}
    _Store.deleted = []
    native_client = _Client()
    service = _InspectionService(native_client)
    monkeypatch.setattr(inspection, "active_map_name", lambda _root: "field-map")
    monkeypatch.setattr(
        inspection,
        "map_service_query",
        lambda _gw, _request: {
            "success": True,
            "record": {"name": "field-map", "version": 3},
        },
    )

    app = FastAPI()
    gateway = SimpleNamespace(_inspection=service, _all_modules={"nav.inspection": service})
    inspection.register_inspection_routes(app, gateway)
    test_client = TestClient(app)
    test_client.app.state.gateway = gateway
    return test_client, native_client


def _route(action: str = "") -> dict:
    return {
        "id": "route-a",
        "name": "Morning loop",
        "map_id": "field-map",
        "map_version": 3,
        "revision": 1,
        "loop_count": 1,
        "failure_policy": "stop",
        "max_retries": 0,
        "points": [
            {
                "id": "dock",
                "x": 1.0,
                "y": 2.0,
                "z": 0.0,
                "yaw": None,
                "tolerance": 0.35,
                "dwell": 0.0,
                "action": action,
                "enabled": True,
            }
        ],
    }


def _persist_evidence(
    root: Path,
    *,
    request_id: str = "evidence-001",
    action: str = "capture:overview",
    run_id: str = "run-001",
    route_revision: int = 2,
):
    from perception.inspection import InspectionEvidenceRequest, InspectionEvidenceStore

    now = time.time()
    return InspectionEvidenceStore(root).persist(
        InspectionEvidenceRequest(
            run_id=run_id,
            route_id="route-a",
            route_revision=route_revision,
            map_id="field-map",
            map_version=3,
            point_id="dock",
            point_index=0,
            request_id=request_id,
            action=action,
            requested_at_s=now,
            deadline_s=now + 10.0,
        ),
        rgb_bytes=b"\xff\xd8inspection\xff\xd9",
        media_type="image/jpeg",
        pose={"frame_id": "map", "x": 1.0, "y": 2.0},
        detections={"objects": [{"label": "car", "confidence": 0.91}]},
    )


def _task_event(
    *,
    task_id: str,
    sequence: int,
    state: int,
    kind: int = 2,
    point_index: int = 0,
    point_count: int = 1,
    point_id: str = "dock",
    action: str = "capture:overview",
    evidence_id: str = "",
    reason: str = "",
) -> dict[str, object]:
    return {
        "ts": 1000.0 + sequence,
        "frame_id": "map",
        "boot_id": "navd-report-boot",
        "event_sequence": sequence,
        "kind": kind,
        "task_id": task_id,
        "request_id": f"native-report-{sequence}",
        "command_request_id": "inspection-report-start",
        "state": state,
        "map_id": "field-map",
        "map_version": 3,
        "route_id": "route-a",
        "route_revision": 7,
        "point_index": point_index,
        "point_count": point_count,
        "loop_index": 0,
        "retry_count": 0,
        "point_id": point_id,
        "action": action,
        "action_request_id": "",
        "evidence_id": evidence_id,
        "reason": reason,
    }


def test_task_report_accepts_native_success_only_with_verified_required_evidence(
    monkeypatch,
    tmp_path,
):
    from gateway.services.inspection_task_lifecycle import (
        ensure_inspection_task_timeline,
    )

    evidence_root = tmp_path / "evidence"
    evidence_status = tmp_path / "inspection_evidence_status.json"
    evidence_status.write_text(
        json.dumps(
            {
                "ready": True,
                "heartbeat_ts": time.time(),
                "supported_actions": ["capture:overview"],
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_DIR", str(evidence_root))
    monkeypatch.setenv(
        "LINGTU_INSPECTION_EVIDENCE_STATUS_FILE",
        str(evidence_status),
    )
    client, _native_client = _client(monkeypatch)
    route = _route(action="capture:overview")
    route["revision"] = 7
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200

    started = client.post(
        "/api/v1/inspection/tasks",
        json={
            "route_id": "route-a",
            "map_id": "field-map",
            "revision": 7,
            "request_id": "inspection-report-start",
        },
    )
    assert started.status_code == 202
    task_id = started.json()["task_id"]
    _persist_evidence(
        evidence_root,
        request_id="evidence-report-001",
        run_id=task_id,
        route_revision=7,
    )

    timeline = ensure_inspection_task_timeline(client.app.state.gateway)
    assert timeline.observe(
        _task_event(
            task_id=task_id,
            sequence=1,
            state=11,
            kind=5,
            evidence_id="evidence-report-001",
            reason="point_action_succeeded",
        )
    )
    assert timeline.observe(
        _task_event(
            task_id=task_id,
            sequence=2,
            state=7,
            point_index=1,
            point_id="",
            action="",
            reason="route_complete",
        )
    )

    response = client.get(f"/api/v1/inspection/tasks/{task_id}/report")

    assert response.status_code == 200
    report = response.json()
    assert report["task_id"] == task_id
    assert report["report_status"] == "COMPLETE"
    assert report["acceptance"] == "ACCEPTABLE"
    assert report["execution"] == {
        "state": "SUCCEEDED",
        "terminal": True,
        "confirmed": True,
        "reason": "route_complete",
        "history_complete": True,
        "history_reason": "",
    }
    assert report["coverage"] == {
        "required_points": 1,
        "completed_points": 1,
        "required_evidence": 1,
        "verified_evidence": 1,
        "missing_evidence": 0,
        "invalid_evidence": 0,
        "unavailable_evidence": 0,
        "unknown_evidence": 0,
    }
    assert report["points"] == [
        {
            "loop_index": 0,
            "point_index": 0,
            "point_id": "dock",
            "action": "capture:overview",
            "status": "COMPLETED",
            "evidence_status": "VERIFIED",
            "evidence_id": "evidence-report-001",
            "reason": "",
        }
    ]
    assert report["issues"] == []


def test_task_report_keeps_the_route_requirements_that_were_started(
    monkeypatch,
):
    from gateway.services.inspection_task_lifecycle import (
        ensure_inspection_task_timeline,
    )

    client, _native_client = _client(monkeypatch)
    started_route = _route(action="")
    started_route["revision"] = 7
    assert (
        client.post("/api/v1/inspection/routes", json=started_route).status_code
        == 200
    )
    started = client.post(
        "/api/v1/inspection/tasks",
        json={
            "route_id": "route-a",
            "map_id": "field-map",
            "revision": 7,
            "request_id": "inspection-report-route-snapshot",
        },
    )
    task_id = started.json()["task_id"]
    timeline = ensure_inspection_task_timeline(client.app.state.gateway)
    assert timeline.observe(
        _task_event(
            task_id=task_id,
            sequence=1,
            state=7,
            point_index=1,
            point_id="",
            action="",
            reason="route_complete",
        )
    )

    edited_route = _route(action="capture:overview")
    edited_route["revision"] = 8
    assert client.post("/api/v1/inspection/routes", json=edited_route).status_code == 200

    response = client.get(f"/api/v1/inspection/tasks/{task_id}/report")

    assert response.status_code == 200
    report = response.json()
    assert report["identity"]["route_revision"] == 7
    assert report["coverage"]["required_evidence"] == 0
    assert report["report_status"] == "COMPLETE"
    assert report["acceptance"] == "ACCEPTABLE"


def test_task_report_keeps_unfinished_required_evidence_pending(
    monkeypatch,
    tmp_path,
):
    from gateway.services.inspection_task_lifecycle import (
        ensure_inspection_task_timeline,
    )

    evidence_root = tmp_path / "evidence"
    evidence_status = tmp_path / "inspection_evidence_status.json"
    evidence_status.write_text(
        json.dumps(
            {
                "ready": True,
                "heartbeat_ts": time.time(),
                "supported_actions": ["capture:overview"],
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_DIR", str(evidence_root))
    monkeypatch.setenv(
        "LINGTU_INSPECTION_EVIDENCE_STATUS_FILE",
        str(evidence_status),
    )
    client, _native_client = _client(monkeypatch)
    route = _route(action="capture:overview")
    route["revision"] = 7
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    started = client.post(
        "/api/v1/inspection/tasks",
        json={
            "route_id": "route-a",
            "map_id": "field-map",
            "revision": 7,
            "request_id": "inspection-report-active",
        },
    )
    task_id = started.json()["task_id"]

    timeline = ensure_inspection_task_timeline(client.app.state.gateway)
    assert timeline.observe(
        _task_event(
            task_id=task_id,
            sequence=1,
            state=3,
            reason="navigating_to_point",
        )
    )

    response = client.get(f"/api/v1/inspection/tasks/{task_id}/report")

    assert response.status_code == 200
    report = response.json()
    assert report["report_status"] == "IN_PROGRESS"
    assert report["acceptance"] == "PENDING"
    assert report["coverage"]["missing_evidence"] == 0
    assert report["points"][0]["status"] == "IN_PROGRESS"
    assert report["points"][0]["evidence_status"] == "PENDING"
    assert report["issues"] == []


def test_task_report_exposes_missing_required_evidence_after_native_success(
    monkeypatch,
    tmp_path,
):
    from gateway.services.inspection_task_lifecycle import (
        ensure_inspection_task_timeline,
    )

    evidence_status = tmp_path / "inspection_evidence_status.json"
    evidence_status.write_text(
        json.dumps(
            {
                "ready": True,
                "heartbeat_ts": time.time(),
                "supported_actions": ["capture:overview"],
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv(
        "LINGTU_INSPECTION_EVIDENCE_STATUS_FILE",
        str(evidence_status),
    )
    monkeypatch.setenv(
        "LINGTU_INSPECTION_EVIDENCE_DIR",
        str(tmp_path / "evidence"),
    )
    client, _native_client = _client(monkeypatch)
    route = _route(action="capture:overview")
    route["revision"] = 7
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    started = client.post(
        "/api/v1/inspection/tasks",
        json={
            "route_id": "route-a",
            "map_id": "field-map",
            "revision": 7,
            "request_id": "inspection-report-missing",
        },
    )
    task_id = started.json()["task_id"]
    timeline = ensure_inspection_task_timeline(client.app.state.gateway)
    assert timeline.observe(
        _task_event(
            task_id=task_id,
            sequence=1,
            state=7,
            point_index=1,
            point_id="",
            action="",
            reason="route_complete",
        )
    )

    response = client.get(f"/api/v1/inspection/tasks/{task_id}/report")

    assert response.status_code == 200
    report = response.json()
    assert report["execution"]["state"] == "SUCCEEDED"
    assert report["report_status"] == "PARTIAL"
    assert report["acceptance"] == "REVIEW_REQUIRED"
    assert report["coverage"]["completed_points"] == 0
    assert report["coverage"]["missing_evidence"] == 1
    assert report["points"][0]["status"] == "MISSING_EVIDENCE"
    assert report["points"][0]["evidence_status"] == "MISSING"
    assert report["issues"][0]["code"] == "missing_evidence"


@pytest.mark.parametrize(
    ("native_state", "expected_report_status"),
    ((8, "FAILED"), (9, "CANCELLED")),
)
def test_task_report_keeps_failed_and_cancelled_execution_not_acceptable(
    monkeypatch,
    native_state,
    expected_report_status,
):
    from gateway.services.inspection_task_lifecycle import (
        ensure_inspection_task_timeline,
    )

    client, _native_client = _client(monkeypatch)
    route = _route(action="")
    route["revision"] = 7
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    started = client.post(
        "/api/v1/inspection/tasks",
        json={
            "route_id": "route-a",
            "map_id": "field-map",
            "revision": 7,
            "request_id": "inspection-report-terminal-" + str(native_state),
        },
    )
    task_id = started.json()["task_id"]
    timeline = ensure_inspection_task_timeline(client.app.state.gateway)
    assert timeline.observe(
        _task_event(
            task_id=task_id,
            sequence=1,
            state=native_state,
            point_id="dock",
            action="",
            reason="native_terminal",
        )
    )

    response = client.get(f"/api/v1/inspection/tasks/{task_id}/report")

    assert response.status_code == 200
    report = response.json()
    assert report["report_status"] == expected_report_status
    assert report["acceptance"] == "NOT_ACCEPTABLE"
    assert report["execution"]["state"] == (
        "FAILED" if native_state == 8 else "CANCELLED"
    )


def test_task_report_refuses_acceptance_when_native_event_history_is_incomplete(
    monkeypatch,
    tmp_path,
):
    from gateway.services.inspection_task_lifecycle import (
        ensure_inspection_task_timeline,
    )

    evidence_status = tmp_path / "inspection_evidence_status.json"
    evidence_status.write_text(
        json.dumps(
            {
                "ready": True,
                "heartbeat_ts": time.time(),
                "supported_actions": ["capture:overview"],
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv(
        "LINGTU_INSPECTION_EVIDENCE_STATUS_FILE",
        str(evidence_status),
    )
    monkeypatch.setenv(
        "LINGTU_INSPECTION_EVIDENCE_DIR",
        str(tmp_path / "evidence"),
    )
    client, _native_client = _client(monkeypatch)
    route = _route(action="capture:overview")
    route["revision"] = 7
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    started = client.post(
        "/api/v1/inspection/tasks",
        json={
            "route_id": "route-a",
            "map_id": "field-map",
            "revision": 7,
            "request_id": "inspection-report-history-gap",
        },
    )
    task_id = started.json()["task_id"]
    timeline = ensure_inspection_task_timeline(client.app.state.gateway)
    assert timeline.observe(
        _task_event(
            task_id=task_id,
            sequence=2,
            state=7,
            point_index=1,
            point_id="",
            action="",
            reason="route_complete",
        )
    )

    response = client.get(f"/api/v1/inspection/tasks/{task_id}/report")

    assert response.status_code == 200
    report = response.json()
    assert report["execution"]["state"] == "SUCCEEDED"
    assert report["execution"]["history_complete"] is False
    assert report["execution"]["history_reason"] == "event_sequence_gap"
    assert report["report_status"] == "UNKNOWN"
    assert report["acceptance"] == "REVIEW_REQUIRED"
    assert report["coverage"]["missing_evidence"] == 0
    assert report["coverage"]["unknown_evidence"] == 1
    assert report["points"][0]["status"] == "UNKNOWN"
    assert report["points"][0]["evidence_status"] == "UNKNOWN"
    assert report["issues"] == [
        {
            "code": "task_history_incomplete",
            "reason": "event_sequence_gap",
        }
    ]


def test_task_report_refuses_a_terminal_event_with_conflicting_route_identity(
    monkeypatch,
):
    from gateway.services.inspection_task_lifecycle import (
        ensure_inspection_task_timeline,
    )

    client, _native_client = _client(monkeypatch)
    route = _route()
    route["revision"] = 7
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    started = client.post(
        "/api/v1/inspection/tasks",
        json={
            "route_id": "route-a",
            "map_id": "field-map",
            "revision": 7,
            "request_id": "inspection-report-identity",
        },
    )
    task_id = started.json()["task_id"]
    timeline = ensure_inspection_task_timeline(client.app.state.gateway)
    assert timeline.observe(_task_event(task_id=task_id, sequence=1, state=2)) is True
    conflicting = _task_event(task_id=task_id, sequence=2, state=7)
    conflicting["route_id"] = "route-b"
    assert timeline.observe(conflicting) is False

    response = client.get(f"/api/v1/inspection/tasks/{task_id}/report")

    assert response.status_code == 200
    report = response.json()
    assert report["execution"]["state"] == "PLANNING"
    assert report["execution"]["history_complete"] is False
    assert report["report_status"] == "UNKNOWN"
    assert report["acceptance"] == "REVIEW_REQUIRED"
    assert report["terminal"] is False


def test_task_report_never_reads_a_mutable_route_for_a_task_without_a_snapshot(
    monkeypatch,
):
    from gateway.services.inspection_task_lifecycle import (
        ensure_inspection_task_timeline,
    )

    client, _native_client = _client(monkeypatch)
    route = _route()
    route["revision"] = 7
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    timeline = ensure_inspection_task_timeline(client.app.state.gateway)
    assert timeline.observe(
        _task_event(task_id="native-only-task", sequence=1, state=7)
    ) is True

    response = client.get("/api/v1/inspection/tasks/native-only-task/report")

    assert response.status_code == 409
    assert response.json()["error"] == "inspection_task_route_snapshot_unavailable"


def test_inspection_routes_use_native_store_and_client(monkeypatch):
    client, native_client = _client(monkeypatch)
    route = {
        "id": "route-a",
        "name": "Morning loop",
        "map_id": "field-map",
        "map_version": 3,
        "revision": 7,
        "loop_count": 2,
        "failure_policy": "retry",
        "max_retries": 1,
        "points": [
            {
                "id": "dock",
                "x": 1.0,
                "y": 2.0,
                "z": 0.1,
                "yaw": 0.2,
                "tolerance": 0.4,
                "dwell": 5.0,
                "action": "",
                "enabled": True,
            }
        ],
    }

    saved = client.post("/api/v1/inspection/routes", json=route)
    listed = client.get("/api/v1/inspection/routes?map_id=field-map")
    detail = client.get("/api/v1/inspection/routes/route-a?map_id=field-map")
    start = client.post(
        "/api/v1/inspection/tasks",
        json={"route_id": "route-a", "map_id": "field-map", "revision": 7, "request_id": "req-1"},
    )
    task_id = start.json()["task_id"]
    pause = client.post(
        f"/api/v1/inspection/tasks/{task_id}/pause",
        json={"reason": "hold", "request_id": "req-2"},
    )
    status = client.get("/api/v1/inspection/status")

    assert saved.status_code == 200
    assert saved.json()["route"]["points"][0]["tolerance"] == 0.4
    assert listed.status_code == 200
    assert listed.json()["count"] == 1
    assert detail.status_code == 200
    assert detail.json()["route"]["revision"] == 7
    assert start.status_code == 202
    assert pause.status_code == 202
    assert status.status_code == 200
    assert status.json()["status"]["current_point_index"] == 1
    assert native_client.calls == [
        ("task_start", task_id, "route-a", 7, "req-1"),
        ("task_pause", task_id, "hold", "req-2"),
    ]


def test_inspection_task_routes_generate_one_task_id_and_never_claim_execution(monkeypatch):
    client, native_client = _client(monkeypatch)
    route = _route()
    route["revision"] = 7
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200

    start = client.post(
        "/api/v1/inspection/tasks",
        json={
            "route_id": "route-a",
            "map_id": "field-map",
            "revision": 7,
            "request_id": "inspection-start-product-42",
        },
    )
    assert start.status_code == 202
    receipt = start.json()
    assert receipt["accepted"] is True
    assert receipt["lifecycle"] == "submission_accepted"
    assert receipt["terminal"] is False
    assert receipt["request_id"] == "inspection-start-product-42"
    assert receipt["task_id"].startswith("inspection-task-")
    task_id = receipt["task_id"]
    assert native_client.calls[-1] == (
        "task_start",
        task_id,
        "route-a",
        7,
        "inspection-start-product-42",
    )

    status = client.get(f"/api/v1/inspection/tasks/{task_id}")
    assert status.status_code == 200
    snapshot = status.json()
    assert snapshot["found"] is True
    assert snapshot["task_id"] == task_id
    assert snapshot["current_state"] == "SUBMISSION_ACCEPTED_AWAITING_NATIVE_EVENT"
    assert snapshot["state_source"] == "business_ack_only"
    assert snapshot["execution_confirmed"] is False
    assert snapshot["terminal"] is False

    pause = client.post(
        f"/api/v1/inspection/tasks/{task_id}/pause",
        json={"reason": "operator_hold", "request_id": "inspection-pause-product-42"},
    )
    assert pause.status_code == 202
    assert pause.json()["lifecycle"] == "submission_accepted"
    assert pause.json()["task_id"] == task_id
    assert native_client.calls[-1] == (
        "task_pause",
        task_id,
        "operator_hold",
        "inspection-pause-product-42",
    )


def test_inspection_start_retry_cannot_change_the_started_route_snapshot(monkeypatch):
    client, native_client = _client(monkeypatch)
    route = _route()
    route["revision"] = 7
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    request = {
        "route_id": "route-a",
        "map_id": "field-map",
        "revision": 7,
        "request_id": "inspection-stable-start-retry",
    }
    first = client.post("/api/v1/inspection/tasks", json=request)
    assert first.status_code == 202
    calls_after_first_start = list(native_client.calls)

    edited = _route()
    edited["revision"] = 8
    assert client.post("/api/v1/inspection/routes", json=edited).status_code == 200
    retry = client.post(
        "/api/v1/inspection/tasks",
        json={**request, "revision": 8},
    )

    assert retry.status_code == 409
    assert retry.json()["error"] == "inspection_task_route_snapshot_mismatch"
    assert native_client.calls == calls_after_first_start


def test_inspection_control_request_id_cannot_be_redefined_before_native(
    monkeypatch,
):
    client, native_client = _client(monkeypatch)
    route = _route()
    route["revision"] = 7
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    started = client.post(
        "/api/v1/inspection/tasks",
        json={
            "route_id": "route-a",
            "map_id": "field-map",
            "revision": 7,
            "request_id": "inspection-control-start",
        },
    )
    task_id = started.json()["task_id"]
    first = client.post(
        f"/api/v1/inspection/tasks/{task_id}/pause",
        json={"reason": "operator_hold", "request_id": "control-once"},
    )
    assert first.status_code == 202
    calls_after_first = list(native_client.calls)

    changed_action = client.post(
        f"/api/v1/inspection/tasks/{task_id}/cancel",
        json={"reason": "operator_hold", "request_id": "control-once"},
    )
    changed_reason = client.post(
        f"/api/v1/inspection/tasks/{task_id}/pause",
        json={"reason": "different_reason", "request_id": "control-once"},
    )

    assert changed_action.status_code == 409
    assert changed_action.json()["error"] == "inspection_request_id_conflict"
    assert changed_reason.status_code == 409
    assert changed_reason.json()["error"] == "inspection_request_id_conflict"
    assert native_client.calls == calls_after_first


def test_inspection_task_collection_rehydrates_the_current_task_by_map(monkeypatch):
    """A console reload can rediscover a still-open task without guessing state."""

    client, _native_client = _client(monkeypatch)
    route = _route()
    route["revision"] = 7
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200

    start = client.post(
        "/api/v1/inspection/tasks",
        json={
            "route_id": "route-a",
            "map_id": "field-map",
            "revision": 7,
            "request_id": "inspection-rehydrate-product-42",
        },
    )
    assert start.status_code == 202
    task_id = start.json()["task_id"]

    listing = client.get(
        "/api/v1/inspection/tasks?map_id=field-map&include_terminal=false"
    )

    assert listing.status_code == 200
    payload = listing.json()
    assert payload["retention"] == "process_local_gateway_projection"
    assert payload["count"] == 1
    assert payload["tasks"][0]["task_id"] == task_id
    assert payload["tasks"][0]["current_state"] == (
        "SUBMISSION_ACCEPTED_AWAITING_NATIVE_EVENT"
    )
    assert payload["tasks"][0]["execution_confirmed"] is False


def test_inspection_task_start_fails_before_native_when_journal_is_corrupt(
    monkeypatch,
    tmp_path,
):
    journal = tmp_path / "inspection_tasks.json"
    journal.write_text('{"body":{},"sha256":"invalid"}\n', encoding="utf-8")
    monkeypatch.setenv("LINGTU_INSPECTION_TASK_JOURNAL", str(journal))
    client, native_client = _client(monkeypatch)
    route = _route()
    route["revision"] = 7
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200

    response = client.post(
        "/api/v1/inspection/tasks",
        json={
            "route_id": "route-a",
            "revision": 7,
            "request_id": "inspection-start-corrupt-journal",
        },
    )

    assert response.status_code == 503
    assert response.json()["error"] == "inspection_task_journal_unavailable"
    assert native_client.calls == []


def test_inspection_task_list_reports_corrupt_journal_instead_of_empty_history(
    monkeypatch,
    tmp_path,
):
    journal = tmp_path / "inspection_tasks.json"
    journal.write_text('{"body":{},"sha256":"invalid"}\n', encoding="utf-8")
    monkeypatch.setenv("LINGTU_INSPECTION_TASK_JOURNAL", str(journal))
    client, _native_client = _client(monkeypatch)

    response = client.get("/api/v1/inspection/tasks")

    assert response.status_code == 503
    assert response.json()["error"] == "inspection_task_journal_unavailable"


def test_retired_taskless_inspection_routes_are_not_registered(monkeypatch):
    """Only task-addressed inspection controls remain public."""

    client, native_client = _client(monkeypatch)
    retired_paths = (
        "/api/v1/inspection/routes/route-a/start",
        "/api/v1/inspection/run/pause",
        "/api/v1/inspection/run/resume",
        "/api/v1/inspection/run/cancel",
    )

    registered_paths = {route.path for route in client.app.routes}
    assert "/api/v1/inspection/routes/{route_id}/start" not in registered_paths
    assert not registered_paths.intersection(retired_paths[1:])

    for path in retired_paths:
        response = client.post(path, json={})
        assert response.status_code == 404
    assert native_client.calls == []


def test_inspection_native_store_unavailable_returns_503(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.routes.inspection as inspection
    class MissingService:
        def list_routes(self, map_id):
            del map_id
            raise RuntimeError("native inspection library not found; set LINGTU_INSPECTION_LIBRARY")

    app = FastAPI()
    service = MissingService()
    inspection.register_inspection_routes(app, SimpleNamespace(_inspection=service))
    response = TestClient(app).get("/api/v1/inspection/routes?map_id=field-map")

    assert response.status_code == 503
    assert response.json()["error"] == "inspection_native_unavailable"


def test_inspection_native_client_unavailable_returns_503(monkeypatch):
    client, _native_client = _client(monkeypatch)
    route = {
        "id": "route-a",
        "name": "Morning loop",
        "map_id": "field-map",
        "map_version": 3,
        "revision": 1,
        "loop_count": 1,
        "failure_policy": "stop",
        "max_retries": 0,
        "points": [
            {
                "id": "dock",
                "x": 1.0,
                "y": 2.0,
                "z": 0.0,
                "yaw": None,
                "tolerance": 0.35,
                "dwell": 0.0,
                "action": "",
                "enabled": True,
            }
        ],
    }
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200

    _native_client.error = "LINGTU_NAV_CLIENT_LIB is not configured"

    response = client.post(
        "/api/v1/inspection/tasks",
        json={"route_id": "route-a", "map_id": "field-map", "revision": 1},
    )

    assert response.status_code == 503
    assert response.json()["error"] == "inspection_native_unavailable"


def test_inspection_native_rejection_is_a_task_conflict_not_an_endpoint_outage(monkeypatch):
    client, native_client = _client(monkeypatch)
    route = _route()
    route["revision"] = 1
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    native_client.accepted = False

    response = client.post(
        "/api/v1/inspection/tasks",
        json={"route_id": "route-a", "map_id": "field-map", "revision": 1, "request_id": "rejected"},
    )

    assert response.status_code == 409
    payload = response.json()
    assert payload["error"] == "inspection_task_rejected"
    assert payload["detail"]["action"] == "start"
    assert payload["detail"]["request_id"] == "rejected"
    assert payload["detail"]["task_id"].startswith("inspection-task-")
    assert "rejected start_task" in payload["detail"]["native_reason"]


def test_inspection_task_control_rejection_is_a_task_conflict(monkeypatch):
    client, native_client = _client(monkeypatch)
    route = _route()
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    start = client.post(
        "/api/v1/inspection/tasks",
        json={"route_id": "route-a", "map_id": "field-map", "revision": 1},
    )
    assert start.status_code == 202
    task_id = start.json()["task_id"]
    native_client.accepted = False

    response = client.post(
        f"/api/v1/inspection/tasks/{task_id}/pause",
        json={"request_id": "pause-rejected"},
    )

    assert response.status_code == 409
    payload = response.json()
    assert payload["error"] == "inspection_task_rejected"
    assert payload["detail"] == {
        "action": "pause",
        "task_id": task_id,
        "request_id": "pause-rejected",
        "native_reason": "inspection service rejected pause_task",
    }


def test_inspection_malformed_ack_is_not_reported_as_accepted(monkeypatch):
    client, native_client = _client(monkeypatch)
    route = _route()
    route["revision"] = 1
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    native_client.accepted = "true"

    response = client.post(
        "/api/v1/inspection/tasks",
        json={"route_id": "route-a", "map_id": "field-map", "revision": 1, "request_id": "malformed"},
    )

    assert response.status_code == 503
    assert response.json()["error"] == "inspection_native_unavailable"
    assert "invalid acknowledgement" in response.json()["message"]


def test_inspection_start_rejects_revision_above_uint64(monkeypatch):
    client, native_client = _client(monkeypatch)

    response = client.post(
        "/api/v1/inspection/tasks",
        json={"route_id": "route-a", "map_id": "field-map", "revision": 1 << 64},
    )

    assert response.status_code == 422
    assert native_client.calls == []


def test_inspection_start_resolves_current_saved_revision(monkeypatch):
    client, native_client = _client(monkeypatch)
    route = {
        "id": "route-a",
        "name": "Morning loop",
        "map_id": "field-map",
        "map_version": 3,
        "revision": 1,
        "loop_count": 1,
        "failure_policy": "stop",
        "max_retries": 0,
        "points": [
            {
                "id": "dock",
                "x": 1.0,
                "y": 2.0,
                "z": 0.0,
                "yaw": None,
                "tolerance": 0.35,
                "dwell": 0.0,
                "action": "",
                "enabled": True,
            }
        ],
    }
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200

    response = client.post(
        "/api/v1/inspection/tasks",
        json={"route_id": "route-a", "map_id": "field-map", "revision": 0, "request_id": "req-current"},
    )

    assert response.status_code == 202
    assert response.json()["revision"] == 1
    assert native_client.calls[-1] == (
        "task_start",
        response.json()["task_id"],
        "route-a",
        1,
        "req-current",
    )


def test_inspection_start_rejects_stale_saved_revision_before_dds(monkeypatch):
    client, native_client = _client(monkeypatch)
    route = {
        "id": "route-a",
        "name": "Morning loop",
        "map_id": "field-map",
        "map_version": 3,
        "revision": 1,
        "loop_count": 1,
        "failure_policy": "stop",
        "max_retries": 0,
        "points": [
            {
                "id": "dock",
                "x": 1.0,
                "y": 2.0,
                "z": 0.0,
                "yaw": None,
                "tolerance": 0.35,
                "dwell": 0.0,
                "action": "",
                "enabled": True,
            }
        ],
    }
    assert client.post("/api/v1/inspection/routes", json=route).status_code == 200
    before = list(native_client.calls)

    response = client.post(
        "/api/v1/inspection/tasks",
        json={"route_id": "route-a", "map_id": "field-map", "revision": 2, "request_id": "req-stale"},
    )

    assert response.status_code == 409
    assert response.json()["error"] == "inspection_route_revision_mismatch"
    assert native_client.calls == before


def test_inspection_client_acquisition_and_ack_do_not_block_event_loop(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.inspection as inspection
    from gateway.schemas import InspectionTaskStartRequest

    entered = threading.Event()
    release = threading.Event()
    getter_threads: list[int] = []
    calls: list[tuple[str, str, int]] = []

    class BlockingInspection:
        def get_route(self, map_id: str, route_id: str):
            return _Store.routes[(map_id, route_id)]

        def start_task(
            self,
            task_id: str,
            route_id: str,
            *,
            revision: int = 0,
            request_id=None,
        ):
            del request_id
            getter_threads.append(threading.get_ident())
            entered.set()
            release.wait(timeout=1.0)
            calls.append((task_id, route_id, revision))
            return True

    _Store.routes = {
        ("field-map", "route-a"): {
            "id": "route-a",
            "map_id": "field-map",
            "revision": 7,
            "points": [],
        }
    }
    monkeypatch.setattr(inspection, "active_map_name", lambda _root: "field-map")

    app = FastAPI()
    service = BlockingInspection()
    inspection.register_inspection_routes(app, SimpleNamespace(_inspection=service))
    endpoint = next(
        route.endpoint
        for route in app.routes
        if route.path == "/api/v1/inspection/tasks" and "POST" in route.methods
    )

    async def run_request():
        loop_thread = threading.get_ident()
        started_at = time.perf_counter()
        request_task = asyncio.create_task(
            endpoint(
                InspectionTaskStartRequest(
                    route_id="route-a",
                    map_id="field-map",
                    revision=7,
                    request_id="nonblocking-inspection",
                ),
            )
        )
        pulse = asyncio.Event()
        asyncio.get_running_loop().call_later(0.02, pulse.set)
        await asyncio.wait_for(pulse.wait(), timeout=0.5)
        heartbeat_delay = time.perf_counter() - started_at
        release.set()
        response = await request_task
        return loop_thread, heartbeat_delay, response

    loop_thread, heartbeat_delay, response = asyncio.run(run_request())

    assert entered.is_set()
    assert heartbeat_delay < 0.5
    assert len(getter_threads) == 1
    assert getter_threads[0] != loop_thread
    assert response.action == "start"
    assert calls == [(response.task_id, "route-a", 7)]


def test_inspection_store_calls_do_not_block_event_loop(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.inspection as inspection

    entered = threading.Event()
    release = threading.Event()
    store_threads: list[int] = []

    class BlockingInspection:
        def list_routes(self, map_id: str):
            del map_id
            store_threads.append(threading.get_ident())
            entered.set()
            release.wait(timeout=0.5)
            return {"routes": []}

    app = FastAPI()
    inspection.register_inspection_routes(app, SimpleNamespace(_inspection=BlockingInspection()))
    endpoint = next(
        route.endpoint
        for route in app.routes
        if route.path == "/api/v1/inspection/routes"
        and "GET" in route.methods
    )

    async def run_request():
        loop_thread = threading.get_ident()
        started_at = time.perf_counter()
        request_task = asyncio.create_task(endpoint(map_id="field-map"))
        pulse = asyncio.Event()
        asyncio.get_running_loop().call_later(0.02, pulse.set)
        await asyncio.wait_for(pulse.wait(), timeout=0.5)
        heartbeat_delay = time.perf_counter() - started_at
        release.set()
        response = await request_task
        return loop_thread, heartbeat_delay, response

    loop_thread, heartbeat_delay, response = asyncio.run(run_request())

    assert entered.is_set()
    assert heartbeat_delay < 0.5
    assert len(store_threads) == 1
    assert store_threads[0] != loop_thread
    assert response["count"] == 0


def test_inspection_status_includes_unavailable_evidence_worker_without_500(monkeypatch, tmp_path):
    status_file = tmp_path / "missing.json"
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_STATUS_FILE", str(status_file))
    client, _native_client = _client(monkeypatch)

    response = client.get("/api/v1/inspection/status")

    assert response.status_code == 200
    worker = response.json()["status"]["evidence_worker"]
    assert worker["ready"] is False
    assert worker["reason"] == "status_file_missing"
    assert "path" not in worker


def test_inspection_status_accepts_fresh_ready_evidence_worker(monkeypatch, tmp_path):
    status_file = tmp_path / "inspection_evidence_status.json"
    status_file.write_text(
        json.dumps(
            {
                "ready": True,
                "heartbeat_ts": time.time(),
                "supported_actions": ["capture:overview", "capture:parking"],
                "worker_id": "worker-1",
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_STATUS_FILE", str(status_file))
    client, _native_client = _client(monkeypatch)

    response = client.get("/api/v1/inspection/status")

    assert response.status_code == 200
    worker = response.json()["status"]["evidence_worker"]
    assert worker["ready"] is True
    assert worker["reason"] == "ready"
    assert worker["supported_actions"] == ["capture:overview", "capture:parking"]
    assert worker["worker_id"] == "worker-1"


def test_inspection_status_preserves_sensor_readiness_reason(monkeypatch, tmp_path):
    status_file = tmp_path / "inspection_evidence_status.json"
    status_file.write_text(
        json.dumps(
            {
                "ready": False,
                "state": "ready",
                "readiness_reason": "rgb_missing",
                "heartbeat_ts": time.time(),
                "supported_actions": ["capture:overview"],
                "analyzers": {"capture:overview": "capture_only"},
                "evidence_root": str(tmp_path / "evidence"),
                "last_error": "",
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_STATUS_FILE", str(status_file))
    client, _native_client = _client(monkeypatch)

    worker = client.get("/api/v1/inspection/status").json()["status"]["evidence_worker"]

    assert worker["ready"] is False
    assert worker["reason"] == "rgb_missing"
    assert worker["readiness_reason"] == "rgb_missing"
    assert worker["state"] == "ready"
    assert worker["analyzers"] == {"capture:overview": "capture_only"}
    assert "evidence_root" not in worker
    assert "path" not in worker


def test_inspection_status_preserves_worker_readiness_reason_and_metadata(monkeypatch, tmp_path):
    status_file = tmp_path / "inspection_evidence_status.json"
    status_file.write_text(
        json.dumps(
            {
                "ready": False,
                "heartbeat_ts": time.time(),
                "readiness_reason": "rgb_missing",
                "state": "ready",
                "evidence_root": str(tmp_path / "evidence"),
                "supported_actions": ["capture:overview"],
                "analyzers": {"capture:overview": "capture_only"},
                "last_error": "rgb_missing",
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_STATUS_FILE", str(status_file))
    client, _native_client = _client(monkeypatch)

    response = client.get("/api/v1/inspection/status")

    assert response.status_code == 200
    worker = response.json()["status"]["evidence_worker"]
    assert worker["ready"] is False
    assert worker["reason"] == "rgb_missing"
    assert worker["readiness_reason"] == "rgb_missing"
    assert worker["state"] == "ready"
    assert "evidence_root" not in worker
    assert "path" not in worker
    assert worker["analyzers"] == {"capture:overview": "capture_only"}
    assert worker["last_error"] == "rgb_missing"


def test_inspection_start_without_actions_does_not_require_evidence_worker(monkeypatch, tmp_path):
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_STATUS_FILE", str(tmp_path / "missing.json"))
    client, native_client = _client(monkeypatch)
    assert client.post("/api/v1/inspection/routes", json=_route(action="")).status_code == 200

    response = client.post(
        "/api/v1/inspection/tasks",
        json={"route_id": "route-a", "map_id": "field-map", "revision": 1, "request_id": "req-no-action"},
    )

    assert response.status_code == 202
    assert native_client.calls[-1] == (
        "task_start",
        response.json()["task_id"],
        "route-a",
        1,
        "req-no-action",
    )


def test_inspection_start_with_action_requires_ready_evidence_worker(monkeypatch, tmp_path):
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_STATUS_FILE", str(tmp_path / "missing.json"))
    client, native_client = _client(monkeypatch)
    assert client.post("/api/v1/inspection/routes", json=_route(action="capture:overview")).status_code == 200

    response = client.post(
        "/api/v1/inspection/tasks",
        json={"route_id": "route-a", "map_id": "field-map", "revision": 1, "request_id": "req-action"},
    )

    assert response.status_code == 503
    assert response.json()["error"] == "inspection_evidence_worker_unavailable"
    assert native_client.calls == []


def test_inspection_start_rejects_unsupported_evidence_action(monkeypatch, tmp_path):
    status_file = tmp_path / "inspection_evidence_status.json"
    status_file.write_text(
        json.dumps(
            {
                "ready": True,
                "heartbeat_ts": time.time(),
                "supported_actions": ["capture:overview"],
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_STATUS_FILE", str(status_file))
    client, native_client = _client(monkeypatch)
    assert client.post("/api/v1/inspection/routes", json=_route(action="capture:parking")).status_code == 200

    response = client.post(
        "/api/v1/inspection/tasks",
        json={"route_id": "route-a", "map_id": "field-map", "revision": 1, "request_id": "req-unsupported"},
    )

    assert response.status_code == 503
    assert response.json()["error"] == "inspection_evidence_action_unsupported"
    assert response.json()["detail"]["unsupported_actions"] == ["capture:parking"]
    public_worker = response.json()["detail"]["evidence_worker"]
    assert "path" not in public_worker
    assert "evidence_root" not in public_worker
    assert native_client.calls == []


def test_inspection_evidence_list_detail_and_artifact_are_verified(monkeypatch, tmp_path):
    root = tmp_path / "evidence"
    _persist_evidence(root)
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_DIR", str(root))
    client, _native_client = _client(monkeypatch)

    listed = client.get("/api/v1/inspection/evidence?limit=10")
    assert listed.status_code == 200
    payload = listed.json()
    assert payload["count"] == 1
    assert payload["integrity_failures"] == 0
    item = payload["evidence"][0]
    assert item["evidence_id"] == "evidence-001"
    assert item["request"]["route_id"] == "route-a"
    assert item["request"]["point_id"] == "dock"
    assert item["analysis"]["verdict"] == "not_evaluated"
    assert {artifact["kind"] for artifact in item["artifacts"]} == {"rgb", "pose", "detections"}
    assert "evidence_dir" not in item
    assert "manifest_path" not in item
    assert all("path" not in artifact for artifact in item["artifacts"])

    detail = client.get("/api/v1/inspection/evidence/evidence-001")
    assert detail.status_code == 200
    assert detail.json()["evidence"]["manifest_sha256"] == item["manifest_sha256"]

    artifact = client.get("/api/v1/inspection/evidence/evidence-001/artifacts/rgb")
    assert artifact.status_code == 200
    assert artifact.headers["content-type"] == "image/jpeg"
    assert artifact.content == b"\xff\xd8inspection\xff\xd9"


def test_inspection_evidence_artifact_rejects_post_manifest_validation_replacement(
    monkeypatch,
    tmp_path,
):
    root = tmp_path / "evidence"
    _persist_evidence(root)
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_DIR", str(root))

    import gateway.routes.inspection as inspection
    from perception.inspection import InspectionEvidenceStore

    class ReplacingEvidenceStore(InspectionEvidenceStore):
        def get(self, request_id: str):
            result = super().get(request_id)
            (result.evidence_dir / "rgb.jpg").write_bytes(b"replaced-after-validation")
            return result

    monkeypatch.setattr(inspection, "InspectionEvidenceStore", ReplacingEvidenceStore)
    client, _native_client = _client(monkeypatch)

    response = client.get("/api/v1/inspection/evidence/evidence-001/artifacts/rgb")

    assert response.status_code == 409
    assert response.json()["error"] == "inspection_evidence_integrity_failed"


def test_inspection_evidence_artifact_reports_storage_io_failure_as_unavailable(
    monkeypatch,
    tmp_path,
):
    import errno

    root = tmp_path / "evidence"
    _persist_evidence(root)
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_DIR", str(root))

    import gateway.routes.inspection as inspection

    def fail_read(_path):
        raise OSError(errno.EIO, "simulated storage failure")

    monkeypatch.setattr(inspection, "_read_regular_file_bytes", fail_read)
    client, _native_client = _client(monkeypatch)

    response = client.get("/api/v1/inspection/evidence/evidence-001/artifacts/rgb")

    assert response.status_code == 503
    assert response.json()["error"] == "inspection_evidence_unavailable"


def test_inspection_evidence_api_rejects_invalid_ids_and_unknown_artifact_kind(monkeypatch, tmp_path):
    root = tmp_path / "evidence"
    _persist_evidence(root)
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_DIR", str(root))
    client, _native_client = _client(monkeypatch)

    invalid = client.get("/api/v1/inspection/evidence/bad$name")
    assert invalid.status_code == 404
    assert invalid.json()["error"] == "inspection_evidence_not_found"

    unknown = client.get("/api/v1/inspection/evidence/evidence-001/artifacts/raw")
    assert unknown.status_code == 400
    assert unknown.json()["error"] == "inspection_evidence_artifact_kind_invalid"


def test_inspection_evidence_api_reports_tamper_and_excludes_it_from_recent(monkeypatch, tmp_path):
    root = tmp_path / "evidence"
    result = _persist_evidence(root)
    (result.evidence_dir / "rgb.jpg").write_bytes(b"tampered")
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_DIR", str(root))
    client, _native_client = _client(monkeypatch)

    detail = client.get("/api/v1/inspection/evidence/evidence-001")
    assert detail.status_code == 409
    assert detail.json()["error"] == "inspection_evidence_integrity_failed"

    listed = client.get("/api/v1/inspection/evidence")
    assert listed.status_code == 200
    assert listed.json()["evidence"] == []
    assert listed.json()["integrity_failures"] == 1


def test_inspection_evidence_artifact_rejects_symlink(monkeypatch, tmp_path):
    root = tmp_path / "evidence"
    result = _persist_evidence(root)
    artifact = result.evidence_dir / "rgb.jpg"
    target = tmp_path / "same-content.jpg"
    target.write_bytes(artifact.read_bytes())
    artifact.unlink()
    try:
        os.symlink(target, artifact)
    except OSError as exc:
        pytest.skip(f"symlink creation unavailable: {exc}")
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_DIR", str(root))
    client, _native_client = _client(monkeypatch)

    response = client.get("/api/v1/inspection/evidence/evidence-001/artifacts/rgb")

    assert response.status_code == 409
    assert response.json()["error"] == "inspection_evidence_integrity_failed"


def test_inspection_evidence_store_unavailable_returns_503(monkeypatch):
    import gateway.routes.inspection as inspection

    client, _native_client = _client(monkeypatch)

    class BrokenEvidenceStore:
        def __init__(self, _root):
            raise OSError("storage offline")

    monkeypatch.setattr(inspection, "InspectionEvidenceStore", BrokenEvidenceStore)

    listed = client.get("/api/v1/inspection/evidence")
    detail = client.get("/api/v1/inspection/evidence/evidence-001")

    assert listed.status_code == 503
    assert listed.json()["error"] == "inspection_evidence_unavailable"
    assert detail.status_code == 503
    assert detail.json()["error"] == "inspection_evidence_unavailable"
