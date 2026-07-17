from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from pathlib import Path

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

    def start(self, route_id: str, *, revision: int = 0, request_id=None):
        self.calls.append(("start", route_id, revision, request_id))

    def pause(self, reason: str, *, request_id=None):
        self.calls.append(("pause", reason, request_id))

    def resume(self, reason: str, *, request_id=None):
        self.calls.append(("resume", reason, request_id))

    def cancel(self, reason: str, *, request_id=None):
        self.calls.append(("cancel", reason, request_id))


def _client(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.routes.inspection as inspection

    _Store.routes = {}
    _Store.deleted = []
    native_client = _Client()
    monkeypatch.setattr(inspection, "NativeInspectionStore", _Store)
    monkeypatch.setattr(
        inspection,
        "get_native_inspection_command_client",
        lambda *, required=False: native_client,
    )
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
    inspection.register_inspection_routes(app, object())
    return TestClient(app), native_client


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


def _persist_evidence(root: Path, *, request_id: str = "evidence-001", action: str = "capture:overview"):
    from perception.inspection import InspectionEvidenceRequest, InspectionEvidenceStore

    now = time.time()
    return InspectionEvidenceStore(root).persist(
        InspectionEvidenceRequest(
            run_id="run-001",
            route_id="route-a",
            route_revision=2,
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
        "/api/v1/inspection/routes/route-a/start",
        json={"map_id": "field-map", "revision": 7, "request_id": "req-1"},
    )
    pause = client.post(
        "/api/v1/inspection/run/pause",
        json={"map_id": "field-map", "reason": "hold", "request_id": "req-2"},
    )
    status = client.get("/api/v1/inspection/status")

    assert saved.status_code == 200
    assert saved.json()["route"]["points"][0]["tolerance"] == 0.4
    assert listed.status_code == 200
    assert listed.json()["count"] == 1
    assert detail.status_code == 200
    assert detail.json()["route"]["revision"] == 7
    assert start.status_code == 200
    assert pause.status_code == 200
    assert status.status_code == 200
    assert status.json()["status"]["current_point_index"] == 1
    assert native_client.calls == [
        ("start", "route-a", 7, "req-1"),
        ("pause", "hold", "req-2"),
    ]


def test_inspection_native_store_unavailable_returns_503(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.routes.inspection as inspection
    from nav.inspection.native import InspectionNativeError

    class MissingStore:
        def __init__(self, map_root):
            raise InspectionNativeError("native inspection library not found; set LINGTU_INSPECTION_LIBRARY")

    monkeypatch.setattr(inspection, "NativeInspectionStore", MissingStore)

    app = FastAPI()
    inspection.register_inspection_routes(app, object())
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

    import gateway.routes.inspection as inspection
    from runtime.adapters.native.inspection_commands import InspectionCommandClientError

    def unavailable(*, required=False):
        raise InspectionCommandClientError("LINGTU_NAV_CLIENT_LIB is not configured")

    monkeypatch.setattr(inspection, "get_native_inspection_command_client", unavailable)

    response = client.post(
        "/api/v1/inspection/routes/route-a/start",
        json={"map_id": "field-map", "revision": 1},
    )

    assert response.status_code == 503
    assert response.json()["error"] == "inspection_native_unavailable"


def test_inspection_start_rejects_revision_above_uint64(monkeypatch):
    client, native_client = _client(monkeypatch)

    response = client.post(
        "/api/v1/inspection/routes/route-a/start",
        json={"map_id": "field-map", "revision": 1 << 64},
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
        "/api/v1/inspection/routes/route-a/start",
        json={"map_id": "field-map", "revision": 0, "request_id": "req-current"},
    )

    assert response.status_code == 200
    assert response.json()["revision"] == 1
    assert native_client.calls[-1] == ("start", "route-a", 1, "req-current")


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
        "/api/v1/inspection/routes/route-a/start",
        json={"map_id": "field-map", "revision": 2, "request_id": "req-stale"},
    )

    assert response.status_code == 409
    assert response.json()["error"] == "inspection_route_revision_mismatch"
    assert native_client.calls == before


def test_inspection_client_acquisition_and_ack_do_not_block_event_loop(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.inspection as inspection
    from gateway.schemas import InspectionStartRequest

    entered = threading.Event()
    release = threading.Event()
    getter_threads: list[int] = []
    calls: list[tuple[str, int]] = []

    class Client:
        def start(self, route_id: str, *, revision: int = 0, request_id=None):
            calls.append((route_id, revision))

    def blocking_getter(*, required=False):
        getter_threads.append(threading.get_ident())
        entered.set()
        release.wait(timeout=1.0)
        return Client()

    _Store.routes = {
        ("field-map", "route-a"): {
            "id": "route-a",
            "map_id": "field-map",
            "revision": 7,
            "points": [],
        }
    }
    monkeypatch.setattr(inspection, "NativeInspectionStore", _Store)
    monkeypatch.setattr(inspection, "get_native_inspection_command_client", blocking_getter)
    monkeypatch.setattr(inspection, "active_map_name", lambda _root: "field-map")

    app = FastAPI()
    inspection.register_inspection_routes(app, object())
    endpoint = next(
        route.endpoint
        for route in app.routes
        if route.path == "/api/v1/inspection/routes/{route_id}/start"
    )

    async def run_request():
        loop_thread = threading.get_ident()
        started_at = time.perf_counter()
        request_task = asyncio.create_task(
            endpoint(
                "route-a",
                InspectionStartRequest(
                    map_id="field-map",
                    revision=7,
                    request_id="nonblocking-inspection",
                ),
            )
        )
        await asyncio.sleep(0.02)
        heartbeat_delay = time.perf_counter() - started_at
        release.set()
        response = await request_task
        return loop_thread, heartbeat_delay, response

    loop_thread, heartbeat_delay, response = asyncio.run(run_request())

    assert entered.is_set()
    assert heartbeat_delay < 0.15
    assert len(getter_threads) == 1
    assert getter_threads[0] != loop_thread
    assert response.action == "start"
    assert calls == [("route-a", 7)]


def test_inspection_store_calls_do_not_block_event_loop(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.inspection as inspection

    entered = threading.Event()
    release = threading.Event()
    store_threads: list[int] = []

    class BlockingStore:
        def __init__(self, map_root):
            self.map_root = map_root

        def __enter__(self):
            return self

        def __exit__(self, *_):
            return None

        def list(self, map_id: str):
            store_threads.append(threading.get_ident())
            entered.set()
            release.wait(timeout=0.5)
            return {"routes": []}

    monkeypatch.setattr(inspection, "NativeInspectionStore", BlockingStore)

    app = FastAPI()
    inspection.register_inspection_routes(app, object())
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
        await asyncio.sleep(0.02)
        heartbeat_delay = time.perf_counter() - started_at
        release.set()
        response = await request_task
        return loop_thread, heartbeat_delay, response

    loop_thread, heartbeat_delay, response = asyncio.run(run_request())

    assert entered.is_set()
    assert heartbeat_delay < 0.15
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
        "/api/v1/inspection/routes/route-a/start",
        json={"map_id": "field-map", "revision": 1, "request_id": "req-no-action"},
    )

    assert response.status_code == 200
    assert native_client.calls[-1] == ("start", "route-a", 1, "req-no-action")


def test_inspection_start_with_action_requires_ready_evidence_worker(monkeypatch, tmp_path):
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_STATUS_FILE", str(tmp_path / "missing.json"))
    client, native_client = _client(monkeypatch)
    assert client.post("/api/v1/inspection/routes", json=_route(action="capture:overview")).status_code == 200

    response = client.post(
        "/api/v1/inspection/routes/route-a/start",
        json={"map_id": "field-map", "revision": 1, "request_id": "req-action"},
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
        "/api/v1/inspection/routes/route-a/start",
        json={"map_id": "field-map", "revision": 1, "request_id": "req-unsupported"},
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
