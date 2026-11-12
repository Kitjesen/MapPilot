"""Focused contracts for the externally supported map API."""

from __future__ import annotations

import asyncio
import json
import threading
import time
from types import SimpleNamespace

import pytest

from gateway.schemas import MapReoptimizeRequest, MapSaveRequest


def _endpoint(app, path: str):
    return next(route.endpoint for route in app.routes if route.path == path)


def test_no_go_http_round_trip(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.maps import routes

    calls = []
    region = {"id": "door", "min": [1, 2, 0], "max": [2, 3, 2]}
    native = {"success": True, "map_id": "hall", "content_epoch": 11,
              "schema": "lingtu.no_go.v1", "frame_id": "map", "regions": [region]}

    def request(_gw, command):
        calls.append(command)
        return native

    monkeypatch.setattr(routes, "_mapd_http_request", request)
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace())
    with TestClient(app) as client:
        assert client.get("/api/v1/maps/hall/no_go").json() == native
        assert client.put("/api/v1/maps/hall/no_go", json={
            "expected_content_epoch": 11, "region": region}).status_code == 200
        assert client.delete("/api/v1/maps/hall/no_go/door?expected_content_epoch=12").status_code == 200
    assert calls == [
        {"action": "get_no_go_regions", "map_id": "hall"},
        {"action": "set_no_go_region", "map_id": "hall", "expected_content_epoch": 11,
         "region": {"id": "door", "min": (1, 2, 0), "max": (2, 3, 2)}},
        {"action": "delete_no_go_region", "map_id": "hall", "region_id": "door",
         "expected_content_epoch": 12},
    ]


@pytest.mark.parametrize("reason,status", [
    ("active_map_conflict", 409), ("source_epoch_mismatch", 409),
    ("invalid_no_go_region", 400), ("map_not_found", 404),
])
def test_no_go_rejection_is_not_success(monkeypatch, reason, status):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.maps import routes

    monkeypatch.setattr(routes, "_mapd_http_request", lambda *_: {
        "success": False, "reason_code": reason})
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace())
    with TestClient(app) as client:
        response = client.put("/api/v1/maps/hall/no_go", json={
            "expected_content_epoch": 11,
            "region": {"id": "door", "min": [2, 2, 0], "max": [1, 3, 2]}})
    assert response.status_code == status
    assert response.json() == {"success": False, "reason_code": reason}


@pytest.mark.parametrize("epoch,region", [
    (0, {"id": "door", "min": [1, 2, 0], "max": [2, 3, 2]}),
    (11, {"id": "bad/name", "min": [1, 2, 0], "max": [2, 3, 2]}),
    (11, {"id": "door", "min": [1, 2], "max": [2, 3, 2]}),
])
def test_no_go_shape_checked_at_http_boundary(monkeypatch, epoch, region):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.maps import routes

    monkeypatch.setattr(routes, "_mapd_http_request", lambda *_: pytest.fail("invalid shape reached mapd"))
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace())
    with TestClient(app) as client:
        response = client.put("/api/v1/maps/hall/no_go", json={
            "expected_content_epoch": epoch, "region": region})
    assert response.status_code == 422


def test_save_map_schema_rejects_public_optimizer_selection():
    with pytest.raises(ValueError):
        MapSaveRequest.model_validate({"name": "warehouse", "optimization": "auto"})


@pytest.mark.parametrize("accepted", [True, False])
def test_reoptimize_uses_archived_operation(monkeypatch, accepted):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as map_routes

    requests = []

    def submit(_gw, request):
        requests.append(request)
        return {
            "accepted": accepted,
            "reason_code": "" if accepted else "patch_bundle_incomplete",
            "status": {
                "job_id": "optimize_job_1", "source_job_id": "save_job_1",
                "state": "QUEUED" if accepted else "FAILED",
                "capture_dir": "/private/capture",
            },
        }

    monkeypatch.setattr(map_routes, "_mapd_http_request", submit)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())
    response = TestClient(app).post(
        "/api/v1/maps/operations/save_job_1/reoptimize", json={"request_id": "optimize_1"},
    )

    assert response.status_code == (202 if accepted else 409)
    assert requests == [{
        "action": "reoptimize_save_map", "job_id": "save_job_1", "request_id": "optimize_1",
    }]
    payload = response.json()
    assert payload["operation"]["operation_id"] == "optimize_job_1"
    assert payload["operation"]["source_operation_id"] == "save_job_1"
    assert payload["success"] is (None if accepted else False)
    if not accepted:
        assert payload["reason_code"] == "patch_bundle_incomplete"
    _assert_external_operation_payload_is_path_free(payload)


@pytest.mark.parametrize("body", [{}, {"request_id": ""}, {"request_id": "a/b"}, {"request_id": "x" * 129}])
def test_reoptimize_rejects_invalid_request(body):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as map_routes

    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())
    response = TestClient(app).post("/api/v1/maps/operations/save_1/reoptimize", json=body)
    assert response.status_code == 422


def test_rename_map_accepts_the_validated_http_request(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as map_routes

    commands = []

    def rename(_gw, command):
        commands.append(command)
        return {"success": True}

    monkeypatch.setattr(map_routes, "_mapd_http_request", rename)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    with TestClient(app) as client:
        response = client.post("/api/v1/map/rename", json={"old_name": "yard", "new_name": "yard2"})

    assert response.status_code == 200
    assert response.json()["success"] is True
    assert commands == [{"action": "rename_map", "map_id": "yard", "new_map_id": "yard2"}]


@pytest.mark.parametrize("auto_process", [True, False])
def test_save_map_accepts_the_validated_http_request(monkeypatch, auto_process):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as map_routes

    requests = []

    def submit(_gw, payload):
        requests.append(payload)
        return {
            "success": False,
            "accepted": True,
            "status": "running",
            "request_id": payload["request_id"],
            "job_id": "save_job_1",
        }

    monkeypatch.setattr(map_routes, "_mapd_http_request", submit)
    app = FastAPI()
    map_routes.register_map_routes(
        app,
        SimpleNamespace(_get_slam_profile=lambda: "native_dds"),
    )

    response = TestClient(app).post(
        "/api/v1/map/save",
        json={"name": "warehouse", "request_id": "request_1", "auto_process": auto_process},
    )

    assert response.status_code == 202
    assert response.json()["operation_id"] == "save_job_1"
    assert requests[0]["auto_process"] is auto_process
    assert MapSaveRequest().auto_process is True


def test_save_map_does_not_expose_optimizer_selection(monkeypatch):
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    captured: dict = {}

    def submit(_gw, payload):
        captured.update(payload)
        return {
            "success": False,
            "status": "running",
            "job_id": "save_job_1",
            "job": {"state": "RUNNING"},
        }

    monkeypatch.setattr(map_routes, "_mapd_http_request", submit)
    app = FastAPI()
    map_routes.register_map_routes(
        app,
        SimpleNamespace(_get_slam_profile=lambda: "native_dds"),
    )

    response = asyncio.run(
        _endpoint(app, "/api/v1/map/save")(MapSaveRequest(name="warehouse"))
    )

    assert response.status_code == 202
    assert "map_optimization" not in captured
    assert "optimization" not in captured


def _payload(response) -> dict:
    return json.loads(response.body)


_PRIVATE_OPERATION_KEYS = {
    "job_dir",
    "work_dir",
    "capture_dir",
    "source_report",
    "artifact_report",
    "map_dir",
    "version",
    "map_content_epoch",
    "manifest",
    "pcd",
    "occupancy",
    "octomap",
    "esdf",
    "traversability",
    "metadata",
    "record",
    "source_map_transaction",
    "source",
}


def _assert_external_operation_payload_is_path_free(payload: dict) -> None:
    def visit(value):
        if isinstance(value, dict):
            assert _PRIVATE_OPERATION_KEYS.isdisjoint(value)
            for child in value.values():
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)

    visit(payload)
    assert "/home/sunrise" not in json.dumps(payload)


def _private_operation_status() -> dict:
    return {
        "job_id": "save_job_1",
        "request_id": "save_job_1",
        "map_id": "warehouse",
        "state": "RUNNING",
        "phase": "BUILD_ARTIFACTS",
        "progress": 0.6,
        "source_saved": True,
        "message": "building /home/sunrise/data/lingtu/maps/warehouse/map.pcd",
        "created_at_ns": 10,
        "updated_at_ns": 20,
        "job_dir": "/home/sunrise/data/lingtu/maps/.save_jobs/save_job_1",
        "work_dir": "/home/sunrise/data/lingtu/maps/.save_jobs/save_job_1/work",
        "capture_dir": "/home/sunrise/data/lingtu/maps/.save_jobs/save_job_1/capture",
        "map_dir": "/home/sunrise/data/lingtu/maps/warehouse",
        "source_report": {
            "source": "/home/sunrise/private/raw.pcd",
            "record": {"path": "/home/sunrise/private/record.json"},
        },
        "artifact_report": {
            "pcd": "/home/sunrise/data/lingtu/maps/warehouse/map.pcd",
        },
    }


@pytest.mark.parametrize("performed,code", [(False, "sequential_chain_incomplete"), (True, "optimized")])
def test_save_operation_preserves_processing_outcome_without_native_reports(monkeypatch, performed, code):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as map_routes
    from gateway.schemas import MapSaveOperationResponse

    status = _private_operation_status()
    status.update(state="SUCCEEDED", phase="DONE")
    status["source_report"].update(
        optimization={"performed": performed, "code": code, "message": "/home/sunrise/private/graph"},
        dynamic_filter={"success": True, "performed": True, "reason_code": "cleaned", "path": "/home/sunrise/private/raw.pcd"},
    )
    monkeypatch.setattr(map_routes, "_mapd_http_request", lambda *_args: {"success": True, "status": status})
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    with TestClient(app) as client:
        response = client.get("/api/v1/maps/operations/save_job_1")

    assert response.status_code == 200
    payload = response.json()
    assert payload["operation"]["processing"] == {
        "optimization": {"performed": performed, "reason_code": code},
        "cleanup": {"success": True, "performed": True, "reason_code": "cleaned"},
    }
    assert MapSaveOperationResponse.model_validate(payload).operation.processing is not None
    _assert_external_operation_payload_is_path_free(payload)


def test_save_map_response_recursively_hides_native_paths(monkeypatch):
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    monkeypatch.setattr(
        map_routes,
        "_mapd_http_request",
        lambda _gw, _payload: {
            "success": False,
            "accepted": True,
            "status": "running",
            "reason_code": "save_job_in_progress",
            "message": "inspect /home/sunrise/data/lingtu/maps/.jobs/save_job_1",
            "job_id": "save_job_1",
            "job": _private_operation_status(),
            "map_dir": "/home/sunrise/data/lingtu/maps/warehouse",
        },
    )
    app = FastAPI()
    map_routes.register_map_routes(
        app,
        SimpleNamespace(_get_slam_profile=lambda: "native_dds"),
    )

    response = asyncio.run(
        _endpoint(app, "/api/v1/map/save")(MapSaveRequest(name="warehouse"))
    )
    payload = _payload(response)

    assert response.status_code == 202
    assert payload["operation"]["state"] == "RUNNING"
    assert payload["operation"]["progress"] == 0.6
    _assert_external_operation_payload_is_path_free(payload)


def test_save_map_running_job_returns_stable_accepted_response(monkeypatch):
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    monkeypatch.setattr(
        map_routes,
        "_mapd_http_request",
        lambda _gw, _payload: {
            "success": False,
            "status": "running",
            "reason_code": "save_job_timeout",
            "message": "SaveMap is still running; query by job_id",
            "job_id": "save_job_1",
            "job": {"state": "RUNNING"},
        },
    )
    gateway = SimpleNamespace(_get_slam_profile=lambda: "native_dds")
    app = FastAPI()
    map_routes.register_map_routes(app, gateway)

    response = asyncio.run(
        _endpoint(app, "/api/v1/map/save")(MapSaveRequest(name="warehouse"))
    )
    payload = _payload(response)

    assert response.status_code == 202
    assert payload["ok"] is True
    assert payload["success"] is None
    assert payload["accepted"] is True
    assert payload["status"] == "running"
    assert payload["reason_code"] == "map_save_in_progress"
    assert payload["operation_id"] == "save_job_1"


def test_save_map_exposes_operation_identity_not_worker_job(monkeypatch):
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    monkeypatch.setattr(
        map_routes,
        "_mapd_http_request",
        lambda _gw, _payload: {
            "success": False,
            "accepted": True,
            "status": "running",
            "request_id": "request_1",
            "job_id": "internal_job_1",
            "job": {
                "request_id": "request_1",
                "job_id": "internal_job_1",
                "state": "RUNNING",
            },
        },
    )
    app = FastAPI()
    map_routes.register_map_routes(
        app,
        SimpleNamespace(_get_slam_profile=lambda: "native_dds"),
    )

    response = asyncio.run(
        _endpoint(app, "/api/v1/map/save")(
            MapSaveRequest(name="warehouse", request_id="request_1")
        )
    )
    payload = _payload(response)

    assert response.status_code == 202
    assert payload["request_id"] == "request_1"
    assert payload["operation_id"] == "internal_job_1"
    assert payload["operation"]["operation_id"] == "internal_job_1"
    assert "job_id" not in payload
    assert "job" not in payload
    assert "job_id" not in payload["operation"]


def test_map_operation_routes_are_the_public_contract():
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    paths = app.openapi()["paths"]
    assert "/api/v1/maps/operations" in paths
    assert "/api/v1/maps/operations/{operation_id}" in paths
    assert "/api/v1/maps/operations/{operation_id}/cancel" in paths
    assert "/api/v1/maps/operations/{operation_id}/retry" in paths
    assert "/api/v1/maps/save-jobs" not in paths
    assert "/api/v1/maps/save-jobs/{job_id}" not in paths
    assert not any("save-jobs" in route.path for route in app.routes)


def test_slam_maps_collection_route_is_canonical_and_maps_alias_is_absent():
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    paths = {route.path for route in app.routes}
    assert "/api/v1/slam/maps" in paths
    assert "/api/v1/maps" not in paths


def test_map_operation_translates_native_job_reason_codes(monkeypatch):
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    monkeypatch.setattr(
        map_routes,
        "_mapd_http_request",
        lambda _gw, _request: {
            "success": False,
            "reason_code": "job_not_found",
            "job_id": "missing_operation",
        },
    )
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    response = asyncio.run(_endpoint(app, "/api/v1/maps/operations/{operation_id}")("missing_operation"))

    assert response.status_code == 404
    payload = _payload(response)
    assert payload["reason_code"] == "operation_not_found"
    assert payload["message"] == "Map-save operation was not found."


def test_public_map_list_hides_internal_entries_and_paths(monkeypatch):
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    monkeypatch.setattr(
        map_routes,
        "_mapd_http_request",
        lambda _gw, _payload: {
            "success": True,
            "active": "warehouse",
            "map_dir": "/home/sunrise/data/lingtu/maps",
            "maps": [
                    {
                        "name": ".codex_backups",
                        "has_pcd": True,
                        "can_activate": False,
                        "record": {"path": "/home/sunrise/private/backup.json"},
                    },
                {
                    "name": "warehouse",
                        "has_pcd": True,
                        "has_octomap": True,
                        "can_activate": True,
                    "state": "/home/sunrise/secret",
                    "record": {"path": "/home/sunrise/private/map_record.json"},
                    "artifacts": [{"path": "/home/sunrise/data/lingtu/maps/warehouse/map.pcd"}],
                },
            ],
        },
    )
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    payload = asyncio.run(_endpoint(app, "/api/v1/slam/maps")())

    assert payload["active"] == "warehouse"
    assert payload["count"] == 1
    assert [item["name"] for item in payload["maps"]] == ["warehouse"]
    assert payload["maps"][0]["state"] is None
    assert "map_dir" not in payload
    assert "record" not in payload["maps"][0]
    assert "artifacts" not in payload["maps"][0]
    assert "/home/sunrise" not in json.dumps(payload)


def test_delete_saved_map_uses_customer_lifecycle_contract(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as map_routes

    calls: list[dict] = []

    def fake_command(_gw, request):
        calls.append(dict(request))
        return {
            "action": "delete_map",
            "success": True,
            "map_id": "warehouse",
            "message": "deleted /home/sunrise/data/lingtu/maps/warehouse",
            "map_dir": "/home/sunrise/data/lingtu/maps/warehouse",
        }

    monkeypatch.setattr(map_routes, "_mapd_http_request", fake_command)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    response = TestClient(app).delete("/api/v1/maps/warehouse")
    payload = response.json()

    assert response.status_code == 200
    assert calls == [{"action": "delete_map", "map_id": "warehouse"}]
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["name"] == "warehouse"
    assert payload["message"] == "Map deleted."
    assert {"action", "map_id", "map_dir"}.isdisjoint(payload)
    assert "/home/sunrise" not in json.dumps(payload)


def test_active_map_mutations_preserve_native_conflict(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as map_routes

    monkeypatch.setattr(
        map_routes,
        "_mapd_http_request",
        lambda _gw, request: {
            "action": request["action"],
            "success": False,
            "reason_code": "active_map_conflict",
            "message": "active map conflict: warehouse",
        },
    )
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    with TestClient(app) as client:
        deleted = client.delete("/api/v1/maps/warehouse")
        renamed = client.post(
            "/api/v1/map/rename",
            json={"old_name": "warehouse", "new_name": "warehouse2"},
        )
        rebuilt = client.post("/api/v1/maps/warehouse/build_octomap")
        edited = client.post(
            "/api/v1/maps/warehouse/voxels/edit",
            json={"state": "occupied", "center": [0.0, 0.0, 0.0]},
        )

    for response in (deleted, renamed, rebuilt, edited):
        assert response.status_code == 409
        assert response.json()["success"] is False
        assert response.json()["reason_code"] == "active_map_conflict"


def test_build_saved_map_octomap_hides_native_paths(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as map_routes

    monkeypatch.setattr(
        map_routes,
        "_mapd_http_request",
        lambda _gw, request: {
            "action": request["action"],
            "success": True,
            "map_id": request["map_id"],
            "map_dir": "/var/lib/lingtu/maps/warehouse",
            "octomap": "/var/lib/lingtu/maps/warehouse/octomap.ot",
            "message": "wrote /var/lib/lingtu/maps/warehouse/octomap.ot",
        },
    )
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    response = TestClient(app).post("/api/v1/maps/warehouse/build_octomap")
    payload = response.json()

    assert response.status_code == 200
    assert payload["ok"] is True
    assert payload["name"] == "warehouse"
    assert payload["message"] == "OctoMap built."
    assert {"action", "map_id", "map_dir", "octomap"}.isdisjoint(payload)
    assert "/var/lib/lingtu" not in json.dumps(payload)


def test_build_saved_map_occupancy_uses_customer_lifecycle_contract(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as map_routes

    calls: list[dict] = []

    def fake_command(_gw, request):
        calls.append(dict(request))
        return {
            "action": "build_occupancy_snapshot",
            "success": True,
            "map_id": "warehouse",
            "occupancy": "/home/sunrise/data/lingtu/maps/warehouse/occupancy.npz",
            "message": "wrote /home/sunrise/data/lingtu/maps/warehouse/occupancy.npz",
        }

    monkeypatch.setattr(map_routes, "_mapd_http_request", fake_command)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    response = TestClient(app).post("/api/v1/maps/warehouse/build_occupancy")
    payload = response.json()

    assert response.status_code == 200
    assert calls == [{"action": "build_occupancy_snapshot", "map_id": "warehouse"}]
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["name"] == "warehouse"
    assert payload["occupancy_ok"] is True
    assert payload["message"] == "Occupancy map built."
    assert {"action", "map_id", "occupancy"}.isdisjoint(payload)
    assert "/home/sunrise" not in json.dumps(payload)


@pytest.mark.parametrize(
    ("method", "path"),
    [
        ("delete", "/api/v1/maps/operations"),
        ("post", "/api/v1/maps/operations/build_occupancy"),
    ],
)
def test_named_map_mutations_reject_reserved_operation_namespace(
    monkeypatch,
    method,
    path,
):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as map_routes

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("reserved map names must fail before maps.service")

    monkeypatch.setattr(map_routes, "_mapd_http_request", fail_if_called)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    response = getattr(TestClient(app), method)(path)

    assert response.status_code == 400
    assert response.json()["reason_code"] == "invalid_map_name"


def test_save_map_keeps_gateway_event_loop_responsive(monkeypatch):
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    order: list[str] = []

    def slow_save(_gw, _payload):
        time.sleep(0.05)
        order.append("save_finished")
        return {
            "success": False,
            "accepted": True,
            "status": "running",
            "job_id": "save_job_1",
        }

    monkeypatch.setattr(map_routes, "_mapd_http_request", slow_save)
    gateway = SimpleNamespace(_get_slam_profile=lambda: "native_dds")
    app = FastAPI()
    map_routes.register_map_routes(app, gateway)

    async def exercise():
        save_task = asyncio.create_task(
            _endpoint(app, "/api/v1/map/save")(MapSaveRequest(name="warehouse"))
        )
        await asyncio.sleep(0)
        order.append("gateway_responsive")
        return await save_task

    response = asyncio.run(exercise())

    assert response.status_code == 202
    assert order == ["gateway_responsive", "save_finished"]


@pytest.mark.parametrize(
    ("path", "action", "args"),
    [
        ("/api/v1/slam/maps", "list_maps", ()),
        ("/api/v1/maps/{name}", "delete_map", ("warehouse",)),
        (
            "/api/v1/maps/{name}/build_occupancy",
            "build_occupancy_snapshot",
            ("warehouse",),
        ),
        ("/api/v1/maps/operations", "list_save_map_jobs", ()),
        ("/api/v1/maps/operations/{operation_id}", "get_save_map_status", ("save_job_1",)),
        (
            "/api/v1/maps/operations/{operation_id}/cancel",
            "cancel_save_map",
            ("save_job_1",),
        ),
        (
            "/api/v1/maps/operations/{operation_id}/retry",
            "retry_save_map",
            ("save_job_1",),
        ),
        (
            "/api/v1/maps/operations/{operation_id}/process",
            "process_save_map",
            ("save_job_1",),
        ),
        (
            "/api/v1/maps/operations/{operation_id}/reoptimize",
            "reoptimize_save_map",
            ("save_job_1", MapReoptimizeRequest(request_id="optimize_1")),
        ),
    ],
)
def test_customer_map_queries_keep_gateway_event_loop_responsive(
    monkeypatch,
    path,
    action,
    args,
):
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    command_started = threading.Event()
    release_command = threading.Event()
    released_while_waiting: list[bool] = []

    def slow_command(_gw, request):
        assert request["action"] == action
        command_started.set()
        released_while_waiting.append(release_command.wait(timeout=0.5))
        if action == "list_maps":
            return {"success": True, "active": "", "maps": []}
        if action == "list_save_map_jobs":
            return {"success": True, "jobs": [], "count": 0}
        if action == "get_save_map_status":
            return {"success": True, "status": {"state": "RUNNING"}}
        return {"success": True, "accepted": True, "job_id": "save_job_1"}

    monkeypatch.setattr(map_routes, "_mapd_http_request", slow_command)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    async def exercise():
        endpoint_task = asyncio.create_task(_endpoint(app, path)(*args))
        try:
            assert await asyncio.wait_for(
                asyncio.to_thread(command_started.wait),
                timeout=1.0,
            )
            assert not endpoint_task.done()

            # This represents an unrelated lightweight request sharing the
            # Gateway event loop. It must run while the map service is blocked.
            release_command.set()
            return await asyncio.wait_for(endpoint_task, timeout=1.0)
        finally:
            release_command.set()
            if endpoint_task.done():
                endpoint_task.exception()

    asyncio.run(exercise())

    assert released_while_waiting == [True]


@pytest.mark.parametrize("operation_id", ["pcd", "points"])
def test_map_operation_detail_route_wins_over_named_map_artifact_routes(
    monkeypatch,
    operation_id,
):
    """The static operation namespace must not become a map named operations."""

    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as map_routes

    calls: list[dict] = []

    def fake_command(_gw, request):
        calls.append(dict(request))
        if request["action"] == "get_save_map_status":
            return {
                "success": True,
                "job_id": operation_id,
                "request_id": operation_id,
                "status": {
                    "job_id": operation_id,
                    "request_id": operation_id,
                    "state": "RUNNING",
                },
            }
        return {
            "success": True,
            "map_id": "operations",
            "frame_id": "map",
            "returned": 0,
            "points": [],
        }

    class CloudViewer:
        @staticmethod
        def scene_identity():
            return {
                "map_id": "operations",
                "protocol_version": 1,
                "epoch": 1,
                "sequence": 1,
                "stamp_s": 1.0,
            }

    gateway = SimpleNamespace(
        _cloud_viewer=CloudViewer(),
    )
    monkeypatch.setattr(map_routes, "_mapd_http_request", fake_command)
    app = FastAPI()
    map_routes.register_map_routes(app, gateway)

    response = TestClient(app, raise_server_exceptions=False).get(f"/api/v1/maps/operations/{operation_id}")

    assert response.status_code == 200
    assert calls == [{"action": "get_save_map_status", "job_id": operation_id}]
    assert response.json()["operation_id"] == operation_id


def test_map_save_rejects_reserved_operation_namespace(monkeypatch):
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("reserved map names must fail before reaching maps.service")

    monkeypatch.setattr(map_routes, "_mapd_http_request", fail_if_called)
    app = FastAPI()
    map_routes.register_map_routes(
        app,
        SimpleNamespace(_get_slam_profile=lambda: "native_dds"),
    )

    response = asyncio.run(
        _endpoint(app, "/api/v1/map/save")(MapSaveRequest(name="operations"))
    )

    assert response.status_code == 400
    assert _payload(response)["reason_code"] == "invalid_map_name"


def test_map_save_stops_before_map_service_when_explore_is_not_safely_parked(
    monkeypatch,
):
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("unsafe Explore save must not reach maps.service")

    monkeypatch.setattr(map_routes, "product_control_owns_explore", lambda _gw: True)
    monkeypatch.setattr(
        map_routes,
        "exploration_map_save_readiness",
        lambda _gw: {
            "can_save": False,
            "reason": "native_exploration_motion_not_idle",
            "message": "The native endpoint still owns motion.",
        },
    )
    monkeypatch.setattr(map_routes, "_mapd_http_request", fail_if_called)
    app = FastAPI()
    map_routes.register_map_routes(
        app,
        SimpleNamespace(_get_slam_profile=lambda: "native_dds"),
    )

    response = asyncio.run(
        _endpoint(app, "/api/v1/map/save")(MapSaveRequest(name="warehouse"))
    )

    assert response.status_code == 409
    assert _payload(response)["reason_code"] == "native_exploration_motion_not_idle"


def test_map_save_operation_schema_exposes_only_customer_fields():
    from gateway.schemas import MapSaveOperationResponse

    properties = MapSaveOperationResponse.model_json_schema()["properties"]

    assert {"accepted", "request_id", "operation_id", "status", "reason_code", "operation", "replayed"} <= set(
        properties
    )
    assert "job_id" not in properties
    assert {
        "path",
        "map_dir",
        "pcd",
        "octomap",
        "occupancy",
        "source",
    }.isdisjoint(properties)


@pytest.mark.parametrize(
    ("path", "action", "expected_status"),
    [
        ("/api/v1/maps/operations", "list_save_map_jobs", 200),
        ("/api/v1/maps/operations/{operation_id}", "get_save_map_status", 200),
        ("/api/v1/maps/operations/{operation_id}/cancel", "cancel_save_map", 200),
        ("/api/v1/maps/operations/{operation_id}/retry", "retry_save_map", 202),
    ],
)
def test_map_operation_routes_recursively_hide_native_paths(
    monkeypatch,
    path,
    action,
    expected_status,
):
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    status = _private_operation_status()

    def fake_command(_gw, request):
        assert request["action"] == action
        common = {
            "success": True,
            "accepted": True,
            "job_id": "save_job_1",
            "request_id": "save_job_1",
            "message": "read /home/sunrise/private/job.state",
            "map_dir": "/home/sunrise/data/lingtu/maps/warehouse",
        }
        if action == "list_save_map_jobs":
            return {**common, "jobs": [status], "count": 1}
        return {**common, "status": status}

    monkeypatch.setattr(map_routes, "_mapd_http_request", fake_command)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())
    endpoint = _endpoint(app, path)

    response = (
        asyncio.run(endpoint())
        if action == "list_save_map_jobs"
        else asyncio.run(endpoint("save_job_1"))
    )
    payload = _payload(response)

    assert response.status_code == expected_status
    assert payload["operation_id"] == "save_job_1"
    item = payload["operations"][0] if action == "list_save_map_jobs" else payload["operation"]
    assert item["state"] == "RUNNING"
    assert item["phase"] == "BUILD_ARTIFACTS"
    _assert_external_operation_payload_is_path_free(payload)


def test_public_map_list_hides_invalid_active_map(monkeypatch):
    from fastapi import FastAPI

    import gateway.maps.routes as map_routes

    monkeypatch.setattr(
        map_routes,
        "_mapd_http_request",
        lambda _gw, _payload: {
            "success": True,
            "active": ".codex_backups",
            "maps": [{"name": "warehouse", "has_pcd": True, "can_activate": True}],
        },
    )
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    payload = asyncio.run(_endpoint(app, "/api/v1/slam/maps")())

    assert payload["active"] == ""


@pytest.mark.parametrize(
    ("error_kind", "expected_reason"),
    [
        ("native", "map_artifact_unavailable"),
        ("os", "mapd_unavailable"),
    ],
)
def test_pcd_errors_never_echo_native_paths(monkeypatch, error_kind, expected_reason):
    from fastapi import FastAPI, HTTPException

    import gateway.maps.routes as map_routes
    from runtime.endpoints.mapd import MapClientError

    error = (
        MapClientError("unexpected_native_error", "failed at /run/lingtu-mapd/mapd.sock")
        if error_kind == "native"
        else OSError("cannot open C:\\lingtu\\private\\map.pcd")
    )

    def fail_open(*_args, **_kwargs):
        raise error

    monkeypatch.setattr(map_routes, "open_artifact", fail_open)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    with pytest.raises(HTTPException) as caught:
        asyncio.run(_endpoint(app, "/api/v1/maps/{name}/pcd")("warehouse"))

    detail = caught.value.detail
    assert detail["reason_code"] == expected_reason
    assert "sunrise" not in str(detail).lower()
    assert "mapd.sock" not in str(detail).lower()
    assert "c:\\lingtu" not in str(detail).lower()


def test_map_list_forwards_navigation_evidence(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as map_routes

    monkeypatch.setattr(
        map_routes,
        "_mapd_http_request",
        lambda _gw, _payload: {
            "success": True,
            "active": "",
            "maps": [
                {
                    "name": "hall",
                    "has_pcd": True,
                    "can_activate": False,
                    "activation_blockers": ["octomap.ot is missing", "saved rays are missing"],
                    "map_evidence": "preview",
                    "record": {"content_epoch": 7},
                },
                {"name": "old", "has_pcd": True, "can_activate": False},
            ],
        },
    )
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    response = TestClient(app).get("/api/v1/slam/maps")
    assert response.status_code == 200
    payload = response.json()

    assert [item["map_evidence"] for item in payload["maps"]] == ["preview", "none"]
    assert [item["content_epoch"] for item in payload["maps"]] == [7, None]
    assert [item["activation_blockers"] for item in payload["maps"]] == [
        ["octomap.ot is missing", "saved rays are missing"],
        [],
    ]


def test_save_edited_map_as_preserves_identity_and_build_provenance(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as routes

    calls = []
    build = {"frame_id": "floor_903", "resolution": 0.1, "source_profile": "ltlio"}
    def request(_gw, command):
        calls.append(command)
        return {"success": True, "map_id": "cleaned", "content_epoch": 22,
                "source_content_epoch": 11, "activation_ready": False,
                "build_options": build, "saved_rays_preserved": True, "replayed": True}
    monkeypatch.setattr(routes, "_mapd_http_request", request)
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace())
    operations = [{"operation": "delete_inside", "bounds": {
        "min_x": 0, "min_y": 0, "min_z": 0, "max_x": 1, "max_y": 1, "max_z": 1}}]
    with TestClient(app) as client:
        response = client.post("/api/v1/maps/original/save_as", json={
            "name": "cleaned", "expected_source_content_epoch": 11, "operations": operations})
        assert response.status_code == 200
        assert response.json()["content_epoch"] == 22
        assert response.json()["activation_ready"] is False
        assert response.json()["build_options"] == build
        assert response.json()["replayed"] is True
        rebuilt = client.post("/api/v1/maps/cleaned/build_octomap", json=build)
        assert rebuilt.status_code == 200
    assert calls == [
        {"action": "save_edited_source", "map_id": "original", "new_map_id": "cleaned",
         "expected_source_content_epoch": 11, "operations": operations},
        {"action": "build_octomap_artifact", "map_id": "cleaned", **build}]


def test_edit_preview_preserves_full_counts_sampled_points_and_order(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as routes

    calls = []
    native = {"success": True, "map_id": "original", "source_content_epoch": 11,
              "frame_id": "floor_903", "point_count": 40000, "removed_points": 300,
              "preview_points": [0, 1, 2], "preview_removed": [3, 4, 5], "preview_sample": True}
    def request(_gw, command):
        calls.append(command)
        return native
    monkeypatch.setattr(routes, "_mapd_http_request", request)
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace())
    operations = [{"operation": "radius", "radius": 0.2, "min_points": 12},
                  {"operation": "voxel", "voxel_size": 0.1}]
    with TestClient(app) as client:
        response = client.post("/api/v1/maps/original/edit_preview", json={
            "expected_source_content_epoch": 11, "operations": operations})
    assert response.status_code == 200
    payload = response.json()
    assert payload["ok"] is True
    for key in native.keys() - {"success"}:
        assert payload[key] == native[key]
    assert calls == [{"action": "preview_source_edits", "map_id": "original",
                      "expected_source_content_epoch": 11, "operations": operations}]


@pytest.mark.parametrize("reason,status", [("source_epoch_mismatch", 409), ("invalid_operations", 400),
                                          ("response_too_large", 400)])
def test_edit_preview_reports_native_rejection(monkeypatch, reason, status):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as routes

    monkeypatch.setattr(routes, "_mapd_http_request", lambda *_: {
        "success": False, "reason_code": reason, "message": "Edit parameters could not be applied."})
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace())
    with TestClient(app) as client:
        response = client.post("/api/v1/maps/original/edit_preview", json={
            "expected_source_content_epoch": 11, "operations": [{"operation": "radius", "radius": -1}]})
    assert response.status_code == status
    assert response.json()["ok"] is False
    assert response.json()["reason_code"] == reason
    assert "预览" in response.json()["message"] or "参数" in response.json()["message"]
    assert "save" not in response.json()["message"].lower()
    if reason == "response_too_large":
        assert "响应上限" in response.json()["message"]


@pytest.mark.parametrize("epoch,operations", [(True, []), (0, []), (11, {})])
def test_edit_preview_rejects_invalid_request_before_mapd(monkeypatch, epoch, operations):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as routes

    monkeypatch.setattr(routes, "_mapd_http_request", lambda *_: pytest.fail("invalid request reached mapd"))
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace())
    with TestClient(app) as client:
        response = client.post("/api/v1/maps/original/edit_preview", json={
            "expected_source_content_epoch": epoch, "operations": operations})
    assert response.status_code == 400


def test_resolution_copy_preserves_source_and_reports_native_build_resolution(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as routes

    calls = []
    def request(_gw, command):
        calls.append(command)
        if command["action"] == "save_edited_source":
            return {"success": True, "map_id": "coarse", "saved_rays_preserved": True,
                    "build_options": {"frame_id": "map", "resolution": .05}}
        return {"success": True, "octomap_result": {"report": {"resolution": .1, "frame_id": "map"}}}
    monkeypatch.setattr(routes, "_mapd_http_request", request)
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace())
    with TestClient(app) as client:
        copied = client.post("/api/v1/maps/original/save_as", json={"name": "coarse",
            "expected_source_content_epoch": 11, "operations": []})
        assert copied.status_code == 200
        assert copied.json()["saved_rays_preserved"] is True
        built = client.post("/api/v1/maps/coarse/build_octomap", json={"resolution": .1, "frame_id": "map"})
        assert built.status_code == 200
        assert built.json()["resolution"] == .1
    assert calls == [
        {"action": "save_edited_source", "map_id": "original", "new_map_id": "coarse",
         "expected_source_content_epoch": 11, "operations": []},
        {"action": "build_octomap_artifact", "map_id": "coarse", "resolution": .1, "frame_id": "map"}]


@pytest.mark.parametrize("reason", ["source_epoch_mismatch", "target_exists", "build_in_progress"])
def test_save_edited_map_conflicts_are_not_reported_as_saved(monkeypatch, reason):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.maps.routes as routes
    monkeypatch.setattr(routes, "_mapd_http_request",
                        lambda *_: {"success": False, "reason_code": reason})
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace())
    with TestClient(app) as client:
        result = client.post("/api/v1/maps/original/save_as", json={"name": "cleaned",
            "expected_source_content_epoch": 11, "operations": [{"operation": "delete_inside"}]})
    assert result.status_code == 409
    assert result.json()["ok"] is False
    assert result.json()["reason_code"] == reason


def test_surface_preview_forwards_geometry_options_and_native_layers(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.maps import routes

    calls = []
    native = {"success": True, "map_id": "hall", "content_epoch": 11, "frame_id": "map",
              "resolution": .1, "observed_count": 20, "inferred_count": 1,
              "observed_xyz": [0, 0, 0], "inferred_xyz": [1, 1, 2], "sampled": True}
    def request(_gw, command):
        calls.append(command)
        return native
    monkeypatch.setattr(routes, "_mapd_http_request", request)
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace())
    with TestClient(app) as client:
        response = client.get("/api/v1/maps/hall/surface?headroom_m=.5&radius_m=.3&max_hole_area_m2=.1&max_slope=.4")
    assert response.status_code == 200
    assert response.json() == native
    assert calls == [{"action": "get_surface_map", "map_id": "hall", "headroom_m": .5,
                      "radius_m": .3, "max_hole_area_m2": .1, "max_slope": .4}]
