# ruff: noqa: S101
"""Focused contracts for the externally supported map API."""

from __future__ import annotations

import asyncio
import json
import threading
import time
from types import SimpleNamespace

import pytest


def _endpoint(app, path: str):
    return next(route.endpoint for route in app.routes if route.path == path)


def _payload(response) -> dict:
    return json.loads(response.body)


_PRIVATE_OPERATION_KEYS = {
    "capture_dir",
    "version_dir",
    "manifest_path",
    "source_report",
    "artifact_report",
    "map_dir",
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
        "version": 3,
        "state": "RUNNING",
        "phase": "BUILD_ARTIFACTS",
        "progress": 0.6,
        "message": "building /home/sunrise/data/lingtu/maps/warehouse/map.pcd",
        "created_at_ns": 10,
        "updated_at_ns": 20,
        "capture_dir": "/home/sunrise/data/lingtu/maps/.jobs/save_job_1/capture",
        "version_dir": "/home/sunrise/data/lingtu/maps/warehouse/versions/3",
        "manifest_path": "/home/sunrise/data/lingtu/maps/warehouse/versions/3/manifest.json",
        "source_report": {
            "source": "/home/sunrise/private/raw.pcd",
            "record": {"path": "/home/sunrise/private/record.json"},
        },
        "artifact_report": {
            "pcd": "/home/sunrise/data/lingtu/maps/warehouse/map.pcd",
        },
    }


def test_save_map_response_recursively_hides_native_paths(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    monkeypatch.setattr(
        map_routes,
        "_map_service_command",
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

    response = asyncio.run(_endpoint(app, "/api/v1/map/save")({"name": "warehouse"}))
    payload = _payload(response)

    assert response.status_code == 202
    assert payload["operation"]["state"] == "RUNNING"
    assert payload["operation"]["progress"] == 0.6
    _assert_external_operation_payload_is_path_free(payload)


def test_save_map_running_job_returns_stable_accepted_response(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    monkeypatch.setattr(
        map_routes,
        "_map_service_command",
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

    response = asyncio.run(_endpoint(app, "/api/v1/map/save")({"name": "warehouse"}))
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

    import gateway.routes.maps as map_routes

    monkeypatch.setattr(
        map_routes,
        "_map_service_command",
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

    response = asyncio.run(_endpoint(app, "/api/v1/map/save")({"name": "warehouse", "request_id": "request_1"}))
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

    import gateway.routes.maps as map_routes

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

    import gateway.routes.maps as map_routes

    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    paths = {route.path for route in app.routes}
    assert "/api/v1/slam/maps" in paths
    assert "/api/v1/maps" not in paths


def test_map_operation_translates_native_job_reason_codes(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    monkeypatch.setattr(
        map_routes,
        "_map_service_command",
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


def test_field_product_rejects_direct_map_activation(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    def fail_if_activated(*_args, **_kwargs):
        raise AssertionError("field map selection must remain owned by ProductControl")

    monkeypatch.setattr(map_routes, "activate_runtime_map", fail_if_activated)
    gateway = SimpleNamespace(
        _compiled_run_plan=SimpleNamespace(process_control="systemd"),
    )
    app = FastAPI()
    map_routes.register_map_routes(app, gateway)

    response = asyncio.run(_endpoint(app, "/api/v1/map/activate")({"name": "warehouse"}))
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["ok"] is False
    assert payload["success"] is False
    assert payload["reason_code"] == "product_map_switch_required"
    assert payload["requested_map"] == "warehouse"
    assert payload["switch_plan"] == "/api/v1/runtime/switch-plan"
    assert "operator_command" not in payload


def test_field_map_activation_returns_exact_product_control_command(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    def fail_if_activated(*_args, **_kwargs):
        raise AssertionError("field map selection must remain owned by ProductControl")

    monkeypatch.setattr(map_routes, "activate_runtime_map", fail_if_activated)
    gateway = SimpleNamespace(
        _compiled_run_plan=SimpleNamespace(
            process_control="systemd",
            product="map",
            env="real",
        ),
    )
    app = FastAPI()
    map_routes.register_map_routes(app, gateway)

    response = asyncio.run(_endpoint(app, "/api/v1/map/activate")({"name": "warehouse"}))
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["switch_plan"] == "/api/v1/runtime/switch-plan"
    assert payload["operator_command"] == (
        "python -m lingtu.control switch nav --env real "
        "--current map --map warehouse --relocalize"
    )


def test_public_map_list_hides_internal_entries_and_paths(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    monkeypatch.setattr(
        map_routes,
        "_map_service_command",
        lambda _gw, _payload: {
            "success": True,
            "active": "warehouse",
            "map_dir": "/home/sunrise/data/lingtu/maps",
            "maps": [
                {
                    "name": ".codex_backups",
                    "has_pcd": True,
                    "record": {"path": "/home/sunrise/private/backup.json"},
                },
                {
                    "name": "warehouse",
                    "has_pcd": True,
                    "has_octomap": True,
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

    import gateway.routes.maps as map_routes

    calls: list[dict] = []

    def fake_command(_gw, request):
        calls.append(dict(request))
        return {
            "action": "delete",
            "success": True,
            "map_id": "warehouse",
            "message": "deleted /home/sunrise/data/lingtu/maps/warehouse",
            "map_dir": "/home/sunrise/data/lingtu/maps/warehouse",
        }

    monkeypatch.setattr(map_routes, "_map_service_command", fake_command)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    response = TestClient(app).delete("/api/v1/maps/warehouse")
    payload = response.json()

    assert response.status_code == 200
    assert calls == [{"action": "delete", "name": "warehouse"}]
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["name"] == "warehouse"
    assert payload["message"] == "Map deleted."
    assert {"action", "map_id", "map_dir"}.isdisjoint(payload)
    assert "/home/sunrise" not in json.dumps(payload)


def test_build_saved_map_occupancy_uses_customer_lifecycle_contract(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    import gateway.routes.maps as map_routes

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

    monkeypatch.setattr(map_routes, "_map_service_command", fake_command)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    response = TestClient(app).post("/api/v1/maps/warehouse/build_occupancy")
    payload = response.json()

    assert response.status_code == 200
    assert calls == [{"action": "build_occupancy", "name": "warehouse"}]
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

    import gateway.routes.maps as map_routes

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("reserved map names must fail before maps.service")

    monkeypatch.setattr(map_routes, "_map_service_command", fail_if_called)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    response = getattr(TestClient(app), method)(path)

    assert response.status_code == 400
    assert response.json()["reason_code"] == "invalid_map_name"


def test_save_map_keeps_gateway_event_loop_responsive(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

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

    monkeypatch.setattr(map_routes, "_map_service_command", slow_save)
    gateway = SimpleNamespace(_get_slam_profile=lambda: "native_dds")
    app = FastAPI()
    map_routes.register_map_routes(app, gateway)

    async def exercise():
        save_task = asyncio.create_task(_endpoint(app, "/api/v1/map/save")({"name": "warehouse"}))
        await asyncio.sleep(0)
        order.append("gateway_responsive")
        return await save_task

    response = asyncio.run(exercise())

    assert response.status_code == 202
    assert order == ["gateway_responsive", "save_finished"]


@pytest.mark.parametrize(
    ("path", "action", "args"),
    [
        ("/api/v1/slam/maps", "list", ()),
        ("/api/v1/maps/{name}", "delete", ("warehouse",)),
        (
            "/api/v1/maps/{name}/build_occupancy",
            "build_occupancy",
            ("warehouse",),
        ),
        ("/api/v1/maps/operations", "list_save_jobs", ()),
        ("/api/v1/maps/operations/{operation_id}", "save_status", ("save_job_1",)),
        (
            "/api/v1/maps/operations/{operation_id}/cancel",
            "cancel_save",
            ("save_job_1",),
        ),
        (
            "/api/v1/maps/operations/{operation_id}/retry",
            "retry_save",
            ("save_job_1",),
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

    import gateway.routes.maps as map_routes

    command_started = threading.Event()
    release_command = threading.Event()
    released_while_waiting: list[bool] = []

    def slow_command(_gw, request):
        assert request["action"] == action
        command_started.set()
        released_while_waiting.append(release_command.wait(timeout=0.5))
        if action == "list":
            return {"success": True, "active": "", "maps": []}
        if action == "list_save_jobs":
            return {"success": True, "jobs": [], "count": 0}
        if action == "save_status":
            return {"success": True, "status": {"state": "RUNNING"}}
        return {"success": True, "accepted": True, "job_id": "save_job_1"}

    monkeypatch.setattr(map_routes, "_map_service_command", slow_command)
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

    import gateway.routes.maps as map_routes

    calls: list[dict] = []

    def fake_command(_gw, request):
        calls.append(dict(request))
        if request["action"] == "save_status":
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
        _active_map_from_maps_service=lambda: "operations",
    )
    monkeypatch.setattr(map_routes, "_map_service_command", fake_command)
    app = FastAPI()
    map_routes.register_map_routes(app, gateway)

    response = TestClient(app, raise_server_exceptions=False).get(f"/api/v1/maps/operations/{operation_id}")

    assert response.status_code == 200
    assert calls == [{"action": "save_status", "job_id": operation_id}]
    assert response.json()["operation_id"] == operation_id


def test_map_save_rejects_reserved_operation_namespace(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("reserved map names must fail before reaching maps.service")

    monkeypatch.setattr(map_routes, "_map_service_command", fail_if_called)
    app = FastAPI()
    map_routes.register_map_routes(
        app,
        SimpleNamespace(_get_slam_profile=lambda: "native_dds"),
    )

    response = asyncio.run(_endpoint(app, "/api/v1/map/save")({"name": "operations"}))

    assert response.status_code == 400
    assert _payload(response)["reason_code"] == "invalid_map_name"


def test_map_save_stops_before_map_service_when_explore_is_not_safely_parked(
    monkeypatch,
):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

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
    monkeypatch.setattr(map_routes, "_map_service_command", fail_if_called)
    app = FastAPI()
    map_routes.register_map_routes(
        app,
        SimpleNamespace(_get_slam_profile=lambda: "native_dds"),
    )

    response = asyncio.run(_endpoint(app, "/api/v1/map/save")({"name": "warehouse"}))

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
        ("/api/v1/maps/operations", "list_save_jobs", 200),
        ("/api/v1/maps/operations/{operation_id}", "save_status", 200),
        ("/api/v1/maps/operations/{operation_id}/cancel", "cancel_save", 200),
        ("/api/v1/maps/operations/{operation_id}/retry", "retry_save", 202),
    ],
)
def test_map_operation_routes_recursively_hide_native_paths(
    monkeypatch,
    path,
    action,
    expected_status,
):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

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
        if action == "list_save_jobs":
            return {**common, "jobs": [status], "count": 1}
        return {**common, "status": status}

    monkeypatch.setattr(map_routes, "_map_service_command", fake_command)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())
    endpoint = _endpoint(app, path)

    response = asyncio.run(endpoint()) if action == "list_save_jobs" else asyncio.run(endpoint("save_job_1"))
    payload = _payload(response)

    assert response.status_code == expected_status
    assert payload["operation_id"] == "save_job_1"
    item = payload["operations"][0] if action == "list_save_jobs" else payload["operation"]
    assert item["state"] == "RUNNING"
    assert item["phase"] == "BUILD_ARTIFACTS"
    _assert_external_operation_payload_is_path_free(payload)


def test_public_map_list_hides_invalid_active_map(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    monkeypatch.setattr(
        map_routes,
        "_map_service_command",
        lambda _gw, _payload: {
            "success": True,
            "active": ".codex_backups",
            "maps": [{"name": "warehouse", "has_pcd": True}],
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

    import gateway.routes.maps as map_routes
    from maps.client import MapClientError

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
