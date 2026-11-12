"""Read-only saved-map previews stay separate from authoritative live points."""

from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.maps import routes


@pytest.fixture
def preview_api(monkeypatch):
    native = {
        "success": True,
        "map_id": "hall",
        "content_epoch": 12,
        "frame_id": "/map",
        "returned": 2,
        "point_count": 100,
        "points": [[1, 2, 3], [4, 5, 6]],
        "pcd": "/private/maps/hall/map.pcd",
    }
    calls = []

    def request(_gw, command):
        calls.append(command)
        return native

    monkeypatch.setattr(routes, "_mapd_http_request", request)
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace())
    with TestClient(app) as client:
        yield client, native, calls


def test_preview_works_without_active_map_and_has_fixed_sample_limit(preview_api, monkeypatch):
    client, _native, calls = preview_api
    monkeypatch.setattr(routes, "active_map", lambda *_: pytest.fail("preview accessed active map"))
    response = client.get("/api/v1/maps/hall/preview?max_points=1000000&content_epoch=12")
    assert response.status_code == 200
    assert response.json() == {
        "schema_version": 1,
        "name": "hall",
        "content_epoch": 12,
        "frame_id": "map",
        "count": 2,
        "total_points": 100,
        "points": [[1, 2, 3], [4, 5, 6]],
        "built_at_ms": None,
        "cleanup_state": "unknown",
    }
    assert calls == [{"action": "get_map_points", "map_id": "hall", "max_points": 24000}]


def test_preview_openapi_exposes_the_saved_map_contract(preview_api):
    client, _native, _calls = preview_api
    document = client.app.openapi()
    endpoint = document["paths"]["/api/v1/maps/{name}/preview"]["get"]
    assert endpoint["responses"]["200"]["content"]["application/json"]["schema"] == {
        "$ref": "#/components/schemas/MapPreviewResponse",
    }
    schema = document["components"]["schemas"]["MapPreviewResponse"]
    assert schema["properties"]["points"]["maxItems"] == 24000
    assert set(schema["properties"]) == {
        "schema_version", "name", "content_epoch", "frame_id", "count", "total_points",
        "points", "built_at_ms", "cleanup_state",
    }


@pytest.mark.parametrize("metadata", [{}, {"frame_id": ""}, {"frame_id": None}])
def test_legacy_map_preview_without_frame_preserves_unknown_metadata(preview_api, metadata):
    client, native, _calls = preview_api
    native.pop("frame_id")
    native.update(metadata)
    native.update(map_id="test6_dynamic", content_epoch=1794064895317, built_at_ms=None)
    response = client.get("/api/v1/maps/test6_dynamic/preview?content_epoch=1794064895317")
    assert response.status_code == 200
    assert response.json()["frame_id"] is None
    assert response.json()["built_at_ms"] is None
    assert response.json()["points"] == native["points"]
    assert response.json()["content_epoch"] == 1794064895317


@pytest.mark.parametrize("cleanup", ["cleaned", "not_cleaned", "unknown"])
def test_preview_preserves_explicit_metadata(preview_api, cleanup):
    client, native, _calls = preview_api
    native.update(built_at_ms=1794200000000, cleanup_state=cleanup)
    response = client.get("/api/v1/maps/hall/preview")
    assert response.status_code == 200
    assert response.json()["built_at_ms"] == 1794200000000
    assert response.json()["cleanup_state"] == cleanup


def test_preview_does_not_infer_cleanup_from_other_metadata(preview_api):
    client, native, _calls = preview_api
    native.update(cleanup_state="failed", cleaned_pcd="cleaned.pcd", state="READY")
    assert client.get("/api/v1/maps/hall/preview").json()["cleanup_state"] == "unknown"


def test_preview_rejects_stale_epoch(preview_api):
    client, _native, _calls = preview_api
    assert client.get("/api/v1/maps/hall/preview?content_epoch=11").status_code == 409


@pytest.mark.parametrize("epoch", ["0", "-1", "bad"])
def test_preview_validates_epoch_before_native_request(preview_api, epoch):
    client, _native, calls = preview_api
    assert client.get(f"/api/v1/maps/hall/preview?content_epoch={epoch}").status_code == 422
    assert calls == []


def test_preview_validates_map_name_before_native_request(preview_api):
    client, _native, calls = preview_api
    assert client.get("/api/v1/maps/bad%20name/preview").status_code == 400
    assert calls == []


@pytest.mark.parametrize("reason,status", [
    ("map_not_found", 404),
    ("map_pcd_not_found", 404),
    ("map_write_in_progress", 409),
    ("map_build_in_progress", 409),
    ("map_pcd_unreadable", 400),
])
def test_preview_native_failure_does_not_expose_paths(preview_api, reason, status):
    client, native, _calls = preview_api
    native.update(success=False, reason_code=reason, message="Cannot read /private/maps/hall/map.pcd")
    response = client.get("/api/v1/maps/hall/preview")
    assert response.status_code == status
    assert response.json()["detail"]["reason_code"] == reason
    assert "/private" not in response.text


@pytest.mark.parametrize("patch,status", [
    ({"map_id": "other"}, 409),
    ({"content_epoch": 0}, 502),
    ({"content_epoch": True}, 502),
    ({"returned": 3}, 502),
    ({"point_count": 1}, 502),
    ({"points": [[1, 2]]}, 502),
    ({"points": [[1, 2, "nan"], [4, 5, 6]]}, 502),
])
def test_preview_rejects_invalid_native_contract(preview_api, patch, status):
    client, native, _calls = preview_api
    native.update(patch)
    assert client.get("/api/v1/maps/hall/preview").status_code == status


def test_preview_transport_unavailable_is_503(monkeypatch):
    def unavailable(*_args):
        raise RuntimeError("mapd unavailable")

    monkeypatch.setattr(routes, "mapd_request", unavailable)
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace())
    with TestClient(app) as client:
        assert client.get("/api/v1/maps/hall/preview").status_code == 503


def test_live_points_still_require_authoritative_active_map(preview_api, monkeypatch):
    client, _native, calls = preview_api
    monkeypatch.setattr(routes, "active_map", lambda *_: "other")
    assert client.get("/api/v1/maps/hall/points").status_code == 409
    assert calls == []
