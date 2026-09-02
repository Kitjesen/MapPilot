from __future__ import annotations

import os
from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")


def test_pcd_download_streams_mapd_descriptor_without_legacy_path_lookup(tmp_path) -> None:
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.routes.maps import register_map_routes
    from runtime.endpoints.mapd import ArtifactHandle

    payload = b"VERSION .7\nDATA binary\nmap-payload"
    path = tmp_path / "map.pcd"
    path.write_bytes(payload)

    class Client:
        def __init__(self) -> None:
            self.calls = []

        def open_artifact(self, map_id: str, capability: str) -> ArtifactHandle:
            self.calls.append((map_id, capability))
            descriptor = os.open(path, os.O_RDONLY)
            return ArtifactHandle(
                metadata={"success": True, "map_id": map_id},
                size_bytes=len(payload),
                filename="map.pcd",
                _descriptor=descriptor,
            )

    map_client = Client()
    gateway = SimpleNamespace(_map_client=map_client)
    app = FastAPI()
    register_map_routes(app, gateway)

    response = TestClient(app).get("/api/v1/maps/yard/pcd")

    assert response.status_code == 200
    assert response.content == payload
    assert response.headers["content-length"] == str(len(payload))
    assert response.headers["content-disposition"] == 'attachment; filename="map.pcd"'
    assert "etag" not in response.headers
    assert map_client.calls == [("yard", "source_pointcloud")]


def test_pcd_download_reports_native_missing_map_as_404() -> None:
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.routes.maps import register_map_routes
    from runtime.endpoints.mapd import MapClientError

    class Client:
        def open_artifact(self, _map_id: str, _capability: str):
            raise MapClientError("map_not_found", "map does not exist")

    app = FastAPI()
    register_map_routes(app, SimpleNamespace(_map_client=Client()))

    response = TestClient(app).get("/api/v1/maps/missing/pcd")

    assert response.status_code == 404
    assert response.json()["detail"] == {
        "reason_code": "map_not_found",
        "message": "Map artifact was not found.",
    }


def test_pcd_download_reports_map_write_in_progress_as_conflict() -> None:
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.routes.maps import register_map_routes
    from runtime.endpoints.mapd import MapClientError

    class Client:
        def open_artifact(self, _map_id: str, _capability: str):
            raise MapClientError(
                "map_write_in_progress",
                "map artifact is being updated",
            )

    app = FastAPI()
    register_map_routes(app, SimpleNamespace(_map_client=Client()))

    response = TestClient(app).get("/api/v1/maps/yard/pcd")

    assert response.status_code == 409
    assert response.json()["detail"] == {
        "reason_code": "map_write_in_progress",
        "message": "Map artifact is being updated.",
    }
