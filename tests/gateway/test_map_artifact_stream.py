from __future__ import annotations

import os
from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")


def test_source_download_uses_operation_archive_not_current_map(tmp_path):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.maps.routes import register_map_routes
    from runtime.endpoints.mapd import ArtifactHandle

    path = tmp_path / "capture.pcd"
    path.write_bytes(b"archived-source-pcd")
    calls = []

    class Client:
        def open_save_source(self, job_id):
            calls.append(job_id)
            return ArtifactHandle(
                metadata={"job_id": job_id}, size_bytes=path.stat().st_size,
                filename="source.pcd", _descriptor=os.open(path, os.O_RDONLY),
            )

    app = FastAPI()
    register_map_routes(app, SimpleNamespace(_map_client=Client()))
    response = TestClient(app).get("/api/v1/maps/operations/save_1/source/pcd")
    assert response.status_code == 200
    assert response.content == b"archived-source-pcd"
    assert calls == ["save_1"]
    assert response.headers["x-lingtu-operation-id"] == "save_1"
    assert "x-lingtu-content-epoch" not in response.headers
    assert "capture" not in response.headers["content-disposition"]


def test_missing_source_does_not_fall_back_to_processed_map():
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.maps.routes import register_map_routes
    from runtime.endpoints.mapd import MapClientError

    class Client:
        def open_save_source(self, _job_id):
            raise MapClientError("saved_source_missing", "missing /private/archive/map.pcd")

    app = FastAPI()
    register_map_routes(app, SimpleNamespace(_map_client=Client()))
    response = TestClient(app).get("/api/v1/maps/operations/old_job/source/pcd")
    assert response.status_code == 404
    assert response.json()["detail"]["reason_code"] == "saved_source_missing"
    assert "/private/" not in response.text


@pytest.mark.parametrize("route,capability,filename", [
    ("pcd", "source_pointcloud", "map.pcd"),
    ("octomap", "global_planning_3d", "octomap.ot"),
])
def test_map_download_streams_mapd_descriptor_without_legacy_path_lookup(
    tmp_path, route, capability, filename,
) -> None:
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.maps.routes import register_map_routes
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
                metadata={
                    "success": True,
                    "map_id": map_id,
                    "content_epoch": 17,
                    "frame_id": "map",
                },
                size_bytes=len(payload),
                filename="map.pcd",
                _descriptor=descriptor,
            )

    map_client = Client()
    gateway = SimpleNamespace(_map_client=map_client)
    app = FastAPI()
    register_map_routes(app, gateway)

    response = TestClient(app).get(f"/api/v1/maps/yard/{route}")

    assert response.status_code == 200
    assert response.content == payload
    assert response.headers["content-length"] == str(len(payload))
    assert response.headers["content-disposition"] == f'attachment; filename="{filename}"'
    assert response.headers["cache-control"] == "no-store"
    assert response.headers["x-lingtu-map-id"] == "yard"
    assert response.headers["x-lingtu-content-epoch"] == "17"
    assert response.headers["x-lingtu-frame-id"] == "map"
    assert "etag" not in response.headers
    assert map_client.calls == [("yard", capability)]


def test_pcd_download_reports_native_missing_map_as_404() -> None:
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.maps.routes import register_map_routes
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

    from gateway.maps.routes import register_map_routes
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
