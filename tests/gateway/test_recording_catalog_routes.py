from __future__ import annotations

# ruff: noqa: S101 - pytest contract assertions intentionally use assert.
from pathlib import Path
from types import SimpleNamespace

from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes.recordings import register_recording_routes


class _RecordingStub:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.deleted: list[str] = []
        self.session = {
            "version": 1,
            "session_id": "field_20260804",
            "state": "completed",
            "session_directory": str(root / "field_20260804"),
            "manager_pid": 1234,
            "created_at_unix_ns": 1,
            "started_at_unix_ns": 2,
            "ended_at_unix_ns": 3,
            "context": {"product": "inspection"},
            "children": [
                {
                    "name": "dds",
                    "required": True,
                    "argv": ["/secret/native/path"],
                    "artifacts": ["dds/sensors.mcap"],
                    "selected_topics": ["/imu/raw"],
                }
            ],
        }

    def list(self, *, limit: int = 100):
        assert limit == 100
        return {
            "sessions": [
                {"session_id": "field_20260804", "state": "completed", "manager_pid": 1234}
            ],
            "truncated": False,
            "disk_free": 10,
            "disk_total": 20,
        }

    def manifest(self, *, session_id: str):
        assert session_id == self.session["session_id"]
        return self.session

    def remove(self, *, session_id: str):
        self.deleted.append(session_id)
        return {"session_id": session_id}

    @staticmethod
    def _validate_session_id(session_id: str) -> str:
        return session_id

    def _root(self) -> Path:
        return self.root


def _client(tmp_path: Path):
    app = FastAPI()
    stub = _RecordingStub(tmp_path)
    session_dir = tmp_path / "field_20260804" / "dds"
    session_dir.mkdir(parents=True)
    (session_dir / "sensors.mcap").write_bytes(b"mcap")
    register_recording_routes(app, SimpleNamespace(_recording=stub))
    return TestClient(app), stub


def test_recording_catalog_hides_host_paths_and_lists_artifacts(tmp_path: Path) -> None:
    client, _stub = _client(tmp_path)

    response = client.get("/api/v1/recordings")
    assert response.status_code == 200
    summary = response.json()["sessions"][0]
    assert summary["session_id"] == "field_20260804"
    assert "manager_pid" not in summary

    response = client.get("/api/v1/recordings/field_20260804")
    assert response.status_code == 200
    payload = response.json()["session"]
    assert payload["artifacts"] == [{"path": "dds/sensors.mcap", "download": "files/dds/sensors.mcap"}]
    assert "session_directory" not in payload
    assert payload["children"][0]["selected_topics"] == ["/imu/raw"]
    assert "argv" not in payload["children"][0]


def test_recording_artifact_download_is_manifest_declared(tmp_path: Path) -> None:
    client, _stub = _client(tmp_path)

    response = client.get("/api/v1/recordings/field_20260804/files/dds/sensors.mcap")
    assert response.status_code == 200
    assert response.content == b"mcap"
    assert response.headers["content-type"] == "application/octet-stream"

    missing = client.get("/api/v1/recordings/field_20260804/files/logs/secret.txt")
    assert missing.status_code == 404

    windows_escape = client.get(
        "/api/v1/recordings/field_20260804/files/dds%5C..%5Csecret.txt"
    )
    assert windows_escape.status_code == 422


def test_recording_delete_delegates_to_native_manager(tmp_path: Path) -> None:
    client, stub = _client(tmp_path)

    response = client.delete("/api/v1/recordings/field_20260804")
    assert response.status_code == 200
    assert response.json() == {"ok": True, "session_id": "field_20260804"}
    assert stub.deleted == ["field_20260804"]
