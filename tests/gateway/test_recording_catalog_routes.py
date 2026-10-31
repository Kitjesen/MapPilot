from __future__ import annotations

import io
import json
import tarfile
from pathlib import Path
from types import SimpleNamespace

from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes.recordings import register_recording_routes
from gateway.services.recording import NativeRecordingError


class _RecordingStub:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.deleted: list[str] = []
        self.series_calls: list[tuple[Path, dict]] = []
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

    def catalog(self):
        return {"topics": [{"topic": "/robot/joint_states", "label": "关节状态"}], "presets": []}

    def inspect_artifact(self, artifact: Path, *, validate: bool = False):
        assert artifact.name in {"sensors.mcap", "sensors.mcap.tmp"}
        if validate:
            return {"version": 1, "ok": True, "mode": "validate", "validated_messages": 12}
        return {"version": 1, "ok": True, "message_count": 12, "topics": []}

    def series_artifact(self, artifact: Path, **query):
        self.series_calls.append((artifact, query))
        return {
            "version": 1, "ok": True, "mode": "series", "topic": query["topic"],
            "field": query["field"] or "angular_velocity.x",
            "fields": [{"path": "angular_velocity.x", "unit": "rad/s"}],
            "start_time_ns": "1700000000000000000", "duration_s": 15.0,
            "window": {"start_s": query["start"], "end_s": query["end"] or 15.0},
            "samples": [{"t": 5.0, "value": 0.2}, {"t": 7.0, "value": 0.3, "gap_before": True}],
            "original_count": 100, "read_count": 20, "numeric_count": 20, "returned_count": 2,
            "downsampling": {"method": "minmax", "max_points": query["max_points"]},
            "frame_id": "imu",
        }

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


def test_recording_topic_catalog_uses_native_metadata(tmp_path: Path) -> None:
    client, _stub = _client(tmp_path)
    response = client.get("/api/v1/recordings/catalog")
    assert response.status_code == 200
    assert response.json()["topics"][0]["topic"] == "/robot/joint_states"


def test_recording_archive_includes_declared_camera_segments_only(tmp_path: Path) -> None:
    client, stub = _client(tmp_path)
    camera = tmp_path / "field_20260804/camera_color"
    camera.mkdir()
    (camera / "000000.mkv").write_bytes(b"video")
    (camera / "unlisted.txt").write_text("private", encoding="utf-8")
    stub.session["children"].append({"name": "camera", "artifacts": ["camera_color/000000.mkv"]})

    response = client.get("/api/v1/recordings/field_20260804/archive")

    assert response.status_code == 200
    assert response.headers["content-type"] == "application/x-tar"
    with tarfile.open(fileobj=io.BytesIO(response.content)) as archive:
        assert archive.getnames() == [
            "field_20260804/session.json",
            "field_20260804/dds/sensors.mcap",
            "field_20260804/camera_color/000000.mkv",
        ]
        assert archive.extractfile("field_20260804/camera_color/000000.mkv").read() == b"video"
        manifest = json.load(archive.extractfile("field_20260804/session.json"))
        assert "session_directory" not in manifest


def test_recording_summary_reports_selected_sources_without_messages(tmp_path: Path) -> None:
    client, _stub = _client(tmp_path)
    response = client.get("/api/v1/recordings/field_20260804/summary")
    assert response.status_code == 200
    assert response.json()["summary"]["missing_topics"] == ["/imu/raw"]


def test_recording_validate_uses_offline_player(tmp_path: Path) -> None:
    client, _stub = _client(tmp_path)
    response = client.post("/api/v1/recordings/field_20260804/validate")
    assert response.status_code == 200
    assert response.json()["validation"]["validated_messages"] == 12


def test_recording_active_session_cannot_export_or_validate_partial_files(tmp_path: Path) -> None:
    client, stub = _client(tmp_path)
    stub.session["state"] = "recording"
    assert client.get("/api/v1/recordings/field_20260804/archive").status_code == 409
    assert client.get("/api/v1/recordings/field_20260804/summary").status_code == 409
    assert client.post("/api/v1/recordings/field_20260804/validate").status_code == 409
    assert client.get("/api/v1/recordings/field_20260804/series", params={"topic": "/imu/raw"}).status_code == 409
    assert stub.series_calls == []


def test_failed_recording_exports_declared_retained_data_and_logs(tmp_path: Path) -> None:
    client, stub = _client(tmp_path)
    session = tmp_path / "field_20260804"
    (session / "dds/sensors.mcap").rename(session / "dds/sensors.mcap.tmp")
    (session / "logs").mkdir()
    (session / "logs/dds.stderr.log").write_bytes(b"required stream missing")
    stub.session["state"] = "failed"
    stub.session["error"] = "required stream missing"
    stub.session["children"][0]["artifacts"] = ["dds/sensors.mcap.tmp", "logs/dds.stderr.log"]

    response = client.get("/api/v1/recordings/field_20260804/archive")

    assert response.status_code == 200
    with tarfile.open(fileobj=io.BytesIO(response.content)) as archive:
        manifest = json.load(archive.extractfile("field_20260804/session.json"))
        assert manifest["state"] == "failed"
        assert archive.extractfile("field_20260804/dds/sensors.mcap.tmp").read() == b"mcap"
        assert archive.extractfile("field_20260804/logs/dds.stderr.log").read() == b"required stream missing"
    assert client.get("/api/v1/recordings/field_20260804/summary").status_code == 200
    assert client.post("/api/v1/recordings/field_20260804/validate").status_code == 200
    response = client.get("/api/v1/recordings/field_20260804/series", params={"topic": "/imu/raw"})
    assert response.status_code == 200
    assert response.json()["state"] == "failed"
    assert stub.series_calls[0][0] == session / "dds/sensors.mcap.tmp"


def test_recording_series_preserves_native_clock_fields_and_query(tmp_path: Path) -> None:
    client, stub = _client(tmp_path)
    response = client.get("/api/v1/recordings/field_20260804/series", params={
        "topic": "/imu/raw", "field": "angular_velocity.x", "start": 5, "end": 10, "max_points": 64,
    })
    assert response.status_code == 200
    payload = response.json()
    assert payload["session_id"] == "field_20260804"
    assert payload["state"] == "completed"
    series = payload["series"]
    assert series["start_time_ns"] == "1700000000000000000"
    assert series["samples"][0]["t"] == 5.0
    assert series["samples"][1]["gap_before"] is True
    assert series["downsampling"] == {"method": "minmax", "max_points": 64}
    assert stub.series_calls == [(tmp_path / "field_20260804/dds/sensors.mcap", {
        "topic": "/imu/raw", "field": "angular_velocity.x", "start": 5.0, "end": 10.0, "max_points": 64,
    })]


def test_recording_series_only_reads_manifest_declared_artifact(tmp_path: Path) -> None:
    client, stub = _client(tmp_path)
    stub.session["children"][0]["artifacts"] = []
    response = client.get("/api/v1/recordings/field_20260804/series", params={"topic": "/imu/raw"})
    assert response.status_code == 404
    assert response.json()["error"] == "recording_dds_artifact_missing"
    assert stub.series_calls == []


def test_recording_series_exposes_safe_query_error(tmp_path: Path) -> None:
    client, stub = _client(tmp_path)

    def unsupported(_artifact, **_query):
        raise NativeRecordingError(
            "recording_series_unsupported_field", "field is not numeric or does not exist", status_code=422,
        )

    stub.series_artifact = unsupported
    response = client.get("/api/v1/recordings/field_20260804/series", params={"topic": "/imu/raw", "field": "wrong"})
    assert response.status_code == 422
    assert response.json() == {
        "error": "recording_series_unsupported_field", "detail": "field is not numeric or does not exist",
    }
