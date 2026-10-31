from __future__ import annotations

import asyncio
import json
import os
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from starlette.responses import JSONResponse

from gateway.routes.recordings import register_recording_routes
from gateway.services.recording import NativeRecordingService
from gateway.services.recording_media import recording_camera_frame, recording_scene_info

ORIGIN = 1792902000000000000


def _decoded_frames(arguments, indices):
    return subprocess.CompletedProcess(
        arguments, 0,
        stdout=b"".join(b"\xff\xd8jpeg" + str(index).encode() + b"\xff\xd9" for index in indices),
        stderr="\n".join(f"pts_time:{index}" for index in indices).encode(),
    )


class _Recording:
    def __init__(self, root: Path, *, dds: bool = True, exact: bool = True):
        self.root = root
        self._environ = {}
        self.scene_calls = []
        self.camera_calls = []
        self.session = {
            "session_id": "media", "state": "completed",
            "children": [{"name": "camera_color", "artifacts": ["camera_color.mcap", "camera_color/segment_00000.mkv"]}],
        }
        if dds:
            self.session["children"].append({"name": "dds", "artifacts": ["dds/sensors.mcap"]})
        self.segments = [{
            "index": 0, "camera_id": "color", "relative_path": "camera_color/segment_00000.mkv",
            "codec": "h264", "width": 640, "height": 480,
            "start_time_ns": str(ORIGIN + 2_000_000_000), "end_time_ns": str(ORIGIN + 4_000_000_000),
            "frame_count": 3, "dropped_frames": 0, "frame_id": "camera_optical", "nominal_fps": 1.0,
            "frame_timestamps_ns": [str(ORIGIN + value) for value in [2_000_000_000, 2_800_000_000, 4_000_000_000]] if exact else [],
            "timing": "frame_timestamp_index" if exact else "segment_cfr_estimate",
        }]
        for child in self.session["children"]:
            for artifact in child["artifacts"]:
                path = root / "media" / artifact
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"fixture")

    def manifest(self, *, session_id):
        assert session_id == "media"
        return self.session

    def _root(self):
        return self.root

    def _validate_session_id(self, session_id):
        return session_id

    def scene_artifact(self, artifact, **query):
        self.scene_calls.append((artifact, query))
        return {
            "version": 1, "ok": True, "mode": "scene-info" if query.get("info") else "scene",
            "start_time_ns": str(ORIGIN), "duration_s": 10.0,
            "topics": [{"topic": "/slam/registered_cloud", "kind": "cloud", "message_count": 12}],
        }

    def camera_index_artifact(self, artifact, *, segment=None):
        self.camera_calls.append(artifact)
        rows = [self.segments[segment]] if segment is not None else [
            {key: value for key, value in item.items() if key not in {"frame_timestamps_ns", "width", "height", "frame_id", "codec"}}
            for item in self.segments
        ]
        return {"version": 1, "ok": True, "mode": "camera-info", "camera_id": "color", "segments": rows}


def _client(service):
    app = FastAPI()
    register_recording_routes(app, SimpleNamespace(_recording=service))
    return TestClient(app)


def test_scene_info_uses_dds_origin_and_preserves_camera_frame_clock(tmp_path):
    service = _Recording(tmp_path)
    info = recording_scene_info(service, "media")
    row = info["camera"]["segments"][0]
    assert info["start_time_ns"] == str(ORIGIN)
    assert info["duration_s"] == 10.0
    assert (row["start_s"], row["end_s"]) == (2.0, 4.0)
    assert "frame_timestamps_ns" not in row
    assert info["camera"]["exact"] is True
    recording_scene_info(service, "media")
    assert len(service.camera_calls) == len(service.scene_calls) == 1


def test_video_only_recording_has_camera_origin_without_dds(tmp_path):
    service = _Recording(tmp_path, dds=False)
    client = _client(service)
    info = client.get("/api/v1/recordings/media/scene-info").json()
    assert info["scene"] is None
    assert info["start_time_ns"] == str(ORIGIN + 2_000_000_000)
    assert info["duration_s"] == 3.0
    assert info["camera"]["segments"][0]["start_s"] == 0.0
    assert client.get("/api/v1/recordings/media/scene").json()["scene"] is None
    assert service.scene_calls == []


def test_camera_coverage_can_start_before_dds_and_extend_after_it(tmp_path):
    service = _Recording(tmp_path)
    service.segments[0].update(start_time_ns=str(ORIGIN - 1_000_000_000), end_time_ns=str(ORIGIN + 12_000_000_000))
    info = recording_scene_info(service, "media")
    assert info["camera"]["segments"][0]["start_s"] == -1.0
    assert info["duration_s"] == 13.0


def test_camera_only_declared_segments_can_be_selected(tmp_path):
    service = _Recording(tmp_path)
    service.session["children"][0]["artifacts"].remove("camera_color/segment_00000.mkv")
    client = _client(service)
    assert client.get("/api/v1/recordings/media/scene-info").json()["camera"]["available"] is False
    assert client.get("/api/v1/recordings/media/camera-frame", params={"t": 3, "segment": 0}).status_code == 404


@pytest.mark.parametrize("exact,requested,ordinal,selected,timing", [
    (True, 2.6, 0, 2.0, "frame_timestamp_index"),
    (False, 3.6, 1, 3.0, "segment_cfr_estimate"),
])
def test_camera_decode_selects_recorded_ordinal_and_reports_source_time(tmp_path, monkeypatch, exact, requested, ordinal, selected, timing):
    service = _Recording(tmp_path, exact=exact)
    calls = []
    monkeypatch.setattr("gateway.services.recording_media.shutil.which", lambda _name: "ffmpeg")

    def run(arguments, **kwargs):
        calls.append((arguments, kwargs))
        return _decoded_frames(arguments, range(3))

    monkeypatch.setattr("gateway.services.recording_media.subprocess.run", run)
    response = _client(service).get("/api/v1/recordings/media/camera-frame", params={"t": requested, "segment": 0})
    assert response.status_code == 200
    assert response.headers["content-type"] == "image/jpeg"
    assert float(response.headers["x-recording-time-s"]) == selected
    assert response.headers["x-recording-frame-index"] == str(ordinal)
    assert response.headers["x-recording-timing"] == timing
    assert response.headers["x-recording-media-time-s"] == str(ordinal)
    assert response.content == b"\xff\xd8jpeg" + str(ordinal).encode() + b"\xff\xd9"
    assert response.headers["x-recording-frame-id"] == "camera_optical"
    arguments, kwargs = calls[0]
    assert "select=between(n\\,0\\,3)" in arguments[arguments.index("-vf") + 1]
    assert kwargs["stdin"] == subprocess.DEVNULL
    assert kwargs["timeout"] == 15
    assert not any(value in arguments for value in ["--domain", "--publish", "--rate"])


def test_camera_uncovered_cursor_does_not_decode_or_freeze_last_frame(tmp_path, monkeypatch):
    service = _Recording(tmp_path)
    monkeypatch.setattr("gateway.services.recording_media.subprocess.run", lambda *_args, **_kwargs: pytest.fail("uncovered time must not decode"))
    assert recording_camera_frame(service, "media", t=5.0, segment=0) is None
    assert _client(service).get("/api/v1/recordings/media/camera-frame", params={"t": 1, "segment": 0}).status_code == 204


@pytest.mark.parametrize("next_start,expected_end,cursor,covered", [
    (4.5, 4.5, 4.2, True), (5.01, 5.01, 5.005, True), (8.0, 5.0, 6.0, False),
])
def test_camera_last_frame_covers_one_period_without_stretching_recording_gap(tmp_path, monkeypatch, next_start, expected_end, cursor, covered):
    service = _Recording(tmp_path)
    second = {**service.segments[0], "index": 1, "relative_path": "camera_color/segment_00001.mkv",
              "start_time_ns": str(ORIGIN + round(next_start * 1e9)), "end_time_ns": str(ORIGIN + round((next_start + 2) * 1e9))}
    service.segments.append(second)
    service.session["children"][0]["artifacts"].append(second["relative_path"])
    (tmp_path / "media" / second["relative_path"]).write_bytes(b"fixture")
    info = recording_scene_info(service, "media")
    assert info["camera"]["segments"][0]["end_s"] == 4.0
    assert info["camera"]["segments"][0]["coverage_end_s"] == expected_end
    monkeypatch.setattr("gateway.services.recording_media.shutil.which", lambda _name: "ffmpeg")
    monkeypatch.setattr("gateway.services.recording_media.subprocess.run", lambda args, **kwargs: _decoded_frames(args, range(3)))
    result = recording_camera_frame(service, "media", t=cursor, segment=0)
    assert (result is not None) is covered
    if covered:
        assert result[1]["X-Recording-Time-S"] == "4.0"


def test_camera_reuses_decoded_ordinal_but_never_returns_cached_image_after_file_removal(tmp_path, monkeypatch):
    service = _Recording(tmp_path)
    calls = []
    monkeypatch.setattr("gateway.services.recording_media.shutil.which", lambda _name: "ffmpeg")

    def run(args, **kwargs):
        calls.append(args)
        return _decoded_frames(args, range(3))

    monkeypatch.setattr("gateway.services.recording_media.subprocess.run", run)
    first = recording_camera_frame(service, "media", t=2.1, segment=0)
    native_calls = len(service.camera_calls)
    second = recording_camera_frame(service, "media", t=3.1, segment=0)
    assert len(calls) == 1
    assert len(service.camera_calls) == native_calls
    assert first[0] != second[0]
    assert second[1]["X-Recording-Frame-Index"] == "1"
    assert second[1]["X-Recording-Media-Time-S"] == "1"
    (tmp_path / "media/camera_color/segment_00000.mkv").unlink()
    assert _client(service).get("/api/v1/recordings/media/camera-frame", params={"t": 2.3, "segment": 0}).status_code == 404


def test_camera_final_short_block_preserves_ordinal_and_each_frame_pts(tmp_path, monkeypatch):
    service = _Recording(tmp_path)
    service.segments[0].update(
        frame_count=6, end_time_ns=str(ORIGIN + 7_000_000_000),
        frame_timestamps_ns=[str(ORIGIN + value * 1_000_000_000) for value in range(2, 8)],
    )
    calls = []
    monkeypatch.setattr("gateway.services.recording_media.shutil.which", lambda _name: "ffmpeg")

    def run(args, **kwargs):
        calls.append(args)
        return _decoded_frames(args, [4, 5])

    monkeypatch.setattr("gateway.services.recording_media.subprocess.run", run)
    fourth = recording_camera_frame(service, "media", t=6.1, segment=0)
    fifth = recording_camera_frame(service, "media", t=7.1, segment=0)
    assert len(calls) == 1
    assert "select=between(n\\,4\\,7)" in calls[0][calls[0].index("-vf") + 1]
    assert fourth[0] == b"\xff\xd8jpeg4\xff\xd9"
    assert fifth[0] == b"\xff\xd8jpeg5\xff\xd9"
    assert fifth[1]["X-Recording-Media-Time-S"] == "5"
    assert fifth[1]["X-Recording-Frame-Index"] == "5"


def test_scene_and_camera_reject_active_recording_before_file_access(tmp_path):
    service = _Recording(tmp_path)
    service.session["state"] = "recording"
    client = _client(service)
    for path in ["scene-info", "scene", "camera-frame?t=2&segment=0"]:
        assert client.get(f"/api/v1/recordings/media/{path}").status_code == 409
    assert service.camera_calls == service.scene_calls == []


def test_scene_forwards_window_roles_and_failed_retained_dds(tmp_path):
    service = _Recording(tmp_path)
    service.session["state"] = "failed"
    (tmp_path / "media/dds/sensors.mcap").rename(tmp_path / "media/dds/sensors.mcap.tmp")
    service.session["children"][1]["artifacts"] = ["dds/sensors.mcap.tmp"]
    response = _client(service).get("/api/v1/recordings/media/scene", params=[
        ("start", "3"), ("end", "8"), ("max_frames", "8"), ("max_points", "512"),
        ("max_poses", "250"), ("cloud_topic", "/slam/registered_cloud"),
        ("pose_topic", "/slam/odometry"), ("path_topic", "/nav/global_path"), ("path_topic", "/nav/local_path"),
    ])
    assert response.status_code == 200
    artifact, query = service.scene_calls[0]
    assert artifact.name == "sensors.mcap.tmp"
    assert query == {
        "start": 3.0, "end": 8.0, "max_frames": 8, "max_points": 512, "max_poses": 250,
        "cloud_topic": "/slam/registered_cloud", "pose_topic": "/slam/odometry",
        "path_topics": ("/nav/global_path", "/nav/local_path"),
    }


def test_scene_json_encoding_does_not_block_the_event_loop(tmp_path, monkeypatch):
    service = _Recording(tmp_path)
    original = JSONResponse.render
    encoding_has_loop = []

    def render(response, content):
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            encoding_has_loop.append(False)
        else:
            encoding_has_loop.append(True)
        return original(response, content)

    monkeypatch.setattr(JSONResponse, "render", render)
    response = _client(service).get("/api/v1/recordings/media/scene", params={"start": 0, "end": 5})
    assert response.status_code == 200
    assert response.json() == {
        "ok": True, "session_id": "media", "scene": {
            "version": 1, "ok": True, "mode": "scene", "start_time_ns": str(ORIGIN), "duration_s": 10.0,
            "topics": [{"topic": "/slam/registered_cloud", "kind": "cloud", "message_count": 12}],
        },
    }
    assert encoding_has_loop == [False]


def test_native_scene_and_camera_commands_only_read_declared_files(tmp_path):
    suffix = ".exe" if os.name == "nt" else ""
    for name in ["lingtu_recorder", "lingtu_dds_player", "lingtu_camera_player"]:
        path = tmp_path / (name + suffix)
        path.write_bytes(b"fixture")
        path.chmod(0o755)
    calls = []

    def run(arguments, **_kwargs):
        calls.append(arguments)
        return subprocess.CompletedProcess(arguments, 0, stdout=json.dumps({"version": 1, "ok": True}), stderr="")

    service = NativeRecordingService(environ={"LINGTU_RECORDING_BIN": str(tmp_path / ("lingtu_recorder" + suffix))}, runner=run)
    artifact = tmp_path / "sensors.mcap"
    index = tmp_path / "camera_color.mcap"
    service.scene_artifact(artifact, info=True)
    service.scene_artifact(
        artifact, start=2.0, end=6.0, max_frames=8, max_points=512, max_poses=250,
        cloud_topic="/slam/registered_cloud", pose_topic="/slam/odometry", path_topics=("/nav/global_path",),
    )
    service.camera_index_artifact(index)
    service.camera_index_artifact(index, segment=2)
    assert calls == [
        [str(tmp_path / ("lingtu_dds_player" + suffix)), "--scene-info", str(artifact), "--json"],
        [str(tmp_path / ("lingtu_dds_player" + suffix)), "--scene", str(artifact), "--json",
         "--start", "2.0", "--end", "6.0", "--max-frames", "8", "--max-points", "512",
         "--max-poses", "250", "--cloud-topic", "/slam/registered_cloud", "--pose-topic", "/slam/odometry",
         "--path-topic", "/nav/global_path"],
        [str(tmp_path / ("lingtu_camera_player" + suffix)), str(index), "--json"],
        [str(tmp_path / ("lingtu_camera_player" + suffix)), str(index), "--json", "--segment", "2"],
    ]
