from __future__ import annotations

import json
import os
import queue
import threading
from pathlib import Path
from types import SimpleNamespace

from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes.recordings import register_recording_routes
from gateway.services.recording_streams import NativeStreamMonitor


class _Pipe:
    def __init__(self) -> None:
        self.lines: queue.Queue[bytes] = queue.Queue()
        self.consumed = threading.Event()
        self.reads = 0

    def readline(self, _size: int) -> bytes:
        self.reads += 1
        if self.reads > 1:
            self.consumed.set()
        return self.lines.get()

    def close(self) -> None:
        pass


class _Process:
    def __init__(self) -> None:
        self.stdout = _Pipe()
        self.exit_code: int | None = None
        self.terminated = threading.Event()

    def poll(self):
        return self.exit_code

    def terminate(self):
        self.exit_code = 0
        self.stdout.lines.put(b"")
        self.terminated.set()

    def wait(self, *, timeout: float):
        assert timeout > 0
        return self.exit_code


def _fixture(tmp_path: Path):
    recorder = tmp_path / ("lingtu_recorder.exe" if os.name == "nt" else "lingtu_recorder")
    observer = tmp_path / ("lingtu_dds_observer.exe" if os.name == "nt" else "lingtu_dds_observer")
    recorder.write_bytes(b"fixture")
    observer.write_bytes(b"fixture")
    observer.chmod(0o755)
    process = _Process()
    starts: list[tuple[list[str], dict]] = []

    def factory(argv, **kwargs):
        starts.append((argv, kwargs))
        return process

    clock = [100.0]
    monitor = NativeStreamMonitor(
        recorder_binary=lambda: recorder,
        environ={"LINGTU_DDS_DOMAIN_ID": "187", "CYCLONEDDS_URI": "loopback-fixture"},
        process_factory=factory,
        monotonic=lambda: clock[0],
    )
    return monitor, process, starts, clock


def _native_snapshot() -> dict:
    return {
        "version": 1, "ok": True, "domain": 187, "observed_at_unix_ns": 123456, "window_s": 0.5,
        "topics": [{
            "topic": "/robot/joint_states", "type_name": "lingtu.dds.JointState",
            "message_count": 12, "bytes_received": 1024, "rate_hz": 24.0,
            "bytes_per_second": 2048.0, "last_sample_age_ms": 5.0, "source_time_ns": 100,
            "preview": {"name": ["hip"], "position": [0.25], "effort": [1.5]},
        }, {
            "topic": "/nav/command/request", "type_name": "lingtu.dds.NavigationCommandRequest",
            "message_count": 0, "bytes_received": 0, "rate_hz": 0.0, "bytes_per_second": 0.0,
            "last_sample_age_ms": None, "source_time_ns": None, "preview": None,
        }],
    }


def test_stream_snapshot_reuses_one_read_only_native_child(tmp_path: Path) -> None:
    monitor, process, starts, clock = _fixture(tmp_path)
    try:
        initial = monitor.snapshot()
        assert initial["state"] == "starting" and initial["topics"] == []
        process.stdout.lines.put(json.dumps(_native_snapshot()).encode() + b"\n")
        assert process.stdout.consumed.wait(2)
        snapshot = monitor.snapshot()
        assert snapshot["state"] == "observing"
        assert snapshot["topics"][0]["preview"]["position"] == [0.25]
        assert snapshot["topics"][1]["last_sample_age_ms"] is None
        assert snapshot["topics"][1]["message_count"] == 0
        assert len(starts) == 1
        assert starts[0][0][1:] == ["--domain", "187", "--interval-ms", "500"]
        assert starts[0][1]["env"]["CYCLONEDDS_URI"] == "loopback-fixture"
        assert starts[0][1]["stderr"] is None
        assert "record" not in starts[0][0]
        clock[0] = 103
        stale = monitor.snapshot()
        assert stale["state"] == "stale" and stale["snapshot_age_ms"] == 3000
        assert len(starts) == 1
    finally:
        monitor.close()
    assert process.terminated.is_set()
    assert monitor.snapshot()["available"] is False
    assert len(starts) == 1


def test_leaving_live_view_releases_observer_after_idle_grace(tmp_path: Path) -> None:
    monitor, process, _starts, clock = _fixture(tmp_path)
    monitor.snapshot()
    clock[0] = 111
    process.stdout.lines.put(json.dumps(_native_snapshot()).encode() + b"\n")
    assert process.terminated.wait(2)
    monitor.close()


def test_bad_native_output_is_unavailable_not_fake_live_data(tmp_path: Path) -> None:
    monitor, process, starts, _clock = _fixture(tmp_path)
    monitor.snapshot()
    process.stdout.lines.put(b"not a native JSON snapshot\n")
    assert process.terminated.wait(2)
    monitor._thread.join(timeout=2)
    result = monitor.snapshot()
    assert result["state"] == "unavailable"
    assert result["error"] == "native_stream_observer_protocol_error"
    assert result["topics"] == []
    assert len(starts) == 1
    monitor.close()


def test_missing_native_observer_does_not_block_recording_catalog(tmp_path: Path) -> None:
    monitor, _process, starts, _clock = _fixture(tmp_path)
    for path in tmp_path.iterdir():
        if "observer" in path.name:
            path.unlink()
    assert monitor.snapshot()["state"] == "unavailable"
    assert starts == []
    monitor.close()


def test_live_route_returns_native_stats_without_owning_module_shutdown() -> None:
    class _RouteApp(FastAPI):
        @property
        def add_event_handler(self):
            raise AttributeError("No legacy event API")

    class _Recording:
        closed = False

        def streams(self):
            return {**_native_snapshot(), "available": True, "state": "observing", "snapshot_age_ms": 10}

        def close_streams(self):
            self.closed = True

    recording = _Recording()
    app = _RouteApp()
    register_recording_routes(app, SimpleNamespace(_recording=recording))
    with TestClient(app) as client:
        response = client.get("/api/v1/recordings/streams")
        assert response.status_code == 200
        assert response.json()["topics"][0]["topic"] == "/robot/joint_states"
        assert response.json()["topics"][0]["rate_hz"] == 24
    assert recording.closed is False


def test_gateway_stop_closes_recording_observer(tmp_path: Path, monkeypatch) -> None:
    from gateway import gateway_module
    from gateway.services.recording import NativeRecordingService

    monitor, process, _starts, _clock = _fixture(tmp_path)
    monitor.snapshot()
    recording = NativeRecordingService.__new__(NativeRecordingService)
    recording._lock = threading.RLock()
    recording._stream_monitor = monitor
    gateway = gateway_module.GatewayModule.__new__(gateway_module.GatewayModule)
    gateway._recording = recording
    calls = []
    monkeypatch.setattr(gateway_module, "stop_background_threads", lambda _gw: calls.append("http"))
    monkeypatch.setattr(gateway_module, "shutdown_teleop", lambda _gw: calls.append("teleop"))
    monkeypatch.setattr(gateway_module.Module, "stop", lambda _gw: calls.append("module"))
    try:
        gateway.stop()
        assert process.terminated.is_set()
        assert monitor.snapshot()["available"] is False
        assert calls == ["http", "teleop", "module"]
    finally:
        monitor.close()
