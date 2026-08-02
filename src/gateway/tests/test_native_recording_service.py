from __future__ import annotations

import asyncio
import json
import subprocess
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi import FastAPI

from gateway.routes.operations import register_operation_routes
from gateway.schemas import RecordingStartRequest
from gateway.services.recording import (
    NativeRecordingError,
    NativeRecordingService,
    RecordingSnapshot,
)
from lingtu.product_lock import ProductControlLock


def _session(path: Path, *, state: str, pid: int = 4321) -> dict[str, object]:
    return {
        "version": 1,
        "session_id": path.name,
        "state": state,
        "session_directory": str(path),
        "manager_pid": pid,
        "created_at_unix_ns": 1_000_000_000,
        "started_at_unix_ns": 2_000_000_000,
        "ended_at_unix_ns": 3_000_000_000 if state in {"completed", "failed"} else None,
        "error": "worker failed" if state == "failed" else None,
        "context": {},
        "children": [],
    }


def _control(
    root: Path,
    *,
    state: str,
    session: dict[str, object] | None,
    healthy: bool = True,
    error: str | None = None,
) -> str:
    return json.dumps(
        {
            "control_version": 1,
            "ok": True,
            "healthy": healthy,
            "state": state,
            "root": str(root),
            "session": session,
            "size_bytes": 42 if session else 0,
            "size_truncated": False,
            "disk_free": 1024,
            "disk_total": 2048,
            "error": error,
        }
    )


class _NativeHarness:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.commands: list[list[str]] = []
        self.session: dict[str, object] | None = None
        self.state = "idle"
        self.healthy = True
        self.error: str | None = None

    def run(self, arguments, **_kwargs):
        argv = list(arguments)
        self.commands.append(argv)
        if argv[1] == "start":
            if self.session is not None and self.state in {"preparing", "recording", "stopping"}:
                output = {
                    "control_version": 1,
                    "ok": False,
                    "error": {
                        "code": "recording_in_progress",
                        "message": "an active session already exists",
                    },
                }
                return subprocess.CompletedProcess(argv, 4, stdout=json.dumps(output), stderr="")
            prefix = argv[argv.index("--prefix") + 1]
            path = self.root / f"{prefix}_generated"
            self.state = "recording"
            self.session = _session(path, state=self.state)
            self.healthy = True
            self.error = None
        elif argv[1] == "stop":
            if self.session is None:
                output = {
                    "control_version": 1,
                    "ok": False,
                    "error": {"code": "not_recording", "message": "no active session"},
                }
                return subprocess.CompletedProcess(argv, 4, stdout=json.dumps(output), stderr="")
            self.state = "completed"
            self.session = {**self.session, "state": self.state, "ended_at_unix_ns": 3_000_000_000}
            self.healthy = True
            self.error = None
        return subprocess.CompletedProcess(
            argv,
            0 if self.healthy else 4,
            stdout=_control(
                self.root,
                state=self.state,
                session=self.session,
                healthy=self.healthy,
                error=self.error,
            ),
            stderr="",
        )


def _service(tmp_path: Path, harness: _NativeHarness) -> NativeRecordingService:
    service = NativeRecordingService(
        repository_root=tmp_path,
        environ={
            "LINGTU_RECORDING_ROOT": str(harness.root),
            "LINGTU_SESSION_ROOT": str(tmp_path / "runtime"),
            "LINGTU_PRODUCT": "inspection",
            "LINGTU_RUN_PLAN_FINGERPRINT": "run-plan-sha256",
        },
        runner=harness.run,
        startup_timeout_s=0.5,
    )
    service._binary = lambda *, required=True: tmp_path / "lingtu_recorder"  # type: ignore[method-assign]
    return service


def test_native_start_status_recovery_and_stop_use_root_control(tmp_path: Path) -> None:
    harness = _NativeHarness(tmp_path / "recordings")
    service = _service(tmp_path, harness)

    started = service.start(duration=60, prefix="field")

    assert started.recording is True
    argv = next(command for command in harness.commands if command[1] == "start")
    assert argv[1] == "start"
    assert argv[argv.index("--root") + 1] == str(harness.root)
    assert argv[argv.index("--prefix") + 1] == "field"
    assert argv[argv.index("--dds") + 1] == "on"
    assert argv[argv.index("--camera") + 1] == "off"
    assert argv[argv.index("--product") + 1] == "inspection"
    assert argv[argv.index("--run-plan-fingerprint") + 1] == "run-plan-sha256"
    assert "bash" not in argv
    assert "record_bag.sh" not in " ".join(argv)

    recovered = _service(tmp_path, harness).status()
    assert recovered.recording is True
    assert recovered.session_id == started.session_id

    stopped = service.stop(timeout_ms=100)
    assert stopped.state == "completed"
    assert stopped.recording is False
    assert all("--root" in command for command in harness.commands)
    assert any(command[1] == "stop" for command in harness.commands)


def test_recording_start_rejects_during_product_transition(tmp_path: Path) -> None:
    harness = _NativeHarness(tmp_path / "recordings")
    service = _service(tmp_path, harness)
    state_dir = tmp_path / "runtime"
    acquired = threading.Event()
    release = threading.Event()

    def hold_product_lock() -> None:
        with ProductControlLock(state_dir, timeout_s=1.0):
            acquired.set()
            release.wait(timeout=5.0)

    owner = threading.Thread(target=hold_product_lock)
    owner.start()
    assert acquired.wait(timeout=2.0)
    try:
        with pytest.raises(NativeRecordingError) as caught:
            service.start(duration=60, prefix="field")
    finally:
        release.set()
        owner.join(timeout=2.0)

    assert caught.value.code == "product_transition_in_progress"
    assert caught.value.status_code == 409
    assert harness.commands == []


def test_missing_native_binary_fails_closed(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("gateway.services.recording.shutil.which", lambda _name: None)
    service = NativeRecordingService(
        repository_root=tmp_path,
        environ={"LINGTU_RECORDING_ROOT": str(tmp_path / "recordings")},
    )

    assert service.status().available is False
    with pytest.raises(NativeRecordingError, match="not installed") as caught:
        service.start(duration=60, prefix="field")
    assert caught.value.code == "native_recorder_unavailable"
    assert caught.value.status_code == 503


def test_exact_native_binary_override_has_highest_precedence(tmp_path: Path) -> None:
    exact = tmp_path / "custom" / "recorder"
    service = NativeRecordingService(
        repository_root=tmp_path,
        environ={
            "LINGTU_RECORDING_BIN": str(exact),
            "LINGTU_RECORDING_BIN_DIR": str(tmp_path / "other"),
        },
    )

    assert service._binary_candidates()[0] == exact


@pytest.mark.parametrize("prefix", ["", "has space", "../escape", "中文", "x" * 41])
def test_recording_request_rejects_invalid_prefix(prefix: str) -> None:
    with pytest.raises(ValueError):
        RecordingStartRequest(prefix=prefix)


@pytest.mark.parametrize(
    ("code", "status_code"),
    [
        ("recording_manifest_invalid", 503),
        ("multiple_recordings_active", 409),
        ("recording_catalog_unsafe", 503),
    ],
)
def test_native_catalog_errors_are_preserved(
    tmp_path: Path,
    code: str,
    status_code: int,
) -> None:
    harness = _NativeHarness(tmp_path / "recordings")

    def fail(arguments, **_kwargs):
        payload = {
            "control_version": 1,
            "ok": False,
            "error": {"code": code, "message": "native rejected catalog"},
        }
        return subprocess.CompletedProcess(list(arguments), 4, stdout=json.dumps(payload), stderr="")

    harness.run = fail  # type: ignore[method-assign]
    service = _service(tmp_path, harness)
    with pytest.raises(NativeRecordingError) as caught:
        service.status()
    assert caught.value.code == code
    assert caught.value.status_code == status_code


def test_unhealthy_active_session_blocks_start_and_can_be_stopped(tmp_path: Path) -> None:
    harness = _NativeHarness(tmp_path / "recordings")
    path = harness.root / "existing"
    harness.session = _session(path, state="recording")
    harness.state = "recording"
    harness.healthy = False
    harness.error = "status probe failed"
    service = _service(tmp_path, harness)

    status = service.status()
    assert status.state == "recording"
    assert status.healthy is False
    assert status.recording is False

    with pytest.raises(NativeRecordingError) as caught:
        service.start(duration=60, prefix="second")
    assert caught.value.code == "recording_in_progress"

    stopped = service.stop(timeout_ms=100)
    assert stopped.state == "completed"


def test_invalid_native_control_response_fails_closed(tmp_path: Path) -> None:
    harness = _NativeHarness(tmp_path / "recordings")

    def invalid(arguments, **_kwargs):
        return subprocess.CompletedProcess(list(arguments), 0, stdout="not-json", stderr="")

    harness.run = invalid  # type: ignore[method-assign]
    service = _service(tmp_path, harness)
    with pytest.raises(NativeRecordingError) as caught:
        service.status()
    assert caught.value.code == "native_recorder_protocol_error"


def _endpoint(app: FastAPI, path: str, method: str):
    for route in app.routes:
        if getattr(route, "path", None) == path and method in getattr(route, "methods", set()):
            return route.endpoint
    raise AssertionError(f"route not found: {method} {path}")


def test_gateway_recording_api_and_deprecated_bag_path_share_native_state() -> None:
    snapshot = RecordingSnapshot(
        available=True,
        healthy=True,
        state="recording",
        session_id="field-001",
        path="/data/recordings/field-001",
        pid=123,
    )

    class _Recording:
        start_kwargs = None

        def start(self, **kwargs):
            self.start_kwargs = kwargs
            return snapshot

        def status(self):
            return snapshot

        def stop(self):
            return RecordingSnapshot(
                available=True,
                healthy=True,
                state="completed",
                session_id=snapshot.session_id,
                path=snapshot.path,
                pid=snapshot.pid,
            )

    app = FastAPI()
    recording = _Recording()
    gateway = SimpleNamespace(_go2rtc_upstream="http://127.0.0.1:1984", _recording=recording)
    register_operation_routes(app, gateway)

    started = asyncio.run(
        _endpoint(app, "/api/v1/recordings/start", "POST")(
            RecordingStartRequest(duration=60, prefix="field", name="sdk-field")
        )
    )
    status = asyncio.run(_endpoint(app, "/api/v1/recordings/status", "GET")())
    legacy_status = asyncio.run(_endpoint(app, "/api/v1/bag/status", "GET")())
    stopped = asyncio.run(_endpoint(app, "/api/v1/recordings/stop", "POST")())

    assert started["backend"] == "native_mcap"
    assert started["prefix"] == "sdk-field"
    assert recording.start_kwargs == {"duration": 60, "prefix": "sdk-field"}
    assert status == legacy_status
    assert status["recording"] is True
    assert stopped["status"] == "completed"


def test_gateway_adapter_contains_no_manifest_or_catalog_implementation() -> None:
    source = (Path(__file__).parents[1] / "services" / "recording.py").read_text(encoding="utf-8")
    for forbidden in (
        "os.scandir",
        "os.walk",
        "session.json",
        "read_text(",
        "stat.S_ISREG",
        "subprocess.Popen",
        "uuid.uuid4",
        "datetime.now",
        "os.kill",
    ):
        assert forbidden not in source


def test_gateway_runtime_has_no_ros_bag_fallback() -> None:
    source = (Path(__file__).parents[1] / "routes" / "operations.py").read_text(encoding="utf-8")
    for forbidden in ("record_bag.sh", "killpg", "_bag_proc", "bash_not_found"):
        assert forbidden not in source
