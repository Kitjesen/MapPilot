from __future__ import annotations

import asyncio
import json
import subprocess
import threading
from functools import lru_cache
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
from lingtu.control import ProductControl
from lingtu.product_lock import ProductControlLock
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RunPlan

PRODUCT_SESSION_ID = "1" * 32
pytestmark = pytest.mark.usefixtures("allow_unbuilt_process_artifacts")


@lru_cache(maxsize=1)
def _inspection_plan() -> RunPlan:
    base = ProductControl(
        env="sim",
        robot="doso/thunder_v4",
        env_config={"backend": "mujoco"},
        process_env={},
    )._resolve("map")
    lifecycle = base.lifecycle
    lifecycle["product"] = "inspection"
    return RunPlan.create(
        product="inspection",
        env=base.env,
        robot=base.robot,
        process_control=base.process_control,
        modules=base.modules,
        processes=base.processes,
        available_processes=base.available_processes,
        stop_before_start=base.stop_before_start,
        contracts=("lingtu.product.inspection.v1",),
        critical_modules=base.critical_modules,
        route_contract=base.route_contract,
        host_config=base.host_config,
        lifecycle=lifecycle,
        native_process_environment=base.native_process_environment,
        parameters=base.parameters,
        simulation=base.simulation,
        support_processes=base.support_processes,
    )


def _exact_inspection_environment(tmp_path: Path) -> dict[str, str]:
    state_dir = (tmp_path / "runtime").resolve()
    state_dir.mkdir(parents=True, exist_ok=True)
    plan = _inspection_plan()
    plan_path = plan.write(state_dir / f"plan-{PRODUCT_SESSION_ID}.json")
    (state_dir / "current.json").write_text(
        json.dumps(
            {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": plan.product,
                "product_variant": plan.product_variant,
                "env": plan.env,
                "run_plan_path": str(plan_path),
                "product_session_id": PRODUCT_SESSION_ID,
                "map_name": None,
                "map_identity": None,
                "committed_at": 1.0,
            },
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return {
        "LINGTU_SESSION_ROOT": str(state_dir),
        "LINGTU_RUN_PLAN": str(plan_path),
        "LINGTU_PRODUCT": plan.product,
        "LINGTU_ENV": "sim",
        "LINGTU_ENV_BACKEND": "mujoco",
        "LINGTU_HOST_BOOT_ID": PRODUCT_SESSION_ID,
        "LINGTU_PRODUCT_SESSION_ID": PRODUCT_SESSION_ID,
    }


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
                context = self.session.get("context")
                expected = {
                    "product": argv[argv.index("--product") + 1]
                    if "--product" in argv
                    else "",
                    "product_session_id": argv[
                        argv.index("--product-session-id") + 1
                    ]
                    if "--product-session-id" in argv
                    else "",
                    "task_id": argv[argv.index("--inspection-task-id") + 1]
                    if "--inspection-task-id" in argv
                    else "",
                }
                if expected["product"] == "inspection" and context == expected:
                    return subprocess.CompletedProcess(
                        argv,
                        0,
                        stdout=_control(
                            self.root,
                            state=self.state,
                            session=self.session,
                        ),
                        stderr="",
                    )
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
            context = dict(self.session["context"])
            if "--product" in argv:
                context["product"] = argv[argv.index("--product") + 1]
            if "--product-session-id" in argv:
                context["product_session_id"] = argv[
                    argv.index("--product-session-id") + 1
                ]
            if "--inspection-task-id" in argv:
                context["task_id"] = argv[argv.index("--inspection-task-id") + 1]
            self.session["context"] = context
            self.healthy = True
            self.error = None
        elif argv[1] == "stop":
            if "--expected-session-id" in argv:
                expected = argv[argv.index("--expected-session-id") + 1]
                if self.session is None or self.session["session_id"] != expected:
                    output = {
                        "control_version": 1,
                        "ok": False,
                        "error": {
                            "code": "recording_session_mismatch",
                            "message": "active recording session does not match",
                        },
                    }
                    return subprocess.CompletedProcess(
                        argv, 4, stdout=json.dumps(output), stderr=""
                    )
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
            **_exact_inspection_environment(tmp_path),
        },
        runner=harness.run,
        startup_timeout_s=0.5,
    )
    service._binary = lambda *, required=True: tmp_path / "lingtu_recorder"  # type: ignore[method-assign]
    return service


def test_native_start_status_recovery_and_stop_use_root_control(tmp_path: Path) -> None:
    harness = _NativeHarness(tmp_path / "recordings")
    service = _service(tmp_path, harness)

    started = service.start(
        duration=60,
        prefix="field",
        capture_profile="evidence",
        task_id="inspection-task-001",
        camera=True,
        minimum_free_gib=12,
    )

    assert started.recording is True
    assert started.product_session_id == PRODUCT_SESSION_ID
    argv = next(command for command in harness.commands if command[1] == "start")
    assert argv[1] == "start"
    assert argv[argv.index("--root") + 1] == str(harness.root)
    assert argv[argv.index("--prefix") + 1] == "field"
    assert argv[argv.index("--dds") + 1] == "on"
    assert argv[argv.index("--dds-preset") + 1] == "inspection-evidence-v1"
    assert argv[argv.index("--inspection-task-id") + 1] == "inspection-task-001"
    assert argv[argv.index("--camera") + 1] == "on"
    assert argv[argv.index("--min-free-gib") + 1] == "12"
    assert argv[argv.index("--product") + 1] == "inspection"
    assert argv[argv.index("--product-session-id") + 1] == PRODUCT_SESSION_ID
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


def test_evidence_recording_supports_task_lifetime_and_exact_session_stop(tmp_path: Path) -> None:
    harness = _NativeHarness(tmp_path / "recordings")
    service = _service(tmp_path, harness)

    started = service.start(
        duration=0,
        prefix="inspection",
        capture_profile="evidence",
        task_id="inspection-task-001",
    )

    with pytest.raises(NativeRecordingError) as caught:
        service.stop(expected_session_id="newer-session")
    assert caught.value.code == "recording_session_mismatch"
    mismatch = harness.commands[-1]
    assert mismatch[1] == "stop"
    assert mismatch[mismatch.index("--expected-session-id") + 1] == "newer-session"
    assert [command[1] for command in harness.commands].count("status") == 0

    stopped = service.stop(expected_session_id=started.session_id)
    assert stopped.state == "completed"
    stop_count = [command[1] for command in harness.commands].count("stop")

    replayed = service.stop(expected_session_id=started.session_id)

    assert replayed.state == "completed"
    assert [command[1] for command in harness.commands].count("stop") == stop_count + 1
    assert [command[1] for command in harness.commands].count("status") == 0


def test_exact_inspection_start_adopts_the_active_native_session(tmp_path: Path) -> None:
    harness = _NativeHarness(tmp_path / "recordings")
    service = _service(tmp_path, harness)

    first = service.start(
        duration=0,
        prefix="inspection",
        capture_profile="evidence",
        task_id="inspection-task-001",
    )
    retry = service.start(
        duration=0,
        prefix="inspection",
        capture_profile="evidence",
        task_id="inspection-task-001",
    )

    assert retry.session_id == first.session_id
    assert [command[1] for command in harness.commands].count("start") == 2


def test_duplicate_environment_does_not_override_run_plan_context(
    tmp_path: Path,
    monkeypatch,
) -> None:
    import gateway.services.recording as recording

    harness = _NativeHarness(tmp_path / "recordings")
    service = _service(tmp_path, harness)
    service._environ = {**service._environ, "LINGTU_ENV": "real"}
    service._environ.pop("LINGTU_ENV_BACKEND")
    seen: list[tuple[str, dict[str, str]]] = []
    plan_path = Path(service._environ["LINGTU_RUN_PLAN"])
    plan = RunPlan.load(plan_path)

    class RealControl:
        def __init__(self, *, env, env_config, process_env):
            seen.append((env, dict(env_config)))
            assert process_env is service._environ

        def _current_plan_and_path(self, _state_dir):
            return plan, plan_path, PRODUCT_SESSION_ID

    monkeypatch.setattr(recording, "ProductControl", RealControl)

    started = service.start(
        duration=0,
        prefix="inspection",
        capture_profile="evidence",
        task_id="inspection-task-real",
    )

    assert started.state == "recording"
    assert seen == [("sim", {})]


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
    "kwargs",
    [
        {"capture_profile": "topics"},
        {"minimum_free_gib": 0},
        {"minimum_free_gib": 101},
    ],
)
def test_recording_request_rejects_unsafe_capture_configuration(
    kwargs: dict[str, object],
) -> None:
    with pytest.raises(ValueError):
        RecordingStartRequest(**kwargs)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"capture_profile": "evidence"},
        {"capture_profile": "evidence", "task_id": "   "},
        {"capture_profile": "sensors", "task_id": "inspection-task-001"},
    ],
)
def test_recording_request_requires_task_id_only_for_evidence(
    kwargs: dict[str, object],
) -> None:
    with pytest.raises(ValueError):
        RecordingStartRequest(**kwargs)


def test_recording_request_normalizes_evidence_task_id() -> None:
    request = RecordingStartRequest(
        capture_profile="evidence",
        task_id="  inspection-task-001  ",
    )

    assert request.task_id == "inspection-task-001"


@pytest.mark.parametrize("task_id", ["x" * 257, "é" * 129, "bad\x00task"])
def test_recording_request_rejects_native_invalid_task_id(task_id: str) -> None:
    with pytest.raises(ValueError):
        RecordingStartRequest(capture_profile="evidence", task_id=task_id)


def test_recording_request_accepts_native_task_id_byte_limit() -> None:
    request = RecordingStartRequest(capture_profile="evidence", task_id="é" * 128)

    assert len(request.task_id.encode("utf-8")) == 256


def test_recording_service_rejects_unknown_capture_profile(tmp_path: Path) -> None:
    harness = _NativeHarness(tmp_path / "recordings")
    service = _service(tmp_path, harness)

    with pytest.raises(NativeRecordingError) as caught:
        service.start(duration=60, prefix="field", capture_profile="raw-topics")

    assert caught.value.code == "invalid_recording_capture_profile"
    assert caught.value.status_code == 422
    assert harness.commands == []


@pytest.mark.parametrize(
    ("capture_profile", "task_id"),
    [
        ("evidence", None),
        ("evidence", "   "),
        ("sensors", "inspection-task-001"),
    ],
)
def test_recording_service_rejects_invalid_task_binding(
    tmp_path: Path,
    capture_profile: str,
    task_id: str | None,
) -> None:
    harness = _NativeHarness(tmp_path / "recordings")
    service = _service(tmp_path, harness)

    with pytest.raises(NativeRecordingError) as caught:
        service.start(
            duration=60,
            prefix="field",
            capture_profile=capture_profile,
            task_id=task_id,
        )

    assert caught.value.code == "invalid_recording_task_id"
    assert caught.value.status_code == 422
    assert harness.commands == []


@pytest.mark.parametrize("task_id", ["x" * 257, "é" * 129, "bad\x00task"])
def test_recording_service_rejects_native_invalid_task_id(
    tmp_path: Path,
    task_id: str,
) -> None:
    harness = _NativeHarness(tmp_path / "recordings")
    service = _service(tmp_path, harness)

    with pytest.raises(NativeRecordingError) as caught:
        service.start(
            duration=60,
            prefix="field",
            capture_profile="evidence",
            task_id=task_id,
        )

    assert caught.value.code == "invalid_recording_task_id"
    assert harness.commands == []


@pytest.mark.parametrize(
    "environment_override",
    [
        {"LINGTU_PRODUCT_SESSION_ID": ""},
    ],
)
def test_evidence_recording_requires_exact_inspection_identity(
    tmp_path: Path,
    environment_override: dict[str, str],
) -> None:
    harness = _NativeHarness(tmp_path / "recordings")
    service = _service(tmp_path, harness)
    service._environ = {**service._environ, **environment_override}

    with pytest.raises(NativeRecordingError) as caught:
        service.start(
            duration=60,
            prefix="field",
            capture_profile="evidence",
            task_id="inspection-task-001",
        )

    assert caught.value.code == "inspection_recording_identity_required"
    assert caught.value.status_code == 409
    assert harness.commands == []


@pytest.mark.parametrize(
    "identity_fault",
    ["no_current", "session_mismatch", "current_product_mismatch"],
)
def test_evidence_recording_rejects_uncommitted_or_stale_host_identity(
    tmp_path: Path,
    identity_fault: str,
) -> None:
    harness = _NativeHarness(tmp_path / "recordings")
    service = _service(tmp_path, harness)
    state_dir = Path(service._environ["LINGTU_SESSION_ROOT"])
    current_path = state_dir / "current.json"
    if identity_fault == "no_current":
        current_path.unlink()
    elif identity_fault == "session_mismatch":
        service._environ = {
            **service._environ,
            "LINGTU_PRODUCT_SESSION_ID": "2" * 32,
        }
    else:
        current = json.loads(current_path.read_text(encoding="utf-8"))
        current["product"] = "nav"
        current_path.write_text(json.dumps(current), encoding="utf-8")

    with pytest.raises(NativeRecordingError) as caught:
        service.start(
            duration=60,
            prefix="field",
            capture_profile="evidence",
            task_id="inspection-task-001",
        )

    assert caught.value.code == "inspection_recording_identity_required"
    assert caught.value.status_code == 409
    assert harness.commands == []


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


def test_status_prefers_specific_session_recovery_reason(tmp_path: Path) -> None:
    harness = _NativeHarness(tmp_path / "recordings")
    path = harness.root / "existing"
    harness.session = {
        **_session(path, state="recording"),
        "error": "recording_recovery_required: manager identity is stale",
    }
    harness.state = "recording"
    harness.healthy = False
    harness.error = "native recording session is unhealthy"

    status = _service(tmp_path, harness).status()

    assert status.error == "recording_recovery_required: manager identity is stale"


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


def test_gateway_recording_api_uses_native_state() -> None:
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
            RecordingStartRequest(
                duration=60,
                prefix="field",
                name="sdk-field",
                capture_profile="evidence",
                task_id="inspection-task-001",
                camera=True,
                minimum_free_gib=12,
            )
        )
    )
    status = asyncio.run(_endpoint(app, "/api/v1/recordings/status", "GET")())
    stopped = asyncio.run(_endpoint(app, "/api/v1/recordings/stop", "POST")())

    for public_payload in (started, status, stopped):
        assert public_payload.get("path") is None
        assert public_payload.get("pid") is None
    assert status.get("exit_code") is None

    assert started["backend"] == "native_mcap"
    assert started["prefix"] == "field"
    assert started["capture_profile"] == "evidence"
    assert started["task_id"] == "inspection-task-001"
    assert started["camera"] is True
    assert started["minimum_free_gib"] == 12
    assert recording.start_kwargs == {
        "duration": 60,
        "prefix": "field",
        "capture_profile": "evidence",
        "task_id": "inspection-task-001",
        "camera": True,
        "minimum_free_gib": 12,
    }
    assert status["recording"] is True
    assert stopped["status"] == "completed"
    assert not any(
        getattr(route, "path", "").startswith("/api/v1/bag/")
        for route in app.routes
    )


def test_gateway_recording_error_does_not_expose_native_details() -> None:
    class _Recording:
        def start(self, **_kwargs):
            raise NativeRecordingError(
                "native_recorder_unavailable",
                "native recording binary is not installed",
                status_code=503,
                detail={
                    "expected": ["/opt/lingtu/secret/lingtu_recorder"],
                    "path": "/data/recordings/private",
                    "pid": 123,
                },
            )

    app = FastAPI()
    gateway = SimpleNamespace(
        _go2rtc_upstream="http://127.0.0.1:1984",
        _recording=_Recording(),
    )
    register_operation_routes(app, gateway)

    response = asyncio.run(
        _endpoint(app, "/api/v1/recordings/start", "POST")(RecordingStartRequest())
    )
    payload = json.loads(response.body)
    assert response.status_code == 503
    assert payload == {"error": "native_recorder_unavailable", "detail": None}


def test_gateway_adapter_contains_no_manifest_or_catalog_implementation() -> None:
    source = (
        Path(__file__).resolve().parents[2] / "src/gateway/services/recording.py"
    ).read_text(encoding="utf-8")
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
    source = (
        Path(__file__).resolve().parents[2] / "src/gateway/routes/operations.py"
    ).read_text(encoding="utf-8")
    for forbidden in ("record_bag.sh", "killpg", "_bag_proc", "bash_not_found"):
        assert forbidden not in source
