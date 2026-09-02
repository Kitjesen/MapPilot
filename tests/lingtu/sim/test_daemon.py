# ruff: noqa: S101
"""Behavior tests for the persistent simulation supervisor daemon."""

from __future__ import annotations

import json
import multiprocessing
import os
import signal
import time
from pathlib import Path
from typing import Any

import pytest

import lingtu.sim.daemon as daemon_module
from lingtu.sim.daemon import (
    SimulationSupervisorDaemonRecord,
    ensure_sim_supervisor,
    load_daemon_record,
    serve_supervisor,
)
from lingtu.sim.rpc import REQUEST_SCHEMA, SupervisorRequest, round_trip
from lingtu.sim.supervisor import SimulationSupervisorClient


def _wait_for_record(
    session_root: Path,
    timeout_s: float = 10,
) -> SimulationSupervisorDaemonRecord:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            return load_daemon_record(session_root)
        except (OSError, RuntimeError):
            time.sleep(0.02)
    raise AssertionError("simulation supervisor daemon record was not published")


def _terminate_identity(record: SimulationSupervisorDaemonRecord) -> None:
    if not record.process_identity.matches():
        return
    if os.name == "nt":
        import ctypes
        from ctypes import wintypes

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.OpenProcess.argtypes = [
            wintypes.DWORD,
            wintypes.BOOL,
            wintypes.DWORD,
        ]
        kernel32.OpenProcess.restype = wintypes.HANDLE
        kernel32.TerminateProcess.argtypes = [wintypes.HANDLE, wintypes.UINT]
        kernel32.TerminateProcess.restype = wintypes.BOOL
        kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
        handle = kernel32.OpenProcess(0x0001, False, record.process_identity.pid)
        assert handle
        try:
            assert kernel32.TerminateProcess(handle, 1)
        finally:
            kernel32.CloseHandle(handle)
    else:
        os.kill(record.process_identity.pid, signal.SIGTERM)
    deadline = time.monotonic() + 15
    while record.process_identity.matches() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert record.process_identity.matches() is False


def _invalid_request(session_root: Path) -> SupervisorRequest:
    product_session_id = "a" * 32
    plan_path = session_root / f"plan-{product_session_id}.json"
    plan_path.write_text("{}\n", encoding="utf-8")
    return SupervisorRequest(
        schema_version=REQUEST_SCHEMA,
        action="apply",
        run_plan_path=str(plan_path),
        product_session_id=product_session_id,
    )


def test_service_publishes_record_after_listening(
    tmp_path: Path,
) -> None:
    session_root = (tmp_path / "session").resolve()
    repository_root = (tmp_path / "repository").resolve()
    session_root.mkdir()
    repository_root.mkdir()
    context = multiprocessing.get_context("spawn")
    process = context.Process(
        target=serve_supervisor,
        args=(session_root, repository_root),
    )
    process.start()
    try:
        record = _wait_for_record(session_root)
        assert record.process_identity.pid == process.pid
        assert record.process_identity.matches() is True
        assert record.repository_root == str(repository_root)
        assert set(record.as_dict()) == {
            "schema_version",
            "repository_root",
            "process_identity",
        }
        time.sleep(0.2)
        assert process.is_alive()

        response = round_trip(
            session_root,
            _invalid_request(session_root),
            timeout_s=5,
        )
        assert response.success is False
        _terminate_identity(record)
        process.join(timeout=5)
        assert process.exitcode is not None
    finally:
        if process.is_alive():
            process.terminate()
            process.join(timeout=2)
        if process.is_alive():
            process.kill()
            process.join(timeout=2)
        process.close()


def test_service_cleans_rpc_files_when_runtime_close_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session_root = (tmp_path / "session").resolve()
    repository_root = (tmp_path / "repository").resolve()
    session_root.mkdir()
    repository_root.mkdir()
    server_closed = False

    class BrokenHandler:
        def __init__(self, _manager: object) -> None:
            pass

        def handle(self, _request: object) -> object:
            return object()

        def close(self) -> None:
            raise RuntimeError("runtime close failed")

    class FakeServer:
        def __init__(self, session: Path) -> None:
            self.session = session

        def __enter__(self) -> FakeServer:
            (self.session / "supervisor").mkdir()
            return self

        def serve_once(self, _handler: object) -> None:
            raise RuntimeError("stop service")

        def close(self) -> None:
            nonlocal server_closed
            server_closed = True

        def __exit__(
            self,
            _exc_type: type[BaseException] | None,
            _exc: BaseException | None,
            _traceback: object | None,
        ) -> None:
            self.close()

    monkeypatch.setattr(daemon_module, "_SimulationRequestHandler", BrokenHandler)
    monkeypatch.setattr(daemon_module, "LocalRpcServer", FakeServer)

    with pytest.raises(RuntimeError, match="runtime close failed"):
        serve_supervisor(session_root, repository_root)

    assert server_closed is True
    assert not (session_root / "supervisor" / "daemon.json").exists()


def test_ensure_starts_once_and_reuses_live_daemon(tmp_path: Path) -> None:
    session_root = (tmp_path / "session").resolve()
    repository_root = (tmp_path / "repository").resolve()
    session_root.mkdir()
    repository_root.mkdir()
    record: SimulationSupervisorDaemonRecord | None = None
    try:
        first = ensure_sim_supervisor(session_root, repository_root, timeout_s=10)
        record = load_daemon_record(session_root)
        second = ensure_sim_supervisor(session_root, repository_root, timeout_s=10)

        assert isinstance(first, SimulationSupervisorClient)
        assert isinstance(second, SimulationSupervisorClient)
        assert load_daemon_record(session_root) == record
        assert record.process_identity.matches() is True
    finally:
        if record is not None:
            _terminate_identity(record)


def test_ensure_cleans_dead_daemon_and_starts_a_new_one(tmp_path: Path) -> None:
    session_root = (tmp_path / "session").resolve()
    repository_root = (tmp_path / "repository").resolve()
    session_root.mkdir()
    repository_root.mkdir()
    previous: SimulationSupervisorDaemonRecord | None = None
    current: SimulationSupervisorDaemonRecord | None = None
    try:
        ensure_sim_supervisor(session_root, repository_root, timeout_s=10)
        previous = load_daemon_record(session_root)
        _terminate_identity(previous)

        client = ensure_sim_supervisor(session_root, repository_root, timeout_s=10)
        current = load_daemon_record(session_root)

        assert isinstance(client, SimulationSupervisorClient)
        assert current.process_identity != previous.process_identity
        assert current.process_identity.matches() is True
    finally:
        if current is not None:
            _terminate_identity(current)
        elif previous is not None:
            _terminate_identity(previous)


def test_live_daemon_with_broken_discovery_is_restarted(tmp_path: Path) -> None:
    session_root = (tmp_path / "session").resolve()
    repository_root = (tmp_path / "repository").resolve()
    session_root.mkdir()
    repository_root.mkdir()
    previous: SimulationSupervisorDaemonRecord | None = None
    current: SimulationSupervisorDaemonRecord | None = None
    try:
        ensure_sim_supervisor(session_root, repository_root, timeout_s=10)
        previous = load_daemon_record(session_root)
        (session_root / "supervisor" / "auth.key").unlink()

        client = ensure_sim_supervisor(session_root, repository_root, timeout_s=10)
        current = load_daemon_record(session_root)

        assert isinstance(client, SimulationSupervisorClient)
        assert current.process_identity != previous.process_identity
        assert previous.process_identity.matches() is False
        assert current.process_identity.matches() is True
    finally:
        if current is not None:
            _terminate_identity(current)
        elif previous is not None:
            _terminate_identity(previous)


def test_daemon_record_accepts_ordinary_json(tmp_path: Path) -> None:
    session_root = (tmp_path / "session").resolve()
    repository_root = (tmp_path / "repository").resolve()
    supervisor_dir = session_root / "supervisor"
    supervisor_dir.mkdir(parents=True)
    repository_root.mkdir()
    record = SimulationSupervisorDaemonRecord.create(
        repository_root=repository_root,
    )
    (supervisor_dir / "daemon.json").write_text(
        json.dumps(record.as_dict(), indent=2),
        encoding="utf-8",
    )

    assert load_daemon_record(session_root) == record


def test_failed_spawn_cleans_stale_discovery(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    session_root = (tmp_path / "session").resolve()
    repository_root = (tmp_path / "repository").resolve()
    supervisor_dir = session_root / "supervisor"
    supervisor_dir.mkdir(parents=True)
    repository_root.mkdir()
    (supervisor_dir / "endpoint.json").write_text("broken", encoding="utf-8")
    (supervisor_dir / "auth.key").write_bytes(b"broken")

    def unavailable_spawn(*_args: object, **_kwargs: object) -> None:
        raise OSError("unavailable")

    monkeypatch.setattr(daemon_module.subprocess, "Popen", unavailable_spawn)
    try:
        ensure_sim_supervisor(session_root, repository_root, timeout_s=1)
    except daemon_module.SimulationSupervisorDaemonError as exc:
        assert "could not be started" in str(exc)
    else:  # pragma: no cover - spawn is forced to fail
        raise AssertionError("expected the daemon spawn to fail")
    assert not (supervisor_dir / "endpoint.json").exists()
    assert not (supervisor_dir / "auth.key").exists()
