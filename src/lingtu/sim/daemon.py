"""Long-lived simulation supervisor daemon."""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import signal
import subprocess
import sys
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from lingtu.run_plan import RunPlan
from lingtu.sim.identity import ProcessIdentity, ProcessIdentityError
from lingtu.sim.process import SimProcessManager
from lingtu.sim.rpc import (
    LocalRpcServer,
    SupervisorProtocolError,
    SupervisorRequest,
    SupervisorResponse,
    load_connection,
    remove_discovery,
)
from lingtu.sim.supervisor import SimulationSupervisorClient, SimulationSupervisorError
from lingtu.switch_contracts import ProcessFailed

DAEMON_SCHEMA = "lingtu.sim_supervisor.daemon.v1"


class SimulationSupervisorDaemonError(RuntimeError):
    """Raised when the simulation supervisor daemon cannot be managed."""


class _SimulationRequestHandler:
    """Bind RPC requests to the daemon's one process manager."""

    def __init__(self, manager: SimProcessManager) -> None:
        self._manager = manager
        self._binding: tuple[RunPlan, str, str] | None = None

    def handle(self, request: SupervisorRequest) -> SupervisorResponse:
        try:
            plan = RunPlan.load(request.run_plan_path)
            binding = (plan, request.run_plan_path, request.product_session_id)
            if binding == self._binding:
                self._manager.assert_bound(plan)
            else:
                self._manager.bind(
                    plan,
                    run_plan_path=request.run_plan_path,
                    product_session_id=request.product_session_id,
                )
                self._binding = binding
            if request.action == "apply":
                report = self._manager.apply(plan)
            elif request.action == "quiesce":
                report = self._manager.quiesce(plan)
            else:
                report = self._manager.stop_plan(plan)
            return SupervisorResponse.ok(report.as_dict())
        except ProcessFailed as exc:
            return SupervisorResponse.ok(exc.report.as_dict())
        except Exception as exc:
            message = str(exc).strip() or type(exc).__name__
            return SupervisorResponse.failed(
                code="operation_failed",
                message=message[:512],
            )

    def close(self) -> None:
        if self._binding is None:
            return
        try:
            self._manager.stop_plan(self._binding[0])
        except Exception as exc:
            raise SimulationSupervisorError(
                "simulation process cleanup failed"
            ) from exc
        self._binding = None


@dataclass(frozen=True)
class SimulationSupervisorDaemonRecord:
    """Identity of one listening simulation supervisor daemon."""

    schema_version: str
    repository_root: str
    process_identity: ProcessIdentity

    def __post_init__(self) -> None:
        if self.schema_version != DAEMON_SCHEMA:
            raise SimulationSupervisorDaemonError(
                "unsupported simulation supervisor daemon schema"
            )
        if not isinstance(self.repository_root, str) or not Path(
            self.repository_root
        ).is_absolute():
            raise SimulationSupervisorDaemonError(
                "daemon repository_root must be an absolute path"
            )
        if not isinstance(self.process_identity, ProcessIdentity):
            raise SimulationSupervisorDaemonError("daemon process identity is invalid")

    @classmethod
    def create(
        cls,
        *,
        repository_root: Path,
    ) -> SimulationSupervisorDaemonRecord:
        return cls(
            schema_version=DAEMON_SCHEMA,
            repository_root=str(repository_root),
            process_identity=ProcessIdentity.current(),
        )

    @classmethod
    def from_dict(
        cls,
        payload: Mapping[str, Any],
    ) -> SimulationSupervisorDaemonRecord:
        try:
            identity = ProcessIdentity.from_dict(payload["process_identity"])
            schema_version = payload["schema_version"]
            repository_root = payload["repository_root"]
        except (KeyError, ProcessIdentityError, TypeError) as exc:
            raise SimulationSupervisorDaemonError(
                "daemon process identity is invalid"
            ) from exc
        return cls(
            schema_version=schema_version,
            repository_root=repository_root,
            process_identity=identity,
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "repository_root": self.repository_root,
            "process_identity": self.process_identity.as_dict(),
        }


def load_daemon_record(
    session_root: str | os.PathLike[str],
) -> SimulationSupervisorDaemonRecord:
    """Load the daemon identity for *session_root*."""

    root = _existing_directory(session_root, label="session_root")
    path = root / "supervisor" / "daemon.json"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise SimulationSupervisorDaemonError(
            "simulation supervisor daemon record is invalid"
        ) from exc
    if type(payload) is not dict:
        raise SimulationSupervisorDaemonError(
            "simulation supervisor daemon record must be a JSON object"
        )
    return SimulationSupervisorDaemonRecord.from_dict(payload)


def serve_supervisor(
    session_root: str | os.PathLike[str],
    repository_root: str | os.PathLike[str],
) -> None:
    """Own one process manager and serve requests until process exit."""

    session = _existing_directory(session_root, label="session_root")
    repository = _existing_directory(repository_root, label="repository_root")
    supervisor_dir = session / "supervisor"
    record_path = supervisor_dir / "daemon.json"
    handler = _SimulationRequestHandler(SimProcessManager(repository))
    try:
        with LocalRpcServer(session) as server:
            _write_record(
                record_path,
                SimulationSupervisorDaemonRecord.create(
                    repository_root=repository,
                ),
            )
            while True:
                try:
                    server.serve_once(handler.handle)
                except SupervisorProtocolError:
                    continue
    finally:
        try:
            handler.close()
        finally:
            record_path.unlink(missing_ok=True)
            try:
                supervisor_dir.rmdir()
            except OSError:
                pass


def ensure_sim_supervisor(
    session_root: str | os.PathLike[str],
    repository_root: str | os.PathLike[str],
    timeout_s: float = 10,
) -> SimulationSupervisorClient:
    """Reuse or start the supervisor daemon for one ProductControl session."""

    session = _existing_directory(session_root, label="session_root")
    repository = _existing_directory(repository_root, label="repository_root")
    deadline = time.monotonic() + _timeout(timeout_s)
    record_path = session / "supervisor" / "daemon.json"

    if record_path.exists():
        record = load_daemon_record(session)
        _require_repository(record, repository)
        if record.process_identity.matches():
            try:
                load_connection(session)
            except (OSError, RuntimeError) as exc:
                if not _terminate_exact_process(record.process_identity):
                    raise SimulationSupervisorDaemonError(
                        "live simulation supervisor has unusable discovery and could not be stopped"
                    ) from exc
                _clean_supervisor_files(session)
            else:
                return SimulationSupervisorClient(session)
    _clean_supervisor_files(session)

    child = _spawn_daemon(session, repository)
    try:
        while True:
            if record_path.exists():
                record = load_daemon_record(session)
                _require_repository(record, repository)
                if not record.process_identity.matches():
                    raise SimulationSupervisorDaemonError(
                        "simulation supervisor daemon exited before readiness"
                    )
                load_connection(session)
                return SimulationSupervisorClient(session)
            if child.poll() is not None:
                raise SimulationSupervisorDaemonError(
                    "simulation supervisor daemon exited before readiness"
                )
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SimulationSupervisorDaemonError(
                    "simulation supervisor daemon readiness timed out"
                )
            time.sleep(min(0.02, remaining))
    except Exception:
        _terminate_spawned_daemon(child)
        _clean_supervisor_files(session)
        raise


def _require_repository(
    record: SimulationSupervisorDaemonRecord,
    repository_root: Path,
) -> None:
    if record.repository_root != str(repository_root):
        raise SimulationSupervisorDaemonError(
            "simulation supervisor daemon state is inconsistent"
        )


def _spawn_daemon(
    session_root: Path,
    repository_root: Path,
) -> subprocess.Popen[bytes]:
    command = (
        sys.executable,
        "-m",
        "lingtu.sim.daemon",
        "serve",
        "--session-root",
        str(session_root),
        "--repository-root",
        str(repository_root),
    )
    creationflags = 0
    startupinfo: subprocess.STARTUPINFO | None = None
    if os.name == "nt":
        creationflags = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW
        startupinfo = subprocess.STARTUPINFO()
        startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startupinfo.wShowWindow = subprocess.SW_HIDE
    try:
        return subprocess.Popen(
            command,
            cwd=repository_root,
            shell=False,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=os.name != "nt",
            creationflags=creationflags,
            startupinfo=startupinfo,
        )
    except OSError as exc:
        raise SimulationSupervisorDaemonError(
            "simulation supervisor daemon could not be started"
        ) from exc


def _terminate_exact_process(identity: ProcessIdentity) -> bool:
    if not identity.matches():
        return True
    if os.name == "nt":
        return _terminate_windows_process(identity.pid)
    if os.name == "posix":
        return _terminate_posix_process(identity.pid)
    return False


def _terminate_windows_process(pid: int) -> bool:
    import ctypes
    from ctypes import wintypes

    process_terminate = 0x0001
    synchronize = 0x00100000
    wait_object_0 = 0
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    kernel32.OpenProcess.restype = wintypes.HANDLE
    kernel32.TerminateProcess.argtypes = [wintypes.HANDLE, wintypes.UINT]
    kernel32.TerminateProcess.restype = wintypes.BOOL
    kernel32.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    kernel32.WaitForSingleObject.restype = wintypes.DWORD
    kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel32.CloseHandle.restype = wintypes.BOOL
    handle = kernel32.OpenProcess(
        process_terminate | synchronize,
        False,
        pid,
    )
    if not handle:
        return False
    try:
        if not kernel32.TerminateProcess(handle, 1):
            return False
        return bool(kernel32.WaitForSingleObject(handle, 1000) == wait_object_0)
    finally:
        kernel32.CloseHandle(handle)


def _terminate_posix_process(pid: int) -> bool:
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        return False
    try:
        descriptor = os.pidfd_open(pid, 0)
    except OSError:
        return False
    try:
        term_signal = int(signal.SIGTERM)
        kill_signal = int(getattr(signal, "SIGKILL", 9))
        signal.pidfd_send_signal(descriptor, term_signal, None, 0)
        readable, _writable, _errors = select.select([descriptor], [], [], 1.0)
        if readable:
            return True
        signal.pidfd_send_signal(descriptor, kill_signal, None, 0)
        readable, _writable, _errors = select.select([descriptor], [], [], 1.0)
        return bool(readable)
    except OSError:
        return False
    finally:
        os.close(descriptor)


def _clean_supervisor_files(session_root: Path) -> None:
    supervisor_dir = session_root / "supervisor"
    remove_discovery(session_root)
    (supervisor_dir / "daemon.json").unlink(missing_ok=True)
    try:
        supervisor_dir.rmdir()
    except OSError:
        pass


def _terminate_spawned_daemon(child: subprocess.Popen[bytes]) -> None:
    if child.poll() is not None:
        return
    try:
        child.terminate()
        child.wait(timeout=1)
    except (OSError, subprocess.TimeoutExpired):
        if child.poll() is None:
            child.kill()
            try:
                child.wait(timeout=1)
            except (OSError, subprocess.TimeoutExpired):
                pass


def _write_record(path: Path, record: SimulationSupervisorDaemonRecord) -> None:
    temporary = path.with_suffix(".tmp")
    try:
        temporary.write_text(json.dumps(record.as_dict()), encoding="utf-8")
        os.replace(temporary, path)
    except OSError as exc:
        raise SimulationSupervisorDaemonError(
            "cannot publish simulation supervisor daemon record"
        ) from exc
    finally:
        temporary.unlink(missing_ok=True)


def _existing_directory(
    value: str | os.PathLike[str],
    *,
    label: str,
) -> Path:
    try:
        path = Path(value).resolve(strict=True)
    except OSError as exc:
        raise SimulationSupervisorDaemonError(f"{label} is unavailable") from exc
    if not path.is_absolute() or not path.is_dir():
        raise SimulationSupervisorDaemonError(
            f"{label} must be an existing absolute directory"
        )
    return path


def _timeout(value: float) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or value <= 0
    ):
        raise SimulationSupervisorDaemonError(
            "timeout_s must be a positive finite number"
        )
    return float(value)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m lingtu.sim.daemon")
    subparsers = parser.add_subparsers(dest="command", required=True)
    serve = subparsers.add_parser("serve")
    serve.add_argument("--session-root", required=True)
    serve.add_argument("--repository-root", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        serve_supervisor(args.session_root, args.repository_root)
    except (OSError, RuntimeError):
        return 2
    return 0


if __name__ == "__main__":  # pragma: no cover - exercised through python -m
    raise SystemExit(main())


__all__ = [
    "DAEMON_SCHEMA",
    "SimulationSupervisorDaemonError",
    "SimulationSupervisorDaemonRecord",
    "ensure_sim_supervisor",
    "load_daemon_record",
    "serve_supervisor",
]
