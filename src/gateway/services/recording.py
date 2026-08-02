"""HTTP-to-native adapter for recording control.

The C++ recorder owns catalog inspection, process identity, session state, DDS
capture, and MCAP persistence. This adapter only launches a native command and
projects its bounded, versioned JSON response into Gateway models.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from lingtu.product_lock import ProductControlBusy, ProductControlLock

_ACTIVE_STATES = frozenset({"preparing", "recording", "stopping"})
_TERMINAL_STATES = frozenset({"completed", "failed"})
_ALL_STATES = _ACTIVE_STATES | _TERMINAL_STATES | {"idle"}
_MAX_CONTROL_RESPONSE_BYTES = 5 * 1024 * 1024
_CONTROL_VERSION = 1

_ERROR_STATUS = {
    "multiple_recordings_active": 409,
    "recording_in_progress": 409,
    "not_recording": 404,
    "recording_catalog_too_large": 503,
    "recording_catalog_unreadable": 503,
    "recording_catalog_unsafe": 503,
    "recording_manifest_invalid": 503,
    "recording_control_busy": 409,
    "native_recorder_start_failed": 503,
    "native_recorder_start_timeout": 504,
}


class NativeRecordingError(RuntimeError):
    """Fail-closed error at the native recording boundary."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        status_code: int = 500,
        detail: Mapping[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.status_code = status_code
        self.detail = dict(detail or {})


@dataclass(frozen=True)
class RecordingSnapshot:
    available: bool
    healthy: bool
    state: str
    session_id: str = ""
    path: str = ""
    pid: int | None = None
    started_ns: int | None = None
    ended_ns: int | None = None
    error: str | None = None
    exit_code: int | None = None
    size_bytes: int = 0
    size_truncated: bool = False
    disk_free: int = 0
    disk_total: int = 0

    @property
    def recording(self) -> bool:
        return self.healthy and self.state in _ACTIVE_STATES

    def as_payload(self) -> dict[str, Any]:
        duration_s = 0.0
        if self.started_ns is not None:
            duration_s = max(
                0.0,
                ((self.ended_ns or time.time_ns()) - self.started_ns) / 1_000_000_000.0,
            )
        return {
            "available": self.available,
            "healthy": self.healthy,
            "backend": "native_mcap",
            "state": self.state,
            "session_id": self.session_id or None,
            "recording": self.recording,
            "path": self.path or None,
            "duration_s": duration_s,
            "size_bytes": self.size_bytes,
            "size_truncated": self.size_truncated,
            "pid": self.pid,
            "exit_code": self.exit_code,
            "disk_free": self.disk_free,
            "disk_total": self.disk_total,
            "error": self.error,
        }


class NativeRecordingService:
    """Stateless adapter over the native recording control contract."""

    def __init__(
        self,
        *,
        repository_root: Path | None = None,
        environ: Mapping[str, str] | None = None,
        runner: Any | None = None,
        startup_timeout_s: float = 5.0,
    ) -> None:
        self._repository_root = repository_root or Path(__file__).resolve().parents[3]
        self._environ = os.environ if environ is None else environ
        self._run = runner or subprocess.run
        self._startup_timeout_s = startup_timeout_s
        self._lock = threading.RLock()

    def _root(self) -> Path:
        value = self._environ.get("LINGTU_RECORDING_ROOT", "~/data/lingtu/recordings")
        return Path(os.path.expanduser(value)).resolve(strict=False)

    def _binary_candidates(self) -> tuple[Path, ...]:
        paths: list[Path] = []
        exact = str(self._environ.get("LINGTU_RECORDING_BIN", "")).strip()
        if exact:
            paths.append(Path(exact).expanduser())
        configured = str(self._environ.get("LINGTU_RECORDING_BIN_DIR", "")).strip()
        if configured:
            paths.append(Path(configured).expanduser() / "lingtu_recorder")
        paths.extend(
            (
                Path("/opt/lingtu/current/build/native-recording/lingtu_recorder"),
                self._repository_root / "build" / "native-recording" / "lingtu_recorder",
            )
        )
        discovered = shutil.which("lingtu_recorder")
        if discovered:
            paths.append(Path(discovered))
        return tuple(paths)

    def _binary(self, *, required: bool = True) -> Path | None:
        for path in self._binary_candidates():
            try:
                if path.is_file() and os.access(path, os.X_OK):
                    return path.resolve(strict=True)
            except OSError:
                continue
        if not required:
            return None
        raise NativeRecordingError(
            "native_recorder_unavailable",
            "native recording binary is not installed",
            status_code=503,
            detail={"expected": [str(path) for path in self._binary_candidates()]},
        )

    def _command(self, arguments: list[str], *, timeout_s: float) -> subprocess.CompletedProcess[str]:
        try:
            return self._run(
                arguments,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=timeout_s,
                check=False,
            )
        except FileNotFoundError as exc:
            raise NativeRecordingError(
                "native_recorder_unavailable",
                "native recording binary disappeared before invocation",
                status_code=503,
            ) from exc
        except subprocess.TimeoutExpired as exc:
            raise NativeRecordingError(
                "native_recorder_timeout",
                "native recording command timed out",
                status_code=504,
                detail={"command": arguments[1]},
            ) from exc
        except OSError as exc:
            raise NativeRecordingError(
                "native_recorder_invocation_failed",
                f"native recording command could not start: {exc}",
                status_code=503,
            ) from exc

    @staticmethod
    def _bounded_int(payload: Mapping[str, Any], key: str) -> int | None:
        value = payload.get(key)
        if isinstance(value, int) and not isinstance(value, bool) and value >= 0:
            return value
        return None

    def _decode_control(
        self,
        result: subprocess.CompletedProcess[str],
        *,
        operation: str,
    ) -> RecordingSnapshot:
        output = result.stdout if isinstance(result.stdout, str) else ""
        if not output or len(output.encode("utf-8", errors="replace")) > _MAX_CONTROL_RESPONSE_BYTES:
            raise NativeRecordingError(
                "native_recorder_protocol_error",
                "native recording control response is missing or exceeds its hard limit",
                status_code=503,
                detail={"operation": operation, "exit_code": result.returncode},
            )
        try:
            payload = json.loads(output)
        except json.JSONDecodeError as exc:
            raise NativeRecordingError(
                "native_recorder_protocol_error",
                "native recording control returned invalid JSON",
                status_code=503,
                detail={"operation": operation, "exit_code": result.returncode},
            ) from exc
        if not isinstance(payload, dict) or payload.get("control_version") != _CONTROL_VERSION:
            raise NativeRecordingError(
                "native_recorder_protocol_error",
                "native recording control version is missing or unsupported",
                status_code=503,
                detail={"operation": operation},
            )
        if payload.get("ok") is not True:
            error = payload.get("error")
            code = (
                str(error.get("code") or "native_recorder_failed")
                if isinstance(error, dict)
                else "native_recorder_failed"
            )
            message = (
                str(error.get("message") or "native recording command failed")
                if isinstance(error, dict)
                else "native recording command failed"
            )
            raise NativeRecordingError(
                code,
                message,
                status_code=_ERROR_STATUS.get(code, 503),
                detail={"operation": operation, "exit_code": result.returncode},
            )

        state = str(payload.get("state") or "")
        healthy = payload.get("healthy")
        session = payload.get("session")
        if state not in _ALL_STATES or not isinstance(healthy, bool):
            raise NativeRecordingError(
                "native_recorder_protocol_error",
                "native recording control returned invalid state fields",
                status_code=503,
                detail={"operation": operation, "state": state},
            )
        if state == "idle":
            if session is not None:
                raise NativeRecordingError(
                    "native_recorder_protocol_error",
                    "idle native recording response unexpectedly contains a session",
                    status_code=503,
                )
            return RecordingSnapshot(
                available=True,
                healthy=healthy,
                state=state,
                exit_code=result.returncode or None,
                disk_free=self._bounded_int(payload, "disk_free") or 0,
                disk_total=self._bounded_int(payload, "disk_total") or 0,
                error=str(payload.get("error")) if payload.get("error") else None,
            )
        if not isinstance(session, dict):
            raise NativeRecordingError(
                "native_recorder_protocol_error",
                "native recording response omitted its authoritative session",
                status_code=503,
                detail={"operation": operation, "state": state},
            )
        session_id = session.get("session_id")
        path = session.get("session_directory")
        pid = session.get("manager_pid")
        if (
            not isinstance(session_id, str)
            or not session_id
            or not isinstance(path, str)
            or not path
            or not isinstance(pid, int)
            or isinstance(pid, bool)
            or pid <= 1
        ):
            raise NativeRecordingError(
                "native_recorder_protocol_error",
                "native recording session identity is invalid",
                status_code=503,
                detail={"operation": operation, "state": state},
            )
        native_error = payload.get("error") or session.get("error")
        return RecordingSnapshot(
            available=True,
            healthy=healthy and state != "failed",
            state=state,
            session_id=session_id,
            path=path,
            pid=pid,
            started_ns=self._bounded_int(session, "started_at_unix_ns"),
            ended_ns=self._bounded_int(session, "ended_at_unix_ns"),
            error=str(native_error) if native_error else None,
            exit_code=result.returncode or None,
            size_bytes=self._bounded_int(payload, "size_bytes") or 0,
            size_truncated=payload.get("size_truncated") is True,
            disk_free=self._bounded_int(payload, "disk_free") or 0,
            disk_total=self._bounded_int(payload, "disk_total") or 0,
        )

    def _native_status(self, binary: Path) -> RecordingSnapshot:
        result = self._command(
            [str(binary), "status", "--root", str(self._root())],
            timeout_s=3.0,
        )
        return self._decode_control(result, operation="status")

    def status(self) -> RecordingSnapshot:
        with self._lock:
            binary = self._binary(required=False)
            if binary is None:
                return RecordingSnapshot(available=False, healthy=True, state="idle")
            return self._native_status(binary)

    def start(self, *, duration: int, prefix: str) -> RecordingSnapshot:
        if not 1 <= duration <= 86_400:
            raise NativeRecordingError(
                "invalid_recording_duration",
                "recording duration must be between 1 and 86400 seconds",
                status_code=422,
            )
        if (
            not prefix
            or len(prefix) > 40
            or any(
                not (
                    character.isascii()
                    and (character.isalnum() or character in "-_")
                )
                for character in prefix
            )
        ):
            raise NativeRecordingError(
                "invalid_recording_prefix",
                "recording prefix must use 1-40 ASCII letters, digits, '-' or '_'",
                status_code=422,
            )
        binary = self._binary()
        try:
            with ProductControlLock(environment=self._environ, timeout_s=0.0):
                with self._lock:
                    arguments = [
                        str(binary),
                        "start",
                        "--root",
                        str(self._root()),
                        "--prefix",
                        prefix,
                        "--startup-timeout-ms",
                        str(max(1, int(self._startup_timeout_s * 1000))),
                        "--seconds",
                        str(duration),
                        "--dds",
                        "on",
                        "--camera",
                        "off",
                    ]
                    product = str(self._environ.get("LINGTU_PRODUCT", "")).strip()
                    fingerprint = str(
                        self._environ.get("LINGTU_RUN_PLAN_FINGERPRINT", "")
                    ).strip()
                    if product:
                        arguments.extend(("--product", product))
                    if fingerprint:
                        arguments.extend(("--run-plan-fingerprint", fingerprint))
                    result = self._command(
                        arguments,
                        timeout_s=self._startup_timeout_s + 2.0,
                    )
                    snapshot = self._decode_control(result, operation="start")
                    if (
                        result.returncode != 0
                        or snapshot.state != "recording"
                        or not snapshot.healthy
                    ):
                        raise NativeRecordingError(
                            "native_recorder_start_failed",
                            snapshot.error
                            or "native recording manager did not become ready",
                            status_code=503,
                            detail=snapshot.as_payload(),
                        )
                    return snapshot
        except ProductControlBusy as exc:
            raise NativeRecordingError(
                "product_transition_in_progress",
                "recording cannot start during a Product transition",
                status_code=409,
            ) from exc

    def stop(self, *, timeout_ms: int = 15_000) -> RecordingSnapshot:
        if not 0 <= timeout_ms <= 300_000:
            raise NativeRecordingError(
                "invalid_recording_stop_timeout",
                "recording stop timeout must be between 0 and 300000 milliseconds",
                status_code=422,
            )
        with self._lock:
            binary = self._binary()
            result = self._command(
                [
                    str(binary),
                    "stop",
                    "--root",
                    str(self._root()),
                    "--timeout-ms",
                    str(timeout_ms),
                ],
                timeout_s=(timeout_ms / 1000.0) + 2.0,
            )
            snapshot = self._decode_control(result, operation="stop")
            if result.returncode != 0 or snapshot.state != "completed":
                raise NativeRecordingError(
                    "native_recorder_stop_failed",
                    snapshot.error or "native recording session did not complete cleanly",
                    detail=snapshot.as_payload(),
                )
            return snapshot
