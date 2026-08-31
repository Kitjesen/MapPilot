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

from lingtu.control import ProductControl
from lingtu.product_lock import ProductControlBusy, ProductControlLock
from lingtu.switch_contracts import is_product_session_id

_ACTIVE_STATES = frozenset({"preparing", "recording", "stopping"})
_TERMINAL_STATES = frozenset({"completed", "failed"})
_ALL_STATES = _ACTIVE_STATES | _TERMINAL_STATES | {"idle"}
_MAX_CONTROL_RESPONSE_BYTES = 5 * 1024 * 1024
_CONTROL_VERSION = 1
_CAPTURE_PRESETS = {
    "sensors": "generic-sensors-v1",
    "evidence": "inspection-evidence-v1",
}

_ERROR_STATUS = {
    "multiple_recordings_active": 409,
    "recording_in_progress": 409,
    "recording_not_found": 404,
    "recording_session_invalid": 422,
    "recording_session_mismatch": 409,
    "recording_remove_failed": 503,
    "invalid_recording_limit": 422,
    "not_recording": 404,
    "recording_catalog_too_large": 503,
    "recording_catalog_unreadable": 503,
    "recording_catalog_unsafe": 503,
    "recording_manifest_invalid": 503,
    "recording_control_busy": 409,
    "native_recorder_start_failed": 503,
    "native_recorder_start_timeout": 504,
    "invalid_recording_capture_profile": 422,
    "invalid_recording_camera": 422,
    "invalid_recording_minimum_free_gib": 422,
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
    product_session_id: str = ""

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


def _exact_inspection_recording_identity(
    environment: Mapping[str, str], state_dir: Path
) -> tuple[str, str]:
    env = str(environment.get("LINGTU_ENV") or "").strip()
    if env not in {"real", "sim"}:
        raise RuntimeError("LINGTU_ENV must be real or sim")
    expected_session_id = str(environment.get("LINGTU_PRODUCT_SESSION_ID") or "").strip()
    if not is_product_session_id(expected_session_id):
        raise RuntimeError("Host Product session is invalid")
    plan, _plan_path, committed_session_id = ProductControl(
        env=env,
        env_config={},
        process_env=environment,
    )._current_plan_and_path(state_dir)
    if plan.product != "inspection":
        raise RuntimeError(
            "evidence recording requires the committed inspection Product"
        )
    if committed_session_id != expected_session_id:
        raise RuntimeError("Host Product session does not match committed current")
    return plan.product, expected_session_id


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
        # A persisted session cause is more specific than the catalog-level error.
        # Fall back to the latter for stale active managers without a session error.
        native_error = session.get("error") or payload.get("error")
        context = session.get("context")
        product_session_id = (
            str(context.get("product_session_id") or "")
            if isinstance(context, Mapping)
            else ""
        )
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
            product_session_id=product_session_id,
        )

    def _decode_json_control(
        self,
        result: subprocess.CompletedProcess[str],
        *,
        operation: str,
    ) -> dict[str, Any]:
        """Decode a bounded native catalog response without owning its state."""
        output = result.stdout if isinstance(result.stdout, str) else ""
        if not output or len(output.encode("utf-8", errors="replace")) > _MAX_CONTROL_RESPONSE_BYTES:
            raise NativeRecordingError(
                "native_recorder_protocol_error",
                "native recording catalog response is missing or exceeds its hard limit",
                status_code=503,
                detail={"operation": operation, "exit_code": result.returncode},
            )
        try:
            payload = json.loads(output)
        except json.JSONDecodeError as exc:
            raise NativeRecordingError(
                "native_recorder_protocol_error",
                "native recording catalog returned invalid JSON",
                status_code=503,
                detail={"operation": operation, "exit_code": result.returncode},
            ) from exc
        if not isinstance(payload, dict) or payload.get("control_version") != _CONTROL_VERSION:
            raise NativeRecordingError(
                "native_recorder_protocol_error",
                "native recording catalog version is missing or unsupported",
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
        return payload

    @staticmethod
    def _validate_session_id(session_id: str) -> str:
        value = str(session_id or "")
        if (
            not value
            or len(value) > 128
            or not value[0].isascii()
            or not value[0].isalnum()
            or any(
                not (character.isascii() and (character.isalnum() or character in "-_."))
                for character in value
            )
        ):
            raise NativeRecordingError(
                "recording_session_invalid",
                "recording session id must use letters, digits, '.', '-' or '_'",
                status_code=422,
            )
        return value

    def list(self, *, limit: int = 100) -> dict[str, Any]:
        """Return a bounded native-owned session catalog projection."""
        if not 1 <= limit <= 256:
            raise NativeRecordingError(
                "invalid_recording_limit",
                "recording list limit must be between 1 and 256",
                status_code=422,
            )
        with self._lock:
            binary = self._binary()
            payload = self._decode_json_control(
                self._command(
                    [str(binary), "list", "--root", str(self._root()), "--limit", str(limit)],
                    timeout_s=5.0,
                ),
                operation="list",
            )
            sessions = payload.get("sessions")
            if not isinstance(sessions, list):
                raise NativeRecordingError(
                    "native_recorder_protocol_error",
                    "native recording catalog omitted its sessions list",
                    status_code=503,
                    detail={"operation": "list"},
                )
            return {
                "sessions": [item for item in sessions if isinstance(item, dict)],
                "truncated": payload.get("truncated") is True,
                "disk_free": self._bounded_int(payload, "disk_free") or 0,
                "disk_total": self._bounded_int(payload, "disk_total") or 0,
            }

    def manifest(self, *, session_id: str) -> dict[str, Any]:
        """Return one native-validated session manifest."""
        normalized_id = self._validate_session_id(session_id)
        with self._lock:
            binary = self._binary()
            payload = self._decode_json_control(
                self._command(
                    [
                        str(binary),
                        "manifest",
                        "--root",
                        str(self._root()),
                        "--session-id",
                        normalized_id,
                    ],
                    timeout_s=5.0,
                ),
                operation="manifest",
            )
            session = payload.get("session")
            if not isinstance(session, dict) or session.get("session_id") != normalized_id:
                raise NativeRecordingError(
                    "native_recorder_protocol_error",
                    "native recording manifest identity is invalid",
                    status_code=503,
                    detail={"operation": "manifest"},
                )
            return session

    def remove(self, *, session_id: str) -> dict[str, Any]:
        """Remove one terminal session through the native root lock."""
        normalized_id = self._validate_session_id(session_id)
        with self._lock:
            binary = self._binary()
            return self._decode_json_control(
                self._command(
                    [
                        str(binary),
                        "remove",
                        "--root",
                        str(self._root()),
                        "--session-id",
                        normalized_id,
                    ],
                    timeout_s=10.0,
                ),
                operation="remove",
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

    def start(
        self,
        *,
        duration: int,
        prefix: str,
        capture_profile: str = "sensors",
        task_id: str | None = None,
        camera: bool = False,
        minimum_free_gib: int = 5,
    ) -> RecordingSnapshot:
        if not 0 <= duration <= 86_400:
            raise NativeRecordingError(
                "invalid_recording_duration",
                "recording duration must be between 0 and 86400 seconds",
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
        preset = _CAPTURE_PRESETS.get(capture_profile)
        if preset is None:
            raise NativeRecordingError(
                "invalid_recording_capture_profile",
                "capture profile must be 'sensors' or 'evidence'",
                status_code=422,
            )
        normalized_task_id = task_id.strip() if isinstance(task_id, str) else None
        task_binding_valid = (
            capture_profile == "evidence"
            and normalized_task_id is not None
            and "\x00" not in normalized_task_id
            and 1 <= len(normalized_task_id.encode("utf-8")) <= 256
        ) or (capture_profile == "sensors" and task_id is None)
        if not task_binding_valid:
            raise NativeRecordingError(
                "invalid_recording_task_id",
                "a non-empty task_id is required only for evidence recording",
                status_code=422,
            )
        if type(camera) is not bool:
            raise NativeRecordingError(
                "invalid_recording_camera",
                "camera must be a boolean",
                status_code=422,
            )
        if (
            not isinstance(minimum_free_gib, int)
            or isinstance(minimum_free_gib, bool)
            or not 1 <= minimum_free_gib <= 100
        ):
            raise NativeRecordingError(
                "invalid_recording_minimum_free_gib",
                "minimum free space must be between 1 and 100 GiB",
                status_code=422,
            )
        binary = self._binary()
        try:
            with ProductControlLock(
                environment=self._environ,
                timeout_s=0.0,
            ) as product_lock:
                product: str | None = None
                product_session_id: str | None = None
                if capture_profile == "evidence":
                    try:
                        product, product_session_id = (
                            _exact_inspection_recording_identity(
                                self._environ, product_lock.state_dir
                            )
                        )
                    except RuntimeError as exc:
                        raise NativeRecordingError(
                            "inspection_recording_identity_required",
                            "evidence recording requires the exact committed inspection Product identity",
                            status_code=409,
                        ) from exc
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
                        "--dds-preset",
                        preset,
                        "--camera",
                        "on" if camera else "off",
                        "--min-free-gib",
                        str(minimum_free_gib),
                    ]
                    if normalized_task_id is not None:
                        arguments.extend(("--inspection-task-id", normalized_task_id))
                    if product is not None:
                        arguments.extend(("--product", product))
                    if product_session_id is not None:
                        arguments.extend(("--product-session-id", product_session_id))
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

    def stop(
        self,
        *,
        timeout_ms: int = 15_000,
        expected_session_id: str | None = None,
    ) -> RecordingSnapshot:
        if not 0 <= timeout_ms <= 300_000:
            raise NativeRecordingError(
                "invalid_recording_stop_timeout",
                "recording stop timeout must be between 0 and 300000 milliseconds",
                status_code=422,
            )
        with self._lock:
            binary = self._binary()
            arguments = [
                str(binary),
                "stop",
                "--root",
                str(self._root()),
                "--timeout-ms",
                str(timeout_ms),
            ]
            if expected_session_id is not None:
                arguments.extend(
                    ("--expected-session-id", self._validate_session_id(expected_session_id))
                )
            result = self._command(
                arguments,
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
