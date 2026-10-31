"""HTTP-to-native adapter for recording control.

The C++ recorder owns catalog inspection, process identity, session state, DDS
capture, and MCAP persistence. This adapter only launches a native command and
projects its bounded, versioned JSON response into Gateway models.
"""

from __future__ import annotations

import json
import math
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
from lingtu.run_plan import RunPlan
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
    "invalid_recording_topics": 422,
    "recording_series_invalid_query": 422,
    "recording_series_unsupported_topic": 422,
    "recording_series_unsupported_field": 422,
    "recording_series_invalid_window": 422,
    "recording_series_unreadable": 422,
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


def _committed_host_recording_identity(
    environment: Mapping[str, str], state_dir: Path
) -> tuple[RunPlan, str]:
    run_plan_path = Path(str(environment.get("LINGTU_RUN_PLAN") or "").strip()).expanduser()
    if not run_plan_path.is_absolute():
        raise RuntimeError("LINGTU_RUN_PLAN must be an absolute path")
    declared_plan = RunPlan.load(run_plan_path)
    expected_session_id = str(environment.get("LINGTU_PRODUCT_SESSION_ID") or "").strip()
    if not is_product_session_id(expected_session_id):
        raise RuntimeError("Host Product session is invalid")
    plan, committed_path, committed_session_id = ProductControl(
        env=declared_plan.env,
        env_config={},
        process_env=environment,
    )._current_plan_and_path(state_dir)
    if committed_path.resolve() != run_plan_path.resolve():
        raise RuntimeError("Host RunPlan does not match committed current")
    if committed_session_id != expected_session_id:
        raise RuntimeError("Host Product session does not match committed current")
    return plan, expected_session_id


def _exact_inspection_recording_identity(
    environment: Mapping[str, str], state_dir: Path
) -> tuple[str, str]:
    plan, session_id = _committed_host_recording_identity(environment, state_dir)
    if plan.product != "inspection":
        raise RuntimeError("evidence recording requires the committed inspection Product")
    return plan.product, session_id


class NativeRecordingService:
    """Native recording adapter with an optional read-only stream monitor."""

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
        self._stream_monitor: Any | None = None

    def streams(self) -> dict[str, Any]:
        """Read observed DDS samples independently of recording selection."""
        from gateway.services.recording_streams import NativeStreamMonitor

        with self._lock:
            if self._stream_monitor is None:
                self._stream_monitor = NativeStreamMonitor(
                    recorder_binary=self._binary, environ=self._environ
                )
            monitor = self._stream_monitor
        return monitor.snapshot()

    def close_streams(self) -> None:
        """Release the optional diagnostic child on Gateway shutdown."""
        with self._lock:
            monitor = self._stream_monitor
        if monitor is not None:
            monitor.close()

    def _root(self) -> Path:
        value = self._environ.get("LINGTU_RECORDING_ROOT", "~/data/lingtu/recordings")
        return Path(os.path.expanduser(value)).resolve(strict=False)

    def _binary_candidates(self) -> tuple[Path, ...]:
        paths: list[Path] = []
        filename = "lingtu_recorder.exe" if os.name == "nt" else "lingtu_recorder"
        exact = str(self._environ.get("LINGTU_RECORDING_BIN", "")).strip()
        if exact:
            paths.append(Path(exact).expanduser())
        configured = str(self._environ.get("LINGTU_RECORDING_BIN_DIR", "")).strip()
        if configured:
            paths.append(Path(configured).expanduser() / filename)
        if os.name == "nt":
            paths.extend((
                self._repository_root / "build/native-recording/windows-x64/Release" / filename,
                self._repository_root / "build/native-recording/Release" / filename,
            ))
        else:
            paths.append(Path("/opt/lingtu/current/bin/lingtu_recorder"))
        paths.append(self._repository_root / "build/native-recording" / filename)
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
        version_key: str = "control_version",
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
        if not isinstance(payload, dict) or payload.get(version_key) != _CONTROL_VERSION:
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
                status_code=_ERROR_STATUS.get(code, 422 if code.startswith(("recording_scene_", "recording_camera_")) else 503),
                detail={"operation": operation, "exit_code": result.returncode},
            )
        return payload

    def catalog(self) -> dict[str, Any]:
        """Expose the installed native topic catalog, not a Python duplicate."""
        binary = self._binary()
        payload = self._decode_json_control(
            self._command([str(binary), "topics", "--json"], timeout_s=5.0),
            operation="topics",
        )
        if not isinstance(payload.get("topics"), list) or not isinstance(payload.get("presets"), list):
            raise NativeRecordingError(
                "native_recorder_protocol_error",
                "native recording topic catalog is missing topics or presets",
                status_code=503,
            )
        return {
            "topics": payload["topics"],
            "presets": payload["presets"],
            "camera": {
                "supported": os.name != "nt",
                "label": "相机视频",
                "source": "color_shared_memory",
            },
        }

    def _player_binary(self) -> Path:
        recorder = self._binary()
        filename = "lingtu_dds_player.exe" if os.name == "nt" else "lingtu_dds_player"
        player = recorder.parent / filename
        if not player.is_file() or not os.access(player, os.X_OK):
            raise NativeRecordingError(
                "native_player_unavailable", "native recording player is not installed", status_code=503
            )
        return player

    def inspect_artifact(self, artifact: Path, *, validate: bool = False) -> dict[str, Any]:
        """Read or validate a resolved DDS artifact without creating a publisher."""
        player = self._player_binary()
        arguments = (
            [str(player), str(artifact), "--dry-run", "--json"]
            if validate
            else [str(player), "--info", str(artifact), "--json"]
        )
        result = self._command(arguments, timeout_s=60.0 if validate else 10.0)
        if result.returncode != 0:
            raise NativeRecordingError(
                "recording_validation_failed" if validate else "recording_summary_failed",
                "native player could not validate the recording" if validate else "native player could not inspect the recording",
                status_code=422,
            )
        return self._decode_json_control(
            result, operation="validate" if validate else "info", version_key="version"
        )

    def series_artifact(
        self,
        artifact: Path,
        *,
        topic: str,
        field: str | None = None,
        start: float = 0.0,
        end: float | None = None,
        max_points: int = 2000,
    ) -> dict[str, Any]:
        """Read a bounded numeric series using native decoding, without DDS replay."""
        if not topic or not 16 <= max_points <= 10000:
            raise NativeRecordingError(
                "recording_series_invalid_query",
                "topic is required; max_points must be between 16 and 10000",
                status_code=422,
            )
        if not math.isfinite(start) or start < 0 or (
            end is not None and (not math.isfinite(end) or end < start)
        ):
            raise NativeRecordingError(
                "recording_series_invalid_window",
                "start and end must be finite seconds with 0 <= start <= end",
                status_code=422,
            )
        arguments = [
            str(self._player_binary()), "--series", str(artifact), "--topic", topic,
            "--start", str(start), "--max-points", str(max_points), "--json",
        ]
        if field is not None:
            arguments.extend(["--field", field])
        if end is not None:
            arguments.extend(["--end", str(end)])
        result = self._command(arguments, timeout_s=60.0)
        payload = self._decode_json_control(result, operation="series", version_key="version")
        if result.returncode != 0 or payload.get("mode") != "series":
            raise NativeRecordingError(
                "native_recorder_protocol_error",
                "native recording player did not return a numeric series",
                status_code=503,
            )
        return payload

    def scene_artifact(
        self, artifact: Path, *, info: bool = False, start: float = 0.0, end: float = 5.0,
        max_frames: int = 32, max_points: int = 2048, max_poses: int = 2000,
        cloud_topic: str | None = None, pose_topic: str | None = None,
        path_topics: tuple[str, ...] = (),
    ) -> dict[str, Any]:
        arguments = [str(self._player_binary()), "--scene-info" if info else "--scene", str(artifact), "--json"]
        if not info:
            arguments.extend([
                "--start", str(start), "--end", str(end), "--max-frames", str(max_frames),
                "--max-points", str(max_points), "--max-poses", str(max_poses),
            ])
            for option, value in (("--cloud-topic", cloud_topic), ("--pose-topic", pose_topic)):
                if value is not None:
                    arguments.extend([option, value])
            for topic in path_topics:
                arguments.extend(["--path-topic", topic])
        result = self._command(arguments, timeout_s=60.0)
        return self._decode_json_control(result, operation="scene-info" if info else "scene", version_key="version")

    def camera_index_artifact(self, artifact: Path, *, segment: int | None = None) -> dict[str, Any]:
        filename = "lingtu_camera_player.exe" if os.name == "nt" else "lingtu_camera_player"
        player = self._binary().parent / filename
        if not player.is_file() or not os.access(player, os.X_OK):
            raise NativeRecordingError(
                "native_camera_player_unavailable", "native camera reader is not installed", status_code=503,
            )
        arguments = [str(player), str(artifact), "--json"]
        if segment is not None:
            arguments.extend(["--segment", str(segment)])
        result = self._command(arguments, timeout_s=10.0)
        return self._decode_json_control(result, operation="camera-info", version_key="version")

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
        topics: list[str] | None = None,
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
        if topics is not None:
            if (
                not isinstance(topics, list)
                or (not topics and (not camera or capture_profile == "evidence"))
                or any(not isinstance(topic, str) for topic in topics)
            ):
                raise NativeRecordingError(
                    "invalid_recording_topics", "select DDS topics or enable camera-only sensor recording", status_code=422
                )
            if topics:
                supported = {item["topic"] for item in self.catalog()["topics"]}
                if any(topic not in supported for topic in topics):
                    raise NativeRecordingError(
                        "invalid_recording_topics", "selected topic is outside the native catalog", status_code=422
                    )
            topics = list(dict.fromkeys(topics))
        try:
            with ProductControlLock(
                environment=self._environ,
                timeout_s=0.0,
            ) as product_lock:
                product: str | None = None
                product_session_id: str | None = None
                robot_id: str | None = None
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
                elif self._environ.get("LINGTU_RUN_PLAN"):
                    try:
                        plan, product_session_id = _committed_host_recording_identity(
                            self._environ, product_lock.state_dir
                        )
                        product = plan.product
                        robot_id = self._environ.get("LINGTU_ROBOT_ID") or None
                    except RuntimeError as exc:
                        raise NativeRecordingError(
                            "recording_product_identity_invalid",
                            "recording Host identity does not match the committed Product",
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
                        "off" if topics == [] else "on",
                        "--camera",
                        "on" if camera else "off",
                        "--min-free-gib",
                        str(minimum_free_gib),
                    ]
                    if topics != []:
                        arguments.extend((
                            "--dds-preset", preset,
                            "--dds-domain", str(self._environ.get("LINGTU_DDS_DOMAIN_ID", "0")),
                        ))
                    if normalized_task_id is not None:
                        arguments.extend(("--inspection-task-id", normalized_task_id))
                    for topic in topics or []:
                        arguments.extend(("--dds-topic", topic))
                    if product is not None:
                        arguments.extend(("--product", product))
                    if product_session_id is not None:
                        arguments.extend(("--product-session-id", product_session_id))
                    if robot_id is not None:
                        arguments.extend(("--robot-id", robot_id))
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
