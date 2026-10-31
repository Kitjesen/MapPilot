"""Cached HTTP view of the native, read-only DDS stream observer."""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable, Mapping

_MAX_SNAPSHOT_BYTES = 256 * 1024
_SNAPSHOT_STALE_S = 2.0
_IDLE_CLOSE_S = 10.0


class NativeStreamMonitor:
    """Own one disposable inspection child, never a Product or recorder."""

    def __init__(
        self,
        *,
        recorder_binary: Callable[[], Path],
        environ: Mapping[str, str],
        process_factory: Any = subprocess.Popen,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._recorder_binary = recorder_binary
        self._environ = environ
        self._popen = process_factory
        self._monotonic = monotonic
        self._lock = threading.RLock()
        self._process: Any | None = None
        self._thread: threading.Thread | None = None
        self._snapshot: dict[str, Any] | None = None
        self._received_at: float | None = None
        self._requested_at = 0.0
        self._error: str | None = None
        self._failed_at: float | None = None
        self._closed = False

    def _start(self) -> None:
        from gateway.services.recording import NativeRecordingError

        recorder = self._recorder_binary()
        filename = "lingtu_dds_observer.exe" if os.name == "nt" else "lingtu_dds_observer"
        observer = recorder.parent / filename
        if not observer.is_file() or not os.access(observer, os.X_OK):
            raise NativeRecordingError(
                "native_stream_observer_unavailable", "native stream observer is not installed", status_code=503
            )
        environment = dict(os.environ)
        environment.update(self._environ)
        arguments = [
            str(observer), "--domain", str(self._environ.get("LINGTU_DDS_DOMAIN_ID", "0")),
            "--interval-ms", "500",
        ]
        self._process = self._popen(
            arguments,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=None,
            env=environment,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
        )
        self._snapshot = None
        self._received_at = None
        self._error = None
        self._failed_at = None
        self._thread = threading.Thread(
            target=self._read_snapshots,
            args=(self._process,),
            name="dds-stream-observer",
            daemon=True,
        )
        self._thread.start()

    @staticmethod
    def _stop_process(process: Any) -> None:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)

    def _read_snapshots(self, process: Any) -> None:
        try:
            while True:
                line = process.stdout.readline(_MAX_SNAPSHOT_BYTES + 1)
                if not line:
                    break
                if len(line) > _MAX_SNAPSHOT_BYTES:
                    raise ValueError("observer snapshot exceeds its response limit")
                payload = json.loads(line)
                if (
                    not isinstance(payload, dict)
                    or payload.get("version") != 1
                    or payload.get("ok") is not True
                    or not isinstance(payload.get("topics"), list)
                ):
                    raise ValueError("observer snapshot has an invalid contract")
                with self._lock:
                    if self._closed or self._monotonic() - self._requested_at > _IDLE_CLOSE_S:
                        break
                    if self._process is not process:
                        break
                    self._snapshot = payload
                    self._received_at = self._monotonic()
        except (ValueError, OSError):
            with self._lock:
                if self._process is process:
                    self._error = "native_stream_observer_protocol_error"
        finally:
            self._stop_process(process)
            if process.stdout is not None:
                process.stdout.close()
            with self._lock:
                if self._process is process:
                    self._process = None
                    if not self._closed and self._monotonic() - self._requested_at <= _IDLE_CLOSE_S:
                        self._error = self._error or "native_stream_observer_exited"
                        self._failed_at = self._monotonic()

    def snapshot(self) -> dict[str, Any]:
        from gateway.services.recording import NativeRecordingError

        with self._lock:
            self._requested_at = self._monotonic()
            if self._closed:
                return {"ok": True, "available": False, "state": "unavailable", "error": "observer_closed", "snapshot_age_ms": None, "topics": []}
            if self._process is None:
                if self._failed_at is not None and self._monotonic() - self._failed_at < 2.0:
                    return {"ok": True, "available": False, "state": "unavailable", "error": self._error, "snapshot_age_ms": None, "topics": []}
                try:
                    self._start()
                except (NativeRecordingError, OSError) as exc:
                    self._error = getattr(exc, "code", "native_stream_observer_unavailable")
                    return {"ok": True, "available": False, "state": "unavailable", "error": self._error, "snapshot_age_ms": None, "topics": []}
            if self._snapshot is None or self._received_at is None:
                return {"ok": True, "available": True, "state": "starting", "error": self._error, "snapshot_age_ms": None, "topics": []}
            age = max(0.0, self._monotonic() - self._received_at)
            return {
                **self._snapshot,
                "available": True,
                "state": "stale" if age > _SNAPSHOT_STALE_S else "observing",
                "snapshot_age_ms": age * 1000.0,
                "error": self._error,
            }

    def close(self) -> None:
        with self._lock:
            self._closed = True
            process, thread = self._process, self._thread
        if process is not None:
            self._stop_process(process)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
