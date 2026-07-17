"""Bounded background JSONL writer for simulation diagnostics.

Diagnostic recording must not stall MuJoCo physics or native-DDS publication.
The producer therefore performs a non-blocking queue submission while one
owned worker serializes and flushes records in order.
"""

from __future__ import annotations

import json
import queue
import threading
from pathlib import Path
from typing import Any, Callable


class AsyncJsonlWriter:
    """Write JSON records in order without blocking the producer thread."""

    _STOP = object()

    def __init__(
        self,
        path: Path,
        *,
        max_pending: int = 2048,
        flush_every: int = 64,
        record_sink: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self.path = Path(path)
        self._flush_every = max(1, int(flush_every))
        self._record_sink = record_sink
        self._queue: queue.Queue[object] = queue.Queue(
            maxsize=max(1, int(max_pending))
        )
        self._lock = threading.Lock()
        self._submitted = 0
        self._written = 0
        self._dropped = 0
        self._failures = 0
        self._max_pending = 0
        self._first_error: BaseException | None = None
        self._closed = False
        self._worker = threading.Thread(
            target=self._run,
            name=f"jsonl-writer:{self.path.name}",
            daemon=True,
        )
        self._worker.start()

    def submit(self, record: dict[str, Any]) -> bool:
        """Queue one record, returning false instead of blocking when full."""

        with self._lock:
            if self._closed:
                raise RuntimeError("JSONL writer is closed")
            self._submitted += 1
        try:
            self._queue.put_nowait(record)
        except queue.Full:
            with self._lock:
                self._dropped += 1
            return False
        with self._lock:
            self._max_pending = max(self._max_pending, self._queue.qsize())
        return True

    def close(self, *, timeout_s: float = 60.0) -> dict[str, int]:
        """Drain queued records, close the file, and return diagnostics."""

        with self._lock:
            already_closed = self._closed
            self._closed = True
        if not already_closed:
            self._queue.put(self._STOP)
            self._worker.join(timeout=max(0.1, float(timeout_s)))
        if self._worker.is_alive():
            raise TimeoutError(f"JSONL writer did not stop: {self.path}")
        diagnostics = self.diagnostics()
        if self._first_error is not None:
            raise RuntimeError(
                f"JSONL writer failed for {self.path}: {self._first_error}"
            ) from self._first_error
        return diagnostics

    def diagnostics(self) -> dict[str, int]:
        with self._lock:
            return {
                "submitted": self._submitted,
                "written": self._written,
                "dropped": self._dropped,
                "failures": self._failures,
                "max_pending": self._max_pending,
            }

    def _record_failure(self, error: BaseException) -> None:
        with self._lock:
            self._failures += 1
            if self._first_error is None:
                self._first_error = error

    def _run(self) -> None:
        stream = None
        try:
            if self._record_sink is None:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                stream = self.path.open("w", encoding="utf-8")
        except BaseException as error:  # worker must keep draining its queue
            self._record_failure(error)

        writes_since_flush = 0
        while True:
            item = self._queue.get()
            if item is self._STOP:
                break
            record = item
            try:
                if not isinstance(record, dict):
                    raise TypeError("JSONL records must be dictionaries")
                if self._record_sink is not None:
                    self._record_sink(record)
                elif stream is not None:
                    stream.write(
                        json.dumps(record, separators=(",", ":")) + "\n"
                    )
                else:
                    raise OSError(f"JSONL output is unavailable: {self.path}")
                with self._lock:
                    self._written += 1
                writes_since_flush += 1
                if stream is not None and writes_since_flush >= self._flush_every:
                    stream.flush()
                    writes_since_flush = 0
            except BaseException as error:
                self._record_failure(error)
        if stream is not None:
            try:
                stream.flush()
                stream.close()
            except BaseException as error:
                self._record_failure(error)
