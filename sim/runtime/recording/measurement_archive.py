"""Lazy, bounded raw measurement archive within one Product-owned run directory."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

from .timeline import SensorPayloadSample


class MeasurementArchive:
    """The sink calls append only while recording is enabled; no new worker.

    Index entries appear only after a complete raw file write. This does not
    fabricate truth snapshots or a legacy RunAllocation for the timeline writer.
    Capacity exhaustion raises rather than silently dropping requested evidence.
    """

    def __init__(self, root: Path, *, product_session_id: str, session_id: str,
                 sensor_ids: list[str], max_bytes: int = 512 * 1024 * 1024):
        if not product_session_id or not session_id or not sensor_ids:
            raise ValueError("measurement recording requires explicit run identity and sensors")
        if type(max_bytes) is not int or max_bytes <= 0:
            raise ValueError("recording byte limit must be positive")
        self.root, self.product_session_id, self.session_id = Path(root), product_session_id, session_id
        self._sensors, self._max_bytes = frozenset(sensor_ids), max_bytes
        self._count = self._bytes = 0
        self._last: dict[tuple[str, int, int], int] = {}
        self._index: Any = None
        self._closed = False
        self._error: str | None = None

    def append(self, sample: SensorPayloadSample) -> None:
        if self._closed:
            raise RuntimeError("measurement recording is closed")
        stamp = (sample.metadata or {}).get("stamp", {})
        if (stamp.get("session_id") != self.session_id or sample.sensor_id not in self._sensors
                or stamp.get("sensor_id") != sample.sensor_id or stamp.get("sequence") != sample.sample_sequence
                or stamp.get("sim_time_ns") != sample.sample_time_ns or sample.encoding != "npz"):
            raise ValueError("measurement recording identity is inconsistent")
        key = (sample.sensor_id, stamp["model_generation"], stamp["reset_generation"])
        if sample.sample_sequence <= self._last.get(key, -1):
            raise ValueError("measurement recording sequence must advance")
        if self._bytes + len(sample.payload) > self._max_bytes:
            self._error = "recording_capacity_exceeded"
            raise RuntimeError(self._error)
        try:
            if self._index is None:
                self.root.mkdir(parents=True, exist_ok=True)
                self._index = (self.root / "index.jsonl").open("x", encoding="utf-8")
            filename = f"sample-{self._count:09d}.npz"
            with (self.root / filename).open("xb") as file:
                file.write(sample.payload)
            entry = dict(file=filename, sensor_id=sample.sensor_id, modality=sample.stream_kind,
                         stamp=stamp, product_session_id=self.product_session_id, bytes=len(sample.payload))
            self._index.write(json.dumps(entry, allow_nan=False) + "\n")
            self._index.flush()
            self._count += 1
            self._bytes += len(sample.payload)
            self._last[key] = sample.sample_sequence
            self._save_summary("recording")
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            raise

    def _save_summary(self, state: str) -> None:
        document = dict(schema="lingtu.measurement-recording.v1", product_session_id=self.product_session_id,
                        session_id=self.session_id, state=state, sample_count=self._count,
                        payload_bytes=self._bytes, max_payload_bytes=self._max_bytes, error=self._error)
        temporary = self.root / "recording.json.tmp"
        temporary.write_text(json.dumps(document, allow_nan=False, indent=2), encoding="utf-8")
        os.replace(temporary, self.root / "recording.json")

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            if self._index is not None:
                try:
                    self._index.close()
                finally:
                    self._save_summary("failed" if self._error else "closed")
