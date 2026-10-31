"""Raw-measurement endpoint for the existing session and recording owners."""

from __future__ import annotations

from collections.abc import Callable

from sim.runtime.sensors.acoustics import AcousticFrame
from sim.runtime.sensors.semantic_camera import SemanticCameraSample
from sim.runtime.sensors.thermal import ThermalFrame

from .measurement_payload import MeasurementSample, encode_measurement_payload
from .timeline import SensorPayloadSample


class MeasurementSink:
    """Keep one latest raw sample; forward enabled recordings to the run owner.

    No threads, files or process lifecycle are created. The callback must persist
    or stage the payload for the current truth-snapshot transaction and propagate
    failures. The caller must not interpret successful staging as a disk commit.
    """

    def __init__(
        self, sensor_id: str, modality: str, *, on_record: Callable[[SensorPayloadSample], None] | None = None
    ):
        if modality not in {"thermal", "acoustic", "semantic"}:
            raise ValueError("unsupported measurement modality")
        if not sensor_id or sensor_id != sensor_id.strip():
            raise ValueError("sensor_id must be a nonempty name")
        if on_record is not None and not callable(on_record):
            raise ValueError("recording callback must be callable")
        self.sensor_id, self.modality, self.on_record = sensor_id, modality, on_record
        self.latest: MeasurementSample | None = None
        self.recording = False
        self._started = self._closed = False

    def start(self) -> dict:
        if self._closed or self._started:
            raise RuntimeError("measurement sink already started or closed")
        self._started = True
        return {"ready": True, "transport": "in_process", "encoding": "npz", "modality": self.modality}

    def publish(self, sample: MeasurementSample) -> None:
        self._require_open()
        expected_type = {"thermal": ThermalFrame, "acoustic": AcousticFrame, "semantic": SemanticCameraSample}
        if not isinstance(sample, expected_type[self.modality]) or sample.stamp.sensor_id != self.sensor_id:
            raise ValueError("measurement sample type or sensor identity differs from sink")
        if self.recording:
            assert self.on_record is not None
            self.on_record(encode_measurement_payload(sample, modality=self.modality))
        self.latest = sample

    def set_recording(self, enabled: bool) -> None:
        self._require_open()
        if type(enabled) is not bool:
            raise ValueError("recording switch must be boolean")
        if enabled and self.on_record is None:
            raise ValueError("measurement endpoint has no recording owner")
        self.recording = enabled

    def close(self) -> None:
        self.latest = None
        self.recording = False
        self._closed = True

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("measurement sink is closed")
        if not self._started:
            raise RuntimeError("measurement sink is not started")
