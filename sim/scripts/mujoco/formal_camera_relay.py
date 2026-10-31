"""Relay owned UE camera captures into the existing Product camera endpoint.

The image's latched truth stamp selects its clock join; polling time never
becomes capture time. This component neither renders nor advances physics.
"""

from __future__ import annotations

import math
import queue
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from sim.adapters.shm.camera_ring import (
    CameraShmAllocation,
    CameraShmChanged,
    CameraShmNotReady,
    CameraShmReader,
    CameraShmStale,
    FrameSnapshot,
    StreamKind,
    WindowsNamedMappingBackend,
)
from sim.scripts.mujoco.native_sensor_records import (
    CAMERA_RECORD_HEADER,
    encode_camera_depth,
    encode_camera_intrinsics,
    encode_camera_rgb,
)

from drivers.real.camera.shm import StreamKind as OutputKind


@dataclass(frozen=True)
class _Capture:
    kind: str
    frame: FrameSnapshot
    epoch_s: float
    counted: bool


class FormalCameraRelay:
    """One bounded copy/publish worker, using only explicitly allocated mappings."""

    def __init__(self, *, sensor_plan: dict, allocation: Any, product_session_id: str,
                 model_generation: int, reset_generation: int, client: Any, services: Any,
                 session_root: Path, stream_stats: dict, mapping_backend: Any = None,
                 slot_capacity: int = 8 * 1024 * 1024) -> None:
        self._session = sensor_plan["session_id"]
        if allocation.session_id != self._session:
            raise ValueError("camera allocation does not belong to SensorPlan session")
        self._product, self._model, self._reset = product_session_id, model_generation, reset_generation
        self._allocation = allocation
        self._backend = mapping_backend or WindowsNamedMappingBackend()
        self._capacity, self._client = slot_capacity, client
        self._wall_time = services.wall_time
        self._specs = {kind: rows[0] for kind in ("rgb", "depth")
                       if (rows := sensor_plan["streams"].get(kind, []))}
        for kind, spec in self._specs.items():
            if (spec["owner"], spec["source"], spec["transport"]) != ("visual", "unreal_camera", "camera_shm"):
                raise ValueError("camera relay requires the declared UE-owned source")
            if len(sensor_plan["streams"][kind]) != 1 or spec["sensor_id"] not in allocation.shm:
                raise ValueError("camera relay requires one allocated stream per image kind")
        if not self._specs:
            raise ValueError("camera relay requires at least one selected UE camera")
        geometry = {(s["frame_id"], s["width"], s["height"]) for s in self._specs.values()}
        if len(geometry) != 1:
            raise ValueError("Product RGB-D endpoint requires one shared optical frame and resolution")
        self._stats = stream_stats
        self._mappings: dict[str, Any] = {}
        self._readers: dict[str, CameraShmReader] = {}
        self._source_sequences = {kind: 0 for kind in self._specs}
        self._history: OrderedDict[int, tuple[int, float]] = OrderedDict()
        self._last_source: dict[str, dict] = {}
        self._active: set[str] = set()
        self._begun = False
        self._last_observed: dict[str, float] = {}
        self._started_s = 0.0
        self._observed_s = 0.0
        self._closed = False
        self._failure: BaseException | None = None
        self._sequence = 0
        self._calibration: tuple | None = None
        self._jobs: queue.Queue[tuple[_Capture, ...] | None] = queue.Queue(maxsize=1)
        self._writers: dict[str, Any] = {}
        source_generation = time.time_ns()
        try:
            spec = next(iter(self._specs.values()))
            for kind, output, stream_kind, capacity in (
                ("rgb", "color", OutputKind.COLOR, spec["width"] * spec["height"] * 3),
                ("depth", "depth", OutputKind.DEPTH, spec["width"] * spec["height"] * 2),
                ("info", "info", OutputKind.INFO, 1),
            ):
                if kind != "info" and kind not in self._specs:
                    continue
                self._writers[output] = services.create_camera_writer(
                    session_root / f"camera_{output}.shm", stream_kind=stream_kind,
                    slot_capacity=capacity, source_generation=source_generation,
                )
        except BaseException:
            for writer in self._writers.values():
                writer.close()
            raise
        self._thread = threading.Thread(target=self._run, name="unreal-camera-relay", daemon=True)
        self._thread.start()

    def observe_truth(self, snapshot: dict, *, epoch_s: float) -> None:
        if (snapshot["session_id"], snapshot["model_generation"], snapshot["reset_generation"]) != (
            self._session, self._model, self._reset,
        ):
            raise ValueError("camera truth belongs to another session or generation")
        if not math.isfinite(epoch_s) or epoch_s <= 0:
            raise ValueError("camera source clock join requires positive finite epoch time")
        join = (snapshot["sim_time_ns"], epoch_s)
        existing = self._history.get(snapshot["sequence"])
        if existing is not None and existing != join:
            raise ValueError("camera truth stamp cannot be restamped with a different clock join")
        self._history[snapshot["sequence"]] = join
        while len(self._history) > 256:
            self._history.popitem(last=False)

    def begin(self, *, started_s: float, started_wall_s: float) -> None:
        self._started_s = started_s
        self._observed_s = started_s
        self._last_observed = {kind: started_s for kind in self._specs}
        self._begun = True

    def ready(self) -> bool:
        self._raise_if_failed()
        return self._active == set(self._specs)

    def publish_due(self, *, monotonic_s: float, observed_s: float | None = None) -> None:
        captures: list[_Capture] = []
        try:
            self._poll(monotonic_s=monotonic_s, observed_s=observed_s, captures=captures)
        except BaseException:
            self._drop(tuple(captures))
            raise
        if captures:
            try:
                self._jobs.put_nowait(tuple(captures))
            except queue.Full:
                try:
                    previous = self._jobs.get_nowait()
                except queue.Empty:
                    previous = None  # The worker took it between put and get.
                if previous is not None:
                    self._drop(previous)
                self._jobs.put_nowait(tuple(captures))

    def _poll(self, *, monotonic_s: float, observed_s: float | None, captures: list[_Capture]) -> None:
        self._raise_if_failed()
        if self._closed:
            raise RuntimeError("camera relay is closed")
        observed = monotonic_s if observed_s is None else observed_s
        self._observed_s = max(self._observed_s, observed)
        for kind, spec in self._specs.items():
            if kind not in self._readers:
                try:
                    mapping = self._backend.open(CameraShmAllocation(
                        self._allocation.shm[spec["sensor_id"]], slot_capacity=self._capacity,
                    ))
                except (CameraShmNotReady, FileNotFoundError):
                    continue
                self._mappings[kind] = mapping
                self._readers[kind] = CameraShmReader(mapping)
            try:
                frame = self._readers[kind].read_latest()
            except (CameraShmNotReady, CameraShmChanged, CameraShmStale):
                frame = None
            if frame is None:
                continue
            if (frame.model_generation, frame.reset_generation) != (self._model, self._reset):
                raise ValueError("UE camera capture has missing or mismatched simulation generations")
            if frame.truth_sequence is None or frame.truth_sequence not in self._history:
                raise ValueError("UE camera capture references unpublished or expired truth")
            sim_time, epoch_s = self._history[frame.truth_sequence]
            if frame.sim_time_ns != sim_time:
                raise ValueError("UE camera capture simulation time does not match its truth snapshot")
            age_s = self._wall_time() - epoch_s
            if age_s < -0.05:
                raise ValueError("UE camera capture clock join is in the future")
            if self._begun and age_s > 1.0:
                continue
            expected = (spec["width"], spec["height"], spec["encoding"], spec["frame_id"])
            if (frame.width, frame.height, frame.encoding, frame.frame_id) != expected:
                raise ValueError("UE camera capture geometry/encoding/frame differs from SensorPlan")
            expected_kind = StreamKind.COLOR if kind == "rgb" else StreamKind.DEPTH
            if frame.stream_kind != expected_kind:
                raise ValueError("UE camera capture stream kind differs from SensorPlan")
            if frame.encoding not in ({"rgb8", "bgr8", "rgba8"} if kind == "rgb" else {"16UC1"}):
                raise ValueError("unsupported UE camera relay encoding")
            if kind == "depth" and not math.isclose(frame.depth_scale, 0.001, rel_tol=0, abs_tol=1e-12):
                raise ValueError("UE camera depth must use the declared millimetre scale")
            previous = self._source_sequences[kind]
            if frame.sequence <= previous:
                raise ValueError("UE camera capture sequence moved backwards")
            if self._begun:
                stats = self._stats[kind]
                skipped = max(0, frame.sequence - previous - 1)
                stats.due(count=skipped + 1, dropped=skipped, lateness_s=0.0)
            self._source_sequences[kind] = frame.sequence
            self._last_observed[kind] = observed
            captures.append(_Capture(kind, frame, epoch_s, self._begun))
        if self._begun and any(
            observed - self._last_observed[kind] > max(1.0, 3.0 / spec["rate_hz"])
            for kind, spec in self._specs.items()
        ):
            raise RuntimeError("required UE camera capture stopped advancing")
    def _publish(self, capture: _Capture) -> None:
        frame, kind = capture.frame, capture.kind
        if capture.counted and self._wall_time() - capture.epoch_s > 1.0:
            raise RuntimeError("UE camera capture expired before endpoint publication")
        calibration = (frame.width, frame.height, frame.fx, frame.fy, frame.cx, frame.cy,
                       frame.dist_k1, frame.dist_k2, frame.dist_p1, frame.dist_p2, frame.dist_k3)
        if self._calibration is not None and calibration != self._calibration:
            raise ValueError("UE camera calibration changed inside one Product session")
        record_info = None
        if self._calibration is None:
            record_info = encode_camera_intrinsics(frame, timestamp_s=capture.epoch_s, sequence=self._sequence)
            self._sequence += 1
        rows = np.frombuffer(frame.payload, dtype=np.uint8).reshape(frame.height, frame.stride)
        if kind == "rgb":
            channels = 4 if frame.encoding == "rgba8" else 3
            pixels = rows[:, :frame.width * channels].reshape(frame.height, frame.width, channels)[:, :, :3]
            if frame.encoding == "bgr8":
                pixels = pixels[:, :, ::-1]
            record = encode_camera_rgb(np.ascontiguousarray(pixels), intrinsics=frame,
                                       timestamp_s=capture.epoch_s, sequence=self._sequence)
            output = "color"
        else:
            depth = rows[:, :frame.width * 2].copy().view("<u2").reshape(frame.height, frame.width)
            record = encode_camera_depth(depth.astype(np.float64) * frame.depth_scale, intrinsics=frame,
                                         timestamp_s=capture.epoch_s, sequence=self._sequence)
            output = "depth"
        self._sequence += 1
        self._client.write((record_info.wire if record_info is not None else b"") + record.wire)
        common = dict(timestamp_ns=int(capture.epoch_s * 1e9), width=frame.width, height=frame.height,
                      frame_id=frame.frame_id, fx=frame.fx, fy=frame.fy, cx=frame.cx, cy=frame.cy,
                      dist_k1=frame.dist_k1, dist_k2=frame.dist_k2, dist_p1=frame.dist_p1,
                      dist_p2=frame.dist_p2, dist_k3=frame.dist_k3)
        if record_info is not None:
            self._writers["info"].publish(**common, stride=0, encoding="camera_info", payload=b"")
            self._calibration = calibration
        self._writers[output].publish(**common, stride=frame.width * (3 if kind == "rgb" else 2),
            encoding="rgb8" if kind == "rgb" else "16UC1", payload=record.payload[CAMERA_RECORD_HEADER.size:])
        if capture.counted:
            self._stats[kind].published()
        self._last_source[kind] = dict(source_sequence=frame.sequence, truth_sequence=frame.truth_sequence,
            sim_time_ns=frame.sim_time_ns, model_generation=frame.model_generation,
            reset_generation=frame.reset_generation, transport_timestamp_ns=frame.timestamp_ns,
            mapped_epoch_s=capture.epoch_s)
        self._active.add(kind)

    def _drop(self, captures: tuple[_Capture, ...]) -> None:
        for capture in captures:
            if capture.counted:
                self._stats[capture.kind].drop_current()

    def _run(self) -> None:
        while True:
            captures = self._jobs.get()
            if captures is None:
                return
            try:
                for _index, capture in enumerate(captures):
                    self._publish(capture)
            except BaseException as exc:
                self._drop(captures[_index:])
                self._failure = exc
                return

    def _raise_if_failed(self) -> None:
        if self._failure is not None:
            raise RuntimeError(f"UE camera relay failed: {self._failure}") from self._failure

    def evidence(self) -> dict:
        schedule = {}
        for kind, spec in self._specs.items():
            scheduled = math.ceil(max(0.0, self._observed_s - self._started_s) * spec["rate_hz"] - 1e-9) if self._begun else 0
            published = self._stats[kind].payload(1.0)["published_count"] if self._begun else 0
            schedule[kind] = dict(rate_hz=spec["rate_hz"], scheduled_count=scheduled,
                published_count=published, unfilled_count=max(0, scheduled - published))
        return dict(product_session_id=self._product, session_id=self._session,
                    source="unreal_camera", streams=dict(self._last_source), declared_schedule=schedule,
                    worker_stopped=not self._thread.is_alive(),
                    failure_type=type(self._failure).__name__ if self._failure is not None else None)

    def close(self) -> None:
        if self._closed:
            self._raise_if_failed()
            return
        self._closed = True
        try:
            if self._thread.is_alive():
                self._jobs.put(None, timeout=4)
                self._thread.join(timeout=4)
            if self._thread.is_alive():
                raise RuntimeError("UE camera relay did not stop")
        finally:
            while True:
                try:
                    pending = self._jobs.get_nowait()
                except queue.Empty:
                    break
                if pending is not None:
                    self._drop(pending)
            for mapping in self._mappings.values():
                close = getattr(mapping, "close", None)
                if close is not None:
                    close()
            for writer in self._writers.values():
                writer.close()
        self._raise_if_failed()
