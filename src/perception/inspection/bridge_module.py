"""Runtime module for native DDS inspection evidence capture.

The native nav endpoint owns route execution and emits evidence requests over a
small C ABI DDS bridge. This module stays in the perception worker, caches the
latest sensor observations, persists auditable evidence, and returns only a
truthful persistence/verdict result.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from runtime import In, Module
from runtime.contracts.inspection_evidence import (
    SUPPORTED_ACTIONS,
    EvidenceConflictError,
    EvidenceValidationError,
    InspectionEvidenceRequest,
    InspectionEvidenceResult,
    InspectionEvidenceStore,
)
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import CameraIntrinsics, Image
from runtime.registry import register

from .native_bridge import NativeInspectionEvidenceBridge

STATUS_SCHEMA_VERSION = "lingtu.inspection.evidence.status.v1"
DEFAULT_EVIDENCE_ROOT = "~/data/lingtu/inspection_evidence"
DEFAULT_STATUS_FILE = "/dev/shm/lingtu/inspection_evidence_status.json"


@dataclass(frozen=True)
class _Sample:
    value: Any
    received_at_s: float
    message_ts_s: float | None


def _default_evidence_root() -> Path:
    return Path(os.environ.get("LINGTU_INSPECTION_EVIDENCE_DIR", DEFAULT_EVIDENCE_ROOT))


def _default_status_file() -> Path:
    return Path(os.environ.get("LINGTU_INSPECTION_EVIDENCE_STATUS_FILE", DEFAULT_STATUS_FILE))


@register(
    "inspection_evidence",
    "native_bridge",
    description="Inspection evidence capture worker using the native C ABI DDS bridge",
)
class InspectionEvidenceModule(Module, layer=3):
    """Persist inspection evidence requested by the native navigation runtime."""

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraIntrinsics]
    odometry: In[Odometry]
    detections_3d: In[list]

    def __init__(
        self,
        *,
        evidence_root: str | os.PathLike[str] | None = None,
        status_file: str | os.PathLike[str] | None = None,
        domain_id: int = 0,
        poll_interval_s: float = 0.05,
        max_frame_age_s: float = 2.0,
        max_odom_age_s: float = 2.0,
        max_rgb_odom_skew_s: float = 0.2,
        heartbeat_interval_s: float = 1.0,
        start_thread: bool = True,
        bridge_factory: Callable[[], Any] | None = None,
        store_factory: Callable[[Path], InspectionEvidenceStore] | None = None,
        jpeg_encoder: Callable[[Image], bytes] | None = None,
        **config: Any,
    ) -> None:
        super().__init__(**config)
        self._evidence_root = Path(evidence_root) if evidence_root is not None else _default_evidence_root()
        self._status_file = Path(status_file) if status_file is not None else _default_status_file()
        self._domain_id = int(domain_id)
        self._poll_interval_s = max(0.01, float(poll_interval_s))
        self._max_frame_age_s = max(0.0, float(max_frame_age_s))
        self._max_odom_age_s = max(0.0, float(max_odom_age_s))
        self._max_rgb_odom_skew_s = float(max_rgb_odom_skew_s)
        if not math.isfinite(self._max_rgb_odom_skew_s) or self._max_rgb_odom_skew_s < 0.0:
            raise ValueError("max_rgb_odom_skew_s must be finite and non-negative")
        self._heartbeat_interval_s = min(1.0, max(0.1, float(heartbeat_interval_s)))
        self._start_thread = bool(start_thread)
        self._bridge_factory = bridge_factory or (
            lambda: NativeInspectionEvidenceBridge(domain_id=self._domain_id)
        )
        self._store_factory = store_factory or InspectionEvidenceStore
        self._jpeg_encoder = jpeg_encoder
        self._store: InspectionEvidenceStore | None = None
        self._bridge: Any | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.RLock()
        self._color: _Sample | None = None
        self._depth: _Sample | None = None
        self._camera_info: _Sample | None = None
        self._odom: _Sample | None = None
        self._detections: _Sample | None = None
        self._state = "init"
        self._last_request_id = ""
        self._last_evidence_id = ""
        self._last_error = ""
        self._last_heartbeat_s = 0.0

    def setup(self) -> None:
        self.color_image.subscribe(self._on_color)
        self.color_image.set_policy("latest")
        self.depth_image.subscribe(self._on_depth)
        self.depth_image.set_policy("latest")
        self.camera_info.subscribe(self._on_camera_info)
        self.camera_info.set_policy("latest")
        self.odometry.subscribe(self._on_odometry)
        self.odometry.set_policy("latest")
        self.detections_3d.subscribe(self._on_detections)
        self.detections_3d.set_policy("latest")
        self._store = self._store_factory(self._evidence_root)
        self._bridge = self._bridge_factory()
        self._state = "ready"
        self._write_status(ready=True, state="ready")

    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._write_status(ready=True, state=self._state)
        if self._start_thread and self._thread is None:
            self._thread = threading.Thread(
                target=self._run_loop,
                name="inspection-evidence",
                daemon=True,
            )
            self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        self._thread = None
        bridge = self._bridge
        self._bridge = None
        if bridge is not None:
            try:
                bridge.close()
            except Exception as exc:
                self._last_error = f"bridge_close_failed:{exc}"
        self._state = "stopped"
        self._write_status(ready=False, state="stopped")
        super().stop()

    def poll_once(self) -> bool:
        """Process at most one DDS request. Returns True when a sample existed."""

        if self._bridge is None:
            raise RuntimeError("InspectionEvidenceModule is not set up")
        try:
            request = self._bridge.take_request()
        except Exception as exc:
            self._last_error = f"take_request_failed:{exc}"
            self._write_status(ready=True, state="error")
            return False
        if request is None:
            self._heartbeat()
            return False
        self._handle_request(request)
        return True

    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            self.poll_once()
            self._stop_event.wait(self._poll_interval_s)

    def _on_color(self, image: Image) -> None:
        self._set_sample("_color", image, getattr(image, "ts", None))

    def _on_depth(self, image: Image) -> None:
        self._set_sample("_depth", image, getattr(image, "ts", None))

    def _on_camera_info(self, info: CameraIntrinsics) -> None:
        self._set_sample("_camera_info", info, getattr(info, "ts", None))

    def _on_odometry(self, odom: Odometry) -> None:
        self._set_sample("_odom", odom, getattr(odom, "ts", None))

    def _on_detections(self, detections: list) -> None:
        self._set_sample("_detections", detections, None)

    def _set_sample(self, name: str, value: Any, message_ts_s: Any) -> None:
        ts = float(message_ts_s) if isinstance(message_ts_s, (int, float)) else None
        with self._lock:
            setattr(self, name, _Sample(value=value, received_at_s=time.time(), message_ts_s=ts))

    def _handle_request(self, request: Mapping[str, Any]) -> None:
        now = time.time()
        normalized_request = _normalize_request_mapping(request)
        request_id = str(normalized_request.get("request_id", ""))
        self._last_request_id = request_id
        self._state = "processing"
        self._write_status(ready=True, state="processing")

        persisted = False
        evidence_id = ""
        reason = "unknown"
        verdict = "inconclusive"
        try:
            existing = self._existing_matching_result(normalized_request)
            if existing is not None:
                persisted = True
                evidence_id = existing.request.request_id
                reason = "persisted"
                verdict = existing.analysis_verdict
            elif now > float(normalized_request.get("deadline_s", 0.0)):
                reason = "deadline_expired"
            else:
                snapshot = self._snapshot(now)
                if snapshot["blocker"]:
                    reason = str(snapshot["blocker"])
                else:
                    result = self._persist_request(normalized_request, snapshot)
                    persisted = True
                    evidence_id = result.request.request_id
                    reason = "persisted"
                    verdict = result.analysis_verdict
        except EvidenceConflictError:
            persisted = False
            reason = "request_id_conflict"
        except EvidenceValidationError as exc:
            persisted = False
            reason = f"invalid_request:{exc}"
        except Exception as exc:
            persisted = False
            reason = f"persist_failed:{exc}"

        if persisted:
            self._last_evidence_id = evidence_id
            self._last_error = ""
        else:
            self._last_error = reason
        self._write_result(
            request_id=request_id,
            evidence_id=evidence_id,
            persisted=persisted,
            reason=reason,
            analysis_verdict=verdict,
        )
        self._state = "ready"
        self._write_status(ready=True, state="ready")

    def _persist_request(
        self,
        request: Mapping[str, Any],
        snapshot: Mapping[str, Any],
    ) -> InspectionEvidenceResult:
        if self._store is None:
            raise RuntimeError("store is not initialized")
        rgb_bytes = self._encode_jpeg(snapshot["color"].value)
        pose = self._pose_payload(snapshot)
        detections = self._detections_payload(snapshot)
        return self._store.persist(
            request,
            rgb_bytes=rgb_bytes,
            media_type="image/jpeg",
            pose=pose,
            detections=detections,
        )

    def _snapshot(self, now: float) -> dict[str, Any]:
        with self._lock:
            color = self._color
            odom = self._odom
            depth = self._depth
            camera_info = self._camera_info
            detections = self._detections

        blocker = self._sensor_readiness_reason(
            now,
            color=color,
            odom=odom,
        )
        if blocker != "ready":
            return {"blocker": blocker}
        return {
            "blocker": "",
            "color": color,
            "odom": odom,
            "depth": depth,
            "camera_info": camera_info,
            "detections": detections,
            "captured_at_s": _sample_timestamp(color),
            "rgb_odom_skew_s": _sample_skew_s(color, odom),
        }

    def _encode_jpeg(self, image: Image) -> bytes:
        if self._jpeg_encoder is not None:
            return self._jpeg_encoder(image)
        try:
            import cv2  # type: ignore[import-untyped]
        except ImportError as exc:
            raise RuntimeError("cv2_unavailable") from exc
        bgr = image.to_bgr().data if hasattr(image, "to_bgr") else image.data
        ok, encoded = cv2.imencode(".jpg", bgr)
        if not ok:
            raise RuntimeError("jpeg_encode_failed")
        return bytes(encoded)

    def _pose_payload(self, snapshot: Mapping[str, Any]) -> dict[str, Any]:
        odom_sample: _Sample = snapshot["odom"]
        odom = odom_sample.value
        pose = odom.to_dict() if hasattr(odom, "to_dict") else {"repr": repr(odom)}
        return {
            "captured_at_s": snapshot["captured_at_s"],
            "odometry_received_at_s": odom_sample.received_at_s,
            "rgb_odom_skew_s": snapshot["rgb_odom_skew_s"],
            "odometry": pose,
        }

    def _detections_payload(self, snapshot: Mapping[str, Any]) -> dict[str, Any]:
        detections_sample = snapshot.get("detections")
        if not isinstance(detections_sample, _Sample):
            return {
                "sample_available": False,
                "interpretation": "inconclusive",
                "reason": "no_detection_sample",
                "objects": [],
            }
        objects = _jsonable_detections(detections_sample.value)
        return {
            "sample_available": True,
            "received_at_s": detections_sample.received_at_s,
            "interpretation": "observations_only_not_clearance",
            "objects": objects,
        }

    def _write_result(
        self,
        *,
        request_id: str,
        evidence_id: str,
        persisted: bool,
        reason: str,
        analysis_verdict: str,
    ) -> None:
        if self._bridge is None:
            return
        try:
            self._bridge.write_result(
                request_id=request_id,
                evidence_id=evidence_id,
                persisted=persisted,
                reason=reason[:255],
                analysis_verdict=analysis_verdict[:255],
                result_at_s=time.time(),
            )
        except Exception as exc:
            self._last_error = f"write_result_failed:{exc}"
            self._write_status(ready=True, state="error")

    def _existing_matching_result(
        self,
        request: Mapping[str, Any],
    ) -> InspectionEvidenceResult | None:
        request_id = str(request.get("request_id", ""))
        if not request_id or self._store is None:
            return None
        try:
            expected = InspectionEvidenceRequest.from_value(request)
            existing = self._store.get(request_id)
        except FileNotFoundError:
            return None
        if existing.request != expected:
            raise EvidenceConflictError(
                "request_id already committed for a different audit identity"
            )
        return existing

    def _heartbeat(self) -> None:
        now = time.time()
        if now - self._last_heartbeat_s >= self._heartbeat_interval_s:
            self._write_status(ready=True, state=self._state)

    def _sensor_readiness_reason(
        self,
        now: float,
        *,
        color: _Sample | None = None,
        odom: _Sample | None = None,
    ) -> str:
        if color is None or odom is None:
            with self._lock:
                color = self._color if color is None else color
                odom = self._odom if odom is None else odom
        color_reason = _sample_readiness_reason(
            color,
            now=now,
            max_age_s=self._max_frame_age_s,
            missing="rgb_missing",
            stale="rgb_stale",
        )
        if color_reason != "ready":
            return color_reason
        odom_reason = _sample_readiness_reason(
            odom,
            now=now,
            max_age_s=self._max_odom_age_s,
            missing="odometry_missing",
            stale="odometry_stale",
        )
        if odom_reason != "ready":
            return odom_reason
        if _sample_skew_s(color, odom) > self._max_rgb_odom_skew_s:
            return "rgb_odometry_unsynchronized"
        return "ready"

    def _write_status(self, *, ready: bool, state: str) -> None:
        now = time.time()
        self._last_heartbeat_s = now
        readiness_reason = state if not ready else self._sensor_readiness_reason(now)
        if state == "error":
            readiness_reason = "worker_error"
        effective_ready = bool(ready and state != "error" and readiness_reason == "ready")
        payload = {
            "schema_version": STATUS_SCHEMA_VERSION,
            "ready": effective_ready,
            "state": state,
            "readiness_reason": readiness_reason,
            "pid": os.getpid(),
            "heartbeat_ts": now,
            "heartbeat_at_s": now,
            "evidence_root": str(self._evidence_root.expanduser()),
            "supported_actions": sorted(SUPPORTED_ACTIONS),
            "max_rgb_odom_skew_s": self._max_rgb_odom_skew_s,
            "analyzers": {
                "capture:overview": "capture_only",
                "capture:parking": "trusted_observation_required",
                "capture:bin_full": "unavailable",
                "capture:plate_ocr": "unavailable",
            },
            "last_request_id": self._last_request_id,
            "last_evidence_id": self._last_evidence_id,
            "last_error": self._last_error,
        }
        _atomic_json_write(self._status_file, payload)


def _sample_readiness_reason(
    sample: _Sample | None,
    *,
    now: float,
    max_age_s: float,
    missing: str,
    stale: str,
) -> str:
    if sample is None:
        return missing
    sample_ts = sample.message_ts_s if sample.message_ts_s is not None else sample.received_at_s
    if not math.isfinite(sample_ts) or sample_ts <= 0.0:
        return stale
    age_s = now - sample_ts
    if age_s > max_age_s or age_s < -max(1.0, max_age_s):
        return stale
    return "ready"


def _sample_timestamp(sample: _Sample) -> float:
    if sample.message_ts_s is not None and math.isfinite(sample.message_ts_s):
        return sample.message_ts_s
    return sample.received_at_s


def _sample_skew_s(left: _Sample, right: _Sample) -> float:
    if left.message_ts_s is not None and right.message_ts_s is not None:
        return abs(_sample_timestamp(left) - _sample_timestamp(right))
    return abs(left.received_at_s - right.received_at_s)


def _jsonable_detections(value: Any) -> list[Any]:
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [_jsonable(item) for item in value]
    return [_jsonable(value)]


def _jsonable(value: Any) -> Any:
    if hasattr(value, "to_dict"):
        return value.to_dict()
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [_jsonable(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return repr(value)


def _normalize_request_mapping(request: Mapping[str, Any]) -> dict[str, Any]:
    normalized = dict(request)
    if "route_revision" not in normalized and "revision" in normalized:
        normalized["route_revision"] = normalized["revision"]
    normalized.pop("revision", None)
    return normalized


def _atomic_json_write(path: Path, payload: Mapping[str, Any]) -> None:
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.{threading.get_ident()}.tmp")
    data = json.dumps(payload, ensure_ascii=False, allow_nan=False, sort_keys=True).encode("utf-8")
    with tmp.open("wb") as handle:
        handle.write(data)
        handle.write(b"\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)
