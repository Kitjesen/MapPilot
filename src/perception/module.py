"""Blueprint-facing RGB-D scene-perception runtime adapter."""

from __future__ import annotations

import logging
import threading
from collections.abc import Mapping
from typing import Any

import numpy as np

from perception.backends import DetectorSpec, ObservationSource
from perception.frames import FrameDrop, FrameResult, FrameSynchronizer, SynchronizedFrame
from perception.pipeline import PerceptionOutcome, PerceptionPipeline, PerceptionSettings
from runtime.module import Module
from runtime.msgs.geometry import PoseStamped
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import CameraIntrinsics, Image
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

_PUBLISHABLE_OUTCOMES = frozenset({"positive", "negative", "degraded"})


@register("perception", "scene", description="RGB-D semantic scene perception module")
class PerceptionModule(Module, layer=3):
    """Synchronize sensor inputs and run the latest complete frame in one worker."""

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraIntrinsics]
    odometry: In[Odometry]
    map_odom_tf: In[dict]

    scene_graph: Out[SceneGraph]
    detections_3d: Out[list]
    robot_pose: Out[PoseStamped]

    def __init__(
        self,
        *,
        settings: PerceptionSettings,
        detector: DetectorSpec | ObservationSource,
        camera_to_body: np.ndarray,
        skip_frames: int,
        max_rgbd_skew_s: float,
        max_odom_age_s: float,
        max_map_odom_age_s: float,
        **config: Any,
    ) -> None:
        super().__init__(**config)
        self._detector_name = str(
            getattr(detector, "name", type(detector).__name__)
        )
        self._synchronizer = FrameSynchronizer(
            camera_to_body=camera_to_body,
            skip_frames=skip_frames,
            max_rgbd_skew_s=max_rgbd_skew_s,
            max_odom_age_s=max_odom_age_s,
            max_map_odom_age_s=max_map_odom_age_s,
        )
        self._pipeline = PerceptionPipeline(settings, detector)

        self._condition = threading.Condition()
        self._publish_lock = threading.RLock()
        self._stop_lock = threading.Lock()
        self._pending: SynchronizedFrame | None = None
        self._worker: threading.Thread | None = None
        self._accepting = False
        self._shutdown = False
        self._setup_complete = False
        self._pipeline_closed = False
        self._stop_started = False

        self._processed_frames = 0
        self._coalesced_frames = 0
        self._latest_detections = 0
        self._last_valid_frame_ts = 0.0
        self._valid_frames = 0
        self._last_outcome_status = ""
        self._last_outcome_reason = ""
        self._outcome_counts: dict[str, int] = {}
        self._failure_counts: dict[str, int] = {}

    def setup(self) -> None:
        """Load the configured backend before exposing any input callback."""

        try:
            self._pipeline.setup()
            self.color_image.subscribe(self._on_color)
            self.depth_image.subscribe(self._on_depth)
            self.camera_info.subscribe(self._on_camera_info)
            self.odometry.subscribe(self._on_odometry)
            self.map_odom_tf.subscribe(self._on_map_odom)
        except Exception:
            self._close_pipeline_once()
            raise
        self._setup_complete = True

    def start(self) -> None:
        """Start the sole inference worker after setup has loaded the backend."""

        if not self._setup_complete:
            raise RuntimeError("PerceptionModule.setup() must complete before start()")
        with self._condition:
            if self._worker is not None:
                return
            self._shutdown = False
            self._accepting = True
            self._worker = threading.Thread(
                target=self._worker_main,
                name="perception-latest-frame",
                daemon=True,
            )
            worker = self._worker
        super().start()
        worker.start()

    def stop(self) -> None:
        """Close inputs, discard pending work, then wait for in-flight inference."""

        with self._stop_lock:
            if self._stop_started:
                return
            self._stop_started = True
            with self._publish_lock:
                self._accepting = False
                super().stop()
            with self._condition:
                self._pending = None
                self._shutdown = True
                self._condition.notify_all()
                worker = self._worker
            if worker is None:
                self._close_pipeline_once()
            elif worker is not threading.current_thread():
                worker.join()

    def startup_readiness(self) -> str | None:
        """Require a live worker, loaded detector, and one valid synchronized frame."""

        if not self.running:
            return "not_running"
        worker = self._worker
        if worker is None or not worker.is_alive():
            return "worker_not_running"
        if not bool(self._pipeline.health().get("detector_ready", False)):
            return "detector_not_ready"
        with self._condition:
            if self._valid_frames == 0:
                return "waiting_for_valid_frame"
        return None

    def health(self) -> dict[str, Any]:
        """Return port, synchronization, worker, and pipeline diagnostics."""

        info: dict[str, Any] = super().port_summary()
        sync_health = self._synchronizer.health()
        pipeline_health = self._pipeline.health()
        with self._condition:
            worker = self._worker
            worker_alive = bool(worker is not None and worker.is_alive())
            if worker_alive:
                worker_state = "stopping" if self._shutdown else "running"
            elif worker is None:
                worker_state = "not_started"
            else:
                worker_state = "stopped"
            processed_frames = self._processed_frames
            coalesced_frames = self._coalesced_frames
            latest_detections = self._latest_detections
            last_valid_frame_ts = self._last_valid_frame_ts
            valid_frames = self._valid_frames
            last_outcome_status = self._last_outcome_status
            last_outcome_reason = self._last_outcome_reason
            outcome_counts = dict(self._outcome_counts)
            failure_counts = dict(self._failure_counts)

        drop_reasons = dict(sync_health.get("drop_reasons", {}))
        source_health = dict(pipeline_health.get("source", {}))
        missing_odom = int(drop_reasons.get("missing_odometry", 0))
        sync_drop_reason = str(sync_health.get("last_drop_reason", ""))
        last_drop_reason = sync_drop_reason
        if not last_drop_reason and last_outcome_status in {"dropped", "failed"}:
            last_drop_reason = last_outcome_reason
        info.update(pipeline_health)
        info.update(
            {
                "frame_count": int(sync_health.get("color_frames", 0)),
                "matched_frames": int(sync_health.get("matched_frames", 0)),
                "processed_frames": processed_frames,
                "coalesced_frames": coalesced_frames,
                "detector_type": self._detector_name,
                "detector_ready": bool(pipeline_health.get("detector_ready", False)),
                "detector_tracker_ready": bool(
                    source_health.get("detector_tracker_ready", False)
                ),
                "detector": {
                    "configured_backend": self._detector_name,
                    "backend": str(
                        pipeline_health.get("detector_type", self._detector_name)
                    ),
                    "degraded": False,
                    "degraded_reason": "",
                },
                "latest_detections": latest_detections,
                "dropped_missing_odom_frames": missing_odom,
                "dropped_unsynced_frames": max(
                    0,
                    int(sync_health.get("dropped_frames", 0)) - missing_odom,
                ),
                "last_drop_reason": last_drop_reason,
                "last_outcome_reason": last_outcome_reason,
                "last_valid_frame_ts": last_valid_frame_ts,
                "valid_frames": valid_frames,
                "worker_alive": worker_alive,
                "worker_state": worker_state,
                "outcome_counts": outcome_counts,
                "failure_counts": failure_counts,
                "synchronizer": sync_health,
            }
        )
        if "detector_tracker_backend" in source_health:
            info["detector_tracker_backend"] = source_health[
                "detector_tracker_backend"
            ]
        if "person_tracking" in source_health:
            info["person_tracking"] = source_health["person_tracking"]
        return info

    def _on_color(self, image: Image) -> None:
        self._accept_results(self._synchronizer.push_color(image))

    def _on_depth(self, image: Image) -> None:
        self._accept_results(self._synchronizer.push_depth(image))

    def _on_camera_info(self, intrinsics: CameraIntrinsics) -> None:
        self._accept_results(self._synchronizer.push_camera_info(intrinsics))

    def _on_odometry(self, odometry: Odometry) -> None:
        self._accept_results(self._synchronizer.push_odometry(odometry))

    def _on_map_odom(self, transform: Mapping[str, Any]) -> None:
        self._accept_results(self._synchronizer.push_map_odom(transform))

    def _accept_results(self, results: tuple[FrameResult, ...]) -> None:
        for result in results:
            if isinstance(result, FrameDrop):
                with self._condition:
                    self._last_outcome_reason = result.reason
                continue
            self._submit(result)

    def _submit(self, frame: SynchronizedFrame) -> None:
        with self._condition:
            if not self._accepting or self._shutdown:
                return
            if self._pending is not None:
                self._coalesced_frames += 1
            self._pending = frame
            self._condition.notify()

    def _worker_main(self) -> None:
        try:
            while True:
                with self._condition:
                    while self._pending is None and not self._shutdown:
                        self._condition.wait()
                    if self._shutdown:
                        return
                    frame = self._pending
                    self._pending = None
                assert frame is not None

                try:
                    outcome = self._pipeline.process(frame)
                except Exception as exc:
                    logger.exception("Perception frame processing failed")
                    self._record_outcome("failed", type(exc).__name__)
                    continue

                self._record_outcome(outcome.status, outcome.reason)
                if outcome.status not in _PUBLISHABLE_OUTCOMES:
                    continue
                payload = _complete_payload(outcome)
                if payload is None:
                    self._record_failure("invalid_pipeline_outcome")
                    continue

                robot_pose, runtime_detections, scene_graph = payload
                detections = list(runtime_detections)
                with self._condition:
                    self._last_valid_frame_ts = float(frame.timestamp)
                    self._valid_frames += 1
                    self._latest_detections = len(detections)
                self._publish_outputs(robot_pose, detections, scene_graph)
        finally:
            self._close_pipeline_once()

    def _publish_outputs(
        self,
        robot_pose: PoseStamped,
        detections: list[Any],
        scene_graph: SceneGraph,
    ) -> None:
        with self._publish_lock:
            if not self._can_publish():
                return
            self.robot_pose.publish(robot_pose)
            if not self._can_publish():
                return
            self.detections_3d.publish(detections)
            if not self._can_publish():
                return
            self.scene_graph.publish(scene_graph)

    def _can_publish(self) -> bool:
        return self._accepting and self.running

    def _record_outcome(self, status: str, reason: str) -> None:
        with self._condition:
            self._processed_frames += 1
            self._outcome_counts[status] = self._outcome_counts.get(status, 0) + 1
            self._last_outcome_status = status
            self._last_outcome_reason = reason
            if status in {"dropped", "failed"}:
                key = reason or status
                self._failure_counts[key] = self._failure_counts.get(key, 0) + 1

    def _record_failure(self, reason: str) -> None:
        with self._condition:
            self._last_outcome_reason = reason
            self._failure_counts[reason] = self._failure_counts.get(reason, 0) + 1

    def _close_pipeline_once(self) -> None:
        with self._condition:
            if self._pipeline_closed:
                return
            self._pipeline_closed = True
        self._pipeline.close()


def _complete_payload(
    outcome: PerceptionOutcome,
) -> tuple[PoseStamped, tuple[Any, ...], SceneGraph] | None:
    robot_pose = outcome.robot_pose
    detections = outcome.detections
    scene_graph = outcome.scene_graph
    if robot_pose is None or detections is None or scene_graph is None:
        return None
    return robot_pose, detections, scene_graph


__all__ = ["PerceptionModule"]
