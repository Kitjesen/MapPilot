"""HostBuilder-facing RGB-D scene-perception runtime adapter."""

from __future__ import annotations

import logging
import os
import threading
import time
from collections.abc import Mapping
from typing import Any

import numpy as np

from perception.backends import DetectorSpec, ObservationSource
from perception.frames import FrameDrop, FrameResult, FrameSynchronizer, SynchronizedFrame
from perception.pipeline import PerceptionOutcome, PerceptionPipeline, PerceptionSettings
from runtime.module import Module
from runtime.msgs.geometry import PoseStamped
from runtime.msgs.nav import NavigationState, Odometry
from runtime.msgs.semantic import SceneGraph, SemanticObservation
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
    navigation_state: In[NavigationState]

    observation: Out[SemanticObservation]
    scene_graph: Out[SceneGraph]
    detections_3d: Out[list]
    robot_pose: Out[PoseStamped]
    observation_image: Out[Image]

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
        product_session_id: str = "",
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
        self._product_session_id = str(
            product_session_id or os.getenv("LINGTU_PRODUCT_SESSION_ID", "")
        )

        self._condition = threading.Condition()
        self._publish_lock = threading.RLock()
        self._stop_lock = threading.RLock()
        self._pending: SynchronizedFrame | None = None
        self._latest_accepted_ts: float | None = None
        self._latest_published_ts: float | None = None
        self._context_revision = self._synchronizer.context_revision()
        self._observation_sequence = 0
        self._spatial_reset_pending = False
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
            self.navigation_state.subscribe(self._on_navigation_state)
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
            self._publish_unavailable("perception_stopped", source_ts=time.time())
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
        self._accept_sync_results(self._synchronizer.push_color(image))

    def _on_depth(self, image: Image) -> None:
        self._accept_sync_results(self._synchronizer.push_depth(image))

    def _on_camera_info(self, intrinsics: CameraIntrinsics) -> None:
        self._accept_sync_results(self._synchronizer.push_camera_info(intrinsics))

    def _on_odometry(self, odometry: Odometry) -> None:
        self._accept_sync_results(self._synchronizer.push_odometry(odometry))

    def _on_map_odom(self, transform: Mapping[str, Any]) -> None:
        self._accept_sync_results(self._synchronizer.push_map_odom(transform))

    def _on_navigation_state(self, state: NavigationState) -> None:
        self._accept_sync_results(self._synchronizer.push_navigation_state(state))

    def _accept_sync_results(self, results: tuple[FrameResult, ...]) -> None:
        revision = self._synchronizer.context_revision()
        invalidation_reason = ""
        with self._condition:
            if revision != self._context_revision:
                previous = self._context_revision
                self._context_revision = revision
                self._pending = None
                self._latest_accepted_ts = None
                self._latest_published_ts = None
                invalidation_reason = "context_changed"
                if any(
                    isinstance(result, FrameDrop)
                    and result.reason in {"map_changed", "camera_restarted"}
                    for result in results
                ):
                    self._spatial_reset_pending = True
                if any(
                    isinstance(result, FrameDrop) and result.reason == "map_changed"
                    for result in results
                ):
                    invalidation_reason = "map_changed"
                elif any(
                    isinstance(result, FrameDrop)
                    and result.reason == "camera_restarted"
                    for result in results
                ):
                    invalidation_reason = "camera_restarted"
                logger.info(
                    "Perception context changed from revision %s to %s: %s",
                    previous,
                    revision,
                    invalidation_reason,
                )
        if invalidation_reason:
            timestamp = next(
                (
                    result.timestamp
                    for result in results
                    if isinstance(result, FrameDrop)
                    and result.reason in {"map_changed", "camera_restarted"}
                ),
                time.time(),
            )
            self._publish_unavailable(invalidation_reason, source_ts=timestamp)
        self._accept_results(results)

    def _accept_results(self, results: tuple[FrameResult, ...]) -> None:
        for result in results:
            if isinstance(result, FrameDrop):
                with self._condition:
                    self._last_outcome_reason = result.reason
                if _drop_invalidates_current_observation(result.reason):
                    self._publish_unavailable(result.reason, source_ts=result.timestamp)
                continue
            self._submit(result)

    def _submit(self, frame: SynchronizedFrame) -> None:
        stale_reason = ""
        with self._condition:
            if not self._accepting or self._shutdown:
                return
            if frame.context_revision != self._context_revision:
                stale_reason = "context_changed"
            elif (
                self._latest_accepted_ts is not None
                and frame.timestamp <= self._latest_accepted_ts
            ):
                stale_reason = "stale_frame"
            if stale_reason:
                self._last_outcome_reason = stale_reason
            else:
                self._latest_accepted_ts = float(frame.timestamp)
                if self._pending is not None:
                    self._coalesced_frames += 1
                self._pending = frame
                self._condition.notify()
        if stale_reason:
            self._record_failure(stale_reason)

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

                with self._condition:
                    reset_spatial_context = self._spatial_reset_pending
                    self._spatial_reset_pending = False
                if reset_spatial_context:
                    self._pipeline.reset_spatial_context()

                try:
                    outcome = self._pipeline.process(frame)
                except Exception as exc:
                    logger.exception("Perception frame processing failed")
                    self._record_outcome("failed", type(exc).__name__)
                    if not self._frame_stale_reason(frame):
                        self._publish_unavailable(
                            type(exc).__name__,
                            source_ts=frame.timestamp,
                        )
                    continue

                self._record_outcome(outcome.status, outcome.reason)
                stale_reason = self._frame_stale_reason(frame)
                if stale_reason:
                    self._record_failure(stale_reason)
                    continue
                if outcome.status not in _PUBLISHABLE_OUTCOMES:
                    self._publish_unavailable(
                        outcome.reason or outcome.status,
                        source_ts=frame.timestamp,
                    )
                    continue
                payload = _complete_payload(outcome)
                if payload is None:
                    self._record_failure("invalid_pipeline_outcome")
                    continue

                robot_pose, runtime_detections, scene_graph = payload
                detections = list(runtime_detections)
                stale_reason = self._frame_stale_reason(frame)
                if stale_reason:
                    self._record_failure(stale_reason)
                    continue
                with self._condition:
                    self._last_valid_frame_ts = float(frame.timestamp)
                    self._valid_frames += 1
                    self._latest_detections = len(detections)
                self._publish_outputs(
                    frame,
                    outcome,
                    robot_pose,
                    detections,
                    scene_graph,
                )
        finally:
            self._close_pipeline_once()

    def _publish_outputs(
        self,
        frame: SynchronizedFrame,
        outcome: PerceptionOutcome,
        robot_pose: PoseStamped,
        detections: list[Any],
        scene_graph: SceneGraph,
    ) -> None:
        with self._publish_lock:
            if not self._can_publish():
                return
            stale_reason = self._frame_stale_reason(frame)
            if stale_reason:
                self._record_failure(stale_reason)
                return
            observation = self._new_observation(
                status=outcome.status,
                reason=outcome.reason,
                frame=frame,
                robot_pose=robot_pose,
                detections=tuple(detections),
                scene_graph=scene_graph,
            )
            with self._condition:
                self._latest_published_ts = float(frame.timestamp)
            self.observation.publish(observation)
            if not self._can_publish():
                return
            self.observation_image.publish(frame.color)
            self.robot_pose.publish(robot_pose)
            if not self._can_publish():
                return
            self.detections_3d.publish(detections)
            if not self._can_publish():
                return
            self.scene_graph.publish(scene_graph)

    def _frame_stale_reason(self, frame: SynchronizedFrame) -> str:
        if frame.context_revision != self._synchronizer.context_revision():
            return "context_changed"
        with self._condition:
            if frame.context_revision != self._context_revision:
                return "context_changed"
            if (
                self._latest_published_ts is not None
                and frame.timestamp <= self._latest_published_ts
            ):
                return "stale_frame"
        return ""

    def _publish_unavailable(self, reason: str, *, source_ts: float) -> None:
        with self._publish_lock:
            if not self._can_publish():
                return
            self.observation.publish(
                self._new_observation(
                    status="unavailable",
                    reason=reason,
                    source_ts=source_ts,
                )
            )

    def _new_observation(
        self,
        *,
        status: str,
        reason: str,
        source_ts: float = 0.0,
        frame: SynchronizedFrame | None = None,
        robot_pose: PoseStamped | None = None,
        detections: tuple[Any, ...] = (),
        scene_graph: SceneGraph | None = None,
    ) -> SemanticObservation:
        self._observation_sequence += 1
        source_generation, map_id, content_epoch = self._synchronizer.context_token()
        if frame is not None:
            source_ts = frame.timestamp
            source_generation, map_id, content_epoch = frame.context_token
        return SemanticObservation(
            product_session_id=self._product_session_id,
            sequence=self._observation_sequence,
            source_ts=float(source_ts),
            completed_ts=time.time(),
            source_generation=source_generation,
            map_id=map_id,
            content_epoch=content_epoch,
            status=status,
            reason=reason,
            image=frame.color if frame is not None else None,
            depth=frame.depth if frame is not None else None,
            camera_info=frame.intrinsics if frame is not None else None,
            robot_pose=robot_pose,
            map_from_camera=frame.map_from_camera if frame is not None else None,
            detections=detections,
            scene_graph=scene_graph,
            odometry_ts=frame.odometry_ts if frame is not None else 0.0,
            map_odom_ts=frame.map_odom_ts if frame is not None else 0.0,
        )

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


def _drop_invalidates_current_observation(reason: str) -> bool:
    if reason in {
        "camera_restarted",
        "map_changed",
        "stale_camera_generation",
        "stale_camera_sample",
        "stale_context_sample",
        "stale_frame",
        "stale_navigation_state",
    }:
        return False
    return not reason.endswith(("_expired", "_buffer_overflow"))


__all__ = ["PerceptionModule"]
