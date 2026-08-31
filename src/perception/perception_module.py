from __future__ import annotations

import logging
import threading
from collections.abc import Mapping
from typing import Any

from perception.backend_manager import BackendManager
from perception.services import DetectionService, SceneGraphService, TrackingService
from runtime import In, Module, Out
from runtime.backend_status import BackendStatus, require_backend
from runtime.config import get_config
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import Detection3D as CoreDetection3D
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import CameraIntrinsics, Image
from runtime.registry import list_plugins, register
from runtime.runtime_interface import map_frame_id
from runtime.tf import map_from_odom_transform_from_mapping

logger = logging.getLogger(__name__)
PERCEPTION_MAP_FRAME_ID = map_frame_id()


@register("detector", "yoloe", description="YOLO-E open-vocabulary instance detector")
class _YOLOEDetectorProvider:
    label = "YOLOEDetector"

    @staticmethod
    def create(module):
        from perception.detection.yoloe_detector import YOLOEDetector

        return YOLOEDetector(
            model_size=module._detector_model_size,
            confidence=module._confidence_threshold,
            iou_threshold=module._detector_iou_threshold,
            device=module._detector_device,
            max_detections=module._detector_max_detections,
        )


@register("detector", "yolo_world", description="YOLO-World open-vocabulary detector")
class _YOLOWorldDetectorProvider:
    label = "YOLOWorldDetector"

    @staticmethod
    def create(module):
        from perception.detection.yolo_world_detector import (
            YOLOWorldDetector,
        )

        return YOLOWorldDetector(
            model_size=module._detector_model_size,
            confidence=module._confidence_threshold,
            iou_threshold=module._detector_iou_threshold,
            device=module._detector_device,
        )


@register("detector", "bpu", description="D-Robotics Nash BPU detector")
class _BPUDetectorProvider:
    label = "BPUDetector"

    @staticmethod
    def create(module):
        from perception.detection.bpu_detector import BPUDetector

        return BPUDetector(
            model_path=module._detector_model_path,
            confidence=module._confidence_threshold,
            iou_threshold=module._detector_iou_threshold,
            max_detections=module._detector_max_detections,
            min_box_size_px=module._detector_min_box_size_px,
        )


@register("detector", "sim_scene", description="Simulation scene observer")
class _SimSceneDetectorProvider:
    label = "SimSceneObserver"

    @staticmethod
    def create(module):
        from perception.detection.sim_scene_observer import (
            SimSceneObserver,
        )

        observer = SimSceneObserver(
            world=module._world,
            scenario_entities=module._scenario_entities,
        )
        module._sim_scene_observer = observer
        return observer


@register("encoder", "clip", description="CLIP image/text encoder")
class _CLIPEncoderProvider:
    label = "CLIPEncoder"

    @staticmethod
    def create(_module):
        from perception.encoding.clip_encoder import CLIPEncoder

        return CLIPEncoder()


@register("encoder", "mobileclip", description="MobileCLIP text encoder")
class _MobileCLIPEncoderProvider:
    label = "MobileCLIPEncoder"

    @staticmethod
    def create(_module):
        from perception.encoding.mobileclip_encoder import (
            MobileCLIPEncoder,
        )

        return MobileCLIPEncoder()


@register("perception_tracker", "bpu", description="BPU detector track-id adapter")
class _BPUTrackerProvider:
    label = "BPUTracker"

    @staticmethod
    def create(module):
        from perception.tracking.bpu_tracker import BPUTracker

        return BPUTracker(module._detector, tracker_type="botsort")


@register("perception_tracker", "instance", description="3D instance scene-graph tracker")
class _InstanceTrackerProvider:
    label = "InstanceTracker"

    @staticmethod
    def create(module):
        from perception.tracking.instance_tracker import InstanceTracker

        return InstanceTracker(
            merge_distance=module._merge_distance,
            iou_threshold=module._tracking_iou_threshold,
            max_objects=module._max_objects,
        )


@register("perception", "scene", description="RGB-D semantic scene perception module")
class PerceptionModule(Module, layer=3):
    """Semantic perception: YOLO detection + CLIP encoding + scene graph tracking.

    Thin Module shell around existing algorithms in perception.
    All heavy lifting is done by InstanceTracker, DetectorBase subclasses,
    CLIPEncoder / MobileCLIPEncoder, and the projection module.
    """

    # -- input ports --
    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraIntrinsics]
    odometry: In[Odometry]
    map_odom_tf: In[dict]

    # -- output ports --
    scene_graph: Out[SceneGraph]
    detections_3d: Out[list]  # list[CoreDetection3D]
    robot_pose: Out[PoseStamped]

    class _CandidateModuleView:
        def __init__(
            self,
            module: PerceptionModule,
            *,
            detector_type: str | None = None,
            encoder_type: str | None = None,
            detector: Any | None = None,
        ) -> None:
            self._module = module
            self._sim_scene_observer = None
            if detector_type is not None:
                self._detector_type = detector_type
            if encoder_type is not None:
                self._encoder_type = encoder_type
            if detector is not None:
                self._detector = detector

        def __getattr__(self, name: str) -> Any:
            return getattr(self._module, name)

    def __init__(
        self,
        detector_type: str = "yoloe",
        encoder_type: str = "none",
        merge_distance: float | None = None,
        confidence_threshold: float | None = None,
        max_depth: float | None = None,
        min_depth: float | None = None,
        depth_scale: float | None = None,
        laplacian_threshold: float | None = None,
        max_objects: int | None = None,
        default_classes: str | None = None,
        skip_frames: int | None = None,
        world: str = "",
        scenario_entities: list[Mapping[str, Any]] | tuple[Mapping[str, Any], ...] = (),
        tracking_iou_threshold: float | None = None,
        detector_iou_threshold: float | None = None,
        detector_max_detections: int | None = None,
        detector_min_box_size_px: int | None = None,
        detector_model_size: str | None = None,
        detector_device: str = "",
        detector_model_path: str = "",
        max_rgbd_skew_s: float = 0.05,
        max_odom_age_s: float = 0.10,
        max_map_odom_age_s: float = 0.50,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        require_backend("detector", detector_type, list_plugins("detector"))
        if encoder_type != "none":
            require_backend("encoder", encoder_type, list_plugins("encoder"))

        # Resolve tunable defaults from robot_config.yaml; explicit kwargs win.
        cfg = get_config().perception
        det_cfg = cfg.detector
        track_cfg = cfg.tracking

        self._detector_type = detector_type
        self._encoder_type = encoder_type
        self._detector_status = BackendStatus.configured_as(detector_type)
        self._encoder_status = BackendStatus.configured_as(encoder_type)
        self._merge_distance = merge_distance if merge_distance is not None else track_cfg.merge_distance
        self._confidence_threshold = (
            confidence_threshold if confidence_threshold is not None else det_cfg.confidence_threshold
        )
        self._max_depth = max_depth if max_depth is not None else cfg.max_depth
        self._min_depth = min_depth if min_depth is not None else cfg.min_depth
        self._depth_scale = depth_scale if depth_scale is not None else cfg.depth_scale
        self._laplacian_threshold = laplacian_threshold if laplacian_threshold is not None else cfg.laplacian_threshold
        self._max_objects = max_objects if max_objects is not None else track_cfg.max_objects
        self._default_classes = default_classes if default_classes is not None else cfg.default_classes
        self._skip_frames = max(skip_frames if skip_frames is not None else cfg.skip_frames, 1)
        self._world = world
        self._scenario_entities = tuple(scenario_entities)
        self._tracking_iou_threshold = (
            tracking_iou_threshold if tracking_iou_threshold is not None else track_cfg.iou_threshold
        )
        self._detector_iou_threshold = (
            detector_iou_threshold if detector_iou_threshold is not None else det_cfg.iou_threshold
        )
        self._detector_max_detections = (
            detector_max_detections if detector_max_detections is not None else det_cfg.max_detections
        )
        self._detector_min_box_size_px = (
            detector_min_box_size_px if detector_min_box_size_px is not None else det_cfg.min_box_size_px
        )
        self._detector_model_size = detector_model_size if detector_model_size is not None else det_cfg.model_size
        self._detector_device = detector_device
        self._detector_model_path = detector_model_path
        self._max_rgbd_skew_s = max(0.0, float(max_rgbd_skew_s))
        self._max_odom_age_s = max(0.0, float(max_odom_age_s))
        self._max_map_odom_age_s = max(0.0, float(max_map_odom_age_s))

        # Camera extrinsics from factory calibration. Matrix construction uses
        # NumPy, so defer it until a frame is actually processed.
        self._camera_config = get_config().camera
        self._T_camera_body = None

        # runtime state (populated during setup)
        self._tracker = None
        self._detector = None
        self._clip_encoder = None
        self._sim_scene_observer = None
        self._detector_tracker = None
        self._detector_tracker_warning_logged = False
        self._frame_count: int = 0
        self._latest_depth: np.ndarray | None = None
        self._latest_depth_ts: float = 0.0
        self._latest_depth_frame_id: str = ""
        self._latest_intrinsics: Any | None = None
        self._latest_odom_matrix: np.ndarray | None = None
        self._latest_source_odom_matrix: np.ndarray | None = None
        self._latest_odom_ts: float = 0.0
        self._latest_odom_frame_id: str = ""
        self._latest_map_odom_matrix: np.ndarray | None = None
        self._latest_map_odom_child_frame_id: str = ""
        self._latest_map_odom_ts: float = 0.0
        self._latest_core_detections: list[CoreDetection3D] = []
        self._dropped_missing_odom_frames = 0
        self._dropped_unsynced_frames = 0
        self._last_drop_reason = ""
        self._sample_lock = threading.RLock()
        self._backend_lock = threading.RLock()
        self._backend_manager = BackendManager(self)
        self._detection_service = DetectionService(
            confidence_threshold=self._confidence_threshold,
            depth_scale=self._depth_scale,
            min_depth=self._min_depth,
            max_depth=self._max_depth,
        )
        self._tracking_service = TrackingService()
        self._scene_graph_service = SceneGraphService()

    def _camera_body_transform(self):
        if self._T_camera_body is None:
            self._T_camera_body = self._camera_config.T_camera_body
        return self._T_camera_body

    # == Lifecycle =============================================================

    def setup(self) -> None:
        """Lazy-import real algorithms, register port subscriptions.

        All backends are resolved through a single registry path via
        BackendManager.  The old factory-vs-direct split has been removed.
        """
        self._backend_manager.setup()
        self._tracking_service.tracker = self._tracker
        self._scene_graph_service.tracker = self._tracker

        # Wire port callbacks. Use "latest" on high-rate camera ports so slow
        # detector/CLIP inference can't block the camera publisher (and in turn
        # starve uvicorn of the GIL, which hangs the Gateway).
        self.color_image.subscribe(self._on_color_frame)
        self.color_image.set_policy("latest")
        self.depth_image.subscribe(self._on_depth)
        self.depth_image.set_policy("latest")
        self.camera_info.subscribe(self._on_camera_info)
        self.odometry.subscribe(self._on_odometry)
        self.map_odom_tf.subscribe(self._on_map_odom_tf)

        # Sanity-check factory calibration
        self._check_calibration()

    def _check_calibration(self) -> None:
        """Warn at startup if camera calibration looks wrong."""
        cfg = get_config().camera
        # Camera position sanity: should be within reasonable robot body bounds
        pos = (cfg.position_x, cfg.position_y, cfg.position_z)
        if all(v == 0.0 for v in pos):
            logger.warning(
                "Camera extrinsics position is all zeros 鈥?3D projections will be wrong. "
                "Check the selected RobotConfig camera section."
            )
        if abs(cfg.position_x) > 2.0 or abs(cfg.position_y) > 2.0 or abs(cfg.position_z) > 3.0:
            logger.warning(
                "Camera position (%.2f, %.2f, %.2f) looks too large for a quadruped. "
                "Check the selected RobotConfig camera section.",
                cfg.position_x,
                cfg.position_y,
                cfg.position_z,
            )
        # Default intrinsics check
        if cfg.fx <= 0 or cfg.fy <= 0:
            logger.warning(
                "Camera default intrinsics have non-positive focal length (fx=%.1f, fy=%.1f). "
                "Runtime CameraInfo must provide valid values.",
                cfg.fx,
                cfg.fy,
            )
        # LiDAR sanity
        lidar_cfg = get_config().lidar
        offset_mag = (lidar_cfg.offset_x**2 + lidar_cfg.offset_y**2 + lidar_cfg.offset_z**2) ** 0.5
        if offset_mag > 1.0:
            logger.warning(
                "LiDAR offset magnitude %.3fm seems large for body-mounted sensor. "
                "Check the selected RobotConfig lidar section.",
                offset_mag,
            )
        logger.info(
            "Calibration OK: camera at (%.3f, %.3f, %.3f), LiDAR offset %.4fm",
            cfg.position_x,
            cfg.position_y,
            cfg.position_z,
            offset_mag,
        )

    def stop(self) -> None:
        """Release GPU resources."""
        self._backend_manager.stop()
        super().stop()

    # == Port callbacks ========================================================

    def _on_depth(self, img: Image) -> None:
        """Cache latest depth frame."""
        with self._sample_lock:
            self._latest_depth = img.data
            self._latest_depth_ts = float(getattr(img, "ts", 0.0) or 0.0)
            self._latest_depth_frame_id = str(getattr(img, "frame_id", "") or "")

    def _on_camera_info(self, intrinsics: CameraIntrinsics) -> None:
        """Cache intrinsics (one-shot, convert to projection.CameraIntrinsics)."""
        with self._sample_lock:
            if self._latest_intrinsics is not None:
                return
            try:
                from perception.tracking.projection import (
                    CameraIntrinsics as ProjIntrinsics,
                )

                self._latest_intrinsics = ProjIntrinsics(
                    fx=intrinsics.fx,
                    fy=intrinsics.fy,
                    cx=intrinsics.cx,
                    cy=intrinsics.cy,
                    width=intrinsics.width,
                    height=intrinsics.height,
                )
            except ImportError:
                self._latest_intrinsics = intrinsics
        logger.info(
            "Camera intrinsics received: fx=%.1f, fy=%.1f, %dx%d",
            intrinsics.fx,
            intrinsics.fy,
            intrinsics.width,
            intrinsics.height,
        )

    def _on_odometry(self, odom: Odometry) -> None:
        """Cache the source-frame body pose and resolve it into ``map``."""
        pos = odom.pose.position
        q = odom.pose.orientation
        rot = _quat_to_rotation(q.x, q.y, q.z, q.w)
        mat = np.eye(4)
        mat[:3, :3] = rot
        mat[:3, 3] = [pos.x, pos.y, pos.z]
        with self._sample_lock:
            self._latest_source_odom_matrix = mat
            self._latest_odom_ts = float(getattr(odom, "ts", 0.0) or 0.0)
            self._latest_odom_frame_id = str(getattr(odom, "frame_id", "") or "")
            self._resolve_latest_map_pose()

    def _on_map_odom_tf(self, raw: Mapping[str, Any]) -> None:
        """Cache ``T_map_from_odom``: odom pose in map coordinates."""
        parsed = _map_odom_matrix(raw)
        with self._sample_lock:
            if parsed is None:
                self._latest_map_odom_matrix = None
                self._latest_map_odom_child_frame_id = ""
                self._latest_map_odom_ts = 0.0
            else:
                (
                    self._latest_map_odom_matrix,
                    self._latest_map_odom_child_frame_id,
                    self._latest_map_odom_ts,
                ) = parsed
            self._resolve_latest_map_pose()

    def _resolve_latest_map_pose(self) -> None:
        source = self._latest_source_odom_matrix
        if source is None:
            self._latest_odom_matrix = None
            return
        if self._latest_odom_frame_id == PERCEPTION_MAP_FRAME_ID:
            self._latest_odom_matrix = source
            return
        if (
            self._latest_odom_frame_id == self._latest_map_odom_child_frame_id
            and self._latest_map_odom_matrix is not None
        ):
            self._latest_odom_matrix = self._latest_map_odom_matrix @ source
            return
        self._latest_odom_matrix = None

    def _on_color_frame(self, img: Image) -> None:
        """Main callback -- each RGB frame triggers the full pipeline."""
        self._frame_count += 1
        if self._frame_count % self._skip_frames != 0:
            return
        with self._sample_lock:
            intrinsics = self._latest_intrinsics
            depth = self._latest_depth
            depth_ts = self._latest_depth_ts
            depth_frame_id = self._latest_depth_frame_id
            source_odom = self._latest_source_odom_matrix
            tf_body_world = self._latest_odom_matrix
            odom_ts = self._latest_odom_ts
            odom_frame_id = self._latest_odom_frame_id
            map_odom_ts = self._latest_map_odom_ts

        if intrinsics is None or depth is None:
            self._last_drop_reason = "missing_rgbd_calibration"
            self._publish_empty_scene_graph(PERCEPTION_MAP_FRAME_ID, self._source_timestamp_or_zero(img))
            return

        if source_odom is None:
            self._dropped_missing_odom_frames += 1
            self._last_drop_reason = "missing_odometry"
            self._publish_empty_scene_graph(PERCEPTION_MAP_FRAME_ID, self._source_timestamp_or_zero(img))
            return
        if not self._samples_are_synchronized(
            img,
            depth_ts=depth_ts,
            depth_frame_id=depth_frame_id,
            odom_matrix=tf_body_world,
            odom_ts=odom_ts,
            odom_frame_id=odom_frame_id,
            map_odom_ts=map_odom_ts,
        ):
            self._dropped_unsynced_frames += 1
            self._publish_empty_scene_graph(PERCEPTION_MAP_FRAME_ID, self._source_timestamp_or_zero(img))
            return
        if tf_body_world is None:
            self._dropped_unsynced_frames += 1
            self._last_drop_reason = "missing_map_odom_transform"
            self._publish_empty_scene_graph(PERCEPTION_MAP_FRAME_ID, self._source_timestamp_or_zero(img))
            return
        source_ts = self._source_timestamp_or_zero(img)
        self._publish_robot_pose(tf_body_world, source_ts)
        # T_map_from_camera = T_map_from_body @ T_body_from_camera.
        tf_camera_world = tf_body_world @ self._camera_body_transform()

        with self._backend_lock:
            try:
                self._process_frame(
                    img,
                    depth,
                    intrinsics,
                    tf_camera_world,
                    PERCEPTION_MAP_FRAME_ID,
                    source_ts,
                )
            except Exception as e:
                self._last_drop_reason = "frame_processing_error"
                self._publish_empty_scene_graph(PERCEPTION_MAP_FRAME_ID, self._source_timestamp_or_zero(img))
                logger.error("Frame processing error (frame=%d): %s", self._frame_count, e)

    def _publish_robot_pose(self, matrix: np.ndarray, source_ts: float) -> None:
        yaw = float(np.arctan2(matrix[1, 0], matrix[0, 0]))
        self.robot_pose.publish(
            PoseStamped(
                pose=Pose(
                    position=Vector3(
                        float(matrix[0, 3]),
                        float(matrix[1, 3]),
                        float(matrix[2, 3]),
                    ),
                    orientation=Quaternion.from_yaw(yaw),
                ),
                ts=source_ts,
                frame_id=PERCEPTION_MAP_FRAME_ID,
            )
        )

    # == Core pipeline =========================================================

    def _source_timestamp_or_zero(self, msg: Any) -> float:
        try:
            value = float(getattr(msg, "ts", 0.0) or 0.0)
        except (TypeError, ValueError):
            return 0.0
        return value if np.isfinite(value) and value > 0.0 else 0.0

    def _samples_are_synchronized(
        self,
        color_img: Image,
        *,
        depth_ts: float,
        depth_frame_id: str,
        odom_matrix: np.ndarray | None,
        odom_ts: float,
        odom_frame_id: str,
        map_odom_ts: float,
    ) -> bool:
        color_ts = float(getattr(color_img, "ts", 0.0) or 0.0)
        color_frame_id = str(getattr(color_img, "frame_id", "") or "")
        if color_frame_id and depth_frame_id and color_frame_id != depth_frame_id:
            self._last_drop_reason = "rgb_depth_frame_mismatch"
            return False
        if odom_frame_id not in {PERCEPTION_MAP_FRAME_ID, "odom"}:
            self._last_drop_reason = "odometry_frame_unsupported"
            return False
        if odom_matrix is None:
            self._last_drop_reason = "missing_map_odom_transform"
            return False
        if (
            not np.isfinite(color_ts)
            or not np.isfinite(depth_ts)
            or not np.isfinite(odom_ts)
            or color_ts <= 0.0
            or depth_ts <= 0.0
            or odom_ts <= 0.0
        ):
            self._last_drop_reason = "invalid_sample_timestamp"
            return False
        if abs(color_ts - depth_ts) > self._max_rgbd_skew_s:
            self._last_drop_reason = "rgb_depth_time_skew"
            return False
        if abs(color_ts - odom_ts) > self._max_odom_age_s:
            self._last_drop_reason = "odom_time_skew"
            return False
        if odom_frame_id == "odom":
            if not np.isfinite(map_odom_ts) or map_odom_ts <= 0.0:
                self._last_drop_reason = "invalid_map_odom_timestamp"
                return False
            if abs(color_ts - map_odom_ts) > self._max_map_odom_age_s:
                self._last_drop_reason = "map_odom_time_skew"
                return False
        self._last_drop_reason = ""
        return True

    def _publish_empty_scene_graph(self, frame_id: str, source_ts: float = 0.0) -> None:
        self._latest_core_detections = []
        self.detections_3d.publish([])
        self.scene_graph.publish(
            SceneGraph(objects=[], relations=[], regions=[], frame_id=frame_id, ts=source_ts)
        )

    def _publish_negative_observation(
        self,
        tf_camera_to_world: np.ndarray,
        intrinsics: Any,
        frame_id: str,
        source_ts: float,
    ) -> None:
        """Advance tracking only after a valid synchronized frame saw no objects."""
        if self._tracker is not None:
            self._tracking_service.update(
                [],
                camera_pos=tf_camera_to_world[:3, 3],
                camera_forward=tf_camera_to_world[:3, 2],
                intrinsics_fx=float(getattr(intrinsics, "fx", 0.0)),
            )
        self._publish_empty_scene_graph(frame_id, source_ts)

    def _process_frame(
        self,
        color_img: Image,
        depth: np.ndarray,
        intrinsics: Any,
        tf_camera_to_world: np.ndarray,
        output_frame_id: str,
        source_ts: float,
    ) -> None:
        """USS-Nav style single-frame processing: detect -> project -> track -> publish."""
        bgr = color_img.to_bgr().data if hasattr(color_img, "to_bgr") else color_img.data

        # Laplacian blur detection is useful for real RGB streams, but the sim-scene
        # backend derives detections from world metadata and should not be blocked by image sharpness.
        if self._sim_scene_observer is None:
            try:
                from perception.detection.laplacian_filter import is_blurry

                if is_blurry(bgr, threshold=self._laplacian_threshold):
                    self._publish_empty_scene_graph(output_frame_id, source_ts)
                    return
            except ImportError:
                pass

        detections_3d = []
        if self._sim_scene_observer is not None:
            detections_3d = self._sim_scene_observer.observe(
                tf_camera_to_world=tf_camera_to_world,
                intrinsics=intrinsics,
                text_prompt=self._default_classes,
            )
            if not detections_3d:
                self._publish_negative_observation(
                    tf_camera_to_world, intrinsics, output_frame_id, source_ts
                )
                return
        else:
            detections_2d = self._run_detector(bgr)
            if detections_2d is None:
                self._publish_empty_scene_graph(output_frame_id, source_ts)
                return
            if not detections_2d:
                self._publish_negative_observation(
                    tf_camera_to_world, intrinsics, output_frame_id, source_ts
                )
                return
            detections_3d = self._detection_service.project_to_3d(
                detections_2d,
                depth,
                tf_camera_to_world,
                intrinsics,
            )
        if not detections_3d:
            self._publish_empty_scene_graph(output_frame_id, source_ts)
            return

        # Instance tracking
        if self._tracker is not None:
            cam_pos = tf_camera_to_world[:3, 3]
            cam_fwd = tf_camera_to_world[:3, 2]
            fx = getattr(intrinsics, "fx", 0.0)
            self._tracking_service.update(
                detections_3d,
                camera_pos=cam_pos,
                camera_forward=cam_fwd,
                intrinsics_fx=fx,
            )

        # Publish detections
        core_dets = self._detection_service.convert_to_core_detections(
            detections_3d,
            source_ts=source_ts,
        )
        self._latest_core_detections = core_dets
        self.detections_3d.publish(core_dets)

        # Publish scene graph
        sg = self._scene_graph_service.build_scene_graph(
            self._latest_core_detections,
            output_frame_id,
            tracker=self._tracker,
        )
        if source_ts > 0.0:
            sg.ts = source_ts
        self.scene_graph.publish(sg)

    # == Backend lifecycle (delegated to BackendManager) =======================

    def reconfigure_backend(
        self,
        category: str,
        backend: str,
        **config: Any,
    ) -> dict[str, Any]:
        """Hot-swap a detector or encoder backend at runtime."""
        if category == "detector":
            return self._backend_manager.reconfigure_detector(backend, **config)
        if category == "encoder":
            return self._backend_manager.reconfigure_encoder(backend, **config)
        return super().reconfigure_backend(category, backend, **config)

    def _run_detector(self, bgr: np.ndarray) -> list | None:
        """Run detector, return Detection2D list."""
        if self._detector_tracker is not None:
            try:
                return self._detector_tracker.track(bgr, self._default_classes)
            except Exception as e:
                if not self._detector_tracker_warning_logged:
                    logger.warning("2D detector tracking failed (%s) -- using raw detections", e)
                    self._detector_tracker_warning_logged = True
        if self._detector is None:
            self._last_drop_reason = "detector_unavailable"
            return None
        try:
            return self._detector.detect(bgr, self._default_classes)
        except Exception as e:
            logger.warning("Detection failed: %s", e)
            self._last_drop_reason = "detector_error"
            return None

    # == Query API =============================================================

    # == Health ================================================================

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["frame_count"] = self._frame_count
        info["detector_type"] = self._detector_type
        info["encoder_type"] = self._encoder_type
        info["detector_ready"] = (
            self._detector is not None or self._sim_scene_observer is not None
        )
        info["detector_tracker_ready"] = self._detector_tracker is not None
        if self._detector_tracker is not None:
            info["detector_tracker_backend"] = getattr(self._detector_tracker, "backend_name", "unknown")
            info["person_tracking"] = getattr(self._detector_tracker, "person_counts", {})
        info["encoder_ready"] = self._clip_encoder is not None
        info["tracker_ready"] = self._tracker is not None
        info["detector"] = self._detector_status.as_health_fields()
        info["encoder"] = self._encoder_status.as_health_fields()
        tracked = 0
        if self._tracker is not None:
            try:
                tracked = len(self._tracker.get_objects())
            except (AttributeError, TypeError):
                pass
        info["tracked_objects"] = tracked
        info["latest_detections"] = len(self._latest_core_detections)
        info["dropped_missing_odom_frames"] = self._dropped_missing_odom_frames
        info["dropped_unsynced_frames"] = self._dropped_unsynced_frames
        info["last_drop_reason"] = self._last_drop_reason
        return info

    # == Query API =============================================================

    @property
    def tracker(self):
        """Access underlying InstanceTracker (for external queries / tests)."""
        return self._tracker

    @property
    def image_encoder(self):
        """Return the shared image encoder used by semantic consumers."""
        encoder = self._clip_encoder
        if (
            encoder is not None
            and callable(getattr(encoder, "encode_text", None))
            and callable(getattr(encoder, "encode_image", None))
        ):
            return encoder
        return None

    @property
    def frame_count(self) -> int:
        return self._frame_count


# == Utility ===================================================================


def _quat_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Quaternion -> 3x3 rotation matrix."""
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _map_odom_matrix(raw: Mapping[str, Any]) -> tuple[np.ndarray, str, float] | None:
    """Parse canonical ``T_map_from_odom`` for the perception pipeline."""
    transform = map_from_odom_transform_from_mapping(raw)
    if transform is None:
        return None
    return transform.to_matrix(), transform.child_frame_id, transform.ts
