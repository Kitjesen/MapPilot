"""perception_module.py -- semantic perception Module wrapper (core framework).

Wraps the PURE ALGORITHM parts of SemanticPerceptionNode as a Module,
fully decoupled from ROS2.  No algorithm is rewritten -- all real work
is delegated to lazy imports from the semantic_perception package.

Port contract:
  In:  color_image, depth_image, camera_info, odometry
  Out: scene_graph, detections_3d

Lifecycle: __init__ -> setup() -> start() -> [color_image drives pipeline] -> stop()
"""

from __future__ import annotations

import json
import logging
import threading
from typing import Any

from core import In, Module, Out
from core.backend_status import BackendStatus, require_backend
from core.config import get_config
from core.msgs.numpy_compat import np
from core.msgs.geometry import Vector3
from core.msgs.nav import Odometry
from core.msgs.semantic import (
    Detection3D as CoreDetection3D,
)
from core.msgs.semantic import (
    Region,
    Relation,
    SceneGraph,
)
from core.msgs.sensor import CameraIntrinsics, Image
from core.registry import get, list_plugins, register
from core.runtime_interface import map_frame_id

logger = logging.getLogger(__name__)
PERCEPTION_MAP_FRAME_ID = map_frame_id()


_PERCEPTION_DETECTOR_BACKENDS = ("yoloe", "yolo_world", "bpu", "sim_scene")
_PERCEPTION_ENCODER_BACKENDS = ("clip", "mobileclip")


@register("detector", "yoloe", description="YOLO-E open-vocabulary instance detector")
class _YOLOEDetectorProvider:
    label = "YOLOEDetector"

    @staticmethod
    def create(module):
        from semantic.perception.yoloe_detector import YOLOEDetector

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
        from semantic.perception.yolo_world_detector import (
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
        from semantic.perception.bpu_detector import BPUDetector

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
        from semantic.perception.sim_scene_observer import (
            SimSceneObserver,
        )

        observer = SimSceneObserver(world=module._world)
        module._sim_scene_observer = observer
        return observer


@register("encoder", "clip", description="CLIP image/text encoder")
class _CLIPEncoderProvider:
    label = "CLIPEncoder"

    @staticmethod
    def create(_module):
        from semantic.perception.clip_encoder import CLIPEncoder

        return CLIPEncoder()


@register("encoder", "mobileclip", description="MobileCLIP text encoder")
class _MobileCLIPEncoderProvider:
    label = "MobileCLIPEncoder"

    @staticmethod
    def create(_module):
        from semantic.perception.mobileclip_encoder import (
            MobileCLIPEncoder,
        )

        return MobileCLIPEncoder()


@register("perception_tracker", "bpu", description="BPU detector track-id adapter")
class _BPUTrackerProvider:
    label = "BPUTracker"

    @staticmethod
    def create(module):
        from semantic.perception.bpu_tracker import BPUTracker

        return BPUTracker(module._detector, tracker_type="botsort")


@register("perception", "scene", description="RGB-D semantic scene perception module")
class PerceptionModule(Module, layer=3):
    _run_in_worker = True
    _worker_group = "perception"
    """Semantic perception: YOLO detection + CLIP encoding + scene graph tracking.

    Thin Module shell around existing algorithms in semantic_perception.
    All heavy lifting is done by InstanceTracker, DetectorBase subclasses,
    CLIPEncoder / MobileCLIPEncoder, and the projection module.
    """

    # -- input ports --
    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraIntrinsics]
    odometry: In[Odometry]

    # -- output ports --
    scene_graph: Out[SceneGraph]
    detections_3d: Out[list]  # list[CoreDetection3D]

    # Backends that api/factory.py's PerceptionFactory supports natively.
    # All others (yoloe, bpu, sim_scene) fall back to the direct-import path.
    _FACTORY_DETECTORS = {"yolo_world"}
    _FACTORY_ENCODERS = {"clip"}

    class _CandidateModuleView:
        def __init__(
            self,
            module: "PerceptionModule",
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
        encoder_type: str = "mobileclip",
        merge_distance: float = 0.5,
        confidence_threshold: float = 0.3,
        max_depth: float = 6.0,
        min_depth: float = 0.3,
        depth_scale: float = 0.001,
        laplacian_threshold: float = 100.0,
        max_objects: int = 200,
        default_classes: str = "door . chair . person . desk . stairs . elevator . sign",
        skip_frames: int = 1,
        world: str = "",
        tracking_iou_threshold: float = 0.3,
        detector_iou_threshold: float = 0.45,
        detector_max_detections: int = 64,
        detector_min_box_size_px: int = 12,
        detector_model_size: str = "l",
        detector_device: str = "",
        detector_model_path: str = "",
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        require_backend("detector", detector_type, list_plugins("detector"))
        require_backend("encoder", encoder_type, list_plugins("encoder"))
        self._detector_type = detector_type
        self._encoder_type = encoder_type
        self._detector_status = BackendStatus.configured_as(detector_type)
        self._encoder_status = BackendStatus.configured_as(encoder_type)
        self._merge_distance = merge_distance
        self._confidence_threshold = confidence_threshold
        self._max_depth = max_depth
        self._min_depth = min_depth
        self._depth_scale = depth_scale
        self._laplacian_threshold = laplacian_threshold
        self._max_objects = max_objects
        self._default_classes = default_classes
        self._skip_frames = max(skip_frames, 1)
        self._world = world
        self._tracking_iou_threshold = tracking_iou_threshold
        self._detector_iou_threshold = detector_iou_threshold
        self._detector_max_detections = detector_max_detections
        self._detector_min_box_size_px = detector_min_box_size_px
        self._detector_model_size = detector_model_size
        self._detector_device = detector_device
        self._detector_model_path = detector_model_path

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
        self._latest_intrinsics: Any | None = None
        self._latest_odom_matrix: np.ndarray | None = None
        self._latest_core_detections: list[CoreDetection3D] = []
        self._backend_lock = threading.RLock()

    def _camera_body_transform(self):
        if self._T_camera_body is None:
            self._T_camera_body = self._camera_config.T_camera_body
        return self._T_camera_body

    # == Lifecycle =============================================================

    def setup(self) -> None:
        """Lazy-import real algorithms, register port subscriptions.

        Components are created through two paths depending on detector_type:
          - Factory path  (yolo_world / clip):  api/factory.py → impl/ adapters
          - Direct path   (yoloe / bpu / sim_scene / mobileclip): legacy lazy imports

        Both paths produce objects with the same duck-typed interface (detect /
        load_model / shutdown for detectors; encode_text / load_model / shutdown
        for encoders), so the rest of the module is unchanged.
        """
        if self._detector_type in self._FACTORY_DETECTORS:
            self._setup_via_factory()
        else:
            self._setup_direct()

        # Wire port callbacks. Use "latest" on high-rate camera ports so slow
        # detector/CLIP inference can't block the camera publisher (and in turn
        # starve uvicorn of the GIL, which hangs the Gateway).
        self.color_image.subscribe(self._on_color_frame)
        self.color_image.set_policy("latest")
        self.depth_image.subscribe(self._on_depth)
        self.depth_image.set_policy("latest")
        self.camera_info.subscribe(self._on_camera_info)
        self.odometry.subscribe(self._on_odometry)

        # Sanity-check factory calibration
        self._check_calibration()

    def _check_calibration(self) -> None:
        """Warn at startup if camera calibration looks wrong."""
        cfg = get_config().camera
        # Camera position sanity: should be within reasonable robot body bounds
        pos = (cfg.position_x, cfg.position_y, cfg.position_z)
        if all(v == 0.0 for v in pos):
            logger.warning(
                "Camera extrinsics position is all zeros — 3D projections will be wrong. "
                "Check config/robot_config.yaml camera section."
            )
        if abs(cfg.position_x) > 2.0 or abs(cfg.position_y) > 2.0 or abs(cfg.position_z) > 3.0:
            logger.warning(
                "Camera position (%.2f, %.2f, %.2f) looks too large for a quadruped. "
                "Check config/robot_config.yaml camera section.",
                cfg.position_x, cfg.position_y, cfg.position_z,
            )
        # Default intrinsics check
        if cfg.fx <= 0 or cfg.fy <= 0:
            logger.warning(
                "Camera default intrinsics have non-positive focal length (fx=%.1f, fy=%.1f). "
                "Runtime CameraInfo must provide valid values.",
                cfg.fx, cfg.fy,
            )
        # LiDAR sanity
        lidar_cfg = get_config().lidar
        offset_mag = (lidar_cfg.offset_x**2 + lidar_cfg.offset_y**2 + lidar_cfg.offset_z**2) ** 0.5
        if offset_mag > 1.0:
            logger.warning(
                "LiDAR offset magnitude %.3fm seems large for body-mounted sensor. "
                "Check config/robot_config.yaml lidar section.",
                offset_mag,
            )
        logger.info(
            "Calibration OK: camera at (%.3f, %.3f, %.3f), LiDAR offset %.4fm",
            cfg.position_x, cfg.position_y, cfg.position_z, offset_mag,
        )

    def _setup_via_factory(self) -> None:
        """Create detector + encoder + tracker through api/factory.py."""
        try:
            from semantic.perception.api.factory import PerceptionFactory
            from semantic.perception.api.types import PerceptionConfig

            cfg = PerceptionConfig(
                detector_type=self._detector_type,
                encoder_type=self._encoder_type if self._encoder_type in self._FACTORY_ENCODERS else "clip",
                confidence_threshold=self._confidence_threshold,
                iou_threshold=self._confidence_threshold,
                merge_distance=self._merge_distance,
                max_depth=self._max_depth,
                min_depth=self._min_depth,
            )

            # Tracker via factory (always "instance")
            self._tracker = PerceptionFactory.create_tracker("instance", cfg)
            logger.info("InstanceTracker created via factory")

            # Detector via factory
            self._detector = PerceptionFactory.create_detector(self._detector_type, cfg)
            self._detector_status.use(self._detector_type, degraded=False)
            logger.info("Detector %r created via factory", self._detector_type)

            # Encoder via factory (only "clip" supported; fall back to mobileclip otherwise)
            enc_type = self._encoder_type if self._encoder_type in self._FACTORY_ENCODERS else "clip"
            self._clip_encoder = PerceptionFactory.create_encoder(enc_type, cfg)
            self._encoder_status.use(
                enc_type,
                reason="factory encoder fallback" if enc_type != self._encoder_type else "",
            )
            logger.info("Encoder %r created via factory", enc_type)

        except Exception as e:
            logger.warning(
                "Factory setup failed (%s) — falling back to direct import path", e
            )
            self._setup_direct()

    def _setup_direct(self) -> None:
        """Create components via direct lazy imports (original path)."""
        # Instance tracker
        try:
            from semantic.perception.instance_tracker import InstanceTracker
            self._tracker = InstanceTracker(
                merge_distance=self._merge_distance,
                iou_threshold=self._tracking_iou_threshold,
                max_objects=self._max_objects,
            )
            logger.info(
                "InstanceTracker initialized (merge_dist=%.2f, iou=%.2f)",
                self._merge_distance,
                self._tracking_iou_threshold,
            )
        except ImportError:
            logger.warning(
                "semantic_perception.instance_tracker not available -- "
                "scene graph tracking disabled"
            )

        # Detector backend (optional -- graceful degrade)
        self._detector = self._init_detector()
        self._detector_tracker = self._init_detector_tracker()

        # Encoder (optional)
        self._clip_encoder = self._init_clip_encoder()

    def stop(self) -> None:
        """Release GPU resources."""
        for attr, name in [("_detector", "Detector"), ("_clip_encoder", "Encoder")]:
            self._dispose_backend(getattr(self, attr, None), name)
        super().stop()

    def _dispose_backend(self, obj: Any | None, name: str) -> None:
        if obj is None:
            return
        for method in ("shutdown", "close", "reset"):
            fn = getattr(obj, method, None)
            if callable(fn):
                try:
                    fn()
                except Exception as e:
                    logger.warning("%s %s() error: %s", name, method, e)
                break

    # == Port callbacks ========================================================

    def _on_depth(self, img: Image) -> None:
        """Cache latest depth frame."""
        self._latest_depth = img.data

    def _on_camera_info(self, intrinsics: CameraIntrinsics) -> None:
        """Cache intrinsics (one-shot, convert to projection.CameraIntrinsics)."""
        if self._latest_intrinsics is not None:
            return
        try:
            from semantic.perception.projection import (
                CameraIntrinsics as ProjIntrinsics,
            )
            self._latest_intrinsics = ProjIntrinsics(
                fx=intrinsics.fx, fy=intrinsics.fy,
                cx=intrinsics.cx, cy=intrinsics.cy,
                width=intrinsics.width, height=intrinsics.height,
            )
        except ImportError:
            self._latest_intrinsics = intrinsics
        logger.info(
            "Camera intrinsics received: fx=%.1f, fy=%.1f, %dx%d",
            intrinsics.fx, intrinsics.fy, intrinsics.width, intrinsics.height,
        )

    def _on_odometry(self, odom: Odometry) -> None:
        """Build 4x4 transform matrix from odometry."""
        pos = odom.pose.position
        q = odom.pose.orientation
        rot = _quat_to_rotation(q.x, q.y, q.z, q.w)
        mat = np.eye(4)
        mat[:3, :3] = rot
        mat[:3, 3] = [pos.x, pos.y, pos.z]
        self._latest_odom_matrix = mat

    def _on_color_frame(self, img: Image) -> None:
        """Main callback -- each RGB frame triggers the full pipeline."""
        self._frame_count += 1
        if self._frame_count % self._skip_frames != 0:
            return
        if self._latest_intrinsics is None or self._latest_depth is None:
            return

        tf_body_world = self._latest_odom_matrix
        if tf_body_world is None:
            # Static fallback when no SLAM/TF — use camera extrinsics as world pose
            tf_camera_world = self._camera_body_transform().copy()
        else:
            # Correct transform chain: T_camera_world = T_body_world @ T_camera_body
            tf_camera_world = tf_body_world @ self._camera_body_transform()

        with self._backend_lock:
            try:
                self._process_frame(img, tf_camera_world)
            except Exception as e:
                logger.error("Frame processing error (frame=%d): %s", self._frame_count, e)

    # == Core pipeline =========================================================

    def _process_frame(self, color_img: Image, tf_camera_to_world: np.ndarray) -> None:
        """USS-Nav style single-frame processing: detect -> project -> track -> publish."""
        bgr = color_img.to_bgr().data if hasattr(color_img, "to_bgr") else color_img.data
        depth = self._latest_depth

        # Laplacian blur detection is useful for real RGB streams, but the sim-scene
        # backend derives detections from world metadata and should not be blocked by image sharpness.
        if self._sim_scene_observer is None:
            try:
                from semantic.perception.laplacian_filter import is_blurry
                if is_blurry(bgr, threshold=self._laplacian_threshold):
                    return
            except ImportError:
                pass

        detections_3d = []
        if self._sim_scene_observer is not None:
            detections_3d = self._sim_scene_observer.observe(
                tf_camera_to_world=tf_camera_to_world,
                intrinsics=self._latest_intrinsics,
                text_prompt=self._default_classes,
            )
        else:
            detections_2d = self._run_detector(bgr)
            if not detections_2d:
                return
            detections_3d = self._project_to_3d(detections_2d, depth, tf_camera_to_world)
        if not detections_3d:
            return

        # Instance tracking
        if self._tracker is not None:
            cam_pos = tf_camera_to_world[:3, 3]
            cam_fwd = tf_camera_to_world[:3, 2]
            fx = getattr(self._latest_intrinsics, "fx", 0.0)
            self._tracker.update(
                detections_3d,
                camera_pos=cam_pos,
                camera_forward=cam_fwd,
                intrinsics_fx=fx,
            )

        # Publish detections
        core_dets = self._convert_detections(detections_3d)
        self._latest_core_detections = core_dets
        self.detections_3d.publish(core_dets)

        # Publish scene graph
        sg = self._build_scene_graph()
        self.scene_graph.publish(sg)

    # == Detector init =========================================================

    def _init_detector(self):
        """Lazy-import detector backend."""
        try:
            det, observer = self._load_detector_candidate(self._detector_type)
            self._sim_scene_observer = observer
            self._detector_status.use(self._detector_type, degraded=False)
            return det
        except KeyError:
            logger.warning(
                "Unknown detector %r. Available: %s",
                self._detector_type,
                list_plugins("detector"),
            )
            self._detector_status.use("unavailable", reason="backend not registered")
        except (ImportError, Exception) as e:
            logger.warning("Detector %r unavailable: %s", self._detector_type, e)
            self._detector_status.use("unavailable", reason=str(e))
        return None

    def _load_detector_candidate(self, backend: str):
        provider = get("detector", backend)
        view = self._CandidateModuleView(self, detector_type=backend)
        det = None
        try:
            det = provider.create(view)
            load_model = getattr(det, "load_model", None)
            if callable(load_model):
                load_model()
            logger.info("%s loaded", getattr(provider, "label", backend))
            return det, view._sim_scene_observer
        except Exception:
            self._dispose_backend(det, "Detector candidate")
            raise

    def _init_detector_tracker(
        self,
        backend: str | None = None,
        detector: Any | None = None,
    ):
        """Create a low-latency 2D tracker for detector outputs when available."""
        backend = self._detector_type if backend is None else backend
        detector = self._detector if detector is None else detector
        if backend != "bpu" or detector is None:
            return None
        try:
            provider = get("perception_tracker", "bpu")
            view = self._CandidateModuleView(self, detector_type=backend, detector=detector)
            tracker = provider.create(view)
            logger.info("BPUTracker loaded: BPU detector outputs will include track_id")
            return tracker
        except Exception as e:
            logger.warning(
                "BPUTracker unavailable (%s) -- falling back to raw BPU detections", e
            )
            return None

    def _init_clip_encoder(self):
        """Lazy-import encoder backend based on encoder_type."""
        try:
            enc = self._load_encoder_candidate(self._encoder_type)
            self._encoder_status.use(self._encoder_type, degraded=False)
            return enc
        except KeyError:
            logger.warning(
                "Unknown encoder %r. Available: %s",
                self._encoder_type,
                list_plugins("encoder"),
            )
            self._encoder_status.use("unavailable", reason="backend not registered")
        except (ImportError, Exception) as e:
            logger.warning("Encoder %r unavailable: %s", self._encoder_type, e)
            self._encoder_status.use("unavailable", reason=str(e))
        return None

    def _load_encoder_candidate(self, backend: str):
        provider = get("encoder", backend)
        view = self._CandidateModuleView(self, encoder_type=backend)
        enc = None
        try:
            enc = provider.create(view)
            load_model = getattr(enc, "load_model", None)
            if callable(load_model):
                load_model()
            logger.info("%s loaded", getattr(provider, "label", backend))
            return enc
        except Exception:
            self._dispose_backend(enc, "Encoder candidate")
            raise

    def reconfigure_backend(
        self,
        category: str,
        backend: str,
        **config: Any,
    ) -> dict[str, Any]:
        if category == "detector":
            return self._reconfigure_detector(backend, **config)
        if category == "encoder":
            return self._reconfigure_encoder(backend, **config)
        return super().reconfigure_backend(category, backend, **config)

    def _reconfigure_detector(self, backend: str, **_config: Any) -> dict[str, Any]:
        try:
            get("detector", backend)
        except KeyError:
            return {
                "ok": False,
                "category": "detector",
                "requested_backend": backend,
                "reason": "unknown_backend",
                "available": list_plugins("detector"),
            }

        try:
            detector, observer = self._load_detector_candidate(backend)
            if detector is None:
                raise RuntimeError("detector unavailable")
            tracker = self._init_detector_tracker(backend, detector)
        except Exception as exc:
            return {
                "ok": False,
                "category": "detector",
                "requested_backend": backend,
                "reason": "backend_reconfigure_failed",
                "error": str(exc),
            }

        with self._backend_lock:
            previous_backend = self._detector_type
            previous_detector = self._detector
            previous_tracker = self._detector_tracker
            previous_observer = self._sim_scene_observer
            self._detector_type = backend
            self._detector_status = BackendStatus.configured_as(backend)
            self._detector = detector
            self._detector_tracker = tracker
            self._sim_scene_observer = observer
            self._detector_status.configured = backend
            self._detector_status.use(backend, degraded=False)

        disposed: list[Any] = []
        for obj, name in (
            (previous_tracker, "Detector tracker"),
            (previous_detector, "Detector"),
            (previous_observer, "SimSceneObserver"),
        ):
            if obj is None or any(obj is seen for seen in disposed):
                continue
            self._dispose_backend(obj, name)
            disposed.append(obj)
        return {
            "ok": True,
            "category": "detector",
            "previous_backend": previous_backend,
            "backend": backend,
            "degraded": False,
        }

    def _reconfigure_encoder(self, backend: str, **_config: Any) -> dict[str, Any]:
        try:
            get("encoder", backend)
        except KeyError:
            return {
                "ok": False,
                "category": "encoder",
                "requested_backend": backend,
                "reason": "unknown_backend",
                "available": list_plugins("encoder"),
            }

        try:
            encoder = self._load_encoder_candidate(backend)
            if encoder is None:
                raise RuntimeError("encoder unavailable")
        except Exception as exc:
            return {
                "ok": False,
                "category": "encoder",
                "requested_backend": backend,
                "reason": "backend_reconfigure_failed",
                "error": str(exc),
            }

        with self._backend_lock:
            previous_backend = self._encoder_type
            previous_encoder = self._clip_encoder
            self._encoder_type = backend
            self._encoder_status = BackendStatus.configured_as(backend)
            self._clip_encoder = encoder
            self._encoder_status.configured = backend
            self._encoder_status.use(backend, degraded=False)

        self._dispose_backend(previous_encoder, "Encoder")
        return {
            "ok": True,
            "category": "encoder",
            "previous_backend": previous_backend,
            "backend": backend,
            "degraded": False,
        }

    def _run_detector(self, bgr: np.ndarray) -> list:
        """Run detector, return Detection2D list."""
        if self._detector_tracker is not None:
            try:
                return self._detector_tracker.track(bgr, self._default_classes)
            except Exception as e:
                if not self._detector_tracker_warning_logged:
                    logger.warning(
                        "2D detector tracking failed (%s) -- using raw detections", e
                    )
                    self._detector_tracker_warning_logged = True
        if self._detector is None:
            return []
        try:
            return self._detector.detect(bgr, self._default_classes)
        except Exception as e:
            logger.warning("Detection failed: %s", e)
            return []

    # == 3D Projection =========================================================

    def _project_to_3d(
        self, detections_2d: list, depth: np.ndarray, tf_camera_to_world: np.ndarray,
    ) -> list:
        """Project 2D detections to 3D (delegates to projection module)."""
        try:
            from semantic.perception.projection import (
                Detection3D as ProjDetection3D,
            )
            from semantic.perception.projection import (
                bbox_center_depth,
                mask_to_pointcloud,
                pointcloud_centroid,
                project_to_3d,
                transform_point,
            )
        except ImportError:
            return self._project_to_3d_fallback(detections_2d, depth, tf_camera_to_world)

        intrinsics = self._latest_intrinsics
        results = []
        for det2d in detections_2d:
            centroid = None
            center_depth = None
            points = None

            # Try mask -> pointcloud (USS-Nav main path)
            if getattr(det2d, "mask", None) is not None:
                points = mask_to_pointcloud(
                    mask=det2d.mask, depth_image=depth,
                    intrinsics=intrinsics, tf_camera_to_world=tf_camera_to_world,
                    depth_scale=self._depth_scale,
                    min_depth=self._min_depth, max_depth=self._max_depth,
                )
                if points is not None and len(points) > 0:
                    centroid = pointcloud_centroid(points)
                    center_depth = float(
                        np.linalg.norm(centroid - tf_camera_to_world[:3, 3])
                    )

            # Fallback: bbox center depth
            if centroid is None:
                d = bbox_center_depth(depth, det2d.bbox, depth_scale=self._depth_scale)
                if d is None or d < self._min_depth or d > self._max_depth:
                    continue
                cx = (det2d.bbox[0] + det2d.bbox[2]) / 2
                cy = (det2d.bbox[1] + det2d.bbox[3]) / 2
                p_camera = project_to_3d(cx, cy, d, intrinsics)
                centroid = transform_point(p_camera, tf_camera_to_world)
                center_depth = d

            results.append(ProjDetection3D(
                position=centroid, label=det2d.label, score=det2d.score,
                bbox_2d=det2d.bbox, depth=center_depth,
                features=getattr(det2d, "features", np.array([])),
                points=points if points is not None else np.empty((0, 3)),
                track_id=getattr(det2d, "track_id", None),
            ))
        return results

    def _project_to_3d_fallback(
        self, detections_2d: list, depth: np.ndarray, tf_camera_to_world: np.ndarray,
    ) -> list:
        """W2-1: masked-depth 3D projection (replaces center-pixel fallback).

        For each 2D detection:
          1. Gather all depth pixels inside the bbox.
          2. Filter invalid (0, non-finite, out of [min_depth, max_depth]).
          3. If fewer than _FALLBACK_MIN_DEPTH_PIXELS valid, drop the detection
             (we refuse to fabricate a 3D position from sparse/unreliable depth).
          4. Take the median of the valid samples as representative depth
             (robust to outliers and reflections).
          5. Back-project the bbox centre pixel using median depth to get a
             camera-frame 3D point; transform to world.
          6. Attach confidence_3d = valid_pixels / total_pixels.
        """
        results = []
        intr = self._latest_intrinsics
        if intr is None:
            return results

        fx = getattr(intr, "fx", 600.0) or 600.0
        fy = getattr(intr, "fy", 600.0) or 600.0
        ccx = getattr(intr, "cx", 320.0)
        ccy = getattr(intr, "cy", 240.0)

        min_valid_pixels = int(getattr(self, "_FALLBACK_MIN_DEPTH_PIXELS", 20))
        h_img, w_img = depth.shape[:2]

        for det2d in detections_2d:
            bbox = det2d.bbox
            x1 = max(0, int(bbox[0]))
            y1 = max(0, int(bbox[1]))
            x2 = min(w_img, int(bbox[2]))
            y2 = min(h_img, int(bbox[3]))
            if x2 <= x1 or y2 <= y1:
                continue

            roi = depth[y1:y2, x1:x2]
            total_pixels = roi.size
            if total_pixels == 0:
                continue

            # Apply depth scale once, then filter valid range
            roi_m = roi.astype(np.float32) * self._depth_scale
            finite_mask = np.isfinite(roi_m) & (roi_m > 0.0)
            range_mask = (roi_m >= self._min_depth) & (roi_m <= self._max_depth)
            valid_mask = finite_mask & range_mask
            valid_depths = roi_m[valid_mask]

            if valid_depths.size < min_valid_pixels:
                continue

            d_median = float(np.median(valid_depths))
            confidence_3d = float(valid_depths.size) / float(total_pixels)

            # Back-project bbox centre using median depth
            px = (bbox[0] + bbox[2]) / 2.0
            py = (bbox[1] + bbox[3]) / 2.0
            p_cam = np.array([
                (px - ccx) * d_median / fx,
                (py - ccy) * d_median / fy,
                d_median,
            ])
            p_world = (tf_camera_to_world @ np.array([*p_cam, 1.0]))[:3]

            # Approximate 3D extent from bbox pixel extent × depth / focal
            bbox_w_px = max(1.0, float(bbox[2] - bbox[0]))
            bbox_h_px = max(1.0, float(bbox[3] - bbox[1]))
            width_3d = bbox_w_px * d_median / fx
            height_3d = bbox_h_px * d_median / fy

            class _FallbackDet3D:
                """Minimal duck-typed Detection3D for fallback path."""
                pass

            det3d = _FallbackDet3D()
            det3d.position = p_world
            det3d.label = det2d.label
            det3d.score = det2d.score
            det3d.bbox_2d = bbox
            det3d.depth = d_median
            det3d.features = getattr(det2d, "features", np.array([]))
            det3d.points = np.empty((0, 3))
            det3d.track_id = getattr(det2d, "track_id", None)
            det3d.confidence_3d = confidence_3d
            det3d.width_3d = width_3d
            det3d.height_3d = height_3d
            results.append(det3d)
        return results

    # == Format conversion =====================================================

    def _convert_detections(self, detections_3d: list) -> list[CoreDetection3D]:
        """projection.Detection3D -> core.msgs.Detection3D."""
        results = []
        for d in detections_3d:
            pos = d.position
            feat = getattr(d, "features", None)
            has_feat = feat is not None and hasattr(feat, "size") and feat.size > 0
            track_id = getattr(d, "track_id", None)
            results.append(CoreDetection3D(
                id=f"track_{track_id}" if track_id is not None else "",
                label=d.label,
                confidence=d.score,
                position=Vector3(float(pos[0]), float(pos[1]), float(pos[2])),
                bbox_2d=[float(x) for x in d.bbox_2d],
                clip_feature=feat if has_feat else None,
            ))
        return results

    def _build_scene_graph(self) -> SceneGraph:
        """Build core SceneGraph message from InstanceTracker state."""
        if self._tracker is None:
            return self._build_detection_scene_graph()
        try:
            sg_json = self._tracker.get_scene_graph_json()
            data = json.loads(sg_json)
        except Exception:
            return self._build_detection_scene_graph()

        objects = []
        for obj in data.get("objects", []):
            pos = obj.get("position", [0, 0, 0])
            if isinstance(pos, dict):
                px, py, pz = float(pos.get("x", 0)), float(pos.get("y", 0)), float(pos.get("z", 0))
            elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
                px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
            else:
                px, py, pz = 0.0, 0.0, 0.0
            label = str(obj.get("label", ""))
            matched_det = self._match_detection_metadata(label, px, py, pz)
            object_id = str(obj.get("id", ""))
            bbox_2d = []
            clip_feature = None
            if matched_det is not None:
                if matched_det.id:
                    object_id = matched_det.id
                bbox_2d = [float(x) for x in matched_det.bbox_2d]
                if matched_det.clip_feature is not None:
                    clip_feature = np.array(matched_det.clip_feature, copy=True)
            objects.append(CoreDetection3D(
                id=object_id,
                label=label,
                confidence=float(obj.get("confidence", obj.get("score", 0))),
                position=Vector3(px, py, pz),
                bbox_2d=bbox_2d,
                clip_feature=clip_feature,
            ))

        relations = []
        for rel in data.get("relations", []):
            relations.append(Relation(
                subject_id=str(rel.get("subject_id", rel.get("subject", ""))),
                predicate=str(rel.get("predicate", rel.get("relation", ""))),
                object_id=str(rel.get("object_id", rel.get("object", ""))),
            ))

        regions = []
        for reg in data.get("rooms", data.get("regions", [])):
            regions.append(Region(
                name=str(reg.get("name", reg.get("room_type", ""))),
                object_ids=[str(oid) for oid in reg.get("object_ids", [])],
            ))

        return SceneGraph(
            objects=objects,
            relations=relations,
            regions=regions,
            frame_id=PERCEPTION_MAP_FRAME_ID,
        )

    def _build_detection_scene_graph(self) -> SceneGraph:
        """Fallback scene graph when tracker state is unavailable."""
        return SceneGraph(
            objects=[self._clone_core_detection(det) for det in self._latest_core_detections],
            frame_id=PERCEPTION_MAP_FRAME_ID,
        )

    def _match_detection_metadata(
        self, label: str, px: float, py: float, pz: float,
    ) -> CoreDetection3D | None:
        best_det: CoreDetection3D | None = None
        best_dist = 1.5
        label_lower = label.lower()

        for det in self._latest_core_detections:
            if label_lower and det.label.lower() != label_lower:
                continue
            dx = det.position.x - px
            dy = det.position.y - py
            dz = det.position.z - pz
            dist = float(np.sqrt(dx * dx + dy * dy + dz * dz))
            if dist < best_dist:
                best_dist = dist
                best_det = det

        return best_det

    @staticmethod
    def _clone_core_detection(det: CoreDetection3D) -> CoreDetection3D:
        clip_feature = None
        if det.clip_feature is not None:
            clip_feature = np.array(det.clip_feature, copy=True)
        return CoreDetection3D(
            id=det.id,
            label=det.label,
            confidence=det.confidence,
            position=Vector3(det.position.x, det.position.y, det.position.z),
            bbox_2d=[float(x) for x in det.bbox_2d],
            clip_feature=clip_feature,
            ts=det.ts,
        )

    # == Query API =============================================================

    # == Health ================================================================

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["frame_count"] = self._frame_count
        info["detector_type"] = self._detector_type
        info["encoder_type"] = self._encoder_type
        info["detector_ready"] = self._detector is not None
        info["detector_tracker_ready"] = self._detector_tracker is not None
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
        return info

    # == Query API =============================================================

    @property
    def tracker(self):
        """Access underlying InstanceTracker (for external queries / tests)."""
        return self._tracker

    @property
    def frame_count(self) -> int:
        return self._frame_count


# == Utility ===================================================================

def _quat_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Quaternion -> 3x3 rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
