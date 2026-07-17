"""ReconstructionModule — streaming RGB-D voxel map with dynamic-scene support.

Combines color projection, semantic labelling, and PLY export into a
single Module with typed ports.

Key improvements over the original stub implementation:
  - _on_odom builds a proper camera_to_world 4×4 from odometry pose + camera
    extrinsic parameters (loaded from robot_config.yaml).
  - A background loop at `process_hz` (default 5 Hz) drains the latest
    buffered color/depth pair and calls ColorProjector.update_from_frame().
  - Dynamic-object masking: bounding boxes from the latest scene_graph are
    passed to ColorProjector so pixels belonging to detected moving objects
    are excluded from the voxel write.
  - Voxel TTL (voxel_ttl config key, default 30 s): stale voxels are pruned
    each processing cycle, keeping the map consistent with the current scene.
  - Explicit camera extrinsic offset (loaded from robot_config.yaml camera
    section) composited onto the robot odometry pose.

Ports:
    In:  color_image (Image)          — BGR colour image from camera
         depth_image (Image)          — depth image (DEPTH_U16 mm or DEPTH_F32 m)
         camera_info (CameraIntrinsics) — pinhole intrinsics
         scene_graph (SceneGraph)     — detection bboxes for dynamic masking
         odometry (Odometry)          — robot body pose in world frame
    Out: semantic_cloud (dict)        — {points_shape, labels_count, stats}
         reconstruction_stats (dict)  — running statistics
"""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Any

from maps.taxonomy import load_semantic_taxonomy
from runtime import In, Module, Out
from runtime.config import get_config
from runtime.msgs.map import MapObservationFrame, SemanticLabelsFrame
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from runtime.registry import register

# ── Default dynamic-class labels that should not be written to the static map ──
# Phase 4: the canonical default now lives in runtime.config.PerceptionConfig;
# this module-level alias is kept so the fallback path is self-contained.
_DYNAMIC_LABELS = frozenset(
    {
        "person",
        "people",
        "pedestrian",
        "car",
        "vehicle",
        "truck",
        "bus",
        "bicycle",
        "motorcycle",
        "dog",
        "cat",
        "animal",
    }
)

# Camera-body extrinsic defaults (identity — corrected from robot_config at init)
_DEFAULT_CAM_BODY = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)


def _pose_to_matrix(odom: Odometry) -> np.ndarray:
    """Convert Odometry to 4×4 body-to-world homogeneous transform."""
    R = odom.pose.orientation.to_rotation_matrix()
    t = np.array([odom.pose.position.x, odom.pose.position.y, odom.pose.position.z], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _rpy_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build 3×3 rotation matrix from roll-pitch-yaw (rad), ZYX convention."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    return Rz @ Ry @ Rx


def _load_cam_body_extrinsic() -> np.ndarray:
    """Build body→camera 4×4 extrinsic from robot_config.yaml camera section.

    Falls back to identity if config is unavailable.
    """
    try:
        from runtime.config import get_config

        cfg = get_config()
        cam = cfg.camera
        R = _rpy_to_rotation(
            float(getattr(cam, "roll", 0.0)),
            float(getattr(cam, "pitch", 0.0)),
            float(getattr(cam, "yaw", 0.0)),
        )
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[0, 3] = float(getattr(cam, "position_x", 0.0))
        T[1, 3] = float(getattr(cam, "position_y", 0.0))
        T[2, 3] = float(getattr(cam, "position_z", 0.0))
        return T
    except Exception:
        return np.array(_DEFAULT_CAM_BODY, dtype=np.float64)


def _load_depth_scale() -> float:
    """Return depth_scale from robot_config.yaml (default 0.001 for mm depths)."""
    try:
        from runtime.config import get_config

        cfg = get_config()
        return float(getattr(cfg.camera, "depth_scale", 0.001))
    except Exception:
        return 0.001


class TSDFColorVolume:
    """W3-3: thin wrapper around Open3D ScalableTSDFVolume with colour fusion.

    Replaces the old raw voxel accumulation (no weighting, frame-over-frame
    overwrites, ghosting from pose noise). TSDF gives proper weighted fusion
    with median-like behaviour near surfaces — noisy poses no longer smear
    geometry.

    Raises RuntimeError on construction if open3d is not installed (Wave 1
    no-silent-fallback discipline). Real TSDF runs on S100P where open3d
    is available; unit tests mock the open3d module.
    """

    def __init__(
        self,
        voxel_length: float = 0.04,
        sdf_trunc: float = 0.15,
    ) -> None:
        try:
            import open3d as o3d
        except ImportError as e:
            raise RuntimeError("TSDFColorVolume requires open3d. Install with: pip install open3d") from e

        self.voxel_length = float(voxel_length)
        self.sdf_trunc = float(sdf_trunc)
        color_type = o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        self._volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.voxel_length,
            sdf_trunc=self.sdf_trunc,
            color_type=color_type,
        )

    def integrate(
        self,
        depth: np.ndarray,
        color: np.ndarray,
        K: np.ndarray,
        extrinsic: np.ndarray,
    ) -> None:
        """Integrate one RGB-D frame into the TSDF volume."""
        import open3d as o3d

        h, w = depth.shape[:2]
        color_img = o3d.geometry.Image(color)
        depth_img = o3d.geometry.Image(depth)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_img,
            depth_img,
            depth_scale=1000.0,
            depth_trunc=5.0,
            convert_rgb_to_intensity=False,
        )
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=w,
            height=h,
            fx=float(K[0, 0]),
            fy=float(K[1, 1]),
            cx=float(K[0, 2]),
            cy=float(K[1, 2]),
        )
        self._volume.integrate(rgbd, intrinsic, extrinsic)

    def extract_point_cloud(self) -> tuple[np.ndarray, np.ndarray]:
        """Return (pts: Nx3 float32, colors: Nx3 float32 in [0,1])."""
        pcd = self._volume.extract_point_cloud()
        pts = np.asarray(pcd.points).astype(np.float32)
        colors = np.asarray(pcd.colors).astype(np.float32)
        return pts, colors

    def extract_mesh(self):
        """Extract a triangle mesh from the fused volume."""
        return self._volume.extract_triangle_mesh()


class _LazyProjectorConfig:
    """Configuration placeholder until the numeric projector is first used."""

    def __init__(self, voxel_size: float, voxel_ttl: float) -> None:
        self._voxel_size = float(voxel_size)
        self._voxel_ttl = float(voxel_ttl)
        self.voxel_count = 0


@register("reconstruction", "default", description="RGB-D reconstruction module")
class ReconstructionModule(Module, layer=3):
    """Streaming RGB-D voxel reconstruction with dynamic-scene support.

    Configuration keys (all optional):
        voxel_size          float  0.05  — voxel edge length in metres
        voxel_ttl           float  30.0  — voxel lifetime in seconds (0 = infinite)
        process_hz          float  5.0   — processing rate (Hz)
        min_points_to_publish int 1000   — minimum voxels before publishing cloud
        save_dir            str   "maps/reconstruction"
        mask_dynamic        bool  True   — skip pixels from dynamic-class detections
        dynamic_labels      list  [...] — override default dynamic label set
    """

    _run_in_worker = True
    _worker_group = "perception"

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraIntrinsics]
    scene_graph: In[SceneGraph]
    map_observation: In[MapObservationFrame]
    odometry: In[Odometry]
    semantic_cloud: Out[dict]
    semantic_labels: Out[SemanticLabelsFrame]
    reconstruction_stats: Out[dict]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)

        self._voxel_size = float(config.get("voxel_size", 0.05))
        self._voxel_ttl = float(config.get("voxel_ttl", 30.0))
        self._projector = _LazyProjectorConfig(self._voxel_size, self._voxel_ttl)
        self._labeler = None

        self._process_hz = float(config.get("process_hz", 5.0))
        self._min_points = int(config.get("min_points_to_publish", 1000))
        self._save_dir = str(config.get("save_dir", "maps/reconstruction"))
        self._mask_dynamic = bool(config.get("mask_dynamic", True))
        self._semantic_scene_max_age_s = max(0.0, float(config.get("semantic_scene_max_age_s", 0.25)))
        self._taxonomy = load_semantic_taxonomy(config.get("semantic_taxonomy_path"))
        self._semantic_scan_labels_published = 0
        self._semantic_scan_labels_skipped = 0

        dyn_labels = config.get("dynamic_labels")
        if dyn_labels:
            self._dynamic_labels: frozenset[str] = frozenset(dyn_labels)
        else:
            # Fall back to the robot_config.yaml perception.dynamic_labels set,
            # keeping the module-level _DYNAMIC_LABELS as the ultimate default.
            self._dynamic_labels = get_config().perception.dynamic_labels or _DYNAMIC_LABELS

        # Extrinsic: camera pose in body frame (loaded from robot_config)
        self._camera_pose_in_body = None
        self._depth_scale: float = _load_depth_scale()

        # Buffered inputs (latest-wins)
        self._latest_color: Image | None = None
        self._latest_depth: Image | None = None
        self._latest_odom: Odometry | None = None
        self._latest_sg: SceneGraph | None = None
        self._intrinsics: CameraIntrinsics | None = None
        self._buf_lock = threading.Lock()

        self._last_update_time: float | None = None
        self._total_frames: int = 0
        self._bg_thread: threading.Thread | None = None
        self._recon_active = threading.Event()

    def _ensure_components(self) -> None:
        if isinstance(self._projector, _LazyProjectorConfig):
            from .color_projector import ColorProjector

            self._projector = ColorProjector(
                voxel_size=self._voxel_size,
                voxel_ttl=self._voxel_ttl,
            )
        if self._labeler is None:
            from .semantic_labeler import SemanticLabeler

            self._labeler = SemanticLabeler()
        if self._camera_pose_in_body is None:
            self._camera_pose_in_body = _load_cam_body_extrinsic()

    def _ensure_labeler(self) -> None:
        if self._labeler is None:
            from .semantic_labeler import SemanticLabeler

            self._labeler = SemanticLabeler()

    # ── Module lifecycle ────────────────────────────────────────────────────

    def setup(self) -> None:
        self._ensure_components()
        # Reconstruction is heavy (CLIP + 3D TSDF). Drop stale frames so the
        # camera publisher never blocks waiting for us — that would starve
        # the uvicorn event loop of the GIL.
        self.color_image.subscribe(self._on_color)
        self.color_image.set_policy("latest")
        self.depth_image.subscribe(self._on_depth)
        self.depth_image.set_policy("latest")
        self.camera_info.subscribe(self._on_camera_info)
        self.scene_graph.subscribe(self._on_scene_graph)
        self.map_observation.subscribe(self._on_map_observation)
        self.map_observation.set_policy("latest")
        self.odometry.subscribe(self._on_odom)

        self._recon_active.set()
        self._bg_thread = threading.Thread(target=self._process_loop, daemon=True, name="reconstruction_bg")
        self._bg_thread.start()

    def teardown(self) -> None:
        self._recon_active.clear()
        if self._bg_thread is not None:
            self._bg_thread.join(timeout=3.0)

    # ── Port callbacks ──────────────────────────────────────────────────────

    def _on_color(self, img: Image) -> None:
        with self._buf_lock:
            self._latest_color = img

    def _on_depth(self, img: Image) -> None:
        with self._buf_lock:
            self._latest_depth = img

    def _on_camera_info(self, info: CameraIntrinsics) -> None:
        # CameraInfo is static; only accept the first message.
        # If depth_scale is provided by the sensor message, prefer it.
        if self._intrinsics is None:
            self._intrinsics = info
            if hasattr(info, "depth_scale") and info.depth_scale > 0:
                self._depth_scale = float(info.depth_scale)

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        self._ensure_labeler()
        with self._buf_lock:
            self._latest_sg = sg
        self._labeler.update_from_scene_graph(sg.to_json())

    def _on_odom(self, odom: Odometry) -> None:
        with self._buf_lock:
            self._latest_odom = odom

    def _on_map_observation(self, observation: MapObservationFrame) -> None:
        """Label one accepted LiDAR scan without changing its point order."""
        self._ensure_labeler()
        with self._buf_lock:
            scene_graph = self._latest_sg
        if (
            scene_graph is None
            or scene_graph.frame_id != observation.frame_id
            or abs(float(scene_graph.ts) - observation.ts) > self._semantic_scene_max_age_s
        ):
            self._semantic_scan_labels_skipped += 1
            return
        labels_text = self._labeler.label_cloud(observation.map_points())
        label_ids = self._taxonomy.encode(labels_text, excluded=self._dynamic_labels)
        if not bool(np.any(label_ids != 0)):
            self._semantic_scan_labels_skipped += 1
            return
        confidence = (label_ids != 0).astype(np.float32)
        self.semantic_labels.publish(
            SemanticLabelsFrame(
                labels=label_ids,
                confidence=confidence,
                sequence=observation.sequence,
                ts=observation.ts,
                frame_id=observation.sensor_frame_id,
                taxonomy=self._taxonomy.name,
                taxonomy_version=self._taxonomy.version,
                source="reconstruction.scene_graph_projection",
                metadata={"scene_graph_ts": float(scene_graph.ts)},
            )
        )
        self._semantic_scan_labels_published += 1

    # ── Background processing loop ──────────────────────────────────────────

    def _process_loop(self) -> None:
        interval = 1.0 / max(self._process_hz, 0.1)
        while self._recon_active.is_set():
            t0 = time.time()
            try:
                self._process_one_frame()
            except Exception:
                pass
            elapsed = time.time() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _process_one_frame(self) -> None:
        self._ensure_components()
        with self._buf_lock:
            color = self._latest_color
            depth = self._latest_depth
            odom = self._latest_odom
            sg = self._latest_sg

        if color is None or depth is None or odom is None:
            return

        intrinsics = self._intrinsics
        if intrinsics is None:
            # Fall back to robot_config camera parameters
            try:
                from runtime.config import get_config

                cam = get_config().camera
                from runtime.msgs.sensor import CameraIntrinsics

                intrinsics = CameraIntrinsics(
                    fx=float(cam.fx),
                    fy=float(cam.fy),
                    cx=float(cam.cx),
                    cy=float(cam.cy),
                    width=int(cam.width),
                    height=int(cam.height),
                    depth_scale=float(cam.depth_scale),
                )
            except Exception:
                return

        # Build camera-to-world: T_world_cam = T_world_body @ T_body_cam
        body_to_world = _pose_to_matrix(odom)
        camera_to_world = body_to_world @ self._camera_pose_in_body

        # Prepare depth array (normalise to mm uint16 equivalent)
        depth_arr = depth.data
        if depth.format == ImageFormat.DEPTH_F32:
            # float32 metres → scale via depth_scale inverse
            depth_mm = (depth_arr / self._depth_scale).astype(np.uint16)
            depth_scale_used = self._depth_scale
        elif depth.format == ImageFormat.DEPTH_U16:
            depth_mm = depth_arr
            depth_scale_used = self._depth_scale
        else:
            # Assume uint16 mm
            depth_mm = depth_arr.astype(np.uint16)
            depth_scale_used = self._depth_scale

        # Prepare BGR image
        color_bgr = color.data
        if color.format.value in ("RGB", "RGBA"):
            color_bgr = color_bgr[..., ::-1].copy()

        # Dynamic masking: build pixel bboxes from current scene_graph
        exclude_boxes: np.ndarray | None = None
        if self._mask_dynamic and sg is not None and sg.objects:
            boxes = []
            for obj in sg.objects:
                if obj.label.lower() in self._dynamic_labels and obj.bbox_2d:
                    bb = obj.bbox_2d
                    if len(bb) >= 4:
                        boxes.append([bb[0], bb[1], bb[2], bb[3]])
            if boxes:
                exclude_boxes = np.array(boxes, dtype=np.float32)

        self._projector.update_from_frame(
            color_bgr=color_bgr,
            depth_mm=depth_mm,
            fx=intrinsics.fx,
            fy=intrinsics.fy,
            cx=intrinsics.cx,
            cy=intrinsics.cy,
            camera_to_world=camera_to_world,
            exclude_boxes=exclude_boxes,
            depth_scale=depth_scale_used,
        )

        self._last_update_time = time.time()
        self._total_frames += 1

        # Periodically prune TTL-expired voxels and publish stats
        if self._total_frames % max(1, int(self._process_hz)) == 0:
            self._projector.prune_stale()
            self.publish_cloud()

    # ── Direct API (no ROS2) ────────────────────────────────────────────────

    def update_frame(
        self,
        color_bgr: np.ndarray,
        depth_mm: np.ndarray,
        intrinsics: CameraIntrinsics,
        camera_to_world: np.ndarray,
        exclude_boxes: np.ndarray | None = None,
    ) -> int:
        """Process one RGB-D frame directly (bypassing ports).

        Parameters
        ----------
        color_bgr : (H, W, 3) uint8
        depth_mm : (H, W) uint16, millimetres
        intrinsics : CameraIntrinsics
        camera_to_world : (4, 4) float64
        exclude_boxes : optional (M, 4) pixel bboxes of dynamic objects

        Returns
        -------
        int
            Number of voxels updated.
        """
        self._ensure_components()
        return self._projector.update_from_frame(
            color_bgr=color_bgr,
            depth_mm=depth_mm,
            fx=intrinsics.fx,
            fy=intrinsics.fy,
            cx=intrinsics.cx,
            cy=intrinsics.cy,
            camera_to_world=camera_to_world,
            exclude_boxes=exclude_boxes,
            depth_scale=self._depth_scale,
        )

    def publish_cloud(self) -> dict[str, Any] | None:
        """Build and publish the semantic cloud if enough points exist.

        Returns the stats dict, or None if below threshold.
        """
        self._ensure_components()
        xyzrgb = self._projector.get_colored_cloud()
        if len(xyzrgb) < self._min_points:
            return None

        labels = self._labeler.label_cloud(xyzrgb)
        label_ids = self._taxonomy.encode(labels, excluded=self._dynamic_labels)
        n_labeled = sum(1 for lb in labels if lb != "background")

        stats: dict[str, Any] = {
            "total_points": len(xyzrgb),
            "labeled_points": n_labeled,
            "objects": self._labeler.object_count,
            "voxels": self._projector.voxel_count,
            "frames_processed": self._total_frames,
        }

        cloud_payload: dict[str, Any] = {
            "points": xyzrgb,
            "label_ids": label_ids,
            "labels": labels,
            "taxonomy": self._taxonomy.name,
            "taxonomy_version": self._taxonomy.version,
            "palette": self._taxonomy.palette,
            "points_shape": list(xyzrgb.shape),
            "labels_count": len(labels),
            "stats": stats,
        }

        self.semantic_cloud.publish(cloud_payload)
        self.reconstruction_stats.publish(stats)
        return stats

    def save_ply(self, filepath: str | None = None) -> dict[str, Any]:
        """Save current reconstruction to PLY file.

        Returns
        -------
        dict with 'success', 'message', and optional 'filepath'.
        """
        from .ply_writer import save_ply_with_labels

        self._ensure_components()
        xyzrgb = self._projector.get_colored_cloud()
        if len(xyzrgb) == 0:
            return {"success": False, "message": "no point cloud data"}

        labels = self._labeler.label_cloud(xyzrgb)
        if filepath is None:
            timestamp = int(time.time())
            filepath = os.path.join(self._save_dir, f"reconstruction_{timestamp}.ply")

        try:
            n = save_ply_with_labels(xyzrgb, labels, filepath)
            return {"success": True, "message": f"saved {n} points to {filepath}", "filepath": filepath}
        except Exception as exc:
            return {"success": False, "message": f"save failed: {exc}"}

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["mesh_vertices"] = 0 if self._projector is None else self._projector.voxel_count
        info["frames_processed"] = self._total_frames
        info["last_update_time"] = self._last_update_time
        info["voxel_ttl_s"] = self._voxel_ttl
        info["mask_dynamic"] = self._mask_dynamic
        return info

    @property
    def voxel_count(self) -> int:
        return 0 if self._projector is None else self._projector.voxel_count

    @property
    def object_count(self) -> int:
        return 0 if self._labeler is None else self._labeler.object_count
