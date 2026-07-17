"""DepthVisualOdomModule — depth camera visual odometry for SLAM degeneracy fallback.

Provides backup odometry from RGB-D camera when LiDAR SLAM degenerates.
Uses ORB features + Lucas-Kanade optical flow + depth PnP for frame-to-frame
relative pose estimation.

Lazy activation: only runs feature detection/tracking when degeneracy >= SEVERE.
When LiDAR is healthy, this module sleeps (near-zero CPU).

Architecture:
    camera → color_image, depth_image, camera_info → this module
    SlamModule/SlamAdapterModule -> localization_status -> this module
    this module -> visual_odometry -> selected SLAM implementation

Based on Selective KF (arXiv 2412.17235) principle:
    Only fuse visual odometry for degenerate DOF directions.
"""

from __future__ import annotations

import importlib
import logging
import threading
import time
from typing import Any

from runtime.module import Module
from runtime.msgs.geometry import Pose, Quaternion, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import CameraIntrinsics, Image
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

_cv2: Any | None = None


def _cv2_module() -> Any:
    """Import OpenCV only when visual odometry actually uses it."""
    global _cv2
    if _cv2 is None:
        _cv2 = importlib.import_module("cv2")
    return _cv2


# Minimum features to compute PnP reliably
MIN_FEATURES_PNP = 8
# Maximum features to track (limit CPU on aarch64)
MAX_FEATURES = 300
# LK optical flow parameters that do not require OpenCV constants at import time
LK_PARAMS_BASE = dict(
    winSize=(21, 21),
    maxLevel=3,
)
# ORB detector parameters
ORB_PARAMS = dict(
    nfeatures=MAX_FEATURES,
    scaleFactor=1.2,
    nlevels=4,
    edgeThreshold=15,
    fastThreshold=12,
)


def _lk_params(cv2_module: Any) -> dict[str, Any]:
    return {
        **LK_PARAMS_BASE,
        "criteria": (
            cv2_module.TERM_CRITERIA_EPS | cv2_module.TERM_CRITERIA_COUNT,
            30,
            0.01,
        ),
    }


@register("visual_odom", "depth", description="Depth-camera visual odometry fallback")
class DepthVisualOdomModule(Module, layer=1):
    """Depth camera visual odometry — degeneracy-triggered backup.

    Lazy: only tracks features when SLAM reports SEVERE/CRITICAL degeneracy.
    Publishes incremental odometry in camera→world frame for selective fusion.
    """

    # Inputs
    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraIntrinsics]
    localization_status: In[dict]

    # Outputs
    visual_odometry: Out[Odometry]
    active: Out[bool]

    def __init__(
        self,
        min_depth: float = 0.3,
        max_depth: float = 10.0,
        reactivation_cooldown: float = 1.0,
        **kw,
    ):
        super().__init__(**kw)
        self._min_depth = min_depth
        self._max_depth = max_depth
        self._cooldown = reactivation_cooldown

        # State
        self._active = False
        self._intrinsics: CameraIntrinsics | None = None
        self._K: np.ndarray | None = None  # 3x3 camera matrix
        self._D: np.ndarray | None = None  # distortion coeffs
        self._undistort_maps: tuple | None = None  # (map1, map2) for cv2.remap

        # Frame tracking
        self._prev_gray: np.ndarray | None = None
        self._prev_depth: np.ndarray | None = None
        self._prev_kps: np.ndarray | None = None  # (N, 2) tracked points
        self._orb = None

        # Accumulated pose (world frame)
        self._T_world_cam = np.eye(4)  # current camera pose in world frame
        self._last_active_time: float = 0.0
        self._frame_count: int = 0

        # Thread safety
        self._lock = threading.Lock()

    def setup(self) -> None:
        self.color_image.subscribe(self._on_color)
        self.depth_image.subscribe(self._on_depth)
        self.camera_info.subscribe(self._on_camera_info)
        self.localization_status.subscribe(self._on_loc_status)
        self.color_image.set_policy("latest")
        self.depth_image.set_policy("latest")

    def _on_camera_info(self, info: CameraIntrinsics) -> None:
        """Cache camera intrinsics matrix."""
        if self._K is not None:
            return  # already initialized
        self._intrinsics = info
        self._K = np.array(
            [
                [info.fx, 0, info.cx],
                [0, info.fy, info.cy],
                [0, 0, 1],
            ],
            dtype=np.float64,
        )
        if info.has_distortion:
            self._D = np.array(info.D_vector, dtype=np.float64)
        else:
            self._D = np.zeros(5, dtype=np.float64)
        # Pre-compute undistortion maps for cv2.remap (fast per-frame undistort)
        if info.has_distortion and np.any(self._D != 0):
            cv2 = _cv2_module()
            self._undistort_maps = cv2.initUndistortRectifyMap(
                self._K,
                self._D,
                None,
                self._K,
                (info.width, info.height),
                cv2.CV_16SC2,
            )
            logger.info("DepthVisualOdom: undistortion maps computed (D=%s)", np.array2string(self._D, precision=4))
        logger.info("DepthVisualOdom: intrinsics received (%.0fx%.0f, fx=%.1f)", info.width, info.height, info.fx)

    def _on_loc_status(self, msg: dict) -> None:
        """Activate/deactivate based on SLAM degeneracy level."""
        degen = msg.get("degeneracy", "NONE")
        should_activate = degen in ("SEVERE", "CRITICAL")

        if should_activate and not self._active:
            self._activate()
        elif not should_activate and self._active:
            now = time.time()
            if now - self._last_active_time > self._cooldown:
                self._deactivate()

    def _activate(self) -> None:
        """Start visual odometry tracking."""
        self._active = True
        self._last_active_time = time.time()
        # Reset keyframe so next color frame becomes the reference
        self._prev_gray = None
        self._prev_kps = None
        self._frame_count = 0
        if self._orb is None:
            cv2 = _cv2_module()
            self._orb = cv2.ORB_create(**ORB_PARAMS)
        self.active.publish(True)
        logger.info("DepthVisualOdom: ACTIVATED (SLAM degenerate)")

    def _deactivate(self) -> None:
        """Stop visual odometry tracking."""
        self._active = False
        self._prev_gray = None
        self._prev_depth = None
        self._prev_kps = None
        self.active.publish(False)
        logger.info("DepthVisualOdom: deactivated (SLAM recovered)")

    _latest_depth: Any | None = None

    def _on_depth(self, img: Image) -> None:
        """Cache latest depth frame."""
        if not self._active:
            return
        self._latest_depth = img.data

    def _on_color(self, img: Image) -> None:
        """Process color frame — ORB detect or LK track → PnP solve."""
        if not self._active or self._K is None:
            return

        with self._lock:
            self._process_frame(img.data)

    def _process_frame(self, rgb: np.ndarray) -> None:
        """Core visual odometry pipeline."""
        cv2 = _cv2_module()
        # Convert to grayscale
        if rgb.ndim == 3 and rgb.shape[2] == 3:
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        elif rgb.ndim == 3 and rgb.shape[2] == 4:
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGBA2GRAY)
        else:
            gray = rgb

        depth = self._latest_depth
        if depth is None:
            return

        # Undistort images so ORB/LK/PnP operate under pinhole model
        if self._undistort_maps is not None:
            map1, map2 = self._undistort_maps
            gray = cv2.remap(gray, map1, map2, cv2.INTER_LINEAR)
            depth = cv2.remap(depth, map1, map2, cv2.INTER_NEAREST)

        # First frame: detect ORB keypoints as initial tracking set
        if self._prev_gray is None:
            kps = self._detect_features(gray)
            if kps is not None and len(kps) >= MIN_FEATURES_PNP:
                self._prev_gray = gray
                self._prev_depth = depth.copy()
                self._prev_kps = kps
            return

        # Track features with Lucas-Kanade optical flow
        tracked_prev, tracked_curr = self._track_features(self._prev_gray, gray, self._prev_kps)
        if tracked_prev is None or len(tracked_prev) < MIN_FEATURES_PNP:
            # Lost tracking — re-detect
            kps = self._detect_features(gray)
            if kps is not None and len(kps) >= MIN_FEATURES_PNP:
                self._prev_gray = gray
                self._prev_depth = depth.copy()
                self._prev_kps = kps
            else:
                self._prev_gray = None
                self._prev_kps = None
            return

        # Build 3D-2D correspondences from previous depth + current 2D
        pts_3d, pts_2d = self._build_correspondences(tracked_prev, tracked_curr, self._prev_depth)
        if pts_3d is None or len(pts_3d) < MIN_FEATURES_PNP:
            self._prev_gray = gray
            self._prev_depth = depth.copy()
            self._prev_kps = tracked_curr
            return

        # Solve PnP with RANSAC
        delta_T = self._solve_pnp(pts_3d, pts_2d)
        if delta_T is not None:
            # Accumulate pose
            self._T_world_cam = self._T_world_cam @ delta_T
            self._frame_count += 1
            self._last_active_time = time.time()

            # Publish odometry
            self._publish_odometry()

        # Update keyframe — re-detect features periodically to avoid drift
        self._prev_gray = gray
        self._prev_depth = depth.copy()
        if self._frame_count % 5 == 0 or len(tracked_curr) < MIN_FEATURES_PNP * 2:
            new_kps = self._detect_features(gray)
            self._prev_kps = new_kps if new_kps is not None else tracked_curr
        else:
            self._prev_kps = tracked_curr

    def _detect_features(self, gray: np.ndarray) -> np.ndarray | None:
        """Detect ORB keypoints, return as (N, 2) float32 array."""
        if self._orb is None:
            return None
        kps = self._orb.detect(gray, None)
        if not kps:
            return None
        pts = np.array([kp.pt for kp in kps], dtype=np.float32)
        return pts

    def _track_features(
        self,
        prev_gray: np.ndarray,
        curr_gray: np.ndarray,
        prev_pts: np.ndarray,
    ) -> tuple:
        """Track features with Lucas-Kanade, return (prev_good, curr_good)."""
        if prev_pts is None or len(prev_pts) == 0:
            return None, None

        pts = prev_pts.reshape(-1, 1, 2)
        cv2 = _cv2_module()
        curr_pts, status, _err = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray, pts, None, **_lk_params(cv2))
        if curr_pts is None or status is None:
            return None, None

        # Bidirectional check for robustness
        back_pts, back_status, _ = cv2.calcOpticalFlowPyrLK(curr_gray, prev_gray, curr_pts, None, **_lk_params(cv2))
        if back_pts is None:
            return None, None

        # Filter: forward-backward error < 1.0 pixel
        fb_err = np.linalg.norm(prev_pts.reshape(-1, 2) - back_pts.reshape(-1, 2), axis=1)
        good = (status.ravel() == 1) & (back_status.ravel() == 1) & (fb_err < 1.0)

        if good.sum() < MIN_FEATURES_PNP:
            return None, None

        return prev_pts[good].reshape(-1, 2), curr_pts[good].reshape(-1, 1, 2).reshape(-1, 2)

    def _build_correspondences(
        self,
        prev_pts: np.ndarray,
        curr_pts: np.ndarray,
        depth: np.ndarray,
    ) -> tuple:
        """Build 3D (from prev frame depth) → 2D (curr frame) correspondences."""
        K = self._K
        scale = 1.0
        if self._intrinsics:
            scale = self._intrinsics.depth_scale

        pts_3d = []
        pts_2d = []

        h, w = depth.shape[:2]
        for i in range(len(prev_pts)):
            u, v = int(round(prev_pts[i, 0])), int(round(prev_pts[i, 1]))
            if u < 0 or u >= w or v < 0 or v >= h:
                continue
            d = float(depth[v, u]) * scale
            if d < self._min_depth or d > self._max_depth:
                continue

            # Back-project to 3D using intrinsics
            x = (prev_pts[i, 0] - K[0, 2]) * d / K[0, 0]
            y = (prev_pts[i, 1] - K[1, 2]) * d / K[1, 1]
            z = d

            pts_3d.append([x, y, z])
            pts_2d.append(curr_pts[i])

        if len(pts_3d) < MIN_FEATURES_PNP:
            return None, None

        return (
            np.array(pts_3d, dtype=np.float64),
            np.array(pts_2d, dtype=np.float64),
        )

    def _solve_pnp(self, pts_3d: np.ndarray, pts_2d: np.ndarray) -> np.ndarray | None:
        """Solve PnP with RANSAC, return 4x4 transform or None."""
        # distCoeffs=None because images are already undistorted in _process_frame
        cv2 = _cv2_module()
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            pts_3d,
            pts_2d,
            self._K,
            None,
            iterationsCount=200,
            reprojectionError=3.0,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success or inliers is None:
            return None
        if len(inliers) < MIN_FEATURES_PNP:
            return None

        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.ravel()

        # Sanity check: reject unreasonable motion
        trans_norm = np.linalg.norm(tvec)
        if trans_norm > 2.0:  # > 2m per frame is unreasonable for quadruped
            logger.debug("DepthVisualOdom: PnP rejected (trans=%.2fm)", trans_norm)
            return None

        return T

    def _publish_odometry(self) -> None:
        """Publish accumulated visual odometry pose."""
        T = self._T_world_cam
        pos = T[:3, 3]
        R = T[:3, :3]

        # Convert rotation matrix to quaternion
        quat = self._rotation_to_quaternion(R)

        self.visual_odometry.publish(
            Odometry(
                pose=Pose(
                    position=Vector3(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
                    orientation=Quaternion(
                        x=float(quat[0]),
                        y=float(quat[1]),
                        z=float(quat[2]),
                        w=float(quat[3]),
                    ),
                ),
                ts=time.time(),
            )
        )

    @staticmethod
    def _rotation_to_quaternion(R: np.ndarray) -> np.ndarray:
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return np.array([x, y, z, w])

    def reset_pose(self, T: np.ndarray | None = None) -> None:
        """Reset accumulated pose (called when SLAM recovers and re-anchors)."""
        with self._lock:
            self._T_world_cam = T if T is not None else np.eye(4)
            self._prev_gray = None
            self._prev_kps = None
            self._frame_count = 0

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["visual_odom"] = {
            "active": self._active,
            "frame_count": self._frame_count,
            "tracking_points": len(self._prev_kps) if self._prev_kps is not None else 0,
            "has_intrinsics": self._K is not None,
        }
        return info
