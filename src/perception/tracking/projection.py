"""
2D → 3D 投影: USS-Nav 风格 mask→点云 投影管线。

USS-Nav pipeline (Fig.4):
  Image + Depth + Mask → Cloud projection → Object 点云 Ci
  每个检测物体得到完整 3D 点云, 而非单点投影。

升级:
  - 原实现: bbox 中心深度 → 单个 3D 点 (精度低)
  - 新实现: instance mask × depth → 完整物体点云 (USS-Nav Eq.1/2 的基础)

参考:
  - USS-Nav §IV-C: masked pixels back-projected via intrinsics π⁻¹
  - ConceptGraphs (ICRA 2024): 类似的 mask→点云管线
"""

from dataclasses import dataclass, field

import numpy as np

from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat

# ── USS-Nav 点云参数 ──
POINTCLOUD_MAX_POINTS = 512  # 每个物体最大点数 (降采样后)
POINTCLOUD_VOXEL_SIZE = 0.02  # 体素降采样分辨率 (m)
POINTCLOUD_MIN_POINTS = 10  # 少于此数的点云视为无效

# ── BBox depth fallback 参数 ──
BBOX_MEDIAN_MIN_VALID_PIXELS = 20  # minimum valid depth pixels inside bbox


@dataclass
class Detection3D:
    """3D 检测结果 (USS-Nav 升级: 含物体点云)。"""

    position: np.ndarray  # [x, y, z] 质心, world frame
    label: str
    score: float
    bbox_2d: np.ndarray  # [x1, y1, x2, y2] in pixels
    depth: float  # 质心深度 (m)
    features: np.ndarray  # 语义特征向量 (Mobile-CLIP text encoding)
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    # USS-Nav: 物体点云 (N, 3) world frame, 降采样后
    track_id: int | None = None  # Optional 2D tracker id (BoT-SORT/ByteTrack)
    confidence_3d: float = 0.0
    width_3d: float = 0.0
    height_3d: float = 0.0


def depth_scale_for_image(
    depth_image: Image,
    intrinsics: CameraIntrinsics,
    *,
    u16_fallback: float,
) -> float:
    """Return the raw-depth-to-metres multiplier for one typed image.

    Float depth is already expressed in metres.  Integer depth normally
    carries its scale in CameraInfo; the configured camera scale is used when
    CameraInfo contains the runtime default rather than a plausible uint16
    scale.
    """
    if depth_image.format is ImageFormat.DEPTH_F32:
        return 1.0
    if depth_image.format is not ImageFormat.DEPTH_U16:
        raise ValueError(f"unsupported_depth_format:{depth_image.format.value}")

    candidate = float(getattr(intrinsics, "depth_scale", 0.0) or 0.0)
    if np.isfinite(candidate) and 0.0 < candidate <= 0.1:
        return candidate
    fallback = float(u16_fallback)
    if not np.isfinite(fallback) or fallback <= 0.0:
        raise ValueError("invalid_u16_depth_scale")
    return fallback


def bbox_center_depth(
    depth_image: np.ndarray,
    bbox: np.ndarray,
    depth_scale: float = 0.001,
    kernel_size: int = 5,
) -> float | None:
    """计算 bbox 中心区域的中值深度 (保留向后兼容)。"""
    x1, y1, x2, y2 = bbox.astype(int)
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    h, w = depth_image.shape[:2]

    r = kernel_size
    y_lo = max(0, cy - r)
    y_hi = min(h, cy + r + 1)
    x_lo = max(0, cx - r)
    x_hi = min(w, cx + r + 1)

    patch = depth_image[y_lo:y_hi, x_lo:x_hi].astype(np.float64) * depth_scale
    valid = patch[patch > 0]

    if len(valid) == 0:
        return None

    return float(np.median(valid))


def undistort_points(
    us: np.ndarray,
    vs: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
) -> tuple:
    """Undistort pixel coordinates using OpenCV.

    Returns (undistorted_us, undistorted_vs) as float64 arrays.
    Falls back to identity if cv2 is unavailable or D is all zeros.
    """
    if D is None or np.allclose(D, 0):
        return us.astype(np.float64), vs.astype(np.float64)
    try:
        import cv2

        pts = np.stack([us.astype(np.float64), vs.astype(np.float64)], axis=-1)
        pts = pts.reshape(-1, 1, 2)
        undist = cv2.undistortPoints(pts, K, D, P=K)
        undist = undist.reshape(-1, 2)
        return undist[:, 0], undist[:, 1]
    except ImportError:
        return us.astype(np.float64), vs.astype(np.float64)


def project_to_3d(
    pixel_u: float,
    pixel_v: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
    K: np.ndarray | None = None,
    D: np.ndarray | None = None,
) -> np.ndarray:
    """单点 2D → 3D 投影 (相机坐标系), with optional undistortion.

    If K and D are provided and D is non-zero, pixel coordinates are
    undistorted before back-projection using cv2.undistortPoints.
    """
    if (
        not np.isfinite(intrinsics.fx)
        or not np.isfinite(intrinsics.fy)
        or intrinsics.fx <= 0.0
        or intrinsics.fy <= 0.0
    ):
        raise ValueError("camera focal lengths must be finite and positive")

    u, v = pixel_u, pixel_v
    if D is not None and K is not None and not np.allclose(D, 0):
        u_arr, v_arr = undistort_points(
            np.array([pixel_u]),
            np.array([pixel_v]),
            K,
            D,
        )
        u, v = float(u_arr[0]), float(v_arr[0])

    x = (u - intrinsics.cx) * depth_m / intrinsics.fx
    y = (v - intrinsics.cy) * depth_m / intrinsics.fy
    z = depth_m
    return np.array([x, y, z])


def transform_point(
    point_camera: np.ndarray,
    tf_camera_to_world: np.ndarray,
) -> np.ndarray:
    """将相机坐标系的点变换到世界坐标系。"""
    p_homo = np.array([*point_camera, 1.0])
    p_world = tf_camera_to_world @ p_homo
    return np.asarray(p_world[:3], dtype=np.float64)


def mask_to_pointcloud(
    mask: np.ndarray,
    depth_image: np.ndarray,
    intrinsics: CameraIntrinsics,
    tf_camera_to_world: np.ndarray,
    depth_scale: float = 0.001,
    min_depth: float = 0.3,
    max_depth: float = 6.0,
    max_points: int = POINTCLOUD_MAX_POINTS,
    voxel_size: float = POINTCLOUD_VOXEL_SIZE,
    K: np.ndarray | None = None,
    D: np.ndarray | None = None,
) -> np.ndarray | None:
    """
    USS-Nav §IV-C: 将 instance mask + depth 反投影为世界坐标系 3D 点云。

    Pipeline:
      1. mask 内像素提取有效深度
      2. 批量反投影 (vectorized) → 相机系点云
      3. 变换到世界系
      4. 体素降采样控制点数

    Args:
        mask: HxW bool, instance segmentation mask
        depth_image: HxW uint16 depth
        intrinsics: 相机内参
        tf_camera_to_world: 4x4 变换矩阵
        depth_scale: depth 值→米
        min_depth, max_depth: 深度有效范围
        max_points: 最大输出点数
        voxel_size: 体素降采样尺寸 (m)

    Returns:
        (N, 3) 世界坐标系点云, 或 None (无效)
    """
    if mask is None or depth_image is None:
        return None

    if mask.shape != depth_image.shape[:2]:
        # Mask/depth size mismatch — cannot safely resize without cv2
        return None

    vs, us = np.where(mask)
    if len(vs) < POINTCLOUD_MIN_POINTS:
        return None

    # Clip to depth image bounds
    h, w = depth_image.shape[:2]
    valid_bounds = (vs < h) & (us < w)
    vs, us = vs[valid_bounds], us[valid_bounds]
    if len(vs) < POINTCLOUD_MIN_POINTS:
        return None

    depths = depth_image[vs, us].astype(np.float64) * depth_scale

    # Filter NaN and inf
    finite_mask = np.isfinite(depths)
    valid = finite_mask & (depths > min_depth) & (depths < max_depth)
    vs, us, depths = vs[valid], us[valid], depths[valid]

    if len(depths) < POINTCLOUD_MIN_POINTS:
        return None

    # 若像素过多, 先随机降采样加速后续计算
    if len(depths) > max_points * 4:
        indices = np.random.choice(len(depths), max_points * 4, replace=False)
        vs, us, depths = vs[indices], us[indices], depths[indices]

    # Guard against zero focal lengths
    fx = intrinsics.fx if intrinsics.fx != 0.0 else 1.0
    fy = intrinsics.fy if intrinsics.fy != 0.0 else 1.0

    # Undistort pixel coordinates if distortion is available
    us_f = us.astype(np.float64)
    vs_f = vs.astype(np.float64)
    if K is not None and D is not None:
        us_f, vs_f = undistort_points(us_f, vs_f, K, D)

    # 批量反投影: pixel (u,v,d) → camera 3D (vectorized)
    x_cam = (us_f - intrinsics.cx) * depths / fx
    y_cam = (vs_f - intrinsics.cy) * depths / fy
    z_cam = depths

    points_cam = np.stack([x_cam, y_cam, z_cam], axis=1)  # (N, 3)

    # 变换到世界坐标系 (批量矩阵乘)
    R = tf_camera_to_world[:3, :3]
    t = tf_camera_to_world[:3, 3]
    points_world = (R @ points_cam.T).T + t  # (N, 3)

    # 体素降采样
    points_world = _voxel_downsample(points_world, voxel_size, max_points)

    if len(points_world) < POINTCLOUD_MIN_POINTS:
        return None

    return points_world


def _voxel_downsample(
    points: np.ndarray,
    voxel_size: float,
    max_points: int,
) -> np.ndarray:
    """
    轻量体素降采样 (纯 numpy, 无 Open3D 依赖)。

    每个体素保留一个点 (体素内均值), 控制总点数。
    """
    if len(points) == 0:
        return points

    # Filter out NaN/inf points before processing
    finite_mask = np.isfinite(points).all(axis=1)
    if not finite_mask.all():
        points = points[finite_mask]
        if len(points) == 0:
            return points

    if voxel_size <= 0 or len(points) <= max_points:
        if len(points) > max_points:
            indices = np.random.choice(len(points), max_points, replace=False)
            return np.asarray(points[indices])
        return points

    quantized = np.floor(points / voxel_size).astype(np.int32)

    _, unique_indices = np.unique(
        quantized,
        axis=0,
        return_index=True,
    )

    downsampled = points[unique_indices]

    if len(downsampled) > max_points:
        indices = np.random.choice(len(downsampled), max_points, replace=False)
        downsampled = downsampled[indices]

    return downsampled


def pointcloud_centroid(points: np.ndarray) -> np.ndarray:
    """计算点云质心。"""
    if points is None or len(points) == 0:
        return np.zeros(3)
    return np.asarray(np.mean(points, axis=0))


def bbox_median_depth_to_detection3d(
    det2d,
    depth_image: np.ndarray,
    tf_camera_to_world: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_scale: float = 0.001,
    min_depth: float = 0.3,
    max_depth: float = 6.0,
    min_valid_pixels: int = BBOX_MEDIAN_MIN_VALID_PIXELS,
    K: np.ndarray | None = None,
    D: np.ndarray | None = None,
) -> Detection3D | None:
    """W2-1: masked-depth median 3D projection fallback.

    For each 2D detection:
      1. Gather all depth pixels inside the bbox.
      2. Filter invalid (0, non-finite, out of [min_depth, max_depth]).
      3. If fewer than *min_valid_pixels* valid, drop the detection
         (we refuse to fabricate a 3D position from sparse/unreliable depth).
      4. Take the median of the valid samples as representative depth
         (robust to outliers and reflections).
      5. Back-project the bbox centre pixel using median depth to get a
         camera-frame 3D point; transform to world.
      6. Attach confidence_3d = valid_pixels / total_pixels.

    Returns a Detection3D with extra attributes confidence_3d, width_3d,
    height_3d, or None when the detection is rejected.
    """
    if intrinsics is None:
        return None

    bbox = det2d.bbox
    x1 = max(0, int(bbox[0]))
    y1 = max(0, int(bbox[1]))
    x2 = min(depth_image.shape[1], int(bbox[2]))
    y2 = min(depth_image.shape[0], int(bbox[3]))
    if x2 <= x1 or y2 <= y1:
        return None

    roi = depth_image[y1:y2, x1:x2]
    total_pixels = roi.size
    if total_pixels == 0:
        return None

    roi_m = roi.astype(np.float32) * depth_scale
    finite_mask = np.isfinite(roi_m) & (roi_m > 0.0)
    range_mask = (roi_m >= min_depth) & (roi_m <= max_depth)
    valid_mask = finite_mask & range_mask
    valid_depths = roi_m[valid_mask]

    if valid_depths.size < min_valid_pixels:
        return None

    d_median = float(np.median(valid_depths))
    confidence_3d = float(valid_depths.size) / float(total_pixels)

    px = (bbox[0] + bbox[2]) / 2.0
    py = (bbox[1] + bbox[3]) / 2.0
    p_cam = project_to_3d(px, py, d_median, intrinsics, K=K, D=D)
    p_world = (tf_camera_to_world @ np.array([*p_cam, 1.0]))[:3]

    fx = intrinsics.fx if intrinsics.fx != 0.0 else 600.0
    fy = intrinsics.fy if intrinsics.fy != 0.0 else 600.0
    bbox_w_px = max(1.0, float(bbox[2] - bbox[0]))
    bbox_h_px = max(1.0, float(bbox[3] - bbox[1]))
    width_3d = bbox_w_px * d_median / fx
    height_3d = bbox_h_px * d_median / fy

    det3d = Detection3D(
        position=p_world,
        label=det2d.label,
        score=det2d.score,
        bbox_2d=bbox,
        depth=d_median,
        features=getattr(det2d, "features", np.array([])),
        points=np.empty((0, 3)),
        track_id=getattr(det2d, "track_id", None),
        confidence_3d=confidence_3d,
        width_3d=width_3d,
        height_3d=height_3d,
    )
    return det3d
