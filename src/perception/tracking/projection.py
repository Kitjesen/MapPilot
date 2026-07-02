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

# ── USS-Nav 点云参数 ──
POINTCLOUD_MAX_POINTS = 512       # 每个物体最大点数 (降采样后)
POINTCLOUD_VOXEL_SIZE = 0.02      # 体素降采样分辨率 (m)
POINTCLOUD_MIN_POINTS = 10        # 少于此数的点云视为无效


@dataclass
class CameraIntrinsics:
    """相机内参。"""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class Detection3D:
    """3D 检测结果 (USS-Nav 升级: 含物体点云)。"""
    position: np.ndarray        # [x, y, z] 质心, world frame
    label: str
    score: float
    bbox_2d: np.ndarray         # [x1, y1, x2, y2] in pixels
    depth: float                # 质心深度 (m)
    features: np.ndarray        # 语义特征向量 (Mobile-CLIP text encoding)
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    # USS-Nav: 物体点云 (N, 3) world frame, 降采样后
    track_id: int | None = None  # Optional 2D tracker id (BoT-SORT/ByteTrack)


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
    K: np.ndarray = None,
    D: np.ndarray = None,
) -> np.ndarray:
    """单点 2D → 3D 投影 (相机坐标系), with optional undistortion.

    If K and D are provided and D is non-zero, pixel coordinates are
    undistorted before back-projection using cv2.undistortPoints.
    """
    if intrinsics.fx == 0.0 or intrinsics.fy == 0.0:
        return np.array([0.0, 0.0, depth_m])

    u, v = pixel_u, pixel_v
    if D is not None and K is not None and not np.allclose(D, 0):
        u_arr, v_arr = undistort_points(
            np.array([pixel_u]), np.array([pixel_v]), K, D,
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
    return p_world[:3]


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
    K: np.ndarray = None,
    D: np.ndarray = None,
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
            return points[indices]
        return points

    quantized = np.floor(points / voxel_size).astype(np.int32)

    _, unique_indices = np.unique(
        quantized, axis=0, return_index=True,
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
    return np.mean(points, axis=0)
