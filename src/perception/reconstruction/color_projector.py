"""
color_projector.py — RGB-D 帧 → 世界坐标系彩色点云

原理:
  每帧 RGB-D: 深度图反投影 → 相机坐标系点云 → 变换到世界坐标系 → 存入体素颜色表
  发布时: 查询体素颜色表 → 输出彩色点云

体素颜色表: dict[(ix, iy, iz)] -> (r, g, b, timestamp)
  分辨率 VOXEL_SIZE (默认 5cm)，越新的观测覆盖越旧的。
  支持 TTL 衰减: voxel_ttl > 0 时，超过 TTL 秒未被观测的体素在查询时被剔除。
  支持动态物体遮罩: exclude_boxes 跳过被检测为动态物体的像素投影。
"""

import threading
import time

import numpy as np

VOXEL_SIZE = 0.05          # 体素边长 (m)
SUBSAMPLE_STEP = 4         # 像素降采样步长 (每 4 像素取一个，降低 CPU)
MIN_DEPTH = 0.3            # 最小有效深度 (m)
MAX_DEPTH = 6.0            # 最大有效深度 (m)
DEFAULT_TTL = 0.0          # 体素 TTL (s)，0 = 永不过期


class ColorProjector:
    """维护一个全局体素颜色+时间戳表，将 RGB-D 帧投影到世界坐标系。

    Parameters
    ----------
    voxel_size : float
        体素边长 (m)，默认 5 cm。
    voxel_ttl : float
        体素存活时间 (s)。0 表示永不过期；正值则超时后在 get_colored_cloud()
        调用时被懒惰剔除，适用于动态场景。
    """

    def __init__(self, voxel_size: float = VOXEL_SIZE, voxel_ttl: float = DEFAULT_TTL):
        self._voxel_size = voxel_size
        self._voxel_ttl = float(voxel_ttl)
        # (ix, iy, iz) -> [r, g, b, timestamp]
        self._voxels: dict[tuple, list] = {}
        self._lock = threading.Lock()

    # ── 公开接口 ────────────────────────────────────────────────

    def update_from_frame(
        self,
        color_bgr: np.ndarray,       # (H, W, 3) BGR uint8
        depth_mm: np.ndarray,        # (H, W) uint16，单位 mm
        fx: float, fy: float, cx: float, cy: float,
        camera_to_world: np.ndarray, # 4×4 float64 变换矩阵
        exclude_boxes: np.ndarray | None = None,  # (M, 4) [x1,y1,x2,y2] 动态遮罩
        depth_scale: float = 0.001,  # 深度值单位到米的缩放，默认 mm→m
    ) -> int:
        """用一帧 RGB-D 数据更新体素颜色表，返回新增/更新的体素数。

        Parameters
        ----------
        exclude_boxes : (M, 4) float array [x1, y1, x2, y2] in pixel coords
            被检测为动态物体的边界框。落在任意框内的像素将跳过投影，
            以避免把运动物体写入静态地图。
        depth_scale : float
            原始深度值 → 米的缩放因子（uint16 毫米深度图用 0.001；
            float32 米深度图用 1.0）。
        """
        h, w = depth_mm.shape
        ts_now = time.time()

        # 降采样像素索引
        rows = np.arange(0, h, SUBSAMPLE_STEP)
        cols = np.arange(0, w, SUBSAMPLE_STEP)
        cc, rr = np.meshgrid(cols, rows)
        cc = cc.ravel()
        rr = rr.ravel()

        # 深度过滤
        depths = depth_mm[rr, cc].astype(np.float32) * float(depth_scale)
        valid = (depths > MIN_DEPTH) & (depths < MAX_DEPTH)
        cc, rr, depths = cc[valid], rr[valid], depths[valid]
        if len(depths) == 0:
            return 0

        # 动态遮罩: 跳过落在 exclude_boxes 内的像素
        if exclude_boxes is not None and len(exclude_boxes) > 0:
            dynamic_mask = np.zeros(len(cc), dtype=bool)
            for box in exclude_boxes:
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
                in_box = (cc >= x1) & (cc <= x2) & (rr >= y1) & (rr <= y2)
                dynamic_mask |= in_box
            keep = ~dynamic_mask
            cc, rr, depths = cc[keep], rr[keep], depths[keep]
            if len(depths) == 0:
                return 0

        # 反投影到相机坐标系
        x_cam = (cc - cx) * depths / fx
        y_cam = (rr - cy) * depths / fy
        z_cam = depths

        # 变换到世界坐标系
        ones = np.ones_like(z_cam)
        pts_cam = np.stack([x_cam, y_cam, z_cam, ones], axis=1)   # Nx4
        pts_world = (camera_to_world @ pts_cam.T).T[:, :3]         # Nx3

        # BGR → RGB 颜色
        colors_rgb = color_bgr[rr, cc][:, ::-1].copy()  # Nx3 uint8

        # 写入体素表（最新帧覆盖旧值，同时记录时间戳）
        idx = (pts_world / self._voxel_size).astype(np.int32)

        with self._lock:
            for i in range(len(idx)):
                key = (int(idx[i, 0]), int(idx[i, 1]), int(idx[i, 2]))
                entry = self._voxels.get(key)
                if entry is None:
                    self._voxels[key] = [int(colors_rgb[i, 0]),
                                         int(colors_rgb[i, 1]),
                                         int(colors_rgb[i, 2]),
                                         ts_now]
                else:
                    entry[0] = int(colors_rgb[i, 0])
                    entry[1] = int(colors_rgb[i, 1])
                    entry[2] = int(colors_rgb[i, 2])
                    entry[3] = ts_now

        return len(idx)

    def get_colored_cloud(self) -> np.ndarray:
        """返回体素彩色点云，shape (N, 6)，列为 [x, y, z, r, g, b]。

        若 voxel_ttl > 0，超时体素在此懒惰删除。
        """
        ttl = self._voxel_ttl
        now = time.time()

        with self._lock:
            if ttl > 0:
                stale = [k for k, v in self._voxels.items() if (now - v[3]) > ttl]
                for k in stale:
                    del self._voxels[k]

            if not self._voxels:
                return np.empty((0, 6), dtype=np.float32)

            keys = np.array(list(self._voxels.keys()), dtype=np.float32)
            colors = np.array([[v[0], v[1], v[2]] for v in self._voxels.values()],
                              dtype=np.float32)

        positions = keys * self._voxel_size
        return np.hstack([positions, colors])

    def prune_stale(self) -> int:
        """主动删除超过 TTL 的体素，返回删除数量。TTL=0 时无操作。"""
        if self._voxel_ttl <= 0:
            return 0
        now = time.time()
        with self._lock:
            stale = [k for k, v in self._voxels.items()
                     if (now - v[3]) > self._voxel_ttl]
            for k in stale:
                del self._voxels[k]
        return len(stale)

    def remove_voxels_in_boxes_world(
        self,
        boxes_world: np.ndarray,  # (M, 6) [xmin, ymin, zmin, xmax, ymax, zmax]
    ) -> int:
        """删除位于给定世界坐标 AABB 内的体素（用于去除已检测到的动态物体残留）。

        Returns
        -------
        int
            删除的体素数量。
        """
        if len(boxes_world) == 0:
            return 0
        removed = 0
        vs = self._voxel_size
        with self._lock:
            keys_to_remove = []
            for key in self._voxels:
                wx = key[0] * vs
                wy = key[1] * vs
                wz = key[2] * vs
                for box in boxes_world:
                    if (box[0] <= wx <= box[3] and
                            box[1] <= wy <= box[4] and
                            box[2] <= wz <= box[5]):
                        keys_to_remove.append(key)
                        break
            for k in keys_to_remove:
                del self._voxels[k]
            removed = len(keys_to_remove)
        return removed

    def colorize_slam_cloud(
        self, points_world: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """用体素颜色表为 SLAM 点云上色。

        Returns
        -------
        xyzrgb : (N, 6) float32
        has_color : (N,) bool — 是否找到颜色
        """
        if len(points_world) == 0:
            return np.empty((0, 6), dtype=np.float32), np.array([], dtype=bool)

        idx = (points_world / self._voxel_size).astype(np.int32)
        colors = np.zeros((len(points_world), 3), dtype=np.float32)
        has_color = np.zeros(len(points_world), dtype=bool)

        with self._lock:
            for i in range(len(idx)):
                key = (int(idx[i, 0]), int(idx[i, 1]), int(idx[i, 2]))
                entry = self._voxels.get(key)
                if entry is not None:
                    colors[i, 0] = entry[0]
                    colors[i, 1] = entry[1]
                    colors[i, 2] = entry[2]
                    has_color[i] = True

        xyzrgb = np.hstack([points_world.astype(np.float32), colors])
        return xyzrgb, has_color

    @property
    def voxel_count(self) -> int:
        with self._lock:
            return len(self._voxels)

    def clear(self) -> None:
        with self._lock:
            self._voxels.clear()
