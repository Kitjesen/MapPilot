#!/usr/bin/env python3
"""
PCT A* 全局规划节点 (Python 实现，零 .so 依赖)
================================================
用途: 在没有 ARM64 编译 .so 的环境下（x86_64 开发机、CI）作为
      全局规划器备选，使用纯 Python 8-连通 A* 在预生成 tomogram 上规划路径。

话题:
  订阅  /nav/goal_pose    geometry_msgs/PoseStamped  — 导航目标
  订阅  /nav/odometry     nav_msgs/Odometry           — 当前位置
  发布  /nav/global_path  nav_msgs/Path               — A* 规划路径 (latched-like, 1Hz 重发)
  发布  /nav/planner_status  std_msgs/String          — IDLE/PLANNING/SUCCESS/FAILED

参数:
  tomogram_file  (string, 必填) — .pickle 文件路径，由 building2_9.pickle 格式生成
  obstacle_thr   (float, 49.9)  — traversability 障碍阈值
  republish_hz   (float, 1.0)   — 路径重发频率 (Hz)，保持 pct_path_adapter 稳定

与原 global_planner.py 的区别:
  - 不依赖 a_star.so / ele_planner.so (ARM64 only)
  - 仅支持 2D 平面 A*（ground-floor slice），不做 3D 爬坡规划
  - 无 TF 依赖：直接从 /nav/odometry 读当前位置
  - 适合 building2_9.pickle 等预生成 tomogram
"""
import heapq
import math
import os
import pickle
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                        HistoryPolicy)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, String


# ── A* helpers ────────────────────────────────────────────────────────────────

def _load_tomogram(path: str):
    """Load building2_9.pickle format tomogram.

    Returns (trav, trav_3d, res, cx, cy, nx, ny, ox, oy, n_slices, slice_h0, slice_dh)
    where trav[i,j] < obstacle_thr means traversable (ground floor),
    trav_3d[k,i,j] is traversability for slice k.
    """
    with open(path, 'rb') as f:
        d = pickle.load(f)
    trav_3d = np.asarray(d['data'][0], dtype=np.float32)  # (n_slices, nx, ny)
    trav    = trav_3d[0]                                   # ground-floor (nx, ny)
    res     = float(d['resolution'])
    cx      = float(d['center'][0])
    cy      = float(d['center'][1])
    n_slices, nx, ny = trav_3d.shape
    slice_h0 = float(d.get('slice_h0', 0.5))
    slice_dh = float(d.get('slice_dh', 0.5))
    return (trav, trav_3d, res, cx, cy, nx, ny, nx // 2, ny // 2,
            n_slices, slice_h0, slice_dh)


def _w2g(wx, wy, cx, cy, res, ox, oy, nx, ny):
    """World (m) → grid index (clipped)."""
    ix = int(round((wx - cx) / res)) + ox
    iy = int(round((wy - cy) / res)) + oy
    return np.clip(ix, 0, nx - 1), np.clip(iy, 0, ny - 1)


def _g2w(ix, iy, cx, cy, res, ox, oy):
    """Grid index → world (m)."""
    return (ix - ox) * res + cx, (iy - oy) * res + cy


def _downsample_path(cells, min_dist_cells=3):
    """等距降采样：保证相邻保留点距离 >= min_dist_cells 格。"""
    if len(cells) <= 2:
        return cells
    kept = [cells[0]]
    for c in cells[1:]:
        dx = c[0] - kept[-1][0]
        dy = c[1] - kept[-1][1]
        if (dx*dx + dy*dy) >= min_dist_cells * min_dist_cells:
            kept.append(c)
    if kept[-1] != cells[-1]:
        kept.append(cells[-1])
    return kept


def _catmull_rom_smooth(cells, n_interp: int = 3):
    """Catmull-Rom 样条平滑：在每对相邻格点间插入 n_interp 个中间点。

    使用重复端点（clamp-end）处理首尾，使路径从起点出发到终点结束。
    返回 list of (ix, iy) float 元组（坐标为浮点，供 _g2w 使用）。
    """
    if len(cells) < 2:
        return cells
    # 构造控制点序列：首尾重复
    pts = [cells[0]] + list(cells) + [cells[-1]]
    result = []
    for i in range(1, len(pts) - 2):
        p0 = np.array(pts[i - 1], dtype=float)
        p1 = np.array(pts[i],     dtype=float)
        p2 = np.array(pts[i + 1], dtype=float)
        p3 = np.array(pts[i + 2], dtype=float)
        if i == 1:
            result.append(tuple(p1))
        for k in range(1, n_interp + 1):
            t = k / (n_interp + 1)
            t2, t3 = t * t, t * t * t
            pt = 0.5 * ((2 * p1)
                        + (-p0 + p2) * t
                        + (2*p0 - 5*p1 + 4*p2 - p3) * t2
                        + (-p0 + 3*p1 - 3*p2 + p3) * t3)
            result.append((float(pt[0]), float(pt[1])))
        result.append(tuple(p2))
    return result


def _find_nearest_free(trav, ci, cj, obs_thr, radius=5):
    """在 radius 格范围内找最近可行格，返回 (ni, nj) 或原点。"""
    if trav[ci, cj] < obs_thr:
        return ci, cj
    best, best_dist = (ci, cj), float('inf')
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            ni, nj = ci + di, cj + dj
            if 0 <= ni < trav.shape[0] and 0 <= nj < trav.shape[1]:
                if trav[ni, nj] < obs_thr:
                    d = di * di + dj * dj
                    if d < best_dist:
                        best, best_dist = (ni, nj), d
    return best


def _ccw(A, B, C):
    """True if A->B->C is counter-clockwise (2D cross product > 0)."""
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


def _segments_intersect(A, B, C, D):
    """True if line segment AB intersects CD (proper intersection only)."""
    return (_ccw(A, C, D) != _ccw(B, C, D)) and (_ccw(A, B, C) != _ccw(A, B, D))


def _check_path_self_intersection(cells):
    """Check if path has self-intersections.

    Uses line segment intersection test on non-adjacent segments.
    Returns number of intersections found (0 = clean path).
    """
    n = len(cells)
    if n < 4:
        return 0
    count = 0
    for i in range(n - 1):
        # Skip adjacent and near-adjacent segments (they share endpoints)
        for j in range(i + 2, n - 1):
            if i == 0 and j == n - 2:
                continue  # skip first-last pair for closed-ish paths
            if _segments_intersect(cells[i], cells[i + 1],
                                   cells[j], cells[j + 1]):
                count += 1
    return count


def _smooth_z_path(poses_xyz, target_grade=0.6, res=0.2):
    """将 3D A* 路径的 Z 跳变分摊到更长的水平段, 使高度变化更接近真实楼梯坡度.

    PCT 番茄图 slice_dh(0.5m) >> res(0.2m), 导致 3D A* 路径楼层切换坡度 250%+.
    本函数将每段连续 Z 变化整体拉伸到 target_grade 坡度对应的水平跨度.
    多个连续跳变作为一个整体处理, 避免重叠窗口冲突.

    Args:
        poses_xyz:    [(wx, wy, wz), ...] 原始路径点
        target_grade: 目标坡度 (垂直/水平比), 0.6 ≈ 30° ≈ 普通楼梯
        res:          水平栅格分辨率 (m)

    Returns:
        [(wx, wy, wz_smoothed), ...] Z 平滑后的路径点 (XY 不变)
    """
    if len(poses_xyz) < 2:
        return list(poses_xyz)

    xs = [p[0] for p in poses_xyz]
    ys = [p[1] for p in poses_xyz]
    zs_raw = [p[2] for p in poses_xyz]

    # 累计水平距离
    horiz = [0.0]
    for i in range(1, len(xs)):
        horiz.append(horiz[-1] + math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1]))
    total_h = horiz[-1]
    if total_h < 1e-6:
        return list(poses_xyz)

    # 将连续 Z 跳变聚合成"过渡段" (consecutive steps where Z changes)
    # transition: (i_first_jump, i_last_jump, z_start, z_end)
    transitions = []
    i = 1
    while i < len(zs_raw):
        dz = zs_raw[i] - zs_raw[i - 1]
        if abs(dz) > 0.05:
            j = i
            z_start = zs_raw[i - 1]
            while j + 1 < len(zs_raw) and abs(zs_raw[j + 1] - zs_raw[j]) > 0.05:
                j += 1
            z_end = zs_raw[j]
            transitions.append([i, j, z_start, z_end])
            i = j + 1
        else:
            i += 1

    if not transitions:
        return list(poses_xyz)

    # 合并相邻过渡段: 若两段间隔 < 合并后所需水平跨度, 则合并为一段
    # 防止独立窗口重叠导致前一段被后一段覆盖 (产生超高坡度)
    merged = [transitions[0]]
    for tr in transitions[1:]:
        last = merged[-1]
        gap = horiz[tr[0]] - horiz[last[1]]          # 两段间的水平间隔
        combined_dz = abs(tr[3] - last[2])            # 合并后总 Z 变化
        span_for_combined = combined_dz / target_grade  # 合并后所需水平跨度
        if gap < span_for_combined:
            # 间隔太小, 合并: 保留首段起点 + 末段终点
            merged[-1] = [last[0], tr[1], last[2], tr[3]]
        else:
            merged.append(tr)
    transitions = [tuple(m) for m in merged]

    zs = list(zs_raw)

    for i_first, i_last, z_start, z_end in transitions:
        dz_total = z_end - z_start
        span_needed = abs(dz_total) / target_grade  # 需要的水平跨度 (m)

        # 以整段跳变的中心为基准, 向前后各延伸 span/2
        h_center = (horiz[i_first] + horiz[i_last]) / 2.0
        h_smooth_start = max(0.0, h_center - span_needed / 2.0)
        h_smooth_end   = min(total_h, h_smooth_start + span_needed)
        actual_span    = h_smooth_end - h_smooth_start

        for k, h in enumerate(horiz):
            if h_smooth_start <= h <= h_smooth_end:
                t = (h - h_smooth_start) / max(actual_span, 1e-6)
                zs[k] = z_start + t * dz_total
            # 跳变段结束后的点保持 z_end (已由 zs_raw 覆盖, 无需额外处理)

    return [(xs[i], ys[i], zs[i]) for i in range(len(xs))]


def _astar_3d(trav_3d, start, goal, obs_thr, timeout_sec=10.0):
    """26-connected 3D A* on multi-slice traversability grid.

    start/goal: (iz, ix, iy)  —  iz is slice index, ix/iy are grid coords
    Returns list of (iz, ix, iy) from start to goal, or None if unreachable.
    Vertical movement costs 2.5× to prefer staying on a floor where possible.
    """
    n_slices, nx, ny = trav_3d.shape
    deadline = time.monotonic() + timeout_sec

    def h(a, b):
        return (abs(a[0] - b[0]) * 2.5 +
                abs(a[1] - b[1]) + abs(a[2] - b[2]))

    open_q   = [(h(start, goal), 0.0, start)]
    g_score  = {start: 0.0}
    came_from = {}

    while open_q:
        if time.monotonic() > deadline:
            raise TimeoutError(
                f'3D A* timeout after {timeout_sec}s '
                f'(visited {len(g_score)} nodes)')
        _, g, cur = heapq.heappop(open_q)
        if g > g_score.get(cur, float('inf')) + 1e-9:
            continue  # stale entry
        if cur == goal:
            path = []
            node = cur
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path
        iz, ix, iy = cur
        for dz in (-1, 0, 1):
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dz == dx == dy == 0:
                        continue
                    # 禁止纯垂直移动 (dz≠0 但 dx=dy=0): 物理上不存在竖直电梯/飞行
                    # 楼梯/坡道必须同时有水平位移才能改变楼层
                    if dz != 0 and dx == 0 and dy == 0:
                        continue
                    nz, ni, nj = iz + dz, ix + dx, iy + dy
                    if not (0 <= nz < n_slices and 0 <= ni < nx and 0 <= nj < ny):
                        continue
                    if trav_3d[nz, ni, nj] >= obs_thr:
                        continue
                    nb = (nz, ni, nj)
                    n_changed = (abs(dz) > 0) + (abs(dx) > 0) + (abs(dy) > 0)
                    step = (1.732 if n_changed == 3 else
                            1.414 if n_changed == 2 else 1.0)
                    if dz != 0:
                        # 坡道/楼梯惩罚: slice_dh(0.5m)/res(0.2m) = 2.5 倍水平距离当量
                        # 配合禁纯垂直, 强制路径沿斜坡移动而非"飞"
                        step *= 2.5  # vertical movement penalty
                    ng = g + step
                    if ng < g_score.get(nb, float('inf')):
                        g_score[nb] = ng
                        came_from[nb] = cur
                        heapq.heappush(open_q, (ng + h(nb, goal), ng, nb))
    return None


def _astar(trav, start, goal, obs_thr, timeout_sec=5.0):
    """8-connected A* on traversability grid.

    Returns list of (ix, iy) from start to goal, or None if unreachable.
    Raises TimeoutError if planning exceeds timeout_sec.
    """
    deadline = time.monotonic() + timeout_sec
    h = lambda a, b: abs(a[0] - b[0]) + abs(a[1] - b[1])
    open_q = [(h(start, goal), 0.0, start, [])]
    visited = {}
    while open_q:
        if time.monotonic() > deadline:
            raise TimeoutError(
                f'A* timeout after {timeout_sec}s '
                f'(visited {len(visited)} nodes)')
        f, g, cur, path = heapq.heappop(open_q)
        if cur in visited:
            continue
        visited[cur] = True
        path = path + [cur]
        if cur == goal:
            return path
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            ni, nj = cur[0] + dx, cur[1] + dy
            if 0 <= ni < trav.shape[0] and 0 <= nj < trav.shape[1]:
                if trav[ni, nj] < obs_thr and (ni, nj) not in visited:
                    cost = g + (1.414 if dx and dy else 1.0)
                    heapq.heappush(
                        open_q, (cost + h((ni, nj), goal), cost, (ni, nj), path))
    return None


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class PctPlannerAstar(Node):
    """Pure-Python A* global planner using a pre-built tomogram pickle."""

    def __init__(self):
        super().__init__('pct_planner_astar')
        self._cbg = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('tomogram_file', '')
        self.declare_parameter('obstacle_thr',  49.9)
        self.declare_parameter('republish_hz',  1.0)
        self.declare_parameter('smooth_min_dist', 3)
        self.declare_parameter('loc_quality_min', 0.3)
        self.declare_parameter('smooth_method', 'catmull_rom')
        self.declare_parameter('smooth_interp',  3)
        self.declare_parameter('astar_timeout_sec', 5.0)

        tomo_file    = self.get_parameter('tomogram_file').value
        self._obs    = self.get_parameter('obstacle_thr').value
        repub_hz     = self.get_parameter('republish_hz').value
        self._smooth_min_dist = self.get_parameter('smooth_min_dist').value
        self._loc_quality_min = self.get_parameter('loc_quality_min').value
        self._smooth_method = self.get_parameter('smooth_method').value
        self._smooth_interp  = self.get_parameter('smooth_interp').value
        self._astar_timeout  = self.get_parameter('astar_timeout_sec').value

        if not tomo_file or not os.path.isfile(tomo_file):
            self.get_logger().error(
                f'tomogram_file not found: "{tomo_file}"\n'
                '  Set parameter tomogram_file:=/path/to/building.pickle')
            raise RuntimeError('tomogram_file required')

        # Load tomogram
        self.get_logger().info(f'Loading tomogram: {tomo_file}')
        (self._trav, self._trav_3d, self._res, self._cx, self._cy,
         self._nx, self._ny, self._ox, self._oy,
         self._n_slices, self._slice_h0, self._slice_dh) = _load_tomogram(tomo_file)
        free_count = int((self._trav < self._obs).sum())
        self.get_logger().info(
            f'Tomogram ready: {self._trav.shape}, res={self._res}m, '
            f'center=({self._cx:.3f},{self._cy:.3f}), '
            f'free={free_count} cells / {self._nx * self._ny}, '
            f'slices={self._n_slices} h0={self._slice_h0}m dh={self._slice_dh}m '
            f'Z=[{self._slice_h0:.1f},{self._slice_h0+(self._n_slices-1)*self._slice_dh:.1f}]m')

        # Robot state
        self._rx = 0.0
        self._ry = 0.0
        self._rz = 0.0   # robot Z (from odometry, 3D mode)
        self._plan_lock = threading.Lock()
        self._last_path: Path | None = None
        self._loc_quality = 1.0  # 默认允许，无信号时不阻塞

        # QoS
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)

        # Publishers
        self._pub_path   = self.create_publisher(Path,   '/nav/global_path',     latched)
        self._pub_status = self.create_publisher(String, '/nav/planner_status',  10)

        # Subscribers
        self.create_subscription(
            Odometry, '/nav/odometry', self._on_odom, be,
            callback_group=self._cbg)
        self.create_subscription(
            PoseStamped, '/nav/goal_pose', self._on_goal, 10,
            callback_group=self._cbg)
        self.create_subscription(
            Float32, '/nav/localization_quality', self._on_loc_quality, be,
            callback_group=self._cbg)

        # Re-publish timer (keep pct_path_adapter fed)
        self.create_timer(1.0 / repub_hz, self._republish)

        self._publish_status('IDLE')
        self.get_logger().info(
            'PctPlannerAstar ready — waiting for /nav/goal_pose')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        self._rx = msg.pose.pose.position.x
        self._ry = msg.pose.pose.position.y
        self._rz = msg.pose.pose.position.z

    def _on_loc_quality(self, msg: Float32):
        self._loc_quality = msg.data

    def _on_goal(self, msg: PoseStamped):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gz = msg.pose.position.z   # 0.0 = 2D goal; >0.1 = 3D goal with Z
        self.get_logger().info(f'Goal received: ({gx:.2f}, {gy:.2f}, z={gz:.2f})')
        if self._loc_quality < self._loc_quality_min:
            self.get_logger().warn(
                f'Localization quality {self._loc_quality:.3f} < {self._loc_quality_min:.3f}, rejecting goal')
            self._publish_status('FAILED')
            return
        if self._plan_lock.locked():
            self.get_logger().warn('Planner busy, dropping goal')
            return
        self._plan_and_publish(gx, gy, gz)

    # ── planning ──────────────────────────────────────────────────────────────

    def _plan_and_publish(self, gx: float, gy: float, gz: float = 0.0):
        """根据 gz 决定使用 2D 或 3D A*。"""
        if gz > 0.1 and self._n_slices > 1:
            self._plan_3d(gx, gy, gz)
        else:
            self._plan_2d(gx, gy)

    def _plan_2d(self, gx: float, gy: float):
        """原有 2D A* 规划（ground-floor）。"""
        with self._plan_lock:
            self._publish_status('PLANNING')
            sx, sy = self._rx, self._ry
            si, sj = _w2g(sx, sy, self._cx, self._cy,
                          self._res, self._ox, self._oy, self._nx, self._ny)
            gi, gj = _w2g(gx, gy, self._cx, self._cy,
                          self._res, self._ox, self._oy, self._nx, self._ny)

            # 起终点障碍物修正
            si, sj = _find_nearest_free(self._trav, si, sj, self._obs, radius=3)
            gi, gj = _find_nearest_free(self._trav, gi, gj, self._obs, radius=5)
            if self._trav[gi, gj] >= self._obs:
                self.get_logger().warn('Goal unreachable even after nearest-free search')
                self._publish_status('FAILED')
                return

            try:
                cells = _astar(self._trav, (si, sj), (gi, gj), self._obs,
                               timeout_sec=self._astar_timeout)
            except TimeoutError as e:
                self.get_logger().error(f'A* planning timeout: {e}')
                self._publish_status('FAILED')
                return
            if cells is None:
                self.get_logger().warn(
                    f'A* no path ({sx:.2f},{sy:.2f})→({gx:.2f},{gy:.2f}), '
                    'using straight-line fallback')
                cells = [(si, sj), (gi, gj)]
                self._publish_status('FAILED')
            else:
                self._publish_status('SUCCESS')

            if cells and len(cells) > 2:
                cells = _downsample_path(cells, min_dist_cells=self._smooth_min_dist)
                if self._smooth_method == 'catmull_rom' and len(cells) >= 2:
                    cells = _catmull_rom_smooth(cells, n_interp=self._smooth_interp)

            n_intersections = _check_path_self_intersection(cells)
            if n_intersections > 0:
                self.get_logger().warn(
                    f'Path has {n_intersections} self-intersection(s), '
                    f'{len(cells)} waypoints')

            path_msg = Path()
            path_msg.header.stamp    = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            for ci, cj in cells:
                wx, wy = _g2w(ci, cj, self._cx, self._cy, self._res,
                              self._ox, self._oy)
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.pose.position.x = wx
                ps.pose.position.y = wy
                ps.pose.position.z = 0.0
                ps.pose.orientation.w = 1.0
                path_msg.poses.append(ps)

            self._last_path = path_msg
            self._pub_path.publish(path_msg)
            self.get_logger().info(
                f'A* path published: {len(cells)} cells  '
                f'({sx:.2f},{sy:.2f})→({gx:.2f},{gy:.2f})')

    def _plan_3d(self, gx: float, gy: float, gz: float):
        """3D A* 规划，使用 tomogram 全部 slices。"""
        with self._plan_lock:
            self._publish_status('PLANNING')
            sx, sy, sz = self._rx, self._ry, self._rz

            # 世界坐标 → 栅格坐标
            si, sj = _w2g(sx, sy, self._cx, self._cy,
                          self._res, self._ox, self._oy, self._nx, self._ny)
            gi, gj = _w2g(gx, gy, self._cx, self._cy,
                          self._res, self._ox, self._oy, self._nx, self._ny)

            # Z → slice index (clamp to valid range)
            iz_s = int(np.clip(
                round((sz - self._slice_h0) / self._slice_dh), 0, self._n_slices - 1))
            iz_g = int(np.clip(
                round((gz - self._slice_h0) / self._slice_dh), 0, self._n_slices - 1))

            self.get_logger().info(
                f'[3D] A* start=({sx:.2f},{sy:.2f},{sz:.2f}) iz={iz_s} '
                f'goal=({gx:.2f},{gy:.2f},{gz:.2f}) iz={iz_g}')

            # 起终点障碍修正（在目标 slice 上寻找最近自由格）
            if self._trav_3d[iz_s, si, sj] >= self._obs:
                si, sj = _find_nearest_free(
                    self._trav_3d[iz_s], si, sj, self._obs, radius=5)
            if self._trav_3d[iz_g, gi, gj] >= self._obs:
                gi, gj = _find_nearest_free(
                    self._trav_3d[iz_g], gi, gj, self._obs, radius=8)
            if self._trav_3d[iz_g, gi, gj] >= self._obs:
                self.get_logger().warn('[3D] Goal slice unreachable, falling back to 2D')
                self._plan_2d(gx, gy)
                return

            try:
                cells_3d = _astar_3d(
                    self._trav_3d,
                    (iz_s, si, sj), (iz_g, gi, gj),
                    self._obs,
                    timeout_sec=self._astar_timeout * 2)
            except TimeoutError as e:
                self.get_logger().error(f'[3D] A* timeout: {e}')
                self._publish_status('FAILED')
                return

            if cells_3d is None:
                self.get_logger().warn('[3D] No 3D path found, falling back to 2D')
                self._plan_2d(gx, gy)
                return

            self._publish_status('SUCCESS')

            # 构建带真实 Z 坐标的路径 (含 Z 平滑: 将楼层切换分摊到足够长的水平段)
            raw_poses = []
            for iz, ci, cj in cells_3d:
                wx, wy = _g2w(ci, cj, self._cx, self._cy, self._res,
                              self._ox, self._oy)
                wz = self._slice_h0 + iz * self._slice_dh
                raw_poses.append((wx, wy, wz))

            # Z 平滑后处理:
            # PCT 番茄图 slice_dh=0.5m >> res=0.2m, 导致 A* 路径的 Z 跳变
            # 以 250% 坡度出现 (等效"飞行"). 将跳变沿路径平滑分摊,
            # 目标坡度 ≤ target_grade (60% 对应普通楼梯约 30°坡度).
            smooth_poses = _smooth_z_path(
                raw_poses, target_grade=0.6, res=self._res)

            path_msg = Path()
            path_msg.header.stamp    = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            for wx, wy, wz in smooth_poses:
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.pose.position.x = wx
                ps.pose.position.y = wy
                ps.pose.position.z = wz   # ← Z 平滑后的高度
                ps.pose.orientation.w = 1.0
                path_msg.poses.append(ps)

            self._last_path = path_msg
            self._pub_path.publish(path_msg)
            self.get_logger().info(
                f'[3D] A* path: {len(cells_3d)} cells (smooth→{len(smooth_poses)}) '
                f'({sx:.2f},{sy:.2f},{sz:.2f})→({gx:.2f},{gy:.2f},{gz:.2f}), '
                f'slices iz={iz_s}→{iz_g}')

    def _republish(self):
        if self._last_path is not None:
            self._last_path.header.stamp = self.get_clock().now().to_msg()
            self._pub_path.publish(self._last_path)

    def _publish_status(self, status: str):
        self._pub_status.publish(String(data=status))


# ── entry ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PctPlannerAstar()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
