#!/usr/bin/env python3
"""
MapPilot 交互式规划演示
=======================
左键 = 设目标 → A* 全局路径规划 → localPlanner 实时避障跟踪
右键 = 添加障碍物
中键 / Backspace = 清除障碍物和路径

底层逻辑完全对齐 C++:
  - localPlanner: 36 候选路径评分 + pathScale 收缩 + 三阶段恢复
  - pathFollower: Pure Pursuit 跟踪
  - pct_adapter:  航点推进 (含最近航点跳跃) + stuck 检测 + 重规划
  - 全局路径:     A* (PCT Planner 核心同算法, 2D 简化版; Jetson .so 无法在 Windows 加载)

用法:
    python tools/interactive_planner.py
"""

import math
import sys
import heapq
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection

# ═══════════════════════════════════════════════════════════════════════════
# 世界参数
# ═══════════════════════════════════════════════════════════════════════════

WORLD_W, WORLD_H = 18.0, 12.0
GRID_RES = 0.15              # A* 栅格分辨率 (m)
ROBOT_START = (1.0, 6.0, 0.0)

# 机器人
VEHICLE_W = 0.55
VEHICLE_L = 0.7

# 规划参数 (对应 localPlanner / pathFollower C++)
ADJACENT_RANGE = 3.8
MAX_SPEED      = 1.4
MAX_ACCEL      = 1.2
MAX_YAW_RATE   = 45.0   # deg/s  (C++ pathFollower: 45.0)
YAW_RATE_GAIN  = 7.5    # (C++ pathFollower: 7.5)
LOOK_AHEAD_BASE = 0.5   # m      (C++ baseLookAheadDis_: 0.5)
LOOK_AHEAD_RATIO = 0.5  #        (C++ lookAheadRatio_: 0.5)
STOP_DIS       = 0.35
SLOWDOWN_DIS   = 2.0
WAYPOINT_DIST  = 0.5    # pct_adapter 航点间距
ARRIVAL_THRE   = 0.5    # pct_adapter 到达阈值

# 候选路径 (对应 C++ 36 rotations × 343 paths; demo 简化为 36 直线方向)
N_PATHS   = 36
PATH_LEN  = 10
PATH_STEP = 0.35
DIR_THRE  = 90.0    # C++ dirThre_: 只评估目标方向 ±90° 内的候选路径
JOY_CLAMP = 95.0    # C++ !twoWayDrive: joyDir 限幅 ±95°
FREEZE_ANG = 90.0   # C++ freezeAng_: 目标在背后 → 停止
GOAL_BEHIND_RANGE = 0.8  # C++ goalBehindRange_

# stuck 重规划参数 (对应 pct_adapter.cpp)
STUCK_TIMEOUT   = 10.0  # s
REPLAN_COOLDOWN = 3.0   # s
MAX_REPLAN      = 2

# 恢复状态机参数 (对应 localPlanner.cpp)
BLOCKED_THRE    = 2.0   # s
ROTATE_TIME     = 2.5   # s
BACKUP_TIME     = 1.5   # s
MAX_RECOVERY    = 3     # 最大恢复循环次数

FPS = 30
DT  = 1.0 / FPS


# ═══════════════════════════════════════════════════════════════════════════
# A* 全局路径规划 (模拟 PCT Planner)
# ═══════════════════════════════════════════════════════════════════════════

def _astar(start_xy, goal_xy, obstacles, res=GRID_RES):
    """
    A* 栅格路径规划 — 与 PCT Planner 核心同算法。

    PCT Planner 内部:  a_star.*.so (pybind11) 在 3D Tomogram 切片上搜索
    这里:             同一 A* 算法，简化为 2D 障碍物膨胀栅格
    (Jetson ARM .so 无法在 Windows 开发机加载，故用纯 Python 实现)

    obstacles: [(cx, cy, radius), ...]
    返回 [(x, y), ...] 路径点列表，或 None 如果无解。
    """
    # 栅格范围
    margin = 0.5
    nx = int((WORLD_W + 2 * margin) / res)
    ny = int((WORLD_H + 2 * margin) / res)
    ox, oy = -margin, -margin  # 栅格原点

    def to_grid(x, y):
        return int((x - ox) / res), int((y - oy) / res)

    def to_world(gx, gy):
        return ox + (gx + 0.5) * res, oy + (gy + 0.5) * res

    sg = to_grid(*start_xy)
    gg = to_grid(*goal_xy)

    # 障碍物膨胀 (机器人半径 + 安全裕量)
    inflate = VEHICLE_W / 2 + 0.15
    blocked = set()
    for cx, cy, r in obstacles:
        ir = int((r + inflate) / res) + 1
        gcx, gcy = to_grid(cx, cy)
        for dx in range(-ir, ir + 1):
            for dy in range(-ir, ir + 1):
                wx, wy = to_world(gcx + dx, gcy + dy)
                if math.hypot(wx - cx, wy - cy) < r + inflate:
                    blocked.add((gcx + dx, gcy + dy))

    # 边界检查
    def valid(gx, gy):
        return 0 <= gx < nx and 0 <= gy < ny and (gx, gy) not in blocked

    if not valid(*sg) or not valid(*gg):
        return None

    # A* 搜索
    open_set = [(0, sg)]
    g_score = {sg: 0}
    came_from = {}

    # 8方向
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)]
    costs = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414]

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == gg:
            # 回溯路径
            path = []
            while current in came_from:
                path.append(to_world(*current))
                current = came_from[current]
            path.append(to_world(*sg))
            path.reverse()
            return path

        for (dx, dy), cost in zip(dirs, costs):
            nb = (current[0] + dx, current[1] + dy)
            if not valid(*nb):
                continue
            ng = g_score[current] + cost
            if ng < g_score.get(nb, float('inf')):
                g_score[nb] = ng
                h = math.hypot(nb[0] - gg[0], nb[1] - gg[1])
                heapq.heappush(open_set, (ng + h, nb))
                came_from[nb] = current

    return None  # 无解


def _downsample_path(path, dist=WAYPOINT_DIST):
    """降采样路径 (对应 pct_adapter downsample_path, 3D距离)"""
    if not path:
        return []
    result = [path[0]]
    last = path[0]
    for p in path[1:]:
        d = math.hypot(p[0] - last[0], p[1] - last[1])
        if d >= dist:
            result.append(p)
            last = p
    if result[-1] != path[-1]:
        result.append(path[-1])
    return result


def _smooth_path(path, iterations=10, weight_smooth=0.3):
    """路径平滑 — Sebastian Thrun gradient descent, 消除 A* 栅格锯齿"""
    if len(path) <= 2:
        return path
    smoothed = [list(p) for p in path]
    for _ in range(iterations):
        for i in range(1, len(smoothed) - 1):
            for j in range(2):
                smoothed[i][j] += weight_smooth * (
                    smoothed[i - 1][j] + smoothed[i + 1][j] - 2 * smoothed[i][j])
    return [tuple(s) for s in smoothed]


# ═══════════════════════════════════════════════════════════════════════════
# 候选路径生成 (localPlanner)
# ═══════════════════════════════════════════════════════════════════════════

def _make_candidate_paths():
    """36 条 body-frame 全方向候选路径 (straight lines, 10° intervals)

    C++ localPlanner: 343 base paths × 36 rotations = 12,348 candidates
      RotLUT: angle = (10.0 * i - 180.0) deg, i = 0..35 → 覆盖 360°
      每个 rotation 将 base path 旋转到对应方向, 然后评分

    Demo 简化: 36 条直线 × 1 shape = 36 candidates, 覆盖 360°
      与 C++ 相同的方向分辨率 (10°), scoring 时按 dirThre_ 过滤
    """
    paths = []
    for i in range(N_PATHS):
        angle = math.radians(10.0 * i - 180.0)  # C++ RotLUT: -180° to +170°
        pts = [(PATH_STEP * k * math.cos(angle),
                PATH_STEP * k * math.sin(angle))
               for k in range(1, PATH_LEN + 1)]
        paths.append(pts)
    return paths

CANDIDATE_BODY = _make_candidate_paths()


def _body_to_world(pts_body, rx, ry, ryaw):
    cy, sy = math.cos(ryaw), math.sin(ryaw)
    return [(rx + cy * px - sy * py, ry + sy * px + cy * py)
            for px, py in pts_body]


def _score_path(pts_world, obstacles, goal_dir, robot_xy, margin_scale=1.0):
    """路径评分 (对应 localPlanner C++ 评分逻辑)"""
    rx, ry = robot_xy
    min_d = float('inf')
    half_w = (VEHICLE_W / 2 + 0.08) * margin_scale

    for px, py in pts_world:
        for ox, oy, r in obstacles:
            d = math.hypot(px - ox, py - oy) - r
            if d < half_w:
                return -1.0
            min_d = min(min_d, d)

    if min_d == float('inf'):
        min_d = ADJACENT_RANGE

    obs_score = min(min_d / (ADJACENT_RANGE * 0.5), 1.0)
    ex, ey = pts_world[-1]
    path_dir = math.atan2(ey - ry, ex - rx)
    diff = abs(math.atan2(math.sin(goal_dir - path_dir),
                           math.cos(goal_dir - path_dir)))
    dir_score = max(0.0, math.cos(diff))   # cos 衰减: 0°→1.0, 60°→0.5, 90°→0

    # 远离障碍时方向权重更大, 贴近障碍时避障权重更大
    if min_d > ADJACENT_RANGE * 0.4:
        return 0.3 * obs_score + 0.7 * dir_score
    return 0.55 * obs_score + 0.45 * dir_score


# ═══════════════════════════════════════════════════════════════════════════
# Robot (完整状态机: localPlanner + pathFollower + pct_adapter)
# ═══════════════════════════════════════════════════════════════════════════

class Robot:
    def __init__(self):
        self.x, self.y, self.yaw = ROBOT_START
        self.speed = 0.0
        self.wyaw = 0.0
        self.scored_paths = []
        self.best_path = []
        self.mode = "IDLE"
        self.reached = False
        self.trail = [(self.x, self.y)]

        # 全局路径 + 航点 (pct_adapter)
        self.global_path = []       # A* 规划的原始路径
        self.waypoints = []         # 降采样后的航点
        self.wp_idx = 0

        # 恢复状态机 (localPlanner)
        self._blocked_start = None
        self._recovery_state = 0
        self._recovery_start = None
        self._recovery_cycles = 0

        # stuck 重规划 (pct_adapter)
        self._stuck_start = None
        self._replan_count = 0
        self._replan_event = None
        self._replan_countdown = 0.0
        self._last_progress_time = None

        # 障碍物列表 (外部管理)
        self.obstacles = []
        self._goal_xy = None

    def set_goal(self, goal_xy, obstacles):
        """设置新目标 → A* 规划全局路径"""
        self.obstacles = list(obstacles)
        self._goal_xy = goal_xy
        path = _astar((self.x, self.y), goal_xy, self.obstacles)
        if path is None:
            self.mode = "NO_PATH"
            self.global_path = []
            self.waypoints = []
            return False
        self.global_path = _smooth_path(path)
        self.waypoints = _downsample_path(self.global_path)
        self.wp_idx = min(1, len(self.waypoints) - 1)
        self.reached = False
        self.mode = "FOLLOWING"
        self._blocked_start = None
        self._recovery_state = 0
        self._recovery_cycles = 0
        self._stuck_start = None
        self._replan_count = 0
        self._replan_event = None
        self._last_progress_time = 0.0
        return True

    # ── pct_adapter: 航点推进 (含最近航点跳跃) ──────────────────────────

    def _update_waypoint(self):
        if not self.waypoints or self.wp_idx >= len(self.waypoints):
            return
        # 限窗搜索最近航点 (+4): 防止路径折叠时跳过整段路径
        # (旧版搜索全部剩余航点, U 形路径会跳到对岸 → 大弧线)
        search_end = min(self.wp_idx + 5, len(self.waypoints))
        best_idx = self.wp_idx
        best_dist = float('inf')
        for i in range(self.wp_idx, search_end):
            d = math.hypot(self.waypoints[i][0] - self.x,
                           self.waypoints[i][1] - self.y)
            if d < best_dist:
                best_dist = d
                best_idx = i
        if best_idx > self.wp_idx:
            self.wp_idx = best_idx

        # 正常到达推进
        gx, gy = self.waypoints[self.wp_idx]
        if math.hypot(gx - self.x, gy - self.y) < ARRIVAL_THRE:
            if self.wp_idx < len(self.waypoints) - 1:
                self.wp_idx += 1

    # ── localPlanner: 候选路径评分 ────────────────────────────────────────

    def _local_plan(self, t):
        if not self.waypoints:
            return
        nearby = [(ox, oy, r) for ox, oy, r in self.obstacles
                  if math.hypot(ox - self.x, oy - self.y) < ADJACENT_RANGE + r + 0.5]

        wi = min(self.wp_idx, len(self.waypoints) - 1)
        # 沿路径前瞻 2m 计算目标方向
        look_dist = 2.0
        ahead_idx = wi
        accum = 0.0
        while ahead_idx < len(self.waypoints) - 1:
            dx = self.waypoints[ahead_idx + 1][0] - self.waypoints[ahead_idx][0]
            dy = self.waypoints[ahead_idx + 1][1] - self.waypoints[ahead_idx][1]
            accum += math.hypot(dx, dy)
            ahead_idx += 1
            if accum >= look_dist:
                break
        gx, gy = self.waypoints[ahead_idx]
        goal_dir = math.atan2(gy - self.y, gx - self.x)

        # ── C++ joyDir_ 逻辑: body-frame 目标方向 + freeze + clamp ──
        joy_dir = goal_dir - self.yaw  # body frame (rad)
        joy_dir = math.atan2(math.sin(joy_dir), math.cos(joy_dir))
        joy_deg = math.degrees(joy_dir)

        goal_dis = math.hypot(gx - self.x, gy - self.y)

        # freezeAng_: 目标在背后且很近 → 停止 (C++ goalBehindRange_)
        if abs(joy_deg) > FREEZE_ANG and goal_dis < GOAL_BEHIND_RANGE:
            joy_deg = 0.0
            joy_dir = 0.0

        # !twoWayDrive: 限幅 ±95° (C++ lines 972-980)
        joy_deg = max(-JOY_CLAMP, min(JOY_CLAMP, joy_deg))
        joy_dir = math.radians(joy_deg)
        goal_dir_clamped = self.yaw + joy_dir  # 世界坐标系下的受限目标方向

        # ── 全方向候选路径评分 (C++ 36 rotations × dirThre_ filter) ──
        scored = []
        dir_thre_rad = math.radians(DIR_THRE)
        for i, pts_body in enumerate(CANDIDATE_BODY):
            rot_deg = 10.0 * i - 180.0  # body frame rotation angle
            # dirThre_ 过滤: 跳过偏离目标方向 >90° 的候选路径
            ang_diff = abs(joy_deg - rot_deg)
            if ang_diff > 180.0:
                ang_diff = 360.0 - ang_diff
            if ang_diff > DIR_THRE:
                scored.append(([], -1.0))
                continue
            pts_w = _body_to_world(pts_body, self.x, self.y, self.yaw)
            s = _score_path(pts_w, nearby, goal_dir_clamped, (self.x, self.y))
            scored.append((pts_w, s))
        self.scored_paths = scored

        best = max(scored, key=lambda x: x[1])
        if best[1] > 0:
            self.best_path = best[0]
            self._blocked_start = None
            self._recovery_state = 0
            self._recovery_cycles = 0
            self.mode = ("AVOIDING"
                         if any(math.hypot(ox - self.x, oy - self.y) < ADJACENT_RANGE
                                for ox, oy, _ in self.obstacles)
                         else "FOLLOWING")
        else:
            # pathScale 收缩 (C++ pathScale_ -= pathScaleStep_)
            found = False
            for scale in [0.7, 0.5, 0.35]:
                relaxed = []
                for i, pts_body in enumerate(CANDIDATE_BODY):
                    rot_deg = 10.0 * i - 180.0
                    ang_diff = abs(joy_deg - rot_deg)
                    if ang_diff > 180.0:
                        ang_diff = 360.0 - ang_diff
                    if ang_diff > DIR_THRE:
                        relaxed.append(([], -1.0))
                        continue
                    pts_w = _body_to_world(pts_body, self.x, self.y, self.yaw)
                    s = _score_path(pts_w, nearby, goal_dir_clamped,
                                    (self.x, self.y), margin_scale=scale)
                    relaxed.append((pts_w, s))
                best_r = max(relaxed, key=lambda x: x[1])
                if best_r[1] > 0:
                    self.scored_paths = relaxed
                    self.best_path = best_r[0]
                    self._blocked_start = None
                    self._recovery_state = 0
                    self._recovery_cycles = 0
                    found = True
                    self.mode = "AVOIDING"
                    break

            if not found:
                # 三阶段恢复状态机
                if self._blocked_start is None:
                    self._blocked_start = t
                blocked_dur = t - self._blocked_start

                if (self._recovery_state == 0 and blocked_dur >= BLOCKED_THRE
                        and self._recovery_cycles < MAX_RECOVERY):
                    self._recovery_state = 1
                    self._recovery_start = t
                elif (self._recovery_state == 1 and self._recovery_start is not None
                      and t - self._recovery_start >= ROTATE_TIME):
                    self._recovery_state = 2
                    self._recovery_start = t
                elif (self._recovery_state == 2 and self._recovery_start is not None
                      and t - self._recovery_start >= BACKUP_TIME):
                    self._recovery_state = 0
                    self._blocked_start = t
                    self._recovery_cycles += 1

                if self._recovery_state == 1:
                    rel_angle = goal_dir - self.yaw
                    rel_angle = math.atan2(math.sin(rel_angle), math.cos(rel_angle))
                    turn_dir = 1.0 if rel_angle >= 0.0 else -1.0
                    pts_body = [(0.15 * i * math.cos(turn_dir * i * 0.25),
                                 0.15 * i * math.sin(turn_dir * i * 0.25))
                                for i in range(1, 7)]
                    self.best_path = _body_to_world(pts_body, self.x, self.y, self.yaw)
                    self.mode = "ROTATING"
                elif self._recovery_state == 2:
                    pts_body = [(-0.2 * i, 0.0) for i in range(1, 6)]
                    self.best_path = _body_to_world(pts_body, self.x, self.y, self.yaw)
                    self.mode = "BACKING_UP"
                else:
                    self.best_path = []
                    self.mode = "BLOCKED"

    # ── pct_adapter: stuck 检测 + 全局重规划 ──────────────────────────────

    def _check_stuck_replan(self, t):
        self._replan_event = None
        if not self.waypoints or self.reached:
            return
        is_stuck = self.mode in ("BLOCKED", "ROTATING", "BACKING_UP")
        if is_stuck:
            if self._stuck_start is None:
                self._stuck_start = t
            stuck_dur = t - self._stuck_start
            self._replan_countdown = max(0.0, STUCK_TIMEOUT - stuck_dur)
            if stuck_dur >= STUCK_TIMEOUT:
                if self._replan_count < MAX_REPLAN:
                    self._do_replan(t)
                else:
                    self._replan_event = "stuck_final"
        else:
            self._stuck_start = None
            self._replan_countdown = 0.0

    def _do_replan(self, t):
        """触发 A* 重规划 (模拟 pct_adapter 重发 goal_pose → PCT 重新规划)"""
        if self._goal_xy is None:
            return
        path = _astar((self.x, self.y), self._goal_xy, self.obstacles)
        self._replan_count += 1
        self._stuck_start = None
        self._replan_event = "replanning"
        if path:
            self.global_path = _smooth_path(path)
            self.waypoints = _downsample_path(self.global_path)
            self.wp_idx = min(1, len(self.waypoints) - 1)
            self._blocked_start = None
            self._recovery_state = 0
            self._recovery_cycles = 0
            self._last_progress_time = t

    # ── pathFollower: Pure Pursuit ────────────────────────────────────────

    def _follow(self):
        if self.mode == "BACKING_UP" and self.best_path:
            target = -MAX_SPEED * 0.35
            step = MAX_ACCEL * DT
            self.speed = max(self.speed - step, target)
            self.wyaw = 0.0
            return

        if not self.best_path:
            self.speed = max(self.speed - MAX_ACCEL * DT * 2, 0.0)
            self.wyaw = 0.0
            return

        ex, ey = self.best_path[-1]
        end_dis = math.hypot(ex - self.x, ey - self.y)

        # 以实际目标距离限制减速 (候选路径端点总是 ~3.5m 远, 不能用它减速)
        if self.waypoints:
            fx, fy = self.waypoints[-1]
            end_dis = min(end_dis, math.hypot(fx - self.x, fy - self.y))

        # 航向控制: 直接跟踪候选路径端点方向 (而非 look-ahead 中间点)
        # 原因: 36 条恒定曲率弧线在 look-ahead 距离 (0.5-1.2m) 处弯曲极小,
        # Pure Pursuit 跟踪中间点 → 转弯迟钝 → 近距离目标大弧线绕行
        # C++ 有 343 条路径形状 (含急弯), 短 look-ahead 仍有效;
        # Demo 只有 36 条, 改用端点方向确保转向灵敏
        path_dir = math.atan2(ey - self.y, ex - self.x)
        dir_diff = math.atan2(math.sin(path_dir - self.yaw),
                               math.cos(path_dir - self.yaw))
        self.wyaw = YAW_RATE_GAIN * dir_diff
        max_wr = MAX_YAW_RATE * math.pi / 180
        self.wyaw = max(-max_wr, min(max_wr, self.wyaw))

        target = MAX_SPEED
        if end_dis < SLOWDOWN_DIS:
            target = MAX_SPEED * (end_dis / SLOWDOWN_DIS)
        if end_dis < STOP_DIS:
            target = 0.0
        if abs(dir_diff) > 0.4:
            target *= max(0.25, 1.0 - abs(dir_diff) / math.pi)

        step = MAX_ACCEL * DT
        self.speed = (min(self.speed + step, target) if self.speed < target
                      else max(self.speed - step, target))

    # ── 仿真步进 ──────────────────────────────────────────────────────────

    def step(self, t):
        if self.reached or not self.waypoints:
            self.speed = max(self.speed - MAX_ACCEL * DT * 2, 0.0)
            return
        self._update_waypoint()
        self._local_plan(t)
        self._check_stuck_replan(t)
        self._follow()

        self.x += self.speed * math.cos(self.yaw) * DT
        self.y += self.speed * math.sin(self.yaw) * DT
        self.yaw += self.wyaw * DT
        self.trail.append((self.x, self.y))

        # 到达终点
        if self.waypoints:
            gx, gy = self.waypoints[-1]
            if math.hypot(gx - self.x, gy - self.y) < STOP_DIS:
                self.reached = True
                self.speed = 0.0
                self.mode = "REACHED"


# ═══════════════════════════════════════════════════════════════════════════
# 可视化工具
# ═══════════════════════════════════════════════════════════════════════════

def _robot_polygon(rx, ry, ryaw):
    L, W = VEHICLE_L, VEHICLE_W
    corners = [(-L/2, -W/2), (L/2, -W/2), (L/2, W/2), (-L/2, W/2)]
    cy, sy = math.cos(ryaw), math.sin(ryaw)
    return [(rx + cy*x - sy*y, ry + sy*x + cy*y) for x, y in corners]


def _colormap_score(s):
    if s < 0:
        return '#ff4444'
    r = max(0.0, 1.0 - s * 1.5)
    g = min(1.0, s * 1.5)
    return (r, g, 0.1)


# ═══════════════════════════════════════════════════════════════════════════
# 交互式主循环
# ═══════════════════════════════════════════════════════════════════════════

class InteractivePlanner:
    def __init__(self):
        self.robot = Robot()
        self.obstacles = [
            (5.0, 3.0, 0.7), (5.0, 9.0, 0.7),
            (9.0, 4.0, 0.8), (9.0, 8.0, 0.6),
            (13.0, 3.5, 0.7), (13.0, 8.5, 0.7),
        ]
        self.robot.obstacles = list(self.obstacles)
        self.t = 0.0
        self.running = False
        self.goal_marker = None

        self._setup_figure()
        self._draw_static()

    def _setup_figure(self):
        self.fig = plt.figure(figsize=(16, 9.5), facecolor='#0d1117')
        self.ax = self.fig.add_axes([0.01, 0.08, 0.72, 0.88])
        self.ax_info = self.fig.add_axes([0.74, 0.08, 0.24, 0.88])

        self.ax.set_facecolor('#161b22')
        self.ax_info.set_facecolor('#0d1117')
        self.ax_info.axis('off')

        self.ax.set_xlim(-0.5, WORLD_W + 0.5)
        self.ax.set_ylim(-0.5, WORLD_H + 0.5)
        self.ax.set_aspect('equal')
        self.ax.tick_params(colors='#58a6ff', labelsize=7)
        self.ax.spines[:].set_color('#30363d')
        self.ax.grid(True, color='#21262d', linewidth=0.5, zorder=0)
        self.ax.set_xlabel('X (m)', color='#8b949e', fontsize=9)
        self.ax.set_ylabel('Y (m)', color='#8b949e', fontsize=9)

        self.title_obj = self.ax.set_title(
            'MapPilot Interactive — 左键设目标 | 右键加障碍 | Backspace清除',
            color='#c9d1d9', fontsize=11, pad=6)

        # 事件绑定
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

        # 创建 artists
        self.arts = {}
        self._create_artists()

    def _create_artists(self):
        ax = self.ax
        arts = self.arts

        # 全局路径
        arts['global_path'], = ax.plot([], [], '--', color='#1f6feb',
                                        linewidth=1.8, alpha=0.7, zorder=2)
        arts['waypoints'] = ax.scatter([], [], color='#388bfd', s=20,
                                        zorder=3, marker='o')

        # 检测范围
        arts['detect_zone'] = plt.Circle((0, 0), ADJACENT_RANGE,
                                          fill=False, color='#1f6feb',
                                          linewidth=0.8, alpha=0.2,
                                          linestyle='--', zorder=3)
        ax.add_patch(arts['detect_zone'])

        # 候选路径
        dummy_segs = [[(0, 0), (0, 0)]] * N_PATHS
        arts['cand_lc'] = LineCollection(dummy_segs, linewidths=0.7,
                                          alpha=0.55, zorder=5)
        ax.add_collection(arts['cand_lc'])

        # 选中局部路径
        arts['best_path'], = ax.plot([], [], '-', color='#3fb950',
                                      linewidth=2.8, zorder=8)

        # 当前航点标记
        arts['cur_waypoint'] = ax.scatter([], [], color='#ffd700', s=120,
                                           marker='*', zorder=9,
                                           edgecolors='#b8860b', linewidths=0.8)

        # 轨迹
        arts['trail'], = ax.plot([], [], '-', color='#79c0ff',
                                  linewidth=1.2, alpha=0.5, zorder=6)

        # 机器人
        dummy_poly = plt.Polygon([(0, 0)] * 4, closed=True,
                                  facecolor='#58a6ff', edgecolor='#cae8ff',
                                  linewidth=1.5, zorder=12)
        ax.add_patch(dummy_poly)
        arts['robot_body'] = dummy_poly

        arts['robot_arrow'] = ax.annotate('', xy=(0, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle='->', color='#cae8ff', lw=2), zorder=13)

        # 起点标记
        sx, sy = ROBOT_START[:2]
        ax.scatter([sx], [sy], s=80, marker='D', color='#79c0ff',
                   edgecolors='#58a6ff', linewidths=1, zorder=14)
        ax.text(sx - 0.1, sy + 0.4, 'START', color='#79c0ff',
                fontsize=8, fontweight='bold', zorder=14)

        # 目标标记 (初始不显示)
        arts['goal_marker'] = ax.scatter([], [], s=200, marker='*',
                                          color='#ffd700', edgecolors='#b8860b',
                                          linewidths=1.5, zorder=15)
        arts['goal_text'] = ax.text(0, 0, '', color='#ffd700',
                                     fontsize=9, fontweight='bold', zorder=15)

        # 障碍物 patches (可动态添加)
        arts['obs_patches'] = []

        # ── 信息面板 ──
        axi = self.ax_info
        arts['info_title'] = axi.text(0.5, 0.97, 'MapPilot Planner',
            ha='center', va='top', transform=axi.transAxes,
            color='#58a6ff', fontsize=13, fontweight='bold')
        arts['info_sub'] = axi.text(0.5, 0.91,
            'Interactive Navigation Demo',
            ha='center', va='top', transform=axi.transAxes,
            color='#8b949e', fontsize=9)
        axi.axhline(0.88, color='#30363d', linewidth=1)

        def _label(y, key, txt):
            axi.text(0.05, y, txt, va='top', transform=axi.transAxes,
                     color='#8b949e', fontsize=8.5)
            arts[key] = axi.text(0.95, y, '--', va='top', ha='right',
                                  transform=axi.transAxes,
                                  color='#c9d1d9', fontsize=9, fontweight='bold')

        _label(0.84, 'i_mode',   'Mode')
        _label(0.78, 'i_speed',  'Speed')
        _label(0.72, 'i_wayp',   'Waypoint')
        _label(0.66, 'i_obs',    'Nearby Obs')
        _label(0.60, 'i_replan', 'Replan')
        _label(0.54, 'i_recov',  'Recovery')
        axi.axhline(0.50, color='#30363d', linewidth=1)

        # 状态消息
        arts['status_msg'] = axi.text(0.5, 0.46,
            '-- 左键点击设置目标 --',
            ha='center', va='top', transform=axi.transAxes,
            color='#8b949e', fontsize=10, fontweight='bold')
        arts['status_detail'] = axi.text(0.5, 0.38, '',
            ha='center', va='top', transform=axi.transAxes,
            color='#8b949e', fontsize=8.5, wrap=True)

        # 图例
        legend_elems = [
            mpatches.Patch(color='#1f6feb', alpha=0.7, label='Global Path (A*)'),
            mpatches.Patch(color='#3fb950', label='Local Path (selected)'),
            mpatches.Patch(color='#ffd700', alpha=0.8, label='Current Waypoint'),
            mpatches.Patch(color='#58a6ff', label='Robot'),
            mpatches.Patch(color='#30363d', ec='#6e7681', label='Obstacle'),
        ]
        ax.legend(handles=legend_elems, loc='lower right',
                  facecolor='#161b22', edgecolor='#30363d',
                  labelcolor='#c9d1d9', fontsize=7.5, framealpha=0.9)

        # 操作提示
        axi.axhline(0.12, color='#30363d', linewidth=1)
        axi.text(0.5, 0.09,
            '[ 左键 ] 设目标    [ 右键 ] 加障碍\n'
            '[ Backspace ] 清除    [ R ] 重置位置',
            ha='center', va='top', transform=axi.transAxes,
            color='#484f58', fontsize=8)

    def _draw_static(self):
        """绘制障碍物"""
        for p in self.arts['obs_patches']:
            p.remove()
        self.arts['obs_patches'] = []
        for ox, oy, r in self.obstacles:
            c = plt.Circle((ox, oy), r, color='#30363d',
                            ec='#6e7681', linewidth=1.2, zorder=4)
            self.ax.add_patch(c)
            self.arts['obs_patches'].append(c)

    def _on_click(self, event):
        if event.inaxes != self.ax:
            return
        if event.button == 1:
            # 左键: 设目标
            gx, gy = event.xdata, event.ydata
            self.robot.obstacles = list(self.obstacles)
            ok = self.robot.set_goal((gx, gy), self.obstacles)
            self.arts['goal_marker'].set_offsets([[gx, gy]])
            self.arts['goal_text'].set_position((gx + 0.2, gy + 0.3))
            self.arts['goal_text'].set_text('GOAL')
            if ok:
                self.running = True
                self.t = 0.0
                self.robot.trail = [(self.robot.x, self.robot.y)]
            else:
                self.arts['status_msg'].set_text('[X] A* 无法到达目标')
                self.arts['status_msg'].set_color('#ff4444')
                self.arts['status_detail'].set_text('目标被障碍物包围\n请换个位置')
                self.fig.canvas.draw_idle()
        elif event.button == 3:
            # 右键: 加障碍物
            ox, oy = event.xdata, event.ydata
            r = 0.5 + np.random.uniform(0, 0.3)
            self.obstacles.append((ox, oy, r))
            c = plt.Circle((ox, oy), r, color='#30363d',
                            ec='#6e7681', linewidth=1.2, zorder=4)
            self.ax.add_patch(c)
            self.arts['obs_patches'].append(c)
            self.robot.obstacles = list(self.obstacles)
            self.fig.canvas.draw_idle()

    def _on_key(self, event):
        if event.key == 'backspace':
            # 清除所有
            self.running = False
            self.robot = Robot()
            self.robot.obstacles = list(self.obstacles)
            self.obstacles.clear()
            self.obstacles.extend([
                (5.0, 3.0, 0.7), (5.0, 9.0, 0.7),
                (9.0, 4.0, 0.8), (9.0, 8.0, 0.6),
                (13.0, 3.5, 0.7), (13.0, 8.5, 0.7),
            ])
            self.robot.obstacles = list(self.obstacles)
            self._draw_static()
            self.arts['global_path'].set_data([], [])
            self.arts['waypoints'].set_offsets(np.empty((0, 2)))
            self.arts['trail'].set_data([], [])
            self.arts['best_path'].set_data([], [])
            self.arts['goal_marker'].set_offsets(np.empty((0, 2)))
            self.arts['goal_text'].set_text('')
            self.arts['status_msg'].set_text('-- 左键点击设置目标 --')
            self.arts['status_msg'].set_color('#8b949e')
            self.arts['status_detail'].set_text('')
            self.fig.canvas.draw_idle()
        elif event.key == 'r':
            # 重置机器人位置
            self.robot.x, self.robot.y, self.robot.yaw = ROBOT_START
            self.robot.speed = 0.0
            self.robot.trail = [(self.robot.x, self.robot.y)]
            self.fig.canvas.draw_idle()

    def _update_frame(self):
        """每帧更新"""
        robot = self.robot
        arts = self.arts

        if self.running and not robot.reached:
            self.t += DT
            robot.step(self.t)

        # ── 全局路径 ──
        if robot.global_path:
            gpx = [p[0] for p in robot.global_path]
            gpy = [p[1] for p in robot.global_path]
            color = '#00ccff' if robot._replan_count > 0 else '#1f6feb'
            arts['global_path'].set_data(gpx, gpy)
            arts['global_path'].set_color(color)
        if robot.waypoints:
            arts['waypoints'].set_offsets(robot.waypoints)

        # ── 检测范围 ──
        arts['detect_zone'].set_center((robot.x, robot.y))

        # ── 候选路径 ──
        if robot.scored_paths:
            segs = []
            colors = []
            for pts, score in robot.scored_paths:
                segs.append([(p[0], p[1]) for p in pts])
                colors.append(_colormap_score(score))
            arts['cand_lc'].set_segments(segs)
            arts['cand_lc'].set_colors(colors)
        else:
            arts['cand_lc'].set_segments([[(0, 0), (0, 0)]] * N_PATHS)

        # ── 选中路径 ──
        if robot.best_path:
            bx = [p[0] for p in robot.best_path]
            by = [p[1] for p in robot.best_path]
            arts['best_path'].set_data(bx, by)
        else:
            arts['best_path'].set_data([], [])

        # ── 当前航点 ──
        if robot.waypoints:
            wi = min(robot.wp_idx, len(robot.waypoints) - 1)
            arts['cur_waypoint'].set_offsets([[robot.waypoints[wi][0],
                                               robot.waypoints[wi][1]]])

        # ── 轨迹 ──
        if len(robot.trail) > 1:
            tx, ty = zip(*robot.trail)
            arts['trail'].set_data(tx, ty)

        # ── 机器人 ──
        poly_pts = _robot_polygon(robot.x, robot.y, robot.yaw)
        arts['robot_body'].set_xy(poly_pts)
        arrow_len = VEHICLE_L * 0.65
        ax_tip = robot.x + arrow_len * math.cos(robot.yaw)
        ay_tip = robot.y + arrow_len * math.sin(robot.yaw)
        arts['robot_arrow'].set_position((robot.x, robot.y))
        arts['robot_arrow'].xy = (ax_tip, ay_tip)
        arts['robot_arrow'].xytext = (robot.x, robot.y)

        # ── 信息面板 ──
        mode_colors = {
            'FOLLOWING': '#3fb950', 'AVOIDING': '#ff6e6e',
            'BLOCKED': '#ff9900', 'ROTATING': '#ff7700',
            'BACKING_UP': '#ff4400', 'REACHED': '#ffd700',
            'IDLE': '#8b949e', 'NO_PATH': '#ff4444',
        }
        arts['i_mode'].set_text(robot.mode)
        arts['i_mode'].set_color(mode_colors.get(robot.mode, '#c9d1d9'))
        arts['i_speed'].set_text(f'{robot.speed * 100:.0f} cm/s')
        if robot.waypoints:
            wi = min(robot.wp_idx, len(robot.waypoints) - 1)
            arts['i_wayp'].set_text(f'{wi + 1} / {len(robot.waypoints)}')
        else:
            arts['i_wayp'].set_text('--')

        n_nearby = sum(1 for ox, oy, _ in self.obstacles
                       if math.hypot(ox - robot.x, oy - robot.y) < ADJACENT_RANGE)
        arts['i_obs'].set_text(str(n_nearby))
        arts['i_obs'].set_color('#ff6e6e' if n_nearby > 0 else '#3fb950')

        arts['i_replan'].set_text(f'{robot._replan_count}/{MAX_REPLAN}')
        arts['i_replan'].set_color(
            '#ff6e6e' if robot._replan_count > 0 else '#3fb950')
        arts['i_recov'].set_text(
            f'{robot._recovery_cycles}/{MAX_RECOVERY}')

        # 状态消息
        if robot.mode == 'REACHED':
            arts['status_msg'].set_text('● 到达目标！')
            arts['status_msg'].set_color('#ffd700')
            note = (f'经 {robot._replan_count} 次全局重规划'
                    if robot._replan_count > 0 else '直达')
            arts['status_detail'].set_text(f'导航完成 ({note})\n点击新目标继续')
        elif robot._replan_event == 'replanning':
            arts['status_msg'].set_text('[>>] 全局重规划！')
            arts['status_msg'].set_color('#00ccff')
            arts['status_detail'].set_text(
                f'A* 从当前位置重新规划\n'
                f'重规划 {robot._replan_count}/{MAX_REPLAN}')
        elif robot._replan_event == 'stuck_final':
            arts['status_msg'].set_text('[!!] stuck_final')
            arts['status_msg'].set_color('#ff0000')
            arts['status_detail'].set_text(
                f'重规划 {MAX_REPLAN} 次仍失败\n等待语义层 LERa 接管')
        elif robot.mode == 'BLOCKED':
            cd = f'{robot._replan_countdown:.1f}s' if robot._replan_countdown > 0 else ''
            arts['status_msg'].set_text('[X] 路径受阻')
            arts['status_msg'].set_color('#ff9900')
            arts['status_detail'].set_text(
                f'pathScale 收缩无解\n'
                f'恢复 {robot._recovery_cycles}/{MAX_RECOVERY}'
                + (f'\nstuck 倒计时 {cd}' if cd else ''))
        elif robot.mode in ('ROTATING', 'BACKING_UP'):
            label = '旋转' if robot.mode == 'ROTATING' else '后退'
            cd = f'{robot._replan_countdown:.1f}s' if robot._replan_countdown > 0 else ''
            arts['status_msg'].set_text(f'[~] 恢复: {label}')
            arts['status_msg'].set_color(mode_colors.get(robot.mode, '#ff7700'))
            arts['status_detail'].set_text(
                f'C++ recoveryState={robot._recovery_state}'
                + (f'\nstuck 倒计时 {cd}' if cd else ''))
        elif robot.mode == 'AVOIDING':
            arts['status_msg'].set_text('[!] 障碍物规避')
            arts['status_msg'].set_color('#ff6e6e')
            arts['status_detail'].set_text(
                f'{N_PATHS} 条候选路径评分\nPure Pursuit 跟踪')
        elif robot.mode == 'FOLLOWING':
            arts['status_msg'].set_text('● 跟踪全局路径')
            arts['status_msg'].set_color('#3fb950')
            arts['status_detail'].set_text('pct_adapter 航点 → localPlanner')
        elif robot.mode == 'NO_PATH':
            arts['status_msg'].set_text('[X] A* 无法到达')
            arts['status_msg'].set_color('#ff4444')
            arts['status_detail'].set_text('目标不可达')

        # 标题
        mode_c = mode_colors.get(robot.mode, '#c9d1d9')
        self.title_obj.set_text(
            f'MapPilot Interactive   t={self.t:.1f}s  |  '
            f'{robot.speed*100:.0f} cm/s  |  {robot.mode}')
        self.title_obj.set_color(mode_c)

    def run(self):
        """交互式主循环"""
        timer = self.fig.canvas.new_timer(interval=1000 // FPS)
        def _tick():
            self._update_frame()
            self.fig.canvas.draw_idle()
        timer.add_callback(_tick)
        timer.start()
        plt.show()


# ═══════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    print('MapPilot 交互式规划演示')
    print('  左键 = 设目标 (A* 全局规划 + 实时避障)')
    print('  右键 = 添加障碍物')
    print('  Backspace = 清除')
    print('  R = 重置机器人位置')
    app = InteractivePlanner()
    app.run()
