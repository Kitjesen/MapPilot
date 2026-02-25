#!/usr/bin/env python3
"""
MapPilot 全局规划 + 局部避障 动画演示
=====================================
模拟 localPlanner (候选路径扇形评分) + pathFollower (Pure Pursuit) 避障过程

用法:
    python tools/viz_planner_demo.py
    python tools/viz_planner_demo.py --save demo.gif   # 保存 GIF
"""

import math
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use('TkAgg' if 'linux' not in sys.platform else 'Agg')
import matplotlib.pyplot as plt
# 中文字体支持
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection

# ═══════════════════════════════════════════════════════════════════════════
# 场景配置
# ═══════════════════════════════════════════════════════════════════════════

WORLD_W, WORLD_H = 18.0, 12.0

# 静态障碍物: (cx, cy, radius)
STATIC_OBS = [
    (3.5, 2.5, 0.7),
    (3.5, 9.5, 0.7),
    (7.0, 4.0, 0.9),
    (7.0, 8.0, 0.6),
    (11.0, 3.0, 0.7),
    (11.0, 9.0, 0.8),
    (5.5, 3.5, 0.5),
    (14.0, 5.0, 0.6),
    (14.0, 7.5, 0.5),
]

# 预规划全局路径 (绕静态障碍物)
GLOBAL_PATH = [
    (0.5,  6.0),
    (2.0,  6.0),
    (4.5,  5.5),
    (6.0,  6.0),
    (8.5,  6.0),
    (10.0, 6.0),
    (12.5, 6.0),
    (13.5, 6.2),
    (15.5, 6.0),
    (17.0, 6.0),
]

# 动态障碍物: 从下向上穿越全局路径
DYN_OBS_TRAJ = [(9.0, 1.0), (9.0, 11.0)]   # 起点→终点
DYN_OBS_R    = 0.55
DYN_OBS_SPD  = 2.2   # m/s, 仿真速度

# 机器人
ROBOT_START  = (0.5, 6.0, 0.0)   # x, y, yaw
VEHICLE_W    = 0.55
VEHICLE_L    = 0.7

# ═══════════════════════════════════════════════════════════════════════════
# 规划参数 (对应 localPlanner / pathFollower)
# ═══════════════════════════════════════════════════════════════════════════

ADJACENT_RANGE  = 3.8    # 障碍物检测半径 (m)
MAX_SPEED       = 1.4    # m/s
MAX_ACCEL       = 1.2    # m/s²
MAX_YAW_RATE    = 65.0   # deg/s
YAW_RATE_GAIN   = 5.5
LOOK_AHEAD      = 0.9    # Pure Pursuit 基础前视距离 (m)
STOP_DIS        = 0.35   # 终点停车阈值 (m)
SLOWDOWN_DIS    = 2.0    # 开始减速距离 (m)
WAYPOINT_THRE   = 1.0    # PCT adapter 航点推进阈值 (m)

# 候选路径: 36 条，角度 [-80°, +80°]
N_PATHS   = 36
PATH_LEN  = 10     # 每条路径点数
PATH_STEP = 0.35   # 路径点间距 (m)

FPS      = 25
DT       = 1.0 / FPS
SIM_TIME = 22.0    # 总仿真时间 (s)

# ═══════════════════════════════════════════════════════════════════════════
# 场景预设库
# ═══════════════════════════════════════════════════════════════════════════

SCENES = {
    # 场景1: 默认 — 动态障碍物从下方横穿
    "default": dict(
        name="动态障碍物穿越",
        desc="动态障碍从下方穿越路径，机器人实时绕行",
        static_obs=[
            (3.5, 2.5, 0.7), (3.5, 9.5, 0.7),
            (7.0, 4.0, 0.9), (7.0, 8.0, 0.6),
            (11.0, 3.0, 0.7), (11.0, 9.0, 0.8),
            (5.5, 3.5, 0.5),
            (14.0, 5.0, 0.6), (14.0, 7.5, 0.5),
        ],
        global_path=[
            (0.5, 6.0), (2.0, 6.0), (4.5, 5.5), (6.0, 6.0),
            (8.5, 6.0), (10.0, 6.0), (12.5, 6.0),
            (13.5, 6.2), (15.5, 6.0), (17.0, 6.0),
        ],
        dyn_traj=[(9.0, 1.0), (9.0, 11.0)],
        dyn_r=0.55, dyn_spd=2.2, sim_time=22.0,
    ),
    # 场景2: 窄走廊 — 两侧障碍密集，路径被大量红色路径封锁
    "corridor": dict(
        name="窄走廊穿行",
        desc="两侧障碍形成走廊，大量候选路径被阻（红色），机器人低速贴边穿行",
        static_obs=[
            (3.5, 8.0, 0.55), (6.0, 7.8, 0.55), (8.5, 8.0, 0.55),
            (11.0, 7.8, 0.55), (13.5, 8.0, 0.55), (16.0, 7.8, 0.55),
            (3.5, 4.0, 0.55), (6.0, 4.2, 0.55), (8.5, 4.0, 0.55),
            (11.0, 4.2, 0.55), (13.5, 4.0, 0.55), (16.0, 4.2, 0.55),
            (2.0, 3.0, 0.6), (2.0, 9.0, 0.6),
        ],
        global_path=[
            (0.5, 6.0), (2.5, 6.0), (5.0, 6.0), (7.5, 6.0),
            (10.0, 6.0), (12.5, 6.0), (15.0, 6.0), (17.0, 6.0),
        ],
        dyn_traj=[(5.0, 10.5), (5.0, 1.5)],
        dyn_r=0.4, dyn_spd=1.5, sim_time=20.0,
    ),
    # 场景4: 路径封堵 — 演示三阶段恢复状态机 (停车→旋转→后退→脱困)
    "blocked": dict(
        name="卡死恢复演示",
        desc="静态障碍封堵路径，触发停车→旋转→后退→脱困完整恢复流程",
        static_obs=[
            # 正面封堵墙 (x≈6)
            (6.2, 4.8, 0.7), (6.2, 5.8, 0.65), (6.2, 6.8, 0.7),
            # 两侧辅助障碍
            (3.5, 3.0, 0.6), (3.5, 9.0, 0.6),
            (10.0, 3.5, 0.7), (10.0, 8.5, 0.7),
            (14.0, 5.0, 0.6), (14.0, 7.5, 0.5),
        ],
        global_path=[
            (0.5, 6.0), (2.5, 6.0), (4.5, 6.0), (7.5, 6.0),
            (10.0, 6.0), (12.5, 6.0), (15.0, 6.0), (17.0, 6.0),
        ],
        dyn_traj=[(15.0, 1.0), (15.0, 11.0)],
        dyn_r=0.4, dyn_spd=1.0, sim_time=28.0,
    ),
    # 场景5: 全局重规划 — 路径封堵 → pct_adapter stuck → 重新规划绕行
    "replan": dict(
        name="全局重规划演示",
        desc="路径封堵 → stuck 10s → pct_adapter 触发 PCT 重规划 → 绕行到达",
        static_obs=[
            # 正面封堵墙 (x≈6, y=4.5~7.5)
            (6.2, 4.5, 0.7), (6.2, 5.6, 0.65), (6.2, 6.7, 0.7), (6.2, 7.8, 0.65),
            # 散落障碍
            (3.5, 2.5, 0.6), (3.5, 9.5, 0.6),
            (10.0, 3.5, 0.7), (10.0, 8.5, 0.7),
            (14.0, 5.5, 0.5), (14.0, 7.0, 0.5),
        ],
        global_path=[
            (0.5, 6.0), (2.5, 6.0), (4.5, 6.0), (7.5, 6.0),
            (10.0, 6.0), (12.5, 6.0), (15.0, 6.0), (17.0, 6.0),
        ],
        # 绕行路径: stuck 后 PCT 重规划生成的新全局路径 (从 x≈5 绕行南侧)
        detour_path=[
            (5.0, 6.0),   # 接近当前位置 (由 _try_replan 动态替换前缀)
            (5.5, 4.0), (6.0, 2.5), (7.5, 2.0), (9.0, 2.5),
            (10.5, 3.5), (12.0, 5.0), (13.5, 6.0),
            (15.0, 6.0), (17.0, 6.0),
        ],
        dyn_traj=[(15.0, 1.0), (15.0, 11.0)],
        dyn_r=0.35, dyn_spd=0.8, sim_time=40.0,
        replan_stuck_sec=10.0,  # pct_adapter stuck 阈值
        max_replan=2,           # 最多重规划次数
    ),
    # 场景3: 对向动态障碍 — 迎面驶来，机器人侧移让道
    "headon": dict(
        name="对向障碍物迎面",
        desc="动态障碍从正前方迎面驶来，机器人必须侧偏让道后回归路径",
        static_obs=[
            (4.0, 2.5, 0.7), (4.0, 9.5, 0.7),
            (9.0, 2.0, 0.8), (9.0, 10.0, 0.8),
            (14.0, 2.5, 0.6), (14.0, 9.5, 0.6),
        ],
        global_path=[
            (0.5, 6.0), (2.5, 6.0), (5.0, 6.0), (7.5, 6.0),
            (10.0, 6.0), (12.5, 6.0), (15.0, 6.0), (17.0, 6.0),
        ],
        dyn_traj=[(16.5, 6.0), (1.0, 6.0)],
        dyn_r=0.65, dyn_spd=1.6, sim_time=20.0,
    ),
}


def _apply_scene(name: str):
    """将场景参数写入全局变量，返回 (场景名, 描述, 场景配置dict)"""
    global STATIC_OBS, GLOBAL_PATH, DYN_OBS_TRAJ, DYN_OBS_R, DYN_OBS_SPD, SIM_TIME
    s = SCENES[name]
    STATIC_OBS   = s['static_obs']
    GLOBAL_PATH  = s['global_path']
    DYN_OBS_TRAJ = s['dyn_traj']
    DYN_OBS_R    = s['dyn_r']
    DYN_OBS_SPD  = s['dyn_spd']
    SIM_TIME     = s['sim_time']
    return s['name'], s['desc'], s

# ═══════════════════════════════════════════════════════════════════════════
# 候选路径生成 (对应 localPlanner readPaths / startPaths)
# ═══════════════════════════════════════════════════════════════════════════

def _make_candidate_paths():
    """生成 body-frame 候选路径 (扇形，对应 localPlanner 的 pathNum_ 条路径)"""
    paths = []
    angles = np.linspace(-math.radians(80), math.radians(80), N_PATHS)
    for a in angles:
        pts = []
        for k in range(1, PATH_LEN + 1):
            # 弧形：沿 a 方向弯曲
            arc = a * 0.45
            x = PATH_STEP * k * math.cos(arc * k / PATH_LEN)
            y = PATH_STEP * k * math.sin(arc * k / PATH_LEN)
            pts.append((x, y))
        paths.append(pts)
    return paths

CANDIDATE_BODY = _make_candidate_paths()


def _body_to_world(pts_body, rx, ry, ryaw):
    cy, sy = math.cos(ryaw), math.sin(ryaw)
    return [(rx + cy * px - sy * py, ry + sy * px + cy * py)
            for px, py in pts_body]


def _score_path(pts_world, obstacles, goal_dir, robot_xy, margin_scale=1.0):
    """
    路径评分 (简化版 localPlanner 评分):
      - 任意点 < 安全距离 → 碰撞 (-1)
      - obs_score: 最近障碍物距离
      - dir_score: 路径末端方向与目标方向对齐程度
    """
    rx, ry = robot_xy
    min_d = float('inf')
    half_w = (VEHICLE_W / 2 + 0.08) * margin_scale   # 安全裕量（可缩放）

    for px, py in pts_world:
        for ox, oy, oр in obstacles:
            d = math.hypot(px - ox, py - oy) - oр
            if d < half_w:
                return -1.0          # 碰撞
            min_d = min(min_d, d)

    if min_d == float('inf'):
        min_d = ADJACENT_RANGE

    obs_score = min(min_d / (ADJACENT_RANGE * 0.5), 1.0)

    # 路径末端方向与目标方向
    ex, ey = pts_world[-1]
    path_dir = math.atan2(ey - ry, ex - rx)
    diff = abs(math.atan2(math.sin(goal_dir - path_dir),
                           math.cos(goal_dir - path_dir)))
    dir_score = max(0.0, 1.0 - diff / math.pi * 1.8)

    return 0.55 * obs_score + 0.45 * dir_score


# ═══════════════════════════════════════════════════════════════════════════
# 机器人状态 & 规划器
# ═══════════════════════════════════════════════════════════════════════════

class Robot:
    def __init__(self, scene_cfg=None):
        self.x, self.y, self.yaw = ROBOT_START
        self.speed  = 0.0
        self.wyaw   = 0.0          # yaw rate
        self.wp_idx = 1            # PCT adapter 当前航点索引
        self.scored_paths = []     # [(pts_world, score), ...]
        self.best_path    = []     # 选中的局部路径
        self.mode = "FOLLOWING"
        self.reached = False
        self.trail = [(self.x, self.y)]
        # ── 恢复状态机 (对应 localPlanner.cpp 三阶段逻辑) ───────────────
        self._blocked_start  = None  # 首次阻塞时刻 (仿真时间 t)
        self._recovery_state = 0     # 0=normal/等待, 1=rotating, 2=backing_up
        self._recovery_start = None  # 当前恢复阶段开始时刻
        self._recovery_cycles = 0    # 恢复循环计数
        self._recovery_max   = 3     # 最大恢复循环次数
        # ── pct_adapter 重规划状态 (对应 pct_path_adapter.cpp 修改) ──────
        self._scene_cfg = scene_cfg or {}
        self._stuck_start = None     # pct_adapter stuck 开始时刻
        self._replan_count = 0       # 已重规划次数
        self._replan_max = self._scene_cfg.get('max_replan', 0)
        self._replan_stuck_sec = self._scene_cfg.get('replan_stuck_sec', 10.0)
        self._replan_event = None    # 'replanning' | 'stuck_final' | None
        self._replan_countdown = 0.0 # 重规划倒计时显示
        self.global_path = list(GLOBAL_PATH)  # 可动态更新

    # ── 动态障碍物位置 ────────────────────────────────────────────────────

    @staticmethod
    def dyn_obs_at(t):
        """返回动态障碍物 (x, y) — 线性插值"""
        x0, y0 = DYN_OBS_TRAJ[0]
        x1, y1 = DYN_OBS_TRAJ[1]
        total_dist = math.hypot(x1 - x0, y1 - y0)
        dist = DYN_OBS_SPD * t
        f = min(dist / total_dist, 1.0)
        return x0 + f * (x1 - x0), y0 + f * (y1 - y0)

    def _all_obstacles(self, t):
        dx, dy = self.dyn_obs_at(t)
        return list(STATIC_OBS) + [(dx, dy, DYN_OBS_R)]

    def _nearby_obstacles(self, all_obs):
        """仅返回 ADJACENT_RANGE 内的障碍物"""
        return [(ox, oy, oр) for ox, oy, oр in all_obs
                if math.hypot(ox - self.x, oy - self.y) < ADJACENT_RANGE + oр + 0.5]

    # ── PCT Adapter: 航点推进 ──────────────────────────────────────────────

    def _update_waypoint(self):
        if self.wp_idx >= len(self.global_path):
            return
        # 限窗搜索最近航点 (+5): 防止 U 形路径跳到对岸
        search_end = min(self.wp_idx + 5, len(self.global_path))
        best_idx = self.wp_idx
        best_dist = float('inf')
        for i in range(self.wp_idx, search_end):
            d = math.hypot(self.global_path[i][0] - self.x,
                           self.global_path[i][1] - self.y)
            if d < best_dist:
                best_dist = d
                best_idx = i
        if best_idx > self.wp_idx:
            self.wp_idx = best_idx
        # 正常到达推进
        gx, gy = self.global_path[self.wp_idx]
        if math.hypot(gx - self.x, gy - self.y) < WAYPOINT_THRE:
            self.wp_idx = min(self.wp_idx + 1, len(self.global_path) - 1)

    # ── localPlanner: 候选路径评分 ────────────────────────────────────────

    def _local_plan(self, all_obs, t):
        nearby = self._nearby_obstacles(all_obs)

        # 目标方向 (当前全局航点)
        gx, gy = self.global_path[min(self.wp_idx, len(self.global_path) - 1)]
        goal_dir = math.atan2(gy - self.y, gx - self.x)

        scored = []
        for pts_body in CANDIDATE_BODY:
            pts_w = _body_to_world(pts_body, self.x, self.y, self.yaw)
            s = _score_path(pts_w, nearby, goal_dir, (self.x, self.y))
            scored.append((pts_w, s))

        self.scored_paths = scored

        # 选最高分路径
        best = max(scored, key=lambda x: x[1])

        if best[1] > 0:
            # ── 有可通行路径：重置恢复状态 ────────────────────────────────
            self.best_path = best[0]
            self._blocked_start   = None
            self._recovery_state  = 0
            self._recovery_cycles = 0
            self.mode = ("AVOIDING"
                         if any(math.hypot(ox - self.x, oy - self.y) < ADJACENT_RANGE
                                for ox, oy, _ in all_obs)
                         else "FOLLOWING")
        else:
            # ── pathScale 收缩循环 (对应 C++ pathScale_ -= pathScaleStep_) ─
            found = False
            for scale in [0.7, 0.5, 0.35]:
                relaxed = []
                for pts_body in CANDIDATE_BODY:
                    pts_w = _body_to_world(pts_body, self.x, self.y, self.yaw)
                    s = _score_path(pts_w, nearby, goal_dir, (self.x, self.y),
                                    margin_scale=scale)
                    relaxed.append((pts_w, s))
                best_r = max(relaxed, key=lambda x: x[1])
                if best_r[1] > 0:
                    self.scored_paths = relaxed
                    self.best_path = best_r[0]
                    self._blocked_start   = None
                    self._recovery_state  = 0
                    self._recovery_cycles = 0
                    found = True
                    self.mode = "AVOIDING"
                    break

            if not found:
                # ── 三阶段恢复状态机 (对应 C++ !pathFound 新逻辑) ──────────
                if self._blocked_start is None:
                    self._blocked_start = t
                blocked_dur = t - self._blocked_start

                # 阶段切换
                BLOCKED_THRE = 2.0
                ROTATE_TIME  = 2.5
                BACKUP_TIME  = 1.5
                if (self._recovery_state == 0 and blocked_dur >= BLOCKED_THRE
                        and self._recovery_cycles < self._recovery_max):
                    self._recovery_state = 1
                    self._recovery_start = t
                elif (self._recovery_state == 1 and
                      self._recovery_start is not None and
                      t - self._recovery_start >= ROTATE_TIME):
                    self._recovery_state = 2
                    self._recovery_start = t
                elif (self._recovery_state == 2 and
                      self._recovery_start is not None and
                      t - self._recovery_start >= BACKUP_TIME):
                    self._recovery_state  = 0
                    self._blocked_start   = t  # 重新计时
                    self._recovery_cycles += 1

                # 生成恢复路径 (body → world)
                if self._recovery_state == 1:
                    # 旋转：弧形路径偏向目标方向
                    rel_angle = goal_dir - self.yaw
                    rel_angle = math.atan2(math.sin(rel_angle), math.cos(rel_angle))
                    turn_dir  = 1.0 if rel_angle >= 0.0 else -1.0
                    pts_body  = [(0.15 * i * math.cos(turn_dir * i * 0.25),
                                  0.15 * i * math.sin(turn_dir * i * 0.25))
                                 for i in range(1, 7)]
                    self.best_path = _body_to_world(pts_body, self.x, self.y, self.yaw)
                    self.mode = "ROTATING"
                elif self._recovery_state == 2:
                    # 后退：body -X 方向
                    pts_body = [(-0.2 * i, 0.0) for i in range(1, 6)]
                    self.best_path = _body_to_world(pts_body, self.x, self.y, self.yaw)
                    self.mode = "BACKING_UP"
                else:
                    # 等待计时：零点路径 → 停车
                    self.best_path = []
                    self.mode = "BLOCKED"

    # ── pct_adapter: stuck 检测 + 全局重规划 ──────────────────────────────

    def _check_stuck_replan(self, t):
        """模拟 pct_adapter stuck 检测 + 自动重发 goal_pose 触发 PCT 重规划"""
        if self._replan_max <= 0:
            return  # 场景未启用重规划

        self._replan_event = None
        is_stuck = self.mode in ("BLOCKED", "ROTATING", "BACKING_UP")

        if is_stuck:
            if self._stuck_start is None:
                self._stuck_start = t
            stuck_dur = t - self._stuck_start
            self._replan_countdown = max(0.0, self._replan_stuck_sec - stuck_dur)

            if stuck_dur >= self._replan_stuck_sec:
                if self._replan_count < self._replan_max:
                    self._try_replan(t)
                else:
                    self._replan_event = "stuck_final"
        else:
            self._stuck_start = None
            self._replan_countdown = 0.0

    def _try_replan(self, t):
        """触发重规划: 用 detour_path 替换全局路径 (模拟 PCT Planner 重新规划)"""
        detour = self._scene_cfg.get('detour_path')
        if not detour:
            return

        self._replan_count += 1
        self._stuck_start = None  # 重置 stuck 计时
        self._replan_event = "replanning"

        # 从当前位置构造新路径: 当前位置 + detour 后续点
        new_path = [(self.x, self.y)]
        for px, py in detour:
            if px > self.x - 0.5:  # 只取在机器人前方的点
                new_path.append((px, py))
        # 确保终点一致
        if new_path[-1] != detour[-1]:
            new_path.append(detour[-1])

        self.global_path = new_path
        self.wp_idx = 1  # 重置航点到新路径第2个点

        # 重置恢复状态 (收到新路径 → 重新开始)
        self._blocked_start   = None
        self._recovery_state  = 0
        self._recovery_cycles = 0

    # ── pathFollower: Pure Pursuit ────────────────────────────────────────

    def _follow(self):
        # ── 后退阶段：直接用负速度，不走 Pure Pursuit ──────────────────
        if self.mode == "BACKING_UP" and self.best_path:
            target = -MAX_SPEED * 0.35          # 后退目标速度
            step = MAX_ACCEL * DT
            self.speed = (self.speed - step if self.speed > target
                          else max(self.speed - step, target))
            self.wyaw = 0.0
            return

        if not self.best_path:
            # ── 对应真实 pathFollower 收到 path_size=1 时的行为 ──────────
            # C++ 行为: joy_speed2 = 0.0，机器人制动停车，不做任何转向
            self.speed = max(self.speed - MAX_ACCEL * DT * 2, 0.0)
            self.wyaw  = 0.0
            return

        # 终点距离
        ex, ey = self.best_path[-1]
        end_dis = math.hypot(ex - self.x, ey - self.y)

        # 前视点
        look = max(LOOK_AHEAD, 0.3 + 0.35 * abs(self.speed))
        look_sq = look * look
        fdx, fdy = ex - self.x, ey - self.y
        for px, py in self.best_path:
            dx, dy = px - self.x, py - self.y
            if dx * dx + dy * dy >= look_sq:
                fdx, fdy = dx, dy
                break

        # 方向差 → yaw rate
        path_dir = math.atan2(fdy, fdx)
        dir_diff = math.atan2(math.sin(path_dir - self.yaw),
                               math.cos(path_dir - self.yaw))
        self.wyaw = YAW_RATE_GAIN * dir_diff
        max_wr = MAX_YAW_RATE * math.pi / 180
        self.wyaw = max(-max_wr, min(max_wr, self.wyaw))

        # 目标速度
        target = MAX_SPEED
        if end_dis < SLOWDOWN_DIS:
            target = MAX_SPEED * (end_dis / SLOWDOWN_DIS)
        if end_dis < STOP_DIS:
            target = 0.0
        # 大转角减速
        if abs(dir_diff) > 0.4:
            target *= max(0.25, 1.0 - abs(dir_diff) / math.pi)

        step = MAX_ACCEL * DT
        self.speed = (min(self.speed + step, target) if self.speed < target
                      else max(self.speed - step, target))

    # ── 仿真步进 ──────────────────────────────────────────────────────────

    def step(self, t):
        if self.reached:
            return
        all_obs = self._all_obstacles(t)
        self._update_waypoint()
        self._local_plan(all_obs, t)
        self._check_stuck_replan(t)  # pct_adapter stuck → 触发全局重规划
        self._follow()

        # 运动学积分
        self.x   += self.speed * math.cos(self.yaw) * DT
        self.y   += self.speed * math.sin(self.yaw) * DT
        self.yaw += self.wyaw * DT
        self.trail.append((self.x, self.y))

        # 到达终点
        gx, gy = self.global_path[-1]
        if math.hypot(gx - self.x, gy - self.y) < STOP_DIS:
            self.reached = True
            self.speed   = 0.0


# ═══════════════════════════════════════════════════════════════════════════
# 可视化
# ═══════════════════════════════════════════════════════════════════════════

def _robot_polygon(rx, ry, ryaw):
    """机器人轮廓 (矩形朝向箭头)"""
    L, W = VEHICLE_L, VEHICLE_W
    corners = [(-L/2, -W/2), (L/2, -W/2), (L/2, W/2), (-L/2, W/2)]
    cy, sy = math.cos(ryaw), math.sin(ryaw)
    return [(rx + cy*x - sy*y, ry + sy*x + cy*y) for x, y in corners]


def make_figure():
    fig = plt.figure(figsize=(15, 9), facecolor='#0d1117')
    ax  = fig.add_axes([0.01, 0.08, 0.72, 0.88])
    ax_info = fig.add_axes([0.74, 0.08, 0.24, 0.88])

    ax.set_facecolor('#161b22')
    ax_info.set_facecolor('#0d1117')
    ax_info.axis('off')

    ax.set_xlim(-0.5, WORLD_W + 0.5)
    ax.set_ylim(-0.5, WORLD_H + 0.5)
    ax.set_aspect('equal')
    ax.tick_params(colors='#58a6ff', labelsize=7)
    ax.spines[:].set_color('#30363d')
    ax.set_xlabel('X (m)', color='#8b949e', fontsize=9)
    ax.set_ylabel('Y (m)', color='#8b949e', fontsize=9)

    # 网格
    ax.grid(True, color='#21262d', linewidth=0.5, zorder=0)

    return fig, ax, ax_info


def build_artists(ax, ax_info):
    """预创建所有 matplotlib artist"""
    artists = {}

    # ── 全局路径 ──────────────────────────────────────────────────────────
    gx = [p[0] for p in GLOBAL_PATH]
    gy = [p[1] for p in GLOBAL_PATH]
    artists['global_path'], = ax.plot(gx, gy, '--', color='#1f6feb',
                                       linewidth=1.8, alpha=0.6,
                                       zorder=2, label='Global Path')
    artists['global_pts'] = ax.scatter(gx, gy, color='#388bfd',
                                        s=25, zorder=3, marker='o')

    # ── 静态障碍物 ────────────────────────────────────────────────────────
    for ox, oy, ор in STATIC_OBS:
        c = plt.Circle((ox, oy), ор, color='#30363d',
                        ec='#6e7681', linewidth=1.2, zorder=4)
        ax.add_patch(c)
        ax.text(ox, oy, '■', ha='center', va='center',
                fontsize=8, color='#6e7681', zorder=5)

    # ── 动态障碍物 ────────────────────────────────────────────────────────
    artists['dyn_obs'] = plt.Circle((0, 0), DYN_OBS_R,
                                     color='#ff6e6e', alpha=0.85,
                                     ec='#ff4444', linewidth=2, zorder=10)
    ax.add_patch(artists['dyn_obs'])
    artists['dyn_label'] = ax.text(0, 0, '!!', ha='center', va='center',
                                    fontsize=12, color='white', zorder=11,
                                    fontweight='bold')

    # ── 检测范围圆 ────────────────────────────────────────────────────────
    artists['detect_zone'] = plt.Circle((0, 0), ADJACENT_RANGE,
                                         fill=False, color='#1f6feb',
                                         linewidth=0.8, alpha=0.25,
                                         linestyle='--', zorder=3)
    ax.add_patch(artists['detect_zone'])

    # ── 候选路径 (LineCollection) ─────────────────────────────────────────
    dummy_segs = [[(0, 0), (0, 0)]] * N_PATHS
    artists['cand_lc'] = LineCollection(dummy_segs, linewidths=0.7,
                                         alpha=0.55, zorder=5)
    ax.add_collection(artists['cand_lc'])

    # ── 选中局部路径 ──────────────────────────────────────────────────────
    artists['best_path'], = ax.plot([], [], '-', color='#3fb950',
                                     linewidth=2.8, zorder=8,
                                     label='Local Path')

    # ── 当前航点 ──────────────────────────────────────────────────────────
    artists['waypoint'] = ax.scatter([], [], color='#ffd700',
                                      s=120, marker='*', zorder=9,
                                      label='Waypoint', edgecolors='#b8860b',
                                      linewidths=0.8)

    # ── 机器人轨迹 ────────────────────────────────────────────────────────
    artists['trail'], = ax.plot([], [], '-', color='#79c0ff',
                                 linewidth=1.2, alpha=0.5, zorder=6)

    # ── 机器人本体 ────────────────────────────────────────────────────────
    dummy_poly = plt.Polygon([(0,0)]*4, closed=True,
                              facecolor='#58a6ff', edgecolor='#cae8ff',
                              linewidth=1.5, zorder=12)
    ax.add_patch(dummy_poly)
    artists['robot_body'] = dummy_poly

    # 前进方向箭头
    artists['robot_arrow'] = ax.annotate('', xy=(0, 0), xytext=(0, 0),
        arrowprops=dict(arrowstyle='->', color='#cae8ff', lw=2),
        zorder=13)

    # 速度箭头
    artists['vel_arrow'] = ax.annotate('', xy=(0, 0), xytext=(0, 0),
        arrowprops=dict(arrowstyle='->', color='#f8c555',
                        lw=2.5, mutation_scale=18),
        zorder=13)

    # ── 终点标记 ──────────────────────────────────────────────────────────
    gx, gy = GLOBAL_PATH[-1]
    ax.scatter([gx], [gy], s=200, marker='*', color='#ffd700',
               edgecolors='#b8860b', linewidths=1.5, zorder=15)
    ax.text(gx + 0.2, gy + 0.3, 'GOAL', color='#ffd700',
            fontsize=9, fontweight='bold', zorder=15)

    # 起点
    sx, sy = GLOBAL_PATH[0]
    ax.text(sx - 0.1, sy + 0.3, 'START', color='#79c0ff',
            fontsize=9, fontweight='bold', zorder=15)

    # ── 图例 ─────────────────────────────────────────────────────────────
    legend_elems = [
        mpatches.Patch(color='#1f6feb', alpha=0.6, label='Global Path (PCT)'),
        mpatches.Patch(color='#3fb950', label='Local Path (selected)'),
        mpatches.Patch(color='#ffd700', alpha=0.8, label='Waypoint (adapter)'),
        mpatches.Patch(color='#58a6ff', label='Robot'),
        mpatches.Patch(color='#ff6e6e', label='Dynamic Obstacle'),
        mpatches.Patch(color='#30363d', ec='#6e7681', label='Static Obstacle'),
    ]
    ax.legend(handles=legend_elems, loc='lower right',
              facecolor='#161b22', edgecolor='#30363d',
              labelcolor='#c9d1d9', fontsize=7.5, framealpha=0.9)

    # ── 信息面板 ──────────────────────────────────────────────────────────
    artists['info_title'] = ax_info.text(0.5, 0.97, 'MapPilot Planner',
        ha='center', va='top', transform=ax_info.transAxes,
        color='#58a6ff', fontsize=13, fontweight='bold')

    artists['info_subtitle'] = ax_info.text(0.5, 0.91, 'Local Navigation Demo',
        ha='center', va='top', transform=ax_info.transAxes,
        color='#8b949e', fontsize=9)

    # 分隔线
    ax_info.axhline(0.88, color='#30363d', linewidth=1)

    def _label(y, key, txt):
        ax_info.text(0.05, y, txt, va='top', transform=ax_info.transAxes,
                     color='#8b949e', fontsize=8.5)
        artists[key] = ax_info.text(0.95, y, '—', va='top', ha='right',
                                     transform=ax_info.transAxes,
                                     color='#c9d1d9', fontsize=9,
                                     fontweight='bold')

    _label(0.84, 'i_time',   'Time')
    _label(0.78, 'i_mode',   'Planner Mode')
    _label(0.72, 'i_speed',  'Speed')
    _label(0.66, 'i_yaw',    'Yaw Rate')
    _label(0.60, 'i_wayp',   'Waypoint')
    _label(0.54, 'i_obs',    'Nearby Obstacles')
    _label(0.48, 'i_best',   'Best Path Score')

    ax_info.axhline(0.44, color='#30363d', linewidth=1)

    # 候选路径分数直方图（简易）
    artists['bar_bg'] = ax_info.text(0.5, 0.40, 'Candidate Path Scores',
        ha='center', va='top', transform=ax_info.transAxes,
        color='#8b949e', fontsize=8.5)

    artists['score_bars'] = []
    bar_w = 0.9 / N_PATHS
    for i in range(N_PATHS):
        bar = mpatches.FancyBboxPatch(
            (0.05 + i * bar_w, 0.25), bar_w * 0.85, 0.0,
            boxstyle='square,pad=0', transform=ax_info.transAxes,
            color='#238636', zorder=5)
        ax_info.add_patch(bar)
        artists['score_bars'].append(bar)

    ax_info.axhline(0.22, color='#30363d', linewidth=1)

    # 状态消息
    artists['status_msg'] = ax_info.text(0.5, 0.18,
        '● Initializing…',
        ha='center', va='top', transform=ax_info.transAxes,
        color='#3fb950', fontsize=9.5, fontweight='bold')

    artists['status_detail'] = ax_info.text(0.5, 0.12,
        '',
        ha='center', va='top', transform=ax_info.transAxes,
        color='#8b949e', fontsize=8, wrap=True)

    # 脚注
    ax_info.text(0.5, 0.03,
        'Simulates localPlanner + pathFollower\nC++ nodes (no ROS2 required)',
        ha='center', va='bottom', transform=ax_info.transAxes,
        color='#484f58', fontsize=7.5)

    return artists


# ═══════════════════════════════════════════════════════════════════════════
# 动画更新
# ═══════════════════════════════════════════════════════════════════════════

def _colormap_score(s):
    """得分 → 颜色: 红(-1/0) → 黄(0.5) → 绿(1)"""
    if s < 0:
        return '#ff4444'
    r = max(0.0, 1.0 - s * 1.5)
    g = min(1.0, s * 1.5)
    return (r, g, 0.1)


def make_animation(save_path=None, scene_name=None, scene_cfg=None):
    robot  = Robot(scene_cfg=scene_cfg)
    n_frames = int(SIM_TIME * FPS)

    if scene_name is None:
        scene_name = '局部避障规划演示'

    fig, ax, ax_info = make_figure()
    arts = build_artists(ax, ax_info)

    title_obj = ax.set_title(f'MapPilot — {scene_name}  t=0.00s',
                               color='#c9d1d9', fontsize=11, pad=6)

    def update(frame):
        t = frame * DT
        robot.step(t)

        # ── 动态障碍物 ────────────────────────────────────────────────────
        dx, dy = robot.dyn_obs_at(t)
        arts['dyn_obs'].set_center((dx, dy))
        arts['dyn_label'].set_position((dx, dy))

        # ── 检测范围 ──────────────────────────────────────────────────────
        arts['detect_zone'].set_center((robot.x, robot.y))

        # ── 候选路径着色 ──────────────────────────────────────────────────
        segs   = []
        colors = []
        for pts, score in robot.scored_paths:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            segs.append(list(zip(xs, ys)))
            colors.append(_colormap_score(score))
        arts['cand_lc'].set_segments(segs)
        arts['cand_lc'].set_colors(colors)

        # ── 选中局部路径 ──────────────────────────────────────────────────
        if robot.best_path:
            bx = [p[0] for p in robot.best_path]
            by = [p[1] for p in robot.best_path]
            arts['best_path'].set_data(bx, by)
        else:
            arts['best_path'].set_data([], [])

        # ── 全局路径动态更新 (重规划后路径改变) ─────────────────────────
        gpath = robot.global_path
        gpx = [p[0] for p in gpath]
        gpy = [p[1] for p in gpath]
        arts['global_path'].set_data(gpx, gpy)
        arts['global_pts'].set_offsets(list(zip(gpx, gpy)))
        # 重规划后路径颜色变化: 蓝 → 青色
        if robot._replan_count > 0:
            arts['global_path'].set_color('#00ccff')
            arts['global_pts'].set_facecolor('#00ccff')

        # ── 当前航点 ──────────────────────────────────────────────────────
        wi = min(robot.wp_idx, len(gpath) - 1)
        arts['waypoint'].set_offsets([[gpath[wi][0], gpath[wi][1]]])

        # ── 机器人轨迹 ────────────────────────────────────────────────────
        if len(robot.trail) > 1:
            tx, ty = zip(*robot.trail)
            arts['trail'].set_data(tx, ty)

        # ── 机器人本体 ────────────────────────────────────────────────────
        poly_pts = _robot_polygon(robot.x, robot.y, robot.yaw)
        arts['robot_body'].set_xy(poly_pts)

        # 前进方向箭头
        arrow_len = VEHICLE_L * 0.65
        ax_tip = robot.x + arrow_len * math.cos(robot.yaw)
        ay_tip = robot.y + arrow_len * math.sin(robot.yaw)
        arts['robot_arrow'].set_position((robot.x, robot.y))
        arts['robot_arrow'].xy = (ax_tip, ay_tip)
        arts['robot_arrow'].xytext = (robot.x, robot.y)

        # 速度矢量箭头
        spd_scale = 0.7
        vx_tip = robot.x + robot.speed * math.cos(robot.yaw) * spd_scale
        vy_tip = robot.y + robot.speed * math.sin(robot.yaw) * spd_scale
        arts['vel_arrow'].xy = (vx_tip, vy_tip)
        arts['vel_arrow'].xytext = (robot.x, robot.y)

        # ── 信息面板更新 ──────────────────────────────────────────────────
        arts['i_time'].set_text(f'{t:.1f} s')
        _mc = {'AVOIDING':'#ff6e6e','BLOCKED':'#ff9900',
               'ROTATING':'#ff7700','BACKING_UP':'#ff4400'}
        arts['i_mode'].set_text(robot.mode)
        arts['i_mode'].set_color(_mc.get(robot.mode, '#3fb950'))
        arts['i_speed'].set_text(f'{robot.speed * 100:.0f} cm/s')
        arts['i_yaw'].set_text(f'{math.degrees(robot.wyaw):.1f} °/s')
        arts['i_wayp'].set_text(f'{wi + 1} / {len(gpath)}')

        # 附近障碍物数量
        all_obs = robot._all_obstacles(t)
        n_nearby = sum(1 for ox, oy, _ in all_obs
                       if math.hypot(ox - robot.x, oy - robot.y) < ADJACENT_RANGE)
        arts['i_obs'].set_text(str(n_nearby))
        arts['i_obs'].set_color('#ff6e6e' if n_nearby > 0 else '#3fb950')

        # 最优路径分数
        if robot.scored_paths:
            best_s = max(s for _, s in robot.scored_paths)
            score_txt = f'{best_s:.2f}' if best_s >= 0 else 'BLOCKED'
            arts['i_best'].set_text(score_txt)
            arts['i_best'].set_color('#3fb950' if best_s > 0.3 else '#ff6e6e')

        # 候选路径分数直方图
        if robot.scored_paths:
            max_bar_h = 0.12
            bar_w = 0.9 / N_PATHS
            for i, (_, s) in enumerate(robot.scored_paths):
                h = max(0.002, min(max_bar_h, (max(s, 0) * max_bar_h)))
                c = _colormap_score(s)
                arts['score_bars'][i].set_height(h)
                arts['score_bars'][i].set_facecolor(c)

        # 状态消息
        if robot.reached:
            arts['status_msg'].set_text('● 到达目标！')
            arts['status_msg'].set_color('#ffd700')
            replan_note = (f'\n(经 {robot._replan_count} 次全局重规划)'
                           if robot._replan_count > 0 else '')
            arts['status_detail'].set_text(f'导航完成。\n机器人已停在目标点。{replan_note}')
        elif robot._replan_event == 'replanning':
            arts['status_msg'].set_text('[>>] 全局重规划！')
            arts['status_msg'].set_color('#00ccff')
            arts['status_detail'].set_text(
                f'pct_adapter stuck → 重发 goal_pose\n'
                f'PCT Planner 重新规划路径\n'
                f'重规划 {robot._replan_count}/{robot._replan_max}')
        elif robot._replan_event == 'stuck_final':
            arts['status_msg'].set_text('[!!] stuck_final 上报')
            arts['status_msg'].set_color('#ff0000')
            arts['status_detail'].set_text(
                f'几何层重规划 {robot._replan_max} 次仍失败\n'
                f'上报 stuck_final 事件\n'
                f'等待语义层 LERa 接管')
        elif robot.mode == 'BLOCKED':
            dur = (t - robot._blocked_start) if robot._blocked_start else 0
            replan_info = ''
            if robot._replan_max > 0 and robot._replan_countdown > 0:
                replan_info = f'\nstuck 倒计时: {robot._replan_countdown:.1f}s → 重规划'
            arts['status_msg'].set_text('[X] 路径受阻 → 等待恢复')
            arts['status_msg'].set_color('#ff9900')
            arts['status_detail'].set_text(
                f'pathScale 收缩仍无解\n'
                f'停车等待 {dur:.1f}s / 2.0s\n'
                f'即将进入旋转恢复阶段{replan_info}')
        elif robot.mode == 'ROTATING':
            replan_info = ''
            if robot._replan_max > 0 and robot._replan_countdown > 0:
                replan_info = f'\nstuck 倒计时: {robot._replan_countdown:.1f}s → 重规划'
            arts['status_msg'].set_text('[~] 恢复: 旋转转向')
            arts['status_msg'].set_color('#ff7700')
            arts['status_detail'].set_text(
                f'C++: recoveryState=1\n'
                f'发布弧形路径朝向目标\n'
                f'pathFollower 跟踪执行{replan_info}')
        elif robot.mode == 'BACKING_UP':
            replan_info = ''
            if robot._replan_max > 0 and robot._replan_countdown > 0:
                replan_info = f'\nstuck 倒计时: {robot._replan_countdown:.1f}s → 重规划'
            arts['status_msg'].set_text('[<<] 恢复: 后退')
            arts['status_msg'].set_color('#ff4400')
            arts['status_detail'].set_text(
                f'C++: recoveryState=2\n'
                f'发布 body -X 后退路径\n'
                f'twoWayDrive → 负速执行{replan_info}')
        elif robot.mode == 'AVOIDING':
            arts['status_msg'].set_text('[!] 障碍物规避')
            arts['status_msg'].set_color('#ff6e6e')
            arts['status_detail'].set_text(
                f'localPlanner: 从{N_PATHS}条候选路径\n'
                f'选出最优安全路径\n'
                f'Pure Pursuit 跟踪执行')
        else:
            arts['status_msg'].set_text('● 跟踪全局路径')
            arts['status_msg'].set_color('#3fb950')
            arts['status_detail'].set_text(
                f'pct_adapter 航点 → localPlanner\n'
                f'pathFollower Pure Pursuit\n'
                f'前视距={LOOK_AHEAD:.1f}m  偏航增益={YAW_RATE_GAIN}')

        # 标题
        mode_color = {
            'AVOIDING':   '#ff6e6e',
            'BLOCKED':    '#ff9900',
            'ROTATING':   '#ff7700',
            'BACKING_UP': '#ff4400',
        }.get(robot.mode, '#c9d1d9')
        arts['i_mode'].set_color(mode_color)
        title_obj.set_text(
            f'MapPilot — {scene_name}   t = {t:.2f} s  |  '
            f'速度: {robot.speed*100:.0f} cm/s  |  模式: {robot.mode}')
        title_obj.set_color(mode_color)

        return list(arts.values())

    anim = FuncAnimation(fig, update, frames=n_frames,
                          interval=1000 // FPS, blit=False, repeat=False)

    if save_path:
        print(f'保存动画到 {save_path} ...')
        writer = PillowWriter(fps=FPS)
        anim.save(save_path, writer=writer, dpi=110)
        print('保存完成')
    else:
        plt.show()

    return anim


# ═══════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MapPilot 规划避障动画演示')
    parser.add_argument('--save', metavar='FILE',
                        help='保存为 GIF/MP4 文件 (例: demo.gif)')
    parser.add_argument('--scenario', metavar='NAME',
                        choices=list(SCENES.keys()), default='default',
                        help=f'场景选择: {", ".join(SCENES.keys())} (默认: default)')
    parser.add_argument('--all', action='store_true',
                        help='批量生成所有场景 GIF (保存到 tools/)')
    args = parser.parse_args()

    if args.all:
        import os
        out_dir = os.path.dirname(os.path.abspath(__file__))
        for sname in SCENES.keys():
            scene_title, scene_desc, scene_cfg = _apply_scene(sname)
            out = os.path.join(out_dir, f'planner_demo_{sname}.gif')
            print(f'\n[{sname}] {scene_title} — {scene_desc}')
            make_animation(save_path=out, scene_name=scene_title, scene_cfg=scene_cfg)
    else:
        scene_title, scene_desc, scene_cfg = _apply_scene(args.scenario)
        print(f'场景: {scene_title} — {scene_desc}')
        make_animation(save_path=args.save, scene_name=scene_title, scene_cfg=scene_cfg)
