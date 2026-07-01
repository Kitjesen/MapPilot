"""
关键帧选择器 — 决定何时运行 YOLO 检测

参考:
  - OpenFly (2025): 轨迹曲率峰值 = 地标观测点
  - DualMap (RAL 2025): 平移 + 旋转 + 时间阈值
  - OnFly: 决策 Agent 高频 + 监测 Agent 低频

策略: 运动阈值 + 视觉变化 + 时间兜底 三合一
  - 移动 > 0.3m → 关键帧
  - 旋转 > 10° → 关键帧
  - 视觉变化 > 12% → 关键帧 (新物体出现)
  - 超过 3s 没更新 → 强制关键帧
  - 最快不超过 10Hz (BPU 有余量但不需要这么快)

效果: 相比固定 3Hz，节省 70% YOLO 调用
"""

import logging
import math
import time
from dataclasses import dataclass

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class KeyframeConfig:
    """关键帧选择参数"""
    min_translation: float = 0.3      # 平移阈值 (米)
    min_rotation: float = 10.0        # 旋转阈值 (度)
    max_interval: float = 3.0         # 最大更新间隔 (秒)
    min_interval: float = 0.1         # 最小更新间隔 (秒, 即最快 10Hz)
    visual_threshold: float = 0.12    # 视觉变化阈值 (像素差异比例)
    curvature_threshold: float = 0.5  # 轨迹曲率阈值 (OpenFly 风格)
    enable_visual: bool = True        # 启用视觉变化检测
    enable_curvature: bool = True     # 启用曲率检测


class KeyframeSelector:
    """
    关键帧选择器 — 根据运动/视觉/曲率决定是否需要新的检测。

    用法:
        selector = KeyframeSelector()
        for frame in camera_stream:
            if selector.is_keyframe(robot_x, robot_y, robot_yaw, frame_gray, timestamp):
                detections = yolo.detect(frame)  # 只在关键帧跑 YOLO
    """

    def __init__(self, config: KeyframeConfig = None):
        self._cfg = config or KeyframeConfig()
        self._last_pos: np.ndarray | None = None
        self._last_yaw: float = 0.0
        self._last_time: float = 0.0
        self._last_gray: np.ndarray | None = None
        self._keyframe_count: int = 0
        self._skip_count: int = 0
        self._total_count: int = 0

        # 曲率检测用的位置历史 (OpenFly 风格)
        self._pos_history: list = []
        self._max_history: int = 5

    def is_keyframe(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        frame_gray: np.ndarray | None = None,
        timestamp: float | None = None,
    ) -> bool:
        """
        判断当前帧是否为关键帧。

        Args:
            robot_x, robot_y: 机器人世界坐标 (米)
            robot_yaw: 机器人朝向 (弧度)
            frame_gray: 灰度图 (可选, 用于视觉变化检测)
            timestamp: 时间戳 (秒, None=自动取当前时间)

        Returns:
            True = 应该跑 YOLO 检测
        """
        if timestamp is None:
            timestamp = time.time()

        self._total_count += 1
        current_pos = np.array([robot_x, robot_y])

        # 第一帧始终是关键帧
        if self._last_pos is None:
            self._accept(current_pos, robot_yaw, frame_gray, timestamp)
            return True

        # 最小间隔保护 (不超过 10Hz)
        dt = timestamp - self._last_time
        if dt < self._cfg.min_interval:
            self._skip_count += 1
            return False

        # === 条件 1: 超时强制更新 ===
        if dt > self._cfg.max_interval:
            self._accept(current_pos, robot_yaw, frame_gray, timestamp)
            logger.debug("Keyframe: timeout (%.1fs)", dt)
            return True

        # === 条件 2: 平移距离 ===
        dist = float(np.linalg.norm(current_pos - self._last_pos))
        if dist > self._cfg.min_translation:
            self._accept(current_pos, robot_yaw, frame_gray, timestamp)
            logger.debug("Keyframe: translation %.2fm", dist)
            return True

        # === 条件 3: 旋转角度 ===
        angle_diff = abs(robot_yaw - self._last_yaw)
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
        if math.degrees(angle_diff) > self._cfg.min_rotation:
            self._accept(current_pos, robot_yaw, frame_gray, timestamp)
            logger.debug("Keyframe: rotation %.1f°", math.degrees(angle_diff))
            return True

        # === 条件 4: 轨迹曲率 (OpenFly 风格) ===
        if self._cfg.enable_curvature and len(self._pos_history) >= 3:
            curvature = self._compute_curvature()
            if curvature > self._cfg.curvature_threshold:
                self._accept(current_pos, robot_yaw, frame_gray, timestamp)
                logger.debug("Keyframe: curvature %.2f", curvature)
                return True

        # === 条件 5: 视觉变化 ===
        if (self._cfg.enable_visual
                and frame_gray is not None
                and self._last_gray is not None):
            change = self._compute_visual_change(frame_gray)
            if change > self._cfg.visual_threshold:
                self._accept(current_pos, robot_yaw, frame_gray, timestamp)
                logger.debug("Keyframe: visual change %.1f%%", change * 100)
                return True

        # 不是关键帧
        self._skip_count += 1
        # 更新位置历史 (用于曲率计算)
        self._pos_history.append(current_pos.copy())
        if len(self._pos_history) > self._max_history:
            self._pos_history.pop(0)

        return False

    def _accept(self, pos, yaw, gray, t):
        """接受为关键帧，更新状态。"""
        self._last_pos = pos.copy()
        self._last_yaw = yaw
        self._last_time = t
        if gray is not None:
            # 缩小存储 (节省内存)
            import cv2
            if gray.shape[0] > 120:
                self._last_gray = cv2.resize(gray, (160, 90))
            else:
                self._last_gray = gray.copy()
        self._keyframe_count += 1
        self._pos_history.append(pos.copy())
        if len(self._pos_history) > self._max_history:
            self._pos_history.pop(0)

    def _compute_curvature(self) -> float:
        """计算最近轨迹的曲率 (OpenFly: 转弯点 = 地标观测点)。"""
        if len(self._pos_history) < 3:
            return 0.0
        p0 = self._pos_history[-3]
        p1 = self._pos_history[-2]
        p2 = self._pos_history[-1]
        v1 = p1 - p0
        v2 = p2 - p1
        cross = float(np.cross(v1, v2))
        norm_v1 = float(np.linalg.norm(v1))
        if norm_v1 < 1e-6:
            return 0.0
        return abs(cross) / (norm_v1 ** 3 + 1e-6)

    def _compute_visual_change(self, current_gray: np.ndarray) -> float:
        """计算视觉变化比例。"""
        import cv2
        if current_gray.shape[0] > 120:
            small = cv2.resize(current_gray, (160, 90))
        else:
            small = current_gray
        if self._last_gray is None or self._last_gray.shape != small.shape:
            return 1.0
        diff = cv2.absdiff(small, self._last_gray)
        return float(np.mean(diff)) / 255.0

    @property
    def stats(self) -> dict:
        """统计信息。"""
        total = max(self._total_count, 1)
        return {
            "keyframes": self._keyframe_count,
            "skipped": self._skip_count,
            "total": self._total_count,
            "keyframe_rate": f"{self._keyframe_count / total:.1%}",
            "savings": f"{self._skip_count / total:.1%}",
        }

    def reset(self):
        """重置状态。"""
        self._last_pos = None
        self._last_yaw = 0.0
        self._last_time = 0.0
        self._last_gray = None
        self._keyframe_count = 0
        self._skip_count = 0
        self._total_count = 0
        self._pos_history.clear()
