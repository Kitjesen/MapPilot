"""BBox-based visual servoing — PD tracking of detected objects.

Adapted from DimOS DetectionNavigation (Apache 2.0 License).

Pipeline:
  1. Receive target bbox [x1,y1,x2,y2] from VLM or detector
  2. bbox center + depth map -> 3D world coordinates
  3. PD controller: angle error -> angular_z, distance error -> linear_x
  4. Continuous tracking: update bbox per frame -> update 3D -> update cmd_vel
  5. Arrival/loss determination

No DimOS framework dependency. Pure Python + numpy. Output TwistStamped-compatible dict.
"""

from __future__ import annotations

# Original: dimos/navigation/visual_servoing/detection_navigation.py
# Original: dimos/navigation/bbox_navigation.py
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import logging
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from core.msgs.numpy_compat import np
from core.config import get_config

logger = logging.getLogger(__name__)

# ── State constants ───────────────────────────────────────────────────────────
STATE_IDLE = "idle"
STATE_TRACKING = "tracking"
STATE_ARRIVED = "arrived"
STATE_LOST = "lost"

# Path where tuned gains are persisted across restarts
_GAINS_PERSIST_PATH = Path.home() / ".lingtu" / "bbox_navigator_gains.json"


class GainAutoTuner:
    """Online relay-based Ziegler-Nichols auto-tuner for BBoxNavigator PD gains.

    The tuner runs a closed-loop relay oscillation experiment on the yaw channel:
      1. Apply alternating ±relay_amplitude steps to the angular input.
      2. Observe the resulting yaw oscillation: period T_u and amplitude a_u.
      3. Estimate ultimate gain K_u = 4*d / (pi*a_u) (relay method formula).
      4. Compute ZN PD gains: Kp = 0.6*K_u, Kd = Kp*T_u/8.

    Usage:
        tuner = GainAutoTuner(relay_amplitude=0.3)
        report = tuner.run_on_synthetic(yaw_series, dt)  # for testing
        # In production: tuner.run_relay_experiment(duration=6.0, cmd_fn, measure_fn)
    """

    def __init__(self, relay_amplitude: float = 0.3) -> None:
        self.relay_amplitude = relay_amplitude

    def compute_zn_pd(
        self, T_u: float, a_u: float
    ) -> tuple[float, float, float, float]:
        """Compute ZN PD gains from relay experiment measurements.

        Args:
            T_u: ultimate period (seconds) — time between successive peaks.
            a_u: ultimate amplitude (radians) — half peak-to-peak yaw swing.

        Returns:
            (K_u, Kp, Kd, converged: float)
            converged is 1.0 if T_u and a_u are physically plausible, else 0.0.
        """
        if T_u <= 0 or a_u <= 0:
            return 0.0, 0.8, 0.0, 0.0  # fallback to DimOS defaults

        K_u = 4.0 * self.relay_amplitude / (math.pi * a_u)
        Kp = 0.6 * K_u
        Kd = Kp * T_u / 8.0
        converged = 1.0
        return K_u, Kp, Kd, converged

    def analyse_oscillation(
        self, yaw_series: list[float], dt: float
    ) -> tuple[float, float]:
        """Estimate oscillation period T_u and amplitude a_u from a yaw time series.

        Uses zero-crossing detection on the detrended signal to find peaks.

        Args:
            yaw_series: list of yaw measurements (radians), evenly spaced at dt seconds.
            dt: time step (seconds).

        Returns:
            (T_u, a_u) — (0.0, 0.0) if fewer than 2 full oscillations detected.
        """
        if len(yaw_series) < 4:
            return 0.0, 0.0

        arr = np.array(yaw_series, dtype=float)
        # Detrend: subtract mean
        arr -= arr.mean()

        # Detect zero-crossings (sign changes)
        signs = np.sign(arr)
        crossings = []
        for i in range(1, len(signs)):
            if signs[i - 1] != 0 and signs[i] != 0 and signs[i - 1] != signs[i]:
                crossings.append(i)

        if len(crossings) < 2:
            return 0.0, 0.0

        # Period = 2 * average half-period (time between consecutive zero-crossings)
        half_periods = [
            (crossings[i] - crossings[i - 1]) * dt
            for i in range(1, len(crossings))
        ]
        T_u = 2.0 * float(np.mean(half_periods))

        # Amplitude = half of peak-to-peak swing
        a_u = (float(arr.max()) - float(arr.min())) / 2.0

        return T_u, a_u


@dataclass
class BBoxNavConfig:
    """视觉伺服导航参数。所有默认值与 DimOS DetectionNavigation 一致。"""
    target_distance: float = 1.5      # 目标保持距离 (m)
    min_distance: float = 0.8         # 太近→后退距离 (m)
    max_linear_speed: float = 0.5     # 最大线速度 (m/s)
    max_angular_speed: float = 0.8    # 最大角速度 (rad/s)
    linear_gain: float = 0.8          # 线速度 P 增益
    angular_gain: float = 1.5         # 角速度 P 增益
    lost_timeout: float = 5.0         # 丢失超时 (s)
    arrived_threshold: float = 0.3    # 到达判定距离容差 (m)
    depth_roi_ratio: float = 0.2      # 深度取样区域占 bbox 面积比例 (边长比)
    servo_takeover_distance: float = 3.0  # 视觉伺服接管距离 (m), 大于此值用规划栈


class BBoxNavigator:
    """
    通用视觉伺服导航器 — 任何物体都能追。

    使用方式:
      1. nav = BBoxNavigator()
      2. 每帧调用 nav.update(bbox, depth_image, camera_intrinsics, robot_pose)
      3. 使用返回的 linear_x / angular_z 发布 TwistStamped

    或低级调用:
      3d = nav.compute_3d_from_bbox(...)
      (lx, az) = nav.compute_cmd_vel(3d, robot_pose)
    """

    def __init__(
        self,
        config: BBoxNavConfig | None = None,
        robot_id: str = "default",
        gains_path: Path | None = None,
    ) -> None:
        self._cfg = config if config is not None else BBoxNavConfig()
        self._robot_id = robot_id
        self._gains_path = gains_path or _GAINS_PERSIST_PATH
        self._state: str = STATE_IDLE
        self._last_bbox_time: float = 0.0
        # Timestamp of the last successful bbox→3D back-projection. Diverges
        # from _last_bbox_time when depth fails (NaN, IQR rejects, empty
        # mask) — used to trigger LOST on sustained depth failure even while
        # bbox detections keep arriving.
        self._last_valid_3d_time: float = 0.0
        self._depth_fail_streak: int = 0
        self._target_bbox: list | None = None
        self._target_3d: np.ndarray | None = None   # [x, y, z] world frame
        self._lock = threading.Lock()
        # Last depth confidence from compute_3d_from_bbox (1.0 = high trust)
        self.depth_confidence: float = 0.0
        # Camera→body transform from factory calibration
        cam_cfg = get_config().camera
        T_camera_body = cam_cfg.T_camera_body
        if isinstance(T_camera_body, list):
            self._R_body_camera = [row[:3] for row in T_camera_body[:3]]
            self._t_camera_body = [row[3] for row in T_camera_body[:3]]
        else:
            self._R_body_camera = T_camera_body[:3, :3]
            self._t_camera_body = T_camera_body[:3, 3]
        # Camera translation in body frame (mounting offset).
        # Defaults: position_x=0.15, position_y=0.0, position_z=0.45
        # Auto-tuner (W3-4) — instantiated on demand via tune_bbox_gains skill
        self._gain_tuner = GainAutoTuner()
        # Load persisted gains (overrides config defaults if file exists)
        self._load_persisted_gains()

    # ── Gain persistence (W3-4) ──────────────────────────────────────────────

    def _load_persisted_gains(self) -> None:
        """Load tuned gains from ~/.lingtu/bbox_navigator_gains.json if present.

        Falls back to DimOS default gains (linear=0.8, angular=1.5) and logs
        an INFO message instructing the user to run the tune skill.
        """
        try:
            if self._gains_path.exists():
                with open(self._gains_path, encoding="utf-8") as fh:
                    data = json.load(fh)
                robot_gains = data.get(self._robot_id)
                if robot_gains:
                    self._cfg.linear_gain = float(robot_gains["linear_gain"])
                    self._cfg.angular_gain = float(robot_gains["angular_gain"])
                    logger.info(
                        "BBoxNavigator: loaded persisted gains for robot_id='%s' "
                        "(Kp_lin=%.3f, Kp_ang=%.3f)",
                        self._robot_id,
                        self._cfg.linear_gain,
                        self._cfg.angular_gain,
                    )
                    return
        except Exception as exc:
            logger.warning("BBoxNavigator: failed to load persisted gains: %s", exc)

        logger.info(
            "BBoxNavigator: using default gains (Kp_lin=%.1f, Kp_ang=%.1f) "
            "— run tune skill for S100P-specific tuning",
            self._cfg.linear_gain,
            self._cfg.angular_gain,
        )

    def _save_gains(self) -> None:
        """Persist current gains to ~/.lingtu/bbox_navigator_gains.json."""
        try:
            self._gains_path.parent.mkdir(parents=True, exist_ok=True)
            data: dict = {}
            if self._gains_path.exists():
                try:
                    with open(self._gains_path, encoding="utf-8") as fh:
                        data = json.load(fh)
                except Exception:
                    data = {}
            data[self._robot_id] = {
                "linear_gain": self._cfg.linear_gain,
                "angular_gain": self._cfg.angular_gain,
            }
            with open(self._gains_path, "w", encoding="utf-8") as fh:
                json.dump(data, fh, indent=2)
            logger.info(
                "BBoxNavigator: gains persisted to %s (robot_id='%s')",
                self._gains_path,
                self._robot_id,
            )
        except Exception as exc:
            logger.error("BBoxNavigator: failed to persist gains: %s", exc)

    def tune_bbox_gains(
        self,
        yaw_series: list[float],
        dt: float,
        duration: float = 6.0,
    ) -> dict:
        """Run Ziegler-Nichols PD tuning from a captured yaw oscillation series.

        This method is the computation core of the tune skill. In production on
        S100P, the yaw_series is captured by applying relay steps via the
        robot driver for `duration` seconds, then calling this method with the
        recorded measurements.

        In tests, pass a synthetic yaw_series directly.

        Args:
            yaw_series: list of yaw measurements (radians) at uniform interval dt.
            dt: time between measurements (seconds).
            duration: nominal experiment duration — used only for the report.

        Returns:
            dict with keys: K_u, T_u, a_u, Kp_ang, Kd_ang, converged, robot_id.
            On success, also updates self._cfg.angular_gain and persists to file.
        """
        T_u, a_u = self._gain_tuner.analyse_oscillation(yaw_series, dt)
        K_u, Kp, Kd, converged = self._gain_tuner.compute_zn_pd(T_u, a_u)

        report = {
            "robot_id": self._robot_id,
            "duration_s": duration,
            "T_u": T_u,
            "a_u": a_u,
            "K_u": K_u,
            "Kp_ang": Kp,
            "Kd_ang": Kd,
            "converged": bool(converged),
        }

        if converged:
            self._cfg.angular_gain = Kp
            self._save_gains()
            logger.info(
                "BBoxNavigator: tuning converged — K_u=%.3f, T_u=%.3fs, "
                "Kp_ang=%.3f, Kd_ang=%.3f",
                K_u, T_u, Kp, Kd,
            )
        else:
            logger.warning(
                "BBoxNavigator: tuning did not converge (T_u=%.3f, a_u=%.3f) "
                "— keeping existing gains",
                T_u, a_u,
            )

        return report

    # ── 公开接口 ─────────────────────────────────────────────────────────────

    def set_target_bbox(
        self, bbox: list, frame_timestamp: float | None = None
    ) -> None:
        """
        设置/更新目标 bbox [x1, y1, x2, y2]。

        可选 frame_timestamp 用于精确丢失计时；
        不传则用调用时刻。
        """
        ts = frame_timestamp if frame_timestamp is not None else time.time()
        with self._lock:
            self._target_bbox = list(bbox)
            self._last_bbox_time = ts
            if self._state in (STATE_IDLE, STATE_LOST):
                self._state = STATE_TRACKING

    def compute_3d_from_bbox(
        self,
        bbox: list,
        depth_image: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        robot_pose: tuple | None = None,  # (x, y, yaw) world frame
    ) -> np.ndarray | None:
        """
        bbox center + depth image → 3D coordinate.

        Steps:
          1. Compute bbox center pixel (u, v).
          2. Sample a 5x5 patch centered on (u, v) inside the bbox.
          3. Apply IQR outlier rejection (drop values outside Q1-1.5*IQR and
             Q3+1.5*IQR). Take the median of the surviving samples.
          4. Publish depth_confidence: 1.0 when >5 valid samples after
             rejection, linearly scaled down to 0.0 at 0-1 samples.
          5. If all samples are rejected or the bbox has no valid depth,
             return None — do not default to 0.
          6. (u, v, depth) → camera-frame 3D point (X_c, Y_c, Z_c).
          7. If robot_pose provided, rotate to world frame.

        Returns:
            np.ndarray shape (3,) or None (no valid depth).
        """
        x1, y1, x2, y2 = float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])

        # Bbox center pixel
        u = (x1 + x2) / 2.0
        v = (y1 + y2) / 2.0

        h_img, w_img = depth_image.shape[:2]

        # Build 5x5 sample grid clamped inside the bbox and image bounds
        bbox_x1_c = max(x1, 0)
        bbox_y1_c = max(y1, 0)
        bbox_x2_c = min(x2, w_img - 1)
        bbox_y2_c = min(y2, h_img - 1)

        # Evenly distribute 5 points across each dimension within the bbox
        bw = bbox_x2_c - bbox_x1_c
        bh = bbox_y2_c - bbox_y1_c
        step_x = bw / 4.0 if bw > 0 else 0.0
        step_y = bh / 4.0 if bh > 0 else 0.0

        samples: list[float] = []
        for iy in range(5):
            py = int(bbox_y1_c + iy * step_y)
            py = max(0, min(py, h_img - 1))
            for ix in range(5):
                px = int(bbox_x1_c + ix * step_x)
                px = max(0, min(px, w_img - 1))
                d = float(depth_image[py, px])
                if d > 0 and np.isfinite(d):
                    samples.append(d)

        # IQR outlier rejection
        depth_m: float | None = None
        self.depth_confidence: float = 0.0

        if len(samples) >= 2:
            arr = np.array(samples, dtype=np.float32)
            q1 = float(np.percentile(arr, 25))
            q3 = float(np.percentile(arr, 75))
            iqr = q3 - q1
            lower = q1 - 1.5 * iqr
            upper = q3 + 1.5 * iqr
            kept = arr[(arr >= lower) & (arr <= upper)]
            n_kept = len(kept)
            # Confidence: 1.0 when >5 survived, linear down to 0.0 at 0-1
            if n_kept > 5:
                self.depth_confidence = 1.0
            elif n_kept <= 1:
                self.depth_confidence = 0.0
            else:
                # n_kept in [2, 5]: linear ramp
                self.depth_confidence = (n_kept - 1) / 4.0
            if n_kept >= 1:
                depth_m = float(np.median(kept))
        elif len(samples) == 1:
            # Single valid sample — low confidence, no outlier rejection possible
            self.depth_confidence = 0.0
            depth_m = samples[0]

        if depth_m is None:
            # No valid depth anywhere in the bbox — do not default to 0
            return None

        # 深度单位归一化: Orbbec Gemini 335 以毫米输出
        # 若 depth_image 最大值 > 100，推断为毫米，转换为米
        if depth_m > 100.0:
            depth_m /= 1000.0

        if depth_m <= 0.0:
            return None

        # 相机坐标系 3D (光学坐标: X 右, Y 下, Z 前)
        X_c = (u - cx) / fx * depth_m
        Y_c = (v - cy) / fy * depth_m
        Z_c = depth_m

        if robot_pose is None:
            # 返回相机坐标系 (调用方自行转换)
            return np.array([X_c, Y_c, Z_c], dtype=float)

        # 转换到世界坐标系: camera 3D → body frame → world frame
        rx, ry, yaw = robot_pose
        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)

        # Camera optical → body frame via calibrated rotation + translation
        cam_pt = np.array([X_c, Y_c, Z_c])
        body_pt = self._R_body_camera @ cam_pt + self._t_camera_body

        wx = rx + cos_y * body_pt[0] - sin_y * body_pt[1]
        wy = ry + sin_y * body_pt[0] + cos_y * body_pt[1]
        wz = body_pt[2]

        return np.array([wx, wy, wz], dtype=float)

    def compute_cmd_vel(
        self,
        target_3d: np.ndarray,
        robot_pose: tuple,   # (x, y, yaw)
    ) -> tuple:
        """
        PD 控制器 — 从 DimOS DetectionNavigation._compute_twist_from_3d 移植。

        参考 detection_navigation.py:153-208:
          - 角度误差 → angular_z (P 控制，归一化到 [-π, π])
          - 距离误差 → linear_x (P 控制，转弯时线速度衰减)
          - 太近 → 后退

        Returns:
            (linear_x: float, angular_z: float)
        """
        cfg = self._cfg
        rx, ry, yaw = robot_pose

        dx = float(target_3d[0]) - rx
        dy = float(target_3d[1]) - ry
        distance = float(np.sqrt(dx * dx + dy * dy))

        # 角度误差，归一化到 [-π, π]. Using modulo instead of a while-loop
        # avoids an infinite loop if angle_error is NaN.
        angle_to_target = np.arctan2(dy, dx)
        angle_error = (angle_to_target - yaw + np.pi) % (2.0 * np.pi) - np.pi

        # 角速度
        angular_z = float(np.clip(
            angle_error * cfg.angular_gain,
            -cfg.max_angular_speed,
            cfg.max_angular_speed,
        ))

        # 线速度
        if distance < cfg.min_distance:
            # 太近，后退
            linear_x = -cfg.max_linear_speed * 0.6
        else:
            distance_error = distance - cfg.target_distance
            # 转弯时减速 (turn_factor: 0.3 ~ 1.0)
            turn_factor = 1.0 - min(abs(angle_error) / np.pi, 0.7)
            linear_x = float(np.clip(
                distance_error * cfg.linear_gain * turn_factor,
                -cfg.max_linear_speed,
                cfg.max_linear_speed,
            ))

        return linear_x, angular_z

    def update(
        self,
        bbox: list,
        depth_image: np.ndarray,
        camera_intrinsics: tuple,  # (fx, fy, cx, cy)
        robot_pose: tuple,         # (x, y, yaw)
    ) -> dict:
        """
        主更新方法 — 每帧调用一次。

        Args:
            bbox: [x1, y1, x2, y2] 像素坐标
            depth_image: 与 RGB 对齐的深度图 (H×W, uint16 mm 或 float32 m)
            camera_intrinsics: (fx, fy, cx, cy)
            robot_pose: (x, y, yaw) 世界坐标系

        Returns:
            {
                "state":     "tracking" | "arrived" | "lost" | "idle",
                "linear_x":  float,
                "angular_z": float,
                "target_3d": [x, y, z] or None,
                "distance":  float,
            }
        """
        now = time.time()
        fx, fy, cx, cy = camera_intrinsics

        # 更新 bbox 时间戳
        self.set_target_bbox(bbox, frame_timestamp=now)

        # 丢失检测 (在本帧有 bbox 时不会触发，但防止外部不调用 set_target_bbox)
        with self._lock:
            elapsed = now - self._last_bbox_time
            if elapsed > self._cfg.lost_timeout and self._state == STATE_TRACKING:
                self._state = STATE_LOST
                self._target_3d = None

        # 3D 投影
        target_3d = self.compute_3d_from_bbox(
            bbox, depth_image, fx, fy, cx, cy, robot_pose
        )

        if target_3d is None:
            with self._lock:
                self._depth_fail_streak += 1
                if self._last_valid_3d_time == 0.0:
                    # First-ever 3D attempt failed — seed the timer so the
                    # LOST transition below has something to diff against.
                    self._last_valid_3d_time = now
                elapsed_no_3d = now - self._last_valid_3d_time
                if self._depth_fail_streak == 1:
                    logger.warning(
                        "BBoxNavigator: depth back-projection failed (bbox=%s)",
                        bbox,
                    )
                if (
                    self._state == STATE_TRACKING
                    and elapsed_no_3d > self._cfg.lost_timeout
                ):
                    logger.info(
                        "BBoxNavigator: target lost — no valid 3D for %.1fs (%d frames)",
                        elapsed_no_3d,
                        self._depth_fail_streak,
                    )
                    self._state = STATE_LOST
                    self._target_3d = None
                current_state = self._state
            return {
                "state": current_state,
                "linear_x": 0.0,
                "angular_z": 0.0,
                "target_3d": None,
                "distance": 0.0,
            }

        with self._lock:
            self._target_3d = target_3d
            self._last_valid_3d_time = now
            self._depth_fail_streak = 0

        # 距离计算 (XY 平面)
        rx, ry, _ = robot_pose
        dx = float(target_3d[0]) - rx
        dy = float(target_3d[1]) - ry
        distance = float(np.sqrt(dx * dx + dy * dy))

        # 到达判定
        if distance <= self._cfg.arrived_threshold:
            with self._lock:
                self._state = STATE_ARRIVED
            return {
                "state": STATE_ARRIVED,
                "linear_x": 0.0,
                "angular_z": 0.0,
                "target_3d": target_3d.tolist(),
                "distance": distance,
            }

        # 控制量计算
        linear_x, angular_z = self.compute_cmd_vel(target_3d, robot_pose)

        with self._lock:
            self._state = STATE_TRACKING

        return {
            "state": STATE_TRACKING,
            "linear_x": linear_x,
            "angular_z": angular_z,
            "target_3d": target_3d.tolist(),
            "distance": distance,
        }

    def stop(self) -> None:
        """停止跟踪，清空状态。"""
        with self._lock:
            self._state = STATE_IDLE
            self._target_bbox = None
            self._target_3d = None
            self._last_bbox_time = 0.0
            self._last_valid_3d_time = 0.0
            self._depth_fail_streak = 0

    def tick_lost_check(self) -> None:
        """
        外部定时调用 (可选)，在没有新 bbox 时推进丢失判定。

        如果 ROS2 节点在检测失败时不调用 update()，
        可每 0.5s 调用一次此方法。
        """
        with self._lock:
            if self._state == STATE_TRACKING:
                elapsed = time.time() - self._last_bbox_time
                if elapsed > self._cfg.lost_timeout:
                    logger.info(
                        "BBoxNavigator: target lost (no bbox for %.1fs)", elapsed
                    )
                    self._state = STATE_LOST
                    self._target_3d = None

    # ── 属性 ─────────────────────────────────────────────────────────────────

    @property
    def is_active(self) -> bool:
        """True 当且仅当正在跟踪目标（tracking 状态）。"""
        with self._lock:
            return self._state == STATE_TRACKING

    @property
    def state(self) -> str:
        """当前状态字符串: idle | tracking | arrived | lost。"""
        with self._lock:
            return self._state

    @property
    def target_3d(self) -> np.ndarray | None:
        """最近一帧计算的目标 3D 坐标（世界坐标系），无则 None。"""
        with self._lock:
            return self._target_3d.copy() if self._target_3d is not None else None
