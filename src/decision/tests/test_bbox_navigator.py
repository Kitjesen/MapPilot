"""BBox Navigator 单元测试 — PD 控制器 + 3D 投影。
不依赖 ROS2，纯 Python + numpy 可运行。

运行:
    cd src/decision && python -m pytest test/test_bbox_navigator.py -v
"""

import time

import numpy as np

from decision.vision.bbox_navigator import (
    STATE_ARRIVED,
    STATE_IDLE,
    STATE_LOST,
    STATE_TRACKING,
    BBoxNavConfig,
    BBoxNavigator,
)

# ---------------------------------------------------------------------------
# 测试辅助
# ---------------------------------------------------------------------------

def _make_depth_image(h: int, w: int, depth_val: float) -> np.ndarray:
    """创建均匀深度图（单位: 毫米，与 Orbbec 输出一致）。"""
    return np.full((h, w), depth_val, dtype=np.float32)


def _make_intrinsics(w: int = 640, h: int = 480, fov_deg: float = 60.0):
    """从图像尺寸和视场角计算相机内参 (fx, fy, cx, cy)。"""
    fx = (w / 2.0) / np.tan(np.radians(fov_deg / 2.0))
    fy = fx
    cx = w / 2.0
    cy = h / 2.0
    return fx, fy, cx, cy


# ---------------------------------------------------------------------------
# 3D 投影测试
# ---------------------------------------------------------------------------

class Test3DProjection:
    def setup_method(self):
        self.nav = BBoxNavigator()
        self.W, self.H = 640, 480
        self.fx, self.fy, self.cx, self.cy = _make_intrinsics(self.W, self.H)

    def test_3d_projection_center(self):
        """bbox 在图像中心 → 3D 点 X_c ≈ 0（正前方）。"""
        cx, cy = self.cx, self.cy
        half = 20
        bbox = [cx - half, cy - half, cx + half, cy + half]
        depth = _make_depth_image(self.H, self.W, 3000.0)  # 3 m in mm
        pt = self.nav.compute_3d_from_bbox(bbox, depth, self.fx, self.fy, self.cx, self.cy)
        assert pt is not None
        # X_c (横向偏移) 应接近 0
        assert abs(pt[0]) < 0.05
        # Z_c (深度) 应约 3 m
        assert abs(pt[2] - 3.0) < 0.1

    def test_3d_projection_offset_left(self):
        """bbox 偏左 → 3D 点 X_c < 0（左侧）。"""
        # bbox 中心在图像左四分之一处
        u_center = self.W * 0.25
        v_center = self.H * 0.5
        half = 20
        bbox = [u_center - half, v_center - half, u_center + half, v_center + half]
        depth = _make_depth_image(self.H, self.W, 2000.0)  # 2 m
        pt = self.nav.compute_3d_from_bbox(bbox, depth, self.fx, self.fy, self.cx, self.cy)
        assert pt is not None
        assert pt[0] < 0.0  # 左侧 → 负 X_c

    def test_depth_roi_median_ignores_noise(self):
        """中值深度应不受少量噪声像素影响。"""
        depth = _make_depth_image(self.H, self.W, 3000.0)
        # 在中心区域注入几个极端值
        depth[235:245, 315:325] = 0.0      # 无效值
        depth[240, 320] = 99999.0           # 异常高值

        cx, cy = self.cx, self.cy
        half = 30
        bbox = [cx - half, cy - half, cx + half, cy + half]
        pt = self.nav.compute_3d_from_bbox(bbox, depth, self.fx, self.fy, self.cx, self.cy)
        assert pt is not None
        # 结果应仍接近 3 m (中值滤波掉了噪声)
        assert abs(pt[2] - 3.0) < 0.3

    def test_camera_translation_applied(self):
        """相机平移偏移在 body→world 投影中应正确添加。

        camera origin (0,0,1) 在 body 坐标系中应为 t = [t_x, t_y, t_z]，
        因为 body_pt = R @ cam_pt + t。
        """
        nav = BBoxNavigator()
        # 覆盖 transform 为已知值以独立测试
        nav._R_body_camera = np.eye(3, dtype=np.float64)
        nav._t_camera_body = np.array([0.15, 0.0, 0.45], dtype=np.float64)

        # bbox 在图像中心 → (cx, cy) → X_c ≈ 0, Y_c ≈ 0, Z_c ≈ 1.0
        half = 20
        bbox = [self.cx - half, self.cy - half, self.cx + half, self.cy + half]
        depth = _make_depth_image(self.H, self.W, 1000.0)  # 1 m
        robot_pose = (0.0, 0.0, 0.0)
        pt = nav.compute_3d_from_bbox(
            bbox, depth, self.fx, self.fy, self.cx, self.cy, robot_pose
        )
        assert pt is not None
        # body_pt = I @ (0, 0, 1) + (0.15, 0, 0.45) = (0.15, 0, 1.45)
        assert abs(pt[0] - 0.15) < 0.05, f"Expected X≈0.15, got {pt[0]}"
        assert abs(pt[1]) < 0.05, f"Expected Y≈0.0, got {pt[1]}"
        assert abs(pt[2] - 1.45) < 0.1, f"Expected Z≈1.45, got {pt[2]}"


# ---------------------------------------------------------------------------
# PD 控制器测试
# ---------------------------------------------------------------------------

class TestPDController:
    def setup_method(self):
        self.nav = BBoxNavigator()

    def test_pd_forward(self):
        """目标在正前方 5 m → 正线速度，角速度接近 0。"""
        # 机器人在原点，朝向 0 rad (向右, +x 方向)
        robot_pose = (0.0, 0.0, 0.0)
        target_3d = np.array([5.0, 0.0, 0.0])
        lx, az = self.nav.compute_cmd_vel(target_3d, robot_pose)
        assert lx > 0.0
        assert abs(az) < 0.1

    def test_pd_backward(self):
        """目标太近 (< min_distance) → 负线速度（后退）。"""
        robot_pose = (0.0, 0.0, 0.0)
        # 距离 0.3 m < min_distance=0.8
        target_3d = np.array([0.3, 0.0, 0.0])
        lx, _az = self.nav.compute_cmd_vel(target_3d, robot_pose)
        assert lx < 0.0

    def test_pd_turn_left(self):
        """目标在左侧 (正 y 方向，机器人朝 +x) → 正角速度（左转）。"""
        robot_pose = (0.0, 0.0, 0.0)  # 朝向 0 rad = +x 轴
        target_3d = np.array([0.0, 5.0, 0.0])  # 正左方 (ROS: +y 是左)
        _, az = self.nav.compute_cmd_vel(target_3d, robot_pose)
        assert az > 0.0

    def test_pd_turn_right(self):
        """目标在右侧 (负 y 方向，机器人朝 +x) → 负角速度（右转）。"""
        robot_pose = (0.0, 0.0, 0.0)
        target_3d = np.array([0.0, -5.0, 0.0])  # 正右方
        _, az = self.nav.compute_cmd_vel(target_3d, robot_pose)
        assert az < 0.0

    def test_pd_speed_clamp_linear(self):
        """线速度不超过 max_linear_speed。"""
        cfg = self.nav._cfg
        robot_pose = (0.0, 0.0, 0.0)
        # 超远目标
        target_3d = np.array([1000.0, 0.0, 0.0])
        lx, _ = self.nav.compute_cmd_vel(target_3d, robot_pose)
        assert abs(lx) <= cfg.max_linear_speed + 1e-6

    def test_pd_speed_clamp_angular(self):
        """角速度不超过 max_angular_speed。"""
        cfg = self.nav._cfg
        robot_pose = (0.0, 0.0, 0.0)
        # 目标在正后方
        target_3d = np.array([-5.0, 0.001, 0.0])
        _, az = self.nav.compute_cmd_vel(target_3d, robot_pose)
        assert abs(az) <= cfg.max_angular_speed + 1e-6


# ---------------------------------------------------------------------------
# 状态机测试
# ---------------------------------------------------------------------------

class TestStateMachine:
    def setup_method(self):
        cfg = BBoxNavConfig(arrived_threshold=0.3, lost_timeout=0.2)
        self.nav = BBoxNavigator(config=cfg)
        self.W, self.H = 640, 480
        self.fx, self.fy, self.cx, self.cy = _make_intrinsics(self.W, self.H)
        self.intrinsics = (self.fx, self.fy, self.cx, self.cy)

    def _center_bbox(self):
        cx, cy = self.cx, self.cy
        half = 20
        return [cx - half, cy - half, cx + half, cy + half]

    def test_arrived_when_very_close(self):
        """距离 < arrived_threshold → state=arrived，速度为 0。"""
        # 深度 0.2 m (< arrived_threshold=0.3m)，机器人在 (0,0)
        depth = _make_depth_image(self.H, self.W, 200.0)  # 200 mm = 0.2 m
        robot_pose = (0.0, 0.0, 0.0)
        bbox = self._center_bbox()
        out = self.nav.update(bbox, depth, self.intrinsics, robot_pose)
        assert out["state"] == STATE_ARRIVED
        assert out["linear_x"] == 0.0
        assert out["angular_z"] == 0.0

    def test_tracking_at_normal_distance(self):
        """正常距离 → state=tracking，有控制输出。

        注意: 相机安装在 body 前方 0.15m，因此图像正前方的目标
        在 body XY 平面有 0.15m 偏移。使用偏右 bbox 使 XY 总
        偏移 > arrived_threshold=0.3 以确保触发 tracking 而非 arrived。
        """
        depth = _make_depth_image(self.H, self.W, 3000.0)  # 3 m
        robot_pose = (0.0, 0.0, 0.0)
        # bbox 偏右: u≈400, v≈240 → X_c≈0.43 → 总 XY 距离 ≈ 0.58 > 0.3
        bbox = [380.0, 220.0, 420.0, 260.0]
        out = self.nav.update(bbox, depth, self.intrinsics, robot_pose)
        assert out["state"] == STATE_TRACKING
        assert out["linear_x"] != 0.0 or out["angular_z"] != 0.0

    def test_lost_timeout(self):
        """超过 lost_timeout 未更新 bbox → state=lost。"""
        # 先设置为 tracking
        self.nav._state = STATE_TRACKING
        self.nav._last_bbox_time = time.time() - 10.0  # 10 秒前

        self.nav.tick_lost_check()
        assert self.nav.state == STATE_LOST

    def test_stop_resets_to_idle(self):
        """调用 stop() → state=idle，所有状态清空。"""
        self.nav._state = STATE_TRACKING
        self.nav.stop()
        assert self.nav.state == STATE_IDLE
        assert self.nav.target_3d is None

    def test_set_target_bbox_resumes_from_lost(self):
        """lost 状态下设置新 bbox → 恢复 tracking。"""
        self.nav._state = STATE_LOST
        self.nav.set_target_bbox([100, 100, 200, 200])
        assert self.nav.state == STATE_TRACKING


# ---------------------------------------------------------------------------
# 配置参数测试
# ---------------------------------------------------------------------------

class TestConfig:
    def test_servo_takeover_distance_exists(self):
        """BBoxNavConfig 应有 servo_takeover_distance 参数。"""
        cfg = BBoxNavConfig()
        assert hasattr(cfg, "servo_takeover_distance")
        assert cfg.servo_takeover_distance > 0.0

    def test_custom_config_applied(self):
        """自定义配置应被正确应用。"""
        cfg = BBoxNavConfig(
            target_distance=2.0,
            max_linear_speed=1.0,
            servo_takeover_distance=5.0,
        )
        nav = BBoxNavigator(config=cfg)
        assert nav._cfg.target_distance == 2.0
        assert nav._cfg.max_linear_speed == 1.0
        assert nav._cfg.servo_takeover_distance == 5.0

    def test_default_config_reasonable_values(self):
        """默认配置参数应在合理范围内。"""
        cfg = BBoxNavConfig()
        assert 0.0 < cfg.target_distance <= 5.0
        assert 0.0 < cfg.max_linear_speed <= 2.0
        assert 0.0 < cfg.max_angular_speed <= 3.0
        assert cfg.lost_timeout > 0.0
        assert 0.0 < cfg.arrived_threshold <= 1.0
