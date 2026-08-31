"""Decision module."""

import math
import tempfile
import time
from pathlib import Path

import numpy as np
import pytest

from decision.vision.bbox import (
    STATE_ARRIVED,
    STATE_IDLE,
    STATE_LOST,
    STATE_TRACKING,
    BBoxNavConfig,
    BBoxNavigator,
    GainAutoTuner,
)

# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------


def _make_depth_image(h: int, w: int, depth_val: float) -> np.ndarray:
    """Make depth image."""
    return np.full((h, w), depth_val, dtype=np.float32)


def _make_intrinsics(w: int = 640, h: int = 480, fov_deg: float = 60.0):
    """Make intrinsics."""
    fx = (w / 2.0) / np.tan(np.radians(fov_deg / 2.0))
    fy = fx
    cx = w / 2.0
    cy = h / 2.0
    return fx, fy, cx, cy


# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------


class Test3DProjection:
    def setup_method(self):
        self.nav = BBoxNavigator()
        self.W, self.H = 640, 480
        self.fx, self.fy, self.cx, self.cy = _make_intrinsics(self.W, self.H)

    def test_3d_projection_center(self):
        """Test 3d projection center."""
        cx, cy = self.cx, self.cy
        half = 20
        bbox = [cx - half, cy - half, cx + half, cy + half]
        depth = _make_depth_image(self.H, self.W, 3000.0)  # 3 m in mm
        pt = self.nav.compute_3d_from_bbox(bbox, depth, self.fx, self.fy, self.cx, self.cy)
        assert pt is not None

        assert abs(pt[0]) < 0.05

        assert abs(pt[2] - 3.0) < 0.1

    def test_3d_projection_offset_left(self):
        """Test 3d projection offset left."""

        u_center = self.W * 0.25
        v_center = self.H * 0.5
        half = 20
        bbox = [u_center - half, v_center - half, u_center + half, v_center + half]
        depth = _make_depth_image(self.H, self.W, 2000.0)  # 2 m
        pt = self.nav.compute_3d_from_bbox(bbox, depth, self.fx, self.fy, self.cx, self.cy)
        assert pt is not None
        assert pt[0] < 0.0

    def test_depth_roi_median_ignores_noise(self):
        """Test depth roi median ignores noise."""
        depth = _make_depth_image(self.H, self.W, 3000.0)

        depth[235:245, 315:325] = 0.0
        depth[240, 320] = 99999.0

        cx, cy = self.cx, self.cy
        half = 30
        bbox = [cx - half, cy - half, cx + half, cy + half]
        pt = self.nav.compute_3d_from_bbox(bbox, depth, self.fx, self.fy, self.cx, self.cy)
        assert pt is not None

        assert abs(pt[2] - 3.0) < 0.3

    def test_camera_translation_applied(self):
        """Test camera translation applied."""
        nav = BBoxNavigator()

        nav._R_body_camera = np.eye(3, dtype=np.float64)
        nav._t_camera_body = np.array([0.15, 0.0, 0.45], dtype=np.float64)

        half = 20
        bbox = [self.cx - half, self.cy - half, self.cx + half, self.cy + half]
        depth = _make_depth_image(self.H, self.W, 1000.0)  # 1 m
        robot_pose = (0.0, 0.0, 0.0)
        pt = nav.compute_3d_from_bbox(bbox, depth, self.fx, self.fy, self.cx, self.cy, robot_pose)
        assert pt is not None
        # body_pt = I @ (0, 0, 1) + (0.15, 0, 0.45) = (0.15, 0, 1.45)
        assert abs(pt[0] - 0.15) < 0.05, f"Expected X≈0.15, got {pt[0]}"
        assert abs(pt[1]) < 0.05, f"Expected Y≈0.0, got {pt[1]}"
        assert abs(pt[2] - 1.45) < 0.1, f"Expected Z≈1.45, got {pt[2]}"

    def test_depth_confidence_is_one_for_clean_samples(self):
        depth = _make_depth_image(100, 100, 2.0)
        assert self.nav.compute_3d_from_bbox([10, 10, 90, 90], depth, 500, 500, 50, 50) is not None
        assert self.nav.depth_confidence == 1.0

    def test_invalid_depth_returns_none_with_zero_confidence(self):
        depth = np.zeros((100, 100), dtype=np.float32)
        assert self.nav.compute_3d_from_bbox([10, 10, 90, 90], depth, 500, 500, 50, 50) is None
        assert self.nav.depth_confidence == 0.0

    def test_depth_sampling_rejects_center_outlier(self):
        depth = _make_depth_image(100, 100, 2.0)
        depth[50, 50] = 200.0
        point = self.nav.compute_3d_from_bbox([10, 10, 90, 90], depth, 500, 500, 50, 50)
        assert point is not None
        assert float(point[2]) == pytest.approx(2.0, abs=0.5)


# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------


class TestPDController:
    def setup_method(self):
        self.nav = BBoxNavigator()

    def test_pd_forward(self):
        """Test pd forward."""

        robot_pose = (0.0, 0.0, 0.0)
        target_3d = np.array([5.0, 0.0, 0.0])
        lx, az = self.nav.compute_cmd_vel(target_3d, robot_pose)
        assert lx > 0.0
        assert abs(az) < 0.1

    def test_pd_backward(self):
        """Test pd backward."""
        robot_pose = (0.0, 0.0, 0.0)

        target_3d = np.array([0.3, 0.0, 0.0])
        lx, _az = self.nav.compute_cmd_vel(target_3d, robot_pose)
        assert lx < 0.0

    def test_pd_turn_left(self):
        """Test pd turn left."""
        robot_pose = (0.0, 0.0, 0.0)
        target_3d = np.array([0.0, 5.0, 0.0])
        _, az = self.nav.compute_cmd_vel(target_3d, robot_pose)
        assert az > 0.0

    def test_pd_turn_right(self):
        """Test pd turn right."""
        robot_pose = (0.0, 0.0, 0.0)
        target_3d = np.array([0.0, -5.0, 0.0])
        _, az = self.nav.compute_cmd_vel(target_3d, robot_pose)
        assert az < 0.0

    def test_pd_speed_clamp_linear(self):
        """Test pd speed clamp linear."""
        cfg = self.nav._cfg
        robot_pose = (0.0, 0.0, 0.0)

        target_3d = np.array([1000.0, 0.0, 0.0])
        lx, _ = self.nav.compute_cmd_vel(target_3d, robot_pose)
        assert abs(lx) <= cfg.max_linear_speed + 1e-6

    def test_pd_speed_clamp_angular(self):
        """Test pd speed clamp angular."""
        cfg = self.nav._cfg
        robot_pose = (0.0, 0.0, 0.0)

        target_3d = np.array([-5.0, 0.001, 0.0])
        _, az = self.nav.compute_cmd_vel(target_3d, robot_pose)
        assert abs(az) <= cfg.max_angular_speed + 1e-6


# ---------------------------------------------------------------------------

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
        """Test arrived when very close."""

        depth = _make_depth_image(self.H, self.W, 200.0)  # 200 mm = 0.2 m
        robot_pose = (0.0, 0.0, 0.0)
        bbox = self._center_bbox()
        out = self.nav.update(bbox, depth, self.intrinsics, robot_pose)
        assert out["state"] == STATE_ARRIVED
        assert out["linear_x"] == 0.0
        assert out["angular_z"] == 0.0

    def test_tracking_at_normal_distance(self):
        """Test tracking at normal distance."""
        depth = _make_depth_image(self.H, self.W, 3000.0)  # 3 m
        robot_pose = (0.0, 0.0, 0.0)

        bbox = [380.0, 220.0, 420.0, 260.0]
        out = self.nav.update(bbox, depth, self.intrinsics, robot_pose)
        assert out["state"] == STATE_TRACKING
        assert out["linear_x"] != 0.0 or out["angular_z"] != 0.0

    def test_lost_timeout(self):
        """Test lost timeout."""

        self.nav._state = STATE_TRACKING
        self.nav._last_bbox_time = time.time() - 10.0

        self.nav.tick_lost_check()
        assert self.nav.state == STATE_LOST

    def test_stop_resets_to_idle(self):
        """Test stop resets to idle."""
        self.nav._state = STATE_TRACKING
        self.nav.stop()
        assert self.nav.state == STATE_IDLE
        assert self.nav.target_3d is None

    def test_set_target_bbox_resumes_from_lost(self):
        """Test set target bbox resumes from lost."""
        self.nav._state = STATE_LOST
        self.nav.set_target_bbox([100, 100, 200, 200])
        assert self.nav.state == STATE_TRACKING


# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------


class TestConfig:
    def test_servo_takeover_distance_exists(self):
        """Test servo takeover distance exists."""
        cfg = BBoxNavConfig()
        assert hasattr(cfg, "servo_takeover_distance")
        assert cfg.servo_takeover_distance > 0.0

    def test_custom_config_applied(self):
        """Test custom config applied."""
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
        """Test default config reasonable values."""
        cfg = BBoxNavConfig()
        assert 0.0 < cfg.target_distance <= 5.0
        assert 0.0 < cfg.max_linear_speed <= 2.0
        assert 0.0 < cfg.max_angular_speed <= 3.0
        assert cfg.lost_timeout > 0.0
        assert 0.0 < cfg.arrived_threshold <= 1.0


class TestGainAutoTuner:
    def test_zn_math_on_known_values(self):
        tuner = GainAutoTuner(relay_amplitude=0.3)
        ku, kp, kd, converged = tuner.compute_zn_pd(2.0, 0.5)

        expected_ku = 4.0 * tuner.relay_amplitude / (math.pi * 0.5)
        assert ku == pytest.approx(expected_ku)
        assert kp == pytest.approx(0.6 * expected_ku)
        assert kd == pytest.approx(kp * 2.0 / 8.0)
        assert converged == 1.0

    def test_degenerate_oscillation_returns_safe_defaults(self):
        _ku, kp, _kd, converged = GainAutoTuner().compute_zn_pd(0.0, 0.5)
        assert converged == 0.0
        assert kp > 0.0

    def test_analyse_oscillation_detects_period(self):
        tuner = GainAutoTuner()
        dt = 0.02
        t = np.arange(0, 4.0, dt)
        period, amplitude = tuner.analyse_oscillation(np.sin(2 * math.pi * t).tolist(), dt)

        assert period == pytest.approx(1.0, rel=0.1)
        assert amplitude == pytest.approx(1.0, abs=0.05)


class TestGainPersistence:
    def test_gain_file_save_and_load_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            gains_path = Path(tmpdir) / "gains.json"
            nav = BBoxNavigator(BBoxNavConfig(), robot_id="test_robot", gains_path=gains_path)
            dt = 0.02
            t = np.arange(0, 4.0, dt)
            report = nav.tune_bbox_gains((0.5 * np.sin(2 * math.pi * t)).tolist(), dt)
            reloaded = BBoxNavigator(BBoxNavConfig(), robot_id="test_robot", gains_path=gains_path)

            assert gains_path.exists()
            if report.get("converged"):
                assert reloaded._cfg.angular_gain == pytest.approx(nav._cfg.angular_gain)

    def test_missing_gains_file_uses_defaults(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            nav = BBoxNavigator(
                BBoxNavConfig(linear_gain=0.8, angular_gain=1.5),
                gains_path=Path(tmpdir) / "missing" / "gains.json",
            )

        assert nav._cfg.linear_gain == pytest.approx(0.8)
        assert nav._cfg.angular_gain == pytest.approx(1.5)
