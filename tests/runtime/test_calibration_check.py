"""Tests for runtime.utils.calibration_check — startup calibration validation."""

import unittest
from dataclasses import dataclass, replace
from unittest.mock import mock_open, patch


# Minimal config stubs for testing (avoid importing real config)
@dataclass
class MockCameraConfig:
    position_x: float = 0.15
    position_y: float = 0.0
    position_z: float = 0.45
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    fx: float = 615.0
    fy: float = 615.0
    cx: float = 320.0
    cy: float = 240.0
    width: int = 640
    height: int = 480
    depth_scale: float = 0.001
    dist_k1: float = 0.0
    dist_k2: float = 0.0
    dist_p1: float = 0.0
    dist_p2: float = 0.0
    dist_k3: float = 0.0


@dataclass
class MockLidarConfig:
    frame_id: str = "livox_frame"
    publish_freq: float = 10.0
    offset_x: float = -0.011
    offset_y: float = -0.02329
    offset_z: float = 0.04412
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    camera_offset_z: float = 0.0


@dataclass
class MockRobotConfig:
    camera: MockCameraConfig = None
    lidar: MockLidarConfig = None
    raw: dict = None

    def __post_init__(self):
        if self.camera is None:
            self.camera = MockCameraConfig()
        if self.lidar is None:
            self.lidar = MockLidarConfig()
        if self.raw is None:
            self.raw = {}


class TestCalibrationCheck(unittest.TestCase):
    """Test run_calibration_check with various config scenarios."""

    def _run(self, config=None, **kw):
        from runtime.utils.calibration_check import run_calibration_check

        return run_calibration_check(config=config or MockRobotConfig(), **kw)

    def test_default_config_passes(self):
        """Default mock config should pass with at most warnings."""
        report = self._run()
        self.assertTrue(report.ok)
        self.assertEqual(len(report.errors), 0)

    def test_negative_focal_length_is_error(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(fx=-100, fy=615))
        report = self._run(cfg)
        self.assertFalse(report.ok)
        self.assertTrue(any("focal" in e.lower() for e in report.errors))

    def test_zero_focal_length_is_error(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(fx=0, fy=0))
        report = self._run(cfg)
        self.assertFalse(report.ok)

    def test_invalid_image_size_is_error(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(width=0, height=0))
        report = self._run(cfg)
        self.assertFalse(report.ok)

    def test_principal_point_far_from_center_warns(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(cx=10.0, cy=10.0))
        report = self._run(cfg)
        self.assertTrue(report.ok)  # warning not error
        self.assertTrue(any("principal" in w.lower() for w in report.warnings))

    def test_aspect_ratio_mismatch_warns(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(fx=615, fy=400))
        report = self._run(cfg)
        self.assertTrue(any("fx" in w and "fy" in w for w in report.warnings))

    def test_slam_approval_does_not_approve_unverified_camera(self):
        from runtime.utils import calibration_check as cc

        cfg = MockRobotConfig(
            raw={
                "calibration": {
                    "slam": {"status": "verified", "reason": "field evidence"},
                    "camera": {"status": "unverified", "reason": "not measured"},
                }
            }
        )
        report = cc.CalibrationReport()

        cc._check_calibration_approval(
            cfg,
            report,
            require_camera=False,
            require_slam=True,
        )

        self.assertEqual(report.errors, [])
        self.assertTrue(any("SLAM/LiDAR-IMU" in item for item in report.info))
        self.assertTrue(any("Camera calibration is unverified" in item for item in report.warnings))

    def test_camera_product_stays_blocked_when_only_slam_is_verified(self):
        from runtime.utils import calibration_check as cc

        cfg = MockRobotConfig(
            raw={
                "calibration": {
                    "slam": {"status": "verified", "reason": "field evidence"},
                    "camera": {"status": "unverified", "reason": "not measured"},
                }
            }
        )
        report = cc.CalibrationReport()

        cc._check_calibration_approval(
            cfg,
            report,
            require_camera=True,
            require_slam=False,
        )

        self.assertTrue(any("Camera calibration is unverified" in item for item in report.errors))


class TestCameraExtrinsics(unittest.TestCase):
    def _run(self, config, **kw):
        from runtime.utils.calibration_check import run_calibration_check

        return run_calibration_check(config=config, **kw)

    def test_all_zero_position_warns(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(position_x=0, position_y=0, position_z=0))
        report = self._run(cfg)
        self.assertTrue(any("zeros" in w.lower() for w in report.warnings))

    def test_all_zero_position_errors_when_required(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(position_x=0, position_y=0, position_z=0))
        report = self._run(cfg, require_camera=True)
        self.assertFalse(report.ok)
        self.assertTrue(any("zeros" in e.lower() for e in report.errors))

    def test_large_position_warns(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(position_x=5.0))
        report = self._run(cfg)
        self.assertTrue(any("too large" in w.lower() for w in report.warnings))

    def test_identity_rotation_warns(self):
        """Camera with identity rotation (looking up not forward) should warn."""
        cfg = MockRobotConfig()  # default has roll=pitch=yaw=0
        report = self._run(cfg)
        self.assertTrue(any("identity" in w.lower() for w in report.warnings))

    def test_nonzero_rotation_no_identity_warn(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(pitch=-1.57))
        report = self._run(cfg)
        self.assertFalse(any("identity" in w.lower() for w in report.warnings))

    def test_large_rotation_warns(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(roll=2.0, pitch=2.0, yaw=2.0))
        report = self._run(cfg)
        self.assertTrue(any("very large" in w.lower() for w in report.warnings))


class TestLidarExtrinsics(unittest.TestCase):
    def _run(self, config):
        from runtime.utils.calibration_check import run_calibration_check

        return run_calibration_check(config=config)

    def _go2_config(self):
        from runtime.config import load_config
        from runtime.utils.calibration_check import LTLIO_CONFIG

        return load_config(str(LTLIO_CONFIG.parent.parent / "robot.yaml"))

    def test_near_zero_lidar_offset_warns(self):
        cfg = MockRobotConfig(lidar=MockLidarConfig(offset_x=0, offset_y=0, offset_z=0))
        report = self._run(cfg)
        self.assertTrue(any("near zero" in w.lower() for w in report.warnings))

    def test_large_lidar_offset_warns(self):
        cfg = MockRobotConfig(lidar=MockLidarConfig(offset_x=1.0, offset_y=0, offset_z=0))
        report = self._run(cfg)
        self.assertTrue(any("large" in w.lower() for w in report.warnings))

    def test_normal_lidar_offset_passes(self):
        cfg = MockRobotConfig()  # default has mag ~ 0.05m
        report = self._run(cfg)
        self.assertTrue(any("lidar offset" in i.lower() for i in report.info))

    def test_product_ltlio_config_path_exists(self):
        from runtime.utils import calibration_check as cc

        self.assertTrue(cc.LTLIO_CONFIG.is_file(), cc.LTLIO_CONFIG)
        self.assertEqual(cc.LTLIO_CONFIG.name, "mid360_ltlio.yaml")
        self.assertIn("config/robots/unitree/go2/sensors", cc.LTLIO_CONFIG.as_posix())

    def test_omitted_rotations_use_identity_but_explicit_bad_rotations_fail(self):
        import yaml

        from runtime.utils import calibration_check as cc

        for rotations, expected_ok in (
            ({}, True),
            ({"r_il": [1.0, 0.0]}, False),
            ({"navigation_body_from_imu_rotation": [1.0, 0.0]}, False),
        ):
            with self.subTest(rotations=rotations):
                payload = {"t_il": [-0.011, -0.02329, 0.04412], **rotations}
                report = cc.CalibrationReport()
                with patch("pathlib.Path.exists", return_value=True), patch(
                    "builtins.open", mock_open(read_data=yaml.safe_dump(payload))
                ):
                    cc._check_lidar_imu_consistency(MockRobotConfig(), report, required=True)
                self.assertEqual(report.ok, expected_ok, report.errors)
                if expected_ok:
                    self.assertTrue(any("body-to-lidar" in item for item in report.info), report.info)
                else:
                    self.assertIn("LTLIO LiDAR/IMU rotations must be 3x3 matrices", report.errors)

    def test_go2_body_lidar_transform_matches_ltlio_runtime(self):
        cfg = self._go2_config()

        report = self._run(cfg)

        self.assertTrue(report.ok, report.errors)
        self.assertFalse(
            any("lidar mount" in warning.lower() for warning in report.warnings),
            report.warnings,
        )
        self.assertTrue(
            any("body-to-lidar" in item.lower() for item in report.info),
            report.info,
        )

    def test_go2_body_lidar_rotation_mismatch_is_error_when_slam_required(self):
        cfg = self._go2_config()
        cfg.lidar = replace(cfg.lidar, pitch=cfg.lidar.pitch + 0.1)

        from runtime.utils.calibration_check import run_calibration_check

        report = run_calibration_check(config=cfg, require_slam=True)

        self.assertFalse(report.ok)
        self.assertTrue(
            any("lidar mount rotation mismatch" in error.lower() for error in report.errors),
            report.errors,
        )


class TestImuNoise(unittest.TestCase):
    def _run(self, noise, *, require_camera=False):
        import yaml

        from runtime.utils import calibration_check as cc

        identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        payload = {
            "t_il": [-0.011, -0.02329, 0.04412],
            "r_il": identity,
            "navigation_body_from_imu_rotation": identity,
            **noise,
        }
        cfg = MockRobotConfig(
            raw={
                "calibration": {
                    "slam": {"status": "verified", "reason": "field evidence"},
                    "camera": {"status": "unverified", "reason": "not measured"},
                }
            }
        )
        with patch("pathlib.Path.exists", return_value=True), patch(
            "builtins.open", mock_open(read_data=yaml.safe_dump(payload))
        ):
            return cc.run_calibration_check(cfg, require_slam=True, require_camera=require_camera)

    def test_omitted_noise_uses_native_defaults(self):
        report = self._run({})
        self.assertTrue(report.ok, report.errors)

    def test_valid_explicit_noise_and_partial_overrides_pass(self):
        for noise in ({"na": 0.01, "ng": 0.003}, {"na": 0.01}, {"ng": 0.003}):
            with self.subTest(noise=noise):
                report = self._run(noise)
                self.assertTrue(report.ok, report.errors)

    def test_invalid_explicit_noise_blocks_startup(self):
        for name in ("na", "ng"):
            for value in (0, -0.01, float("nan"), float("inf"), "invalid", None):
                with self.subTest(name=name, value=value):
                    report = self._run({name: value})
                    self.assertFalse(report.ok)
                    self.assertTrue(any("SLAM IMU noise" in item for item in report.errors), report.errors)

    def test_high_explicit_noise_keeps_warning(self):
        report = self._run({"na": 0.6})
        self.assertTrue(report.ok, report.errors)
        self.assertTrue(any("High IMU noise: na=" in item for item in report.warnings), report.warnings)

    def test_omitted_noise_does_not_approve_unverified_camera(self):
        report = self._run({}, require_camera=True)
        self.assertFalse(report.ok)
        self.assertTrue(any("Camera calibration is unverified" in item for item in report.errors), report.errors)
        self.assertFalse(any("SLAM IMU noise" in item for item in report.errors), report.errors)


class TestDepthScale(unittest.TestCase):
    def _run(self, config):
        from runtime.utils.calibration_check import run_calibration_check

        return run_calibration_check(config=config)

    def test_negative_depth_scale_is_error(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(depth_scale=-0.001))
        report = self._run(cfg)
        self.assertFalse(report.ok)
        self.assertTrue(any("depth_scale" in e for e in report.errors))

    def test_identity_depth_scale_warns(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(depth_scale=1.0))
        report = self._run(cfg)
        self.assertTrue(any("depth_scale=1.0" in w for w in report.warnings))

    def test_normal_depth_scale_passes(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(depth_scale=0.001))
        report = self._run(cfg)
        self.assertTrue(any("depth_scale" in i for i in report.info))

    def test_large_depth_scale_warns(self):
        cfg = MockRobotConfig(camera=MockCameraConfig(depth_scale=1000.0))
        report = self._run(cfg)
        self.assertTrue(any("depth_scale" in w for w in report.warnings))


class TestCalibrationReport(unittest.TestCase):
    def test_report_ok_when_no_errors(self):
        from runtime.utils.calibration_check import CalibrationReport

        r = CalibrationReport()
        self.assertTrue(r.ok)

    def test_report_not_ok_with_error(self):
        from runtime.utils.calibration_check import CalibrationReport

        r = CalibrationReport(errors=["bad"])
        self.assertFalse(r.ok)

    def test_summary_all_pass(self):
        from runtime.utils.calibration_check import CalibrationReport

        r = CalibrationReport()
        self.assertIn("passed", r.summary())

    def test_summary_with_errors(self):
        from runtime.utils.calibration_check import CalibrationReport

        r = CalibrationReport(errors=["e1", "e2"], warnings=["w1"])
        s = r.summary()
        self.assertIn("2 ERROR", s)
        self.assertIn("1 warning", s)


class TestTimeOffset(unittest.TestCase):
    """Tests for the LTLIO LiDAR↔IMU time-offset check."""

    def _run_check(self, lio_offset=None, required=False):
        """Invoke _check_time_offset with mocked yaml loads."""
        from runtime.utils import calibration_check as cc

        report = cc.CalibrationReport()

        def fake_open(path, *args, **kwargs):
            from io import StringIO

            import yaml as _y

            payload = {}
            if str(path) == str(cc.LTLIO_CONFIG) and lio_offset is not None:
                payload = {"time_diff_lidar_to_imu": lio_offset}
            return StringIO(_y.dump(payload))

        with patch("pathlib.Path.exists", return_value=True), patch("builtins.open", side_effect=fake_open):
            cc._check_time_offset(report, required=required)
        return report

    def test_in_range_offset_passes(self):
        report = self._run_check(lio_offset=0.005)
        self.assertEqual(len(report.errors), 0)
        self.assertEqual(len(report.warnings), 0)

    def test_out_of_range_warns(self):
        report = self._run_check(lio_offset=0.5)
        self.assertTrue(any("exceeds plausible" in w for w in report.warnings))

    def test_out_of_range_errors_when_required(self):
        report = self._run_check(lio_offset=0.5, required=True)
        self.assertTrue(any("exceeds plausible" in e for e in report.errors))

    def test_missing_offsets_silent(self):
        report = self._run_check(lio_offset=None)
        self.assertEqual(len(report.errors), 0)
        self.assertEqual(len(report.warnings), 0)
        self.assertEqual(len(report.info), 0)

    def test_unreadable_ltlio_config_warns(self):
        from runtime.utils import calibration_check as cc

        report = cc.CalibrationReport()
        with patch("pathlib.Path.exists", return_value=True), patch(
            "builtins.open",
            side_effect=UnicodeDecodeError("utf-8", b"\xff", 0, 1, "invalid"),
        ):
            cc._check_time_offset(report, required=False)

        self.assertTrue(any("Cannot read LTLIO calibration data" in warning for warning in report.warnings))



if __name__ == "__main__":
    unittest.main()
