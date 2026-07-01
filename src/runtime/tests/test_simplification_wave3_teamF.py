"""Wave 3 Team F — W3-4 BBoxNavigator gain auto-tuner + W3-5 PersonTracker OSNet Re-ID.

All tests are mock-based and require no hardware, ROS2, BPU, or torchreid.

Tests:
  W3-5: OSNetReIDEncoder
    1. test_osnet_raises_when_no_backend — RuntimeError when bpu_infer and torchreid both absent
    2. test_osnet_encode_returns_normalised_512d — mock BPU backend, verify output shape + norm
    3. test_person_tracker_adaptive_threshold — threshold scales with crowd density
    4. test_person_tracker_osnet_match_stat — osnet_match counter increments on Re-ID hit
    5. test_person_tracker_motion_prediction — predicted position advances by velocity*dt

  W3-4: GainAutoTuner + BBoxNavigator persistence + skill wire
    6. test_zn_math_on_synthetic_oscillation — ZN formula correct for known T_u / a_u
    7. test_gain_file_load_save_roundtrip — persist then load gains back correctly
    8. test_tune_skill_is_registered — tune_bbox_gains is discoverable as a @skill
    9. test_analyse_oscillation_detects_period — sinusoid zero-crossings → correct T_u
"""

from __future__ import annotations

import math
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

# ---------------------------------------------------------------------------
# Helpers to mock heavy deps before importing modules under test
# ---------------------------------------------------------------------------

def _stub_modules(*names: str) -> dict:
    """Return a dict of stub modules suitable for patch.dict(sys.modules, ...)."""
    stubs = {}
    for name in names:
        parts = name.split(".")
        # Ensure all parent packages exist
        for i in range(1, len(parts) + 1):
            key = ".".join(parts[:i])
            if key not in stubs:
                stubs[key] = types.ModuleType(key)
    return stubs


# ---------------------------------------------------------------------------
# W3-5 — OSNetReIDEncoder tests
# ---------------------------------------------------------------------------

class TestOSNetReIDEncoder(unittest.TestCase):
    """W3-5: OSNetReIDEncoder — backend selection, output shape, normalisation."""

    def _import_encoder(self, extra_stubs: dict | None = None):
        """Import OSNetReIDEncoder with bpu_infer and torchreid absent by default."""
        base = _stub_modules("torch", "torchvision", "PIL", "PIL.Image", "cv2")
        if extra_stubs:
            base.update(extra_stubs)
        with patch.dict(sys.modules, base):
            # Remove any cached import so patch takes effect
            sys.modules.pop(
                "decision.vision.osnet_reid", None
            )
            from decision.vision.osnet_reid import OSNetReIDEncoder
        return OSNetReIDEncoder

    def test_osnet_raises_when_no_backend(self):
        """RuntimeError when bpu_infer and torchreid are both absent."""
        # bpu_infer and torchreid are not in sys.modules → ImportError on import
        OSNetReIDEncoder = self._import_encoder()
        with self.assertRaises(RuntimeError) as ctx:
            OSNetReIDEncoder()
        self.assertIn("no backend available", str(ctx.exception))

    def test_osnet_encode_returns_normalised_512d(self):
        """Mock BPU backend: encode() returns (512,) float32 L2-normalised vector."""
        # Build a fake bpu_infer stub that returns a 512-dim output
        bpu_infer_stub = types.ModuleType("bpu_infer")
        fake_model = MagicMock()
        fake_model.forward.return_value = np.random.randn(512).astype(np.float32)
        bpu_infer_stub.load = MagicMock(return_value=fake_model)

        # Also stub PIL so _resize works
        pil_stub = types.ModuleType("PIL")
        pil_image_stub = types.ModuleType("PIL.Image")
        pil_image_stub.BILINEAR = 2
        pil_stub.Image = pil_image_stub

        stubs = _stub_modules("torch", "torchvision", "cv2")
        stubs["bpu_infer"] = bpu_infer_stub
        stubs["PIL"] = pil_stub
        stubs["PIL.Image"] = pil_image_stub

        sys.modules.pop("decision.vision.osnet_reid", None)
        with patch.dict(sys.modules, stubs):
            from decision.vision.osnet_reid import OSNetReIDEncoder

            # Create a temp file to simulate the .hbm existing
            with tempfile.NamedTemporaryFile(suffix=".hbm", delete=False) as tmp:
                hbm_path = tmp.name

            encoder = OSNetReIDEncoder(bpu_model_path=hbm_path)

        self.assertEqual(encoder.backend, "bpu")
        self.assertEqual(encoder.feature_dim, 512)

        crop = np.random.randint(0, 255, (256, 128, 3), dtype=np.uint8)
        feat = encoder.encode(crop)

        self.assertEqual(feat.shape, (512,))
        self.assertEqual(feat.dtype, np.float32)
        # L2 norm must be 1.0 (within floating point tolerance)
        self.assertAlmostEqual(float(np.linalg.norm(feat)), 1.0, places=5)


class TestPersonTrackerReID(unittest.TestCase):
    """W3-5: PersonTracker — adaptive threshold, stat counters, motion prediction."""

    def _make_tracker(self):
        """Build a PersonTracker with minimal deps mocked."""
        stubs = _stub_modules(
            "torch", "torchvision", "clip", "PIL", "PIL.Image",
            "open3d", "chromadb", "qp_perception",
            "qp_perception.reid", "qp_perception.reid.extractor",
            "qp_perception.tracking", "qp_perception.tracking.fusion",
            "qp_perception.selection", "qp_perception.selection.person_following",
        )
        sys.modules.pop("decision.vision.person_tracker", None)
        with patch.dict(sys.modules, stubs):
            from decision.vision.person_tracker import PersonTracker
        return PersonTracker()

    def test_person_tracker_adaptive_threshold_sparse(self):
        """<=2 candidates → threshold is 0.55."""
        tracker = self._make_tracker()
        threshold = tracker._adaptive_reid_threshold(2)
        self.assertAlmostEqual(threshold, 0.55, places=5)

    def test_person_tracker_adaptive_threshold_dense(self):
        """>=5 candidates → threshold is 0.70."""
        tracker = self._make_tracker()
        threshold = tracker._adaptive_reid_threshold(5)
        self.assertAlmostEqual(threshold, 0.70, places=5)

    def test_person_tracker_adaptive_threshold_interpolated(self):
        """3 candidates → threshold is linearly interpolated between 0.55 and 0.70."""
        tracker = self._make_tracker()
        threshold = tracker._adaptive_reid_threshold(3)
        # t = (3-2)/3 = 1/3; expected = 0.55 + 1/3 * 0.15 ≈ 0.60
        expected = 0.55 + (1.0 / 3.0) * (0.70 - 0.55)
        self.assertAlmostEqual(threshold, expected, places=5)

    def test_person_tracker_osnet_match_stat(self):
        """osnet_match counter increments when OSNet Re-ID finds a match."""
        tracker = self._make_tracker()

        # Inject a mock OSNet encoder that always returns a fixed 512-dim unit vector
        mock_osnet = MagicMock()
        fixed_feat = np.ones(512, dtype=np.float32) / math.sqrt(512)
        mock_osnet.encode.return_value = fixed_feat
        tracker._osnet_encoder = mock_osnet

        # Set up a locked target with matching osnet_feat
        from decision.vision.person_tracker import TrackedPerson
        tracker._person = TrackedPerson(
            position=[1.0, 1.0, 0.0],
            velocity=[0.0, 0.0],
            osnet_feat=fixed_feat.copy(),
            appearance=None,
        )

        # Candidate person far away (distance > MATCH_DIST_THRESHOLD) but matching appearance
        # Position is 5m away so distance matching fails and Re-ID is triggered
        candidates = [{
            "id": "unknown",
            "label": "person",
            "position": [6.0, 6.0, 0.0],
            "bbox": [100, 100, 200, 300],
            "confidence": 0.9,
        }]

        # Provide a fake rgb frame so _crop_person works
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)

        initial_count = tracker._reid_stats["osnet_match"]
        tracker._match_person(candidates, rgb)
        # osnet_match should have incremented (score = 1.0 > any threshold)
        self.assertGreater(
            tracker._reid_stats["osnet_match"], initial_count,
            "osnet_match counter should increment on successful OSNet Re-ID",
        )

    def test_person_tracker_motion_prediction(self):
        """Motion prediction advances position by velocity * dt."""
        tracker = self._make_tracker()

        from decision.vision.person_tracker import TrackedPerson
        tracker._person = TrackedPerson(
            position=[2.0, 3.0, 0.0],
            velocity=[1.0, -0.5],
        )

        dt = 0.3
        predicted = tracker._predict_position(dt=dt)

        self.assertAlmostEqual(predicted[0], 2.0 + 1.0 * dt, places=6)
        self.assertAlmostEqual(predicted[1], 3.0 + (-0.5) * dt, places=6)
        self.assertAlmostEqual(predicted[2], 0.0, places=6)

    def test_reid_stats_initialised(self):
        """_reid_stats dict exists with correct keys."""
        tracker = self._make_tracker()
        keys = {"osnet_match", "clip_fallback", "motion_dominant", "lost"}
        self.assertEqual(set(tracker._reid_stats.keys()), keys)
        for k in keys:
            self.assertEqual(tracker._reid_stats[k], 0)


# ---------------------------------------------------------------------------
# W3-4 — GainAutoTuner + BBoxNavigator persistence + skill wire
# ---------------------------------------------------------------------------

class TestGainAutoTuner(unittest.TestCase):
    """W3-4: GainAutoTuner — ZN math correctness and oscillation analysis."""

    def _make_tuner(self):
        # bbox_navigator imports runtime.config at module level; mock get_config
        stubs = _stub_modules("cv2")
        config_mock = MagicMock()
        config_mock.camera.T_camera_body = np.eye(4)
        with patch.dict(sys.modules, stubs):
            with patch("runtime.config.get_config", return_value=config_mock):
                sys.modules.pop("decision.vision.bbox_navigator", None)
                from decision.vision.bbox_navigator import GainAutoTuner
        return GainAutoTuner(relay_amplitude=0.3)

    def test_zn_math_on_known_values(self):
        """ZN formula: K_u = 4d/(pi*a_u), Kp = 0.6*K_u, Kd = Kp*T_u/8."""
        tuner = self._make_tuner()
        T_u = 2.0
        a_u = 0.5
        d = tuner.relay_amplitude  # 0.3

        K_u, Kp, Kd, converged = tuner.compute_zn_pd(T_u, a_u)

        expected_Ku = 4.0 * d / (math.pi * a_u)
        expected_Kp = 0.6 * expected_Ku
        expected_Kd = expected_Kp * T_u / 8.0

        self.assertAlmostEqual(K_u, expected_Ku, places=6)
        self.assertAlmostEqual(Kp, expected_Kp, places=6)
        self.assertAlmostEqual(Kd, expected_Kd, places=6)
        self.assertEqual(converged, 1.0)

    def test_zn_degenerate_returns_defaults(self):
        """T_u=0 or a_u=0 → converged=0.0, returns safe default gains."""
        tuner = self._make_tuner()
        _K_u, Kp, _Kd, converged = tuner.compute_zn_pd(0.0, 0.5)
        self.assertEqual(converged, 0.0)
        # Should return the DimOS default angular gain (1.5) or any safe value
        self.assertGreater(Kp, 0.0)

    def test_analyse_oscillation_detects_period(self):
        """Sinusoidal yaw series → T_u close to the true period."""
        tuner = self._make_tuner()
        true_T = 1.0   # 1-second oscillation
        dt = 0.02      # 50 Hz
        t = np.arange(0, 4.0, dt)   # 4 seconds → 4 full cycles
        yaw = np.sin(2 * math.pi / true_T * t)

        T_u, a_u = tuner.analyse_oscillation(yaw.tolist(), dt)

        # Period should be within 10% of true period
        self.assertGreater(T_u, 0.0, "T_u should be positive")
        self.assertAlmostEqual(T_u, true_T, delta=true_T * 0.10,
                               msg=f"Expected T_u ≈ {true_T}s, got {T_u:.4f}s")
        # Amplitude should be close to 1.0 (half peak-to-peak of sin)
        self.assertAlmostEqual(a_u, 1.0, delta=0.05)


class TestBBoxNavigatorGainPersistence(unittest.TestCase):
    """W3-4: BBoxNavigator gain save/load roundtrip using a temp file."""

    def _make_navigator(self, gains_path: Path):
        stubs = _stub_modules("cv2")
        config_mock = MagicMock()
        config_mock.camera.T_camera_body = np.eye(4)
        with patch.dict(sys.modules, stubs):
            with patch("runtime.config.get_config", return_value=config_mock):
                sys.modules.pop("decision.vision.bbox_navigator", None)
                from decision.vision.bbox_navigator import (
                    BBoxNavConfig,
                    BBoxNavigator,
                )
        nav = BBoxNavigator(
            config=BBoxNavConfig(linear_gain=0.8, angular_gain=1.5),
            robot_id="test_robot",
            gains_path=gains_path,
        )
        return nav

    def test_gain_file_save_and_load_roundtrip(self):
        """Persisted gains are reloaded correctly by a fresh BBoxNavigator instance."""
        with tempfile.TemporaryDirectory() as tmpdir:
            gains_path = Path(tmpdir) / "gains.json"

            stubs = _stub_modules("cv2")
            config_mock = MagicMock()
            config_mock.camera.T_camera_body = np.eye(4)

            with patch.dict(sys.modules, stubs):
                with patch("runtime.config.get_config", return_value=config_mock):
                    sys.modules.pop("decision.vision.bbox_navigator", None)
                    from decision.vision.bbox_navigator import (
                        BBoxNavConfig,
                        BBoxNavigator,
                    )

                    # First navigator: run tune_bbox_gains with synthetic oscillation
                    nav1 = BBoxNavigator(
                        config=BBoxNavConfig(linear_gain=0.8, angular_gain=1.5),
                        robot_id="test_robot",
                        gains_path=gains_path,
                    )

                    # Synthetic oscillation: sin at 1Hz, dt=0.02 → 4 full cycles
                    dt = 0.02
                    t = np.arange(0, 4.0, dt)
                    yaw_series = (0.5 * np.sin(2 * math.pi * t)).tolist()

                    report = nav1.tune_bbox_gains(yaw_series=yaw_series, dt=dt)
                    saved_angular_gain = nav1._cfg.angular_gain

                    # File must exist now
                    self.assertTrue(gains_path.exists(), "Gains file must be created after tuning")

                    # Second navigator: should load the persisted gains
                    nav2 = BBoxNavigator(
                        config=BBoxNavConfig(linear_gain=0.8, angular_gain=1.5),
                        robot_id="test_robot",
                        gains_path=gains_path,
                    )

            # If tuning converged, nav2 angular_gain must match nav1's
            if report.get("converged"):
                self.assertAlmostEqual(
                    nav2._cfg.angular_gain, saved_angular_gain, places=6,
                    msg="Loaded gain must match persisted value",
                )

    def test_missing_gains_file_uses_defaults(self):
        """No gains file → default gains kept and INFO logged."""
        with tempfile.TemporaryDirectory() as tmpdir:
            gains_path = Path(tmpdir) / "nonexistent" / "gains.json"

            stubs = _stub_modules("cv2")
            config_mock = MagicMock()
            config_mock.camera.T_camera_body = np.eye(4)

            with patch.dict(sys.modules, stubs):
                with patch("runtime.config.get_config", return_value=config_mock):
                    sys.modules.pop("decision.vision.bbox_navigator", None)
                    from decision.vision.bbox_navigator import (
                        BBoxNavConfig,
                        BBoxNavigator,
                    )
                    nav = BBoxNavigator(
                        config=BBoxNavConfig(linear_gain=0.8, angular_gain=1.5),
                        gains_path=gains_path,
                    )

            # Default gains must be preserved
            self.assertAlmostEqual(nav._cfg.linear_gain, 0.8, places=5)
            self.assertAlmostEqual(nav._cfg.angular_gain, 1.5, places=5)


class TestTuneBboxGainsSkillWired(unittest.TestCase):
    """W3-4: tune_bbox_gains skill is discoverable on VisualServoModule."""

    def test_tune_skill_is_registered_as_skill(self):
        """VisualServoModule.tune_bbox_gains must carry __skill__ or __rpc__ marker."""
        stubs = _stub_modules(
            "cv2", "torch", "torchvision", "clip", "PIL", "PIL.Image",
            "open3d", "chromadb", "rclpy", "rclpy.node", "rclpy.qos",
            "qp_perception", "qp_perception.reid",
            "qp_perception.reid.extractor",
            "qp_perception.tracking", "qp_perception.tracking.fusion",
            "qp_perception.selection", "qp_perception.selection.person_following",
        )
        config_mock = MagicMock()
        config_mock.camera.T_camera_body = np.eye(4)

        with patch.dict(sys.modules, stubs):
            with patch("runtime.config.get_config", return_value=config_mock):
                for mod_name in list(sys.modules.keys()):
                    if "visual_servo_module" in mod_name or "bbox_navigator" in mod_name:
                        sys.modules.pop(mod_name, None)
                from decision.modules.visual_servo_module import (
                    VisualServoModule,
                )

        method = getattr(VisualServoModule, "tune_bbox_gains", None)
        self.assertIsNotNone(method, "tune_bbox_gains must exist on VisualServoModule")
        # The @skill decorator sets __skill__ or __rpc__ attribute
        has_marker = getattr(method, "__skill__", False) or getattr(method, "__rpc__", False)
        self.assertTrue(
            has_marker,
            "tune_bbox_gains must be decorated with @skill (should have __skill__ or __rpc__)",
        )


if __name__ == "__main__":
    unittest.main()
