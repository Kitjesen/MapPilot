"""Tests for SLAM + GNSS global position anchoring in SlamBridgeModule.

Covers the ``_fuse_gnss_position`` stage of ``_fuse_odometry``:
  - port wiring
  - quality / freshness / covariance gating
  - one-shot alignment (mapâ†”ENU offset lock)
  - per-frame weighted blend (healthy vs. degraded SLAM)
  - RTK_FLOAT weight discount
  - preservation of Z and orientation
  - fusion disable switch

These tests do not start watchdog threads or DDS; they drive the fusion
path directly the way ``test_degeneracy_fusion.py`` does.
"""

from __future__ import annotations

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from runtime.msgs.geometry import Pose, Quaternion, Vector3
from runtime.msgs.gnss import GnssFixType, GnssOdom
from runtime.msgs.nav import Odometry


def _make_slam_odom(x: float = 0.0, y: float = 0.0, z: float = 0.0,
                    qw: float = 1.0) -> Odometry:
    return Odometry(
        pose=Pose(
            position=Vector3(x=x, y=y, z=z),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=qw),
        ),
        ts=time.time(),
    )


def _make_gnss_odom(e: float = 0.0, n: float = 0.0, u: float = 0.0,
                    fix_type: GnssFixType = GnssFixType.RTK_FIXED,
                    cov_e: float = 0.01, cov_n: float = 0.01) -> GnssOdom:
    return GnssOdom(
        east=e, north=n, up=u,
        cov_e=cov_e, cov_n=cov_n, cov_u=0.04,
        fix_type=fix_type,
        ts=time.time(),
    )


def _make_bridge(**kw):
    from localization.bridge import SlamBridgeModule
    defaults = {"odom_timeout": 0.1, "cloud_timeout": 0.2, "watchdog_hz": 20}
    defaults.update(kw)
    return SlamBridgeModule(**defaults)


def _prime_healthy(m) -> None:
    """Put the bridge in the state where GNSS alignment is allowed to lock."""
    from localization.bridge import DEGEN_NONE, LOC_TRACKING
    m._loc_state = LOC_TRACKING
    m._degen_level = DEGEN_NONE


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# Port / config surface
# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class TestGnssPortSurface(unittest.TestCase):

    def test_gnss_odom_port_exists(self):
        m = _make_bridge()
        self.assertIn("gnss_odom", m.ports_in)

    def test_gnss_fusion_defaults(self):
        m = _make_bridge()
        self.assertTrue(m._gnss_fusion)
        self.assertIsNone(m._last_gnss_odom)
        self.assertIsNone(m._gnss_map_offset)
        self.assertEqual(m._gnss_fused_count, 0)

    def test_gnss_fusion_disabled_switch(self):
        m = _make_bridge(gnss_fusion=False)
        self.assertFalse(m._gnss_fusion)


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# Quality / freshness gating
# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class TestGnssGating(unittest.TestCase):

    def test_no_gnss_passthrough(self):
        m = _make_bridge()
        m.setup()
        _prime_healthy(m)

        slam = _make_slam_odom(x=3.0, y=4.0, z=0.5)
        fused = m._fuse_odometry(slam)
        self.assertAlmostEqual(fused.pose.position.x, 3.0)
        self.assertAlmostEqual(fused.pose.position.y, 4.0)
        self.assertAlmostEqual(fused.pose.position.z, 0.5)
        self.assertIsNone(m._gnss_map_offset)

    def test_single_fix_ignored(self):
        m = _make_bridge()
        m.setup()
        _prime_healthy(m)

        m._on_gnss_odom(_make_gnss_odom(e=100.0, n=200.0,
                                         fix_type=GnssFixType.SINGLE))
        fused = m._fuse_odometry(_make_slam_odom(x=1.0, y=2.0))
        # No alignment, no blend
        self.assertIsNone(m._gnss_map_offset)
        self.assertAlmostEqual(fused.pose.position.x, 1.0)
        self.assertAlmostEqual(fused.pose.position.y, 2.0)

    def test_dgps_ignored(self):
        m = _make_bridge()
        m.setup()
        _prime_healthy(m)

        m._on_gnss_odom(_make_gnss_odom(fix_type=GnssFixType.DGPS))
        fused = m._fuse_odometry(_make_slam_odom(x=1.0))
        self.assertIsNone(m._gnss_map_offset)
        self.assertAlmostEqual(fused.pose.position.x, 1.0)

    def test_stale_gnss_ignored(self):
        m = _make_bridge(gnss_max_age_s=0.5)
        m.setup()
        _prime_healthy(m)

        m._on_gnss_odom(_make_gnss_odom(e=50.0, n=60.0))
        # Force last receive timestamp into the past
        m._last_gnss_rx_ts = time.time() - 5.0

        fused = m._fuse_odometry(_make_slam_odom(x=1.0, y=2.0))
        self.assertIsNone(m._gnss_map_offset)
        self.assertAlmostEqual(fused.pose.position.x, 1.0)

    def test_high_covariance_ignored(self):
        m = _make_bridge(gnss_max_std_m=0.5)
        m.setup()
        _prime_healthy(m)

        # cov_e + cov_n = 2.0 â†?h_std = 1.0, exceeds 0.5 limit
        m._on_gnss_odom(_make_gnss_odom(e=10.0, n=10.0,
                                         cov_e=1.0, cov_n=1.0))
        fused = m._fuse_odometry(_make_slam_odom(x=1.0, y=2.0))
        self.assertIsNone(m._gnss_map_offset)
        self.assertAlmostEqual(fused.pose.position.x, 1.0)


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# Alignment (one-shot offset lock)
# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class TestGnssAlignment(unittest.TestCase):

    def test_first_rtk_fixed_locks_offset(self):
        m = _make_bridge()
        m.setup()
        _prime_healthy(m)

        # SLAM at (10, 20), GNSS ENU at (3, 4) â†?offset = (7, 16)
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        fused = m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0))

        self.assertIsNotNone(m._gnss_map_offset)
        self.assertAlmostEqual(m._gnss_map_offset[0], 7.0)
        self.assertAlmostEqual(m._gnss_map_offset[1], 16.0)
        # First frame: pose not modified â€?only locks
        self.assertAlmostEqual(fused.pose.position.x, 10.0)
        self.assertAlmostEqual(fused.pose.position.y, 20.0)

    def test_no_alignment_when_slam_unhealthy(self):
        """Uninitialised / lost SLAM must not lock bad offset."""
        from localization.bridge import LOC_UNINIT
        m = _make_bridge()
        m.setup()
        m._loc_state = LOC_UNINIT

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0))
        self.assertIsNone(m._gnss_map_offset)

    def test_no_alignment_when_slam_degenerate(self):
        from localization.bridge import DEGEN_CRITICAL, LOC_TRACKING
        m = _make_bridge()
        m.setup()
        m._loc_state = LOC_TRACKING
        m._degen_level = DEGEN_CRITICAL

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0))
        self.assertIsNone(m._gnss_map_offset)


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# Weighted blend
# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class TestGnssBlend(unittest.TestCase):

    def _lock_and_drift(self, m, lock_slam_xy=(10.0, 20.0),
                        lock_gnss_en=(3.0, 4.0)):
        """Lock alignment at healthy SLAM pose, return True when done."""
        _prime_healthy(m)
        m._on_gnss_odom(_make_gnss_odom(e=lock_gnss_en[0], n=lock_gnss_en[1]))
        m._fuse_odometry(_make_slam_odom(x=lock_slam_xy[0], y=lock_slam_xy[1]))
        self.assertIsNotNone(m._gnss_map_offset)

    def test_healthy_blend_small_alpha(self):
        m = _make_bridge(gnss_alpha_healthy=0.05)
        m.setup()
        self._lock_and_drift(m)  # offset = (7, 16)

        # Now SLAM drifts to (20, 30); GNSS still at (3, 4) â†?in map (10, 20)
        # Expected fused XY: (20*0.95 + 10*0.05, 30*0.95 + 20*0.05)
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        fused = m._fuse_odometry(_make_slam_odom(x=20.0, y=30.0))

        self.assertAlmostEqual(fused.pose.position.x, 20.0 * 0.95 + 10.0 * 0.05)
        self.assertAlmostEqual(fused.pose.position.y, 30.0 * 0.95 + 20.0 * 0.05)

    def test_degraded_blend_large_alpha(self):
        from localization.bridge import DEGEN_CRITICAL
        m = _make_bridge(gnss_alpha_healthy=0.05, gnss_alpha_degraded=0.5)
        m.setup()
        self._lock_and_drift(m)

        # Degrade SLAM state
        m._degen_level = DEGEN_CRITICAL

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        fused = m._fuse_odometry(_make_slam_odom(x=20.0, y=30.0))

        # With Î±=0.5, fused = slam*0.5 + gnss_in_map*0.5
        self.assertAlmostEqual(fused.pose.position.x, 0.5 * 20.0 + 0.5 * 10.0)
        self.assertAlmostEqual(fused.pose.position.y, 0.5 * 30.0 + 0.5 * 20.0)

    def test_rtk_float_weight_discount(self):
        m = _make_bridge(gnss_alpha_healthy=0.1, gnss_rtk_float_scale=0.3)
        m.setup()
        self._lock_and_drift(m)

        # RTK_FLOAT sample â†?effective Î± = 0.1 * 0.3 = 0.03
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0,
                                         fix_type=GnssFixType.RTK_FLOAT))
        fused = m._fuse_odometry(_make_slam_odom(x=20.0, y=30.0))

        eff = 0.03
        self.assertAlmostEqual(fused.pose.position.x,
                                20.0 * (1 - eff) + 10.0 * eff)

    def test_z_untouched(self):
        m = _make_bridge()
        m.setup()
        self._lock_and_drift(m)

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0, u=999.0))
        fused = m._fuse_odometry(_make_slam_odom(x=20.0, y=30.0, z=1.5))
        # Z must stay with SLAM, GNSS up=999 must NOT leak in
        self.assertAlmostEqual(fused.pose.position.z, 1.5)

    def test_orientation_untouched(self):
        m = _make_bridge()
        m.setup()
        self._lock_and_drift(m)

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        slam = Odometry(
            pose=Pose(
                position=Vector3(x=20.0, y=30.0, z=0.0),
                orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9),
            ),
            ts=time.time(),
        )
        fused = m._fuse_odometry(slam)

        self.assertAlmostEqual(fused.pose.orientation.x, 0.1)
        self.assertAlmostEqual(fused.pose.orientation.y, 0.2)
        self.assertAlmostEqual(fused.pose.orientation.z, 0.3)
        self.assertAlmostEqual(fused.pose.orientation.w, 0.9)

    def test_fused_count_increments(self):
        m = _make_bridge()
        m.setup()
        self._lock_and_drift(m)
        self.assertEqual(m._gnss_fused_count, 0)

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(_make_slam_odom(x=20.0, y=30.0))
        self.assertEqual(m._gnss_fused_count, 1)

        m._on_gnss_odom(_make_gnss_odom(e=3.5, n=4.5))
        m._fuse_odometry(_make_slam_odom(x=21.0, y=31.0))
        self.assertEqual(m._gnss_fused_count, 2)


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# Disable switch
# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class TestGnssDisabled(unittest.TestCase):

    def test_disabled_bypasses_all_gnss_logic(self):
        m = _make_bridge(gnss_fusion=False)
        m.setup()
        _prime_healthy(m)

        # Even with perfectly valid RTK data, nothing should happen
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        fused = m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0))

        self.assertIsNone(m._gnss_map_offset)
        self.assertEqual(m._gnss_fused_count, 0)
        self.assertAlmostEqual(fused.pose.position.x, 10.0)
        self.assertAlmostEqual(fused.pose.position.y, 20.0)


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# Residual guard / re-alignment
# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class TestResidualGuard(unittest.TestCase):
    """Sustained large residual should trigger re-locking of the offset.

    We shortcut time by mutating the ring-buffer entries â€?this makes tests
    deterministic instead of depending on wall-clock.
    """

    def _make_locked(self, **kw):
        m = _make_bridge(**kw)
        m.setup()
        _prime_healthy(m)
        # Lock at slam=(0,0), gnss=(0,0) â†?offset = (0, 0)
        m._on_gnss_odom(_make_gnss_odom(e=0.0, n=0.0))
        m._fuse_odometry(_make_slam_odom(x=0.0, y=0.0))
        assert m._gnss_map_offset is not None
        return m

    def test_residual_tracked_on_each_frame(self):
        m = self._make_locked()
        # SLAM drifts 2m from GNSS-aligned position
        m._on_gnss_odom(_make_gnss_odom(e=0.0, n=0.0))
        m._fuse_odometry(_make_slam_odom(x=2.0, y=0.0))
        self.assertAlmostEqual(m._gnss_last_residual_m, 2.0, places=3)

    def test_short_window_does_not_relock(self):
        m = self._make_locked(gnss_residual_warn_m=1.0,
                               gnss_residual_warn_duration_s=10.0)
        # Only a few frames with large residual â†?not enough window time
        for _ in range(3):
            m._on_gnss_odom(_make_gnss_odom(e=0.0, n=0.0))
            m._fuse_odometry(_make_slam_odom(x=10.0, y=0.0))
        self.assertIsNotNone(m._gnss_map_offset)
        self.assertEqual(m._gnss_relock_count, 0)

    def test_sustained_residual_triggers_relock(self):
        m = self._make_locked(
            gnss_residual_warn_m=1.0,
            gnss_residual_warn_duration_s=10.0,
            gnss_residual_warn_ratio=0.7,
        )
        # Drive residual=10m across 10 frames; then backdate history so the
        # window is "old enough" to fire (older than 5s = half window).
        for _ in range(10):
            m._on_gnss_odom(_make_gnss_odom(e=0.0, n=0.0))
            m._fuse_odometry(_make_slam_odom(x=10.0, y=0.0))

        now = time.time()
        m._gnss_residual_history = [
            (now - 7.0 + i * 0.5, 10.0) for i in range(10)
        ]

        # Next frame: guard checks and should fire
        m._on_gnss_odom(_make_gnss_odom(e=0.0, n=0.0))
        m._fuse_odometry(_make_slam_odom(x=10.0, y=0.0))

        self.assertIsNone(m._gnss_map_offset)
        self.assertEqual(m._gnss_relock_count, 1)

    def test_relock_allows_fresh_lock(self):
        m = self._make_locked(
            gnss_residual_warn_m=1.0,
            gnss_residual_warn_duration_s=10.0,
        )
        # Force relock
        now = time.time()
        m._gnss_residual_history = [
            (now - 7.0 + i * 0.5, 10.0) for i in range(10)
        ]
        m._on_gnss_odom(_make_gnss_odom(e=0.0, n=0.0))
        m._fuse_odometry(_make_slam_odom(x=10.0, y=0.0))
        self.assertIsNone(m._gnss_map_offset)

        # Next healthy GNSS should re-lock with new offset
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(_make_slam_odom(x=13.0, y=14.0))
        self.assertIsNotNone(m._gnss_map_offset)
        self.assertAlmostEqual(m._gnss_map_offset[0], 10.0)
        self.assertAlmostEqual(m._gnss_map_offset[1], 10.0)


class TestAntennaOffset(unittest.TestCase):
    """The antenna reports its own ENU position, not the body's. When the
    robot rotates, the lever-arm offset couples with orientation and
    introduces apparent horizontal motion that must be subtracted before
    fusing into the body-centric SLAM pose.
    """

    def test_default_is_zero_offset(self):
        m = _make_bridge()
        self.assertTrue((m._gnss_antenna_offset == 0.0).all())

    def test_invalid_offset_shape_rejected(self):
        with self.assertRaises(ValueError):
            _make_bridge(gnss_antenna_offset=(0.1, 0.2))

    def test_offset_lockframe_at_zero_yaw(self):
        """At yaw=0 with antenna at (0.5, 0, 0) body, the antenna is 0.5m
        ahead of body in +X map direction. Lock should record the offset
        assuming GNSS reports antenna pos."""
        m = _make_bridge(gnss_antenna_offset=(0.5, 0.0, 0.0))
        m.setup()
        _prime_healthy(m)

        # SLAM says body is at (10, 20). With yaw=0, antenna is at (10.5, 20).
        # If GNSS reports antenna=(3, 4), the body-equivalent GNSS is
        # (3 - 0.5, 4 - 0.0) = (2.5, 4.0). Offset = (10 - 2.5, 20 - 4) = (7.5, 16).
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0, qw=1.0))

        self.assertIsNotNone(m._gnss_map_offset)
        self.assertAlmostEqual(m._gnss_map_offset[0], 7.5, places=5)
        self.assertAlmostEqual(m._gnss_map_offset[1], 16.0, places=5)

    def test_offset_rotates_with_yaw(self):
        """Rotate robot 90Â° CCW (yaw=+Ï€/2). Antenna at (0.5, 0, 0) body
        now points in +Y direction of map. So antenna_map_offset = (0, 0.5).
        """
        import math

        from runtime.msgs.geometry import Quaternion

        m = _make_bridge(gnss_antenna_offset=(0.5, 0.0, 0.0))
        m.setup()
        _prime_healthy(m)

        q = Quaternion.from_yaw(math.pi / 2)
        slam = Odometry(
            pose=Pose(
                position=Vector3(x=10.0, y=20.0, z=0.0),
                orientation=q,
            ),
            ts=time.time(),
        )

        # Body at (10, 20), yaw=90Â° â†?antenna at (10, 20.5) in map
        # GNSS reports antenna (3, 4) â†?body-equivalent (3 - 0, 4 - 0.5) = (3, 3.5)
        # Offset = (10 - 3, 20 - 3.5) = (7, 16.5)
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(slam)

        self.assertIsNotNone(m._gnss_map_offset)
        self.assertAlmostEqual(m._gnss_map_offset[0], 7.0, places=5)
        self.assertAlmostEqual(m._gnss_map_offset[1], 16.5, places=5)

    def test_zero_offset_fast_path(self):
        """When antenna offset is (0,0,0) we skip rotation math â€?result
        should match the pre-offset implementation exactly."""
        m = _make_bridge(gnss_antenna_offset=(0.0, 0.0, 0.0))
        m.setup()
        _prime_healthy(m)

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0))

        self.assertAlmostEqual(m._gnss_map_offset[0], 7.0)
        self.assertAlmostEqual(m._gnss_map_offset[1], 16.0)


class TestGnssFusionHealthPort(unittest.TestCase):

    def test_health_port_exists(self):
        m = _make_bridge()
        self.assertIn("gnss_fusion_health", m.ports_out)

    def test_health_snapshot_schema(self):
        m = _make_bridge()
        m.setup()
        _prime_healthy(m)

        received: list[dict] = []
        m.gnss_fusion_health._add_callback(received.append)

        # First call: lock offset, no snapshot yet because we return before publish
        m._on_gnss_odom(_make_gnss_odom(e=0.0, n=0.0))
        m._fuse_odometry(_make_slam_odom(x=0.0, y=0.0))

        # Second call: publish path exercised
        m._on_gnss_odom(_make_gnss_odom(e=0.0, n=0.0))
        m._fuse_odometry(_make_slam_odom(x=1.0, y=0.0))

        self.assertGreater(len(received), 0)
        snap = received[-1]
        for key in (
            "enabled", "alignment_locked", "map_offset",
            "last_residual_m", "fused_count", "relock_count",
            "last_relock_ts", "last_fix_type", "last_gnss_age_s",
        ):
            self.assertIn(key, snap, f"missing key: {key}")
        self.assertTrue(snap["enabled"])
        self.assertTrue(snap["alignment_locked"])
        self.assertEqual(snap["last_fix_type"], "RTK_FIXED")
        self.assertGreaterEqual(snap["fused_count"], 1)
        self.assertEqual(snap["relock_count"], 0)


class TestE2EAutoWire(unittest.TestCase):
    """End-to-end contract: `autoconnect(GnssModule, SlamBridgeModule)` must
    auto-wire ``GnssModule.gnss_odom`` â†?``SlamBridgeModule.gnss_odom`` by
    matching (port_name, msg_type). This protects against someone renaming a
    port and silently breaking drift anchoring on real hardware.
    """

    def _build_system(self):
        from runtime.blueprint import autoconnect
        from localization.gnss_module import GnssModule
        from localization.bridge import SlamBridgeModule

        bp = autoconnect(
            GnssModule.blueprint(origin_lat=31.0, origin_lon=121.0,
                                  origin_alt=0.0,
                                  auto_init_origin=False,
                                  min_sat_used=0,  # tests skip sat count
                                  max_hdop=99.0,   # tests skip HDOP
                                  status_rate_hz=0.0),
            SlamBridgeModule.blueprint(
                odom_timeout=0.1, cloud_timeout=0.2, watchdog_hz=20),
        )
        return bp.build()

    def test_autoconnect_wires_gnss_to_slam_bridge(self):
        from runtime.msgs.gnss import GnssFix, GnssFixType

        system = self._build_system()
        system.start()   # runs setup(), which subscribes the bridge to gnss_odom
        try:
            gnss = system.modules["GnssModule"]
            bridge = system.modules["SlamBridgeModule"]

            # Fresh bridge: no GNSS received yet
            self.assertIsNone(bridge._last_gnss_odom)

            # Inject a fix that passes the GNSS quality gate
            fix = GnssFix(
                lat=31.00001, lon=121.00001, alt=10.0,
                fix_type=GnssFixType.RTK_FIXED,
                covariance=(0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.04),
                num_sat=12, num_sat_used=12,
                ts=time.time(),
            )
            gnss.inject_fix(fix)

            # After a single fix, SlamBridgeModule should have received the
            # derived GnssOdom via the auto-wired port.
            self.assertIsNotNone(bridge._last_gnss_odom)
            self.assertGreater(bridge._last_gnss_rx_ts, 0.0)
            self.assertEqual(
                bridge._last_gnss_odom.fix_type, GnssFixType.RTK_FIXED
            )
        finally:
            try:
                system.stop()
            except Exception:
                pass

    def test_end_to_end_fusion_with_lock(self):
        """Inject GNSS, inject SLAM odom through fusion, verify offset locks."""
        from runtime.msgs.gnss import GnssFix, GnssFixType
        from localization.bridge import DEGEN_NONE, LOC_TRACKING

        system = self._build_system()
        system.start()
        try:
            gnss = system.modules["GnssModule"]
            bridge = system.modules["SlamBridgeModule"]
            # Prime health state so the lock is permitted
            bridge._loc_state = LOC_TRACKING
            bridge._degen_level = DEGEN_NONE

            # Inject a good RTK fix â€?autoconnect pipes it into the bridge
            fix = GnssFix(
                lat=31.0, lon=121.0, alt=0.0,
                fix_type=GnssFixType.RTK_FIXED,
                covariance=(0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.04),
                num_sat=12, num_sat_used=12,
                ts=time.time(),
            )
            gnss.inject_fix(fix)

            # Drive a SLAM odometry sample through the fusion â€?this locks
            # the offset (first-frame behaviour).
            slam = _make_slam_odom(x=5.0, y=7.0)
            fused = bridge._fuse_odometry(slam)

            self.assertIsNotNone(bridge._gnss_map_offset)
            # First frame must not alter the pose
            self.assertAlmostEqual(fused.pose.position.x, 5.0)
            self.assertAlmostEqual(fused.pose.position.y, 7.0)
        finally:
            try:
                system.stop()
            except Exception:
                pass


class TestGnssSkillMethods(unittest.TestCase):
    """@skill methods on SlamBridgeModule â€?exposed to MCP / REPL."""

    def test_get_gnss_fusion_status_returns_json(self):
        import json
        m = _make_bridge()
        m.setup()
        _prime_healthy(m)
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0))

        s = m.get_gnss_fusion_status()
        parsed = json.loads(s)
        self.assertTrue(parsed["enabled"])
        self.assertTrue(parsed["alignment_locked"])
        self.assertEqual(parsed["last_fix_type"], "RTK_FIXED")
        self.assertIn("map_offset", parsed)
        self.assertIn("antenna_offset_body", parsed)

    def test_relock_clears_offset(self):
        import json
        m = _make_bridge()
        m.setup()
        _prime_healthy(m)
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0))
        self.assertIsNotNone(m._gnss_map_offset)
        prev_count = m._gnss_relock_count

        s = m.relock_gnss_alignment()
        parsed = json.loads(s)
        self.assertEqual(parsed["status"], "relocked")
        self.assertIsNone(m._gnss_map_offset)
        self.assertEqual(m._gnss_relock_count, prev_count + 1)

    def test_set_gnss_fusion_toggles_and_clears_offset(self):
        import json
        m = _make_bridge()
        m.setup()
        _prime_healthy(m)
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0))
        self.assertTrue(m._gnss_fusion)
        self.assertIsNotNone(m._gnss_map_offset)

        s = m.set_gnss_fusion(False)
        parsed = json.loads(s)
        self.assertEqual(parsed["status"], "ok")
        self.assertFalse(parsed["enabled"])
        self.assertFalse(m._gnss_fusion)
        # Disabling also drops the stale offset
        self.assertIsNone(m._gnss_map_offset)

        # Re-enable works
        m.set_gnss_fusion(True)
        self.assertTrue(m._gnss_fusion)

    def test_skills_are_discoverable_by_mcp(self):
        """@skill decorator must flag methods so MCP auto-discovery picks
        them up via Module.get_skill_infos()."""
        m = _make_bridge()
        skills = {info.func_name for info in m.get_skill_infos()}
        self.assertIn("get_gnss_fusion_status", skills)
        self.assertIn("relock_gnss_alignment", skills)
        self.assertIn("set_gnss_fusion", skills)


if __name__ == "__main__":
    unittest.main()
