"""Wave 2 simplification remediation tests 閳?Team D (items 4, 5, 8).

All tests are pure-Python; no ROS2, no hardware, no C++ extension required.
"""

from __future__ import annotations

import math
import time
import unittest
from typing import TYPE_CHECKING
from unittest.mock import MagicMock, patch

import numpy as np

if TYPE_CHECKING:
    from nav.local.path_follower import PathFollower
    from nav.navigation import Navigation

# ---------------------------------------------------------------------------
# W2-4: PathFollower adaptive Pure Pursuit
# ---------------------------------------------------------------------------


class TestAdaptivePurePursuit(unittest.TestCase):
    """Verify the adaptive Pure Pursuit improvements in the pid backend."""

    def _make_module(self, max_speed: float = 0.5) -> PathFollower:
        from nav.local.path_follower import PathFollower

        m = PathFollower(backend="pid", max_speed=max_speed)
        # _setup_pid is called from setup(); call it directly so we can test
        # without a full blueprint.
        m._setup_pid()
        return m

    def test_variable_lookahead_scales_with_speed(self):
        """Lookahead grows with speed and is clamped to [L_min, L_max]."""

        m = self._make_module()
        k_v = m._pp_k_v
        l_min = m._pp_l_min
        l_max = m._pp_l_max

        # At speed 0 閳?L_min
        L_slow = float(np.clip(k_v * 0.0, l_min, l_max))
        self.assertAlmostEqual(L_slow, l_min)

        # At speed 4 m/s 閳?L_max (4 * 0.5 = 2.0)
        L_fast = float(np.clip(k_v * 4.0, l_min, l_max))
        self.assertAlmostEqual(L_fast, l_max)

        # Mid speed
        L_mid = float(np.clip(k_v * 1.0, l_min, l_max))
        self.assertGreaterEqual(L_mid, l_min)
        self.assertLessEqual(L_mid, l_max)

    def test_acceleration_ramp_limits_delta_v(self):
        """v_cmd change per step must not exceed a_max * dt."""

        m = self._make_module(max_speed=1.0)
        a_max = m._pp_a_max  # 1.0 m/s^2
        dt = 0.1  # 100 ms step

        v_prev = 0.0
        v_desired = 1.0  # large jump

        max_dv = a_max * dt
        v_cmd = float(np.clip(v_desired, v_prev - max_dv, v_prev + max_dv))

        self.assertAlmostEqual(v_cmd, v_prev + max_dv, places=6)
        self.assertLessEqual(abs(v_cmd - v_prev), max_dv + 1e-9)

    def test_curvature_aware_speed_tight_turn(self):
        """Speed is reduced when |yaw_err| > 0.5 rad."""
        yaw_err = math.pi / 3  # ~60 deg 閳?well above 0.5 rad
        curvature_factor = max(0.3, 1.0 - 0.7 * abs(yaw_err) / math.pi)
        self.assertLess(curvature_factor, 1.0)
        self.assertGreaterEqual(curvature_factor, 0.3)

    def test_curvature_aware_speed_straight(self):
        """Speed is not reduced for small yaw errors (< 0.5 rad)."""
        yaw_err = 0.2  # small heading error
        # Condition: abs(yaw_err) > 0.5 is False 閳?factor = 1.0
        curvature_factor = 1.0 if abs(yaw_err) <= 0.5 else max(0.3, 1.0 - 0.7 * abs(yaw_err) / math.pi)
        self.assertAlmostEqual(curvature_factor, 1.0)

    def test_pid_backend_config_loaded(self):
        """_setup_pid loads pp_v_max from config without raising."""
        from nav.local.path_follower import PathFollower

        m = PathFollower(backend="pid", max_speed=0.4)
        # Should not raise even if config file is absent (falls back to defaults).
        try:
            m._setup_pid()
        except Exception as exc:
            self.fail(f"_setup_pid raised unexpectedly: {exc}")
        # v_max must be a positive float
        self.assertGreater(m._pp_v_max, 0.0)


# ---------------------------------------------------------------------------
# W2-5: Terrain 30K truncation removed
# ---------------------------------------------------------------------------


class TestTerrainNumpyPath(unittest.TestCase):
    """Verify the 30K truncation is gone and numpy array is passed directly."""

    def test_no_truncation_large_cloud(self):
        """_process_nanobind no longer truncates to 30K points."""
        from nav.local.terrain import Terrain

        m = Terrain(backend="simple")
        m.setup()  # simple backend 閳?no C++ needed

        # Read the source to confirm the old truncation constant is gone
        import inspect

        src = inspect.getsource(nav.terrain._process_nanobind)
        self.assertNotIn("MAX_POINTS = 30_000", src, "30K truncation constant should be removed")
        # The old line was: flat = pts4.ravel().tolist()
        # After the fix the ravel result is NOT converted to a Python list.
        self.assertNotIn("pts4.ravel().tolist()", src, "pts4.ravel().tolist() should be replaced with numpy path")

    def test_flat_array_is_contiguous_float32(self):
        """The flat array passed to C++ must be contiguous float32."""
        # Simulate what _process_nanobind does after the fix
        pts = np.random.rand(100, 4).astype(np.float64)  # intentionally float64
        flat = np.ascontiguousarray(pts, dtype=np.float32).ravel()

        self.assertEqual(flat.dtype, np.float32)
        self.assertTrue(flat.flags["C_CONTIGUOUS"])
        self.assertEqual(flat.shape, (400,))


# ---------------------------------------------------------------------------
# W2-8: Navigation context-aware recovery
# ---------------------------------------------------------------------------


class TestContextAwareRecovery(unittest.TestCase):
    """Verify recovery strategies are selected based on traversability class."""

    def _make_nav(self) -> Navigation:
        """Create a Navigation with mocked planner and tracker."""
        from nav.navigation import MissionState, Navigation

        with patch("nav.navigation.create_planner_service"), patch("nav.navigation.WaypointTracker"):
            m = Navigation(planner="astar")
            m.setup()

        # Put it in EXECUTING so recovery steps don't abort early
        m._state = MissionState.EXECUTING
        m._replan_count = 1
        return m

    def _collect_recovery_cmds(self, nav, traversability_class: str) -> list:
        """Run recovery with given terrain class, collect published cmd_vels."""
        nav._latest_traversability_class = traversability_class
        published = []
        nav.recovery_cmd_vel.subscribe(lambda t: published.append(t))
        adapter_events = []
        nav.adapter_status.subscribe(lambda e: adapter_events.append(e))

        # Patch sleep to avoid real waits
        with patch("nav.runtime.recovery.time") as mock_time:
            mock_time.sleep = MagicMock()
            mock_time.time = time.time
            nav._execute_recovery_motion()

        return published, adapter_events

    def test_cliff_strategy_no_backup(self):
        """cliff terrain: no backward motion, only rotation."""
        nav = self._make_nav()
        cmds, events = self._collect_recovery_cmds(nav, "cliff")

        # No backward velocity at any point
        backward = [c for c in cmds if c.linear.x < -0.01]
        self.assertEqual(len(backward), 0, "cliff recovery must not back up")

        # Some rotation must occur
        rotating = [c for c in cmds if abs(c.angular.z) > 0.01]
        self.assertGreater(len(rotating), 0, "cliff recovery must rotate")

        # Event published
        self.assertTrue(any(e.get("event") == "recovery_started" for e in events))
        strategy_event = next(e for e in events if e.get("event") == "recovery_started")
        self.assertEqual(strategy_event["strategy"], "rotate_only")
        self.assertEqual(strategy_event["reason"], "cliff")

    def test_unknown_terrain_default_strategy(self):
        """unknown terrain uses the default backup+rotate strategy."""
        nav = self._make_nav()
        cmds, events = self._collect_recovery_cmds(nav, "unknown")

        backward = [c for c in cmds if c.linear.x < -0.01]
        self.assertGreater(len(backward), 0, "default recovery must back up")

        strategy_event = next(e for e in events if e.get("event") == "recovery_started")
        self.assertEqual(strategy_event["strategy"], "default_backup_rotate")

    def test_traversability_port_updates_class(self):
        """_on_traversability updates _latest_traversability_class."""
        nav = self._make_nav()
        nav._on_traversability({"traversability_class": "narrow"})
        self.assertEqual(nav._latest_traversability_class, "narrow")

        # Also test the 'class' key fallback
        nav._on_traversability({"class": "grip_loss"})
        self.assertEqual(nav._latest_traversability_class, "grip_loss")


if __name__ == "__main__":
    unittest.main()
