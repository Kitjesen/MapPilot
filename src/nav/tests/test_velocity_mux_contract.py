"""Contract tests for VelocityMux priority arbitration order, timeout fallthrough,
freeze/unfreeze, and health reporting.

All tests are pure-Python, no ROS2 / hardware / MuJoCo required.
"""

from __future__ import annotations

import time
import unittest

from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.stream import In, Out


class TestVelocityMuxContract(unittest.TestCase):
    """Contract tests for nav.velocity_mux."""

    def _make(self, source_timeout: float = 0.5):
        from nav.services.safety.velocity_mux import VelocityMux
        return VelocityMux(source_timeout=source_timeout)

    @staticmethod
    def _twist(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> Twist:
        return Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, wz))

    @staticmethod
    def _odom(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> Odometry:
        return Odometry(
            pose=Pose(
                position=Vector3(x, y, 0.0),
                orientation=Quaternion.from_yaw(yaw),
            )
        )

    @staticmethod
    def _costmap(cost: float = 0.0) -> dict:
        grid = [[0.0 for _ in range(5)] for _ in range(5)]
        grid[2][3] = cost
        return {"grid": grid, "resolution": 1.0, "origin": [-2.0, -2.0]}

    def _make_collision_mux(self, **kwargs):
        from nav.services.safety.velocity_mux import VelocityMux

        mux = VelocityMux(
            source_timeout=5.0,
            enable_collision_monitor=True,
            collision_monitor_horizon_s=1.0,
            collision_monitor_step_s=0.5,
            **kwargs,
        )
        mux.setup()
        return mux

    def test_ports_in(self):
        """Must declare input ports for all 4 velocity sources."""
        m = self._make()
        self.assertIn("teleop_cmd_vel", m.ports_in)
        self.assertIn("visual_servo_cmd_vel", m.ports_in)
        self.assertIn("recovery_cmd_vel", m.ports_in)
        self.assertIn("path_follower_cmd_vel", m.ports_in)
        self.assertIn("collision_odometry", m.ports_in)
        self.assertIn("collision_costmap", m.ports_in)
        for port_name in (
            "teleop_cmd_vel",
            "visual_servo_cmd_vel",
            "recovery_cmd_vel",
            "path_follower_cmd_vel",
            "collision_odometry",
            "collision_costmap",
        ):
            self.assertIsInstance(m.ports_in[port_name], In)

    def test_ports_out(self):
        """Must declare output ports for driver cmd_vel and active source."""
        m = self._make()
        self.assertIn("driver_cmd_vel", m.ports_out)
        self.assertIn("active_source", m.ports_out)
        self.assertIsInstance(m.ports_out["driver_cmd_vel"], Out)
        self.assertIsInstance(m.ports_out["active_source"], Out)

    def test_layer_is_0(self):
        """VelocityMux must be layer 0."""
        m = self._make()
        self.assertEqual(m.layer, 0)

    def test_initial_source_priorities(self):
        """Priority order must be: teleop > visual_servo > recovery > path_follower."""
        m = self._make()
        self.assertGreater(
            m._sources["teleop"]["priority"],
            m._sources["visual_servo"]["priority"],
        )
        self.assertGreater(
            m._sources["visual_servo"]["priority"],
            m._sources["recovery"]["priority"],
        )
        self.assertGreater(
            m._sources["recovery"]["priority"],
            m._sources["path_follower"]["priority"],
        )

    def test_highest_priority_source_wins(self):
        """When multiple sources are active, the highest priority must win."""
        m = self._make(source_timeout=5.0)
        m.setup()
        active_sources = []
        m.active_source._add_callback(active_sources.append)
        driver_twists = []
        m.driver_cmd_vel._add_callback(driver_twists.append)

        # Publish path_follower first (lowest priority)
        m._on_source("path_follower", self._twist(vx=0.5))
        # Then recovery (medium priority)
        m._on_source("recovery", self._twist(vx=0.3))
        # Then teleop (highest priority)
        m._on_source("teleop", self._twist(vx=1.0))

        self.assertEqual(active_sources[-1], "teleop")
        self.assertAlmostEqual(driver_twists[-1].linear.x, 1.0)

    def test_active_source_publishes_correct_twist(self):
        """The active source's twist must be forwarded to driver_cmd_vel."""
        m = self._make(source_timeout=5.0)
        m.setup()
        driver_twists = []
        m.driver_cmd_vel._add_callback(driver_twists.append)

        m._on_source("visual_servo", self._twist(vx=0.8, wz=0.5))

        self.assertGreaterEqual(len(driver_twists), 1)
        self.assertAlmostEqual(driver_twists[-1].linear.x, 0.8)
        self.assertAlmostEqual(driver_twists[-1].angular.z, 0.5)

    def test_zero_autonomy_command_releases_source(self):
        m = self._make(source_timeout=5.0)
        m.setup()
        driver_twists = []
        m.driver_cmd_vel._add_callback(driver_twists.append)

        m._on_source("recovery", self._twist(vx=0.2))
        self.assertEqual(m.health()["active_source"], "recovery")

        m._on_source("recovery", Twist.zero())

        health = m.health()
        self.assertEqual(health["active_source"], "none")
        self.assertFalse(health["sources"]["recovery"]["active"])
        self.assertTrue(driver_twists[-1].is_zero())

    def test_teleop_zero_keeps_manual_source_active(self):
        m = self._make(source_timeout=5.0)
        m.setup()

        m._on_source("teleop", Twist.zero())

        health = m.health()
        self.assertEqual(health["active_source"], "teleop")
        self.assertTrue(health["sources"]["teleop"]["active"])

    def test_timeout_fallthrough(self):
        """When the active source times out, next highest priority must take over."""
        m = self._make(source_timeout=0.05)
        m.setup()
        active_sources = []
        m.active_source._add_callback(active_sources.append)

        # Publish teleop (high priority), then wait for it to time out
        m._on_source("teleop", self._twist(vx=1.0))

        # Now publish path_follower at same time (it's still alive below)
        # Actually the test for timeout - when teleop times out, nothing else active
        # Let's do it differently: publish teleop, wait for timeout, then recover
        time.sleep(0.06)  # wait for teleop to time out
        # Now path_follower publishes (should become active since teleop timed out)
        m._on_source("path_follower", self._twist(vx=0.5))
        self.assertEqual(active_sources[-1], "path_follower")

    def test_freeze_stops_outputs(self):
        """When frozen, VelocityMux must publish zero twist and reject inputs."""
        m = self._make(source_timeout=5.0)
        m.setup()
        driver_twists = []
        m.driver_cmd_vel._add_callback(driver_twists.append)

        m.freeze()
        self.assertTrue(m.is_frozen)

        # After freeze, publishing a source must NOT affect the output
        m._on_source("teleop", self._twist(vx=1.0))
        # The freeze itself publishes a zero twist
        self.assertAlmostEqual(driver_twists[-1].linear.x, 0.0)

    def test_unfreeze_restores_operation(self):
        """After unfreeze, VelocityMux must resume normal arbitration."""
        m = self._make(source_timeout=5.0)
        m.setup()
        driver_twists = []
        m.driver_cmd_vel._add_callback(driver_twists.append)

        # Freeze and verify
        m.freeze()
        m._on_source("teleop", self._twist(vx=1.0))
        saved_len = len(driver_twists)

        # Unfreeze and publish
        m.unfreeze()
        self.assertFalse(m.is_frozen)
        m._on_source("teleop", self._twist(vx=0.7))

        # After unfreeze, twist should be forwarded
        self.assertGreater(len(driver_twists), saved_len)
        self.assertAlmostEqual(driver_twists[-1].linear.x, 0.7)

    def test_freeze_is_idempotent(self):
        """Calling freeze multiple times must not raise."""
        m = self._make()
        m.freeze()
        m.freeze()  # second call
        self.assertTrue(m.is_frozen)
        m.unfreeze()
        self.assertFalse(m.is_frozen)
        m.unfreeze()  # second call (no-op)
        self.assertFalse(m.is_frozen)

    def test_health_returns_sources_and_active(self):
        """health() must include active_source and per-source status."""
        m = self._make(source_timeout=5.0)
        m.setup()
        m._on_source("teleop", self._twist(vx=1.0))

        h = m.health()
        self.assertIn("active_source", h)
        self.assertIn("sources", h)
        self.assertIn("teleop", h["sources"])
        self.assertIn("path_follower", h["sources"])
        self.assertEqual(h["active_source"], "teleop")

    def test_initial_active_source_is_empty(self):
        """Before any source publishes, active_source must be empty string."""
        m = self._make()
        self.assertEqual(m._active, "")

    def test_sanitize_drops_non_finite_twist(self):
        """A twist with nan/inf values must be replaced with zero twist."""
        m = self._make()
        bad = Twist(linear=Vector3(float("nan"), 0.0, 0.0))
        sanitized = m._sanitize_twist(bad)
        self.assertAlmostEqual(sanitized.linear.x, 0.0)
        self.assertAlmostEqual(sanitized.angular.z, 0.0)

    def test_select_active_returns_highest_priority_active(self):
        """_select_active must return the highest priority source within timeout."""
        m = self._make(source_timeout=5.0)
        now = time.time()

        # Path follower published 1 second ago, teleop 3 seconds ago
        m._sources["path_follower"]["last_time"] = now - 1.0
        m._sources["teleop"]["last_time"] = now - 3.0

        winner = m._select_active(now)
        self.assertEqual(winner, "teleop")  # higher priority, both still active

    def test_select_active_returns_empty_when_all_timed_out(self):
        """When all sources have timed out, _select_active must return empty string."""
        m = self._make(source_timeout=0.5)
        now = time.time()

        m._sources["teleop"]["last_time"] = now - 1.0
        m._sources["path_follower"]["last_time"] = now - 1.0

        winner = m._select_active(now)
        self.assertEqual(winner, "")

    def test_collision_monitor_passes_clear_projected_motion(self):
        m = self._make_collision_mux()
        driver_twists = []
        m.driver_cmd_vel._add_callback(driver_twists.append)

        m._on_odometry(self._odom())
        m._on_costmap(self._costmap(0.0))
        m._on_source("teleop", self._twist(vx=1.0))

        self.assertAlmostEqual(driver_twists[-1].linear.x, 1.0)
        self.assertEqual(m.health()["collision_monitor"]["action"], "pass")

    def test_collision_monitor_stops_projected_collision(self):
        m = self._make_collision_mux()
        driver_twists = []
        m.driver_cmd_vel._add_callback(driver_twists.append)

        m._on_odometry(self._odom())
        m._on_costmap(self._costmap(100.0))
        m._on_source("teleop", self._twist(vx=1.0))

        self.assertTrue(driver_twists[-1].is_zero())
        monitor = m.health()["collision_monitor"]
        self.assertEqual(monitor["action"], "stop")
        self.assertEqual(monitor["reason"], "projected_collision")

    def test_collision_monitor_slows_near_obstacle(self):
        m = self._make_collision_mux(collision_monitor_slowdown_scale=0.25)
        driver_twists = []
        m.driver_cmd_vel._add_callback(driver_twists.append)

        m._on_odometry(self._odom())
        m._on_costmap(self._costmap(70.0))
        m._on_source("teleop", self._twist(vx=1.0))

        self.assertAlmostEqual(driver_twists[-1].linear.x, 0.25)
        self.assertEqual(m.health()["collision_monitor"]["action"], "slowdown")

    def test_collision_monitor_stops_without_costmap(self):
        m = self._make_collision_mux()
        driver_twists = []
        m.driver_cmd_vel._add_callback(driver_twists.append)

        m._on_odometry(self._odom())
        m._on_source("teleop", self._twist(vx=1.0))

        self.assertTrue(driver_twists[-1].is_zero())
        monitor = m.health()["collision_monitor"]
        self.assertEqual(monitor["action"], "stop")
        self.assertEqual(monitor["reason"], "costmap_missing")

    def test_collision_monitor_rechecks_when_costmap_changes(self):
        m = self._make_collision_mux()
        driver_twists = []
        m.driver_cmd_vel._add_callback(driver_twists.append)

        m._on_odometry(self._odom())
        m._on_costmap(self._costmap(0.0))
        m._on_source("teleop", self._twist(vx=1.0))
        self.assertAlmostEqual(driver_twists[-1].linear.x, 1.0)

        m._on_costmap(self._costmap(100.0))

        self.assertTrue(driver_twists[-1].is_zero())
        self.assertEqual(m.health()["collision_monitor"]["reason"], "projected_collision")


if __name__ == "__main__":
    unittest.main()
