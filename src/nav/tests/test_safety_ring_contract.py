"""Contract tests for SafetyRing port types, safety level computation,
stop_cmd publishing, and AI-callable skills.

All tests are pure-Python, no ROS2 / hardware / MuJoCo required.
"""

from __future__ import annotations

import json
import math
import time
import unittest
from unittest.mock import patch

import numpy as np

from runtime.msgs.geometry import Pose, PoseStamped, Twist, Vector3
from runtime.msgs.nav import Odometry, Path
from runtime.stream import In, Out


class TestSafetyRingContract(unittest.TestCase):
    """Contract tests for nav.services.safety."""

    def _make(self, **kw):
        from nav.services.safety.safety_ring import SafetyRing
        kwargs = dict(cross_track_warn=1.5, cross_track_danger=3.0)
        kwargs.update(kw)
        return SafetyRing(**kwargs)

    def test_ports_in(self):
        """Must declare all required input ports."""
        m = self._make()
        self.assertIn("odometry", m.ports_in)
        self.assertIn("path", m.ports_in)
        self.assertIn("cmd_vel", m.ports_in)
        self.assertIn("mission_status", m.ports_in)
        self.assertIn("localization_status", m.ports_in)
        self.assertIn("gnss_fusion_health", m.ports_in)

    def test_ports_out(self):
        """Must declare all required output ports."""
        m = self._make()
        self.assertIn("stop_cmd", m.ports_out)
        self.assertIn("safety_state", m.ports_out)
        self.assertIn("execution_eval", m.ports_out)
        self.assertIn("dialogue_state", m.ports_out)
        # Port type checks
        self.assertIsInstance(m.ports_out["stop_cmd"], Out)
        self.assertIsInstance(m.ports_out["safety_state"], Out)
        self.assertIsInstance(m.ports_out["execution_eval"], Out)
        self.assertIsInstance(m.ports_out["dialogue_state"], Out)

    def test_layer_is_0(self):
        """SafetyRing must be layer 0."""
        m = self._make()
        self.assertEqual(m.layer, 0)

    def test_initial_safety_level_is_safe(self):
        """Before any data, safety level should be SAFE."""
        m = self._make()
        self.assertEqual(m._safety_level.name, "SAFE")

    def test_safe_level_with_valid_odometry(self):
        """With valid odometry and cmd_vel, level stays SAFE."""
        m = self._make()
        m.setup()
        # Publish valid odometry
        odom = Odometry(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=(0, 0, 0, 1)),
            twist=Twist(linear=Vector3(0.1, 0.0, 0.0)),
        )
        m._on_odom(odom)
        # Publish valid cmd_vel
        m._on_cmdvel(Twist(linear=Vector3(0.1, 0.0, 0.0)))
        self.assertEqual(m._safety_level.name, "SAFE")

    def test_wall_clock_jump_does_not_expire_live_safety_links(self):
        """NTP or operator wall-clock changes must not affect link deadlines."""
        m = self._make()
        m.setup()
        healthy_odom = Odometry(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
            twist=Twist(linear=Vector3(0.1, 0.0, 0.0)),
        )
        m._on_odom(healthy_odom)
        m._on_cmdvel(Twist(linear=Vector3(0.1, 0.0, 0.0)))
        m._on_localization_status({"state": "TRACKING", "confidence": 1.0})

        with patch(
            "nav.services.safety.safety_ring.time.time",
            return_value=time.time() + 3600.0,
        ):
            m._publish_safety()

        self.assertEqual(m._safety_level.name, "SAFE")

    def test_stop_level_on_odom_timeout(self):
        """When odometry times out, level should be STOP and stop_cmd=2."""
        m = self._make(odom_timeout_ms=50.0)
        m.setup()
        stops = []
        m.stop_cmd._add_callback(stops.append)
        # Publish odometry once, then wait for timeout
        m._on_odom(Odometry(pose=Pose(position=Vector3(1.0, 2.0, 0.0))))
        # Simulate timeout
        m._last_odom_time = time.monotonic() - 1.0
        m._publish_safety()
        self.assertEqual(m._safety_level.name, "STOP")
        self.assertEqual(stops[-1], 2)

    def test_stop_level_on_invalid_odom(self):
        """Non-finite odometry values trigger STOP."""
        m = self._make()
        m.setup()
        stops = []
        m.stop_cmd._add_callback(stops.append)
        # Odometry with nan position
        m._on_odom(Odometry(
            pose=Pose(position=Vector3(float("nan"), 2.0, 0.0)),
            twist=Twist(linear=Vector3(0.1, 0.0, 0.0)),
        ))
        self.assertEqual(m._safety_level.name, "STOP")
        self.assertEqual(stops[-1], 2)

    def test_warn_level_on_cmd_vel_timeout_with_motion_intent(self):
        """When cmd_vel times out but odometry is recent, level should be WARN."""
        m = self._make(cmd_vel_timeout_ms=50.0)
        m.setup()
        stops = []
        m.stop_cmd._add_callback(stops.append)
        # Set up a path to establish motion intent
        path = Path(poses=[
            PoseStamped(pose=Pose(position=Vector3(0.0, 0.0, 0.0))),
            PoseStamped(pose=Pose(position=Vector3(5.0, 0.0, 0.0))),
        ])
        m._on_path(path)
        # Publish recent odometry
        m._on_odom(Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0))))
        # Simulate cmd_vel timeout by setting last_cmdvel_time far in past
        m._last_cmdvel_time = time.monotonic() - 1.0
        m._publish_safety()
        self.assertEqual(m._safety_level.name, "WARN")
        self.assertEqual(stops[-1], 1)

    def test_cross_track_error_no_path(self):
        """Without a path, cross_track_error must be 0.0."""
        m = self._make()
        self.assertAlmostEqual(m._cross_track_error(), 0.0)

    def test_cross_track_error_with_path(self):
        """With a straight path, cross_track_error reflects lateral offset."""
        m = self._make()
        path = Path(poses=[
            PoseStamped(pose=Pose(position=Vector3(0.0, 0.0, 0.0))),
            PoseStamped(pose=Pose(position=Vector3(10.0, 0.0, 0.0))),
        ])
        m._on_path(path)
        m._robot_xy = np.array([5.0, 1.0])  # 1 m off path centre
        self.assertAlmostEqual(m._cross_track_error(), 1.0, places=4)

    def test_distance_to_goal_no_goal(self):
        """Without a goal, distance_to_goal must be inf."""
        m = self._make()
        self.assertEqual(m._distance_to_goal(), float("inf"))

    def test_distance_to_goal_with_path(self):
        """With a path, distance_to_goal is distance to last waypoint."""
        m = self._make()
        path = Path(poses=[
            PoseStamped(pose=Pose(position=Vector3(0.0, 0.0, 0.0))),
            PoseStamped(pose=Pose(position=Vector3(3.0, 4.0, 0.0))),
        ])
        m._on_path(path)
        m._robot_xy = np.array([0.0, 0.0])
        self.assertAlmostEqual(m._distance_to_goal(), 5.0, places=4)

    def test_evaluate_idle_no_path(self):
        """Without a path, assessment must be IDLE."""
        m = self._make()
        self.assertEqual(m._assessment.name, "IDLE")

    def test_health_returns_info(self):
        """health() must include safety_ring info."""
        m = self._make()
        h = m.health()
        self.assertIn("safety_ring", h)
        self.assertIn("level", h["safety_ring"])
        self.assertIn("assessment", h["safety_ring"])
        self.assertIn("has_path", h["safety_ring"])

    def test_get_safety_status_skill(self):
        """@skill get_safety_status returns JSON with expected fields."""
        m = self._make()
        result = json.loads(m.get_safety_status())
        self.assertIn("level", result)
        self.assertIn("cross_track_error", result)
        self.assertIn("distance_to_goal", result)
        self.assertIn("modules_ok", result)

    def test_emergency_stop_skill(self):
        """@skill emergency_stop publishes stop_cmd=2 and returns estop status."""
        m = self._make()
        stops = []
        m.stop_cmd._add_callback(stops.append)
        result = json.loads(m.emergency_stop())
        self.assertEqual(result["status"], "estop_triggered")
        self.assertEqual(stops[-1], 2)

    def test_emergency_stop_stays_latched_until_explicit_reset(self):
        """Healthy inputs must not auto-clear an operator emergency stop."""
        m = self._make()
        m.setup()
        stops = []
        m.stop_cmd._add_callback(stops.append)
        healthy_odom = Odometry(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
            twist=Twist(linear=Vector3(0.1, 0.0, 0.0)),
        )
        healthy_cmd = Twist(linear=Vector3(0.1, 0.0, 0.0))
        m._on_odom(healthy_odom)
        m._on_cmdvel(healthy_cmd)
        m._on_localization_status({"state": "TRACKING", "confidence": 1.0})
        stops.clear()

        triggered = json.loads(m.emergency_stop())
        m._on_odom(healthy_odom)
        m._on_cmdvel(healthy_cmd)
        m._on_localization_status({"state": "TRACKING", "confidence": 1.0})

        self.assertEqual(triggered["status"], "estop_triggered")
        self.assertTrue(m._estop_latched)
        self.assertEqual(m._safety_level.name, "STOP")
        self.assertEqual(stops[-1], 2)

        reset = json.loads(m.reset_emergency_stop())

        self.assertEqual(reset["status"], "estop_reset")
        self.assertFalse(m._estop_latched)
        self.assertEqual(m._safety_level.name, "SAFE")
        self.assertEqual(stops[-1], 0)

    def test_emergency_stop_reset_is_rejected_while_base_stop_is_active(self):
        """Reset cannot bypass an independent hard safety fault."""
        m = self._make()
        stops = []
        m.stop_cmd._add_callback(stops.append)
        m.emergency_stop()

        result = json.loads(m.reset_emergency_stop())

        self.assertEqual(result["status"], "estop_reset_rejected")
        self.assertTrue(m._estop_latched)
        self.assertEqual(m._safety_level.name, "STOP")
        self.assertEqual(stops[-1], 2)

    def test_safety_state_published_fields(self):
        """safety_state output must be a SafetyState with correct level."""
        m = self._make()
        states = []
        m.safety_state._add_callback(states.append)
        m._publish_safety()
        self.assertGreaterEqual(len(states), 1)
        self.assertIn("level", states[-1].__dict__)

    def test_localization_lost_triggers_stop(self):
        """When localization state is LOST, safety level must be STOP."""
        m = self._make()
        m.setup()
        stops = []
        m.stop_cmd._add_callback(stops.append)
        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        self.assertEqual(m._safety_level.name, "STOP")
        self.assertEqual(stops[-1], 2)

    def test_localization_degraded_triggers_warn(self):
        """When localization state is DEGRADED, safety level must be WARN."""
        m = self._make()
        m.setup()
        stops = []
        m.stop_cmd._add_callback(stops.append)
        # Ensure odometry and cmd_vel are alive
        m._on_odom(Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0))))
        m._on_cmdvel(Twist(linear=Vector3(0.1, 0.0, 0.0)))
        # Trigger degraded localization
        m._on_localization_status({"state": "DEGRADED", "confidence": 0.5})
        self.assertEqual(m._safety_level.name, "WARN")
        self.assertEqual(stops[-1], 1)

    def test_unknown_received_localization_state_fails_closed(self):
        """A typo or future unknown localization state must never permit motion."""
        m = self._make()
        m.setup()
        stops = []
        m.stop_cmd._add_callback(stops.append)
        m._on_odom(Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0))))
        m._on_cmdvel(Twist(linear=Vector3(0.1, 0.0, 0.0)))

        m._on_localization_status({"state": "TRAKCING", "confidence": 1.0})

        self.assertEqual(m._safety_level.name, "STOP")
        self.assertEqual(stops[-1], 2)

    def test_mcp_emergency_stop_resolves_to_safety_ring_latch(self):
        from gateway.mcp_server import MCPServerModule

        ring = self._make()
        mcp = MCPServerModule(port=0)
        mcp.on_system_modules(
            {
                "nav.safety": ring,
                "MCPServerModule": mcp,
            }
        )

        selected = mcp._tool_registry["emergency_stop"]
        self.assertIs(selected.__self__, ring)
        result = json.loads(selected())
        self.assertTrue(result["latched"])
        self.assertTrue(ring.health()["safety_ring"]["estop_latched"])


if __name__ == "__main__":
    unittest.main()
