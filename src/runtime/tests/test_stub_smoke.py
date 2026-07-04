"""Stub-mode smoke test: verify data flows through the minimal blueprint.

Builds the stub blueprint (no SLAM, no semantic, no gateway) and verifies:
  1. Blueprint builds without error
  2. All expected modules are present
  3. System starts/stops cleanly
  4. Odometry flows from driver to navigation
  5. Safety stop_cmd reaches driver and navigation
  6. Goal injection produces a mission status change

TODO: Replace time.sleep with threading.Event for module output
      synchronization. Module start/stop and data flow assertions
      can use subscriber callbacks with Events instead of fixed waits.
"""

from __future__ import annotations

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

# Remove LLM keys to avoid side effects
for k in ["MOONSHOT_API_KEY", "OPENAI_API_KEY", "ANTHROPIC_API_KEY", "DASHSCOPE_API_KEY"]:
    os.environ.pop(k, None)


def _build_stub():
    from runtime.blueprints.products.thunder import thunder_blueprint
    return thunder_blueprint(
        robot="stub",
        slam_profile="none",
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        enable_native=False,
    )


class TestStubSmoke(unittest.TestCase):
    """Lightweight smoke tests with minimal timing-sensitive waits."""

    def test_blueprint_builds(self):
        bp = _build_stub()
        self.assertIsNotNone(bp)

    def test_expected_modules_present(self):
        bp = _build_stub()
        names = {e.name for e in bp._entries}
        self.assertIn("StubDogModule", names)
        self.assertIn("nav.mission", names)
        self.assertIn("nav.safety", names)

    def test_system_start_stop(self):
        bp = _build_stub()
        system = bp.build()
        system.start()
        time.sleep(0.05)
        system.stop()

    def test_odometry_flows_to_navigation(self):
        """StubDogModule publishes odometry and Navigation receives it."""
        bp = _build_stub()
        system = bp.build()

        nav = system.get_module("nav.mission")
        odom_received = []
        # Tap into nav's internal state after odom arrives
        orig_on_odom = nav._on_odom

        def _capture_odom(msg):
            odom_received.append(msg)
            orig_on_odom(msg)

        nav.odometry.subscribe(_capture_odom)

        system.start()
        time.sleep(0.1)  # stub driver publishes odom at ~10Hz
        system.stop()

        self.assertGreater(len(odom_received), 0,
                           "No odometry received by Navigation")

    def test_safety_stop_reaches_driver(self):
        """nav.services.safety.stop_cmd wired to driver.stop_signal."""
        bp = _build_stub()
        system = bp.build()

        drv = system.get_module("StubDogModule")
        stops = []
        if hasattr(drv, "stop_signal"):
            drv.stop_signal.subscribe(lambda msg: stops.append(msg))

        system.start()
        time.sleep(0.03)

        # Inject a stop command
        safety = system.get_module("nav.safety")
        safety.stop_cmd.publish(2)  # STOP level

        time.sleep(0.03)
        system.stop()

        self.assertGreater(len(stops), 0,
                           "Safety stop_cmd did not reach driver")

    def test_goal_triggers_mission_status(self):
        """Injecting a goal_pose produces mission_status output."""
        bp = _build_stub()
        system = bp.build()

        statuses = []
        nav = system.get_module("nav.mission")
        nav.mission_status.subscribe(lambda s: statuses.append(s))

        system.start()
        time.sleep(0.08)  # let odom arrive

        from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
        goal = PoseStamped(
            pose=Pose(position=Vector3(3.0, 2.0, 0.0),
                      orientation=Quaternion(0, 0, 0, 1)),
            ts=time.time(), frame_id="map",
        )
        nav.goal_pose._deliver(goal)
        time.sleep(0.1)
        system.stop()

        self.assertGreater(len(statuses), 0,
                           "No mission_status after goal injection")

    def test_health_reports(self):
        """All modules produce health() dicts."""
        bp = _build_stub()
        system = bp.build()
        system.start()
        time.sleep(0.03)

        for name, mod in system._modules.items():
            if hasattr(mod, "health"):
                h = mod.health()
                self.assertIsInstance(h, dict, f"{name}.health() not a dict")

        system.stop()


if __name__ == "__main__":
    unittest.main()
