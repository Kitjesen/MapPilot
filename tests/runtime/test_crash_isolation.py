"""Tests for runtime decoupling — crash isolation via transport-mediated wiring.

Proves that when a module connected via transport crashes (raises exception),
the publisher and other modules continue operating normally.
"""

import unittest

from runtime import Blueprint, In, Module, Out


class SafetyController(Module, layer=0):
    """Safety-critical module that must never stop."""
    stop_cmd: Out[int]
    heartbeat: Out[int]


class RobotDriver(Module, layer=1):
    """Robot driver — receives safety commands and publishes odometry."""
    stop_signal: In[int]
    odometry: Out[int]


class Perception(Module, layer=3):
    """Perception module — can crash (OOM, model failure)."""
    odometry: In[int]
    scene_graph: Out[dict]

    def setup(self):
        self._crash_on_next = False
        self.odometry.subscribe(self._on_odom)

    def _on_odom(self, msg):
        if self._crash_on_next:
            raise RuntimeError("PERCEPTION CRASHED: simulated OOM")
        self.scene_graph.publish({"objects": msg})


class Planner(Module, layer=4):
    """Planner — receives scene graph, can also crash."""
    scene_graph: In[dict]
    goal: Out[str]

    def setup(self):
        self.scene_graph.subscribe(self._on_sg)

    def _on_sg(self, sg):
        self.goal.publish(f"goal_from_{sg.get('objects', '?')}")


class TestCrashIsolationCallback(unittest.TestCase):
    """Without transport, a subscriber crash propagates to the publisher's thread."""

    def test_callback_crash_is_caught(self):
        """Direct callback: exception is caught by Out.publish(), publisher survives."""
        bp = Blueprint()
        bp.add(RobotDriver)
        bp.add(Perception)
        bp.auto_wire()
        handle = bp.build()
        handle.start()

        driver = handle.get_module("RobotDriver")
        perception = handle.get_module("Perception")

        # Normal flow works
        driver.odometry.publish(1)
        self.assertEqual(perception.scene_graph.msg_count, 1)

        # Crash perception
        perception._crash_on_next = True
        driver.odometry.publish(2)  # should NOT crash the driver

        # Driver is still alive
        self.assertTrue(driver.running)

        # Perception crashed but driver survived
        perception._crash_on_next = False
        driver.odometry.publish(3)
        self.assertEqual(perception.scene_graph.msg_count, 2)  # recovered

        handle.stop()


class TestCrashIsolationTransport(unittest.TestCase):
    """With transport, crash isolation is even stronger — data flows through transport layer."""

    def test_transport_isolates_crash(self):
        """Transport-mediated: subscriber crash doesn't affect publisher at all."""
        bp = Blueprint()
        bp.add(RobotDriver)
        bp.add(Perception)
        bp.wire("RobotDriver", "odometry", "Perception", "odometry",
                delivery="local")
        handle = bp.build()
        handle.start()

        driver = handle.get_module("RobotDriver")
        perception = handle.get_module("Perception")

        # Normal flow
        driver.odometry.publish(10)
        self.assertEqual(perception.odometry.latest, 10)

        # Crash perception callback
        perception._crash_on_next = True
        driver.odometry.publish(20)  # goes through transport

        # Driver is completely unaffected
        self.assertTrue(driver.running)
        self.assertEqual(driver.odometry.msg_count, 2)

        handle.stop()


class TestThreeTierWiring(unittest.TestCase):
    """Full 3-tier wiring: safety(callback) + control(callback) + semantic(transport)."""

    def test_three_tier_decoupling(self):
        bp = Blueprint()
        bp.add(SafetyController)
        bp.add(RobotDriver)
        bp.add(Perception)
        bp.add(Planner)

        # Tier 1: safety — direct callback
        bp.wire("SafetyController", "stop_cmd", "RobotDriver", "stop_signal")

        # Tier 2: control — direct callback (via auto_wire)
        # RobotDriver.odometry → Perception.odometry (same name+type)

        # Tier 3: semantic — transport decoupled
        bp.wire("Perception", "scene_graph", "Planner", "scene_graph",
                delivery="local")

        bp.auto_wire()
        handle = bp.build()
        handle.start()

        safety = handle.get_module("SafetyController")
        driver = handle.get_module("RobotDriver")
        perception = handle.get_module("Perception")
        planner = handle.get_module("Planner")

        # Full pipeline works
        driver.odometry.publish(1)
        self.assertEqual(planner.goal.msg_count, 1)

        # Safety works
        safety.stop_cmd.publish(2)
        self.assertEqual(driver.stop_signal.msg_count, 1)

        # Crash perception — planner stops getting data but safety still works
        perception._crash_on_next = True
        driver.odometry.publish(2)  # perception crashes on this

        # Safety is completely unaffected
        safety.stop_cmd.publish(2)
        self.assertEqual(driver.stop_signal.msg_count, 2)

        # Driver is completely unaffected
        self.assertTrue(driver.running)

        handle.stop()


class TestTransportDataIntegrity(unittest.TestCase):
    """Verify data arrives correctly through transport, not just callback."""

    def test_data_round_trip_through_transport(self):
        bp = Blueprint()
        bp.add(Perception)
        bp.add(Planner)
        bp.wire("Perception", "scene_graph", "Planner", "scene_graph",
                delivery="local")
        handle = bp.build()
        handle.start()

        perception = handle.get_module("Perception")
        planner = handle.get_module("Planner")

        # Publish complex data
        perception.scene_graph.publish({"objects": 42, "rooms": ["kitchen"]})

        # Planner receives it correctly
        self.assertEqual(planner.scene_graph.latest,
                         {"objects": 42, "rooms": ["kitchen"]})
        self.assertEqual(planner.goal.msg_count, 1)

        handle.stop()


if __name__ == "__main__":
    unittest.main(verbosity=2)
