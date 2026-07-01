"""Tests for Blueprint transport-mediated wiring — runtime decoupling.

Verifies that wire(transport="dds"/"shm"/instance) routes data through
a transport backend instead of direct callbacks.
"""

import time
import unittest
from unittest.mock import patch

from runtime import Blueprint, In, Module, Out
from runtime.msgs.geometry import Twist
from runtime.runtime_interface import TOPICS
from runtime.transport.local import LocalTransport


class Producer(Module, layer=1):
    data: Out[int]

class Consumer(Module, layer=3):
    data: In[int]


class OtherConsumer(Module, layer=3):
    data: In[int]


class CmdProducer(Module, layer=1):
    cmd_vel: Out[Twist]


class CmdConsumer(Module, layer=3):
    cmd_vel: In[Twist]


class TestDirectCallbackDefault(unittest.TestCase):
    """Default wire() uses direct callback — existing behavior."""

    def test_default_is_callback(self):
        bp = Blueprint()
        bp.add(Producer)
        bp.add(Consumer)
        bp.wire("Producer", "data", "Consumer", "data")  # no transport
        handle = bp.build()
        handle.start()

        prod = handle.get_module("Producer")
        cons = handle.get_module("Consumer")

        received = []
        cons.data.subscribe(received.append)
        prod.data.publish(42)

        self.assertEqual(received, [42])
        handle.stop()

    def test_callback_wire_records_explicit_topic_contract(self):
        bp = Blueprint()
        bp.add(Producer)
        bp.add(Consumer)
        bp.wire("Producer", "data", "Consumer", "data", topic="/planner/data")
        handle = bp.build()
        handle.start()

        prod = handle.get_module("Producer")
        cons = handle.get_module("Consumer")

        prod.data.publish(42)
        self.assertEqual(cons.data.latest, 42)

        health = handle.comm_health()
        self.assertEqual(health["links"][0]["delivery"], "callback")
        self.assertEqual(health["links"][0]["transport"], "callback")
        self.assertEqual(health["links"][0]["topic"], "/planner/data")
        handle.stop()


class TestLocalTransportWiring(unittest.TestCase):
    """wire(transport="local") routes through LocalTransport."""

    def test_local_transport_delivers(self):
        bp = Blueprint()
        bp.add(Producer)
        bp.add(Consumer)
        bp.wire("Producer", "data", "Consumer", "data", transport="local")
        handle = bp.build()
        handle.start()

        prod = handle.get_module("Producer")
        cons = handle.get_module("Consumer")

        # Consumer's In port gets data via transport, not callback
        self.assertEqual(cons.data.msg_count, 0)
        prod.data.publish(99)
        self.assertEqual(cons.data.msg_count, 1)
        self.assertEqual(cons.data.latest, 99)

        health = handle.comm_health()
        self.assertEqual(health["links"][0]["delivery"], "transport")
        self.assertEqual(health["links"][0]["transport"], "local")
        self.assertEqual(health["links"][0]["topic"], "/Producer/data")
        handle.stop()

    def test_local_transport_uses_explicit_topic_contract(self):
        bp = Blueprint()
        bp.add(Producer)
        bp.add(Consumer)
        bp.wire(
            "Producer",
            "data",
            "Consumer",
            "data",
            transport="local",
            topic="/planner/data",
        )
        handle = bp.build()
        handle.start()

        prod = handle.get_module("Producer")
        cons = handle.get_module("Consumer")

        prod.data.publish(100)
        self.assertEqual(cons.data.latest, 100)

        health = handle.comm_health()
        self.assertEqual(health["links"][0]["delivery"], "transport")
        self.assertEqual(health["links"][0]["transport"], "local")
        self.assertEqual(health["links"][0]["topic"], "/planner/data")
        handle.stop()

    def test_local_transport_instance(self):
        """Pass a Transport instance directly."""
        transport = LocalTransport()
        bp = Blueprint()
        bp.add(Producer)
        bp.add(Consumer)
        bp.wire("Producer", "data", "Consumer", "data", transport=transport)
        handle = bp.build()
        handle.start()

        prod = handle.get_module("Producer")
        cons = handle.get_module("Consumer")

        prod.data.publish(77)
        self.assertEqual(cons.data.latest, 77)
        handle.stop()

    def test_local_transport_fanout_delivers_to_all_subscribers(self):
        """One Out port can feed multiple transport-wired In ports."""
        bp = Blueprint()
        bp.add(Producer)
        bp.add(Consumer)
        bp.add(OtherConsumer)
        bp.wire("Producer", "data", "Consumer", "data", transport="local")
        bp.wire("Producer", "data", "OtherConsumer", "data", transport="local")
        handle = bp.build()
        handle.start()

        prod = handle.get_module("Producer")
        cons = handle.get_module("Consumer")
        other = handle.get_module("OtherConsumer")

        prod.data.publish(101)

        self.assertEqual(cons.data.latest, 101)
        self.assertEqual(other.data.latest, 101)
        self.assertEqual(cons.data.msg_count, 1)
        self.assertEqual(other.data.msg_count, 1)
        handle.stop()


class TestSHMTransportWiring(unittest.TestCase):
    """wire(transport="shm") routes through shared memory."""

    def test_shm_transport_delivers(self):
        """SHM subscriber retries until publisher creates the region."""
        bp = Blueprint()
        bp.add(Producer)
        bp.add(Consumer)
        bp.wire("Producer", "data", "Consumer", "data", transport="shm")
        handle = bp.build()
        handle.start()

        prod = handle.get_module("Producer")
        cons = handle.get_module("Consumer")

        prod.data.publish(123)
        # SHM: subscriber retries attach + polling delay (poll_interval=0.002s)
        time.sleep(0.1)
        self.assertEqual(cons.data.latest, 123)
        handle.stop()


class TestLCMTransportWiring(unittest.TestCase):
    """wire(transport="lcm") is explicit and optional."""

    def test_lcm_transport_missing_optional_package_raises(self):
        import runtime.transport.lcm as lcm_mod

        original_available = lcm_mod._LCM_AVAILABLE
        lcm_mod._LCM_AVAILABLE = False
        try:
            bp = Blueprint()
            bp.add(Producer)
            bp.add(Consumer)
            bp.wire("Producer", "data", "Consumer", "data", transport="lcm")
            with self.assertRaisesRegex(ImportError, "lcm is not installed"):
                bp.build()
        finally:
            lcm_mod._LCM_AVAILABLE = original_available


class TestDDSTransportWiring(unittest.TestCase):
    """Generic DDS wiring must not carry registered product topics."""

    def test_dds_transport_rejects_registered_product_topic(self):
        class FakeDDSBackend:
            name = "dds"

            def create_publisher(self, _topic):
                raise AssertionError("publisher should not be created")

            def create_subscriber(self, _topic, _callback):
                raise AssertionError("subscriber should not be created")

            def close(self):
                pass

        bp = Blueprint()
        bp.add(CmdProducer)
        bp.add(CmdConsumer)
        bp.wire(
            "CmdProducer",
            "cmd_vel",
            "CmdConsumer",
            "cmd_vel",
            transport="dds",
            topic=TOPICS.cmd_vel,
        )

        with patch("runtime.transport.factory.create_transport", return_value=FakeDDSBackend()):
            with self.assertRaisesRegex(ValueError, "registered product topic /nav/cmd_vel"):
                bp.build()


class TestMixedTransportWiring(unittest.TestCase):
    """Same blueprint with both callback and transport connections."""

    def test_mixed_callback_and_transport(self):

        class Safety(Module, layer=0):
            alert: Out[str]

        class Driver(Module, layer=1):
            alert: In[str]
            odom: Out[int]

        class Planner(Module, layer=4):
            odom: In[int]

        bp = Blueprint()
        bp.add(Safety)
        bp.add(Driver)
        bp.add(Planner)

        # Safety → Driver: direct callback (fast, safety-critical)
        bp.wire("Safety", "alert", "Driver", "alert")
        # Driver → Planner: via local transport (decoupled)
        bp.wire("Driver", "odom", "Planner", "odom", transport="local")

        handle = bp.build()
        handle.start()

        safety = handle.get_module("Safety")
        driver = handle.get_module("Driver")
        planner = handle.get_module("Planner")

        # Alert goes through callback
        driver_alerts = []
        driver.alert.subscribe(driver_alerts.append)
        safety.alert.publish("STOP")
        self.assertEqual(driver_alerts, ["STOP"])

        # Odom goes through transport
        driver.odom.publish(42)
        self.assertEqual(planner.odom.latest, 42)

        handle.stop()


class TestNavigationPlanTransportContract(unittest.TestCase):
    """Local-planner execution wires are the C++ process boundary."""

    def test_navigation_plan_wires_default_to_direct_callbacks(self):
        from runtime.blueprints.wires.navigation import navigation_execution_specs

        specs = navigation_execution_specs()

        self.assertTrue(specs)
        self.assertTrue(all(spec.transport is None for spec in specs))
        self.assertTrue(all(spec.topic is None for spec in specs))

    def test_navigation_plan_transport_uses_canonical_topics(self):
        from runtime.blueprints.wires.navigation import navigation_execution_specs

        specs = navigation_execution_specs(local_planner_transport="local")
        edges = {
            (spec.out_module, spec.out_port, spec.in_module, spec.in_port): spec
            for spec in specs
        }

        self.assertEqual(
            edges[
                ("nav.mission", "global_path", "nav.local_planner", "global_path")
            ].topic,
            TOPICS.global_path,
        )
        self.assertEqual(
            edges[
                ("nav.terrain", "terrain_map", "nav.local_planner", "terrain_map")
            ].topic,
            TOPICS.terrain_map,
        )
        self.assertEqual(
            edges[
                (
                    "nav.terrain",
                    "traversability",
                    "nav.local_planner",
                    "traversability",
                )
            ].topic,
            TOPICS.traversability,
        )
        self.assertEqual(
            edges[
                ("nav.local_planner", "local_path", "nav.path_follower", "local_path")
            ].topic,
            TOPICS.local_path,
        )
        self.assertEqual(
            edges[
                (
                    "nav.local_planner",
                    "control_hint",
                    "nav.path_follower",
                    "control_hint",
                )
            ].topic,
            TOPICS.local_planner_control_hint,
        )
        self.assertTrue(all(spec.transport == "local" for spec in specs))


class TestInvalidTransport(unittest.TestCase):

    def test_unknown_string_raises(self):
        bp = Blueprint()
        bp.add(Producer)
        bp.add(Consumer)
        bp.wire("Producer", "data", "Consumer", "data", transport="mqtt")
        with self.assertRaises(ValueError):
            bp.build()


if __name__ == "__main__":
    unittest.main(verbosity=2)
