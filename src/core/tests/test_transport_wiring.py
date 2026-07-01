"""Tests for Blueprint transport-mediated wiring — runtime decoupling.

Verifies that wire(transport="dds"/"shm"/instance) routes data through
a transport backend instead of direct callbacks.
"""

import time
import unittest

from core import Blueprint, In, Module, Out
from core.transport.local import LocalTransport


class Producer(Module, layer=1):
    data: Out[int]

class Consumer(Module, layer=3):
    data: In[int]


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
        import core.transport.lcm as lcm_mod

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
