"""Tests for communication statistics (stream.py rate/latency/error tracking).

Covers:
  - Out: rate_hz, publish_errors, stale_ms
  - In: rate_hz, deliver_count, callback_errors, avg/max_callback_ms, stale_ms
  - Blueprint.comm_health() aggregation
"""

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from runtime.stream import In, Out


class TestOutStats(unittest.TestCase):

    def test_initial_rate_is_zero(self):
        o = Out(int, "test_out")
        self.assertEqual(o.rate_hz, 0.0)

    def test_publish_errors_counted(self):
        o = Out(int, "test_out")
        def bad_cb(msg):
            raise ValueError("boom")
        o._add_callback(bad_cb)
        o.publish(1)
        o.publish(2)
        self.assertEqual(o.publish_errors, 2)

    def test_stale_ms_negative_when_never_published(self):
        o = Out(int, "test_out")
        self.assertEqual(o.stale_ms, -1.0)

    def test_stale_ms_positive_after_publish(self):
        o = Out(int, "test_out")
        o.publish(42)
        time.sleep(0.01)
        self.assertGreater(o.stale_ms, 0.0)

    def test_msg_count_increments(self):
        o = Out(int, "test_out")
        for i in range(10):
            o.publish(i)
        self.assertEqual(o.msg_count, 10)

    def test_callback_count(self):
        o = Out(int, "test_out")
        o._add_callback(lambda x: None)
        o._add_callback(lambda x: None)
        self.assertEqual(o.callback_count, 2)

    def test_repr_includes_rate(self):
        o = Out(int, "test_out")
        # Force rate to non-zero
        o._rate_hz = 10.5
        r = repr(o)
        self.assertIn("10.5Hz", r)


class TestInStats(unittest.TestCase):

    def test_initial_rate_is_zero(self):
        p = In(int, "test_in")
        self.assertEqual(p.rate_hz, 0.0)

    def test_deliver_count_tracks_actual_deliveries(self):
        p = In(int, "test_in")
        p.subscribe(lambda x: None)
        for i in range(5):
            p._deliver(i)
        self.assertEqual(p.deliver_count, 5)
        self.assertEqual(p.msg_count, 5)

    def test_callback_errors_counted(self):
        p = In(int, "test_in")
        def bad_cb(msg):
            raise RuntimeError("oops")
        p.subscribe(bad_cb)
        p._deliver(1)
        p._deliver(2)
        self.assertEqual(p.callback_errors, 2)
        self.assertEqual(p.deliver_count, 2)

    def test_max_callback_ms_tracked(self):
        p = In(int, "test_in")
        def slow_cb(msg):
            time.sleep(0.02)
        p.subscribe(slow_cb)
        p._deliver(1)
        self.assertGreater(p.max_callback_ms, 15.0)

    def test_avg_callback_ms(self):
        p = In(int, "test_in")
        def cb(msg):
            time.sleep(0.01)
        p.subscribe(cb)
        for i in range(3):
            p._deliver(i)
        self.assertGreater(p.avg_callback_ms, 5.0)
        self.assertEqual(p.deliver_count, 3)

    def test_avg_callback_ms_zero_when_no_deliveries(self):
        p = In(int, "test_in")
        self.assertEqual(p.avg_callback_ms, 0.0)

    def test_stale_ms_negative_when_never_received(self):
        p = In(int, "test_in")
        self.assertEqual(p.stale_ms, -1.0)

    def test_stale_ms_positive_after_delivery(self):
        p = In(int, "test_in")
        p._deliver(1)
        time.sleep(0.01)
        self.assertGreater(p.stale_ms, 0.0)

    def test_drop_count_with_latest_policy(self):
        p = In(int, "test_in")
        p.set_policy("latest")
        results = []
        def slow_cb(msg):
            time.sleep(0.05)
            results.append(msg)
        p.subscribe(slow_cb)
        # First delivery blocks in callback
        import threading
        t = threading.Thread(target=p._deliver, args=(1,))
        t.start()
        time.sleep(0.01)
        # Second delivery should be dropped (busy)
        p._deliver(2)
        t.join()
        self.assertEqual(p.drop_count, 1)
        self.assertEqual(len(results), 1)

    def test_buffer_policy_tracks_latency(self):
        p = In(int, "test_in")
        p.set_policy("buffer", size=3)
        batches = []
        def cb(batch):
            time.sleep(0.01)
            batches.append(batch)
        p.subscribe(cb)
        p._deliver(1)
        p._deliver(2)
        p._deliver(3)  # triggers batch delivery
        self.assertEqual(len(batches), 1)
        self.assertEqual(batches[0], [1, 2, 3])
        self.assertEqual(p.deliver_count, 1)
        self.assertGreater(p.max_callback_ms, 5.0)

    def test_repr_includes_rate(self):
        p = In(int, "test_in")
        p._rate_hz = 50.0
        r = repr(p)
        self.assertIn("50.0Hz", r)


class TestOutInWiring(unittest.TestCase):
    """Test Out→In wiring preserves stats."""

    def test_publish_to_subscriber_tracks_both(self):
        out = Out(int, "source")
        inp = In(int, "sink")
        received = []
        inp.subscribe(received.append)
        out._add_callback(inp._deliver)

        for i in range(5):
            out.publish(i)

        self.assertEqual(out.msg_count, 5)
        self.assertEqual(inp.msg_count, 5)
        self.assertEqual(inp.deliver_count, 5)
        self.assertEqual(len(received), 5)

    def test_error_in_subscriber_tracked_on_in_port(self):
        out = Out(int, "source")
        inp = In(int, "sink")
        def bad_cb(msg):
            raise ValueError("fail")
        inp.subscribe(bad_cb)
        out._add_callback(inp._deliver)

        out.publish(1)
        # Error happens inside In._deliver callback, not Out.publish callback
        self.assertEqual(inp.callback_errors, 1)
        self.assertEqual(inp.deliver_count, 1)
        # Out sees no error (its callback to _deliver succeeded)
        self.assertEqual(out.publish_errors, 0)


class TestPortSummaryStats(unittest.TestCase):
    """Test Module.port_summary() includes new stats."""

    def test_port_summary_has_comm_fields(self):
        from runtime.module import Module
        from runtime.stream import In, Out

        class DummyMod(Module):
            data_out: Out[int]
            data_in: In[int]
            def setup(self): pass

        m = DummyMod()
        summary = m.port_summary()

        # Check In port fields
        in_info = summary["ports_in"]["data_in"]
        self.assertIn("rate_hz", in_info)
        self.assertIn("drop_count", in_info)
        self.assertIn("deliver_count", in_info)
        self.assertIn("callback_errors", in_info)
        self.assertIn("avg_callback_ms", in_info)
        self.assertIn("max_callback_ms", in_info)
        self.assertIn("stale_ms", in_info)

        # Check Out port fields
        out_info = summary["ports_out"]["data_out"]
        self.assertIn("rate_hz", out_info)
        self.assertIn("publish_errors", out_info)
        self.assertIn("stale_ms", out_info)


class TestBlueprintCommHealth(unittest.TestCase):
    """Test Blueprint.comm_health() aggregation."""

    def test_comm_health_basic(self):
        from runtime.blueprint import Blueprint
        from runtime.module import Module

        class Producer(Module):
            output: Out[int]
            def setup(self): pass

        class Consumer(Module):
            input: In[int]
            def setup(self): pass

        bp = Blueprint()
        bp.add(Producer)
        bp.add(Consumer)
        bp.wire("Producer", "output", "Consumer", "input")
        system = bp.build()
        system.start()

        # Publish some messages
        prod = system.get_module("Producer")
        prod.output.publish(1)
        prod.output.publish(2)
        prod.output.publish(3)

        ch = system.comm_health()
        self.assertEqual(ch["link_count"], 1)
        self.assertEqual(ch["total_drops"], 0)
        self.assertEqual(ch["total_errors"], 0)
        self.assertEqual(len(ch["links"]), 1)

        link = ch["links"][0]
        self.assertEqual(link["src"], "Producer.output")
        self.assertEqual(link["dst"], "Consumer.input")
        self.assertEqual(link["delivery"], "callback")
        self.assertEqual(link["transport"], "callback")
        self.assertIsNone(link["topic"])
        self.assertEqual(link["out_count"], 3)
        self.assertEqual(link["in_count"], 3)

        system.stop()

    def test_comm_health_with_errors(self):
        from runtime.blueprint import Blueprint
        from runtime.module import Module

        class Sender(Module):
            out: Out[int]
            def setup(self): pass

        class BadReceiver(Module):
            inp: In[int]
            def setup(self):
                self.inp.subscribe(self._on_msg)
            def _on_msg(self, msg):
                raise RuntimeError("intentional")

        bp = Blueprint()
        bp.add(Sender)
        bp.add(BadReceiver)
        bp.wire("Sender", "out", "BadReceiver", "inp")
        system = bp.build()
        system.start()

        sender = system.get_module("Sender")
        sender.out.publish(1)

        ch = system.comm_health()
        self.assertEqual(ch["total_errors"], 1)
        self.assertTrue(len(ch["warnings"]) > 0)

        system.stop()


if __name__ == "__main__":
    unittest.main()
