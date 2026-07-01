"""Tests for Module lifecycle hardening — idempotent stop, reference cleanup."""

import threading
import unittest

from runtime import Blueprint, In, Module, Out


class SimpleSource(Module, layer=1):
    data: Out[int]

class SimpleSink(Module, layer=3):
    data: In[int]


class TestIdempotentStop(unittest.TestCase):

    def test_double_stop_no_error(self):
        mod = SimpleSource()
        mod.start()
        mod.stop()
        mod.stop()  # second call must not raise

    def test_stop_sets_closed(self):
        mod = SimpleSource()
        mod.start()
        self.assertFalse(mod._closed)
        mod.stop()
        self.assertTrue(mod._closed)

    def test_stop_sets_not_running(self):
        mod = SimpleSource()
        mod.start()
        self.assertTrue(mod.running)
        mod.stop()
        self.assertFalse(mod.running)

    def test_concurrent_stop_safe(self):
        """Two threads calling stop() simultaneously must not crash."""
        mod = SimpleSource()
        mod.start()
        errors = []

        def do_stop():
            try:
                mod.stop()
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=do_stop) for _ in range(10)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.assertEqual(errors, [])
        self.assertTrue(mod._closed)


class TestReferenceCleanup(unittest.TestCase):

    def test_out_callbacks_cleared_after_stop(self):
        mod = SimpleSource()
        mod.data._add_callback(lambda x: None)
        self.assertEqual(mod.data.callback_count, 1)
        mod.stop()
        self.assertEqual(mod.data.callback_count, 0)

    def test_out_transport_cleared_after_stop(self):
        from runtime.transport.local import LocalTransport
        mod = SimpleSource()
        mod.data._bind_transport(LocalTransport(), "/test")
        self.assertIsNotNone(mod.data._transport)
        mod.stop()
        self.assertIsNone(mod.data._transport)

    def test_in_subscriber_cleared_after_stop(self):
        mod = SimpleSink()
        mod.data.subscribe(lambda x: None)
        self.assertTrue(mod.data.connected)
        mod.stop()
        self.assertFalse(mod.data.connected)

    def test_system_stop_clears_modules(self):
        handle = (
            Blueprint()
            .add(SimpleSource)
            .add(SimpleSink)
            .auto_wire()
            .build()
        )
        handle.start()
        self.assertGreater(len(handle._modules), 0)
        handle.stop()
        self.assertEqual(len(handle._modules), 0)

    def test_data_flow_works_before_stop_breaks_after(self):
        """After stop(), publishing no longer reaches the sink."""
        handle = (
            Blueprint()
            .add(SimpleSource)
            .add(SimpleSink)
            .auto_wire()
            .build()
        )
        handle.start()

        src = handle.get_module("SimpleSource")
        sink = handle.get_module("SimpleSink")

        received = []
        sink.data.subscribe(received.append)
        src.data.publish(42)
        self.assertEqual(received, [42])

        handle.stop()
        # After stop, modules dict is cleared — can't get_module anymore
        self.assertEqual(len(handle._modules), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
