"""Tests for enterprise-grade features: redaction, preflight, graceful shutdown, port metrics."""

from __future__ import annotations

import logging
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from runtime.module import Module
from runtime.stream import In, Out

# ── RedactSecretsFilter ──────────────────────────────────────────────────────

class TestRedactSecretsFilter(unittest.TestCase):

    def test_redacts_known_secret(self):
        from runtime.utils.redact import RedactSecretsFilter

        os.environ["TEST_API_KEY"] = "super-secret-value-12345"
        try:
            f = RedactSecretsFilter()
            record = logging.LogRecord(
                "test", logging.INFO, "", 0,
                "Calling API with key super-secret-value-12345", None, None,
            )
            f.filter(record)
            self.assertNotIn("super-secret-value-12345", record.msg)
            self.assertIn("***", record.msg)
        finally:
            del os.environ["TEST_API_KEY"]

    def test_ignores_short_values(self):
        from runtime.utils.redact import RedactSecretsFilter

        os.environ["TEST_SECRET"] = "short"  # < 8 chars
        try:
            f = RedactSecretsFilter()
            record = logging.LogRecord(
                "test", logging.INFO, "", 0,
                "value is short", None, None,
            )
            f.filter(record)
            self.assertIn("short", record.msg)
        finally:
            del os.environ["TEST_SECRET"]

    def test_passes_through_non_secret(self):
        from runtime.utils.redact import RedactSecretsFilter

        f = RedactSecretsFilter()
        record = logging.LogRecord(
            "test", logging.INFO, "", 0,
            "Normal log message", None, None,
        )
        result = f.filter(record)
        self.assertTrue(result)
        self.assertEqual(record.getMessage(), "Normal log message")


# ── Module.preflight() ───────────────────────────────────────────────────────

class _FailingPreflight(Module, layer=0):
    out: Out[int]

    def preflight(self):
        return "missing required resource"


class _PassingPreflight(Module, layer=0):
    out: Out[int]

    def preflight(self):
        return None


class TestModulePreflight(unittest.TestCase):

    def test_default_preflight_returns_none(self):
        m = Module()
        self.assertIsNone(m.preflight())

    def test_failing_preflight_returns_reason(self):
        m = _FailingPreflight()
        self.assertEqual(m.preflight(), "missing required resource")

    def test_passing_preflight_returns_none(self):
        m = _PassingPreflight()
        self.assertIsNone(m.preflight())

    def test_preflight_integrated_in_system_start(self):
        """Failing preflight → module marked as failed, not started."""
        from runtime.blueprint import Blueprint
        bp = Blueprint()
        bp.add(_FailingPreflight, name="Fail")
        bp.add(_PassingPreflight, name="Pass")
        system = bp.build()
        system.start()

        # Failed module key is class name, not the `name` param
        failed_keys = list(system._failed_modules.keys())
        self.assertTrue(any("FailingPreflight" in k for k in failed_keys),
                        f"Expected failing module in {failed_keys}")
        self.assertFalse(any("PassingPreflight" in k for k in failed_keys))
        system.stop()


# ── Graceful shutdown with timeout ───────────────────────────────────────────

class _HangingModule(Module, layer=0):
    out: Out[int]

    def stop(self):
        time.sleep(30)  # hang forever (in test context)
        super().stop()


class TestGracefulShutdown(unittest.TestCase):

    def test_hung_module_does_not_block_shutdown(self):
        from runtime.blueprint import Blueprint
        bp = Blueprint()
        bp.add(_HangingModule, name="Hang")
        system = bp.build()
        system.start()

        t0 = time.time()
        system.stop(timeout_per_module=0.5)
        elapsed = time.time() - t0

        self.assertLess(elapsed, 3.0,
                        f"Shutdown took {elapsed:.1f}s — should have timed out at 0.5s")


# ── In port latency percentiles ──────────────────────────────────────────────

class TestPortLatencyPercentiles(unittest.TestCase):

    def test_empty_returns_empty_dict(self):
        port = In(int, "test")
        self.assertEqual(port.latency_percentiles(), {})

    def test_records_after_deliver(self):
        port = In(int, "test")
        values = []
        port.subscribe(lambda x: values.append(x))

        for i in range(10):
            port._deliver(i)

        pct = port.latency_percentiles()
        self.assertIn("p50_ms", pct)
        self.assertIn("p95_ms", pct)
        self.assertIn("p99_ms", pct)
        self.assertIn("max_ms", pct)
        self.assertEqual(pct["samples"], 10)

    def test_circular_buffer_wraps(self):
        port = In(int, "test")
        port.subscribe(lambda x: None)

        # Deliver more than window size
        for i in range(300):
            port._deliver(i)

        pct = port.latency_percentiles()
        self.assertEqual(pct["samples"], In._LATENCY_WINDOW)

    def test_latency_in_port_summary(self):
        """port_summary() includes latency percentiles."""

        class _M(Module, layer=0):
            data: In[int]
            out: Out[int]

        m = _M()
        m.setup()
        m.data.subscribe(lambda x: None)
        m.data._deliver(42)

        summary = m.port_summary()
        self.assertIn("latency", summary["ports_in"]["data"])
        self.assertIn("p50_ms", summary["ports_in"]["data"]["latency"])


if __name__ == "__main__":
    unittest.main()
