"""Tests for DDS transport observability metrics (dds_metrics.py).

Covers:
  - DDSMetrics: publish/receive counting, rate calculation, latency, drops
  - snapshot() output structure and values
  - reset() clears state
  - Thread safety under concurrent writes
  - Environment variable switch (LINGTU_DDS_METRICS)
  - extract_timestamp() for various message shapes
  - Integration helpers (record_publish / record_receive) respect the switch
  - Global singleton
"""

import os
import sys
import threading
import time
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from runtime.transport import dds_metrics
from runtime.transport.dds_metrics import (
    DDSMetrics,
    extract_timestamp,
    get_global_metrics,
    metrics_enabled,
    record_publish,
    record_receive,
    reset_env_cache,
)


class TestDDSMetricsBasic(unittest.TestCase):
    """Core DDSMetrics behavior — counting, bytes, snapshot, reset."""

    def test_empty_snapshot(self):
        m = DDSMetrics()
        self.assertEqual(m.snapshot(), {})

    def test_publish_count(self):
        m = DDSMetrics()
        for _ in range(5):
            m.record_publish("topic_a", size_bytes=100)
        snap = m.snapshot()
        self.assertEqual(snap["topic_a"]["msg_count"], 5)
        self.assertEqual(snap["topic_a"]["publish_count"], 5)
        self.assertEqual(snap["topic_a"]["receive_count"], 0)
        self.assertEqual(snap["topic_a"]["bytes_total"], 500)
        self.assertEqual(snap["topic_a"]["publish_bytes"], 500)

    def test_receive_count(self):
        m = DDSMetrics()
        for _ in range(3):
            m.record_receive("topic_b", size_bytes=64)
        snap = m.snapshot()
        self.assertEqual(snap["topic_b"]["msg_count"], 3)
        self.assertEqual(snap["topic_b"]["receive_count"], 3)
        self.assertEqual(snap["topic_b"]["publish_count"], 0)
        self.assertEqual(snap["topic_b"]["bytes_total"], 192)
        self.assertEqual(snap["topic_b"]["receive_bytes"], 192)

    def test_mixed_publish_receive(self):
        m = DDSMetrics()
        m.record_publish("topic_c", size_bytes=10)
        m.record_publish("topic_c", size_bytes=20)
        m.record_receive("topic_c", size_bytes=30)
        snap = m.snapshot()["topic_c"]
        self.assertEqual(snap["msg_count"], 3)
        self.assertEqual(snap["publish_count"], 2)
        self.assertEqual(snap["receive_count"], 1)
        self.assertEqual(snap["bytes_total"], 60)
        self.assertEqual(snap["publish_bytes"], 30)
        self.assertEqual(snap["receive_bytes"], 30)

    def test_reset_clears(self):
        m = DDSMetrics()
        m.record_publish("topic_a", size_bytes=10)
        m.record_receive("topic_b", size_bytes=20)
        self.assertEqual(len(m.snapshot()), 2)
        m.reset()
        self.assertEqual(m.snapshot(), {})

    def test_multiple_topics_independent(self):
        m = DDSMetrics()
        m.record_publish("t1", size_bytes=1)
        m.record_receive("t2", size_bytes=2)
        m.record_publish("t3", size_bytes=3)
        snap = m.snapshot()
        self.assertEqual(set(snap.keys()), {"t1", "t2", "t3"})
        self.assertEqual(snap["t1"]["msg_count"], 1)
        self.assertEqual(snap["t2"]["msg_count"], 1)
        self.assertEqual(snap["t3"]["msg_count"], 1)

    def test_snapshot_is_a_copy(self):
        """snapshot() must return a value that doesn't change on later writes."""
        m = DDSMetrics()
        m.record_publish("topic_a", size_bytes=1)
        snap = m.snapshot()
        m.record_publish("topic_a", size_bytes=1)
        self.assertEqual(snap["topic_a"]["msg_count"], 1)
        self.assertEqual(m.snapshot()["topic_a"]["msg_count"], 2)


class TestDDSMetricsRate(unittest.TestCase):
    """Sliding-window rate calculation."""

    def test_rate_zero_with_fewer_than_two_messages(self):
        m = DDSMetrics()
        m.record_receive("t")
        self.assertEqual(m.snapshot()["t"]["msg_rate_hz"], 0.0)

    def test_rate_positive_after_multiple_messages(self):
        m = DDSMetrics()
        for _ in range(10):
            m.record_receive("t")
            time.sleep(0.01)
        rate = m.snapshot()["t"]["msg_rate_hz"]
        self.assertGreater(rate, 0.0)

    def test_rate_uses_sliding_window(self):
        """Old messages outside the window should not affect the rate."""
        m = DDSMetrics(window_size=5)
        # Fill window with slow messages
        for _ in range(5):
            m.record_receive("t")
            time.sleep(0.05)
        # Now send fast messages that push old ones out
        for _ in range(5):
            m.record_receive("t")
            time.sleep(0.001)
        rate = m.snapshot()["t"]["msg_rate_hz"]
        # Rate should be dominated by the fast messages. Windows timer
        # granularity (~15ms) caps the effective fast-message rate well below
        # the 1ms sleep target, so use a looser bound there; the slow messages
        # run at ~20Hz, so 30Hz still proves the window slid.
        threshold = 30.0 if sys.platform == "win32" else 50.0
        self.assertGreater(rate, threshold)

    def test_rate_decreases_when_no_new_messages(self):
        """The rate reflects only messages in the window, not extrapolation."""
        m = DDSMetrics(window_size=100)
        for _ in range(10):
            m.record_receive("t")
            time.sleep(0.01)
        rate1 = m.snapshot()["t"]["msg_rate_hz"]
        time.sleep(0.1)
        # Without new messages, the window timestamps don't change, so rate
        # stays the same (window is frozen).
        rate2 = m.snapshot()["t"]["msg_rate_hz"]
        self.assertAlmostEqual(rate1, rate2, places=1)


class TestDDSMetricsLatency(unittest.TestCase):
    """Latency calculation from publish timestamps."""

    def test_latency_zero_when_no_timestamp(self):
        m = DDSMetrics()
        m.record_receive("t")
        self.assertEqual(m.snapshot()["t"]["last_latency_ms"], 0.0)

    def test_latency_positive_with_timestamp(self):
        m = DDSMetrics()
        pub_ts = time.time() - 0.01  # 10ms ago
        m.record_receive("t", timestamp=pub_ts)
        latency = m.snapshot()["t"]["last_latency_ms"]
        self.assertGreater(latency, 5.0)
        self.assertLess(latency, 100.0)

    def test_latency_zero_for_future_timestamp(self):
        """Negative latency (future timestamp) is clamped to 0."""
        m = DDSMetrics()
        m.record_receive("t", timestamp=time.time() + 10)
        self.assertEqual(m.snapshot()["t"]["last_latency_ms"], 0.0)

    def test_latency_updates_on_each_receive(self):
        m = DDSMetrics()
        m.record_receive("t", timestamp=time.time() - 0.05)
        lat1 = m.snapshot()["t"]["last_latency_ms"]
        m.record_receive("t", timestamp=time.time() - 0.01)
        lat2 = m.snapshot()["t"]["last_latency_ms"]
        self.assertGreater(lat1, lat2)


class TestDDSMetricsDrops(unittest.TestCase):
    """Drop detection via sequence number gaps."""

    def test_no_drops_with_sequential_seq(self):
        m = DDSMetrics()
        for s in range(10):
            m.record_receive("t", seq=s)
        self.assertEqual(m.snapshot()["t"]["drop_count"], 0)

    def test_drops_detected_on_gap(self):
        m = DDSMetrics()
        m.record_receive("t", seq=0)
        m.record_receive("t", seq=1)
        m.record_receive("t", seq=5)
        # Gap: 2,3,4 missing -> 3 drops
        self.assertEqual(m.snapshot()["t"]["drop_count"], 3)

    def test_no_drop_check_without_seq(self):
        m = DDSMetrics()
        m.record_receive("t")
        m.record_receive("t")
        self.assertEqual(m.snapshot()["t"]["drop_count"], 0)

    def test_drop_count_resets_on_reset(self):
        m = DDSMetrics()
        m.record_receive("t", seq=0)
        m.record_receive("t", seq=10)
        self.assertEqual(m.snapshot()["t"]["drop_count"], 9)
        m.reset()
        m.record_receive("t", seq=0)
        m.record_receive("t", seq=1)
        self.assertEqual(m.snapshot()["t"]["drop_count"], 0)


class TestExtractTimestamp(unittest.TestCase):
    """extract_timestamp() for different message shapes."""

    def test_none_for_plain_object(self):
        obj = SimpleNamespace(x=1)
        self.assertIsNone(extract_timestamp(obj))

    def test_timestamp_attribute(self):
        ts = time.time() - 1.0
        msg = SimpleNamespace(timestamp=ts)
        self.assertAlmostEqual(extract_timestamp(msg), ts)

    def test_ts_attribute(self):
        ts = time.time() - 2.0
        msg = SimpleNamespace(ts=ts)
        self.assertAlmostEqual(extract_timestamp(msg), ts)

    def test_header_stamp_sec_nanosec(self):
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1700000000, nanosec=500_000_000)))
        ts = extract_timestamp(msg)
        self.assertAlmostEqual(ts, 1700000000.5)

    def test_dict_with_ts(self):
        ts = time.time() - 0.5
        msg = {"ts": ts, "data": "hello"}
        self.assertAlmostEqual(extract_timestamp(msg), ts)

    def test_dict_with_timestamp(self):
        ts = time.time() - 0.5
        msg = {"timestamp": ts}
        self.assertAlmostEqual(extract_timestamp(msg), ts)

    def test_none_message(self):
        self.assertIsNone(extract_timestamp(None))

    def test_header_stamp_zero_returns_none(self):
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)))
        self.assertIsNone(extract_timestamp(msg))

    def test_broken_object_does_not_raise(self):
        class BadAttr:
            @property
            def timestamp(self):
                raise RuntimeError("boom")

        self.assertIsNone(extract_timestamp(BadAttr()))


class TestEnvSwitch(unittest.TestCase):
    """LINGTU_DDS_METRICS environment variable control."""

    def setUp(self):
        reset_env_cache()

    def tearDown(self):
        reset_env_cache()

    def test_disabled_by_default(self):
        os.environ.pop("LINGTU_DDS_METRICS", None)
        self.assertFalse(metrics_enabled())

    def test_enabled_with_1(self):
        os.environ["LINGTU_DDS_METRICS"] = "1"
        self.assertTrue(metrics_enabled())

    def test_enabled_with_true(self):
        os.environ["LINGTU_DDS_METRICS"] = "true"
        self.assertTrue(metrics_enabled())

    def test_enabled_with_yes(self):
        os.environ["LINGTU_DDS_METRICS"] = "yes"
        self.assertTrue(metrics_enabled())

    def test_enabled_with_on(self):
        os.environ["LINGTU_DDS_METRICS"] = "on"
        self.assertTrue(metrics_enabled())

    def test_disabled_with_0(self):
        os.environ["LINGTU_DDS_METRICS"] = "0"
        self.assertFalse(metrics_enabled())

    def test_disabled_with_empty(self):
        os.environ["LINGTU_DDS_METRICS"] = ""
        self.assertFalse(metrics_enabled())

    def test_disabled_with_random(self):
        os.environ["LINGTU_DDS_METRICS"] = "maybe"
        self.assertFalse(metrics_enabled())

    def test_disabled_with_case_insensitive_true(self):
        os.environ["LINGTU_DDS_METRICS"] = "TRUE"
        self.assertTrue(metrics_enabled())

    def test_cache_is_used(self):
        os.environ["LINGTU_DDS_METRICS"] = "1"
        self.assertTrue(metrics_enabled())
        os.environ["LINGTU_DDS_METRICS"] = "0"
        # Still True because cached
        self.assertTrue(metrics_enabled())
        reset_env_cache()
        self.assertFalse(metrics_enabled())


class TestIntegrationHelpers(unittest.TestCase):
    """record_publish / record_receive respect the env switch."""

    def setUp(self):
        reset_env_cache()
        # Use a fresh global metrics instance to avoid cross-test pollution.
        dds_metrics._global_metrics = DDSMetrics()

    def tearDown(self):
        dds_metrics._global_metrics = None
        reset_env_cache()
        os.environ.pop("LINGTU_DDS_METRICS", None)

    def test_record_publish_noop_when_disabled(self):
        os.environ.pop("LINGTU_DDS_METRICS", None)
        record_publish("t", size_bytes=10)
        self.assertEqual(get_global_metrics().snapshot(), {})

    def test_record_receive_noop_when_disabled(self):
        os.environ.pop("LINGTU_DDS_METRICS", None)
        record_receive("t", msg=SimpleNamespace(timestamp=time.time()))
        self.assertEqual(get_global_metrics().snapshot(), {})

    def test_record_publish_works_when_enabled(self):
        os.environ["LINGTU_DDS_METRICS"] = "1"
        record_publish("t", size_bytes=42)
        snap = get_global_metrics().snapshot()
        self.assertEqual(snap["t"]["msg_count"], 1)
        self.assertEqual(snap["t"]["publish_count"], 1)
        self.assertEqual(snap["t"]["bytes_total"], 42)

    def test_record_receive_works_when_enabled(self):
        os.environ["LINGTU_DDS_METRICS"] = "1"
        pub_ts = time.time() - 0.005
        msg = SimpleNamespace(timestamp=pub_ts)
        record_receive("t", msg=msg, size_bytes=16)
        snap = get_global_metrics().snapshot()
        self.assertEqual(snap["t"]["msg_count"], 1)
        self.assertEqual(snap["t"]["receive_count"], 1)
        self.assertEqual(snap["t"]["bytes_total"], 16)
        self.assertGreater(snap["t"]["last_latency_ms"], 0.0)

    def test_record_receive_with_none_msg(self):
        os.environ["LINGTU_DDS_METRICS"] = "1"
        record_receive("t", msg=None)
        snap = get_global_metrics().snapshot()
        self.assertEqual(snap["t"]["msg_count"], 1)
        self.assertEqual(snap["t"]["last_latency_ms"], 0.0)


class TestGlobalSingleton(unittest.TestCase):
    """get_global_metrics() returns the same instance."""

    def test_singleton_identity(self):
        dds_metrics._global_metrics = None
        a = get_global_metrics()
        b = get_global_metrics()
        self.assertIs(a, b)

    def test_singleton_is_ddsmetrics(self):
        dds_metrics._global_metrics = None
        m = get_global_metrics()
        self.assertIsInstance(m, DDSMetrics)


class TestThreadSafety(unittest.TestCase):
    """Concurrent writes must not corrupt state or raise."""

    def test_concurrent_publish_and_receive(self):
        m = DDSMetrics(window_size=1000)
        num_threads = 8
        per_thread = 500
        errors: list[Exception] = []

        def worker(tid: int):
            try:
                topic = f"topic_{tid % 3}"
                for i in range(per_thread):
                    if i % 2 == 0:
                        m.record_publish(topic, size_bytes=i)
                    else:
                        m.record_receive(topic, size_bytes=i, seq=i)
            except Exception as exc:
                errors.append(exc)

        threads = [threading.Thread(target=worker, args=(t,)) for t in range(num_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.assertEqual(errors, [])

        total_expected = num_threads * per_thread
        snap = m.snapshot()
        actual_total = sum(s["msg_count"] for s in snap.values())
        self.assertEqual(actual_total, total_expected)

    def test_concurrent_snapshot_during_writes(self):
        m = DDSMetrics(window_size=500)
        stop = threading.Event()
        errors: list[Exception] = []

        def writer():
            try:
                i = 0
                while not stop.is_set():
                    m.record_publish("t", size_bytes=i % 100)
                    m.record_receive("t", size_bytes=i % 50, seq=i)
                    i += 1
            except Exception as exc:
                errors.append(exc)

        def snapshotter():
            try:
                while not stop.is_set():
                    m.snapshot()
                    time.sleep(0.001)
            except Exception as exc:
                errors.append(exc)

        w = threading.Thread(target=writer)
        s = threading.Thread(target=snapshotter)
        w.start()
        s.start()
        time.sleep(0.2)
        stop.set()
        w.join(timeout=2)
        s.join(timeout=2)

        self.assertEqual(errors, [])
        self.assertGreater(m.snapshot()["t"]["msg_count"], 0)

    def test_concurrent_reset_during_writes(self):
        m = DDSMetrics(window_size=500)
        stop = threading.Event()
        errors: list[Exception] = []

        def writer():
            try:
                i = 0
                while not stop.is_set():
                    m.record_receive("t", size_bytes=i, seq=i)
                    i += 1
            except Exception as exc:
                errors.append(exc)

        def resetter():
            try:
                while not stop.is_set():
                    m.reset()
                    time.sleep(0.005)
            except Exception as exc:
                errors.append(exc)

        w = threading.Thread(target=writer)
        r = threading.Thread(target=resetter)
        w.start()
        r.start()
        time.sleep(0.2)
        stop.set()
        w.join(timeout=2)
        r.join(timeout=2)

        self.assertEqual(errors, [])


class TestErrorSafety(unittest.TestCase):
    """Metrics collection must never propagate exceptions."""

    def test_record_publish_with_bad_size_does_not_raise(self):
        m = DDSMetrics()

        class BadInt:
            def __add__(self, other):
                raise RuntimeError("nope")

            __radd__ = __add__

        # size_bytes as a non-int that breaks addition
        m.record_publish("t", size_bytes=BadInt())  # type: ignore[arg-type]
        # Should not raise; msg_count still incremented (size may be 0)
        snap = m.snapshot()
        self.assertIn("t", snap)

    def test_record_receive_with_bad_timestamp_does_not_raise(self):
        m = DDSMetrics()

        class BadFloat:
            def __float__(self):
                raise ValueError("nope")

        m.record_receive("t", timestamp=BadFloat())  # type: ignore[arg-type]
        snap = m.snapshot()
        self.assertEqual(snap["t"]["last_latency_ms"], 0.0)
        self.assertEqual(snap["t"]["msg_count"], 1)


if __name__ == "__main__":
    unittest.main()
if __name__ == "__main__":
    unittest.main()
