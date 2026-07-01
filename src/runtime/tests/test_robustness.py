"""Tests for runtime.utils.robustness — retry, retry_async, async_timeout, gpu_safe."""

from __future__ import annotations

import asyncio
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from runtime.utils.robustness import async_timeout, gpu_safe, retry, retry_async

# ── retry (sync) ─────────────────────────────────────────────────────────────

class TestRetry(unittest.TestCase):

    def test_succeeds_first_try(self):
        call_count = 0

        @retry(max_attempts=3, backoff=0.01)
        def f():
            nonlocal call_count
            call_count += 1
            return "ok"

        self.assertEqual(f(), "ok")
        self.assertEqual(call_count, 1)

    def test_retries_on_failure(self):
        call_count = 0

        @retry(max_attempts=3, backoff=0.01)
        def f():
            nonlocal call_count
            call_count += 1
            if call_count < 3:
                raise ValueError("not yet")
            return "ok"

        self.assertEqual(f(), "ok")
        self.assertEqual(call_count, 3)

    def test_raises_after_max_attempts(self):
        @retry(max_attempts=2, backoff=0.01)
        def f():
            raise RuntimeError("always fails")

        with self.assertRaises(RuntimeError):
            f()

    def test_only_catches_specified_exceptions(self):
        call_count = 0

        @retry(max_attempts=3, backoff=0.01, on=(ValueError,))
        def f():
            nonlocal call_count
            call_count += 1
            raise TypeError("not a ValueError")

        with self.assertRaises(TypeError):
            f()
        self.assertEqual(call_count, 1)  # no retry

    def test_jitter_varies_wait_time(self):
        """With jitter=True, backoff intervals vary."""
        waits = []

        def mock_sleep(t):
            waits.append(t)
            # Don't actually sleep in test

        import runtime.utils.robustness as mod
        old_sleep = mod._sleep
        mod._sleep = mock_sleep
        try:
            @retry(max_attempts=4, backoff=1.0, jitter=True)
            def f():
                raise ValueError("fail")

            with self.assertRaises(ValueError):
                f()
        finally:
            mod._sleep = old_sleep

        # 3 waits (4 attempts - 1)
        self.assertEqual(len(waits), 3)
        # With jitter, waits should not be exact powers of 2
        # Base waits would be 1.0, 2.0, 4.0; jitter makes them ±25%
        for i, w in enumerate(waits):
            base = 1.0 * (2 ** i)
            self.assertGreater(w, base * 0.7)
            self.assertLess(w, base * 1.3)

    def test_no_jitter(self):
        waits = []

        import runtime.utils.robustness as mod
        old_sleep = mod._sleep
        mod._sleep = lambda t: waits.append(t)
        try:
            @retry(max_attempts=3, backoff=1.0, jitter=False)
            def f():
                raise ValueError("fail")

            with self.assertRaises(ValueError):
                f()
        finally:
            mod._sleep = old_sleep

        self.assertEqual(waits, [1.0, 2.0])


# ── retry_async ──────────────────────────────────────────────────────────────

class TestRetryAsync(unittest.TestCase):

    def test_async_retry_succeeds(self):
        call_count = 0

        @retry_async(max_attempts=3, backoff=0.01)
        async def f():
            nonlocal call_count
            call_count += 1
            if call_count < 2:
                raise ValueError("not yet")
            return "ok"

        result = asyncio.get_event_loop().run_until_complete(f())
        self.assertEqual(result, "ok")
        self.assertEqual(call_count, 2)

    def test_async_retry_raises_after_max(self):
        @retry_async(max_attempts=2, backoff=0.01)
        async def f():
            raise RuntimeError("always fails")

        with self.assertRaises(RuntimeError):
            asyncio.get_event_loop().run_until_complete(f())


# ── async_timeout ────────────────────────────────────────────────────────────

class TestAsyncTimeout(unittest.TestCase):

    def test_returns_value_within_timeout(self):
        @async_timeout(1.0)
        async def f():
            return 42

        result = asyncio.get_event_loop().run_until_complete(f())
        self.assertEqual(result, 42)

    def test_returns_timeout_value_on_timeout(self):
        @async_timeout(0.1, timeout_return="TIMEOUT")
        async def f():
            await asyncio.sleep(10.0)
            return "never"

        result = asyncio.get_event_loop().run_until_complete(f())
        self.assertEqual(result, "TIMEOUT")

    def test_default_timeout_return_is_none(self):
        @async_timeout(0.05)
        async def f():
            await asyncio.sleep(10.0)

        result = asyncio.get_event_loop().run_until_complete(f())
        self.assertIsNone(result)


# ── gpu_safe ─────────────────────────────────────────────────────────────────

class TestGpuSafe(unittest.TestCase):

    def test_normal_return(self):
        @gpu_safe(fallback="fallback")
        def f():
            return "ok"

        self.assertEqual(f(), "ok")

    def test_runtime_error_returns_fallback(self):
        @gpu_safe(fallback="safe")
        def f():
            raise RuntimeError("CUDA out of memory")

        self.assertEqual(f(), "safe")

    def test_non_cuda_runtime_error_returns_fallback(self):
        @gpu_safe(fallback="safe")
        def f():
            raise RuntimeError("generic runtime error")

        self.assertEqual(f(), "safe")

    def test_callable_fallback(self):
        @gpu_safe(fallback=lambda: [1, 2, 3])
        def f():
            raise RuntimeError("boom")

        self.assertEqual(f(), [1, 2, 3])

    def test_non_runtime_error_propagates(self):
        @gpu_safe(fallback="safe")
        def f():
            raise TypeError("not a RuntimeError")

        with self.assertRaises(TypeError):
            f()


if __name__ == "__main__":
    unittest.main()
