"""Decision module."""

import asyncio
import concurrent.futures
import json
import unittest
import warnings
from unittest.mock import MagicMock, patch

from decision.llm.client import (
    LLMConfig,
    LLMError,
    MockLLMClient,
    create_llm_client,
)


class TestMockBackendDirect(unittest.TestCase):
    """Test documentation."""

    def test_mock_client_creation(self):
        """Test documentation."""
        config = LLMConfig(backend="mock", model="mock-model")
        client = create_llm_client(config)
        self.assertIsInstance(client, MockLLMClient)

    def test_mock_client_is_available(self):
        """Test documentation."""
        config = LLMConfig(backend="mock")
        client = MockLLMClient(config)
        self.assertTrue(client.is_available())

    def test_mock_client_returns_valid_json(self):
        """Test documentation."""
        config = LLMConfig(backend="mock")
        client = MockLLMClient(config)

        messages = [
            {"role": "system", "content": "You are a navigation assistant."},
            {"role": "user", "content": "鎵惧埌妞呭瓙"},
        ]

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(client.chat(messages))
        finally:
            loop.close()

        self.assertIsInstance(result, str)
        parsed = json.loads(result)
        self.assertIn("action", parsed)
        self.assertIn("confidence", parsed)

    def test_mock_client_aliases(self):
        """Test documentation."""
        for alias in ("mock", "offline", "test"):
            config = LLMConfig(backend=alias)
            client = create_llm_client(config)
            self.assertIsInstance(client, MockLLMClient, f"alias '{alias}' failed")


class TestAsyncEventLoop(unittest.TestCase):
    """Test documentation."""

    def test_chat_in_new_event_loop_no_deprecation(self):
        """Test documentation."""
        # Regression: avoid asyncio.get_event_loop() deprecation warnings.
        config = LLMConfig(backend="mock")
        client = MockLLMClient(config)
        messages = [{"role": "user", "content": "go to kitchen"}]

        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            loop = asyncio.new_event_loop()
            try:
                result = loop.run_until_complete(client.chat(messages))
            finally:
                loop.close()

            deprecation_warnings = [
                x for x in w if issubclass(x.category, DeprecationWarning) and "event loop" in str(x.message).lower()
            ]
            self.assertEqual(len(deprecation_warnings), 0, f"Got unexpected DeprecationWarning: {deprecation_warnings}")
        self.assertIsNotNone(result)

    def test_chat_in_threadpool_executor(self):
        """Test documentation."""
        # Regression: a worker thread with no default event loop still works.
        config = LLMConfig(backend="mock")
        client = MockLLMClient(config)
        messages = [{"role": "user", "content": "find the table"}]

        def _thread_call():
            loop = asyncio.new_event_loop()
            try:
                return loop.run_until_complete(client.chat(messages))
            finally:
                loop.close()

        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(_thread_call)
            result = future.result(timeout=5.0)

        self.assertIsNotNone(result)
        parsed = json.loads(result)
        self.assertIn("action", parsed)

    def test_multiple_concurrent_calls(self):
        """Test documentation."""
        config = LLMConfig(backend="mock")
        client = MockLLMClient(config)

        async def _multi_call():
            tasks = [client.chat([{"role": "user", "content": f"go to room {i}"}]) for i in range(5)]
            return await asyncio.gather(*tasks)

        loop = asyncio.new_event_loop()
        try:
            results = loop.run_until_complete(_multi_call())
        finally:
            loop.close()

        self.assertEqual(len(results), 5)
        for r in results:
            parsed = json.loads(r)
            self.assertIn("action", parsed)


class TestLLMTimeout(unittest.TestCase):
    """Test documentation."""

    def test_timeout_raises_llm_error(self):
        """Test documentation."""
        config = LLMConfig(backend="openai", timeout_sec=0.001, max_retries=0)

        with patch.dict("sys.modules", {"openai": MagicMock()}):
            from decision.llm.client import OpenAIClient

            client = OpenAIClient(config)

            mock_openai = MagicMock()

            async def _raise_timeout(**kw):
                raise TimeoutError("Connection timed out")

            mock_openai.chat.completions.create = _raise_timeout
            client._client = mock_openai

            loop = asyncio.new_event_loop()
            try:
                with self.assertRaises(LLMError) as ctx:
                    loop.run_until_complete(client.chat([{"role": "user", "content": "hello"}]))
                self.assertIn("failed", str(ctx.exception).lower())
            finally:
                loop.close()

    def test_mock_client_never_times_out(self):
        """Test documentation."""
        config = LLMConfig(backend="mock", timeout_sec=0.001)
        client = MockLLMClient(config)

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(client.chat([{"role": "user", "content": "find bed"}]))
        finally:
            loop.close()

        self.assertIsNotNone(result)
        self.assertIn("action", json.loads(result))


if __name__ == "__main__":
    unittest.main()
