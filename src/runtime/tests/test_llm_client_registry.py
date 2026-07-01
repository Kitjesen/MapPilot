from __future__ import annotations

import pytest

from runtime.registry import list_plugins, register, restore, snapshot
from decision.llm.llm_client import (
    LLMConfig,
    MockLLMClient,
    available_llm_backends,
    create_llm_client,
    resolve_llm_backend,
)


def test_llm_client_registry_names_and_aliases_are_visible():
    assert {"openai", "claude", "qwen", "moonshot", "mock"} <= set(
        list_plugins("llm_client")
    )
    assert resolve_llm_backend("kimi") == "moonshot"
    assert resolve_llm_backend("offline") == "mock"
    assert "kimi" in available_llm_backends()


def test_create_llm_client_uses_registered_provider():
    saved = snapshot()
    created = object()

    try:
        @register("llm_client", "fake")
        class FakeLLMProvider:
            @staticmethod
            def create(_config):
                return created

        assert create_llm_client(LLMConfig(backend="fake")) is created
    finally:
        restore(saved)


def test_create_llm_client_preserves_mock_backend():
    client = create_llm_client(LLMConfig(backend="mock"))

    assert isinstance(client, MockLLMClient)


def test_create_llm_client_unknown_backend_lists_aliases():
    with pytest.raises(ValueError, match="Unknown LLM backend: 'bogus'"):
        create_llm_client(LLMConfig(backend="bogus"))
    with pytest.raises(ValueError, match="kimi"):
        create_llm_client(LLMConfig(backend="bogus"))
