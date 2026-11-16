"""Conversation-only provider choices; each turn owns its resolved connection."""

import os
from dataclasses import replace

from decision.llm.acp import ACP_AGENTS
from decision.llm.client import LLMConfig

PROVIDERS = {
    "qwen": ("https://dashscope.aliyuncs.com/compatible-mode/v1", "DASHSCOPE_API_KEY", {
        "qwen-plus": "Qwen Plus",
        "qwen-flash": "Qwen Flash",
        "qwen3-max": "Qwen3 Max",
    }),
    "openai": ("https://api.openai.com/v1", "OPENAI_API_KEY", {
        "gpt-6.1-sol": "GPT-6.1 Sol",
    }),
    "claude": ("https://api.anthropic.com/v1/", "ANTHROPIC_API_KEY", {
        "claude-sonnet-5-5": "Claude Sonnet 5.5",
        "claude-opus-5-5": "Claude Opus 5.5",
        "claude-haiku-5-5": "Claude Haiku 5.5",
    }),
    "grok": ("https://api.x.ai/v1", "XAI_API_KEY", {
        "grok-4.7": "Grok 4.7 · xAI",
    }),
}

ACP_MODELS = {model: spec["label"] for model, spec in ACP_AGENTS.items()}


def conversation_config(default: LLMConfig, model: str | None) -> LLMConfig:
    from decision.llm.acp import acp_agent_config
    from decision.llm.codex import BASE_URL, MODELS

    if model in MODELS:
        return replace(default, backend="codex", model=model.removeprefix("codex:"), base_url=BASE_URL,
                       api_key_env="", timeout_sec=120, max_retries=0)
    if model in ACP_AGENTS:
        return acp_agent_config(default, model)
    if not model or model == default.model:
        return default
    for backend, (url, key, models) in PROVIDERS.items():
        if model in models:
            return replace(default, backend=backend, model=model, base_url=url,
                           api_key_env=key, timeout_sec=120, max_retries=0)
    return default


def provider_choices(default: LLMConfig):
    for _, _, models in PROVIDERS.values():
        for model, label in models.items():
            config = conversation_config(default, model)
            yield config, model, label, bool(os.environ.get(config.api_key_env))
    from decision.llm.acp import acp_agent_choices
    from decision.llm.codex import MODELS
    from decision.llm.codex_app_server import app_server

    try:
        connected = app_server.status()["connected"]
    except Exception:
        connected = False
    for model, label in MODELS.items():
        yield conversation_config(default, model), model, label, connected
    yield from acp_agent_choices(default)
