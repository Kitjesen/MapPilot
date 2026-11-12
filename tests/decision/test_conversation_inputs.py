"""Conversation selections and uploaded content reach the model protocol."""

import base64
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from decision.llm.client import LLMConfig
from decision.tasks import conversation_inputs as inputs


@pytest.fixture
def config():
    return LLMConfig(backend="kimi", model="kimi-k2.6", api_key_env="MOONSHOT_API_KEY",
                     base_url="https://api.moonshot.cn/v1")


def test_qwen_connection_reasoning_and_attachment_contract(config, monkeypatch):
    from decision.llm.conversation_models import conversation_config, provider_choices
    monkeypatch.setenv("DASHSCOPE_API_KEY", "test-only")
    for model in ("qwen-plus", "qwen-flash", "qwen3-max"):
        selected = conversation_config(config, model)
        assert selected.backend == "qwen"
        assert selected.base_url == "https://dashscope.aliyuncs.com/compatible-mode/v1"
        assert selected.api_key_env == "DASHSCOPE_API_KEY"
        inputs.validate_conversation_selection(config, model, "default", [])
        with pytest.raises(ValueError, match="不支持图片"):
            inputs.validate_conversation_selection(config, model, "default", [{"mime_type": "image/png"}])
    qwen = conversation_config(config, "qwen-plus")
    assert inputs.conversation_model_settings(qwen, qwen.model, "on").extra_body == {"enable_thinking": True}
    assert inputs.conversation_model_settings(qwen, qwen.model, "default").extra_body == {"enable_thinking": False}
    monkeypatch.delenv("DASHSCOPE_API_KEY")
    assert all(not row[3] for row in provider_choices(config) if row[0].backend == "qwen")
    with pytest.raises(ValueError, match="DASHSCOPE_API_KEY"):
        inputs.validate_conversation_selection(config, "qwen-plus", "default", [])


def test_options_use_account_models_and_each_models_supported_reasoning(config, monkeypatch):
    client = SimpleNamespace(models=SimpleNamespace(list=lambda: SimpleNamespace(data=[
        SimpleNamespace(id="kimi-k2.6"), SimpleNamespace(id="kimi-k3"), SimpleNamespace(id="unsupported-model"),
    ])))
    monkeypatch.setattr(inputs, "_client", lambda _config: nullcontext(client))
    options = inputs.get_conversation_options(config)
    assert [model["id"] for model in options["models"] if model["provider"] == "moonshot"] == ["kimi-k2.6", "kimi-k3"]
    assert [value["id"] for value in options["models"][0]["reasoning"]] == ["default", "off", "on"]
    assert [value["id"] for value in options["models"][1]["reasoning"]] == ["default", "low", "high", "max"]


@pytest.mark.parametrize("model,reasoning,expected", [
    ("kimi-k2.6", "off", {"thinking": {"type": "disabled"}}),
    ("kimi-k2.6", "on", {"thinking": {"type": "enabled", "keep": "all"}}),
    ("kimi-k3", "high", {"reasoning_effort": "high"}),
    ("kimi-k3.1-pro-highspeed", "default", {"reasoning_effort": "max"}),
])
def test_provider_settings_preserve_real_thinking_controls(config, model, reasoning, expected):
    pytest.importorskip("agents")
    inputs.validate_conversation_selection(config, model, reasoning, [])
    settings = inputs.conversation_model_settings(config, model, reasoning)
    assert settings.extra_body == expected
    assert settings.include_usage is True
    assert settings.temperature is None
    assert settings.parallel_tool_calls is False


def test_invalid_model_or_thinking_mode_is_rejected(config):
    for model, reasoning in [("other-provider-model", "default"), ("kimi-k2.6", "max"), ("kimi-k3", "off")]:
        with pytest.raises(ValueError):
            inputs.validate_conversation_selection(config, model, reasoning, [])


def test_text_attachment_is_available_in_model_input(config):
    data = "地图说明\n入口在南侧。".encode()
    attachment = inputs.prepare_attachment(config, "说明.md", "text/markdown", base64.b64encode(data).decode())
    assert attachment["data"] == data
    assert attachment["mime_type"] == "text/plain"
    content = inputs.conversation_user_input("入口在哪里？", [attachment])["content"]
    assert content[0]["text"] == "入口在哪里？"
    assert "入口在南侧" in content[1]["text"]
    assert inputs.conversation_user_input("hello") == {"role": "user", "content": "hello"}


def test_image_uses_native_vision_content(config):
    data = base64.b64encode(b"\x89PNG\r\n\x1a\nfixture").decode()
    attachment = inputs.prepare_attachment(config, "target.png", "application/octet-stream", data)
    part = inputs.conversation_user_input("", [attachment])["content"][-1]
    assert part["type"] == "input_image"
    assert part["image_url"] == f"data:image/png;base64,{data}"
    with pytest.raises(ValueError, match="不支持图片"):
        inputs.validate_conversation_selection(LLMConfig(), None, "default", [attachment])


def test_document_extraction_releases_the_temporary_provider_file(config, monkeypatch):
    files = SimpleNamespace(create=Mock(return_value=SimpleNamespace(id="temporary-file")),
                            content=Mock(return_value=SimpleNamespace(text="# Specification\n42")), delete=Mock())
    monkeypatch.setattr(inputs, "_client", lambda *_args, **_kwargs: nullcontext(SimpleNamespace(files=files)))
    attachment = inputs.prepare_attachment(config, "spec.pdf", "application/pdf", base64.b64encode(b"%PDF-fixture").decode())
    assert "42" in attachment["content"][0]["text"]
    assert files.create.call_args.kwargs["purpose"] == "file-extract"
    files.delete.assert_called_once_with("temporary-file")
    files.content.side_effect = RuntimeError("provider unavailable")
    with pytest.raises(RuntimeError):
        inputs.prepare_attachment(config, "spec.pdf", "application/pdf", base64.b64encode(b"%PDF-fixture").decode())
    assert files.delete.call_count == 2


@pytest.mark.parametrize("name,data", [("file.txt", "not-base64!"), ("file.exe", "eA=="), ("file.png", "eA=="), ("empty.txt", "")])
def test_invalid_attachments_are_rejected(config, name, data):
    with pytest.raises(ValueError):
        inputs.prepare_attachment(config, name, "application/octet-stream", data)


def test_other_providers_use_their_own_credentials_and_protocol_settings(config, monkeypatch):
    from decision.llm.conversation_models import conversation_config

    monkeypatch.setenv("OPENAI_API_KEY", "test-openai")
    monkeypatch.setenv("ANTHROPIC_API_KEY", "test-anthropic")
    gpt = conversation_config(config, "gpt-6.1-sol")
    assert gpt.backend == "openai" and gpt.api_key_env == "OPENAI_API_KEY"
    assert gpt.base_url == "https://api.openai.com/v1"
    inputs.validate_conversation_selection(config, gpt.model, "xhigh", [])
    assert inputs.conversation_model_settings(gpt, gpt.model, "xhigh").reasoning.effort == "xhigh"
    claude = conversation_config(config, "claude-sonnet-5-5")
    assert claude.base_url == "https://api.anthropic.com/v1/"
    assert claude.api_key_env == "ANTHROPIC_API_KEY"
    settings = inputs.conversation_model_settings(claude, claude.model, "default")
    assert settings.temperature is None and settings.reasoning is None and settings.max_tokens == 8192
    with pytest.raises(ValueError, match="不支持这个思考"):
        inputs.validate_conversation_selection(config, claude.model, "high", [])
    assert config.backend == "kimi" and config.api_key_env == "MOONSHOT_API_KEY"


def test_missing_provider_credentials_are_visible_and_rejected(config, monkeypatch):
    from decision.llm.conversation_models import provider_choices

    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    assert next(item for item in provider_choices(config) if item[1] == "gpt-6.1-sol")[3] is False
    with pytest.raises(ValueError, match="OPENAI_API_KEY"):
        inputs.validate_conversation_selection(config, "gpt-6.1-sol", "default", [])


def test_codex_subscription_is_distinct_from_api_key_and_keeps_images(config, monkeypatch):
    from decision.llm.codex import account
    from decision.llm.conversation_models import conversation_config

    monkeypatch.setattr(account, "status", lambda: {"connected": False})
    with pytest.raises(ValueError, match="登录"):
        inputs.validate_conversation_selection(config, "codex:gpt-6.1-sol", "default", [])
    monkeypatch.setattr(account, "status", lambda: {"connected": True})
    inputs.validate_conversation_selection(config, "codex:gpt-6.1-sol", "high", [{"mime_type": "image/png"}])
    chosen = conversation_config(config, "codex:gpt-6.1-sol")
    assert chosen.backend == "codex" and chosen.model == "gpt-6.1-sol"
    assert chosen.api_key_env == ""
    assert inputs.conversation_model_settings(chosen, chosen.model, "high").store is False


@pytest.mark.parametrize("connected", [False, True])
def test_unavailable_default_does_not_block_codex_login(config, monkeypatch, connected):
    from decision.llm.codex import account

    monkeypatch.setattr(account, "status", lambda: {"connected": connected})
    monkeypatch.setattr(inputs, "_client", Mock(side_effect=ValueError("default unavailable")))
    options = inputs.get_conversation_options(config)
    assert options["models"][0]["configured"] is False
    assert next(row for row in options["models"] if row["provider"] == "codex")["configured"] is connected
    assert {row["provider"] for row in options["models"]} == {"moonshot", "openai", "claude", "qwen", "codex"}
