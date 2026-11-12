"""Provider options and attachment input for persistent robot conversations."""

from __future__ import annotations

import base64
import binascii
import os
from pathlib import Path

from decision.llm.client import LLMConfig, resolve_llm_backend
from decision.llm.conversation_models import PROVIDERS, conversation_config, provider_choices

MAX_ATTACHMENT_BYTES = 10 * 1024 * 1024
MAX_ATTACHMENTS = 4
MAX_DOCUMENT_CHARACTERS = 120_000
TEXT_EXTENSIONS = {".txt", ".md", ".csv", ".json", ".yaml", ".yml", ".log", ".py", ".cpp", ".h", ".ts", ".js"}
IMAGE_TYPES = {".png": "image/png", ".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".webp": "image/webp"}
DOCUMENT_TYPES = {".pdf": "application/pdf", ".docx": "application/vnd.openxmlformats-officedocument.wordprocessingml.document", ".doc": "application/msword"}
KIMI_MODELS = {
    "kimi-k2.6": "Kimi K2.6",
    "kimi-k3": "Kimi K3",
    "kimi-k3.1-pro": "Kimi K3.1 Pro",
    "kimi-k3.1-pro-highspeed": "Kimi K3.1 Pro Highspeed",
}


def _moonshot(config: LLMConfig) -> bool:
    return resolve_llm_backend(config.backend) == "moonshot"


def _model_option(config: LLMConfig, model: str) -> dict:
    reasoning = [{"id": "default", "label": "默认"}]
    if resolve_llm_backend(config.backend) == "qwen" and model in {"qwen-plus", "qwen-flash"}:
        reasoning += [{"id": "off", "label": "快速"}, {"id": "on", "label": "深度思考"}]
    if _moonshot(config) and model == "kimi-k2.6":
        reasoning += [{"id": "off", "label": "快速"}, {"id": "on", "label": "深度思考"}]
    elif _moonshot(config) and model in KIMI_MODELS:
        reasoning += [{"id": "low", "label": "低"}, {"id": "high", "label": "高"}, {"id": "max", "label": "最高"}]
    elif config.backend == "codex":
        reasoning += [{"id": key, "label": label} for key, label in
                      [("low", "低"), ("medium", "中"), ("high", "高"), ("xhigh", "更高")]]
    elif resolve_llm_backend(config.backend) == "openai" and model in PROVIDERS["openai"][2]:
        reasoning += [{"id": key, "label": label} for key, label in
                      [("low", "低"), ("medium", "中"), ("high", "高"), ("xhigh", "更高"), ("max", "最高")]]
    extensions = TEXT_EXTENSIONS | set(IMAGE_TYPES)
    if _moonshot(config):
        extensions = extensions | set(DOCUMENT_TYPES)
    if resolve_llm_backend(config.backend) == "qwen":
        extensions = TEXT_EXTENSIONS
    return {"attachment_accept": ",".join(sorted(extensions)), "id": model, "label": KIMI_MODELS.get(model, model), "reasoning": reasoning,
            "provider": resolve_llm_backend(config.backend),
            "image_input": config.backend == "codex" or (_moonshot(config) and model in KIMI_MODELS)
                           or any(model in entry[2] for provider, entry in PROVIDERS.items() if provider != "qwen")}


def _client(config: LLMConfig, timeout: float = 20):
    from openai import OpenAI

    key = os.environ.get(config.api_key_env)
    if not key:
        raise ValueError("模型密钥尚未配置。")
    return OpenAI(api_key=key, base_url=config.base_url or None, timeout=timeout, max_retries=0)


def get_conversation_options(config: LLMConfig) -> dict:
    models = [config.model]
    default_error = ""
    if _moonshot(config):
        from openai import OpenAIError

        try:
            with _client(config) as client:
                available = {model.id for model in client.models.list().data}
            models = [model for model in KIMI_MODELS if model in available]
            if config.model not in models:
                models.insert(0, config.model)
        except (ValueError, OpenAIError):
            default_error = "默认模型连接不可用，请在模型连接中配置或登录其他提供商。"
    choices = [_model_option(config, model) for model in models]
    if default_error:
        for choice in choices:
            choice.update(configured=False, connection_error=default_error)
    for connection, model, label, configured in provider_choices(config):
        if model not in models:
            choices.append({**_model_option(connection, model), "label": label,
                            "configured": configured, "key_env": connection.api_key_env})
    extensions = set(TEXT_EXTENSIONS) | set(IMAGE_TYPES)
    if _moonshot(config):
        extensions.update(IMAGE_TYPES)
        extensions.update(DOCUMENT_TYPES)
    return {
        "available": True,
        "provider": resolve_llm_backend(config.backend),
        "default_model": config.model,
        "default_reasoning": "default",
        "models": choices,
        "attachments": {"accept": ",".join(sorted(extensions)), "max_files": MAX_ATTACHMENTS,
                        "max_file_bytes": MAX_ATTACHMENT_BYTES},
    }


def validate_conversation_selection(config: LLMConfig, model: str | None, reasoning: str,
                                    attachments: list[dict]) -> None:
    selected = model or config.model
    config = conversation_config(config, selected)
    if config.backend == "codex":
        from decision.llm.codex import account

        if not account.status()["connected"]:
            raise ValueError("请先在模型连接中登录 ChatGPT / Codex。")
    elif selected != config.model and not (_moonshot(config) and selected in KIMI_MODELS):
        raise ValueError("当前服务未配置这个模型。")
    if any(selected in entry[2] for entry in PROVIDERS.values()) and not os.environ.get(config.api_key_env):
        raise ValueError(f"请在服务端配置 {config.api_key_env} 后使用这个模型。")
    option = _model_option(config, selected)
    if reasoning not in {choice["id"] for choice in option["reasoning"]}:
        raise ValueError("所选模型不支持这个思考选项。")
    if len(attachments) > MAX_ATTACHMENTS:
        raise ValueError("每条消息最多附加 4 个文件。")
    if not option["image_input"] and any(item["mime_type"].startswith("image/") for item in attachments):
        raise ValueError("所选模型不支持图片，请更换模型或移除图片。")


def prepare_attachment(config: LLMConfig, name: str, mime_type: str, data_base64: str) -> dict:
    if not name or len(name) > 255:
        raise ValueError("附件名称为空或过长。")
    if len(data_base64) > ((MAX_ATTACHMENT_BYTES + 2) // 3) * 4:
        raise ValueError("单个附件最大 10 MB。")
    try:
        data = base64.b64decode(data_base64, validate=True)
    except (binascii.Error, ValueError) as exc:
        raise ValueError("附件编码无效。") from exc
    if not data or len(data) > MAX_ATTACHMENT_BYTES:
        raise ValueError("附件不能为空，单个附件最大 10 MB。")
    extension = Path(name).suffix.lower()
    if extension in IMAGE_TYPES:
        mime_type = IMAGE_TYPES[extension]
        signatures = {
            "image/png": data.startswith(b"\x89PNG\r\n\x1a\n"),
            "image/jpeg": data.startswith(b"\xff\xd8\xff"),
            "image/webp": data.startswith(b"RIFF") and data[8:12] == b"WEBP",
        }
        if not signatures[mime_type]:
            raise ValueError("图片格式与文件扩展名不匹配。")
        content = [{"type": "input_text", "text": f"附件图片 {name}"},
                   {"type": "input_image", "image_url": f"data:{mime_type};base64,{data_base64}", "detail": "auto"}]
    else:
        if extension in TEXT_EXTENSIONS:
            mime_type = "text/plain"
            try:
                document = data.decode("utf-8-sig")
            except UnicodeDecodeError as exc:
                raise ValueError("文本附件需要 UTF-8 编码。") from exc
        elif extension in DOCUMENT_TYPES and _moonshot(config):
            mime_type = DOCUMENT_TYPES[extension]
            with _client(config, timeout=60) as client:
                uploaded = client.files.create(file=(name, data, mime_type), purpose="file-extract")
                try:
                    document = client.files.content(uploaded.id).text
                finally:
                    client.files.delete(uploaded.id)
        else:
            raise ValueError("当前服务不支持这个附件格式。")
        if len(document) > MAX_DOCUMENT_CHARACTERS:
            raise ValueError("文档提取文本超过 12 万字符，请拆分后上传。")
        if not document.strip():
            raise ValueError("附件中没有提取到可读取的文字。")
        content = [{"type": "input_text", "text": f"附件文档 {name}\n\n{document}"}]
    return {"name": name, "mime_type": mime_type, "size": len(data), "data": data, "content": content}


def conversation_model_settings(config: LLMConfig, model: str, reasoning: str):
    from agents import ModelSettings

    if config.backend == "codex":
        from openai.types.shared import Reasoning

        return ModelSettings(store=False, parallel_tool_calls=False,
                             reasoning=Reasoning(effort="medium" if reasoning == "default" else reasoning))

    if resolve_llm_backend(config.backend) == "qwen" and model in {"qwen-plus", "qwen-flash"}:
        return ModelSettings(parallel_tool_calls=False, extra_body={"enable_thinking": reasoning == "on"})

    if resolve_llm_backend(config.backend) == "claude":
        return ModelSettings(parallel_tool_calls=False, max_tokens=8192)
    if resolve_llm_backend(config.backend) == "openai" and model in PROVIDERS["openai"][2]:
        from openai.types.shared import Reasoning
        return ModelSettings(parallel_tool_calls=False,
                             reasoning=Reasoning(effort="medium" if reasoning == "default" else reasoning))
    if _moonshot(config) and model in KIMI_MODELS:
        if model == "kimi-k2.6":
            thinking = {"type": "disabled"} if reasoning == "off" else {"type": "enabled", "keep": "all"}
            extra = {"thinking": thinking}
        else:
            extra = {"reasoning_effort": "max" if reasoning == "default" else reasoning}
        return ModelSettings(parallel_tool_calls=False, include_usage=True, extra_body=extra)
    return ModelSettings(temperature=config.temperature, parallel_tool_calls=False)


def conversation_user_input(text: str, attachments: list[dict] | None = None) -> dict:
    if not attachments:
        return {"role": "user", "content": text}
    content = [{"type": "input_text", "text": text}] if text else []
    for attachment in attachments:
        content.extend(attachment["content"])
    return {"role": "user", "content": content}
