"""Decision module."""

import asyncio
import json
import logging
import os
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass

from runtime.registry import get, list_plugins, register

logger = logging.getLogger(__name__)


@dataclass
class LLMConfig:
    """L L M Config."""

    backend: str = "openai"
    model: str = "gpt-4o-mini"
    api_key_env: str = "OPENAI_API_KEY"
    timeout_sec: float = 10.0
    max_retries: int = 2
    temperature: float = 0.2
    base_url: str = ""


class LLMClientBase(ABC):
    """L L M Client Base."""

    def __init__(self, config: LLMConfig):
        self.config = config
        self._api_key = os.environ.get(config.api_key_env, "")
        if not self._api_key:
            logger.warning(
                "API key not found in env var '%s'. Set it before making LLM calls.",
                config.api_key_env,
            )
        elif len(self._api_key) < 10:
            logger.warning(
                "%s appears invalid (too short: %d chars), %s backend may fail",
                config.api_key_env,
                len(self._api_key),
                config.backend,
            )

    @abstractmethod
    async def chat(
        self,
        messages: list[dict[str, str]],
        temperature: float | None = None,
    ) -> str:
        """Chat."""
        ...

    @abstractmethod
    def is_available(self) -> bool:
        """Is available."""
        ...

    async def close(self):
        """Close underlying HTTP resources. Override in subclasses that hold clients."""
        pass


class LLMError(Exception):
    """L L M Error."""

    pass


# ================================================================

# ================================================================


class OpenAIClient(LLMClientBase):
    """Open A I Client."""

    def __init__(self, config: LLMConfig):
        super().__init__(config)
        self._client = None

    def _ensure_client(self):
        if self._client is None:
            try:
                from openai import AsyncOpenAI

                kwargs = {
                    "api_key": self._api_key,
                    "timeout": self.config.timeout_sec,
                }

                if hasattr(self.config, "base_url") and self.config.base_url:
                    kwargs["base_url"] = self.config.base_url
                self._client = AsyncOpenAI(**kwargs)
            except ImportError:
                raise LLMError("openai package not installed. Run: pip install openai") from None

    async def chat(
        self,
        messages: list[dict],
        temperature: float | None = None,
    ) -> str:
        self._ensure_client()
        temp = temperature if temperature is not None else self.config.temperature

        for attempt in range(self.config.max_retries + 1):
            try:
                kwargs = dict(
                    model=self.config.model,
                    messages=messages,
                    temperature=temp,
                    max_tokens=4096,
                    stream=True,
                )
                stream = await self._client.chat.completions.create(**kwargs)
                chunks = []
                async for chunk in stream:
                    delta = chunk.choices[0].delta if chunk.choices else None
                    if delta and delta.content:
                        chunks.append(delta.content)
                content = "".join(chunks)
                if not content.strip() and attempt < self.config.max_retries:
                    logger.warning(
                        "OpenAI API returned empty response, retrying (%d/%d)",
                        attempt + 1,
                        self.config.max_retries,
                    )
                    await asyncio.sleep(2**attempt + random.uniform(0, 0.5 * 2**attempt))
                    continue
                return content
            except (KeyboardInterrupt, SystemExit):
                raise
            except LLMError:
                raise
            except Exception as e:
                err_str = str(e)
                if "invalid temperature" in err_str and temp != 1.0:
                    logger.info("Retrying with temperature=1.0 (model constraint)")
                    temp = 1.0
                    continue
                if attempt < self.config.max_retries:
                    err_lower = err_str.lower()
                    status_code = getattr(e, "status_code", None) or getattr(e, "code", None)
                    if status_code == 429 or (status_code is None and ("429" in err_str or "rate" in err_lower)):
                        wait = min(2 ** (attempt + 1), 30.0)
                        logger.warning(
                            "Rate limited (429), waiting %.1fs before retry %d/%d",
                            wait,
                            attempt + 1,
                            self.config.max_retries,
                        )
                        await asyncio.sleep(wait + random.uniform(0, 0.5 * wait))
                    elif status_code in (500, 502, 503) or (
                        status_code is None and any(s in err_str for s in ("500", "502", "503"))
                    ):
                        wait = 2**attempt
                        logger.warning(
                            "Server error on attempt %d: %s, retrying in %ds",
                            attempt + 1,
                            e,
                            wait,
                        )
                        await asyncio.sleep(wait + random.uniform(0, 0.5 * wait))
                    elif "timeout" in err_lower or "timed out" in err_lower:
                        logger.warning(
                            "OpenAI timeout on attempt %d, retrying immediately",
                            attempt + 1,
                        )
                    else:
                        wait = 2**attempt
                        logger.warning(
                            "OpenAI API attempt %d failed (%s): %s, retrying in %ds",
                            attempt + 1,
                            type(e).__name__,
                            e,
                            wait,
                        )
                        await asyncio.sleep(wait + random.uniform(0, 0.5 * wait))
                else:
                    raise LLMError(f"OpenAI API failed after {attempt + 1} attempts: {e}") from e

        # Should not reach here, but guard against falling through with empty content
        return ""

    async def chat_with_image(
        self,
        text_prompt: str,
        image_base64: str,
        system_prompt: str = "",
        temperature: float | None = None,
    ) -> str:
        """Chat with image."""
        self._ensure_client()
        temp = temperature if temperature is not None else self.config.temperature

        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})

        messages.append(
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": text_prompt},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{image_base64}",
                            "detail": "low",
                        },
                    },
                ],
            }
        )

        for attempt in range(self.config.max_retries + 1):
            try:
                model = self.config.model
                if "mini" in model:
                    model = model.replace("mini", "")
                    if not model.endswith("o"):
                        model = "gpt-4o"

                response = await self._client.chat.completions.create(
                    model=model,
                    messages=messages,
                    temperature=temp,
                    max_tokens=4096,
                )
                msg = response.choices[0].message
                return msg.content or getattr(msg, "reasoning_content", None) or getattr(msg, "reasoning", None) or ""
            except (KeyboardInterrupt, SystemExit):
                raise
            except LLMError:
                raise
            except Exception as e:
                err_str = str(e)
                if "invalid temperature" in err_str and temp != 1.0:
                    temp = 1.0
                    continue
                if attempt < self.config.max_retries:
                    logger.warning(
                        "OpenAI Vision attempt %d failed (%s): %s",
                        attempt + 1,
                        type(e).__name__,
                        e,
                    )
                    await asyncio.sleep(2**attempt + random.uniform(0, 0.5 * 2**attempt))
                else:
                    raise LLMError(f"OpenAI Vision failed: {e}") from e

    async def close(self):
        """Close underlying HTTP client to release connections."""
        if self._client is not None:
            try:
                await self._client.close()
            except (AttributeError, TypeError):
                pass
            self._client = None

    def is_available(self) -> bool:
        return bool(self._api_key)


# ================================================================

# ================================================================


class ClaudeClient(LLMClientBase):
    """Claude Client."""

    def __init__(self, config: LLMConfig):
        super().__init__(config)
        self._client = None

    def _ensure_client(self):
        if self._client is None:
            try:
                from anthropic import AsyncAnthropic

                kwargs = dict(
                    api_key=self._api_key,
                    timeout=self.config.timeout_sec,
                )
                if self.config.base_url:
                    kwargs["base_url"] = self.config.base_url
                self._client = AsyncAnthropic(**kwargs)
            except ImportError:
                raise LLMError("anthropic package not installed. Run: pip install anthropic") from None

    async def chat(
        self,
        messages: list[dict[str, str]],
        temperature: float | None = None,
    ) -> str:
        self._ensure_client()
        temp = temperature if temperature is not None else self.config.temperature

        system_msg = ""
        chat_messages = []
        for m in messages:
            if m["role"] == "system":
                system_msg = m["content"]
            else:
                chat_messages.append(m)

        for attempt in range(self.config.max_retries + 1):
            try:
                response = await self._client.messages.create(
                    model=self.config.model,
                    max_tokens=4096,
                    system=system_msg,
                    messages=chat_messages,
                    temperature=temp,
                )

                return "".join(block.text for block in response.content if hasattr(block, "text"))
            except (KeyboardInterrupt, SystemExit):
                raise
            except LLMError:
                raise
            except Exception as e:
                if attempt < self.config.max_retries:
                    err_str = str(e)
                    err_lower = err_str.lower()
                    status_code = getattr(e, "status_code", None) or getattr(e, "code", None)
                    if status_code == 429 or (status_code is None and ("429" in err_str or "rate" in err_lower)):
                        wait = min(2 ** (attempt + 1), 30.0)
                        logger.warning(
                            "Claude rate limited (429), waiting %.1fs before retry %d/%d",
                            wait,
                            attempt + 1,
                            self.config.max_retries,
                        )
                        await asyncio.sleep(wait + random.uniform(0, 0.5 * wait))
                    elif status_code in (500, 502, 503) or (
                        status_code is None and any(s in err_str for s in ("500", "502", "503"))
                    ):
                        wait = 2**attempt
                        logger.warning(
                            "Claude server error on attempt %d: %s, retrying in %ds",
                            attempt + 1,
                            e,
                            wait,
                        )
                        await asyncio.sleep(wait + random.uniform(0, 0.5 * wait))
                    elif "timeout" in err_lower or "timed out" in err_lower:
                        logger.warning(
                            "Claude timeout on attempt %d, retrying immediately",
                            attempt + 1,
                        )
                    else:
                        wait = 2**attempt
                        logger.warning(
                            "Claude API attempt %d failed (%s): %s, retrying in %ds",
                            attempt + 1,
                            type(e).__name__,
                            e,
                            wait,
                        )
                        await asyncio.sleep(wait + random.uniform(0, 0.5 * wait))
                else:
                    raise LLMError(f"Claude API failed after {attempt + 1} attempts: {e}") from e

        # Should not reach here, but guard against falling through
        return ""

    async def close(self):
        """Close underlying HTTP client to release connections."""
        if self._client is not None:
            try:
                await self._client.close()
            except (AttributeError, TypeError):
                pass
            self._client = None

    def is_available(self) -> bool:
        return bool(self._api_key)


# ================================================================

# ================================================================


class QwenClient(LLMClientBase):
    """Qwen Client."""

    def __init__(self, config: LLMConfig):
        super().__init__(config)

    async def chat(
        self,
        messages: list[dict[str, str]],
        temperature: float | None = None,
    ) -> str:
        temp = temperature if temperature is not None else self.config.temperature

        for attempt in range(self.config.max_retries + 1):
            try:
                result = await asyncio.wait_for(
                    asyncio.get_running_loop().run_in_executor(None, self._sync_call, messages, temp),
                    timeout=self.config.timeout_sec,
                )
                return result
            except asyncio.TimeoutError:
                if attempt < self.config.max_retries:
                    logger.warning(
                        "Qwen timeout on attempt %d/%d, retrying...",
                        attempt + 1,
                        self.config.max_retries,
                    )
                    await asyncio.sleep(1 + random.uniform(0, 0.5))
                    continue
                raise LLMError(f"Qwen API call timed out after {self.config.timeout_sec}s") from None
            except (KeyboardInterrupt, SystemExit):
                raise
            except LLMError:
                raise
            except Exception as e:
                if attempt < self.config.max_retries:
                    err_str = str(e)
                    err_lower = err_str.lower()
                    status_code = getattr(e, "status_code", None) or getattr(e, "code", None)
                    if status_code == 429 or (status_code is None and ("429" in err_str or "rate" in err_lower)):
                        wait = min(2 ** (attempt + 1), 30.0)
                        logger.warning(
                            "Qwen rate limited (429), waiting %.1fs before retry %d/%d",
                            wait,
                            attempt + 1,
                            self.config.max_retries,
                        )
                        await asyncio.sleep(wait + random.uniform(0, 0.5 * wait))
                    elif status_code in (500, 502, 503) or (
                        status_code is None and any(s in err_str for s in ("500", "502", "503"))
                    ):
                        wait = 2**attempt
                        logger.warning(
                            "Qwen server error on attempt %d: %s, retrying in %ds",
                            attempt + 1,
                            e,
                            wait,
                        )
                        await asyncio.sleep(wait + random.uniform(0, 0.5 * wait))
                    elif "timeout" in err_lower or "timed out" in err_lower:
                        logger.warning(
                            "Qwen timeout on attempt %d, retrying immediately",
                            attempt + 1,
                        )
                    else:
                        wait = 2**attempt
                        logger.warning(
                            "Qwen API attempt %d failed (%s): %s, retrying in %ds",
                            attempt + 1,
                            type(e).__name__,
                            e,
                            wait,
                        )
                        await asyncio.sleep(wait + random.uniform(0, 0.5 * wait))
                else:
                    raise LLMError(f"Qwen API failed after {attempt + 1} attempts: {e}") from e

        # Should not reach here, but guard against falling through
        return ""

    def _sync_call(self, messages: list[dict[str, str]], temperature: float) -> str:
        """Sync call."""
        try:
            import dashscope
            from dashscope import Generation
        except ImportError:
            raise LLMError("dashscope package not installed. Run: pip install dashscope") from None

        response = Generation.call(
            model=self.config.model,
            messages=messages,
            temperature=temperature,
            result_format="message",
            timeout=self.config.timeout_sec,
            api_key=self._api_key,
        )

        if response.status_code == 200:
            return response.output.choices[0].message.content
        else:
            raise LLMError(f"Qwen API error {response.status_code}: {response.message}")

    def is_available(self) -> bool:
        return bool(self._api_key)


# ================================================================

# ================================================================


class MoonshotClient(OpenAIClient):
    """Moonshot Client."""

    _DEFAULT_BASE_URL = "https://api.kimi.com/coding/v1"

    def __init__(self, config: LLMConfig):
        import copy

        config = copy.copy(config)
        if not config.base_url:
            config.base_url = self._DEFAULT_BASE_URL
        if config.api_key_env == "OPENAI_API_KEY":
            config.api_key_env = "MOONSHOT_API_KEY"
        if config.model in ("gpt-4o-mini", "gpt-4o"):
            config.model = "kimi-k2.5"
        super().__init__(config)

    def _ensure_client(self):
        if self._client is None:
            try:
                from openai import AsyncOpenAI

                kwargs = {
                    "api_key": self._api_key,
                    "timeout": self.config.timeout_sec,
                    "base_url": self.config.base_url or self._DEFAULT_BASE_URL,
                    "default_headers": {"User-Agent": "claude-code/1.0"},
                }
                self._client = AsyncOpenAI(**kwargs)
            except ImportError:
                raise LLMError("openai package not installed. Run: pip install openai") from None

    async def chat_with_image(self, *args, **kwargs) -> str:
        raise LLMError("Moonshot Kimi does not support vision input yet")


# ================================================================

# ================================================================


class MockLLMClient(LLMClientBase):
    """Mock L L M Client."""

    _ROOM_HINTS: dict = {
        "eat": ("kitchen", ["dining table", "refrigerator", "table"]),
        "dining": ("kitchen", ["dining table", "refrigerator", "table"]),
        "kitchen": ("kitchen", ["dining table", "refrigerator", "counter"]),
        "cook": ("kitchen", ["stove", "refrigerator", "counter"]),
        "sleep": ("bedroom", ["bed", "pillow", "wardrobe"]),
        "bed": ("bedroom", ["bed"]),
        "bedroom": ("bedroom", ["bed", "wardrobe"]),
        "sit": ("living_room", ["sofa", "chair"]),
        "relax": ("living_room", ["sofa", "tv"]),
        "living": ("living_room", ["sofa", "tv", "chair"]),
        "bathroom": ("bathroom", ["toilet", "sink", "bathtub"]),
        "wash": ("bathroom", ["sink", "toilet"]),
        "work": ("study", ["desk", "chair", "bookshelf"]),
        "study": ("study", ["desk", "bookshelf"]),
    }

    def __init__(self, config: LLMConfig):
        super().__init__(config)

    def is_available(self) -> bool:
        return True

    async def chat(
        self,
        messages: list[dict[str, str]],
        temperature: float | None = None,
    ) -> str:
        """Chat."""
        import json as _json
        import re

        full_text = " ".join(m.get("content", "") for m in messages).lower()

        objects: list[dict] = []
        try:
            for m in messages:
                content = m.get("content", "")

                match = re.search(r'\{[\s\S]*?"objects"[\s\S]*?\}', content)
                if match:
                    sg = _json.loads(match.group(0))
                    objects = sg.get("objects", [])
                    if objects:
                        break
        except (json.JSONDecodeError, TypeError, KeyError, ValueError):
            pass  # Mock client: scene graph parsing is best-effort

        target_label = ""
        target_x, target_y, target_z = 1.0, 0.0, 0.0
        confidence = 0.0
        reasoning = "MockLLM: rule-based keyword match (not real inference)"

        preferred_labels: list[str] = []
        for kw, (room, labels) in self._ROOM_HINTS.items():
            if kw in full_text:
                preferred_labels = labels
                reasoning = f"Mock LLM: keyword '{kw}' suggests room={room}; preferred labels={labels}"
                break

        if objects and preferred_labels:
            for pref in preferred_labels:
                for obj in objects:
                    if pref in obj.get("label", "").lower():
                        pos = obj.get("position", {})
                        if isinstance(pos, dict):
                            target_x = float(pos.get("x", 1.0))
                            target_y = float(pos.get("y", 0.0))
                            target_z = float(pos.get("z", 0.0))
                        elif isinstance(pos, (list, tuple)) and len(pos) >= 2:
                            target_x, target_y = float(pos[0]), float(pos[1])
                            target_z = float(pos[2]) if len(pos) > 2 else 0.0
                        target_label = obj.get("label", pref)

                        break
                if target_label:
                    break

        if not target_label:
            common_objects = [
                "chair",
                "table",
                "sofa",
                "bed",
                "door",
                "refrigerator",
                "dining table",
                "counter",
                "desk",
                "tv",
            ]
            for name in common_objects:
                if name in full_text:
                    target_label = name

                    for obj in objects:
                        if name in obj.get("label", "").lower():
                            pos = obj.get("position", {})
                            if isinstance(pos, dict):
                                target_x = float(pos.get("x", 1.0))
                                target_y = float(pos.get("y", 0.0))
                                target_z = float(pos.get("z", 0.0))
                            elif isinstance(pos, (list, tuple)) and len(pos) >= 2:
                                target_x, target_y = float(pos[0]), float(pos[1])
                                target_z = float(pos[2]) if len(pos) > 2 else 0.0
                            break
                    break

        if not target_label:
            target_label = "unknown"
            reasoning = "Mock LLM: no known target matched; suggesting exploration"

        response = {
            "action": "navigate",
            "target": {"x": target_x, "y": target_y, "z": target_z},
            "target_label": target_label,
            "confidence": confidence,
            "reasoning": reasoning,
            "mock": True,
        }
        return _json.dumps(response, ensure_ascii=False)


# ================================================================

# ================================================================

_BACKEND_ALIASES = {
    "openai": "openai",
    "gpt": "openai",
    "claude": "claude",
    "anthropic": "claude",
    "qwen": "qwen",
    "dashscope": "qwen",
    "moonshot": "moonshot",
    "kimi": "moonshot",
    "mock": "mock",
    "offline": "mock",
    "test": "mock",
}


@register("llm_client", "openai", description="OpenAI-compatible chat client")
class _OpenAIClientProvider:
    @staticmethod
    def create(config: LLMConfig) -> LLMClientBase:
        return OpenAIClient(config)


@register("llm_client", "claude", description="Anthropic Claude chat client")
class _ClaudeClientProvider:
    @staticmethod
    def create(config: LLMConfig) -> LLMClientBase:
        return ClaudeClient(config)


@register("llm_client", "qwen", description="Alibaba Qwen/DashScope chat client")
class _QwenClientProvider:
    @staticmethod
    def create(config: LLMConfig) -> LLMClientBase:
        return QwenClient(config)


@register("llm_client", "moonshot", description="Moonshot/Kimi OpenAI-compatible chat client")
class _MoonshotClientProvider:
    @staticmethod
    def create(config: LLMConfig) -> LLMClientBase:
        return MoonshotClient(config)


@register("llm_client", "mock", description="Offline deterministic mock LLM client")
class _MockLLMClientProvider:
    @staticmethod
    def create(config: LLMConfig) -> LLMClientBase:
        return MockLLMClient(config)


def resolve_llm_backend(name: str) -> str:
    """Resolve aliases such as kimi/offline to canonical plugin names."""
    return _BACKEND_ALIASES.get(name.lower(), name.lower())


def available_llm_backends() -> list[str]:
    """Return canonical plugin names plus public aliases accepted by config."""
    return sorted(set(_BACKEND_ALIASES) | set(list_plugins("llm_client")))


def create_llm_client(config: LLMConfig) -> LLMClientBase:
    """Create llm client."""
    backend = resolve_llm_backend(config.backend)
    try:
        provider = get("llm_client", backend)
        return provider.create(config)
    except KeyError:
        raise ValueError(f"Unknown LLM backend: '{config.backend}'. Supported: {available_llm_backends()}") from None
