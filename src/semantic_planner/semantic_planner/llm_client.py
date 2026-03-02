"""
Cloud LLM 客户端抽象层。

支持后端:
  - OpenAI (GPT-4o / GPT-4o-mini)
  - Moonshot Kimi (kimi-k2.5 / moonshot-v1-8k, OpenAI 兼容 API)
  - Anthropic Claude (Claude 3.5 Sonnet / Haiku)
  - Alibaba Qwen (通义千问, 通过 DashScope SDK)

所有客户端都使用异步接口, 便于在 ROS2 回调中不阻塞。
"""

import asyncio
import json
import logging
import os
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class LLMConfig:
    """LLM 配置。"""
    backend: str = "openai"
    model: str = "gpt-4o-mini"
    api_key_env: str = "OPENAI_API_KEY"
    timeout_sec: float = 10.0
    max_retries: int = 2
    temperature: float = 0.2
    base_url: str = ""  # 自定义API base URL（如Moonshot）


class LLMClientBase(ABC):
    """LLM 客户端抽象基类。"""

    def __init__(self, config: LLMConfig):
        self.config = config
        self._api_key = os.environ.get(config.api_key_env, "")
        if not self._api_key:
            logger.warning(
                "API key not found in env var '%s'. "
                "Set it before making LLM calls.",
                config.api_key_env,
            )

    @abstractmethod
    async def chat(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
    ) -> str:
        """
        发送对话请求。

        Args:
            messages: OpenAI 格式消息列表 [{"role": "system", "content": "..."}, ...]
            temperature: 覆盖默认温度

        Returns:
            助手回复文本

        Raises:
            LLMError: API 调用失败
        """
        ...

    @abstractmethod
    def is_available(self) -> bool:
        """检查 API Key 是否配置。"""
        ...


class LLMError(Exception):
    """LLM 调用异常。"""
    pass


# ================================================================
#  OpenAI 客户端
# ================================================================

class OpenAIClient(LLMClientBase):
    """OpenAI API 客户端 (GPT-4o / GPT-4o-mini)。

    支持 Vision: 当 messages 中包含 image_url content 时,
    自动使用 GPT-4o 的多模态能力 (参考 VLMnav 论文)。
    """

    def __init__(self, config: LLMConfig):
        super().__init__(config)
        self._client = None

    def _ensure_client(self):
        if self._client is None:
            try:
                from openai import AsyncOpenAI
                # 支持自定义base_url（如Moonshot）
                kwargs = {
                    "api_key": self._api_key,
                    "timeout": self.config.timeout_sec,
                }
                # 检查是否有自定义base_url
                if hasattr(self.config, 'base_url') and self.config.base_url:
                    kwargs["base_url"] = self.config.base_url
                self._client = AsyncOpenAI(**kwargs)
            except ImportError:
                raise LLMError(
                    "openai package not installed. Run: pip install openai"
                )

    async def chat(
        self,
        messages: List[Dict],
        temperature: Optional[float] = None,
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
                        attempt + 1, self.config.max_retries,
                    )
                    await asyncio.sleep(2 ** attempt)
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
                    wait = 2 ** attempt
                    logger.warning(
                        "OpenAI API attempt %d failed (%s): %s, retrying in %ds",
                        attempt + 1, type(e).__name__, e, wait,
                    )
                    await asyncio.sleep(wait)
                else:
                    raise LLMError(f"OpenAI API failed after {attempt + 1} attempts: {e}") from e

    async def chat_with_image(
        self,
        text_prompt: str,
        image_base64: str,
        system_prompt: str = "",
        temperature: Optional[float] = None,
    ) -> str:
        """
        带图像的对话 (GPT-4o Vision)。

        参考 VLMnav (2024): 发送当前相机帧给 VLM,
        让它基于视觉直接判断目标位置。

        Args:
            text_prompt: 文本提示
            image_base64: base64 编码的 JPEG 图片
            system_prompt: 系统提示
            temperature: 温度

        Returns:
            助手回复文本
        """
        self._ensure_client()
        temp = temperature if temperature is not None else self.config.temperature

        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})

        messages.append({
            "role": "user",
            "content": [
                {"type": "text", "text": text_prompt},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{image_base64}",
                        "detail": "low",  # low = 更快更便宜, high = 更精确
                    },
                },
            ],
        })

        for attempt in range(self.config.max_retries + 1):
            try:
                # Vision 调用必须用支持图像的模型
                model = self.config.model
                if "mini" in model:
                    model = model.replace("mini", "")  # 4o-mini 不支持 vision → 升级到 4o
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
                        attempt + 1, type(e).__name__, e,
                    )
                    await asyncio.sleep(2 ** attempt)
                else:
                    raise LLMError(f"OpenAI Vision failed: {e}") from e

    def is_available(self) -> bool:
        return bool(self._api_key)


# ================================================================
#  Anthropic Claude 客户端
# ================================================================

class ClaudeClient(LLMClientBase):
    """Anthropic Claude API 客户端。"""

    def __init__(self, config: LLMConfig):
        super().__init__(config)
        self._client = None

    def _ensure_client(self):
        if self._client is None:
            try:
                from anthropic import AsyncAnthropic
                self._client = AsyncAnthropic(
                    api_key=self._api_key,
                    timeout=self.config.timeout_sec,
                )
            except ImportError:
                raise LLMError(
                    "anthropic package not installed. Run: pip install anthropic"
                )

    async def chat(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
    ) -> str:
        self._ensure_client()
        temp = temperature if temperature is not None else self.config.temperature

        # Claude API 要求 system 消息单独传递
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
                # Claude 返回 content blocks
                return "".join(
                    block.text for block in response.content
                    if hasattr(block, "text")
                )
            except (KeyboardInterrupt, SystemExit):
                raise
            except LLMError:
                raise
            except Exception as e:
                if attempt < self.config.max_retries:
                    wait = 2 ** attempt
                    logger.warning(
                        "Claude API attempt %d failed (%s): %s, retrying in %ds",
                        attempt + 1, type(e).__name__, e, wait,
                    )
                    await asyncio.sleep(wait)
                else:
                    raise LLMError(f"Claude API failed after {attempt + 1} attempts: {e}") from e

    def is_available(self) -> bool:
        return bool(self._api_key)


# ================================================================
#  阿里巴巴 Qwen 客户端 (DashScope)
# ================================================================

class QwenClient(LLMClientBase):
    """阿里通义千问 API 客户端 (通过 DashScope SDK)。"""

    def __init__(self, config: LLMConfig):
        super().__init__(config)

    async def chat(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
    ) -> str:
        temp = temperature if temperature is not None else self.config.temperature

        for attempt in range(self.config.max_retries + 1):
            try:
                # DashScope SDK 是同步的, 在线程池中运行
                result = await asyncio.get_running_loop().run_in_executor(
                    None, self._sync_call, messages, temp
                )
                return result
            except (KeyboardInterrupt, SystemExit):
                raise
            except LLMError:
                raise
            except Exception as e:
                if attempt < self.config.max_retries:
                    wait = 2 ** attempt
                    logger.warning(
                        "Qwen API attempt %d failed (%s): %s, retrying in %ds",
                        attempt + 1, type(e).__name__, e, wait,
                    )
                    await asyncio.sleep(wait)
                else:
                    raise LLMError(f"Qwen API failed after {attempt + 1} attempts: {e}") from e

    def _sync_call(self, messages: List[Dict[str, str]], temperature: float) -> str:
        """同步调用 DashScope API。"""
        try:
            import dashscope
            from dashscope import Generation
        except ImportError:
            raise LLMError(
                "dashscope package not installed. Run: pip install dashscope"
            )

        dashscope.api_key = self._api_key
        response = Generation.call(
            model=self.config.model,
            messages=messages,
            temperature=temperature,
            result_format="message",
        )

        if response.status_code == 200:
            return response.output.choices[0].message.content
        else:
            raise LLMError(
                f"Qwen API error {response.status_code}: {response.message}"
            )

    def is_available(self) -> bool:
        return bool(self._api_key)


# ================================================================
#  Moonshot Kimi 客户端 (OpenAI 兼容 API)
# ================================================================

class MoonshotClient(OpenAIClient):
    """Moonshot Kimi API 客户端 (kimi-k2.5 / moonshot-v1-8k)。

    Kimi 使用 OpenAI 兼容接口, 只需设置 base_url 和 API key。
    默认 base_url: https://api.moonshot.cn/v1
    默认 model: moonshot-v1-8k (可切换 kimi-k2.5)
    """

    _DEFAULT_BASE_URL = "https://api.moonshot.cn/v1"

    def __init__(self, config: LLMConfig):
        if not config.base_url:
            config.base_url = self._DEFAULT_BASE_URL
        if config.api_key_env == "OPENAI_API_KEY":
            config.api_key_env = "MOONSHOT_API_KEY"
        if config.model in ("gpt-4o-mini", "gpt-4o"):
            config.model = "moonshot-v1-8k"
        super().__init__(config)

    async def chat_with_image(self, *args, **kwargs) -> str:
        raise LLMError("Moonshot Kimi does not support vision input yet")


# ================================================================
#  Mock 客户端 (离线测试 / API 不可用时)
# ================================================================

class MockLLMClient(LLMClientBase):
    """Mock LLM 客户端 — 无需 API Key，立即返回预设导航响应。

    用途:
      - 离线单元测试 (API 不可用 / 无余额时)
      - CI/CD 环境中的 Slow Path 功能验证

    响应策略:
      - 解析指令中的房间/物体关键词
      - 匹配场景图中的对应物体
      - 返回合法 JSON 格式的导航结果
    """

    # 房间关键词 → 常见房间物体
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
        return True  # mock 永远可用

    async def chat(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
    ) -> str:
        """返回与指令语义一致的 mock 导航 JSON。"""
        import re, json as _json

        # 拼接所有消息文本以提取关键信息
        full_text = " ".join(m.get("content", "") for m in messages).lower()

        # 1. 尝试从消息中解析场景图对象列表
        objects: List[dict] = []
        try:
            for m in messages:
                content = m.get("content", "")
                # 找到第一个包含 "objects" 的 JSON 块
                match = re.search(r'\{[\s\S]*?"objects"[\s\S]*?\}', content)
                if match:
                    sg = _json.loads(match.group(0))
                    objects = sg.get("objects", [])
                    if objects:
                        break
        except (json.JSONDecodeError, TypeError, KeyError):
            pass  # Mock client: scene graph parsing is best-effort

        # 2. 确定目标物体
        target_label = ""
        target_x, target_y, target_z = 1.0, 0.0, 0.0
        confidence = 0.72
        reasoning = "Mock LLM: 根据语义推断导航目标"

        # 按房间关键词匹配
        room_match = None
        preferred_labels: List[str] = []
        for kw, (room, labels) in self._ROOM_HINTS.items():
            if kw in full_text:
                room_match = room
                preferred_labels = labels
                reasoning = (
                    f"Mock LLM: 指令含'{kw}'关键词 → 推断目标区域为{room}，"
                    f"优先匹配{labels}"
                )
                break

        # 在场景图中查找最佳匹配物体
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
                        confidence = 0.80
                        break
                if target_label:
                    break

        if not target_label:
            # 直接在全文中找物体标签
            common_objects = [
                "chair", "table", "sofa", "bed", "door", "refrigerator",
                "dining table", "counter", "desk", "tv",
            ]
            for name in common_objects:
                if name in full_text:
                    target_label = name
                    # 从场景图找坐标
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
            reasoning = "Mock LLM: 未识别目标，建议探索"

        response = {
            "action": "navigate",
            "target": {"x": target_x, "y": target_y, "z": target_z},
            "target_label": target_label,
            "confidence": confidence,
            "reasoning": reasoning,
        }
        return _json.dumps(response, ensure_ascii=False)


# ================================================================
#  工厂函数
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


def create_llm_client(config: LLMConfig) -> LLMClientBase:
    """根据配置创建 LLM 客户端。"""
    backend = _BACKEND_ALIASES.get(config.backend.lower(), config.backend.lower())

    if backend == "openai":
        return OpenAIClient(config)
    elif backend == "claude":
        return ClaudeClient(config)
    elif backend == "qwen":
        return QwenClient(config)
    elif backend == "moonshot":
        return MoonshotClient(config)
    elif backend == "mock":
        return MockLLMClient(config)
    else:
        raise ValueError(
            f"Unknown LLM backend: '{config.backend}'. "
            f"Supported: {sorted(_BACKEND_ALIASES.keys())}"
        )
