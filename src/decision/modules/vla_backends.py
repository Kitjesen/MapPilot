"""VLA inference backend abstraction layer.

Provides pluggable backends for vision-language-action navigation calls.
All backends are async and return a normalized action dict.
"""

from __future__ import annotations

import json
import logging
import os
from abc import ABC, abstractmethod

try:
    import httpx
except ImportError:  # pragma: no cover
    httpx = None  # type: ignore

logger = logging.getLogger(__name__)


def _require_httpx() -> None:
    """Raise a clear error if httpx is not installed."""
    if httpx is None:
        raise ImportError(
            "VLA HTTP backends require 'httpx'. Install it with 'pip install httpx'."
        )


class VLABackend(ABC):
    """VLA inference backend abstraction."""

    @abstractmethod
    async def infer(self, image_b64: str, instruction: str, context: dict) -> dict:
        """Run VLA inference and return a normalized action dict.

        Args:
            image_b64: Base64-encoded JPEG image string (no data URI prefix).
            instruction: Natural language instruction.
            context: Extra context such as robot pose, scene graph summary.

        Returns:
            Normalized action dict with keys:
            - action: str (navigate/approach/explore/stop)
            - target_x: float
            - target_y: float
            - target_z: float
            - confidence: float
            - reason: str
        """
        ...

    @staticmethod
    def _normalize_action(raw: dict) -> dict:
        """Normalize an action dict to the canonical schema."""
        action = str(raw.get("action", "stop")).lower().strip()
        if action not in {"navigate", "approach", "explore", "stop"}:
            action = "stop"
        return {
            "action": action,
            "target_x": float(raw.get("target_x", 0.0)),
            "target_y": float(raw.get("target_y", 0.0)),
            "target_z": float(raw.get("target_z", 0.0)),
            "confidence": float(raw.get("confidence", 0.0)),
            "reason": str(raw.get("reason", "")),
        }

    @staticmethod
    def _extract_json(text: str) -> dict:
        """Extract the first JSON object from a model response string."""
        text = text.strip()
        if text.startswith("```"):
            # Strip markdown code fences
            lines = text.splitlines()
            if lines[0].startswith("```"):
                lines = lines[1:]
            if lines and lines[-1].startswith("```"):
                lines = lines[:-1]
            text = "\n".join(lines).strip()
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass
        # Try to find a JSON object substring
        start = text.find("{")
        end = text.rfind("}")
        if start != -1 and end != -1 and end > start:
            try:
                return json.loads(text[start : end + 1])
            except json.JSONDecodeError as exc:
                raise ValueError(f"No valid JSON found in response: {text[:200]}") from exc
        raise ValueError(f"No valid JSON found in response: {text[:200]}")


class OpenAIVLABackend(VLABackend):
    """OpenAI GPT-4o Vision backend for VLA."""

    def __init__(
        self,
        api_key: str = "",
        model: str = "gpt-4o",
        timeout_sec: float = 5.0,
        base_url: str = "https://api.openai.com/v1",
    ):
        _require_httpx()
        self._api_key = api_key or os.environ.get("OPENAI_API_KEY", "")
        self._model = model
        self._timeout_sec = timeout_sec
        self._base_url = base_url.rstrip("/")
        self._client: httpx.AsyncClient | None = None

    async def infer(self, image_b64: str, instruction: str, context: dict) -> dict:
        """Run GPT-4o vision inference and return a normalized action."""
        _require_httpx()
        x = float(context.get("x", 0.0))
        y = float(context.get("y", 0.0))
        system_prompt = context.get(
            "system_prompt",
            (
                "You are a robot navigation assistant. Given an image and a command, "
                "output a single JSON object with keys: action, target_x, target_y, "
                "target_z, confidence, reason. action must be one of "
                "navigate|approach|explore|stop. Coordinates are in the map frame in metres."
            ),
        )
        messages = [
            {"role": "system", "content": system_prompt},
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": (f"Robot at ({x:.1f},{y:.1f}). Instruction: {instruction}"),
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"},
                    },
                ],
            },
        ]
        if self._client is None:
            self._client = httpx.AsyncClient(timeout=self._timeout_sec)
        resp = await self._client.post(
            f"{self._base_url}/chat/completions",
            json={
                "model": self._model,
                "messages": messages,
                "max_tokens": 256,
                "temperature": 0.2,
            },
            headers={
                "Authorization": f"Bearer {self._api_key}",
                "Content-Type": "application/json",
            },
        )
        resp.raise_for_status()
        payload = resp.json()
        text = payload["choices"][0]["message"]["content"]
        return self._normalize_action(self._extract_json(text))


class ClaudeVLABackend(VLABackend):
    """Anthropic Claude 3.5 Sonnet backend for VLA."""

    def __init__(
        self,
        api_key: str = "",
        model: str = "claude-3-5-sonnet-20241022",
        timeout_sec: float = 5.0,
        base_url: str = "https://api.anthropic.com",
    ):
        _require_httpx()
        self._api_key = api_key or os.environ.get("ANTHROPIC_API_KEY", "")
        self._model = model
        self._timeout_sec = timeout_sec
        self._base_url = base_url.rstrip("/")
        self._client: httpx.AsyncClient | None = None

    async def infer(self, image_b64: str, instruction: str, context: dict) -> dict:
        """Run Claude 3.5 Sonnet vision inference and return a normalized action."""
        _require_httpx()
        x = float(context.get("x", 0.0))
        y = float(context.get("y", 0.0))
        system_prompt = context.get(
            "system_prompt",
            (
                "You are a robot navigation assistant. Given an image and a command, "
                "output a single JSON object with keys: action, target_x, target_y, "
                "target_z, confidence, reason. action must be one of "
                "navigate|approach|explore|stop. Coordinates are in the map frame in metres."
            ),
        )
        if self._client is None:
            self._client = httpx.AsyncClient(timeout=self._timeout_sec)
        resp = await self._client.post(
            f"{self._base_url}/v1/messages",
            json={
                "model": self._model,
                "max_tokens": 256,
                "temperature": 0.2,
                "system": system_prompt,
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": (f"Robot at ({x:.1f},{y:.1f}). Instruction: {instruction}"),
                            },
                            {
                                "type": "image",
                                "source": {
                                    "type": "base64",
                                    "media_type": "image/jpeg",
                                    "data": image_b64,
                                },
                            },
                        ],
                    },
                ],
            },
            headers={
                "x-api-key": self._api_key,
                "anthropic-version": "2023-06-01",
                "Content-Type": "application/json",
            },
        )
        resp.raise_for_status()
        payload = resp.json()
        text = ""
        for block in payload.get("content", []):
            if block.get("type") == "text":
                text += block.get("text", "")
        return self._normalize_action(self._extract_json(text))


class QwenVLABackend(VLABackend):
    """DashScope / Qwen-VL backend for VLA (OpenAI-compatible endpoint)."""

    def __init__(
        self,
        api_key: str = "",
        model: str = "qwen-vl-max",
        timeout_sec: float = 5.0,
        base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
    ):
        _require_httpx()
        self._api_key = api_key or os.environ.get("DASHSCOPE_API_KEY", "")
        self._model = model
        self._timeout_sec = timeout_sec
        self._base_url = base_url.rstrip("/")
        self._client: httpx.AsyncClient | None = None

    async def infer(self, image_b64: str, instruction: str, context: dict) -> dict:
        """Run Qwen-VL inference and return a normalized action."""
        _require_httpx()
        x = float(context.get("x", 0.0))
        y = float(context.get("y", 0.0))
        system_prompt = context.get(
            "system_prompt",
            (
                "You are a robot navigation assistant. Given an image and a command, "
                "output a single JSON object with keys: action, target_x, target_y, "
                "target_z, confidence, reason. action must be one of "
                "navigate|approach|explore|stop. Coordinates are in the map frame in metres."
            ),
        )
        messages = [
            {"role": "system", "content": system_prompt},
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": (f"Robot at ({x:.1f},{y:.1f}). Instruction: {instruction}"),
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"},
                    },
                ],
            },
        ]
        if self._client is None:
            self._client = httpx.AsyncClient(timeout=self._timeout_sec)
        resp = await self._client.post(
            f"{self._base_url}/chat/completions",
            json={
                "model": self._model,
                "messages": messages,
                "max_tokens": 256,
                "temperature": 0.2,
            },
            headers={
                "Authorization": f"Bearer {self._api_key}",
                "Content-Type": "application/json",
            },
        )
        resp.raise_for_status()
        payload = resp.json()
        text = payload["choices"][0]["message"]["content"]
        return self._normalize_action(self._extract_json(text))


class MockVLABackend(VLABackend):
    """Mock backend for offline testing.

    Returns a deterministic action based on the instruction text.
    """

    def __init__(self, *, fixed_action: dict | None = None, call_count: int = 0):
        self._fixed_action = fixed_action
        self._call_count = call_count

    async def infer(self, image_b64: str, instruction: str, context: dict) -> dict:
        """Return a deterministic mock action for offline testing."""
        self._call_count += 1
        if self._fixed_action is not None:
            return self._normalize_action(self._fixed_action)
        instruction_lower = instruction.lower()
        if "stop" in instruction_lower:
            action = "stop"
            target_x = target_y = 0.0
            confidence = 1.0
            reason = "Mock stop requested"
        elif "explore" in instruction_lower or "look around" in instruction_lower:
            action = "explore"
            target_x = float(context.get("x", 0.0)) + 1.0
            target_y = float(context.get("y", 0.0)) + 0.5
            confidence = 0.7
            reason = "Mock exploration target"
        elif "approach" in instruction_lower or "near" in instruction_lower:
            action = "approach"
            target_x = float(context.get("x", 0.0)) + 1.0
            target_y = float(context.get("y", 0.0)) + 0.5
            confidence = 0.8
            reason = "Mock approach target"
        else:
            action = "navigate"
            # Default target 5m ahead in x for far-distance testing
            target_x = float(context.get("x", 0.0)) + 5.0
            target_y = float(context.get("y", 0.0))
            confidence = 0.9
            reason = "Mock navigation target"
        return self._normalize_action(
            {
                "action": action,
                "target_x": target_x,
                "target_y": target_y,
                "target_z": 0.0,
                "confidence": confidence,
                "reason": reason,
            }
        )


def create_vla_backend(
    backend: str,
    *,
    api_key: str = "",
    model: str = "",
    timeout_sec: float = 5.0,
    base_url: str = "",
) -> VLABackend:
    """Factory for VLA backends."""
    backend = backend.lower().strip()
    if backend == "openai":
        return OpenAIVLABackend(
            api_key=api_key,
            model=model or "gpt-4o",
            timeout_sec=timeout_sec,
            base_url=base_url or "https://api.openai.com/v1",
        )
    if backend == "claude":
        return ClaudeVLABackend(
            api_key=api_key,
            model=model or "claude-3-5-sonnet-20241022",
            timeout_sec=timeout_sec,
            base_url=base_url or "https://api.anthropic.com",
        )
    if backend == "qwen":
        return QwenVLABackend(
            api_key=api_key,
            model=model or "qwen-vl-max",
            timeout_sec=timeout_sec,
            base_url=base_url or "https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
    if backend == "mock":
        return MockVLABackend()
    raise ValueError(f"Unknown VLA backend '{backend}'")
