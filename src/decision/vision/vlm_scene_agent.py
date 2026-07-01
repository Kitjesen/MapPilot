"""VLMSceneAgent — VLM (vision-language model) scene understanding utility.

NOT a Module. A plain utility class used by AgentLoop and SemanticPlannerModule
to add visual reasoning to the agent's observe→think→act cycle.

Provides four methods:
  describe_scene(image, context)        — describe what the camera sees
  find_object_in_scene(image, target)   — locate a specific object in the frame
  assess_situation(image, goal, sg)     — assess whether the view helps reach a goal
  compare_scenes(image1, image2)        — describe the difference between two frames

All methods are async and return immediately with a fallback string when:
  - No LLM client is available
  - The LLM client does not support vision (e.g. MoonshotClient)
  - The image is None or empty
  - The VLM call fails for any reason

Image encoding:
  - Input is np.ndarray (H, W, 3) BGR or RGB uint8 (OpenCV convention is BGR)
  - Resized to max 512px on the longest side before encoding (token budget)
  - Encoded as JPEG base64 string passed in OpenAI-compatible image_url content

LLM backend support:
  - OpenAI / GPT-4o:    uses chat_with_image() which exists on OpenAIClient
  - Claude:             uses generic chat() with vision message format
  - Kimi/Moonshot:      vision not supported — falls back gracefully
  - Mock:               returns canned responses for offline testing
  - Qwen:               falls back to text-only mode (no vision SDK in use)
"""

from __future__ import annotations

import asyncio
import base64
import io
import json
import logging
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

# Max image dimension before encoding — keeps token cost low on edge hardware.
_MAX_IMAGE_DIM = 512


def _encode_image(image: np.ndarray) -> str | None:
    """Resize image to max _MAX_IMAGE_DIM, encode as JPEG base64.

    Args:
        image: np.ndarray (H, W, 3) or (H, W) uint8. BGR or RGB — both work
               for JPEG encoding since we only care about compression, not color.

    Returns:
        base64 string, or None if encoding fails.
    """
    if image is None or image.size == 0:
        return None
    try:
        import cv2  # OpenCV is always available on the robot
        h, w = image.shape[:2]
        scale = min(_MAX_IMAGE_DIM / max(h, w, 1), 1.0)
        if scale < 1.0:
            new_w = max(1, int(w * scale))
            new_h = max(1, int(h * scale))
            image = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return None
        return base64.b64encode(buf.tobytes()).decode("ascii")
    except Exception as e:
        logger.debug("VLMSceneAgent: image encode failed: %s", e)
        return None


def _encode_image_pil(image: np.ndarray) -> str | None:
    """Fallback encoder using PIL when OpenCV is unavailable."""
    if image is None or image.size == 0:
        return None
    try:
        from PIL import Image as PilImage
        h, w = image.shape[:2]
        scale = min(_MAX_IMAGE_DIM / max(h, w, 1), 1.0)
        pil = PilImage.fromarray(image)
        if scale < 1.0:
            new_w = max(1, int(w * scale))
            new_h = max(1, int(h * scale))
            pil = pil.resize((new_w, new_h), PilImage.LANCZOS)
        buf = io.BytesIO()
        pil.save(buf, format="JPEG", quality=80)
        return base64.b64encode(buf.getvalue()).decode("ascii")
    except Exception as e:
        logger.debug("VLMSceneAgent: PIL encode failed: %s", e)
        return None


def encode_image_b64(image: np.ndarray) -> str | None:
    """Encode numpy image to base64 JPEG string. Tries OpenCV first, then PIL."""
    b64 = _encode_image(image)
    if b64 is None:
        b64 = _encode_image_pil(image)
    return b64


class VLMSceneAgent:
    """VLM scene understanding for the navigation agent.

    Usage::

        agent = VLMSceneAgent(llm_client)
        desc = await agent.describe_scene(rgb_frame)
        result = await agent.find_object_in_scene(rgb_frame, "red backpack")
    """

    def __init__(self, llm_client: Any):
        """
        Args:
            llm_client: Any LLMClientBase subclass. Vision calls are skipped
                        gracefully when the backend doesn't support images.
        """
        self._llm = llm_client
        # Detect whether the backend supports vision
        self._has_vision = self._check_vision_support()

    def _check_vision_support(self) -> bool:
        """Return True if the LLM client is known to support vision input."""
        if self._llm is None:
            return False
        cls_name = type(self._llm).__name__
        # MoonshotClient explicitly raises LLMError on vision calls
        if cls_name == "MoonshotClient":
            return False
        # QwenClient uses DashScope sync SDK — no vision path implemented
        if cls_name == "QwenClient":
            return False
        # MockLLMClient: treat as supporting vision so offline tests run
        if cls_name == "MockLLMClient":
            return True
        # OpenAIClient and ClaudeClient both support vision
        if hasattr(self._llm, "chat_with_image") or cls_name in (
            "OpenAIClient", "ClaudeClient"
        ):
            return True
        return False

    async def describe_scene(
        self,
        image: np.ndarray,
        context: str = "",
    ) -> str:
        """Ask the VLM to describe what it sees in the current camera frame.

        Args:
            image: RGB/BGR uint8 numpy array from the robot camera.
            context: Optional hint about what the robot is trying to do.

        Returns:
            Natural language scene description, or a fallback string on error.
        """
        if not self._has_vision or image is None:
            return "VLM scene description not available (no vision support)"

        b64 = encode_image_b64(image)
        if b64 is None:
            return "VLM scene description not available (image encoding failed)"

        prompt = "Describe what you see in this robot camera image briefly and accurately."
        if context:
            prompt += f" The robot is currently trying to: {context}"

        try:
            return await self._vlm_call(prompt, b64)
        except Exception as e:
            logger.warning("VLMSceneAgent.describe_scene failed: %s", e)
            return f"Scene description unavailable: {e}"

    async def find_object_in_scene(
        self,
        image: np.ndarray,
        target: str,
    ) -> dict:
        """Ask the VLM to locate a specific object in the camera frame.

        Args:
            image: RGB/BGR uint8 numpy array.
            target: Natural language description of the object to find.

        Returns:
            dict with keys:
              found (bool): whether the object appears to be visible
              description (str): what the VLM sees regarding the target
              direction (str): rough direction — "left", "right", "center", "not visible"
              estimated_distance (str): rough distance — "close", "medium", "far", "unknown"
        """
        fallback = {
            "found": False,
            "description": "VLM not available",
            "direction": "unknown",
            "estimated_distance": "unknown",
        }

        if not self._has_vision or image is None:
            return fallback

        b64 = encode_image_b64(image)
        if b64 is None:
            return {**fallback, "description": "Image encoding failed"}

        prompt = (
            f"Look for: '{target}'\n"
            "Respond in JSON only:\n"
            '{"found": true/false, "description": "...", '
            '"direction": "left|right|center|not visible", '
            '"estimated_distance": "close (<1m)|medium (1-3m)|far (>3m)|unknown"}'
        )

        try:
            raw = await self._vlm_call(prompt, b64)
            return self._parse_json_response(raw, fallback)
        except Exception as e:
            logger.warning("VLMSceneAgent.find_object_in_scene failed: %s", e)
            return {**fallback, "description": f"VLM call failed: {e}"}

    async def assess_situation(
        self,
        image: np.ndarray,
        goal: str,
        scene_graph: dict | None = None,
    ) -> dict:
        """Ask the VLM whether the current view helps the robot reach its goal.

        Args:
            image: RGB/BGR uint8 numpy array.
            goal: The navigation goal (natural language).
            scene_graph: Optional scene graph dict for additional context.

        Returns:
            dict with keys:
              relevant (bool): whether what's visible is relevant to the goal
              suggestion (str): what the robot should do next based on the view
              obstacles (list[str]): notable obstacles or hazards visible
        """
        fallback = {
            "relevant": False,
            "suggestion": "VLM assessment not available",
            "obstacles": [],
        }

        if not self._has_vision or image is None:
            return fallback

        b64 = encode_image_b64(image)
        if b64 is None:
            return {**fallback, "suggestion": "Image encoding failed"}

        sg_hint = ""
        if scene_graph:
            objects = scene_graph.get("objects", [])
            if objects:
                labels = [o.get("label", "") for o in objects[:10] if o.get("label")]
                sg_hint = f"\nKnown objects from scene graph: {', '.join(labels)}"

        prompt = (
            f"Robot goal: '{goal}'{sg_hint}\n"
            "Looking at the current camera view:\n"
            "1. Is what you see relevant to reaching the goal?\n"
            "2. What should the robot do next based on what it sees?\n"
            "3. What obstacles or hazards are visible?\n\n"
            "Respond in JSON only:\n"
            '{"relevant": true/false, "suggestion": "...", "obstacles": ["..."]}'
        )

        try:
            raw = await self._vlm_call(prompt, b64)
            return self._parse_json_response(raw, fallback)
        except Exception as e:
            logger.warning("VLMSceneAgent.assess_situation failed: %s", e)
            return {**fallback, "suggestion": f"VLM call failed: {e}"}

    async def compare_scenes(
        self,
        image1: np.ndarray,
        image2: np.ndarray,
    ) -> str:
        """Ask the VLM to describe the difference between two camera frames.

        Useful for confirming that the robot has moved or that a situation
        has changed (e.g. obstacle cleared, reached a new area).

        Args:
            image1: First frame (earlier in time).
            image2: Second frame (later in time).

        Returns:
            Natural language description of what changed between the two frames.
        """
        if not self._has_vision:
            return "VLM scene comparison not available (no vision support)"
        if image1 is None or image2 is None:
            return "VLM scene comparison not available (missing image)"

        b64_1 = encode_image_b64(image1)
        b64_2 = encode_image_b64(image2)
        if b64_1 is None or b64_2 is None:
            return "VLM scene comparison not available (image encoding failed)"

        # Send both images in a single message — all vision backends accept
        # multiple image_url entries in the same content block.
        try:
            return await self._vlm_call_two_images(
                "The robot took two camera images at different times. "
                "Image 1 is earlier, Image 2 is later. "
                "Briefly describe what changed between them.",
                b64_1,
                b64_2,
            )
        except Exception as e:
            logger.warning("VLMSceneAgent.compare_scenes failed: %s", e)
            return f"Scene comparison unavailable: {e}"

    # ── Internal VLM call helpers ─────────────────────────────────────────────

    async def _vlm_call(self, prompt: str, image_b64: str) -> str:
        """Send a single-image VLM request using the best available method."""
        cls_name = type(self._llm).__name__

        # MockLLMClient — return a plausible canned response
        if cls_name == "MockLLMClient":
            return await self._mock_vision_response(prompt)

        # OpenAIClient / MoonshotClient — use the dedicated vision method
        if hasattr(self._llm, "chat_with_image"):
            return await self._llm.chat_with_image(
                text_prompt=prompt,
                image_base64=image_b64,
                temperature=0.3,
            )

        # ClaudeClient — use generic chat() with multimodal content block
        if cls_name == "ClaudeClient":
            return await self._claude_vision_call(prompt, image_b64)

        # Unknown backend — attempt OpenAI-style multimodal chat()
        return await self._generic_vision_chat(prompt, image_b64)

    async def _vlm_call_two_images(
        self, prompt: str, b64_1: str, b64_2: str
    ) -> str:
        """Send two images in a single VLM request."""
        cls_name = type(self._llm).__name__

        if cls_name == "MockLLMClient":
            return await self._mock_vision_response(prompt)

        # For two-image calls we always use the generic OpenAI multimodal format
        # since chat_with_image() only accepts one image.
        return await self._generic_vision_chat(
            prompt, b64_1, extra_images=[b64_2]
        )

    async def _claude_vision_call(self, prompt: str, image_b64: str) -> str:
        """Vision call for ClaudeClient using the Anthropic multimodal format."""
        messages = [
            {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": "image/jpeg",
                            "data": image_b64,
                        },
                    },
                    {"type": "text", "text": prompt},
                ],
            }
        ]
        return await self._llm.chat(messages, temperature=0.3)

    async def _generic_vision_chat(
        self,
        prompt: str,
        image_b64: str,
        extra_images: list | None = None,
    ) -> str:
        """Generic OpenAI-compatible multimodal chat() call."""
        content = [
            {"type": "text", "text": prompt},
            {
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"},
            },
        ]
        for extra_b64 in (extra_images or []):
            content.append({
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{extra_b64}"},
            })
        messages = [{"role": "user", "content": content}]
        return await self._llm.chat(messages, temperature=0.3)

    @staticmethod
    async def _mock_vision_response(prompt: str) -> str:
        """Return a canned mock response for offline testing."""
        await asyncio.sleep(0.01)  # simulate async I/O
        prompt_lower = prompt.lower()
        if "find" in prompt_lower or "look for" in prompt_lower:
            return json.dumps({
                "found": True,
                "description": "Mock: target object visible ahead",
                "direction": "center",
                "estimated_distance": "medium (1-3m)",
            })
        if "assess" in prompt_lower or "goal" in prompt_lower:
            return json.dumps({
                "relevant": True,
                "suggestion": "Mock: continue forward toward the target area",
                "obstacles": [],
            })
        if "changed" in prompt_lower or "compare" in prompt_lower:
            return "Mock: the robot has moved forward; the view has shifted slightly."
        return "Mock: a corridor with white walls, clear path ahead."

    @staticmethod
    def _parse_json_response(raw: str, fallback: dict) -> dict:
        """Try to extract a JSON object from the VLM response text."""
        # Try direct parse first
        try:
            data = json.loads(raw.strip())
            if isinstance(data, dict):
                return {**fallback, **data}
        except (json.JSONDecodeError, ValueError):
            pass

        # Scan for a balanced JSON object in the text
        start = raw.find("{")
        if start >= 0:
            depth = 0
            for i in range(start, len(raw)):
                if raw[i] == "{":
                    depth += 1
                elif raw[i] == "}":
                    depth -= 1
                    if depth == 0:
                        try:
                            data = json.loads(raw[start : i + 1])
                            if isinstance(data, dict):
                                return {**fallback, **data}
                        except (json.JSONDecodeError, ValueError):
                            pass
                        break

        # No JSON found — treat the raw text as the description/suggestion field
        result = dict(fallback)
        if "description" in fallback:
            result["description"] = raw.strip()
        elif "suggestion" in fallback:
            result["suggestion"] = raw.strip()
        return result
