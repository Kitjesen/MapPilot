# Inspired by DimOS navigation/visual/query.py, Apache 2.0 License
"""
VLM 开放词汇 bbox 检测 — "图里有X吗？在哪？"
借鉴 DimOS navigation/visual/query.py

用法:
  bbox = await query_object_bbox(llm_client, image_base64, "红色椅子")
  # bbox = [x1, y1, x2, y2] 或 None

  results = await query_multiple_objects(llm_client, image_base64, "椅子")
  # [{"name": "椅子", "bbox": [x1,y1,x2,y2], "confidence": 0.9}, ...]

适配 LingTu LLM 后端:
  - OpenAIClient (GPT-4o Vision) — 完整支持
  - ClaudeClient — 支持图像输入
  - MoonshotClient — 不支持视觉，会抛 LLMError
  函数接受任何有 chat_with_image(text_prompt, image_base64, system_prompt) 方法的客户端。
"""

import asyncio
import hashlib
import json
import logging
import re
import time
from typing import Any

logger = logging.getLogger(__name__)


# ================================================================
#  W2-11: in-memory bbox cache with exponential-backoff retry
# ================================================================

_CACHE_TTL_S: float = 5.0
_RETRY_BACKOFFS: tuple[float, ...] = (0.5, 1.5, 3.0)  # 3 attempts total

# Module-level cache: key -> {"bbox": [x,y,x,y], "confidence": float, "ts": float}
_bbox_cache: dict[str, dict[str, Any]] = {}


def _cache_key(target_object: str, image_base64: str) -> str:
    """Stable cache key from (target, image hash)."""
    img_hash = hashlib.md5(image_base64.encode("utf-8", errors="ignore")).hexdigest()[:16]
    return f"{target_object}|{img_hash}"


def _cache_put(key: str, bbox: list[float], confidence: float = 1.0) -> None:
    """Store bbox with timestamp; older entries aren't purged until access."""
    _bbox_cache[key] = {
        "bbox": list(bbox),
        "confidence": float(confidence),
        "ts": time.time(),
    }


def _cache_get(key: str, allow_stale: bool = False) -> dict[str, Any] | None:
    """Fetch cache entry. Returns None when missing or (when allow_stale=False)
    past the TTL. `allow_stale=True` is used by the retry-exhaustion path to
    return a low-confidence stale reading instead of nothing."""
    entry = _bbox_cache.get(key)
    if entry is None:
        return None
    age = time.time() - entry["ts"]
    if allow_stale:
        return entry
    return entry if age < _CACHE_TTL_S else None


async def query_object_bbox(
    llm_client,
    image_base64: str,
    target_description: str,
    language: str = "zh",
) -> list[float] | None:
    """Locate a single object bbox via VLM, with retry + cache.

    Behaviour:
      1. Fresh cache hit within TTL — return immediately, no VLM call.
      2. Call VLM with exponential-backoff retries at 0.5s / 1.5s / 3.0s.
      3. On success, populate cache and return the bbox.
      4. On all retries failing, return a stale cache entry (low confidence)
         if one exists; else None. NEVER returns a fabricated value.
    """
    client = _resolve_vision_client(llm_client)
    if client is None:
        logger.warning("query_object_bbox: no vision-capable client available")
        return None

    key = _cache_key(target_description, image_base64)

    # Fresh cache short-circuits the VLM call entirely.
    fresh = _cache_get(key, allow_stale=False)
    if fresh is not None:
        return list(fresh["bbox"])

    system_prompt, user_prompt = _build_bbox_prompt(target_description, language)

    last_exc: Exception | None = None
    for attempt_idx, backoff in enumerate(_RETRY_BACKOFFS, start=1):
        try:
            response = await client.chat_with_image(
                text_prompt=user_prompt,
                image_base64=image_base64,
                system_prompt=system_prompt,
            )
            bbox = _extract_bbox_from_response(response)
            if bbox is not None:
                _cache_put(key, bbox, confidence=1.0)
            return bbox
        except Exception as e:
            last_exc = e
            logger.warning(
                "query_object_bbox attempt %d/%d failed: %s",
                attempt_idx, len(_RETRY_BACKOFFS), e,
            )
            if attempt_idx < len(_RETRY_BACKOFFS):
                await asyncio.sleep(backoff)

    # All retries exhausted — try stale cache before giving up.
    stale = _cache_get(key, allow_stale=True)
    if stale is not None:
        logger.error(
            "query_object_bbox: all %d retries failed (last err=%s); "
            "returning stale cache entry (age=%.1fs) at low confidence",
            len(_RETRY_BACKOFFS), last_exc, time.time() - stale["ts"],
        )
        return list(stale["bbox"])

    logger.error(
        "query_object_bbox: all %d retries failed (last err=%s) and no cache",
        len(_RETRY_BACKOFFS), last_exc,
    )
    return None


async def query_multiple_objects(
    llm_client,
    image_base64: str,
    target_description: str,
    language: str = "zh",
) -> list[dict[str, Any]]:
    """
    用 VLM 在当前图像中定位多个匹配目标。

    Args:
        llm_client: LLM 客户端（需支持 chat_with_image）
        image_base64: JPEG 图像的 base64 编码
        target_description: 目标描述（如 "椅子", "door"）
        language: "zh" 或 "en"

    Returns:
        [{"name": "红色椅子", "bbox": [x1,y1,x2,y2], "confidence": 0.9}, ...]
        若无匹配则返回空列表。
    """
    client = _resolve_vision_client(llm_client)
    if client is None:
        logger.warning("query_multiple_objects: no vision-capable client available")
        return []

    system_prompt, user_prompt = _build_multi_bbox_prompt(target_description, language)

    try:
        response = await client.chat_with_image(
            text_prompt=user_prompt,
            image_base64=image_base64,
            system_prompt=system_prompt,
        )
    except Exception as e:
        logger.warning("query_multiple_objects: VLM call failed: %s", e)
        return []

    return _extract_multi_bbox_from_response(response)


# ================================================================
#  内部辅助
# ================================================================

def _resolve_vision_client(llm_client):
    """
    从传入对象中取出支持 chat_with_image 的客户端。

    兼容两种传入方式:
    1. 直接传 LLMClientBase 子类（OpenAIClient / ClaudeClient）
    2. 传 GoalResolver 实例 — 内部持有 _primary / _fallback
    """
    # 直接有 chat_with_image 方法
    if hasattr(llm_client, "chat_with_image"):
        return llm_client

    # GoalResolver / 复合客户端 — 尝试找内部的视觉后端
    for attr in ("_primary", "_fallback", "_client"):
        candidate = getattr(llm_client, attr, None)
        if candidate is not None and hasattr(candidate, "chat_with_image"):
            return candidate

    return None


def _build_bbox_prompt(target: str, language: str) -> tuple[str, str]:
    """构建单目标 VLM bbox 查询的 system + user prompt。"""
    if language == "zh":
        system = (
            "你是一个机器人视觉助手，负责在图像中定位目标物体。\n"
            "只输出严格的 JSON，不要任何额外文字。\n"
            "格式: {\"name\": \"物体名称\", \"bbox\": [x1, y1, x2, y2]}\n"
            "其中 x1,y1 是左上角像素坐标，x2,y2 是右下角像素坐标。\n"
            "如果图中没有该物体，返回: {\"name\": null, \"bbox\": null}"
        )
        user = (
            f"请在这张图片中找到 '{target}'。\n"
            "返回 JSON 格式的 bbox 坐标，未找到则返回 null。"
        )
    else:
        system = (
            "You are a robot vision assistant that locates objects in images.\n"
            "Output ONLY strict JSON, no extra text.\n"
            "Format: {\"name\": \"object_name\", \"bbox\": [x1, y1, x2, y2]}\n"
            "where x1,y1 is the top-left and x2,y2 is the bottom-right corner in pixels.\n"
            "If the object is not found, return: {\"name\": null, \"bbox\": null}"
        )
        user = (
            f"Find '{target}' in this image.\n"
            "Return the bbox as JSON, or null if not found."
        )
    return system, user


def _build_multi_bbox_prompt(target: str, language: str) -> tuple[str, str]:
    """构建多目标 VLM bbox 查询的 system + user prompt。"""
    if language == "zh":
        system = (
            "你是一个机器人视觉助手，负责在图像中定位所有匹配的目标物体。\n"
            "只输出严格的 JSON 数组，不要任何额外文字。\n"
            "格式: [{\"name\": \"物体名称\", \"bbox\": [x1,y1,x2,y2], \"confidence\": 0.0-1.0}, ...]\n"
            "其中 x1,y1 是左上角像素坐标，x2,y2 是右下角像素坐标。\n"
            "如果图中没有该物体，返回空数组: []"
        )
        user = (
            f"请在这张图片中找到所有的 '{target}'。\n"
            "返回 JSON 数组格式的所有 bbox 坐标和置信度，未找到则返回 []。"
        )
    else:
        system = (
            "You are a robot vision assistant that locates all matching objects in images.\n"
            "Output ONLY a strict JSON array, no extra text.\n"
            "Format: [{\"name\": \"object_name\", \"bbox\": [x1,y1,x2,y2], \"confidence\": 0.0-1.0}, ...]\n"
            "where x1,y1 is the top-left and x2,y2 is the bottom-right corner in pixels.\n"
            "If no objects are found, return an empty array: []"
        )
        user = (
            f"Find all instances of '{target}' in this image.\n"
            "Return a JSON array with bbox and confidence for each match, or [] if none found."
        )
    return system, user


def _extract_bbox_from_response(response: str) -> list[float] | None:
    """
    从 VLM 响应中提取单个 bbox，容错处理。

    支持的格式:
      {"name": "...", "bbox": [x1,y1,x2,y2]}
      {"bbox": [x1,y1,x2,y2]}
      [x1,y1,x2,y2]
      "x1,y1,x2,y2"
      ```json ... ```
    """
    if not response or not response.strip():
        return None

    # 先尝试解析完整 JSON（含 markdown 代码块剥离）
    data = _parse_json_tolerant(response)
    if data is not None:
        # dict 形式: {"bbox": [...]} 或 {"name": ..., "bbox": [...]}
        if isinstance(data, dict):
            bbox_raw = data.get("bbox")
            if bbox_raw is None:
                # bbox is null → target not found
                return None
            return _coerce_bbox(bbox_raw)

        # 直接是数组形式: [x1,y1,x2,y2]
        if isinstance(data, list):
            return _coerce_bbox(data)

    # 最后尝试从纯文本中抠出 4 个数字
    numbers = re.findall(r"[-+]?\d*\.?\d+", response)
    if len(numbers) >= 4:
        try:
            coords = [float(n) for n in numbers[:4]]
            if _is_valid_bbox(coords):
                return coords
        except (ValueError, TypeError):
            pass

    logger.debug("_extract_bbox_from_response: could not parse bbox from: %.200s", response)
    return None


def _extract_multi_bbox_from_response(response: str) -> list[dict[str, Any]]:
    """
    从 VLM 响应中提取多目标 bbox 列表，容错处理。

    期望格式:
      [{"name": "...", "bbox": [x1,y1,x2,y2], "confidence": 0.9}, ...]
    """
    if not response or not response.strip():
        return []

    data = _parse_json_tolerant(response)
    if data is None:
        return []

    # 顶层是数组
    if isinstance(data, list):
        return _normalize_multi_results(data)

    # 顶层是 dict，可能包含 "objects" / "results" / "detections" 键
    for key in ("objects", "results", "detections", "items"):
        if key in data and isinstance(data[key], list):
            return _normalize_multi_results(data[key])

    # 单个 dict 当作只有一个结果
    if isinstance(data, dict) and "bbox" in data and data["bbox"] is not None:
        result = _normalize_single_result(data)
        return [result] if result else []

    return []


def _normalize_multi_results(raw_list: list) -> list[dict[str, Any]]:
    """规范化多目标结果列表，过滤掉解析失败的条目。"""
    out = []
    for item in raw_list:
        if not isinstance(item, dict):
            continue
        normalized = _normalize_single_result(item)
        if normalized:
            out.append(normalized)
    return out


def _normalize_single_result(item: dict) -> dict[str, Any] | None:
    """规范化单条结果，返回 {"name", "bbox", "confidence"} 或 None。"""
    bbox_raw = item.get("bbox")
    if bbox_raw is None:
        return None
    bbox = _coerce_bbox(bbox_raw)
    if bbox is None:
        return None
    try:
        confidence = float(item.get("confidence", item.get("score", 1.0)))
    except (TypeError, ValueError):
        confidence = 1.0
    return {
        "name": item.get("name", item.get("label", "")),
        "bbox": bbox,
        "confidence": min(max(confidence, 0.0), 1.0),
    }


def _parse_json_tolerant(text: str) -> Any:
    """
    容错 JSON 解析，处理:
    - markdown 代码块 (```json ... ```)
    - 前后多余文字
    - 截断 JSON（尝试修复）
    """
    # 剥离 markdown 代码块
    code_match = re.search(r"```(?:json)?\s*([\s\S]*?)```", text)
    if code_match:
        candidate = code_match.group(1).strip()
        try:
            return json.loads(candidate)
        except json.JSONDecodeError:
            pass

    # 直接尝试整段
    stripped = text.strip()
    try:
        return json.loads(stripped)
    except json.JSONDecodeError:
        pass

    # 找第一个 [ 或 { 到最后一个 ] 或 }
    for open_ch, close_ch in [("[", "]"), ("{", "}")]:
        start = stripped.find(open_ch)
        end = stripped.rfind(close_ch)
        if start != -1 and end > start:
            try:
                return json.loads(stripped[start:end + 1])
            except json.JSONDecodeError:
                pass

    return None


def _coerce_bbox(raw) -> list[float] | None:
    """将各种形式的 bbox 输入规范化为 [x1,y1,x2,y2] 浮点列表。"""
    if raw is None:
        return None

    # 已经是列表/元组
    if isinstance(raw, (list, tuple)):
        if len(raw) == 4:
            try:
                coords = [float(v) for v in raw]
                if _is_valid_bbox(coords):
                    return coords
            except (TypeError, ValueError):
                pass
        return None

    # 逗号分隔字符串: "x1,y1,x2,y2"
    if isinstance(raw, str):
        parts = re.split(r"[,\s]+", raw.strip())
        if len(parts) == 4:
            try:
                coords = [float(p) for p in parts]
                if _is_valid_bbox(coords):
                    return coords
            except (TypeError, ValueError):
                pass

    return None


def _is_valid_bbox(coords: list[float]) -> bool:
    """检查 bbox 坐标是否合理（非负、x2>x1、y2>y1）。"""
    if len(coords) != 4:
        return False
    x1, y1, x2, y2 = coords
    return x1 >= 0 and y1 >= 0 and x2 > x1 and y2 > y1
