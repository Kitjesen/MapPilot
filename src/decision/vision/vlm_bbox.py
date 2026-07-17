# Inspired by DimOS navigation/visual/query.py, Apache 2.0 License
"""VLM bounding-box query helper."""

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
    """Query object bbox."""
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
                attempt_idx,
                len(_RETRY_BACKOFFS),
                e,
            )
            if attempt_idx < len(_RETRY_BACKOFFS):
                await asyncio.sleep(backoff)

    stale = _cache_get(key, allow_stale=True)
    if stale is not None:
        logger.error(
            "query_object_bbox: all %d retries failed (last err=%s); "
            "returning stale cache entry (age=%.1fs) at low confidence",
            len(_RETRY_BACKOFFS),
            last_exc,
            time.time() - stale["ts"],
        )
        return list(stale["bbox"])

    logger.error(
        "query_object_bbox: all %d retries failed (last err=%s) and no cache",
        len(_RETRY_BACKOFFS),
        last_exc,
    )
    return None


async def query_multiple_objects(
    llm_client,
    image_base64: str,
    target_description: str,
    language: str = "zh",
) -> list[dict[str, Any]]:
    """Query multiple objects."""
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

# ================================================================


def _resolve_vision_client(llm_client):
    """Resolve vision client."""

    if hasattr(llm_client, "chat_with_image"):
        return llm_client

    for attr in ("_primary", "_fallback", "_client"):
        candidate = getattr(llm_client, attr, None)
        if candidate is not None and hasattr(candidate, "chat_with_image"):
            return candidate

    return None


def _build_bbox_prompt(target: str, language: str) -> tuple[str, str]:
    """Build a strict JSON prompt for a single object bbox."""
    system = (
        "You are a robot vision assistant that locates objects in images.\n"
        "Output ONLY strict JSON, no extra text.\n"
        'Format: {"name": "object_name", "bbox": [x1, y1, x2, y2]}\n'
        "where x1,y1 is the top-left and x2,y2 is the bottom-right corner in pixels.\n"
        'If the object is not found, return: {"name": null, "bbox": null}'
    )
    user = f"Find '{target}' in this image.\nReturn the bbox as JSON, or null if not found."
    return system, user


def _build_multi_bbox_prompt(target: str, language: str) -> tuple[str, str]:
    """Build a strict JSON prompt for multiple matching bboxes."""
    system = (
        "You are a robot vision assistant that locates all matching objects in images.\n"
        "Output ONLY a strict JSON array, no extra text.\n"
        'Format: [{"name": "object_name", "bbox": [x1,y1,x2,y2], "confidence": 0.0-1.0}, ...]\n'
        "where x1,y1 is the top-left and x2,y2 is the bottom-right corner in pixels.\n"
        "If no objects are found, return an empty array: []"
    )
    user = (
        f"Find all instances of '{target}' in this image.\n"
        "Return a JSON array with bbox and confidence for each match, or [] if none found."
    )
    return system, user


def _extract_bbox_from_response(response: str) -> list[float] | None:
    """Extract bbox from response."""
    if not response or not response.strip():
        return None

    data = _parse_json_tolerant(response)
    if data is not None:
        if isinstance(data, dict):
            bbox_raw = data.get("bbox")
            if bbox_raw is None:
                return None
            return _coerce_bbox(bbox_raw)

        if isinstance(data, list):
            return _coerce_bbox(data)

    numbers = re.findall(r"[-+].\d*\..\d+", response)
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
    """Extract multi bbox from response."""
    if not response or not response.strip():
        return []

    data = _parse_json_tolerant(response)
    if data is None:
        return []

    if isinstance(data, list):
        return _normalize_multi_results(data)

    for key in ("objects", "results", "detections", "items"):
        if key in data and isinstance(data[key], list):
            return _normalize_multi_results(data[key])

    if isinstance(data, dict) and "bbox" in data and data["bbox"] is not None:
        result = _normalize_single_result(data)
        return [result] if result else []

    return []


def _normalize_multi_results(raw_list: list) -> list[dict[str, Any]]:
    """Normalize multi results."""
    out = []
    for item in raw_list:
        if not isinstance(item, dict):
            continue
        normalized = _normalize_single_result(item)
        if normalized:
            out.append(normalized)
    return out


def _normalize_single_result(item: dict) -> dict[str, Any] | None:
    """Normalize single result."""
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
    """Parse json tolerant."""

    code_match = re.search(r"```(?:json)?\s*([\s\S]*?)```", text)
    if code_match:
        candidate = code_match.group(1).strip()
        try:
            return json.loads(candidate)
        except json.JSONDecodeError:
            pass

    stripped = text.strip()
    try:
        return json.loads(stripped)
    except json.JSONDecodeError:
        pass

    for open_ch, close_ch in [("[", "]"), ("{", "}")]:
        start = stripped.find(open_ch)
        end = stripped.rfind(close_ch)
        if start != -1 and end > start:
            try:
                return json.loads(stripped[start : end + 1])
            except json.JSONDecodeError:
                pass

    return None


def _coerce_bbox(raw) -> list[float] | None:
    """Coerce bbox."""
    if raw is None:
        return None

    if isinstance(raw, (list, tuple)):
        if len(raw) == 4:
            try:
                coords = [float(v) for v in raw]
                if _is_valid_bbox(coords):
                    return coords
            except (TypeError, ValueError):
                pass
        return None

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
    """Is valid bbox."""
    if len(coords) != 4:
        return False
    x1, y1, x2, y2 = coords
    return x1 >= 0 and y1 >= 0 and x2 > x1 and y2 > y1
