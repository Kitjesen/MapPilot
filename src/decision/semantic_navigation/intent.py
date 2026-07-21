"""Deterministic fast path and guarded LLM fallback for semantic navigation.

This module deliberately stops at symbolic intent.  Coordinates must come from
the local map/place catalog after parsing, never from a language model.
"""

from __future__ import annotations

import re
import unicodedata
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Any


class SemanticAction(str, Enum):
    """Commands admitted by the first semantic-navigation interface."""

    NAVIGATE = "navigate"
    START_TOUR = "start_tour"
    PAUSE_TOUR = "pause_tour"
    RESUME_TOUR = "resume_tour"
    CANCEL_TOUR = "cancel_tour"


class TravelMode(str, Enum):
    """User preference for a cross-floor connector."""

    ANY = "any"
    STAIRS = "stairs"
    ELEVATOR = "elevator"


@dataclass(frozen=True)
class SemanticIntent:
    """A symbolic command that is safe to ground against local map data."""

    action: SemanticAction
    raw_text: str
    target_query: str = ""
    floor_id: str = ""
    tour_id: str = ""
    travel_mode: TravelMode = TravelMode.ANY
    confidence: float = 1.0
    source: str = "rules"
    needs_clarification: bool = False
    reason: str = ""


class SymbolicIntentError(ValueError):
    """Raised when a slow-path result crosses the symbolic intent boundary."""


SlowParser = Callable[[str], Mapping[str, Any] | None]

_CHINESE_DIGITS = {
    "零": 0,
    "〇": 0,
    "一": 1,
    "二": 2,
    "两": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    "六": 6,
    "七": 7,
    "八": 8,
    "九": 9,
}
_FLOOR_PATTERN = r"(?:第)?(?:负)?(?:\d{1,3}|[零〇一二两三四五六七八九十百]+)(?:楼|层)"
_FLOOR_RE = re.compile(rf"(?P<floor>{_FLOOR_PATTERN})")
_PUNCTUATION_RE = re.compile(r"[\s，。！？、,.!?;；:：]+")
_NAV_PREFIX_RE = re.compile(r"^(?:请|麻烦|帮我|可以)?(?:带我到|带我去|带我前往|导航到|导航去|前往|去往|走到|到|去)+")
_STAIR_RE = re.compile(r"(?:走|经由|通过|从|使用)(?:楼梯|步梯|安全梯)(?:上|下|去|到|前往)?")
_ELEVATOR_RE = re.compile(r"(?:坐|乘|搭乘|搭|使用|通过)(?:电梯|升降梯)(?:上|下|去|到|前往)?")

_FORBIDDEN_COORDINATE_KEYS = {
    "x",
    "y",
    "z",
    "yaw",
    "pose",
    "position",
    "coordinates",
    "target_x",
    "target_y",
    "target_z",
    "latitude",
    "longitude",
}
_ALLOWED_SYMBOLIC_KEYS = {
    "action",
    "target_query",
    "floor_id",
    "tour_id",
    "travel_mode",
    "confidence",
    "needs_clarification",
    "reason",
}


def _compact(text: str) -> str:
    normalized = unicodedata.normalize("NFKC", str(text or "")).strip()
    return _PUNCTUATION_RE.sub("", normalized)


def _chinese_integer(token: str) -> int | None:
    if not token:
        return None
    if token.isdigit():
        return int(token)
    if token in _CHINESE_DIGITS:
        return _CHINESE_DIGITS[token]

    # Floor numbers are small in practice.  This covers 十, 十六, 二十, 二十三,
    # and the analogous 百 forms without adding a general numeral dependency.
    total = 0
    current = 0
    for char in token:
        if char in _CHINESE_DIGITS:
            current = _CHINESE_DIGITS[char]
        elif char == "十":
            total += (current or 1) * 10
            current = 0
        elif char == "百":
            total += (current or 1) * 100
            current = 0
        else:
            return None
    return total + current


def normalize_floor_id(value: str | int | None) -> str:
    """Normalize common Chinese/Arabic floor forms to building floor IDs."""

    if value is None:
        return ""
    if isinstance(value, bool):
        return ""
    if isinstance(value, int):
        return _floor_id(value)
    text = _compact(str(value)).lower()
    if not text:
        return ""
    basement = re.fullmatch(r"floor[-_]?b(\d+)", text)
    if basement:
        return _floor_id(-int(basement.group(1)))
    existing = re.fullmatch(r"floor[-_]?(-?\d+)", text)
    if existing:
        return _floor_id(int(existing.group(1)))
    text = text.removeprefix("第")
    text = text.removesuffix("楼").removesuffix("层")
    negative = text.startswith("负")
    if negative:
        text = text[1:]
    number = _chinese_integer(text)
    if number is None:
        return ""
    return _floor_id(-number if negative else number)


def _floor_id(number: int) -> str:
    return f"floor-{number}" if number >= 0 else f"floor-b{abs(number)}"


class HybridSemanticIntentParser:
    """Use an offline grammar first and an optional symbolic slow path second."""

    def __init__(self, slow_parser: SlowParser | None = None) -> None:
        self._slow_parser = slow_parser

    def parse(self, text: str) -> SemanticIntent | None:
        """Parse one utterance without ever accepting model-generated poses."""

        raw_text = unicodedata.normalize("NFKC", str(text or "")).strip()
        if not raw_text:
            return None
        fast = self._parse_rules(raw_text)
        if fast is not None:
            return fast
        if self._slow_parser is None:
            return None
        payload = self._slow_parser(raw_text)
        if payload is None:
            return None
        return self.from_symbolic_mapping(payload, raw_text=raw_text)

    @staticmethod
    def from_symbolic_mapping(
        payload: Mapping[str, Any],
        *,
        raw_text: str,
    ) -> SemanticIntent:
        """Validate a model/tool result at the no-coordinate boundary."""

        _reject_coordinate_fields(payload)
        unknown_keys = {
            str(key).strip().lower() for key in payload if str(key).strip().lower() not in _ALLOWED_SYMBOLIC_KEYS
        }
        if unknown_keys:
            fields = ", ".join(sorted(unknown_keys))
            raise SymbolicIntentError(f"unsupported symbolic fields: {fields}")
        try:
            action = SemanticAction(_symbolic_text(payload, "action", max_length=32).lower())
        except ValueError as exc:
            raise SymbolicIntentError("unsupported symbolic action") from exc
        try:
            travel_mode = TravelMode(
                _symbolic_text(
                    payload,
                    "travel_mode",
                    default=TravelMode.ANY.value,
                    max_length=32,
                ).lower()
            )
        except ValueError as exc:
            raise SymbolicIntentError("unsupported travel mode") from exc

        target_query = _symbolic_text(payload, "target_query", max_length=256)
        tour_id = _symbolic_text(payload, "tour_id", max_length=128)
        reason = _symbolic_text(payload, "reason", max_length=256)
        raw_floor = payload.get("floor_id")
        if isinstance(raw_floor, bool) or (raw_floor is not None and not isinstance(raw_floor, (str, int))):
            raise SymbolicIntentError("floor_id must be text or an integer")
        floor_id = normalize_floor_id(raw_floor)
        if raw_floor not in (None, "") and not floor_id:
            raise SymbolicIntentError("unsupported floor_id")
        needs_clarification = payload.get("needs_clarification", False)
        if not isinstance(needs_clarification, bool):
            raise SymbolicIntentError("needs_clarification must be a boolean")
        if action is SemanticAction.NAVIGATE and not target_query and not needs_clarification:
            raise SymbolicIntentError("navigate requires target_query")
        if action is SemanticAction.START_TOUR and not tour_id and not needs_clarification:
            raise SymbolicIntentError("start_tour requires tour_id")
        return SemanticIntent(
            action=action,
            raw_text=raw_text,
            target_query=target_query,
            floor_id=floor_id,
            tour_id=tour_id,
            travel_mode=travel_mode,
            confidence=_bounded_confidence(payload.get("confidence", 0.7)),
            source="llm",
            needs_clarification=needs_clarification,
            reason=reason,
        )

    def _parse_rules(self, raw_text: str) -> SemanticIntent | None:
        compact = _compact(raw_text)

        if compact in {"暂停导览", "暂停展厅导览", "导览暂停"}:
            return SemanticIntent(SemanticAction.PAUSE_TOUR, raw_text)
        if compact in {"继续", "继续导览", "恢复导览", "导览继续"}:
            return SemanticIntent(SemanticAction.RESUME_TOUR, raw_text)
        if compact in {"取消导览", "结束导览", "停止导览", "终止导览"}:
            return SemanticIntent(SemanticAction.CANCEL_TOUR, raw_text)
        if compact.startswith(("开始", "启动")) and "导览" in compact:
            tour_id = compact[2:]
            if tour_id:
                return SemanticIntent(
                    SemanticAction.START_TOUR,
                    raw_text,
                    target_query=tour_id,
                    tour_id=tour_id,
                )

        travel_mode = TravelMode.ANY
        navigation_text = compact
        if _STAIR_RE.search(navigation_text):
            travel_mode = TravelMode.STAIRS
            navigation_text = _STAIR_RE.sub("", navigation_text, count=1)
        elif _ELEVATOR_RE.search(navigation_text):
            travel_mode = TravelMode.ELEVATOR
            navigation_text = _ELEVATOR_RE.sub("", navigation_text, count=1)

        floor_match = _FLOOR_RE.search(navigation_text)
        floor_id = normalize_floor_id(floor_match.group("floor")) if floor_match else ""
        if floor_match:
            navigation_text = navigation_text[: floor_match.start()] + navigation_text[floor_match.end() :]
        stripped = _NAV_PREFIX_RE.sub("", navigation_text, count=1).strip()
        looks_like_navigation = bool(
            travel_mode is not TravelMode.ANY
            or _NAV_PREFIX_RE.match(navigation_text)
            or (floor_match and floor_match.start() == 0 and stripped)
        )
        if not looks_like_navigation:
            return None
        if not stripped:
            return SemanticIntent(
                SemanticAction.NAVIGATE,
                raw_text,
                floor_id=floor_id,
                travel_mode=travel_mode,
                confidence=0.55,
                needs_clarification=True,
                reason="place_required",
            )
        return SemanticIntent(
            SemanticAction.NAVIGATE,
            raw_text,
            target_query=stripped,
            floor_id=floor_id,
            travel_mode=travel_mode,
            confidence=0.98 if floor_id else 0.92,
        )


def _bounded_confidence(value: Any) -> float:
    if isinstance(value, bool):
        raise SymbolicIntentError("confidence must be numeric")
    try:
        confidence = float(value)
    except (TypeError, ValueError) as exc:
        raise SymbolicIntentError("confidence must be numeric") from exc
    if not 0.0 <= confidence <= 1.0:
        raise SymbolicIntentError("confidence must be between 0 and 1")
    return confidence


def _symbolic_text(
    payload: Mapping[str, Any],
    key: str,
    *,
    default: str = "",
    max_length: int,
) -> str:
    value = payload.get(key, default)
    if value is None:
        return default
    if not isinstance(value, str):
        raise SymbolicIntentError(f"{key} must be text")
    text = value.strip()
    if len(text) > max_length:
        raise SymbolicIntentError(f"{key} is too long")
    return text


def _reject_coordinate_fields(payload: Mapping[str, Any]) -> None:
    for key, value in payload.items():
        normalized_key = str(key).strip().lower()
        if normalized_key in _FORBIDDEN_COORDINATE_KEYS:
            raise SymbolicIntentError(f"coordinate field is forbidden: {normalized_key}")
        if isinstance(value, Mapping):
            _reject_coordinate_fields(value)
        elif isinstance(value, list):
            for item in value:
                if isinstance(item, Mapping):
                    _reject_coordinate_fields(item)
