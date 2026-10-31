"""Small, deterministic object-query normalization for semantic grounding.

The query layer only turns a user's object phrase into a symbolic label and
aliases. It never invents a map pose. Geometry and native navigation remain
owned by the scene query and planner layers.
"""

from __future__ import annotations

import re
import unicodedata
from dataclasses import dataclass


@dataclass(frozen=True)
class ObjectQuery:
    """A text-derived object request that still needs map grounding."""

    raw_text: str
    label: str
    aliases: tuple[str, ...]
    quantity: int = 1
    action: str = "find"
    source: str = "rules"


_CHINESE_QUANTITY = re.compile(r"^(?P<quantity>[一二两三四五六七八九十百\d]+)(?P<unit>瓶|罐|杯|个|只|件|台|本|张|把)?")
_FIND_PREFIX = re.compile(
    r"^(?:请|麻烦|帮我|能不能|可以|帮忙)?(?:帮我)?(?:找一下|找一找|找到|找|搜一下|搜索|定位|发现|看看|查看)?"
)
_EN_FIND_PREFIX = re.compile(r"^(?:please\s+)?(?:find|locate|look\s+for|search\s+for)\s+", re.I)

_ALIASES: dict[str, tuple[str, ...]] = {
    "water_bottle": (
        "water_bottle",
        "water bottle",
        "bottle of water",
        "bottled water",
        "bottle",
        "水瓶",
        "瓶装水",
        "矿泉水",
        "水",
    ),
}


def _compact(text: str) -> str:
    text = unicodedata.normalize("NFKC", str(text or "")).strip()
    return re.sub(r"[\s，。！？、,.!?;；:：]+", "", text)


def _number(token: str) -> int:
    if token.isdigit():
        return max(1, int(token))
    values = {"一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}
    if token in values:
        return values[token]
    if token == "十":
        return 10
    if token.endswith("十") and token[:-1] in values:
        return values[token[:-1]] * 10
    return 1


def _canonicalize(value: str) -> tuple[str, tuple[str, ...]]:
    normalized = _compact(value).lower()
    for label, aliases in _ALIASES.items():
        if normalized in {_compact(alias).lower() for alias in aliases}:
            return label, aliases
    return normalized, (normalized,)


def parse_object_query(text: str) -> ObjectQuery | None:
    """Parse a find request into a canonical label without using an LLM.

    Known water phrases are intentionally explicit because ``水`` by itself
    is not a geometric object. It resolves to ``water_bottle`` only as a
    query alias; an actual map instance still has to provide matching semantic
    evidence and bound geometry.
    """

    raw = unicodedata.normalize("NFKC", str(text or "")).strip()
    if not raw:
        return None
    compact = _compact(raw)
    remainder = _FIND_PREFIX.sub("", compact, count=1)
    if remainder == compact:
        remainder = _EN_FIND_PREFIX.sub("", raw.strip(), count=1).strip()
        if remainder == raw.strip():
            return None
    if not remainder:
        return None
    remainder = re.sub(r"^(?:a|an|the)\s+", "", remainder, flags=re.IGNORECASE)

    quantity = 1
    match = _CHINESE_QUANTITY.match(remainder)
    if match and match.group("unit"):
        quantity = _number(match.group("quantity"))
        remainder = remainder[match.end() :]
    remainder = remainder.strip(" \t，。！？、,.!?;；:：")
    if not remainder:
        return None
    label, aliases = _canonicalize(remainder)
    return ObjectQuery(raw_text=raw, label=label, aliases=aliases, quantity=quantity)


__all__ = ["ObjectQuery", "parse_object_query"]
