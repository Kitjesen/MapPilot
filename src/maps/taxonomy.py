"""Versioned semantic taxonomy shared by perception, maps, and Gateway."""

from __future__ import annotations

import json
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Iterable

from runtime.msgs.numpy_compat import np


@dataclass(frozen=True)
class SemanticTaxonomy:
    name: str
    version: int
    ids: dict[str, int]
    palette: dict[str, dict[str, str]]

    def encode(self, labels: Iterable[str], *, excluded: frozenset[str] = frozenset()) -> np.ndarray:
        excluded_normalized = {str(label).strip().lower() for label in excluded}
        values = [
            0 if str(label).strip().lower() in excluded_normalized else self.ids.get(str(label).strip().lower(), 0)
            for label in labels
        ]
        return np.asarray(values, dtype=np.uint16)


@lru_cache(maxsize=4)
def load_semantic_taxonomy(path: str | Path | None = None) -> SemanticTaxonomy:
    if path is None:
        path = Path(__file__).resolve().parents[2] / "config" / "semantic_taxonomy.json"
    source = Path(path)
    payload = json.loads(source.read_text(encoding="utf-8"))
    name = str(payload.get("name") or "").strip()
    version = int(payload.get("version") or 0)
    classes = payload.get("classes")
    if not name or version <= 0 or not isinstance(classes, list):
        raise ValueError("semantic taxonomy requires name, positive version, and classes")
    ids: dict[str, int] = {}
    palette: dict[str, dict[str, str]] = {}
    seen_ids: set[int] = set()
    for item in classes:
        if not isinstance(item, dict):
            raise ValueError("semantic taxonomy classes must be objects")
        label_id = int(item.get("id", -1))
        class_name = str(item.get("name") or "").strip().lower()
        color = str(item.get("color") or "").strip()
        if not class_name or label_id < 0 or label_id > 65535 or label_id in seen_ids:
            raise ValueError("semantic taxonomy class id/name is invalid or duplicated")
        seen_ids.add(label_id)
        palette[str(label_id)] = {"name": class_name, "color": color}
        for alias in [class_name, *(item.get("aliases") or [])]:
            key = str(alias).strip().lower()
            previous = ids.get(key)
            if not key or (previous is not None and previous != label_id):
                raise ValueError("semantic taxonomy alias is empty or ambiguous")
            ids[key] = label_id
    if ids.get("unknown") != 0 or ids.get("background") != 0:
        raise ValueError("semantic taxonomy id 0 must represent unknown/background")
    return SemanticTaxonomy(name=name, version=version, ids=ids, palette=palette)
