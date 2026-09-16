from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class Detection:
    class_id: int
    class_name: str
    confidence: float
    box: tuple[float, float, float, float]
    track_id: int | None = None

    @property
    def box_area(self) -> float:
        x1, y1, x2, y2 = self.box
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    def to_dict(self) -> dict[str, Any]:
        return {
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": round(float(self.confidence), 4),
            "box_xyxy": [round(float(value), 2) for value in self.box],
            "track_id": self.track_id,
        }


@dataclass
class Track:
    id: int
    class_name: str
    box: tuple[float, float, float, float]
    confidence: float
    first_seen: float
    last_seen: float
    missed: int = 0

    @property
    def age_seconds(self) -> float:
        return max(0.0, self.last_seen - self.first_seen)
