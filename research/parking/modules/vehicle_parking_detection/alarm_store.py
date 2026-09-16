from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

import cv2

from .parking_engine import ParkingTrigger


class AlarmStore:
    def __init__(self, output_dir: str | Path):
        self.output_dir = Path(output_dir)
        self.image_dir = self.output_dir / "alarm_images"
        self.events_jsonl = self.output_dir / "alarm_events.jsonl"
        self.latest_json = self.output_dir / "latest_alarm_event.json"
        self.image_dir.mkdir(parents=True, exist_ok=True)

    def save(self, frame, trigger: ParkingTrigger) -> dict[str, str]:
        now = datetime.now().astimezone()
        safe_location = self._safe_name(trigger.location)
        event_id = f"{now.strftime('%Y%m%d_%H%M%S_%f')}_{safe_location}"
        image_name = f"{event_id}.jpg"
        image_path = self.image_dir / image_name
        cv2.imwrite(str(image_path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])

        payload = {
            "keyframe_image": f"alarm_images/{image_name}",
            "time": now.isoformat(timespec="seconds"),
            "event_name": trigger.event_name,
            "location": trigger.location,
        }
        self.output_dir.mkdir(parents=True, exist_ok=True)
        with self.events_jsonl.open("a", encoding="utf-8") as file:
            file.write(json.dumps(payload, ensure_ascii=False) + "\n")
        self.latest_json.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return payload

    @staticmethod
    def _safe_name(value: str) -> str:
        text = "".join(ch if ch.isascii() and (ch.isalnum() or ch in "._-") else "_" for ch in str(value))
        return text.strip("_") or "location"
