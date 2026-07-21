from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class RoiConfig:
    id: str
    name: str
    polygon: list[tuple[int, int]]
    dwell_seconds: float
    cooldown_seconds: float
    scene_id: str = ""
    scene_anchor: dict[str, Any] | None = None
    scene_anchors: list[dict[str, Any]] | None = None

    @property
    def location(self) -> str:
        return self.name or self.id


DEFAULT_POINT_ID = "no_parking_01"
DEFAULT_LOCATION = "禁停区01"
DEFAULT_EVENT_NAME = "车辆违停"


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def dump_yaml(path: str | Path, data: dict[str, Any]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False, width=120),
        encoding="utf-8",
    )


def normalize_polygon(raw_polygon: Any) -> list[tuple[int, int]]:
    points: list[tuple[int, int]] = []
    for item in raw_polygon or []:
        try:
            x, y = item
        except (TypeError, ValueError):
            continue
        points.append((int(round(float(x))), int(round(float(y)))))
    return points if len(points) >= 3 else []


def site_point(config: dict[str, Any], point_id: str = DEFAULT_POINT_ID) -> dict[str, Any]:
    points = config.get("points", []) or []
    for point in points:
        if str(point.get("point_id") or point.get("patrol_point_id") or "") == point_id:
            return point or {}
    if points:
        return points[0] or {}
    return {
        "point_id": point_id,
        "location": DEFAULT_LOCATION,
        "rois": [],
    }


def runtime_config(config: dict[str, Any]) -> dict[str, Any]:
    defaults = {
        "event_name": DEFAULT_EVENT_NAME,
        "vehicle_class": "vehicle",
        "record_fps": 15.0,
        "process_interval": 0.0667,
        "infer_interval": 0.0667,
        "input_height": 640,
        "input_width": 320,
        "input_format": "rgb_i8_centered",
        "hbm_backend": "hbm_runtime",
        "conf": 0.15,
        "iou": 0.50,
        "dwell_seconds": 2.0,
        "cooldown_seconds": 300.0,
        "detection_ttl": 1.0,
        "track_iou_threshold": 0.10,
        "track_center_distance_ratio": 0.85,
        "track_max_missed": 12,
        "track_max_age_seconds": 30.0,
        "scene_match_enabled": True,
        "scene_match_min_score": 0.60,
        "scene_match_min_margin": 0.08,
        "scene_match_hold_seconds": 5.0,
    }
    defaults.update(config.get("runtime", {}) or {})
    return defaults


def camera_config(config: dict[str, Any]) -> dict[str, Any]:
    defaults = {
        "id": "front",
        "topic": "/image_combine_jpeg",
        "view": "bottom",
        "rotation": "ccw90",
    }
    defaults.update(config.get("camera", {}) or {})
    return defaults


def roi_configs(config: dict[str, Any], point_id: str = DEFAULT_POINT_ID) -> list[RoiConfig]:
    runtime = runtime_config(config)
    point = site_point(config, point_id)
    default_location = str(point.get("location") or point.get("name") or DEFAULT_LOCATION)
    rois = []
    for item in point.get("rois", []) or []:
        roi_id = str(item.get("id") or "").strip()
        if not roi_id:
            continue
        polygon = normalize_polygon(item.get("polygon"))
        rois.append(
            RoiConfig(
                id=roi_id,
                name=str(item.get("name") or item.get("location") or default_location or roi_id),
                polygon=polygon,
                dwell_seconds=float(item.get("dwell_seconds", runtime["dwell_seconds"])),
                cooldown_seconds=float(item.get("cooldown_seconds", runtime["cooldown_seconds"])),
                scene_id=str(item.get("scene_id") or roi_id),
                scene_anchor=item.get("scene_anchor") if isinstance(item.get("scene_anchor"), dict) else None,
                scene_anchors=item.get("scene_anchors") if isinstance(item.get("scene_anchors"), list) else None,
            )
        )
    return rois


def point_location(config: dict[str, Any], point_id: str = DEFAULT_POINT_ID) -> str:
    point = site_point(config, point_id)
    return str(point.get("location") or point.get("name") or DEFAULT_LOCATION)
