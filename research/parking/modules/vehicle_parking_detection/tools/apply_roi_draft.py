from __future__ import annotations

import argparse
import copy
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

MODULES_ROOT = Path(__file__).resolve().parents[2]
if str(MODULES_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULES_ROOT))

from vehicle_parking_detection.config import DEFAULT_POINT_ID, dump_yaml, load_yaml, normalize_polygon


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SITE_CONFIG = PROJECT_ROOT / "modules" / "vehicle_parking_detection" / "configs" / "site_rois.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Apply reviewed no-parking ROI draft to site_rois.yaml.")
    parser.add_argument("--draft", required=True)
    parser.add_argument("--site-config", default=str(DEFAULT_SITE_CONFIG))
    parser.add_argument("--write", action="store_true")
    return parser.parse_args()


def find_or_create_point(site_config: dict[str, Any], point_id: str, location: str) -> dict[str, Any]:
    points = site_config.setdefault("points", [])
    for point in points:
        if str(point.get("point_id") or point.get("patrol_point_id") or "") == point_id:
            return point
    point = {"point_id": point_id, "location": location, "enabled": True, "rois": []}
    points.append(point)
    return point


def apply_draft(site_config: dict[str, Any], draft: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    updated = copy.deepcopy(site_config)
    point_id = str(draft.get("point_id") or DEFAULT_POINT_ID)
    location = str(draft.get("location") or "禁停区01")
    point = find_or_create_point(updated, point_id, location)
    point["location"] = location
    point_rois = point.setdefault("rois", [])
    by_id = {str(roi.get("id")): roi for roi in point_rois if roi.get("id")}
    changes = []

    for item in draft.get("rois", []) or []:
        roi_id = str(item.get("id") or "").strip()
        if not roi_id:
            continue
        polygon = normalize_polygon(item.get("polygon"))
        if not polygon:
            continue
        roi = by_id.get(roi_id)
        if roi is None:
            roi = {"id": roi_id}
            point_rois.append(roi)
        roi["name"] = str(item.get("name") or location)
        roi["polygon"] = [[int(x), int(y)] for x, y in polygon]
        roi["dwell_seconds"] = float(item.get("dwell_seconds", 3))
        roi["cooldown_seconds"] = float(item.get("cooldown_seconds", 300))
        for key in (
            "scene_id",
            "anchor_frame_index",
            "anchor_second",
            "anchor_keyframe_image",
            "scene_anchor",
            "scene_anchors",
            "scene_anchor_count",
        ):
            if key in item:
                roi[key] = copy.deepcopy(item[key])
        changes.append(f"{point_id}/{roi_id}: {len(polygon)} points")

    if changes:
        point["roi_last_updated"] = datetime.now().isoformat(timespec="seconds")
        point["roi_source_run_dir"] = draft.get("source_run_dir", "")
    return updated, changes


def main() -> None:
    args = parse_args()
    draft_path = Path(args.draft)
    site_path = Path(args.site_config)
    if not draft_path.exists():
        raise SystemExit(f"draft not found: {draft_path}")
    if not site_path.exists():
        raise SystemExit(f"site config not found: {site_path}")
    updated, changes = apply_draft(load_yaml(site_path), load_yaml(draft_path))
    if not changes:
        print("No non-empty ROI polygons to apply.")
        return
    print("Planned changes:")
    for change in changes:
        print(f"  - {change}")
    if not args.write:
        print("Dry-run only. Re-run with --write to update site_rois.yaml.")
        return
    backup = site_path.with_name(site_path.name + f".bak_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    backup.write_text(site_path.read_text(encoding="utf-8"), encoding="utf-8")
    dump_yaml(site_path, updated)
    print(f"Updated: {site_path.resolve()}")
    print(f"Backup: {backup.resolve()}")


if __name__ == "__main__":
    main()
