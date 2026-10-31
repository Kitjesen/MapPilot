"""Prepare a FactoryPark route for the existing public inspection API.

This offline authoring helper never starts a Product or sends motion commands.
The native planner still owns the executable path between these destinations.
"""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Mapping
from pathlib import Path
from typing import Any

CHECKPOINT_IDS = ("checkpoint_gate", "checkpoint_warehouse", "checkpoint_tank_farm")
LAYOUT_PATH = Path(__file__).resolve().parents[4] / "sim/packages/worlds/factory_park_hf/generated/expanded-layout.json"


def create_inspection_route(
    layout: Mapping[str, Any], *, map_id: str, map_content_epoch: int, revision: int
) -> dict[str, Any]:
    """Bind existing destinations to an explicitly selected saved-map version."""
    if not map_id.strip() or type(map_content_epoch) is not int or map_content_epoch <= 0:
        raise ValueError("a saved map_id and positive map_content_epoch are required")
    if type(revision) is not int or revision <= 0:
        raise ValueError("a positive route revision is required")
    checkpoints = {point["id"]: point for point in layout["checkpoints"]}
    points = []
    for point_id in CHECKPOINT_IDS:
        if point_id not in checkpoints:
            raise ValueError(f"FactoryPark checkpoint is missing: {point_id}")
        position = checkpoints[point_id]["position_m"]
        if len(position) != 3 or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in position):
            raise ValueError(f"FactoryPark checkpoint has invalid coordinates: {point_id}")
        points.append({
            "id": point_id, "x": position[0], "y": position[1], "z": position[2],
            "yaw": None, "tolerance": .35, "dwell": 1.0,
            "action": "capture:overview", "enabled": True,
        })
    return {
        "id": "factory_park_inspection", "name": "FactoryPark 厂区巡检",
        "map_id": map_id.strip(), "map_content_epoch": map_content_epoch, "revision": revision,
        "loop_count": 1, "failure_policy": "stop", "max_retries": 0, "points": points,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--map-content-epoch", type=int, required=True)
    parser.add_argument("--revision", type=int, required=True)
    args = parser.parse_args()
    layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
    print(json.dumps(create_inspection_route(layout, map_id=args.map_id,
        map_content_epoch=args.map_content_epoch, revision=args.revision), ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
