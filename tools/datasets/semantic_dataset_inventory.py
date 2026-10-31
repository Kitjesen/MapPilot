#!/usr/bin/env python3
"""Report the local evidence available for semantic-map development."""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path
from typing import Any


def _pcd_metadata(path: Path) -> dict[str, Any]:
    header = path.read_bytes().split(b"DATA", 1)[0].decode("ascii", errors="ignore")
    fields_match = re.search(r"^FIELDS (.+)$", header, re.MULTILINE)
    points_match = re.search(r"^POINTS (\d+)$", header, re.MULTILINE)
    return {
        "path": path.as_posix(),
        "bytes": path.stat().st_size,
        "fields": fields_match.group(1).split() if fields_match else [],
        "points": int(points_match.group(1)) if points_match else None,
    }


def build_inventory(repo_root: Path) -> dict[str, Any]:
    data_root = repo_root / "sim" / "evaluation" / "data"
    files = [path for path in data_root.rglob("*") if path.is_file()]
    extensions = Counter(path.suffix.lower() or "<none>" for path in files)
    pcds = [_pcd_metadata(path) for path in sorted(data_root.rglob("*.pcd"))]
    semantic_entities = repo_root / "sim" / "packages" / "worlds" / "factory_park_hf" / "generated" / "semantic-entities.json"
    semantic_entity_summary: dict[str, Any] | None = None
    if semantic_entities.is_file():
        document = json.loads(semantic_entities.read_text(encoding="utf-8"))
        semantic_entity_summary = {
            "path": semantic_entities.relative_to(repo_root).as_posix(),
            "schema": document.get("schema"),
            "entity_count": len(document.get("entities", [])),
            "route_count": len(document.get("routes", [])),
            "coordinate_frame": document.get("coordinate_frame"),
        }

    rgb_extensions = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}
    rgbd_files = [path for path in files if path.suffix.lower() in rgb_extensions]
    return {
        "schema_version": "lingtu.semantic_dataset_inventory.v1",
        "data_root": data_root.relative_to(repo_root).as_posix(),
        "local_file_counts": dict(sorted(extensions.items())),
        "pcd_geometry": {
            "status": "ready",
            "modalities": ["lidar_geometry"],
            "files": pcds,
            "object_semantics": False,
        },
        "lidar_imu_replay": {
            "status": "present_for_replay",
            "modalities": ["lidar", "imu"],
            "bag_count": extensions.get(".bag", 0),
            "ros2_db3_count": extensions.get(".db3", 0),
            "camera_frames": 0,
            "object_semantics": False,
        },
        "rgbd_frames_in_evaluation_data": {
            "status": "missing" if not rgbd_files else "present",
            "files": [path.relative_to(repo_root).as_posix() for path in rgbd_files],
            "object_semantics": bool(rgbd_files),
        },
        "simulation_semantic_ground_truth": semantic_entity_summary,
        "interpretation": {
            "current_proof": "geometry and LiDAR/IMU replay only",
            "missing_proof": "RGB/RGB-D to map-frame object association",
            "next_dataset": "small deterministic simulation recording with RGB, depth, LiDAR, IMU, poses, calibration, and semantic entity truth",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--write", type=Path, help="write the JSON report to this path")
    args = parser.parse_args()
    report = build_inventory(args.repo_root.resolve())
    encoded = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.write:
        args.write.parent.mkdir(parents=True, exist_ok=True)
        args.write.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
