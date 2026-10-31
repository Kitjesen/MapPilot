#!/usr/bin/env python3
"""Save and reload ReplicaCAD instance records without granting motion."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def check(graph_path: Path, output: Path) -> dict[str, Any]:
    graph = json.loads(graph_path.read_text(encoding="utf-8"))
    records = graph.get("instance_records", [])
    if not isinstance(records, list) or not records:
        raise ValueError("graph has no instance_records")
    saved = output.with_name(output.stem + ".instances.json")
    saved.parent.mkdir(parents=True, exist_ok=True)
    saved.write_text(json.dumps(records, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    reloaded = json.loads(saved.read_text(encoding="utf-8"))
    source_ids = [str(record.get("instance_id", "")) for record in records]
    reload_ids = [str(record.get("instance_id", "")) for record in reloaded]
    stable = source_ids == reload_ids and all(
        left.get("geometry_ref") == right.get("geometry_ref")
        and left.get("map_identity") == right.get("map_identity")
        for left, right in zip(records, reloaded)
    )
    report = {
        "schema_version": "lingtu.semantic_instance_persistence.v1",
        "graph": graph_path.as_posix(),
        "saved_records": saved.as_posix(),
        "map_identity": graph.get("map_id"),
        "record_count": len(records),
        "reloaded_count": len(reloaded),
        "stable_instance_ids": stable,
        "geometry_refs_reloaded": stable,
        "persistence_status": "dataset_artifact_only",
        "motion_authorization": False,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("graph", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(check(args.graph, args.output), ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
