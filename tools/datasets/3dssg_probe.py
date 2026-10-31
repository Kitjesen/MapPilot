#!/usr/bin/env python3
"""Convert one 3DSSG scene into a compact LingTu semantic scene graph.

3DSSG supplies object labels and relation triplets from indoor reconstructions.
It does not include the reconstructed point clouds in this download, so the
output keeps geometry binding explicit instead of inventing object poses.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _load_scans(path: Path) -> list[dict[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    scans = payload.get("scans")
    if not isinstance(scans, list):
        raise ValueError(f"{path} does not contain a scans list")
    return scans


def _select_scan(
    objects_scans: list[dict[str, Any]],
    relationship_scans: dict[str, dict[str, Any]],
    scan_id: str | None,
) -> dict[str, Any]:
    if scan_id:
        for scan in objects_scans:
            if scan.get("scan") == scan_id:
                return scan
        raise ValueError(f"3DSSG scan not found: {scan_id}")
    for scan in objects_scans:
        labels = {str(obj.get("label", "")) for obj in scan.get("objects", [])}
        if not {"table", "box"}.issubset(labels):
            continue
        by_id = {str(obj.get("id")): str(obj.get("label", "")) for obj in scan.get("objects", [])}
        relations = relationship_scans.get(str(scan.get("scan")), {}).get("relationships", [])
        if any(
            isinstance(relation, list)
            and len(relation) >= 4
            and str(relation[3]) in {"close by", "near"}
            and {by_id.get(str(relation[0])), by_id.get(str(relation[1]))} == {"table", "box"}
            for relation in relations
        ):
            return scan
    for scan in objects_scans:
        labels = {str(obj.get("label", "")) for obj in scan.get("objects", [])}
        if {"table", "box"}.issubset(labels):
            return scan
    if not objects_scans:
        raise ValueError("3DSSG objects file contains no scans")
    return objects_scans[0]


def convert_scene(dataset_dir: Path, scan_id: str | None = None) -> dict[str, Any]:
    objects_scans = _load_scans(dataset_dir / "objects.json")
    relationship_scans = {
        str(scan.get("scan")): scan
        for scan in _load_scans(dataset_dir / "relationships.json")
    }
    selected = _select_scan(objects_scans, relationship_scans, scan_id)
    selected_id = str(selected["scan"])
    relation_scan = relationship_scans.get(selected_id, {})

    nodes: list[dict[str, Any]] = []
    object_ids: set[str] = set()
    for item in selected.get("objects", []):
        object_id = str(item.get("id"))
        object_ids.add(object_id)
        nodes.append(
            {
                "id": f"object:{selected_id}:{object_id}",
                "kind": "object",
                "label": str(item.get("label", "object")),
                "ontology": {
                    "nyu40": item.get("nyu40"),
                    "rio27": item.get("rio27"),
                    "eigen13": item.get("eigen13"),
                    "global_id": item.get("global_id"),
                },
                "affordances": item.get("affordances", [])
                + item.get("state_affordances", []),
                "attributes": item.get("attributes", {}),
                "geometry_binding": {
                    "status": "unavailable",
                    "reason": "3DSSG annotation-only download; 3RScan reconstruction is separate",
                },
            }
        )

    edges: list[dict[str, Any]] = []
    skipped_relations = 0
    for relation in relation_scan.get("relationships", []):
        if not isinstance(relation, list) or len(relation) < 4:
            skipped_relations += 1
            continue
        subject, target, predicate_id, predicate = relation[:4]
        subject_id = str(subject)
        target_id = str(target)
        if subject_id not in object_ids or target_id not in object_ids:
            skipped_relations += 1
            continue
        edges.append(
            {
                "subject": f"object:{selected_id}:{subject_id}",
                "predicate": str(predicate),
                "predicate_id": predicate_id,
                "object": f"object:{selected_id}:{target_id}",
            }
        )

    return {
        "schema_version": "lingtu.semantic_scene_graph.v1",
        "source": {
            "dataset": "3DSSG",
            "scan": selected_id,
            "objects_file": (dataset_dir / "objects.json").as_posix(),
            "relationships_file": (dataset_dir / "relationships.json").as_posix(),
        },
        "nodes": nodes,
        "relations": edges,
        "geometry_binding": {
            "status": "unavailable",
            "required_inputs": ["3RScan reconstruction", "camera poses", "point-to-instance association"],
            "motion_authorization": False,
        },
        "stats": {
            "node_count": len(nodes),
            "relation_count": len(edges),
            "skipped_relations": skipped_relations,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset_dir", type=Path, help="directory containing objects.json and relationships.json")
    parser.add_argument("--scan", dest="scan_id", help="3DSSG scan UUID; defaults to a table+box scene")
    parser.add_argument("--output", type=Path, required=True, help="output semantic scene graph JSON")
    args = parser.parse_args()
    result = convert_scene(args.dataset_dir, args.scan_id)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": args.output.as_posix(), "scan": result["source"]["scan"], **result["stats"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
