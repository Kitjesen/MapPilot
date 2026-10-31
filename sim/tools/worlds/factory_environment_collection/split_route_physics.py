"""Split blocked source components from an expanded factory route candidate.

The split preserves complete source components and existing collision geometry.
It creates an unqualified physics candidate only; it does not publish a World,
approve a spawn, or invent collision for blocked visual components.
"""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from typing import Any

from .bay_layout import materialize_selection


def split_route_physics(
    candidate: dict[str, Any], selection: dict[str, Any], source_xml: str
) -> tuple[str, dict[str, Any], dict[str, Any]]:
    """Return filtered MJCF, manifest, and an auditable split report."""

    if candidate.get("schema") != "lingtu.factory-collision-candidate.v1":
        raise ValueError("route physics split requires the canonical collision candidate")
    if selection.get("schema") != "lingtu.factory-operating-area-review.v1":
        raise ValueError("route physics split requires an operating-area review")
    blocked = {row["source_id"] for row in selection.get("blockers", [])}
    component_ids = set(selection.get("component_ids", []))
    # bay_layout deliberately keeps blocked no-geometry components out of
    # component_ids while retaining them in blockers for the visual review.
    # Treat both representations as the same explicit exclusion.
    retained_ids = component_ids - blocked
    geometries = [
        row for row in candidate.get("geometries", []) if row.get("component_source_id") in retained_ids
    ]
    if not retained_ids or not geometries:
        raise ValueError("route physics split retained no source collision geometry")

    split_selection = copy.deepcopy(selection)
    split_selection["component_ids"] = sorted(retained_ids)
    split_selection["geometry_ids"] = [row["source_id"] for row in geometries]
    split_selection["blockers"] = []
    split_selection["status"] = "route_physics_candidate_blocked_visual_components_excluded"
    xml, manifest = materialize_selection(source_xml, candidate, split_selection)
    manifest["route_physics_split"] = {
        "status": "candidate_unqualified",
        "excluded_blocked_component_ids": sorted(blocked),
        "retained_component_count": len(retained_ids),
        "retained_geometry_count": len(geometries),
        "visual_components_require_separate_no_collision_review": sorted(blocked),
    }
    report = {
        "schema": "lingtu.factory-route-physics-split.v1",
        "status": "candidate_unqualified",
        "source_selection": "build/inspection-tests/20261024/factory-route-layout-01/selection.json",
        "candidate_source": "build/inspection-tests/20261022/factory-collision-candidate-02/manifest.json",
        "retained_components": len(retained_ids),
        "retained_geometries": len(geometries),
        "excluded_blocked_components": sorted(blocked),
        "collision_geometry_policy": "complete_existing_source_components_only",
        "qualified_world": False,
        "spawn_approved": False,
        "map_created": False,
        "route_created": False,
    }
    return xml, manifest, report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--selection", type=Path, required=True)
    parser.add_argument("--source-xml", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"route physics output already exists: {args.output}")
    args.output.mkdir(parents=True)
    candidate = json.loads(args.candidate.read_text(encoding="utf-8"))
    selection = json.loads(args.selection.read_text(encoding="utf-8"))
    xml, manifest, report = split_route_physics(
        candidate, selection, args.source_xml.read_text(encoding="utf-8")
    )
    (args.output / "route-physics-world.xml").write_text(xml, encoding="utf-8")
    (args.output / "route-physics-manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    (args.output / "split-report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(report))


if __name__ == "__main__":
    main()
