"""Select a source-bound operating area without clipping or qualifying geometry."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from .collision_candidate import FRAME, SOURCE_SCHEMA


def _intersects(low, high, minimum, maximum):
    return bool(np.all(np.asarray(high) >= minimum) and np.all(np.asarray(low) <= maximum))


def _visual_bounds(component):
    low = np.asarray(component["visual_bounds_min_cm"], dtype=float)
    high = np.asarray(component["visual_bounds_max_cm"], dtype=float)
    return np.array([low[0], -high[1], low[2]]) * 0.01, np.array([high[0], -low[1], high[2]]) * 0.01


def review_operating_area(source: dict, candidate: dict, *, minimum_m, maximum_m) -> dict:
    """Make explicit whole-component dispositions, not a map or motion permit.

    Visual bounds are used only to flag unsupported components needing review.
    Every retained collider remains the original, full source geometry.
    """
    minimum, maximum = np.asarray(minimum_m, dtype=float), np.asarray(maximum_m, dtype=float)
    if (
        minimum.shape != (3,)
        or maximum.shape != (3,)
        or not np.isfinite([minimum, maximum]).all()
        or np.any(minimum >= maximum)
    ):
        raise ValueError("operating area requires finite increasing XYZ bounds")
    if source["schema"] != SOURCE_SCHEMA or candidate["frame"] != FRAME:
        raise ValueError("source and candidate coordinate contracts differ")
    originals = {c["source_id"]: c for actor in source["actors"] for c in actor["components"]}
    geometries = {g["source_id"]: g for g in candidate["geometries"]}
    retained, component_ids, dispositions, blockers = [], [], {}, []
    for component in candidate["components"]:
        cid = component["source_id"]
        original = originals[cid]
        geoms = [geometries[gid] for gid in component["geometry_ids"]]
        if geoms:
            if any(
                _intersects(g["collision_bounds"]["min_m"], g["collision_bounds"]["max_m"], minimum, maximum)
                for g in geoms
            ):
                retained.extend(geoms)
                component_ids.append(cid)
                decision = "retain_complete_source_component"
            else:
                decision = "outside_operating_area"
        elif original.get("class") == "SplineComponent":
            decision = "authoring_curve_not_render_geometry"
        elif original["kind"] not in {"static_mesh", "skeletal_mesh"}:
            decision = "non_mesh_component_not_physical_geometry"
        else:
            low, high = _visual_bounds(original)
            if _intersects(low, high, minimum, maximum):
                decision = "blocked_missing_source_collision"
                blockers.append(dict(source_id=cid, reason=decision, kind=original["kind"]))
            else:
                decision = "outside_operating_area"
        dispositions[cid] = dict(decision=decision, geometry_count=len(geoms), source_class=original.get("class"))
    if not retained:
        raise ValueError("operating area selects no actual collision geometry")
    bounds = dict(
        min_m=np.min([g["collision_bounds"]["min_m"] for g in retained], axis=0).tolist(),
        max_m=np.max([g["collision_bounds"]["max_m"] for g in retained], axis=0).tolist(),
    )
    return dict(
        schema="lingtu.factory-operating-area-review.v1",
        frame=FRAME,
        operating_bounds_m=dict(min_m=minimum.tolist(), max_m=maximum.tolist()),
        operating_bounds_role="review_and_measurement_scope_not_collision_geometry_or_motion_authority",
        component_ids=component_ids,
        geometry_ids=[g["source_id"] for g in retained],
        retained_geometry_bounds_m=bounds,
        geometry_clipped=False,
        dispositions=dispositions,
        blockers=blockers,
        qualified_world=False,
        map_created=False,
        spawn_approved=False,
    )


def materialize_selection(world_xml: str, candidate: dict, selection: dict) -> tuple[str, dict]:
    """Keep complete selected components with original mesh/geom attributes."""
    if selection["blockers"]:
        raise ValueError("operating area has unresolved source geometry")
    component_ids = set(selection["component_ids"])
    geoms = [g for g in candidate["geometries"] if g["component_source_id"] in component_ids]
    if {g["source_id"] for g in geoms} != set(selection["geometry_ids"]):
        raise ValueError("selection cannot cut or replace a source component's geometry")
    names = {g["mjcf_geom"] for g in geoms}
    root = ET.fromstring(world_xml)
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("source world has no worldbody")
    found = {g.get("name") for g in worldbody.iter("geom")}
    if not names.issubset(found):
        raise ValueError("source world and geometry names differ")
    for parent in list(worldbody.iter()):
        for child in list(parent):
            if child.tag == "geom" and child.get("name") not in names:
                parent.remove(child)
    mesh_names = {g.get("mesh") for g in worldbody.iter("geom")}
    asset = root.find("asset")
    if asset is not None:
        for child in list(asset):
            if child.tag == "mesh" and child.get("name") not in mesh_names:
                asset.remove(child)
    subset = copy.deepcopy(candidate)
    subset["components"] = [c for c in subset["components"] if c["source_id"] in component_ids]
    subset["geometries"] = copy.deepcopy(geoms)
    subset["source_summary"] = subset.pop("summary")
    subset["summary"] = {
        "accepted_geometries": len(geoms),
        "components_with_geometry": len(subset["components"]),
        "accepted_unique_source_primitives": len({g["primitive_source_id"] for g in geoms}),
        "source_static_mesh_assets": len({g["asset_source_id"] for g in geoms}),
    }
    subset["collision_bounds"] = copy.deepcopy(selection["retained_geometry_bounds_m"])
    subset["selection_bounds_m"] = copy.deepcopy(selection["operating_bounds_m"])
    subset["operating_area_review"] = copy.deepcopy(selection)
    subset["status"] = "selected_whole_source_components_not_qualified"
    return ET.tostring(root, encoding="unicode"), subset


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--bounds", nargs=6, type=float, required=True, metavar="XYZ_MIN_MAX")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = review_operating_area(
        json.loads(args.source.read_text(encoding="utf-8")),
        json.loads(args.candidate.read_text(encoding="utf-8")),
        minimum_m=args.bounds[:3],
        maximum_m=args.bounds[3:],
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(
        json.dumps(
            dict(
                components=len(result["component_ids"]),
                geometries=len(result["geometry_ids"]),
                blockers=result["blockers"],
                qualified_world=False,
            )
        )
    )


if __name__ == "__main__":
    main()
