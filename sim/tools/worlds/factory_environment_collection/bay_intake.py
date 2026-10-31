"""Read-only inventory of the selected licensed factory bay.

Run this file in the isolated FactoryPreview Python commandlet with
LINGTU_FACTORY_BAY_INTAKE_DIR set. Bounds are visual AABBs in Unreal cm;
they are NOT collision proxies, navigation clearance, or inspection poses.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any


def select_bay(
    actors: list[dict[str, Any]], *, minimum_cm: list[float], maximum_cm: list[float]
) -> list[dict[str, Any]]:
    """Keep intersecting actors and expose geometry that cannot be copied whole."""
    selected = []
    for actor in actors:
        low, high = actor["bounds_min_cm"], actor["bounds_max_cm"]
        if all(high[i] >= minimum_cm[i] and low[i] <= maximum_cm[i] for i in range(3)):
            selected.append({
                **actor,
                "crosses_bay_boundary": any(
                    low[i] < minimum_cm[i] or high[i] > maximum_cm[i] for i in range(3)
                ),
            })
    return selected


def main() -> None:
    import unreal

    output = Path(os.environ["LINGTU_FACTORY_BAY_INTAKE_DIR"])
    output.mkdir(parents=True, exist_ok=True)
    if Path(unreal.Paths.get_project_file_path()).name != "FactoryProject.uproject":
        raise RuntimeError("intake must run in the isolated FactoryPreview project")
    world = unreal.EditorLoadingAndSavingUtils.load_map("/Game/Maps/Demonstration")
    if world is None:
        raise RuntimeError("factory Demonstration world did not load")
    subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actors = []
    for actor in subsystem.get_all_level_actors():
        components = actor.get_components_by_class(unreal.StaticMeshComponent)
        meshes = sorted({
            component.get_editor_property("static_mesh").get_path_name()
            for component in components
            if component.get_editor_property("static_mesh") is not None
        })
        if not meshes:
            continue
        center, extent = actor.get_actor_bounds(False)
        location = actor.get_actor_location()
        rotation = actor.get_actor_rotation()
        scale = actor.get_actor_scale3d()
        actors.append({
            "name": actor.get_name(), "label": actor.get_actor_label(),
            "source_actor": actor.get_path_name(), "class": actor.get_class().get_name(),
            "mesh_assets": meshes,
            "location_cm": [location.x, location.y, location.z],
            "rotation_degrees_pyr": [rotation.pitch, rotation.yaw, rotation.roll],
            "scale": [scale.x, scale.y, scale.z],
            "bounds_min_cm": [center.x - extent.x, center.y - extent.y, center.z - extent.z],
            "bounds_max_cm": [center.x + extent.x, center.y + extent.y, center.z + extent.z],
        })
    # The first authoring region surrounds the reviewed corridor camera.
    # Its edges are intake boundaries, not approved physical barriers.
    minimum, maximum = [3000, -2000, -100], [7000, 1600, 1500]
    selected = select_bay(actors, minimum_cm=minimum, maximum_cm=maximum)
    report = {
        "purpose": "source_bay_inventory_not_collision_or_runnable_world",
        "source_project": unreal.Paths.get_project_file_path(),
        "source_world": world.get_path_name(), "source_map_saved": False,
        "units": "Unreal_cm", "bounds_min_cm": minimum, "bounds_max_cm": maximum,
        "static_mesh_actor_count": len(actors), "selected_actor_count": len(selected),
        "crossing_actor_count": sum(actor["crosses_bay_boundary"] for actor in selected),
        "mesh_assets": sorted({mesh for actor in selected for mesh in actor["mesh_assets"]}),
        "actors": sorted(selected, key=lambda actor: actor["name"]),
    }
    with (output / "bay-intake.json").open("x", encoding="utf-8") as file:
        json.dump(report, file, ensure_ascii=False, indent=2)
    unreal.log(f"LINGTU_FACTORY_BAY_INTAKE_COMPLETE actors={len(selected)} saved_source=false")


if __name__ == "__main__":
    main()
