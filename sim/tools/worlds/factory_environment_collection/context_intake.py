"""Read original enclosure and lighting omitted by the initial low-height bay cut.

Run only in FactoryPreview as an editor commandlet. Does not save source assets,
copy content, qualify collision, or start gameplay.
"""

import json
import os
import re
import sys
from pathlib import Path

import unreal as ue

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from sim.tools.worlds.factory_environment_collection import bay_export as export

MINIMUM = [5000, -1000, -100]
MAXIMUM = [6600, 1000, 4000]
STRUCTURAL = re.compile(r"roof|ceiling|wall|beam|column|window|girder|pillar|truss", re.I)


def intersects(low, high):
    return all(high[i] >= MINIMUM[i] and low[i] <= MAXIMUM[i] for i in range(3))


def main():
    output = Path(os.environ["LINGTU_FACTORY_CONTEXT_DIR"])
    if export.normalized_path(ue.Paths.get_project_file_path()) != export.normalized_path(export.SOURCE_PROJECT):
        raise RuntimeError("context intake requires the isolated FactoryPreview source project")
    output.mkdir(parents=True, exist_ok=False)
    world = ue.EditorLoadingAndSavingUtils.load_map(export.SOURCE_MAP)
    if world is None:
        raise RuntimeError("source factory level failed to load")
    export.BAY_MIN, export.BAY_MAX = MINIMUM, MAXIMUM
    assets, roots = {}, set()
    budget = {"components": 0, "instances": 0}
    structure, lights = [], []
    actors = ue.get_editor_subsystem(ue.EditorActorSubsystem).get_all_level_actors()
    for actor in actors:
        for component in actor.get_components_by_class(ue.StaticMeshComponent):
            mesh = component.static_mesh
            if mesh is None or not STRUCTURAL.search(actor.get_actor_label() + " " + mesh.get_path_name()):
                continue
            spatial = export.component_spatial(ue, component)
            if intersects(spatial[1], spatial[2]):
                row = export.component_record(ue, component, assets, roots, budget, spatial)
                row["actor_label"] = actor.get_actor_label()
                structure.append(row)
        for component in actor.get_components_by_class(ue.LightComponentBase):
            transform = export.transform_record(component.get_world_transform())
            position = transform["translation_cm"]
            radius = export.property_probe(component, "attenuation_radius", float)
            extent = radius.get("value", 0.0)
            global_light = isinstance(component, (ue.DirectionalLightComponent, ue.SkyLightComponent))
            if not global_light and not intersects([v - extent for v in position], [v + extent for v in position]):
                continue
            row = {
                "source_id": component.get_path_name(),
                "actor_label": actor.get_actor_label(),
                "class": component.get_class().get_name(),
                "world_transform": transform,
                "attenuation_radius_cm": radius,
                "properties": {},
            }
            for name in (
                "intensity",
                "intensity_units",
                "light_color",
                "use_temperature",
                "temperature",
                "cast_shadows",
                "mobility",
                "source_radius",
                "source_length",
                "source_width",
                "source_height",
                "inner_cone_angle",
                "outer_cone_angle",
                "light_source_angle",
                "indirect_lighting_intensity",
                "volumetric_scattering_intensity",
                "source_type",
            ):
                row["properties"][name] = export.property_probe(component, name)
            for name in ("ies_texture", "cubemap"):
                value = export.property_probe(component, name, export.path_or_none)
                row["properties"][name] = value
                if value.get("value"):
                    roots.add(value["value"])
            lights.append(row)
    registry = ue.AssetRegistryHelpers.get_asset_registry()
    dependencies = export.dependency_manifest(
        registry,
        ue.AssetRegistryDependencyOptions(
            include_soft_package_references=True,
            include_hard_package_references=True,
            include_searchable_names=False,
            include_soft_management_references=False,
            include_hard_management_references=False,
        ),
        roots,
    )
    report = {
        "schema": "lingtu.factory-enclosure-light-intake.v1",
        "source_project": export.SOURCE_PROJECT,
        "source_world": world.get_path_name(),
        "bounds_min_cm": MINIMUM,
        "bounds_max_cm": MAXIMUM,
        "bounds_role": "authoring_context_not_motion_permission",
        "source_map_saved": False,
        "assets_copied": False,
        "qualified_world": False,
        "structural_components": structure,
        "lights": lights,
        "mesh_assets": assets,
        "dependencies": dependencies,
    }
    (output / "context.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    ue.log(f"LINGTU_FACTORY_CONTEXT_READ structures={len(structure)} lights={len(lights)} saved=false")


if __name__ == "__main__":
    main()
