"""Same-source static initial-pose authoring, not a runtime world importer.

Offline CLI writes a NEW plan only; it never launches UE or copies assets.
After separately approved migration, call ``author_editor_level(plan)`` inside
an owned RobotSimUE Editor. Never run that entry in the source project.
"""

from __future__ import annotations

import argparse
import filecmp
import json
from copy import deepcopy
from math import isclose
from pathlib import Path

LEVEL_PATH = "/Game/RobotSim/Worlds/FactoryInspectionBay/FactoryInspectionBay_StaticInitialPose"

API_EVIDENCE = {
    "assettools": "Source/Developer/AssetTools/Public/IAssetTools.h:633-637; Python MigratePackages overload has no exact dependency-set argument; not used",
    "level": "Source/Editor/LevelEditor/Public/LevelEditorSubsystem.h:147,174 NewLevel/SaveCurrentLevel",
    "spawn": "Source/Editor/UnrealEd/Public/Subsystems/EditorActorSubsystem.h:228 SpawnActorFromClass",
    "transform": "Source/Runtime/Engine/Classes/GameFramework/Actor.h:1799 K2_SetActorTransform ScriptName=SetActorTransform",
    "mesh": "Source/Runtime/Engine/Classes/Engine/StaticMeshActor.h:25-26 StaticMeshComponent UPROPERTY",
    "physics": "Source/Runtime/Engine/Classes/Components/PrimitiveComponent.h:1600,1661,2026,2036 SetMaterial/SetSimulatePhysics/SetCollisionEnabled/SetCollisionProfileName",
    "scope": "API references only; execution outcome belongs to the run-local saved-level readback",
}


def build_visual_manifest(source, candidate, component_ids=None):
    """Pure stable-ID projection of components with retained collision geometry."""
    import numpy as np

    from .collision_candidate import FRAME, SOURCE_FRAME, SOURCE_SCHEMA, UE_TO_M, transform_matrix

    if (
        source.get("schema") != SOURCE_SCHEMA
        or source.get("frame") != SOURCE_FRAME
        or source.get("units") != "Unreal_cm"
        or source.get("source_inventory_matches") is not True
        or candidate.get("schema") != "lingtu.factory-collision-candidate.v1"
        or any(source[k] != candidate[k] for k in ("source_project", "source_world"))
    ):
        raise ValueError("source/candidate provenance or frame mismatch")
    components = {c["source_id"]: (a, c) for a in source["actors"] for c in a["components"]}
    assets = {a["source_id"] for a in source["mesh_assets"] if a["kind"] == "static_mesh"}
    geometries = {g["source_id"]: g for g in candidate["geometries"]}
    used = []
    rows = []
    for selected in candidate["components"]:
        if not selected["geometry_ids"]:
            continue
        cid = selected["source_id"]
        if cid not in components:
            raise ValueError(f"missing source component: {cid}")
        actor, component = components[cid]
        if (
            component["kind"] != "static_mesh"
            or component.get("class") != "StaticMeshComponent"
            or selected["kind"] != "static_mesh"
            or component["mesh_asset"] not in assets
            or component["mesh_asset"] != selected["mesh_asset"]
            or actor["source_id"] != selected["actor_source_id"]
            or component["world_transform"] != selected["source_world_transform"]
        ):
            raise ValueError(f"source component/asset/transform mismatch: {cid}")
        for gid in selected["geometry_ids"]:
            geom = geometries.get(gid)
            if (
                geom is None
                or geom["component_source_id"] != cid
                or geom["asset_source_id"] != component["mesh_asset"]
                or geom["actor_source_id"] != actor["source_id"]
            ):
                raise ValueError(f"source geometry association mismatch: {gid}")
            used.append(gid)
        transform = component["world_transform"]
        matrix = UE_TO_M @ transform_matrix(transform)
        if not np.allclose(matrix, selected["canonical_from_asset_cm"], atol=1e-10, rtol=0):
            raise ValueError(f"canonical transform mismatch: {cid}")
        x, y, z = transform["translation_cm"]
        qx, qy, qz, qw = transform["rotation_xyzw"]
        rows.append(
            {
                "source_id": component["source_id"],
                "actor_source_id": actor["source_id"],
                "mesh_asset": component["mesh_asset"],
                "materials": deepcopy(component["materials"]),
                "ue_world_transform": deepcopy(transform),
                "canonical_transform": {
                    "translation_m": [0.01 * x, -0.01 * y, 0.01 * z],
                    "rotation_xyzw": [-qx, qy, -qz, qw],
                    "scale3d": list(transform["scale3d"]),
                },
                "canonical_from_asset_cm": matrix.tolist(),
                "geometry_ids": list(selected["geometry_ids"]),
                "visible": component["visible"]["value"],
                "collision_profile": "NoCollision",
                "collision_enabled": "NO_COLLISION",
                "simulate_physics": False,
            }
        )
    if len(used) != len(set(used)) or set(used) != set(geometries):
        raise ValueError("duplicate or orphan candidate geometry")
    retained = {r["source_id"] for r in rows}
    chosen = retained if component_ids is None else set(component_ids)
    if not chosen.issubset(retained):
        raise ValueError(f"selection must contain retained static component IDs: {sorted(chosen - retained)}")
    excluded = [r for r in rows if r["source_id"] not in chosen]
    rows = [r for r in rows if r["source_id"] in chosen]
    return {
        "schema": "lingtu.factory-static-visual-authoring.v1",
        "status": "planned_static_initial_pose_not_authored_not_qualified",
        "source_project": source["source_project"],
        "source_world": source["source_world"],
        "level_path": LEVEL_PATH,
        "frame": FRAME,
        "origin_shift_m": [0, 0, 0],
        "visual_aabb_used": False,
        "limitations": [
            "Static initial pose only; no animation or Blueprint gameplay reconstruction",
            "Only retained-geometry components; missing geometry is not completed",
            "Skeletal and deformed/instanced meshes, lights and VFX are not reconstructed",
            "SplineComponent is an editor curve helper, not missing deformed collision",
            "No WorldPackage publication, navigation, physics, RGB or inspection qualification",
        ],
        "components": rows,
        "selection": "all_retained_static" if component_ids is None else "explicit_review_component_ids",
        "excluded_geometry_ids": [g for r in excluded for g in r["geometry_ids"]],
        "omitted_components": [
            {
                "source_id": c["source_id"],
                "reason": (
                    "editor_curve_helper_not_deformed_mesh"
                    if components[c["source_id"]][1].get("class") == "SplineComponent"
                    else "no retained candidate geometry"
                ),
            }
            for c in candidate["components"]
            if not c["geometry_ids"]
        ]
        + [{"source_id": r["source_id"], "reason": "outside explicit component selection"} for r in excluded],
    }


def dependency_closure(source, manifest):
    """Follow the export's package records, not its broader root inventory."""
    records = {r["package"]: r for r in source["dependencies"]["packages"]}
    pending = {
        ref.split(".", 1)[0] for c in manifest["components"] for ref in [c["mesh_asset"], *c["materials"]] if ref
    }
    result = {}
    while pending:
        package = pending.pop()
        if package in result:
            continue
        row = records.get(package)
        if row is None:
            raise ValueError(f"missing dependency record: {package}")
        if package.startswith("/Script/") and row["disposition"] == "native_type_reference_not_asset_copy":
            result[package] = deepcopy(row)
            continue
        if (
            row["disposition"] != "inventory_only_no_copy"
            or row.get("dependencies", {}).get("status") != "available"
            or not package.startswith(("/Game/", "/Engine/"))
        ):
            raise ValueError(f"unresolved/excluded dependency required by selection: {package}")
        result[package] = deepcopy(row)
        pending.update(row["dependencies"]["value"])
    return [result[p] for p in sorted(result)]


def asset_file_plan(manifest, source_content, target_content, engine_content, *, reuse_identical=False):
    """Read-only exact file inventory; references and relative paths never change."""
    source_content, target_content, engine_content = (
        Path(p).resolve() for p in (source_content, target_content, engine_content)
    )
    if source_content == target_content:
        raise ValueError("source and target Content must differ")
    files, external = [], []
    for row in manifest["dependencies"]:
        package = row["package"]
        if package.startswith("/Script/"):
            external.append({"package": package, "action": "native_reference_no_copy"})
            continue
        game = package.startswith("/Game/")
        relative = Path(package.split("/", 2)[2])
        base = (source_content if game else engine_content) / relative
        main = base.with_suffix(".uasset")
        if not main.is_file():
            raise FileNotFoundError(f"real asset required; no Cube fallback: {main}")
        actual = sorted(
            p
            for p in base.parent.glob(base.name + ".*")
            if p.is_file() and p.name[len(base.name) :] in {".uasset", ".uexp", ".ubulk", ".uptnl", ".m.ubulk"}
        )
        if not game:
            external.append(
                {
                    "package": package,
                    "action": "installed_engine_reference_no_copy",
                    "files": [{"source": str(p), "bytes": p.stat().st_size} for p in actual],
                }
            )
            continue
        target_base = target_content / relative
        existing = list(target_base.parent.glob(target_base.name + ".*"))
        if existing and (
            not reuse_identical
            or {p.name for p in existing} != {p.name for p in actual}
            or any(not filecmp.cmp(p, target_base.parent / p.name, shallow=False) for p in actual)
        ):
            raise FileExistsError(f"target package conflict; no overwrite: {target_base}")
        for file in actual:
            files.append(
                {
                    "package": package,
                    "source": str(file),
                    "target": str(target_content / file.relative_to(source_content)),
                    "bytes": file.stat().st_size,
                    "action": "reuse_verified_identical" if existing else "copy_new",
                }
            )
    level = target_content / (manifest["level_path"].removeprefix("/Game/") + ".umap")
    if level.exists() or level.with_suffix(".uasset").exists():
        raise FileExistsError(f"target level conflict: {level}")
    return {
        "files": files,
        "copy_bytes": sum(f["bytes"] for f in files if f["action"] == "copy_new"),
        "copy_packages": len({f["package"] for f in files if f["action"] == "copy_new"}),
        "external_references": external,
        "source_content": str(source_content),
        "target_content": str(target_content),
        "target_level_file": str(level),
        "action": "plan_only_no_assets_copied",
        "size_scope": "actual uncooked package files and sidecars; not Cook/DDC/runtime memory",
    }


def write_plan(candidate_dir, source_content, target_content, engine_content, output_dir, component_ids=None):
    """Write NEW offline artifacts only; never execute the generated copy script."""
    candidate_dir, output_dir = Path(candidate_dir).resolve(), Path(output_dir).resolve()
    if output_dir.exists():
        raise FileExistsError(f"new plan directory required: {output_dir}")
    source = json.loads((candidate_dir / "source-with-collision.json").read_text(encoding="utf-8"))
    candidate = json.loads((candidate_dir / "manifest.json").read_text(encoding="utf-8"))
    manifest = build_visual_manifest(source, candidate, component_ids)
    manifest["dependencies"] = dependency_closure(source, manifest)
    manifest["asset_plan"] = asset_file_plan(manifest, source_content, target_content, engine_content)
    manifest["source_files"] = [str(candidate_dir / name) for name in ("manifest.json", "source-with-collision.json")]
    manifest["api_evidence"] = API_EVIDENCE
    output_dir.mkdir(parents=True, exist_ok=False)
    (output_dir / "visual-manifest.json").write_text(
        json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    (output_dir / "copy-assets.ps1").write_text(
        """# Execute only after approval. Does not launch or save any UE project.
$ErrorActionPreference = 'Stop'
$plan = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot 'visual-manifest.json') | ConvertFrom-Json
$files = $plan.asset_plan.files
foreach ($f in $files) {
    if (-not (Test-Path -LiteralPath $f.source -PathType Leaf)) { throw "Missing source: $($f.source)" }
    if ((Get-Item -LiteralPath $f.source).Length -ne $f.bytes) { throw "Source size changed: $($f.source)" }
    $base = [IO.Path]::GetFileNameWithoutExtension($f.target)
    $parent = [IO.Path]::GetDirectoryName($f.target)
    if (Test-Path -LiteralPath $parent) {
        if (Get-ChildItem -LiteralPath $parent -Filter "$base.*") { throw "Target conflict: $($f.target)" }
    }
}
foreach ($f in $files) {
    [IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($f.target)) | Out-Null
    [IO.File]::Copy($f.source, $f.target, $false)
}
""",
        encoding="utf-8",
    )
    return manifest


def author_editor_level(plan):
    """Explicit editor-only entry AFTER approved copies; saves only a NEW level.

    ``plan`` is the dict read from visual-manifest.json. This does not migrate,
    resolve a session, rebuild gameplay, start physics, or authorize motion.
    Run only in an owned Editor: NewLevel closes its current map without saving.
    """
    import unreal as ue

    content = Path(ue.Paths.project_content_dir()).resolve()
    if (
        Path(ue.Paths.get_project_file_path()).name != "RobotSimUE.uproject"
        or content != Path(plan["asset_plan"]["target_content"]).resolve()
    ):
        raise ValueError("author only in the planned RobotSimUE target, never the source project")
    assets = ue.get_editor_subsystem(ue.EditorAssetSubsystem)
    levels = ue.get_editor_subsystem(ue.LevelEditorSubsystem)
    level = plan["level_path"]
    disk_level = content / (level.removeprefix("/Game/") + ".umap")
    if not level.startswith("/Game/RobotSim/Worlds/FactoryInspectionBay/"):
        raise ValueError("independent FactoryInspectionBay level required")
    if assets.does_asset_exist(level) or disk_level.exists() or disk_level.with_suffix(".uasset").exists():
        raise FileExistsError(f"target level conflict; no overwrite: {level}")
    for file in plan["asset_plan"]["files"]:
        target = Path(file["target"])
        if not target.is_file() or target.stat().st_size != file["bytes"]:
            raise FileNotFoundError(f"planned migrated asset absent/size changed: {target}")
    # Commandlets may not have indexed the copied or Engine material packages.
    registry_files = [f["target"] for f in plan["asset_plan"]["files"] if f["target"].endswith(".uasset")]
    registry_files.extend(
        f["source"]
        for row in plan["asset_plan"].get("external_references", [])
        for f in row.get("files", [])
        if f["source"].endswith(".uasset")
    )
    ue.AssetRegistryHelpers.get_asset_registry().scan_files_synchronous(registry_files, True)
    loaded = {}
    references = {r["package"] for r in plan["dependencies"] if not r["package"].startswith("/Script/")}
    references.update(ref for c in plan["components"] for ref in [c["mesh_asset"], *c["materials"]] if ref)
    for ref in sorted(references):
        asset = assets.load_asset(ref)
        if asset is None:
            raise FileNotFoundError(f"real asset required; no default Cube/material: {ref}")
        loaded[ref] = asset
    for row in plan["components"]:
        if not isinstance(loaded[row["mesh_asset"]], ue.StaticMesh):
            raise ValueError(f"not a real StaticMesh: {row['mesh_asset']}")
        if any(ref and not isinstance(loaded[ref], ue.MaterialInterface) for ref in row["materials"]):
            raise ValueError(f"not a real material: {row['source_id']}")
    if not levels.new_level(level):
        raise RuntimeError(f"could not create new level: {level}")
    actors = ue.get_editor_subsystem(ue.EditorActorSubsystem)
    for row in plan["components"]:
        actor = actors.spawn_actor_from_class(ue.StaticMeshActor, ue.Vector(0, 0, 0))
        if actor is None:
            raise RuntimeError(f"actor creation failed: {row['source_id']}")
        component = actor.get_editor_property("static_mesh_component")
        component.set_mobility(ue.ComponentMobility.MOVABLE)
        component.set_simulate_physics(False)
        component.set_collision_profile_name("NoCollision", False)
        component.set_collision_enabled(ue.CollisionEnabled.NO_COLLISION)
        if not component.set_static_mesh(loaded[row["mesh_asset"]]):
            raise RuntimeError(f"mesh assignment failed: {row['source_id']}")
        for slot, ref in enumerate(row["materials"]):
            component.set_material(slot, loaded[ref] if ref else None)
        component.set_visibility(row["visible"])
        original = row["ue_world_transform"]
        transform = ue.Transform()
        transform.set_editor_property("translation", ue.Vector(*original["translation_cm"]))
        transform.set_editor_property("rotation", ue.Quat(*original["rotation_xyzw"]))
        transform.set_editor_property("scale3d", ue.Vector(*original["scale3d"]))
        actor.set_actor_transform(transform, False, True)
        actor.set_actor_label(row["source_id"].rsplit(".", 2)[-2])
        actor.set_editor_property("tags", ["factory_source_component=" + row["source_id"]])
        observed = actor.get_actor_transform()
        observed_transform = {
            "translation_cm": [getattr(observed.translation, k) for k in "xyz"],
            "rotation_xyzw": [getattr(observed.rotation, k) for k in "xyzw"],
            "scale3d": [getattr(observed.scale3d, k) for k in "xyz"],
        }
        # Quaternion signs may flip on round-trip without changing orientation.
        if sum(a * b for a, b in zip(observed_transform["rotation_xyzw"], original["rotation_xyzw"])) < 0:
            observed_transform["rotation_xyzw"] = [-v for v in observed_transform["rotation_xyzw"]]
        if not all(
            isclose(a, b, abs_tol=1e-6, rel_tol=0)
            for key in original
            for a, b in zip(observed_transform[key], original[key])
        ):
            raise RuntimeError(f"editor transform readback mismatch: {row['source_id']}")
        component.set_mobility(ue.ComponentMobility.STATIC)
        if component.get_collision_enabled() != ue.CollisionEnabled.NO_COLLISION or component.is_simulating_physics():
            raise RuntimeError(f"visual physics policy mismatch: {row['source_id']}")
    if not levels.save_current_level():
        raise RuntimeError(f"new level save failed: {level}")
    return {
        "level_path": level,
        "components": len(plan["components"]),
        "status": "static_initial_pose_authored_not_previewed_not_qualified",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate-dir", required=True, type=Path)
    parser.add_argument("--source-content", required=True, type=Path)
    parser.add_argument("--target-content", required=True, type=Path)
    parser.add_argument("--engine-content", required=True, type=Path)
    parser.add_argument(
        "--component-ids",
        type=Path,
        help="JSON list or review selection.json with component_ids; default all retained static",
    )
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    ids = None if args.component_ids is None else json.loads(args.component_ids.read_text(encoding="utf-8"))
    if isinstance(ids, dict):
        ids = ids["component_ids"]
    plan = write_plan(
        args.candidate_dir, args.source_content, args.target_content, args.engine_content, args.output, ids
    )
    print(
        json.dumps(
            {
                "components": len(plan["components"]),
                "packages": plan["asset_plan"]["copy_packages"],
                "files": len(plan["asset_plan"]["files"]),
                "copy_bytes": plan["asset_plan"]["copy_bytes"],
                "output": str(args.output),
                "assets_copied": False,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
