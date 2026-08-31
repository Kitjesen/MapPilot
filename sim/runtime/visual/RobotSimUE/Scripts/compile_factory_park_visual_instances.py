"""Compile FactoryPark VisualOnly dressing into one serialized HISM batch actor."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import traceback
import uuid
from pathlib import Path
from typing import Mapping, Sequence

try:
    import unreal
except ModuleNotFoundError:
    unreal = None


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = Path(__file__).resolve().parents[5]
BUILDER_PATH = SCRIPT_DIR / "build_factory_park_hf.py"
EVIDENCE_ROOT = Path(
    os.environ.get(
        "LINGTU_FACTORY_PARK_INSTANCE_EVIDENCE_ROOT",
        REPO_ROOT / "build" / "factory-park-hf" / "unreal-instance-compiler",
    )
).resolve()
SUCCESS_SENTINEL = Path(
    os.environ.get(
        "LINGTU_FACTORY_PARK_INSTANCE_SUCCESS",
        EVIDENCE_ROOT / "FactoryPark_HF.instances.success.json",
    )
).resolve()
STAGE_SENTINEL = Path(
    os.environ.get(
        "LINGTU_FACTORY_PARK_INSTANCE_STAGE",
        EVIDENCE_ROOT / "FactoryPark_HF.instances.stage.json",
    )
).resolve()
ERROR_SENTINEL = Path(
    os.environ.get(
        "LINGTU_FACTORY_PARK_INSTANCE_ERROR",
        EVIDENCE_ROOT / "FactoryPark_HF.instances.error.json",
    )
).resolve()
UNATTENDED = os.environ.get("LINGTU_FACTORY_PARK_INSTANCE_UNATTENDED") == "1"
INSTANCE_PHASE = os.environ.get("LINGTU_FACTORY_PARK_INSTANCE_PHASE", "").strip()

BATCH_ACTOR_LABEL = "FactoryPark_HF_VisualInstanceBatches"
BATCH_ACTOR_TAG = "LingTuVisualInstanceBatch"
GENERATED_MAP_ROOT = "/Game/RobotSim/Generated/FactoryParkHF/Maps"


def _load_builder_module():
    spec = importlib.util.spec_from_file_location(
        "lingtu_factory_park_hf_builder_for_instances",
        BUILDER_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load FactoryPark builder: {BUILDER_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _material_asset_path(builder: object, source_material: str, semantic_class: str) -> str:
    name = builder._material_instance_name(source_material, semantic_class)
    root = builder.MATERIAL_ASSET_ROOT
    return f"{root}/{name}.{name}"


def _compile_projection_contract_from_sources() -> dict[str, object]:
    """Resolve the exact source identity partition used by the UE compiler."""

    builder = _load_builder_module()
    world = builder._validate_world_recipe()
    realism = builder._validate_realism_recipe(world)
    authoring = builder._validate_blender_authoring(world, realism)
    instance_plan = builder._build_visual_instance_plan(authoring)

    placed_records = list(authoring["placed_objects"])
    source_ids = [str(record["stable_id"]) for record in placed_records]
    if len(source_ids) != len(set(source_ids)):
        raise RuntimeError("FactoryPark source placements contain duplicate stable IDs")
    layout_ids = sorted(
        str(record["stable_id"])
        for record in authoring["layout_objects"]
    )
    physics_shared_ids = sorted(
        str(record["stable_id"])
        for record in placed_records
        if record["visual_only"] is False
    )
    fallback_ids = sorted(str(value) for value in instance_plan["fallback_stable_ids"])

    groups: list[dict[str, object]] = []
    instanced_ids: list[str] = []
    for raw_group in instance_plan["groups"]:
        instances = list(raw_group["instances"])
        stable_ids = [str(instance["stable_id"]) for instance in instances]
        instanced_ids.extend(stable_ids)
        groups.append(
            {
                "group_id": str(raw_group["group_id"]),
                "primitive_mesh": str(raw_group["primitive_mesh"]),
                "material_asset": _material_asset_path(
                    builder,
                    str(raw_group["source_material"]),
                    str(raw_group["semantic_class"]),
                ),
                "source_material": str(raw_group["source_material"]),
                "semantic_class": str(raw_group["semantic_class"]),
                "cast_shadow": bool(raw_group["cast_shadow"]),
                "instance_count": len(instances),
                "stable_ids": stable_ids,
                "instances": instances,
            }
        )

    expected_ids = set(source_ids)
    individual_ids = set(layout_ids) | set(fallback_ids)
    instanced_id_set = set(instanced_ids)
    if individual_ids.intersection(instanced_id_set):
        raise RuntimeError("FactoryPark individual and instanced stable IDs overlap")
    if individual_ids | instanced_id_set != expected_ids:
        raise RuntimeError("FactoryPark instance partition does not cover the source exactly")
    if len(instanced_ids) != len(instanced_id_set):
        raise RuntimeError("FactoryPark instance groups repeat a stable ID")

    projected_actor_count = len(individual_ids) + (1 if groups else 0)
    contract = {
        "schema": "lingtu.sim.factory-park-visual-instance-contract.v1",
        "map_path": builder.MAP_PATH,
        "world_visual_binding": builder.WORLD_BINDING,
        "source_artifact_set_digest": str(authoring["artifact_set_digest"]),
        "source_stable_id_count": len(source_ids),
        "layout_actor_count": len(layout_ids),
        "physics_shared_actor_count": len(physics_shared_ids),
        "fallback_visual_actor_count": len(fallback_ids),
        "instanced_visual_count": len(instanced_ids),
        "instance_group_count": len(groups),
        "projected_environment_actor_count": projected_actor_count,
        "actor_reduction": len(source_ids) - projected_actor_count,
        "expected_stable_ids": sorted(expected_ids),
        "expected_individual_actor_ids": sorted(individual_ids),
        "layout_actor_ids": layout_ids,
        "physics_shared_actor_ids": physics_shared_ids,
        "fallback_visual_actor_ids": fallback_ids,
        "authority": {
            "physics": "mujoco",
            "unreal_collision": "NoCollision",
            "visual_instance_classification": "VisualOnly",
        },
        "groups": groups,
    }
    contract_digest = hashlib.sha256(
        json.dumps(
            contract,
            ensure_ascii=True,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()
    contract["contract_digest"] = contract_digest
    contract["target_map_path"] = (
        f"{GENERATED_MAP_ROOT}/FactoryPark_HF_Instanced_{contract_digest[:16]}"
    )
    return contract


def _validate_expected_groups(
    contract: Mapping[str, object],
    observed_groups: Sequence[Mapping[str, object]],
) -> tuple[set[str], int]:
    expected_groups = {
        str(group["group_id"]): group
        for group in contract["groups"]  # type: ignore[index]
    }
    observed_by_id: dict[str, Mapping[str, object]] = {}
    for observed in observed_groups:
        group_id = str(observed.get("group_id", ""))
        if not group_id or group_id in observed_by_id:
            raise RuntimeError("FactoryPark observed instance groups repeat or omit a group ID")
        observed_by_id[group_id] = observed
    if set(observed_by_id) != set(expected_groups):
        raise RuntimeError("FactoryPark observed instance group set is not exact")

    stable_ids: set[str] = set()
    instance_count = 0
    for group_id in sorted(expected_groups):
        expected = expected_groups[group_id]
        observed = observed_by_id[group_id]
        expected_ids = [str(value) for value in expected["stable_ids"]]
        observed_ids = [str(value) for value in observed.get("stable_ids", [])]
        if observed_ids != expected_ids:
            raise RuntimeError(f"FactoryPark stable ID order drifted in instance group {group_id}")
        if (
            observed.get("instance_count") != expected.get("instance_count")
            or observed.get("primitive_mesh") != expected.get("primitive_mesh")
            or observed.get("material_asset") != expected.get("material_asset")
            or observed.get("component_material_asset") != expected.get("material_asset")
            or observed.get("cast_shadow") is not expected.get("cast_shadow")
        ):
            raise RuntimeError(f"FactoryPark rendering identity drifted in instance group {group_id}")
        if not (
            observed.get("collision_disabled") is True
            and observed.get("generate_overlap_events") is False
            and observed.get("can_ever_affect_navigation") is False
            and observed.get("simulate_physics") is False
            and observed.get("strict_live_state_verified") is True
        ):
            raise RuntimeError(
                f"FactoryPark presentation policy or live transform state failed for instance group {group_id}"
            )
        overlap = stable_ids.intersection(observed_ids)
        if overlap:
            raise RuntimeError(f"FactoryPark stable IDs repeat across groups: {sorted(overlap)}")
        stable_ids.update(observed_ids)
        instance_count += len(observed_ids)
    return stable_ids, instance_count


def _validate_observed_projection(
    contract: Mapping[str, object],
    *,
    individual_actor_ids: Sequence[str],
    observed_groups: Sequence[Mapping[str, object]],
    batch_actor_count: int,
) -> dict[str, object]:
    """Validate the saved representation without trusting actor-count heuristics."""

    if batch_actor_count != 1:
        raise RuntimeError(
            f"FactoryPark requires exactly one visual batch actor, found {batch_actor_count}"
        )
    expected_individual = set(str(value) for value in contract["expected_individual_actor_ids"])
    observed_individual = [str(value) for value in individual_actor_ids]
    if len(observed_individual) != len(set(observed_individual)):
        raise RuntimeError("FactoryPark individual actors repeat a stable ID")
    if set(observed_individual) != expected_individual:
        raise RuntimeError("FactoryPark individual actor stable ID set is not exact")

    instanced_ids, observed_instance_count = _validate_expected_groups(
        contract,
        observed_groups,
    )
    if expected_individual.intersection(instanced_ids):
        raise RuntimeError("FactoryPark saved representation duplicates stable IDs")
    expected_all = set(str(value) for value in contract["expected_stable_ids"])
    if expected_individual | instanced_ids != expected_all:
        raise RuntimeError("FactoryPark saved representation lost stable IDs")
    return {
        "stable_id_set_exact": True,
        "collision_authority_verified": True,
        "observed_stable_id_count": len(expected_all),
        "observed_individual_actor_count": len(observed_individual),
        "observed_batch_actor_count": batch_actor_count,
        "observed_group_count": len(observed_groups),
        "observed_instance_count": observed_instance_count,
    }


def _actor_tags(actor: object) -> set[str]:
    return {str(tag) for tag in actor.get_editor_property("tags")}


def _collect_individual_actor_map(actor_subsystem: object) -> dict[str, object]:
    actors: dict[str, object] = {}
    for actor in actor_subsystem.get_all_level_actors():
        stable_tags = sorted(tag for tag in _actor_tags(actor) if tag.startswith("StableId:"))
        if not stable_tags:
            continue
        if len(stable_tags) != 1:
            raise RuntimeError(
                f"FactoryPark actor {actor.get_actor_label()} has ambiguous StableId tags"
            )
        stable_id = stable_tags[0].split(":", 1)[1]
        if stable_id in actors:
            raise RuntimeError(f"FactoryPark repeats StableId:{stable_id}")
        actors[stable_id] = actor
    return actors


def _find_batch_actors(actor_subsystem: object) -> list[object]:
    return [
        actor
        for actor in actor_subsystem.get_all_level_actors()
        if BATCH_ACTOR_TAG in _actor_tags(actor)
    ]


def _get_editor_property(target: object, *names: str) -> object:
    errors: list[str] = []
    for name in names:
        try:
            return target.get_editor_property(name)
        except Exception as error:
            errors.append(f"{name}: {error}")
    raise RuntimeError(f"required Unreal property is unavailable ({'; '.join(errors)})")


def _observe_batch_actor(
    builder: object,
    actor: object,
    expected_groups: Sequence[Mapping[str, object]],
) -> list[dict[str, object]]:
    expected_by_id = {
        str(group["group_id"]): group
        for group in expected_groups
    }
    groups = _get_editor_property(actor, "visual_groups", "VisualGroups")
    observations: list[dict[str, object]] = []
    for group in groups:
        group_id = str(_get_editor_property(group, "group_id", "GroupId"))
        stable_ids = [
            str(value)
            for value in _get_editor_property(group, "stable_ids", "StableIds")
        ]
        component = _get_editor_property(
            group,
            "instance_component",
            "InstanceComponent",
        )
        mesh = _get_editor_property(group, "mesh", "Mesh")
        material = _get_editor_property(group, "material", "Material")
        cast_shadow = bool(_get_editor_property(group, "cast_shadow", "bCastShadow"))
        expected = expected_by_id.get(group_id)
        if expected is None:
            raise RuntimeError(f"FactoryPark observed unexpected instance group {group_id}")
        expected_transforms = [
            _instance_transform(instance)
            for instance in expected["instances"]  # type: ignore[index]
        ]
        expected_stable_ids = [str(value) for value in expected["stable_ids"]]
        expected_mesh = unreal.load_asset(str(expected["primitive_mesh"]))
        expected_material = unreal.load_asset(str(expected["material_asset"]))
        if expected_mesh is None or expected_material is None:
            raise RuntimeError(f"FactoryPark expected group assets are unavailable: {group_id}")
        strict_live_state_verified = bool(
            actor.validate_visual_group(
                group_id,
                expected_mesh,
                expected_material,
                expected_transforms,
                expected_stable_ids,
                bool(expected["cast_shadow"]),
            )
        )
        actual_component_material = component.get_material(0)
        collision = builder._read_collision_component_policy(component)
        try:
            can_affect_navigation = bool(
                component.get_editor_property("can_ever_affect_navigation")
            )
        except Exception as error:
            raise RuntimeError(
                f"FactoryPark cannot verify navigation policy for {group_id}: {error}"
            ) from error
        observations.append(
            {
                "group_id": group_id,
                "stable_ids": stable_ids,
                "instance_count": int(component.get_instance_count()),
                "primitive_mesh": str(mesh.get_path_name()),
                "material_asset": str(material.get_path_name()),
                "component_material_asset": (
                    ""
                    if actual_component_material is None
                    else str(actual_component_material.get_path_name())
                ),
                "cast_shadow": cast_shadow,
                "collision_disabled": collision["collision_disabled"],
                "generate_overlap_events": collision["generate_overlap_events"],
                "can_ever_affect_navigation": can_affect_navigation,
                "simulate_physics": bool(component.is_simulating_physics()),
                "strict_live_state_verified": strict_live_state_verified,
            }
        )
    return observations


def _spawn_batch_actor(actor_subsystem: object) -> object:
    batch_actor_class = getattr(unreal, "LingTuSimVisualInstanceBatchActor", None)
    if batch_actor_class is None:
        raise RuntimeError(
            "LingTuSimVisualInstanceBatchActor is unavailable; build RobotSimUEEditor first"
        )
    actor = actor_subsystem.spawn_actor_from_class(
        batch_actor_class,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    if actor is None:
        raise RuntimeError("could not spawn FactoryPark visual instance batch actor")
    actor.set_actor_label(BATCH_ACTOR_LABEL)
    actor.set_editor_property(
        "tags",
        [
            unreal.Name("FactoryParkHF"),
            unreal.Name("VisualOnly"),
            unreal.Name(BATCH_ACTOR_TAG),
        ],
    )
    actor.set_actor_enable_collision(False)
    return actor


def _instance_transform(instance: Mapping[str, object]) -> object:
    location = instance["location_cm"]
    quaternion = instance["quaternion_xyzw"]
    scale = instance["scale_xyz"]
    return unreal.Transform(
        location=unreal.Vector(*location),
        rotation=unreal.Quat(*quaternion).rotator(),
        scale=unreal.Vector(*scale),
    )


def _materialize_projection(
    builder: object,
    actor_subsystem: object,
    contract: Mapping[str, object],
) -> dict[str, object]:
    """Build every batch first, then remove only the represented source actors."""

    expected_all = set(str(value) for value in contract["expected_stable_ids"])
    expected_individual = set(
        str(value) for value in contract["expected_individual_actor_ids"]
    )
    instanced_ids = expected_all - expected_individual
    individual_actors = _collect_individual_actor_map(actor_subsystem)
    batch_actors = _find_batch_actors(actor_subsystem)

    if len(batch_actors) == 1 and set(individual_actors) == expected_individual:
        observations = _observe_batch_actor(
            builder,
            batch_actors[0],
            contract["groups"],  # type: ignore[arg-type,index]
        )
        _validate_observed_projection(
            contract,
            individual_actor_ids=sorted(individual_actors),
            observed_groups=observations,
            batch_actor_count=1,
        )
        return {"mode": "already_compiled", "removed_source_actor_count": 0}
    if batch_actors:
        raise RuntimeError("FactoryPark map contains a partial or duplicate instance compilation")
    if set(individual_actors) != expected_all:
        raise RuntimeError("FactoryPark source map stable ID set is not exact before compilation")

    batch_actor = _spawn_batch_actor(actor_subsystem)
    for group in contract["groups"]:  # type: ignore[index]
        mesh = unreal.load_asset(str(group["primitive_mesh"]))
        material = unreal.load_asset(str(group["material_asset"]))
        if mesh is None or material is None:
            raise RuntimeError(
                f"FactoryPark instance group assets are unavailable: {group['group_id']}"
            )
        transforms = [_instance_transform(instance) for instance in group["instances"]]
        stable_ids = [str(value) for value in group["stable_ids"]]
        if not batch_actor.add_visual_group(
            str(group["group_id"]),
            mesh,
            material,
            transforms,
            stable_ids,
            bool(group["cast_shadow"]),
        ):
            raise RuntimeError(
                f"FactoryPark batch actor rejected complete group {group['group_id']}"
            )

    observations = _observe_batch_actor(
        builder,
        batch_actor,
        contract["groups"],  # type: ignore[arg-type,index]
    )
    _validate_expected_groups(contract, observations)
    for stable_id in sorted(instanced_ids):
        if not actor_subsystem.destroy_actor(individual_actors[stable_id]):
            raise RuntimeError(f"could not remove instanced source actor {stable_id}")
    return {
        "mode": "compiled",
        "removed_source_actor_count": len(instanced_ids),
    }


def _asset_package_path(raw_path: object) -> str:
    return str(raw_path).split(".", maxsplit=1)[0]


def _materialize_source_copy_to_staging(
    builder: object,
    actor_subsystem: object,
    asset_subsystem: object,
    contract: Mapping[str, object],
    staging_map_path: str,
) -> dict[str, object]:
    """Mutate only the in-memory source world, save its copy, then exit."""

    editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
    source_world = editor_subsystem.get_editor_world()
    if source_world is None:
        raise RuntimeError("FactoryPark source editor world is unavailable")
    materialization = _materialize_projection(builder, actor_subsystem, contract)
    if not unreal.EditorLoadingAndSavingUtils.save_map(source_world, staging_map_path):
        raise RuntimeError(f"could not save FactoryPark staging map: {staging_map_path}")
    if not asset_subsystem.does_asset_exist(staging_map_path):
        raise RuntimeError("FactoryPark Save Map did not create the staging asset")
    return materialization


def _atomic_write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f"{path.name}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _verify_loaded_projection(
    builder: object,
    actor_subsystem: object,
    contract: Mapping[str, object],
) -> dict[str, object]:
    individual_actors = _collect_individual_actor_map(actor_subsystem)
    batch_actors = _find_batch_actors(actor_subsystem)
    observations = (
        _observe_batch_actor(
            builder,
            batch_actors[0],
            contract["groups"],  # type: ignore[arg-type,index]
        )
        if len(batch_actors) == 1
        else []
    )
    return _validate_observed_projection(
        contract,
        individual_actor_ids=sorted(individual_actors),
        observed_groups=observations,
        batch_actor_count=len(batch_actors),
    )


def _find_unique_actor_by_label(actor_subsystem: object, label: str) -> object:
    matches = [
        actor
        for actor in actor_subsystem.get_all_level_actors()
        if str(actor.get_actor_label()) == label
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"FactoryPark target rendering requires exactly one actor labelled {label}, "
            f"found {len(matches)}"
        )
    return matches[0]


def _synchronize_target_rendering_contract(
    builder: object,
    actor_subsystem: object,
) -> dict[str, object]:
    """Apply the source rendering contract to an existing generated target map."""

    world = builder._validate_world_recipe()
    realism = builder._validate_realism_recipe(world)
    lighting = realism.get("lighting")
    if not isinstance(lighting, Mapping):
        raise RuntimeError("FactoryPark target rendering requires a lighting recipe")
    sun_recipe = lighting.get("sun")
    if not isinstance(sun_recipe, Mapping):
        raise RuntimeError("FactoryPark target rendering requires a sun recipe")

    sun_actor = _find_unique_actor_by_label(actor_subsystem, str(builder.SUN_LABEL))
    post_process = _find_unique_actor_by_label(
        actor_subsystem,
        str(builder.POST_PROCESS_LABEL),
    )
    sun_component = getattr(sun_actor, "light_component", None)
    if sun_component is None:
        raise RuntimeError("FactoryPark target sun actor has no light component")

    sun_evidence = builder._configure_sun_component(sun_component, sun_recipe)
    lumen_evidence = builder._configure_lumen_world(post_process, realism)
    if not (
        isinstance(sun_evidence, Mapping)
        and sun_evidence.get("verified") is True
        and isinstance(lumen_evidence, Mapping)
        and isinstance(lumen_evidence.get("exposure"), Mapping)
        and lumen_evidence["exposure"].get("verified") is True
    ):
        raise RuntimeError("FactoryPark target rendering contract did not verify")
    return {
        "sun": dict(sun_evidence),
        "lumen": dict(lumen_evidence),
        "verified": True,
    }


def _verify_loaded_source_map_pristine(
    builder: object,
    actor_subsystem: object,
    contract: Mapping[str, object],
    expected_fingerprint: Mapping[str, object],
) -> dict[str, object]:
    actual_fingerprint = _capture_source_map_fingerprint(
        builder,
        actor_subsystem,
        str(contract["map_path"]),
    )
    if actual_fingerprint != expected_fingerprint:
        raise RuntimeError(
            "FactoryPark immutable source map package or actor state changed during staged compilation"
        )
    individual_ids = set(_collect_individual_actor_map(actor_subsystem))
    expected_ids = set(str(value) for value in contract["expected_stable_ids"])
    batch_count = len(_find_batch_actors(actor_subsystem))
    if individual_ids != expected_ids or batch_count != 0:
        raise RuntimeError("FactoryPark immutable source map changed during staged compilation")
    return {
        "source_map_unchanged": True,
        "source_stable_id_count": len(individual_ids),
        "source_batch_actor_count": batch_count,
        "source_package_sha256": actual_fingerprint["package_sha256"],
        "source_actor_fingerprint_sha256": actual_fingerprint["actor_fingerprint_sha256"],
    }


def _capture_source_map_fingerprint(
    builder: object,
    actor_subsystem: object,
    source_map_path: str,
) -> dict[str, object]:
    if not source_map_path.startswith("/Game/"):
        raise RuntimeError(f"FactoryPark source map is outside /Game: {source_map_path}")
    package_file = (
        builder.PROJECT_DIR
        / "Content"
        / Path(*source_map_path.removeprefix("/Game/").split("/"))
    ).with_suffix(".umap")
    if not package_file.is_file():
        raise RuntimeError(f"FactoryPark source map package file is missing: {package_file}")

    actor_records: list[dict[str, object]] = []
    for stable_id, actor in sorted(_collect_individual_actor_map(actor_subsystem).items()):
        tags = sorted(_actor_tags(actor))
        location = actor.get_actor_location()
        quaternion = actor.get_actor_rotation().quaternion()
        scale = actor.get_actor_scale3d()
        component = actor.get_editor_property("static_mesh_component")
        mesh = component.get_editor_property("static_mesh")
        material_paths: list[str] = []
        for slot in range(int(component.get_num_materials())):
            material = component.get_material(slot)
            material_paths.append("" if material is None else str(material.get_path_name()))
        collision = builder._read_collision_component_policy(component)
        actor_records.append(
            {
                "stable_id": stable_id,
                "actor_label": str(actor.get_actor_label()),
                "actor_class": str(actor.get_class().get_path_name()),
                "tags": tags,
                "location_cm": [float(location.x), float(location.y), float(location.z)],
                "quaternion_xyzw": [
                    float(quaternion.x),
                    float(quaternion.y),
                    float(quaternion.z),
                    float(quaternion.w),
                ],
                "scale": [float(scale.x), float(scale.y), float(scale.z)],
                "mesh_asset": "" if mesh is None else str(mesh.get_path_name()),
                "material_assets": material_paths,
                "cast_shadow": bool(component.get_editor_property("cast_shadow")),
                "collision": collision,
            }
        )
    actor_payload = json.dumps(
        actor_records,
        ensure_ascii=True,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return {
        "map_path": source_map_path,
        "package_file": str(package_file),
        "package_bytes": package_file.stat().st_size,
        "package_sha256": builder._sha256_file(package_file),
        "stable_actor_count": len(actor_records),
        "actor_fingerprint_sha256": hashlib.sha256(actor_payload).hexdigest(),
    }


def _delete_stale_staging_assets(asset_subsystem: object) -> list[str]:
    """Remove abandoned generated maps before loading this run's staging world."""

    staging_root = f"{GENERATED_MAP_ROOT}/Staging"
    removed: list[str] = []
    for raw_path in sorted(
        unreal.EditorAssetLibrary.list_assets(
            staging_root,
            recursive=True,
            include_folder=False,
        ),
        key=str,
    ):
        package_path = _asset_package_path(raw_path)
        if not package_path.startswith(f"{staging_root}/FactoryPark_HF_"):
            raise RuntimeError(
                f"refusing to remove unexpected generated staging asset: {package_path}"
            )
        if not asset_subsystem.delete_asset(package_path):
            raise RuntimeError(f"could not remove stale staging asset: {package_path}")
        removed.append(package_path)
    return removed


def _read_stage_evidence() -> dict[str, object]:
    try:
        payload = json.loads(STAGE_SENTINEL.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"could not read FactoryPark stage evidence: {STAGE_SENTINEL}") from error
    if not isinstance(payload, dict):
        raise RuntimeError("FactoryPark stage evidence must be an object")
    return payload


def _stage_factory_park_visual_instances() -> dict[str, object]:
    """Build and save staging without attempting to unload its standalone UWorld."""

    builder = _load_builder_module()
    contract = _compile_projection_contract_from_sources()
    level_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)
    source_map_path = str(contract["map_path"])
    target_map_path = str(contract["target_map_path"])
    if not asset_subsystem.does_asset_exist(source_map_path):
        raise RuntimeError(f"FactoryPark source map is missing: {source_map_path}")
    if not level_subsystem.load_level(source_map_path):
        raise RuntimeError(f"could not load immutable FactoryPark source map: {source_map_path}")
    source_fingerprint = _capture_source_map_fingerprint(
        builder,
        actor_subsystem,
        source_map_path,
    )
    removed_staging_assets = _delete_stale_staging_assets(asset_subsystem)

    staging_map_path: str | None = None
    if asset_subsystem.does_asset_exist(target_map_path):
        materialization = {
            "mode": "existing_target",
            "removed_source_actor_count": 0,
        }
    else:
        staging_map_path = (
            f"{GENERATED_MAP_ROOT}/Staging/FactoryPark_HF_{uuid.uuid4().hex}"
        )
        materialization = _materialize_source_copy_to_staging(
            builder,
            actor_subsystem,
            asset_subsystem,
            contract,
            staging_map_path,
        )

    payload: dict[str, object] = {
        "schema": "lingtu.sim.factory-park-visual-instance-stage.v1",
        "result": "staged",
        "engine_version": unreal.SystemLibrary.get_engine_version(),
        "source_map_path": source_map_path,
        "staging_map_path": staging_map_path,
        "target_map_path": target_map_path,
        "contract": contract,
        "source_fingerprint": source_fingerprint,
        "materialization": materialization,
        "removed_stale_staging_assets": removed_staging_assets,
    }
    _atomic_write_json(STAGE_SENTINEL, payload)
    unreal.log(
        "LINGTU_FACTORY_PARK_INSTANCE_STAGED="
        f"{STAGE_SENTINEL} staging={staging_map_path or 'existing-target'}"
    )
    return payload


def _publish_factory_park_visual_instances() -> dict[str, object]:
    """Verify source and publish the saved map from a fresh Editor process."""

    builder = _load_builder_module()
    contract = _compile_projection_contract_from_sources()
    stage = _read_stage_evidence()
    if (
        stage.get("schema") != "lingtu.sim.factory-park-visual-instance-stage.v1"
        or stage.get("result") != "staged"
        or stage.get("contract") != contract
    ):
        raise RuntimeError("FactoryPark stage evidence does not match the current contract")

    level_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)
    source_map_path = str(contract["map_path"])
    target_map_path = str(contract["target_map_path"])
    if stage.get("source_map_path") != source_map_path:
        raise RuntimeError("FactoryPark stage source map path changed before publication")
    if stage.get("target_map_path") != target_map_path:
        raise RuntimeError("FactoryPark stage target map path changed before publication")
    if not level_subsystem.load_level(source_map_path):
        raise RuntimeError(f"could not load immutable FactoryPark source map: {source_map_path}")
    expected_source_fingerprint = stage.get("source_fingerprint")
    if not isinstance(expected_source_fingerprint, dict):
        raise RuntimeError("FactoryPark stage evidence omitted source fingerprint")
    source_verification = _verify_loaded_source_map_pristine(
        builder,
        actor_subsystem,
        contract,
        expected_source_fingerprint,
    )

    staging_map_path = stage.get("staging_map_path")
    if staging_map_path is not None:
        if not isinstance(staging_map_path, str) or not staging_map_path.startswith(
            f"{GENERATED_MAP_ROOT}/Staging/FactoryPark_HF_"
        ):
            raise RuntimeError("FactoryPark stage evidence contains an invalid staging map path")
        if asset_subsystem.does_asset_exist(target_map_path):
            raise RuntimeError("FactoryPark target appeared between stage and publish phases")
        if not asset_subsystem.does_asset_exist(staging_map_path):
            raise RuntimeError("FactoryPark staging map disappeared before publication")
        if not asset_subsystem.rename_asset(staging_map_path, target_map_path):
            raise RuntimeError(
                f"could not promote FactoryPark staging map to {target_map_path}"
            )
    elif not asset_subsystem.does_asset_exist(target_map_path):
        raise RuntimeError("FactoryPark existing target disappeared before publication")

    if not level_subsystem.load_level(target_map_path):
        raise RuntimeError(f"could not load compiled FactoryPark map: {target_map_path}")
    rendering_contract = _synchronize_target_rendering_contract(
        builder,
        actor_subsystem,
    )
    editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
    target_world = editor_subsystem.get_editor_world()
    if target_world is None:
        raise RuntimeError("FactoryPark compiled target editor world is unavailable")
    if not unreal.EditorLoadingAndSavingUtils.save_map(target_world, target_map_path):
        raise RuntimeError(f"could not save FactoryPark target rendering: {target_map_path}")
    verification = _verify_loaded_projection(builder, actor_subsystem, contract)
    payload = {
        "schema": "lingtu.sim.factory-park-visual-instance-evidence.v1",
        "result": "success",
        "engine_version": unreal.SystemLibrary.get_engine_version(),
        "source_map_path": source_map_path,
        "map_path": target_map_path,
        "contract": contract,
        "materialization": stage["materialization"],
        "verification": verification,
        "rendering_contract": rendering_contract,
        "source_verification": source_verification,
        "physics_authority": "mujoco",
        "unreal_collision": "NoCollision",
    }
    _atomic_write_json(SUCCESS_SENTINEL, payload)
    unreal.log(
        "LINGTU_FACTORY_PARK_INSTANCE_READY="
        f"{SUCCESS_SENTINEL} instances={verification['observed_instance_count']}"
    )
    return payload


def compile_factory_park_visual_instances() -> dict[str, object]:
    """Run one explicit half of the process-isolated publication transaction."""

    if unreal is None:
        raise RuntimeError("instance compilation must run inside Unreal Editor Python")
    if INSTANCE_PHASE == "stage":
        return _stage_factory_park_visual_instances()
    if INSTANCE_PHASE == "publish":
        return _publish_factory_park_visual_instances()
    raise RuntimeError(
        "LINGTU_FACTORY_PARK_INSTANCE_PHASE must be exactly 'stage' or 'publish'"
    )


def _write_error(error: BaseException) -> None:
    _atomic_write_json(
        ERROR_SENTINEL,
        {
            "schema": "lingtu.sim.factory-park-visual-instance-error.v1",
            "result": "error",
            "phase": INSTANCE_PHASE,
            "error": f"{type(error).__name__}: {error}",
            "traceback": traceback.format_exc(),
        },
    )


def main() -> None:
    """Run the explicit Unreal Editor compiler entry point."""

    try:
        compile_factory_park_visual_instances()
    except Exception as error:
        _write_error(error)
        if unreal is not None:
            unreal.log_error(f"LINGTU_FACTORY_PARK_INSTANCE_ERROR={ERROR_SENTINEL}")
        raise
    finally:
        if UNATTENDED and unreal is not None:
            unreal.SystemLibrary.quit_editor()


if __name__ == "__main__":
    main()
