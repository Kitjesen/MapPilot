"""Static source projection contracts; no Unreal or physics execution."""

import copy
import json
from pathlib import Path

import numpy as np
import pytest
from sim.tools.worlds.factory_environment_collection import visual_authoring as visual


def fixture():
    mesh = "/Game/Meshes/Motor.Motor"
    material = "/Game/Materials/Paint.Paint"
    transform = {
        "translation_cm": [5200, -125, 83],
        "rotation_xyzw": [0, 0, 0.6, 0.8],
        "scale3d": [-2, 3, 0.5],
    }
    component = {
        "source_id": "/source/actor.motor",
        "kind": "static_mesh",
        "class": "StaticMeshComponent",
        "mesh_asset": mesh,
        "materials": [material, None, material],
        "world_transform": transform,
        "visible": {"status": "available", "value": True},
    }
    source = {
        "schema": "lingtu.factory-bay-authoring.v1",
        "units": "Unreal_cm",
        "frame": "Unreal_left_handed_Xforward_Yright_Zup",
        "source_inventory_matches": True,
        "source_project": "FactoryProject.uproject",
        "source_world": "/Game/Maps/Demonstration.Demonstration",
        "actors": [{"source_id": "/source/actor", "components": [component]}],
        "mesh_assets": [{"source_id": mesh, "kind": "static_mesh"}],
    }
    candidate = {
        "schema": "lingtu.factory-collision-candidate.v1",
        "source_project": source["source_project"],
        "source_world": source["source_world"],
        "components": [
            {
                "source_id": component["source_id"],
                "actor_source_id": "/source/actor",
                "kind": "static_mesh",
                "mesh_asset": mesh,
                "source_world_transform": copy.deepcopy(transform),
                "geometry_ids": ["geom:1", "geom:2"],
                "canonical_from_asset_cm": [
                    [-0.0056, -0.0288, 0, 52],
                    [0.0192, -0.0084, 0, 1.25],
                    [0, 0, 0.005, 0.83],
                    [0, 0, 0, 1],
                ],
            }
        ],
        "geometries": [
            {
                "source_id": gid,
                "component_source_id": component["source_id"],
                "actor_source_id": "/source/actor",
                "asset_source_id": mesh,
            }
            for gid in ["geom:1", "geom:2"]
        ],
    }
    return source, candidate


def test_preserves_component_material_slots_negative_scale_and_both_frames():
    source, candidate = fixture()
    before = copy.deepcopy((source, candidate))
    result = visual.build_visual_manifest(source, candidate)
    (row,) = result["components"]
    original = source["actors"][0]["components"][0]
    assert row["source_id"] == original["source_id"]
    assert row["mesh_asset"] == original["mesh_asset"]
    assert row["materials"] == original["materials"]
    assert row["ue_world_transform"] == original["world_transform"]
    np.testing.assert_allclose(row["canonical_transform"]["translation_m"], [52, 1.25, 0.83])
    assert row["canonical_transform"]["rotation_xyzw"] == [0, 0, -0.6, 0.8]
    assert row["canonical_transform"]["scale3d"] == [-2, 3, 0.5]
    np.testing.assert_allclose(
        row["canonical_from_asset_cm"], candidate["components"][0]["canonical_from_asset_cm"], atol=1e-14
    )
    assert row["geometry_ids"] == ["geom:1", "geom:2"]
    assert row["collision_profile"] == "NoCollision" and row["simulate_physics"] is False
    assert (source, candidate) == before


@pytest.mark.parametrize(
    "damage",
    [
        "missing_component",
        "wrong_mesh",
        "wrong_actor",
        "missing_geometry",
        "orphan_geometry",
        "wrong_geometry_asset",
        "changed_transform",
        "missing_asset",
        "spline",
    ],
)
def test_incomplete_or_mismatched_source_geometry_association_fails(damage):
    source, candidate = fixture()
    component = source["actors"][0]["components"][0]
    if damage == "missing_component":
        source["actors"][0]["components"] = []
    elif damage == "wrong_mesh":
        component["mesh_asset"] = "/Game/Wrong.Wrong"
    elif damage == "wrong_actor":
        candidate["components"][0]["actor_source_id"] = "/wrong"
    elif damage == "missing_geometry":
        candidate["geometries"].pop()
    elif damage == "orphan_geometry":
        candidate["components"][0]["geometry_ids"].pop()
    elif damage == "wrong_geometry_asset":
        candidate["geometries"][0]["asset_source_id"] = "/wrong"
    elif damage == "changed_transform":
        component["world_transform"]["scale3d"][0] = 2
    elif damage == "missing_asset":
        source["mesh_assets"] = []
    else:
        component["class"] = "SplineMeshComponent"
    with pytest.raises(ValueError):
        visual.build_visual_manifest(source, candidate)


def test_context_extension_reuses_only_byte_identical_existing_packages(tmp_path):
    content, target, engine = (tmp_path / n for n in ("source", "target", "engine"))
    for root in (content, target):
        (root / "Meshes").mkdir(parents=True)
        (root / "Meshes/A.uasset").write_bytes(b"unchanged-source")
    manifest = {"dependencies": [{"package": "/Game/Meshes/A"}], "level_path": visual.LEVEL_PATH}
    plan = visual.asset_file_plan(manifest, content, target, engine, reuse_identical=True)
    assert plan["copy_bytes"] == 0 and plan["files"][0]["action"] == "reuse_verified_identical"
    (target / "Meshes/A.uasset").write_bytes(b"user-edited-asset")
    with pytest.raises(FileExistsError):
        visual.asset_file_plan(manifest, content, target, engine, reuse_identical=True)
    assert (target / "Meshes/A.uasset").read_bytes() == b"user-edited-asset"


def dependency_source():
    source, candidate = fixture()
    refs = {
        "/Game/Meshes/Motor": ["/Game/Materials/Paint", "/Script/Engine"],
        "/Game/Materials/Paint": ["/Game/Materials/Base", "/Game/Textures/T"],
        "/Game/Materials/Base": ["/Game/Textures/T"],
        "/Game/Textures/T": ["/Engine/BasicShapes/BasicShapeMaterial"],
        "/Engine/BasicShapes/BasicShapeMaterial": [],
        "/Game/Unused": [],
    }
    source["dependencies"] = {
        "complete": True,
        "pending_budget": [],
        "packages": [
            {
                "package": p,
                "disposition": "inventory_only_no_copy",
                "asset_classes": ["StaticMesh" if p.endswith("Motor") else "Material"],
                "dependencies": {"status": "available", "value": ds},
            }
            for p, ds in refs.items()
        ]
        + [{"package": "/Script/Engine", "disposition": "native_type_reference_not_asset_copy"}],
    }
    return source, candidate


def test_dependency_closure_follows_exported_records_and_counts_actual_sidecars(tmp_path):
    source, candidate = dependency_source()
    manifest = visual.build_visual_manifest(source, candidate)
    manifest["dependencies"] = visual.dependency_closure(source, manifest)
    packages = {p["package"] for p in manifest["dependencies"]}
    assert "/Game/Unused" not in packages
    assert "/Game/Materials/Base" in packages and "/Script/Engine" in packages
    content, target, engine = (tmp_path / n for n in ("source", "target", "engine"))
    for p in manifest["dependencies"]:
        if p["package"].startswith("/Script/"):
            continue
        root = content if p["package"].startswith("/Game/") else engine
        file = root / (p["package"].split("/", 2)[2] + ".uasset")
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_bytes(b"asset")
    sidecar = content / "Textures/T.ubulk"
    sidecar.write_bytes(b"1234567")
    plan = visual.asset_file_plan(manifest, content, target, engine)
    assert plan["copy_bytes"] == 27 and len(plan["files"]) == 5
    assert sum(f["bytes"] for f in plan["files"]) == 27
    assert next(f for f in plan["files"] if f["source"].endswith("T.ubulk"))["bytes"] == 7
    assert not target.exists()
    assert all(f["target"].startswith(str(target)) for f in plan["files"])
    sidecar.unlink()
    (content / "Meshes/Motor.uasset").unlink()
    with pytest.raises(FileNotFoundError, match="Motor"):
        visual.asset_file_plan(manifest, content, target, engine)


@pytest.mark.parametrize("damage", ["missing_record", "unavailable", "excluded_gameplay"])
def test_dependency_gaps_are_not_silently_completed(damage):
    source, candidate = dependency_source()
    records = source["dependencies"]["packages"]
    row = next(r for r in records if r["package"] == "/Game/Materials/Base")
    if damage == "missing_record":
        records.remove(row)
    elif damage == "unavailable":
        row["dependencies"] = {"status": "unavailable"}
    else:
        row["disposition"] = "non_geometry_dependency_excluded"
    with pytest.raises(ValueError):
        visual.dependency_closure(source, visual.build_visual_manifest(source, candidate))


def test_review_component_selection_keeps_whole_geometry_and_classifies_helpers():
    source, candidate = fixture()
    selected = source["actors"][0]["components"][0]["source_id"]
    source["actors"][0]["components"].append(
        {
            "source_id": "curve",
            "kind": "other_scene_component",
            "class": "SplineComponent",
        }
    )
    candidate["components"].append({"source_id": "curve", "geometry_ids": []})
    full = visual.build_visual_manifest(source, candidate)
    subset = visual.build_visual_manifest(source, candidate, component_ids=[selected])
    assert subset["components"] == full["components"]
    assert subset["omitted_components"] == [{"source_id": "curve", "reason": "editor_curve_helper_not_deformed_mesh"}]
    empty = visual.build_visual_manifest(source, candidate, component_ids=[])
    assert not empty["components"]
    assert empty["excluded_geometry_ids"] == ["geom:1", "geom:2"]
    with pytest.raises(ValueError, match="retained"):
        visual.build_visual_manifest(source, candidate, component_ids=["curve"])


REAL_ROOT = Path(__file__).resolve().parents[2] / "build/inspection-tests/20261022/factory-collision-candidate-02"


def test_real_plan_is_complete_read_only_and_has_exact_disk_inventory(tmp_path):
    if not REAL_ROOT.exists():
        pytest.skip("licensed local source export absent; not replaced by synthetic evidence")
    source_content = Path("D:/epic/Projects/FactoryPreview/Content")
    if not source_content.exists():
        pytest.skip("licensed source assets absent")
    plan = visual.write_plan(
        REAL_ROOT,
        source_content,
        tmp_path / "target",
        Path("D:/Program Files/Epic Games/UE_5.8/Engine/Content"),
        tmp_path / "plan",
    )
    assert len(plan["components"]) == 460
    assert sum(len(r["geometry_ids"]) for r in plan["components"]) == 1065
    assert len(plan["omitted_components"]) == 887
    assert sum(r["reason"] == "editor_curve_helper_not_deformed_mesh" for r in plan["omitted_components"]) == 2
    assert any(any(s < 0 for s in r["ue_world_transform"]["scale3d"]) for r in plan["components"])
    files = plan["asset_plan"]["files"]
    assert plan["asset_plan"]["copy_bytes"] == sum(Path(f["source"]).stat().st_size for f in files)
    assert not (tmp_path / "target").exists()
    assert json.loads((tmp_path / "plan/visual-manifest.json").read_text(encoding="utf-8")) == plan
    assert (tmp_path / "plan/copy-assets.ps1").exists()
    with pytest.raises(FileExistsError):
        visual.write_plan(
            REAL_ROOT,
            source_content,
            tmp_path / "target",
            Path("D:/Program Files/Epic Games/UE_5.8/Engine/Content"),
            tmp_path / "plan",
        )


def test_editor_preflights_real_material_and_refuses_source_project(monkeypatch, tmp_path):
    import sys
    from types import SimpleNamespace
    from unittest.mock import Mock

    source, candidate = dependency_source()
    plan = visual.build_visual_manifest(source, candidate)
    plan["dependencies"] = visual.dependency_closure(source, plan)
    plan["asset_plan"] = {"target_content": str(tmp_path / "Content"), "files": []}
    assets, levels = Mock(), Mock()
    assets.does_asset_exist.return_value = False
    assets.load_asset.side_effect = lambda ref: None if ref == "/Game/Materials/Paint" else object()
    ue = SimpleNamespace(
        Paths=SimpleNamespace(
            project_content_dir=lambda: str(tmp_path / "Content"), get_project_file_path=lambda: "RobotSimUE.uproject"
        ),
        EditorAssetSubsystem="assets",
        LevelEditorSubsystem="levels",
        AssetRegistryHelpers=SimpleNamespace(get_asset_registry=lambda: Mock()),
        get_editor_subsystem=lambda kind: {"assets": assets, "levels": levels}[kind],
    )
    monkeypatch.setitem(sys.modules, "unreal", ue)
    with pytest.raises(FileNotFoundError, match=r"real asset.*Paint"):
        visual.author_editor_level(plan)
    levels.new_level.assert_not_called()
    ue.Paths.get_project_file_path = lambda: source["source_project"]
    with pytest.raises(ValueError, match="RobotSimUE"):
        visual.author_editor_level(plan)
    levels.new_level.assert_not_called()


def test_editor_entry_preserves_pose_slots_and_no_collision(monkeypatch, tmp_path):
    import sys
    from types import SimpleNamespace
    from unittest.mock import Mock

    class Struct(SimpleNamespace):
        def set_editor_property(self, name, value):
            setattr(self, name, value)

    class Mesh:
        pass

    class Material:
        pass

    source, candidate = dependency_source()
    plan = visual.build_visual_manifest(source, candidate)
    plan["dependencies"] = visual.dependency_closure(source, plan)
    plan["asset_plan"] = {
        "target_content": str(tmp_path / "Content"),
        "files": [],
        "external_references": [{"files": [{"source": "Engine/CustomRotator.uasset"}]}],
    }
    mesh, material = Mesh(), Material()
    assets, levels, actors, actor, component = (Mock() for _ in range(5))
    assets.does_asset_exist.return_value = False
    registry = Mock()

    def load_after_scan(ref):
        registry.scan_files_synchronous.assert_called_once_with(["Engine/CustomRotator.uasset"], True)
        return mesh if "Motor" in ref else material

    assets.load_asset.side_effect = load_after_scan
    actors.spawn_actor_from_class.return_value = actor
    actor.get_editor_property.return_value = component
    actor.get_actor_transform.side_effect = lambda: actor.set_actor_transform.call_args.args[0]
    component.get_collision_enabled.return_value = "NO_COLLISION"
    component.is_simulating_physics.return_value = False
    ue = SimpleNamespace(
        Paths=SimpleNamespace(
            project_content_dir=lambda: str(tmp_path / "Content"), get_project_file_path=lambda: "RobotSimUE.uproject"
        ),
        EditorAssetSubsystem="assets",
        LevelEditorSubsystem="levels",
        EditorActorSubsystem="actors",
        AssetRegistryHelpers=SimpleNamespace(get_asset_registry=lambda: registry),
        get_editor_subsystem=lambda kind: {"assets": assets, "levels": levels, "actors": actors}[kind],
        StaticMesh=Mesh,
        MaterialInterface=Material,
        StaticMeshActor="StaticMeshActor",
        Vector=lambda x, y, z: Struct(x=x, y=y, z=z),
        Quat=lambda x, y, z, w: Struct(x=x, y=y, z=z, w=w),
        Transform=Struct,
        ComponentMobility=SimpleNamespace(MOVABLE="movable", STATIC="static"),
        CollisionEnabled=SimpleNamespace(NO_COLLISION="NO_COLLISION"),
    )
    monkeypatch.setitem(sys.modules, "unreal", ue)
    result = visual.author_editor_level(plan)
    assert result["components"] == 1 and "not_qualified" in result["status"]
    assert actors.spawn_actor_from_class.call_args.args[0] == "StaticMeshActor"
    assert component.set_material.call_args_list[1].args == (1, None)
    assert component.set_material.call_args_list[2].args == (2, material)
    transform = actor.set_actor_transform.call_args.args[0]
    assert [transform.scale3d.x, transform.scale3d.y, transform.scale3d.z] == [-2, 3, 0.5]
    assert [transform.translation.x, transform.translation.y, transform.translation.z] == [5200, -125, 83]
    component.set_simulate_physics.assert_called_with(False)
    component.set_collision_profile_name.assert_called_with("NoCollision", False)
    levels.save_current_level.assert_called_once()


def test_actual_review_02_preserves_entire_components_and_recipe_engine():
    selection_path = REAL_ROOT.parents[1] / "20261023/factory-operating-area-02/selection.json"
    if not selection_path.exists():
        pytest.skip("local shared review selection absent")
    source = json.loads((REAL_ROOT / "source-with-collision.json").read_text(encoding="utf-8"))
    candidate = json.loads((REAL_ROOT / "manifest.json").read_text(encoding="utf-8"))
    selection = json.loads(selection_path.read_text(encoding="utf-8"))
    manifest = visual.build_visual_manifest(source, candidate, selection["component_ids"])
    assert {r["source_id"] for r in manifest["components"]} == set(selection["component_ids"])
    original = {r["source_id"]: r for r in candidate["components"]}
    assert all(r["geometry_ids"] == original[r["source_id"]]["geometry_ids"] for r in manifest["components"])
    assert any("SM_AssemblyLineControl22_81" in r["source_id"] for r in manifest["components"])
    (engine,) = [
        r
        for r in manifest["components"]
        if r["source_id"].endswith("BP_SplineMovableV2_0.NODE_AddStaticMeshComponent-1")
    ]
    assert engine["canonical_transform"]["translation_m"][0] == pytest.approx(52.33265259066842)
    assert any("FactoryFloorLarge" in r["source_id"] for r in manifest["components"])


@pytest.mark.parametrize("conflict", ["Meshes/Motor.uasset", "Meshes/Motor.ubulk", "level"])
def test_target_package_or_level_conflicts_rejected_without_overwrite(tmp_path, conflict):
    source, candidate = fixture()
    manifest = visual.build_visual_manifest(source, candidate)
    manifest["dependencies"] = [{"package": "/Game/Meshes/Motor"}]
    content, target = tmp_path / "source", tmp_path / "target"
    real = content / "Meshes/Motor.uasset"
    real.parent.mkdir(parents=True)
    real.write_bytes(b"real")
    relative = manifest["level_path"].removeprefix("/Game/") + ".umap" if conflict == "level" else conflict
    occupied = target / relative
    occupied.parent.mkdir(parents=True)
    occupied.write_bytes(b"preserve")
    with pytest.raises(FileExistsError, match="conflict"):
        visual.asset_file_plan(manifest, content, target, tmp_path / "engine")
    assert occupied.read_bytes() == b"preserve"


def test_editor_module_load_does_not_require_offline_numpy_or_scipy(monkeypatch):
    import builtins
    import runpy

    original_import = builtins.__import__

    def editor_import(name, *args, **kwargs):
        if name.split(".")[0] in {"numpy", "scipy", "unreal"} or name == "collision_candidate":
            raise ImportError(f"offline dependency must not load in editor entry: {name}")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", editor_import)
    assert callable(runpy.run_path(visual.__file__)["author_editor_level"])
