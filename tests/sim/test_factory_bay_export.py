"""Behavioral tests of the source boundary; no Unreal/Blender/build process."""

import copy
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from sim.tools.worlds.factory_environment_collection import bay_export as export


def vector(x=0, y=0, z=0):
    return SimpleNamespace(x=x, y=y, z=z)


def transform(x=4000, y=0, z=0, *, scale=(1, 1, 1), quat=(0, 0, 0, 1)):
    return SimpleNamespace(
        translation=vector(x, y, z),
        scale3d=vector(*scale),
        rotation=SimpleNamespace(**dict(zip(("x", "y", "z", "w"), quat))),
    )


class Source:
    """Source objects have read methods ONLY: unintended mutation fails loudly."""

    def __init__(self, path, **properties):
        self.path = path
        self.properties = properties

    def get_path_name(self):
        return self.path

    def get_editor_property(self, name):
        if name not in self.properties:
            raise AttributeError(f"unexposed property {name}")
        return self.properties[name]

    def get_class(self):
        return SimpleNamespace(
            get_name=lambda: type(self).__name__, get_path_name=lambda: "/Script/Engine." + type(self).__name__
        )


class Scene(Source):
    def __init__(self, path, *, pose=None, center=None, extent=None, **properties):
        super().__init__(path, **properties)
        self.pose = pose or transform()
        self.center = center or self.pose.translation
        self.extent = extent or vector(20, 20, 20)

    def get_world_transform(self):
        return self.pose


class Static(Scene):
    def __init__(self, path, mesh, *, materials=(), **kwargs):
        super().__init__(path, static_mesh=mesh, visible=True, **kwargs)
        self.materials = materials

    def get_num_materials(self):
        return len(self.materials)

    def get_material(self, index):
        return self.materials[index]

    def get_collision_enabled(self):
        return "QUERY_AND_PHYSICS"

    def get_collision_profile_name(self):
        return "BlockAll"


class Instanced(Static):
    def __init__(self, *args, instances=(), **kwargs):
        super().__init__(*args, **kwargs)
        self.instances = instances
        self.transform_reads = []

    def get_instance_count(self):
        return len(self.instances)

    def get_instance_transform(self, index, *, world_space):
        assert world_space is True
        self.transform_reads.append(index)
        return self.instances[index]


class Spline(Static):
    def get_start_position(self):
        return vector(0, 0, 0)

    def get_end_position(self):
        return vector(500, 10, 0)


class Skeletal(Scene):
    def __init__(self, path, mesh, physics=None, **kwargs):
        super().__init__(path, physics_asset_override=physics, visible=True, **kwargs)
        self.mesh = mesh

    def get_skeletal_mesh_asset(self):
        return self.mesh

    def get_num_materials(self):
        return 0

    def get_bone_transform(self, bone, space):
        assert (bone, space) == ("arm", "WORLD")
        return transform(4100, 200, 700)


class Actor(Source):
    def __init__(self, name, components, *, center=None, extent=None):
        super().__init__(export.SOURCE_MAP + ".Demonstration:PersistentLevel." + name)
        self.components = components
        self.center = center or vector(4000, 0, 100)
        self.extent = extent or vector(100, 100, 100)

    def get_actor_label(self):
        return "display label not identity"

    def get_components_by_class(self, cls):
        assert cls is Scene
        return self.components

    def get_actor_bounds(self, only_colliding):
        assert only_colliding is False
        return self.center, self.extent


class Mesh(Source):
    def get_bounds(self):
        return SimpleNamespace(origin=vector(), box_extent=vector(10, 20, 30))


class Registry:
    def __init__(self, graph=None, classes=None):
        self.graph = graph or {}
        self.classes = classes or {}
        self.queries = []

    def get_assets_by_package_name(self, package):
        self.queries.append(("metadata", package))
        cls = self.classes.get(package, "StaticMesh")
        return [] if cls is None else [SimpleNamespace(asset_class_path=SimpleNamespace(asset_name=cls))]

    def get_dependencies(self, package, options):
        self.queries.append(("dependencies", package))
        return self.graph.get(package, [])


class Transient:
    def __init__(self):
        self.properties = {"errors": []}

    def set_editor_property(self, name, value):
        self.properties[name] = value

    def get_editor_property(self, name):
        return self.properties[name]


def fake_unreal(actors=(), registry=None):
    logs, calls = [], []

    def load_map(path):
        calls.append(("load_map", path))
        return Source(export.SOURCE_MAP + ".Demonstration")

    return SimpleNamespace(
        Paths=SimpleNamespace(get_project_file_path=lambda: export.SOURCE_PROJECT),
        EditorLoadingAndSavingUtils=SimpleNamespace(load_map=load_map),
        EditorActorSubsystem=object(),
        SceneComponent=Scene,
        StaticMeshComponent=Static,
        SkeletalMeshComponent=Skeletal,
        InstancedStaticMeshComponent=Instanced,
        SplineMeshComponent=Spline,
        SystemLibrary=SimpleNamespace(get_component_bounds=lambda c: (c.center, c.extent, 0)),
        get_editor_subsystem=lambda _: SimpleNamespace(get_all_level_actors=lambda: list(actors)),
        AssetRegistryDependencyOptions=Transient,
        AssetRegistryHelpers=SimpleNamespace(get_asset_registry=lambda: registry or Registry()),
        RelativeTransformSpace=SimpleNamespace(RTS_WORLD="WORLD"),
        log=logs.append,
        logs=logs,
        calls=calls,
    )


def intake(actors):
    return {
        "source_project": export.SOURCE_PROJECT,
        "source_world": export.SOURCE_MAP + ".Demonstration",
        "bounds_min_cm": export.BAY_MIN[:],
        "bounds_max_cm": export.BAY_MAX[:],
        "selected_actor_count": len(actors),
        "mesh_assets": ["/Game/Meshes/Selected.Selected"],
        "actors": [{"source_actor": a.get_path_name(), "crosses_bay_boundary": False} for a in actors],
    }


def empty_agg(**kwargs):
    return Source(
        "agg",
        **{
            name: kwargs.get(name, [])
            for name in (
                "sphere_elems",
                "box_elems",
                "sphyl_elems",
                "convex_elems",
                "tapered_capsule_elems",
                "level_set_elems",
                "skinned_level_set_elems",
                "ml_level_set_elems",
                "skinned_triangle_mesh_elems",
            )
        },
    )


def body(agg=None, **kwargs):
    return Source(
        "/Game/Meshes/Selected.Selected:BodySetup",
        agg_geom=agg or empty_agg(),
        collision_trace_flag="CTF_UseComplexAsSimple",
        **kwargs,
    )


@pytest.mark.parametrize(
    ("low", "high", "expected"),
    [
        ([4000, 0, 0], [4020, 20, 50], "selected_for_authoring_review_not_qualified"),
        ([2000, 0, 0], [4000, 100, 10], "crossing_pending_review_no_clip_no_migration"),
        ([8000, 0, 0], [9000, 100, 10], "outside_bay"),
        ([7000, 0, 0], [7000, 10, 10], "selected_for_authoring_review_not_qualified"),
    ],
)
def test_dispositions_do_not_clip(low, high, expected):
    before = copy.deepcopy((low, high))
    assert export.disposition(low, high) == expected
    assert (low, high) == before


def test_transformed_visual_bounds_include_rotation_negative_and_nonuniform_scale():
    pose = export.transform_record(transform(4000, 0, 0, scale=(-2, 3, 1), quat=(0, 0, math.sqrt(0.5), math.sqrt(0.5))))
    low, high = export.transformed_bounds([-10, -20, -30], [10, 20, 30], pose)
    assert low == pytest.approx([3940, -20, -30])
    assert high == pytest.approx([4060, 20, 30])


def test_collision_serializes_actual_shapes_not_aabb_and_preserves_native_units():
    rotation = SimpleNamespace(pitch=10, yaw=20, roll=30)
    box = Source(
        "box", center=vector(1, 2, 3), rotation=rotation, x=20, y=40, z=60, collision_enabled="QUERY_AND_PHYSICS"
    )
    sphere = Source("sphere", center=vector(), radius=4, collision_enabled="QUERY_ONLY")
    capsule = Source("capsule", center=vector(), rotation=rotation, radius=5, length=11)
    convex = Source(
        "hull",
        vertex_data=[vector(), vector(1), vector(0, 1), vector(0, 0, 1)],
        index_data=[0, 1, 2],
        transform=transform(1, 2, 3),
    )
    report = export.collision_body(
        body(empty_agg(box_elems=[box], sphere_elems=[sphere], sphyl_elems=[capsule], convex_elems=[convex]))
    )
    shapes = {s["kind"]: s["geometry"]["value"] for s in report["shapes"]}
    assert shapes["box"]["full_size_cm"] == [20, 40, 60]
    assert shapes["box"]["center_cm"] == [1, 2, 3]
    assert shapes["capsule_z"]["segment_length_cm"] == 11
    assert shapes["sphere"]["radius_cm"] == 4
    assert shapes["convex"]["element_transform"]["value"]["translation_cm"] == [1, 2, 3]
    assert len(shapes["convex"]["vertices_cm"]) == 4
    assert report["collision_trace_flag"]["value"] == "CTF_UseComplexAsSimple"
    assert report["qualified_collision"] is False and report["visual_aabb_used"] is False


def test_missing_collision_exposure_is_unavailable_never_fabricated():
    report = export.object_collision(Source("mesh"))
    assert report["status"] == "unavailable"
    assert "body_setup" in report["reason"]
    assert "shapes" not in report
    assert report["qualified_collision"] is False
    assert export.collision_body(None)["status"] == "absent"


def test_partial_convex_access_does_not_invent_identity_transform():
    convex = Source("hull", vertex_data=[vector()], index_data=[])
    report = export.collision_body(body(empty_agg(convex_elems=[convex])))
    geometry = report["shapes"][0]["geometry"]
    assert geometry["status"] == "available"
    assert geometry["value"]["vertices_cm"] == [[0, 0, 0]]
    assert geometry["value"]["element_transform"]["status"] == "unavailable"
    assert "value" not in geometry["value"]["element_transform"]


def test_dependency_closure_keeps_material_texture_excludes_builtdata_maps_and_gameplay():
    registry = Registry(
        {
            "/Game/Meshes/Selected": [
                "/Game/Materials/M",
                "/Game/Maps/Demonstration_BuiltData",
                "/Game/Maps/Demonstration",
                "/Game/Blueprints/PW_ForkLift",
                "/Game/Blueprints/OtherLogic",
                "/Script/Engine",
            ],
            "/Game/Materials/M": ["/Game/Textures/T", "/Game/Meshes/Selected"],
            "/Game/Blueprints/OtherLogic": ["/Game/Unwanted/WholeWorld"],
        },
        {
            "/Game/Materials/M": "MaterialInstanceConstant",
            "/Game/Textures/T": "Texture2D",
            "/Game/Blueprints/OtherLogic": "Blueprint",
        },
    )
    result = export.dependency_manifest(registry, object(), {"/Game/Meshes/Selected.Selected"})
    rows = {r["package"]: r for r in result["packages"]}
    assert rows["/Game/Textures/T"]["disposition"] == "inventory_only_no_copy"
    assert rows["/Game/Maps/Demonstration_BuiltData"]["disposition"] == "baked_lighting_excluded"
    assert rows["/Game/Blueprints/OtherLogic"]["disposition"] == "non_geometry_dependency_excluded"
    assert "/Game/Unwanted/WholeWorld" not in rows
    assert ("dependencies", "/Game/Blueprints/PW_ForkLift") not in registry.queries
    assert result["complete"] is True


def test_dependency_budget_and_missing_metadata_are_explicit(monkeypatch):
    monkeypatch.setattr(export, "MAX_DEPENDENCIES", 1)
    registry = Registry({"/Game/A": ["/Game/B"]})
    result = export.dependency_manifest(registry, None, {"/Game/A.A"})
    assert result["pending_budget"] == ["/Game/B"] and result["complete"] is False
    result = export.dependency_manifest(Registry(classes={"/Game/A": None}), None, {"/Game/A.A"})
    assert result["complete"] is False
    assert result["packages"][0]["disposition"] == "unresolved"


def test_actual_component_not_actor_pose_and_stable_source_id():
    mesh = Mesh("/Game/Meshes/Selected.Selected", body_setup=body())
    component = Static(
        "component:path", mesh, pose=transform(4500, 20, 30), materials=[Source("/Game/Materials/Override.Override")]
    )
    actor = Actor("Selected", [component])
    ue, selected = fake_unreal([actor]), intake([actor])
    before = copy.deepcopy(selected)
    report, _ = export.collect_manifest(ue, selected, Source(export.SOURCE_MAP + ".Demonstration"))
    row = report["actors"][0]["components"][0]
    assert row["world_transform"]["translation_cm"] == [4500, 20, 30]
    assert row["source_id"] == "component:path"
    assert row["materials"] == ["/Game/Materials/Override.Override"]
    assert "/Game/Materials/Override.Override" in report["dependencies"]["roots"]
    assert selected == before and component.pose.translation.x == 4500
    assert report["qualified_collision"] is False and report["source_map_saved"] is False
    again, _ = export.collect_manifest(ue, selected, Source(export.SOURCE_MAP + ".Demonstration"))
    assert report == again


def test_instances_have_actual_world_transforms_and_individual_dispositions():
    mesh = Mesh("/Game/Meshes/Selected.Selected", body_setup=None)
    component = Instanced("instances", mesh, instances=[transform(4000), transform(8000), transform(7000)])
    row = export.component_record(fake_unreal(), component, {}, set(), {"instances": 0})
    assert row["kind"] == "instanced_static_mesh"
    assert [i["source_id"] for i in row["instances"]] == [
        "instances#instance:0",
        "instances#instance:1",
        "instances#instance:2",
    ]
    assert [i["world_transform"]["translation_cm"][0] for i in row["instances"]] == [4000, 8000, 7000]
    assert [i["disposition"] for i in row["instances"]] == [
        "selected_for_authoring_review_not_qualified",
        "outside_bay",
        "crossing_pending_review_no_clip_no_migration",
    ]


def test_instance_budget_fails_before_transform_reads(monkeypatch):
    monkeypatch.setattr(export, "MAX_INSTANCES", 1)
    c = Instanced("instances", Mesh("mesh"), instances=[transform(), transform()])
    with pytest.raises(RuntimeError, match="instance budget"):
        export.component_record(fake_unreal(), c, {}, set(), {"instances": 0})
    assert c.transform_reads == []


def test_failed_instance_transform_is_not_replaced_by_component_transform():
    c = Instanced("instances", Mesh("mesh"), instances=[None])
    with pytest.raises(RuntimeError, match="missing instance transform"):
        export.component_record(fake_unreal(), c, {}, set(), {"instances": 0})


def test_spline_keeps_deformation_and_existing_component_collision_separate():
    mesh = Mesh("/Game/Meshes/Spline.Spline", body_setup=body(empty_agg()))
    c = Spline("spline", mesh, body_setup=body(empty_agg()), smooth_interp_roll_scale=True)
    row = export.component_record(fake_unreal(), c, {}, set(), {"instances": 0})
    assert row["kind"] == "spline_mesh"
    assert row["spline"]["end_position"]["value"] == [500, 10, 0]
    assert row["spline"]["start_tangent"]["status"] == "unavailable"
    assert "deformed_component_local" in row["collision"]["space"]
    assert row["collision"]["qualified_collision"] is False


def test_skeletal_physics_bodies_use_bone_pose_not_static_mesh_collision():
    mesh = Source("/Game/Meshes/Arm.Arm")
    physics = Source("/Game/Physics/Arm.Arm", skeletal_body_setups=[body(bone_name="arm")])
    c = Skeletal("arm", mesh, physics)
    row = export.component_record(fake_unreal(), c, {}, set(), {"instances": 0})
    assert row["kind"] == "skeletal_mesh" and row["physics_asset"] == "/Game/Physics/Arm.Arm"
    assert row["collision"]["bodies"][0]["bone_world_transform"]["value"]["translation_cm"] == [4100, 200, 700]
    assert row["collision"]["qualified_collision"] is False


def test_original_selection_retained_and_new_skeletal_light_crossing_are_supplements():
    original = Actor("Original", [], center=vector(9000, 0, 0))
    missing = Actor("Missing", [])
    light = Actor("Light", [Scene("light", extent=vector(2000, 20, 20))])
    outside = Actor("Distant", [Scene("far", pose=transform(12000))], center=vector(12000, 0, 0))
    report, _ = export.collect_manifest(
        fake_unreal([original, light, outside]),
        intake([original, missing]),
        Source(export.SOURCE_MAP + ".Demonstration"),
    )
    rows = {r["source_id"]: r for r in report["actors"]}
    assert original.get_path_name() in rows and outside.get_path_name() not in rows
    assert rows[light.get_path_name()]["in_original_intake"] is False
    assert rows[light.get_path_name()]["components"][0]["disposition"].startswith("crossing")
    assert report["source_inventory_matches"] is False
    assert report["original_intake_missing_actors"] == [missing.get_path_name()]


@pytest.mark.parametrize("change", ["source_project", "source_world", "bounds_min_cm", "selected_actor_count"])
def test_wrong_intake_identity_rejected(change):
    selected = intake([])
    selected[change] = "other" if change.startswith("source") else [0, 0, 0] if change == "bounds_min_cm" else 42
    with pytest.raises(ValueError):
        export.validate_intake(selected)


def test_point_light_uses_component_origin_even_when_actor_bounds_are_outside():
    light = Scene("point-light", pose=transform(4000, 200, 500), center=vector(), extent=vector())
    actor = Actor("LightOnly", [light], center=vector(), extent=vector())
    report, _ = export.collect_manifest(fake_unreal([actor]), intake([]), Source("world"))
    row = report["actors"][0]["components"][0]
    assert row["source_id"] == "point-light"
    assert row["visual_bounds_min_cm"] == [4000, 200, 500]
    assert row["visual_bounds_max_cm"] == [4000, 200, 500]
    assert row["disposition"] == "selected_for_authoring_review_not_qualified"
    assert row["collision"]["status"] == "unavailable"


def test_original_mesh_drift_does_not_silently_claim_same_inventory():
    actor = Actor("Original", [Static("component", Mesh("/Game/Meshes/New.New", body_setup=None))])
    selected = intake([actor])
    selected["actors"][0]["mesh_assets"] = ["/Game/Meshes/Old.Old"]
    report, _ = export.collect_manifest(fake_unreal([actor]), selected, Source("world"))
    assert report["source_inventory_matches"] is False
    assert report["actors"][0]["original_mesh_assets_match"] is False
    assert report["original_intake_mesh_mismatches"] == [
        {
            "source_id": actor.get_path_name(),
            "expected": ["/Game/Meshes/Old.Old"],
            "observed": ["/Game/Meshes/New.New"],
        }
    ]


def test_old_forklift_actor_is_explicitly_excluded_without_component_payload_reads():
    actor = Actor("Legacy", [Static("component", Mesh("must-not-be-read"))])
    actor.get_class = lambda: Source("/Game/Blueprints/PW_ForkLift.PW_ForkLift_C")
    report, assets = export.collect_manifest(fake_unreal([actor]), intake([actor]), Source("world"))
    assert report["actors"][0]["disposition"] == "excluded_old_gameplay"
    assert report["actors"][0]["components"] == []
    assert assets == {} and report["dependencies"]["roots"] == []


@pytest.mark.parametrize(
    ("budget_name", "budget", "expected"),
    [
        ("MAX_ACTORS", 0, "actor budget"),
        ("MAX_COMPONENTS", 0, "component budget"),
    ],
)
def test_source_scan_budgets_are_enforced(monkeypatch, budget_name, budget, expected):
    monkeypatch.setattr(export, budget_name, budget)
    actor = Actor("Source", [Scene("component")])
    with pytest.raises(RuntimeError, match=expected):
        export.collect_manifest(fake_unreal([actor]), intake([actor]), Source("world"))


def test_reflection_gap_in_skeletal_body_list_is_not_a_successful_empty_body_export():
    c = Skeletal("arm", Source("/Game/Arm.Arm"), Source("/Game/ArmPhysics.ArmPhysics"))
    row = export.component_record(fake_unreal(), c, {}, set(), {"instances": 0})
    assert row["collision"]["body_access"]["status"] == "unavailable"
    assert row["collision"]["qualified_collision"] is False


def test_unavailable_fbx_options_never_prevent_the_primary_manifest(monkeypatch, tmp_path):
    actor = Actor("Source", [Static("component", Mesh("/Game/Meshes/Selected.Selected", body_setup=None))])
    ue = fake_unreal([actor])
    output = configure_main(monkeypatch, tmp_path, ue, intake([actor]))
    monkeypatch.setenv("LINGTU_FACTORY_BAY_FBX_LIMIT", "1")
    export.main()
    assert (output / "bay-export.json").exists()
    attempts = json.loads((output / "mesh-exports.json").read_text(encoding="utf-8"))
    assert attempts[0]["status"] == "unavailable_or_failed"
    assert "FbxExportOption" in attempts[0]["attempt"]["reason"]


def configure_main(monkeypatch, tmp_path, ue, selected):
    intake_path = tmp_path / "intake.json"
    intake_path.write_text(json.dumps(selected), encoding="utf-8")
    monkeypatch.setitem(sys.modules, "unreal", ue)
    monkeypatch.setenv("LINGTU_FACTORY_BAY_INTAKE", str(intake_path))
    monkeypatch.setenv("LINGTU_FACTORY_BAY_EXPORT_DIR", str(tmp_path / "new-run"))
    monkeypatch.setenv("LINGTU_FACTORY_BAY_FBX_LIMIT", "0")
    return tmp_path / "new-run"


def test_main_fake_source_boundary_writes_only_new_manifest_without_any_export_load(monkeypatch, tmp_path):
    a = Actor("Source", [Static("component", Mesh("/Game/Meshes/Selected.Selected", body_setup=None))])
    ue = fake_unreal([a])
    output = configure_main(monkeypatch, tmp_path, ue, intake([a]))
    export.main()
    assert sorted(p.name for p in output.iterdir()) == ["bay-export.json", "mesh-exports.json"]
    report = json.loads((output / "bay-export.json").read_text(encoding="utf-8"))
    assert report["source_inventory_matches"] is True
    assert report["original_intake_actor_count"] == 1
    assert report["source_assets_copied"] is False and report["qualified_collision"] is False
    assert ue.calls == [("load_map", export.SOURCE_MAP)]
    assert "saved_source=false" in ue.logs[-1]
    before = (output / "bay-export.json").read_bytes()
    with pytest.raises(FileExistsError):
        export.main()
    assert (output / "bay-export.json").read_bytes() == before
    assert ue.calls == [("load_map", export.SOURCE_MAP)]


def test_wrong_same_named_project_rejected_before_loading_world(monkeypatch, tmp_path):
    ue = fake_unreal()
    ue.Paths.get_project_file_path = lambda: "D:/elsewhere/FactoryProject.uproject"
    output = configure_main(monkeypatch, tmp_path, ue, intake([]))
    with pytest.raises(RuntimeError, match="exact isolated"):
        export.main()
    assert ue.calls == [] and not output.exists()


def test_output_inside_source_is_rejected_without_writing(monkeypatch, tmp_path):
    ue = fake_unreal()
    configure_main(monkeypatch, tmp_path, ue, intake([]))
    monkeypatch.setenv("LINGTU_FACTORY_BAY_EXPORT_DIR", "D:/epic/Projects/FactoryPreview/Export")
    with pytest.raises(ValueError, match="independent"):
        export.main()
    assert ue.calls == []


def test_bad_map_result_never_becomes_success_manifest(monkeypatch, tmp_path):
    ue = fake_unreal()
    ue.EditorLoadingAndSavingUtils.load_map = lambda _: None
    output = configure_main(monkeypatch, tmp_path, ue, intake([]))
    with pytest.raises(RuntimeError, match="did not load"):
        export.main()
    assert not (output / "bay-export.json").exists()


def test_bounded_fbx_sets_unattended_no_bake_no_collision_and_reports_disk_evidence(tmp_path):
    tasks = []

    def run(task):
        tasks.append(task.properties)
        Path(task.properties["filename"]).write_bytes(b"fake fbx")
        return True

    ue = SimpleNamespace(
        FbxExportOption=Transient,
        FbxMaterialBakeMode=SimpleNamespace(DISABLED="disabled"),
        AssetExportTask=Transient,
        StaticMeshExporterFBX=Transient,
        load_asset=lambda path: Source(path),
        Exporter=SimpleNamespace(run_asset_export_task=run),
    )
    manifest = {"mesh_assets": [{"source_id": f"/Game/Meshes/{i}.{i}", "kind": "static_mesh"} for i in range(3)]}
    result = export.export_fbx(ue, manifest, tmp_path, 1)
    assert len(tasks) == 1
    assert tasks[0]["automated"] is True and tasks[0]["prompt"] is False
    assert tasks[0]["replace_identical"] is False
    assert tasks[0]["options"].properties["bake_material_inputs"] == "disabled"
    assert tasks[0]["options"].properties["collision"] is False
    assert result[0]["status"] == "written_not_material_fidelity_verified"
    assert result[1]["status"] == "skipped_mesh_budget"
    with pytest.raises(ValueError, match=r"0\.\.100"):
        export.export_fbx(ue, manifest, tmp_path, 101)


def test_fbx_true_without_file_is_failure_and_does_not_invent_material_success(tmp_path):
    ue = SimpleNamespace(
        FbxExportOption=Transient,
        FbxMaterialBakeMode=SimpleNamespace(DISABLED=0),
        AssetExportTask=Transient,
        StaticMeshExporterFBX=Transient,
        load_asset=lambda p: Source(p),
        Exporter=SimpleNamespace(run_asset_export_task=lambda _: True),
    )
    result = export.export_fbx(ue, {"mesh_assets": [{"source_id": "mesh", "kind": "static_mesh"}]}, tmp_path, 1)
    assert result[0]["status"] == "unavailable_or_failed"


@pytest.mark.skipif(
    not export.DEFAULT_INTAKE.is_file(), reason="restricted local source evidence is not in public clones"
)
def test_actual_corrected_inventory_is_458_actors_100_meshes_and_117_crossings():
    selected = json.loads(export.DEFAULT_INTAKE.read_text(encoding="utf-8"))
    rows = export.validate_intake(selected)
    assert len(rows) == 458
    assert len(selected["mesh_assets"]) == 100
    assert sum(a["crosses_bay_boundary"] for a in rows.values()) == 117
