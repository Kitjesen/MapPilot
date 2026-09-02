# ruff: noqa: S101

"""Integration contracts for Forest_HF Blender authoring boundaries."""

from __future__ import annotations

import hashlib
import importlib
import itertools
import json
import math
import struct
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest


def _author():
    return importlib.import_module("sim.tools.worlds.forest_hf.blender_author")


def _canonical_json(value: object) -> bytes:
    return (
        json.dumps(value, sort_keys=True, indent=2, separators=(",", ": "), allow_nan=False)
        + "\n"
    ).encode("utf-8")


def _terrain_recipe(
    tmp_path: Path,
    payload: bytes,
    *,
    extent_m: tuple[float, float] = (20.0, 20.0),
    seed: int | None = None,
) -> dict[str, Any]:
    generated = tmp_path / "generated"
    generated.mkdir()
    raw_path = generated / "heightfield_u16.raw"
    raw_path.write_bytes(payload)
    artifact = {
        "path": "generated/heightfield_u16.raw",
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }
    asset_identity = [{"path": artifact["path"], "sha256": artifact["sha256"]}]
    asset_set_sha256 = hashlib.sha256(
        (
            json.dumps(
                asset_identity,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
    ).hexdigest()
    manifest = {
        "schema": "lingtu.sim.forest-asset-manifest.v1",
        "world_package": "forest_hf@2.0.0",
        "canonical_source": {
            "path": artifact["path"],
            "sample_type": "uint16",
            "endianness": "little",
            "dimensions_px": [2, 2],
            "extent_m": list(extent_m),
        },
        "artifacts": [artifact],
        "asset_set_sha256": asset_set_sha256,
    }
    if seed is not None:
        manifest["seed"] = seed
    (generated / "asset-manifest.json").write_bytes(_canonical_json(manifest))
    return {
        "terrain_source": {
            "package_root": str(tmp_path),
            "canonical_source": artifact["path"],
            "mesh": "generated/terrain.obj",
            "asset_manifest": "generated/asset-manifest.json",
            "enforce_asset_manifest": True,
            "dimensions_px": [2, 2],
            "sample_type": "uint16",
            "endianness": "little",
            "row_order": "north_to_south",
            "extent_m": list(extent_m),
            "elevation_range_m": [-10.0, 70.0],
        }
    }


def _materialized_production_recipe(author: Any, tmp_path: Path) -> dict[str, Any]:
    recipe = author.load_forest_recipe(author.DEFAULT_RECIPE)
    extent_m = tuple(recipe["terrain_source"]["extent_m"])
    fixture = _terrain_recipe(
        tmp_path,
        b"\x00\x80" * 4,
        extent_m=extent_m,
        seed=recipe["seed"],
    )
    recipe["terrain_source"] = fixture["terrain_source"]
    return recipe


def _write_glb(path: Path, document: dict[str, Any]) -> None:
    encoded = json.dumps(document, separators=(",", ":")).encode("utf-8")
    encoded += b" " * (-len(encoded) % 4)
    total_length = 12 + 8 + len(encoded)
    path.write_bytes(
        b"glTF"
        + struct.pack("<II", 2, total_length)
        + struct.pack("<II", len(encoded), 0x4E4F534A)
        + encoded
    )


def _distance_to_segment(point: tuple[float, float], start: list[float], end: list[float]) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return math.dist(point, start)
    ratio = min(1.0, max(0.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / denominator))
    return math.dist(point, (start[0] + ratio * dx, start[1] + ratio * dy))


def test_production_recipe_requires_the_canonical_terrain_asset_manifest() -> None:
    author = _author()

    recipe = author.load_forest_recipe(author.DEFAULT_RECIPE)

    assert recipe["terrain_height_mode"] == "canonical_package"
    assert recipe["terrain_source"]["enforce_asset_manifest"] is True
    assert recipe["terrain_source"]["asset_manifest"] == "generated/asset-manifest.json"
    assert recipe["terrain_source"].get("mesh") is None


def test_production_recipe_removes_the_duplicate_closed_loop_trail_endpoint() -> None:
    author = _author()

    recipe = author.load_forest_recipe(author.DEFAULT_RECIPE)

    centerline = recipe["trail"]["centerline_m"]
    assert centerline[0] != centerline[-1]


def test_production_forest_has_enough_route_visible_canopy() -> None:
    author = _author()
    recipe = author.load_forest_recipe(author.DEFAULT_RECIPE)

    assert recipe["tree_count"] >= 1200


def test_production_layout_places_a_visible_tree_belt_near_the_trailhead(
    tmp_path: Path,
) -> None:
    author = _author()
    recipe = _materialized_production_recipe(author, tmp_path)

    layout = author.generate_forest_layout(recipe)
    trailhead = layout["trail"]["centerline_m"][0]
    nearby_trees = [
        tree
        for tree in layout["instances"]
        if math.dist(tree["position_m"][:2], trailhead) <= 100.0
    ]

    assert len(nearby_trees) >= 20


def test_canonical_heightfield_accepts_exact_manifest_identity_and_shape(tmp_path: Path) -> None:
    author = _author()
    payload = b"".join(value.to_bytes(2, "little") for value in (65535, 32768, 0, 16384))
    recipe = _terrain_recipe(tmp_path, payload)

    samples, width, height = author._load_canonical_heightfield(recipe)

    assert [width, height] == [2, 2]
    assert list(samples) == [65535, 32768, 0, 16384]


def test_add_terrain_builds_visual_mesh_directly_from_canonical_raw_without_obj(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    author = _author()
    payload = b"".join(value.to_bytes(2, "little") for value in (65535, 32768, 0, 16384))
    recipe = _terrain_recipe(tmp_path, payload)
    recipe["terrain_height_mode"] = "canonical_package"

    class FakeMesh:
        def __init__(self) -> None:
            self.vertices: list[tuple[float, float, float]] = []
            self.faces: list[tuple[int, ...]] = []
            self.materials: list[object] = []
            self.polygons: list[SimpleNamespace] = []

        def from_pydata(self, vertices: list[tuple[float, float, float]], _edges: list[object], faces: list[tuple[int, ...]]) -> None:
            self.vertices = vertices
            self.faces = faces
            self.polygons = [SimpleNamespace(use_smooth=False) for _face in faces]

    class FakeObject(dict[str, object]):
        pass

    mesh = FakeMesh()
    obj = FakeObject()
    linked: list[FakeObject] = []
    fake_bpy = SimpleNamespace(
        data=SimpleNamespace(
            meshes=SimpleNamespace(new=lambda _name: mesh),
            objects=SimpleNamespace(new=lambda _name, _mesh: obj),
        )
    )
    collection = SimpleNamespace(objects=SimpleNamespace(link=linked.append))
    monkeypatch.setattr(author, "bpy", fake_bpy)
    monkeypatch.setattr(author, "blender_terrain_preview_dimensions", lambda: (2, 2))
    monkeypatch.setattr(author, "_assign_planar_uv", lambda *_args: None)

    terrain = author._add_terrain(recipe, {"extent_m": [1000.0, 1000.0]}, collection, object())

    assert terrain is obj
    assert linked == [obj]
    assert len(mesh.vertices) == 4
    assert [vertex[2] for vertex in mesh.vertices] == pytest.approx([-10.0, 10.0003051804, 70.0, 30.0006103609])
    assert obj["canonical_source"] == "generated/heightfield_u16.raw"
    assert obj["classification"] == "VisualOnly"
    assert obj["collision_profile"] == "NoCollision"
    assert all(polygon.use_smooth for polygon in mesh.polygons)


def _author_trail_mesh(
    author: Any,
    monkeypatch: pytest.MonkeyPatch,
    centerline: list[list[float]],
) -> Any:
    class UVLayers:
        def __init__(self) -> None:
            self.layer: SimpleNamespace | None = None

        def new(self, *, name: str) -> SimpleNamespace:
            self.layer = SimpleNamespace(
                name=name,
                data=[],
            )
            return self.layer

    class FakeMesh:
        def __init__(self) -> None:
            self.vertices: list[SimpleNamespace] = []
            self.faces: list[tuple[int, ...]] = []
            self.loops: list[SimpleNamespace] = []
            self.materials: list[object] = []
            self.uv_layers = UVLayers()

        def from_pydata(
            self,
            vertices: list[tuple[float, float, float]],
            _edges: list[object],
            faces: list[tuple[int, ...]],
        ) -> None:
            self.vertices = [
                SimpleNamespace(co=SimpleNamespace(x=x, y=y, z=z))
                for x, y, z in vertices
            ]
            self.faces = faces
            self.loops = [
                SimpleNamespace(index=index, vertex_index=vertex_index)
                for index, vertex_index in enumerate(
                    vertex_index for face in faces for vertex_index in face
                )
            ]
            self.uv_layers.layer = None

    class FakeObject(dict[str, object]):
        pass

    mesh = FakeMesh()
    obj = FakeObject()

    def add_uv_layer(*, name: str) -> SimpleNamespace:
        mesh.uv_layers.layer = SimpleNamespace(
            name=name,
            data=[SimpleNamespace(uv=None) for _loop in mesh.loops],
        )
        return mesh.uv_layers.layer

    mesh.uv_layers.new = add_uv_layer
    fake_bpy = SimpleNamespace(
        data=SimpleNamespace(
            meshes=SimpleNamespace(new=lambda _name: mesh),
            objects=SimpleNamespace(new=lambda _name, _mesh: obj),
        )
    )
    collection = SimpleNamespace(objects=SimpleNamespace(link=lambda _obj: None))
    monkeypatch.setattr(author, "_require_bpy", lambda: fake_bpy)
    monkeypatch.setattr(author, "_canonical_height_sampler", lambda _recipe: lambda _x, _y: 0.0)
    author._add_trail(
        {"terrain_height_mode": "canonical_package"},
        {
            "seed": 5808,
            "trail": {
                "centerline_m": centerline,
                "clearance_radius_m": 4.0,
                "closed_loop": False,
            },
        },
        collection,
        object(),
    )
    return mesh


def test_trail_mesh_subdivides_every_segment_to_at_most_ten_metres(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    author = _author()

    mesh = _author_trail_mesh(
        author,
        monkeypatch,
        [[0.0, 0.0], [125.0, 0.0], [125.0, 37.0]],
    )
    centers = [
        (
            (mesh.vertices[index].co.x + mesh.vertices[index + 1].co.x) / 2.0,
            (mesh.vertices[index].co.y + mesh.vertices[index + 1].co.y) / 2.0,
        )
        for index in range(0, len(mesh.vertices), 2)
    ]

    assert max(math.dist(first, second) for first, second in itertools.pairwise(centers)) <= 10.0


def test_trail_uv_uses_accumulated_metres_and_equal_world_scale(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    author = _author()

    mesh = _author_trail_mesh(
        author,
        monkeypatch,
        [[0.0, 0.0], [30.0, 0.0], [30.0, 40.0]],
    )
    uv_by_vertex: dict[int, tuple[float, float]] = {}
    for loop in mesh.loops:
        uv_by_vertex.setdefault(loop.vertex_index, tuple(mesh.uv_layers.layer.data[loop.index].uv))
    centers = [
        (
            (mesh.vertices[index].co.x + mesh.vertices[index + 1].co.x) / 2.0,
            (mesh.vertices[index].co.y + mesh.vertices[index + 1].co.y) / 2.0,
        )
        for index in range(0, len(mesh.vertices), 2)
    ]
    longitudinal = [uv_by_vertex[index][0] for index in range(0, len(mesh.vertices), 2)]
    transverse_span = abs(uv_by_vertex[0][1] - uv_by_vertex[1][1])
    world_width = math.dist(
        (mesh.vertices[0].co.x, mesh.vertices[0].co.y),
        (mesh.vertices[1].co.x, mesh.vertices[1].co.y),
    )
    metres_per_u = [
        math.dist(first, second) / (second_u - first_u)
        for first, second, first_u, second_u in zip(
            centers,
            centers[1:],
            longitudinal,
            longitudinal[1:],
        )
    ]

    assert longitudinal[-1] > 1.0
    assert max(metres_per_u) - min(metres_per_u) <= 1e-6
    assert longitudinal[-1] / transverse_span == pytest.approx(70.0 / world_width)


def test_canonical_heightfield_rejects_same_size_content_drift(tmp_path: Path) -> None:
    author = _author()
    payload = b"".join(value.to_bytes(2, "little") for value in (1, 2, 3, 4))
    recipe = _terrain_recipe(tmp_path, payload)
    raw_path = tmp_path / "generated/heightfield_u16.raw"
    raw_path.write_bytes(b"".join(value.to_bytes(2, "little") for value in (4, 3, 2, 1)))

    with pytest.raises(ValueError, match="artifact identity mismatch"):
        author._load_canonical_heightfield(recipe)


def test_canonical_heightfield_rejects_manifest_shape_mismatch(tmp_path: Path) -> None:
    author = _author()
    recipe = _terrain_recipe(tmp_path, b"\x00\x00" * 4)
    manifest_path = tmp_path / "generated/asset-manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["canonical_source"]["dimensions_px"] = [3, 3]
    manifest["validation_resolution"] = True
    manifest_path.write_bytes(_canonical_json(manifest))

    with pytest.raises(ValueError, match="expected 18"):
        author._load_canonical_heightfield(recipe)


def test_all_visual_placements_exclude_every_route_and_clearing(tmp_path: Path) -> None:
    author = _author()
    recipe = _materialized_production_recipe(author, tmp_path)

    layout = author.generate_forest_layout(recipe)

    radii = {"rock": 0.75, "fallen_log": 1.25, "understory": 0.35}
    placements = [
        (item, float(item["crown_radius_m"])) for item in layout["instances"]
    ] + [
        (item, radii[item["kind"]] * float(item["scale"])) for item in layout["dressing"]
    ]
    for item, radius in placements:
        point = tuple(item["position_m"][:2])
        for corridor in recipe["route_corridors"]:
            distances = [
                _distance_to_segment(point, start, end)
                for start, end in zip(corridor["centerline_m"], corridor["centerline_m"][1:])
            ]
            assert min(distances) >= float(corridor["clearance_radius_m"]) + radius
        for clearing in recipe["exclusion_zones"]:
            assert math.dist(point, clearing["center_m"]) >= float(clearing["radius_m"]) + radius


def test_scene_export_selection_excludes_template_collection(monkeypatch: pytest.MonkeyPatch) -> None:
    author = _author()

    class FakeObject:
        def __init__(self, name: str) -> None:
            self.name = name
            self.selected = False
            self.hidden = True

        def hide_set(self, hidden: bool) -> None:
            self.hidden = hidden

        def select_set(self, selected: bool) -> None:
            self.selected = selected

    scene_names = ("Terrain", "Trail", "Trees", "Dressing", "Lighting", "Cameras")
    scene_objects = {name: FakeObject(name) for name in scene_names}
    template = FakeObject("LT_Forest_Asset_Pine")
    collections = {
        **{name: SimpleNamespace(all_objects=[obj]) for name, obj in scene_objects.items()},
        "Templates": SimpleNamespace(all_objects=[template]),
    }
    fake_bpy = SimpleNamespace(
        ops=SimpleNamespace(object=SimpleNamespace(select_all=lambda **_kwargs: None))
    )
    monkeypatch.setattr(author, "bpy", fake_bpy)

    author._select_scene_export_objects(collections)

    assert all(item.selected and not item.hidden for item in scene_objects.values())
    assert template.selected is False
    assert template.hidden is True


def test_exported_glb_rejects_template_mother_mesh_nodes(tmp_path: Path) -> None:
    author = _author()
    path = tmp_path / "scene.glb"
    _write_glb(
        path,
        {
            "asset": {"version": "2.0"},
            "nodes": [{"name": "LT_Forest_Asset_Pine", "mesh": 0}],
            "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}],
            "materials": [],
        },
    )

    with pytest.raises(ValueError, match="excluded template nodes"):
        author.validate_exported_glb(path)


def test_each_asset_slot_glb_is_validated_with_template_name_exception_only(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    author = _author()

    class FakeObject:
        def __init__(self) -> None:
            self.hide_render = True
            self.hide_viewport = True
            self.children_recursive: tuple[object, ...] = ()
            self.hidden = True

        def hide_get(self) -> bool:
            return self.hidden

        def hide_set(self, hidden: bool) -> None:
            self.hidden = hidden

        def select_set(self, _selected: bool) -> None:
            return None

    objects = {object_name: FakeObject() for _, object_name, _, _, _ in author._FALLBACK_ASSET_SLOTS}

    def write_export(*, filepath: str, **_kwargs: object) -> set[str]:
        Path(filepath).write_bytes(b"export")
        return {"FINISHED"}

    fake_bpy = SimpleNamespace(
        data=SimpleNamespace(objects=SimpleNamespace(get=objects.get)),
        ops=SimpleNamespace(
            object=SimpleNamespace(select_all=lambda **_kwargs: None),
            export_scene=SimpleNamespace(gltf=write_export, fbx=write_export),
        ),
        context=SimpleNamespace(view_layer=SimpleNamespace(objects=SimpleNamespace(active=None))),
    )
    validations: list[tuple[Path, bool]] = []
    monkeypatch.setattr(author, "bpy", fake_bpy)
    monkeypatch.setattr(
        author,
        "validate_exported_glb",
        lambda path, *, forbid_template_nodes=True, **_kwargs: validations.append(
            (Path(path), forbid_template_nodes)
        ),
    )

    author._export_asset_slot_files(tmp_path)

    assert len(validations) == len(author._FALLBACK_ASSET_SLOTS)
    assert all(path.suffix == ".glb" and forbid is False for path, forbid in validations)


def test_exported_glb_requires_uvs_for_textured_primitives(tmp_path: Path) -> None:
    author = _author()
    path = tmp_path / "scene.glb"
    _write_glb(
        path,
        {
            "asset": {"version": "2.0"},
            "nodes": [{"name": "VisualOnly_ForestTerrain", "mesh": 0}],
            "meshes": [
                {"primitives": [{"attributes": {"POSITION": 0}, "material": 0}]}
            ],
            "materials": [
                {"pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}
            ],
            "textures": [{"source": 0}],
            "images": [{"uri": "data:image/png;base64,UE5H"}],
        },
    )

    with pytest.raises(ValueError, match="missing TEXCOORD_0"):
        author.validate_exported_glb(path)


def test_exported_glb_requires_at_least_one_textured_primitive(tmp_path: Path) -> None:
    author = _author()
    path = tmp_path / "scene.glb"
    _write_glb(
        path,
        {
            "asset": {"version": "2.0"},
            "nodes": [{"name": "VisualOnly_ForestTerrain", "mesh": 0}],
            "meshes": [
                {"primitives": [{"attributes": {"POSITION": 0, "TEXCOORD_0": 1}}]}
            ],
            "materials": [{}],
        },
    )

    with pytest.raises(ValueError, match="at least one textured primitive"):
        author.validate_exported_glb(path)


def test_generated_material_contracts_cover_every_actual_helper_material() -> None:
    author = _author()
    material_ids = tuple(author.procedural_material_specs())

    records = author.generated_material_contracts(material_ids)

    assert {item["id"] for item in records} == set(material_ids)
    for material in records:
        assert set(material["textures"]) == {"base_color", "normal", "orm"}
        for texture in material["textures"].values():
            assert texture["path"].endswith(".png")
            assert texture["source"].startswith("repo://")


def test_manifest_hashes_every_generated_helper_texture(tmp_path: Path) -> None:
    author = _author()
    recipe = _materialized_production_recipe(author, tmp_path)
    material_ids = tuple(author.procedural_material_specs())
    recipe["materials"] = author.generated_material_contracts(material_ids)
    layout = author.generate_forest_layout(recipe)
    artifacts = []
    expected_paths = set()
    for material in recipe["materials"]:
        for channel, texture in material["textures"].items():
            payload = f"{material['id']}:{channel}:png".encode()
            expected_paths.add(texture["path"])
            artifacts.append(
                {
                    "role": f"texture_{material['id']}_{channel}",
                    "path": texture["path"],
                    "bytes": len(payload),
                    "sha256": hashlib.sha256(payload).hexdigest(),
                }
            )

    manifest = author.build_authoring_manifest(
        recipe, layout, artifacts, require_material_artifacts=True
    )

    actual_paths = {
        texture["path"]
        for material in manifest["materials"]
        for texture in material["textures"].values()
    }
    assert actual_paths == expected_paths
    assert all(
        texture["bytes"] > 0 and len(texture["sha256"]) == 64
        for material in manifest["materials"]
        for texture in material["textures"].values()
    )


def test_helper_tree_mothers_are_the_exported_asset_slot_objects() -> None:
    author = _author()
    helper_specs = author.procedural_asset_specs()
    slot_names = {
        slot["semantic_class"]: slot["object_name"]
        for slot in author.build_fallback_asset_contract()["slots"]
    }

    assert helper_specs["pine"]["object_name"] == slot_names["pine"]
    assert helper_specs["birch"]["object_name"] == slot_names["birch"]


def test_planar_uv_contract_assigns_texcoords_to_every_mesh_loop() -> None:
    author = _author()

    class UVLayers:
        def __init__(self, loop_count: int) -> None:
            self.layer = None
            self.loop_count = loop_count

        def get(self, _name: str):
            return self.layer

        def new(self, *, name: str):
            self.layer = SimpleNamespace(
                name=name,
                data=[SimpleNamespace(uv=None) for _ in range(self.loop_count)],
            )
            return self.layer

    mesh = SimpleNamespace(
        vertices=[SimpleNamespace(co=SimpleNamespace(x=-10.0, y=-5.0)), SimpleNamespace(co=SimpleNamespace(x=10.0, y=5.0))],
        loops=[SimpleNamespace(index=0, vertex_index=0), SimpleNamespace(index=1, vertex_index=1)],
        uv_layers=UVLayers(2),
    )

    author._assign_planar_uv(mesh, 10.0, 5.0)

    assert mesh.uv_layers.layer.name == "UVMap"
    assert mesh.uv_layers.layer.data[0].uv == pytest.approx((0.0, 0.0))
    assert mesh.uv_layers.layer.data[1].uv == pytest.approx((1.0, 1.0))


def test_manifest_requires_an_actual_artifact_for_every_material_texture(tmp_path: Path) -> None:
    author = _author()
    recipe = _materialized_production_recipe(author, tmp_path)
    layout = author.generate_forest_layout(recipe)

    with pytest.raises(ValueError, match="has no generated artifact"):
        author.build_authoring_manifest(recipe, layout, [], require_material_artifacts=True)


def test_manifest_records_actual_texture_path_size_and_hash(tmp_path: Path) -> None:
    author = _author()
    recipe = _materialized_production_recipe(author, tmp_path)
    layout = author.generate_forest_layout(recipe)
    artifacts = []
    expected: dict[str, tuple[int, str]] = {}
    for material in recipe["materials"]:
        for channel, texture in material["textures"].items():
            payload = f"{material['id']}:{channel}".encode()
            digest = hashlib.sha256(payload).hexdigest()
            expected[texture["path"]] = (len(payload), digest)
            artifacts.append(
                {
                    "role": f"texture_{material['id']}_{channel}",
                    "path": texture["path"],
                    "bytes": len(payload),
                    "sha256": digest,
                }
            )

    manifest = author.build_authoring_manifest(
        recipe, layout, artifacts, require_material_artifacts=True
    )

    for material in manifest["materials"]:
        for texture in material["textures"].values():
            size, digest = expected[texture["path"]]
            assert texture["bytes"] == size
            assert texture["sha256"] == digest


def test_blender_command_uses_factory_startup_before_background_authoring(tmp_path: Path) -> None:
    author = _author()

    command = author.build_blender_command(
        "D:/Development/Blender/5.2/blender.exe",
        repo_root=tmp_path,
        recipe="sim/packages/worlds/forest_hf/2.0.0/terrain.recipe.json",
        output_dir="build/forest-hf/blender-v2",
        width=960,
        height=540,
        samples=16,
    )

    assert command[:5] == [
        "D:\\Development\\Blender\\5.2\\blender.exe",
        "--factory-startup",
        "--background",
        "--python",
        str((tmp_path / "sim/tools/worlds/forest_hf/blender_author.py").resolve()),
    ]
    assert command[5] == "--"
    assert command[-6:] == ["--width", "960", "--height", "540", "--samples", "16"]
