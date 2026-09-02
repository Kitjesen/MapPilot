# ruff: noqa: S101

"""Behavior contracts for deterministic Forest_HF Blender authoring."""

from __future__ import annotations

import hashlib
import importlib
import json
import math
import struct
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any

import pytest

from sim.catalog.importers.contracts import digest_document


def _author() -> ModuleType:
    """Import the forest authoring contract without requiring Blender."""

    return importlib.import_module("sim.tools.worlds.forest_hf.blender_author")


def _recipe(*, seed: int = 5808) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.forest-authoring-recipe.v1",
        "world_package": "forest_hf@2.0.0",
        "seed": seed,
        "extent_m": [80.0, 60.0],
        "tree_count": 32,
        "trail": {
            "centerline_m": [[-36.0, 0.0], [0.0, 4.0], [36.0, 0.0]],
            "clearance_radius_m": 3.0,
        },
        "species": [
            {
                "id": "pine",
                "weight": 0.7,
                "height_m": [5.0, 9.0],
                "crown_radius_m": [1.2, 2.1],
                "material": "pine_bark_needles",
            },
            {
                "id": "birch",
                "weight": 0.3,
                "height_m": [4.0, 7.0],
                "crown_radius_m": [1.0, 1.8],
                "material": "birch_bark_leaves",
            },
        ],
        "materials": [
            {
                "id": "pine_bark_needles",
                "shader": "principled_pbr",
                "textures": {
                    "base_color": {
                        "path": "textures/forest/pine_basecolor.png",
                        "sha256": "a" * 64,
                        "source": "LingTu procedural texture bake",
                        "license": "LicenseRef-LingTu-Project-Owned",
                    },
                    "normal": {
                        "path": "textures/forest/pine_normal.png",
                        "sha256": "b" * 64,
                        "source": "LingTu procedural texture bake",
                        "license": "LicenseRef-LingTu-Project-Owned",
                    },
                    "roughness": {
                        "path": "textures/forest/pine_roughness.png",
                        "sha256": "c" * 64,
                        "source": "LingTu procedural texture bake",
                        "license": "LicenseRef-LingTu-Project-Owned",
                    },
                },
            },
            {
                "id": "birch_bark_leaves",
                "shader": "principled_pbr",
                "textures": {
                    "base_color": {
                        "path": "textures/forest/birch_basecolor.png",
                        "sha256": "d" * 64,
                        "source": "LingTu procedural texture bake",
                        "license": "LicenseRef-LingTu-Project-Owned",
                    },
                    "normal": {
                        "path": "textures/forest/birch_normal.png",
                        "sha256": "e" * 64,
                        "source": "LingTu procedural texture bake",
                        "license": "LicenseRef-LingTu-Project-Owned",
                    },
                    "roughness": {
                        "path": "textures/forest/birch_roughness.png",
                        "sha256": "f" * 64,
                        "source": "LingTu procedural texture bake",
                        "license": "LicenseRef-LingTu-Project-Owned",
                    },
                },
            },
        ],
        "hard_rules": {
            "classification": "VisualOnly",
            "collision_profile": "NoCollision",
            "physics_authority": "mujoco",
        },
    }


def _canonical_json(value: object) -> bytes:
    return (
        json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            separators=(",", ": "),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def _asset_set_digest(records: list[dict[str, Any]]) -> str:
    identity = [{"path": item["path"], "sha256": item["sha256"]} for item in records]
    payload = (json.dumps(identity, sort_keys=True, separators=(",", ":")) + "\n").encode()
    return hashlib.sha256(payload).hexdigest()


def _materialize_validation_heightfield(recipe: dict[str, Any], root: Path) -> None:
    generated = root / "generated"
    generated.mkdir(parents=True)
    payload = b"\x00\x80" * 4
    raw = generated / "heightfield_u16.raw"
    raw.write_bytes(payload)
    artifacts = [
        {
            "path": "generated/heightfield_u16.raw",
            "bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
        }
    ]
    manifest = {
        "schema": "lingtu.sim.forest-asset-manifest.v1",
        "world_package": "forest_hf@2.0.0",
        "seed": recipe["seed"],
        "validation_resolution": True,
        "canonical_source": {
            "path": "generated/heightfield_u16.raw",
            "sample_type": "uint16",
            "endianness": "little",
            "dimensions_px": [2, 2],
            "extent_m": recipe["terrain_source"]["extent_m"],
        },
        "artifacts": artifacts,
        "asset_set_sha256": _asset_set_digest(artifacts),
    }
    (generated / "asset-manifest.json").write_bytes(_canonical_json(manifest))
    recipe["terrain_source"]["package_root"] = str(root)


def _load_recipe(
    author: ModuleType,
    tmp_path: Path,
    *,
    seed: int = 5808,
) -> dict[str, Any]:
    path = tmp_path / f"forest-{seed}.recipe.json"
    path.write_bytes(_canonical_json(_recipe(seed=seed)))
    return author.load_forest_recipe(path)


def _distance_to_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.dist(point, start)
    projection = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    ratio = min(1.0, max(0.0, projection))
    closest = (start[0] + ratio * dx, start[1] + ratio * dy)
    return math.dist(point, closest)


def _all_strings(value: object) -> list[str]:
    if isinstance(value, str):
        return [value]
    if isinstance(value, dict):
        return [item for child in value.values() for item in _all_strings(child)]
    if isinstance(value, list):
        return [item for child in value for item in _all_strings(child)]
    return []


def test_module_imports_without_bpy() -> None:
    author = _author()

    assert author.BPY_AVAILABLE is False
    assert author.WORLD_PACKAGE == "forest_hf@2.0.0"


def test_same_seed_produces_byte_identical_forest_layout(tmp_path: Path) -> None:
    author = _author()
    recipe = _load_recipe(author, tmp_path, seed=5808)

    first = author.generate_forest_layout(recipe)
    second = author.generate_forest_layout(recipe)

    assert _canonical_json(first) == _canonical_json(second)


def test_different_seed_changes_tree_layout(tmp_path: Path) -> None:
    author = _author()

    first = author.generate_forest_layout(_load_recipe(author, tmp_path, seed=5808))
    second = author.generate_forest_layout(_load_recipe(author, tmp_path, seed=5809))

    assert first["instances"] != second["instances"]


def test_loaded_recipe_preserves_material_texture_provenance(tmp_path: Path) -> None:
    author = _author()

    recipe = _load_recipe(author, tmp_path)

    assert {material["id"] for material in recipe["materials"]} == {
        "pine_bark_needles",
        "birch_bark_leaves",
    }
    for material in recipe["materials"]:
        assert material["shader"] == "principled_pbr"
        assert set(material["textures"]) == {"base_color", "normal", "roughness"}
        for texture in material["textures"].values():
            assert len(texture["sha256"]) == 64
            assert texture["source"]
            assert texture["license"]


def test_every_generated_forest_instance_is_visual_only_and_collision_free(
    tmp_path: Path,
) -> None:
    author = _author()

    layout = author.generate_forest_layout(_load_recipe(author, tmp_path))

    assert len(layout["instances"]) == 32
    for instance in layout["instances"]:
        assert instance["classification"] == "VisualOnly"
        assert instance["collision_profile"] == "NoCollision"
        assert instance["collision"] is False
        assert instance["simulate_physics"] is False
        assert instance["can_ever_affect_navigation"] is False
        assert instance["physics_representation"] == "none"


def test_layout_maps_mujoco_metres_to_unreal_centimetres(tmp_path: Path) -> None:
    author = _author()

    layout = author.generate_forest_layout(_load_recipe(author, tmp_path))
    contract = layout["coordinate_contract"]

    assert contract["source_frame"] == "mujoco_rh_z_up_m"
    assert contract["blender_frame"] == "right_handed_z_up_m"
    assert contract["unreal_position_mapping"] == "(x,y,z)m -> (100*x,-100*y,100*z)cm"
    for instance in layout["instances"]:
        x_m, y_m, z_m = instance["position_m"]
        assert instance["unreal_position_cm"] == pytest.approx([100.0 * x_m, -100.0 * y_m, 100.0 * z_m])


def test_tree_canopies_keep_the_robot_trail_clear(tmp_path: Path) -> None:
    author = _author()
    recipe = _load_recipe(author, tmp_path)

    layout = author.generate_forest_layout(recipe)
    trail = recipe["trail"]
    segments = list(zip(trail["centerline_m"], trail["centerline_m"][1:]))

    for instance in layout["instances"]:
        point = tuple(instance["position_m"][:2])
        clearance = trail["clearance_radius_m"] + instance["crown_radius_m"]
        assert min(_distance_to_segment(point, tuple(start), tuple(end)) for start, end in segments) >= clearance


def test_manifest_and_digest_cover_relative_output_artifacts(tmp_path: Path) -> None:
    author = _author()
    recipe = _load_recipe(author, tmp_path)
    layout = author.generate_forest_layout(recipe)
    artifacts = []
    for role, filename in {
        "editable_blend": "forest_hf.blend",
        "unreal_scene_fbx": "forest_hf.fbx",
        "portable_scene_glb": "forest_hf.glb",
        "robot_eye_preview": "forest_hf.robot-eye.png",
    }.items():
        payload = f"fixture:{role}".encode()
        artifacts.append(
            {
                "role": role,
                "path": filename,
                "bytes": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
            }
        )

    manifest = author.build_authoring_manifest(
        recipe,
        layout,
        artifacts,
    )

    identity = [
        {"role": item["role"], "path": item["path"], "sha256": item["sha256"]}
        for item in sorted(manifest["assets"], key=lambda item: (item["role"], item["path"]))
    ]
    assert manifest["artifact_set_digest"] == hashlib.sha256(_canonical_json(identity)).hexdigest()
    assert manifest["digest"] == digest_document({key: value for key, value in manifest.items() if key != "digest"})
    assert not any(Path(value).is_absolute() for value in _all_strings(manifest))


def test_recipe_rejects_absolute_texture_paths(tmp_path: Path) -> None:
    author = _author()
    recipe = _recipe()
    recipe["materials"][0]["textures"]["base_color"]["path"] = "C:/private/forest/pine_basecolor.png"
    path = tmp_path / "absolute-path.recipe.json"
    path.write_bytes(_canonical_json(recipe))

    with pytest.raises(ValueError, match="relative"):
        author.load_forest_recipe(path)


def test_fallback_asset_slots_are_stable_replaceable_and_procedural() -> None:
    author = _author()

    contract = author.build_fallback_asset_contract()

    assert contract["schema"] == "lingtu.sim.forest-asset-library.v1"
    assert [slot["slot_id"] for slot in contract["slots"]] == [
        "forest.asset.birch",
        "forest.asset.boulder",
        "forest.asset.pine",
    ]
    assert [slot["object_name"] for slot in contract["slots"]] == [
        "LT_Forest_Asset_Birch",
        "LT_Forest_Asset_Boulder",
        "LT_Forest_Asset_Pine",
    ]
    for slot in contract["slots"]:
        assert slot["source"]["kind"] == "procedural"
        assert slot["source"]["generator"] == "lingtu_blender_fallback"
        assert slot["replacement"]["preserve"] == ["slot_id", "object_name"]
        assert slot["export"] == {
            "path": f"assets/forest_asset_{slot['semantic_class']}.glb",
            "artifact_role": f"asset_slot_{slot['semantic_class']}_glb",
        }
        assert slot["unreal_export"] == {
            "path": f"assets/forest_asset_{slot['semantic_class']}.fbx",
            "artifact_role": f"asset_slot_{slot['semantic_class']}_fbx",
            "source_units": "metres",
            "target_units": "centimetres",
            "export_global_scale": 1.0,
            "coordinate_scale": 100.0,
        }
        assert "tripo" not in json.dumps(slot["source"]).lower()


def test_fallback_assets_define_normalized_geometry_uv_and_pbr_contracts() -> None:
    author = _author()

    first = author.build_fallback_asset_contract()
    second = author.build_fallback_asset_contract()

    assert _canonical_json(first) == _canonical_json(second)
    for slot in first["slots"]:
        assert slot["normalization"]["units"] == "metres"
        assert slot["normalization"]["up_axis"] == "+Z"
        assert slot["normalization"]["forward_axis"] == "+Y"
        assert slot["normalization"]["origin"] == "grounded_base_center"
        assert slot["normalization"]["rotation_deg"] == [0.0, 0.0, 0.0]
        assert all(value > 0.0 for value in slot["normalization"]["reference_size_m"])
        assert slot["topology"]["deterministic"] is True
        assert slot["topology"]["generator_version"] == 1
        assert slot["uv"]["map_name"] == "UVMap"
        assert slot["uv"]["deterministic"] is True
        assert slot["pbr"]["shader"] == "principled_pbr"
        assert set(slot["pbr"]["channels"]) == {"base_color", "normal", "roughness"}


def test_manifest_publishes_asset_slots_and_editable_export_contract(tmp_path: Path) -> None:
    author = _author()
    recipe = _load_recipe(author, tmp_path)
    layout = author.generate_forest_layout(recipe)

    manifest = author.build_authoring_manifest(recipe, layout, [])

    assert manifest["asset_library"] == author.build_fallback_asset_contract()
    assert manifest["export_contract"] == {
        "editable_blend": {"path": "forest_hf.blend", "editable": True},
        "portable_scene_glb": {"path": "forest_hf.glb", "editable": False},
        "unreal_scene_fbx": {"path": "forest_hf.fbx", "editable": False},
        "asset_slot_glbs": {
            "forest.asset.birch": {
                "path": "assets/forest_asset_birch.glb",
                "artifact_role": "asset_slot_birch_glb",
                "editable": False,
            },
            "forest.asset.boulder": {
                "path": "assets/forest_asset_boulder.glb",
                "artifact_role": "asset_slot_boulder_glb",
                "editable": False,
            },
            "forest.asset.pine": {
                "path": "assets/forest_asset_pine.glb",
                "artifact_role": "asset_slot_pine_glb",
                "editable": False,
            },
        },
        "asset_slot_fbxs": {
            "forest.asset.birch": {
                "path": "assets/forest_asset_birch.fbx",
                "artifact_role": "asset_slot_birch_fbx",
                "editable": False,
                "target": "unreal_static_mesh_import",
                "source_units": "metres",
                "target_units": "centimetres",
                "export_global_scale": 1.0,
                "coordinate_scale": 100.0,
            },
            "forest.asset.boulder": {
                "path": "assets/forest_asset_boulder.fbx",
                "artifact_role": "asset_slot_boulder_fbx",
                "editable": False,
                "target": "unreal_static_mesh_import",
                "source_units": "metres",
                "target_units": "centimetres",
                "export_global_scale": 1.0,
                "coordinate_scale": 100.0,
            },
            "forest.asset.pine": {
                "path": "assets/forest_asset_pine.fbx",
                "artifact_role": "asset_slot_pine_fbx",
                "editable": False,
                "target": "unreal_static_mesh_import",
                "source_units": "metres",
                "target_units": "centimetres",
                "export_global_scale": 1.0,
                "coordinate_scale": 100.0,
            },
        },
    }
    assert manifest["asset_slot_exports"] == []
    layout_slots = {instance["asset_slot_id"] for instance in layout["instances"]}
    assert layout_slots == {"forest.asset.birch", "forest.asset.pine"}
    assert {item["asset_slot_id"] for item in layout["dressing"] if item["kind"] == "rock"} == {
        "forest.asset.boulder"
    }


def test_unreal_fbx_export_preserves_metres_for_unreal_unit_conversion() -> None:
    author = _author()

    assert author._unreal_fbx_export_kwargs(use_selection=True) == {
        "use_selection": True,
        "global_scale": 1.0,
        "apply_unit_scale": False,
        "apply_scale_options": "FBX_SCALE_NONE",
        "axis_forward": "-Z",
        "axis_up": "Y",
        "use_mesh_modifiers": True,
        "add_leaf_bones": False,
        "bake_anim": False,
    }
    source = author.__file__.read_text(encoding="utf-8") if isinstance(author.__file__, Path) else Path(author.__file__).read_text(encoding="utf-8")
    assert source.count("**_unreal_fbx_export_kwargs(") == 2


def test_manifest_binds_each_asset_slot_to_the_exported_glb_digest(tmp_path: Path) -> None:
    author = _author()
    recipe = _load_recipe(author, tmp_path)
    layout = author.generate_forest_layout(recipe)
    artifacts = []
    for semantic_class in ("birch", "boulder", "pine"):
        for extension in ("glb", "fbx"):
            payload = f"standalone:{semantic_class}:{extension}".encode()
            artifacts.append(
                {
                    "role": f"asset_slot_{semantic_class}_{extension}",
                    "path": f"assets/forest_asset_{semantic_class}.{extension}",
                    "bytes": len(payload),
                    "sha256": hashlib.sha256(payload).hexdigest(),
                }
            )

    manifest = author.build_authoring_manifest(recipe, layout, artifacts)

    assert [item["slot_id"] for item in manifest["asset_slot_exports"]] == [
        "forest.asset.birch",
        "forest.asset.birch",
        "forest.asset.boulder",
        "forest.asset.boulder",
        "forest.asset.pine",
        "forest.asset.pine",
    ]
    assert {item["format"] for item in manifest["asset_slot_exports"]} == {"glb", "fbx"}
    assert all(len(item["sha256"]) == 64 for item in manifest["asset_slot_exports"])


def test_every_production_recipe_species_resolves_to_a_declared_tree_slot(tmp_path: Path) -> None:
    author = _author()
    recipe = author.load_forest_recipe(author.DEFAULT_RECIPE)
    _materialize_validation_heightfield(recipe, tmp_path)

    layout = author.generate_forest_layout(recipe)

    declared = {slot["slot_id"] for slot in author.build_fallback_asset_contract()["slots"]}
    assert {instance["species_id"] for instance in layout["instances"]} == {
        species["id"] for species in recipe["species"]
    }
    assert {instance["asset_slot_id"] for instance in layout["instances"]} <= declared
    assert all(instance["asset_slot_id"] in {"forest.asset.pine", "forest.asset.birch"} for instance in layout["instances"])


def test_default_recipe_explicitly_normalizes_the_two_kilometre_terrain_contract() -> None:
    author = _author()

    recipe = author.load_forest_recipe(author.DEFAULT_RECIPE)

    assert recipe["source_schema"] == "lingtu.sim.forest-terrain-recipe.v1"
    assert recipe["world_package"] == "forest_hf@2.0.0"
    assert recipe["extent_m"] == [1000.0, 1000.0]
    assert recipe["terrain_source"]["extent_m"] == [2000.0, 2000.0]
    assert recipe["terrain_source"]["canonical_source"] == "generated/heightfield_u16.raw"
    assert recipe["terrain_source"]["unreal_heightmap"] == "generated/heightfield_r16.png"
    assert recipe["terrain_source"]["unreal_format"] == "ue_landscape_r16_png"
    assert recipe["terrain_source"].get("mesh") is None
    assert recipe["trail"]["centerline_m"][0] == [-760.0, -650.0]


def test_canonical_layout_fails_closed_without_materialized_heightfield(tmp_path: Path) -> None:
    author = _author()
    recipe = author.load_forest_recipe(author.DEFAULT_RECIPE)
    recipe["terrain_source"]["package_root"] = str(tmp_path)

    with pytest.raises(FileNotFoundError, match="heightfield"):
        author.generate_forest_layout(recipe)


def test_ue_centimetre_left_handed_vertices_convert_to_blender_rh_metres() -> None:
    author = _author()

    converted = [
        author.ue_obj_cm_lh_to_blender_m_rh(vertex)
        for vertex in ((-100000.0, -100000.0, -1000.0), (100000.0, 100000.0, 7000.0))
    ]

    assert converted == [(-1000.0, 1000.0, -10.0), (1000.0, -1000.0, 70.0)]
    assert [min(point[0] for point in converted), max(point[0] for point in converted)] == [-1000.0, 1000.0]
    assert [min(point[1] for point in converted), max(point[1] for point in converted)] == [-1000.0, 1000.0]


def test_blender_terrain_preview_uses_the_513_source_cap() -> None:
    author = _author()

    assert author.blender_terrain_preview_dimensions() == (513, 513)


@pytest.mark.parametrize("mode", ["basecolor", "normal", "orm"])
def test_generated_texture_pixels_are_tileable_on_both_axes(mode: str) -> None:
    author = _author()
    size = 32

    pixels = author._procedural_pixels(size, 5808, mode, (0.45, 0.31, 0.18))

    def pixel(x: int, y: int) -> tuple[float, float, float, float]:
        offset = 4 * (y * size + x)
        return tuple(pixels[offset : offset + 4])

    horizontal_delta = max(
        abs(left - right)
        for y in range(size)
        for left, right in zip(pixel(0, y), pixel(size - 1, y))
    )
    vertical_delta = max(
        abs(top - bottom)
        for x in range(size)
        for top, bottom in zip(pixel(x, 0), pixel(x, size - 1))
    )

    assert horizontal_delta <= 0.02
    assert vertical_delta <= 0.02


def test_trail_uvs_repeat_every_twelve_to_sixteen_metres() -> None:
    author = _author()
    points = [[0.0, 0.0], [28.0, 0.0]]

    coordinates = author._trail_uv_coordinates(points, trail_width_m=4.0)
    inferred_tile_m = 28.0 / (coordinates[2][0] - coordinates[0][0])

    assert 12.0 <= inferred_tile_m <= 16.0


def test_terrain_uvs_repeat_every_twenty_four_to_thirty_two_metres(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    author = _author()
    uv_calls: list[dict[str, float | None]] = []

    class FakeMesh:
        def __init__(self) -> None:
            self.materials: list[object] = []
            self.polygons: list[object] = []

        def from_pydata(self, _vertices: object, _edges: object, _faces: object) -> None:
            return None

    class FakeObject(dict[str, object]):
        users_collection: list[object] = []

    fake_mesh = FakeMesh()
    fake_blender = SimpleNamespace(
        data=SimpleNamespace(
            meshes=SimpleNamespace(new=lambda _name: fake_mesh),
            objects=SimpleNamespace(new=lambda _name, _mesh: FakeObject()),
        )
    )
    collection = SimpleNamespace(objects=SimpleNamespace(link=lambda _obj: None))
    recipe = {
        "terrain_height_mode": "canonical_package",
        "terrain_source": {
            "dimensions_px": [513, 513],
            "canonical_source": "generated/heightfield_u16.raw",
        },
    }
    layout = {"extent_m": [1000.0, 1000.0]}

    monkeypatch.setattr(author, "_require_bpy", lambda: fake_blender)
    monkeypatch.setattr(author, "_canonical_height_sampler", lambda _recipe: lambda _x, _y: 0.0)
    monkeypatch.setattr(author, "blender_terrain_preview_dimensions", lambda *_args: (2, 2))
    monkeypatch.setattr(author, "_set_visual_only", lambda *_args: None)
    monkeypatch.setattr(
        author,
        "_assign_planar_uv",
        lambda _mesh, _extent_x, _extent_y, *, tile_m=None: uv_calls.append({"tile_m": tile_m}),
    )

    author._add_terrain(recipe, layout, collection, object())

    assert len(uv_calls) == 1
    assert uv_calls[0]["tile_m"] is not None
    assert 24.0 <= float(uv_calls[0]["tile_m"]) <= 32.0


def test_scene_setup_keeps_exposure_and_world_fill_in_daylight_range(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    author = _author()
    scene = SimpleNamespace(
        render=SimpleNamespace(image_settings=SimpleNamespace()),
        view_settings=SimpleNamespace(),
        collection=SimpleNamespace(children=SimpleNamespace(link=lambda _collection: None)),
    )
    background = SimpleNamespace(
        inputs={
            "Color": SimpleNamespace(default_value=None),
            "Strength": SimpleNamespace(default_value=None),
        }
    )
    world = SimpleNamespace(
        use_nodes=False,
        node_tree=SimpleNamespace(nodes={"Background": background}),
    )
    fake_blender = SimpleNamespace(
        ops=SimpleNamespace(wm=SimpleNamespace(read_factory_settings=lambda **_kwargs: None)),
        context=SimpleNamespace(scene=scene),
        data=SimpleNamespace(
            worlds=SimpleNamespace(new=lambda _name: world),
            collections=SimpleNamespace(new=lambda name: SimpleNamespace(name=name)),
        ),
    )
    monkeypatch.setattr(author, "_require_bpy", lambda: fake_blender)

    author._setup_scene(640, 360, 8)

    assert 0.5 <= scene.view_settings.exposure <= 1.25
    assert 0.4 <= background.inputs["Strength"].default_value <= 0.7


def test_outdoor_tree_and_ground_albedo_is_brightened_without_clipping() -> None:
    author = _author()

    assert author.outdoor_material_base_color("pine_needles", "#142B20") == pytest.approx(
        (40 / 255, 86 / 255, 64 / 255)
    )
    assert author.outdoor_material_base_color("pine_bark", "#2B2018") == pytest.approx(
        (86 / 255, 64 / 255, 48 / 255)
    )
    assert author.outdoor_material_base_color("forest_ground", "#303D1C") == pytest.approx(
        (96 / 255, 122 / 255, 56 / 255)
    )
    assert max(author.outdoor_material_base_color("forest_ground", "#303D1C")) < 1.0
    assert author.outdoor_material_base_color("birch_bark", "#D8D1C1") == pytest.approx(
        (216 / 255, 209 / 255, 193 / 255)
    )


def test_unknown_tree_species_fails_closed_instead_of_reusing_the_wrong_slot(tmp_path: Path) -> None:
    author = _author()
    raw = _recipe()
    raw["species"][0]["id"] = "oak"
    path = tmp_path / "unsupported-species.recipe.json"
    path.write_bytes(_canonical_json(raw))

    with pytest.raises(ValueError, match="fallback asset slot"):
        author.generate_forest_layout(author.load_forest_recipe(path))


def test_v2_recipe_preserves_every_route_corridor_and_spawn_goal_clearing() -> None:
    author = _author()

    recipe = author.load_forest_recipe(author.DEFAULT_RECIPE)

    assert len(recipe["route_corridors"]) == 4
    assert {item["stable_id"] for item in recipe["route_corridors"]} == {
        "forest.route.primary_loop.v1",
        "forest.route.branch_north.v1",
        "forest.route.branch_east.v1",
        "forest.route.branch_southwest.v1",
    }
    assert {item["id"] for item in recipe["exclusion_zones"]} == {
        "forest.spawn.trailhead.v1",
        "forest.goal.overlook.v1",
    }


def test_every_dressing_item_respects_all_route_and_clearing_clearance(tmp_path: Path) -> None:
    author = _author()
    recipe = _load_recipe(author, tmp_path)
    recipe["route_corridors"] = [
        {"stable_id": "route-a", "centerline_m": [[-36.0, 0.0], [36.0, 0.0]], "clearance_radius_m": 4.0},
        {"stable_id": "route-b", "centerline_m": [[0.0, -28.0], [0.0, 28.0]], "clearance_radius_m": 3.0},
    ]
    recipe["exclusion_zones"] = [{"id": "clearing", "center_m": [18.0, 16.0], "radius_m": 6.0}]

    layout = author.generate_forest_layout(recipe)
    radii = {"rock": 0.75, "fallen_log": 1.25, "understory": 0.35}
    for item in layout["dressing"]:
        point = tuple(item["position_m"][:2])
        radius = radii[item["kind"]] * item["scale"]
        for corridor in recipe["route_corridors"]:
            segments = list(zip(corridor["centerline_m"], corridor["centerline_m"][1:]))
            assert min(_distance_to_segment(point, tuple(a), tuple(b)) for a, b in segments) >= (
                corridor["clearance_radius_m"] + radius
            )
        assert math.dist(point, (18.0, 16.0)) >= 6.0 + radius


def test_v2_physics_proxy_is_only_the_canonical_heightfield(tmp_path: Path) -> None:
    author = _author()
    recipe = author.load_forest_recipe(author.DEFAULT_RECIPE)
    _materialize_validation_heightfield(recipe, tmp_path)

    layout = author.generate_forest_layout(recipe)

    assert layout["physics"]["proxies"] == [
        {
            "stable_id": "forest.physics.terrain",
            "shape": "heightfield",
            "extent_m": [1000.0, 1000.0],
            "source": "generated/heightfield_u16.raw",
            "authority": "mujoco",
        }
    ]


def test_canonical_raw_height_sampler_uses_north_to_south_mapping(tmp_path: Path) -> None:
    author = _author()
    generated = tmp_path / "generated"
    generated.mkdir()
    values = [65535, 32768, 0, 16384]  # north-west, north-east, south-west, south-east
    payload = b"".join(value.to_bytes(2, "little") for value in values)
    (generated / "heightfield_u16.raw").write_bytes(payload)
    recipe = {
        "terrain_source": {
            "package_root": str(tmp_path),
            "canonical_source": "generated/heightfield_u16.raw",
            "mesh": "generated/terrain.obj",
            "dimensions_px": [2, 2],
            "sample_type": "uint16",
            "endianness": "little",
            "row_order": "north_to_south",
            "extent_m": [20.0, 20.0],
            "elevation_range_m": [-10.0, 70.0],
        }
    }

    sample = author._canonical_height_sampler(recipe)

    assert sample(-10.0, 10.0) == pytest.approx(70.0)
    assert sample(-10.0, -10.0) == pytest.approx(-10.0)


def test_manifest_replaces_texture_contract_digest_with_actual_file_identity(tmp_path: Path) -> None:
    author = _author()
    recipe = _load_recipe(author, tmp_path)
    layout = author.generate_forest_layout(recipe)
    texture = recipe["materials"][0]["textures"]["base_color"]
    payload = b"actual generated texture bytes"
    artifact = {
        "role": "texture_fixture",
        "path": texture["path"],
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }

    manifest = author.build_authoring_manifest(recipe, layout, [artifact])
    recorded = manifest["materials"][0]["textures"]["base_color"]

    assert recorded["sha256"] == artifact["sha256"]
    assert recorded["bytes"] == artifact["bytes"]


def test_manifest_exposes_exact_ground_pbr_bindings_and_artifact_identity(tmp_path: Path) -> None:
    author = _author()
    recipe = _load_recipe(author, tmp_path)
    recipe["materials"] = author.generated_material_contracts(["forest_ground"])
    layout = author.generate_forest_layout(recipe)
    artifacts = []
    for channel, suffix in (("base_color", "basecolor"), ("normal", "normal"), ("orm", "orm")):
        payload = f"ground:{channel}".encode()
        artifacts.append(
            {
                "role": f"texture_forest_ground_{suffix}",
                "path": f"textures/forest/forest_ground_{suffix}.png",
                "bytes": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
            }
        )

    manifest = author.build_authoring_manifest(recipe, layout, artifacts, require_material_artifacts=True)
    ground = manifest["materials"][0]

    assert ground["bindings"] == {
        "base_color": {"texture": "base_color", "color_space": "sRGB"},
        "normal": {"texture": "normal", "color_space": "Non-Color", "compression": "NormalMap"},
        "ambient_occlusion": {"texture": "orm", "channel": "R"},
        "roughness": {"texture": "orm", "channel": "G"},
        "metallic": {"texture": "orm", "channel": "B"},
    }
    assert {name: value["bytes"] for name, value in ground["textures"].items()} == {
        "base_color": len(b"ground:base_color"),
        "normal": len(b"ground:normal"),
        "orm": len(b"ground:orm"),
    }
    assert all(len(value["sha256"]) == 64 for value in ground["textures"].values())


def test_blender_argument_delimiter_is_parsed_for_explicit_argv(tmp_path: Path) -> None:
    author = _author()

    args = author.parse_cli_args(
        ["--background", "scene.blend", "--", "--repo-root", str(tmp_path), "--validate-only"]
    )

    assert args.repo_root == tmp_path.resolve()
    assert args.validate_only is True


def test_blender_export_tempdir_is_local_readable_and_restored() -> None:
    author = _author()
    original_tempdir = author.tempfile.tempdir
    original_temporary_directory = author.tempfile.TemporaryDirectory
    work_dir = Path.cwd() / "build" / "forest-hf" / f"test-export-temp-{author.uuid.uuid4().hex}"
    work_dir.mkdir(mode=0o777, parents=True)
    try:
        with author._local_blender_export_tempdir(work_dir) as export_tempdir:
            assert export_tempdir == (work_dir / ".export-tmp").resolve()
            assert author.tempfile.tempdir == str(export_tempdir)
            assert author.tempfile.TemporaryDirectory is not original_temporary_directory
            with author.tempfile.TemporaryDirectory(prefix="unit-") as nested_name:
                nested = Path(nested_name)
                assert nested.parent == export_tempdir
                probe = nested / "read-write.txt"
                probe.write_text("ok", encoding="utf-8")
                assert probe.read_text(encoding="utf-8") == "ok"
            assert not nested.exists()

        assert author.tempfile.tempdir is original_tempdir
        assert author.tempfile.TemporaryDirectory is original_temporary_directory
    finally:
        author.shutil.rmtree(work_dir, ignore_errors=True)


def test_blender_export_operator_must_finish() -> None:
    author = _author()

    author._require_finished({"FINISHED"}, operation="GLB scene export")
    with pytest.raises(RuntimeError, match=r"GLB scene export failed.*CANCELLED"):
        author._require_finished({"CANCELLED"}, operation="GLB scene export")


def test_export_existing_blend_argument_resolves_from_repo_root() -> None:
    author = _author()
    repo_root = Path.cwd().resolve()

    args = author.parse_cli_args(
        [
            "--repo-root",
            str(repo_root),
            "--export-existing-blend",
            "build/forest-hf/forest_hf.blend",
        ]
    )

    assert args.export_existing_blend == repo_root / "build/forest-hf/forest_hf.blend"


def test_blender_command_uses_factory_startup_background_and_exact_delimiter(tmp_path: Path) -> None:
    author = _author()

    command = author.build_blender_command(
        "D:/Development/Blender/5.2/blender.exe",
        repo_root=tmp_path,
        recipe="sim/packages/worlds/forest_hf/2.0.0/terrain.recipe.json",
        output_dir="build/forest-hf/v2",
        width=640,
        height=360,
        samples=8,
    )

    assert command[:6] == [
        "D:\\Development\\Blender\\5.2\\blender.exe",
        "--factory-startup",
        "--background",
        "--python",
        str(tmp_path.resolve() / "sim/tools/worlds/forest_hf/blender_author.py"),
        "--",
    ]
    assert command[6:] == [
        "--repo-root",
        str(tmp_path.resolve()),
        "--recipe",
        str(tmp_path.resolve() / "sim/packages/worlds/forest_hf/2.0.0/terrain.recipe.json"),
        "--output-dir",
        str(tmp_path.resolve() / "build/forest-hf/v2"),
        "--width",
        "640",
        "--height",
        "360",
        "--samples",
        "8",
    ]


def test_manifest_can_require_complete_material_artifacts(tmp_path: Path) -> None:
    author = _author()
    recipe = _load_recipe(author, tmp_path)
    layout = author.generate_forest_layout(recipe)

    with pytest.raises(ValueError, match="has no generated artifact"):
        author.build_authoring_manifest(recipe, layout, [], require_material_artifacts=True)


def test_canonical_terrain_artifact_identity_detects_same_size_tampering(tmp_path: Path) -> None:
    author = _author()
    generated = tmp_path / "generated"
    generated.mkdir()
    raw = generated / "heightfield_u16.raw"
    original = b"\x00\x00\xff\xff\x00\x00\xff\xff"
    raw.write_bytes(original)
    artifacts = [
        {
            "path": "generated/heightfield_u16.raw",
            "bytes": len(original),
            "sha256": hashlib.sha256(original).hexdigest(),
        }
    ]
    manifest = {
        "schema": "lingtu.sim.forest-asset-manifest.v1",
        "world_package": "forest_hf@2.0.0",
        "seed": 20260813,
        "validation_resolution": False,
        "canonical_source": {
            "path": "generated/heightfield_u16.raw",
            "sample_type": "uint16",
            "endianness": "little",
            "dimensions_px": [2, 2],
            "extent_m": [20.0, 20.0],
        },
        "artifacts": artifacts,
        "asset_set_sha256": _asset_set_digest(artifacts),
    }
    (generated / "asset-manifest.json").write_bytes(_canonical_json(manifest))
    recipe = {
        "seed": 20260813,
        "terrain_source": {
            "package_root": str(tmp_path),
            "canonical_source": "generated/heightfield_u16.raw",
            "mesh": "generated/terrain.obj",
            "asset_manifest": "generated/asset-manifest.json",
            "enforce_asset_manifest": True,
            "dimensions_px": [2, 2],
            "sample_type": "uint16",
            "endianness": "little",
            "row_order": "north_to_south",
            "extent_m": [20.0, 20.0],
            "elevation_range_m": [-10.0, 70.0],
        }
    }

    author._load_canonical_heightfield(recipe)
    raw.write_bytes(b"\x01\x00\xfe\xff\x00\x00\xff\xff")

    with pytest.raises(ValueError, match="identity mismatch"):
        author._load_canonical_heightfield(recipe)


def test_materialized_validation_heightfield_uses_manifest_resolution(tmp_path: Path) -> None:
    author = _author()
    generated = tmp_path / "generated"
    generated.mkdir()
    values = [0, 16384, 32768, 65535]
    payload = b"".join(value.to_bytes(2, "little") for value in values)
    raw = generated / "heightfield_u16.raw"
    raw.write_bytes(payload)
    artifact = {
        "path": "generated/heightfield_u16.raw",
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }
    manifest = {
        "schema": "lingtu.sim.forest-asset-manifest.v1",
        "world_package": "forest_hf@2.0.0",
        "seed": 20260813,
        "validation_resolution": True,
        "canonical_source": {
            "path": artifact["path"],
            "sample_type": "uint16",
            "endianness": "little",
            "dimensions_px": [2, 2],
            "extent_m": [2000.0, 2000.0],
        },
        "artifacts": [artifact],
        "asset_set_sha256": _asset_set_digest([artifact]),
    }
    (generated / "asset-manifest.json").write_bytes(_canonical_json(manifest))
    recipe = {
        "seed": 20260813,
        "terrain_source": {
            "package_root": str(tmp_path),
            "canonical_source": artifact["path"],
            "mesh": "generated/terrain.obj",
            "asset_manifest": "generated/asset-manifest.json",
            "enforce_asset_manifest": True,
            "dimensions_px": [4033, 4033],
            "sample_type": "uint16",
            "endianness": "little",
            "row_order": "north_to_south",
            "extent_m": [2000.0, 2000.0],
            "elevation_range_m": [-10.0, 70.0],
        },
    }

    samples, width, height = author._load_canonical_heightfield(recipe)

    assert (width, height) == (2, 2)
    assert list(samples) == values

    manifest["validation_resolution"] = False
    (generated / "asset-manifest.json").write_bytes(_canonical_json(manifest))
    with pytest.raises(ValueError, match="validation_resolution=true"):
        author._load_canonical_heightfield(recipe)

    manifest["validation_resolution"] = True
    manifest["canonical_source"]["extent_m"] = [1999.0, 2000.0]
    (generated / "asset-manifest.json").write_bytes(_canonical_json(manifest))
    with pytest.raises(ValueError, match="extent drifted"):
        author._load_canonical_heightfield(recipe)

    manifest["canonical_source"]["extent_m"] = [2000.0, 2000.0]
    manifest["asset_set_sha256"] = "0" * 64
    (generated / "asset-manifest.json").write_bytes(_canonical_json(manifest))
    with pytest.raises(ValueError, match="identity digest mismatch"):
        author._load_canonical_heightfield(recipe)


def _write_synthetic_glb(path: Path, document: dict[str, Any], binary_chunk: bytes | None = None) -> None:
    json_chunk = json.dumps(document, separators=(",", ":")).encode()
    json_chunk += b" " * (-len(json_chunk) % 4)
    chunks = struct.pack("<II", len(json_chunk), 0x4E4F534A) + json_chunk
    if binary_chunk is not None:
        binary_chunk += b"\x00" * (-len(binary_chunk) % 4)
        chunks += struct.pack("<II", len(binary_chunk), 0x004E4942) + binary_chunk
    total_length = 12 + len(chunks)
    path.write_bytes(
        b"glTF"
        + struct.pack("<II", 2, total_length)
        + chunks
    )


def test_exported_glb_validator_rejects_templates_and_missing_textured_uvs(tmp_path: Path) -> None:
    author = _author()
    glb = tmp_path / "scene.glb"
    document = {
        "asset": {"version": "2.0"},
        "nodes": [{"name": "Template_fallen_log", "mesh": 0}],
        "materials": [{"pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}],
        "textures": [{"source": 0}],
        "images": [{"uri": "data:image/png;base64,UE5H"}],
        "meshes": [{"primitives": [{"material": 0, "attributes": {"POSITION": 0}}]}],
    }
    _write_synthetic_glb(glb, document)

    with pytest.raises(ValueError, match="excluded template"):
        author.validate_exported_glb(glb)

    document["nodes"][0]["name"] = "forest_tree_0001"
    _write_synthetic_glb(glb, document)
    with pytest.raises(ValueError, match="TEXCOORD_0"):
        author.validate_exported_glb(glb)


def test_exported_glb_validator_accepts_textured_uv_primitive(tmp_path: Path) -> None:
    author = _author()
    glb = tmp_path / "scene.glb"
    _write_synthetic_glb(
        glb,
        {
            "asset": {"version": "2.0"},
            "nodes": [{"name": "forest_tree_0001", "mesh": 0}],
            "materials": [{"normalTexture": {"index": 0}}],
            "textures": [{"source": 0}],
            "images": [{"bufferView": 0, "mimeType": "image/png"}],
            "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": 4}],
            "buffers": [{"byteLength": 4}],
            "meshes": [
                {"primitives": [{"material": 0, "attributes": {"POSITION": 0, "TEXCOORD_0": 1}}]}
            ],
        },
        b"PNG!",
    )

    report = author.validate_exported_glb(glb)

    assert report["textured_primitive_count"] == 1
    assert report["embedded_texture_count"] == 1


@pytest.mark.parametrize(
    ("textures", "images", "buffer_views", "binary_chunk", "match"),
    [
        ([{"source": 1}], [{"bufferView": 0}], [{"buffer": 0, "byteLength": 4}], b"PNG!", "source is out of bounds"),
        ([{"source": 0}], [{"bufferView": 0}], [{"buffer": 0, "byteLength": 0}], b"", "payload is empty"),
        ([{"source": 0}], [{"uri": "data:image/png;base64,"}], [], None, "payload is empty"),
    ],
)
def test_exported_glb_validator_rejects_broken_embedded_texture_chains(
    tmp_path: Path,
    textures: list[dict[str, Any]],
    images: list[dict[str, Any]],
    buffer_views: list[dict[str, Any]],
    binary_chunk: bytes | None,
    match: str,
) -> None:
    author = _author()
    glb = tmp_path / "broken.glb"
    _write_synthetic_glb(
        glb,
        {
            "asset": {"version": "2.0"},
            "nodes": [{"name": "forest_tree_0001", "mesh": 0}],
            "materials": [{"normalTexture": {"index": 0}}],
            "textures": textures,
            "images": images,
            "bufferViews": buffer_views,
            "buffers": [{"byteLength": 4}],
            "meshes": [
                {"primitives": [{"material": 0, "attributes": {"POSITION": 0, "TEXCOORD_0": 1}}]}
            ],
        },
        binary_chunk,
    )

    with pytest.raises(ValueError, match=match):
        author.validate_exported_glb(glb)


def test_exported_glb_validator_can_allow_slot_template_name_with_real_texture(tmp_path: Path) -> None:
    author = _author()
    glb = tmp_path / "slot.glb"
    _write_synthetic_glb(
        glb,
        {
            "asset": {"version": "2.0"},
            "nodes": [{"name": "LT_Forest_Asset_Pine", "mesh": 0}],
            "materials": [{"pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}],
            "textures": [{"source": 0}],
            "images": [{"uri": "data:image/png;base64,UE5H"}],
            "meshes": [
                {"primitives": [{"material": 0, "attributes": {"POSITION": 0, "TEXCOORD_0": 1}}]}
            ],
        },
    )

    report = author.validate_exported_glb(glb, forbid_template_nodes=False)

    assert report["embedded_texture_count"] == 1
