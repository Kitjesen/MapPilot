# ruff: noqa: S101

"""Regression contracts for externally conditioned Forest_HF asset slots."""

from __future__ import annotations

import hashlib
import importlib
import json
import os
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest


def _author() -> Any:
    return importlib.import_module("sim.tools.worlds.forest_hf.blender_author")


def _recipe() -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.forest-authoring-recipe.v1",
        "world_package": "forest_hf@2.0.0",
        "seed": 5808,
        "extent_m": [20.0, 20.0],
        "tree_count": 2,
        "trail": {
            "centerline_m": [[-8.0, 0.0], [8.0, 0.0]],
            "clearance_radius_m": 2.0,
        },
        "species": [
            {
                "id": "pine",
                "weight": 0.5,
                "height_m": [5.0, 8.0],
                "crown_radius_m": [1.0, 2.0],
                "material": "pine_bark_needles",
            },
            {
                "id": "birch",
                "weight": 0.5,
                "height_m": [4.0, 7.0],
                "crown_radius_m": [1.0, 1.8],
                "material": "birch_bark_leaves",
            },
        ],
        "materials": [
            {
                "id": material_id,
                "shader": "principled_pbr",
                "textures": {
                    channel: {
                        "path": f"textures/{material_id}_{channel}.png",
                        "sha256": digest_character * 64,
                        "source": "test fixture",
                        "license": "LicenseRef-LingTu-Project-Owned",
                    }
                    for channel, digest_character in (
                        ("base_color", "a"),
                        ("normal", "b"),
                        ("roughness", "c"),
                    )
                },
            }
            for material_id in ("pine_bark_needles", "birch_bark_leaves")
        ],
        "hard_rules": {
            "classification": "VisualOnly",
            "collision_profile": "NoCollision",
            "physics_authority": "mujoco",
        },
    }


def _write_external_assets(root: Path) -> dict[str, dict[str, object]]:
    slots: dict[str, dict[str, object]] = {}
    for semantic_class in ("pine", "birch", "boulder"):
        payload = f"conditioned-glb:{semantic_class}".encode()
        relative_path = Path("conditioned") / f"{semantic_class}.glb"
        path = root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
        slots[semantic_class] = {
            "path": relative_path.as_posix(),
            "bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
        }
    return slots


def _write_recipe(path: Path, recipe: dict[str, Any]) -> None:
    path.write_text(json.dumps(recipe), encoding="utf-8")


def test_recipe_accepts_exact_external_asset_slot_identities(tmp_path: Path) -> None:
    author = _author()
    recipe = _recipe()
    recipe["external_asset_slots"] = _write_external_assets(tmp_path)
    recipe_path = tmp_path / "forest.recipe.json"
    _write_recipe(recipe_path, recipe)

    loaded = author.load_forest_recipe(recipe_path)

    for semantic_class, expected in recipe["external_asset_slots"].items():
        actual = loaded["external_asset_slots"][semantic_class]
        assert actual == expected


def test_recipe_rejects_external_asset_outside_artifact_root(tmp_path: Path) -> None:
    author = _author()
    outside = tmp_path.parent / f"{tmp_path.name}-outside.glb"
    outside.write_bytes(b"outside")
    recipe = _recipe()
    recipe["external_asset_slots"] = _write_external_assets(tmp_path)
    recipe["external_asset_slots"]["pine"] = {
        "path": f"../{outside.name}",
        "bytes": outside.stat().st_size,
        "sha256": hashlib.sha256(outside.read_bytes()).hexdigest(),
    }
    recipe_path = tmp_path / "forest.recipe.json"
    _write_recipe(recipe_path, recipe)

    with pytest.raises(ValueError, match=r"portable relative path|artifact root|contain"):
        author.load_forest_recipe(recipe_path)


def test_recipe_rejects_external_asset_with_wrong_byte_identity(tmp_path: Path) -> None:
    author = _author()
    recipe = _recipe()
    recipe["external_asset_slots"] = _write_external_assets(tmp_path)
    recipe["external_asset_slots"]["birch"]["bytes"] += 1
    recipe_path = tmp_path / "forest.recipe.json"
    _write_recipe(recipe_path, recipe)

    with pytest.raises(ValueError, match=r"bytes|identity"):
        author.load_forest_recipe(recipe_path)


def test_recipe_rejects_external_asset_with_wrong_sha256_identity(tmp_path: Path) -> None:
    author = _author()
    recipe = _recipe()
    recipe["external_asset_slots"] = _write_external_assets(tmp_path)
    recipe["external_asset_slots"]["boulder"]["sha256"] = "0" * 64
    recipe_path = tmp_path / "forest.recipe.json"
    _write_recipe(recipe_path, recipe)

    with pytest.raises(ValueError, match=r"sha256|identity"):
        author.load_forest_recipe(recipe_path)


def test_recipe_rejects_external_asset_symlink(tmp_path: Path) -> None:
    author = _author()
    recipe = _recipe()
    recipe["external_asset_slots"] = _write_external_assets(tmp_path)
    target = tmp_path / "conditioned" / "pine.glb"
    link = tmp_path / "conditioned" / "pine-link.glb"
    try:
        os.symlink(target, link)
    except OSError as exc:
        pytest.skip(f"symlinks unavailable on this host: {exc}")
    recipe["external_asset_slots"]["pine"] = {
        "path": "conditioned/pine-link.glb",
        "bytes": target.stat().st_size,
        "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
    }
    recipe_path = tmp_path / "forest.recipe.json"
    _write_recipe(recipe_path, recipe)

    with pytest.raises(ValueError, match=r"link|regular"):
        author.load_forest_recipe(recipe_path)


def test_recipe_preserves_optional_external_asset_embed_depth(tmp_path: Path) -> None:
    author = _author()
    recipe = _recipe()
    recipe["external_asset_slots"] = _write_external_assets(tmp_path)
    recipe["external_asset_slots"]["pine"]["embed_depth_m"] = 0.125
    recipe_path = tmp_path / "forest.recipe.json"
    _write_recipe(recipe_path, recipe)

    loaded = author.load_forest_recipe(recipe_path)

    assert loaded["external_asset_slots"]["pine"]["embed_depth_m"] == 0.125
    assert "embed_depth_m" not in loaded["external_asset_slots"]["birch"]


@pytest.mark.parametrize("embed_depth_m", [-0.01, float("nan"), 0.500001])
def test_recipe_rejects_invalid_external_asset_embed_depth(
    tmp_path: Path,
    embed_depth_m: float,
) -> None:
    author = _author()
    recipe = _recipe()
    recipe["external_asset_slots"] = _write_external_assets(tmp_path)
    recipe["external_asset_slots"]["boulder"]["embed_depth_m"] = embed_depth_m
    recipe_path = tmp_path / "forest.recipe.json"
    _write_recipe(recipe_path, recipe)

    with pytest.raises(ValueError, match=r"embed_depth_m|finite|0\.5"):
        author.load_forest_recipe(recipe_path)


@pytest.mark.parametrize(
    ("semantic_class", "position_m", "embed_depth_m", "expected_position_m"),
    [
        ("pine", [1.0, 2.0, 3.0], 0.15, [1.0, 2.0, 2.85]),
        ("birch", [-1.0, 4.0, 0.5], 0.2, [-1.0, 4.0, 0.3]),
        ("boulder", [5.0, -2.0, 1.0], 0.4, [5.0, -2.0, 0.6]),
    ],
)
def test_external_asset_placement_embeds_each_slot_below_the_surface(
    semantic_class: str,
    position_m: list[float],
    embed_depth_m: float,
    expected_position_m: list[float],
) -> None:
    author = _author()

    actual = author.asset_slot_position_m(
        position_m,
        semantic_class,
        {semantic_class: {"embed_depth_m": embed_depth_m}},
    )

    assert actual == pytest.approx(expected_position_m)


def test_procedural_asset_placement_does_not_change_z() -> None:
    author = _author()

    actual = author.asset_slot_position_m([1.0, 2.0, 3.0], "pine", {})

    assert actual == [1.0, 2.0, 3.0]


def test_recipe_without_external_slots_keeps_procedural_templates(monkeypatch: pytest.MonkeyPatch) -> None:
    author = _author()
    procedural = {"pine": object(), "birch": object()}
    monkeypatch.setattr(author, "build_visual_templates", lambda *_args: procedural)
    blender = SimpleNamespace(
        ops=SimpleNamespace(
            import_scene=SimpleNamespace(
                gltf=lambda **_kwargs: pytest.fail("recipe without external slots must not import a GLB")
            )
        )
    )

    templates = author.build_recipe_visual_templates(blender, object(), {}, _recipe())

    assert templates["pine"] is procedural["pine"]
    assert templates["birch"] is procedural["birch"]


class _FakeObject(dict[str, object]):
    def __init__(self, name: str) -> None:
        super().__init__()
        self.name = name
        self.parent = None
        self.children_recursive: tuple[_FakeObject, ...] = ()
        self.hide_render = False
        self.hide_viewport = False
        self.location = [0.0, 0.0, 0.0]
        self.rotation_euler = [0.0, 0.0, 0.0]
        self.scale = (1.0, 1.0, 1.0)
        self.matrix_world = SimpleNamespace(copy=lambda: object())

    def hide_set(self, _value: bool) -> None:
        pass


def test_external_slot_roots_preserve_stable_visual_only_identity(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    author = _author()
    recipe = _recipe()
    recipe["external_asset_slots"] = _write_external_assets(tmp_path)
    recipe_path = tmp_path / "forest.recipe.json"
    _write_recipe(recipe_path, recipe)
    loaded = author.load_forest_recipe(recipe_path)

    class Objects(list[_FakeObject]):
        def new(self, name: str, _data: object) -> _FakeObject:
            obj = _FakeObject(name)
            self.append(obj)
            return obj

    objects = Objects()
    context = SimpleNamespace(selected_objects=[])

    def import_glb(*, filepath: str) -> set[str]:
        root = _FakeObject(f"Imported_{Path(filepath).stem}")
        objects.append(root)
        context.selected_objects = [root]
        return {"FINISHED"}

    blender = SimpleNamespace(
        context=context,
        data=SimpleNamespace(objects=objects),
        ops=SimpleNamespace(import_scene=SimpleNamespace(gltf=import_glb)),
    )
    template_collection = SimpleNamespace(objects=SimpleNamespace(link=lambda _obj: None))
    monkeypatch.setattr(author, "_require_bpy", lambda: blender)
    templates = author.build_recipe_visual_templates(blender, template_collection, {}, loaded)

    assert [templates[key].name for key in ("pine", "birch", "boulder")] == [
        "LT_Forest_Asset_Pine",
        "LT_Forest_Asset_Birch",
        "LT_Forest_Asset_Boulder",
    ]
    for semantic_class in ("pine", "birch", "boulder"):
        root = templates[semantic_class]
        assert root["asset_slot_id"] == f"forest.asset.{semantic_class}"
        assert root["classification"] == "VisualOnly"
        assert root["collision_profile"] == "NoCollision"
        assert root["collision"] is False
        assert root["simulate_physics"] is False
        assert root["can_ever_affect_navigation"] is False


def test_rock_dressing_reuses_the_conditioned_boulder_template(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    author = _author()
    boulder = _FakeObject("LT_Forest_Asset_Boulder")
    fallen_log = _FakeObject("fallen_log")
    fallen_log.data = SimpleNamespace(materials=[])
    context = SimpleNamespace(object=None)

    def reject_procedural_boulder(**_kwargs: object) -> None:
        pytest.fail("rock dressing must not create a second procedural boulder template")

    def add_fallen_log(**_kwargs: object) -> None:
        context.object = fallen_log

    blender = SimpleNamespace(
        context=context,
        ops=SimpleNamespace(
            mesh=SimpleNamespace(
                primitive_ico_sphere_add=reject_procedural_boulder,
                primitive_cylinder_add=add_fallen_log,
            )
        ),
    )
    linked_sources: list[object] = []

    def link_object(source: object, *_args: object) -> _FakeObject:
        linked_sources.append(source)
        return _FakeObject("rock_instance")

    monkeypatch.setattr(author, "_require_bpy", lambda: blender)
    monkeypatch.setattr(author, "_move_to_collection", lambda *_args: None)
    monkeypatch.setattr(author, "_link_object", link_object)

    author._add_dressing(
        _recipe(),
        {
            "seed": 5808,
            "trail": {
                "centerline_m": [[-8.0, 0.0], [8.0, 0.0]],
                "clearance_radius_m": 2.0,
            },
            "dressing": [
                {
                    "kind": "rock",
                    "stable_id": "forest.dressing.rock.000",
                    "position_m": [4.0, 5.0, 0.0],
                    "yaw_deg": 10.0,
                    "scale": 1.0,
                }
            ],
        },
        object(),
        object(),
        {"rock": object(), "bark": object()},
        {"boulder": boulder},
        {"boulder": boulder},
    )

    assert linked_sources == [boulder]
