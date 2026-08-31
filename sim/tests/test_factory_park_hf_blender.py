# ruff: noqa: S101

"""Contracts for FactoryPark_HF Blender authoring without requiring bpy."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from sim.tools.worlds.factory_park_hf import blender_author

REPO_ROOT = Path(__file__).resolve().parents[2]


def _layout() -> dict[str, object]:
    return {
        "schema": "lingtu.sim.expanded-world-layout.v1",
        "world_package": "factory_park_hf@1.0.0",
        "coordinate_system": {
            "frame": "mujoco_rh_z_up_m",
            "handedness": "right",
            "up_axis": "z",
            "linear_unit": "metre",
        },
        "extent_m": [40.0, 30.0],
        "terrain": {
            "terrain_obj": "sim/worlds/factory_park_hf/generated/terrain.obj",
            "obj_frame": "unreal_lh_z_up_cm",
            "source_frame": "mujoco_rh_z_up_m",
            "extent_m": [40.0, 30.0],
        },
        "objects": [
            {
                "id": "road_main",
                "semantic_class": "road",
                "shape": "box",
                "position_m": [0.0, 0.0, 0.025],
                "size_m": [20.0, 8.0, 0.05],
                "yaw_deg": 0.0,
                "material": "asphalt",
                "collision": True,
                "visual_only": False,
            },
            {
                "id": "loading_ramp",
                "semantic_class": "loading_ramp",
                "shape": "box",
                "position_m": [4.0, 5.0, 0.4],
                "size_m": [5.0, 5.2, 0.24],
                "yaw_deg": 20.0,
                "pitch_deg": -6.0,
                "material": "steel",
                "collision": True,
                "visual_only": False,
            },
            {
                "id": "tank_01",
                "semantic_class": "storage_tank",
                "shape": "cylinder",
                "position_m": [10.0, -5.0, 4.0],
                "radius_m": 4.0,
                "half_height_m": 4.0,
                "yaw_deg": 0.0,
                "material": "tank_white",
                "collision": True,
                "visual_only": False,
            },
        ],
    }


def test_module_imports_without_blender_and_parses_blender_arguments(tmp_path: Path) -> None:
    assert blender_author.BPY_AVAILABLE is False

    args = blender_author.parse_cli_args(
        [
            "ignored-by-blender",
            "--",
            "--repo-root",
            str(tmp_path),
            "--layout",
            "layout.json",
            "--output-dir",
            "artifacts",
            "--format",
            "fbx",
            "--width",
            "960",
            "--height",
            "540",
            "--samples",
            "8",
        ]
    )

    assert args.layout == (tmp_path / "layout.json").resolve()
    assert args.output_dir == (tmp_path / "artifacts").resolve()
    assert (
        args.realism_recipe == (tmp_path / "sim/packages/worlds/factory_park_hf/visual/realism.recipe.json").resolve()
    )
    assert args.export_format == "fbx"
    assert (args.width, args.height, args.samples) == (960, 540, 8)


def test_realism_v2_missing_recipe_uses_deterministic_visual_only_defaults(
    tmp_path: Path,
) -> None:
    settings = blender_author.load_realism_recipe(tmp_path / "missing.recipe.json")

    assert settings.profile == "industrial_realism_v2"
    assert settings.seed == 5208
    assert settings.actor_budget == (1200, 1800)
    assert settings.source == "built_in_fallback"
    assert len(settings.digest) == 64
    assert settings.dressing == ()


def test_realism_recipe_rejects_any_collision_or_untraceable_dressing(
    tmp_path: Path,
) -> None:
    recipe_path = tmp_path / "realism.recipe.json"
    recipe_path.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.factory-park-realism-recipe.v1",
                "profile": "industrial_realism_v2",
                "seed": 7,
                "actor_budget": {"min": 1200, "max": 1800},
                "dressing": [
                    {
                        "stable_id": "visual/recipe/bad",
                        "derived_from": "road_main",
                        "collision": True,
                        "visual_only": True,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="VisualOnly and collision-free"):
        blender_author.load_realism_recipe(recipe_path)

    payload = json.loads(recipe_path.read_text(encoding="utf-8"))
    payload["dressing"][0]["collision"] = False
    payload["dressing"][0]["derived_from"] = ""
    recipe_path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="derived_from"):
        blender_author.load_realism_recipe(recipe_path)


def test_realism_v2_stable_variation_is_reproducible_and_bounded() -> None:
    first = blender_author.stable_variation("dock_01", 4, low=-0.25, high=0.75)
    second = blender_author.stable_variation("dock_01", 4, low=-0.25, high=0.75)
    other = blender_author.stable_variation("dock_01", 5, low=-0.25, high=0.75)

    assert first == second
    assert -0.25 <= first <= 0.75
    assert other != first


def test_semantic_checkpoint_is_a_non_materialized_descriptor() -> None:
    item = replace(
        blender_author.validate_layout(_layout())[0],
        stable_id="checkpoint_gate",
        semantic_class="semantic_checkpoint",
        collision=False,
        visual_only=True,
    )
    state = SimpleNamespace(semantic_feature_descriptors=[])

    blender_author._record_semantic_feature_descriptor(state, item)

    assert state.semantic_feature_descriptors == [
        {
            "stable_id": "checkpoint_gate",
            "asset_key": "semantic/checkpoint_gate",
            "semantic_class": "semantic_checkpoint",
            "blender_object": None,
            "mesh_name": None,
            "shape": "box",
            "position_m": [0.0, 0.0, 0.025],
            "yaw_deg": 0.0,
            "pitch_deg": 0.0,
            "quaternion_wxyz": [1.0, -0.0, 0.0, 0.0],
            "scale": [1.0, 1.0, 1.0],
            "dimensions_m": [20.0, 8.0, 0.05],
            "material": "asphalt",
            "collision": False,
            "visual_only": True,
            "physics_proxy": "none",
            "source": "expanded_layout_semantic",
            "materialized": False,
            "exported_mesh": False,
        }
    ]


def test_validates_dimensions_pitch_and_stable_transform_identity() -> None:
    objects = blender_author.validate_layout(_layout())

    assert [item.stable_id for item in objects] == ["road_main", "loading_ramp", "tank_01"]
    assert objects[0].dimensions_m == (20.0, 8.0, 0.05)
    assert objects[1].pitch_deg == -6.0
    assert objects[2].dimensions_m == (8.0, 8.0, 8.0)
    assert objects[2].radius_m == 4.0
    assert objects[2].half_height_m == 4.0

    yaw = math.radians(20.0) / 2.0
    pitch = math.radians(-6.0) / 2.0
    assert objects[1].quaternion_wxyz == pytest.approx(
        (
            math.cos(yaw) * math.cos(pitch),
            -math.sin(yaw) * math.sin(pitch),
            math.cos(yaw) * math.sin(pitch),
            math.sin(yaw) * math.cos(pitch),
        )
    )


def test_rejects_visual_collision_and_export_name_collisions() -> None:
    layout = _layout()
    objects = list(layout["objects"])
    objects[0] = {**objects[0], "visual_only": True}
    layout["objects"] = objects
    with pytest.raises(ValueError, match="both collision and VisualOnly"):
        blender_author.validate_layout(layout)

    layout = _layout()
    objects = list(layout["objects"])
    objects.append({**objects[0], "id": "road-main"})
    layout["objects"] = objects
    with pytest.raises(ValueError, match="collide after Blender-safe normalization"):
        blender_author.validate_layout(layout)


def test_bounds_include_pitched_layout_boxes() -> None:
    objects = blender_author.validate_layout(_layout())
    bounds = blender_author.compute_layout_bounds(objects)

    assert bounds["min"][0] == pytest.approx(-10.0)
    assert bounds["max"][2] == pytest.approx(8.0)
    ramp_bounds = blender_author.compute_layout_bounds([objects[1]])
    assert ramp_bounds["min"][2] < 0.28  # Pitch expands beyond the unrotated half-height.


def test_coordinate_contract_captures_obj_conversion_and_unreal_mapping() -> None:
    contract = blender_author.coordinate_contract()

    assert contract["source"] == {
        "frame": "mujoco_rh_z_up_m",
        "handedness": "right",
        "up_axis": "+Z",
        "forward_axis": "+X",
        "units": "m",
    }
    assert contract["terrain_obj_to_blender"]["vertex_mapping"] == (
        "(x_cm,y_cm,z_cm) -> (0.01*x_cm,-0.01*y_cm,0.01*z_cm)"
    )
    assert contract["terrain_obj_to_blender"]["reverse_face_winding"] is True
    assert contract["unreal"]["position_mapping"] == "(x,y,z)m -> (100*x,-100*y,100*z)cm"
    assert contract["unreal"]["import_scale"] == 1.0
    assert contract["unreal"]["import_uniform_scale"] == 1.0
    assert contract["unreal"]["import_uniform_scale_supported"] is False
    assert contract["unreal"]["unit_conversion_strategy"] == "placement_actor_scale"
    assert contract["unreal"]["fbx_actor_uniform_scale"] == 100.0
    assert contract["unreal"]["terrain_actor_uniform_scale"] == 1.0
    assert contract["fbx"]["contains_authoritative_terrain"] is False
    assert contract["fbx"]["unreal_transform_vertex_to_absolute"] is False
    assert contract["fbx"]["unreal_import_uniform_scale"] == 1.0
    assert contract["fbx"]["unreal_import_uniform_scale_supported"] is False
    assert contract["fbx"]["unreal_actor_uniform_scale"] == 100.0
    assert contract["fbx"]["unreal_terrain_actor_uniform_scale"] == 1.0


def test_artifact_digest_uses_role_path_and_content_hash() -> None:
    records = [
        {"role": "z", "path": "z.glb", "sha256": "b" * 64},
        {"role": "a", "path": "a.fbx", "sha256": "a" * 64},
    ]
    identity = [
        {"path": "a.fbx", "role": "a", "sha256": "a" * 64},
        {"path": "z.glb", "role": "z", "sha256": "b" * 64},
    ]
    canonical = (
        json.dumps(
            identity,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            separators=(",", ": "),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")

    assert blender_author.artifact_set_digest(records) == hashlib.sha256(canonical).hexdigest()


def test_fbx_asset_record_requires_actor_scale_for_unreal_metre_to_centimetre(
    tmp_path: Path,
) -> None:
    fbx = tmp_path / "factory_park_hf.fbx"
    fbx.write_bytes(b"fixture")

    record = blender_author._asset_record("unreal_scene_fbx", fbx, tmp_path)

    assert record["units"] == "m"
    assert record["unreal_import"]["import_uniform_scale"] == 1.0
    assert record["unreal_import"]["import_uniform_scale_supported"] is False
    assert record["unreal_placement"] == {
        "required": True,
        "actor_uniform_scale": 100.0,
        "terrain_actor_uniform_scale": 1.0,
        "unit_conversion": "1 imported FBX unit * actor scale 100 = 100 Unreal cm/UU",
    }


def test_script_has_fail_closed_terrain_conversion_and_stable_outputs() -> None:
    source = (REPO_ROOT / "sim/tools/worlds/factory_park_hf/blender_author.py").read_text(encoding="utf-8")

    assert "vertex.co.x *= 0.01" in source
    assert "vertex.co.y *= -0.01" in source
    assert "bmesh.ops.reverse_faces" in source
    assert "extent_alignment_verified" in source
    assert source.count("save_as_mainfile") == 1
    for filename in (
        "factory_park_hf.blend",
        "factory_park_hf.fbx",
        "factory_park_hf.glb",
        "factory_park_hf.site-aerial-v2.png",
        "factory_park_hf.south-gate-robot-eye-v2.png",
        "factory_park_hf.loading-dock-v2.png",
        "factory_park_hf.tank-area-v2.png",
        "authoring.manifest.json",
        "artifact-set.digest.json",
    ):
        assert filename in source
    for manifest_contract in (
        '"profile": state.realism.profile',
        '"material_profile": "procedural_pbr_v2"',
        '"detail_counts": dict(sorted(state.detail_counts.items()))',
        '"physics_shared_unchanged": True',
        '"actor_budget_status": "within_budget"',
    ):
        assert manifest_contract in source


def test_checked_in_layout_is_accepted_by_blender_authoring_contract() -> None:
    layout_path = REPO_ROOT / "sim/worlds/factory_park_hf/generated/expanded-layout.json"
    layout, objects = blender_author.load_layout(layout_path)

    assert layout["schema"] == "lingtu.sim.expanded-world-layout.v1"
    assert len(layout["layout_digest"]) == 64
    assert len(objects) >= 100
    assert {item.semantic_class for item in objects} >= {
        "road",
        "main_factory_building",
        "warehouse_building",
        "shipping_container",
        "storage_tank",
        "boundary_fence",
        "streetlight",
        "parking_space",
    }
