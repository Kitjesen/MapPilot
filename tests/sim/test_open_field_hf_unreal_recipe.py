"""Static and source-integrity contracts for the OpenField_HF UE recipe."""

# ruff: noqa: S101

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "sim" / "runtime" / "visual" / "RobotSimUE" / "Scripts" / "build_open_field_hf.py"
RUNNER_PATH = SCRIPT_PATH.with_name("run_open_field_hf.ps1")
WORLD_RECIPE_PATH = REPO_ROOT / "sim" / "packages" / "worlds" / "open_field_hf" / "visual" / "ue_import.recipe.json"


def _load_builder_module():
    spec = importlib.util.spec_from_file_location("lingtu_open_field_hf_builder", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_open_field_hf_builder_validates_same_source_recipe_and_digest() -> None:
    builder = _load_builder_module()
    world = builder._validate_world_recipe()

    recipe = json.loads(WORLD_RECIPE_PATH.read_text(encoding="utf-8"))
    assert recipe["target_level"] == "/Game/RobotSim/Maps/OpenField_HF"
    assert recipe["blender_mesh_import"] == {
        "coordinates": "unreal_lh_z_up_cm",
        "import_scale": 1.0,
        "source": "sim/packages/worlds/open_field_hf/generated/terrain.obj",
        "vertex_per_height_sample": True,
    }
    assert world["terrain_digest"] == recipe["sources"]["terrain_obj"]["sha256"]
    assert world["heightfield_digest"] == recipe["sources"]["heightfield_png"]["sha256"]
    assert world["grid"] == (253, 253)
    assert len(world["vertices"]) == 253 * 253


def test_open_field_hf_builder_stages_one_uv_per_source_vertex() -> None:
    builder = _load_builder_module()
    world = builder._validate_world_recipe()
    builder.EVIDENCE_DIR = REPO_ROOT / "build" / "unreal-open-field-hf"

    staged_path, staged_digest = builder._stage_unreal_obj(
        world["terrain_path"], world["grid"], world["terrain_digest"]
    )
    lines = staged_path.read_text(encoding="ascii").splitlines()

    assert builder._sha256_file(staged_path) == staged_digest
    assert sum(line.startswith("v ") for line in lines) == 253 * 253
    assert sum(line.startswith("vt ") for line in lines) == 253 * 253
    assert sum(line.startswith("f ") for line in lines) == 127_008
    assert all(
        token.split("/")[0] == token.split("/")[1]
        for line in lines
        if line.startswith("f ")
        for token in line.split()[1:]
    )


def test_open_field_hf_builder_uses_independent_map_and_real_terrain() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert 'MAP_PATH = "/Game/RobotSim/Maps/OpenField_HF"' in source
    assert "ThunderV4_RuntimePreview" not in source
    assert "SM_OpenField_HF_Terrain" in source
    assert "terrain OBJ is not a 160 m Unreal-centimetre mesh" in source
    assert "scale=unreal.Vector(1.0, 1.0, 1.0)" in source
    assert "/Engine/BasicShapes/Plane.Plane" not in source
    assert "ue58_interchange_uv_contract=vertex_index_equals_uv_index" in source
    assert 'f"{index}/{index}"' in source
    assert '"geometry_changed": False' in source


def test_open_field_hf_builder_preserves_runtime_body_bindings() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "EXPECTED_ROBOT_BODY_COUNT = 21" in source
    assert "unreal.LingTuSimBodyActor" in source
    assert "set_body_stable_id" in source
    assert 'unreal.Name("LingTuBodyBinding")' in source
    assert "actor.attach_to_actor" in source
    assert "component.get_num_materials()" in source
    assert "for material_index in range(material_slot_count)" in source
    assert "component.get_material(material_index)" in source


def test_open_field_hf_visual_only_props_are_explicit_and_non_colliding() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert 'VISUAL_ONLY_TAG = "VisualOnly"' in source
    assert "VisualOnly_Rock_" in source
    assert "VisualOnly_Grass_" in source
    assert "VisualOnly_MudPatch_" in source
    assert "VisualOnly_Deadwood_" in source
    assert 'unreal.load_asset("/Engine/BasicShapes/Cone.Cone")' not in source
    assert 'set_editor_property("cast_shadow", False)' in source
    assert "HERO_VIEW_FAR_CLEARANCE_CM = 4000.0" in source
    assert "math.tan(math.radians(HERO_CAMERA_FOV_DEGREES * 0.5))" in source
    assert source.count("x, y = _sample_clear_visual_prop_xy") == 4
    assert "z + 50.0 * grass_scale_z" in source
    assert all(scale in source for scale in ("(2.6, 1.2)", "(1.8, 1.0)", "(3.0, 1.1)", "(2.4, 1.4)"))
    assert "set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)" in source
    assert '"collision": "disabled"' in source


def test_open_field_hf_visual_quality_uses_ue58_safe_low_frequency_graph() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "MaterialExpressionPower" not in source
    assert "MaterialExpressionOneMinus" not in source
    assert source.count("MaterialExpressionNoise") == 2
    assert "0.00042" in source
    assert "0.00016" in source
    assert source.count('set_editor_property("levels", 1)') == 2
    assert "graph=dual-low-frequency-noise-lerp" in source
    assert all(name in source for name in ("z00", "z10", "z01", "z11"))
    assert "LINGTU_OPEN_FIELD_HF_TERRAIN_REUSED" in source
    assert '"OpenField_RobotFill"' in source
    assert "_is_clear_of_hero_view" in source
    assert "HERO_CAMERA_FOV_DEGREES = 36.0" in source
    assert "(0.08, 0.10, 0.12)" in source
    assert "(0.025, 0.035, 0.045)" in source
    assert 'set_editor_property("intensity", 160.0)' in source
    assert 'set_editor_property("field_of_view", HERO_CAMERA_FOV_DEGREES)' in source


def test_open_field_hf_hero_frustum_excludes_props_through_far_terrain() -> None:
    builder = _load_builder_module()
    forward_x = -builder.HERO_CAMERA_X_CM
    forward_y = -builder.HERO_CAMERA_Y_CM
    length = (forward_x**2 + forward_y**2) ** 0.5

    for distance_cm in (500.0, 2000.0, 3900.0):
        x = builder.HERO_CAMERA_X_CM + forward_x / length * distance_cm
        y = builder.HERO_CAMERA_Y_CM + forward_y / length * distance_cm
        assert not builder._is_clear_of_hero_view(x, y)


def test_open_field_hf_screenshot_and_sentinel_contract() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")
    runner = RUNNER_PATH.read_text(encoding="utf-8")

    assert "SCREENSHOT_WIDTH = 1920" in source
    assert "SCREENSHOT_HEIGHT = 1080" in source
    assert "take_high_res_screenshot" in source
    assert "LINGTU_OPEN_FIELD_HF_READY=" in source
    assert '"cook": False' in source
    assert '"package": False' in source
    assert "-DDC-ForceMemoryCache" in runner
    assert "'-unattended'" in runner
    assert "if ($Unattended) { 'Hidden' } else { 'Normal' }" in runner
    assert "Start-Process" in runner
    assert "D:\\Development\\Blender\\5.2\\blender.exe" in runner
    assert "Write-LauncherErrorSentinel" in runner
    assert "Failed to compile Material.*M_OpenField_HF_Terrain" in runner
    assert "M_OpenField_HF_Terrain.*Failed to compile Material" in runner
    assert "Unreal Editor exited before OpenField_HF success" in runner
    assert "$NoProgressMinutes" in runner
