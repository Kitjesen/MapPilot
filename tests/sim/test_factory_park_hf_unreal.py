
"""Static and pure-Python contracts for the FactoryPark_HF UE builder."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT
    / "sim"
    / "runtime"
    / "visual"
    / "RobotSimUE"
    / "Scripts"
    / "build_factory_park_hf.py"
)
RUNNER_PATH = SCRIPT_PATH.with_name("run_factory_park_hf.ps1")
REALISM_RECIPE_PATH = (
    REPO_ROOT
    / "sim"
    / "packages"
    / "worlds"
    / "factory_park_hf"
    / "visual"
    / "realism.recipe.json"
)
BLENDER_MANIFEST_PATH = (
    REPO_ROOT
    / "build"
    / "factory-park-hf"
    / "blender-v2"
    / "authoring.manifest.json"
)


def _has_current_blender_authoring() -> bool:
    if not BLENDER_MANIFEST_PATH.is_file():
        return False
    try:
        manifest = json.loads(BLENDER_MANIFEST_PATH.read_text(encoding="utf-8"))
        source_path = Path(manifest["source_layout"]["path"])
        if not source_path.is_absolute():
            source_path = REPO_ROOT / source_path
        source_path.resolve(strict=True).relative_to(REPO_ROOT.resolve())
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        return False
    return source_path.is_file()


def _load_builder_module():
    spec = importlib.util.spec_from_file_location("lingtu_factory_park_hf_builder", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_factory_park_builder_validates_materialized_world_recipe() -> None:
    builder = _load_builder_module()
    world = builder._validate_world_recipe()

    assert builder.MAP_PATH == "/Game/RobotSim/Maps/FactoryPark_HF"
    assert world["recipe"]["world_package"] == "factory_park_hf@1.0.0"
    assert world["recipe"]["binding"] == "WorldVisual:FactoryParkHF"
    assert world["layout"]["schema"] == "lingtu.sim.expanded-world-layout.v1"
    assert world["layout_digest"] == world["layout"]["layout_digest"]
    assert len(world["layout_objects"]) == world["recipe"]["unreal_layout_import"]["object_count"]
    assert world["terrain"]["coordinates"] == "unreal_lh_z_up_cm"
    assert world["terrain"]["grid"] == tuple(world["recipe"]["coordinate_contract"]["grid_px"])
    assert math.isclose(
        world["terrain"]["spawn_height_cm"],
        100.0 * world["layout"]["spawn"]["position_m"][2],
        abs_tol=0.02,
    )


def test_factory_park_coordinate_conversion_matches_runtime_contract() -> None:
    builder = _load_builder_module()
    half_sqrt = math.sqrt(0.5)

    assert builder.source_to_unreal_location_cm((1.25, -2.0, 0.5)) == (125.0, 200.0, 50.0)
    assert builder.source_to_unreal_quaternion_xyzw((1.0, 0.0, 0.0, 0.0)) == (
        0.0,
        0.0,
        0.0,
        1.0,
    )
    assert builder.source_to_unreal_quaternion_xyzw((half_sqrt, 0.0, 0.0, half_sqrt)) == pytest.approx(
        (0.0, 0.0, -half_sqrt, half_sqrt)
    )
    assert builder.source_scale_to_unreal_actor_scale((1.0, 1.0, 1.0)) == (100.0, 100.0, 100.0)


def test_factory_park_expected_pbr_count_is_derived_from_current_placements() -> None:
    builder = _load_builder_module()
    authoring = {
        "placed_objects": [
            {"material": "painted_steel", "semantic_class": "safety_bollard"},
            {"material": "painted_steel", "semantic_class": "safety_bollard"},
            {"material": "pallet_wood", "semantic_class": "pallet_stack"},
        ]
    }

    assert builder._expected_native_pbr_instance_count(authoring) == 3


@pytest.mark.skipif(
    not BLENDER_MANIFEST_PATH.is_file(),
    reason="requires generated FactoryPark Blender authoring artifacts",
)
def test_factory_park_visual_only_plan_batches_primitive_dressing_without_collision() -> None:
    builder = _load_builder_module()
    manifest = json.loads(BLENDER_MANIFEST_PATH.read_text(encoding="utf-8"))
    visual_records = manifest["scene"]["visual_only_objects"]

    plan = builder._build_visual_instance_plan(
        {"visual_only_objects": visual_records}
    )

    assert plan["schema"] == "lingtu.sim.factory-park-visual-instance-plan.v1"
    assert plan["source_instance_count"] == 1591
    assert plan["instanced_instance_count"] == 1564
    assert plan["fallback_actor_count"] == 27
    assert plan["group_count"] == 92
    assert plan["projected_visual_actor_count"] == 28
    assert plan["authority"] == {
        "classification": "VisualOnly",
        "physics": "mujoco",
        "collision_enabled": False,
        "generate_overlap_events": False,
        "can_ever_affect_navigation": False,
    }
    planned_ids = {
        instance["stable_id"]
        for group in plan["groups"]
        for instance in group["instances"]
    } | set(plan["fallback_stable_ids"])
    assert planned_ids == {record["stable_id"] for record in visual_records}
    assert all(
        group["component_class"]
        == "HierarchicalInstancedStaticMeshComponent"
        for group in plan["groups"]
    )
    assert all(
        group["primitive_mesh"]
        in {
            "/Engine/BasicShapes/Cube.Cube",
            "/Engine/BasicShapes/Cylinder.Cylinder",
            "/Engine/BasicShapes/Sphere.Sphere",
        }
        for group in plan["groups"]
    )


def test_factory_park_visual_instance_plan_rejects_authority_drift() -> None:
    builder = _load_builder_module()
    record = {
        "stable_id": "visual/a",
        "source": "blender_derived_visual",
        "physics_proxy": "none",
        "collision": True,
        "visual_only": True,
        "shape": "box",
        "material": "steel",
        "semantic_class": "fixture",
        "dimensions_m": [1.0, 1.0, 1.0],
        "position_m": [0.0, 0.0, 0.5],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
    }

    with pytest.raises(RuntimeError, match="VisualOnly authority"):
        builder._build_visual_instance_plan({"visual_only_objects": [record]})


def test_factory_park_success_requires_three_captured_acceptance_screenshots() -> None:
    builder = _load_builder_module()
    captured = [
        {
            "target_id": target_id,
            "status": "captured",
            "unsupported": False,
            "bytes": 1024,
            "sha256": "a" * 64,
        }
        for target_id in sorted(builder.REQUIRED_ACCEPTANCE_CAMERA_IDS)
    ]

    builder._validate_acceptance_screenshot_result_records(captured)
    captured[0] = {
        **captured[0],
        "status": "unsupported",
        "unsupported": True,
        "bytes": 0,
        "sha256": None,
    }
    with pytest.raises(RuntimeError, match="3/3 captured"):
        builder._validate_acceptance_screenshot_result_records(captured)


def test_factory_park_screenshot_failure_writes_error_not_success() -> None:
    builder = _load_builder_module()
    success_payloads: list[object] = []
    error_payloads: list[object] = []
    records = [
        {
            "target_id": target_id,
            "status": "unsupported",
            "unsupported": True,
            "bytes": 0,
            "sha256": None,
        }
        for target_id in sorted(builder.REQUIRED_ACCEPTANCE_CAMERA_IDS)
    ]

    completed = builder._finalize_acceptance_screenshot_results(
        {"result": "success"},
        records,
        success_writer=success_payloads.append,
        error_writer=lambda error, payload, screenshots: error_payloads.append(
            (error, payload, screenshots)
        ),
    )

    assert completed is False
    assert success_payloads == []
    assert len(error_payloads) == 1
    assert error_payloads[0][1]["result"] == "error"


def test_factory_park_screenshot_scheduler_uses_fail_closed_finalizer() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "def _write_screenshot_error_evidence(" in source
    assert source.count("_finalize_acceptance_screenshot_results(") >= 3
    assert "success_writer=_write_success_evidence" in source
    assert "error_writer=_write_screenshot_error_evidence" in source
    assert "_validate_acceptance_screenshot_result_records(screenshots)" in source


def test_factory_park_launcher_rejects_any_uncaptured_acceptance_screenshot() -> None:
    runner = RUNNER_PATH.read_text(encoding="utf-8")

    assert "$screenshot.status -ne 'captured'" in runner
    assert "$screenshot.unsupported -ne $false" in runner
    assert "[long]$screenshot.bytes -le 0" in runner
    assert "^[0-9a-f]{64}$" in runner
    assert "elseif ($screenshot.status -eq 'unsupported')" not in runner


def test_factory_park_clear_level_fails_when_any_actor_survives() -> None:
    builder = _load_builder_module()

    class ActorSubsystem:
        def get_all_level_actors(self):
            return ["removed", "survivor"]

        def destroy_actor(self, actor):
            return actor != "survivor"

    with pytest.raises(RuntimeError, match="could not destroy actor survivor"):
        builder._clear_current_level(ActorSubsystem())


def test_factory_park_collision_policy_is_modified_and_strictly_read_back() -> None:
    builder = _load_builder_module()

    class CollisionEnabled:
        NO_COLLISION = 0
        QUERY_AND_PHYSICS = 3

    builder.unreal = SimpleNamespace(CollisionEnabled=CollisionEnabled)

    class Component:
        def __init__(self, *, persist: bool = True, modify_result: bool = True) -> None:
            self.enabled = CollisionEnabled.QUERY_AND_PHYSICS
            self.profile = "BlockAll"
            self.overlap = True
            self.persist = persist
            self.modify_result = modify_result
            self.calls: list[object] = []

        def modify(self, always_mark_dirty=True):
            self.calls.append(("modify", always_mark_dirty))
            return self.modify_result

        def get_collision_enabled(self):
            return self.enabled

        def set_collision_enabled(self, value):
            self.calls.append(("set_collision_enabled", value))
            if self.persist:
                self.enabled = value

        def get_collision_profile_name(self):
            return self.profile

        def set_collision_profile_name(self, value):
            self.calls.append(("set_collision_profile_name", value))
            if self.persist:
                self.profile = value

        def get_editor_property(self, property_name):
            assert property_name == "generate_overlap_events"
            return self.overlap

        def set_editor_property(self, property_name, value):
            assert property_name == "generate_overlap_events"
            self.calls.append(("set_editor_property", property_name, value))
            if self.persist:
                self.overlap = value

    component = Component()
    result = builder._disable_collision(component)

    assert result["before"]["collision_disabled"] is False
    assert result["after"]["collision_disabled"] is True
    assert result["after"]["collision_profile_name"] == "NoCollision"
    assert result["after"]["generate_overlap_events"] is False
    assert ("modify", True) in component.calls
    assert ("set_collision_enabled", CollisionEnabled.NO_COLLISION) in component.calls
    assert ("set_collision_profile_name", "NoCollision") in component.calls
    assert ("set_editor_property", "generate_overlap_events", False) in component.calls

    unattended_component = Component(modify_result=False)
    unattended_result = builder._disable_collision(unattended_component)
    assert unattended_result["component_modify_called"] is True
    assert unattended_result["component_modify_returned"] is False
    assert unattended_result["after"]["verified"] is True

    with pytest.raises(RuntimeError, match="collision policy readback failed"):
        builder._disable_collision(Component(persist=False))


def test_factory_park_loaded_map_audit_requires_exact_stable_id_set() -> None:
    builder = _load_builder_module()
    authoring = {
        "placed_objects": [
            {
                "stable_id": "fixture/a",
                "semantic_class": "fixture",
                "visual_only": True,
                "mesh_name": "SM_A",
                "position_m": [0.0, 0.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "scale": [1.0, 1.0, 1.0],
            }
        ]
    }
    transforms = [
        {
            "stable_id": "fixture/a",
            "source_position_m": [0.0, 0.0, 0.0],
            "source_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "location_cm": [0.0, 0.0, 0.0],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "source_scale": [1.0, 1.0, 1.0],
            "scale": [100.0, 100.0, 100.0],
        }
    ]
    assignments = [
        {
            "stable_id": "fixture/a",
            "material_asset": "/Game/Fixture.M_Fixture",
            "component_slot_count": 1,
        }
    ]
    observation = {
        "stable_id": "fixture/a",
        "semantic_class": "fixture",
        "visual_only": True,
        "mesh_name": "SM_A",
        "location_cm": [0.0, 0.0, 0.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "scale": [100.0, 100.0, 100.0],
        "material_paths": ["/Game/Fixture.M_Fixture"],
        "collision_disabled": True,
        "collision_profile_name": "NoCollision",
        "generate_overlap_events": False,
    }

    evidence = builder._validate_loaded_map_observations(
        authoring,
        transforms,
        assignments,
        [observation],
        terrain_actor_count=1,
        preplaced_robot_bindings=0,
        robot_actor_count=0,
        robot_mesh_references=0,
    )
    assert evidence["stable_id_set_exact"] is True

    with pytest.raises(RuntimeError, match="StableId set"):
        builder._validate_loaded_map_observations(
            authoring,
            transforms,
            assignments,
            [],
            terrain_actor_count=1,
            preplaced_robot_bindings=0,
            robot_actor_count=0,
            robot_mesh_references=0,
        )


def test_factory_park_loaded_map_audit_rejects_runtime_contract_drift() -> None:
    builder = _load_builder_module()
    stable_id = "fixture/a"
    material_path = "/Game/RobotSim/Fixture/MI_A.MI_A"
    authoring = {
        "placed_objects": [
            {
                "stable_id": stable_id,
                "semantic_class": "fixture",
                "visual_only": True,
                "mesh_name": "SM_A",
                "position_m": [1.0, 2.0, 3.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "scale": [1.0, 1.0, 1.0],
            }
        ]
    }
    transforms = [
        {
            "stable_id": stable_id,
            "source_position_m": [1.0, 2.0, 3.0],
            "source_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "location_cm": [100.0, -200.0, 300.0],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "source_scale": [1.0, 1.0, 1.0],
            "scale": [100.0, 100.0, 100.0],
        }
    ]
    assignments = [
        {
            "stable_id": stable_id,
            "material_asset": material_path,
            "component_slot_count": 2,
        }
    ]
    observation = {
        "stable_id": stable_id,
        "semantic_class": "fixture",
        "visual_only": True,
        "mesh_name": "SM_A",
        "location_cm": [100.0, -200.0, 300.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "scale": [100.0, 100.0, 100.0],
        "material_paths": [material_path, material_path],
        "collision_disabled": True,
        "collision_profile_name": "NoCollision",
        "generate_overlap_events": False,
    }

    evidence = builder._validate_loaded_map_observations(
        authoring,
        transforms,
        assignments,
        [observation],
        terrain_actor_count=1,
        preplaced_robot_bindings=0,
        robot_actor_count=0,
        robot_mesh_references=0,
    )
    assert evidence["transform_readback_count"] == 1
    assert evidence["material_assignment_readback_count"] == 1
    assert evidence["collision_disabled_count"] == 1

    drift_cases = (
        ("location_cm", [101.0, -200.0, 300.0], "transform"),
        ("material_paths", [material_path, "/Game/Wrong.Wrong"], "material"),
        ("collision_disabled", False, "collision"),
        ("collision_profile_name", "BlockAll", "collision"),
        ("generate_overlap_events", True, "collision"),
    )
    for field, value, message in drift_cases:
        drifted = {**observation, field: value}
        with pytest.raises(RuntimeError, match=message):
            builder._validate_loaded_map_observations(
                authoring,
                transforms,
                assignments,
                [drifted],
                terrain_actor_count=1,
                preplaced_robot_bindings=0,
                robot_actor_count=0,
                robot_mesh_references=0,
            )


def test_factory_park_refresh_records_post_reload_runtime_audit() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")
    runner = RUNNER_PATH.read_text(encoding="utf-8")

    assert "def _audit_loaded_factory_map(" in source
    assert source.count("_audit_loaded_factory_map(") >= 3
    assert source.count('"loaded_map_audit": loaded_map_audit') >= 2
    assert "$successJson.loaded_map_audit" in runner
    assert "post_save_reload_runtime_v1" in runner
    assert "def _apply_collision_policy_existing_map(" in source
    assert '"collision_persistence_policy": collision_persistence_policy' in source
    assert "$successJson.collision_persistence_policy" in runner
    assert "$auditRecord.collision_enabled -ne '<CollisionEnabled.NO_COLLISION: 0>'" in runner
    assert "$refreshCollisionChangeCount" in runner
    assert "$refreshCollisionModified" in runner
    assert "$screenshotRefresh.environment_collision_components_modified" in runner


def test_factory_park_realism_recipe_is_a_single_fail_closed_visual_authority() -> None:
    builder = _load_builder_module()
    world = builder._validate_world_recipe()
    realism = builder._validate_realism_recipe(world)

    assert realism["recipe_path"] == REALISM_RECIPE_PATH.resolve()
    assert realism["profile"] == "industrial_realism_v2"
    assert realism["layout_digest"] == world["layout_digest"]
    assert realism["hard_rules"] == {
        "added_dressing_classification": "VisualOnly",
        "added_dressing_collision": False,
        "allow_new_physics_authority": False,
        "layout_unchanged": True,
        "mujoco_world_unchanged": True,
        "physics_authority": "mujoco",
    }
    assert realism["material_profiles"]
    assert realism["material_profile_by_source_material"]["asphalt"] == "asphalt_worn"
    assert realism["material_profile_by_source_material"]["tank_white"] == "tank_enamel_white"
    assert realism["material_profile_by_source_material"]["glass"] == "industrial_glass"
    assert {target["id"] for target in realism["preview_targets"]} >= {
        "south_gate_robot_eye",
        "loading_dock_robot_eye",
        "tank_farm_inspection",
    }
    assert realism["lighting"]["unreal"]["global_illumination"] == "Lumen"
    assert realism["lighting"]["unreal"]["reflections"] == "Lumen"


def test_factory_park_realism_recipe_rejects_physics_or_layout_drift() -> None:
    builder = _load_builder_module()
    recipe = json.loads(REALISM_RECIPE_PATH.read_text(encoding="utf-8"))

    recipe["hard_rules"]["added_dressing_collision"] = True
    with pytest.raises(RuntimeError, match="VisualOnly authority"):
        builder._validate_realism_recipe_document(recipe, recipe["layout_digest"])

    recipe = json.loads(REALISM_RECIPE_PATH.read_text(encoding="utf-8"))
    with pytest.raises(RuntimeError, match="layout digest"):
        builder._validate_realism_recipe_document(recipe, "0" * 64)


def test_factory_park_material_resolution_uses_recipe_then_manifest_derived_fallback() -> None:
    builder = _load_builder_module()
    realism = builder._validate_realism_recipe(builder._validate_world_recipe())

    asphalt = builder._resolve_pbr_surface("asphalt", "road", realism)
    assert asphalt["profile"] == "asphalt_worn"
    assert asphalt["mapping"] == "recipe_applies_to"
    assert asphalt["base_color_srgb"] == [0.12, 0.13, 0.14]
    assert 0.72 <= asphalt["roughness"] <= 0.92

    glass = builder._resolve_pbr_surface("window_glass", "window", realism)
    assert glass["profile"] == "industrial_glass"
    assert glass["mapping"] == "manifest_material_semantic"
    assert glass["source_material"] == "window_glass"
    assert glass["source_semantic_class"] == "window"

    first = builder._resolve_pbr_surface("vegetation_leaf", "tree_canopy", realism)
    second = builder._resolve_pbr_surface("vegetation_leaf", "tree_canopy", realism)
    assert first == second
    assert first["mapping"] == "manifest_material_semantic"
    assert first["profile"] == "manifest_derived_fallback"
    assert all(0.0 <= channel <= 1.0 for channel in first["base_color_srgb"])
    assert 0.0 <= first["metallic"] <= 1.0
    assert 0.0 <= first["roughness"] <= 1.0


def test_factory_park_material_resolution_prevents_magenta_fallbacks() -> None:
    builder = _load_builder_module()
    realism = builder._validate_realism_recipe(builder._validate_world_recipe())

    expected_swatches = {
        ("transformer_dark", "parked_truck"): [0.055, 0.065, 0.07],
        ("galvanized", "electrical_cabinet"): [0.43, 0.46, 0.47],
        ("safety_black", "forklift_mast"): [0.035, 0.04, 0.045],
        ("safety_red", "barrier_arm"): [0.55, 0.045, 0.03],
        ("vehicle_blue", "parked_forklift"): [0.08, 0.22, 0.34],
        ("vehicle_white", "parked_truck"): [0.72, 0.74, 0.72],
    }
    for identity, expected in expected_swatches.items():
        surface = builder._resolve_pbr_surface(*identity, realism)
        assert surface["base_color_srgb"] == expected
        assert surface["base_color_source"] == "manifest_material_swatch"

    fallback = builder._resolve_pbr_surface("unlisted_fixture", "unknown_prop", realism)
    assert fallback["base_color_source"] == "neutral_manifest_identity_fallback"
    assert max(fallback["base_color_srgb"]) - min(fallback["base_color_srgb"]) <= 0.04


def test_factory_park_tank_acceptance_remediation_is_ue_only_and_deterministic() -> None:
    builder = _load_builder_module()
    target = {
        "id": "tank_farm_inspection",
        "position_m": [102.0, -54.0, 1.6],
        "look_at_m": [65.0, -36.0, 3.5],
    }

    frame = builder._resolve_acceptance_camera_frame(target)
    assert frame["requested_position_m"] == [102.0, -54.0, 1.6]
    assert frame["actual_position_m"] == [35.0, -62.0, 4.2]
    assert frame["requested_look_at_m"] == [65.0, -36.0, 3.5]
    assert frame["actual_look_at_m"] == [65.0, -36.0, 4.8]
    assert frame["adjustment"] == "avoid_east_drainage_reed_raster_occlusion"

    assert builder.TANK_BUND_FILL_LIGHT == {
        "label": "FactoryPark_HF_TankBund_QAFill",
        "position_m": (100.0, -62.0, 4.5),
        "look_at_m": (65.0, -53.5, 0.7),
        "intensity": 25000.0,
        "attenuation_radius_cm": 6500.0,
        "source_width_cm": 3500.0,
        "source_height_cm": 1200.0,
    }


@pytest.mark.skipif(
    not _has_current_blender_authoring(),
    reason="requires generated FactoryPark Blender authoring artifacts",
)
def test_factory_park_drainage_reeds_remain_visible_but_do_not_cast_ue_shadows() -> None:
    builder = _load_builder_module()
    world = builder._validate_world_recipe()
    realism = builder._validate_realism_recipe(world)
    authoring = builder._validate_blender_authoring(world, realism)
    drainage_reeds = [
        record
        for record in authoring["placed_objects"]
        if record["semantic_class"] == "drainage_reed"
    ]

    assert len(drainage_reeds) == 48
    assert all(record["visual_only"] is True for record in drainage_reeds)
    assert all(builder._should_cast_visual_shadow(record) is False for record in drainage_reeds)
    assert builder._visual_shadow_component_policy(drainage_reeds[0]) == {
        "cast_shadow": False,
        "affect_distance_field_lighting": False,
        "visible_in_ray_tracing": False,
    }
    assert builder._should_cast_visual_shadow(
        {"semantic_class": "containment_bund", "visual_only": True}
    ) is True
    assert builder._visual_shadow_component_policy(
        {"semantic_class": "containment_bund", "visual_only": True}
    ) == {}

    source = SCRIPT_PATH.read_text(encoding="utf-8")
    runner = RUNNER_PATH.read_text(encoding="utf-8")
    assert "def _apply_visual_shadow_policy_existing_map(" in source
    assert '"profile": "visual_only_micro_dressing_v1"' in source
    assert "$successJson.visual_shadow_policy" in runner
    assert "drainage_reed" in runner
    assert "cast_shadow -ne $false" in runner
    assert "affect_distance_field_lighting" in runner
    assert "visible_in_ray_tracing" in runner


@pytest.mark.skipif(
    not _has_current_blender_authoring(),
    reason="requires generated FactoryPark Blender authoring artifacts",
)
def test_factory_park_builder_fail_closed_validates_frozen_blender_v2_manifest() -> None:
    builder = _load_builder_module()
    world = builder._validate_world_recipe()
    realism = builder._validate_realism_recipe(world)
    authoring = builder._validate_blender_authoring(world, realism)

    assert authoring["realism"]["profile"] == "industrial_realism_v2"
    assert authoring["realism"]["recipe"]["sha256"] == realism["recipe_sha256"]
    assert authoring["realism"]["total_mesh_actor_count"] == len(authoring["placed_objects"]) + 1
    assert 1200 <= authoring["realism"]["total_mesh_actor_count"] <= 1800
    assert len(authoring["semantic_feature_descriptors"]) == 6
    assert authoring["semantic_descriptors_not_materialized"] is True
    assert (
        len(authoring["layout_objects"])
        + len(authoring["terrain_feature_descriptors"])
        + len(authoring["semantic_feature_descriptors"])
        == len(world["layout_objects"])
    )
    semantic_ids = {record["stable_id"] for record in authoring["semantic_feature_descriptors"]}
    assert semantic_ids
    assert semantic_ids.isdisjoint(record["stable_id"] for record in authoring["placed_objects"])
    assert all(record["semantic_class"] == "semantic_checkpoint" for record in authoring["semantic_feature_descriptors"])
    assert all(record["materialized"] is False for record in authoring["semantic_feature_descriptors"])
    assert all(record["exported_mesh"] is False for record in authoring["semantic_feature_descriptors"])
    for semantic_class in (
        "security_camera",
        "barrier_arm",
        "parked_forklift",
        "tank_access_ladder",
        "tank_valve",
        "asphalt_patch",
        "manhole_cover",
        "roof_gutter",
        "hvac_unit",
    ):
        assert authoring["realism"]["detail_counts"][semantic_class] > 0


def test_factory_park_source_records_fail_closed() -> None:
    builder = _load_builder_module()
    payload = SCRIPT_PATH
    allowed_root = payload.parent
    record = {
        "path": payload.name,
        "bytes": payload.stat().st_size,
        "sha256": builder._sha256_file(payload),
    }

    assert (
        builder._validate_file_record("payload", record, allowed_root, allowed_root)
        == payload.resolve()
    )
    record["bytes"] += 1
    with pytest.raises(RuntimeError, match="byte count mismatch"):
        builder._validate_file_record("payload", record, allowed_root, allowed_root)
    record["bytes"] -= 1
    record["sha256"] = "0" * 64
    with pytest.raises(RuntimeError, match="SHA256 mismatch"):
        builder._validate_file_record("payload", record, allowed_root, allowed_root)
    record["path"] = "../escape.bin"
    with pytest.raises(RuntimeError, match="escapes allowed root"):
        builder._validate_file_record("payload", record, allowed_root, allowed_root)


def test_factory_park_builder_is_robot_free_and_disables_ue_collision() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "PREPLACED_ROBOT_BINDINGS = 0" in source
    assert 'FORBIDDEN_BINDING_TAGS = {"LingTuBodyBinding"}' in source
    assert "_count_preplaced_robot_bindings" in source
    assert '"preplaced_robot_bindings": preplaced_robot_bindings' in source
    assert '"robot_actor_count": 0' in source
    assert '"robot_mesh_references": robot_mesh_references' in source
    assert "spawn_actor_from_class(unreal.LingTuSimBodyActor" not in source
    assert "set_body_stable_id" not in source
    assert "ROBOT_MESH_DESTINATION" not in source

    assert "set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)" in source
    assert 'set_collision_profile_name("NoCollision")' in source
    assert 'get_editor_property("generate_overlap_events")' in source
    assert 'set_editor_property("generate_overlap_events", False)' in source
    assert 'set_editor_property("auto_generate_collision", False)' in source
    assert "unreal.StaticMeshEditorSubsystem" in source
    assert "subsystem.remove_collisions(mesh)" in source
    assert '"LingTu.UeCollision", "Disabled"' in source
    assert 'PHYSICS_SHARED_TAG = "PhysicsShared"' in source
    assert 'VISUAL_ONLY_TAG = "VisualOnly"' in source
    assert '"collision": "disabled_in_unreal"' in source


def test_factory_park_builder_imports_and_places_stable_blender_meshes() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "unreal.AssetImportTask()" in source
    assert "unreal.FbxImportUI()" in source
    assert 'set_editor_property("combine_meshes", False)' in source
    assert 'set_editor_property("transform_vertex_to_absolute", False)' in source
    assert 'set_editor_property("bake_pivot_in_vertex", False)' in source
    assert "FBX_IMPORT_UNIFORM_SCALE = 1.0" in source
    assert "ENVIRONMENT_ACTOR_UNIT_SCALE = 100.0" in source
    assert "TERRAIN_ACTOR_UNIT_SCALE = 1.0" in source
    assert 'set_editor_property("import_uniform_scale", FBX_IMPORT_UNIFORM_SCALE)' in source
    assert '"unreal_scene_fbx"' in source
    assert '"portable_scene_glb"' in source
    assert 'scene.get("layout_objects")' in source
    assert 'scene.get("visual_only_objects")' in source
    assert 'record["stable_id"]' in source
    assert 'record["mesh_name"]' in source
    assert "source_to_unreal_location_cm" in source
    assert "source_to_unreal_quaternion_xyzw" in source
    assert "level_subsystem.new_level(MAP_PATH, False)" in source
    assert "level_subsystem.save_current_level()" in source


def test_factory_park_builder_materializes_native_pbr_and_assigns_every_mesh() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert 'REALISM_PROFILE = "industrial_realism_v2"' in source
    assert "unreal.MaterialEditingLibrary" in source
    assert "unreal.MaterialExpressionVectorParameter" in source
    assert "unreal.MaterialExpressionScalarParameter" in source
    assert "unreal.MaterialExpressionSaturate" in source
    assert "unreal.MaterialExpressionWorldPosition" not in source
    assert "unreal.MaterialExpressionNoise" not in source
    assert "unreal.MaterialExpressionMultiply" not in source
    assert "unreal.MaterialExpressionAdd" not in source
    assert '"SurfaceVariation"' not in source
    assert '("roughness_to_saturate", roughness, "", roughness_saturate, "")' in source
    assert "get_material_expression_input_names" in source
    assert "get_material_expression_output_names" in source
    assert '"roughness_path": "clamped_parameter"' in source
    assert '"world_position_noise_used": False' in source
    assert '"base_color_policy": "manifest_material_swatch_then_neutral_identity_fallback"' in source
    assert "unreal.MaterialInstanceConstant" in source
    assert "unreal.MaterialInstanceConstantFactoryNew" in source
    assert "set_material_instance_parent" in source
    assert "set_material_instance_vector_parameter_value" in source
    assert "set_material_instance_scalar_parameter_value" in source
    assert "get_material_instance_vector_parameter_value" in source
    assert "get_material_instance_scalar_parameter_value" in source
    assert "verified_by_getter_readback_due_ue58_false_return_value" in source
    assert "if not library.set_material_instance_vector_parameter_value" not in source
    assert "if not library.set_material_instance_scalar_parameter_value" not in source
    assert 'record["material"]' in source
    assert 'record["semantic_class"]' in source
    assert "mesh.set_material(material_index, material)" in source
    assert "component.set_material(material_index, material)" in source
    assert '"native_pbr_materials"' in source
    assert '"source_material"' in source
    assert '"source_semantic_class"' in source
    assert '"material_asset"' in source
    build_body = source.split("def build_factory_park_hf()", 1)[1]
    assert build_body.index("_build_native_pbr_material_library") < build_body.index(
        "_import_terrain"
    )


def test_factory_park_builder_configures_lumen_movable_lights_and_acceptance_cameras() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "unreal.ComponentMobility.MOVABLE" in source
    assert "r.DynamicGlobalIlluminationMethod 1" in source
    assert "r.ReflectionMethod 1" in source
    assert "r.Shadow.Virtual.Enable 1" in source
    assert "unreal.AutomationLibrary.take_high_res_screenshot" in source
    assert "unreal.AutomationLibrary.finish_loading_before_screenshot" in source
    assert "unreal.AutomationLibrary.set_scalability_quality_to_epic" in source
    assert 'getattr(unreal, "register_slate_post_tick_callback", None)' in source
    assert "task.is_valid_task()" in source
    assert "task.is_task_done()" in source
    assert "SCREENSHOT_TIMEOUT_SECONDS" in source
    assert '"south_gate_robot_eye"' in source
    assert '"loading_dock_robot_eye"' in source
    assert '"tank_farm_inspection"' in source
    assert '"acceptance_cameras"' in source
    assert '"screenshots"' in source
    assert '"sha256"' in source
    assert '"unsupported"' in source
    assert "_configure_optional_nanite" in source
    assert 'budgets.get("enable_nanite", False)' in source
    assert '"non_fatal": True' in source
    assert '"nanite": nanite' in source


def test_factory_park_builder_has_explicit_entrypoint_and_evidence() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert 'if __name__ == "__main__":' in source
    assert "else:\n    main()" not in source
    assert '"schema": "lingtu.sim.unreal-factory-park-hf-evidence.v1"' in source
    assert '"result": "success"' in source
    assert '"layout_digest": world["layout_digest"]' in source
    assert '"artifact_set_digest": authoring["artifact_set_digest"]' in source
    assert '"actor_count": terrain_actor_count' in source
    assert "terrain_actor_count != 1" in source
    assert '"stable_actor_world_transforms": stable_actor_world_transforms' in source
    assert '"semantic_descriptors_not_materialized": True' in source
    assert '"actor_count": 0' in source
    assert '"mesh_count": 0' in source
    assert '"fbx_mesh_vertices": "local_origin"' in source
    assert '"transform_vertex_to_absolute": False' in source
    assert '"fbx_import_uniform_scale_requested": FBX_IMPORT_UNIFORM_SCALE' in source
    assert '"fbx_import_uniform_scale_supported": False' in source
    assert '"environment_actor_unit_scale": ENVIRONMENT_ACTOR_UNIT_SCALE' in source
    assert '"terrain_actor_unit_scale": TERRAIN_ACTOR_UNIT_SCALE' in source
    assert '"unit_conversion_strategy": "placement_actor_scale"' in source
    assert '"coordinate_conversion"' in source
    assert '"lighting": lighting' in source
    assert '"camera": {' in source
    assert '"editor_map": True' in source
    assert '"cook": False' in source
    assert '"package": False' in source
    assert "LINGTU_FACTORY_PARK_HF_READY" in source
    assert "LINGTU_FACTORY_PARK_HF_ERROR" in source


def test_factory_park_launcher_validates_inputs_and_only_runs_unreal_builder() -> None:
    runner = RUNNER_PATH.read_text(encoding="utf-8")

    for parameter in (
        "[string]$UnrealRoot",
        "[string]$BlenderExe",
        "[string]$WorldRecipe",
        "[string]$RealismRecipe",
        "[string]$BlenderManifest",
        "[string]$ArtifactDigest",
        "[string]$EvidenceRoot",
        "[string]$DerivedDataCacheRoot",
        "[switch]$SkipUnrealBuild",
        "[switch]$Unattended",
        "[switch]$ValidateExistingEvidence",
        "[int]$TimeoutMinutes",
        "[int]$NoProgressMinutes",
    ):
        assert parameter in runner

    assert "D:\\Development\\Blender\\5.2\\blender.exe" in runner
    assert "Engine\\Binaries\\Win64\\UnrealEditor.exe" in runner
    assert "build_factory_park_hf.py" in runner
    assert "lingtu.sim.unreal-world-import-recipe.v1" in runner
    assert "lingtu.sim.factory-park-realism-recipe.v1" in runner
    assert "industrial_realism_v2" in runner
    assert "/Game/RobotSim/Maps/FactoryPark_HF" in runner
    assert "lingtu.sim.blender-authoring-manifest.v1" in runner
    assert "coordinate_contract.unreal.fbx_actor_uniform_scale -ne 100.0" in runner
    assert "unreal_import.import_uniform_scale_supported -ne $false" in runner
    assert "unreal_placement.actor_uniform_scale -ne 100.0" in runner
    assert "coordinate_conversion.environment_actor_unit_scale -ne 100.0" in runner
    assert "stable_actor_world_transforms" in runner
    assert "semantic_feature_descriptors" in runner
    assert "semantic_descriptors_not_materialized" in runner
    assert "semantic checkpoint was incorrectly materialized as a mesh" in runner
    assert "Assert-NumericVectorClose" in runner
    assert "expectedByStableId" in runner
    assert "source_position_m" in runner
    assert "source_quaternion_wxyz" in runner
    assert "quaternion_xyzw" in runner
    assert "mesh_asset_dimensions_uu" in runner
    assert "expected_world_dimensions_cm" in runner
    assert "world_dimensions_cm" in runner
    assert "Get-FileHash" in runner
    assert "--version" in runner
    assert "-DDC-ForceMemoryCache" in runner
    assert "'-unattended'" in runner
    assert "if ($Unattended) { 'Hidden' } else { 'Normal' }" in runner
    assert "Start-Process" in runner
    assert "LINGTU_FACTORY_PARK_HF_RECIPE" in runner
    assert "LINGTU_FACTORY_PARK_HF_REALISM_RECIPE" in runner
    assert "LINGTU_FACTORY_PARK_HF_BLENDER_MANIFEST" in runner
    assert "LINGTU_FACTORY_PARK_HF_ARTIFACT_DIGEST" in runner
    assert "Write-LauncherErrorSentinel" in runner
    assert "ProductControl" not in runner
    assert "lingtu.control" not in runner
    assert "live_visual" not in runner

    assert "build\\factory-park-hf\\blender-v2" in runner
    assert "build\\factory-park-hf\\unreal-v2" in runner


def test_factory_park_launcher_and_builder_prove_writable_derived_data_cache() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")
    runner = RUNNER_PATH.read_text(encoding="utf-8")

    assert "UE-LocalDataCachePath" in runner
    assert "UE-SharedDataCachePath" in runner
    assert "-Context 'DerivedDataCacheRoot'" in runner
    assert "LINGTU_FACTORY_PARK_HF_DDC_FORCE_MEMORY_CACHE" in runner
    assert '"derived_data_cache": derived_data_cache' in source
    assert "_validate_derived_data_cache" in source
    assert "get_command_line" in source
    assert "local_override_under_evidence_root" in source
    assert "successJson.derived_data_cache" in runner


def test_factory_park_launcher_scales_vectors_component_wise_and_can_revalidate() -> None:
    runner = RUNNER_PATH.read_text(encoding="utf-8")

    assert "function ConvertTo-ComponentScaledNumericVector" in runner
    assert "$scaledValues[$index] = [double]$Factors[$index] * $sourceNumber" in runner
    assert "-Factors ([double[]]@(100.0, -100.0, 100.0))" in runner
    assert "-Factors ([double[]]@(100.0, 100.0, 100.0))" in runner
    assert "100.0 * $expectedPosition[0]," not in runner
    assert "100.0 * $expectedSourceScale[0]," not in runner
    assert "100.0 * $expectedMeshDimensionsUu[0]," not in runner
    assert "if (-not $ValidateExistingEvidence)" in runner
    assert "$errorItem.LastWriteTimeUtc -ge $successItem.LastWriteTimeUtc" in runner
    assert "Existing FactoryPark_HF error evidence is newer than or as new as" in runner
    assert "ValidatedExistingEvidence=$successSentinel" in runner
    assert "native_pbr_materials" in runner
    assert "successJson.native_pbr_materials.master_graph" in runner
    assert "master_graph.roughness_path" in runner
    assert "'clamped_parameter'" in runner
    assert "master_graph.world_position_noise_used -ne $false" in runner
    assert "master_graph.expression_connection_count -ne 1" in runner
    assert "PSObject.Properties).Count -ne 3" in runner
    assert "base_color_policy -ne" in runner
    assert "manifest_material_swatch_then_neutral_identity_fallback" in runner
    assert "$requiredMaterialSwatches" in runner
    assert "neutral_manifest_identity_fallback" in runner
    assert "acceptance_cameras" in runner
    assert "screenshots" in runner
    assert "industrial_realism_v2" in runner


def test_factory_park_launcher_allows_full_asset_persistence_to_finish() -> None:
    runner = RUNNER_PATH.read_text(encoding="utf-8")

    assert "[int]$TimeoutMinutes = 60" in runner


def test_factory_park_launcher_reads_every_json_as_explicit_utf8() -> None:
    runner = RUNNER_PATH.read_text(encoding="utf-8")

    assert "function Read-JsonUtf8" in runner
    assert "[System.IO.File]::ReadAllText(" in runner
    assert "[System.Text.Encoding]::UTF8" in runner
    assert "return $jsonText | ConvertFrom-Json" in runner
    assert "Get-Content -Raw -LiteralPath" not in runner


def test_factory_park_launcher_rejects_reparse_point_path_traversal() -> None:
    runner = RUNNER_PATH.read_text(encoding="utf-8")

    assert "function Assert-NoReparsePointTraversal" in runner
    assert "[System.IO.FileAttributes]::ReparsePoint" in runner
    assert "Resolve-Path -LiteralPath $lexicalPath" in runner
    assert "Resolve-PathInsideRoot `" in runner
    assert "-Context 'acceptance screenshot'" in runner


def test_factory_park_launcher_can_refresh_screenshots_without_reimporting_meshes() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")
    runner = RUNNER_PATH.read_text(encoding="utf-8")

    assert "[switch]$RefreshScreenshotsOnly" in runner
    assert "LINGTU_FACTORY_PARK_HF_REFRESH_SCREENSHOTS_ONLY" in runner
    assert "FactoryPark_HF.previous.success.json" in runner
    assert "def refresh_acceptance_screenshots()" in source
    assert "_validate_refresh_source_evidence" in source
    assert '"mode": "screenshot_refresh_v1"' in source
    assert "_schedule_acceptance_screenshots" in source
    assert "_spawn_tank_bund_fill_light" in source
    assert "FactoryPark_HF_TankBund_QAFill" in runner
    assert "lighting_remediation" in runner
    assert "cast_shadows -ne $false" in runner
