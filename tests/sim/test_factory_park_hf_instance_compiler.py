
"""Contracts for compiling FactoryPark visual dressing into UE HISM batches."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT
    / "sim"
    / "runtime"
    / "visual"
    / "RobotSimUE"
    / "Scripts"
    / "compile_factory_park_visual_instances.py"
)
RUNNER_PATH = SCRIPT_PATH.with_name("run_factory_park_visual_instances.ps1")
BATCH_ACTOR_HEADER = (
    REPO_ROOT
    / "sim"
    / "runtime"
    / "visual"
    / "RobotSimUE"
    / "Plugins"
    / "LingTuSim"
    / "Source"
    / "LingTuSimVisual"
    / "Public"
    / "LingTuSimVisualInstanceBatchActor.h"
)
BATCH_ACTOR_SOURCE = BATCH_ACTOR_HEADER.parents[1] / "Private" / "LingTuSimVisualInstanceBatchActor.cpp"
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
        source_layout = manifest["source_layout"]["path"]
        source_path = Path(source_layout)
        if not source_path.is_absolute():
            source_path = REPO_ROOT / source_path
        source_path.resolve(strict=True).relative_to(REPO_ROOT.resolve())
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        return False
    return source_path.is_file()


def _load_compiler_module():
    spec = importlib.util.spec_from_file_location(
        "lingtu_factory_park_visual_instance_compiler",
        SCRIPT_PATH,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.skipif(
    not _has_current_blender_authoring(),
    reason="requires generated FactoryPark Blender authoring artifacts",
)
def test_factory_park_instance_contract_reduces_actor_count_without_changing_identity() -> None:
    compiler = _load_compiler_module()
    contract = compiler._compile_projection_contract_from_sources()

    assert contract["schema"] == "lingtu.sim.factory-park-visual-instance-contract.v1"
    assert contract["source_stable_id_count"] == 1730
    assert contract["layout_actor_count"] == 139
    assert contract["physics_shared_actor_count"] == 89
    assert contract["fallback_visual_actor_count"] == 27
    assert contract["instanced_visual_count"] == 1564
    assert contract["instance_group_count"] == 92
    assert contract["projected_environment_actor_count"] == 167
    assert contract["actor_reduction"] == 1563
    assert len(contract["contract_digest"]) == 64
    assert contract["target_map_path"].startswith(
        "/Game/RobotSim/Generated/FactoryParkHF/Maps/FactoryPark_HF_Instanced_"
    )
    assert contract["authority"] == {
        "physics": "mujoco",
        "unreal_collision": "NoCollision",
        "visual_instance_classification": "VisualOnly",
    }
    expected_ids = set(contract["expected_stable_ids"])
    individual_ids = set(contract["expected_individual_actor_ids"])
    instanced_ids = {
        stable_id
        for group in contract["groups"]
        for stable_id in group["stable_ids"]
    }
    assert not individual_ids.intersection(instanced_ids)
    assert individual_ids | instanced_ids == expected_ids


def test_observed_instance_projection_is_exact_and_fail_closed() -> None:
    compiler = _load_compiler_module()
    contract = {
        "expected_stable_ids": ["layout/a", "visual/a", "visual/b"],
        "expected_individual_actor_ids": ["layout/a"],
        "groups": [
            {
                "group_id": "group/a",
                "stable_ids": ["visual/a", "visual/b"],
                "instance_count": 2,
                "primitive_mesh": "/Engine/BasicShapes/Cube.Cube",
                "material_asset": "/Game/Test/M_Test.M_Test",
                "cast_shadow": False,
            }
        ],
    }
    observed_groups = [
        {
            "group_id": "group/a",
            "stable_ids": ["visual/a", "visual/b"],
            "instance_count": 2,
            "primitive_mesh": "/Engine/BasicShapes/Cube.Cube",
            "material_asset": "/Game/Test/M_Test.M_Test",
            "component_material_asset": "/Game/Test/M_Test.M_Test",
            "cast_shadow": False,
            "collision_disabled": True,
            "generate_overlap_events": False,
            "can_ever_affect_navigation": False,
            "simulate_physics": False,
            "strict_live_state_verified": True,
        }
    ]

    evidence = compiler._validate_observed_projection(
        contract,
        individual_actor_ids=["layout/a"],
        observed_groups=observed_groups,
        batch_actor_count=1,
    )

    assert evidence["stable_id_set_exact"] is True
    assert evidence["collision_authority_verified"] is True
    assert evidence["observed_instance_count"] == 2

    observed_groups[0]["collision_disabled"] = False
    with pytest.raises(RuntimeError, match="presentation policy"):
        compiler._validate_observed_projection(
            contract,
            individual_actor_ids=["layout/a"],
            observed_groups=observed_groups,
            batch_actor_count=1,
        )

    observed_groups[0]["collision_disabled"] = True
    observed_groups[0]["strict_live_state_verified"] = False
    with pytest.raises(RuntimeError, match="presentation policy"):
        compiler._validate_observed_projection(
            contract,
            individual_actor_ids=["layout/a"],
            observed_groups=observed_groups,
            batch_actor_count=1,
        )


def test_instance_compiler_synchronizes_source_rendering_contract_to_target_map() -> None:
    compiler = _load_compiler_module()

    class Actor:
        def __init__(self, label: str) -> None:
            self._label = label
            self.light_component = object()

        def get_actor_label(self) -> str:
            return self._label

    sun = Actor("FactoryPark_HF_Sun")
    post_process = Actor("FactoryPark_HF_PostProcess")

    class ActorSubsystem:
        @staticmethod
        def get_all_level_actors() -> list[Actor]:
            return [sun, post_process]

    class Builder:
        SUN_LABEL = "FactoryPark_HF_Sun"
        POST_PROCESS_LABEL = "FactoryPark_HF_PostProcess"

        @staticmethod
        def _validate_world_recipe() -> dict[str, object]:
            return {"world": "validated"}

        @staticmethod
        def _validate_realism_recipe(world: object) -> dict[str, object]:
            assert world == {"world": "validated"}
            return {
                "lighting": {
                    "sun": {
                        "illuminance_lux": 78000.0,
                        "color_temperature_k": 5600.0,
                        "angular_diameter_deg": 0.535,
                    },
                    "exposure": {"mode": "manual", "ev100": 14.0},
                }
            }

        @staticmethod
        def _configure_sun_component(component: object, recipe: object) -> dict[str, object]:
            assert component is sun.light_component
            assert recipe == {
                "illuminance_lux": 78000.0,
                "color_temperature_k": 5600.0,
                "angular_diameter_deg": 0.535,
            }
            return {"verified": True, "illuminance_lux": 78000.0}

        @staticmethod
        def _configure_lumen_world(actor: object, realism: object) -> dict[str, object]:
            assert actor is post_process
            assert realism["lighting"]["exposure"] == {
                "mode": "manual",
                "ev100": 14.0,
            }
            return {
                "exposure": {"verified": True, "mode": "manual", "ev100": 14.0},
            }

    evidence = compiler._synchronize_target_rendering_contract(Builder(), ActorSubsystem())

    assert evidence == {
        "sun": {"verified": True, "illuminance_lux": 78000.0},
        "lumen": {
            "exposure": {"verified": True, "mode": "manual", "ev100": 14.0},
        },
        "verified": True,
    }

    class DuplicateSunActorSubsystem:
        @staticmethod
        def get_all_level_actors() -> list[Actor]:
            return [sun, Actor("FactoryPark_HF_Sun"), post_process]

    with pytest.raises(RuntimeError, match=r"exactly one.*FactoryPark_HF_Sun"):
        compiler._synchronize_target_rendering_contract(
            Builder(),
            DuplicateSunActorSubsystem(),
        )


def test_instance_compiler_creates_complete_batch_before_destroying_source_actors() -> None:
    _load_compiler_module()
    source = SCRIPT_PATH.read_text(encoding="utf-8")
    function_source = source[
        source.index("def _materialize_projection(") : source.index(
            "def _materialize_source_copy_to_staging("
        )
    ]

    assert function_source.index("add_visual_group(") < function_source.index(
        "destroy_actor("
    )
    assert "save_current_level()" not in function_source
    assert "physics_proxy" not in function_source

    stage_source = source[
        source.index("def _stage_factory_park_visual_instances(") : source.index(
            "def _publish_factory_park_visual_instances("
        )
    ]
    publish_source = source[
        source.index("def _publish_factory_park_visual_instances(") : source.index(
            "def compile_factory_park_visual_instances("
        )
    ]
    assert "duplicate_asset(" not in stage_source
    assert "_materialize_source_copy_to_staging(" in stage_source
    assert "load_level(source_map_path)" in stage_source
    assert stage_source.count("load_level(") == 1
    assert "STAGE_SENTINEL" in stage_source
    assert "source_fingerprint" in stage_source
    assert "load_level(staging_map_path)" not in publish_source
    assert publish_source.index("rename_asset(") < publish_source.index(
        "load_level(target_map_path)"
    )
    assert publish_source.index("load_level(target_map_path)") < publish_source.index(
        "_synchronize_target_rendering_contract("
    )
    assert publish_source.index("_synchronize_target_rendering_contract(") < publish_source.index(
        "EditorLoadingAndSavingUtils.save_map("
    )
    assert publish_source.index("EditorLoadingAndSavingUtils.save_map(") < publish_source.index(
        "_verify_loaded_projection("
    )
    assert "_verify_loaded_projection(" in publish_source
    assert '"rendering_contract": rendering_contract' in publish_source
    assert "source_fingerprint" in publish_source
    assert "package_sha256" in source
    assert "actor_fingerprint_sha256" in source
    assert "immutable source map package or actor state changed" in source

    staging_mutation_source = source[
        source.index("def _materialize_source_copy_to_staging(") : source.index(
            "def _atomic_write_json("
        )
    ]
    assert "EditorLoadingAndSavingUtils.save_map(" in staging_mutation_source
    assert staging_mutation_source.index("_materialize_projection(") < (
        staging_mutation_source.index("EditorLoadingAndSavingUtils.save_map(")
    )
    assert "get_editor_world()" in staging_mutation_source
    assert "load_level(" not in staging_mutation_source
    assert "_materialize_projection(" in staging_mutation_source
    assert "save_current_level()" not in staging_mutation_source
    assert "load_level(source_map_path)" not in staging_mutation_source

    assert "load_level(source_map_path)" not in stage_source[
        stage_source.index("_materialize_source_copy_to_staging(") :
    ]
    assert "load_level(source_map_path)" not in publish_source[
        publish_source.index("load_level(target_map_path)") :
    ]


def test_instance_runner_preserves_exec_cmd_as_one_argument_and_forbids_compilation() -> None:
    runner = RUNNER_PATH.read_text(encoding="utf-8")

    assert "$startInfo.ArgumentList.Add($argument)" in runner
    assert "Start-Process" not in runner
    assert '"-ExecCmds=py $compiler"' in runner
    assert "foreach ($phase in @('stage', 'publish'))" in runner
    assert "$env:LINGTU_FACTORY_PARK_INSTANCE_PHASE = $phase" in runner
    assert "FactoryPark_HF.instances.stage.json" in runner
    assert '"Unreal-$phase.log"' in runner
    assert "'-NoCompile'" in runner
    assert "$env:UE_SKIP_UBT_SDK_SETUP = '1'" in runner
    assert "preflight is not clean" in runner
    assert "spawned or overlapped a build process" in runner
    assert "} finally {" in runner
    assert "$processStarted = $false" in runner
    assert "$processStarted = $process.Start()" in runner
    assert "if ($processStarted -and -not $process.HasExited)" in runner
    assert "$process.Kill($true)" in runner
    assert "$process.Dispose()" in runner
    assert "LINGTU_FACTORY_PARK_INSTANCE_READY" not in runner


def test_batch_actor_exposes_strict_serialized_and_live_transform_validation() -> None:
    header = BATCH_ACTOR_HEADER.read_text(encoding="utf-8")
    source = BATCH_ACTOR_SOURCE.read_text(encoding="utf-8")

    assert "TArray<FTransform> InstanceTransforms" in header
    assert "bool ValidateVisualGroup(" in header
    assert 'UFUNCTION(BlueprintCallable, Category="LingTuSim|FactoryPark")' in header
    assert "Component.GetInstanceTransform(" in source
    assert "Component.GetMaterial(0) != ExpectedMaterial" in source
    assert "LingTuSim::Visual::HasPresentationPolicy(Component)" in source
