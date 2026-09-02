"""Static contracts for the robot-free OpenFieldRuntime UE recipe."""

# ruff: noqa: S101

from __future__ import annotations

import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT / "sim" / "runtime" / "visual" / "RobotSimUE" / "Scripts" / "build_open_field_runtime.py"
)


def _load_builder_module():
    spec = importlib.util.spec_from_file_location("lingtu_open_field_runtime_builder", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_open_field_runtime_validates_same_source_recipe() -> None:
    builder = _load_builder_module()
    world = builder._validate_world_recipe()

    assert builder.MAP_PATH == "/Game/RobotSim/Maps/OpenFieldRuntime"
    assert builder.SOURCE_LEVEL_PATH == "/Game/RobotSim/Maps/OpenField_HF"
    assert builder.TERRAIN_MESH_PATH == (
        "/Game/RobotSim/Worlds/OpenFieldHF/Meshes/"
        "SM_OpenField_HF_Terrain.SM_OpenField_HF_Terrain"
    )
    assert builder.TERRAIN_MATERIAL_PATH == (
        "/Game/RobotSim/Worlds/OpenFieldHF/Materials/"
        "M_OpenField_HF_Terrain.M_OpenField_HF_Terrain"
    )
    assert world["terrain_digest"] == "9074c291d2bf67b9a2fcbddb5855a79d5b8de1d2d725a9fd8769aad7f9191a81"
    assert world["heightfield_digest"] == "65e1631acf8265ce5318384b9b1364ecd07d0f29e2bfaf7f7f59bd2e2bc229c9"
    assert world["bounds_cm"] == (-8000.0, 8000.0, -8000.0, 8000.0)


def test_open_field_runtime_source_contract_is_robot_free_and_fail_closed() -> None:
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "PREPLACED_ROBOT_BINDINGS = 0" in source
    assert '"preplaced_robot_bindings": preplaced_robot_bindings' in source
    assert '"robot_actor_count": 0' in source
    assert '"robot_mesh_references": 0' in source
    assert "unreal.load_asset(TERRAIN_MESH_PATH)" in source
    assert "unreal.load_asset(TERRAIN_MATERIAL_PATH)" in source
    assert "required same-source terrain mesh is missing" in source
    assert "required same-source terrain material is missing" in source
    assert "same-source terrain mesh metadata mismatch" in source
    assert "StaticMesh {mesh.get_path_name()} has no assignable material slots" in source
    assert "set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)" in source
    assert "level_subsystem.new_level(MAP_PATH, False)" in source
    assert "_clear_current_level(actor_subsystem)" in source

    forbidden_tokens = (
        "unreal.LingTuSimBodyActor",
        "set_body_stable_id",
        "ROBOT_MESH_DESTINATION",
        "_import_robot_meshes",
        "_spawn_robot",
        "thunderv4",
        "ThunderV4",
        "Omni",
    )
    for token in forbidden_tokens:
        assert token not in source


def test_open_field_runtime_deterministic_actor_camera_and_evidence_constants() -> None:
    builder = _load_builder_module()

    assert builder.TERRAIN_LABEL == "OpenFieldRuntime_Terrain_SameSource"
    assert builder.SUN_LABEL == "OpenFieldRuntime_Sun"
    assert builder.SKYLIGHT_LABEL == "OpenFieldRuntime_SkyLight"
    assert builder.ATMOSPHERE_LABEL == "OpenFieldRuntime_SkyAtmosphere"
    assert builder.FOG_LABEL == "OpenFieldRuntime_AtmosphericFog"
    assert builder.POST_PROCESS_LABEL == "OpenFieldRuntime_ColorPipeline"
    assert builder.CAMERA_LABEL == "OpenFieldRuntime_SessionCamera"
    assert builder.CAMERA_LOCATION_CM == (420.0, -520.0, 245.0)
    assert builder.CAMERA_TARGET_CM == (0.0, 0.0, 72.0)
    assert builder.CAMERA_FOV_DEGREES == 42.0
    assert builder.EVIDENCE_PATH.name == "OpenFieldRuntime.evidence.json"
    assert builder.EVIDENCE_PATH.parent.name == "Automation"

    source = SCRIPT_PATH.read_text(encoding="utf-8")
    assert "unreal.AutoReceiveInput.PLAYER0" in source
    assert '"auto_activate_for_player": 0' in source
    assert "set_level_viewport_camera_info(location, rotation)" in source
    assert '"mujoco_motion_authority": True' in source
