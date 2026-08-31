# ruff: noqa: S101

from __future__ import annotations

from pathlib import Path

import pytest

from sim.runtime.visual.RobotSimUE.Scripts.build_thunderv4_rws01_preview import (
    EXPECTED_RUNTIME_MESHES,
    MESH_DESTINATION,
    PAYLOAD_VISUAL_POLICY,
    acceptance_camera_post_process_profile,
    payload_mount_transform_cm,
    preview_map_strategy,
    select_imported_static_mesh_paths,
)


def test_rws01_preview_mounts_one_visual_only_payload_on_thunder_base() -> None:
    assert payload_mount_transform_cm() == {
        "location_cm": [0.0, 0.0, 14.0],
        "rotation_deg": [0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
    }
    assert EXPECTED_RUNTIME_MESHES == (
        "SM_RWS01_EOSensor",
        "SM_RWS01_Launcher",
        "SM_RWS01_MountBase",
        "SM_RWS01_RecoilHousing",
        "SM_RWS01_YawFrame",
    )
    assert PAYLOAD_VISUAL_POLICY == {
        "collision_enabled": "NoCollision",
        "simulate_physics": False,
        "generate_overlap_events": False,
        "authority": "MuJoCo",
    }


def test_rws01_acceptance_camera_preserves_dark_pbr_material_detail() -> None:
    assert acceptance_camera_post_process_profile() == {
        "auto_exposure_min_brightness": 0.58,
        "auto_exposure_max_brightness": 0.58,
        "auto_exposure_bias": -2.0,
        "bloom_intensity": 0.05,
    }


def test_rws01_preview_consumes_only_the_accepted_conditioned_asset() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    source = (
        repo_root
        / "sim"
        / "runtime"
        / "visual"
        / "RobotSimUE"
        / "Scripts"
        / "build_thunderv4_rws01_preview.py"
    ).read_text(encoding="utf-8")

    assert "conditioned-v4" in source
    assert "conditioned-v3" not in source
    assert "rws-01-v002-runtime.glb" in source
    assert "rws-01-v002-runtime.fbx" not in source
    assert "FbxImportUI" not in source
    assert MESH_DESTINATION == "/Game/RobotSim/Payloads/FictionalRWS01/Runtime"
    assert "set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)" in source
    assert "set_simulate_physics(False)" in source
    assert 'set_editor_property("simulate_physics", False)' not in source
    assert 'set_editor_property("generate_overlap_events", False)' in source
    assert "LingTuPayloadPreview" in source
    assert "task.imported_object_paths" in source
    assert 'f"{MESH_DESTINATION}/{name}.{name}"' not in source


def test_rws01_preview_selects_only_meshes_from_the_current_glb_import() -> None:
    imported = [
        (
            "/Game/RobotSim/Payloads/FictionalRWS01/Meshes/"
            f"rws-01-v002-runtime/StaticMeshes/{name}.{name}"
        )
        for name in EXPECTED_RUNTIME_MESHES
    ]
    imported.append(
        "/Game/RobotSim/Payloads/FictionalRWS01/Meshes/"
        "rws-01-v002-runtime/Textures/tripo_image_0_base_color."
        "tripo_image_0_base_color"
    )

    selected = select_imported_static_mesh_paths(imported)

    assert tuple(selected) == EXPECTED_RUNTIME_MESHES
    assert all("/rws-01-v002-runtime/StaticMeshes/" in path for path in selected.values())


def test_rws01_preview_never_falls_back_to_stale_root_fbx_meshes() -> None:
    stale_root_assets = [
        f"/Game/RobotSim/Payloads/FictionalRWS01/Meshes/{name}.{name}"
        for name in EXPECTED_RUNTIME_MESHES
    ]

    with pytest.raises(RuntimeError, match="current GLB import"):
        select_imported_static_mesh_paths(stale_root_assets)


def test_rws01_preview_uses_one_loaded_map_without_copy_or_save_as() -> None:
    assert preview_map_strategy() == {
        "load": "/Game/RobotSim/Maps/ThunderV4_RuntimePreview",
        "persist": False,
    }

    source = (
        Path(__file__).resolve().parents[2]
        / "sim"
        / "runtime"
        / "visual"
        / "RobotSimUE"
        / "Scripts"
        / "build_thunderv4_rws01_preview.py"
    ).read_text(encoding="utf-8")
    assert "duplicate_asset" not in source
    assert "save_current_level" not in source
