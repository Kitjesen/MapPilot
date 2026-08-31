"""Contracts for visual-only conditioning of first-batch Tripo forest assets."""

# ruff: noqa: S101

from __future__ import annotations

import hashlib
import json
import struct
from pathlib import Path

import pytest

from sim.tools.assets.forest_asset_conditioning import (
    CONDITIONER_CONTRACT,
    build_forest_asset_conditioning_plan,
    build_forest_blender_conditioning_command,
    validate_forest_asset_conditioning_plan,
    write_forest_asset_conditioning_plan,
)


def _fixture(root: Path, asset_class: str) -> tuple[Path, Path]:
    model = root / f"source/{asset_class}.glb"
    task = root / f"source/{asset_class}.task.json"
    model.parent.mkdir()
    document = json.dumps({"asset": {"version": "2.0"}}).encode("utf-8")
    document += b" " * (-len(document) % 4)
    model.write_bytes(
        struct.pack("<4sII", b"glTF", 2, 20 + len(document))
        + struct.pack("<II", len(document), 0x4E4F534A)
        + document
    )
    task.write_text(
        json.dumps(
            {
                "task_id": f"task-{asset_class}",
                "status": "success",
                "type": "text_to_model",
                "input": {"model_version": "v3.0-20250812", "pbr": True, "texture": True},
            }
        ),
        encoding="utf-8",
    )
    return model, task


def _plan(root: Path, asset_class: str) -> dict:
    model, task = _fixture(root, asset_class)
    return build_forest_asset_conditioning_plan(
        artifact_root=root,
        asset_id=f"tripo-{asset_class}-01",
        asset_class=asset_class,
        source_model_path=model,
        task_path=task,
        output_directory="conditioned-v1",
        world_entity_id=f"forest-{asset_class}-01",
        unreal_asset_path=f"/Game/LingTu/Forest/SM_{asset_class.title()}_01",
    )


@pytest.mark.parametrize(
    ("asset_class", "dimensions", "lods"),
    [
        ("pine", [3.7, 3.7, 8.0], [(0.08, 120_000), (0.03, 45_000), (0.01, 15_000)]),
        ("birch", [2.3, 2.3, 7.0], [(0.08, 120_000), (0.03, 45_000), (0.01, 15_000)]),
        ("boulder", [1.6, 1.3, 1.1], [(0.06, 90_000), (0.02, 30_000), (0.005, 8_000)]),
        ("grass_clump", [0.55, 0.42, 0.22], [(0.12, 24_000), (0.04, 8_000), (0.01, 2_000)]),
        ("fern_clump", [0.78, 0.68, 0.43], [(0.15, 36_000), (0.05, 12_000), (0.012, 3_000)]),
        (
            "forest_floor_debris",
            [0.9, 0.68, 0.1],
            [(0.12, 30_000), (0.04, 10_000), (0.01, 2_500)],
        ),
    ],
)
def test_plan_freezes_forest_dimensions_lods_and_visual_authority(
    tmp_path: Path, asset_class: str, dimensions: list[float], lods: list[tuple[float, int]]
) -> None:
    plan = _plan(tmp_path, asset_class)

    assert plan["conditioner_contract"] == CONDITIONER_CONTRACT
    assert plan["asset"]["role"] == "forest_visual_candidate"
    assert plan["geometry"]["target_dimensions_m"] == dimensions
    assert plan["geometry"]["source_axis_order"] == ["x", "y", "z"]
    assert plan["geometry"]["imported_coordinate_system"] == {"up": "+Z", "unit": "meter"}
    assert plan["geometry"]["normalization"] == "axis_fit_center_xy_ground_z"
    assert [(lod["triangle_ratio"], lod["max_triangles"]) for lod in plan["lods"]] == lods
    assert plan["binding"]["physics_authority"] == "mujoco_world_proxy"
    assert plan["binding"]["visual_mesh_is_physics_proxy"] is False
    assert "proxy" not in plan["binding"]
    assert plan["binding"]["unreal"] == {
        "asset_path": f"/Game/LingTu/Forest/SM_{asset_class.title()}_01",
        "collision_profile": "NoCollision",
        "collision_enabled": False,
        "simulate_physics": False,
        "generate_overlap_events": False,
        "can_ever_affect_navigation": False,
    }
    assert plan["qualification"] == {
        "state": "QUARANTINED",
        "blockers": [
            "license_and_usage_rights_unverified",
            "unreal_import_not_verified",
            "topology_not_verified",
        ],
        "promotion_target": "WorldPackage.visual facet",
    }


def test_plan_binds_both_source_artifacts_by_bytes_and_hash(tmp_path: Path) -> None:
    plan = _plan(tmp_path, "pine")

    for name in ("model", "task"):
        source = plan["source"][name]
        path = tmp_path / source["path"]
        assert source["bytes"] == path.stat().st_size
        assert source["sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert plan["source"]["task"]["task_id"] == "task-pine"


@pytest.mark.parametrize("asset_class", ["grass_clump", "fern_clump", "forest_floor_debris"])
def test_understory_plan_binds_model_and_task_to_their_exact_source_bytes(
    tmp_path: Path, asset_class: str
) -> None:
    plan = _plan(tmp_path, asset_class)

    for name in ("model", "task"):
        source = plan["source"][name]
        path = tmp_path / source["path"]
        assert source["bytes"] == path.stat().st_size
        assert source["sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.mark.parametrize("asset_class", ["grass_clump", "fern_clump", "forest_floor_debris"])
def test_understory_command_uses_the_dedicated_forest_conditioner(
    tmp_path: Path, asset_class: str
) -> None:
    plan_path = write_forest_asset_conditioning_plan(
        tmp_path / "plan.json", _plan(tmp_path, asset_class)
    )
    command = build_forest_blender_conditioning_command(
        "blender", repo_root=Path(__file__).parents[2], plan_path=plan_path
    )

    assert Path(command[command.index("--python") + 1]).name == (
        "blender_forest_asset_conditioner.py"
    )
    assert command[command.index("--") : command.index("--plan-bytes")] == [
        "--",
        "--plan",
        str(plan_path.resolve()),
    ]
    assert "--disable-autoexec" in command
    assert "--script-sha256" in command


def test_validation_rejects_policy_tampering(tmp_path: Path) -> None:
    plan = _plan(tmp_path, "boulder")
    plan["lods"][0]["triangle_ratio"] = 0.5

    with pytest.raises(ValueError, match="digest"):
        validate_forest_asset_conditioning_plan(plan)


def test_builder_rejects_traversing_output_directory(tmp_path: Path) -> None:
    _fixture(tmp_path, "boulder")
    with pytest.raises(ValueError, match="canonical relative path"):
        build_forest_asset_conditioning_plan(
            artifact_root=tmp_path,
            asset_id="tripo-boulder-01",
            asset_class="boulder",
            source_model_path=tmp_path / "source/boulder.glb",
            task_path=tmp_path / "source/boulder.task.json",
            output_directory="../escaped",
            world_entity_id="forest-boulder-01",
            unreal_asset_path="/Game/LingTu/Forest/SM_Boulder_01",
        )


def test_command_builder_requires_repository_conditioner(tmp_path: Path) -> None:
    plan_path = write_forest_asset_conditioning_plan(tmp_path / "plan.json", _plan(tmp_path, "birch"))
    with pytest.raises(ValueError, match="repository forest conditioner"):
        build_forest_blender_conditioning_command(
            "blender", repo_root=tmp_path, plan_path=plan_path
        )


def test_command_builder_rejects_generic_static_prop_conditioner(tmp_path: Path) -> None:
    plan_path = write_forest_asset_conditioning_plan(tmp_path / "plan.json", _plan(tmp_path, "pine"))

    with pytest.raises(ValueError, match="repository forest conditioner"):
        build_forest_blender_conditioning_command(
            "blender", repo_root=tmp_path, plan_path=plan_path
        )


def test_source_axis_mapping_is_fixed_after_blender_glb_import(tmp_path: Path) -> None:
    model, task = _fixture(tmp_path, "pine")

    with pytest.raises(ValueError, match="x, y, z"):
        build_forest_asset_conditioning_plan(
            artifact_root=tmp_path,
            asset_id="tripo-pine-01",
            asset_class="pine",
            source_model_path=model,
            task_path=task,
            output_directory="conditioned-v1",
            world_entity_id="forest-pine-01",
            unreal_asset_path="/Game/LingTu/Forest/SM_Pine_01",
            source_axis_order=("y", "x", "z"),
        )


def test_plan_publication_never_replaces_existing_file(tmp_path: Path) -> None:
    plan = _plan(tmp_path, "pine")
    target = tmp_path / "plan.json"
    target.write_text("winner\n", encoding="utf-8")

    with pytest.raises(FileExistsError):
        write_forest_asset_conditioning_plan(target, plan)
    assert target.read_text(encoding="utf-8") == "winner\n"
