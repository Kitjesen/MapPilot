"""Pure policy tests for the Blender forest asset conditioner."""

# ruff: noqa: S101

from __future__ import annotations

import copy
import json
import shutil
import struct
from pathlib import Path
from types import SimpleNamespace

import pytest

from sim.tools.assets import blender_forest_asset_conditioner as conditioner_module
from sim.tools.assets import static_prop_conditioning as conditioning_contract
from sim.tools.assets.blender_forest_asset_conditioner import (
    CONDITIONER_CONTRACT,
    REPORT_SCHEMA,
    _assert_visual_only_plan,
    _effective_decimation_ratio,
    _validate_import_budgets,
)
from sim.tools.assets.forest_asset_conditioning import build_forest_asset_conditioning_plan


def _visual_only_plan() -> dict:
    return {
        "conditioner_contract": CONDITIONER_CONTRACT,
        "binding": {
            "physics_authority": "mujoco_world_proxy",
            "visual_mesh_is_physics_proxy": False,
            "unreal": {
                "collision_profile": "NoCollision",
                "collision_enabled": False,
                "simulate_physics": False,
                "generate_overlap_events": False,
                "can_ever_affect_navigation": False,
            },
        },
        "qualification": {"state": "QUARANTINED"},
    }


@pytest.mark.parametrize(
    ("source", "ratio", "maximum", "expected"),
    [
        (2_000_000, 0.08, 120_000, 0.06),
        (1_000_000, 0.03, 45_000, 0.03),
        (2_000_000, 0.005, 8_000, 0.004),
    ],
)
def test_effective_decimation_ratio_obeys_ratio_and_hard_budget(
    source: int, ratio: float, maximum: int, expected: float
) -> None:
    assert _effective_decimation_ratio(source, ratio, maximum) == pytest.approx(expected)


@pytest.mark.parametrize(
    ("path", "value"),
    [
        (("binding", "visual_mesh_is_physics_proxy"), True),
        (("binding", "unreal", "collision_enabled"), True),
        (("binding", "unreal", "collision_profile"), "BlockAll"),
        (("qualification", "state"), "APPROVED"),
    ],
)
def test_visual_only_guard_fails_closed(path: tuple[str, ...], value: object) -> None:
    plan = copy.deepcopy(_visual_only_plan())
    target = plan
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value

    with pytest.raises(RuntimeError, match="visual-only"):
        _assert_visual_only_plan(plan)


def test_script_contract_never_authors_collision_geometry() -> None:
    source = (
        Path(__file__).parents[2] / "sim"
        / "tools"
        / "assets"
        / "blender_forest_asset_conditioner.py"
    ).read_text(encoding="utf-8")

    assert REPORT_SCHEMA == "lingtu.sim.forest-asset-conditioning-report.v1"
    assert "primitive_cube_add" not in source
    assert "collision_shape" not in source
    assert '"lingtu_asset_role"] = "VisualOnly"' in source
    assert '"lingtu_collision_profile"] = "NoCollision"' in source
    assert '"lingtu_qualification"] = "QUARANTINED"' in source


def test_script_uses_exact_axis_fit_and_grounding() -> None:
    source = (
        Path(__file__).parents[2] / "sim"
        / "tools"
        / "assets"
        / "blender_forest_asset_conditioner.py"
    ).read_text(encoding="utf-8")

    assert '"axis_scale": axis_scale' in source
    assert "target / source" in source
    assert "-minimum.z" in source


def test_import_budget_guard_rejects_excessive_scene() -> None:
    with pytest.raises(RuntimeError, match="vertex budget"):
        _validate_import_budgets(
            objects=1,
            meshes=1,
            vertices=2_000_001,
            triangles=1,
            materials=1,
            texture_pixels=1,
        )


def test_condition_failure_cleans_private_staging(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    staging = tmp_path / ".lingtu-forest-conditioning-failed"
    staging.mkdir()

    def fail(_plan: Path, **_kwargs: object) -> Path:
        conditioner_module._ACTIVE_STAGING = staging
        raise RuntimeError("boom")

    monkeypatch.setattr(conditioner_module, "_condition_impl", fail)
    with pytest.raises(RuntimeError, match="boom"):
        conditioner_module.condition(tmp_path / "plan.json")
    assert not staging.exists()
    shutil.rmtree(staging, ignore_errors=True)


def _forest_plan(root: Path) -> dict[str, object]:
    source = root / "source"
    source.mkdir(parents=True)
    model = source / "pine.glb"
    document = json.dumps({"asset": {"version": "2.0"}}).encode("utf-8")
    document += b" " * (-len(document) % 4)
    model.write_bytes(
        struct.pack("<4sII", b"glTF", 2, 20 + len(document))
        + struct.pack("<II", len(document), 0x4E4F534A)
        + document
    )
    task = source / "pine.task.json"
    task.write_text(
        json.dumps(
            {
                "task_id": "task-pine",
                "status": "success",
                "type": "text_to_model",
                "input": {"model_version": "v3.1-20260211", "pbr": True, "texture": True},
            }
        ),
        encoding="utf-8",
    )
    return build_forest_asset_conditioning_plan(
        artifact_root=root,
        asset_id="tripo-pine-01",
        asset_class="pine",
        source_model_path=model,
        task_path=task,
        output_directory="conditioned-v1",
        world_entity_id="forest-pine-01",
        unreal_asset_path="/Game/LingTu/Forest/SM_Pine_01",
    )


def _patch_forest_impl(
    monkeypatch: pytest.MonkeyPatch,
    plan: dict[str, object],
    *,
    fail_export: bool = False,
) -> None:
    material = SimpleNamespace(name="ForestPBR")
    lod = SimpleNamespace(material_slots=[SimpleNamespace(material=material)])
    image = SimpleNamespace(
        name="ForestTexture", packed_file=object(), size=(16, 16), type="IMAGE"
    )

    def save_mainfile(*, filepath: str) -> set[str]:
        Path(filepath).write_bytes(b"blend")
        return {"FINISHED"}

    fake_bpy = SimpleNamespace(
        context=SimpleNamespace(
            scene=SimpleNamespace(
                unit_settings=SimpleNamespace(system=None, scale_length=None)
            )
        ),
        data=SimpleNamespace(objects=[], images=[image]),
        ops=SimpleNamespace(
            import_scene=SimpleNamespace(gltf=lambda **_kwargs: {"FINISHED"}),
            file=SimpleNamespace(pack_all=lambda: {"FINISHED"}),
            wm=SimpleNamespace(save_as_mainfile=save_mainfile),
        ),
    )
    monkeypatch.setattr(conditioner_module, "_require_blender", lambda: None)
    monkeypatch.setattr(conditioner_module, "bpy", fake_bpy)
    monkeypatch.setattr(
        conditioner_module,
        "_read_plan",
        lambda _path: (
            plan,
            {
                "bytes": 1,
                "sha256": "0" * 64,
                "parent_identity": list(conditioning_contract._directory_identity(_path.parent)),
            },
        ),
    )
    monkeypatch.setattr(conditioner_module, "_clear_scene", lambda: None)
    monkeypatch.setattr(conditioner_module, "_enforce_scene_import_budgets", lambda _items: None)
    monkeypatch.setattr(conditioner_module, "_join_imported_meshes", lambda _items, _asset: lod)
    monkeypatch.setattr(
        conditioner_module,
        "_normalize",
        lambda *_args: ([1.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, 1.0]),
    )
    monkeypatch.setattr(
        conditioner_module,
        "_topology_report",
        lambda _obj: {"boundary_edges": 0, "non_manifold_edges": 0, "wire_edges": 0},
    )
    monkeypatch.setattr(conditioner_module, "_triangle_count", lambda _obj: 10)
    monkeypatch.setattr(conditioner_module, "_make_lods", lambda *_args: [lod, lod, lod])

    def export_lod(_obj: object, root: Path, output: dict[str, str]) -> dict[str, object]:
        if fail_export:
            raise RuntimeError("forest mid-pipeline failure")
        for key in ("glb", "fbx"):
            target = root / output[key]
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(key.encode("ascii"))
        return {"name": output["name"]}

    monkeypatch.setattr(conditioner_module, "_export_lod", export_lod)
    monkeypatch.setattr(
        conditioner_module,
        "_render_neutral_preview",
        lambda _lods, path: path.write_bytes(b"preview"),
    )


def test_forest_condition_impl_publishes_complete_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path / "asset"
    plan = _forest_plan(root)
    _patch_forest_impl(monkeypatch, plan)

    report = conditioner_module._condition_impl(root / "plan.json")

    output = root / "conditioned-v1"
    assert report == output / "conditioning-report.json"
    assert (output / "tripo-pine-01.blend").is_file()
    assert (output / "preview.png").is_file()


def test_forest_condition_impl_failure_leaves_no_mix(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path / "asset"
    plan = _forest_plan(root)
    _patch_forest_impl(monkeypatch, plan, fail_export=True)

    with pytest.raises(RuntimeError, match="forest mid-pipeline failure"):
        conditioner_module._condition_impl(root / "plan.json")
    assert not (root / "conditioned-v1").exists()
    assert not list(root.glob(".lingtu-forest-conditioning-*"))


def test_forest_condition_impl_preserves_existing_winner(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path / "asset"
    plan = _forest_plan(root)
    _patch_forest_impl(monkeypatch, plan)
    winner = root / "conditioned-v1"
    winner.mkdir()
    (winner / "winner.txt").write_text("keep", encoding="utf-8")

    with pytest.raises(RuntimeError, match="already exists"):
        conditioner_module._condition_impl(root / "plan.json")
    assert (winner / "winner.txt").read_text(encoding="utf-8") == "keep"


def test_forest_snapshot_rejects_parent_and_final_file_exchange(tmp_path: Path) -> None:
    root = tmp_path / "asset"
    plan = _forest_plan(root)
    original = root / "source"
    displaced = root / "source-original"
    original.rename(displaced)
    original.mkdir()
    (original / "pine.glb").write_bytes((displaced / "pine.glb").read_bytes() + b"changed")
    staging = root / "snapshot"
    staging.mkdir()

    with pytest.raises(RuntimeError, match="identity changed"):
        conditioner_module._snapshot_source(
            original / "pine.glb", plan["source"]["model"], staging
        )
