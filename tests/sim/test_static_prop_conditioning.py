# ruff: noqa: S101
"""Contracts for deterministic Blender conditioning of generated static props."""

from __future__ import annotations

import copy
import hashlib
import json
import struct
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import pytest

from sim.catalog.importers.contracts import digest_document
from sim.tools.assets import blender_static_prop_conditioner as static_conditioner
from sim.tools.assets import static_prop_conditioning as conditioning_contract
from sim.tools.assets.static_prop_conditioning import (
    _atomic_publish_no_replace,
    _validate_self_contained_glb,
    build_blender_conditioning_command,
    build_static_prop_conditioning_plan,
    topology_requires_review,
    validate_static_prop_conditioning_plan,
    write_static_prop_conditioning_plan,
)


def _source_fixture(root: Path) -> tuple[Path, Path]:
    source = root / "source"
    source.mkdir(parents=True)
    model = source / "model.glb"
    document = json.dumps({"asset": {"version": "2.0"}}).encode("utf-8")
    document += b" " * (-len(document) % 4)
    model.write_bytes(
        struct.pack("<4sII", b"glTF", 2, 20 + len(document))
        + struct.pack("<II", len(document), 0x4E4F534A)
        + document
    )
    task = source / "task.json"
    task.write_text(
        json.dumps(
            {
                "task_id": "task-jersey-001",
                "type": "text_to_model",
                "status": "success",
                "input": {
                    "model_version": "v3.1-20260211",
                    "pbr": True,
                    "texture": True,
                },
                "credits_consumed": 50,
            }
        ),
        encoding="utf-8",
    )
    return model, task


def _build_plan(root: Path) -> dict[str, object]:
    model, task = _source_fixture(root)
    return build_static_prop_conditioning_plan(
        artifact_root=root,
        asset_id="concrete-jersey-barrier-v001",
        source_model_path=model,
        task_path=task,
        output_directory="conditioned-v1",
        world_entity_id="element__operations_safety__gravel_barrier_west",
        semantic_class="jersey_barrier",
        target_dimensions_m=(2.5, 0.65, 0.9),
        source_axis_order=("x", "y", "z"),
        mujoco_box_proxy_size_m=(2.5, 0.65, 0.9),
        unreal_asset_path=(
            "/Game/RobotSim/Staging/Tripo/ConcreteJerseyBarrier/"
            "SM_ConcreteJerseyBarrier"
        ),
    )


def test_plan_is_portable_deterministic_and_keeps_one_physics_authority(
    tmp_path: Path,
) -> None:
    first = _build_plan(tmp_path / "asset")
    second = build_static_prop_conditioning_plan(
        artifact_root=tmp_path / "asset",
        asset_id="concrete-jersey-barrier-v001",
        source_model_path=tmp_path / "asset/source/model.glb",
        task_path=tmp_path / "asset/source/task.json",
        output_directory="conditioned-v1",
        world_entity_id="element__operations_safety__gravel_barrier_west",
        semantic_class="jersey_barrier",
        target_dimensions_m=(2.5, 0.65, 0.9),
        source_axis_order=("x", "y", "z"),
        mujoco_box_proxy_size_m=(2.5, 0.65, 0.9),
        unreal_asset_path=(
            "/Game/RobotSim/Staging/Tripo/ConcreteJerseyBarrier/"
            "SM_ConcreteJerseyBarrier"
        ),
    )

    assert first == second
    assert first["schema"] == "lingtu.sim.static-prop-conditioning-plan.v1"
    model_bytes = (tmp_path / "asset/source/model.glb").read_bytes()
    assert first["source"]["model"] == {
        "path": "source/model.glb",
        "bytes": len(model_bytes),
        "sha256": hashlib.sha256(model_bytes).hexdigest(),
    }
    task_bytes = (tmp_path / "asset/source/task.json").read_bytes()
    assert first["source"]["task"]["bytes"] == len(task_bytes)
    assert first["source"]["task"]["sha256"] == hashlib.sha256(task_bytes).hexdigest()
    assert first["geometry"] == {
        "target_dimensions_m": [2.5, 0.65, 0.9],
        "source_axis_order": ["x", "y", "z"],
        "normalization": "uniform_fit_center_xy_ground_z",
        "normal_policy": "recalculate_consistent_outside",
    }
    assert first["lods"] == [
        {"name": "LOD0", "triangle_ratio": 1.0},
        {"name": "LOD1", "triangle_ratio": 0.5},
        {"name": "LOD2", "triangle_ratio": 0.2},
    ]
    assert first["binding"]["physics"] == {
        "authority": "mujoco",
        "proxy": {"shape": "box", "size_m": [2.5, 0.65, 0.9]},
        "render_mesh_is_collider": False,
    }
    assert first["binding"]["unreal"] == {
        "asset_path": (
            "/Game/RobotSim/Staging/Tripo/ConcreteJerseyBarrier/"
            "SM_ConcreteJerseyBarrier"
        ),
        "collision_profile": "NoCollision",
        "collision_enabled": False,
        "simulate_physics": False,
        "generate_overlap_events": False,
        "can_ever_affect_navigation": False,
    }
    assert first["outputs"] == {
        "directory": "conditioned-v1",
        "blend": "conditioned-v1/concrete-jersey-barrier-v001.blend",
        "preview": "conditioned-v1/preview.png",
        "inspection": "conditioned-v1/conditioning-report.json",
        "lods": [
            {
                "name": "LOD0",
                "glb": "conditioned-v1/concrete-jersey-barrier-v001.LOD0.glb",
                "fbx": "conditioned-v1/concrete-jersey-barrier-v001.LOD0.fbx",
            },
            {
                "name": "LOD1",
                "glb": "conditioned-v1/concrete-jersey-barrier-v001.LOD1.glb",
                "fbx": "conditioned-v1/concrete-jersey-barrier-v001.LOD1.fbx",
            },
            {
                "name": "LOD2",
                "glb": "conditioned-v1/concrete-jersey-barrier-v001.LOD2.glb",
                "fbx": "conditioned-v1/concrete-jersey-barrier-v001.LOD2.fbx",
            },
        ],
    }
    assert first["qualification"]["state"] == "QUARANTINED"
    assert first["qualification"]["blockers"] == [
        "license_and_usage_rights_unverified",
        "unreal_import_not_verified",
    ]
    assert first["digest"] == digest_document(
        {key: value for key, value in first.items() if key != "digest"}
    )


@pytest.mark.parametrize(
    ("path", "value", "message"),
    [
        (("binding", "physics", "authority"), "unreal", "authority must be mujoco"),
        (
            ("binding", "physics", "render_mesh_is_collider"),
            True,
            "render_mesh_is_collider must be false",
        ),
        (
            ("binding", "unreal", "collision_profile"),
            "BlockAll",
            "collision_profile must be NoCollision",
        ),
    ],
)
def test_validator_rejects_a_second_collision_authority(
    tmp_path: Path,
    path: tuple[str, ...],
    value: object,
    message: str,
) -> None:
    document = copy.deepcopy(_build_plan(tmp_path / "asset"))
    target = document
    for component in path[:-1]:
        target = target[component]  # type: ignore[assignment,index]
    target[path[-1]] = value
    body = {key: item for key, item in document.items() if key != "digest"}
    document["digest"] = digest_document(body)

    with pytest.raises(ValueError, match=message):
        validate_static_prop_conditioning_plan(document)


def test_blender_command_uses_factory_startup_and_the_frozen_plan(tmp_path: Path) -> None:
    artifact_root = tmp_path / "asset"
    plan = _build_plan(artifact_root)
    plan_path = write_static_prop_conditioning_plan(
        artifact_root / "conditioning.plan.json",
        plan,
    )
    script = tmp_path / "sim/tools/assets/blender_static_prop_conditioner.py"
    script.parent.mkdir(parents=True)
    script.write_bytes(b"# frozen conditioner\n")

    command = build_blender_conditioning_command(
        "D:/Development/Blender/5.2/blender.exe",
        repo_root=tmp_path,
        plan_path=plan_path,
    )

    assert command == [
        "D:\\Development\\Blender\\5.2\\blender.exe",
        "--factory-startup",
        "--background",
        "--disable-autoexec",
        "--python-exit-code",
        "1",
        "--python",
        str((tmp_path / "sim/tools/assets/blender_static_prop_conditioner.py").resolve()),
        "--",
        "--plan",
        str(plan_path.resolve()),
        "--plan-bytes",
        str(plan_path.stat().st_size),
        "--plan-sha256",
        hashlib.sha256(plan_path.read_bytes()).hexdigest(),
        "--conditioner-contract",
        "lingtu.sim.static-prop-conditioner.v1",
        "--script-sha256",
        hashlib.sha256(
            (tmp_path / "sim/tools/assets/blender_static_prop_conditioner.py").read_bytes()
        ).hexdigest(),
    ]


def test_plan_rejects_non_glb_or_oversized_source(tmp_path: Path) -> None:
    root = tmp_path / "asset"
    model, task = _source_fixture(root)
    wrong = model.with_suffix(".gltf")
    wrong.write_bytes(model.read_bytes())
    with pytest.raises(ValueError, match="single-file .glb"):
        build_static_prop_conditioning_plan(
            artifact_root=root,
            asset_id="bad-source-v001",
            source_model_path=wrong,
            task_path=task,
            output_directory="conditioned-v1",
            world_entity_id="bad_source_01",
            semantic_class="prop",
            target_dimensions_m=(1, 1, 1),
            source_axis_order=("x", "y", "z"),
            mujoco_box_proxy_size_m=(1, 1, 1),
            unreal_asset_path="/Game/LingTu/Props/SM_Bad",
        )


def test_plan_rejects_glb_with_external_uri(tmp_path: Path) -> None:
    root = tmp_path / "asset"
    _, task = _source_fixture(root)
    model = root / "source/external.glb"
    document = json.dumps(
        {"asset": {"version": "2.0"}, "buffers": [{"byteLength": 4, "uri": "../x.bin"}]}
    ).encode("utf-8")
    document += b" " * (-len(document) % 4)
    model.write_bytes(
        struct.pack("<4sII", b"glTF", 2, 20 + len(document))
        + struct.pack("<II", len(document), 0x4E4F534A)
        + document
    )
    with pytest.raises(ValueError, match="external buffers URIs"):
        build_static_prop_conditioning_plan(
            artifact_root=root,
            asset_id="external-v001",
            source_model_path=model,
            task_path=task,
            output_directory="conditioned-v1",
            world_entity_id="external_01",
            semantic_class="prop",
            target_dimensions_m=(1, 1, 1),
            source_axis_order=("x", "y", "z"),
            mujoco_box_proxy_size_m=(1, 1, 1),
            unreal_asset_path="/Game/LingTu/Props/SM_External",
        )


def _glb_with_bin(document: dict[str, object], binary: bytes = b"abcd") -> bytes:
    encoded = json.dumps(document).encode("utf-8")
    encoded += b" " * (-len(encoded) % 4)
    binary += b"\x00" * (-len(binary) % 4)
    length = 12 + 8 + len(encoded) + 8 + len(binary)
    return (
        struct.pack("<4sII", b"glTF", 2, length)
        + struct.pack("<II", len(encoded), 0x4E4F534A)
        + encoded
        + struct.pack("<II", len(binary), 0x004E4942)
        + binary
    )


def test_glb_validator_accepts_bounded_embedded_image_closure() -> None:
    _validate_self_contained_glb(
        _glb_with_bin(
            {
                "asset": {"version": "2.0"},
                "buffers": [{"byteLength": 4}],
                "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": 4}],
                "images": [{"bufferView": 0, "mimeType": "image/png"}],
            }
        )
    )


@pytest.mark.parametrize(
    ("document", "message"),
    [
        ({"asset": {"version": "1.0"}}, "asset.version"),
        (
            {
                "asset": {"version": "2.0"},
                "buffers": [{"byteLength": 4}],
                "bufferViews": [{"buffer": 0, "byteOffset": 2, "byteLength": 4}],
            },
            "BIN closure",
        ),
        (
            {"asset": {"version": "2.0"}, "buffers": [{"byteLength": 0}]},
            "byteLength is invalid",
        ),
        (
            {"asset": {"version": "2.0"}, "buffers": [{"byteLength": 400}]},
            "declared buffer",
        ),
        (
            {
                "asset": {"version": "2.0"},
                "buffers": [{"byteLength": 4}],
                "bufferViews": [{"buffer": 0, "byteLength": 4}],
                "images": [{"bufferView": 2, "mimeType": "image/png"}],
            },
            "invalid bufferView",
        ),
    ],
)
def test_glb_validator_rejects_invalid_version_and_reference_boundaries(
    document: dict[str, object], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        _validate_self_contained_glb(_glb_with_bin(document))


def test_plan_publication_never_replaces_existing_file(tmp_path: Path) -> None:
    plan = _build_plan(tmp_path / "asset")
    target = tmp_path / "asset/conditioning.plan.json"
    target.write_text("winner\n", encoding="utf-8")

    with pytest.raises(FileExistsError):
        write_static_prop_conditioning_plan(target, plan)
    assert target.read_text(encoding="utf-8") == "winner\n"


def test_plan_builder_has_no_custom_profile_escape_hatch(tmp_path: Path) -> None:
    root = tmp_path / "asset"
    model, task = _source_fixture(root)
    profile = root / "custom-profile.json"
    profile.write_text("{}", encoding="utf-8")

    with pytest.raises(TypeError, match="profile_path"):
        build_static_prop_conditioning_plan(
            artifact_root=root,
            asset_id="custom-profile-v001",
            source_model_path=model,
            task_path=task,
            output_directory="conditioned-v1",
            world_entity_id="custom_profile_01",
            semantic_class="prop",
            target_dimensions_m=(1, 1, 1),
            source_axis_order=("x", "y", "z"),
            mujoco_box_proxy_size_m=(1, 1, 1),
            unreal_asset_path="/Game/LingTu/Props/SM_Custom",
            profile_path=profile,
        )


def test_directory_publication_race_has_exactly_one_winner(tmp_path: Path) -> None:
    target = tmp_path / "published"
    sources: list[Path] = []
    for index in range(8):
        source = tmp_path / f"candidate-{index}"
        source.mkdir()
        (source / "winner.txt").write_text(str(index), encoding="utf-8")
        sources.append(source)

    def publish(source: Path) -> bool:
        try:
            _atomic_publish_no_replace(source, target)
            return True
        except FileExistsError:
            return False

    with ThreadPoolExecutor(max_workers=len(sources)) as pool:
        results = list(pool.map(publish, sources))

    assert results.count(True) == 1
    assert (target / "winner.txt").read_text(encoding="utf-8") in {
        str(index) for index in range(len(sources))
    }


@pytest.mark.parametrize("fails", [False, True])
def test_static_condition_wrapper_cleans_private_staging_on_every_exit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, fails: bool
) -> None:
    staging = tmp_path / ".lingtu-static-conditioning-test"
    staging.mkdir()
    published = tmp_path / "published/report.json"
    published.parent.mkdir()
    published.write_text("{}", encoding="utf-8")

    def materialize(_plan: Path, **_kwargs: object) -> Path:
        static_conditioner._ACTIVE_STAGING = staging
        if fails:
            raise RuntimeError("boom")
        return published

    monkeypatch.setattr(static_conditioner, "_condition_impl", materialize)
    if fails:
        with pytest.raises(RuntimeError, match="boom"):
            static_conditioner.condition(tmp_path / "plan.json")
    else:
        assert static_conditioner.condition(tmp_path / "plan.json") == published
    assert not staging.exists()


def _patch_static_impl_runtime(
    monkeypatch: pytest.MonkeyPatch,
    plan: dict[str, object],
    *,
    fail_export: bool = False,
) -> list[Path]:
    staged_texture_roots: list[Path] = []
    material = SimpleNamespace(name="PBR", use_nodes=False, node_tree=None)
    lod = SimpleNamespace(material_slots=[SimpleNamespace(material=material)])

    def save_mainfile(*, filepath: str) -> set[str]:
        Path(filepath).write_bytes(b"blend")
        return {"FINISHED"}

    fake_bpy = SimpleNamespace(
        context=SimpleNamespace(
            scene=SimpleNamespace(
                unit_settings=SimpleNamespace(system=None, scale_length=None)
            )
        ),
        data=SimpleNamespace(objects=[]),
        ops=SimpleNamespace(
            import_scene=SimpleNamespace(gltf=lambda **_kwargs: {"FINISHED"}),
            wm=SimpleNamespace(save_as_mainfile=save_mainfile),
        ),
    )
    monkeypatch.setattr(static_conditioner, "_require_blender", lambda: None)
    monkeypatch.setattr(static_conditioner, "bpy", fake_bpy)
    monkeypatch.setattr(
        static_conditioner,
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
    monkeypatch.setattr(static_conditioner, "_clear_scene", lambda: None)
    monkeypatch.setattr(static_conditioner, "_enforce_scene_import_budgets", lambda _items: None)
    monkeypatch.setattr(static_conditioner, "_join_meshes", lambda _items, _asset: lod)
    monkeypatch.setattr(
        static_conditioner,
        "_normalize_geometry",
        lambda *_args, **_kwargs: ([1.0, 1.0, 1.0], [1.0, 1.0, 1.0], 1.0),
    )
    monkeypatch.setattr(
        static_conditioner,
        "_topology_report",
        lambda _obj: {"boundary_edges": 0, "non_manifold_edges": 0, "wire_edges": 0},
    )
    monkeypatch.setattr(static_conditioner, "_make_lods", lambda *_args: [lod, lod, lod])

    def save_textures(path: Path) -> list[dict[str, object]]:
        staged_texture_roots.append(path)
        (path / "texture.png").write_bytes(b"png")
        return []

    def export_lod(_obj: object, *, root: Path, output: dict[str, str]) -> dict[str, object]:
        if fail_export:
            raise RuntimeError("mid-pipeline failure")
        for key in ("glb", "fbx"):
            target = root / output[key]
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(key.encode("ascii"))
        return {"name": output["name"]}

    monkeypatch.setattr(static_conditioner, "_save_textures", save_textures)
    monkeypatch.setattr(static_conditioner, "_export_lod", export_lod)
    monkeypatch.setattr(
        static_conditioner,
        "_render_preview",
        lambda _lods, path: path.write_bytes(b"preview"),
    )
    return staged_texture_roots


def test_static_condition_impl_publishes_one_complete_staged_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path / "asset"
    plan = _build_plan(root)
    staged_texture_roots = _patch_static_impl_runtime(monkeypatch, plan)

    report = static_conditioner._condition_impl(root / "plan.json")

    output = root / "conditioned-v1"
    assert report == output / "conditioning-report.json"
    assert (output / "texture.png").is_file()
    assert (output / "concrete-jersey-barrier-v001.blend").is_file()
    assert (output / "preview.png").is_file()
    assert staged_texture_roots[0].name == "conditioned-v1"
    assert staged_texture_roots[0] != output


def test_static_condition_impl_failure_leaves_no_public_or_private_mix(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path / "asset"
    plan = _build_plan(root)
    _patch_static_impl_runtime(monkeypatch, plan, fail_export=True)

    with pytest.raises(RuntimeError, match="mid-pipeline failure"):
        static_conditioner._condition_impl(root / "plan.json")

    assert not (root / "conditioned-v1").exists()
    assert not list(root.glob(".lingtu-static-conditioning-*"))


def test_static_condition_impl_never_overwrites_a_winner(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path / "asset"
    plan = _build_plan(root)
    _patch_static_impl_runtime(monkeypatch, plan)
    winner = root / "conditioned-v1"
    winner.mkdir()
    (winner / "winner.txt").write_text("keep", encoding="utf-8")

    with pytest.raises(RuntimeError, match="already exists"):
        static_conditioner._condition_impl(root / "plan.json")
    assert (winner / "winner.txt").read_text(encoding="utf-8") == "keep"


def test_source_snapshot_rejects_final_file_exchange(tmp_path: Path) -> None:
    root = tmp_path / "asset"
    plan = _build_plan(root)
    model = root / "source/model.glb"
    model.write_bytes(model.read_bytes() + b"changed")
    staging = root / "snapshot"
    staging.mkdir()

    with pytest.raises(RuntimeError, match="identity changed"):
        static_conditioner._snapshot_source(model, plan["source"]["model"], staging)


def test_source_snapshot_rejects_parent_directory_exchange(tmp_path: Path) -> None:
    root = tmp_path / "asset"
    plan = _build_plan(root)
    original = root / "source"
    displaced = root / "source-original"
    original.rename(displaced)
    original.mkdir()
    (original / "model.glb").write_bytes((displaced / "model.glb").read_bytes() + b"changed")
    staging = root / "snapshot"
    staging.mkdir()

    with pytest.raises(RuntimeError, match="identity changed"):
        static_conditioner._snapshot_source(
            original / "model.glb", plan["source"]["model"], staging
        )


def test_plan_parent_exchange_is_detected_after_stable_plan_read(tmp_path: Path) -> None:
    root = tmp_path / "asset"
    plan = _build_plan(root)
    plan_path = write_static_prop_conditioning_plan(root / "plan.json", plan)
    _document, evidence = static_conditioner._read_plan(plan_path)
    displaced = tmp_path / "asset-original"
    root.rename(displaced)
    root.mkdir()

    with pytest.raises(RuntimeError, match="parent directory identity changed"):
        conditioning_contract._assert_directory_identity(
            root, tuple(evidence["parent_identity"])
        )


def test_handle_relative_publish_never_writes_or_deletes_replacement_root(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path / "asset"
    plan = _build_plan(root)
    _patch_static_impl_runtime(monkeypatch, plan)
    displaced = tmp_path / "asset-trusted"
    calls = 0
    exchange_blocked = False
    real_assert = conditioning_contract._assert_directory_identity

    def exchange_after_check(path: Path, expected: tuple[int, int]) -> None:
        nonlocal calls, exchange_blocked
        calls += 1
        if calls < 3:
            real_assert(path, expected)
            return
        try:
            root.rename(displaced)
        except PermissionError:
            exchange_blocked = True
            return
        root.mkdir()
        (root / "replacement.txt").write_text("untouched", encoding="utf-8")

    monkeypatch.setattr(static_conditioner, "_assert_directory_identity", exchange_after_check)

    static_conditioner._condition_impl(root / "plan.json")

    if exchange_blocked:
        trusted_location = displaced if displaced.exists() else root
        assert (trusted_location / "conditioned-v1/conditioning-report.json").is_file()
    else:
        assert (displaced / "conditioned-v1/conditioning-report.json").is_file()
        assert (root / "replacement.txt").read_text(encoding="utf-8") == "untouched"
        assert not (root / "conditioned-v1").exists()


def test_topology_review_uses_post_weld_defects_not_raw_uv_seams() -> None:
    assert not topology_requires_review(
        {
            "boundary_edges": 0,
            "non_manifold_edges": 0,
            "wire_edges": 0,
        }
    )
    assert topology_requires_review(
        {
            "boundary_edges": 12,
            "non_manifold_edges": 12,
            "wire_edges": 0,
        }
    )


@pytest.mark.parametrize("invalid", [-1, True, 1.5, "0"])
def test_topology_review_rejects_invalid_defect_counts(invalid: object) -> None:
    with pytest.raises(ValueError, match=r"topology\.boundary_edges"):
        topology_requires_review(
            {
                "boundary_edges": invalid,
                "non_manifold_edges": 0,
                "wire_edges": 0,
            }
        )


def test_plan_rejects_a_symlinked_source_file(tmp_path: Path) -> None:
    root = tmp_path / "asset"
    model, task = _source_fixture(root)
    linked_model = root / "source" / "linked-model.glb"
    try:
        linked_model.symlink_to(model)
    except OSError as exc:
        pytest.skip(f"symlink creation is unavailable: {exc}")

    with pytest.raises(ValueError, match="link-free"):
        build_static_prop_conditioning_plan(
            artifact_root=root,
            asset_id="linked-source-v001",
            source_model_path=linked_model,
            task_path=task,
            output_directory="conditioned-v1",
            world_entity_id="linked_source_01",
            semantic_class="test_prop",
            target_dimensions_m=(1.0, 1.0, 1.0),
            source_axis_order=("x", "y", "z"),
            mujoco_box_proxy_size_m=(1.0, 1.0, 1.0),
            unreal_asset_path="/Game/RobotSim/Staging/Linked/SM_Linked",
        )


def test_blender_command_rejects_a_symlinked_plan(tmp_path: Path) -> None:
    artifact_root = tmp_path / "asset"
    plan = _build_plan(artifact_root)
    real_plan = write_static_prop_conditioning_plan(
        artifact_root / "conditioning.plan.json",
        plan,
    )
    linked_plan = artifact_root / "linked-conditioning.plan.json"
    try:
        linked_plan.symlink_to(real_plan)
    except OSError as exc:
        pytest.skip(f"symlink creation is unavailable: {exc}")

    with pytest.raises(ValueError, match="link-free"):
        build_blender_conditioning_command(
            "D:/Development/Blender/5.2/blender.exe",
            repo_root=tmp_path,
            plan_path=linked_plan,
        )
