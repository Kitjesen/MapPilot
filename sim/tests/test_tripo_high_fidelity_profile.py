# ruff: noqa: S101
"""Contracts for Tripo-generated visual candidates."""

from __future__ import annotations

import copy
import hashlib
import json
import os
import shutil
import struct
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

import sim.tools.assets.tripo_visual_candidate as tripo_candidate_module
from sim.catalog.importers.contracts import digest_document
from sim.tools.assets.tripo_high_fidelity import (
    PROFILE_PATH,
    build_multiview_payload,
    load_profile,
)
from sim.tools.assets.tripo_visual_candidate import (
    _validate_glb_v2,
    build_visual_candidate_manifest,
    validate_visual_candidate_closure,
    validate_visual_candidate_manifest,
)
from sim.tools.assets.tripo_visual_candidate import (
    main as visual_candidate_main,
)


def _self_contained_glb() -> bytes:
    document = json.dumps(
        {"asset": {"version": "2.0"}, "scenes": [{"nodes": []}], "scene": 0},
        separators=(",", ":"),
    ).encode("utf-8")
    document += b" " * (-len(document) % 4)
    chunk = struct.pack("<II", len(document), 0x4E4F534A) + document
    return struct.pack("<4sII", b"glTF", 2, 12 + len(chunk)) + chunk


def _glb_with_document(document: dict[str, object], *, chunk_type: int = 0x4E4F534A) -> bytes:
    body = json.dumps(document, separators=(",", ":")).encode()
    body += b" " * (-len(body) % 4)
    chunk = struct.pack("<II", len(body), chunk_type) + body
    return struct.pack("<4sII", b"glTF", 2, 12 + len(chunk)) + chunk


def _glb_with_bin(document: dict[str, object], payload: bytes) -> bytes:
    json_chunk = _glb_with_document(document)[12:]
    padded = payload + b"\x00" * (-len(payload) % 4)
    bin_chunk = struct.pack("<II", len(padded), 0x004E4942) + padded
    return struct.pack("<4sII", b"glTF", 2, 12 + len(json_chunk) + len(bin_chunk)) + json_chunk + bin_chunk


@pytest.mark.parametrize(
    ("body", "message"),
    [
        (b"glTF", "truncated GLB header"),
        (struct.pack("<4sII", b"glTF", 1, 12), "GLB v2"),
        (
            struct.pack("<4sII", b"glTF", 2, 999) + struct.pack("<II", 0, 0x4E4F534A),
            "declared length",
        ),
        (
            struct.pack("<4sII", b"glTF", 2, 24) + struct.pack("<II", 8, 0x4E4F534A) + b"{}  ",
            "chunk boundary",
        ),
        (_glb_with_document({"asset": {"version": "2.0"}}, chunk_type=0x004E4942), "first GLB chunk"),
        (
            _glb_with_document(
                {"asset": {"version": "2.0"}, "buffers": [{"uri": "external.bin"}]}
            ),
            "external URI",
        ),
    ],
)
def test_glb_v2_parser_rejects_malformed_or_external_artifacts(
    body: bytes,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        _validate_glb_v2(body, "model")


@pytest.mark.parametrize(
    ("body", "message"),
    [
        (
            _glb_with_document({"asset": {"version": "2.0"}, "buffers": [{"byteLength": 4}]}),
            "requires one BIN",
        ),
        (
            _glb_with_bin({"asset": {"version": "2.0"}, "buffers": [{"byteLength": 1}]}, b"abcde"),
            "BIN length",
        ),
        (
            _glb_with_bin(
                {
                    "asset": {"version": "2.0"},
                    "buffers": [{"byteLength": 4}],
                    "bufferViews": [{"buffer": 0, "byteOffset": 3, "byteLength": 2}],
                },
                b"abcd",
            ),
            "bufferView exceeds",
        ),
        (
            _glb_with_bin(
                {
                    "asset": {"version": "2.0"},
                    "buffers": [{"byteLength": 4}],
                    "images": [{"bufferView": 0, "mimeType": "image/png"}],
                },
                b"abcd",
            ),
            "images references a missing bufferView",
        ),
    ],
)
def test_glb_v2_parser_enforces_binary_reference_closure(body: bytes, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        _validate_glb_v2(body, "model")


def test_glb_v2_parser_accepts_one_bounded_bin_with_spec_padding() -> None:
    body = _glb_with_bin(
        {
            "asset": {"version": "2.0"},
            "buffers": [{"byteLength": 5}],
            "bufferViews": [{"buffer": 0, "byteOffset": 1, "byteLength": 4}],
            "images": [{"bufferView": 0, "mimeType": "image/png"}],
        },
        b"abcde",
    )
    _validate_glb_v2(body, "model")


def test_profile_requires_high_fidelity_pbr_without_collision_authority() -> None:
    profile = load_profile()

    assert profile["api"] == {
        "base_url": "https://openapi.tripo3d.com/v3",
        "create_endpoint": "/generation/multiview-to-model",
        "task_endpoint_template": "/tasks/{task_id}",
        "model_url_ttl_seconds": 300,
        "download_immediately": True,
    }
    assert profile["generation"]["face_limit"] == 100_000
    assert profile["unreal_candidate_policy"] == {
        "canonical_candidate_root": "/Game/RobotSim/Staging/Tripo",
        "auto_spawn_in_production_map": False,
        "collision": "NoCollision",
        "nanite_for_static_mesh": True,
        "skeletal_mesh_allowed": False,
    }
    assert profile["ingress"]["ue_bridge"]["role"] == "editor_preview_only"
    assert profile["promotion"]["required_candidate_blockers"] == [
        "license_and_usage_rights_unverified",
        "unreal_import_not_verified",
    ]
    assert profile["cost_and_secret_policy"] == {
        "execute_requires_explicit_credit_confirmation": True,
        "persist_api_key": False,
        "log_authorization_header": False,
    }


def test_payload_is_deterministic_and_never_contains_credentials() -> None:
    payload = build_multiview_payload(
        {
            "right": "file_right",
            "front": "file_front",
            "back": "file_back",
        },
        model_seed=20260812,
        texture_seed=12082602,
    )

    assert payload["inputs"] == [
        {"front": "file_front"},
        {"back": "file_back"},
        {"right": "file_right"},
    ]
    assert payload["model"] == "v3.1-20260211"
    assert payload["geometry_quality"] == "detailed"
    assert payload["texture_quality"] == "extreme"
    assert payload["texture"] is payload["pbr"] is payload["auto_size"] is True
    assert payload["face_limit"] == 100_000
    assert not ({"api_key", "authorization", "token"} & set(payload))


@pytest.mark.parametrize(
    ("views", "message"),
    [
        ({"left": "file_left", "back": "file_back"}, "front view"),
        ({"front": "file_front"}, "at least two views"),
        ({"front": "file_same", "right": "file_same"}, "distinct images"),
        ({"front": "file_front", "top": "file_top"}, "unsupported Tripo view keys"),
    ],
)
def test_payload_rejects_inputs_that_cannot_support_multiview_reconstruction(
    views: dict[str, str],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        build_multiview_payload(views, model_seed=1, texture_seed=2)


def test_profile_validation_fails_closed_on_collision_authority(tmp_path: Path) -> None:
    profile = PROFILE_PATH.read_text(encoding="utf-8").replace(
        '"collision": "NoCollision"',
        '"collision": "BlockAll"',
    )
    path = tmp_path / "unsafe-profile.json"
    path.write_text(profile, encoding="utf-8")

    with pytest.raises(ValueError, match="second collision authority"):
        load_profile(path)


def _write_candidate_fixture(root: Path) -> tuple[Path, Path, Path, Path]:
    root.mkdir(parents=True, exist_ok=True)
    source_image = root / "front.png"
    source_image.write_bytes(b"front-reference-image")
    source_image_sha = hashlib.sha256(source_image.read_bytes()).hexdigest()
    right_image = root / "right.png"
    right_image.write_bytes(b"right-reference-image")
    right_image_sha = hashlib.sha256(right_image.read_bytes()).hexdigest()
    model = root / "model.glb"
    model.write_bytes(_self_contained_glb())

    model_sha = hashlib.sha256(model.read_bytes()).hexdigest()
    task = root / "task.json"
    task.write_text(
        json.dumps(
            {
                "task_id": "task-cabinet-001",
                "type": "multiview_to_model",
                "status": "success",
                "input": {
                    "model_version": "v3.1-20260211",
                    "model_seed": 101,
                    "texture_seed": 202,
                    "geometry_quality": "detailed",
                    "texture": True,
                    "pbr": True,
                    "texture_quality": "extreme",
                    "auto_size": True,
                    "export_uv": True,
                    "face_limit": 100_000,
                    "smart_low_poly": False,
                    "source_images": [
                        {
                            "view": "front",
                            "path": "front.png",
                            "bytes": source_image.stat().st_size,
                            "sha256": source_image_sha,
                        },
                        {
                            "view": "right",
                            "path": "right.png",
                            "bytes": right_image.stat().st_size,
                            "sha256": right_image_sha,
                        }
                    ],
                },
                "credits_consumed": 20,
            }
        ),
        encoding="utf-8",
    )
    inspection = root / "inspection.json"
    inspection.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.tripo-blender-inspection.v1",
                "source": {
                    "task_id": "task-cabinet-001",
                    "bytes": model.stat().st_size,
                    "sha256": model_sha,
                },
                "aabb_m": {"dimensions": [0.306125, 0.610811, 1.0]},
                "mesh_objects": 1,
                "vertices": 50_483,
                "triangles": 87_505,
                "uv_layers": 1,
                "non_manifold_edges": 12_861,
            }
        ),
        encoding="utf-8",
    )
    assessment = root / "assessment.json"
    assessment.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.visual-candidate-assessment.v1",
                "asset_id": "industrial-electrical-cabinet-v001",
                "task_id": "task-cabinet-001",
                "artifact": {
                    "path": "model.glb",
                    "bytes": model.stat().st_size,
                    "sha256": model_sha,
                },
                "qualification": {
                    "background_or_midground_visual": "pass",
                    "hero_closeup_visual": "fail",
                    "unreal_collision_mesh": "reject",
                    "mujoco_collision_authority": "required",
                },
            }
        ),
        encoding="utf-8",
    )
    return model, task, inspection, assessment


def _build_candidate_fixture_manifest(root: Path) -> dict[str, object]:
    model, task, inspection, assessment = _write_candidate_fixture(root)
    return build_visual_candidate_manifest(
        asset_id="industrial-electrical-cabinet-v001",
        world_entity_id="element__operations_safety__container_equipment_01",
        semantic_class="equipment_cabinet",
        model_path=model,
        task_path=task,
        inspection_path=inspection,
        assessment_path=assessment,
        source_axis_order=("y", "x", "z"),
        proxy_size_m=(1.2, 0.55, 1.8),
        unreal_asset_path=(
            "/Game/RobotSim/Staging/Tripo/IndustrialElectricalCabinet/"
            "SM_IndustrialElectricalCabinet"
        ),
    )


def _tamper_and_rehash(
    manifest: dict[str, object],
    path: tuple[str, ...],
    value: object,
) -> dict[str, object]:
    tampered = copy.deepcopy(manifest)
    target: dict[str, object] = tampered
    for component in path[:-1]:
        child = target[component]
        assert isinstance(child, dict)
        target = child
    target[path[-1]] = value
    body = {key: item for key, item in tampered.items() if key != "digest"}
    tampered["digest"] = digest_document(body)
    return tampered


def test_visual_candidate_binds_render_mesh_to_separate_mujoco_proxy(tmp_path: Path) -> None:
    model, task, inspection, assessment = _write_candidate_fixture(tmp_path)

    manifest = build_visual_candidate_manifest(
        asset_id="industrial-electrical-cabinet-v001",
        world_entity_id="element__operations_safety__container_equipment_01",
        semantic_class="equipment_cabinet",
        model_path=model,
        task_path=task,
        inspection_path=inspection,
        assessment_path=assessment,
        source_axis_order=("y", "x", "z"),
        proxy_size_m=(1.2, 0.55, 1.8),
        unreal_asset_path=(
            "/Game/RobotSim/Staging/Tripo/IndustrialElectricalCabinet/"
            "SM_IndustrialElectricalCabinet"
        ),
    )

    assert manifest["schema"] == "lingtu.sim.visual-asset-candidate.v1"
    assert manifest["binding"]["world_entity_id"] == (
        "element__operations_safety__container_equipment_01"
    )
    assert manifest["binding"]["physics"] == {
        "authority": "mujoco",
        "proxy": {"shape": "box", "size_m": [1.2, 0.55, 1.8]},
        "render_mesh_is_collider": False,
    }
    assert manifest["binding"]["unreal"] == {
        "asset_path": (
            "/Game/RobotSim/Staging/Tripo/IndustrialElectricalCabinet/"
            "SM_IndustrialElectricalCabinet"
        ),
        "collision_profile": "NoCollision",
        "collision_enabled": False,
        "simulate_physics": False,
        "generate_overlap_events": False,
        "can_ever_affect_navigation": False,
        "auto_spawn_in_production_map": False,
    }
    normalization = manifest["geometry"]["normalization"]
    assert normalization["policy"] == "uniform_fit_inside_mujoco_proxy"
    assert normalization["source_axis_order"] == ["y", "x", "z"]
    assert normalization["uniform_scale"] == pytest.approx(0.55 / 0.306125)
    assert normalization["normalized_dimensions_m"] == pytest.approx(
        [0.610811 * (0.55 / 0.306125), 0.55, 1.0 * (0.55 / 0.306125)]
    )
    assert all(
        observed <= limit + 1e-12
        for observed, limit in zip(
            normalization["normalized_dimensions_m"],
            manifest["binding"]["physics"]["proxy"]["size_m"],
        )
    )
    assert manifest["qualification"]["state"] == "QUARANTINED"
    assert set(manifest["qualification"]["blockers"]) >= {
        "license_and_usage_rights_unverified",
        "pbr_materials_missing",
        "topology_review_required",
        "unreal_import_not_verified",
    }
    assert manifest["digest"] == digest_document(
        {key: value for key, value in manifest.items() if key != "digest"}
    )
    evidence = manifest["evidence"]
    assert evidence["task"]["path"].startswith("evidence/task.")
    assert evidence["inspection"]["path"].startswith("evidence/inspection.")
    assert evidence["assessment"]["path"].startswith("evidence/assessment.")
    assert evidence["profile"]["path"].startswith("evidence/profile.")
    assert evidence["source_images"][0]["path"].startswith("sources/front.")
    assert evidence["source_images"][0]["sha256"] == hashlib.sha256(
        (tmp_path / "front.png").read_bytes()
    ).hexdigest()


def test_visual_candidate_validator_returns_the_canonical_manifest(tmp_path: Path) -> None:
    manifest = _build_candidate_fixture_manifest(tmp_path)

    assert validate_visual_candidate_manifest(manifest) == manifest


@pytest.mark.parametrize(
    ("path", "value", "message"),
    [
        (("asset", "role"), "robot_visual", "asset.role"),
        (("asset", "kind"), "skeletal_mesh", "asset.kind"),
        (("asset", "artifact", "path"), "../model.glb", "artifact.path"),
        (("asset", "artifact", "bytes"), 0, "artifact.bytes"),
        (("asset", "artifact", "sha256"), "not-a-digest", "artifact.sha256"),
        (("provenance", "provider"), "unknown", "provenance.provider"),
        (("provenance", "task_type"), "text_to_model", "provenance.task_type"),
        (("provenance", "model_seed"), True, "provenance.model_seed"),
        (
            ("geometry", "normalization", "source_axis_order"),
            ["x", "x", "z"],
            "source_axis_order",
        ),
        (("geometry", "normalization", "uniform_scale"), 0.25, "uniform_scale"),
        (
            ("geometry", "normalization", "normalized_dimensions_m"),
            [0.1, 0.2, 0.3],
            "normalized_dimensions_m",
        ),
        (("binding", "physics", "authority"), "unreal", "physics.authority"),
        (
            ("binding", "physics", "render_mesh_is_collider"),
            True,
            "render_mesh_is_collider",
        ),
        (("binding", "physics", "proxy", "shape"), "mesh", "proxy.shape"),
        (
            ("binding", "unreal", "collision_profile"),
            "BlockAll",
            "collision_profile",
        ),
        (("binding", "unreal", "collision_enabled"), True, "collision_enabled"),
        (("binding", "unreal", "simulate_physics"), True, "simulate_physics"),
        (
            ("binding", "unreal", "generate_overlap_events"),
            True,
            "generate_overlap_events",
        ),
        (
            ("binding", "unreal", "can_ever_affect_navigation"),
            True,
            "can_ever_affect_navigation",
        ),
        (
            ("binding", "unreal", "auto_spawn_in_production_map"),
            True,
            "auto_spawn_in_production_map",
        ),
        (("qualification", "state"), "QUALIFIED", "qualification.state"),
        (
            ("qualification", "blockers"),
            ["z_blocker", "a_blocker", "a_blocker"],
            "sorted and unique",
        ),
        (
            ("qualification", "promotion_target"),
            "RobotPackage.visual facet",
            "promotion_target",
        ),
        (
            ("qualification", "blockers"),
            ["pbr_materials_missing", "topology_review_required"],
            "required blockers",
        ),
    ],
)
def test_visual_candidate_validator_rejects_rehashed_semantic_tampering(
    tmp_path: Path,
    path: tuple[str, ...],
    value: object,
    message: str,
) -> None:
    manifest = _build_candidate_fixture_manifest(tmp_path)
    tampered = _tamper_and_rehash(manifest, path, value)

    with pytest.raises(ValueError, match=message):
        validate_visual_candidate_manifest(tampered)


def test_visual_candidate_validator_rejects_unknown_nested_fields(tmp_path: Path) -> None:
    manifest = _build_candidate_fixture_manifest(tmp_path)
    tampered = copy.deepcopy(manifest)
    binding = tampered["binding"]
    assert isinstance(binding, dict)
    unreal = binding["unreal"]
    assert isinstance(unreal, dict)
    unreal["use_complex_as_simple"] = True
    body = {key: item for key, item in tampered.items() if key != "digest"}
    tampered["digest"] = digest_document(body)

    with pytest.raises(ValueError, match=r"binding\.unreal keys"):
        validate_visual_candidate_manifest(tampered)


def test_visual_candidate_rejects_artifact_identity_drift(tmp_path: Path) -> None:
    model, task, inspection, assessment = _write_candidate_fixture(tmp_path)
    model.write_bytes(b"changed-after-inspection")

    with pytest.raises(ValueError, match="artifact identity|self-contained GLB|GLB v2"):
        build_visual_candidate_manifest(
            asset_id="industrial-electrical-cabinet-v001",
            world_entity_id="element__operations_safety__container_equipment_01",
            semantic_class="equipment_cabinet",
            model_path=model,
            task_path=task,
            inspection_path=inspection,
            assessment_path=assessment,
            source_axis_order=("y", "x", "z"),
            proxy_size_m=(1.2, 0.55, 1.8),
            unreal_asset_path="/Game/RobotSim/Staging/Tripo/SM_Cabinet",
        )


def test_profile_rejects_unknown_keys_and_nonofficial_api_routes(tmp_path: Path) -> None:
    profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    profile["unexpected"] = True
    path = tmp_path / "profile.json"
    path.write_text(json.dumps(profile), encoding="utf-8")
    with pytest.raises(ValueError, match="profile keys"):
        load_profile(path)

    profile.pop("unexpected")
    profile["api"]["base_url"] = "https://tripo.invalid/v3"
    path.write_text(json.dumps(profile), encoding="utf-8")
    with pytest.raises(ValueError, match="official Tripo API"):
        load_profile(path)


def test_visual_candidate_requires_one_self_contained_glb(tmp_path: Path) -> None:
    model, task, inspection, assessment = _write_candidate_fixture(tmp_path)
    renamed = model.with_suffix(".gltf")
    model.rename(renamed)
    with pytest.raises(ValueError, match="self-contained GLB"):
        build_visual_candidate_manifest(
            asset_id="industrial-electrical-cabinet-v001",
            world_entity_id="element__operations_safety__container_equipment_01",
            semantic_class="equipment_cabinet",
            model_path=renamed,
            task_path=task,
            inspection_path=inspection,
            assessment_path=assessment,
            source_axis_order=("y", "x", "z"),
            proxy_size_m=(1.2, 0.55, 1.8),
            unreal_asset_path="/Game/RobotSim/Staging/Tripo/SM_Cabinet",
        )


@pytest.mark.parametrize(
    "linked_input",
    ["model", "task", "inspection", "assessment", "profile", "source_image"],
)
def test_visual_candidate_rejects_symlinked_evidence(
    tmp_path: Path,
    linked_input: str,
) -> None:
    model, task, inspection, assessment = _write_candidate_fixture(tmp_path)
    inputs = {
        "model": model,
        "task": task,
        "inspection": inspection,
        "assessment": assessment,
        "profile": PROFILE_PATH,
    }
    profile = PROFILE_PATH
    if linked_input == "source_image":
        original = tmp_path / "front.png"
        target = tmp_path / "front-real.png"
        original.rename(target)
        linked = original
    else:
        target = inputs[linked_input]
        linked = tmp_path / f"linked-{target.name}"
    try:
        os.symlink(target, linked)
    except OSError:
        pytest.skip("symlink creation is unavailable")
    if linked_input in inputs:
        inputs[linked_input] = linked
    if linked_input == "profile":
        profile = linked
    with pytest.raises(ValueError, match=r"link|reparse"):
        build_visual_candidate_manifest(
            asset_id="industrial-electrical-cabinet-v001",
            world_entity_id="element__operations_safety__container_equipment_01",
            semantic_class="equipment_cabinet",
            model_path=inputs["model"],
            task_path=inputs["task"],
            inspection_path=inputs["inspection"],
            assessment_path=inputs["assessment"],
            source_axis_order=("y", "x", "z"),
            proxy_size_m=(1.2, 0.55, 1.8),
            unreal_asset_path="/Game/RobotSim/Staging/Tripo/SM_Cabinet",
            profile_path=profile,
        )


def _candidate_cli_argv(root: Path, output: Path) -> list[str]:
    model, task, inspection, assessment = _write_candidate_fixture(root)
    return [
        "--asset-id", "industrial-electrical-cabinet-v001",
        "--world-entity-id", "element__operations_safety__container_equipment_01",
        "--semantic-class", "equipment_cabinet",
        "--model", str(model),
        "--task", str(task),
        "--inspection", str(inspection),
        "--assessment", str(assessment),
        "--source-axis-order", "y", "x", "z",
        "--proxy-size-m", "1.2", "0.55", "1.8",
        "--unreal-asset-path", "/Game/RobotSim/Staging/Tripo/SM_Cabinet",
        "--output", str(output),
    ]


def test_cli_publication_is_no_replace_idempotent_and_relocatable(tmp_path: Path) -> None:
    output = tmp_path / "published" / "candidate.json"
    argv = _candidate_cli_argv(tmp_path / "inputs", output)
    assert visual_candidate_main(argv) == 0
    first = output.read_bytes()
    first_manifest = validate_visual_candidate_closure(output)
    assert visual_candidate_main(argv) == 0
    assert output.read_bytes() == first
    relocated = tmp_path / "relocated"
    shutil.copytree(output.parent, relocated)
    relocated_manifest = validate_visual_candidate_closure(relocated / output.name)
    assert relocated_manifest["digest"] == first_manifest["digest"]
    output.write_text("competing publication", encoding="utf-8")
    with pytest.raises(ValueError, match="already exists with different content"):
        visual_candidate_main(argv)


def test_concurrent_identical_writers_publish_one_complete_package(tmp_path: Path) -> None:
    output = tmp_path / "published" / "candidate.json"
    argv = _candidate_cli_argv(tmp_path / "inputs", output)
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(lambda _: visual_candidate_main(argv), range(2)))
    assert results == [0, 0]
    validate_visual_candidate_closure(output)


def test_closure_validator_rejects_artifact_drift_and_links(tmp_path: Path) -> None:
    output = tmp_path / "published" / "candidate.json"
    argv = _candidate_cli_argv(tmp_path / "inputs", output)
    assert visual_candidate_main(argv) == 0
    manifest = validate_visual_candidate_closure(output)
    artifact = output.parent / manifest["asset"]["artifact"]["path"]
    original = artifact.read_bytes()
    artifact.write_bytes(original[:-1] + bytes([original[-1] ^ 1]))
    with pytest.raises(ValueError, match="closure identity mismatch"):
        validate_visual_candidate_closure(output)
    artifact.write_bytes(original)
    real = artifact.with_name("real.glb")
    artifact.rename(real)
    try:
        os.symlink(real, artifact)
    except OSError:
        pytest.skip("file symlink creation is unavailable")
    with pytest.raises(ValueError, match=r"link|reparse"):
        validate_visual_candidate_closure(output)


@pytest.mark.parametrize("evidence_name", ["task", "inspection", "assessment", "profile"])
def test_closure_rejects_readdressed_and_resigned_semantic_evidence(
    tmp_path: Path,
    evidence_name: str,
) -> None:
    output = tmp_path / "published" / "candidate.json"
    argv = _candidate_cli_argv(tmp_path / "inputs", output)
    assert visual_candidate_main(argv) == 0
    manifest = json.loads(output.read_text(encoding="utf-8"))
    record = manifest["evidence"][evidence_name]
    old_path = output.parent / record["path"]
    document = json.loads(old_path.read_text(encoding="utf-8"))
    if evidence_name == "task":
        document["input"]["model_version"] = "attacker-model"
    elif evidence_name == "inspection":
        document["schema"] = "attacker.inspection.v1"
    elif evidence_name == "assessment":
        document["task_id"] = "attacker-task"
    else:
        document["generation"]["face_limit"] = 99_999
    body = json.dumps(document, separators=(",", ":")).encode()
    sha256 = hashlib.sha256(body).hexdigest()
    new_path = output.parent / "evidence" / f"{evidence_name}.{sha256}.json"
    new_path.write_bytes(body)
    record.update({"path": new_path.relative_to(output.parent).as_posix(), "bytes": len(body), "sha256": sha256})
    unsigned = {key: value for key, value in manifest.items() if key != "digest"}
    manifest["digest"] = digest_document(unsigned)
    output.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError):
        validate_visual_candidate_closure(output)


@pytest.mark.parametrize("linked_component", ["parent", "target"])
def test_publication_rejects_symlinked_parent_or_target(
    tmp_path: Path,
    linked_component: str,
) -> None:
    real = tmp_path / "real"
    real.mkdir()
    link = tmp_path / "linked"
    try:
        os.symlink(real, link, target_is_directory=True)
    except OSError:
        pytest.skip("directory symlink creation is unavailable")
    if linked_component == "parent":
        output = link / "package" / "candidate.json"
    else:
        output = link / "candidate.json"
    argv = _candidate_cli_argv(tmp_path / "inputs", output)
    with pytest.raises(ValueError, match=r"link|reparse"):
        visual_candidate_main(argv)


def test_publication_parent_swap_cannot_redirect_atomic_rename(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    container = tmp_path / "container"
    container.mkdir()
    output = container / "published" / "candidate.json"
    argv = _candidate_cli_argv(tmp_path / "inputs", output)
    real_rename = tripo_candidate_module._rename_directory_noreplace
    moved = tmp_path / "moved-container"
    swap_outcome: list[str] = []

    def racing_rename(
        parent: Path,
        parent_fd: int | None,
        source_name: str,
        target_name: str,
    ) -> None:
        try:
            parent.rename(moved)
        except OSError:
            swap_outcome.append("blocked")
        else:
            swap_outcome.append("moved")
        real_rename(parent, parent_fd, source_name, target_name)

    monkeypatch.setattr(
        tripo_candidate_module,
        "_rename_directory_noreplace",
        racing_rename,
    )
    if os.name == "nt":
        assert visual_candidate_main(argv) == 0
        assert swap_outcome == ["blocked"]
        validate_visual_candidate_closure(output)
    else:
        with pytest.raises(ValueError, match="publication parent changed"):
            visual_candidate_main(argv)
        assert swap_outcome == ["moved"]
        validate_visual_candidate_closure(moved / "published" / "candidate.json")
