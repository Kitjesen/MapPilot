# ruff: noqa: S101
"""Contracts for the offline UE5.8 robot asset-library projection."""

from __future__ import annotations

import hashlib
import json
import shutil
import struct
from pathlib import Path

import pytest

from sim.catalog.management import SimCatalog
from sim.catalog.resolver import CatalogResolver
from sim.catalog.importers.contracts import digest_document
from sim.tools.assets.build_ue_asset_library import (
    DEFAULT_PACKAGE_REFERENCES,
    UEAssetLibraryError,
    build_repository_ue_asset_library,
    build_ue_asset_library,
)
from sim.tools.assets.tripo_visual_candidate import build_visual_candidate_manifest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = REPO_ROOT / "sim" / "contracts" / "schemas" / "ue-assets.v1.json"


def _catalog() -> SimCatalog:
    return SimCatalog(
        CatalogResolver(
            REPO_ROOT,
            (
                REPO_ROOT / "sim" / "packages" / "robots" / "omni_cart",
                REPO_ROOT / "sim" / "packages" / "payloads" / "fictional_rws_01" / "1.0.0",
            ),
        )
    )


def _candidate_manifest(tmp_path: Path) -> Path:
    tmp_path.mkdir(parents=True, exist_ok=True)
    source_images = []
    for view in ("front", "right"):
        image = tmp_path / f"{view}.png"
        image.write_bytes(f"{view}-reference".encode())
        source_images.append(
            {
                "view": view,
                "path": image.name,
                "bytes": image.stat().st_size,
                "sha256": hashlib.sha256(image.read_bytes()).hexdigest(),
            }
        )
    model = tmp_path / "model.glb"
    glb_json = json.dumps(
        {"asset": {"version": "2.0"}, "scenes": [{"nodes": []}], "scene": 0},
        separators=(",", ":"),
    ).encode()
    glb_json += b" " * (-len(glb_json) % 4)
    glb_chunk = struct.pack("<II", len(glb_json), 0x4E4F534A) + glb_json
    model.write_bytes(struct.pack("<4sII", b"glTF", 2, 12 + len(glb_chunk)) + glb_chunk)
    model_sha256 = hashlib.sha256(model.read_bytes()).hexdigest()

    task = tmp_path / "task.json"
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
                    "source_images": source_images,
                },
                "credits_consumed": 20,
            }
        ),
        encoding="utf-8",
    )
    inspection = tmp_path / "inspection.json"
    inspection.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.tripo-blender-inspection.v1",
                "source": {
                    "task_id": "task-cabinet-001",
                    "bytes": model.stat().st_size,
                    "sha256": model_sha256,
                },
                "aabb_m": {"dimensions": [0.3, 0.6, 1.0]},
                "mesh_objects": 1,
                "vertices": 50_000,
                "triangles": 87_000,
                "uv_layers": 1,
                "non_manifold_edges": 12,
            }
        ),
        encoding="utf-8",
    )
    assessment = tmp_path / "assessment.json"
    assessment.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.visual-candidate-assessment.v1",
                "asset_id": "industrial-electrical-cabinet-v001",
                "task_id": "task-cabinet-001",
                "artifact": {
                    "path": "model.glb",
                    "bytes": model.stat().st_size,
                    "sha256": model_sha256,
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
        unreal_asset_path=("/Game/RobotSim/Staging/Tripo/IndustrialElectricalCabinet/SM_IndustrialElectricalCabinet"),
    )
    artifact_path = tmp_path / manifest["asset"]["artifact"]["path"]
    artifact_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(model, artifact_path)
    path = tmp_path / "visual-asset-candidate.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def test_default_repository_library_targets_current_thunderv4_visuals() -> None:
    assert DEFAULT_PACKAGE_REFERENCES == (
        ("robot", "thunderv4@1.0.3"),
        ("payload", "fictional_rws_01@1.0.0"),
    )

    library = build_repository_ue_asset_library(REPO_ROOT)
    entries = {entry["entry_id"]: entry for entry in library["entries"]}

    assert set(entries) == {
        "robot:thunderv4@1.0.3",
        "payload:fictional_rws_01@1.0.0",
    }
    assert entries["robot:thunderv4@1.0.3"]["visual"]["component_count"] == 21
    assert library["summary"]["quarantined_candidate_count"] == 0


def test_projects_catalog_packages_and_quarantined_candidates_without_promotion(
    tmp_path: Path,
) -> None:
    library = build_ue_asset_library(
        _catalog(),
        package_references=(
            ("robot", "omni_cart@1.0.0"),
            ("payload", "fictional_rws_01@1.0.0"),
        ),
        candidate_manifest_paths=(_candidate_manifest(tmp_path),),
    )

    assert library["schema"] == "lingtu.sim.ue-asset-library.v1"
    assert library["engine"] == {"name": "Unreal Engine", "major": 5, "minor": 8}
    assert library["summary"] == {
        "catalog_package_count": 2,
        "entry_count": 3,
        "quarantined_candidate_count": 1,
    }

    entries = {entry["entry_id"]: entry for entry in library["entries"]}
    robot = entries["robot:omni_cart@1.0.0"]
    assert robot["availability"] == "CATALOG_PACKAGE"
    assert robot["visual"]["component_count"] == 5
    assert robot["render_policy"] == {
        "physics_authority": "mujoco",
        "unreal_collision_profile": "NoCollision",
        "unreal_simulate_physics": False,
    }

    payload = entries["payload:fictional_rws_01@1.0.0"]
    assert payload["availability"] == "CATALOG_PACKAGE"
    assert payload["visual"]["component_count"] == 5
    assert payload["visual"]["binding"] == "PayloadVisual:FictionalRWS01"

    candidate = entries["candidate:industrial-electrical-cabinet-v001"]
    assert candidate["availability"] == "QUARANTINED"
    assert set(candidate["qualification"]["blockers"]) >= {
        "license_and_usage_rights_unverified",
        "pbr_materials_missing",
        "topology_review_required",
        "unreal_import_not_verified",
    }
    assert candidate["render_policy"] == robot["render_policy"]
    assert candidate["visual"]["unreal_assets"] == [
        "/Game/RobotSim/Staging/Tripo/IndustrialElectricalCabinet/SM_IndustrialElectricalCabinet"
    ]
    assert candidate["source"]["manifest"]["path"] == "visual-asset-candidate.json"
    assert candidate["source"]["artifact"]["path"].endswith(".glb")
    assert candidate["source"]["artifact"] == {
        "path": candidate["source"]["artifact"]["path"],
        "bytes": (tmp_path / candidate["source"]["artifact"]["path"]).stat().st_size,
    }

    serialized = json.dumps(library, sort_keys=True)
    for removed in (
        "catalog_content_sha256",
        "package_artifact_sha256",
        "manifest_sha256",
        "package_fingerprint",
        "content/sha256/",
    ):
        assert removed not in serialized


def test_asset_library_has_a_versioned_schema(tmp_path: Path) -> None:
    from tests.sim.test_sim_plan_schemas import _validate

    library = build_ue_asset_library(
        _catalog(),
        package_references=(("robot", "omni_cart@1.0.0"),),
        candidate_manifest_paths=(_candidate_manifest(tmp_path),),
    )
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))

    assert schema["$id"] == "lingtu.sim.ue-asset-library.v1"
    _validate(library, schema, schema)


def test_external_candidate_location_does_not_change_library_identity(
    tmp_path: Path,
) -> None:
    first = build_ue_asset_library(
        _catalog(),
        package_references=(),
        candidate_manifest_paths=(_candidate_manifest(tmp_path / "first"),),
    )
    second = build_ue_asset_library(
        _catalog(),
        package_references=(),
        candidate_manifest_paths=(_candidate_manifest(tmp_path / "second"),),
    )

    assert first == second


def test_candidate_cannot_gain_navigation_authority(tmp_path: Path) -> None:
    manifest_path = _candidate_manifest(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["binding"]["unreal"]["can_ever_affect_navigation"] = True
    body = {key: value for key, value in manifest.items() if key != "digest"}
    manifest["digest"] = digest_document(body)
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(
        UEAssetLibraryError,
        match=r"binding\.unreal\.can_ever_affect_navigation",
    ):
        build_ue_asset_library(
            _catalog(),
            package_references=(),
            candidate_manifest_paths=(manifest_path,),
        )


def test_robot_projection_is_revalidated_after_catalog_construction(
    tmp_path: Path,
) -> None:
    package_root = tmp_path / "sim" / "packages" / "robots" / "omni_cart"
    model_root = tmp_path / "sim" / "packages" / "robots" / "omni_cart"
    shutil.copytree(REPO_ROOT / "sim" / "packages" / "robots" / "omni_cart", package_root)
    shutil.copytree(REPO_ROOT / "sim" / "packages" / "robots" / "omni_cart", model_root)
    catalog = SimCatalog(CatalogResolver(tmp_path, (package_root,)))

    projection_path = package_root / "visual" / "robot.visual-projection.json"
    projection = json.loads(projection_path.read_text(encoding="utf-8"))
    projection["components"][0]["local_transform"]["scale"] = ["invalid", 1.0, 1.0]
    projection_path.write_text(json.dumps(projection), encoding="utf-8")

    with pytest.raises(UEAssetLibraryError, match="canonical visual projection"):
        build_ue_asset_library(
            catalog,
            package_references=(("robot", "omni_cart@1.0.0"),),
        )


def test_candidate_role_is_not_trusted_from_a_rehashed_manifest(tmp_path: Path) -> None:
    manifest_path = _candidate_manifest(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["asset"]["role"] = "robot"
    body = {key: value for key, value in manifest.items() if key != "digest"}
    manifest["digest"] = digest_document(body)
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(UEAssetLibraryError, match=r"asset\.role"):
        build_ue_asset_library(
            _catalog(),
            package_references=(),
            candidate_manifest_paths=(manifest_path,),
        )
