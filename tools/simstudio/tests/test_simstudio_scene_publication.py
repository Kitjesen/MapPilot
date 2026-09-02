# ruff: noqa: D103, S101, S314
"""SceneDraft publication into immutable, versioned WorldPackages."""

from __future__ import annotations

import copy
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import pytest
import yaml
from sim.catalog.composer import SessionComposer
from sim.catalog.management import SimCatalog
from sim.catalog.resolver import CatalogResolver
from sim.catalog.importers import ImportCode, ImportFailure
from sim.runtime.coordinator.run_allocation import load_resolved_session_bundle
from tools.simstudio.http.app import create_app
from tools.simstudio.service.application import SimulationStudioService
from tools.simstudio.service.http import API_PREFIX
from tools.simstudio.service.models import RevisionConflict
from tools.simstudio.service.scene_draft_service import SceneDraftService
from tools.simstudio.service.scene_publication import ScenePublicationService
from tools.simstudio.service.scene_tools import FactoryParkSceneTool
from tools.simstudio.service.session_service import SessionAuthoringService
from tools.simstudio.service.store import StudioStore

REPO_ROOT = Path(__file__).resolve().parents[3]


def _batch(*, x_m: float = 24.0) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.factory-park-element-batch.v1",
        "batch_id": "parking_safety",
        "description": "Versioned scene publication fixture.",
        "elements": [
            {
                "instance_key": "cone_01",
                "element_type": "traffic_cone",
                "surface_id": "parking_apron",
                "position_xy_m": [x_m, -44.0],
                "yaw_deg": 0.0,
            }
        ],
    }


def _repository(tmp_path: Path) -> Path:
    repo = tmp_path / "repo"
    generated = repo / "sim" / "packages" / "worlds" / "factory_park_hf" / "generated"
    generated.mkdir(parents=True)
    source = REPO_ROOT / "sim" / "packages" / "worlds" / "factory_park_hf" / "generated"
    for name in ("expanded-layout.json", "heightfield_f32.bin"):
        shutil.copyfile(source / name, generated / name)
    return repo


def _services(tmp_path: Path) -> tuple[SceneDraftService, ScenePublicationService, Path]:
    repo = _repository(tmp_path)
    store = StudioStore(tmp_path / "studio")
    scene_tool = FactoryParkSceneTool(repo)
    drafts = SceneDraftService(store, scene_tool)
    publications = ScenePublicationService(
        store=store,
        scene_tool=scene_tool,
        repo_root=repo,
    )
    return drafts, publications, repo


def test_scene_draft_publishes_an_immutable_geometry_preserving_world_package(
    tmp_path: Path,
) -> None:
    drafts, publications, repo = _services(tmp_path)
    draft = drafts.create_scene_draft(_batch())

    result = publications.publish(
        draft["id"],
        revision=1,
        package={
            "id": "factory_park_safety_layout",
            "version": "1.0.0",
            "description": "FactoryPark safety layout",
        },
        idempotency_key="publish-scene-v1",
    )

    assert result["schema"] == "lingtu.sim.studio.scene-publication.v1"
    assert result["publication"]["package"]["ref"] == "factory_park_safety_layout@1.0.0"
    assert result["scene_draft"]["status"] == "published"
    assert result["scene_draft"]["revision"] == 2
    assert result["source"] == {
        "scene_draft_id": draft["id"],
        "scene_draft_revision": 1,
        "batch_digest": draft["payload"]["batch_digest"],
        "base_layout_digest": draft["payload"]["layout_digest"],
    }

    package_root = repo / "sim" / "packages" / "worlds" / "factory_park_safety_layout" / "1.0.0"
    manifest = yaml.safe_load((package_root / "world.package.yaml").read_text(encoding="utf-8"))
    cone = next(item for item in manifest["entities"] if item["entity_id"] == "element__parking_safety__cone_01")
    assert cone["geometry"] == {"shape": "cylinder", "radius_m": 0.2, "half_height_m": 0.35}
    assert cone["visual"]["mode"] == "runtime"
    baked_entities = [
        item
        for item in manifest["entities"]
        if item["entity_id"] != "element__parking_safety__cone_01"
    ]
    assert baked_entities
    assert {item["visual"]["mode"] for item in baked_entities} == {"level"}

    projection = json.loads(
        (package_root / "visual" / "world.visual-projection.json").read_text(
            encoding="utf-8"
        )
    )
    assert [item["entity_id"] for item in projection["entities"]] == [
        "element__parking_safety__cone_01"
    ]
    assert projection["entities"][0]["material"]["key"] == "container_orange"

    root = ET.parse(package_root / "world.xml").getroot()
    geom = root.find("./worldbody/body[@name='entity_element__parking_safety__cone_01']/geom")
    assert geom is not None
    assert geom.attrib["type"] == "cylinder"
    assert [float(value) for value in geom.attrib["size"].split()] == [0.2, 0.35]

    provenance = json.loads((package_root / "provenance" / "world.provenance.json").read_text(encoding="utf-8"))
    source_files = {item["path"] for item in provenance["source_intake"]["files"]}
    assert "scene-draft.json" in source_files
    assert CatalogResolver.from_repository(repo).find_package(
        "factory_park_safety_layout@1.0.0",
        kind="world",
    ).manifest_path == package_root / "world.package.yaml"

    assert publications.publish(
        draft["id"],
        revision=1,
        package={
            "id": "factory_park_safety_layout",
            "version": "1.0.0",
            "description": "FactoryPark safety layout",
        },
        idempotency_key="publish-scene-v1",
    ) == result


def test_scene_publication_rejects_stale_revision_before_catalog_mutation(tmp_path: Path) -> None:
    drafts, publications, repo = _services(tmp_path)
    draft = drafts.create_scene_draft(_batch())
    changed = copy.deepcopy(_batch())
    changed["description"] = "Revision two"
    drafts.update_scene_draft(draft["id"], revision=1, batch=changed)

    with pytest.raises(RevisionConflict):
        publications.publish(
            draft["id"],
            revision=1,
            package={"id": "stale_layout", "version": "1.0.0", "description": "Stale"},
        )

    assert not (repo / "sim" / "packages" / "worlds" / "stale_layout").exists()


def test_scene_publication_never_reuses_one_package_identity_for_different_content(
    tmp_path: Path,
) -> None:
    drafts, publications, _repo = _services(tmp_path)
    draft = drafts.create_scene_draft(_batch())
    first = publications.publish(
        draft["id"],
        revision=1,
        package={"id": "safety_layout", "version": "1.0.0", "description": "Safety"},
    )
    revised = drafts.update_scene_draft(draft["id"], revision=first["scene_draft"]["revision"], batch=_batch(x_m=25.0))

    with pytest.raises(ImportFailure) as exc_info:
        publications.publish(
            draft["id"],
            revision=revised["revision"],
            package={"id": "safety_layout", "version": "1.0.0", "description": "Safety"},
        )

    assert exc_info.value.code == ImportCode.PROMOTION_CONFLICT


def test_scene_publication_http_contract_is_strict_and_discoverable(tmp_path: Path) -> None:
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient

    drafts, publications, _repo = _services(tmp_path)
    service = SimulationStudioService(
        store=drafts.store,
        scene_draft_service=drafts,
        scene_publication_service=publications,
    )
    with TestClient(create_app(service)) as client:
        draft_response = client.post(
            f"{API_PREFIX}/scene-drafts",
            json={"scene_tool": "factory-park-hf", "batch": _batch()},
        )
        assert draft_response.status_code == 201
        draft = draft_response.json()["result"]
        response = client.post(
            f"{API_PREFIX}/scene-drafts/{draft['id']}/publish",
            json={
                "revision": 1,
                "package": {
                    "id": "http_safety_layout",
                    "version": "1.0.0",
                    "description": "HTTP safety layout",
                },
            },
            headers={"Idempotency-Key": "http-scene-publish"},
        )
        assert response.status_code == 201, response.text
        assert response.json()["result"]["publication"]["package"]["ref"] == "http_safety_layout@1.0.0"

        capabilities = client.get(f"{API_PREFIX}/capabilities").json()["result"]
        assert capabilities["read_models"]["scene_drafts"]["publish"] == (
            f"{API_PREFIX}/scene-drafts/{{scene_draft_id}}/publish"
        )

        rejected = client.post(
            f"{API_PREFIX}/scene-drafts/{draft['id']}/publish",
            json={
                "revision": 1,
                "package": {
                    "id": "unsafe_layout",
                    "version": "1.0.0",
                    "description": "Unsafe",
                    "output_dir": "D:/outside",
                },
            },
        )
        assert rejected.status_code == 422
        assert rejected.json()["error"]["code"] == "SIMSTUDIO_INVALID_REQUEST"


def test_application_refreshes_live_catalog_views_after_scene_publication(tmp_path: Path) -> None:
    repo = _repository(tmp_path)
    store = StudioStore(tmp_path / "studio")
    resolver = CatalogResolver.from_repository(repo)
    catalog = SimCatalog(resolver)
    composer = SessionComposer(resolver, artifact_root=store.root / "bundles")
    scene_tool = FactoryParkSceneTool(repo)
    session_service = SessionAuthoringService(store, composer, catalog)
    service = SimulationStudioService(
        catalog,
        composer,
        store=store,
        session_service=session_service,
        scene_draft_service=SceneDraftService(store, scene_tool),
        scene_publication_service=ScenePublicationService(
            store=store,
            scene_tool=scene_tool,
            repo_root=repo,
        ),
        repo_root=repo,
    )
    draft = service.create_scene_draft(_batch())

    published = service.publish_scene_draft(
        draft["id"],
        revision=draft["revision"],
        package={
            "id": "live_catalog_layout",
            "version": "1.0.0",
            "description": "Visible without a Studio restart",
        },
    )

    references = {item["package"]["ref"] for item in service.list_packages(kind="world")["packages"]}
    assert published["publication"]["package"]["ref"] in references
    assert service.catalog is not catalog
    assert service.composer is not composer
    assert service.composer.resolver is service.catalog.resolver
    assert session_service.catalog is service.catalog
    assert session_service.composer.resolver is service.catalog.resolver


def test_published_world_is_immediately_composable_with_integrity_bound_projection(
    tmp_path: Path,
) -> None:
    repo = _repository(tmp_path)
    for relative in (
        "sim/packages/robots/doso/thunder_v4",
        "sim/packages/controllers/doso/thunder_v4/locomotion",
        "sim/packages/sensor_rigs/doso/thunder_v4/navigation",
        "sim/packages/sensors",
    ):
        shutil.copytree(REPO_ROOT / relative, repo / relative)
    store = StudioStore(tmp_path / "studio")
    resolver = CatalogResolver.from_repository(repo)
    service = SimulationStudioService(
        SimCatalog(resolver),
        SessionComposer(resolver, artifact_root=store.root / "bundles"),
        store=store,
        session_service=SessionAuthoringService(
            store,
            SessionComposer(resolver, artifact_root=store.root / "untrusted"),
            SimCatalog(resolver),
        ),
        scene_draft_service=SceneDraftService(
            store,
            FactoryParkSceneTool(repo),
        ),
        scene_publication_service=ScenePublicationService(
            store=store,
            scene_tool=FactoryParkSceneTool(repo),
            repo_root=repo,
        ),
        repo_root=repo,
    )
    scene_draft = service.create_scene_draft(_batch())

    published = service.publish_scene_draft(
        scene_draft["id"],
        revision=scene_draft["revision"],
        package={
            "id": "immediate_compose_layout",
            "version": "1.0.0",
            "description": "World ref visible to the refreshed authoring service.",
        },
    )
    world_ref = published["publication"]["package"]["ref"]

    intent = {
        "schema": "lingtu.sim.session-intent.v1",
        "session": {
            "session_id": "published_world_compose",
            "mujoco_version": "3.10.0",
            "seed": 7,
            "world": world_ref,
            "robots": [
                {
                    "instance_id": "robot_01",
                    "package": "thunderv4@1.0.3",
                    "spawn": {
                        "position_m": [0.0, 0.0, 0.0],
                        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    },
                }
            ],
            "runtime": {
                "backend": "mujoco",
                "mode": "headless",
                "required_bindings": ["physics", "visual"],
            },
        },
    }
    draft = service.create_draft(intent)
    assert service.catalog.resolver.find_package(world_ref, kind="world").ref == world_ref

    bundle = service.compose_draft(draft["id"], revision=draft["revision"])
    bundle_dir = store.root / bundle["payload"]["bundle_path"]
    visual_plan = json.loads(
        (bundle_dir / "visual.plan.json").read_text(encoding="utf-8")
    )
    world = visual_plan["world"]
    projection_ref = world["projection"]
    projection_path = (repo / projection_ref["path"]).resolve()
    projection = json.loads(projection_path.read_text(encoding="utf-8"))

    assert world["package"]["id"] == "immediate_compose_layout"
    assert world["package"]["version"] == "1.0.0"
    assert projection_ref["schema"] == "lingtu.sim.world-visual-projection.v1"
    assert projection["schema"] == projection_ref["schema"]
    assert projection["package"]["id"] == "immediate_compose_layout"
    assert projection["package"]["version"] == "1.0.0"
    assert "sha256" not in projection_ref
    assert "digest" not in projection
    assert "artifact_content_digest" not in projection_ref

    loaded = load_resolved_session_bundle(bundle_dir, repo_root=repo)
    assert loaded.plans["visual.plan.json"]["world"]["projection"] == projection_ref
