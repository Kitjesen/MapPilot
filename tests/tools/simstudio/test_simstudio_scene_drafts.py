# ruff: noqa: D103, S101
"""Durable authoring contracts for validated SimStudio scene drafts."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import pytest
from tools.simstudio.http.app import create_app
from tools.simstudio.service.application import SimulationStudioService
from tools.simstudio.service.http import API_PREFIX
from tools.simstudio.service.models import RevisionConflict
from tools.simstudio.service.scene_draft_service import SceneDraftService
from tools.simstudio.service.scene_tools import FactoryParkSceneTool, SceneToolValidationError
from tools.simstudio.service.store import StudioStore

REPO_ROOT = Path(__file__).resolve().parents[3]


def _batch() -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.factory-park-element-batch.v1",
        "batch_id": "parking_safety",
        "description": "Studio-owned scene draft fixture.",
        "elements": [
            {
                "instance_key": "cone_01",
                "element_type": "traffic_cone",
                "surface_id": "parking_apron",
                "position_xy_m": [24.0, -44.0],
                "yaw_deg": 0.0,
            }
        ],
    }


def _scene_service(tmp_path: Path) -> SceneDraftService:
    return SceneDraftService(
        StudioStore(tmp_path / "studio"),
        FactoryParkSceneTool(REPO_ROOT),
    )


def _result(response: Any, status_code: int = 200) -> Any:
    assert response.status_code == status_code, response.text
    payload = response.json()
    assert payload["ok"] is True
    return payload["result"]


def test_scene_drafts_are_validated_durable_and_cas_versioned(tmp_path: Path) -> None:
    service = _scene_service(tmp_path)
    created = service.create_scene_draft(_batch(), idempotency_key="scene-create")

    assert len(created["id"]) == 32
    assert created["revision"] == 1
    assert created["status"] == "draft"
    assert created["payload"]["schema"] == "lingtu.sim.studio.scene-draft-payload.v1"
    assert created["payload"]["scene_tool"] == "factory-park-hf"
    assert created["payload"]["world_package"] == "factory_park_hf@1.0.0"
    assert created["payload"]["batch"] == _batch()
    assert created["payload"]["batch_digest"] == created["payload"]["validation"]["digest"]
    assert created["payload"]["layout_digest"]
    assert service.create_scene_draft(_batch(), idempotency_key="scene-create") == created

    restarted = SceneDraftService(
        StudioStore(tmp_path / "studio"),
        FactoryParkSceneTool(REPO_ROOT),
    )
    assert restarted.get_scene_draft(created["id"]) == created
    assert restarted.list_scene_drafts() == [created]

    changed = _batch()
    changed["description"] = "Revised draft."
    updated = restarted.update_scene_draft(
        created["id"],
        revision=1,
        batch=changed,
    )
    assert updated["revision"] == 2
    assert updated["payload"]["batch"]["description"] == "Revised draft."
    with pytest.raises(RevisionConflict):
        restarted.update_scene_draft(created["id"], revision=1, batch=_batch())


def test_invalid_scene_geometry_never_enters_the_store(tmp_path: Path) -> None:
    service = _scene_service(tmp_path)
    invalid = copy.deepcopy(_batch())
    invalid["elements"][0]["position_xy_m"] = [500.0, 500.0]

    with pytest.raises(SceneToolValidationError, match="footprint"):
        service.create_scene_draft(invalid)

    assert service.list_scene_drafts() == []


def test_scene_draft_http_contract_supports_create_list_get_and_cas_update(
    tmp_path: Path,
) -> None:
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient

    scene_service = _scene_service(tmp_path)
    service = SimulationStudioService(
        store=scene_service.store,
        scene_draft_service=scene_service,
    )
    with TestClient(create_app(service)) as client:
        created = _result(
            client.post(
                f"{API_PREFIX}/scene-drafts",
                json={"scene_tool": "factory-park-hf", "batch": _batch()},
                headers={"Idempotency-Key": "http-scene-create"},
            ),
            201,
        )
        assert _result(client.get(f"{API_PREFIX}/scene-drafts")) == [created]
        assert _result(client.get(f"{API_PREFIX}/scene-drafts/{created['id']}")) == created

        changed = _batch()
        changed["description"] = "HTTP revision"
        updated = _result(
            client.put(
                f"{API_PREFIX}/scene-drafts/{created['id']}",
                json={"revision": 1, "batch": changed},
            )
        )
        assert updated["revision"] == 2
        conflict = client.put(
            f"{API_PREFIX}/scene-drafts/{created['id']}",
            json={"revision": 1, "batch": _batch()},
        )
        assert conflict.status_code == 409
        assert conflict.json()["error"]["code"] == "SIMSTUDIO_CONFLICT"

        capabilities = _result(client.get(f"{API_PREFIX}/capabilities"))
        assert capabilities["read_models"]["scene_drafts"] == {
            "list": f"{API_PREFIX}/scene-drafts",
            "get": f"{API_PREFIX}/scene-drafts/{{scene_draft_id}}",
            "create": f"{API_PREFIX}/scene-drafts",
            "update": f"{API_PREFIX}/scene-drafts/{{scene_draft_id}}",
        }


@pytest.mark.parametrize("field", ["command", "cwd", "executable", "output_dir", "path"])
def test_scene_draft_http_rejects_process_and_filesystem_controls(
    tmp_path: Path,
    field: str,
) -> None:
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient

    scene_service = _scene_service(tmp_path)
    service = SimulationStudioService(
        store=scene_service.store,
        scene_draft_service=scene_service,
    )
    request = {"scene_tool": "factory-park-hf", "batch": _batch()}
    request["batch"][field] = "outside"
    with TestClient(create_app(service)) as client:
        response = client.post(f"{API_PREFIX}/scene-drafts", json=request)

    assert response.status_code == 422
    assert response.json()["error"]["code"] == "SIMSTUDIO_INVALID_REQUEST"
