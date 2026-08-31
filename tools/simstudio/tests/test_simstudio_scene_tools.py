# ruff: noqa: D103,S101
"""HTTP contracts for the read-only FactoryPark_HF SceneTool slice."""

from __future__ import annotations

import copy
import hashlib
import json
import subprocess
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from tools.simstudio.service.http import API_PREFIX, create_app

REPO_ROOT = Path(__file__).resolve().parents[3]
SCENE_TOOL_ROOT = f"{API_PREFIX}/scene-tools/factory-park-hf"


def _client(repo_root: Path = REPO_ROOT) -> Any:
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient

    service = SimpleNamespace(
        package_service=SimpleNamespace(repo_root=repo_root),
    )
    return TestClient(create_app(service))


def _result(response: Any, status_code: int = 200) -> Any:
    assert response.status_code == status_code, response.text
    payload = response.json()
    assert payload["ok"] is True
    return payload["result"]


def _error(response: Any, status_code: int = 422) -> dict[str, Any]:
    assert response.status_code == status_code, response.text
    payload = response.json()
    assert payload["ok"] is False
    assert payload["error"] == payload["diagnostics"][0]
    return payload["error"]


def _batch() -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.factory-park-element-batch.v1",
        "batch_id": "studio_preview",
        "description": "Read-only SceneTool validation fixture.",
        "elements": [
            {
                "instance_key": "parking_cone",
                "element_type": "traffic_cone",
                "surface_id": "parking_apron",
                "position_xy_m": [24.0, -44.0],
            },
            {
                "instance_key": "parking_guide",
                "element_type": "lane_marker",
                "surface_id": "parking_apron",
                "position_xy_m": [25.0, -34.0],
                "yaw_deg": 90.0,
            },
        ],
    }


def _write_authoritative_layout(
    repo_root: Path,
    layout: dict[str, Any],
    *,
    refresh_digest: bool,
) -> None:
    if refresh_digest:
        layout_without_digest = dict(layout)
        layout_without_digest.pop("layout_digest", None)
        payload = (
            json.dumps(
                layout_without_digest,
                ensure_ascii=False,
                sort_keys=True,
                indent=2,
                separators=(",", ": "),
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
        layout["layout_digest"] = hashlib.sha256(payload).hexdigest()
    target = repo_root / "sim/worlds/factory_park_hf/generated/expanded-layout.json"
    target.parent.mkdir(parents=True)
    target.write_text(json.dumps(layout), encoding="utf-8")


def test_factory_park_scene_tool_catalog_is_read_only_and_authority_explicit() -> None:
    with _client() as client:
        catalog = _result(client.get(f"{SCENE_TOOL_ROOT}/catalog"))

    assert catalog["schema"] == "lingtu.sim.studio.scene-tool-catalog.v1"
    assert catalog["scene_tool"] == "factory-park-hf"
    assert catalog["world_package"] == "factory_park_hf@1.0.0"
    assert catalog["element_batch_schema"] == "lingtu.sim.factory-park-element-batch.v1"
    assert catalog["read_only"] is True
    assert catalog["executes_runtime"] is False
    assert set(catalog["element_types"]) == {
        "equipment_cabinet",
        "fire_cabinet",
        "industrial_drum",
        "jersey_barrier",
        "lane_marker",
        "pallet_stack",
        "safety_bollard",
        "safety_sign",
        "traffic_cone",
        "wheel_stop",
    }
    assert {item["authority"] for item in catalog["element_types"].values()} == {
        "PhysicsShared",
        "VisualOnly",
    }
    assert catalog["layout_digest"]
    parking = next(
        surface for surface in catalog["surfaces"] if surface["surface_id"] == "parking_apron"
    )
    assert parking == {
        "surface_id": "parking_apron",
        "semantic_class": "parking_area",
        "position_xy_m": [25.0, -34.0],
        "size_xy_m": [38.0, 34.0],
        "yaw_deg": 0.0,
    }
    assert catalog["spawn"] == {
        "position_xy_m": [0.0, -76.0],
        "clearance_radius_m": 4.0,
    }


def test_factory_park_scene_tool_validates_to_stable_ids_authority_and_digest() -> None:
    batch = _batch()

    with _client() as client:
        first = _result(
            client.post(f"{SCENE_TOOL_ROOT}/element-batches/validate", json=batch)
        )
        second = _result(
            client.post(f"{SCENE_TOOL_ROOT}/element-batches/validate", json=batch)
        )

    assert first == second
    assert first["schema"] == "lingtu.sim.studio.scene-tool-validation.v1"
    assert first["scene_tool"] == "factory-park-hf"
    assert first["valid"] is True
    assert first["batch_id"] == "studio_preview"
    assert len(first["digest"]) == 64
    assert first["layout_digest"]
    assert first["stable_ids"] == [
        "element__studio_preview__parking_cone",
        "element__studio_preview__parking_guide",
    ]
    assert first["elements"] == [
        {
            "stable_id": "element__studio_preview__parking_cone",
            "element_type": "traffic_cone",
            "authority": "PhysicsShared",
        },
        {
            "stable_id": "element__studio_preview__parking_guide",
            "element_type": "lane_marker",
            "authority": "VisualOnly",
        },
    ]
    assert first["diagnostics"] == []
    serialized = json.dumps(first, sort_keys=True)
    assert str(REPO_ROOT) not in serialized
    assert "expanded-layout.json" not in serialized


@pytest.mark.parametrize(
    ("location", "field", "value"),
    [
        ("batch", "command", ["blender", "--background"]),
        ("batch", "cwd", "../outside"),
        ("batch", "env", {"LINGTU_ENV": "real"}),
        ("batch", "executable", "UnrealEditor.exe"),
        ("batch", "path", "C:/outside/world.blend"),
        ("element", "command", "spawn_actor"),
        ("element", "path", "../outside.obj"),
    ],
)
def test_factory_park_scene_tool_rejects_process_and_path_controls_before_compile(
    location: str,
    field: str,
    value: object,
) -> None:
    batch = copy.deepcopy(_batch())
    target = batch if location == "batch" else batch["elements"][0]
    target[field] = value

    with _client() as client:
        error = _error(
            client.post(f"{SCENE_TOOL_ROOT}/element-batches/validate", json=batch)
        )

    assert error["code"] == "SIMSTUDIO_INVALID_REQUEST"
    assert "request validation failed" in error["message"]


def test_factory_park_scene_tool_reports_compiler_geometry_diagnostics() -> None:
    batch = _batch()
    batch["elements"][0]["position_xy_m"] = [500.0, 500.0]

    with _client() as client:
        error = _error(
            client.post(f"{SCENE_TOOL_ROOT}/element-batches/validate", json=batch)
        )

    assert error == {
        "code": "SIMSTUDIO_SCENE_ELEMENT_BATCH_INVALID",
        "message": "element footprint does not fit inside support surface",
        "details": {"scene_tool": "factory-park-hf"},
    }
    assert str(REPO_ROOT) not in json.dumps(error, sort_keys=True)


def test_factory_park_scene_tool_rejects_stale_authoritative_layout_digest(
    tmp_path: Path,
) -> None:
    source = REPO_ROOT / "sim/worlds/factory_park_hf/generated/expanded-layout.json"
    layout = json.loads(source.read_text(encoding="utf-8"))
    layout["checkpoints"][0]["label"] = "tampered-without-digest-update"
    _write_authoritative_layout(tmp_path, layout, refresh_digest=False)

    with _client(tmp_path) as client:
        error = _error(
            client.post(f"{SCENE_TOOL_ROOT}/element-batches/validate", json=_batch()),
            500,
        )

    assert error["code"] == "SIMSTUDIO_INTERNAL_ERROR"
    assert error["message"] == "factory-park-hf authoritative layout digest is stale"
    assert str(tmp_path) not in json.dumps(error, sort_keys=True)


def test_factory_park_scene_tool_attributes_malformed_layout_to_the_server(
    tmp_path: Path,
) -> None:
    source = REPO_ROOT / "sim/worlds/factory_park_hf/generated/expanded-layout.json"
    layout = json.loads(source.read_text(encoding="utf-8"))
    parking_apron = next(
        item for item in layout["objects"] if item["id"] == "parking_apron"
    )
    parking_apron.pop("position_m")
    _write_authoritative_layout(tmp_path, layout, refresh_digest=True)

    with _client(tmp_path) as client:
        error = _error(
            client.post(f"{SCENE_TOOL_ROOT}/element-batches/validate", json=_batch()),
            500,
        )

    assert error["code"] == "SIMSTUDIO_INTERNAL_ERROR"
    assert error["message"] == "factory-park-hf authoritative layout objects are invalid"
    assert str(tmp_path) not in json.dumps(error, sort_keys=True)


def test_factory_park_scene_tool_validation_never_writes_or_starts_a_process(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    original_open = Path.open

    def guarded_open(path: Path, mode: str = "r", *args: Any, **kwargs: Any) -> Any:
        if any(flag in mode for flag in ("w", "a", "x", "+")):
            raise AssertionError(f"SceneTool attempted a file write with mode {mode!r}")
        return original_open(path, mode, *args, **kwargs)

    def reject_process(*args: Any, **kwargs: Any) -> Any:
        del kwargs
        raise AssertionError(f"SceneTool attempted process execution: {args!r}")

    with _client() as client:
        monkeypatch.setattr(Path, "open", guarded_open)
        monkeypatch.setattr(subprocess, "Popen", reject_process)
        monkeypatch.setattr(subprocess, "run", reject_process)
        result = _result(
            client.post(
                f"{SCENE_TOOL_ROOT}/element-batches/validate",
                json=_batch(),
            )
        )

    assert result["valid"] is True


def test_factory_park_scene_tool_openapi_body_is_strict_json_without_paths() -> None:
    app = create_app(SimpleNamespace(package_service=SimpleNamespace(repo_root=REPO_ROOT)))
    document = app.openapi()
    operation = document["paths"][
        f"{SCENE_TOOL_ROOT}/element-batches/validate"
    ]["post"]
    reference = operation["requestBody"]["content"]["application/json"]["schema"][
        "$ref"
    ]
    schema_name = reference.rsplit("/", 1)[-1]
    batch_schema = document["components"]["schemas"][schema_name]
    element_reference = batch_schema["properties"]["elements"]["items"]["$ref"]
    element_schema = document["components"]["schemas"][
        element_reference.rsplit("/", 1)[-1]
    ]

    assert batch_schema["additionalProperties"] is False
    assert set(batch_schema["properties"]) == {
        "schema",
        "batch_id",
        "description",
        "elements",
    }
    assert element_schema["additionalProperties"] is False
    serialized = json.dumps(operation, sort_keys=True).lower()
    assert not {
        "command",
        "cwd",
        "env",
        "executable",
        "path",
    } & set(serialized.replace('"', " ").replace(":", " ").split())
