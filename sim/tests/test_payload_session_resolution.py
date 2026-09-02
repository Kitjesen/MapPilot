# ruff: noqa: S101
"""PayloadPackage resolution contracts for payload-aware v2 sessions."""

from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest
import yaml

from sim.catalog import CatalogError, CatalogResolver

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "thunderv4_rws01_preview"
    / "session.yaml"
)


def _resolver() -> CatalogResolver:
    return CatalogResolver.from_repository(REPO_ROOT)


def _write_session(tmp_path: Path, document: dict) -> Path:
    path = tmp_path / "session.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def test_payload_v2_session_resolves_into_physics_and_visual_plans() -> None:
    resolver = _resolver()

    first = resolver.resolve(SESSION)
    second = resolver.resolve(SESSION)

    assert first.session_id == second.session_id
    assert first.physics_json == second.physics_json
    assert first.visual_json == second.visual_json
    assert first.session["session_id"] == "thunderv4_rws01_preview"
    assert first.physics_plan["schema"] == "lingtu.sim.physics-plan.v2"
    assert first.visual_plan["schema"] == "lingtu.sim.visual-plan.v2"

    physics_payload = first.physics_plan["robots"][0]["payloads"][0]
    assert physics_payload["instance_id"] == "rws_01"
    assert physics_payload["namespace"] == "rws_01"
    assert physics_payload["robot_instance_id"] == "thunder_01"
    assert physics_payload["parent_frame"] == "payload_top"
    assert physics_payload["parent_body"] == "base_link"
    assert physics_payload["mount_transform"] == {
        "position_m": [0.0, 0.0, 0.14],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
    assert physics_payload["model"] == {
        "mjcf": (
            "sim/packages/payloads/fictional_rws_01/1.0.0/"
            "mjcf/fictional_rws_01.xml"
        ),
        "attach_root": "payload_base",
    }

    visual_payload = first.visual_plan["robots"][0]["payloads"][0]
    assert visual_payload["instance_id"] == "rws_01"
    assert visual_payload["robot_instance_id"] == "thunder_01"
    assert visual_payload["parent_frame"] == "payload_top"
    assert visual_payload["binding"] == "PayloadVisual:FictionalRWS01"
    assert visual_payload["authority"] == "mujoco"
    assert visual_payload["ue_collision"] == "disabled"
    assert visual_payload["projection"] == {
        "schema": "lingtu.sim.payload-visual-projection.v1",
        "path": (
            "sim/packages/payloads/fictional_rws_01/1.0.0/"
            "visual/payload.visual-projection.json"
        ),
    }


def test_payload_changes_update_the_resolved_physics_plan(tmp_path: Path) -> None:
    document = yaml.safe_load(SESSION.read_text(encoding="utf-8"))
    first_path = _write_session(tmp_path / "first", document)
    changed = copy.deepcopy(document)
    changed["robots"][0]["payloads"][0]["extrinsic"] = {
        "position_m": [0.01, 0.0, 0.0],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
    second_path = _write_session(tmp_path / "second", changed)

    first = _resolver().resolve(first_path)
    second = _resolver().resolve(second_path)

    assert first.session_id == second.session_id == "thunderv4_rws01_preview"
    assert (
        first.physics_plan["robots"][0]["payloads"][0]["mount_transform"]
        != second.physics_plan["robots"][0]["payloads"][0]["mount_transform"]
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ({"parent_frame": "missing_mount"}, "unknown parent_frame"),
        ({"parent_frame": "front_camera"}, "requires parent role"),
    ],
)
def test_payload_mount_validation_fails_closed(
    tmp_path: Path,
    mutation: dict[str, object],
    message: str,
) -> None:
    document = yaml.safe_load(SESSION.read_text(encoding="utf-8"))
    document["robots"][0]["payloads"][0].update(mutation)

    with pytest.raises(CatalogError, match=message):
        _resolver().resolve(_write_session(tmp_path, document))


def test_payload_instance_ids_are_session_unique(tmp_path: Path) -> None:
    document = yaml.safe_load(SESSION.read_text(encoding="utf-8"))
    duplicate = copy.deepcopy(document["robots"][0]["payloads"][0])
    document["robots"][0]["payloads"].append(duplicate)

    with pytest.raises(CatalogError, match="duplicate payload instance_id"):
        _resolver().resolve(_write_session(tmp_path, document))


def test_payload_v2_artifacts_satisfy_their_strict_schemas() -> None:
    from sim.tests.test_sim_plan_schemas import _validate

    resolved = _resolver().resolve(SESSION)
    for filename, document in (
        ("physics-plan.v2.json", resolved.physics_plan),
        ("visual-plan.v2.json", resolved.visual_plan),
    ):
        schema = json.loads(
            (REPO_ROOT / "sim" / "contracts" / "schemas" / filename).read_text(encoding="utf-8")
        )
        _validate(document, schema, schema)
