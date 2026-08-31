# ruff: noqa: S101
"""S3 contract tests for SessionIntent and SessionComposer."""

from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION_INTENT_SCHEMA = REPO_ROOT / "schemas" / "simulation" / "session-intent.v1.json"
SESSION_INTENT_V2_SCHEMA = REPO_ROOT / "schemas" / "simulation" / "session-intent.v2.json"


def _session() -> dict[str, Any]:
    return {
        "session_id": "contract_session",
        "mujoco_version": "3.10.0",
        "seed": 7,
        "world": "open_field@1.0.0",
        "robots": [
            {
                "instance_id": "robot_01",
                "package": "omni_cart@1.0.0",
                "spawn": {
                    "position_m": [0.0, 0.0, 0.0],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                },
            }
        ],
        "runtime": {
            "backend": "mujoco",
            "mode": "headless",
            "required_bindings": ["physics"],
        },
    }


def _intent() -> dict[str, Any]:
    return {"schema": "lingtu.sim.session-intent.v1", "session": _session()}


def _asdict(value: Any) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    if hasattr(value, "to_dict"):
        return value.to_dict()
    if hasattr(value, "__dict__"):
        return dict(value.__dict__)
    raise AssertionError(f"expected mapping-like value, got {type(value)!r}")


class RecordingResolver:
    def __init__(self) -> None:
        self.calls: list[Any] = []
        self.documents: list[dict[str, Any]] = []

    def resolve(self, session: Any) -> Any:
        self.calls.append(session)
        session_document = yaml.safe_load(Path(session).read_text(encoding="utf-8"))
        self.documents.append(session_document)

        class Resolved:
            session_id = session_document["session_id"]
            scenario_json = None

            def write_bundle(self, bundle_dir: Path) -> None:
                (bundle_dir / "physics.plan.json").write_text("{}", encoding="utf-8")
                (bundle_dir / "visual.plan.json").write_text("{}", encoding="utf-8")
                (bundle_dir / "sensor.plan.json").write_text("{}", encoding="utf-8")
                (bundle_dir / "control.plan.json").write_text("{}", encoding="utf-8")
                (bundle_dir / "transport.intent.json").write_text("{}", encoding="utf-8")

        return Resolved()


def _composer(resolver: RecordingResolver, artifact_root: Path) -> Any:
    from sim.catalog.composer import SessionComposer

    return SessionComposer(resolver, artifact_root=artifact_root)


def _compose(composer: Any, intent: Any, output_dir: Path) -> Any:
    if hasattr(composer, "compose"):
        return composer.compose(intent, output_dir=output_dir)
    return composer(intent)


def test_session_intent_schema_accepts_complete_session_and_optional_schema() -> None:
    jsonschema = pytest.importorskip("jsonschema")
    schema = json.loads(SESSION_INTENT_SCHEMA.read_text(encoding="utf-8"))
    payload = _intent()

    jsonschema.validate(payload, schema)
    with_schema = copy.deepcopy(payload)
    with_schema["session"]["schema"] = "lingtu.sim.session.v1"
    jsonschema.validate(with_schema, schema)

    invalid = copy.deepcopy(payload)
    invalid["overrides"] = {"runtime": {"backend": "unreal"}}
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(invalid, schema)


def test_composer_outputs_session_v1_applies_legal_overrides_and_is_byte_stable(tmp_path: Path) -> None:
    resolver = RecordingResolver()
    composer = _composer(resolver, tmp_path)
    intent = _intent()
    intent["overrides"] = {"seed": 42, "runtime": {"mode": "preview", "required_bindings": ["physics", "visual"]}}

    first = _asdict(_compose(composer, copy.deepcopy(intent), tmp_path / "first").to_dict())
    second = _asdict(_compose(composer, copy.deepcopy(intent), tmp_path / "second").to_dict())

    stable_fields = ("session_id", "session_spec")
    assert {field: first[field] for field in stable_fields} == {field: second[field] for field in stable_fields}
    assert json.dumps(first["session_spec"], sort_keys=True, separators=(",", ":"), allow_nan=False) == json.dumps(
        second["session_spec"], sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    assert first["session_spec"]["schema"] == "lingtu.sim.session.v1"
    assert first["session_spec"]["seed"] == 42
    assert first["session_spec"]["runtime"]["mode"] == "preview"
    assert first["session_spec"]["runtime"]["required_bindings"] == ["physics", "visual"]


def test_composer_preserves_payloads_and_emits_session_v2(tmp_path: Path) -> None:
    jsonschema = pytest.importorskip("jsonschema")
    intent = _intent()
    intent["schema"] = "lingtu.sim.session-intent.v2"
    intent["session"]["schema"] = "lingtu.sim.session.v2"
    intent["session"]["robots"][0]["package"] = "thunderv4@1.0.3"
    intent["session"]["robots"][0]["payloads"] = [
        {
            "instance_id": "rws_01",
            "package": "fictional_rws_01@1.0.0",
            "parent_frame": "payload_top",
            "extrinsic": {
                "position_m": [0.0, 0.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
        }
    ]
    schema = json.loads(SESSION_INTENT_V2_SCHEMA.read_text(encoding="utf-8"))
    jsonschema.validate(intent, schema)

    result = _asdict(
        _compose(_composer(RecordingResolver(), tmp_path), intent, tmp_path / "bundle").to_dict()
    )

    assert result["session_spec"]["schema"] == "lingtu.sim.session.v2"
    assert result["session_spec"]["robots"][0]["payloads"] == intent["session"]["robots"][0]["payloads"]


def test_composer_calls_the_supplied_catalog_resolver_once_with_generated_session_spec(tmp_path: Path) -> None:
    resolver = RecordingResolver()
    composer = _composer(resolver, tmp_path)
    intent = _intent()

    result = _asdict(_compose(composer, intent, tmp_path / "bundle").to_dict())

    assert len(resolver.calls) == 1
    assert Path(resolver.calls[0]).parent.name.startswith(".bundle.staging-")
    assert resolver.documents[0] == result["session_spec"]
    assert yaml.safe_load(Path(result["session_spec_path"]).read_text(encoding="utf-8")) == result["session_spec"]


@pytest.mark.parametrize(
    "override",
    [
        {"seed": "42"},
        {"runtime": {"mode": ""}},
        {"runtime": {"backend": "unreal"}},
        {"runtime": {"required_bindings": ["physics", "unknown"]}},
        {"runtime": {"required_bindings": ["physics", "physics"]}},
        {"runtime": {"required_bindings": ["visual"]}},
        {"unexpected": True},
    ],
)
def test_composer_rejects_illegal_overrides_with_structured_code(override: dict[str, Any], tmp_path: Path) -> None:
    intent = _intent()
    intent["overrides"] = override

    with pytest.raises(Exception) as exc_info:
        _compose(_composer(RecordingResolver(), tmp_path), intent, tmp_path / "bundle")

    code = getattr(exc_info.value, "code", "")
    assert getattr(code, "value", code) == "SESSION_INTENT_OVERRIDE_INVALID"
    assert not (tmp_path / "bundle").exists()


def test_composer_rejects_output_escape_before_writing(tmp_path: Path) -> None:
    artifact_root = tmp_path / "owned"
    outside = tmp_path / "outside"

    with pytest.raises(Exception) as exc_info:
        _compose(_composer(RecordingResolver(), artifact_root), _intent(), outside)

    code = getattr(exc_info.value, "code", "")
    assert getattr(code, "value", code) == "SIMCATALOG_PATH_TRAVERSAL"
    assert not outside.exists()


def test_composer_publishes_atomically_and_rejects_existing_target(tmp_path: Path) -> None:
    composer = _composer(RecordingResolver(), tmp_path)
    target = tmp_path / "bundle"
    _compose(composer, _intent(), target)

    with pytest.raises(Exception) as exc_info:
        _compose(composer, _intent(), target)

    code = getattr(exc_info.value, "code", "")
    assert getattr(code, "value", code) == "SIMCATALOG_ARTIFACT_CONFLICT"
    assert not list(tmp_path.glob(".bundle.staging-*"))


@pytest.mark.parametrize(
    "runtime",
    [
        {"backend": "mujoco", "mode": "headless", "required_bindings": ["physics", "physics"]},
        {"backend": "mujoco", "mode": "unreal", "required_bindings": ["visual"]},
        {"backend": "unreal", "mode": "unreal", "required_bindings": ["physics", "visual"]},
        {"backend": "mujoco", "mode": "headless", "required_bindings": ["physics", {"bad": "value"}]},
    ],
)
def test_direct_session_resolve_rejects_invalid_runtime_contract(
    tmp_path: Path,
    runtime: dict[str, Any],
) -> None:
    from sim.catalog import CatalogError, CatalogResolver

    session = _session()
    session["schema"] = "lingtu.sim.session.v1"
    session["runtime"] = runtime
    path = tmp_path / "session.yaml"
    path.write_text(json.dumps(session, sort_keys=True), encoding="utf-8")

    with pytest.raises(CatalogError):
        CatalogResolver.from_repository(REPO_ROOT).resolve(path)
