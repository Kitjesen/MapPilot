# ruff: noqa: S101
"""S3 contract tests for simulation catalog management queries."""

from __future__ import annotations

import copy
import json
import subprocess
from pathlib import Path
from typing import Any

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
QUALIFICATION_SCHEMA = REPO_ROOT / "schemas" / "simulation" / "qualification.v1.json"


def _asdict(value: Any) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    if hasattr(value, "to_dict"):
        return value.to_dict()
    if hasattr(value, "__dict__"):
        return dict(value.__dict__)
    raise AssertionError(f"expected mapping-like value, got {type(value)!r}")


def _diagnostic_code(exc: BaseException) -> str:
    code = getattr(exc, "code", None)
    return getattr(code, "value", code) or ""


def _catalog(root: Path) -> Any:
    from sim.catalog.management import SimCatalog

    if hasattr(SimCatalog, "from_repository"):
        return SimCatalog.from_repository(root)
    return SimCatalog(root)


def _call(target: Any, *names: str, **kwargs: Any) -> Any:
    if "id" in kwargs and "version" in kwargs and "reference" not in kwargs:
        kwargs["reference"] = f"{kwargs.pop('id')}@{kwargs.pop('version')}"
    for name in names:
        method = getattr(target, name, None)
        if method is not None:
            return method(**kwargs)
    raise AssertionError(f"{type(target).__name__} is missing one of {names!r}")


def _world_manifest(package_dir: Path, package_id: str = "field", version: str = "1.0.0") -> Path:
    package_dir.mkdir(parents=True, exist_ok=True)
    (package_dir / "world.xml").write_text(
        '<mujoco><option timestep="0.002" integrator="RK4" solver="Newton" iterations="100" '
        'gravity="0 0 -9.81"/><worldbody/></mujoco>\n',
        encoding="utf-8",
    )
    (package_dir / "provenance.json").write_text('{"owner":"test"}\n', encoding="utf-8")
    manifest = package_dir / "world.package.yaml"
    manifest.write_text(
        f"""schema: lingtu.sim.world-package.v1
id: {package_id}
version: {version}
kind: world
physics:
  mjcf: world.xml
  global_policy:
    timestep_s: 0.002
    integrator: rk4
    solver: newton
    iterations: 100
    gravity_mps2: [0.0, 0.0, -9.81]
visual:
  binding: WorldVisual:Field
  level: /Game/RobotSim/Maps/Field
entities: []
""",
        encoding="utf-8",
    )
    return manifest


def _scenario_manifest(package_dir: Path, world: str = "field@1.0.0") -> Path:
    package_dir.mkdir(parents=True, exist_ok=True)
    manifest = package_dir / "scenario.package.yaml"
    manifest.write_text(
        f"""schema: lingtu.sim.scenario-package.v1
id: patrol
version: 1.0.0
kind: scenario
world: {world}
entities:
  - entity_id: pedestrian_01
    entity_type: pedestrian
    authority: scenario
    initial_transform:
      position_m: [0.0, 0.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
    physics_proxy: kinematic
    semantic_class: person
""",
        encoding="utf-8",
    )
    return manifest


def _qualification(root: Path) -> Path:
    path = root / "sim" / "qualifications" / "world" / "field" / "1.0.0.qualification.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    evidence_path = path.parent / "evidence" / "schema.json"
    evidence_path.parent.mkdir(parents=True, exist_ok=True)
    evidence_path.write_text('{"ok":true}\n', encoding="utf-8")
    path.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.qualification-record.v1",
                "package": {
                    "kind": "world",
                    "id": "field",
                    "version": "1.0.0",
                },
                "qualified_capabilities": {},
                "provenance": {"path": "provenance.json"},
                "checks": [
                    {
                        "id": "schema",
                        "status": "passed",
                        "evidence": [{"path": "evidence/schema.json", "sha256": "3" * 64}],
                    }
                ],
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return path


def test_qualification_record_schema_is_strict_and_accepts_frozen_shape() -> None:
    jsonschema = pytest.importorskip("jsonschema")
    schema = json.loads(QUALIFICATION_SCHEMA.read_text(encoding="utf-8"))
    payload = {
        "schema": "lingtu.sim.qualification-record.v1",
        "package": {
            "kind": "robot",
            "id": "thunderv4",
            "version": "1.0.0",
        },
        "qualified_capabilities": {"locomotion": ["walk"]},
        "provenance": {"path": "provenance.json"},
        "checks": [
            {
                "id": "contract",
                "status": "passed",
                "evidence": [{"path": "evidence/run.json", "sha256": "3" * 64}],
            }
        ],
    }

    jsonschema.validate(payload, schema)
    invalid = copy.deepcopy(payload)
    invalid["extra"] = True
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(invalid, schema)


def test_list_inspect_dependencies_and_qualification_are_read_only(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def fail_popen(*args: Any, **kwargs: Any) -> None:
        raise AssertionError(f"catalog query unexpectedly started a process: {args!r} {kwargs!r}")

    monkeypatch.setattr(subprocess, "Popen", fail_popen)
    root = tmp_path
    _world_manifest(root / "sim" / "packages" / "worlds" / "field")
    _scenario_manifest(root / "sim" / "packages" / "scenarios" / "patrol")
    _qualification(root)

    catalog = _catalog(root)
    listed = _asdict(_call(catalog, "list_packages", "list", kind="world"))
    assert listed["schema"] == "lingtu.sim.catalog-list.v1"
    assert listed["packages"][0]["package"] == {
        "kind": "world",
        "id": "field",
        "version": "1.0.0",
        "ref": "field@1.0.0",
    }
    assert listed["packages"][0]["manifest"] == {
        "path": "sim/packages/worlds/field/world.package.yaml"
    }

    inspected = _asdict(_call(catalog, "inspect_package", "inspect", kind="world", id="field", version="1.0.0"))
    assert inspected["package"]["ref"] == "field@1.0.0"
    assert "fingerprint" not in inspected

    dependencies = _asdict(
        _call(catalog, "dependencies", "dependency_graph", kind="scenario", id="patrol", version="1.0.0")
    )
    assert dependencies["root"]["ref"] == "patrol@1.0.0"
    assert dependencies["edges"][0]["target"]["ref"] == "field@1.0.0"

    qualification = _asdict(
        _call(catalog, "qualification", "qualification_status", kind="world", id="field", version="1.0.0")
    )
    assert qualification["state"] == "qualified"
    assert qualification["qualified_capabilities"] == {}
    assert qualification["diagnostics"] == []


def test_duplicate_identity_path_traversal_missing_dependency_and_stale_qualification_are_structured(
    tmp_path: Path,
) -> None:
    duplicate_root = tmp_path / "duplicate"
    _world_manifest(duplicate_root / "sim" / "packages" / "worlds" / "field_a")
    _world_manifest(duplicate_root / "sim" / "packages" / "worlds" / "field_b")
    with pytest.raises(Exception) as duplicate:
        _catalog(duplicate_root)
    assert _diagnostic_code(duplicate.value) == "SIMCATALOG_DUPLICATE_IDENTITY"

    traversal_root = tmp_path / "traversal"
    manifest = _world_manifest(traversal_root / "sim" / "packages" / "worlds" / "field")
    manifest.write_text(
        manifest.read_text(encoding="utf-8").replace("mjcf: world.xml", "mjcf: C:/outside.xml"), encoding="utf-8"
    )
    with pytest.raises(Exception) as traversal:
        _catalog(traversal_root)
    assert _diagnostic_code(traversal.value) == "SIMCATALOG_PATH_TRAVERSAL"

    missing_root = tmp_path / "missing"
    _scenario_manifest(missing_root / "sim" / "packages" / "scenarios" / "patrol", world="absent@9.9.9")
    with pytest.raises(Exception) as missing:
        _call(_catalog(missing_root), "dependencies", "dependency_graph", kind="scenario", id="patrol", version="1.0.0")
    assert _diagnostic_code(missing.value) == "SIMCATALOG_DEPENDENCY_MISSING"

    stale_root = tmp_path / "stale"
    _world_manifest(stale_root / "sim" / "packages" / "worlds" / "field")
    report_path = _qualification(stale_root)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["package"]["version"] = "9.9.9"
    report_path.write_text(json.dumps(report, sort_keys=True), encoding="utf-8")
    invalid_status = _call(
        _catalog(stale_root), "qualification", "qualification_status", kind="world", id="field", version="1.0.0"
    )
    assert invalid_status["state"] == "invalid"
    assert invalid_status["diagnostics"][0]["code"] == "SIMCATALOG_QUALIFICATION_INVALID"


def test_package_artifact_content_is_not_fingerprinted_by_qualification(tmp_path: Path) -> None:
    manifest = _world_manifest(tmp_path / "sim" / "packages" / "worlds" / "field")
    _qualification(tmp_path)
    catalog = _catalog(tmp_path)

    (manifest.parent / "world.xml").write_text("<mujoco><worldbody><geom/></worldbody></mujoco>\n", encoding="utf-8")

    status = catalog.qualification("field@1.0.0", kind="world")

    assert status["state"] == "qualified"


def test_external_runtime_artifact_content_is_not_fingerprinted_by_qualification(tmp_path: Path) -> None:
    manifest = _world_manifest(tmp_path / "sim" / "packages" / "worlds" / "field")
    external_model = tmp_path / "sim" / "worlds" / "field" / "world.xml"
    external_model.parent.mkdir(parents=True, exist_ok=True)
    external_model.write_bytes((manifest.parent / "world.xml").read_bytes())
    manifest.write_text(
        manifest.read_text(encoding="utf-8").replace(
            "mjcf: world.xml",
            "mjcf: ../../../worlds/field/world.xml",
        ),
        encoding="utf-8",
    )
    _qualification(tmp_path)
    catalog = _catalog(tmp_path)

    external_model.write_text(
        '<mujoco><option timestep="0.002" integrator="RK4" solver="Newton" iterations="100" '
        'gravity="0 0 -9.81"/><worldbody><geom type="plane" size="1 1 0.1"/></worldbody></mujoco>\n',
        encoding="utf-8",
    )

    status = catalog.qualification("field@1.0.0", kind="world")

    assert status["state"] == "qualified"


def test_qualification_provenance_path_must_exist_in_package(tmp_path: Path) -> None:
    _world_manifest(tmp_path / "sim" / "packages" / "worlds" / "field")
    report_path = _qualification(tmp_path)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["provenance"]["path"] = "missing.json"
    report_path.write_text(json.dumps(report, sort_keys=True), encoding="utf-8")

    status = _catalog(tmp_path).qualification("field@1.0.0", kind="world")

    assert status["state"] == "invalid"
    assert status["diagnostics"][0]["code"] == "SIMCATALOG_QUALIFICATION_INVALID"


@pytest.mark.parametrize("field", ["provenance", "evidence"])
def test_qualification_ads_paths_are_rejected_as_path_traversal(tmp_path: Path, field: str) -> None:
    _world_manifest(tmp_path / "sim" / "packages" / "worlds" / "field")
    report_path = _qualification(tmp_path)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if field == "provenance":
        report["provenance"]["path"] = "provenance.json:ads"
        expected_context = ".provenance.path"
    else:
        report["checks"][0]["evidence"][0]["path"] = "evidence/schema.json:ads"
        expected_context = ".checks[0].evidence[0].path"
    report_path.write_text(json.dumps(report, sort_keys=True), encoding="utf-8")

    status = _catalog(tmp_path).qualification("field@1.0.0", kind="world")

    assert status["state"] == "invalid"
    diagnostic = status["diagnostics"][0]
    assert diagnostic["code"] == "SIMCATALOG_PATH_TRAVERSAL"
    assert expected_context in diagnostic["context"]


def test_missing_qualification_reports_unverified(tmp_path: Path) -> None:
    _world_manifest(tmp_path / "sim" / "packages" / "worlds" / "field")

    status = _asdict(
        _call(_catalog(tmp_path), "qualification", "qualification_status", kind="world", id="field", version="1.0.0")
    )

    assert status["state"] == "unverified"
    assert status["qualified_capabilities"] == {}


@pytest.mark.parametrize("failure", ["missing", "absent_evidence"])
def test_passed_qualification_requires_verified_evidence(tmp_path: Path, failure: str) -> None:
    _world_manifest(tmp_path / "sim" / "packages" / "worlds" / "field")
    report_path = _qualification(tmp_path)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if failure == "missing":
        (report_path.parent / "evidence" / "schema.json").unlink()
    else:
        report["checks"][0].pop("evidence")
        report_path.write_text(json.dumps(report, sort_keys=True), encoding="utf-8")

    status = _catalog(tmp_path).qualification("field@1.0.0", kind="world")

    assert status["state"] == "invalid"
    assert status["diagnostics"][0]["code"] == "SIMCATALOG_QUALIFICATION_INVALID"
