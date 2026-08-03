"""Contract tests for deterministic simulation package resolution."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from sim.catalog import CatalogError, CatalogResolver


REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = REPO_ROOT / "sim" / "scenarios" / "catalog" / "thunder_omni_contract" / "session.yaml"


def _resolver() -> CatalogResolver:
    return CatalogResolver.from_repository(REPO_ROOT)


def test_resolves_two_robot_session_into_one_physics_plan() -> None:
    resolved = _resolver().resolve(SESSION)

    assert resolved.session_lock["session_digest"] == resolved.session_digest
    assert resolved.physics_plan["schema"] == "lingtu.sim.physics-plan.v1"
    assert resolved.physics_plan["composition"]["model_kind"] == "single_mjmodel"
    assert [robot["instance_id"] for robot in resolved.physics_plan["robots"]] == [
        "thunder_01",
        "cart_01",
    ]
    assert all(robot["namespace"] for robot in resolved.physics_plan["robots"])
    assert all("body_count" not in robot for robot in resolved.physics_plan["robots"])


def test_resolution_is_byte_stable_and_excludes_run_allocation() -> None:
    resolver = _resolver()
    first = resolver.resolve(SESSION)
    second = resolver.resolve(SESSION)

    assert first.session_digest == second.session_digest
    assert first.lock_json == second.lock_json
    assert first.physics_json == second.physics_json
    assert "run_allocation" not in first.session_lock
    assert "ports" not in first.session_lock
    assert "dds_domain" not in first.session_lock


def test_robot_and_controller_are_separate_package_records() -> None:
    resolved = _resolver().resolve(SESSION)
    packages = resolved.session_lock["packages"]

    kinds = {(package["id"], package["kind"]) for package in packages}
    assert ("thunderv4", "robot") in kinds
    assert ("thunderv4_locomotion", "controller") in kinds
    thunder = next(package for package in packages if package["id"] == "thunderv4")
    assert "policy" not in thunder
    assert "robot_config" not in resolved.session_lock


def test_writes_modular_session_bundle(tmp_path: Path) -> None:
    resolved = _resolver().resolve(SESSION)
    bundle = resolved.write_bundle(tmp_path / "bundle")

    assert bundle == tmp_path / "bundle"
    assert (bundle / "session.lock.json").read_text(encoding="utf-8") == resolved.lock_json
    assert (bundle / "physics.plan.json").read_text(encoding="utf-8") == resolved.physics_json

    lock = json.loads((bundle / "session.lock.json").read_text(encoding="utf-8"))
    physics = json.loads((bundle / "physics.plan.json").read_text(encoding="utf-8"))
    assert lock["session_digest"] == physics["session_digest"]


def test_unknown_sensor_rig_is_rejected(tmp_path: Path) -> None:
    source = SESSION.read_text(encoding="utf-8")
    broken = source.replace("package: thunderv4@1.0.0", "package: thunderv4@1.0.0", 1)
    broken = broken.replace("sensor_rig: thunderv4_navigation@1.0.0", "sensor_rig: broken_rig@1.0.0")
    session = tmp_path / "broken.yaml"
    session.write_text(broken, encoding="utf-8")

    with pytest.raises(CatalogError, match="broken_rig"):
        _resolver().resolve(session)


def test_unknown_session_key_is_rejected(tmp_path: Path) -> None:
    source = SESSION.read_text(encoding="utf-8")
    session = tmp_path / "broken.yaml"
    session.write_text(source + "\nextra_key: forbidden\n", encoding="utf-8")

    with pytest.raises(CatalogError, match="unknown key.*extra_key"):
        _resolver().resolve(session)


def test_mujoco_compatibility_is_checked(tmp_path: Path) -> None:
    source = SESSION.read_text(encoding="utf-8").replace("mujoco_version: 3.10.0", "mujoco_version: 3.9.0")
    session = tmp_path / "incompatible.yaml"
    session.write_text(source, encoding="utf-8")

    with pytest.raises(CatalogError, match="requires MuJoCo 3.10.x"):
        _resolver().resolve(session)
