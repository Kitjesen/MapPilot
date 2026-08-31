# ruff: noqa: S101, S603, S607
"""Contract tests for the SimStudio package import service."""

from __future__ import annotations

import copy
import os
import struct
import subprocess
from pathlib import Path
from typing import Any

import pytest
from sim.catalog.management import SimCatalog
from sim.importers.world import WorldImporter
from tools.simstudio.service import package_service as package_service_module
from tools.simstudio.service.package_service import PackageImportService, PackageServiceError
from tools.simstudio.service.store import StudioStore


def _service(tmp_path: Path) -> PackageImportService:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()
    store = StudioStore(tmp_path / "store")
    return PackageImportService(
        catalog=SimCatalog.from_repository(repo_root),
        store=store,
        repo_root=repo_root,
    )


def _robot_source(service: PackageImportService, name: str = "robot-src") -> str:
    source = service.inbox_root / name
    source.mkdir(parents=True)
    (source / "LICENSE.txt").write_text("Test license\n", encoding="utf-8")
    (source / "robot.xml").write_text(
        """<mujoco model="studio_bot">
  <compiler angle="radian" autolimits="true"/>
  <worldbody>
    <body name="base_link">
      <joint name="floating_base_joint" type="free"/>
      <geom name="body_visual" type="box" size="0.2 0.1 0.05" rgba="0.2 0.4 0.6 1"/>
      <site name="imu" pos="0 0 0.1" size="0.001"/>
    </body>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )
    return name


def _robot_request(**overrides: object) -> dict[str, object]:
    request: dict[str, object] = {
        "schema": "lingtu.sim.robot-import-request.v1",
        "id": "studio_bot",
        "version": "1.0.0",
        "source_format": "mjcf",
        "source_model": "robot.xml",
        "units": {"length": "m", "angle": "radian"},
        "provenance": {
            "owner": "LingTu tests",
            "license": "Test-Only",
            "license_file": "LICENSE.txt",
            "source_uri": "file://studio-test",
        },
        "physics": {"attach_root": "base_link", "root_joint": "floating_base_joint"},
        "visual": {"binding": "RobotVisual:StudioBot"},
        "semantic": {"class": "test_robot"},
        "frames": [{"name": "base_link", "role": "body"}, {"name": "imu", "role": "sensor_mount"}],
        "interfaces": {"state": ["lingtu.sim.base-state.v1"], "command": ["lingtu.sim.base-velocity.v1"]},
        "defaults": {"controller": None, "sensor_rig": None},
        "declared_capabilities": {"locomotion": ["drive"], "sensor_mounts": ["imu"]},
    }
    request.update(overrides)
    return request


def _world_source(service: PackageImportService, name: str = "world-src") -> str:
    source = service.inbox_root / name
    source.mkdir(parents=True)
    (source / "LICENSE.txt").write_text("project owned test asset\n", encoding="utf-8")
    (source / "height.r16").write_bytes(struct.pack("<4H", 0, 16_384, 32_768, 65_535))
    return name


def _world_request(**overrides: Any) -> dict[str, Any]:
    request: dict[str, Any] = {
        "schema": "lingtu.sim.world-import-request.v1",
        "package": {"id": "studio_field", "version": "1.0.0", "description": "Imported test field"},
        "source": {
            "provenance": {
                "owner": "LingTu tests",
                "license": "LicenseRef-Test",
                "license_file": "LICENSE.txt",
                "source_uri": "file://studio-test",
                "third_party_assets": [],
            },
        },
        "units": {"length": "m", "up_axis": "Z", "handedness": "RH"},
        "heightmap": {
            "path": "height.r16",
            "width": 2,
            "height": 2,
            "extent_m": [2.0, 2.0],
            "elevation_min_m": -1.0,
            "elevation_max_m": 1.0,
        },
        "visual": {"binding": "WorldVisual:StudioField", "level": "/Game/RobotSim/Maps/StudioField"},
        "spawn": {
            "position_m": [-1.0, 1.0, -1.0],
            "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "height_tolerance_m": 1e-6,
        },
        "entities": [],
        "bounds": {"min_m": [-1.0, -1.0, -1.0], "max_m": [1.0, 1.0, 1.0]},
    }
    request.update(overrides)
    return request


def test_robot_import_success_crud_and_diagnostics_are_persisted(tmp_path: Path) -> None:
    """Persist a successful robot import and expose it through CRUD methods."""
    service = _service(tmp_path)
    job = service.create_import_job(kind="robot", request=_robot_request(), source_entry=_robot_source(service))

    assert job["status"] == "READY", job["payload"].get("diagnostics")
    assert job["payload"]["kind"] == "robot"
    assert job["payload"]["diagnostics"] == []
    assert job["payload"]["source_identity"]["kind"] == "directory"
    assert job["payload"]["result"]["package"]["ref"] == "studio_bot@1.0.0"
    assert service.get_import_job(job["id"]) == job
    assert service.list_import_jobs()[0]["id"] == job["id"]


def test_world_import_success_path_persists_ready_job(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Persist a successful world import with its staged package path."""
    monkeypatch.setattr(WorldImporter, "_compile_mjcf", lambda _self, _path: {"status": "passed", "backend": "test"})
    service = _service(tmp_path)

    job = service.create_import_job(kind="world", request=_world_request(), source_entry=_world_source(service))

    assert job["status"] == "READY"
    assert job["payload"]["kind"] == "world"
    assert job["payload"]["result"]["package"]["ref"] == "studio_field@1.0.0"
    draft = job["payload"]["draft"]
    assert draft["schema"] == "lingtu.sim.import-draft-reference.v1"
    assert draft["staging_ref"]
    assert draft["locations"]["package"] == "package"
    assert not any(
        isinstance(value, str) and Path(value).is_absolute()
        for value in [draft["staging_ref"], *draft["locations"].values()]
        if value is not None
    )


@pytest.mark.parametrize("entry", ["/absolute", "../escape", "a\\b", "a:b", ""])
def test_import_rejects_unsafe_inbox_source_entries(tmp_path: Path, entry: str) -> None:
    """Reject source entries that are not managed relative POSIX paths."""
    service = _service(tmp_path)
    with pytest.raises(PackageServiceError) as error:
        service.create_import_job(kind="robot", request=_robot_request(), source_entry=entry)
    assert error.value.to_dict()["code"] == "PACKAGE_UNSAFE_SOURCE"


def test_import_rejects_symlink_source(tmp_path: Path) -> None:
    """Reject symlinked inbox entries before an importer can read them."""
    service = _service(tmp_path)
    target = tmp_path / "outside"
    target.mkdir()
    link = service.inbox_root / "linked"
    try:
        link.symlink_to(target, target_is_directory=True)
    except (OSError, NotImplementedError):
        pytest.skip("symbolic links are unavailable in this Windows test environment")
    with pytest.raises(PackageServiceError) as error:
        service.create_import_job(kind="robot", request=_robot_request(), source_entry="linked")
    assert error.value.to_dict()["code"] == "PACKAGE_UNSAFE_SOURCE"


def test_import_rejects_reparse_inbox_root_before_source_identity(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Reject a reparse inbox root before any source file is hashed."""
    service = _service(tmp_path)
    _robot_source(service)
    original = StudioStore._is_reparse_point

    def fake_reparse(path: Path) -> bool:
        return Path(path) == service.inbox_root or original(path)

    def fail_file_records(_path: Path) -> tuple[()]:
        raise AssertionError("source identity must not be read through a reparse inbox")

    monkeypatch.setattr(StudioStore, "_is_reparse_point", staticmethod(fake_reparse))
    monkeypatch.setattr(package_service_module, "file_records", fail_file_records)

    with pytest.raises(PackageServiceError) as error:
        service.create_import_job(kind="robot", request=_robot_request(), source_entry="robot-src")

    assert error.value.to_dict()["code"] == "PACKAGE_UNSAFE_SOURCE"


@pytest.mark.skipif(os.name != "nt", reason="Windows junction regression test")
def test_import_rejects_windows_junction_inbox_root_before_source_identity(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Reject a real Windows inbox junction before reading its target."""
    repo_root = tmp_path / "repo"
    repo_root.mkdir()
    store = StudioStore(tmp_path / "store")
    outside = tmp_path / "outside"
    (outside / "robot-src").mkdir(parents=True)
    (outside / "robot-src" / "foreign.xml").write_text("<mujoco/>\n", encoding="utf-8")
    junction = tmp_path / "inbox-junction"
    created = subprocess.run(
        ["cmd", "/c", "mklink", "/J", str(junction), str(outside)],
        check=False,
        capture_output=True,
        text=True,
    )
    if created.returncode != 0:
        pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")
    service = PackageImportService(
        catalog=SimCatalog.from_repository(repo_root),
        store=store,
        repo_root=repo_root,
        inbox_root=junction,
    )

    def fail_file_records(_path: Path) -> tuple[()]:
        raise AssertionError("source identity must not be read through a junction")

    monkeypatch.setattr(package_service_module, "file_records", fail_file_records)
    try:
        with pytest.raises(PackageServiceError) as error:
            service.create_import_job(kind="robot", request=_robot_request(), source_entry="robot-src")
        assert error.value.to_dict()["code"] == "PACKAGE_UNSAFE_SOURCE"
    finally:
        if junction.exists():
            junction.rmdir()


@pytest.mark.skipif(os.name != "nt", reason="Windows junction regression test")
def test_import_rejects_windows_junction_source_component_before_source_identity(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Reject a junction below the inbox before reading its target."""
    service = _service(tmp_path)
    service.inbox_root.mkdir(parents=True, exist_ok=True)
    outside = tmp_path / "outside"
    (outside / "robot-src").mkdir(parents=True)
    (outside / "robot-src" / "foreign.xml").write_text("<mujoco/>\n", encoding="utf-8")
    junction = service.inbox_root / "linked"
    created = subprocess.run(
        ["cmd", "/c", "mklink", "/J", str(junction), str(outside)],
        check=False,
        capture_output=True,
        text=True,
    )
    if created.returncode != 0:
        pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")

    def fail_file_records(_path: Path) -> tuple[()]:
        raise AssertionError("source identity must not be read through a junction component")

    monkeypatch.setattr(package_service_module, "file_records", fail_file_records)
    try:
        with pytest.raises(PackageServiceError) as error:
            service.create_import_job(
                kind="robot",
                request=_robot_request(),
                source_entry="linked/robot-src",
            )
        assert error.value.to_dict()["code"] == "PACKAGE_UNSAFE_SOURCE"
    finally:
        if junction.exists():
            junction.rmdir()


def test_unknown_kind_fails_closed(tmp_path: Path) -> None:
    """Reject unsupported package kinds consistently."""
    service = _service(tmp_path)
    with pytest.raises(PackageServiceError) as error:
        service.create_import_job(kind="controller", request={}, source_entry="missing")
    assert error.value.to_dict()["details"]["supported"] == ["robot", "world"]
    with pytest.raises(PackageServiceError):
        service.list_packages(kind="not_a_package_kind")


def test_failed_import_persists_structured_diagnostics(tmp_path: Path) -> None:
    """Persist importer failures as structured diagnostics on the job."""
    service = _service(tmp_path)
    source_entry = _robot_source(service)
    invalid = copy.deepcopy(_robot_request(source_model="missing.xml"))

    job = service.create_import_job(kind="robot", request=invalid, source_entry=source_entry)

    assert job["status"] == "FAILED"
    assert job["payload"]["diagnostics"][0]["code"].startswith("SIMIMPORT_")
    assert service.get_import_job(job["id"])["payload"]["diagnostics"] == job["payload"]["diagnostics"]


def test_ready_promote_idempotency_and_catalog_refresh(tmp_path: Path) -> None:
    """Promote once, replay safely, and refresh catalog queries."""
    service = _service(tmp_path)
    job = service.create_import_job(kind="robot", request=_robot_request(), source_entry=_robot_source(service))

    fresh_service = PackageImportService(
        catalog=SimCatalog.from_repository(service.repo_root),
        store=service.store,
        repo_root=service.repo_root,
    )
    promoted = fresh_service.promote_import_job(job["id"], idempotency_key="promote-robot")
    repeated = fresh_service.promote_import_job(job["id"], idempotency_key="promote-robot")

    assert promoted["status"] == "PROMOTED"
    assert repeated["id"] == promoted["id"]
    assert promoted["payload"]["promotion"]["package"]["ref"] == "studio_bot@1.0.0"
    assert promoted["payload"]["promotion"]["package_root"] == "sim/robots/studio_bot"
    assert promoted["payload"]["promotion"]["qualification_path"] == (
        "sim/qualifications/robot/studio_bot/1.0.0.qualification.json"
    )
    listed = fresh_service.list_packages(kind="robot")
    assert [item["package"]["ref"] for item in listed["packages"]] == ["studio_bot@1.0.0"]
    assert fresh_service.detail_package("studio_bot@1.0.0", kind="robot")["package"]["id"] == "studio_bot"
    assert fresh_service.validate_package("studio_bot@1.0.0", kind="robot")["catalog_valid"] is True
    assert fresh_service.qualification("studio_bot@1.0.0", kind="robot")["state"] == "qualified"


def test_promote_requires_ready_and_errors_are_structured(tmp_path: Path) -> None:
    """Refuse promotion of failed jobs with a stable service error."""
    service = _service(tmp_path)
    job = service.create_import_job(kind="robot", request=_robot_request(source_model="missing.xml"), source_entry=_robot_source(service))

    with pytest.raises(PackageServiceError) as error:
        service.promote_import_job(job["id"])

    payload = error.value.to_dict()
    assert payload["code"] == "PACKAGE_IMPORT_NOT_READY"
    assert payload["details"]["status"] == "FAILED"
